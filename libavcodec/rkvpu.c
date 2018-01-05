/*
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include <assert.h>
#include <string.h>

#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <drm_fourcc.h>
#include <xf86drm.h>
#include <drm/rockchip_drm.h>
#include <video/rk_vpu_service.h>

#include "libavutil/common.h"
#include "libavutil/log.h"
#include "libavutil/time.h"

#include "avcodec.h"
#include "rkvpu.h"

#define VPU_DEVICE_NAME "/dev/vpu-service"
#define VPU_CLIENT_TYPE_DEC 0x1

int ff_rkvpu_set_hw_regs(AVCodecContext *avctx, void *hw_regs, int nb_regs)
{
    RKVPUContext *rkctx = avctx->internal->hwaccel_priv_data;
    struct vpu_request req = {0};
    int ret;

    av_log(avctx, AV_LOG_DEBUG, "%s: avctx=%p hw_regs=%p nb_regs=%d\n", __func__, avctx, hw_regs, nb_regs);

    req.req = hw_regs;
    req.size = nb_regs * sizeof(uint32_t);

    ret = ioctl(rkctx->fd, VPU_IOC_SET_REG, &req);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: ioctl VPU_IOC_SET_REG failed, ret=%d errno=%d\n", __func__, ret, errno);
        return AVERROR(EINVAL);
    }

    return 0;
}

int ff_rkvpu_get_hw_regs(AVCodecContext *avctx, void *hw_regs, int nb_regs)
{
    RKVPUContext *rkctx = avctx->internal->hwaccel_priv_data;
    struct vpu_request req = {0};
    int ret;

    av_log(avctx, AV_LOG_DEBUG, "%s: avctx=%p hw_regs=%p nb_regs=%d\n", __func__, avctx, hw_regs, nb_regs);

    req.req = hw_regs;
    req.size = nb_regs * sizeof(uint32_t);

    ret = ioctl(rkctx->fd, VPU_IOC_GET_REG, &req);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: ioctl VPU_IOC_GET_REG failed, ret=%d errno=%d\n", __func__, ret, errno);
        return AVERROR(EINVAL);
    }

    return 0;
}

int ff_rkvpu_gem_mmap(RKGEMDescriptor *gem)
{
    struct drm_rockchip_gem_map_off gem_map = {0};
    void *addr;
    int ret;

    av_log(NULL, AV_LOG_DEBUG, "%s: gem=%p handle=%u size=%lu fd=%d offset=%lu addr=%p\n", __func__, gem, gem->handle, gem->size, gem->fd, gem->offset, gem->addr);

    if (gem->addr)
        return 0;

    gem_map.handle = gem->handle;
    ret = drmIoctl(gem->hwctx->fd, DRM_IOCTL_ROCKCHIP_GEM_MAP_OFFSET, &gem_map);
    if (ret) {
        av_log(NULL, AV_LOG_ERROR, "%s: ioctl DRM_IOCTL_ROCKCHIP_GEM_MAP_OFFSET failed, ret=%d errno=%d\n", __func__, ret, errno);
        return AVERROR(EINVAL);
    }
    gem->offset = gem_map.offset;

    addr = mmap(NULL, gem->size, PROT_READ | PROT_WRITE, MAP_SHARED, gem->hwctx->fd, gem_map.offset);
    if (addr == MAP_FAILED) {
        av_log(NULL, AV_LOG_ERROR, "%s: mmap failed, errno=%d\n", __func__, errno);
        return AVERROR(EINVAL);
    }
    gem->addr = addr;

    return 0;
}

int ff_rkvpu_gem_munmap(RKGEMDescriptor *gem)
{
    int ret;

    av_log(NULL, AV_LOG_DEBUG, "%s: gem=%p handle=%u size=%lu fd=%d offset=%lu addr=%p\n", __func__, gem, gem->handle, gem->size, gem->fd, gem->offset, gem->addr);

    if (!gem->addr)
        return 0;

    ret = munmap(gem->addr, gem->size);
    if (ret) {
        av_log(NULL, AV_LOG_ERROR, "%s: munmap failed, ret=%d errno=%d\n", __func__, ret, errno);
        return AVERROR(EINVAL);
    }

    return 0;
}

void ff_rkvpu_gem_free(RKGEMDescriptor *gem)
{
    struct drm_gem_close gem_close = {0};
    int ret;

    if (!gem)
        return;

    av_log(NULL, AV_LOG_DEBUG, "%s: gem=%p handle=%u size=%lu fd=%d offset=%lu addr=%p\n", __func__, gem, gem->handle, gem->size, gem->fd, gem->offset, gem->addr);

    if (gem->addr)
        ff_rkvpu_gem_munmap(gem);

    if (gem->handle) {
        gem_close.handle = gem->handle;
        ret = drmIoctl(gem->hwctx->fd, DRM_IOCTL_GEM_CLOSE, &gem_close);
        if (ret)
            av_log(NULL, AV_LOG_ERROR, "%s: ioctl DRM_IOCTL_GEM_CLOSE failed, ret=%d errno=%d\n", __func__, ret, errno);
    }

    av_free(gem);
}

RKGEMDescriptor *ff_rkvpu_gem_alloc(AVDRMDeviceContext *hwctx, uint64_t size)
{
    RKGEMDescriptor *gem = NULL;
    struct drm_rockchip_gem_create gem_create = {0};
    struct drm_prime_handle prime_handle = {0};
    int ret;

    av_log(NULL, AV_LOG_DEBUG, "%s: hwctx=%p fd=%d size=%lu\n", __func__, hwctx, hwctx->fd, size);

    gem = av_mallocz(sizeof(RKGEMDescriptor));
    if (!gem)
        return NULL;

    gem->hwctx = hwctx;

    gem_create.size = size;
    ret = drmIoctl(gem->hwctx->fd, DRM_IOCTL_ROCKCHIP_GEM_CREATE, &gem_create);
    if (ret) {
        av_log(NULL, AV_LOG_ERROR, "%s: ioctl DRM_IOCTL_ROCKCHIP_GEM_CREATE failed, ret=%d errno=%d\n", __func__, ret, errno);
        ff_rkvpu_gem_free(gem);
        return NULL;
    }

    gem->handle = gem_create.handle;
    gem->size = gem_create.size;

    prime_handle.handle = gem->handle;
    ret = drmIoctl(gem->hwctx->fd, DRM_IOCTL_PRIME_HANDLE_TO_FD, &prime_handle);
    if (ret) {
        av_log(NULL, AV_LOG_ERROR, "%s: ioctl DRM_IOCTL_PRIME_HANDLE_TO_FD failed, ret=%d errno=%d\n", __func__, ret, errno);
        ff_rkvpu_gem_free(gem);
        return NULL;
    }

    gem->fd = prime_handle.fd;

    return gem;
}

static void rkvpu_buffer_free(void *opaque, uint8_t *data)
{
    RKGEMDescriptor *gem = (RKGEMDescriptor*)data;

    av_log(NULL, AV_LOG_DEBUG, "%s: gem=%p handle=%u size=%lu fd=%d offset=%lu addr=%p\n", __func__, gem, gem->handle, gem->size, gem->fd, gem->offset, gem->addr);

    ff_rkvpu_gem_free(gem);
}

static AVBufferRef *rkvpu_buffer_alloc(void *opaque, int size)
{
    AVCodecContext *avctx = opaque;
    AVHWFramesContext *hwfc = (AVHWFramesContext*)avctx->hw_frames_ctx->data;
    AVDRMDeviceContext *hwctx = hwfc->device_ctx->hwctx;
    RKGEMDescriptor *gem = NULL;
    AVBufferRef *ref = NULL;
    uint32_t width, height;

    av_log(NULL, AV_LOG_DEBUG, "%s: avctx=%p hwfc=%p width=%d height=%d hwctx=%p fd=%d\n", __func__, avctx, hwfc, hwfc->width, hwfc->height, hwctx, hwctx->fd);

    // TODO: use correct format and size
    width = FFALIGN(hwfc->width, 16);
    height = FFALIGN(hwfc->height, 16);

    gem = ff_rkvpu_gem_alloc(hwctx, width * height * 3 / 2);
    if (!gem)
        return NULL;

    ref = av_buffer_create((uint8_t*)gem, sizeof(RKGEMDescriptor), rkvpu_buffer_free, NULL, 0);
    if (!ref) {
        ff_rkvpu_gem_free(gem);
        return NULL;
    }

    gem->format = DRM_FORMAT_NV12;
    gem->width = width;
    gem->height = height;

    return ref;
}

static void rkvpu_pool_free(void *opaque)
{
    av_log(NULL, AV_LOG_DEBUG, "%s: opaque=%p\n", __func__, opaque);
}

int ff_rkvpu_decode_init(AVCodecContext *avctx)
{
    RKVPUContext *rkctx = avctx->internal->hwaccel_priv_data;
    int ret;

    av_log(avctx, AV_LOG_DEBUG, "%s: avctx=%p rkctx=%p hw_frames_ctx=%p\n", __func__, avctx, rkctx, avctx->hw_frames_ctx);

    rkctx->fd = open(VPU_DEVICE_NAME, O_RDWR);
    if (rkctx->fd < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: opening %s failed, errno=%d\n", __func__, VPU_DEVICE_NAME, errno);
        return AVERROR(EINVAL);
    }

    ret = ioctl(rkctx->fd, VPU_IOC_SET_CLIENT_TYPE, VPU_CLIENT_TYPE_DEC);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: ioctl VPU_IOC_SET_CLIENT_TYPE failed, ret=%d errno=%d\n", __func__, ret, errno);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    rkctx->pool = av_buffer_pool_init2(sizeof(RKGEMDescriptor), avctx, rkvpu_buffer_alloc, rkvpu_pool_free);
    if (!rkctx->pool) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    return 0;

fail:
    ff_rkvpu_decode_uninit(avctx);
    return ret;
}

int ff_rkvpu_decode_uninit(AVCodecContext *avctx)
{
    RKVPUContext *rkctx = avctx->internal->hwaccel_priv_data;

    av_log(avctx, AV_LOG_DEBUG, "%s: avctx=%p rkctx=%p fd=%d pool=%p bitstream=%p qp_table=%p hw_regs=%p\n", __func__, avctx, rkctx, rkctx->fd, rkctx->pool, rkctx->bitstream, rkctx->qp_table, rkctx->hw_regs);

    av_freep(&rkctx->hw_regs);

    ff_rkvpu_gem_free(rkctx->qp_table);
    ff_rkvpu_gem_free(rkctx->bitstream);

    av_buffer_pool_uninit(&rkctx->pool);

    close(rkctx->fd);

    return 0;
}
