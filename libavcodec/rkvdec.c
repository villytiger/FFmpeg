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
#include "rkvdec.h"

#define VPU_DEVICE_NAME "/dev/rkvdec"

enum VPU_CLIENT_TYPE {
    VPU_ENC     = 0x0,
    VPU_DEC     = 0x1,
    VPU_PP      = 0x2,
    VPU_DEC_PP  = 0x3,
};

static void rkvdec_buffer_free(void *opaque, uint8_t *data)
{
    AVHWFramesContext *hwfc = opaque;
    AVDRMDeviceContext *hwctx = hwfc->device_ctx->hwctx;
    RKGEMDescriptor *desc = (RKGEMDescriptor*)data;
    struct drm_gem_close gem_close = {0};
    int ret;

    av_log(NULL, AV_LOG_DEBUG, "%s: hwfc=%p hwctx=%p fd=%d desc=%p handle=%u size=%llu prime_fd=%d\n", __func__, hwfc, hwctx, hwctx->fd, desc, desc->handle, desc->size, desc->prime_fd);

    gem_close.handle = desc->handle;
    if (gem_close.handle) {
        ret = drmIoctl(hwctx->fd, DRM_IOCTL_GEM_CLOSE, &gem_close);
        if (ret)
            av_log(NULL, AV_LOG_ERROR, "%s: ioctl DRM_IOCTL_GEM_CLOSE failed, ret=%d errno=%d\n", __func__, ret, errno);
    }

    av_free(desc);
}

static AVBufferRef *rkvdec_buffer_alloc(void *opaque, int size)
{
    AVCodecContext *avctx = opaque;
    AVHWFramesContext *hwfc = (AVHWFramesContext*)avctx->hw_frames_ctx->data;
    AVDRMDeviceContext *hwctx = hwfc->device_ctx->hwctx;
    RKGEMDescriptor *desc = NULL;
    AVBufferRef *ref = NULL;
    struct drm_rockchip_gem_create gem_create = {0};
    struct drm_prime_handle prime_handle = {0};
    int ret;

    av_log(NULL, AV_LOG_DEBUG, "%s: avctx=%p hwfc=%p width=%d height=%d hwctx=%p fd=%d\n", __func__, avctx, hwfc, hwfc->width, hwfc->height, hwctx, hwctx->fd);

    desc = av_mallocz(sizeof(RKGEMDescriptor));
    if (!desc)
        return NULL;

    ref = av_buffer_create((uint8_t*)desc, sizeof(RKGEMDescriptor), rkvdec_buffer_free, hwfc, AV_BUFFER_FLAG_READONLY);
    if (!ref) {
        av_freep(&desc);
        return NULL;
    }

    // TODO: use correct format and size
    desc->format = DRM_FORMAT_NV12;
    desc->width = FFALIGN(hwfc->width, 16);
    desc->height = FFALIGN(hwfc->height, 16);
    gem_create.size = desc->width * desc->height * 3 / 2;

    ret = drmIoctl(hwctx->fd, DRM_IOCTL_ROCKCHIP_GEM_CREATE, &gem_create);
    if (ret)
        av_log(NULL, AV_LOG_ERROR, "%s: ioctl DRM_IOCTL_ROCKCHIP_GEM_CREATE failed, ret=%d errno=%d\n", __func__, ret, errno);
    desc->handle = gem_create.handle;
    desc->size = gem_create.size;

    prime_handle.handle = desc->handle;
    ret = drmIoctl(hwctx->fd, DRM_IOCTL_PRIME_HANDLE_TO_FD, &prime_handle);
    if (ret)
        av_log(NULL, AV_LOG_ERROR, "%s: ioctl DRM_IOCTL_PRIME_HANDLE_TO_FD failed, ret=%d errno=%d\n", __func__, ret, errno);
    desc->prime_fd = prime_handle.fd;

    return ref;
}

static void rkvdec_pool_free(void *opaque)
{
    av_log(NULL, AV_LOG_DEBUG, "%s: opaque=%p\n", __func__, opaque);
}

int ff_rkvdec_decode_init(AVCodecContext *avctx)
{
    RKVDECContext *rkctx = avctx->internal->hwaccel_priv_data;
    int ret;

    av_log(avctx, AV_LOG_DEBUG, "%s: avctx=%p rkctx=%p\n", __func__, avctx, rkctx);

    rkctx->fd = open(VPU_DEVICE_NAME, O_RDWR);
    if (rkctx->fd < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: opening %s failed, errno=%d\n", __func__, VPU_DEVICE_NAME, errno);
        return AVERROR(EINVAL);
    }

    ret = ioctl(rkctx->fd, VPU_IOC_SET_CLIENT_TYPE, VPU_DEC);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: ioctl VPU_IOC_SET_CLIENT_TYPE failed, ret=%d errno=%d\n", __func__, ret, errno);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    rkctx->pool = av_buffer_pool_init2(sizeof(RKGEMDescriptor), avctx, rkvdec_buffer_alloc, rkvdec_pool_free);
    if (!rkctx->pool) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    return 0;

fail:
    ff_rkvdec_decode_uninit(avctx);
    return ret;
}

int ff_rkvdec_decode_uninit(AVCodecContext *avctx)
{
    RKVDECContext *rkctx = avctx->internal->hwaccel_priv_data;

    av_log(avctx, AV_LOG_DEBUG, "%s: avctx=%p rkctx=%p fd=%d pool=%p\n", __func__, avctx, rkctx, rkctx->fd, rkctx->pool);

    av_buffer_pool_uninit(&rkctx->pool);

    close(rkctx->fd);

    return 0;
}
