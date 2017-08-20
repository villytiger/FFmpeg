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

#include "libavutil/common.h"
#include "libavutil/log.h"
#include "libavutil/time.h"

#include "avcodec.h"
#include "rkvdec.h"

#define VPU_DEVICE_NAME                     "/dev/rkvdec"

#define VPU_IOC_MAGIC                       'l'
#define VPU_IOC_SET_CLIENT_TYPE             _IOW(VPU_IOC_MAGIC, 1, unsigned long)
#define VPU_IOC_GET_HW_FUSE_STATUS          _IOW(VPU_IOC_MAGIC, 2, unsigned long)
#define VPU_IOC_SET_REG                     _IOW(VPU_IOC_MAGIC, 3, unsigned long)
#define VPU_IOC_GET_REG                     _IOW(VPU_IOC_MAGIC, 4, unsigned long)
#define VPU_IOC_PROBE_IOMMU_STATUS          _IOR(VPU_IOC_MAGIC, 5, unsigned long)
#define VPU_IOC_WRITE(nr, size)             _IOC(_IOC_WRITE, VPU_IOC_MAGIC, (nr), (size))

enum VPU_CLIENT_TYPE {
    VPU_ENC         = 0x0,
    VPU_DEC         = 0x1,
    VPU_PP          = 0x2,
    VPU_DEC_PP      = 0x3,
};

static MppCodingType ff_rkvdec_get_codingtype(enum AVCodecID codec_id)
{
    switch (codec_id) {
    //case AV_CODEC_ID_H264:          return MPP_VIDEO_CodingAVC;
    case AV_CODEC_ID_HEVC:          return MPP_VIDEO_CodingHEVC;
    //case AV_CODEC_ID_VP8:           return MPP_VIDEO_CodingVP8;
    //case AV_CODEC_ID_VP9:           return MPP_VIDEO_CodingVP9;
    //case AV_CODEC_ID_MPEG2VIDEO:    return MPP_VIDEO_CodingMPEG2;
    //case AV_CODEC_ID_MPEG4:         return MPP_VIDEO_CodingMPEG4;
    default:                        return MPP_VIDEO_CodingUnused;
    }
}

int ff_rkvdec_write_data(AVCodecContext *avctx, const uint8_t *buffer, uint32_t size, int64_t dts, int64_t pts)
{
    RKVDECContext *rkctx = avctx->internal->hwaccel_priv_data;
    int ret;
    MppPacket packet;

    ret = mpp_packet_init(&packet, buffer, size);
    if (ret != MPP_OK) {
        av_log(avctx, AV_LOG_ERROR, "%s: initializing MPP packet failed, ret=%d\n",  __func__, ret);
        return AVERROR_UNKNOWN;
    }

    //mpp_packet_set_dts(packet, dts);
    //mpp_packet_set_pts(packet, pts);

    if (!buffer)
        mpp_packet_set_eos(packet);

    ret = rkctx->mpi->decode_put_packet(rkctx->mppctx, packet);
    if (ret != MPP_OK) {
        if (ret == MPP_ERR_BUFFER_FULL) {
            av_log(avctx, AV_LOG_DEBUG, "%s: buffer full writing %u bytes to decoder\n", __func__, size);
            ret = AVERROR(EAGAIN);
        } else
            ret = AVERROR_UNKNOWN;
    }
    else
        av_log(avctx, AV_LOG_DEBUG, "%s: wrote %u bytes to decoder\n", __func__, size);

    mpp_packet_deinit(&packet);

    return ret;
}

int ff_rkvdec_retrieve_frame(AVCodecContext *avctx, AVFrame *frame)
{
    RKVDECContext *rkctx = avctx->internal->hwaccel_priv_data;
    MppFrame mppframe = NULL;
    int ret;

restart:
    ret = rkctx->mpi->decode_get_frame(rkctx->mppctx, &mppframe);
    if (ret != MPP_OK && ret != MPP_ERR_TIMEOUT) {
        av_log(avctx, AV_LOG_ERROR, "%s: getting a MPP frame failed, ret=%d\n", __func__, ret);
        return AVERROR_UNKNOWN;
    }

    if (mppframe) {
        if (mpp_frame_get_info_change(mppframe)) {
            av_log(avctx, AV_LOG_INFO, "%s: decoder noticed an info change (%dx%d), format=%d\n", __func__,
                                        (int)mpp_frame_get_width(mppframe), (int)mpp_frame_get_height(mppframe),
                                        (int)mpp_frame_get_fmt(mppframe));
            rkctx->mpi->control(rkctx->mppctx, MPP_DEC_SET_INFO_CHANGE_READY, NULL);
            mpp_frame_deinit(&mppframe);
            goto restart;
        } else if (mpp_frame_get_eos(mppframe)) {
            av_log(avctx, AV_LOG_INFO, "%s: received a EOS frame\n", __func__);
        } else if (mpp_frame_get_discard(mppframe)) {
            av_log(avctx, AV_LOG_INFO, "%s: received a discard frame\n", __func__);
        } else if (mpp_frame_get_errinfo(mppframe)) {
            av_log(avctx, AV_LOG_INFO, "%s: received a errinfo frame\n", __func__);
        } else {
            av_log(avctx, AV_LOG_DEBUG, "%s: received a frame\n", __func__);
        }
        mpp_frame_deinit(&mppframe);
    } else if (ret == MPP_ERR_TIMEOUT) {
        av_log(avctx, AV_LOG_INFO, "%s: timeout when trying to get a frame from MPP\n", __func__);
    }

    return 0;
}

int ff_rkvdec_decode_init(AVCodecContext *avctx)
{
    RKVDECContext *rkctx = avctx->internal->hwaccel_priv_data;
    int ret;
    MppCodingType codingtype;
    RK_S32 paramS32;
    RK_S64 paramS64;

    av_log(avctx, AV_LOG_DEBUG, "%s: avctx=%p rkctx=%p\n", __func__, avctx, rkctx);

    /*
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
    */

    codingtype = ff_rkvdec_get_codingtype(avctx->codec_id);
    if (codingtype == MPP_VIDEO_CodingUnused) {
        av_log(avctx, AV_LOG_ERROR, "%s: unsupported codec, codec=%s\n", __func__, avcodec_get_name(avctx->codec_id));
        return AVERROR(EINVAL);
    }

    ret = mpp_create(&rkctx->mppctx, &rkctx->mpi);
    if (ret != MPP_OK) {
        av_log(avctx, AV_LOG_ERROR, "%s: creating MPP context failed, ret=%d\n", __func__, ret);
        ret = AVERROR_UNKNOWN;
        goto fail;
    }

    ret = mpp_init(rkctx->mppctx, MPP_CTX_DEC, codingtype);
    if (ret != MPP_OK) {
        av_log(avctx, AV_LOG_ERROR, "%s: initializing MPP context failed, ret=%d\n",  __func__, ret);
        ret = AVERROR_UNKNOWN;
        goto fail;
    }

    paramS32 = MPP_POLL_BLOCK;
    ret = rkctx->mpi->control(rkctx->mppctx, MPP_SET_OUTPUT_BLOCK, &paramS32);
    if (ret != MPP_OK) {
        av_log(avctx, AV_LOG_ERROR, "%s: set blocking mode failed, ret=%d\n",  __func__, ret);
        ret = AVERROR_UNKNOWN;
        goto fail;
    }

    paramS64 = 100;
    ret = rkctx->mpi->control(rkctx->mppctx, MPP_SET_OUTPUT_BLOCK_TIMEOUT, &paramS64);
    if (ret != MPP_OK) {
        av_log(avctx, AV_LOG_ERROR, "%s: set blocking timeout failed, ret=%d\n",  __func__, ret);
        ret = AVERROR_UNKNOWN;
        goto fail;
    }

    av_log(avctx, AV_LOG_INFO, "%s: MPP context initialized\n",  __func__);

    return 0;

fail:
    ff_rkvdec_decode_uninit(avctx);
    return ret;
}

int ff_rkvdec_decode_uninit(AVCodecContext *avctx)
{
    RKVDECContext *rkctx = avctx->internal->hwaccel_priv_data;

    av_log(avctx, AV_LOG_DEBUG, "%s: avctx=%p rkctx=%p\n", __func__, avctx, rkctx);

    if (rkctx->mppctx) {
        rkctx->mpi->reset(rkctx->mppctx);
        mpp_destroy(rkctx->mppctx);
        rkctx->mpi = NULL;
        rkctx->mppctx = NULL;
    }

    //close(rkctx->fd);

    return 0;
}
