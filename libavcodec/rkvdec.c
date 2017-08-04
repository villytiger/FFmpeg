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

    return 0;

fail:
    ff_rkvdec_decode_uninit(avctx);
    return ret;
}

int ff_rkvdec_decode_uninit(AVCodecContext *avctx)
{
    RKVDECContext *rkctx = avctx->internal->hwaccel_priv_data;

    av_log(avctx, AV_LOG_DEBUG, "%s: avctx=%p rkctx=%p\n", __func__, avctx, rkctx);

    close(rkctx->fd);

    return 0;
}
