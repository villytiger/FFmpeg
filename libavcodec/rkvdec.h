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

#ifndef AVCODEC_RKVDEC_H
#define AVCODEC_RKVDEC_H

#include "config.h"

#include "libavutil/hwcontext_drm.h"

#include "libavutil/hwcontext.h"

#include "avcodec.h"
#include "internal.h"

typedef struct RKVDECContext {
    int fd;
} RKVDECContext;

int ff_rkvdec_decode_init(AVCodecContext *avctx);

int ff_rkvdec_decode_uninit(AVCodecContext *avctx);

#endif /* AVCODEC_RKVDEC_H */
