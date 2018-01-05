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

#ifndef AVCODEC_RKVPU_H
#define AVCODEC_RKVPU_H

#include "config.h"

#include "libavutil/hwcontext.h"
#include "libavutil/hwcontext_drm.h"

#include "avcodec.h"
#include "internal.h"

typedef struct RKGEMDescriptor {
    AVDRMDeviceContext *hwctx;
    uint32_t handle;
    uint64_t size;
    int fd;
    uint64_t offset;
    void *addr;
    uint32_t format;
    uint32_t width;
    uint32_t height;
} RKGEMDescriptor;

typedef struct RKVPUContext {
    int fd;
    AVBufferPool *pool;
    RKGEMDescriptor *bitstream;
    RKGEMDescriptor *qp_table;
    void *hw_regs;
} RKVPUContext;

int ff_rkvpu_set_hw_regs(AVCodecContext *avctx, void *hw_regs, int nb_regs);

int ff_rkvpu_get_hw_regs(AVCodecContext *avctx, void *hw_regs, int nb_regs);

int ff_rkvpu_gem_mmap(RKGEMDescriptor *gem);

int ff_rkvpu_gem_munmap(RKGEMDescriptor *gem);

void ff_rkvpu_gem_free(RKGEMDescriptor *gem);

RKGEMDescriptor *ff_rkvpu_gem_alloc(AVDRMDeviceContext *hwctx, uint64_t size);

int ff_rkvpu_decode_init(AVCodecContext *avctx);

int ff_rkvpu_decode_uninit(AVCodecContext *avctx);

#endif /* AVCODEC_RKVPU_H */
