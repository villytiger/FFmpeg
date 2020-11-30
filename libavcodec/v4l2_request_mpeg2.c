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

#include "hwconfig.h"
#include "mpegvideo.h"
#include "v4l2_request.h"
#include "mpeg2-ctrls.h"

typedef struct V4L2RequestControlsMPEG2 {
    struct v4l2_ctrl_mpeg2_slice_params slice_params;
    struct v4l2_ctrl_mpeg2_sequence sequence;
    struct v4l2_ctrl_mpeg2_picture picture;
    struct v4l2_ctrl_mpeg2_quantization quantization;
} V4L2RequestControlsMPEG2;

static int v4l2_request_mpeg2_start_frame(AVCodecContext *avctx,
                                          av_unused const uint8_t *buffer,
                                          av_unused uint32_t size)
{
    const MpegEncContext *s = avctx->priv_data;
    V4L2RequestControlsMPEG2 *controls = s->current_picture_ptr->hwaccel_picture_private;
    V4L2RequestDescriptor *req = (V4L2RequestDescriptor*)s->current_picture_ptr->f->data[0];

    controls->slice_params = (struct v4l2_ctrl_mpeg2_slice_params) {
        .quantiser_scale_code = s->qscale >> 1,
    };

    switch (s->pict_type) {
    case AV_PICTURE_TYPE_B:
        controls->slice_params.backward_ref_ts = ff_v4l2_request_get_capture_timestamp(s->next_picture.f);
        // fall-through
    case AV_PICTURE_TYPE_P:
        controls->slice_params.forward_ref_ts = ff_v4l2_request_get_capture_timestamp(s->last_picture.f);
    }

    controls->sequence = (struct v4l2_ctrl_mpeg2_sequence) {
        .horizontal_size = s->width,
        .vertical_size = s->height,
        .vbv_buffer_size = req->output.size,
        .profile_and_level_indication = 0,
        .chroma_format = s->chroma_format,
    };

    if (s->progressive_sequence)
        controls->sequence.flags |= V4L2_MPEG2_SEQ_FLAG_PROGRESSIVE;

    controls->picture = (struct v4l2_ctrl_mpeg2_picture) {
        .picture_coding_type = s->pict_type,
        .f_code[0][0] = s->mpeg_f_code[0][0],
        .f_code[0][1] = s->mpeg_f_code[0][1],
        .f_code[1][0] = s->mpeg_f_code[1][0],
        .f_code[1][1] = s->mpeg_f_code[1][1],
        .intra_dc_precision = s->intra_dc_precision,
        .picture_structure = s->picture_structure,
    };

    if (s->top_field_first)
        controls->picture.flags |= V4L2_MPEG2_PIC_FLAG_TOP_FIELD_FIRST;
    if (s->frame_pred_frame_dct)
        controls->picture.flags |= V4L2_MPEG2_PIC_FLAG_FRAME_PRED_DCT;
    if (s->concealment_motion_vectors)
        controls->picture.flags |= V4L2_MPEG2_PIC_FLAG_CONCEALMENT_MV;
    if (s->q_scale_type)
        controls->picture.flags |= V4L2_MPEG2_PIC_FLAG_Q_SCALE_TYPE;
    if (s->intra_vlc_format)
        controls->picture.flags |= V4L2_MPEG2_PIC_FLAG_INTRA_VLC;
    if (s->alternate_scan)
        controls->picture.flags |= V4L2_MPEG2_PIC_FLAG_ALT_SCAN;
    if (s->repeat_first_field)
        controls->picture.flags |= V4L2_MPEG2_PIC_FLAG_REPEAT_FIRST;
    if (s->progressive_frame)
        controls->picture.flags |= V4L2_MPEG2_PIC_FLAG_PROGRESSIVE;

    controls->picture.flags |=
        V4L2_MPEG2_PIC_FLAG_LOAD_INTRA |
        V4L2_MPEG2_PIC_FLAG_LOAD_NON_INTRA |
        V4L2_MPEG2_PIC_FLAG_LOAD_CHROMA_INTRA |
        V4L2_MPEG2_PIC_FLAG_LOAD_CHROMA_NON_INTRA;

    for (int i = 0; i < 64; i++) {
        int n = s->idsp.idct_permutation[ff_zigzag_direct[i]];
        controls->quantization.intra_quantiser_matrix[i] = s->intra_matrix[n];
        controls->quantization.non_intra_quantiser_matrix[i] = s->inter_matrix[n];
        controls->quantization.chroma_intra_quantiser_matrix[i] = s->chroma_intra_matrix[n];
        controls->quantization.chroma_non_intra_quantiser_matrix[i] = s->chroma_inter_matrix[n];
    }

    return ff_v4l2_request_reset_frame(avctx, s->current_picture_ptr->f);
}

static int v4l2_request_mpeg2_decode_slice(AVCodecContext *avctx, const uint8_t *buffer, uint32_t size)
{
    const MpegEncContext *s = avctx->priv_data;

    return ff_v4l2_request_append_output_buffer(avctx, s->current_picture_ptr->f, buffer, size);
}

static int v4l2_request_mpeg2_end_frame(AVCodecContext *avctx)
{
    const MpegEncContext *s = avctx->priv_data;
    V4L2RequestControlsMPEG2 *controls = s->current_picture_ptr->hwaccel_picture_private;

    struct v4l2_ext_control control[] = {
        {
            .id = V4L2_CID_MPEG_VIDEO_MPEG2_SLICE_PARAMS,
            .ptr = &controls->slice_params,
            .size = sizeof(controls->slice_params),
        },
        {
            .id = V4L2_CID_MPEG_VIDEO_MPEG2_SEQUENCE,
            .ptr = &controls->sequence,
            .size = sizeof(controls->sequence),
        },
        {
            .id = V4L2_CID_MPEG_VIDEO_MPEG2_PICTURE,
            .ptr = &controls->picture,
            .size = sizeof(controls->picture),
        },
        {
            .id = V4L2_CID_MPEG_VIDEO_MPEG2_QUANTIZATION,
            .ptr = &controls->quantization,
            .size = sizeof(controls->quantization),
        },
    };

    return ff_v4l2_request_decode_frame(avctx, s->current_picture_ptr->f, control, FF_ARRAY_ELEMS(control));
}

static int v4l2_request_mpeg2_init(AVCodecContext *avctx)
{
    return ff_v4l2_request_init(avctx, V4L2_PIX_FMT_MPEG2_SLICE, 1024 * 1024, NULL, 0);
}

const AVHWAccel ff_mpeg2_v4l2request_hwaccel = {
    .name           = "mpeg2_v4l2request",
    .type           = AVMEDIA_TYPE_VIDEO,
    .id             = AV_CODEC_ID_MPEG2VIDEO,
    .pix_fmt        = AV_PIX_FMT_DRM_PRIME,
    .start_frame    = v4l2_request_mpeg2_start_frame,
    .decode_slice   = v4l2_request_mpeg2_decode_slice,
    .end_frame      = v4l2_request_mpeg2_end_frame,
    .frame_priv_data_size = sizeof(V4L2RequestControlsMPEG2),
    .init           = v4l2_request_mpeg2_init,
    .uninit         = ff_v4l2_request_uninit,
    .priv_data_size = sizeof(V4L2RequestContext),
    .frame_params   = ff_v4l2_request_frame_params,
    .caps_internal  = HWACCEL_CAP_ASYNC_SAFE,
};
