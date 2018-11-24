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

#include "h264dec.h"
#include "hwaccel.h"
#include "v4l2_request.h"

static int v4l2_request_h264_start_frame(AVCodecContext *avctx,
                                         av_unused const uint8_t *buffer,
                                         av_unused uint32_t size)
{
    const H264Context *h = avctx->priv_data;

    return ff_v4l2_request_reset_frame(avctx, h->cur_pic_ptr->f);
}

static int v4l2_request_h264_decode_slice(AVCodecContext *avctx, const uint8_t *buffer, uint32_t size)
{
    const H264Context *h = avctx->priv_data;

    return ff_v4l2_request_append_output_buffer(avctx, h->cur_pic_ptr->f, buffer, size);
}

static void fill_single_pred_weight_table(const H264SliceContext *sl, int list,
                                          struct v4l2_h264_weight_factors *factors)
{
    unsigned int i, j;

    for (i = 0; i < sl->ref_count[list]; i++) {
        if (sl->pwt.luma_weight_flag[list]) {
            factors->luma_weight[i] = sl->pwt.luma_weight[i][list][0];
            factors->luma_offset[i] = sl->pwt.luma_weight[i][list][1];
        } else {
            factors->luma_weight[i] = 1 << sl->pwt.luma_log2_weight_denom;
            factors->luma_offset[i] = 0;
        }
        for (j = 0; j < 2; j++) {
            if (sl->pwt.chroma_weight_flag[list]) {
                factors->chroma_weight[i][j] = sl->pwt.chroma_weight[i][list][j][0];
                factors->chroma_offset[i][j] = sl->pwt.chroma_weight[i][list][j][1];
            } else {
                factors->chroma_weight[i][j] = 1 << sl->pwt.chroma_log2_weight_denom;
                factors->chroma_offset[i][j] = 0;
            }
        }
    }
}

typedef struct DPB {
    int                         size;
    int                         max_size;
    struct v4l2_h264_dpb_entry *entries;
} DPB;

static void fill_pic(struct v4l2_h264_dpb_entry *entry,
                     const H264Picture          *pic)
{
    entry->buf_index = ff_v4l2_request_get_capture_index(pic->f);
    entry->frame_num = pic->long_ref ? pic->pic_id : pic->frame_num;
    entry->flags = V4L2_H264_DPB_ENTRY_FLAG_VALID | V4L2_H264_DPB_ENTRY_FLAG_ACTIVE;
    if ((pic->reference & PICT_FRAME) != PICT_FRAME)
        entry->flags |= (pic->reference & PICT_TOP_FIELD) ? V4L2_H264_DPB_ENTRY_FLAG_TOP_FIELD : V4L2_H264_DPB_ENTRY_FLAG_BOTTOM_FIELD;
    if (pic->reference)
        entry->flags |= pic->long_ref ? V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM : V4L2_H264_DPB_ENTRY_FLAG_SHORT_TERM;
    //if ((pic->reference & PICT_TOP_FIELD) && pic->field_poc[0] != INT_MAX)
        entry->top_field_order_cnt = pic->field_poc[0];
    //if ((pic->reference & PICT_BOTTOM_FIELD) && pic->field_poc[1] != INT_MAX)
        entry->bottom_field_order_cnt = pic->field_poc[1];
}

static int dpb_add(DPB *dpb, const H264Picture *pic)
{
    int i;

    for (i = 0; i < dpb->size; i++) {
        struct v4l2_h264_dpb_entry * const entry = &dpb->entries[i];
        if (entry->buf_index == ff_v4l2_request_get_capture_index(pic->f)) {
            struct v4l2_h264_dpb_entry temp_entry;

            fill_pic(&temp_entry, pic);
	    av_log(NULL, AV_LOG_ERROR, "%s: DPB index: %d buffer index: %d\n", __func__, i, entry->buf_index);

            if ((temp_entry.flags ^ entry->flags) & (V4L2_H264_DPB_ENTRY_FLAG_TOP_FIELD | V4L2_H264_DPB_ENTRY_FLAG_BOTTOM_FIELD)) {
                entry->flags |= temp_entry.flags & (V4L2_H264_DPB_ENTRY_FLAG_TOP_FIELD | V4L2_H264_DPB_ENTRY_FLAG_BOTTOM_FIELD);
                if (temp_entry.flags & V4L2_H264_DPB_ENTRY_FLAG_TOP_FIELD) {
                    entry->top_field_order_cnt    = temp_entry.top_field_order_cnt;
                } else {
                    entry->bottom_field_order_cnt = temp_entry.bottom_field_order_cnt;
                }
            }
            return 0;
        }
    }

    if (dpb->size >= dpb->max_size)
        return -1;

    fill_pic(&dpb->entries[dpb->size++], pic);
    av_log(NULL, AV_LOG_ERROR, "%s: new DPB index: %d buffer index: %d\n", __func__, dpb->size-1, dpb->entries[dpb->size-1].buf_index);
    return 0;
}

static int fill_dpb(struct v4l2_ctrl_h264_decode_param *decode,
                    const H264Context                  *h)
{
    DPB dpb;
    int i;

    dpb.size     = 0;
    dpb.max_size = FF_ARRAY_ELEMS(decode->dpb);
    dpb.entries  = decode->dpb;

    for (i = 0; i < h->short_ref_count; i++) {
        const H264Picture *pic = h->short_ref[i];
        if (pic && dpb_add(&dpb, pic) < 0)
            return -1;
    }

    for (i = 0; i < 16; i++) {
        const H264Picture *pic = h->long_ref[i];
        if (pic && dpb_add(&dpb, pic) < 0)
            return -1;
    }

    for (i = 0; i < FF_ARRAY_ELEMS(h->DPB); i++) {
        const H264Picture *pic = &h->DPB[i];

        if (pic != h->cur_pic_ptr && pic->f->buf[0])
            dpb_add(&dpb, pic);
    }

    av_log(NULL, AV_LOG_ERROR, "%s: num of DPB entries: %d\n", __func__, dpb.size);

    return 0;
}

static uint8_t get_ref_pic_index(const H264Picture *pic,
                                 struct v4l2_ctrl_h264_decode_param *decode)
{
    int frame_buf_index;
    int frame_num;
    uint8_t i;

    if (!pic)
        return 0;

    frame_buf_index = ff_v4l2_request_get_capture_index(pic->f);
    frame_num = pic->long_ref ? pic->pic_id : pic->frame_num;

    for (i = 0; i < FF_ARRAY_ELEMS(decode->dpb); i++) {
        struct v4l2_h264_dpb_entry *entry = &decode->dpb[i];
	if ((entry->flags & V4L2_H264_DPB_ENTRY_FLAG_VALID) && (entry->buf_index == frame_buf_index) && (entry->frame_num == frame_num)) {
		av_log(NULL, AV_LOG_ERROR, "%s: index found: %d for buf_index: %d\n", __func__, i, frame_buf_index);
            return i;
	}
    }

    av_log(NULL, AV_LOG_ERROR, "%s: index not found for buf_index: %d\n", __func__, frame_buf_index);

    return 0;
}

static int v4l2_request_h264_end_frame(AVCodecContext *avctx)
{
    const H264Context *h = avctx->priv_data;
    V4L2RequestDescriptor *req = (V4L2RequestDescriptor*)h->cur_pic_ptr->f->data[0];
    struct v4l2_ctrl_h264_scaling_matrix matrix = { 0 };
    struct v4l2_ctrl_h264_decode_param decode = { 0 };
    struct v4l2_ctrl_h264_slice_param slice = { 0 };
    struct v4l2_ctrl_h264_pps req_pps = { 0 };
    struct v4l2_ctrl_h264_sps req_sps = { 0 };
    struct v4l2_ext_control control[5];
    const H264SliceContext *sl = &h->slice_ctx[0];
    const SPS *sps = h->ps.sps;
    const PPS *pps = h->ps.pps;
    int count, i;

    req_sps = (struct v4l2_ctrl_h264_sps) {
        .profile_idc = sps->profile_idc,
        .constraint_set_flags = sps->constraint_set_flags,
        .level_idc = sps->level_idc,
        .seq_parameter_set_id = sps->sps_id,
        .chroma_format_idc = sps->chroma_format_idc,
        .bit_depth_luma_minus8 = sps->bit_depth_luma - 8,
        .bit_depth_chroma_minus8 = sps->bit_depth_chroma - 8,
        .log2_max_frame_num_minus4 = sps->log2_max_frame_num - 4,
        .pic_order_cnt_type = sps->poc_type,
        .log2_max_pic_order_cnt_lsb_minus4 = sps->log2_max_poc_lsb - 4,
        .offset_for_non_ref_pic = sps->offset_for_non_ref_pic,
        .offset_for_top_to_bottom_field = sps->offset_for_top_to_bottom_field,
        .num_ref_frames_in_pic_order_cnt_cycle = sps->poc_cycle_length,
        //.offset_for_ref_frame[255] - not required? not set by libva-v4l2-request
        .max_num_ref_frames = 0, // what is this? not set by libva-v4l2-request
        .pic_width_in_mbs_minus1 = h->mb_width - 1,
	.pic_height_in_map_units_minus1 = sps->frame_mbs_only_flag ? h->mb_height - 1 : h->mb_height / 2 - 1,
    };

    if (sps->residual_color_transform_flag)
        req_sps.flags |= V4L2_H264_SPS_FLAG_SEPARATE_COLOUR_PLANE;
    if (sps->gaps_in_frame_num_allowed_flag)
        req_sps.flags |= V4L2_H264_SPS_FLAG_GAPS_IN_FRAME_NUM_VALUE_ALLOWED;
    if (sps->frame_mbs_only_flag)
        req_sps.flags |= V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY;
    if (sps->mb_aff)
        req_sps.flags |= V4L2_H264_SPS_FLAG_MB_ADAPTIVE_FRAME_FIELD;
    if (sps->direct_8x8_inference_flag)
        req_sps.flags |= V4L2_H264_SPS_FLAG_DIRECT_8X8_INFERENCE;
    if (sps->delta_pic_order_always_zero_flag)
        req_sps.flags |= V4L2_H264_SPS_FLAG_DELTA_PIC_ORDER_ALWAYS_ZERO;

    req_pps = (struct v4l2_ctrl_h264_pps) {
        .pic_parameter_set_id = 0, /* not needed? not present anywhere? */
        .seq_parameter_set_id = pps->sps_id,
        .num_slice_groups_minus1 = pps->slice_group_count - 1,
        .num_ref_idx_l0_default_active_minus1 = pps->ref_count[0] - 1,
        .num_ref_idx_l1_default_active_minus1 = pps->ref_count[1] - 1,
        .weighted_bipred_idc = pps->weighted_bipred_idc,
        .pic_init_qp_minus26 = pps->init_qp - 26,
        .pic_init_qs_minus26 = pps->init_qs - 26,
        .chroma_qp_index_offset = pps->chroma_qp_index_offset[0],
        .second_chroma_qp_index_offset = pps->chroma_qp_index_offset[1],
    };

    if (pps->cabac)
        req_pps.flags |= V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE;

    if (pps->weighted_pred)
        req_pps.flags |= V4L2_H264_PPS_FLAG_WEIGHTED_PRED;

    if (pps->transform_8x8_mode)
        req_pps.flags |= V4L2_H264_PPS_FLAG_TRANSFORM_8X8_MODE;

    if (pps->constrained_intra_pred)
        req_pps.flags |= V4L2_H264_PPS_FLAG_CONSTRAINED_INTRA_PRED;

    if (pps->pic_order_present)
        req_pps.flags |= V4L2_H264_PPS_FLAG_BOTTOM_FIELD_PIC_ORDER_IN_FRAME_PRESENT;

    if (pps->deblocking_filter_parameters_present)
        req_pps.flags |= V4L2_H264_PPS_FLAG_DEBLOCKING_FILTER_CONTROL_PRESENT;

    if (pps->redundant_pic_cnt_present)
        req_pps.flags |= V4L2_H264_PPS_FLAG_REDUNDANT_PIC_CNT_PRESENT;

    memcpy(matrix.scaling_list_4x4, pps->scaling_matrix4, sizeof(matrix.scaling_list_4x4));
    memcpy(matrix.scaling_list_8x8, pps->scaling_matrix8, sizeof(matrix.scaling_list_8x8));

    decode = (struct v4l2_ctrl_h264_decode_param) {
        .num_slices = 1, /* TODO */
        .idr_pic_flag = h->picture_idr,
        .nal_ref_idc = h->nal_ref_idc,
        .top_field_order_cnt = h->cur_pic_ptr->field_poc[0],
        .bottom_field_order_cnt = h->cur_pic_ptr->field_poc[1],
        //.ref_pic_list_p0[32]  - not required? not set by libva-v4l2-request
        //.ref_pic_list_b0[32]  - not required? not set by libva-v4l2-request
        //.ref_pic_list_b1[32]  - not required? not set by libva-v4l2-request
    };

    fill_dpb(&decode, h);

    slice = (struct v4l2_ctrl_h264_slice_param) {
        .size = req->output.used,
        .header_bit_size = get_bits_count(&sl->gb),
        .first_mb_in_slice = (sl->mb_y >> FIELD_OR_MBAFF_PICTURE(h)) * h->mb_width + sl->mb_x,
        .slice_type = ff_h264_get_slice_type(sl),
        .pic_parameter_set_id = sl->pps_id,
        .colour_plane_id = 0, /* what is this? */
        .frame_num = h->poc.frame_num,
        .idr_pic_id = 0, /* what is this? */
        .pic_order_cnt_lsb = sl->poc_lsb,
        .delta_pic_order_cnt_bottom = sl->delta_poc_bottom,
        .delta_pic_order_cnt0 = sl->delta_poc[0],
        .delta_pic_order_cnt1 = sl->delta_poc[1],
        .redundant_pic_cnt = sl->redundant_pic_count,

        /* Size in bits of dec_ref_pic_marking() syntax element. */
        .dec_ref_pic_marking_bit_size = 0,
        /* Size in bits of pic order count syntax. */
        .pic_order_cnt_bit_size = 0,

	.cabac_init_idc = h->ps.pps->cabac ? sl->cabac_init_idc : 0,
        .slice_qp_delta = sl->qscale - pps->init_qp,
	.slice_qs_delta = 0, /* XXX not implemented by FFmpeg */
        .disable_deblocking_filter_idc = sl->deblocking_filter < 2 ? !sl->deblocking_filter : sl->deblocking_filter,
        .slice_alpha_c0_offset_div2 = sl->slice_alpha_c0_offset / 2,
        .slice_beta_offset_div2 = sl->slice_beta_offset / 2,
        .slice_group_change_cycle = 0, /* what is this? */

        .num_ref_idx_l0_active_minus1 = sl->list_count > 0 ? sl->ref_count[0] - 1 : 0,
        .num_ref_idx_l1_active_minus1 = sl->list_count > 1 ? sl->ref_count[1] - 1 : 0,
    };

    av_log(NULL, AV_LOG_ERROR, "%s: slice type: %d\n", __func__, slice.slice_type);

    if (sl->slice_type == AV_PICTURE_TYPE_B && sl->direct_spatial_mv_pred)
        slice.flags |= V4L2_H264_SLICE_FLAG_DIRECT_SPATIAL_MV_PRED;
    if ((h->picture_structure & PICT_FRAME) != PICT_FRAME) {
        slice.flags |= V4L2_H264_SLICE_FLAG_FIELD_PIC;
        if (h->picture_structure & PICT_BOTTOM_FIELD)
            slice.flags |= V4L2_H264_SLICE_FLAG_BOTTOM_FIELD;
    }

    slice.pred_weight_table.chroma_log2_weight_denom = sl->pwt.chroma_log2_weight_denom;
    slice.pred_weight_table.luma_log2_weight_denom = sl->pwt.luma_log2_weight_denom;

    count = sl->list_count > 0 ? sl->ref_count[0] : 0;
    av_log(NULL, AV_LOG_ERROR, "%s: backward ref count: %d\n", __func__, count);
    for (i = 0; i < count; i++) {
        slice.ref_pic_list0[i] = get_ref_pic_index(sl->ref_list[0][i].parent, &decode) << 1;
        if (sl->ref_list[0][i].reference & PICT_BOTTOM_FIELD)
            slice.ref_pic_list0[i] |= 1;
    }
    if (count)
        fill_single_pred_weight_table(sl, 0, &slice.pred_weight_table.weight_factors[0]);

    count = sl->list_count > 1 ? sl->ref_count[1] : 0;
    av_log(NULL, AV_LOG_ERROR, "%s: forward ref count: %d\n", __func__, count);
    for (i = 0; i < count; i++) {
        slice.ref_pic_list1[i] = get_ref_pic_index(sl->ref_list[1][i].parent, &decode) << 1;
        if (sl->ref_list[1][i].reference & PICT_BOTTOM_FIELD)
            slice.ref_pic_list1[i] |= 1;
    }
    if (count)
        fill_single_pred_weight_table(sl, 1, &slice.pred_weight_table.weight_factors[1]);

    control[0].id = V4L2_CID_MPEG_VIDEO_H264_SPS;
    control[0].ptr = &req_sps;
    control[0].size = sizeof(req_sps);

    control[1].id = V4L2_CID_MPEG_VIDEO_H264_PPS;
    control[1].ptr = &req_pps;
    control[1].size = sizeof(req_pps);

    control[2].id = V4L2_CID_MPEG_VIDEO_H264_SCALING_MATRIX;
    control[2].ptr = &matrix;
    control[2].size = sizeof(matrix);

    control[3].id = V4L2_CID_MPEG_VIDEO_H264_SLICE_PARAMS;
    control[3].ptr = &slice;
    control[3].size = sizeof(slice);

    control[4].id = V4L2_CID_MPEG_VIDEO_H264_DECODE_PARAMS;
    control[4].ptr = &decode;
    control[4].size = sizeof(decode);

    return ff_v4l2_request_decode_frame(avctx, h->cur_pic_ptr->f, control, 5);
}

static int v4l2_request_h264_init(AVCodecContext *avctx)
{
    return ff_v4l2_request_init(avctx, V4L2_PIX_FMT_H264_SLICE, 1024 * 1024, NULL, 0);
}

const AVHWAccel ff_h264_v4l2request_hwaccel = {
    .name           = "h264_v4l2request",
    .type           = AVMEDIA_TYPE_VIDEO,
    .id             = AV_CODEC_ID_H264,
    .pix_fmt        = AV_PIX_FMT_DRM_PRIME,
    .start_frame    = v4l2_request_h264_start_frame,
    .decode_slice   = v4l2_request_h264_decode_slice,
    .end_frame      = v4l2_request_h264_end_frame,
    .init           = v4l2_request_h264_init,
    .uninit         = ff_v4l2_request_uninit,
    .priv_data_size = sizeof(V4L2RequestContext),
    .frame_params   = ff_v4l2_request_frame_params,
    .caps_internal  = HWACCEL_CAP_ASYNC_SAFE,
};
