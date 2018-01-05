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

#include "mpegutils.h"
#include "mpegvideo.h"

#include "rkvpu.h"

typedef struct RKVPUHWRegsMPEG2 {
    uint32_t      ppReg[50];

    struct {
        uint32_t  sw_dec_out_tiled_e  : 1;
        uint32_t  sw_dec_latency      : 6;
        uint32_t  sw_pic_fixed_quant  : 1;
        uint32_t  sw_filtering_dis    : 1;
        uint32_t  sw_skip_mode        : 1;
        uint32_t  sw_dec_scmd_dis     : 1;
        uint32_t  sw_dec_adv_pre_dis  : 1;
        uint32_t  sw_priority_mode    : 1;
        uint32_t  sw_refbu2_thr       : 12;
        uint32_t  sw_refbu2_picid     : 5;
        uint32_t  reserve0            : 2;
    } config1;

    struct {
        uint32_t  sw_stream_len       : 24;
        uint32_t  sw_stream_len_hi    : 1;
        uint32_t  sw_init_qp          : 6;
        uint32_t  reserve0            : 1;
    } stream_buffinfo;

    struct {
        uint32_t  sw_startmb_y        : 8;
        uint32_t  sw_startmb_x        : 9;
        uint32_t  sw_apf_threshold    : 14;
        uint32_t  reserve0            : 1;
    } error_position;

    uint32_t      sw_dec_mode;

    struct {
        uint32_t  sw_dec_in_endian    : 1;
        uint32_t  sw_dec_out_endian   : 1;
        uint32_t  sw_dec_inswap32_e   : 1;
        uint32_t  sw_dec_outswap32_e  : 1;
        uint32_t  sw_dec_strswap32_e  : 1;
        uint32_t  sw_dec_strendian_e  : 1;
        uint32_t  reserve0            : 26;
    } config2;

    struct {
        uint32_t  sw_dec_irq          : 1;
        uint32_t  sw_dec_irq_dis      : 1;
        uint32_t  reserve0            : 2;
        uint32_t  sw_dec_rdy_int      : 1;
        uint32_t  sw_dec_bus_int      : 1;
        uint32_t  sw_dec_buffer_int   : 1;
        uint32_t  reserve1            : 1;
        uint32_t  sw_dec_aso_int      : 1;
        uint32_t  sw_dec_slice_int    : 1;
        uint32_t  sw_dec_pic_inf      : 1;
        uint32_t  reserve2            : 1;
        uint32_t  sw_dec_error_int    : 1;
        uint32_t  sw_dec_timeout      : 1;
        uint32_t  reserve3            : 18;
    } interrupt;

    struct {
        uint32_t  sw_dec_axi_rd_id    : 8;
        uint32_t  sw_dec_axi_wr_id    : 8;
        uint32_t  sw_dec_max_burst    : 5;
        uint32_t  sw_paral_bus_e      : 1;
        uint32_t  sw_dec_data_disc_e  : 1;
        uint32_t  sw_axi_dec_sel      : 1;
        uint32_t  reserve0            : 8;
    } config3;

    struct {
        uint32_t  sw_dec_e            : 1;
        uint32_t  sw_refbu2_buf_e     : 1;
        uint32_t  sw_dec_out_dis      : 1;
        uint32_t  reserve0            : 1;
        uint32_t  sw_dec_clk_gate_e   : 1;
        uint32_t  sw_dec_timeout_e    : 1;
        uint32_t  sw_picord_count_e   : 1;
        uint32_t  sw_seq_mbaff_e      : 1;
        uint32_t  sw_reftopfirst_e    : 1;
        uint32_t  sw_ref_topfield_e   : 1;
        uint32_t  sw_write_mvs_e      : 1;
        uint32_t  sw_sorenson_e       : 1;
        uint32_t  sw_fwd_interlace_e  : 1;
        uint32_t  sw_pic_topfield_e   : 1;
        uint32_t  sw_pic_inter_e      : 1;
        uint32_t  sw_pic_b_e          : 1;
        uint32_t  sw_pic_fieldmode_e  : 1;
        uint32_t  sw_pic_interlace_e  : 1;
        uint32_t  sw_pjpeg_e          : 1;
        uint32_t  sw_divx3_e          : 1;
        uint32_t  sw_rlc_mode_e       : 1;
        uint32_t  sw_ch_8pix_ileav_e  : 1;
        uint32_t  sw_start_code_e     : 1;
        uint32_t  reserve1            : 2;
        uint32_t  sw_inter_double_speed : 1;
        uint32_t  sw_intra_double_speed : 1;
        uint32_t  sw_intra_3_cycle_enhance : 1;
        uint32_t  sw_prefetch_single_channel_e : 1;
        uint32_t  sw_cache_e          : 1;
        uint32_t  reserve2            : 1;
        uint32_t  sw_dec_ahb_hlock_e  : 1;
    } control;

    uint32_t      soft_reset;
    uint32_t      reserve0;

    struct {
        uint32_t  sw_reserve          : 2;
        uint32_t  sw_dec_ch8pix_base  : 30;
    } ch8pix;

    uint32_t      slice_table;
    uint32_t      directmv_reg;
    uint32_t      cur_pic_base;
    uint32_t      VLC_base;

    struct {
        uint32_t  sw_refbu_y_offset   : 9;
        uint32_t  sw_reserve          : 3;
        uint32_t  sw_refbu_fparmod_e  : 1;
        uint32_t  sw_refbu_eval_e     : 1;
        uint32_t  sw_refbu_picid      : 5;
        uint32_t  sw_refbu_thr        : 12;
        uint32_t  sw_refbu_e          : 1;
    } refbuff_ctl;

    struct {
        uint32_t  build_version       : 3;
        uint32_t  product_IDen        : 1;
        uint32_t  minor_version       : 8;
        uint32_t  major_version       : 4;
        uint32_t  product_numer       : 16;
    } id;

    struct {
        uint32_t  sw_reserve          : 25;
        uint32_t  sw_dec_rtl_rom      : 1;
        uint32_t  sw_dec_rv_prof      : 2;
        uint32_t  sw_ref_buff2_exist  : 1;
        uint32_t  sw_dec_divx_prof    : 1;
        uint32_t  sw_dec_refbu_ilace  : 1;
        uint32_t  sw_dec_jpeg_exten   : 1;
    } syn_cfg;

    struct {
        uint32_t  sw_refbu_top_sum    : 16;
        uint32_t  sw_refbu_bot_sum    : 16;
    } refbuff_info3;

    struct {
        uint32_t  sw_refbu_intra_sum  : 16;
        uint32_t  sw_refbu_hit_sum    : 16;
    } refbuff_info1;

    struct {
        uint32_t  sw_refbu_mv_sum     : 22;
        uint32_t  sw_reserve          : 10;
    } refbuff_info2;

    struct {
        uint32_t  sw_dec_max_owidth   : 11;
        uint32_t  sw_dec_soren_prof   : 1;
        uint32_t  sw_dec_bus_width    : 2;
        uint32_t  sw_dec_synth_lan    : 2;
        uint32_t  sw_dec_bus_strd     : 4;
        uint32_t  sw_ref_buff_exist   : 1;
        uint32_t  sw_dec_obuff_leve   : 1;
        uint32_t  sw_dec_pjpeg_exist  : 1;
        uint32_t  sw_vp6_prof         : 1;
        uint32_t  sw_dec_h264_prof    : 2;
        uint32_t  sw_dec_mpeg4_prof   : 2;
        uint32_t  sw_dec_jpeg_prof    : 1;
        uint32_t  sw_dec_vc1_prof     : 2;
        uint32_t  sw_dec_mpeg2_prof   : 1;
    } syn_config;

    uint32_t      reserve2;

    struct {
        uint32_t  debug_dec_mb_count  : 13;
        uint32_t  debug_referreq1     : 1;
        uint32_t  debug_referreq0     : 1;
        uint32_t  debug_filter_req    : 1;
        uint32_t  debug_framerdy      : 1;
        uint32_t  debug_strm_da_e     : 1;
        uint32_t  debug_res_c_req     : 1;
        uint32_t  debug_res_y_req     : 1;
        uint32_t  debug_rlc_req       : 1;
        uint32_t  debug_mv_req        : 10;
    } debug_info;

    uint32_t      reserve3[46];

    struct {
        uint32_t  sw_ref_frames       : 5;
        uint32_t  sw_topfieldfirst_e  : 1;
        uint32_t  sw_alt_scan_e       : 1;
        uint32_t  sw_mb_height_off    : 4;
        uint32_t  sw_pic_mb_height_p  : 8;
        uint32_t  sw_mb_width_off     : 4;
        uint32_t  sw_pic_mb_width     : 9;
    } pic_params;

    uint32_t      reserve4;

    struct {
        uint32_t  sw_frame_pred_dct   : 1;
        uint32_t  sw_intra_vlc_tab    : 1;
        uint32_t  sw_intra_dc_prec    : 2;
        uint32_t  sw_con_mv_e         : 1;
        uint32_t  reserve             : 19;
        uint32_t  sw_qscale_type      : 1;
        uint32_t  reserve1            : 1;
        uint32_t  sw_stream_start_bit : 6;
    } stream_bitinfo;

    uint32_t      reserve5[8];
    uint32_t      ref0;
    uint32_t      reserve6[2];
    uint32_t      ref2;
    uint32_t      ref3;

    struct {
        uint32_t  reserve             : 1;
        uint32_t  sw_mv_accuracy_bwd  : 1;
        uint32_t  sw_mv_accuracy_fwd  : 1;
        uint32_t  sw_fcode_bwd_ver    : 4;
        uint32_t  sw_fcode_bwd_hor    : 4;
        uint32_t  sw_fcode_fwd_ver    : 4;
        uint32_t  sw_fcode_fwd_hor    : 4;
        uint32_t  sw_alt_scan_flag_e  : 1;
        uint32_t  reserve1            : 12;
    } dec_info;

    uint32_t      reserve7[11];
    uint32_t      ref1;
    uint32_t      reserve8[10];
} RKVPUHWRegsMPEG2;

typedef struct RKVPUPictureMPEG2 {
    unsigned        slice_count;
    const uint8_t   *bitstream;
    unsigned        bitstream_size;
} RKVPUPictureMPEG2;

static int rkvpu_mpeg2_start_frame(AVCodecContext *avctx,
                                  av_unused const uint8_t *buffer,
                                  av_unused uint32_t size)
{
    RKVPUContext *rkctx = avctx->internal->hwaccel_priv_data;
    AVHWFramesContext *hwfc = (AVHWFramesContext*)avctx->hw_frames_ctx->data;
    AVDRMDeviceContext *hwctx = hwfc->device_ctx->hwctx;
    const MpegEncContext *s = avctx->priv_data;
    RKVPUPictureMPEG2 *pic = s->current_picture_ptr->hwaccel_picture_private;
    AVDRMFrameDescriptor *desc;

    av_log(avctx, AV_LOG_DEBUG, "%s: avctx=%p rkctx=%p pic=%p\n", __func__, avctx, rkctx, pic);

    pic->slice_count = 0;
    pic->bitstream_size = 0;
    pic->bitstream = NULL;

    desc = (AVDRMFrameDescriptor*)s->current_picture_ptr->f->data[0];
    if (desc)
        memset(desc, 0, sizeof(AVDRMFrameDescriptor));

    if (!rkctx->bitstream) {
        rkctx->bitstream = ff_rkvpu_gem_alloc(hwctx, 2 * 1024 * 1024);
        ff_rkvpu_gem_mmap(rkctx->bitstream);
    }

    if (!rkctx->qp_table) {
        rkctx->qp_table = ff_rkvpu_gem_alloc(hwctx, 4096);
        ff_rkvpu_gem_mmap(rkctx->qp_table);
    }

    return 0;
}

static int rkvpu_mpeg2_decode_slice(AVCodecContext *avctx,
                                   const uint8_t *buffer,
                                   uint32_t size)
{
    const MpegEncContext *s = avctx->priv_data;
    RKVPUPictureMPEG2 *pic = s->current_picture_ptr->hwaccel_picture_private;

    av_log(avctx, AV_LOG_INFO, "%s: avctx=%p pic=%p slice_count=%u size=%u\n", __func__, avctx, pic, pic->slice_count, size);

    if (!pic->bitstream)
        pic->bitstream = buffer;
    pic->bitstream_size += size;

    pic->slice_count++;

    return 0;
}

static int rkvpu_mpeg2_end_frame(AVCodecContext *avctx)
{
    RKVPUContext *rkctx = avctx->internal->hwaccel_priv_data;
    RKVPUHWRegsMPEG2 *hw_regs = rkctx->hw_regs;
    const MpegEncContext *s = avctx->priv_data;
    RKVPUPictureMPEG2 *pic = s->current_picture_ptr->hwaccel_picture_private;
    AVDRMFrameDescriptor *desc;
    AVDRMLayerDescriptor *layer;
    RKGEMDescriptor *gem, *last_picture_gem, *next_picture_gem;
    uint8_t *qp_table;
    int i;

    av_log(avctx, AV_LOG_DEBUG, "%s: avctx=%p pic=%p slice_count=%u bitstream_size=%u\n", __func__, avctx, pic, pic->slice_count, pic->bitstream_size);

    if (pic->slice_count <= 0 || pic->bitstream_size <= 0)
        return -1;

    desc = (AVDRMFrameDescriptor*)s->current_picture_ptr->f->data[0];
    if (desc) {
        RKVPUHWRegsMPEG2 hw_regs_out = {0};

        s->current_picture_ptr->f->buf[1] = av_buffer_pool_get(rkctx->pool);
        if (!s->current_picture_ptr->f->buf[1])
            return AVERROR(ENOMEM);

        s->current_picture_ptr->f->data[1] = (uint8_t*)s->current_picture_ptr->f->buf[1]->data;
        gem = (RKGEMDescriptor*)s->current_picture_ptr->f->data[1];

        av_log(avctx, AV_LOG_DEBUG, "%s: desc=%p gem=%p handle=%u fd=%d format=%u width=%u height=%u size=%lu\n", __func__, desc, gem, gem->handle, gem->fd, gem->format, gem->width, gem->height, gem->size);

        hw_regs->ppReg[0] = 0;
        hw_regs->dec_info.sw_mv_accuracy_fwd = 1;
        hw_regs->dec_info.sw_mv_accuracy_bwd = 1;
        hw_regs->dec_info.sw_fcode_fwd_hor = s->mpeg_f_code[0][0];
        hw_regs->dec_info.sw_fcode_fwd_ver = s->mpeg_f_code[0][1];
        hw_regs->dec_info.sw_fcode_bwd_hor = s->mpeg_f_code[1][0];
        hw_regs->dec_info.sw_fcode_bwd_ver = s->mpeg_f_code[1][1];

        if (avctx->codec_id == AV_CODEC_ID_MPEG2VIDEO) {
            hw_regs->sw_dec_mode = 5;
        } else {
            hw_regs->sw_dec_mode = 6;
            if (s->full_pel[0])
                hw_regs->dec_info.sw_mv_accuracy_fwd = 0;
            if (s->full_pel[1])
                hw_regs->dec_info.sw_mv_accuracy_bwd = 0;
        }

        hw_regs->pic_params.sw_pic_mb_width = s->mb_width;
        hw_regs->pic_params.sw_pic_mb_height_p = s->mb_height;
        hw_regs->control.sw_pic_interlace_e = 1 - s->progressive_sequence;

        hw_regs->control.sw_pic_fieldmode_e = (s->picture_structure != PICT_FRAME);
        hw_regs->control.sw_pic_topfield_e = (s->picture_structure == PICT_TOP_FIELD);
        hw_regs->control.sw_pic_b_e = (s->pict_type == AV_PICTURE_TYPE_B);
        hw_regs->control.sw_pic_inter_e = (s->pict_type != AV_PICTURE_TYPE_I);

        hw_regs->pic_params.sw_topfieldfirst_e = s->top_field_first;
        hw_regs->control.sw_fwd_interlace_e = 0;
        hw_regs->control.sw_write_mvs_e = 0;
        hw_regs->pic_params.sw_alt_scan_e = s->alternate_scan;
        hw_regs->dec_info.sw_alt_scan_flag_e = s->alternate_scan;

        hw_regs->stream_bitinfo.sw_qscale_type = s->q_scale_type;
        hw_regs->stream_bitinfo.sw_intra_dc_prec = s->intra_dc_precision;
        hw_regs->stream_bitinfo.sw_con_mv_e = s->concealment_motion_vectors;
        hw_regs->stream_bitinfo.sw_intra_vlc_tab = s->intra_vlc_format;
        hw_regs->stream_bitinfo.sw_frame_pred_dct = s->frame_pred_frame_dct;
        hw_regs->stream_buffinfo.sw_init_qp = 1;

        hw_regs->error_position.sw_startmb_x = 0;
        hw_regs->error_position.sw_startmb_y = 0;
        hw_regs->control.sw_dec_out_dis = 0;
        hw_regs->config1.sw_filtering_dis = 1;

        memcpy(rkctx->bitstream->addr, pic->bitstream, pic->bitstream_size);

        qp_table = (uint8_t*)rkctx->qp_table->addr;
        for (i = 0; i < 64; i++) {
            qp_table[i + 0] = (uint8_t)s->intra_matrix[i];
            qp_table[i + 64] = (uint8_t)s->inter_matrix[i];
            qp_table[i + 128] = (uint8_t)s->chroma_intra_matrix[i];
            qp_table[i + 192] = (uint8_t)s->chroma_inter_matrix[i];
        }

        hw_regs->VLC_base = rkctx->bitstream->fd;
        hw_regs->slice_table = rkctx->qp_table->fd;

        if (s->picture_structure == PICT_TOP_FIELD || s->picture_structure == PICT_FRAME) {
            hw_regs->cur_pic_base = gem->fd;
        } else {
            hw_regs->cur_pic_base = gem->fd | (s->mb_width << 14);
        }

        last_picture_gem = (RKGEMDescriptor*)s->last_picture.f->data[1];
        if (!last_picture_gem)
            last_picture_gem = gem;

        if (s->pict_type == AV_PICTURE_TYPE_B) {
            next_picture_gem = (RKGEMDescriptor*)s->next_picture.f->data[1];
            if (!next_picture_gem)
                next_picture_gem = last_picture_gem;

            hw_regs->ref0 = last_picture_gem->fd;
            hw_regs->ref1 = last_picture_gem->fd;
            hw_regs->ref2 = next_picture_gem->fd;
            hw_regs->ref3 = next_picture_gem->fd;
        } else {
            if (s->picture_structure == PICT_FRAME ||
               (s->picture_structure == PICT_TOP_FIELD && s->top_field_first) ||
               (s->picture_structure == PICT_BOTTOM_FIELD && !s->top_field_first)) {
                hw_regs->ref0 = last_picture_gem->fd;
                hw_regs->ref1 = last_picture_gem->fd;
            } else if (s->picture_structure == PICT_TOP_FIELD) {
                hw_regs->ref0 = last_picture_gem->fd;
                hw_regs->ref1 = gem->fd;
            } else if (s->picture_structure == PICT_BOTTOM_FIELD) {
                hw_regs->ref0 = gem->fd;
                hw_regs->ref1 = last_picture_gem->fd;
            }
            hw_regs->ref2 = gem->fd;
            hw_regs->ref3 = gem->fd;
        }

        hw_regs->stream_buffinfo.sw_stream_len = pic->bitstream_size;
        hw_regs->stream_bitinfo.sw_stream_start_bit = 0;
        hw_regs->control.sw_dec_e = 1;

        ff_rkvpu_set_hw_regs(avctx, hw_regs, 159);
        ff_rkvpu_get_hw_regs(avctx, &hw_regs_out, 159);

        // TODO: check for errors
        av_log(avctx, AV_LOG_DEBUG, "%s: sw_dec_error_int=%d sw_dec_buffer_int=%d\n", __func__, hw_regs_out.interrupt.sw_dec_error_int, hw_regs_out.interrupt.sw_dec_buffer_int);

        desc->nb_objects = 1;
        desc->objects[0].fd = gem->fd;
        desc->objects[0].size = gem->size;

        desc->nb_layers = 1;
        layer = &desc->layers[0];
        layer->format = gem->format;
        layer->nb_planes = 2;

        layer->planes[0].object_index = 0;
        layer->planes[0].offset = 0;
        layer->planes[0].pitch = gem->width;

        layer->planes[1].object_index = 0;
        layer->planes[1].offset = layer->planes[0].pitch * gem->height;
        layer->planes[1].pitch = layer->planes[0].pitch;
    }

    return 0;
}

static int rkvpu_mpeg2_init(AVCodecContext *avctx)
{
    RKVPUContext *rkctx = avctx->internal->hwaccel_priv_data;
    RKVPUHWRegsMPEG2 *hw_regs;

    av_log(avctx, AV_LOG_DEBUG, "%s: avctx=%p rkctx=%p\n", __func__, avctx, rkctx);

    hw_regs = av_mallocz(sizeof(RKVPUHWRegsMPEG2));
    if (!hw_regs)
        return AVERROR(ENOMEM);

    hw_regs->control.sw_dec_timeout_e = 1;
    hw_regs->config2.sw_dec_strswap32_e = 1;
    hw_regs->config2.sw_dec_strendian_e = 1;
    hw_regs->config2.sw_dec_inswap32_e = 1;
    hw_regs->config2.sw_dec_outswap32_e = 1;

    hw_regs->control.sw_dec_clk_gate_e = 1;
    hw_regs->config2.sw_dec_in_endian = 1;
    hw_regs->config2.sw_dec_out_endian = 1;

    hw_regs->config1.sw_dec_out_tiled_e = 0;
    hw_regs->config3.sw_dec_max_burst = 16;
    hw_regs->config1.sw_dec_scmd_dis = 0;
    hw_regs->config1.sw_dec_adv_pre_dis = 0;
    hw_regs->error_position.sw_apf_threshold = 8;

    hw_regs->config1.sw_dec_latency = 0;
    hw_regs->config3.sw_dec_data_disc_e = 0;

    hw_regs->interrupt.sw_dec_irq = 0;
    hw_regs->config3.sw_dec_axi_rd_id = 0;
    hw_regs->config3.sw_dec_axi_wr_id = 0;

    hw_regs->sw_dec_mode = 8;

    rkctx->hw_regs = hw_regs;

    return ff_rkvpu_decode_init(avctx);
}

AVHWAccel ff_mpeg2_rkvpu_hwaccel = {
    .name           = "mpeg2_rkvpu",
    .type           = AVMEDIA_TYPE_VIDEO,
    .id             = AV_CODEC_ID_MPEG2VIDEO,
    .pix_fmt        = AV_PIX_FMT_DRM_PRIME,
    .init           = &rkvpu_mpeg2_init,
    .uninit         = ff_rkvpu_decode_uninit,
    .start_frame    = &rkvpu_mpeg2_start_frame,
    .decode_slice   = &rkvpu_mpeg2_decode_slice,
    .end_frame      = &rkvpu_mpeg2_end_frame,
    .frame_priv_data_size = sizeof(RKVPUPictureMPEG2),
    .priv_data_size = sizeof(RKVPUContext),
};
