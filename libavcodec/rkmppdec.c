#include <drm/drm_fourcc.h>
#include <pthread.h>
#include <rockchip/mpp_buffer.h>
#include <rockchip/rk_mpi.h>
#include <time.h>
#include <unistd.h>

#include "avcodec.h"
#include "drmprime.h"
#include "internal.h"
#include "libavutil/buffer.h"
#include "libavutil/common.h"
#include "libavutil/frame.h"
#include "libavutil/imgutils.h"
#include "libavutil/log.h"

#define RECEIVE_FRAME_TIMEOUT   100
#define FRAMEGROUP_MAX_FRAMES   16

typedef struct {
    MppCtx ctx;
    MppApi *mpi;
    MppBufferGroup frame_group;

    char first_frame;
    char first_packet;
    char eos_reached;
} RKMPPDecoder;

typedef struct {
    AVClass *av_class;
    AVBufferRef *decoder_ref;
    AVBSFContext *bsf;
} RKMPPDecodeContext;

typedef struct {
    MppFrame frame;
    AVBufferRef *decoder_ref;
} RKMPPFrameContext;

static MppCodingType ffrkmpp_get_codingtype(AVCodecContext *avctx)
{
    switch (avctx->codec_id) {
    case AV_CODEC_ID_H264:  return MPP_VIDEO_CodingAVC;
    case AV_CODEC_ID_HEVC:  return MPP_VIDEO_CodingHEVC;
    case AV_CODEC_ID_VP8:   return MPP_VIDEO_CodingVP8;
    default:                return MPP_VIDEO_CodingUnused;
    }
}

static int ffrkmpp_get_frameformat(MppFrameFormat mppformat)
{
    switch (mppformat) {
    case MPP_FMT_YUV420SP:          return DRM_FORMAT_NV12;
#ifdef DRM_FORMAT_NV12_10
    case MPP_FMT_YUV420SP_10BIT:    return DRM_FORMAT_NV12_10;
#endif
    default:                        return 0;
    }
}

static int ffrkmpp_init_bitstream(AVCodecContext *avctx)
{
    RKMPPDecodeContext *rk_context = avctx->priv_data;
    int ret = 0;

    if (!rk_context->bsf) {
        const AVBitStreamFilter *bsf = NULL;
        if (avctx->codec_id == AV_CODEC_ID_H264)
            bsf = av_bsf_get_by_name("h264_mp4toannexb");
        else if (avctx->codec_id == AV_CODEC_ID_HEVC)
            bsf = av_bsf_get_by_name("hevc_mp4toannexb");
        else {
            av_log(avctx, AV_LOG_DEBUG, "Not using any bitstream filter\n");
            return 0;
        }

        if (!bsf)
            return AVERROR_BSF_NOT_FOUND;

        av_log(avctx, AV_LOG_DEBUG, "Using bitstream filter %s\n", bsf->name);

        if ((ret = av_bsf_alloc(bsf, &rk_context->bsf)))
            return ret;

        if (((ret = avcodec_parameters_from_context(rk_context->bsf->par_in, avctx)) < 0) ||
            ((ret = av_bsf_init(rk_context->bsf)) < 0)) {
            av_bsf_free(&rk_context->bsf);
            return ret;
        }
    }

    return 0;
}

static int ffrkmpp_write_data(AVCodecContext *avctx, uint8_t *buffer, int size, int64_t pts)
{
    RKMPPDecodeContext *rk_context = avctx->priv_data;
    RKMPPDecoder *decoder = (RKMPPDecoder *)rk_context->decoder_ref->data;
    MPP_RET ret = MPP_NOK;
    MppPacket packet;

    // create the MPP packet
    ret = mpp_packet_init(&packet, buffer, size);
    if (ret != MPP_OK) {
        av_log(avctx, AV_LOG_ERROR, "Failed to init MPP packet (code = %d)\n", ret);
        return AVERROR_UNKNOWN;
    }

    mpp_packet_set_pts(packet, pts);

    if (!buffer)
        mpp_packet_set_eos(packet);

    ret = decoder->mpi->decode_put_packet(decoder->ctx, packet);
    if (ret != MPP_OK) {
        if (ret == MPP_ERR_BUFFER_FULL) {
            av_log(avctx, AV_LOG_DEBUG, "Buffer full writing %d bytes to decoder\n", size);
            ret = AVERROR(EAGAIN);
        } else
            ret = AVERROR_UNKNOWN;
    }
    else
        av_log(avctx, AV_LOG_DEBUG, "Wrote %d bytes to decoder\n", size);

    mpp_packet_deinit(&packet);

    return ret;
}

static int ffrkmpp_close_decoder(AVCodecContext *avctx)
{
    RKMPPDecodeContext *rk_context = avctx->priv_data;
    av_buffer_unref(&rk_context->decoder_ref);
    av_bsf_free(&rk_context->bsf);
    return 0;
}

static void ffrkmpp_release_decoder(void *opaque, uint8_t *data)
{
    RKMPPDecoder *decoder = (RKMPPDecoder *)data;

    decoder->mpi->reset(decoder->ctx);
    mpp_destroy(decoder->ctx);
    decoder->ctx = NULL;

    if (decoder->frame_group) {
        mpp_buffer_group_put(decoder->frame_group);
        decoder->frame_group = NULL;
    }

    av_free(decoder);
}

static int ffrkmpp_init_decoder(AVCodecContext *avctx)
{
    RKMPPDecodeContext *rk_context = avctx->priv_data;
    RKMPPDecoder *decoder = NULL;
    MppCodingType codectype = MPP_VIDEO_CodingUnused;
    MPP_RET ret = MPP_NOK;
    RK_S64 paramS64;
    RK_S32 paramS32;

    if ((ret = ff_get_format(avctx, avctx->codec->pix_fmts)) < 0)
        return ret;

    avctx->pix_fmt = ret;

    // create a decoder and a ref to it
    decoder = av_mallocz(sizeof(RKMPPDecoder));
    if (!decoder) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    rk_context->decoder_ref = av_buffer_create((uint8_t *)decoder, sizeof(*decoder), ffrkmpp_release_decoder,
                                               NULL, AV_BUFFER_FLAG_READONLY);
    if (!rk_context->decoder_ref) {
        av_free(decoder);
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    av_log(avctx, AV_LOG_DEBUG, "Initializing RKMPP decoder.\n");

    codectype = ffrkmpp_get_codingtype(avctx);
    if (codectype == MPP_VIDEO_CodingUnused) {
        av_log(avctx, AV_LOG_ERROR, "Unknown codec type (%d).\n", avctx->codec_id);
        ret = AVERROR_UNKNOWN;
        goto fail;
    }

    // Create the MPP context
    ret = mpp_create(&decoder->ctx, &decoder->mpi);
    if (ret != MPP_OK) {
        av_log(avctx, AV_LOG_ERROR, "Failed to create MPP context (code = %d).\n", ret);
        ret = AVERROR_UNKNOWN;
        goto fail;
    }

    // initialize mpp
    ret = mpp_init(decoder->ctx, MPP_CTX_DEC, codectype);
    if (ret != MPP_OK) {
        av_log(avctx, AV_LOG_ERROR, "Failed to initialize MPP context (code = %d).\n", ret);
        ret = AVERROR_UNKNOWN;
        goto fail;
    }

    // make decode calls blocking with a timeout
    paramS32 = MPP_POLL_BLOCK;
    ret = decoder->mpi->control(decoder->ctx, MPP_SET_OUTPUT_BLOCK, &paramS32);
    if (ret != MPP_OK) {
        av_log(avctx, AV_LOG_ERROR, "Failed to set blocking mode on MPI (code = %d).\n", ret);
        ret = AVERROR_UNKNOWN;
        goto fail;
    }

    paramS64 = RECEIVE_FRAME_TIMEOUT;
    ret = decoder->mpi->control(decoder->ctx, MPP_SET_OUTPUT_BLOCK_TIMEOUT, &paramS64);
    if (ret != MPP_OK) {
        av_log(avctx, AV_LOG_ERROR, "Failed to set block timeout on MPI (code = %d).\n", ret);
        ret = AVERROR_UNKNOWN;
        goto fail;
    }

    ret = mpp_buffer_group_get_internal(&decoder->frame_group, MPP_BUFFER_TYPE_ION);
    if (ret) {
       av_log(avctx, AV_LOG_ERROR, "Failed to retrieve buffer group (code = %d)\n", ret);
       ret = AVERROR_UNKNOWN;
       goto fail;
    }

    ret = decoder->mpi->control(decoder->ctx, MPP_DEC_SET_EXT_BUF_GROUP, decoder->frame_group);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR, "Failed to assign buffer group (code = %d)\n", ret);
        ret = AVERROR_UNKNOWN;
        goto fail;
    }

    ret = mpp_buffer_group_limit_config(decoder->frame_group, 0, FRAMEGROUP_MAX_FRAMES);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR, "Failed to set buffer group limit (code = %d)\n", ret);
        ret = AVERROR_UNKNOWN;
        goto fail;
    }

    // eventually create a bistream filter for formats that require it
    ret = ffrkmpp_init_bitstream(avctx);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR, "Failed to initialize bitstream (code = %d)\n", ret);
        goto fail;
    }

    decoder->first_packet = 1;

    av_log(avctx, AV_LOG_DEBUG, "RKMPP decoder initialized successfully.\n");
    return 0;

fail:
    av_log(avctx, AV_LOG_ERROR, "Failed to initialize RKMPP decoder.\n");
    ffrkmpp_close_decoder(avctx);
    return ret;
}

static int ffrkmpp_send_packet(AVCodecContext *avctx, const AVPacket *avpkt)
{
    RKMPPDecodeContext *rk_context = avctx->priv_data;
    RKMPPDecoder *decoder = (RKMPPDecoder *)rk_context->decoder_ref->data;
    MPP_RET ret = MPP_OK;
    AVPacket filter_pkt = {0};
    AVPacket filtered_pkt = {0};

    if (decoder->eos_reached)
        return AVERROR_EOF;

    // handle EOF
    if (!avpkt->size) {
        // ignore empty packets until first packet
        if (decoder->first_packet)
            return 0;
        av_log(avctx, AV_LOG_DEBUG, "End of stream.\n");
        decoder->eos_reached = 1;
        ret = ffrkmpp_write_data(avctx, NULL, 0, 0);
        if (ret)
            av_log(avctx, AV_LOG_ERROR, "Failed to send EOS to decoder (code = %d)\n", ret);
        return ret;
    }

    // first we bitstream the packet if it's required
    if (rk_context->bsf) {
        if ((ret = av_packet_ref(&filter_pkt, avpkt)))
            return ret;

        if ((ret = av_bsf_send_packet(rk_context->bsf, &filter_pkt))) {
            av_packet_unref(&filter_pkt);
            return ret;
        }

        if ((ret = av_bsf_receive_packet(rk_context->bsf, &filtered_pkt)))
            return ret;

        avpkt = &filtered_pkt;
    }

    // on first packet, send extradata
    if (decoder->first_packet) {
        if (rk_context->bsf && rk_context->bsf->par_out->extradata_size)
            ret = ffrkmpp_write_data(avctx, rk_context->bsf->par_out->extradata,
                                            rk_context->bsf->par_out->extradata_size,
                                            avpkt->pts);
        else if (!rk_context->bsf && avctx->extradata_size)
            ret = ffrkmpp_write_data(avctx, avctx->extradata,
                                            avctx->extradata_size,
                                            avpkt->pts);
        if (ret) {
            av_log(avctx, AV_LOG_ERROR, "Failed to write extradata to decoder (code = %d)\n", ret);
            goto fail;
        }
        decoder->first_packet = 0;
    }

    // now send packet
    ret = ffrkmpp_write_data(avctx, avpkt->data, avpkt->size, avpkt->pts);
    if (ret)
        av_log(avctx, AV_LOG_ERROR, "Failed to write data to decoder (code = %d)\n", ret);

fail:
    if (rk_context->bsf)
        av_packet_unref(&filtered_pkt);

    return ret;
}

static void ffrkmpp_release_frame(void *opaque, uint8_t *data)
{
    av_drmprime *primedata = (av_drmprime *)data;
    AVBufferRef *framecontextref = (AVBufferRef *)opaque;
    RKMPPFrameContext *framecontext = (RKMPPFrameContext *)framecontextref->data;

    mpp_frame_deinit(&framecontext->frame);
    av_buffer_unref(&framecontext->decoder_ref);
    av_buffer_unref(&framecontextref);

    av_free(primedata);
}

static int ffrkmpp_retrieve_frame(AVCodecContext *avctx, AVFrame *frame)
{
    RKMPPDecodeContext *rk_context = avctx->priv_data;
    RKMPPDecoder *decoder = (RKMPPDecoder *)rk_context->decoder_ref->data;
    RKMPPFrameContext *framecontext = NULL;
    AVBufferRef *framecontextref = NULL;
    MPP_RET ret = MPP_NOK;
    MppFrame mppframe = NULL;
    MppBuffer buffer = NULL;
    av_drmprime *primedata = NULL;
    int retrycount = 0;

    // on start of decoding, MPP can return -1, which is supposed to be expected
    // this is due to some internal MPP init which is not completed, that will
    // only happen in the first few frames queries, but should not be interpreted
    // as an error, Therefore we need to retry a couple times when we get -1
    // in order to let it time to complete it's init, then we sleep a bit between retries.
retry_get_frame:
    ret = decoder->mpi->decode_get_frame(decoder->ctx, &mppframe);
    if (ret != MPP_OK && ret != MPP_ERR_TIMEOUT) {
        if (retrycount < 5) {
            av_log(avctx, AV_LOG_DEBUG, "Failed to get a frame, retrying (code = %d, retrycount = %d)\n", ret, retrycount);
            usleep(10000);
            retrycount++;
            goto retry_get_frame;
        } else {
            av_log(avctx, AV_LOG_ERROR, "Failed to get a frame from MPP (code = %d)\n", ret);
            goto fail;
        }
    }

    if (mppframe) {
        // Check wether we have a special frame or not
        if (mpp_frame_get_info_change(mppframe)) {
            av_log(avctx, AV_LOG_INFO, "Decoder noticed an info change (%dx%d), format=%d\n",
                                        (int)mpp_frame_get_width(mppframe), (int)mpp_frame_get_height(mppframe),
                                        (int)mpp_frame_get_fmt(mppframe));

            avctx->width = mpp_frame_get_width(mppframe);
            avctx->height = mpp_frame_get_height(mppframe);

            decoder->mpi->control(decoder->ctx, MPP_DEC_SET_INFO_CHANGE_READY, NULL);
            decoder->first_frame = 1;

            // here decoder is fully initialized, we need to feed it again with data
            ret = AVERROR(EAGAIN);
            goto fail;
        } else if (mpp_frame_get_eos(mppframe)) {
            av_log(avctx, AV_LOG_DEBUG, "Received a EOS frame.\n");
            decoder->eos_reached = 1;
            ret = AVERROR_EOF;
            goto fail;
        } else if (mpp_frame_get_discard(mppframe) || mpp_frame_get_errinfo(mppframe)) {
            av_log(avctx, AV_LOG_DEBUG, "Received a discard/errinfo frame.\n");
            ret = AVERROR(EAGAIN);
            goto fail;
        }

        // here we should have a valid frame
        av_log(avctx, AV_LOG_DEBUG, "Received a frame.\n");

        // setup general frame fields
        frame->format   = AV_PIX_FMT_RKMPP;
        frame->width    = mpp_frame_get_width(mppframe);
        frame->height   = mpp_frame_get_height(mppframe);
        frame->pts      = mpp_frame_get_pts(mppframe);

        // now setup the frame buffer info
        buffer = mpp_frame_get_buffer(mppframe);
        if (buffer) {
            primedata = av_mallocz(sizeof(av_drmprime));
            if (!primedata) {
                ret = AVERROR(ENOMEM);
                goto fail;
            }

            primedata->strides[0]   = mpp_frame_get_hor_stride(mppframe);
            primedata->strides[1]   = primedata->strides[0];
            primedata->offsets[0]   = 0;
            primedata->offsets[1]   = primedata->strides[0] * mpp_frame_get_ver_stride(mppframe);
            primedata->fds[0]       = mpp_buffer_get_fd(buffer);
            primedata->format       = ffrkmpp_get_frameformat(mpp_frame_get_fmt(mppframe));

            // we also allocate a struct in buf[0] that will allow to hold additionnal information
            // for releasing properly MPP frames and decoder
            framecontextref = av_buffer_allocz(sizeof(*framecontext));
            if (!framecontextref) {
                ret = AVERROR(ENOMEM);
                goto fail;
            }

            // MPP decoder needs to be closed only when all frames have been released.
            framecontext = (RKMPPFrameContext *)framecontextref->data;
            framecontext->decoder_ref = av_buffer_ref(rk_context->decoder_ref);
            framecontext->frame = mppframe;

            frame->data[3]  = (uint8_t *)primedata;
            frame->buf[0]   = av_buffer_create((uint8_t *)primedata, sizeof(*primedata), ffrkmpp_release_frame,
                                               framecontextref, AV_BUFFER_FLAG_READONLY);

            if (!frame->buf[0]) {
                ret = AVERROR(ENOMEM);
                goto fail;
            }

            decoder->first_frame = 0;
            return 0;
        } else {
            av_log(avctx, AV_LOG_ERROR, "Failed to retrieve the frame buffer, frame is dropped (code = %d)\n", ret);
            mpp_frame_deinit(&mppframe);
        }
    } else if (decoder->eos_reached) {
        return AVERROR_EOF;
    } else if (ret == MPP_ERR_TIMEOUT) {
        av_log(avctx, AV_LOG_DEBUG, "Timeout when trying to get a frame from MPP\n");
    }

    return AVERROR(EAGAIN);

fail:
    if (mppframe)
        mpp_frame_deinit(&mppframe);

    if (framecontext)
        av_buffer_unref(&framecontext->decoder_ref);

    if (framecontextref)
        av_buffer_unref(&framecontextref);

    if (primedata)
        av_free(primedata);

    return ret;
}

static int ffrkmpp_decode(AVCodecContext *avctx, void *frame, int *got_frame, AVPacket *avpkt)
{
    int buffer_full = 0;
    int retries = 0;
    int ret;

    *got_frame = 0;

    do {
        if (avpkt) {
            ret = ffrkmpp_send_packet(avctx, avpkt);
            if (ret < 0 && ret != AVERROR(EAGAIN) && ret != AVERROR_EOF) {
                av_log(avctx, AV_LOG_ERROR, "Failed to send packet to decoder (code = %d)\n", ret);
                return ret;
            }
            buffer_full = ret == AVERROR(EAGAIN);
        }

        if (!*got_frame) {
            ret = ffrkmpp_retrieve_frame(avctx, frame);
            if (ret < 0 && ret != AVERROR(EAGAIN) && ret != AVERROR_EOF) {
                av_log(avctx, AV_LOG_ERROR, "Failed to retrieve frame (code = %d)\n", ret);
                return ret;
            }
            if (ret >= 0)
                *got_frame = 1;
        }
    }
    while (buffer_full && retries++ < 5);

    if (buffer_full)
        av_log(avctx, AV_LOG_INFO, "Failed to send packet to decoder (buffer_full = %d, got_frame = %d)\n", buffer_full, *got_frame);
    else if (!*got_frame)
        av_log(avctx, AV_LOG_INFO, "Failed to retrieve frame from decoder (buffer_full = %d, got_frame = %d)\n", buffer_full, *got_frame);

    return !buffer_full && avpkt ? avpkt->size : 0;
}

static void ffrkmpp_flush(AVCodecContext *avctx)
{
    RKMPPDecodeContext *rk_context = avctx->priv_data;
    RKMPPDecoder *decoder = (RKMPPDecoder *)rk_context->decoder_ref->data;
    MPP_RET ret = MPP_NOK;

    av_log(avctx, AV_LOG_DEBUG, "Flush.\n");

    ret = decoder->mpi->reset(decoder->ctx);
    if (ret == MPP_OK) {
        decoder->first_frame = 1;
        decoder->first_packet = 1;
    } else
        av_log(avctx, AV_LOG_ERROR, "Failed to reset MPI (code = %d)\n", ret);
}

#define FFRKMPP_DEC_HWACCEL(NAME, ID) \
  AVHWAccel ff_##NAME##_rkmpp_hwaccel = { \
      .name     = #NAME "_rkmpp", \
      .type     = AVMEDIA_TYPE_VIDEO,\
      .id       = ID, \
      .pix_fmt  = AV_PIX_FMT_RKMPP,\
  };

#define FFRKMPP_DEC_CLASS(NAME) \
    static const AVClass ffrkmpp_##NAME##_dec_class = { \
        .class_name = "rkmpp_" #NAME "_dec", \
        .version    = LIBAVUTIL_VERSION_INT, \
    };

#define FFRKMPP_DEC(NAME, ID) \
    FFRKMPP_DEC_CLASS(NAME) \
    FFRKMPP_DEC_HWACCEL(NAME, ID) \
    AVCodec ff_##NAME##_rkmpp_decoder = { \
        .name           = #NAME "_rkmpp", \
        .long_name      = NULL_IF_CONFIG_SMALL(#NAME " (rkmpp)"), \
        .type           = AVMEDIA_TYPE_VIDEO, \
        .id             = ID, \
        .priv_data_size = sizeof(RKMPPDecodeContext), \
        .init           = ffrkmpp_init_decoder, \
        .close          = ffrkmpp_close_decoder, \
        .decode         = ffrkmpp_decode, \
        .flush          = ffrkmpp_flush, \
        .priv_class     = &ffrkmpp_##NAME##_dec_class, \
        .capabilities   = AV_CODEC_CAP_DELAY, \
        .caps_internal  = FF_CODEC_CAP_SETS_PKT_DTS, \
        .pix_fmts       = (const enum AVPixelFormat[]) { AV_PIX_FMT_RKMPP, \
                                                         AV_PIX_FMT_NONE}, \
    };

FFRKMPP_DEC(h264, AV_CODEC_ID_H264)
FFRKMPP_DEC(hevc, AV_CODEC_ID_HEVC)
FFRKMPP_DEC(vp8,  AV_CODEC_ID_VP8)
