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

#include <drm_fourcc.h>
#include <linux/media.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <sys/sysmacros.h>
#include <libudev.h>

#include "decode.h"
#include "hwaccel.h"
#include "internal.h"
#include "libavutil/hwcontext_drm.h"

typedef struct {
    AVDRMFrameDescriptor base;
    int index;
    AVBufferRef *decoder_ref;
} V4L2FrameDescriptor;

typedef struct {
    struct v4l2_format format;
    uint32_t capabilities;
    int streaming;
} V4L2BufferQueue;

typedef struct {
    int fd;
    V4L2BufferQueue output;
    V4L2BufferQueue capture;
    AVBufferRef *frames_ref;
    AVBufferRef *device_ref;
} V4L2Decoder;

typedef struct {
    AVClass *av_class;
    AVBufferRef *decoder_ref;
} V4L2DecoderContext;

static int v4l2request_try_format(AVCodecContext *avctx, V4L2Decoder *decoder, V4L2BufferQueue *queue, uint32_t pixelformat)
{
    enum v4l2_buf_type type = queue->format.type;
    struct v4l2_fmtdesc fmtdesc = {
        .index = 0,
        .type = type,
    };

    if (V4L2_TYPE_IS_OUTPUT(type)) {
        struct v4l2_create_buffers buffers = {
            .count = 0,
            .memory = V4L2_MEMORY_MMAP,
            .format.type = type,
        };

        if (ioctl(decoder->fd, VIDIOC_CREATE_BUFS, &buffers) < 0) {
            av_log(avctx, AV_LOG_ERROR, "%s: create buffers failed for type %u, %s (%d)\n", __func__, type, strerror(errno), errno);
            return -1;
        }

        queue->capabilities = buffers.capabilities;
    }

    while (ioctl(decoder->fd, VIDIOC_ENUM_FMT, &fmtdesc) >= 0) {
        if (fmtdesc.pixelformat == pixelformat)
            return 0;

        fmtdesc.index++;
    }

    av_log(avctx, AV_LOG_INFO, "%s: pixelformat %u not supported for type %u\n", __func__, pixelformat, type);
    return -1;
}

static int v4l2request_set_format(AVCodecContext *avctx, V4L2Decoder *decoder, V4L2BufferQueue *queue, uint32_t pixelformat, uint32_t buffersize)
{
    enum v4l2_buf_type type = queue->format.type;
    struct v4l2_format format = {
        .type = type,
    };

    if (V4L2_TYPE_IS_MULTIPLANAR(type)) {
        format.fmt.pix_mp.width = avctx->coded_width;
        format.fmt.pix_mp.height = avctx->coded_height;
        format.fmt.pix_mp.pixelformat = pixelformat;
        format.fmt.pix_mp.plane_fmt[0].sizeimage = buffersize;
        format.fmt.pix_mp.num_planes = 1;
    } else {
        format.fmt.pix.width = avctx->coded_width;
        format.fmt.pix.height = avctx->coded_height;
        format.fmt.pix.pixelformat = pixelformat;
        format.fmt.pix.sizeimage = buffersize;
    }

    return ioctl(decoder->fd, VIDIOC_S_FMT, &format);
}

static int v4l2request_get_format(AVCodecContext *avctx, V4L2Decoder *decoder, V4L2BufferQueue *queue)
{
    struct v4l2_format *format = &queue->format;
    int ret;

    ret = ioctl(decoder->fd, VIDIOC_G_FMT, format);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: get format failed, %s (%d)\n", __func__, strerror(errno), errno);
        return AVERROR(EINVAL);
    }

    if (V4L2_TYPE_IS_MULTIPLANAR(format->type)) {
        av_log(avctx, AV_LOG_DEBUG, "%s: pixelformat=%d width=%u height=%u bytesperline=%u sizeimage=%u num_planes=%u\n", __func__, format->fmt.pix_mp.pixelformat, format->fmt.pix_mp.width, format->fmt.pix_mp.height, format->fmt.pix_mp.plane_fmt[0].bytesperline, format->fmt.pix_mp.plane_fmt[0].sizeimage, format->fmt.pix_mp.num_planes);
    } else {
        av_log(avctx, AV_LOG_DEBUG, "%s: pixelformat=%d width=%u height=%u bytesperline=%u sizeimage=%u\n", __func__, format->fmt.pix.pixelformat, format->fmt.pix.width, format->fmt.pix.height, format->fmt.pix.bytesperline, format->fmt.pix.sizeimage);
    }

    return 0;
}

static int v4l2request_set_streaming(AVCodecContext *avctx, V4L2Decoder *decoder, V4L2BufferQueue *queue, int streaming)
{
    int ret;

    ret = ioctl(decoder->fd, streaming ? VIDIOC_STREAMON : VIDIOC_STREAMOFF, &queue->format.type);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: setting streaming on/off failed, %s (%d)\n", __func__, strerror(errno), errno);
        return AVERROR(EINVAL);
    }

    queue->streaming = streaming;

    return 0;
}

static int v4l2request_probe_video_device(struct udev_device *device, AVCodecContext *avctx, V4L2Decoder *decoder, uint32_t pixelformat)
{
    int ret = AVERROR(EINVAL);
    struct v4l2_capability capability = {0};
    uint32_t capabilities = 0;

    const char *path = udev_device_get_devnode(device);
    if (!path) {
        av_log(avctx, AV_LOG_ERROR, "%s: get video device devnode failed\n", __func__);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    decoder->fd = open(path, O_RDWR | O_NONBLOCK, 0);
    if (decoder->fd < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: opening %s failed, %s (%d)\n", __func__, path, strerror(errno), errno);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    ret = ioctl(decoder->fd, VIDIOC_QUERYCAP, &capability);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: get video capability failed, %s (%d)\n", __func__, strerror(errno), errno);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    if (capability.capabilities & V4L2_CAP_DEVICE_CAPS)
        capabilities = capability.device_caps;
    else
        capabilities = capability.capabilities;

    av_log(avctx, AV_LOG_DEBUG, "%s: avctx=%p decoder=%p path=%s capabilities=%u\n", __func__, avctx, decoder, path, capabilities);

    if ((capabilities & V4L2_CAP_STREAMING) != V4L2_CAP_STREAMING) {
        av_log(avctx, AV_LOG_ERROR, "%s: missing required streaming capability\n", __func__);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    if ((capabilities & V4L2_CAP_VIDEO_M2M_MPLANE) == V4L2_CAP_VIDEO_M2M_MPLANE) {
        decoder->output.format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        decoder->capture.format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    } else if ((capabilities & V4L2_CAP_VIDEO_M2M) == V4L2_CAP_VIDEO_M2M) {
        decoder->output.format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        decoder->capture.format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    } else {
        av_log(avctx, AV_LOG_ERROR, "%s: missing required mem2mem capability\n", __func__);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    ret = v4l2request_try_format(avctx, decoder, &decoder->output, pixelformat);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: try output format failed\n", __func__);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    // Initialization - 1. Set the coded format on ``OUTPUT`` via :c:func:`VIDIOC_S_FMT`
    ret = v4l2request_set_format(avctx, decoder, &decoder->output, pixelformat, 0);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: set output format failed, %s (%d)\n", __func__, strerror(errno), errno);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    return 0;

fail:
    if (decoder->fd >= 0) {
        close(decoder->fd);
        decoder->fd = -1;
    }
    return ret;
}

static void v4l2request_uninit_decoder(V4L2Decoder *decoder)
{
    av_buffer_unref(&decoder->frames_ref);
    av_buffer_unref(&decoder->device_ref);
}

static int v4l2request_init_decoder(AVCodecContext *avctx, V4L2Decoder *decoder)
{
    int ret;

    ret = v4l2request_get_format(avctx, decoder, &decoder->output);
    if (ret < 0)
        goto fail;

    decoder->device_ref = av_hwdevice_ctx_alloc(AV_HWDEVICE_TYPE_DRM);
    if (!decoder->device_ref) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    ret = av_hwdevice_ctx_init(decoder->device_ref);
    if (ret < 0)
        goto fail;

    // TODO: Initialization - 2. Allocate source (bytestream) buffers via :c:func:`VIDIOC_REQBUFS` on ``OUTPUT``.

    // TODO: Initialization - 3. Start streaming on the ``OUTPUT`` queue via :c:func:`VIDIOC_STREAMON`.
    //ret = v4l2request_set_streaming(avctx, decoder, &decoder->output, 1);
    //if (ret < 0)
    //    goto fail;

    // HACK: Capture Setup - Call :c:func:`VIDIOC_G_FMT` on the ``CAPTURE`` queue to get format for the destination buffers parsed/decoded from the bytestream.
    ret = v4l2request_get_format(avctx, decoder, &decoder->capture);
    if (ret < 0)
        goto fail;

    // HACK: Capture Setup - 10. Allocate ``CAPTURE`` buffers via :c:func:`VIDIOC_REQBUFS` on the ``CAPTURE`` queue.

    // HACK: queue all capture buffers

    // HACK: Capture Setup - 11. Call :c:func:`VIDIOC_STREAMON` on the ``CAPTURE`` queue to start decoding frames.
    //ret = v4l2request_set_streaming(avctx, decoder, &decoder->capture, 1);
    //if (ret < 0)
    //    goto fail;

    // HACK:
    avctx->pix_fmt = AV_PIX_FMT_DRM_PRIME;
    avctx->width = avctx->coded_width;
    avctx->height = avctx->coded_height;

    return 0;

fail:
    v4l2request_uninit_decoder(decoder);
    return ret;
}

static int v4l2request_probe(AVCodecContext *avctx, V4L2Decoder *decoder, uint32_t pixelformat)
{
    struct udev *udev;
    struct udev_enumerate *enumerate;
    struct udev_list_entry *devices;
    struct udev_list_entry *entry;
    struct udev_device *device;
    int ret = AVERROR(EINVAL);

    udev = udev_new();
    if (!udev) {
        av_log(avctx, AV_LOG_ERROR, "%s: allocating udev context failed\n", __func__);
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    enumerate = udev_enumerate_new(udev);
    if (!enumerate) {
        av_log(avctx, AV_LOG_ERROR, "%s: allocating udev enumerator failed\n", __func__);
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    udev_enumerate_add_match_subsystem(enumerate, "video4linux");
    udev_enumerate_scan_devices(enumerate);

    devices = udev_enumerate_get_list_entry(enumerate);
    udev_list_entry_foreach(entry, devices) {
        const char *path = udev_list_entry_get_name(entry);
        if (!path)
            continue;

        device = udev_device_new_from_syspath(udev, path);
        if (!device)
            continue;

        ret = v4l2request_probe_video_device(device, avctx, decoder, pixelformat);
        udev_device_unref(device);

        if (!ret)
            break;
    }

    udev_enumerate_unref(enumerate);

    if (!ret)
        ret = v4l2request_init_decoder(avctx, decoder);

fail:
    udev_unref(udev);
    return ret;
}

static void v4l2request_release_decoder(void *opaque, uint8_t *data)
{
    V4L2Decoder *decoder = (V4L2Decoder *)data;

    av_log(NULL, AV_LOG_DEBUG, "%s: decoder=%p fd=%d\n", __func__, decoder, decoder->fd);

    if (decoder->fd >= 0)
        close(decoder->fd);

    v4l2request_uninit_decoder(decoder);

    av_free(decoder);
}

static int v4l2request_close(AVCodecContext *avctx)
{
    V4L2DecoderContext *ctx = avctx->priv_data;
    V4L2Decoder *decoder = (V4L2Decoder *)ctx->decoder_ref->data;

    av_log(avctx, AV_LOG_DEBUG, "%s: avctx=%p ctx=%p decoder=%p\n", __func__, avctx, ctx, decoder);

    av_buffer_unref(&ctx->decoder_ref);
    return 0;
}

static int v4l2request_init(AVCodecContext *avctx)
{
    V4L2DecoderContext *ctx = avctx->priv_data;
    V4L2Decoder *decoder;
    uint32_t pixelformat;
    int ret;

    switch (avctx->codec_id) {
    case AV_CODEC_ID_H264:          pixelformat = V4L2_PIX_FMT_H264; break;
    case AV_CODEC_ID_HEVC:          pixelformat = V4L2_PIX_FMT_HEVC; break;
    case AV_CODEC_ID_MPEG1VIDEO:    pixelformat = V4L2_PIX_FMT_MPEG1; break;
    case AV_CODEC_ID_MPEG2VIDEO:    pixelformat = V4L2_PIX_FMT_MPEG2; break;
    case AV_CODEC_ID_MPEG4:         pixelformat = V4L2_PIX_FMT_MPEG4; break;
    case AV_CODEC_ID_H263:          pixelformat = V4L2_PIX_FMT_H263; break;
    case AV_CODEC_ID_VC1:           pixelformat = V4L2_PIX_FMT_VC1_ANNEX_G; break;
    case AV_CODEC_ID_VP8:           pixelformat = V4L2_PIX_FMT_VP8; break;
    case AV_CODEC_ID_VP9:           pixelformat = V4L2_PIX_FMT_VP9; break;
    default:
        av_log(avctx, AV_LOG_ERROR, "%s: unknown codec, avctx=%p ctx=%p\n", __func__, avctx, ctx);
        return AVERROR(EINVAL);
    }

    av_log(avctx, AV_LOG_DEBUG, "%s: avctx=%p ctx=%p pixelformat=%08x\n", __func__, avctx, ctx, pixelformat);

    decoder = av_mallocz(sizeof(V4L2Decoder));
    if (!decoder)
        return AVERROR(ENOMEM);

    decoder->fd = -1;

    ctx->decoder_ref = av_buffer_create((uint8_t *)decoder, sizeof(*decoder), v4l2request_release_decoder,
                                        NULL, AV_BUFFER_FLAG_READONLY);
    if (!ctx->decoder_ref) {
        av_free(decoder);
        return AVERROR(ENOMEM);
    }

    av_log(avctx, AV_LOG_DEBUG, "%s: avctx=%p ctx=%p decoder=%p\n", __func__, avctx, ctx, decoder);

    ret = v4l2request_probe(avctx, decoder, pixelformat);
    if (ret < 0)
        goto fail;

    av_log(avctx, AV_LOG_DEBUG, "%s: avctx=%p ctx=%p decoder=%p fd=%d device=%p\n", __func__, avctx, ctx, decoder, decoder->fd, decoder->device_ref->data);

    return 0;

fail:
    av_log(avctx, AV_LOG_ERROR, "%s: failed to initialize\n", __func__);
    v4l2request_close(avctx);
    return ret;
}

static void v4l2request_release_frame(void *opaque, uint8_t *data)
{
    V4L2FrameDescriptor *desc = (V4L2FrameDescriptor *)data;

    av_log(NULL, AV_LOG_DEBUG, "%s: frame=%p desc=%p\n", __func__, opaque, desc);

    av_buffer_unref(&desc->decoder_ref);

    av_free(desc);
}

static int v4l2request_receive_frame(AVCodecContext *avctx, AVFrame *frame)
{
    V4L2DecoderContext *ctx = avctx->priv_data;
    V4L2Decoder *decoder = (V4L2Decoder *)ctx->decoder_ref->data;
    AVPacket pkt = {0};
    int ret;

    av_log(avctx, AV_LOG_DEBUG, "%s: avctx=%p decoder=%p frame=%p\n", __func__, avctx, decoder, frame);

    ret = ff_decode_get_packet(avctx, &pkt);
    if (ret < 0/* && ret != AVERROR_EOF*/) {
        if (ret != AVERROR_EOF && ret != AVERROR(EAGAIN))
            av_log(avctx, AV_LOG_DEBUG, "%s: get_packet failed, avctx=%p ret=%d\n", __func__, avctx, ret);
        return ret;
    }

    // TODO: dequeue capture buffer and return frame descriptor

    {
        V4L2FrameDescriptor *desc;
        desc = av_mallocz(sizeof(V4L2FrameDescriptor));
        if (!desc) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }

        av_log(avctx, AV_LOG_DEBUG, "%s: frame=%p desc=%p\n", __func__, frame, desc);

        //drm->nb_objects = 1;
        //drm->objects[0].fd = mpp_buffer_get_fd(buffer);
        //drm->objects[0].size = mpp_buffer_get_size(buffer);

        //drm->nb_layers = 1;
        //layer = &drm->layers[0];
        //layer->format = drmformat;
        //layer->nb_planes = 2;

        //layer->planes[0].object_index = 0;
        //layer->planes[0].offset = 0;
        //layer->planes[0].pitch = mpp_frame_get_hor_stride(mppframe);

        //layer->planes[1].object_index = 0;
        //layer->planes[1].offset = layer->planes[0].pitch * mpp_frame_get_ver_stride(mppframe);
        //layer->planes[1].pitch = layer->planes[0].pitch;

        desc->decoder_ref = av_buffer_ref(ctx->decoder_ref);
        frame->data[0] = (uint8_t *)desc;
        frame->buf[0] = av_buffer_create((uint8_t *)desc, sizeof(*desc), v4l2request_release_frame,
                                         frame, AV_BUFFER_FLAG_READONLY);

        // HACK:
        frame->format = avctx->pix_fmt;
        frame->width = avctx->width;
        frame->height = avctx->height;
        frame->sample_aspect_ratio = avctx->sample_aspect_ratio;
        frame->pts = AV_NOPTS_VALUE;
        frame->pkt_dts = AV_NOPTS_VALUE;
    }

fail:
    av_packet_unref(&pkt);
    return ret;
}

static void v4l2request_flush(AVCodecContext *avctx)
{
    av_log(avctx, AV_LOG_DEBUG, "%s: avctx=%p\n", __func__, avctx);
}

static const AVCodecHWConfigInternal *v4l2request_hw_configs[] = {
    HW_CONFIG_INTERNAL(DRM_PRIME),
    NULL
};

#define V4L2REQUEST_DEC_CLASS(NAME) \
    static const AVClass v4l2request_##NAME##_dec_class = { \
        .class_name = "v4l2request_" #NAME "_dec", \
        .item_name  = av_default_item_name, \
        .version    = LIBAVUTIL_VERSION_INT, \
    };

#define V4L2REQUEST_DEC(NAME, ID, BSFS) \
    V4L2REQUEST_DEC_CLASS(NAME) \
    AVCodec ff_##NAME##_v4l2request_decoder = { \
        .name           = #NAME "_v4l2request", \
        .long_name      = NULL_IF_CONFIG_SMALL(#NAME " (v4l2request)"), \
        .type           = AVMEDIA_TYPE_VIDEO, \
        .id             = ID, \
        .capabilities   = AV_CODEC_CAP_HARDWARE | AV_CODEC_CAP_DELAY | AV_CODEC_CAP_AVOID_PROBING, \
        .pix_fmts       = (const enum AVPixelFormat[]) { AV_PIX_FMT_DRM_PRIME, \
                                                         AV_PIX_FMT_NONE }, \
        .priv_class     = &v4l2request_##NAME##_dec_class, \
        .wrapper_name   = "v4l2request", \
        .priv_data_size = sizeof(V4L2DecoderContext), \
        .init           = v4l2request_init, \
        .close          = v4l2request_close, \
        .receive_frame  = v4l2request_receive_frame, \
        .flush          = v4l2request_flush, \
        .bsfs           = BSFS, \
        .hw_configs     = v4l2request_hw_configs, \
    };

V4L2REQUEST_DEC(h264,   AV_CODEC_ID_H264,       "h264_mp4toannexb")
V4L2REQUEST_DEC(hevc,   AV_CODEC_ID_HEVC,       "hevc_mp4toannexb")
V4L2REQUEST_DEC(mpeg1,  AV_CODEC_ID_MPEG1VIDEO, NULL)
V4L2REQUEST_DEC(mpeg2,  AV_CODEC_ID_MPEG2VIDEO, NULL)
V4L2REQUEST_DEC(mpeg4,  AV_CODEC_ID_MPEG4,      NULL)
V4L2REQUEST_DEC(h263,   AV_CODEC_ID_H263,       NULL)
V4L2REQUEST_DEC(vc1 ,   AV_CODEC_ID_VC1,        NULL)
V4L2REQUEST_DEC(vp8,    AV_CODEC_ID_VP8,        NULL)
V4L2REQUEST_DEC(vp9,    AV_CODEC_ID_VP9,        NULL)
