// -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
//
// Quick hack based on ffmpeg
// tutorial http://dranger.com/ffmpeg/tutorial01.html
// in turn based on a tutorial by
// Martin Bohme (boehme@inb.uni-luebeckREMOVETHIS.de)
//
// HELP NEEDED
// Note, this is known to not be optimal, causing flicker etc. It is at this
// point merely a demonstration of what is possible. It also serves as a
// converter to a 'stream' (-O option) which then can be played quickly with
// the led-image-viewer.
//
// Pull requests are welcome to address
//    * Use hardware acceleration if possible. The Pi does have some
//      acceleration features IIRC, so if we could use these, that would be
//      great.
//    * Other improvements that could reduce the flicker on a Raspberry Pi.
//      Currently it seems to create flicker in particular when decoding larger
//      videos due to memory bandwidth overload (?). Might already be fixed
//      with using hardware acceleration.
//    * Add sound ? Right now, we don't decode the sound. It is usually
//      not very useful as the builtin-sound is disabled when running the
//      LED matrix, but if there is an external USB sound adapter, it might
//      be nice.


// Ancient AV versions forgot to set this.
#define __STDC_CONSTANT_MACROS

// libav: "U NO extern C in header ?"
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <libavdevice/avdevice.h>
#include <libswresample/swresample.h>
}

#include <SDL2/SDL.h>
#include <SDL2/SDL_thread.h>

#include <assert.h>
#include <fcntl.h>
#include <getopt.h>
#include <limits.h>
#include <signal.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#include <thread>

#include "led-matrix.h"
#include "content-streamer.h"

using rgb_matrix::FrameCanvas;
using rgb_matrix::RGBMatrix;
using rgb_matrix::StreamWriter;
using rgb_matrix::StreamIO;

/* Prevents SDL from overriding main() */
#ifdef __MINGW32__
#undef main
#endif

#define SDL_AUDIO_BUFFER_SIZE 1024
#define MAX_AUDIO_FRAME_SIZE 192000

/**
 * PacketQueue Structure Declaration.
 */
typedef struct PacketQueue
{
    AVPacketList * first_pkt;
    AVPacketList * last_pkt;
    int nb_packets;
    int size;
    SDL_mutex * mutex;
    SDL_cond * cond;
} PacketQueue;

// audio PacketQueue instance
PacketQueue audioq;

// global quit flag
int quit = 0;

void packet_queue_init(PacketQueue * q);

int packet_queue_put(
        PacketQueue * queue,
        AVPacket * packet
    );

static int packet_queue_get(
              PacketQueue * q,
              AVPacket * pkt,
              int block
           );

void audio_callback(
        void * userdata,
        Uint8 * stream,
        int len
     );

int audio_decode_frame(
        AVCodecContext * aCodecCtx,
        uint8_t * audio_buf,
        int buf_size
    );

static int audio_resampling(
              AVCodecContext * audio_decode_ctx,
              AVFrame * audio_decode_frame,
              enum AVSampleFormat out_sample_fmt,
              int out_channels,
              int out_sample_rate,
              uint8_t * out_buf
          );


volatile bool interrupt_received = false;
static void InterruptHandler(int) {
  interrupt_received = true;
}

struct LedPixel {
  uint8_t r, g, b;
};
void CopyFrame(AVFrame *pFrame, FrameCanvas *canvas,
               int offset_x, int offset_y,
               int width, int height) {
  for (int y = 0; y < height; ++y) {
    LedPixel *pix = (LedPixel*) (pFrame->data[0] + y*pFrame->linesize[0]);
    for (int x = 0; x < width; ++x, ++pix) {
      canvas->SetPixel(x + offset_x, y + offset_y, pix->r, pix->g, pix->b);
    }
  }
}

// Scale "width" and "height" to fit within target rectangle of given size.
void ScaleToFitKeepAspect(int fit_in_width, int fit_in_height,
                          int *width, int *height) {
  if (*height < fit_in_height && *width < fit_in_width) return; // Done.
  const float height_ratio = 1.0 * (*height) / fit_in_height;
  const float width_ratio  = 1.0 * (*width) / fit_in_width;
  const float ratio = (height_ratio > width_ratio) ? height_ratio : width_ratio;
  *width = roundf(*width / ratio);
  *height = roundf(*height / ratio);
}

static int usage(const char *progname, const char *msg = NULL) {
  if (msg) {
    fprintf(stderr, "%s\n", msg);
  }
  fprintf(stderr, "Show one or a sequence of video files on the RGB-Matrix\n");
  fprintf(stderr, "usage: %s [options] <video> [<video>...]\n", progname);
  fprintf(stderr, "Options:\n"
          "\t-F                 : Full screen without black bars; aspect ratio might suffer\n"
          "\t-O<streamfile>     : Output to stream-file instead of matrix (don't need to be root).\n"
          "\t-s <count>         : Skip these number of frames in the beginning.\n"
          "\t-c <count>         : Only show this number of frames (excluding skipped frames).\n"
          "\t-V<vsync-multiple> : Instead of native video framerate, playback framerate\n"
          "\t                     is a fraction of matrix refresh. In particular with a stable refresh,\n"
          "\t                     this can result in more smooth playback. Choose multiple for desired framerate.\n"
          "\t                     (Tip: use --led-limit-refresh for stable rate)\n"
	  "\t-T <threads>       : Number of threads used to decode (default 1, max=%d)\n"
          "\t-v                 : verbose; prints video metadata and other info.\n"
          "\t-f                 : Loop forever.\n",
	  (int)std::thread::hardware_concurrency());

  fprintf(stderr, "\nGeneral LED matrix options:\n");
  rgb_matrix::PrintMatrixFlags(stderr);
  return 1;
}

static void add_nanos(struct timespec *accumulator, long nanoseconds) {
  accumulator->tv_nsec += nanoseconds;
  while (accumulator->tv_nsec > 1000000000) {
    accumulator->tv_nsec -= 1000000000;
    accumulator->tv_sec += 1;
  }
}

// Convert deprecated color formats to new and manually set the color range.
// YUV has funny ranges (16-235), while the YUVJ are 0-255. SWS prefers to
// deal with the YUV range, but then requires to set the output range.
// https://libav.org/documentation/doxygen/master/pixfmt_8h.html#a9a8e335cf3be472042bc9f0cf80cd4c5
SwsContext *CreateSWSContext(const AVCodecContext *codec_ctx,
                             int display_width, int display_height) {
  AVPixelFormat pix_fmt;
  bool src_range_extended_yuvj = true;
  // Remap deprecated to new pixel format.
  switch (codec_ctx->pix_fmt) {
  case AV_PIX_FMT_YUVJ420P: pix_fmt = AV_PIX_FMT_YUV420P; break;
  case AV_PIX_FMT_YUVJ422P: pix_fmt = AV_PIX_FMT_YUV422P; break;
  case AV_PIX_FMT_YUVJ444P: pix_fmt = AV_PIX_FMT_YUV444P; break;
  case AV_PIX_FMT_YUVJ440P: pix_fmt = AV_PIX_FMT_YUV440P; break;
  default:
    src_range_extended_yuvj = false;
    pix_fmt = codec_ctx->pix_fmt;
  }
  SwsContext *swsCtx = sws_getContext(codec_ctx->width, codec_ctx->height,
                                      pix_fmt,
                                      display_width, display_height,
                                      AV_PIX_FMT_RGB24, SWS_BILINEAR,
                                      NULL, NULL, NULL);
  if (src_range_extended_yuvj) {
    // Manually set the source range to be extended. Read modify write.
    int dontcare[4];
    int src_range, dst_range;
    int brightness, contrast, saturation;
    sws_getColorspaceDetails(swsCtx, (int**)&dontcare, &src_range,
                             (int**)&dontcare, &dst_range, &brightness,
                             &contrast, &saturation);
    const int* coefs = sws_getCoefficients(SWS_CS_DEFAULT);
    src_range = 1;  // New src range.
    sws_setColorspaceDetails(swsCtx, coefs, src_range, coefs, dst_range,
                             brightness, contrast, saturation);
  }
  return swsCtx;
}

int main(int argc, char *argv[]) {
  RGBMatrix::Options matrix_options;
  rgb_matrix::RuntimeOptions runtime_opt;
  // If started with 'sudo': make sure to drop privileges to same user
  // we started with, which is the most expected (and allows us to read
  // files as that user).
  runtime_opt.drop_priv_user = getenv("SUDO_UID");
  runtime_opt.drop_priv_group = getenv("SUDO_GID");
  if (!rgb_matrix::ParseOptionsFromFlags(&argc, &argv,
                                         &matrix_options, &runtime_opt)) {
    return usage(argv[0]);
  }

  int vsync_multiple = 1;
  bool use_vsync_for_frame_timing = false;
  bool maintain_aspect_ratio = true;
  bool verbose = false;
  bool forever = false;
  unsigned thread_count = 1;
  int stream_output_fd = -1;
  unsigned int frame_skip = 0;
  int64_t framecount_limit = INT64_MAX;

  int opt;
  while ((opt = getopt(argc, argv, "vO:R:Lfc:s:FV:T:")) != -1) {
    switch (opt) {
    case 'v':
      verbose = true;
      break;
    case 'f':
      forever = true;
      break;
    case 'O':
      stream_output_fd = open(optarg, O_CREAT|O_TRUNC|O_WRONLY, 0644);
      if (stream_output_fd < 0) {
        perror("Couldn't open output stream");
        return 1;
      }
      break;
    case 'L':
      fprintf(stderr, "-L is deprecated. Use\n\t--led-pixel-mapper=\"U-mapper\" --led-chain=4\ninstead.\n");
      return 1;
      break;
    case 'R':
      fprintf(stderr, "-R is deprecated. "
              "Use --led-pixel-mapper=\"Rotate:%s\" instead.\n", optarg);
      return 1;
      break;
    case 'c':
      framecount_limit = atoll(optarg);
      break;
    case 's':
      frame_skip = atoi(optarg);
      break;
    case 'T':
      thread_count = atoi(optarg);
      break;
    case 'F':
      maintain_aspect_ratio = false;
      break;
    case 'V':
      vsync_multiple = atoi(optarg);
      if (vsync_multiple <= 0)
        return usage(argv[0],
                     "-V: VSync-multiple needs to be a positive integer");
      use_vsync_for_frame_timing = true;
      break;
    default:
      return usage(argv[0]);
    }
  }

  if (optind >= argc) {
    fprintf(stderr, "Expected video filename.\n");
    return usage(argv[0]);
  }

  const bool multiple_videos = (argc > optind + 1);

  // We want to have the matrix start unless we actually write to a stream.
  runtime_opt.do_gpio_init = (stream_output_fd < 0);
  RGBMatrix *matrix = RGBMatrix::CreateFromOptions(matrix_options, runtime_opt);
  if (matrix == NULL) {
    return 1;
  }
  FrameCanvas *offscreen_canvas = matrix->CreateFrameCanvas();

  long frame_count = 0;
  StreamIO *stream_io = NULL;
  StreamWriter *stream_writer = NULL;
  if (stream_output_fd >= 0) {
    stream_io = new rgb_matrix::FileStreamIO(stream_output_fd);
    stream_writer = new StreamWriter(stream_io);
    if (forever) {
      fprintf(stderr, "-f (forever) doesn't make sense with -O; disabling\n");
      forever = false;
    }
  }

  // If we only have to loop a single video, we can avoid doing the
  // expensive video stream set-up and just repeat in an inner loop.
  const bool one_video_forever = forever && !multiple_videos;
  const bool multiple_video_forever = forever && multiple_videos;

#if LIBAVFORMAT_VERSION_INT < AV_VERSION_INT(58, 9, 100)
  av_register_all();
#endif
  avdevice_register_all();
  avformat_network_init();

    // Initialize SDL.
    int ret = -1;
    ret = SDL_Init(SDL_INIT_AUDIO | SDL_INIT_TIMER);
    if (ret != 0)
    {
        printf("Could not initialize SDL - %s\n.", SDL_GetError());
        return -1;
    }

  signal(SIGTERM, InterruptHandler);
  signal(SIGINT, InterruptHandler);

  do {
    for (int m = optind; m < argc && !interrupt_received; ++m) {
      const char *movie_file = argv[m];
      if (strcmp(movie_file, "-") == 0) {
        movie_file = "/dev/stdin";
      }

      AVFormatContext *format_context = avformat_alloc_context();
      if (avformat_open_input(&format_context, movie_file, NULL, NULL) != 0) {
        perror("Issue opening file: ");
        return -1;
      }

      if (avformat_find_stream_info(format_context, NULL) < 0) {
        fprintf(stderr, "Couldn't find stream information\n");
        return -1;
      }

      if (verbose) av_dump_format(format_context, 0, movie_file, 0);

      // Find the first video stream
      int videoStream = -1;
      int audioStream = -1;

      AVCodecParameters *codec_parameters = NULL;
      const AVCodec *av_codec = NULL;
      for (int i = 0; i < (int)format_context->nb_streams; ++i) {
        codec_parameters = format_context->streams[i]->codecpar;
        av_codec = avcodec_find_decoder(codec_parameters->codec_id);
        if (!av_codec) continue;
        if (codec_parameters->codec_type == AVMEDIA_TYPE_VIDEO && videoStream < 0) {
          videoStream = i;
        }

        if (codec_parameters->codec_type == AVMEDIA_TYPE_AUDIO && audioStream < 0) {
          audioStream = i;
        }
      }
      if (videoStream == -1) {
        printf("Could not find video stream.\n");
        return false;
      }

      if (audioStream == -1) {
        printf("Could not find audio stream.\n");
        return false;
      }

      // Frames per second; calculate wait time between frames.
      AVStream *const stream = format_context->streams[videoStream];
      AVRational rate = av_guess_frame_rate(format_context, stream, NULL);
      const long frame_wait_nanos = 1e9 * rate.den / rate.num;
      if (verbose) fprintf(stderr, "FPS: %f\n", 1.0*rate.num / rate.den);

      // retrieve audio codec
      AVCodec * aCodec = NULL;
      aCodec = avcodec_find_decoder(format_context->streams[audioStream]->codecpar->codec_id);
      if (aCodec == NULL) {
          printf("Unsupported codec!\n");
          return -1;
      }

      // retrieve audio codec context
      AVCodecContext * aCodecCtx = NULL;
      aCodecCtx = avcodec_alloc_context3(aCodec);
      ret = avcodec_parameters_to_context(aCodecCtx, format_context->streams[audioStream]->codecpar);
      if (ret != 0) {
          printf("Could not copy codec context.\n");
          return -1;
      }

      // audio specs containers
      SDL_AudioSpec wanted_specs;
      SDL_AudioSpec specs;

      // set audio settings from codec info
      wanted_specs.freq = aCodecCtx->sample_rate;
      wanted_specs.format = AUDIO_S16SYS;
      wanted_specs.channels = aCodecCtx->channels;
      wanted_specs.silence = 0;
      wanted_specs.samples = SDL_AUDIO_BUFFER_SIZE;
      wanted_specs.callback = audio_callback;
      wanted_specs.userdata = aCodecCtx;

      // Uint32 audio device id
      SDL_AudioDeviceID audioDeviceID;

      // open audio device
      audioDeviceID = SDL_OpenAudioDevice( 
                            NULL,
                            0,
                            &wanted_specs,
                            &specs,
                            SDL_AUDIO_ALLOW_FORMAT_CHANGE
                        );

      // SDL_OpenAudioDevice returns a valid device ID that is > 0 on success or 0 on failure
      if (audioDeviceID == 0) {
          printf("Failed to open audio device: %s.\n", SDL_GetError());
          return -1;
      }

      // initialize the audio AVCodecContext to use the given audio AVCodec
      ret = avcodec_open2(aCodecCtx, aCodec, NULL);
      if (ret < 0) {
          printf("Could not open audio codec.\n");
          return -1;
      }

      // init audio PacketQueue
      packet_queue_init(&audioq);

      // start playing audio on the given audio device
      SDL_PauseAudioDevice(audioDeviceID, 0);

      AVCodecContext *codec_context = avcodec_alloc_context3(av_codec);
      if (thread_count > 1 &&
          av_codec->capabilities & AV_CODEC_CAP_FRAME_THREADS &&
          std::thread::hardware_concurrency() > 1) {
        codec_context->thread_type = FF_THREAD_FRAME;
        codec_context->thread_count =
          std::min(thread_count, std::thread::hardware_concurrency());
      }

      if (avcodec_parameters_to_context(codec_context, codec_parameters) < 0)
        return -1;
      if (avcodec_open2(codec_context, av_codec, NULL) < 0)
        return -1;

      /*
       * Prepare frame to hold the scaled target frame to be send to matrix.
       */
      int display_width = codec_context->width;
      int display_height = codec_context->height;
      if (maintain_aspect_ratio) {
        display_width = codec_context->width;
        display_height = codec_context->height;
        // Make display fit within canvas.
        ScaleToFitKeepAspect(matrix->width(), matrix->height(),
                             &display_width, &display_height);
      } else {
        display_width = matrix->width();
        display_height = matrix->height();
      }
      // Letterbox or pillarbox black bars.
      const int display_offset_x = (matrix->width() - display_width)/2;
      const int display_offset_y = (matrix->height() - display_height)/2;

      // The output_frame_ will receive the scaled result.
      AVFrame *output_frame = av_frame_alloc();
      if (av_image_alloc(output_frame->data, output_frame->linesize,
                         display_width, display_height, AV_PIX_FMT_RGB24,
                         64) < 0) {
        return -1;
      }

      if (verbose) {
        fprintf(stderr, "Scaling %dx%d -> %dx%d; black border x:%d y:%d\n",
                codec_context->width, codec_context->height,
                display_width, display_height,
                display_offset_x, display_offset_y);
      }

      // initialize SWS context for software scaling
      SwsContext *const sws_ctx = CreateSWSContext(
        codec_context, display_width, display_height);
      if (!sws_ctx) {
        fprintf(stderr, "Trouble doing scaling to %dx%d :(\n",
                matrix->width(), matrix->height());
        return 1;
      }


      struct timespec next_frame;

      AVPacket *packet = av_packet_alloc();
      AVFrame *decode_frame = av_frame_alloc();  // Decode video into this
      do {
        int64_t frames_left = framecount_limit;
        unsigned int frames_to_skip = frame_skip;
        if (one_video_forever) {
          av_seek_frame(format_context, videoStream, 0, AVSEEK_FLAG_ANY);
          avcodec_flush_buffers(codec_context);
        }
        clock_gettime(CLOCK_MONOTONIC, &next_frame);

        int decode_in_flight = 0;
        bool state_reading = true;

        while (!interrupt_received && frames_left > 0) {
          if (state_reading &&
              av_read_frame(format_context, packet) != 0) {
            state_reading = false;  // ran out of packets from input
          }

          if (!state_reading && decode_in_flight == 0)
            break;  // Decoder fully drained.

          // Is this a packet from the video stream?
          if (state_reading && packet->stream_index != videoStream) {
            if (packet->stream_index == audioStream) {
              // put the AVPacket in the audio PacketQueue
              packet_queue_put(&audioq, packet);
            } else {
              av_packet_unref(packet);
            }
            continue;  // Not interested in that.
          }

          if (state_reading) {
            // Decode video frame
            if (avcodec_send_packet(codec_context, packet) == 0) {
              ++decode_in_flight;
            }
            av_packet_unref(packet);
          } else {
            avcodec_send_packet(codec_context, nullptr); // Trigger decode drain
          }

          while (decode_in_flight &&
                 avcodec_receive_frame(codec_context, decode_frame) == 0) {
            --decode_in_flight;

            if (frames_to_skip) { frames_to_skip--; continue; }

            // Determine absolute end of this frame now so that we don't include
            // decoding overhead. TODO: skip frames if getting too slow ?
            add_nanos(&next_frame, frame_wait_nanos);

            // Convert the image from its native format to RGB
            sws_scale(sws_ctx, (uint8_t const * const *)decode_frame->data,
                      decode_frame->linesize, 0, codec_context->height,
                      output_frame->data, output_frame->linesize);
            CopyFrame(output_frame, offscreen_canvas,
                      display_offset_x, display_offset_y,
                      display_width, display_height);
            frame_count++;
            frames_left--;
            if (stream_writer) {
              if (verbose) fprintf(stderr, "%6ld", frame_count);
              stream_writer->Stream(*offscreen_canvas, frame_wait_nanos/1000);
            } else {
              offscreen_canvas = matrix->SwapOnVSync(offscreen_canvas,
                                                     vsync_multiple);
            }
            if (!stream_writer && !use_vsync_for_frame_timing) {
              clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_frame, NULL);
            }
          }
        }
      } while (one_video_forever && !interrupt_received);

      av_packet_free(&packet);

      av_frame_free(&output_frame);
      av_frame_free(&decode_frame);
      avcodec_close(codec_context);
      avformat_close_input(&format_context);
    }
  } while (multiple_video_forever && !interrupt_received);

  if (interrupt_received) {
    // Feedback for Ctrl-C, but most importantly, force a newline
    // at the output, so that commandline-shell editing is not messed up.
    fprintf(stderr, "Got interrupt. Exiting\n");
  }

  delete matrix;
  delete stream_writer;
  delete stream_io;
  fprintf(stderr, "Total of %ld frames decoded\n", frame_count);

  return 0;
}

/**
 * Initialize the given PacketQueue.
 *
 * @param   q   the PacketQueue to be initialized.
 */
void packet_queue_init(PacketQueue * q)
{
    // alloc memory for the audio queue
    memset(
        q,
        0,
        sizeof(PacketQueue)
      );

    // Returns the initialized and unlocked mutex or NULL on failure
    q->mutex = SDL_CreateMutex();
    if (!q->mutex)
    {
        // could not create mutex
        printf("SDL_CreateMutex Error: %s.\n", SDL_GetError());
        return;
    }

    // Returns a new condition variable or NULL on failure
    q->cond = SDL_CreateCond();
    if (!q->cond)
    {
        // could not create condition variable
        printf("SDL_CreateCond Error: %s.\n", SDL_GetError());
        return;
    }
}


/**
 * Put the given AVPacket in the given PacketQueue.
 *
 * @param   q   the queue to be used for the insert
 * @param   pkt the AVPacket to be inserted in the queue
 *
 * @return      0 if the AVPacket is correctly inserted in the given PacketQueue.
 */
int packet_queue_put(PacketQueue * q, AVPacket * pkt)
{
    /* [4]
     * if (av_dup_packet(pkt) < 0) { return -1; }
     */

    // alloc the new AVPacketList to be inserted in the audio PacketQueue
    AVPacketList * avPacketList;
    avPacketList = (AVPacketList*)av_malloc(sizeof(AVPacketList));

    // check the AVPacketList was allocated
    if (!avPacketList)
    {
        return -1;
    }

    // add reference to the given AVPacket
    avPacketList->pkt = * pkt;

    // the new AVPacketList will be inserted at the end of the queue
    avPacketList->next = NULL;

    // lock mutex
    SDL_LockMutex(q->mutex);

    // check the queue is empty
    if (!q->last_pkt)
    {
        // if it is, insert as first
        q->first_pkt = avPacketList;
    }
    else
    {
        // if not, insert as last
        q->last_pkt->next = avPacketList;
    }

    // point the last AVPacketList in the queue to the newly created AVPacketList
    q->last_pkt = avPacketList;

    // increase by 1 the number of AVPackets in the queue
    q->nb_packets++;

    // increase queue size by adding the size of the newly inserted AVPacket
    q->size += avPacketList->pkt.size;

    // notify packet_queue_get which is waiting that a new packet is available
    SDL_CondSignal(q->cond);

    // unlock mutex
    SDL_UnlockMutex(q->mutex);

    return 0;
}

/**
 * Get the first AVPacket from the given PacketQueue.
 *
 * @param   q       the PacketQueue to extract from
 * @param   pkt     the first AVPacket extracted from the queue
 * @param   block   0 to avoid waiting for an AVPacket to be inserted in the given
 *                  queue, != 0 otherwise.
 *
 * @return          < 0 if returning because the quit flag is set, 0 if the queue
 *                  is empty, 1 if it is not empty and a packet was extract (pkt)
 */
static int packet_queue_get(PacketQueue * q, AVPacket * pkt, int block)
{
    int ret;

    AVPacketList * avPacketList;

    // lock mutex
    SDL_LockMutex(q->mutex);

    for (;;)
    {
        // check quit flag
        if (quit)
        {
            ret = -1;
            break;
        }

        // point to the first AVPacketList in the queue
        avPacketList = q->first_pkt;

        // if the first packet is not NULL, the queue is not empty
        if (avPacketList)
        {
            // place the second packet in the queue at first position
            q->first_pkt = avPacketList->next;

            // check if queue is empty after removal
            if (!q->first_pkt)
            {
                // first_pkt = last_pkt = NULL = empty queue
                q->last_pkt = NULL;
            }

            // decrease the number of packets in the queue
            q->nb_packets--;

            // decrease the size of the packets in the queue
            q->size -= avPacketList->pkt.size;

            // point pkt to the extracted packet, this will return to the calling function
            *pkt = avPacketList->pkt;

            // free memory
            av_free(avPacketList);

            ret = 1;
            break;
        }
        else if (!block)
        {
            ret = 0;
            break;
        }
        else
        {
            // unlock mutex and wait for cond signal, then lock mutex again
            SDL_CondWait(q->cond, q->mutex);
        }
    }

    // unlock mutex
    SDL_UnlockMutex(q->mutex);

    return ret;
}

/**
 * Pull in data from audio_decode_frame(), store the result in an intermediary
 * buffer, attempt to write as many bytes as the amount defined by len to
 * stream, and get more data if we don't have enough yet, or save it for later
 * if we have some left over.
 *
 * @param   userdata    the pointer we gave to SDL.
 * @param   stream      the buffer we will be writing audio data to.
 * @param   len         the size of that buffer.
 */
void audio_callback(void * userdata, Uint8 * stream, int len)
{
    // retrieve the audio codec context
    AVCodecContext * aCodecCtx = (AVCodecContext *) userdata;

    int len1 = -1;
    int audio_size = -1;

    // The size of audio_buf is 1.5 times the size of the largest audio frame
    // that FFmpeg will give us, which gives us a nice cushion.
    static uint8_t audio_buf[(MAX_AUDIO_FRAME_SIZE * 3) / 2];
    static unsigned int audio_buf_size = 0;
    static unsigned int audio_buf_index = 0;

    while (len > 0)
    {
        // check global quit flag
        if (quit)
        {
            return;
        }

        if (audio_buf_index >= audio_buf_size)
        {
            // we have already sent all avaialble data; get more
            audio_size = audio_decode_frame(aCodecCtx, audio_buf, sizeof(audio_buf));

            // if error
            if (audio_size < 0)
            {
                // output silence
                audio_buf_size = 1024;

                // clear memory
                memset(audio_buf, 0, audio_buf_size);
                printf("audio_decode_frame() failed.\n");
            }
            else
            {
                audio_buf_size = audio_size;
            }

            audio_buf_index = 0;
        }

        len1 = audio_buf_size - audio_buf_index;

        if (len1 > len)
        {
            len1 = len;
        }

        // copy data from audio buffer to the SDL stream
        memcpy(stream, (uint8_t *)audio_buf + audio_buf_index, len1);

        len -= len1;
        stream += len1;
        audio_buf_index += len1;
    }
}

/**
 * Get a packet from the queue if available. Decode the extracted packet. Once
 * we have the frame, resample it and simply copy it to our audio buffer, making
 * sure the data_size is smaller than our audio buffer.
 *
 * @param   aCodecCtx   the audio AVCodecContext used for decoding
 * @param   audio_buf   the audio buffer to write into
 * @param   buf_size    the size of the audio buffer, 1.5 larger than the one
 *                      provided by FFmpeg
 *
 * @return              0 if everything goes well, -1 in case of error or quit
 */
int audio_decode_frame(AVCodecContext * aCodecCtx, uint8_t * audio_buf, int buf_size)
{
    AVPacket * avPacket = av_packet_alloc();
    static uint8_t * audio_pkt_data = NULL;
    static int audio_pkt_size = 0;

    // allocate a new frame, used to decode audio packets
    static AVFrame * avFrame = NULL;
    avFrame = av_frame_alloc();
    if (!avFrame)
    {
        printf("Could not allocate AVFrame.\n");
        return -1;
    }

    int len1 = 0;
    int data_size = 0;

    for (;;)
    {
        // check global quit flag
        if (quit)
        {
            return -1;
        }

        while (audio_pkt_size > 0)
        {
            int got_frame = 0;

            // [5]
            // len1 = avcodec_decode_audio4(aCodecCtx, avFrame, &got_frame, avPacket);
            int ret = avcodec_receive_frame(aCodecCtx, avFrame);
            if (ret == 0)
            {
                got_frame = 1;
            }
            if (ret == AVERROR(EAGAIN))
            {
                ret = 0;
            }
            if (ret == 0)
            {
                ret = avcodec_send_packet(aCodecCtx, avPacket);
            }
            if (ret == AVERROR(EAGAIN))
            {
                ret = 0;
            }
            else if (ret < 0)
            {
                printf("avcodec_receive_frame error");
                return -1;
            }
            else
            {
                len1 = avPacket->size;
            }

            if (len1 < 0)
            {
                // if error, skip frame
                audio_pkt_size = 0;
                break;
            }

            audio_pkt_data += len1;
            audio_pkt_size -= len1;
            data_size = 0;

            if (got_frame)
            {
                // audio resampling
                data_size = audio_resampling(
                                aCodecCtx,
                                avFrame,
                                AV_SAMPLE_FMT_S16,
                                aCodecCtx->channels,
                                aCodecCtx->sample_rate,
                                audio_buf
                            );

                assert(data_size <= buf_size);
            }

            if (data_size <= 0)
            {
                // no data yet, get more frames
                continue;
            }

            // we have the data, return it and come back for more later
            return data_size;
        }

        if (avPacket->data)
        {
            // wipe the packet
            av_packet_unref(avPacket);
        }

        // get more audio AVPacket
        int ret = packet_queue_get(&audioq, avPacket, 1);

        // if packet_queue_get returns < 0, the global quit flag was set
        if (ret < 0)
        {
            return -1;
        }

        audio_pkt_data = avPacket->data;
        audio_pkt_size = avPacket->size;
    }

    return 0;
}

/**
 * Resample the audio data retrieved using FFmpeg before playing it.
 *
 * @param   audio_decode_ctx    the audio codec context retrieved from the original AVFormatContext.
 * @param   decoded_audio_frame the decoded audio frame.
 * @param   out_sample_fmt      audio output sample format (e.g. AV_SAMPLE_FMT_S16).
 * @param   out_channels        audio output channels, retrieved from the original audio codec context.
 * @param   out_sample_rate     audio output sample rate, retrieved from the original audio codec context.
 * @param   out_buf             audio output buffer.
 *
 * @return                      the size of the resampled audio data.
 */
static int audio_resampling(                                  // 1
                        AVCodecContext * audio_decode_ctx,
                        AVFrame * decoded_audio_frame,
                        enum AVSampleFormat out_sample_fmt,
                        int out_channels,
                        int out_sample_rate,
                        uint8_t * out_buf
                      )
{
    // check global quit flag
    if (quit)
    {
        return -1;
    }

    SwrContext * swr_ctx = NULL;
    int ret = 0;
    int64_t in_channel_layout = audio_decode_ctx->channel_layout;
    int64_t out_channel_layout = AV_CH_LAYOUT_STEREO;
    int out_nb_channels = 0;
    int out_linesize = 0;
    int in_nb_samples = 0;
    int out_nb_samples = 0;
    int max_out_nb_samples = 0;
    uint8_t ** resampled_data = NULL;
    int resampled_data_size = 0;

    swr_ctx = swr_alloc();

    if (!swr_ctx)
    {
        printf("swr_alloc error.\n");
        return -1;
    }

    // get input audio channels
    in_channel_layout = (audio_decode_ctx->channels ==
                     av_get_channel_layout_nb_channels(audio_decode_ctx->channel_layout)) ?   // 2
                     audio_decode_ctx->channel_layout :
                     av_get_default_channel_layout(audio_decode_ctx->channels);

    // check input audio channels correctly retrieved
    if (in_channel_layout <= 0)
    {
        printf("in_channel_layout error.\n");
        return -1;
    }

    // set output audio channels based on the input audio channels
    if (out_channels == 1)
    {
        out_channel_layout = AV_CH_LAYOUT_MONO;
    }
    else if (out_channels == 2)
    {
        out_channel_layout = AV_CH_LAYOUT_STEREO;
    }
    else
    {
        out_channel_layout = AV_CH_LAYOUT_SURROUND;
    }

    // retrieve number of audio samples (per channel)
    in_nb_samples = decoded_audio_frame->nb_samples;
    if (in_nb_samples <= 0)
    {
        printf("in_nb_samples error.\n");
        return -1;
    }

    // Set SwrContext parameters for resampling
    av_opt_set_int(   // 3
        swr_ctx,
        "in_channel_layout",
        in_channel_layout,
        0
    );

    // Set SwrContext parameters for resampling
    av_opt_set_int(
        swr_ctx,
        "in_sample_rate",
        audio_decode_ctx->sample_rate,
        0
    );

    // Set SwrContext parameters for resampling
    av_opt_set_sample_fmt(
        swr_ctx,
        "in_sample_fmt",
        audio_decode_ctx->sample_fmt,
        0
    );

    // Set SwrContext parameters for resampling
    av_opt_set_int(
        swr_ctx,
        "out_channel_layout",
        out_channel_layout,
        0
    );

    // Set SwrContext parameters for resampling
    av_opt_set_int(
        swr_ctx,
        "out_sample_rate",
        out_sample_rate,
        0
    );

    // Set SwrContext parameters for resampling
    av_opt_set_sample_fmt(
        swr_ctx,
        "out_sample_fmt",
        out_sample_fmt,
        0
    );

    // Once all values have been set for the SwrContext, it must be initialized
    // with swr_init().
    ret = swr_init(swr_ctx);;
    if (ret < 0)
    {
        printf("Failed to initialize the resampling context.\n");
        return -1;
    }

    max_out_nb_samples = out_nb_samples = av_rescale_rnd(
                                              in_nb_samples,
                                              out_sample_rate,
                                              audio_decode_ctx->sample_rate,
                                              AV_ROUND_UP
                                          );

    // check rescaling was successful
    if (max_out_nb_samples <= 0)
    {
        printf("av_rescale_rnd error.\n");
        return -1;
    }

    // get number of output audio channels
    out_nb_channels = av_get_channel_layout_nb_channels(out_channel_layout);

    ret = av_samples_alloc_array_and_samples(
              &resampled_data,
              &out_linesize,
              out_nb_channels,
              out_nb_samples,
              out_sample_fmt,
              0
          );

    if (ret < 0)
    {
        printf("av_samples_alloc_array_and_samples() error: Could not allocate destination samples.\n");
        return -1;
    }

    // retrieve output samples number taking into account the progressive delay
    out_nb_samples = av_rescale_rnd(
                        swr_get_delay(swr_ctx, audio_decode_ctx->sample_rate) + in_nb_samples,
                        out_sample_rate,
                        audio_decode_ctx->sample_rate,
                        AV_ROUND_UP
                     );

    // check output samples number was correctly retrieved
    if (out_nb_samples <= 0)
    {
        printf("av_rescale_rnd error\n");
        return -1;
    }

    if (out_nb_samples > max_out_nb_samples)
    {
        // free memory block and set pointer to NULL
        av_free(resampled_data[0]);

        // Allocate a samples buffer for out_nb_samples samples
        ret = av_samples_alloc(
                  resampled_data,
                  &out_linesize,
                  out_nb_channels,
                  out_nb_samples,
                  out_sample_fmt,
                  1
              );

        // check samples buffer correctly allocated
        if (ret < 0)
        {
            printf("av_samples_alloc failed.\n");
            return -1;
        }

        max_out_nb_samples = out_nb_samples;
    }

    if (swr_ctx)
    {
        // do the actual audio data resampling
        ret = swr_convert(
                  swr_ctx,
                  resampled_data,
                  out_nb_samples,
                  (const uint8_t **) decoded_audio_frame->data,
                  decoded_audio_frame->nb_samples
              );

        // check audio conversion was successful
        if (ret < 0)
        {
            printf("swr_convert_error.\n");
            return -1;
        }

        // Get the required buffer size for the given audio parameters
        resampled_data_size = av_samples_get_buffer_size(
                                  &out_linesize,
                                  out_nb_channels,
                                  ret,
                                  out_sample_fmt,
                                  1
                              );

        // check audio buffer size
        if (resampled_data_size < 0)
        {
            printf("av_samples_get_buffer_size error.\n");
            return -1;
        }
    }
    else
    {
        printf("swr_ctx null error.\n");
        return -1;
    }

    // copy the resampled data to the output buffer
    memcpy(out_buf, resampled_data[0], resampled_data_size);

    /*
     * Memory Cleanup.
     */
    if (resampled_data)
    {
        // free memory block and set pointer to NULL
        av_freep(&resampled_data[0]);
    }

    av_freep(&resampled_data);
    resampled_data = NULL;

    if (swr_ctx)
    {
        // Free the given SwrContext and set the pointer to NULL
        swr_free(&swr_ctx);
    }

    return resampled_data_size;
}
