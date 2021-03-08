/**
 * Copyright (C) 2020 Parrot Drones SAS
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <signal.h>
#include <math.h>
#include <unistd.h>

#define ULOG_TAG ms_sample
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include <libpomp.h>
#include <libtelemetry.h>
#include <video-ipc/vipc_client.h>
#include <video-ipc/vipc_client_cfg.h>
#include <opencv2/opencv.hpp>

#include <google/protobuf/empty.pb.h>
#include <samples/hello/cv-service/messages.msghub.h>

#define VIPC_DEPTH_MAP_STREAM	"fstcam_stereo_depth_filtered"
#define TLM_SECTION_USER	"drone_controller"
#define TLM_SECTION_OUT		"cv@hello"
#define TLM_SECTION_OUT_RATE	1000
#define TLM_SECTION_OUT_COUNT	10
#define MSGHUB_ADDR		"unix:/tmp/hello-cv-service"
#define CLOSE_DEPTH		0.8f /* [m] */
#define FAR_DEPTH		1.2f /* [m] */

struct tlm_data_in {
	struct timespec timestamp;

	struct {
		float x;
		float y;
		float z;
	} velocity;

	struct {
		float x;
		float y;
		float z;
	} position_absolute;

	struct {
		float yaw;
		float pitch;
		float roll;
	} attitude;
};

struct tlm_data_out {
	struct {
		float x, y, z;
		float depth_mean;
		float confidence;
	} algo;
};

class HelloServiceCommandHandler
	: public ::samples::hello::cv_service::messages::msghub::
		  CommandHandler {
public:
	inline HelloServiceCommandHandler() : mProcess(false) {}

	void setProcess(bool msg) override
	{
		mProcess = msg;
	}

	bool getProcess()
	{
		return mProcess;
	}
private:
	bool mProcess;
};

struct context {
	using HelloServiceEventSender =
		::samples::hello::cv_service::messages::msghub::EventSender;

	/* Main loop of the program */
	pomp::Loop loop;

	/* Consumer to get drone telemetry */
	struct tlm_consumer *consumer;

	/* Structure where to save telemetry data */
	struct tlm_data_in tlm_data_in;

	/* Producer to log some telemetry */
	struct tlm_producer *producer;

	/* Structure where to save telemetry data */
	struct tlm_data_out tlm_data_out;

	/* Video ipc client */
	struct vipcc_ctx *vipcc;

	/* Video ipc frame dimensions */
	struct vipc_dim frame_dim;

	/* Depth frame */
	cv::Mat depth_frame;

	/* Mask frame */
	cv::Mat mask_frame;

	/* Message hub */
	msghub::MessageHub *msg;

	/* Message hub channel */
	msghub::Channel *msg_channel;

	/* Message hub command handler */
	HelloServiceCommandHandler msg_cmd_handler;

	/* Message hub event sender */
	HelloServiceEventSender msg_evt_sender;

	/* Previous depth mean value */
	float previous_depth_mean;

	/* Close state */
	bool is_close;
};

static const struct tlm_reg_field s_tlm_data_in_fields[] = {
	TLM_REG_FIELD_SCALAR_EX(struct tlm_data_in, velocity.x,
			"linear_velocity_ned.x", TLM_TYPE_FLOAT32),
	TLM_REG_FIELD_SCALAR_EX(struct tlm_data_in, velocity.y,
			"linear_velocity_ned.y", TLM_TYPE_FLOAT32),
	TLM_REG_FIELD_SCALAR_EX(struct tlm_data_in, velocity.z,
			"linear_velocity_ned.z", TLM_TYPE_FLOAT32),

	TLM_REG_FIELD_SCALAR_EX(struct tlm_data_in, position_absolute.x,
			"position_absolute.x", TLM_TYPE_FLOAT32),
	TLM_REG_FIELD_SCALAR_EX(struct tlm_data_in, position_absolute.y,
			"position_absolute.y", TLM_TYPE_FLOAT32),
	TLM_REG_FIELD_SCALAR_EX(struct tlm_data_in, position_absolute.z,
			"position_absolute.z", TLM_TYPE_FLOAT32),

	TLM_REG_FIELD_SCALAR_EX(struct tlm_data_in, attitude.yaw,
			"attitude_euler_angles.yaw", TLM_TYPE_FLOAT32),
	TLM_REG_FIELD_SCALAR_EX(struct tlm_data_in, attitude.pitch,
			"attitude_euler_angles.pitch", TLM_TYPE_FLOAT32),
	TLM_REG_FIELD_SCALAR_EX(struct tlm_data_in, attitude.roll,
			"attitude_euler_angles.roll", TLM_TYPE_FLOAT32),
};

static const struct tlm_reg_field s_tlm_data_out_fields[] = {
	TLM_REG_FIELD_SCALAR(struct tlm_data_out, algo.x, TLM_TYPE_FLOAT32),
	TLM_REG_FIELD_SCALAR(struct tlm_data_out, algo.y, TLM_TYPE_FLOAT32),
	TLM_REG_FIELD_SCALAR(struct tlm_data_out, algo.z, TLM_TYPE_FLOAT32),
	TLM_REG_FIELD_SCALAR(struct tlm_data_out, algo.depth_mean,
			TLM_TYPE_FLOAT32),
	TLM_REG_FIELD_SCALAR(struct tlm_data_out, algo.confidence,
			TLM_TYPE_FLOAT32),
};

static const struct tlm_reg_struct s_tlm_data_in_struct =
	TLM_REG_STRUCT("tlm_data_in", s_tlm_data_in_fields);

static const struct tlm_reg_struct s_tlm_data_out_struct =
	TLM_REG_STRUCT("tlm_data_out", s_tlm_data_out_fields);

/* Global context (so the signal handler can access it */
static struct context s_ctx;
/* Stop flag, set to 1 by signal handler to exit cleanly */
static volatile int stop;


static void context_clean(struct context *ctx)
{
	int res = 0;

	delete ctx->msg;

	if (ctx->vipcc != NULL) {
		res = vipcc_destroy(ctx->vipcc);
		if (res < 0)
			ULOG_ERRNO("vipcc_destroy", -res);
	}

	if (ctx->producer != NULL) {
		res = tlm_producer_destroy(ctx->producer);
		if (res < 0)
			ULOG_ERRNO("tlm_producer_destroy", -res);
	}

	if (ctx->consumer != NULL) {
		res = tlm_consumer_destroy(ctx->consumer);
		if (res < 0)
			ULOG_ERRNO("tlm_consumer_destroy", -res);
	}
}

static int context_start(struct context *ctx)
{
	int res = 0;

	/* vipc */
	res = vipcc_start(ctx->vipcc);
	if (res < 0) {
		ULOG_ERRNO("vipc_start", -res);
		return res;
	}

	/* msg */
	ctx->msg_channel = ctx->msg->startServerChannel(
		pomp::Address(MSGHUB_ADDR), 0666);
	if (ctx->msg_channel == nullptr) {
		ULOGE("Failed to start server channel on '%s'", MSGHUB_ADDR);
		res = -EPERM;
	}

	ctx->msg->attachMessageHandler(&ctx->msg_cmd_handler);
	ctx->msg->attachMessageSender(&ctx->msg_evt_sender, ctx->msg_channel);

	return 0;
}

static int context_stop(struct context *ctx)
{
	int res = 0;

	/* msg */
	ctx->msg->detachMessageSender(&ctx->msg_evt_sender);
	ctx->msg->detachMessageHandler(&ctx->msg_cmd_handler);
	ctx->msg->stop();
	ctx->msg_channel = nullptr;

	/* vipc */
	res = vipcc_stop(ctx->vipcc);
	if (res < 0)
		ULOG_ERRNO("vipcc_stop", -res);

	return res;
}

static void status_cb(struct vipcc_ctx *ctx,
		      const struct vipc_status *st,
		      void *userdata)
{
	struct context *ud = (struct context *)userdata;

	for (unsigned int i = 0; i < st->num_planes; i++) {
		ULOGI("method %s, plane %d, w %d, h %d, stride %d",
			st->method,
			i,
			st->width,
			st->height,
			st->planes[i].stride);
	}

	ud->frame_dim.width = st->width;
	ud->frame_dim.height = st->height;
	ud->mask_frame =
		cv::Mat(ud->frame_dim.height, ud->frame_dim.width, CV_8UC1);

	/* Start running */
	context_start(ud);

}

static void frame_cb(struct vipcc_ctx *ctx,
		     const struct vipc_frame *frame,
		     void *be_frame,
		     void *userdata)
{
	struct context *ud = (struct context *)userdata;
	int i, j;
	struct timespec ts;
	float depth_mean;
	int res = 0;

	ULOGD("received frame %08x", frame->index);

	if (!ud->msg_cmd_handler.getProcess()) {
		vipcc_release(ctx, frame);
		return;
	}

	ud->tlm_data_out.algo.confidence = 0.f;

	if (frame->width != ud->frame_dim.width) {
		ULOGE("frame width (%d) different than status width (%d)",
			frame->width, ud->frame_dim.width);
		vipcc_release(ctx, frame);
		return;
	}
	if (frame->height != ud->frame_dim.height) {
		ULOGE("frame height (%d) different than status height (%d)",
			frame->height, ud->frame_dim.height);
		vipcc_release(ctx, frame);
		return;
	}
	if (frame->num_planes != 1) {
		ULOGE("wrong number of planes (%d)", frame->num_planes);
		vipcc_release(ctx, frame);
		return;
	}
	if (frame->format != VACQ_PIX_FORMAT_RAW32) {
		ULOGE("wrong format");
		vipcc_release(ctx, frame);
		return;
	}

	ud->depth_frame = cv::Mat(ud->frame_dim.height, ud->frame_dim.width,
			CV_32F, (void *)frame->planes[0].virt_addr,
			frame->planes[0].stride);

	ud->mask_frame.setTo(1);
	for (i = 0; i < ud->depth_frame.rows; ++i) {
		for (j = 0; j < ud->depth_frame.cols; ++j) {
			if (ud->depth_frame.at<float>(i, j) < 0.f ||
				ud->depth_frame.at<float>(i, j) == INFINITY)
				ud->mask_frame.at<uint8_t>(i, j) = 0;
		}
	}

	depth_mean = (float)cv::mean(ud->depth_frame, ud->mask_frame).val[0];

	/* Save timestamp of the frame */
	ts.tv_sec = frame->ts_sof_ns / 1000000000UL;
	ts.tv_nsec = frame->ts_sof_ns % 1000000000UL;

	res = tlm_consumer_get_sample_with_timestamp(ud->consumer, NULL,
			TLM_LATEST, &ud->tlm_data_in.timestamp);
	if (res < 0 && res != -ENOENT) {
		ULOG_ERRNO("tlm_consumer_get_sample_with_timestamp", -res);
		goto out;
	}

	ud->tlm_data_out.algo.x = ud->tlm_data_in.position_absolute.x;
	ud->tlm_data_out.algo.y = ud->tlm_data_in.position_absolute.y;
	ud->tlm_data_out.algo.z = ud->tlm_data_in.position_absolute.z;
	ud->tlm_data_out.algo.depth_mean = depth_mean;
	ud->tlm_data_out.algo.confidence = 1.f;

	/* Send event message if required */
	if (depth_mean <= CLOSE_DEPTH && ud->previous_depth_mean > CLOSE_DEPTH
			&& !ud->is_close) {
		const ::google::protobuf::Empty message;
		ud->msg_evt_sender.close(message);
		ud->is_close = true;
	}
	if (depth_mean >= FAR_DEPTH && ud->previous_depth_mean < FAR_DEPTH
			&& ud->is_close) {
		const ::google::protobuf::Empty message;
		ud->msg_evt_sender.far(message);
		ud->is_close = false;
	}
	ud->previous_depth_mean = depth_mean;

out:
	ud->depth_frame.release();
	vipcc_release(ctx, frame);
	res = tlm_producer_put_sample(ud->producer, &ts);
	if (res < 0)
		ULOG_ERRNO("tlm_producer_put_sample", -res);
}

static void
conn_status_cb(struct vipcc_ctx *ctx, bool connected, void *userdata)
{
	ULOGI("connected: %d", connected);
}

static void
eos_cb(struct vipcc_ctx *ctx, enum vipc_eos_reason reason, void *userdata)
{
	ULOGI("eos received: %s (%u)", vipc_eos_reason_to_str(reason), reason);
}

static struct vipcc_cb client_cbs = {.status_cb = status_cb,
				     .configure_cb = NULL,
				     .frame_cb = frame_cb,
				     .connection_status_cb = conn_status_cb,
				     .eos_cb = eos_cb};

static int context_init(struct context *ctx)
{
	int res = 0;
	struct vipcc_cfg_info vipc_info;

	/* Create telemetry consumer */
	ctx->consumer = tlm_consumer_new();
	if (ctx->consumer == NULL) {
		res = -ENOMEM;
		ULOG_ERRNO("tlm_consumer_new", -res);
		goto error;
	}
	res = tlm_consumer_reg_struct_ptr(ctx->consumer, &ctx->tlm_data_in,
		TLM_SECTION_USER, &s_tlm_data_in_struct);
	if (res < 0) {
		ULOG_ERRNO("tlm_consumer_reg_struct_ptr", -res);
		goto error;
	}
	res = tlm_consumer_reg_complete(ctx->consumer);
	if (res < 0) {
		ULOG_ERRNO("tlm_consumer_reg_complete", -res);
		goto error;
	}

	/* Create telemetry producer */
	ctx->producer = tlm_producer_new(TLM_SECTION_OUT,
			TLM_SECTION_OUT_COUNT, TLM_SECTION_OUT_RATE);
	if (ctx->producer == NULL) {
		res = -ENOMEM;
		ULOG_ERRNO("tlm_producer_new", -res);
		goto error;
	}
	res = tlm_producer_reg_struct_ptr(ctx->producer, &ctx->tlm_data_out,
		NULL, &s_tlm_data_out_struct);
	if (res < 0) {
		ULOG_ERRNO("tlm_producer_reg_struct_ptr", -res);
		goto error;
	}
	res = tlm_producer_reg_complete(ctx->producer);
	if (res < 0) {
		ULOG_ERRNO("tlm_producer_reg_complete", -res);
		goto error;
	}

	/* Get vipc cfg info */
	res = vipcc_cfg_get_info(VIPC_DEPTH_MAP_STREAM, &vipc_info);
	if (res < 0) {
		ULOG_ERRNO("vipcc_cfg_get_info('%s')", -res,
			VIPC_DEPTH_MAP_STREAM);
		goto error;
	} else {
		/* Create vipc client */
		ctx->vipcc = vipcc_new((struct pomp_loop *)s_ctx.loop,
				&client_cbs,
				vipc_info.be_cbs,
				vipc_info.address,
				ctx,
				5,
				true);
		if (ctx->vipcc == NULL) {
			ULOG_ERRNO("vipcc_new", -res);
			goto error;
		}
	}
	vipcc_cfg_release_info(&vipc_info);

	/* Create message hub */
	ctx->msg = new msghub::MessageHub(&s_ctx.loop, nullptr);
	if (ctx->msg == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("msg_new", -res);
		goto error;
	}

	return 0;

error:
	context_clean(ctx);
	return res;
}


static void sighandler(int signum)
{
	/* Set stopped flag and wakeup loop */
	ULOGI("signal %d (%s) received", signum, strsignal(signum));
	stop = 1;
	pomp_loop_wakeup((struct pomp_loop *)s_ctx.loop);
}

int main(int argc, char *argv[])
{
	int res = 0;

	/* Initialize context */
	res = context_init(&s_ctx);
	if (res < 0)
		goto out;

	/* Setup signal handler */
	signal(SIGINT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGPIPE, SIG_IGN);

	/* Run loop until stop is requested */
	while (!stop)
		pomp_loop_wait_and_process((struct pomp_loop *)s_ctx.loop, -1);

	/* Stop and cleanup */
	context_stop(&s_ctx);

	signal(SIGINT, SIG_DFL);
	signal(SIGTERM, SIG_DFL);
	signal(SIGPIPE, SIG_DFL);
	context_clean(&s_ctx);
out:
	return res == 0 ? 0 : -1;
}
