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

#include <libpomp.hpp>
#include <libtelemetry.h>
#include <video-ipc/vipc_client.h>
#include <video-ipc/vipc_client_cfg.h>

#include <google/protobuf/empty.pb.h>
#include <samples/hello/cv-service/messages.msghub.h>

#include "processing.h"

#define VIPC_DEPTH_MAP_STREAM "fstcam_stereo_depth_filtered"
#define TLM_SECTION_USER "drone_controller"
#define TLM_SECTION_OUT "cv@hello"
#define TLM_SECTION_OUT_RATE 1000
#define TLM_SECTION_OUT_COUNT 10
#define MSGHUB_ADDR "unix:/tmp/hello-cv-service"
#define CLOSE_DEPTH 0.8f /* [m] */
#define FAR_DEPTH 1.2f   /* [m] */

struct tlm_data_in {
	struct {
		float x;
		float y;
		float z;
	} velocity;

	struct {
		float x;
		float y;
		float z;
	} position_global;

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

class HelloServiceCommandHandler : public ::samples::hello::cv_service::
					   messages::msghub::CommandHandler {
public:
	inline HelloServiceCommandHandler(struct context *ctx) : mCtx(ctx) {}

	virtual void processingStart(
		const ::google::protobuf::Empty &args) override;

	virtual void processingStop(
		const ::google::protobuf::Empty &args) override;

private:
	struct context *mCtx;
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

	/* Processing result notification event */
	struct pomp_evt *processing_evt;

	/* Processing in background thread */
	struct processing *processing;

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

	inline context() : msg_cmd_handler(this), is_close(false) {}
};

// clang-format off
static const struct tlm_reg_field s_tlm_data_in_fields[] = {
	TLM_REG_FIELD_SCALAR_EX(struct tlm_data_in, velocity.x,
			"linear_velocity_global.x", TLM_TYPE_FLOAT32),
	TLM_REG_FIELD_SCALAR_EX(struct tlm_data_in, velocity.y,
			"linear_velocity_global.y", TLM_TYPE_FLOAT32),
	TLM_REG_FIELD_SCALAR_EX(struct tlm_data_in, velocity.z,
			"linear_velocity_global.z", TLM_TYPE_FLOAT32),

	TLM_REG_FIELD_SCALAR_EX(struct tlm_data_in, position_global.x,
			"position_global.x", TLM_TYPE_FLOAT32),
	TLM_REG_FIELD_SCALAR_EX(struct tlm_data_in, position_global.y,
			"position_global.y", TLM_TYPE_FLOAT32),
	TLM_REG_FIELD_SCALAR_EX(struct tlm_data_in, position_global.z,
			"position_global.z", TLM_TYPE_FLOAT32),

	TLM_REG_FIELD_SCALAR_EX(struct tlm_data_in, attitude.yaw,
			"attitude_euler_angles.yaw", TLM_TYPE_FLOAT32),
	TLM_REG_FIELD_SCALAR_EX(struct tlm_data_in, attitude.pitch,
			"attitude_euler_angles.pitch", TLM_TYPE_FLOAT32),
	TLM_REG_FIELD_SCALAR_EX(struct tlm_data_in, attitude.roll,
			"attitude_euler_angles.roll", TLM_TYPE_FLOAT32),
};

static const struct tlm_reg_field s_tlm_data_out_fields[] = {
	TLM_REG_FIELD_SCALAR(struct tlm_data_out, algo.x,
			TLM_TYPE_FLOAT32),
	TLM_REG_FIELD_SCALAR(struct tlm_data_out, algo.y,
			TLM_TYPE_FLOAT32),
	TLM_REG_FIELD_SCALAR(struct tlm_data_out, algo.z,
			TLM_TYPE_FLOAT32),
	TLM_REG_FIELD_SCALAR(struct tlm_data_out, algo.depth_mean,
			TLM_TYPE_FLOAT32),
	TLM_REG_FIELD_SCALAR(struct tlm_data_out, algo.confidence,
			TLM_TYPE_FLOAT32),
};
// clang-format on

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

	if (ctx->processing != NULL) {
		processing_destroy(ctx->processing);
		ctx->processing = NULL;
	}

	if (ctx->processing_evt != NULL) {
		res = pomp_evt_detach_from_loop(ctx->processing_evt, ctx->loop);
		if (res < 0)
			ULOG_ERRNO("pomp_evt_detach_from_loop", -res);
		res = pomp_evt_destroy(ctx->processing_evt);
		if (res < 0)
			ULOG_ERRNO("pomp_evt_destroy", -res);
		ctx->processing_evt = NULL;
	}

	delete ctx->msg;
	ctx->msg = NULL;

	if (ctx->producer != NULL) {
		res = tlm_producer_destroy(ctx->producer);
		if (res < 0)
			ULOG_ERRNO("tlm_producer_destroy", -res);
		ctx->producer = NULL;
	}

	if (ctx->consumer != NULL) {
		res = tlm_consumer_destroy(ctx->consumer);
		if (res < 0)
			ULOG_ERRNO("tlm_consumer_destroy", -res);
		ctx->consumer = NULL;
	}
}

static int context_start(struct context *ctx)
{
	/* msg */
	ctx->msg_channel =
		ctx->msg->startServerChannel(pomp::Address(MSGHUB_ADDR), 0666);
	if (ctx->msg_channel == nullptr) {
		ULOGE("Failed to start server channel on '%s'", MSGHUB_ADDR);
		return -EPERM;
	}

	ctx->msg->attachMessageHandler(&ctx->msg_cmd_handler);
	ctx->msg->attachMessageSender(&ctx->msg_evt_sender, ctx->msg_channel);

	return 0;
}

static int context_stop(struct context *ctx)
{
	int res = 0;

	/* Processing */
	processing_stop(ctx->processing);
	/* Stop vipc and processing */
	ctx->msg_cmd_handler.processingStop(::google::protobuf::Empty());

	/* msg */
	ctx->msg->detachMessageSender(&ctx->msg_evt_sender);
	ctx->msg->detachMessageHandler(&ctx->msg_cmd_handler);
	ctx->msg->stop();
	ctx->msg_channel = nullptr;

	return res;
}

static void processing_evt_cb(struct pomp_evt *evt, void *userdata)
{
	int res = 0;
	struct context *ud = (struct context *)userdata;
	struct processing_output output;

	/* Get result from processing object */
	res = processing_get_output(ud->processing, &output);
	if (res < 0) {
		ULOG_ERRNO("processing_get_output", -res);
		return;
	}

	/* Update telemetry output */
	ud->tlm_data_out.algo.x = output.x;
	ud->tlm_data_out.algo.y = output.y;
	ud->tlm_data_out.algo.z = output.z;
	ud->tlm_data_out.algo.depth_mean = output.depth_mean;
	ud->tlm_data_out.algo.confidence = output.confidence;

	/* Write in telemetry */
	res = tlm_producer_put_sample(ud->producer, &output.ts);
	if (res < 0)
		ULOG_ERRNO("tlm_producer_put_sample", -res);

	/* Send event message if required */
	if (output.depth_mean <= CLOSE_DEPTH
	    && ud->previous_depth_mean > CLOSE_DEPTH && !ud->is_close) {
		const ::google::protobuf::Empty message;
		ud->msg_evt_sender.close(message);
		ud->is_close = true;
	}
	if (output.depth_mean >= FAR_DEPTH
	    && ud->previous_depth_mean < FAR_DEPTH && ud->is_close) {
		const ::google::protobuf::Empty message;
		ud->msg_evt_sender.far(message);
		ud->is_close = false;
	}
	ud->previous_depth_mean = output.depth_mean;
}

static void status_cb(struct vipcc_ctx *ctx,
		      const struct vipc_status *st,
		      void *userdata)
{
	int res = 0;
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

	res = vipcc_start(ctx);
	if (res < 0)
		ULOG_ERRNO("vipcc_start", -res);
}

static void frame_cb(struct vipcc_ctx *ctx,
		     const struct vipc_frame *frame,
		     void *be_frame,
		     void *userdata)
{
	int res = 0;
	struct context *ud = (struct context *)userdata;
	struct processing_input input;
	struct timespec timestamp;

	ULOGD("received frame %08x", frame->index);

	/* Sanity checks */
	if (frame->width != ud->frame_dim.width) {
		ULOGE("frame width (%d) different than status width (%d)",
		      frame->width,
		      ud->frame_dim.width);
		goto out;
	}
	if (frame->height != ud->frame_dim.height) {
		ULOGE("frame height (%d) different than status height (%d)",
		      frame->height,
		      ud->frame_dim.height);
		goto out;
	}
	if (frame->num_planes != 1) {
		ULOGE("wrong number of planes (%d)", frame->num_planes);
		goto out;
	}
	if (frame->format != VACQ_PIX_FORMAT_RAW32) {
		ULOGE("wrong format");
		goto out;
	}

	/* Get latest telemetry data */
	timestamp.tv_sec = frame->ts_sof_ns / 1000000000UL;
	timestamp.tv_nsec = frame->ts_sof_ns % 1000000000UL;
	res = tlm_consumer_get_sample(ud->consumer, &timestamp, TLM_CLOSEST);
	if (res < 0 && res != -ENOENT) {
		ULOG_ERRNO("tlm_consumer_get_sample_with_timestamp", -res);
		goto out;
	}

	/* Setup input structure for processing */
	memset(&input, 0, sizeof(input));
	input.frame = frame;
	input.position_global.x = ud->tlm_data_in.position_global.x;
	input.position_global.y = ud->tlm_data_in.position_global.y;
	input.position_global.z = ud->tlm_data_in.position_global.z;

	res = processing_step(ud->processing, &input);
	if (res < 0) {
		ULOG_ERRNO("processing_step", -res);
		goto out;
	}

	/* The frame has been given to the processing object */
	frame = NULL;

	/* In case of error, need to release the input frame */
out:
	if (frame != NULL)
		vipcc_release(ctx, frame);
}

static void conn_status_cb(struct vipcc_ctx *ctx,
			   bool connected,
			   void *userdata)
{
	ULOGI("connected: %d", connected);
}

static void eos_cb(struct vipcc_ctx *ctx,
		   enum vipc_eos_reason reason,
		   void *userdata)
{
	ULOGI("eos received: %s (%u)", vipc_eos_reason_to_str(reason), reason);
}

static const struct vipcc_cb s_vipc_client_cbs = {.status_cb = status_cb,
						  .configure_cb = NULL,
						  .frame_cb = frame_cb,
						  .connection_status_cb =
							  conn_status_cb,
						  .eos_cb = eos_cb};

static int context_init(struct context *ctx)
{
	int res = 0;

	/* Create telemetry consumer */
	ctx->consumer = tlm_consumer_new();
	if (ctx->consumer == NULL) {
		res = -ENOMEM;
		ULOG_ERRNO("tlm_consumer_new", -res);
		goto error;
	}
	res = tlm_consumer_reg_struct_ptr(ctx->consumer,
					  &ctx->tlm_data_in,
					  TLM_SECTION_USER,
					  &s_tlm_data_in_struct);
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
	ctx->producer = tlm_producer_new(
		TLM_SECTION_OUT, TLM_SECTION_OUT_COUNT, TLM_SECTION_OUT_RATE);
	if (ctx->producer == NULL) {
		res = -ENOMEM;
		ULOG_ERRNO("tlm_producer_new", -res);
		goto error;
	}
	res = tlm_producer_reg_struct_ptr(ctx->producer,
					  &ctx->tlm_data_out,
					  NULL,
					  &s_tlm_data_out_struct);
	if (res < 0) {
		ULOG_ERRNO("tlm_producer_reg_struct_ptr", -res);
		goto error;
	}
	res = tlm_producer_reg_complete(ctx->producer);
	if (res < 0) {
		ULOG_ERRNO("tlm_producer_reg_complete", -res);
		goto error;
	}

	/* Create message hub */
	ctx->msg = new msghub::MessageHub(&s_ctx.loop, nullptr);
	if (ctx->msg == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("msg_new", -res);
		goto error;
	}

	/* Create processing notification event */
	ctx->processing_evt = pomp_evt_new();
	if (ctx->processing_evt == NULL) {
		res = -ENOMEM;
		ULOG_ERRNO("pomp_evt_new", -res);
		goto error;
	}
	res = pomp_evt_attach_to_loop(
		ctx->processing_evt, ctx->loop.get(), &processing_evt_cb, ctx);
	if (res < 0) {
		ULOG_ERRNO("pomp_evt_attach_to_loop", -res);
		goto error;
	}

	/* Create processing object */
	res = processing_new(ctx->processing_evt, &ctx->processing);
	if (res < 0) {
		ULOG_ERRNO("processing_new", -res);
		goto error;
	}

	return 0;

error:
	context_clean(ctx);
	return res;
}

void HelloServiceCommandHandler::processingStart(
	const ::google::protobuf::Empty &args)
{
	int res = 0;
	struct vipcc_cfg_info vipc_info;
	memset(&vipc_info, 0, sizeof(vipc_info));

	/* Make sure not already in progress */
	ULOG_ERRNO_RETURN_IF(mCtx->vipcc != NULL, EBUSY);

	/* Get vipc cfg info */
	res = vipcc_cfg_get_info(VIPC_DEPTH_MAP_STREAM, &vipc_info);
	if (res < 0) {
		ULOG_ERRNO("vipcc_cfg_get_info('%s')",
			   -res,
			   VIPC_DEPTH_MAP_STREAM);
		goto error;
	} else {
		/* Create vipc client */
		mCtx->vipcc = vipcc_new(mCtx->loop.get(),
					&s_vipc_client_cbs,
					vipc_info.be_cbs,
					vipc_info.address,
					mCtx,
					5,
					true);
		if (mCtx->vipcc == NULL) {
			ULOG_ERRNO("vipcc_new", -res);
			goto error;
		}
	}
	vipcc_cfg_release_info(&vipc_info);

	/* Background thread processing */
	res = processing_start(mCtx->processing);
	if (res < 0) {
		ULOG_ERRNO("processing_start", -res);
		goto error;
	}

error:
	/* TODO */
	;
}

void HelloServiceCommandHandler::processingStop(
	const ::google::protobuf::Empty &args)
{
	/* Background thread processing */
	processing_stop(mCtx->processing);

	if (mCtx->vipcc != NULL) {
		vipcc_stop(mCtx->vipcc);
		vipcc_destroy(mCtx->vipcc);
		mCtx->vipcc = NULL;
	}
}

static void sighandler(int signum)
{
	/* Set stopped flag and wakeup loop */
	ULOGI("signal %d (%s) received", signum, strsignal(signum));
	stop = 1;
	pomp_loop_wakeup(s_ctx.loop.get());
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

	context_start(&s_ctx);

	/* Run loop until stop is requested */
	while (!stop)
		pomp_loop_wait_and_process(s_ctx.loop.get(), -1);

	/* Stop and cleanup */
	context_stop(&s_ctx);

	signal(SIGINT, SIG_DFL);
	signal(SIGTERM, SIG_DFL);
	signal(SIGPIPE, SIG_DFL);
	context_clean(&s_ctx);
out:
	return res == 0 ? 0 : -1;
}
