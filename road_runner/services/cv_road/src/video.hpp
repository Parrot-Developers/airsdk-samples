/**
 * Copyright (c) 2023 Parrot Drones SAS
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 * * Neither the name of the Parrot Company nor the names
 *   of its contributors may be used to endorse or promote products
 *   derived from this software without specific prior written
 *   permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * PARROT COMPANY BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#pragma once

#include <libpomp.hpp>

#include "listener.hpp"

#include <video-ipc/vipc_client.h>
#include <video-ipc/vipc_client_cfg.h>

#define VIPC_FRONT_CAM_STREAM "fcam_airsdk"

class Video {
private:
	/* Pomp loop object */
	pomp::Loop *mLoop;

	/* Video ipc client */
	struct vipcc_ctx *mVipcc;

	/* Video ipc frame dimensions */
	struct vipc_dim *mFrameDim;

	/* Listener object for Processing class */
	Listener *mListener;

public:
	/**
	 * Constructor
	 *
	 * @param msg url string.
	 */
	Video(pomp::Loop *loop) :
			mLoop(loop), mVipcc(nullptr), mFrameDim(nullptr){};

	/**
	 * Destructor
	 */
	~Video();

	/**
	 * Start vipc
	 *
	 * @param listener A Listener object.
	 * @return  0 in case of success, negative errno in case of error.
	 */
	int vipcStart(Listener *listener);

	/**
	 * Stop vipc
	 */
	void vipcStop();
};
