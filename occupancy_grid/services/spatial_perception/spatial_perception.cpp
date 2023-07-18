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

#include "spatial_perception.hpp"
#include <string>

#include <libpomp.hpp>

#define ULOG_TAG spatial_perception
#include <ulog.h>

SpatialPerception::SpatialPerception(pomp::Loop *loop,
                                     SpatialPerception::Client *client,
                                     const std::string &serverAddress,
                                     const std::string &consumerName)
    : mLoop(loop), mClient(client), mServerAddress(serverAddress),
      mConsumerName(consumerName), mMoserClient(nullptr),
      mMoserConsumer(nullptr), mLastGrid(nullptr), mIsReady(false) {}

SpatialPerception::~SpatialPerception() {}

int SpatialPerception::start() {
  moser::Client::Config moserConfig = {.addr = mServerAddress};

  int ret = moser::Client::create(mLoop, moserConfig, this, &mMoserClient);

  if (ret < 0) {
    ULOG_ERRNO("start moser ipc client", -ret);
    return ret;
  }

  ret = mMoserClient->addConsumer(mConsumerName, &mMoserConsumer);

  if (ret < 0) {
    mMoserConsumer = nullptr;
    ULOG_ERRNO("moser add consumer '%s'", -ret, mConsumerName.c_str());
    return ret;
  }

  ret = mMoserClient->start();
  if (ret < 0) {
    ULOG_ERRNO("moser client start", -ret);
    return ret;
  }

  return 0;
}

void SpatialPerception::stop() {
  this->mIsReady = false;
  if (mMoserClient != nullptr) {
    mMoserClient->stop();
    if (mMoserConsumer != nullptr) {
      if (mLastGrid != nullptr)
        mMoserClient->releaseGrid(mMoserConsumer, std::move(mLastGrid));
      mLastGrid = nullptr;
      mMoserClient->removeConsumer(mMoserConsumer);
      mMoserConsumer = nullptr;
    }
    moser::Client::destroy(mMoserClient);
    mMoserClient = nullptr;
  }
}

void SpatialPerception::gridReceived(moser::Client::Consumer *consumer,
                                     std::unique_ptr<moser::IGrid> grid) {
  bool wasNull = (mLastGrid == nullptr);
  if (!wasNull)
    mMoserClient->releaseGrid(consumer, std::move(mLastGrid));
  mLastGrid = std::move(grid);
  if (wasNull) {
    ULOGI("first grid");
    this->mIsReady = true;
    mClient->onSpatialPerceptionReady();
  }
}

const moser::IGrid &SpatialPerception::getGrid() const { return *mLastGrid; }
