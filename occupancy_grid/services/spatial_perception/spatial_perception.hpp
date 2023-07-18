
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

#include <memory>
#include <string>

#include <libmoser_ipc_client.hpp>
#include <libpomp.hpp>

class SpatialPerception : public ::moser::Client::Callbacks {
public:
  class Client {
  public:
    virtual void onSpatialPerceptionReady() = 0;
  };

  SpatialPerception(pomp::Loop *loop, SpatialPerception::Client *client,
                    const std::string &server, const std::string &consumer);
  ~SpatialPerception();

  /* Subscribe to occupation grid server. */
  int start();
  /* Unsubscribe from the occupation grid server. */
  void stop();

  void gridReceived(moser::Client::Consumer *consumer,
                    std::unique_ptr<moser::IGrid> grid) override;

  const moser::IGrid &getGrid() const;

  bool isReady() const { return this->mIsReady; }

private:
  /* The associated runloop. */
  pomp::Loop *const mLoop;
  /* The client to notify. */
  SpatialPerception::Client *const mClient;
  /* The occupation grid server address. */
  const std::string &mServerAddress;
  /* The occupation grid consumer name. */
  const std::string &mConsumerName;
  /* The occupation grid client. */
  moser::Client *mMoserClient;
  /* The occupation grid consumer that will get notified by the client. */
  moser::Client::Consumer *mMoserConsumer;
  /* The last occupation grid received from the client. */
  std::unique_ptr<moser::IGrid> mLastGrid;
  /* A flag indicating whether the spatial perception is ready and thus
   * `mLastGrid` can be accessed. */
  bool mIsReady;
};