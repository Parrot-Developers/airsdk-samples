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
#include <cstring>

#define ULOG_TAG service_move_along_cfg
#include <ulog.hpp>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "configuration.hpp"

static const std::string MOVE_ALONG_SERVICE_CONFIG_PATH =
	"/etc/services/move_along.cfg";

#define CFG_CHECK(E) ULOG_ERRNO_RETURN_ERR_IF(E < 0, EINVAL)

int MissionConfiguration::start()
{
	int res = 0;

	const std::string path = cfgreader::ConfigReader::insertMissionRootDir(
		MOVE_ALONG_SERVICE_CONFIG_PATH);

	cfgreader::FileConfigReader reader(path);

	res = reader.load();
	if (res < 0) {
		ULOG_ERRNO("cannot load %s", -res, path.c_str());
		return res;
	}

	res = reader.get("move_along", mVelocityCfg);
	if (res < 0) {
		ULOG_ERRNO("cannot read move_along config", -res);
		return res;
	}

	ULOGI("move_along config : horizontal velocity(%f) vertical verlocity(%f) yaw velocity(%f)",
	      mVelocityCfg.horizontalSpeed,
	      mVelocityCfg.verticalSpeed,
	      mVelocityCfg.yawSpeed);

	return res;
}

namespace cfgreader {
template <>
int SettingReader<MissionConfiguration::velocityCfg>::read(
	const libconfig::Setting &set,
	MissionConfiguration::velocityCfg &v)
{
	ULOGN("Getting configuration values");
	CFG_CHECK(ConfigReader::getField(
		set, "horizontalSpeed", v.horizontalSpeed));
	CFG_CHECK(
		ConfigReader::getField(set, "verticalSpeed", v.verticalSpeed));
	CFG_CHECK(ConfigReader::getField(set, "yawSpeed", v.yawSpeed));
	return 0;
}
} // namespace cfgreader