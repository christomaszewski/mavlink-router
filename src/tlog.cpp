/*
 * This file is part of the MAVLink Router project
 *
 * Copyright (C) 2023 Petrosilius <petrosilius@searchwing.org>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "tlog.h"

#include <assert.h>
#include <endian.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <unistd.h>

#include <chrono>

#include <common/log.h>
#include <common/util.h>

bool TLog::_logging_start_timeout()
{
    //_target_system_id = -1;
    return true;
}

bool TLog::start()
{
    if (!LogEndpoint::start()) {
        return false;
    }

    return true;
}

void TLog::stop()
{
    if (_file == -1) {
        log_info("TLog not started");
        return;
    }

    LogEndpoint::stop();
}

int TLog::write_msg(const struct buffer *buffer)
{
    /* set the expected system id to the first autopilot that we get a heartbeat from */
    if (_target_system_id == -1 && buffer->curr.msg_id == MAVLINK_MSG_ID_HEARTBEAT
        && buffer->curr.src_compid == MAV_COMP_ID_AUTOPILOT1) {
        _target_system_id = buffer->curr.src_sysid;
    }

    /* Check if we should start or stop logging */
    _handle_auto_start_stop(buffer);

    uint64_t ms_since_epoch = std::chrono::duration_cast<std::chrono::microseconds>(
                                  std::chrono::system_clock::now().time_since_epoch())
                                  .count();

    ms_since_epoch = htobe64(ms_since_epoch);

    struct iovec iov[2];
    iov[0].iov_base = &ms_since_epoch;
    iov[0].iov_len = sizeof(uint64_t);
    iov[1].iov_base = buffer->data;
    iov[1].iov_len = buffer->len;

    ssize_t r = writev(_file, iov, 2);
    if (r < 0) {
        log_error("TLog [%d]%s: Error writing to log file (%m)", fd, _name.c_str());
        return buffer->len; // like the log endpoints' general policy: logging is best-effort
    }

    return buffer->len;
}