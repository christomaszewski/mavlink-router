/*
 * This file is part of the MAVLink Router project
 *
 * Copyright (C) 2017  Intel Corporation. All rights reserved.
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
#pragma once

#include <common/conf_file.h>

#include <assert.h>
#include <dirent.h>

#include <memory>
#include <string>

#include "endpoint.h"
#include "logwriter.h"
#include "timeout.h"

#define LOG_ENDPOINT_SYSTEM_ID 2

enum class LogMode {
    always = 0,  ///< Log from start until mavlink-router exits
    while_armed, ///< Start logging when the vehicle is armed until it's disarmed

    disabled ///< Do not try to start logging (only used internally)
};

struct LogOptions {
    enum class MavDialect { Auto, Common, Ardupilotmega };

    std::string logs_dir;                         // conf "Log" or CLI "log"
    LogMode log_mode{LogMode::always};            // conf "LogMode"
    MavDialect mavlink_dialect{MavDialect::Auto}; // conf "MavlinkDialect"
    unsigned long min_free_space;                 // conf "MinFreeSpace"
    unsigned long max_log_files;                  // conf "MaxLogFiles"
    int fcu_id{-1};                               // conf "LogSystemId"
    bool log_telemetry{false};                    // conf "LogTelemetry"
};

class LogEndpoint : public Endpoint {
public:
    LogEndpoint(std::string name, LogOptions conf);

    virtual bool start();
    virtual void stop();

    /**
     * Check existing log files and mark logs as read-only if needed.
     * This handles the case where the system (or mavlink-router) crashed or
     * lost power.
     */
    void mark_unfinished_logs();

    void log_aggregate(unsigned int interval_sec) override;

    static const ConfFile::OptionsTable option_table[];
    static int parse_mavlink_dialect(const char *val, size_t val_len, void *storage,
                                     size_t storage_len);
    static int parse_log_mode(const char *val, size_t val_len, void *storage, size_t storage_len);
    static int parse_fcu_id(const char *val, size_t val_len, void *storage, size_t storage_len);

protected:
    LogOptions _config;
    int _target_system_id;
    int _file = -1;

    struct {
        Timeout *logging_start = nullptr;
        Timeout *fsync = nullptr;
        Timeout *alive = nullptr;
    } _timeout;
    uint32_t _timeout_write_total = 0;
    std::shared_ptr<LogWriter> _writer; ///< background writer, shared by all log endpoints
    uint32_t _dropped_records = 0;      ///< log data refused by a full writer ring
    uint32_t _writer_errors_seen = 0;   ///< LogWriter::Error::count at the last poll
    bool _write_error_reported = false; ///< the default _handle_write_error() spoke up

    virtual const char *_get_logfile_extension() = 0;

    /// A write, fsync or close on the current log file failed on the writer thread. Called
    /// on the routing thread from the 1 Hz _fsync() tick, so it may stop() or restart the
    /// log. Default: report once per file and carry on -- log data is best-effort.
    virtual void _handle_write_error(int err);
    void _poll_write_errors();

    void _send_msg(const mavlink_message_t *msg, int target_sysid);
    void _remove_logging_start_timeout();
    bool _start_alive_timeout();

    virtual bool _logging_start_timeout() = 0;
    virtual bool _alive_timeout();

    bool _fsync();
    void _sync_dir_entry(int dir_fd);

    /// Write log data at the current file offset. Returns len on success, -EAGAIN when the
    /// data cannot be accepted right now (callers keep or drop it — log data is best-effort),
    /// a shorter count on a short write, or a negative errno.
    ssize_t _log_write(const void *buf, size_t len);
    /// Positioned variant (BinLog's random-access blocks); does not move the file offset.
    ssize_t _log_pwrite(const void *buf, size_t len, off_t offset);

    void _handle_auto_start_stop(const struct buffer *pbuf);

private:
    int _get_file(const char *extension);
    static uint32_t _get_prefix(DIR *dir);
    static DIR *_open_or_create_dir(const char *name);

    /**
     * Delete old logs until a certain amount of free space and total number of log files are met.
     * This can be configured using the .conf file, options MinFreeSpace and MaxLogFiles.
     */
    void _delete_old_logs();

    char _filename[64];
};
