/*
 * This file is part of the MAVLink Router project
 *
 * Copyright (C) 2026  MAVLink Router Contributors. All rights reserved.
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

#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

/*
 * Background writer shared by all log endpoints.
 *
 * Writes to a regular file ignore O_NONBLOCK: they normally land in the page cache in
 * microseconds but block for tens to hundreds of milliseconds during dirty-page writeback
 * throttling or SD-card garbage collection — and log endpoints used to issue them on the
 * routing thread, stalling every endpoint at once. This single worker thread performs the
 * actual write/pwrite/fsync/close syscalls instead; producers (the routing thread) only
 * copy the record into a fixed ring and NEVER block: when the ring is full the record is
 * dropped and counted — log data is best-effort, routing latency is not.
 *
 * The worker never calls back into the Mainloop or any endpoint: all MAVLink emission
 * (acks, nacks, start/stop messages) stays on the routing thread.
 */
class LogWriter {
public:
    static constexpr size_t DATA_MAX = 2048; ///< covers ULog's 2 KiB buffer, the largest record
    static constexpr size_t QUEUE_CAPACITY = 128;
    static constexpr size_t PATH_MAX_REC = 256;

    /// One writer shared by all log endpoints: hold the returned shared_ptr for the
    /// endpoint's lifetime; the thread drains and stops when the last owner releases it.
    static std::shared_ptr<LogWriter> instance();

    ~LogWriter();

    // Enqueue operations — true when accepted, false when the ring is full (the record is
    // dropped and counted). All data is copied; nothing references caller memory afterwards.
    bool write(int fd, const void *buf, size_t len);
    bool pwrite(int fd, const void *buf, size_t len, off_t offset);
    bool fsync(int fd);
    /// fsync then close fd (the worker takes ownership of it); with a non-null path the file
    /// is additionally chmod'ed read-only afterwards (marks a finished log)
    bool sync_close(int fd, const char *path);

    /// Block until every queued record is executed. Teardown/test aid — not for the hot path.
    void drain();

    uint32_t dropped() const { return _dropped; }

private:
    LogWriter();

    enum class Op : uint8_t { Write, PWrite, Fsync, SyncClose };

    struct Record {
        Op op;
        int fd;
        uint16_t len;
        off_t offset;
        char path[PATH_MAX_REC];
        uint8_t data[DATA_MAX];
    };

    bool _enqueue(Op op, int fd, const void *buf, size_t len, off_t offset, const char *path);
    void _run();

    std::vector<Record> _ring{QUEUE_CAPACITY};
    size_t _head = 0;      ///< next record to execute; only the worker touches the head slot
    size_t _count = 0;     ///< queued records
    uint32_t _dropped = 0; ///< records rejected because the ring was full
    bool _stop = false;
    std::mutex _mutex;
    std::condition_variable _wake;  ///< worker wakeup: work available or stopping
    std::condition_variable _empty; ///< drain() wakeup: ring fully executed
    std::thread _thread;
};
