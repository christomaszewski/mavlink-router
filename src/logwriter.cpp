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
#include "logwriter.h"

#include <errno.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include <common/log.h>

std::shared_ptr<LogWriter> LogWriter::instance()
{
    // only ever called from the routing thread, so the weak_ptr needs no locking
    static std::weak_ptr<LogWriter> weak;
    auto strong = weak.lock();
    if (!strong) {
        strong = std::shared_ptr<LogWriter>(new LogWriter());
        weak = strong;
    }
    return strong;
}

LogWriter::LogWriter()
{
    _thread = std::thread(&LogWriter::_run, this);
}

LogWriter::~LogWriter()
{
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _stop = true;
    }
    _wake.notify_one();
    _thread.join(); // the worker only exits with an empty ring, so queued data is written
    if (_dropped > 0) {
        log_warning("LogWriter: %u log record(s) dropped (queue full)", _dropped);
    }
}

bool LogWriter::write(int fd, const void *buf, size_t len)
{
    return _enqueue(Op::Write, fd, buf, len, 0, nullptr);
}

bool LogWriter::pwrite(int fd, const void *buf, size_t len, off_t offset)
{
    return _enqueue(Op::PWrite, fd, buf, len, offset, nullptr);
}

bool LogWriter::fsync(int fd)
{
    return _enqueue(Op::Fsync, fd, nullptr, 0, 0, nullptr);
}

bool LogWriter::sync_close(int fd, const char *path)
{
    return _enqueue(Op::SyncClose, fd, nullptr, 0, 0, path);
}

bool LogWriter::_enqueue(Op op, int fd, const void *buf, size_t len, off_t offset, const char *path)
{
    if (len > DATA_MAX || (path != nullptr && strlen(path) >= PATH_MAX_REC)) {
        return false; // cannot be represented; treated by callers like a full queue
    }

    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_count == QUEUE_CAPACITY) {
            _dropped++;
            return false;
        }

        Record &rec = _ring[(_head + _count) % QUEUE_CAPACITY];
        rec.op = op;
        rec.fd = fd;
        rec.len = (uint16_t)len;
        rec.offset = offset;
        if (buf != nullptr && len > 0) {
            memcpy(rec.data, buf, len);
        }
        if (path != nullptr) {
            memcpy(rec.path, path, strlen(path) + 1);
        } else {
            rec.path[0] = '\0';
        }
        _count++;
    }
    _wake.notify_one();
    return true;
}

void LogWriter::_run()
{
    std::unique_lock<std::mutex> lock(_mutex);
    while (true) {
        while (_count == 0) {
            _empty.notify_all();
            if (_stop) {
                return; // the ring is empty: nothing can be lost
            }
            _wake.wait(lock);
        }

        // The head slot belongs to the worker while _count > 0 (producers append at
        // head+count), so the syscalls run without the lock and without copying the record.
        Record &rec = _ring[_head];
        lock.unlock();

        switch (rec.op) {
        case Op::Write:
        case Op::PWrite: {
            size_t done = 0;
            while (done < rec.len) {
                ssize_t r;
                if (rec.op == Op::Write) {
                    r = ::write(rec.fd, rec.data + done, rec.len - done);
                } else {
                    r = ::pwrite(rec.fd, rec.data + done, rec.len - done, rec.offset + done);
                }
                if (r < 0) {
                    if (errno == EINTR) {
                        continue;
                    }
                    log_error("LogWriter: write failed on fd %d (%m)", rec.fd);
                    break;
                }
                done += r;
            }
            break;
        }
        case Op::Fsync:
            ::fsync(rec.fd);
            break;
        case Op::SyncClose:
            ::fsync(rec.fd);
            ::close(rec.fd);
            if (rec.path[0] != '\0') {
                // mark the finished log read-only
                chmod(rec.path, S_IRUSR | S_IRGRP | S_IROTH);
            }
            break;
        }

        lock.lock();
        _head = (_head + 1) % QUEUE_CAPACITY;
        _count--;
    }
}

void LogWriter::drain()
{
    std::unique_lock<std::mutex> lock(_mutex);
    _empty.wait(lock, [this] { return _count == 0; });
}
