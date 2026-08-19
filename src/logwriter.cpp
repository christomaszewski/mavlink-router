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
#include <pthread.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>

#include <exception>

#include <common/log.h>

std::shared_ptr<LogWriter> LogWriter::instance()
{
    // only ever called from the routing thread, so the weak_ptr needs no locking
    static std::weak_ptr<LogWriter> weak;
    auto strong = weak.lock();
    if (!strong) {
        try {
            strong = std::shared_ptr<LogWriter>(new LogWriter());
        } catch (const std::exception &e) {
            // RLIMIT_NPROC or out of memory: the caller runs without logging instead of
            // taking the router down with an uncaught exception
            log_error("LogWriter: cannot start the writer thread (%s)", e.what());
            return nullptr;
        }
        weak = strong;
    }
    return strong;
}

LogWriter::LogWriter()
{
    /* The worker must not be the thread that receives SIGTERM/SIGINT: the daemon installs
     * plain sigaction handlers, and a signal delivered to the worker sets the exit flag
     * without interrupting the routing thread's epoll_wait, so shutdown would wait for the
     * next routed message. A new thread inherits the creator's mask — block everything
     * while it starts (the aio helper threads this replaces did the same). */
    sigset_t all, saved;
    sigfillset(&all);
    pthread_sigmask(SIG_BLOCK, &all, &saved);
    try {
        _thread = std::thread(&LogWriter::_run, this);
    } catch (...) {
        pthread_sigmask(SIG_SETMASK, &saved, nullptr);
        throw;
    }
    pthread_sigmask(SIG_SETMASK, &saved, nullptr);
    pthread_setname_np(_thread.native_handle(), "log-writer"); // visible in ps/top -H
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
    return _enqueue(Op::Write, fd, buf, len, 0);
}

bool LogWriter::pwrite(int fd, const void *buf, size_t len, off_t offset)
{
    return _enqueue(Op::PWrite, fd, buf, len, offset);
}

bool LogWriter::fsync(int fd)
{
    return _enqueue(Op::Fsync, fd, nullptr, 0, 0);
}

void LogWriter::sync_close(int fd)
{
    _enqueue(Op::SyncClose, fd, nullptr, 0, 0);
}

uint32_t LogWriter::dropped() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _dropped;
}

bool LogWriter::_enqueue(Op op, int fd, const void *buf, size_t len, off_t offset)
{
    if (len > DATA_MAX) {
        return false; // cannot be represented; treated by callers like a full queue
    }

    {
        std::unique_lock<std::mutex> lock(_mutex);
        switch (op) {
        case Op::Write:
        case Op::PWrite:
            if (_count >= QUEUE_CAPACITY - CONTROL_RESERVE) {
                _dropped++;
                return false;
            }
            break;
        case Op::Fsync:
            if (_count == QUEUE_CAPACITY) {
                return false;
            }
            break;
        case Op::SyncClose:
            // even the reserve is taken: the worker frees a slot when its current syscall
            // returns, and an fd handed over must never be dropped on the floor
            _room.wait(lock, [this] { return _count < QUEUE_CAPACITY; });
            break;
        }

        Record &rec = _ring[(_head + _count) % QUEUE_CAPACITY];
        rec.op = op;
        rec.fd = fd;
        rec.len = (uint16_t)len;
        rec.offset = offset;
        if (buf != nullptr && len > 0) {
            memcpy(rec.data, buf, len);
        }
        _count++;
    }
    _wake.notify_one();
    return true;
}

/// Runs one record's syscalls without the lock. Returns 0 or the errno of the failure.
int LogWriter::_execute(const Record &rec)
{
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
                return errno;
            }
            if (r == 0) {
                return EIO; // no progress with bytes pending: looping would never end
            }
            done += r;
        }
        return 0;
    }
    case Op::Fsync:
        return ::fsync(rec.fd) < 0 ? errno : 0;
    case Op::SyncClose: {
        int err = ::fsync(rec.fd) < 0 ? errno : 0;
        ::close(rec.fd);
        return err;
    }
    }
    return 0;
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
        const Record &rec = _ring[_head];
        lock.unlock();

        const int err = _execute(rec);
        if (err != 0) {
            // one line per failure streak: a full or dead disk fails every record
            if (rec.fd != _failing_fd) {
                log_error("LogWriter: %s failed on fd %d (%s)",
                          rec.op == Op::Fsync || rec.op == Op::SyncClose ? "fsync" : "write",
                          rec.fd,
                          strerror(err));
                _failing_fd = rec.fd;
            }
        } else if (rec.fd == _failing_fd) {
            _failing_fd = -1;
        }
        if (rec.op == Op::SyncClose && rec.fd == _failing_fd) {
            _failing_fd = -1; // the number is free for reuse by an unrelated file
        }

        lock.lock();
        _head = (_head + 1) % QUEUE_CAPACITY;
        _count--;
        if (_count == QUEUE_CAPACITY - 1) {
            _room.notify_all(); // a sync_close() may be waiting for exactly this slot
        }
    }
}

void LogWriter::drain()
{
    std::unique_lock<std::mutex> lock(_mutex);
    _empty.wait(lock, [this] { return _count == 0; });
}
