/*
 * This file is part of the MAVLink Router project
 *
 * Copyright (C) 2021  MAVLink Router Contributors. All rights reserved.
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

#include "autolog.h"
#include "binlog.h"
#include "endpoint.h"
#include "mainloop.h"
#include "tlog.h"
#include "ulog.h"

#include <dirent.h>
#include <endian.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <deque>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "mainloop.h"

/**
 * Endpoint base class
 */

// Create custom inhertited class b/c Endpoint can't be initialized containing pure virtual methods
class TestEndpoint : public Endpoint {
public:
    TestEndpoint()
        : Endpoint{"Test", "foobar"} {};
    ~TestEndpoint() override{};

    // dummy-implement virtual methods
    int write_msg(const struct buffer *pbuf) override { return true; };
    ssize_t _read_msg(uint8_t *buf, size_t len) override { return 0; };

    // expose some internal data
    void set_sys_comp_ids(std::vector<uint16_t> sys_comp_ids) { _sys_comp_ids = sys_comp_ids; };
    std::vector<uint16_t> get_sys_comp_ids() { return _sys_comp_ids; };
};

static uint16_t build_sys_comp_id(unsigned sysid, unsigned compid)
{
    return ((sysid & 0xff) << 8) | (compid & 0xff);
}

TEST(EndpointTest, HasSysId)
{
    TestEndpoint endpoint;
    std::vector<uint16_t> sys_comp_ids{};

    // start with empty knwon systems list
    EXPECT_FALSE(endpoint.has_sys_id(4));

    // add one system
    sys_comp_ids.push_back(build_sys_comp_id(12, 1));
    endpoint.set_sys_comp_ids(sys_comp_ids);
    EXPECT_TRUE(endpoint.has_sys_id(12));
    EXPECT_FALSE(endpoint.has_sys_id(11));
    EXPECT_FALSE(endpoint.has_sys_id(13));
    EXPECT_FALSE(endpoint.has_sys_id(0));
    EXPECT_FALSE(endpoint.has_sys_id(255));

    // add more systems
    sys_comp_ids.push_back(build_sys_comp_id(254, 1));
    endpoint.set_sys_comp_ids(sys_comp_ids);
    EXPECT_TRUE(endpoint.has_sys_id(12));
    EXPECT_TRUE(endpoint.has_sys_id(254));
    EXPECT_FALSE(endpoint.has_sys_id(11));
    EXPECT_FALSE(endpoint.has_sys_id(13));
    EXPECT_FALSE(endpoint.has_sys_id(253));
    EXPECT_FALSE(endpoint.has_sys_id(255));
    EXPECT_FALSE(endpoint.has_sys_id(0));
}

// this method does NOT check for broadcast rules
TEST(EndpointTest, HasSysCompId)
{
    TestEndpoint endpoint;
    std::vector<uint16_t> sys_comp_ids{};

    // start with empty knwon systems list
    EXPECT_FALSE(endpoint.has_sys_comp_id(4, 0));

    // add one system
    sys_comp_ids.push_back(build_sys_comp_id(12, 1));
    endpoint.set_sys_comp_ids(sys_comp_ids);
    EXPECT_TRUE(endpoint.has_sys_comp_id(12, 1));
    EXPECT_FALSE(endpoint.has_sys_comp_id(12, 0));
    EXPECT_FALSE(endpoint.has_sys_comp_id(13, 1));
    EXPECT_FALSE(endpoint.has_sys_comp_id(13, 0));
    EXPECT_FALSE(endpoint.has_sys_comp_id(0, 1));
    EXPECT_FALSE(endpoint.has_sys_comp_id(0, 0));

    // add more systems
    sys_comp_ids.push_back(build_sys_comp_id(254, 190));
    endpoint.set_sys_comp_ids(sys_comp_ids);
    EXPECT_TRUE(endpoint.has_sys_comp_id(12, 1));
    EXPECT_FALSE(endpoint.has_sys_comp_id(12, 0));
    EXPECT_FALSE(endpoint.has_sys_comp_id(13, 1));
    EXPECT_FALSE(endpoint.has_sys_comp_id(13, 0));
    EXPECT_FALSE(endpoint.has_sys_comp_id(0, 1));
    EXPECT_FALSE(endpoint.has_sys_comp_id(0, 0));

    EXPECT_TRUE(endpoint.has_sys_comp_id(254, 190));
    EXPECT_FALSE(endpoint.has_sys_comp_id(254, 0));
    EXPECT_FALSE(endpoint.has_sys_comp_id(254, 1));
    EXPECT_FALSE(endpoint.has_sys_comp_id(255, 190));
    EXPECT_FALSE(endpoint.has_sys_comp_id(255, 0));
    EXPECT_FALSE(endpoint.has_sys_comp_id(255, 1));
}

TEST(EndpointTest, AcceptMsg_EmptyKnownSystems)
{
    TestEndpoint endpoint;
    buffer test_msg;
    test_msg.curr.msg_id = 1;

    // accept message with no target address
    test_msg.curr.src_sysid = 1;
    test_msg.curr.src_compid = 1;
    test_msg.curr.target_sysid = -1;
    test_msg.curr.target_compid = -1;
    EXPECT_EQ(endpoint.accept_msg(&test_msg), Endpoint::AcceptState::Accepted);

    // reject message with any target address (since no system connected)
    test_msg.curr.src_sysid = 1;
    test_msg.curr.src_compid = 1;
    test_msg.curr.target_sysid = 254;
    test_msg.curr.target_compid = 190;
    EXPECT_EQ(endpoint.accept_msg(&test_msg), Endpoint::AcceptState::Rejected);

    // reject message with any target address (since no system connected) - component broadcast, too
    test_msg.curr.src_sysid = 1;
    test_msg.curr.src_compid = 1;
    test_msg.curr.target_sysid = 254;
    test_msg.curr.target_compid = 0;
    EXPECT_EQ(endpoint.accept_msg(&test_msg), Endpoint::AcceptState::Rejected);

    test_msg.curr.src_sysid = 1;
    test_msg.curr.src_compid = 1;
    test_msg.curr.target_sysid = 254;
    test_msg.curr.target_compid = -1;
    EXPECT_EQ(endpoint.accept_msg(&test_msg), Endpoint::AcceptState::Rejected);
}

TEST(EndpointTest, AcceptMsg_WithKnownSystems)
{
    TestEndpoint endpoint;
    buffer test_msg;
    test_msg.curr.msg_id = 1;

    // add a system to the list of connected systems
    std::vector<uint16_t> sys_comp_ids{};
    sys_comp_ids.push_back(build_sys_comp_id(12, 1));
    endpoint.set_sys_comp_ids(sys_comp_ids);

    // reject message with same source address
    test_msg.curr.src_sysid = 12;
    test_msg.curr.src_compid = 1;
    test_msg.curr.target_sysid = -1;
    test_msg.curr.target_compid = -1;
    EXPECT_EQ(endpoint.accept_msg(&test_msg), Endpoint::AcceptState::Rejected);

    // accept message with no target address
    test_msg.curr.src_sysid = 1;
    test_msg.curr.src_compid = 1;
    test_msg.curr.target_sysid = -1;
    test_msg.curr.target_compid = -1;
    EXPECT_EQ(endpoint.accept_msg(&test_msg), Endpoint::AcceptState::Accepted);

    // accept message with broadcast target address
    test_msg.curr.src_sysid = 1;
    test_msg.curr.src_compid = 1;
    test_msg.curr.target_sysid = 0;
    test_msg.curr.target_compid = 0;
    EXPECT_EQ(endpoint.accept_msg(&test_msg), Endpoint::AcceptState::Accepted);

    // accept message with broadcast target component address
    test_msg.curr.src_sysid = 1;
    test_msg.curr.src_compid = 1;
    test_msg.curr.target_sysid = 12;
    test_msg.curr.target_compid = 0;
    EXPECT_EQ(endpoint.accept_msg(&test_msg), Endpoint::AcceptState::Accepted);

    // reject message with other target address
    test_msg.curr.src_sysid = 1;
    test_msg.curr.src_compid = 1;
    test_msg.curr.target_sysid = 12;
    test_msg.curr.target_compid = 190;
    EXPECT_EQ(endpoint.accept_msg(&test_msg), Endpoint::AcceptState::Rejected);

    test_msg.curr.src_sysid = 1;
    test_msg.curr.src_compid = 1;
    test_msg.curr.target_sysid = 13;
    test_msg.curr.target_compid = 0;
    EXPECT_EQ(endpoint.accept_msg(&test_msg), Endpoint::AcceptState::Rejected);
}

TEST(EndpointTest, AcceptMsg_OutMsgIdFilter)
{
    TestEndpoint endpoint;
    buffer test_msg;

    // broadcast message should normally be accepted
    test_msg.curr.src_sysid = 1;
    test_msg.curr.src_compid = 1;
    test_msg.curr.target_sysid = -1;
    test_msg.curr.target_compid = -1;

    // only allow heartbeat messages
    endpoint.filter_add_allowed_out_msg_id(1);

    // accept message with allowed message ID
    test_msg.curr.msg_id = 1;
    EXPECT_EQ(endpoint.accept_msg(&test_msg), Endpoint::AcceptState::Accepted);

    // reject message with other message IDs
    test_msg.curr.msg_id = 2;
    EXPECT_EQ(endpoint.accept_msg(&test_msg), Endpoint::AcceptState::Filtered);
    test_msg.curr.msg_id = 255;
    EXPECT_EQ(endpoint.accept_msg(&test_msg), Endpoint::AcceptState::Filtered);
    test_msg.curr.msg_id = 368;
    EXPECT_EQ(endpoint.accept_msg(&test_msg), Endpoint::AcceptState::Filtered);
}

TEST(EndpointTest, BlockMsg_OutMsgIdFilter)
{
    TestEndpoint endpoint;
    buffer test_msg;

    // broadcast message should normally be accepted
    test_msg.curr.src_sysid = 1;
    test_msg.curr.src_compid = 1;
    test_msg.curr.target_sysid = -1;
    test_msg.curr.target_compid = -1;

    // Block specific (100) msg id messages
    endpoint.filter_add_blocked_out_msg_id(100);

    // Make sure that the message with the blocked message ID is acutally filtered
    test_msg.curr.msg_id = 100;
    EXPECT_EQ(endpoint.accept_msg(&test_msg), Endpoint::AcceptState::Filtered);

    // accept message with other message IDs
    test_msg.curr.msg_id = 2;
    EXPECT_EQ(endpoint.accept_msg(&test_msg), Endpoint::AcceptState::Accepted);
    test_msg.curr.msg_id = 255;
    EXPECT_EQ(endpoint.accept_msg(&test_msg), Endpoint::AcceptState::Accepted);
    test_msg.curr.msg_id = 368;
    EXPECT_EQ(endpoint.accept_msg(&test_msg), Endpoint::AcceptState::Accepted);
}

TEST(EndpointTest, AcceptMsg_OutCompFilter)
{
    TestEndpoint endpoint;
    buffer test_msg;

    // broadcast message should normally be accepted
    test_msg.curr.msg_id = 1;
    test_msg.curr.src_sysid = 1;
    test_msg.curr.target_sysid = -1;
    test_msg.curr.target_compid = -1;

    // only allow heartbeat messages
    endpoint.filter_add_allowed_out_src_comp(1);

    // accept message with allowed source component ID
    test_msg.curr.src_compid = 1;
    EXPECT_EQ(endpoint.accept_msg(&test_msg), Endpoint::AcceptState::Accepted);

    // reject message with other source component IDs
    test_msg.curr.src_compid = 2;
    EXPECT_EQ(endpoint.accept_msg(&test_msg), Endpoint::AcceptState::Filtered);
    test_msg.curr.src_compid = 255;
    EXPECT_EQ(endpoint.accept_msg(&test_msg), Endpoint::AcceptState::Filtered);
}

TEST(EndpointTest, BlockMsg_OutCompFilter)
{
    TestEndpoint endpoint;
    buffer test_msg;

    // broadcast message should normally be accepted
    test_msg.curr.msg_id = 1;
    test_msg.curr.src_sysid = 1;
    test_msg.curr.target_sysid = -1;
    test_msg.curr.target_compid = -1;

    // only block messages with source component 100
    endpoint.filter_add_blocked_out_src_comp(100);

    // reject message with blocked source component ID
    test_msg.curr.src_compid = 100;
    EXPECT_EQ(endpoint.accept_msg(&test_msg), Endpoint::AcceptState::Filtered);

    // accept message with other source component IDs
    test_msg.curr.src_compid = 2;
    EXPECT_EQ(endpoint.accept_msg(&test_msg), Endpoint::AcceptState::Accepted);
    test_msg.curr.src_compid = 255;
    EXPECT_EQ(endpoint.accept_msg(&test_msg), Endpoint::AcceptState::Accepted);
}

TEST(EndpointTest, AcceptMsg_OutSysFilter)
{
    TestEndpoint endpoint;
    buffer test_msg;

    // broadcast message should normally be accepted
    test_msg.curr.msg_id = 1;
    test_msg.curr.src_compid = 1;
    test_msg.curr.target_sysid = -1;
    test_msg.curr.target_compid = -1;

    // only allow heartbeat messages
    endpoint.filter_add_allowed_out_src_sys(42);

    // accept message with allowed source system ID
    test_msg.curr.src_sysid = 42;
    EXPECT_EQ(endpoint.accept_msg(&test_msg), Endpoint::AcceptState::Accepted);

    // reject message with other source system IDs
    test_msg.curr.src_sysid = 2;
    EXPECT_EQ(endpoint.accept_msg(&test_msg), Endpoint::AcceptState::Filtered);
    test_msg.curr.src_sysid = 255;
    EXPECT_EQ(endpoint.accept_msg(&test_msg), Endpoint::AcceptState::Filtered);
}

TEST(EndpointTest, BlockMsg_OutSysFilter)
{
    TestEndpoint endpoint;
    buffer test_msg;

    // broadcast message should normally be accepted
    test_msg.curr.msg_id = 1;
    test_msg.curr.src_compid = 1;
    test_msg.curr.target_sysid = -1;
    test_msg.curr.target_compid = -1;

    // only block specific (42) system id
    endpoint.filter_add_blocked_out_src_sys(42);

    // block message with blocked source system ID
    test_msg.curr.src_sysid = 42;
    EXPECT_EQ(endpoint.accept_msg(&test_msg), Endpoint::AcceptState::Filtered);

    // accept message with other source system IDs
    test_msg.curr.src_sysid = 2;
    EXPECT_EQ(endpoint.accept_msg(&test_msg), Endpoint::AcceptState::Accepted);
    test_msg.curr.src_sysid = 255;
    EXPECT_EQ(endpoint.accept_msg(&test_msg), Endpoint::AcceptState::Accepted);
}

TEST(EndpointTest, AcceptMsg_InMsgIdFilter)
{
    TestEndpoint endpoint;
    buffer test_msg;

    // broadcast message should normally be accepted
    test_msg.curr.src_sysid = 1;
    test_msg.curr.src_compid = 1;
    test_msg.curr.target_sysid = -1;
    test_msg.curr.target_compid = -1;

    // only allow heartbeat messages
    endpoint.filter_add_allowed_in_msg_id(1);

    // accept message with allowed message ID
    test_msg.curr.msg_id = 1;
    EXPECT_EQ(endpoint.allowed_by_incoming_filters(&test_msg), true);

    // reject message with other message IDs
    test_msg.curr.msg_id = 2;
    EXPECT_EQ(endpoint.allowed_by_incoming_filters(&test_msg), false);
    test_msg.curr.msg_id = 255;
    EXPECT_EQ(endpoint.allowed_by_incoming_filters(&test_msg), false);
    test_msg.curr.msg_id = 368;
    EXPECT_EQ(endpoint.allowed_by_incoming_filters(&test_msg), false);
}

TEST(EndpointTest, BlockMsg_InMsgIdFilter)
{
    TestEndpoint endpoint;
    buffer test_msg;

    // broadcast message should normally be accepted
    test_msg.curr.src_sysid = 1;
    test_msg.curr.src_compid = 1;
    test_msg.curr.target_sysid = -1;
    test_msg.curr.target_compid = -1;

    // Block specific (78) incoming msg id's
    endpoint.filter_add_blocked_in_msg_id(78);

    // reject message with blocked message ID
    test_msg.curr.msg_id = 78;
    EXPECT_EQ(endpoint.allowed_by_incoming_filters(&test_msg), false);

    // accept message with other message IDs
    test_msg.curr.msg_id = 2;
    EXPECT_EQ(endpoint.allowed_by_incoming_filters(&test_msg), true);
    test_msg.curr.msg_id = 255;
    EXPECT_EQ(endpoint.allowed_by_incoming_filters(&test_msg), true);
    test_msg.curr.msg_id = 368;
    EXPECT_EQ(endpoint.allowed_by_incoming_filters(&test_msg), true);
}

TEST(EndpointTest, AcceptMsg_InCompFilter)
{
    TestEndpoint endpoint;
    buffer test_msg;

    // broadcast message should normally be accepted
    test_msg.curr.msg_id = 1;
    test_msg.curr.src_sysid = 1;
    test_msg.curr.target_sysid = -1;
    test_msg.curr.target_compid = -1;

    // only allow heartbeat messages
    endpoint.filter_add_allowed_in_src_comp(1);

    // accept message with allowed source component ID
    test_msg.curr.src_compid = 1;
    EXPECT_EQ(endpoint.allowed_by_incoming_filters(&test_msg), true);

    // reject message with other source component IDs
    test_msg.curr.src_compid = 2;
    EXPECT_EQ(endpoint.allowed_by_incoming_filters(&test_msg), false);
    test_msg.curr.src_compid = 255;
    EXPECT_EQ(endpoint.allowed_by_incoming_filters(&test_msg), false);
}

TEST(EndpointTest, BlockMsg_InCompFilter)
{
    TestEndpoint endpoint;
    buffer test_msg;

    // broadcast message should normally be accepted
    test_msg.curr.msg_id = 1;
    test_msg.curr.src_sysid = 1;
    test_msg.curr.target_sysid = -1;
    test_msg.curr.target_compid = -1;

    // Block messages with specific (96) component id
    endpoint.filter_add_blocked_in_src_comp(96);

    // reject message with blocked source component ID
    test_msg.curr.src_compid = 96;
    EXPECT_EQ(endpoint.allowed_by_incoming_filters(&test_msg), false);

    // accept message with other source component IDs
    test_msg.curr.src_compid = 2;
    EXPECT_EQ(endpoint.allowed_by_incoming_filters(&test_msg), true);
    test_msg.curr.src_compid = 255;
    EXPECT_EQ(endpoint.allowed_by_incoming_filters(&test_msg), true);
}

TEST(EndpointTest, AcceptMsg_InSysFilter)
{
    TestEndpoint endpoint;
    buffer test_msg;

    // broadcast message should normally be accepted
    test_msg.curr.msg_id = 1;
    test_msg.curr.src_compid = 1;
    test_msg.curr.target_sysid = -1;
    test_msg.curr.target_compid = -1;

    // only allow heartbeat messages
    endpoint.filter_add_allowed_in_src_sys(23);

    // accept message with allowed source component ID
    test_msg.curr.src_sysid = 23;
    EXPECT_EQ(endpoint.allowed_by_incoming_filters(&test_msg), true);

    // reject message with other source component IDs
    test_msg.curr.src_sysid = 2;
    EXPECT_EQ(endpoint.allowed_by_incoming_filters(&test_msg), false);
    test_msg.curr.src_sysid = 255;
    EXPECT_EQ(endpoint.allowed_by_incoming_filters(&test_msg), false);
}

TEST(EndpointTest, BlockMsg_InSysFilter)
{
    TestEndpoint endpoint;
    buffer test_msg;

    // broadcast message should normally be accepted
    test_msg.curr.msg_id = 1;
    test_msg.curr.src_compid = 1;
    test_msg.curr.target_sysid = -1;
    test_msg.curr.target_compid = -1;

    // Block incoming messages from specific (23) system id
    endpoint.filter_add_blocked_in_src_sys(23);

    // accept message with allowed source component ID
    test_msg.curr.src_sysid = 23;
    EXPECT_EQ(endpoint.allowed_by_incoming_filters(&test_msg), false);

    // reject message with other source component IDs
    test_msg.curr.src_sysid = 2;
    EXPECT_EQ(endpoint.allowed_by_incoming_filters(&test_msg), true);
    test_msg.curr.src_sysid = 255;
    EXPECT_EQ(endpoint.allowed_by_incoming_filters(&test_msg), true);
}

/**
 * TX queue (Endpoint base class)
 */

// Scripted-transport endpoint: _write_raw pops one result per call from `script` (positive N =
// accept up to N bytes, negative = that error; empty script = accept everything) and records
// accepted bytes in `written`, so tests can assert byte-exact, in-order delivery. With
// `close_on_error` set, an error also closes the fd and clears the queue, the way a
// reconnecting transport does.
class TxQueueTestEndpoint : public Endpoint {
public:
    TxQueueTestEndpoint()
        : Endpoint{"Test", "txqueue"}
    {
        fd = ::open("/dev/null", O_RDWR); // handle_canwrite() treats fd < 0 as "closed"
    }
    ~TxQueueTestEndpoint() override
    {
        if (fd >= 0) {
            ::close(fd);
        }
    }

    int write_msg(const struct buffer *pbuf) override { return _tx_send_or_queue(pbuf); }
    ssize_t _read_msg(uint8_t *buf, size_t len) override { return 0; }

    ssize_t _write_raw(const uint8_t *data, size_t len) override
    {
        ssize_t r = (ssize_t)len;
        if (!script.empty()) {
            r = script.front();
            script.pop_front();
        }
        if (r < 0) {
            if (close_on_error && r != -EAGAIN) {
                ::close(fd);
                fd = -1;
                _tx_queue_clear("closed on error");
            }
            return r;
        }
        if ((size_t)r > len) {
            r = (ssize_t)len;
        }
        written.insert(written.end(), data, data + r);
        return r;
    }

    std::deque<ssize_t> script;
    std::vector<uint8_t> written;
    bool close_on_error = false;

    void set_datagram() { _tx_is_stream = false; }
    bool set_queue_size(size_t bytes) { return _tx_set_queue_size(bytes); }
    size_t queued_frames() const { return _tx_pending_lens.size(); }
    size_t pending_bytes() const { return _tx_pending_bytes(); }
    uint32_t drops() const { return _stat.write.drops; }
    uint32_t queue_hwm() const { return _stat.write.queue_hwm; }
    uint16_t partial_len() const { return _tx_partial_len; }
    void clear_queue() { _tx_queue_clear("test"); }
};

static struct buffer make_frame(std::vector<uint8_t> &storage, uint8_t pattern, unsigned int len)
{
    storage.assign(len, pattern);
    struct buffer buf = {};
    buf.data = storage.data();
    buf.len = len;
    return buf;
}

TEST(TxQueueTest, FlushOnEmptyQueueReturnsZero)
{
    TxQueueTestEndpoint ep;
    EXPECT_EQ(ep.flush_pending_msgs(), 0);
}

TEST(TxQueueTest, EnqueueOnEagainThenDrainInOrder)
{
    TxQueueTestEndpoint ep;
    std::vector<uint8_t> s1, s2, s3;
    struct buffer f1 = make_frame(s1, 0x11, 100);
    struct buffer f2 = make_frame(s2, 0x22, 150);
    struct buffer f3 = make_frame(s3, 0x33, 200);

    ep.script = {-EAGAIN, -EAGAIN, -EAGAIN};
    EXPECT_EQ(ep.write_msg(&f1), -EAGAIN);
    EXPECT_EQ(ep.write_msg(&f2), -EAGAIN);
    EXPECT_EQ(ep.write_msg(&f3), -EAGAIN);
    EXPECT_EQ(ep.queued_frames(), 3u);

    EXPECT_EQ(ep.flush_pending_msgs(), 0);
    EXPECT_EQ(ep.queued_frames(), 0u);
    ASSERT_EQ(ep.written.size(), 450u);
    EXPECT_TRUE(std::equal(s1.begin(), s1.end(), ep.written.begin()));
    EXPECT_TRUE(std::equal(s2.begin(), s2.end(), ep.written.begin() + 100));
    EXPECT_TRUE(std::equal(s3.begin(), s3.end(), ep.written.begin() + 250));
}

TEST(TxQueueTest, PartialWriteResumesMidFrame)
{
    TxQueueTestEndpoint ep;
    std::vector<uint8_t> s1;
    struct buffer f1 = make_frame(s1, 0x44, 200);

    ep.script = {60}; // direct send takes 60 bytes; the tail must be buffered, never discarded
    EXPECT_EQ(ep.write_msg(&f1), -EAGAIN);
    EXPECT_EQ(ep.partial_len(), 140);

    ep.script = {50, -EAGAIN}; // resume makes progress, then blocks again
    EXPECT_EQ(ep.flush_pending_msgs(), -EAGAIN);
    EXPECT_EQ(ep.partial_len(), 90);

    ep.script.clear();
    EXPECT_EQ(ep.flush_pending_msgs(), 0);
    EXPECT_EQ(ep.partial_len(), 0);
    ASSERT_EQ(ep.written.size(), 200u);
    EXPECT_EQ(ep.written, s1); // byte-exact reassembly: no torn or duplicated bytes
}

TEST(TxQueueTest, DatagramShortWriteConsumesWholeFrame)
{
    TxQueueTestEndpoint ep;
    ep.set_datagram();
    std::vector<uint8_t> s1, s2;
    struct buffer f1 = make_frame(s1, 0x55, 100);
    struct buffer f2 = make_frame(s2, 0x66, 100);

    ep.script = {-EAGAIN};
    EXPECT_EQ(ep.write_msg(&f1), -EAGAIN);
    ep.script = {40}; // a datagram cannot be resumed: consume it, don't stash a tail
    EXPECT_EQ(ep.flush_pending_msgs(), 0);
    EXPECT_EQ(ep.queued_frames(), 0u);
    EXPECT_EQ(ep.partial_len(), 0);

    ep.script = {40}; // short direct send behaves the same
    EXPECT_EQ(ep.write_msg(&f2), (int)f2.len);
    EXPECT_EQ(ep.partial_len(), 0);
}

TEST(TxQueueTest, OverflowDropsOldestWholeFrames)
{
    TxQueueTestEndpoint ep;
    std::vector<uint8_t> storage;

    ep.script.assign(100, -EAGAIN);
    for (unsigned int i = 0; i < 40; i++) { // 40 * 250B > 8KiB: the oldest 8 must go
        struct buffer f = make_frame(storage, (uint8_t)i, 250);
        ep.write_msg(&f);
    }
    EXPECT_EQ(ep.queued_frames(), 32u);
    EXPECT_EQ(ep.drops(), 8u);
    EXPECT_GE(ep.queue_hwm(), 8000u);

    ep.script.clear();
    EXPECT_EQ(ep.flush_pending_msgs(), 0);
    ASSERT_EQ(ep.written.size(), 32u * 250u);
    // oldest surviving frame is #8, and every frame arrives whole (uniform pattern per block)
    for (unsigned int i = 0; i < 32; i++) {
        for (unsigned int b = 0; b < 250; b++) {
            ASSERT_EQ(ep.written[i * 250 + b], (uint8_t)(i + 8));
        }
    }
}

TEST(TxQueueTest, PartialFrameSurvivesOverflow)
{
    TxQueueTestEndpoint ep;
    std::vector<uint8_t> sp, sf;
    struct buffer fp = make_frame(sp, 0x77, 200);

    ep.script = {60}; // 140-byte tail parked in the partial slot
    EXPECT_EQ(ep.write_msg(&fp), -EAGAIN);
    EXPECT_EQ(ep.partial_len(), 140);

    ep.script.assign(100, -EAGAIN);
    for (unsigned int i = 0; i < 40; i++) { // overflow the ring
        struct buffer f = make_frame(sf, (uint8_t)i, 250);
        ep.write_msg(&f);
    }
    EXPECT_EQ(ep.partial_len(), 140); // ring drops never touch the on-the-wire frame's tail
    EXPECT_GT(ep.drops(), 0u);

    ep.script.clear();
    EXPECT_EQ(ep.flush_pending_msgs(), 0);
    ASSERT_GE(ep.written.size(), 200u);
    EXPECT_TRUE(std::all_of(ep.written.begin(), ep.written.begin() + 200, [](uint8_t b) {
        return b == 0x77;
    })); // the whole frame: 60 bytes sent directly, its tail flushed before everything else
}

TEST(TxQueueTest, HandleCanwriteReportsPendingData)
{
    TxQueueTestEndpoint ep;
    std::vector<uint8_t> s;
    struct buffer f = make_frame(s, 0x12, 100);

    ep.script = {-EAGAIN, -EAGAIN};
    EXPECT_EQ(ep.write_msg(&f), -EAGAIN);
    EXPECT_TRUE(ep.handle_canwrite()); // still blocked: EPOLLOUT stays armed
    ep.script.clear();
    EXPECT_FALSE(ep.handle_canwrite()); // drained: disarm
    EXPECT_EQ(ep.written, s);
}

TEST(TxQueueTest, HardWriteErrorClearsQueue)
{
    TxQueueTestEndpoint ep;
    std::vector<uint8_t> s1, s2, s3;
    struct buffer f1 = make_frame(s1, 0x11, 100);
    struct buffer f2 = make_frame(s2, 0x22, 100);
    struct buffer f3 = make_frame(s3, 0x33, 100);

    ep.script = {60, -EAGAIN}; // f1 leaves a tail in the partial slot, f2 is queued whole
    EXPECT_EQ(ep.write_msg(&f1), -EAGAIN);
    EXPECT_EQ(ep.write_msg(&f2), -EAGAIN);
    ASSERT_EQ(ep.partial_len(), 40);
    ASSERT_EQ(ep.queued_frames(), 1u);

    // the flush ahead of f3 hits a hard error: everything pending is dropped (a failing link's
    // backlog is stale), f3 is neither sent nor queued, and the error reaches the caller so
    // route_msg() can act on it
    ep.script = {-EPIPE};
    EXPECT_EQ(ep.write_msg(&f3), -EPIPE);
    EXPECT_EQ(ep.partial_len(), 0);
    EXPECT_EQ(ep.queued_frames(), 0u);
    EXPECT_EQ(ep.pending_bytes(), 0u);
    EXPECT_EQ(ep.drops(), 2u);
    EXPECT_EQ(ep.written.size(), 60u);

    // nothing is pending afterwards, so EPOLLOUT must not stay armed
    EXPECT_FALSE(ep.handle_canwrite());
}

TEST(TxQueueTest, ErrorThatClosesFdIsNotDoubleCounted)
{
    TxQueueTestEndpoint ep;
    ep.close_on_error = true;
    std::vector<uint8_t> s1, s2;
    struct buffer f1 = make_frame(s1, 0x31, 100);
    struct buffer f2 = make_frame(s2, 0x32, 100);

    ep.script = {-EAGAIN, -EAGAIN};
    EXPECT_EQ(ep.write_msg(&f1), -EAGAIN);
    EXPECT_EQ(ep.write_msg(&f2), -EAGAIN);
    ASSERT_EQ(ep.queued_frames(), 2u);

    // the transport closes and clears on the error (as TCP does when it schedules a
    // reconnect): the flush must not count those frames a second time, and must report
    // "nothing to re-arm" rather than let the mainloop poke the dead fd
    ep.script = {-EPIPE};
    EXPECT_TRUE(ep.handle_canwrite());
    EXPECT_EQ(ep.fd, -1);
    EXPECT_EQ(ep.queued_frames(), 0u);
    EXPECT_EQ(ep.drops(), 2u);
}

TEST(TxQueueTest, OversizedFrameIsRejected)
{
    TxQueueTestEndpoint ep;
    std::vector<uint8_t> s;
    struct buffer big = make_frame(s, 0x99, MAVLINK_MAX_PACKET_LEN + 1);

    EXPECT_EQ(ep.write_msg(&big), -EMSGSIZE);
    EXPECT_EQ(ep.drops(), 1u);
    EXPECT_EQ(ep.queued_frames(), 0u);
    EXPECT_TRUE(ep.written.empty());

    struct buffer max = make_frame(s, 0x98, MAVLINK_MAX_PACKET_LEN);
    ep.script = {-EAGAIN};
    EXPECT_EQ(ep.write_msg(&max), -EAGAIN); // the largest legal frame still queues
    EXPECT_EQ(ep.queued_frames(), 1u);
    EXPECT_EQ(ep.drops(), 1u);
}

TEST(TxQueueTest, ZeroLengthWriteIsTreatedAsWouldBlock)
{
    TxQueueTestEndpoint ep;
    std::vector<uint8_t> s;
    struct buffer f = make_frame(s, 0x21, 200);

    ep.script = {0}; // no progress on the direct send: the frame is queued whole
    EXPECT_EQ(ep.write_msg(&f), -EAGAIN);
    EXPECT_EQ(ep.queued_frames(), 1u);
    EXPECT_EQ(ep.partial_len(), 0);

    ep.script = {60};
    EXPECT_EQ(ep.flush_pending_msgs(), -EAGAIN);
    ASSERT_EQ(ep.partial_len(), 140);
    ep.script = {0}; // no progress mid-frame: the tail waits for EPOLLOUT instead of spinning
    EXPECT_EQ(ep.flush_pending_msgs(), -EAGAIN);
    EXPECT_EQ(ep.partial_len(), 140);
    EXPECT_EQ(ep.drops(), 0u);

    ep.script.clear();
    EXPECT_EQ(ep.flush_pending_msgs(), 0);
    EXPECT_EQ(ep.written, s);
}

TEST(TxQueueTest, ClearCountsDroppedFrames)
{
    TxQueueTestEndpoint ep;
    std::vector<uint8_t> s;

    ep.script.assign(10, -EAGAIN);
    for (unsigned int i = 0; i < 3; i++) {
        struct buffer f = make_frame(s, (uint8_t)i, 100);
        ep.write_msg(&f);
    }
    EXPECT_EQ(ep.queued_frames(), 3u);

    ep.clear_queue();
    EXPECT_EQ(ep.queued_frames(), 0u);
    EXPECT_EQ(ep.pending_bytes(), 0u);
    EXPECT_EQ(ep.drops(), 3u);
}

TEST(TxQueueTest, ConfiguredSizeBoundsTheQueue)
{
    TxQueueTestEndpoint ep;
    std::vector<uint8_t> storage;

    EXPECT_FALSE(ep.set_queue_size(TX_QUEUE_SIZE_MIN - 1)); // must always hold a whole frame
    EXPECT_FALSE(ep.set_queue_size(TX_QUEUE_SIZE_MAX + 1));
    ASSERT_TRUE(ep.set_queue_size(1000));

    ep.script.assign(100, -EAGAIN);
    for (unsigned int i = 0; i < 5; i++) { // 5 * 250B fit the default 8 KiB, not 1000B
        struct buffer f = make_frame(storage, (uint8_t)i, 250);
        EXPECT_EQ(ep.write_msg(&f), -EAGAIN);
    }
    EXPECT_EQ(ep.queued_frames(), 4u);
    EXPECT_EQ(ep.drops(), 1u);
    EXPECT_EQ(ep.queue_hwm(), 1000u);

    ep.script.clear();
    EXPECT_EQ(ep.flush_pending_msgs(), 0);
    ASSERT_EQ(ep.written.size(), 1000u);
    EXPECT_EQ(ep.written[0], 1); // frame #0 was the one dropped: oldest first
}

/**
 * UART Endpoint
 */
TEST(UartEndpointTest, Init)
{
    UartEndpoint uart{"testname"};

    EXPECT_EQ(uart.get_type(), ENDPOINT_TYPE_UART);

    // we can't call setup() without a real UART device
}

TEST(UartEndpointTest, ConfigValidateBaud)
{
    UartEndpointConfig config;
    config.device = "/dev/ttyUSB0";

    // build valid config
    config.baudrates.push_back(115200);
    EXPECT_TRUE(UartEndpoint::validate_config(config))
        << "with " << config.baudrates.size() << " bauds";

    config.baudrates.push_back(57600);
    EXPECT_TRUE(UartEndpoint::validate_config(config))
        << "with " << config.baudrates.size() << " bauds";

    // build invalid baud rate
    config.baudrates.clear();
    EXPECT_FALSE(UartEndpoint::validate_config(config))
        << "with " << config.baudrates.size() << " bauds";
}

TEST(UartEndpointTest, ConfigValidateDevice)
{
    UartEndpointConfig config;
    config.baudrates.push_back(115200);

    // build valid config
    config.device = "/dev/ttyUSB0";
    EXPECT_TRUE(UartEndpoint::validate_config(config)) << "with device " << config.device;

    // build invalid baud rate
    config.device = "";
    EXPECT_FALSE(UartEndpoint::validate_config(config)) << "with device " << config.device;
}

TEST(UartEndpointTest, ConfigValidateTxQueueSize)
{
    UartEndpointConfig config;
    config.device = "/dev/ttyUSB0";
    config.baudrates.push_back(115200);

    EXPECT_TRUE(UartEndpoint::validate_config(config)) << "with the default queue size";
    for (unsigned long size :
         {(unsigned long)TX_QUEUE_SIZE_MIN, (unsigned long)TX_QUEUE_SIZE_MAX}) {
        config.tx_queue_size = size;
        EXPECT_TRUE(UartEndpoint::validate_config(config)) << "with TxQueueSize " << size;
    }
    for (unsigned long size : {0UL, TX_QUEUE_SIZE_MIN - 1UL, TX_QUEUE_SIZE_MAX + 1UL}) {
        config.tx_queue_size = size;
        EXPECT_FALSE(UartEndpoint::validate_config(config)) << "with TxQueueSize " << size;
    }
}

class TestUartEndpoint : public UartEndpoint {
public:
    TestUartEndpoint()
        : UartEndpoint{"uart-test"}
    {
    }
    size_t queued_frames() const { return _tx_pending_lens.size(); }
    uint16_t partial_len() const { return _tx_partial_len; }
};

// Real-kernel EAGAIN: write over a socketpair with the smallest possible send buffer until the
// kernel pushes back, then drain and verify the receiver sees the exact byte stream — no torn,
// missing or duplicated frames.
TEST(UartEndpointTest, QueueOnBlockedFdPreservesStream)
{
    int sp[2];
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_STREAM, 0, sp), 0);
    int sndbuf = 1; // the kernel clamps this to its floor — never assume the resulting size
    ASSERT_EQ(setsockopt(sp[0], SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf)), 0);
    ASSERT_EQ(fcntl(sp[0], F_SETFL, O_NONBLOCK), 0);
    ASSERT_EQ(fcntl(sp[1], F_SETFL, O_NONBLOCK), 0);

    TestUartEndpoint uart{};
    uart.fd = sp[0]; // closed by the endpoint destructor

    // write distinct-patterned frames until the kernel buffer forces queueing, then a few
    // more — but stay far below queue capacity so no frame is legitimately dropped
    std::vector<uint8_t> storage, expected;
    unsigned int extra = 0;
    for (unsigned int i = 0; i < 200 && extra < 5; i++) {
        struct buffer f = make_frame(storage, (uint8_t)i, 280);
        int r = uart.write_msg(&f);
        ASSERT_TRUE(r == (int)f.len || r == -EAGAIN) << "write_msg returned " << r;
        expected.insert(expected.end(), storage.begin(), storage.end());
        if (r == -EAGAIN) {
            extra++;
        }
    }
    ASSERT_GE(extra, 5u) << "kernel never pushed back; cannot exercise the queue";

    // alternate draining the receiver and flushing until the queue is empty
    std::vector<uint8_t> received;
    uint8_t buf[4096];
    ssize_t n;
    int flush_ret = -EAGAIN;
    for (int guard = 0; guard < 1000 && flush_ret == -EAGAIN; guard++) {
        while ((n = read(sp[1], buf, sizeof(buf))) > 0) {
            received.insert(received.end(), buf, buf + n);
        }
        flush_ret = uart.flush_pending_msgs();
    }
    EXPECT_EQ(flush_ret, 0);
    EXPECT_EQ(uart.queued_frames(), 0u);
    EXPECT_EQ(uart.partial_len(), 0);
    while ((n = read(sp[1], buf, sizeof(buf))) > 0) {
        received.insert(received.end(), buf, buf + n);
    }

    EXPECT_EQ(received, expected);
    close(sp[1]);
}

/**
 * UDP Endpoint
 */
TEST(UdpEndpointTest, Init)
{
    UdpEndpoint udp{"testname"};

    EXPECT_EQ(udp.get_type(), ENDPOINT_TYPE_UDP);

    // TODO: create a temporary UDP socket to connect to
}

class TestUdpEndpoint : public UdpEndpoint {
public:
    TestUdpEndpoint()
        : UdpEndpoint{"udp-test"}
    {
    }
    using UdpEndpoint::_read_msg;
    using UdpEndpoint::open;
    using UdpEndpoint::read_msg;
    uint32_t incomplete_msgs() const { return _incomplete_msgs; }
};

// The endpoint is opened in server mode on an ephemeral loopback port; datagrams are delivered
// to it through the sender socket. open() makes the receiver non-blocking, so a read finding
// nothing pending fails the test instead of hanging it.
class UdpReadTest : public ::testing::Test {
protected:
    TestUdpEndpoint udp{};
    int sender = -1;
    struct sockaddr_in addr = {};

    void SetUp() override
    {
        ASSERT_TRUE(udp.open("127.0.0.1", 0, UdpEndpointConfig::Mode::Server));
        socklen_t addrlen = sizeof(addr);
        ASSERT_EQ(getsockname(udp.fd, (struct sockaddr *)&addr, &addrlen), 0);

        sender = socket(AF_INET, SOCK_DGRAM, 0);
        ASSERT_GE(sender, 0);
    }

    void TearDown() override
    {
        close(sender);
        close(udp.fd);
        udp.fd = -1;
    }

    void send(const uint8_t *data, size_t len)
    {
        EXPECT_EQ(sendto(sender, data, len, 0, (struct sockaddr *)&addr, sizeof(addr)),
                  (ssize_t)len);
    }
};

TEST_F(UdpReadTest, TruncatedDatagramIsDropped)
{
    uint8_t big[128];
    memset(big, 0xAB, sizeof(big));
    send(big, sizeof(big));

    // a datagram larger than the buffer must be dropped entirely, not truncated into the parser
    uint8_t small[64];
    EXPECT_EQ(udp._read_msg(small, sizeof(small)), 0);
    EXPECT_EQ(udp.incomplete_msgs(), 1U);

    // a datagram that fits is still delivered
    send(big, 32);
    EXPECT_EQ(udp._read_msg(small, sizeof(small)), 32);
    EXPECT_EQ(udp.incomplete_msgs(), 1U);
}

TEST_F(UdpReadTest, TruncatedDatagramDiscardsLeftoverPartialFrame)
{
    struct buffer buf = {};

    // junk with a frame start byte near its end: the parser keeps the bytes from the start byte
    // on, waiting for the rest of a header that no later datagram will ever deliver
    const uint8_t junk[] = {0x00, 0xFD, 0x00, 0x00};
    send(junk, sizeof(junk));
    EXPECT_EQ(udp.read_msg(&buf), 0);
    ASSERT_EQ(udp.rx_buf.len, 3U);

    // a datagram that fits an empty buffer no longer fits next to the leftover and is dropped;
    // the leftover has to go with it, or every later datagram of this size is dropped as well
    uint8_t datagram[RX_BUF_MAX_SIZE - 2] = {};
    send(datagram, sizeof(datagram));
    EXPECT_EQ(udp.read_msg(&buf), 0);
    EXPECT_EQ(udp.incomplete_msgs(), 1U);
    EXPECT_EQ(udp.rx_buf.len, 0U);

    // MAVLink v2 HEARTBEAT (msgid 0) from 1/1, heading a datagram of the same size
    static const uint8_t heartbeat[]
        = {0xFD, 0x09, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00,
           0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x03, 0x03, 0x83, 0x46};
    memcpy(datagram, heartbeat, sizeof(heartbeat));
    send(datagram, sizeof(datagram));
    EXPECT_EQ(udp.read_msg(&buf), Endpoint::ReadOk);
    EXPECT_EQ(buf.curr.msg_id, 0U);
    EXPECT_EQ(udp.incomplete_msgs(), 1U);
}

TEST(UdpEndpointTest, ConfigValidateAddress)
{
    UdpEndpointConfig config;
    config.port = 14550;
    config.mode = UdpEndpointConfig::Mode::Client;

    // build valid config
    config.address = "127.0.0.1";
    EXPECT_TRUE(UdpEndpoint::validate_config(config)) << "with address " << config.address;

    config.address = "[::1]";
    EXPECT_TRUE(UdpEndpoint::validate_config(config)) << "with address " << config.address;

    // build invalid IP address
    config.address = "";
    EXPECT_FALSE(UdpEndpoint::validate_config(config)) << "with address " << config.address;

    config.address = "[127.0.0.1]";
    EXPECT_FALSE(UdpEndpoint::validate_config(config)) << "with address " << config.address;

    config.address = "::1";
    EXPECT_FALSE(UdpEndpoint::validate_config(config)) << "with address " << config.address;
}

class UdpEndpointConfigPortTestFixture : public ::testing::TestWithParam<UdpEndpointConfig::Mode> {
};

TEST_P(UdpEndpointConfigPortTestFixture, UdpPortRangeCheck)
{
    UdpEndpointConfig config;
    config.address = "127.0.0.1";

    // build valid config
    config.mode = GetParam();
    config.port = 14550;
    EXPECT_TRUE(UdpEndpoint::validate_config(config))
        << "with port " << std::to_string(config.port);

    config.port = ULONG_MAX - 1;
    EXPECT_TRUE(UdpEndpoint::validate_config(config))
        << "with port " << std::to_string(config.port);

    config.port = 1;
    EXPECT_TRUE(UdpEndpoint::validate_config(config))
        << "with port " << std::to_string(config.port);

    // build invalid port
    config.port = ULONG_MAX;
    EXPECT_FALSE(UdpEndpoint::validate_config(config))
        << "with port " << std::to_string(config.port);

    config.port = 0;
    EXPECT_FALSE(UdpEndpoint::validate_config(config))
        << "with port " << std::to_string(config.port);
}

INSTANTIATE_TEST_SUITE_P(UdpEndpointTest, UdpEndpointConfigPortTestFixture,
                         ::testing::Values(UdpEndpointConfig::Mode::Client,
                                           UdpEndpointConfig::Mode::Server));

TEST(UdpEndpointTest, ConfigValidateMode)
{
    UdpEndpointConfig config;
    config.address = "127.0.0.1";
    config.port = 14550;

    // build valid config
    config.mode = UdpEndpointConfig::Mode::Client;
    EXPECT_TRUE(UdpEndpoint::validate_config(config)) << "with Client mode";

    config.mode = UdpEndpointConfig::Mode::Server;
    EXPECT_TRUE(UdpEndpoint::validate_config(config)) << "with Server mode";

    // build invalid mode
    config.mode = UdpEndpointConfig::Mode::Undefined;
    EXPECT_FALSE(UdpEndpoint::validate_config(config)) << "with Undefined mode";
}

TEST(UdpEndpointTest, ConfigValidateTxQueueSize)
{
    UdpEndpointConfig config;
    config.address = "127.0.0.1";
    config.port = 14550;
    config.mode = UdpEndpointConfig::Mode::Client;

    EXPECT_TRUE(UdpEndpoint::validate_config(config)) << "with the default queue size";
    for (unsigned long size :
         {(unsigned long)TX_QUEUE_SIZE_MIN, (unsigned long)TX_QUEUE_SIZE_MAX}) {
        config.tx_queue_size = size;
        EXPECT_TRUE(UdpEndpoint::validate_config(config)) << "with TxQueueSize " << size;
    }
    for (unsigned long size : {0UL, TX_QUEUE_SIZE_MIN - 1UL, TX_QUEUE_SIZE_MAX + 1UL}) {
        config.tx_queue_size = size;
        EXPECT_FALSE(UdpEndpoint::validate_config(config)) << "with TxQueueSize " << size;
    }
}

class UdpTxTestEndpoint : public UdpEndpoint {
public:
    UdpTxTestEndpoint()
        : UdpEndpoint{"udp-tx-test"}
    {
    }
    using UdpEndpoint::open;
    size_t queued_frames() const { return _tx_pending_lens.size(); }
};

// UDP datagram-queue semantics (enqueue on EAGAIN, whole-datagram resend, drop-oldest) are
// covered by the scripted TxQueueTest suite: a real loopback UDP socket cannot produce a
// deterministic EAGAIN (loopback drops instead of blocking). These are transport smoke tests.
class UdpTxTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        Mainloop &mainloop = Mainloop::init();
        // returns -EBUSY when a previous test left the epoll fd open -- that instance works fine
        mainloop.open();
    }
    void TearDown() override { Mainloop::teardown(); }
};

TEST_F(UdpTxTest, ClientModeSendsWholeDatagrams)
{
    int rx = socket(AF_INET, SOCK_DGRAM, 0);
    ASSERT_GE(rx, 0);
    struct sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    ASSERT_EQ(bind(rx, (struct sockaddr *)&addr, sizeof(addr)), 0);
    socklen_t alen = sizeof(addr);
    ASSERT_EQ(getsockname(rx, (struct sockaddr *)&addr, &alen), 0);

    UdpTxTestEndpoint udp{};
    ASSERT_TRUE(udp.open("127.0.0.1", ntohs(addr.sin_port), UdpEndpointConfig::Mode::Client));

    std::vector<uint8_t> s;
    struct buffer f = make_frame(s, 0x5A, 200);
    EXPECT_EQ(udp.write_msg(&f), (int)f.len);
    EXPECT_EQ(udp.flush_pending_msgs(), 0);
    EXPECT_EQ(udp.queued_frames(), 0u);

    struct pollfd pfd = {};
    pfd.fd = rx;
    pfd.events = POLLIN;
    ASSERT_EQ(poll(&pfd, 1, 2000), 1);
    uint8_t rbuf[512];
    ssize_t n = recv(rx, rbuf, sizeof(rbuf), 0);
    ASSERT_EQ(n, 200);
    EXPECT_TRUE(std::all_of(rbuf, rbuf + 200, [](uint8_t b) { return b == 0x5A; }));
    close(rx);
}

TEST_F(UdpTxTest, ServerModeWithoutPeerWritesNothing)
{
    UdpTxTestEndpoint udp{};
    ASSERT_TRUE(udp.open("127.0.0.1", 0, UdpEndpointConfig::Mode::Server));

    std::vector<uint8_t> s;
    struct buffer f = make_frame(s, 0x5B, 100);
    EXPECT_EQ(udp.write_msg(&f), 0); // no peer has spoken yet: nothing to write for
    EXPECT_EQ(udp.queued_frames(), 0u);
}

/**
 * TCP Endpoint
 */
TEST(TcpEndpointTest, Init)
{
    TcpEndpoint tcp{"testname"};

    EXPECT_EQ(tcp.get_type(), ENDPOINT_TYPE_TCP);
    EXPECT_TRUE(tcp.is_valid());
    EXPECT_FALSE(tcp.is_critical());

    // we can't call setup() without a TCP server to connect to
}

class NoDelayTcpEndpoint : public TcpEndpoint {
public:
    NoDelayTcpEndpoint()
        : TcpEndpoint{"tcp-test"}
    {
    }
    using TcpEndpoint::open;
};

TEST(TcpEndpointTest, ClientSetsTcpNoDelay)
{
    // open() registers the socket with the Mainloop once the connect is asynchronous, and
    // close() always unregisters it, so run the test against an initialised instance
    Mainloop &mainloop = Mainloop::init();
    mainloop.open();

    // listener on an ephemeral loopback port: open() connects to it right away
    int listener = socket(AF_INET, SOCK_STREAM, 0);
    ASSERT_GE(listener, 0);
    struct sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    ASSERT_EQ(bind(listener, (struct sockaddr *)&addr, sizeof(addr)), 0);
    socklen_t addrlen = sizeof(addr);
    ASSERT_EQ(getsockname(listener, (struct sockaddr *)&addr, &addrlen), 0);
    ASSERT_EQ(listen(listener, 1), 0);

    NoDelayTcpEndpoint tcp{};
    ASSERT_TRUE(tcp.open("127.0.0.1", ntohs(addr.sin_port)));

    // the option is set before connect(), so it is readable whether or not the handshake has
    // completed by now
    int nodelay = 0;
    socklen_t len = sizeof(nodelay);
    EXPECT_EQ(getsockopt(tcp.fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, &len), 0);
    EXPECT_EQ(nodelay, 1);

    tcp.close();
    close(listener);
    Mainloop::teardown();
}

TEST(TcpEndpointTest, ConfigValidateAddress)
{
    TcpEndpointConfig config;
    config.port = 14550;

    // build valid config
    config.address = "127.0.0.1";
    EXPECT_TRUE(TcpEndpoint::validate_config(config)) << "with address " << config.address;

    config.address = "[::1]";
    EXPECT_TRUE(TcpEndpoint::validate_config(config)) << "with address " << config.address;

    // build invalid IP address
    config.address = "";
    EXPECT_FALSE(TcpEndpoint::validate_config(config)) << "with address " << config.address;

    config.address = "[127.0.0.1]";
    EXPECT_FALSE(TcpEndpoint::validate_config(config)) << "with address " << config.address;

    config.address = "::1";
    EXPECT_FALSE(TcpEndpoint::validate_config(config)) << "with address " << config.address;
}

TEST(TcpEndpointTest, ConfigValidatePort)
{
    TcpEndpointConfig config;
    config.address = "127.0.0.1";

    // build valid config
    config.port = 14550;
    EXPECT_TRUE(TcpEndpoint::validate_config(config))
        << "with port " << std::to_string(config.port);

    config.port = ULONG_MAX - 1;
    EXPECT_TRUE(TcpEndpoint::validate_config(config))
        << "with port " << std::to_string(config.port);

    config.port = 1;
    EXPECT_TRUE(TcpEndpoint::validate_config(config))
        << "with port " << std::to_string(config.port);

    // build invalid port
    config.port = ULONG_MAX;
    EXPECT_FALSE(TcpEndpoint::validate_config(config))
        << "with port " << std::to_string(config.port);

    config.port = 0;
    EXPECT_FALSE(TcpEndpoint::validate_config(config))
        << "with port " << std::to_string(config.port);
}

class TestTcpEndpoint : public TcpEndpoint {
public:
    TestTcpEndpoint()
        : TcpEndpoint{"tcp-test"}
    {
    }
    using TcpEndpoint::open;
    bool connecting() const { return _connecting; }
};

// TCP client connect tests need a Mainloop instance: open() registers the fd with epoll, and a
// failed connect completion unregisters it again.
class TcpConnectTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        Mainloop &mainloop = Mainloop::init();
        // returns -EBUSY when a previous test left the epoll fd open -- that instance works fine
        mainloop.open();
    }
    void TearDown() override { Mainloop::teardown(); }

    // socket bound to an ephemeral loopback port, not listening; port returned by reference
    static int bind_loopback(unsigned long &port)
    {
        int fd = socket(AF_INET, SOCK_STREAM, 0);
        if (fd < 0) {
            return -1;
        }
        struct sockaddr_in addr = {};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        socklen_t addrlen = sizeof(addr);
        if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0
            || getsockname(fd, (struct sockaddr *)&addr, &addrlen) < 0) {
            close(fd);
            return -1;
        }
        port = ntohs(addr.sin_port);
        return fd;
    }

    static int open_listener(unsigned long &port, int backlog = 1)
    {
        int fd = bind_loopback(port);
        if (fd >= 0 && listen(fd, backlog) < 0) {
            close(fd);
            return -1;
        }
        return fd;
    }

    // Listener a connect to which never completes: listen(fd, 0) admits a single established
    // connection (the helper, which is never accepted) and with its accept queue full the
    // kernel drops further SYNs without answering (Linux defaults to
    // net.ipv4.tcp_abort_on_overflow=0), so a client keeps retransmitting the SYN and its
    // connect stays pending for as long as the listener and the helper are kept open.
    static int open_stalled_listener(unsigned long &port, int &helper)
    {
        int fd = open_listener(port, 0);
        if (fd < 0) {
            return -1;
        }
        struct sockaddr_in addr = {};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr.sin_port = htons(port);
        helper = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);
        if (helper < 0) {
            close(fd);
            return -1;
        }
        // the listener becomes readable once the helper's connection sits in its accept queue
        int r = connect(helper, (struct sockaddr *)&addr, sizeof(addr));
        if ((r < 0 && errno != EINPROGRESS) || !wait_for(fd, POLLIN, 2000)) {
            close(helper);
            close(fd);
            return -1;
        }
        return fd;
    }

    static bool wait_for(int fd, short events, int timeout_ms)
    {
        struct pollfd pfd = {};
        pfd.fd = fd;
        pfd.events = events;
        return poll(&pfd, 1, timeout_ms) == 1;
    }

    static bool wait_writable(int fd, int timeout_ms) { return wait_for(fd, POLLOUT, timeout_ms); }
};

TEST_F(TcpConnectTest, AsyncConnectSuccess)
{
    unsigned long port = 0;
    int listener = open_listener(port);
    ASSERT_GE(listener, 0);

    TestTcpEndpoint tcp{};
    ASSERT_TRUE(tcp.open("127.0.0.1", port));
    // a non-blocking connect is reported as in progress even on loopback (EINPROGRESS)
    ASSERT_TRUE(tcp.connecting());
    ASSERT_TRUE(wait_writable(tcp.fd, 2000));
    EXPECT_FALSE(tcp.handle_canwrite()); // false: mainloop switches EPOLLOUT -> EPOLLIN
    EXPECT_FALSE(tcp.connecting());
    EXPECT_GE(tcp.fd, 0);
    EXPECT_TRUE(tcp.is_valid());

    close(listener);
}

TEST_F(TcpConnectTest, AsyncConnectRefused)
{
    // bound but not listening: the kernel answers the SYN with a RST (ECONNREFUSED), and holding
    // the port for the whole test keeps another process from listening on it in the meantime
    unsigned long port = 0;
    int bound = bind_loopback(port);
    ASSERT_GE(bound, 0);

    TestTcpEndpoint tcp{};
    ASSERT_TRUE(tcp.open("127.0.0.1", port));
    ASSERT_TRUE(tcp.connecting());
    ASSERT_TRUE(wait_writable(tcp.fd, 2000));
    EXPECT_TRUE(tcp.handle_canwrite()); // true: fd already closed, no mod_fd wanted
    EXPECT_EQ(tcp.fd, -1);
    EXPECT_FALSE(tcp.connecting());
    EXPECT_TRUE(tcp.is_valid()); // a refused connect must not get the endpoint garbage-collected

    close(bound);
}

TEST_F(TcpConnectTest, OpenDoesNotBlock)
{
    unsigned long port = 0;
    int helper = -1;
    int listener = open_stalled_listener(port, helper);
    ASSERT_GE(listener, 0);

    TestTcpEndpoint tcp{};
    auto start = std::chrono::steady_clock::now();
    ASSERT_TRUE(tcp.open("127.0.0.1", port));
    auto elapsed = std::chrono::steady_clock::now() - start;
    EXPECT_LT(elapsed, std::chrono::seconds(1));
    EXPECT_TRUE(tcp.connecting());
    // the attempt really is stalled: still pending well after open() returned
    EXPECT_FALSE(wait_writable(tcp.fd, 100));

    close(helper);
    close(listener);
}

TEST_F(TcpConnectTest, WriteSkippedWhileConnecting)
{
    unsigned long port = 0;
    int helper = -1;
    int listener = open_stalled_listener(port, helper);
    ASSERT_GE(listener, 0);

    TestTcpEndpoint tcp{};
    ASSERT_TRUE(tcp.open("127.0.0.1", port));
    ASSERT_TRUE(tcp.connecting());

    // guards return before any buffer field is inspected
    buffer test_msg{};
    EXPECT_EQ(tcp.write_msg(&test_msg), 0);
    EXPECT_EQ(tcp.accept_msg(&test_msg), Endpoint::AcceptState::Rejected);

    close(helper);
    close(listener);
}

TEST(TcpEndpointTest, ConfigValidateTxQueueSize)
{
    TcpEndpointConfig config;
    config.address = "127.0.0.1";
    config.port = 14550;

    EXPECT_TRUE(TcpEndpoint::validate_config(config)) << "with the default queue size";
    for (unsigned long size :
         {(unsigned long)TX_QUEUE_SIZE_MIN, (unsigned long)TX_QUEUE_SIZE_MAX}) {
        config.tx_queue_size = size;
        EXPECT_TRUE(TcpEndpoint::validate_config(config)) << "with TxQueueSize " << size;
    }
    for (unsigned long size : {0UL, TX_QUEUE_SIZE_MIN - 1UL, TX_QUEUE_SIZE_MAX + 1UL}) {
        config.tx_queue_size = size;
        EXPECT_FALSE(TcpEndpoint::validate_config(config)) << "with TxQueueSize " << size;
    }
}

class TcpTxTestEndpoint : public TcpEndpoint {
public:
    TcpTxTestEndpoint()
        : TcpEndpoint{"tcp-tx-test"}
    {
    }
    size_t queued_frames() const { return _tx_pending_lens.size(); }
    uint16_t partial_len() const { return _tx_partial_len; }
    uint32_t drops() const { return _stat.write.drops; }
    void set_retry_timeout(int sec) { _retry_timeout = sec; }
    using TcpEndpoint::close;
};

// TcpEndpoint::close() (also run by the destructor) talks to the Mainloop epoll instance
class TcpTxTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        Mainloop &mainloop = Mainloop::init();
        // returns -EBUSY when a previous test left the epoll fd open -- that instance works fine
        mainloop.open();
    }
    void TearDown() override { Mainloop::teardown(); }

    static void make_blocked_pair(int sp[2])
    {
        ASSERT_EQ(socketpair(AF_UNIX, SOCK_STREAM, 0, sp), 0);
        int sndbuf = 1; // clamped to the kernel's floor
        ASSERT_EQ(setsockopt(sp[0], SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf)), 0);
        ASSERT_EQ(fcntl(sp[0], F_SETFL, O_NONBLOCK), 0);
        ASSERT_EQ(fcntl(sp[1], F_SETFL, O_NONBLOCK), 0);
    }
};

TEST_F(TcpTxTest, QueueOnBlockedFdPreservesStream)
{
    int sp[2];
    make_blocked_pair(sp);

    TcpTxTestEndpoint tcp{};
    tcp.fd = sp[0]; // closed by the endpoint destructor

    std::vector<uint8_t> storage, expected;
    unsigned int extra = 0;
    for (unsigned int i = 0; i < 200 && extra < 5; i++) {
        struct buffer f = make_frame(storage, (uint8_t)i, 280);
        int r = tcp.write_msg(&f);
        ASSERT_TRUE(r == (int)f.len || r == -EAGAIN) << "write_msg returned " << r;
        expected.insert(expected.end(), storage.begin(), storage.end());
        if (r == -EAGAIN) {
            extra++;
        }
    }
    ASSERT_GE(extra, 5u) << "kernel never pushed back; cannot exercise the queue";

    std::vector<uint8_t> received;
    uint8_t buf[4096];
    ssize_t n;
    int flush_ret = -EAGAIN;
    for (int guard = 0; guard < 1000 && flush_ret == -EAGAIN; guard++) {
        while ((n = read(sp[1], buf, sizeof(buf))) > 0) {
            received.insert(received.end(), buf, buf + n);
        }
        flush_ret = tcp.flush_pending_msgs();
    }
    EXPECT_EQ(flush_ret, 0);
    EXPECT_EQ(tcp.queued_frames(), 0u);
    EXPECT_EQ(tcp.partial_len(), 0);
    while ((n = read(sp[1], buf, sizeof(buf))) > 0) {
        received.insert(received.end(), buf, buf + n);
    }

    EXPECT_EQ(received, expected);
    close(sp[1]);
}

TEST_F(TcpTxTest, CloseClearsQueueWithAccounting)
{
    int sp[2];
    make_blocked_pair(sp);

    TcpTxTestEndpoint tcp{};
    tcp.fd = sp[0];

    std::vector<uint8_t> storage;
    unsigned int queued = 0;
    for (unsigned int i = 0; i < 200 && queued == 0; i++) {
        struct buffer f = make_frame(storage, (uint8_t)i, 280);
        tcp.write_msg(&f);
        queued = tcp.queued_frames();
    }
    ASSERT_GT(tcp.queued_frames(), 0u);

    tcp.close();
    EXPECT_EQ(tcp.queued_frames(), 0u);
    EXPECT_EQ(tcp.partial_len(), 0);
    EXPECT_GT(tcp.drops(), 0u); // cleared frames are accounted as drops
    close(sp[1]);
}

TEST_F(TcpTxTest, EpipeInvalidatesEndpointWithoutRetry)
{
    // not required any more (send() passes MSG_NOSIGNAL), kept so a regression there shows up
    // as a failed expectation and not as a killed test process
    signal(SIGPIPE, SIG_IGN);

    int sp[2];
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_STREAM, 0, sp), 0);
    close(sp[1]); // peer gone: next send gets EPIPE

    TcpTxTestEndpoint tcp{};
    tcp.fd = sp[0];

    std::vector<uint8_t> s;
    struct buffer f = make_frame(s, 0x42, 100);
    EXPECT_EQ(tcp.write_msg(&f), -EPIPE);
    EXPECT_FALSE(tcp.is_valid()); // RetryTimeout defaults to 0 for a bare endpoint
    EXPECT_EQ(tcp.queued_frames(), 0u);
}

// EPIPE with RetryTimeout > 0 while frames are pending: the reconnect path closes the fd and
// clears the queue from inside the flush. Every pending frame must be counted exactly once, and
// handle_canwrite() must not ask the mainloop to re-arm an fd that no longer exists.
TEST_F(TcpTxTest, EpipeDuringFlushWithRetryClosesAndCountsOnce)
{
    int sp[2];
    make_blocked_pair(sp);

    TcpTxTestEndpoint tcp{};
    tcp.fd = sp[0];
    tcp.set_retry_timeout(5); // the reconnect timer lands on the fixture's Mainloop

    std::vector<uint8_t> storage;
    for (unsigned int i = 0; i < 200 && tcp.queued_frames() < 3; i++) {
        struct buffer f = make_frame(storage, (uint8_t)i, 280);
        tcp.write_msg(&f);
    }
    ASSERT_EQ(tcp.queued_frames(), 3u);
    uint32_t pending = (uint32_t)tcp.queued_frames() + (tcp.partial_len() > 0 ? 1 : 0);
    uint32_t drops_before = tcp.drops();

    close(sp[1]);                       // peer gone: the next send fails with EPIPE
    EXPECT_TRUE(tcp.handle_canwrite()); // nothing to re-arm: the fd is gone
    EXPECT_EQ(tcp.fd, -1);
    EXPECT_TRUE(tcp.is_valid()); // a reconnect is scheduled, the endpoint stays
    EXPECT_EQ(tcp.queued_frames(), 0u);
    EXPECT_EQ(tcp.partial_len(), 0);
    EXPECT_EQ(tcp.drops(), drops_before + pending);
}

/**
 * Log Endpoint
 */
static int count_logs(const char *dir_path)
{
    DIR *dir = opendir(dir_path);
    if (dir == nullptr) {
        return -1;
    }
    int count = 0;
    struct dirent *ent;
    uint32_t u;
    while ((ent = readdir(dir)) != nullptr) {
        if (sscanf(ent->d_name, "%u-", &u) == 1) {
            count++;
        }
    }
    closedir(dir);
    return count;
}

// path of the log with sequence number idx in dir_path, "" when there is none
static std::string find_log(const char *dir_path, unsigned idx)
{
    std::string found;
    DIR *dir = opendir(dir_path);
    if (dir == nullptr) {
        return found;
    }
    struct dirent *ent;
    uint32_t u;
    while ((ent = readdir(dir)) != nullptr) {
        if (sscanf(ent->d_name, "%u-", &u) == 1 && u == idx) {
            found = std::string(dir_path) + "/" + ent->d_name;
            break;
        }
    }
    closedir(dir);
    return found;
}

// delete a test's temporary log directory and the files in it
static void remove_dir(const char *dir_path)
{
    DIR *dir = opendir(dir_path);
    if (dir == nullptr) {
        return;
    }
    struct dirent *ent;
    while ((ent = readdir(dir)) != nullptr) {
        if (strcmp(ent->d_name, ".") != 0 && strcmp(ent->d_name, "..") != 0) {
            unlinkat(dirfd(dir), ent->d_name, 0);
        }
    }
    closedir(dir);
    rmdir(dir_path);
}

static std::vector<uint8_t> read_file(const char *path)
{
    std::vector<uint8_t> content;
    int file = open(path, O_RDONLY);
    if (file < 0) {
        return content;
    }
    uint8_t chunk[4096];
    ssize_t n;
    while ((n = read(file, chunk, sizeof(chunk))) > 0) {
        content.insert(content.end(), chunk, chunk + n);
    }
    close(file);
    return content;
}

/*
 * A pipe nobody reads stands in for stalled storage: the worker blocks writing it and the
 * ring fills up behind it. The pipe is shrunk to one page so that it absorbs the same
 * couple of records on every kernel page size.
 */
static int make_stall_pipe(int pfd[2])
{
    if (pipe(pfd) != 0) {
        return -1;
    }
    fcntl(pfd[1], F_SETPIPE_SZ, 4096);
    return 0;
}

// queue full-size records to the pipe until the writer refuses one (or max are queued),
// then give the worker a moment to block on the full pipe and top the ring up again, so
// that the data share of the ring is full and stays full; returns how many were accepted
static uint32_t stall_writer(LogWriter &writer, int pipe_wfd, uint32_t max = UINT32_MAX)
{
    uint8_t record[LogWriter::DATA_MAX];
    uint32_t accepted = 0;
    for (int round = 0; round < 3 && accepted < max; round++) {
        while (accepted < max) {
            memcpy(record, &accepted, sizeof(accepted)); // per-record sequence number
            if (!writer.write(pipe_wfd, record, sizeof(record))) {
                break;
            }
            accepted++;
        }
        usleep(2000);
    }
    return accepted;
}

// consume everything the stalled worker was asked to write, then wait for the ring to empty
static std::vector<uint8_t> release_writer(LogWriter &writer, int pipe_rfd, size_t bytes)
{
    std::vector<uint8_t> got;
    uint8_t chunk[4096];
    while (got.size() < bytes) {
        ssize_t n = read(pipe_rfd, chunk, sizeof(chunk));
        if (n <= 0) {
            break;
        }
        got.insert(got.end(), chunk, chunk + n);
    }
    writer.drain();
    return got;
}

// start()/stop() talk to the Mainloop timeout machinery
class LogStartStopTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        Mainloop &mainloop = Mainloop::init();
        // returns -EBUSY when a previous test left the epoll fd open -- that instance works fine
        mainloop.open();
    }
    void TearDown() override { Mainloop::teardown(); }
};

TEST_F(LogStartStopTest, RetentionRunsAfterStopNotAtStart)
{
    char dir_tmpl[] = "/tmp/logretention_XXXXXX";
    char *dir = mkdtemp(dir_tmpl);
    ASSERT_NE(dir, nullptr);

    // two finished (read-only) logs from previous flights
    for (const char *name : {"00001-2026-01-01_00-00-00.tlog", "00002-2026-01-02_00-00-00.tlog"}) {
        char path[PATH_MAX];
        ASSERT_LT(snprintf(path, sizeof(path), "%s/%s", dir, name), (int)sizeof(path));
        int file = open(path, O_WRONLY | O_CREAT, 0644);
        ASSERT_GE(file, 0);
        ASSERT_EQ(write(file, "x", 1), 1);
        close(file);
        chmod(path, S_IRUSR | S_IRGRP | S_IROTH);
    }

    LogOptions conf;
    conf.logs_dir = dir;
    conf.log_mode = LogMode::always;
    conf.min_free_space = 0;
    conf.max_log_files = 1;

    TLog tlog{conf};
    ASSERT_TRUE(tlog.start());
    // start() must not have scanned or deleted anything: both old logs plus the new one exist
    EXPECT_EQ(count_logs(dir), 3);

    tlog.stop();
    // stop() reclaims: only the newest (the just-finished log) survives MaxLogFiles=1
    EXPECT_EQ(count_logs(dir), 1);

    remove_dir(dir);
}

class TestTLog : public TLog {
public:
    TestTLog(const LogOptions &conf)
        : TLog{conf}
    {
    }
    int get_file() const { return _file; }
    void set_file(int file) { _file = file; }
    void use_writer(std::shared_ptr<LogWriter> writer)
    {
        _writer = writer;
        _writer_errors_seen = writer->last_error().count; // as start() does
    }
    bool tick_fsync() { return _fsync(); }

    int error_hook_calls = 0;
    int error_hook_err = 0;

protected:
    void _handle_write_error(int err) override
    {
        error_hook_calls++;
        error_hook_err = err;
    }
};

static LogOptions log_test_options()
{
    LogOptions conf;
    conf.logs_dir = "./";
    conf.log_mode = LogMode::disabled; // keep write_msg from trying to start a real log
    conf.min_free_space = 0;
    conf.max_log_files = 0;
    return conf;
}

TEST(LogEndpointTest, TlogWritesTimestampAndPayload)
{
    char path[] = "/tmp/tlog_test_XXXXXX";
    int file = mkstemp(path);
    ASSERT_GE(file, 0);

    auto writer = LogWriter::instance();
    ASSERT_NE(writer, nullptr);
    TestTLog tlog{log_test_options()};
    tlog.set_file(file);
    tlog.use_writer(writer);

    uint8_t payload[64];
    memset(payload, 0x6C, sizeof(payload));
    struct buffer msg = {};
    msg.data = payload;
    msg.len = sizeof(payload);
    msg.curr.msg_id = 1; // not a heartbeat

    EXPECT_EQ(tlog.write_msg(&msg), (int)msg.len);
    writer->drain(); // the record is written by the background writer

    // one record: 8-byte big-endian microsecond timestamp, then the payload verbatim
    uint8_t content[128];
    ssize_t n = pread(file, content, sizeof(content), 0);
    ASSERT_EQ(n, (ssize_t)(sizeof(uint64_t) + sizeof(payload)));
    EXPECT_TRUE(std::all_of(content + 8, content + n, [](uint8_t b) { return b == 0x6C; }));

    uint64_t stamp_be;
    memcpy(&stamp_be, content, sizeof(stamp_be));
    // sanity: decodes to a time after 2020-01-01 (in microseconds)
    EXPECT_GT(be64toh(stamp_be), 1577836800ULL * 1000000ULL);

    close(file);
    unlink(path);
}

TEST(LogEndpointTest, TlogIgnoresMessagesWhileNotLogging)
{
    // The endpoint accepts every routed message, but must queue nothing while no log is
    // open: the worker could only fail such records, and they would take ring slots from
    // a log that is open.
    uint8_t payload[64] = {};
    struct buffer msg = {};
    msg.data = payload;
    msg.len = sizeof(payload);
    msg.curr.msg_id = 1;

    // before the first start() there is not even a writer
    TestTLog tlog{log_test_options()};
    EXPECT_EQ(tlog.write_msg(&msg), (int)msg.len);

    // with a writer but no file: fill the data slots first, so that any record the
    // endpoint tried to queue would show up as a refused one
    int pfd[2];
    ASSERT_EQ(make_stall_pipe(pfd), 0);
    auto writer = LogWriter::instance();
    ASSERT_NE(writer, nullptr);
    tlog.use_writer(writer);
    const uint32_t stalled = stall_writer(*writer, pfd[1]);
    const uint32_t dropped_before = writer->dropped();

    for (int i = 0; i < 10; i++) {
        EXPECT_EQ(tlog.write_msg(&msg), (int)msg.len);
    }
    EXPECT_EQ(writer->dropped(), dropped_before);

    release_writer(*writer, pfd[0], stalled * LogWriter::DATA_MAX);
    close(pfd[0]);
    close(pfd[1]);
}

TEST_F(LogStartStopTest, TlogStopStartHandsOffFd)
{
    // stop() hands the file to the writer, and the writer may still hold writes for it
    // when start() opens the next log. The old fd must stay open until those are done --
    // if it were closed early, its number could be reused for the new log and stale
    // records would land in the wrong file.
    char dir_tmpl[] = "/tmp/loghandoff_XXXXXX";
    char *dir = mkdtemp(dir_tmpl);
    ASSERT_NE(dir, nullptr);

    LogOptions conf;
    conf.logs_dir = dir;
    conf.log_mode = LogMode::always;
    conf.min_free_space = 0;
    conf.max_log_files = 0;

    int pfd[2];
    ASSERT_EQ(make_stall_pipe(pfd), 0);

    uint8_t payload[16];
    struct buffer msg = {};
    msg.data = payload;
    msg.len = sizeof(payload);
    msg.curr.msg_id = 1;

    {
        TestTLog tlog{conf};
        ASSERT_TRUE(tlog.start());
        auto writer = LogWriter::instance(); // the endpoint's own instance
        ASSERT_NE(writer, nullptr);
        const int old_fd = tlog.get_file();

        memset(payload, 0xAA, sizeof(payload));
        EXPECT_EQ(tlog.write_msg(&msg), (int)msg.len);
        writer->drain();

        // stall the worker so the close queued by stop() is still pending across start()
        const uint32_t stalled = stall_writer(*writer, pfd[1], 4);
        tlog.stop();
        ASSERT_TRUE(tlog.start());
        const int new_fd = tlog.get_file();
        EXPECT_NE(new_fd, old_fd); // the old fd is still open, so its number is not reused
        memset(payload, 0xBB, sizeof(payload));
        EXPECT_EQ(tlog.write_msg(&msg), (int)msg.len);

        release_writer(*writer, pfd[0], stalled * LogWriter::DATA_MAX);

        // only now is the old fd closed; each file holds exactly its own record
        EXPECT_EQ(fcntl(old_fd, F_GETFD), -1);
        EXPECT_NE(fcntl(new_fd, F_GETFD), -1);
        std::vector<uint8_t> old_log = read_file(find_log(dir, 0).c_str());
        std::vector<uint8_t> new_log = read_file(find_log(dir, 1).c_str());
        ASSERT_EQ(old_log.size(), sizeof(uint64_t) + sizeof(payload));
        ASSERT_EQ(new_log.size(), sizeof(uint64_t) + sizeof(payload));
        EXPECT_TRUE(
            std::all_of(old_log.begin() + 8, old_log.end(), [](uint8_t b) { return b == 0xAA; }));
        EXPECT_TRUE(
            std::all_of(new_log.begin() + 8, new_log.end(), [](uint8_t b) { return b == 0xBB; }));

        tlog.stop();
        writer->drain();
    }

    close(pfd[0]);
    close(pfd[1]);
    remove_dir(dir);
}

TEST(LogEndpointTest, WriteErrorReachesEndpointOnce)
{
    // a read-only fd: every write fails on the worker with EBADF, while fsync still works
    char path[] = "/tmp/tlog_err_XXXXXX";
    int tmp = mkstemp(path);
    ASSERT_GE(tmp, 0);
    close(tmp);
    int file = open(path, O_RDONLY);
    ASSERT_GE(file, 0);

    auto writer = LogWriter::instance();
    ASSERT_NE(writer, nullptr);
    TestTLog tlog{log_test_options()};
    tlog.set_file(file);
    tlog.use_writer(writer);

    uint8_t payload[16] = {};
    struct buffer msg = {};
    msg.data = payload;
    msg.len = sizeof(payload);
    msg.curr.msg_id = 1;

    // the record is accepted; the failure happens later, on the worker
    EXPECT_EQ(tlog.write_msg(&msg), (int)msg.len);
    writer->drain();
    EXPECT_EQ(tlog.error_hook_calls, 0);

    // the endpoint learns about it on its next 1 Hz tick, on the routing thread
    EXPECT_TRUE(tlog.tick_fsync());
    EXPECT_EQ(tlog.error_hook_calls, 1);
    EXPECT_EQ(tlog.error_hook_err, EBADF);

    // the same failure is not reported twice (the tick's own fsync succeeded)
    writer->drain();
    EXPECT_TRUE(tlog.tick_fsync());
    EXPECT_EQ(tlog.error_hook_calls, 1);

    // a failure on some other fd is not this endpoint's...
    int other = open(path, O_RDONLY);
    ASSERT_GE(other, 0);
    EXPECT_TRUE(writer->write(other, payload, sizeof(payload)));
    writer->drain();
    EXPECT_EQ(writer->last_error().fd, other);
    EXPECT_TRUE(tlog.tick_fsync());
    EXPECT_EQ(tlog.error_hook_calls, 1);

    // ...and once the writer has closed the failing fd, its number may be reused by an
    // unrelated file: the error must no longer name it
    writer->sync_close(other);
    writer->drain();
    EXPECT_EQ(writer->last_error().fd, -1);

    close(file);
    unlink(path);
}

class TestBinLog : public BinLog {
public:
    TestBinLog(const LogOptions &conf)
        : BinLog{conf}
    {
    }
    int get_file() const { return _file; }
    void set_file(int file) { _file = file; }
    bool tick_fsync() { return _fsync(); }
};

TEST_F(LogStartStopTest, BinLogRestartsOnWriteError)
{
    // A failed block write used to restart the log from _logging_data_process(); with the
    // writer the failure is only known on the next tick, which must restart it just the same.
    char dir_tmpl[] = "/tmp/logbinerr_XXXXXX";
    char *dir = mkdtemp(dir_tmpl);
    ASSERT_NE(dir, nullptr);

    LogOptions conf;
    conf.logs_dir = dir;
    conf.log_mode = LogMode::always;
    conf.min_free_space = 0;
    conf.max_log_files = 0;

    {
        TestBinLog binlog{conf};
        ASSERT_TRUE(binlog.start());
        auto writer = LogWriter::instance();
        ASSERT_NE(writer, nullptr);
        const int good_fd = binlog.get_file();

        // swap the log's fd for a read-only one, so the next record fails on the worker
        int bad_fd = open(find_log(dir, 0).c_str(), O_RDONLY);
        ASSERT_GE(bad_fd, 0);
        struct stat bad_st;
        ASSERT_EQ(fstat(bad_fd, &bad_st), 0);
        binlog.set_file(bad_fd);
        uint8_t block[8] = {};
        EXPECT_TRUE(writer->write(bad_fd, block, sizeof(block)));
        writer->drain();

        EXPECT_TRUE(binlog.tick_fsync());
        // restarted: a second log exists and is written through a different, open fd
        EXPECT_EQ(count_logs(dir), 2);
        EXPECT_GE(binlog.get_file(), 0);
        EXPECT_NE(binlog.get_file(), bad_fd);
        writer->drain();
        // stop() handed the failed fd to the writer, which closed it. The number may already
        // belong to something else (start() creates timer fds, and the worker's close can win
        // that race), so check the descriptor no longer refers to the failed file.
        struct stat st;
        EXPECT_TRUE(fstat(bad_fd, &st) < 0 || st.st_ino != bad_st.st_ino);

        close(good_fd); // the descriptor swapped out above
        binlog.stop();
        writer->drain();
    }

    remove_dir(dir);
}

class TestULog : public ULog {
public:
    TestULog(const LogOptions &conf)
        : ULog{conf}
    {
    }
    void set_file(int file) { _file = file; }
    void use_writer(std::shared_ptr<LogWriter> writer) { _writer = writer; }
};

// append one complete ULog message (3-byte header, then payload_len bytes of fill)
static void ulog_append_msg(std::vector<uint8_t> &stream, uint8_t type, uint16_t payload_len,
                            uint8_t fill)
{
    stream.push_back(payload_len & 0xff);
    stream.push_back(payload_len >> 8);
    stream.push_back(type);
    stream.insert(stream.end(), payload_len, fill);
}

// route one LOGGING_DATA message carrying stream[from, from + len) into the endpoint
static void ulog_send(TestULog &ulog, uint16_t seq, const std::vector<uint8_t> &stream, size_t from,
                      size_t len)
{
    // the endpoint reads a little past the payload while it strips the file header, as it
    // may from its receive buffer, so give the message some room after it
    uint8_t frame[sizeof(mavlink_logging_data_t) + 64] = {};
    auto *data = (mavlink_logging_data_t *)frame;
    data->sequence = seq;
    data->length = (uint8_t)len;
    data->first_message_offset = 0;
    memcpy(data->data, stream.data() + from, len);

    uint8_t stx = MAVLINK_STX_MAVLINK1; // a v1 frame: no trailing zeros to restore
    struct buffer msg = {};
    msg.data = &stx;
    msg.len = 1;
    msg.curr.msg_id = MAVLINK_MSG_ID_LOGGING_DATA;
    msg.curr.payload = frame;
    msg.curr.payload_len = MAVLINK_MSG_ID_LOGGING_DATA_LEN;
    EXPECT_EQ(ulog.write_msg(&msg), (int)msg.len);
}

TEST(LogEndpointTest, UlogCoalescesCompleteMessages)
{
    // A seqpacket socket keeps the writer's records apart: every write() the worker issues
    // arrives as exactly one packet, so the records are observable as well as their bytes.
    int sv[2];
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_SEQPACKET, 0, sv), 0);

    auto writer = LogWriter::instance();
    ASSERT_NE(writer, nullptr);
    TestULog ulog{log_test_options()};
    ulog.set_file(sv[1]);
    ulog.use_writer(writer);

    // the stream: the 16-byte file header, five complete messages, one message split
    // across two LOGGING_DATA messages, and a complete one behind it
    std::vector<uint8_t> stream
        = {0x55, 0x4C, 0x6F, 0x67, 0x01, 0x12, 0x35, 0x01, 0, 0, 0, 0, 0, 0, 0, 0};
    ulog_append_msg(stream, 'A', 10, 0xA0);
    ulog_append_msg(stream, 'B', 30, 0xB0);
    ulog_append_msg(stream, 'C', 50, 0xC0);
    ulog_append_msg(stream, 'D', 20, 0xD0);
    ulog_append_msg(stream, 'E', 40, 0xE0);
    const size_t complete = stream.size();
    ulog_append_msg(stream, 'F', 57, 0xF0);
    const size_t split = complete + 25; // cut inside message F
    ulog_append_msg(stream, 'G', 10, 0x60);
    ASSERT_LE(split, 249u);
    ASSERT_LE(stream.size() - split, 249u);

    ulog_send(ulog, 0, stream, 0, split);
    writer->drain();

    // the file header goes first, then the five complete messages in ONE record; the
    // incomplete message stays in the endpoint's buffer
    uint8_t packet[LogWriter::DATA_MAX];
    ASSERT_EQ(read(sv[0], packet, sizeof(packet)), 16);
    EXPECT_EQ(memcmp(packet, stream.data(), 16), 0);
    ASSERT_EQ(read(sv[0], packet, sizeof(packet)), (ssize_t)(complete - 16));
    EXPECT_EQ(memcmp(packet, stream.data() + 16, complete - 16), 0);
    ASSERT_EQ(fcntl(sv[0], F_SETFL, O_NONBLOCK), 0);
    EXPECT_EQ(read(sv[0], packet, sizeof(packet)), -1);
    EXPECT_EQ(errno, EAGAIN);

    ulog_send(ulog, 1, stream, split, stream.size() - split);
    writer->drain();

    // once completed, the split message and the one behind it again form one record, and
    // the records concatenate to exactly the stream
    ASSERT_EQ(read(sv[0], packet, sizeof(packet)), (ssize_t)(stream.size() - complete));
    EXPECT_EQ(memcmp(packet, stream.data() + complete, stream.size() - complete), 0);
    EXPECT_EQ(read(sv[0], packet, sizeof(packet)), -1);
    EXPECT_EQ(errno, EAGAIN);

    close(sv[0]);
    close(sv[1]);
}

/**
 * LogWriter (background log IO)
 */

TEST(LogWriterTest, FifoByteExactAcrossFiles)
{
    char path_a[] = "/tmp/logwriter_a_XXXXXX";
    char path_b[] = "/tmp/logwriter_b_XXXXXX";
    int fd_a = mkstemp(path_a);
    int fd_b = mkstemp(path_b);
    ASSERT_GE(fd_a, 0);
    ASSERT_GE(fd_b, 0);

    auto writer = LogWriter::instance();
    ASSERT_NE(writer, nullptr);
    uint8_t block[100];

    memset(block, 0xA1, sizeof(block));
    EXPECT_TRUE(writer->write(fd_a, block, sizeof(block)));
    memset(block, 0xB1, sizeof(block));
    EXPECT_TRUE(writer->write(fd_b, block, sizeof(block)));
    memset(block, 0xA2, sizeof(block));
    EXPECT_TRUE(writer->write(fd_a, block, sizeof(block)));
    memset(block, 0xB2, sizeof(block));
    EXPECT_TRUE(writer->pwrite(fd_b, block, sizeof(block), 100));
    writer->drain();

    uint8_t content[256];
    ASSERT_EQ(pread(fd_a, content, sizeof(content), 0), 200);
    EXPECT_TRUE(std::all_of(content, content + 100, [](uint8_t b) { return b == 0xA1; }));
    EXPECT_TRUE(std::all_of(content + 100, content + 200, [](uint8_t b) { return b == 0xA2; }));
    ASSERT_EQ(pread(fd_b, content, sizeof(content), 0), 200);
    EXPECT_TRUE(std::all_of(content, content + 100, [](uint8_t b) { return b == 0xB1; }));
    EXPECT_TRUE(std::all_of(content + 100, content + 200, [](uint8_t b) { return b == 0xB2; }));

    close(fd_a);
    close(fd_b);
    unlink(path_a);
    unlink(path_b);
}

TEST(LogWriterTest, SyncCloseClosesFdAfterQueuedWrites)
{
    char path[] = "/tmp/logwriter_c_XXXXXX";
    int file = mkstemp(path);
    ASSERT_GE(file, 0);

    auto writer = LogWriter::instance();
    ASSERT_NE(writer, nullptr);
    uint8_t block[64];
    memset(block, 0xC3, sizeof(block));
    EXPECT_TRUE(writer->write(file, block, sizeof(block)));
    writer->sync_close(file); // worker owns the fd from here
    writer->drain();

    struct stat st;
    ASSERT_EQ(stat(path, &st), 0);
    EXPECT_EQ(st.st_size, 64); // the write queued before the close made it
    EXPECT_EQ(fcntl(file, F_GETFD), -1);
    EXPECT_EQ(errno, EBADF);
    unlink(path);
}

TEST(LogWriterTest, FullQueueDropsButNeverBlocks)
{
    // With the worker stalled on the pipe, further enqueues must return false immediately
    // instead of blocking, and every accepted record must still arrive whole and in order.
    int pfd[2];
    ASSERT_EQ(make_stall_pipe(pfd), 0);

    auto writer = LogWriter::instance();
    ASSERT_NE(writer, nullptr);
    constexpr size_t RECLEN = LogWriter::DATA_MAX;
    constexpr uint32_t TOTAL = 200; // > ring capacity + what the pipe buffer absorbs
    uint8_t record[RECLEN];

    const uint32_t dropped_before = writer->dropped(); // shared writer: not necessarily 0
    uint32_t accepted = 0;
    for (uint32_t i = 0; i < TOTAL; i++) {
        memcpy(record, &i, sizeof(i)); // per-record sequence number for the FIFO check
        if (writer->write(pfd[1], record, RECLEN)) {
            accepted++;
        }
    }
    const uint32_t dropped = writer->dropped() - dropped_before;
    EXPECT_GT(dropped, 0u);
    EXPECT_EQ(accepted + dropped, TOTAL);

    std::vector<uint8_t> got = release_writer(*writer, pfd[0], accepted * RECLEN);
    ASSERT_EQ(got.size(), accepted * RECLEN);

    // accepted records arrive whole and in order (sequence numbers strictly increasing)
    uint32_t prev = 0;
    bool first = true;
    for (size_t off = 0; off < got.size(); off += RECLEN) {
        uint32_t seq;
        memcpy(&seq, got.data() + off, sizeof(seq));
        if (!first) {
            ASSERT_GT(seq, prev);
        }
        prev = seq;
        first = false;
    }

    close(pfd[0]);
    close(pfd[1]);
}

TEST(LogWriterTest, SyncCloseSurvivesFullRing)
{
    int pfd[2];
    ASSERT_EQ(make_stall_pipe(pfd), 0);
    char path[] = "/tmp/logwriter_d_XXXXXX";
    int file = mkstemp(path);
    ASSERT_GE(file, 0);

    auto writer = LogWriter::instance();
    ASSERT_NE(writer, nullptr);
    uint8_t block[LogWriter::DATA_MAX];
    memset(block, 0xD4, sizeof(block));
    EXPECT_TRUE(writer->write(file, block, 64));
    writer->drain();

    // stall the worker and fill the data share of the ring: data is refused from now on...
    const uint32_t dropped_before = writer->dropped();
    const uint32_t stalled = stall_writer(*writer, pfd[1]);
    EXPECT_GT(writer->dropped(), dropped_before);
    // ...but control records still find room in the reserve, until even that is taken and
    // a periodic fsync is skipped...
    uint32_t fsyncs = 0;
    while (writer->fsync(file)) {
        fsyncs++;
    }
    EXPECT_GE(fsyncs, (uint32_t)LogWriter::CONTROL_RESERVE);
    // ...while a close is not: it waits until the worker's current write returns and
    // frees a slot
    std::thread reader([&] { release_writer(*writer, pfd[0], stalled * LogWriter::DATA_MAX); });
    writer->sync_close(file);
    reader.join();

    // exactly the accepted bytes reached the file, which the worker then closed
    struct stat st;
    ASSERT_EQ(stat(path, &st), 0);
    EXPECT_EQ(st.st_size, 64);
    EXPECT_EQ(fcntl(file, F_GETFD), -1);
    EXPECT_EQ(errno, EBADF);

    close(pfd[0]);
    close(pfd[1]);
    unlink(path);
}

TEST(LogWriterTest, WorkerBlocksAllSignals)
{
    // mavlink-routerd's SIGTERM/SIGINT handlers only set a flag that the routing thread
    // notices when its epoll_wait returns: a signal delivered to the worker instead would
    // not interrupt that wait, so the worker must not be a candidate for delivery.
    auto writer = LogWriter::instance();
    ASSERT_NE(writer, nullptr);

    DIR *tasks = opendir("/proc/self/task");
    ASSERT_NE(tasks, nullptr);
    bool found = false;
    unsigned long long blocked = 0;
    struct dirent *ent;
    while ((ent = readdir(tasks)) != nullptr) {
        char path[PATH_MAX];
        char line[256] = {};
        snprintf(path, sizeof(path), "/proc/self/task/%s/comm", ent->d_name);
        FILE *comm = fopen(path, "r");
        if (comm == nullptr) {
            continue;
        }
        const bool is_worker
            = fgets(line, sizeof(line), comm) != nullptr && strncmp(line, "log-writer", 10) == 0;
        fclose(comm);
        if (!is_worker) {
            continue;
        }
        snprintf(path, sizeof(path), "/proc/self/task/%s/status", ent->d_name);
        FILE *status = fopen(path, "r");
        ASSERT_NE(status, nullptr);
        while (fgets(line, sizeof(line), status) != nullptr) {
            if (sscanf(line, "SigBlk: %llx", &blocked) == 1) {
                found = true;
                break;
            }
        }
        fclose(status);
        break;
    }
    closedir(tasks);

    ASSERT_TRUE(found) << "no thread named log-writer";
    EXPECT_NE(blocked & (1ULL << (SIGTERM - 1)), 0u);
    EXPECT_NE(blocked & (1ULL << (SIGINT - 1)), 0u);

    // the creating thread's own mask is restored: signals still reach the daemon's handlers
    sigset_t mine;
    ASSERT_EQ(pthread_sigmask(SIG_BLOCK, nullptr, &mine), 0);
    EXPECT_FALSE(sigismember(&mine, SIGTERM));
    EXPECT_FALSE(sigismember(&mine, SIGINT));
}

TEST(LogEndpointTest, Init)
{
    LogOptions conf;
    conf.logs_dir = "./";

    BinLog binlog{conf}; // assertions should not fail
    EXPECT_EQ(binlog.get_type(), ENDPOINT_TYPE_LOG);

    ULog ulog{conf};
    EXPECT_EQ(ulog.get_type(), ENDPOINT_TYPE_LOG);

    AutoLog autolog{conf};
    EXPECT_EQ(autolog.get_type(), ENDPOINT_TYPE_LOG);

    TLog tlog{conf};
    EXPECT_EQ(tlog.get_type(), ENDPOINT_TYPE_LOG);
}
