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

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <signal.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <deque>

#include <gtest/gtest.h>

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
