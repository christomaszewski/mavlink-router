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

#include <limits.h>
#include <netinet/in.h>
#include <poll.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>

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
    int flush_pending_msgs() override { return -ENOSYS; }
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
    uint32_t incomplete_msgs() const { return _incomplete_msgs; }
};

TEST(UdpEndpointTest, TruncatedDatagramIsDropped)
{
    TestUdpEndpoint udp{};

    // receiver socket, owned (and closed) by the endpoint, on an ephemeral loopback port
    udp.fd = socket(AF_INET, SOCK_DGRAM, 0);
    ASSERT_GE(udp.fd, 0);
    struct sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    ASSERT_EQ(bind(udp.fd, (struct sockaddr *)&addr, sizeof(addr)), 0);
    socklen_t addrlen = sizeof(addr);
    ASSERT_EQ(getsockname(udp.fd, (struct sockaddr *)&addr, &addrlen), 0);

    int sender = socket(AF_INET, SOCK_DGRAM, 0);
    ASSERT_GE(sender, 0);

    uint8_t big[128];
    memset(big, 0xAB, sizeof(big));
    ASSERT_EQ(sendto(sender, big, sizeof(big), 0, (struct sockaddr *)&addr, sizeof(addr)),
              (ssize_t)sizeof(big));

    // a datagram larger than the buffer must be dropped entirely, not truncated into the parser
    uint8_t small[64];
    EXPECT_EQ(udp._read_msg(small, sizeof(small)), 0);
    EXPECT_EQ(udp.incomplete_msgs(), 1U);

    // a datagram that fits is still delivered
    ASSERT_EQ(sendto(sender, big, 32, 0, (struct sockaddr *)&addr, sizeof(addr)), 32);
    EXPECT_EQ(udp._read_msg(small, sizeof(small)), 32);
    EXPECT_EQ(udp.incomplete_msgs(), 1U);

    close(sender);
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

INSTANTIATE_TEST_CASE_P(UdpEndpointTest, UdpEndpointConfigPortTestFixture,
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

    // listening socket on an ephemeral loopback port; port returned by reference
    static int open_listener(unsigned long &port)
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
            || getsockname(fd, (struct sockaddr *)&addr, &addrlen) < 0 || listen(fd, 1) < 0) {
            close(fd);
            return -1;
        }
        port = ntohs(addr.sin_port);
        return fd;
    }

    static bool wait_writable(int fd, int timeout_ms)
    {
        struct pollfd pfd = {};
        pfd.fd = fd;
        pfd.events = POLLOUT;
        return poll(&pfd, 1, timeout_ms) == 1;
    }
};

TEST_F(TcpConnectTest, AsyncConnectSuccess)
{
    unsigned long port;
    int listener = open_listener(port);
    ASSERT_GE(listener, 0);

    TestTcpEndpoint tcp{};
    ASSERT_TRUE(tcp.open("127.0.0.1", port));
    if (tcp.connecting()) { // loopback may also connect immediately
        ASSERT_TRUE(wait_writable(tcp.fd, 2000));
        EXPECT_FALSE(tcp.handle_canwrite()); // false: mainloop switches EPOLLOUT -> EPOLLIN
    }
    EXPECT_FALSE(tcp.connecting());
    EXPECT_GE(tcp.fd, 0);
    EXPECT_TRUE(tcp.is_valid());

    close(listener);
}

TEST_F(TcpConnectTest, AsyncConnectRefused)
{
    // bind an ephemeral port, then close it again: connecting to it gets a fast RST
    unsigned long port;
    int listener = open_listener(port);
    ASSERT_GE(listener, 0);
    close(listener);

    TestTcpEndpoint tcp{};
    if (!tcp.open("127.0.0.1", port)) {
        return; // synchronous failure is acceptable, nothing more to check
    }
    if (tcp.connecting()) {
        ASSERT_TRUE(wait_writable(tcp.fd, 2000));
        EXPECT_TRUE(tcp.handle_canwrite()); // true: fd already closed, no mod_fd wanted
    }
    EXPECT_EQ(tcp.fd, -1);
    EXPECT_FALSE(tcp.connecting());
    EXPECT_TRUE(tcp.is_valid()); // a refused connect must not get the endpoint garbage-collected
}

TEST_F(TcpConnectTest, OpenDoesNotBlock)
{
    // 192.0.2.1 (TEST-NET-1) is guaranteed unroutable: environments split between instant
    // ENETUNREACH and EINPROGRESS-then-hang, but a non-blocking connect returns immediately
    // in both cases -- so assert ONLY the elapsed time
    TestTcpEndpoint tcp{};
    auto start = std::chrono::steady_clock::now();
    tcp.open("192.0.2.1", 5760);
    auto elapsed = std::chrono::steady_clock::now() - start;
    EXPECT_LT(elapsed, std::chrono::seconds(1));
}

TEST_F(TcpConnectTest, WriteSkippedWhileConnecting)
{
    TestTcpEndpoint tcp{};
    if (!tcp.open("192.0.2.1", 5760) || !tcp.connecting()) {
        GTEST_SKIP() << "environment rejects TEST-NET-1 synchronously; no connecting state";
    }

    // guards return before any buffer field is inspected
    buffer test_msg{};
    EXPECT_EQ(tcp.write_msg(&test_msg), 0);
    EXPECT_EQ(tcp.accept_msg(&test_msg), Endpoint::AcceptState::Rejected);
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
