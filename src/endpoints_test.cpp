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
#include "tlog.h"
#include "ulog.h"

#include <dirent.h>
#include <endian.h>
#include <fcntl.h>
#include <limits.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
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
