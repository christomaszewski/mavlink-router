#include "mainloop.h"

#include <sys/epoll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <gtest/gtest.h>

// closes its fd as soon as it becomes readable, like a TCP endpoint whose peer hung up
class ClosingPollable : public Pollable {
public:
    int reads = 0;
    int canwrites = 0;

    int handle_read() override
    {
        reads++;
        close(fd);
        fd = -1;
        Mainloop::get_instance().request_exit(0);
        return 0;
    }

    bool handle_canwrite() override
    {
        canwrites++;
        return true;
    }
};

TEST(MainLoopTest, closed_fd_skips_remaining_events)
{
    Mainloop &mainloop = Mainloop::init();
    mainloop.open();

    // a pending byte makes sv[0] readable while it is also writable, so epoll reports EPOLLIN
    // and EPOLLOUT in a single event
    int sv[2];
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_STREAM, 0, sv), 0);
    ASSERT_EQ(write(sv[1], "x", 1), 1);

    ClosingPollable p{};
    p.fd = sv[0];
    ASSERT_EQ(mainloop.add_fd(p.fd, &p, EPOLLIN | EPOLLOUT), 0);

    EXPECT_EQ(mainloop.loop(), 0);
    EXPECT_EQ(p.reads, 1);
    EXPECT_EQ(p.canwrites, 0); // the fd is gone: no write attempt and no mod_fd() on -1

    close(sv[1]);
    mainloop.teardown();
}

TEST(MainLoopTest, termination)
{
    Mainloop &mainloop = Mainloop::init();
    mainloop.open();
    mainloop.request_exit(0);
    int ret = mainloop.loop();
    EXPECT_EQ(0, ret);
    mainloop.teardown();
}

TEST(MainLoopTest, wrong_termination)
{
    Mainloop &mainloop = Mainloop::init();
    mainloop.open();
    mainloop.request_exit(-1);
    int ret = mainloop.loop();
    EXPECT_EQ(-1, ret);
    mainloop.teardown();
}
