#pragma once

#include <stdio.h>
#include <thread>

#include "tcpconnection.h"
#include "tcpserver.h"
#include "tcpconnection.h"
#include "eventloop.h"

#include "mavlink.h"



using MavlinkMessageCallback=std::function<void (const mavlink_message_t *)>;

class AdbServer
{
public:
    AdbServer();
    ~AdbServer();

    void start();
    void stop();
    void send(const char *message,int len);
    void setMavlinkMessageCallback(const MavlinkMessageCallback &cb);
    bool getAppConnectionStatus();

private:
    void onServerConnection(const Network::TcpConnectionPtr &conn);
    void onMessage(const Network::TcpConnectionPtr &conn,Network::DataBuffer *buf,Network::Timestamp receiveTime);
    void runInThread();
private:
    std::thread loop_thread_;
    Network::EventLoop *loop_;
    shared_ptr<Network::TcpServer> server_ptr_;
    std::weak_ptr<Network::TcpConnection> client_ptr_;
    MavlinkMessageCallback  message_mavlink_callback_;
    bool status_;
};