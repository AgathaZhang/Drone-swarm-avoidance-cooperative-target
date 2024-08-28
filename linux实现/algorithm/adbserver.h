#pragma once

#include <stdio.h>
#include <thread>

#include "tcpconnection.h"
#include "tcpserver.h"
#include "tcpconnection.h"
#include "eventloop.h"

#include "mavlink.h"

#define ONE_PACK_SIZE               1024
#define DANCE_FILE_DIR              "/mnt/sdcard"

#define DANCE_FILE_REQUEST_OK       0
#define DANCE_FILE_REQUEST_ERR      1
#define DANCE_FILE_TRANS_OK         2
#define DANCE_FILE_TRANS_ERR        3


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

    void delete_directory(const char *path);
    int checkAndCreateDir(const char *path);
    int extractZipFile(const char *zipFilePath, const char *outputDir);
    void DanceFileUnzipThread();
    int DanceFileUnzipStart();
    int DanceAdbAck(unsigned char cmd, char *filename, unsigned int size, char *md5);
    int readFileAndCalculateMD5(char *filepath, unsigned char OutMD5[16]);
private:
    void onServerConnection(const Network::TcpConnectionPtr &conn);
    void onMessage(const Network::TcpConnectionPtr &conn,Network::DataBuffer *buf,Network::Timestamp receiveTime);
    void runInThread();

public:
    int mDanceFileFlag;  //0-空闲  1-正在传文件 2-正在解压文件
    char mDanceName[64];
    int mDanceSize;

private:
    std::thread loop_thread_;
    Network::EventLoop *loop_;
    shared_ptr<Network::TcpServer> server_ptr_;
    std::weak_ptr<Network::TcpConnection> client_ptr_;
    MavlinkMessageCallback  message_mavlink_callback_;
    bool status_;
};