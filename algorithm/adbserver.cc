#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <stdlib.h>

#include "adbserver.h"
#include "../comm/minizip/zip.h"
#include "../comm/minizip/unzip.h"
#include "../comm/md5/md5.h"
#include "mavlink_msg_dance_info.h"

AdbServer::AdbServer()
{
    // server_msg_codec_ptr_=make_shared<MessageCodec>();
    mDanceFileFlag = 0;
    status_ = false;
    printf("AdbServer\n");
}

AdbServer::~AdbServer()
{
    if (loop_)
    {
        delete loop_;
    }
    printf("%s\n", "AdbServer::~AdbServer");
}

void AdbServer::send(const char *message, int len)
{
    if (client_ptr_.lock())
    {
        client_ptr_.lock()->send(message, len);
    }
}

void AdbServer::start()
{
    loop_thread_ = std::thread(std::bind(&AdbServer::runInThread, this));
}

void AdbServer::stop()
{
    server_ptr_.reset();
    sleep(2);
    loop_->quit();
    loop_thread_.join();
}

void AdbServer::runInThread()
{
    loop_ = new Network::EventLoop();
    Network::InetAddress listenAddr("0.0.0.0", 11120);
    server_ptr_ = make_shared<Network::TcpServer>(loop_, listenAddr, "AdbServer");
    server_ptr_->setConnectionCallback(std::bind(&AdbServer::onServerConnection, this, _1));
    server_ptr_->setMessageCallback(std::bind(&AdbServer::onMessage, this, _1, _2, _3));
    server_ptr_->start();
    loop_->loop();
}

unsigned char msg[2048];
void AdbServer::onMessage(const Network::TcpConnectionPtr &conn, Network::DataBuffer *buf, Network::Timestamp receiveTime)
{
    mavlink_status_t status;
    mavlink_message_t mav_msg;
    // string msgBuffer(buf->retrieveAllAsString());
    // printf("msgBuffer size %d\n",msgBuffer.size());

    int msg_size = buf->readableBytes();
    memcpy(msg, buf->peek(), msg_size);

    for (int i = 0; i < msg_size; i++)
    {
        if (mavlink_parse_char(MAVLINK_COMM_1, msg[i], &mav_msg, &status))
        {
            message_mavlink_callback_(&mav_msg);
        }
    }

    buf->retrieve(msg_size);
}

void AdbServer::onServerConnection(const Network::TcpConnectionPtr &conn)
{
    if (conn->connected())
    {
        client_ptr_ = conn;
        status_ = true;
        printf("connected to adbserver\n");
    }
    else if (conn->disconnected())
    {
        status_ = false;
        printf("connect is close\n");
    }
}

void AdbServer::setMavlinkMessageCallback(const MavlinkMessageCallback &cb)
{
    message_mavlink_callback_ = cb;
}

bool AdbServer::getAppConnectionStatus()
{
    return status_;
}

// 删除目录及其内容
void AdbServer::delete_directory(const char *path)
{
    DIR *dir;
    struct dirent *entry;
    char full_path[1024];

    dir = opendir(path);
    if (dir == NULL)
    {
        perror("Error opening directory");
        return;
    }

    while ((entry = readdir(dir)) != NULL)
    {
        // 忽略当前目录和父目录
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
        {
            continue;
        }

        // 拼接完整路径
        snprintf(full_path, sizeof(full_path), "%s/%s", path, entry->d_name);

        // 判断文件类型
        if (entry->d_type == DT_DIR)
        {
            // 递归删除子目录
            delete_directory(full_path);
        }
        else
        {
            // 删除文件
            remove(full_path);
        }
    }

    closedir(dir);

    // 删除空目录
    if (rmdir(path) != 0)
    {
        perror("Error deleting directory");
    }
}
// 检查目录是否存在，不存在则创建
int AdbServer::checkAndCreateDir(const char *path)
{
    struct stat st;

    // 检查目录是否存在
    if (stat(path, &st) == -1)
    {
        // 目录不存在，尝试创建
        if (mkdir(path, 0755) == -1)
        {
            perror("mkdir");
            return -1;
        }
        return 0; // 创建成功
    }

    // 检查是否为一个目录
    if (!S_ISDIR(st.st_mode))
    {
        fprintf(stderr, "%s is not a directory.\n", path);
        return -1;
    }

    return 0; // 目录已存在
}
int AdbServer::extractZipFile(const char *zipFilePath, const char *outputDir)
{
    if (zipFilePath == NULL || outputDir == NULL)
    {
        printf("extractZipFile param err...\n");
        return -1;
    }

    printf("unzip start...\n");

    // Open the zip file
    unzFile zipFile = unzOpen(zipFilePath);
    if (!zipFile)
    {
        printf("unzOpen err...\n");
        return -1;
    }

    // Get the number of entries
    if (unzGoToFirstFile(zipFile) != UNZ_OK)
    {
        printf("unzGoToFirstFile err...\n");
        unzClose(zipFile);
        return -1;
    }

    delete_directory(outputDir);
    checkAndCreateDir(outputDir);

    do
    {
        char filename[256];
        unz_file_info fileInfo;
        if (unzGetCurrentFileInfo(zipFile, &fileInfo, filename, sizeof(filename), nullptr, 0, nullptr, 0) != UNZ_OK)
        {
            printf("getting file info err...\n");
            continue;
        }

        // Open file in the zip archive
        if (unzOpenCurrentFile(zipFile) != UNZ_OK)
        {
            printf("Error opening file:%s\n", filename);
            continue;
        }

        printf("decompression file: %s\n", filename);

        // Create the output file path
        std::string OutputDir = outputDir;
        std::string outputPath = OutputDir + "/" + filename;
        std::ofstream outFile(outputPath, std::ios::binary);
        if (!outFile)
        {
            printf("Error creating file: %s\n", outputPath.c_str());
            unzCloseCurrentFile(zipFile);
            continue;
        }

        // Read the file contents from the zip archive
        std::vector<char> buffer(fileInfo.uncompressed_size);
        int bytesRead = unzReadCurrentFile(zipFile, buffer.data(), buffer.size());
        if (bytesRead < 0)
        {
            printf("reading file from zip err...\n");
        }
        else
        {
            outFile.write(buffer.data(), bytesRead);
        }

        // Close the current file in the zip archive
        unzCloseCurrentFile(zipFile);
        outFile.close();

    } while (unzGoToNextFile(zipFile) == UNZ_OK);

    // Close the zip file
    unzClose(zipFile);

    printf("unzip end...\n");

    return 0;
}

void AdbServer::DanceFileUnzipThread()
{
    char ZipPath[100] = {0};
    char UnzipPath[100] = {0};
    int ret;

    sprintf(ZipPath, "%s/%s", DANCE_FILE_DIR, mDanceName);
    sscanf(ZipPath, "%[^.].zip", UnzipPath);

    ret = extractZipFile(ZipPath, UnzipPath);
    if (ret == 0)
    {
        remove(ZipPath);
        DanceAdbAck(DANCE_FILE_TRANS_OK, NULL, 0, NULL); // 回应adb
        mDanceFileFlag = 0;
    }
    else
    {
        DanceAdbAck(DANCE_FILE_TRANS_ERR, NULL, 0, NULL); // 回应adb
        mDanceFileFlag = 0;
    }
}

int AdbServer::DanceFileUnzipStart()
{
    std::thread(std::bind(&AdbServer::DanceFileUnzipThread, this)).detach();
}

int AdbServer::DanceAdbAck(unsigned char cmd, char *filename, unsigned int size, char *md5)
{
    mavlink_message_t mavlink_msg;
    mavlink_dance_info_t dance;
    uint16_t mavlink_size;
    uint8_t mavlink_buf[MAVLINK_MAX_PACKET_LEN];

    memset(&dance, 0, sizeof(dance));
    memset(mavlink_buf, 0, sizeof(mavlink_buf));

    dance.cmd = cmd;
    dance.dance_size = size;
    if (filename)
        strcpy(dance.dance_name, filename);
    if (md5)
        memcpy(dance.dance_md5, md5, 16);

    mavlink_msg_dance_info_encode(0, 0, &mavlink_msg, &dance);
    mavlink_size = mavlink_msg_to_send_buffer(mavlink_buf, &mavlink_msg);

    // adb send flow
    if (true == status_)
    {
        send((char *)mavlink_buf, mavlink_size);
    }
    else
    {
        printf("AdbServer send failed\n");
        return -1;
    }

    return 0;
}

int AdbServer::readFileAndCalculateMD5(char *file_path, unsigned char OutMD5[16])
{
    FILE *fp = NULL;
    int file_size;

    uint32_t readPackets;
    uint32_t readLastLen;
    uint32_t readcnt;
    uint8_t readBuffer[ONE_PACK_SIZE];
    md5Ctx_t md5Data = {0};

    if (file_path == NULL || OutMD5 == NULL)
    {
        printf("readFileAndCalculateMD5 param = NULL, err...\n");
        return -1;
    }

    // 打开文件
    fp = fopen(file_path, "r");
    if (!fp)
    {
        printf("SendUpdateDroneThread() SEND_FILE_MD5 fopen %s failed...\n", file_path);
        return -1;
    }

    // 获取文件总大小，计算包数
    fseek(fp, 0L, SEEK_END);
    file_size = ftell(fp);

    readPackets = file_size / ONE_PACK_SIZE;
    readLastLen = file_size % ONE_PACK_SIZE;
    if (readLastLen)
    {
        readPackets++;
    }

    // md5计算初始化
    fseek(fp, 0, SEEK_SET);
    MD5Init(&md5Data);
    readcnt = 0;
    memset(readBuffer, 0, sizeof(readBuffer));
    printf("wait for md5 calc[%d]--->\r\n", readPackets);

    for (int i = 0; i < readPackets; i++)
    {
        printf("md5 calc[%d]\n", i);
        // 最后1包数据结束MD5计算
        if ((readPackets - i == 1))
        {
            printf("read cal the end--->\r\n");

            if (readLastLen > 0)
            {
                readcnt += fread(readBuffer, 1, readLastLen, fp);
                MD5Update(&md5Data, readBuffer, readLastLen);
            }
            else
            {
                readcnt += fread(readBuffer, 1, ONE_PACK_SIZE, fp);
                MD5Update(&md5Data, readBuffer, ONE_PACK_SIZE);
            }

            MD5Final(&md5Data, OutMD5);
        }
        else // 非最后一包
        {
            readcnt += fread(readBuffer, 1, ONE_PACK_SIZE, fp);
            MD5Update(&md5Data, readBuffer, ONE_PACK_SIZE);
        }
    }

    fclose(fp);

    return 0;
}
