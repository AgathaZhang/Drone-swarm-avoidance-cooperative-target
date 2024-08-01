#include "adbserver.h"

AdbServer::AdbServer()
{
    //server_msg_codec_ptr_=make_shared<MessageCodec>();
    status_ = false;
    printf("AdbServer");
}

AdbServer::~AdbServer()
{
    if (loop_) {
        delete loop_;
    }
    printf("%s\n","AdbServer::~AdbServer");
}

void AdbServer::send(const char *message, int len)
{
    if (client_ptr_.lock()) {
        client_ptr_.lock()->send(message,len);
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
    loop_=new Network::EventLoop();
    Network::InetAddress listenAddr("0.0.0.0",11120);
    server_ptr_=make_shared<Network::TcpServer>(loop_, listenAddr, "AdbServer");
    server_ptr_->setConnectionCallback(std::bind(&AdbServer::onServerConnection,this,_1));
    server_ptr_->setMessageCallback(std::bind(&AdbServer::onMessage,this,_1,_2,_3));
    server_ptr_->start();
    loop_->loop();
}

unsigned char msg[2048];
void AdbServer::onMessage(const Network::TcpConnectionPtr &conn,Network::DataBuffer *buf,Network::Timestamp receiveTime)
{
    mavlink_status_t status;
    mavlink_message_t mav_msg;
    //string msgBuffer(buf->retrieveAllAsString());
    //printf("msgBuffer size %d\n",msgBuffer.size());

    int msg_size = buf->readableBytes();
    memcpy(msg, buf->peek(), msg_size);

    for (int i=0; i<msg_size; i++) {
        if (mavlink_parse_char(MAVLINK_COMM_1, msg[i], &mav_msg, &status)) {
            message_mavlink_callback_(&mav_msg);
        }
    }

    buf->retrieve(msg_size);
}

void AdbServer::onServerConnection(const Network::TcpConnectionPtr &conn)
{   
    if (conn->connected()) {
        client_ptr_=conn;
        status_ = true;
        printf("connected to adbserver\n");
    } else if(conn->disconnected()) {
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

