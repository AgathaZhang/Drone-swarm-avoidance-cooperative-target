#include "uart.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

int uart_open(const char* dev, int speed)
{
    int fd;

    if((fd = open(dev, O_RDWR | O_NOCTTY /*| O_NDELAY*/)) < 0)
    {
        printf("open %s is failed\n", dev);
    } else {
        printf("open %s is success\n", dev);
        uart_set_opt(fd, speed, 8, 'N', 1);
    }

    return fd;
}

void uart_close(int fd)
{
    close(fd);
}

int uart_set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtio,oldtio;

    if (tcgetattr(fd, &oldtio) !=  0) {
        printf("tcgetattr error\n");
        return -1;
    }
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag  |=  CLOCAL | CREAD;

    newtio.c_cflag &= ~CSIZE;
    switch(nBits)
    {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    }

    switch(nEvent)
    {
    case 'O':
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP);
        break;
    case 'E':
        newtio.c_iflag |= (INPCK | ISTRIP);
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        break;
    case 'N':
        newtio.c_cflag &= ~PARENB;
        break;
    }

    switch(nSpeed)
    {   
    case 500000:
        cfsetispeed(&newtio, B500000);
        cfsetospeed(&newtio, B500000);
        break;
    case 921600:
        cfsetispeed(&newtio, B921600);
        cfsetospeed(&newtio, B921600);
        break;
    case 1500000:
        cfsetispeed(&newtio, B1500000);
        cfsetospeed(&newtio, B1500000);
        break;
    case 2400:
        cfsetispeed(&newtio, B2400);
        cfsetospeed(&newtio, B2400);
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    case 460800:
        cfsetispeed(&newtio, B460800);
        cfsetospeed(&newtio, B460800);
        break;
    default:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    }

    if (nStop == 1)
        newtio.c_cflag &=  ~CSTOPB;
    else if (nStop == 2)
        newtio.c_cflag |=  CSTOPB;

    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;

    tcflush(fd, TCIFLUSH);
    if ((tcsetattr(fd, TCSANOW, &newtio)) != 0) {
        printf("com set error\n");
        return -1;
    }

    return 0;
}

int uart_send(int fd, const unsigned char* buf, int len)
{
    int ret = 0;
    int count = 0;

    tcflush(fd, TCIFLUSH);

    while (len > 0) {
        ret = write(fd, (unsigned char*)buf + count, len);
        if (ret < 1)
            break;
        count += ret;
        len = len - ret;
    }

    return count;
}

int uart_recv(int fd, char* buf, int len, int timeout_ms)
{
    fd_set rset;
    int ret;
    struct timeval t;

    FD_ZERO(&rset);
    FD_SET(fd, &rset);
    if (timeout_ms) {
        t.tv_sec = timeout_ms / 1000;
        t.tv_usec = (timeout_ms - t.tv_sec * 1000) * 1000;
        ret = select(fd + 1, &rset, NULL, NULL, &t);
    } else {
        ret = select(fd + 1, &rset, NULL, NULL, NULL);
    }

    if (ret <= 0) {
        if (ret < 0)
            printf("Select uart device error=%d\n", errno);
        return 0;
    } else {
        ret = read(fd, (char *)buf, len);
        return ret;
    }
}
