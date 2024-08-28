#ifndef HAL_UART_H__
#define HAL_UART_H__

#ifdef __cplusplus
extern "C" {
#endif

int uart_open(const char* dev, int speed);
void uart_close(int fd);
int uart_set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop);
int uart_send(int fd, const unsigned char * buf, int len);
int uart_recv(int fd, char* buf, int len, int timeout_ms);

#ifdef __cplusplus
}
#endif

#endif // HAL_UART_H__
