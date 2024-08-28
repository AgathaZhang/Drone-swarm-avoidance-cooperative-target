#include "icm42670.h"
#include <string.h>

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

// static int gSpidev = -1;
struct spi_ioc_transfer xfer[2];
static float accSensitivity = 0.244f; // 加速度的最小分辨率 mg/LSB
static float gyroSensitivity = 32.8f;

// uint8_t icm42670_readwritebyte(uint8_t dat)
//{
//	return spi1_readwritebyte(dat);
// }
static void icm42670_readReg(int fd, uint8_t reg, uint8_t *rddata)
{
	int ret = 0;
	unsigned int speed = 2000000;
	uint8_t bits = 8;
	unsigned int value;
	struct spi_ioc_transfer mesg[1];
	unsigned char tx_buf[4];
	unsigned char rx_buf[4];


	memset(tx_buf, 0, sizeof tx_buf);
	memset(rx_buf, 0, sizeof rx_buf);
	tx_buf[0] = (reg | 0x80);
	tx_buf[1] = 0;
	memset(mesg, 0, sizeof mesg);
	mesg[0].tx_buf = (unsigned long)tx_buf;
	mesg[0].rx_buf = (unsigned long)rx_buf;
	mesg[0].len = 2;
	mesg[0].speed_hz = 2000000;
	mesg[0].bits_per_word = 8;
	//mesg[0].cs_change = 1; // here is err, read imu err

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), mesg);
	if (ret != mesg[0].len) {
		//close(fd);
		printf("111111111111111 %d\n", ret);
		return;
	}
	printf("rx_buf[0] 0x%x,rx_buf[1] 0x%x,reg 0x%x,tx_buf[0] 0x%x\n", rx_buf[0],rx_buf[1],reg,tx_buf[0]);
	*rddata = rx_buf[1];
}

static void icm42670_writeReg(int fd, uint8_t reg, uint8_t wrdata)
{
    int status;
	uint8_t send_buffer[2];
	uint8_t recv_buffer[2];

	//assert()

    send_buffer[0] = reg;
	send_buffer[1] = wrdata;
	xfer[0].tx_buf = (unsigned long)send_buffer;
	xfer[0].len = 2;

	status = ioctl(fd, SPI_IOC_MESSAGE(1), xfer);
	if (status < 0) {
		perror("SPI_IOC_MESSGE ReadReg");
		return;
	}
	usleep(1000);
}

// 配置MREG1、2、3 参考手册13节
static void icm42670_readMregReg(int fd, uint8_t reg, uint8_t *rddata, uint8_t Mreg)
{
    int status;
    uint8_t send_buffer[2];
    uint8_t recv_buffer[2];
    uint8_t BLK_SEL_W_data =0;
    //RT_ASSERT(dev != RT_NULL);
    if (Mreg == 1) {
        BLK_SEL_W_data = 0;
    } else if (Mreg == 2) {   
        BLK_SEL_W_data = 0x28;
    } else if (Mreg == 3) {
        BLK_SEL_W_data = 0x50;
    }
    send_buffer[0] = BLK_SEL_W;
    send_buffer[1] = BLK_SEL_W_data;
    xfer[0].tx_buf = (unsigned long)send_buffer;
	xfer[0].len = 2;
	xfer[1].rx_buf = (unsigned long)recv_buffer;
	xfer[1].len = 1;
	status = ioctl(fd, SPI_IOC_MESSAGE(2), xfer);
	if (status < 0) {
		perror("SPI_IOC_MESSGE ReadReg");
		return;
	}
    usleep(200);
    //rt_spi_transfer(dev, send_buffer, recv_buffer, 2);
    //rt_thread_delay(1);
    send_buffer[0] = MADDR_R;
    send_buffer[1] = reg;
    xfer[0].tx_buf = (unsigned long)send_buffer;
	xfer[0].len = 2;
	xfer[1].rx_buf = (unsigned long)recv_buffer;
	xfer[1].len = 1;
	status = ioctl(fd, SPI_IOC_MESSAGE(2), xfer);
	if (status < 0) {
		perror("SPI_IOC_MESSGE ReadReg");
		return;
	}
    usleep(200);
    //rt_spi_transfer(dev, send_buffer, recv_buffer, 2);
    //rt_thread_delay(1);
    send_buffer[0] = (0x80 | M_R);
    send_buffer[1] = (0x80 | M_R);
    xfer[0].tx_buf = (unsigned long)send_buffer;
	xfer[0].len = 2;
	xfer[1].rx_buf = (unsigned long)recv_buffer;
	xfer[1].len = 1;
	status = ioctl(fd, SPI_IOC_MESSAGE(2), xfer);
	if (status < 0) {
		perror("SPI_IOC_MESSGE ReadReg");
		return;
	}
    usleep(200);
    //rt_spi_transfer(dev, send_buffer, recv_buffer, 2);
    //rt_thread_delay(1);

    *rddata = recv_buffer[1];
}
// 配置MREG1、2、3 参考手册13节
static void icm42670_writeMregReg(int fd, uint8_t reg, uint8_t wrdata, uint8_t Mreg)
{
    int status;
    uint8_t send_buffer[2];
    uint8_t recv_buffer[2];
    uint8_t BLK_SEL_W_data =0;

    //RT_ASSERT(dev != RT_NULL);
    if (Mreg == 1) {
        BLK_SEL_W_data = 0;
    } else if (Mreg == 2) {   
        BLK_SEL_W_data = 0x28;
    } else if (Mreg == 3) {
        BLK_SEL_W_data = 0x50;
    }
    send_buffer[0] = BLK_SEL_W;
    send_buffer[1] = BLK_SEL_W_data;
    xfer[0].tx_buf = (unsigned long)send_buffer;
	xfer[0].len = 2;
	status = ioctl(fd, SPI_IOC_MESSAGE(1), xfer);
	if (status < 0) {
		perror("SPI_IOC_MESSGE ReadReg");
		return;
	}
    usleep(200);
    //rt_spi_transfer(dev, send_buffer, recv_buffer, 2);
    //rt_thread_delay(1);
    send_buffer[0] = MADDR_W;
    send_buffer[1] = reg;
    xfer[0].tx_buf = (unsigned long)send_buffer;
	xfer[0].len = 2;
	status = ioctl(fd, SPI_IOC_MESSAGE(1), xfer);
	if (status < 0) {
		perror("SPI_IOC_MESSGE ReadReg");
		return;
	}
    usleep(200);
    //rt_spi_transfer(dev, send_buffer, recv_buffer, 2);
    //rt_thread_delay(1);
    send_buffer[0] = (0x80 | M_W);
    send_buffer[1] = wrdata;
    xfer[0].tx_buf = (unsigned long)send_buffer;
	xfer[0].len = 2;
	status = ioctl(fd, SPI_IOC_MESSAGE(1), xfer);
	if (status < 0) {
		perror("SPI_IOC_MESSGE ReadReg");
		return;
	}
    usleep(200);
    //rt_spi_transfer(dev, send_buffer, recv_buffer, 2);
    //rt_thread_delay(1);
}

float get_Icm42670GetAres(int fd)
{
    uint8_t reg_val = 0;
    icm42670_readReg(fd, ICM42670_ACCEL_CONFIG0,&reg_val);
    reg_val>>=5;
    switch (reg_val)
    {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    case AFS_2G:
        accSensitivity = ICM42670_G_PER_LSB_2;
        break;
    case AFS_4G:
        accSensitivity = ICM42670_G_PER_LSB_4;
        break;
    case AFS_8G:
        accSensitivity = ICM42670_G_PER_LSB_8;
        break;
    case AFS_16G:
        accSensitivity = ICM42670_G_PER_LSB_16;
        break;
    }

    return accSensitivity;
}

float get_Icm42670GetGres(int fd)
{
    uint8_t reg_val = 0;
    icm42670_readReg(fd, ICM42670_GYRO_CONFIG0,&reg_val);
    reg_val>>=5;
    switch (reg_val)
    {
    case GFS_250DPS:
        gyroSensitivity = ICM42670_DEG_PER_LSB_250;
        break;
    case GFS_500DPS:
        gyroSensitivity = ICM42670_DEG_PER_LSB_500;
        break;
    case GFS_1000DPS:
        gyroSensitivity = ICM42670_DEG_PER_LSB_1000;
        break;
    case GFS_2000DPS:
        gyroSensitivity = ICM42670_DEG_PER_LSB_2000;
        break;
    }
    return gyroSensitivity;
}

static int spi_init(const char* filename)
{
    int fd = -1;
	uint8_t  mode, msb, bits;
    msb = 0;
    bits = 8;
	uint32_t speed = 2000000;

	if ((fd = open(filename,O_RDWR)) < 0)
	{
		printf("Failed to open the bus.");	
		exit(1);
	}

	///////////////
	// Verifications
	///////////////
	//possible modes: mode |= SPI_LOOP; mode |= SPI_CPHA; mode |= SPI_CPOL; mode |= SPI_LSB_FIRST; mode |= SPI_CS_HIGH; mode |= SPI_3WIRE; mode |= SPI_NO_CS; mode |= SPI_READY;
	//multiple possibilities using |
	

    mode = SPI_MODE_0;
	if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0)   {
        perror("can't set spi mode");
        return -1;
	}

    if (ioctl(fd, SPI_IOC_RD_MODE, &mode) < 0)
	{
		perror("SPI rd_mode");
        return -1;
	}

	if (ioctl(fd, SPI_IOC_RD_LSB_FIRST, &msb) < 0)
	{
		perror("SPI rd_lsb_fist");
        return -1;
	}

	if (ioctl(fd, SPI_IOC_WR_LSB_FIRST, &msb) < 0)
	{
		perror("SPI rd_lsb_fist");
        return -1;
	}
	if (ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits) < 0) 
	{
		perror("SPI bits_per_word");
        return -1;
	}

	if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) 
	{
		perror("SPI bits_per_word");
        return -1;
	}
	
	if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed)<0)  
	{
		perror("can't set max speed hz");
        return -1;
	}
	
	if (ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0) 
	{
		perror("SPI max_speed_hz");
        return -1;
	}


	printf("%s: spi mode %d, %d bits %sper word, %d Hz max\n",filename, mode, bits, msb ? "(lsb first) " : "(msb first)", speed);

	//xfer[0].tx_buf = (unsigned long)buf;
	//xfer[0].len = 3; /* Length of  command to write*/
	xfer[0].cs_change = 0; /* Keep CS activated */
	xfer[0].delay_usecs = 0, //delay in us
	xfer[0].speed_hz = 2000000, //speed
	xfer[0].bits_per_word = 8, // bites per word 8

	//xfer[1].rx_buf = (unsigned long) buf2;
	//xfer[1].len = 4; /* Length of Data to read */
	xfer[1].cs_change = 0; /* Keep CS activated */
	xfer[1].delay_usecs = 0;
	xfer[1].speed_hz = 2000000;
	xfer[1].bits_per_word = 8;

	return fd;
}

int IMUDEV_Open(const char* dev)
{
    int fd = -1;
    fd = spi_init(dev);

    return fd;
}

void IMUDEV_Close(int fd)
{
    close(fd);
}

int ICM42670_Init(int fd)//(char *sensor_name)
{
    //struct rt_spi_configuration cfg;
    uint8_t mpu_test = 0;
    uint8_t reg_val = 0;
    uint8_t id = 0;
    //if(sensor_name == NULL)
    //    return RT_ERROR;
    //icm42670_bo_dev = (struct rt_spi_device *)rt_device_find(sensor_name);
    //if(icm42670_bo_dev == NULL)
    //{
    //    rt_kprintf("[ICM43670] ERR not find %s\n",sensor_name);
    //    return RT_ERROR;
    //}
    //param_get("MPU_TEST", &mpu_test);

    //cfg.data_width = 8;
    //cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible Modes 0 */
    //SPI3 = 84M/4,8,16,32 = 21M, 10.5M, 5.25M, ...
    //cfg.max_hz = 1000*1000; /* 11000kbit/s */
    //rt_spi_configure(icm42670_bo_dev, &cfg);

    // int fd = -1;
    // fd = spi_init(file_name);
    // printf("file_name %s,spidev fd = %d\n",file_name, fd);
    icm42670_writeReg(fd, BLK_SEL_R, 0);// Ensure BLK_SEL_R and BLK_SEL_W are set to 0
    icm42670_writeReg(fd, BLK_SEL_W, 0);
    icm42670_writeReg(fd, DEVICE_CONFIG, 0x04); // 4线spi(标准)

    icm42670_writeReg(fd, BLK_SEL_R, 0);
    icm42670_writeReg(fd, BLK_SEL_W, 0);
    icm42670_writeReg(fd, ICM42670_SIGNAL_PATH_RESET, (1<<4));//reset dev

    icm42670_writeReg(fd, BLK_SEL_R, 0);
    icm42670_writeReg(fd, BLK_SEL_W, 0);
    icm42670_writeReg(fd, ICM42670_DEVICE_CONFIG, 0x04); // 4线spi(标准)
	usleep(500*1000);
    icm42670_readReg(fd, ICM42670_WHO_AM_I,&id);
    icm42670_writeReg(fd, ICM42670_PWR_MGMT0, (1<<4));//要使能IDLE标志位才能配置MREG1、2、3 参考手册13节  
    icm42670_writeReg(fd, ICM42670_FIFO_CONFIG1, 0x01);  //开启FIFO_bypass 跳过FIFO
    icm42670_writeMregReg(fd, ICM42670_FIFO_CONFIG5_MREG1,0x00,1);//关闭所有FIFO
    icm42670_writeReg(fd, ICM42670_INT_SOURCE0, 0x00);//中断配置为0
    get_Icm42670GetAres(fd);
    // icm42670_readReg(icm42670_bo_dev,ICM42670_ACCEL_CONFIG0,reg_val); // page74
    reg_val =0;
    reg_val |= (AFS_16G << 5);                            // 量程 ±8g
    reg_val |= (AODR_400Hz);                             // 输出速率 1600HZ
    icm42670_writeReg(fd, ICM42670_ACCEL_CONFIG0, reg_val);
    get_Icm42670GetGres(fd);
    // reg_val = icm42670_readReg(icm42670_bo_dev,ICM42670_GYRO_CONFIG0,reg_val);
    // page73
    reg_val =0;
    reg_val |= (GFS_2000DPS << 5); // 量程 ±2000dps
    reg_val |= (AODR_400Hz);        // 输出速率 1600HZ
    icm42670_writeReg(fd, ICM42670_GYRO_CONFIG0, reg_val);

    reg_val = 0x02;
    icm42670_writeReg(fd, ICM42670_GYRO_CONFIG1, reg_val);//GYRO LPF 121HZ
    reg_val = 0x02;
    icm42670_writeReg(fd, ICM42670_ACCEL_CONFIG1, reg_val);//acc LPF 121HZ

    reg_val =0; 
    icm42670_readReg(fd, ICM42670_PWR_MGMT0,&reg_val); // 读取PWR—MGMT0当前寄存器的值(page72)
    // reg_val &= ~(1 << 5);//使能温度测量
    reg_val |= ((3) << 2); // 设置GYRO_MODE  0:关闭 1:待机 2:预留 3:低噪声
    reg_val |= (3);        // 设置ACCEL_MODE 0:关闭 1:关闭 2:低功耗 3:低噪声
    icm42670_writeReg(fd, ICM42670_PWR_MGMT0, reg_val);
    usleep(200);
    //rt_thread_delay(1); // 操作完PWR—MGMT0寄存器后 200us内不能有任何读写寄存器的操作

    icm42670_readReg(fd, ICM42670_WHO_AM_I,&id);
    if(id == 0x67)
    {   
        //cfg.max_hz = 1000*20000; /* 20M */
		//rt_spi_configure(icm42670_bo_dev, &cfg);
        printf("[DEV]##ICM42670 found bo_dev.\n");
        return 0;
    }else
    {
         printf("[DEV]##ICM42670 not found bo_dev. id=%x\n", id);
         return -1;
    }
}


static int ICM42670_ReadMulti(int fd, uint8_t reg, uint8_t *buff, uint8_t num)
{
    int status;
	uint8_t send_buffer[2];

	send_buffer[0] = (0x80 | reg);
	xfer[0].tx_buf = (unsigned long)send_buffer;
	xfer[0].len = 1;
	xfer[1].rx_buf = (unsigned long)buff;
	xfer[1].len = num;

	status = ioctl(fd, SPI_IOC_MESSAGE(2), xfer);
	if (status < 0) {
		perror("SPI_IOC_MESSGE ReadReg");
		return -1;
	}

	//printf("ReadReg: %x.\n", buff[0]);

    return 0;
}

int ICM42670_GetMotion6(int fd, uint8_t num, int16_t* ax, int16_t* ay, int16_t* az,
                        int16_t* gx, int16_t* gy, int16_t* gz, int16_t* temp)
{
    uint8_t buffer[14];
	int result;

	if (num == 0)
		result = ICM42670_ReadMulti(fd, ICM42670_TEMP_OUT_H, buffer, 14);
	else
		return -1;
	
	if(result == 0)
	{
		*ax = (((int16_t) buffer[2]) << 8) | buffer[3];
		*ay = (((int16_t) buffer[4]) << 8) | buffer[5];
		*az = (((int16_t) buffer[6]) << 8) | buffer[7];
		*gx = (((int16_t) buffer[8]) << 8) | buffer[9];
		*gy = (((int16_t) buffer[10]) << 8) | buffer[11];
		*gz = (((int16_t) buffer[12]) << 8) | buffer[13];
		
		*temp = (((int16_t) buffer[0]) << 8) | buffer[1];

        //printf("%ld %ld %ld %d %d\n", *ax,*ay,*az, buffer[2], buffer[3]);
	}
	
	return result;
}

// struct rt_spi_device* get_icm42670_dev(char* sensors_name)
// {
// 	if (rt_strcmp(sensors_name,"icm42670_bo") == 0)
// 	{
// 		return icm42670_bo_dev;
// 	}
// 	else
// 	{
// 		rt_kprintf("[icm42670] ERR not  not found dev.");
//         return NULL;
// 	}
// }