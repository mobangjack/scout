#include "serial.hpp"

#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <semaphore.h>

Serial::Serial(const char* port, int baudrate)
{
	this->port = port;
	this->baudrate = baudrate;
	this->fd = -1;
}

Serial::~Serial()
{
	release();
}

int Serial::connect()
{
	fd = open(port,  O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (fd == -1)
	{
		printf("Serial open error!\n");
		return 0;
	}
	struct termios newtio;
	bzero(&newtio, sizeof(newtio));
	//设置字符大小
	newtio.c_cflag |= CLOCAL | CREAD;
	newtio.c_cflag &= ~CSIZE;
	//设置停止位
	newtio.c_cflag |= CS8;
	//设置奇偶校验位
	newtio.c_cflag &= ~PARENB;
	newtio.c_iflag &= ~INPCK;
	//波特率
	switch(baudrate) {
	case 9600 :
		cfsetispeed(&newtio, B9600);
		cfsetospeed(&newtio, B9600);
		break;
	case 19200 :
		cfsetispeed(&newtio, B19200);
		cfsetospeed(&newtio, B19200);
		break;
	case 38400 :
		cfsetispeed(&newtio, B38400);
		cfsetospeed(&newtio, B38400);
		break;
	case 57600 :
		cfsetispeed(&newtio, B57600);
		cfsetospeed(&newtio, B57600);
		break;
	case 115200 :
		cfsetispeed(&newtio, B115200);
		cfsetospeed(&newtio, B115200);
		break;
	default :
		cfsetispeed(&newtio, B115200);
		cfsetospeed(&newtio, B115200);
		break;
	}
	//停止位1位
	newtio.c_cflag &= ~CSTOPB;
	//设置最少字符等待时间
	newtio.c_cc[VTIME] = 0;
	newtio.c_cc[VMIN] = 0;
	newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /*Input*/
	newtio.c_oflag &= ~OPOST;   /*Output*/

	tcflush(fd, TCIFLUSH);
	if ((tcsetattr(fd, TCSANOW, &newtio)) != 0)
	{
		perror("Fail to set serial attributes!");
		release();
		return 0;
	}
	tcflush(fd, TCIFLUSH);
	printf("Serial open success!\n");

	return 1;
}

int Serial::rx(unsigned char* data, int len)
{
	return read(fd, data, len);
}

int Serial::tx(unsigned char* data, int len)
{
	return write(fd, data, len);
}

void Serial::release()
{
	if(fd != -1)
	{
		close(fd);
		fd = -1;
	}
}

