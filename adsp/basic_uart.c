#include <dev_fs_lib_serial.h>
#include <dspal_log.h>
#include <errno.h>
#include <fcntl.h>
#include <status.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include "basic_uart.h"
#include "platform.h"

const char *serial_path[MAX_UART_DEV_NUM] = {
  "/dev/tty-2", "/dev/tty-1",
  "/dev/tty-3", "/dev/tty-4",
  "/dev/tty-5", "/dev/tty-6"
};

int serial_fd = -1;
struct termios oldtio;

void port_read_callback(void *context, char *buffer, size_t num_bytes) {
  int rx_dev_id = (int)context;
  char rx_buffer[SERIAL_SIZE_OF_DATA_BUFFER];

  if (num_bytes > 0) {
    memcpy(rx_buffer, buffer, num_bytes);
    rx_buffer[num_bytes] = 0;
    LOG_INFO("/dev/tty-%d read callback received bytes [%d]: %s",
      rx_dev_id, num_bytes, rx_buffer);
  } else {
    LOG_ERR("error: read callback with no data in the buffer");
  }
}

int set_interface_attribs(int fd, int baud, int parity) {
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0)
	{
		LOG_ERR("error %d from tcgetattr", errno);
		return -1;
	}

	cfsetospeed (&tty, baud);
	cfsetispeed (&tty, baud);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK;         // disable break processing

	tty.c_lflag = 0;                // no canonical processing,
                                  // no echoing, no signaling characters

	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN]  = 0;            // read doesn't block
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout
  
  // shut off xon/xoff ctrl
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);

  // map CR to NL (st CR will terminate input)
  // ignore modem controls
  // enable reading
  tty.c_iflag |= (ICRNL | CLOCAL | CREAD);

	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
																	// enable reading
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

  // Clean the modem line
  tcflush(fd, TCIFLUSH);

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
	{
		LOG_ERR("error %d from tcsetattr", errno);
		return -1;
	}
	return 0;
}

void set_blocking(int fd, int should_block) {
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0)
	{
		LOG_ERR("error %d from tggetattr", errno);
		return;
	}

	tty.c_cc[VMIN]  = should_block ? 1 : 0;
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
		LOG_ERR("error %d setting term attributes", errno);
}

void print_array(const uint8_t *buf, uint8_t len) {
  char to_print[4*MAX_MSG_LEN+1],
       a_char[5]; // 3 chars max for a uint8 number and empty space
  uint8_t i=0;

  memset(to_print,0,4*MAX_MSG_LEN+1);

  while (i++ < len) {
    sprintf(a_char,"%d ",*buf++);
    strcat(to_print, a_char);
  }

  LOG_INFO("array: %s",to_print);
}

void print_char_array(const char *buf, uint8_t len) {
  LOG_INFO("char array");
  print_array((const uint8_t *)buf,len);
}

int serial_interface_serial_open() {
	LOG_INFO("Opening serial port");
  serial_fd = open(serial_path[0], O_RDWR | O_NOCTTY);
  if (serial_fd >= SUCCESS) {
    LOG_INFO("Opened serial port number %d", serial_fd);

    // Save current serial port settings
    tcgetattr(serial_fd,&oldtio);
		// Set baud to 115200 bps, 8n1 (no parity)
		set_interface_attribs(serial_fd,B38400,0);
		// Set no blocking
		set_blocking(serial_fd, 0);
  } else {
  //FIXME log error!
	LOG_INFO("Error opening serial port");
    serial_fd = ERROR;
  }

  return serial_fd;
}

int serial_interface_serial_close(int fd) {
	LOG_INFO("Closing serial port");
  
  // restore old port settings
  tcsetattr(fd,TCSANOW,&oldtio);

  if (!close(fd)) {
    LOG_INFO("Successfully closed serial port number %d", fd);
  } else {
    LOG_INFO("Error closing serial port");
    fd = ERROR;
  }

  return fd;
}

int serial_interface_serial_read_callback(int fd) {
  int active_devices = 0;
  int runs, res;

  LOG_INFO("Beginning serial read callback setup");

  struct dspal_serial_ioctl_receive_data_callback recv_cb;
  recv_cb.rx_data_callback_func_ptr = port_read_callback;

  recv_cb.context = (void *)(1);

  res = ioctl(fd,
      SERIAL_IOCTL_SET_RECEIVE_DATA_CALLBACK,
      (void *)&recv_cb);

  LOG_INFO("Set serial read callback on %s %s",
    serial_path[0], res < SUCCESS ? "failed" : "succeeded");

  if (res < SUCCESS) {
    //TODO associate with any file if necessary
    LOG_INFO("Closing file %s",
      serial_path[0]);
    close(fd);
    fd = ERROR;
  }

	return fd;
}

int serial_interface_serial_read(int fd, char* rx_buffer) {
  int num_bytes_read = 0;
  int active_devices = 0;
  int runs, i;

  LOG_INFO("Beginning serial read");
  
  num_bytes_read = read(fd, rx_buffer,
      SERIAL_SIZE_OF_DATA_BUFFER);
  LOG_INFO("%s read bytes [%d]:",
      serial_path[0], num_bytes_read);
  print_char_array(rx_buffer, num_bytes_read);

  if (num_bytes_read < 0) {
    //TODO associate with any file if necessary
    LOG_INFO("Closing file %s",
      serial_path[0]);
    close(fd);
    fd = ERROR;
  }

	return fd;
}

int serial_interface_serial_write(int fd, const char* tx_buffer,
                                  int len) {
  int num_bytes_written = 0;

  LOG_INFO("len: %d", len);

  LOG_INFO("Beginning serial write");

  print_char_array(tx_buffer, len);

  num_bytes_written = write(fd,
      tx_buffer,
      len);

  if (num_bytes_written == (ssize_t)len) {
    //TODO associate with any file if necessary
    LOG_INFO("Wrote %d bytes to %s", num_bytes_written,
        serial_path[0]);
  } else {
    //TODO associate with any file if necessary
    LOG_ERR("failed to write to %s", serial_path[0]);
    LOG_INFO("Closing file %s", serial_path[0]);
    close(fd);
    fd = ERROR;
  }

	return fd;
}

int serial_interface_serial_read_write(int fd, char* rx_buffer, const char* tx_buffer, int len){
  int res = fd;
  if (serial_interface_serial_write(fd, tx_buffer, len) == fd) {
		usleep(SERIAL_SIZE_OF_DATA_BUFFER*100);
		if (serial_interface_serial_read(fd, rx_buffer) != fd) {
    res = ERROR;
		} 
	} else {
		res = ERROR;
	}
  return res;
}

