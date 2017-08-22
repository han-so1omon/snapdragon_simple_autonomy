#ifndef BASIC_UART_H
#define BASIC_UART_H

#include <stdint.h>

#ifdef _ROS
#include <platform.h>
#else
#include "../common/platform.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*
typedef struct serial_thread_info{
  int serial_fd, res;
  char tx_buf[SERIAL_SIZE_OF_DATA_BUFFER];
  char rx_buf[SERIAL_SIZE_OF_DATA_BUFFER];
} serial_thread_info_s;
*/

//void sig_io_handler(int sig);

//void serial_read_write_loop(void *bufs);

char get_high_byte(uint16_t i);

char get_low_byte(uint16_t i);

void print_array(const uint8_t *buf, uint8_t len);

void print_char_array(const char *buf, uint8_t len);

int serial_open();

int serial_close(int fd);

int serial_read_callback(int fd);

int serial_read(int fd, char* rx_buffer);

int serial_write(int fd, const char* tx_buffer, int len);

int serial_read_write(int fd,
                      char* rx_buffer,
                      const char* tx_buffer,
                      int len);

#ifdef __cplusplus
}
#endif

#endif
