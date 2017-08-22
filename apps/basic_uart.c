//#include <pthread.h>
//#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "../common/dspal_log.h"
#include "../common/status.h"
#include "basic_uart.h"
#include "serial_interface.h"

//bool wait_flag = false;

/*
void sig_io_handler(int sig) {
  wait_flag = false;
}

*/

/*
void serial_read_write_loop(void *th_info) {

  char tx_buf[SERIAL_SIZE_OF_DATA_BUFFER];
  double base = 1500, range = 500,
         phase = 1000, phase_offset = M_PI/4;
  uint16_t val;
  uint8_t i;
  int8_t inc = 1;

  serial_thread_info_s *sth_info;

  sth_info = (serial_thread_info_s *)th_info;

  //Set elka packet values
  uint8_t data_offset = 4;
  tx_buf[0] = 11;
  tx_buf[1] = 4;
  tx_buf[2] = 255;
  tx_buf[3] = 255;

  if (sth_info->serial_fd < 0) {
    sth_info->res = ERROR;
    pthread_exit(&(sth_info->res));
  } else {
    sth_info->res = SUCCESS;
    
    // Send sinusoidal signal to motors.
    // 4 motors, offset in phase by pi/4 rad
    while (true) {
      for (i=0;i<4;i++) {
        //val = (uint16_t)(base + range*sin(phase + phase_offset*i));
        //val = 1000*(i+1);
        if (i==0 || i == 1 || i==3) val = 4000;
        else {
          val = phase;
        }
        tx_buf[i*2+data_offset] = get_high_byte(val);
        tx_buf[i*2+data_offset+1] = get_low_byte(val);
        printf("val %d: %d\nhigh byte: %d\tlow byte: %d\n\
combined bytes: %d\n",
            i,val,tx_buf[i*2+4],tx_buf[i*2+5],
            ((tx_buf[i*2+4]<<8) | tx_buf[i*2+5]));
        printf("phase: %f\n",phase);
      }

      //phase += .025;
      //phase = fmod((phase),2*M_PI);
      phase += 5*inc;
      if ((phase > 2000) | (phase < 1500)) {
        if (phase > 2000) phase = 2000;
        else if (phase < 1500) phase = 1500;
        inc *= -1;
      }

      sth_info->serial_fd = example_interface_serial_read_write(sth_info->serial_fd,
          sth_info->rx_buf, tx_buf);

      //sth_info->serial_fd = example_interface_serial_read_write(sth_info->serial_fd,
      //    sth_info->rx_buf, sth_info->tx_buf);
      usleep(20000);
      //usleep(500000);
    }
  }

}
*/

char get_high_byte(uint16_t i) {
  return (i & 0xff00) >> 8;
}

char get_low_byte(uint16_t i) {
  return i & 0xff;
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

int serial_open() {
  return serial_interface_serial_open();
}

int serial_close(int fd) {
  return serial_interface_serial_close(fd);
}

int serial_read_callback(int fd) {
  return serial_interface_serial_read_callback(fd);
}

int serial_read(int fd, char* rx_buffer) {
  return serial_interface_serial_read(fd, rx_buffer);
}

int serial_write(int fd, const char* tx_buffer, int len) {
  LOG_INFO("len: %d", len);

  print_char_array(tx_buffer, len);

  return serial_interface_serial_write(
      fd,
      tx_buffer,
      len);
}

int serial_read_write(int fd,
                      char* rx_buffer,
                      const char* tx_buffer,
                      int len) {
  return serial_interface_serial_read_write(
          fd,
          rx_buffer,
          tx_buffer,
          len);
}
