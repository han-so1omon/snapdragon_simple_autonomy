#ifndef BASIC_UART_H
#define BASIC_UART_H

#ifndef SUCCESS
#define SUCCESS 0
#endif

#ifndef ERROR
#define ERROR -1
#endif

void port_read_callback(void *context, char *buffer, size_t num_bytes);

int set_interface_attribs(int fd, int baud, int parity);

void set_blocking(int fd, int should_block); 

void print_array(const uint8_t *buf, uint8_t len);

void print_char_array(const char *buf, uint8_t len);

int serial_interface_serial_open();

int serial_interface_serial_close(int fd); 

int serial_interface_serial_read_callback(int fd);

int serial_interface_serial_read(int fd, char* rx_buffer);

int serial_interface_serial_write(int fd, const char* tx_buffer,
                                  int len);

int serial_interface_serial_read_write(
    int fd,
    char* rx_buffer,
    const char* tx_buffer,
    int len);

#endif
