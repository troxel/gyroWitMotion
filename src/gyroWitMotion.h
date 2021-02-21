#ifndef GYRORCV_H
#define GYRORCV_H

#include <termios.h>

void  intHandler(int sig);
int open_com(char * sdevice, speed_t  baudrate,unsigned char vmin);
int read_mpu6050(struct State_t * state, int fd);
int read_mpu6050_chunk(struct State_t * state,int portfd);
void print_state(struct State_t * state);
void print_omega(struct State_t * state);
void read_bytes(int portfd);

#endif 