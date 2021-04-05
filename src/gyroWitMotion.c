/* Steve Troxel 2/21/2021

Reads a WitMotion gyro and displays to the console and updates a shared memory region.

*/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

// Shared mem stuff
#include <sys/mman.h>
#include <sys/stat.h>
#include <semaphore.h>

#include "mstate.h"
#include "shmcmn.h"
#include "gyroWitMotion.h"

#define GYROPORT "/dev/ttyUSB0"

struct State_t gyro_state;	
struct State_t *state_ptr;	

const float ACC_CONF   = 16.0 / 32768;  // Denominator sensivity range change as configured 
const float OMEGA_CONV = 250.0 / 32768; // Denominator sensivity range change as configured
const float ANG_CONV   =  180.0 / 32768;

int main(int argc, char **argv) {
	
	signal(SIGINT, intHandler);

    struct timespec t1={0,0}, t2={0,0};

	char shmfile[21]  = SHMFILE;
	char gyroport[21] = GYROPORT;

	// Allow command line port and file specification (dev feature)
	// Have to supply both or none
	// synopsis >GyroRcv /dev/ttyUSB1 /dev/shm/gyro1
	if ( argc == 3 ){
		strncpy(gyroport,argv[1],(sizeof gyroport)-1);	
		strncpy(shmfile,argv[2],(sizeof shmfile)-1);	
	}
	
	printf("Opening %s commport and creating %s mmap file\n",gyroport,shmfile);
	
   	state_ptr = open_shm(shmfile,sizeof(struct State_t));
	//state_ptr = &gyro_state; // <-- if not using mmap

	// ----- Serial ---- //
    int portfd = open_com(gyroport,B115200,33);
	//tcflush(portfd,TCIOFLUSH);

	clock_gettime(CLOCK_MONOTONIC, &t1);
	while(1){

		//read_mpu6050(state_ptr,portfd);
		read_mpu6050_chunk(state_ptr,portfd);
		clock_gettime(CLOCK_MONOTONIC, &t2);
		//long diff = diff_ns(&t1,&t2); printf("diff => %ld",diff);
		//t1.tv_sec = t2.tv_sec; t1.tv_nsec = t2.tv_nsec;

		//read_bytes(portfd);
		print_state(state_ptr);
		//print_omega(state_ptr);
		//memcpy(state_ptr,sizeof(struct State_t));

	}

	close_shm(state_ptr,sizeof(struct State_t));
}	

// Read bytes
void read_bytes(int portfd){

	unsigned char rbuf;
	int bytes_avail;

	// Read while there 11 bytes of data
	int cnt; 
	while ( (cnt = read(portfd, &rbuf, sizeof(rbuf)) ) )
    {
		ioctl(portfd, FIONREAD, &bytes_avail);
		if ( rbuf == 0x55 ) printf("-> %d \n",bytes_avail);
		printf("%02X ",rbuf);		

		if ( cnt != sizeof(rbuf) ) {
			printf("error %d %s\n",cnt,strerror(errno));
			close(portfd);
			exit(0);
		}
	}
}

// ----------------------------------------------------
int read_mpu6050(struct State_t * state_ptr,int portfd){

	unsigned char rbuf[11] = {0};
	unsigned char dummy;
	int bytes_avail;

	// Read while there 11 bytes of data
	int cnt; 

	ioctl(portfd, FIONREAD, &bytes_avail);
	printf("Start %d\n",bytes_avail);

	while ( (cnt = read(portfd, &rbuf, sizeof(rbuf)) ) ) {
		ioctl(portfd, FIONREAD, &bytes_avail);
				
        if ( cnt != sizeof(rbuf) ) {
			int rtn = read(portfd,&rbuf + cnt - 1,sizeof(rbuf)-cnt);
			
			printf("incomplete %d  read %d\n",cnt,rtn);
			
		}

		for ( int i=0;i<sizeof(rbuf);i++){
			printf("%02X ",rbuf[i]);			
		}
		printf("  avail=%d\n",bytes_avail);

		if ( rbuf[0] != 0x55 ) {
			ioctl(portfd, FIONREAD, &bytes_avail);
			
			printf("Out of sync %0X ... flushing %d\n",rbuf[0],bytes_avail);
			tcflush(portfd,TCIOFLUSH);
			read(portfd,&dummy,1);
			
	    	continue; 
		} 


		
 		switch( rbuf[1] ) {
			case 0x51:
				state_ptr->acc[0] = (float)(signed int)(rbuf[3]<<8 | rbuf[2]) * ACC_CONF;
				state_ptr->acc[1] = (float)(rbuf[5]<<8 | rbuf[4]) * ACC_CONF;
				state_ptr->acc[2] = (float)(rbuf[7]<<8 | rbuf[6]) * ACC_CONF;
				state_ptr->tdata[0] = (float)(rbuf[9]<<8| rbuf[8])/340+36.25;
				break;
			case 0x52:
				state_ptr->omega[0] = (float)(signed int)(rbuf[3]<<8 | rbuf[2]) * OMEGA_CONV;
				state_ptr->omega[1] = (float)(signed int)(rbuf[5]<<8 | rbuf[4]) * OMEGA_CONV;
				state_ptr->omega[2] = (float)(signed int)(rbuf[7]<<8 | rbuf[6]) * OMEGA_CONV;
				state_ptr->tdata[1] = (float)(rbuf[9]<<8| rbuf[8])/340+36.25;
				break;
			case 0x53:
				state_ptr->angle[0] = (float)(rbuf[3]<<8 | rbuf[2]) * ANG_CONV;
				state_ptr->angle[1] = (float)(rbuf[5]<<8 | rbuf[4]) * ANG_CONV;
				state_ptr->angle[2] = (float)(rbuf[7]<<8 | rbuf[6]) * ANG_CONV;
				state_ptr->tdata[2] = (float)(rbuf[9]<<8| rbuf[8])/340+36.25;

				return(1);
				
			case 0x54:
				return(1);	
        }
    }
	
	return(1);  
}		

// ----------------------------------------------------
int read_mpu6050_chunk(struct State_t * state_ptr,int portfd){

	unsigned char rbuf[33] = {0};
	unsigned char dummy[11];
	int bytes_avail;

	// Read while there 11 bytes of data
	int cnt; 
	static int init = 1;

	ioctl(portfd, FIONREAD, &bytes_avail);
	//printf("Start %d\n",bytes_avail);

	// First 64 bytes are out of sync... 
	// Bug in mpu640 code? or something uniq with my rig? 
	// This saves a sync sequence...  
	if ( init ) {
	    read(portfd, &rbuf, 33);
	    read(portfd, &rbuf, 31);
		init = 0;
	}

	while ( (cnt = read(portfd, &rbuf, sizeof(rbuf)) ) ) {

		ioctl(portfd, FIONREAD, &bytes_avail);
				
        if ( cnt != sizeof(rbuf) ) {
			int rtn = read(portfd,&rbuf + cnt - 1,sizeof(rbuf)-cnt);
			printf("incomplete %d  read %d\n",cnt,rtn);
			continue; // try again
		}

		/* ------------------------------------ 
		for ( int i=0;i<sizeof(rbuf);i++){
			if ( !( i % 11 ) ) { printf("\n"); }
			printf("%02X ",rbuf[i]);			
		}
		printf("  cnt=%d avail=%d\n",cnt,bytes_avail);
		//-------------------------------------- */
		
		if ( ( rbuf[0] != 0x55 ) || (rbuf[1] != 0x51) )   {
			ioctl(portfd, FIONREAD, &bytes_avail);
			printf("Out of sync %0X %0X ... moving one char to right %d\n",rbuf[0],rbuf[1],bytes_avail);
			read(portfd, &dummy, 1);
			continue; // try again
		} 

		break; // made it
	}	

	state_ptr->acc[0] = (float)(short int)(rbuf[3]<<8 | rbuf[2]) * ACC_CONF;
	state_ptr->acc[1] = (float)(short int)(rbuf[5]<<8 | rbuf[4]) * ACC_CONF;
	state_ptr->acc[2] = (float)(short int)(rbuf[7]<<8 | rbuf[6]) * ACC_CONF;
	state_ptr->tdata[0] = (float)(rbuf[9]<<8| rbuf[8])/340+36.25;

	state_ptr->omega[0] = (float)(short int)(rbuf[14]<<8 | rbuf[13]) * OMEGA_CONV;
	state_ptr->omega[1] = (float)(short int)(rbuf[16]<<8 | rbuf[15]) * OMEGA_CONV;
	state_ptr->omega[2] = (float)(short int)(rbuf[18]<<8 | rbuf[17]) * OMEGA_CONV;
	state_ptr->tdata[1] = (float)(rbuf[20]<<8| rbuf[19])/340+36.25;

	state_ptr->angle[0] = (float)(short int)(rbuf[25]<<8 | rbuf[24]) * ANG_CONV;
	state_ptr->angle[1] = (float)(short int)(rbuf[27]<<8 | rbuf[26]) * ANG_CONV;
	state_ptr->angle[2] = (float)(short int)(rbuf[29]<<8 | rbuf[28]) * ANG_CONV;
	state_ptr->tdata[2] = (float)(rbuf[31]<<8| rbuf[30])/340+36.25;

	return(1);  
}		

// --------------------------------------
void print_state(struct State_t * state_ptr){

	printf("\033[2J\033[F\033[F\033[F\033[F");
	for( int i=0;i<3;i++){
		printf("%7.3f \t %7.3f \t %7.3f\n", state_ptr->angle[i],state_ptr->omega[i],state_ptr->acc[i]);
	}
	//printf("%d\n",state_ptr->bytes_avail);
	//printf("%7.3f %7.3f %7.3f\n",state_ptr->tdata[0],state_ptr->tdata[1],state_ptr->tdata[2]);
}

// --------------------------------------
void print_omega(struct State_t * state_ptr){
	printf("%*s", abs( (int)(state_ptr->omega[2]+1) ) , " ");
	printf("%7.1f\n", state_ptr->omega[2]);
}


// -----------------------------------------------------------------------
int open_com(char * sdevice, speed_t  baudrate,unsigned char vmin) {

	int serial_port = open(sdevice, O_RDWR);
	if (serial_port < 0) {
	
    	fprintf(stderr,"Error %s from open: %s\n", sdevice, strerror(errno));
		exit(-1);
	}

	struct termios tty;

	if(tcgetattr(serial_port, &tty) != 0) {
    	printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
	}

	tty.c_cflag &= ~PARENB;
	tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used

	tty.c_cflag &= ~CSIZE; 
	tty.c_cflag |= CS8; 
	tty.c_cflag &= ~CRTSCTS; 
	tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines
	tty.c_lflag &= ~ICANON;
	tty.c_lflag &= ~ISIG;
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);
	tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
	tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
	tty.c_oflag &= ~ONLCR;
	tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
	tty.c_cc[VMIN] = vmin;   // always wait this many bytes before returning

	// B1200, B1800, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800
	cfsetispeed(&tty, baudrate);

	if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
    	printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
	}

	return(serial_port);
}


void  intHandler(int sig)
{
	exit(1);
}






