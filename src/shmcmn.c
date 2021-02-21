#include <sys/mman.h>
#include <sys/stat.h>
#include <semaphore.h>
#include <sys/types.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h> // ftruncate

#include "shmcmn.h"

int shmfd; 

struct State_t * open_shm(char * shmfile) {

	shmfd = open(shmfile, O_RDWR | O_CREAT, 0660);
  	if ( shmfd < 0 )  {
		PrintErr("Shared Memory Open failed...");
		exit(-1);
	}

	ftruncate(shmfd, sizeof(struct State_t)); 

	struct State_t * state_ptr = mmap(NULL,
        	    sizeof(struct State_t),
        	    PROT_READ | PROT_WRITE,
				MAP_SHARED, 
       			shmfd,
        	    0);

	return(state_ptr);		

}

void close_shm(struct State_t * state_ptr) {

	/* clean up */
  	munmap(state_ptr, sizeof(struct State_t));
  	close(shmfd);
}


long diff_ns(struct timespec * t1, struct timespec * t2)
{
	// t1 -> prev
	// t2 -> now
	long tdiff; 
	if ( t1->tv_sec == t2->tv_sec ){
		tdiff = t2->tv_nsec - t1->tv_nsec; 
	} 
	else {
		tdiff = (t2->tv_sec - t1->tv_sec) * 1000000000UL + (t2->tv_nsec - t1->tv_nsec); 
	}
	return(tdiff);
}	
