#ifndef SHMCMN_H
#define SHMCMN_H



#define SHMFILE  "/dev/shm/gyro0"

struct State_t {
    float angle[3];
	float omega[3];
	float acc[3];
	float tdata[3];
	int bytes_avail;
};

struct State_t * open_shm(char * shmfile);
void close_shm(struct State_t * state_ptr);

long diff_ns(struct timespec * t1, struct timespec * t2);

// ---- Error Macro ----
#define PrintErr(...) do { \
    fprintf (stderr,__VA_ARGS__); \
    fprintf (stderr," @ %s (%d)\n", __FILE__, __LINE__ - 2); \
} while (0)


#endif /* SHMCMN_H */