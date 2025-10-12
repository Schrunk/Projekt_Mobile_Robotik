#ifndef SHARED_MEMORY_COMMON_H
#define SHARED_MEMORY_COMMON_H

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/sem.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SHM_KEY 12345
#define SEM_KEY 54321
#define MAX_MESSAGE_SIZE 256
#define MAX_CLIENTS 5

// Message structure
struct Message {
    int client_id;
    char content[MAX_MESSAGE_SIZE];
    time_t timestamp;
    int is_new;  // Flag to indicate if message is new (1) or processed (0)
};

// Shared memory structure
struct SharedData {
    struct Message client_messages[MAX_CLIENTS];  // Messages from clients
    struct Message server_responses[MAX_CLIENTS]; // Echo responses from server
    int active_clients;
    time_t last_activity;
};

// Semaphore operations
extern struct sembuf sem_lock;    // P operation (wait/lock)
extern struct sembuf sem_unlock;  // V operation (signal/unlock)

// Function prototypes
int create_shared_memory();
int attach_shared_memory();
int create_semaphore();
int lock_semaphore(int sem_id);
int unlock_semaphore(int sem_id);
void cleanup_resources(int shm_id, int sem_id);

#ifdef __cplusplus
}
#endif

#endif // SHARED_MEMORY_COMMON_H