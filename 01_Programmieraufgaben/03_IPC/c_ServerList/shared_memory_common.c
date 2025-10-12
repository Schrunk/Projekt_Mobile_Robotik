#include "shared_memory_common.h"

// Semaphore operations
struct sembuf sem_lock = {0, -1, 0};    // P operation (wait/lock)
struct sembuf sem_unlock = {0, 1, 0};   // V operation (signal/unlock)

// Create shared memory segment
int create_shared_memory() {
    int shm_id = shmget(SHM_KEY, sizeof(struct SharedData), IPC_CREAT | 0666);
    if (shm_id == -1) {
        perror("shmget failed");
        return -1;
    }
    return shm_id;
}

// Attach to existing shared memory
int attach_shared_memory() {
    int shm_id = shmget(SHM_KEY, sizeof(struct SharedData), 0666);
    if (shm_id == -1) {
        perror("shmget failed");
        return -1;
    }
    return shm_id;
}

// Create semaphore
int create_semaphore() {
    int sem_id = semget(SEM_KEY, 1, IPC_CREAT | 0666);
    if (sem_id == -1) {
        perror("semget failed");
        return -1;
    }
    
    // Initialize semaphore to 1 (unlocked)
    if (semctl(sem_id, 0, SETVAL, 1) == -1) {
        perror("semctl SETVAL failed");
        return -1;
    }
    
    return sem_id;
}

// Lock semaphore (P operation)
int lock_semaphore(int sem_id) {
    if (semop(sem_id, &sem_lock, 1) == -1) {
        perror("semop lock failed");
        return -1;
    }
    return 0;
}

// Unlock semaphore (V operation)
int unlock_semaphore(int sem_id) {
    if (semop(sem_id, &sem_unlock, 1) == -1) {
        perror("semop unlock failed");
        return -1;
    }
    return 0;
}

// Cleanup resources
void cleanup_resources(int shm_id, int sem_id) {
    if (shm_id != -1) {
        if (shmctl(shm_id, IPC_RMID, NULL) == -1) {
            perror("shmctl IPC_RMID failed");
        } else {
            printf("Shared memory cleaned up\n");
        }
    }
    
    if (sem_id != -1) {
        if (semctl(sem_id, 0, IPC_RMID) == -1) {
            perror("semctl IPC_RMID failed");
        } else {
            printf("Semaphore cleaned up\n");
        }
    }
}