#include "peer_shared_memory.h"

// Semaphore operations
struct sembuf peer_sem_lock = {0, -1, 0};    // P operation (wait/lock)
struct sembuf peer_sem_unlock = {0, 1, 0};   // V operation (signal/unlock)

// Create shared memory segment for peer system
int create_peer_shared_memory() {
    int shm_id = shmget(PEER_SHM_KEY, sizeof(struct PeerSharedData), IPC_CREAT | 0666);
    if (shm_id == -1) {
        perror("shmget failed");
        return -1;
    }
    return shm_id;
}

// Attach to existing shared memory for peer system
int attach_peer_shared_memory() {
    int shm_id = shmget(PEER_SHM_KEY, sizeof(struct PeerSharedData), 0666);
    if (shm_id == -1) {
        perror("shmget failed - no shared memory found");
        return -1;
    }
    return shm_id;
}

// Create semaphore for peer system
int create_peer_semaphore() {
    int sem_id = semget(PEER_SEM_KEY, 1, IPC_CREAT | 0666);
    if (sem_id == -1) {
        perror("semget failed");
        return -1;
    }
    
    // Initialize semaphore to 1 (unlocked)
    union semun {
        int val;
        struct semid_ds *buf;
        unsigned short *array;
    } sem_union;
    
    sem_union.val = 1;
    if (semctl(sem_id, 0, SETVAL, sem_union) == -1) {
        perror("semctl SETVAL failed");
        return -1;
    }
    
    return sem_id;
}

// Lock semaphore (P operation)
int lock_peer_semaphore(int sem_id) {
    if (semop(sem_id, &peer_sem_lock, 1) == -1) {
        perror("semop lock failed");
        return -1;
    }
    return 0;
}

// Unlock semaphore (V operation)
int unlock_peer_semaphore(int sem_id) {
    if (semop(sem_id, &peer_sem_unlock, 1) == -1) {
        perror("semop unlock failed");
        return -1;
    }
    return 0;
}

// Cleanup resources
void cleanup_peer_resources(int shm_id, int sem_id) {
    if (shm_id != -1) {
        if (shmctl(shm_id, IPC_RMID, NULL) == -1) {
            perror("shmctl IPC_RMID failed");
        } else {
            printf("Peer shared memory cleaned up\n");
        }
    }
    
    if (sem_id != -1) {
        if (semctl(sem_id, 0, IPC_RMID) == -1) {
            perror("semctl IPC_RMID failed");
        } else {
            printf("Peer semaphore cleaned up\n");
        }
    }
}

// Parse command string to command enum
PeerCommand parse_peer_command(const char* command_str) {
    if (strncmp(command_str, "add", 3) == 0) return PEER_CMD_ADD;
    if (strcmp(command_str, "list") == 0) return PEER_CMD_LIST;
    if (strncmp(command_str, "get", 3) == 0) return PEER_CMD_GET;
    if (strncmp(command_str, "remove", 6) == 0) return PEER_CMD_REMOVE;
    if (strcmp(command_str, "clear") == 0) return PEER_CMD_CLEAR;
    if (strcmp(command_str, "clear_all") == 0) return PEER_CMD_CLEAR_ALL;
    if (strcmp(command_str, "size") == 0) return PEER_CMD_SIZE;
    if (strncmp(command_str, "find", 4) == 0) return PEER_CMD_FIND;
    if (strcmp(command_str, "peers") == 0) return PEER_CMD_PEERS;
    if (strcmp(command_str, "help") == 0) return PEER_CMD_HELP;
    return PEER_CMD_UNKNOWN;
}

// Convert command enum to string
const char* peer_command_to_string(PeerCommand cmd) {
    switch (cmd) {
        case PEER_CMD_ADD: return "add";
        case PEER_CMD_LIST: return "list";
        case PEER_CMD_GET: return "get";
        case PEER_CMD_REMOVE: return "remove";
        case PEER_CMD_CLEAR: return "clear";
        case PEER_CMD_CLEAR_ALL: return "clear_all";
        case PEER_CMD_SIZE: return "size";
        case PEER_CMD_FIND: return "find";
        case PEER_CMD_PEERS: return "peers";
        case PEER_CMD_HELP: return "help";
        default: return "unknown";
    }
}