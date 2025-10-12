#include "shared_memory_list.h"
#include <string.h>

// Create shared memory segment for list server
int create_list_shared_memory() {
    int shm_id = shmget(LIST_SHM_KEY, sizeof(struct ListSharedData), IPC_CREAT | 0666);
    if (shm_id == -1) {
        perror("shmget failed");
        return -1;
    }
    return shm_id;
}

// Attach to existing shared memory for list server
int attach_list_shared_memory() {
    int shm_id = shmget(LIST_SHM_KEY, sizeof(struct ListSharedData), 0666);
    if (shm_id == -1) {
        perror("shmget failed");
        return -1;
    }
    return shm_id;
}

// Create semaphore for list server
int create_list_semaphore() {
    int sem_id = semget(LIST_SEM_KEY, 1, IPC_CREAT | 0666);
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

// Parse command string to command enum
ListCommand parse_command(const char* command_str) {
    if (strncmp(command_str, "add", 3) == 0) return CMD_ADD;
    if (strcmp(command_str, "list") == 0) return CMD_LIST;
    if (strncmp(command_str, "get", 3) == 0) return CMD_GET;
    if (strncmp(command_str, "remove", 6) == 0) return CMD_REMOVE;
    if (strcmp(command_str, "clear") == 0) return CMD_CLEAR;
    if (strcmp(command_str, "size") == 0) return CMD_SIZE;
    if (strncmp(command_str, "find", 4) == 0) return CMD_FIND;
    if (strcmp(command_str, "help") == 0) return CMD_HELP;
    return CMD_UNKNOWN;
}

// Convert command enum to string
const char* command_to_string(ListCommand cmd) {
    switch (cmd) {
        case CMD_ADD: return "add";
        case CMD_LIST: return "list";
        case CMD_GET: return "get";
        case CMD_REMOVE: return "remove";
        case CMD_CLEAR: return "clear";
        case CMD_SIZE: return "size";
        case CMD_FIND: return "find";
        case CMD_HELP: return "help";
        default: return "unknown";
    }
}