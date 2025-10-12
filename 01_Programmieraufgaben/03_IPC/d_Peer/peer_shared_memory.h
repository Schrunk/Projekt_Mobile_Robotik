#ifndef PEER_SHARED_MEMORY_H
#define PEER_SHARED_MEMORY_H

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
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PEER_SHM_KEY 12347
#define PEER_SEM_KEY 54323
#define MAX_PEER_LIST_ENTRIES 50
#define MAX_PEER_CLIENTS 10
#define MAX_PEER_MESSAGE_SIZE 256
#define MAX_PEER_NAME_SIZE 32

// List entry for peer system
struct PeerListEntry {
    char content[MAX_PEER_MESSAGE_SIZE];
    char author[MAX_PEER_NAME_SIZE];
    time_t timestamp;
    int peer_id;
    bool is_active;
    int entry_id;  // Unique identifier for the entry
};

// Peer information
struct PeerInfo {
    int peer_id;
    char name[MAX_PEER_NAME_SIZE];
    time_t last_seen;
    bool is_active;
    int entries_created;
};

// Commands for list operations
typedef enum {
    PEER_CMD_UNKNOWN = 0,
    PEER_CMD_ADD,       // add <text>
    PEER_CMD_LIST,      // list
    PEER_CMD_GET,       // get <entry_id>
    PEER_CMD_REMOVE,    // remove <entry_id>
    PEER_CMD_CLEAR,     // clear (only own entries)
    PEER_CMD_CLEAR_ALL, // clear_all (admin function)
    PEER_CMD_SIZE,      // size
    PEER_CMD_FIND,      // find <text>
    PEER_CMD_PEERS,     // peers (show connected peers)
    PEER_CMD_HELP       // help
} PeerCommand;

// Shared data structure for peers
struct PeerSharedData {
    struct PeerListEntry entries[MAX_PEER_LIST_ENTRIES];
    struct PeerInfo peers[MAX_PEER_CLIENTS];
    int total_entries;
    int active_entries;
    int next_entry_id;
    int active_peers;
    time_t last_activity;
    char last_action[MAX_PEER_MESSAGE_SIZE];
};

// Semaphore operations
extern struct sembuf peer_sem_lock;
extern struct sembuf peer_sem_unlock;

// Function prototypes
int create_peer_shared_memory();
int attach_peer_shared_memory();
int create_peer_semaphore();
int lock_peer_semaphore(int sem_id);
int unlock_peer_semaphore(int sem_id);
void cleanup_peer_resources(int shm_id, int sem_id);

PeerCommand parse_peer_command(const char* command_str);
const char* peer_command_to_string(PeerCommand cmd);

#ifdef __cplusplus
}
#endif

#endif // PEER_SHARED_MEMORY_H