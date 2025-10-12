#ifndef SHARED_MEMORY_LIST_H
#define SHARED_MEMORY_LIST_H

#include "shared_memory_common.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LIST_SHM_KEY 12346
#define LIST_SEM_KEY 54322
#define MAX_LIST_ENTRIES 100
#define MAX_COMMAND_SIZE 64

// Commands for list manipulation
typedef enum {
    CMD_UNKNOWN = 0,
    CMD_ADD,        // add <text>
    CMD_LIST,       // list
    CMD_GET,        // get <index>
    CMD_REMOVE,     // remove <index>
    CMD_CLEAR,      // clear
    CMD_SIZE,       // size
    CMD_FIND,       // find <text>
    CMD_HELP        // help
} ListCommand;

// List entry structure
struct ListEntry {
    char content[MAX_MESSAGE_SIZE];
    time_t timestamp;
    int client_id;
    bool is_active;
};

// Client request structure
struct ClientRequest {
    int client_id;
    ListCommand command;
    char argument[MAX_MESSAGE_SIZE];  // For add, get, remove, find commands
    int index;                        // For get, remove commands
    time_t timestamp;
    bool is_new;
    bool processed;
};

// Server response structure
struct ServerResponse {
    int client_id;
    char content[MAX_MESSAGE_SIZE * 2]; // Larger for list outputs
    bool success;
    int result_count;  // For list operations
    time_t timestamp;
    bool is_new;
};

// Shared data for list server
struct ListSharedData {
    struct ListEntry entries[MAX_LIST_ENTRIES];
    struct ClientRequest client_requests[MAX_CLIENTS];
    struct ServerResponse server_responses[MAX_CLIENTS];
    int entry_count;
    int active_clients;
    time_t last_activity;
};

// Function prototypes for list server
int create_list_shared_memory();
int attach_list_shared_memory();
int create_list_semaphore();
ListCommand parse_command(const char* command_str);
const char* command_to_string(ListCommand cmd);

#ifdef __cplusplus
}
#endif

#endif // SHARED_MEMORY_LIST_H