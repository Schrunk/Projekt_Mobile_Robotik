# Peer-to-Peer IPC System

A sophisticated peer-to-peer system for shared list management using POSIX shared memory and semaphores. Multiple peer clients can simultaneously access and modify a shared list with full synchronization and ownership tracking.

## 🌟 Features

### Core Functionality
- **Multi-Peer Support**: Up to 10 concurrent peer clients
- **Shared List Operations**: Add, list, get, remove, clear, size, find entries
- **Ownership Management**: Peers can only remove their own entries
- **Real-time Synchronization**: Thread-safe operations with semaphore protection
- **Peer Discovery**: See all connected peers and their activity
- **Activity Tracking**: Monitor last actions and peer statistics

### Advanced Capabilities
- **Automatic Memory Management**: First peer initializes, last peer can clean up
- **Robust Error Handling**: Comprehensive error messages and recovery
- **Timestamp Tracking**: All entries include creation time and author info
- **Search Functionality**: Find entries by content across all peers
- **Interactive Interface**: User-friendly command-line interface

## 🏗️ System Architecture

### Data Structures

```c
typedef struct PeerListEntry {
    int entry_id;           // Unique entry identifier
    int peer_id;           // ID of the peer who created this entry
    char author[64];       // Name of the author peer
    char content[256];     // Entry content
    time_t timestamp;      // Creation timestamp
    bool is_active;        // Whether entry is active
} PeerListEntry;

typedef struct PeerInfo {
    int peer_id;           // Unique peer identifier
    char name[64];         // Peer display name
    time_t last_seen;      // Last activity timestamp
    bool is_active;        // Whether peer is connected
    int entries_created;   // Number of entries created by this peer
} PeerInfo;

typedef struct PeerSharedData {
    PeerListEntry entries[100];    // Shared list entries
    PeerInfo peers[10];            // Connected peer information
    int total_entries;             // Total entries ever created
    int active_entries;            // Currently active entries
    int active_peers;              // Number of connected peers
    int next_entry_id;             // Next available entry ID
    time_t last_activity;          // Last system activity
    char last_action[128];         // Description of last action
} PeerSharedData;
```

### IPC Configuration
- **Shared Memory Key**: `0x1235` (decimal: 4661)
- **Semaphore Key**: `0x1236` (decimal: 4662)
- **Memory Size**: Approximately 30KB for full data structure
- **Synchronization**: Single semaphore for critical section protection

## 🚀 Quick Start

### Building the System

```bash
# Build the peer client
make

# Or build with debug information
make debug
```

### Basic Usage

```bash
# Start first peer
./peer_client 1 Alice

# Start second peer (in another terminal)
./peer_client 2 Bob

# Start third peer with auto-generated name
./peer_client 3
```

### Command Examples

```bash
# In Alice's terminal
Alice> add Hello from Alice
Alice> add This is my second message
Alice> list

# In Bob's terminal  
Bob> add Bob is here too
Bob> peers
Bob> find Alice

# In any terminal
> size
> help
> quit
```

## 📖 Command Reference

### List Management Commands

| Command | Syntax | Description | Example |
|---------|--------|-------------|---------|
| `add` | `add <text>` | Add new entry to shared list | `add Hello World` |
| `list` | `list` | Show all active entries | `list` |
| `get` | `get <id>` | Get detailed info for entry | `get 5` |
| `remove` | `remove <id>` | Remove your own entry | `remove 3` |
| `clear` | `clear` | Remove all your entries | `clear` |
| `clear_all` | `clear_all` | Remove ALL entries (⚠️ careful!) | `clear_all` |
| `size` | `size` | Show list statistics | `size` |
| `find` | `find <text>` | Search entries by content | `find hello` |

### System Commands

| Command | Syntax | Description | Example |
|---------|--------|-------------|---------|
| `peers` | `peers` | Show connected peers | `peers` |
| `help` | `help` | Show command help | `help` |
| `quit` | `quit` | Exit client | `quit` |

## 🧪 Testing & Development

### Multi-Peer Testing

```bash
# Automated test setup
make test

# Manual testing with multiple terminals
# Terminal 1:
./peer_client 1 Alice

# Terminal 2:
./peer_client 2 Bob

# Terminal 3:
./peer_client 3 Charlie
```

### Test Scenarios

1. **Basic Operations**:
   ```bash
   Alice> add First message
   Bob> add Second message  
   Charlie> list
   Alice> get 1
   Bob> remove 2
   ```

2. **Search and Discovery**:
   ```bash
   Alice> add Programming is fun
   Bob> add I love programming too
   Charlie> find programming
   Alice> peers
   ```

3. **Ownership Testing**:
   ```bash
   Alice> add My message
   Bob> remove 1    # Should fail - not Bob's entry
   Alice> remove 1  # Should succeed
   ```

### Memory Testing

```bash
# Check for memory leaks (requires valgrind)
make memcheck
```

## 🛠️ System Administration

### Monitoring IPC Resources

```bash
# Show current IPC status
make ipc-status

# Show system limits
make limits

# Manual IPC inspection
ipcs -m    # Show shared memory segments
ipcs -s    # Show semaphore arrays
```

### Cleanup Operations

```bash
# Clean build artifacts only
make clean

# Clean IPC resources (⚠️ destroys all data!)
make clean-ipc

# Full cleanup
make distclean
```

### Installation

```bash
# Install to system path
make install

# Uninstall
make uninstall
```

## 🔧 Troubleshooting

### Common Issues

1. **"Failed to create shared memory"**
   - Check system IPC limits: `make limits`
   - Clean existing resources: `make clean-ipc`
   - Verify permissions

2. **"Failed to acquire lock"**
   - Another peer may have crashed while holding lock
   - Clean semaphores: `make clean-ipc`
   - Restart all peers

3. **"Peer ID must be between 1 and 10"**
   - Use valid peer ID range
   - Each peer needs unique ID

4. **Compilation Errors**
   - Check dependencies: `make check-deps`
   - Ensure C++14 support: `g++ --version`

### Emergency Recovery

```bash
# If system becomes unresponsive
make clean-ipc    # ⚠️ This destroys all data!

# Force rebuild
make rebuild
```

### System Limits

Check current limits:
```bash
# Shared memory limits
cat /proc/sys/kernel/shmmax    # Max segment size
cat /proc/sys/kernel/shmall    # Max total pages

# Semaphore limits  
cat /proc/sys/kernel/sem       # Semaphore limits
```

## 🏆 Advanced Features

### Peer Statistics
- Track number of entries created per peer
- Monitor last activity timestamps
- System-wide activity logging

### Ownership Model
- Peers can only remove their own entries
- `clear` removes only own entries
- `clear_all` removes everything (use carefully)

### Automatic Cleanup
- First peer initializes shared memory
- Peers automatically register/unregister
- Last peer can clean up resources

### Error Recovery
- Robust semaphore handling
- Automatic retry mechanisms
- Graceful degradation

## 📊 Performance Considerations

### Scalability
- **Max Peers**: 10 concurrent clients
- **Max Entries**: 100 total entries
- **Memory Usage**: ~30KB shared memory
- **Latency**: Microsecond-level lock operations

### Optimization Tips
- Keep entry content under 256 characters
- Use descriptive peer names for clarity
- Regular cleanup of old entries
- Monitor system with `make ipc-status`

## 🔐 Security Notes

- **Local System Only**: IPC keys are system-local
- **No Authentication**: Trust model within single system
- **File Permissions**: Standard Unix permissions apply
- **Resource Limits**: Bounded by system IPC limits

## 📚 Technical References

- **POSIX Shared Memory**: `shmget()`, `shmat()`, `shmctl()`
- **POSIX Semaphores**: `semget()`, `semop()`, `semctl()`
- **C++14 Standard**: Modern C++ features used throughout
- **Thread Safety**: Semaphore-based critical section protection

---

*Built with modern C++14 and POSIX IPC for maximum compatibility and performance.*