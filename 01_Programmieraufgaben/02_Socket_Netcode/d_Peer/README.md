# Peer-to-Peer Client System

A decentralized client system where each client can discover and communicate with other clients on the same network.

## Features

- **Automatic Port Assignment**: Each client finds a free port starting from 8080
- **Peer Discovery**: Clients automatically discover each other via PING/PONG mechanism
- **Real-time Messaging**: Send messages to all connected peers
- **Message Queue System**: Messages are queued to avoid input conflicts
- **Asynchronous Replies**: Reply to messages at any time without blocking
- **Peer Management**: List all connected peers and cleanup inactive ones
- **Thread-safe Architecture**: Robust multi-threading implementation

## How It Works

1. Each client starts and finds the first available port from 8080-8089
2. Clients periodically ping all ports in range to discover other clients
3. When a new client is discovered, both clients save each other's information
4. Messages can be broadcast to all peers
5. Incoming messages are queued for later processing
6. Replies can be sent via the `reply` command at any convenient time

## Building

```bash
# Using the provided script:
./start_peer.sh [client_name]

# Or manually:
g++ -std=c++11 -pthread -Wall -Wextra -o peer_client peer_client.cpp
```

## Usage

### Single Client
```bash
./peer_client [optional_name]
```

### Multiple Clients for Testing
```bash
# Terminal 1
./start_peer.sh Alice

# Terminal 2  
./start_peer.sh Bob

# Terminal 3
./start_peer.sh Charlie
```

## Commands

Available commands during execution:

- `list` - Show all known peers
- `send <message>` - Send message to all peers
- `ping` - Manual peer discovery
- `messages` - Show pending message queue
- `reply <sender_id>` - Reply to message from specific peer
- `quit` - Exit application

## Example Session

```
Alice> list
=== Connected Peers ===
Own ID: Alice at 127.0.0.1:8080
  Bob at 127.0.0.1:8081 (last seen 2s ago)
  Charlie at 127.0.0.1:8082 (last seen 3s ago)
===================

Alice> send Hello everyone!
Broadcasting message to 2 peers...

[New message received from Bob]

Alice> messages
=== Pending Messages ===
1. From Bob: "Hi Alice! How are you?"
   Received: 5 seconds ago
2. From Charlie: "Everything good over there?"
   Received: 2 seconds ago
===============

Alice> reply Bob
Replying to Bob: "Hi Alice! How are you?"
Your reply: Thanks, all good! How about you?
Reply sent to Bob.

Alice> messages
=== Pending Messages ===
1. From Charlie: "Everything good over there?"
   Received: 12 seconds ago
===============
```

## Protocol

The system uses a simple text-based protocol:

- `PING <port> <clientId>` - Discovery announcement
- `PONG <port> <clientId>` - Discovery response  
- `MESSAGE <senderId> <content>` - Broadcast message
- `REPLY <senderId> <content>` - Reply to message

## Configuration

- `BASE_PORT`: Starting port (default: 8080)
- `MAX_CLIENTS`: Maximum number of clients (default: 10)
- `DISCOVERY_INTERVAL`: Ping interval in seconds (default: 60)
- `LOCAL`: Localhost mode for local testing

## Network Requirements

- All clients must be on the same network/subnet
- Ports 8080-8089 must be available
- No firewall blocking the port range

## Architecture

The system is based on a multi-threading architecture:

- **Server Thread**: Listens for incoming connections
- **Discovery Thread**: Performs periodic peer discovery
- **Main Thread**: User interaction and command processing
- **Message Queue**: Thread-safe queue for asynchronous messages

### Improvements over Original Version

- **Thread-safe Message Queue**: Eliminates input conflicts between main loop and message handling
- **Asynchronous Replies**: Replies can be sent at any convenient time
- **Enhanced Debug Output**: Comprehensive thread-safe logging functionality
- **Deadlock Prevention**: Robust mutex management prevents thread deadlocks
- **Improved Peer Discovery**: Optimized PING/PONG mechanism with timeout handling