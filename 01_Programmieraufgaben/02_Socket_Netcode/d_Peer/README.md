# Peer-to-Peer Client System

A decentralized client system where each client can discover and communicate with other clients on the same network.

## Features

- **Automatic Port Assignment**: Each client finds a free port starting from 8080
- **Peer Discovery**: Clients automatically discover each other via ping mechanism
- **Real-time Messaging**: Send messages to all connected peers
- **Interactive Responses**: Receive messages and reply interactively
- **Peer Management**: List all connected peers and cleanup inactive ones

## How It Works

1. Each client starts and finds the first available port from 8080-8089
2. Clients periodically ping all ports in range to discover other clients
3. When a new client is discovered, both clients save each other's information
4. Messages can be broadcast to all peers with interactive response capability

## Building

```bash
make
```

## Usage

### Single Client
```bash
./peer_client [optional_name]
```

### Multiple Clients for Testing
```bash
# Terminal 1
make run1    # Starts "Alice"

# Terminal 2  
make run2    # Starts "Bob"

# Terminal 3
make run3    # Starts "Charlie"
```

### Automated Testing
```bash
chmod +x test_multiple.sh
./test_multiple.sh
```

## Commands

Once a client is running:

- `list` - Show all connected peers
- `send <message>` - Send message to all peers
- `ping` - Manual peer discovery
- `quit` - Exit the application

## Example Session

```
Alice> list
=== Connected Peers ===
Own ID: Alice at 192.168.1.100:8080
  Bob at 192.168.1.100:8081 (last seen 2s ago)
  Charlie at 192.168.1.100:8082 (last seen 3s ago)
===================

Alice> send Hello everyone!
Broadcasting message to 2 peers...

[MESSAGE] Bob: Hi Alice!
Reply? (y/n): y
Your reply: Nice to hear from you!

[REPLY] Bob: Thanks!
```

## Protocol

The system uses a simple text-based protocol:

- `PING <port> <clientId>` - Discover announcement
- `PONG <port> <clientId>` - Discovery response  
- `MESSAGE <senderId> <content>` - Broadcast message
- `REPLY <senderId> <content>` - Reply to message

## Configuration

- `BASE_PORT`: Starting port (default: 8080)
- `MAX_CLIENTS`: Maximum number of clients (default: 10)
- `DISCOVERY_INTERVAL`: Ping interval in seconds (default: 5)

## Network Requirements

- All clients must be on the same network/subnet
- Ports 8080-8089 must be available
- No firewall blocking the port range