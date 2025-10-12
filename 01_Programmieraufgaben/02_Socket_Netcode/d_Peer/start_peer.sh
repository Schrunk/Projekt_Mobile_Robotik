#!/bin/bash

echo "=== Compiling Peer Client ==="
g++ -std=c++11 -pthread -Wall -Wextra -o peer_client peer_client.cpp

if [ $? -eq 0 ]; then
    echo "Compilation successful!"
    echo "=== Starting Peer Client ==="
    
    if [ $# -eq 0 ]; then
        echo "Usage: $0 [client_name]"
        echo "Starting with automatic name..."
        ./peer_client
    else
        echo "Starting client '$1'..."
        ./peer_client "$1"
    fi
else
    echo "Compilation failed!"
    exit 1
fi