#!/bin/bash

echo "=== Kompiliere List Server ==="
g++ -std=c++11 -pthread -Wall -Wextra -o list_server list_server.cpp

if [ $? -eq 0 ]; then
    echo "Server-Kompilierung erfolgreich!"
    echo "=== Starte List Server ==="
    ./list_server
else
    echo "Server-Kompilierungsfehler!"
    exit 1
fi