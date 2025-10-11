#!/bin/bash

echo "=== Kompiliere List Client ==="
g++ -std=c++11 -Wall -Wextra -o list_client list_client.cpp

if [ $? -eq 0 ]; then
    echo "Client-Kompilierung erfolgreich!"
    echo "=== Starte List Client ==="
    ./list_client $1
else
    echo "Client-Kompilierungsfehler!"
    exit 1
fi