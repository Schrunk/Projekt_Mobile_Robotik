CC = gcc
CXX = g++
CFLAGS = -Wall -Wextra -std=c99 -g
CXXFLAGS = -Wall -Wextra -std=c++14 -g
LDFLAGS = 

# Source files
COMMON_SRC = ../shared_memory_common.c
LIST_COMMON_SRC = shared_memory_list.c
LIST_SERVER_SRC = list_server.cpp
LIST_CLIENT_SRC = list_client.cpp

# Object files
COMMON_OBJ = shared_memory_common.o
LIST_COMMON_OBJ = shared_memory_list.o
LIST_SERVER_OBJ = list_server.o
LIST_CLIENT_OBJ = list_client.o

# Executables
LIST_SERVER_BIN = list_server
LIST_CLIENT_BIN = list_client

# Default target
all: $(LIST_SERVER_BIN) $(LIST_CLIENT_BIN)

# List Server executable
$(LIST_SERVER_BIN): $(LIST_SERVER_OBJ) $(COMMON_OBJ) $(LIST_COMMON_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

# List Client executable
$(LIST_CLIENT_BIN): $(LIST_CLIENT_OBJ) $(COMMON_OBJ) $(LIST_COMMON_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

# Copy and compile common shared memory functions
$(COMMON_OBJ): $(COMMON_SRC) shared_memory_common.h
	cp $(COMMON_SRC) .
	$(CC) $(CFLAGS) -c shared_memory_common.c -o $@

# Compile list-specific shared memory functions
$(LIST_COMMON_OBJ): $(LIST_COMMON_SRC) shared_memory_list.h shared_memory_common.h
	$(CC) $(CFLAGS) -c $< -o $@

# Compile C++ source files
%.o: %.cpp shared_memory_common.h shared_memory_list.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean build artifacts
clean:
	rm -f *.o $(LIST_SERVER_BIN) $(LIST_CLIENT_BIN) shared_memory_common.c

# Clean system IPC resources
clean-ipc:
	@echo "Cleaning up IPC resources..."
	@ipcs -m | grep $(shell whoami) | awk '{print $$2}' | xargs -r ipcrm -m 2>/dev/null || true
	@ipcs -s | grep $(shell whoami) | awk '{print $$2}' | xargs -r ipcrm -s 2>/dev/null || true
	@echo "IPC cleanup complete"

# Run List server
run-server: $(LIST_SERVER_BIN)
	./$(LIST_SERVER_BIN)

# Run List client
run-client: $(LIST_CLIENT_BIN)
	./$(LIST_CLIENT_BIN)

# Test list server and client
test: $(LIST_SERVER_BIN) $(LIST_CLIENT_BIN)
	@echo "Starting List server in background..."
	./$(LIST_SERVER_BIN) &
	@SERVER_PID=$$!; \
	echo "Waiting for server to initialize..."; \
	sleep 2; \
	echo "Running automated test..."; \
	echo -e "help\nadd Hello World\nadd Second Entry\nlist\nget 0\nfind Hello\nsize\nclear\nsize\nquit" | ./$(LIST_CLIENT_BIN) 1; \
	echo "Stopping server..."; \
	kill $$SERVER_PID 2>/dev/null || true

# Interactive test - start server and client in separate terminals
interactive: $(LIST_SERVER_BIN) $(LIST_CLIENT_BIN)
	@echo "=========================================="
	@echo "Interactive List Server Test"
	@echo "=========================================="
	@echo "1. Start the server:"
	@echo "   ./$(LIST_SERVER_BIN)"
	@echo ""
	@echo "2. In another terminal, start the client:"
	@echo "   ./$(LIST_CLIENT_BIN) 1"
	@echo ""
	@echo "3. Try these commands in the client:"
	@echo "   help"
	@echo "   add Your first entry"
	@echo "   add Another item"
	@echo "   list"
	@echo "   get 0"
	@echo "   find entry"
	@echo "   size"
	@echo "   remove 1"
	@echo "   list"
	@echo "   clear"
	@echo "   quit"
	@echo "=========================================="

# Help target
help:
	@echo "List Server Makefile - Available targets:"
	@echo "  all          - Build list server and client"
	@echo "  clean        - Remove build artifacts"
	@echo "  clean-ipc    - Clean up system IPC resources"
	@echo "  run-server   - Run the list server"
	@echo "  run-client   - Run the list client"
	@echo "  test         - Automated test of server/client"
	@echo "  interactive  - Show interactive test instructions"
	@echo "  help         - Show this help"
	@echo ""
	@echo "Usage examples:"
	@echo "  make all && make run-server     # Start server"
	@echo "  make run-client                 # Start client (in another terminal)"
	@echo "  make test                       # Run automated test"

.PHONY: all clean clean-ipc run-server run-client test interactive help