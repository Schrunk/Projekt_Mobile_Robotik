CC = gcc
CXX = g++
CFLAGS = -Wall -Wextra -std=c99 -g
CXXFLAGS = -Wall -Wextra -std=c++14 -g
LDFLAGS = 

# Source files
COMMON_SRC = shared_memory_common.c
SERVER_CPP_SRC = a_Server/server.cpp
CLIENT_CPP_SRC = b_Client/client.cpp

# Object files
COMMON_OBJ = $(COMMON_SRC:.c=.o)
SERVER_CPP_OBJ = $(SERVER_CPP_SRC:.cpp=.o)
CLIENT_CPP_OBJ = $(CLIENT_CPP_SRC:.cpp=.o)

# Executables
SERVER_CPP_BIN = a_server/server_cpp
CLIENT_CPP_BIN = b_client/client_cpp

# Default target - build C++ versions
all: $(SERVER_CPP_BIN) $(CLIENT_CPP_BIN)

# C++ Server executable
$(SERVER_CPP_BIN): $(SERVER_CPP_OBJ) $(COMMON_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

# C++ Client executable
$(CLIENT_CPP_BIN): $(CLIENT_CPP_OBJ) $(COMMON_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

# Object files
%.o: %.c shared_memory_common.h
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.cpp shared_memory_common.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean build artifacts
clean:
	rm -f *.o */*.o $(SERVER_CPP_BIN) $(CLIENT_CPP_BIN)

# Clean system IPC resources
clean-ipc:
	@echo "Cleaning up IPC resources..."
	@ipcs -m | grep $(shell whoami) | awk '{print $$2}' | xargs -r ipcrm -m 2>/dev/null || true
	@ipcs -s | grep $(shell whoami) | awk '{print $$2}' | xargs -r ipcrm -s 2>/dev/null || true
	@echo "IPC cleanup complete"

# Run C++ server
run-server: $(SERVER_CPP_BIN)
	./$(SERVER_CPP_BIN)

# Run C++ client
run-client: $(CLIENT_CPP_BIN)
	./$(CLIENT_CPP_BIN)

# Test with C++ versions
test: $(SERVER_CPP_BIN) $(CLIENT_CPP_BIN)
	@echo "Starting C++ server in background..."
	./$(SERVER_CPP_BIN) &
	@echo "Waiting for server to initialize..."
	@sleep 2
	@echo "Starting test client..."
	echo "Hello from C++ test client" | ./$(CLIENT_CPP_BIN) 1

# Help target
help:
	@echo "Available targets:"
	@echo "  all              - Build C++ server and client"
	@echo "  all-versions     - Build both C and C++ versions"
	@echo "  server_cpp       - Build C++ server only"
	@echo "  server_c         - Build C server only"
	@echo "  client_cpp       - Build C++ client only"
	@echo "  client_c         - Build C client only"
	@echo "  clean            - Remove build artifacts"
	@echo "  clean-ipc        - Clean up system IPC resources"
	@echo "  run-server       - Run the C++ server"
	@echo "  run-server-c     - Run the C server"
	@echo "  run-client       - Run the C++ client"
	@echo "  run-client-c     - Run the C client"
	@echo "  test             - Run automated test with C++ versions"
	@echo "  help             - Show this help"

.PHONY: all clean clean-ipc run-server run-client test help