# Makefile for Peer IPC System
# Supports shared memory communication between multiple peer clients

# Compiler and flags
CXX = g++
CC = gcc
CXXFLAGS = -std=c++14 -Wall -Wextra -O2
CFLAGS = -std=c99 -Wall -Wextra -O2

# Target executable
TARGET = peer_client

# Source files
CXX_SOURCES = peer_client.cpp
C_SOURCES = peer_shared_memory.c
HEADERS = peer_shared_memory.h

# Object files
CXX_OBJECTS = $(CXX_SOURCES:.cpp=.o)
C_OBJECTS = $(C_SOURCES:.c=.o)
OBJECTS = $(CXX_OBJECTS) $(C_OBJECTS)

# Default target
all: $(TARGET)

# Build the main executable
$(TARGET): $(OBJECTS)
	@echo "🔗 Linking peer client..."
	$(CXX) $(OBJECTS) -o $(TARGET)
	@echo "✅ Built $(TARGET)"

# Compile C++ source files
%.o: %.cpp $(HEADERS)
	@echo "⚙️  Compiling C++ file: $<"
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Compile C source files
%.o: %.c $(HEADERS)
	@echo "⚙️  Compiling C file: $<"
	$(CC) $(CFLAGS) -c $< -o $@

# Clean build artifacts
clean:
	@echo "🧹 Cleaning build artifacts..."
	rm -f $(OBJECTS) $(TARGET)
	@echo "✅ Clean completed"

# Clean shared memory and semaphores (use carefully!)
clean-ipc:
	@echo "🧹 Cleaning IPC resources..."
	@ipcs -m | grep 0x1235 | awk '{print $$2}' | xargs -r ipcrm -m 2>/dev/null || true
	@ipcs -s | grep 0x1236 | awk '{print $$2}' | xargs -r ipcrm -s 2>/dev/null || true
	@echo "✅ IPC resources cleaned"

# Full clean (build artifacts + IPC)
distclean: clean clean-ipc

# Install target (copy to bin directory)
install: $(TARGET)
	@echo "📦 Installing $(TARGET)..."
	sudo cp $(TARGET) /usr/local/bin/
	@echo "✅ Installed to /usr/local/bin/"

# Uninstall target
uninstall:
	@echo "🗑️  Uninstalling $(TARGET)..."
	sudo rm -f /usr/local/bin/$(TARGET)
	@echo "✅ Uninstalled from /usr/local/bin/"

# Test with multiple peers
test: $(TARGET)
	@echo "🧪 Testing peer system..."
	@echo "📝 This will start test scenario with multiple peers"
	@echo "⚡ Open multiple terminals and run:"
	@echo "   Terminal 1: ./$(TARGET) 1 Alice"
	@echo "   Terminal 2: ./$(TARGET) 2 Bob"
	@echo "   Terminal 3: ./$(TARGET) 3 Charlie"
	@echo ""
	@echo "💡 Try these commands in different terminals:"
	@echo "   Alice> add Hello from Alice"
	@echo "   Bob> add Bob is here"
	@echo "   Charlie> list"
	@echo "   Alice> peers"
	@echo "   Bob> find Bob"
	@echo ""
	@echo "🚀 Starting first peer (Alice)..."
	./$(TARGET) 1 Alice

# Test single peer
test-single: $(TARGET)
	@echo "🧪 Testing single peer..."
	./$(TARGET) 1 TestPeer

# Debug build
debug: CXXFLAGS += -g -DDEBUG
debug: CFLAGS += -g -DDEBUG
debug: $(TARGET)
	@echo "🐛 Debug build completed"

# Check for memory leaks (requires valgrind)
memcheck: $(TARGET)
	@echo "🔍 Running memory check..."
	@echo "⚠️  Note: This will run interactively, type 'quit' to exit"
	valgrind --leak-check=full --show-leak-kinds=all ./$(TARGET) 1 MemTest

# Show IPC status
ipc-status:
	@echo "📊 IPC Status Report"
	@echo "==================="
	@echo ""
	@echo "📋 Shared Memory Segments:"
	@ipcs -m | head -1
	@ipcs -m | grep 0x1235 || echo "No peer shared memory found"
	@echo ""
	@echo "🔒 Semaphore Arrays:"
	@ipcs -s | head -1
	@ipcs -s | grep 0x1236 || echo "No peer semaphores found"
	@echo ""
	@echo "💡 Use 'make clean-ipc' to remove all IPC resources"

# Show system resource limits
limits:
	@echo "📊 System IPC Limits"
	@echo "===================="
	@echo ""
	@echo "🗂️  Shared Memory Limits:"
	@cat /proc/sys/kernel/shmmax 2>/dev/null | awk '{print "   Max segment size: " $$1 " bytes"}'
	@cat /proc/sys/kernel/shmall 2>/dev/null | awk '{print "   Max total size: " $$1 " pages"}'
	@echo ""
	@echo "🔒 Semaphore Limits:"
	@cat /proc/sys/kernel/sem 2>/dev/null | awk '{print "   Max per array: " $$1 ", Max arrays: " $$4}'

# Show help
help:
	@echo "📖 Peer IPC System Makefile"
	@echo "============================"
	@echo ""
	@echo "🎯 Build Targets:"
	@echo "   all          - Build the peer client (default)"
	@echo "   debug        - Build with debug symbols"
	@echo "   clean        - Remove build artifacts"
	@echo "   clean-ipc    - Remove IPC resources (⚠️  destroys data!)"
	@echo "   distclean    - Full clean (build + IPC)"
	@echo ""
	@echo "📦 Installation:"
	@echo "   install      - Install to /usr/local/bin"
	@echo "   uninstall    - Remove from /usr/local/bin"
	@echo ""
	@echo "🧪 Testing:"
	@echo "   test         - Start multi-peer test scenario"
	@echo "   test-single  - Test with single peer"
	@echo "   memcheck     - Run with valgrind (requires valgrind)"
	@echo ""
	@echo "📊 Monitoring:"
	@echo "   ipc-status   - Show current IPC resources"
	@echo "   limits       - Show system IPC limits"
	@echo ""
	@echo "💡 Usage Examples:"
	@echo "   make && ./peer_client 1 Alice"
	@echo "   make test"
	@echo "   make ipc-status"
	@echo "   make clean-ipc  # ⚠️  Use when system is stuck"

# Force rebuild
rebuild: clean all

# Check dependencies
check-deps:
	@echo "🔍 Checking dependencies..."
	@which $(CXX) >/dev/null || (echo "❌ g++ not found" && exit 1)
	@which $(CC) >/dev/null || (echo "❌ gcc not found" && exit 1)
	@echo "✅ All dependencies found"

# Phony targets
.PHONY: all clean clean-ipc distclean install uninstall test test-single debug memcheck ipc-status limits help rebuild check-deps