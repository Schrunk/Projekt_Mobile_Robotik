# Makefile für ROS 2 Minimal-Publisher (rclcpp + std_msgs)
# Nutzung: im Root eines ROS 2 Workspace (mit Unterordner 'src/') ausführen.
# Targets:
#   make scaffold  - Erstellt Paket + Beispielcode + CMakeLists
#   make build     - Baut das Paket mit colcon
#   make run       - Startet den Publisher
#   make clean     - Löscht build/install/log
#   make help      - Hilfe

SHELL := /bin/bash

# Paket-/Node-Namen
PKG_NAME := pong_game
NODE_NAME := pong_game

# Workspace-Verzeichnisse
WS_ROOT := $(CURDIR)
SRC_DIR := $(WS_ROOT)/src
PKG_DIR := $(SRC_DIR)/$(PKG_NAME)

# Prüfe Tools
check-deps:
	@command -v ros2 >/dev/null 2>&1 || { echo "❌ 'ros2' nicht gefunden. Bitte ROS 2 Umgebung sourcen (z.B. source /opt/ros/humble/setup.bash)"; exit 1; }
	@command -v colcon >/dev/null 2>&1 || { echo "❌ 'colcon' nicht gefunden. Bitte colcon installieren/sourcen."; exit 1; }
	@echo "✅ Abhängigkeiten gefunden."

# Bauen des Pakets
build: check-deps
	@echo "🔧 Baue Paket $(PKG_NAME) ..."
	@cd "$(WS_ROOT)" && colcon build --packages-select $(PKG_NAME) $(NODE_NAME)
	@echo "✅ Build abgeschlossen."

# Publisher starten
run:
	@echo "🚀 Starte $(PKG_NAME)/$(NODE_NAME) ..."
	@bash -c "source \"$(CURDIR)/install/setup.bash\" && ros2 run $(PKG_NAME) $(NODE_NAME)"

# Sauber aufräumen
clean:
	@echo "🧹 Lösche build/install/log ..."
	@rm -rf "$(WS_ROOT)/build" "$(WS_ROOT)/install" "$(WS_ROOT)/log"
	@echo "✅ Clean abgeschlossen."

help:
	@echo "📖 Makefile-Hilfe (ROS 2 Minimal-Publisher)"
	@echo "make build     - Paket bauen (colcon)"
	@echo "make run       - Publisher starten"
	@echo "make clean     - build/install/log löschen"
	@echo "make check-deps- Abhängigkeiten prüfen"