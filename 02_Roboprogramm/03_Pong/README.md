# TurtleBot4 Pong Game

In dem folgenden Projekt geht es um die Erstellung eines rudimentären Pong Spieles. Der Turtlebot4 agiert dabei als Puk, welcher von Seite zu Seite fährt. 
Bevor ein Spiel Starten kann müssen die Nutzer ein Feld vorbereiten. Dieses sollte optimaler Weise aus einem rechteckigem Gang mit flachen Seiten bestehen. Die Punktezonen des Spielfeldes sollten unterschiedlich zum Boden gefärbt werden. Umso größer der Unterschied umso besser funktioniert das Punktetracking. 

## INIT State
- Sensoren und Dienste werden gestartet und geladen
- 

## IDLE State
- Roboter wartet auf weitere Eingaben
- Übergang zu SETUP wenn Butten 2 (rechter) gedrückt

## SETUP State
- Kalibirerung der Bodenrückgaben
- zuerst setzen auf Spielfeldboden -> Odometer Wert speichern
- Roboter wird angehoben -> wenn wieder abgesetzt
- Stand auf Punktefläche -> Odometerwert abspeichern
- sinnvolle Grenze durch Algo setzen
- wieder anheben und absetzen -> Setzen des Mittelpunktes

## DRIVE State
- kontinuierliche Anstereuerung der Motoren (geradeaus Fahrt)
- Fahrt bis Bumper anschlägt -> Wechsel in BOUNCE

## BOUNCE State
- Berechnung des Winkels zur Wand mittels der 

Ein vollständiges Template für eine State Machine basierend auf dem **State Pattern** Design Pattern. Dieses Template implementiert die fünf gewünschten States mit den drei Hauptfunktionen `onEnter()`, `run()` und `onExit()`.

## 🏗️ Design Pattern: State Pattern

Das State Pattern ermöglicht es einem Objekt, sein Verhalten zu ändern, wenn sich sein interner Zustand ändert. Das Objekt erscheint, als hätte es seine Klasse geändert.

### Hauptkomponenten:
- **State (Abstrakte Basisklasse)**: Definiert die Schnittstelle für alle konkreten States
- **ConcreteState**: Implementiert das Verhalten für einen spezifischen Zustand
- **StateMachine (Context)**: Verwaltet den aktuellen State und Übergänge

## 🎯 Implementierte States

### 1. **InitState** - Initialisierungszustand
```cpp
// Funktionen:
void onEnter();  // System-Setup, Hardware-Initialisierung
void run();      // Schrittweise Initialisierung  
void onExit();   // Übergang zu IdleState
```
**Zweck**: Systemstart, Kalibrierung, Verbindungsaufbau

### 2. **IdleState** - Wartezustand  
```cpp
// Funktionen:
void onEnter();  // Bereitschaftsmodus aktivieren
void run();      // Auf Befehle/Events warten
void onExit();   // Vorbereitung für aktiven Zustand
```
**Zweck**: Warten auf Benutzereingaben oder Sensor-Events

### 3. **DriveState** - Fahrzustand
```cpp
// Funktionen: 
void onEnter();  // Navigation starten
void run();      // Bewegung ausführen, Hindernisse erkennen
void onExit();   // Bewegung stoppen, Position speichern
```
**Zweck**: Normale Navigation und Fortbewegung

### 4. **BounceState** - Hindernisausweichung
```cpp
// Funktionen:
void onEnter();  // Ausweichmanöver initialisieren
void run();      // Rückwärts fahren, drehen, Weg prüfen
void onExit();   // Ausweichsensoren zurücksetzen
```
**Zweck**: Reaktion auf Hindernisse mit Ausweichverhalten

### 5. **BackToStartState** - Rückkehr zum Start
```cpp
// Funktionen:
void onEnter();  // Rücknavigation planen
void run();      // Route zur Startposition folgen
void onExit();   // Heimkehr abschließen
```
**Zweck**: Automatische Rückkehr zur Ausgangsposition

## 🔄 State Transitions

```
InitState ────────→ IdleState
    ↑                   ↓
    │                DriveState ←──────┐
    │                ↓       ↓        │
    │           BounceState   BackToStartState
    │                ↑             ↓
    └────────────────┴─────────────┘
```

## 🚀 Verwendung

### Build und Ausführung:
```bash
# Bauen
make

# Ausführen  
make run

# Mit Debug-Informationen
make debug && make run-debug
```

### Beispiel-Ausgabe:
```
========================================
    TurtleBot4 State Machine Demo
========================================

[StateMachine] Created
[StateMachine] Initializing states...
[InitState] Entering initialization state
[InitState] Step 1: Initializing hardware...
[InitState] Step 2: Checking node availability...
[InitState] Initialization complete! Transitioning to IDLE state
[IdleState] Entering idle state
[IdleState] Robot is ready and waiting for commands...
```

## 📁 Projektstruktur

```
02_Turtlebot4/
├── src/
│   ├── main.cpp                 # Hauptprogramm
│   ├── statemachine.hpp/.cpp    # State Machine Implementierung
│   └── states/
│       ├── state.hpp            # Abstrakte Basisklasse
│       ├── init_state.hpp/.cpp  # Initialisierungszustand
│       ├── idle_state.hpp/.cpp  # Wartezustand
│       ├── bounce_state.hpp/.cpp# Hindernisausweichung
│       ├── drive_state.hpp/.cpp # Fahrzustand
│       └── back_to_start_state.hpp/.cpp # Rückkehr
├── Makefile                     # Build-System
└── README.md                    # Diese Datei
```

## 🛠️ Template-Funktionen

### Basis State-Klasse:
```cpp
class State {
public:
    virtual void onEnter() = 0;  // Setup beim State-Eintritt
    virtual void run() = 0;      # Hauptlogik (wird wiederholt aufgerufen)
    virtual void onExit() = 0;   # Cleanup beim State-Verlassen
    virtual const char* getName() const = 0;  // Debug-Name

protected:
    StateMachine* m_stateMachine;  // Referenz für Übergänge
};
```

### State Machine:
```cpp
class StateMachine {
public:
    void initialize();                        // Initialisierung
    void update();                           // Ein Update-Zyklus
    void transitionTo(StateType newState);   // State-Übergang
    StateType getCurrentStateType() const;    // Aktueller State
    bool isRunning() const;                  // Läuft die Machine?
    void stop();                             // Stoppen
};
```

## 🎯 Anpassung für eigene Projekte

### 1. Neue States hinzufügen:
```cpp
// 1. StateType erweitern
enum class StateType {
    // ... bestehende States
    MY_NEW_STATE
};

// 2. State-Klasse erstellen
class MyNewState : public State {
    void onEnter() override { /* Setup */ }
    void run() override { /* Logik */ }  
    void onExit() override { /* Cleanup */ }
    const char* getName() const override { return "MyNewState"; }
};

// 3. In StateMachine registrieren
registerState(StateType::MY_NEW_STATE, std::make_unique<MyNewState>(this));
```

### 2. State-Übergänge anpassen:
```cpp
// In einem State's run() method:
if (condition) {
    m_stateMachine->transitionTo(StateType::TARGET_STATE);
}
```

### 3. Zusätzliche Daten übergeben:
```cpp
// State-Klasse erweitern:
class DriveState : public State {
private:
    float m_targetX, m_targetY;  // Zielkoordinaten
    
public:
    void setTarget(float x, float y) { m_targetX = x; m_targetY = y; }
};
```

## 🔧 Entwicklungstools

```bash
# Code formatieren
make format

# Statische Analyse  
make analyze

# Speicher-Check
make memcheck

# State-Übersicht anzeigen
make show-states

# Hilfe anzeigen
make help
```

## 📚 Design Pattern Vorteile

1. **Erweiterbarkeit**: Neue States einfach hinzufügbar
2. **Wartbarkeit**: Jeder State ist isoliert und testbar  
3. **Flexibilität**: State-Übergänge sind konfigurierbar
4. **Debugging**: Klare State-Hierarchie und Logging
5. **Wiederverwendbarkeit**: Template für andere Roboter-Projekte

## 🎮 Anwendungsszenarien

- **Autonome Navigation**: IDLE → DRIVE → BOUNCE → DRIVE
- **Missionsabschluss**: DRIVE → BACK_TO_START → IDLE
- **Systemstart**: INIT → IDLE
- **Fehlerbehandlung**: Jeder State kann zu IDLE zurückkehren

---

*Dieses Template folgt modernen C++17 Standards und bewährten Design Patterns für robuste Robotersteuerung.*
