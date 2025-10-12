# IPC - Inter Process Communication

Kommunikation zwischen Prozessen über Speicherbereiche hinweg z.B. Client & Server, GUI und Hintergrunddienst

## Varainten

### "lokale" Varainten:
| Mechanismus                            | Beschreibung                                                                                              | Beispiel                          |
| -------------------------------------- | --------------------------------------------------------------------------------------------------------- | --------------------------------- |
| **Pipes / Named Pipes (FIFOs)**        | Ein Prozess schreibt Daten, ein anderer liest sie.                                                        | `pipe()`, `mkfifo()` in Unix      |
| **Message Queues**                     | Nachrichtenbasiert; asynchrones Senden/Empfangen.                                                         | POSIX Message Queues              |
| **Shared Memory**                      | Gemeinsamer Speicherbereich, auf den mehrere Prozesse zugreifen. Schnell, aber erfordert Synchronisation. | `shmget()`, `mmap()`              |
| **Semaphoren / Mutexe**                | Synchronisationsmechanismen für Zugriffskontrolle.                                                        | `sem_init()`, `pthread_mutex_t`   |
| **Signals / Events**                   | Ein Prozess benachrichtigt einen anderen (z. B. Interrupt oder Ereignis).                                 | `kill(pid, SIGUSR1)`              |
| **Sockets (lokal oder über Netzwerk)** | Ermöglichen Kommunikation über TCP/UDP oder Unix Domain Sockets.                                          | `socket()`, `bind()`, `connect()` |

### Netzwerk Varianetn:
| Mechanismus                        | Beschreibung                                                  | Beispiel                         |
| ---------------------------------- | ------------------------------------------------------------- | -------------------------------- |
| **TCP/UDP-Sockets**                | Grundlegende Netzwerkkommunikation.                           | klassische Client/Server-Modelle |
| **RPC (Remote Procedure Call)**    | Prozess A ruft Funktion in Prozess B auf, als wäre sie lokal. | gRPC, XML-RPC                    |
| **Message Broker / Queue-Systeme** | Asynchrone Kommunikation über zentrale Message-Systeme.       | RabbitMQ, Kafka, ZeroMQ, RTOS    |
| **Shared Data Services**           | Prozesse greifen über gemeinsame Datendienste zu.             | Redis, Memcached                 |


## ausgewählte Variante 
- Shared Memory mit Semaphoren

