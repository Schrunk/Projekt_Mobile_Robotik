#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <errno.h>

#include <micro_ros_platformio.h>
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include <lwip/inet.h>

// --- Globale Transportzustände ---
static int sock = -1;
static struct sockaddr_in agent_addr;
static bool transport_configured = false;

// --- Hilfsfunktionen ---
static bool wifi_ready()
{
  return WiFi.status() == WL_CONNECTED && WiFi.localIP() != (uint32_t)0;
}

// micro-ROS ruft das auf, wenn die Session startet
bool esp32_wifi_transport_open(struct uxrCustomTransport* transport)
{
  (void)transport;

  if (!transport_configured) {
    Serial.println("[uROS/WIFI] open(): transport not configured!");
    return false;
  }

  if (!wifi_ready()) {
    Serial.println("[uROS/WIFI] open(): WiFi not connected yet.");
    return false;
  }

  if (sock >= 0) {
    Serial.println("[uROS/WIFI] open(): socket already open.");
    return true;
  }

  sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sock < 0) {
    Serial.printf("[uROS/WIFI] open(): socket() failed, errno=%d\n", errno);
    return false;
  }

  // (Optional) Socket-Reuse: nicht zwingend, aber praktisch beim Rebooten
  int one = 1;
  setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

  Serial.printf("[uROS/WIFI] open(): UDP socket created (fd=%d), agent=%s:%u\n",
                sock, inet_ntoa(agent_addr.sin_addr), ntohs(agent_addr.sin_port));
  return true;
}

// micro-ROS ruft das auf, wenn die Session endet
bool esp32_wifi_transport_close(struct uxrCustomTransport* transport)
{
  (void)transport;

  if (sock >= 0) {
    close(sock);
    Serial.println("[uROS/WIFI] close(): socket closed.");
    sock = -1;
  }
  return true;
}

// Senden von Paketen an den Agent
size_t esp32_wifi_transport_write(struct uxrCustomTransport* transport,
                                  const uint8_t* buf, size_t len, uint8_t* err)
{
  (void)transport;
  if (sock < 0) {
    Serial.println("[uROS/WIFI] write(): socket not open.");
    if (err) *err = 1;
    return 0;
  }

  // Debug: Größe der ausgehenden Pakete
  // Achtung: kann viel werden – bei Bedarf auskommentieren
  // Serial.printf("[uROS/WIFI] write(): sending %u bytes\n", (unsigned)len);

  int sent = sendto(sock, buf, len, 0,
                    (struct sockaddr*)&agent_addr, sizeof(agent_addr));
  if (sent < 0) {
    int e = errno;
    if (err) *err = 1;
    Serial.printf("[uROS/WIFI] write(): sendto() failed, errno=%d\n", e);
    return 0;
  }
  return (size_t)sent;
}

// Empfangen von Paketen vom Agent
size_t esp32_wifi_transport_read(struct uxrCustomTransport* transport,
                                 uint8_t* buf, size_t len, int timeout_ms, uint8_t* err)
{
  (void)transport;
  if (sock < 0) {
    if (err) *err = 1;
    Serial.println("[uROS/WIFI] read(): socket not open.");
    return 0;
  }

  fd_set readset;
  FD_ZERO(&readset);
  FD_SET(sock, &readset);

  struct timeval tv;
  tv.tv_sec  = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;

  int sel = select(sock + 1, &readset, NULL, NULL, &tv);
  if (sel < 0) {
    int e = errno;
    if (err) *err = 1;
    Serial.printf("[uROS/WIFI] read(): select() failed, errno=%d\n", e);
    return 0;
  }
  if (sel == 0) {
    // Timeout – kein Fehler, einfach kein Paket
    return 0;
  }

  int recvd = recvfrom(sock, buf, len, 0, NULL, NULL);
  if (recvd < 0) {
    int e = errno;
    if (err) *err = 1;
    Serial.printf("[uROS/WIFI] read(): recvfrom() failed, errno=%d\n", e);
    return 0;
  }

  // Debug:
  // Serial.printf("[uROS/WIFI] read(): received %d bytes\n", recvd);
  return (size_t)recvd;
}

// Benutzerfreundlicher Setup-Wrapper: konfiguriert nur die Zieladresse und registriert den Transport.
// Der Socket wird ERST in open() erstellt.
bool set_microros_wifi_transports(const char* ssid,
                                  const char* password,
                                  const char* agent_ip,
                                  uint16_t agent_port)
{
  (void)ssid;
  (void)password;

  if (!wifi_ready()) {
    Serial.println("[uROS/WIFI] config(): WiFi not connected. Did you call WiFi.begin() and wait?");
    // Kein Hard-Fehler – manche rufen das vor dem Connect auf.
  }

  memset(&agent_addr, 0, sizeof(agent_addr));
  agent_addr.sin_family = AF_INET;
  agent_addr.sin_port   = htons(agent_port);

  if (inet_pton(AF_INET, agent_ip, &agent_addr.sin_addr.s_addr) != 1) {
    Serial.println("[uROS/WIFI] config(): Invalid Agent IP!");
    return false;
  }

  rmw_uros_set_custom_transport(
      /*is_reliable=*/true,          // UDP ist unzuverlässig, aber micro-ROS erwartet hier nur "custom transport"
      /*args=*/NULL,
      esp32_wifi_transport_open,
      esp32_wifi_transport_close,
      esp32_wifi_transport_write,
      esp32_wifi_transport_read);

  transport_configured = true;
  Serial.printf("[uROS/WIFI] config(): custom transport set for %s:%u\n",
                agent_ip, agent_port);
  return true;
}
