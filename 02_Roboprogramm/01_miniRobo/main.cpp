#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

// --- Pins ---
#define UART_TX_PIN 17
#define START_BYTE 0xAA
#define BUMPER_LEFT 12
#define BUMPER_RIGHT 14
#define BUZZER_PIN 4
#define TRIG_PIN 26
#define ECHO_PIN 25

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- ROS Variablen ---
rcl_subscription_t sub_cmd_vel;
rcl_subscription_t sub_beep;
rcl_subscription_t sub_display;
rcl_publisher_t pub_bumper;
rcl_publisher_t pub_distance;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

geometry_msgs__msg__Twist msg_cmd;
std_msgs__msg__Bool msg_beep;
std_msgs__msg__String msg_display;
std_msgs__msg__Bool msg_bumper;
std_msgs__msg__Float32 msg_distance;

// --- Helper ---
#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) error_loop(); }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

void error_loop() {
  while (1) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
  }
}

// --- Hardware Funktionen ---
bool bumper_triggered() {
  return (digitalRead(BUMPER_LEFT) == LOW || digitalRead(BUMPER_RIGHT) == LOW);
}

float readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  return duration / 58.0;
}

void send_motor_command(uint8_t id, uint8_t mode, uint8_t dir, uint16_t param) {
  Serial2.write(START_BYTE);
  Serial2.write(id);
  Serial2.write(mode);
  Serial2.write(dir);
  Serial2.write((uint8_t)(param >> 8));
  Serial2.write((uint8_t)(param & 0xFF));
}

void beep_warning() {
  for (int i = 0; i < 200; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(1);
    digitalWrite(BUZZER_PIN, LOW);
    delay(1);
  }
}

void update_display_text(const char *text) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(text);
  display.display();
}

// --- ROS Callbacks ---
unsigned long last_cmd_time = 0;
bool motor_active = false;

void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  float linear = msg->linear.x;
  float angular = msg->angular.z;

  uint8_t dir1 = 0, dir2 = 0;
  uint16_t stepsize = 0;
  uint8_t mode = 1;

  if (linear != 0) {
    dir1 = (linear <= 0) ? 0 : 1;
    dir2 = (linear >= 0) ? 0 : 1;
    stepsize = 3;
  } else if (angular != 0) {
    dir1 = (angular <= 0) ? 0 : 1;
    dir2 = (angular <= 0) ? 0 : 1;    
    stepsize = 3;
  } else {
    stepsize = 0;
  }
  // Debug-Ausgaben
  Serial.println("=== Neue cmd_vel Nachricht ===");
  Serial.print("linear.x: ");
  Serial.println(linear, 3);   // mit 3 Nachkommastellen
  Serial.print("angular.z: ");
  Serial.println(angular, 3);
  Serial.print("stepsize: ");
  Serial.println(stepsize);
  Serial.print("dir1: ");
  Serial.println(dir1);
  Serial.print("dir2: ");
  Serial.println(dir2);
  Serial.println("------------------------------");
  send_motor_command(1, mode, dir1, stepsize);
  send_motor_command(2, mode, dir2, stepsize);

  last_cmd_time = millis();
  motor_active = (stepsize > 0);
}

void beep_callback(const void *msgin) {
  (void)msgin;
  beep_warning();
}

void display_callback(const void *msgin) {
  const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
  Serial.println("[ROS2] Callback erhalten!");

  // Inhalt anzeigen
  Serial.printf("[ROS2] Daten: %.*s\n", (int)msg->data.size, msg->data.data);

  char buffer[128];
  snprintf(buffer, sizeof(buffer), "%.*s", (int)msg->data.size, msg->data.data);
  update_display_text(buffer);
}

// --- Setup ---
void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, -1, UART_TX_PIN);

  pinMode(BUMPER_LEFT, INPUT_PULLUP);
  pinMode(BUMPER_RIGHT, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    error_loop();
  }
  update_display_text("Booting...");

  // --- WLAN / micro-ROS Agent ---
  IPAddress agent_ip(192, 168, 178, 141);
  uint16_t agent_port = 8888;
  char ssid[] = "MKWlan";          // ✅ Lösung 1: kein const
  char psk[]  = "MK57241381";      // ✅ Lösung 1: kein const

  set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_robot_node", "", &support));

  // Subscriber
  RCCHECK(rclc_subscription_init_default(
      &sub_cmd_vel, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel"));
  RCCHECK(rclc_subscription_init_default(
      &sub_beep, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/beep"));
  RCCHECK(rclc_subscription_init_default(
      &sub_display, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/display_text"));

  // Publisher
  RCCHECK(rclc_publisher_init_default(
      &pub_bumper, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/bumper_state"));
  RCCHECK(rclc_publisher_init_default(
      &pub_distance, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/distance"));

  msg_display.data.data = (char *)malloc(128);
  msg_display.data.size = 0;
  msg_display.data.capacity = 128;

  msg_beep.data = false;

  // Executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd_vel, &msg_cmd, &cmd_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_beep, &msg_beep, &beep_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_display, &msg_display, &display_callback, ON_NEW_DATA));

  update_display_text("Ready");
}

// --- Loop ---
void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50)));

  // --- Sensor Publish ---
  msg_bumper.data = bumper_triggered();
  RCSOFTCHECK(rcl_publish(&pub_bumper, &msg_bumper, NULL));

  msg_distance.data = readUltrasonic();
  RCSOFTCHECK(rcl_publish(&pub_distance, &msg_distance, NULL));

  if (motor_active && (millis() - last_cmd_time > 300)) {
    // Timeout -> Motor anhalten
    send_motor_command(1, 0, 0, 0);
    send_motor_command(2, 0, 0, 0);
    motor_active = false;
  }

  //delay(100);
}
