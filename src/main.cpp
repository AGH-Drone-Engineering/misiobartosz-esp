#include <Arduino.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <HardwareSerial.h>
#include <MAVLink.h>
#include "sbus.h"



void sendMission(float lat_wp, float lon_wp, float altitude, float delay_seconds, uint8_t servo_channel = 5, uint16_t servo_value = 94);
boolean fetch_and_read();
void send();

uint8_t servo_channel = 5;
uint8_t servo_value = 94;

uint8_t system_id = 1;
uint8_t component_id = MAV_COMP_ID_AUTOPILOT1;
uint8_t target_system = 1; // ID Twojego drona
uint8_t target_component = MAV_COMP_ID_AUTOPILOT1;

HardwareSerial mySerial(1); // Używamy portu szeregowego 1

// Set your Static IP address
IPAddress local_IP(192, 168, 18, 90);
IPAddress gateway(192, 168, 18, 1);

IPAddress subnet(255, 255, 0, 0);
IPAddress primaryDNS(8, 8, 8, 8);   // optional
IPAddress secondaryDNS(8, 8, 4, 4); // optional

const char* ssid = "toya89029830";
const char* password = "21378317";

WebServer server(80);

double longitude = 0.0;
double latitude = 0.0;
double altitude = 0.0;
double delay_seconds = 300.0;

bool dataReceived = false;
bool setMavlink = false;
bool set_proxy = false;

// dane do sbus
const int RX_PIN = D2;
const int TX_PIN = D3;
const int SW_PIN = D7;

// throttle
const int THR_CH = 2;
const int THR_0 = 200;
const int THR_60 = 1200;

// arm
const int ARM_CH = 4;
const int ARM_FALSE = 200;
const int ARM_TRUE = 1800;

// mode
const int MODE_CH = 5;
const int MODE_1 = 200;
const int MODE_2 = 1000;
const int MODE_3 = 1800;

// czas spadku [ms]
const int TIME_FALL = 500;
const int TIME_60 = 4500;
const int REPEAT = 20;

int thr_val = THR_0;
int arm_val = ARM_FALSE;
int mode_val = MODE_1;

boolean is_mounted = false;
boolean is_released = true;
boolean is_auto = false;

bfs::SbusRx sbus_rx(&mySerial, RX_PIN, TX_PIN, true);
bfs::SbusTx sbus_tx(&mySerial, RX_PIN, TX_PIN, true);
bfs::SbusData data;

void setup()
{
  delay(2000);

  Serial.begin(115200);

  pinMode(SW_PIN, INPUT_PULLUP);

  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS))
  {
    Serial.println("STA Failed to configure");
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print('.');
    delay(500);
  }
  Serial.println("\nConnected to Wi-Fi");

  // Endpoint do ustawiania współrzędnych
  server.on("/set-coordinates", HTTP_POST, []()
            {
    if (server.hasArg("plain"))
     {
      String body = server.arg("plain");
      StaticJsonDocument<300> doc;
      DeserializationError error = deserializeJson(doc, body);
      if (error) {
        server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid JSON\"}");
        return;
      }

      longitude = doc["long"];
      latitude = doc["lat"];
      altitude = doc["altitude"];
      delay_seconds = doc["delay"];
      servo_value = doc["servo_value"];

      server.send(200, "application/json", "{\"status\":\"success\"}");
      dataReceived = true;

      // Zatrzymanie serwera po odebraniu danych
      server.stop();
      WiFi.disconnect(true);
    } else {
      server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"No data received\"}");
    } });
  server.begin();
}

void loop()
{
  server.handleClient();

  if (dataReceived)
  {
    if (!setMavlink)
    {
      mySerial.begin(57600, SERIAL_8N1, 2, 3); // Komunikacja MAVLink na pinach 2 i 3
      setMavlink = true;
      sendMission(latitude, longitude, altitude, delay_seconds, servo_channel, servo_value);
    }

    while (mySerial.available())
    {
      mavlink_message_t msg;
      mavlink_status_t status;
      uint8_t c = mySerial.read();
      if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
      {
        Serial.println("Odebrano wiadomość MAVLink");
        mySerial.end(); // Zakończenie komunikacji MAVLink
        Serial.println("Zakończono komunikację MAVLink");

        // Przełączenie na SBUS po zakończeniu komunikacji MAVLink
        set_proxy = true;

        dataReceived = false;
        sbus_rx.Begin();
        sbus_tx.Begin();
        break;
      }
    }
  }
  if (set_proxy)
  {
    // Inicjalizacja SBUS na tych samych pinach co wcześniej MAVLink
    // Logika SBUS
    if (is_released && digitalRead(SW_PIN) == LOW)
    {
      Serial.println("aa");

      is_mounted = true;
      is_released = false;
    }
    if (!is_released && digitalRead(SW_PIN) == HIGH)
    {

      is_auto = true;
      is_mounted = false;
      thr_val = THR_0;
      arm_val = ARM_TRUE;
      mode_val = MODE_2;

      for (int i = 0; i < TIME_FALL; i += REPEAT)
      {
        Serial.println(fetch_and_read());

        send();
        delay(REPEAT);
      }

      thr_val = THR_60;
      for (int i = 0; i < TIME_60; i += REPEAT)
      {
        Serial.println("dd");

        fetch_and_read();
        send();
        delay(REPEAT);
      }

      fetch_and_read();
      mode_val = MODE_3;
      send();

      is_released = true;
    }

    if (fetch_and_read())
    {
      Serial.println("cc");

      for (int8_t i = 0; i < data.NUM_CH; i++)
      {
        Serial.print(data.ch[i]);
        Serial.print("\t");
      }
      send();
    }
  }
}

boolean fetch_and_read()
{
  if (sbus_rx.Read())
  {
    data = sbus_rx.data();
    if (!is_mounted && data.ch[ARM_CH] >= ARM_TRUE - 100)
    {
      is_auto = false;
    }
    return true;
  }
  return false;
}

void send()
{
  if (is_auto)
  {
    if (!is_released)
    {
      data.ch[THR_CH] = thr_val;
    }
    data.ch[ARM_CH] = arm_val;
    data.ch[MODE_CH] = mode_val;
  }
  sbus_tx.data(data);
  sbus_tx.Write();
}


void sendMission(float lat_wp, float lon_wp, float altitude, float delay_seconds, uint8_t servo_channel, uint16_t servo_value)
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len;

  // 1. Wysłanie wiadomości MISSION_COUNT z liczbą punktów w misji
  uint16_t mission_count = 5; // Start + waypoint + serwo + delay + lądowanie
  mavlink_msg_mission_count_pack(
      system_id,
      component_id,
      &msg,
      target_system,
      target_component,
      mission_count,
      0, // target_component (plan)
      0  // mission_type (standard mission)
  );
  len = mavlink_msg_to_send_buffer(buf, &msg);
  mySerial.write(buf, len);
  Serial.println("MISSION_COUNT wysłano");

  delay(1000); // Poczekaj chwilę, aby kontroler lotu przetworzył wiadomość


  // 2. Wysłanie punktu startowego (pozycja 0)
  mavlink_msg_mission_item_int_pack(
      system_id,
      component_id,
      &msg,
      target_system,
      target_component,
      0,                                 // Seq
      MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, // Frame
      MAV_CMD_NAV_WAYPOINT,              // Command
      1,                                 // Current (1 oznacza, że to pierwszy punkt)
      1,                                 // Autocontinue
      0.0,                               // Param1
      0.0,                               // Param2
      0.0,                               // Param3
      0.0,                               // Param4
      (int32_t)(lat_wp * 1e7),           // X (szerokość geograficzna)
      (int32_t)(lon_wp * 1e7),           // Y (długość geograficzna)
      altitude,                          // Z (wysokość w metrach)
      MAV_MISSION_TYPE_MISSION           // mission_type
  );
  len = mavlink_msg_to_send_buffer(buf, &msg);
  mySerial.write(buf, len);
  Serial.println("Punkt startowy wysłano");

  delay(1000);


  // 4. Ustawienie serwa (pozycja 2)
  mavlink_msg_mission_item_int_pack(
      system_id,
      component_id,
      &msg,
      target_system,
      target_component,
      1,                                 // Seq
      MAV_FRAME_MISSION,                 // Frame
      MAV_CMD_DO_SET_SERVO,              // Command
      0,                                 // Current
      1,                                 // Autocontinue
      servo_channel,                     // Param1 (kanał serwa)
      servo_value,                       // Param2 (wartość PWM dla serwa)
      0.0,                               // Param3
      0.0,                               // Param4
      0,                                 // X (nieużywane)
      0,                                 // Y (nieużywane)
      0.0,                               // Z (nieużywane)
      MAV_MISSION_TYPE_MISSION           // mission_type
  );
  len = mavlink_msg_to_send_buffer(buf, &msg);
  mySerial.write(buf, len);
  Serial.println("Ustawienie serwa wysłano");

  delay(1000);


  // 3. Wysłanie waypointa (pozycja 1)
  mavlink_msg_mission_item_int_pack(
      system_id,
      component_id,
      &msg,
      target_system,
      target_component,
      2,                                 // Seq
      MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, // Frame
      MAV_CMD_NAV_WAYPOINT,              // Command
      0,                                 // Current
      1,                                 // Autocontinue
      0.0,                               // Param1
      0.0,                               // Param2
      0.0,                               // Param3
      0.0,                               // Param4
      (int32_t)(lat_wp * 1e7),           // X
      (int32_t)(lon_wp * 1e7),           // Y
      altitude,                          // Z
      MAV_MISSION_TYPE_MISSION           // mission_type
  );
  len = mavlink_msg_to_send_buffer(buf, &msg);
  mySerial.write(buf, len);
  Serial.println("Waypoint wysłano");

  delay(1000);

  // 5. Wysłanie komendy opóźnienia (Delay) (pozycja 3)
  mavlink_msg_mission_item_int_pack(
      system_id,
      component_id,
      &msg,
      target_system,
      target_component,
      3,                       // Seq
      MAV_FRAME_MISSION,       // Frame
      MAV_CMD_CONDITION_DELAY, // Command
      0,                       // Current
      1,                       // Autocontinue
      delay_seconds,           // Param1 (czas opóźnienia w sekundach)
      0.0,                     // Param2
      0.0,                     // Param3
      0.0,                     // Param4
      0,                       // X (nieużywane)
      0,                       // Y (nieużywane)
      0.0,                     // Z (nieużywane)
      MAV_MISSION_TYPE_MISSION // mission_type
  );
  len = mavlink_msg_to_send_buffer(buf, &msg);
  mySerial.write(buf, len);
  Serial.println("Opóźnienie wysłano");

  delay(1000);

  // 6. Wysłanie punktu lądowania (pozycja 4)
  mavlink_msg_mission_item_int_pack(
      system_id,
      component_id,
      &msg,
      target_system,
      target_component,
      4,                                 // Seq
      MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, // Frame
      MAV_CMD_NAV_LAND,                  // Command
      0,                                 // Current
      1,                                 // Autocontinue
      0.0,                               // Param1
      0.0,                               // Param2
      0.0,                               // Param3
      0.0,                               // Param4
      (int32_t)(lat_wp * 1e7),           // X
      (int32_t)(lon_wp * 1e7),           // Y
      0.0,                               // Z (0 dla lądowania)
      MAV_MISSION_TYPE_MISSION           // mission_type
  );
  len = mavlink_msg_to_send_buffer(buf, &msg);
  mySerial.write(buf, len);
  Serial.println("Lądowanie wysłano");

  delay(1000);
}
