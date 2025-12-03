// This sketch runs on an ESP32 and acts as the receiver for the ESP-NOW Long Range

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <math.h>

//ESP Now Setings
const uint8_t senderMacAddress[] = {0xCC,0xDB,0xA7,0x13,0x55,0xA8}; // The MAC of the Sender/TX
const long TELEMETRY_INTERVAL_MS = 500;


// Global variables to track statistics and time
uint32_t s_packets_received = 0;
unsigned long s_last_print_time = 0;
volatile uint16_t s_rx_counter_period = 0;

u16_t duty1;
u16_t duty2;
u16_t duty3;

const u8_t Disconnect_Pin = 15;
const u8_t Motor_pin = 1;
const u8_t Servo_Left_pin = 21;
const u8_t Servo_Right_Pin = 2;
const u8_t BATTERY_VOLTAGE_PIN = 0;

const u16_t Motor_PWM_friquency = 400;
const u16_t PWM_friquency = 50;
const u8_t PWM_bitRate = 12;

u32_t id;
u16_t input_thrust;
float input_roll, input_pitch, input_yaw;
float battery_voltage;
int rawADC;

typedef struct __attribute__((packed)) {
    uint32_t packet_id; 
    float roll;         
    float pitch;        
    float yaw;          
    uint16_t thrust;   
} flight_control_packet_t;

// --- Telemetry Data Structure (8 bytes total) ---
typedef struct __attribute__((packed)) {
    uint8_t packet_id;      
    uint16_t packet_count_500ms; 
    uint16_t battery_mV;    
} telemetry_packet_t; 

// Telemetry packet to be sent
telemetry_packet_t s_telemetry_data;
const float VOLTAGE_DIVIDER_RATIO = 4.65;

void SignalLost(){
  input_pitch = input_roll = input_yaw = 0.00;
  input_thrust = 0;
}


void bldcpwm(){
  duty1 = map(input_thrust, 0, 3000, 180, 420);
  duty2 = map(constrain((input_pitch + input_roll),-40,40), -40, 40, 180, 420);
  duty3 = map(constrain((input_roll - input_pitch), -40, 40), -40, 40, 180, 420);
  ledcWrite(Motor_pin, duty1);
  ledcWrite(Servo_Left_pin, duty2);
  ledcWrite(Servo_Right_Pin, duty3);
}

//void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len){
void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len) {
    s_packets_received++;

    flight_control_packet_t incoming_data;
    memcpy(&incoming_data, incomingData, sizeof(incoming_data));
    id = incoming_data.packet_id;
    input_thrust = incoming_data.thrust;
    input_thrust = (input_thrust >= 100 ? input_thrust : 0);
    input_pitch = incoming_data.pitch;
    input_roll = incoming_data.roll;
    input_yaw = incoming_data.yaw;
    bldcpwm();
}

void getBatteryVoltage() {
  rawADC = analogRead(BATTERY_VOLTAGE_PIN);
  float V_out = ((float)rawADC / 4095.0) * 3.3+0.4;
  battery_voltage = V_out *VOLTAGE_DIVIDER_RATIO;
}

void TelemetryTask(void *pvParameters) {
    Serial.println("TelemetryTask started.");
    const TickType_t xDelay = pdMS_TO_TICKS(TELEMETRY_INTERVAL_MS);
    for (;;) {
        getBatteryVoltage();
        s_telemetry_data.packet_id = 0;
        s_telemetry_data.packet_count_500ms =  s_packets_received;
        if(s_packets_received<=0){SignalLost();}
        s_telemetry_data.battery_mV = (uint16_t)(battery_voltage * 1000.0f);
        //Serial.printf("Battary voltage = %d\n",s_telemetry_data.battery_mV);
        s_packets_received = 0;
        esp_now_send(senderMacAddress, (uint8_t *)&s_telemetry_data, sizeof(s_telemetry_data));
        vTaskDelay(xDelay);   
    }
      vTaskDelete(NULL);
}

void setupespnowlr(){
    WiFi.mode(WIFI_STA);
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
    esp_now_init();
    esp_now_register_recv_cb(OnDataRecv);
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, senderMacAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
    Serial.println("ESP-NOW initialized. Waiting for control data...");
    xTaskCreate(TelemetryTask,"TelemetryTask",4096,NULL,2, NULL);
}

// --- Arduino Setup ---
void setup() {

  // Attach PWM channels to motors and servos
  ledcAttach(Motor_pin, Motor_PWM_friquency, PWM_bitRate); 
  ledcAttach(Servo_Left_pin, PWM_friquency, PWM_bitRate);
  ledcAttach(Servo_Right_Pin, PWM_friquency, PWM_bitRate);

  ledcWrite(Motor_pin, 180);
  ledcWrite(Servo_Left_pin, 300);
  ledcWrite(Servo_Right_Pin, 300);

  unsigned long currentMillis = millis();
  while (!Serial && (millis() - currentMillis) < 500) {
    delay(10);
  }

  Serial.begin(115200);
  delay(100);

  pinMode(Disconnect_Pin, OUTPUT);
  digitalWrite(Disconnect_Pin, HIGH);

  pinMode(WIFI_ENABLE, OUTPUT);
  digitalWrite(WIFI_ENABLE, LOW);

  delay(100);

  pinMode(WIFI_ANT_CONFIG, OUTPUT);
  digitalWrite(WIFI_ANT_CONFIG,HIGH);

  setupespnowlr();
}


void loop() {
  // The main loop is minimal.
  //Serial.printf("|-- OK --| thrust : %lu | Total Received: %u\n",thrust,s_packets_received);
  Serial.printf("|-- OK --| ID:%lu | R:%.1f, P:%.1f, Y:%.1f, T:%u | Total Received: %u\n", id,input_roll,input_pitch,input_yaw,input_thrust,s_telemetry_data.packet_count_500ms*2);
  vTaskDelay(700);
}