// This sketch sends simulated flight control data via ESP-NOW Long Range (LR).

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <math.h>
#include <freertos/FreeRTOS.h> 

// --- Configuration ---
const uint8_t broadcastAddress[] = {0xB4, 0x3A, 0x45, 0x89, 0x76, 0x88};
const long SEND_INTERVAL_MS = 15;

const u8_t throttlr_pin = 34;
const u8_t pich_pin = 32;
const u8_t roll_pin = 35;
const u8_t yaw_pin = 33;
const u8_t mode_pin = 25;

u16_t throttle = 0;
float pitch = 0;
float roll = 0;
float yaw = 0;
int16_t stick_output_throttle = 0;
int16_t stick_output_pich = 0;
int16_t stick_output_roll = 0;
int16_t stick_output_yaw = 0;
u32_t stick_Base_Value_Th = 0;
u32_t stick_Base_Value_pi = 0;
u32_t stick_Base_Value_ro = 0;
u32_t stick_Base_Value_ya = 0;

const float pitch_max = 40, pitch_min = -40;
const float roll_max = 30, roll_min = -30;
const float yaw_max = 10, yaw_min = -10;

typedef struct __attribute__((packed)) {
    uint32_t packet_id; 
    float roll;         
    float pitch;        
    float yaw;          
    uint16_t thrust;    
} flight_control_packet_t;

flight_control_packet_t s_control_data;

u16_t Rec_packet, battaryV;

typedef struct __attribute__((packed)) {
    uint8_t packet_id;     
    uint16_t packet_count_500ms; 
    uint16_t battery_mV;  
} telemetry_packet_t; 

telemetry_packet_t s_last_telemetry;

//void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {}


void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len){
    memcpy(&s_last_telemetry, incomingData, sizeof(s_last_telemetry));
    Rec_packet = s_last_telemetry.packet_count_500ms * 2;
    battaryV = s_last_telemetry.battery_mV;
}

void getstickvalue(){
    //stick_output_throttle = analogRead(throttlr_pin); }
    throttle = analogRead(throttlr_pin);
    stick_output_pich = analogRead(pich_pin);
    stick_output_roll = analogRead(roll_pin);
    stick_output_yaw = analogRead(yaw_pin);
}

void stickcalibration(){
    throttle = 0;
    pitch = 0;
    roll = 0;
    yaw = 0;
    int limit = 500;
    for (size_t i = 0; i < limit; i++)
    {
        getstickvalue();
        //stick_Base_Value_Th = stick_output_throttle + stick_Base_Value_Th;
        stick_Base_Value_pi = stick_output_pich + stick_Base_Value_pi;
        stick_Base_Value_ro = stick_output_roll + stick_Base_Value_ro;
        stick_Base_Value_ya = stick_output_yaw + stick_Base_Value_ya;
        delay(2);
    }

    //stick_Base_Value_Th = stick_Base_Value_Th / limit;
    stick_Base_Value_pi = stick_Base_Value_pi / limit;
    stick_Base_Value_ro = stick_Base_Value_ro / limit;
    stick_Base_Value_ya = stick_Base_Value_ya / limit;
    throttle = 0;
    pitch = 0;
    roll = 0;
    yaw = 0;
}


void getAngValue(){
    getstickvalue();
    //throttle = stick_output_throttle;
    stick_output_pich = stick_output_pich - stick_Base_Value_pi;
    stick_output_roll = stick_output_roll - stick_Base_Value_ro;
    //stick_output_throttle = ((stick_output_throttle >= 20) || (stick_output_throttle <= -20) ? stick_output_throttle: 0);
    stick_output_pich = ((stick_output_pich >= 15) || (stick_output_pich <= -15) ? stick_output_pich: 0);
    stick_output_roll = ((stick_output_roll >= 15) || (stick_output_roll <= -15) ? stick_output_roll: 0);
    //throttle = constrain((throttle + (stick_output_throttle >= 0 ? stick_output_throttle * 0.5 : stick_output_throttle * 4)), 0, 60000);
    pitch = constrain((stick_output_pich >= 0 ? (float)stick_output_pich * 0.02 : (float)stick_output_pich / 31.25), -40, 40);
    roll = constrain((stick_output_roll >= 0 ? (float)stick_output_roll * 0.02 : (float)stick_output_roll / 31.25), -40, 40);
}

void avgstickvalue(){

}

void SendTask(void *pvParameters) {
    Serial.println("SendTask started.");
    const TickType_t xDelay = pdMS_TO_TICKS(SEND_INTERVAL_MS);
    for (;;) {
        unsigned long currentTime = millis();
        getAngValue();
        s_control_data.packet_id++;
        //float time_sec = currentTime / 1000.0f;
        s_control_data.roll = roll;
        s_control_data.pitch = pitch;
        s_control_data.yaw = 0.00;
        s_control_data.thrust = throttle;
        esp_now_send(broadcastAddress, (uint8_t *)&s_control_data,sizeof(s_control_data));
        vTaskDelay(xDelay);
    }
    vTaskDelete(NULL);
}

void setupespnowlr(){
    throttle = 0;
    pitch = 0;
    roll = 0;
    yaw = 0;
    WiFi.mode(WIFI_STA);
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
    esp_now_init();
    //esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
    Serial.println("ESP-NOW initialized and broadcast peer added.");
}

// --- Arduino Setup ---
void setup() {
    throttle = 0;
    pitch = 0;
    roll = 0;
    yaw = 0;
    pinMode(2, OUTPUT);
    digitalWrite(2, LOW);
    Serial.begin(115200);
    delay(100);
    setupespnowlr();
    delay(20);
    stickcalibration();
    while(true){
        getstickvalue();
        if(throttle==0){
            digitalWrite(2, HIGH);
            break;
        }else{
            digitalWrite(2, LOW);
            delay(50);
            digitalWrite(2, HIGH);
            delay(50);
            digitalWrite(2, LOW);
            delay(50);
            digitalWrite(2, HIGH);
            delay(50);
            digitalWrite(2, LOW);
            delay(500);
        }
    }
    xTaskCreate(SendTask,"SendTask",4096,NULL,3,NULL);
}

// --- Arduino Loop ---
void loop() {
    //Serial.printf(" Reciver hz %u | Batt: %.2f V\n",Rec_packet,s_last_telemetry.battery_mV / 1000.0f);
    Serial.printf("throttle = %d | pitch= %.2f | Roll= %.2f\n",throttle,pitch,roll);
    vTaskDelay(500); 
} 