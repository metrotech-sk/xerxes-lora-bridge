#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>  
#include "SSD1306.h" 
#include "driver/uart.h"
#include "esp_log.h"

#include "Master.hpp"
#include "Protocol.hpp"
#include "esp32/EspUart.hpp"
#include <string>
#include <sstream>
#include <vector>

#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     14   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define BAND    868E6

constexpr uint8_t _pin_tx = 13;
constexpr uint8_t _pin_rx = 15;
constexpr uint8_t _pin_tx_en = 2;

auto xn = Xerxes::EspUart(_pin_tx, _pin_rx, _pin_tx_en);
auto xp = Xerxes::Protocol(&xn);

SSD1306 display(0x3c, 4, 15);
String packSize = "--";

static uint rcvd = 0;
constexpr char TAG[] = "main";


void onRxInt(int packSize)
{
    rcvd = packSize;
}

std::vector<uint8_t> receive(int packetSize) {
    std::string pkt;
    std::vector<uint8_t> bytes;

    int rssi = LoRa.packetRssi();

    ESP_LOGD(TAG, "Receiving packet w/ size: %d", packetSize);
    for (int i = 0; i < packetSize; i++) 
    { 
        auto next_char = LoRa.read();
        bytes.push_back((uint8_t)next_char);
        pkt += (char)next_char;
    }
    ESP_LOGI(TAG, "Received packet '%s'", pkt.c_str());

    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_10);
    std::stringstream ss;
    ss << "RSSI " << rssi << "dBm";
    display.drawString(0, 0, ss.str().c_str());
    ss = std::stringstream();
    ss << "Rcvd: " << packetSize << " bytes";
    display.drawString(0, 12, ss.str().c_str());
    display.drawStringMaxWidth(0 , 24 , 128, pkt.c_str());
    display.display();
    ESP_LOGD(TAG, "RSSI %d dBm.", rssi);

    return bytes;
}

void initLora(LoRaClass &_LoRa)
{
    ESP_LOGD(TAG, "LoRa Receiver init");
    SPI.begin(SCK, MISO, MOSI, SS);
    _LoRa.setPins(SS, RST, DI0);  
    if (!_LoRa.begin(868E6)) {
        ESP_LOGE(TAG, "Starting LoRa failed!");
        esp_restart();
    }

    _LoRa.onReceive(onRxInt);
    _LoRa.disableInvertIQ(); 
    _LoRa.receive();    
    _LoRa.setSpreadingFactor(7); // spreading factor SF7
    _LoRa.setSignalBandwidth(125e3); // frequency bandwidth 125kHz
    _LoRa.setCodingRate4(5); // coding factor 4:5

    _LoRa.setTxPower(20);
    _LoRa.enableCrc();

    ESP_LOGD(TAG, "LoRa Receiver init done");
}

void setup() {
    Serial.begin(921600);

    pinMode(OLED_RST, OUTPUT);
    digitalWrite(OLED_RST, LOW);    // set GPIO16 low to reset OLED
    delay(50); 
    digitalWrite(OLED_RST, HIGH); // while OLED is running, must set GPIO16 in high、
    
    initLora(LoRa);

    display.init();
    //display.flipScreenVertically();  
    display.setFont(ArialMT_Plain_10);
    display.drawString(0 , 0 , "Listening");
    display.display();

    ESP_LOGD(TAG, "Setting wakeups.");
    gpio_wakeup_enable(static_cast<gpio_num_t>(LORA_IRQ), GPIO_INTR_HIGH_LEVEL);
    esp_sleep_enable_gpio_wakeup();

    ESP_LOGD(TAG, "Setting wakeups done. Setup done.");
}

void sleep_ms(uint ms)
{
    esp_sleep_enable_timer_wakeup(ms * 1000);
    esp_light_sleep_start();
}

void loop() {
    if(rcvd)
    {
        auto bytes = receive(rcvd);
        ESP_LOGD(TAG, "Sending packet to Xerxes.");
        auto time_start = esp_timer_get_time();
        xp.sendMessage(Xerxes::Message(Xerxes::Packet(bytes)));
        auto time_end = esp_timer_get_time();
        rcvd = 0;
        ESP_LOGD(TAG, "Sending packet to Xerxes done. Took %lld us.", time_end - time_start);
    }

    // wait for Serial to send data
    Serial.flush(1);
    sleep_ms(10);
    
}