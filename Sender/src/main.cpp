#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>  
#include "SSD1306.h"

#include <iostream>
#include <stdio.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include <string>
#include "Message.hpp"
#include "MessageId.h"
#include "Packet.hpp"

constexpr double BAND = 868E6;


SSD1306 display(0x3c, 4, 15);

static uint32_t ctr = 0;
static Xerxes::Packet rcvdPacket;

constexpr char TAG[] = "main";

void message(String text, uint x=0, uint y=0, bool clear=false)
{
    if(clear)
    {
        display.clear();
    }
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_10);

    display.drawString(x, y, text);
    display.display();
}

void LoRa_rxMode()
{
    LoRa.disableInvertIQ();               // normal mode
    LoRa.receive();                       // set receive mode
}

void LoRa_txMode()
{
    LoRa.idle();                          // set standby mode
    LoRa.enableInvertIQ();                // active invert I and Q signals
}

void onReceive(int packetSize) 
{
    digitalWrite(25, HIGH);
    std::vector<uint8_t> data;

    while (LoRa.available()) {
        uint8_t next = LoRa.read();
        data.push_back(next);
    }
    rcvdPacket.setData(data);
    digitalWrite(25, LOW);

    auto rcvdStr = rcvdPacket.toString();
    if(rcvdPacket.isValidPacket())
    {
        if(packetIsValidMessage(rcvdPacket))
        {
            ESP_LOGD(TAG, "Valid message received: %s", rcvdStr.c_str());
            for(auto const & c: rcvdPacket.getData())
            {
                Serial.write(c);
            }
        }
        else
        {
            ESP_LOGW(TAG, "Invalid message received: %s", rcvdStr.c_str());
        }
    }
    else
    {
        ESP_LOGW(TAG, "Invalid packet received: %s", rcvdStr.c_str());
    }

    Serial.flush(1);
}

void onTxDone() 
{
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    LoRa_rxMode();  // go back to receive mode 
    //LoRa.idle(); 
}

void sendPacketOverLoRa(Xerxes::Packet & packet)
{
    if(packet.isValidPacket())
    {
        ESP_LOGD(TAG, "Valid packet received: %s", packet.toString().c_str());
        if(packetIsValidMessage(packet))
        {
            display.clear();
            message(String("Sending packet: ") + String(ctr++), 0, 0, false); 
            ESP_LOGD(TAG, "Packet is valid message. Sending over LoRa.");

            // wait while LoRa is transmitting last message
            while(!LoRa.beginPacket());

            auto tx_start = esp_timer_get_time();
            for(const auto & c : packet.getData())
            {
                LoRa.write(c);
            }
            LoRa.endPacket(true);

            // sleep until tx is done
            esp_light_sleep_start();
            // awaken by rxDone event here
            ESP_LOGD(TAG, "Packet sent in %d ms, current time: %lldms", (esp_timer_get_time() - tx_start)/1000, esp_timer_get_time()/1000);                
            //after wake up from txDone evt:
            message("Done!", 0, 32, false); 
            Serial.flush(1);
        }
        else
        {
            ESP_LOGW(TAG, "Invalid message received: %s", packet.toString().c_str());
        }
    }
    else
    {
        ESP_LOGW(TAG, "Invalid packet received: %s", packet.toString().c_str());
    }
}

bool send = false;
void onBtnPressed() 
{
    ESP_LOGD(TAG, "Button pressed");    
    
    Xerxes::Packet packet = Xerxes::Message(0xFE, 0xFF, MSGID_SYNC).toPacket();
    sendPacketOverLoRa(packet);
}


void setup() {
    Serial.begin(921600);
    Serial.setTimeout(30);  // set timeout for serial reading 30ms
    Serial.flush();
    
    ESP_LOGI(TAG, "CPU Frequency: %d MHz", getCpuFrequencyMhz());
    
    pinMode(OLED_RST,OUTPUT);
    pinMode(LED_BUILTIN,OUTPUT);
    
    digitalWrite(OLED_RST, LOW);    // set GPIO16 low to reset OLED
    delay(50); 
    digitalWrite(OLED_RST, HIGH); // while OLED is running, must set GPIO16 in high
    ESP_LOGD(TAG, "OLED reset");
    
    display.init();
    // display.flipScreenVertically();  
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, "Device started");
    ESP_LOGD(TAG, "OLED init");
    
    SPI.begin(LORA_SCK,LORA_MISO,LORA_MOSI,LORA_CS);
    LoRa.setPins(LORA_CS,LORA_RST,LORA_IRQ);

    if (!LoRa.begin(BAND))
    {
        ESP_LOGE(TAG, "Starting LoRa failed!");
        esp_restart();
    }
    ESP_LOGD(TAG, "LoRa init");

    LoRa.setSpreadingFactor(7); // spreading factor SF7
    LoRa.setSignalBandwidth(125e3); // frequency bandwidth 125kHz
    LoRa.setCodingRate4(5); // coding factor 4:5
    // this results in a -127.5 dBm sensitivity
    // and a 5.47 kbps data rate

    LoRa.setTxPower(20);
    LoRa.enableCrc();
    ESP_LOGD(TAG, "LoRa config: SF7, BW125kHz, CR4/5, 20dBm, CRC, Freq: %f", BAND);
    
    LoRa.onReceive(onReceive);
    LoRa.onTxDone(onTxDone);
    LoRa_rxMode();
    ESP_LOGD(TAG, "LoRa config - rx mode");

    gpio_wakeup_enable(static_cast<gpio_num_t>(LORA_IRQ), GPIO_INTR_HIGH_LEVEL); 
    attachInterrupt(digitalPinToInterrupt(0), onBtnPressed, RISING);
    esp_sleep_enable_gpio_wakeup();
    
    ESP_LOGD(TAG, "Setup done");
    message("READY!", 0, 0, true);
    Serial.flush(1);
}


void sleep_ms(uint32_t ms)
{
    esp_sleep_enable_timer_wakeup(ms * 1000);
    esp_light_sleep_start();
}


Xerxes::Packet readPacketFromSerial()
{
    // start with SOH in buffer
    std::vector<uint8_t> rawMsg = {0x01};
    // wait for SOH in serial
    while(Serial.read() != 0x01);
    ESP_LOGD(TAG, "SOH received");
    // read xerxes message from serial
    auto len = Serial.read();
    rawMsg.push_back(len);
    ESP_LOGD(TAG, "Reading %d bytes from serial", len);
    for(int i = 0; i < len - 2; i++)
    {
        rawMsg.push_back(Serial.read());
    }
    auto rcvd = Xerxes::Packet();
    rcvd.setData(rawMsg);
    return rcvd;
}


void loop() {
    Xerxes::Packet packet = readPacketFromSerial();
    sendPacketOverLoRa(packet);    
}