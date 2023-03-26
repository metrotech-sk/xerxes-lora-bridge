#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>  
#include "SSD1306.h"
#include "esp_task_wdt.h"

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

// I2C OLED Display works with SSD1306 driver
#ifdef OLED_SDA
#undef OLED_SDA
#endif
#define OLED_SDA 21

#ifdef OLED_SCL
#undef OLED_SCL
#endif
#define OLED_SCL 22

#ifdef OLED_RST
#undef OLED_RST
#endif
#define OLED_RST 16

// SPI LoRa Radio
#define SCK 5       // GPIO5 - SX1276 SCK
#define MISO 19     // GPIO19 - SX1276 MISO
#define MOSI 27    // GPIO27 - SX1276 MOSI
#define CS 18     // GPIO18 - SX1276 CS
#define RST 23   // GPIO14 - SX1276 RST
#define IRQ 26  // GPIO26 - SX1276 IRQ (interrupt request)


constexpr double BAND = 868E6;
#define PIN_LED 2

SSD1306 display(0x3c, OLED_SDA, OLED_SCL);

static uint32_t ctr = 0;
static Xerxes::Packet rxPacket;
static Xerxes::Packet txPacket;
static bool serialReceived = false;
static bool btnPressed = false;
static bool receivedLora = false;

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
    digitalWrite(PIN_LED, HIGH);
    std::vector<uint8_t> data;

    while (LoRa.available()) {
        uint8_t next = LoRa.read();
        data.push_back(next);
    }
    rxPacket.setData(data);
    digitalWrite(PIN_LED, LOW);
    receivedLora = true;
}

void handleReceivedPacket()
{
    if(rxPacket.isValidPacket())
    {
        if(packetIsValidMessage(rxPacket))
        {
            ESP_LOGD(TAG, "Valid message received: %s", rxPacket.toString().c_str());
            for(auto const & c: rxPacket.getData())
            {
                Serial.write(c);
            }
        }
        else
        {
            ESP_LOGW(TAG, "Invalid message received: %s", rxPacket.toString().c_str());
        }
    }
    else
    {
        ESP_LOGW(TAG, "Invalid packet received: %s", rxPacket.toString().c_str());
    }
    Serial.flush(1);
}

void onTxDone() 
{
    digitalWrite(PIN_LED, LOW);    // turn the LED off by making the voltage LOW
    LoRa_rxMode();  // go back to receive mode 
    //LoRa.idle(); 
}


void sendPacketOverLoRaUnsafe(Xerxes::Packet & packet, bool updateDisplay = true)
{
    if(updateDisplay)
    {   
        ESP_LOGD(TAG, "Updating display");
        message(String("Sending packet: ") + String(ctr++), 0, 0, true); 
    }

    ESP_LOGD(TAG, "Enabling radio");
    LoRa_txMode();
    digitalWrite(PIN_LED, HIGH);   // turn the LED on (HIGH is the voltage level)
    LoRa.beginPacket();
    for(const auto & c : packet.getData())
    {
        LoRa.write(c);
    }
    LoRa.endPacket(true);
    LoRa_rxMode();
    ESP_LOGD(TAG, "Packet sent.");
}


void sendPacketOverLoRaSafe(Xerxes::Packet & packet)
{
    if(packet.isValidPacket())
    {
        ESP_LOGD(TAG, "Valid packet received: %s", packet.toString().c_str());
        if(packetIsValidMessage(packet))
        {
            ESP_LOGD(TAG, "Packet is valid message. Sending over LoRa.");
            sendPacketOverLoRaUnsafe(packet);
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


void onBtnPressed() 
{
    btnPressed = true;
}


void setup() {
    Serial.begin(921600);
    Serial.setTimeout(1);  // set timeout for serial reading 1ms
    Serial.flush();
    
    ESP_LOGI(TAG, "CPU Frequency: %d MHz", getCpuFrequencyMhz());
    
    pinMode(OLED_RST,OUTPUT);
    pinMode(LED_BUILTIN,OUTPUT);
    
    digitalWrite(OLED_RST, LOW);    // set GPIO16 low to reset OLED
    delay(50); 
    digitalWrite(OLED_RST, HIGH); // while OLED is running, must set GPIO16 in high
    ESP_LOGD(TAG, "OLED reset");
    
    display.init();
    display.flipScreenVertically();  
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, "Device started");
    ESP_LOGD(TAG, "OLED init");
    
    SPI.begin(SCK, MISO, MOSI, CS);
    LoRa.setPins(CS, RST, IRQ);

    if (!LoRa.begin(BAND))
    {
        ESP_LOGE(TAG, "Starting LoRa failed!");
        esp_restart();
    }
    ESP_LOGD(TAG, "LoRa init");

    LoRa.setSpreadingFactor(7); // spreading factor SF7
    LoRa.setSignalBandwidth(LORA_BW_KHZ * 1000); // frequency bandwidth 125kHz
    LoRa.setCodingRate4(5); // coding factor 4:5
    // this results in a -127.5 dBm sensitivity
    // and a 5.47 kbps data rate

    LoRa.setTxPower(20);
    LoRa.enableCrc();
    ESP_LOGD(TAG, "LoRa config: SF7, BW%dkHz, CR4/5, 20dBm, CRC, Freq: %f", LORA_BW_KHZ, BAND);
    
    LoRa.onReceive(onReceive);
    LoRa.onTxDone(onTxDone);
    LoRa_rxMode();
    ESP_LOGD(TAG, "LoRa config - rx mode");

    gpio_wakeup_enable(static_cast<gpio_num_t>(LORA_IRQ), GPIO_INTR_HIGH_LEVEL); 
    attachInterrupt(digitalPinToInterrupt(0), onBtnPressed, RISING);
    esp_sleep_enable_gpio_wakeup();
    
    ESP_LOGD(TAG, "Setup done");
    message("READY!", 0, 0, true);
}


void sleep_ms(uint32_t ms)
{
    auto timeNow = esp_timer_get_time();
    Serial.flush();
    auto restOfSleep = ms * 1000 - (esp_timer_get_time() - timeNow);
    esp_sleep_enable_timer_wakeup(restOfSleep);
    esp_light_sleep_start();
}


bool readPacketFromSerial(uint32_t timeoutUs = 5000)
{
    auto startUs = esp_timer_get_time();

    // start with SOH in buffer
    std::vector<uint8_t> rawMsg = {0x01};
    // wait for SOH in serial
    while(Serial.read() != 0x01)
    {
        if(esp_timer_get_time() - startUs > timeoutUs)
        {
            return false;
        }
    }
    ESP_LOGD(TAG, "SOH received");
    // read xerxes message from serial
    auto len = Serial.read();
    rawMsg.push_back(len);
    ESP_LOGD(TAG, "Reading %d bytes from serial", len);
    for(int i = 0; i < len - 2; i++)
    {
        rawMsg.push_back(Serial.read());
        if(esp_timer_get_time() - startUs > timeoutUs)
        {
            return false;
        }
    }

    txPacket.setData(rawMsg);
    return txPacket.isValidPacket();
}


void loop() {
    
    serialReceived = readPacketFromSerial();

    if(btnPressed)
    {
        btnPressed = false;
        ESP_LOGD(TAG, "Button pressed");    
        Xerxes::Packet syncPacket = Xerxes::Message(0xFE, 0xFF, MSGID_SYNC).toPacket();
        ESP_LOGD(TAG, "Sending packet: %s", syncPacket.toString().c_str());
        sendPacketOverLoRaUnsafe(syncPacket);
    }
    
    if(serialReceived)
    {
        serialReceived = false;
        if(packetIsValidMessage(txPacket))
        {
            ESP_LOGD(TAG, "Valid message received from serial: %s", txPacket.toString().c_str());
            sendPacketOverLoRaUnsafe(txPacket);
        }
        else
        {
            ESP_LOGW(TAG, "Invalid message received from serial: %s", txPacket.toString().c_str());
        }
    }

    if(receivedLora)
    {
        receivedLora = false;
        handleReceivedPacket();
        message("Reply from:", 0, 12, false);
        if(rxPacket.isValidPacket() && packetIsValidMessage(rxPacket))
        {
            Xerxes::Message rxMessage = Xerxes::Message(rxPacket);
            message(String(rxMessage.srcAddr), 0, 24, false);
        }
    }
}