#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>  
#include "SSD1306.h"

#include <iostream>
#include <stdio.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_sleep.h"

#define BAND    868E6


unsigned int counter = 0;
static uint32_t tStart = 0;

SSD1306 display(0x3c, 4, 15);

static String rcvdMsg("");
static bool rcvd = 0;

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

void LoRa_rxMode(){
    LoRa.disableInvertIQ();               // normal mode
    LoRa.receive();                       // set receive mode
}

void LoRa_txMode(){
    LoRa.idle();                          // set standby mode
    LoRa.enableInvertIQ();                // active invert I and Q signals
}

void LoRa_sendMessage(String message) {
    LoRa_txMode();                        // set tx mode
    LoRa.beginPacket();                   // start packet
    LoRa.print(message);                  // add payload
    LoRa.endPacket(true);                 // finish packet and send it
}

void onReceive(int packetSize) {
    digitalWrite(LED_BUILTIN, HIGH);
    rcvdMsg = "";
    while (LoRa.available()) {
        rcvdMsg += (char)LoRa.read();
    }
    rcvd = 1;
    digitalWrite(LED_BUILTIN, LOW);
}

void onTxDone() {
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    LoRa_rxMode();   
    //LoRa.idle(); 
}

void setup() {
    setCpuFrequencyMhz(240); // set CPU core to 240MHz
    Serial.begin(115200);
    Serial.setTimeout(300);  // set timeout for serial reading 300ms
    vTaskDelay(100 / portTICK_PERIOD_MS);  // wait for serial monitor to open
    
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
    
    LoRa.onReceive(onReceive);
    LoRa.onTxDone(onTxDone);
    LoRa_rxMode();
    ESP_LOGD(TAG, "LoRa config - rx mode");

    gpio_wakeup_enable(static_cast<gpio_num_t>(LORA_IRQ), GPIO_INTR_HIGH_LEVEL); 
    esp_sleep_enable_timer_wakeup(1e6); // sleep for 1s max
    esp_sleep_enable_gpio_wakeup();
    
    ESP_LOGD(TAG, "Setup done");
}


void loop() {

    display.displayOn();

    char buf[4];
    esp_fill_random(&buf, sizeof(buf));
    auto rcvd = String(counter++) + " - " + String(millis());
    if(Serial.available())
    {
        rcvd = String(Serial.readString());
    }

    message(String("Sending packet: "), 0, 0, true);
    message(String(rcvd), 0, 16, false); 

    // send packet
    tStart = millis();
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    LoRa.beginPacket();
    for(auto c:rcvd)
    {
        LoRa.write(c);
    }
    //LoRa.print(rcvd);
    LoRa.endPacket(true);
    
    ESP_LOGD(TAG, "Packet sent: %s", rcvd.c_str());

    esp_light_sleep_start();

    //after wake up from txDone evt:
    ESP_LOGD(TAG, "Packet sent in %d ms", millis()-tStart);
    message("Done!", 0, 32, false); 
    esp_light_sleep_start(); // sleep for another second

    
    //esp_deep_sleep(3e6);
}