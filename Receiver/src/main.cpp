#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>  
#include "SSD1306.h" 
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_assert.h"

#include "Master.hpp"
#include "Protocol.hpp"
#include "esp32/EspUart.hpp"
#include <string>
#include <sstream>
#include <vector>
#include "Version.h"
#include "Leaf.hpp"

#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     14   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define LED     25   // GPIO25 -- LED
#define BAND    868E6

constexpr uint8_t _pin_tx = 13;
constexpr uint8_t _pin_rx = 15;
constexpr uint8_t _pin_tx_en = 2;

auto xn = Xerxes::EspUart(_pin_tx, _pin_rx, _pin_tx_en);
auto xp = Xerxes::Protocol(&xn);
auto xm = Xerxes::Master(&xp, 0xFE, 10000);
Xerxes::Leaf leaf;

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
    display.setFont(ArialMT_Plain_24);
    std::stringstream ss;
    ss << "S: " << rssi << "dB";
    display.drawString(0, 0, ss.str().c_str());
    //ss = std::stringstream();
    //ss << "Rcvd: " << packetSize << " bytes";
    //display.drawString(0, 12, ss.str().c_str());
    //display.drawStringMaxWidth(0 , 24 , 128, pkt.c_str());
    display.display();
    ESP_LOGD(TAG, "RSSI %d dBm.", rssi);

    return bytes;
}


void LoRa_rxMode(){
    LoRa.disableInvertIQ();               // normal mode
    LoRa.receive();                       // set receive mode
}


void onTxDone() {
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    LoRa_rxMode();   
    //LoRa.idle(); 
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
    _LoRa.onTxDone(onTxDone);
    _LoRa.disableInvertIQ(); 
    _LoRa.receive();    
    _LoRa.setSpreadingFactor(7); // spreading factor SF7
    _LoRa.setSignalBandwidth(LORA_BW_KHZ*1000); // frequency bandwidth 125kHz
    _LoRa.setCodingRate4(5); // coding factor 4:5

    _LoRa.setTxPower(20);
    _LoRa.enableCrc();

    ESP_LOGD(TAG, "LoRa Receiver init done");
}

void setup() {
    Serial.begin(921600);

    pinMode(LED, OUTPUT);
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

    ESP_LOGD(TAG, "Setting wakeups done. Searching for sensor.");
    
    for(int addr = 0; addr < 0x20; addr++)
    {
        try
        {
            ping_reply_t rply = xm.ping(addr);
            ESP_LOGI(TAG, "Found sensor at address 0x%02X.", addr);
            assert(rply.v_major == PROTOCOL_VERSION_MAJ && rply.v_minor == PROTOCOL_VERSION_MIN);
            leaf = Xerxes::Leaf(addr, &xm);
            break;
        }
        catch(const Xerxes::TimeoutError& e)
        {
            ESP_LOGD(TAG, "No sensor found.");
        }
        catch(const std::exception& e)
        {
            ESP_LOGE(TAG, "Error: %s", e.what());
        }
        catch(...)
        {
            ESP_LOGE(TAG, "Unknown error.");
        }
    }
}

void sleep_ms(uint ms)
{
    esp_sleep_enable_timer_wakeup(ms * 1000);
    esp_light_sleep_start();
}

void handleMessage(const Xerxes::Message & _msg)
{
    bool got_reply = false;
    ESP_LOGD(TAG, "DstAddr: %d", _msg.dstAddr);
    ESP_LOGD(TAG, "Leaf addr: %d", leaf.getAddr());

    if(_msg.dstAddr == leaf.getAddr() or _msg.dstAddr == BROADCAST_ADDRESS)
    {
        ESP_LOGD(TAG, "Forwarding packet to Xerxes leaf.");
        xp.sendMessage(_msg);
        
        Xerxes::Message replyFromSensor;
        got_reply = xp.readMessage(replyFromSensor, 10000);
        
        // mock reply
        ESP_LOGW(TAG, "Mocking reply from sensor.");
        got_reply = true;
        replyFromSensor = Xerxes::Message(0, 0xFE, MSGID_PING_REPLY, std::vector<uint8_t>{1, 4, 1});

        if(got_reply)
        {
            ESP_LOGI(TAG, "Got reply from sensor.");

            auto pkt = replyFromSensor.toPacket();

            ESP_LOGD(TAG, "Sending packet %s to gateway.", pkt.toString().c_str());
            LoRa.beginPacket();
            for(auto c:pkt.getData())
            {
                LoRa.write(c);
            }
            LoRa.endPacket(true);
        }
    }
}

void loop() 
{
    if(rcvd)
    {
        auto bytes_vec = receive(rcvd);
        auto time_start = esp_timer_get_time();

        auto packet = Xerxes::Packet::EmptyPacket();
        packet.setData(bytes_vec);
        if(packet.isValidPacket())
        {
            if(packetIsValidMessage(packet))
            {
                ESP_LOGD(TAG, "Got valid message.");
                handleMessage(Xerxes::Message(packet));
            }
            else
            {
                ESP_LOGI(TAG, "Got valid packet, but not a message.");
            }
        }
        else
        {
            ESP_LOGW(TAG, "Invalid packet received.");
            rcvd = 0;
        }
        
        auto time_end = esp_timer_get_time();
        ESP_LOGW(TAG, "Message handle done. Took %lld us.", time_end - time_start);
        rcvd = 0;

        // wait for Serial to send data
        Serial.flush(1);
    }
    sleep_ms(10);  // 1ms = 21mA, 10ms = 17mA, 100ms = 15mA
    
}