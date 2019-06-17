# Sensors_MQTT
ESP8266 with MHZ19, BME280, TM1627 and 1 Color LED sends data to MQTT server, display time, CO2 and temp.

Connections to Lolin (NodeMCU V3):
1. MHZ19
    Rx  --> D1
    Tx  --> D2
    Vin --> VU
2. BME280
    SCL --> D4
    SDA --> D5
3. TM1627
    CLK --> D7
    DIO --> D6
4. LED (1pc from ws2812 strip)
    DATA --> D3
