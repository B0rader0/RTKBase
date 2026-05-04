# RTK Base
RTK Base is inspired by the ESP32 XBee official firmware for the Ardusimple [WiFi NTRIP Master](https://www.ardusimple.com/product/wifi-ntrip-master/) ESP32 XBee device [[3D model]](https://github.com/nebkat/esp32-xbee/blob/master/esp32-xbee-board.step), made with [ESP-IDF](https://github.com/espressif/esp-idf). 

RTK Base is implemented on a esp32 to the UART of which a UM980 GNSS module is connected.
The base station is intended to transfer the GNSS data (NMEA and/or RTCM) over WiFi to different clients.

## Features
- WiFi Station - AP mode.
- WiFi Hotspot - STA mode.
- Web Interface for configuring the base.
- UART configuration for the attached UM980
- TCP socket Server - intended to serve clients like UPrecise.
- NTRIP Caster for serving up to 4 rovers.
- Two NTRIP clients for connecting ot internet casters such as rtk2go.com.
- Command console for sending commands to the UM980 and monitor its data stream.

## Help

To be implemented


## Pinout

To be implemented
