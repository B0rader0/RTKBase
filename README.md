# RTK Base
RTK Base is inspired by the ESP32 XBee official firmware for the Ardusimple [WiFi NTRIP Master](https://www.ardusimple.com/product/wifi-ntrip-master/).

RTK Base is implemented on an ESP32 and uses UM980 GNSS module connected to the UART of the ESP32.
The base station is intended to transfer NMEA and/or RTCM messages from UM980 over WiFi to different clients.
The firmware is written using ESP-IDF 5.x.

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
