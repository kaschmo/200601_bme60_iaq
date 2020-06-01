# IAQ Sensor based on BME680
ESP8266 based sensor station with Bosch BME680. Calculating IAQ Values using the BSEC library.

## Functionality
Reads Temperature, Humidity, Pressure and Gas Resistance values from BME680 via SPI bus.
Uses BSEC Arduino library to calculate Indoor Air Quality (IAQ)
Sends values via MQTT
OTA support for Wifi updates.
The code is mostly based on the example script from BSEC library, adding Wifi, OTA and MQTT functionality around.

## Arduino/VS Code Setup
The environment is a bit complicated to set up to make the BSEC Run.
Otherwise simple usage
- create a wifi_credials.h file with your ssid, pwd in there
- change config.h to your needs

### Summary BSEC Integration
- Use Arduino 1.8.11 (not the latest version!)
- Add BSEC Arduino library via library manager
- Do not copy around any files or stuff as other tutorials state
- Change platform.txt (see below)
- Change eagle.app.v6.common.ld (see below)
- Run some of the BSEC lib example codes as trial

#### Platform.txt Changes
- Can be found on MAC in Macintosh HD⁩ ▸ ⁨Users⁩ ▸ XXX ▸ ⁨Library⁩ ▸ ⁨Arduino15⁩ ▸ ⁨packages⁩ ▸ ⁨esp8266⁩ ▸ ⁨hardware⁩ ▸ ⁨esp8266⁩ ▸ ⁨2.3.0⁩ . Folder is not visible by default.
- Perform the 2 changes to the file according to step 3 in  https://github.com/BoschSensortec/BSEC-Arduino-library
- Do not copy the whole recipe string, but add "{compiler.libraries.ldflags}" to the "-- startgroup option"

#### eagle.app.v6.common.ld Changes
- Can be found on MAC in Macintosh HD⁩ ▸ ⁨Users⁩ ▸ XXX ▸ ⁨Library⁩ ▸ ⁨Arduino15⁩ ▸ ⁨packages⁩ ▸ ⁨esp8266⁩ ▸ ⁨hardware⁩ ▸ ⁨esp8266⁩ ▸ ⁨2.3.0⁩ ▸ ⁨tools⁩ ▸ ⁨sdk⁩ ▸ ⁨ld⁩
- Add the "libalgosec" entry according to step 4 in BSEC description  https://github.com/BoschSensortec/BSEC-Arduino-library

### Wiring BME680 breakout
- Using the BlueDot BME680 Breakout Board
- Using Wire() via SPI bus. this defaults to D1, D2 pins on NodeMCU 
BME680 SDI -> D2
BME680 SCK -> D1
BME680 SDO -> GND. This puts the sensor on address 0x76