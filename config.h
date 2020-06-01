//Configuration File for BME680 project
int timer_update_state = 30000; //update status via MQTT every 30 sec

//MQTT Topics
const char* inTopic = "cmnd/room_sensor_iaq/#";
const char* outTopic = "stat/room_sensor_iaq/";
const char* mqtt_id = "room_sensor_iaq";

//---------BSEC BME Initiatlization----------
/* Configure the BSEC library with information about the sensor
    18v/33v = Voltage at Vdd. 1.8V or 3.3V
    3s/300s = BSEC operating mode, BSEC_SAMPLE_RATE_LP or BSEC_SAMPLE_RATE_ULP
    4d/28d = Operating age of the sensor in days
*/
