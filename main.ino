#include <stdint.h>
#include <math.h>
#include <WiFi.h>
#include <WiFiUdp.h> //for NTP
#include "NTPClient.h"
#include <Wire.h> //for I2C
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h> //for Adafruit_BME280
#include <Adafruit_BME280.h>


#if defined(ARDUINO) && ARDUINO >= 100 //wtf?
    #define printByte(args)  write(args);
#else
    #define printByte(args)  print(args,BYTE);
#endif

#define LCD_1 0x27
#define BME280_1 0x76
#define GY49 0x4a
#define ERROR_DELAY 1500
#define DEBUG_DELAY 1000
#define SDA 21
#define SCL 22
#define BRIGHT 25 //D25 on right which controls the bright of LCD
#define D32 32 //I want to use it to control LEDs
#define LED_BUILTIN 2

WiFiUDP ntpUDP;
NTPClient my_time_client(ntpUDP);
LiquidCrystal_I2C lcd_1(LCD_1,16,2);
Adafruit_BME280 bme280_1;

#define UPCIR 0
#define OHM 1
uint8_t upcir[8]={0x7,0x5,0x7,0x0,0x0,0x0,0x0};
uint8_t ohm[8]={0,0,0b1110,0b10001,0b10001,0b1010,0b11011};

void blink_led(uint16_t* intervals,uint8_t len){
    bool s_flag=HIGH;
    for(uint8_t i=0;i<len;i++){
        digitalWrite(LED_BUILTIN,s_flag);
        s_flag=(s_flag==HIGH)?LOW:HIGH;
        delay(intervals[i]);
    }
    digitalWrite(LED_BUILTIN,LOW);
}

uint8_t connect_wifi(char * ssid,char * password){
    //output status
    Serial.print("wifi connecting...");
    lcd_1.clear();
    lcd_1.print("wifi connecting...");
    //connect
    WiFi.begin(ssid,password);
    uint8_t ax=0;
    while(WiFi.status()!= WL_CONNECTED){
        delay(1000);
        Serial.print(".");
        if(ax==0){
            lcd_1.setCursor(0,1);
            lcd_1.print(".");
        }else if(ax<16){
            lcd_1.print(".");
        }else if(ax==16){
            lcd_1.setCursor(0,1);
            lcd_1.print("_");
        }else if(ax<32){
            lcd_1.print("_");
        }else{
            lcd_1.setCursor(0,1);
            lcd_1.print("connect failed");
            delay(1500);
            return 1;
        }
        ax++;
    }
    //output result
    Serial.println("\nwifi connected!");
    Serial.print("ip: ");
    Serial.println(WiFi.localIP());
    Serial.print("netmask: ");
    Serial.println(WiFi.subnetMask());
    Serial.print("gateway: ");
    Serial.println(WiFi.gatewayIP());
    Serial.print("channel: ");
    Serial.println(WiFi.channel());
    //WiFi.setAutoReconnect(true);
    Serial.print("auto-reconnect: ");
    Serial.println(WiFi.getAutoReconnect());
    lcd_1.clear();
    lcd_1.print("wifi connected! ");
    delay(DEBUG_DELAY);
    return 0;
}

inline void sync_time(){
    Serial.print("syncing...");
    lcd_1.clear();
    lcd_1.print("syncing...");

    if(WiFi.status()!=WL_CONNECTED){
        Serial.print("cannot sync because wifi is not ok");
        lcd_1.clear();
        lcd_1.print("sync fail: no wifi");
        delay(ERROR_DELAY);
        return;
    }

    my_time_client.begin();
    my_time_client.setTimeOffset(28800);
    my_time_client.setUpdateInterval(3600000);
    //my_time_client.setPoolServerName("0.cn.pool.ntp.org");
    my_time_client.setPoolServerName("ntp.tuna.tsinghua.edu.cn");
    my_time_client.setTimeout(400);

    uint8_t ax=0;
    while(!my_time_client.forceUpdate()){
        delay(2000);
        if(ax==0){
            lcd_1.setCursor(0,1);
            lcd_1.print(".");
        }else if(ax<16){
            lcd_1.print(".");
        }else if(ax==16){
            lcd_1.setCursor(0,1);
            lcd_1.print("_");
        }else if(ax<32){
            lcd_1.print("_");
        }else{
            lcd_1.setCursor(0,1);
            lcd_1.print("failed");
            delay(1500);
            return;
        }
        ax++;
    }

    Serial.print("\nsuccess: ");
    Serial.println(my_time_client.getFormattedTime());
    lcd_1.clear();
    lcd_1.print("synced: "+my_time_client.getFormattedTime());
    delay(DEBUG_DELAY);
}

inline void init_lcd(LiquidCrystal_I2C *plcd){
    (*plcd).init();
    delay(100);
    (*plcd).backlight();
    (*plcd).leftToRight();
    (*plcd).noAutoscroll();
    (*plcd).createChar(UPCIR,upcir);
    //(*plcd).createChar(OHM,ohm);
    delay(100);
    (*plcd).clear();
    (*plcd).print("lcd set. ");
    delay(DEBUG_DELAY);
}

inline void init_GY49(){
    Wire.beginTransmission(GY49);
    // Select configuration register
    Wire.write(0x02);
    // Continuous mode, Integration time = 800 ms
    Wire.write(0x40);
    // Stop I2C transmission
    Wire.endTransmission();
}

inline void init_bme280(){
    lcd_1.clear();
    if(bme280_1.begin(BME280_1)){
        Serial.print("found bme280 at ");
        Serial.println("0x"+String(BME280_1,HEX));
        lcd_1.print("found bme280 at ");
        lcd_1.setCursor(0,1);
        lcd_1.print("0x"+String(BME280_1,HEX));
        delay(DEBUG_DELAY);
    }else{
        Serial.print("lost bme280 at ");
        Serial.println("0x"+String(BME280_1,HEX));
        lcd_1.print("lost bme280 at ");
        lcd_1.setCursor(0,1);
        lcd_1.print("0x"+String(BME280_1,HEX));
        delay(ERROR_DELAY);
    }
}

uint16_t TEST_BLINK[]={100,100,100,100,100}; //used in setup

void setup(){
    Serial.begin(230400);
    Serial.println("serial inited");
    pinMode(LED_BUILTIN,OUTPUT);
    blink_led(TEST_BLINK,sizeof(TEST_BLINK)/sizeof(TEST_BLINK[0]));
    dacWrite(BRIGHT,255);

    Wire.begin(SDA,SCL);
    init_lcd(&lcd_1);
    init_GY49();
    init_bme280();
    pinMode(D32,OUTPUT);
    digitalWrite(D32,LOW);

    connect_wifi((char *)"StudentOffice",(char *)"studentsoffice3");
    sync_time();

    Serial.println("ready!");
    lcd_1.clear();
}

#define SIGMA_HUMI 9.0F
#define CRATE_HUMI 2.0F //experience parameter, wtf?
#define LINE2_LOOP_PERI 3 //in second, at least 2

float temp_last=NAN,pres_last=NAN,humi_last=NAN,aqi_last=NAN;
float temp_toshow=NAN,pres_toshow=NAN,humi_toshow=NAN,aqi_toshow=NAN;
float temp_now=25;
float pres_now=101000;
float humi_now=50;
float sigma_humi=100;

uint8_t cali_ct=0;

void update_sensor(uint32_t eps){
    float temp_2801,pres_2801,humi_2801;
    temp_2801=bme280_1.readTemperature();
    pres_2801=bme280_1.readPressure();
    humi_2801=bme280_1.readHumidity();

    //average
    float temp_avg,pres_avg,humi_avg;
    temp_avg=temp_2801;
    pres_avg=pres_2801;
    humi_avg=humi_2801;

    //filte
    temp_now+=0.9*(temp_avg-temp_now);
    pres_now+=0.9*(pres_avg-pres_now);
    float k_humi;
    k_humi=sigma_humi/(sigma_humi+SIGMA_HUMI);
    humi_now+=k_humi*(humi_avg-humi_now);
    sigma_humi*=(1-k_humi);
    sigma_humi+=CRATE_HUMI;

    //finally calc what to show
    temp_toshow=round(temp_now*10.0F)/10.0F;
    pres_toshow=round(pres_now/100.0F)/10.0F;
    humi_toshow=round(humi_now);
}

/*float get_lumi(){
    uint32_t data[2]={0,0};

    Wire.beginTransmission(GY49);
    // Select data register
    Wire.write(0x03);
    // Stop I2C transmission
    Wire.endTransmission();
    // Request 2 bytes of data
    Wire.requestFrom(GY49,2);
    // Read 2 bytes of data
    // luminance msb, luminance lsb
    if(Wire.available()==2){
       data[0] = Wire.read();
       data[1] = Wire.read();
    }

    // Convert the data to lux
    int exponent = (data[0] & 0xF0) >> 4;
    int mantissa = ((data[0] & 0x0F) << 4) | (data[1] & 0x0F);
    float luminance = pow(2, exponent) * mantissa * 0.045;

    // Output data to serial monitor
    Serial.print("get lumi:");
    Serial.println(luminance);
    return luminance;
}*/

void show_second_line(uint32_t eps){
    if(temp_toshow!=temp_last){
        lcd_1.setCursor(9,0);
        lcd_1.print(String(temp_toshow,1));
        lcd_1.printByte(UPCIR);
        lcd_1.print("C ");
        temp_last=temp_toshow;
    }

    uint8_t mode=eps%LINE2_LOOP_PERI;
    if(mode==0){ //clear every LINE2_LOOP_PERI second. Because this is designed for multi-line output
        lcd_1.setCursor(0,1);
        lcd_1.print(String(pres_toshow,1)+String("kPa "));
        pres_last=pres_toshow;
        lcd_1.setCursor(9,1);
        lcd_1.print(String(humi_toshow,0)+String("%    "));
        humi_last=humi_toshow;
    }else{
        if(pres_toshow!=pres_last){
            lcd_1.setCursor(0,1);
            lcd_1.print(String(pres_toshow,1)+String("kPa "));
            pres_last=pres_toshow;
        }
        if(humi_toshow!=humi_last){
            lcd_1.setCursor(9,1);
            lcd_1.print(String(humi_toshow,0)+String("%    "));
            humi_last=humi_toshow;
        }
    }
}

uint8_t virtual_touch(){
    Serial.println("virtual touching...");
    lcd_1.setCursor(0,1);
    lcd_1.print("touching...     ");

    digitalWrite(D32,HIGH);
    delay(50); //30 is enough, 10 not work
    digitalWrite(D32,LOW);

    lcd_1.setCursor(0,1);
    lcd_1.print("touched         ");
    delay(DEBUG_DELAY);
    return 0;
}

#define LOOP_DELAY 1 //1ms
#define CLOSE_HOUR 0 //hour to close led
#define OPEN_HOUR 6 //hour to open led
uint32_t eps1,eps2;
String s_first_line="s_first_line";
uint8_t led_status=3; //3 for brightest, 0 for power off

void loop(){
    String s_time=my_time_client.getFormattedTime();
    if(s_time!=s_first_line){
        uint8_t index;
        for(index=0;index<8;index++){
            if(s_time[index]!=s_first_line[index]) break;
        }
        lcd_1.setCursor(index,0);
        lcd_1.print(s_time.substring(index,8));
        s_first_line=s_time;

        int8_t re=my_time_client.update();
        if(re==0){
            Serial.println("0: sync but failed");
        }else if(re==1){
            Serial.println("1: sync and suc");
        }else if(re==2){
            ;
        }else if(re==3){
            Serial.println("3: last failed was just happen");
        }else{
            Serial.print("return value error: ");
            Serial.println(re);
        }
    }

    eps2=my_time_client.getEpochTime();
    if((eps1!=eps2) && (my_time_client.get_millis()>=500)){
        eps1=eps2;
        update_sensor(eps1);
        show_second_line(eps1);

        int hour=my_time_client.getHours();
        int minute=my_time_client.getMinutes();
        Serial.println(String(hour)+":"+String(minute));
        if(hour==CLOSE_HOUR and minute==0 and led_status==3){
            if(virtual_touch()==0) led_status=2;
        }else if(hour==CLOSE_HOUR and minute==30 and led_status==2){
            if(virtual_touch()==0) led_status=1;
        }else if(hour==OPEN_HOUR and minute==0 and led_status==1){
            if(virtual_touch()==0) led_status=0;
        }else if(hour==OPEN_HOUR and minute==10 and led_status==0){
            if(virtual_touch()==0) led_status=3;
        }else{
            ;//Serial.println("not time for touch");
        }
    }
    delay(LOOP_DELAY);
}