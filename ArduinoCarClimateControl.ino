#include "functions.h"

#include <Servo.h>

#include <OneWire.h>
#include <DallasTemperature.h>

#include <Wire.h>
#include <Adafruit_GFX.h>      
#include <Adafruit_SSD1306.h>  
#include <BH1750.h>

#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

////Адреса Пинов
#define POT_PIN A0
#define SERVO_PIN D8
#define THERMO_PIN D6
#define LED 2 

#define IN1 D7 //сигнал от генератора о вращении двигателя 
#define OUT1 D5 //включение фар ближнего света
int engine_timer=0;


#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET LED_BUILTIN  //4 

#define POT_ARR_SIZE 5 //размер массива для сглаживания значений потенциометра
#define POT_GATE 4
#define POT_GATE2 5

#define DELAY 0
#define TEMP_MEASUREMENT_DELAY 750


////////адреса в памяти////////
#define EEPROM_POT sizeof(int)
#define EEPROM_SERVO 0
#define EEPROM_PID 10*sizeof(int)
#define EEPROM_THERMO 20*sizeof(int)


#ifndef APSSID
#define APSSID "HR-V Climate"
#define APPSK  "sochi2014"
#endif

#define SERVER_DEBUG 0  //отладочные сообщения сервера
#define TEMP_DEBUG 0     //отладочные сообщения температур
#define DERIVATIVE_DEBUG 0  //отладочные сообщения производных
#define SERVO_DEBUG 0
#define LIGHT_DEBUG 0
#define ENGINE_DEBUG 0


Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

BH1750 lightMeter;
float light_reading=0;

OneWire oneWire(THERMO_PIN);
DallasTemperature sensors(&oneWire);



uint8_t sensor_1[8] = { 0x28, 0xFF, 0x7D, 0x11, 0xC4, 0xA1, 0x95, 0xB7  };  // уличный 
uint8_t sensor_2[8] = { 0x28, 0xFF, 0x03, 0xC3, 0x80, 0x16, 0x05, 0xAC  };  //поток воздуха
uint8_t sensor_3[8] = { 0x28, 0xA6, 0xC1, 0xD5, 0x0E, 0x00, 0x00, 0x6B  };  //салон

volatile float measured_temperature_1=0,measured_temperature_2=0,measured_temperature_3=0;  //измеряемые температуры
float salon_temperature_wanted=0;   //желаемая температура в потоке
float stream_temperature_wanted=0;  //желаемая температура потока
float salon_temperature_wanted_min=10,salon_temperature_wanted_max=50;             //диапазон возможных желаемых температур
int temperature_equality_point=20;                                                 //температура при которой совпадает желаемая температура потока с желаемой температурой салона
int temperature_k=0;

float temperature1_derivative1=0; //первая производная температуры1
float temperature1_derivative2=0; //вторая производная температуры1
float estimated_temperature_1=0;
boolean is_sensor_reading=0;


Servo my_servo; // create servo object to control a servo

int pot_smoothing_array[POT_ARR_SIZE]= { 0 };
int pot_smoothing_array_cursor = 0;
int pot_pos_new=0;
int pot_pos_percent=0;
int pot_pos_current=0; 
int pot_pos_min=0;
int pot_pos_max=1024;

int pot_center_percent=0;




boolean is_servo_attached = 0;
int servo_pos_new = 0;      
int servo_pos_current = 0;
int servo_pos_percent=0; 
int servo_pos_min=1500;   // границы поворота серво
int servo_pos_max=2380;   //
int servo_move_delay=20000;

//pid_ parameters
int k1=0; 
int k2=0;
int k3=0;
int k4=0;
int k5=0;
int k6=0;
int derivative_delay =2000; //интервал вычисления производных
int light_delay =1500; //интервал измерения света
int engine_delay=3000; //интервал обработки сигнала с двигателя
float integral=0;
int set_request_current_step=0;


long prevMoveTime=0;
int prev_delta=0;
int current_delta=0;

//WiFi Connection configuration
const char *ssid = APSSID;
const char *password = APPSK;

IPAddress my_ip=0;
ESP8266WebServer server(80);

/////////////////////////SETUP////////////////
void setupServo(){
  EEPROM.get(EEPROM_SERVO,servo_pos_current);
  //servo_pos_current=map(servo_pos_percent,0,100,servo_pos_min,servo_pos_max);
  servo_pos_percent=map(servo_pos_current,servo_pos_min,servo_pos_max,0,100);
  my_servo.writeMicroseconds(servo_pos_current);  
  display.display();

  my_servo.attach(SERVO_PIN,500,2500); 
  display.display();
  delay(1500);
  my_servo.detach();
  display.display();
}

void setupWifi(){
  Serial.println("\n");
  Serial.print("Configuring access point...");
  WiFi.softAP(ssid, password);

  my_ip = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(my_ip);
  startServer();
  Serial.println("HTTP server started");
}

void setupDisplay() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x32)
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,28);
  display.display();
  display.clearDisplay();
}

void setupEEPROM() {
  EEPROM.begin(512);
}

void findSensors(){
  int thermo_devices_count=0;
  DallasTemperature sensors(&oneWire);
  DeviceAddress Thermometer;
  sensors.begin();
  Serial.print("Locating devices... \n Found ");
  thermo_devices_count = sensors.getDeviceCount();
  Serial.print(thermo_devices_count, DEC);
  Serial.println(" devices.");
  Serial.println("Printing addresses...");
  for (int i = 0;  i < thermo_devices_count;  i++)
  {
    Serial.print("Sensor ");
    Serial.print(i+1);
    Serial.print(" : ");
    sensors.getAddress(Thermometer, i);
    printAddress(Thermometer);
  }
}

void setupPot(){
  pinMode(POT_PIN, INPUT);
  EEPROM.get(EEPROM_POT,pot_pos_current); //????????????????????????????????????????
  for(int i=0;i < POT_ARR_SIZE;i++){
    pot_smoothing_array[i]=pot_pos_current;
    }
  pot_pos_percent=100*(pot_pos_current-pot_pos_min)/(pot_pos_max-pot_pos_min);
  pot_center_percent=pot_pos_percent;
}

void setup(){
  Serial.begin(9600);

  pinMode(OUT1,OUTPUT);
  pinMode(IN1,INPUT);
  digitalWrite(OUT1, LOW);


  Wire.begin();
  lightMeter.begin();
  
  setupDisplay();
  setupEEPROM();
  prevMoveTime=millis();
  
  setupPot();
  setupWifi();
  setupServo();
  findSensors();
  sensors.setResolution(sensor_1, 12);
  sensors.setResolution(sensor_2, 12);
  sensors.setResolution(sensor_3, 12);
  load_k();
  //EEPROM.get(EEPROM_THERMO,measured_temperature_1);
  //EEPROM.get(EEPROM_THERMO+sizeof(int),measured_temperature_2);
  //EEPROM.get(EEPROM_THERMO+2*sizeof(int),measured_temperature_3);

}



///////////
void print_k()
{
  Serial.print(" k1 ");
  Serial.print(k1);
  Serial.print(" k2 ");
  Serial.print(k2);
  Serial.print(" k3 ");
  Serial.print(k3);
  Serial.print(" k4 ");
  Serial.print(k4);
  Serial.print(" k5 ");
  Serial.print(k5);
  Serial.print(" k6 ");
  Serial.print(k6);
  Serial.println("");
}

void save_k()
{
  EEPROM.put(EEPROM_PID,k1);
  EEPROM.put(EEPROM_PID+sizeof(int),k2);
  EEPROM.put(EEPROM_PID+2*sizeof(int),k3);
  EEPROM.put(EEPROM_PID+3*sizeof(int),k4);
  EEPROM.put(EEPROM_PID+4*sizeof(int),k5);
  EEPROM.put(EEPROM_PID+5*sizeof(int),k6);
  EEPROM.commit();
  if(SERVER_DEBUG){
    Serial.println("Saved k");
    print_k();
  }
}

void load_k()
{
  EEPROM.get(EEPROM_PID,k1);
  EEPROM.get(EEPROM_PID+sizeof(int),k2);
  EEPROM.get(EEPROM_PID+2*sizeof(int),k3);
  EEPROM.get(EEPROM_PID+3*sizeof(int),k4);
  EEPROM.get(EEPROM_PID+4*sizeof(int),k5);
  EEPROM.get(EEPROM_PID+5*sizeof(int),k6);
  if(SERVER_DEBUG){
    Serial.println("Loaded k");
    print_k();
  }
}
 
////////////////////////
void measure_temperature(){ //измерение температур
  static int last_measurement_time=0;
  int current_time=millis();
  if(current_time-last_measurement_time>=TEMP_MEASUREMENT_DELAY)
  {
    if(TEMP_DEBUG)
    {
      Serial.println("temp measurement");
    }
    
    last_measurement_time=current_time;
    
    measured_temperature_1=sensors.getTempC(sensor_1);
    measured_temperature_2=sensors.getTempC(sensor_2);
    measured_temperature_3=sensors.getTempC(sensor_3);

    if(TEMP_DEBUG){
      Serial.print("measured t: ");
      Serial.print(measured_temperature_1);
      Serial.print(" ");
      Serial.print(measured_temperature_2);
      Serial.print(" ");
      Serial.println(measured_temperature_3);
    }

    sensors.requestTemperatures();
    /*oneWire.reset();
    oneWire.select(sensor_1);    
    oneWire.write(0xBE); // Read Scratchpad (чтение регистров)  
    measured_temperature_1 =  oneWire.read() | (oneWire.read()<<8);  //чтение два раза по 8 бит
    //measured_temperature_1=((measured_temperature_1*10)>>4)/10;       //деление на 16, т.к датчик возвращает в виде 16*градус Цельсия
    measured_temperature_1=measured_temperature_1/16;

    oneWire.reset();
    oneWire.select(sensor_2);    
    oneWire.write(0xBE); // Read Scratchpad (чтение регистров)  
    measured_temperature_2 =  oneWire.read()| (oneWire.read()<<8); 
    //measured_temperature_2=((measured_temperature_2*10)>>4)/10;
    measured_temperature_2=measured_temperature_2/16;

    oneWire.reset();
    oneWire.select(sensor_3);    
    oneWire.write(0xBE); // Read Scratchpad (чтение регистров)  
    measured_temperature_3 =  oneWire.read()| (oneWire.read()<<8); 
    //measured_temperature_3=((measured_temperature_3*10)>>4)/10;
    measured_temperature_3=measured_temperature_3/16;



    //EEPROM.put(EEPROM_THERMO,measured_temperature_1);
    //EEPROM.put(EEPROM_THERMO+sizeof(int),measured_temperature_2);
    //EEPROM.put(EEPROM_THERMO+2*sizeof(int),measured_temperature_3);
    //EEPROM.commit();

    


    oneWire.reset();  // сброс шины
        oneWire.write(0xCC);//обращение ко всем датчикам
        oneWire.write(0x44);// начать преобразование (без паразитного питания)  
    */

  } 
  
}
void process_temperature(){ //нахождение приближенной температуры
  static int prev_time=millis();                    //время последнего измерения
  static float prev_value1=measured_temperature_1;   //последнее измерение  
  static float prev_value2=0;   
  int current_time=millis();
  int timer=current_time-prev_time;         //время прошедшее с последнего измерения
  static int time_to_rest=0;

  //if(measured_temperature_1! prev_value)  //измерение по изменению
  if(timer>derivative_delay)                  //измерение по интервалу
  {
    temperature1_derivative1=1000*(measured_temperature_1 -prev_value1)/(current_time-prev_time); // градусы в секунду
    temperature1_derivative2=1000*(temperature1_derivative1-prev_value2)/(current_time-prev_time);

    
    if(DERIVATIVE_DEBUG){
          Serial.print("t1 derivative: ");
          Serial.print(temperature1_derivative1);
          Serial.print(" current value ");
          Serial.print(measured_temperature_1);

          Serial.print(" last value1 ");
          Serial.print (prev_value1);
          Serial.print(" last value2 ");
          Serial.print (prev_value2);
          Serial.print(" current time ");
          Serial.print(current_time);
          Serial.print(" last time ");
          Serial.print(prev_time);
          Serial.print(" delay ");
          Serial.println(derivative_delay);
        }
    
    prev_value1=measured_temperature_1;
    prev_value2=temperature1_derivative1;
    prev_time=current_time;
    if(sign(temperature1_derivative1)!=sign(temperature1_derivative2))
    {
      Serial.print(" ye ");
      time_to_rest=abs(temperature1_derivative1/temperature1_derivative2);
    }
    else
    {
      time_to_rest=0;
    }
    
  }
  estimated_temperature_1=measured_temperature_1+temperature1_derivative1*time_to_rest;
}

void process_engine(){
  static int prev_time=millis();                    //время последнего измерения
  int current_time=millis();
  int timer=current_time-prev_time;      

  if(timer>engine_delay)                  //измерение по интервалу
  {
    prev_time=current_time;  
    int in1=digitalRead(IN1);
     if (in1)
      {
      if (millis()-engine_timer>1000){
        digitalWrite(OUT1, HIGH);
        }
      }
      else{
        engine_timer=millis();
        digitalWrite(OUT1, LOW);
    }
    if(ENGINE_DEBUG){
      Serial.print("input: ");
      Serial.println(in1);
    }
  }

  
 
}
void process_light(){ 
  static int prev_time=millis();                    //время последнего измерения
  int current_time=millis();
  int timer=current_time-prev_time;         //время прошедшее с последнего измерения

  //if(measured_temperature_1! prev_value)  //измерение по изменению
  if(timer>light_delay)                  //измерение по интервалу
  {
    prev_time=current_time;  
    light_reading = lightMeter.readLightLevel();
    if(LIGHT_DEBUG){
      Serial.print("Light: ");
      Serial.print(  light_reading);
      Serial.println(" lx");
    }
  }
}

void moveServo(){
  static int prev_time=millis();
  int current_time=millis();
  

  servo_pos_new=map(servo_pos_percent,0,100,servo_pos_min,servo_pos_max);
  EEPROM.put(EEPROM_SERVO,servo_pos_new);
  EEPROM.commit();
  
  if(SERVO_DEBUG){
  Serial.print("moveServo: ");
  Serial.print(servo_pos_new);
  Serial.print(" ");
  Serial.print(servo_pos_current);
  Serial.print(" ");
  Serial.print(servo_pos_percent);
  Serial.print(" ");
  Serial.print(pot_pos_percent);
  Serial.print(" ");
  Serial.print(servo_pos_min);
  Serial.print(" ");
  Serial.print(servo_pos_max);
  Serial.println(" | ");
  }

  if(servo_pos_new!=servo_pos_current || current_time-prev_time>=servo_move_delay)  
  {
    if(SERVO_DEBUG){Serial.println(" servo update ");}
    prev_time=current_time;
    servo_pos_current=servo_pos_new;
    my_servo.writeMicroseconds(servo_pos_current);
    display.display();
    if(is_servo_attached==0){
      my_servo.attach(SERVO_PIN,500,2500);
      is_servo_attached=1;
      if(SERVO_DEBUG){Serial.println(" servo attached ");}
      }
  }
  else
  {
    if(is_servo_attached!=0){
      display.display();
      my_servo.detach(); 
      display.display();
      is_servo_attached=0;
      if(SERVO_DEBUG){Serial.println(" servo detached ");}
    }
  }
}

void readPotSmooth(){
  pot_pos_new=0;
  pot_smoothing_array[pot_smoothing_array_cursor] =analogRead(POT_PIN);
  pot_smoothing_array_cursor=(pot_smoothing_array_cursor+1)% POT_ARR_SIZE;  //увеличение на 1 и округление до размеров массива
  for(int i=0;i < POT_ARR_SIZE;i++){
    pot_pos_new+=pot_smoothing_array[i];
  //  Serial.print(pot_smoothing_array[i]);
  //  Serial.print(" ");
  }
  pot_pos_new=pot_pos_new/POT_ARR_SIZE;
  //Serial.print("  Average: ");
  //Serial.print(pot_pos_new);
  //Serial.print(" Old: ");
  //Serial.print(pot_pos_current);
  //Serial.print("\n");

  if(abs(pot_pos_current-pot_pos_new)>=POT_GATE)
  {
    pot_pos_current=pot_pos_new;
    EEPROM.put(EEPROM_POT,pot_pos_current);
    EEPROM.commit();
  }
  pot_pos_percent=100*(pot_pos_current-pot_pos_min)/(pot_pos_max-pot_pos_min);
  //Serial.println(pot_pos_percent);
}

///HANDLERS////////
void startServer(){
  server.on("/",handleRoot);                          //отправка веб страницы
  server.on("/getData",handleDataGetRequest);  //отправка температуры и остальных показателей на сайт
  server.on("/setParams",handleParametersSetRequest); //отправка коэффициентов с сайта
  server.on("/getParams",handleParametersGetRequest); //отправка коэффициентов на сайт
  server.on("/saveParams",handleParametersSaveRequest); //сохранение коэффициентов в eeprom
  server.on("/loadParams",handleParametersLoadRequest); //загрузка коэффициентов из eeprom
  server.on("/resetClock",handleResetClock); //загрузка коэффициентов из eeprom
  
  server.begin(); 
}
void handleResetClock()
{
  set_request_current_step=0;
  server.send(200, "text/plane","");
}

void handleRoot() {
 server.send(200, "text/html", MAIN_page); //основная страница
}
void handleDataGetRequest(){
  server.send(200, "text/plane",String(measured_temperature_1)+"|"
                              +String(estimated_temperature_1)+"|"
                              +String(temperature1_derivative1)+"|"
                              +String(temperature1_derivative2)+"|"
                              +String(measured_temperature_2)+"|"
                              +String(measured_temperature_3)+"|"
                              +String(salon_temperature_wanted)+"|"
                              +String(stream_temperature_wanted)+"|"
                              +String(pot_pos_percent)+"|"
                              +String(servo_pos_percent)+"|"
                              +String(light_reading));
}

void handleParametersSetRequest(){
  int request_step=server.arg("time").toInt();
  if(request_step>=set_request_current_step)  //запросы, дошедшие с опозданием игнорируются
  {
    set_request_current_step=request_step;
    k1 = server.arg("one").toInt();
    k2 = server.arg("two").toInt();
    k3 = server.arg("three").toInt();
    k4 = server.arg("four").toInt();
    k5 = server.arg("five").toInt();
    k6 = server.arg("six").toInt();
    //derivative_delay = server.arg("four").toInt();
    server.send(200, "text/plane","");
    if(SERVER_DEBUG){
      Serial.println("Set k");
      print_k();
    }
  }
  else{
    if(SERVER_DEBUG){
      Serial.print("Skipped set request number ");
      Serial.print(request_step);
      Serial.print(" latest ");
      Serial.println(set_request_current_step);
    }
  }
}

void handleParametersSaveRequest(){
  save_k();
  server.send(200, "text/plane","");
}

void handleParametersGetRequest(){
  if(SERVER_DEBUG){
  Serial.println("Got k");
  print_k();
  }
  server.send(200, "text/plane",String(k1)+"|"+String(k2)+"|"+String(k3)+"|"+String(k4)+"|"+String(k5)+"|"+String(k6));
}

void handleParametersLoadRequest(){
  load_k();
  server.send(200, "text/plane",String(k1)+"|"+String(k2)+"|"+String(k3)+"|"+String(k4)+"|"+String(k5)+"|"+String(k6));
}

///////////////////////////////////////////

int computePID(float input, float setpoint, float kp, float ki, float kii, float kd, float dt, int minOut, int maxOut) {
  kp=kp/100;
  ki=ki/100;
  kd=kd/100;
  float err = setpoint - input;
  static float prevErr = 0;
  integral = constrain(integral + (float)err * kii, -50, 50);
  float D = (err - prevErr) / dt;
  prevErr = err;
  Serial.println(integral);
  Serial.println(err);
  Serial.println(prevErr);
  Serial.println(D);
  return constrain(err * kp + 50+integral*ki + D * kd, minOut, maxOut);
}

float step_by_step_search( float prev_servo_value, float current_temperature, float wanted_temperature, float temp_error ,float step_size, long delay){
  
  /*
  Serial.print("prev ");
  Serial.print(prevMoveTime);
  Serial.print(" ");
  Serial.print("current ");
  Serial.print(millis());
  Serial.print(" ");
  Serial.print("delay ");
  Serial.print(delay);
  Serial.print("    ");
  Serial.print("wanted ");
  Serial.print(wanted_temperature);
  Serial.print(" ");
  Serial.print("current ");
  Serial.print(current_temperature);
  Serial.print("   ");
  Serial.print("err ");
  Serial.print(temp_error);
  Serial.print(" ");

  */
    current_delta=abs(wanted_temperature-current_temperature);
    if(current_delta>temp_error){
      /*
      Serial.print("delta");
      Serial.print(wanted_temperature-current_temperature);
      Serial.print(" ");
      */
      if(millis()-prevMoveTime>delay){
        /*
        Serial.print("Prev_Delta");
        Serial.print(prev_delta);
        Serial.print(" ");
        */
        if(current_delta>=prev_delta){
          prevMoveTime=millis();
          prev_delta=current_delta;
          //Serial.print("upd");
          return constrain(prev_servo_value+sign(wanted_temperature-current_temperature)*constrain(current_delta-1,1,step_size),0,100); 
        }
      }
      else
      { 
        return prev_servo_value;
      }
  }
  else
      { 
        return prev_servo_value;
      }
}

void loop(){
  Serial.println("--------tick-------");
  server.handleClient();
  measure_temperature();
  readPotSmooth();
  
  salon_temperature_wanted = map(pot_pos_percent,0,100,salon_temperature_wanted_min,salon_temperature_wanted_max);

  stream_temperature_wanted = salon_temperature_wanted;//+temperature_k*( temperature_equality_point-measured_temperature_2);

  process_temperature();
  process_light();
  process_engine();
  //в крайнем положении должен игнорировать пид и просто выставлят
  //добавить ипром на серво
  
  if (pot_pos_percent>93){
    salon_temperature_wanted=salon_temperature_wanted_max;
    servo_pos_percent=100;
  }
  else
  {
    if (pot_pos_percent<7){
      salon_temperature_wanted=salon_temperature_wanted_min;
    servo_pos_percent=0;
  }
  else
    {
      if(measured_temperature_1>200 || k1==0){ //мануальный режим при первом коэффициенте ==0 либо нерабочем датчике
        servo_pos_percent=pot_pos_percent;
        }
      else
      {
        if(abs(pot_pos_percent-pot_center_percent)>POT_GATE2){
            if(sign(pot_pos_percent-pot_center_percent)==sign(pot_pos_percent-servo_pos_percent)){  //резко двигаем серво на новую позицию только если это будет движение в нужном направлении
              servo_pos_percent=pot_pos_percent;
            }
            pot_center_percent=pot_pos_percent;
              }
        else
        {
            servo_pos_percent=step_by_step_search(servo_pos_percent,measured_temperature_1,stream_temperature_wanted, k1, k2, k3);
            //Serial.print(servo_pos_percent);
        }
        //servo_pos_percent=computePID(measured_temperature_1,salon_temperature_wanted, k1, k2, 1 ,k3, DELAY, 0, 100);
      }
    }
  }
  
  //servo_pos_percent=pot_pos_percent;

  moveServo();    

  /*
  display.clearDisplay(); 
  
  displayTemperature(&display,0,0,1,measured_temperature_1);
  displayTemperature(&display,40,0,1,measured_temperature_2);
  displayTemperature(&display,80,0,1,measured_temperature_3);
  
  displayText(&display,00,10,1,"min");
  displayText(&display,20,10,1,servo_pos_min);
  
  displayText(&display,70,10,1,"max");
  displayText(&display,90,10,1,servo_pos_max);

  displayText(&display,0,20,1,"%");
  displayText(&display,25,20,1,pot_pos_percent);

  displayText(&display,0,30,1,"abs");
  displayText(&display,25,30,1,servo_pos_current);

  displayText(&display,0,40,1,k1);
  displayText(&display,20,40,1,k2);
  displayText(&display,40,40,1,k3);

  displayText(&display,40,45,1,"");
  display.print(my_ip);

  display.display();
  delay(DELAY);
  */
}
