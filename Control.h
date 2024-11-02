#include <Arduino.h>
#include <TroykaCurrent.h>
#include <ESPAsyncWebServer.h>
//#include <HCSR04.h>
#include <WiFi.h>

#include <AsyncDelay.h>
#include <jsnsr04t.h>

//====================================
// датчик температуры //
#define B 3950              // B-коэффициент
#define SERIAL_R 10000      // сопротивление последовательного резистора, 10 кОм
#define THERMISTOR_R 22000  // номинальное сопротивления термистора, 10 кОм
#define NOMINAL_T 25        // номинальная температура (при которой TR = 10 кОм)  #define B 3950 // B-коэффициент
                            // номинальная температура (при которой TR = 10 кОм)
//==============================
bool isSaveControl = false;
//=============================

struct Relay {
  int pin;
  bool manualMode;
  bool statePin;
  bool lastState;
  String description;
};

struct Control {
  std::vector<Relay> relays;
  int limitWorkPressurePump;
  int limitTimeWorkMainPump;
  float limitTemperature;
  int limitBigTank;
  int limitSmallTank;
  int limitOffSmallTank;
  bool distanceBigTank;
  bool isWorkInCold;
};

Control control;

const int relay_1 = 5;   // Скважинный насос
const int relay_2 = 18;  // Насос давления
const int relay_3 = 16;  // Насос перекачки
const int relay_4 = 23;  // Воздушный компрессор

// Функция для инициализации реле в структуре управления
void initializeControl() {
  // Инициализация массива реле
  control.relays = {
    { relay_1, false, false, false, "Скважинный насос" },
    { relay_2, false, false, false, "Насос давления" },
    { relay_3, false, false, false, "Насос перекачки" },
    { relay_4, false, false, false, "Воздушный компрессор" }
  };
  control.limitWorkPressurePump = 1800;
  control.limitTemperature = 5;
  control.limitBigTank = 100;
  control.limitSmallTank = 100;
  control.limitOffSmallTank = 130;
  control.limitTimeWorkMainPump = 10800;  // 4 часа = 14400 3 = 10800
  control.distanceBigTank = false;
  control.isWorkInCold = false;
}
//==================================
//25,2
//==================================
const int tempPin = 32;          // датчик температуры //+
const int sensButtonPin_1 = 17;  //34;  // датчик воды кнопка
const int sensor_2 = 33;         // датчик тока1 // +
//const int sensor_3 = 35; // датчик тока2 //-
// датчик растояния_1 //+
const int TRIG_PIN_1 = 19;  //RX
const int ECHO_PIN_1 = 21;  // XT
// датчик растояния_2 //+
const int TRIG_PIN_2 = 22;  //RX
const int ECHO_PIN_2 = 34; //17  // XT
//==================================
// температура при которой включается скваженный насос, если меньше насос не работает. Будет по другому когда будет компрессор
float Temp = 0.5;
String text = "";

int distanceToWaterInSmallTanks;
int distanceToWaterInBigTank;

float sensorCurrent_1;

unsigned long buttonPressStartTime = 0;

bool lastOnMainPump = false;

bool isStartCompressorScript = false;

unsigned long timerWaterInSmallTanks = 0;
unsigned long timerWaterInSmallTanks2 = 0;
unsigned long TimerRele2 = 0;
unsigned long timeWorkMainPump = 0;
unsigned long timeTransferMainPump = 0;
unsigned long timerSmallTanks = 0;
unsigned long timerSmallTanks2 = 0;
unsigned long timerWorkСompressor = 0;
unsigned long timerDistanceBigTank = 0;
unsigned long timerDistanceBigTank2 = 0;

bool startTimerWorkСompressor = false;
bool startTimeTransferMainPump = false;
bool startTimerRele2 = false;
bool startTimerWaterInSmallTanks = false;
bool startTimerWaterInSmallTanks2 = false;
bool startTimerSmallTanks = false;
bool startTimerSmallTanks2 = false;
bool startTimeWorkMainPump = false;

bool startTimerDistanceBigTank = false;
bool startTimerDistanceBigTank2 = false;

bool falgShowMessagePressurePump = true;

bool falgShowMessageWaterInSmallTanks = true;
bool falgShowMessageWaterInSmallTanks2 = true;

bool falgShowMessageOnPumpSmallTanks = true;
bool falgShowMessageOnPumpSmallTanks2 = true;

bool flagShowMessageOnPumpBigTank = true;
bool flagShowMessageOnPumpBigTank2 = true;

int stageWorkingCompressor = 0;

//=======================

ACS712 sensorCurrent(sensor_2);

//HCSR04 hc(TRIG_PIN_1, ECHO_PIN_1);
//HCSR04 hc2(TRIG_PIN_2, ECHO_PIN_2);

JsnSr04T ultrasonicSensor1(ECHO_PIN_1, TRIG_PIN_1, LOG_LEVEL_FATAL ); // LOG_LEVEL_VERBOSE
AsyncDelay measureDelay1;

JsnSr04T ultrasonicSensor2(ECHO_PIN_2, TRIG_PIN_2, LOG_LEVEL_FATAL ); // LOG_LEVEL_VERBOSE
AsyncDelay measureDelay2;

//======================

bool debug = false;
//====================================================//

float getTemp() {
  int tt = analogRead(tempPin);
  int t = map(tt, 0, 4095, 0, 1023);
  float tr = 1023.0 / t - 1;
  tr = SERIAL_R / tr;
  float steinhart;
  steinhart = tr / THERMISTOR_R;            // (R/Ro)
  steinhart = log(steinhart);               // ln(R/Ro)
  steinhart /= B;                           // 1/B * ln(R/Ro)
  steinhart += 1.0 / (NOMINAL_T + 273.15);  // + (1/To)
  steinhart = 1.0 / steinhart;              // Invert
  steinhart -= 273.15 + 1;

  steinhart = (steinhart < -200) ? 5.0 : steinhart;

  return steinhart;
}
//===================================

void setupControl() {
  // Кнопки датчики //
  pinMode(relay_1, OUTPUT);
  pinMode(relay_2, OUTPUT);
  pinMode(relay_3, OUTPUT);
  pinMode(relay_4, OUTPUT);

  pinMode(sensButtonPin_1, INPUT_PULLDOWN);

  digitalWrite(relay_1, LOW);
  digitalWrite(relay_2, LOW);
  digitalWrite(relay_3, LOW);
  digitalWrite(relay_4, LOW);

  pinMode(tempPin, INPUT);   // датчик тепературы
  pinMode(sensor_2, INPUT);  // датчик тока 1

  // датчик растояния 1
  pinMode(TRIG_PIN_1, OUTPUT);
  pinMode(ECHO_PIN_1, INPUT);

    // датчик растояния 2
  pinMode(TRIG_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_2, INPUT);

  for (auto& r : control.relays) {
    pinMode(r.pin, OUTPUT);
    if (r.manualMode) {
      digitalWrite(r.pin, r.statePin ? HIGH : LOW);  // Устанавливаем состояние пина
      //r.lastState = r.statePin;
    }
  }

    ultrasonicSensor1.begin(Serial);
  measureDelay1.start(500, AsyncDelay::MILLIS);

      ultrasonicSensor2.begin(Serial);
  measureDelay2.start(500, AsyncDelay::MILLIS);
}
//======================
String debugInfo() {
  String debugString;

  debugString += "\nРеле:\n";
  for (const auto& relay : control.relays) {
    debugString += "Pin: " + String(relay.pin);
    debugString += ", ManualMode: " + String(relay.manualMode);
    debugString += ", statePin: " + String(digitalRead(relay.pin));
    debugString += ", Description: " + relay.description;
    debugString += "\n";
  }

  debugString += "\nПараметры контроля:\n";
  debugString += "Limit Work Pressure Pump: " + String(control.limitWorkPressurePump) + "\n";
  debugString += "Limit Time Work Main Pump: " + String(control.limitTimeWorkMainPump) + "\n";
  debugString += "Limit Temperature: " + String(control.limitTemperature) + "\n";
  debugString += "Limit Big Tank: " + String(control.limitBigTank) + "\n";
  debugString += "Limit Small Tank: " + String(control.limitSmallTank) + "\n";

  debugString += "\nДатчики и кнопки:\n";
  debugString += "Temp Pin: " + String(tempPin) + "\n";
  debugString += "Sensor 2 Pin: " + String(sensor_2) + "\n";
  debugString += "Sensor Button Pin: " + String(sensButtonPin_1) + "\n";
  debugString += "TRIG Pin 1: " + String(TRIG_PIN_1) + "\n";
  debugString += "ECHO Pin 1: " + String(ECHO_PIN_1) + "\n";
  debugString += "TRIG Pin 2: " + String(TRIG_PIN_2) + "\n";
  debugString += "ECHO Pin 2: " + String(ECHO_PIN_2) + "\n";

  debugString += "\nТекущие значения:\n";
  debugString += "Temp: " + String(Temp) + "\n";

  debugString += "Current sensor: " + String(sensorCurrent_1) + "\n";
  debugString += "Distance To Water In Small Tanks: " + String(distanceToWaterInSmallTanks) + "\n";
  debugString += "Distance To Water In Big Tank: " + String(distanceToWaterInBigTank) + "\n";
  debugString += "Sensor Button state Pin: " + String(digitalRead(sensButtonPin_1)) + "\n";
  debugString += "Button Press Start Time: " + String(buttonPressStartTime) + "\n";

  debugString += "Timer Water In Big Tank: " + String(timeWorkMainPump) + "\n";
  debugString += "Timer Pressure pump: " + String(TimerRele2) + "\n"; 
  debugString += "Work in cold: " + String(control.isWorkInCold) + "\n";

  if (Serial.available() > 0) {
    Serial.println(debugString);
  }
  return debugString;
}


void manualWork(String arg) {
  if (arg == "?\n" || arg == "status\n") {
    debugInfo();
  }
  debug = (arg == "debug\n");
}

//=====================================================//

void printRelayStates(void (*callFunc)(String)) {
  for (auto& relay : control.relays) {
    relay.statePin = digitalRead(relay.pin);  // Читаем текущее состояние реле

    // Проверяем, изменилось ли состояние
    if (relay.statePin != relay.lastState) {
      String sOut = relay.description;
      sOut += ": ";
      sOut += relay.statePin ? "включено" : "отключено"; // включено / отключено
      sOut += "\n";

      callFunc(sOut);

      relay.lastState = relay.statePin;  // Обновляем предыдущее состояние
    }
  }
}

//=================[Главный сценарий]==================//

 void mainScenario(void (*callFunc)(String)) {  

  if (Serial.available() > 0) {
    String arg = Serial.readString();
    manualWork(arg);
    if (debug) {
      debugInfo();
    }
  }

//  distanceToWaterInSmallTanks = hc.dist();
//  distanceToWaterInBigTank = hc2.dist();

  if (measureDelay1.isExpired()) {
      distanceToWaterInSmallTanks = ultrasonicSensor1.readDistance();    
        measureDelay1.repeat();
    } 

   if (measureDelay2.isExpired()) {
      distanceToWaterInBigTank = ultrasonicSensor2.readDistance();    
        measureDelay2.repeat();
    }    
  

  float val = sensorCurrent.readCurrentAC() - 8.14;
  sensorCurrent_1 = (val > 0) ? val : 0;

  Temp = getTemp();

  bool isWorkingСold = control.isWorkInCold ? true : (Temp > control.limitTemperature);

  delay(200);

  //==================[ Cкваженный насос ]================//

  if (control.relays[0].statePin) {
    startTimeWorkMainPump = true;

    stageWorkingCompressor = 0;

  } else {
    startTimeWorkMainPump = false; 
    }

    if (startTimeWorkMainPump && timeWorkMainPump > control.limitTimeWorkMainPump) {

      digitalWrite(relay_1, LOW);
      control.relays[0].manualMode = true;
      String say = "Насос работает более установленного времени: " + String(control.limitTimeWorkMainPump / 60) + " минут" + "!\n Исправте ошибку и запустите систему снова /restart \n";
      callFunc(say);
      timeWorkMainPump = 0;
      startTimeWorkMainPump = false;

      lastOnMainPump = true;
    }

    //============================[ Cкваженный насос поплывок ]==============================
    
    if (!control.relays[0].manualMode && !startTimerWorkСompressor) {  // если не ручной режим:
      // с кнопкой
      //На контакте Sensor1 подтянуто минус и при замыкании на плюс происходит событие // buttonSensor1.read()  // Реле 1 , насос включен
      if (!control.distanceBigTank && digitalRead(sensButtonPin_1) == HIGH && isWorkingСold && flagShowMessageOnPumpBigTank && !startTimerWorkСompressor) {  //&& TimerWork > 5

        digitalWrite(relay_1, HIGH);
        startTimeWorkMainPump = true;

        lastOnMainPump = false;

        flagShowMessageOnPumpBigTank = false;

      } else if (digitalRead(sensButtonPin_1) == LOW && !flagShowMessageOnPumpBigTank) { // && Temp > control.limitTemperature

        digitalWrite(relay_1, LOW);

        startTimeWorkMainPump = false;
        flagShowMessageOnPumpBigTank = true;

        lastOnMainPump = true;
      }
    }

    //================[ Насос давления ]===================

    if (sensorCurrent_1 > 3) {
      startTimerRele2 = true;
      if (TimerRele2 > control.limitWorkPressurePump && falgShowMessagePressurePump) {
        
        String say = "Насос давления работает без перерыва более 30 минут \nПроверьте утечку и перезапустите систему /resetManual\n";
        Serial.println(say);
        digitalWrite(relay_2, HIGH);  // тут наоборот
        
        callFunc(say);
        falgShowMessagePressurePump = false;
      }
    } else {
      falgShowMessagePressurePump = true;
      startTimerRele2 = false;
    }

    if (distanceToWaterInSmallTanks < 500 && distanceToWaterInSmallTanks != 0) {

      if (distanceToWaterInSmallTanks > control.limitOffSmallTank && !control.relays[1].manualMode) {
        if (!control.relays[1].manualMode) {
          startTimerSmallTanks = true;
          if (timerSmallTanks > 5 && falgShowMessageWaterInSmallTanks) {
            //delay(300);
            digitalWrite(relay_2, HIGH);  // тут наоборот
            Serial.println("Низкий уровень воды в малых баках");
            String say = "Низкий уровень воды в малых баках! \n";

            callFunc(say);
            falgShowMessageWaterInSmallTanks = false;
            falgShowMessageWaterInSmallTanks2 = true;
            startTimerSmallTanks = false;
            timerSmallTanks = 0;
          }
        }
      } else if (distanceToWaterInSmallTanks < control.limitOffSmallTank && falgShowMessageWaterInSmallTanks2 && !control.relays[1].manualMode) {
        startTimerSmallTanks2 = true;
        if (timerSmallTanks2 > 5) {
          //delay(300);
          digitalWrite(relay_2, LOW);  // тут наоборот

          falgShowMessageWaterInSmallTanks = true;
          falgShowMessageWaterInSmallTanks2 = false;
          startTimerSmallTanks2 = false;
          timerSmallTanks2 = 0;
        }
      }
    }

    //==================[ Насос перекачки ]====================

    if (distanceToWaterInSmallTanks < 500 && distanceToWaterInSmallTanks != 0) {
      if (distanceToWaterInSmallTanks > control.limitSmallTank) {
        startTimerWaterInSmallTanks = true;
        if (timerWaterInSmallTanks > 5 && falgShowMessageOnPumpSmallTanks && !control.relays[2].manualMode) {
          //delay(300);
          digitalWrite(relay_3, HIGH);

          falgShowMessageOnPumpSmallTanks2 = true;
          falgShowMessageOnPumpSmallTanks = false;
          startTimerWaterInSmallTanks = false;
          timerWaterInSmallTanks = 0;
        }
      } else {
        startTimerWaterInSmallTanks = false;
      }
    }

    // если выключили контроллер нааполнили каким то образом баки потом включили то не стработает
    if (distanceToWaterInSmallTanks < 33 && distanceToWaterInSmallTanks != 0) {  // && !control.relays[3].manualMode) растояние когда нужно ОТКЛючить насос перекачк
      startTimerWaterInSmallTanks2 = true;

      if (timerWaterInSmallTanks2 > 5 && falgShowMessageOnPumpSmallTanks2) {  //
        //delay(300);
        digitalWrite(relay_3, LOW);  // выключаем реле 3 - насос перекачки

        falgShowMessageOnPumpSmallTanks = true;
        falgShowMessageOnPumpSmallTanks2 = false;
        startTimerWaterInSmallTanks2 = false;
        timerWaterInSmallTanks2 = 0;
      }
    } else {
      startTimerWaterInSmallTanks2 = false;
    }

    //=====================[ Скважинный насос при дистанции ]=====================

    if (control.distanceBigTank && distanceToWaterInBigTank < 500 && distanceToWaterInBigTank != 0 && !startTimerWorkСompressor) {
      if (distanceToWaterInBigTank > control.limitBigTank && flagShowMessageOnPumpBigTank && isWorkingСold && !control.relays[0].manualMode) {
        startTimerDistanceBigTank = true;
        if (timerDistanceBigTank > 5) {
          //delay(200);
          digitalWrite(relay_1, HIGH);

          flagShowMessageOnPumpBigTank2 = true;
          flagShowMessageOnPumpBigTank = false;
          startTimerDistanceBigTank = false;
          timerDistanceBigTank = 0;

          lastOnMainPump = false;
        }
      } else if (distanceToWaterInBigTank < 33 && !flagShowMessageOnPumpBigTank && !control.relays[0].manualMode) {
        startTimerDistanceBigTank2 = true;
        if (timerDistanceBigTank2 > 5) {
          //delay(200);
          digitalWrite(relay_1, LOW);

          flagShowMessageOnPumpBigTank2 = true;
          flagShowMessageOnPumpBigTank = true;
          startTimerDistanceBigTank2 = false;
          timerDistanceBigTank2 = 0;

          lastOnMainPump = true;
        }
      }
    }

    //=======================[ Компрессор сценарий ]===============================
    ///stageWorkingCompressor = 0 потом включается сценарий и stageWorkingCompressor = 1 до 5 так и остается пока не сработает скваенный насос и stageWorkingCompressor = 0
   
    if ((digitalRead(relay_1) != HIGH && Temp < control.limitTemperature && lastOnMainPump && stageWorkingCompressor != 5 && !control.relays[3].manualMode ) || isStartCompressorScript) {

      //-------- 1 - Отключены компрессор и насос чтобы вода слилась

      isStartCompressorScript = true;
      
      if (timerWorkСompressor == 0 && stageWorkingCompressor == 0) {
        
      startTimerWorkСompressor = true;
      stageWorkingCompressor = 1;  

      digitalWrite(relay_1, LOW);
      digitalWrite(relay_4, LOW);

      String say = "Запущен сценарий продувки трубы - этап #0\n";
      callFunc(say);
      } else
      
      //---------- Ожидаем 15 минут и включаем компрессор
       
      if (timerWorkСompressor > 900 && stageWorkingCompressor == 1) {

      digitalWrite(relay_1, LOW);
      digitalWrite(relay_4, HIGH);

      stageWorkingCompressor = 2;

      String say = "Cценарий продувки трубы - этап #1\n";
      callFunc(say);
      } else

      //---------- Отключаем компрессор через 30 минут
      
      if (timerWorkСompressor > 2700 && stageWorkingCompressor == 2) {
        digitalWrite(relay_4, LOW);
        stageWorkingCompressor = 3;

      String say = "Сценарий продувки трубы - этап #2\n";
      callFunc(say);
      } else
      
      //---------- Включили на 15 минут
      
        if (timerWorkСompressor > 3600 && stageWorkingCompressor == 3) {
        digitalWrite(relay_4, HIGH);
        stageWorkingCompressor = 4;

      String say = "Сценарий продувки трубы - этап #3\n";
      callFunc(say);
      } else
      
      //---------- Отключили через 15 минут
      
        if (timerWorkСompressor > 4500 && stageWorkingCompressor == 4) {
        digitalWrite(relay_4, LOW);

        startTimerWorkСompressor = false;
        stageWorkingCompressor = 5;
        lastOnMainPump = false;

        isStartCompressorScript = false;

      String say = "Сценарий трубы остановлен - этап #5\n";
      callFunc(say);
      }
    }

    printRelayStates(callFunc);

  }

  ////=======================================================

   void controlTask() {

//  void controlTask(void* pvParameters) {
//    const TickType_t xDelay = pdMS_TO_TICKS(1000);
//
//    while (true) {

      //Timer for Rele2 тайемер если насос давления работает более 30 минут +
      if (startTimerRele2) {
        TimerRele2++;
      } else {
        TimerRele2 = 0;
      }

      // Timer for SmallTanks насос давления , малые баки 5 секунд +
      if (startTimerSmallTanks) {
        timerSmallTanks++;
      }

      if (startTimerSmallTanks2) {
        timerSmallTanks2++;
      }

      // Timer for WaterInSmallTanks насос перекачки 5 сек +
      if (startTimerWaterInSmallTanks) {
        timerWaterInSmallTanks++;
      }

      // Timer for WaterInSmallTanks2 насос перекачки отключение 5 сек +
      if (startTimerWaterInSmallTanks2) {
        timerWaterInSmallTanks2++;
      }


      // таймер основного насоса 5 сек при дистанции +
      if (startTimerDistanceBigTank) {
        timerDistanceBigTank++;
      }

      if (startTimerDistanceBigTank2) {
        timerDistanceBigTank2++;
      }

      // таймер если скважный насос работает более 3 часов +
      if (startTimeWorkMainPump) {
        timeWorkMainPump++;
      } else {
        timeWorkMainPump = 0;
      }

      // Timer for WorkCompressor
      if (startTimerWorkСompressor) {
        timerWorkСompressor++;
      } else {
        timerWorkСompressor = 0;
      }
      //===============
      //debugInfo();
      //   statusUpdate = mainScenario();
//      vTaskDelay(xDelay);
//    }
  }
  //======================================================
  // Функция для сохранения структуры Control в SPIFFS
  void saveControlToSPIFFS() {
    isSaveControl = true;
    // Открытие файла для записи
    File file = SPIFFS.open("/control.json", FILE_WRITE);
    if (!file) {
      isSaveControl = false;
      Serial.println("Не удалось открыть файл для записи");
      return;
    }

    // Создание JSON документа
    DynamicJsonDocument doc(2048);
    JsonArray relays = doc.createNestedArray("relays");

    for (auto& relay : control.relays) {
      JsonObject relayObj = relays.createNestedObject();
      relayObj["pin"] = relay.pin;
      relayObj["manualMode"] = relay.manualMode;
      relayObj["statePin"] = relay.statePin;
      relayObj["description"] = relay.description;
    }

    // Добавление новых параметров
    doc["limitWorkPressurePump"] = control.limitWorkPressurePump;
    doc["limitTemperature"] = control.limitTemperature;
    doc["limitBigTank"] = control.limitBigTank;
    doc["limitSmallTank"] = control.limitSmallTank;
    doc["limitOffSmallTank"] = control.limitOffSmallTank;
    doc["limitTimeWorkMainPump"] = control.limitTimeWorkMainPump;
    doc["distanceBigTank"] = control.distanceBigTank; 
    doc["isWorkInCold"] = control.isWorkInCold;
    // Сериализация JSON документа в файл
    if (serializeJson(doc, file) == 0) {
      Serial.println("Ошибка при записи в файл");
    }

    file.close();
    isSaveControl = false;
  }

  //=======================================================
  // Функция для чтения структуры Control из SPIFFS
  void loadControlFromSPIFFS() {
    // Открытие файла для чтения
    File file = SPIFFS.open("/control.json", FILE_READ);
    if (!file) {
      Serial.println("Не удалось открыть файл для чтения");
      initializeControl();
      saveControlToSPIFFS();
      delay(100);
      return;
    }

    // Создание JSON документа
    DynamicJsonDocument doc(2048);
    DeserializationError error = deserializeJson(doc, file);
    if (error) {
      Serial.println("loadControl: Ошибка при чтении файла");
      delay(1000);
      initializeControl();
      delay(500);
      saveControlToSPIFFS();
      delay(1000);
      return;
    }

    // Чтение данных из JSON документа
    JsonArray relays = doc["relays"];
    control.relays.clear();  // Очистка вектора перед загрузкой новых данных
    for (JsonObject relayObj : relays) {
      Relay relay;
      relay.pin = relayObj["pin"];
      relay.manualMode = relayObj["manualMode"];
      relay.statePin = relayObj["statePin"];
      relay.description = relayObj["description"].as<String>();
      control.relays.push_back(relay);
    }

    // Чтение новых параметров
    control.limitWorkPressurePump = doc["limitWorkPressurePump"] | 1800;  // Значение по умолчанию
    control.limitTemperature = doc["limitTemperature"] | 5;               // Значение по умолчанию
    control.limitBigTank = doc["limitBigTank"] | 100;                     // Значение по умолчанию
    control.limitSmallTank = doc["limitSmallTank"] | 100;
    control.limitOffSmallTank = doc["limitOffSmallTank"] | 130;
    control.limitTimeWorkMainPump = doc["limitTimeWorkMainPump"] | 10800;  // Значение по умолчанию
    control.distanceBigTank = doc.containsKey("distanceBigTank") ? doc["distanceBigTank"].as<bool>() : false;
    control.isWorkInCold = doc.containsKey("isWorkInCold") ? doc["isWorkInCold"].as<bool>() : false; 

    file.close();
  }

  //============

  String sendStatus() {
    // Получаем текущий IP-адрес и SSID подключения
    String ipAddress = WiFi.localIP().toString();
    String ssid = WiFi.SSID();

    // Строка для справки с режимом работы каждого реле
    String helpText = "";
    // Добавляем информацию о состоянии каждого реле
    helpText += "Текущий статус:\n";
    int relayIndex = 1;  // Индекс для команд /on и /off
    for (const auto& relay : control.relays) {
      String commandOn = "/on" + String(relayIndex);
      String commandOff = "/off" + String(relayIndex);
      String mode_ = relay.manualMode ? "Manual" : "Auto";
      String state_ = digitalRead(relay.pin) ? "On" : "Off";
      helpText += String(commandOn + " " + commandOff + " - " + relay.description + " (" + mode_ + " / " + state_ + ")\n\n");
      relayIndex++;
    }

    helpText += "/resetManual - Сбросить в Auto\n";
    helpText += "/status - Статус и управление \n";
    helpText += "/help - Справка \n\n";

    // Добавляем IP-адрес и SSID
    helpText += "Доступ из сети: http://" + ipAddress + "\n";
    helpText += "Имя сети Wi-Fi: " + ssid + "\n\n";

    helpText += "Текущая температура: " + String(Temp) + " °С\n";
    helpText += "Текущее время работы главного насоса [" + String(timeWorkMainPump / 60) + " минут]\n";
    helpText += "Текущее значение таймера насоса давления [" + String(TimerRele2 / 60) + " минут]\n";
    helpText += "Расстояние до воды в малых баках [" + String(distanceToWaterInSmallTanks) + " см.]\n";
    helpText += (distanceToWaterInBigTank > 0 || control.distanceBigTank) ? "Расстояние до воды в большом баке [" + String(distanceToWaterInBigTank) + "см.]\n" : "";
    helpText += "Текущее значение датчика тока 1 [" + String(sensorCurrent_1) + " А]\n";
    helpText += "Текущее значение таймера работы компрессора [" + String(timerWorkСompressor) + " минут]\n";

    return helpText;
  }

  //===============================
  
  String sendHelp() {
    String helpMessage = "Доступные команды:\n";
    helpMessage += "/on1 /off1 - Скважинный насос\n";
    helpMessage += "/on2 /off2 - Насос давления\n";
    helpMessage += "/on3 /off3 - Насос перекачки\n";
    helpMessage += "/on4 /off4 - Воздушный компрессор\n";
    helpMessage += "/scriptOn /scriptOff - Сценарий продувки компрессором \n\n";
    helpMessage += "/status - Показать текущий статус\n";
    helpMessage += "/resetManual - Сбросить все реле в автоматический режим\n";
    helpMessage += "/resetFull - Сбросить все параметры\n";
    helpMessage += "/debug - Отладочная информация\n\n";
    helpMessage += "/setLimitWorkPressurePump <value> - Предел времени работы насоса давления (в сек.) [" + String(control.limitWorkPressurePump) + "]\n";
    helpMessage += "/setLimitTemperature <value> - Установить предел температуры [" + String(control.limitTemperature) + "]\n";
    helpMessage += "/setLimitBigTank <value> - Установить предел большого бака (в см.) [" + String(control.limitBigTank) + "]\n";
    helpMessage += "/setLimitSmallTank <value> - Предел включения заполнения малого бака (в см.) [" + String(control.limitSmallTank) + "]\n";
    helpMessage += "/setLimitOffSmallTank <value> - Отключения насоса давления при пределе малого бака (в см.) [" + String(control.limitOffSmallTank) + "]\n";
    helpMessage += "/setLimitTimeWorkMainPump <value> - Предел времени работы главного насоса (в сек.) [" + String(control.limitTimeWorkMainPump) + "]\n";
    helpMessage += "/setDistanceBigTank <true/false> - Срабатывать по дистанции большого бака [" + String(control.distanceBigTank ? "true" : "false") + "]\n";
    helpMessage += "/setWorkInCold <true/false> - Включать скважинныей насос при морозе [" + String(control.isWorkInCold ? "true" : "false") + "]\n";
    return helpMessage;
  }
