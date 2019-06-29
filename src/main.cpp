#include <Arduino.h>

class DecodeOOK {
protected:
    byte total_bits, bits, flip, state, pos, data[25];
 
    virtual char decode (word width) =0;
   
public:
 
    enum { UNKNOWN, T0, T1, T2, T3, OK, DONE };
 
    DecodeOOK () { resetDecoder(); }
 
    bool nextPulse (word width) {
        if (state != DONE)
       
            switch (decode(width)) {
                case -1: resetDecoder(); break;
                case 1:  done(); break;
            }
        return isDone();
    }
   
    bool isDone () const { return state == DONE; }
 
    const byte* getData (byte& count) const {
        count = pos;
        return data;
    }
   
    void resetDecoder () {
        total_bits = bits = pos = flip = 0;
        state = UNKNOWN;
    }
   
    // add one bit to the packet data buffer
   
    virtual void gotBit (char value) {
        total_bits++;
        byte *ptr = data + pos;
        *ptr = (*ptr >> 1) | (value << 7);
 
        if (++bits >= 8) {
            bits = 0;
            if (++pos >= sizeof data) {
                resetDecoder();
                return;
            }
        }
        state = OK;
    }
   
    // store a bit using Manchester encoding
    void manchester (char value) {
        flip ^= value; // manchester code, long pulse flips the bit
        gotBit(flip);
    }
   
    // move bits to the front so that all the bits are aligned to the end
    void alignTail (byte max =0) {
        // align bits
        if (bits != 0) {
            data[pos] >>= 8 - bits;
            for (byte i = 0; i < pos; ++i)
                data[i] = (data[i] >> bits) | (data[i+1] << (8 - bits));
            bits = 0;
        }
        // optionally shift bytes down if there are too many of 'em
        if (max > 0 && pos > max) {
            byte n = pos - max;
            pos = max;
            for (byte i = 0; i < pos; ++i)
                data[i] = data[i+n];
        }
    }
   
    void reverseBits () {
        for (byte i = 0; i < pos; ++i) {
            byte b = data[i];
            for (byte j = 0; j < 8; ++j) {
                data[i] = (data[i] << 1) | (b & 1);
                b >>= 1;
            }
        }
    }
   
    void reverseNibbles () {
        for (byte i = 0; i < pos; ++i)
            data[i] = (data[i] << 4) | (data[i] >> 4);
    }
   
    void done () {
        while (bits)
            gotBit(0); // padding
        state = DONE;
    }
};
 
// 433 MHz decoders
 
 
class OregonDecoderV2 : public DecodeOOK {
public:
    OregonDecoderV2() {}
   
    // add one bit to the packet data buffer
    virtual void gotBit (char value) {
        if(!(total_bits & 0x01))
        {
            data[pos] = (data[pos] >> 1) | (value ? 0x80 : 00);
        }
        total_bits++;
        pos = total_bits >> 4;
        if (pos >= sizeof data) {
            resetDecoder();
            return;
        }
        state = OK;
    }
   
virtual char decode(word width) 
      {
         if (200 <= width && width < 1200) 
         {
            byte w = width >= 700;

            switch (state) 
            {
               case UNKNOWN:
                  if (w != 0) 
                  {
                     // Long pulse
                     ++flip;
                  } 
                  else if (w == 0 && 24 <= flip) 
                  {
                     // Short pulse, start bit
                     flip = 0;
                     state = T0;
                  } 
                  else 
                  {
                     // Сброс декодера
                     return -1;
                  }
                  break;
               case OK:
                  if (w == 0) 
                  {
                     // короткий импульс
                     state = T0;
                  }
                  else 
                  {
                     // длинный импульс
                     manchester(1);
                  }
                  break;
               case T0:
                  if (w == 0) 
                  {
                     // второй коротуий импульс
                     manchester(0);
                  } 
                  else 
                  {
                     // Reset decoder
                     return -1;
                  }
               break;
            }
         } 
         else if (width >= 2500  && pos >= 8) 
         {
            return 1;
         } 
         else 
         {
            return -1;
         }
         return 0;
      }

};

 
OregonDecoderV2 orscV2;
 
// Объявляем переменные и константы
const byte maxSensors = 5; // Максимальное количество регистрируемых сенсоров
float pressure; // Давление
unsigned long sensorID[maxSensors]; // Массив с идентификаторами сенсоров
int sensorType[maxSensors]; // Массив с типами сенсоров
byte sensorChannel[maxSensors]; // Массив с каналами сенсоров
int sensorTemp[maxSensors]; // Массив с данными температуры от сенсоров
byte sensorTempSign[maxSensors]; //Массив с признаком знака температуры
byte sensorHum[maxSensors]; // Массив с данными влажности
byte sensorHumStr[maxSensors]; // Массив с данными об условиях
byte sensorBatteryStatus[maxSensors]; // Массив с данными о статусе батареи
byte sensorDataUpdated[maxSensors]; // Массив с признаком обновления данных
 
// Режим отладки (будет выводить на терминал)
const boolean DEBUG = false;
  
//#define R_LED 13 // data receiving LED
//unsigned long r_ledNow = 0;
//unsigned long t_ledNow = 0;
//int errorDelay = 150;
 
volatile word pulse;
 
// Обработчик прерывания при поступлении данных с радиомодуля
void ext_int_1(void) {
    static word last;
    // determine the pulse length in microseconds, for either polarity
    pulse = micros() - last;
    last += pulse;
}
 
// Функция первичной обработки данных и регистрации сенсоров
void reportSerial (const char* s, class DecodeOOK& decoder) {
    byte pos;
    const byte* data = decoder.getData(pos);
    if (DEBUG) {
      Serial.print(s);
      Serial.print(' ');
    }
   
  //  byte checksum = data[0] >> 4;
//    for (byte i = 1; i < 8; ++i) {
//        checksum += data[i] >> 4;
 //       checksum += data[i] & 0x0F;
//    }
   
    //Проверяем контрольную сумму
 //   if (checksum == data[8]) {
     
      byte currentSensor = 255;
      byte lastRegisteredSensor = -1;
      unsigned long sensID = (unsigned long) data[0] << 16;
      sensID += (unsigned long) data[1] << 8;
      sensID += (unsigned long) data[3];
     
      for (byte i = 0; i < maxSensors; ++i) {
        if (sensorID[i] == 0) {
          lastRegisteredSensor = i;
          break;
        }  
        if (sensorID[i] == sensID) {
          currentSensor = i;
          break;
        }
      }
 
      //определяем канал
      byte channel;
      switch (data[2]){
      case 0x10:
        channel = 1;
        break;
      case 0x20:
        channel = 2;
        break;
      case 0x40:
        channel = 3;
        break;
      }    
     
      // Поискали сенсор, если не нашли до регистрируем его
      
      if (currentSensor == 255) {
           Serial.print("Обнаружен новый сенсор с ID: ");
           Serial.println(sensID,HEX);
           Serial.print("Добавлен в систему с индексом: ");
           Serial.println(lastRegisteredSensor);
        }

        if(lastRegisteredSensor >= 0) {
          sensorID[lastRegisteredSensor] = sensID;
          currentSensor = lastRegisteredSensor;        
        }
        else {
         
            Serial.println("слишком много сенсоров, этот не будет добавлен");        
            return;
        }
         
      //Заносим данные с сенсора по массивам
      sensorChannel[currentSensor] = channel;
      sensorType[currentSensor] = (data[0] << 8)+data[1];
     
      switch (sensorType[currentSensor]) {
        // Для других типов сенсоров, придется тут добавить что-то своё
        case 0x1A2D:
          sensorTemp[currentSensor] = (data[5] >> 4)*100+(data[5] & 0x0F)*10+(data[4] >> 4);
          sensorTempSign[currentSensor] = 0;
          if((data[6] & 0x08) == 0x08) {
            sensorTempSign[currentSensor] = 1;
          }
          sensorHum[currentSensor] = (data[7] & 0x0F)*10+(data[6] >> 4);
          sensorHumStr[currentSensor] = data[7] >> 6;
          sensorBatteryStatus[currentSensor] = (data[4] & 0x04) >> 2;
          break;
        case 0xEA4C:
          sensorTemp[currentSensor] = (data[5] >> 4)*100+(data[5] & 0x0F)*10+(data[4] >> 4);
           if((data[6] & 0x08) == 0x08) {
            sensorTempSign[currentSensor] = 1;
          }
          sensorHum[currentSensor] = 0;
          sensorHumStr[currentSensor] = 255;

          if((data[6] & 0x08) == 0x08) {
            sensorTempSign[currentSensor] = 1;
          }
            sensorBatteryStatus[currentSensor] = (data[4] & 0x04) >> 2;
          break;
      }
     
      sensorDataUpdated[currentSensor] = 1;
         
      if (DEBUG) {
        for (byte i = 0; i < pos; ++i) {
          Serial.print(data[i] >> 4, HEX);
          Serial.print(data[i] & 0x0F, HEX);
        }
        Serial.println();
      }
   // } else {
 //     if (DEBUG) {
 //       Serial.println("Checksum failed");
 //     }
 //   }
   
    decoder.resetDecoder();
}
 
 
void reportSensorTHSerial(byte sensorNum) {
  Serial.print("   Sensor temperature: ");
  if ((sensorTempSign[sensorNum] == 0) && (sensorTemp[sensorNum] != 0)) {
    Serial.print("+");
  }
  if (sensorTempSign[sensorNum] == 1) {
    Serial.print("-");
  }
  Serial.print(sensorTemp[sensorNum]/10);
  Serial.print(".");
  Serial.print(sensorTemp[sensorNum] - int(sensorTemp[sensorNum]/10)*10);
  Serial.println("C");
         
  Serial.print("   Sensor humidity: ");
  Serial.print(sensorHum[sensorNum]);
  Serial.print("% - ");
  switch (sensorHumStr[sensorNum]) {
    case 0:
      Serial.println("normal");
      break;
    case 1:
      Serial.println("comfort");
      break;
    case 2:
      Serial.println("dry");
      break;
    case 3:
      Serial.println("wet");
      break;
    case 255:
      Serial.println("НЕТ ДАННЫХ");
      break;
      
  }
 
  if (sensorBatteryStatus[sensorNum] == 0) {        
    Serial.print("   Хороший уровень батареи / статус:");
    Serial.println(sensorBatteryStatus[sensorNum]);
  } else {
    Serial.println("   Низкий уровень батареи / статус:");
    Serial.println(sensorBatteryStatus[sensorNum]);
  }
}


// Функция вывода данных с сенсоров на терминал  
void sendDataSerial() {
 // Serial.println("=========================================");
          for (byte i = 0; i < maxSensors; ++i) {
          if ((sensorID[i] != 0) && (sensorDataUpdated[i] == 1)) {
            Serial.print("ID сенсора: ");
            Serial.println(sensorID[i],HEX);
            Serial.print("   Канал сенсора: ");
            Serial.println(sensorChannel[i],HEX);
         
            Serial.print("   Тип сенсора: ");
          // Запуск специфичных для типа сенсора функций вывода данных
            switch (sensorType[i]) {
              case 0x1A2D:
                Serial.println("THGN-132N");
                reportSensorTHSerial(i);
                break;
              case 0xEA4C:
                Serial.println("THN-132N");
                reportSensorTHSerial(i);
                break;
              default:
                Serial.println("Сенсор не подключен");
            }
          }
          sensorDataUpdated[i] = 1;
        }
        Serial.println("=========================================");
}
 
void setup () {
  pinMode(2, INPUT);
  Serial.begin(9600);
  Serial.println("\n[Arduino Oregon Data v1.0]");
   
  attachInterrupt(digitalPinToInterrupt(2), ext_int_1, CHANGE);

 
   
   // Cleaning array with registered sensors
   for (byte i = 0; i < maxSensors; ++i) {  
     sensorID[i] = 0;
     sensorDataUpdated[i] = 0;
   }
  // pinMode(R_LED, OUTPUT);
 //    digitalWrite(13, LOW);
}
 
void loop () {
   
    //static int i = 0;
    cli();
    word p = pulse;
    pulse = 0;
    sei();
   
    if (p != 0) {
        //reportSerial("OSV2", orscV2);  
        if (orscV2.nextPulse(p)) {
            sendDataSerial();
            reportSerial("OSV2", orscV2);  
   //         r_ledNow = millis()+200;
  //          digitalWrite(R_LED, HIGH);
        } 
    }
 
 //   if (millis() >= r_ledNow) {
 //     digitalWrite(R_LED, LOW);
 //   }
//   if (millis() >= t_ledNow) {
 //   }
}
 