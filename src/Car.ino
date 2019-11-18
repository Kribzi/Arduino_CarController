#include <Arduino_FreeRTOS.h>
#include <IRremote.h>

#include <semphr.h>


const int Echo = 11;
const int Trig = 10;
const int RECV_PIN = 8;

const int leftTireSpeed = 5;
const int rightTireSpeed = 6;
const int leftTireForward = 4;
const int leftTireBackward = 7;
const int rightTireForward = 3;
const int rightTireBackward = 9;

SemaphoreHandle_t distanceMutex = NULL;



unsigned long distance;
unsigned int tireSpeed = 255;

IRrecv receiver(RECV_PIN);
decode_results results;

#define POWER 0x00FF629D
#define A 0x00FF22DD
#define B 0x00FF02FD
#define C 0x00FFC23D
#define UP 0x00FF9867
#define DOWN 0x00FF38C7
#define LEFT 0x00FF30CF
#define RIGHT 0x00FF7A85
#define SELECT 0x00FF18E7
#define pdMS_TO_TICKS( xTimeInMs ) ( ( TickType_t ) ( ( ( unsigned long ) ( xTimeInMs ) * ( TickType_t )configTICK_RATE_HZ ) / ( TickType_t ) 1000 ) )



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(Echo,INPUT);
  pinMode(Trig,OUTPUT);
  pinMode(leftTireSpeed,OUTPUT);
  pinMode(rightTireSpeed,OUTPUT);
  pinMode(rightTireForward,OUTPUT);
  pinMode(rightTireBackward,OUTPUT);
  pinMode(leftTireForward,OUTPUT);
  pinMode(rightTireForward,OUTPUT);

  

  if(xTaskCreate(DistanceSensorTask,"T1",128,NULL,1,NULL)== pdFAIL){
    Serial.print("Failed to create DistanceSensorTask");
  }

  if(xTaskCreate(RemoteControllerTask,"R",128,NULL,0,NULL)== pdFAIL){
    Serial.print("Failed to create RemoteControllerTask");
  }

  if(xTaskCreate(CarController,"CC",128,NULL,0,NULL)== pdFAIL){
    Serial.print("Failed to create CarControllerTask");
  }

  distanceMutex = xSemaphoreCreateMutex();
  if(distanceMutex == NULL){
    Serial.print("Mutex not created successfully");
  }

  

 receiver.enableIRIn();
 receiver.blink13(true);
  
  

}

void DistanceSensorTask(void * param){
   for(;;){
     digitalWrite(Trig,LOW);
    delayMicroseconds(2);
    digitalWrite(Trig,HIGH);
    delayMicroseconds(10);
    digitalWrite(Trig,LOW);

    xSemaphoreTake(distanceMutex, portMAX_DELAY);
    distance = pulseIn(Echo,HIGH)/2.0*0.034;
    xSemaphoreGive(distanceMutex);
    
    Serial.print("distance ");
    Serial.println(distance,DEC);
  
   
    vTaskDelay(pdMS_TO_TICKS(150));
   }
  
 
}


void RemoteControllerTask(void * param){
  for(;;){
    if(receiver.decode(&results)){

      
      receiver.resume();
    }
    else{
      results.value = 0;
    }
    //Serial.print(results.value,HEX);
    vTaskDelay(pdMS_TO_TICKS(150));
  } 
}

void CarController(void * param){
  for(;;){
    switch(results.value){
          case A:
            tireSpeed = 255*0.3;
            break;
          case B:
            tireSpeed = 255 * 0.6;
            break;
          case C:
            tireSpeed = 255;
            break;
          case UP:
             xSemaphoreTake(distanceMutex, portMAX_DELAY);
             if (distance <= 20){
              xSemaphoreGive(distanceMutex);
              break;
             }
             xSemaphoreGive(distanceMutex);
             analogWrite(leftTireSpeed,tireSpeed);
             analogWrite(rightTireSpeed,tireSpeed);
             digitalWrite(leftTireForward,HIGH);
             digitalWrite(leftTireBackward,LOW);
             digitalWrite(rightTireForward,HIGH);
             digitalWrite(rightTireBackward,LOW);
            break;
          case DOWN:
            analogWrite(leftTireSpeed,tireSpeed);
            analogWrite(rightTireSpeed,tireSpeed);
            digitalWrite(leftTireForward,LOW);
            digitalWrite(leftTireBackward,HIGH);
            digitalWrite(rightTireForward,LOW);
            digitalWrite(rightTireBackward,HIGH);
            break;
          case LEFT:
            analogWrite(leftTireSpeed,tireSpeed/2);
            analogWrite(rightTireSpeed,tireSpeed);
            digitalWrite(leftTireForward,HIGH);
            digitalWrite(leftTireBackward,LOW);
            digitalWrite(rightTireForward,HIGH);
            digitalWrite(rightTireBackward,LOW);
            break;
          case RIGHT:
            xSemaphoreGive(distanceMutex);
            analogWrite(leftTireSpeed,tireSpeed);
            analogWrite(rightTireSpeed,tireSpeed/2);
            digitalWrite(leftTireForward,HIGH);
            digitalWrite(leftTireBackward,LOW);
            digitalWrite(rightTireForward,HIGH);
            digitalWrite(rightTireBackward,LOW);
            break;
  
          case 0xFFFFFFFF:
            xSemaphoreTake(distanceMutex, portMAX_DELAY);
            if (distance <= 20){
              xSemaphoreGive(distanceMutex);
              analogWrite(leftTireSpeed,0);
              analogWrite(rightTireSpeed,0);

              break;
             }
            xSemaphoreGive(distanceMutex);
            analogWrite(leftTireSpeed,tireSpeed);
            analogWrite(rightTireSpeed,tireSpeed);
            break;
          default:
           analogWrite(leftTireSpeed,0);
           analogWrite(rightTireSpeed,0);
           break;
        }
    vTaskDelay(pdMS_TO_TICKS(150));
  }
}


void loop() {
  

}
