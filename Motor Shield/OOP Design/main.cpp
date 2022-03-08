#include "myRobot.h"
#include <Arduino_FreeRTOS.h>

 myRobot robot;
 myRobot::Encoder encoder(ENCODER1_A,ENCODER1_B);

void task1(void *arg);
void task2(void *arg);


TaskHandle_t Task_Handle1;
TaskHandle_t Task_Handle2;

void setup() 
{
 Serial.begin(9600);

 xTaskCreate(task1,"Task1",100,NULL,1,&Task_Handle1);
 xTaskCreate(task2,"Task2",100,NULL,1,&Task_Handle2); 
 vTaskStartScheduler();
}

void loop() {/*DO NOT USE*/}

void task1(void *arg)
{
  (void) arg;

  TickType_t getTick;
  getTick = xTaskGetTickCount();
  
  while(true)
  {
    robot.move("forward",100);
    vTaskDelayUntil(&getTick,5000/portTICK_PERIOD_MS);
    robot.move("reverse",100);  
  }
}


void task2(void *arg)
{
  (void) arg;

  while(true)
  {
    encoder.show();
  }     

}
