#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <semphr.h>  
#include <dht11.h>
#include <SPI.h>
#include <LoRa.h>
#include <string.h>

dht11 DHT;
#define DHT11_PIN 4
#define alarmPin 13
#define interruptPin 3
#define USE_AVG
#define sharpLEDPin  7   //sensor LED.
#define sharpVoPin   A5   //sensor Vo.
#define N 100
static unsigned long VoRawTotal = 0;
static int VoRawCount = 0; 
static float Voc = 0.6;   
const float K = 0.5;

const TickType_t xTicksToWait = 100 / portTICK_PERIOD_MS;


SemaphoreHandle_t xSerialSemaphore;
SemaphoreHandle_t xBinarySemaphore;

QueueHandle_t xQueue;
QueueHandle_t xQueue2;


void TaskDHT11( void *pvParameters );
void TaskMQ9( void *pvParameters );
void TaskDUST( void *pvParameters );
void TaskLORA( void *pvParameters);
void TaskAlarm(void *pvParameters);  
// void TaskLCD( void *pvParameters );
void TaskDEBUG( void *pvParameters );

static void vHandlerTask( void *pvParameters );
static void vExampleInterruptHandler( void ); 
boolean state = false;
void setup() {
  pinMode(sharpLEDPin, OUTPUT);

  Serial.begin(9600);
  pinMode(9, OUTPUT);
  LoRa.setPins(53, 9, 2);
  vSemaphoreCreateBinary( xBinarySemaphore );

  pinMode(alarmPin, OUTPUT);
  pinMode(interruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), vExampleInterruptHandler, CHANGE);  

  //SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));  Serial.println("LoRa Sender");

  while (!Serial);

  if ( xSerialSemaphore == NULL ) 
  {
    xSerialSemaphore = xSemaphoreCreateMutex();  
      xSemaphoreGive( ( xSerialSemaphore ) ); 
  }

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  delay(1000);

  LoRa.setSpreadingFactor(8);
  LoRa.setSignalBandwidth(62.5E3);
  LoRa.crc();

  xQueue = xQueueCreate( 3, sizeof( float ) );
  xQueue2 = xQueueCreate( 1, sizeof( float ) );

  if ( xQueue != NULL ) {
    xTaskCreate(
      TaskMQ9
      ,  (const portCHAR *) "ReadMQ9"
      ,  200  
      ,  NULL
      ,  1  
      ,  NULL );

    xTaskCreate(
      TaskDHT11
      ,  (const portCHAR *)"ReadDHT11"  
      ,  200  
      ,  NULL
      ,  2  
      ,  NULL );
    xTaskCreate(
      TaskDUST
      ,  (const portCHAR *) "ReadDustSensor"
      ,  1000  
      ,  NULL
      ,  3  
      ,  NULL );

    xTaskCreate(
      TaskLORA
      ,  (const portCHAR *) "Lora"
      ,  200  
      ,  NULL
      ,  4  
      ,  NULL );
    xTaskCreate(  vHandlerTask, "Handler", 200, NULL, 5, NULL ); // ISR
    }
  else
  {
    /* The queue could not be created. */
  }
}

void loop()
{
}


void TaskDHT11( void *pvParameters  )  
{
  TickType_t xLastWakeTime;
  portBASE_TYPE xStatus;
  xLastWakeTime = xTaskGetTickCount();
  for (;;) 
  {
    int chk = DHT.read(DHT11_PIN);   

    // lcd.setCursor(13,0);
    // lcd.print(DHT.temperature);
    // lcd.setCursor(13,1);
    // lcd.print(DHT.humidity);
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      switch (chk) {
        case DHTLIB_OK:
          Serial.println("DOING...\t");
          break;
        case DHTLIB_ERROR_CHECKSUM:
          Serial.println("Checksum error,\t");
          break;
        case DHTLIB_ERROR_TIMEOUT:
          Serial.println("Time out error,\t");
          break;
        default:
          Serial.println("Unknown error,\t");
          break;
      }
      // Serial.print("xLastWakeTime DHT: ");
      // Serial.println(xLastWakeTime);
      Serial.print("Humidity: ");
      Serial.println(DHT.humidity);
      Serial.print("Temperature: ");
      Serial.println(DHT.temperature);
      xStatus = xQueueSendToBack( xQueue, &DHT.humidity, 0 );
      xStatus = xQueueSendToBack( xQueue, &DHT.temperature, 0 );
      if ( xStatus != pdPASS ) Serial.println( "Could not send to the queue" );

      xSemaphoreGive( xSerialSemaphore ); 
    }
    vTaskDelayUntil( &xLastWakeTime, 625 );
  }
}

void TaskMQ9( void *pvParameters  )  
{
  TickType_t xLastWakeTime;
  portBASE_TYPE xStatus;
  xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    float sensor_volt;
    float RS_gas; // Get value of RS in a GAS
    float ratio; // Get ratio RS_GAS/RS_air
    int sensorValue = analogRead(A0);
    sensor_volt = (float)sensorValue / 1024 * 5.0;
    RS_gas = (5.0 - sensor_volt) / sensor_volt; // omit *RL
    ratio = RS_gas/0.29;  
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {

      Serial.print("CO = ");
      Serial.print(ratio);
      Serial.print("\t");      
      Serial.println(sensor_volt);      

      xStatus = xQueueSendToBack( xQueue, &ratio, 0 );
      if( xStatus != pdPASS ) Serial.println( "Could not send to the queue" );

      xSemaphoreGive( xSerialSemaphore );
    }
    vTaskDelayUntil( &xLastWakeTime, 625 );

  }
}

void TaskDUST( void *pvParameters )  
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  portBASE_TYPE xStatus;
  for (;;) {
    digitalWrite(sharpLEDPin, LOW);
    vTaskDelay( 280 / portTICK_PERIOD_MS );
    int VoRaw = analogRead(sharpVoPin);
    digitalWrite(sharpLEDPin, HIGH);
    vTaskDelay( 9620 / portTICK_PERIOD_MS );
    float Vo = VoRaw;
    Vo = Vo / 1024.0 * 5.0; // VOTLS :))
    float dustDensity = 0.17 * Vo - 0.1;
    if ( dustDensity < 0)
    {
      dustDensity = 0.00;
    }

    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      Serial.print("Dust density: ");
      Serial.print(dustDensity);
      Serial.println(" ug/m3");

      xStatus = xQueueSendToBack( xQueue2, &dustDensity, 0 );
      xSemaphoreGive( xSerialSemaphore );
    }
  }
}

void TaskDEBUG( void *pvParameters )  
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  for (;;) 
  {
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      xSemaphoreGive( xSerialSemaphore ); 
    }
    vTaskDelayUntil( &xLastWakeTime, ( 10000 / portTICK_PERIOD_MS ) );
  }
}

void TaskLORA( void *pvParameters)  
{
  String dataPackage = "";
  float h, t, c, dust;
  char* arrData;
  portBASE_TYPE xStatus;
  for (;;)
  {
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {

      if ( uxQueueMessagesWaiting (xQueue) == 3 && uxQueueMessagesWaiting (xQueue2) == 1) {
        xStatus = xQueueReceive( xQueue, &h, 0 );
        LoRa.beginPacket();
        LoRa.print('1');
        LoRa.print('\n');
        LoRa.print(h);
        LoRa.print('\n');
        xStatus = xQueueReceive( xQueue, &t, 0 );
        LoRa.print(t);
        LoRa.print('\n');       
        xStatus = xQueueReceive( xQueue, &c, 0 );
        LoRa.print(c);
        LoRa.print('\n');       
        xStatus = xQueueReceive( xQueue2, &dust, 0 );
        LoRa.print(dust);
        LoRa.print('\n');
        LoRa.endPacket();        
  
        Serial.print("\nQueue received: ");
        Serial.print(h);
        Serial.print(t);
        Serial.print(c);
        Serial.println(dust);
        Serial.println("_______________________________");        
      }
      xSemaphoreGive( xSerialSemaphore ); 
    }
    vTaskDelay( 1000 / portTICK_PERIOD_MS );
  }
}

static void vHandlerTask( void *pvParameters )
{
  xSemaphoreTake( xBinarySemaphore, 0);

  for( ;; )
  {
    xSemaphoreTake( xBinarySemaphore, portMAX_DELAY );
    if(state){
        LoRa.beginPacket();
        LoRa.print('0');
        LoRa.endPacket(); 
        state = false;
    }
  }
}

static void  vExampleInterruptHandler( void ){
  portBASE_TYPE xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;

  xSemaphoreGiveFromISR( xBinarySemaphore, (signed portBASE_TYPE*)&xHigherPriorityTaskWoken );

  if( xHigherPriorityTaskWoken == pdTRUE )
  {
    state = true;
    vPortYield();
  }
}

























































































































































































































