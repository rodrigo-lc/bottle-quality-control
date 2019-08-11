/* ----------------------------------------------------------------------------
   Project:     Bottle Quality Control
   Description: A system for handling four stages of a 1,5L grape juice bottle
   File:        main.c
   Author:      Rodrigo Luiz da Costa
   Created:		  21/05/2019
   Modified:	  30/06/2019
   Version:     1.0
   Purpose:     Final project for the Projeto Integrador II class in IFSC/DAELN
   ------------------------------------------------------------------------- */

/* ----------------------------------------------------------------------------
 						              		Includes and defines
   --------------------------------------------------------------------------*/
#define BIT_OCCUPANCY_SENSOR_1 2
#define BIT_OCCUPANCY_SENSOR_2 3
#define BIT_OCCUPANCY_SENSOR_3 4
#define BUZZER 5
#define LCD_ADDRESS 0x3F

#define ITERATIONS    20 // Number of iterations. INICIALMENTE 5
#define TRIGGER_PIN   12 // Arduino pin tied to trigger pin on ping sensor.
#define ECHO_PIN      11 // Arduino pin tied to echo pin on ping sensor.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about

#define COLOR_S0  9
#define COLOR_S1  8
#define COLOR_S2  7
#define COLOR_S3  6
#define COLOR_OUT A0

#define RED_CALIBRATION 420 // 0 to 255
#define GREEN_CALIBRATION 505 // 0 to 255
#define BLUE_CALIBRATION 350 // 0 to 255

#define DHTPIN 10
#define DHTTYPE DHT22  

#include <acpr.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <NewPing.h>

/* ----------------------------------------------------------------------------
 						              		 Globals 
   --------------------------------------------------------------------------*/

/* Serial packages */
/* Barcode: | STATE | SIZE | TYPE | BARCODE_NUMBER | */
typedef struct {
  String state, size, type, value;
} barcode_t;
/* OCR: | STATE | SIZE | STRING | */
typedef struct {
  String state, size, reg_map;
} ocr_t;
typedef struct {
  barcode_t barcode;
  ocr_t ocr;
} serial_package_t;
/* Production state machine */
typedef enum production_state_machine {
  START,
  HEIGHT,
  CTU,
  BARCODE,
  OCR,
  INVALID = 0xFF,
} production_state_machine_t;
/* Flags */
typedef struct {
  union {
    struct {
      uint8_t flag_isr_1s : 1;
      uint8_t flag_5s : 1;
      uint8_t flag_test : 1;
    };
    uint8_t all_flags : 8;
  };
} system_status_t;

/* Height variables and functions */
unsigned long pingTimer[ITERATIONS]; // Holds the times when the next ping should happen for each iteration.
unsigned int cm[ITERATIONS];         // Where the ping distances are stored.
uint8_t currentIteration = 0;        // Keeps track of iteration step.
void echoCheck(void);
uint32_t oneSensorCycle(void); 
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins

/* Color sensor variables and functions */
static void color(void);
static int red = 0;
static int green = 0;
static int blue = 0;

/* General variables */
static volatile system_status_t system_status;
static serial_package_t serial_package;
static production_state_machine_t production_state_machine = START; // Estado de reset = start
static void buzzer_beep(uint8_t n);
static volatile uint8_t occupancy_state_new;
static volatile uint8_t occupancy_state_old;
uint16_t seconds_counter_5s;
uint8_t seconds_counter_3s;
uint32_t distance_cm = 0;

/* LCD global */
LiquidCrystal_I2C lcd(LCD_ADDRESS,16,2);  // set the LCD address to 0x3F for a 16 chars and 2 line display

/* DHT global */
DHT dht(DHTPIN, DHTTYPE);

/* ----------------------------------------------------------------------------
 						              		Setup - Runs once
   --------------------------------------------------------------------------*/
void setup()
{
  // Initialize timer1
  noInterrupts();             // disable all interrupts
  TCCR1A = 0x00;
  TCCR1B = 0x00;
  TCNT1  = 0x00;
  OCR1A = 15624;              // compare match register 16MHz/1024 x15624 = 1Hz     0x3D08
  TCCR1B |= (1 << WGM12) | (1 << CS10) | (1 << CS12);      // CTC mode and 1024 prescaler
  TIMSK1 |= (1 << OCIE1A);    // enable timer compare interrupt
  interrupts();               // enable all interrupts
  
  /* Pin modes */
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(BIT_OCCUPANCY_SENSOR_1, INPUT);
  pinMode(BIT_OCCUPANCY_SENSOR_2, INPUT);
  pinMode(BIT_OCCUPANCY_SENSOR_3, INPUT);
  /* Color */
  pinMode(COLOR_S0, OUTPUT);
  pinMode(COLOR_S1, OUTPUT);
  pinMode(COLOR_S2, OUTPUT);
  pinMode(COLOR_S3, OUTPUT);
  pinMode(COLOR_OUT, INPUT);
  digitalWrite(COLOR_S0, HIGH);
  digitalWrite(COLOR_S1, LOW);

  /* Begin serial with 9600 */
  Serial.begin(9600);

  /* LCD */
  lcd.init();                      // initialize the lcd 
  lcd.backlight();
  lcd.clear();
  
  /* DHT */
  dht.begin();

  /* Ultrassonic */
  pingTimer[0] = millis() + 75;            // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < ITERATIONS; i++)  {// Set the starting time for each iteration.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
  }

  /* INIT MESSAGE */
  //Serial.println("BOTTLE QUALITY CONTROL V2 - Rodrigo Belisário e Rodrigo Costa\r\n");
 // Serial.println("Sensors: OCCUPANCY, HEIGHT, COLOR, TEMPERATURE, UMIDITY, BARCODE and OCR");
  //Serial.println("Actuators: DISPLAY LCD, SERIAL");
  //Serial.println("Copyright (c) <2019> <RODRIGO COSTA> - MIT Licence\r\n"); // 2 lines down
  //Serial.print("Initializing... "); 
  lcd.setCursor(0,0);
  lcd.println("BQC System      ");
  lcd.setCursor(0,1);
  lcd.print("PI 2 - 2019/2    ");
  digitalWrite(BUZZER, HIGH);
  for(uint8_t i = 0; i < 7; i++) {
    lcd.noBacklight();
    Serial.print("... ;");
    delay(200);
    lcd.backlight();
    Serial.print("... ;");
    delay(200);
  }
  digitalWrite(BUZZER, LOW);
  Serial.print(";");
}

/* ----------------------------------------------------------------------------
 						              		Main loop
   --------------------------------------------------------------------------*/
void loop()
{
  /* State machine controlled by occupancy sensors */
  switch (production_state_machine)
  {
    /* ----------------------------------------------------------------------------
 						              		START STATE - Waits for first sensor
    --------------------------------------------------------------------------*/
    case START:
      lcd.clear();
      lcd.print("STATE: START");
      lcd.setCursor(0,1);
      lcd.print("Insert bottle...");
      Serial.print("STATE: START;");
      for (;;) {
          if(system_status.flag_isr_1s) {
            system_status.flag_isr_1s = 0;
            seconds_counter_3s++;
            if(seconds_counter_3s >= 3) {
         
              buzzer_beep(1);
              seconds_counter_3s = 0;
            }
          }
          
        /* Start state, waiting for a bottle in the first sensor */
        if (occupancy_state_new == 0b001 && system_status.flag_test) {
          production_state_machine = HEIGHT;
          break;
        }
      }
      break;
    /* ----------------------------------------------------------------------------
 						        		HEIGHT STATE - Read height and request img proc. height
    --------------------------------------------------------------------------*/
    case HEIGHT:
      buzzer_beep(3);
      lcd.clear();
      lcd.print("STATE: HEIGHT");
      lcd.setCursor(0,1);
      lcd.print("Bottle in P1");
      Serial.print("STATE: HEIGHT;");
      
      for (;;) {
        /* Height sensor - JSN-SR04T */
        distance_cm = 0;
        for (uint8_t i = 0; i < ITERATIONS; i++) { // Loop through all the iterations.
          if (millis() >= pingTimer[i]) {          // Is it this iteration's time to ping?
            pingTimer[i] += PING_INTERVAL * ITERATIONS; // Set next time this sensor will be pinged.
            if (i == 0 && currentIteration == ITERATIONS - 1) {
              distance_cm = oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
              Serial.print(distance_cm);
              Serial.print(" cm;");
            }
            sonar.timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
            currentIteration = i;        // Sensor being accessed.
            cm[currentIteration] = 0;    // Make distance zero in case there's no ping echo for this iteration.
            sonar.ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).

            
            /* Parking sensor beep logic */
          }
        }
      
         
        if(distance_cm == 28) { // SOMENTE SE IGUAL A 27cm
          lcd.setCursor(0,1);
          lcd.print("VOLUME OK!      ");
          digitalWrite(BUZZER, HIGH);
          delay(2000);
          digitalWrite(BUZZER, LOW);
          delay(500);

          Serial.print("HEIGHT SEND;");
          Serial.print(distance_cm); Serial.print(";"); Serial.print("OK;");
          Serial.print("HEIGHT REQUEST;");
          lcd.setCursor(0,0);
          lcd.print("STATE: RQ HEIGHT");
          delay(1000);
         // buzzer_beep(3);
          String height_request_string;
          String height_size_string;
          String height_height_string;
          for (;;) {
            if (Serial.available() > 0) {
              height_request_string = Serial.readStringUntil(';');
              if ( height_request_string == "HEIGHT OK"){
                buzzer_beep(3);
                height_size_string = Serial.readStringUntil(';');  
                height_height_string = Serial.readStringUntil(';'); 
                break;
              }
              else{
                Serial.print("Height reading with problems;");
              }
            }
          }
          distance_cm = 0;
          goto jump;  // don't blane me, less than 1 hours to find and kill the bug. Obviously I didn't.
          //
          //production_state_machine = CTU; 
          //break;
        }  
      }
      break;
    /* ----------------------------------------------------------------------------
 						        		BARCODE/COLOR/TEMPERATURE/UMIDITY STATE - Read temp, umidity and color, send all to GUI
    --------------------------------------------------------------------------*/
    case CTU:
      jump:
      float h;
      float t;
      lcd.setCursor(0,1);
      lcd.print("Change to P2");
      while(!(occupancy_state_new == 0b010));
      buzzer_beep(3);
      lcd.clear();
      lcd.print("STATE: C/T/U");
      Serial.print("STATE: C/T/U;");
      for (;;) {
        /* -------------- TEMPERATURE - DHT22 ---------------------------*/
        // Leitura da umidade
        h = dht.readHumidity();
        // Leitura da temperatura (Celsius)
        t = dht.readTemperature();
       
         // Verifica se o sensor esta respondendo
        if (isnan(h) || isnan(t)) {
          Serial.print("Falha ao ler dados do sensor DHT!;");
          //return;
        }
        // Mostra a temperatura no serial monitor e no display
        //Serial.print("Temperatura: "); 
        //Serial.print(t);
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print(t);
        lcd.print(" *C ");
        //Serial.print(" *C  ");
        // Mostra a umidade no serial monitor e no display
        //Serial.print("Umidade: "); 
        //Serial.print(h);
        //Serial.println(" %");
        //delay(1000);
        lcd.print(h);
        lcd.println(" %  ");

    
        
        /* -------------- COLOR SENSOR - TCS230 ---------------------------*/
        
        // Loop de leitura de cor
        //for(;;) {
          color(); //Chama a rotina que le as cores  
          //Mostra no serial monitor os valores detectados  
          /* 
          Serial.print("    R: ");  
          Serial.print(red, DEC);  
          Serial.print("    G: ");  
          Serial.print(green, DEC);  
          Serial.print("    B: ");  
          Serial.print(blue, DEC);  
          */
          lcd.setCursor(0,1);
          lcd.print(red, DEC); 
          lcd.print(" "); 
          lcd.print(green, DEC);  
          lcd.print(" "); 
          lcd.print(blue, DEC);  
          lcd.print(" "); 
          
          // Wait 2s (time for DHT22 do the conversions)
          delay(2000);
          // Verifica se a cor foi detectada de acordo com faixa de calibração

          /* Color send */
          Serial.print("COLOR RGB SEND;"); 
          Serial.print(red, DEC); Serial.print(";");
          Serial.print(green, DEC); Serial.print(";");
          Serial.print(blue, DEC); Serial.print(";");
          if (red <= (RED_CALIBRATION+50) && red >= (RED_CALIBRATION-50)) {  
            if(green <= (GREEN_CALIBRATION+50) && green >= (GREEN_CALIBRATION-50)) {
              if(blue <= (BLUE_CALIBRATION+50) && blue >= (BLUE_CALIBRATION-50)) {
                digitalWrite(BUZZER, HIGH);
                Serial.print("OK;");
                
                lcd.clear();
                lcd.print("STATE: COR");
                lcd.setCursor(0,1);
                lcd.print("COR OK!");
                delay(2000);
                digitalWrite(BUZZER, LOW);

           
                /* Barcode request state */
                production_state_machine = BARCODE;
                lcd.setCursor(0,1);
                lcd.print("                         ");
                lcd.setCursor(0,1);
                lcd.print("Change to P3  ");
                
                while(!(occupancy_state_new == 0b100));
                production_state_machine = BARCODE;
                system_status.flag_test = 0;
                
               goto barcode_goto;   // don't blane me, less than 1 hours to find and kill the bug. Obviously I didn't.
          } } }  
          else {
              Serial.print("NOK;");
          } 
          /* Temperature send */
          Serial.print("TEMPERATURE UMIDITY SEND;");
          Serial.print(t); Serial.print(";"); 
          Serial.print(h); Serial.print(";");
      }
      break;
    /* ----------------------------------------------------------------------------
 						  	READ BARCODE STATE - Request barcode number and put it in serial
    --------------------------------------------------------------------------*/
    case BARCODE:
      barcode_goto:
      buzzer_beep(3);
      lcd.clear();
      lcd.print("STATE: BARCODE");
      lcd.setCursor(0,1);
      lcd.print("Requesting...");
      Serial.print("STATE: BARCODE;");
      Serial.print("BARCODE REQUEST;"); 
      for (;;) {
        /* Package */
        //|        Estado            |   Qtd caracteres mensagem     |  Tipo do código(até 10 caracteres)   |      Código de barras     |
        //| "BARCODE OK" ou "ERRO" ; |                              ;|            "XXXXXXXXXXX"           ; |                          ;|              
        //|                          |                               |                                      |                           |
        /* Waits from BQC GUI to send data */
        if (Serial.available() > 0) {
          serial_package.barcode.state = Serial.readStringUntil(';');
          if (serial_package.barcode.state != "BARCODE OK") {
            if (serial_package.barcode.state == "BARCODE NOK") {
              Serial.print("Erro na leitura do código de barras;");
              //Serial.print("BARCODE REQUEST;"); // Pede novamente
              lcd.setCursor(0,1);
              lcd.print("Verify bottle!   ");
            }
            else {
              Serial.print("Erro desconhecido, tente reconectar PC a serial;");
            }
            break;
          }
          else if (serial_package.barcode.state == "BARCODE OK") {
            /* Read data from BQC GUI */
            serial_package.barcode.size = Serial.readStringUntil(';');
            serial_package.barcode.type = Serial.readStringUntil(';');
            serial_package.barcode.value = Serial.readStringUntil(';');

            /* Print barcode serial package */
            Serial.print(serial_package.barcode.size);
            Serial.print(" | ");
            Serial.print(serial_package.barcode.type);
            Serial.print(" | ");
            Serial.print(serial_package.barcode.value);
            Serial.print(" | ");
            Serial.print(";");
            
            // lcd barcode
            lcd.clear();
            lcd.print(serial_package.barcode.value);
            lcd.setCursor(0,1);
            lcd.print(serial_package.barcode.type);
            
            buzzer_beep(3);
            delay(4000);
            goto ocr_jump;   // don't blane me, less than 1 hours to find and kill the bug. Obviously I didn't.
            //break;
          }
        }   
      }

        //break;
    /* ----------------------------------------------------------------------------
 						  	READ OCR STATE - Request OCR string and put it in serial
    --------------------------------------------------------------------------*/
    case OCR:
      ocr_jump:
      buzzer_beep(3);
      lcd.clear();
      lcd.print("STATE: OCR    ");
      lcd.setCursor(0,1);
      lcd.print("Requesting... ");
      Serial.print("STATE: OCR;");
      Serial.print("OCR REQUEST;");
      for (;;) {
        if (Serial.available() > 0) {
          serial_package.ocr.state = Serial.readStringUntil(';');
          if (serial_package.ocr.state != "OCR OK") {
            if (serial_package.ocr.state == "OCR NOK") {
              Serial.print("Erro na leitura OCR;");
              serial_package.ocr.size = Serial.readStringUntil(';'); // Dummy read
              serial_package.ocr.reg_map = Serial.readStringUntil(';'); // Dummy read
              Serial.print("OCR REQUEST;"); // Acontece novamente
              continue;
            }
            else {
              serial_package.ocr.size = Serial.readStringUntil(';'); // Dummy read
              serial_package.ocr.reg_map = Serial.readStringUntil(';'); // Dummy read
              Serial.print("Erro desconhecido, tente reconectar PC a serial;");
              Serial.print("OCR REQUEST;"); // Acontece novamente
              continue;
            }
          }
          /* Read OCR data from BQC GUI */
          serial_package.ocr.size = Serial.readStringUntil(';');
          serial_package.ocr.reg_map = Serial.readStringUntil(';');

          /* Print OCR serial package */
          Serial.print(serial_package.ocr.size);
          Serial.print(" | ");
          Serial.print(serial_package.ocr.reg_map);
          Serial.print(" | ");
          Serial.print(";");
          lcd.clear();
          lcd.print(serial_package.ocr.size);
          lcd.setCursor(0,1);
          lcd.print(serial_package.ocr.reg_map);
          buzzer_beep(3);
          break;
        }
      }

      /* -------------------------- ENDING SUBSTATE --------------------------------------- */
     
      while(!(occupancy_state_new == 0b000));
      buzzer_beep(3);
      /* LCD end animation + beep */
      lcd.clear();
      lcd.print("  GRAN LEGADO  ");
      lcd.setCursor(0,1);
      lcd.print("FIM DO PROCESSO");
      
      delay(500);
      digitalWrite(BUZZER, LOW);

      for(uint8_t i = 0; i < 10; i++){
        lcd.noBacklight();
        delay(200);
        lcd.backlight();
        delay(200);
      }
      // Do something to finish
      delay(1000);
      /* Return */
      production_state_machine = START;
      break;
      /* ----------------------------------------------------------------------------
                      DEFAULT STATE - Error
    --------------------------------------------------------------------------*/
    /* 
    default:
      Serial.print("ERRO: Estado inválido, reinicie o sistema!");
      for(;;);
      break;
    */
      /* ---------------------------------------------------------------------------------------- */
  }  /* switch - Finite State Machine */
} /* void loop */











  /* -------------------------------------------------------------------------
                        Function definitions
  --------------------------------------------------------------------------*/
void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar.check_timer())
    cm[currentIteration] = sonar.ping_result / US_ROUNDTRIP_CM;
}
uint32_t oneSensorCycle() { // All iterations complete, calculate the median.
  unsigned int uS[ITERATIONS];
  uint8_t j, it = ITERATIONS;
  uS[0] = NO_ECHO;
  for (uint8_t i = 0; i < it; i++) { // Loop through iteration results.
    if (cm[i] != NO_ECHO) { // Ping in range, include as part of median.
      if (i > 0) {          // Don't start sort till second ping.
        for (j = i; j > 0 && uS[j - 1] < cm[i]; j--) // Insertion sort loop.
          uS[j] = uS[j - 1];                         // Shift ping array to correct position for sort insertion.
      } else j = 0;         // First ping is sort starting point.
      uS[j] = cm[i];        // Add last ping to array in sorted position.
    } else it--;            // Ping COLOR_OUT of range, skip and don't include as part of median.
  }
  return (uS[it >> 1]);
}
static void color(void)
{
  //Rotina que le o valor das cores
  digitalWrite(COLOR_S2, LOW);
  digitalWrite(COLOR_S3, LOW);
  //count OUT, pRed, RED
  red = pulseIn(COLOR_OUT, digitalRead(COLOR_OUT) == HIGH ? LOW : HIGH);
  digitalWrite(COLOR_S3, HIGH);
  //count OUT, pBLUE, BLUE
  blue = pulseIn(COLOR_OUT, digitalRead(COLOR_OUT) == HIGH ? LOW : HIGH);
  digitalWrite(COLOR_S2, HIGH);
  //count OUT, pGreen, GREEN
  green = pulseIn(COLOR_OUT, digitalRead(COLOR_OUT) == HIGH ? LOW : HIGH);
}

/* Beeps buzzer N times */
static void buzzer_beep(uint8_t n)
{
  uint8_t i;
  for (i = 0; i < n; i++) {
    digitalWrite(BUZZER, HIGH);
    delay(30);
    digitalWrite(BUZZER, LOW);
    delay(200);
  }
}











  /* -------------------------------------------------------------------------
              Interrupts - TIMER1 - 1s timebase
  --------------------------------------------------------------------------*/
/* TIMER1 COMPA CTC Mode - 1s */
ISR(TIMER1_COMPA_vect)
{
  /* Production state control */
  occupancy_state_old = occupancy_state_new;
  occupancy_state_new = 0;
  if (!digitalRead(BIT_OCCUPANCY_SENSOR_1)) {
    occupancy_state_new += 1; /* 0b001 */
  }
  if (!digitalRead(BIT_OCCUPANCY_SENSOR_2)) {
    occupancy_state_new += 2; /* 0b010 */
  }
  if (!digitalRead(BIT_OCCUPANCY_SENSOR_3)) {
    occupancy_state_new += 4; /* 0b100 */
  }
    
  switch (occupancy_state_new) {
    case 0b001:
    case 0b010:
    case 0b100:
      if (occupancy_state_new != occupancy_state_old) {
        system_status.flag_test = 1;
      }
      // Serial.println(occupancy_state_new, BIN);
      break;
    default:
      // Do nothing
      break;
  }
  system_status.flag_isr_1s = 1;
} /* TIMER1 COMPA CTC Mode - 1s */

