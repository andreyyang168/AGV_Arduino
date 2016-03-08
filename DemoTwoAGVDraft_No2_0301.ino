/*---------------Timer func good without IRrecv-----------------------------*/
/*-------------- + Ultrasonic/ IRdist/ RFID---------------------------------*/
/*-------------- + XBee(Arduino2Arduino)/ Barcode/ Ecompass-----------------*/
/* ============= Ecompass only judge if heading to Destnation ver. ===========
*/
#include <AFMotor.h>                // L293 shield library
#include <PID_v1.h>                 // PID library
#include <IRremote.h>               // IR Remote Control library
#include <Ultrasonic.h>             // Ultrasonic:SR04 library
#include <DistanceGP2Y0A41SK.h>    //Infrared Proximity Sensor Unit librairy, Measuring distance : 4 to 30 cm
#include <Timer.h>                 // Timer library to do several tasks in a period
#include <SPI.h>                   // SPI library for MFRC522
#include <MFRC522.h>               //RFID module library
//#include <math.h>                 //Math library to calculate acos()
#include <PS2Keyboard.h>           // BarcodeScanner library
#include <Wire.h>                  //i2C
#include <HMC5883L.h>              // HMC5883L library
#include <QTRSensors.h>            // IR tracking library
/*****************  Motor Declaration****************/
AF_DCMotor motor1(1);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

/*****************  Motor velocity Variables ****************/
unsigned long sample_time = 0;
unsigned long old_time = 0;
float D = 4.77; // wheel diameter in cm 
float velocity1 = 0;
float angular_veocity1 = 0;
float velocity3 = 0;
float angular_veocity3 = 0;
float velocity4 = 0;
float angular_veocity4 = 0;
int GoForwardEnable = 0, GoBackwardEnable = 0;
int GoCrab_LeftEnable = 0, GoCrab_RightEnable = 0;
int Rotate_CWEnable = 0, Rotate_CCWEnable = 0;
int R45_ForwardEnable = 0, L45_ForwardEnable = 0, R45_BackwardEnable = 0, L45_BackwardEnable = 0;


/***************** Encoder setting, respected to AF_DCMotor ******************/
#define encoderPinA_1   2          // Connect encoder1 wire A to pin  2
#define encoderPinA_3  18          // Connect encoder2 wire A to pin 18
#define encoderPinA_4  19          // Connect encoder3 wire A to pin 19
//#define encoderPinB_1  33          // Connect encoder1 wire B to pin 33
//#define encoderPinB_3  34          // Connect encoder2 wire B to pin 34
//#define encoderPinB_4  35          // Connect encoder3 wire B to pin 35
volatile long encoderCount_1 = 0;  // Counting encoder pulses
long encoderCountOld_1 = 0;
volatile long EncoderCount_1 = 0;  // Counting encoder pulses
long EncoderCountOld_1 = 0;

volatile long encoderCount_3 = 0;  // Counting encoder pulses
long encoderCountOld_3 = 0;
volatile long EncoderCount_3 = 0;  // Counting encoder pulses
long EncoderCountOld_3 = 0;

volatile long encoderCount_4 = 0;  // Counting encoder pulses
long encoderCountOld_4 = 0;


long encoderCount_BeforeTurn1, encoderCount_AfterTurn1;
long encoderCount_BeforeTurn3, encoderCount_AfterTurn3;
long encoderCount_BeforeTurn4, encoderCount_AfterTurn4;

/*****************  PID setting & Declaration****************/
double Setpoint_1,Setpoint_3, Setpoint_4, Input_1, Output_1, Input_3, Output_3, Input_4, Output_4;
//double Setpoint_Enc,Input_Enc, Output_Enc;        
double Kp1=4, Ki1=10.8, Kd1=0.65;          // PID perameters good for M1 
double Kp3=4.8, Ki3=12.96, Kd3=0.78;          // PID perameters good for M3
double Kp4=3, Ki4=10.8, Kd4=0.1;          // PID perameters good for M4
//double KpE=2, KiE=5, KdE=1;           // defult PID perameters   
int outVal1, outVal3, outVal4;        // PID output constrain value for PWM
PID PID_1(&Input_1, &Output_1, &Setpoint_1, Kp1, Ki1, Kd1, DIRECT);
PID PID_3(&Input_3, &Output_3, &Setpoint_3, Kp3, Ki3, Kd3, DIRECT);
PID PID_4(&Input_4, &Output_4, &Setpoint_4, Kp4, Ki4, Kd4, DIRECT);
//PID PID_Enc(&Input_Enc, &Output_Enc, &Setpoint_Enc, KpE, KiE, KdE, DIRECT);

/*****************  IR Remote Control setting ****************/
const int RECV_PIN1 = 44;
const int RECV_PIN2 = 45;
const int RECV_PIN3 = 46;
IRrecv irrecv1(RECV_PIN1);
IRrecv irrecv2(RECV_PIN2);
IRrecv irrecv3(RECV_PIN3);
decode_results results;   //result from irrecv1,2,3
unsigned long IRincome;

/*****************  SR04 Declaration ****************/
#define TRIGGER_F_PIN  22
#define ECHO_F_PIN     23
Ultrasonic ultrasonic_F(TRIGGER_F_PIN, ECHO_F_PIN);
#define TRIGGER_R_PIN  24
#define ECHO_R_PIN     25
Ultrasonic ultrasonic_R(TRIGGER_R_PIN, ECHO_R_PIN);
#define TRIGGER_L_PIN  26
#define ECHO_L_PIN     27
Ultrasonic ultrasonic_L(TRIGGER_L_PIN, ECHO_L_PIN);
long microsec_F, microsec_R, microsec_L;
float cmMsec_F, cmMsec_R, cmMsec_L;  
int goDetectFlag = 0;
int RightEndTerminal = 0;
int LeftEndTerminal = 0;
int encoderRecordIndex = 0;

/*****************  Infrared Proximity Sensor Declaration ****************/
DistanceGP2Y0A41SK IRdist_RF;   //Right Front  direction
DistanceGP2Y0A41SK IRdist_RM;   //Right Middle direction
DistanceGP2Y0A41SK IRdist_RB;   //Right Back   direction
DistanceGP2Y0A41SK IRdist_LF;   //Left  Front  direction
DistanceGP2Y0A41SK IRdist_LM;   //Left  Middle direction
DistanceGP2Y0A41SK IRdist_LB;   //Left  Back   direction
int distance_RF, distance_RM, distance_RB; // right side distance
int distance_LF, distance_LM, distance_LB; //  left side distance
int IRdist_RF_pick, IRdist_RM_pick, IRdist_RB_pick; //which IR be choosen
int IRdist_LF_pick, IRdist_LM_pick, IRdist_LB_pick; //which IR be choosen
int L_Obs; //Length of obstacle
int IRdist_min, IRdist_min2; //min distance and second min distance
double h, theta_rad, theta_Orient;

/*****************  Timer Declaration ****************/
Timer timer_RFIDread;
Timer timer_QTRSensors;
Timer timer_Velocity;
Timer timer_HMC5883L;
Timer timer_IRdist;
Timer timer_Barcode;
Timer timer_SR04;
Timer timer_Battery;


/*****************  BTN Declaration ****************/ 
const int StopBtnPin = 48;
int autoAvoidingFlag = 0;
const int DemoBtnPin = 49;

/*****************  RFID(NFC) Declaration ****************/
#define RST_PIN   41     // Digital_Output: HIGH/LOW
#define SS_PIN    53     //
MFRC522 mfrc522(SS_PIN, RST_PIN);  // Create MFRC522 instance
byte upDockTag[10]= {0x09, 0x22, 0xe7, 0x45}; //Tag1 UID
byte downDockTag[10]= {0x43, 0x36, 0xe8, 0x45}; //Tag2 UID
byte AGV1_Tag[10]= {0xb3, 0x34, 0xe5, 0x45}; //Tag3 UID
byte AGV2_Tag[10]= {0xb9, 0x6b, 0xe7, 0x45}; //Tag4 UID
unsigned long uid;
int RFdetectEnable = 1; //for Enable RFIDevent() func. 1:Enable RFID()

/*****************  Xbee Declaration *********************/
char XBeeVal;   // income char
String XBeeStr; // char to String
String XB_BCD; // income Barcode String

/************* BarcodeScanner Declaration ****************/
/* Valid irq pins:
     Arduino Mega: 2, 3, 18, 19, 20, 21 */
const int DataPin = 30;
const int IRQpin =  3;
int barcodeIndex = 0;
String barcodeBuf="", barcodeNow="", barcodePrev=""; 
PS2Keyboard keyboard;

/************* QTRSensors Declaration ****************/
#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   31     // LED ON
#define D1  33  //sensor 8 (Sensor朝下時最左邊那顆)
#define D2  35  //sensor 7
#define D3  37  //sensor 6
#define D4  39  //sensor 5
#define D5  32  //sensor 4
#define D6  34  //sensor 3
#define D7  36  //sensor 2
#define D8  38  //sensor 1

QTRSensorsRC qtrrc((unsigned char[]) {D1, D2, D3, D4, D5, D6, D7, D8}, NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS];
uint8_t QTR_value;
double PWMcomp1=0, PWMcomp3=0, PWMcomp4=0; // motor PWM for direction compensation 

/****************** E-Compass Declaration ****************/
HMC5883L compass;  // Store our compass as a variable.
float Yaw; // angle on XY-plane

/****************** Demo var Declaration ****************/
String UpTerminal = "X01Y13", DownTerminal = "X01Y07"; //barcode tag
String DestinationPos = UpTerminal; //set initial DestinationPos. Confirm Before Test!!

String XBeeStrSub, barcodeNowSub, UpTerSub, DownTerSub; //for findNearTerm()
int nearTerminal; //for findNearTerm()
int XB2Int, barcode2Int, UpTer2Int, DownTer2Int; //for findNearTerm()

//float UpDockAng = 151.2, DownDockAng = 352.29; //2F, confirm before test==>AGV1
float UpDockAng = 192.25, DownDockAng = 326.20; //2F, confirm before test==>AGV2
//float UpDockAng = 255, DownDockAng = 58; //3F, confirm before test
float DestinationAng = UpDockAng; //set  initial DestinationAng. Confirm Before Test!!
float Tolerance = 45; //Yaw tolerance of DestinationAng

const int BattPin1 = A14;
const int BattPin2 = A15;
//float QtyBatt1, QtyBatt2, thresholdVol = 3.7; //2 Vol is of Arduino set 
float QtyBatt1, QtyBatt2, thresholdVol_74 = 5.3, thresholdVol_111 = 9; //2 Vol is of Arduino set 
int demoGoEnable = 0; //enter demo or not
int lowBattFlag = 0;  //detect law Battery or not
int rfid_event = 0, make_way = 0;       //detectRFID and makeWay Flag

int LowDockOcpd = 0, UpDockOcpd = 0;  //inform the other which dock is occupied

/*****************  Indicator Declaration ****************/
const int lowBattPin = 40;
const int onChargingPin = 42;

const int QTR_Indi_Led0 = A0;
const int QTR_Indi_Led1 = A1;
const int QTR_Indi_Led2 = A2;
const int QTR_Indi_Led3 = A3;
const int QTR_Indi_Led4 = A4;
const int QTR_Indi_Led5 = A5;
const int QTR_Indi_Led6 = A6;

/*****************  Button Variables ****************/
//int stopBtnState = 0;
//int old_stopBtnState = 0;

void setup(){   
    Serial.begin(9600);
//  Serial.println("t, angleV1, vel1, out_1, angleV3, vel3, out_3, angleV3, vel3, out_3");
  //-------------IR recieve setting-------------------------------------------
//  irrecv1.enableIRIn(); // Start the receiver
//  irrecv2.enableIRIn(); // Start the receiver
//  irrecv3.enableIRIn(); // Start the receiver
//  //-------------Encoder Interrupt--------------------------------------------
  pinMode(encoderPinA_1, INPUT); 
  digitalWrite(encoderPinA_1, HIGH);       // turn on pullup resistor
  pinMode(encoderPinA_3, INPUT); 
  digitalWrite(encoderPinA_3, HIGH);       // turn on pullup resistor
  pinMode(encoderPinA_4, INPUT); ; 
  digitalWrite(encoderPinA_4, HIGH);       // turn on pullup resistor
  attachInterrupt(digitalPinToInterrupt(encoderPinA_1), EncoderISR_1, FALLING);  // encoder pin on interrupt 0 --> pin 02
  attachInterrupt(digitalPinToInterrupt(encoderPinA_3), EncoderISR_3, FALLING);  // encoder pin on interrupt 5 --> pin 18
  attachInterrupt(digitalPinToInterrupt(encoderPinA_4), EncoderISR_4, FALLING);  // encoder pin on interrupt 4 --> pin 19
  //-------------PID--------------------------------------------
  PID_1.SetMode(AUTOMATIC);
  PID_3.SetMode(AUTOMATIC);
  PID_4.SetMode(AUTOMATIC);
//  PID_Enc.SetMode(AUTOMATIC);
  //-------------Motor Initialize-------------------------------
  motor1.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
  //-------------Infrared Proximity Sensor Initialize-----------
  IRdist_RF.begin(A10);  
  IRdist_RM.begin(A9);
  IRdist_RB.begin(A8);
  IRdist_LF.begin(A11);
  IRdist_LM.begin(A12);
  IRdist_LB.begin(A13);
  //-------------Barcode Scanner Setting------------------------
  keyboard.begin(DataPin, IRQpin);  // barcode_init
  delay(1000);
  //-------------E-compass Setting------------------------------
  Wire.begin(); // Start the I2C interface.
  compass = HMC5883L();  // Construct a new HMC5883 compass.
  compass.SetScale(1.3); // Set the scale of the compass.
  compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
  //-------------Timer event Setting----------------------------
  timer_RFIDread.every(100, MFRC522read); //每經過100毫秒，就會呼叫 MFRC522read
  timer_QTRSensors.every(120, IRtracking); //每經過120毫秒，就會呼叫 headingAngle
  timer_Barcode.every(200, readBarcode); //每經過200毫秒，就會呼叫 readBarcode
  timer_Velocity.every(300, calVelocity); //每經過300毫秒，就會呼叫 calVelocity
  timer_HMC5883L.every(450, headingAngle); //每經過450毫秒，就會呼叫 headingAngle
  timer_SR04.every(500, ultrasonicDetect); //每經過500毫秒，就會呼叫 ultrasonicDetect
  timer_IRdist.every(1000, IR_distance); //每經過1000毫秒，就會呼叫 IR_distance
  timer_Battery.every(10000, detectVol); //每經過10秒，就會呼叫 detectVol
  
  //-------------BTN Setting------------------------------------
  pinMode(StopBtnPin, INPUT);
  pinMode(DemoBtnPin, INPUT);
  //-------------RFID(NFC) Setting------------------------------
  SPI.begin();      // Init SPI bus
  mfrc522.PCD_Init();   // Init MFRC522
//  Serial.println("Try the most used default keys to print block 0 of a MIFARE PICC.");
  //-------------LED Setting------------------------------------
  pinMode(lowBattPin, OUTPUT); //40
  pinMode(onChargingPin, OUTPUT); //42
  
  pinMode(QTR_Indi_Led0, OUTPUT); //A0
  pinMode(QTR_Indi_Led1, OUTPUT); //A1
  pinMode(QTR_Indi_Led2, OUTPUT); //A2
  pinMode(QTR_Indi_Led3, OUTPUT); //A3
  pinMode(QTR_Indi_Led4, OUTPUT); //A4
  pinMode(QTR_Indi_Led5, OUTPUT); //A5
  pinMode(QTR_Indi_Led6, OUTPUT); //A6
  
  //-------------QTRSensors Setting------------------------------------
//  startCalibrate();   //Initialize QTRSensors
  
   
}



void loop(){

/* --------------------------Update Velocity Input-----------------------
-----------------------------------------------------------------------*/
  Input_1 = velocity1;
  Input_3 = velocity3;
  Input_4 = velocity4;
  
/* --------------------------Update timer even---------------------------
-----------------------------------------------------------------------*/
  timer_RFIDread.update();    // MFRC522 RFID 100ms
  timer_QTRSensors.update();  // headingAngle 120ms
  timer_Velocity.update();    // calVelocity 200ms
  timer_HMC5883L.update();    // headingAngle 250ms
  timer_IRdist.update();      // IR_distance 300ms
  timer_Barcode.update();     // readBarcode 350ms
  timer_SR04.update();        // ultrasonicDetect 500ms
  timer_Battery.update();     // ultrasonicDetect 3000ms
  
/* ---------------------------Barcode----------------------------------------
----confirm barcodeBuf if correct and store into latest 2 barcode----------*/
  if(barcodeBuf.length()>=6 ){
      if(barcodeBuf.startsWith("X01Y")){
          barcodeBuf.trim(); 
          barcodeBuf.substring(0); 
          barcodeNow = barcodeBuf;
          Serial.print(barcodeNow);   /*after trim*/
      }
  }
      

/* --------------------------RFIDevent------------------------------
--------------------goCharging or makeway--------------------------*/
  if(RFdetectEnable == 1) RFIDevent(); //1: Enable RFIDevent() func.
//    Serial.println(uid);


/* ------------------Enter One AGV Demo mode -------------------------------------------
 * ------------------Trigger by XBee recv char 'g' -----------------------------------*/
  if(demoGoEnable == 1){
    /* Set Destination-->judge direction if correct-->if NOT, then turn arouind-->keep going 
    *      |_______________________________________________________________________|       */

//    Serial.println("demoGoEnable == 1");
  if(RFdetectEnable == 1) RFIDevent(); //1: Enable RFIDevent() func.
  
    /* Give tolerance*/
    float lowLimitAng = DestinationAng-Tolerance; 
    float highLimitAng = DestinationAng+Tolerance; 
    /* Concern Yaw critical value:0 
     *  1.) lowLimitAng > 0, highLimitAng < 360
     *  2.) highLimitAng > 360
     *  3.) lowLimitAng < 0
     *  PS# Yaw angle: for turn around judgment 
    */
    
    /*~~~~~~~~~~~~~~~~~~~~~Sufficient Battery!!~~~~~~~~~~~~~~~~~~~ */
    if(lowBattFlag == 0){
      /* float UpDockAng = 151.2, DownDockAng = 352.29; //2F, confirm before test */  
//      Serial.println("lowBattFlag = 0");
      /** 1.)  lowLimitAng > 0, highLimitAng < 360 ---> Normal Condition*/ 
      if( 0<lowLimitAng && highLimitAng<360){
//        Serial.println("1.) 0<lowLimitAng && highLimitAng<360 ...............");
          /* 如果轉到DestinationPos的角度"DestinationAng"範圍內 */
          if( lowLimitAng<= Yaw && Yaw<=highLimitAng ){
//            Serial.println("1.) lowLimitAng<= Yaw && Yaw<=highLimitAng ----------------");
              /* 最新一筆barcode跟DestinationPos不同，keep going */
              if(barcodeNow != DestinationPos){
//                Serial.println("1.)lowBattFlag == 0,Normal==>Go!! Go!! Go!! Go!! Go!! Go!! Go!!");
                Rotate_CCWEnable = 0;
                GoForwardEnable = 1; //let AGV go!
              }
              
              /* 如果是到達上端點，設置新的DestinationPos = DownTerminal, 與新方向DestinationAng = DownDockAngle */
              if(barcodeNow == UpTerminal){
//                Serial.println("1.)now, barcodeNow == UpTerminal, change Destination");
                DestinationPos = DownTerminal;
                DestinationAng = DownDockAng;
              }
              
              /* 如果是到達下端點，設置新的DestinationPos = UpTerminal, 與新方向DestinationAng = UpDockAngle */
              else if( barcodeNow == DownTerminal){
//                Serial.println("1.)now, barcodeNow == DownTerminal, change Destination");
                DestinationPos = UpTerminal;
                DestinationAng = UpDockAng;
              }
          } // lowLimitAng<= Yaw && Yaw<=highLimitAng
          
          else{
            //        Serial.print("turn180,  Yaw:");Serial.println(Yaw);
              turn180();
          }          
      } // 0<lowLimitAng && highLimitAng<360 
          
      /** 2.)  highLimitAng > 360  Condition*/ 
      if(highLimitAng > 360){
        highLimitAng = highLimitAng-360;
        
        if(highLimitAng<= Yaw && Yaw<=lowLimitAng){
           turn180();
        } 
        else{
           /* 最新一筆barcode跟DestinationPos不同，keep going */
            if(barcodeNow != DestinationPos){
//              Serial.println("2.)highLimit >360===>Go!! Go!! Go!! Go!! Go!! Go!! Go!!");
              Rotate_CCWEnable = 0;
              GoForwardEnable = 1; //let AGV go!
            }
            
            /* 如果是到達上端點，設置新的DestinationPos = DownTerminal, 與新方向DestinationAng = DownDockAngle */
            if(barcodeNow == UpTerminal){ 
//              Serial.println("2.)now, barcodeNow == UpTerminal, change Destination");
              DestinationPos = DownTerminal;
              DestinationAng = DownDockAng;
            }
            
            /* 如果是到達下端點，設置新的DestinationPos = UpTerminal, 與新方向DestinationAng = UpDockAngle */
            else if( barcodeNow == DownTerminal){ 
//              Serial.println("2.)now, barcodeNow == DownTerminal, change Destination");
              DestinationPos = UpTerminal;
              DestinationAng = UpDockAng;
            }
        }
      } // highLimitAng > 360
      
      /** 3.)  lowLimitAng < 0  Condition*/ 
      if(lowLimitAng < 0){
        lowLimitAng = lowLimitAng+360;
        if(highLimitAng<= Yaw && Yaw<=lowLimitAng){
           turn180();
        }
        else{
           /* 最新一筆barcode跟DestinationPos不同，keep going */
            if(barcodeNow != DestinationPos){
//              Serial.println("3.)lowLimit <0===>Go!! Go!! Go!! Go!! Go!! Go!! Go!!");
              Rotate_CCWEnable = 0;
              GoForwardEnable = 1; //let AGV go!
            }
            
            /* 如果是到達上端點，設置新的DestinationPos = DownTerminal, 與新方向DestinationAng = DownDockAngle */
            if(barcodeNow == UpTerminal){ 
//              Serial.println("3.)now, barcodeNow == UpTerminal, change Destination");
              DestinationPos = DownTerminal;
              DestinationAng = DownDockAng;
            }
            
            /* 如果是到達下端點，設置新的DestinationPos = UpTerminal, 與新方向DestinationAng = UpDockAngle */
            else if( barcodeNow == DownTerminal){ 
//              Serial.println("3.)now, barcodeNow == DownTerminal, change Destination");
              DestinationPos = UpTerminal;
              DestinationAng = UpDockAng;
            }
        }
      } //lowLimitAng < 0
    } //lowBattFlag==0
    
    /*~~~~~~~~~~~~~~~~~~~~~電量不足!! ~~~~~~~~~~~~~~~~~*/
    if(lowBattFlag == 1){
//      Serial.println("lowBattFlag == 1");
      /* 尋找最接近的充電站 => Set DestinationPos & DestinationAng, NOT yet turn180 */
      findDock();
            
      /** 1.)  lowLimitAng > 0, highLimitAng < 360 ---> Normal Condition*/
      if( 0<lowLimitAng && highLimitAng<360 ){
          if( lowLimitAng<Yaw && Yaw<highLimitAng ){
            
              /* 尚未到達充電站前的barcode位置，但是方向正確，繼續行走 */
              if( barcodeNow!=DestinationPos ){
//                 Serial.println("1.)<=thresholdVol, Go! Go! Go! Go! Go! Go! Go! ");
                Rotate_CCWEnable = 0;
                GoForwardEnable = 1;
              }
              
              /* 到達充電站前的barcode位置，而且方向正確，嘗試充電!! */
              if( barcodeNow==DestinationPos ){
                /* 距離充電站>5.45，且還沒讀到RFID，approaching */
                if(cmMsec_F > 5.45 && memcmp(upDockTag, mfrc522.uid.uidByte, mfrc522.uid.size) != 0){
//                  Serial.println("1.)<=thresholdVol, cmMsec_F > 5.45 ");
                  GoForward();  // 應該要慢速
                }
                /* 距離充電站<=5.45，且還沒讀到RFID，左右平移掃Tag */
                else if(cmMsec_F <= 5.45 && memcmp(upDockTag, mfrc522.uid.uidByte, mfrc522.uid.size) != 0){
//                  Serial.println("<=thresholdVol, GoCrab_Search ");
                  GoCrab_Search(); 
                }
                else  Stop();
              }
          } // lowLimitAng<Yaw && Yaw<highLimitAng
          /* 沒電情況發生Yaw角度偏離DestinationAng, 強制轉彎導正
           * PS# Yaw角度只為判斷AGV是否要轉180度 */
          else{
//            Serial.println("1.)<=thresholdVol, turn180");
            turn180();
          }
      }   // 0<lowLimitAng && highLimitAng<360
      
      /** 2.)  highLimitAng > 360  Condition*/
      if( highLimitAng > 360 ){
          highLimitAng = highLimitAng - 360;
         /* 沒電情況發生Yaw角度偏離DestinationAng, 強制轉彎導正 
          * PS# Yaw角度只為判斷AGV是否要轉180度 */
          if( highLimitAng<Yaw && Yaw<lowLimitAng){
//            Serial.println("2.)<=thresholdVol, turn180");
            turn180();
          }
          else{
              /* 尚未到達充電站前的barcode位置，但是方向正確，繼續行走 */
              if( barcodeNow!=DestinationPos ){
//                 Serial.println("2.)<=thresholdVol, Go! Go! Go! Go! Go! Go! Go! ");
                Rotate_CCWEnable = 0;
                GoForwardEnable = 1;
              }
              
              /* 到達充電站前的barcode位置，而且方向正確，嘗試充電!! */
              if( barcodeNow==DestinationPos ){
                /* 距離充電站>5.45，且還沒讀到RFID，approaching */
                if(cmMsec_F > 5.45 && memcmp(upDockTag, mfrc522.uid.uidByte, mfrc522.uid.size) != 0){
//                  Serial.println("2.)<=thresholdVol, cmMsec_F > 5.45 ");
                  GoForward();  // 應該要慢速
                }
                /* 距離充電站<=5.45，且還沒讀到RFID，左右平移掃Tag */
                else if(cmMsec_F <= 5.45 && memcmp(upDockTag, mfrc522.uid.uidByte, mfrc522.uid.size) != 0){
//                  Serial.println("2.)<=thresholdVol, GoCrab_Search ");
                  GoCrab_Search(); 
                }
                else  Stop();
              }
          } // else
      } //highLimitAng>360

      /** 3.)  lowLimitAng < 0  Condition*/
      if( lowLimitAng < 0 ){
          lowLimitAng = lowLimitAng + 360;
         /* 沒電情況發生Yaw角度偏離DestinationAng, 強制轉彎導正 
          * PS# Yaw角度只為判斷AGV是否要轉180度 */
          if( highLimitAng<Yaw && Yaw<lowLimitAng){
//            Serial.println("3.)<=thresholdVol, turn180");
            turn180();
          }
          else{
              /* 尚未到達充電站前的barcode位置，但是方向正確，繼續行走 */
              if( barcodeNow!=DestinationPos ){
//                 Serial.println("3.)<=thresholdVol, Go! Go! Go! Go! Go! Go! Go! ");
                Rotate_CCWEnable = 0;
                GoForwardEnable = 1;
              }
              
              /* 到達充電站前的barcode位置，而且方向正確，嘗試充電!! */
              if( barcodeNow==DestinationPos ){
                /* 距離充電站>5.45，且還沒讀到RFID，approaching */
                if(cmMsec_F > 5.45 && memcmp(upDockTag, mfrc522.uid.uidByte, mfrc522.uid.size) != 0){
//                  Serial.println("3.)<=thresholdVol, cmMsec_F > 5.45 ");
                  slowGoForward();  //low speed
                }
                /* 距離充電站<=5.45，且還沒讀到RFID，左右平移掃Tag */
                else if(cmMsec_F <= 5.45 && memcmp(upDockTag, mfrc522.uid.uidByte, mfrc522.uid.size) != 0){
//                  Serial.println("3.)<=thresholdVol, GoCrab_Search ");
                  GoCrab_Search(); 
                }
                else  Stop();
              }
          } // else
      } //lowLimitAng<0
      
    } // lowBattFlag=1 End
    
    /* Two AGV Avoidance without Detecting RFID Tag */ 
    if(cmMsec_F <= 25){
        slowGoForward(); // 前方偵測到任何,減速
//        Serial.println("SlowMotion...");
        if(cmMsec_F <= 10){
          GoBackward(); //反向剎車
          Stop(); // 太近了(已小於RFID的模組距離),停止
        }     
    }
   
  } // demoGoEnable = 1 End

  
/* ---------------------------BTN response-------------------------------
/* -----------------------Switch different mode------------------------*/
//Demo Button
  if(digitalRead(DemoBtnPin) == HIGH){
     XBeeVal = 'g'; // i.e. demoGoEnable=1
  } 
  
//Stop  Button
  if( digitalRead(StopBtnPin) == HIGH){
    XBeeVal = 's'; // i.e. Stop()
  } 

/* ------------------------------XBee------------------------------------------
---------------------switch response with keyboard char----------------------*/
  XBeeRecv();      // Read over XBee
  switch(XBeeVal){ // 單引號 = char
      case 'w':    //前
        GoForwardEnable = 1;
//        Serial.println("XBee: GoForward");
      break;
      
      case 'x':  //後
        GoBackwardEnable = 1;
//        Serial.println("XBee: GoBackward");
      break;
      
      case 'a':  //螃蟹左
        GoCrab_LeftEnable = 1;
//        Serial.println("XBee: GoCrab_Left");
      break;
      
      case 'd':  //螃蟹右
        GoCrab_RightEnable = 1;
//        Serial.println("XBee: GoCrab_Right");
      break;

      case 's':  //停止
        Stop();
//        Serial.println("XBee: Stop");
      break;
      
      case 'r':  //右自轉
        Rotate_CWEnable = 1;
//        Serial.println("XBee: Rotate_CW");
      break;
      
      case 't':  //左自轉
        Rotate_CCWEnable = 1;
//        Serial.println("XBee: Rotate_CCW");
      break;

      case 'e':  //Right Forward
        R45_ForwardEnable = 1;
//        Serial.println("XBee: R45_ForwardEnable");
      break;

      case 'q':  //Left Forward
        L45_ForwardEnable = 1;
//        Serial.println("XBee: L45_ForwardEnable");
      break;

      case 'c':  //Right Backward
        R45_BackwardEnable = 1;
//        Serial.println("XBee: R45_BackwardEnable");
      break;

      case 'z':  //Left Backward
        L45_BackwardEnable = 1;
//        Serial.println("XBee: L45_BackwardEnable");
      break;

      case 'g':  //Left Backward
        demoGoEnable = 1;
//        Serial.println("XBee: domeGoEnable");
      break;

      case 'u':  //Left Backward
        slowGoForward();
//        Serial.println("slowGoForward");
      break;
      
      case 'M':  //Left Backward
        makeWay();
      break;
      case 'N':  //Left Backward
        BacktoLine();
      break;
    }
    
/* -----------------------------------IR Receive-------------------------------------------
-----------------------------------------------------------------------------------------*/
  if (irrecv1.decode(&results) || irrecv2.decode(&results) || irrecv3.decode(&results)) {
    if(results.value != 0xFFFFFFFF){
      IRincome = results.value;
    } 
    switch(IRincome){
      case 0x97483BFB:    //前
        Stop();
        GoForwardEnable = 1;
//        Serial.println("IRrecv.: GoForward");
      break;
      
      case 0x488F3CBB:  //後
        Stop();
        GoBackwardEnable = 1;
//        Serial.println("IRrecv.:GoBackward");
      break;
      
      case 0x9716BE3F:  //螃蟹左
        Stop();
        GoCrab_LeftEnable = 1;
//        Serial.println("IRrecv.:GoCrab_Left");
      break;
      
      case 0x6182021B:  //螃蟹右
        Stop();
        GoCrab_RightEnable = 1;
//        Serial.println("IRrecv.:GoCrab_Right");
      break;

      case 0x3D9AE3F7:  //Release, setSpeed(0)=run(RELEASE)
        Stop();
//        Serial.println("IRrecv.:Stop");
      break;
      
      case 0x3EC3FC1B:  //右自轉
        Stop();
        Rotate_CWEnable = 1;
//        Serial.println("IRrecv.:Rotate_CW");
      break;
      
      case 0x32C6FDF7:  //左自轉
        Stop();
        Rotate_CCWEnable = 1;
//        Serial.println("IRrecv.:Rotate_CCW");
      break;

      case 0xF0C41643:  //Right Forward
        Stop();
        R45_ForwardEnable = 1;
//        Serial.println("IRrecv.:R45_ForwardEnable");
      break;

      case 0xC101E57B:  //Left Forward
        Stop();
        L45_ForwardEnable = 1;
//        Serial.println("IRrecv.:L45_ForwardEnable");
      break;

      case 0x449E79F:  //Right Backward
        Stop();
        R45_BackwardEnable = 1;
//        Serial.println("IRrecv.:R45_BackwardEnable");
      break;

      case 0x8C22657B:  //Left Backward
        Stop();
        L45_BackwardEnable = 1;
//        Serial.println("IRrecv.:L45_BackwardEnable");
      break;
    }  
    irrecv1.resume(); // Receive the next value
    irrecv2.resume(); // Receive the next value
    irrecv3.resume(); // Receive the next value
  }
//------------------------IR controller response---------------------------
//-------------------------------------------------------------------------
  if(GoForwardEnable == 1){
    GoForward();
//    Serial.println("IRresp.: GoForward");
  }
  else if(GoBackwardEnable == 1){
    GoBackward();
//    Serial.println("IRresp.: GoBackward");
  }
  else if(GoCrab_LeftEnable == 1){
    GoCrab_Left();
//    Serial.println("IRresp.: GoCrab_Left");
  }
  else if(GoCrab_RightEnable == 1){
    GoCrab_Right();
//    Serial.println("IRresp.: GoCrab_Right");
  }
  else if(Rotate_CWEnable == 1){
    Rotate_CW();
//    Serial.println("IRresp.: Rotate_CW");
  }
  else if(Rotate_CCWEnable == 1){
    Rotate_CCW();
//    Serial.println("IRresp.: Rotate_CCW");
  }
  else if(R45_ForwardEnable == 1){
    R45_Forward();
//    Serial.println("IRresp.: R45_ForwardEnable");
  }
  else if(L45_ForwardEnable == 1){
    L45_Forward();
//    Serial.println("IRresp.: L45_ForwardEnable");
  }
  else if(R45_BackwardEnable == 1){
    R45_Backward();
//    Serial.println("IRresp.: R45_BackwardEnable");
  }
  else if(L45_BackwardEnable == 1){
    L45_Backward();
//    Serial.println("IRresp.: L45_BackwardEnable");
  }

  

  
} // loop()  end


/*~~~~~~~~~~~~~~~~  Call Back Function ~~~~~~~~~~~~~~~~~~*/
/*****************  Motor behavier ****************************/

void GoForward() {
      motor3.run(FORWARD);
//        Setpoint_3 = 5;   
//        PID_3.Compute();
//        PID_3.SetOutputLimits(0, 195);
//        motor3.setSpeed(Output_3);
        motor3.setSpeed(77.8 + PWMcomp3);

//----------------------------------------------------
//        Setpoint_Enc = 0; // cm/sec
//        PID_Enc.Compute();
//----------------------------------------------------
        motor1.run(BACKWARD);
//        Setpoint_1 = 5; // cm/sec
//        PID_1.Compute();
//        PID_1.SetOutputLimits(0, 155);
//        motor1.setSpeed(Output_1);
        motor1.setSpeed(60 + PWMcomp1);

//----------------------------------------------------
        motor4.run(RELEASE); 
     
      int gogo = 1; 
      delay(200);
      if(gogo == 1){
        motor3.setSpeed(66 + PWMcomp3);
        motor1.setSpeed(54 + PWMcomp3);      
      }       
}
void slowGoForward(){
        motor3.run(FORWARD);
//        Setpoint_3 = 5;   
//        PID_3.Compute();
//        PID_3.SetOutputLimits(0, 195);
//        motor3.setSpeed(Output_3);
        motor3.setSpeed(66 + PWMcomp3);
//----------------------------------------------------
        motor1.run(BACKWARD);
//        Setpoint_1 = 5; // cm/sec
//        PID_1.Compute();
//        PID_1.SetOutputLimits(0, 155);
//        motor1.setSpeed(Output_1);
        motor1.setSpeed(54 + PWMcomp1);
//----------------------------------------------------
        motor4.run(RELEASE);
}
void GoBackward() {
//        Setpoint_3 = 9.5;   
        motor3.run(BACKWARD);
//        PID_3.SetOutputLimits(0, 200);
//        PID_3.Compute();  
//        motor3.setSpeed(Output_3);
//        outVal3 =  constrain(Output_3, 0, 50);
        motor3.setSpeed(77.8 + PWMcomp3);
//----------------------------------------------------
//        Setpoint_Enc = 0; // cm/sec
//        PID_Enc.Compute();
//----------------------------------------------------
//        Setpoint_1 = 9.5; // cm/sec
        motor1.run(FORWARD);
//        PID_1.SetOutputLimits(0, 150);
//        PID_1.Compute();
//        motor1.setSpeed(Output_1);
//        outVal1 =  constrain(Output_1, 0, 50);
        motor1.setSpeed(60 + PWMcomp1);
//----------------------------------------------------
        motor4.run(RELEASE);
}
void GoCrab_Right() {
        motor1.run(BACKWARD);
//        Setpoint_1 = 5.5; // cm/sec
//        PID_1.SetOutputLimits(0, 125);
//        PID_1.Compute();
//        motor1.setSpeed(Output_1);

        motor1.setSpeed(60);//without PID
//---------------------------------------------------- 
        motor3.run(BACKWARD);
//        Setpoint_3 = 5.5;
//        PID_3.SetTunings(Kp1, Ki1, Kd1);
//        PID_3.SetOutputLimits(0, 155);
//        PID_3.Compute();
//        motor3.setSpeed(Output_3);

        motor3.setSpeed(77.8);//without PID
//----------------------------------------------------
        motor4.run(FORWARD);
//        Setpoint_4 = 9;
//        PID_4.SetOutputLimits(0, 240);
//        PID_4.Compute();
//        motor4.setSpeed(Output_4);
//Serial.print("output4= ");Serial.println(Output_4);
        motor4.setSpeed(125);// without PID
}
void GoCrab_Left() {
        motor1.run(FORWARD);
        Setpoint_1 = 5.5; // cm/sec
//        PID_1.SetOutputLimits(0, 125);
//        PID_1.Compute();
//        motor1.setSpeed(Output_1);

        motor1.setSpeed(60);//without PID
//---------------------------------------------------- 
        motor3.run(FORWARD);
        Setpoint_3 = 5.5;
//        PID_3.SetTunings(Kp1, Ki1, Kd1);
//        PID_3.SetOutputLimits(0, 155);
//        PID_3.Compute();
//        motor3.setSpeed(Output_3);

        motor3.setSpeed(77.8);//without PID
//----------------------------------------------------
        motor4.run(BACKWARD);
//        Setpoint_4 = 9;
//        PID_3.SetOutputLimits(0, 240);
//        PID_4.Compute();
//        motor4.setSpeed(Output_4);
//Serial.print("output4= ");Serial.println(Output_4);
        motor4.setSpeed(125);// without PID
}
void Rotate_CCW() {
        motor1.run(FORWARD);
        motor1.setSpeed(60);
        motor3.run(FORWARD);
        motor3.setSpeed(70);
        motor4.run(FORWARD);
        motor4.setSpeed(80);
}
void Rotate_CW() {
        motor1.run(BACKWARD);
        motor1.setSpeed(82);
        motor3.run(BACKWARD);
        motor3.setSpeed(92);
        motor4.run(BACKWARD);
        motor4.setSpeed(68);
}
void R45_Forward() {
        Setpoint_1 = 11.2; // cm/sec
        motor1.run(BACKWARD);
//        PID_1.Compute();
//        outVal1 =  constrain(Output_1, 0, 50);
//        motor1.setSpeed(60 + outVal1);
        motor1.setSpeed(170);
//----------------------------------------------------
        Setpoint_3 = 3;
        motor3.run(FORWARD);
//        PID_3.Compute();
//        outVal3 =  constrain(Output_3, 0, 50);
//        motor3.setSpeed(35 + outVal3);
        motor3.setSpeed(85);
//----------------------------------------------------
        Setpoint_4 = 8.2;
        motor4.run(FORWARD);
//        PID_4.Compute();
//        outVal4 =  constrain(Output_4, 0, 50);
//        motor4.setSpeed(40 + outVal4);
        motor4.setSpeed(110);
}
void L45_Forward() {
        Setpoint_1 = 3; // cm/sec
        motor1.run(BACKWARD);
//        PID_1.Compute();
//        outVal1 =  constrain(Output_1, 0, 50);
//        motor1.setSpeed(60 + outVal1);
        motor1.setSpeed(85); 
//----------------------------------------------------
        Setpoint_3 = 11.2;
        motor3.run(FORWARD);
//        PID_3.Compute();
//        outVal3 =  constrain(Output_3, 0, 50);
//        motor3.setSpeed(35 + outVal3);
        motor3.setSpeed(170);
//----------------------------------------------------
        Setpoint_4 = 8.2;
        motor4.run(BACKWARD);
//        PID_4.Compute();
//        outVal4 =  constrain(Output_4, 0, 50);
//        motor4.setSpeed(40 + outVal4);
        motor4.setSpeed(110);
}
void R45_Backward() {
        Setpoint_1 = 11.2; // cm/sec
        motor1.run(FORWARD);
//        PID_1.Compute();
//        outVal1 =  constrain(Output_1, 0, 50);
//        motor1.setSpeed(60 + outVal1);
        motor1.setSpeed(85); 
//----------------------------------------------------
        Setpoint_3 = 3;
        motor3.run(BACKWARD);
//        PID_3.Compute();
//        outVal3 =  constrain(Output_3, 0, 50);
//        motor3.setSpeed(35 + outVal3);
        motor3.setSpeed(170); 
//----------------------------------------------------
        Setpoint_4 = 8.2;
        motor4.run(FORWARD);
//        PID_4.Compute();
//        outVal4 =  constrain(Output_4, 0, 50);
//        motor4.setSpeed(40 + outVal4);
        motor4.setSpeed(110);
}
void L45_Backward() {
        Setpoint_1 = 11.2; // cm/sec
        motor1.run(FORWARD);
//        PID_1.Compute();
//        outVal1 =  constrain(Output_1, 0, 50);
//        motor1.setSpeed(60 + outVal1);
        motor1.setSpeed(170); 
//----------------------------------------------------
        Setpoint_3 = 3;
        motor3.run(BACKWARD);
//        PID_3.Compute();
//        outVal3 =  constrain(Output_3, 0, 50);
//        motor3.setSpeed(35 + outVal3);
        motor3.setSpeed(85);
//----------------------------------------------------
        Setpoint_4 = 8.2;
        motor4.run(BACKWARD);
//        PID_4.Compute();
//        outVal4 =  constrain(Output_4, 0, 50);
//        motor4.setSpeed(40 + outVal4);
        motor4.setSpeed(110);
}
void GoCrab_Search(){
  if(encoderRecordIndex == 0){
//    Serial.print("encoderCount_BeforeTurn1: "); Serial.println(encoderCount_BeforeTurn1);
    encoderCount_1 = 0;
    encoderCount_BeforeTurn1 = encoderCount_1;
    encoderRecordIndex = 1;
//    Serial.print("encoderRecordIndex = 1 <==================================== ");
  }
  while( LeftEndTerminal == 0 && RightEndTerminal == 0){ //開始搜尋
      if(encoderCount_1 < encoderCount_BeforeTurn1+450){  //先往右跑搜尋
        GoCrab_Right();
//        Serial.print("encoderCount_BeforeTurn1: "); Serial.println(encoderCount_BeforeTurn1);
//        Serial.print(", ");Serial.print(encoderCount_1); Serial.println("==>Go Right First..........");
       }
      if(encoderCount_1 >= encoderCount_BeforeTurn1+450) RightEndTerminal = 1;  // arrive Right terminal
  }
  while( LeftEndTerminal == 0 && RightEndTerminal == 1){ //右移搜尋完畢
      if( encoderCount_BeforeTurn1+450 <= encoderCount_1 < encoderCount_BeforeTurn1+1350){  //再往左跑搜尋
        GoCrab_Left();
//        Serial.print("encoderCount_BeforeTurn1: "); Serial.println(encoderCount_BeforeTurn1);
//        Serial.print(", ");Serial.print(encoderCount_1); Serial.println("==>Then Go left ..........");
      }
      if(encoderCount_1 >= encoderCount_BeforeTurn1+1350) LeftEndTerminal = 1; // arrive Left terminal
  }
  while( LeftEndTerminal == 1 && RightEndTerminal == 1){ //回去原路徑上
      if( encoderCount_BeforeTurn1+1350 <= encoderCount_1 < encoderCount_BeforeTurn1+1800){
        GoCrab_Right();
//        Serial.print("encoderCount_BeforeTurn1: "); Serial.println(encoderCount_BeforeTurn1);
//        Serial.print(", ");Serial.print(encoderCount_1); Serial.println("==>Back to line .........................");
      }
      if( encoderCount_1 >= encoderCount_BeforeTurn1+1800){
        goDetectFlag = 0;
        Stop();
//        Serial.print("Demo Stop!!!!Demo Stop!!!!Demo Stop!!!!Demo Stop!!!!Demo Stop!!!!Demo Stop!!!!Demo Stop!!!!Demo Stop!!!!");
      }
  }
}
void Stop() {    
        Setpoint_1 = 0;
        Setpoint_3 = 0;
        Setpoint_4 = 0;
        Output_1 = 0;
        Output_3 = 0;
        Output_4 = 0;
        demoGoEnable = 0;
        autoAvoidingFlag = 0;  // from btn
        goDetectFlag = 0;   //RFID Tag search motion
        RFdetectEnable = 1; //for Enable RFIDevent() func. 1:Enable
//        LowDockOcpd = 0;
//        UpDockOcpd = 0;
        LeftEndTerminal = 0;
        RightEndTerminal = 0;
        GoForwardEnable = 0;
        GoBackwardEnable = 0;
        GoCrab_LeftEnable = 0;
        GoCrab_RightEnable = 0;
        Rotate_CWEnable = 0;
        Rotate_CCWEnable = 0;
        R45_ForwardEnable = 0;
        L45_ForwardEnable = 0;
        R45_BackwardEnable = 0;
        L45_BackwardEnable = 0;
        motor1.run(RELEASE);
        motor3.run(RELEASE);
        motor4.run(RELEASE);
        motor1.setSpeed(0);
        motor3.setSpeed(0);
        motor4.setSpeed(0);
}

/*****************  Encoder Interrupt CallBack ***************************/
void EncoderISR_1() {
    encoderCount_1 += 1;
//    EncoderCount_1 = map(encoderCount_1,1,encoderCount_1,1,encoderCount_1*2);
}
void EncoderISR_3() {
    encoderCount_3 += 1;
//    EncoderCount_3 = map(encoderCount_3,1,encoderCount_3,1,encoderCount_3/2);
}
void EncoderISR_4() {
    encoderCount_4 += 1;
}

/*****************  Calculate Velocity Timer CallBack ****************/
void calVelocity(){
//  Serial.println("Velocity");
  detachInterrupt(encoderPinA_1); // Stop detachInterrupt 5
  detachInterrupt(encoderPinA_3); // Stop detachInterrupt 4
  detachInterrupt(encoderPinA_4); // Stop detachInterrupt 3
  angular_veocity1 = (float) (encoderCount_1-encoderCountOld_1)/0.2; // pulse/sec
  angular_veocity3 = (float) (encoderCount_3-encoderCountOld_3)/0.2; // pulse/sec
  angular_veocity4 = (float) (encoderCount_4-encoderCountOld_4)/0.2; // pulse/sec
  velocity1 = (float) 3.1416*D*(encoderCount_1-encoderCountOld_1)/(900*0.2) ;  // 周長*(pulse/PPM) / 0.2 (cm/s)　M1:900PPR
  velocity3 = (float) 3.1416*D*(encoderCount_3-encoderCountOld_3)/(900*0.2) ;  // 周長*(pulse/PPM) / 0.2 (cm/s)  M3:900PPR
  velocity4 = (float) 3.1416*D*(encoderCount_4-encoderCountOld_4)/(900*0.2) ;  // 周長*(pulse/PPM) / 0.2 (cm/s)  M4:900PPR 
//    Serial.print("Count_1:");        Serial.print(encoderCount_1);  
//    Serial.print(" velocity1:");     Serial.print(velocity1);
//    Serial.print(" Output_1:");      Serial.print(Output_1);
//    Serial.print("    /Count_3: ");  Serial.print(encoderCount_3);
//    Serial.print(" velocity3:");     Serial.print(velocity3);
//    Serial.print(" Output_3:");      Serial.print(Output_3);
//    Serial.print("    / Count_4: "); Serial.print(encoderCount_4);
//    Serial.print(" velocity4:");     Serial.print(velocity4);
//    Serial.print(" Output_4:");      Serial.println(Output_4);
  encoderCountOld_1 = encoderCount_1;      // Set previous count
  encoderCountOld_3 = encoderCount_3;      // Set previous count
  encoderCountOld_4 = encoderCount_4;      // Set previous count   
  attachInterrupt(digitalPinToInterrupt(encoderPinA_1), EncoderISR_1, FALLING); // Reset attachInterrupt 
  attachInterrupt(digitalPinToInterrupt(encoderPinA_3), EncoderISR_3, FALLING); // Reset attachInterrupt 
  attachInterrupt(digitalPinToInterrupt(encoderPinA_4), EncoderISR_4, FALLING); // Reset attachInterrupt 
}

/*****************  Ultrasonic SR04 Timer CallBack *******************/
void ultrasonicDetect(){ 
//  Serial.println("Ultra");
  microsec_F = ultrasonic_F.timing(); // 計算距離聲波回彈時間 microseconds, MS, 沒有時間延時函數指令
  microsec_R = ultrasonic_R.timing();
  microsec_L = ultrasonic_L.timing(); 
  cmMsec_F = ultrasonic_F.convert(microsec_F, Ultrasonic::CM); // 計算距離;單位: 公分
  cmMsec_R = ultrasonic_R.convert(microsec_R, Ultrasonic::CM); 
  cmMsec_L = ultrasonic_L.convert(microsec_L, Ultrasonic::CM);
//  Serial.print("MS: ");  Serial.print(microsec_F);
//  Serial.print(", F_CM: "); Serial.println(cmMsec_F);
//  Serial.print(", R_CM: ");  Serial.print(cmMsec_R);
//  Serial.print(", L_CM: ");  Serial.println(cmMsec_L);
}

/*****************  IR distance Timer CallBack ***********************/
void IR_distance(){
//  Serial.println("IRrecv");
  distance_RF = IRdist_RF.getDistanceCentimeter();
  distance_RM = IRdist_RM.getDistanceCentimeter();
  distance_RB = IRdist_RB.getDistanceCentimeter();
  distance_LF = IRdist_LF.getDistanceCentimeter();
  distance_LM = IRdist_LM.getDistanceCentimeter(); 
  distance_LB = IRdist_LB.getDistanceCentimeter();
//  Serial.print("distance_RF: "); Serial.print(distance_RF);
//  Serial.print(", distance_LF: "); Serial.print(distance_LF);
//  Serial.print(", distance_RM: "); Serial.print(distance_RM);
//  Serial.print(", distance_LM: "); Serial.print(distance_LM);
//  Serial.print(", distance_RB: "); Serial.print(distance_RB);
//  Serial.print(", distance_LB: "); Serial.println(distance_LB); 
}
/*****************  RFID(NFC) Timer CallBack *************************/
void MFRC522read(){
//  Serial.println("RFID");
  if(mfrc522.PICC_IsNewCardPresent()){
    uid = getID();
//    Serial.print("  MFRC552Read, uid="); Serial.println(uid);
  }
//  mfrc522.PICC_ReadCardSerial();
}

// mfrc522.PICC_IsNewCardPresent() should be checked before 
// @return the card UID
unsigned long getID(){
  if ( ! mfrc522.PICC_ReadCardSerial()) { //Since a PICC placed get Serial and continue
    return -1;
  }
  unsigned long hex_num;
  hex_num =  mfrc522.uid.uidByte[0] << 24;
  hex_num += mfrc522.uid.uidByte[1] << 16;
  hex_num += mfrc522.uid.uidByte[2] <<  8;
  hex_num += mfrc522.uid.uidByte[3];
  mfrc522.PICC_HaltA(); // Stop reading
  return hex_num;
}

/*****************  XBee recv Timer CallBack *************************/
void XBeeRecv(){
//  Serial.println("XBee");

  if (Serial.available()) {
    
    /*read data*/
    XBeeVal = Serial.read(); // XBeeVal is a char data
    XBeeStr = XBeeStr + XBeeVal; 

    /*get data I need and clear buffer*/
    if(XBeeStr.length()>=6){
      if(XBeeStr.startsWith("X01Y")){
        XBeeStr.substring(0); 
        XB_BCD = XBeeStr;  // String of 6 char
      }
      XBeeStr = ""; //clear Serial Buffer
      serialFlush(); //clear Serial Buffer
    }
   
//    Serial.print("XBeeVal="); Serial.print(XBeeVal);
//    /*after trim*/
//    Serial.print(barcodeNow); Serial.print(",="); Serial.println(barcodeNow.length()); 
//    /*barcode from the other*/
//    Serial.print("BCD="); Serial.print(XB_BCD); Serial.print(",="); Serial.println(XB_BCD.length());
    

    /*-----------------------------2 AGVs Avoidance-------------------------*/    
    /*收到'I': 另一台已偵測到我的RFID Tag*/
    if (XBeeVal == 'I') {  
      RFdetectEnable = 0;  //shut down RFIDevent() to prevent set rfid_event = 1 both!
      Serial.print(barcodeNow); //發出barcodeNow給另一AGV,以判斷最近Terminal & 誰該讓路
      GoBackward();  //反向剎車
      Stop();
      RFdetectEnable = 0; //claim twice
      delay(200);
    }
   
    /*收到被偵測那台的barcodeNow ==> 判斷誰要讓路*/
    else if (XB_BCD.startsWith("X01Y",0) ) {  
      XB_BCD.trim(); //delete white space in barcode string
//      XB_BCD.substring(0,6);
      findNearTerm(); //找最近的Terminal
       
      /* for makeWay(), 兩台AGV的barcodeNow做比較, 位置離nearTerminal比較遠的AGV要讓路  */
      if(barcodeNow != XB_BCD && rfid_event == 1){
          if(abs(nearTerminal-barcode2Int) > abs(nearTerminal-XB2Int)){
            makeWay();     //send 'K' included
            rfid_event = 0; //reset condition of detect RFIDevent Flag 
            make_way = 1;  //set condition of make_Way() Flag    
          }
          else{
            Serial.println('J');
            Stop();
            delay(200); 
            digitalWrite(QTR_Indi_Led0, HIGH); //A0
            digitalWrite(QTR_Indi_Led6, HIGH); //A6  
            rfid_event = 0; //reset condition of detect RFIDevent Flag
          }
      }
      /* for BacktoLine(), 讓路車收到另一台發送所經的BarcodeNow, 如果與讓路車相同則可以BacktoLine*/
//      else if(barcodeNow == XB_BCD){ /* 另一台barcode = 讓路的barcode */
      else if(barcodeNow == XB_BCD && make_way == 1){ /* 另一台barcode = 讓路的barcode */  
          delay(500);
          BacktoLine(); 
          make_way = 0; //reset condition of make_Way() Flag    
          RFdetectEnable = 1;  //reset
          demoGoEnable = 1;   
      }
          
    }

    /*收到'J': 表示要執行makeWay()*/
    if (XBeeVal == 'J') {
//      Serial.println("AGV1 recv J");
//      Serial.print("AGV1 makeWay...");
      makeWay();     //send 'K' included
      make_way = 1;  //set condition of make_Way() Flag
    }
    
    /*收到'K': makeWay()到達右移點時,發送'K'使另一台keep going*/
    else if (XBeeVal == 'K') {
      digitalWrite(QTR_Indi_Led0, HIGH); //A0
      digitalWrite(QTR_Indi_Led2, HIGH); //A2
      digitalWrite(QTR_Indi_Led4, HIGH); //A4
      digitalWrite(QTR_Indi_Led6, HIGH); //A6
      delay(1000);
      RFdetectEnable = 1;  //reset
      demoGoEnable = 1;
      XBeeVal = 'g';
    }


    /*---------------------------set Destination when lowBatt-------------------------*/ 
    else if (XBeeVal == 'L'){
       LowDockOcpd = 1;
    }
    else if (XBeeVal == 'P'){
    
       UpDockOcpd = 1;
    }
    
    
  } //if (Serial.available())

}
/*****************  Clear Serial Buffer ***************************/
void serialFlush(){
    while(Serial.available() > 0) {
      char t = Serial.read();
    }
}
/*****************  Barcode Timer CallBack ***************************/
void readBarcode(){
//  Serial.println("ReadBarcode");
    String str = "";
    char tmp;
    while (keyboard.available())
    {
      
        tmp = keyboard.read();
        str = str + tmp;
        if (tmp == PS2_ENTER) //check enter Cmd
        {
//            str = str + "\n"; //第二個Enter
            tmp = keyboard.read(); //read the last char
            if (tmp == 0x6A){ //=> ASCII:j (the last char)
                barcodeBuf =  str;
//                Serial.println("inner,barcode:"+barcodeBuf);
            }
        }
        
    }
    barcodeBuf =  str;
//    Serial.println("outer,barcode:"+barcodeBuf);
}
/*****************  E-compass Timer CallBack ***************************/
void headingAngle(){
  MagnetometerRaw raw = compass.ReadRawAxis(); // Retrive the raw values from the compass (not scaled).
  MagnetometerScaled scaled = compass.ReadScaledAxis(); // Retrived the scaled values from the compass (scaled to the configured scale).
  float xHeading = atan2(scaled.YAxis, scaled.XAxis);
  if(xHeading < 0) xHeading += 2*PI;
  if(xHeading > 2*PI) xHeading -= 2*PI;
  Yaw = xHeading * 180/M_PI;
//  Serial.print("XYDegrees:");
//  Serial.println(Yaw);

//  float yHeading = atan2(scaled.ZAxis, scaled.XAxis);
//  if(yHeading < 0) yHeading += 2*PI;
//  if(yHeading > 2*PI) yHeading -= 2*PI;
//  float Pitch = yHeading * 180/M_PI;
//  Serial.print("/ XZDegrees:");
//  Serial.print(Pitch);

//  float zHeading = atan2(scaled.ZAxis, scaled.YAxis);
//  if(zHeading < 0) zHeading += 2*PI;
//  if(zHeading > 2*PI) zHeading -= 2*PI;
//  float Row = zHeading * 180/M_PI;
//  Serial.print("/ YZDegrees:");
//  Serial.println(Row);
}

/*****************  Battery Timer CallBack ***************************/
void detectVol(){
  /* 電量偵測 
    */

    QtyBatt1 = analogRead(A14)/1024.0*7.4; //7.4v
    QtyBatt2 = analogRead(A15)/1024.0*11.1; //11.1v
    if( QtyBatt1<=thresholdVol_74 || QtyBatt2<=thresholdVol_111){
      digitalWrite(lowBattPin,HIGH); //低電量要回家, Red LED on, pin 40
      lowBattFlag = 1;
//      Serial.println("<=thresholdVol");
    }
//    else {
//      digitalWrite(lowBattPin,LOW);
//      lowBattFlag = 0; //for test
//      Serial.println(">thresholdVol"); //for test
//    }
    
//    Serial.print("QtyBatt1(A14)="); Serial.println(QtyBatt1);
//    Serial.print("QtyBatt2(A15)="); Serial.println(QtyBatt2);
}

/*****************  Function for Demo ***************************/
void findDock(){ //沒電才觸發找回家方向, 並設置"最接近"的充電站
//  Serial.println("Find Dock....");
  /*  String: X01Y01, X01Y02, ... 
   *  確認最新barcode的位置:barcodeNow
   *  設置Destination為最接近的Dock
  */
//  if( barcodePrev.substring(4)<barcodeNow.substring(4) && (正Y角度-5)<=xDegrees<=(正Y角度+5)) //行經路徑為+Y ex:08<09
//  else if( barcodePrev.substring(4)<barcodeNow.substring(4) && (負Y角度-5)<=xDegrees<=(負Y角度+5)) //行經路徑為+Y ex:08<09
//  else if( barcodePrev.substring(4)>barcodeNow.substring(4) && (正Y角度-5)<=xDegrees<=(正Y角度+5)) //行經路徑為-Y ex:09>08
//  else if( barcodePrev.substring(4)>barcodeNow.substring(4) && (負Y角度-5)<=xDegrees<=(負Y角度+5)) //行經路徑為-Y ex:09>08
  /*取barcode tag 的Y數字*/
  String  UpTerminal_Y = UpTerminal.substring(4);
  String  barcodeNow_Y = barcodeNow.substring(4);
  String  DownTerminal_Y = DownTerminal.substring(4);
  
  /*第一台出現lowBatt的情形，須判斷那個Dock比較近*/ 
  /*transform String to Int, so that we can do comparison*/ 
  /*離DownTerminal比較近*/
  if( (UpTerminal_Y.toInt()-barcodeNow_Y.toInt()) > (barcodeNow_Y.toInt()-DownTerminal_Y.toInt()) && LowDockOcpd==0 && UpDockOcpd==0){
      DestinationPos = DownTerminal;
      DestinationAng = DownDockAng;
      Serial.println('L'); //inform the other 
  }
  /*離upTerminal比較近*/
  else{
      DestinationPos = UpTerminal;
      DestinationAng = UpDockAng;
      Serial.println('P'); //inform the other 
  }

  /*第二台出現lowBatt的情形，根據LowDockIndex/ UpDockIndex*/ 
  if(LowDockOcpd == 1){
      DestinationPos = UpTerminal;
      DestinationAng = UpDockAng;
  }
  else if(UpDockOcpd == 1){
      DestinationPos = DownTerminal;
      DestinationAng = DownDockAng;
  }
}

void RFIDevent(){
  /* RFID 
   * Memory Comparison(Array1,Array2,Aize of Bytes), the same => return 0
   * upDockTag: terminal1, {0x09, 0x22, 0xe7, 0x45}; //Tag1 UID
   * downDockTag: terminal2, {0x43, 0x36, 0xe8, 0x45}; //Tag2 UID
   * AGV1_Tag: obstacle, {0xb3, 0x34, 0xe5, 0x45}; //Tag3 UID
   * AGV2_Tag: obstacle, {0xb9, 0x6b, 0xe7, 0x45}; //Tag4 UID
  */
  if(memcmp(upDockTag, mfrc522.uid.uidByte, mfrc522.uid.size) == 0)
  {
      // double check
      if(mfrc522.uid.size != 0)      
        chargingAside(); //橫移,trun on charging LED
  }
  
  else if(memcmp(downDockTag, mfrc522.uid.uidByte, mfrc522.uid.size) == 0)
  { 
      // double check
      if(mfrc522.uid.size != 0)      
        chargingAside(); //橫移,turn on charging LED
  }
  
  else if(memcmp(AGV1_Tag, mfrc522.uid.uidByte, mfrc522.uid.size) == 0)
  {
      if(mfrc522.uid.size != 0){      // double check
  //      Serial.println("detect AGV1_Tag (tag3)~~~~~begin to makeWay()~~~~~~~~~~~");
  
        Serial.println('I'); //傳送定義字元'I'告知另一AGV執行makeWay()
        rfid_event = 1;
        GoBackward();  //反向剎車
        Stop();
        delay(200); //stay stop() 200ms
            digitalWrite(QTR_Indi_Led0, HIGH); //A0
            digitalWrite(QTR_Indi_Led1, HIGH); //A1
            digitalWrite(QTR_Indi_Led2, HIGH); //A2
            digitalWrite(QTR_Indi_Led3, HIGH); //A3
            digitalWrite(QTR_Indi_Led4, HIGH); //A4
            digitalWrite(QTR_Indi_Led5, HIGH); //A5
            digitalWrite(QTR_Indi_Led6, HIGH); //A6
      }
  }
  /* Reset */
  mfrc522.uid.size = 0;
//  mfrc522.uid.uidByte = {0x00, 0x00, 0x00, 0x00};
  uid = 0;
}

void chargingAside(){ //讀到RFID才觸發
  encoderCount_4 = 0;
  encoderCount_BeforeTurn4 = encoderCount_4; //record current position(angle)
  encoderCount_AfterTurn4 = encoderCount_BeforeTurn4 + 1800;   //offset 1800 pulse 
  while(encoderCount_4 < encoderCount_AfterTurn4 ){
    GoCrab_Left();
  }
  
  encoderCount_4 = 0;
  encoderCount_BeforeTurn4 = encoderCount_4; //record current position(angle)
  encoderCount_AfterTurn4 = encoderCount_BeforeTurn4 + 900;   //offset 1800 pulse
  while(encoderCount_4 < encoderCount_AfterTurn4 ){
    Rotate_CW();
  }
  
  digitalWrite(onChargingPin, HIGH); //充電中, Green LED on.
  while(1) Stop();
}

/*  當感應到AGV上的RFID先停止, 觸發makeWay() */
void makeWay(){
//  Serial.print(" makeWay");
  //右移
  encoderCount_4 = 0; //reset count
  encoderCount_BeforeTurn4 = encoderCount_4; //record current position(angle)
  encoderCount_AfterTurn4 = encoderCount_BeforeTurn4 + 140;   //offset 112 pulse 
  while(encoderCount_4 < encoderCount_AfterTurn4 ){
      GoCrab_Right();
//      Serial.println("MOVE RIGHT");
  }
  
  GoCrab_Left();  //反向剎車
  Stop();
  
  Serial.print('K'); // 傳送右移讓路訊號給另一台
//  Serial.print("MoveRight, count4= "); Serial.println(encoderCount_AfterTurn4);
  delay(200);
  
}

/*  收到第二次的XBeeStr=barcodeNow, 觸發BacktoLine() */
void BacktoLine(){
  //  Serial.print(" Back2Line");
  //左移
  encoderCount_4 = 0; //reset count
  encoderCount_BeforeTurn4 = encoderCount_4; //record current position(angle)
  encoderCount_AfterTurn4 = encoderCount_BeforeTurn4 + 134;   //offset 112 pulse 
  while(encoderCount_4 < encoderCount_AfterTurn4 ){
    GoCrab_Left();
//    Serial.println("MOVE LEFT");
  }
  
  GoCrab_Right();  //反向剎車
  Stop();
//  Serial.print("Moveleft, count4= "); Serial.println(encoderCount_AfterTurn4);
  delay(200);

}


void byPass(){
  /* 過程中在行進路線上遇意外的障礙物
   * 必須要偵測出並且停止
   * 或走另一路徑繼續完成整個閃避動作
   * 側半邊測距讓AGV知道已經越過obstacle. //{...; ...; }
  */ 
  GoBackward();  //反向剎車
  Stop();
  delay(500);
//  Serial.println("now begin do byPass()");

  //先後退一小段
  encoderCount_3 = 0; //reset count
  encoderCount_BeforeTurn3 = encoderCount_3; //record current position(angle)
  encoderCount_AfterTurn3 = encoderCount_BeforeTurn3 + 28;   //offset 28 pulse 
  while(encoderCount_3 < encoderCount_AfterTurn3){
    GoBackward();
//    Serial.println("GOBACK");
  }
  
//  Serial.print("GoForward, count3= "); Serial.println(encoderCount_AfterTurn3);
  GoForward();  //反向剎車
  Stop();
  delay(500);
  
  //右移
  encoderCount_4 = 0; //reset count
  encoderCount_BeforeTurn4 = encoderCount_4; //record current position(angle)
  encoderCount_AfterTurn4 = encoderCount_BeforeTurn4 + 112;   //offset 112 pulse 
  while(encoderCount_4 < encoderCount_AfterTurn4 ){
    GoCrab_Right();
//    Serial.println("MOVE RIGHT");
  }
  
  GoCrab_Left();  //反向剎車
  Stop();
//  Serial.print("MoveRight, count4= "); Serial.println(encoderCount_AfterTurn4);
  delay(500);
  
  //往前走
  encoderCount_3 = 0; //reset count
  encoderCount_BeforeTurn3 = encoderCount_3; //record current position(angle)
  encoderCount_AfterTurn3 = encoderCount_BeforeTurn3 + 168;   //offset 2600 pulse 
  while(encoderCount_3 < encoderCount_AfterTurn3){
    slowGoForward();
//    Serial.println("SideGoForward");
  }
  
//  Serial.print("GoForward, count3= "); Serial.println(encoderCount_AfterTurn3);
  GoBackward();  //反向剎車
  Stop();
  delay(500);
  
  //左移
  encoderCount_4 = 0; //reset count
  encoderCount_BeforeTurn4 = encoderCount_4; //record current position(angle)
  encoderCount_AfterTurn4 = encoderCount_BeforeTurn4 + 112;   //offset 112 pulse 
  while(encoderCount_4 < encoderCount_AfterTurn4 ){
    GoCrab_Left();
//    Serial.println("MOVE LEFT");
  }
  
  GoCrab_Right();  //反向剎車
  Stop();
//  Serial.print("Moveleft, count4= "); Serial.println(encoderCount_AfterTurn4);
  delay(200);
  demoGoEnable = 1;
}

void turn180(){
  encoderCount_BeforeTurn4 = encoderCount_4;
  while( encoderCount_4 < encoderCount_BeforeTurn4+200){ //強制轉180度  
    Rotate_CCW();
//    Serial.println(encoderCount_4);
  }
  
  if( encoderCount_4 >= encoderCount_BeforeTurn4+200){ //=>"2920" with 11.1V Batt/ "3100" with 7.4V Batt
      Rotate_CW(); //反向剎車
      delay(100);
      Stop();
//      Serial.println("180~~~~~~~~~~~~~~~~~~~~~~~~~~~");
  }
  demoGoEnable = 1;
}

void startCalibrate()
{
    digitalWrite(onChargingPin, HIGH);
//    Serial.print("Calibrating ...");
    for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
    {        
        qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)      
    }
//    Serial.println("");    

    // print the calibration minimum values
//    Serial.print("Min : ");
    for (int i = 0; i < NUM_SENSORS; i++)
    {
//      Serial.print(qtrrc.calibratedMinimumOn[i]);
//      Serial.print(' ');
    }
//    Serial.println("");

    // print the calibration maximum values
//    Serial.print("Max : ");
    for (int i = 0; i < NUM_SENSORS; i++)
    {
//      Serial.print(qtrrc.calibratedMaximumOn[i]);
//      Serial.print(' ');
    }
//    Serial.println("Finish ...");
//    Serial.println();
//    Serial.println();

    digitalWrite(onChargingPin, LOW);
}

uint8_t Conversion()
{
    unsigned int tmp = 0;
    uint8_t value = 0;
    for (unsigned char i = 0; i < NUM_SENSORS; i++)
    {
        tmp = 0;
        if (sensorValues[i] > 100)
            tmp = ((sensorValues[i]/100)-1);
        else
            tmp = 0;

        if (tmp >= 5)
            value = value + (1<<(7-i));
    }

    return value;
}

void IRtracking(){
    unsigned int position = qtrrc.readLine(sensorValues);   // get sensor value
    
    /*  For debug and print all sensor values
     *  if debug OK comment below...          */
//    for (unsigned char i = 0; i < NUM_SENSORS; i++)
//    {
//        if (sensorValues[i] > 100)
//          Serial.print((sensorValues[i]/100)-1);
//        else
//          Serial.print("0");
//        
//        Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
//    }
//    Serial.print('|');
//    Serial.print(position); // comment this line out if you are using raw values
//
//    Serial.print('\t');
    
    QTR_value = Conversion(); // do NOT comment
//    Serial.println(Conversion(), HEX);


    /*for easy to quick do startCalibrate()*/
//    buttonState = digitalRead(buttonPin);
//    if (buttonState == HIGH) {
//      startCalibrate();
//    }

/*
0   0   0   0   0   0   0   1       0x01
0   0   0   0   0   0   1   1       0x03
0   0   0   0   0   0   1   0       0x02
0   0   0   0   0   1   1   0       0x06
0   0   0   0   0   1   0   0       0x04    往左偏
0   0   0   0   1   1   0   0       0x0C      ︽
0   0   0   0   1   0   0   0       0x08      ||
0   0   0   1   1   0   0   0       0x18    正中間
0   0   0   1   0   0   0   0       0x10      ||
0   0   1   1   0   0   0   0       0x30      ︾ 
0   0   1   0   0   0   0   0       0x20    往右偏
0   1   1   0   0   0   0   0       0x60
0   1   0   0   0   0   0   0       0x40
1   1   0   0   0   0   0   0       0xc0
1   0   0   0   0   0   0   0       0x80
*/

/* ---------------------------QTR compensation-----------------------------------
---------------------- Direction compensation by adding PWM--------------------*/
  switch(QTR_value){
    /* 往左偏, and more...*/
    case 0x08:    
        PWMcomp1 = 2.5;
        PWMcomp3 = -2.5;
//        Serial.println("0x08");
    break;
    
    case 0x0C:    
        PWMcomp1 = 5;
        PWMcomp3 = -5;
//        Serial.println("0x0C");
        digitalWrite(QTR_Indi_Led0, LOW); //A0
        digitalWrite(QTR_Indi_Led1, LOW); //A1
        digitalWrite(QTR_Indi_Led2, HIGH); //A2
        digitalWrite(QTR_Indi_Led3, LOW); //A3
        digitalWrite(QTR_Indi_Led4, LOW); //A4
        digitalWrite(QTR_Indi_Led5, LOW); //A5
        digitalWrite(QTR_Indi_Led6, LOW); //A6
    break;
    
    case 0x04:    
        PWMcomp1 = 10;
        PWMcomp3 = -10;
//        Serial.println("0x04");
    break;
    
    case 0x06:    
        PWMcomp1 = 12.5;
        PWMcomp3 = -12.5;
//        Serial.println("0x06");
        digitalWrite(QTR_Indi_Led0, LOW); //A0
        digitalWrite(QTR_Indi_Led1, HIGH); //A1
        digitalWrite(QTR_Indi_Led2, LOW); //A2
        digitalWrite(QTR_Indi_Led3, LOW); //A3
        digitalWrite(QTR_Indi_Led4, LOW); //A4
        digitalWrite(QTR_Indi_Led5, LOW); //A5
        digitalWrite(QTR_Indi_Led6, LOW); //A6
    break; 
    
    case 0x02:    
        PWMcomp1 = 15;
        PWMcomp3 = -15;
//        Serial.println("0x02");
    break;
    
    case 0x03:    
        PWMcomp1 = 17.5;
        PWMcomp3 = -17.5;
//        Serial.println("0x03");
        digitalWrite(QTR_Indi_Led0, HIGH); //A0
        digitalWrite(QTR_Indi_Led1, LOW); //A1
        digitalWrite(QTR_Indi_Led2, LOW); //A2
        digitalWrite(QTR_Indi_Led3, LOW); //A3
        digitalWrite(QTR_Indi_Led4, LOW); //A4
        digitalWrite(QTR_Indi_Led5, LOW); //A5
        digitalWrite(QTR_Indi_Led6, LOW); //A6
    break;
    
    case 0x01:    
        PWMcomp1 = 20;
        PWMcomp3 = -20;
//        Serial.println("0x01");
    break;
    /* 往右偏, and more...*/
    case 0x10:    
        PWMcomp1 = -2.5;
        PWMcomp3 = 2.5;
//        Serial.println("0x10");
    break;
    
    case 0x30:    
        PWMcomp1 = -5;
        PWMcomp3 = 5;
//        Serial.println("0x30");
        digitalWrite(QTR_Indi_Led0, LOW); //A0
        digitalWrite(QTR_Indi_Led1, LOW); //A1
        digitalWrite(QTR_Indi_Led2, LOW); //A2
        digitalWrite(QTR_Indi_Led3, LOW); //A3
        digitalWrite(QTR_Indi_Led4, HIGH); //A4
        digitalWrite(QTR_Indi_Led5, LOW); //A5
        digitalWrite(QTR_Indi_Led6, LOW); //A6
    break;
    
    case 0x020:    
        PWMcomp1 = -10;
        PWMcomp3 = 10;
//        Serial.println("0x020"); 
    break;
    
    case 0x60:    
        PWMcomp1 = -12.5;
        PWMcomp3 = 12.5;
//        Serial.println("0x60");
        digitalWrite(QTR_Indi_Led0, LOW); //A0
        digitalWrite(QTR_Indi_Led1, LOW); //A1
        digitalWrite(QTR_Indi_Led2, LOW); //A2
        digitalWrite(QTR_Indi_Led3, LOW); //A3
        digitalWrite(QTR_Indi_Led4, LOW); //A4
        digitalWrite(QTR_Indi_Led5, HIGH); //A5
        digitalWrite(QTR_Indi_Led6, LOW); //A6
    break;
    
    case 0x40:    
        PWMcomp1 = -15;
        PWMcomp3 = 15;
//        Serial.println("0x40");
    break;
    
    case 0xc0:    
        PWMcomp1 = -17.5;
        PWMcomp3 = 17.5;
//        Serial.println("0xc0");
        digitalWrite(QTR_Indi_Led0, LOW); //A0
        digitalWrite(QTR_Indi_Led1, LOW); //A1
        digitalWrite(QTR_Indi_Led2, LOW); //A2
        digitalWrite(QTR_Indi_Led3, LOW); //A3
        digitalWrite(QTR_Indi_Led4, LOW); //A4
        digitalWrite(QTR_Indi_Led5, LOW); //A5
        digitalWrite(QTR_Indi_Led6, HIGH);//A6
    break;
    
    case 0x80:    
        PWMcomp1 = -20;
        PWMcomp3 = 20;
//        Serial.println("0x80");
    break;

    /* On the middle of line*/
    case 0x18: 
        PWMcomp1 = 0;
        PWMcomp3 = 0;
//        Serial.println("0x18");
        digitalWrite(QTR_Indi_Led0, LOW); //A0
        digitalWrite(QTR_Indi_Led1, LOW); //A1
        digitalWrite(QTR_Indi_Led2, LOW); //A2
        digitalWrite(QTR_Indi_Led3, HIGH); //A3
        digitalWrite(QTR_Indi_Led4, LOW); //A4
        digitalWrite(QTR_Indi_Led5, LOW); //A5
        digitalWrite(QTR_Indi_Led6, LOW); //A6
    break;
    
    /* Not on the line*/
    default:
        PWMcomp1 = 0;
        PWMcomp3 = 0;
//        Serial.println("Null, QTR_value");
        digitalWrite(QTR_Indi_Led0, LOW); //A0
        digitalWrite(QTR_Indi_Led1, LOW); //A1
        digitalWrite(QTR_Indi_Led2, LOW); //A2
        digitalWrite(QTR_Indi_Led3, LOW); //A3
        digitalWrite(QTR_Indi_Led4, LOW); //A4
        digitalWrite(QTR_Indi_Led5, LOW); //A5
        digitalWrite(QTR_Indi_Led6, LOW); //A6
    break;
  }
}


void findNearTerm(){
  float midPos;

  // 只留後面2字元
  XBeeStrSub = XBeeStr.substring(4); 
  barcodeNowSub = barcodeNow.substring(4); 
  UpTerSub = UpTerminal.substring(4);
  DownTerSub = DownTerminal.substring(4);
  
  // Str to Int
  XB2Int = XBeeStrSub.toInt();
  barcode2Int = barcodeNowSub.toInt();
  UpTer2Int = UpTerSub.toInt();  //--> 13
  DownTer2Int = DownTerSub.toInt(); //--> 1
  
  // Determine The Nearest Terminal
  midPos = (XB2Int+barcode2Int)/2 ; //兩AGV中間位置
  if(abs(UpTer2Int-midPos) >= abs(DownTer2Int-midPos))
    nearTerminal = DownTer2Int; // 獲得最靠近的是DownTerminal
  else
    nearTerminal = UpTer2Int; // 獲得最靠近的是UpTerminal

}
