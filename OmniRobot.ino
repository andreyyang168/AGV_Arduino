#include <AFMotor.h>                // L293 shield library
#include <PID_v1.h>                 // PID library
#include <IRremote.h>               // IR Remote Control library
#include <Ultrasonic.h>             // Ultrasonic:SR04 library
#include <DistanceGP2Y0A41SK.h>    //Infrared Proximity Sensor Unit librairy, Measuring distance : 4 to 30 cm
#include <Timer.h>                 // Timer library to do several tasks in a period
#include <SPI.h>                   // SPI library for MFRC522
#include <MFRC522.h>            //RFID module library
#include <math.h>                  //Math library to calculate acos()
/*****************  Motor Declaration****************/
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

/*****************  Motor velocity Variables ****************/
unsigned long sample_time = 0;
unsigned long old_time = 0;
float D = 4.77; // wheel diameter in cm 
float velocity1 = 0;
float angular_veocity1 = 0;
float velocity2 = 0;
float angular_veocity2 = 0;
float velocity3 = 0;
float angular_veocity3 = 0;
float velocity4 = 0;
float angular_veocity4 = 0;
int GoForwardEnable = 0, GoBackwardEnable = 0;
int GoCrab_LeftEnable = 0, GoCrab_RightEnable = 0;
int Rotate_CWEnable = 0, Rotate_CCWEnable = 0;
int R45_ForwardEnable = 0, L45_ForwardEnable = 0, R45_BackwardEnable = 0, L45_BackwardEnable = 0;

/***************** Encoder setting, respected to AF_DCMotor ******************/
#define encoderPinA_2  18          // Connect encoder1 wire A to pin 18
#define encoderPinB_2  33          // Connect encoder1 wire B to pin 33
#define encoderPinA_3  19          // Connect encoder2 wire A to pin 19
#define encoderPinB_3  34          // Connect encoder2 wire B to pin 34
#define encoderPinA_4  20          // Connect encoder3 wire A to pin 20
#define encoderPinB_4  35          // Connect encoder3 wire B to pin 35
volatile long encoderCount_2 = 0;  // Counting encoder pulses
long encoderCountOld_2 = 0;
volatile long encoderCount_3 = 0;  // Counting encoder pulses
long encoderCountOld_3 = 0;
volatile long EncoderCount_3 = 0;  // Counting encoder pulses
long EncoderCountOld_3 = 0;
volatile long encoderCount_4 = 0;  // Counting encoder pulses
long encoderCountOld_4 = 0;
long encoderCount_BeforeTurn = 0;
long encoderCount_AfterTurn = 0;

/*****************  PID setting & Declaration****************/
double Setpoint_2,Setpoint_3, Setpoint_4, Input_2, Output_2, Input_3, Output_3, Input_4, Output_4;   
//double Kp=2, Ki=5, Kd=1;          // PID perameters     
//double Kp=12, Ki=10.2, Kd=0.85;          // PID perameters  
double Kp=20, Ki=33, Kd=8.5;          // PID perameters  
int outVal2, outVal3, outVal4; // PID output constrain value for PWM
PID PID_2(&Input_2, &Output_2, &Setpoint_2, Kp, Ki, Kd, DIRECT);
PID PID_3(&Input_3, &Output_3, &Setpoint_3, Kp, Ki, Kd, DIRECT);
PID PID_4(&Input_4, &Output_4, &Setpoint_4, Kp, Ki, Kd, DIRECT);

/*****************  IR Remote Control setting ****************/
int RECV_PIN1 = 44;
int RECV_PIN2 = 45;
int RECV_PIN3 = 46;
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
float cmMsec_F, cmMsec_R, cmMsec_L;
int goDetectFlag = 0;

/*****************  Infrared Proximity Sensor Declaration ****************/
DistanceGP2Y0A41SK IRdist_RF;   //Right Front  direction
DistanceGP2Y0A41SK IRdist_RM;   //Right Middle direction
DistanceGP2Y0A41SK IRdist_RB;   //Right Back   direction
DistanceGP2Y0A41SK IRdist_LF;   //Left  Front  direction
DistanceGP2Y0A41SK IRdist_LM;   //Left  Middle direction
DistanceGP2Y0A41SK IRdist_LB;   //Left  Back   direction
int distance_RF, distance_RM, distance_RB; // right side distance
int distance_LF, distance_LM, distance_LB; //  left side distance
//給定模組方位
float IRangle_RF = 3.1416/6;   //  30
float IRangle_RM = 3.1416/2;   //  90
float IRangle_RB = 3.1416*5/6; // 150 
float IRangle_LF =-3.1416/3;   // -30
float IRangle_LM =-3.1416/2;   // -90
float IRangle_LB =-3.1416*5/6; //-150
int IRdist_RF_pick, IRdist_RM_pick, IRdist_RB_pick; //which IR be choosen
int IRdist_LF_pick, IRdist_LM_pick, IRdist_LB_pick; //which IR be choosen
int L_Obs; //Length of obstacle
int distcmp1, distcmp2, distcmp3, distcmp4; // comparison results
int IRdist_min, IRdist_min2; //min distance and second min distance
double h, theta_rad, theta_Orient;

/*****************  Infrared Proximity Sensor Declaration ****************/
Timer timer_Velocity;
Timer timer_SR04;
Timer timer_IRdist;
Timer timer_RFIDread;

/*****************  BTN Declaration ****************/
const int AvoidingBtnPin = 49;
int autoAvoidingFlag = 0;
const int DemoBtnPin = 48;
int demoFlag = 0;

/*****************  RFID(NFC) Declaration ****************/
#define RST_PIN   41     // Digital_Output: HIGH/LOW
#define SS_PIN    53     //
MFRC522 mfrc522(SS_PIN, RST_PIN);  // Create MFRC522 instance
byte goodCard1[10]= {0x09, 0x22, 0xe7, 0x45}; //Tag UID
byte goodCard2[10]= {0xc1, 0x40, 0xdc, 0x0e}; //Tag UID



void setup(){   
  Serial.begin(9600);
  Serial.println("t, angleV2, vel2, out_2, angleV3, vel3, out_3, angleV3, vel3, out_3,");
  //-------------IR recieve setting-------------------------------------------
  irrecv1.enableIRIn(); // Start the receiver
  irrecv2.enableIRIn(); // Start the receiver
  irrecv3.enableIRIn(); // Start the receiver
  //-------------Encoder Interrupt--------------------------------------------
  pinMode(encoderPinA_2, INPUT); 
  pinMode(encoderPinB_2, INPUT); 
  digitalWrite(encoderPinA_2, HIGH);       // turn on pullup resistor
  digitalWrite(encoderPinB_2, HIGH);       // turn on pullup resistor
  pinMode(encoderPinA_3, INPUT); 
  pinMode(encoderPinB_3, INPUT); 
  digitalWrite(encoderPinA_3, HIGH);       // turn on pullup resistor
  digitalWrite(encoderPinB_3, HIGH);       // turn on pullup resistor
  pinMode(encoderPinA_4, INPUT); 
  pinMode(encoderPinB_4, INPUT); 
  digitalWrite(encoderPinA_4, HIGH);       // turn on pullup resistor
  digitalWrite(encoderPinB_4, HIGH);       // turn on pullup resistor
  attachInterrupt(digitalPinToInterrupt(encoderPinA_2), EncoderISR_2, FALLING);  // encoder pin on interrupt 5 --> pin 18
  attachInterrupt(digitalPinToInterrupt(encoderPinA_3), EncoderISR_3, FALLING);  // encoder pin on interrupt 4 --> pin 19
  attachInterrupt(digitalPinToInterrupt(encoderPinA_4), EncoderISR_4, FALLING);  // encoder pin on interrupt 3 --> pin 20
  //-------------PID--------------------------------------------
  PID_2.SetMode(AUTOMATIC);
  PID_3.SetMode(AUTOMATIC);
  PID_4.SetMode(AUTOMATIC);
  //-------------Motor Initialize-------------------------------
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
  //-------------Infrared Proximity Sensor Initialize-----------
  IRdist_RF.begin(A10);  
  IRdist_RM.begin(A9);
  IRdist_RB.begin(A8);
  IRdist_LF.begin(A11);
  IRdist_LM.begin(A12);
  IRdist_LB.begin(A13);
  //-------------Timer event Setting----------------------------
  timer_Velocity.every(200, calVelocity); //每經過200毫秒，就會呼叫 calVelocity
  timer_SR04.every(500, ultrasonicDetect); //每經過500毫秒，就會呼叫 ultrasonicDetect
  timer_IRdist.every(300, IR_distance); //每經過300毫秒，就會呼叫 IR_distance
  timer_RFIDread.every(100, MFRC522read); //每經過100毫秒，就會呼叫 IR_distance
  //-------------btn Setting----------------------------
  pinMode(AvoidingBtnPin, INPUT);
  pinMode(DemoBtnPin, INPUT);
  //-------------RFID(NFC) Setting----------------------------
  SPI.begin();      // Init SPI bus
  mfrc522.PCD_Init();   // Init MFRC522
  Serial.println("Try the most used default keys to print block 0 of a MIFARE PICC.");
  
}



void loop(){
/* --------------------------Update timer even---------------------------
-----------------------------------------------------------------------*/
  timer_Velocity.update(); //calVelocity 200ms
  timer_SR04.update();     //ultrasonicDetect 500ms
  timer_IRdist.update();   //IR_distance 300ms
  timer_RFIDread.update(); //MFRC522 RFID 100ms
  
/* ------------------------------RFID------------------------------------------
----Memory Comparison(Array1,Array2,Aize of Bytes), the same => return 0-----*/
  if(memcmp(goodCard1, mfrc522.uid.uidByte, mfrc522.uid.size) == 0)
  {
    if(mfrc522.uid.size != 0){      // double check
    Serial.print("GOODCARD1,  ");
    Serial.println(mfrc522.uid.size);
    GoBackward();
    }
  }
  else if(memcmp(goodCard2, mfrc522.uid.uidByte, mfrc522.uid.size) == 0)
  {
    if(mfrc522.uid.size != 0){      // double check
    Serial.print("GOODCARD2,  ");
    Serial.println(mfrc522.uid.size);
    GoForward();
    }
  }
  else
  {
    Serial.print("BAD,  ");
    Serial.println(mfrc522.uid.size);
    Stop();
  }
  
/* ------------------IR distance and orientation-------------------------
-----------------IR 偵測只判斷轉向--SR04負責AGV逼近Tag--------------------*/
//// 在這之前須按下BTN啟動這個DEMO模式

    if(distance_RF<=8 || distance_RM<=8 || distance_RB<=8 || distance_LF<=8 || distance_LM<=8 || distance_LB<=8){  // 臨界距離=8
      if(distance_RF<=8) IRdist_RF_pick = 1; else IRdist_RF_pick = 0;
      if(distance_RM<=8) IRdist_RM_pick = 1; else IRdist_RM_pick = 0;
      if(distance_RB<=8) IRdist_RB_pick = 1; else IRdist_RB_pick = 0;
      if(distance_LF<=8) IRdist_LF_pick = 1; else IRdist_LF_pick = 0;
      if(distance_LM<=8) IRdist_LM_pick = 1; else IRdist_LM_pick = 0;
      if(distance_LB<=8) IRdist_LB_pick = 1; else IRdist_LB_pick = 0;
      Stop();
      Serial.println("detected directions ====> stop");
      /*---- 只有1個方向<=臨界距離----*/
      //-------------- only右前方,30度 --------------
      if(IRdist_RF_pick ==1 && IRdist_RM_pick == 0 && IRdist_RB_pick == 0 && IRdist_LF_pick == 0 && IRdist_LM_pick == 0 && IRdist_LB_pick == 0){
        encoderCount_BeforeTurn = encoderCount_2;  //record current position(angle)
        encoderCount_AfterTurn = (17.5/2.4)*(56/360)*30 + encoderCount_BeforeTurn;   //Obstacle angle, degree=>pulse
        Serial.println("30 degree...30 degree...30 degree...");      
        //-------- while loop 直到判斷false才會跳出 -----------------
        while(encoderCount_2 < encoderCount_AfterTurn ){
          Rotate_CW();  //<===需要更改為更好的轉向動作
          Serial.println("while....while....while....");
        }
        goDetectFlag = 1;
      } //end only右前方,30度
      
      //-------------- only左前方,30度 --------------
      else if(IRdist_RF_pick ==0 && IRdist_RM_pick == 0 && IRdist_RB_pick == 0 && IRdist_LF_pick == 1 && IRdist_LM_pick == 0 && IRdist_LB_pick == 0){
        encoderCount_BeforeTurn = encoderCount_2;  //record current position(angle)
        encoderCount_AfterTurn = (17.5/2.4)*(56/360)*30 + encoderCount_BeforeTurn;   //Obstacle angle, degree=>pulse
        Serial.println("30 degree...30 degree...30 degree...");      
        //-------- while loop 直到判斷false才會跳出 -----------------
        while(encoderCount_2 < encoderCount_AfterTurn ){
          Rotate_CCW();  //<===需要更改為更好的轉向動作
          Serial.println("while....while....while....");
        }
        goDetectFlag = 1;
      } //end only左前方,30度
      
      /*---- 有2個方向<=臨界距離----*/
      //-------------- 障礙位於-30~30度的方位 --------------
      else if(IRdist_RF_pick ==1 && IRdist_RM_pick == 0 && IRdist_RB_pick == 0 && IRdist_LF_pick == 1 && IRdist_LM_pick == 0 && IRdist_LB_pick == 0){
        encoderCount_BeforeTurn = encoderCount_2;  //record current position
        L_Obs = sqrt( sq(distance_LF)+sq(distance_RF)-2*distance_LF*distance_RF*cos(3.1416/3) ); // Cos Law, 鄰近IR相距60度=pi/3
        h = distance_LF*distance_RF*sin(3.1416/3)/L_Obs; // 鄰近IR相距60度=pi/3 
         
        if(distance_LF < distance_RF){  //遇到左斜面
          theta_rad = acos(h/distance_LM); //in radians
          theta_Orient = (3.1416/6) - theta_rad; // theta+pi/6 in rad 
          encoderCount_AfterTurn = ( (17.5/2.4)*(56/360)*theta_Orient*(180/3.1416) )+ encoderCount_BeforeTurn;   //Obstacle angle, rad=>degree=>pulse
          Serial.println("-30 ~ 0 degree...distance_LF < distance_RF");
          while(encoderCount_2 < encoderCount_AfterTurn ){
            Rotate_CCW();  //<===需要更改為更好的轉向動作
            Serial.println("while....while....while....");
          }
        }
        else{  //遇到右斜面
          theta_rad = acos(h/distance_RF); //in radians
          theta_Orient = (3.1416/6) - theta_rad; // theta+pi/6 in rad 
          encoderCount_AfterTurn = ( (17.5/2.4)*(56/360)*theta_Orient*(180/3.1416) )+ encoderCount_BeforeTurn;   //Obstacle angle, rad=>degree=>pulse
          Serial.println("0 ~ 30 degree...distance_LF > distance_RF");
          while(encoderCount_2 < encoderCount_AfterTurn ){
            Rotate_CW();  //<===需要更改為更好的轉向動作
            Serial.println("while....while....while....");
          }
        }
        goDetectFlag = 1;   
      } //end 障礙位於-30-30度的方位轉向 
  
         
      //-------------- 障礙位於30~90度的方位 --------------
      else if(IRdist_RF_pick ==1 && IRdist_RM_pick == 1 && IRdist_RB_pick == 0 && IRdist_LF_pick == 0 && IRdist_LM_pick == 0 && IRdist_LB_pick == 0){
        encoderCount_BeforeTurn = encoderCount_2;  //record current position
        L_Obs = sqrt( sq(distance_RF)+sq(distance_RM)-2*distance_RF*distance_RM*cos(3.1416/3) ); // Cos Law, 鄰近IR相距60度=pi/3
        h = distance_RF*distance_LF*sin(3.1416/3)/L_Obs; // 鄰近IR相距60度=pi/3
        theta_rad = acos(h/distance_RF); //in radians
        theta_Orient = theta_rad+(3.1416/6); // theta+pi/6 in rad     
        encoderCount_AfterTurn = ( (17.5/2.4)*(56/360)*theta_Orient*(180/3.1416) )+ encoderCount_BeforeTurn;   //Obstacle angle, rad=>degree=>pulse
        Serial.println("30 ~ 90 degree...30 ~ 90 degree...");      
        //-------- while loop 直到判斷false才會跳出 -----------------
        while(encoderCount_2 < encoderCount_AfterTurn ){
          Rotate_CW();  //<===需要更改為更好的轉向動作
          
          Serial.println("while....while....while....");
        }
        goDetectFlag = 1;
      } //end 障礙位於30-90度的方位轉向
  
      //-------------- 障礙位於-90~-30度的方位 --------------
      else if(IRdist_RF_pick ==0 && IRdist_RM_pick == 0 && IRdist_RB_pick == 0 && IRdist_LF_pick == 1 && IRdist_LM_pick == 1 && IRdist_LB_pick == 0){
        encoderCount_BeforeTurn = encoderCount_2;  //record current position
        L_Obs = sqrt( sq(distance_LF)+sq(distance_LM)-2*distance_LF*distance_LM*cos(3.1416/3) ); // Cos Law, 鄰近IR相距60度=pi/3
        h = distance_LF*distance_LM*sin(3.1416/3)/L_Obs; // 鄰近IR相距60度=pi/3
        theta_rad = acos(h/distance_LF); //in radians
        theta_Orient = theta_rad+(3.1416/6); // theta+pi/6 in rad     
        encoderCount_AfterTurn = ( (17.5/2.4)*(56/360)*theta_Orient*(180/3.1416) )+ encoderCount_BeforeTurn;   //Obstacle angle, rad=>degree=>pulse
        Serial.println("-90 ~ 30 degree...-90 ~ 30 degree...");      
        //-------- while loop 直到判斷false才會跳出 -----------------
        while(encoderCount_2 < encoderCount_AfterTurn ){
          Rotate_CCW();  //<===需要更改為更好的轉向動作
          Serial.println("while....while....while....");
        }
        goDetectFlag = 1;
      } //end 障礙位於-90~-30度的方位轉向

    }//end <臨界距離
  
    
  
/* ------------------------SR04 前進讀Tag--------------------------------
-----------------------------------------------------------------------*/
  if(goDetectFlag == 1){
    // 轉向後正面距離Tag太遠 & 沒讀到RFID =>前進
    if(cmMsec_F > 0.5 && memcmp(goodCard1, mfrc522.uid.uidByte, mfrc522.uid.size) != 0 && mfrc522.uid.size != 0){
      GoForward();
    } 
    // 轉向後正面距離Tag OK, but 沒讀到RFID =>左右移動
    else if(cmMsec_F <= 0.5 && memcmp(goodCard1, mfrc522.uid.uidByte, mfrc522.uid.size) != 0 && mfrc522.uid.size != 0){
      GoCrab_Search(); // 左右移動動作
    }
    else goDetectFlag = 0;
  } //end goDetectFlag = 1
  
/* --------------------------IR Receive----------------------------------
-----------------------------------------------------------------------*/
  if (irrecv1.decode(&results) || irrecv2.decode(&results) || irrecv3.decode(&results)) {
    if(results.value != 0xFFFFFFFF){
      IRincome = results.value;
    } 
    switch(IRincome){
      case 0x97483BFB:    //前
        Stop();
        GoForwardEnable = 1;
        Serial.println("IRrecv.: GoForward");
      break;
      
      case 0x488F3CBB:  //後
        Stop();
        GoBackwardEnable = 1;
        Serial.println("IRrecv.:GoBackward");
      break;
      
      case 0x9716BE3F:  //螃蟹左
        Stop();
        GoCrab_LeftEnable = 1;
        Serial.println("IRrecv.:GoCrab_Left");
      break;
      
      case 0x6182021B:  //螃蟹右
        Stop();
        GoCrab_RightEnable = 1;
        Serial.println("IRrecv.:GoCrab_Right");
      break;

      case 0x3D9AE3F7:  //Release, setSpeed(0)=run(RELEASE)
        Stop();
        Serial.println("IRrecv.:Stop");
      break;
      
      case 0x3EC3FC1B:  //右自轉
        Stop();
        Rotate_CWEnable = 1;
        Serial.println("IRrecv.:Rotate_CW");
      break;
      
      case 0x32C6FDF7:  //左自轉
        Stop();
        Rotate_CCWEnable = 1;
        Serial.println("IRrecv.:Rotate_CCW");
      break;

      case 0xF0C41643:  //Right Forward
        Stop();
        R45_ForwardEnable = 1;
        Serial.println("IRrecv.:R45_ForwardEnable");
      break;

      case 0xC101E57B:  //Left Forward
        Stop();
        L45_ForwardEnable = 1;
        Serial.println("IRrecv.:L45_ForwardEnable");
      break;

      case 0x449E79F:  //Right Backward
        Stop();
        R45_BackwardEnable = 1;
        Serial.println("IRrecv.:R45_BackwardEnable");
      break;

      case 0x8C22657B:  //Left Backward
        Stop();
        L45_BackwardEnable = 1;
        Serial.println("IRrecv.:L45_BackwardEnable");
      break;
    }  
    irrecv1.resume(); // Receive the next value
    irrecv2.resume(); // Receive the next value
    irrecv3.resume(); // Receive the next value
  }
//------------IR controller response---------------------------------------
//-------------------------------------------------------------------------
  if(GoForwardEnable == 1){
    GoForward();
    Serial.println("IRresp.: GoForward");
  }
  else if(GoBackwardEnable == 1){
    GoBackward();
    Serial.println("IRresp.: GoBackward");
  }
  else if(GoCrab_LeftEnable == 1){
    GoCrab_Left();
    Serial.println("IRresp.: GoCrab_Left");
  }
  else if(GoCrab_RightEnable == 1){
    GoCrab_Right();
    Serial.println("IRresp.: GoCrab_Right");
  }
  else if(Rotate_CWEnable == 1){
    Rotate_CW();
    Serial.println("IRresp.: Rotate_CW");
  }
  else if(Rotate_CCWEnable == 1){
    Rotate_CCW();
    Serial.println("IRresp.: Rotate_CCW");
  }
  else if(R45_ForwardEnable == 1){
    R45_Forward();
    Serial.println("IRresp.: R45_ForwardEnable");
  }
  else if(L45_ForwardEnable == 1){
    L45_Forward();
    Serial.println("IRresp.: L45_ForwardEnable");
  }
  else if(R45_BackwardEnable == 1){
    R45_Backward();
    Serial.println("IRresp.: R45_BackwardEnable");
  }
  else if(L45_BackwardEnable == 1){
    L45_Backward();
    Serial.println("IRresp.: L45_BackwardEnable");
  }
  
//------------------------btn response---------------------------
//---------------------------------------------------------------
  if(digitalRead(AvoidingBtnPin) == HIGH){
    autoAvoidingFlag = 1;
    Serial.println("autoAvoidingFlag = 1");
  } //End: digitalRead(AvoidingBtnPin) == HIGH
  
  if(digitalRead(DemoBtnPin) == HIGH){
    demoFlag = 1;
    Serial.println("demoFlag = 1");
  } //End: digitalRead(DemoBtnPin) == HIGH

  
  if(autoAvoidingFlag == 1){  
      if(distance_LF >= 15 && distance_LF >= 15){ // 直行
        GoForwardEnable = 1;
        Rotate_CCWEnable = 0;
        Rotate_CWEnable = 0;
        Serial.println("distance_LF >= 15,GoGoGoGoGoGoForward");
      }
      if(distance_LF <= 15 && distance_LF<distance_RF){ //左前逼近牆, 右轉
        GoForwardEnable = 0;
        Rotate_CCWEnable = 0;
        Rotate_CWEnable = 1;
        Serial.println("distance_LF <= 15,CW_CW_CW_CW_CW_CW_CW_");
      }
      if(distance_RF <= 15 && distance_LF>distance_RF){ //右前逼近牆, 左轉
        GoForwardEnable = 0;
        Rotate_CCWEnable = 1;
        Rotate_CWEnable = 0;
        Serial.println("distance_RF <= 15,CCW_CCW_CCW_CCW_CCW_CCW_");
      }
  } //End: autoAvoidingFlag == 1

  
} // loop()  end


/*****************  Call Back Function ****************/
/*****************  Motor behavier ****************/
void GoForward() {
        Setpoint_2 = 17; // cm/sec
        Setpoint_3 = 17;   
        motor2.run(BACKWARD);
//        PID_2.Compute();
//        outVal2 =  constrain(Output_2, 0, 50);
//        motor2.setSpeed(100 + outVal2);
        motor2.setSpeed(129);
//        motor2.setSpeed(170);
        motor3.run(FORWARD);
//        PID_3.Compute();
//        outVal3 =  constrain(Output_3, 0, 50);
//        motor3.setSpeed(100 + outVal3);
        motor3.setSpeed(130);
        motor4.run(RELEASE);
}
void GoBackward() {
        Setpoint_2 = 17; // cm/sec
        Setpoint_3 = 17;   
        motor2.run(FORWARD);
//        PID_2.Compute();
//        outVal2 =  constrain(Output_2, 0, 50);
//        motor2.setSpeed(100 + outVal2);
//        motor2.setSpeed(129);
        motor2.setSpeed(170);
        motor3.run(BACKWARD);
//        PID_3.Compute();
//        outVal3 =  constrain(Output_3, 0, 50);
//        motor3.setSpeed(97 + outVal3);
        motor3.setSpeed(130);
        motor4.run(RELEASE);
}
void GoCrab_Right() {
        Setpoint_2 = 7; // cm/sec
        Setpoint_3 = 7;
        Setpoint_4 = 14;
        motor2.run(BACKWARD);
        PID_2.Compute();
        outVal2 =  constrain(Output_2, 0, 50);
//        motor2.setSpeed(72 + outVal2);
        motor2.setSpeed(95);
        motor3.run(BACKWARD);
        PID_3.Compute();
        outVal3 =  constrain(Output_3, 0, 50);
//        motor3.setSpeed(73 + outVal3);
        motor3.setSpeed(95);
        motor4.run(FORWARD);
        PID_4.Compute();
        outVal4 =  constrain(Output_4, 0, 50);
//        motor4.setSpeed(200 + outVal4);
        motor4.setSpeed(192);
}
void GoCrab_Left() {
        Setpoint_2 = 7; // cm/sec
        Setpoint_3 = 7;
        Setpoint_4 = 14;
        motor2.run(FORWARD);
//        PID_2.Compute();
//        outVal2 =  constrain(Output_2, 0, 50);
//        motor2.setSpeed(72 + outVal2);
        motor2.setSpeed(95);
        motor3.run(FORWARD);
//        PID_3.Compute();
//        outVal3 =  constrain(Output_3, 0, 50);
//        motor3.setSpeed(74 + outVal3);
        motor3.setSpeed(95);
        motor4.run(BACKWARD);
//        PID_4.Compute();
//        outVal4 =  constrain(Output_4, 0, 50);
//        motor4.setSpeed(200 + outVal4);
        motor4.setSpeed(192);
}
void Rotate_CCW() {
        motor2.run(FORWARD);
        motor2.setSpeed(110);
        motor3.run(FORWARD);
        motor3.setSpeed(110);
        motor4.run(FORWARD);
        motor4.setSpeed(110);
}
void Rotate_CW() {
        motor2.run(BACKWARD);
        motor2.setSpeed(110);
        motor3.run(BACKWARD);
        motor3.setSpeed(110);
        motor4.run(BACKWARD);
        motor4.setSpeed(110);
}
void R45_Forward() {
        Setpoint_2 = 11.2; // cm/sec
        Setpoint_3 = 3;
        Setpoint_4 = 8.2;
        motor2.run(BACKWARD);
//        PID_2.Compute();
//        outVal2 =  constrain(Output_2, 0, 50);
//        motor2.setSpeed(60 + outVal2);
        motor2.setSpeed(170); 
        
        motor3.run(FORWARD);
//        PID_3.Compute();
//        outVal3 =  constrain(Output_3, 0, 50);
//        motor3.setSpeed(35 + outVal3);
        motor3.setSpeed(85);
        
        motor4.run(FORWARD);
//        PID_4.Compute();
//        outVal4 =  constrain(Output_4, 0, 50);
//        motor4.setSpeed(40 + outVal4);
        motor4.setSpeed(110);
}
void L45_Forward() {
        Setpoint_2 = 3; // cm/sec
        Setpoint_3 = 11.2;
        Setpoint_4 = 8.2;
        motor2.run(BACKWARD);
//        PID_2.Compute();
//        outVal2 =  constrain(Output_2, 0, 50);
//        motor2.setSpeed(60 + outVal2);
        motor2.setSpeed(85); 
        
        motor3.run(FORWARD);
//        PID_3.Compute();
//        outVal3 =  constrain(Output_3, 0, 50);
//        motor3.setSpeed(35 + outVal3);
        motor3.setSpeed(170);
        
        motor4.run(BACKWARD);
//        PID_4.Compute();
//        outVal4 =  constrain(Output_4, 0, 50);
//        motor4.setSpeed(40 + outVal4);
        motor4.setSpeed(110);
}
void R45_Backward() {
//        Setpoint_2 = 11.2; // cm/sec
//        Setpoint_3 = 3;
//        Setpoint_4 = 8.2;
        Setpoint_2 = 11.2; // cm/sec
        Setpoint_3 = 3;
        Setpoint_4 = 8.2;
        motor2.run(FORWARD);
//        PID_2.Compute();
//        outVal2 =  constrain(Output_2, 0, 50);
//        motor2.setSpeed(60 + outVal2);
        motor2.setSpeed(85); 
        
        motor3.run(BACKWARD);
//        PID_3.Compute();
//        outVal3 =  constrain(Output_3, 0, 50);
//        motor3.setSpeed(35 + outVal3);
        motor3.setSpeed(170);
        
        motor4.run(FORWARD);
//        PID_4.Compute();
//        outVal4 =  constrain(Output_4, 0, 50);
//        motor4.setSpeed(40 + outVal4);
        motor4.setSpeed(110);
}
void L45_Backward() {
//        Setpoint_2 = 11.2; // cm/sec
//        Setpoint_3 = 3;
//        Setpoint_4 = 8.2;
        Setpoint_2 = 11.2; // cm/sec
        Setpoint_3 = 3;
        Setpoint_4 = 8.2;
        motor2.run(FORWARD);
//        PID_2.Compute();
//        outVal2 =  constrain(Output_2, 0, 50);
//        motor2.setSpeed(60 + outVal2);
        motor2.setSpeed(170); 
        
        motor3.run(BACKWARD);
//        PID_3.Compute();
//        outVal3 =  constrain(Output_3, 0, 50);
//        motor3.setSpeed(35 + outVal3);
        motor3.setSpeed(85);
        
        motor4.run(BACKWARD);
//        PID_4.Compute();
//        outVal4 =  constrain(Output_4, 0, 50);
//        motor4.setSpeed(40 + outVal4);
        motor4.setSpeed(110);
}
void GoCrab_Search(){
  int RightEndTermonal = 0;
  int LeftEndTermonal = 0;
  encoderCount_BeforeTurn = encoderCount_2;
  if(encoderCount_2 < encoderCount_BeforeTurn+56){  //先往右跑搜尋
    GoCrab_Right();   
  }
  if(encoderCount_2 = encoderCount_BeforeTurn+56) RightEndTermonal = 1;  // arrive Right terminal
  if( encoderCount_BeforeTurn+56 <= encoderCount_2 < encoderCount_BeforeTurn+56+112){  //再往左跑搜尋
    GoCrab_Left();
  }
  if(encoderCount_2 = encoderCount_BeforeTurn+56+112) LeftEndTermonal = 1; // arrive Left terminal
  if( encoderCount_BeforeTurn+56+112 <= encoderCount_2 < encoderCount_BeforeTurn+56+112+56){
    GoCrab_Right();
  }
  if(encoderCount_BeforeTurn+56+112+56 <= encoderCount_2 ) Stop;
  
}
void Stop() {
        motor1.run(RELEASE);
        motor2.run(RELEASE);
        motor3.run(RELEASE);
        motor4.run(RELEASE);
        motor1.setSpeed(0);
        motor2.setSpeed(0);
        motor3.setSpeed(0);
        motor4.setSpeed(0);
        Setpoint_2 = 0;
        Setpoint_3 = 0;
        Setpoint_4 = 0;
        Output_2 = 0;
        Output_3 = 0;
        Output_4 = 0;
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
        autoAvoidingFlag = 0;
        demoFlag = 0;
}

/*****************  Encoder CallBack ****************/
void EncoderISR_2() {
    encoderCount_2 += 1;
}
void EncoderISR_3() {
    encoderCount_3 += 1;
    EncoderCount_3 = map(encoderCount_3,1,encoderCount_3,1,encoderCount_3/2);
}
void EncoderISR_4() {
    encoderCount_4 += 1;
}

/*****************  Calculate Velocity CallBack****************/
void calVelocity(){
  detachInterrupt(encoderPinA_2); // Stop detachInterrupt 5
  detachInterrupt(encoderPinA_3); // Stop detachInterrupt 4
  detachInterrupt(encoderPinA_4); // Stop detachInterrupt 3
  angular_veocity2 = (float) (encoderCount_2-encoderCountOld_2)/0.2; // pulse/sec
  angular_veocity3 = (float) (EncoderCount_3-EncoderCountOld_3)/0.2; // pulse/sec
  angular_veocity4 = (float) (encoderCount_4-encoderCountOld_4)/0.2; // pulse/sec
  velocity2 = (float) 3.1416*D*(encoderCount_2-encoderCountOld_2)/(56*0.2) ;  // 周長*(pulse/PPM) / 0.5 (cm/s)　M2:56PPR
  velocity3 = (float) 3.1416*D*(EncoderCount_3-EncoderCountOld_3)/(112*0.2) ;  // 周長*(pulse/PPM) / 0.5 (cm/s)  M3:112PPR
  velocity4 = (float) 3.1416*D*(encoderCount_4-encoderCountOld_4)/(56*0.2) ;  // 周長*(pulse/PPM) / 0.5 (cm/s)  M4:56PPR   
//    Serial.print(sample_time);
//    Serial.print("   ");
//    Serial.print(encoderCount_2);
//    Serial.print("   ");
//    Serial.print(velocity2);
//    Serial.print("   ");
//    Serial.print(outVal2);
//    Serial.print("   ");
//    Serial.print(EncoderCount_3);
//    Serial.print("   ");
//    Serial.print(velocity3);
//    Serial.print("   ");
//    Serial.print(outVal3);
//    Serial.print("   ");
//    Serial.print(encoderCount_4);
//    Serial.print("   ");
//    Serial.print(velocity4);
//    Serial.print("   ");
//    Serial.print(outVal4);
//    Serial.print("   ");
//    Serial.print(GoForwardEnable);
//    Serial.print("   ");
//    Serial.print(GoBackwardEnable);
//    Serial.print("   ");
//    Serial.print(GoCrab_LeftEnable);
//    Serial.print("   ");
//    Serial.print(GoCrab_RightEnable);
//    Serial.print("   ");
//    Serial.print(R45_ForwardEnable);
//    Serial.print("   ");
//    Serial.print(L45_ForwardEnable);
//    Serial.print("   ");
//    Serial.print(R45_BackwardEnable);
//    Serial.print("   ");
//    Serial.println(R45_BackwardEnable);
  encoderCountOld_2 = encoderCount_2;      // Set previous count
  EncoderCountOld_3 = EncoderCount_3;      // Set previous count
  encoderCountOld_4 = encoderCount_4;      // Set previous count   
  attachInterrupt(digitalPinToInterrupt(encoderPinA_2), EncoderISR_2, FALLING); // Reset attachInterrupt 
  attachInterrupt(digitalPinToInterrupt(encoderPinA_3), EncoderISR_3, FALLING); // Reset attachInterrupt 
  attachInterrupt(digitalPinToInterrupt(encoderPinA_4), EncoderISR_4, FALLING); // Reset attachInterrupt 
}

/***************ultrasonic SR04 CallBack****************/
void ultrasonicDetect(){
  float cmMsec_F, cmMsec_R, cmMsec_L;
  long microsec_F = ultrasonic_F.timing(); // 計算距離聲波回彈時間 microseconds, MS, 沒有時間延時函數指令
  long microsec_R = ultrasonic_R.timing(); 
  long microsec_L = ultrasonic_L.timing(); 
  cmMsec_F = ultrasonic_F.convert(microsec_F, Ultrasonic::CM); // 計算距離;單位: 公分
  cmMsec_R = ultrasonic_R.convert(microsec_R, Ultrasonic::CM); 
  cmMsec_L = ultrasonic_L.convert(microsec_L, Ultrasonic::CM);
//  Serial.print("MS: ");
//  Serial.print(microsec_F);
  Serial.print(", F_CM: ");
  Serial.println(cmMsec_F);
//  Serial.print(", R_CM: ");
//  Serial.print(cmMsec_R);
//  Serial.print(", L_CM: ");
//  Serial.println(cmMsec_L);
}

/*****************  IR distance CallBack****************/
void IR_distance(){
  distance_RF = IRdist_RF.getDistanceCentimeter();
  distance_RM = IRdist_RM.getDistanceCentimeter();
  distance_RB = IRdist_RB.getDistanceCentimeter();
  distance_LF = IRdist_LF.getDistanceCentimeter();
  distance_LM = IRdist_LM.getDistanceCentimeter();
  distance_LB = IRdist_LB.getDistanceCentimeter();
  Serial.print("distance_RF in centimeters: ");
  Serial.print(distance_RF);
  Serial.print(", distance_LF: ");
  Serial.print(distance_LF);
  Serial.print(", distance_RM: ");
  Serial.print(distance_RM);
  Serial.print(", distance_LM: ");
  Serial.print(distance_LM);
  Serial.print(", distance_RB: ");
  Serial.print(distance_RB);
  Serial.print(", distance_LB: ");
  Serial.println(distance_LB);
}
/*****************  RFID(NFC) CallBack******************/
void MFRC522read(){
  mfrc522.PICC_IsNewCardPresent();
  mfrc522.PICC_ReadCardSerial();
}

