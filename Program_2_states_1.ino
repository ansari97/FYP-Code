#include <PID_v1.h>
#include <SharpIR.h>

//*********Function Declarations*********//

long pulse(double);
double angle(long);

void enc_pls0(void);
void enc_pls1(void);
void enc_pls2(void);
void enc_pls3(void);

void pwm_out(int, int);

//*********Variables and Pin Numbers*********//
//Sequence based on items

//Trigger triggers
#define MAX 0
#define T_DOWN 1
#define MIN 2
#define L_OFF 3

//states
#define FLIGHT_D 0
#define STANCE_D 1
#define STANCE_U 2
#define FLIGHT_U 3

//trigger and state variables; state is previous state; state is current state
int trigger, state = FLIGHT_U;


//US sensor
#define usoPin 38
#define usiPin 39

//US sensor variable
double duration;
double height = 0;
double max_h = 0, p_max_h;    //Previous maximum height stores maximum height of previous hop when max_h variable is reset
double min_h = 1000;          //must be greater than starting height of robot
double p1_h = 0;
double p2_h = 0;
double p3_h = 0;
double v1;
double v2;

//motor number constants
#define RHFE 0
#define LHFE 1
#define RKFE 2
#define LKFE 3

//motor pin constants
#define enb 0
#define clc 1
#define aclc 2
#define encA 3
#define encB 4

//Motor pins
const int mot[4][5] = {{5, 23, 22, 21, 24},                            //mot[0]: R_HFE
  {6, 25, 26, 20, 27},                                                 //mot[1]: L_HFE
  {7, 29, 28, 19, 30},                                                 //mot[2]: R_KFE
  {8, 31, 32, 18, 33}                                                  //mot[3]: L_KFE
};

//Pulses per revolution of motor encoders
#define PPR 1120

//Limit switches pin constants
//#define F 0
//#define B 1

//Limit switch pins
//const int lim_swt[4] = {2, 3, 40, 41};    //pins 2 and 3 on interrupt, rest within loop; 2 limit switches on ecah pin for each motor

//Angle reference constants for arrays
#define th2 0
#define th3 1
#define th4 2

//Right and left reference constants
#define R 0
#define L 1

//initial L2 and L3 angles in degrees
double init_th[2][2] = {{00, 90},   //Right th2(hip), right th3(knee)
  {60, 90}                          //Left th2(hip), left th3(knee)
};

//Encoder pulses
long mot_pls[4] = {pulse(init_th[R][th2]), pulse(init_th[L][th2]), pulse(init_th[R][th3]), pulse(init_th[L][th3])};     //RHFE, LHFE, RKFE, LKFE

//Instantaneous angles in degrees
double th[2][3];      //Right th2, th3, th4; Left th2, th3, th4

//PID gain constants
#define kp 0
#define ki 1
#define kd 2

// PID gains (Kp, Ki, Kd)
double PID_k[4][3] = {{0, 0, 0.0},    //RHFE
  {0, 0, 0.0},                      //LHFE
  {0, 0, 0.0},                      //RKFE
  {0, 0, 0.0}                       //LKFE
};

//PID parameter constants
#define ip 0
#define op 1
#define sp 2

//PID paramters: inputs(current position), output(PWM output), setpoint(desired position); all in pulses
double PID_param[4][3] = {{0, 0, 0},    //RHFE
  {0, 0, 0},                            //RLHFE
  {0, 0, 0},                            //RKFE
  {0, 0, 0}                             //LKFE
};

//Defining PID controllers for each motor: PID PID_name(input, output, setpoint, kp, ki, kd, mode)
PID PID_RHFE(&PID_param[RHFE][ip], &PID_param[RHFE][op], &PID_param[RHFE][sp], PID_k[RHFE][kp], PID_k[RHFE][ki], PID_k[RHFE][kd], DIRECT);
PID PID_LHFE(&PID_param[LHFE][ip], &PID_param[LHFE][op], &PID_param[LHFE][sp], PID_k[LHFE][kp], PID_k[LHFE][ki], PID_k[LHFE][kd], DIRECT);
PID PID_RKFE(&PID_param[RKFE][ip], &PID_param[RKFE][op], &PID_param[RKFE][sp], PID_k[RKFE][kp], PID_k[RKFE][ki], PID_k[RKFE][kd], DIRECT);
PID PID_LKFE(&PID_param[LKFE][ip], &PID_param[LKFE][op], &PID_param[LKFE][sp], PID_k[LKFE][kp], PID_k[LKFE][ki], PID_k[LKFE][kd], DIRECT);


/*
  //Ankle potentiometer
  // define before *th* array !!
  #define R_ank 0
  #define L_ank 1
  const int ank_pin[2] = {A1, A2};
  double ank_ang[2] = {0, 0};
*/

//push buttons; Check debouncing

const int ft_but_pin[2][2] = {{34, 35}, {36, 37}};
int ft_but[2][2] = {{LOW, LOW},   //Right 1 and right 2
  {LOW, LOW}
};                      //Left 1 and left 2
// Define p_button trigger

//Touchdown LED pins
const int led_pin[2] = {11, 12};  //Right and left


//*********       setup()       *********//

void setup() {

  //Attaches interrupt to all motor encoders' chanA at fallin trigger; ISRs(enc_pls...) checks chanB trigger to determine direction and position
  attachInterrupt(digitalPinToInterrupt(mot[RHFE][encA]), enc_pls0, FALLING);
  attachInterrupt(digitalPinToInterrupt(mot[LHFE][encA]), enc_pls1, FALLING);
  attachInterrupt(digitalPinToInterrupt(mot[RKFE][encA]), enc_pls2, FALLING);
  attachInterrupt(digitalPinToInterrupt(mot[LKFE][encA]), enc_pls3, FALLING);

  /*
    attachInterrupt(digitalPinToInterrupt(lim_swt[RHFE]), mot_stop_RHFE, LOW);
    attachInterrupt(digitalPinToInterrupt(lim_swt[LHFE]), mot_stop_LHFE, LOW);
  */

  //Sets motor pinmodes
  for (int i = 0; i < 4; i++) {
    pinMode(mot[i][enb], OUTPUT);
    pinMode(mot[i][clc], OUTPUT);
    pinMode(mot[i][aclc], OUTPUT);
    pinMode(mot[i][encA], INPUT_PULLUP);
    pinMode(mot[i][encB], INPUT_PULLUP);
  }

  //Sets US sensor pin modes
  pinMode(usoPin, OUTPUT); // Sets the usoPin as an Output
  pinMode(usiPin, INPUT);  // Sets the usiPin as an Input

  //Sets Ft push buttons and touchdown led pin modes
  for (int i = 0; i < 2; i++) {
    pinMode(led_pin[i], OUTPUT);
    pinMode(ft_but_pin[i][0], INPUT);
    pinMode(ft_but_pin[i][1], INPUT);
  }

  //Turn off touchdown LEDs
  for (int i = 0; i < 2; i++) {
    digitalWrite(led_pin[i], LOW);
  }

  //sets 31KHz PWM to prevent motor noise
  TCCR3B &= ~ 7; TCCR3B |= 3;   // pins 2, 3, 5
  TCCR4B &= ~ 7; TCCR4B |= 3;   // pins 6, 7, 8

  //Sets sampling time and PID PWM ouptut limits for all PID controllers
  PID_RHFE.SetMode(AUTOMATIC);           //sets PID in Auto mode
  PID_RHFE.SetSampleTime(10);            //refreshes rate of PID controller
  PID_RHFE.SetOutputLimits(-200, 200);   //this is the MAX PWM value to move motor

  PID_LHFE.SetMode(AUTOMATIC);           //sets PID in Auto mode
  PID_LHFE.SetSampleTime(10);            //refreshes rate of PID controller
  PID_LHFE.SetOutputLimits(-200, 200);   //this is the MAX PWM value to move motor

  PID_RKFE.SetMode(AUTOMATIC);           //sets PID in Auto mode
  PID_RKFE.SetSampleTime(10);            //refreshes rate of PID controller
  PID_RKFE.SetOutputLimits(-200, 200);   //this is the MAX PWM value to move motor

  PID_LKFE.SetMode(AUTOMATIC);           //sets PID in Auto mode
  PID_LKFE.SetSampleTime(10);            //refreshes rate of PID controller
  PID_LKFE.SetOutputLimits(-200, 200);   //this is the MAX PWM value to move motor

  //Begins serial communication
  Serial.begin(9600);

}//////////////////////////////////////////////////////////////////////////////


//*********       loop()       *********//

void loop() {

  trigger = -1;

  // defines relationship b/w mot_pls and link angles
  // hip angle measured from torso fore side to link in clc direction for both legs;
  // knee angle measured clc from L2 extension; *set ankle angle direction convention* !!
  th[R][th2] = (angle(mot_pls[RHFE]));
  th[L][th2] = (angle(mot_pls[LHFE]));

  th[R][th3] = ((angle(mot_pls[RKFE]) + init_th[R][th3]) / 2) - (angle(mot_pls[RHFE]) - init_th[R][th2]);
  th[L][th3] = ((angle(mot_pls[LKFE]) + init_th[L][th3]) / 2) - (angle(mot_pls[LHFE]) - init_th[L][th2]);

  //Ft push button triggers
  for (int i = 0; i < 2; i++) {
    ft_but[i][0] = digitalRead(ft_but_pin[i][0]);
    ft_but[i][1] = digitalRead(ft_but_pin[i][1]);
  }


  //PID input is the encoder output in pulses
  // Sets at the beginning of loop() function
  for (int i = 0; i < 4; i++) {
    PID_param[i][ip] = mot_pls[i];
  }

  // PID set points
  //* Sets within PID function * !!
  PID_param[RHFE][sp] = ((double)20 / 360) * PPR;
  PID_param[LHFE][sp] = ((double)30 / 360) * PPR;
  PID_param[RKFE][sp] = ((double)30 / 360) * PPR;
  PID_param[LKFE][sp] = ((double)30 / 360) * PPR;

  //Serial.print("RHFE Op: ");
  //Serial.print(PID_param[RHFE][op]);

  //Serial.print("  ");

  //Compute PID output
  // * Sets within PID function * !!
  //PID_RHFE.Compute();
  //PID_LHFE.Compute();
  //PID_RKFE.Compute();
  //PID_LKFE.Compute();


  Serial.print("RHFE Op: ");
  Serial.print(PID_param[RHFE][op]);

  Serial.print("  ");

  //PID output to motor
  // Set within PID function !!
  for (int i = 0; i < 4; i++) {
    pwm_out(i, PID_param[i][op]);
  }


  //pwm_out(RHFE, PID_param[RHFE][op]);
  //pwm_out(LKFE, PID_param[LKFE][op]);

  //Serial.print("RHFE Op/PWM: ");
  //Serial.print(PID_param[RHFE][op]);

  //Serial.print("  ");

  Serial.print("RHFE Sp: ");
  Serial.print(PID_param[RHFE][sp]);

  Serial.print("  ");

  Serial.print("RHFE Ip: ");
  Serial.print(th[R][th2]);

  //Serial.print("  ");

  //Serial.print("LHFE Op/PWM: ");
  // Serial.print(PID_param[LHFE][op]);

  //Serial.print("  ");

  //  Serial.print("LHFE Sp: ");
  //Serial.print(PID_param[LHFE][sp]);

  //Serial.print("  ");

  //Serial.print("LHFE Ip: ");
  //Serial.print(PID_param[LHFE][ip]);

  //Serial.print("  ");

  //Serial.print("LKFE Op/PWM: ");
  // Serial.print(PID_param[LKFE][op]);

  //Serial.print("  ");

  //Serial.print("LKFE Sp: ");
  //Serial.print(PID_param[LKFE][sp]);

  //Serial.print("  ");

  //Serial.print("LKFE Ip: ");
  //Serial.print(PID_param[LKFE][ip]);

  //Serial.print("  ");

  //Serial.print("RKFE Op/PWM: ");
  // Serial.print(PID_param[RKFE][op]);

  //Serial.print("  ");

  //Serial.print("RKFE Sp: ");
  //Serial.print(PID_param[RKFE][sp]);

  //Serial.print("  ");

  //Serial.print("RKFE Ip: ");
  //Serial.print(PID_param[RKFE][ip]);

  //*************************//
  //US sensor
  // Clears the usoPin
  digitalWrite(usoPin, LOW);
  delayMicroseconds(2);

  // Sets the usoPin on HIGH trigger for 10 micro seconds
  digitalWrite(usoPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(usoPin, LOW);

  //Previous loop velocity
  v2 = v1;

  // Reads the usiPin, returns the sound wave travel time in microseconds
  duration = pulseIn(usiPin, HIGH);
  // Calculating the height in mm
  height = (double) duration * 0.346 / 2; //in mm

  //Check and update maximum height; 3 is threshold for change
  if (height - max_h > 3) {
    max_h = height;
  }

  //Check and update minimum height; 3 is threshold for change
  if (min_h - height > 3) {
    min_h = height;
  }

  //Check v1(delta h); v1 is positive in vertical up direction
  v1 = height - p1_h;

  //Update previous height
  p1_h = height;

  //Check for MAX trigger and update trigger and state
  if (v1 < 0 && v2 > 0 && v1 < -5)  {

    if (state == FLIGHT_U) {
      state = FLIGHT_D;
      trigger = MAX;
    }

    /*
        if (state != state) {
          state = state;
        }*/

  }

  //Check for MIN trigger and update trigger and state
  if (v1 > 0 && v2 < 0 && v1 > 5 && (ft_but[L][0] == LOW || ft_but[L][1] == LOW || ft_but[R][0] == LOW || ft_but[R][1] == LOW)) {

    if (state == STANCE_D) {
      state = STANCE_U;
      trigger = MIN;
    }

    /*
        if (state != state) {
          state = state;
        }*/
  }



  /*
    else {
      trigger = -1;
    }
  */

  // Prints the height on the Serial Monitor
  Serial.print("  ");
  Serial.print("height: ");
  Serial.print(height);

  Serial.print("  ");

  Serial.print("max height: ");
  Serial.print(max_h);

  Serial.print("  ");

  Serial.print("min height: ");
  Serial.print(min_h);

  //Make a function:check_foot_trigger() !!
  //When right foot makes contact (touchdown)
  if (ft_but[R][0] == LOW || ft_but[R][1] == LOW) {
    digitalWrite(led_pin[R], HIGH);
    p_max_h = max_h;
    max_h = 0;

    if (state == FLIGHT_D) {
      state = STANCE_D;
      trigger = T_DOWN;
    }
    /*
        if (state != state) {
          state = state;
        }*/

  }


  //When left foot makes contact (touchdown)
  if (ft_but[L][0] == LOW || ft_but[L][1] == LOW) {
    digitalWrite(led_pin[L], HIGH);
    p_max_h = max_h;
    max_h = 0;



    if (state == FLIGHT_D ) {
      state = STANCE_D;
      trigger = T_DOWN;
    }

    /*
        if (state != state) {
          state = state;
        }*/

  }

  //When right foot leaves contact (lift-off)
  if (ft_but[R][0] == HIGH && ft_but[R][1] == HIGH) {
    digitalWrite(led_pin[R], LOW);

    if (ft_but[L][0] == HIGH && ft_but[L][1] == HIGH) {

      if (state == STANCE_U) {
        state = FLIGHT_U;
        trigger = L_OFF;
      }

      /*
            if (state != state) {
              state = state;
            }*/

    }
  }

  //When left foot leaves contact (lift-off)
  if (ft_but[L][0] == HIGH && ft_but[L][1] == HIGH) {
    digitalWrite(led_pin[L], LOW);

    if (ft_but[R][0] == HIGH && ft_but[R][1] == HIGH) {

      if (state == STANCE_U) {
        state = FLIGHT_U;
        trigger = L_OFF;
      }

      /*
        if (state != state) {
        state = state;
        }*/

    }

  }


  Serial.print("  ");

  Serial.print("Trigger: ");

  switch (trigger) {
    case 0:
      Serial.print("MAX");
      break;
    case 1:
      Serial.print("T_DOWN");
      break;
    case 2:
      Serial.print("MIN");
      break;
    case 3:
      Serial.print("L_OFF");
      break;
    case -1:
      Serial.print("INTERMEDIATE");
      break;
  }

  Serial.print("  ");

  Serial.print("state: ");

  switch (state) {
    case 0:
      Serial.print("FLIGHT_D");
      break;
    case 1:
      Serial.print("STANCE_D");
      break;
    case 2:
      Serial.print("STANCE_U");
      break;
    case 3:
      Serial.print("FLIGHT_U");
      break;
  }

  Serial.println("");

}//////////////////////////////////////////////////////////////////////////////

//*********Functions*********//

//Motor PID function; Setpoint in deg; mot_pls in pulses
//Remove mot_pls since PID_param[ip] already assigned mot_pls?? !!
//Add max/min PID PWM output !!
void mot_PID(int motnum, double th_sp, long mot_pls) {
  PID_param[motnum][sp] = pulse(th_sp);
  PID_param[motnum][ip] = mot_pls;

  switch (motnum) {
    case 0:
      PID_RHFE.Compute();
      break;
    case 1:
      PID_LHFE.Compute();
      break;
    case 2:
      PID_RKFE.Compute();
      break;
    case 3:
      PID_LKFE.Compute();
      break;
  }

  pwm_out(motnum, PID_param[motnum][op]);
}

//Motor driver function based on PID PWM output
void pwm_out(int motnum, int out) {
  if (out >= 0) {                               //clockwise
    digitalWrite(mot[motnum][clc], HIGH);
    digitalWrite(mot[motnum][aclc], LOW);
    analogWrite(mot[motnum][enb], out);
  }
  else {                                        //anticlockwise
    out = -out;
    digitalWrite(mot[motnum][clc], LOW);
    digitalWrite(mot[motnum][aclc], HIGH);
    analogWrite(mot[motnum][enb], out);
  }
}

//ISRs to count encoder pulses
void enc_pls0() {
  if (digitalRead(mot[RHFE][encB]) == LOW) { //Clc positive
    mot_pls[RHFE]++;
  }
  else {
    mot_pls[RHFE]--;
  }
}

void enc_pls1() {
  if (digitalRead(mot[LHFE][encB]) == HIGH) { //Aclc positive
    mot_pls[LHFE]++;
  }
  else {
    mot_pls[LHFE]--;
  }
}

void enc_pls2() {
  if (digitalRead(mot[RKFE][encB]) == LOW) { //Clc positive
    mot_pls[RKFE]++;
  }
  else {
    mot_pls[RKFE]--;
  }
}

void enc_pls3() {
  if (digitalRead(mot[LKFE][encB]) == HIGH) { //Alc positive
    mot_pls[LKFE]++;
  }
  else {
    mot_pls[LKFE]--;
  }
}

///Motor stop
void mot_stop_RHFE() {
  analogWrite(mot[RHFE][enb], 0);
  digitalWrite(mot[RHFE][clc], LOW);
  digitalWrite(mot[RHFE][aclc], LOW);
}

void mot_stop_LHFE() {
  analogWrite(mot[LHFE][enb], 0);
  digitalWrite(mot[LHFE][clc], LOW);
  digitalWrite(mot[LHFE][aclc], LOW);
}

//encoder pulses to angle
double angle(long pulse) {
  double ang = (((double) pulse / PPR) * 360);
  return ang;
}
long pulse(double angle) {
  long pls = ((angle / 360) * PPR);
  return pls;
}

