/* 
Gabriel Campa Jr.
GitHub: https://github.com/TheSickness6
POLOBot Wheel Controller Code
*/ 

/*
Includes
Currently only using Wire library for I2C.
*/
#include <Wire.h>

/*
My Variables
Variables are split up between I2C, Motor Control, Encoder, and PID Controller.
*/

//Motor Control
int MotorPin=9;	//Pin used for motor PWM.
int DirPin=8;	//Pin used for directional.
double Volt2Duty=255/6; //Converts Voltage to PWM duty cycle.
double DutyCyc=0; //PWM duty cycle.

//I2C
volatile byte Puls=0;	//RPM desired from Pi 4
volatile byte Direc=255;	//Direction desired from Pi4

//Encoder
double TimeDelay=.25;	//Time for timer 3, 250ms
volatile double count=0;	//Number of pulses read from counter 0.
double CounttoRPM=12;	//Convert number of pulses read to RPM 60/(20*.250).
int CntVal=62499;	//Timer 3 Prescaller value for 250ms

//PID Controller
double dRPM,fRPM,PIDout,err,errrate;	//Demand speed, Feedback speed, PID Output, error, and error rate.
volatile double errold,errsum;	//Old Error, Sum of error.
//double Kp=0.026962,Ki=0.199707534,Kd=0; //Original PID values generated using MATLAB
double Kp=.06,Ki=0.0001,Kd=0;	//Proportional, Integral, Derivative gains.
double Interv=.0005;  //PID loop interval calculated using serial monitor.

/*
My Functions
Functions used for I2C, timer 3 ISR, PID controller, and motor control.
*/

/*Sends current encoder value through I2C to Pi 4. 
Value is sent in pulses read.*/
void sendSpeed()
{
	Wire.write(byte(count));
}

/*
Receives demand speed and direction through I2C from Pi 4. Speed value is received in RPM.
Direction can be 0 for foward, 1 for reverse, and 255 for stop motors.
*/
void getSpeed()
{
	Puls = Wire.read();    // receive a byte
	Direc = Wire.read(); 
}

/*Timer 3 ISR function. Gets count from counter 0 and then resets to 0. 
Also reloads prescaler value for timer 3.*/
ISR(TIMER3_OVF_vect)
{
	TCNT3= CntVal;
	count=TCNT0;
	TCNT0=0;
}

/*PID function for motor control. Receives demand speed and actual speed.
Outputs PID voltage.*/
double mPID(double W,double Wf)
{
	err=W-Wf;
  Serial.print("DSpeed: ");
  Serial.print(W,5);
  Serial.print(" ASpeed: ");
  Serial.println(Wf,5);
	errsum=errsum+err*Interv;
	errrate=(err-errold)/Interv;
	double Proc=Kp*err;
	double Intc=Ki*errsum;
	double Derc=Kd*errrate;
	errold=err;
	return Proc+Intc+Derc;
}

//Motor control function. Receives desired RPM and converts to PWM.
void movMotors(double SpeedRPM)
{
	DutyCyc=(SpeedRPM)*Volt2Duty;
	if (DutyCyc>255)	
	{
		analogWrite(MotorPin,255);
	}
	else	
	{
		analogWrite(MotorPin,DutyCyc);
	}
}

//Initial setup
void setup() 
{
	Serial.begin(9600);
	noInterrupts();
	//Counter Setup
	TCCR0A=0;	//No special settings
	TCCR0B=0x07;	//Clock from external source
	TCNT0=0;
	//Timer Setup
	TCCR3A=0; //No special settings
	TCCR3B=0;
	TCCR3C=0;
	TCNT3=62499;	//(16MHz/(1024*T))-1 Result must be < 65536
	// Set CS10 and CS12 bits for 1024 prescaler
	TCCR3B |= (1 << CS32) | (1 << CS30);  
	// enable timer compare interrupt
	TIMSK3 |= (1 << TOIE3);
	interrupts();
	pinMode(MotorPin,OUTPUT);
	pinMode(DirPin,OUTPUT);
	Wire.begin(0x66); //67=Left 66=Right
	Wire.onRequest(sendSpeed);
	Wire.onReceive(getSpeed);
}

//Main loop
void loop() 
{
  if (Direc==1||Direc==0){  //Movement command given.
    dRPM=Puls;  //Get demand speed. In RPM.
    fRPM=count*CounttoRPM;  //Calculate actual speed.
    PIDout=mPID(dRPM,fRPM); //Calculate PID output.
    /*If direction is set to go foward and PID value is larger than 0.
    Or if direction is set to reverse but PID value is larger than 0.
    Set motor to go foward.*/
    if (Direc==0){digitalWrite(DirPin,0);}
    /*If direction is set to go reverse and PID value is below 0.
    Or if direction is set to foward but PID value is below 0.
    Set motor to go reverse.*/
    else if (Direc==1){digitalWrite(DirPin,1);}
    movMotors(abs(PIDout)); //Time to move!
  }
  else {  //Default state. No movement.
    analogWrite(MotorPin,0);
    errold=0; //Reset error variables.
    errsum=0;
  }
}
