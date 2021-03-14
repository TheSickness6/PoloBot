  #include <Wire.h>
//My Varialbles
double Demand=100;
double TimeDelay=.25;
volatile double count=0;
volatile byte Puls=0;
volatile byte Direc=0;
double dRPM,fRPM,PIDout;
double PultoRPM=60/20;
double CounttoRPM=12;
int MotorPin=9;
int DirPin=8;
volatile double errold,errsum;


void sendSpeed()
{
  Wire.write(byte(count));
}

void getSpeed()
{
	Puls = Wire.read();    // receive a byte
	Direc = Wire.read(); 
}

ISR(TIMER3_OVF_vect)
{
  TCNT3= 62499;
  count=TCNT0;
  TCNT0=0;
}

double mPID(double W,double Wf)
{
  double Kp=.09,Ki=0.02,Kd=0.01;
  double Interv=.25;
	double err=W-Wf;
	double Proc=Kp*err;
	double Intc=Ki*errsum;
	double Derc=Kd*(err-errold)/Interv;
  errold=err;
  errsum=errsum+err*Interv;
	return Proc+Intc+Derc;
}

void movMotors(double SpeedRPM)
{
  double Volt2Duty=255/6;
	double DutyCyc=(SpeedRPM)*Volt2Duty;
	if (Direc==0)
	{
  digitalWrite(DirPin,HIGH);
		if (DutyCyc>255)	
		{
			analogWrite(MotorPin,255);
		}
		else	
		{
			analogWrite(MotorPin,DutyCyc);
		}
	}
	else if (Direc==1)
	{
  digitalWrite(DirPin,LOW);
		if (DutyCyc>255)	
		{
			analogWrite(MotorPin,255);
		}
		else	
		{
			analogWrite(MotorPin,DutyCyc);
		}
	}
}
//Initial setup
void setup() 
{
 Serial.begin(9600);
 noInterrupts();
 TCCR0A=0;	//No special settings
 TCCR0B=0x07;	//Clock from external source
 //TCCR2C=0;
 TCNT0=0;
 //Timer Setup
 TCCR3A=0; //No special settings
 TCCR3B=0;
 TCCR3C=0;
 TCNT3=62499;
 // Set CS10 and CS12 bits for 1024 prescaler
 TCCR3B |= (1 << CS32) | (1 << CS30);  
 // enable timer compare interrupt
 TIMSK3 |= (1 << TOIE3);
 interrupts();
 pinMode(MotorPin,OUTPUT);
 pinMode(DirPin,OUTPUT);
 Wire.begin(0x66);
 Wire.onRequest(sendSpeed);
 Wire.onReceive(getSpeed);
}

//Main loop
void loop() 
{
  dRPM=Puls;
  fRPM=count*CounttoRPM;
  if (Direc==255)
  {
    analogWrite(MotorPin,0);
    errold=0;
    errsum=0;
  }
  else
  {
    PIDout=mPID(abs(dRPM),abs(fRPM));
    movMotors(PIDout);
  }
  Serial.print("Demand: ");
  Serial.print(dRPM);
  Serial.print("RPM Actual: ");
  Serial.print(fRPM);
  Serial.print("RPM Direction");
  Serial.print(Direc);
  Serial.print("\n");
}
