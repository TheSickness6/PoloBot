#Gabriel Campa Jr
#GitHub: https://github.com/TheSickness6
#POLOBot Wheel Controller Code
#------------------------------------------------------------------------------------------------------
#Imports:
from machine import Pin, PWM, Timer
import utime
#------------------------------------------------------------------------------------------------------
#Variables:
#i2C:
dRPM = 0    #Demand speed in RPM.
Direc = 0 #Direction. 0 = foward, 1 = reverse, 255 = Stop

#Motor Control:
Mpin = 27   #GPIO pin 27 will be used for motor output.
Dpin = 26   #GPIO pin 26 will be used for directional control.
MotorPin = PWM(Pin(Mpin))   #Motor pin PWM set up.
MotorPin.freq(1000)     #PWM frequency set to 1kHz
DirPin = Pin(Dpin,Pin.OUT)  #Directional pin set as output.
MotorPin.duty_u16(0)    #Initial PWM value set to 0.
Volt_Duty = 65025/6     #Conversion to duty cycle from voltage. 65025 is 100% Duty Cycle.

#PID Controller:
PIDOut = 0  #Output of PID Controller.
pretime= 0  #Saved previous time, used for PID interval.
interv = 0  #Time interval since the last time the PID function is accessed.
err = 0 #Error = Demand - Actual Speed.
errrate = 0 #Rate of error.
errsum = 0  #Error sum.
errold = 0  #Old error, used for calculations.
Kp = 0.065  #Proportional Gain.
Ki = 0.0002  #Integral Gain.
Kd = 0  #Derivative Gain.

#Encoder:
us_sec = 1/1000000
dt = 0   #Time difference between new and old time.
otime = 0 #Old time.
ctime = 0 #current time.
Count_RPM = 12  #Pulses to RPM conversion, will probably be changed.
Epin = 15   #GPIO pin 15 will be used for encoder input.
EncPin = Pin(Epin,Pin.IN,Pin.PULL_DOWN)   #Set encoder pin to input.
tiktok = 0
D=0
Tim = Timer()
#------------------------------------------------------------------------------------------------------
#Functions:
#Interrupt function runs when the rising edge of the encoder pulse is detected.
#1. Saves the time the interrupt is triggered last.
#2. Gets new time and saves that.
#3. dt= newtime - oldtime
def enc_handler(pin):
    global ctime,dt,otime,tiktok
    ctime = utime.ticks_us() #Get current time.
    EncPin.irq(handler=None)    #Turn off interrupt.
    if (tiktok==0):
        dt = utime.ticks_diff(ctime,otime) * us_sec #calculate time difference.
        otime = ctime #Save current time to old time.
        tiktok = 1
    else:        
        tiktok = 0
    EncPin.irq(handler=enc_handler) #Turn on interrupt.

#Initialize interrupt
EncPin.irq(trigger=Pin.IRQ_RISING,handler=enc_handler)  #Set interrupt for rising edge of pulse.

def timeTick(timer):
    global otime,dt
    if ((utime.ticks_diff(utime.ticks_us(),otime) * us_sec) > 1):
        dt = 0
    else:
        dt=dt

#Initialize Timer for .25 seconds
Tim.init(freq=4, mode=Timer.PERIODIC, callback=timeTick)

#Function moves motor using PWM. Voltage input is converted to duty cycle.
#Input is Voltage value from 0 to 6V.
#if voltage is higher than 6V, set to max duty cycle.
#else set to desired duty cycle.
def MovMotors(Vin):
    DutyCyc = Volt_Duty * Vin
    if (Vin > 6):
        MotorPin.duty_u16(65025)
    else:
        MotorPin.duty_u16(round(DutyCyc))

#Function takes in desired and encoder RPM and calculates PID output value.
#PID gain values are found above.
def mPID(W,Wf):
    global pretime,Interv,errsum,errrate,errold
    Interv = utime.ticks_diff(utime.ticks_us(),pretime) * us_sec
    pretime = utime.ticks_us()
    err = W - Wf
    errsum += err * Interv
    errrate = (err-errold) / Interv
    Proc = Kp * err
    Intc = Ki * errsum
    Derc = Kd * errrate
    #print("Demand: ",W," Actual:", Wf," Vpid: ",Proc + Intc + Derc)
    #print("Demand: ",W," Actual:", Wf/3)
    errold = err
    return Proc + Intc + Derc
    
#------------------------------------------------------------------------------------------------------
#Main loop:
while True:
    dRPM = 100
    Direc = 0
    if (Direc == 0 or Direc == 1):  #If direction value is not 0 or 1, calculate PID value and then move motor.
        if (dt==0):
            PIDOut = mPID(dRPM,0)
        else:
            PIDOut = mPID(dRPM,D*3/dt)
            #print(3/dt)
        DirPin.value(Direc)
        MovMotors(abs(PIDOut))
    else:   #Else if direction is not foward or reverse, clear all error variables and motor duty cycle to 0.
        MotorPin.duty_u16(0)
        errold = 0
        errsum = 0
        err = 0
        errrate = 0
        otime = 0 
        ctime = 0