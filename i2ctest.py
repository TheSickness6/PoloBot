import smbus 
import time 
addr=0x06 
bus=smbus.SMBus(1)

while 1:
	time.sleep(.25)
	try:
		mess=bus.read_word_data(addr,16)
	except:
		print ("Read Error")
	else:
		mess1=(mess)>>8
		mess2=mess&0xFF
		print ("Left Wheel: ",mess1*12," RPM,  Right Wheel: ",mess2*12,"RPM")
