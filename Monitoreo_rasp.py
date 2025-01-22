from ST7735 import TFT
from sysfont import sysfont
from machine import UART,SPI,Pin, PWM
import onewire
import ds18x20
import time
import math
import _thread
import select
import sys


###____________Serial comunication w/Raspberry pi_________________________###

uart1 = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5))


# uart1.write('hello')  # write 5 bytes
# uart1.read(5)         # read up to 5 bytes


###___________Serial comunication w/TFT St7735___________________________###

spi = SPI(1, baudrate=20000000, polarity=0, phase=0,
          sck=Pin(10), mosi=Pin(11), miso=None)

tft=TFT(spi, 8, 12, 9)

tft.initr()
tft.rgb(True)
tft.rotation(2)


##______________________________________________________________________________

# Define the PIR sensor input pin
pir_pin = Pin(21, Pin.IN)

# Define the LED pin (optional, for visual feedback)
led_pin = Pin(25, Pin.OUT)

# Flags to indicate motion detection state
motion_detected = False

motion_stopped_printed = False


#Contador de personas
Counter_P = 0



# Callback function to handle motion detection
def pir_interrupt(pin):
    global motion_detected
    
    time.sleep_ms(50)  # Tiempo de debounce
    
    if pin.value() == 1:  # Rising edge (motion detected)
        motion_detected = True
        
    else:  # Falling edge (motion stopped)
        motion_detected = False

# Configure the interrupt on the PIR pin for both rising and falling edges
pir_pin.irq(trigger=(Pin.IRQ_RISING | Pin.IRQ_FALLING), handler=pir_interrupt)

##_______________________________________________________________________________

ds_pin = machine.Pin(22)
ds_sensor = ds18x20.DS18X20(onewire.OneWire(ds_pin))

roms = ds_sensor.scan()
print('Found DS devices: ', roms)

##_______________________________________________________________________________

CurrentSensor = machine.ADC(26)
conversion_factor = 3.3 / (65535)

##_______________________________________________________________________________

output2 = machine.Pin(2)
output3 = machine.Pin(3)
output4 = machine.Pin(6)
output5 = machine.Pin(7)

output2_pwm = PWM(output2)
output3_pwm = PWM(output3)
output4_pwm = PWM(output4)
output5_pwm = PWM(output5)


# Step size for changing the duty cycle
duty_step = 128  

#Set PWM frequency
frequency = 5000

output2_pwm.freq (frequency)
output3_pwm.freq (frequency)
output4_pwm.freq (frequency)
output5_pwm.freq (frequency)


count = 0
tempo = 0
tempobool = False

##_______________________________________________________________________________
    
    
def SensorTemp():
    
    ds_sensor.convert_temp()
    time.sleep_ms(750)
    for rom in roms:
        print(rom)
        tempC = ds_sensor.read_temp(rom)
        tempF = tempC * (9/5) +32
        
    return tempC


def  SensorCurrent():
    
    
    voltage = CurrentSensor.read_u16() * conversion_factor
    
    return voltage



def fadeall(pic, dutyQ):
    
    for duty_cycle in range(0, dutyQ, duty_step):
        
        pic.duty_u16(duty_cycle)
        time.sleep(0.005)
        


def fadeoff(pic, dutyQ):
    
    for duty_cycle in range(65536, dutyQ, -duty_step):
        
        pic.duty_u16(duty_cycle)
        time.sleep(0.005)
            
   
   
def fade(pic, up, dw, dutyQ):
    
    if up == 1:
        for duty_cycle in range(25000, dutyQ, duty_step):
            pic.duty_u16(duty_cycle)
#             print("Output: %s , Duty: %s ." % (pic, duty_cycle))
            time.sleep(0.005)
            
    if dw == 1:
        for duty_cycle in range(65536, dutyQ , -duty_step):
            pic.duty_u16(duty_cycle)
#             print("Output: %s , Duty: %s ." % (pic, duty_cycle))
            time.sleep(0.005)
            
            
fade(output2_pwm, 1, 0, 25000)

def Pir_sensor():
    
    global motion_detected
    
    global motion_stopped_printed
    
    global Counter_P
    
    count_not_motion = 0
    
    strip_flag = False
    
    while True:
            
        if motion_detected and not motion_stopped_printed:
            print("Motion detected!")
            if strip_flag == False:
                fade(output2_pwm, 1, 0, 65536)
                strip_flag = True
                motion_stopped_printed = True
                Counter_P += 1
            else:
                count_not_motion = 0
            print("FadeUP")
            

        elif not motion_detected and motion_stopped_printed:
            print("Motion stopped")
            count_not_motion += 1
            time.sleep_ms(50)
            if count_not_motion == 300:
                count_not_motion = 0
                fade(output2_pwm, 0, 1, 25000)
                strip_flag = False
            motion_stopped_printed = False 
            
        
_thread.start_new_thread(Pir_sensor, ())


while True:

    time.sleep(0.2)
    count += 1
    tempo += 1
    
        
    Medicion_de_temp = SensorTemp()
    Medicion_de_Current = SensorCurrent()
    PIR_count = Counter_P
    
    if count == 15:
        
        tft.fill(TFT.BLACK)
        
        print('temperature (ºC):', "{:.2f}".format(Medicion_de_temp))
        
        tft.text((3, 30), "Temp  C", TFT.GREEN, sysfont, 2, nowrap=True)
        tft.text((3, 56), str(Medicion_de_temp), TFT.GREEN, sysfont, 2, nowrap=True)
        
    elif count == 30:
        
        tft.fill(TFT.BLACK)
        
        print('voltage (V):', "{:.2f}".format(Medicion_de_Current))
        
        tft.text((3, 30), "Current: ", TFT.GREEN, sysfont, 2, nowrap=True)
        tft.text((3, 56), str(Medicion_de_Current), TFT.GREEN, sysfont, 2, nowrap=True)
        
    elif count == 45:
        
        tft.fill(TFT.BLACK)
        
        print('conteo de personas:', "{:.2f}".format(PIR_count))
        
        tft.text((3, 52), "Count: ", TFT.GREEN, sysfont, 2, nowrap=True)
        tft.text((3, 68), str(PIR_count), TFT.GREEN, sysfont, 2, nowrap=True)
        
        count = 0
        
        
    
    uart1.write(f"{Medicion_de_temp:.2f}, {Medicion_de_Current:.2f}, {PIR_count}\n")
    
    time.sleep(0.1)
    
    rasp1 = uart1.read(5)
    
    if rasp1 == 'manoi':
        fadeall(output3_pwm, 65536)
    
    elif rasp1 == 'manod':
        fadeall(output4_pwm, 65536)
    
    elif rasp1 == 'manos':
        fadeall(output5_pwm, 65536)
    
    elif rasp1 == 'newpr':
        PIR_count += 1
    
    elif rasp1 == 'cruze':
        fadeoff(output3_pwm, 0)
        fadeoff(output4_pwm, 0)
        fadeoff(output5_pwm, 0)
    
    elif rasp1 == 'noone':
        tempobool = True
    
    
    if tempobool == True and tempo == 30:
        fadeoff(output3_pwm, 0)
        fadeoff(output4_pwm, 0)
        fadeoff(output5_pwm, 0)
        tempo = 0
        tempobool = False
        
    elif tempobool ==False:
        tempo = 0
        
    
