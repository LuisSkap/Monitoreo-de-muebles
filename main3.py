from ST7735 import TFT
from sysfont import sysfont
from machine import SPI,Pin, PWM
import onewire
import ds18x20
import time
import math
import _thread

spi = SPI(1, baudrate=20000000, polarity=0, phase=0,
          sck=Pin(10), mosi=Pin(11), miso=None)

tft=TFT(spi, 8, 12, 9)

tft.initr()
tft.rgb(True)
##______________________________________________________________________________
# Define the PIR sensor input pin
pir_pin = Pin(21, Pin.IN)

# Define the LED pin (optional, for visual feedback)
led_pin = Pin(25, Pin.OUT)

# Flags to indicate motion detection state
motion_detected = False

motion_stopped_printed = True


#Contador de personas
Counter_P = 0

# Callback function to handle motion detection
def pir_interrupt(pin):
    global motion_detected
    if pin.value() == 1:  # Rising edge (motion detected)
        motion_detected = True
        led_pin.value(1)  # Turn on the LED
    else:  # Falling edge (motion stopped)
        motion_detected = False
        led_pin.value(0)  # Turn off the LED

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
output2_pwm = PWM(output2)

# Step size for changing the duty cycle
duty_step = 128  

#Set PWM frequency
frequency = 5000

output2_pwm.freq (frequency)

##_______________________________________________________________________________

def SensorPIR():
    tft.fill(TFT.BLACK)
    
    global motion_detected

    global motion_stopped_printed
    
    global Counter_P 
    
    if motion_detected and not motion_stopped_printed:
        print("Motion detected!")
        tft.text((3, 20), "Motion ", TFT.GREEN, sysfont, 2, nowrap=True)
        tft.text((3, 36), "detected! ", TFT.GREEN, sysfont, 2, nowrap=True)
        motion_stopped_printed = True  # Set the flag to indicate motion detected
        Counter_P += 1

    elif not motion_detected and motion_stopped_printed:
        print("Motion stopped")
        tft.text((3, 20), "Motion ", TFT.GREEN, sysfont, 2, nowrap=True)
        tft.text((3, 36), "stopped", TFT.GREEN, sysfont, 2, nowrap=True)
        motion_stopped_printed = False  # Reset the flag
    
    tft.text((3, 52), "Count: ", TFT.GREEN, sysfont, 2, nowrap=True)
    tft.text((3, 68), str(Counter_P), TFT.GREEN, sysfont, 2, nowrap=True)
    

def SensorTemp():
    
    tft.fill(TFT.BLACK)
    
    ds_sensor.convert_temp()
    time.sleep_ms(750)
    for rom in roms:
        print(rom)
        tempC = ds_sensor.read_temp(rom)
        tempF = tempC * (9/5) +32
        print('temperature (ºC):', "{:.2f}".format(tempC))
        print('temperature (ºF):', "{:.2f}".format(tempF))
        tft.text((3, 30), "Temp  C", TFT.GREEN, sysfont, 2, nowrap=True)
        tft.text((3, 56), str(tempC), TFT.GREEN, sysfont, 2, nowrap=True)


def  SensorCurrent():
    
    tft.fill(TFT.BLACK)
    
    voltage = CurrentSensor.read_u16() * conversion_factor
    print(voltage)
    tft.text((3, 30), "Current: ", TFT.GREEN, sysfont, 2, nowrap=True)
    tft.text((3, 56), str(voltage), TFT.GREEN, sysfont, 2, nowrap=True)
   
   
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

def PWM_LED():
    
    global motion_detected
    
    global motion_stopped_printed
    
    count_not_motion = 0
    
    strip_flag = False
    
    while True:
        
#         if motion_detected and not motion_stopped_printed:
#             fade(output2_pwm, 1, 0, 65536)
#             print("FadeUP")
#             time.sleep_ms(5000)
#             fade(output2_pwm, 0, 1, 25000)
            
        if motion_detected and not motion_stopped_printed:
            print("Motion detected!")
            if strip_flag == False:
                fade(output2_pwm, 1, 0, 65536)
                strip_flag = True
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
            
        
_thread.start_new_thread(PWM_LED, ())


while True:
    
    SensorTemp()
    
    time.sleep_ms(1500)
    
    SensorCurrent()
    
    time.sleep_ms(1500)
    
    SensorPIR()
    
    time.sleep_ms(1500)
    
    

    
    