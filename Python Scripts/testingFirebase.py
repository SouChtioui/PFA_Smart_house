import RPi.GPIO as GPIO   #import the RPi.GPIO module to allow us use the board GPIO pins.
import pyrebase           #import the pyrebase module which allows us to communicate with the firebase servers.
import time    #import the time modulde to allow us do the delay stuff.
import os                 #for the webcam
import Adafruit_DHT
import smbus
import subprocess

config = {                #define a dictionary named config with several key-value pairs that configure the connection to the database.
  "apiKey": "AIzaSyAa2frpydUs0FoBO0pIP6iPUN9gIHcxWkA",
  "authDomain": "iiasmarthouse.firebaseapp.com",
  "databaseURL": "https://iiasmarthouse.firebaseio.com/",
  "storageBucket": "iiasmarthouse.appspot.com"
}

firebase = pyrebase.initialize_app(config)   #initialize the communication with the "firebase" servers using the previous config data.

DHT_SENSOR = Adafruit_DHT.DHT22
DHT_PIN = 26

GPIO.setmode(GPIO.BCM)                                                                         #Set the GPIO Scheme numbering system to the BCM mode.
GPIO.setwarnings(False)                                                                        #disable warnings
GPIO.setup(DHT_PIN,GPIO.IN)                                                                    #set the "redLED" variable pin (12) as an output pin.
GPIO.setup(17,GPIO.OUT)
GPIO.setup(27,GPIO.OUT)
GPIO.setup(22,GPIO.OUT)
GPIO.setup(10,GPIO.OUT)
GPIO.setup(9,GPIO.OUT)
GPIO.setup(11,GPIO.OUT)
GPIO.setup(5,GPIO.OUT)
GPIO.setup(6,GPIO.OUT)
# GPIO.setup(13,GPIO.OUT)
# servo=GPIO.PWM(13,50)

rooms={"Bathroom":{"light_bath":17,"win_bath":27},
   "Bedroom":{"ac_bed":22,"light_bed":10},
   "kitchen":{"ac_kitchen":9,"light_kitchen":11},
   "L_room":{"ac_l_room":5,"light_l_room":6}}

# Define some device parameters
I2C_ADDR  = 0x27 # I2C device address
LCD_WIDTH = 16   # Maximum characters per line

# Define some device constants
LCD_CHR = 1 # Mode - Sending data
LCD_CMD = 0 # Mode - Sending command

LCD_LINE_1 = 0x80 # LCD RAM address for the 1st line
LCD_LINE_2 = 0xC0 # LCD RAM address for the 2nd line
LCD_LINE_3 = 0x94 # LCD RAM address for the 3rd line
LCD_LINE_4 = 0xD4 # LCD RAM address for the 4th line

LCD_BACKLIGHT  = 0x08  # On
#LCD_BACKLIGHT = 0x00  # Off

ENABLE = 0b00000100 # Enable bit

# Timing constants
E_PULSE = 0.0005
E_DELAY = 0.0005

#Open I2C interface
#bus = smbus.SMBus(0)  # Rev 1 Pi uses 0
bus = smbus.SMBus(1) # Rev 2 Pi uses 1

def lcd_init():
  # Initialise display
  lcd_byte(0x33,LCD_CMD) # 110011 Initialise
  lcd_byte(0x32,LCD_CMD) # 110010 Initialise
  lcd_byte(0x06,LCD_CMD) # 000110 Cursor move direction
  lcd_byte(0x0C,LCD_CMD) # 001100 Display On,Cursor Off, Blink Off 
  lcd_byte(0x28,LCD_CMD) # 101000 Data length, number of lines, font size
  lcd_byte(0x01,LCD_CMD) # 000001 Clear display
  time.sleep(E_DELAY)

def lcd_byte(bits, mode):
  # Send byte to data pins
  # bits = the data
  # mode = 1 for data
  #        0 for command

  bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT
  bits_low = mode | ((bits<<4) & 0xF0) | LCD_BACKLIGHT

  # High bits
  bus.write_byte(I2C_ADDR, bits_high)
  lcd_toggle_enable(bits_high)

  # Low bits
  bus.write_byte(I2C_ADDR, bits_low)
  lcd_toggle_enable(bits_low)

def lcd_toggle_enable(bits):
  # Toggle enable
  time.sleep(E_DELAY)
  bus.write_byte(I2C_ADDR, (bits | ENABLE))
  time.sleep(E_PULSE)
  bus.write_byte(I2C_ADDR,(bits & ~ENABLE))
  time.sleep(E_DELAY)

def lcd_string(message,line):
  # Send string to display

  message = message.ljust(LCD_WIDTH," ")

  lcd_byte(line, LCD_CMD)

  for i in range(LCD_WIDTH):
    lcd_byte(ord(message[i]),LCD_CHR)

def printText(line1,line2,SleepTime):
    lcd_string(line1,LCD_LINE_1)
    lcd_string(line2,LCD_LINE_2)
    time.sleep(SleepTime)

def ReadDeviceValue(room,device):
    database = firebase.database()                             #take an instance from the firebase database which is pointing to the root directory of your database.
    ButtonsBucket = database.child("Buttons")                  #get the child path in your database and store it inside the "RGBControlBucket" variable.
    BathroomBucket = ButtonsBucket.child(room)           #get the child path from the child and store it
    DeviceBucket=BathroomBucket.child(device).get()
    state=DeviceBucket.val()
#    print("{} is {}".format(device,state))
    return(state)

def ReadSettingsValue(device):
    database = firebase.database()                             #take an instance from the firebase database which is pointing to the root directory of your database.
    SettingsBucket = database.child("Settings")                  #get the child path in your database and store it inside the "RGBControlBucket" variable.
    DeviceBucket=SettingsBucket.child(device).get()
    state=DeviceBucket.val()
    return(state)

def WriteDeviceValue(room,device,value):
    database = firebase.database()                             #take an instance from the firebase database which is pointing to the root directory of your database.
    ButtonsBucket = database.child("Buttons")                  #get the child path in your database and store it inside the "RGBControlBucket" variable.
    BathroomBucket = ButtonsBucket.child(room)    
    BathroomBucket.update({device:value})
    
def setPinState(room,device,status):
    if (status=="ON"):
        GPIO.output(rooms[room][device],GPIO.HIGH)
    else:
        GPIO.output(rooms[room][device],GPIO.LOW)
#     time.sleep(0.01)

def getTemperature():
    _,temperature = Adafruit_DHT.read_retry(DHT_SENSOR,DHT_PIN)
    return(round(temperature,2))

def capture_image(imageName):
    local_path = "/home/pi/firebase/{}.jpg".format(imageName)
    path_on_firebase="Security/{}.jpg".format(imageName)
    s='fswebcam -r 255x255 --no-banner -S 15 --jpeg 95 --save {}'.format(local_path)
    os.system(s)
    storage=firebase.storage()
    storage.child(path_on_firebase).put(local_path)


def updateTemp(desTemp):    
    desiredTemp="Desired T: {}".format(desTemp)
    actualTemp="Actual T: {}".format(str(getTemperature()))
    printText(actualTemp,desiredTemp,2)
    _,__,___=tv_init()
    
channels={"1":"TV5","2":"Nat Geo","3":"France 3","4":"SpaceToon"}

def tv_init():
    tvState=ReadDeviceValue("TV","state")
    channelNum=ReadDeviceValue("TV","channel")
    volume=ReadDeviceValue("TV","volume")
    if(tvState=="OFF"):
        printText("TV is OFF","",0.01)
    else:
        line1="Channel {}".format(channelNum)
        line2=channels[channelNum]
        printText(line1,line2,0.01)
    return tvState,channelNum,volume

def tv_on():
    printText("TV is ON","",2.0)
    channelNum=ReadDeviceValue("TV","channel")   
    printText("Channel {}".format(channelNum),channels[channelNum],0.01)
    
def tv_off():
    printText("TV is OFF","",0.01)
        
def tv_volume_update(volume):
    printText("Volume",volume,2.0)
    channelNum=ReadDeviceValue("TV","channel")
    printText("Channel {}".format(channelNum),channels[channelNum],0.01)
    
def tv_channel_update(channelNum):
    printText("Channel {}".format(channelNum),channels[channelNum],0.01)    

def stream_handler(message):
    words=message["path"].split('/')
    if (len(words)==3):
        room=words[1]
        device=words[2]
        data=message["data"]
#TV
        if(room=="TV"):
            if(device=="state"):
                if(data=="ON"):
                    tv_on()
                else:
                    tv_off()
            elif(device=="channel"):
                if(ReadDeviceValue("TV","state")=="ON"):
                    tv_channel_update(data)
            elif(device=="volume"):
                if(ReadDeviceValue("TV","state")=="ON"):
                    tv_volume_update(data)
#camera            
        elif(room=="Camera"):
            if((device=="capture") and (data=="ON")):
                capture_image("capture")
                WriteDeviceValue("Camera","capture","DONE")
            elif((device=="suspect") and (data=="ON")):
                capture_image("suspect")
                WriteDeviceValue("Camera","suspect","OFF")
#door
        elif(room=="Door"):
            if((device=="knock") and (data=="ON")):
                subprocess.call(['/home/pi/Desktop/door.sh'])                    
                WriteDeviceValue("Door","knock","OFF")
                if(ReadDeviceValue("Camera","suspect")=="ON"):
                    capture_image("suspect")
                    WriteDeviceValue("Camera","suspect","OFF")
#room        
        else:
            setPinState(room,device,data)

# rooms={"Bathroom":{"pompe":25,"light_bath":17,"win_bath":4},
#    "Bedroom":{"ac_bed":27,"light_bed":22,"win_bed":10},
#    "kitchen":{"ac_kitchen":9,"light_kitchen":12,"win_kitchen":13},
#    "L_room":{"ac_l_room":26,"light_l_room":23,"win_l_room":24},
#     "Camera":{"capture":0,"suspect":0}}


try:
    lcd_init()
    tv_init()
    database = firebase.database()
    my_stream = database.child("Buttons").stream(stream_handler)
except :
    GPIO.cleanup()
    print("\nBye Bye")
    lcd_byte(0x01, LCD_CMD)
    my_stream.close()