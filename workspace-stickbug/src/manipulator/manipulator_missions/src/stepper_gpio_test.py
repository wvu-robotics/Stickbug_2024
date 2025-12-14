import RPi.GPIO as GPIO
import time

# Pin Definitions
PUL = 11 # BCM pin 18, BOARD pin 12
DIR = 7
ENA = 40

def main():
    # Pin Setup:
    GPIO.setmode(GPIO.BOARD)  # BCM pin-numbering scheme from Raspberry Pi
    # set pin as an output pin with optional initial state of HIGH
    GPIO.setup(PUL, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(DIR,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(ENA,GPIO.OUT,initial=GPIO.HIGH)

    GPIO.output(ENA,GPIO.HIGH)
    GPIO.output(DIR,GPIO.HIGH)


    print("Starting demo now! Press CTRL+C to exit")
    try:
        while True:
            time.sleep(.5) # microseconds 100/1000000.0
            GPIO.output(PUL, GPIO.HIGH)
            #print("high")
            time.sleep(.5) # 100/1000000.0
            GPIO.output(PUL, GPIO.LOW)
            #print("low")
            
    finally:
        GPIO.cleanup()

if __name__ == '__main__':
    main()
