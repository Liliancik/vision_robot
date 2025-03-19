import RPi.GPIO as GPIO
import time
from CRobot import CRobot

LMPins = (8, 11)
RMPins = (10, 18)
PWMPins = (7, 9)
Robot = CRobot(LMPins, RMPins, PWMPins)

el = 4
er = 26

countl = 0
countr = 0


GPIO.setup(el, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(er, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def encoder_callback_left(channel):
    global countl
    countl += 1

def encoder_callback_right(channel):
    global countr
    countr += 1

# Add event detection for encoder pins
GPIO.add_event_detect(el, GPIO.RISING, callback=encoder_callback_left)
GPIO.add_event_detect(er, GPIO.RISING, callback=encoder_callback_right)

    
    
try:
    while(countl < 20 and countr < 20):
        Robot.backward(0.5)
    else:
        Robot.stop()
        countl = 0
        countr = 0
    
    time.sleep(2)
    
    while(countl < 19 and countr < 20):
        Robot.left(0.7)
    else:
        Robot.stop()
        print(f"Left encoder count: {countl}")
        print(f"Right encoder count: {countr}")
        countl = 0
        countr = 0
    
    time.sleep(2)
    
    
    

    
    while(countl < 20 and countr < 20):
        Robot.backward(0.5)
    else:
        Robot.stop()
        countl = 0
        countr = 0
    
    time.sleep(2)
        
    
    while(countl < 19 and countr < 20):
        Robot.right(0.7)
    else:
        Robot.stop()
        print(f"Left encoder count: {countl}")
        print(f"Right encoder count: {countr}")
        countl = 0
        countr = 0
    
    time.sleep(2)
    

    
    while(countl < 20 and countr < 20):
        Robot.backward(0.5)
    else:
        Robot.stop()
        countl = 0
        countr = 0
    
    time.sleep(2)
    
    while(countl < 19 and countr < 20):
        Robot.right(0.7)
    else:
        Robot.stop()
        print(f"Left encoder count: {countl}")
        print(f"Right encoder count: {countr}")
        countl = 0
        countr = 0
    
    time.sleep(2)


    
    while(countl < 20 and countr < 20):
        Robot.backward(0.5)
    else:
        Robot.stop()
        countl = 0
        countr = 0
    
    time.sleep(2)
    
    # Move Motor A forward at 50% speed
    while(countl < 20 and countr < 20):
        Robot.left(0.7)
    else:
        Robot.stop()
        


    # Move Motor B forward at 75% speed


    print(f"Left encoder count: {countl}")
    print(f"Right encoder count: {countr}")

    

except KeyboardInterrupt:
    print("Program interrupted")

finally:

    GPIO.cleanup()
    print("GPIO cleaned up")
