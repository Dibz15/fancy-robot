
import pigpio as io
import time as time
import sys
import os


def servo_pt(pi, pan, tilt):
    servo_to_angle(pi, s_pan, pan)
    servo_to_angle(pi, s_tilt, tilt)

def servo_add(pi, servo_pin, addition):
    pwm = pi.get_servo_pulsewidth(servo_pin)
    angle = pwm_to_angle(float(pwm))
    added = angle + addition
    if added < 180:
        servo_to_angle(pi, servo_pin, added)
    else:
        servo_to_angle(pi, servo_pin, 180)    


def servo_sub(pi, servo_pin, subtraction):
    pwm = pi.get_servo_pulsewidth(servo_pin)
    angle = pwm_to_angle(float(pwm))
    subtracted = angle - subtraction
    if subtracted > 0:
    	servo_to_angle(pi, servo_pin, subtracted)
    else:
	servo_to_angle(pi, servo_pin, 0)	

def servo_to_angle(pi, servo_pin, angle):
    angle2 = -1
    try:
        angle2 = float(angle)
    except:
	return
    else:
	pi.set_servo_pulsewidth(servo_pin, angle_to_pwm(angle2))

def angle_to_pwm(angle):
	normalized = normalize(0, 180, angle, 500, 2500)
	return normalized
	
def pwm_to_angle(pwm):
	normalized = normalize(500, 2500, pwm, 0, 180)
	return normalized


#DC motor stuff
def set_motor_duty(pi, motor_pin, duty):
	pi.set_PWM_dutycycle(motor_pin, duty)

def set_motor_dir(pi, motor, dir, duty):
	if motor == left_motor:
		if dir == FORWARD:
			set_motor_duty(pi, left_forward, duty)	
		elif dir == REVERSE:
			set_motor_duty(pi, left_reverse, duty)			
	elif motor == right_motor:
		if dir == FORWARD:
			set_motor_duty(pi, right_forward, duty)
		elif dir == REVERSE:
			set_motor_duty(pi, right_reverse, duty)


"""  * Normalizes a value by placing it proportionately between a <i>smaller</i> upper and lower limit given a <i>larger</i> upper and lower limit
	 * 
	 * @param valueMax - the maximum real value in the range, which would equal the maximum normal value
	 * @param valueMin - the minimum real value in the range, which would equal the minimum normal value
	 * @param value - the real value to normalize
	 * @param normalMax - the maximum normalized value
	 * @param normalMin - the minimum normalized value
	 * @return - the value as a normal, proportionately situated in between the normalMax and normalMin
"""
def normalize(valueMin, valueMax, value, normalMin, normalMax):
	#  180           180      0   
    valueRange = valueMax - valueMin
	#     2000       2500         500
    normalRange = normalMax - normalMin
	#    .5           90        180          180
    valuePosition = (value - valueMin) / valueRange
	#     1000                0.5            2000
    normalPosition = valuePosition * normalRange
	#   1500           500           1000
    normalized = normalMin + normalPosition
    return normalized


left_forward = 21
left_reverse = 20

right_forward = 19
right_reverse = 26

left_motor = 1
right_motor = 2

FORWARD = 1
REVERSE = -1

s_tilt= 13
s_pan = 6






