import motors as mo
import time
import sys
import os
import pigpio as io
import curses

os.system("clear")
os.system("pigs SERVO 6 1500")
os.system("pigs SERVO 13 1500")

DUTY = 255

left_motor = 1
right_motor = 2

FORWARD = 1
REVERSE = -1

s_tilt= 13
s_pan = 6

pi = io.pi()
pi.set_mode(s_pan, io.OUTPUT)
pi.set_mode(s_tilt, io.OUTPUT)
pi.set_mode(mo.left_forward, io.OUTPUT)
pi.set_mode(mo.left_reverse, io.OUTPUT)
pi.set_mode(mo.right_forward, io.OUTPUT)
pi.set_mode(mo.right_reverse, io.OUTPUT)

pi.set_PWM_frequency(mo.left_forward, 20)
pi.set_PWM_frequency(mo.left_reverse, 20)

pi.set_PWM_frequency(mo.right_forward, 20)
pi.set_PWM_frequency(mo.right_reverse, 20)

def run(screen, command):
	screen.clear()
	screen.border(0)
		
        if command == "exit":
            	return True
        elif command == "up" or command == "u":
            	mo.servo_sub(pi, s_tilt, 20)
        elif command == "down" or command == "d":		
            	mo.servo_add(pi, s_tilt, 20)
        elif command == "sleft" or command == "sl":
             	mo.servo_add(pi, s_pan, 20)
        elif command == "sright" or command == "sr":
            	mo.servo_sub(pi, s_pan, 20)
        elif "co" in command:
		values = command.split(" ")
		pan_angle = values[1]
		tilt_angle = values[2]
		mo.servo_pt(pi, pan_angle, tilt_angle)
	elif command == "forward" or command == "f":
		mo.set_motor_dir(pi, left_motor, FORWARD, DUTY)
		mo.set_motor_dir(pi, right_motor, FORWARD, DUTY)
		print "Motor Duty: ", pi.get_PWM_dutycycle(left_forward)
	elif command == "reverse" or command == "re":
		mo.set_motor_dir(pi, left_motor, REVERSE, DUTY)
		mo.set_motor_dir(pi, right_motor, REVERSE, DUTY)
	elif command == "left" or command == "l":
		mo.set_motor_dir(pi, left_motor, REVERSE, DUTY)
		mo.set_motor_dir(pi, right_motor, FORWARD, DUTY)
	elif command == "right" or command == "ri":
		mo.set_motor_dir(pi, left_motor, FORWARD, DUTY)
		mo.set_motor_dir(pi, right_motor, REVERSE, DUTY)
	elif command == "stop" or command == "s":
		mo.set_motor_duty(pi, mo.left_forward, 0)
		mo.set_motor_duty(pi, mo.left_reverse, 0)
		mo.set_motor_duty(pi, mo.right_forward, 0)
		mo.set_motor_duty(pi, mo.right_reverse, 0)
	elif "duty" in command:
            	values = command.split(" ")
            	global DUTY 
		DUTY = float(values[1])	
	else:
		pass	

def get_input(prompt):
	screen.clear()
	screen.border(0)
	screen.addstr(2, 2, prompt)
	screen.refresh()
	input = screen.getstr(10, 10, 60)
	return input

def stop_motors():
	mo.set_motor_duty(pi, mo.left_forward, 0)
        mo.set_motor_duty(pi, mo.left_reverse, 0)
       	mo.set_motor_duty(pi, mo.right_forward, 0)
        mo.set_motor_duty(pi, mo.right_reverse, 0)

def keyboard_control(screen):
#	screen.nodelay(1)
	while True:
		screen.clear()
		string = "Enter Key Inputs. Use 'q' to exit. Duty: " + str(DUTY)
        	screen.addstr(2, 2, string)
        	#screen.refresh()
		pressed = False
        	char = screen.getch()
		if char == 113: 
			break
		elif char == curses.KEY_RIGHT or char == 54:
			pressed = True
			stop_motors()
			mo.set_motor_dir(pi, left_motor, FORWARD, DUTY)
                	mo.set_motor_dir(pi, right_motor, REVERSE, DUTY)	
		elif char == curses.KEY_LEFT or char == 52:
			pressed = True
			stop_motors()
			mo.set_motor_dir(pi, right_motor, FORWARD, DUTY)
	                mo.set_motor_dir(pi, left_motor, REVERSE, DUTY)
		elif char == curses.KEY_UP or char == 56:
			pressed = True
			stop_motors()
			mo.set_motor_dir(pi, left_motor, FORWARD, DUTY)
	                mo.set_motor_dir(pi, right_motor, FORWARD, DUTY)
		elif char == curses.KEY_DOWN or char == 53:
			pressed = True
			stop_motors()
			mo.set_motor_dir(pi, left_motor, REVERSE, DUTY)
	                mo.set_motor_dir(pi, right_motor, REVERSE, DUTY)
		elif char == 119: #w
			mo.servo_add(pi, s_tilt, 3)
		elif char == 97:  #a
			mo.servo_add(pi, s_pan, 3)
		elif char == 115: #s
			mo.servo_sub(pi, s_tilt, 3)
		elif char == 100: #d
			mo.servo_sub(pi, s_pan, 3)
		elif char == ' ':
			stop_motors()
		elif char == 60:
			global DUTY
			if DUTY > 5:
				DUTY -= 10
		elif char == 62:
			global DUTY
			if DUTY <= 245:
				DUTY += 10
		if pressed == False:
			stop_motors()

		time.sleep(0.01)



screen = curses.initscr()

def ui():
	x = 0
	while x != ord('3'):
		screen = curses.initscr()
		
		screen.clear()
		screen.border(0)
		screen.addstr(2, 2, "Please enter mode number: ")
		screen.addstr(4, 4, "1 - Old command interface")
		screen.addstr(5, 4, "2 - Keyboard Controller")
		screen.addstr(6, 4, "3 - Exit")
		screen.refresh()

		x = screen.getch()

		if x == ord('1'):
			while True:
				exit = run(screen, get_input("Command: "))
				if exit:
					break
		
		if x == ord('2'):
			keyboard_control(screen)
						
			

ui()

curses.endwin()
pi.stop
sys.exit()
