#! /usr/bin/python3
# safe_restart_shutdown_interrupt_Pi.py
#
# -----------------------------------------------------------------------------
#                 Raspberry Pi Safe Restart and Shutdown Python Script
# -----------------------------------------------------------------------------
# WRITTEN BY: Ho Yun "Bobby" Chan
# @ SparkFun Electronics
# MODIFIED: 3/18/2021
# DATE: 3/31/2020
#
#
# Based on code from the following blog and tutorials:
#
#    Kevin Godden
#    https://www.ridgesolutions.ie/index.php/2013/02/22/raspberry-pi-restart-shutdown-your-pi-from-python-code/
#
#    Pete Lewis
#    https://learn.sparkfun.com/tutorials/raspberry-pi-stand-alone-programmer#resources-and-going-further
#
#    Shawn Hymel
#    https://learn.sparkfun.com/tutorials/python-programming-tutorial-getting-started-with-the-raspberry-pi/experiment-1-digital-input-and-output
#
#    Ben Croston raspberry-gpio-python module
#    https://sourceforge.net/p/raspberry-gpio-python/wiki/Inputs/
#
# ==================== DESCRIPTION ====================
#
# This python script takes advantage of the Qwiic pHat v2.0's
# built-in general purpose button to safely reboot/shutdown you Pi:
#
#    1.) If you press the button momentarily, the Pi will reboot.
#    2.) Holding down the button for about 3 seconds the Pi will shutdown.
#
# This example also takes advantage of interrupts so that it uses a negligible
# amount of CPU. This is more efficient since it isn't taking up all of the Pi's
# processing power.
#
# ========== TUTORIAL ==========
#  For more information on running this script on startup,
#  check out the associated tutorial to adjust your "rc.local" file:
#
#        https://learn.sparkfun.com/tutorials/raspberry-pi-safe-reboot-and-shutdown-button
#
# ========== PRODUCTS THAT USE THIS CODE ==========
#
#   Feel like supporting our work? Buy a board from SparkFun!
#
#        Qwiic pHAT v2.0
#        https://www.sparkfun.com/products/15945
#
#   You can also use any button but you would need to wire it up
#   instead of stacking the pHAT on your Pi.
#
# LICENSE: This code is released under the MIT License (http://opensource.org/licenses/MIT)
#
# Distributed as-is; no warranty is given
#
# -----------------------------------------------------------------------------

import os
import time
#import RPi.GPIO as GPIO #Python Package Reference: https://pypi.org/project/RPi.GPIO/
import OPi.GPIO as GPIO #Python Package Reference: https://pypi.org/project/OPi.GPIO/

# Must be root
if os.geteuid() != 0:
    print("must run as root")
    exit(1)

# Pin definition
# RPi
#reset_shutdown_pin = 17
#led_pulldown_pin = 22
# OPi
reset_shutdown_pin = 11 # GPIO 70  # PC6
led_pulldown_pin = 15 # GPIO 72  # PC8

# Suppress warnings
GPIO.setwarnings(False)

# Use "GPIO" pin numbering - ORANGE PI ZERO 3 SPECIFIC
GPIO.setmode(GPIO.CUSTOM)  # See /usr/local/lib/python3.8/dist-packages/OPi/pin_mappings.py

# Use built-in internal pullup resistor so the pin is not floating
# if using a momentary push button without a resistor.
#GPIO.setup(reset_shutdown_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Use Qwiic pHAT's pullup resistor so that the pin is not floating
# Or in my case, a physical resistor
GPIO.setup(reset_shutdown_pin, GPIO.IN)

GPIO.setup(led_pulldown_pin, GPIO.OUT)
GPIO.output(led_pulldown_pin, 0)  # Led off

time.sleep(1)
GPIO.output(led_pulldown_pin, 1)  # Led off
time.sleep(1)
GPIO.output(led_pulldown_pin, 0)  # Led off

# modular function to restart Pi
def restart():
    print("restarting Pi")
    GPIO.output(led_pulldown_pin, 1)
    command = "/sbin/shutdown -r now"
    import subprocess
    process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
    output = process.communicate()[0]
    print(output)

# modular function to shutdown Pi
def shut_down():
    print("shutting down")
    GPIO.output(led_pulldown_pin, 1)   # TBD: Does it ever shut off?
    command = "/sbin/shutdown -h now"
    import subprocess
    process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
    output = process.communicate()[0]
    print(output)


print("Configured. Starting monitoring")
while True:
    #short delay, otherwise this code will take up a lot of the Pi's processing power
    time.sleep(0.5)

    # wait for a button press with switch debounce on the falling edge so that this script
    # is not taking up too many resources in order to shutdown/reboot the Pi safely
    #channel = GPIO.wait_for_edge(reset_shutdown_pin, GPIO.FALLING, bouncetime=200)  # RPi version
    #print(help(GPIO.wait_for_edge))
    channel = GPIO.wait_for_edge(reset_shutdown_pin, GPIO.FALLING)#, bouncetime=200)

    if channel is None:
        print('Timeout occurred')
    else:
        print('Edge detected on channel', channel)

        # For troubleshooting, uncomment this line to output button status on command line
        #print('GPIO state is = ', GPIO.input(reset_shutdown_pin))
        counter = 0

        while GPIO.input(reset_shutdown_pin) == False:
            # For troubleshooting, uncomment this line to view the counter. If it reaches a value above 4, we will restart.
            #print(counter)
            counter += 1
            time.sleep(0.5)

            # long button press
            if counter > 4:
                shut_down()

        #if short button press, restart!
        #restart()  # Let's try this one again later.

