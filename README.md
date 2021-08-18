# Bikeduino

Used ATmega328P on arduino with implementation in C

Link to Devpost -> [Bikeduino Devpost](https://devpost.com/software/bikeduino?ref_content=my-projects-tab&ref_feature=my_projects)

Link to Final Demo -> [Bikeduino Final Demo](https://youtu.be/XtnqrxLER8M)

The code is split into two main categories. First, halleffect.c contains the code for initializing and running the speedometer / odometer system. This file utilizes the LCD.c file to print items to our LCD screen. Next, photoresistor.c contains the implementation for the LED. This is where our ADC thresholds were implemented and used to change the brightness of the LED on our bike.

LCD.c and LCD.h are files created by us using the LCD implementation in C from [Donald Weiman](http://web.alfredstate.edu/faculty/weimandn/programming/lcd/ATmega328/LCD_code_gcc_8d.html). The code was split and edited in the initialization steps; however, we utliized his functions to send data to our LCD screen. This allowed us to write to the screen using C in Atmel rand gave us a deeper understanding of how the LCD worked rather than using LiquidCrystal commonly used in the Arduino IDE.
