# Elegoo Smart Robot Car Kit V3.0 Plus
 
Alternative car firmware adding music playback, color lights, car speed setting, and a follow mode.
This car you can also easily build yourself using common Arduino Uno components. Just connect them to the correponding pins (see #defines in the .ino code). Program your Arduino Uno using Arduino IDE.

You can download the complete documentation and instructions from
https://www.elegoo.com/download/
if you need more details.

The App to control your car via Bluetooth:
ELEGOO BLE TOOL
https://play.google.com/store/apps/details?id=air.com.elegoo.elegooTool
This app also works with common HM-10 BLE module (in the app make a scan and manually select your BLE module). 
The app offers a great "Program"-mode for graphical programming of the car.

To benefit from music and lights, connect a passive buzzer via 100 Ohm resistor to the pin
	BEEP_PIN A3
and a 3-color LED via 220 Ohm resistors to the pins
	RGB1_PIN A2
	RGB2_PIN A1
	RGB3_PIN A0

In the App select "Smart Robot Car" and add the following commands to the "DIY Control":
f - forward, b - backward, l - left, r - right, s - stop, 1 - line tracking, 2 - object avoidance, 3 - follow, 
4, 5, 6, 0 - play music, 7, 8, 9 - toggle LED RGB colors, * - decrease car speed, # - increase car speed.
	
Try all the keys on IR remote to explore the new features :-)
Enjoy!
Merry Christmas and Happy New Year 2021!
Alexey
