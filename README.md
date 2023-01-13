# Microcontrollers
In this assignment, you will design and Control a Cooler in order to Control the temperature of a
house by using Pulse Width Modulation and Proportional Control as Follows. We also Use a
heater but we will NOT control the Temperature in this case
Analog Input 0 (AI0): It is the (Pot. P1), It will be used to set the Set Point. It is a Potentiometer
of 10K Ohm. Scale the value read to 0 – 100C.
We will Refer to it as SP (Set Point Temperature).
Analog Input 2 (AI2): This is already connected to a simulator that reads a voltage already
connected to a sensor. Read the Voltage (0 –5) then scale it to temperature by just multiplying
it by 100 since an increase of 10ms represent an increase by 1C. The minimum Value that it
reads is around 27 degrees without doing extra changes. We will leave it as is to simplify things
and we will control Temperatures above the minimum Value. The Value of this sensor is the
actual temperature of the room. 
We shall Refer to it as T (Temperature) or RT (Room Temperature).
Heater: This is already connected to RC5. We will turn Control the RC5 by generating a Pulse
width Modulated Signal. We will not use an actual PWM signal but we by use Time3 Interrupt
to generate a Pulse Width Modulation 
Cooler (Fan): This is already connected to RC2 which is the CCP1. We will use actual Pulse width
Modulation to control the amount of Cooling.
Hysteresis (HS): This is a Value that we should set Digitally through Interrupt INT1 at RB1
(Decrement) and INT2 at RN2 (Increment). The HS should between 0 – 4. INT2 will increment HS
until it reaches 4 and will stay at 4. INT1 will decrement HS until it reaches 0 and will stay at 0.
Hysteresis’s has meaning only when in the Temperature Control Modes. INT1 and INT2 will be
used to increment/decrement different values depending on the Operating mode in a similar to
the description of setting the HS.
Cooling and Amount: In Cooling Mode , the amount of Heating or Cooling will be proportional
to the difference between the Actual Temperature and the Set Point as explained in the Modes
below.
Operation Modes: This is a Value that we should set Digitally through Interrupt INT0 at RB0.
There shall be 3 Modes: Off Mode, Heating, Cooling. main that The user can select (Mode 1: Off
Mode, Mode 2: Heat Mode and Mode 3: Cool Mode) as described below. INT0 will circulate
1
through the Modes (You can use 3 values e.g. (0 – 2) and circulate through them. Rollover when
the value exceeds 2.
.So, there will be 3 modes which are: Mode 1: Heat Mode, Mode 2: Auto Cool Mode 
The details of the modes are explained below. 
1. Mode 0: OFF mode. The system shall start in this mode on Power up. The System Shall
be turned OFF Heater OFF(0%) and Cooler OF(0%) if the user Presses RB3 which shall be
read in the main Loop. This is Mode 0 below.
.
2. Mode 1 (Heat): In this mode, simply turn ON the heater . The Cooler should be turned
Off. 
3. Auto Cool Mode: (displayed as Cool): 
In this Mode, we will control cooler such that the Temperature will be automatically
Controlled to become Almost Equal to the Set Point. In other words, we bring it to the
Set Point to within a Hysteresis(H) Value (A small value, we will restrict to a Max of 4 as
explained in Page 1 it can be from (0 --4). This Means that the Difference between the
actual Temperature and the Setpoint should be very small (No more than the H Value).
 We should Not Heat and Cool at the Same Time. 
Auto Cool: In this mode the Temperature should be controlled such that the Actual
Temperature should be around the Set Point referred to ad SP (SetPoint). The
Cooler should be controlled by the Pulse Width Modulation signal CCP1 at RC2. The
Value of the PWM percentage should be proportional to the CoolError = T – SP if T >
SP and should be turned Off if T < ( SP-HS). The percentage of Cooling (PWM
percentage value) should be set as follows.
 
 CoolError = T – SP
 If( CoolError > 0)
 PWM percentage value = CError*100/10; 
 Set the CCP1 PWM to PWM percentage value
If this value is less than 25% set it to 25%
 If( T < (SP -HS) )
PWM percentage value =0 ;// Cooler OFF
In real life, the Heater should be always off but here we are forced to make it ON half the time,
so we toggle it between ON and Off when T fall below (SP – HS), that is ( T < (SP -HS) )
2
because we cannot increase the temperature if we do not do that. in real life, in summer, the
temperature rises when we turn the cooler off, we cannot do that here.
To set the Heater to 50% ON (half the time) No pulse modulation here. You can use Timer 3 to
create an Interrupt say every 100 ms. Use the Interrupt toggle the Heater between ON and
OFF on. Make sure that you do that Only if this condition explained in the previous paragraph
occurs. So you can set a Variable if we are cooling and ( T < (SP -HS) ) and make the Interrupt
toggle in this case.
Notes: 
1. The temperature should be automatically updated as well as HS and the Mode if
changed.
2. The main program should an infinite loop and there should be noticeable flicker in the
screen. It should be continuously reading the Analog Inputs and displaying them. You
can put a small delay if needed.
3
3. The Interrupt code should be minimal, just change the Variables) and do actual
displaying in the Main Program.
4. Keep your set points less than 60 . There are special cases for the Heater, since this is a
simulator. You cannot make the Temperature more than 76 and cannot be less than 27.
5. Make sure when you enable your Interrupts to Enable the Most Two significant bits bit 7
and 6 in INTCON. (GIEH, and GIEL) Otherwise your Timer 3 interrupt will not work
correctly. Disable the Priority.
