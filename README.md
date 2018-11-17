# PWN-Fan-Controller

This project was completed for the course: MECHTRON 3TA4, Embedded Systems Design 1 @ McMaster University(Fall 2018).

The user is able to select a setpoint temperature; which when exceeded signals the fan to turn on. 
An LM35 temperature sensor is used to detect the temperature. The analog signal is amplified with an LM358 OpAmp before being fed into the STM32 board. 

The C code converts the analog signal to digital every half-second and checks if the temperature exceeds the setpoint temperature. If it does; the fan turns and stays on until the temperature falls below the setpoint. 

The temperature displays on the board's LCD screen, and the setpoint mode(when accessed) is also displayed on the screen.
