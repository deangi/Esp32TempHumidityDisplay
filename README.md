Uses an ESP32 "Cheap Yellow Display" for a temperature and humidity monitor with graphs of temperature and humidity over time.
These boards have a 2.8 inch color display with resistive touch capability.
These are available from several sources on Amazon for less than $20.
Here are some example pictures.

<img width="679" height="666" alt="image" src="https://github.com/user-attachments/assets/450a6d0b-aca0-4f9e-96b0-5960f0d6f1e2" />

<img width="1145" height="915" alt="image" src="https://github.com/user-attachments/assets/93b50d03-ff4b-4302-8da4-cde2f13ee333" />

On the back side there is a connector that can support connection to a sensor/ 
My project used the DHT-11 temperature and humidity sensor - also widely available on Amazon.

For the board I had available, I used the a connector labeled CN1 or I2C which had 3.3V, GND, IO22, and IO27 available.
It is labeled CN1 on my board.  Different CYD boards may have diffences on the interface connectors.   
Look for a connector that has 3.3V and GND at least one GPIO pin.   
If it's not IO22 then you'll have to change the DHT_PIN define in the source INO sketch file.

#define DHT_PIN   22

For example on the picture of the back side of the board above, the project would use the I2C connector (top-right) where you see 3.3V, GND, IO25 and IO32.
If you use the IO25 pin to connect to the sensor (along with 3.3V and GND) then the source code would be modified (one line) like this:

#define DHT_PIN   25

The DHT=11 sensor requires just three connections, +3.3V, Ground, and one IO pin. 
With my board, in this project, the IO pin is connected to the ESP32 IO22 GPIO.
But for the board pictured, you might use IO25.

Front view showing display and plot of temperature:

<img width="320" height="230" alt="IMG_1287" src="https://github.com/user-attachments/assets/8af674dc-c95d-4a22-9fd7-9b4edab8eedc" />

Back view showing how DHT-11 is plugged into connector CN1 (top of board above the LED):

<img width="320" height="240" alt="IMG_1288" src="https://github.com/user-attachments/assets/a9b3cd22-dcab-49be-a9b1-b731a334baaf" />

Detail of DHT-11 sensor connection showing three wires, RED=3.3V, Black=GND, Blue=-IO22:

<img width="240" height="320" alt="IMG_1289" src="https://github.com/user-attachments/assets/9dd6489e-24b9-498f-9c0d-318635e46929" />


For Arduino IDE, board is set to ESP32-WROOM module, partitioning is set to default.  

Uses the Adafruit DHT library for reading data from the DHT-11

Uses the LovyanGFX driver for the display panel.

More information in the comments of the sketch file:

Only two source files are used.

ESP32_SPI_8341.h - defines the LovyanGFX interface.
TempHumidityDisplay.ino - sketch file for the CYD ESP32 board.

I used a 3D printer to make a case - printables.com has several sample cases for the CYD board.










