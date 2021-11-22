# LidarStm32f103 <br>
ladar_MB_1R2T connection stm32f103 board <br>
An example of connecting Chinese lidar model MB-1R2T V1.5.8 to stm32f103.
As an example, the developer board described here is used: https://github.com/pav2000/DevBoardSTM32F103CBT <br>
<img src="https://github.com/pav2000/LidarStm32f103/blob/main/demo.gif" width="300" /> <br>
<br>
Starting from software version 0.35 transition to hardware 1.4.<br>
Rotating the encoder changes the scale of the screen (from 1 to 6). The blue LED indicates the reception of the packet
from the lidar. The red LED blinks when there are errors in the input packet.<br>
The connection of the periphery can be viewed in the file LidarStm32f103.pdf, where the microcontroller settings are indicated.<br>
<br>
UART connection parameters: <br>
BR 153600 (bit 6.51us) <br>
8 bit <br>
parity none <br>
stop bit 1 <br>
Bit format LSB <br>
UART connection<br>
<img src="https://github.com/pav2000/LidarStm32f103/blob/main/lidar.jpg" width="480" /> <br>
<br>
Speed control:<br>
max speed(pwm to GND) about 9 hz and 200 measurements per rotation<br>
min speed (pwm to 5v) about 3.7hz 1024 measurements per rotation<br>
<br>
Package structure:<br>
1.0xAA, 0x55 - header<br>
2.0x38 - lidar type (firmware type ??) possible values 0x28, 0x3С. We need to find out! And change it in the code!<br>
3.0x28 - number of measurements in the package<br>
4.2 bytes - starting angle of sending (least significant byte first)<br>
5.2 bytes - end angle of sending (least significant byte first)<br>
6.2 bytes ????<br>
7. Data - length number of measurements (point 3) * 3 bytes. Each dimension is three bytes long. 1 byte - signal quality, 2 and 3 - distance to the object (least significant byte first)<br>
<br>
