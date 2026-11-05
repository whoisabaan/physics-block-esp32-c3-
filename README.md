# physics-block-esp32-c3-
this code for a 'physics block' which tells multple physical values (acceleration x,y,z merged , velocity, distance travelled etc. this is a part of a much bigger project named modulas )

to make it work - use bmi160 module's i2c pins and set i2c_addr to 0x69 by connectind sdo to gnd.

 BMI160 --- ESP32 C3

          SDA --- SDA
          SCL --- SCL  
          GND --- GND
          VCC --- 3V3

SDO --- GND (YOU CAN USE GND ON BMI160 OR ESP32 C3)

if you consider changing the MCU make sure to change the wire sda scl config inside setup function. (the line  ->  wire(sda of your board,scl of your board);)
