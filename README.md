# gyroWitMotion
A C based console program to read and display data from a WitMotion (MPU6050 based) ARHS. Specifically this has been tested with the WitMotion HWT905 
and BWT901CL but I suspect will work with other similar Wit Motion devices. 

# Synopsis

> ./gyroWitMotion commdevice shmmemFile

ie

> ./bin/gyroWitMotion /dev/ttyUSB2 /dev/shm/gyro0

# Issues

In evaluating the gyro 

https://www.wit-motion.com/digital-inclinometer/witmotion-hwt905-rs232.html

I came across an issue when rotating on a turntable the angular velocity output from the HWT905 output drops to zero for some reason after a few seconds. Then when the rotation reverses direction the output from the HWT905 is double the actual rotation rate. And then again the output from the HWT905 drops to zero during constant rotation. See attached plot with a ground reference gyro or in the docs directory. 

![Gyro Traces](/docs/gyro_traces.jpg)

Witmotion acknowledged this issue but only suggested to remove the "gyro calibration" in the configuration.

Note: without the gyro calibration option select the gyro wander bias is about 60 degrees per hour. 
