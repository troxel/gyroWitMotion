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

I came across an issue when rotating on a turntable the angular velocity output from the HWT905 output drops to zero for some reason after a few seconds. Then when the rotation reverses direction the output from the HWT905 is double the actual rotation rate. And then again the output from the HWT905 drops to zero during constant rotation. See the plot
of a ground reference gyro in the docs directory. In the plot "gyro1" is the HWT905 with gyro calibration selected and other "Gyro2" is a reference gyro and demonstrated the true state of rotation. 

![Gyro Traces](/docs/gyro_traces.jpg)

Witmotion acknowledged this issue but only suggested to remove the "gyro calibration" in the configuration. Perhaps sometime they will fix this but until then this is a major problem for our intended use. 

Note: Without the gyro calibration option selected the HWT905 gyro wander bias is about 60 degrees per hour perhaps closer to 70 degree per hour. 
