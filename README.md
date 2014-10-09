Kamstrup602
===========

Using an Arduino Uno R3 to read parameters of a Kamstrup 602 by using the Kamstrup Metering Protocol (KMP). Readings are transfered over WIFI (using the Arduino WIFI shield and HTTP GETs) to a Synology NAS running MySQL. Java script code shows readings as graphs.

Note that this application uses a modified version of the SoftwareSerial library (with the transmitter inverted).
