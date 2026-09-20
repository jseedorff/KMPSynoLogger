Logger/reader for the Kamstrup 602 electronic district heating meter.
- Using an Arduino Uno R3 and Arduino WiFi Shield to read parameters of a Kamstrup 602 (by using the KMP protocol).
- Readings are transfered to a Synology NAS running MySQL. PHP and Java script code can be used to show readings as graphs.
- Project makes use of the KMPSoftwareSerial and Time libraries (present in the Library repository).
- Physical interface between the Kamstrup KMP and the Arduino I/Os: The Arduino output connects to the KMP REQ terminal through a 1 kOhm resistor. The Arduino input connects directly to the KMP DAT terminal with a 10 kOhm pull-up resistor connected to the Arduino +5 VDC terminal. Arduino GND connects directly to the KMP GND terminal.
