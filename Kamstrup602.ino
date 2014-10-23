#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <SoftwareSerial.h>                    // Using a modified version of SoftwareSerial where the transmitter is inverted
#include <Time.h> 

// Wifi
char ssid[] = "Test";                          // Network SSID (name) 
char pass[] = "Pwd1234!";                      // Network password
int status = WL_IDLE_STATUS;                   // Status of the Wifi radio
unsigned long lastConnectionTime = 0;          // Last time a connection was made to the server, in milliseconds
boolean lastConnected = false;                 // Dtate of the connection last time through the main loop
const unsigned long postingInterval = 1000;    // Delay between updates, in milliseconds
IPAddress server(192,168,10,26);               // IP address of the Synology server
WiFiClient client;

// NTP Servers:
IPAddress timeServer(129, 6, 15, 28); // timeserver

const int timeZone = 2;     // Central European Time
//const int timeZone = -5;  // Eastern Standard Time (USA)
//const int timeZone = -4;  // Eastern Daylight Time (USA)
//const int timeZone = -8;  // Pacific Standard Time (USA)
//const int timeZone = -7;  // Pacific Daylight Time (USA)

WiFiUDP Udp;
unsigned int localPort = 2390;  // local port to listen for UDP packets

// Kamstrup Multical 602
word const kregnums[] = { 0x003C,0x0050,0x0056,0x0057,0x0059,0x004a,0x004b,0x0044,0x0045 };
char* kregstrings[] = { "Energy","Cur_Power","Temp_T1","Temp_T2","Diff_Temp","Flow_1","Flow_2","Vol_1","Vol_2" };
#define NUMREGS 9                              // Number of registers that are to be read from the Kamstrup
#define KAMBAUD 1200                           // Baud rate of the Kamstrup KMP Data connection

// Database
int gkreg = 1;
time_t t;

// Sorting of the registers accoring to type (power, temperature, flow, volume etc.
char* kdbtype[] = { "FV_Energy","FV_Power","FV_Temp","FV_Temp","FV_Temp","FV_Flow","FV_Flow","FV_Vol","FV_Vol" };

// Units
char*  units[65] = {"","Wh","kWh","MWh","GWh","j","kj","Mj",
	"Gj","Cal","kCal","Mcal","Gcal","varh","kvarh","Mvarh","Gvarh",
        "VAh","kVAh","MVAh","GVAh","kW","kW","MW","GW","kvar","kvar","Mvar",
        "Gvar","VA","kVA","MVA","GVA","V","A","kV","kA","C","K","l","m3",
        "l/h","m3/h","m3xC","ton","ton/h","h","hh:mm:ss","yy:mm:dd","yyyy:mm:dd",
        "mm:dd","","bar","RTC","ASCII","m3 x 10","ton xr 10","GJ x 10","minutes","Bitfield",
        "s","ms","days","RTC-Q","Datetime"};

// Arduino Uno pin allocations
#define PIN_KAMSER_RX  5   // Kamstrup Data DAT/62 (Yellow wire)
#define PIN_KAMSER_TX  6   // Kamstrup Data REQ/63 (Green wire)
#define PIN_LED        13  // Standard Arduino LED

// Kamstrup KMP serial
#define KAMTIMEOUT 300     // Kamstrup timeout after transmit
SoftwareSerial kamSer(PIN_KAMSER_RX, PIN_KAMSER_TX, true);  // Initialize serial

/******************************/
/**                          **/
/**          SETUP           **/
/**                          **/
/******************************/
void setup () {
  // Start the serial port (Console)
  Serial.begin(9600);
  
  // Just for debug
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, 0);
  
  // Setup for Kamstrup serial connection
  pinMode(PIN_KAMSER_RX,INPUT);
  pinMode(PIN_KAMSER_TX,OUTPUT);
  kamSer.begin(KAMBAUD);

  // Give the Wifi module time to boot 
  delay(1000);
  
  // Attempt to connect using WPA2 encryption:
  Serial.println("Attempting to connect to WPA network...");
  status = WiFi.begin(ssid, pass);
  
  // if not connected, just stop here
  if ( status != WL_CONNECTED) { 
    Serial.println("Could not establish Wifi connection");
    while(true);
  } 
  
  // If connected, print out info about the connection:
  else {
    Serial.println("Connection to network established");
    printWifiStatus();
  }
  
  // Sync time to NTP server
  Udp.begin(localPort);
  Serial.println("waiting for sync");
  setSyncProvider(getNtpTime);
}

/******************************/
/**                          **/
/**          LOOP            **/
/**                          **/
/******************************/
void loop () {
  float fValue = 0;  // The value of the last read register
  int digits;
  
  // If we are about to read the first register, then register the time
  if (gkreg == 1) {
    t = now();    
  }
  
  // Flush and stop to prevent socket issues
  // client.flush();
  // client.stop();  

  // if there's no net connection, but there was one last time through the loop, stop the client
 // if (!client.connected() && lastConnected) { 
 //    Serial.println("Disconnecting");
 //    client.stop();
 // }
  
  // Check if it is time to do a Poll
  if(!client.connected() && (millis() - lastConnectionTime > postingInterval)) {

    client.stop();
    
    // Poll the Kamstrup register for data 
    Serial.println("");
    Serial.println("Reading Multical 602");
    fValue = kamReadReg(gkreg);
    
    // Adjust Vol with offset value
    if (gkreg == 7) {
      fValue = fValue - 23.83;
    }  
    
    // Attempt to connect with the server
    Serial.println("Attempting to connect with server");
    if (client.connect(server, 80)) {
      Serial.println("Connected!");
      Serial.println("Send data to server");

      // Write register to server using HTTP PUT
      client.print("PUT /data_post1.php?type=");
      client.print(kdbtype[gkreg]);
      client.print("&name=");
      client.print(kregstrings[gkreg]);
      client.print("&value=");
      client.print(fValue);
      client.print("&id=");
      client.print(gkreg+300); // Avoid that ID collides with Z-Wave devices
      client.print("&time=");
      client.print(year(t)); 
      client.print("-");
      client.print(month(t));
      client.print("-");
      client.print(day(t));
      client.print("-");
      digits = hour(t);
      if (digits < 10) {
        client.print("0"); 
      }
      client.print(digits);
      client.print(":");
      digits = minute(t);
      if (digits < 10) {
        client.print("0"); 
      }
      client.print(digits);
      client.print(":");
      digits = second(t);
      if (digits < 10) {
        client.print("0");
      }
      client.println(digits);
      
      
      // Note the time that the connection was made
      lastConnectionTime = millis();

      // Serial output - just for debug
      Serial.print("/data_post1.php?type=");
      Serial.print(kdbtype[gkreg]);
      Serial.print("&name=");
      Serial.print(kregstrings[gkreg]);
      Serial.print("&value=");
      Serial.print(fValue);
      Serial.print("&id=");
      Serial.print(gkreg+300); // Avoid that ID collides with Z-Wave devices
      Serial.print("&time=");
      Serial.print(year(t)); 
      Serial.print("-");
      Serial.print(month(t));
      Serial.print("-");
      Serial.print(day(t));
      Serial.print("-");
      digits = hour(t);
      if (digits < 10) {
        Serial.print("0"); 
      }
      Serial.print(digits);
      Serial.print(":");
      digits = minute(t);
      if (digits < 10) {
        Serial.print("0"); 
      }
      Serial.print(digits);
      Serial.print(":");
      digits = second(t);
      if (digits < 10) {
        Serial.print("0");
      }
      Serial.println(digits);
           
      Serial.println("Done sending data");
     
      // Next register
      gkreg++;
      
      // Only read temperatures
      if (gkreg > NUMREGS-1) {
        gkreg = 1;
        // Wait 5 minutes between each series of readings
        delay(300000);
      }
    }
    else {
      // Couldn't make a connection
      Serial.println("Connection failed");
      client.stop();
    }
  }
  // Store the state of the connection for next time through the loop
  //lastConnected = client.connected();
 
  // Flash the LED pin - just for debug
  digitalWrite(PIN_LED, digitalRead(PIN_KAMSER_RX));
}

/******************************/
/**                          **/
/**       KamReadReg         **/
/**                          **/
/******************************/
// kamReadReg - read a Kamstrup register
float kamReadReg(unsigned short kreg) {

  byte recvmsg[40];  // buffer of bytes to hold the received data
  float rval;        // this will hold the final value

  // Prepare the message frame and send it to the Kamstrup
  byte sendmsg[] = { 0x3f, 0x10, 0x01, (kregnums[kreg] >> 8), (kregnums[kreg] & 0xff) };
  kamSend(sendmsg, 5);

  // Listen for an answer
  unsigned short rxnum = kamReceive(recvmsg);

  // Check if number of received bytes > 0 
  if(rxnum != 0){
    
    // Decode the received message
    rval = kamDecode(kreg, recvmsg);
    
    // Print out received value to terminal - just for debug
    Serial.print(kregstrings[kreg]);
    Serial.print(": ");
    Serial.print(rval);
    Serial.print(" ");
    Serial.println();
    
    return rval;
  }
}

/******************************/
/**                          **/
/**         KamSend          **/
/**                          **/
/******************************/
// kamSend - send data to Kamstrup meter
void kamSend(byte const *msg, int msgsize) {

  // Append checksum bytes to message
  byte newmsg[msgsize+2];
  for (int i = 0; i < msgsize; i++) { newmsg[i] = msg[i]; }
  newmsg[msgsize++] = 0x00;
  newmsg[msgsize++] = 0x00;
  int c = crc_1021(newmsg, msgsize);
  newmsg[msgsize-2] = (c >> 8);
  newmsg[msgsize-1] = c & 0xff;

  // Build the final transmit message - escape various bytes
  byte txmsg[20] = { 0x80 };   // Prefix
  int txsize = 1;
  for (int i = 0; i < msgsize; i++) {
    if (newmsg[i] == 0x06 or newmsg[i] == 0x0d or newmsg[i] == 0x1b or newmsg[i] == 0x40 or newmsg[i] == 0x80) {
      txmsg[txsize++] = 0x1b;
      txmsg[txsize++] = newmsg[i] ^ 0xff;
    } else {
      txmsg[txsize++] = newmsg[i];
    }
  }
  txmsg[txsize++] = 0x0d;  // EOF

  // send to serial interface
  for (int x = 0; x < txsize; x++) {
    kamSer.write(txmsg[x]);
  }

}

/******************************/
/**                          **/
/**       KamReceive         **/
/**                          **/
/******************************/
// kamReceive - receive bytes from Kamstrup meter
unsigned short kamReceive(byte recvmsg[]) {

  byte rxdata[50];  // Buffer to hold received data
  unsigned long rxindex = 0;
  unsigned long starttime = millis();
  
  kamSer.flush();  // Flush serial buffer - might contain noise

  byte r;
  
  // Loop until EOL received or timeout
  while(r != 0x0d){
    
    // Handle rx timeout
    if(millis()-starttime > KAMTIMEOUT) {
      Serial.println("Timed out listening for data");
      return 0;
    }

    // Handle incoming data
    if (kamSer.available()) {

      // Receive byte
      r = kamSer.read();
      if(r != 0x40) {  // Don't append if it's the start marker
        
        // Append data
        rxdata[rxindex] = r;
        rxindex++; 
      }

    }
  }

  // Remove escape markers from received data
  unsigned short j = 0;
  for (unsigned short i = 0; i < rxindex -1; i++) {
    if (rxdata[i] == 0x1b) {
      byte v = rxdata[i+1] ^ 0xff;
      if (v != 0x06 and v != 0x0d and v != 0x1b and v != 0x40 and v != 0x80){
        Serial.print("Missing escape ");
        Serial.println(v, HEX);
      }
      recvmsg[j] = v;
      i++; // skip
    } else {
      recvmsg[j] = rxdata[i];
    }
    j++;
  }
  
  // Check CRC
  if (crc_1021(recvmsg,j)) {
    Serial.println("CRC error: ");
    return 0;
  }
  
  return j;
  
}

/******************************/
/**                          **/
/**       KamDecode          **/
/**                          **/
/******************************/
// kamDecode - decodes received data
float kamDecode(unsigned short const kreg, byte const *msg) {

  // Skip if message is not valid
  if (msg[0] != 0x3f or msg[1] != 0x10) {
    return false;
  }
  if (msg[2] != (kregnums[kreg] >> 8) or msg[3] != (kregnums[kreg] & 0xff)) {
    return false;
  }
    
  // Decode the significant
  long x = 0;
  for (int i = 0; i < msg[5]; i++) {
    x <<= 8;
    x |= msg[i + 7];
  }
  
  // Decode the exponent
  int i = msg[6] & 0x3f;
  if (msg[6] & 0x40) {
    i = -i;
  };
  float ifl = pow(10,i);
  if (msg[6] & 0x80) {
    ifl = -ifl;
  }
  
  // Return final value
  return (float )(x * ifl);

}

/******************************/
/**                          **/
/**       crc_1021           **/
/**                          **/
/******************************/
// crc_1021 - calculate crc16
long crc_1021(byte const *inmsg, unsigned int len){
  long creg = 0x0000;
  for(unsigned int i = 0; i < len; i++) {
    int mask = 0x80;
    while(mask > 0) {
      creg <<= 1;
      if (inmsg[i] & mask){
        creg |= 1;
      }
      mask>>=1;
      if (creg & 0x10000) {
        creg &= 0xffff;
        creg ^= 0x1021;
      }
    }
  }
  return creg;
}

/******************************/
/**                          **/
/**     printWifiStatus      **/
/**                          **/
/******************************/
void printWifiStatus() {
   // Print the SSID of the network
   Serial.print("SSID: ");
   Serial.println(WiFi.SSID());

   // Print the WiFi shield's IP address
   IPAddress ip = WiFi.localIP();
   Serial.print("IP Address: ");
   Serial.println(ip);

   // Print the received signal strength
   long rssi = WiFi.RSSI();
   Serial.print("signal strength (RSSI):");
   Serial.print(rssi);
   Serial.println(" dBm");
}

/*-------- NTP code ----------*/

const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

/******************************/
/**                          **/
/**       getNtpTime()       **/
/**                          **/
/******************************/
time_t getNtpTime()
{
  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  sendNTPpacket(timeServer);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

/******************************/
/**                          **/
/**      sendNTPpacket       **/
/**                          **/
/******************************/
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:                 
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}
