
#include <SPI.h>
#include <Ethernet.h>
#include "MQ135.h"
#include "DHT.h"
// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(169, 254, 196, 177);



#define         MQ_PIN                       (0)     //define which analog input channel you are going to use
#define         MQ_PIN135                    (1)
#define         MQ_PIN9                      (2)     //analog input channel


#define         RL_VALUE                     (10)     //define the load resistance on the board, in kilo ohms
#define         RO_CLEAN_AIR_FACTOR          (9.83)  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
                                                     //which is derived from the chart in datasheet
#define ANALOGPIN A1 
MQ135 gasSensor = MQ135(ANALOGPIN);

/***********************Software Related Macros************************************/
#define         CALIBARAION_SAMPLE_TIMES     (50)    //define how many samples you are going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL  (500)   //define the time interal(in milisecond) between each samples in the
                                                     //cablibration phase
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interal(in milisecond) between each samples in 
                                                     //normal operation

/**********************Application Related Macros**********************************/
#define         GAS_LPG                      (0)
#define         GAS_CO                       (1)
#define         GAS_SMOKE                    (2)
#define         GAS_ALCOHOL                  (3)
#define         GAS_CH4                      (5)

#define DHTPIN 2     // what pin we're connected to
#define DHTTYPE DHT11   // DHT 11 
DHT dht(DHTPIN, DHTTYPE);
/*****************************Globals***********************************************/
float           LPGCurve[3]  =  {2.3,0.21,-0.47};   //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent"
                                                    //to the original curve. 
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.21), point2: (lg10000, -0.59) 
float           COCurve[3]  =  {2.3,0.72,-0.34};    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.72), point2: (lg10000,  0.15) 
float           SmokeCurve[3] ={2.3,0.53,-0.44};    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.53), point2: (lg10000,  -0.22)                                                     
float           Ro           =  10;                 //Ro is initialized to 10 kilo ohms

float           AlcoholCurve[3] ={1.9,0.71,-0.28};  //two points,a line formed which is "approx equi" to the original curve. 
//float           H2Curve[3]  =  {2.1,0.33,-0.40};   //two points,a line formed which is "approx equi" to the original curve. 
float           CH4Curve[3]  =  {2.8,0.77,-0.46};    ///two points,a line formed which is "approx equi" to the original curve.  
int th=700; //this is the air quality threshold
int minth=300;
int maxth=700;

float lpg;
float co;
float smoke;
float quality;
float co2;
float ch4;
float alcohol;
float temperature;
float humidity;




// Initialize the Ethernet server library
// with the IP address and port you want to use
// (port 80 is default for HTTP):
EthernetServer server(80);

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  // start the Ethernet connection and the server:
  Ethernet.begin(mac, ip);
  server.begin();
  Serial.print("server is at ");
  Serial.println(Ethernet.localIP());

Serial.print("Calibrating...\n");                
  Ro = MQCalibration(MQ_PIN);                       //Calibrating the sensor. Please make sure the sensor is in clean air 
                                                    //when you perform the calibration                    
  Serial.print("Calibration is done...\n"); 
  Serial.print("Ro=");
  Serial.print(Ro);
  Serial.print("kohm");
  Serial.print("\n");

   float rzero = gasSensor.getRZero();
  delay(3000);
  Serial.print("MQ135 RZERO Calibration Value : ");
  Serial.println(rzero);

  dht.begin();  
}


void loop() {

  // listen for incoming clients
  EthernetClient client = server.available();
  if (client) {
    Serial.println("new client");
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // send a standard http response header
           int h = dht.readHumidity();
 int t = dht.readTemperature();

   Serial.print("LPG:"); 
   lpg=MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG);
   Serial.print(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG) );
   Serial.print( "ppm" );
   Serial.print("    ");   
   Serial.print("CO:"); 
   co=MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_CO);
   Serial.print(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_CO) );
   Serial.print( "ppm" );
   Serial.print("    ");   
   Serial.print("SMOKE:"); 
   smoke=MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_SMOKE);
   Serial.print(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_SMOKE) );
   Serial.print( "ppm" );
   Serial.print(" ");
    float ppm = gasSensor.getPPM();
    co2=ppm;
   //delay was here 
  digitalWrite(13,HIGH);
  Serial.print("CO2 ppm value : ");
  Serial.print(ppm);
  Serial.print( "ppm" );
 Serial.print("    ");   
Serial.print("Alcohol:");
alcohol= MQGetGasPercentage(MQRead(MQ_PIN135)/Ro,GAS_ALCOHOL);
   Serial.print(MQGetGasPercentage(MQRead(MQ_PIN135)/Ro,GAS_ALCOHOL) );
   Serial.print( "ppm" );
   Serial.print("    ");   
   Serial.print("CH4:"); 
   ch4=MQGetGasPercentage(MQRead(MQ_PIN135)/Ro,GAS_CH4);
   Serial.print(MQGetGasPercentage(MQRead(MQ_PIN135)/Ro,GAS_CH4) );
   Serial.print( "ppm" );
   Serial.print("    "); 
  int sensorValue = analogRead(A1);
 Serial.print("Air Quality = ");
  Serial.print(sensorValue);
   if(sensorValue>maxth)
   {
    Serial.println("You are breathing bad air");
    quality=3;
    }
    else if(sensorValue<minth)
    {
      Serial.println("You are breathing good air");
      quality=1;
      
    }
    else
    {
       quality=2;
       Serial.println("You air quality is normal. Beware of increasing pollution");
      }

  
   Serial.print("\n");
   Serial.print("Temperature:"); 
   temperature=t;
   Serial.print(t );
   Serial.print( "C" );
   Serial.print("    "); 
   Serial.print("Humidity:"); 
   humidity=h;
   Serial.print(h );
   Serial.print( "%" );
   Serial.print("    "); 
   Serial.print("\n");



          
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println("Connection: close");  // the connection will be closed after completion of the response
          client.println("Refresh: 5");  // refresh the page automatically every 5 sec
          client.println();
          client.println("<!DOCTYPE HTML>");
          client.println("<html>");
          client.println("<body>");
          client.println("MY SENSOR VALUES");
          client.println("<table border=");
          client.println("\'1\'");
          client.println("<cellspacing=");
          client.println("\"0\"");
          
            client.println("<cellspacing=");
            client.println("\"0\"");
            client.println(">");
          client.println("<tr>");
          client.println(" <td>LPG</td>");
          client.println("<td>CO</td>");
          client.println("<td>SMOKE</td>");
          client.println("<td>CO2</td>");
          client.println("<td>COMMENTS</td>");
          client.println("<td>CH4</td>");
          client.println("<td>ALCOHOL</td>");
          client.println("<td>TEMPERATURE</td>");
          client.println("<td>HUMIDITY</td>");
          client.println("</tr>");
          // output the value of each analog input pin
          for (int analogChannel = 0; analogChannel < 6; analogChannel++ ) {
            client.println("<tr>");
             client.println("<td>");
            client.print(lpg);
              client.println("</td>");
               client.println("<td>");
            client.print(co);
           client.println("</td>");
           client.println("<td>");
            client.print(smoke);
             client.println("</td>");
            client.println("<td>");
            client.print(co2);
             client.println("</td>");
            client.println("<td>");
            client.print(quality);
             client.println("</td>");
           client.println("<td>");
            client.print(ch4);
             client.println("</td>");
           client.println("<td>");
            client.print(alcohol);
             client.println("</td>");
           client.println("<td>");
            client.print(temperature);
             client.println("</td>");

            client.println("<td>");
            client.print(humidity);
            client.println("</td>");

             client.println("</tr>");
              delay(1000);
          }
          client.println("</table>");
           client.println("</body");
          client.println("</html>");
          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        } else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);
    // close the connection:
    client.stop();
    Serial.println("client disconnected");
    Ethernet.maintain();
  }
}

/****************** MQResistanceCalculation ****************************************
Input:   raw_adc - raw value read from adc, which represents the voltage
Output:  the calculated sensor resistance
Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage
         across the load resistor and its resistance, the resistance of the sensor
         could be derived.
************************************************************************************/ 
float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}

float MQ135ResistanceCalculation(int raw_adc135)
{
  return ( ((float)RL_VALUE*(1023-raw_adc135)/raw_adc135));
}

/***************************** MQCalibration ****************************************
Input:   mq_pin - analog channel
Output:  Ro of the sensor
Remarks: This function assumes that the sensor is in clean air. It use  
         MQResistanceCalculation to calculates the sensor resistance in clean air 
         and then divides it with RO_CLEAN_AIR_FACTOR. RO_CLEAN_AIR_FACTOR is about 
         10, which differs slightly between different sensors.
************************************************************************************/ 
float MQCalibration(int mq_pin)
{
  int i;
  float val=0;

  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {            //take multiple samples
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val/CALIBARAION_SAMPLE_TIMES;                   //calculate the average value

  val = val/RO_CLEAN_AIR_FACTOR;                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro 
                                                        //according to the chart in the datasheet 

  return val; 
}

float MQCalibration135(int mq_pin135)
{
  int i135;
  float val135=0;
 
  for (i135=0;i135<CALIBARAION_SAMPLE_TIMES;i135++) {            //take multiple samples
    val135 += MQ135ResistanceCalculation(analogRead(mq_pin135));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val135 = val135/CALIBARAION_SAMPLE_TIMES;                   //calculate the average value
 
  val135 = val135/RO_CLEAN_AIR_FACTOR;                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro 
                                                        //according to the chart in the datasheet 
 
  return val135; 
}
/*****************************  MQRead *********************************************
Input:   mq_pin - analog channel
Output:  Rs of the sensor
Remarks: This function use MQResistanceCalculation to caculate the sensor resistenc (Rs).
         The Rs changes as the sensor is in the different consentration of the target
         gas. The sample times and the time interval between samples could be configured
         by changing the definition of the macros.
************************************************************************************/ 
float MQRead(int mq_pin)
{
  int i;
  float rs=0;

  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }

  rs = rs/READ_SAMPLE_TIMES;

  return rs;  
}

float MQRead135(int mq_pin135)
{
  int i135;
  float rs135=0;
 
  for (i135=0;i135<READ_SAMPLE_TIMES;i135++) {
    rs135 += MQ135ResistanceCalculation(analogRead(mq_pin135));
    delay(READ_SAMPLE_INTERVAL);
  }
 
  rs135 = rs135/READ_SAMPLE_TIMES;
 
  return rs135;  
}

/*****************************  MQGetGasPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         gas_id      - target gas type
Output:  ppm of the target gas
Remarks: This function passes different curves to the MQGetPercentage function which 
         calculates the ppm (parts per million) of the target gas.
************************************************************************************/ 
int MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( gas_id == GAS_LPG ) {
     return MQGetPercentage(rs_ro_ratio,LPGCurve);
  } else if ( gas_id == GAS_CO ) {
     return MQGetPercentage(rs_ro_ratio,COCurve);
  } else if ( gas_id == GAS_SMOKE ) {
     return MQGetPercentage(rs_ro_ratio,SmokeCurve);
  }    

  return 0;
}

int MQ135GetGasPercentage(float rs_ro_ratio135, int gas_id135)
{
  if ( gas_id135 == GAS_ALCOHOL ) {
     return MQGetPercentage(rs_ro_ratio135,AlcoholCurve);
  } else if ( gas_id135 == GAS_CH4 ) {
     return MQGetPercentage(rs_ro_ratio135,CH4Curve);
  }
 
  return 0;
}

/*****************************  MQGetPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         pcurve      - pointer to the curve of the target gas
Output:  ppm of the target gas
Remarks: By using the slope and a point of the line. The x(logarithmic value of ppm) 
         of the line could be derived if y(rs_ro_ratio) is provided. As it is a 
         logarithmic coordinate, power of 10 is used to convert the result to non-logarithmic 
         value.
************************************************************************************/ 
int  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}

