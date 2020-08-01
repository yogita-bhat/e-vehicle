/*......This code involves loging parameters like li-ion battery power, energy, vehicle speed & inclination along with motor and ambient temperature............... */

#include <Wire.h>

 
// constants won't change. Used here to 
// set pin numbers:
double a0 = A0;    // select the input pin for the current sensor
double ledPin = 13;      // select the pin for the LED
double sensorValue = 0;  // variable to store the value coming from the sensor
double Vvalue=0;
//double a1 = A1;
//double a2 = A3;
//double currentsenspin = A2;
double a1 = A1; // select the input pin for the current sensor
double volts=0;
double currentvalue=0;
double Ivalue=0;double Vavg=0;double Iavg=0;
double startmillis=0;double minuite=60000;double reqsec=5000;
double endmillis=0;double hour=minuite*60;
double power=0;double energy5=0;double energy=0;
//import processing.serial.*;
double previousMillis = 0;        // will store last time LED was updated
double Vb,Ipv,Ib,It,IB,IPV,IT,VB,Vval,IBval,IPVval,Itval,Vbavg=0.0,Ipvavg=0.0,Ibavg=0.0,Itavg=0.0,Ppv,Pb,Pt;
int i;
 
// the follow variables is a long because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
double intervalsec = 5000;           // interval at which to blink (milliseconds)
 int x,y;int count=0;
int gyro_x, gyro_y, gyro_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
boolean set_gyro_angles;

long acc_x, acc_y, acc_z, acc_total_vector;
float angle_roll_acc, angle_pitch_acc;

float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
float angle_pitch_output, angle_roll_output;

long loop_timer;
int temp;
#include <OneWire.h>
#include <DallasTemperature.h>
OneWire ds18x20[] = {4, 5};
const int oneWireCount = sizeof(ds18x20)/sizeof(OneWire);
DallasTemperature sensor[oneWireCount];


float D = 30; // Diameter in cm (include at the top)
const int dataIN = 10; //IR sensor INPUT
unsigned long prevmillis; // To store time
unsigned long duration; // To store time difference
unsigned long refresh; // To store time for refresh of reading

int rpm; // RPM value

float speed;

boolean currentstate; // Current state of IR input scan
boolean prevstate; // State of IR sensor in previous scan


void setup() {
Wire.begin();                                                        //Start I2C as master
  setup_mpu_6050_registers();                                          //Setup the registers of the MPU-6050 
  for (int cal_int = 0; cal_int < 1000 ; cal_int ++){                  //Read the raw acc and gyro data from the MPU-6050 for 1000 times
    read_mpu_6050_data();                                             
    gyro_x_cal += gyro_x;                                              //Add the gyro x offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z offset to the gyro_z_cal variable
    delay(3);                                                          //Delay 3us to have 250Hz for-loop
  }

  // divide by 1000 to get avarage offset
  gyro_x_cal /= 1000;                                                 
  gyro_y_cal /= 1000;                                                 
  gyro_z_cal /= 1000;                                                 
  Serial.begin(115200);
  loop_timer = micros();       
  // pinMode(ledPin, OUTPUT); 
   //Serial.println("Dallas Temperature Sensors");
  //Serial.print("============Ready with ");
  //Serial.print(oneWireCount);
 // Serial.println(" Sensors================");
  
  // Start up the library on all defined bus-wires
  DeviceAddress deviceAddress;
  for (int i = 0; i < oneWireCount; i++){
    sensor[i].setOneWire(&ds18x20[i]);
    sensor[i].begin();
    if (sensor[i].getAddress(deviceAddress, 0)) sensor[i].setResolution(deviceAddress, 12);
  }
     pinMode(dataIN,INPUT);       
  prevmillis = 0;
 prevstate = LOW;  
  Serial.println("t     Temp1    Temp2    Vb     Ib     angle     rpm      speed");
  }
 

  void setup_mpu_6050_registers(){
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                             
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();                                             
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();                                             
}


void read_mpu_6050_data(){                                             //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  acc_x = Wire.read()<<8|Wire.read();                                  
  acc_y = Wire.read()<<8|Wire.read();                                  
  acc_z = Wire.read()<<8|Wire.read();                                  
  temp = Wire.read()<<8|Wire.read();                                   
  gyro_x = Wire.read()<<8|Wire.read();                                 
  gyro_y = Wire.read()<<8|Wire.read();                                 
  gyro_z = Wire.read()<<8|Wire.read();                                 
}
void loop()
{
  read_mpu_6050_data();   
 //Subtract the offset values from the raw gyro values
  gyro_x -= gyro_x_cal;                                                
  gyro_y -= gyro_y_cal;                                                
  gyro_z -= gyro_z_cal;                                                
         
  //Gyro angle calculations . Note 0.0000611 = 1 / (250Hz x 65.5)
  angle_pitch += gyro_x * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gyro_y * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel
  
  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       //Calculate the pitch angle
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       //Calculate the roll angle
  
  angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
  angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll

  if(set_gyro_angles){                                                 //If the IMU is already started
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else{                                                                //At first start
    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
    angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
    set_gyro_angles = true;                                            //Set the IMU started flag
  }
  
  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value
  //Serial.print(" | Angle  = "); Serial.println();

 while(micros() - loop_timer < 500);                                 //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop 
 loop_timer = micros();//Reset the loop timer






 // RPM Measurement
  currentstate = digitalRead(dataIN); // Read IR sensor state
 if( prevstate != currentstate) // If there is change in input
   {
     if( currentstate == HIGH ) // If input only changes from LOW to HIGH
       {
         duration = ( micros() - prevmillis ); // Time difference between revolution in microsecond
         rpm = (60000000/duration); // rpm = (1/ time millis)*1000*1000*60;
         prevmillis = micros(); // store time for next revolution calculation
       }
      
   }
  prevstate = currentstate; // store this scan (prev scan) data for next scan
  
 speed = 3.15159*(D/100000.0)*(rpm*60.0); // speed in km/hr
 
//speed = 3.14159*(D/100.0)*(rpm/60.0); //speed in m/s;

  
  
  /* {
      
       Serial.println(rpm);  
     Serial.println("speed");
    }*/

  // here is where you'd put code that needs to be running all the time.
  if (abs(millis()-endmillis)<500){
//    int count=0;Vavg=0;Iavg=0;
//    for (int x=0;x<100;x++){
    // save the last time you blinked the LED 
 
    // if the LED is off turn it on and vice-versa:
    Vb =analogRead(a0);
    //Serial.println(Vb);
    // turn the ledPin on
    //It=analogRead(a1);
    //Ipv=analogRead(a2);
    Ib=analogRead(a1);   
//  Serial.println(currentvalue);
//Ivalue=((currentvalue)*5.0/1024.0-1.466)*25.0;
   // Vvalue=((sensorValue)*5.0/1024.0-1.466)*82.0;
   // Vval=(Vb*5.0/1024.0-1.497)*122.0;
   // IBval=(Ib*5.0/1024.0-1.497)*17.0;
    Ivalue=((currentvalue)*5.0/1024.0-2.41)*40.0;
    Vvalue=((sensorValue)*5.0/1024.0-2.48)*114.0;
   // IPVval=Ipv*3.8*10.85/1024.0;
    //Itval=It*4.0*17.8/1024.0;
  //  volts=value*50;
    //Serial.println(Vval);
//    Serial.println(Ivalue);
    Vbavg=Vbavg+Vval;
    Ipvavg=Ipvavg+IPVval;
    Ibavg=Ibavg+IBval;
    Itavg=Itavg+Itval;
    count++;
   }
    //Serial.println(count);
//    Serial.println(count);
  if (abs(millis()-endmillis)>500){
//      Serial.println(abs(millis()-endmillis));
    //Serial.println(count);
    Vbavg=Vbavg/count;
    Ipvavg=Ipvavg/count;
    Ibavg=Ibavg/count;
    Itavg=Itavg/count;
    
    Ppv=Vbavg*Ipvavg;
    Pb=Vbavg*Ibavg;
    Pt=Vbavg*Itavg;
    double presentime=millis();
    double tim=abs(millis()-endmillis);
    //Serial.println(tim);

    double second=presentime/1000.0;
    Serial.print(second,4);
    Serial.print("\t");
    for (int i = 0; i < oneWireCount; i++) {
    sensor[i].requestTemperatures();
  } 
    Serial.print(sensor[0].getTempCByIndex(0),4);
    Serial.print("\t");
    Serial.print(sensor[1].getTempCByIndex(0),4);
    Serial.print("\t");
    Serial.print(Vbavg,4);
    Serial.print("\t");
    //Serial.print(Ipvavg,4);
    //Serial.print("\t");
    Serial.print(Ibavg,4);
    Serial.print("\t");
    //Serial.print(Itavg,4);
    //Serial.print("\t");
                             //Serial.print(Pb,4);
                             //Serial.print("\t");
    double energy5=Pt*tim/(3600.00*1000.0);
    energy5=energy5/1000.00;
    //Serial.print(energy5);
    energy=energy+energy5;
    //Serial.print("Energy: ");
    //Serial.print(energy,4);
    //Serial.print("\t");
    Serial.print(angle_pitch_output,4);
    Serial.print("\t");
    Serial.print(rpm);  
    Serial.print("\t");
    Serial.print(speed,4);
    Serial.println();
    endmillis=millis();
    count=0;Vbavg=0;Ipvavg=0;Ibavg=0;Itavg=0;energy5=0;
  }
  
//    float  milisec = millis(); // calculate time in milliseconds
//    Serial.print(milisec);
//    float tim=(milisec)/1000; // convert milliseconds to seconds
//    Serial.print("Vavg : ");
//    Serial.print("\t");
//delay(2000);

  }
