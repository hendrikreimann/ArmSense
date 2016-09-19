/* ******************************************************* */
/* I2C code for ITG-3200 Gyro                              */
/*                                                         */
/* ******************************************************* */

//I2C addresses 
int GyroAddress = 0x68;      //Write:0xD0  Read:0xD1



//============================================
// Gyro
//============================================
void Init_Gyro()
{
  //Set the sample rate divider
  Wire.beginTransmission(GyroAddress);
  Wire.write(0x15);  // Sample rate divider register
  Wire.write(9);  // Sample rate divider is 10 (register value + 1)
  Wire.endTransmission();
  delay(20);
  
  //Set the DLPF
  Wire.beginTransmission(GyroAddress);
  Wire.write(0x16);  // DLPF register
  Wire.write( (0x03<<3) | (0x03<<0) );  // Set the full-scale range to +/- 2000deg/sec, and the low pass filter to 20Hz
  Wire.endTransmission();
  delay(20);	
  
  //Setup the clock reference
  Wire.beginTransmission(GyroAddress);
  Wire.write(0x3E);  // Power and clock register
  Wire.write(1);  // Use the PLL with the X Gyro as the clock reference
  Wire.endTransmission();
  delay(20);	
  
  // Because our main loop runs at 50Hz we adjust the output data rate to 50Hz (25Hz bandwith)
  //Wire.beginTransmission(AccelAddress);
  //Wire.write(0x2C);  // Rate
  //Wire.write(0x09);  // set to 50Hz, normal operation
  //Wire.endTransmission();
}

// Reads the angular rates from the Gyro
void Read_Gyro()
{
  int i = 0;
  byte buff[8];  //6 bytes of angular rate data, and 2 bytes of temperature data
  
  Wire.beginTransmission(GyroAddress); 
  Wire.write(0x1B);        //The temperature and gyro data starts at address 0x1B
  Wire.endTransmission(); //end transmission
  
  Wire.beginTransmission(GyroAddress); //start transmission to device
  Wire.requestFrom(GyroAddress, 8);    // request 8 bytes from device
  
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.read();  // receive one byte
    i++;
  }
  Wire.endTransmission(); //end transmission
  
  if (i==8)  // All bytes received?
    {
      //get the raw data
      sen_data.gyro_x_raw = ((((int)buff[2]) << 8) | buff[3])/Gyro_Gain_X;    // X axis 
      sen_data.gyro_y_raw = ((((int)buff[4]) << 8) | buff[5])/Gyro_Gain_Y;    // Y axis 
      sen_data.gyro_z_raw = ((((int)buff[6]) << 8) | buff[7])/Gyro_Gain_Z;    // Z axis
      
      //subtract the offset
      sen_data.gyro_x = sen_data.gyro_x_raw - sen_offset.gyro_offset[0];    // X axis 
      sen_data.gyro_y = sen_data.gyro_y_raw - sen_offset.gyro_offset[1];    // Y axis 
      sen_data.gyro_z = sen_data.gyro_z_raw - sen_offset.gyro_offset[2];    // Z axis
      
      //change the sign if needed
      /*sen_data.gyro_x *= SENSOR_SIGN[0];    // X axis 
      sen_data.gyro_y *= SENSOR_SIGN[1];    // Y axis 
      sen_data.gyro_z *= SENSOR_SIGN[2];    // Z axis*/
           
      //To radians / s
      sen_data.gyro_x = ToRad(sen_data.gyro_x);
      sen_data.gyro_y = ToRad(sen_data.gyro_y);
      sen_data.gyro_z = ToRad(sen_data.gyro_z);
      
      //adjust any random offset
      /*if(abs(sen_data.gyro_x)<=0.021){
        sen_data.gyro_x = 0.0; 
      }
      
      if(abs(sen_data.gyro_y)<=0.021){
        sen_data.gyro_y = 0.0; 
      }
      
      if(abs(sen_data.gyro_z)<=0.021){
        sen_data.gyro_z = 0.0; 
      }*/
    }
  else
    Serial.println("!ERR: Error reading Gyro info!");
}

