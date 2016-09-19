/* ******************************************************* */
/* I2C code for ADXL345 accelerometer                      */
/*                                                         */
/* ******************************************************* */

//I2C addresses 
int AccelAddress = 0x53;     //Write:0xA6  Read:0xA7



//============================================
// Accelerometer
//============================================
void Init_Accel()
{
  Wire.beginTransmission(AccelAddress);
  Wire.write(0x2D);  // power register
  Wire.write(0x08);  // measurement mode
  Wire.endTransmission();
  delay(20);
  Wire.beginTransmission(AccelAddress);
  Wire.write(0x31);  // Data format register
  Wire.write(0x08);  // set to full resolution
  Wire.endTransmission();
  delay(20);	
  // Because our main loop runs at 50Hz we adjust the output data rate to 50Hz (25Hz bandwith)
  //Wire.beginTransmission(AccelAddress);
  //Wire.write(0x2C);  // Rate
  //Wire.write(0x09);  // set to 50Hz, normal operation
  //Wire.endTransmission();
  
}


// Reads x,y and z accelerometer registers
void Read_Accel()
{
  int i = 0;
  byte buff[6];
  
  Wire.beginTransmission(AccelAddress); 
  Wire.write(0x32);        //sends address to read from
  Wire.endTransmission(); //end transmission
  
  Wire.beginTransmission(AccelAddress); //start transmission to device
  Wire.requestFrom(AccelAddress, 6);    // request 6 bytes from device
  
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.read();  // receive one byte
    i++;
  }
  Wire.endTransmission(); //end transmission
  
  if (i==6)  // All bytes received?
    {   
      //this was too messy to do all at once so I broke it up into 3 steps
      
      //get the raw data
      sen_data.accel_x_raw = ( ((int)buff[1]) << 8) | buff[0]; // X axis
      sen_data.accel_y_raw = ( ((int)buff[3]) << 8) | buff[2]; // Y axis 
      sen_data.accel_z_raw = ( ((int)buff[5]) << 8) | buff[4]; // Z axis
            
      //scale and offset
      sen_data.accel_x = sen_data.accel_x_raw * sen_offset.accel_scale[0] - sen_offset.accel_offset[0];
      sen_data.accel_y = sen_data.accel_y_raw * sen_offset.accel_scale[1] - sen_offset.accel_offset[1];
      sen_data.accel_z = sen_data.accel_z_raw * sen_offset.accel_scale[2] - sen_offset.accel_offset[2];
      
      //change the sign if needed
      /*sen_data.accel_x *= 1* SENSOR_SIGN[3];
      sen_data.accel_y *= 1* SENSOR_SIGN[4];
      sen_data.accel_z *= 1* SENSOR_SIGN[5];
      */
      // What is this doing here??
      /*if(sen_data.accel_x<0){
        sen_data.accel_x *= 0.978;
      }
      sen_data.accel_x *= 0.9734;

      if(sen_data.accel_y<0){
        sen_data.accel_y *= 1.0;
      }
      sen_data.accel_y *= 0.967;

      if(sen_data.accel_z<0){
        sen_data.accel_z *= 1.045;
      }
      sen_data.accel_z *= 0.99;*/
    }
  else
    Serial.println("!ERR: Error reading accelerometer info!");

  
}

