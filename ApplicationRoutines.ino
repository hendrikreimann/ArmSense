

//=================================
// Fill in the calibration values
// this could be expanded to allow the values to determined 
// using a calibration mode for example
void mongooseCalibrate(void)
{
 
    // See the calibration guide for more details on what each 
    // of these are for. These values are unique for each Mongoose
    // Magnetometer calibration values can also be dependent on
    // the sensor platform
    //AHRS for Uppsala  
    /*  
    sen_offset.accel_offset[0]     = 12.0; 
    sen_offset.accel_offset[1]     = -3.5; 
    sen_offset.accel_offset[2]     = 18.0;
    
    sen_offset.accel_scale[0]     = 1.0; 
    sen_offset.accel_scale[1]     = 1.0;
    sen_offset.accel_scale[2]     = 1.0;
    
    sen_offset.gyro_offset[0]      = 1,7;
    sen_offset.gyro_offset[1]      = 0.0;
    sen_offset.gyro_offset[2]      = 0.0;
    
    sen_offset.magnetom_offset[0]  = 90.0;
    sen_offset.magnetom_offset[1]  = -140.0;
    sen_offset.magnetom_offset[2]  = -35.0;
    
    sen_offset.magnetom_XY_Theta   = ToRad(0);
    sen_offset.magnetom_XY_Scale   = 1.0;
    
    sen_offset.magnetom_YZ_Theta   = ToRad(0);
    sen_offset.magnetom_YZ_Scale   = 1.0;
    */
    
    //AHRS for Malmö                         
    // Original Values
   /* sen_offset.accel_offset[0]     =  0; 
    sen_offset.accel_offset[1]     =  7.5; 
    sen_offset.accel_offset[2]     =  10;
    
    sen_offset.accel_scale[0]     =   0.973; 
    sen_offset.accel_scale[1]     =   0.96;
    sen_offset.accel_scale[2]     =   1.028;
    
    sen_offset.gyro_offset[0]      =  -6.8;
    sen_offset.gyro_offset[1]      =  0.23;
    sen_offset.gyro_offset[2]      =  0;
    
    sen_offset.magnetom_offset[0]  = 50;
    sen_offset.magnetom_offset[1]  = -200;
    sen_offset.magnetom_offset[2]  = 0;
    
    sen_offset.magnetom_XY_Theta   = ToRad(0);
    sen_offset.magnetom_XY_Scale   = 1.0;
    
    sen_offset.magnetom_YZ_Theta   = ToRad(0);
    sen_offset.magnetom_YZ_Scale   = 1.0;*/

    // New Red Sensor Values
    # if SENSOR_COLOR == 1
    sen_offset.accel_offset[0]     = 8.939; 
    sen_offset.accel_offset[1]     =  .729; 
    sen_offset.accel_offset[2]     = 15.054;
    
    sen_offset.accel_scale[0]     =  .9883; 
    sen_offset.accel_scale[1]     =  .9885;
    sen_offset.accel_scale[2]     =  1.02;
    
    sen_offset.gyro_offset[0]      =  -2;
    sen_offset.gyro_offset[1]      =  4;
    sen_offset.gyro_offset[2]      =  -1;
    
    sen_offset.magnetom_offset[0]  = 50;
    sen_offset.magnetom_offset[1]  = -200;
    sen_offset.magnetom_offset[2]  = 0;
    
    sen_offset.magnetom_XY_Theta   = ToRad(0);
    sen_offset.magnetom_XY_Scale   = 1.0;
    
    sen_offset.magnetom_YZ_Theta   = ToRad(0);
    sen_offset.magnetom_YZ_Scale   = 1.0;
    #endif
    
    // New White Sensor Values
    # if SENSOR_COLOR == 0
    sen_offset.accel_offset[0]     =  3.428; 
    sen_offset.accel_offset[1]     =  -1.4943; 
    sen_offset.accel_offset[2]     = 1.8718;
    
    sen_offset.accel_scale[0]     =  .9698; 
    sen_offset.accel_scale[1]     =  .9821;
    sen_offset.accel_scale[2]     =  1.0065;
    
    sen_offset.gyro_offset[0]      =  0;
    sen_offset.gyro_offset[1]      =  1;
    sen_offset.gyro_offset[2]      =  0;
    
    sen_offset.magnetom_offset[0]  = 50;
    sen_offset.magnetom_offset[1]  = -200;
    sen_offset.magnetom_offset[2]  = 0;
    
    sen_offset.magnetom_XY_Theta   = ToRad(0);
    sen_offset.magnetom_XY_Scale   = 1.0;
    
    sen_offset.magnetom_YZ_Theta   = ToRad(0);
    sen_offset.magnetom_YZ_Scale   = 1.0;
    #endif
} 

void StatusLEDToggle()
{
  static unsigned int counter = 0;
  static char state = 0;
  
  counter++;
  if (counter > 20)
  {
    counter=0;
    if(state)
    {
      digitalWrite(STATUS_LED,LOW);
      state = FALSE;
    }
    else
    {
      digitalWrite(STATUS_LED,HIGH);
      state = TRUE;
    }
  }
}



//EEPROM writing and reading Data structure
int EEPROM_WriteByteArray(int ee, int count, byte * p)
{
  int i = 0;
        
  for(i=0;i<count;i++)
    EEPROM.write(ee++, *p++);
          
  return(i);
}
    
int EEPROM_ReadByteArray(int ee, int count, byte * p)
{
  int i = 0;
        
  for(i=0;i<count;i++)
    *p++ = EEPROM.read(ee++);
          
  return(i);
}






