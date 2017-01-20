
void printdata(void)
{    

      Serial.print(timer);
      Serial.print(", ");
      
 // #if outputMode==0
 if (outputMode == 0)
  {
        Serial.print(inclination_angle);
        Serial.print(", ");
        Serial.print(ProvideFeedback);
        Serial.print(", ");
        Serial.print(MotorStatus);
        Serial.print(", ");
        Serial.print(inclination_angle_from_accel);
        Serial.print(", ");
        Serial.print(threshold);
        Serial.print(", ");
        Serial.print(alpha);
  //#endif
  }
 //#if outputMode==1
 if (outputMode == 1)
  {    
        Serial.print(sen_data.accel_x);
        Serial.print (", ");
        Serial.print(sen_data.accel_y);
        Serial.print (", ");
        Serial.print(sen_data.accel_z);
        Serial.print(", ");
        Serial.print(sen_data.gyro_x);
        Serial.print(", ");
        Serial.print(sen_data.gyro_y);
        Serial.print(", ");
        Serial.print(sen_data.gyro_z);
 //#endif
  }
 //#if outputMode==2
if (outputMode == 2)
{
      Serial.print(sen_data.accel_x_raw);
      Serial.print (",");
      Serial.print(sen_data.accel_y_raw);
      Serial.print (",");
      Serial.print(sen_data.accel_z_raw);
      Serial.print(",");
      Serial.print(sen_data.gyro_x_raw);
      Serial.print(",");
      Serial.print(sen_data.gyro_y_raw);
      Serial.print(",");
      Serial.print(sen_data.gyro_z_raw);
 //#endif    
}   

if (outputMode == 3)
{
    Serial.print(sen_offset.accel_offset[0]);
    Serial.print (",");
    Serial.print(sen_offset.accel_offset[1]);
    Serial.print (",");
    Serial.print(sen_offset.accel_offset[2]);
    Serial.print (",");
    Serial.print(sen_offset.accel_scale[0]);
    Serial.print (",");
    Serial.print(sen_offset.accel_scale[1]);
    Serial.print (",");
    Serial.print(sen_offset.accel_scale[2]);
    Serial.print (",");
    Serial.print(sen_offset.gyro_offset[0]);
    Serial.print (",");
    Serial.print(sen_offset.gyro_offset[1]);
    Serial.print (",");
    Serial.print(sen_offset.gyro_offset[2]);     
    //Serial.print (",      ");

}

      //Serial.print(outputMode);
      
      Serial.println();    

}

long convert_to_dec(float x)
{
  return x*10000000;
}

