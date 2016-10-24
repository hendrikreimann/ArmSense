/**************************************************/
void Normalize(void)
{
  float error=0;
  float temporary[3][3];
  float renorm=0;
  boolean problem=FALSE;
  
  error= -Vector_Dot_Product(&DCM_Matrix[0][0],&DCM_Matrix[1][0])*.5; //eq.19

  Vector_Scale(&temporary[0][0], &DCM_Matrix[1][0], error); //eq.19
  Vector_Scale(&temporary[1][0], &DCM_Matrix[0][0], error); //eq.19
  
  Vector_Add(&temporary[0][0], &temporary[0][0], &DCM_Matrix[0][0]);//eq.19
  Vector_Add(&temporary[1][0], &temporary[1][0], &DCM_Matrix[1][0]);//eq.19
  
  Vector_Cross_Product(&temporary[2][0],&temporary[0][0],&temporary[1][0]); // c= a x b //eq.20
  
  renorm= Vector_Dot_Product(&temporary[0][0],&temporary[0][0]); 
  if (renorm < 1.5625f && renorm > 0.64f) {
    renorm= .5 * (3-renorm);                                                 //eq.21
  } else if (renorm < 100.0f && renorm > 0.01f) {
    renorm= 1. / sqrt(renorm);  
    Serial.print("Square root called in renormalization");  
  } else {
    problem = TRUE;
    Serial.print("Problem detected!   Renorm 1 = ");
    Serial.println(renorm);
  }
      Vector_Scale(&DCM_Matrix[0][0], &temporary[0][0], renorm);
  
  renorm= Vector_Dot_Product(&temporary[1][0],&temporary[1][0]); 
  if (renorm < 1.5625f && renorm > 0.64f) {
    renorm= .5 * (3-renorm);                                                 //eq.21
  } else if (renorm < 100.0f && renorm > 0.01f) {
    renorm= 1. / sqrt(renorm);    
    Serial.print("Square root called in renormalization");
  } else {
    problem = TRUE;
    Serial.print("Problem detected!   Renorm 2 = ");
    Serial.println(renorm);
  }
  Vector_Scale(&DCM_Matrix[1][0], &temporary[1][0], renorm);
  
  renorm= Vector_Dot_Product(&temporary[2][0],&temporary[2][0]); 
  if (renorm < 1.5625f && renorm > 0.64f) {
    renorm= .5 * (3-renorm);                                                 //eq.21
  } else if (renorm < 100.0f && renorm > 0.01f) {
    renorm= 1. / sqrt(renorm);  
    Serial.print("Square root called in renormalization");  
  } else {
    problem = TRUE;
    Serial.print("Problem detected!   Renorm 3 = ");
    Serial.println(renorm);
  }
  Vector_Scale(&DCM_Matrix[2][0], &temporary[2][0], renorm);
  
  if (problem) {                // Our solution is blowing up and we will force back to initial condition.  Hope we are not upside down!
      DCM_Matrix[0][0]= 1.0f;
      DCM_Matrix[0][1]= 0.0f;
      DCM_Matrix[0][2]= 0.0f;
      DCM_Matrix[1][0]= 0.0f;
      DCM_Matrix[1][1]= 1.0f;
      DCM_Matrix[1][2]= 0.0f;
      DCM_Matrix[2][0]= 0.0f;
      DCM_Matrix[2][1]= 0.0f;
      DCM_Matrix[2][2]= 1.0f;
      problem = FALSE;  
  }
}

/**************************************************/
void Drift_correction(void)
{
  float mag_heading_x;
  float mag_heading_y;
  float errorCourse;
  //Compensation the Roll, Pitch and Yaw drift. 
  static float Scaled_Omega_P[3];
  static float Scaled_Omega_I[3];
  float Accel_magnitude;
  float Accel_weight;
  float Integrator_magnitude;
  float mag_projection;
  
  //*****Roll and Pitch***************

  // Calculate the magnitude of the accelerometer vector
  Accel_magnitude = sqrt(Accel_Vector[0]*Accel_Vector[0] + Accel_Vector[1]*Accel_Vector[1] + Accel_Vector[2]*Accel_Vector[2]);
  Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity.
  // Dynamic weighting of accelerometer info (reliability filter)
  // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
  Accel_weight = constrain(1 - 2*abs(1 - Accel_magnitude),0,1);  //  

  Vector_Cross_Product(&errorRollPitch[0],&Accel_Vector[0],&DCM_Matrix[2][0]); //adjust the ground of reference
  Vector_Scale(&Omega_P[0],&errorRollPitch[0],Kp_ROLLPITCH*Accel_weight);
  
  Vector_Scale(&Scaled_Omega_I[0],&errorRollPitch[0],Ki_ROLLPITCH*Accel_weight);
  Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);     
  
  //*****YAW***************
  // We make the gyro YAW drift correction based on compass magnetic heading
 
  mag_heading_x = cos(sen_data.magnetom_heading);
  mag_heading_y = sin(sen_data.magnetom_heading);
  errorCourse=(DCM_Matrix[0][0]*mag_heading_y) - (DCM_Matrix[1][0]*mag_heading_x);  //Calculating YAW error
  Vector_Scale(errorYaw,&DCM_Matrix[2][0],errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.
  
  Vector_Scale(&Scaled_Omega_P[0],&errorYaw[0],Kp_YAW);//.01proportional of YAW.
  Vector_Add(Omega_P,Omega_P,Scaled_Omega_P);//Adding  Proportional.
  
  Vector_Scale(&Scaled_Omega_I[0],&errorYaw[0],Ki_YAW);//.00001Integrator
  Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);//adding integrator to the Omega_I
  
  //  Here we will place a limit on the integrator so that the integrator cannot ever exceed half the saturation limit of the gyros
  Integrator_magnitude = sqrt(Vector_Dot_Product(Omega_I,Omega_I));
  if (Integrator_magnitude > ToRad(300)) {
    Vector_Scale(Omega_I,Omega_I,0.5f*ToRad(300)/Integrator_magnitude);
    Serial.print("Integrator being contrained from ");
    Serial.print(ToDeg(Integrator_magnitude));
    Serial.println(" degrees");
  }    
}

/**************************************************/
/*
void Accel_adjust(void)
{
 Accel_Vector[1] += Accel_Scale(speed_3d*Omega[2]);  // Centrifugal force on Acc_y = GPS_speed*GyroZ
 Accel_Vector[2] -= Accel_Scale(speed_3d*Omega[1]);  // Centrifugal force on Acc_z = GPS_speed*GyroY 
}
*/
/**************************************************/



void Matrix_update(void)
{
  Gyro_Vector[0]=ToRad(sen_data.gyro_x); //gyro x roll
  Gyro_Vector[1]=ToRad(sen_data.gyro_y); //gyro y pitch
  Gyro_Vector[2]=ToRad(sen_data.gyro_z); //gyro Z yaw
 
  Accel_Vector[0]=sen_data.accel_x;
  Accel_Vector[1]=sen_data.accel_y;
  Accel_Vector[2]=sen_data.accel_z;
  
  //Mag_Vector[0]=magnetom_x;  //This is the magnetometer direction vector as measured
  //Mag_Vector[1]=magnetom_y;
  //Mag_Vector[2]=magnetom_z;
  
  Vector_Add(&Omega[0], &Gyro_Vector[0], &Omega_I[0]);  //adding proportional term
  Vector_Add(&Omega_Vector[0], &Omega[0], &Omega_P[0]); //adding Integrator term

  //Accel_adjust();    //Remove centrifugal acceleration.   We are not using this function in this version - we have no speed measurement
  
 
 #define OUTPUTMODE 1 
 #if OUTPUTMODE==1         
  Update_Matrix[0][0]=0;
  Update_Matrix[0][1]=-G_Dt*Omega_Vector[2];//-z
  Update_Matrix[0][2]=G_Dt*Omega_Vector[1];//y
  Update_Matrix[1][0]=G_Dt*Omega_Vector[2];//z
  Update_Matrix[1][1]=0;
  Update_Matrix[1][2]=-G_Dt*Omega_Vector[0];//-x
  Update_Matrix[2][0]=-G_Dt*Omega_Vector[1];//-y
  Update_Matrix[2][1]=G_Dt*Omega_Vector[0];//x
  Update_Matrix[2][2]=0;
 #else                    // Uncorrected data (no drift correction)
  Update_Matrix[0][0]=0;
  Update_Matrix[0][1]=-G_Dt*Gyro_Vector[2];//-z
  Update_Matrix[0][2]=G_Dt*Gyro_Vector[1];//y
  Update_Matrix[1][0]=G_Dt*Gyro_Vector[2];//z
  Update_Matrix[1][1]=0;
  Update_Matrix[1][2]=-G_Dt*Gyro_Vector[0];
  Update_Matrix[2][0]=-G_Dt*Gyro_Vector[1];
  Update_Matrix[2][1]=G_Dt*Gyro_Vector[0];
  Update_Matrix[2][2]=0;
 #endif

  Matrix_Multiply(DCM_Matrix,Update_Matrix,Temporary_Matrix); //a*b=c

  for(int x=0; x<3; x++) //Matrix Addition (update)
  {
    for(int y=0; y<3; y++)
    {
      DCM_Matrix[x][y]+=Temporary_Matrix[x][y];
    } 
  }
}

void Euler_angles(void)
{
  #if (OUTPUTMODE==2)         // Only accelerometer info (debugging purposes)
    roll = atan2(Accel_Vector[1],Accel_Vector[2]);    // atan2(acc_y,acc_z)
    pitch = -asin((Accel_Vector[0])/(double)GRAVITY); // asin(acc_x)
    yaw = MAG_Heading;                     // Magnetic heading
  #else
    pitch = -asin(DCM_Matrix[2][0]);
    roll = atan2(DCM_Matrix[2][1],DCM_Matrix[2][2]);
    yaw = atan2(DCM_Matrix[1][0],DCM_Matrix[0][0]);  //  ***** Need to correct for magnetic variation
  #endif
}

void Inclination_angles(void)
// Combined accel and gyro
// NOT DEFINED: need a way to initialize gyro and combined trajectories -- first value of Accel_Vector;  normVector of what is dt???

{
  // get vertical estimate from accelerometers
  vertical_sensor_from_accel[0] = Accel_Vector[0];
  vertical_sensor_from_accel[1] = Accel_Vector[1];
  vertical_sensor_from_accel[2] = Accel_Vector[2];

  // normalize accelerometer estimate
  vertical_sensor_from_accel_norm = sqrt(vertical_sensor_from_accel[0]*vertical_sensor_from_accel[0] + vertical_sensor_from_accel[1]*vertical_sensor_from_accel[1] + vertical_sensor_from_accel[2]*vertical_sensor_from_accel[2]);
  vertical_sensor_from_accel[0] = vertical_sensor_from_accel[0] / vertical_sensor_from_accel_norm;
  vertical_sensor_from_accel[1] = vertical_sensor_from_accel[1] / vertical_sensor_from_accel_norm;
  vertical_sensor_from_accel[2] = vertical_sensor_from_accel[2] / vertical_sensor_from_accel_norm;
  
  // GYROSCOPE INCLINATION ANGLE
  angular_velocity_body_matrix[0][0] = 0;
  angular_velocity_body_matrix[0][1] = -sen_data.gyro_z;
  angular_velocity_body_matrix[0][2] = sen_data.gyro_y;
  angular_velocity_body_matrix[1][0] = sen_data.gyro_z;
  angular_velocity_body_matrix[1][1] = 0;
  angular_velocity_body_matrix[1][2] = -sen_data.gyro_x;
  angular_velocity_body_matrix[2][0] = -sen_data.gyro_y;
  angular_velocity_body_matrix[2][1] = sen_data.gyro_x;
  angular_velocity_body_matrix[2][2] = 0;

  // ------------------------------------------- this part is the old way of combining as a weighted sum of two estimates
  // rotational velocity based on gyroscope readings
//  vertical_sensor_dot[0] = -(angular_velocity_body_matrix[0][0]*vertical_sensor[0] + angular_velocity_body_matrix[0][1]*vertical_sensor[1] + angular_velocity_body_matrix[0][2]*vertical_sensor[2]);
//  vertical_sensor_dot[1] = -(angular_velocity_body_matrix[1][0]*vertical_sensor[0] + angular_velocity_body_matrix[1][1]*vertical_sensor[1] + angular_velocity_body_matrix[1][2]*vertical_sensor[2]);
//  vertical_sensor_dot[2] = -(angular_velocity_body_matrix[2][0]*vertical_sensor[0] + angular_velocity_body_matrix[2][1]*vertical_sensor[1] + angular_velocity_body_matrix[2][2]*vertical_sensor[2]);
//
//  // integrate rotational velocity
//  vertical_sensor_from_gyro[0] = vertical_sensor[0] + G_Dt * vertical_sensor_dot[0];
//  vertical_sensor_from_gyro[1] = vertical_sensor[1] + G_Dt * vertical_sensor_dot[1];
//  vertical_sensor_from_gyro[2] = vertical_sensor[2] + G_Dt * vertical_sensor_dot[2];
//
//  // normalize after integration
//  vertical_sensor_norm = sqrt(vertical_sensor_from_gyro[0]*vertical_sensor_from_gyro[0] + vertical_sensor_from_gyro[1]*vertical_sensor_from_gyro[1] + vertical_sensor_from_gyro[2]*vertical_sensor_from_gyro[2]);
//  vertical_sensor_from_gyro[0] = vertical_sensor_from_gyro[0] / vertical_sensor_norm;
//  vertical_sensor_from_gyro[1] = vertical_sensor_from_gyro[1] / vertical_sensor_norm;
//  vertical_sensor_from_gyro[2] = vertical_sensor_from_gyro[2] / vertical_sensor_norm;
//
//  // make relaxation step towards accelerometer estimate
//  vertical_sensor[0] = (gamma * vertical_sensor_from_gyro[0] + (1-gamma) * vertical_sensor_from_accel[0]);
//  vertical_sensor[1] = (gamma * vertical_sensor_from_gyro[1] + (1-gamma) * vertical_sensor_from_accel[1]);
//  vertical_sensor[2] = (gamma * vertical_sensor_from_gyro[2] + (1-gamma) * vertical_sensor_from_accel[2]);
//
//  // normalize after relaxation
//  vertical_sensor_norm = sqrt(vertical_sensor[0]*vertical_sensor[0] + vertical_sensor[1]*vertical_sensor[1] + vertical_sensor[2]*vertical_sensor[2]);
//  vertical_sensor[0] = vertical_sensor[0] / vertical_sensor_norm;
//  vertical_sensor[1] = vertical_sensor[1] / vertical_sensor_norm;
//  vertical_sensor[2] = vertical_sensor[2] / vertical_sensor_norm;
  // -------------------------------------------

  // ------------------------------------------- this part is the new way of combining, by defining the rate of change as a weighted sum of rotational velocity and error vs. accel estimate
  
  // rotational velocity based on gyroscope readings
  vertical_sensor_dot_from_gyro[0] = -(angular_velocity_body_matrix[0][0]*vertical_sensor[0] + angular_velocity_body_matrix[0][1]*vertical_sensor[1] + angular_velocity_body_matrix[0][2]*vertical_sensor[2]);
  vertical_sensor_dot_from_gyro[1] = -(angular_velocity_body_matrix[1][0]*vertical_sensor[0] + angular_velocity_body_matrix[1][1]*vertical_sensor[1] + angular_velocity_body_matrix[1][2]*vertical_sensor[2]);
  vertical_sensor_dot_from_gyro[2] = -(angular_velocity_body_matrix[2][0]*vertical_sensor[0] + angular_velocity_body_matrix[2][1]*vertical_sensor[1] + angular_velocity_body_matrix[2][2]*vertical_sensor[2]);

  // rotational velocity based on difference to accelerometer estimate
  vertical_sensor_dot_from_accel[0] = - alpha * (vertical_sensor[0] - vertical_sensor_from_accel[0]);
  vertical_sensor_dot_from_accel[1] = - alpha * (vertical_sensor[1] - vertical_sensor_from_accel[1]);
  vertical_sensor_dot_from_accel[2] = - alpha * (vertical_sensor[2] - vertical_sensor_from_accel[2]);

  // combine the two estimates
  vertical_sensor_dot[0] = vertical_sensor_dot_from_gyro[0] + vertical_sensor_dot_from_accel[0];
  vertical_sensor_dot[1] = vertical_sensor_dot_from_gyro[1] + vertical_sensor_dot_from_accel[1];
  vertical_sensor_dot[2] = vertical_sensor_dot_from_gyro[2] + vertical_sensor_dot_from_accel[2];

  // integrate rotational velocity
  vertical_sensor[0] = vertical_sensor[0] + G_Dt * vertical_sensor_dot[0];
  vertical_sensor[1] = vertical_sensor[1] + G_Dt * vertical_sensor_dot[1];
  vertical_sensor[2] = vertical_sensor[2] + G_Dt * vertical_sensor_dot[2];

  // normalize after integration
  vertical_sensor_norm = sqrt(vertical_sensor[0]*vertical_sensor[0] + vertical_sensor[1]*vertical_sensor[1] + vertical_sensor[2]*vertical_sensor[2]);
  vertical_sensor[0] = vertical_sensor[0] / vertical_sensor_norm;
  vertical_sensor[1] = vertical_sensor[1] / vertical_sensor_norm;
  vertical_sensor[2] = vertical_sensor[2] / vertical_sensor_norm;
  
  // -------------------------------------------
  

  // calculate inclination angles
  inclination_angle = acos(vertical_sensor[0]*sensor_axis_sensor[0] + vertical_sensor[1]*sensor_axis_sensor[1] + vertical_sensor[2]*sensor_axis_sensor[2]);
  inclination_angle_from_accel = acos(vertical_sensor_from_accel[0]*sensor_axis_sensor[0] + vertical_sensor_from_accel[1]*sensor_axis_sensor[1] + vertical_sensor_from_accel[2]*sensor_axis_sensor[2]);
  
  // convert to degrees
  inclination_angle = ToDeg(inclination_angle);
  inclination_angle_from_accel = ToDeg(inclination_angle_from_accel);

}
