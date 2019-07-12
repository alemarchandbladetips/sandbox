#define R_TERRE 6366000

uint16_t field_idx, idx_in_field;
String inString = "";    // string to hold input

uint8_t header_read = 0;

float lat, lat_int_tmp, lat_dec_tmp, lat_0_int, lat_0_dec;
float lon, lon_int_tmp, lon_dec_tmp, lon_0_int, lon_0_dec;
float alt, alt_tmp, alt_0;
float GPS_dop, GPS_mode, GPS_mode_prev =0;

int8_t GPS_init_done = 0;





// already defined variables


uint8_t gps_msg_speed[100];

float buffer_float;
int32_t int32_buffer;
unsigned char *ptr_int32_buffer = (unsigned char *)&int32_buffer;
int32_t expected_size;
uint16_t i,j;
float NED_coordinates[3], NED_coordinates_prev[3], NED_coordinates_offset[3], NED_speed[3];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  Serial1.begin(57600);
}

void loop() {
 uint8_t x;

  // idx = index of the character in the NMEA package, idx = 0 => Starting char of NMEA '$'
  // field_idx = index of the field, each field is separated by ',' field_idx = 0 => NMEA package type

  if (header_read == 0 && Serial1.available() > 1)
  {
    x = Serial1.read();

    if (x == 0xB5) //181
    {
      x = Serial1.read();

      if (x == 0x62) //98
      {
        expected_size = 40;
        header_read = 1;
      }
    }

    if(x == 0x24) // 0x24 = '$', starting char of NMEA packages
    {
      x = Serial1.read();

      if (x == 0x47) // 0x47 = 'G' : GPS position
      {
        expected_size = 40;
        header_read = 2;
        field_idx = 0;
        idx_in_field = 0;
      }      
    }
  }
  

  // GPS POSITION, NMEA package
  if(Serial1.available() && header_read == 2)
  {
    x = Serial1.read(); 

///////////////////////////////////////////////////////////
//// Identification of package begining, type, field count

    idx_in_field++;

    // Finding field separators
    if(x == 0x2C) // 0x24 = ',' field separator of NMEA packages
    {
      field_idx++;
      idx_in_field = 0; 
    }

///////////////////////////////////////////////////////////
//////// Position package decoding
    ////////////////////////////////////////
    // latitude 
    if(field_idx == 2)
    {

      // integer part of the latitude
      if(idx_in_field>=1 && idx_in_field<=2)
      {
        inString += (char)x;
      }
      if(idx_in_field==2)
      {
        lat_int_tmp = inString.toFloat();   
        inString = "";
      }

      // decimal part of latitude
      if(idx_in_field>=3 && idx_in_field<=10)
      {
        inString += (char)x;
        //Serial.println(inString);
      }
      if(idx_in_field==10)
      {
        lat_dec_tmp = inString.toFloat()/60.0;
        inString = "";
      }
    }
    // Sign of latitude : S (south) => negative value
    if(field_idx==3 && idx_in_field == 1)
    {
      if(x==0x53) // "S"
      {
        lat_int_tmp = -lat_int_tmp;
        lat_dec_tmp = -lat_dec_tmp;
      }
      if(GPS_init_done==0) // reference 
      {
        lat_0_int = lat_int_tmp;
        lat_0_dec = lat_dec_tmp;
      }
      lat = lat_int_tmp-lat_0_int;
      lat += (lat_dec_tmp-lat_0_dec);
      NED_coordinates[0] = lat*R_TERRE/180*PI;
    }

    ////////////////////////////////////////
    // longitude 
    if(field_idx == 4)
    {

      // integer part of the longitude
      if(idx_in_field>=1 && idx_in_field<=3)
      {
        inString += (char)x;
      }
      if(idx_in_field==3)
      {
        lon_int_tmp = inString.toFloat();   
        inString = "";
      }

      // decimal part of longitude
      if(idx_in_field>=4 && idx_in_field<=11)
      {
        inString += (char)x;
      }
      if(idx_in_field==11)
      {
        lon_dec_tmp = inString.toFloat()/60.0;
        inString = "";
      }
    }
    // Sign of longitude : W (west) => negative value
    if(field_idx==5 && idx_in_field == 1)
    {
      if(x==0x57) // "W"
      {
        lon_int_tmp = -lon_int_tmp;
        lon_dec_tmp = -lon_dec_tmp;
      }
      if(GPS_init_done==0) // reference 
      {
        lon_0_int = lon_int_tmp;
        lon_0_dec = lon_dec_tmp;
      }
      lon = lon_int_tmp-lon_0_int;
      lon += (lon_dec_tmp-lon_0_dec);
      NED_coordinates[1] = lon*R_TERRE/180*PI;
    }
 // } 

      ////////////////////////////////////////
    // mode
    if(field_idx == 6 && idx_in_field>=1)
    {
      inString += (char)x;
    }
    if(field_idx==7 && idx_in_field == 0)
    {
      GPS_mode = inString.toFloat();   
      inString = "";
    }

  ////////////////////////////////////////
    // DOP
    if(field_idx == 8 && idx_in_field>=1)
    {
      inString += (char)x;
    }
    if(field_idx==9 && idx_in_field == 0)
    {
      GPS_dop = inString.toFloat();   
      inString = "";
    }

    ////////////////////////////////////////
    // altitude
    if(field_idx == 9 && idx_in_field>=1)
    {
      inString += (char)x;
    }
    if(field_idx==10 && idx_in_field == 0)
    {
      alt_tmp = inString.toFloat();   
      inString = "";
      if(GPS_init_done==0) // reference 
      {
        alt_0 = alt_tmp;
      }
      NED_coordinates[2] = alt_tmp-alt_0;

    }

    if(field_idx==11 && idx_in_field == 0)
    {
      if(GPS_mode>1)
      {
        GPS_init_done = 1;
      }
      header_read = 0;

      if (GPS_mode!=GPS_mode_prev && GPS_mode_prev!=1)
      {
        for (i = 0; i < 3; i++)
        {
          NED_coordinates_offset[i] = NED_coordinates_prev[i] - NED_coordinates[i];
        }
      }

      GPS_mode_prev = GPS_mode;
      for (i = 0; i < 3; i++)
      {
        NED_coordinates[i] = NED_coordinates[i] + NED_coordinates_offset[i];
        NED_coordinates_prev[i] = NED_coordinates[i];
      }

      Serial.print(" ");Serial.print(GPS_mode);
      Serial.print(" ");Serial.print(NED_coordinates[0]);
      Serial.print(" ");Serial.print(NED_coordinates[1]);
      Serial.print(" ");Serial.print(-NED_coordinates[2]);
      Serial.println(" ");
//      x_gps = NED_coordinates[1];
//      y_gps = NED_coordinates[0];
//      z_gps = -NED_coordinates[2];
//      vx_gps = NED_speed[1];
//      vy_gps = NED_speed[0];
//      vz_gps = -NED_speed[2];
    }
  } // end of Serial available
 
///////////////////////////////////////////////////////////
//////// Speed package decoding
  if (header_read == 1 && Serial1.available() > expected_size - 1 + 2 ) // 2=end of the header
  {
    header_read = 0;
    Serial1.readBytes(gps_msg_speed, 2); // end of the header
    Serial1.readBytes(gps_msg_speed, expected_size);

    for (i = 0; i < 3; i++)
    {
      for (j = 0; j < 4; j++)
      {
        ptr_int32_buffer[j] = gps_msg_speed[4 * i + j + 6];
      }
      NED_speed[i] = ((float)(int32_buffer)); //en centimètres/s
      if(i<2)
      {
        Serial.print(" "); Serial.print(NED_speed[i]/100.0); 
      } else
      {
        Serial.print(" "); Serial.print(-NED_speed[i]/100.0); 
      }
    }
  }
}
