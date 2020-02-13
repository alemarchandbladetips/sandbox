#include <SoftwareSerial.h>

#define R_TERRE 6366000.0

#define PACKET_START 0xAA // starting char of package
#define PACKET_STOP 0x55 // starting char of package

SoftwareSerial myserial(8,9);

// constant used to enable/disable communication, debug, timing checks
const int8_t transmit_raw = 1;
const int8_t print_data = 1;
const int8_t print_timing = 0;

uint8_t transmition_ready = 0;

extern float x_gps, y_gps, z_gps;
extern float vx_gps, vy_gps, vz_gps;

//Variables et paramètres du GPS //
uint16_t field_idx, idx_in_field;
String inString = "";    // string to hold input

float lat, lat_int_tmp, lat_dec_tmp, lat_0_int, lat_0_dec;
float lon, lon_int_tmp, lon_dec_tmp, lon_0_int, lon_0_dec;
float alt, alt_tmp, alt_0;
float GPS_dop, GPS_mode, GPS_mode_prev =0;
uint8_t GPS_mode_u8;

int8_t GPS_init_done = 0;

uint8_t gps_msg_speed[44]; uint8_t gps_msg_pos[44];
uint8_t gps_header[2];
float NED_coordinates[3], NED_coordinates_prev[3], NED_coordinates_offset[3];
float NED_speed[3];
uint8_t gps_flag, diffSoln, carrSoln, carrSoln_prev, relPosValid, gpsSanity = 0;
int8_t expected_size = 40;
uint8_t header_read =0, position_recieved;
uint32_t init_timer, init_done;

float x_gps=0, y_gps=0, z_gps=0;
float vx_gps=0, vy_gps=0, vz_gps=0;

int32_t int32_buffer;
unsigned char *ptr_int32_buffer = (unsigned char *)&int32_buffer;
uint32_t uint32_buffer;
unsigned char *ptr_uint32_buffer = (unsigned char *)&uint32_buffer;
int16_t buffer_int16;
unsigned char *ptr_buffer_int16 = (unsigned char *)&buffer_int16;
int32_t buffer_int32;
unsigned char *ptr_buffer_int32 = (unsigned char *)&buffer_int32;
float buffer_float;
unsigned char *ptr_buffer = (unsigned char *)&buffer_float;

static uint8_t x;
static uint8_t i, j;
/////////////////////////////////////////////////////////////////////

void setup() {
  // put your setup code here, to run once:
  Serial1.begin(57600);
  Serial.begin(57600);
  myserial.begin(57600);
  Serial.println("Start");
  init_timer = millis();
}

/////////////////////////////////////////////////////////////////////

void loop() {
       // idx = index of the character in the NMEA package, idx = 0 => Starting char of NMEA '$'
  // field_idx = index of the field, each field is separated by ',' field_idx = 0 => NMEA package type

  if (header_read == 0 && myserial.available() > 5)
  {
    x = myserial.read();

    //Serial.print("-");Serial.write(x);

    if (x == 0xB5) //181
    {
      x = myserial.read();

      if (x == 0x62) //98
      {
        x = myserial.read();
        x = myserial.read();
        expected_size = 40;
        header_read = 1;
      }
    }

    if(x == 0x24) // 0x24 = '$', starting char of NMEA packages
    {
      //Serial.println("\t");
      x = myserial.read();

      if (x == 0x47) // 0x47 = 'G' : GPS position
      {
        x = myserial.read();

        if (x == 0x4E) // 0x4E = 'N' : GPS position
        {
          x = myserial.read();
            if (x == 0x47) // 0x47 = 'G' : GPS position
            {
              x = myserial.read();
              if (x == 0x47) // 0x47 = 'G' : GPS position
            {
              x = myserial.read();
              if (x == 0x41) // 0x41 = 'A' : GPS position
              {
            
                expected_size = 40;
                header_read = 2;
                field_idx = 0;
                idx_in_field = 0;
              }
            }
            }
          }
        }
      }      
    }
  

  // GPS POSITION, NMEA package
  if(myserial.available() && header_read == 2)
  {
    x = myserial.read(); 
    //Serial.write( x );
///////////////////////////////////////////////////////////
//// Identification of package begining, type, field count

    idx_in_field++;

    // Finding field separators
    if(x == 0x2C) // 0x2C = ',' field separator of NMEA packages
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
      if(GPS_mode>1 || (millis()-init_timer)>30000.0 )
      {
        GPS_init_done = 1;
      }
      header_read = 0;
      transmition_ready = 1;

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
    }
  } // end of Serial available
 
///////////////////////////////////////////////////////////
//////// Speed package decoding
  if (header_read == 1 && myserial.available() > expected_size - 1  ) // 2=end of the header
  {
    header_read = 0;
    //Serial.print( "vitesse" );
    
    myserial.readBytes(gps_msg_speed, expected_size);
    
    for (i = 0; i < 3; i++)
    {
      for (j = 0; j < 4; j++)
      {
        ptr_int32_buffer[j] = gps_msg_speed[4 * i + j + 6];
      }
      NED_speed[i] = ((float)(int32_buffer)); //en centimètres/s
    }
  }

/////////////////////////////////////////////////////////
/////// Data transmition, to the control part of the drone

  if (transmition_ready)
  {
    if(transmit_raw){ myserial.write(PACKET_START); } // starting byte
    // NED_coordinates
    
    for(i=0;i<3;i++)
    {
      buffer_float = NED_coordinates[i]*100.0;
      if(print_data){ Serial.print(buffer_float); Serial.print(" ");}
      for (j = 0; j < 4; j++)
      {
        if (transmit_raw) { myserial.write(ptr_buffer[j]); }
      }
    }

    // NED_speed
    for(i=0;i<3;i++)
    {
      buffer_float = NED_speed[i];
      if(print_data){ Serial.print(buffer_float); Serial.print(" ");}
      for (j = 0; j < 4; j++)
      {
        if (transmit_raw) { myserial.write(ptr_buffer[j]); }
      }
    }

    // DOP
    buffer_float = GPS_dop*100;
    if(print_data){ Serial.print(buffer_float); Serial.print(" ");}
    for (j = 0; j < 4; j++)
    {
      if (transmit_raw) { myserial.write(ptr_buffer[j]); }
    }

    // mode
    GPS_mode_u8 = (uint8_t)GPS_mode;
    if(print_data){ Serial.print(GPS_mode_u8); Serial.print(" ");}
    if (transmit_raw) { myserial.write(GPS_mode_u8); }
    

    if(print_data){ Serial.println(" ");}

    if(transmit_raw){ myserial.write(PACKET_STOP); } // ending byte
    transmition_ready = 0;
  }
}
