

uint8_t raw_data[100], start_char, footer[3], data_availability;
uint16_t datalen, checkSum, checkSumCalc;

float buffer_float, ypr_data[3], sensor_data[9];
unsigned char *ptr_buffer = (unsigned char *)&buffer_float;

int redLedPin = 10;
int greenLedPin = 11;
int gndLedPin = 12;

long time1, time2, time3;

// CheckSum function: provided by SBG
uint16_t calcCRC(const void *pBuffer, uint16_t bufferSize)
{
  const uint8_t *pBytesArray =  (const uint8_t*)pBuffer;
  uint16_t poly = 0x8408;
  uint16_t crc = 0;
  uint8_t carry;
  uint8_t i_bits;
  uint16_t j;
  for (j =0; j < bufferSize; j++)
  {
    crc = crc ^ pBytesArray[j];
    for (i_bits = 0; i_bits < 8; i_bits++)
    {
      carry = crc & 1;
      crc = crc / 2;
      if (carry)
      {
        crc = crc^poly;
      } 
    }
  }
  return crc; 
}

void setup() {
  Serial.begin(115200);
  Serial.println("Start");
  pinMode(gndLedPin,OUTPUT);
  digitalWrite(gndLedPin, LOW);
  pinMode(redLedPin,OUTPUT);
  digitalWrite(redLedPin, LOW);
  pinMode(greenLedPin,OUTPUT);
  digitalWrite(greenLedPin, LOW);
  Serial.println("Corrupted Data");
  data_availability = 0;
}

void loop() {
  int i,j;
  
  if(Serial.available()>55)
  {
    start_char = Serial.read();
    //Serial.print(start_char); Serial.print(" ");
    if(start_char == 0xFF) // first character is OK, we can start reading the rest of package
    {
      analogWrite(greenLedPin, 120);
      time1 = micros();
      Serial.readBytes(raw_data,4);
      //Serial.print(raw_data[0]); Serial.print(" ");Serial.print(raw_data[1]); Serial.print(" ");Serial.print(raw_data[2]); Serial.print(" ");Serial.print(raw_data[3]); Serial.print(" ");
      if(raw_data[0] == 0x02) // second char is OK too
      {
        datalen = raw_data[3] + (raw_data[2]<<8); // Number of data to read
        //Serial.print(datalen); Serial.print(" ");
        Serial.readBytes(raw_data+4,datalen); // read the data with the length specified in the header

        Serial.readBytes(footer,3); // read the footer
        checkSum = footer[1] + ((uint16_t)footer[0]<<8);
        checkSumCalc = calcCRC(raw_data+1,datalen+3);
        if(footer[2]==0x03 && checkSum==checkSumCalc) // The end char is OK and the computed checksum correspond to the one in the footer.
        {
          analogWrite(redLedPin, 0);
          data_availability = 1;
          Serial.write(255);
          
          for(i=0;i<3;i++) // decode YPR data
          {
            for(j=0;j<4;j++)
            {
              Serial.write(raw_data[4*i+j+4]);
              ptr_buffer[j] = raw_data[4*i+j+4];
            }
            ypr_data[i] = buffer_float*57.2957795; // 180/pi
            //Serial.print(ypr_data[i]); Serial.print(" ");
          }
          for(i=0;i<9;i++) // decode sensor data, acc, gyr, mag
          {
            for(j=0;j<4;j++)
            {
              Serial.write(raw_data[4*i+j+16]);
              ptr_buffer[j] = raw_data[4*i+j+16];
            }
            sensor_data[i] = buffer_float;
            //Serial.print(sensor_data[i]); Serial.print(" ");
          }
          //Serial.println(" ");
        } else
        {
          Serial.println("Corrupted Data");
          data_availability = 0;
          analogWrite(redLedPin, 120);
        }
      }
    }
  } else if( (micros() - time1) >15000)
  {
    analogWrite(greenLedPin, 0);
    Serial.println("No Data");
  }
}
