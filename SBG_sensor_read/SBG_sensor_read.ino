

const int8_t transmit_raw = 0;
const int8_t print_data = 1;


uint8_t raw_data[100], start_char, footer[3], data_availability;
uint32_t device_status;
uint16_t datalen, checkSum, checkSumCalc;

float buffer_float, ypr_data[3], sensor_data[9];
unsigned char *ptr_buffer = (unsigned char *)&buffer_float;

uint32_t buffer_uint32;
unsigned char *ptr_buffer_uint32 = (unsigned char *)&buffer_uint32;

int redLedPin = 5;
int greenLedPin = 6;
int gndLedPin = 7;

int red_led_blink = 0;
int red_led_blink_counter = 0;
int red_led_blink_status = 0;

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

// Routine d'interruption
ISR(TIMER2_OVF_vect) {
  TCNT2 = 256 - 250; // 250 x 16 µS = 4 ms
  red_led_blink_counter++;
  if(red_led_blink_counter>20)
  {
    red_led_blink_counter = 0;
    if(red_led_blink)
    {
      if(red_led_blink_status)
      {
        analogWrite(redLedPin, 0);
        red_led_blink_status = 0;
      } else
      {
        analogWrite(redLedPin, 120);
        red_led_blink_status = 1;
      }
    }
  }
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
  
  cli(); // Désactive l'interruption globale
  bitClear (TCCR2A, WGM20); // WGM20 = 0
  bitClear (TCCR2A, WGM21); // WGM21 = 0 
  TCCR2B = 0b00001111; // 
  TIMSK2 = 0b00000001; // Interruption locale autorisée par TOIE2
  sei(); // Active l'interruption globale
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
        Serial.readBytes(raw_data+4,datalen); // read the data with the length specified in the header

        Serial.readBytes(footer,3); // read the footer
        checkSum = footer[1] + ((uint16_t)footer[0]<<8);
        checkSumCalc = calcCRC(raw_data+1,datalen+3);
        if(footer[2]==0x03 && checkSum==checkSumCalc && raw_data[52] == 0xFF && (raw_data[53] & 0x01) == 0x01) // The end char is OK and the computed checksum correspond to the one in the footer.
        {
          //analogWrite(redLedPin, 0);
          data_availability = 1;
          if(transmit_raw){ Serial.write(255); }
          
          for(i=0;i<3;i++) // decode YPR data
          {
            for(j=0;j<4;j++)
            {
              if(transmit_raw){ Serial.write(raw_data[4*i+j+4]); }
              ptr_buffer[j] = raw_data[4*i+j+4];
            }
            ypr_data[i] = buffer_float*57.2957795; // 180/pi
            //if(print_data){ Serial.print(ypr_data[i]); Serial.print(" "); }
          }
          if(print_data){ Serial.print(ypr_data[2]); Serial.print(" "); }
          for(i=0;i<9;i++) // decode sensor data, acc, gyr, mag
          {
            for(j=0;j<4;j++)
            {
              if(transmit_raw){ Serial.write(raw_data[4*i+j+16]); }
              ptr_buffer[j] = raw_data[4*i+j+16];
            }
            sensor_data[i] = buffer_float;
            //if(print_data){ Serial.print(sensor_data[i]); Serial.print(" "); }
          }
          //if(print_data){ Serial.print(raw_data[53]); Serial.print(" "); }
          if(transmit_raw){ Serial.write(raw_data[53]); }
          if((raw_data[53] & 0x6E) == 0x6E)
          {
            if(print_data){ Serial.print(2); Serial.print(" "); }
            analogWrite(redLedPin, 0);
            red_led_blink = 0;
          } else if ((raw_data[53] & 0x6E) == 0x2E)
          {
            if(print_data){ Serial.print(1); Serial.print(" "); }
            red_led_blink = 1;
          } else
          {
            if(print_data){ Serial.print(0); Serial.print(" "); }
            analogWrite(redLedPin, 120);
            red_led_blink = 0;
          }
          if(print_data){ Serial.println(" "); }
        } else
        {
          if(print_data){ Serial.println("Corrupted Data"); }
          if(transmit_raw){ Serial.write(0xAA);Serial.write(0xAA); }
          data_availability = 0;
          analogWrite(redLedPin, 120);
          analogWrite(greenLedPin, 0);
        }
      }
    }
  } else if( (micros() - time1) >15000)
  {
    analogWrite(greenLedPin, 0);
    analogWrite(redLedPin, 0);
    red_led_blink = 0;
    if(print_data){ Serial.println("No Data"); }
    if(transmit_raw){ Serial.write(0x55);Serial.write(0x55); }
  }
}
