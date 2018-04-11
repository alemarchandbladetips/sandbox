#define DELAY_SAMPLE 35

float proper_acc_buffer[DELAY_SAMPLE*3];
int16_t proper_acc_buffer_index = 0;

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

void add_to_buffer3(float input[3],float *buffer_, int16_t *buffer_index, int16_t buffer_size)
{
  int16_t i;
  for(i=0;i<3;i++)
  {
    buffer_[3*(*buffer_index)+i] = input[i];
  }
  (*buffer_index)++;
  if(*buffer_index > buffer_size-1)
  {
    *buffer_index = 0;
  }
}

//////////////////////////////////////////////////////////////////////

int8_t get_in_buffer3(float *buffer_, int16_t *buffer_index, int16_t buffer_size, int16_t data_index, float output[3])
{
  int16_t index_in_buffer, i;

  if(data_index>buffer_size-1)
  {
    return -1;
  }

  index_in_buffer = data_index+*buffer_index;
  if(index_in_buffer > buffer_size-1)
  {
    index_in_buffer -= buffer_size;
  }
  
  for(i=0;i<3;i++)
  {
    output[i] = buffer_[3*index_in_buffer+i];
  }
  return 0;
}

//////////////////////////////////////////////////////////////////////

void clear_buffer3(float *buffer_, int16_t *buffer_index, int16_t buffer_size)
{
  int16_t i,j;
  for(i=0;i<buffer_size;i++)
  {
    for(j=0;j<3;j++)
    {
      buffer_[3*i+j] = 0;
    }
  }
  *buffer_index = 0;
}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  clear_buffer3(proper_acc_buffer, &proper_acc_buffer_index, DELAY_SAMPLE);
}

void loop() {
  int8_t i, ret;
  float input_vector[3];

  for(i=1;i<100;i++)
  {
    input_vector[0] = i*1.0;
    input_vector[1] = i*1.01;
    input_vector[2] = i*1.02;
    add_to_buffer3(input_vector,proper_acc_buffer, &proper_acc_buffer_index, DELAY_SAMPLE);
  }

  Serial.print(proper_acc_buffer_index); Serial.println(" ");

  for(i=0;i<40;i++)
  {
    ret = get_in_buffer3(proper_acc_buffer, &proper_acc_buffer_index, DELAY_SAMPLE, i, input_vector);
    if(ret==0)
    {
      Serial.print(i); Serial.print(" ");
      Serial.print(ret); Serial.print(" ");
      Serial.print(input_vector[0]); Serial.print(" ");
      Serial.print(input_vector[1]); Serial.print(" ");
      Serial.print(input_vector[2]); Serial.println(" ");
    } else
    {
      Serial.print(i); Serial.print(" ");
      Serial.print(ret); Serial.println(" ");
    }
  }

  delay(60000);

}
