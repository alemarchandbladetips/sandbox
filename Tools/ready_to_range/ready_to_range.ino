
#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>

////////////////////////////////////////////////
////////////////// PARAMETERS //////////////////
////////////////////////////////////////////////

uint16_t destination_id = 0x6058;     // the network id of the other pozyx device: fill in the network id of the other device
signed int range_step_mm = 1000;      // every 1000mm in range, one LED less will be giving light.

uint8_t ranging_protocol = POZYX_RANGE_PROTOCOL_PRECISION; // ranging protocol of the Pozyx.

uint16_t remote_id = 0x6065;          // the network ID of the remote device
bool remote = false;                  // whether to use the given remote device for ranging

float alpha = 0.83;
float range_f = 0.0;
uint16_t val_uint16_t;

uint32_t dt_test, tmp_test;

////////////////////////////////////////////////

void setup(){
  Serial.begin(115200);

  if(Pozyx.begin() == POZYX_FAILURE){
    Serial.println("ERROR: Unable to connect to POZYX shield");
    Serial.println("Reset required");
    delay(100);
    abort();
  }
  // setting the remote_id back to NULL will use the local Pozyx
  if (!remote){
    remote_id = NULL;
  }
  Serial.println("------------POZYX RANGING V1.0------------");
  Serial.println("NOTES:");
  Serial.println("- Change the parameters:");
  Serial.println("\tdestination_id (target device)");
  Serial.println("\trange_step (mm)");
  Serial.println();
  Serial.println("- Approach target device to see range and");
  Serial.println("led control");
  Serial.println("------------POZYX RANGING V1.0------------");
  Serial.println();
  Serial.println("START Ranging:");

  // make sure the pozyx system has no control over the LEDs, we're the boss
  uint8_t led_config = 0x0;
  Pozyx.setLedConfig(led_config, remote_id);
  // do the same with the remote device
  Pozyx.setLedConfig(led_config, destination_id);
  // set the ranging protocol
  Pozyx.setRangingProtocol(ranging_protocol, remote_id);
  tmp_test = micros();
}

void loop(){
  device_range_t range;
  int status = 0;
  
  
  dt_test = micros() - tmp_test;
  if ( dt_test > 50000 )
  {   
      tmp_test += dt_test;
      
      status = Pozyx.doRanging(destination_id, &range);
  
      if (status == POZYX_SUCCESS){
         range_f = (1-alpha)*range_f+alpha*((float)range.distance);
         val_uint16_t = (uint16_t)(range_f/10.0);
      }
      else
      {
         val_uint16_t = 0;
      }

      
      uint8_t *ptr_val_int16_t = (uint8_t *) &val_uint16_t;

//      Serial.print(range_);Serial.print(" ");
//      Serial.print(val_int16_t);Serial.println(" ");
//      Serial.print(range2_);Serial.print(" ");
//      Serial.print(range2_f);Serial.println(" ");

      Serial.write(137);
      Serial.write( *(ptr_val_int16_t) );
      Serial.write( *(ptr_val_int16_t+1) );
      Serial.write(173);
  }
  
  
//  
//  // let's perform ranging with the destination
//  if(!remote)
//    status = Pozyx.doRanging(destination_id, &range);
//  else
//    status = Pozyx.doRemoteRanging(remote_id, destination_id, &range);
//  
//  if (status == POZYX_SUCCESS){
////    Serial.print(range.timestamp);
////    Serial.print("ms, ");
//    range_f = (1-alpha)*range_f+alpha*((float)range.distance);
//    Serial.println(range_f);
//   
//    
//    // now control some LEDs; the closer the two devices are, the more LEDs will be lit
//    if (ledControl(range.distance) == POZYX_FAILURE){
//      Serial.println("0");
////      Serial.println("ERROR: setting (remote) leds");
//    }
//  }
//  else{
//    Serial.println("ERROR: ranging");
//  }
}

int ledControl(uint32_t range){
  int status = POZYX_SUCCESS;
  // set the LEDs of the pozyx device
  status &= Pozyx.setLed(4, (range < range_step_mm), remote_id);
  status &= Pozyx.setLed(3, (range < 2*range_step_mm), remote_id);
  status &= Pozyx.setLed(2, (range < 3*range_step_mm), remote_id);
  status &= Pozyx.setLed(1, (range < 4*range_step_mm), remote_id);

  // set the LEDs of the destination pozyx device
  status &= Pozyx.setLed(4, (range < range_step_mm), destination_id);
  status &= Pozyx.setLed(3, (range < 2*range_step_mm), destination_id);
  status &= Pozyx.setLed(2, (range < 3*range_step_mm), destination_id);
  status &= Pozyx.setLed(1, (range < 4*range_step_mm), destination_id);

  // status will be zero if setting the LEDs failed somewhere along the way
  return status;
}
