#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>
/*
 * ******Parametres Reception Gains XBee
 */
/******************************/
uint16_t remote_id = 0x6000;                            // set this to the ID of the remote device
bool remote = false;                                    // set this to true to use the remote ID

boolean use_processing = false;                         // set this to true to output data for the processing sketch
//boolean use_processing = true; 

uint8_t num_anchors = 7;                                    // the number of anchors
uint16_t anchors[7] = {0x6058,  0x602f, 0x603e,   0x6036,   0x604f,   0x6016, 0x6050};     // the network id of the anchors: change these to the network ids of your anchors.
int32_t anchors_x[7]= {-626 ,   5001,   0,        3720,     860,      3670,   5032};               // anchor x-coorindates in mm
int32_t anchors_y[7]= {3537 ,   2165,   0,        0,        7310,     7310,   5367};                  // anchor y-coordinates in mm
int32_t heights[7] = { 2238 ,   2815,   3180,     2860,     2185,     1995,   2528};   
uint8_t algorithm = POZYX_POS_ALG_TRACKING;
uint8_t dimension = POZYX_3D;                           // positioning dimension
//uint8_t dimension = POZYX_2D;                           // positioning dimension
//uint8_t dimension = POZYX_2_5D; 
int32_t height = 1000;                                  // height of device, required in 2.5D positioning

/******************************/
////PARAMETRES DU FILTRE
const int H=15;
int16_t PosXs=0, PosYs=0, PosXs_prev=0, PosYs_prev=0;
int X[H], Y[H];
float promX, promY;
float VelX, VelY;
float VelXs_, VelYs_;
float VelXs_1, VelYs_1;
int VelXs, VelYs;
//////////////////////////////////////////////
// Communication
int16_t buffer_int16;
unsigned char *ptr_buffer_int16 = (unsigned char *)&buffer_int16;

void setup(){
  
 
/******************************************/
 Serial.begin(38400);

  if(Pozyx.begin() == POZYX_FAILURE){
    Serial.println(F("ERROR: Unable to connect to POZYX shield"));
    Serial.println(F("Reset required"));
    delay(100);
    abort();
  }

  if(!remote){
    remote_id = NULL;
  }
/*
  Serial.println(F("----------POZYX POSITIONING V1.0----------"));
  Serial.println(F("NOTES:"));
  Serial.println(F("- No parameters required."));
  Serial.println();
  Serial.println(F("- System will auto start anchor configuration"));
  Serial.println();
  Serial.println(F("- System will auto start positioning"));
  Serial.println(F("----------POZYX POSITIONING V1.0----------"));
  Serial.println();
  Serial.println(F("Performing manual anchor configuration:"));
*/
  // clear all previous devices in the device list
  Pozyx.clearDevices(remote_id);
  // sets the anchor manually
  setAnchorsManual();

  printCalibrationResult();
  delay(1000);

  //Serial.println(F("Starting positioning: "));
/******************************************/

}

void loop(){

  int i,j;
    coordinates_t position;
    int status;
  
    if(remote){
        status = Pozyx.doRemotePositioning(remote_id, &position, dimension, height, algorithm);
      } else{
          status = Pozyx.doPositioning(&position, dimension, height, algorithm);
         }

   Historique(position, X, Y);
   Promedio(X, &promX, 1, H-1);
   Promedio(Y, &promY, 1, H-1);
   getVelocity(X, &VelX, 1, 0.025);
   getVelocity(Y, &VelY, 1, 0.025);
   if (VelX>2000) {VelX=2000;}; if (VelX<-2000) {VelX=-2000;}
   if (VelY>2000) {VelY=2000;}; if (VelY<-2000) {VelY=-2000;}
// mise en forme avant denvoyer via série
    PosXs=(int16_t)(100.0*promX/24.0);
    PosYs=(int16_t)(100.0*promY/30.0);
    VelXs=(int16_t)(10.0*VelX);
    VelYs=(int16_t)(10.0*VelY);
    /*
//     dt=millis()-last_millis;
//    last_millis+=dt;
//   Serial.print("dt: ");Serial.print(dt); Serial.print("  ");
//    Serial.print("PosX: ");Serial.print(position.x); Serial.print("  ");
//    Serial.print("PosY: ");Serial.println(position.y);  
  Serial.print(position.x/24.0);Serial.print(" ");
  Serial.print(PosXs/100.0);Serial.print(" ");
  //Serial.print(PosYs/100.0);Serial.print(" ");
  //Serial.print(1.5*400.0*VelXs_);Serial.print(" ");
  //Serial.print(1.5*400.0*VelYs_);Serial.print(" ");
  //Serial.print(VelXs);Serial.print(" ");
  //Serial.print(VelYs);Serial.print(" ");

   Serial.println(" ");
  */
  
   Serial.write(137);
    buffer_int16 = PosXs;
    for(j=0;j<2;j++)
    {
      Serial.write(ptr_buffer_int16[j]);
    }
    buffer_int16 = PosYs;
    for(j=0;j<2;j++)
    {
      Serial.write(ptr_buffer_int16[j]);
    }
    buffer_int16 = VelXs;
    for(j=0;j<2;j++)
    {
      Serial.write(ptr_buffer_int16[j]);
    }
    buffer_int16 = VelYs;
    for(j=0;j<2;j++)
    {
      Serial.write(ptr_buffer_int16[j]);
    }
   Serial.write(173);
}

/*************************************************/
void Historique(coordinates_t coor, int A[H], int B[H]){
    if (coor.x>5000){coor.x=5000;}
    if (coor.x<-1000){coor.x=-1000;}
    if (coor.y>7500){coor.y=7500;}
    if (coor.y<0){coor.y=0;}
    
      for(int k=0;k<(H-1);k++){
        A[k]=A[k+1];
        B[k]=B[k+1];
      }
      A[H-1]=coor.x;
      B[H-1]=coor.y;
  }
  
 void Promedio(int A[H], float* prom,int NbData, int posf){
      float sum=0;
      int vali=posf+1-NbData;
      int valf=posf+1;
      for(int k=vali;k<valf;k++){
        sum+=A[k];
        }
      *prom=sum/NbData;      
  }

void getVelocity(int A[H], float* Vel, int NbData, float Ts ){
     float vali;
     float valf;
     Promedio(A, &valf, NbData , H-1 );
     Promedio(A, &vali, NbData , H-2 );
     *Vel=(valf-vali)/Ts;
  }

/************************************************/

void printErrorCode(String operation){
  uint8_t error_code;
  if (remote_id == NULL){
    Pozyx.getErrorCode(&error_code);
    Serial.print("ERROR ");
    Serial.print(operation);
    Serial.print(", local error code: 0x");
    Serial.println(error_code, HEX);
    return;
  }
  int status = Pozyx.getErrorCode(&error_code, remote_id);
  if(status == POZYX_SUCCESS){
    Serial.print("ERROR ");
    Serial.print(operation);
    Serial.print(" on ID 0x");
    Serial.print(remote_id, HEX);
    Serial.print(", error code: 0x");
    Serial.println(error_code, HEX);
  }else{
    Pozyx.getErrorCode(&error_code);
    Serial.print("ERROR ");
    Serial.print(operation);
    Serial.print(", couldn't retrieve remote error code, local error: 0x");
    Serial.println(error_code, HEX);
  }
}

// print out the anchor coordinates (also required for the processing sketch)
void printCalibrationResult(){
  uint8_t list_size;
  int status;

  status = Pozyx.getDeviceListSize(&list_size, remote_id);
  Serial.print("list size: ");
  Serial.println(status*list_size);

  if(list_size == 0){
    printErrorCode("configuration");
    return;
  }

  uint16_t device_ids[list_size];
  status &= Pozyx.getDeviceIds(device_ids, list_size, remote_id);

  Serial.println(F("Calibration result:"));
  Serial.print(F("Anchors found: "));
  Serial.println(list_size);

  coordinates_t anchor_coor;
  for(int i = 0; i < list_size; i++)
  {
    /*Serial.print("ANCHOR,");
    Serial.print("0x");
    Serial.print(device_ids[i], HEX);
    Serial.print(",");*/
    Pozyx.getDeviceCoordinates(device_ids[i], &anchor_coor, remote_id);
    /*Serial.print(anchor_coor.x);
    Serial.print(",");
    Serial.print(anchor_coor.y);
    Serial.print(",");
    Serial.println(anchor_coor.z);*/
  }
}

// function to manually set the anchor coordinates
void setAnchorsManual(){
  for(int i = 0; i < num_anchors; i++){
    device_coordinates_t anchor;
    anchor.network_id = anchors[i];
    anchor.flag = 0x1;
    anchor.pos.x = anchors_x[i];
    anchor.pos.y = anchors_y[i];
    anchor.pos.z = heights[i];
    Pozyx.addDevice(anchor, remote_id);
    if (num_anchors > 4)
    {
      Pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO, num_anchors);
    }
 }
 
}
