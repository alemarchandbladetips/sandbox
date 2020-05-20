#ifndef SDP3X_H
	#define SDP3X_H

	// convert two 8 bit values to one word
	//#define BIU16(data, start) (((uint16_t)(data)[start]) << 8 | ((data)[start + 1]))
 
 #define BIU16(data, start) ( ( (uint16_t) data[start] ) << 8 | ( data[start + 1] ) )

	// data length of result from I2C
	#define COMMAND_DATA_LENGTH 2
	//#define RESULT_DATA_LENGTH 6
  #define RESULT_DATA_LENGTH 9

	#define DEFAULT_SDP3X_I2C_ADDRESS  0x21
	#define DEFAULT_SDP8XX_I2C_ADDRESS 0x25

	// triggered mode with 50ms conversion time
	typedef enum {
//	   SDP_MEASUREMENT_COMMAND_0 = 0x36,
//	   SDP_MEASUREMENT_COMMAND_1 = 0x2F
     SDP_MEASUREMENT_COMMAND_0 = 0x36,
     SDP_MEASUREMENT_COMMAND_1 = 0x1E
	} SDP3xCommands;

	class SDP3xClass
	{
	  public:
	  	SDP3xClass() : mI2CAddress(DEFAULT_SDP3X_I2C_ADDRESS), temps_exec(millis()) {}
	  	void setI2CAddress(uint8_t i2cAddress);

	  	uint8_t readSensor(uint8_t* readData, uint8_t size);
	  	float getPressureDiff(void);
      void getInfo_PT(void);
	  	float getTemperature(void);
      float getTemperature_SPD3x(void);
      uint16_t getScale_dP_SPD3x(void);
      float getdP_SDP3X(void);

	  private:
	  	uint8_t mI2CAddress;
      float temperature_SDP3x;
      float dP_SDP3x;
      uint16_t scale_dP_SDP3x;
      uint32_t temps_exec, dt_exec;
      boolean OK2read = false;
      boolean OK_Request = false;
	};

	extern SDP3xClass SDP3x;

#endif
