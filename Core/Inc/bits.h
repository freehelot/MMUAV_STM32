/*
 * bits.h
 *
 *  Created on: 26 Apr 2021
 *      Author: hrvoje
 */

#ifndef INC_BITS_H_
#define INC_BITS_H_

#include <stdbool.h>
/*
typedef struct gconf_t
{
	static uint8_t address = 0x00;
	union{
		uint32_t sr : 18;
		struct{
			bool i_scale_analog : 1,
			internal_rsense :1,
			en_pwm_mode : 1,
			enc_commutation : 1, // 2130, 5130
			shaft : 1,
			diag0_error : 1,
			diag0_otpw : 1,
			diag0_stall : 1,
			diag1_stall : 1,
			diag1_index : 1,
			diag1_onstate : 1,
			diag1_steps_skipped : 1,
			diag0_int_pushpull : 1,
			diag1_pushpull : 1,
			small_hysteresis : 1,
			stop_enable : 1,
			direct_mode : 1;
		};
	};
}gconf_t;
*/

#define MASK_32_DOWN (0x0000FFFF)
#define MASK_8_DOWN (0x0F)

//SPI_STAT
#define SPI_STATUS_RESET_MASK                        	0x01  //
#define SPI_STATUS_RESET_SHIFT                          0           // min: 0, max: 1, default: 0
#define SPI_STATUS_DRV_ERR_MASK                         0x02  //
#define SPI_STATUS_DRV_ERR_SHIFT                        1           // min: 0, max: 1, default: 0
#define SPI_STATUS_SG2_MASK                        	  	0x04  //
#define SPI_STATUS_SG2_SHIFT                            2           // min: 0, max: 1, default: 0
#define SPI_STATUS_STAND_MASK                           0x08  //
#define SPI_STATUS_STAND_SHIFT                          3           // min: 0, max: 1, default: 0


typedef struct gconf_t
{
	//uint8_t address = 0x00; //address of GCONF
	union
	{
		//const static uint8_t address = 0x00; //address of GCONF
		uint32_t sr : 18;
		struct
		{
			unsigned i_scale_analog : 1,
			internal_rsense : 1,
			en_pwm_mode : 1,
			enc_commutation : 1,
			shaft : 1,
			diag0_error : 1,
			diag0_otpw : 1,
			diag0_stall : 1,
			diag1_stall : 1,
			diag1_index : 1,
			diag1_onstate : 1,
			diag1_steps_skipped : 1,
			diag0_int_pushpull : 1,
			diag1_pushpull : 1,
			small_hysteresis : 1,
			stop_enable : 1,
			direct_mode : 1,
			test_mode : 1;
		};
	};
}gconf_t;


typedef struct ihold_irun_t {
  union {
    uint32_t sr : 20;
    struct {
      uint8_t ihold : 5,
                    : 3,
              irun : 5,
                   : 3,
              iholddelay : 4;
    };
  };
}ihold_irun_t ;


typedef struct chopconf_t {
  union {
    uint32_t sr : 32;
    struct {
      uint8_t toff : 4,
              hstrt : 3,
              hend : 4,
      	  	  fd3  	: 1;
	  bool		  disfdcc : 1,
              rndtf : 1,
              chm : 1;
      uint8_t tbl : 2;
      bool    vsense : 1,
              vhighfs : 1,
              vhighchm : 1;
      uint8_t sync : 4, // 2130, 5130
              mres : 4;
      bool    intpol : 1,
              dedge : 1,
              diss2g : 1;
    };

  };
}chopconf_t;


typedef struct coolconf_t
{
	 union {
	    uint32_t sr : 25;
	    struct {
	      uint8_t semin : 4,
	                    : 1,
	              seup : 2,
	                    : 1,
	              semax : 4,
	                    : 1,
	              sedn : 2;
	      bool    seimin : 1;
	      int8_t  sgt : 7,
	                  : 1;
	      bool    sfilt : 1;
	    };
	 };
}coolconf_t;



typedef struct PWMCONF_t {
  union {
    uint32_t sr : 22;
    struct {
      uint8_t pwm_ampl : 8,
              pwm_grad : 8,
              pwm_freq : 2;
      bool pwm_autoscale : 1,
           pwm_symmetric : 1;
      uint8_t freewheel : 2;
    };
  };
}pwmconf_t;

typedef struct DRV_STATUS_t {
    union {
      uint32_t sr;
      struct {
        uint16_t sg_result : 10;
        uint8_t            : 5;
        bool fsactive : 1;
        uint8_t cs_actual : 5,
                          : 3;
        bool  stallGuard : 1,
              ot : 1,
              otpw : 1,
              s2ga : 1,
              s2gb : 1,
              ola : 1,
              olb : 1,
              stst : 1;
      };
    };
}drv_status_t;


union
{
	unsigned char byte;
	// a structure with 8 single bit bit-field objects, overlapping the union member "byte"
	struct
	{
		unsigned b0:1;
		unsigned b1:1;
		unsigned b2:1;
		unsigned b3:1;
		unsigned b4:1;
		unsigned b5:1;
		unsigned b6:1;
		unsigned b7:1;
	};
}byte_u;

#endif /* INC_BITS_H_ */
