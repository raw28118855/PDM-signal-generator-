#include "pdm.h"
#include <stdint.h>   

uint8_t PDM_Next(uint16_t number_of_sampling);
uint32_t quant_error=0;

uint8_t PDM_Next(uint16_t number_of_sampling){
  
  quant_error+=number_of_sampling ;
  if(quant_error>=(bit_depth_minus_1)){
    quant_error-=bit_depth_minus_1;
    return 1;
  }
  else return 0;
}

