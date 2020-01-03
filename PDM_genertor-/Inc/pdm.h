#include <stdint.h>   
#define M_PI 3.1415926535
#define pdm_clock 3200000
#define bit_depth 65536
#define bit_depth_minus_1  bit_depth-1
#define bit_depth_half bit_depth/2

/******begin PV functon defin***************/
uint8_t PDM_Next(uint16_t number_of_sampling);