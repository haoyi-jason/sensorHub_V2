#ifndef _EXTI_CFG_
#define _EXTI_CFG_
#include "hal.h"


typedef struct{
    uint8_t intPort;
    uint8_t intBit;
}int_state_t;

extern int_state_t intState;


void adc_int_handler(EXTDriver *extp, expchannel_t channel);
void sdmmc_int_handler(EXTDriver *extp, expchannel_t channel);
void user_button_handler(EXTDriver *extp, expchannel_t channel);
//void zero_cross_int_func(EXTDriver *extp, expchannel_t channel);

void adxl_int_handler(EXTDriver *extp, expchannel_t channel);
void bmi160_int1_handler(EXTDriver *extp, expchannel_t channel);
void bmi160_int2_handler(EXTDriver *extp, expchannel_t channel);
void gpioa2_int_handler(EXTDriver *extp, expchannel_t channel);
void btlink_int_handler(EXTDriver *extp, expchannel_t channel);

void p7_3_intHandler(EXTDriver *extp, expchannel_t channel);
void p7_4_intHandler(EXTDriver *extp, expchannel_t channel);
void p7_5_intHandler(EXTDriver *extp, expchannel_t channel);
void p7_13_intHandler(EXTDriver *extp, expchannel_t channel);

void p8_3_intHandler(EXTDriver *extp, expchannel_t channel);
void p8_4_intHandler(EXTDriver *extp, expchannel_t channel);
void p8_5_intHandler(EXTDriver *extp, expchannel_t channel);
void p8_13_intHandler(EXTDriver *extp, expchannel_t channel);


static const EXTConfig extcfg = {
{
    {EXT_CH_MODE_DISABLED, NULL}, 
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_RISING_EDGE | EXT_MODE_GPIOA, gpioa2_int_handler}, //BMI160, P8.3 (PA11)
    {EXT_CH_MODE_DISABLED, NULL}, 
    {EXT_CH_MODE_DISABLED, NULL}, 
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_FALLING_EDGE | EXT_MODE_GPIOB, adxl_int_handler}, // ADXL-355, P7.4,(PB1)
    {EXT_CH_MODE_BOTH_EDGES | EXT_MODE_GPIOB, sdmmc_int_handler}, 
    {EXT_CH_MODE_FALLING_EDGE | EXT_MODE_GPIOA, bmi160_int1_handler}, //BMI160, P8.3 (PA11)
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_FALLING_EDGE | EXT_MODE_GPIOC, user_button_handler}, // User button, PB8
    {EXT_CH_MODE_DISABLED, NULL}, 
    {EXT_CH_MODE_DISABLED, NULL},
  }
};



#endif