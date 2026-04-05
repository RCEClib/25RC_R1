#ifndef __ELRS_H__
#define __ELRS_H__

#include "main.h"
#include "usart.h"

#define BUFF_SIZE 36

#define CRSF_ADDR 0XC8
#define CRSF_LEN 0X22
#define CRSF_TYPE 0X16

typedef enum {
    ON,
    OFF
} double_key_state;

typedef enum {
    TOP,
    MID,
    BOT
} triple_key_state;

typedef struct {
    uint16_t online;

    struct {
        uint16_t ch[16];
    } rc;

    struct {
        int16_t l_x;
        int16_t l_y;
        int16_t r_x;
        int16_t r_y;
    } joy;

    struct {
        int16_t S1;
        int16_t S2;
    } var;

    struct {
        uint8_t SA;
        uint8_t SB;
        uint8_t SC;
        uint8_t SD;
        uint8_t SE;
        uint8_t SF;
    } key;
} remoter_t;

HAL_StatusTypeDef ELRS_Init(void);

extern uint8_t rx_buff[BUFF_SIZE];
extern remoter_t remoter;

#endif /*__ELRS_H__ */
