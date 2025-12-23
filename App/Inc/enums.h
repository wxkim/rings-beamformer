#ifndef ENUMS_H
#define ENUMS_H

# include <stdint.h>

// 64 bit helpers
#define AWMF_BIT(n) (UINT64_C(1) << (n))
#define AWMF_FIELD_MASK(width) ((UINT64_C(1) << (width)) - 1u)

//PDET [15:10]
#define AWMF_MODE_PDET_BW_SHIFT 10u
#define AWMF_MODE_PDET_BW_WIDTH 6u

#define AWMF_MODE_PDET_BW_MASK (AWMF_FIELD_MASK(AWMF_MODE_PDET_BW_WIDTH) << AWMF_MODE_PDET_BW_SHIFT)

//Register Address

typedef enum{
    //Control
    AWMF_REG_MODE = 0x000,
    AWMF_REG_BW_TX_A = 0x001,
    AWMF_REG_BW_RX_A = 0x002,
    AWMF_REG_RX_MEMADDR = 0x003,
    AWMF_REG_TX_MEMADDR = 0x004,
    AWMF_REG_ATTTC = 0x005,

    AWMF_REG_ENGR0 = 0x00D,
    AWMF_REG_ATC_CALC = 0x016,
    AWMF_REG_ATC_CONFIG3 = 0x01F,
    AWMF_REG_ATC_CONFIG5 = 0x021,

    AWMF_REG_BW_TX_B = 0x022,
    AWMF_REG_BW_RX_B = 0x023,

    //Telemetry
    AWMF_REG_TELEM0 = 0x030,
    AWMF_REG_TELEM1 = 0x031,
    AWMF_REG_TELEM2 = 0x032,
    AWMF_REG_TELEM3 = 0x033,

    //MISC
    AWMF_REG_SPARE = 0x03A,
    AWMF_REG_RADDR = 0x03D,
    AWMF_REG_PROD_ID = 0x03E,

    //Beam Steering Base Addresses
    AWMF_REG_RX_TDBS_BASE = 0x100,
    AWMF_REG_TX_TDBS_BASE = 0x140,
    AWMF_REG_FBS_BASE = 0x200,

} awmf0165_reg_t;


// Common On/Off
typedef enum {AWMF_DISABLE = 0, AWMF_ENABLE = 1} awmf_enable_t;

// MODE Register field values
typedef enum {AWMF_PARITY_OFF = 0, AWMF_PARITY_ON = 1} awmf_parity_en_t;
typedef enum {AWMF_ATC_SW = 0, AWMF_ATC_AUTO = 1} awmf_atc_mode_t;
typedef enum {AWMF_SPI_FBS = 0, AWMF_SPI_TDBS = 1} awmf_spi_mem_sel_t;

// MODE  bit masks 
#define AWMF_MODE_PARITY_DISCARD_EN AWMF_BIT(40)
#define AWMF_MODE_PARITY_EN AWMF_BIT(39)
#define AWMF_MODE_ATC_MODE AWMF_BIT(30)
#define AWMF_MODE_SW_RESET AWMF_BIT(24)
#define AWMF_MODE_SPI_LDB_EN AWMF_BIT(22)
#define AWMF_MODE_ZCAL_INIT AWMF_BIT(3)
#define AWMF_MODE_SPI_FBS_SEL AWMF_BIT(2)
#define AWMF_MODE_SPI_TDBS_SEL AWMF_BIT(1)

// TX/RX phase gain codes
typedef uint8_t awmf_gain_code_t; // 4-bit 0.5 db steps
typedef uint8_t awmf_phase_code_t; // 6 bit 5.625 degrees steps

// Channels
typedef enum
{
    AWMF_CH1 = 0,
    AWMF_CH2,
    AWMF_CH3,
    AWMF_CH4
} awmf_channel_t;

//ATC trigger / update modes

typedef enum 
{
    AWMF_ATC_TRIG_NONE = 0,
    AWMF_ATC_TRIG_CSB = 1,
    AWMF_ATC_TRIG_TX_EN = 2,
    AWMF_ATC_TRIG_RX_EN = 3
} awmf_atc_trigger_t;

typedef enum
{
    AWMF_ATC_UPDATE_IMMEDIATE = 0,
    AWMF_ATC_UPDATE_SPI_END = 1,
    AWMF_ATC_UPDATE_EXIT_EN = 2,
    AWMF_ATC_UPDATE_BEAM_END = 3
}awmf_atc_update_trig_t;

// ENG register controls

typedef enum
{
    AWMF_ATC_USE = 0,
    AWMF_ATC_IGNORE = 1
}  awmf_atc_bypass_t;

typedef enum
{
    AWMF_TXRX_DISABLED = 0,
    AWMF_TXRX_ENABLED = 1
} awmf_txrx_enable_t;

//telemetry channel selector

typedef enum
{
    AWMF_TELEM_CH1A = 0,
    AWMF_TELEM_CH2A,
    AWMF_TELEM_CH3A,
    AWMF_TELEM_CH4A,
    AWMF_TELEM_CH1B,
    AWMF_TELEM_CH2B,
    AWMF_TELEM_CH3B,
    AWMF_TELEM_CH4B
} awmf_telem_channel_t;

// TDBS memory row selector 

typedef enum
{
    AWMF_TDBS_ROW_0 = 0,
    AWMF_TDBS_ROW_1 = 1
}awmf_tdbs_row_t;

typedef enum {AWMF_SPI_OK = 0, AWMF_SPI_ERROR = 1} awmf_spi_result_t;

#endif 