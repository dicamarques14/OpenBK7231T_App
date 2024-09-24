#include "drv_hlw8112.h"

#include <math.h>
#include <stdint.h>

#include "../logging/logging.h"
#include "../new_pins.h"
#include "../cmnds/cmd_public.h"
#include "drv_pwrCal.h"
#include "drv_spi.h"

// Datasheet says 900 kHz is supported, but it produced ~50% check sum errors  
#define HLW8112_SPI_BAUD_RATE 1000000  // 900000

#define DEFAULT_VOLTAGE_CAL 15188
#define DEFAULT_CURRENT_CAL 251210
#define DEFAULT_POWER_CAL 598

typedef struct {
    uint32_t i_rms;
    uint32_t v_rms;
    int32_t watt;
    uint32_t cf_cnt;
    uint32_t freq;
} hlw8112_data_t;


static void ScaleAndUpdate(hlw8112_data_t *data) {
}

// Function to enable write access
void writeEnable() {
    uint8_t send[3];
    send[0] = HLW8112_REG_SPECIAL;
    send[1] = HLW8112_COMMAND_WRITE_EN;
    send[2] = 0x00;
    SPI_WriteBytes(send, sizeof(send));
}

// Function to disable write access (optional, to protect registers after writing)
void writeDisable() {
    uint8_t send[3];
    send[0] = HLW8112_REG_SPECIAL;
    send[1] = HLW8112_COMMAND_WRITE_PROTECT;
    send[2] = 0x00;
    SPI_WriteBytes(send, sizeof(send));
}

static int SPI_ReadReg(uint8_t reg, uint16_t *val) {
    reg = reg & 0x7F;  // MSB 0 indicates read
    uint16_t recv;

    SPI_WriteBytes(reg, sizeof(reg));
    SPI_ReadBytes(recv, sizeof(recv));

    *val = recv;
    return 0;
}

static int SPI_WriteReg(uint8_t reg, uint16_t val) {
    reg = reg | 0x80;  // MSB 1 indicates write
    uint8_t send[3];
    send[0] = reg;
    send[1] = (val >> 8);
    send[2] = ((val >> 16) & 0xFF);


    SPI_WriteBytes(send, sizeof(send));

    uint32_t read;
    SPI_ReadReg(reg, &read);
    if (read == val) {
        return 0;
    }

    ADDLOG_ERROR(LOG_FEATURE_ENERGYMETER,
                 "Failed to write reg %02X val %02X: read %02X", reg, val,
                 read);
    return -1;
}

static void HLW8112_Init(void) {
}

// THIS IS called by 'startDriver HLW8112' command
void HLW8112_SPI_Init(void) {
    ADDLOG_WARN(LOG_FEATURE_ENERGYMETER,
                 "HLW8112_SPI_Init");
    HLW8112_Init();

    SPI_DriverInit();
    spi_config_t cfg;
    cfg.role = SPI_ROLE_MASTER;
    cfg.bit_width = SPI_BIT_WIDTH_8BITS;
    cfg.polarity = SPI_POLARITY_LOW;
    cfg.phase = SPI_PHASE_1ST_EDGE;
    cfg.wire_mode = SPI_4WIRE_MODE;
    cfg.baud_rate = HLW8112_SPI_BAUD_RATE;
    cfg.bit_order = SPI_MSB_FIRST;
    SPI_Init(&cfg);

    // System Control: Enable Voltage and Current Channels
    SPI_WriteReg(HLW8112_REG_SYSCON, 0x0A04); // Enable all channels (default value)

    // Energy Measure Control: Set measurement mode
    SPI_WriteReg(HLW8112_REG_EMUCON, 0x0000); // Default energy measure control register

    ADDLOG_WARN(LOG_FEATURE_ENERGYMETER,
                 "HLW8112_SPI_Init_OUT");
}

void HLW8112_SPI_RunEverySecond(void) { 

    ADDLOG_WARN(LOG_FEATURE_ENERGYMETER,
                 "I'm Alive");

    
    uint16_t voltage_rms = 0;
    uint16_t ufreq = 0;
    uint16_t peaku = 0;
    SPI_ReadReg(HLW8112_REG_RMSU, &voltage_rms);
    SPI_ReadReg(HLW8112_REG_UFREQ, &ufreq);
    SPI_ReadReg(HLW8112_REG_PEAKU, &peaku);
    ADDLOG_INFO(LOG_FEATURE_ENERGYMETER,
                 "read -> %i | %i | %i |", voltage_rms, ufreq, peaku);
}

void HLW8112_AppendInformationToHTTPIndexPage(http_request_t *request)
{
}