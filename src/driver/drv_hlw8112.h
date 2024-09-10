#pragma once

void HLW8112_SPI_Init(void);
void HLW8112_SPI_RunEverySecond(void);

//Calibration parameters and metering control registers
#define	HLW8112_SYSCON_REG 0X00
#define	HLW8112_EMUCON_REG 0X01
#define	HLW8112_HFConst_REG 0X02
#define	HLW8112_PAStart_REG 0X03
#define	HLW8112_PBStart_REG 0X04
#define	HLW8112_PAGain_REG 0X05
#define	HLW8112_PBGain_REG 0X06
#define	HLW8112_PhaseA_REG 0X07
#define	HLW8112_PhaseB_REG 0X08
#define	HLW8112_PAOS_REG 0X0A
#define	HLW8112_PBOS_REG 0X0B
#define	HLW8112_RmsIAOS_REG 0X0E
#define	HLW8112_RmsIBOS_REG 0X0F
#define	HLW8112_IBGain_REG 0X10
#define	HLW8112_PSGain_REG 0X11
#define	HLW8112_PSOS_REG 0X12
#define	HLW8112_EMUCON2_REG 0X13

//Meter Parameter and Status Register

//Interrupt Register
#define	HLW8112_IE_REG 0X40
#define	HLW8112_IF_REG 0X41
#define	HLW8112_RIF_REG 0X42

//System Status Register