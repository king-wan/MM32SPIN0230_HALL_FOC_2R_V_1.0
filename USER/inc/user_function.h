#ifndef __USER_SUB_H
#define __USER_SUB_H

#include "mm32_device.h"
#include "hal_conf.h"
#include "motor_profile.h"

/*
 * Board-level parameters
 * ----------------------
 * motor_profile.h stores motor-specific parameters.
 * This file stores controller-board-specific analog front-end and protection
 * parameters, which normally change only when the PCB hardware changes.
 *
 * File role:
 * - Edit this file when the controller board hardware changes.
 * - Typical reasons: shunt resistor changed, op-amp gain changed, bus divider
 *   changed, or protection thresholds must match a new power stage.
 * - For motor replacement only, this file is usually left unchanged.
 */

/*
 * Current measurement parameters
 * ISUM_R_VALUE: bus-current shunt resistor value, unit mOhm.
 * ISUM_AMP_FACTOR: bus-current amplifier gain, unit 0.1x.
 * ISHUNT_R_VALUE: phase-current shunt resistor value, unit mOhm.
 * ISHUNT_AMP_FACTOR: phase-current amplifier gain, unit 0.1x.
 * ISUM_GAIN: bus-current conversion gain in q15 domain, approximately counts per 1 A.
 * ISHUNT_GAIN: phase-current conversion gain in q15 domain, approximately counts per 1 A.
 * SPEED_LOOP_MAX_CURRENT_Q15: speed-loop current ceiling converted from motor current units to q15.
 * RUN_MIN_OUT_Q15: minimum running torque/current converted from motor current units to q15.
 */
//--------------------- Current measurement -------------------------------------------------------------------
#define 	ISUM_R_VALUE    		50 	//unit:milli ohm, 50 means 0.05 ohm
#define 	ISUM_AMP_FACTOR  		50 	//unit:0.1 amplification factor, 50 means 50*0.1 = 5 amplification factor
#define 	ISHUNT_R_VALUE    		100 //unit:milli ohm, 50 means 0.05 ohm
#define 	ISHUNT_AMP_FACTOR		50 	//unit:0.1 amplification factor, 50 means 50*0.1 = 5 amplification factor
#define 	ISUM_GAIN 		((ISUM_R_VALUE*ISUM_AMP_FACTOR*32768)/50000) 		//50*5*32768/5000 = 1638---1A
#define 	ISHUNT_GAIN 	((ISHUNT_R_VALUE*ISHUNT_AMP_FACTOR*32768)/50000) 	//50*5*32768/5000 = 1638---1A
#define     SPEED_LOOP_MAX_CURRENT_Q15  ((ISHUNT_GAIN * SPEED_LOOP_MAX_CURRENT_A10) / 10)
#define     RUN_MIN_OUT_Q15             ((ISHUNT_GAIN * RUN_MIN_CURRENT_A10) / 10)

/*
 * DC bus and protection parameters
 * VBUS_PULL_UP_R: upper resistor of the DC-bus divider, unit 0.1 kOhm.
 * VBUS_PULL_DOWN_R: lower resistor of the DC-bus divider, unit 0.1 kOhm.
 * VBUS_HIGH_VALUE1: over-voltage trip threshold, unit 0.01 V.
 * VBUS_HIGH_VALUE2: over-voltage release threshold, unit 0.01 V.
 * VBUS_LOW_VALUE1: under-voltage trip threshold, unit 0.01 V.
 * VBUS_LOW_VALUE2: under-voltage release threshold, unit 0.01 V.
 * IBUS_OVER_VALUE: software over-current protection threshold, unit 0.01 A.
 * IBUS_LIMIT_VALUE: software current-limit threshold, unit 0.01 A.
 * LACK_PHASE_DETECT_CYCLE: observation window used for missing-phase diagnosis.
 */
//--------------------- DC Bus voltage measurement --------------------------------------------------------------
#define 	VBUS_PULL_UP_R     1000 	//unit : 0.1K Ohm, 1000 means 100K Ohm //20181215
#define 	VBUS_PULL_DOWN_R   100  	//unit : 0.1K Ohm, 100  means 10K  Ohm //20181215

#define 	VBUS_HIGH_VALUE1	3600  		//过压阀值		36V
#define 	VBUS_HIGH_VALUE2	3500  		//过压恢复		35V
#define 	VBUS_LOW_VALUE1		1600   	 	//欠压阀值		20V
#define 	VBUS_LOW_VALUE2		1700   	  	//欠压恢复		21V
#define 	IBUS_OVER_VALUE		400   		//软件过流值		4A
#define 	IBUS_LIMIT_VALUE	300			//软件限流		3A
#define 	LACK_PHASE_DETECT_CYCLE		1000

//电机状态表
#define 	IDLESTATE      	0
#define		RUNSTATE  		1
#define 	BRAKESTATE     	2
#define 	ERRORSTATE     	3

/*
 * Motor state definitions
 * IDLESTATE: idle / PWM disabled.
 * RUNSTATE: normal closed-loop running.
 * BRAKESTATE: active braking state.
 * ERRORSTATE: latched fault state.
 *
 * ADC_components fields
 * SPEED: potentiometer/speed command ADC raw sample.
 * VBusInput/VBusAvrg: raw and filtered bus voltage.
 * IBusInput/IBusAvrg: raw and filtered bus current.
 * IU/IV: phase current feedback used by FOC.
 * EU/EV/EW: phase voltage or back-EMF related samples.
 */
typedef struct
{ 
	u16 SPEED;
	u16	VBusInput;
	u16 VBusAvrg;
	s16	IBusInput;
	s16 IBusAvrg;
	s16 IU;
	s16 IV;
	s16 EU;
	s16 EV;
	s16 EW;
}ADC_components; 	


extern int32_t IsumGain;
extern int32_t IshuntGain;
extern int32_t SpeedGain;
extern uint32_t VbusGain;
extern ADC_components ADC_Structure;

/*
 * Motion control command layer
 * ----------------------------
 * A fixed 10-byte UART packet configures the motion mode and parameters.
 * The low-level FOC loop consumes the globals below every 5 ms.
 *
 * Request frame layout:
 * [0]=0xA5 [1]=cmd [2]=arg0 [3]=arg1 [4]=data0 [5]=data1 [6]=data2 [7]=data3 [8]=sum_lo [9]=sum_hi
 * checksum = sum(frame[0..7])
 *
 * Commands:
 * SET_MODE: arg0=mode, data0..1=speed rpm, data2..3=period ms or turns
 * JOG: arg0=action(0 step,1 hold,2 release), arg1=dir(1 forward,2 reverse)
 * STOP / QUERY: payload ignored
 */
#define MOTION_UART_FRAME_LEN             10U
#define MOTION_UART_SYNC_REQ              0xA5U
#define MOTION_UART_SYNC_RSP              0x5AU

#define MOTION_CMD_SET_MODE               0x01U
#define MOTION_CMD_JOG                    0x02U
#define MOTION_CMD_STOP                   0x03U
#define MOTION_CMD_QUERY                  0x04U

#define MOTION_STATUS_OK                  0x00U
#define MOTION_STATUS_BAD_SYNC            0x01U
#define MOTION_STATUS_BAD_CHECKSUM        0x02U
#define MOTION_STATUS_BAD_CMD             0x03U
#define MOTION_STATUS_BAD_PARAM           0x04U

#define MOTION_MODE_STOP                  0U
#define MOTION_MODE_FORWARD               1U
#define MOTION_MODE_REVERSE               2U
#define MOTION_MODE_RECIP_TIME            3U
#define MOTION_MODE_RECIP_TURNS           4U
#define MOTION_MODE_JOG                   5U

#define MOTION_DIR_FORWARD                1U
#define MOTION_DIR_REVERSE                2U

extern uint8_t MotionModeStatus;
extern uint8_t MotionRunEnable;
extern int8_t MotionRunDir;
extern uint16_t MotionSpeedCmdRpm10;
extern uint8_t MotionPositionEnable;
extern int16_t MotionPositionTargetQ15;
extern int16_t MotionPositionHoldIqQ15;
extern int16_t MotionSpeedCurrentLimitQ15;
extern uint8_t MotionReverseBraking;
extern int16_t MotionBrakeIqQ15;
extern uint16_t MotionTorquePermille;
extern uint8_t MotionParkAngleValid;
extern int16_t MotionParkAngleQ15;

extern void Init_Parameter(void);
extern void MotionCtrl_Init(void);
extern void MotionCtrl_Update5ms(int16_t angle_q15, int16_t speed_rpm10, int16_t iq_q15);
extern uint16_t MotionCtrl_ProcessFrame(const uint8_t *rx, uint16_t rx_len, uint8_t *tx, uint16_t tx_cap);

#endif
