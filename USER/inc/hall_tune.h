#ifndef __HALL_TUNE_H_
#define __HALL_TUNE_H_

/*
 * Hall electrical angle compensation in Q15 electrical angle units.
 *
 * Signal flow in this project:
 * Hall state -> sector center angle table -> HallAngleOffset compensation
 * -> ctrlAngle -> Park/IPark -> PWM output.
 *
 * This offset compensates the fixed error between the theoretical Hall
 * sector center and the actual best FOC electrical angle of the motor.
 * If the offset is wrong, startup torque becomes weak and the motor may
 * need a hand push to rotate. The tuned value below is the calibration
 * result that allows direct startup on this hardware.
 */
#define HALL_EANGLE_OFFSET_Q15      (-6800)
#define HALL_SPEED_ADV_ENABLE       1
#define HALL_SPEED_ADV_DELAY_US     500
#define HALL_SPEED_ADV_MIN_RPM10    30
#define HALL_SPEED_ADV_MAX_Q15      2730
#define HALL_SPEED_ADV_HI_START_RPM10 1800
#define HALL_SPEED_ADV_HI_END_RPM10   3200
#define HALL_SPEED_ADV_HI_MAX_Q15     4096

/* UART debug prints. Set to 0 for normal use. */
#define HALL_DEBUG_ENABLE           0
#define HALL_DEBUG_EDGE_ONLY        0
#define HALL_DEBUG_START_STOP       0

/*
 * Startup tuning for the 24 V winding of maxon ECX SPEED 16 M.
 * These values are still conservative versus the motor's 2.11 A nominal
 * current, but much more suitable than the previous tiny bench values.
 */
#define START_ALIGN_IQ_Q15          2000
#define START_KICK_IQ_Q15           3200
#define START_KICK_ADV_ANGLE_Q15    3200
#define START_OL_IQ_Q15             2600
#define START_OL_STEP_INIT_Q15      6
#define START_OL_STEP_MAX_Q15       28
#define START_OL_STEP_RAMP_CYCLES   32
#define START_OL_EXIT_SPD_RPM       30
#define START_OL_MAX_CYCLES_CFG     2200

/*
 * Hall-only position control for the current 1 pole-pair motor.
 *
 * With 3 Hall sensors the raw mechanical position resolution is 6 sectors
 * per turn, so the practical target here is a stable coarse position servo
 * rather than a high-resolution servo axis.
 */
#define POSITION_LOOP_ENABLE        1
#define POSITION_COARSE_MODE        1
#define POSITION_CMD_SRC_POT        1
#define POSITION_HOLD_DIR           MOTOR_DIR
#define POSITION_REF_Q15_DEFAULT    0
#define POSITION_ERR_DEAD_Q15       1800
#define POSITION_ERR_RELEASE_Q15    2600
#define POSITION_SPEED_MAX_RPM      1200
#define POSITION_SPEED_MIN_RPM      180
#define POSITION_TORQUE_MIN_Q15     520
#define POSITION_HOLD_IQ_Q15        480
#define POSITION_MOVE_IQ_Q15        1600
#define POSITION_MOVE_ADV_Q15       5460
#define POSITION_MOVE_ENTER_Q15     3200
#define POSITION_MOVE_EXIT_Q15      1800
#define POSITION_COARSE_HOLD_IQ_Q15 220
#define POSITION_COARSE_HOLD_DEAD_Q15 4200
#define POSITION_COARSE_SETTLE_IQ_Q15 180
#define POSITION_COARSE_SETTLE_CYCLES 320
#define POSITION_SECTOR_DWELL_CYCLES 10
#define POSITION_KP_Q15             900
#define POSITION_KI_Q15             8
#define POSITION_POT_MIN_ADC        450
#define POSITION_POT_MAX_ADC        3950
#define POSITION_REF_FILTER_SHIFT   3
#define POSITION_SECTOR_COUNT       6

#endif
