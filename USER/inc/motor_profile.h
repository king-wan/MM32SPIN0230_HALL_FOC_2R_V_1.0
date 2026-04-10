#ifndef __MOTOR_PROFILE_H__
#define __MOTOR_PROFILE_H__

/*
 * Motor profile
 * -------------
 * Central place for motor-specific parameters and control tuning.
 *
 * For the same controller board, if you replace the motor, start by editing
 * only this file. Board-level analog front-end settings such as shunt value,
 * bus divider, and op-amp gain remain in other headers because they belong to
 * the hardware platform rather than the motor itself.
 *
 * Recommended order when replacing the motor:
 * 1. Update "Basic motor data" from the datasheet first.
 * 2. Update "Speed-mode command and ramp tuning" to match the new speed range
 *    and allowable current.
 * 3. Retune "Startup and open-loop takeover tuning" so the motor can start
 *    reliably from standstill.
 * 4. Retune "Hall angle and speed advance tuning" to improve current phase,
 *    efficiency, and high-speed smoothness.
 * 5. If position mode is used, retune the "Hall-only position control" block
 *    last, after speed mode already runs stably.
 *
 * File role:
 * - This is the first file to edit when changing to a different motor.
 * - For normal motor replacement, most parameter changes should stay here.
 * - If the motor changes but the controller board does not, start here and
 *   avoid touching other headers until necessary.
 *
 * Parameter classes used in this file:
 * [Required] Must be updated for a new motor before first power-up.
 *            These come from the datasheet, wiring direction check, or the
 *            intended speed/current limits of the application.
 * [Tuning]   Keep the existing values as a safe starting point, then retune
 *            on the bench after the new motor can already spin.
 *
 * New motor quick-fill template:
 * Copy the values below from the new motor datasheet before first power-up.
 *
 * Required template:
 *   POLEPAIRS                       = ?
 *   MOTOR_DIR                       = 1 or -1 (verify actual positive direction)
 *   MOTOR_NOMINAL_VOLTAGE_V         = ?
 *   MOTOR_NOMINAL_SPEED_RPM         = ?
 *   MOTOR_NOLOAD_SPEED_RPM          = ?
 *   MOTOR_NOMINAL_CURRENT_A10       = ?   (0.1 A unit, for example 2.1 A -> 21)
 *   MOTOR_TERMINAL_RESISTANCE_MOHM  = ?   (mOhm, for example 0.84 ohm -> 840)
 *   MOTOR_TERMINAL_INDUCTANCE_UH    = ?   (uH)
 *   MOTOR_SPEED_CONSTANT_RPM_PER_V  = ?
 *   MOTOR_TORQUE_CONSTANT_MNM_PER_A = ?   (0.1 mNm/A)
 *
 * First power-up conservative starting points:
 *   MOTOR_COMMAND_MAX_RPM10         = about 40% to 70% of rated speed
 *   SPEED_LOOP_MAX_CURRENT_A10      = about 30% to 60% of rated current
 *   RUN_MIN_REF_RPM                 = start small, then raise only if low-speed is weak
 *   RUN_MIN_CURRENT_A10             = start small, then raise only if low-speed is weak
 *   START_ALIGN_IQ_Q15              = start conservative, then raise until startup is repeatable
 *   START_KICK_IQ_Q15               = start conservative, then raise until startup is crisp
 *   START_OL_IQ_Q15                 = start conservative, then raise until takeover is stable
 *
 * Bench retuning order after first spin:
 *   1. Startup current / open-loop parameters
 *   2. Speed limit / ramp / minimum running torque
 *   3. Hall angle offset and speed advance
 *   4. Field weakening
 *   5. Position-mode parameters, only if needed
 *
 * Current project status for the present motor:
 * - Speed mode is the current working baseline.
 * - POSITION_LOOP_ENABLE is intentionally kept at 0 by default.
 * - Injected current sampling, dynamic CCR4 trigger, Hall speed advance,
 *   runtime current filtering, and idle offset tracking have been integrated
 *   into this baseline.
 *
 * Parameters that are already treated as the current baseline for this motor:
 * - Basic motor data block
 * - Speed-mode command/ramp block
 * - Startup and open-loop takeover block
 * - Current speed-loop operating limits used by normal running
 *
 * Parameters that are still the best candidates for future optimization:
 * - HALL_EANGLE_OFFSET_Q15
 * - HALL_SPEED_ADV_DELAY_US
 * - HALL_SPEED_ADV_MAX_Q15
 * - HALL_SPEED_ADV_HI_MAX_Q15
 * - FIELD_WEAKEN_START_RPM
 * - FIELD_WEAKEN_FULL_RPM
 * - FIELD_WEAKEN_MAX_ID_Q15
 * - Position-mode parameters, only when position mode is intentionally enabled
 *
 * Minimum modification checklist for a new motor:
 * [A] Must update from datasheet:
 *     POLEPAIRS
 *     MOTOR_DIR
 *     MOTOR_NOMINAL_VOLTAGE_V
 *     MOTOR_NOMINAL_SPEED_RPM
 *     MOTOR_NOLOAD_SPEED_RPM
 *     MOTOR_NOMINAL_CURRENT_A10
 *     MOTOR_TERMINAL_RESISTANCE_MOHM
 *     MOTOR_TERMINAL_INDUCTANCE_UH
 *     MOTOR_SPEED_CONSTANT_RPM_PER_V
 *     MOTOR_TORQUE_CONSTANT_MNM_PER_A
 * [B] Must check on first power-up:
 *     MOTOR_COMMAND_MAX_RPM10
 *     SPEED_LOOP_MAX_CURRENT_A10
 *     RUN_MIN_REF_RPM
 *     RUN_MIN_CURRENT_A10
 *     START_ALIGN_IQ_Q15
 *     START_KICK_IQ_Q15
 *     START_OL_IQ_Q15
 * [C] Usually retune after the motor can already run:
 *     HALL_EANGLE_OFFSET_Q15
 *     HALL_SPEED_ADV_DELAY_US
 *     HALL_SPEED_ADV_MAX_Q15
 *     FIELD_WEAKEN_START_RPM
 *     FIELD_WEAKEN_FULL_RPM
 *     FIELD_WEAKEN_MAX_ID_Q15
 * [D] Only retune if using position mode:
 *     POSITION_LOOP_ENABLE
 *     POSITION_HOLD_IQ_Q15
 *     POSITION_MOVE_IQ_Q15
 *     POSITION_ERR_DEAD_Q15
 *     POSITION_ERR_RELEASE_Q15
 *
 * Quick symptom-to-parameter map:
 * - Cannot start or only twitches:
 *   START_ALIGN_IQ_Q15, START_KICK_IQ_Q15, START_OL_IQ_Q15,
 *   START_OL_STEP_INIT_Q15, START_OL_STEP_MAX_Q15
 * - Starts but feels weak at low speed:
 *   SPEED_LOOP_MAX_CURRENT_A10, RUN_MIN_CURRENT_A10, RUN_MIN_REF_RPM
 * - High-speed roughness or poor efficiency:
 *   HALL_EANGLE_OFFSET_Q15, HALL_SPEED_ADV_DELAY_US,
 *   HALL_SPEED_ADV_MAX_Q15, HALL_SPEED_ADV_HI_MAX_Q15
 * - Top speed too low or too aggressive:
 *   MOTOR_COMMAND_MAX_RPM10, MOTOR_RAMP_INC_RPM10, MOTOR_RAMP_DEC_RPM10,
 *   FIELD_WEAKEN_START_RPM, FIELD_WEAKEN_FULL_RPM
 * - Stop/hold behavior feels poor:
 *   POS_HOLD_IQ_STOP, STOP_BRAKE_IQ, STOP_PARK_IQ
 * - Position mode jitters or stops inconsistently:
 *   POSITION_HOLD_IQ_Q15, POSITION_COARSE_HOLD_IQ_Q15,
 *   POSITION_COARSE_SETTLE_IQ_Q15, POSITION_ERR_DEAD_Q15
 *
 * Normally do not modify user_function.h or parameter.h when only the motor
 * changes. Touch those files only if the controller PCB, current sensing,
 * voltage divider, PWM frequency, or ADC filter strategy also changes.
 */

/* [Required] Basic motor data: always update these first when replacing the motor. */
#define POLEPAIRS                       1       /* Motor pole-pair count. */
#define MAX_SPEED                       4000    /* Internal speed limit, unit: 0.1 krpm. */
#define MOTOR_DIR                       -1      /* Positive rotation direction: 1=CW, -1=CCW. */

/*
 * [Required] Motor reference used for the current tuning pass:
 * maxon ECX SPEED 16 M, 24 V winding with Hall sensors.
 */
#define MOTOR_NOMINAL_VOLTAGE_V         24      /* Rated terminal voltage, unit: V. */
#define MOTOR_NOMINAL_SPEED_RPM         50300   /* Rated speed from datasheet, unit: rpm. */
#define MOTOR_NOLOAD_SPEED_RPM          54900   /* No-load speed from datasheet, unit: rpm. */
#define MOTOR_NOMINAL_CURRENT_A10       21      /* Rated continuous current, unit: 0.1 A. */
#define MOTOR_TERMINAL_RESISTANCE_MOHM  841     /* Phase/terminal resistance, unit: mOhm. */
#define MOTOR_TERMINAL_INDUCTANCE_UH    53      /* Phase/terminal inductance, unit: uH. */
#define MOTOR_SPEED_CONSTANT_RPM_PER_V  2300    /* Speed constant, unit: rpm/V. */
#define MOTOR_TORQUE_CONSTANT_MNM_PER_A 416     /* Torque constant, unit: 0.1 mNm/A. */

/* [Current baseline][Required + Tuning] Speed-mode limits must be checked first; ramp feel is then tuned on the bench. */
#define MOTOR_COMMAND_MAX_RPM10         3200    /* Potentiometer full-scale command, unit: 10 rpm. */
#define MOTOR_RAMP_INC_RPM10            4       /* Speed command ramp-up step per update, unit: 10 rpm. */
#define MOTOR_RAMP_DEC_RPM10            8       /* Speed command ramp-down step per update, unit: 10 rpm. */
#define SPEED_LOOP_MAX_CURRENT_A10      12      /* Speed-loop Iq limit, unit: 0.1 A. */
#define RUN_MIN_REF_RPM                 240     /* Minimum speed reference after startup, unit: 10 rpm. */
#define RUN_MIN_CURRENT_A10             2       /* Minimum running torque current, unit: 0.1 A. */
#define SPEED_START_THRESHOLD           120     /* Pot/speed command threshold to enter run, unit: 10 rpm. */
#define SPEED_STOP_THRESHOLD            80      /* Pot/speed command threshold to exit run, unit: 10 rpm. */
#define ZERO_REF_CLAMP_TH               40      /* Force reference to zero below this threshold, unit: 10 rpm. */

/* [Future optimization][Tuning] Speed loop field-weakening / d-axis current shaping: retune only after speed mode is already stable. */
#define IDREF_INIT_Q15                  (-1000) /* Default d-axis current reference at reset, unit: q15 current. */
#define FIELD_WEAKEN_START_RPM          500     /* Speed to start negative Id ramp, unit: 10 rpm. */
#define FIELD_WEAKEN_FULL_RPM           800     /* Speed to reach full negative Id, unit: 10 rpm. */
#define FIELD_WEAKEN_MAX_ID_Q15         350     /* Maximum field-weakening Id magnitude, unit: q15 current. */

/* [Future optimization][Tuning] Hall angle and speed advance tuning: retune after startup and base speed loop are working. */
#define HALL_EANGLE_OFFSET_Q15          (-6800) /* Static Hall electrical-angle offset, unit: q15 angle. */
#define HALL_SPEED_ADV_ENABLE           1       /* Enable speed-based Hall angle advance: 1=on, 0=off. */
#define HALL_SPEED_ADV_DELAY_US         500     /* Equivalent system delay used for lead compensation, unit: us. */
#define HALL_SPEED_ADV_MIN_RPM10        30      /* Minimum speed to begin Hall lead compensation, unit: 10 rpm. */
#define HALL_SPEED_ADV_MAX_Q15          2730    /* Normal-region Hall lead cap, unit: q15 angle. */
#define HALL_SPEED_ADV_HI_START_RPM10   1800    /* Speed where high-speed extra lead starts, unit: 10 rpm. */
#define HALL_SPEED_ADV_HI_END_RPM10     3200    /* Speed where high-speed extra lead reaches full scale, unit: 10 rpm. */
#define HALL_SPEED_ADV_HI_MAX_Q15       4096    /* High-speed Hall lead cap, unit: q15 angle. */

/* [Current baseline][Tuning] Startup and open-loop takeover tuning: key block for reliable start from standstill. */
#define START_ALIGN_IQ_Q15              2000    /* Alignment current before motion, unit: q15 current. */
#define START_KICK_IQ_Q15               3200    /* Initial kick current for breakaway, unit: q15 current. */
#define START_KICK_ADV_ANGLE_Q15        3200    /* Advance angle during kick, unit: q15 angle. */
#define START_OL_IQ_Q15                 2600    /* Open-loop startup current, unit: q15 current. */
#define START_OL_STEP_INIT_Q15          6       /* Initial open-loop angle increment, unit: q15 angle per PWM cycle. */
#define START_OL_STEP_MAX_Q15           28      /* Maximum open-loop angle increment, unit: q15 angle per PWM cycle. */
#define START_OL_STEP_RAMP_CYCLES       32      /* PWM cycles between open-loop ramp updates. */
#define START_OL_EXIT_SPD_RPM           30      /* Speed to exit open-loop startup, unit: 10 rpm. */
#define START_OL_MAX_CYCLES_CFG         2200    /* Timeout limit for open-loop startup, unit: PWM cycles. */
#define START_ALIGN_PWM_CYCLES          1200    /* Alignment hold time, unit: PWM cycles. */
#define START_LOCK_PWM_CYCLES           900     /* Rotor lock time after alignment, unit: PWM cycles. */

/* [Derived] Aliases used by the control sources: keep these synchronized with the startup block above. */
#define POS_HOLD_IQ_START               START_ALIGN_IQ_Q15        /* Alias: hold current used during startup alignment. */
#define START_KICK_IQ                   START_KICK_IQ_Q15         /* Alias: kick current used by MC_Drive. */
#define START_KICK_ADV_ANGLE            START_KICK_ADV_ANGLE_Q15  /* Alias: kick advance angle used by MC_Drive. */
#define START_OL_IQ                     START_OL_IQ_Q15           /* Alias: open-loop current used by MC_Drive. */
#define START_OL_STEP_INIT              START_OL_STEP_INIT_Q15    /* Alias: initial open-loop angle step. */
#define START_OL_STEP_MAX               START_OL_STEP_MAX_Q15     /* Alias: maximum open-loop angle step. */
#define START_OL_EXIT_SPD               START_OL_EXIT_SPD_RPM     /* Alias: open-loop exit speed threshold. */
#define START_OL_MAX_CYCLES             START_OL_MAX_CYCLES_CFG   /* Alias: open-loop timeout. */

/* [Current baseline][Tuning] Start/stop hold and braking tuning: adjust when start/stop feel or parking behavior is not ideal. */
#define POS_HOLD_ENTRY_SPD              40      /* Below this speed, allow stop-hold logic, unit: 10 rpm. */
#define POS_HOLD_IQ_STOP                220     /* Holding current after stop request, unit: q15 current. */
#define POS_HOLD_ANGLE_DEAD             900     /* Angle deadband for stop hold, unit: q15 angle. */
#define POS_HOLD_IQ_BRAKE               0       /* Extra braking current inside hold window, unit: q15 current. */
#define HOLD_TARGET_HALL                1       /* Default Hall sector used when state is invalid. */
#define HOLD_HALL_STABLE_CNT            8       /* Stable Hall samples required before hold engages. */
#define HOLD_TARGET_BIAS                900     /* Bias angle applied to hold target, unit: q15 angle. */
#define MOTOR_CMD_ACTIVE_TH             40      /* Command-active threshold used by idle logic, unit: 10 rpm. */
#define STOP_PWM_OFF_SPD                20      /* Speed below which PWM can be turned off, unit: 10 rpm. */
#define STOP_BRAKE_ENTRY_SPD            220     /* Speed to enter stop braking, unit: 10 rpm. */
#define STOP_BRAKE_EXIT_SPD             80      /* Speed to exit stop braking, unit: 10 rpm. */
#define STOP_BRAKE_IQ                   180     /* Active braking current during stop, unit: q15 current. */
#define STOP_BRAKE_MAX_CYCLES           700     /* Max braking duration, unit: PWM cycles. */
#define STOP_PARK_ENABLE                1       /* Enable stop-park alignment: 1=on, 0=off. */
#define STOP_PARK_DELAY_CYCLES          180     /* Delay before park alignment after stop, unit: PWM cycles. */
#define STOP_PARK_IQ                    70      /* Park alignment current after stop, unit: q15 current. */
#define STOP_PARK_ANGLE_DEAD            700     /* Angle deadband for park alignment, unit: q15 angle. */
#define STOP_PARK_ERR_HYST              120     /* Hysteresis for park error polarity flip, unit: q15 angle. */

/*
 * [Future optimization][Tuning] Hall-only position control for the current 1 pole-pair motor.
 *
 * With 3 Hall sensors the raw mechanical position resolution is 6 sectors
 * per turn, so the practical target here is a stable coarse position servo
 * rather than a high-resolution servo axis.
 *
 * Retune this block only after speed mode already runs well.
 */
#define POSITION_LOOP_ENABLE            0       /* Position mode master switch: 1=position, 0=speed. */
#define POSITION_COARSE_MODE            1       /* Use Hall-sector coarse positioning: 1=on, 0=continuous angle mode. */
#define POSITION_CMD_SRC_POT            1       /* Use potentiometer as position command source: 1=on. */
#define POSITION_HOLD_DIR               MOTOR_DIR /* Fixed direction reference used for coarse Hall positioning. */
#define POSITION_REF_Q15_DEFAULT        0       /* Default position reference after reset, unit: q15 angle. */
#define POSITION_ERR_DEAD_Q15           1800    /* Error band to declare position reached, unit: q15 angle. */
#define POSITION_ERR_RELEASE_Q15        2600    /* Error band to leave hold and resume movement, unit: q15 angle. */
#define POSITION_SPEED_MAX_RPM          1200    /* Max speed command in position mode, unit: 10 rpm. */
#define POSITION_SPEED_MIN_RPM          180     /* Minimum crawl speed in position mode, unit: 10 rpm. */
#define POSITION_TORQUE_MIN_Q15         520     /* Minimum torque while moving to target, unit: q15 current. */
#define POSITION_HOLD_IQ_Q15            480     /* Hold current when position is reached, unit: q15 current. */
#define POSITION_MOVE_IQ_Q15            1600    /* Movement current during position transitions, unit: q15 current. */
#define POSITION_MOVE_ADV_Q15           5460    /* Extra lead angle during position move, unit: q15 angle. */
#define POSITION_MOVE_ENTER_Q15         3200    /* Error threshold to enter move mode, unit: q15 angle. */
#define POSITION_MOVE_EXIT_Q15          1800    /* Error threshold to exit move mode, unit: q15 angle. */
#define POSITION_COARSE_HOLD_IQ_Q15     220     /* Light hold current inside target Hall sector, unit: q15 current. */
#define POSITION_COARSE_HOLD_DEAD_Q15   4200    /* Deadband for coarse Hall-sector hold, unit: q15 angle. */
#define POSITION_COARSE_SETTLE_IQ_Q15   180     /* Small settling current when first entering target sector, unit: q15 current. */
#define POSITION_COARSE_SETTLE_CYCLES   320     /* Duration of settling current, unit: PWM cycles. */
#define POSITION_SECTOR_DWELL_CYCLES    10      /* Minimum dwell before sector command updates, unit: 5 ms ticks. */
#define POSITION_KP_Q15                 900     /* Position PI proportional gain, unit: q15 gain. */
#define POSITION_KI_Q15                 8       /* Position PI integral gain, unit: q15 gain. */
#define POSITION_POT_MIN_ADC            450     /* Potentiometer ADC low endpoint used for normalization. */
#define POSITION_POT_MAX_ADC            3950    /* Potentiometer ADC high endpoint used for normalization. */
#define POSITION_REF_FILTER_SHIFT       3       /* Position reference IIR filter strength, larger means slower. */
#define POSITION_SECTOR_COUNT           6       /* Hall sectors per mechanical turn for this motor. */

#endif
