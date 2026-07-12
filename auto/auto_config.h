#ifndef AUTO_CONFIG_H
#define AUTO_CONFIG_H

/* Timing */
#define AUTO_CONTROL_PERIOD_MS             (10u)
#define AUTO_START_SETTLE_MS               (1000u)
#define AUTO_ROUTE_TIMEOUT_MS              (0u)      /* 0 = disabled while tuning recorded-route replay. */

/* Sensor requirements.
 * IMU required. GPS/encoder optional for initial testing.
 * Enable GPS/encoder AFTER confirming they work on the bench.
 */
#define AUTO_REQUIRE_IMU                   (1u)
#define AUTO_REQUIRE_GPS                   (0u)
#define AUTO_REQUIRE_ENCODER               (1u)  /* 缂栫爜鍣ㄥ繀椤绘湁鏁堟墠鍏佽鑷姩椹鹃┒ */

#define AUTO_GPS_TIMEOUT_MS                (1200u)
#define AUTO_IMU_TIMEOUT_MS                (150u)
#define AUTO_ENCODER_TIMEOUT_MS            (250u)

/* Route tracking */
#define AUTO_LOOKAHEAD_DISTANCE_CM         (50.0f)   /* 鍑忓皬鍓嶈锛屾洿绱ц创褰曞埗璺嚎 */
#define AUTO_NAV_SEARCH_WINDOW_POINTS      (12u)
/* !! CRITICAL: WAYPOINT_REACHED_CM 蹇呴』 < RECORD_INTERVAL_CM锛屽惁鍒欒矾寰勭偣琚珛鍗宠烦杩?!! */
#define AUTO_WAYPOINT_REACHED_CM           (10.0f)  /* Must stay below AUTO_RECORD_INTERVAL_CM. */
#define AUTO_PROGRESS_MAX_CTE_CM           (35.0f)
#define AUTO_ODOM_PROGRESS_MAX_CTE_CM      (60.0f)
#define AUTO_ODOM_DONE_DISTANCE_CM         (70.0f)
#define AUTO_PROGRESS_ADVANCE_T            (1.15f)
#define AUTO_PROGRESS_INDEX_LEAD_LIMIT     (3u)
#define AUTO_NAV_MAX_HEADING_ERROR_DEG     (60.0f)
#define AUTO_CROSS_TRACK_HEADING_LOOKAHEAD_CM (90.0f)
#define AUTO_CROSS_TRACK_HEADING_LIMIT_DEG (50.0f)
#define AUTO_CROSS_TRACK_STEER_LIMIT       (35.0f)
#define AUTO_ROUTE_DONE_MARGIN_CM          (8.0f)
#define AUTO_DYNAMIC_ROUTE_ABORT_CTE_CM    (45.0f)
#define AUTO_DYNAMIC_ROUTE_ABORT_HEADING_DEG (100.0f)
#define AUTO_DYNAMIC_ROUTE_ABORT_HEADING_MIN_CTE_CM (25.0f)
#define AUTO_DYNAMIC_ROUTE_ABORT_CTE_CONFIRM_TICKS (15u)
#define AUTO_DYNAMIC_ROUTE_ABORT_CTE_HARD_CM (100.0f)
#define AUTO_DYNAMIC_ROUTE_ABORT_AFTER_CM  (20.0f)
#define AUTO_DYNAMIC_CROSS_TRACK_HEADING_LOOKAHEAD_CM (35.0f)
#define AUTO_DYNAMIC_CROSS_TRACK_HEADING_LIMIT_DEG (18.0f)
#define AUTO_DYNAMIC_CROSS_TRACK_STEER_LIMIT (18.0f)
#define AUTO_DYNAMIC_CROSS_TRACK_KP        (0.22f)
#define AUTO_DYNAMIC_TURN_BLEND_START_T    (0.75f)
#define AUTO_DYNAMIC_PREVIEW_HEADING_LIMIT_DEG (22.0f)
#define AUTO_DYNAMIC_PRETURN_START_CM      (18.0f)
#define AUTO_DYNAMIC_PRETURN_FULL_CM       (4.0f)
#define AUTO_DYNAMIC_TURN_DETECT_DEG       (12.0f)
#define AUTO_DYNAMIC_TURN_GAIN_START_DEG   (20.0f)
#define AUTO_DYNAMIC_TURN_HEADING_KP       (2.2f)
#define AUTO_DYNAMIC_TURN_ABORT_START_DEG  (25.0f)
#define AUTO_DYNAMIC_TURN_ABORT_CTE_CM     (65.0f)
#define AUTO_DYNAMIC_TURN_CONTEXT_CTE_CM   (85.0f)
#define AUTO_DYNAMIC_HEADING_LOOKAHEAD_CM  (15.0f)
#define AUTO_DYNAMIC_SEGMENT_REFINE_WINDOW_POINTS (6u)
#define AUTO_DYNAMIC_SEGMENT_HEADING_GATE_DEG (75.0f)
#define AUTO_DYNAMIC_SEGMENT_HEADING_PENALTY (8.0f)
#define AUTO_DYNAMIC_SEGMENT_INDEX_PENALTY_CM (8.0f)
#define AUTO_DYNAMIC_SEGMENT_ENDPOINT_PENALTY_CM (38.0f)
#define AUTO_DYNAMIC_STEER_FEEDFORWARD_LOOKAHEAD_CM (35.0f)
#define AUTO_DYNAMIC_STEER_FEEDFORWARD_GAIN (1.0f)
#define AUTO_DYNAMIC_STEER_FEEDFORWARD_DEAD_PERCENT (6.0f)
#define AUTO_DYNAMIC_STEER_FEEDFORWARD_MAX_PERCENT (90.0f)
#define AUTO_DYNAMIC_ROUTE_DONE_CTE_CM     (12.0f)
#define AUTO_DYNAMIC_ROUTE_DONE_HEADING_DEG (8.0f)
#define AUTO_DYNAMIC_ROUTE_FINISH_OVERRUN_CM (60.0f)
#define AUTO_DYNAMIC_ROUTE_FINISH_SPEED_PERCENT (18.0f)
#define AUTO_HEADING_KP                    (2.0f)   /* 鎻愰珮锛?.2涓嶅锛屽集閬撹浆鍚戝姏涓嶈冻 */
#define AUTO_STEER_DEAD_DEG                (1.5f)  /* 杩涗竴姝ュ噺灏忔鍖猴紝鎻愰珮鐏垫晱搴?*/
#define AUTO_SPEED_MIN_PERCENT             (15.0f)  /* 鏈€浣庨€熷害闂ㄩ檺锛氫綆浜庢鍊煎仠姝㈣緭鍑猴紝娑堥櫎鐢垫満鎶栨尟 */
#define AUTO_STEER_MIN_PERCENT             (0.0f)   /* 绂佺敤闂ㄦ锛氳鎵€鏈夎浆鍚戞寚浠ょ洿鎺ユ墽琛?*/
/* 鑷姩閲囩偣闂磋窛锛氭帹杞︾Щ鍔ㄨ秴杩囨璺濈鑷姩璁板綍涓€涓矾寰勭偣锛屾棤闇€鎸塊EY2 */
#define AUTO_RECORD_INTERVAL_CM            (20.0f)  /* 鍑忓皬闂磋窛锛氭洿澶氭洸绾跨粏鑺傦紙200鐐光啋40m锛?*/
#define AUTO_CROSS_TRACK_KP                (0.12f)   /* 妯悜绾犲亸宸茬鐢紙鍥炵幆璺嚎涓婁細寮曡捣鎸崱锛?*/
#define AUTO_MAX_STEER_PERCENT             (100.0f) /* 鍏ㄥ姏锛氶獙璇佺數鏈鸿兘鍚﹁浆鍔?*/

#define AUTO_RECORD_TURN_HEADING_DEG       (8.0f)
#define AUTO_RECORD_TURN_MIN_DELTA_CM      (4.0f)
#define AUTO_RECORD_MAX_SEGMENT_CM         (55.0f)

/* Speed command */
#define AUTO_CRUISE_SPEED_PERCENT          (25.0f)
#define AUTO_SLOW_SPEED_PERCENT            (25.0f)
#define AUTO_ALIGN_SPEED_PERCENT           (25.0f)
#define AUTO_REVERSE_SPEED_PERCENT         (-45.0f) /* 鍊掕溅閫熷害 */
#define AUTO_SUBJECT3_RETURN_REVERSE_SPEED_PERCENT (-50.0f)

/* Garage alignment. Heading convention is defined by the platform adapter:
 * 0 deg points along local +X, positive is counter-clockwise.
 */
#define AUTO_GARAGE_APPROACH_HEADING_DEG   (0.0f)
#define AUTO_GARAGE_ALIGN_TOL_DEG          (6.0f)
#define AUTO_GARAGE_ALIGN_TIMEOUT_MS       (6000u)

/* Parking scripted stages. Tune on the real car. */
#define AUTO_PARK_STAGE_MAX                (6u)
#define AUTO_PARK_REVERSE_TURN_CM          (140.0f)
#define AUTO_PARK_REVERSE_COUNTER_CM       (80.0f)
#define AUTO_PARK_REVERSE_FINAL_CM         (90.0f)
#define AUTO_PARK_STAGE_TIMEOUT_MS         (5000u)

/* Screen diagnostics.
 * Default is Seekfree IPS200 over SPI. Change AUTO_SCREEN_DEVICE if your car
 * uses IPS114, TFT180, or OLED.
 */
#define AUTO_SCREEN_DEVICE_NONE            (0u)
#define AUTO_SCREEN_DEVICE_IPS200_SPI      (1u)
#define AUTO_SCREEN_DEVICE_IPS200_PAR8     (2u)
#define AUTO_SCREEN_DEVICE_IPS114          (3u)
#define AUTO_SCREEN_DEVICE_TFT180          (4u)
#define AUTO_SCREEN_DEVICE_OLED            (5u)

#ifndef AUTO_SCREEN_DEVICE
#define AUTO_SCREEN_DEVICE                 AUTO_SCREEN_DEVICE_IPS200_SPI
#endif

#define AUTO_SCREEN_REFRESH_MS             (200u)
#define AUTO_SCREEN_CHAR_WIDTH             (8u)
#define AUTO_SCREEN_LINE_HEIGHT            (16u)
#define AUTO_SCREEN_MAX_TEXT_CHARS         (20u)
#define AUTO_ENABLE_UART_TELEMETRY         (0u)

/* Push-recorded route points kept in RAM before copying from UART logs. */
#define AUTO_ROUTE_RECORDER_MAX_POINTS     (400u)

/* Seekfree integration switches.
 * Hardware sensors enabled for Subject 1 navigation.
 * IMU963RA: 9-axis IMU with magnetometer for accurate heading
 * GN42A: Dual-frequency GNSS for precise positioning
 * Rear direction encoders: left TIM2 P33_7/P33_6, right TIM5 P10_3/P10_1
 */
#define AUTO_SEEKFREE_USE_IMU660RA         (0u)
#define AUTO_SEEKFREE_USE_IMU660RB         (0u)
#define AUTO_SEEKFREE_USE_IMU660RX         (0u)
#define AUTO_SEEKFREE_USE_IMU963RA         (1u)
#define AUTO_SEEKFREE_USE_GNSS             (0u)   /* 闅旂GPS铻嶅悎锛岄伩鍏嶅共鎵版儻瀵煎潗鏍?*/
#define AUTO_SEEKFREE_USE_ENCODER          (1u)

#define AUTO_SEEKFREE_DRIVE_PWM_MAX        (8000.0f)
#define AUTO_SEEKFREE_DIFF_STEER_ENABLE    (1u)
#define AUTO_SEEKFREE_DIFF_STEER_MAX_RATIO (0.40f)
#define AUTO_SEEKFREE_DIFF_STEER_START_PERCENT (5.0f)
#define AUTO_SEEKFREE_STEER_KP             (22.0f)
#define AUTO_SEEKFREE_STEER_KI             (0.0f)
#define AUTO_SEEKFREE_STEER_KD             (0.0f)
#define AUTO_SEEKFREE_STEER_I_LIMIT        (1000.0f)
#define AUTO_SEEKFREE_STEER_CLOSED_LOOP    (1u)     /* 1=steering ADC PID, 0=open-loop PWM */
#define AUTO_SEEKFREE_STEER_CLOSED_PWM_MIN (3600.0f)
#define AUTO_SEEKFREE_STEER_CLOSED_MIN_CMD_PERCENT (3.0f)

/* Dual rear direction encoder: CH1=count pulse, CH2=direction. */
#define AUTO_SEEKFREE_RIGHT_ENCODER_INDEX      (TIM5_ENCODER)
#define AUTO_SEEKFREE_RIGHT_ENCODER_COUNT_PIN  (TIM5_ENCODER_CH1_P10_3)
#define AUTO_SEEKFREE_RIGHT_ENCODER_DIR_PIN    (TIM5_ENCODER_CH2_P10_1)

#define AUTO_SEEKFREE_LEFT_ENCODER_INDEX       (TIM2_ENCODER)
#define AUTO_SEEKFREE_LEFT_ENCODER_COUNT_PIN   (TIM2_ENCODER_CH1_P33_7)
#define AUTO_SEEKFREE_LEFT_ENCODER_DIR_PIN     (TIM2_ENCODER_CH2_P33_6)

#define AUTO_SEEKFREE_ENCODER_USE_AVERAGE      (1u)
#define AUTO_SEEKFREE_RIGHT_ENCODER_SIGN       (-1.0f)
#define AUTO_SEEKFREE_LEFT_ENCODER_SIGN        (1.0f)

#define AUTO_SEEKFREE_ENCODER_CM_PER_COUNT (0.003f)  /* 宸叉牎鍑嗭細1m瀹炴祴=3302缂栫爜鍣╟m */

/* 闄€铻轰华姣斾緥鏍℃锛氬疄娴?0掳鏃嬭浆鍙姤鍛?4.5掳锛屾牎姝ｇ郴鏁?90/34.5鈮?.61 */
#define AUTO_SEEKFREE_REAR_TRACK_CM        (62.5f)
#define AUTO_SEEKFREE_WHEELBASE_CM         (62.5f)
#define AUTO_SEEKFREE_STEER_TOTAL_DEG      (58.0f)
#define AUTO_FUSION_MIN_DELTA_CM           (0.03f)
#define AUTO_FUSION_MAX_YAW_DELTA_DEG      (12.0f)
#define AUTO_FUSION_CORRECTION_GAIN        (0.0f)
#define AUTO_FUSION_CORRECTION_LIMIT_DEG   (0.80f)
#define AUTO_FUSION_AGREE_TOL_DEG          (8.0f)

#define AUTO_SEEKFREE_GYRO_SCALE           (1.03f)  /* 鏍℃锛氶檧铻轰华宸插熀鏈噯纭紝寰皟1.03 */
#define AUTO_SEEKFREE_GYRO_SIGN            (1.0f)
#define AUTO_SEEKFREE_GYRO_BIAS_SAMPLES    (120u)
#define AUTO_SEEKFREE_GYRO_BIAS_DELAY_MS   (5u)
#define AUTO_SEEKFREE_GNSS_TYPE            (GN42A)

/* Subject 1 inertial replay tuning.
 * These overrides keep the first real-car runs conservative:
 * - no local forward search, so progress cannot jump to the end;
 * - lower steering gain/output, because steering ADC feedback is unusable;
 * - slower speed while the inertial route follower is being verified.
 */
#undef AUTO_LOOKAHEAD_DISTANCE_CM
#define AUTO_LOOKAHEAD_DISTANCE_CM         (25.0f)
#define AUTO_NAV_LOOKAHEAD_DISTANCE_CM     (30.0f)
#undef AUTO_NAV_SEARCH_WINDOW_POINTS
#define AUTO_NAV_SEARCH_WINDOW_POINTS      (4u)
#undef AUTO_PROGRESS_INDEX_LEAD_LIMIT
#define AUTO_PROGRESS_INDEX_LEAD_LIMIT     (1u)
#undef AUTO_PROGRESS_MAX_CTE_CM
#define AUTO_PROGRESS_MAX_CTE_CM           (25.0f)
#undef AUTO_ODOM_PROGRESS_MAX_CTE_CM
#define AUTO_ODOM_PROGRESS_MAX_CTE_CM      (35.0f)
#undef AUTO_ODOM_DONE_DISTANCE_CM
#define AUTO_ODOM_DONE_DISTANCE_CM         (45.0f)
#undef AUTO_CROSS_TRACK_HEADING_LOOKAHEAD_CM
#define AUTO_CROSS_TRACK_HEADING_LOOKAHEAD_CM (60.0f)
#undef AUTO_CROSS_TRACK_HEADING_LIMIT_DEG
#define AUTO_CROSS_TRACK_HEADING_LIMIT_DEG (35.0f)
#undef AUTO_DYNAMIC_CROSS_TRACK_HEADING_LOOKAHEAD_CM
#define AUTO_DYNAMIC_CROSS_TRACK_HEADING_LOOKAHEAD_CM (55.0f)
#undef AUTO_DYNAMIC_CROSS_TRACK_HEADING_LIMIT_DEG
#define AUTO_DYNAMIC_CROSS_TRACK_HEADING_LIMIT_DEG (30.0f)
#undef AUTO_DYNAMIC_CROSS_TRACK_STEER_LIMIT
#define AUTO_DYNAMIC_CROSS_TRACK_STEER_LIMIT (30.0f)
#undef AUTO_DYNAMIC_ROUTE_ABORT_CTE_CM
#define AUTO_DYNAMIC_ROUTE_ABORT_CTE_CM    (55.0f)
#undef AUTO_DYNAMIC_ROUTE_ABORT_HEADING_DEG
#define AUTO_DYNAMIC_ROUTE_ABORT_HEADING_DEG (150.0f)
#undef AUTO_DYNAMIC_ROUTE_ABORT_AFTER_CM
#define AUTO_DYNAMIC_ROUTE_ABORT_AFTER_CM  (25.0f)
#undef AUTO_RECORD_INTERVAL_CM
#define AUTO_RECORD_INTERVAL_CM            (20.0f)
#undef AUTO_HEADING_KP
#define AUTO_HEADING_KP                    (1.5f)
#undef AUTO_STEER_DEAD_DEG
#define AUTO_STEER_DEAD_DEG                (2.0f)
#undef AUTO_MAX_STEER_PERCENT
#define AUTO_MAX_STEER_PERCENT             (100.0f)
#undef AUTO_CROSS_TRACK_KP
#define AUTO_CROSS_TRACK_KP                (0.12f)
#undef AUTO_CRUISE_SPEED_PERCENT
#define AUTO_CRUISE_SPEED_PERCENT          (25.0f)
#undef AUTO_SLOW_SPEED_PERCENT
#define AUTO_SLOW_SPEED_PERCENT            (25.0f)

/* Open-loop steering PWM. Steering ADC feedback is too small/noisy to close
 * the loop, so navigation command is mapped directly to a capped motor PWM.
 */
#define AUTO_SEEKFREE_STEER_PWM_MAX        (5000.0f)
#define AUTO_SEEKFREE_STEER_PWM_MIN        (1200.0f)

#endif
