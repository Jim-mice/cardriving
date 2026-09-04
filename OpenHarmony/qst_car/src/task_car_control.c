#include <stdint.h>
#include <stdio.h>

#include "cmsis_os2.h"
#include "hi_time.h"
#include "app_time.h"
#include "line_sensor.h"
#include "task_car_control.h"
#include "task_hcsr04.h"
#include "task10_bluetooth.h"
#include "udp_telemetry.h"
#include "wifiiot_uart.h"
#include "task_encoder_cal.h"
#include "task_encoder_arc_roundtrip.h"
#include "task_encoder_right_arc_roundtrip.h"
#include "task_encoder_two_arc_roundtrip.h"
#include "task_encoder_segment_b_progress.h"
#include "task_encoder_bpath_follow.h"

#define CAR_IR_DIAGNOSTIC_MODE 0
#define CAR_TRACE_TEST_MODE    1
/* Keep AVOID opt-in until its first on-car validation; it never overlaps TRACE. */
#define CAR_AVOID_TEST_MODE    0
/* Competition V2: distinguish a double-marker finish from a single-marker dead end. */
#define TRACE_RACE_TEST_MODE   0
/* Logging only: it does not participate in any motor decision. */
#define TRACE_SHARP_TURN_DIAG_MODE 0
/* Isolated reverse-on-line experiment; it never enters the RACE/DEADEND flow. */
#define TRACE_REVERSE_TEST_MODE 0
/* Reverse V2 test: separate straight reverse translation from in-place alignment. */
#define TRACE_REVERSE_V2_TEST_MODE 0
/* Reverse V3 test: reverse translation with the yaw opposite to forward TRACE. */
#define TRACE_REVERSE_V3_TEST_MODE 0
#define TRACE_REVERSE_V4_TEST_MODE 0
#define TRACE_REVERSE_V5_TEST_MODE 0
#define TRACE_REVERSE_V6_TEST_MODE 0
#define TRACE_REVERSE_V7_TEST_MODE 0
#define TRACE_REVERSE_V8_TEST_MODE 0
/* V8.1 edge-lock code is retained below; V8.2 is the active simple servo. */
#define TRACE_REVERSE_V8_SIMPLE_SERVO_MODE 1
/* V8.4 paired-turn recenter experiment; V8.3 remains compiled as history only. */
#define TRACE_REVERSE_V8_PAIRED_RECENTER_MODE 0
/* V8.5 is the active attended straight-line micro paired reverse experiment. */
#define TRACE_REVERSE_V8_MICRO_PAIRED_MODE 1
/* V8.6 biased micro controller leaves a small net correction each cycle. */
#define TRACE_REVERSE_V8_BIASED_MICRO_MODE 1
/* Independent straight-branch trajectory-memory experiment. */
#define REVERSE_REPLAY_STRAIGHT_TEST_MODE 0
#define CROSS_AND_PROBE_TEST_MODE 0
#define ENCODER_PWM_CAL_TEST_MODE 0
#define ENCODER_DISTANCE_STRAIGHT_ROUNDTRIP_TEST_MODE 0
#define ENCODER_DISTANCE_LEFT_ARC_ROUNDTRIP_TEST_MODE 0
#define ENCODER_DISTANCE_RIGHT_ARC_ROUNDTRIP_TEST_MODE 0
#define ENCODER_TWO_ARC_CONTINUOUS_ROUNDTRIP_TEST_MODE 0
#define ENCODER_SEGMENT_B_PROGRESS_TEST_MODE 0
#define ENCODER_BPATH_FOLLOW_V1_TEST_MODE 1
/* Keep every encoder-only motor experiment free of HCSR04 servo scanning. */
#define ENCODER_ONLY_EXPERIMENT_MODE \
    ((ENCODER_PWM_CAL_TEST_MODE == 1) || \
     (ENCODER_DISTANCE_STRAIGHT_ROUNDTRIP_TEST_MODE == 1) || \
     (ENCODER_DISTANCE_LEFT_ARC_ROUNDTRIP_TEST_MODE == 1) || \
     (ENCODER_DISTANCE_RIGHT_ARC_ROUNDTRIP_TEST_MODE == 1) || \
     (ENCODER_TWO_ARC_CONTINUOUS_ROUNDTRIP_TEST_MODE == 1) || \
     (ENCODER_SEGMENT_B_PROGRESS_TEST_MODE == 1))
#define TRACE_TURNAROUND_TEST_MODE 0
#define CAR_LINE_CALIBRATION_MODE 0
#define CAR_STRAIGHT_DIAGNOSTIC_MODE 0
#define STM32_UART_LINK_TEST_MODE 0
#define LINE_SENSOR_SIDE_TEST_MODE 0
/* Temporary hardware diagnostics; keep both disabled for normal track running. */
#define LINE_SENSOR_ANALYSIS_MODE 0
#define LINE_SENSOR_CALIBRATION_MODE 0

#define CAR_CONTROL_PERIOD_MS  30U
#define TRACE_RECORD_DIAG_PERIOD_MS 200U
#define TRACE_LIVE_FORWARD_PERIOD_MS 60U
#define TRACE_LIVE_RETURN_PERIOD_MS  100U
#define LINE_LIVE_PERIOD_MS          60U
#define MOTOR_COMMAND_HEARTBEAT_MS 200U
#define MOTOR_HEARTBEAT_LOG_EVERY  5U
#define CAR_LINE_CALIBRATION_PERIOD_MS 100U
#define CAR_LINE_CALIBRATION_HEARTBEAT_MS 1000U
#define LINE_SENSOR_ANALYSIS_PERIOD_MS 10U
#define LINE_SENSOR_ANALYSIS_WINDOW_MS 2000U
#define LINE_SENSOR_CALIBRATION_PERIOD_MS 100U
#define TRACE_START_DELAY_MS   2000U
#define TRACE_RECOVERY_HOLD_MS 0U
#define TRACE_CORRECTION_MIN_HOLD_MS 180U
#define TRACE_BIAS_WINDOW_MS 1000U
#define TRACE_BIAS_LONG_CORRECTION_MS 300U
#define TRACE_BIAS_FWD_MIN_MS 300U
#define TRACE_BIAS_DOMINANCE_MS 180U
#define CROSSBAR_PRE_FWD_MS 180U
#define CROSSBAR_MIN_BLACK_MS 50U
#define CROSSBAR_MAX_EVENT_MS 500U
#define CROSSBAR_COOLDOWN_MS 300U
#define LONG_11_MIN_MS 120U
#define LONG_11_COOLDOWN_MS 300U
#define RIGHT_STOPLINE_CONTEXT_MAX_AGE_MS 120U
#define RIGHT_STOPLINE_COOLDOWN_MS 120U
/* Temporary integration mode: classify separated raw-11 episodes on this route. */
#define DOUBLE_STOP_AUTO_RETURN_TEST_MODE 1
/*
 * Route-level provisional boundary.  After the first crossbar has cleared and
 * normal FWD has resumed, the next confirmed 10/LEFT ends the second-line
 * search as SINGLE.  Replace this with calibrated encoder distance later.
 */
#define PROVISIONAL_ROUTE_BOUNDARY_TEST_MODE 1
/*
 * Keep a physically wide first crossbar episode distinct from the much
 * shorter local association window after that crossbar has cleared.  The
 * latter prevents an unrelated fork raw-11 from remaining eligible for a
 * distant route-boundary SINGLE_RETURN.
 */
#define STOPLINE_FIRST_11_EPISODE_MAX_MS 900U
#define STOPLINE_POST_FIRST_MAX_GAP_MS   2800U
/* Neutral qualified stable-11 relation window; independent from stopline tuning. */
#define ELEVEN_EVENT_LIVE_MS             3000U
#define ELEVEN_QUALIFY_MIN_MS                0U
/* BPATH stores 512 forward points; retain a bounded forensic subset. */
#define ELEVEN_EVENT_HISTORY_CAPACITY      64U
/* Temporary semantic experiment; legacy stopline remains telemetry-only. */
#define THREE_ELEVEN_SEQUENCE_TEST_MODE     1
/* Deprecated compatibility/debug value: V2 never expires a lone E1. */
#define SEQ11_FIRST_WAIT_MAX_MS          3500U
#define SEQ11_THIRD_WAIT_MAX_MS          2000U
#define SEQ11_LONG_GAP_MS                1500U
#define SEQ11_FINISH_GAP_RATIO_NUM          3U
#define SEQ11_FINISH_GAP_RATIO_DEN          2U
/* Terminal paint is a pair of distinct physical stable-11 segments.  It is
 * deliberately independent of 240 ms semantic ElevenEvent qualification. */
#define CLOSE_PAIR_SEGMENT_MIN_STABLE_SAMPLES 2U
#define CLOSE_PAIR_TERMINAL_GAP_MAX_MS        SEQ11_LONG_GAP_MS
/* Temporary competition calibration from physical terminal samples: a fork
 * pair measured about 769 counts and the real terminal about 1233 counts. */
#define TERMINAL_PAIR_MIN_TRAVEL_COUNTS       1000U
/* No counts-to-mm calibration exists in this project.  Keep the stale-E1
 * mechanism disabled rather than misrepresenting raw encoder counts as the
 * requested 250 mm physical clearance.  A calibrated nonzero count value may
 * enable it later, with its calibration record kept alongside this constant. */
#define SEQ11_SINGLE_STALE_ENCODER_TRAVEL 0U
/*
 * Calibration required: max encoder travel from crossbar #1 exit to crossbar
 * #2 entry.  Zero deliberately disables this optional encoder-distance path;
 * the provisional route LEFT boundary remains the active SINGLE path.
 */
#define SECOND_STOP_LOOKAHEAD_TICKS 0U
#define FORK_BACKOFF_SAMPLES 12U
#define FORK_SELECT_MAX_MS 1200U
/* Attended pre-E1 curve replay; isolated from ordinary TRACE semantics. */
#define REENTRY_CURVE_EDGE_SKIP_SAMPLES 3U
#define REENTRY_CURVE_LOOKBACK_SAMPLES 16U
#define REENTRY_CURVE_MIN_SAMPLES 8U
#define REENTRY_CURVE_MIN_RATIO_PERCENT 10U
#define REENTRY_CAPTURE_STABLE_COUNT 2U
#define REENTRY_APPROACH_TIMEOUT_MS 2000U
#define REENTRY_ANCHOR_SETTLE_STABLE_SAMPLES 3U
#define REENTRY_ANCHOR_SETTLE_MAX_DELTA 3U
#define REENTRY_ANCHOR_SETTLE_TIMEOUT_MS 1000U
#define REENTRY_FORCE_TIMEOUT_MS 1200U
/* A LEFT history replay owns the recovery motor for this minimum interval.
 * Its separate hard limit preserves the bounded fallback. */
#define REENTRY_LEFT_REPLAY_MIN_MS 1200U
#define REENTRY_LEFT_REPLAY_HARD_TIMEOUT_MS 1800U
#define REENTRY_HISTORY_TRUST_ADVANCES 4U
#define REENTRY_NO_HISTORY_DEPART_ADVANCES 10U
#define REENTRY_STRAIGHT_TRUST_ADVANCES 4U
#define REENTRY_STRAIGHT_ERROR_RATIO_PERCENT 3U
#define REENTRY_SWEEP_SCALE_PERCENT 100U
#define REENTRY_SWEEP_LOBE_ADVANCES 6U
#define REENTRY_SWEEP_MAX_LOBES 8U
/* LEFT active search deliberately retains a net left spatial bias while
 * remaining bounded and reversible through the scratch BPATH epoch. */
#define REENTRY_LEFT_SWEEP_LEFT_ADVANCES 8U
#define REENTRY_LEFT_SWEEP_RIGHT_ADVANCES 5U
#define REENTRY_REEXIT_MAX_ADVANCES REENTRY_NO_HISTORY_DEPART_ADVANCES

#define TRACE_FORWARD_SPEED    100
#define TRACE_SLOW_PWM         75
#define TRACE_FAST_PWM         125
#define TRACE_OUTER_SPEED      TRACE_FAST_PWM
#define TRACE_INNER_SPEED      TRACE_SLOW_PWM
#define TRACE_RECOVER_SPEED    120
#define TRACE_REVERSE_FORWARD_SPEED 100
#define TRACE_REVERSE_INNER_SPEED   60
#define TRACE_REVERSE_OUTER_SPEED   120
#define TRACE_REVERSE_DEBUG_HEARTBEAT_MS 100U
#define REV2_BACK_SPEED             100
#define REV2_ALIGN_SPEED             90
#define REV2_ALIGN_CLEAR_SAMPLES     2U
#define REV2_ALIGN_SETTLE_MS         60U
#define REV2_ALIGN_TIMEOUT_MS        500U
#define REV2_DEBUG_HEARTBEAT_MS      100U
#define REV3_BACK_SPEED             100
#define REV3_INNER_SPEED             60
#define REV3_OUTER_SPEED            120
#define REV3_DEBUG_HEARTBEAT_MS     100U
#define REV4_BACK_SPEED              100
#define REV4_INNER_SPEED              60
#define REV4_OUTER_SPEED             120
#define REV4_CORRECT_PULSE_MS        100U
#define REV4_PROBE_BACK_MS           150U
#define REV4_MAX_SAME_SIDE_PULSES      4U
#define REV4_DEBUG_HEARTBEAT_MS      100U
#define V5_SHIFT_YAW_OUT_MS          80U
#define V5_SHIFT_BACK_MS             250U
#define V5_SHIFT_YAW_BACK_MS         80U
#define V5_RECHECK_BACK_MS           120U
#define V5_MAX_LATERAL_SHIFTS        2U
#define HEADING_PULSE_MAX_MS         100U
#define V6_SHIFT_YAW_OUT_MAX_MS      80U
#define V6_SHIFT_BACK_MAX_MS         1500U
#define V6_VERIFY_BACK_MS            150U
#define V7_SWEEP_TO_OPPOSITE_MAX_MS  1500U
#define V7_SWEEP_ENTRY_YAW_MS        120U
#define V7_VERIFY_CENTER_MS          150U
#define V8_EDGE_ENTRY_MS               90U
#define V8_EDGE_LOCK_CONFIRM_MS       300U
#define V8_EDGE_GAP_GRACE_MS          300U
#define V8_EDGE_CORRECTION_MS          30U
#define V8_DEBUG_HEARTBEAT_MS         100U
#define REPLAY_FRAME_CAPACITY         1200U
#define REPLAY_TURNAROUND_SETTLE_MS    300U
#define REPLAY_DEBUG_HEARTBEAT_MS      100U
#define PROBE_CROSS_11_MAX_MS          300U
#define PROBE_FORWARD_GUARD_MS         120U
#define PROBE_SCAN_LEFT_MS              90U
#define PROBE_SCAN_RIGHT_MS            180U
#define PROBE_SCAN_RESTORE_LEFT_MS      90U
#define REPLAY_MARKER_SIGNED_PREAMBLE_MAX_MS 120U
#define REPLAY_MARKER_CONFIRM_FORWARD_MS    120U
#define REV4_ARM_CLEAR_MS            300U
#define REV4_REARM_CLEAR_MS          500U
#define TRACE_MARKER_CONFIRM_SAMPLES 2U
#define TRACE_MARKER_CLEAR_SAMPLES   2U
#define TRACE_RACE_HEARTBEAT_MS      500U
#define TRACE_RACE_DEBUG_HEARTBEAT_MS 100U
#define TRACE_MARKER_PROBE_SPEED      90
#define TRACE_DOUBLE_MARKER_WINDOW_MS 1500U
#define TRACE_FINISH_SLOW_SPEED       90
#define TRACE_FINISH_SLOW_MS          150U
#define TRACE_DIAG_HEARTBEAT_MS       100U
#define TRACE_DIAG_HISTORY_SAMPLES    67U
#define TRACE_DIAG_LOST_LOOKBACK_MS   300U
#define TRACE_DIAG_LOST_HOLD_MS       180U
#define REENTRY_LEFT_EXPLORE_MIN_MS   3000U
#define REENTRY_LEFT_EXPLORE_PWM_L    80
#define REENTRY_LEFT_EXPLORE_PWM_R    120
#define REENTRY_BIAS_MS               1200U
#define REENTRY_MICRO_STRAIGHT_MS     300U
#define REENTRY_SENSOR_ACQUIRE_MS     2000U
static const uint8_t g_reentryWheelPairs[11][2] __attribute__((unused)) = {
    {90U,110U},{92U,108U},{94U,106U},{96U,104U},{98U,102U},{100U,100U},
    {102U,98U},{104U,96U},{106U,94U},{108U,92U},{110U,90U}
};
#define AVOID_FRONT_STOP_CM    35.0f
#define AVOID_SIDE_CLEAR_CM    30.0f
#define AVOID_DISTANCE_STALE_MS 3000U
#define AVOID_FORWARD_SPEED    100
#define AVOID_TURN_INNER_SPEED 60
#define AVOID_TURN_OUTER_SPEED 120
#define BLE_DIAG_STAT_PERIOD_MS 1000U

extern void car_stop(void);
extern void stm32motor_control(int motorA, int motorB);

static volatile CarMode g_carMode = CAR_MODE_IDLE;
static int g_carControlTaskStarted;

/*
 * LINE_LIVE is deliberately independent from the TRACE debounce and every
 * BPATH lifecycle state.  It remains a live view of GPIO13/GPIO14 after the
 * vehicle has stopped, so manual track checks never depend on motor state.
 */
static uint8_t g_lineLiveRawLeft;
static uint8_t g_lineLiveRawRight;
static uint8_t g_lineLiveCandidateState;
static uint8_t g_lineLiveCandidateSamples;
static uint8_t g_lineLiveStableState;
static uint8_t g_lineLiveStableValid;
static uint32_t g_lineLiveLastTick;
static uint32_t g_lineLiveSequence;
static uint32_t g_lineLiveQueueFailCount;
static UdpEncoderTelemetryState g_lineLiveEncoder;
static uint16_t g_lineLiveBpathRecordCount;
static uint16_t g_lineLiveBpathRecordCapacity;
static const char *g_lineLivePhase = "BOOT";
static const char *g_lineLiveAction = "STOP";
static int g_lineLiveMotorLeft;
static int g_lineLiveMotorRight;
static const char *g_lineLiveBias = "NONE";
static uint32_t g_lineLiveBiasLeftMs;
static uint32_t g_lineLiveBiasRightMs;
static uint32_t g_lineLiveBiasFwdMs;
static const char *g_lineLiveCorrection = "NONE";
static uint32_t g_lineLiveCorrectionStartTick;
static const char *g_lineLiveRightStopState = "IDLE";
static uint32_t g_lineLiveRightStopContextAgeMs;
static uint32_t g_lineLiveRightStopEntryCount;
static uint32_t g_lineLiveRightStopCandidateCount;
static uint32_t g_lineLiveRightStop11Ms;
static uint32_t g_lineLiveRightStopLast11Ms;
static char g_lineLiveRightStopLastExitRaw[3] = "--";
static const char *g_lineLiveStopClassState = "WAIT_FIRST";
static uint32_t g_lineLiveStopClassFirstCount;
static uint32_t g_lineLiveStopClassSecondCount;
static uint32_t g_lineLiveStopClassSingleCount;
static uint32_t g_lineLiveStopClassDoubleCount;
static uint32_t g_lineLiveStopClassExpireCount;
static const char *g_lineLiveStopClassSingleReason = "NONE";
static uint32_t g_lineLiveStopClassCandidateAgeMs;
static uint32_t g_lineLiveStopClassCancelCount;
static const char *g_lineLiveStopClassCancelReason = "NONE";
static uint32_t g_lineLiveStopClassProbeTicks;
static uint8_t g_lineLiveStopClassProbeArmed;
static uint32_t g_lineLiveStopClassLeftBoundaryCount;
static int32_t g_lineLiveStopClassFirstExitLeft;
static int32_t g_lineLiveStopClassFirstExitRight;
static int32_t g_lineLiveStopClassSecondEnterLeft;
static int32_t g_lineLiveStopClassSecondEnterRight;
static const char *g_lineLiveStopClassDecision = "NONE";
static uint32_t g_lineLiveElevenActiveId;
static uint8_t g_lineLiveElevenActive;
static uint32_t g_lineLiveElevenCount;
static uint32_t g_lineLiveElevenHistoryCount;
static uint32_t g_lineLiveElevenDropCount;
static const char *g_lineLiveStopClaim = "NONE";
static const char *g_lineLiveForkClaim = "NONE";
static const char *g_lineLiveSeq11State = "IDLE";
static uint32_t g_lineLiveSeq11E1;
static uint32_t g_lineLiveSeq11E2;
static uint32_t g_lineLiveSeq11E3;
static uint32_t g_lineLiveSeq11Gap12Ms;
static uint32_t g_lineLiveSeq11Gap23Ms;
static const char *g_lineLiveSeq11Claim = "NONE";
static const char *g_lineLiveSeq11Decision = "NONE";
static uint32_t g_lineLiveSeq11ForkEventId;
static uint16_t g_lineLiveSeq11ForkPathIndex;
/* TaskCarControl owns LINE_LIVE formatting; static storage preserves its
 * 4096-byte task stack while retaining every sticky classifier field. */
static char g_lineLiveText[1536];
static const char *ForkTestDirectionName(uint8_t direction);
static const char *ForkTestStateName(int state);
static const char *g_lineLiveForkState = "IDLE";
static uint8_t g_lineLiveForkMarkValid;
static const char *g_lineLiveForkTaken = "NONE";
static const char *g_lineLiveForkDesired = "NONE";
static uint16_t g_lineLiveForkRecordIndex;
static uint16_t g_lineLiveForkReturnCursor;
static uint8_t g_lineLiveForkMarkReached;
static uint16_t g_lineLiveForkBackoffProgress;
static uint8_t g_lineLiveForkReady;
static uint32_t g_lineLiveForkSelectMs;
static uint32_t g_lineLiveForkCaptureCount;
static const char *g_lineLiveForkResult = "NONE";

static void LineLiveSetControl(const char *phase, const char *action,
                               int motorLeft, int motorRight)
{
    g_lineLivePhase = phase;
    g_lineLiveAction = action;
    g_lineLiveMotorLeft = motorLeft;
    g_lineLiveMotorRight = motorRight;
}

static void LineLiveSetCorrection(const char *direction, uint32_t now)
{
    g_lineLiveCorrection = direction;
    g_lineLiveCorrectionStartTick = now;
}

static void LineLiveUpdateSensor(WifiIotGpioValue left, WifiIotGpioValue right,
                                 int sensorValid)
{
    uint8_t rawState;

    if (sensorValid == 0) {
        g_lineLiveStableValid = 0U;
        g_lineLiveCandidateSamples = 0U;
        return;
    }
    g_lineLiveRawLeft = left == WIFI_IOT_GPIO_VALUE1 ? 1U : 0U;
    g_lineLiveRawRight = right == WIFI_IOT_GPIO_VALUE1 ? 1U : 0U;
    rawState = (uint8_t)((g_lineLiveRawLeft << 1) | g_lineLiveRawRight);
    if (rawState != g_lineLiveCandidateState) {
        g_lineLiveCandidateState = rawState;
        g_lineLiveCandidateSamples = 1U;
    } else if (g_lineLiveCandidateSamples < 2U) {
        g_lineLiveCandidateSamples++;
    }
    if (g_lineLiveCandidateSamples >= 2U) {
        g_lineLiveStableState = g_lineLiveCandidateState;
        g_lineLiveStableValid = 1U;
    }
}

static void LineLivePublish(uint32_t now)
{
    const char *sensor = "--";

    if ((uint32_t)(now - g_lineLiveLastTick) < AppMsToTicks(LINE_LIVE_PERIOD_MS)) {
        return;
    }
    if (g_lineLiveStableValid != 0U) {
        static char sensorText[3];

        sensorText[0] = (g_lineLiveStableState & 0x02U) != 0U ? '1' : '0';
        sensorText[1] = (g_lineLiveStableState & 0x01U) != 0U ? '1' : '0';
        sensorText[2] = '\0';
        sensor = sensorText;
    }
    /* Shared encoder telemetry snapshot only; this does not read hardware. */
    UdpTelemetryReadEncoder(&g_lineLiveEncoder);
    BPathExternalGetForwardRecordProgress(&g_lineLiveBpathRecordCount,
                                          &g_lineLiveBpathRecordCapacity);
    g_lineLiveSequence++;
    (void)snprintf(g_lineLiveText, sizeof(g_lineLiveText),
        "LINE_LIVE ms=%u seq=%u raw_l=%u raw_r=%u sensor=%s action=%s phase=%s motor_l=%d motor_r=%d corr_dir=%s corr_ms=%u enc_l=%ld enc_r=%ld bpath_rec_count=%u bpath_rec_cap=%u bias=%s bias_left_ms=%u bias_right_ms=%u bias_fwd_ms=%u queue_fail=%u 11evt_active_id=%u 11evt_active=%u 11evt_count=%u 11evt_history_count=%u 11evt_drop_count=%u stopclaim=%s forkclaim=%s seq11_state=%s seq11_e1=%u seq11_e2=%u seq11_e3=%u seq11_gap12_ms=%u seq11_gap23_ms=%u seq11_claim=%s seq11_decision=%s seq11_fork_event_id=%u seq11_fork_path_index=%u rstop_state=%s rstop_ctx_age_ms=%u rstop_entry_count=%u rstop_candidate_count=%u rstop_11_ms=%u rstop_last_11_ms=%u rstop_last_exit_raw=%s stopcls_state=%s stopcls_first_count=%u stopcls_second_count=%u stopcls_single_count=%u stopcls_double_count=%u stopcls_expire_count=%u stopcls_single_reason=%s stopcls_candidate_age_ms=%u stopcls_cancel_count=%u stopcls_cancel_reason=%s stopcls_probe_ticks=%u stopcls_probe_limit_ticks=%u stopcls_probe_armed=%u stopcls_left_boundary_count=%u stopcls_first_exit_enc_l=%ld stopcls_first_exit_enc_r=%ld stopcls_second_enter_enc_l=%ld stopcls_second_enter_enc_r=%ld stopcls_decision=%s forktest_state=%s forktest_mark_valid=%u forktest_taken=%s forktest_desired=%s forktest_record_index=%u forktest_return_cursor=%u forktest_mark_reached=%u forktest_backoff_progress=%u forktest_ready=%u forktest_select_ms=%u forktest_capture_count=%u forktest_result=%s",
        (unsigned int)AppTicksToMs(now), (unsigned int)g_lineLiveSequence,
        (unsigned int)g_lineLiveRawLeft, (unsigned int)g_lineLiveRawRight,
        sensor, g_lineLiveAction, g_lineLivePhase,
        g_lineLiveMotorLeft, g_lineLiveMotorRight, g_lineLiveCorrection,
        g_lineLiveCorrection[0] == 'N' ? 0U : (unsigned int)AppTicksToMs(now - g_lineLiveCorrectionStartTick),
        (long)g_lineLiveEncoder.totalLeft, (long)g_lineLiveEncoder.totalRight,
        (unsigned int)g_lineLiveBpathRecordCount,
        (unsigned int)g_lineLiveBpathRecordCapacity,
        g_lineLiveBias, (unsigned int)g_lineLiveBiasLeftMs,
        (unsigned int)g_lineLiveBiasRightMs, (unsigned int)g_lineLiveBiasFwdMs,
        (unsigned int)g_lineLiveQueueFailCount,
        (unsigned int)g_lineLiveElevenActiveId,
        (unsigned int)g_lineLiveElevenActive,
        (unsigned int)g_lineLiveElevenCount,
        (unsigned int)g_lineLiveElevenHistoryCount,
        (unsigned int)g_lineLiveElevenDropCount,
        g_lineLiveStopClaim, g_lineLiveForkClaim,
        g_lineLiveSeq11State,
        (unsigned int)g_lineLiveSeq11E1, (unsigned int)g_lineLiveSeq11E2,
        (unsigned int)g_lineLiveSeq11E3,
        (unsigned int)g_lineLiveSeq11Gap12Ms, (unsigned int)g_lineLiveSeq11Gap23Ms,
        g_lineLiveSeq11Claim, g_lineLiveSeq11Decision,
        (unsigned int)g_lineLiveSeq11ForkEventId,
        (unsigned int)g_lineLiveSeq11ForkPathIndex,
        g_lineLiveRightStopState,
        (unsigned int)g_lineLiveRightStopContextAgeMs,
        (unsigned int)g_lineLiveRightStopEntryCount,
        (unsigned int)g_lineLiveRightStopCandidateCount,
        (unsigned int)g_lineLiveRightStop11Ms,
        (unsigned int)g_lineLiveRightStopLast11Ms,
        g_lineLiveRightStopLastExitRaw, g_lineLiveStopClassState,
        (unsigned int)g_lineLiveStopClassFirstCount,
        (unsigned int)g_lineLiveStopClassSecondCount,
        (unsigned int)g_lineLiveStopClassSingleCount,
        (unsigned int)g_lineLiveStopClassDoubleCount,
        (unsigned int)g_lineLiveStopClassExpireCount,
        g_lineLiveStopClassSingleReason,
        (unsigned int)g_lineLiveStopClassCandidateAgeMs,
        (unsigned int)g_lineLiveStopClassCancelCount,
        g_lineLiveStopClassCancelReason,
        (unsigned int)g_lineLiveStopClassProbeTicks,
        (unsigned int)SECOND_STOP_LOOKAHEAD_TICKS,
        (unsigned int)g_lineLiveStopClassProbeArmed,
        (unsigned int)g_lineLiveStopClassLeftBoundaryCount,
        (long)g_lineLiveStopClassFirstExitLeft,
        (long)g_lineLiveStopClassFirstExitRight,
        (long)g_lineLiveStopClassSecondEnterLeft,
        (long)g_lineLiveStopClassSecondEnterRight,
        g_lineLiveStopClassDecision,
        g_lineLiveForkState, (unsigned int)g_lineLiveForkMarkValid,
        g_lineLiveForkTaken, g_lineLiveForkDesired,
        (unsigned int)g_lineLiveForkRecordIndex, (unsigned int)g_lineLiveForkReturnCursor,
        (unsigned int)g_lineLiveForkMarkReached, (unsigned int)g_lineLiveForkBackoffProgress,
        (unsigned int)g_lineLiveForkReady, (unsigned int)g_lineLiveForkSelectMs,
        (unsigned int)g_lineLiveForkCaptureCount, g_lineLiveForkResult);
    if (UdpTelemetryQueueExperimentText(g_lineLiveText) != 0) {
        g_lineLiveQueueFailCount++;
    }
    g_lineLiveLastTick = now;
}

#if (LINE_SENSOR_SIDE_TEST_MODE == 1) || (TRACE_TRACK_GEOMETRY_TEST_MODE == 1)
/* Static tests own only an unconditional safe stop from CarControlTask. */
static void TraceStaticTestForceStop(void)
{
    stm32motor_control(0, 0);
    UdpTelemetryUpdateMotorCommand(0, 0);
}
#endif

#if (MOTOR_RESPONSE_TEST_MODE == 1)
static volatile MotorResponseCommand g_motorResponsePendingCommand;
#endif

#if (MOTOR_TURN_DIRECTION_TEST_MODE == 1)
#define MOTOR_TURN_DIRECTION_DURATION_MS 600U
static volatile MotorTurnDirectionCommand g_motorTurnDirectionPendingCommand;
static MotorTurnDirectionCommand g_motorTurnDirectionActiveCommand;
static uint32_t g_motorTurnDirectionStopTick;

static const char *MotorTurnDirectionActionName(MotorTurnDirectionCommand command)
{
    if (command == MOTOR_TURN_DIRECTION_COMMAND_LEFT) return "LEFT";
    if (command == MOTOR_TURN_DIRECTION_COMMAND_RIGHT) return "RIGHT";
    if (command == MOTOR_TURN_DIRECTION_COMMAND_FORWARD) return "FWD";
    return "STOP";
}

static void MotorTurnDirectionPublish(const char *event,
                                      MotorTurnDirectionCommand command,
                                      int leftCommand, int rightCommand)
{
    char text[112];

    (void)snprintf(text, sizeof(text),
        "TURNTEST event=%s action=%s cmd_l=%d cmd_r=%d",
        event, MotorTurnDirectionActionName(command), leftCommand, rightCommand);
    (void)UdpTelemetryQueueExperimentText(text);
}

static void MotorTurnDirectionStep(uint32_t now, MotorTurnDirectionCommand command,
                                   int *leftCommand, int *rightCommand)
{
    *leftCommand = 0;
    *rightCommand = 0;

    if (command == MOTOR_TURN_DIRECTION_COMMAND_STOP) {
        if (g_motorTurnDirectionActiveCommand != MOTOR_TURN_DIRECTION_COMMAND_NONE) {
            MotorTurnDirectionPublish("STOP", g_motorTurnDirectionActiveCommand, 0, 0);
        }
        g_motorTurnDirectionActiveCommand = MOTOR_TURN_DIRECTION_COMMAND_NONE;
        return;
    }

    if (g_motorTurnDirectionActiveCommand == MOTOR_TURN_DIRECTION_COMMAND_NONE &&
        command != MOTOR_TURN_DIRECTION_COMMAND_NONE) {
        int startLeft = 0;
        int startRight = 0;

        if (command == MOTOR_TURN_DIRECTION_COMMAND_LEFT) {
            startLeft = 90;
            startRight = 110;
        } else if (command == MOTOR_TURN_DIRECTION_COMMAND_RIGHT) {
            startLeft = 110;
            startRight = 90;
        } else if (command == MOTOR_TURN_DIRECTION_COMMAND_FORWARD) {
            startLeft = 100;
            startRight = 100;
        }
        if (startLeft != 0 || startRight != 0) {
            g_motorTurnDirectionActiveCommand = command;
            g_motorTurnDirectionStopTick = now +
                AppMsToTicks(MOTOR_TURN_DIRECTION_DURATION_MS);
            MotorTurnDirectionPublish("START", command, startLeft, startRight);
        }
    }

    if (g_motorTurnDirectionActiveCommand != MOTOR_TURN_DIRECTION_COMMAND_NONE) {
        if ((int32_t)(now - g_motorTurnDirectionStopTick) >= 0) {
            MotorTurnDirectionPublish("STOP", g_motorTurnDirectionActiveCommand, 0, 0);
            g_motorTurnDirectionActiveCommand = MOTOR_TURN_DIRECTION_COMMAND_NONE;
        } else if (g_motorTurnDirectionActiveCommand == MOTOR_TURN_DIRECTION_COMMAND_LEFT) {
            *leftCommand = 90;
            *rightCommand = 110;
        } else if (g_motorTurnDirectionActiveCommand == MOTOR_TURN_DIRECTION_COMMAND_RIGHT) {
            *leftCommand = 110;
            *rightCommand = 90;
        } else {
            *leftCommand = 100;
            *rightCommand = 100;
        }
    }
}
#endif

#if (ENCODER_BPATH_FOLLOW_V1_TEST_MODE == 1) && (LINE_SENSOR_SIDE_TEST_MODE == 0)
static uint8_t g_traceRecordDiagStartPending;
static uint32_t g_traceRecordDiagLastTick;

typedef enum {
    BPATH_CONTROL_DISARMED = 0,
    BPATH_CONTROL_TRACE_ARM,
    BPATH_CONTROL_TRACE_RECORD,
    BPATH_CONTROL_RETURN_SETTLE,
    BPATH_CONTROL_BPATH_RETURN,
    BPATH_CONTROL_FORK_READY,
    BPATH_CONTROL_FORK_SELECT,
    BPATH_CONTROL_REENTRY_LEFT_ESTABLISH,
    BPATH_CONTROL_REENTRY_ANCHOR_SETTLE,
    BPATH_CONTROL_REENTRY_APPROACH,
    BPATH_CONTROL_REENTRY_DEPART,
    BPATH_CONTROL_REENTRY_FORCE,
    BPATH_CONTROL_REENTRY_LINE_SWEEP,
    BPATH_CONTROL_RECOVERY_RETURN_SETTLE,
    BPATH_CONTROL_RECOVERY_RETURN,
    BPATH_CONTROL_DONE
} BpathControlState;

static volatile BpathControlCommand g_bpathPendingCommand;
static volatile BpathCommandSource g_bpathPendingSource;
static BpathControlState g_bpathControlState = BPATH_CONTROL_DISARMED;
static uint8_t g_bpathAutoBootRun;
/* External-route lifecycle only: never reset by an internal reentry epoch. */
static uint8_t g_startLineConsumed;
static uint32_t g_startLineEventId;
static uint8_t g_manualReturnAuthorized;
/* Test-only, one-shot authorization for DOUBLE_STOP_AUTO_RETURN_TEST_MODE. */
static uint8_t g_doubleStopReturnAuthorized;
static uint8_t g_doubleStopTestReturnActive;
static uint8_t g_manualReturnPipelineActive;
static uint8_t g_bpathAbortStop;
static const char *g_returnTrigger = "NONE";
static uint8_t g_sensorSemanticValid;
static uint32_t g_sensorSemanticEpoch = 1U;
static volatile ForkTestCommand g_forkTestPendingCommand;

typedef enum {
    FORKTEST_IDLE = 0,
    FORKTEST_MARKED,
    FORKTEST_RETURNING_TO_FORK,
    FORKTEST_BACKOFF,
    FORKTEST_READY_REENTRY,
    FORKTEST_SELECT_OPPOSITE,
    FORKTEST_CAPTURED,
    FORKTEST_FAILED
} ForkTestState;

static ForkTestState g_forkTestState;
static uint8_t g_forkTestMarkValid;
static uint8_t g_forkTestTaken;
static uint8_t g_forkTestDesired;
static uint16_t g_forkTestRecordIndex;
static uint16_t g_forkTestReferenceIndex;
static uint16_t g_forkTestBackoffStopReferenceIndex;
static uint16_t g_forkTestReturnCursor;
static uint8_t g_forkTestMarkReached;
static uint16_t g_forkTestBackoffProgress;
static uint32_t g_forkTestSelectStartTick;
static uint32_t g_forkTestCaptureCount;
static const char *g_forkTestResult = "NONE";

/* TWO_LONG returns stop before the first qualified ElevenEvent, using the
 * same reverse source-index mapping and sample-count backoff as FORKTEST.
 * This is deliberately a parked test state: it never enables GO or selection. */
typedef enum {
    AUTO_FORK_IDLE = 0,
    AUTO_FORK_ARMED,
    AUTO_FORK_RETURNING_TO_MARK,
    AUTO_FORK_BACKOFF,
    AUTO_FORK_READY_REENTRY
} AutoForkReturnState;

typedef struct {
    AutoForkReturnState state;
    uint32_t eventId;
    uint16_t forwardStartIndex;
    uint16_t forwardEndIndex;
    uint16_t markReferenceIndex;
    uint16_t backoffStopReferenceIndex;
    uint16_t returnCursor;
    uint16_t backoffProgress;
    uint8_t markReached;
} AutoForkReturn;

static AutoForkReturn g_autoForkReturn;

typedef enum {
    REENTRY_CURVE_UNKNOWN = 0,
    REENTRY_CURVE_LEFT,
    REENTRY_CURVE_RIGHT,
    REENTRY_CURVE_STRAIGHT
} ReentryCurve;

typedef enum {
    REENTRY_TEST_IDLE = 0,
    REENTRY_TEST_LEFT_ESTABLISH,
    REENTRY_TEST_ANCHOR_SETTLE,
    REENTRY_TEST_READY,
    REENTRY_TEST_REFUSED,
    REENTRY_TEST_APPROACH,
    REENTRY_TEST_DEPART,
    REENTRY_TEST_FORCE,
    REENTRY_TEST_LINE_SWEEP,
    REENTRY_TEST_RECOVERY_RETURN,
    REENTRY_TEST_RETRY_READY
} ReentryTestState;

typedef enum {
    REENTRY_REPLAY_COMMAND_NONE = 0,
    REENTRY_REPLAY_COMMAND_TURN,
    REENTRY_REPLAY_COMMAND_FWD
} ReentryReplayCommand;

typedef enum {
    REENTRY_STRAIGHT_COMMAND_NONE = 0,
    REENTRY_STRAIGHT_COMMAND_FWD,
    REENTRY_STRAIGHT_COMMAND_LEFT,
    REENTRY_STRAIGHT_COMMAND_RIGHT
} ReentryStraightCommand;

typedef struct {
    ReentryCurve curve;
    ReentryCurve historyCurve;
    ReentryTestState state;
    uint32_t eventId;
    uint16_t forwardStartIndex;
    uint16_t sampleCount;
    uint64_t leftSum;
    uint64_t rightSum;
    int64_t diff;
    uint32_t ratioPercent;
    uint32_t startTick;
    uint32_t leftExploreStartTick;
    int32_t leftExploreStartEncoder;
    int32_t rightExploreStartEncoder;
    uint8_t leftExploreStarted;
    uint8_t candidateIndex;
    uint32_t phaseStartTick;
    int32_t replayStartLeftEncoder;
    int32_t replayStartRightEncoder;
    int32_t straightStartLeftEncoder;
    int32_t straightStartRightEncoder;
    uint32_t replayTurnCycles;
    uint32_t replayFwdCycles;
    uint8_t replayEarlySensorLogged;
    uint8_t replayMinDwellLogged;
    uint8_t replayTrustDeferredLogged;
    uint8_t captureStableCount;
    uint8_t replayEncoderBaselineValid;
    uint8_t replayLastCommand;
    uint8_t straightEncoderBaselineValid;
    uint8_t straightLastCommand;
    uint8_t attempt;
    uint8_t post11Active;
    uint16_t post11Advances;
    uint8_t valid;
    uint8_t historyDirectional;
    uint8_t historyDataFault;
    uint8_t candidateUsesReplay;
    uint16_t departAdvances;
    uint32_t settleStartTick;
    uint32_t settleLastValidCount;
    int32_t settleLastLeft;
    int32_t settleLastRight;
    uint8_t settleSampleValid;
    uint8_t settleStableCount;
    uint8_t settleResumeRetry;
} ReentryCurveTest;

static ReentryCurveTest g_reentryCurveTest;

/* This owns no path storage.  Once the first fork-return is parked, the
 * existing BPATH arrays are reinitialized as a temporary recovery epoch. */
typedef struct {
    uint8_t recording;
    uint8_t recordStartPending;
    uint8_t firstForwardPointSeen;
    uint8_t directionalStableCount;
    uint16_t lastRecordCount;
    uint16_t sweepLobeAdvances;
    uint8_t sweepLobeIndex;
    ReentryCurve sweepDirection;
    uint8_t straightSeenStable10;
    uint8_t straightSeenStable01;
    uint8_t straightStableState;
    uint8_t sweepExitedOldEleven;
    uint16_t reexitAdvances;
    int32_t anchorLeft;
    int32_t anchorRight;
    const char *failureReason;
    const char *returnReason;
} ReentryRecoveryPath;

static ReentryRecoveryPath g_reentryRecoveryPath;
static uint32_t g_traceLiveLastTick;
static uint8_t g_autoReturn11Active;
static uint32_t g_autoReturn11StartTick;
static int g_motorCommandValid;
static void TraceResetState11Counter(void);
static void TraceClearActiveCorrection(void);
static void TraceBiasReset(uint32_t now);
static void TraceCurveReset(void);
static void TraceCurveEnd(uint32_t now, const char *reason);
static void CrossbarObserverStop(uint32_t now, const char *reason);
static void CrossbarRawForensicsReset(void);
static void Long11ObserverReset(void);
static void RightStoplineCandidateObserverReset(void);
static void DoubleStopAutoReturnObserverReset(void);
static void DoubleStopAutoReturnObserverDeactivate(void);
static void ElevenEventReset(void);
static void ThreeElevenResetForTrace(void);
static void ThreeElevenLogEpochClear(void);
static void BpathControlUpdateIdleRolling(void);
static void ForkTestReset(void);
static void AutoForkReturnReset(void);
static void ReentryCurveTestReset(void);
static void ReentryTestAutoStart(uint32_t now);
static void ReentryTestFail(uint32_t now, const char *reason);
static void ReentryRecoveryBeginReturn(uint32_t now, const char *reason);
static void ReentryAnchorSettleBegin(uint32_t now, uint8_t retry);
static void ReentryAnchorSettleStep(uint32_t now);
static void ReentryTestStep(uint32_t now, WifiIotGpioValue rawLeft,
                            WifiIotGpioValue rawRight, int sensorValid);
static void ReentryRecoveryReturnStep(uint32_t now);
static void ReentryReplayApply(ReentryReplayCommand command, int64_t travelLeft,
                               int64_t travelRight, int64_t currentDiff,
                               uint64_t currentTotal);
static int ReentryTestStartNewTraceEpoch(uint32_t now);
static void ClosePairTerminalReset(void);
static void SensorSemanticSetValid(uint8_t valid, const char *reason);
static const char *ForkTestDirectionName(uint8_t direction);
static const char *ForkTestStateName(int state);

static void BpathControlPublish(const char *event)
{
    (void)UdpTelemetryQueueExperimentText(event);
}

static const char *BpathCommandSourceName(BpathCommandSource source)
{
    if (source == BPATH_COMMAND_SOURCE_UDP) return "UDP";
    if (source == BPATH_COMMAND_SOURCE_BLE) return "BLE";
    return "NONE";
}

static int BpathControlEncoderReady(void)
{
    UdpEncoderTelemetryState encoder;

    UdpTelemetryReadEncoder(&encoder);
    return encoder.validCount != 0U;
}

static void BpathControlSyncSensorSemantic(void)
{
    uint8_t valid = g_bpathControlState == BPATH_CONTROL_TRACE_RECORD ||
        g_bpathControlState == BPATH_CONTROL_REENTRY_APPROACH ||
        g_bpathControlState == BPATH_CONTROL_REENTRY_FORCE ||
        g_bpathControlState == BPATH_CONTROL_REENTRY_LINE_SWEEP;

    SensorSemanticSetValid(valid,
        valid != 0U ? "SEMANTIC_FORWARD_OR_REACQUISITION" :
            "DETERMINISTIC_CONTROL");
}

static void AutoReturn11Reset(void)
{
    g_autoReturn11Active = 0U;
    g_autoReturn11StartTick = 0U;
}

static void BpathControlFinish(void)
{
    if (g_bpathControlState != BPATH_CONTROL_DONE) {
        TraceCurveEnd(osKernelGetTickCount(), g_bpathAbortStop != 0U ? "ABORT" : "DONE");
        CrossbarObserverStop(osKernelGetTickCount(), g_bpathAbortStop != 0U ? "ABORT" : "DONE");
        CrossbarRawForensicsReset();
        Long11ObserverReset();
        RightStoplineCandidateObserverReset();
        g_bpathControlState = BPATH_CONTROL_DONE;
        g_manualReturnAuthorized = 0U;
        g_doubleStopReturnAuthorized = 0U;
        g_manualReturnPipelineActive = 0U;
        DoubleStopAutoReturnObserverDeactivate();
        g_returnTrigger = "NONE";
        AutoReturn11Reset();
        TraceResetState11Counter();
        TraceClearActiveCorrection();
        TraceBiasReset(0U);
        ReentryCurveTestReset();
        if (g_bpathAbortStop != 0U) {
            LineLiveSetControl("ABORT", "STOP", 0, 0);
        } else {
            LineLiveSetControl("DONE", "DONE", 0, 0);
        }
        BpathControlPublish("BPATHCTL event=COMPLETE state=DONE");
    }
}

static void BpathControlAbort(const char *reason)
{
    char text[128];

    g_bpathAbortStop = 1U;
    LineLiveSetControl("ABORT", "STOP", 0, 0);
    (void)snprintf(text, sizeof(text), "BPATHCTL event=ABORT reason=%s", reason);
    BpathControlPublish(text);
    BpathControlFinish();
}

static void BpathControlRecoverFromForwardOverflow(uint32_t now)
{
    BPathFollowInit();
    g_bpathAbortStop = 0U;
    g_manualReturnAuthorized = 0U;
    g_doubleStopReturnAuthorized = 0U;
    g_manualReturnPipelineActive = 0U;
    g_returnTrigger = "NONE";
    g_startLineConsumed = 0U;
    g_startLineEventId = 0U;
    TraceCurveReset();
    CrossbarObserverStop(now, "FORWARD_OVERFLOW_RECOVER");
    CrossbarRawForensicsReset();
    Long11ObserverReset();
    RightStoplineCandidateObserverReset();
    DoubleStopAutoReturnObserverReset();
    ElevenEventReset();
    ThreeElevenResetForTrace();
    AutoForkReturnReset();
    ForkTestReset();
    ReentryCurveTestReset();
    TraceResetState11Counter();
    TraceClearActiveCorrection();
    TraceBiasReset(now);
    g_bpathControlState = BPATH_CONTROL_TRACE_RECORD;
    g_sensorSemanticValid = 1U;
    (void)BPathExternalRecordStart(now);
    LineLiveSetControl("TRACE", "TRACE_RECOVER", 0, 0);
    BpathControlPublish("BPATHCTL event=TRACE_RECOVER reason=FORWARD_OVERFLOW");
}

static void BpathControlReturnGuardBlock(const char *fromState,
                                         const char *requestedState)
{
    char text[176];

    g_manualReturnAuthorized = 0U;
    g_doubleStopReturnAuthorized = 0U;
    g_manualReturnPipelineActive = 0U;
    g_returnTrigger = "NONE";
    g_bpathAutoBootRun = 0U;
    TraceCurveEnd(osKernelGetTickCount(), "ABORT");
    CrossbarObserverStop(osKernelGetTickCount(), "GUARD_BLOCK");
    CrossbarRawForensicsReset();
    Long11ObserverReset();
    RightStoplineCandidateObserverReset();
    DoubleStopAutoReturnObserverReset();
    ElevenEventReset();
    ThreeElevenResetForTrace();
    ForkTestReset();
    AutoForkReturnReset();
    ReentryCurveTestReset();
    g_bpathControlState = BPATH_CONTROL_DISARMED;
    g_bpathAbortStop = 0U;
    LineLiveSetControl("DISARMED", "STOP", 0, 0);
    TraceClearActiveCorrection();
    TraceBiasReset(0U);
    (void)snprintf(text, sizeof(text),
        "BPATHCTL event=RETURN_GUARD_BLOCK reason=NO_MANUAL_REQUEST from_state=%s requested_state=%s",
        fromState, requestedState);
    BpathControlPublish(text);
}

static void TraceLivePublish(uint32_t now, const char *sensor, const char *action,
                             int leftCommand, int rightCommand, const char *phase,
                             uint32_t periodMs, int force)
{
    (void)now;
    (void)sensor;
    (void)periodMs;
    (void)force;
    /* Legacy callers now only update the always-on LINE_LIVE snapshot. */
    LineLiveSetControl(phase, action, leftCommand, rightCommand);
}

static void BpathControlBeginRun(uint8_t autoBoot, uint8_t externalRouteStart)
{
    BPathFollowInit();
    g_bpathAutoBootRun = autoBoot;
    g_manualReturnAuthorized = 0U;
    g_doubleStopReturnAuthorized = 0U;
    g_manualReturnPipelineActive = 0U;
    g_bpathAbortStop = 0U;
    g_returnTrigger = "NONE";
    g_bpathControlState = BPATH_CONTROL_TRACE_ARM;
    g_traceLiveLastTick = 0U;
    LineLiveSetControl("BOOT", "STOP", 0, 0);
    AutoReturn11Reset();
    TraceResetState11Counter();
    TraceClearActiveCorrection();
    TraceBiasReset(osKernelGetTickCount());
    TraceCurveReset();
    CrossbarObserverStop(osKernelGetTickCount(), "TRACE_INIT");
    CrossbarRawForensicsReset();
    Long11ObserverReset();
    RightStoplineCandidateObserverReset();
    DoubleStopAutoReturnObserverReset();
    ElevenEventReset();
    ThreeElevenResetForTrace();
    AutoForkReturnReset();
    ForkTestReset();
    ReentryCurveTestReset();
    if (externalRouteStart != 0U) {
        g_startLineConsumed = 0U;
        g_startLineEventId = 0U;
    }
    BpathControlPublish(autoBoot != 0U ?
        "BPATHCTL event=AUTO_START state=TRACE_ARM" :
        "BPATHCTL event=START state=TRACE_ARM");
}

static int BpathControlBeginReturn(uint32_t now)
{
    char text[128];
    uint8_t authorized = g_manualReturnAuthorized;

#if (DOUBLE_STOP_AUTO_RETURN_TEST_MODE == 1)
    if (g_doubleStopReturnAuthorized != 0U && g_doubleStopTestReturnActive != 0U) {
        authorized = 1U;
    }
#endif
    if (authorized == 0U) {
        BpathControlReturnGuardBlock("TRACE_RECORD", "RETURN_SETTLE");
        return -1;
    }
    /* A fresh RETURN command or this explicit test gate grants one transition. */
    g_manualReturnAuthorized = 0U;
    g_doubleStopReturnAuthorized = 0U;
    g_manualReturnPipelineActive = 1U;
    TraceCurveEnd(now, "RETURN");
    AutoReturn11Reset();
    TraceResetState11Counter();
    TraceClearActiveCorrection();
    TraceBiasReset(now);
    CrossbarObserverStop(now, "RETURN");
    CrossbarRawForensicsReset();
    Long11ObserverReset();
    RightStoplineCandidateObserverReset();
    BPathExternalSetIdleRolling(0U, "RETURN");
    BPathExternalRecordStop(now);
    g_bpathControlState = BPATH_CONTROL_RETURN_SETTLE;
    LineLiveSetControl("RETURN_SETTLE", "RETURN_SETTLE", 0, 0);
    (void)snprintf(text, sizeof(text),
        "BPATHCTL event=RETURN state=RETURN_SETTLE trigger=%s",
        g_returnTrigger);
    BpathControlPublish(text);
    return 0;
}

static void TraceLivePublishManualReturnTrigger(uint32_t now, const char *trigger)
{
    (void)now;
    (void)trigger;
    LineLiveSetControl("RETURN_SETTLE", "RETURN_TRIGGER", 0, 0);
}

static int BpathControlBeginTraceRecord(uint32_t now)
{
    if (g_bpathControlState != BPATH_CONTROL_TRACE_ARM) {
        return -1;
    }
    if (BPathExternalRecordStart(now) != 0) {
        BpathControlAbort("RECORD_START_FAILURE");
        return -1;
    }
    g_bpathControlState = BPATH_CONTROL_TRACE_RECORD;
    CrossbarObserverStop(now, "TRACE_INIT");
    CrossbarRawForensicsReset();
    Long11ObserverReset();
    RightStoplineCandidateObserverReset();
    DoubleStopAutoReturnObserverReset();
    ElevenEventReset();
    ThreeElevenResetForTrace();
    AutoForkReturnReset();
    ReentryCurveTestReset();
    g_traceRecordDiagStartPending = 1U;
    g_traceRecordDiagLastTick = 0U;
    return 0;
}

/* A successful reentry has reached the correct line on a new route. Reuse the
 * normal BPATH run and recorder initialization so no old wrong-branch path,
 * ElevenEvent history, or SEQ11 working window crosses the epoch boundary. */
static int ReentryTestStartNewTraceEpoch(uint32_t now)
{
    UdpEncoderTelemetryState encoder;
    UdpTelemetryReadEncoder(&encoder);
    ThreeElevenLogEpochClear();
    if (BPathExternalRecordStart(now) != 0) {
        return -1;
    }
    g_bpathControlState = BPATH_CONTROL_TRACE_RECORD;
    g_bpathAutoBootRun = 0U;
    g_bpathAbortStop = 0U;
    g_motorCommandValid = 0;
    ElevenEventReset();
    ThreeElevenResetForTrace();
    g_reentryCurveTest.state = REENTRY_TEST_IDLE;
    g_sensorSemanticValid = 1U;
    printf("TRACE_INIT sensor=%02x last_action=%s\r\n",
           (unsigned int)g_lineLiveStableState, g_lineLiveAction);
    printf("TRACE_FIRST_CONTROL sensor=%02x decision=STABLE_SENSOR\r\n",
           (unsigned int)g_lineLiveStableState);
    printf("REENTRY event=NEW_TRACE_EPOCH seq_state=IDLE bpath=RECORDING\r\n");
    return 0;
}

#if (LINE_SENSOR_ANALYSIS_MODE == 0) && (LINE_SENSOR_CALIBRATION_MODE == 0) && \
    (CAR_LINE_CALIBRATION_MODE == 0) && \
    (STM32_UART_LINK_TEST_MODE == 0) && \
    (LINE_SENSOR_SIDE_TEST_MODE == 0)
static void TraceResetRecovery(void);
#endif

static void BpathControlConsumeCommand(uint32_t now)
{
    BpathControlCommand command = g_bpathPendingCommand;
    BpathCommandSource source = g_bpathPendingSource;

    if (command == BPATH_CONTROL_COMMAND_NONE) {
        return;
    }
    g_bpathPendingCommand = BPATH_CONTROL_COMMAND_NONE;
    g_bpathPendingSource = BPATH_COMMAND_SOURCE_NONE;

    if (command == BPATH_CONTROL_COMMAND_START) {
        if (g_bpathControlState == BPATH_CONTROL_DISARMED) {
            BpathControlBeginRun(0U, 1U);
        } else if (g_bpathControlState == BPATH_CONTROL_TRACE_ARM ||
                   g_bpathControlState == BPATH_CONTROL_TRACE_RECORD ||
                   g_bpathControlState == BPATH_CONTROL_RETURN_SETTLE ||
                   g_bpathControlState == BPATH_CONTROL_BPATH_RETURN) {
            BpathControlPublish("BPATHCTL event=IGNORE cmd=START reason=ALREADY_RUNNING");
        } else {
            BpathControlPublish("BPATHCTL event=IGNORE cmd=START reason=NOT_DISARMED");
        }
    } else if (command == BPATH_CONTROL_COMMAND_RETURN) {
        if (g_bpathControlState == BPATH_CONTROL_TRACE_RECORD) {
            if (source == BPATH_COMMAND_SOURCE_UDP || source == BPATH_COMMAND_SOURCE_BLE) {
                char text[128];

                (void)snprintf(text, sizeof(text),
                    "BPATHCTL event=RETURN_ACCEPT source=%s from_state=TRACE_RECORD",
                    BpathCommandSourceName(source));
                BpathControlPublish(text);
                g_manualReturnAuthorized = 1U;
                g_returnTrigger = source == BPATH_COMMAND_SOURCE_UDP ?
                    "MANUAL_UDP" : "MANUAL_BLE";
                if (BpathControlBeginReturn(now) == 0) {
                    TraceLivePublishManualReturnTrigger(now, g_returnTrigger);
                }
            } else {
                BpathControlPublish("BPATHCTL event=IGNORE cmd=RETURN reason=NO_FRESH_SOURCE");
            }
        } else {
            BpathControlPublish("BPATHCTL event=IGNORE cmd=RETURN reason=NOT_FORWARD_ACTIVE");
        }
    } else if (command == BPATH_CONTROL_COMMAND_RESET) {
        TraceCurveEnd(now, "RESET");
        BPathFollowInit();
        g_bpathAutoBootRun = 0U;
        g_manualReturnAuthorized = 0U;
        g_doubleStopReturnAuthorized = 0U;
        g_manualReturnPipelineActive = 0U;
        g_bpathAbortStop = 0U;
        g_returnTrigger = "NONE";
        g_traceLiveLastTick = 0U;
        g_startLineConsumed = 0U;
        g_startLineEventId = 0U;
        AutoReturn11Reset();
        TraceResetState11Counter();
        TraceClearActiveCorrection();
        TraceBiasReset(now);
        TraceCurveReset();
        CrossbarObserverStop(now, "RESET");
        CrossbarRawForensicsReset();
        Long11ObserverReset();
        RightStoplineCandidateObserverReset();
        DoubleStopAutoReturnObserverReset();
        ElevenEventReset();
        ThreeElevenResetForTrace();
        ForkTestReset();
        AutoForkReturnReset();
        ReentryCurveTestReset();
#if (LINE_SENSOR_ANALYSIS_MODE == 0) && (LINE_SENSOR_CALIBRATION_MODE == 0) && \
    (CAR_LINE_CALIBRATION_MODE == 0) && \
    (STM32_UART_LINK_TEST_MODE == 0) && \
    (LINE_SENSOR_SIDE_TEST_MODE == 0)
        TraceResetRecovery();
#endif
        g_bpathControlState = BPATH_CONTROL_DISARMED;
        LineLiveSetControl("DISARMED", "STOP", 0, 0);
        BpathControlPublish("BPATHCTL event=RESET state=DISARMED");
    }
}
#endif

#if (TRACE_OBSERVER_TEST_MODE == 1) && (ENCODER_BPATH_FOLLOW_V1_TEST_MODE == 1) && \
    (LINE_SENSOR_SIDE_TEST_MODE == 0)
#define TRACE_OBSERVER_TELEMETRY_MS 50U
typedef enum {
    TRACE_OBSERVER_CORRECTION_NONE = 0,
    TRACE_OBSERVER_CORRECTION_LEFT,
    TRACE_OBSERVER_CORRECTION_RIGHT
} TraceObserverCorrection;

static volatile TraceObserverCommand g_traceObserverPendingCommand;
static int g_traceObserverActive;
static uint8_t g_traceObserverStableState;
static uint8_t g_traceObserverCandidateState;
static uint8_t g_traceObserverCandidateSamples;
static int g_traceObserverStableValid;
static TraceObserverCorrection g_traceObserverLastCorrection;
static uint32_t g_traceObserverLastTelemetryTick;

static const char *TraceObserverCorrectionName(TraceObserverCorrection correction)
{
    if (correction == TRACE_OBSERVER_CORRECTION_LEFT) return "LEFT";
    if (correction == TRACE_OBSERVER_CORRECTION_RIGHT) return "RIGHT";
    return "NONE";
}

static void TraceObserverReset(void)
{
    g_traceObserverStableState = 0U;
    g_traceObserverCandidateState = 0U;
    g_traceObserverCandidateSamples = 0U;
    g_traceObserverStableValid = 0;
    g_traceObserverLastCorrection = TRACE_OBSERVER_CORRECTION_NONE;
    g_traceObserverLastTelemetryTick = 0U;
}

static void TraceObserverPublishControl(const char *event)
{
    (void)UdpTelemetryQueueExperimentText(event);
}

static void TraceObserverStop(const char *reason)
{
    char text[96];

    TraceObserverReset();
    g_traceObserverActive = 0;
    if (reason == NULL) {
        TraceObserverPublishControl("TRACEOBSCTL event=STOP");
    } else {
        (void)snprintf(text, sizeof(text), "TRACEOBSCTL event=STOP reason=%s", reason);
        TraceObserverPublishControl(text);
    }
}

static void TraceObserverConsumeCommand(void)
{
    TraceObserverCommand command = g_traceObserverPendingCommand;

    if (command == TRACE_OBSERVER_COMMAND_NONE) {
        return;
    }
    g_traceObserverPendingCommand = TRACE_OBSERVER_COMMAND_NONE;
    if (command == TRACE_OBSERVER_COMMAND_START) {
        if (g_traceObserverActive != 0) {
            TraceObserverPublishControl("TRACEOBSCTL event=IGNORE reason=ALREADY_RUNNING");
            return;
        }
        /* Observer must never inherit an active BPATH workflow or its recorder. */
        g_bpathControlState = BPATH_CONTROL_DISARMED;
        TraceObserverReset();
        g_traceObserverActive = 1;
        TraceObserverPublishControl("TRACEOBSCTL event=START");
    } else if (command == TRACE_OBSERVER_COMMAND_STOP) {
        TraceObserverStop(NULL);
    }
}

static void TraceObserverUpdateStableState(WifiIotGpioValue left, WifiIotGpioValue right)
{
    uint8_t rawState = 0U;

    if (left == WIFI_IOT_GPIO_VALUE1) rawState |= 0x02U;
    if (right == WIFI_IOT_GPIO_VALUE1) rawState |= 0x01U;
    if (g_traceObserverCandidateSamples == 0U || rawState != g_traceObserverCandidateState) {
        g_traceObserverCandidateState = rawState;
        g_traceObserverCandidateSamples = 1U;
        return;
    }
    if (g_traceObserverCandidateSamples < 2U) {
        g_traceObserverCandidateSamples++;
    }
    if (g_traceObserverCandidateSamples == 2U &&
        (g_traceObserverStableValid == 0 ||
         g_traceObserverStableState != g_traceObserverCandidateState)) {
        g_traceObserverStableState = g_traceObserverCandidateState;
        g_traceObserverStableValid = 1;
    }
}

static const char *TraceObserverWouldAction(void)
{
    if (g_traceObserverStableState == 0x02U) {
        g_traceObserverLastCorrection = TRACE_OBSERVER_CORRECTION_LEFT;
        return "WOULD_LEFT";
    }
    if (g_traceObserverStableState == 0x01U) {
        g_traceObserverLastCorrection = TRACE_OBSERVER_CORRECTION_RIGHT;
        return "WOULD_RIGHT";
    }
    if (g_traceObserverStableState == 0x03U) {
        if (g_traceObserverLastCorrection == TRACE_OBSERVER_CORRECTION_LEFT) {
            return "WOULD_HOLD_LEFT";
        }
        if (g_traceObserverLastCorrection == TRACE_OBSERVER_CORRECTION_RIGHT) {
            return "WOULD_HOLD_RIGHT";
        }
        return "WOULD_CENTER";
    }
    return "WOULD_FWD";
}

static void TraceObserverStep(uint32_t now, WifiIotGpioValue rawLeft,
                              WifiIotGpioValue rawRight)
{
    char text[224];
    const char *action;

    TraceObserverUpdateStableState(rawLeft, rawRight);
    /* Keep observer history current on every fresh 30 ms sample, not just on publish. */
    action = TraceObserverWouldAction();
    if ((uint32_t)(now - g_traceObserverLastTelemetryTick) <
        AppMsToTicks(TRACE_OBSERVER_TELEMETRY_MS)) {
        return;
    }
    (void)snprintf(text, sizeof(text),
        "TRACE_OBS ms=%u raw_l=%d raw_r=%d stable_l=%u stable_r=%u sensor=%u%u action=%s last_corr=%s motor_l=0 motor_r=0",
        (unsigned int)AppTicksToMs(now), (int)rawLeft, (int)rawRight,
        (unsigned int)((g_traceObserverStableState >> 1) & 0x01U),
        (unsigned int)(g_traceObserverStableState & 0x01U),
        (unsigned int)((g_traceObserverStableState >> 1) & 0x01U),
        (unsigned int)(g_traceObserverStableState & 0x01U),
        action,
        TraceObserverCorrectionName(g_traceObserverLastCorrection));
    (void)UdpTelemetryQueueExperimentText(text);
    g_traceObserverLastTelemetryTick = now;
}
#endif

#if (LINE_SENSOR_ANALYSIS_MODE == 0) && (LINE_SENSOR_CALIBRATION_MODE == 0) && \
    (CAR_LINE_CALIBRATION_MODE == 0) && \
    (STM32_UART_LINK_TEST_MODE == 0) && \
    (LINE_SENSOR_SIDE_TEST_MODE == 0)
typedef enum {
    TRACE_CORRECTION_NONE = 0,
    TRACE_CORRECTION_LEFT,
    TRACE_CORRECTION_RIGHT
} TraceCorrection;

typedef enum {
    TRACE_ACTION_STOP = 0,
    TRACE_ACTION_FORWARD,
    TRACE_ACTION_LEFT,
    TRACE_ACTION_RIGHT,
    TRACE_ACTION_RECOVER_LEFT,
    TRACE_ACTION_RECOVER_RIGHT,
    TRACE_ACTION_HOLD_LEFT,
    TRACE_ACTION_HOLD_RIGHT,
    TRACE_ACTION_RECOVER_CENTER,
    TRACE_ACTION_RIGHT_11,
    TRACE_ACTION_COUNTER_LEFT,
    TRACE_ACTION_COUNTER_RIGHT
} TraceAction;

static TraceCorrection g_lastCorrection;
static TraceAction g_lastAction = TRACE_ACTION_STOP;
static int g_motorLeftCommand;
static int g_motorRightCommand;
static int g_motorCommandValid;
static int g_motorDiagValid;
static int g_motorDiagLeft;
static int g_motorDiagRight;
static uint32_t g_lastMotorTxTick;
static uint8_t g_motorHeartbeatCount;
static uint8_t g_stableState;
static uint8_t g_candidateState;
static uint8_t g_candidateSamples;
static int g_stableStateValid;
static uint8_t g_state11CounterActive;
static TraceCorrection g_state11CounterDirection;
static TraceCorrection g_activeCorrectionDirection;
static uint32_t g_activeCorrectionStartTick;
typedef enum { TRACE_BIAS_NONE = 0, TRACE_BIAS_LEFT, TRACE_BIAS_RIGHT } TraceBias;
static TraceBias g_traceBias;
static TraceBias g_traceBiasApplied;
static uint32_t g_traceBiasWindowStartTick;
static uint32_t g_traceBiasLastTick;
static uint32_t g_traceBiasLeftMs;
static uint32_t g_traceBiasRightMs;
static uint32_t g_traceBiasFwdMs;
static uint8_t g_traceBiasLongCorrection;
static TraceCorrection g_traceBiasCorrectionDirection;
static uint32_t g_traceBiasCorrectionStartTick;

static void TraceBiasReset(uint32_t now)
{
    g_traceBias = TRACE_BIAS_NONE;
    g_traceBiasApplied = TRACE_BIAS_NONE;
    g_traceBiasWindowStartTick = now;
    g_traceBiasLastTick = now;
    g_traceBiasLeftMs = g_traceBiasRightMs = g_traceBiasFwdMs = 0U;
    g_traceBiasLongCorrection = 0U;
    g_traceBiasCorrectionDirection = TRACE_CORRECTION_NONE;
    g_traceBiasCorrectionStartTick = 0U;
    g_lineLiveBias = "NONE";
    g_lineLiveBiasLeftMs = g_lineLiveBiasRightMs = g_lineLiveBiasFwdMs = 0U;
}

static void TraceBiasObserve(TraceAction action)
{
    uint32_t now = osKernelGetTickCount();
    uint32_t elapsed = AppTicksToMs(now - g_traceBiasLastTick);
    TraceCorrection direction = TRACE_CORRECTION_NONE;
    char text[160];

    g_traceBiasLastTick = now;
    if (g_bpathControlState != BPATH_CONTROL_TRACE_RECORD) return;
    if (action == TRACE_ACTION_LEFT || action == TRACE_ACTION_RECOVER_LEFT) {
        g_traceBiasLeftMs += elapsed;
        direction = TRACE_CORRECTION_LEFT;
    } else if (action == TRACE_ACTION_RIGHT || action == TRACE_ACTION_RECOVER_RIGHT ||
               action == TRACE_ACTION_RIGHT_11) {
        g_traceBiasRightMs += elapsed;
        direction = TRACE_CORRECTION_RIGHT;
    } else if (action == TRACE_ACTION_FORWARD) {
        g_traceBiasFwdMs += elapsed;
    }
    if (direction == TRACE_CORRECTION_NONE) {
        g_traceBiasCorrectionDirection = TRACE_CORRECTION_NONE;
        g_traceBiasCorrectionStartTick = 0U;
    } else if (direction != g_traceBiasCorrectionDirection) {
        g_traceBiasCorrectionDirection = direction;
        g_traceBiasCorrectionStartTick = now;
    }
    if (g_traceBiasCorrectionDirection != TRACE_CORRECTION_NONE &&
        AppTicksToMs(now - g_traceBiasCorrectionStartTick) >= TRACE_BIAS_LONG_CORRECTION_MS) {
        g_traceBiasLongCorrection = 1U;
    }
    g_lineLiveBiasLeftMs = g_traceBiasLeftMs;
    g_lineLiveBiasRightMs = g_traceBiasRightMs;
    g_lineLiveBiasFwdMs = g_traceBiasFwdMs;
    if (AppTicksToMs(now - g_traceBiasWindowStartTick) < TRACE_BIAS_WINDOW_MS) return;
    if (g_traceBiasLongCorrection == 0U && g_traceBiasFwdMs >= TRACE_BIAS_FWD_MIN_MS) {
        if (g_traceBiasLeftMs >= g_traceBiasRightMs + TRACE_BIAS_DOMINANCE_MS) g_traceBias = TRACE_BIAS_LEFT;
        else if (g_traceBiasRightMs >= g_traceBiasLeftMs + TRACE_BIAS_DOMINANCE_MS) g_traceBias = TRACE_BIAS_RIGHT;
        else g_traceBias = TRACE_BIAS_NONE;
    } else {
        g_traceBias = TRACE_BIAS_NONE;
    }
    g_lineLiveBias = g_traceBias == TRACE_BIAS_LEFT ? "LEFT" :
                     g_traceBias == TRACE_BIAS_RIGHT ? "RIGHT" : "NONE";
    (void)snprintf(text, sizeof(text),
        "TRACE_BIAS event=UPDATE bias=%s left_ms=%u right_ms=%u fwd_ms=%u fwd_cmd_l=%d fwd_cmd_r=%d",
        g_lineLiveBias, (unsigned int)g_traceBiasLeftMs, (unsigned int)g_traceBiasRightMs,
        (unsigned int)g_traceBiasFwdMs,
        g_traceBias == TRACE_BIAS_LEFT ? 99 : g_traceBias == TRACE_BIAS_RIGHT ? 101 : 100,
        g_traceBias == TRACE_BIAS_LEFT ? 101 : g_traceBias == TRACE_BIAS_RIGHT ? 99 : 100);
    (void)UdpTelemetryQueueExperimentText(text);
    g_traceBiasWindowStartTick = now;
    g_traceBiasLeftMs = g_traceBiasRightMs = g_traceBiasFwdMs = 0U;
    g_traceBiasLongCorrection = 0U;
}

#if (TRACE_STEP_RESPONSE_TEST_MODE == 1)
#define TRACE_STEP_RESPONSE_COAST_MS 500U
typedef enum {
    TRACE_STEP_RESPONSE_IDLE = 0,
    TRACE_STEP_RESPONSE_PULSE,
    TRACE_STEP_RESPONSE_COAST
} TraceStepResponseState;

static volatile TraceStepResponseCommand g_traceStepResponsePendingCommand;
static TraceStepResponseState g_traceStepResponseState;
static TraceStepResponseCommand g_traceStepResponseActiveCommand;
static uint32_t g_traceStepResponseStartTick;
#endif

#if (REVERSE_REPLAY_STRAIGHT_TEST_MODE == 1)
typedef struct {
    int16_t motorLeft;
    int16_t motorRight;
    uint8_t stableSensor;
    uint8_t reserved;
} ReplayFrame;

typedef enum {
    REPLAY_WAIT_START = 0,
    REPLAY_FORWARD_RECORD,
    REPLAY_MARKER_CONFIRM_FORWARD,
    REPLAY_MARKER_SETTLE,
    REPLAY_REVERSE,
    REPLAY_DONE,
    REPLAY_ERROR
} ReplayState;

/* 1200 * 6 bytes = 7200 bytes static RAM; never allocated on task stacks. */
static ReplayFrame g_replayFrames[REPLAY_FRAME_CAPACITY];
static ReplayState g_replayState = REPLAY_WAIT_START;
static uint32_t g_replayFrameCount;
static int32_t g_replayIndex;
static uint32_t g_replayForwardStartTick;
static uint32_t g_replayMarkerTick;
static uint32_t g_replayLastTelemetryTick;
static int g_replayRecordActive;
static int g_replayBufferFull;
static int g_replayLastLeftCommand;
static int g_replayLastRightCommand;
static uint32_t g_replayStraightFrames;
static uint32_t g_replayLeftCorrectionFrames;
static uint32_t g_replayRightCorrectionFrames;
static uint32_t g_replayOtherFrames;
static uint32_t g_replayCommandChanges;
static uint8_t g_replayPreviousStableSensor;
static uint8_t g_replayCurrentStableSensor;
static uint8_t g_replayStateBeforePrevious;
static uint8_t g_replayStableSensorValid;
static uint8_t g_replayStableBeforeMarker[5];
static uint8_t g_replayStableHistoryCount;
static uint32_t g_replayLast00Tick;
static uint32_t g_replayLast10Tick;
static uint32_t g_replayLast01Tick;
static uint32_t g_replayStable00StartTick;
static uint32_t g_replayLastStable00DurationMs;
static uint32_t g_replayStableStateEntryTick;
static uint32_t g_replayPreviousStableEntryTick;
static uint8_t g_replayMarkerDecision;
static UdpReplayMarkerReason g_replayMarkerReason;
static uint8_t g_replaySignedPreambleSensor;
static uint32_t g_replaySignedPreambleMs;
static uint32_t g_replayMarkerConfirmStartTick;

static int ReplayRecordFrame(int leftCommand, int rightCommand)
{
    ReplayFrame *frame;

    if (g_replayFrameCount >= REPLAY_FRAME_CAPACITY) {
        g_replayBufferFull = 1;
        return -1;
    }

    frame = &g_replayFrames[g_replayFrameCount++];
    frame->motorLeft = (int16_t)leftCommand;
    frame->motorRight = (int16_t)rightCommand;
    frame->stableSensor = g_stableState;
    frame->reserved = 0U;

    if (leftCommand == TRACE_FORWARD_SPEED && rightCommand == TRACE_FORWARD_SPEED) {
        g_replayStraightFrames++;
    } else if (leftCommand == TRACE_INNER_SPEED && rightCommand == TRACE_OUTER_SPEED) {
        g_replayLeftCorrectionFrames++;
    } else if (leftCommand == TRACE_OUTER_SPEED && rightCommand == TRACE_INNER_SPEED) {
        g_replayRightCorrectionFrames++;
    } else {
        g_replayOtherFrames++;
    }
    if (g_replayFrameCount > 1U &&
        (leftCommand != g_replayLastLeftCommand || rightCommand != g_replayLastRightCommand)) {
        g_replayCommandChanges++;
    }
    g_replayLastLeftCommand = leftCommand;
    g_replayLastRightCommand = rightCommand;
    return 0;
}

static int ReplayObserveStableSensor(uint32_t now)
{
    int entered = 0;

    if (g_replayStableSensorValid == 0U) {
        g_replayCurrentStableSensor = g_stableState;
        g_replayStableStateEntryTick = now;
        g_replayStableSensorValid = 1U;
    } else if (g_stableState != g_replayCurrentStableSensor) {
        g_replayStateBeforePrevious = g_replayPreviousStableSensor;
        g_replayPreviousStableSensor = g_replayCurrentStableSensor;
        g_replayPreviousStableEntryTick = g_replayStableStateEntryTick;
        g_replayCurrentStableSensor = g_stableState;
        g_replayStableStateEntryTick = now;
        entered = 1;
    }

    if (g_stableState == 0U) {
        g_replayLast00Tick = now;
        if (g_replayStable00StartTick == 0U) {
            g_replayStable00StartTick = now;
        }
    } else if (g_replayStable00StartTick != 0U) {
        g_replayLastStable00DurationMs =
            AppTicksToMs((uint32_t)(now - g_replayStable00StartTick));
        g_replayStable00StartTick = 0U;
    }
    if (g_stableState == 2U) {
        g_replayLast10Tick = now;
    } else if (g_stableState == 1U) {
        g_replayLast01Tick = now;
    }
    return entered;
}

static void ReplayRecordDiagnosticSample(WifiIotGpioValue rawLeft,
                                         WifiIotGpioValue rawRight,
                                         uint32_t now)
{
    UdpReplayHistoryFrame frame = {0};
    uint8_t index;

    frame.timestampMs = AppTicksToMs(now);
    frame.rawLeft = (rawLeft == WIFI_IOT_GPIO_VALUE1) ? 1U : 0U;
    frame.rawRight = (rawRight == WIFI_IOT_GPIO_VALUE1) ? 1U : 0U;
    frame.stableSensor = g_stableState;
    frame.previousStableSensor = g_replayPreviousStableSensor;
    frame.traceAction = (uint8_t)g_lastAction;
    frame.leftCommand = g_motorLeftCommand;
    frame.rightCommand = g_motorRightCommand;
    UdpTelemetryRecordReplayHistory(&frame);

    if (g_replayStableHistoryCount < 5U) {
        g_replayStableBeforeMarker[g_replayStableHistoryCount++] = g_stableState;
    } else {
        for (index = 0U; index < 4U; index++) {
            g_replayStableBeforeMarker[index] = g_replayStableBeforeMarker[index + 1U];
        }
        g_replayStableBeforeMarker[4] = g_stableState;
    }
}
#endif
#if (TRACE_REVERSE_TEST_MODE == 0) && (TRACE_REVERSE_V2_TEST_MODE == 0) && \
    (TRACE_REVERSE_V3_TEST_MODE == 0) && (TRACE_REVERSE_V4_TEST_MODE == 0)
static uint32_t g_lastBleStatTick;
#endif
static TraceCorrection g_recoveryCorrection;
static uint32_t g_recoveryStartTick;
static int g_recoveryActive;

typedef enum {
    AVOID_ACTION_STOP = 0,
    AVOID_ACTION_FORWARD,
    AVOID_ACTION_LEFT_TURN,
    AVOID_ACTION_RIGHT_TURN,
    AVOID_ACTION_BLOCKED
} AvoidAction;

static AvoidAction g_lastAvoidAction = AVOID_ACTION_STOP;
static int g_avoidLeftCommand;
static int g_avoidRightCommand;
static int g_avoidMotorCommandValid;
static uint32_t g_avoidLastMotorTxTick;

#if (TRACE_REVERSE_TEST_MODE == 0) && (TRACE_REVERSE_V2_TEST_MODE == 0) && \
    (TRACE_REVERSE_V3_TEST_MODE == 0) && (TRACE_REVERSE_V4_TEST_MODE == 0)
static UdpTelemetryAction TraceToUdpTelemetryAction(TraceAction action)
{
    switch (action) {
        case TRACE_ACTION_FORWARD:
            return UDP_TELEMETRY_ACTION_FORWARD;
        case TRACE_ACTION_LEFT:
            return UDP_TELEMETRY_ACTION_LEFT;
        case TRACE_ACTION_RIGHT:
            return UDP_TELEMETRY_ACTION_RIGHT;
        case TRACE_ACTION_RECOVER_LEFT:
            return UDP_TELEMETRY_ACTION_RECOVER_LEFT;
        case TRACE_ACTION_RECOVER_RIGHT:
            return UDP_TELEMETRY_ACTION_RECOVER_RIGHT;
        case TRACE_ACTION_RIGHT_11:
            return UDP_TELEMETRY_ACTION_RIGHT;
        case TRACE_ACTION_HOLD_LEFT:
            return UDP_TELEMETRY_ACTION_HOLD_LEFT;
        case TRACE_ACTION_HOLD_RIGHT:
            return UDP_TELEMETRY_ACTION_HOLD_RIGHT;
        case TRACE_ACTION_COUNTER_LEFT:
            return UDP_TELEMETRY_ACTION_LEFT;
        case TRACE_ACTION_COUNTER_RIGHT:
            return UDP_TELEMETRY_ACTION_RIGHT;
        case TRACE_ACTION_RECOVER_CENTER:
            return UDP_TELEMETRY_ACTION_HOLD_CENTER;
        default:
            return UDP_TELEMETRY_ACTION_STOP;
    }
}
#endif

#if (TRACE_REVERSE_TEST_MODE == 0) && (TRACE_REVERSE_V2_TEST_MODE == 0) && \
    (TRACE_REVERSE_V3_TEST_MODE == 0) && (TRACE_REVERSE_V4_TEST_MODE == 0)
__attribute__((unused)) static void TraceUpdateUdpTelemetry(void)
{
    UdpTelemetryUpdate((uint8_t)((g_stableState >> 1) & 0x01U),
                       (uint8_t)(g_stableState & 0x01U),
                       g_stableState, TraceToUdpTelemetryAction(g_lastAction),
                       g_motorLeftCommand, g_motorRightCommand);
}
#endif

static void TraceSendMotorCommand(int leftCommand, int rightCommand, int heartbeat)
{
#if (REVERSE_REPLAY_STRAIGHT_TEST_MODE == 1)
    /* Record immediately before the exact STM32 call.  On a full fixed
     * buffer, replace the requested movement with an immediate stop. */
    if (g_replayRecordActive != 0 && heartbeat == 0) {
        if (ReplayRecordFrame(leftCommand, rightCommand) != 0) {
            leftCommand = 0;
            rightCommand = 0;
        }
    }
#endif
    stm32motor_control(leftCommand, rightCommand);
    if (g_motorDiagValid == 0 || g_motorDiagLeft != leftCommand ||
        g_motorDiagRight != rightCommand) {
        printf("MOTORCMD owner=TRACE cmd_l=%d cmd_r=%d\r\n",
               leftCommand, rightCommand);
        g_motorDiagLeft = leftCommand;
        g_motorDiagRight = rightCommand;
        g_motorDiagValid = 1;
    }
    g_motorLeftCommand = leftCommand;
    g_motorRightCommand = rightCommand;
    g_motorCommandValid = 1;
    g_lastMotorTxTick = osKernelGetTickCount();

    if (heartbeat != 0) {
        g_motorHeartbeatCount++;
        if (g_motorHeartbeatCount >= MOTOR_HEARTBEAT_LOG_EVERY) {
            printf("TX MOTOR HB L=%d R=%d\r\n", leftCommand, rightCommand);
            g_motorHeartbeatCount = 0U;
        }
    }
}

#if (ENCODER_ONLY_EXPERIMENT_MODE == 1) || \
    (ENCODER_BPATH_FOLLOW_V1_TEST_MODE == 1) || \
    (MOTOR_RESPONSE_TEST_MODE == 1) || \
    (LINE_SENSOR_SIDE_TEST_MODE == 1)
/* Encoder experiments remain owned by TaskCarControl, but deliberately bypass
 * TRACE/replay semantics. Keep the generic telemetry snapshot truthful. */
static void EncoderExperimentSendMotorCommand(int leftCommand, int rightCommand,
                                              uint32_t now)
{
    stm32motor_control(leftCommand, rightCommand);
    g_motorLeftCommand = leftCommand;
    g_motorRightCommand = rightCommand;
    g_motorCommandValid = 1;
    g_lastMotorTxTick = now;
    UdpTelemetryUpdateMotorCommand(leftCommand, rightCommand);
}
#endif

__attribute__((unused)) static void TraceHeartbeatIfDue(uint32_t now)
{
    if (g_motorCommandValid == 0 ||
        (uint32_t)(now - g_lastMotorTxTick) <
        AppMsToTicks(MOTOR_COMMAND_HEARTBEAT_MS)) {
        return;
    }

    TraceSendMotorCommand(g_motorLeftCommand, g_motorRightCommand, 1);
}

static void TraceBleDiagAction(TraceAction action)
{
    switch (action) {
        case TRACE_ACTION_FORWARD:
            (void)BleUartSendString("LINE 00 FORWARD 100 100\r\n");
            break;
        case TRACE_ACTION_LEFT:
            (void)BleUartSendString("LINE 10 LEFT 100 120\r\n");
            break;
        case TRACE_ACTION_RIGHT:
            (void)BleUartSendString("LINE 01 RIGHT 120 100\r\n");
            break;
        case TRACE_ACTION_RECOVER_LEFT:
            (void)BleUartSendString("LINE 00 RECOVER_LEFT 100 120\r\n");
            break;
        case TRACE_ACTION_RECOVER_RIGHT:
            (void)BleUartSendString("LINE 00 RECOVER_RIGHT 120 100\r\n");
            break;
        case TRACE_ACTION_RIGHT_11:
            (void)BleUartSendString("LINE 11 RIGHT_11 110 90\r\n");
            break;
        case TRACE_ACTION_HOLD_LEFT:
            (void)BleUartSendString("LINE 11 HOLD_LEFT 100 120\r\n");
            break;
        case TRACE_ACTION_HOLD_RIGHT:
            (void)BleUartSendString("LINE 11 HOLD_RIGHT 120 100\r\n");
            break;
        case TRACE_ACTION_RECOVER_CENTER:
            (void)BleUartSendString("LINE 11 HOLD_CENTER 120 120\r\n");
            break;
        default:
            break;
    }
}

#if (TRACE_RACE_TEST_MODE == 0) && (TRACE_REVERSE_TEST_MODE == 0) && \
    (TRACE_REVERSE_V2_TEST_MODE == 0) && (TRACE_REVERSE_V3_TEST_MODE == 0) && \
    (TRACE_REVERSE_V4_TEST_MODE == 0)
__attribute__((unused)) static void TraceBleDiagHeartbeat(uint32_t now)
{
    char text[64];
    const char *actionName = "STOP";

    if (g_motorCommandValid == 0 ||
        (uint32_t)(now - g_lastBleStatTick) <
        AppMsToTicks(BLE_DIAG_STAT_PERIOD_MS)) {
        return;
    }

    switch (g_lastAction) {
        case TRACE_ACTION_FORWARD:
            actionName = "FWD";
            break;
        case TRACE_ACTION_LEFT:
        case TRACE_ACTION_RECOVER_LEFT:
        case TRACE_ACTION_HOLD_LEFT:
            actionName = "LEFT";
            break;
        case TRACE_ACTION_RIGHT:
        case TRACE_ACTION_RECOVER_RIGHT:
        case TRACE_ACTION_RIGHT_11:
        case TRACE_ACTION_HOLD_RIGHT:
            actionName = "RIGHT";
            break;
        case TRACE_ACTION_RECOVER_CENTER:
            actionName = "HOLD";
            break;
        default:
            break;
    }

    (void)sprintf(text, "STAT line=%u%u action=%s L=%d R=%d\r\n",
                  (unsigned int)((g_stableState >> 1) & 0x01U),
                  (unsigned int)(g_stableState & 0x01U),
                  actionName, g_motorLeftCommand, g_motorRightCommand);
    (void)BleUartSendString(text);
    g_lastBleStatTick = now;
}
#endif

static void TraceApplyAction(TraceAction action)
{
    int actionChanged = (action != g_lastAction);
    int commandChanged = actionChanged;

    /* A new one-second bias decision must re-send FWD even when FWD is unchanged. */
    if (action == TRACE_ACTION_FORWARD && g_traceBiasApplied != g_traceBias) {
        commandChanged = 1;
    }

    if (commandChanged == 0 && g_motorCommandValid != 0
#if (REVERSE_REPLAY_STRAIGHT_TEST_MODE == 1)
        && g_replayRecordActive == 0
#endif
       ) {
        TraceBiasObserve(action);
        return;
    }

    switch (action) {
        case TRACE_ACTION_FORWARD:
            TraceSendMotorCommand(g_traceBias == TRACE_BIAS_LEFT ? 99 : g_traceBias == TRACE_BIAS_RIGHT ? 101 : TRACE_FORWARD_SPEED,
                                  g_traceBias == TRACE_BIAS_LEFT ? 101 : g_traceBias == TRACE_BIAS_RIGHT ? 99 : TRACE_FORWARD_SPEED, 0);
            g_traceBiasApplied = g_traceBias;
            if (actionChanged != 0) printf("TRACE forward bias=%s\r\n", g_lineLiveBias);
            break;
        case TRACE_ACTION_LEFT:
            TraceSendMotorCommand(TRACE_INNER_SPEED, TRACE_OUTER_SPEED, 0);
            if (actionChanged != 0) printf("TRACE left L=%d R=%d\r\n", TRACE_INNER_SPEED, TRACE_OUTER_SPEED);
            break;
        case TRACE_ACTION_RIGHT:
            TraceSendMotorCommand(TRACE_OUTER_SPEED, TRACE_INNER_SPEED, 0);
            if (actionChanged != 0) printf("TRACE right L=%d R=%d\r\n", TRACE_OUTER_SPEED, TRACE_INNER_SPEED);
            break;
        case TRACE_ACTION_RECOVER_LEFT:
            TraceSendMotorCommand(TRACE_INNER_SPEED, TRACE_OUTER_SPEED, 0);
            if (actionChanged != 0) printf("TRACE recover-left L=%d R=%d\r\n", TRACE_INNER_SPEED, TRACE_OUTER_SPEED);
            break;
        case TRACE_ACTION_RECOVER_RIGHT:
            TraceSendMotorCommand(TRACE_OUTER_SPEED, TRACE_INNER_SPEED, 0);
            if (actionChanged != 0) printf("TRACE recover-right L=%d R=%d\r\n", TRACE_OUTER_SPEED, TRACE_INNER_SPEED);
            break;
        case TRACE_ACTION_RIGHT_11:
            TraceSendMotorCommand(TRACE_OUTER_SPEED, TRACE_INNER_SPEED, 0);
            if (actionChanged != 0) printf("TRACE right-11 L=%d R=%d\r\n", TRACE_OUTER_SPEED, TRACE_INNER_SPEED);
            break;
        case TRACE_ACTION_HOLD_LEFT:
            TraceSendMotorCommand(TRACE_INNER_SPEED, TRACE_OUTER_SPEED, 0);
            if (actionChanged != 0) printf("TRACE hold-left L=%d R=%d\r\n", TRACE_INNER_SPEED, TRACE_OUTER_SPEED);
            break;
        case TRACE_ACTION_HOLD_RIGHT:
            TraceSendMotorCommand(TRACE_OUTER_SPEED, TRACE_INNER_SPEED, 0);
            if (actionChanged != 0) printf("TRACE hold-right L=%d R=%d\r\n", TRACE_OUTER_SPEED, TRACE_INNER_SPEED);
            break;
        case TRACE_ACTION_COUNTER_LEFT:
            TraceSendMotorCommand(TRACE_INNER_SPEED, TRACE_OUTER_SPEED, 0);
            if (actionChanged != 0) printf("TRACE counter-left L=%d R=%d\r\n", TRACE_INNER_SPEED, TRACE_OUTER_SPEED);
            break;
        case TRACE_ACTION_COUNTER_RIGHT:
            TraceSendMotorCommand(TRACE_OUTER_SPEED, TRACE_INNER_SPEED, 0);
            if (actionChanged != 0) printf("TRACE counter-right L=%d R=%d\r\n", TRACE_OUTER_SPEED, TRACE_INNER_SPEED);
            break;
        case TRACE_ACTION_RECOVER_CENTER:
            TraceSendMotorCommand(TRACE_RECOVER_SPEED, TRACE_RECOVER_SPEED, 0);
            if (actionChanged != 0) printf("TRACE recover-center L=%d R=%d\r\n", TRACE_RECOVER_SPEED, TRACE_RECOVER_SPEED);
            break;
        case TRACE_ACTION_STOP:
        default:
            TraceSendMotorCommand(0, 0, 0);
            break;
    }

    g_lastAction = action;
    TraceBiasObserve(action);
    if (actionChanged != 0) {
        TraceBleDiagAction(action);
    }
}

#if (ENCODER_BPATH_FOLLOW_V1_TEST_MODE == 1) && (LINE_SENSOR_SIDE_TEST_MODE == 0)
static const char *TraceRecordDiagActionName(TraceAction action)
{
    switch (action) {
        case TRACE_ACTION_FORWARD: return "FWD";
        case TRACE_ACTION_LEFT: return "LEFT";
        case TRACE_ACTION_RIGHT: return "RIGHT";
        case TRACE_ACTION_RECOVER_LEFT: return "LEFT_MIN";
        case TRACE_ACTION_RECOVER_RIGHT: return "RIGHT_MIN";
        case TRACE_ACTION_RIGHT_11: return "RIGHT_11";
        case TRACE_ACTION_HOLD_LEFT: return "HOLD_L";
        case TRACE_ACTION_HOLD_RIGHT: return "HOLD_R";
        case TRACE_ACTION_COUNTER_LEFT: return "COUNTER_LEFT";
        case TRACE_ACTION_COUNTER_RIGHT: return "COUNTER_RIGHT";
        case TRACE_ACTION_RECOVER_CENTER: return "REC_C";
        default: return "STOP";
    }
}

static const char *TraceRecordDiagCorrectionName(TraceCorrection correction)
{
    if (correction == TRACE_CORRECTION_LEFT) return "LEFT";
    if (correction == TRACE_CORRECTION_RIGHT) return "RIGHT";
    return "NONE";
}

/* Event-only trace evidence; no detector or motor decision reads this state. */
static void CrossbarStateText(uint8_t state, char text[3]);
static uint8_t g_crossbarRawForensicsValid;
static uint8_t g_crossbarRawForensicsPrevious;
static uint32_t g_crossbarRawForensicsSequence;

static void CrossbarRawForensicsReset(void)
{
    g_crossbarRawForensicsValid = 0U;
    g_crossbarRawForensicsPrevious = 0U;
}

static void CrossbarRawForensicsPublish(uint32_t now, const char *event,
                                        uint8_t previousState, uint8_t rawState)
{
    UdpEncoderTelemetryState encoder;
    char previousText[3];
    char rawText[3];
    char stableText[3];
    char text[240];
    uint32_t correctionMs = g_activeCorrectionDirection == TRACE_CORRECTION_NONE ? 0U :
        AppTicksToMs(now - g_activeCorrectionStartTick);

    UdpTelemetryReadEncoder(&encoder);
    CrossbarStateText(previousState, previousText);
    CrossbarStateText(rawState, rawText);
    CrossbarStateText(g_stableStateValid != 0U ? g_stableState : rawState, stableText);
    (void)snprintf(text, sizeof(text),
        "CROSSBAR_RAW%s seq=%u ms=%u prev=%s raw=%s stable=%s action=%s enc_l=%ld enc_r=%ld corr_dir=%s corr_ms=%u",
        event != NULL ? event : "", (unsigned int)g_crossbarRawForensicsSequence,
        (unsigned int)AppTicksToMs(now), previousText, rawText, stableText,
        TraceRecordDiagActionName(g_lastAction), (long)encoder.totalLeft, (long)encoder.totalRight,
        TraceRecordDiagCorrectionName(g_activeCorrectionDirection), (unsigned int)correctionMs);
    (void)UdpTelemetryQueueExperimentText(text);
}

static void CrossbarRawForensicsObserve(uint32_t now, WifiIotGpioValue rawLeft,
                                        WifiIotGpioValue rawRight)
{
    uint8_t rawState = (uint8_t)((rawLeft == WIFI_IOT_GPIO_VALUE1 ? 2U : 0U) |
                                 (rawRight == WIFI_IOT_GPIO_VALUE1 ? 1U : 0U));
    uint8_t previousState;

    if (g_crossbarRawForensicsValid == 0U) {
        g_crossbarRawForensicsValid = 1U;
        g_crossbarRawForensicsPrevious = rawState;
        return;
    }
    previousState = g_crossbarRawForensicsPrevious;
    if (rawState == previousState) return;

    g_crossbarRawForensicsSequence++;
    CrossbarRawForensicsPublish(now, NULL, previousState, rawState);
    if (previousState != 0x03U && rawState == 0x03U) {
        CrossbarRawForensicsPublish(now, " event=ENTER_11", previousState, rawState);
    } else if (previousState == 0x03U && rawState != 0x03U) {
        CrossbarRawForensicsPublish(now, " event=EXIT_11", previousState, rawState);
    }
    g_crossbarRawForensicsPrevious = rawState;
}

/*
 * Passive long-11 interval observer.  It only compresses a continuous raw
 * 11 interval into telemetry; it has no connection to motor or BPATH control.
 */
typedef enum {
    LONG_11_OBSERVER_IDLE = 0,
    LONG_11_OBSERVER_IN_11,
    LONG_11_OBSERVER_FIRED,
    LONG_11_OBSERVER_COOLDOWN
} Long11ObserverState;

static Long11ObserverState g_long11ObserverState;
static uint8_t g_long11CooldownClearSeen;
static uint32_t g_long11EnterTick;
static uint32_t g_long11CooldownStartTick;
static uint32_t g_long11Sequence;
static int32_t g_long11EnterEncoderLeft;
static int32_t g_long11EnterEncoderRight;
static uint8_t g_pre11RawValid;
static uint8_t g_pre11PreviousRawState;
static uint8_t g_pre11FwdContextActive;
static uint32_t g_pre11FwdContextStartTick;

static int32_t Long11Abs(int32_t value)
{
    return value < 0 ? -value : value;
}

static void Long11ObserverReset(void)
{
    g_long11ObserverState = LONG_11_OBSERVER_IDLE;
    g_long11CooldownClearSeen = 0U;
    g_long11EnterTick = 0U;
    g_long11CooldownStartTick = 0U;
    g_long11EnterEncoderLeft = 0;
    g_long11EnterEncoderRight = 0;
    g_pre11RawValid = 0U;
    g_pre11PreviousRawState = 0U;
    g_pre11FwdContextActive = 0U;
    g_pre11FwdContextStartTick = 0U;
}

/* Evidence only: records the FWD/00 context that immediately precedes raw 11. */
static void Pre11ContextObserve(uint32_t now, WifiIotGpioValue rawLeft,
                                WifiIotGpioValue rawRight)
{
    UdpEncoderTelemetryState encoder;
    uint8_t rawState = (uint8_t)((rawLeft == WIFI_IOT_GPIO_VALUE1 ? 2U : 0U) |
                                 (rawRight == WIFI_IOT_GPIO_VALUE1 ? 1U : 0U));
    uint8_t stableState = g_stableStateValid != 0U ? g_stableState : rawState;
    char previousText[3];
    char stableText[3];
    char text[224];
    uint32_t preFwdMs = 0U;

    if (rawState == 0x00U && g_lastAction == TRACE_ACTION_FORWARD) {
        if (g_pre11FwdContextActive == 0U) {
            g_pre11FwdContextActive = 1U;
            g_pre11FwdContextStartTick = now;
        }
    } else if (rawState != 0x03U) {
        g_pre11FwdContextActive = 0U;
        g_pre11FwdContextStartTick = 0U;
    }

    if (g_pre11RawValid == 0U) {
        g_pre11RawValid = 1U;
        g_pre11PreviousRawState = rawState;
        return;
    }
    if (g_pre11PreviousRawState == 0x03U || rawState != 0x03U) {
        g_pre11PreviousRawState = rawState;
        return;
    }

    if (g_pre11FwdContextActive != 0U) {
        preFwdMs = AppTicksToMs(now - g_pre11FwdContextStartTick);
    }
    UdpTelemetryReadEncoder(&encoder);
    CrossbarStateText(g_pre11PreviousRawState, previousText);
    CrossbarStateText(stableState, stableText);
    (void)snprintf(text, sizeof(text),
        "PRE11_CONTEXT ms=%u prev_raw=%s raw=11 stable=%s action=%s pre_fwd_ms=%u enc_l=%ld enc_r=%ld",
        (unsigned int)AppTicksToMs(now), previousText, stableText,
        TraceRecordDiagActionName(g_lastAction), (unsigned int)preFwdMs,
        (long)encoder.totalLeft, (long)encoder.totalRight);
    (void)UdpTelemetryQueueExperimentText(text);
    g_pre11PreviousRawState = rawState;
    g_pre11FwdContextActive = 0U;
    g_pre11FwdContextStartTick = 0U;
}

static void Long11ObserverObserve(uint32_t now, WifiIotGpioValue rawLeft,
                                  WifiIotGpioValue rawRight)
{
    UdpEncoderTelemetryState encoder;
    uint8_t rawState = (uint8_t)((rawLeft == WIFI_IOT_GPIO_VALUE1 ? 2U : 0U) |
                                 (rawRight == WIFI_IOT_GPIO_VALUE1 ? 1U : 0U));
    uint32_t elapsedMs;
    char text[224];

    switch (g_long11ObserverState) {
        case LONG_11_OBSERVER_IDLE:
            if (rawState != 0x03U) break;
            UdpTelemetryReadEncoder(&encoder);
            g_long11EnterTick = now;
            g_long11EnterEncoderLeft = encoder.totalLeft;
            g_long11EnterEncoderRight = encoder.totalRight;
            g_long11ObserverState = LONG_11_OBSERVER_IN_11;
            (void)snprintf(text, sizeof(text),
                "LONG_11_OBS event=ENTER ms=%u enter_enc_l=%ld enter_enc_r=%ld",
                (unsigned int)AppTicksToMs(now), (long)encoder.totalLeft, (long)encoder.totalRight);
            (void)UdpTelemetryQueueExperimentText(text);
            break;

        case LONG_11_OBSERVER_IN_11:
            elapsedMs = AppTicksToMs(now - g_long11EnterTick);
            if (rawState != 0x03U) {
                (void)snprintf(text, sizeof(text),
                    "LONG_11_OBS event=SHORT_EXIT duration_ms=%u",
                    (unsigned int)elapsedMs);
                (void)UdpTelemetryQueueExperimentText(text);
                g_long11ObserverState = LONG_11_OBSERVER_IDLE;
                break;
            }
            if (elapsedMs < LONG_11_MIN_MS) break;

            UdpTelemetryReadEncoder(&encoder);
            g_long11Sequence++;
            (void)snprintf(text, sizeof(text),
                "LONG_11_EVENT seq=%u threshold_ms=%u elapsed_ms=%u enter_enc_l=%ld enter_enc_r=%ld current_enc_l=%ld current_enc_r=%ld",
                (unsigned int)g_long11Sequence, (unsigned int)LONG_11_MIN_MS,
                (unsigned int)elapsedMs, (long)g_long11EnterEncoderLeft,
                (long)g_long11EnterEncoderRight, (long)encoder.totalLeft, (long)encoder.totalRight);
            (void)UdpTelemetryQueueExperimentText(text);
            g_long11ObserverState = LONG_11_OBSERVER_FIRED;
            break;

        case LONG_11_OBSERVER_FIRED:
            if (rawState == 0x03U) break;
            UdpTelemetryReadEncoder(&encoder);
            elapsedMs = AppTicksToMs(now - g_long11EnterTick);
            (void)snprintf(text, sizeof(text),
                "LONG_11_OBS event=EXIT total_11_ms=%u delta_l=%ld delta_r=%ld distance_ticks=%ld",
                (unsigned int)elapsedMs, (long)(encoder.totalLeft - g_long11EnterEncoderLeft),
                (long)(encoder.totalRight - g_long11EnterEncoderRight),
                (long)((Long11Abs(encoder.totalLeft - g_long11EnterEncoderLeft) +
                        Long11Abs(encoder.totalRight - g_long11EnterEncoderRight)) / 2));
            (void)UdpTelemetryQueueExperimentText(text);
            g_long11CooldownStartTick = now;
            g_long11CooldownClearSeen = 1U;
            g_long11ObserverState = LONG_11_OBSERVER_COOLDOWN;
            break;

        case LONG_11_OBSERVER_COOLDOWN:
            if (rawState != 0x03U) g_long11CooldownClearSeen = 1U;
            if (g_long11CooldownClearSeen != 0U &&
                AppTicksToMs(now - g_long11CooldownStartTick) >= LONG_11_COOLDOWN_MS) {
                g_long11ObserverState = LONG_11_OBSERVER_IDLE;
            }
            break;

        default:
            Long11ObserverReset();
            break;
    }
}

/*
 * Passive RIGHT-context to raw-11 observer.  This only records a local sensor
 * sequence; it neither changes TRACE decisions nor submits BPATH commands.
 */
typedef enum {
    RIGHT_STOPLINE_OBSERVER_IDLE = 0,
    RIGHT_STOPLINE_OBSERVER_CONTEXT,
    RIGHT_STOPLINE_OBSERVER_IN_11,
    RIGHT_STOPLINE_OBSERVER_COOLDOWN
} RightStoplineObserverState;

static RightStoplineObserverState g_rightStoplineObserverState;
static uint8_t g_rightStoplinePreviousRawValid;
static uint8_t g_rightStoplinePreviousRawState;
static uint32_t g_rightStoplineContextLastTick;
static uint32_t g_rightStoplineEnter11Tick;
static uint32_t g_rightStoplineCooldownStartTick;

static void RightStoplineRawText(uint8_t state, char text[3])
{
    text[0] = (state & 0x02U) != 0U ? '1' : '0';
    text[1] = (state & 0x01U) != 0U ? '1' : '0';
    text[2] = '\0';
}

static uint8_t RightStoplineActionIsRight(void)
{
    return g_lastAction == TRACE_ACTION_RIGHT ||
           g_lastAction == TRACE_ACTION_RECOVER_RIGHT ||
           g_lastAction == TRACE_ACTION_RIGHT_11;
}

static void RightStoplineCandidateObserverReset(void)
{
    g_rightStoplineObserverState = RIGHT_STOPLINE_OBSERVER_IDLE;
    g_rightStoplinePreviousRawValid = 0U;
    g_rightStoplinePreviousRawState = 0U;
    g_rightStoplineContextLastTick = 0U;
    g_rightStoplineEnter11Tick = 0U;
    g_rightStoplineCooldownStartTick = 0U;
    g_lineLiveRightStopState = "IDLE";
    g_lineLiveRightStopContextAgeMs = 0U;
    g_lineLiveRightStopEntryCount = 0U;
    g_lineLiveRightStopCandidateCount = 0U;
    g_lineLiveRightStop11Ms = 0U;
    g_lineLiveRightStopLast11Ms = 0U;
    g_lineLiveRightStopLastExitRaw[0] = '-';
    g_lineLiveRightStopLastExitRaw[1] = '-';
    g_lineLiveRightStopLastExitRaw[2] = '\0';
}

static void RightStoplineCandidateObserverObserve(uint32_t now,
                                                  WifiIotGpioValue rawLeft,
                                                  WifiIotGpioValue rawRight)
{
    uint8_t rawState = (uint8_t)((rawLeft == WIFI_IOT_GPIO_VALUE1 ? 2U : 0U) |
                                 (rawRight == WIFI_IOT_GPIO_VALUE1 ? 1U : 0U));
    uint8_t stableState = g_stableStateValid != 0U ? g_stableState : rawState;
    uint32_t contextAgeMs = 0U;
    uint32_t elapsed11Ms = 0U;

    if (g_rightStoplinePreviousRawValid == 0U) {
        g_rightStoplinePreviousRawValid = 1U;
        g_rightStoplinePreviousRawState = rawState;
    }

    if (g_rightStoplineObserverState == RIGHT_STOPLINE_OBSERVER_CONTEXT ||
        g_rightStoplineObserverState == RIGHT_STOPLINE_OBSERVER_IN_11) {
        contextAgeMs = AppTicksToMs(now - g_rightStoplineContextLastTick);
    }

    switch (g_rightStoplineObserverState) {
        case RIGHT_STOPLINE_OBSERVER_IDLE:
            if (rawState == 0x01U ||
                (rawState != 0x03U &&
                 (stableState == 0x01U || RightStoplineActionIsRight() != 0U))) {
                g_rightStoplineContextLastTick = now;
                g_rightStoplineObserverState = RIGHT_STOPLINE_OBSERVER_CONTEXT;
                g_lineLiveRightStopState = "RIGHT_CONTEXT";
                g_lineLiveRightStopContextAgeMs = 0U;
            }
            break;

        case RIGHT_STOPLINE_OBSERVER_CONTEXT:
            if (rawState == 0x01U ||
                (rawState != 0x03U &&
                 (stableState == 0x01U || RightStoplineActionIsRight() != 0U))) {
                g_rightStoplineContextLastTick = now;
                contextAgeMs = 0U;
            }
            g_lineLiveRightStopContextAgeMs = contextAgeMs;
            if (rawState == 0x03U && g_rightStoplinePreviousRawState != 0x03U &&
                contextAgeMs <= RIGHT_STOPLINE_CONTEXT_MAX_AGE_MS) {
                g_rightStoplineEnter11Tick = now;
                g_lineLiveRightStopEntryCount++;
                g_lineLiveRightStop11Ms = 0U;
                g_lineLiveRightStopState = "IN_11";
                g_rightStoplineObserverState = RIGHT_STOPLINE_OBSERVER_IN_11;
            } else if (contextAgeMs > RIGHT_STOPLINE_CONTEXT_MAX_AGE_MS) {
                g_lineLiveRightStopState = "IDLE";
                g_lineLiveRightStopContextAgeMs = 0U;
                g_rightStoplineObserverState = RIGHT_STOPLINE_OBSERVER_IDLE;
            }
            break;

        case RIGHT_STOPLINE_OBSERVER_IN_11:
            elapsed11Ms = AppTicksToMs(now - g_rightStoplineEnter11Tick);
            g_lineLiveRightStop11Ms = elapsed11Ms;
            g_lineLiveRightStopContextAgeMs = AppTicksToMs(now - g_rightStoplineContextLastTick);
            if (rawState != 0x03U) {
                RightStoplineRawText(rawState, g_lineLiveRightStopLastExitRaw);
                g_lineLiveRightStopLast11Ms = elapsed11Ms;
                g_lineLiveRightStop11Ms = 0U;
                g_lineLiveRightStopCandidateCount++;
                g_lineLiveRightStopState = "COOLDOWN";
                g_rightStoplineCooldownStartTick = now;
                g_rightStoplineObserverState = RIGHT_STOPLINE_OBSERVER_COOLDOWN;
            }
            break;

        case RIGHT_STOPLINE_OBSERVER_COOLDOWN:
            g_lineLiveRightStopContextAgeMs = 0U;
            g_lineLiveRightStop11Ms = 0U;
            if (AppTicksToMs(now - g_rightStoplineCooldownStartTick) >=
                RIGHT_STOPLINE_COOLDOWN_MS) {
                g_lineLiveRightStopState = "IDLE";
                g_rightStoplineObserverState = RIGHT_STOPLINE_OBSERVER_IDLE;
            }
            break;

        default:
            RightStoplineCandidateObserverReset();
            break;
    }

    g_rightStoplinePreviousRawState = rawState;
}

/*
 * Neutral stable-11 episode store.  It owns no semantic decision: temporal
 * life is deliberately separate from persistent forward-path provenance.
 */
typedef struct {
    uint32_t id;
    uint32_t enterTick;
    uint32_t enterMs;
    uint32_t exitMs;
    uint16_t forwardPathStartIndex;
    uint16_t forwardPathEndIndex;
    uint8_t preRaw;  /* fresh raw snapshot when stable 11 is entered */
    uint8_t postRaw; /* fresh raw snapshot when stable 11 is exited */
    uint8_t active;
    uint8_t open;
    uint8_t valid;
} ElevenEvent;

typedef enum {
    ELEVEN_EVENT_SIGNAL_NONE = 0,
    ELEVEN_EVENT_SIGNAL_ENTER,
    ELEVEN_EVENT_SIGNAL_EXIT
} ElevenEventSignal;

static ElevenEvent g_elevenEventHistory[ELEVEN_EVENT_HISTORY_CAPACITY];
static uint32_t g_elevenEventCount;
static uint32_t g_elevenEventHistoryCount;
static uint32_t g_elevenEventDropCount;
static uint8_t g_elevenEventPreviousStableValid;
static uint8_t g_elevenEventPreviousStable;
static int g_elevenEventOpenSlot = -1;

/* One bounded semantic candidate; it adds no history and is reset with TRACE. */
typedef struct {
    uint32_t startTick;
    uint32_t startMs;
    uint16_t startPathIndex;
    uint8_t startRaw;
    uint8_t active;
    uint8_t qualified;
    uint8_t stable11Samples;
} ElevenEventCandidate;

static ElevenEventCandidate g_elevenCandidate;

static void ElevenEventSyncLineLive(void)
{
    uint32_t latestId = 0U;
    uint8_t active = 0U;
    uint32_t index;

    for (index = 0U; index < ELEVEN_EVENT_HISTORY_CAPACITY; index++) {
        if (g_elevenEventHistory[index].valid != 0U &&
            g_elevenEventHistory[index].active != 0U &&
            g_elevenEventHistory[index].id >= latestId) {
            latestId = g_elevenEventHistory[index].id;
            active = 1U;
        }
    }
    g_lineLiveElevenActiveId = latestId;
    g_lineLiveElevenActive = active;
    g_lineLiveElevenCount = g_elevenEventCount;
    g_lineLiveElevenHistoryCount = g_elevenEventHistoryCount;
    g_lineLiveElevenDropCount = g_elevenEventDropCount;
}

static void ElevenEventReset(void)
{
    uint32_t index;

    for (index = 0U; index < ELEVEN_EVENT_HISTORY_CAPACITY; index++) {
        g_elevenEventHistory[index].valid = 0U;
    }
    g_elevenEventCount = 0U;
    g_elevenEventHistoryCount = 0U;
    g_elevenEventDropCount = 0U;
    g_elevenEventPreviousStableValid = 0U;
    g_elevenEventPreviousStable = 0U;
    g_elevenEventOpenSlot = -1;
    g_elevenCandidate.startTick = 0U;
    g_elevenCandidate.startMs = 0U;
    g_elevenCandidate.startPathIndex = 0xffffU;
    g_elevenCandidate.startRaw = 0U;
    g_elevenCandidate.active = 0U;
    g_elevenCandidate.qualified = 0U;
    g_elevenCandidate.stable11Samples = 0U;
    ElevenEventSyncLineLive();
}

static int ElevenEventAllocateSlot(void)
{
    uint32_t index;
    int oldestSlot = -1;
    uint32_t oldestId = 0xffffffffU;

    for (index = 0U; index < ELEVEN_EVENT_HISTORY_CAPACITY; index++) {
        if (g_elevenEventHistory[index].valid == 0U) {
            return (int)index;
        }
        if (g_elevenEventHistory[index].open == 0U &&
            g_elevenEventHistory[index].id < oldestId) {
            oldestId = g_elevenEventHistory[index].id;
            oldestSlot = (int)index;
        }
    }
    if (oldestSlot >= 0) {
        g_elevenEventDropCount++;
    }
    return oldestSlot;
}

/* The caller supplies the owner-loop's fresh raw snapshot and its existing
 * two-sample stable state. Only stable transitions define semantic episodes;
 * raw is retained only as entry/exit forensic context. */
static ElevenEventSignal ElevenEventObserve(uint32_t now, WifiIotGpioValue rawLeft,
                                             WifiIotGpioValue rawRight,
                                             uint8_t stableState,
                                             const ElevenEvent **eventOut)
{
    uint8_t rawState = (uint8_t)((rawLeft == WIFI_IOT_GPIO_VALUE1 ? 2U : 0U) |
                                 (rawRight == WIFI_IOT_GPIO_VALUE1 ? 1U : 0U));
    uint32_t nowMs = AppTicksToMs(now);
    uint32_t dwellMs;
    uint32_t index;

    *eventOut = NULL;
    for (index = 0U; index < ELEVEN_EVENT_HISTORY_CAPACITY; index++) {
        if (g_elevenEventHistory[index].valid != 0U &&
            g_elevenEventHistory[index].active != 0U &&
            AppTicksToMs(now - g_elevenEventHistory[index].enterTick) >=
                ELEVEN_EVENT_LIVE_MS) {
            g_elevenEventHistory[index].active = 0U;
        }
    }
    if (g_elevenEventPreviousStableValid == 0U) {
        g_elevenEventPreviousStableValid = 1U;
        g_elevenEventPreviousStable = stableState == 0x03U ? 0U : stableState;
    }
    if (stableState == 0x03U && g_elevenEventPreviousStable != 0x03U) {
        /* Freeze rolling compaction before exposing startPathIndex. */
        BPathExternalSetIdleRolling(0U, "ELEVEN_CANDIDATE");
        g_elevenCandidate.startTick = now;
        g_elevenCandidate.startMs = nowMs;
        g_elevenCandidate.startRaw = rawState;
        if (BPathExternalGetForwardRecordIndex(&g_elevenCandidate.startPathIndex) != 0) {
            g_elevenCandidate.startPathIndex = 0xffffU;
        }
        g_elevenCandidate.active = 1U;
        g_elevenCandidate.qualified = 0U;
        g_elevenCandidate.stable11Samples = 1U;
        g_elevenEventPreviousStable = stableState;
        printf("SEQ11EVT event=CAND start=%u\r\n", (unsigned int)g_elevenCandidate.startMs);
        ElevenEventSyncLineLive();
        return ELEVEN_EVENT_SIGNAL_NONE;
    }
    if (g_elevenCandidate.active != 0U && stableState == 0x03U &&
        g_elevenCandidate.qualified == 0U) {
        int slot;
        ElevenEvent *event;

        if (g_elevenCandidate.stable11Samples < 0xffU) {
            g_elevenCandidate.stable11Samples++;
        }
        dwellMs = nowMs - g_elevenCandidate.startMs;
        if (g_elevenCandidate.stable11Samples >= 2U) {
            slot = ElevenEventAllocateSlot();
            if (slot < 0) {
                g_elevenEventDropCount++;
                g_elevenCandidate.qualified = 1U;
                g_elevenEventPreviousStable = stableState;
                ElevenEventSyncLineLive();
                return ELEVEN_EVENT_SIGNAL_NONE;
            }
            event = &g_elevenEventHistory[slot];
            event->id = ++g_elevenEventCount;
            event->enterTick = g_elevenCandidate.startTick;
            event->enterMs = g_elevenCandidate.startMs;
            event->exitMs = 0U;
            event->preRaw = g_elevenCandidate.startRaw;
            event->postRaw = 0U;
            event->active = 1U;
            event->open = 1U;
            event->valid = 1U;
            event->forwardPathStartIndex = g_elevenCandidate.startPathIndex;
            event->forwardPathEndIndex = 0xffffU;
            if (g_elevenEventHistoryCount < ELEVEN_EVENT_HISTORY_CAPACITY) {
                g_elevenEventHistoryCount++;
            }
            g_elevenEventOpenSlot = slot;
            g_elevenCandidate.qualified = 1U;
            *eventOut = event;
            printf("SEQ11EVT event=QUALIFY id=%u start=%u now=%u dwell=%u path_start=%u\r\n",
                   (unsigned int)event->id, (unsigned int)event->enterMs,
                   (unsigned int)nowMs, (unsigned int)dwellMs,
                   (unsigned int)event->forwardPathStartIndex);
            g_elevenEventPreviousStable = stableState;
            ElevenEventSyncLineLive();
            return ELEVEN_EVENT_SIGNAL_ENTER;
        }
    }
    if (g_elevenCandidate.active != 0U && stableState != 0x03U &&
        g_elevenEventPreviousStable == 0x03U) {
        dwellMs = nowMs - g_elevenCandidate.startMs;
        if (g_elevenCandidate.qualified == 0U) {
            printf("SEQ11EVT event=SHORT start=%u end=%u dwell=%u\r\n",
                   (unsigned int)g_elevenCandidate.startMs, (unsigned int)nowMs,
                   (unsigned int)dwellMs);
        } else if (g_elevenEventOpenSlot >= 0) {
            ElevenEvent *event = &g_elevenEventHistory[g_elevenEventOpenSlot];

            event->exitMs = nowMs;
            event->postRaw = rawState;
            event->open = 0U;
            if (BPathExternalGetForwardRecordIndex(&event->forwardPathEndIndex) != 0) {
                event->forwardPathEndIndex = 0xffffU;
            }
            *eventOut = event;
            g_elevenEventOpenSlot = -1;
            printf("SEQ11EVT event=EXIT id=%u enter=%u exit=%u duration=%u path_end=%u\r\n",
                   (unsigned int)event->id, (unsigned int)event->enterMs,
                   (unsigned int)event->exitMs,
                   (unsigned int)(event->exitMs - event->enterMs),
                   (unsigned int)event->forwardPathEndIndex);
            g_elevenCandidate.active = 0U;
            g_elevenCandidate.qualified = 0U;
            g_elevenCandidate.stable11Samples = 0U;
            g_elevenEventPreviousStable = stableState;
            ElevenEventSyncLineLive();
            return ELEVEN_EVENT_SIGNAL_EXIT;
        }
        g_elevenCandidate.active = 0U;
        g_elevenCandidate.qualified = 0U;
        g_elevenCandidate.stable11Samples = 0U;
        g_elevenEventPreviousStable = stableState;
        ElevenEventSyncLineLive();
        return ELEVEN_EVENT_SIGNAL_NONE;
    }
    g_elevenEventPreviousStable = stableState;
    ElevenEventSyncLineLive();
    return ELEVEN_EVENT_SIGNAL_NONE;
}

typedef enum {
    SEQ11_IDLE = 0,
    SEQ11_HAVE_FIRST,
    SEQ11_HAVE_SECOND
} ThreeElevenSequenceState;

typedef enum {
    SEQ11_CLAIM_NONE = 0,
    SEQ11_CLAIM_RETURN,
    SEQ11_CLAIM_STOP
} ThreeElevenSequenceClaim;

typedef struct {
    ThreeElevenSequenceState state;
    uint32_t firstEventId;
    uint32_t secondEventId;
    uint32_t thirdEventId;
    uint16_t firstPathIndex;
    uint16_t secondPathIndex;
    uint16_t thirdPathIndex;
    uint32_t firstEnterMs;
    uint32_t secondEnterMs;
    uint32_t thirdEnterMs;
    uint32_t gap12Ms;
    uint32_t gap23Ms;
    uint32_t firstWaitStartTick;
    uint32_t secondWaitStartTick;
    ThreeElevenSequenceClaim claim;
    uint8_t paused;
} ThreeElevenSequenceClassifier;

static ThreeElevenSequenceClassifier g_threeEleven;
static uint32_t g_threeElevenForkEventId;
static uint16_t g_threeElevenForkPathIndex;
static uint32_t g_threeElevenStaleEventId;
static int32_t g_threeElevenStaleExitLeft;
static int32_t g_threeElevenStaleExitRight;
static uint8_t g_threeElevenStaleExitValid;

typedef struct {
    uint8_t active;
    uint8_t stable11Samples;
    uint8_t pulseOneValid;
    uint32_t pulseOneEpoch;
    uint8_t pulseOneExitEncoderValid;
    uint32_t enterMs;
    uint32_t pulseOneExitMs;
    int32_t pulseOneExitEncoderLeft;
    int32_t pulseOneExitEncoderRight;
} ClosePairTerminalDetector;

static ClosePairTerminalDetector g_closePairTerminal;

static const char *ThreeElevenStateName(ThreeElevenSequenceState state)
{
    switch (state) {
        case SEQ11_HAVE_FIRST: return "HAVE_FIRST";
        case SEQ11_HAVE_SECOND: return "HAVE_SECOND";
        default: return "IDLE";
    }
}

static const char *ThreeElevenClaimName(ThreeElevenSequenceClaim claim)
{
    switch (claim) {
        case SEQ11_CLAIM_RETURN: return "RETURN";
        case SEQ11_CLAIM_STOP: return "STOP";
        default: return "NONE";
    }
}

static void ThreeElevenLogEpochClear(void)
{
    printf("SEQ11 event=EPOCH_CLEAR reason=REENTRY_SUCCESS old_e1=%u old_e2=%u old_e3=%u\r\n",
           (unsigned int)g_threeEleven.firstEventId,
           (unsigned int)g_threeEleven.secondEventId,
           (unsigned int)g_threeEleven.thirdEventId);
}

static const ElevenEvent *ElevenEventFindById(uint32_t id)
{
    uint32_t index;

    for (index = 0U; index < ELEVEN_EVENT_HISTORY_CAPACITY; index++) {
        if (g_elevenEventHistory[index].valid != 0U &&
            g_elevenEventHistory[index].id == id) {
            return &g_elevenEventHistory[index];
        }
    }
    return NULL;
}

static const char *ReentryCurveName(ReentryCurve curve)
{
    if (curve == REENTRY_CURVE_LEFT) return "LEFT";
    if (curve == REENTRY_CURVE_RIGHT) return "RIGHT";
    if (curve == REENTRY_CURVE_STRAIGHT) return "STRAIGHT";
    return "UNKNOWN";
}

static ReentryCurve ReentryCurveOpposite(ReentryCurve curve)
{
    if (curve == REENTRY_CURVE_LEFT) return REENTRY_CURVE_RIGHT;
    if (curve == REENTRY_CURVE_RIGHT) return REENTRY_CURVE_LEFT;
    return REENTRY_CURVE_UNKNOWN;
}

static ReentryCurve ReentrySweepDirectionForLobe(ReentryCurve candidate,
                                                  uint8_t lobeIndex)
{
    if (candidate == REENTRY_CURVE_STRAIGHT) {
        return (lobeIndex & 1U) != 0U ? REENTRY_CURVE_LEFT : REENTRY_CURVE_RIGHT;
    }
    return (lobeIndex & 1U) != 0U ? ReentryCurveOpposite(candidate) : candidate;
}

static uint16_t ReentrySweepLobeAdvanceLimit(ReentryCurve candidate,
                                              ReentryCurve direction)
{
    if (candidate == REENTRY_CURVE_LEFT) {
        return direction == REENTRY_CURVE_LEFT ?
            REENTRY_LEFT_SWEEP_LEFT_ADVANCES : REENTRY_LEFT_SWEEP_RIGHT_ADVANCES;
    }
    return REENTRY_SWEEP_LOBE_ADVANCES;
}

/* The safe-left handoff is a vehicle-control property: every bounded
 * STRAIGHT candidate may hand normal TRACE a stable 10, but never a 01. */
static uint8_t ReentryIsStraightCandidate(void)
{
    return g_reentryCurveTest.curve == REENTRY_CURVE_STRAIGHT ? 1U : 0U;
}

static uint8_t ReentryHasAnotherAttempt(void)
{
    return g_reentryCurveTest.attempt < 3U ? 1U : 0U;
}

static uint64_t ReentryUnsignedTravelDelta(int32_t newer, int32_t older)
{
    int64_t delta = (int64_t)newer - (int64_t)older;

    return delta < 0 ? (uint64_t)(-delta) : (uint64_t)delta;
}

/* Read only E1-preceding encoder history once the final TWO_LONG decision is
 * known.  The final edge samples are deliberately excluded from this curve
 * estimate because they can belong to the intersection transition itself. */
static void ReentryCurveLock(const ElevenEvent *firstEvent)
{
    uint16_t windowEnd;
    uint16_t windowStart;
    uint16_t index;
    int32_t previousLeft;
    int32_t previousRight;
    uint64_t total;
    uint64_t absoluteDiff;

    ReentryCurveTestReset();
    if (firstEvent == NULL || firstEvent->valid == 0U ||
        firstEvent->forwardPathStartIndex == 0xffffU ||
        firstEvent->forwardPathStartIndex <= REENTRY_CURVE_EDGE_SKIP_SAMPLES) {
        printf("REENTRYCURVE event=LOCK e1=%u start_idx=%u samples=0 skip=%u left=0 right=0 diff=0 ratio=0 curve=UNKNOWN\r\n",
               firstEvent == NULL ? 0U : (unsigned int)firstEvent->id,
               firstEvent == NULL ? 0xffffU :
                   (unsigned int)firstEvent->forwardPathStartIndex,
               (unsigned int)REENTRY_CURVE_EDGE_SKIP_SAMPLES);
        return;
    }

    windowEnd = (uint16_t)(firstEvent->forwardPathStartIndex -
                           REENTRY_CURVE_EDGE_SKIP_SAMPLES - 1U);
    windowStart = windowEnd >= (REENTRY_CURVE_LOOKBACK_SAMPLES - 1U) ?
        (uint16_t)(windowEnd - (REENTRY_CURVE_LOOKBACK_SAMPLES - 1U)) : 0U;
    if (BPathExternalGetForwardPointTravel(windowStart, &previousLeft,
                                           &previousRight) != 0) {
        g_reentryCurveTest.eventId = firstEvent->id;
        g_reentryCurveTest.forwardStartIndex = firstEvent->forwardPathStartIndex;
        g_reentryCurveTest.historyDataFault = 1U;
        printf("REENTRYCURVE event=LOCK e1=%u start_idx=%u samples=0 skip=%u left=0 right=0 diff=0 ratio=0 curve=UNKNOWN\r\n",
               (unsigned int)firstEvent->id,
               (unsigned int)firstEvent->forwardPathStartIndex,
               (unsigned int)REENTRY_CURVE_EDGE_SKIP_SAMPLES);
        return;
    }

    g_reentryCurveTest.eventId = firstEvent->id;
    g_reentryCurveTest.forwardStartIndex = firstEvent->forwardPathStartIndex;
    g_reentryCurveTest.sampleCount = (uint16_t)(windowEnd - windowStart + 1U);
    for (index = (uint16_t)(windowStart + 1U); index <= windowEnd; index++) {
        int32_t currentLeft;
        int32_t currentRight;

        if (BPathExternalGetForwardPointTravel(index, &currentLeft, &currentRight) != 0) {
            ReentryCurveTestReset();
            g_reentryCurveTest.eventId = firstEvent->id;
            g_reentryCurveTest.forwardStartIndex = firstEvent->forwardPathStartIndex;
            g_reentryCurveTest.historyDataFault = 1U;
            printf("REENTRYCURVE event=LOCK e1=%u start_idx=%u samples=0 skip=%u left=0 right=0 diff=0 ratio=0 curve=UNKNOWN\r\n",
                   (unsigned int)firstEvent->id,
                   (unsigned int)firstEvent->forwardPathStartIndex,
                   (unsigned int)REENTRY_CURVE_EDGE_SKIP_SAMPLES);
            return;
        }
        g_reentryCurveTest.leftSum += ReentryUnsignedTravelDelta(currentLeft, previousLeft);
        g_reentryCurveTest.rightSum += ReentryUnsignedTravelDelta(currentRight, previousRight);
        previousLeft = currentLeft;
        previousRight = currentRight;
    }

    total = g_reentryCurveTest.leftSum + g_reentryCurveTest.rightSum;
    g_reentryCurveTest.diff = (int64_t)g_reentryCurveTest.rightSum -
                              (int64_t)g_reentryCurveTest.leftSum;
    absoluteDiff = g_reentryCurveTest.diff < 0 ?
        (uint64_t)(-g_reentryCurveTest.diff) : (uint64_t)g_reentryCurveTest.diff;
    if (total != 0U) {
        g_reentryCurveTest.ratioPercent = (uint32_t)((absoluteDiff * 100U) / total);
    }
    if (g_reentryCurveTest.sampleCount < REENTRY_CURVE_MIN_SAMPLES || total == 0U) {
        g_reentryCurveTest.curve = REENTRY_CURVE_UNKNOWN;
    } else if (absoluteDiff * 100U < total * REENTRY_CURVE_MIN_RATIO_PERCENT) {
        g_reentryCurveTest.valid = 1U;
        g_reentryCurveTest.curve = REENTRY_CURVE_STRAIGHT;
    } else {
        g_reentryCurveTest.valid = 1U;
        g_reentryCurveTest.curve = g_reentryCurveTest.diff > 0 ?
            REENTRY_CURVE_LEFT : REENTRY_CURVE_RIGHT;
    }
    g_reentryCurveTest.historyCurve = g_reentryCurveTest.curve;
    g_reentryCurveTest.historyDirectional =
        g_reentryCurveTest.curve == REENTRY_CURVE_LEFT ||
        g_reentryCurveTest.curve == REENTRY_CURVE_RIGHT ? 1U : 0U;
    if (g_reentryCurveTest.curve == REENTRY_CURVE_LEFT) {
        printf("REENTRYPOLICY event=SELECT history=LEFT attempt1=LEFT attempt2=STRAIGHT attempt3=RIGHT\r\n");
    } else if (g_reentryCurveTest.curve == REENTRY_CURVE_RIGHT) {
        printf("REENTRYPOLICY event=SELECT history=RIGHT attempt1=LEFT attempt2=STRAIGHT attempt3=RIGHT\r\n");
    } else if (g_reentryCurveTest.curve == REENTRY_CURVE_STRAIGHT) {
        printf("REENTRYHISTORY event=LOCK curve=STRAIGHT\r\n");
        printf("REENTRYPOLICY event=SELECT history=STRAIGHT attempt1=LEFT attempt2=STRAIGHT attempt3=RIGHT\r\n");
    }
    printf("REENTRYCURVE event=LOCK e1=%u start_idx=%u samples=%u skip=%u left=%llu right=%llu diff=%lld ratio=%u curve=%s\r\n",
           (unsigned int)g_reentryCurveTest.eventId,
           (unsigned int)g_reentryCurveTest.forwardStartIndex,
           (unsigned int)g_reentryCurveTest.sampleCount,
           (unsigned int)REENTRY_CURVE_EDGE_SKIP_SAMPLES,
           (unsigned long long)g_reentryCurveTest.leftSum,
           (unsigned long long)g_reentryCurveTest.rightSum,
           (long long)g_reentryCurveTest.diff,
           (unsigned int)g_reentryCurveTest.ratioPercent,
           ReentryCurveName(g_reentryCurveTest.curve));
}

/* Arm only from the final TWO_LONG semantic decision.  The persistent history
 * record, not the temporal active id, owns the target forward source index. */
static int AutoForkArmTwoLong(const ElevenEvent *firstEvent)
{
    uint16_t forwardCount;
    uint16_t forwardCapacity;

    BPathExternalGetForwardRecordProgress(&forwardCount, &forwardCapacity);
    g_autoForkReturn.state = AUTO_FORK_ARMED;
    g_autoForkReturn.eventId = firstEvent->id;
    g_autoForkReturn.forwardStartIndex = firstEvent->forwardPathStartIndex;
    g_autoForkReturn.forwardEndIndex = firstEvent->forwardPathEndIndex;
    g_autoForkReturn.markReferenceIndex = 0U;
    g_autoForkReturn.backoffStopReferenceIndex = 0U;
    g_autoForkReturn.returnCursor = 0U;
    g_autoForkReturn.backoffProgress = 0U;
    g_autoForkReturn.markReached = 0U;
    ReentryCurveLock(firstEvent);
    printf("AUTOFORK event=ARM e1=%u e1_start=%u e1_end=%u current_forward=%u\r\n",
           (unsigned int)g_autoForkReturn.eventId,
           (unsigned int)g_autoForkReturn.forwardStartIndex,
           (unsigned int)g_autoForkReturn.forwardEndIndex,
           forwardCount == 0U ? 0U : (unsigned int)(forwardCount - 1U));
    return 0;
}

static void ThreeElevenSyncLineLive(void)
{
    g_lineLiveSeq11State = ThreeElevenStateName(g_threeEleven.state);
    g_lineLiveSeq11E1 = g_threeEleven.firstEventId;
    g_lineLiveSeq11E2 = g_threeEleven.secondEventId;
    g_lineLiveSeq11E3 = g_threeEleven.thirdEventId;
    g_lineLiveSeq11Gap12Ms = g_threeEleven.gap12Ms;
    g_lineLiveSeq11Gap23Ms = g_threeEleven.gap23Ms;
    g_lineLiveSeq11Claim = ThreeElevenClaimName(g_threeEleven.claim);
    g_lineLiveSeq11ForkEventId = g_threeElevenForkEventId;
    g_lineLiveSeq11ForkPathIndex = g_threeElevenForkPathIndex;
}

static void ClosePairTerminalReset(void)
{
    g_closePairTerminal.active = 0U;
    g_closePairTerminal.stable11Samples = 0U;
    g_closePairTerminal.pulseOneValid = 0U;
    g_closePairTerminal.pulseOneEpoch = 0U;
    g_closePairTerminal.pulseOneExitEncoderValid = 0U;
    g_closePairTerminal.enterMs = 0U;
    g_closePairTerminal.pulseOneExitMs = 0U;
    g_closePairTerminal.pulseOneExitEncoderLeft = 0;
    g_closePairTerminal.pulseOneExitEncoderRight = 0;
}

static void SensorSemanticSetValid(uint8_t valid, const char *reason)
{
    if (valid == g_sensorSemanticValid) {
        return;
    }
    if (valid == 0U) {
        ClosePairTerminalReset();
        g_sensorSemanticValid = 0U;
        printf("SENSOR event=SEMANTIC_INVALID reason=%s epoch=%u\r\n",
               reason, (unsigned int)g_sensorSemanticEpoch);
        return;
    }
    g_sensorSemanticEpoch++;
    g_sensorSemanticValid = 1U;
    printf("REENTRY event=SENSOR_VALID reason=%s epoch=%u\r\n",
           reason, (unsigned int)g_sensorSemanticEpoch);
}

/* Reset only temporary sequence memory; retain the selected fork path mark. */
static void ResetThreeElevenSequenceClassifier(void)
{
    g_threeEleven.state = SEQ11_IDLE;
    g_threeEleven.firstEventId = 0U;
    g_threeEleven.secondEventId = 0U;
    g_threeEleven.thirdEventId = 0U;
    g_threeEleven.firstPathIndex = 0xffffU;
    g_threeEleven.secondPathIndex = 0xffffU;
    g_threeEleven.thirdPathIndex = 0xffffU;
    g_threeEleven.firstEnterMs = 0U;
    g_threeEleven.secondEnterMs = 0U;
    g_threeEleven.thirdEnterMs = 0U;
    g_threeEleven.gap12Ms = 0U;
    g_threeEleven.gap23Ms = 0U;
    g_threeEleven.firstWaitStartTick = 0U;
    g_threeEleven.secondWaitStartTick = 0U;
}

static void ThreeElevenResetForTrace(void)
{
    g_threeEleven.claim = SEQ11_CLAIM_NONE;
    g_threeEleven.paused = 0U;
    g_threeElevenForkEventId = 0U;
    g_threeElevenForkPathIndex = 0xffffU;
    g_threeElevenStaleEventId = 0U;
    g_threeElevenStaleExitLeft = 0;
    g_threeElevenStaleExitRight = 0;
    g_threeElevenStaleExitValid = 0U;
    g_lineLiveSeq11Decision = "NONE";
    ClosePairTerminalReset();
    ResetThreeElevenSequenceClassifier();
    ThreeElevenSyncLineLive();
}

/* This deliberately does not create ElevenEvents or sequence members.  It
 * recognizes only two independently exited physical stable-11 segments after
 * the external start line has completed. */
static void ClosePairTerminalObserve(uint32_t now, uint8_t stableState)
{
    UdpEncoderTelemetryState encoder;
    uint32_t nowMs = AppTicksToMs(now);
    uint32_t gapMs;
    int64_t deltaLeft;
    int64_t deltaRight;
    uint64_t averageTravel;
    uint8_t eligible = g_sensorSemanticValid != 0U &&
        g_bpathControlState == BPATH_CONTROL_TRACE_RECORD &&
        g_startLineConsumed != 0U && g_startLineEventId == 0U;

    if (eligible == 0U) {
        ClosePairTerminalReset();
        return;
    }
    if (g_closePairTerminal.pulseOneValid != 0U &&
        g_closePairTerminal.active == 0U &&
        nowMs - g_closePairTerminal.pulseOneExitMs >= CLOSE_PAIR_TERMINAL_GAP_MAX_MS) {
        g_closePairTerminal.pulseOneValid = 0U;
    }
    if (stableState == 0x03U) {
        if (g_closePairTerminal.active == 0U) {
            g_closePairTerminal.active = 1U;
            g_closePairTerminal.stable11Samples = 1U;
            g_closePairTerminal.enterMs = nowMs;
            if (g_closePairTerminal.pulseOneValid != 0U &&
                g_closePairTerminal.enterMs - g_closePairTerminal.pulseOneExitMs >=
                    CLOSE_PAIR_TERMINAL_GAP_MAX_MS) {
                g_closePairTerminal.pulseOneValid = 0U;
            }
            return;
        }
        if (g_closePairTerminal.stable11Samples < 0xffU) {
            g_closePairTerminal.stable11Samples++;
        }
        if (g_closePairTerminal.stable11Samples == CLOSE_PAIR_SEGMENT_MIN_STABLE_SAMPLES) {
            if (g_closePairTerminal.pulseOneValid == 0U) {
                printf("TERMINAL event=SEGMENT_START index=1 enter_ms=%u stable_samples=%u\r\n",
                       (unsigned int)g_closePairTerminal.enterMs,
                       (unsigned int)g_closePairTerminal.stable11Samples);
                printf("ADMIN event=FIRST_SEGMENT epoch=%u enter_ms=%u\r\n",
                       (unsigned int)g_sensorSemanticEpoch,
                       (unsigned int)g_closePairTerminal.enterMs);
                return;
            }
            gapMs = g_closePairTerminal.enterMs - g_closePairTerminal.pulseOneExitMs;
            if (gapMs < CLOSE_PAIR_TERMINAL_GAP_MAX_MS &&
                g_closePairTerminal.pulseOneEpoch == g_sensorSemanticEpoch) {
                UdpTelemetryReadEncoder(&encoder);
                deltaLeft = (int64_t)encoder.totalLeft -
                    (int64_t)g_closePairTerminal.pulseOneExitEncoderLeft;
                deltaRight = (int64_t)encoder.totalRight -
                    (int64_t)g_closePairTerminal.pulseOneExitEncoderRight;
                averageTravel = g_closePairTerminal.pulseOneExitEncoderValid != 0U &&
                    encoder.validCount != 0U && deltaLeft >= 0 && deltaRight >= 0 ?
                    ((uint64_t)deltaLeft + (uint64_t)deltaRight) / 2U : 0U;
                printf("TERMINAL event=SEGMENT_START index=2 enter_ms=%u gap_ms=%u "
                       "stable_samples=%u enc_l=%ld enc_r=%ld\r\n",
                       (unsigned int)g_closePairTerminal.enterMs,
                       (unsigned int)gapMs,
                       (unsigned int)g_closePairTerminal.stable11Samples,
                       (long)encoder.totalLeft, (long)encoder.totalRight);
                printf("ADMIN event=SECOND_SEGMENT epoch=%u enter_ms=%u gap_ms=%u\r\n",
                       (unsigned int)g_sensorSemanticEpoch,
                       (unsigned int)g_closePairTerminal.enterMs,
                       (unsigned int)gapMs);
                printf("TERMINAL event=PAIR_CHECK gap_ms=%u delta_l=%lld delta_r=%lld "
                       "avg_counts=%llu min_counts=%u encoder_valid=%u\r\n",
                       (unsigned int)gapMs,
                       (long long)deltaLeft, (long long)deltaRight,
                       (unsigned long long)averageTravel,
                       (unsigned int)TERMINAL_PAIR_MIN_TRAVEL_COUNTS,
                       (unsigned int)(g_closePairTerminal.pulseOneExitEncoderValid != 0U &&
                           encoder.validCount != 0U ? 1U : 0U));
                if (g_closePairTerminal.pulseOneExitEncoderValid != 0U &&
                    deltaLeft >= 0 && deltaRight >= 0 &&
                    averageTravel >= TERMINAL_PAIR_MIN_TRAVEL_COUNTS) {
                    printf("TERMINAL event=DOUBLE_LINE_STOP gap_ms=%u avg_counts=%llu "
                           "min_counts=%u\r\n",
                           (unsigned int)gapMs, (unsigned long long)averageTravel,
                           (unsigned int)TERMINAL_PAIR_MIN_TRAVEL_COUNTS);
                    printf("ADMIN event=CLOSE_PAIR_STOP epoch=%u gap_ms=%u "
                           "avg_counts=%llu\r\n",
                           (unsigned int)g_sensorSemanticEpoch,
                           (unsigned int)gapMs,
                           (unsigned long long)averageTravel);
                    ThreeElevenSyncLineLive();
                } else if (g_closePairTerminal.pulseOneExitEncoderValid != 0U &&
                           encoder.validCount != 0U && deltaLeft >= 0 && deltaRight >= 0) {
                    printf("TERMINAL event=PAIR_TOO_CLOSE avg_counts=%llu min_counts=%u\r\n",
                           (unsigned long long)averageTravel,
                           (unsigned int)TERMINAL_PAIR_MIN_TRAVEL_COUNTS);
                    /* The active second segment becomes the next first segment
                     * when it exits; discard only the older anchor now. */
                    g_closePairTerminal.pulseOneValid = 0U;
                    g_closePairTerminal.pulseOneExitEncoderValid = 0U;
                } else {
                    printf("TERMINAL event=PAIR_REJECT reason=%s\r\n",
                           g_closePairTerminal.pulseOneExitEncoderValid == 0U ||
                           encoder.validCount == 0U ? "ENCODER_INVALID" : "NON_FORWARD");
                    g_closePairTerminal.pulseOneValid = 0U;
                    g_closePairTerminal.pulseOneExitEncoderValid = 0U;
                }
            }
        }
        return;
    }
    if (g_closePairTerminal.active != 0U) {
        if (g_closePairTerminal.stable11Samples >= CLOSE_PAIR_SEGMENT_MIN_STABLE_SAMPLES) {
            UdpTelemetryReadEncoder(&encoder);
            g_closePairTerminal.pulseOneValid = 1U;
            g_closePairTerminal.pulseOneEpoch = g_sensorSemanticEpoch;
            g_closePairTerminal.pulseOneExitMs = nowMs;
            g_closePairTerminal.pulseOneExitEncoderValid =
                encoder.validCount != 0U ? 1U : 0U;
            g_closePairTerminal.pulseOneExitEncoderLeft = encoder.totalLeft;
            g_closePairTerminal.pulseOneExitEncoderRight = encoder.totalRight;
            printf("TERMINAL event=SEGMENT_EXIT index=1 enter_ms=%u exit_ms=%u "
                   "stable_samples=%u enc_l=%ld enc_r=%ld encoder_valid=%u\r\n",
                   (unsigned int)g_closePairTerminal.enterMs,
                   (unsigned int)nowMs,
                   (unsigned int)g_closePairTerminal.stable11Samples,
                   (long)encoder.totalLeft, (long)encoder.totalRight,
                   (unsigned int)g_closePairTerminal.pulseOneExitEncoderValid);
        }
        g_closePairTerminal.active = 0U;
        g_closePairTerminal.stable11Samples = 0U;
        g_closePairTerminal.enterMs = 0U;
    }
}

/* Concentrated V1 interval rule; only this helper changes for later tuning. */
static uint8_t ThreeElevenIsFinish(uint32_t gap12Ms, uint32_t gap23Ms)
{
    return ((uint64_t)gap12Ms * SEQ11_FINISH_GAP_RATIO_DEN) >=
           ((uint64_t)gap23Ms * SEQ11_FINISH_GAP_RATIO_NUM) ? 1U : 0U;
}

/* Sliding-window primitives deliberately retain the newest usable event(s).
 * They change semantic anchors only; ElevenEvent path history is independent. */
static void ThreeElevenSetFirst(const ElevenEvent *event)
{
    g_threeEleven.state = SEQ11_HAVE_FIRST;
    g_threeEleven.firstEventId = event->id;
    g_threeEleven.firstPathIndex = event->forwardPathStartIndex;
    g_threeEleven.firstEnterMs = event->enterMs;
    g_threeEleven.secondEventId = 0U;
    g_threeEleven.thirdEventId = 0U;
    g_threeEleven.secondPathIndex = 0xffffU;
    g_threeEleven.thirdPathIndex = 0xffffU;
    g_threeEleven.secondEnterMs = 0U;
    g_threeEleven.thirdEnterMs = 0U;
    g_threeEleven.gap12Ms = 0U;
    g_threeEleven.gap23Ms = 0U;
    g_threeEleven.firstWaitStartTick = 0U;
    g_threeEleven.secondWaitStartTick = 0U;
}

static void ThreeElevenPromoteSecond(void)
{
    const ElevenEvent *second = ElevenEventFindById(g_threeEleven.secondEventId);

    if (second == NULL) {
        /* History overflow is the only non-semantic reason an anchor vanishes. */
        ResetThreeElevenSequenceClassifier();
        return;
    }
    ThreeElevenSetFirst(second);
    g_lineLiveSeq11Decision = "PROMOTE_E2";
}

static void ThreeElevenSlideWindow(const ElevenEvent *third, uint32_t now)
{
    const ElevenEvent *second = ElevenEventFindById(g_threeEleven.secondEventId);

    if (second == NULL) {
        ThreeElevenSetFirst(third);
        g_lineLiveSeq11Decision = "SLIDE_E3";
        return;
    }
    g_threeEleven.state = SEQ11_HAVE_SECOND;
    g_threeEleven.firstEventId = second->id;
    g_threeEleven.firstPathIndex = second->forwardPathStartIndex;
    g_threeEleven.firstEnterMs = second->enterMs;
    g_threeEleven.secondEventId = third->id;
    g_threeEleven.secondPathIndex = third->forwardPathStartIndex;
    g_threeEleven.secondEnterMs = third->enterMs;
    g_threeEleven.thirdEventId = 0U;
    g_threeEleven.thirdPathIndex = 0xffffU;
    g_threeEleven.thirdEnterMs = 0U;
    g_threeEleven.gap12Ms = g_threeEleven.gap23Ms;
    g_threeEleven.gap23Ms = 0U;
    g_threeEleven.firstWaitStartTick = 0U;
    g_threeEleven.secondWaitStartTick = now;
    g_lineLiveSeq11Decision = "SLIDE";
}

static void ThreeElevenSequenceOnEnter(uint32_t now, const ElevenEvent *event)
{
    const ElevenEvent *previous;
    uint64_t finishLhs;
    uint64_t finishRhs;
    uint8_t finish;

    if (event == NULL || g_threeEleven.paused != 0U) {
        return;
    }
    if (g_threeEleven.state == SEQ11_IDLE) {
        ThreeElevenSetFirst(event);
        g_lineLiveSeq11Decision = "E1";
    } else if (g_threeEleven.state == SEQ11_HAVE_FIRST) {
        previous = ElevenEventFindById(g_threeEleven.firstEventId);
        if (previous == NULL || previous->exitMs == 0U || event->enterMs < previous->exitMs) {
            /* Keep the newest event usable even if a stale anchor was lost. */
            ThreeElevenSetFirst(event);
            g_lineLiveSeq11Decision = "REANCHOR_E1";
            return;
        }
        g_threeEleven.secondEventId = event->id;
        g_threeEleven.secondPathIndex = event->forwardPathStartIndex;
        g_threeEleven.secondEnterMs = event->enterMs;
        g_threeEleven.gap12Ms = event->enterMs - previous->exitMs;
        g_threeEleven.secondWaitStartTick = now;
        g_threeEleven.state = SEQ11_HAVE_SECOND;
        printf("SEQ11 event=PAIR e1=%u e2=%u gap12_ms=%u class=%s\r\n",
               (unsigned int)g_threeEleven.firstEventId,
               (unsigned int)g_threeEleven.secondEventId,
               (unsigned int)g_threeEleven.gap12Ms,
               g_threeEleven.gap12Ms < SEQ11_LONG_GAP_MS ? "SHORT" : "LONG");
        /* Semantic E1/E2 continues to classify fork timing only.  A terminal
         * is owned exclusively by the physical-segment detector above. */
        g_lineLiveSeq11Decision = "E2";
    } else if (g_threeEleven.state == SEQ11_HAVE_SECOND) {
        previous = ElevenEventFindById(g_threeEleven.secondEventId);
        if (previous == NULL || previous->exitMs == 0U || event->enterMs < previous->exitMs) {
            /* Preserve the newest observed event rather than dropping it. */
            ThreeElevenSetFirst(event);
            g_lineLiveSeq11Decision = "REANCHOR_E3";
            return;
        }
        g_threeEleven.thirdEventId = event->id;
        g_threeEleven.thirdPathIndex = event->forwardPathStartIndex;
        g_threeEleven.thirdEnterMs = event->enterMs;
        g_threeEleven.gap23Ms = event->enterMs - previous->exitMs;
        finishLhs = (uint64_t)g_threeEleven.gap12Ms * SEQ11_FINISH_GAP_RATIO_DEN;
        finishRhs = (uint64_t)g_threeEleven.gap23Ms * SEQ11_FINISH_GAP_RATIO_NUM;
        finish = ThreeElevenIsFinish(g_threeEleven.gap12Ms, g_threeEleven.gap23Ms);
        printf("SEQ11THREE e1=%u e2=%u e3=%u g12=%u g23=%u lhs=%llu rhs=%llu legacy_finish=%u\r\n",
               (unsigned int)g_threeEleven.firstEventId,
               (unsigned int)g_threeEleven.secondEventId,
               (unsigned int)g_threeEleven.thirdEventId,
               (unsigned int)g_threeEleven.gap12Ms, (unsigned int)g_threeEleven.gap23Ms,
               (unsigned long long)finishLhs, (unsigned long long)finishRhs,
               (unsigned int)finish);
        /* Legacy three-event comparison remains diagnostic only.  Normal
         * route STOP is reserved for the physical close-segment detector. */
        (void)finish;
        g_lineLiveSeq11Decision = "SLIDE_E3_LEGACY";
        ThreeElevenSlideWindow(event, now);
        ThreeElevenSyncLineLive();
        return;
    }
    ThreeElevenSyncLineLive();
}

/* Timeout is deliberately lower priority than the owner-loop's fresh 11 edge.
 * A same-cycle ENTER must first be evaluated as E3 (FINISH or SLIDE). */
static void ThreeElevenSequenceTick(uint32_t now, uint8_t newElevenEnterThisLoop,
                                    uint8_t freshRawState, uint8_t candidateActive)
{
    if (g_threeEleven.paused != 0U) {
        return;
    }
    if (g_threeEleven.state == SEQ11_HAVE_SECOND &&
               newElevenEnterThisLoop == 0U && candidateActive == 0U &&
               freshRawState != 0x03U &&
               AppTicksToMs(now - g_threeEleven.secondWaitStartTick) >
                   SEQ11_THIRD_WAIT_MAX_MS) {
        if (g_threeEleven.gap12Ms >= SEQ11_LONG_GAP_MS) {
            const ElevenEvent *firstEvent =
                ElevenEventFindById(g_threeEleven.firstEventId);
            g_threeElevenForkEventId = g_threeEleven.firstEventId;
            g_threeElevenForkPathIndex = g_threeEleven.firstPathIndex;
            g_threeEleven.claim = SEQ11_CLAIM_RETURN;
            g_threeEleven.paused = 1U;
            (void)AutoForkArmTwoLong(firstEvent);
            g_lineLiveSeq11Decision = "TWO_LONG_RETURN";
        } else {
            /* Short E1/E2 remains useful: E2 becomes the next anchor. */
            ThreeElevenPromoteSecond();
            ThreeElevenSyncLineLive();
            return;
        }
        ThreeElevenSyncLineLive();
        ResetThreeElevenSequenceClassifier();
        return;
    }
    ThreeElevenSyncLineLive();
}

/* A lone fully exited E1 must not remain a semantic anchor indefinitely.
 * Encoder distance starts at the physical E1 exit, never qualification or
 * ENTER, and is evaluated only during an ordinary forward TRACE epoch. */
static void ThreeElevenObserveSingleStale(uint32_t now, ElevenEventSignal signal,
                                          const ElevenEvent *event,
                                          uint8_t freshRawState)
{
    UdpEncoderTelemetryState encoder;
#if (SEQ11_SINGLE_STALE_ENCODER_TRAVEL != 0U)
    int64_t deltaLeft;
    int64_t deltaRight;
    uint64_t travel;
#endif

    (void)now;
    if (g_threeEleven.state != SEQ11_HAVE_FIRST ||
        g_threeEleven.paused != 0U ||
        g_threeEleven.secondEventId != 0U ||
        g_bpathControlState != BPATH_CONTROL_TRACE_RECORD) {
        g_threeElevenStaleEventId = 0U;
        g_threeElevenStaleExitValid = 0U;
        return;
    }
    if (signal == ELEVEN_EVENT_SIGNAL_EXIT && event != NULL &&
        event->id == g_threeEleven.firstEventId && event->exitMs != 0U) {
        /* Snapshot the actual encoder frame at semantic EXIT.  The previous
         * implementation used the most recently recorded BPATH point, which
         * can predate EXIT and falsely inflate post-E1 travel. */
        UdpTelemetryReadEncoder(&encoder);
        if (encoder.validCount != 0U) {
            g_threeElevenStaleExitLeft = encoder.totalLeft;
            g_threeElevenStaleExitRight = encoder.totalRight;
            g_threeElevenStaleEventId = event->id;
            g_threeElevenStaleExitValid = 1U;
            printf("SEQ11 event=STALE_E1_ARMED e1=%u exit_l=%ld exit_r=%ld "
                   "mode=DISABLED_NO_MM_CALIBRATION\r\n",
                   (unsigned int)event->id,
                   (long)g_threeElevenStaleExitLeft,
                   (long)g_threeElevenStaleExitRight);
        }
    }
    if (g_threeElevenStaleExitValid == 0U ||
        g_threeElevenStaleEventId != g_threeEleven.firstEventId ||
        g_elevenCandidate.active != 0U || freshRawState == 0x03U ||
        g_stableStateValid == 0U || g_stableState == 0x03U) {
        return;
    }
#if (SEQ11_SINGLE_STALE_ENCODER_TRAVEL == 0U)
    /* No physical distance conversion: never clear an E1 on an invented
     * count threshold.  E2 delivery has already run before this helper. */
    return;
#else
    UdpTelemetryReadEncoder(&encoder);
    if (encoder.validCount == 0U) {
        return;
    }
    deltaLeft = (int64_t)encoder.totalLeft - (int64_t)g_threeElevenStaleExitLeft;
    deltaRight = (int64_t)encoder.totalRight - (int64_t)g_threeElevenStaleExitRight;
    if (deltaLeft < 0 || deltaRight < 0) {
        return;
    }
    travel = ((uint64_t)deltaLeft + (uint64_t)deltaRight) / 2U;
    if (travel < SEQ11_SINGLE_STALE_ENCODER_TRAVEL) {
        return;
    }
    printf("SEQ11 event=STALE_E1_CLEAR e1=%u exit_l=%ld exit_r=%ld cur_l=%ld cur_r=%ld "
           "travel_counts_l=%lld travel_counts_r=%lld travel_counts=%llu threshold_counts=%u\r\n",
           (unsigned int)g_threeEleven.firstEventId,
           (long)g_threeElevenStaleExitLeft, (long)g_threeElevenStaleExitRight,
           (long)encoder.totalLeft, (long)encoder.totalRight,
           (long long)deltaLeft, (long long)deltaRight,
           (unsigned long long)travel,
           (unsigned int)SEQ11_SINGLE_STALE_ENCODER_TRAVEL);
    ThreeElevenResetForTrace();
#endif
}

/* Compacting is safe only before any candidate owns a source index and while
 * the sequence window has no unresolved fork semantics. */
static void BpathControlUpdateIdleRolling(void)
{
    uint8_t idle = g_bpathControlState == BPATH_CONTROL_TRACE_RECORD &&
        g_threeEleven.state == SEQ11_IDLE &&
        g_threeEleven.firstEventId == 0U &&
        g_threeEleven.secondEventId == 0U &&
        g_threeEleven.thirdEventId == 0U &&
        g_threeEleven.claim == SEQ11_CLAIM_NONE &&
        g_threeEleven.paused == 0U &&
        g_elevenCandidate.active == 0U ? 1U : 0U;
    const char *reason = g_elevenCandidate.active != 0U ?
        "ELEVEN_CANDIDATE" : (idle != 0U ? "SEQ11_IDLE" : "SEQ11_CONTEXT");

    BPathExternalSetIdleRolling(idle, reason);
}

/* Test-only first/second raw-11 classifier; it consumes neutral events only. */
typedef enum {
    DOUBLE_STOP_OBSERVER_WAIT_FIRST = 0,
    DOUBLE_STOP_OBSERVER_IN_FIRST,
    DOUBLE_STOP_OBSERVER_WAIT_POSTLINE_FWD,
    DOUBLE_STOP_OBSERVER_PROBE_SECOND,
    DOUBLE_STOP_OBSERVER_SINGLE_RETURN,
    DOUBLE_STOP_OBSERVER_DOUBLE_STOP
} DoubleStopObserverState;

static DoubleStopObserverState g_doubleStopObserverState;
static int32_t g_doubleStopFirstEnterLeft;
static int32_t g_doubleStopFirstEnterRight;
static uint32_t g_doubleStopFirstEnterMs;
static uint32_t g_doubleStopFirstExitMs;
static uint32_t g_doubleStopFirstEnterTick;
static uint32_t g_doubleStopFirstExitTick;
static uint32_t g_doubleStopFirstEventId;
static int32_t g_doubleStopProbeStartLeft;
static int32_t g_doubleStopProbeStartRight;
static uint32_t g_doubleStopProbeStartMs;

typedef enum {
    STOPLINE_CLAIM_NONE = 0,
    STOPLINE_CLAIM_SINGLE,
    STOPLINE_CLAIM_DOUBLE
} StoplineSemanticClaim;

typedef enum {
    FORK_CLAIM_NONE = 0,
    FORK_CLAIM_CANDIDATE,
    FORK_CLAIM_CONFIRMED
} ForkSemanticClaim;

typedef enum {
    TRACK_SEMANTIC_ACTION_NONE = 0,
    TRACK_SEMANTIC_ACTION_BEGIN_RETURN,
    TRACK_SEMANTIC_ACTION_FINAL_STOP
} TrackSemanticAction;

static StoplineSemanticClaim g_stoplineSemanticClaim;
static ForkSemanticClaim g_forkSemanticClaim;
static uint32_t g_stoplineClaimFirstEventId;
static uint32_t g_forkConfirmedEventId;
static void ForkClassifierReset(void);

static void DoubleStopAutoReturnObserverReset(void)
{
    g_doubleStopObserverState = DOUBLE_STOP_OBSERVER_WAIT_FIRST;
    g_doubleStopFirstEnterLeft = 0;
    g_doubleStopFirstEnterRight = 0;
    g_doubleStopFirstEnterMs = 0U;
    g_doubleStopFirstExitMs = 0U;
    g_doubleStopFirstEnterTick = 0U;
    g_doubleStopFirstExitTick = 0U;
    g_doubleStopFirstEventId = 0U;
    g_doubleStopProbeStartLeft = 0;
    g_doubleStopProbeStartRight = 0;
    g_doubleStopProbeStartMs = 0U;
    g_lineLiveStopClassState = "WAIT_FIRST";
    g_lineLiveStopClassFirstCount = 0U;
    g_lineLiveStopClassSecondCount = 0U;
    g_lineLiveStopClassSingleCount = 0U;
    g_lineLiveStopClassDoubleCount = 0U;
    g_lineLiveStopClassExpireCount = 0U;
    g_lineLiveStopClassSingleReason = "NONE";
    g_lineLiveStopClassCandidateAgeMs = 0U;
    g_lineLiveStopClassCancelCount = 0U;
    g_lineLiveStopClassCancelReason = "NONE";
    g_lineLiveStopClassProbeTicks = 0U;
    g_lineLiveStopClassProbeArmed = 0U;
    g_lineLiveStopClassLeftBoundaryCount = 0U;
    g_lineLiveStopClassFirstExitLeft = 0;
    g_lineLiveStopClassFirstExitRight = 0;
    g_lineLiveStopClassSecondEnterLeft = 0;
    g_lineLiveStopClassSecondEnterRight = 0;
    g_lineLiveStopClassDecision = "NONE";
    g_doubleStopTestReturnActive = 0U;
    g_stoplineSemanticClaim = STOPLINE_CLAIM_NONE;
    g_stoplineClaimFirstEventId = 0U;
    g_lineLiveStopClaim = "NONE";
    ForkClassifierReset();
}

/* Deactivate edge processing at lifecycle end while retaining sticky results. */
static void DoubleStopAutoReturnObserverDeactivate(void)
{
    g_doubleStopObserverState = DOUBLE_STOP_OBSERVER_DOUBLE_STOP;
    g_doubleStopTestReturnActive = 0U;
    if (g_lineLiveStopClassDecision[0] == 'N') {
        g_lineLiveStopClassState = "STOPPED";
    }
}

/*
 * Expiry is an observer-only state cleanup.  It never returns an action, so a
 * stale first raw-11 can neither stop nor begin RETURN at a distant LEFT.
 * Preserve EXPIRED in LINE_LIVE until a new independent raw-11 edge begins a
 * fresh candidate.
 */
static void DoubleStopAutoReturnObserverExpire(void)
{
    g_doubleStopObserverState = DOUBLE_STOP_OBSERVER_WAIT_FIRST;
    g_lineLiveStopClassState = "EXPIRED";
    g_lineLiveStopClassDecision = "EXPIRED";
    g_lineLiveStopClassExpireCount++;
    g_lineLiveStopClassProbeTicks = 0U;
    g_lineLiveStopClassProbeArmed = 0U;
    g_stoplineSemanticClaim = STOPLINE_CLAIM_NONE;
    g_stoplineClaimFirstEventId = 0U;
    g_lineLiveStopClaim = "NONE";
}

static void DoubleStopAutoReturnObserverBeginSingleReturn(const char *reason)
{
    g_lineLiveStopClassSingleCount++;
    g_lineLiveStopClassSingleReason = reason;
    g_lineLiveStopClassDecision = "SINGLE_RETURN";
    g_lineLiveStopClassState = "SINGLE_RETURN";
    g_doubleStopObserverState = DOUBLE_STOP_OBSERVER_SINGLE_RETURN;
    g_stoplineSemanticClaim = STOPLINE_CLAIM_SINGLE;
    g_stoplineClaimFirstEventId = g_doubleStopFirstEventId;
    g_lineLiveStopClaim = "SINGLE";
}

/* V1 supplies isolated fork runtime only; automatic confirmation is deferred. */
static void ForkClassifierObserve(ElevenEventSignal signal, const ElevenEvent *event)
{
    (void)signal;
    (void)event;
}

static void ForkClassifierReset(void)
{
    g_forkSemanticClaim = FORK_CLAIM_NONE;
    g_forkConfirmedEventId = 0U;
    g_lineLiveForkClaim = "NONE";
}

/* The only place where semantic claims become RETURN or STOP actions. */
static TrackSemanticAction ResolveTrackSemanticDecision(void)
{
#if (THREE_ELEVEN_SEQUENCE_TEST_MODE == 1)
    if (g_threeEleven.claim == SEQ11_CLAIM_STOP) {
        return TRACK_SEMANTIC_ACTION_FINAL_STOP;
    }
    if (g_threeEleven.claim == SEQ11_CLAIM_RETURN) {
        return TRACK_SEMANTIC_ACTION_BEGIN_RETURN;
    }
    return TRACK_SEMANTIC_ACTION_NONE;
#else
    if (g_stoplineSemanticClaim == STOPLINE_CLAIM_DOUBLE) {
        return TRACK_SEMANTIC_ACTION_FINAL_STOP;
    }
    if (g_stoplineSemanticClaim == STOPLINE_CLAIM_SINGLE &&
        !(g_forkSemanticClaim == FORK_CLAIM_CONFIRMED &&
          g_forkConfirmedEventId == g_stoplineClaimFirstEventId)) {
        return TRACK_SEMANTIC_ACTION_BEGIN_RETURN;
    }
    return TRACK_SEMANTIC_ACTION_NONE;
#endif
}

/*
 * Explicit higher-level ownership, such as an attended FORKTEST MARK, can
 * cancel only an undecided stopline candidate.  This is deliberately
 * idempotent and never rewinds an already-started RETURN or completed STOP.
 */
static void StoplineCandidateCancel(const char *reason)
{
    if (g_doubleStopObserverState != DOUBLE_STOP_OBSERVER_IN_FIRST &&
        g_doubleStopObserverState != DOUBLE_STOP_OBSERVER_WAIT_POSTLINE_FWD &&
        g_doubleStopObserverState != DOUBLE_STOP_OBSERVER_PROBE_SECOND) {
        return;
    }

    g_doubleStopObserverState = DOUBLE_STOP_OBSERVER_WAIT_FIRST;
    g_doubleStopFirstEnterLeft = 0;
    g_doubleStopFirstEnterRight = 0;
    g_doubleStopFirstEnterMs = 0U;
    g_doubleStopFirstExitMs = 0U;
    g_doubleStopFirstEnterTick = 0U;
    g_doubleStopFirstExitTick = 0U;
    g_doubleStopFirstEventId = 0U;
    g_doubleStopProbeStartLeft = 0;
    g_doubleStopProbeStartRight = 0;
    g_doubleStopProbeStartMs = 0U;
    g_lineLiveStopClassProbeTicks = 0U;
    g_lineLiveStopClassProbeArmed = 0U;
    g_lineLiveStopClassCandidateAgeMs = 0U;
    g_lineLiveStopClassSingleReason = "NONE";
    g_lineLiveStopClassState = "WAIT_FIRST";
    g_lineLiveStopClassDecision = "CANCELLED";
    g_lineLiveStopClassCancelCount++;
    g_lineLiveStopClassCancelReason = reason;
    g_stoplineSemanticClaim = STOPLINE_CLAIM_NONE;
    g_stoplineClaimFirstEventId = 0U;
    g_lineLiveStopClaim = "NONE";
}

/* The caller invokes this with the same fresh raw snapshot used by TRACE. */
static StoplineSemanticClaim DoubleStopAutoReturnObserverObserve(
    uint32_t now, ElevenEventSignal signal, const ElevenEvent *event,
    uint8_t stableState, TraceAction action)
{
    UdpEncoderTelemetryState encoder;
    int32_t deltaLeft;
    int32_t deltaRight;

    UdpTelemetryReadEncoder(&encoder);
    switch (g_doubleStopObserverState) {
        case DOUBLE_STOP_OBSERVER_WAIT_FIRST:
            if (signal == ELEVEN_EVENT_SIGNAL_ENTER && event != NULL) {
                g_lineLiveStopClassFirstCount++;
                g_doubleStopFirstEnterLeft = encoder.totalLeft;
                g_doubleStopFirstEnterRight = encoder.totalRight;
                g_doubleStopFirstEnterMs = event->enterMs;
                g_doubleStopFirstEnterTick = now;
                g_doubleStopFirstExitTick = 0U;
                g_doubleStopFirstEventId = event->id;
                g_lineLiveStopClassDecision = "NONE";
                g_lineLiveStopClassSingleReason = "NONE";
                g_lineLiveStopClassCandidateAgeMs = 0U;
                g_lineLiveStopClassState = "IN_FIRST";
                g_doubleStopObserverState = DOUBLE_STOP_OBSERVER_IN_FIRST;
            }
            break;

        case DOUBLE_STOP_OBSERVER_IN_FIRST:
            if (AppTicksToMs(now - g_doubleStopFirstEnterTick) >
                STOPLINE_FIRST_11_EPISODE_MAX_MS) {
                /* A single raw-11 episode wider than the local allowance is
                 * not allowed to seed a later stopline decision. */
                DoubleStopAutoReturnObserverExpire();
            } else if (signal == ELEVEN_EVENT_SIGNAL_EXIT && event != NULL &&
                       event->id == g_doubleStopFirstEventId) {
                g_lineLiveStopClassFirstExitLeft = encoder.totalLeft;
                g_lineLiveStopClassFirstExitRight = encoder.totalRight;
                g_doubleStopFirstExitMs = event->exitMs;
                g_doubleStopFirstExitTick = now;
                g_lineLiveStopClassCandidateAgeMs = 0U;
                g_lineLiveStopClassProbeTicks = 0U;
                g_lineLiveStopClassProbeArmed = 0U;
                g_lineLiveStopClassState = "WAIT_POSTLINE_FWD";
                g_doubleStopObserverState = DOUBLE_STOP_OBSERVER_WAIT_POSTLINE_FWD;
            }
            break;

        case DOUBLE_STOP_OBSERVER_WAIT_POSTLINE_FWD:
            g_lineLiveStopClassCandidateAgeMs =
                AppTicksToMs(now - g_doubleStopFirstExitTick);
            if (AppTicksToMs(now - g_doubleStopFirstExitTick) >=
                STOPLINE_POST_FIRST_MAX_GAP_MS) {
                DoubleStopAutoReturnObserverBeginSingleReturn("TIMEOUT");
                return STOPLINE_CLAIM_SINGLE;
            }
            /* Ignore post-crossbar LEFT_MIN/RIGHT_MIN until actual FWD resumes. */
            if (action == TRACE_ACTION_FORWARD) {
                g_doubleStopProbeStartLeft = encoder.totalLeft;
                g_doubleStopProbeStartRight = encoder.totalRight;
                g_doubleStopProbeStartMs = AppTicksToMs(now);
                g_lineLiveStopClassProbeTicks = 0U;
                g_lineLiveStopClassProbeArmed = 1U;
                g_lineLiveStopClassState = "PROBE_SECOND";
                g_doubleStopObserverState = DOUBLE_STOP_OBSERVER_PROBE_SECOND;
            }
            break;

        case DOUBLE_STOP_OBSERVER_PROBE_SECOND:
            g_lineLiveStopClassCandidateAgeMs =
                AppTicksToMs(now - g_doubleStopFirstExitTick);
            deltaLeft = encoder.totalLeft - g_doubleStopProbeStartLeft;
            deltaRight = encoder.totalRight - g_doubleStopProbeStartRight;
            if (deltaLeft < 0) deltaLeft = -deltaLeft;
            if (deltaRight < 0) deltaRight = -deltaRight;
            g_lineLiveStopClassProbeTicks =
                ((uint32_t)deltaLeft + (uint32_t)deltaRight) / 2U;
            if (signal == ELEVEN_EVENT_SIGNAL_ENTER && event != NULL) {
                g_lineLiveStopClassSecondCount++;
                g_lineLiveStopClassSecondEnterLeft = encoder.totalLeft;
                g_lineLiveStopClassSecondEnterRight = encoder.totalRight;
                g_lineLiveStopClassDoubleCount++;
                g_lineLiveStopClassDecision = "DOUBLE_STOP";
                g_lineLiveStopClassState = "DOUBLE_STOP";
                g_doubleStopObserverState = DOUBLE_STOP_OBSERVER_DOUBLE_STOP;
                g_stoplineSemanticClaim = STOPLINE_CLAIM_DOUBLE;
                g_stoplineClaimFirstEventId = g_doubleStopFirstEventId;
                g_lineLiveStopClaim = "DOUBLE";
                return STOPLINE_CLAIM_DOUBLE;
            }
#if (PROVISIONAL_ROUTE_BOUNDARY_TEST_MODE == 1)
            /* A stable 10 plus actual LEFT is the next route segment, not LEFT_MIN. */
            if (stableState == 0x02U && action == TRACE_ACTION_LEFT) {
                g_lineLiveStopClassLeftBoundaryCount++;
                DoubleStopAutoReturnObserverBeginSingleReturn("LEFT_BOUNDARY");
                return STOPLINE_CLAIM_SINGLE;
            }
#endif
#if (SECOND_STOP_LOOKAHEAD_TICKS > 0U)
            if (g_lineLiveStopClassProbeTicks >= SECOND_STOP_LOOKAHEAD_TICKS) {
                DoubleStopAutoReturnObserverBeginSingleReturn("LOOKAHEAD");
                return STOPLINE_CLAIM_SINGLE;
            }
#endif
            if (AppTicksToMs(now - g_doubleStopFirstExitTick) >=
                STOPLINE_POST_FIRST_MAX_GAP_MS) {
                DoubleStopAutoReturnObserverBeginSingleReturn("TIMEOUT");
                return STOPLINE_CLAIM_SINGLE;
            }
            break;

        case DOUBLE_STOP_OBSERVER_SINGLE_RETURN:
        case DOUBLE_STOP_OBSERVER_DOUBLE_STOP:
            break;

        default:
            DoubleStopAutoReturnObserverReset();
            break;
    }

    return STOPLINE_CLAIM_NONE;
}

/*
 * Passive crossbar pulse observer.  It deliberately owns no TRACE state and
 * never submits a motor or BPATH command: the event is diagnostic-only.
 */
typedef enum {
    CROSSBAR_OBSERVER_IDLE = 0,
    CROSSBAR_OBSERVER_CANDIDATE,
    CROSSBAR_OBSERVER_COOLDOWN
} CrossbarObserverState;

static CrossbarObserverState g_crossbarObserverState;
static uint8_t g_crossbarStraightArmed;
static uint8_t g_crossbarCooldownClearSeen;
static uint32_t g_crossbarLastNonFwdTick;
static uint32_t g_crossbarCandidateStartTick;
static uint32_t g_crossbarCooldownStartTick;
static uint32_t g_crossbarCandidatePreFwdMs;
static int32_t g_crossbarCandidateEncoderLeft;
static int32_t g_crossbarCandidateEncoderRight;
static uint32_t g_crossbarSequence;
static uint8_t g_crossbarReject11Active;

static int32_t CrossbarAbs(int32_t value)
{
    return value < 0 ? -value : value;
}

static void CrossbarStateText(uint8_t state, char text[3])
{
    text[0] = (state & 0x02U) != 0U ? '1' : '0';
    text[1] = (state & 0x01U) != 0U ? '1' : '0';
    text[2] = '\0';
}

static void CrossbarObserverCancel(uint32_t now, const char *reason)
{
    char text[176];

    if (g_crossbarObserverState == CROSSBAR_OBSERVER_CANDIDATE) {
        g_crossbarSequence++;
        (void)snprintf(text, sizeof(text),
            "CROSSBAR_OBS event=CANDIDATE_CANCEL seq=%u reason=%s black_ms=%u",
            (unsigned int)g_crossbarSequence, reason,
            (unsigned int)AppTicksToMs(now - g_crossbarCandidateStartTick));
        (void)UdpTelemetryQueueExperimentText(text);
    }
    g_crossbarObserverState = CROSSBAR_OBSERVER_IDLE;
    g_crossbarStraightArmed = 0U;
    g_crossbarLastNonFwdTick = now;
}

static void CrossbarObserverStop(uint32_t now, const char *reason)
{
    CrossbarObserverCancel(now, reason);
    g_crossbarObserverState = CROSSBAR_OBSERVER_IDLE;
    g_crossbarStraightArmed = 0U;
    g_crossbarCooldownClearSeen = 0U;
    g_crossbarLastNonFwdTick = now;
    g_crossbarCandidateStartTick = 0U;
    g_crossbarCooldownStartTick = 0U;
    g_crossbarCandidatePreFwdMs = 0U;
    g_crossbarCandidateEncoderLeft = 0;
    g_crossbarCandidateEncoderRight = 0;
    g_crossbarReject11Active = 0U;
}

static void CrossbarObserverBeginCandidate(uint32_t now, uint8_t rawState,
                                            uint8_t stableState)
{
    UdpEncoderTelemetryState encoder;
    char stableText[3];
    char text[200];

    UdpTelemetryReadEncoder(&encoder);
    g_crossbarObserverState = CROSSBAR_OBSERVER_CANDIDATE;
    g_crossbarCandidateStartTick = now;
    g_crossbarCandidatePreFwdMs = AppTicksToMs(now - g_crossbarLastNonFwdTick);
    g_crossbarCandidateEncoderLeft = encoder.totalLeft;
    g_crossbarCandidateEncoderRight = encoder.totalRight;
    CrossbarStateText(stableState, stableText);
    g_crossbarSequence++;
    (void)snprintf(text, sizeof(text),
        "CROSSBAR_OBS event=CANDIDATE_ENTER seq=%u raw=%u%u stable=%s pre_fwd_ms=%u",
        (unsigned int)g_crossbarSequence, (unsigned int)((rawState >> 1) & 0x01U),
        (unsigned int)(rawState & 0x01U), stableText,
        (unsigned int)g_crossbarCandidatePreFwdMs);
    (void)UdpTelemetryQueueExperimentText(text);
}

static void CrossbarObserverConfirm(uint32_t now, uint8_t exitRawState)
{
    UdpEncoderTelemetryState encoder;
    int32_t deltaLeft;
    int32_t deltaRight;
    int32_t widthTicks;
    char text[224];

    UdpTelemetryReadEncoder(&encoder);
    deltaLeft = encoder.totalLeft - g_crossbarCandidateEncoderLeft;
    deltaRight = encoder.totalRight - g_crossbarCandidateEncoderRight;
    widthTicks = (CrossbarAbs(deltaLeft) + CrossbarAbs(deltaRight)) / 2;
    g_crossbarSequence++;
    (void)snprintf(text, sizeof(text),
        "CROSSBAR_EVENT seq=%u black_ms=%u width_ticks=%ld delta_l=%ld delta_r=%ld enter_raw=11 exit_raw=%u%u pre_fwd_ms=%u",
        (unsigned int)g_crossbarSequence,
        (unsigned int)AppTicksToMs(now - g_crossbarCandidateStartTick),
        (long)widthTicks, (long)deltaLeft, (long)deltaRight,
        (unsigned int)((exitRawState >> 1) & 0x01U), (unsigned int)(exitRawState & 0x01U),
        (unsigned int)g_crossbarCandidatePreFwdMs);
    (void)UdpTelemetryQueueExperimentText(text);
    g_crossbarObserverState = CROSSBAR_OBSERVER_COOLDOWN;
    g_crossbarCooldownStartTick = now;
    g_crossbarCooldownClearSeen = 1U;
}

static void CrossbarObserverReject11(uint32_t now, uint8_t rawState,
                                     uint8_t stableState, const char *reason)
{
    char stableText[3];
    char text[216];

    if (g_crossbarReject11Active != 0U) return;
    g_crossbarReject11Active = 1U;
    CrossbarStateText(stableState, stableText);
    g_crossbarSequence++;
    (void)snprintf(text, sizeof(text),
        "CROSSBAR_OBS event=REJECT_11 seq=%u reason=%s raw=%u%u stable=%s action=%s pre_fwd_ms=%u",
        (unsigned int)g_crossbarSequence, reason,
        (unsigned int)((rawState >> 1) & 0x01U), (unsigned int)(rawState & 0x01U),
        stableText, TraceRecordDiagActionName(g_lastAction),
        (unsigned int)AppTicksToMs(now - g_crossbarLastNonFwdTick));
    (void)UdpTelemetryQueueExperimentText(text);
}

static void CrossbarObserverObserveNotTrace(uint32_t now, WifiIotGpioValue rawLeft,
                                             WifiIotGpioValue rawRight)
{
    uint8_t rawState = (uint8_t)((rawLeft == WIFI_IOT_GPIO_VALUE1 ? 2U : 0U) |
                                 (rawRight == WIFI_IOT_GPIO_VALUE1 ? 1U : 0U));
    uint8_t stableState = g_stableStateValid != 0U ? g_stableState : rawState;

    if (rawState == 0x03U) {
        CrossbarObserverReject11(now, rawState, stableState, "NOT_TRACE");
    } else {
        g_crossbarReject11Active = 0U;
    }
}

static void CrossbarObserverObserve(uint32_t now, WifiIotGpioValue rawLeft,
                                    WifiIotGpioValue rawRight)
{
    uint8_t rawState = (uint8_t)((rawLeft == WIFI_IOT_GPIO_VALUE1 ? 2U : 0U) |
                                 (rawRight == WIFI_IOT_GPIO_VALUE1 ? 1U : 0U));
    uint8_t stableState = g_stableStateValid != 0U ? g_stableState : rawState;
    uint8_t blackPresent = rawState == 0x03U || stableState == 0x03U;
    uint32_t blackMs;

    if (rawState != 0x03U) g_crossbarReject11Active = 0U;
    if (g_crossbarObserverState == CROSSBAR_OBSERVER_COOLDOWN) {
        if (rawState == 0x03U) {
            CrossbarObserverReject11(now, rawState, stableState, "COOLDOWN");
        }
        if (blackPresent == 0U) g_crossbarCooldownClearSeen = 1U;
        if (g_crossbarCooldownClearSeen != 0U &&
            AppTicksToMs(now - g_crossbarCooldownStartTick) >= CROSSBAR_COOLDOWN_MS) {
            g_crossbarObserverState = CROSSBAR_OBSERVER_IDLE;
            g_crossbarStraightArmed = 0U;
            g_crossbarLastNonFwdTick = now;
        }
        return;
    }

    if (g_crossbarObserverState == CROSSBAR_OBSERVER_CANDIDATE) {
        blackMs = AppTicksToMs(now - g_crossbarCandidateStartTick);
        if (rawState == 0x01U || rawState == 0x02U) {
            CrossbarObserverCancel(now, "DIRECTIONAL_RAW");
        } else if (blackMs >= CROSSBAR_MAX_EVENT_MS) {
            CrossbarObserverCancel(now, "MAX_EVENT_MS");
        } else if (blackPresent == 0U) {
            if (blackMs >= CROSSBAR_MIN_BLACK_MS) {
                CrossbarObserverConfirm(now, rawState);
            } else {
                CrossbarObserverCancel(now, "BLACK_TOO_SHORT");
            }
        }
        return;
    }

    /* The candidate is admitted from the preceding FWD-only window. */
    if (rawState == 0x03U && g_crossbarStraightArmed != 0U) {
        CrossbarObserverBeginCandidate(now, rawState, stableState);
        return;
    }
    if (rawState == 0x03U) {
        CrossbarObserverReject11(now, rawState, stableState,
            g_lastAction != TRACE_ACTION_FORWARD ? "NON_FWD_RECENT" : "NOT_STRAIGHT_ARMED");
        return;
    }
    if (g_lastAction != TRACE_ACTION_FORWARD) {
        g_crossbarLastNonFwdTick = now;
        g_crossbarStraightArmed = 0U;
    } else if (g_crossbarStraightArmed == 0U &&
               AppTicksToMs(now - g_crossbarLastNonFwdTick) >= CROSSBAR_PRE_FWD_MS) {
        char text[144];

        g_crossbarStraightArmed = 1U;
        g_crossbarSequence++;
        (void)snprintf(text, sizeof(text),
            "CROSSBAR_OBS event=ARM seq=%u pre_fwd_ms=%u",
            (unsigned int)g_crossbarSequence,
            (unsigned int)AppTicksToMs(now - g_crossbarLastNonFwdTick));
        (void)UdpTelemetryQueueExperimentText(text);
    }
}

static void TraceRecordDiagPublish(int rawLeft, int rawRight, uint32_t now)
{
    char text[180];
    int startEvent = (g_traceRecordDiagStartPending != 0U);

    if (startEvent == 0 &&
        (uint32_t)(now - g_traceRecordDiagLastTick) <
        AppMsToTicks(TRACE_RECORD_DIAG_PERIOD_MS)) {
        return;
    }
    (void)snprintf(text, sizeof(text),
        "TRACE_DIAG%s profile=GOLDEN_RESTORE_V1 raw_l=%d raw_r=%d stable_l=%u stable_r=%u action=%s cmd_l=%d cmd_r=%d last_corr=%s",
        startEvent != 0 ? " event=START" : "",
        rawLeft, rawRight,
        (unsigned int)((g_stableState >> 1) & 0x01U),
        (unsigned int)(g_stableState & 0x01U),
        TraceRecordDiagActionName(g_lastAction),
        g_motorLeftCommand, g_motorRightCommand,
        TraceRecordDiagCorrectionName(g_lastCorrection));
    (void)UdpTelemetryQueueExperimentText(text);
    g_traceRecordDiagStartPending = 0U;
    g_traceRecordDiagLastTick = now;
}

typedef struct {
    uint8_t active;
    TraceCorrection direction;
    uint32_t startTick;
    uint32_t lastTick;
    uint32_t rawMs[4];
    uint32_t stableMs[4];
} TraceCurveSegment;

static TraceCurveSegment g_traceCurveSegment;

static void TraceCurveStateText(uint8_t state, char text[3])
{
    text[0] = (state & 0x02U) != 0U ? '1' : '0';
    text[1] = (state & 0x01U) != 0U ? '1' : '0';
    text[2] = '\0';
}

static TraceCorrection TraceCurveActionDirection(TraceAction action)
{
    if (action == TRACE_ACTION_LEFT || action == TRACE_ACTION_RECOVER_LEFT) {
        return TRACE_CORRECTION_LEFT;
    }
    if (action == TRACE_ACTION_RIGHT || action == TRACE_ACTION_RECOVER_RIGHT ||
        action == TRACE_ACTION_RIGHT_11) {
        return TRACE_CORRECTION_RIGHT;
    }
    return TRACE_CORRECTION_NONE;
}

static void TraceCurveAccumulate(uint32_t now, uint8_t rawState, uint8_t stableState)
{
    uint32_t elapsed;

    if (g_traceCurveSegment.active == 0U) {
        return;
    }
    elapsed = AppTicksToMs(now - g_traceCurveSegment.lastTick);
    if (rawState < 4U) {
        g_traceCurveSegment.rawMs[rawState] += elapsed;
    }
    if (stableState < 4U) {
        g_traceCurveSegment.stableMs[stableState] += elapsed;
    }
    g_traceCurveSegment.lastTick = now;
}

static void TraceCurveReset(void)
{
    uint8_t index;

    g_traceCurveSegment.active = 0U;
    g_traceCurveSegment.direction = TRACE_CORRECTION_NONE;
    g_traceCurveSegment.startTick = 0U;
    g_traceCurveSegment.lastTick = 0U;
    for (index = 0U; index < 4U; index++) {
        g_traceCurveSegment.rawMs[index] = 0U;
        g_traceCurveSegment.stableMs[index] = 0U;
    }
}

static void TraceCurveStart(uint32_t now, TraceCorrection direction)
{
    char text[128];

    TraceCurveReset();
    g_traceCurveSegment.active = 1U;
    g_traceCurveSegment.direction = direction;
    g_traceCurveSegment.startTick = now;
    g_traceCurveSegment.lastTick = now;
    (void)snprintf(text, sizeof(text),
        "TRACE_CURVE_EVENT event=CORR_START dir=%s ms=%u",
        TraceRecordDiagCorrectionName(direction), (unsigned int)AppTicksToMs(now));
    (void)UdpTelemetryQueueExperimentText(text);
}

static void TraceCurveEnd(uint32_t now, const char *reason)
{
    char eventText[192];
    char segmentText[256];
    char rawText[3];
    char stableText[3];
    uint8_t rawState;
    uint8_t stableState;
    uint32_t duration;

    if (g_traceCurveSegment.active == 0U) {
        return;
    }
    rawState = (uint8_t)((g_lineLiveRawLeft << 1) | g_lineLiveRawRight);
    stableState = g_stableStateValid != 0 ? g_stableState : rawState;
    TraceCurveAccumulate(now, rawState, stableState);
    duration = AppTicksToMs(now - g_traceCurveSegment.startTick);
    TraceCurveStateText(rawState, rawText);
    TraceCurveStateText(stableState, stableText);
    (void)snprintf(eventText, sizeof(eventText),
        "TRACE_CURVE_EVENT event=CORR_END dir=%s duration_ms=%u end_raw=%s end_stable=%s reason=%s",
        TraceRecordDiagCorrectionName(g_traceCurveSegment.direction),
        (unsigned int)duration, rawText, stableText, reason);
    (void)UdpTelemetryQueueExperimentText(eventText);
    (void)snprintf(segmentText, sizeof(segmentText),
        "TRACE_CURVE_SEGMENT dir=%s duration_ms=%u raw00_ms=%u raw10_ms=%u raw01_ms=%u raw11_ms=%u stable00_ms=%u stable10_ms=%u stable01_ms=%u stable11_ms=%u end_reason=%s",
        TraceRecordDiagCorrectionName(g_traceCurveSegment.direction), (unsigned int)duration,
        (unsigned int)g_traceCurveSegment.rawMs[0], (unsigned int)g_traceCurveSegment.rawMs[2],
        (unsigned int)g_traceCurveSegment.rawMs[1], (unsigned int)g_traceCurveSegment.rawMs[3],
        (unsigned int)g_traceCurveSegment.stableMs[0], (unsigned int)g_traceCurveSegment.stableMs[2],
        (unsigned int)g_traceCurveSegment.stableMs[1], (unsigned int)g_traceCurveSegment.stableMs[3], reason);
    (void)UdpTelemetryQueueExperimentText(segmentText);
    TraceCurveReset();
}

static void TraceCurveObserve(uint32_t now, WifiIotGpioValue rawLeft,
                              WifiIotGpioValue rawRight)
{
    char text[224];
    char rawText[3];
    char stableText[3];
    const char *reason = NULL;
    uint8_t rawState = (uint8_t)((rawLeft == WIFI_IOT_GPIO_VALUE1 ? 2U : 0U) |
                                 (rawRight == WIFI_IOT_GPIO_VALUE1 ? 1U : 0U));
    uint8_t stableState = g_stableStateValid != 0 ? g_stableState : rawState;
    TraceCorrection direction = TraceCurveActionDirection(g_lastAction);
    uint32_t correctionMs = g_activeCorrectionDirection == TRACE_CORRECTION_NONE ? 0U :
        AppTicksToMs(now - g_activeCorrectionStartTick);

    if (g_traceCurveSegment.active != 0U) {
        TraceCurveAccumulate(now, rawState, stableState);
        if (direction == TRACE_CORRECTION_NONE) {
            reason = (g_lastAction == TRACE_ACTION_COUNTER_LEFT ||
                      g_lastAction == TRACE_ACTION_COUNTER_RIGHT) ? "COUNTER_11" : "FWD_00";
        } else if (direction != g_traceCurveSegment.direction) {
            reason = "OPPOSITE_RAW";
        }
        if (reason != NULL) {
            TraceCurveEnd(now, reason);
        }
    }
    if (direction != TRACE_CORRECTION_NONE && g_traceCurveSegment.active == 0U) {
        TraceCurveStart(now, direction);
    }
    TraceCurveStateText(rawState, rawText);
    TraceCurveStateText(stableState, stableText);
    (void)snprintf(text, sizeof(text),
        "TRACE_CURVE ms=%u raw=%s stable=%s action=%s cmd_l=%d cmd_r=%d corr_dir=%s corr_ms=%u bias=%s last_corr=%s",
        (unsigned int)AppTicksToMs(now), rawText, stableText,
        TraceRecordDiagActionName(g_lastAction), g_motorLeftCommand, g_motorRightCommand,
        TraceRecordDiagCorrectionName(g_activeCorrectionDirection), (unsigned int)correctionMs,
        g_lineLiveBias, TraceRecordDiagCorrectionName(g_lastCorrection));
    (void)UdpTelemetryQueueExperimentText(text);
}
#endif

#if (TRACE_OPEN_LOOP_STRAIGHT_TEST_MODE == 1)
static uint32_t g_traceOpenLoopLastTick;

/* Outer diagnostic override for BPATH TRACE_RECORD only: sensor sampling and
 * debounce remain active, but the final forward command is fixed at 100/100. */
static void TraceOpenLoopStraightStep(int rawLeft, int rawRight, uint32_t now)
{
    char text[160];

    TraceApplyAction(TRACE_ACTION_FORWARD);
    if ((uint32_t)(now - g_traceOpenLoopLastTick) < AppMsToTicks(100U)) {
        return;
    }
    (void)snprintf(text, sizeof(text),
        "TRACE_OPENLOOP ms=%u raw_l=%d raw_r=%d state=%u%u cmd_l=100 cmd_r=100",
        (unsigned int)((uint64_t)now * 1000U / osKernelGetTickFreq()),
        rawLeft, rawRight,
        (unsigned int)((g_stableState >> 1) & 0x01U),
        (unsigned int)(g_stableState & 0x01U));
    (void)UdpTelemetryQueueExperimentText(text);
    g_traceOpenLoopLastTick = now;
}
#endif

static void TraceResetDebounce(void)
{
    g_stableState = 0U;
    g_candidateState = 0U;
    g_candidateSamples = 0U;
    g_stableStateValid = 0;
    TraceResetState11Counter();
    g_activeCorrectionDirection = TRACE_CORRECTION_NONE;
    LineLiveSetCorrection("NONE", 0U);
}

static void TraceResetState11Counter(void)
{
    g_state11CounterActive = 0U;
    g_state11CounterDirection = TRACE_CORRECTION_NONE;
}

static void TraceSetActiveCorrection(TraceCorrection direction, uint32_t now)
{
    if (g_activeCorrectionDirection != direction) {
        g_activeCorrectionDirection = direction;
        g_activeCorrectionStartTick = now;
        LineLiveSetCorrection(direction == TRACE_CORRECTION_LEFT ? "LEFT" : "RIGHT", now);
    }
}

static void TraceClearActiveCorrection(void)
{
    g_activeCorrectionDirection = TRACE_CORRECTION_NONE;
    LineLiveSetCorrection("NONE", 0U);
}

static void TraceResetRecovery(void)
{
    g_recoveryCorrection = TRACE_CORRECTION_NONE;
    g_recoveryStartTick = 0U;
    g_recoveryActive = 0;
}

static int TraceUpdateStableState(WifiIotGpioValue left, WifiIotGpioValue right)
{
    uint8_t rawState = 0U;

    if (left == WIFI_IOT_GPIO_VALUE1) {
        rawState |= 0x02U;
    }
    if (right == WIFI_IOT_GPIO_VALUE1) {
        rawState |= 0x01U;
    }

    if (g_candidateSamples == 0U || rawState != g_candidateState) {
        g_candidateState = rawState;
        g_candidateSamples = 1U;
        return 0;
    }

    if (g_candidateSamples < 2U) {
        g_candidateSamples++;
    }

    if (g_candidateSamples == 2U &&
        (g_stableStateValid == 0 || g_stableState != g_candidateState)) {
        g_stableState = g_candidateState;
        g_stableStateValid = 1;
        return 1;
    }

    return 0;
}

#if (TRACE_STEP_RESPONSE_TEST_MODE == 1)
static const char *TraceStepResponseDirectionName(TraceStepResponseCommand command)
{
    if (command == TRACE_STEP_RESPONSE_COMMAND_LEFT_100 ||
        command == TRACE_STEP_RESPONSE_COMMAND_LEFT_200 ||
        command == TRACE_STEP_RESPONSE_COMMAND_LEFT_300) {
        return "LEFT";
    }
    return "RIGHT";
}

static uint32_t TraceStepResponseDurationMs(TraceStepResponseCommand command)
{
    if (command == TRACE_STEP_RESPONSE_COMMAND_LEFT_100 ||
        command == TRACE_STEP_RESPONSE_COMMAND_RIGHT_100) return 100U;
    if (command == TRACE_STEP_RESPONSE_COMMAND_LEFT_200 ||
        command == TRACE_STEP_RESPONSE_COMMAND_RIGHT_200) return 200U;
    return 300U;
}

static uint8_t TraceStepResponseRequiredState(TraceStepResponseCommand command)
{
    return (TraceStepResponseDirectionName(command)[0] == 'L') ? 0x02U : 0x01U;
}

static void TraceStepResponseCommands(TraceStepResponseCommand command,
                                      int *leftCommand, int *rightCommand)
{
    if (TraceStepResponseDirectionName(command)[0] == 'L') {
        *leftCommand = TRACE_INNER_SPEED;
        *rightCommand = TRACE_OUTER_SPEED;
    } else {
        *leftCommand = TRACE_OUTER_SPEED;
        *rightCommand = TRACE_INNER_SPEED;
    }
}

static void TraceStepResponsePublishSample(const char *event, uint32_t elapsedMs,
                                            const char *direction, int rawLeft, int rawRight,
                                            int leftCommand, int rightCommand)
{
    char text[224];

    (void)snprintf(text, sizeof(text),
        "%s elapsed_ms=%u direction=%s raw_l=%d raw_r=%d stable_l=%u stable_r=%u state=%u%u cmd_l=%d cmd_r=%d",
        event, (unsigned int)elapsedMs, direction, rawLeft, rawRight,
        (unsigned int)((g_stableState >> 1) & 0x01U),
        (unsigned int)(g_stableState & 0x01U),
        (unsigned int)((g_stableState >> 1) & 0x01U),
        (unsigned int)(g_stableState & 0x01U), leftCommand, rightCommand);
    (void)UdpTelemetryQueueExperimentText(text);
}

static void TraceStepResponsePublishCoast(uint32_t elapsedMs, const char *direction,
                                          int rawLeft, int rawRight)
{
    char text[224];

    (void)snprintf(text, sizeof(text),
        "TRACE_STEP_COAST elapsed_ms=%u direction=%s raw_l=%d raw_r=%d stable_l=%u stable_r=%u state=%u%u motor_l=0 motor_r=0",
        (unsigned int)elapsedMs, direction, rawLeft, rawRight,
        (unsigned int)((g_stableState >> 1) & 0x01U),
        (unsigned int)(g_stableState & 0x01U),
        (unsigned int)((g_stableState >> 1) & 0x01U),
        (unsigned int)(g_stableState & 0x01U));
    (void)UdpTelemetryQueueExperimentText(text);
}

static void TraceStepResponseStep(uint32_t now, int sensorValid,
                                  WifiIotGpioValue rawLeft, WifiIotGpioValue rawRight,
                                  int *leftCommand, int *rightCommand)
{
    TraceStepResponseCommand command = g_traceStepResponsePendingCommand;
    uint32_t elapsedMs;

    *leftCommand = 0;
    *rightCommand = 0;
    g_traceStepResponsePendingCommand = TRACE_STEP_RESPONSE_COMMAND_NONE;

    if (sensorValid != 0) {
        (void)TraceUpdateStableState(rawLeft, rawRight);
    } else {
        TraceResetDebounce();
    }

    if (command == TRACE_STEP_RESPONSE_COMMAND_STOP) {
        g_traceStepResponseState = TRACE_STEP_RESPONSE_IDLE;
        g_traceStepResponseActiveCommand = TRACE_STEP_RESPONSE_COMMAND_NONE;
        (void)UdpTelemetryQueueExperimentText("TRACE_STEP event=STOP state=IDLE");
        return;
    }

    if (g_traceStepResponseState == TRACE_STEP_RESPONSE_IDLE) {
        if (command == TRACE_STEP_RESPONSE_COMMAND_NONE) {
            return;
        }
        if (sensorValid == 0 || g_stableStateValid == 0 ||
            g_stableState != TraceStepResponseRequiredState(command)) {
            char text[152];

            (void)snprintf(text, sizeof(text),
                "TRACE_STEP event=REJECT direction=%s duration_ms=%u state=%u%u reason=NEED_%s",
                TraceStepResponseDirectionName(command),
                (unsigned int)TraceStepResponseDurationMs(command),
                (unsigned int)((g_stableState >> 1) & 0x01U),
                (unsigned int)(g_stableState & 0x01U),
                TraceStepResponseDirectionName(command)[0] == 'L' ? "10" : "01");
            (void)UdpTelemetryQueueExperimentText(text);
            return;
        }
        g_traceStepResponseState = TRACE_STEP_RESPONSE_PULSE;
        g_traceStepResponseActiveCommand = command;
        g_traceStepResponseStartTick = now;
        TraceStepResponseCommands(command, leftCommand, rightCommand);
        {
            char text[176];

            (void)snprintf(text, sizeof(text),
                "TRACE_STEP event=BEGIN direction=%s duration_ms=%u pre_state=%u%u cmd_l=%d cmd_r=%d",
                TraceStepResponseDirectionName(command),
                (unsigned int)TraceStepResponseDurationMs(command),
                (unsigned int)((g_stableState >> 1) & 0x01U),
                (unsigned int)(g_stableState & 0x01U), *leftCommand, *rightCommand);
            (void)UdpTelemetryQueueExperimentText(text);
        }
        TraceStepResponsePublishSample("TRACE_STEP_SAMPLE", 0U,
                                       TraceStepResponseDirectionName(command),
                                       (int)rawLeft, (int)rawRight,
                                       *leftCommand, *rightCommand);
        return;
    }

    elapsedMs = AppTicksToMs(now - g_traceStepResponseStartTick);
    if (g_traceStepResponseState == TRACE_STEP_RESPONSE_PULSE) {
        if (elapsedMs >= TraceStepResponseDurationMs(g_traceStepResponseActiveCommand)) {
            char text[160];

            g_traceStepResponseState = TRACE_STEP_RESPONSE_COAST;
            g_traceStepResponseStartTick = now;
            (void)snprintf(text, sizeof(text),
                "TRACE_STEP event=PULSE_END direction=%s duration_ms=%u state=%u%u",
                TraceStepResponseDirectionName(g_traceStepResponseActiveCommand),
                (unsigned int)TraceStepResponseDurationMs(g_traceStepResponseActiveCommand),
                (unsigned int)((g_stableState >> 1) & 0x01U),
                (unsigned int)(g_stableState & 0x01U));
            (void)UdpTelemetryQueueExperimentText(text);
            TraceStepResponsePublishCoast(0U,
                                          TraceStepResponseDirectionName(g_traceStepResponseActiveCommand),
                                          (int)rawLeft, (int)rawRight);
            return;
        }
        TraceStepResponseCommands(g_traceStepResponseActiveCommand, leftCommand, rightCommand);
        TraceStepResponsePublishSample("TRACE_STEP_SAMPLE", elapsedMs,
                                       TraceStepResponseDirectionName(g_traceStepResponseActiveCommand),
                                       (int)rawLeft, (int)rawRight,
                                       *leftCommand, *rightCommand);
        return;
    }

    if (elapsedMs >= TRACE_STEP_RESPONSE_COAST_MS) {
        char text[160];

        (void)snprintf(text, sizeof(text),
            "TRACE_STEP event=END direction=%s duration_ms=%u final_state=%u%u",
            TraceStepResponseDirectionName(g_traceStepResponseActiveCommand),
            (unsigned int)TraceStepResponseDurationMs(g_traceStepResponseActiveCommand),
            (unsigned int)((g_stableState >> 1) & 0x01U),
            (unsigned int)(g_stableState & 0x01U));
        (void)UdpTelemetryQueueExperimentText(text);
        g_traceStepResponseState = TRACE_STEP_RESPONSE_IDLE;
        g_traceStepResponseActiveCommand = TRACE_STEP_RESPONSE_COMMAND_NONE;
        return;
    }
    TraceStepResponsePublishCoast(elapsedMs,
                                  TraceStepResponseDirectionName(g_traceStepResponseActiveCommand),
                                  (int)rawLeft, (int)rawRight);
}
#endif

#if (TRACE_REVERSE_TEST_MODE == 0) && (TRACE_REVERSE_V2_TEST_MODE == 0) && \
    (TRACE_REVERSE_V3_TEST_MODE == 0) && (TRACE_REVERSE_V4_TEST_MODE == 0)
__attribute__((unused)) static void TraceControlStep(WifiIotGpioValue left, WifiIotGpioValue right, uint32_t now)
{
    uint8_t rawState = (uint8_t)((left == WIFI_IOT_GPIO_VALUE1 ? 2U : 0U) |
                                 (right == WIFI_IOT_GPIO_VALUE1 ? 1U : 0U));
    uint32_t correctionElapsed = g_activeCorrectionDirection == TRACE_CORRECTION_NONE ?
        0U : AppTicksToMs(now - g_activeCorrectionStartTick);

    /* Fast-entry is intentionally raw and asymmetric: directional evidence
     * starts a physical correction immediately; stable debounce remains intact. */
    if (rawState == 0x02U) {
        TraceResetRecovery();
        TraceResetState11Counter();
        TraceSetActiveCorrection(TRACE_CORRECTION_LEFT, now);
        g_lastCorrection = TRACE_CORRECTION_LEFT;
        TraceApplyAction(TRACE_ACTION_LEFT);
        return;
    }
    if (rawState == 0x01U) {
        TraceResetRecovery();
        TraceResetState11Counter();
        TraceSetActiveCorrection(TRACE_CORRECTION_RIGHT, now);
        g_lastCorrection = TRACE_CORRECTION_RIGHT;
        TraceApplyAction(TRACE_ACTION_RIGHT);
        return;
    }

    /* A correction must move the chassis before 00/11 may cancel or counter it. */
    if (g_activeCorrectionDirection != TRACE_CORRECTION_NONE &&
        correctionElapsed < TRACE_CORRECTION_MIN_HOLD_MS) {
        if (g_activeCorrectionDirection == TRACE_CORRECTION_LEFT) {
            TraceApplyAction(TRACE_ACTION_RECOVER_LEFT);
        } else {
            TraceApplyAction(TRACE_ACTION_RECOVER_RIGHT);
        }
        return;
    }

    if (g_stableState == 0x03U) {
        /* Right-curve forensics show stable 11 is the dominant in-curve
         * observation on the failed RIGHT mirror. Keep physical RIGHT until
         * clear 00 or raw 10 proves the opposite direction. LEFT keeps its
         * existing 11 counter-steer behavior for this one-sided experiment. */
        if (g_activeCorrectionDirection == TRACE_CORRECTION_RIGHT) {
            TraceResetRecovery();
            TraceResetState11Counter();
            TraceApplyAction(TRACE_ACTION_RIGHT_11);
            return;
        }
        /* In this manual-return debug profile, 11 is not a line marker and
         * does not hold the old turn. Entering 11 latches the opposite of
         * the preceding normal correction once, preventing 30 ms flip-flop. */
        TraceResetRecovery();
        if (g_state11CounterActive == 0U) {
            g_state11CounterActive = 1U;
            if (g_lastCorrection == TRACE_CORRECTION_LEFT) {
                g_state11CounterDirection = TRACE_CORRECTION_RIGHT;
            } else if (g_lastCorrection == TRACE_CORRECTION_RIGHT) {
                g_state11CounterDirection = TRACE_CORRECTION_LEFT;
            } else {
                g_state11CounterDirection = TRACE_CORRECTION_NONE;
            }
        }
        if (g_state11CounterDirection == TRACE_CORRECTION_LEFT) {
            TraceSetActiveCorrection(TRACE_CORRECTION_LEFT, now);
            TraceApplyAction(TRACE_ACTION_COUNTER_LEFT);
        } else if (g_state11CounterDirection == TRACE_CORRECTION_RIGHT) {
            TraceSetActiveCorrection(TRACE_CORRECTION_RIGHT, now);
            TraceApplyAction(TRACE_ACTION_COUNTER_RIGHT);
        } else {
            TraceClearActiveCorrection();
            TraceApplyAction(TRACE_ACTION_FORWARD);
        }
        return;
    }

    TraceResetState11Counter();

    if (g_stableState == 0x00U) {
        TraceClearActiveCorrection();
        if (g_recoveryActive == 0 &&
            (g_lastAction == TRACE_ACTION_LEFT ||
             g_lastAction == TRACE_ACTION_RIGHT ||
             g_lastAction == TRACE_ACTION_HOLD_LEFT ||
             g_lastAction == TRACE_ACTION_HOLD_RIGHT)) {
            g_recoveryCorrection = g_lastCorrection;
            g_recoveryStartTick = now;
            g_recoveryActive = 1;
        }

        if (g_recoveryActive != 0 &&
            (uint32_t)(now - g_recoveryStartTick) <
            AppMsToTicks(TRACE_RECOVERY_HOLD_MS)) {
            if (g_recoveryCorrection == TRACE_CORRECTION_LEFT) {
                TraceApplyAction(TRACE_ACTION_RECOVER_LEFT);
            } else if (g_recoveryCorrection == TRACE_CORRECTION_RIGHT) {
                TraceApplyAction(TRACE_ACTION_RECOVER_RIGHT);
            } else {
                TraceApplyAction(TRACE_ACTION_FORWARD);
            }
        } else {
            TraceResetRecovery();
            TraceApplyAction(TRACE_ACTION_FORWARD);
        }
    } else if (g_stableState == 0x02U) {
        TraceResetRecovery();
        TraceSetActiveCorrection(TRACE_CORRECTION_LEFT, now);
        g_lastCorrection = TRACE_CORRECTION_LEFT;
        TraceApplyAction(TRACE_ACTION_LEFT);
    } else if (g_stableState == 0x01U) {
        TraceResetRecovery();
        TraceSetActiveCorrection(TRACE_CORRECTION_RIGHT, now);
        g_lastCorrection = TRACE_CORRECTION_RIGHT;
        TraceApplyAction(TRACE_ACTION_RIGHT);
    } else {
        TraceApplyAction(TRACE_ACTION_STOP);
    }
}
#endif

#if (TRACE_REVERSE_TEST_MODE == 1)
typedef enum {
    REVERSE_ACTION_STOP = 0,
    REVERSE_ACTION_FORWARD,
    REVERSE_ACTION_LEFT,
    REVERSE_ACTION_RIGHT
} ReverseTraceAction;

static ReverseTraceAction g_reverseAction = REVERSE_ACTION_STOP;
static TraceCorrection g_reverseLastCorrection = TRACE_CORRECTION_NONE;
static uint8_t g_reverseRawLeft;
static uint8_t g_reverseRawRight;
static uint8_t g_reversePreviousSensor;
static uint8_t g_reverseCurrentSensor;
static int g_reverseSensorValid;
static uint16_t g_reverseConsecutive00;
static uint16_t g_reverseConsecutive01;
static uint16_t g_reverseConsecutive10;
static uint16_t g_reverseConsecutive11;
static uint32_t g_reverseLastTelemetryTick;
static int g_reverseStartPublished;

static UdpReverseAction ReverseToUdpAction(ReverseTraceAction action)
{
    switch (action) {
        case REVERSE_ACTION_FORWARD:
            return UDP_REVERSE_ACTION_FORWARD;
        case REVERSE_ACTION_LEFT:
            return UDP_REVERSE_ACTION_LEFT;
        case REVERSE_ACTION_RIGHT:
            return UDP_REVERSE_ACTION_RIGHT;
        default:
            return UDP_REVERSE_ACTION_STOP;
    }
}

static void ReverseTracePublish(UdpReverseEvent event, uint32_t now)
{
    UdpReverseTelemetryState state;

    state.event = event;
    state.rawLeft = g_reverseRawLeft;
    state.rawRight = g_reverseRawRight;
    state.sensorState = g_stableState;
    state.previousSensorState = g_reversePreviousSensor;
    state.lastCorrection = (uint8_t)g_reverseLastCorrection;
    state.action = ReverseToUdpAction(g_reverseAction);
    state.leftCommand = g_motorLeftCommand;
    state.rightCommand = g_motorRightCommand;
    state.consecutive00 = g_reverseConsecutive00;
    state.consecutive01 = g_reverseConsecutive01;
    state.consecutive10 = g_reverseConsecutive10;
    state.consecutive11 = g_reverseConsecutive11;
    state.sequence = 0U;
    UdpTelemetryUpdateReverse(&state);
    g_reverseLastTelemetryTick = now;
}

static void ReverseTraceReset(uint32_t now)
{
    g_reverseAction = REVERSE_ACTION_STOP;
    g_reverseLastCorrection = TRACE_CORRECTION_NONE;
    g_reverseRawLeft = 0U;
    g_reverseRawRight = 0U;
    g_reversePreviousSensor = 0U;
    g_reverseCurrentSensor = 0U;
    g_reverseSensorValid = 0;
    g_reverseConsecutive00 = 0U;
    g_reverseConsecutive01 = 0U;
    g_reverseConsecutive10 = 0U;
    g_reverseConsecutive11 = 0U;
    g_reverseLastTelemetryTick = now;
    g_reverseStartPublished = 0;
}

static void ReverseTraceApplyAction(ReverseTraceAction action)
{
    if (action == g_reverseAction && g_motorCommandValid != 0) {
        return;
    }

    switch (action) {
        case REVERSE_ACTION_FORWARD:
            TraceSendMotorCommand(-TRACE_REVERSE_FORWARD_SPEED,
                                  -TRACE_REVERSE_FORWARD_SPEED, 0);
            printf("REVERSE forward L=%d R=%d\r\n", -TRACE_REVERSE_FORWARD_SPEED,
                   -TRACE_REVERSE_FORWARD_SPEED);
            break;
        case REVERSE_ACTION_LEFT:
            /* Candidate reverse kinematics: validate on the car before use in recovery. */
            TraceSendMotorCommand(-TRACE_REVERSE_OUTER_SPEED,
                                  -TRACE_REVERSE_INNER_SPEED, 0);
            printf("REVERSE left L=%d R=%d\r\n", -TRACE_REVERSE_OUTER_SPEED,
                   -TRACE_REVERSE_INNER_SPEED);
            break;
        case REVERSE_ACTION_RIGHT:
            /* Candidate reverse kinematics: validate on the car before use in recovery. */
            TraceSendMotorCommand(-TRACE_REVERSE_INNER_SPEED,
                                  -TRACE_REVERSE_OUTER_SPEED, 0);
            printf("REVERSE right L=%d R=%d\r\n", -TRACE_REVERSE_INNER_SPEED,
                   -TRACE_REVERSE_OUTER_SPEED);
            break;
        case REVERSE_ACTION_STOP:
        default:
            TraceSendMotorCommand(0, 0, 0);
            break;
    }

    g_reverseAction = action;
}

/*
 * Reverse-only test controller. It consumes the established physical-left/right
 * sensor order and intentionally does not share the forward TraceControlStep.
 */
static void ReverseTraceControlStep(WifiIotGpioValue physicalLeft,
                                    WifiIotGpioValue physicalRight, uint32_t now)
{
    uint8_t sensorChanged = 0U;
    ReverseTraceAction previousAction = g_reverseAction;

    g_reverseRawLeft = (physicalLeft == WIFI_IOT_GPIO_VALUE1) ? 1U : 0U;
    g_reverseRawRight = (physicalRight == WIFI_IOT_GPIO_VALUE1) ? 1U : 0U;
    if (g_reverseSensorValid == 0 || g_reverseCurrentSensor != g_stableState) {
        g_reversePreviousSensor = (g_reverseSensorValid == 0) ?
            g_stableState : g_reverseCurrentSensor;
        g_reverseCurrentSensor = g_stableState;
        g_reverseSensorValid = 1;
        sensorChanged = 1U;
    }

    g_reverseConsecutive00 = (g_stableState == 0x00U) ?
        (uint16_t)(g_reverseConsecutive00 + 1U) : 0U;
    g_reverseConsecutive01 = (g_stableState == 0x01U) ?
        (uint16_t)(g_reverseConsecutive01 + 1U) : 0U;
    g_reverseConsecutive10 = (g_stableState == 0x02U) ?
        (uint16_t)(g_reverseConsecutive10 + 1U) : 0U;
    g_reverseConsecutive11 = (g_stableState == 0x03U) ?
        (uint16_t)(g_reverseConsecutive11 + 1U) : 0U;

    if (g_stableState == 0x00U) {
        ReverseTraceApplyAction(REVERSE_ACTION_FORWARD);
    } else if (g_stableState == 0x02U) {
        g_reverseLastCorrection = TRACE_CORRECTION_LEFT;
        ReverseTraceApplyAction(REVERSE_ACTION_LEFT);
    } else if (g_stableState == 0x01U) {
        g_reverseLastCorrection = TRACE_CORRECTION_RIGHT;
        ReverseTraceApplyAction(REVERSE_ACTION_RIGHT);
    } else {
        ReverseTraceApplyAction(REVERSE_ACTION_STOP);
        if (sensorChanged != 0U || previousAction != REVERSE_ACTION_STOP) {
            printf("REVERSE sensor 11, stop\r\n");
        }
    }

    if (g_stableState == 0x03U &&
        (sensorChanged != 0U || previousAction != g_reverseAction)) {
        ReverseTracePublish(UDP_REVERSE_EVENT_SENSOR_11_STOP, now);
    } else if (g_reverseStartPublished == 0) {
        g_reverseStartPublished = 1;
        ReverseTracePublish(UDP_REVERSE_EVENT_START, now);
    } else if (sensorChanged != 0U || previousAction != g_reverseAction) {
        ReverseTracePublish(UDP_REVERSE_EVENT_SENSOR, now);
    } else if ((uint32_t)(now - g_reverseLastTelemetryTick) >=
               AppMsToTicks(TRACE_REVERSE_DEBUG_HEARTBEAT_MS)) {
        ReverseTracePublish(UDP_REVERSE_EVENT_HEARTBEAT, now);
    }
}
#endif

#if (TRACE_REVERSE_V2_TEST_MODE == 1)
typedef enum {
    REV2_BACK_STRAIGHT = 0,
    REV2_ALIGN_LEFT,
    REV2_ALIGN_RIGHT,
    REV2_SETTLING,
    REV2_STOPPED
} ReverseV2State;

static ReverseV2State g_reverseV2State = REV2_STOPPED;
static uint8_t g_reverseV2RawLeft;
static uint8_t g_reverseV2RawRight;
static uint8_t g_reverseV2LastSensor;
static int g_reverseV2SensorValid;
static uint16_t g_reverseV2Consecutive00;
static uint16_t g_reverseV2Consecutive01;
static uint16_t g_reverseV2Consecutive10;
static uint16_t g_reverseV2Consecutive11;
static uint32_t g_reverseV2AlignStartTick;
static uint32_t g_reverseV2SettleStartTick;
static uint32_t g_reverseV2LastTelemetryTick;
static int g_reverseV2StartPublished;
static int g_reverseV2StopLatched;

static UdpReverseV2State ReverseV2ToUdpState(ReverseV2State state)
{
    switch (state) {
        case REV2_BACK_STRAIGHT:
            return UDP_REVERSE_V2_STATE_BACK;
        case REV2_ALIGN_LEFT:
            return UDP_REVERSE_V2_STATE_ALIGN_LEFT;
        case REV2_ALIGN_RIGHT:
            return UDP_REVERSE_V2_STATE_ALIGN_RIGHT;
        case REV2_SETTLING:
            return UDP_REVERSE_V2_STATE_SETTLING;
        default:
            return UDP_REVERSE_V2_STATE_STOPPED;
    }
}

static void ReverseV2Publish(UdpReverseV2Event event, uint32_t now)
{
    UdpReverseV2TelemetryState state;

    state.event = event;
    state.state = ReverseV2ToUdpState(g_reverseV2State);
    state.rawLeft = g_reverseV2RawLeft;
    state.rawRight = g_reverseV2RawRight;
    state.sensorState = g_stableState;
    state.leftCommand = g_motorLeftCommand;
    state.rightCommand = g_motorRightCommand;
    state.consecutive00 = g_reverseV2Consecutive00;
    state.consecutive01 = g_reverseV2Consecutive01;
    state.consecutive10 = g_reverseV2Consecutive10;
    state.consecutive11 = g_reverseV2Consecutive11;
    state.alignElapsedMs = ((g_reverseV2State == REV2_ALIGN_LEFT ||
                             g_reverseV2State == REV2_ALIGN_RIGHT) != 0) ?
        AppTicksToMs(now - g_reverseV2AlignStartTick) : 0U;
    state.sequence = 0U;
    UdpTelemetryUpdateReverseV2(&state);
    g_reverseV2LastTelemetryTick = now;
}

static void ReverseV2ApplyState(ReverseV2State state)
{
    if (state == g_reverseV2State && g_motorCommandValid != 0) {
        return;
    }

    switch (state) {
        case REV2_BACK_STRAIGHT:
            TraceSendMotorCommand(-REV2_BACK_SPEED, -REV2_BACK_SPEED, 0);
            printf("REVERSEV2 back L=%d R=%d\r\n", -REV2_BACK_SPEED, -REV2_BACK_SPEED);
            break;
        case REV2_ALIGN_LEFT:
            /* Candidate only: stop reverse translation and yaw the chassis left in place. */
            TraceSendMotorCommand(-REV2_ALIGN_SPEED, REV2_ALIGN_SPEED, 0);
            printf("REVERSEV2 align-left L=%d R=%d\r\n", -REV2_ALIGN_SPEED, REV2_ALIGN_SPEED);
            break;
        case REV2_ALIGN_RIGHT:
            /* Candidate only: stop reverse translation and yaw the chassis right in place. */
            TraceSendMotorCommand(REV2_ALIGN_SPEED, -REV2_ALIGN_SPEED, 0);
            printf("REVERSEV2 align-right L=%d R=%d\r\n", REV2_ALIGN_SPEED, -REV2_ALIGN_SPEED);
            break;
        case REV2_SETTLING:
        case REV2_STOPPED:
        default:
            TraceSendMotorCommand(0, 0, 0);
            break;
    }
    g_reverseV2State = state;
}

static void ReverseV2Reset(uint32_t now)
{
    g_reverseV2State = REV2_STOPPED;
    g_reverseV2RawLeft = 0U;
    g_reverseV2RawRight = 0U;
    g_reverseV2LastSensor = 0U;
    g_reverseV2SensorValid = 0;
    g_reverseV2Consecutive00 = 0U;
    g_reverseV2Consecutive01 = 0U;
    g_reverseV2Consecutive10 = 0U;
    g_reverseV2Consecutive11 = 0U;
    g_reverseV2AlignStartTick = now;
    g_reverseV2SettleStartTick = now;
    g_reverseV2LastTelemetryTick = now;
    g_reverseV2StartPublished = 0;
    g_reverseV2StopLatched = 0;
}

/* Experimental Reverse V2: do not combine reverse translation with correction.
 * It is intentionally separate from ReverseTraceControlStep (V1), whose
 * continuous reverse differential correction failed during on-car testing. */
static void ReverseV2ControlStep(WifiIotGpioValue physicalLeft,
                                 WifiIotGpioValue physicalRight, uint32_t now)
{
    uint8_t sensorChanged = 0U;
    ReverseV2State previousState = g_reverseV2State;
    UdpReverseV2Event event = UDP_REVERSE_V2_EVENT_SENSOR;

    g_reverseV2RawLeft = (physicalLeft == WIFI_IOT_GPIO_VALUE1) ? 1U : 0U;
    g_reverseV2RawRight = (physicalRight == WIFI_IOT_GPIO_VALUE1) ? 1U : 0U;
    if (g_reverseV2SensorValid == 0 || g_reverseV2LastSensor != g_stableState) {
        g_reverseV2LastSensor = g_stableState;
        g_reverseV2SensorValid = 1;
        sensorChanged = 1U;
    }

    g_reverseV2Consecutive00 = (g_stableState == 0x00U) ?
        (uint16_t)(g_reverseV2Consecutive00 + 1U) : 0U;
    g_reverseV2Consecutive01 = (g_stableState == 0x01U) ?
        (uint16_t)(g_reverseV2Consecutive01 + 1U) : 0U;
    g_reverseV2Consecutive10 = (g_stableState == 0x02U) ?
        (uint16_t)(g_reverseV2Consecutive10 + 1U) : 0U;
    g_reverseV2Consecutive11 = (g_stableState == 0x03U) ?
        (uint16_t)(g_reverseV2Consecutive11 + 1U) : 0U;

    if (g_reverseV2State == REV2_STOPPED) {
        if (g_reverseV2StopLatched != 0) {
            ReverseV2ApplyState(REV2_STOPPED);
        } else if (g_stableState == 0x03U) {
            g_reverseV2StopLatched = 1;
            ReverseV2ApplyState(REV2_STOPPED);
            event = UDP_REVERSE_V2_EVENT_SENSOR_11_STOP;
        } else if (g_stableState == 0x01U) {
            g_reverseV2AlignStartTick = now;
            /* Right sensor hit: yaw left so the reverse trajectory corrects right. */
            ReverseV2ApplyState(REV2_ALIGN_LEFT);
            event = UDP_REVERSE_V2_EVENT_ALIGN_LEFT_START;
        } else if (g_stableState == 0x02U) {
            g_reverseV2AlignStartTick = now;
            /* Left sensor hit: yaw right so the reverse trajectory corrects left. */
            ReverseV2ApplyState(REV2_ALIGN_RIGHT);
            event = UDP_REVERSE_V2_EVENT_ALIGN_RIGHT_START;
        } else {
            ReverseV2ApplyState(REV2_BACK_STRAIGHT);
            event = UDP_REVERSE_V2_EVENT_START;
        }
    } else if (g_reverseV2State == REV2_BACK_STRAIGHT) {
        if (g_stableState == 0x03U) {
            g_reverseV2StopLatched = 1;
            ReverseV2ApplyState(REV2_STOPPED);
            event = UDP_REVERSE_V2_EVENT_SENSOR_11_STOP;
        } else if (g_stableState == 0x01U) {
            g_reverseV2AlignStartTick = now;
            /* Right sensor hit: yaw left so the reverse trajectory corrects right. */
            ReverseV2ApplyState(REV2_ALIGN_LEFT);
            event = UDP_REVERSE_V2_EVENT_ALIGN_LEFT_START;
        } else if (g_stableState == 0x02U) {
            g_reverseV2AlignStartTick = now;
            /* Left sensor hit: yaw right so the reverse trajectory corrects left. */
            ReverseV2ApplyState(REV2_ALIGN_RIGHT);
            event = UDP_REVERSE_V2_EVENT_ALIGN_RIGHT_START;
        }
    } else if (g_reverseV2State == REV2_ALIGN_LEFT ||
               g_reverseV2State == REV2_ALIGN_RIGHT) {
        if (g_stableState == 0x03U) {
            g_reverseV2StopLatched = 1;
            ReverseV2ApplyState(REV2_STOPPED);
            event = UDP_REVERSE_V2_EVENT_SENSOR_11_STOP;
        } else if (g_reverseV2Consecutive00 >= REV2_ALIGN_CLEAR_SAMPLES) {
            g_reverseV2SettleStartTick = now;
            ReverseV2ApplyState(REV2_SETTLING);
            event = UDP_REVERSE_V2_EVENT_ALIGN_CLEAR;
        } else if ((uint32_t)(now - g_reverseV2AlignStartTick) >=
                   AppMsToTicks(REV2_ALIGN_TIMEOUT_MS)) {
            g_reverseV2StopLatched = 1;
            ReverseV2ApplyState(REV2_STOPPED);
            event = UDP_REVERSE_V2_EVENT_ALIGN_TIMEOUT;
        }
    } else if (g_reverseV2State == REV2_SETTLING) {
        if (g_stableState == 0x03U) {
            g_reverseV2StopLatched = 1;
            ReverseV2ApplyState(REV2_STOPPED);
            event = UDP_REVERSE_V2_EVENT_SENSOR_11_STOP;
        } else if ((uint32_t)(now - g_reverseV2SettleStartTick) >=
                   AppMsToTicks(REV2_ALIGN_SETTLE_MS)) {
            ReverseV2ApplyState(REV2_BACK_STRAIGHT);
            event = UDP_REVERSE_V2_EVENT_BACK_RESUME;
        }
    }

    if (g_reverseV2StartPublished == 0) {
        g_reverseV2StartPublished = 1;
        ReverseV2Publish(UDP_REVERSE_V2_EVENT_START, now);
    } else if (sensorChanged != 0U || previousState != g_reverseV2State ||
               event != UDP_REVERSE_V2_EVENT_SENSOR) {
        ReverseV2Publish(event, now);
    } else if ((uint32_t)(now - g_reverseV2LastTelemetryTick) >=
               AppMsToTicks(REV2_DEBUG_HEARTBEAT_MS)) {
        ReverseV2Publish(UDP_REVERSE_V2_EVENT_HEARTBEAT, now);
    }
}
#endif

#if (TRACE_REVERSE_V3_TEST_MODE == 1)
typedef enum {
    REV3_ACTION_STOP = 0,
    REV3_ACTION_BACK,
    REV3_ACTION_CORRECT_LEFT,
    REV3_ACTION_CORRECT_RIGHT
} ReverseV3Action;

static ReverseV3Action g_reverseV3Action = REV3_ACTION_STOP;
static TraceCorrection g_reverseV3LastCorrection = TRACE_CORRECTION_NONE;
static uint8_t g_reverseV3RawLeft;
static uint8_t g_reverseV3RawRight;
static uint8_t g_reverseV3PreviousSensor;
static uint8_t g_reverseV3CurrentSensor;
static int g_reverseV3SensorValid;
static uint16_t g_reverseV3Consecutive00;
static uint16_t g_reverseV3Consecutive01;
static uint16_t g_reverseV3Consecutive10;
static uint16_t g_reverseV3Consecutive11;
static uint32_t g_reverseV3LastTelemetryTick;
static int g_reverseV3StartPublished;
static int g_reverseV3StopLatched;

static UdpReverseV3Action ReverseV3ToUdpAction(ReverseV3Action action)
{
    switch (action) {
        case REV3_ACTION_BACK:
            return UDP_REVERSE_V3_ACTION_BACK;
        case REV3_ACTION_CORRECT_LEFT:
            return UDP_REVERSE_V3_ACTION_CORRECT_LEFT;
        case REV3_ACTION_CORRECT_RIGHT:
            return UDP_REVERSE_V3_ACTION_CORRECT_RIGHT;
        default:
            return UDP_REVERSE_V3_ACTION_STOP;
    }
}

static void ReverseV3Publish(UdpReverseV3Event event, uint32_t now)
{
    UdpReverseV3TelemetryState state;

    state.event = event;
    state.rawLeft = g_reverseV3RawLeft;
    state.rawRight = g_reverseV3RawRight;
    state.sensorState = g_stableState;
    state.previousSensorState = g_reverseV3PreviousSensor;
    state.lastCorrection = (uint8_t)g_reverseV3LastCorrection;
    state.action = ReverseV3ToUdpAction(g_reverseV3Action);
    state.leftCommand = g_motorLeftCommand;
    state.rightCommand = g_motorRightCommand;
    state.consecutive00 = g_reverseV3Consecutive00;
    state.consecutive01 = g_reverseV3Consecutive01;
    state.consecutive10 = g_reverseV3Consecutive10;
    state.consecutive11 = g_reverseV3Consecutive11;
    state.sequence = 0U;
    UdpTelemetryUpdateReverseV3(&state);
    g_reverseV3LastTelemetryTick = now;
}

static void ReverseV3ApplyAction(ReverseV3Action action)
{
    if (action == g_reverseV3Action && g_motorCommandValid != 0) {
        return;
    }

    switch (action) {
        case REV3_ACTION_BACK:
            TraceSendMotorCommand(-REV3_BACK_SPEED, -REV3_BACK_SPEED, 0);
            printf("REVERSEV3 back L=%d R=%d\r\n", -REV3_BACK_SPEED, -REV3_BACK_SPEED);
            break;
        case REV3_ACTION_CORRECT_LEFT:
            /* Left sensor hit: reverse trajectory corrects left while chassis yaws right. */
            TraceSendMotorCommand(-REV3_INNER_SPEED, -REV3_OUTER_SPEED, 0);
            printf("REVERSEV3 correct-left yaw-right L=%d R=%d\r\n",
                   -REV3_INNER_SPEED, -REV3_OUTER_SPEED);
            break;
        case REV3_ACTION_CORRECT_RIGHT:
            /* Right sensor hit: reverse trajectory corrects right while chassis yaws left. */
            TraceSendMotorCommand(-REV3_OUTER_SPEED, -REV3_INNER_SPEED, 0);
            printf("REVERSEV3 correct-right yaw-left L=%d R=%d\r\n",
                   -REV3_OUTER_SPEED, -REV3_INNER_SPEED);
            break;
        case REV3_ACTION_STOP:
        default:
            TraceSendMotorCommand(0, 0, 0);
            break;
    }
    g_reverseV3Action = action;
}

static void ReverseV3Reset(uint32_t now)
{
    g_reverseV3Action = REV3_ACTION_STOP;
    g_reverseV3LastCorrection = TRACE_CORRECTION_NONE;
    g_reverseV3RawLeft = 0U;
    g_reverseV3RawRight = 0U;
    g_reverseV3PreviousSensor = 0U;
    g_reverseV3CurrentSensor = 0U;
    g_reverseV3SensorValid = 0;
    g_reverseV3Consecutive00 = 0U;
    g_reverseV3Consecutive01 = 0U;
    g_reverseV3Consecutive10 = 0U;
    g_reverseV3Consecutive11 = 0U;
    g_reverseV3LastTelemetryTick = now;
    g_reverseV3StartPublished = 0;
    g_reverseV3StopLatched = 0;
}

/* Experimental Reverse V3: direct reverse translation plus the yaw opposite
 * to forward TRACE. It deliberately has no V2 ALIGN, SETTLE, or timeout path. */
static void ReverseV3ControlStep(WifiIotGpioValue physicalLeft,
                                 WifiIotGpioValue physicalRight, uint32_t now)
{
    uint8_t sensorChanged = 0U;
    ReverseV3Action previousAction = g_reverseV3Action;
    UdpReverseV3Event event = UDP_REVERSE_V3_EVENT_SENSOR;

    g_reverseV3RawLeft = (physicalLeft == WIFI_IOT_GPIO_VALUE1) ? 1U : 0U;
    g_reverseV3RawRight = (physicalRight == WIFI_IOT_GPIO_VALUE1) ? 1U : 0U;
    if (g_reverseV3SensorValid == 0 || g_reverseV3CurrentSensor != g_stableState) {
        g_reverseV3PreviousSensor = (g_reverseV3SensorValid == 0) ?
            g_stableState : g_reverseV3CurrentSensor;
        g_reverseV3CurrentSensor = g_stableState;
        g_reverseV3SensorValid = 1;
        sensorChanged = 1U;
    }

    g_reverseV3Consecutive00 = (g_stableState == 0x00U) ?
        (uint16_t)(g_reverseV3Consecutive00 + 1U) : 0U;
    g_reverseV3Consecutive01 = (g_stableState == 0x01U) ?
        (uint16_t)(g_reverseV3Consecutive01 + 1U) : 0U;
    g_reverseV3Consecutive10 = (g_stableState == 0x02U) ?
        (uint16_t)(g_reverseV3Consecutive10 + 1U) : 0U;
    g_reverseV3Consecutive11 = (g_stableState == 0x03U) ?
        (uint16_t)(g_reverseV3Consecutive11 + 1U) : 0U;

    if (g_reverseV3StopLatched != 0) {
        ReverseV3ApplyAction(REV3_ACTION_STOP);
    } else if (g_stableState == 0x00U) {
        ReverseV3ApplyAction(REV3_ACTION_BACK);
    } else if (g_stableState == 0x02U) {
        g_reverseV3LastCorrection = TRACE_CORRECTION_LEFT;
        ReverseV3ApplyAction(REV3_ACTION_CORRECT_LEFT);
    } else if (g_stableState == 0x01U) {
        g_reverseV3LastCorrection = TRACE_CORRECTION_RIGHT;
        ReverseV3ApplyAction(REV3_ACTION_CORRECT_RIGHT);
    } else {
        g_reverseV3StopLatched = 1;
        ReverseV3ApplyAction(REV3_ACTION_STOP);
        event = UDP_REVERSE_V3_EVENT_SENSOR_11_STOP;
    }

    if (g_reverseV3StartPublished == 0) {
        g_reverseV3StartPublished = 1;
        ReverseV3Publish(UDP_REVERSE_V3_EVENT_START, now);
    } else if (sensorChanged != 0U || previousAction != g_reverseV3Action ||
               event != UDP_REVERSE_V3_EVENT_SENSOR) {
        ReverseV3Publish(event, now);
    } else if ((uint32_t)(now - g_reverseV3LastTelemetryTick) >=
               AppMsToTicks(REV3_DEBUG_HEARTBEAT_MS)) {
        ReverseV3Publish(UDP_REVERSE_V3_EVENT_HEARTBEAT, now);
    }
}
#endif

#if (TRACE_REVERSE_V4_TEST_MODE == 1)
typedef enum {
    REV4_WAIT_CLEAR = 0,
    REV4_BACK,
    REV4_PULSE_LEFT,
    REV4_PULSE_RIGHT,
    REV4_PROBE_BACK,
    REV4_STOPPED
} ReverseV4State;

static ReverseV4State g_reverseV4State = REV4_STOPPED;
static uint8_t g_reverseV4RawLeft;
static uint8_t g_reverseV4RawRight;
static uint8_t g_reverseV4PreviousSensor;
static uint8_t g_reverseV4CurrentSensor;
static uint8_t g_reverseV4TriggerSensor;
static uint16_t g_reverseV4SameSidePulses;
static int g_reverseV4SensorValid;
static int g_reverseV4StartPublished;
static uint32_t g_reverseV4StateStartTick;
static uint32_t g_reverseV4LastTelemetryTick;
static uint32_t g_reverseV4ClearSinceTick;
static uint32_t g_reverseV4RearmSinceTick;

static UdpReverseV4State ReverseV4ToUdpState(ReverseV4State state)
{
    switch (state) {
        case REV4_BACK: return UDP_REVERSE_V4_STATE_BACK;
        case REV4_PULSE_LEFT: return UDP_REVERSE_V4_STATE_PULSE_LEFT;
        case REV4_PULSE_RIGHT: return UDP_REVERSE_V4_STATE_PULSE_RIGHT;
        case REV4_PROBE_BACK: return UDP_REVERSE_V4_STATE_PROBE_BACK;
        default: return UDP_REVERSE_V4_STATE_STOPPED;
    }
}

static void ReverseV4ApplyState(ReverseV4State state)
{
    if (state == g_reverseV4State && g_motorCommandValid != 0) {
        return;
    }
    switch (state) {
        case REV4_BACK:
        case REV4_PROBE_BACK:
            TraceSendMotorCommand(-REV4_BACK_SPEED, -REV4_BACK_SPEED, 0);
            break;
        case REV4_PULSE_RIGHT:
            TraceSendMotorCommand(-REV4_INNER_SPEED, -REV4_OUTER_SPEED, 0);
            break;
        case REV4_PULSE_LEFT:
            TraceSendMotorCommand(-REV4_OUTER_SPEED, -REV4_INNER_SPEED, 0);
            break;
        default:
            TraceSendMotorCommand(0, 0, 0);
            break;
    }
    g_reverseV4State = state;
}

static void ReverseV4Publish(UdpReverseV4Event event, uint32_t now)
{
    UdpReverseV4TelemetryState state;
    uint32_t elapsedMs = AppTicksToMs((uint32_t)(now - g_reverseV4StateStartTick));

    state.event = event;
    state.state = ReverseV4ToUdpState(g_reverseV4State);
    state.rawLeft = g_reverseV4RawLeft;
    state.rawRight = g_reverseV4RawRight;
    state.sensorState = g_stableState;
    state.previousSensorState = g_reverseV4PreviousSensor;
    state.triggerSensorState = g_reverseV4TriggerSensor;
    state.leftCommand = g_motorLeftCommand;
    state.rightCommand = g_motorRightCommand;
    state.sameSidePulses = g_reverseV4SameSidePulses;
    state.pulseElapsedMs = (g_reverseV4State == REV4_PULSE_LEFT ||
                            g_reverseV4State == REV4_PULSE_RIGHT) ? elapsedMs : 0U;
    state.probeElapsedMs = (g_reverseV4State == REV4_PROBE_BACK) ? elapsedMs : 0U;
    state.sequence = 0U;
    UdpTelemetryUpdateReverseV4(&state);
    g_reverseV4LastTelemetryTick = now;
}

static void ReverseV4Enter(ReverseV4State state, uint8_t trigger, uint32_t now)
{
    g_reverseV4TriggerSensor = trigger;
    g_reverseV4StateStartTick = now;
    ReverseV4ApplyState(state);
}

static void ReverseV4Reset(uint32_t now)
{
    g_reverseV4State = REV4_WAIT_CLEAR;
    g_reverseV4RawLeft = 0U;
    g_reverseV4RawRight = 0U;
    g_reverseV4PreviousSensor = 0U;
    g_reverseV4CurrentSensor = 0U;
    g_reverseV4TriggerSensor = 0U;
    g_reverseV4SameSidePulses = 0U;
    g_reverseV4SensorValid = 0;
    g_reverseV4StartPublished = 0;
    g_reverseV4StateStartTick = now;
    g_reverseV4LastTelemetryTick = now;
    g_reverseV4ClearSinceTick = 0U;
    g_reverseV4RearmSinceTick = 0U;
}

/* Reverse V4: a fixed yaw pulse, then forced straight reverse so the trailing
 * front sensors obtain a new spatial observation before another correction. */
static void ReverseV4ControlStep(WifiIotGpioValue physicalLeft,
                                 WifiIotGpioValue physicalRight, uint32_t now)
{
    uint8_t sensorChanged = 0U;
    ReverseV4State previousState = g_reverseV4State;
    UdpReverseV4Event event = UDP_REVERSE_V4_EVENT_SENSOR;

    g_reverseV4RawLeft = (physicalLeft == WIFI_IOT_GPIO_VALUE1) ? 1U : 0U;
    g_reverseV4RawRight = (physicalRight == WIFI_IOT_GPIO_VALUE1) ? 1U : 0U;
    if (g_reverseV4SensorValid == 0 || g_reverseV4CurrentSensor != g_stableState) {
        g_reverseV4PreviousSensor = (g_reverseV4SensorValid == 0) ?
            g_stableState : g_reverseV4CurrentSensor;
        g_reverseV4CurrentSensor = g_stableState;
        g_reverseV4SensorValid = 1;
        sensorChanged = 1U;
    }

    switch (g_reverseV4State) {
        case REV4_WAIT_CLEAR:
            ReverseV4ApplyState(REV4_WAIT_CLEAR);
            g_reverseV4RearmSinceTick = 0U;
            if (g_stableState == 0x00U) {
                if (g_reverseV4ClearSinceTick == 0U) {
                    g_reverseV4ClearSinceTick = now;
                } else if ((uint32_t)(now - g_reverseV4ClearSinceTick) >=
                           AppMsToTicks(REV4_ARM_CLEAR_MS)) {
                    g_reverseV4ClearSinceTick = 0U;
                    ReverseV4Enter(REV4_BACK, 0U, now);
                    event = UDP_REVERSE_V4_EVENT_ARMED;
                }
            } else {
                g_reverseV4ClearSinceTick = 0U;
                if (sensorChanged != 0U) event = UDP_REVERSE_V4_EVENT_WAIT_CLEAR;
            }
            break;
        case REV4_BACK:
        case REV4_PULSE_LEFT:
        case REV4_PULSE_RIGHT:
        case REV4_PROBE_BACK:
            if (g_stableState == 0x03U) {
                ReverseV4Enter(REV4_STOPPED, g_reverseV4TriggerSensor, now);
                event = UDP_REVERSE_V4_EVENT_SENSOR_11_STOP;
                break;
            }
            switch (g_reverseV4State) {
            case REV4_BACK:
                if (g_stableState == 0x02U) {
                    g_reverseV4SameSidePulses = 1U;
                    ReverseV4Enter(REV4_PULSE_RIGHT, 0x02U, now);
                    event = UDP_REVERSE_V4_EVENT_PULSE_START;
                } else if (g_stableState == 0x01U) {
                    g_reverseV4SameSidePulses = 1U;
                    ReverseV4Enter(REV4_PULSE_LEFT, 0x01U, now);
                    event = UDP_REVERSE_V4_EVENT_PULSE_START;
                } else {
                    g_reverseV4SameSidePulses = 0U;
                    ReverseV4ApplyState(REV4_BACK);
                }
                break;
            case REV4_PULSE_LEFT:
            case REV4_PULSE_RIGHT:
                if ((uint32_t)(now - g_reverseV4StateStartTick) >=
                    AppMsToTicks(REV4_CORRECT_PULSE_MS)) {
                    ReverseV4Publish(UDP_REVERSE_V4_EVENT_PULSE_END, now);
                    ReverseV4Enter(REV4_PROBE_BACK, g_reverseV4TriggerSensor, now);
                    event = UDP_REVERSE_V4_EVENT_PROBE_START;
                }
                break;
            case REV4_PROBE_BACK:
                if ((uint32_t)(now - g_reverseV4StateStartTick) >=
                    AppMsToTicks(REV4_PROBE_BACK_MS)) {
                    event = UDP_REVERSE_V4_EVENT_PROBE_RESULT;
                    ReverseV4Publish(event, now);
                    if (g_stableState == 0x00U) {
                        g_reverseV4SameSidePulses = 0U;
                        ReverseV4Enter(REV4_BACK, 0U, now);
                    } else if (g_stableState != g_reverseV4TriggerSensor) {
                        g_reverseV4SameSidePulses = 1U;
                        ReverseV4Enter((g_stableState == 0x02U) ? REV4_PULSE_RIGHT :
                                      REV4_PULSE_LEFT, g_stableState, now);
                    } else if (g_reverseV4SameSidePulses >= REV4_MAX_SAME_SIDE_PULSES) {
                        ReverseV4Enter(REV4_STOPPED, g_reverseV4TriggerSensor, now);
                        event = UDP_REVERSE_V4_EVENT_PULSE_LIMIT;
                    } else {
                        g_reverseV4SameSidePulses++;
                        ReverseV4Enter((g_stableState == 0x02U) ? REV4_PULSE_RIGHT :
                                      REV4_PULSE_LEFT, g_stableState, now);
                    }
                    if (g_reverseV4State == REV4_PULSE_LEFT ||
                        g_reverseV4State == REV4_PULSE_RIGHT) {
                        event = UDP_REVERSE_V4_EVENT_PULSE_START;
                    }
                }
                break;
            default:
                break;
            }
            break;
        case REV4_STOPPED:
        default:
            ReverseV4ApplyState(REV4_STOPPED);
            g_reverseV4ClearSinceTick = 0U;
            if (g_stableState == 0x00U) {
                if (g_reverseV4RearmSinceTick == 0U) g_reverseV4RearmSinceTick = now;
                else if ((uint32_t)(now - g_reverseV4RearmSinceTick) >=
                         AppMsToTicks(REV4_REARM_CLEAR_MS)) {
                    g_reverseV4RearmSinceTick = 0U;
                    ReverseV4Enter(REV4_WAIT_CLEAR, 0U, now);
                    event = UDP_REVERSE_V4_EVENT_REARM;
                }
            } else {
                g_reverseV4RearmSinceTick = 0U;
            }
            break;
    }

    if (g_reverseV4StartPublished == 0) {
        g_reverseV4StartPublished = 1;
        ReverseV4Publish(UDP_REVERSE_V4_EVENT_START, now);
    } else if (sensorChanged != 0U || previousState != g_reverseV4State ||
               event != UDP_REVERSE_V4_EVENT_SENSOR) {
        ReverseV4Publish(event, now);
    } else if ((uint32_t)(now - g_reverseV4LastTelemetryTick) >=
               AppMsToTicks(REV4_DEBUG_HEARTBEAT_MS)) {
        ReverseV4Publish(UDP_REVERSE_V4_EVENT_HEARTBEAT, now);
    }
}
#endif

#if (TRACE_REVERSE_V5_TEST_MODE == 1)
typedef enum { REV5_WAIT_CLEAR = 0, REV5_BACK, REV5_HEADING_LEFT, REV5_HEADING_RIGHT,
    REV5_HEADING_PROBE, REV5_SHIFT_YAW_OUT_LEFT, REV5_SHIFT_YAW_OUT_RIGHT,
    REV5_SHIFT_BACK, REV5_SHIFT_YAW_BACK_LEFT, REV5_SHIFT_YAW_BACK_RIGHT,
    REV5_RECHECK_BACK, REV5_STOPPED } ReverseV5State;
static ReverseV5State g_reverseV5State = REV5_STOPPED;
static uint8_t g_reverseV5SensorValid, g_reverseV5CurrentSensor, g_reverseV5PreviousSensor;
static uint8_t g_reverseV5Trigger;
static uint16_t g_reverseV5ShiftCount;
static uint32_t g_reverseV5StateStart, g_reverseV5ClearSince, g_reverseV5RearmSince, g_reverseV5LastTelemetry;
static int g_reverseV5StartPublished;
static int g_reverseV5BackBannerPublished;
static uint8_t ReverseV5UdpState(ReverseV5State s) { return (s == REV5_BACK) ? UDP_REVERSE_V4_STATE_BACK : (s == REV5_STOPPED || s == REV5_WAIT_CLEAR) ? UDP_REVERSE_V4_STATE_STOPPED : UDP_REVERSE_V4_STATE_PROBE_BACK; }
static void ReverseV5Apply(ReverseV5State s) { int l = 0, r = 0; switch (s) {
    case REV5_BACK: case REV5_SHIFT_BACK: case REV5_RECHECK_BACK: l = r = -REV4_BACK_SPEED; break;
    case REV5_HEADING_RIGHT: case REV5_SHIFT_YAW_OUT_LEFT: l = -REV4_INNER_SPEED; r = -REV4_OUTER_SPEED; break;
    case REV5_HEADING_LEFT: case REV5_SHIFT_YAW_OUT_RIGHT: l = -REV4_OUTER_SPEED; r = -REV4_INNER_SPEED; break;
    case REV5_SHIFT_YAW_BACK_LEFT: l = -REV4_INNER_SPEED; r = -REV4_OUTER_SPEED; break;
    case REV5_SHIFT_YAW_BACK_RIGHT: l = -REV4_OUTER_SPEED; r = -REV4_INNER_SPEED; break;
    default: break; } TraceSendMotorCommand(l, r, 0); if (s == REV5_BACK && !g_reverseV5BackBannerPublished) { printf("V5 MOTOR BACK COMMAND L=-100 R=-100\\r\\n"); g_reverseV5BackBannerPublished = 1; } g_reverseV5State = s; }
static void ReverseV5Enter(ReverseV5State s, uint8_t trig, uint32_t now) { g_reverseV5Trigger = trig; g_reverseV5StateStart = now; ReverseV5Apply(s); }
static void ReverseV5Publish(UdpReverseV5Event event, uint32_t now) { UdpReverseV4TelemetryState t = {0}; t.event = (UdpReverseV4Event)event; t.state = (UdpReverseV4State)ReverseV5UdpState(g_reverseV5State); t.sensorState = g_stableState; t.previousSensorState = g_reverseV5PreviousSensor; t.triggerSensorState = g_reverseV5Trigger; t.leftCommand = g_motorLeftCommand; t.rightCommand = g_motorRightCommand; t.sameSidePulses = g_reverseV5ShiftCount; t.pulseElapsedMs = AppTicksToMs((uint32_t)(now - g_reverseV5StateStart)); UdpTelemetryUpdateReverseV4(&t); g_reverseV5LastTelemetry = now; }
static void ReverseV5Reset(uint32_t now) { g_reverseV5State = REV5_WAIT_CLEAR; g_reverseV5SensorValid = 0; g_reverseV5StartPublished = 0; g_reverseV5BackBannerPublished = 0; g_reverseV5ClearSince = g_reverseV5RearmSince = 0; g_reverseV5ShiftCount = 0; g_reverseV5StateStart = g_reverseV5LastTelemetry = now; }
static void ReverseV5ControlStep(WifiIotGpioValue l, WifiIotGpioValue r, uint32_t now) {
    uint8_t changed = 0; ReverseV5State prev = g_reverseV5State; UdpReverseV5Event ev = UDP_REVERSE_V5_EVENT_HEARTBEAT;
    if (!g_reverseV5SensorValid || g_reverseV5CurrentSensor != g_stableState) { g_reverseV5PreviousSensor = g_reverseV5CurrentSensor; g_reverseV5CurrentSensor = g_stableState; g_reverseV5SensorValid = 1; changed = 1; }
    (void)l; (void)r;
    if (g_reverseV5State == REV5_WAIT_CLEAR) { ReverseV5Apply(REV5_WAIT_CLEAR); if (g_stableState == 0) { if (!g_reverseV5ClearSince) g_reverseV5ClearSince = now; else if ((uint32_t)(now-g_reverseV5ClearSince)>=AppMsToTicks(REV4_ARM_CLEAR_MS)) { ReverseV5Enter(REV5_BACK,0,now); ev=UDP_REVERSE_V5_EVENT_ARMED; } } else g_reverseV5ClearSince=0; }
    else if (g_reverseV5State == REV5_STOPPED) { ReverseV5Apply(REV5_STOPPED); if (g_stableState==0) { if (!g_reverseV5RearmSince) g_reverseV5RearmSince=now; else if ((uint32_t)(now-g_reverseV5RearmSince)>=AppMsToTicks(REV4_REARM_CLEAR_MS)) { ReverseV5Enter(REV5_WAIT_CLEAR,0,now); ev=UDP_REVERSE_V5_EVENT_REARM; } } else g_reverseV5RearmSince=0; }
    else if (g_stableState==3) { ReverseV5Enter(REV5_STOPPED,g_reverseV5Trigger,now); ev=UDP_REVERSE_V5_EVENT_SENSOR_11_STOP; }
    else switch (g_reverseV5State) {
    case REV5_BACK: if(g_stableState==2){g_reverseV5Trigger=2;ReverseV5Enter(REV5_HEADING_RIGHT,2,now);ev=UDP_REVERSE_V5_EVENT_HEADING_PULSE_START;} else if(g_stableState==1){g_reverseV5Trigger=1;ReverseV5Enter(REV5_HEADING_LEFT,1,now);ev=UDP_REVERSE_V5_EVENT_HEADING_PULSE_START;} else ReverseV5Apply(REV5_BACK); break;
    case REV5_HEADING_LEFT: case REV5_HEADING_RIGHT: if((uint32_t)(now-g_reverseV5StateStart)>=AppMsToTicks(REV4_CORRECT_PULSE_MS)){ReverseV5Enter(REV5_HEADING_PROBE,g_reverseV5Trigger,now);ev=UDP_REVERSE_V5_EVENT_HEADING_PROBE;} break;
    case REV5_HEADING_PROBE: if((uint32_t)(now-g_reverseV5StateStart)>=AppMsToTicks(REV4_PROBE_BACK_MS)){ if(g_stableState==0){ReverseV5Enter(REV5_BACK,0,now);} else if(g_stableState!=g_reverseV5Trigger){ReverseV5Enter(REV5_STOPPED,g_reverseV5Trigger,now);ev=UDP_REVERSE_V5_EVENT_OPPOSITE_AFTER_HEADING_PROBE;} else if(g_reverseV5ShiftCount>=V5_MAX_LATERAL_SHIFTS){ReverseV5Enter(REV5_STOPPED,g_reverseV5Trigger,now);ev=UDP_REVERSE_V5_EVENT_LATERAL_SHIFT_LIMIT;} else {g_reverseV5ShiftCount++;ReverseV5Enter(g_reverseV5Trigger==2?REV5_SHIFT_YAW_OUT_LEFT:REV5_SHIFT_YAW_OUT_RIGHT,g_reverseV5Trigger,now);ev=UDP_REVERSE_V5_EVENT_LATERAL_SHIFT_START;} } break;
    case REV5_SHIFT_YAW_OUT_LEFT: case REV5_SHIFT_YAW_OUT_RIGHT: if((uint32_t)(now-g_reverseV5StateStart)>=AppMsToTicks(V5_SHIFT_YAW_OUT_MS)){ReverseV5Enter(REV5_SHIFT_BACK,g_reverseV5Trigger,now);ev=UDP_REVERSE_V5_EVENT_SHIFT_BACK;} break;
    case REV5_SHIFT_BACK: if((uint32_t)(now-g_reverseV5StateStart)>=AppMsToTicks(V5_SHIFT_BACK_MS)){ReverseV5Enter(g_reverseV5Trigger==2?REV5_SHIFT_YAW_BACK_RIGHT:REV5_SHIFT_YAW_BACK_LEFT,g_reverseV5Trigger,now);ev=UDP_REVERSE_V5_EVENT_SHIFT_YAW_BACK;} break;
    case REV5_SHIFT_YAW_BACK_LEFT: case REV5_SHIFT_YAW_BACK_RIGHT: if((uint32_t)(now-g_reverseV5StateStart)>=AppMsToTicks(V5_SHIFT_YAW_BACK_MS)){ReverseV5Enter(REV5_RECHECK_BACK,g_reverseV5Trigger,now);ev=UDP_REVERSE_V5_EVENT_RECHECK;} break;
    case REV5_RECHECK_BACK: if((uint32_t)(now-g_reverseV5StateStart)>=AppMsToTicks(V5_RECHECK_BACK_MS)){if(g_stableState==0){ReverseV5Enter(REV5_BACK,0,now);ev=UDP_REVERSE_V5_EVENT_LATERAL_RECENTER_SUCCESS;} else if(g_stableState!=g_reverseV5Trigger){ReverseV5Enter(REV5_STOPPED,g_reverseV5Trigger,now);ev=UDP_REVERSE_V5_EVENT_LATERAL_OVERSHOOT;} else if(g_reverseV5ShiftCount>=V5_MAX_LATERAL_SHIFTS){ReverseV5Enter(REV5_STOPPED,g_reverseV5Trigger,now);ev=UDP_REVERSE_V5_EVENT_LATERAL_SHIFT_LIMIT;} else {ReverseV5Enter(g_reverseV5Trigger==2?REV5_SHIFT_YAW_OUT_LEFT:REV5_SHIFT_YAW_OUT_RIGHT,g_reverseV5Trigger,now);ev=UDP_REVERSE_V5_EVENT_LATERAL_RECENTER_SAME_SIDE;}} break;
    default: break; }
    if(!g_reverseV5StartPublished){g_reverseV5StartPublished=1;ReverseV5Publish(UDP_REVERSE_V5_EVENT_START,now);} else if(changed || prev!=g_reverseV5State || ev!=UDP_REVERSE_V5_EVENT_HEARTBEAT) ReverseV5Publish(ev,now); else if((uint32_t)(now-g_reverseV5LastTelemetry)>=AppMsToTicks(REV4_DEBUG_HEARTBEAT_MS)) ReverseV5Publish(UDP_REVERSE_V5_EVENT_HEARTBEAT,now);
}
#endif

#if (TRACE_REVERSE_V6_TEST_MODE == 1)
typedef enum { REV6_WAIT_CLEAR=0, REV6_BACK, REV6_HEADING_LEFT, REV6_HEADING_RIGHT,
 REV6_SHIFT_OUT_LEFT, REV6_SHIFT_OUT_RIGHT, REV6_SHIFT_BACK, REV6_RESTORE_LEFT,
 REV6_RESTORE_RIGHT, REV6_VERIFY_BACK, REV6_STOPPED } ReverseV6State;
static ReverseV6State g_reverseV6State=REV6_STOPPED;
static uint8_t g_reverseV6Valid,g_reverseV6Current,g_reverseV6Previous,g_reverseV6Origin,g_reverseV6Capture;
static uint32_t g_reverseV6Start,g_reverseV6ClearSince,g_reverseV6RearmSince,g_reverseV6ActualYaw,g_reverseV6RestoreTarget,g_reverseV6Last;
static uint8_t g_reverseV6ShiftCount; static int g_reverseV6Banner,g_reverseV6BackBanner;
static void ReverseV6Apply(ReverseV6State s){int l=0,r=0;switch(s){case REV6_BACK:case REV6_SHIFT_BACK:case REV6_VERIFY_BACK:l=r=-REV4_BACK_SPEED;break;case REV6_HEADING_RIGHT:case REV6_SHIFT_OUT_LEFT:l=-REV4_INNER_SPEED;r=-REV4_OUTER_SPEED;break;case REV6_HEADING_LEFT:case REV6_SHIFT_OUT_RIGHT:l=-REV4_OUTER_SPEED;r=-REV4_INNER_SPEED;break;case REV6_RESTORE_LEFT:l=-REV4_INNER_SPEED;r=-REV4_OUTER_SPEED;break;case REV6_RESTORE_RIGHT:l=-REV4_OUTER_SPEED;r=-REV4_INNER_SPEED;break;default:break;}TraceSendMotorCommand(l,r,0);g_reverseV6State=s;if(s==REV6_BACK&&!g_reverseV6BackBanner){printf("V6 MOTOR BACK COMMAND L=-100 R=-100\\r\\n");g_reverseV6BackBanner=1;}}
static void ReverseV6Enter(ReverseV6State s,uint8_t o,uint32_t n){g_reverseV6Origin=o;g_reverseV6Start=n;ReverseV6Apply(s);}
static void ReverseV6Publish(UdpReverseV6Event e,uint32_t n,WifiIotGpioValue l,WifiIotGpioValue r){UdpReverseV4TelemetryState t={0};t.event=(UdpReverseV4Event)e;t.sensorState=g_stableState;t.previousSensorState=g_reverseV6Previous;t.rawLeft=(l==WIFI_IOT_GPIO_VALUE1);t.rawRight=(r==WIFI_IOT_GPIO_VALUE1);t.triggerSensorState=g_reverseV6Origin;t.leftCommand=g_motorLeftCommand;t.rightCommand=g_motorRightCommand;t.pulseElapsedMs=AppTicksToMs(n-g_reverseV6Start);t.sameSidePulses=g_reverseV6ShiftCount;UdpTelemetryUpdateReverseV4(&t);g_reverseV6Last=n;}
static void ReverseV6Reset(uint32_t n){g_reverseV6State=REV6_WAIT_CLEAR;g_reverseV6Valid=0;g_reverseV6Capture=0;g_reverseV6Banner=0;g_reverseV6BackBanner=0;g_reverseV6ClearSince=g_reverseV6RearmSince=0;g_reverseV6ShiftCount=0;g_reverseV6Start=g_reverseV6Last=n;}
static void ReverseV6ControlStep(WifiIotGpioValue l,WifiIotGpioValue r,uint32_t n){uint8_t changed=0;ReverseV6State p=g_reverseV6State;UdpReverseV6Event e=UDP_REVERSE_V6_EVENT_HEARTBEAT;if(!g_reverseV6Valid){g_reverseV6Previous=g_stableState;g_reverseV6Current=g_stableState;g_reverseV6Valid=1;changed=1;}else{g_reverseV6Previous=g_reverseV6Current;g_reverseV6Current=g_stableState;if(g_reverseV6Previous!=g_reverseV6Current)changed=1;}if(g_reverseV6State==REV6_WAIT_CLEAR){ReverseV6Apply(REV6_WAIT_CLEAR);if(!g_stableState){if(!g_reverseV6ClearSince)g_reverseV6ClearSince=n;else if(n-g_reverseV6ClearSince>=AppMsToTicks(REV4_ARM_CLEAR_MS)){ReverseV6Enter(REV6_BACK,0,n);e=UDP_REVERSE_V6_EVENT_ARMED;}}else g_reverseV6ClearSince=0;}else if(g_reverseV6State==REV6_STOPPED){ReverseV6Apply(REV6_STOPPED);if(!g_stableState){if(!g_reverseV6RearmSince)g_reverseV6RearmSince=n;else if(n-g_reverseV6RearmSince>=AppMsToTicks(REV4_REARM_CLEAR_MS)){ReverseV6Enter(REV6_WAIT_CLEAR,0,n);e=UDP_REVERSE_V6_EVENT_REARM;}}else g_reverseV6RearmSince=0;}else if(g_stableState==3){ReverseV6Enter(REV6_STOPPED,g_reverseV6Origin,n);e=UDP_REVERSE_V6_EVENT_SENSOR_11_STOP;}else switch(g_reverseV6State){case REV6_BACK:if(g_stableState==2){ReverseV6Enter(REV6_HEADING_RIGHT,2,n);e=UDP_REVERSE_V6_EVENT_HEADING_START;}else if(g_stableState==1){ReverseV6Enter(REV6_HEADING_LEFT,1,n);e=UDP_REVERSE_V6_EVENT_HEADING_START;}else ReverseV6Apply(REV6_BACK);break;case REV6_HEADING_LEFT:case REV6_HEADING_RIGHT:if(g_stableState==0&&g_reverseV6Previous==g_reverseV6Origin){g_reverseV6Capture=1;ReverseV6Enter(REV6_VERIFY_BACK,g_reverseV6Origin,n);e=UDP_REVERSE_V6_EVENT_HEADING_CAPTURE_00;}else if(g_stableState!=g_reverseV6Origin&&g_stableState!=0){ReverseV6Enter(REV6_STOPPED,g_reverseV6Origin,n);e=UDP_REVERSE_V6_EVENT_HEADING_OVERSHOOT;}else if(n-g_reverseV6Start>=AppMsToTicks(HEADING_PULSE_MAX_MS)){g_reverseV6ActualYaw=HEADING_PULSE_MAX_MS;ReverseV6Enter(g_reverseV6Origin==2?REV6_SHIFT_OUT_LEFT:REV6_SHIFT_OUT_RIGHT,g_reverseV6Origin,n);e=UDP_REVERSE_V6_EVENT_LATERAL_START;}break;case REV6_SHIFT_OUT_LEFT:case REV6_SHIFT_OUT_RIGHT:if(g_stableState==0&&g_reverseV6Previous==g_reverseV6Origin){g_reverseV6Capture=1;g_reverseV6ActualYaw=n-g_reverseV6Start;g_reverseV6RestoreTarget=AppTicksToMs(g_reverseV6ActualYaw);ReverseV6Enter(g_reverseV6Origin==2?REV6_RESTORE_RIGHT:REV6_RESTORE_LEFT,g_reverseV6Origin,n);e=UDP_REVERSE_V6_EVENT_LATERAL_CAPTURE_00;}else if(g_stableState!=g_reverseV6Origin&&g_stableState!=0){ReverseV6Enter(REV6_STOPPED,g_reverseV6Origin,n);e=UDP_REVERSE_V6_EVENT_LATERAL_OPPOSITE_WITHOUT_CAPTURE;}else if(n-g_reverseV6Start>=AppMsToTicks(V6_SHIFT_YAW_OUT_MAX_MS)){ReverseV6Enter(REV6_SHIFT_BACK,g_reverseV6Origin,n);e=UDP_REVERSE_V6_EVENT_LATERAL_START;}break;case REV6_SHIFT_BACK:if(g_stableState==0&&g_reverseV6Previous==g_reverseV6Origin){g_reverseV6Capture=1;g_reverseV6RestoreTarget=V6_SHIFT_YAW_OUT_MAX_MS;ReverseV6Enter(g_reverseV6Origin==2?REV6_RESTORE_RIGHT:REV6_RESTORE_LEFT,g_reverseV6Origin,n);e=UDP_REVERSE_V6_EVENT_LATERAL_CAPTURE_00;}else if(g_stableState!=g_reverseV6Origin&&g_stableState!=0){ReverseV6Enter(REV6_STOPPED,g_reverseV6Origin,n);e=UDP_REVERSE_V6_EVENT_LATERAL_OPPOSITE_WITHOUT_CAPTURE;}else if(n-g_reverseV6Start>=AppMsToTicks(V6_SHIFT_BACK_MAX_MS)){ReverseV6Enter(REV6_STOPPED,g_reverseV6Origin,n);e=UDP_REVERSE_V6_EVENT_LATERAL_CAPTURE_TIMEOUT;}break;case REV6_RESTORE_LEFT:case REV6_RESTORE_RIGHT:if(g_stableState==g_reverseV6Origin){ReverseV6Enter(REV6_STOPPED,g_reverseV6Origin,n);e=UDP_REVERSE_V6_EVENT_RESTORE_RETURN_ORIGINAL_SIDE;}else if(g_stableState!=0){ReverseV6Enter(REV6_STOPPED,g_reverseV6Origin,n);e=UDP_REVERSE_V6_EVENT_RESTORE_CROSS_OPPOSITE_SIDE;}else if(n-g_reverseV6Start>=AppMsToTicks(g_reverseV6RestoreTarget)){ReverseV6Enter(REV6_VERIFY_BACK,g_reverseV6Origin,n);e=UDP_REVERSE_V6_EVENT_VERIFY_START;}break;case REV6_VERIFY_BACK:if(g_stableState!=0){ReverseV6Enter(REV6_STOPPED,g_reverseV6Origin,n);}else if(n-g_reverseV6Start>=AppMsToTicks(V6_VERIFY_BACK_MS)){ReverseV6Enter(REV6_BACK,0,n);e=UDP_REVERSE_V6_EVENT_TRACK_REACQUIRED;}break;default:break;}if(!g_reverseV6Banner){g_reverseV6Banner=1;printf("=== ACTIVE CAR MODE: REVERSE V6 ===\\r\\nBUILD MODE: REVERSE_V6 %s %s\\r\\n",__DATE__,__TIME__);}if(changed||p!=g_reverseV6State||e!=UDP_REVERSE_V6_EVENT_HEARTBEAT)ReverseV6Publish(e,n,l,r);else if(n-g_reverseV6Last>=AppMsToTicks(REV4_DEBUG_HEARTBEAT_MS))ReverseV6Publish(UDP_REVERSE_V6_EVENT_HEARTBEAT,n,l,r);}
#endif

#if (TRACE_REVERSE_V7_TEST_MODE == 1)
typedef enum {REV7_WAIT_CLEAR=0,REV7_BACK,REV7_HEADING_LEFT,REV7_HEADING_RIGHT,REV7_SWEEP_ENTRY_LEFT,REV7_SWEEP_ENTRY_RIGHT,REV7_SWEEP_LEFT,REV7_SWEEP_RIGHT,REV7_OPPOSITE,REV7_RETURN_LEFT,REV7_RETURN_RIGHT,REV7_VERIFY,REV7_STOPPED} ReverseV7State;
static ReverseV7State g_reverseV7State=REV7_STOPPED; static uint8_t g_reverseV7Valid,g_reverseV7Cur,g_reverseV7Prev,g_reverseV7Origin,g_reverseV7Center,g_reverseV7Opp,g_reverseV7ReturnSeen; static uint32_t g_reverseV7Start,g_reverseV7Clear,g_reverseV7Rearm,g_reverseV7Last; static int g_reverseV7Banner;
static void ReverseV7Apply(ReverseV7State s){int l=0,r=0;switch(s){case REV7_BACK:case REV7_SWEEP_LEFT:case REV7_SWEEP_RIGHT:case REV7_VERIFY:l=r=-REV4_BACK_SPEED;break;case REV7_HEADING_RIGHT:case REV7_SWEEP_ENTRY_RIGHT:l=-REV4_INNER_SPEED;r=-REV4_OUTER_SPEED;break;case REV7_HEADING_LEFT:case REV7_SWEEP_ENTRY_LEFT:l=-REV4_OUTER_SPEED;r=-REV4_INNER_SPEED;break;case REV7_RETURN_LEFT:l=-REV4_OUTER_SPEED;r=-REV4_INNER_SPEED;break;case REV7_RETURN_RIGHT:l=-REV4_INNER_SPEED;r=-REV4_OUTER_SPEED;break;default:break;}TraceSendMotorCommand(l,r,0);g_reverseV7State=s;}
static void ReverseV7Enter(ReverseV7State s,uint8_t o,uint32_t n){g_reverseV7Origin=o;g_reverseV7Start=n;ReverseV7Apply(s);}
static void ReverseV7Publish(UdpReverseV7Event e,uint32_t n,WifiIotGpioValue l,WifiIotGpioValue r){UdpReverseV4TelemetryState t={0};t.event=(UdpReverseV4Event)e;t.state=(UdpReverseV4State)(300+g_reverseV7State);t.sensorState=g_stableState;t.previousSensorState=g_reverseV7Prev;t.rawLeft=(l==WIFI_IOT_GPIO_VALUE1);t.rawRight=(r==WIFI_IOT_GPIO_VALUE1);t.triggerSensorState=g_reverseV7Origin;t.leftCommand=g_motorLeftCommand;t.rightCommand=g_motorRightCommand;t.pulseElapsedMs=AppTicksToMs(n-g_reverseV7Start);t.targetMs=(g_reverseV7State==REV7_SWEEP_ENTRY_LEFT||g_reverseV7State==REV7_SWEEP_ENTRY_RIGHT)?V7_SWEEP_ENTRY_YAW_MS:0U;UdpTelemetryUpdateReverseV4(&t);g_reverseV7Last=n;}
static void ReverseV7Reset(uint32_t n){g_reverseV7State=REV7_WAIT_CLEAR;g_reverseV7Valid=0;g_reverseV7Center=g_reverseV7Opp=g_reverseV7ReturnSeen=0;g_reverseV7Banner=0;g_reverseV7Clear=g_reverseV7Rearm=0;g_reverseV7Start=g_reverseV7Last=n;}
static void ReverseV7ControlStep(WifiIotGpioValue l,WifiIotGpioValue r,uint32_t n){uint8_t ch=0;ReverseV7State p=g_reverseV7State;UdpReverseV7Event e=UDP_REVERSE_V7_EVENT_HEARTBEAT;if(!g_reverseV7Valid){g_reverseV7Prev=g_stableState;g_reverseV7Cur=g_stableState;g_reverseV7Valid=1;ch=1;}else{g_reverseV7Prev=g_reverseV7Cur;g_reverseV7Cur=g_stableState;ch=(g_reverseV7Prev!=g_reverseV7Cur);}if(g_reverseV7State==REV7_WAIT_CLEAR){ReverseV7Apply(REV7_WAIT_CLEAR);if(!g_stableState){if(!g_reverseV7Clear)g_reverseV7Clear=n;else if(n-g_reverseV7Clear>=AppMsToTicks(REV4_ARM_CLEAR_MS)){ReverseV7Enter(REV7_BACK,0,n);e=UDP_REVERSE_V7_EVENT_ARMED;}}else g_reverseV7Clear=0;}else if(g_reverseV7State==REV7_STOPPED){ReverseV7Apply(REV7_STOPPED);if(!g_stableState){if(!g_reverseV7Rearm)g_reverseV7Rearm=n;else if(n-g_reverseV7Rearm>=AppMsToTicks(REV4_REARM_CLEAR_MS)){ReverseV7Enter(REV7_WAIT_CLEAR,0,n);e=UDP_REVERSE_V7_EVENT_REARM;}}else g_reverseV7Rearm=0;}else if(g_stableState==3){ReverseV7Enter(REV7_STOPPED,g_reverseV7Origin,n);e=UDP_REVERSE_V7_EVENT_SENSOR_11_STOP;}else switch(g_reverseV7State){case REV7_BACK:if(g_stableState==2){ReverseV7Enter(REV7_SWEEP_ENTRY_RIGHT,2,n);e=UDP_REVERSE_V7_EVENT_SWEEP_ENTRY_YAW_START;}else if(g_stableState==1){ReverseV7Enter(REV7_SWEEP_ENTRY_LEFT,1,n);e=UDP_REVERSE_V7_EVENT_SWEEP_ENTRY_YAW_START;}else ReverseV7Apply(REV7_BACK);break;case REV7_HEADING_LEFT:case REV7_HEADING_RIGHT:if(g_stableState==0&&g_reverseV7Prev==g_reverseV7Origin){ReverseV7Enter(REV7_VERIFY,g_reverseV7Origin,n);e=UDP_REVERSE_V7_EVENT_CENTER_WINDOW_ENTER;}else if(g_stableState!=0&&g_stableState!=g_reverseV7Origin){ReverseV7Enter(REV7_STOPPED,g_reverseV7Origin,n);e=UDP_REVERSE_V7_EVENT_VERIFY_OPPOSITE_SIDE;}else if(n-g_reverseV7Start>=AppMsToTicks(HEADING_PULSE_MAX_MS)){ReverseV7Enter(g_reverseV7Origin==2?REV7_SWEEP_ENTRY_RIGHT:REV7_SWEEP_ENTRY_LEFT,g_reverseV7Origin,n);e=UDP_REVERSE_V7_EVENT_SWEEP_ENTRY_YAW_START;}break;case REV7_SWEEP_ENTRY_LEFT:case REV7_SWEEP_ENTRY_RIGHT:if(g_stableState!=0&&g_stableState!=g_reverseV7Origin){ReverseV7Enter(REV7_STOPPED,g_reverseV7Origin,n);e=UDP_REVERSE_V7_EVENT_VERIFY_OPPOSITE_SIDE;}else if(n-g_reverseV7Start>=AppMsToTicks(V7_SWEEP_ENTRY_YAW_MS)){ReverseV7Enter(g_reverseV7Origin==2?REV7_SWEEP_LEFT:REV7_SWEEP_RIGHT,g_reverseV7Origin,n);e=UDP_REVERSE_V7_EVENT_SWEEP_START;}break;case REV7_SWEEP_LEFT:case REV7_SWEEP_RIGHT:if(g_stableState==0&&g_reverseV7Prev==g_reverseV7Origin)g_reverseV7Center=1;if(g_stableState==(g_reverseV7Origin==2?1:2)){g_reverseV7Opp=1;ReverseV7Enter(g_reverseV7Origin==2?REV7_RETURN_LEFT:REV7_RETURN_RIGHT,g_reverseV7Origin,n);e=UDP_REVERSE_V7_EVENT_OPPOSITE_EDGE_CONFIRMED;}else if(n-g_reverseV7Start>=AppMsToTicks(V7_SWEEP_TO_OPPOSITE_MAX_MS)){ReverseV7Enter(REV7_STOPPED,g_reverseV7Origin,n);e=UDP_REVERSE_V7_EVENT_OPPOSITE_EDGE_TIMEOUT;}break;case REV7_RETURN_LEFT:case REV7_RETURN_RIGHT:if(g_stableState==0&&g_reverseV7Prev!=(g_reverseV7Origin)){g_reverseV7ReturnSeen=1;ReverseV7Enter(REV7_VERIFY,g_reverseV7Origin,n);e=UDP_REVERSE_V7_EVENT_RETURN_CENTER_CAPTURE;}else if(n-g_reverseV7Start>=AppMsToTicks(HEADING_PULSE_MAX_MS)){ReverseV7Enter(REV7_STOPPED,g_reverseV7Origin,n);e=UDP_REVERSE_V7_EVENT_OPPOSITE_EDGE_TIMEOUT;}break;case REV7_VERIFY:if(g_stableState!=0){ReverseV7Enter(REV7_STOPPED,g_reverseV7Origin,n);e=(g_stableState==g_reverseV7Origin)?UDP_REVERSE_V7_EVENT_VERIFY_ORIGINAL_SIDE:UDP_REVERSE_V7_EVENT_VERIFY_OPPOSITE_SIDE;}else if(n-g_reverseV7Start>=AppMsToTicks(V7_VERIFY_CENTER_MS)){ReverseV7Enter(REV7_BACK,0,n);e=UDP_REVERSE_V7_EVENT_TRACK_REACQUIRED;}break;default:break;}if(!g_reverseV7Banner){g_reverseV7Banner=1;printf("=== ACTIVE CAR MODE: REVERSE V7 ===\\r\\nBUILD MODE: REVERSE_V7 %s %s\\r\\n",__DATE__,__TIME__);}if(ch||p!=g_reverseV7State||e!=UDP_REVERSE_V7_EVENT_HEARTBEAT)ReverseV7Publish(e,n,l,r);else if(n-g_reverseV7Last>=AppMsToTicks(REV4_DEBUG_HEARTBEAT_MS))ReverseV7Publish(UDP_REVERSE_V7_EVENT_HEARTBEAT,n,l,r);}
#endif

#if (TRACE_REVERSE_V8_TEST_MODE == 1) && (TRACE_REVERSE_V8_SIMPLE_SERVO_MODE == 0)
/*
 * V8 is a straight-line experiment.  Persistent 10/01 is treated as a
 * candidate observable edge lock; it is not a proof of centered tracking.
 */
typedef enum {
    REV8_WAIT_CLEAR = 0,
    REV8_BACK,
    REV8_EDGE_ENTRY_RIGHT,
    REV8_EDGE_ENTRY_LEFT,
    REV8_EDGE_CANDIDATE,
    REV8_EDGE_LOCKED,
    REV8_EDGE_GAP,
    REV8_STOPPED
} ReverseV8State;

typedef enum {
    REV8_CORRECTION_NONE = 0,
    REV8_CORRECTION_TOWARD,
    REV8_CORRECTION_AWAY
} ReverseV8Correction;

static ReverseV8State g_reverseV8State = REV8_STOPPED;
static uint8_t g_reverseV8Valid;
static uint8_t g_reverseV8Current;
static uint8_t g_reverseV8Previous;
static uint8_t g_reverseV8EdgeSide;
static uint32_t g_reverseV8Start;
static uint32_t g_reverseV8ClearSince;
static uint32_t g_reverseV8LastTelemetry;
static uint32_t g_reverseV8CorrectionStart;
static ReverseV8Correction g_reverseV8Correction;
static uint8_t g_reverseV8CorrectionActive;
static int g_reverseV8Banner;

static void ReverseV8Apply(ReverseV8State state)
{
    int leftCommand = 0;
    int rightCommand = 0;

    switch (state) {
        case REV8_BACK:
        case REV8_EDGE_CANDIDATE:
        case REV8_EDGE_LOCKED:
        case REV8_EDGE_GAP:
            leftCommand = -REV4_BACK_SPEED;
            rightCommand = -REV4_BACK_SPEED;
            break;
        case REV8_EDGE_ENTRY_RIGHT:
            /* physical-left edge (10): yaw RIGHT while reversing. */
            leftCommand = -REV4_INNER_SPEED;
            rightCommand = -REV4_OUTER_SPEED;
            break;
        case REV8_EDGE_ENTRY_LEFT:
            /* physical-right edge (01): yaw LEFT while reversing. */
            leftCommand = -REV4_OUTER_SPEED;
            rightCommand = -REV4_INNER_SPEED;
            break;
        default:
            break;
    }

    TraceSendMotorCommand(leftCommand, rightCommand, 0);
    g_reverseV8Correction = REV8_CORRECTION_NONE;
    g_reverseV8CorrectionActive = 0U;
    g_reverseV8State = state;
}

static void ReverseV8ApplyCorrection(ReverseV8Correction correction, uint32_t now)
{
    int leftCommand;
    int rightCommand;

    if (g_reverseV8EdgeSide == 1U) {
        /* RIGHT edge: toward=line LEFT yaw; away=line RIGHT yaw. */
        leftCommand = (correction == REV8_CORRECTION_TOWARD) ?
                      -REV4_OUTER_SPEED : -REV4_INNER_SPEED;
        rightCommand = (correction == REV8_CORRECTION_TOWARD) ?
                       -REV4_INNER_SPEED : -REV4_OUTER_SPEED;
    } else {
        /* LEFT edge: toward=line RIGHT yaw; away=line LEFT yaw. */
        leftCommand = (correction == REV8_CORRECTION_TOWARD) ?
                      -REV4_INNER_SPEED : -REV4_OUTER_SPEED;
        rightCommand = (correction == REV8_CORRECTION_TOWARD) ?
                       -REV4_OUTER_SPEED : -REV4_INNER_SPEED;
    }

    TraceSendMotorCommand(leftCommand, rightCommand, 0);
    g_reverseV8Correction = correction;
    g_reverseV8CorrectionActive = 1U;
    g_reverseV8CorrectionStart = now;
}

/* Return 1 while the one-step pulse owns the motor; 2 when it just ended. */
static int ReverseV8CorrectionPoll(uint32_t now)
{
    if (g_reverseV8CorrectionActive == 0U) {
        return 0;
    }
    if ((uint32_t)(now - g_reverseV8CorrectionStart) <
        AppMsToTicks(V8_EDGE_CORRECTION_MS)) {
        return 1;
    }

    g_reverseV8CorrectionActive = 0U;
    ReverseV8Apply(g_reverseV8State);
    return 2;
}

static void ReverseV8Enter(ReverseV8State state, uint8_t edgeSide, uint32_t now)
{
    g_reverseV8EdgeSide = edgeSide;
    g_reverseV8Start = now;
    ReverseV8Apply(state);
}

static uint32_t ReverseV8TargetMs(void)
{
    if (g_reverseV8CorrectionActive != 0U) {
        return V8_EDGE_CORRECTION_MS;
    }
    if (g_reverseV8State == REV8_EDGE_ENTRY_RIGHT ||
        g_reverseV8State == REV8_EDGE_ENTRY_LEFT) {
        return V8_EDGE_ENTRY_MS;
    }
    if (g_reverseV8State == REV8_EDGE_CANDIDATE) {
        return V8_EDGE_LOCK_CONFIRM_MS;
    }
    if (g_reverseV8State == REV8_EDGE_GAP) {
        return V8_EDGE_GAP_GRACE_MS;
    }
    return 0U;
}

static void ReverseV8Publish(UdpReverseV8Event event, uint32_t now,
                             WifiIotGpioValue left, WifiIotGpioValue right)
{
    UdpReverseV4TelemetryState telemetry = {0};

    telemetry.event = (UdpReverseV4Event)event;
    telemetry.state = (UdpReverseV4State)(400 + g_reverseV8State);
    telemetry.sensorState = g_stableState;
    telemetry.previousSensorState = g_reverseV8Previous;
    telemetry.rawLeft = (left == WIFI_IOT_GPIO_VALUE1);
    telemetry.rawRight = (right == WIFI_IOT_GPIO_VALUE1);
    telemetry.triggerSensorState = g_reverseV8EdgeSide;
    telemetry.edgeSide = g_reverseV8EdgeSide;
    telemetry.correctionDirection = (uint8_t)g_reverseV8Correction;
    telemetry.leftCommand = g_motorLeftCommand;
    telemetry.rightCommand = g_motorRightCommand;
    telemetry.pulseElapsedMs = AppTicksToMs((uint32_t)(now - g_reverseV8Start));
    telemetry.targetMs = ReverseV8TargetMs();
    UdpTelemetryUpdateReverseV4(&telemetry);
    g_reverseV8LastTelemetry = now;
}

static void ReverseV8Reset(uint32_t now)
{
    g_reverseV8State = REV8_WAIT_CLEAR;
    g_reverseV8Valid = 0U;
    g_reverseV8Current = 0U;
    g_reverseV8Previous = 0U;
    g_reverseV8EdgeSide = 0U;
    g_reverseV8ClearSince = 0U;
    g_reverseV8CorrectionStart = now;
    g_reverseV8Correction = REV8_CORRECTION_NONE;
    g_reverseV8CorrectionActive = 0U;
    g_reverseV8Start = now;
    g_reverseV8LastTelemetry = now;
    g_reverseV8Banner = 0;
}

static void ReverseV8ControlStep(WifiIotGpioValue left, WifiIotGpioValue right,
                                 uint32_t now)
{
    ReverseV8State previousState = g_reverseV8State;
    UdpReverseV8Event event = UDP_REVERSE_V8_EVENT_HEARTBEAT;
    uint8_t changed = 0U;
    uint8_t motorChanged = 0U;
    uint8_t rawState = ((left == WIFI_IOT_GPIO_VALUE1) ? 2U : 0U) |
                       ((right == WIFI_IOT_GPIO_VALUE1) ? 1U : 0U);

    if (g_reverseV8Valid == 0U) {
        g_reverseV8Previous = g_stableState;
        g_reverseV8Current = g_stableState;
        g_reverseV8Valid = 1U;
        changed = 1U;
        event = UDP_REVERSE_V8_EVENT_START;
    } else {
        g_reverseV8Previous = g_reverseV8Current;
        g_reverseV8Current = g_stableState;
        changed = (g_reverseV8Previous != g_reverseV8Current);
    }

    if (changed != 0U) {
        event = UDP_REVERSE_V8_EVENT_SIMPLE_STATE_CHANGE;
    }

    if (g_reverseV8State == REV8_WAIT_CLEAR) {
        ReverseV8Apply(REV8_WAIT_CLEAR);
        if (g_stableState == 0U) {
            if (g_reverseV8ClearSince == 0U) {
                g_reverseV8ClearSince = now;
            } else if ((uint32_t)(now - g_reverseV8ClearSince) >=
                       AppMsToTicks(REV4_ARM_CLEAR_MS)) {
                ReverseV8Enter(REV8_BACK, 0U, now);
                event = UDP_REVERSE_V8_EVENT_ARMED;
            }
        } else {
            g_reverseV8ClearSince = 0U;
        }
    } else if (g_reverseV8State == REV8_STOPPED) {
        /* V8 safety stops are latched until the mode is restarted. */
        ReverseV8Apply(REV8_STOPPED);
    } else if (g_stableState == 3U) {
        ReverseV8Enter(REV8_STOPPED, g_reverseV8EdgeSide, now);
        event = UDP_REVERSE_V8_EVENT_SENSOR_11_STOP;
    } else {
        switch (g_reverseV8State) {
            case REV8_BACK:
                if (g_stableState == 2U) {
                    ReverseV8Enter(REV8_EDGE_ENTRY_RIGHT, 2U, now);
                    event = UDP_REVERSE_V8_EVENT_EDGE_ENTRY_START;
                } else if (g_stableState == 1U) {
                    ReverseV8Enter(REV8_EDGE_ENTRY_LEFT, 1U, now);
                    event = UDP_REVERSE_V8_EVENT_EDGE_ENTRY_START;
                } else {
                    ReverseV8Apply(REV8_BACK);
                }
                break;

            case REV8_EDGE_ENTRY_RIGHT:
            case REV8_EDGE_ENTRY_LEFT:
                if ((uint32_t)(now - g_reverseV8Start) >= AppMsToTicks(V8_EDGE_ENTRY_MS)) {
                    ReverseV8Enter(REV8_EDGE_CANDIDATE, g_reverseV8EdgeSide, now);
                    event = UDP_REVERSE_V8_EVENT_EDGE_CANDIDATE_START;
                }
                break;

            case REV8_EDGE_CANDIDATE:
                if (g_stableState == g_reverseV8EdgeSide) {
                    if ((uint32_t)(now - g_reverseV8Start) >=
                        AppMsToTicks(V8_EDGE_LOCK_CONFIRM_MS)) {
                        ReverseV8Enter(REV8_EDGE_LOCKED, g_reverseV8EdgeSide, now);
                        event = UDP_REVERSE_V8_EVENT_EDGE_LOCKED;
                    } else {
                        ReverseV8Apply(REV8_EDGE_CANDIDATE);
                    }
                } else if (g_stableState == 0U) {
                    ReverseV8Enter(REV8_STOPPED, g_reverseV8EdgeSide, now);
                    event = UDP_REVERSE_V8_EVENT_EDGE_LOST;
                } else {
                    ReverseV8Enter(REV8_STOPPED, g_reverseV8EdgeSide, now);
                    event = UDP_REVERSE_V8_EVENT_EDGE_SIDE_SWITCH;
                }
                break;

            case REV8_EDGE_LOCKED:
                if (g_stableState == g_reverseV8EdgeSide) {
                    int correctionStatus = ReverseV8CorrectionPoll(now);
                    if (correctionStatus == 2) {
                        motorChanged = 1U;
                    } else if (correctionStatus == 0) {
                        if (rawState == 0U) {
                            ReverseV8ApplyCorrection(REV8_CORRECTION_TOWARD, now);
                            event = UDP_REVERSE_V8_EVENT_EDGE_CORRECT_TOWARD;
                        } else if (rawState == 3U) {
                            ReverseV8ApplyCorrection(REV8_CORRECTION_AWAY, now);
                            event = UDP_REVERSE_V8_EVENT_EDGE_CORRECT_AWAY;
                        } else {
                            ReverseV8Apply(REV8_EDGE_LOCKED);
                        }
                    }
                } else if (g_stableState == 0U) {
                    ReverseV8Enter(REV8_EDGE_GAP, g_reverseV8EdgeSide, now);
                    event = UDP_REVERSE_V8_EVENT_EDGE_GAP_START;
                } else {
                    ReverseV8Enter(REV8_STOPPED, g_reverseV8EdgeSide, now);
                    event = UDP_REVERSE_V8_EVENT_EDGE_SIDE_SWITCH;
                }
                break;

            case REV8_EDGE_GAP:
                if (g_stableState == g_reverseV8EdgeSide) {
                    ReverseV8Enter(REV8_EDGE_LOCKED, g_reverseV8EdgeSide, now);
                    event = UDP_REVERSE_V8_EVENT_EDGE_REACQUIRED;
                } else if (g_stableState != 0U) {
                    ReverseV8Enter(REV8_STOPPED, g_reverseV8EdgeSide, now);
                    event = UDP_REVERSE_V8_EVENT_EDGE_SIDE_SWITCH;
                } else if ((uint32_t)(now - g_reverseV8Start) >=
                           AppMsToTicks(V8_EDGE_GAP_GRACE_MS)) {
                    ReverseV8Enter(REV8_STOPPED, g_reverseV8EdgeSide, now);
                    event = UDP_REVERSE_V8_EVENT_EDGE_LOST;
                } else {
                    int correctionStatus = ReverseV8CorrectionPoll(now);
                    if (correctionStatus == 2) {
                        motorChanged = 1U;
                    } else if (correctionStatus == 0) {
                        ReverseV8ApplyCorrection(REV8_CORRECTION_TOWARD, now);
                        event = UDP_REVERSE_V8_EVENT_EDGE_CORRECT_TOWARD;
                    }
                }
                break;

            default:
                break;
        }
    }

    if (g_reverseV8Banner == 0) {
        g_reverseV8Banner = 1;
        printf("=== ACTIVE CAR MODE: REVERSE V8 ===\r\n");
    }
    if (changed != 0U || motorChanged != 0U || previousState != g_reverseV8State ||
        event != UDP_REVERSE_V8_EVENT_HEARTBEAT) {
        ReverseV8Publish(event, now, left, right);
    } else if ((uint32_t)(now - g_reverseV8LastTelemetry) >=
               AppMsToTicks(V8_DEBUG_HEARTBEAT_MS)) {
        ReverseV8Publish(UDP_REVERSE_V8_EVENT_HEARTBEAT, now, left, right);
    }
}
#endif

#if (TRACE_REVERSE_V8_TEST_MODE == 1) && (TRACE_REVERSE_V8_SIMPLE_SERVO_MODE == 1) && \
    (TRACE_REVERSE_V8_PAIRED_RECENTER_MODE == 0) && \
    (TRACE_REVERSE_V8_MICRO_PAIRED_MODE == 0)
/* V8.2 retains V8.1 above, but moves only from the two-sample stable state. */
typedef enum {
    REV8_WAIT_CLEAR = 0,
    REV8_SIMPLE_STRAIGHT,
    REV8_SIMPLE_NUDGE_RIGHT,
    REV8_SIMPLE_EDGE_LEFT_HOLD,
    REV8_SIMPLE_NUDGE_LEFT,
    REV8_SIMPLE_EDGE_RIGHT_HOLD,
    REV8_SIMPLE_TRANSIT_LEFT_TO_RIGHT,
    REV8_SIMPLE_TRANSIT_RIGHT_TO_LEFT,
    REV8_SIMPLE_RIDE_BACK_NUDGE_LEFT,
    REV8_SIMPLE_RIDE_BACK_HOLD_LEFT,
    REV8_SIMPLE_RIDE_BACK_NUDGE_RIGHT,
    REV8_SIMPLE_RIDE_BACK_HOLD_RIGHT,
    REV8_SIMPLE_SEARCH_BACK_LEFT,
    REV8_SIMPLE_SEARCH_BACK_RIGHT,
    REV8_STOPPED
} ReverseV8State;

static ReverseV8State g_reverseV8State = REV8_STOPPED;
static uint8_t g_reverseV8Valid, g_reverseV8Current, g_reverseV8Previous;
static uint8_t g_reverseV8EdgeSide;
static uint32_t g_reverseV8Start, g_reverseV8ClearSince, g_reverseV8LastTelemetry;
static uint32_t g_reverseV8LastNudgeElapsedMs;
static int g_reverseV8Banner;

#define V8_SIMPLE_TRANSIT_MAX_MS      500U
#define V8_SIMPLE_SEARCH_BACK_MS      800U
#define V8_SIMPLE_NUDGE_MS             120U

static void ReverseV8Apply(ReverseV8State state)
{
    int leftCommand = 0;
    int rightCommand = 0;

    switch (state) {
        case REV8_SIMPLE_STRAIGHT:
        case REV8_SIMPLE_EDGE_LEFT_HOLD:
        case REV8_SIMPLE_EDGE_RIGHT_HOLD:
        case REV8_SIMPLE_RIDE_BACK_HOLD_LEFT:
        case REV8_SIMPLE_RIDE_BACK_HOLD_RIGHT:
            leftCommand = rightCommand = -REV4_BACK_SPEED;
            break;
        case REV8_SIMPLE_NUDGE_RIGHT:
        case REV8_SIMPLE_TRANSIT_LEFT_TO_RIGHT:
        case REV8_SIMPLE_RIDE_BACK_NUDGE_RIGHT:
        case REV8_SIMPLE_SEARCH_BACK_RIGHT:
            /* Right wheel boost: reverse yaw RIGHT. */
            leftCommand = -REV4_BACK_SPEED;
            rightCommand = -REV4_OUTER_SPEED;
            break;
        case REV8_SIMPLE_NUDGE_LEFT:
        case REV8_SIMPLE_TRANSIT_RIGHT_TO_LEFT:
        case REV8_SIMPLE_RIDE_BACK_NUDGE_LEFT:
        case REV8_SIMPLE_SEARCH_BACK_LEFT:
            /* Left wheel boost: reverse yaw LEFT. */
            leftCommand = -REV4_OUTER_SPEED;
            rightCommand = -REV4_BACK_SPEED;
            break;
        default:
            break;
    }
    if (g_motorCommandValid == 0U || g_motorLeftCommand != leftCommand ||
        g_motorRightCommand != rightCommand) {
        TraceSendMotorCommand(leftCommand, rightCommand, 0);
    }
    g_reverseV8State = state;
}

static int ReverseV8IsNudgeState(ReverseV8State state);

static void ReverseV8SetState(ReverseV8State state, uint32_t now)
{
    if (g_reverseV8State != state) {
        g_reverseV8Start = now;
        if (ReverseV8IsNudgeState(state) != 0) {
            g_reverseV8LastNudgeElapsedMs = 0U;
        }
    }
    ReverseV8Apply(state);
}

static int ReverseV8IsNudgeState(ReverseV8State state)
{
    return state == REV8_SIMPLE_NUDGE_RIGHT ||
           state == REV8_SIMPLE_NUDGE_LEFT ||
           state == REV8_SIMPLE_RIDE_BACK_NUDGE_LEFT ||
           state == REV8_SIMPLE_RIDE_BACK_NUDGE_RIGHT;
}

static void ReverseV8Publish(UdpReverseV8Event event, uint32_t now,
                             WifiIotGpioValue left, WifiIotGpioValue right)
{
    UdpReverseV4TelemetryState telemetry = {0};
    telemetry.event = (UdpReverseV4Event)event;
    telemetry.state = (UdpReverseV4State)(420 + g_reverseV8State);
    telemetry.sensorState = g_stableState;
    telemetry.previousSensorState = g_reverseV8Previous;
    telemetry.rawLeft = (left == WIFI_IOT_GPIO_VALUE1);
    telemetry.rawRight = (right == WIFI_IOT_GPIO_VALUE1);
    telemetry.triggerSensorState = g_reverseV8EdgeSide;
    telemetry.edgeSide = g_reverseV8EdgeSide;
    telemetry.expectedSensorState = (g_reverseV8EdgeSide == 2U) ? 1U :
                                      (g_reverseV8EdgeSide == 1U) ? 2U : 0U;
    telemetry.leftCommand = g_motorLeftCommand;
    telemetry.rightCommand = g_motorRightCommand;
    telemetry.pulseElapsedMs = AppTicksToMs((uint32_t)(now - g_reverseV8Start));
    if (g_reverseV8State == REV8_SIMPLE_TRANSIT_LEFT_TO_RIGHT ||
        g_reverseV8State == REV8_SIMPLE_TRANSIT_RIGHT_TO_LEFT) {
        telemetry.transitElapsedMs = telemetry.pulseElapsedMs;
    }
    if (g_reverseV8State == REV8_SIMPLE_SEARCH_BACK_LEFT ||
        g_reverseV8State == REV8_SIMPLE_SEARCH_BACK_RIGHT) {
        telemetry.searchElapsedMs = telemetry.pulseElapsedMs;
    }
    if (ReverseV8IsNudgeState(g_reverseV8State) != 0) {
        telemetry.nudgeElapsedMs = telemetry.pulseElapsedMs;
        telemetry.targetMs = V8_SIMPLE_NUDGE_MS;
    } else if (event == UDP_REVERSE_V8_EVENT_SIMPLE_NUDGE_END) {
        telemetry.nudgeElapsedMs = g_reverseV8LastNudgeElapsedMs;
        telemetry.targetMs = V8_SIMPLE_NUDGE_MS;
    }
    UdpTelemetryUpdateReverseV4(&telemetry);
    g_reverseV8LastTelemetry = now;
}

static void ReverseV8Reset(uint32_t now)
{
    g_reverseV8State = REV8_WAIT_CLEAR;
    g_reverseV8Valid = 0U;
    g_reverseV8Current = 0U;
    g_reverseV8Previous = 0U;
    g_reverseV8EdgeSide = 0U;
    g_reverseV8ClearSince = 0U;
    g_reverseV8LastNudgeElapsedMs = 0U;
    g_reverseV8Start = now;
    g_reverseV8LastTelemetry = now;
    g_reverseV8Banner = 0;
}

static UdpReverseV8Event ReverseV8SelectSimpleAction(uint32_t now,
                                                       uint8_t sensorChanged)
{
    switch (g_stableState) {
        case 0U:
            /* 00 is straight only before either physical edge has been seen. */
            if (g_reverseV8EdgeSide == 2U) {
                ReverseV8SetState(REV8_SIMPLE_TRANSIT_LEFT_TO_RIGHT, now);
            } else if (g_reverseV8EdgeSide == 1U) {
                ReverseV8SetState(REV8_SIMPLE_TRANSIT_RIGHT_TO_LEFT, now);
            } else {
                ReverseV8SetState(REV8_SIMPLE_STRAIGHT, now);
            }
            break;
        case 2U: /* physical LEFT black: boost the opposite right wheel. */
            g_reverseV8EdgeSide = 2U;
            if (sensorChanged != 0U) {
                ReverseV8SetState(REV8_SIMPLE_NUDGE_RIGHT, now);
                return UDP_REVERSE_V8_EVENT_SIMPLE_NUDGE_START;
            }
            if (g_reverseV8State == REV8_SIMPLE_NUDGE_RIGHT &&
                (uint32_t)(now - g_reverseV8Start) >=
                AppMsToTicks(V8_SIMPLE_NUDGE_MS)) {
                g_reverseV8LastNudgeElapsedMs =
                    AppTicksToMs((uint32_t)(now - g_reverseV8Start));
                ReverseV8SetState(REV8_SIMPLE_EDGE_LEFT_HOLD, now);
                return UDP_REVERSE_V8_EVENT_SIMPLE_NUDGE_END;
            }
            break;
        case 1U: /* physical RIGHT black: boost the opposite left wheel. */
            g_reverseV8EdgeSide = 1U;
            if (sensorChanged != 0U) {
                ReverseV8SetState(REV8_SIMPLE_NUDGE_LEFT, now);
                return UDP_REVERSE_V8_EVENT_SIMPLE_NUDGE_START;
            }
            if (g_reverseV8State == REV8_SIMPLE_NUDGE_LEFT &&
                (uint32_t)(now - g_reverseV8Start) >=
                AppMsToTicks(V8_SIMPLE_NUDGE_MS)) {
                g_reverseV8LastNudgeElapsedMs =
                    AppTicksToMs((uint32_t)(now - g_reverseV8Start));
                ReverseV8SetState(REV8_SIMPLE_EDGE_RIGHT_HOLD, now);
                return UDP_REVERSE_V8_EVENT_SIMPLE_NUDGE_END;
            }
            break;
        default: /* 11 rides back using the last known edge; unknown stays stopped. */
            if (g_reverseV8EdgeSide == 2U) {
                if (sensorChanged != 0U) {
                    ReverseV8SetState(REV8_SIMPLE_RIDE_BACK_NUDGE_LEFT, now);
                    return UDP_REVERSE_V8_EVENT_SIMPLE_NUDGE_START;
                }
                if (g_reverseV8State == REV8_SIMPLE_RIDE_BACK_NUDGE_LEFT &&
                    (uint32_t)(now - g_reverseV8Start) >=
                    AppMsToTicks(V8_SIMPLE_NUDGE_MS)) {
                    g_reverseV8LastNudgeElapsedMs =
                        AppTicksToMs((uint32_t)(now - g_reverseV8Start));
                    ReverseV8SetState(REV8_SIMPLE_RIDE_BACK_HOLD_LEFT, now);
                    return UDP_REVERSE_V8_EVENT_SIMPLE_NUDGE_END;
                }
            } else if (g_reverseV8EdgeSide == 1U) {
                if (sensorChanged != 0U) {
                    ReverseV8SetState(REV8_SIMPLE_RIDE_BACK_NUDGE_RIGHT, now);
                    return UDP_REVERSE_V8_EVENT_SIMPLE_NUDGE_START;
                }
                if (g_reverseV8State == REV8_SIMPLE_RIDE_BACK_NUDGE_RIGHT &&
                    (uint32_t)(now - g_reverseV8Start) >=
                    AppMsToTicks(V8_SIMPLE_NUDGE_MS)) {
                    g_reverseV8LastNudgeElapsedMs =
                        AppTicksToMs((uint32_t)(now - g_reverseV8Start));
                    ReverseV8SetState(REV8_SIMPLE_RIDE_BACK_HOLD_RIGHT, now);
                    return UDP_REVERSE_V8_EVENT_SIMPLE_NUDGE_END;
                }
            } else {
                ReverseV8SetState(REV8_WAIT_CLEAR, now);
            }
            break;
    }
    return UDP_REVERSE_V8_EVENT_SIMPLE_HEARTBEAT;
}

static void ReverseV8ControlStep(WifiIotGpioValue left, WifiIotGpioValue right,
                                 uint32_t now)
{
    ReverseV8State previousState = g_reverseV8State;
    UdpReverseV8Event event = UDP_REVERSE_V8_EVENT_SIMPLE_HEARTBEAT;
    uint8_t changed = 0U;

    if (g_reverseV8Valid == 0U) {
        g_reverseV8Previous = g_stableState;
        g_reverseV8Current = g_stableState;
        g_reverseV8Valid = 1U;
        changed = 1U;
    } else {
        g_reverseV8Previous = g_reverseV8Current;
        g_reverseV8Current = g_stableState;
        changed = (g_reverseV8Previous != g_reverseV8Current);
    }

    if (g_reverseV8State == REV8_WAIT_CLEAR) {
        ReverseV8Apply(REV8_WAIT_CLEAR);
        if (g_stableState != 3U) {
            if (g_reverseV8ClearSince == 0U) {
                g_reverseV8ClearSince = now;
            } else if ((uint32_t)(now - g_reverseV8ClearSince) >=
                       AppMsToTicks(REV4_ARM_CLEAR_MS)) {
                g_reverseV8ClearSince = 0U;
                /* Arming establishes the current side as a fresh entry. */
                event = ReverseV8SelectSimpleAction(now, 1U);
            }
        } else {
            g_reverseV8ClearSince = 0U;
        }
    } else if (g_reverseV8State != REV8_STOPPED) {
        /* Keep the reverse search alive while its stable input remains 00. */
        if (!(g_stableState == 0U &&
              (g_reverseV8State == REV8_SIMPLE_SEARCH_BACK_LEFT ||
               g_reverseV8State == REV8_SIMPLE_SEARCH_BACK_RIGHT))) {
            event = ReverseV8SelectSimpleAction(now, changed);
        }

        if (g_stableState == 0U &&
            (g_reverseV8State == REV8_SIMPLE_TRANSIT_LEFT_TO_RIGHT ||
             g_reverseV8State == REV8_SIMPLE_TRANSIT_RIGHT_TO_LEFT) &&
            (uint32_t)(now - g_reverseV8Start) >=
            AppMsToTicks(V8_SIMPLE_TRANSIT_MAX_MS)) {
            ReverseV8SetState((g_reverseV8EdgeSide == 2U) ?
                              REV8_SIMPLE_SEARCH_BACK_LEFT :
                              REV8_SIMPLE_SEARCH_BACK_RIGHT, now);
            event = UDP_REVERSE_V8_EVENT_SIMPLE_SEARCH_BACK_START;
        } else if (g_stableState == 0U &&
                   (g_reverseV8State == REV8_SIMPLE_SEARCH_BACK_LEFT ||
                    g_reverseV8State == REV8_SIMPLE_SEARCH_BACK_RIGHT) &&
                   (uint32_t)(now - g_reverseV8Start) >=
                   AppMsToTicks(V8_SIMPLE_SEARCH_BACK_MS)) {
            ReverseV8SetState(REV8_STOPPED, now);
            event = UDP_REVERSE_V8_EVENT_SIMPLE_LINE_LOST;
        } else if ((g_stableState == 1U || g_stableState == 2U) &&
                   (previousState == REV8_SIMPLE_SEARCH_BACK_LEFT ||
                    previousState == REV8_SIMPLE_SEARCH_BACK_RIGHT)) {
            event = UDP_REVERSE_V8_EVENT_SIMPLE_EDGE_REACQUIRED;
        } else if (event == UDP_REVERSE_V8_EVENT_SIMPLE_HEARTBEAT &&
                   (changed != 0U || previousState != g_reverseV8State)) {
            event = UDP_REVERSE_V8_EVENT_SIMPLE_STATE_CHANGE;
        }
    }

    if (g_reverseV8Banner == 0) {
        g_reverseV8Banner = 1;
        printf("=== ACTIVE CAR MODE: REVERSE V8.2 SIMPLE SERVO ===\r\n");
    }
    if (event == UDP_REVERSE_V8_EVENT_SIMPLE_HEARTBEAT &&
        (changed != 0U || previousState != g_reverseV8State)) {
        event = UDP_REVERSE_V8_EVENT_SIMPLE_STATE_CHANGE;
    }
    if (changed != 0U || previousState != g_reverseV8State ||
        event != UDP_REVERSE_V8_EVENT_SIMPLE_HEARTBEAT) {
        ReverseV8Publish(event, now, left, right);
    } else if ((uint32_t)(now - g_reverseV8LastTelemetry) >=
               AppMsToTicks(V8_DEBUG_HEARTBEAT_MS)) {
        ReverseV8Publish(UDP_REVERSE_V8_EVENT_SIMPLE_HEARTBEAT, now, left, right);
    }
}
#endif

#if (TRACE_REVERSE_V8_TEST_MODE == 1) && (TRACE_REVERSE_V8_PAIRED_RECENTER_MODE == 1)
/*
 * V8.4 paired-turn recenter experiment.  A stable side edge starts one signed
 * symmetric manoeuvre; 00 observed during that manoeuvre is a trusted center
 * candidate, followed by an equal-time opposite yaw to restore heading.
 */
typedef enum {
    REV8_PAIR_WAIT_CLEAR = 0,
    REV8_PAIR_INITIAL_STRAIGHT,
    REV8_PAIR_SHIFT_FROM_LEFT,
    REV8_PAIR_SHIFT_FROM_RIGHT,
    REV8_PAIR_RESTORE_FROM_LEFT,
    REV8_PAIR_RESTORE_FROM_RIGHT,
    REV8_PAIR_TRUSTED_STRAIGHT,
    REV8_PAIR_ABORT,
    REV8_PAIR_STOPPED
} ReverseV8State;

static ReverseV8State g_reverseV8State = REV8_PAIR_STOPPED;
static uint8_t g_reverseV8Valid, g_reverseV8Current, g_reverseV8Previous;
static uint8_t g_reverseV8EdgeSide, g_reverseV8CenterConfidence;
static uint8_t g_reverseV8RestoreStartPending;
static uint32_t g_reverseV8Start, g_reverseV8ClearSince, g_reverseV8LastTelemetry;
static uint32_t g_reverseV8ShiftDurationMs, g_reverseV8RestoreTargetMs;
static int g_reverseV8Banner;

#define V8_BASE_SPEED                  100U
#define V8_TURN_DELTA                    5U
#define V8_PAIR_SHIFT_MAX_MS           600U
#define V8_PAIR_RESTORE_MAX_MS         600U

static void ReverseV8Apply(ReverseV8State state)
{
    int leftCommand = 0;
    int rightCommand = 0;

    switch (state) {
        case REV8_PAIR_INITIAL_STRAIGHT:
        case REV8_PAIR_TRUSTED_STRAIGHT:
        case REV8_PAIR_ABORT:
            leftCommand = rightCommand = -(int)V8_BASE_SPEED;
            break;
        case REV8_PAIR_SHIFT_FROM_LEFT:
            /* Left probe is black: slow it, boost the right wheel, yaw RIGHT. */
            leftCommand = -((int)V8_BASE_SPEED - (int)V8_TURN_DELTA);
            rightCommand = -((int)V8_BASE_SPEED + (int)V8_TURN_DELTA);
            break;
        case REV8_PAIR_SHIFT_FROM_RIGHT:
            /* Right probe is black: boost the left wheel, slow the right, yaw LEFT. */
            leftCommand = -((int)V8_BASE_SPEED + (int)V8_TURN_DELTA);
            rightCommand = -((int)V8_BASE_SPEED - (int)V8_TURN_DELTA);
            break;
        case REV8_PAIR_RESTORE_FROM_LEFT:
            /* Exact differential reversal of SHIFT_FROM_LEFT: yaw LEFT. */
            leftCommand = -((int)V8_BASE_SPEED + (int)V8_TURN_DELTA);
            rightCommand = -((int)V8_BASE_SPEED - (int)V8_TURN_DELTA);
            break;
        case REV8_PAIR_RESTORE_FROM_RIGHT:
            /* Exact differential reversal of SHIFT_FROM_RIGHT: yaw RIGHT. */
            leftCommand = -((int)V8_BASE_SPEED - (int)V8_TURN_DELTA);
            rightCommand = -((int)V8_BASE_SPEED + (int)V8_TURN_DELTA);
            break;
        case REV8_PAIR_WAIT_CLEAR:
        case REV8_PAIR_STOPPED:
        default:
            leftCommand = 0;
            rightCommand = 0;
            break;
    }

    if (g_motorCommandValid == 0U || g_motorLeftCommand != leftCommand ||
        g_motorRightCommand != rightCommand) {
        TraceSendMotorCommand(leftCommand, rightCommand, 0);
    }
    g_reverseV8State = state;
}

static void ReverseV8SetState(ReverseV8State state, uint32_t now)
{
    if (g_reverseV8State != state) {
        g_reverseV8Start = now;
    }
    ReverseV8Apply(state);
}

static uint32_t ReverseV8PairElapsed(uint32_t now)
{
    return AppTicksToMs((uint32_t)(now - g_reverseV8Start));
}

static void ReverseV8Publish(UdpReverseV8Event event, uint32_t now,
                             WifiIotGpioValue left, WifiIotGpioValue right)
{
    UdpReverseV4TelemetryState telemetry = {0};
    uint32_t phaseElapsedMs = ReverseV8PairElapsed(now);

    telemetry.event = (UdpReverseV4Event)event;
    telemetry.state = (UdpReverseV4State)(440 + g_reverseV8State);
    telemetry.sensorState = g_stableState;
    telemetry.previousSensorState = g_reverseV8Previous;
    telemetry.rawLeft = (left == WIFI_IOT_GPIO_VALUE1);
    telemetry.rawRight = (right == WIFI_IOT_GPIO_VALUE1);
    telemetry.triggerSensorState = g_reverseV8EdgeSide;
    telemetry.edgeSide = g_reverseV8EdgeSide;
    telemetry.centerConfidence = g_reverseV8CenterConfidence;
    telemetry.leftCommand = g_motorLeftCommand;
    telemetry.rightCommand = g_motorRightCommand;
    telemetry.pulseElapsedMs = phaseElapsedMs;
    telemetry.shiftDurationMs = g_reverseV8ShiftDurationMs;
    telemetry.restoreTargetMs = g_reverseV8RestoreTargetMs;
    if (g_reverseV8State == REV8_PAIR_SHIFT_FROM_LEFT ||
        g_reverseV8State == REV8_PAIR_SHIFT_FROM_RIGHT) {
        telemetry.shiftElapsedMs = phaseElapsedMs;
        telemetry.targetMs = V8_PAIR_SHIFT_MAX_MS;
    } else if (g_reverseV8State == REV8_PAIR_RESTORE_FROM_LEFT ||
               g_reverseV8State == REV8_PAIR_RESTORE_FROM_RIGHT) {
        telemetry.restoreElapsedMs = phaseElapsedMs;
        telemetry.targetMs = g_reverseV8RestoreTargetMs;
    }
    UdpTelemetryUpdateReverseV4(&telemetry);
    g_reverseV8LastTelemetry = now;
}

static void ReverseV8StartShift(uint8_t originEdge, uint32_t now)
{
    g_reverseV8EdgeSide = originEdge;
    g_reverseV8CenterConfidence = 0U;
    g_reverseV8ShiftDurationMs = 0U;
    g_reverseV8RestoreTargetMs = 0U;
    ReverseV8SetState((originEdge == 2U) ? REV8_PAIR_SHIFT_FROM_LEFT :
                      REV8_PAIR_SHIFT_FROM_RIGHT, now);
}

static void ReverseV8StartRestore(uint8_t originEdge, uint32_t now,
                                  uint32_t shiftDurationMs)
{
    g_reverseV8EdgeSide = originEdge;
    g_reverseV8ShiftDurationMs = shiftDurationMs;
    g_reverseV8RestoreTargetMs = (shiftDurationMs > V8_PAIR_RESTORE_MAX_MS) ?
                                  V8_PAIR_RESTORE_MAX_MS : shiftDurationMs;
    g_reverseV8RestoreStartPending = 1U;
    ReverseV8SetState((originEdge == 2U) ? REV8_PAIR_RESTORE_FROM_LEFT :
                      REV8_PAIR_RESTORE_FROM_RIGHT, now);
}

static void ReverseV8Reset(uint32_t now)
{
    g_reverseV8State = REV8_PAIR_WAIT_CLEAR;
    g_reverseV8Valid = 0U;
    g_reverseV8Current = 0U;
    g_reverseV8Previous = 0U;
    g_reverseV8EdgeSide = 0U;
    g_reverseV8CenterConfidence = 0U;
    g_reverseV8RestoreStartPending = 0U;
    g_reverseV8ClearSince = 0U;
    g_reverseV8ShiftDurationMs = 0U;
    g_reverseV8RestoreTargetMs = 0U;
    g_reverseV8Start = now;
    g_reverseV8LastTelemetry = now;
    g_reverseV8Banner = 0;
}

static UdpReverseV8Event ReverseV8PairSelect(uint32_t now)
{
    uint32_t elapsedMs = ReverseV8PairElapsed(now);

    switch (g_reverseV8State) {
        case REV8_PAIR_INITIAL_STRAIGHT:
        case REV8_PAIR_TRUSTED_STRAIGHT:
            if (g_stableState == 2U) {
                ReverseV8StartShift(2U, now);
                return UDP_REVERSE_V8_EVENT_PAIR_SHIFT_START;
            }
            if (g_stableState == 1U) {
                ReverseV8StartShift(1U, now);
                return UDP_REVERSE_V8_EVENT_PAIR_SHIFT_START;
            }
            if (g_stableState == 3U && g_reverseV8EdgeSide != 0U) {
                /* A known-side 11 receives the matching reverse paired turn. */
                ReverseV8StartRestore(g_reverseV8EdgeSide, now,
                                      (g_reverseV8ShiftDurationMs != 0U) ?
                                      g_reverseV8ShiftDurationMs : CAR_CONTROL_PERIOD_MS);
                return UDP_REVERSE_V8_EVENT_PAIR_HIT_11;
            }
            ReverseV8Apply(g_reverseV8State);
            break;

        case REV8_PAIR_SHIFT_FROM_LEFT:
        case REV8_PAIR_SHIFT_FROM_RIGHT:
            if (g_stableState == 0U || g_stableState == 3U) {
                uint8_t hit11 = (g_stableState == 3U);
                g_reverseV8CenterConfidence = (hit11 == 0U) ? 1U : 0U;
                ReverseV8StartRestore(g_reverseV8EdgeSide, now, elapsedMs);
                return (hit11 != 0U) ? UDP_REVERSE_V8_EVENT_PAIR_HIT_11 :
                                      UDP_REVERSE_V8_EVENT_PAIR_TRUSTED_00;
            }
            if (elapsedMs >= V8_PAIR_SHIFT_MAX_MS) {
                ReverseV8SetState(REV8_PAIR_ABORT, now);
                return UDP_REVERSE_V8_EVENT_PAIR_SHIFT_TIMEOUT;
            }
            ReverseV8Apply(g_reverseV8State);
            break;

        case REV8_PAIR_RESTORE_FROM_LEFT:
        case REV8_PAIR_RESTORE_FROM_RIGHT:
            if (g_reverseV8RestoreStartPending != 0U) {
                g_reverseV8RestoreStartPending = 0U;
                ReverseV8Apply(g_reverseV8State);
                return UDP_REVERSE_V8_EVENT_PAIR_RESTORE_START;
            }
            if (elapsedMs >= g_reverseV8RestoreTargetMs) {
                ReverseV8SetState(REV8_PAIR_TRUSTED_STRAIGHT, now);
                return UDP_REVERSE_V8_EVENT_PAIR_RESTORE_END;
            }
            ReverseV8Apply(g_reverseV8State);
            break;

        case REV8_PAIR_ABORT:
            /* Timeout is visible for one control period before the latched stop. */
            ReverseV8SetState(REV8_PAIR_STOPPED, now);
            break;

        case REV8_PAIR_WAIT_CLEAR:
        case REV8_PAIR_STOPPED:
        default:
            break;
    }
    return UDP_REVERSE_V8_EVENT_PAIR_HEARTBEAT;
}

static void ReverseV8ControlStep(WifiIotGpioValue left, WifiIotGpioValue right,
                                 uint32_t now)
{
    ReverseV8State previousState = g_reverseV8State;
    UdpReverseV8Event event = UDP_REVERSE_V8_EVENT_PAIR_HEARTBEAT;
    uint8_t changed;

    if (g_reverseV8Valid == 0U) {
        g_reverseV8Previous = g_stableState;
        g_reverseV8Current = g_stableState;
        g_reverseV8Valid = 1U;
        changed = 1U;
    } else {
        g_reverseV8Previous = g_reverseV8Current;
        g_reverseV8Current = g_stableState;
        changed = (g_reverseV8Previous != g_reverseV8Current);
    }

    if (g_reverseV8State == REV8_PAIR_WAIT_CLEAR) {
        ReverseV8Apply(REV8_PAIR_WAIT_CLEAR);
        if (g_stableState != 3U) {
            if (g_reverseV8ClearSince == 0U) {
                g_reverseV8ClearSince = now;
            } else if ((uint32_t)(now - g_reverseV8ClearSince) >=
                       AppMsToTicks(REV4_ARM_CLEAR_MS)) {
                g_reverseV8ClearSince = 0U;
                if (g_stableState == 2U || g_stableState == 1U) {
                    ReverseV8StartShift(g_stableState, now);
                    event = UDP_REVERSE_V8_EVENT_PAIR_SHIFT_START;
                } else {
                    ReverseV8SetState(REV8_PAIR_INITIAL_STRAIGHT, now);
                    event = UDP_REVERSE_V8_EVENT_ARMED;
                }
            }
        } else {
            g_reverseV8ClearSince = 0U;
        }
    } else if (g_reverseV8State != REV8_PAIR_STOPPED) {
        event = ReverseV8PairSelect(now);
    } else {
        ReverseV8Apply(REV8_PAIR_STOPPED);
    }

    if (g_reverseV8Banner == 0) {
        g_reverseV8Banner = 1;
        printf("=== ACTIVE CAR MODE: REVERSE V8.4 PAIRED RECENTER ===\r\n");
    }
    if (changed != 0U || previousState != g_reverseV8State ||
        event != UDP_REVERSE_V8_EVENT_PAIR_HEARTBEAT) {
        ReverseV8Publish(event, now, left, right);
    } else if ((uint32_t)(now - g_reverseV8LastTelemetry) >=
               AppMsToTicks(V8_DEBUG_HEARTBEAT_MS)) {
        ReverseV8Publish(UDP_REVERSE_V8_EVENT_PAIR_HEARTBEAT, now, left, right);
    }
}
#endif

#if (TRACE_REVERSE_V8_TEST_MODE == 1) && (TRACE_REVERSE_V8_MICRO_PAIRED_MODE == 1) && \
    (TRACE_REVERSE_V8_BIASED_MICRO_MODE == 0)
/* V8.5: repeated two-step S manoeuvres; stable GPIO remains the only control input. */
typedef enum {
    REV8_MICRO_WAIT_CLEAR = 0,
    REV8_MICRO_STRAIGHT,
    REV8_MICRO_SHIFT_RIGHT,
    REV8_MICRO_RESTORE_LEFT,
    REV8_MICRO_SHIFT_LEFT,
    REV8_MICRO_RESTORE_RIGHT,
    REV8_MICRO_SETTLE,
    REV8_MICRO_STOPPED
} ReverseV8State;

static ReverseV8State g_reverseV8State = REV8_MICRO_STOPPED;
static uint8_t g_reverseV8Valid, g_reverseV8Current, g_reverseV8Previous;
static uint8_t g_reverseV8EdgeSide, g_reverseV8OriginSensor;
static uint8_t g_reverseV8RestoreStartPending, g_reverseV8SettleStartPending;
static uint32_t g_reverseV8Start, g_reverseV8ClearSince, g_reverseV8LastTelemetry;
static uint32_t g_reverseV8ShiftActualMs, g_reverseV8RestoreTargetMs;
static uint16_t g_reverseV8PairCount;
static int g_reverseV8Banner;

#define V8_REVERSE_BASE                95U
#define V8_REVERSE_SLOW                90U
#define V8_REVERSE_FAST               100U
#define V8_MICRO_SHIFT_MAX_MS          60U
#define V8_MICRO_SETTLE_MS             60U

static void ReverseV8Apply(ReverseV8State state)
{
    int leftCommand = 0;
    int rightCommand = 0;

    switch (state) {
        case REV8_MICRO_STRAIGHT:
        case REV8_MICRO_SETTLE:
            leftCommand = rightCommand = -(int)V8_REVERSE_BASE;
            break;
        case REV8_MICRO_SHIFT_RIGHT:
        case REV8_MICRO_RESTORE_RIGHT:
            leftCommand = -(int)V8_REVERSE_SLOW;
            rightCommand = -(int)V8_REVERSE_FAST;
            break;
        case REV8_MICRO_SHIFT_LEFT:
        case REV8_MICRO_RESTORE_LEFT:
            leftCommand = -(int)V8_REVERSE_FAST;
            rightCommand = -(int)V8_REVERSE_SLOW;
            break;
        case REV8_MICRO_WAIT_CLEAR:
        case REV8_MICRO_STOPPED:
        default:
            leftCommand = 0;
            rightCommand = 0;
            break;
    }

    if (g_motorCommandValid == 0U || g_motorLeftCommand != leftCommand ||
        g_motorRightCommand != rightCommand) {
        TraceSendMotorCommand(leftCommand, rightCommand, 0);
    }
    g_reverseV8State = state;
}

static void ReverseV8SetState(ReverseV8State state, uint32_t now)
{
    if (g_reverseV8State != state) {
        g_reverseV8Start = now;
    }
    ReverseV8Apply(state);
}

static uint32_t ReverseV8MicroElapsed(uint32_t now)
{
    return AppTicksToMs((uint32_t)(now - g_reverseV8Start));
}

static int ReverseV8MicroIsShift(ReverseV8State state)
{
    return state == REV8_MICRO_SHIFT_RIGHT || state == REV8_MICRO_SHIFT_LEFT;
}

static int ReverseV8MicroIsRestore(ReverseV8State state)
{
    return state == REV8_MICRO_RESTORE_LEFT || state == REV8_MICRO_RESTORE_RIGHT;
}

static void ReverseV8Publish(UdpReverseV8Event event, uint32_t now,
                             WifiIotGpioValue left, WifiIotGpioValue right)
{
    UdpReverseV4TelemetryState telemetry = {0};
    uint32_t phaseElapsedMs = ReverseV8MicroElapsed(now);

    telemetry.event = (UdpReverseV4Event)event;
    telemetry.state = (UdpReverseV4State)(460 + g_reverseV8State);
    telemetry.sensorState = g_stableState;
    telemetry.previousSensorState = g_reverseV8Previous;
    telemetry.rawLeft = (left == WIFI_IOT_GPIO_VALUE1);
    telemetry.rawRight = (right == WIFI_IOT_GPIO_VALUE1);
    telemetry.triggerSensorState = g_reverseV8OriginSensor;
    telemetry.edgeSide = g_reverseV8EdgeSide;
    telemetry.leftCommand = g_motorLeftCommand;
    telemetry.rightCommand = g_motorRightCommand;
    telemetry.sameSidePulses = g_reverseV8PairCount;
    telemetry.pulseElapsedMs = phaseElapsedMs;
    telemetry.shiftDurationMs = g_reverseV8ShiftActualMs;
    telemetry.restoreTargetMs = g_reverseV8RestoreTargetMs;
    if (ReverseV8MicroIsShift(g_reverseV8State) != 0) {
        telemetry.shiftElapsedMs = phaseElapsedMs;
        telemetry.targetMs = V8_MICRO_SHIFT_MAX_MS;
    } else if (ReverseV8MicroIsRestore(g_reverseV8State) != 0) {
        telemetry.restoreElapsedMs = phaseElapsedMs;
        telemetry.targetMs = g_reverseV8RestoreTargetMs;
    } else if (g_reverseV8State == REV8_MICRO_SETTLE) {
        telemetry.targetMs = V8_MICRO_SETTLE_MS;
    }
    UdpTelemetryUpdateReverseV4(&telemetry);
    g_reverseV8LastTelemetry = now;
}

static void ReverseV8StartMicroShift(uint8_t yawRight, uint8_t originSensor,
                                     uint32_t now)
{
    g_reverseV8OriginSensor = originSensor;
    if (originSensor == 2U) {
        g_reverseV8EdgeSide = 2U;
    } else if (originSensor == 1U) {
        g_reverseV8EdgeSide = 1U;
    }
    g_reverseV8ShiftActualMs = 0U;
    g_reverseV8RestoreTargetMs = 0U;
    g_reverseV8RestoreStartPending = 0U;
    ReverseV8SetState((yawRight != 0U) ? REV8_MICRO_SHIFT_RIGHT :
                      REV8_MICRO_SHIFT_LEFT, now);
}

static void ReverseV8StartMicroRestore(uint32_t now, uint32_t actualShiftMs)
{
    g_reverseV8ShiftActualMs = actualShiftMs;
    g_reverseV8RestoreTargetMs = actualShiftMs;
    g_reverseV8RestoreStartPending = 1U;
    ReverseV8SetState((g_reverseV8State == REV8_MICRO_SHIFT_RIGHT) ?
                      REV8_MICRO_RESTORE_LEFT : REV8_MICRO_RESTORE_RIGHT, now);
}

static void ReverseV8StartMicroSettle(uint32_t now)
{
    g_reverseV8SettleStartPending = 1U;
    ReverseV8SetState(REV8_MICRO_SETTLE, now);
}

static void ReverseV8Reset(uint32_t now)
{
    g_reverseV8State = REV8_MICRO_WAIT_CLEAR;
    g_reverseV8Valid = 0U;
    g_reverseV8Current = 0U;
    g_reverseV8Previous = 0U;
    g_reverseV8EdgeSide = 0U;
    g_reverseV8OriginSensor = 0U;
    g_reverseV8RestoreStartPending = 0U;
    g_reverseV8SettleStartPending = 0U;
    g_reverseV8ClearSince = 0U;
    g_reverseV8ShiftActualMs = 0U;
    g_reverseV8RestoreTargetMs = 0U;
    g_reverseV8PairCount = 0U;
    g_reverseV8Start = now;
    g_reverseV8LastTelemetry = now;
    g_reverseV8Banner = 0;
}

static UdpReverseV8Event ReverseV8MicroReevaluate(uint32_t now, uint8_t repeat)
{
    if (g_stableState == 2U) {
        ReverseV8StartMicroShift(1U, 2U, now);
        return (repeat != 0U) ? UDP_REVERSE_V8_EVENT_MICRO_PAIR_REPEAT :
                               UDP_REVERSE_V8_EVENT_MICRO_SHIFT_START;
    }
    if (g_stableState == 1U) {
        ReverseV8StartMicroShift(0U, 1U, now);
        return (repeat != 0U) ? UDP_REVERSE_V8_EVENT_MICRO_PAIR_REPEAT :
                               UDP_REVERSE_V8_EVENT_MICRO_SHIFT_START;
    }
    if (g_stableState == 3U && g_reverseV8EdgeSide != 0U) {
        /* 11 rides off with a pair opposite the most recent edge. */
        ReverseV8StartMicroShift((g_reverseV8EdgeSide == 1U) ? 1U : 0U, 3U, now);
        return (repeat != 0U) ? UDP_REVERSE_V8_EVENT_MICRO_PAIR_REPEAT :
                               UDP_REVERSE_V8_EVENT_MICRO_SHIFT_START;
    }
    ReverseV8SetState(REV8_MICRO_STRAIGHT, now);
    return (repeat != 0U) ? UDP_REVERSE_V8_EVENT_MICRO_SETTLE_END :
                           UDP_REVERSE_V8_EVENT_MICRO_HEARTBEAT;
}

static UdpReverseV8Event ReverseV8MicroSelect(uint32_t now)
{
    uint32_t elapsedMs = ReverseV8MicroElapsed(now);

    if (ReverseV8MicroIsShift(g_reverseV8State) != 0) {
        if (g_reverseV8OriginSensor != 3U && g_stableState == 0U) {
            ReverseV8StartMicroRestore(now, elapsedMs);
            return UDP_REVERSE_V8_EVENT_MICRO_SHIFT_END;
        }
        if (elapsedMs >= V8_MICRO_SHIFT_MAX_MS) {
            ReverseV8StartMicroRestore(now, V8_MICRO_SHIFT_MAX_MS);
            return UDP_REVERSE_V8_EVENT_MICRO_SHIFT_END;
        }
        ReverseV8Apply(g_reverseV8State);
        return UDP_REVERSE_V8_EVENT_MICRO_HEARTBEAT;
    }

    if (ReverseV8MicroIsRestore(g_reverseV8State) != 0) {
        if (g_reverseV8RestoreStartPending != 0U) {
            g_reverseV8RestoreStartPending = 0U;
            ReverseV8Apply(g_reverseV8State);
            return UDP_REVERSE_V8_EVENT_MICRO_RESTORE_START;
        }
        if (elapsedMs >= g_reverseV8RestoreTargetMs) {
            ReverseV8StartMicroSettle(now);
            return UDP_REVERSE_V8_EVENT_MICRO_RESTORE_END;
        }
        ReverseV8Apply(g_reverseV8State);
        return UDP_REVERSE_V8_EVENT_MICRO_HEARTBEAT;
    }

    if (g_reverseV8State == REV8_MICRO_SETTLE) {
        if (g_reverseV8SettleStartPending != 0U) {
            g_reverseV8SettleStartPending = 0U;
            ReverseV8Apply(REV8_MICRO_SETTLE);
            return UDP_REVERSE_V8_EVENT_MICRO_SETTLE_START;
        }
        if (elapsedMs >= V8_MICRO_SETTLE_MS) {
            g_reverseV8PairCount++;
            return ReverseV8MicroReevaluate(now, 1U);
        }
        ReverseV8Apply(REV8_MICRO_SETTLE);
        return UDP_REVERSE_V8_EVENT_MICRO_HEARTBEAT;
    }

    return ReverseV8MicroReevaluate(now, 0U);
}

static void ReverseV8ControlStep(WifiIotGpioValue left, WifiIotGpioValue right,
                                 uint32_t now)
{
    ReverseV8State previousState = g_reverseV8State;
    UdpReverseV8Event event = UDP_REVERSE_V8_EVENT_MICRO_HEARTBEAT;
    uint8_t changed;

    if (g_reverseV8Valid == 0U) {
        g_reverseV8Previous = g_stableState;
        g_reverseV8Current = g_stableState;
        g_reverseV8Valid = 1U;
        changed = 1U;
    } else {
        g_reverseV8Previous = g_reverseV8Current;
        g_reverseV8Current = g_stableState;
        changed = (g_reverseV8Previous != g_reverseV8Current);
    }

    if (g_reverseV8State == REV8_MICRO_WAIT_CLEAR) {
        ReverseV8Apply(REV8_MICRO_WAIT_CLEAR);
        if (g_stableState != 3U) {
            if (g_reverseV8ClearSince == 0U) {
                g_reverseV8ClearSince = now;
            } else if ((uint32_t)(now - g_reverseV8ClearSince) >=
                       AppMsToTicks(REV4_ARM_CLEAR_MS)) {
                g_reverseV8ClearSince = 0U;
                event = ReverseV8MicroReevaluate(now, 0U);
            }
        } else {
            g_reverseV8ClearSince = 0U;
        }
    } else if (g_reverseV8State != REV8_MICRO_STOPPED) {
        event = ReverseV8MicroSelect(now);
    } else {
        ReverseV8Apply(REV8_MICRO_STOPPED);
    }

    if (g_reverseV8Banner == 0) {
        g_reverseV8Banner = 1;
        printf("=== ACTIVE CAR MODE: REVERSE V8.5 MICRO PAIRED SERVO ===\r\n");
    }
    if (changed != 0U || previousState != g_reverseV8State ||
        event != UDP_REVERSE_V8_EVENT_MICRO_HEARTBEAT) {
        ReverseV8Publish(event, now, left, right);
    } else if ((uint32_t)(now - g_reverseV8LastTelemetry) >=
               AppMsToTicks(V8_DEBUG_HEARTBEAT_MS)) {
        ReverseV8Publish(UDP_REVERSE_V8_EVENT_MICRO_HEARTBEAT, now, left, right);
    }
}
#endif

#if (TRACE_REVERSE_V8_TEST_MODE == 1) && (TRACE_REVERSE_V8_BIASED_MICRO_MODE == 1)
/* V8.6: low-gain biased micro cycles; all decisions use the debounced state. */
typedef enum {
    REV8_BIASED_WAIT_CLEAR = 0, REV8_BIASED_STRAIGHT,
    REV8_BIASED_SHIFT_RIGHT, REV8_BIASED_RESTORE_LEFT,
    REV8_BIASED_SHIFT_LEFT, REV8_BIASED_RESTORE_RIGHT,
    REV8_BIASED_SETTLE, REV8_ESCAPE_11_RIGHT, REV8_ESCAPE_11_LEFT,
    REV8_ESCAPE_11_STRAIGHT, REV8_BIASED_STOPPED
} ReverseV8State;
static ReverseV8State g_reverseV8State = REV8_BIASED_STOPPED;
static uint8_t g_reverseV8Valid, g_reverseV8Current, g_reverseV8Previous;
static uint8_t g_reverseV8EdgeSide, g_reverseV8OriginSensor;
static uint8_t g_reverseV8RestoreStartPending, g_reverseV8SettleStartPending;
static uint32_t g_reverseV8Start, g_reverseV8ClearSince, g_reverseV8LastTelemetry;
static uint32_t g_reverseV8ShiftActualMs, g_reverseV8RestoreTargetMs;
static uint16_t g_reverseV8PairCount, g_reverseV8EscapeCount;
static int g_reverseV8Banner;
#define V8_REVERSE_BASE 95U
#define V8_REVERSE_SLOW 90U
#define V8_REVERSE_FAST 100U
#define V8_MICRO_SHIFT_MS 60U
#define V8_MICRO_RESTORE_MS 30U
#define V8_MICRO_SETTLE_MS 60U
#define V8_ESCAPE_PULSE_MS 30U

static void ReverseV8Apply(ReverseV8State s)
{ int l=0,r=0; switch(s) {
case REV8_BIASED_STRAIGHT: case REV8_BIASED_SETTLE: case REV8_ESCAPE_11_STRAIGHT:
    l=r=-(int)V8_REVERSE_BASE; break;
case REV8_BIASED_SHIFT_RIGHT: case REV8_BIASED_RESTORE_RIGHT: case REV8_ESCAPE_11_RIGHT:
    l=-(int)V8_REVERSE_SLOW; r=-(int)V8_REVERSE_FAST; break;
case REV8_BIASED_SHIFT_LEFT: case REV8_BIASED_RESTORE_LEFT: case REV8_ESCAPE_11_LEFT:
    l=-(int)V8_REVERSE_FAST; r=-(int)V8_REVERSE_SLOW; break;
case REV8_BIASED_WAIT_CLEAR: case REV8_BIASED_STOPPED: default: l=r=0; break;
} if (g_motorCommandValid==0U || g_motorLeftCommand!=l || g_motorRightCommand!=r) {
    TraceSendMotorCommand(l,r,0);
} g_reverseV8State=s; }
static void ReverseV8SetState(ReverseV8State s,uint32_t n)
{ if(g_reverseV8State!=s) g_reverseV8Start=n; ReverseV8Apply(s); }
static uint32_t ReverseV8Elapsed(uint32_t n){return AppTicksToMs((uint32_t)(n-g_reverseV8Start));}
static int ReverseV8IsShift(ReverseV8State s){return s==REV8_BIASED_SHIFT_RIGHT||s==REV8_BIASED_SHIFT_LEFT;}
static int ReverseV8IsRestore(ReverseV8State s){return s==REV8_BIASED_RESTORE_LEFT||s==REV8_BIASED_RESTORE_RIGHT;}
static void ReverseV8Publish(UdpReverseV8Event e,uint32_t n,WifiIotGpioValue l,WifiIotGpioValue r)
{ UdpReverseV4TelemetryState t={0}; uint32_t p=ReverseV8Elapsed(n); t.event=(UdpReverseV4Event)e; t.state=(UdpReverseV4State)(480+g_reverseV8State);
t.sensorState=g_stableState;t.previousSensorState=g_reverseV8Previous;t.rawLeft=(l==WIFI_IOT_GPIO_VALUE1);t.rawRight=(r==WIFI_IOT_GPIO_VALUE1);t.triggerSensorState=g_reverseV8OriginSensor;t.edgeSide=g_reverseV8EdgeSide;t.leftCommand=g_motorLeftCommand;t.rightCommand=g_motorRightCommand;t.sameSidePulses=g_reverseV8PairCount;t.probeElapsedMs=g_reverseV8EscapeCount;t.pulseElapsedMs=p;t.shiftDurationMs=g_reverseV8ShiftActualMs;t.restoreTargetMs=g_reverseV8RestoreTargetMs;
if(ReverseV8IsShift(g_reverseV8State)){t.shiftElapsedMs=p;t.targetMs=V8_MICRO_SHIFT_MS;} else if(ReverseV8IsRestore(g_reverseV8State)){t.restoreElapsedMs=p;t.targetMs=g_reverseV8RestoreTargetMs;} else if(g_reverseV8State==REV8_BIASED_SETTLE||g_reverseV8State==REV8_ESCAPE_11_STRAIGHT)t.targetMs=V8_MICRO_SETTLE_MS; else if(g_reverseV8State==REV8_ESCAPE_11_RIGHT||g_reverseV8State==REV8_ESCAPE_11_LEFT)t.targetMs=V8_ESCAPE_PULSE_MS; UdpTelemetryUpdateReverseV4(&t);g_reverseV8LastTelemetry=n; }
static void ReverseV8StartShift(uint8_t right,uint8_t origin,uint32_t n)
{g_reverseV8OriginSensor=origin;if(origin==2U)g_reverseV8EdgeSide=2U;else if(origin==1U)g_reverseV8EdgeSide=1U;g_reverseV8ShiftActualMs=0;g_reverseV8RestoreTargetMs=0;ReverseV8SetState(right?REV8_BIASED_SHIFT_RIGHT:REV8_BIASED_SHIFT_LEFT,n);}
static void ReverseV8StartRestore(uint32_t n,uint32_t actual)
{g_reverseV8ShiftActualMs=actual;g_reverseV8RestoreTargetMs=(actual<V8_MICRO_RESTORE_MS)?actual:V8_MICRO_RESTORE_MS;g_reverseV8RestoreStartPending=1;ReverseV8SetState(g_reverseV8State==REV8_BIASED_SHIFT_RIGHT?REV8_BIASED_RESTORE_LEFT:REV8_BIASED_RESTORE_RIGHT,n);}
static void ReverseV8StartSettle(uint32_t n){g_reverseV8SettleStartPending=1;ReverseV8SetState(REV8_BIASED_SETTLE,n);}
static void ReverseV8Reset(uint32_t n)
{g_reverseV8State=REV8_BIASED_WAIT_CLEAR;g_reverseV8Valid=0;g_reverseV8Current=g_reverseV8Previous=0;g_reverseV8EdgeSide=g_reverseV8OriginSensor=0;g_reverseV8RestoreStartPending=g_reverseV8SettleStartPending=0;g_reverseV8ClearSince=0;g_reverseV8ShiftActualMs=g_reverseV8RestoreTargetMs=0;g_reverseV8PairCount=g_reverseV8EscapeCount=0;g_reverseV8Start=g_reverseV8LastTelemetry=n;g_reverseV8Banner=0;}
static UdpReverseV8Event ReverseV8Reevaluate(uint32_t n,uint8_t repeat)
{if(g_stableState==2U){ReverseV8StartShift(1,2U,n);return repeat?UDP_REVERSE_V8_EVENT_BIASED_REPEAT:UDP_REVERSE_V8_EVENT_BIASED_10_SHIFT;}if(g_stableState==1U){ReverseV8StartShift(0,1U,n);return repeat?UDP_REVERSE_V8_EVENT_BIASED_REPEAT:UDP_REVERSE_V8_EVENT_BIASED_01_SHIFT;}ReverseV8SetState(REV8_BIASED_STRAIGHT,n);return(g_stableState==3U)?UDP_REVERSE_V8_EVENT_AMBIGUOUS_11:UDP_REVERSE_V8_EVENT_AMBIGUOUS_00;}
static UdpReverseV8Event ReverseV8BiasedSelect(uint32_t n)
{uint32_t e=ReverseV8Elapsed(n);
if(g_reverseV8State!=REV8_BIASED_STRAIGHT && g_reverseV8State!=REV8_BIASED_WAIT_CLEAR && g_stableState==2U && g_reverseV8OriginSensor!=2U){ReverseV8StartShift(1,2U,n);return UDP_REVERSE_V8_EVENT_SIGNED_SENSOR_OVERRIDE;}
if(g_reverseV8State!=REV8_BIASED_STRAIGHT && g_reverseV8State!=REV8_BIASED_WAIT_CLEAR && g_stableState==1U && g_reverseV8OriginSensor!=1U){ReverseV8StartShift(0,1U,n);return UDP_REVERSE_V8_EVENT_SIGNED_SENSOR_OVERRIDE;}
if(ReverseV8IsShift(g_reverseV8State)){if(g_stableState==0U){ReverseV8StartRestore(n,e);return UDP_REVERSE_V8_EVENT_BIASED_SHIFT_END;}if(e>=V8_MICRO_SHIFT_MS){ReverseV8StartRestore(n,V8_MICRO_SHIFT_MS);return UDP_REVERSE_V8_EVENT_BIASED_SHIFT_END;}ReverseV8Apply(g_reverseV8State);return UDP_REVERSE_V8_EVENT_BIASED_HEARTBEAT;}
if(ReverseV8IsRestore(g_reverseV8State)){if(g_reverseV8RestoreStartPending){g_reverseV8RestoreStartPending=0;ReverseV8Apply(g_reverseV8State);return(g_reverseV8OriginSensor==2U)?UDP_REVERSE_V8_EVENT_BIASED_10_RESTORE:UDP_REVERSE_V8_EVENT_BIASED_01_RESTORE;}if(e>=g_reverseV8RestoreTargetMs){ReverseV8StartSettle(n);return UDP_REVERSE_V8_EVENT_BIASED_SETTLE;}ReverseV8Apply(g_reverseV8State);return UDP_REVERSE_V8_EVENT_BIASED_HEARTBEAT;}
if(g_reverseV8State==REV8_BIASED_SETTLE){if(g_reverseV8SettleStartPending){g_reverseV8SettleStartPending=0;ReverseV8Apply(g_reverseV8State);return UDP_REVERSE_V8_EVENT_BIASED_SETTLE;}if(e>=V8_MICRO_SETTLE_MS){g_reverseV8PairCount++;return ReverseV8Reevaluate(n,1U);}ReverseV8Apply(g_reverseV8State);return UDP_REVERSE_V8_EVENT_BIASED_HEARTBEAT;}
return ReverseV8Reevaluate(n,0U);}
static void ReverseV8ControlStep(WifiIotGpioValue l,WifiIotGpioValue r,uint32_t n)
{ReverseV8State prev=g_reverseV8State;UdpReverseV8Event e=UDP_REVERSE_V8_EVENT_BIASED_HEARTBEAT;uint8_t ch;if(!g_reverseV8Valid){g_reverseV8Previous=g_stableState;g_reverseV8Current=g_stableState;g_reverseV8Valid=1;ch=1;}else{g_reverseV8Previous=g_reverseV8Current;g_reverseV8Current=g_stableState;ch=(g_reverseV8Previous!=g_reverseV8Current);}if(g_reverseV8State==REV8_BIASED_WAIT_CLEAR){ReverseV8Apply(REV8_BIASED_WAIT_CLEAR);if(g_stableState!=3U){if(!g_reverseV8ClearSince)g_reverseV8ClearSince=n;else if((uint32_t)(n-g_reverseV8ClearSince)>=AppMsToTicks(REV4_ARM_CLEAR_MS)){g_reverseV8ClearSince=0;e=ReverseV8Reevaluate(n,0U);}}else g_reverseV8ClearSince=0;}else if(g_reverseV8State!=REV8_BIASED_STOPPED)e=ReverseV8BiasedSelect(n);else ReverseV8Apply(REV8_BIASED_STOPPED);if(!g_reverseV8Banner){g_reverseV8Banner=1;printf("=== ACTIVE CAR MODE: REVERSE V8.6 BIASED MICRO SERVO ===\r\n");}if(ch||prev!=g_reverseV8State||e!=UDP_REVERSE_V8_EVENT_BIASED_HEARTBEAT)ReverseV8Publish(e,n,l,r);else if((uint32_t)(n-g_reverseV8LastTelemetry)>=AppMsToTicks(V8_DEBUG_HEARTBEAT_MS))ReverseV8Publish(UDP_REVERSE_V8_EVENT_BIASED_HEARTBEAT,n,l,r);}
#endif

#if (TRACE_SHARP_TURN_DIAG_MODE == 1)
typedef struct {
    uint32_t tick;
    uint8_t sensorState;
    UdpTelemetryAction action;
    int leftCommand;
    int rightCommand;
} TraceDiagHistorySample;

static TraceDiagHistorySample g_traceDiagHistory[TRACE_DIAG_HISTORY_SAMPLES];
static uint8_t g_traceDiagHistoryNext;
static uint8_t g_traceDiagHistoryCount;
static uint8_t g_traceDiagRawLeft;
static uint8_t g_traceDiagRawRight;
static uint8_t g_traceDiagCurrentSensor;
static uint8_t g_traceDiagPreviousSensor;
static uint8_t g_traceDiagLastNon00Sensor;
static uint8_t g_traceDiagLastNon11Sensor;
static uint8_t g_traceDiagSensorValid;
static TraceAction g_traceDiagLastAction;
static uint8_t g_traceDiagLastCorrection;
static uint32_t g_traceDiagActionChangeTick;
static uint32_t g_traceDiagSensorChangeTick;
static uint32_t g_traceDiagCorrectionTick;
static uint32_t g_traceDiagLast10Tick;
static uint32_t g_traceDiagLast01Tick;
static uint32_t g_traceDiagZeroStartTick;
static uint32_t g_traceDiagLastTelemetryTick;
static uint16_t g_traceDiagConsecutive00;
static uint16_t g_traceDiagConsecutive10;
static uint16_t g_traceDiagConsecutive01;
static uint16_t g_traceDiagConsecutive11;
static int g_traceDiagPossibleLostReported;

static void TraceDiagReset(uint32_t now)
{
    g_traceDiagHistoryNext = 0U;
    g_traceDiagHistoryCount = 0U;
    g_traceDiagRawLeft = 0U;
    g_traceDiagRawRight = 0U;
    g_traceDiagCurrentSensor = 0U;
    g_traceDiagPreviousSensor = 0U;
    g_traceDiagLastNon00Sensor = 0U;
    g_traceDiagLastNon11Sensor = 0U;
    g_traceDiagSensorValid = 0U;
    g_traceDiagLastAction = g_lastAction;
    g_traceDiagLastCorrection = (uint8_t)g_lastCorrection;
    g_traceDiagActionChangeTick = now;
    g_traceDiagSensorChangeTick = now;
    g_traceDiagCorrectionTick = now;
    g_traceDiagLast10Tick = now;
    g_traceDiagLast01Tick = now;
    g_traceDiagZeroStartTick = now;
    g_traceDiagLastTelemetryTick = now;
    g_traceDiagConsecutive00 = 0U;
    g_traceDiagConsecutive10 = 0U;
    g_traceDiagConsecutive01 = 0U;
    g_traceDiagConsecutive11 = 0U;
    g_traceDiagPossibleLostReported = 0;
}

static void TraceDiagHistoryAdd(uint32_t now)
{
    TraceDiagHistorySample *sample = &g_traceDiagHistory[g_traceDiagHistoryNext];

    sample->tick = now;
    sample->sensorState = g_stableState;
    sample->action = TraceToUdpTelemetryAction(g_lastAction);
    sample->leftCommand = g_motorLeftCommand;
    sample->rightCommand = g_motorRightCommand;
    g_traceDiagHistoryNext = (uint8_t)((g_traceDiagHistoryNext + 1U) %
                                       TRACE_DIAG_HISTORY_SAMPLES);
    if (g_traceDiagHistoryCount < TRACE_DIAG_HISTORY_SAMPLES) {
        g_traceDiagHistoryCount++;
    }
}

static void TraceDiagHistorySummary(uint16_t *count00, uint16_t *count01,
                                    uint16_t *count10, uint16_t *count11)
{
    uint8_t index;
    uint8_t count;

    *count00 = 0U;
    *count01 = 0U;
    *count10 = 0U;
    *count11 = 0U;
    for (count = 0U; count < g_traceDiagHistoryCount; count++) {
        index = (uint8_t)((g_traceDiagHistoryNext + TRACE_DIAG_HISTORY_SAMPLES -
                           g_traceDiagHistoryCount + count) % TRACE_DIAG_HISTORY_SAMPLES);
        switch (g_traceDiagHistory[index].sensorState) {
            case 0x00U:
                (*count00)++;
                break;
            case 0x01U:
                (*count01)++;
                break;
            case 0x02U:
                (*count10)++;
                break;
            default:
                (*count11)++;
                break;
        }
    }
}

static uint32_t TraceDiagAgeMs(uint32_t now, uint32_t then)
{
    return AppTicksToMs((uint32_t)(now - then));
}

static void TraceDiagPublish(UdpTraceDebugEvent event, uint32_t now)
{
    UdpTraceDebugState state;

    TraceDiagHistorySummary(&state.history00, &state.history01,
                            &state.history10, &state.history11);
    state.event = event;
    state.rawLeft = g_traceDiagRawLeft;
    state.rawRight = g_traceDiagRawRight;
    state.sensorState = g_stableState;
    state.previousSensorState = g_traceDiagPreviousSensor;
    state.lastNon00SensorState = g_traceDiagLastNon00Sensor;
    state.lastNon11SensorState = g_traceDiagLastNon11Sensor;
    state.action = TraceToUdpTelemetryAction(g_lastAction);
    state.lastAction = TraceToUdpTelemetryAction(g_traceDiagLastAction);
    state.lastCorrection = (uint8_t)g_lastCorrection;
    state.leftCommand = g_motorLeftCommand;
    state.rightCommand = g_motorRightCommand;
    state.actionAgeMs = TraceDiagAgeMs(now, g_traceDiagActionChangeTick);
    state.sensorAgeMs = TraceDiagAgeMs(now, g_traceDiagSensorChangeTick);
    state.correctionAgeMs = TraceDiagAgeMs(now, g_traceDiagCorrectionTick);
    state.last10AgeMs = TraceDiagAgeMs(now, g_traceDiagLast10Tick);
    state.last01AgeMs = TraceDiagAgeMs(now, g_traceDiagLast01Tick);
    state.consecutive00 = g_traceDiagConsecutive00;
    state.consecutive10 = g_traceDiagConsecutive10;
    state.consecutive01 = g_traceDiagConsecutive01;
    state.consecutive11 = g_traceDiagConsecutive11;
    state.sequence = 0U;
    UdpTelemetryUpdateTraceDebug(&state);
    g_traceDiagLastTelemetryTick = now;
}

static void TraceDiagPoll(WifiIotGpioValue rawLeft, WifiIotGpioValue rawRight,
                          int stableChanged, uint32_t now)
{
    uint8_t physicalLeft = (rawLeft == WIFI_IOT_GPIO_VALUE1) ? 1U : 0U;
    uint8_t physicalRight = (rawRight == WIFI_IOT_GPIO_VALUE1) ? 1U : 0U;
    int rawChanged = (physicalLeft != g_traceDiagRawLeft ||
                      physicalRight != g_traceDiagRawRight) ? 1 : 0;
    int actionChanged = (g_lastAction != g_traceDiagLastAction) ? 1 : 0;
    int correctionChanged = ((uint8_t)g_lastCorrection !=
                             g_traceDiagLastCorrection) ? 1 : 0;

    g_traceDiagRawLeft = physicalLeft;
    g_traceDiagRawRight = physicalRight;
    if (g_traceDiagSensorValid == 0 || stableChanged != 0) {
        if (g_traceDiagSensorValid != 0 && g_traceDiagCurrentSensor != g_stableState) {
            g_traceDiagPreviousSensor = g_traceDiagCurrentSensor;
            g_traceDiagCurrentSensor = g_stableState;
            g_traceDiagSensorChangeTick = now;
        } else if (g_traceDiagSensorValid == 0) {
            g_traceDiagCurrentSensor = g_stableState;
            g_traceDiagPreviousSensor = g_stableState;
        }
        g_traceDiagSensorValid = 1U;
    }

    if (g_stableState != 0x00U) {
        g_traceDiagLastNon00Sensor = g_stableState;
        g_traceDiagZeroStartTick = now;
        g_traceDiagPossibleLostReported = 0;
    }
    if (g_stableState != 0x03U) {
        g_traceDiagLastNon11Sensor = g_stableState;
    }
    if (g_stableState == 0x02U) {
        g_traceDiagLast10Tick = now;
    }
    if (g_stableState == 0x01U) {
        g_traceDiagLast01Tick = now;
    }

    g_traceDiagConsecutive00 = (g_stableState == 0x00U) ?
        (uint16_t)(g_traceDiagConsecutive00 + 1U) : 0U;
    g_traceDiagConsecutive10 = (g_stableState == 0x02U) ?
        (uint16_t)(g_traceDiagConsecutive10 + 1U) : 0U;
    g_traceDiagConsecutive01 = (g_stableState == 0x01U) ?
        (uint16_t)(g_traceDiagConsecutive01 + 1U) : 0U;
    g_traceDiagConsecutive11 = (g_stableState == 0x03U) ?
        (uint16_t)(g_traceDiagConsecutive11 + 1U) : 0U;

    if (actionChanged != 0) {
        g_traceDiagLastAction = g_lastAction;
        g_traceDiagActionChangeTick = now;
    }
    if (correctionChanged != 0) {
        g_traceDiagLastCorrection = (uint8_t)g_lastCorrection;
        g_traceDiagCorrectionTick = now;
    }

    TraceDiagHistoryAdd(now);
    if (g_stableState == 0x00U && g_traceDiagPossibleLostReported == 0 &&
        ((uint32_t)(now - g_traceDiagLast10Tick) <=
         AppMsToTicks(TRACE_DIAG_LOST_LOOKBACK_MS) ||
         (uint32_t)(now - g_traceDiagLast01Tick) <=
         AppMsToTicks(TRACE_DIAG_LOST_LOOKBACK_MS)) &&
        (uint32_t)(now - g_traceDiagZeroStartTick) >=
        AppMsToTicks(TRACE_DIAG_LOST_HOLD_MS)) {
        g_traceDiagPossibleLostReported = 1;
        TraceDiagPublish(UDP_TRACEDEBUG_EVENT_POSSIBLE_LOST_LINE, now);
        return;
    }

    if (rawChanged != 0 || stableChanged != 0 || actionChanged != 0) {
        TraceDiagPublish(UDP_TRACEDEBUG_EVENT_UPDATE, now);
    } else if ((uint32_t)(now - g_traceDiagLastTelemetryTick) >=
               AppMsToTicks(TRACE_DIAG_HEARTBEAT_MS)) {
        TraceDiagPublish(UDP_TRACEDEBUG_EVENT_HEARTBEAT, now);
    }
}
#endif

#if (TRACE_RACE_TEST_MODE == 1)
typedef enum {
    RACE_WAIT_START = 0,
    RACE_LEAVING_START,
    RACE_RUNNING,
    RACE_MARKER1,
    RACE_MARKER1_CLEAR,
    RACE_MARKER_PROBE,
    RACE_FINISH_CONFIRMED,
    RACE_FINISH_SLOW,
    RACE_FINISH_STOPPED,
    RACE_DEADEND_CONFIRMED,
    RACE_DEADEND_STOPPED
} TraceRaceState;

static TraceRaceState g_raceState;
static uint32_t g_raceStartTick;
static uint32_t g_raceMarkerTick;
static uint32_t g_raceMarker1ClearTick;
static uint32_t g_raceMarker2Tick;
static uint32_t g_racePhaseStartTick;
static uint32_t g_raceLastTelemetryTick;
static uint32_t g_raceLastDebugTick;
static uint8_t g_raceMarkerSamples;
static uint8_t g_raceClearSamples;
static uint8_t g_racePreviousSensor;
static uint8_t g_raceCurrentSensor;
static uint8_t g_raceLastNon00Sensor;
static uint8_t g_raceLastNon11Sensor;
static int g_raceSensorValid;
static uint32_t g_raceTransitions;
static TraceAction g_raceLastAction;
static uint32_t g_raceLastActionChangeTick;
static uint32_t g_raceLeftCorrectionCount;
static uint32_t g_raceRightCorrectionCount;
static uint32_t g_raceLeftCorrectionTicks;
static uint32_t g_raceRightCorrectionTicks;
static uint32_t g_raceCorrectionTick;
static uint32_t g_raceLast10Tick;
static uint32_t g_raceLast01Tick;
static uint8_t g_raceRawLeft;
static uint8_t g_raceRawRight;

static int RaceActionIsLeft(TraceAction action)
{
    return (action == TRACE_ACTION_LEFT || action == TRACE_ACTION_RECOVER_LEFT ||
            action == TRACE_ACTION_HOLD_LEFT) ? 1 : 0;
}

static int RaceActionIsRight(TraceAction action)
{
    return (action == TRACE_ACTION_RIGHT || action == TRACE_ACTION_RECOVER_RIGHT ||
            action == TRACE_ACTION_HOLD_RIGHT) ? 1 : 0;
}

static uint32_t RaceElapsedMs(uint32_t now)
{
    if (g_raceState < RACE_RUNNING) {
        return 0U;
    }
    return AppTicksToMs((uint32_t)(now - g_raceStartTick));
}

static uint32_t RaceMarkerGapMs(void)
{
    if (g_raceMarker2Tick == 0U || g_raceMarker1ClearTick == 0U) {
        return 0U;
    }
    return AppTicksToMs((uint32_t)(g_raceMarker2Tick - g_raceMarker1ClearTick));
}

static uint32_t RaceProbeElapsedMs(uint32_t now)
{
    if (g_raceMarker1ClearTick == 0U) {
        return 0U;
    }
    return AppTicksToMs((uint32_t)(now - g_raceMarker1ClearTick));
}

static void RacePublish(UdpRaceEvent event, uint32_t now)
{
    UdpRaceTelemetryState state;

    state.event = event;
    state.raceState = (uint8_t)g_raceState;
    state.sensorState = g_stableState;
    state.previousSensorState = g_racePreviousSensor;
    state.lastNon00SensorState = g_raceLastNon00Sensor;
    state.lastNon11SensorState = g_raceLastNon11Sensor;
    state.action = TraceToUdpTelemetryAction(g_lastAction);
    state.leftCommand = g_motorLeftCommand;
    state.rightCommand = g_motorRightCommand;
    state.elapsedMs = RaceElapsedMs(now);
    state.lastActionChangeMs = AppTicksToMs((uint32_t)(now - g_raceLastActionChangeTick));
    state.transitionCount = g_raceTransitions;
    state.leftCorrectionCount = g_raceLeftCorrectionCount;
    state.rightCorrectionCount = g_raceRightCorrectionCount;
    state.leftCorrectionMs = AppTicksToMs(g_raceLeftCorrectionTicks);
    state.rightCorrectionMs = AppTicksToMs(g_raceRightCorrectionTicks);
    state.markerGapMs = RaceMarkerGapMs();
    state.marker1ElapsedMs = (g_raceMarkerTick == 0U) ? 0U :
        AppTicksToMs((uint32_t)(g_raceMarkerTick - g_raceStartTick));
    state.probeElapsedMs = RaceProbeElapsedMs(now);
    state.last10AgeMs = AppTicksToMs((uint32_t)(now - g_raceLast10Tick));
    state.last01AgeMs = AppTicksToMs((uint32_t)(now - g_raceLast01Tick));
    state.sequence = 0U;
    UdpTelemetryUpdateRace(&state);
    g_raceLastTelemetryTick = now;
    g_raceLastDebugTick = now;
}

static int RaceUpdateSensorContext(uint32_t now)
{
    int changed = 0;

    if (g_raceSensorValid == 0) {
        g_raceCurrentSensor = g_stableState;
        g_racePreviousSensor = g_stableState;
        g_raceSensorValid = 1;
        changed = 1;
    } else if (g_raceCurrentSensor != g_stableState) {
        g_racePreviousSensor = g_raceCurrentSensor;
        g_raceCurrentSensor = g_stableState;
        g_raceTransitions++;
        changed = 1;
    }
    if (g_stableState != 0x00U) {
        g_raceLastNon00Sensor = g_stableState;
    }
    if (g_stableState != 0x03U) {
        g_raceLastNon11Sensor = g_stableState;
    }
    (void)now;
    return changed;
}

static int RaceUpdateActionContext(uint32_t now)
{
    uint32_t duration;
    int changed = 0;

    if (g_raceLastAction == g_lastAction) {
        return 0;
    }
    duration = (uint32_t)(now - g_raceLastActionChangeTick);
    if (RaceActionIsLeft(g_raceLastAction) != 0) {
        g_raceLeftCorrectionTicks += duration;
    } else if (RaceActionIsRight(g_raceLastAction) != 0) {
        g_raceRightCorrectionTicks += duration;
    }

    g_raceLastAction = g_lastAction;
    g_raceLastActionChangeTick = now;
    if (RaceActionIsLeft(g_lastAction) != 0) {
        g_raceLeftCorrectionCount++;
    } else if (RaceActionIsRight(g_lastAction) != 0) {
        g_raceRightCorrectionCount++;
    }
    if (g_lastCorrection == TRACE_CORRECTION_LEFT ||
        g_lastCorrection == TRACE_CORRECTION_RIGHT) {
        g_raceCorrectionTick = now;
    }
    changed = 1;
    return changed;
}

static void RaceReset(uint32_t now)
{
    g_raceState = RACE_WAIT_START;
    g_raceStartTick = 0U;
    g_raceMarkerTick = 0U;
    g_raceMarker1ClearTick = 0U;
    g_raceMarker2Tick = 0U;
    g_racePhaseStartTick = now;
    g_raceLastTelemetryTick = now;
    g_raceMarkerSamples = 0U;
    g_raceClearSamples = 0U;
    g_racePreviousSensor = 0U;
    g_raceCurrentSensor = 0U;
    g_raceLastNon00Sensor = 0U;
    g_raceLastNon11Sensor = 0U;
    g_raceSensorValid = 0;
    g_raceTransitions = 0U;
    g_raceLastAction = g_lastAction;
    g_raceLastActionChangeTick = now;
    g_raceLeftCorrectionCount = 0U;
    g_raceRightCorrectionCount = 0U;
    g_raceLeftCorrectionTicks = 0U;
    g_raceRightCorrectionTicks = 0U;
    g_raceCorrectionTick = now;
    g_raceLast10Tick = now;
    g_raceLast01Tick = now;
    g_raceRawLeft = 0U;
    g_raceRawRight = 0U;
}

static void RaceSendSpeed(int leftCommand, int rightCommand)
{
    TraceSendMotorCommand(leftCommand, rightCommand, 0);

    /* Race probe/finish commands are still forward or stop telemetry actions. */
    if (leftCommand == 0 && rightCommand == 0) {
        g_lastAction = TRACE_ACTION_STOP;
    } else if (leftCommand == rightCommand) {
        g_lastAction = TRACE_ACTION_FORWARD;
    }
}

static void RaceDebugPublish(uint32_t now)
{
    UdpRaceDebugState state;

    state.raceState = (uint8_t)g_raceState;
    state.rawLeft = g_raceRawLeft;
    state.rawRight = g_raceRawRight;
    state.sensorState = g_stableState;
    state.previousSensorState = g_racePreviousSensor;
    state.lastNon00SensorState = g_raceLastNon00Sensor;
    state.lastNon11SensorState = g_raceLastNon11Sensor;
    state.lastCorrection = (uint8_t)g_lastCorrection;
    state.action = TraceToUdpTelemetryAction(g_lastAction);
    state.leftCommand = g_motorLeftCommand;
    state.rightCommand = g_motorRightCommand;
    state.elapsedMs = RaceElapsedMs(now);
    state.correctionAgeMs = AppTicksToMs((uint32_t)(now - g_raceCorrectionTick));
    state.leftCorrectionCount = g_raceLeftCorrectionCount;
    state.rightCorrectionCount = g_raceRightCorrectionCount;
    state.transitionCount = g_raceTransitions;
    state.sequence = 0U;
    UdpTelemetryUpdateRaceDebug(&state);
    g_raceLastDebugTick = now;
}

static void RaceControlStep(WifiIotGpioValue rawLeft, WifiIotGpioValue rawRight,
                            uint32_t now)
{
    int sensorChanged = RaceUpdateSensorContext(now);
    int actionChanged = 0;

    g_raceRawLeft = (rawLeft == WIFI_IOT_GPIO_VALUE1) ? 1U : 0U;
    g_raceRawRight = (rawRight == WIFI_IOT_GPIO_VALUE1) ? 1U : 0U;
    if (g_stableState == 0x02U) {
        g_raceLast10Tick = now;
    } else if (g_stableState == 0x01U) {
        g_raceLast01Tick = now;
    }

    switch (g_raceState) {
        case RACE_WAIT_START:
            TraceApplyAction(TRACE_ACTION_STOP);
            if (g_stableState == 0x03U) {
                g_raceState = RACE_LEAVING_START;
                g_raceClearSamples = 0U;
                TraceApplyAction(TRACE_ACTION_FORWARD);
                RaceUpdateActionContext(now);
                RacePublish(UDP_RACE_EVENT_START_MARKER, now);
                RaceDebugPublish(now);
                printf("RACE start marker\r\n");
            }
            break;

        case RACE_LEAVING_START:
            TraceApplyAction(TRACE_ACTION_FORWARD);
            if (g_stableState != 0x03U) {
                if (g_raceClearSamples < TRACE_MARKER_CLEAR_SAMPLES) {
                    g_raceClearSamples++;
                }
                if (g_raceClearSamples >= TRACE_MARKER_CLEAR_SAMPLES) {
                    g_raceState = RACE_RUNNING;
                    g_raceStartTick = now;
                    g_raceMarkerSamples = 0U;
                    RaceUpdateActionContext(now);
                    RacePublish(UDP_RACE_EVENT_START_CLEAR, now);
                    RaceDebugPublish(now);
                    printf("RACE start clear\r\n");
                }
            } else {
                g_raceClearSamples = 0U;
            }
            break;

        case RACE_RUNNING:
            if (g_stableState == 0x03U) {
                if (g_raceMarkerSamples < TRACE_MARKER_CONFIRM_SAMPLES) {
                    g_raceMarkerSamples++;
                }
                if (g_raceMarkerSamples < TRACE_MARKER_CONFIRM_SAMPLES) {
                    TraceControlStep(WIFI_IOT_GPIO_VALUE1, WIFI_IOT_GPIO_VALUE1, now);
                } else {
                    RaceUpdateActionContext(now);
                    g_raceMarkerTick = now;
                    g_raceState = RACE_MARKER1;
                    g_raceClearSamples = 0U;
                    RaceSendSpeed(TRACE_MARKER_PROBE_SPEED, TRACE_MARKER_PROBE_SPEED);
                    RacePublish(UDP_RACE_EVENT_MARKER1, now);
                    RaceDebugPublish(now);
                    printf("RACE marker1 probe L=%d R=%d\r\n",
                           TRACE_MARKER_PROBE_SPEED, TRACE_MARKER_PROBE_SPEED);
                }
            } else {
                g_raceMarkerSamples = 0U;
                TraceControlStep(((g_stableState & 0x02U) != 0U) ?
                                     WIFI_IOT_GPIO_VALUE1 : WIFI_IOT_GPIO_VALUE0,
                                 ((g_stableState & 0x01U) != 0U) ?
                                     WIFI_IOT_GPIO_VALUE1 : WIFI_IOT_GPIO_VALUE0,
                                 now);
            }
            actionChanged = RaceUpdateActionContext(now);
            if (sensorChanged != 0 || actionChanged != 0) {
                RacePublish(UDP_RACE_EVENT_SENSOR, now);
            }
            break;

        case RACE_MARKER1:
            RaceSendSpeed(TRACE_MARKER_PROBE_SPEED, TRACE_MARKER_PROBE_SPEED);
            if (g_stableState != 0x03U) {
                if (g_raceClearSamples < TRACE_MARKER_CLEAR_SAMPLES) {
                    g_raceClearSamples++;
                }
                if (g_raceClearSamples >= TRACE_MARKER_CLEAR_SAMPLES) {
                    g_raceMarker1ClearTick = now;
                    g_raceState = RACE_MARKER1_CLEAR;
                    RacePublish(UDP_RACE_EVENT_MARKER1_CLEAR, now);
                    RaceDebugPublish(now);
                    printf("RACE marker1 clear\r\n");
                }
            } else {
                g_raceClearSamples = 0U;
            }
            break;

        case RACE_MARKER1_CLEAR:
            g_raceState = RACE_MARKER_PROBE;
            g_racePhaseStartTick = now;
            RaceSendSpeed(TRACE_MARKER_PROBE_SPEED, TRACE_MARKER_PROBE_SPEED);
            break;

        case RACE_MARKER_PROBE:
            RaceSendSpeed(TRACE_MARKER_PROBE_SPEED, TRACE_MARKER_PROBE_SPEED);
            if (g_stableState == 0x03U) {
                if (g_raceMarkerSamples < TRACE_MARKER_CONFIRM_SAMPLES) {
                    g_raceMarkerSamples++;
                }
                if (g_raceMarkerSamples >= TRACE_MARKER_CONFIRM_SAMPLES) {
                    g_raceMarker2Tick = now;
                    g_raceState = RACE_FINISH_CONFIRMED;
                    RacePublish(UDP_RACE_EVENT_FINISH_MARKER2, now);
                    RaceDebugPublish(now);
                    printf("RACE finish marker2 gap=%u ms\r\n",
                           (unsigned int)RaceMarkerGapMs());
                }
            } else {
                g_raceMarkerSamples = 0U;
            }
            if (g_raceState == RACE_MARKER_PROBE &&
                (uint32_t)(now - g_raceMarker1ClearTick) >=
                AppMsToTicks(TRACE_DOUBLE_MARKER_WINDOW_MS)) {
                g_raceState = RACE_DEADEND_CONFIRMED;
                g_lastAction = TRACE_ACTION_STOP;
                RaceSendSpeed(0, 0);
                RaceUpdateActionContext(now);
                RacePublish(UDP_RACE_EVENT_DEADEND, now);
                RacePublish(UDP_RACE_EVENT_DEADEND_SUMMARY, now);
                RaceDebugPublish(now);
                printf("RACE deadend, stop\r\n");
            }
            break;

        case RACE_FINISH_CONFIRMED:
            g_raceState = RACE_FINISH_SLOW;
            g_racePhaseStartTick = now;
            RaceSendSpeed(TRACE_FINISH_SLOW_SPEED, TRACE_FINISH_SLOW_SPEED);
            RacePublish(UDP_RACE_EVENT_FINISH_SLOW, now);
            RaceDebugPublish(now);
            break;

        case RACE_FINISH_SLOW:
            if ((uint32_t)(now - g_racePhaseStartTick) >=
                AppMsToTicks(TRACE_FINISH_SLOW_MS)) {
                g_raceState = RACE_FINISH_STOPPED;
                g_lastAction = TRACE_ACTION_STOP;
                RaceSendSpeed(0, 0);
                RaceUpdateActionContext(now);
                RacePublish(UDP_RACE_EVENT_FINISH_STOP, now);
                RaceDebugPublish(now);
                printf("RACE_RESULT elapsed_to_finish_ms=%u elapsed_to_stop_ms=%u marker_gap_ms=%u\r\n",
                       (unsigned int)AppTicksToMs((uint32_t)(g_raceMarker2Tick - g_raceStartTick)),
                       (unsigned int)RaceElapsedMs(now),
                       (unsigned int)RaceMarkerGapMs());
            }
            break;

        case RACE_DEADEND_CONFIRMED:
            g_raceState = RACE_DEADEND_STOPPED;
            break;

        case RACE_FINISH_STOPPED:
        case RACE_DEADEND_STOPPED:
        default:
            break;
    }

    if (sensorChanged != 0 || actionChanged != 0) {
        RaceDebugPublish(now);
    } else if ((uint32_t)(now - g_raceLastDebugTick) >=
               AppMsToTicks(TRACE_RACE_DEBUG_HEARTBEAT_MS)) {
        RaceDebugPublish(now);
    }

    if ((uint32_t)(now - g_raceLastTelemetryTick) >=
        AppMsToTicks(TRACE_RACE_HEARTBEAT_MS)) {
        RacePublish(UDP_RACE_EVENT_HEARTBEAT, now);
    }
}
#endif

static int AvoidDistanceFresh(uint8_t valid, uint32_t timestampMs, uint32_t nowMs)
{
    return (valid != 0U &&
            (uint32_t)(nowMs - timestampMs) <= AVOID_DISTANCE_STALE_MS) ? 1 : 0;
}

static UdpAvoidAction AvoidToUdpAction(AvoidAction action)
{
    switch (action) {
        case AVOID_ACTION_FORWARD:
            return UDP_AVOID_ACTION_FORWARD;
        case AVOID_ACTION_LEFT_TURN:
            return UDP_AVOID_ACTION_LEFT_TURN;
        case AVOID_ACTION_RIGHT_TURN:
            return UDP_AVOID_ACTION_RIGHT_TURN;
        case AVOID_ACTION_BLOCKED:
            return UDP_AVOID_ACTION_BLOCKED;
        default:
            return UDP_AVOID_ACTION_STOP;
    }
}

static void AvoidSendMotorCommand(int leftCommand, int rightCommand)
{
    stm32motor_control(leftCommand, rightCommand);
    g_avoidLeftCommand = leftCommand;
    g_avoidRightCommand = rightCommand;
    g_avoidMotorCommandValid = 1;
    g_avoidLastMotorTxTick = osKernelGetTickCount();
}

static void AvoidApplyAction(AvoidAction action)
{
    if (action == g_lastAvoidAction && g_avoidMotorCommandValid != 0) {
        return;
    }

    switch (action) {
        case AVOID_ACTION_FORWARD:
            AvoidSendMotorCommand(AVOID_FORWARD_SPEED, AVOID_FORWARD_SPEED);
            printf("AVOID forward L=%d R=%d\r\n", AVOID_FORWARD_SPEED,
                   AVOID_FORWARD_SPEED);
            break;
        case AVOID_ACTION_LEFT_TURN:
            AvoidSendMotorCommand(AVOID_TURN_INNER_SPEED, AVOID_TURN_OUTER_SPEED);
            printf("AVOID left L=%d R=%d\r\n", AVOID_TURN_INNER_SPEED,
                   AVOID_TURN_OUTER_SPEED);
            break;
        case AVOID_ACTION_RIGHT_TURN:
            AvoidSendMotorCommand(AVOID_TURN_OUTER_SPEED, AVOID_TURN_INNER_SPEED);
            printf("AVOID right L=%d R=%d\r\n", AVOID_TURN_OUTER_SPEED,
                   AVOID_TURN_INNER_SPEED);
            break;
        case AVOID_ACTION_BLOCKED:
            AvoidSendMotorCommand(0, 0);
            printf("AVOID blocked, stop\r\n");
            break;
        case AVOID_ACTION_STOP:
        default:
            AvoidSendMotorCommand(0, 0);
            printf("AVOID distance unavailable, stop\r\n");
            break;
    }

    g_lastAvoidAction = action;
}

static void AvoidHeartbeatIfDue(uint32_t now)
{
    if (g_avoidMotorCommandValid == 0 ||
        (uint32_t)(now - g_avoidLastMotorTxTick) <
        AppMsToTicks(MOTOR_COMMAND_HEARTBEAT_MS)) {
        return;
    }
    AvoidSendMotorCommand(g_avoidLeftCommand, g_avoidRightCommand);
}

static AvoidAction AvoidSelectAction(const Hcsr04Snapshot *snapshot, uint32_t nowMs)
{
    int frontFresh = AvoidDistanceFresh(snapshot->frontValid,
                                        snapshot->frontTimestampMs, nowMs);
    int leftFresh = AvoidDistanceFresh(snapshot->leftValid,
                                       snapshot->leftTimestampMs, nowMs);
    int rightFresh = AvoidDistanceFresh(snapshot->rightValid,
                                        snapshot->rightTimestampMs, nowMs);

    /* Unknown or stale front distance is never interpreted as clear. */
    if (frontFresh == 0) {
        return AVOID_ACTION_STOP;
    }

    if (snapshot->frontCm > AVOID_FRONT_STOP_CM) {
        /* Normal forward travel needs a fresh, complete three-direction scan. */
        if (leftFresh != 0 && rightFresh != 0) {
            return AVOID_ACTION_FORWARD;
        }
        return AVOID_ACTION_STOP;
    }

    if (leftFresh != 0 && rightFresh != 0) {
        if (snapshot->leftCm > snapshot->rightCm &&
            snapshot->leftCm > AVOID_SIDE_CLEAR_CM) {
            return AVOID_ACTION_LEFT_TURN;
        }
        if (snapshot->rightCm > snapshot->leftCm &&
            snapshot->rightCm > AVOID_SIDE_CLEAR_CM) {
            return AVOID_ACTION_RIGHT_TURN;
        }
        return AVOID_ACTION_BLOCKED;
    }

    /* With a confirmed front obstacle, one valid and clear side is sufficient. */
    if (leftFresh != 0 && snapshot->leftCm > AVOID_SIDE_CLEAR_CM) {
        return AVOID_ACTION_LEFT_TURN;
    }
    if (rightFresh != 0 && snapshot->rightCm > AVOID_SIDE_CLEAR_CM) {
        return AVOID_ACTION_RIGHT_TURN;
    }
    return AVOID_ACTION_BLOCKED;
}

static void AvoidUpdateUdpTelemetry(const Hcsr04Snapshot *snapshot, uint32_t nowMs)
{
    UdpAvoidTelemetryState state;

    state.leftCm = snapshot->leftCm;
    state.frontCm = snapshot->frontCm;
    state.rightCm = snapshot->rightCm;
    state.leftValid = (uint8_t)AvoidDistanceFresh(snapshot->leftValid,
                                                   snapshot->leftTimestampMs, nowMs);
    state.frontValid = (uint8_t)AvoidDistanceFresh(snapshot->frontValid,
                                                    snapshot->frontTimestampMs, nowMs);
    state.rightValid = (uint8_t)AvoidDistanceFresh(snapshot->rightValid,
                                                    snapshot->rightTimestampMs, nowMs);
    state.action = AvoidToUdpAction(g_lastAvoidAction);
    state.leftCommand = g_avoidLeftCommand;
    state.rightCommand = g_avoidRightCommand;
    state.sequence = 0U;
    UdpTelemetryUpdateAvoid(&state);
}

#endif

#if (REVERSE_REPLAY_STRAIGHT_TEST_MODE == 1)
static void ReplayPublish(UdpReplayEvent event, uint32_t now,
                          int origLeft, int origRight,
                          int replayLeft, int replayRight,
                          uint32_t frameIndex)
{
    UdpReplayTelemetryState telemetry = {0};

    telemetry.event = event;
    telemetry.phase = (uint8_t)g_replayState;
    telemetry.recordedSensor = (frameIndex < g_replayFrameCount) ?
        g_replayFrames[frameIndex].stableSensor : 0U;
    telemetry.currentSensor = g_stableState;
    telemetry.lowMagnitude = ((replayLeft != 0 && replayLeft > -90 && replayLeft < 90) ||
                              (replayRight != 0 && replayRight > -90 && replayRight < 90)) ? 1U : 0U;
    telemetry.origLeftCommand = origLeft;
    telemetry.origRightCommand = origRight;
    telemetry.replayLeftCommand = replayLeft;
    telemetry.replayRightCommand = replayRight;
    telemetry.frameIndex = frameIndex;
    telemetry.frameCount = g_replayFrameCount;
    telemetry.forwardDurationMs = (g_replayForwardStartTick != 0U) ?
        AppTicksToMs((uint32_t)(((g_replayMarkerTick != 0U) ? g_replayMarkerTick : now) -
                                 g_replayForwardStartTick)) : 0U;
    telemetry.elapsedMs = (g_replayForwardStartTick != 0U) ?
        AppTicksToMs((uint32_t)(now - g_replayForwardStartTick)) : 0U;
    telemetry.previousStableSensor = g_replayPreviousStableSensor;
    telemetry.stableBefore11[0] = g_replayStableBeforeMarker[0];
    telemetry.stableBefore11[1] = g_replayStableBeforeMarker[1];
    telemetry.stableBefore11[2] = g_replayStableBeforeMarker[2];
    telemetry.stableBefore11[3] = g_replayStableBeforeMarker[3];
    telemetry.stableBefore11[4] = g_replayStableBeforeMarker[4];
    telemetry.sinceLast00Ms = (g_replayLast00Tick != 0U) ?
        AppTicksToMs((uint32_t)(now - g_replayLast00Tick)) : 0U;
    telemetry.sinceLast10Ms = (g_replayLast10Tick != 0U) ?
        AppTicksToMs((uint32_t)(now - g_replayLast10Tick)) : 0U;
    telemetry.sinceLast01Ms = (g_replayLast01Tick != 0U) ?
        AppTicksToMs((uint32_t)(now - g_replayLast01Tick)) : 0U;
    telemetry.stable00DurationMs = g_replayLastStable00DurationMs;
    telemetry.stateBeforePrevious = g_replayStateBeforePrevious;
    telemetry.signedPreambleSensor = g_replaySignedPreambleSensor;
    telemetry.markerDecision = g_replayMarkerDecision;
    telemetry.markerReason = (uint8_t)g_replayMarkerReason;
    telemetry.signedPreambleMs = g_replaySignedPreambleMs;
    telemetry.markerLimitMs = REPLAY_MARKER_SIGNED_PREAMBLE_MAX_MS;
    telemetry.confirmElapsedMs = (g_replayMarkerConfirmStartTick != 0U) ?
        AppTicksToMs((uint32_t)(now - g_replayMarkerConfirmStartTick)) : 0U;
    telemetry.confirmTargetMs = REPLAY_MARKER_CONFIRM_FORWARD_MS;
    if (event == UDP_REPLAY_EVENT_COMMAND_CHANGE ||
        event == UDP_REPLAY_EVENT_HEARTBEAT ||
        event == UDP_REPLAY_EVENT_SENSOR_MISMATCH ||
        event == UDP_REPLAY_EVENT_REVERSE_COMMAND_CHANGE) {
        telemetry.markerDecision = 0U;
        telemetry.markerReason = UDP_REPLAY_MARKER_REASON_NONE;
        telemetry.signedPreambleSensor = 0U;
        telemetry.signedPreambleMs = 0U;
    }
    UdpTelemetryUpdateReplay(&telemetry);
    g_replayLastTelemetryTick = now;
}

static void ReplayReset(uint32_t now)
{
    g_replayState = REPLAY_WAIT_START;
    g_replayFrameCount = 0U;
    g_replayIndex = -1;
    g_replayForwardStartTick = 0U;
    g_replayMarkerTick = 0U;
    g_replayLastTelemetryTick = now;
    g_replayRecordActive = 0;
    g_replayBufferFull = 0;
    g_replayLastLeftCommand = 0;
    g_replayLastRightCommand = 0;
    g_replayStraightFrames = 0U;
    g_replayLeftCorrectionFrames = 0U;
    g_replayRightCorrectionFrames = 0U;
    g_replayOtherFrames = 0U;
    g_replayCommandChanges = 0U;
    g_replayPreviousStableSensor = 0U;
    g_replayCurrentStableSensor = 0U;
    g_replayStateBeforePrevious = 0U;
    g_replayStableSensorValid = 0U;
    g_replayStableBeforeMarker[0] = 0U;
    g_replayStableBeforeMarker[1] = 0U;
    g_replayStableBeforeMarker[2] = 0U;
    g_replayStableBeforeMarker[3] = 0U;
    g_replayStableBeforeMarker[4] = 0U;
    g_replayStableHistoryCount = 0U;
    g_replayLast00Tick = 0U;
    g_replayLast10Tick = 0U;
    g_replayLast01Tick = 0U;
    g_replayStable00StartTick = 0U;
    g_replayLastStable00DurationMs = 0U;
    g_replayStableStateEntryTick = 0U;
    g_replayPreviousStableEntryTick = 0U;
    g_replayMarkerDecision = 0U;
    g_replayMarkerReason = UDP_REPLAY_MARKER_REASON_NONE;
    g_replaySignedPreambleSensor = 0U;
    g_replaySignedPreambleMs = 0U;
    g_replayMarkerConfirmStartTick = 0U;
}

/* Called only on a non-11 -> 11 stable transition. */
static int ReplayMarkerCandidateAccept(uint32_t now)
{
    g_replayMarkerDecision = 0U;
    g_replayMarkerReason = UDP_REPLAY_MARKER_REASON_NO_00_CONTEXT;
    g_replaySignedPreambleSensor = g_replayPreviousStableSensor;
    g_replaySignedPreambleMs = 0U;

    if (g_replayPreviousStableSensor == 0U) {
        g_replayMarkerDecision = 1U;
        g_replayMarkerReason = UDP_REPLAY_MARKER_REASON_DIRECT_00_TO_11;
        return 1;
    }

    if (g_replayPreviousStableSensor == 2U || g_replayPreviousStableSensor == 1U) {
        g_replaySignedPreambleMs = AppTicksToMs(
            (uint32_t)(now - g_replayPreviousStableEntryTick));
        if (g_replayStateBeforePrevious == 0U &&
            g_replaySignedPreambleMs <= REPLAY_MARKER_SIGNED_PREAMBLE_MAX_MS) {
            g_replayMarkerDecision = 1U;
            g_replayMarkerReason = (g_replayPreviousStableSensor == 2U) ?
                UDP_REPLAY_MARKER_REASON_SHORT_10_PREAMBLE :
                UDP_REPLAY_MARKER_REASON_SHORT_01_PREAMBLE;
            return 1;
        }
        if (g_replayStateBeforePrevious == 0U) {
            g_replayMarkerReason = UDP_REPLAY_MARKER_REASON_SIGNED_PREAMBLE_TOO_LONG;
        }
    }
    return 0;
}

static void ReplayControlStep(WifiIotGpioValue left, WifiIotGpioValue right,
                              uint32_t now)
{
    WifiIotGpioValue stableLeft;
    WifiIotGpioValue stableRight;
    int previousLeft = g_motorLeftCommand;
    int previousRight = g_motorRightCommand;
    uint32_t frameIndex;
    ReplayFrame *frame;
    int replayLeft;
    int replayRight;
    int stableEntered = 0;

    if (g_replayState == REPLAY_FORWARD_RECORD ||
        g_replayState == REPLAY_MARKER_CONFIRM_FORWARD) {
        stableEntered = ReplayObserveStableSensor(now);
    }
    stableLeft = ((g_stableState & 0x02U) != 0U) ?
        WIFI_IOT_GPIO_VALUE1 : WIFI_IOT_GPIO_VALUE0;
    stableRight = ((g_stableState & 0x01U) != 0U) ?
        WIFI_IOT_GPIO_VALUE1 : WIFI_IOT_GPIO_VALUE0;

    switch (g_replayState) {
        case REPLAY_WAIT_START:
            TraceApplyAction(TRACE_ACTION_STOP);
            if (g_stableState == 0U) {
                g_replayState = REPLAY_FORWARD_RECORD;
                g_replayForwardStartTick = now;
                g_replayRecordActive = 1;
                g_motorCommandValid = 0;
                ReplayPublish(UDP_REPLAY_EVENT_RECORD_START, now, 0, 0, 0, 0, 0U);
            } else if ((uint32_t)(now - g_replayLastTelemetryTick) >=
                       AppMsToTicks(REPLAY_DEBUG_HEARTBEAT_MS)) {
                ReplayPublish(UDP_REPLAY_EVENT_WAIT_START, now, 0, 0, 0, 0, 0U);
            }
            break;

        case REPLAY_FORWARD_RECORD:
        case REPLAY_MARKER_CONFIRM_FORWARD:
            if (g_replayState == REPLAY_FORWARD_RECORD &&
                g_stableState == 3U && stableEntered != 0) {
                int markerAccepted = ReplayMarkerCandidateAccept(now);
                ReplayPublish(UDP_REPLAY_EVENT_MARKER_CANDIDATE, now, 0, 0, 0, 0,
                              g_replayFrameCount);
                if (markerAccepted != 0) {
                    g_replayMarkerConfirmStartTick = now;
                    g_replayState = REPLAY_MARKER_CONFIRM_FORWARD;
                    ReplayPublish(UDP_REPLAY_EVENT_MARKER_CONFIRM_START, now,
                                  g_motorLeftCommand, g_motorRightCommand, 0, 0,
                                  g_replayFrameCount);
                } else {
                    ReplayPublish(UDP_REPLAY_EVENT_MARKER_REJECTED, now,
                                  0, 0, 0, 0, g_replayFrameCount);
                }
            }

            TraceControlStep(stableLeft, stableRight, now);
            TraceUpdateUdpTelemetry();
            if (g_replayBufferFull != 0) {
                g_replayRecordActive = 0;
                g_replayState = REPLAY_ERROR;
                TraceApplyAction(TRACE_ACTION_STOP);
                ReplayPublish(UDP_REPLAY_EVENT_BUFFER_FULL, now, 0, 0, 0, 0,
                              g_replayFrameCount);
                break;
            }
            ReplayRecordDiagnosticSample(left, right, now);
            if (g_replayState == REPLAY_MARKER_CONFIRM_FORWARD) {
                uint32_t confirmElapsed = AppTicksToMs((uint32_t)(now - g_replayMarkerConfirmStartTick));
                if (g_stableState != 3U && confirmElapsed < REPLAY_MARKER_CONFIRM_FORWARD_MS) {
                    ReplayPublish(UDP_REPLAY_EVENT_MARKER_CONFIRM_REJECT, now,
                                  g_motorLeftCommand, g_motorRightCommand, 0, 0,
                                  (g_replayFrameCount == 0U) ? 0U : g_replayFrameCount - 1U);
                    g_replayMarkerDecision = 0U;
                    g_replayMarkerReason = UDP_REPLAY_MARKER_REASON_NONE;
                    g_replayMarkerConfirmStartTick = 0U;
                    g_replayState = REPLAY_FORWARD_RECORD;
                } else if (g_stableState == 3U && confirmElapsed >= REPLAY_MARKER_CONFIRM_FORWARD_MS) {
                    g_replayRecordActive = 0;
                    g_replayMarkerTick = now;
                    g_replayState = REPLAY_MARKER_SETTLE;
                    ReplayPublish(UDP_REPLAY_EVENT_MARKER_CONFIRMED, now,
                                  g_motorLeftCommand, g_motorRightCommand, 0, 0,
                                  g_replayFrameCount);
                    UdpTelemetryRequestReplayHistoryDump();
                    ReplayPublish(UDP_REPLAY_EVENT_END_MARKER_CONTEXT, now, 0, 0, 0, 0,
                                  g_replayFrameCount);
                    TraceApplyAction(TRACE_ACTION_STOP);
                    ReplayPublish(UDP_REPLAY_EVENT_END_MARKER, now, 0, 0, 0, 0,
                                  g_replayFrameCount);
                } else if ((uint32_t)(now - g_replayLastTelemetryTick) >=
                           AppMsToTicks(REPLAY_DEBUG_HEARTBEAT_MS)) {
                    ReplayPublish(UDP_REPLAY_EVENT_MARKER_CONFIRM_HEARTBEAT, now,
                                  g_motorLeftCommand, g_motorRightCommand, 0, 0,
                                  (g_replayFrameCount == 0U) ? 0U : g_replayFrameCount - 1U);
                }
            }
            if (previousLeft != g_motorLeftCommand || previousRight != g_motorRightCommand) {
                ReplayPublish(UDP_REPLAY_EVENT_COMMAND_CHANGE, now,
                              g_motorLeftCommand, g_motorRightCommand, 0, 0,
                              (g_replayFrameCount == 0U) ? 0U : g_replayFrameCount - 1U);
            } else if ((uint32_t)(now - g_replayLastTelemetryTick) >=
                       AppMsToTicks(REPLAY_DEBUG_HEARTBEAT_MS)) {
                ReplayPublish(UDP_REPLAY_EVENT_HEARTBEAT, now,
                              g_motorLeftCommand, g_motorRightCommand, 0, 0,
                              (g_replayFrameCount == 0U) ? 0U : g_replayFrameCount - 1U);
            }
            break;

        case REPLAY_MARKER_SETTLE:
            TraceApplyAction(TRACE_ACTION_STOP);
            if ((uint32_t)(now - g_replayMarkerTick) >=
                AppMsToTicks(REPLAY_TURNAROUND_SETTLE_MS)) {
                if (g_replayFrameCount == 0U) {
                    g_replayState = REPLAY_ERROR;
                    ReplayPublish(UDP_REPLAY_EVENT_BUFFER_FULL, now, 0, 0, 0, 0, 0U);
                } else {
                    g_replayState = REPLAY_REVERSE;
                    g_replayIndex = (int32_t)g_replayFrameCount - 1;
                    ReplayPublish(UDP_REPLAY_EVENT_REVERSE_START, now, 0, 0, 0, 0,
                                  (uint32_t)g_replayIndex);
                }
            }
            break;

        case REPLAY_REVERSE:
            if (g_replayIndex < 0) {
                g_replayState = REPLAY_DONE;
                TraceApplyAction(TRACE_ACTION_STOP);
                ReplayPublish(UDP_REPLAY_EVENT_DONE, now, 0, 0, 0, 0, 0U);
                break;
            }
            frameIndex = (uint32_t)g_replayIndex;
            frame = &g_replayFrames[frameIndex];
            replayLeft = -(int)frame->motorLeft;
            replayRight = -(int)frame->motorRight;
            TraceSendMotorCommand(replayLeft, replayRight, 0);
            g_replayIndex--;
            if (frame->stableSensor != g_stableState) {
                ReplayPublish(UDP_REPLAY_EVENT_SENSOR_MISMATCH, now,
                              frame->motorLeft, frame->motorRight,
                              replayLeft, replayRight, frameIndex);
            } else if (previousLeft != replayLeft || previousRight != replayRight) {
                ReplayPublish(UDP_REPLAY_EVENT_REVERSE_COMMAND_CHANGE, now,
                              frame->motorLeft, frame->motorRight,
                              replayLeft, replayRight, frameIndex);
            } else if ((uint32_t)(now - g_replayLastTelemetryTick) >=
                       AppMsToTicks(REPLAY_DEBUG_HEARTBEAT_MS)) {
                ReplayPublish(UDP_REPLAY_EVENT_HEARTBEAT, now,
                              frame->motorLeft, frame->motorRight,
                              replayLeft, replayRight, frameIndex);
            }
            break;

        case REPLAY_DONE:
        case REPLAY_ERROR:
        default:
            TraceApplyAction(TRACE_ACTION_STOP);
            break;
    }
}

static int ReplayIsMoving(void)
{
    return (g_replayState == REPLAY_FORWARD_RECORD ||
            g_replayState == REPLAY_REVERSE) ? 1 : 0;
}
#endif

#if (CROSS_AND_PROBE_TEST_MODE == 1)
typedef enum { PROBE_WAIT_START, PROBE_TRACE_FORWARD, PROBE_CROSS_11,
               PROBE_FORWARD_GUARD, PROBE_SCAN_LEFT, PROBE_SCAN_RIGHT,
               PROBE_SCAN_RESTORE_LEFT, PROBE_STOPPED } ProbeState;
static ProbeState g_probeState;
static uint32_t g_probePhaseStartTick;
static uint32_t g_probeCrossDurationMs;
static uint32_t g_probeGuardElapsedMs;
static uint32_t g_probeScanElapsedMs;
static uint8_t g_probePrevStable;
static uint8_t g_probeCandidatePrev;
static uint16_t g_probeBlackSeenCount;
static const char *g_probeResult = "NONE";
static uint8_t g_probeBootLogged;
static uint8_t g_probeWaitLogged;
static uint8_t g_probeStartLogged;
static uint8_t g_probeTraceStartLogged;

static void ProbeLog(const char *event, uint32_t now,
                     WifiIotGpioValue rawLeft, WifiIotGpioValue rawRight)
{
    printf("PROBE event=%s control_ms=%u probe_phase=%u phase_elapsed_ms=%u raw_left=%u raw_right=%u stable_sensor=%u%u prev_stable_sensor=%u%u L=%d R=%d candidate_entry_sensor=11 candidate_prev_sensor=%u cross_11_duration_ms=%u guard_elapsed_ms=%u scan_elapsed_ms=%u black_seen_count=%u result=%s\r\n",
           event, (unsigned int)AppTicksToMs(now), (unsigned int)g_probeState,
           (unsigned int)AppTicksToMs((uint32_t)(now - g_probePhaseStartTick)),
           (unsigned int)(rawLeft == WIFI_IOT_GPIO_VALUE1),
           (unsigned int)(rawRight == WIFI_IOT_GPIO_VALUE1),
           (unsigned int)((g_stableState >> 1) & 1U), (unsigned int)(g_stableState & 1U),
           (unsigned int)((g_probePrevStable >> 1) & 1U), (unsigned int)(g_probePrevStable & 1U),
           g_motorLeftCommand, g_motorRightCommand, (unsigned int)g_probeCandidatePrev,
           (unsigned int)g_probeCrossDurationMs, (unsigned int)g_probeGuardElapsedMs,
           (unsigned int)g_probeScanElapsedMs, (unsigned int)g_probeBlackSeenCount, g_probeResult);
}

static void ProbeStop(uint32_t now, const char *result,
                      WifiIotGpioValue l, WifiIotGpioValue r)
{
    g_probeResult = result; g_probeState = PROBE_STOPPED;
    TraceSendMotorCommand(0, 0, 0); ProbeLog("RESULT", now, l, r);
}

static void ProbeReset(uint32_t now)
{
    g_probeState = PROBE_WAIT_START; g_probePhaseStartTick = now;
    g_probeCrossDurationMs = g_probeGuardElapsedMs = g_probeScanElapsedMs = 0U;
    g_probePrevStable = 0U; g_probeCandidatePrev = 0U;
    g_probeBlackSeenCount = 0U; g_probeResult = "NONE";
    g_probeBootLogged = 0U; g_probeWaitLogged = 0U;
    g_probeStartLogged = 0U; g_probeTraceStartLogged = 0U;
}

static void ProbeControlStep(WifiIotGpioValue rawLeft, WifiIotGpioValue rawRight,
                             uint32_t now, int stableChanged)
{
    uint32_t elapsed = AppTicksToMs((uint32_t)(now - g_probePhaseStartTick));
    int black = (g_stableState != 0U);
    if (g_probeState == PROBE_WAIT_START) {
        TraceSendMotorCommand(0, 0, 0);
        if (g_probeWaitLogged == 0U) {
            ProbeLog("WAIT_START", now, rawLeft, rawRight);
            g_probeWaitLogged = 1U;
        }
        if (g_stableState == 0U) {
            g_probeState = PROBE_TRACE_FORWARD; g_probePhaseStartTick = now;
            ProbeLog("START_CONDITION_MET", now, rawLeft, rawRight);
            ProbeLog("FORWARD_START", now, rawLeft, rawRight);
        }
        return;
    }
    if (g_probeState == PROBE_TRACE_FORWARD && g_stableState == 3U && stableChanged != 0) {
        g_probeCandidatePrev = g_probePrevStable; g_probeState = PROBE_CROSS_11;
        g_probePhaseStartTick = now; g_probeCrossDurationMs = 0U; g_probeBlackSeenCount = 0U;
        ProbeLog("CANDIDATE_11", now, rawLeft, rawRight); ProbeLog("CROSS_START", now, rawLeft, rawRight);
        UdpTelemetryRequestReplayHistoryDump();
    }
    if (g_probeState == PROBE_TRACE_FORWARD && g_probeTraceStartLogged == 0U) {
        ProbeLog("TRACE_FORWARD_START", now, rawLeft, rawRight);
        g_probeTraceStartLogged = 1U;
    }
    switch (g_probeState) {
        case PROBE_TRACE_FORWARD:
            TraceControlStep((g_stableState & 0x02U) ? WIFI_IOT_GPIO_VALUE1 : WIFI_IOT_GPIO_VALUE0,
                             (g_stableState & 0x01U) ? WIFI_IOT_GPIO_VALUE1 : WIFI_IOT_GPIO_VALUE0, now);
            if (g_probeStartLogged == 0U) {
                ProbeLog("MOTOR_APPLY", now, rawLeft, rawRight);
                g_probeStartLogged = 1U;
            }
            break;
        case PROBE_CROSS_11:
            TraceSendMotorCommand(100, 100, 0); g_probeCrossDurationMs = elapsed; ProbeLog("CROSS_STEP", now, rawLeft, rawRight);
            if (g_stableState != 3U) { ProbeLog("CROSS_EXIT", now, rawLeft, rawRight); if (g_stableState == 1U || g_stableState == 2U) ProbeStop(now, "LINE_FOUND_DIRECT", rawLeft, rawRight); else { g_probeState = PROBE_FORWARD_GUARD; g_probePhaseStartTick = now; ProbeLog("GUARD_START", now, rawLeft, rawRight); } }
            else if (elapsed >= PROBE_CROSS_11_MAX_MS) ProbeStop(now, "INCONCLUSIVE_11_STUCK", rawLeft, rawRight);
            break;
        case PROBE_FORWARD_GUARD:
            TraceSendMotorCommand(100, 100, 0); g_probeGuardElapsedMs = elapsed; ProbeLog("GUARD_STEP", now, rawLeft, rawRight);
            if (g_stableState == 1U || g_stableState == 2U) ProbeStop(now, "LINE_FOUND_GUARD", rawLeft, rawRight);
            else if (elapsed >= PROBE_FORWARD_GUARD_MS) { ProbeLog("GUARD_END", now, rawLeft, rawRight); if (g_stableState != 0U) ProbeStop(now, "INCONCLUSIVE_GUARD", rawLeft, rawRight); else { g_probeState = PROBE_SCAN_LEFT; g_probePhaseStartTick = now; ProbeLog("SCAN_LEFT_START", now, rawLeft, rawRight); } }
            break;
        case PROBE_SCAN_LEFT: case PROBE_SCAN_RIGHT: case PROBE_SCAN_RESTORE_LEFT:
            if (black) { g_probeBlackSeenCount++; ProbeLog("BLACK_FOUND", now, rawLeft, rawRight); ProbeStop(now, (g_stableState == 3U) ? "BLACK_FOUND_DURING_SCAN" : "LINE_FOUND_SIGNED", rawLeft, rawRight); break; }
            g_probeScanElapsedMs = elapsed;
            if (g_probeState == PROBE_SCAN_LEFT) { TraceSendMotorCommand(60, 120, 0); if (elapsed >= PROBE_SCAN_LEFT_MS) { g_probeState = PROBE_SCAN_RIGHT; g_probePhaseStartTick = now; ProbeLog("SCAN_RIGHT_START", now, rawLeft, rawRight); } }
            else if (g_probeState == PROBE_SCAN_RIGHT) { TraceSendMotorCommand(120, 60, 0); if (elapsed >= PROBE_SCAN_RIGHT_MS) { g_probeState = PROBE_SCAN_RESTORE_LEFT; g_probePhaseStartTick = now; ProbeLog("SCAN_RESTORE_START", now, rawLeft, rawRight); } }
            else { TraceSendMotorCommand(60, 120, 0); if (elapsed >= PROBE_SCAN_RESTORE_LEFT_MS) ProbeStop(now, "NO_LINE_FOUND_AFTER_FULL_SCAN", rawLeft, rawRight); }
            break;
        default: TraceSendMotorCommand(0, 0, 0); break;
    }
    g_probePrevStable = g_stableState;
}
#endif

static const char *ForkTestDirectionName(uint8_t direction)
{
    if (direction == TRACE_CORRECTION_LEFT) return "LEFT";
    if (direction == TRACE_CORRECTION_RIGHT) return "RIGHT";
    return "NONE";
}

static const char *ForkTestStateName(int state)
{
    switch (state) {
        case FORKTEST_MARKED: return "MARKED";
        case FORKTEST_RETURNING_TO_FORK: return "RETURNING_TO_FORK";
        case FORKTEST_BACKOFF: return "BACKOFF";
        case FORKTEST_READY_REENTRY: return "READY_REENTRY";
        case FORKTEST_SELECT_OPPOSITE: return "SELECT_OPPOSITE";
        case FORKTEST_CAPTURED: return "CAPTURED";
        case FORKTEST_FAILED: return "FAILED";
        default: return "IDLE";
    }
}

static void ForkTestLineLiveSync(uint32_t now)
{
    g_lineLiveForkState = ForkTestStateName(g_forkTestState);
    g_lineLiveForkMarkValid = g_forkTestMarkValid;
    g_lineLiveForkTaken = ForkTestDirectionName(g_forkTestTaken);
    g_lineLiveForkDesired = ForkTestDirectionName(g_forkTestDesired);
    g_lineLiveForkRecordIndex = g_forkTestRecordIndex;
    g_lineLiveForkReturnCursor = g_forkTestReturnCursor;
    g_lineLiveForkMarkReached = g_forkTestMarkReached;
    g_lineLiveForkBackoffProgress = g_forkTestBackoffProgress;
    g_lineLiveForkReady = g_forkTestState == FORKTEST_READY_REENTRY ? 1U : 0U;
    g_lineLiveForkSelectMs = g_forkTestState == FORKTEST_SELECT_OPPOSITE ?
        AppTicksToMs(now - g_forkTestSelectStartTick) : 0U;
    g_lineLiveForkCaptureCount = g_forkTestCaptureCount;
    g_lineLiveForkResult = g_forkTestResult;
}

static void ForkTestReset(void)
{
    g_forkTestPendingCommand = FORK_TEST_COMMAND_NONE;
    g_forkTestState = FORKTEST_IDLE;
    g_forkTestMarkValid = 0U;
    g_forkTestTaken = TRACE_CORRECTION_NONE;
    g_forkTestDesired = TRACE_CORRECTION_NONE;
    g_forkTestRecordIndex = 0U;
    g_forkTestReferenceIndex = 0U;
    g_forkTestBackoffStopReferenceIndex = 0U;
    g_forkTestReturnCursor = 0U;
    g_forkTestMarkReached = 0U;
    g_forkTestBackoffProgress = 0U;
    g_forkTestSelectStartTick = 0U;
    g_forkTestCaptureCount = 0U;
    g_forkTestResult = "NONE";
    ForkTestLineLiveSync(osKernelGetTickCount());
}

static void AutoForkReturnReset(void)
{
    g_autoForkReturn = (AutoForkReturn){0};
    g_autoForkReturn.forwardStartIndex = 0xffffU;
    g_autoForkReturn.forwardEndIndex = 0xffffU;
}

static void ReentryCurveTestReset(void)
{
    g_reentryCurveTest = (ReentryCurveTest){0};
    g_reentryCurveTest.curve = REENTRY_CURVE_UNKNOWN;
    g_reentryCurveTest.state = REENTRY_TEST_IDLE;
    g_reentryCurveTest.forwardStartIndex = 0xffffU;
    g_reentryRecoveryPath = (ReentryRecoveryPath){0};
    g_reentryRecoveryPath.returnReason = "NONE";
}

static void ReentryTestFail(uint32_t now, const char *reason)
{
    uint32_t elapsedMs = AppTicksToMs(now - g_reentryCurveTest.startTick);

    printf("REENTRYTEST event=FAIL reason=%s elapsed_ms=%u turn_cycles=%u fwd_cycles=%u\r\n",
           reason, (unsigned int)elapsedMs,
           (unsigned int)g_reentryCurveTest.replayTurnCycles,
           (unsigned int)g_reentryCurveTest.replayFwdCycles);
    if (g_reentryRecoveryPath.recording != 0U) {
        ReentryRecoveryBeginReturn(now, reason);
    } else {
        g_reentryCurveTest.attempt = 0U;
        ReentryAnchorSettleBegin(now, 1U);
    }
}

/* The logical reverse endpoint is not necessarily the physical stop point.
 * Keep the motor at 0/0 until fresh encoder frames prove that reverse coast
 * has settled, then let the next owner-loop create the recovery epoch. */
static void ReentryAnchorSettleBegin(uint32_t now, uint8_t retry)
{
    UdpEncoderTelemetryState encoder;

    UdpTelemetryReadEncoder(&encoder);
    g_reentryCurveTest.state = REENTRY_TEST_ANCHOR_SETTLE;
    g_reentryCurveTest.settleStartTick = now;
    g_reentryCurveTest.settleLastValidCount = encoder.validCount;
    g_reentryCurveTest.settleLastLeft = encoder.totalLeft;
    g_reentryCurveTest.settleLastRight = encoder.totalRight;
    g_reentryCurveTest.settleSampleValid = encoder.validCount != 0U ? 1U : 0U;
    g_reentryCurveTest.settleStableCount = 0U;
    g_reentryCurveTest.settleResumeRetry = retry;
    g_bpathControlState = BPATH_CONTROL_REENTRY_ANCHOR_SETTLE;
    TraceApplyAction(TRACE_ACTION_STOP);
    LineLiveSetControl("READY_REENTRY_SETTLE", "STOP", 0, 0);
    printf("REENTRYSETTLE event=START attempt=%u enc_l=%ld enc_r=%ld retry=%u\r\n",
           (unsigned int)g_reentryCurveTest.attempt,
           (long)encoder.totalLeft, (long)encoder.totalRight,
           (unsigned int)retry);
}

static void ReentryAnchorSettleStep(uint32_t now)
{
    UdpEncoderTelemetryState encoder;
    int64_t deltaLeft;
    int64_t deltaRight;
    uint64_t absLeft;
    uint64_t absRight;

    TraceApplyAction(TRACE_ACTION_STOP);
    LineLiveSetControl("READY_REENTRY_SETTLE", "STOP", 0, 0);
    UdpTelemetryReadEncoder(&encoder);
    if (AppTicksToMs(now - g_reentryCurveTest.settleStartTick) >=
        REENTRY_ANCHOR_SETTLE_TIMEOUT_MS) {
        ReentryTestFail(now, "ANCHOR_SETTLE_TIMEOUT");
        return;
    }
    if (encoder.validCount == 0U ||
        encoder.validCount == g_reentryCurveTest.settleLastValidCount) {
        return;
    }
    if (g_reentryCurveTest.settleSampleValid == 0U) {
        g_reentryCurveTest.settleLastValidCount = encoder.validCount;
        g_reentryCurveTest.settleLastLeft = encoder.totalLeft;
        g_reentryCurveTest.settleLastRight = encoder.totalRight;
        g_reentryCurveTest.settleSampleValid = 1U;
        return;
    }
    deltaLeft = (int64_t)encoder.totalLeft -
        (int64_t)g_reentryCurveTest.settleLastLeft;
    deltaRight = (int64_t)encoder.totalRight -
        (int64_t)g_reentryCurveTest.settleLastRight;
    absLeft = deltaLeft < 0 ? (uint64_t)(-deltaLeft) : (uint64_t)deltaLeft;
    absRight = deltaRight < 0 ? (uint64_t)(-deltaRight) : (uint64_t)deltaRight;
    g_reentryCurveTest.settleLastValidCount = encoder.validCount;
    g_reentryCurveTest.settleLastLeft = encoder.totalLeft;
    g_reentryCurveTest.settleLastRight = encoder.totalRight;
    if (absLeft <= REENTRY_ANCHOR_SETTLE_MAX_DELTA &&
        absRight <= REENTRY_ANCHOR_SETTLE_MAX_DELTA) {
        g_reentryCurveTest.settleStableCount++;
    } else {
        g_reentryCurveTest.settleStableCount = 0U;
    }
    if (g_reentryCurveTest.settleStableCount < REENTRY_ANCHOR_SETTLE_STABLE_SAMPLES) {
        return;
    }
    printf("REENTRYSETTLE event=DONE stable_samples=%u anchor_l=%ld anchor_r=%ld\r\n",
           (unsigned int)g_reentryCurveTest.settleStableCount,
           (long)encoder.totalLeft, (long)encoder.totalRight);
    g_reentryCurveTest.state = g_reentryCurveTest.settleResumeRetry != 0U ?
        REENTRY_TEST_RETRY_READY : REENTRY_TEST_READY;
    g_bpathControlState = BPATH_CONTROL_FORK_READY;
}

__attribute__((unused)) static int ReentryRecoveryStart(uint32_t now)
{
    uint16_t count = 0U;
    UdpEncoderTelemetryState encoder;

    /* The completed wrong-branch return no longer needs its BPATH storage.
     * Reuse that single storage pool as a scratch recovery route. */
    UdpTelemetryReadEncoder(&encoder);
    if (encoder.validCount == 0U) {
        return -1;
    }
    BPathFollowInit();
    if (BPathExternalRecordStart(now) != 0) {
        return -1;
    }
    BPathExternalGetForwardRecordProgress(&count, NULL);
    g_reentryRecoveryPath = (ReentryRecoveryPath){0};
    g_reentryRecoveryPath.recording = 1U;
    /* RecordStart creates the zero source mark.  Do not sample motion until
     * APPROACH has issued the first forward owner command. */
    g_reentryRecoveryPath.recordStartPending = 1U;
    g_reentryRecoveryPath.lastRecordCount = count;
    g_reentryRecoveryPath.anchorLeft = encoder.totalLeft;
    g_reentryRecoveryPath.anchorRight = encoder.totalRight;
    g_reentryRecoveryPath.failureReason = "RECOVERY_RECORD_FAILURE";
    g_reentryRecoveryPath.returnReason = "NONE";
    printf("RECOVERYPATH event=START attempt=%u anchor_l=%ld anchor_r=%ld phase=READY_REENTRY\r\n",
           (unsigned int)g_reentryCurveTest.attempt,
           (long)g_reentryRecoveryPath.anchorLeft,
           (long)g_reentryRecoveryPath.anchorRight);
    return 0;
}

/* BPATH only appends a source point when fresh encoder movement arrives. */
static int ReentryRecoveryRecordStep(uint32_t now, uint16_t *advanced)
{
    uint16_t count = 0U;
    UdpEncoderTelemetryState encoder;

    if (advanced != NULL) {
        *advanced = 0U;
    }
    if (g_reentryRecoveryPath.recording == 0U) {
        return -1;
    }
    if (g_reentryRecoveryPath.recordStartPending != 0U) {
        g_reentryRecoveryPath.recordStartPending = 0U;
        return 0;
    }
    if (BPathExternalRecordStep(now) != 0) {
        return -1;
    }
    BPathExternalGetForwardRecordProgress(&count, NULL);
    if (count > g_reentryRecoveryPath.lastRecordCount) {
        if (advanced != NULL) {
            *advanced = (uint16_t)(count - g_reentryRecoveryPath.lastRecordCount);
        }
        g_reentryRecoveryPath.lastRecordCount = count;
        if (g_reentryRecoveryPath.firstForwardPointSeen == 0U) {
            int64_t deltaLeft;
            int64_t deltaRight;

            UdpTelemetryReadEncoder(&encoder);
            deltaLeft = (int64_t)encoder.totalLeft -
                (int64_t)g_reentryRecoveryPath.anchorLeft;
            deltaRight = (int64_t)encoder.totalRight -
                (int64_t)g_reentryRecoveryPath.anchorRight;
            printf("RECOVERYPATH event=FIRST_FORWARD_POINT enc_l=%ld enc_r=%ld "
                   "delta_l=%lld delta_r=%lld\r\n",
                   (long)encoder.totalLeft, (long)encoder.totalRight,
                   (long long)deltaLeft, (long long)deltaRight);
            if (deltaLeft < 0 || deltaRight < 0) {
                BPathExternalRecordStop(now);
                g_reentryRecoveryPath.recording = 0U;
                g_reentryRecoveryPath.failureReason =
                    "RECOVERY_FORWARD_START_INVALID";
                printf("RECOVERYPATH event=INVALID reason=RECOVERY_FORWARD_START_INVALID\r\n");
                return -1;
            }
            g_reentryRecoveryPath.firstForwardPointSeen = 1U;
        }
    }
    return 0;
}

static void ReentryRecoveryBeginReturn(uint32_t now, const char *reason)
{
    uint16_t count = 0U;

    if (g_reentryRecoveryPath.recording == 0U) {
        ReentryTestFail(now, "RECOVERY_PATH_INVALID");
        return;
    }
    BPathExternalGetForwardRecordProgress(&count, NULL);
    BPathExternalRecordStop(now);
    g_reentryRecoveryPath.recording = 0U;
    g_reentryRecoveryPath.returnReason = reason;
    g_reentryCurveTest.state = REENTRY_TEST_RECOVERY_RETURN;
    g_bpathControlState = BPATH_CONTROL_RECOVERY_RETURN_SETTLE;
    printf("REENTRYATTEMPT event=RETURN_START attempt=%u reason=%s recovery_points=%u\r\n",
           (unsigned int)g_reentryCurveTest.attempt, reason, (unsigned int)count);
    LineLiveSetControl("RECOVERY_RETURN", "RETURN_SETTLE", 0, 0);
}

static void ReentryLineSweepApply(void)
{
    int inner = (TRACE_SLOW_PWM * REENTRY_SWEEP_SCALE_PERCENT) / 100U;
    int outer = (TRACE_FAST_PWM * REENTRY_SWEEP_SCALE_PERCENT) / 100U;

    if (g_reentryRecoveryPath.sweepDirection == REENTRY_CURVE_LEFT) {
        TraceSendMotorCommand(inner, outer, 0);
        g_lastAction = TRACE_ACTION_LEFT;
        LineLiveSetControl("REENTRY_LINE_SWEEP", "SWEEP_LEFT", inner, outer);
    } else {
        TraceSendMotorCommand(outer, inner, 0);
        g_lastAction = TRACE_ACTION_RIGHT;
        LineLiveSetControl("REENTRY_LINE_SWEEP", "SWEEP_RIGHT", outer, inner);
    }
}

static void ReentryLineSweepStart(uint32_t now, const char *reason)
{
    g_reentryCurveTest.state = REENTRY_TEST_LINE_SWEEP;
    g_reentryCurveTest.captureStableCount = 0U;
    g_reentryRecoveryPath.directionalStableCount = 0U;
    g_reentryRecoveryPath.sweepLobeAdvances = 0U;
    g_reentryRecoveryPath.sweepLobeIndex = 1U;
    g_reentryRecoveryPath.straightSeenStable10 = 0U;
    g_reentryRecoveryPath.straightSeenStable01 = 0U;
    g_reentryRecoveryPath.straightStableState = 0xffU;
    g_reentryRecoveryPath.sweepExitedOldEleven = 0U;
    g_reentryRecoveryPath.reexitAdvances = 0U;
    /* Start away from a directional candidate.  STRAIGHT has no side bias,
     * so it always begins a symmetric sweep at LEFT. */
    g_reentryRecoveryPath.sweepDirection =
        ReentrySweepDirectionForLobe(g_reentryCurveTest.curve, 1U);
    g_bpathControlState = BPATH_CONTROL_REENTRY_LINE_SWEEP;
    printf("REENTRYTRUST event=EXPIRED candidate=%s advances=%u reason=%s\r\n",
           ReentryCurveName(g_reentryCurveTest.curve),
           (unsigned int)(g_reentryCurveTest.curve == REENTRY_CURVE_STRAIGHT ?
                              g_reentryCurveTest.departAdvances :
                              g_reentryCurveTest.post11Advances), reason);
    printf("REENTRYSCAN event=START attempt=%u candidate=%s first_sweep=%s\r\n",
           (unsigned int)g_reentryCurveTest.attempt,
           ReentryCurveName(g_reentryCurveTest.curve),
           ReentryCurveName(g_reentryRecoveryPath.sweepDirection));
    printf("REENTRYSCAN event=LOBE index=1 dir=%s advances=%u\r\n",
           ReentryCurveName(g_reentryRecoveryPath.sweepDirection),
           (unsigned int)ReentrySweepLobeAdvanceLimit(
               g_reentryCurveTest.curve, g_reentryRecoveryPath.sweepDirection));
    (void)now;
    ReentryLineSweepApply();
}

static void ReentryStraightTraceHandoff(uint32_t now)
{
    printf("REENTRYSTRAIGHT event=TRACE_HANDOFF reason=SAFE_LEFT_REACQUIRE "
           "sensor=10 stable_cycles=%u\r\n",
           (unsigned int)REENTRY_CAPTURE_STABLE_COUNT);
    if (ReentryTestStartNewTraceEpoch(now) != 0) {
        TraceApplyAction(TRACE_ACTION_STOP);
        LineLiveSetControl("REENTRY_FAIL", "STOP", 0, 0);
    }
}

/* This runs only on the owner-loop after READY_REENTRY has completed its
 * explicit 0/0 cycle.  It does not write motors itself. */
static void ReentryTestAutoStart(uint32_t now)
{
    if ((g_reentryCurveTest.state != REENTRY_TEST_READY &&
         g_reentryCurveTest.state != REENTRY_TEST_RETRY_READY) ||
        g_bpathControlState != BPATH_CONTROL_FORK_READY ||
        g_autoForkReturn.state != AUTO_FORK_READY_REENTRY) {
        return;
    }
    g_reentryCurveTest.state = REENTRY_TEST_LEFT_ESTABLISH;
    g_reentryCurveTest.leftExploreStartTick = now;
    g_bpathControlState = BPATH_CONTROL_REENTRY_LEFT_ESTABLISH;
    printf("REENTRY event=LEFT_TURN_START pwm_l=%d pwm_r=%d duration_ms=%u\r\n",
           TRACE_SLOW_PWM, TRACE_FAST_PWM,
           (unsigned int)REENTRY_MICRO_STRAIGHT_MS);
}

static void ReentryReplayApply(ReentryReplayCommand command, int64_t travelLeft,
                               int64_t travelRight, int64_t currentDiff,
                               uint64_t currentTotal)
{
    uint64_t targetTotal = g_reentryCurveTest.leftSum + g_reentryCurveTest.rightSum;
    const char *commandName = command == REENTRY_REPLAY_COMMAND_TURN ?
        ReentryCurveName(g_reentryCurveTest.curve) : "FWD";

    if (command == REENTRY_REPLAY_COMMAND_TURN) {
        g_reentryCurveTest.replayTurnCycles++;
        if (g_reentryCurveTest.curve == REENTRY_CURVE_LEFT) {
            TraceApplyAction(TRACE_ACTION_LEFT);
            LineLiveSetControl("REENTRY_REPLAY", "REPLAY_LEFT",
                               TRACE_SLOW_PWM, TRACE_FAST_PWM);
        } else {
            TraceApplyAction(TRACE_ACTION_RIGHT);
            LineLiveSetControl("REENTRY_REPLAY", "REPLAY_RIGHT",
                               TRACE_FAST_PWM, TRACE_SLOW_PWM);
        }
    } else {
        g_reentryCurveTest.replayFwdCycles++;
        /* Replay FWD is the frozen 100/100 primitive, not TRACE slow bias. */
        TraceSendMotorCommand(TRACE_FORWARD_SPEED, TRACE_FORWARD_SPEED, 0);
        g_lastAction = TRACE_ACTION_FORWARD;
        LineLiveSetControl("REENTRY_REPLAY", "REPLAY_FWD",
                           TRACE_FORWARD_SPEED, TRACE_FORWARD_SPEED);
    }

    if (g_reentryCurveTest.replayLastCommand != (uint8_t)command) {
        printf("REENTRYREPLAY event=CONTROL cmd=%s travel_l=%lld travel_r=%lld current_diff=%lld current_total=%llu target_diff=%lld target_total=%llu\r\n",
               commandName, (long long)travelLeft, (long long)travelRight,
               (long long)currentDiff, (unsigned long long)currentTotal,
               (long long)g_reentryCurveTest.diff,
               (unsigned long long)targetTotal);
        g_reentryCurveTest.replayLastCommand = (uint8_t)command;
    }
}

static void ReentryStraightApply(ReentryStraightCommand command, int64_t travelLeft,
                                 int64_t travelRight, int64_t error,
                                 uint64_t currentTotal)
{
    const char *commandName = "FWD";

    if (command == REENTRY_STRAIGHT_COMMAND_LEFT) {
        commandName = "LEFT";
        TraceApplyAction(TRACE_ACTION_LEFT);
        LineLiveSetControl("REENTRY_STRAIGHT", "STRAIGHT_LEFT",
                           TRACE_SLOW_PWM, TRACE_FAST_PWM);
    } else if (command == REENTRY_STRAIGHT_COMMAND_RIGHT) {
        commandName = "RIGHT";
        TraceApplyAction(TRACE_ACTION_RIGHT);
        LineLiveSetControl("REENTRY_STRAIGHT", "STRAIGHT_RIGHT",
                           TRACE_FAST_PWM, TRACE_SLOW_PWM);
    } else {
        TraceSendMotorCommand(TRACE_FORWARD_SPEED, TRACE_FORWARD_SPEED, 0);
        g_lastAction = TRACE_ACTION_FORWARD;
        LineLiveSetControl("REENTRY_STRAIGHT", "STRAIGHT_FWD",
                           TRACE_FORWARD_SPEED, TRACE_FORWARD_SPEED);
    }
    if (g_reentryCurveTest.straightLastCommand != (uint8_t)command) {
        printf("REENTRYSTRAIGHT event=CONTROL cmd=%s travel_l=%lld travel_r=%lld "
               "error=%lld total=%llu\r\n",
               commandName, (long long)travelLeft, (long long)travelRight,
               (long long)error, (unsigned long long)currentTotal);
        g_reentryCurveTest.straightLastCommand = (uint8_t)command;
    }
}

/* Shared by STRAIGHT departure and bounded old-11 re-exit.  It keeps the
 * already validated encoder-balanced 3% deadband controller identical in
 * both phases rather than treating 100/100 as physical straightness. */
static int ReentryStraightControlStep(void)
{
    UdpEncoderTelemetryState encoder;
    int64_t travelLeft;
    int64_t travelRight;
    int64_t currentDiff;
    uint64_t currentTotal;
    uint64_t absError;
    ReentryStraightCommand straightCommand = REENTRY_STRAIGHT_COMMAND_FWD;

    UdpTelemetryReadEncoder(&encoder);
    if (encoder.validCount == 0U ||
        g_reentryCurveTest.straightEncoderBaselineValid == 0U) {
        return -1;
    }
    travelLeft = (int64_t)encoder.totalLeft -
        (int64_t)g_reentryCurveTest.straightStartLeftEncoder;
    travelRight = (int64_t)encoder.totalRight -
        (int64_t)g_reentryCurveTest.straightStartRightEncoder;
    currentTotal = ReentryUnsignedTravelDelta(encoder.totalLeft,
        g_reentryCurveTest.straightStartLeftEncoder) +
        ReentryUnsignedTravelDelta(encoder.totalRight,
        g_reentryCurveTest.straightStartRightEncoder);
    currentDiff = travelLeft - travelRight;
    absError = currentDiff < 0 ? (uint64_t)(-currentDiff) :
        (uint64_t)currentDiff;
    if (currentTotal != 0U &&
        absError * 100U > currentTotal * REENTRY_STRAIGHT_ERROR_RATIO_PERCENT) {
        straightCommand = currentDiff > 0 ? REENTRY_STRAIGHT_COMMAND_LEFT :
            REENTRY_STRAIGHT_COMMAND_RIGHT;
    }
    ReentryStraightApply(straightCommand, travelLeft, travelRight,
                         currentDiff, currentTotal);
    return 0;
}

/* Reverse the scratch recovery epoch all the way to its source index zero.
 * This deliberately does not reuse AUTOFORK's mark/backoff observer. */
static void ReentryRecoveryReturnStep(uint32_t now)
{
    int leftCommand = 0;
    int rightCommand = 0;
    uint32_t terminalDelayMs = 0U;

    if (g_bpathControlState == BPATH_CONTROL_RECOVERY_RETURN_SETTLE) {
        if (BPathExternalRecordStep(now) != 0) {
            ReentryTestFail(now, "RECOVERY_SETTLE_RECORD_FAILURE");
            return;
        }
        EncoderExperimentSendMotorCommand(0, 0, now);
        LineLiveSetControl("RECOVERY_RETURN", "RETURN_SETTLE", 0, 0);
        if (BPathExternalReturnSettleComplete(now) != 0) {
            if (BPathExternalRecordFinish(now) != 0 ||
                BPathExternalReturnStart(now) != 0) {
                ReentryTestFail(now, "RECOVERY_REFERENCE_FAILURE");
                return;
            }
            g_bpathControlState = BPATH_CONTROL_RECOVERY_RETURN;
            printf("REENTRYATTEMPT event=RETURN_REVERSE reason=%s\r\n",
                   g_reentryRecoveryPath.returnReason);
        }
        return;
    }

    if (g_bpathControlState != BPATH_CONTROL_RECOVERY_RETURN) {
        return;
    }
    BPathFollowStep(now, &leftCommand, &rightCommand);
    if (BPathFollowTakeTerminalExecutionRequest(&terminalDelayMs) != 0) {
        uint32_t waitStartUs = (uint32_t)hi_get_us();
        uint32_t actualWaitUs;
        uint32_t actualStopMs;
        uint32_t actualStopTick;

        EncoderExperimentSendMotorCommand(leftCommand, rightCommand, now);
        hi_udelay(terminalDelayMs * 1000U);
        actualWaitUs = (uint32_t)((uint32_t)hi_get_us() - waitStartUs);
        actualStopMs = hi_get_milli_seconds();
        actualStopTick = osKernelGetTickCount();
        EncoderExperimentSendMotorCommand(0, 0, actualStopTick);
        BPathFollowNotifyTerminalStopExecuted(actualStopMs, actualWaitUs);
    } else {
        EncoderExperimentSendMotorCommand(leftCommand, rightCommand, now);
    }
    LineLiveSetControl("RECOVERY_RETURN", "RETURN", leftCommand, rightCommand);
    if (BPathFollowIsFinished() != 0) {
        if (BPathExternalReturnAborted() != 0) {
            ReentryTestFail(now, "RECOVERY_RETURN_ABORT");
            return;
        }
        EncoderExperimentSendMotorCommand(0, 0, now);
        printf("REENTRYATTEMPT event=RETURN_DONE attempt=%u\r\n",
               (unsigned int)g_reentryCurveTest.attempt);
        if (ReentryHasAnotherAttempt() != 0U) {
            ReentryAnchorSettleBegin(now, 1U);
        } else {
            g_reentryCurveTest.attempt = 0U;
            ReentryAnchorSettleBegin(now, 1U);
        }
    }
}

/* This is the only experimental motor owner.  It runs only after the existing
 * TWO_LONG reverse has parked at READY_REENTRY, so normal TRACE is untouched. */
static void ReentryTestStep(uint32_t now, WifiIotGpioValue rawLeft,
                            WifiIotGpioValue rawRight, int sensorValid)
{
    uint8_t rawState;
    uint32_t elapsedMs;
    UdpEncoderTelemetryState encoder;
    int64_t travelLeft;
    int64_t travelRight;
    int64_t currentDiff;
    int64_t signedCurrentDiff;
    uint64_t currentTotal;
    uint64_t targetTotal;
    uint64_t targetAbsDiff;
    uint32_t replayHardTimeoutMs;
    ReentryReplayCommand replayCommand;
    uint16_t recordAdvanced = 0U;
    uint8_t leftReplayDwell =
        (g_reentryCurveTest.candidateUsesReplay != 0U &&
         g_reentryCurveTest.historyCurve == REENTRY_CURVE_LEFT &&
         g_reentryCurveTest.curve == REENTRY_CURVE_LEFT &&
         g_reentryCurveTest.attempt == 1U) ? 1U : 0U;

    replayHardTimeoutMs = leftReplayDwell != 0U ?
        REENTRY_LEFT_REPLAY_HARD_TIMEOUT_MS : REENTRY_FORCE_TIMEOUT_MS;

    rawState = (uint8_t)((rawLeft == WIFI_IOT_GPIO_VALUE1 ? 2U : 0U) |
                         (rawRight == WIFI_IOT_GPIO_VALUE1 ? 1U : 0U));
    if (g_reentryCurveTest.state == REENTRY_TEST_LEFT_ESTABLISH) {
        TraceSendMotorCommand(TRACE_SLOW_PWM, TRACE_FAST_PWM, 0);
        g_lastAction = TRACE_ACTION_LEFT;
        LineLiveSetControl("REENTRY_LEFT_TURN", "LEFT",
                           TRACE_SLOW_PWM, TRACE_FAST_PWM);
        elapsedMs = AppTicksToMs(now - g_reentryCurveTest.leftExploreStartTick);
        if (elapsedMs >= REENTRY_MICRO_STRAIGHT_MS) {
            if (ReentryTestStartNewTraceEpoch(now) != 0) {
                TraceApplyAction(TRACE_ACTION_STOP);
                LineLiveSetControl("REENTRY_FAIL", "STOP", 0, 0);
            } else {
                printf("REENTRY event=TRACE_HANDOFF elapsed_ms=%u\r\n",
                       (unsigned int)elapsedMs);
            }
        }
        return;
    }

    if (g_reentryCurveTest.state == REENTRY_TEST_DEPART &&
        g_reentryCurveTest.candidateIndex < 11U) {
        if (ReentryRecoveryRecordStep(now, &recordAdvanced) != 0) {
            ReentryTestFail(now, g_reentryRecoveryPath.failureReason);
            return;
        }
        elapsedMs = AppTicksToMs(now - g_reentryCurveTest.phaseStartTick);
        TraceSendMotorCommand(100, 100, 0);
        LineLiveSetControl("REENTRY_MICRO_STRAIGHT", "FWD", 100, 100);
        if (elapsedMs >= REENTRY_MICRO_STRAIGHT_MS) {
            SensorSemanticSetValid(1U, "ACQUIRE_START");
            g_reentryCurveTest.phaseStartTick = now;
            g_reentryCurveTest.state = REENTRY_TEST_LINE_SWEEP;
            g_bpathControlState = BPATH_CONTROL_REENTRY_LINE_SWEEP;
            printf("REENTRY event=ACQUIRE_START index=%u timeout_ms=%u\r\n",
                   (unsigned int)g_reentryCurveTest.candidateIndex,
                   (unsigned int)REENTRY_SENSOR_ACQUIRE_MS);
        }
        return;
    }

    if (sensorValid == 0) {
        ReentryTestFail(now, "SENSOR_READ_FAILURE");
        return;
    }
    UdpTelemetryReadEncoder(&encoder);
    (void)TraceUpdateStableState(rawLeft, rawRight);
    if (ReentryRecoveryRecordStep(now, &recordAdvanced) != 0) {
        ReentryTestFail(now, g_reentryRecoveryPath.failureReason);
        return;
    }
    elapsedMs = AppTicksToMs(now - g_reentryCurveTest.startTick);

    if (g_reentryCurveTest.state == REENTRY_TEST_APPROACH) {
        if (rawState == 0x03U) {
            g_reentryCurveTest.state = g_reentryCurveTest.candidateUsesReplay != 0U ?
                REENTRY_TEST_FORCE : REENTRY_TEST_DEPART;
            g_reentryCurveTest.startTick = now;
            g_reentryCurveTest.captureStableCount = 0U;
            g_reentryCurveTest.departAdvances = 0U;
            g_reentryCurveTest.replayTurnCycles = 0U;
            g_reentryCurveTest.replayFwdCycles = 0U;
            g_reentryCurveTest.post11Active = 0U;
            g_reentryCurveTest.post11Advances = 0U;
            /* Advance(s) appended while APPROACH owned the motor must not
             * consume the bounded departure budget before its first command. */
            recordAdvanced = 0U;
            g_motorCommandValid = 0U;
            if (g_reentryCurveTest.candidateUsesReplay != 0U) {
                if (encoder.validCount == 0U) {
                    ReentryTestFail(now, "ENCODER_INVALID");
                    return;
                }
                g_reentryCurveTest.replayStartLeftEncoder = encoder.totalLeft;
                g_reentryCurveTest.replayStartRightEncoder = encoder.totalRight;
                g_reentryCurveTest.replayEncoderBaselineValid = 1U;
                g_reentryCurveTest.replayLastCommand = REENTRY_REPLAY_COMMAND_NONE;
                g_reentryCurveTest.replayEarlySensorLogged = 0U;
                g_reentryCurveTest.replayMinDwellLogged = 0U;
                g_reentryCurveTest.replayTrustDeferredLogged = 0U;
                g_bpathControlState = BPATH_CONTROL_REENTRY_FORCE;
            } else {
                g_reentryCurveTest.replayEncoderBaselineValid = 0U;
                if (g_reentryCurveTest.curve == REENTRY_CURVE_STRAIGHT) {
                    if (encoder.validCount == 0U) {
                        ReentryTestFail(now, "ENCODER_INVALID");
                        return;
                    }
                    g_reentryCurveTest.straightStartLeftEncoder = encoder.totalLeft;
                    g_reentryCurveTest.straightStartRightEncoder = encoder.totalRight;
                    g_reentryCurveTest.straightEncoderBaselineValid = 1U;
                    g_reentryCurveTest.straightLastCommand = REENTRY_STRAIGHT_COMMAND_NONE;
                } else {
                    g_reentryCurveTest.straightEncoderBaselineValid = 0U;
                }
                g_bpathControlState = BPATH_CONTROL_REENTRY_DEPART;
            }
            printf("REENTRYTEST event=E1_RAW_ENTER raw=%u stable=%u curve=%s\r\n",
                   (unsigned int)rawState, (unsigned int)g_stableState,
                   ReentryCurveName(g_reentryCurveTest.curve));
            if (g_reentryCurveTest.candidateUsesReplay != 0U) {
                printf("REENTRYREPLAY event=START candidate=%s attempt=%u min_ms=%u hard_ms=%u target_left=%llu target_right=%llu target_diff=%lld target_total=%llu target_ratio=%u\r\n",
                       ReentryCurveName(g_reentryCurveTest.curve),
                       (unsigned int)g_reentryCurveTest.attempt,
                       (unsigned int)(leftReplayDwell != 0U ? REENTRY_LEFT_REPLAY_MIN_MS : 0U),
                       (unsigned int)replayHardTimeoutMs,
                       (unsigned long long)g_reentryCurveTest.leftSum,
                       (unsigned long long)g_reentryCurveTest.rightSum,
                       (long long)g_reentryCurveTest.diff,
                       (unsigned long long)(g_reentryCurveTest.leftSum +
                                            g_reentryCurveTest.rightSum),
                       (unsigned int)g_reentryCurveTest.ratioPercent);
            } else if (g_reentryCurveTest.curve == REENTRY_CURVE_STRAIGHT) {
                printf("REENTRYSTRAIGHT event=START attempt=%u enc_l=%ld enc_r=%ld "
                       "deadband_percent=%u limit_advances=%u\r\n",
                       (unsigned int)g_reentryCurveTest.attempt,
                       (long)g_reentryCurveTest.straightStartLeftEncoder,
                       (long)g_reentryCurveTest.straightStartRightEncoder,
                       (unsigned int)REENTRY_STRAIGHT_ERROR_RATIO_PERCENT,
                       (unsigned int)REENTRY_STRAIGHT_TRUST_ADVANCES);
            } else {
                printf("REENTRYDEPART event=START attempt=%u dir=%s limit_advances=%u\r\n",
                       (unsigned int)g_reentryCurveTest.attempt,
                       ReentryCurveName(g_reentryCurveTest.curve),
                       (unsigned int)REENTRY_NO_HISTORY_DEPART_ADVANCES);
            }
        } else if (elapsedMs >= REENTRY_APPROACH_TIMEOUT_MS) {
            ReentryRecoveryBeginReturn(now, "NO_E1");
            return;
        } else {
            TraceControlStep(rawLeft, rawRight, now);
            LineLiveSetControl("REENTRY_APPROACH", "TRACE", g_motorLeftCommand,
                               g_motorRightCommand);
            return;
        }
    }

    if (g_reentryCurveTest.state == REENTRY_TEST_DEPART) {
        uint8_t directional = rawState != 0x03U && g_stableStateValid != 0U &&
            (g_stableState == 0x02U || g_stableState == 0x01U) ? 1U : 0U;
        uint8_t insideOldEleven = rawState == 0x03U ||
            g_stableStateValid == 0U || g_stableState == 0x03U ? 1U : 0U;
        uint16_t departLimit = g_reentryCurveTest.curve == REENTRY_CURVE_STRAIGHT ?
            REENTRY_STRAIGHT_TRUST_ADVANCES : REENTRY_NO_HISTORY_DEPART_ADVANCES;

        if (g_reentryCurveTest.curve == REENTRY_CURVE_STRAIGHT) {
            if (directional != 0U) {
                g_reentryCurveTest.captureStableCount++;
                if (g_reentryCurveTest.captureStableCount >= REENTRY_CAPTURE_STABLE_COUNT) {
                    if (ReentryIsStraightCandidate() != 0U &&
                        g_stableState == 0x02U) {
                        ReentryStraightTraceHandoff(now);
                    } else {
                        /* For this asymmetric candidate, 01 remains evidence
                         * of the right branch, not a successful handoff. */
                        printf("REENTRYSTRAIGHT event=%s sensor=%u advances=%u\r\n",
                               g_stableState == 0x01U ? "RIGHT_SIDE_HIT" : "SIDE_HIT",
                               (unsigned int)g_stableState,
                               (unsigned int)g_reentryCurveTest.departAdvances);
                        ReentryLineSweepStart(now, "STRAIGHT_SIDE_HIT");
                    }
                    return;
                }
            } else {
                g_reentryCurveTest.captureStableCount = 0U;
            }
        } else if (directional != 0U) {
            g_reentryCurveTest.captureStableCount++;
            if (g_reentryCurveTest.captureStableCount >= REENTRY_CAPTURE_STABLE_COUNT) {
                printf("REENTRYDEPART event=LINE_FOUND attempt=%u sensor=%u advances=%u\r\n",
                       (unsigned int)g_reentryCurveTest.attempt,
                       (unsigned int)g_stableState,
                       (unsigned int)g_reentryCurveTest.departAdvances);
                if (ReentryTestStartNewTraceEpoch(now) != 0) {
                    TraceApplyAction(TRACE_ACTION_STOP);
                    LineLiveSetControl("REENTRY_FAIL", "STOP", 0, 0);
                }
                return;
            }
        } else {
            g_reentryCurveTest.captureStableCount = 0U;
        }
        if (g_reentryCurveTest.curve == REENTRY_CURVE_STRAIGHT &&
            insideOldEleven == 0U && g_reentryCurveTest.post11Active == 0U) {
            g_reentryCurveTest.post11Active = 1U;
            g_reentryCurveTest.departAdvances = 0U;
        } else if (g_reentryCurveTest.curve != REENTRY_CURVE_STRAIGHT ||
                   g_reentryCurveTest.post11Active != 0U) {
            if (recordAdvanced != 0U) {
                g_reentryCurveTest.departAdvances = (uint16_t)
                    (g_reentryCurveTest.departAdvances + recordAdvanced);
            }
        }
        if (g_reentryCurveTest.post11Active != 0U ||
            g_reentryCurveTest.curve != REENTRY_CURVE_STRAIGHT) {
            if (g_reentryCurveTest.departAdvances >= departLimit) {
                if (insideOldEleven != 0U) {
                    /* A bounded departure that never leaves the old 11
                     * cannot become a meaningful line sweep. */
                    printf("REENTRYDEPART event=BOUND_INSIDE_11 candidate=%s advances=%u\r\n",
                           ReentryCurveName(g_reentryCurveTest.curve),
                           (unsigned int)g_reentryCurveTest.departAdvances);
                    ReentryRecoveryBeginReturn(now, "DEPART_BOUND_INSIDE_11");
                    return;
                }
                printf("%s event=DONE advances=%u\r\n",
                       g_reentryCurveTest.curve == REENTRY_CURVE_STRAIGHT ?
                           "REENTRYSTRAIGHT" : "REENTRYDEPART",
                       (unsigned int)g_reentryCurveTest.departAdvances);
                ReentryLineSweepStart(now, g_reentryCurveTest.curve == REENTRY_CURVE_STRAIGHT ?
                                     "STRAIGHT_TRUST_LIMIT" : "NO_HISTORY_DEPART_DONE");
                return;
            }
        }
        if (g_reentryCurveTest.curve == REENTRY_CURVE_STRAIGHT) {
            if (ReentryStraightControlStep() != 0) {
                ReentryTestFail(now, "ENCODER_INVALID");
                return;
            }
        } else if (g_reentryCurveTest.curve == REENTRY_CURVE_LEFT) {
            TraceApplyAction(TRACE_ACTION_LEFT);
            LineLiveSetControl("REENTRY_DEPART", "DEPART_LEFT",
                               TRACE_SLOW_PWM, TRACE_FAST_PWM);
        } else {
            TraceApplyAction(TRACE_ACTION_RIGHT);
            LineLiveSetControl("REENTRY_DEPART", "DEPART_RIGHT",
                               TRACE_FAST_PWM, TRACE_SLOW_PWM);
        }
        return;
    }

    if (g_reentryCurveTest.state == REENTRY_TEST_LINE_SWEEP) {
        if (g_reentryCurveTest.candidateIndex < 11U) {
            TraceSendMotorCommand(100, 100, 0);
            LineLiveSetControl("REENTRY_SENSOR_ACQUIRE", "FWD", 100, 100);
            if (g_stableState == 0x01U || g_stableState == 0x02U) {
                printf("REENTRY event=ACQUIRE_SUCCESS index=%u sensor=%02x elapsed_ms=%u\r\n",
                       (unsigned int)g_reentryCurveTest.candidateIndex,
                       (unsigned int)g_stableState,
                       (unsigned int)AppTicksToMs(now - g_reentryCurveTest.phaseStartTick));
                if (ReentryTestStartNewTraceEpoch(now) != 0) {
                    ReentryTestFail(now, "TRACE_EPOCH_START_FAILURE");
                }
                return;
            }
            if (AppTicksToMs(now - g_reentryCurveTest.phaseStartTick) >=
                REENTRY_SENSOR_ACQUIRE_MS) {
                printf("REENTRY event=ACQUIRE_TIMEOUT index=%u\r\n",
                       (unsigned int)g_reentryCurveTest.candidateIndex);
                ReentryRecoveryBeginReturn(now, "ACQUIRE_TIMEOUT");
                g_reentryCurveTest.candidateIndex =
                    (uint8_t)((g_reentryCurveTest.candidateIndex + 1U) % 11U);
            }
            return;
        }
        uint8_t directional = rawState != 0x03U && g_stableStateValid != 0U &&
            (g_stableState == 0x02U || g_stableState == 0x01U) ? 1U : 0U;
        uint8_t insideOldEleven = rawState == 0x03U ||
            g_stableStateValid == 0U || g_stableState == 0x03U ? 1U : 0U;

        /* A wide old-E1 11 is intersection geometry, not searchable
         * candidate space.  Keep sampling, but never consume scan evidence
         * or spatial lobe budget there. */
        if (insideOldEleven != 0U) {
            g_reentryRecoveryPath.directionalStableCount = 0U;
            g_reentryRecoveryPath.straightStableState = 0xffU;
            if (g_reentryRecoveryPath.sweepExitedOldEleven != 0U) {
                if (recordAdvanced != 0U) {
                    g_reentryRecoveryPath.reexitAdvances = (uint16_t)
                        (g_reentryRecoveryPath.reexitAdvances + recordAdvanced);
                }
                if (g_reentryRecoveryPath.reexitAdvances >= REENTRY_REEXIT_MAX_ADVANCES) {
                    printf("REENTRYSEARCH event=REEXIT_BOUND candidate=%s advances=%u\r\n",
                           ReentryCurveName(g_reentryCurveTest.curve),
                           (unsigned int)g_reentryRecoveryPath.reexitAdvances);
                    ReentryRecoveryBeginReturn(now, "REEXIT_BOUND_INSIDE_11");
                    return;
                }
                /* Re-entering old E1 is not line evidence.  Steer back out
                 * with the candidate owner, without spending any lobe budget. */
                if (g_reentryCurveTest.curve == REENTRY_CURVE_LEFT) {
                    TraceApplyAction(TRACE_ACTION_LEFT);
                    LineLiveSetControl("REENTRY_LINE_SWEEP", "REEXIT_LEFT",
                                       TRACE_SLOW_PWM, TRACE_FAST_PWM);
                } else if (g_reentryCurveTest.curve == REENTRY_CURVE_STRAIGHT) {
                    if (ReentryStraightControlStep() != 0) {
                        ReentryTestFail(now, "ENCODER_INVALID");
                    }
                } else {
                    ReentryLineSweepApply();
                }
            } else {
                ReentryLineSweepApply();
            }
            return;
        }
        if (g_reentryRecoveryPath.sweepExitedOldEleven == 0U) {
            g_reentryRecoveryPath.sweepExitedOldEleven = 1U;
            g_reentryRecoveryPath.reexitAdvances = 0U;
        }

        if (g_reentryCurveTest.curve == REENTRY_CURVE_STRAIGHT) {
            if (directional == 0U) {
                g_reentryRecoveryPath.directionalStableCount = 0U;
                g_reentryRecoveryPath.straightStableState = 0xffU;
            } else if (g_reentryRecoveryPath.straightStableState != g_stableState) {
                g_reentryRecoveryPath.straightStableState = g_stableState;
                g_reentryRecoveryPath.directionalStableCount = 1U;
            } else if (g_reentryRecoveryPath.directionalStableCount <
                       REENTRY_CAPTURE_STABLE_COUNT) {
                g_reentryRecoveryPath.directionalStableCount++;
            }
            if (g_reentryRecoveryPath.directionalStableCount >=
                REENTRY_CAPTURE_STABLE_COUNT) {
                if (g_stableState == 0x02U &&
                    g_reentryRecoveryPath.straightSeenStable10 == 0U) {
                    g_reentryRecoveryPath.straightSeenStable10 = 1U;
                    printf("REENTRYSCAN event=SIDE_HIT candidate=STRAIGHT sensor=10\r\n");
                    if (ReentryIsStraightCandidate() != 0U) {
                        ReentryStraightTraceHandoff(now);
                        return;
                    }
                } else if (g_stableState == 0x01U &&
                           g_reentryRecoveryPath.straightSeenStable01 == 0U) {
                    g_reentryRecoveryPath.straightSeenStable01 = 1U;
                    printf("REENTRYSCAN event=%s candidate=STRAIGHT sensor=01\r\n",
                           ReentryIsStraightCandidate() != 0U ?
                               "RIGHT_SIDE_HIT" : "SIDE_HIT");
                }
                if (ReentryIsStraightCandidate() == 0U &&
                    g_reentryRecoveryPath.straightSeenStable10 != 0U &&
                    g_reentryRecoveryPath.straightSeenStable01 != 0U) {
                    printf("REENTRYSCAN event=STRAIGHT_CONFIRMED seen10=1 seen01=1\r\n");
                    if (ReentryTestStartNewTraceEpoch(now) != 0) {
                        TraceApplyAction(TRACE_ACTION_STOP);
                        LineLiveSetControl("REENTRY_FAIL", "STOP", 0, 0);
                    }
                    return;
                }
            }
        } else if (directional != 0U) {
            g_reentryRecoveryPath.directionalStableCount++;
            if (g_reentryRecoveryPath.directionalStableCount >= REENTRY_CAPTURE_STABLE_COUNT) {
                printf("REENTRYSCAN event=LINE_FOUND attempt=%u candidate=%s sensor=%u lobe=%u recovery_advances=%u\r\n",
                       (unsigned int)g_reentryCurveTest.attempt,
                       ReentryCurveName(g_reentryCurveTest.curve),
                       (unsigned int)g_stableState,
                       (unsigned int)g_reentryRecoveryPath.sweepLobeIndex,
                       (unsigned int)(g_reentryRecoveryPath.lastRecordCount == 0U ?
                                      0U : g_reentryRecoveryPath.lastRecordCount - 1U));
                if (ReentryTestStartNewTraceEpoch(now) != 0) {
                    TraceApplyAction(TRACE_ACTION_STOP);
                    LineLiveSetControl("REENTRY_FAIL", "STOP", 0, 0);
                }
                return;
            }
        } else {
            g_reentryRecoveryPath.directionalStableCount = 0U;
        }
        if (recordAdvanced != 0U) {
            g_reentryRecoveryPath.sweepLobeAdvances = (uint16_t)
                (g_reentryRecoveryPath.sweepLobeAdvances + recordAdvanced);
            if (g_reentryRecoveryPath.sweepLobeAdvances >=
                ReentrySweepLobeAdvanceLimit(g_reentryCurveTest.curve,
                                              g_reentryRecoveryPath.sweepDirection)) {
                g_reentryRecoveryPath.sweepLobeIndex++;
                if (g_reentryRecoveryPath.sweepLobeIndex > REENTRY_SWEEP_MAX_LOBES) {
                    if (g_reentryCurveTest.curve == REENTRY_CURVE_STRAIGHT) {
                        const char *reason =
                            (g_reentryRecoveryPath.straightSeenStable10 != 0U ||
                             g_reentryRecoveryPath.straightSeenStable01 != 0U) ?
                            "ONE_SIDED_LINE" : "NO_LINE";
                        if (ReentryIsStraightCandidate() != 0U &&
                            g_reentryRecoveryPath.straightSeenStable01 != 0U &&
                            g_reentryRecoveryPath.straightSeenStable10 == 0U) {
                            reason = "RIGHT_ONLY_LINE";
                        }
                        printf("REENTRYSCAN event=STRAIGHT_REJECT reason=%s seen10=%u seen01=%u\r\n",
                               reason,
                               (unsigned int)g_reentryRecoveryPath.straightSeenStable10,
                               (unsigned int)g_reentryRecoveryPath.straightSeenStable01);
                        ReentryRecoveryBeginReturn(now, reason);
                    } else {
                        printf("REENTRYSCAN event=NO_LINE attempt=%u candidate=%s lobes=%u\r\n",
                               (unsigned int)g_reentryCurveTest.attempt,
                               ReentryCurveName(g_reentryCurveTest.curve),
                               (unsigned int)REENTRY_SWEEP_MAX_LOBES);
                        ReentryRecoveryBeginReturn(now, "NO_LINE");
                    }
                    return;
                }
                g_reentryRecoveryPath.sweepLobeAdvances = 0U;
                g_reentryRecoveryPath.sweepDirection = ReentrySweepDirectionForLobe(
                    g_reentryCurveTest.curve, g_reentryRecoveryPath.sweepLobeIndex);
                printf("REENTRYSCAN event=LOBE index=%u dir=%s advances=%u\r\n",
                       (unsigned int)g_reentryRecoveryPath.sweepLobeIndex,
                       ReentryCurveName(g_reentryRecoveryPath.sweepDirection),
                       (unsigned int)ReentrySweepLobeAdvanceLimit(
                           g_reentryCurveTest.curve,
                           g_reentryRecoveryPath.sweepDirection));
            }
        }
        ReentryLineSweepApply();
        return;
    }

    if (g_reentryCurveTest.state != REENTRY_TEST_FORCE) {
        return;
    }
    if (encoder.validCount == 0U || g_reentryCurveTest.replayEncoderBaselineValid == 0U) {
        ReentryTestFail(now, "ENCODER_INVALID");
        return;
    }
    elapsedMs = AppTicksToMs(now - g_reentryCurveTest.startTick);
    if (leftReplayDwell != 0U && elapsedMs < REENTRY_LEFT_REPLAY_MIN_MS &&
        rawState != 0x03U && g_stableStateValid != 0U &&
        (g_stableState == 0x02U || g_stableState == 0x01U) &&
        g_reentryCurveTest.replayEarlySensorLogged == 0U) {
        printf("REENTRYREPLAY event=EARLY_LINE_IGNORED sensor=%u elapsed_ms=%u\r\n",
               (unsigned int)g_stableState, (unsigned int)elapsedMs);
        g_reentryCurveTest.replayEarlySensorLogged = 1U;
    }
    if (leftReplayDwell != 0U && elapsedMs >= REENTRY_LEFT_REPLAY_MIN_MS &&
        g_reentryCurveTest.replayMinDwellLogged == 0U) {
        printf("REENTRYREPLAY event=MIN_DWELL_DONE elapsed_ms=%u\r\n",
               (unsigned int)elapsedMs);
        g_reentryCurveTest.replayMinDwellLogged = 1U;
    }
    if (rawState != 0x03U && g_stableStateValid != 0U &&
        (g_stableState == 0x02U || g_stableState == 0x01U) &&
        (leftReplayDwell == 0U || elapsedMs >= REENTRY_LEFT_REPLAY_MIN_MS) &&
        (leftReplayDwell == 0U || g_stableState == 0x02U)) {
        g_reentryCurveTest.captureStableCount++;
        printf("REENTRYTEST event=CAPTURE_PROGRESS count=%u sensor=%u\r\n",
               (unsigned int)g_reentryCurveTest.captureStableCount,
               (unsigned int)g_stableState);
        if (g_reentryCurveTest.captureStableCount >= REENTRY_CAPTURE_STABLE_COUNT) {
            printf("REENTRYTEST event=SUCCESS attempt=%u curve=%s sensor=%u elapsed_ms=%u turn_cycles=%u fwd_cycles=%u\r\n",
                   (unsigned int)g_reentryCurveTest.attempt,
                   ReentryCurveName(g_reentryCurveTest.curve), (unsigned int)g_stableState,
                   (unsigned int)AppTicksToMs(now - g_reentryCurveTest.startTick),
                   (unsigned int)g_reentryCurveTest.replayTurnCycles,
                   (unsigned int)g_reentryCurveTest.replayFwdCycles);
            if (ReentryTestStartNewTraceEpoch(now) != 0) {
                /* BpathControlBeginTraceRecord already follows its safe abort
                 * path if the fresh encoder snapshot cannot start recording. */
                TraceApplyAction(TRACE_ACTION_STOP);
                LineLiveSetControl("REENTRY_FAIL", "STOP", 0, 0);
            }
            if (leftReplayDwell != 0U) {
                printf("REENTRYREPLAY event=TRACE_HANDOFF sensor=%u elapsed_ms=%u\r\n",
                       (unsigned int)g_stableState, (unsigned int)elapsedMs);
            }
            return;
        }
    } else {
        g_reentryCurveTest.captureStableCount = 0U;
    }
    if (rawState != 0x03U && g_reentryCurveTest.post11Active == 0U) {
        g_reentryCurveTest.post11Active = 1U;
        g_reentryCurveTest.post11Advances = 0U;
    } else if (g_reentryCurveTest.post11Active != 0U && recordAdvanced != 0U) {
        g_reentryCurveTest.post11Advances = (uint16_t)
            (g_reentryCurveTest.post11Advances + recordAdvanced);
    }
    if (g_reentryCurveTest.post11Active != 0U &&
        g_reentryCurveTest.post11Advances >= REENTRY_HISTORY_TRUST_ADVANCES) {
        if (leftReplayDwell != 0U && elapsedMs < REENTRY_LEFT_REPLAY_MIN_MS) {
            if (g_reentryCurveTest.replayTrustDeferredLogged == 0U) {
                printf("REENTRYREPLAY event=TRUST_DEFERRED elapsed_ms=%u reason=MIN_DWELL\r\n",
                       (unsigned int)elapsedMs);
                g_reentryCurveTest.replayTrustDeferredLogged = 1U;
            }
        } else {
            printf("REENTRYREPLAY event=SWEEP_HANDOFF elapsed_ms=%u reason=TRUST_EXPIRED\r\n",
                   (unsigned int)elapsedMs);
            ReentryLineSweepStart(now, "HISTORY_TRUST_LIMIT");
            return;
        }
    }
    if (elapsedMs >= replayHardTimeoutMs) {
        printf("REENTRYREPLAY event=SWEEP_HANDOFF elapsed_ms=%u reason=TIMEOUT\r\n",
               (unsigned int)elapsedMs);
        ReentryLineSweepStart(now, "REPLAY_TIMEOUT");
        return;
    }

    travelLeft = (int64_t)encoder.totalLeft - (int64_t)g_reentryCurveTest.replayStartLeftEncoder;
    travelRight = (int64_t)encoder.totalRight - (int64_t)g_reentryCurveTest.replayStartRightEncoder;
    if (travelLeft < 0 || travelRight < 0) {
        ReentryTestFail(now, "ENCODER_INVALID");
        return;
    }
    currentTotal = (uint64_t)travelLeft + (uint64_t)travelRight;
    currentDiff = travelRight - travelLeft;
    signedCurrentDiff = g_reentryCurveTest.curve == REENTRY_CURVE_LEFT ?
        currentDiff : -currentDiff;
    targetTotal = g_reentryCurveTest.leftSum + g_reentryCurveTest.rightSum;
    targetAbsDiff = g_reentryCurveTest.diff < 0 ?
        (uint64_t)(-g_reentryCurveTest.diff) : (uint64_t)g_reentryCurveTest.diff;
    if (currentTotal == 0U || signedCurrentDiff <= 0 ||
        (uint64_t)signedCurrentDiff * targetTotal < targetAbsDiff * currentTotal) {
        replayCommand = REENTRY_REPLAY_COMMAND_TURN;
    } else {
        replayCommand = REENTRY_REPLAY_COMMAND_FWD;
    }
    ReentryReplayApply(replayCommand, travelLeft, travelRight, currentDiff, currentTotal);
}

/* Shared FORKTEST/AUTOFORK reverse-progress rule.  The cursor is a reverse
 * reference/source sample cursor, not an owner-loop iteration counter. */
static int ReturnMarkBackoffObserve(uint16_t markReferenceIndex,
                                    uint16_t backoffStopReferenceIndex,
                                    uint8_t *markReached,
                                    uint16_t *backoffProgress,
                                    uint16_t *returnCursor)
{
    uint16_t cursor;

    if (markReached == NULL || backoffProgress == NULL || returnCursor == NULL ||
        BPathExternalGetReturnReferenceCursor(&cursor) != 0) {
        return 0;
    }
    *returnCursor = cursor;
    if (*markReached == 0U && cursor >= markReferenceIndex) {
        *markReached = 1U;
    }
    if (*markReached != 0U) {
        *backoffProgress = cursor > markReferenceIndex ?
            (uint16_t)(cursor - markReferenceIndex) : 0U;
        if (cursor >= backoffStopReferenceIndex) {
            return 1;
        }
    }
    return 0;
}

static int AutoForkPrepareReturn(void)
{
    uint16_t referenceStartIndex;

    referenceStartIndex = 0U;
    (void)BPathExternalMapForwardIndexToReference(
        g_autoForkReturn.forwardStartIndex, &g_autoForkReturn.markReferenceIndex);
    (void)BPathExternalMapForwardIndexToReference(0U, &referenceStartIndex);
    g_autoForkReturn.backoffStopReferenceIndex =
        (uint16_t)((uint32_t)g_autoForkReturn.markReferenceIndex + FORK_BACKOFF_SAMPLES <=
                   (uint32_t)referenceStartIndex ?
                   (uint32_t)g_autoForkReturn.markReferenceIndex + FORK_BACKOFF_SAMPLES :
                   (uint32_t)referenceStartIndex);
    g_autoForkReturn.returnCursor = 0U;
    g_autoForkReturn.backoffProgress = 0U;
    g_autoForkReturn.markReached = 0U;
    g_autoForkReturn.state = AUTO_FORK_RETURNING_TO_MARK;
    printf("AUTOFORK event=RETURN_BEGIN mark_forward=%u mark_reverse=%u\r\n",
           (unsigned int)g_autoForkReturn.forwardStartIndex,
           (unsigned int)g_autoForkReturn.markReferenceIndex);
    return 0;
}

static int AutoForkObserveReturn(void)
{
    uint8_t wasMarkReached = g_autoForkReturn.markReached;

    if (g_autoForkReturn.state != AUTO_FORK_RETURNING_TO_MARK &&
        g_autoForkReturn.state != AUTO_FORK_BACKOFF) {
        return 0;
    }
    (void)ReturnMarkBackoffObserve(g_autoForkReturn.markReferenceIndex,
                                   g_autoForkReturn.backoffStopReferenceIndex,
                                   &g_autoForkReturn.markReached,
                                   &g_autoForkReturn.backoffProgress,
                                   &g_autoForkReturn.returnCursor);
    if (wasMarkReached == 0U && g_autoForkReturn.markReached != 0U) {
        g_autoForkReturn.state = AUTO_FORK_BACKOFF;
        printf("AUTOFORK event=MARK_REACHED forward_source=%u reverse_cursor=%u\r\n",
               (unsigned int)g_autoForkReturn.forwardStartIndex,
               (unsigned int)g_autoForkReturn.returnCursor);
    }
    if (g_autoForkReturn.state == AUTO_FORK_BACKOFF &&
        g_autoForkReturn.returnCursor >= g_autoForkReturn.backoffStopReferenceIndex) {
        printf("AUTOFORK event=BACKOFF_DONE samples=%u forward_source=%u\r\n",
               (unsigned int)g_autoForkReturn.backoffProgress,
               (unsigned int)g_autoForkReturn.forwardStartIndex);
        return 1;
    }
    return 0;
}

/* Returns one only when this owner-loop consumed a valid attended fork claim. */
static uint8_t ForkTestConsumeCommand(void)
{
    ForkTestCommand command = g_forkTestPendingCommand;

    g_forkTestPendingCommand = FORK_TEST_COMMAND_NONE;
    if (command == FORK_TEST_COMMAND_RESET) {
        ForkTestReset();
        return 0U;
    }
    if ((command == FORK_TEST_COMMAND_MARK_LEFT || command == FORK_TEST_COMMAND_MARK_RIGHT) &&
        g_bpathControlState == BPATH_CONTROL_TRACE_RECORD &&
        BPathExternalGetForwardRecordIndex(&g_forkTestRecordIndex) == 0) {
        g_forkTestTaken = command == FORK_TEST_COMMAND_MARK_LEFT ?
            TRACE_CORRECTION_LEFT : TRACE_CORRECTION_RIGHT;
        g_forkTestDesired = g_forkTestTaken == TRACE_CORRECTION_LEFT ?
            TRACE_CORRECTION_RIGHT : TRACE_CORRECTION_LEFT;
        g_forkTestMarkValid = 1U;
        g_forkTestState = FORKTEST_MARKED;
        ForkTestLineLiveSync(osKernelGetTickCount());
        return 1U;
    } else if (command == FORK_TEST_COMMAND_GO &&
               g_bpathControlState == BPATH_CONTROL_FORK_READY &&
               g_forkTestState == FORKTEST_READY_REENTRY) {
        g_forkTestState = FORKTEST_SELECT_OPPOSITE;
        g_forkTestSelectStartTick = osKernelGetTickCount();
        g_bpathControlState = BPATH_CONTROL_FORK_SELECT;
    }
    ForkTestLineLiveSync(osKernelGetTickCount());
    return 0U;
}

static int ForkTestPrepareReturn(void)
{
    uint16_t referenceStartIndex;

    if (g_forkTestMarkValid == 0U ||
        BPathExternalMapForwardIndexToReference(g_forkTestRecordIndex,
                                                &g_forkTestReferenceIndex) != 0 ||
        BPathExternalMapForwardIndexToReference(0U, &referenceStartIndex) != 0) {
        return -1;
    }
    g_forkTestBackoffStopReferenceIndex =
        (uint16_t)((uint32_t)g_forkTestReferenceIndex + FORK_BACKOFF_SAMPLES <=
                   (uint32_t)referenceStartIndex ?
                   (uint32_t)g_forkTestReferenceIndex + FORK_BACKOFF_SAMPLES :
                   (uint32_t)referenceStartIndex);
    g_forkTestState = FORKTEST_RETURNING_TO_FORK;
    g_forkTestMarkReached = 0U;
    g_forkTestBackoffProgress = 0U;
    ForkTestLineLiveSync(osKernelGetTickCount());
    return 0;
}

static int ForkTestObserveReturn(void)
{
    if ((g_forkTestState != FORKTEST_RETURNING_TO_FORK &&
         g_forkTestState != FORKTEST_BACKOFF)) {
        return 0;
    }
    if (ReturnMarkBackoffObserve(g_forkTestReferenceIndex,
                                 g_forkTestBackoffStopReferenceIndex,
                                 &g_forkTestMarkReached,
                                 &g_forkTestBackoffProgress,
                                 &g_forkTestReturnCursor) == 0) {
        if (g_forkTestMarkReached == 0U) {
            ForkTestLineLiveSync(osKernelGetTickCount());
            return 0;
        }
    }
    if (g_forkTestState == FORKTEST_RETURNING_TO_FORK &&
        g_forkTestMarkReached != 0U) {
        g_forkTestMarkReached = 1U;
        g_forkTestState = FORKTEST_BACKOFF;
    }
    if (g_forkTestState == FORKTEST_BACKOFF &&
        g_forkTestReturnCursor >= g_forkTestBackoffStopReferenceIndex) {
        ForkTestLineLiveSync(osKernelGetTickCount());
        return 1;
    }
    ForkTestLineLiveSync(osKernelGetTickCount());
    return 0;
}

static void ForkTestSelectStep(uint32_t now, uint8_t stableState)
{
    uint8_t target = g_forkTestDesired == TRACE_CORRECTION_LEFT ? 0x02U : 0x01U;

    if ((uint32_t)(now - g_forkTestSelectStartTick) >= AppMsToTicks(FORK_SELECT_MAX_MS)) {
        TraceApplyAction(TRACE_ACTION_STOP);
        g_forkTestState = FORKTEST_FAILED;
        g_forkTestResult = "SELECT_TIMEOUT";
        BpathControlFinish();
        ForkTestLineLiveSync(now);
        return;
    }
    if (g_forkTestDesired == TRACE_CORRECTION_LEFT) {
        TraceApplyAction(TRACE_ACTION_LEFT);
    } else {
        TraceApplyAction(TRACE_ACTION_RIGHT);
    }
    if (stableState == target) {
        g_forkTestCaptureCount++;
        TraceApplyAction(TRACE_ACTION_STOP);
        g_forkTestState = FORKTEST_CAPTURED;
        g_forkTestResult = g_forkTestDesired == TRACE_CORRECTION_LEFT ?
            "CAPTURED_LEFT" : "CAPTURED_RIGHT";
        BpathControlFinish();
    }
    ForkTestLineLiveSync(now);
}

static void CarControlTask(void *argument)
{
#if (LINE_SENSOR_CALIBRATION_MODE == 1)
    WifiIotGpioValue physicalLeft = WIFI_IOT_GPIO_VALUE0;
    WifiIotGpioValue physicalRight = WIFI_IOT_GPIO_VALUE0;

    (void)argument;
    car_stop();
    printf("LINE SENSOR CALIBRATION MODE: motor stopped\r\n");

    for (;;) {
        /*
         * LineSensorRead returns physicalLeft=GPIO13 and physicalRight=GPIO14.
         * CAL labels are intentionally retained for the existing Windows tool:
         * CAL L is physical right, and CAL R is physical left.
         */
        if (LineSensorRead(&physicalLeft, &physicalRight) == 1) {
            UdpTelemetryUpdateLineCalibration((uint8_t)physicalRight,
                                              (uint8_t)physicalLeft);
        }
        osDelay(AppMsToTicks(LINE_SENSOR_CALIBRATION_PERIOD_MS));
    }
#elif (LINE_SENSOR_ANALYSIS_MODE == 1)
    WifiIotGpioValue left = WIFI_IOT_GPIO_VALUE0;
    WifiIotGpioValue right = WIFI_IOT_GPIO_VALUE0;
    uint32_t left0 = 0U;
    uint32_t left1 = 0U;
    uint32_t leftTransitions = 0U;
    uint32_t right0 = 0U;
    uint32_t right1 = 0U;
    uint32_t rightTransitions = 0U;
    uint32_t windowStartTick;
    WifiIotGpioValue previousLeft = WIFI_IOT_GPIO_VALUE0;
    WifiIotGpioValue previousRight = WIFI_IOT_GPIO_VALUE0;
    int hasPreviousSample = 0;

    (void)argument;
    car_stop();
    windowStartTick = osKernelGetTickCount();
    printf("LINE SENSOR ANALYSIS MODE: motor stopped, raw period=%u ms\r\n",
           (unsigned int)LINE_SENSOR_ANALYSIS_PERIOD_MS);

    for (;;) {
        uint32_t now = osKernelGetTickCount();

        /* Direct GPIO read through LineSensorRead: no debounce or trace decision. */
        if (LineSensorRead(&left, &right) == 1) {
            if (left == WIFI_IOT_GPIO_VALUE1) {
                left1++;
            } else {
                left0++;
            }
            if (right == WIFI_IOT_GPIO_VALUE1) {
                right1++;
            } else {
                right0++;
            }

            if (hasPreviousSample != 0) {
                if (left != previousLeft) {
                    leftTransitions++;
                }
                if (right != previousRight) {
                    rightTransitions++;
                }
            }
            previousLeft = left;
            previousRight = right;
            hasPreviousSample = 1;
        }

        if ((uint32_t)(now - windowStartTick) >=
            AppMsToTicks(LINE_SENSOR_ANALYSIS_WINDOW_MS)) {
            uint32_t leftTotal = left0 + left1;
            uint32_t rightTotal = right0 + right1;
            uint32_t leftPctTenth = 0U;
            uint32_t rightPctTenth = 0U;

            if (leftTotal != 0U) {
                leftPctTenth = (left1 * 1000U + (leftTotal / 2U)) / leftTotal;
            }
            if (rightTotal != 0U) {
                rightPctTenth = (right1 * 1000U + (rightTotal / 2U)) / rightTotal;
            }

            printf("SENSORSTAT ms=%u L0=%u L1=%u Lt=%u Lpct1=%u.%u "
                   "R0=%u R1=%u Rt=%u Rpct1=%u.%u\r\n",
                   (unsigned int)((uint64_t)now * 1000U / osKernelGetTickFreq()),
                   (unsigned int)left0, (unsigned int)left1,
                   (unsigned int)leftTransitions,
                   (unsigned int)(leftPctTenth / 10U),
                   (unsigned int)(leftPctTenth % 10U),
                   (unsigned int)right0, (unsigned int)right1,
                   (unsigned int)rightTransitions,
                   (unsigned int)(rightPctTenth / 10U),
                   (unsigned int)(rightPctTenth % 10U));
            UdpTelemetryUpdateSensorStats(left0, left1, leftTransitions,
                                           right0, right1, rightTransitions);

            left0 = 0U;
            left1 = 0U;
            leftTransitions = 0U;
            right0 = 0U;
            right1 = 0U;
            rightTransitions = 0U;
            hasPreviousSample = 0;
            windowStartTick = now;
        }

        osDelay(AppMsToTicks(LINE_SENSOR_ANALYSIS_PERIOD_MS));
    }
#elif (LINE_SENSOR_SIDE_TEST_MODE == 1)
    WifiIotGpioValue left = WIFI_IOT_GPIO_VALUE0;
    WifiIotGpioValue right = WIFI_IOT_GPIO_VALUE0;

    (void)argument;
    printf("TRACE SENSOR SIDE TEST START: motor forced 0/0\r\n");

    for (;;) {
        uint32_t now = osKernelGetTickCount();

        /* This owner-loop test intentionally wins over every motion workflow. */
        TraceStaticTestForceStop();
        if (LineSensorRead(&left, &right) == 1) {
            char text[160];

            /* Raw, unfiltered mapping: GPIO13 is TRACE left; GPIO14 is TRACE right. */
            (void)snprintf(text, sizeof(text),
                "TRACE_SIDE ms=%u gpio13=%d gpio14=%d trace_left=%d trace_right=%d motor_l=0 motor_r=0",
                (unsigned int)((uint64_t)now * 1000U / osKernelGetTickFreq()),
                (int)left, (int)right, (int)left, (int)right);
            (void)UdpTelemetryQueueExperimentText(text);
        } else {
            (void)UdpTelemetryQueueExperimentText(
                "TRACE_SIDE event=READ_FAIL motor_l=0 motor_r=0");
        }

        osDelay(AppMsToTicks(CAR_LINE_CALIBRATION_PERIOD_MS));
    }
#elif (STM32_UART_LINK_TEST_MODE == 1)
    static const uint8_t motorFrame[6] =
    {
        0xFCU, 0x00U, 0x78U, 0x00U, 0x78U, 0xFDU
};
    uint32_t count = 0U;

    (void)argument;
    osDelay(AppMsToTicks(2000U));

    for (;;) {
        count++;
        (void)UartWrite(WIFI_IOT_UART_IDX_2,
                        (unsigned char *)motorFrame,
                        sizeof(motorFrame));
        printf("UARTTEST TX %u: FC 00 78 00 78 FD\r\n", (unsigned int)count);
        osDelay(AppMsToTicks(500U));
    }
#else
    WifiIotGpioValue left = WIFI_IOT_GPIO_VALUE0;
    WifiIotGpioValue right = WIFI_IOT_GPIO_VALUE0;
    Hcsr04Snapshot hcsr04Snapshot;
#if (CAR_LINE_CALIBRATION_MODE == 0)
    CarMode previousMode = CAR_MODE_IDLE;
    uint32_t traceModeStartTick = 0U;
    int traceModeStarted = 0;
    int sensorReadFailed = 0;
    (void)sensorReadFailed;
#endif
    (void)argument;

#if (ENCODER_DISTANCE_LEFT_ARC_ROUNDTRIP_TEST_MODE == 1)
    ArcRoundtripInit();
#endif
#if (ENCODER_DISTANCE_RIGHT_ARC_ROUNDTRIP_TEST_MODE == 1)
    RightArcRoundtripInit();
#endif
#if (ENCODER_TWO_ARC_CONTINUOUS_ROUNDTRIP_TEST_MODE == 1)
    TwoArcRoundtripInit();
#endif
#if (ENCODER_SEGMENT_B_PROGRESS_TEST_MODE == 1)
    BProgressInit();
#endif
#if (ENCODER_BPATH_FOLLOW_V1_TEST_MODE == 1) && (MOTOR_RESPONSE_TEST_MODE == 0) && \
    (LINE_SENSOR_SIDE_TEST_MODE == 0)
    BpathControlPublish("BPATHCTL event=READY state=DISARMED");
#if (AUTO_TRACE_BOOT_TEST_MODE == 1)
    BpathControlBeginRun(1U, 1U);
#else
    BPathFollowInit();
#endif
#endif
#if (MOTOR_RESPONSE_TEST_MODE == 1)
    MotorResponseInit();
#endif

    /* One stop frame at startup; no repeated stop traffic while idle. */
    car_stop();

#if (THREE_ELEVEN_SEQUENCE_TEST_MODE == 1)
    printf("SEQ11CFG qualify_ms=%u long_gap_ms=%u third_wait_ms=%u finish_dir=G12_GT_G23 finish_ratio=3/2\r\n",
           (unsigned int)ELEVEN_QUALIFY_MIN_MS,
           (unsigned int)SEQ11_LONG_GAP_MS,
           (unsigned int)SEQ11_THIRD_WAIT_MAX_MS);
#endif

#if (CAR_LINE_CALIBRATION_MODE == 1)
    {
        WifiIotGpioValue previousLeft = WIFI_IOT_GPIO_VALUE0;
        WifiIotGpioValue previousRight = WIFI_IOT_GPIO_VALUE0;
        uint32_t lastHeartbeatTick = 0U;
        int hasPreviousSample = 0;

        printf("CAL line calibration mode\r\n");
        for (;;) {
            uint32_t now = osKernelGetTickCount();

            if (LineSensorRead(&left, &right) == 1) {
                if (hasPreviousSample == 0 || left != previousLeft || right != previousRight) {
                    printf("CAL LINE L=%d R=%d\r\n", (int)left, (int)right);
                    previousLeft = left;
                    previousRight = right;
                    hasPreviousSample = 1;
                    lastHeartbeatTick = now;
                } else if ((uint32_t)(now - lastHeartbeatTick) >=
                           AppMsToTicks(CAR_LINE_CALIBRATION_HEARTBEAT_MS)) {
                    printf("CAL steady L=%d R=%d\r\n", (int)left, (int)right);
                    lastHeartbeatTick = now;
                }
            }

            osDelay(AppMsToTicks(CAR_LINE_CALIBRATION_PERIOD_MS));
        }
    }
#else
    for (;;) {
        CarMode mode = CarControlGetMode();
        uint32_t now = osKernelGetTickCount();
        WifiIotGpioValue lineLiveRawLeft = WIFI_IOT_GPIO_VALUE0;
        WifiIotGpioValue lineLiveRawRight = WIFI_IOT_GPIO_VALUE0;
        int lineLiveSensorValid = LineSensorRead(&lineLiveRawLeft, &lineLiveRawRight);
        uint8_t forkTestClaimedThisLoop = 0U;

        /* One fresh owner-loop read drives the independent, always-on view. */
        LineLiveUpdateSensor(lineLiveRawLeft, lineLiveRawRight, lineLiveSensorValid);
        LineLivePublish(now);
#if (TRACE_OBSERVER_TEST_MODE == 1) && (ENCODER_BPATH_FOLLOW_V1_TEST_MODE == 1) && \
    (LINE_SENSOR_SIDE_TEST_MODE == 0)
        {
            int observerWasActive = g_traceObserverActive;

            TraceObserverConsumeCommand();
            if (g_traceObserverActive != 0) {
                WifiIotGpioValue observerRawLeft = WIFI_IOT_GPIO_VALUE0;
                WifiIotGpioValue observerRawRight = WIFI_IOT_GPIO_VALUE0;

                if (g_bpathPendingCommand != BPATH_CONTROL_COMMAND_NONE) {
                    TraceObserverStop("BPATH_COMMAND");
                } else if (mode != CAR_MODE_TRACE) {
                    TraceObserverStop("MODE_CHANGE");
                    CarControlSetMode(CAR_MODE_IDLE);
                } else if (LineSensorRead(&observerRawLeft, &observerRawRight) != 1) {
                    TraceObserverStop("SENSOR_READ_FAIL");
                } else {
                    /* Pure observer: this function only reads sensor state and queues text. */
                    TraceObserverStep(now, observerRawLeft, observerRawRight);
                }
                /* TaskCarControl is the only motor owner; observer never owns a motor API. */
                EncoderExperimentSendMotorCommand(0, 0, now);
                osDelay(AppMsToTicks(CAR_CONTROL_PERIOD_MS));
                continue;
            }
            if (observerWasActive != 0) {
                EncoderExperimentSendMotorCommand(0, 0, now);
                osDelay(AppMsToTicks(CAR_CONTROL_PERIOD_MS));
                continue;
            }
        }
#endif
#if (TRACE_STEP_RESPONSE_TEST_MODE == 1)
        {
            WifiIotGpioValue stepRawLeft = WIFI_IOT_GPIO_VALUE0;
            WifiIotGpioValue stepRawRight = WIFI_IOT_GPIO_VALUE0;
            int stepLeft = 0;
            int stepRight = 0;
            int stepSensorValid = LineSensorRead(&stepRawLeft, &stepRawRight);

            /* This attended diagnostic owns every motor command while enabled. */
            TraceStepResponseStep(now, stepSensorValid, stepRawLeft, stepRawRight,
                                  &stepLeft, &stepRight);
            EncoderExperimentSendMotorCommand(stepLeft, stepRight, now);
            osDelay(AppMsToTicks(CAR_CONTROL_PERIOD_MS));
            continue;
        }
#endif
#if (TRACE_TRACK_GEOMETRY_TEST_MODE == 1)
        {
            WifiIotGpioValue geometryLeft = WIFI_IOT_GPIO_VALUE0;
            WifiIotGpioValue geometryRight = WIFI_IOT_GPIO_VALUE0;

            /* This owner-loop test wins over TRACE, BPATH, AVOID, and all motor tests. */
            TraceStaticTestForceStop();
            if (LineSensorRead(&geometryLeft, &geometryRight) == 1) {
                char text[176];

                /* Raw, unfiltered TRACE mapping: GPIO13/left and GPIO14/right. */
                (void)snprintf(text, sizeof(text),
                    "TRACE_GEOM ms=%u gpio13=%d gpio14=%d left=%d right=%d state=%d%d motor_l=0 motor_r=0",
                    (unsigned int)((uint64_t)now * 1000U / osKernelGetTickFreq()),
                    (int)geometryLeft, (int)geometryRight,
                    (int)geometryLeft, (int)geometryRight,
                    (int)geometryLeft, (int)geometryRight);
                (void)UdpTelemetryQueueExperimentText(text);
            } else {
                (void)UdpTelemetryQueueExperimentText(
                    "TRACE_GEOM event=READ_FAIL motor_l=0 motor_r=0");
            }
            osDelay(AppMsToTicks(CAR_LINE_CALIBRATION_PERIOD_MS));
            continue;
        }
#endif
#if (MOTOR_TURN_DIRECTION_TEST_MODE == 1)
        {
            int turnLeft = 0;
            int turnRight = 0;
            MotorTurnDirectionCommand command = g_motorTurnDirectionPendingCommand;

            g_motorTurnDirectionPendingCommand = MOTOR_TURN_DIRECTION_COMMAND_NONE;
            MotorTurnDirectionStep(now, command, &turnLeft, &turnRight);
            EncoderExperimentSendMotorCommand(turnLeft, turnRight, now);
            osDelay(AppMsToTicks(CAR_CONTROL_PERIOD_MS));
            continue;
        }
#endif
#if (MOTOR_RESPONSE_TEST_MODE == 1)
        {
            int motorCalLeft = 0;
            int motorCalRight = 0;
            MotorResponseCommand command = g_motorResponsePendingCommand;

            g_motorResponsePendingCommand = MOTOR_RESPONSE_COMMAND_NONE;
            MotorResponseStep(now, command, &motorCalLeft, &motorCalRight);
            EncoderExperimentSendMotorCommand(motorCalLeft, motorCalRight, now);
            osDelay(AppMsToTicks(CAR_CONTROL_PERIOD_MS));
            continue;
        }
#endif
#if (ENCODER_BPATH_FOLLOW_V1_TEST_MODE == 1) && (LINE_SENSOR_SIDE_TEST_MODE == 0)
        BpathControlConsumeCommand(now);
        BpathControlSyncSensorSemantic();
#if (FORK_BACKTRACK_PROOF_TEST_MODE == 1)
        /* Fresh lineLiveRaw* was sampled above; MARK is consumed only here in
         * the motor owner so it can arbitrate this iteration's classifier. */
        forkTestClaimedThisLoop = ForkTestConsumeCommand();
#endif
#if (PRE_E1_CURVE_GUIDED_REENTRY_TEST_MODE == 1)
        ReentryTestAutoStart(now);
#endif
        if (g_bpathControlState != BPATH_CONTROL_TRACE_RECORD) {
            CrossbarObserverObserveNotTrace(now, lineLiveRawLeft, lineLiveRawRight);
        }
        if (g_bpathControlState == BPATH_CONTROL_DISARMED ||
            g_bpathControlState == BPATH_CONTROL_DONE) {
            EncoderExperimentSendMotorCommand(0, 0, now);
            if (g_bpathControlState == BPATH_CONTROL_DONE) {
                LineLiveSetControl("DONE", "DONE", 0, 0);
            } else {
                LineLiveSetControl("DISARMED", "STOP", 0, 0);
            }
            osDelay(AppMsToTicks(CAR_CONTROL_PERIOD_MS));
            continue;
        }
#if (PRE_E1_CURVE_GUIDED_REENTRY_TEST_MODE == 1)
        if (g_bpathControlState == BPATH_CONTROL_REENTRY_ANCHOR_SETTLE) {
            /* No recorder or candidate motor owner is active until physical
             * reverse coast has settled at the new recovery anchor. */
            ReentryAnchorSettleStep(now);
            osDelay(AppMsToTicks(CAR_CONTROL_PERIOD_MS));
            continue;
        }
        if (g_bpathControlState == BPATH_CONTROL_REENTRY_LEFT_ESTABLISH ||
            g_bpathControlState == BPATH_CONTROL_REENTRY_APPROACH ||
            g_bpathControlState == BPATH_CONTROL_REENTRY_DEPART ||
            g_bpathControlState == BPATH_CONTROL_REENTRY_FORCE ||
            g_bpathControlState == BPATH_CONTROL_REENTRY_LINE_SWEEP) {
            /* Use this loop's same fresh raw snapshot.  No ElevenEvent or
             * SEQ11 consumer runs while the attended reentry experiment owns
             * the control loop. */
            ReentryTestStep(now, lineLiveRawLeft, lineLiveRawRight, lineLiveSensorValid);
            osDelay(AppMsToTicks(CAR_CONTROL_PERIOD_MS));
            continue;
        }
        if (g_bpathControlState == BPATH_CONTROL_RECOVERY_RETURN_SETTLE ||
            g_bpathControlState == BPATH_CONTROL_RECOVERY_RETURN) {
            /* Recovery-only reverse of the scratch path.  It owns the loop,
             * so neither normal TRACE nor SEQ11 can consume repeated 11s. */
            ReentryRecoveryReturnStep(now);
            osDelay(AppMsToTicks(CAR_CONTROL_PERIOD_MS));
            continue;
        }
#endif
#if (FORK_BACKTRACK_PROOF_TEST_MODE == 1)
        if (g_bpathControlState == BPATH_CONTROL_FORK_READY) {
            EncoderExperimentSendMotorCommand(0, 0, now);
            if (g_autoForkReturn.state == AUTO_FORK_READY_REENTRY) {
                LineLiveSetControl("READY_REENTRY", "STOP", 0, 0);
            } else {
                LineLiveSetControl("FORK_READY", "STOP", 0, 0);
            }
            osDelay(AppMsToTicks(CAR_CONTROL_PERIOD_MS));
            continue;
        }
        if (g_bpathControlState == BPATH_CONTROL_FORK_SELECT) {
            ForkTestSelectStep(now, g_lineLiveStableState);
            if (g_bpathControlState == BPATH_CONTROL_FORK_SELECT) {
                LineLiveSetControl("FORK_SELECT", "SELECT", g_motorLeftCommand,
                                   g_motorRightCommand);
            } else {
                LineLiveSetControl("DONE", "DONE", 0, 0);
            }
            osDelay(AppMsToTicks(CAR_CONTROL_PERIOD_MS));
            continue;
        }
#endif
        if (g_bpathControlState == BPATH_CONTROL_RETURN_SETTLE) {
            if (g_manualReturnPipelineActive == 0U) {
                BpathControlReturnGuardBlock("RETURN_SETTLE", "RETURN_SETTLE");
                EncoderExperimentSendMotorCommand(0, 0, now);
                osDelay(AppMsToTicks(CAR_CONTROL_PERIOD_MS));
                continue;
            }
            if (BPathExternalRecordStep(now) != 0) {
                BpathControlAbort("RETURN_SETTLE_RECORD_FAILURE");
            }
            EncoderExperimentSendMotorCommand(0, 0, now);
            LineLiveSetControl("RETURN_SETTLE", "RETURN_SETTLE", 0, 0);
            if (g_bpathAutoBootRun != 0U) {
                TraceLivePublish(now, "--", "RETURN", 0, 0, "RETURN",
                                 TRACE_LIVE_RETURN_PERIOD_MS, 0);
            }
            if (g_bpathControlState == BPATH_CONTROL_RETURN_SETTLE &&
                BPathExternalReturnSettleComplete(now) != 0) {
                if (BPathExternalRecordFinish(now) != 0 ||
                    BPathExternalReturnStart(now) != 0) {
                    BpathControlAbort("REFERENCE_FAILURE");
                } else {
                    g_bpathControlState = BPATH_CONTROL_BPATH_RETURN;
                    LineLiveSetControl("RETURN", "RETURN", 0, 0);
                    if (g_autoForkReturn.state == AUTO_FORK_ARMED) {
                        (void)AutoForkPrepareReturn();
                    } else {
#if (FORK_BACKTRACK_PROOF_TEST_MODE == 1)
                        (void)ForkTestPrepareReturn();
#endif
                    }
                }
            }
            osDelay(AppMsToTicks(CAR_CONTROL_PERIOD_MS));
            continue;
        }
        if (g_bpathControlState == BPATH_CONTROL_BPATH_RETURN) {
            if (g_manualReturnPipelineActive == 0U) {
                BpathControlReturnGuardBlock("BPATH_RETURN", "BPATH_RETURN");
                EncoderExperimentSendMotorCommand(0, 0, now);
                osDelay(AppMsToTicks(CAR_CONTROL_PERIOD_MS));
                continue;
            }
            int returnLeft = 0;
            int returnRight = 0;
            uint32_t terminalDelayMs = 0U;

            BPathFollowStep(now, &returnLeft, &returnRight);
            if (AutoForkObserveReturn() != 0) {
                returnLeft = 0;
                returnRight = 0;
                g_manualReturnPipelineActive = 0U;
                g_autoForkReturn.state = AUTO_FORK_READY_REENTRY;
                ReentryAnchorSettleBegin(now, 0U);
                printf("AUTOFORK event=READY_REENTRY e1=%u backoff=%u\r\n",
                       (unsigned int)g_autoForkReturn.eventId,
                       (unsigned int)g_autoForkReturn.backoffProgress);
            }
            if (g_bpathControlState == BPATH_CONTROL_FORK_READY &&
                g_autoForkReturn.state == AUTO_FORK_READY_REENTRY) {
                EncoderExperimentSendMotorCommand(0, 0, now);
                LineLiveSetControl("READY_REENTRY", "STOP", 0, 0);
                osDelay(AppMsToTicks(CAR_CONTROL_PERIOD_MS));
                continue;
            }
#if (FORK_BACKTRACK_PROOF_TEST_MODE == 1)
            if (ForkTestObserveReturn() != 0) {
                returnLeft = 0;
                returnRight = 0;
                g_manualReturnPipelineActive = 0U;
                g_forkTestState = FORKTEST_READY_REENTRY;
                g_forkTestResult = "READY";
                g_bpathControlState = BPATH_CONTROL_FORK_READY;
            }
            if (g_bpathControlState == BPATH_CONTROL_FORK_READY) {
                EncoderExperimentSendMotorCommand(0, 0, now);
                LineLiveSetControl("FORK_READY", "STOP", 0, 0);
                osDelay(AppMsToTicks(CAR_CONTROL_PERIOD_MS));
                continue;
            }
#endif
            if (BPathFollowTakeTerminalExecutionRequest(&terminalDelayMs) != 0) {
                uint32_t waitStartUs;
                uint32_t actualWaitUs;
                uint32_t actualStopMs;
                uint32_t actualStopTick;

                EncoderExperimentSendMotorCommand(returnLeft, returnRight, now);
                waitStartUs = (uint32_t)hi_get_us();
                hi_udelay(terminalDelayMs * 1000U);
                actualWaitUs = (uint32_t)((uint32_t)hi_get_us() - waitStartUs);
                actualStopMs = hi_get_milli_seconds();
                actualStopTick = osKernelGetTickCount();
                EncoderExperimentSendMotorCommand(0, 0, actualStopTick);
                BPathFollowNotifyTerminalStopExecuted(actualStopMs, actualWaitUs);
                LineLiveSetControl("RETURN", "RETURN", 0, 0);
            } else {
                EncoderExperimentSendMotorCommand(returnLeft, returnRight, now);
                LineLiveSetControl("RETURN", "RETURN", returnLeft, returnRight);
            }
            if (g_bpathAutoBootRun != 0U) {
                TraceLivePublish(now, "--", "RETURN", returnLeft, returnRight, "RETURN",
                                 TRACE_LIVE_RETURN_PERIOD_MS, 0);
            }
            if (g_bpathControlState == BPATH_CONTROL_BPATH_RETURN && BPathFollowIsFinished() != 0) {
                BpathControlFinish();
                if (g_bpathAutoBootRun != 0U) {
                    TraceLivePublish(now, "--", "DONE", 0, 0, "DONE", 0U, 1);
                }
            }
            osDelay(AppMsToTicks(CAR_CONTROL_PERIOD_MS));
            continue;
        }
#endif
#if (ENCODER_ONLY_EXPERIMENT_MODE == 1)
        {
            int experimentLeft = 0;
            int experimentRight = 0;
#if (ENCODER_BPATH_FOLLOW_V1_TEST_MODE == 1)
            uint32_t terminalDelayMs = 0U;
            BpathControlConsumeCommand(now);
            if (g_bpathControlState == BPATH_CONTROL_BPATH_RETURN) {
                BPathFollowStep(now, &experimentLeft, &experimentRight);
            }
            if (g_bpathControlState == BPATH_CONTROL_BPATH_RETURN &&
                BPathFollowTakeTerminalExecutionRequest(&terminalDelayMs) != 0) {
                uint32_t waitStartUs;
                uint32_t actualWaitUs;
                uint32_t actualStopMs;
                uint32_t actualStopTick;

                /* TaskCarControl remains the sole motor owner. The only
                 * sub-cycle exception sends the frozen command, waits once,
                 * then sends the coordinated 0/0 terminal stop. */
                EncoderExperimentSendMotorCommand(experimentLeft, experimentRight, now);
                waitStartUs = (uint32_t)hi_get_us();
                hi_udelay(terminalDelayMs * 1000U);
                actualWaitUs = (uint32_t)((uint32_t)hi_get_us() - waitStartUs);
                actualStopMs = hi_get_milli_seconds();
                actualStopTick = osKernelGetTickCount();
                EncoderExperimentSendMotorCommand(0, 0, actualStopTick);
                BPathFollowNotifyTerminalStopExecuted(actualStopMs, actualWaitUs);
            } else {
                EncoderExperimentSendMotorCommand(experimentLeft, experimentRight, now);
            }
            if (g_bpathControlState == BPATH_CONTROL_BPATH_RETURN && BPathFollowIsFinished() != 0) {
                BpathControlFinish();
            }
#elif (ENCODER_SEGMENT_B_PROGRESS_TEST_MODE == 1)
            BProgressStep(now, &experimentLeft, &experimentRight);
#elif (ENCODER_TWO_ARC_CONTINUOUS_ROUNDTRIP_TEST_MODE == 1)
            TwoArcRoundtripStep(now, &experimentLeft, &experimentRight);
#elif (ENCODER_DISTANCE_RIGHT_ARC_ROUNDTRIP_TEST_MODE == 1)
            RightArcRoundtripStep(now, &experimentLeft, &experimentRight);
#elif (ENCODER_DISTANCE_LEFT_ARC_ROUNDTRIP_TEST_MODE == 1)
            ArcRoundtripStep(now, &experimentLeft, &experimentRight);
#else
            EncoderCalStep(now, &experimentLeft, &experimentRight);
#endif
#if (ENCODER_BPATH_FOLLOW_V1_TEST_MODE != 1)
            EncoderExperimentSendMotorCommand(experimentLeft, experimentRight, now);
#endif
            osDelay(AppMsToTicks(CAR_CONTROL_PERIOD_MS));
            continue;
        }
#endif

#if (CROSS_AND_PROBE_TEST_MODE == 1)
        /* This experiment owns the control loop regardless of high-level CarMode. */
        if (g_probeBootLogged == 0U) {
            printf("PROBE event=BOOT test_mode=1 car_mode=%u stable_sensor=%u%u\r\n",
                   (unsigned int)mode, (unsigned int)((g_stableState >> 1) & 1U),
                   (unsigned int)(g_stableState & 1U));
            g_probeBootLogged = 1U;
        }
        if (LineSensorRead(&left, &right) != 1) {
            TraceResetDebounce();
            TraceSendMotorCommand(0, 0, 0);
        } else {
            int probeStableChanged = TraceUpdateStableState(left, right);
            if (g_stableStateValid != 0) {
                ProbeControlStep(left, right, now, probeStableChanged);
            }
        }
        osDelay(AppMsToTicks(CAR_CONTROL_PERIOD_MS));
        continue;
#endif

        if (mode != previousMode) {
            if (previousMode == CAR_MODE_AVOID && mode != CAR_MODE_AVOID) {
                /* AVOID relinquishes the motor with one explicit stop frame. */
                g_lastAvoidAction = AVOID_ACTION_FORWARD;
                AvoidApplyAction(AVOID_ACTION_STOP);
            }
            TraceResetDebounce();
            TraceResetRecovery();
            g_lastCorrection = TRACE_CORRECTION_NONE;
            sensorReadFailed = 0;
            traceModeStarted = 0;
            traceModeStartTick = now;
#if (TRACE_REVERSE_TEST_MODE == 0) && (TRACE_REVERSE_V2_TEST_MODE == 0) && \
    (TRACE_REVERSE_V3_TEST_MODE == 0) && (TRACE_REVERSE_V4_TEST_MODE == 0)
            g_lastBleStatTick = now;
#endif

            if (mode == CAR_MODE_TRACE) {
                /* Force the first TRACE command after mode entry to resync STM32. */
                g_motorCommandValid = 0;
                g_motorHeartbeatCount = 0U;
#if (TRACE_SHARP_TURN_DIAG_MODE == 1)
                TraceDiagReset(now);
#endif
#if (TRACE_RACE_TEST_MODE == 1)
                RaceReset(now);
                printf("RACE wait start marker\r\n");
#endif
#if (TRACE_REVERSE_TEST_MODE == 1)
                ReverseTraceReset(now);
                printf("REVERSE test armed\r\n");
#endif
#if (TRACE_REVERSE_V2_TEST_MODE == 1)
                ReverseV2Reset(now);
                printf("REVERSEV2 test armed\r\n");
#endif
#if (TRACE_REVERSE_V3_TEST_MODE == 1)
                ReverseV3Reset(now);
                printf("REVERSEV3 test armed\r\n");
#endif
#if (TRACE_REVERSE_V4_TEST_MODE == 1)
                ReverseV4Reset(now);
                printf("REVERSEV4 test armed\r\n");
#endif
#if (TRACE_REVERSE_V5_TEST_MODE == 1)
                ReverseV5Reset(now);
                printf("REVERSEV5 test armed\r\n");
                printf("=== ACTIVE CAR MODE: REVERSE V5 ===\r\n");
                printf("V5 BACK = -100 -100\r\n");
                printf("V5 HEAD LEFT = -120 -60\r\n");
                printf("V5 HEAD RIGHT = -60 -120\r\n");
                printf("BUILD MODE: REVERSE_V5 %s %s\r\n", __DATE__, __TIME__);
#endif
#if (TRACE_REVERSE_V6_TEST_MODE == 1)
                ReverseV6Reset(now);
#endif
#if (TRACE_REVERSE_V7_TEST_MODE == 1)
                ReverseV7Reset(now);
#endif
#if (TRACE_REVERSE_V8_TEST_MODE == 1)
                ReverseV8Reset(now);
                printf("REVERSEV8 test armed\r\n");
#endif
#if (REVERSE_REPLAY_STRAIGHT_TEST_MODE == 1)
                ReplayReset(now);
                printf("REPLAY wait stable 00\r\n");
#endif
#if (CROSS_AND_PROBE_TEST_MODE == 1)
                ProbeReset(now);
                printf("CROSS_AND_PROBE_V1 armed\r\n");
#endif
#if (CAR_TRACE_TEST_MODE == 1)
                printf("TRACE test starts in 2 seconds\r\n");
#else
                traceModeStarted = 1;
                printf("TRACE mode start\r\n");
#endif
            } else if (mode == CAR_MODE_AVOID) {
                g_avoidMotorCommandValid = 0;
                g_lastAvoidAction = AVOID_ACTION_FORWARD;
                AvoidApplyAction(AVOID_ACTION_STOP);
                printf("AVOID mode start\r\n");
            } else {
                TraceApplyAction(TRACE_ACTION_STOP);
            }
            previousMode = mode;
        }

        if (mode == CAR_MODE_TRACE) {
#if (CAR_TRACE_TEST_MODE == 1)
            if (traceModeStarted == 0) {
                if (g_bpathAutoBootRun == 0U &&
                    (uint32_t)(now - traceModeStartTick) < AppMsToTicks(TRACE_START_DELAY_MS)) {
                    osDelay(AppMsToTicks(CAR_CONTROL_PERIOD_MS));
                    continue;
                }
                traceModeStarted = 1;
                printf("TRACE mode start\r\n");
            }
#endif
#if (TRACE_RACE_TEST_MODE == 1)
            if (LineSensorRead(&left, &right) != 1) {
                TraceResetDebounce();
                AutoReturn11Reset();
                TraceResetState11Counter();
                TraceApplyAction(TRACE_ACTION_STOP);
                if (sensorReadFailed == 0) {
                    printf("TRACE sensor read failed\r\n");
                    sensorReadFailed = 1;
                }
            } else {
                sensorReadFailed = 0;
                WifiIotGpioValue rawLeft = left;
                WifiIotGpioValue rawRight = right;
                (void)rawLeft; (void)rawRight;
                int stableChanged = TraceUpdateStableState(left, right);
                if (stableChanged != 0) {
                    printf("TRACE sensor %u%u\r\n",
                           (unsigned int)((g_stableState >> 1) & 0x01U),
                           (unsigned int)(g_stableState & 0x01U));
                }
                if (g_stableStateValid != 0) {
                    RaceControlStep(rawLeft, rawRight, now);
                    TraceUpdateUdpTelemetry();
                }
            }
            if (g_raceState != RACE_FINISH_STOPPED &&
                g_raceState != RACE_DEADEND_STOPPED) {
                TraceHeartbeatIfDue(now);
            }
#else
#if (CROSS_AND_PROBE_TEST_MODE == 1)
            if (LineSensorRead(&left, &right) != 1) {
                TraceResetDebounce(); ProbeStop(now, "INCONCLUSIVE_GUARD", left, right);
            } else {
                sensorReadFailed = 0;
                int stableChanged = TraceUpdateStableState(left, right);
                if (g_stableStateValid != 0) ProbeControlStep(left, right, now, stableChanged);
            }
#elif (TRACE_REVERSE_TEST_MODE == 1)
            if (LineSensorRead(&left, &right) != 1) {
                TraceResetDebounce();
                ReverseTraceApplyAction(REVERSE_ACTION_STOP);
                if (sensorReadFailed == 0) {
                    printf("REVERSE sensor read failed\r\n");
                    sensorReadFailed = 1;
                }
            } else {
                sensorReadFailed = 0;
                if (TraceUpdateStableState(left, right) != 0) {
                    printf("REVERSE sensor %u%u\r\n",
                           (unsigned int)((g_stableState >> 1) & 0x01U),
                           (unsigned int)(g_stableState & 0x01U));
                }
                if (g_stableStateValid != 0) {
                    ReverseTraceControlStep(left, right, now);
                }
            }
            if (g_reverseAction != REVERSE_ACTION_STOP) {
                TraceHeartbeatIfDue(now);
            }
#elif (TRACE_REVERSE_V2_TEST_MODE == 1)
            if (LineSensorRead(&left, &right) != 1) {
                TraceResetDebounce();
                ReverseV2ApplyState(REV2_STOPPED);
                if (sensorReadFailed == 0) {
                    printf("REVERSEV2 sensor read failed\r\n");
                    sensorReadFailed = 1;
                }
            } else {
                sensorReadFailed = 0;
                if (TraceUpdateStableState(left, right) != 0) {
                    printf("REVERSEV2 sensor %u%u\r\n",
                           (unsigned int)((g_stableState >> 1) & 0x01U),
                           (unsigned int)(g_stableState & 0x01U));
                }
                if (g_stableStateValid != 0) {
                    ReverseV2ControlStep(left, right, now);
                }
            }
            if (g_reverseV2State != REV2_STOPPED) {
                TraceHeartbeatIfDue(now);
            }
#elif (TRACE_REVERSE_V3_TEST_MODE == 1)
            if (LineSensorRead(&left, &right) != 1) {
                TraceResetDebounce();
                ReverseV3ApplyAction(REV3_ACTION_STOP);
                if (sensorReadFailed == 0) {
                    printf("REVERSEV3 sensor read failed\r\n");
                    sensorReadFailed = 1;
                }
            } else {
                sensorReadFailed = 0;
                if (TraceUpdateStableState(left, right) != 0) {
                    printf("REVERSEV3 sensor %u%u\r\n",
                           (unsigned int)((g_stableState >> 1) & 0x01U),
                           (unsigned int)(g_stableState & 0x01U));
                }
                if (g_stableStateValid != 0) {
                    ReverseV3ControlStep(left, right, now);
                }
            }
            if (g_reverseV3Action != REV3_ACTION_STOP) {
                TraceHeartbeatIfDue(now);
            }
#elif (TRACE_REVERSE_V4_TEST_MODE == 1)
            if (LineSensorRead(&left, &right) != 1) {
                TraceResetDebounce();
                ReverseV4ApplyState(REV4_STOPPED);
                if (sensorReadFailed == 0) {
                    printf("REVERSEV4 sensor read failed\r\n");
                    sensorReadFailed = 1;
                }
            } else {
                sensorReadFailed = 0;
                if (TraceUpdateStableState(left, right) != 0) {
                    printf("REVERSEV4 sensor %u%u\r\n",
                           (unsigned int)((g_stableState >> 1) & 0x01U),
                           (unsigned int)(g_stableState & 0x01U));
                }
                if (g_stableStateValid != 0) {
                    ReverseV4ControlStep(left, right, now);
                }
            }
            if (g_reverseV4State != REV4_STOPPED) {
                TraceHeartbeatIfDue(now);
            }
#elif (TRACE_REVERSE_V5_TEST_MODE == 1)
            if (LineSensorRead(&left, &right) != 1) {
                TraceResetDebounce();
                ReverseV5Apply(REV5_STOPPED);
            } else {
                if (TraceUpdateStableState(left, right) != 0 || g_stableStateValid != 0) {
                    if (g_stableStateValid != 0) ReverseV5ControlStep(left, right, now);
                }
                TraceHeartbeatIfDue(now);
            }
#elif (TRACE_REVERSE_V6_TEST_MODE == 1)
            if (LineSensorRead(&left, &right) != 1) {
                TraceResetDebounce();
                ReverseV6Apply(REV6_STOPPED);
            } else {
                if (TraceUpdateStableState(left, right) != 0 || g_stableStateValid != 0) {
                    if (g_stableStateValid != 0) ReverseV6ControlStep(left, right, now);
                }
                TraceHeartbeatIfDue(now);
            }
#elif (TRACE_REVERSE_V7_TEST_MODE == 1)
            if (LineSensorRead(&left, &right) != 1) {
                TraceResetDebounce();
                ReverseV7Apply(REV7_STOPPED);
            } else {
                if (TraceUpdateStableState(left, right) != 0 || g_stableStateValid != 0) {
                    if (g_stableStateValid != 0) ReverseV7ControlStep(left, right, now);
                }
                TraceHeartbeatIfDue(now);
            }
#elif (REVERSE_REPLAY_STRAIGHT_TEST_MODE == 1)
            if (LineSensorRead(&left, &right) != 1) {
                TraceResetDebounce();
                g_replayRecordActive = 0;
                g_replayState = REPLAY_ERROR;
                TraceApplyAction(TRACE_ACTION_STOP);
                if (sensorReadFailed == 0) {
                    printf("REPLAY sensor read failed\r\n");
                    sensorReadFailed = 1;
                }
            } else {
                sensorReadFailed = 0;
                if (TraceUpdateStableState(left, right) != 0) {
                    printf("REPLAY sensor %u%u\r\n",
                           (unsigned int)((g_stableState >> 1) & 0x01U),
                           (unsigned int)(g_stableState & 0x01U));
                }
                if (g_stableStateValid != 0) {
                    ReplayControlStep(left, right, now);
                }
            }
            if (ReplayIsMoving() != 0) {
                TraceHeartbeatIfDue(now);
            }
#elif (TRACE_REVERSE_V8_TEST_MODE == 1)
            if (LineSensorRead(&left, &right) != 1) {
                TraceResetDebounce();
#if (TRACE_REVERSE_V8_PAIRED_RECENTER_MODE == 1)
                ReverseV8Apply(REV8_PAIR_STOPPED);
#elif (TRACE_REVERSE_V8_BIASED_MICRO_MODE == 1)
                ReverseV8Apply(REV8_BIASED_STOPPED);
#elif (TRACE_REVERSE_V8_MICRO_PAIRED_MODE == 1)
                ReverseV8Apply(REV8_MICRO_STOPPED);
#else
                ReverseV8Apply(REV8_STOPPED);
#endif
                if (sensorReadFailed == 0) {
                    printf("REVERSEV8 sensor read failed\r\n");
                    sensorReadFailed = 1;
                }
            } else {
                sensorReadFailed = 0;
                if (TraceUpdateStableState(left, right) != 0) {
                    printf("REVERSEV8 sensor %u%u\r\n",
                           (unsigned int)((g_stableState >> 1) & 0x01U),
                           (unsigned int)(g_stableState & 0x01U));
                }
                if (g_stableStateValid != 0) {
                    ReverseV8ControlStep(left, right, now);
                }
#if (TRACE_REVERSE_V8_PAIRED_RECENTER_MODE == 1)
                if (g_reverseV8State != REV8_PAIR_STOPPED) {
#elif (TRACE_REVERSE_V8_BIASED_MICRO_MODE == 1)
                if (g_reverseV8State != REV8_BIASED_STOPPED) {
#elif (TRACE_REVERSE_V8_MICRO_PAIRED_MODE == 1)
                if (g_reverseV8State != REV8_MICRO_STOPPED) {
#else
                if (g_reverseV8State != REV8_STOPPED) {
#endif
                    TraceHeartbeatIfDue(now);
                }
            }
#else
#if (CAR_STRAIGHT_DIAGNOSTIC_MODE == 1)
            if (LineSensorRead(&left, &right) != 1) {
                TraceApplyAction(TRACE_ACTION_STOP);
                if (sensorReadFailed == 0) {
                    printf("TRACE sensor read failed\r\n");
                    sensorReadFailed = 1;
                }
            } else {
                sensorReadFailed = 0;
                TraceApplyAction(TRACE_ACTION_FORWARD);
            }
            TraceHeartbeatIfDue(now);
            osDelay(AppMsToTicks(CAR_CONTROL_PERIOD_MS));
            continue;
#endif
            int traceDiagRawLeft = -1;
            int traceDiagRawRight = -1;
            if (lineLiveSensorValid == 0) {
                CrossbarObserverStop(now, "SENSOR_READ_FAILURE");
                Long11ObserverReset();
                RightStoplineCandidateObserverReset();
                DoubleStopAutoReturnObserverReset();
                ElevenEventReset();
                ThreeElevenResetForTrace();
                TraceResetDebounce();
                TraceApplyAction(TRACE_ACTION_STOP);
                LineLiveSetControl("TRACE", "STOP", 0, 0);
                if (sensorReadFailed == 0) {
                    printf("TRACE sensor read failed\r\n");
                    sensorReadFailed = 1;
                }
            } else {
                left = lineLiveRawLeft;
                right = lineLiveRawRight;
                sensorReadFailed = 0;
                WifiIotGpioValue rawLeft = left;
                WifiIotGpioValue rawRight = right;
                uint8_t freshRawState = (uint8_t)((rawLeft == WIFI_IOT_GPIO_VALUE1 ? 2U : 0U) |
                                                  (rawRight == WIFI_IOT_GPIO_VALUE1 ? 1U : 0U));
                StoplineSemanticClaim stoplineClaim = STOPLINE_CLAIM_NONE;
                TrackSemanticAction semanticAction = TRACK_SEMANTIC_ACTION_NONE;
                ElevenEventSignal elevenSignal = ELEVEN_EVENT_SIGNAL_NONE;
                const ElevenEvent *elevenEvent = NULL;
                uint8_t newElevenEnterThisLoop = 0U;
                uint8_t elevenEnterForSeq = 0U;
                traceDiagRawLeft = (int)rawLeft;
                traceDiagRawRight = (int)rawRight;
                (void)rawLeft; (void)rawRight;
                int stableChanged = TraceUpdateStableState(left, right);
                if (stableChanged != 0) {
                    printf("TRACE sensor %u%u\r\n",
                           (unsigned int)((g_stableState >> 1) & 0x01U),
                           (unsigned int)(g_stableState & 0x01U));
                }

                if (g_stableStateValid != 0) {
                    left = ((g_stableState & 0x02U) != 0U) ?
                           WIFI_IOT_GPIO_VALUE1 : WIFI_IOT_GPIO_VALUE0;
                    right = ((g_stableState & 0x01U) != 0U) ?
                            WIFI_IOT_GPIO_VALUE1 : WIFI_IOT_GPIO_VALUE0;
#if (ENCODER_BPATH_FOLLOW_V1_TEST_MODE == 1)
                    if (g_bpathControlState == BPATH_CONTROL_TRACE_ARM &&
                        (g_bpathAutoBootRun == 0U ||
                         BpathControlEncoderReady() != 0) &&
                        BpathControlBeginTraceRecord(now) != 0) {
                        EncoderExperimentSendMotorCommand(0, 0, now);
                    }
                    if (g_bpathControlState == BPATH_CONTROL_TRACE_RECORD) {
#if (AUTO_RETURN_ON_11_TEST_MODE == 1)
                        if (g_bpathAutoBootRun != 0U && g_stableState == 0x03U) {
                            if (g_autoReturn11Active == 0U) {
                                g_autoReturn11Active = 1U;
                                g_autoReturn11StartTick = now;
                            }
                            if ((uint32_t)(now - g_autoReturn11StartTick) <
                                AppMsToTicks(AUTO_RETURN_11_CONFIRM_MS)) {
                                TraceResetRecovery();
                                g_lastCorrection = TRACE_CORRECTION_RIGHT;
                                TraceApplyAction(TRACE_ACTION_RIGHT);
                                TraceUpdateUdpTelemetry();
                                TraceLivePublish(now, "11", "RIGHT_11", 110, 90,
                                                 "TRACE", TRACE_LIVE_FORWARD_PERIOD_MS, 0);
                            } else {
                            if (BPathExternalHasForwardMovement() == 0) {
                                TraceApplyAction(TRACE_ACTION_STOP);
                                BPathExternalAbort("EMPTY_FORWARD_PATH");
                                TraceLivePublish(now, "11", "RETURN_TRIGGER", 0, 0,
                                                 "RETURN", 0U, 1);
                                BpathControlFinish();
                                TraceLivePublish(now, "--", "DONE", 0, 0,
                                                 "DONE", 0U, 1);
                            } else {
                                TraceApplyAction(TRACE_ACTION_STOP);
                                TraceLivePublish(now, "11", "RETURN_TRIGGER", 0, 0,
                                                 "RETURN", 0U, 1);
                                (void)BpathControlBeginReturn(now);
                            }
                            }
                        } else
#endif
                        {
                        AutoReturn11Reset();
#if (DOUBLE_STOP_AUTO_RETURN_TEST_MODE == 1)
                        /* TRACE, neutral event creation, independent consumers, resolver. */
                        TraceControlStep(rawLeft, rawRight, now);
                        if (g_sensorSemanticValid != 0U) {
                            elevenSignal = ElevenEventObserve(now, rawLeft, rawRight,
                                                              g_stableState, &elevenEvent);
                        } else {
                            elevenSignal = ELEVEN_EVENT_SIGNAL_NONE;
                            if (g_stableState == 0x03U) {
                                printf("ADMIN event=IGNORED_11 sensor_valid=0 phase=TRACE\r\n");
                            }
                        }
                        if (elevenSignal == ELEVEN_EVENT_SIGNAL_ENTER &&
                            g_startLineConsumed == 0U && elevenEvent != NULL) {
                            /* The first qualified marker belongs to the external
                             * route start, not to an internal BPATH/reentry epoch. */
                            g_startLineConsumed = 1U;
                            g_startLineEventId = elevenEvent->id;
                            printf("SEQ11EVT event=QUALIFY id=%u role=START_LINE seq_delivered=0\r\n",
                                   (unsigned int)elevenEvent->id);
                        } else if (elevenSignal == ELEVEN_EVENT_SIGNAL_ENTER) {
                            elevenEnterForSeq = 1U;
                        }
                        if (elevenSignal == ELEVEN_EVENT_SIGNAL_EXIT && elevenEvent != NULL &&
                            elevenEvent->id == g_startLineEventId) {
                            printf("SEQ11 event=START_LINE_IGNORED event_id=%u enter_ms=%u exit_ms=%u path_start=%u\r\n",
                                   (unsigned int)elevenEvent->id,
                                   (unsigned int)elevenEvent->enterMs,
                                   (unsigned int)elevenEvent->exitMs,
                                   (unsigned int)elevenEvent->forwardPathStartIndex);
                            /* ElevenEvent ids restart on an internal TRACE epoch;
                             * do not let this finished marker alias a later event. */
                            g_startLineEventId = 0U;
                        }
                        /* This terminal-only observer deliberately runs beside,
                         * rather than inside, ElevenEvent qualification: a pair
                         * of short stable-11 paint strokes must never manufacture
                         * semantic E1/E2 events. */
                        ClosePairTerminalObserve(now, g_stableState);
                        ForkClassifierObserve(elevenSignal, elevenEvent);
 #if (FORK_BACKTRACK_PROOF_TEST_MODE == 1)
                        if (forkTestClaimedThisLoop != 0U) {
                            /* Legacy attended-proof path only.  Automatic fork
                             * classification never mutates stopline state. */
                            StoplineCandidateCancel("FORK_CLAIM");
                        } else
 #endif
                        {
                            stoplineClaim = DoubleStopAutoReturnObserverObserve(
                                now, elevenSignal, elevenEvent, g_stableState, g_lastAction);
                            (void)stoplineClaim;
                        }
#if (THREE_ELEVEN_SEQUENCE_TEST_MODE == 1)
                        /* Sequence semantics consume only neutral ElevenEvent ENTERs,
                         * never the GPIO state directly. */
                        if (elevenEnterForSeq != 0U) {
                            newElevenEnterThisLoop = 1U;
                            ThreeElevenSequenceOnEnter(now, elevenEvent);
                        }
                        ThreeElevenSequenceTick(now, newElevenEnterThisLoop, freshRawState,
                                                g_elevenCandidate.active);
                        ThreeElevenObserveSingleStale(now, elevenSignal, elevenEvent,
                                                      freshRawState);
                        if (newElevenEnterThisLoop != 0U &&
                            g_threeEleven.claim == SEQ11_CLAIM_RETURN) {
                            /* Defensive invariant: an ENTER must always defeat
                             * a same-loop two-event timeout.  Do not panic. */
                            printf("SEQ11 ERROR event=RETURN_ON_NEW_ELEVEN\r\n");
                            g_threeEleven.claim = SEQ11_CLAIM_NONE;
                            g_threeEleven.paused = 0U;
                            ThreeElevenSyncLineLive();
                        }
#endif
                        /* In sequence-test mode this resolver deliberately ignores
                         * legacy stopline claims; old observer telemetry remains live. */
                        semanticAction = ResolveTrackSemanticDecision();
                        if (semanticAction == TRACK_SEMANTIC_ACTION_FINAL_STOP) {
                            /* Only a close pair of qualified ElevenEvents is a
                             * normal-route terminal; faults retain their own stops. */
                            TraceApplyAction(TRACE_ACTION_STOP);
                            BpathControlFinish();
                        } else if (semanticAction == TRACK_SEMANTIC_ACTION_BEGIN_RETURN) {
                            TraceApplyAction(TRACE_ACTION_STOP);
                            g_doubleStopReturnAuthorized = 1U;
                            g_doubleStopTestReturnActive = 1U;
                            g_returnTrigger = "AUTO_FORK_TWO_LONG";
                            if (BpathControlBeginReturn(now) == 0) {
                                TraceLivePublishManualReturnTrigger(now, g_returnTrigger);
                            }
                        } else
#endif
                        {
#if (TRACE_OPEN_LOOP_STRAIGHT_TEST_MODE == 1)
                        TraceOpenLoopStraightStep(traceDiagRawLeft, traceDiagRawRight, now);
#else
#if (DOUBLE_STOP_AUTO_RETURN_TEST_MODE == 0)
                        TraceControlStep(rawLeft, rawRight, now);
#endif
#endif
                        TraceCurveObserve(now, rawLeft, rawRight);
                        CrossbarRawForensicsObserve(now, rawLeft, rawRight);
                        /* Observer-only: it cannot alter TRACE/BPATH motor decisions. */
                        CrossbarObserverObserve(now, rawLeft, rawRight);
                        Pre11ContextObserve(now, rawLeft, rawRight);
                        Long11ObserverObserve(now, rawLeft, rawRight);
                        RightStoplineCandidateObserverObserve(now, rawLeft, rawRight);
                        /* Snapshot only: socket I/O is owned by UdpTelemetryTask. */
                        TraceUpdateUdpTelemetry();
                        {
                            char sensorText[3];
                            sensorText[0] = ((g_stableState & 0x02U) != 0U) ? '1' : '0';
                            sensorText[1] = ((g_stableState & 0x01U) != 0U) ? '1' : '0';
                            sensorText[2] = '\0';
                            TraceLivePublish(now, sensorText,
                                TraceRecordDiagActionName(g_lastAction),
                                g_motorLeftCommand, g_motorRightCommand, "TRACE",
                                TRACE_LIVE_FORWARD_PERIOD_MS, 0);
                        }
                        }
                        }
                    }
#else
                    TraceControlStep(left, right, now);
                    /* Snapshot only: socket I/O is owned by UdpTelemetryTask. */
                    TraceUpdateUdpTelemetry();
#endif
                }
#if (TRACE_SHARP_TURN_DIAG_MODE == 1)
                TraceDiagPoll(rawLeft, rawRight, stableChanged, now);
#endif
            }
            TraceHeartbeatIfDue(now);
            TraceBleDiagHeartbeat(now);
#if (ENCODER_BPATH_FOLLOW_V1_TEST_MODE == 1)
            if (g_bpathControlState == BPATH_CONTROL_TRACE_RECORD) {
                TraceRecordDiagPublish(traceDiagRawLeft, traceDiagRawRight, now);
                BpathControlUpdateIdleRolling();
                if (BPathExternalRecordStep(now) != 0) {
                    BpathControlRecoverFromForwardOverflow(now);
                }
            }
#endif
#endif
#endif
        } else if (mode == CAR_MODE_AVOID) {
            if (Hcsr04GetSnapshot(&hcsr04Snapshot) != 0) {
                uint32_t nowMs = AppTicksToMs(now);
                AvoidApplyAction(AvoidSelectAction(&hcsr04Snapshot, nowMs));
                AvoidUpdateUdpTelemetry(&hcsr04Snapshot, nowMs);
            } else {
                AvoidApplyAction(AVOID_ACTION_STOP);
            }
            AvoidHeartbeatIfDue(now);
        }
#if (CAR_IR_DIAGNOSTIC_MODE == 1)
        else if (LineSensorRead(&left, &right) == 1) {
            printf("LINE L=%d R=%d\r\n", (int)left, (int)right);
        }
#endif

        osDelay(AppMsToTicks(CAR_CONTROL_PERIOD_MS));
    }
#endif
#endif
}

int TaskCarControlInit(void)
{
    osThreadAttr_t attr;

    if (g_carControlTaskStarted != 0) {
        return 0;
    }

    attr.name = "car_control";
    attr.attr_bits = 0;
    attr.cb_mem = NULL;
    attr.cb_size = 0;
    attr.stack_mem = NULL;
    /* RETURN transition can synchronously format multiple forensic records. */
    attr.stack_size = 4096;
    attr.priority = osPriorityNormal;

    if (osThreadNew(CarControlTask, NULL, &attr) == NULL) {
        printf("car control thread create failed\r\n");
        return -1;
    }

    g_carControlTaskStarted = 1;

#if (LINE_SENSOR_ANALYSIS_MODE == 0) && (LINE_SENSOR_CALIBRATION_MODE == 0) && \
    (LINE_SENSOR_SIDE_TEST_MODE == 0) && \
    (CAR_LINE_CALIBRATION_MODE == 0) && (CAR_AVOID_TEST_MODE == 1)
    CarControlSetMode(CAR_MODE_AVOID);
#elif (LINE_SENSOR_ANALYSIS_MODE == 0) && (LINE_SENSOR_CALIBRATION_MODE == 0) && \
    (LINE_SENSOR_SIDE_TEST_MODE == 0) && \
    (CAR_LINE_CALIBRATION_MODE == 0) && (CAR_TRACE_TEST_MODE == 1)
    CarControlSetMode(CAR_MODE_TRACE);
#endif
    return 0;
}

void CarControlSetMode(CarMode mode)
{
#if (CAR_LINE_CALIBRATION_MODE == 1)
    (void)mode;
    g_carMode = CAR_MODE_IDLE;
    return;
#endif

    if (mode != CAR_MODE_TRACE && mode != CAR_MODE_AVOID) {
        g_carMode = CAR_MODE_IDLE;
        return;
    }

    g_carMode = mode;
}

CarMode CarControlGetMode(void)
{
    return g_carMode;
}

void CarControlSubmitBpathCommand(BpathControlCommand command)
{
    CarControlSubmitBpathCommandFromSource(command, BPATH_COMMAND_SOURCE_NONE);
}

void CarControlSubmitBpathCommandFromSource(BpathControlCommand command,
                                            BpathCommandSource source)
{
#if (MOTOR_TURN_DIRECTION_TEST_MODE == 1)
    if (command == BPATH_CONTROL_COMMAND_RESET) {
        g_motorTurnDirectionPendingCommand = MOTOR_TURN_DIRECTION_COMMAND_STOP;
    }
#elif (ENCODER_BPATH_FOLLOW_V1_TEST_MODE == 1) && (LINE_SENSOR_SIDE_TEST_MODE == 0)
    if (command == BPATH_CONTROL_COMMAND_START ||
        command == BPATH_CONTROL_COMMAND_RETURN ||
        command == BPATH_CONTROL_COMMAND_RESET) {
        g_bpathPendingCommand = command;
        g_bpathPendingSource = source;
    }
#else
    (void)command;
    (void)source;
#endif
}

void CarControlSubmitMotorTurnDirectionCommand(MotorTurnDirectionCommand command)
{
#if (MOTOR_TURN_DIRECTION_TEST_MODE == 1)
    if (command == MOTOR_TURN_DIRECTION_COMMAND_LEFT ||
        command == MOTOR_TURN_DIRECTION_COMMAND_RIGHT ||
        command == MOTOR_TURN_DIRECTION_COMMAND_FORWARD ||
        command == MOTOR_TURN_DIRECTION_COMMAND_STOP) {
        g_motorTurnDirectionPendingCommand = command;
    }
#else
    (void)command;
#endif
}

void CarControlSubmitTraceStepResponseCommand(TraceStepResponseCommand command)
{
#if (TRACE_STEP_RESPONSE_TEST_MODE == 1)
    if (command >= TRACE_STEP_RESPONSE_COMMAND_LEFT_100 &&
        command <= TRACE_STEP_RESPONSE_COMMAND_STOP) {
        g_traceStepResponsePendingCommand = command;
    }
#else
    (void)command;
#endif
}

void CarControlSubmitForkTestCommand(ForkTestCommand command)
{
#if (FORK_BACKTRACK_PROOF_TEST_MODE == 1) && \
    (ENCODER_BPATH_FOLLOW_V1_TEST_MODE == 1) && (LINE_SENSOR_SIDE_TEST_MODE == 0)
    if (command >= FORK_TEST_COMMAND_RESET && command <= FORK_TEST_COMMAND_GO) {
        g_forkTestPendingCommand = command;
    }
#else
    (void)command;
#endif
}


void CarControlSubmitTraceObserverCommand(TraceObserverCommand command)
{
#if (TRACE_OBSERVER_TEST_MODE == 1)
    if (command == TRACE_OBSERVER_COMMAND_START ||
        command == TRACE_OBSERVER_COMMAND_STOP) {
        g_traceObserverPendingCommand = command;
    }
#else
    (void)command;
#endif
}

void CarControlSubmitMotorResponseCommand(MotorResponseCommand command)
{
#if (MOTOR_RESPONSE_TEST_MODE == 1)
    if (command == MOTOR_RESPONSE_COMMAND_START ||
        command == MOTOR_RESPONSE_COMMAND_STOP) {
        g_motorResponsePendingCommand = command;
    }
#else
    (void)command;
#endif
}

int CarControlReverseV8TestModeEnabled(void)
{
#if (TRACE_REVERSE_V8_TEST_MODE == 1) || (REVERSE_REPLAY_STRAIGHT_TEST_MODE == 1)
    return 1;
#else
    return 0;
#endif
}

int CarControlEncoderOnlyExperimentModeEnabled(void)
{
#if (ENCODER_ONLY_EXPERIMENT_MODE == 1)
    return 1;
#else
    return 0;
#endif
}
