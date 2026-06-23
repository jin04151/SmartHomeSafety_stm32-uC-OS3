/*
*********************************************************************************************************
* RTOS Smart Home Safety System - app.c
* Based on HW #6 starter skeleton.
*
* Core flow:
*   EXTI ISR(gas/flame/PIR/button) -> semaphore / event flag -> task -> queue -> output/log
*
* NOTE:
*   1) Check pin mapping for your own board/wiring.
*   2) AppUsartTryGetLine() is intentionally compatible with HW5 style USART code.
*      Replace USARTx if your skeleton uses a different USART.
*********************************************************************************************************
*/

#include  <includes.h>
#include "stm32f4xx_tim.h"
#include  "stm32f4xx.h"
#include  "stm32f4xx_rcc.h"
#include  "stm32f4xx_gpio.h"
#include  "stm32f4xx_exti.h"
#include  "stm32f4xx_syscfg.h"
#include  "misc.h"

/* ---------------- Priority ---------------- */
#define APP_CFG_EMERGENCY_TASK_PRIO      3u
#define APP_CFG_SECURITY_TASK_PRIO       4u
#define APP_CFG_INPUT_TASK_PRIO          5u
#define APP_CFG_STATE_TASK_PRIO          6u
#define APP_CFG_USART_TASK_PRIO          7u
#define APP_CFG_OUTPUT_TASK_PRIO         8u
#define APP_CFG_MONITOR_TASK_PRIO        9u

#define APP_CFG_TASK_STK_SIZE            APP_CFG_TASK_START_STK_SIZE
#define APP_EVENT_Q_SIZE                 16u
#define APP_CMD_LINE_LEN                 40u

/* ---------------- EXTI input pins ----------------
 * Gas   D0 -> D8  / PF12, EXTI12
 * Flame DO -> D2  / PF15, EXTI15
 * PIR   DO -> D4  / PF14, EXTI14
 * Button   -> PC13, EXTI13
 */
#define GAS_GPIO_PORT                    GPIOF
#define GAS_GPIO_PIN                     GPIO_Pin_12      /* D8 / PF12 */
#define GAS_EXTI_LINE                    EXTI_Line12
#define GAS_EXTI_PORT_SRC                EXTI_PortSourceGPIOF
#define GAS_EXTI_PIN_SRC                 EXTI_PinSource12

#define FLAME_GPIO_PORT                  GPIOF
#define FLAME_GPIO_PIN                   GPIO_Pin_15      /* D2 / PF15 */
#define FLAME_EXTI_LINE                  EXTI_Line15
#define FLAME_EXTI_PORT_SRC              EXTI_PortSourceGPIOF
#define FLAME_EXTI_PIN_SRC               EXTI_PinSource15

#define PIR_GPIO_PORT                    GPIOF
#define PIR_GPIO_PIN                     GPIO_Pin_14      /* D4 / PF14 */
#define PIR_EXTI_LINE                    EXTI_Line14
#define PIR_EXTI_PORT_SRC                EXTI_PortSourceGPIOF
#define PIR_EXTI_PIN_SRC                 EXTI_PinSource14

#define BTN_GPIO_PORT                    GPIOC
#define BTN_GPIO_PIN                     GPIO_Pin_13
#define BTN_EXTI_LINE                    EXTI_Line13
#define BTN_EXTI_PORT_SRC                EXTI_PortSourceGPIOC
#define BTN_EXTI_PIN_SRC                 EXTI_PinSource13


/* ---------------- Output LED pins ----------------
 * Power LED R -> D15 / PB8 : 전력 차단 빨강
 * Power LED G -> D14 / PB9 : 전력 정상 초록
 *
 * Gas LED R   -> D12 / PA6 : 가스 차단 빨강
 * Gas LED G   -> D11 / PA7 : 가스 정상 초록
 */
#define POWER_LED_GPIO_PORT              GPIOB
#define POWER_LED_CUT_RED_PIN            GPIO_Pin_8       /* D15 / PB8 */
#define POWER_LED_NORMAL_GREEN_PIN       GPIO_Pin_9       /* D14 / PB9 */

#define GAS_LED_GPIO_PORT                GPIOA
#define GAS_LED_CUT_RED_PIN              GPIO_Pin_6       /* D12 / PA6 */
#define GAS_LED_NORMAL_GREEN_PIN         GPIO_Pin_7       /* D11 / PA7 */


#define BUZZER_GPIO_PORT                 GPIOE
#define BUZZER_GPIO_PIN                  GPIO_Pin_13    /* D3 / PE13 */

#define APP_BUTTON_DEBOUNCE_MS           50u

#define APP_FLAG_GAS                     0x01u
#define APP_FLAG_FLAME                   0x02u

/* ---------------- Types ---------------- */
typedef enum {
    APP_MODE_HOME = 0,
    APP_MODE_OUT,
    APP_MODE_EMERGENCY
} APP_MODE;

typedef enum {
    APP_EMG_NONE = 0,
    APP_EMG_GAS,
    APP_EMG_FLAME,
    APP_EMG_INTRUSION
} APP_EMERGENCY_REASON;

typedef enum {
    APP_EVENT_NONE = 0,
    APP_EVENT_REQ_HOME,
    APP_EVENT_REQ_OUT,
    APP_EVENT_REQ_CLEAR,
    APP_EVENT_GAS,
    APP_EVENT_FLAME,
    APP_EVENT_INTRUSION,
    APP_EVENT_CMD_STATUS,
    APP_EVENT_CMD_HELP
} APP_EVENT_TYPE;

typedef struct {
    APP_EVENT_TYPE Type;
    CPU_INT32U     Value;
} APP_EVENT;

/* ---------------- Prototypes ---------------- */
static void AppTaskStart     (void *p_arg);
static void AppTaskEmergency (void *p_arg);
static void AppTaskSecurity  (void *p_arg);
static void AppTaskInput     (void *p_arg);
static void AppTaskState     (void *p_arg);
static void AppTaskUsart     (void *p_arg);
static void AppTaskOutput    (void *p_arg);
static void AppTaskMonitor   (void *p_arg);

static void AppObjCreate     (void);
static void AppTaskCreate    (void);
static void AppGpioInit      (void);
static void AppExtiInit      (void);

static void AppExti15_10ISR  (void);

static void AppEventPost     (APP_EVENT_TYPE type, CPU_INT32U value);
static void AppCmdParse      (CPU_CHAR *line);
static void AppPrintHelp     (void);
static void AppPrintStatus   (void);
static void AppTrace         (const CPU_CHAR *msg);
static CPU_BOOLEAN AppUsartTryGetLine(CPU_CHAR *buf, CPU_INT16U buf_len);
static CPU_BOOLEAN AppStrEq  (const CPU_CHAR *a, const CPU_CHAR *b);

static void AppApplyOutputs  (void);
static void AppServoSetAngle (CPU_INT08U degree);
static CPU_BOOLEAN AppDangerStillActive(void);

static void Setup_Usart3(void);
static void Setup_Servo_PWM(void);
static void Setup_Usart6_BT(void);

/* ---------------- Kernel objects / globals ---------------- */
static OS_TCB  AppTaskStartTCB;
static CPU_STK AppTaskStartStk[APP_CFG_TASK_START_STK_SIZE];

static OS_TCB  AppTaskEmergencyTCB;
static OS_TCB  AppTaskSecurityTCB;
static OS_TCB  AppTaskInputTCB;
static OS_TCB  AppTaskStateTCB;
static OS_TCB  AppTaskUsartTCB;
static OS_TCB  AppTaskOutputTCB;
static OS_TCB  AppTaskMonitorTCB;

static CPU_STK AppTaskEmergencyStk[APP_CFG_TASK_STK_SIZE];
static CPU_STK AppTaskSecurityStk [APP_CFG_TASK_STK_SIZE];
static CPU_STK AppTaskInputStk    [APP_CFG_TASK_STK_SIZE];
static CPU_STK AppTaskStateStk    [APP_CFG_TASK_STK_SIZE];
static CPU_STK AppTaskUsartStk    [APP_CFG_TASK_STK_SIZE];
static CPU_STK AppTaskOutputStk   [APP_CFG_TASK_STK_SIZE];
static CPU_STK AppTaskMonitorStk  [APP_CFG_TASK_STK_SIZE];

static OS_FLAG_GRP AppEmergencyFlags;
static OS_SEM      AppPirSem;
static OS_SEM      AppButtonSem;
static OS_Q        AppEventQ;
static OS_MUTEX    AppModeMutex;

static APP_EVENT   AppEventPool[APP_EVENT_Q_SIZE];
static CPU_INT08U  AppEventWrIx;

static APP_MODE    AppMode;
static APP_MODE    AppPrevMode;
static APP_EMERGENCY_REASON  AppEmergencyReason;

static CPU_BOOLEAN AppEmergencyActive;
static CPU_BOOLEAN AppPirDetected;

static CPU_INT32U  AppGasCount;
static CPU_INT32U  AppFlameCount;
static CPU_INT32U  AppPirCount;
static CPU_INT32U  AppButtonCount;
static CPU_INT32U  AppCommandCount;

/* ---------------- main ---------------- */
int main(void)
{
    OS_ERR err;

    RCC_DeInit();
    AppGpioInit();

    BSP_IntDisAll();
    CPU_Init();
    Mem_Init();
    Math_Init();
    OSInit(&err);

    OSTaskCreate(&AppTaskStartTCB,
                 "App Task Start",
                 AppTaskStart,
                 (void *)0u,
                 APP_CFG_TASK_START_PRIO,
                 &AppTaskStartStk[0u],
                 APP_CFG_TASK_START_STK_SIZE / 10u,
                 APP_CFG_TASK_START_STK_SIZE,
                 0u,
                 0u,
                 (void *)0u,
                 OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,
                 &err);

    OSStart(&err);

    return 0u;
}

/* ---------------- Start task ---------------- */
static void AppTaskStart(void *p_arg)
{
    OS_ERR err;
    (void)p_arg;
    BSP_Init();
    BSP_Tick_Init();
    AppExtiInit();
    Setup_Usart3();
    Setup_Servo_PWM();
    Setup_Usart6_BT();
	#if OS_CFG_STAT_TASK_EN > 0u
    OSStatTaskCPUUsageInit(&err);
	#endif
	#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();
	#endif
    AppObjCreate();
    AppTaskCreate();
    AppTrace("\r\n[BOOT] Smart Home Safety System Start\r\n");
    AppPrintHelp();
    OSTaskDel((OS_TCB *)0, &err);
}

/* ---------------- Emergency: gas/flame ---------------- */
static void AppTaskEmergency(void *p_arg)
{
    OS_ERR      err;
    CPU_TS      ts;
    OS_FLAGS    flags;
    CPU_BOOLEAN post_event;
    CPU_SR_ALLOC();

    (void)p_arg;

    while (DEF_TRUE) {
        flags = OSFlagPend(&AppEmergencyFlags,
                           APP_FLAG_GAS | APP_FLAG_FLAME,
                           0u,
                           OS_OPT_PEND_FLAG_SET_ANY |
                           OS_OPT_PEND_FLAG_CONSUME |
                           OS_OPT_PEND_BLOCKING,
                           &ts,
                           &err);

        if ((flags & APP_FLAG_GAS) != 0u) {
            post_event = DEF_FALSE;

            /*
             * AppMode가 아니라 AppEmergencyActive로 먼저 latch.
             * State Task가 아직 AppMode를 EMERGENCY로 바꾸기 전이어도
             * 중복 GAS 이벤트를 막기 위함.
             */
            CPU_CRITICAL_ENTER();

            if (AppEmergencyActive == DEF_FALSE) {
                AppEmergencyActive = DEF_TRUE;
                AppEmergencyReason = APP_EMG_GAS;
                post_event = DEF_TRUE;
            }

            CPU_CRITICAL_EXIT();

            if (post_event == DEF_TRUE) {
                AppGasCount++;
                AppTrace("[ALERT][EMERGENCY] GAS detected\r\n");
                AppEventPost(APP_EVENT_GAS, 0u);
            }
        }

        if ((flags & APP_FLAG_FLAME) != 0u) {
            post_event = DEF_FALSE;

            CPU_CRITICAL_ENTER();

            if (AppEmergencyActive == DEF_FALSE) {
                AppEmergencyActive = DEF_TRUE;
                AppEmergencyReason = APP_EMG_FLAME;
                post_event = DEF_TRUE;
            }

            CPU_CRITICAL_EXIT();

            if (post_event == DEF_TRUE) {
                AppFlameCount++;
                AppTrace("[ALERT][EMERGENCY] FLAME detected\r\n");
                AppEventPost(APP_EVENT_FLAME, 0u);
            }
        }
    }
}

/* ---------------- Security: PIR only active in OUT mode ---------------- */
static void AppTaskSecurity(void *p_arg)
{
    OS_ERR   err;
    CPU_TS   ts;
    APP_MODE mode_snapshot;

    (void)p_arg;

    while (DEF_TRUE) {
        OSSemPend(&AppPirSem,
                  0u,
                  OS_OPT_PEND_BLOCKING,
                  &ts,
                  &err);

        /*
         * 간단한 PIR 안정화/노이즈 필터
         */
        OSTimeDlyHMSM(0u,
                      0u,
                      0u,
                      100u,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);

        /*
         * 아직도 PIR 입력이 HIGH일 때만 진짜 감지로 인정
         */
        if (GPIO_ReadInputDataBit(PIR_GPIO_PORT, PIR_GPIO_PIN) == Bit_RESET) {
            continue;
        }

        OSMutexPend(&AppModeMutex,
                    0u,
                    OS_OPT_PEND_BLOCKING,
                    &ts,
                    &err);

        mode_snapshot = AppMode;

        OSMutexPost(&AppModeMutex,
                    OS_OPT_POST_NONE,
                    &err);

        if (mode_snapshot == APP_MODE_OUT) {
            AppPirCount++;
            AppPirDetected = DEF_TRUE;

            AppTrace("[ALERT][SECURITY] Intrusion detected in OUT mode\r\n");
            AppEventPost(APP_EVENT_INTRUSION, 0u);
        } else {
            /*
             * HOME/EMERGENCY에서는 정상 움직임 또는 이미 비상상태이므로 무시
             * 로그 도배 방지를 위해 출력하지 않음
             */
        }
    }
}

/* ---------------- Button input ---------------- */
static void AppTaskInput(void *p_arg)
{
    OS_ERR   err;
    CPU_TS   ts;
    APP_MODE mode_snapshot;

    (void)p_arg;

    while (DEF_TRUE) {
        OSSemPend(&AppButtonSem,
                  0u,
                  OS_OPT_PEND_BLOCKING,
                  &ts,
                  &err);

        OSTimeDlyHMSM(0u,
                      0u,
                      0u,
                      APP_BUTTON_DEBOUNCE_MS,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);

        AppButtonCount++;

        OSMutexPend(&AppModeMutex,
                    0u,
                    OS_OPT_PEND_BLOCKING,
                    &ts,
                    &err);

        mode_snapshot = AppMode;

        OSMutexPost(&AppModeMutex,
                    OS_OPT_POST_NONE,
                    &err);

        if (mode_snapshot == APP_MODE_HOME) {
            AppTrace("[INFO][INPUT] Button -> OUT request\r\n");
            AppEventPost(APP_EVENT_REQ_OUT, 0u);
        } else if (mode_snapshot == APP_MODE_OUT) {
            AppTrace("[INFO][INPUT] Button -> HOME request\r\n");
            AppEventPost(APP_EVENT_REQ_HOME, 0u);
        } else {
            AppTrace("[WARN][INPUT] Button -> CLEAR emergency request\r\n");
            AppEventPost(APP_EVENT_REQ_CLEAR, 0u);
        }
    }
}

/* ---------------- State controller ---------------- */
static void AppTaskState(void *p_arg)
{
    OS_ERR      err;
    OS_MSG_SIZE msg_size;
    CPU_TS      ts;
    APP_EVENT  *p_event;

    (void)p_arg;

    while (DEF_TRUE) {
        p_event = (APP_EVENT *)OSQPend(&AppEventQ,
                                       0u,
                                       OS_OPT_PEND_BLOCKING,
                                       &msg_size,
                                       &ts,
                                       &err);

        if (p_event == (APP_EVENT *)0) {
            continue;
        }

        OSMutexPend(&AppModeMutex,
                    0u,
                    OS_OPT_PEND_BLOCKING,
                    &ts,
                    &err);

        switch (p_event->Type) {
        case APP_EVENT_REQ_OUT:
            if (AppMode != APP_MODE_EMERGENCY) {
                AppPrevMode = AppMode;
                AppMode = APP_MODE_OUT;
                AppTrace("[INFO][STATE] Mode changed: OUT\r\n");
            }
            break;

        case APP_EVENT_REQ_HOME:
            if (AppMode != APP_MODE_EMERGENCY) {
                AppPrevMode = AppMode;
                AppMode = APP_MODE_HOME;
                AppTrace("[INFO][STATE] Mode changed: HOME\r\n");
            }
            break;

        case APP_EVENT_GAS:
            if (AppMode != APP_MODE_EMERGENCY) {
                AppPrevMode = AppMode;
                AppMode = APP_MODE_EMERGENCY;
                AppEmergencyActive = DEF_TRUE;
                AppEmergencyReason = APP_EMG_GAS;

                AppTrace("[ALERT][STATE] Mode changed: EMERGENCY - GAS\r\n");
                //AppBtSend("[ALERT] GAS detected at home!\r\n");
            }
            break;

        case APP_EVENT_FLAME:
            if (AppMode != APP_MODE_EMERGENCY) {
                AppPrevMode = AppMode;
                AppMode = APP_MODE_EMERGENCY;
                AppEmergencyActive = DEF_TRUE;
                AppEmergencyReason = APP_EMG_FLAME;

                AppTrace("[ALERT][STATE] Mode changed: EMERGENCY - FLAME\r\n");
                //AppBtSend("[ALERT] FIRE detected at home!\r\n");
            }
            break;

        case APP_EVENT_INTRUSION:
            if (AppMode != APP_MODE_EMERGENCY) {
                AppPrevMode = AppMode;
                AppMode = APP_MODE_EMERGENCY;
                AppEmergencyActive = DEF_TRUE;
                AppEmergencyReason = APP_EMG_INTRUSION;

                AppTrace("[ALERT][STATE] Mode changed: EMERGENCY - INTRUSION\r\n");
                //AppBtSend("[ALERT] Intrusion detected while you are out!\r\n");
            }
            break;

        case APP_EVENT_REQ_CLEAR:
            if (AppMode == APP_MODE_EMERGENCY) {
                if (AppDangerStillActive() == DEF_FALSE) {
                    AppMode = AppPrevMode;
                    AppEmergencyActive = DEF_FALSE;
                    AppEmergencyReason = APP_EMG_NONE;
                    AppTrace("[INFO][STATE] Emergency cleared\r\n");
                } else {
                    AppTrace("[WARN][STATE] Clear rejected: danger still active\r\n");
                }
            }
            break;

        case APP_EVENT_CMD_STATUS:
            AppPrintStatus();
            break;

        case APP_EVENT_CMD_HELP:
            AppPrintHelp();
            break;

        default:
            break;
        }

        OSMutexPost(&AppModeMutex,
                    OS_OPT_POST_NONE,
                    &err);

        AppApplyOutputs();
    }
}

/* ---------------- USART command task ---------------- */
static void AppTaskUsart(void *p_arg)
{
    OS_ERR   err;
    CPU_CHAR line[APP_CMD_LINE_LEN];

    (void)p_arg;

    while (DEF_TRUE) {
        if (AppUsartTryGetLine(line, sizeof(line)) == DEF_TRUE) {
            AppCommandCount++;
            AppCmdParse(line);
        }

        OSTimeDlyHMSM(0u,
                      0u,
                      0u,
                      20u,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);
    }
}

/* ---------------- Output heartbeat / emergency blink ---------------- */
static void AppTaskOutput(void *p_arg)
{
    OS_ERR      err;
    CPU_BOOLEAN servo_toggle = DEF_FALSE;
    CPU_INT08U  servo_tick = 0u;

    (void)p_arg;

    while (DEF_TRUE) {
        if (AppMode == APP_MODE_EMERGENCY) {

            /*
             * Emergency common:
             * - Buzzer ON
             * - Power cut LED RED ON
             */
            GPIO_SetBits(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN);

            GPIO_SetBits  (POWER_LED_GPIO_PORT, POWER_LED_CUT_RED_PIN);
            GPIO_ResetBits(POWER_LED_GPIO_PORT, POWER_LED_NORMAL_GREEN_PIN);

            if (AppEmergencyReason == APP_EMG_FLAME) {
                /*
                 * FLAME:
                 * - Power cut ON
                 * - Gas state remains normal
                 * - Servo sprinkler motion
                 */
                GPIO_ResetBits(GAS_LED_GPIO_PORT, GAS_LED_CUT_RED_PIN);
                GPIO_SetBits  (GAS_LED_GPIO_PORT, GAS_LED_NORMAL_GREEN_PIN);

                servo_tick++;

                if (servo_tick >= 3u) {
                    servo_tick = 0u;

                    servo_toggle = (servo_toggle == DEF_TRUE)
                                   ? DEF_FALSE
                                   : DEF_TRUE;

                    if (servo_toggle == DEF_TRUE) {
                        AppServoSetAngle(30u);
                    } else {
                        AppServoSetAngle(150u);
                    }
                }

            } else if (AppEmergencyReason == APP_EMG_GAS) {
                /*
                 * GAS:
                 * - Power cut ON
                 * - Gas cut ON
                 * - Servo stopped
                 */
                GPIO_SetBits  (GAS_LED_GPIO_PORT, GAS_LED_CUT_RED_PIN);
                GPIO_ResetBits(GAS_LED_GPIO_PORT, GAS_LED_NORMAL_GREEN_PIN);

                servo_tick = 0u;
                servo_toggle = DEF_FALSE;
                AppServoSetAngle(0u);

            } else {
                /*
                 * INTRUSION or unknown emergency:
                 * - Buzzer only
                 * - Keep power cut LED ON because OUT mode is power-cut concept
                 * - Gas remains normal
                 */
                GPIO_ResetBits(GAS_LED_GPIO_PORT, GAS_LED_CUT_RED_PIN);
                GPIO_SetBits  (GAS_LED_GPIO_PORT, GAS_LED_NORMAL_GREEN_PIN);

                servo_tick = 0u;
                servo_toggle = DEF_FALSE;
                AppServoSetAngle(0u);
            }

        } else {
            servo_tick = 0u;
            servo_toggle = DEF_FALSE;
            AppApplyOutputs();
        }

        OSTimeDlyHMSM(0u,
                      0u,
                      0u,
                      200u,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);
    }
}

/* ---------------- Monitor task ---------------- */
static void AppTaskMonitor(void *p_arg)
{
    OS_ERR err;

    (void)p_arg;

    while (DEF_TRUE) {

        OSTimeDlyHMSM(0u,
                      0u,
                      0u,
                      100u,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);
    }
}

/* ---------------- Kernel objects ---------------- */
static void AppObjCreate(void)
{
    OS_ERR err;

    OSFlagCreate(&AppEmergencyFlags,
                 "Emergency Flags",
                 0u,
                 &err);

    OSSemCreate(&AppPirSem,
                "PIR Semaphore",
                0u,
                &err);

    OSSemCreate(&AppButtonSem,
                "Button Semaphore",
                0u,
                &err);

    OSQCreate(&AppEventQ,
              "Event Queue",
              APP_EVENT_Q_SIZE,
              &err);

    OSMutexCreate(&AppModeMutex,
                  "Mode Mutex",
                  &err);

    AppMode = APP_MODE_HOME;
    AppPrevMode = APP_MODE_HOME;
    AppEmergencyReason = APP_EMG_NONE;
    AppEmergencyActive = DEF_FALSE;
    AppPirDetected = DEF_FALSE;

    AppEventWrIx = 0u;

    AppGasCount = 0u;
    AppFlameCount = 0u;
    AppPirCount = 0u;
    AppButtonCount = 0u;
    AppCommandCount = 0u;
}

/* ---------------- Task create ---------------- */
static void AppTaskCreate(void)
{
    OS_ERR err;

    OSTaskCreate(&AppTaskEmergencyTCB,
                 "Emergency Task",
                 AppTaskEmergency,
                 0,
                 APP_CFG_EMERGENCY_TASK_PRIO,
                 &AppTaskEmergencyStk[0],
                 APP_CFG_TASK_STK_SIZE / 10,
                 APP_CFG_TASK_STK_SIZE,
                 0,
                 0,
                 0,
                 OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,
                 &err);

    OSTaskCreate(&AppTaskSecurityTCB,
                 "Security Task",
                 AppTaskSecurity,
                 0,
                 APP_CFG_SECURITY_TASK_PRIO,
                 &AppTaskSecurityStk[0],
                 APP_CFG_TASK_STK_SIZE / 10,
                 APP_CFG_TASK_STK_SIZE,
                 0,
                 0,
                 0,
                 OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,
                 &err);

    OSTaskCreate(&AppTaskInputTCB,
                 "Input Task",
                 AppTaskInput,
                 0,
                 APP_CFG_INPUT_TASK_PRIO,
                 &AppTaskInputStk[0],
                 APP_CFG_TASK_STK_SIZE / 10,
                 APP_CFG_TASK_STK_SIZE,
                 0,
                 0,
                 0,
                 OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,
                 &err);

    OSTaskCreate(&AppTaskStateTCB,
                 "State Task",
                 AppTaskState,
                 0,
                 APP_CFG_STATE_TASK_PRIO,
                 &AppTaskStateStk[0],
                 APP_CFG_TASK_STK_SIZE / 10,
                 APP_CFG_TASK_STK_SIZE,
                 0,
                 0,
                 0,
                 OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,
                 &err);

    OSTaskCreate(&AppTaskUsartTCB,
                 "USART Task",
                 AppTaskUsart,
                 0,
                 APP_CFG_USART_TASK_PRIO,
                 &AppTaskUsartStk[0],
                 APP_CFG_TASK_STK_SIZE / 10,
                 APP_CFG_TASK_STK_SIZE,
                 0,
                 0,
                 0,
                 OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,
                 &err);

    OSTaskCreate(&AppTaskOutputTCB,
                 "Output Task",
                 AppTaskOutput,
                 0,
                 APP_CFG_OUTPUT_TASK_PRIO,
                 &AppTaskOutputStk[0],
                 APP_CFG_TASK_STK_SIZE / 10,
                 APP_CFG_TASK_STK_SIZE,
                 0,
                 0,
                 0,
                 OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,
                 &err);

    OSTaskCreate(&AppTaskMonitorTCB,
                 "Monitor Task",
                 AppTaskMonitor,
                 0,
                 APP_CFG_MONITOR_TASK_PRIO,
                 &AppTaskMonitorStk[0],
                 APP_CFG_TASK_STK_SIZE / 10,
                 APP_CFG_TASK_STK_SIZE,
                 0,
                 0,
                 0,
                 OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,
                 &err);
}

/* ---------------- GPIO / EXTI ---------------- */
static void AppGpioInit(void)
{
    GPIO_InitTypeDef gpio;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA |
                       RCC_AHB1Periph_GPIOB |
                       RCC_AHB1Periph_GPIOC |
                       RCC_AHB1Periph_GPIOD |
                       RCC_AHB1Periph_GPIOE |
                       RCC_AHB1Periph_GPIOF,
                       ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,
                           ENABLE);

    /*
     * LED output
     */
    gpio.GPIO_Mode  = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;

    /* Power LED: PB8, PB9 */
    gpio.GPIO_Pin = POWER_LED_CUT_RED_PIN |
                    POWER_LED_NORMAL_GREEN_PIN;
    GPIO_Init(POWER_LED_GPIO_PORT, &gpio);

    /* Gas LED: PA6, PA7 */
    gpio.GPIO_Pin = GAS_LED_CUT_RED_PIN |
                    GAS_LED_NORMAL_GREEN_PIN;
    GPIO_Init(GAS_LED_GPIO_PORT, &gpio);

    /*
     * Buzzer output
     */
    gpio.GPIO_Pin = BUZZER_GPIO_PIN;
    GPIO_Init(BUZZER_GPIO_PORT, &gpio);

    /*
     * Sensor inputs
     */
    gpio.GPIO_Mode  = GPIO_Mode_IN;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;

    /*
     * GAS: D8 / PF12
     * 테스트 중 선을 뺐을 때 floating 방지용 풀다운
     */
    gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
    gpio.GPIO_Pin  = GAS_GPIO_PIN;
    GPIO_Init(GAS_GPIO_PORT, &gpio);

    /*
    * FLAME: D2 / PF15
    * 테스트 중 선을 뺐을 때 floating 방지
    */
    gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
    gpio.GPIO_Pin  = FLAME_GPIO_PIN;
    GPIO_Init(FLAME_GPIO_PORT, &gpio);

    /*
     * PIR: D4 / PF14
     * OUT 선을 뺐을 때 floating 방지
     */
    gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
    gpio.GPIO_Pin  = PIR_GPIO_PIN;
    GPIO_Init(PIR_GPIO_PORT, &gpio);

    /*
     * User button input: PC13
     */
    gpio.GPIO_Mode  = GPIO_Mode_IN;
    gpio.GPIO_PuPd  = GPIO_PuPd_DOWN;
    gpio.GPIO_Pin   = BTN_GPIO_PIN;
    GPIO_Init(BTN_GPIO_PORT, &gpio);

    /* 초기 HOME 상태:
     * 전력 정상 초록 ON
     * 가스 정상 초록 ON
     * 차단 빨강들은 OFF
     */
    GPIO_ResetBits(POWER_LED_GPIO_PORT, POWER_LED_CUT_RED_PIN);
    GPIO_SetBits  (POWER_LED_GPIO_PORT, POWER_LED_NORMAL_GREEN_PIN);

    GPIO_ResetBits(GAS_LED_GPIO_PORT, GAS_LED_CUT_RED_PIN);
    GPIO_SetBits  (GAS_LED_GPIO_PORT, GAS_LED_NORMAL_GREEN_PIN);

    GPIO_ResetBits(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN);
}

static void AppExtiInit(void)
{
    EXTI_InitTypeDef exti;

    SYSCFG_EXTILineConfig(GAS_EXTI_PORT_SRC,
                          GAS_EXTI_PIN_SRC);

    SYSCFG_EXTILineConfig(FLAME_EXTI_PORT_SRC,
                          FLAME_EXTI_PIN_SRC);

    SYSCFG_EXTILineConfig(PIR_EXTI_PORT_SRC,
                          PIR_EXTI_PIN_SRC);

    SYSCFG_EXTILineConfig(BTN_EXTI_PORT_SRC,
                          BTN_EXTI_PIN_SRC);

    EXTI_ClearITPendingBit(GAS_EXTI_LINE |
                           FLAME_EXTI_LINE |
                           PIR_EXTI_LINE |
                           BTN_EXTI_LINE);

    exti.EXTI_Mode    = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Rising;
    exti.EXTI_LineCmd = ENABLE;

    exti.EXTI_Line = GAS_EXTI_LINE;
    EXTI_Init(&exti);

    exti.EXTI_Line = FLAME_EXTI_LINE;
    EXTI_Init(&exti);

    exti.EXTI_Line = PIR_EXTI_LINE;
    EXTI_Init(&exti);

    exti.EXTI_Trigger = EXTI_Trigger_Falling;
    exti.EXTI_Line    = BTN_EXTI_LINE;
    EXTI_Init(&exti);

    BSP_IntVectSet(BSP_INT_ID_EXTI15_10,
                   AppExti15_10ISR);

    BSP_IntPrioSet(BSP_INT_ID_EXTI15_10,
                   5u);

    BSP_IntEn(BSP_INT_ID_EXTI15_10);
}

/* ---------------- ISR ---------------- */

static void AppExti15_10ISR(void)
{
    OS_ERR err;

    /* GAS: PF12 / D8 */
    if (EXTI_GetITStatus(GAS_EXTI_LINE) != RESET) {
        EXTI_ClearITPendingBit(GAS_EXTI_LINE);

        OSFlagPost(&AppEmergencyFlags,
                   APP_FLAG_GAS,
                   OS_OPT_POST_FLAG_SET,
                   &err);
    }

    /* FLAME: PF15 / D2 */
    if (EXTI_GetITStatus(FLAME_EXTI_LINE) != RESET) {
        EXTI_ClearITPendingBit(FLAME_EXTI_LINE);

        OSFlagPost(&AppEmergencyFlags,
                   APP_FLAG_FLAME,
                   OS_OPT_POST_FLAG_SET,
                   &err);
    }

    /* PIR: PF14 / D4 */
    if (EXTI_GetITStatus(PIR_EXTI_LINE) != RESET) {
        EXTI_ClearITPendingBit(PIR_EXTI_LINE);

        OSSemPost(&AppPirSem,
                  OS_OPT_POST_1,
                  &err);
    }

    /* Button: PC13 */
    if (EXTI_GetITStatus(BTN_EXTI_LINE) != RESET) {
        EXTI_ClearITPendingBit(BTN_EXTI_LINE);

        OSSemPost(&AppButtonSem,
                  OS_OPT_POST_1,
                  &err);
    }
}

/* ---------------- Event / command helpers ---------------- */
static void AppEventPost(APP_EVENT_TYPE type, CPU_INT32U value)
{
    APP_EVENT *p_event;
    OS_ERR     err;
    CPU_SR_ALLOC();

    CPU_CRITICAL_ENTER();

    p_event = &AppEventPool[AppEventWrIx];

    AppEventWrIx++;
    if (AppEventWrIx >= APP_EVENT_Q_SIZE) {
        AppEventWrIx = 0u;
    }

    p_event->Type  = type;
    p_event->Value = value;

    CPU_CRITICAL_EXIT();

    OSQPost(&AppEventQ,
            p_event,
            sizeof(APP_EVENT),
            OS_OPT_POST_FIFO,
            &err);
}

static void AppCmdParse(CPU_CHAR *line)
{
    if (AppStrEq(line, "help") == DEF_TRUE) {
        AppEventPost(APP_EVENT_CMD_HELP, 0u);
    } else if (AppStrEq(line, "status") == DEF_TRUE) {
        AppEventPost(APP_EVENT_CMD_STATUS, 0u);
    } else if (AppStrEq(line, "out") == DEF_TRUE) {
        AppEventPost(APP_EVENT_REQ_OUT, 0u);
    } else if (AppStrEq(line, "home") == DEF_TRUE) {
        AppEventPost(APP_EVENT_REQ_HOME, 0u);
    } else if (AppStrEq(line, "clear") == DEF_TRUE) {
        AppEventPost(APP_EVENT_REQ_CLEAR, 0u);
    } else {
        AppTrace("[WARN][USART] Unknown command. Type help\r\n");
    }
}

static void AppPrintHelp(void)
{
    AppTrace("[HELP] home | out | clear | status | help\r\n");
}

static void AppPrintStatus(void)
{
    APP_TRACE_DBG(("[MONITOR] mode=%d gas=%lu flame=%lu pir=%lu btn=%lu cmd=%lu\r\n",
                   (int)AppMode,
                   (unsigned long)AppGasCount,
                   (unsigned long)AppFlameCount,
                   (unsigned long)AppPirCount,
                   (unsigned long)AppButtonCount,
                   (unsigned long)AppCommandCount));
}

/* ---------------- Output helpers ---------------- */
static void AppApplyOutputs(void)
{
    if (AppMode == APP_MODE_HOME) {
        /*
         * HOME:
         * Power normal, Gas normal
         */
        GPIO_ResetBits(POWER_LED_GPIO_PORT, POWER_LED_CUT_RED_PIN);
        GPIO_SetBits  (POWER_LED_GPIO_PORT, POWER_LED_NORMAL_GREEN_PIN);

        GPIO_ResetBits(GAS_LED_GPIO_PORT, GAS_LED_CUT_RED_PIN);
        GPIO_SetBits  (GAS_LED_GPIO_PORT, GAS_LED_NORMAL_GREEN_PIN);

        GPIO_ResetBits(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN);
        AppServoSetAngle(0u);

    } else if (AppMode == APP_MODE_OUT) {
        /*
         * OUT:
         * Power cut, Gas normal
         */
        GPIO_SetBits  (POWER_LED_GPIO_PORT, POWER_LED_CUT_RED_PIN);
        GPIO_ResetBits(POWER_LED_GPIO_PORT, POWER_LED_NORMAL_GREEN_PIN);

        GPIO_ResetBits(GAS_LED_GPIO_PORT, GAS_LED_CUT_RED_PIN);
        GPIO_SetBits  (GAS_LED_GPIO_PORT, GAS_LED_NORMAL_GREEN_PIN);

        GPIO_ResetBits(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN);
        AppServoSetAngle(0u);

    } else {
        /*
         * EMERGENCY initial output.
         * Continuous emergency output is handled by AppTaskOutput().
         */
        GPIO_SetBits  (POWER_LED_GPIO_PORT, POWER_LED_CUT_RED_PIN);
        GPIO_ResetBits(POWER_LED_GPIO_PORT, POWER_LED_NORMAL_GREEN_PIN);

        GPIO_SetBits(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN);

        if (AppEmergencyReason == APP_EMG_GAS) {
            GPIO_SetBits  (GAS_LED_GPIO_PORT, GAS_LED_CUT_RED_PIN);
            GPIO_ResetBits(GAS_LED_GPIO_PORT, GAS_LED_NORMAL_GREEN_PIN);
            AppServoSetAngle(0u);

        } else if (AppEmergencyReason == APP_EMG_FLAME) {
            GPIO_ResetBits(GAS_LED_GPIO_PORT, GAS_LED_CUT_RED_PIN);
            GPIO_SetBits  (GAS_LED_GPIO_PORT, GAS_LED_NORMAL_GREEN_PIN);
            AppServoSetAngle(90u);

        } else {
            GPIO_ResetBits(GAS_LED_GPIO_PORT, GAS_LED_CUT_RED_PIN);
            GPIO_SetBits  (GAS_LED_GPIO_PORT, GAS_LED_NORMAL_GREEN_PIN);
            AppServoSetAngle(0u);
        }
    }
}

static void AppServoSetAngle(CPU_INT08U degree)
{
	CPU_INT16U pulse_us;

	    pulse_us = 1000u + ((CPU_INT16U)degree * 1000u / 180u);

	    TIM_SetCompare1(TIM1, pulse_us);
}

static CPU_BOOLEAN AppDangerStillActive(void)
{
    CPU_BOOLEAN gas_on;
    CPU_BOOLEAN flame_on;

    gas_on = (GPIO_ReadInputDataBit(GAS_GPIO_PORT, GAS_GPIO_PIN) == Bit_SET)
             ? DEF_TRUE
             : DEF_FALSE;

    flame_on = (GPIO_ReadInputDataBit(FLAME_GPIO_PORT, FLAME_GPIO_PIN) == Bit_SET)
               ? DEF_TRUE
               : DEF_FALSE;

    if ((gas_on == DEF_TRUE) || (flame_on == DEF_TRUE)) {
        return DEF_TRUE;
    }

    return DEF_FALSE;
}

/* ---------------- USART / string helpers ---------------- */
static void AppTrace(const CPU_CHAR *msg)
{
    while (*msg != '\0') {
        /* Send log to PC via USART3 */
        while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET) {
        }
        USART_SendData(USART3, (uint16_t)(*msg));

        /* Send log to Bluetooth via USART6 simultaneously */
        while (USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET) {
        }
        USART_SendData(USART6, (uint16_t)(*msg));

        msg++;
    }
}

static CPU_BOOLEAN AppUsartTryGetLine(CPU_CHAR *buf, CPU_INT16U buf_len)
{
    static CPU_CHAR  rx_buf[APP_CMD_LINE_LEN];
    static CPU_INT16U ix = 0u;

    CPU_CHAR ch = (CPU_CHAR)0;
    CPU_BOOLEAN data_received = DEF_FALSE;

    /* Check input from PC (USART3) */
    if (USART_GetFlagStatus(USART3, USART_FLAG_RXNE) != RESET) {
        ch = (CPU_CHAR)USART_ReceiveData(USART3);
        data_received = DEF_TRUE;
    }
    /* Check input from Bluetooth (USART6) */
    else if (USART_GetFlagStatus(USART6, USART_FLAG_RXNE) != RESET) {
        ch = (CPU_CHAR)USART_ReceiveData(USART6);
        data_received = DEF_TRUE;
    }

    /* Return if no data is received from either channel */
    if (data_received == DEF_FALSE) {
        return DEF_FALSE;
    }

    APP_TRACE_DBG(("%c", ch));

    if ((ch == '\r') || (ch == '\n')) {
        rx_buf[ix] = (CPU_CHAR)0;
        ix = 0u;

        if (rx_buf[0] == (CPU_CHAR)0) {
            return DEF_FALSE;
        }

        for (ix = 0u;
             (ix < buf_len - 1u) && (rx_buf[ix] != (CPU_CHAR)0);
             ix++) {
            buf[ix] = rx_buf[ix];
        }

        buf[ix] = (CPU_CHAR)0;
        ix = 0u;

        AppTrace("\r\n");

        return DEF_TRUE;
    }

    if (ix < (APP_CMD_LINE_LEN - 1u)) {
        rx_buf[ix++] = ch;
    } else {
        ix = 0u;
        AppTrace("\r\n[WARN][USART] command buffer overflow\r\n");
    }

    return DEF_FALSE;
}

static CPU_BOOLEAN AppStrEq(const CPU_CHAR *a, const CPU_CHAR *b)
{
    while ((*a != (CPU_CHAR)0) && (*b != (CPU_CHAR)0)) {
        if (*a != *b) {
            return DEF_FALSE;
        }

        a++;
        b++;
    }

    if ((*a == (CPU_CHAR)0) && (*b == (CPU_CHAR)0)) {
        return DEF_TRUE;
    }

    return DEF_FALSE;
}
/* ---------------- USART3 SETUP ---------------- */
static void Setup_Usart3(void)
{
    GPIO_InitTypeDef gpio_init = {0};
    USART_InitTypeDef usart_init = {0};

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);

    gpio_init.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9;
    gpio_init.GPIO_Mode  = GPIO_Mode_AF;
    gpio_init.GPIO_OType = GPIO_OType_PP;
    gpio_init.GPIO_PuPd  = GPIO_PuPd_UP;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOD, &gpio_init);

    usart_init.USART_BaudRate = 115200;
    usart_init.USART_WordLength = USART_WordLength_8b;
    usart_init.USART_StopBits = USART_StopBits_1;
    usart_init.USART_Parity = USART_Parity_No;
    usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart_init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART3, &usart_init);
    USART_Cmd(USART3, ENABLE);
}

static void Setup_Servo_PWM(void)
{
    GPIO_InitTypeDef         gpio_init = {0};
    TIM_TimeBaseInitTypeDef  tim_init  = {0};
    TIM_OCInitTypeDef        oc_init   = {0};

    /* GPIOE clock + TIM1 clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    gpio_init.GPIO_Pin   = GPIO_Pin_9;
    gpio_init.GPIO_Mode  = GPIO_Mode_AF;
    gpio_init.GPIO_OType = GPIO_OType_PP;
    gpio_init.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &gpio_init);

    GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);

    /* TIM1: 180MHz / (180-1+1) / (20000-1+1) = 50Hz (20ms) */
    tim_init.TIM_Prescaler     = 180 - 1;       /* 1MHz tick */
    tim_init.TIM_Period        = 20000 - 1;     /* 20ms period */
    tim_init.TIM_CounterMode   = TIM_CounterMode_Up;
    tim_init.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM1, &tim_init);

    /* TIM1 CH1 PWM mode */
    oc_init.TIM_OCMode      = TIM_OCMode_PWM1;
    oc_init.TIM_OutputState = TIM_OutputState_Enable;
    oc_init.TIM_Pulse       = 1000;
    oc_init.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC1Init(TIM1, &oc_init);

    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);

    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
}
/* ---------------- USART6 (HC-06 Bluetooth) SETUP ----------------
 * HC-06 TXD -> D0 / PG9  / USART6_RX
 * HC-06 RXD -> D1 / PG14 / USART6_TX
 */
static void Setup_Usart6_BT(void)
{
    GPIO_InitTypeDef  gpio_init  = {0};
    USART_InitTypeDef usart_init = {0};

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

    /*
     * D0  = PG9  = USART6_RX
     * D1  = PG14 = USART6_TX
     */
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource9,  GPIO_AF_USART6);
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource14, GPIO_AF_USART6);

    gpio_init.GPIO_Pin   = GPIO_Pin_9 | GPIO_Pin_14;
    gpio_init.GPIO_Mode  = GPIO_Mode_AF;
    gpio_init.GPIO_OType = GPIO_OType_PP;
    gpio_init.GPIO_PuPd  = GPIO_PuPd_UP;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOG, &gpio_init);

    /*
     * HC-06 기본 baud는 보통 9600
     */
    usart_init.USART_BaudRate            = 9600;
    usart_init.USART_WordLength          = USART_WordLength_8b;
    usart_init.USART_StopBits            = USART_StopBits_1;
    usart_init.USART_Parity              = USART_Parity_No;
    usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart_init.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART6, &usart_init);
    USART_Cmd(USART6, ENABLE);
}