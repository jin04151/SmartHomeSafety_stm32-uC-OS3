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

/* ---------------- EXTI input pin example ----------------
 * Gas   DO -> PA0, EXTI0
 * Flame DO -> PA1, EXTI1
 * PIR   DO -> PA2, EXTI2
 * Button   -> PC13, EXTI13
 */
#define GAS_GPIO_PORT                    GPIOA
#define GAS_GPIO_PIN                     GPIO_Pin_0
#define GAS_EXTI_LINE                    EXTI_Line0
#define GAS_EXTI_PORT_SRC                EXTI_PortSourceGPIOA
#define GAS_EXTI_PIN_SRC                 EXTI_PinSource0

#define FLAME_GPIO_PORT                  GPIOF
#define FLAME_GPIO_PIN                   GPIO_Pin_15
#define FLAME_EXTI_LINE                  EXTI_Line15
#define FLAME_EXTI_PORT_SRC              EXTI_PortSourceGPIOF
#define FLAME_EXTI_PIN_SRC               EXTI_PinSource15

#define PIR_GPIO_PORT                    GPIOA
#define PIR_GPIO_PIN                     GPIO_Pin_2
#define PIR_EXTI_LINE                    EXTI_Line2
#define PIR_EXTI_PORT_SRC                EXTI_PortSourceGPIOA
#define PIR_EXTI_PIN_SRC                 EXTI_PinSource2

#define BTN_GPIO_PORT                    GPIOC
#define BTN_GPIO_PIN                     GPIO_Pin_13
#define BTN_EXTI_LINE                    EXTI_Line13
#define BTN_EXTI_PORT_SRC                EXTI_PortSourceGPIOC
#define BTN_EXTI_PIN_SRC                 EXTI_PinSource13

/* ---------------- Output pin example ----------------
 * LED1 alarm    -> PA5  (LED1 module RED)
 * LED2 power    -> PA7  (LED2 module RED)
 * LED3 status   -> PB10 (LED2 module GREEN)
 * Buzzer        -> PD12
 * Servo         -> placeholder function. If PWM is ready, replace AppServoSetAngle().
 */
#define LED_GPIO_PORT     GPIOA
#define LED_ALARM_PIN     GPIO_Pin_5
#define LED_POWER_PIN     GPIO_Pin_7
#define LED_STATUS_PIN    GPIO_Pin_6

#define BUZZER_GPIO_PORT                 GPIOD
#define BUZZER_GPIO_PIN                  GPIO_Pin_12

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

static void AppExti0ISR      (void);
static void AppExti1ISR      (void);
static void AppExti2ISR      (void);
static void AppExti15_10ISR  (void);

static void AppEventPost     (APP_EVENT_TYPE type, CPU_INT32U value);
static void AppCmdParse      (CPU_CHAR *line);
static void AppPrintHelp     (void);
static void AppPrintStatus   (void);
static void AppTrace         (const CPU_CHAR *msg);
static CPU_BOOLEAN AppUsartTryGetLine(CPU_CHAR *buf, CPU_INT16U buf_len);
static CPU_BOOLEAN AppStrEq  (const CPU_CHAR *a, const CPU_CHAR *b);

static void AppApplyOutputs  (void);
static void AppAlarmOn       (void);
static void AppAlarmOff      (void);
static void AppPowerCut      (void);
static void AppPowerOn       (void);
static void AppServoSetAngle (CPU_INT08U degree);
static CPU_BOOLEAN AppDangerStillActive(void);

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

    while (DEF_TRUE) {

        OSTimeDlyHMSM(0u,
                  0u,
                  1u,
                  0u,
                  OS_OPT_TIME_HMSM_STRICT,
                  &err);
    }
}

/* ---------------- Emergency: gas/flame ---------------- */
static void AppTaskEmergency(void *p_arg)
{
    OS_ERR   err;
    CPU_TS   ts;
    OS_FLAGS flags;

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
            AppGasCount++;
            AppTrace("[ALERT][EMERGENCY] GAS detected\r\n");
            AppEventPost(APP_EVENT_GAS, 0u);
        }

        if ((flags & APP_FLAG_FLAME) != 0u) {
            AppFlameCount++;
            AppTrace("[ALERT][EMERGENCY] FLAME detected\r\n");
            AppEventPost(APP_EVENT_FLAME, 0u);
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

        AppPirCount++;
        AppPirDetected = DEF_TRUE;

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
            AppTrace("[ALERT][SECURITY] Intrusion detected in OUT mode\r\n");
            AppEventPost(APP_EVENT_INTRUSION, 0u);
        } else {
            AppTrace("[INFO][SECURITY] PIR ignored in HOME/EMERGENCY mode\r\n");
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
        case APP_EVENT_FLAME:
        case APP_EVENT_INTRUSION:
            if (AppMode != APP_MODE_EMERGENCY) {
                AppPrevMode = AppMode;
            }

            AppMode = APP_MODE_EMERGENCY;
            AppEmergencyActive = DEF_TRUE;

            AppTrace("[ALERT][STATE] Mode changed: EMERGENCY\r\n");
            break;

        case APP_EVENT_REQ_CLEAR:
            if (AppMode == APP_MODE_EMERGENCY) {
                if (AppDangerStillActive() == DEF_FALSE) {
                    AppMode = AppPrevMode;
                    AppEmergencyActive = DEF_FALSE;
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
    CPU_BOOLEAN blink = DEF_FALSE;

    (void)p_arg;

    while (DEF_TRUE) {
        if (AppMode == APP_MODE_EMERGENCY) {
            blink = (blink == DEF_TRUE) ? DEF_FALSE : DEF_TRUE;

            if (blink == DEF_TRUE) {
                GPIO_SetBits(LED_GPIO_PORT, LED_ALARM_PIN);
                GPIO_SetBits(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN);
            } else {
                GPIO_ResetBits(LED_GPIO_PORT, LED_ALARM_PIN);
                GPIO_ResetBits(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN);
            }
        } else {
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
    BitAction flame_raw;

    (void)p_arg;

    while (DEF_TRUE) {
        flame_raw = GPIO_ReadInputDataBit(FLAME_GPIO_PORT, FLAME_GPIO_PIN);

        if (flame_raw == Bit_SET) {
            GPIO_SetBits(GPIOB, GPIO_Pin_0);
        } else {
            GPIO_ResetBits(GPIOB, GPIO_Pin_0);
        }

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
                           RCC_AHB1Periph_GPIOF,
                           ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,
                           ENABLE);

    gpio.GPIO_Mode  = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    gpio.GPIO_Pin   = LED_ALARM_PIN | LED_POWER_PIN | LED_STATUS_PIN;
    GPIO_Init(LED_GPIO_PORT, &gpio);

    gpio.GPIO_Pin = BUZZER_GPIO_PIN;
    GPIO_Init(BUZZER_GPIO_PORT, &gpio);

    gpio.GPIO_Mode  = GPIO_Mode_IN;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    gpio.GPIO_PuPd  = GPIO_PuPd_DOWN;
    gpio.GPIO_Pin   = GAS_GPIO_PIN | PIR_GPIO_PIN;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = FLAME_GPIO_PIN;
    GPIO_Init(FLAME_GPIO_PORT, &gpio);

    /* 踰꾪듉 �엯�젰: PC13 */
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio.GPIO_Pin  = BTN_GPIO_PIN;
    GPIO_Init(BTN_GPIO_PORT, &gpio);

    GPIO_ResetBits(LED_GPIO_PORT, LED_ALARM_PIN | LED_STATUS_PIN);
    GPIO_SetBits(LED_GPIO_PORT, LED_POWER_PIN);
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

    BSP_IntVectSet(BSP_INT_ID_EXTI0,
                   AppExti0ISR);

    BSP_IntVectSet(BSP_INT_ID_EXTI1,
                   AppExti1ISR);

    BSP_IntVectSet(BSP_INT_ID_EXTI2,
                   AppExti2ISR);

    BSP_IntVectSet(BSP_INT_ID_EXTI15_10,
                   AppExti15_10ISR);

    BSP_IntPrioSet(BSP_INT_ID_EXTI0,
                   5u);

    BSP_IntPrioSet(BSP_INT_ID_EXTI1,
                   5u);

    BSP_IntPrioSet(BSP_INT_ID_EXTI2,
                   5u);

    BSP_IntPrioSet(BSP_INT_ID_EXTI15_10,
                   5u);

    BSP_IntEn(BSP_INT_ID_EXTI0);
    BSP_IntEn(BSP_INT_ID_EXTI1);
    BSP_IntEn(BSP_INT_ID_EXTI2);
    BSP_IntEn(BSP_INT_ID_EXTI15_10);
}

/* ---------------- ISR ---------------- */
static void AppExti0ISR(void)
{
    OS_ERR err;

    if (EXTI_GetITStatus(GAS_EXTI_LINE) != RESET) {
        EXTI_ClearITPendingBit(GAS_EXTI_LINE);

        OSFlagPost(&AppEmergencyFlags,
                   APP_FLAG_GAS,
                   OS_OPT_POST_FLAG_SET,
                   &err);
    }
}

static void AppExti1ISR(void)
{
    OS_ERR err;

    if (EXTI_GetITStatus(FLAME_EXTI_LINE) != RESET) {
        EXTI_ClearITPendingBit(FLAME_EXTI_LINE);

        OSFlagPost(&AppEmergencyFlags,
                   APP_FLAG_FLAME,
                   OS_OPT_POST_FLAG_SET,
                   &err);
    }
}

static void AppExti2ISR(void)
{
    OS_ERR err;

    if (EXTI_GetITStatus(PIR_EXTI_LINE) != RESET) {
        EXTI_ClearITPendingBit(PIR_EXTI_LINE);

        OSSemPost(&AppPirSem,
                  OS_OPT_POST_1,
                  &err);
    }
}

static void AppExti15_10ISR(void)
{
    OS_ERR err;

    if (EXTI_GetITStatus(FLAME_EXTI_LINE) != RESET) {
        EXTI_ClearITPendingBit(FLAME_EXTI_LINE);

        GPIO_SetBits(LED_GPIO_PORT, LED_ALARM_PIN);

        OSFlagPost(&AppEmergencyFlags,
                   APP_FLAG_FLAME,
                   OS_OPT_POST_FLAG_SET,
                   &err);
    }

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
        AppAlarmOff();
        AppPowerOn();
        GPIO_ResetBits(LED_GPIO_PORT, LED_STATUS_PIN);
        AppServoSetAngle(0u);
    } else if (AppMode == APP_MODE_OUT) {
        AppAlarmOff();
        AppPowerCut();
        GPIO_SetBits(LED_GPIO_PORT, LED_STATUS_PIN);
        AppServoSetAngle(0u);
    } else {
        AppAlarmOn();
        AppPowerCut();
        GPIO_SetBits(LED_GPIO_PORT, LED_STATUS_PIN);
        AppServoSetAngle(90u);
    }
}

static void AppAlarmOn(void)
{
    GPIO_SetBits(LED_GPIO_PORT,
                 LED_ALARM_PIN);

    GPIO_SetBits(BUZZER_GPIO_PORT,
                 BUZZER_GPIO_PIN);
}

static void AppAlarmOff(void)
{
    GPIO_ResetBits(LED_GPIO_PORT,
                   LED_ALARM_PIN);

    GPIO_ResetBits(BUZZER_GPIO_PORT,
                   BUZZER_GPIO_PIN);
}

static void AppPowerCut(void)
{
    GPIO_ResetBits(LED_GPIO_PORT,
                   LED_POWER_PIN);
}

static void AppPowerOn(void)
{
    GPIO_SetBits(LED_GPIO_PORT,
                 LED_POWER_PIN);
}

static void AppServoSetAngle(CPU_INT08U degree)
{
    /*
     * Replace this with TIM PWM code when servo PWM is ready.
     *
     * 0 degree  -> about 1.0 ms pulse
     * 90 degree -> about 1.5 ms pulse
     */
    (void)degree;
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
    UsartPrint((const char *)msg);
}

static CPU_BOOLEAN AppUsartTryGetLine(CPU_CHAR *buf, CPU_INT16U buf_len)
{
    static CPU_CHAR   rx_buf[APP_CMD_LINE_LEN];
    static CPU_INT16U ix = 0u;

    CPU_CHAR ch;

    /*
     * If your HW5 used a different USART, replace USART3 below.
     */
    if (USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == RESET) {
        return DEF_FALSE;
    }

    ch = (CPU_CHAR)USART_ReceiveData(USART3);

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
