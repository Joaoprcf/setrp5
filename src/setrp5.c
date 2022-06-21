/**
 * @file Setrp5.c
 * @author João Ferreira (joaoprcf@ua.pt), Daniel Oliveira (danielsoliveira@ua.pt), João Carvalho (jpmffc@ua.pt)
 * @brief SETR Lab 5 project
 * @version 0.1
 * @date 2022-06-15
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/adc.h>
#include <sys/printk.h>
#include <sys/util.h>
#include <sys/__assert.h>
#include <string.h>
#include <timing/timing.h>
#include <stdlib.h>
#include <stdio.h>
#include <hal/nrf_saadc.h>
#include <drivers/pwm.h>
#include <inttypes.h>
#include <console/console.h>
#include <time.h>

#define DEBUGREADINGS 1               ///< Alias for DEBUGREADINGS
#define GPIO0_NID DT_NODELABEL(gpio0) ///< Alias for the node label
#define SW4_NODE 0x1c                 ///< Alias for external button 0
#define SW5_NODE 0x1d                 ///< Alias for external button 1
#define SW6_NODE 0x1e                 ///< Alias for external button 2
#define SW7_NODE 0x1f                 ///< Alias for external button 3
#define BUTTON_NUM 4                  ///< Total number of buttons
#define KEYBOARD_EVENT 4              ///< Alias for the KEYBOARD_EVENT

#define button_reader_prio 1   ///< Priority for the button reader thread
#define value_filter_prio 1    ///< Priority for the filter thread
#define controller_prio 1      ///< Priority for the controller thread
#define terminal_reader_prio 1 ///< Priority for the terminal reader thread

#define button_reader_period 20  ///< Period for reading buttons
#define value_filter_period 1000 ///< Period for filter
#define controller_period 200    ///< Period for controller

#define STACK_SIZE 1024         ///< Number for the stack size
#define ARRAY_SIZE 5            ///< Size of the array
#define BUFFER_SIZE 10          ///< Size of the buffer
#define TERMINAL_BUFFER_SIZE 23 ///< Size of the terminal buffer
#define SECONDS_IN_A_DAY 86400  ///< Number of seconds in a day

#define ADC_NID DT_NODELABEL(adc)                                        ///< Label of the ADC
#define ADC_RESOLUTION 10                                                ///< ADC resolutation
#define ADC_GAIN ADC_GAIN_1_4                                            ///< ADC gain
#define ADC_REFERENCE ADC_REF_VDD_1_4                                    ///< ADC reference
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40) ///< ADC time of acquisition
#define ADC_CHANNEL_ID 1                                                 ///< ADC id channel
#define ADC_CHANNEL_INPUT NRF_SAADC_INPUT_AIN1                           ///< ADC input channel

#define PWM_LED0_NODE DT_ALIAS(pwm_led0) ///< Alias for the PWM node

#if DT_NODE_HAS_STATUS(PWM_LED0_NODE, okay)
#define PWM_CTLR DT_PWMS_CTLR(PWM_LED0_NODE)       ///< PWM controller
#define PWM_CHANNEL DT_PWMS_CHANNEL(PWM_LED0_NODE) ///< PWM channel
#define PWM_FLAGS DT_PWMS_FLAGS(PWM_LED0_NODE)     ///< PWM flags
#else
#error "Unsupported board: pwm-led0 devicetree alias is not defined"
#define PWM_CTLR DT_INVALID_NODE ///< PWM controller
#define PWM_CHANNEL 0            ///< PWM channel
#define PWM_FLAGS 0              ///< PWM flags
#endif

uint16_t adcbuffer[ARRAY_SIZE]; ///< ADC buffer
_Atomic uint16_t avg_value = 0; ///< Avg value for the readings

_Atomic bool manualMode = true;       ///< Bool for the Manual/Auto Mode
_Atomic uint32_t manualIntensity = 0; ///< Intensity for Manual mode

/**
 * @brief Struct for the Button

  */
static char SWNODE[] = {
    SW4_NODE,
    SW5_NODE,
    SW6_NODE,
    SW7_NODE};

/**
 * @brief Struct for the Debouncer

  */
typedef struct debouncer
{
    unsigned int dcIndex;
    char state;
};
/**
 * @brief Struct for Rules

  */
typedef struct st_rule
{
    uint32_t from;
    uint32_t to;
    uint32_t intensity;

} Rule;

/**
 * Verification of Rules
 * @param  seconds               uint32_t
 * @param  rule                  Rule
 * @return         boolean
 */
bool isRuleValid(uint32_t seconds, Rule *rule)
{
    if (rule->from < rule->to && seconds >= rule->from && seconds < rule->to)
        return true;

    if (rule->from > rule->to && (seconds >= rule->from || seconds < rule->to))
        return true;

    return false;
}

static struct debouncer buttondb[BUTTON_NUM]; ///< Struct Debouncer
const struct device *gpio0_dev[BUTTON_NUM];   ///< Struct Device for the Board
const struct device *pwm = NULL;              ///< Struct Device for the PWM

const struct device *adc_dev = NULL; ///< Struct Device for the ADC
_Atomic uint8_t ptr = 0;             ///<

static Rule rules[20];    ///< Struct Rules
uint32_t rules_size = 0;  ///< Size of Rules
struct k_mutex rule_lock; ///< Mutex for Rules

char terminalBuffer[TERMINAL_BUFFER_SIZE] = {0}; ///< Size of the buffer for the reading terminal
_Atomic int idxBuffer = 0;                       ///< ID of the Atomic buffer

K_THREAD_STACK_DEFINE(button_reader_stack, STACK_SIZE);   ///< Size fo Stack for the Thread reading buttons
K_THREAD_STACK_DEFINE(value_filter_stack, STACK_SIZE);    ///< Size fo Stack for the Thread filter
K_THREAD_STACK_DEFINE(controller_stack, STACK_SIZE);      ///< Size fo Stack for the Thread controller
K_THREAD_STACK_DEFINE(terminal_reader_stack, STACK_SIZE); ///< Size fo Stack for the Thread reading terminal

struct k_thread button_reader_data;   ///< Struct for the thread reading buttons
struct k_thread value_filter_data;    ///< Struct for the thread Filter
struct k_thread controller_data;      ///< Struct for the thread controller
struct k_thread terminal_reader_data; ///< Struct for the thread reading terminal

/* Create task IDs */
k_tid_t button_reader_tid;   ///< ID for button thread
k_tid_t value_filter_tid;    ///< ID for Filter thread
k_tid_t controller_tid;      ///< ID for controller thread
k_tid_t terminal_reader_tid; ///< ID for terminal reader thread

void thread_button_reader(void *args);   ///< Thread for reading buttons
void thread_value_filter(void *args);    ///< Thread for the filter of values read
void thread_controller(void *args);      ///< Thread for the controller
void thread_terminal_reader(void *args); ///< Thread for reading the terminal

/**
 * @brief Retrieve an ADC sample a buffer
 *
 * @return int

  */
static const struct adc_channel_cfg my_channel_cfg = {
    .gain = ADC_GAIN,
    .reference = ADC_REFERENCE,
    .acquisition_time = ADC_ACQUISITION_TIME,
    .channel_id = ADC_CHANNEL_ID,
    .input_positive = ADC_CHANNEL_INPUT};

/**
 * Samples values of ADC
 * @param  adc_sample_buffer               uint16_t
 * @return                   int
 */
static int adc_sample(uint16_t *adc_sample_buffer)
{
    int ret;
    const struct adc_sequence sequence = {
        .channels = BIT(ADC_CHANNEL_ID),
        .buffer = adc_sample_buffer,
        .buffer_size = sizeof(adc_sample_buffer),
        .resolution = ADC_RESOLUTION,
    };

    if (adc_dev == NULL)
    {
        return -1;
    }
    ret = adc_read(adc_dev, &sequence);
    if (ret)
    {
        ;
    }
    else if (adc_sample_buffer[0] > 1023)
    {
        return -1;
    }
    else
    {
        adcbuffer[ptr] = adc_sample_buffer[0];
        ptr = (ptr + 1) % ARRAY_SIZE;
    }

    return ret;
}

/**
 * Filter array values from the adc buffer
 * @param  arr               uint16_t
 * @return     int16_t
 */
int16_t filter(uint16_t *arr)
{
    int i;
    uint16_t sum = 0;

    for (i = 0; i < ARRAY_SIZE; i++)
    {
        sum += arr[i];
    }
    float mean = sum / (float)ARRAY_SIZE;

    sum = 0;
    int ctn = 0;
    for (i = 0; i < ARRAY_SIZE; i++)
    {
        if (mean * 2.0 > (float)arr[i] && mean * 0.5 < (float)arr[i])
        {
            sum += arr[i];
            ctn += 1;
        }
    }
    return ctn ? (uint16_t)(sum / (float)ctn) : (int)mean;
}

/**
 * Verify commands
 * @param  cmd               char
 * @return     boolean
 */
bool verifyCommand(char *cmd)
{
    char colonArray[] = {2, 5, 11, 14};

    if (cmd[TERMINAL_BUFFER_SIZE - 1] != '\0')
        return false;
    for (int idx = 0; idx < TERMINAL_BUFFER_SIZE - 1; idx++)
    {
        bool shouldBeNum = true;
        if (cmd[idx] == '\0')
        {
            printk("Should not be s0\n");
            return false;
        }
        if (idx == 8)
        {
            shouldBeNum = false;
            if (cmd[idx] != '-')
            {
                printk("Should be - on idx %d\n", idx);
                return false;
            }
        }
        if (idx == 17)
        {
            shouldBeNum = false;
            if (cmd[idx] != '=')
            {
                printk("Should be = on idx %d\n", idx);
                return false;
            }
        }
        for (int colonidx = 0; colonidx < 4; colonidx++)
        {

            if (idx == colonArray[colonidx])
            {
                shouldBeNum = false;
                if (cmd[idx] != ':')
                {
                    printk("Should be : on idx %d\n", idx);
                    return false;
                }
            }
        }
        if (shouldBeNum && (cmd[idx] < '0' || cmd[idx] > '9'))
        {
            printk("Should be a valid number on idx %d\n", idx);
            return false;
        }
    }

    int intensity = atoi(cmd + 18);
    if (intensity > 1000)
    {
        printk("Intensity should be less than 1000, it is %d\n", intensity);
        return false;
    }

    return true;
}

const uint32_t DECIMAL_HOUR = 10 * 60 * 60; ///< Alias for DECIMAL_HOUR label
const uint32_t UNIT_HOUR = 60 * 60;         ///< Alias for UNIT_HOUR label
const uint32_t DECIMAL_MINUTE = 10 * 60;    ///< Alias for DECIMAL_MINUTE label
const uint32_t UNIT_MINUTE = 1 * 60;        ///< Alias for UNIT_MINUTE label
const uint32_t DECIMAL_SECONDS = 10;        ///< Alias for DECIMAL_SECONDS label
const uint32_t UNIT_SECONDS = 1;            ///< Alias for UNIT_SECONDS label

/**
 * Formatation of time
 * @param  cmd               char
 * @return     uint32_t
 */
uint32_t timeFormatToSeconds(char *cmd)
{
    return (cmd[0] - '0') * DECIMAL_HOUR + (cmd[1] - '0') * UNIT_HOUR + (cmd[3] - '0') * DECIMAL_MINUTE + (cmd[4] - '0') * UNIT_MINUTE + (cmd[6] - '0') * DECIMAL_SECONDS + (cmd[7] - '0') * UNIT_SECONDS;
}

/**
 * Implementation of Rules
 * @param cmd  char
 */
void addRule(char *cmd)
{
    uint32_t start = timeFormatToSeconds(cmd);
    uint32_t end = timeFormatToSeconds(cmd + 9);
    int intensity = atoi(cmd + 18);
    // open mutex
    k_mutex_lock(&rule_lock, K_FOREVER);
    rules[rules_size].from = start;
    rules[rules_size].to = end;
    rules[rules_size].intensity = intensity;
    rules_size++;
    // close mutex
    k_mutex_unlock(&rule_lock);
    printk("Adding rule %d - %d = %d\n", start, end, intensity);
}
/**
 * Receive desired intensity
 * @param  milli               uint32_t
 * @return       uint32_t
 */
uint32_t getDesiredIntensity(uint32_t milli)
{
    uint32_t totalSeconds = (milli / 1000) % SECONDS_IN_A_DAY;

    int ruleIdx;
    for (ruleIdx = 0; ruleIdx < rules_size; ruleIdx++)
    {

        if (isRuleValid(totalSeconds, &rules[ruleIdx]))
        {
            uint32_t intensity = rules[ruleIdx].intensity;
            return intensity;
        }
    }

    return 0;
}
/**
 * Handler of the commands
 * @param cmd  char
 */
void commandHandler(char *cmd)
{

    if (!verifyCommand(cmd))
    {
        printk("Non valid command!\n");
        return;
    }
    printk("Command verified with success\n");
    addRule(cmd);
}
/**
 * Handler of Events
 * @param event  uint8_t
 */
void eventHandler(uint8_t event)
{
    switch (event)
    {
    case 0:
        printk("\nManual mode is now activated, intensity is %d\n", manualIntensity);
        manualMode = true;
        break;
    case 1:
        memset(terminalBuffer, '\0', sizeof(terminalBuffer));
        idxBuffer = 0;
        printk("Automatic mode is now activated\n");
        printk("Insert rule in format hh:mm:ss-hh:mm:ss=iiii\n: ");
        manualMode = false;
        break;
    case 2:
        if (!manualMode)
            break;
        if (manualIntensity > 10)
            manualIntensity -= 10;
        else
            manualIntensity = 0;
        printk("Intensity is %d\n", manualIntensity);
        break;

    case 3:
        if (!manualMode)
            break;
        if (manualIntensity + 10 < 1000)
            manualIntensity += 10;
        else
            manualIntensity = 1000;
        printk("Intensity is %d\n", manualIntensity);
        break;

    default:
        break;
    }
}
/**
 * Main function
 * @return  int
 */
int main(void)
{

    console_init();

    pwm = DEVICE_DT_GET(PWM_CTLR);
    if (!device_is_ready(pwm))
    {
        printk("Error: PWM device %s is not ready\n", pwm->name);
        return;
    }

    for (int btidx = 0; btidx < BUTTON_NUM; btidx++)
    {
        int ret;
        gpio0_dev[btidx] = device_get_binding(DT_LABEL(GPIO0_NID));
        ret = gpio_pin_configure(gpio0_dev[btidx], SWNODE[btidx], GPIO_INPUT | GPIO_PULL_UP);
        if (ret != 0)
        {
            printk("Error %d: failed to configure external button at index %d\n",
                   ret, btidx);
            return 1;
        }

        buttondb[btidx].dcIndex = 0;
        buttondb[btidx].state = 0;
    }
    k_mutex_init(&rule_lock);

    adc_dev = device_get_binding(DT_LABEL(ADC_NID));
    if (!adc_dev)
    {
        printk("ADC device_get_binding() failed\n");
    }
    int err = adc_channel_setup(adc_dev, &my_channel_cfg);
    if (err)
    {
        printk("adc_channel_setup() failed with error code %d\n", err);
    }

    printk("Manual mode is active\n");

    // thread initialization

    value_filter_tid = k_thread_create(&value_filter_data, value_filter_stack,
                                       K_THREAD_STACK_SIZEOF(value_filter_stack), thread_value_filter,
                                       NULL, NULL, NULL, value_filter_prio, 0, K_NO_WAIT);

    button_reader_tid = k_thread_create(&button_reader_data, button_reader_stack,
                                        K_THREAD_STACK_SIZEOF(button_reader_stack), thread_button_reader,
                                        NULL, NULL, NULL, button_reader_prio, 0, K_NO_WAIT);

    controller_tid = k_thread_create(&controller_data, controller_stack,
                                     K_THREAD_STACK_SIZEOF(controller_stack), thread_controller,
                                     NULL, NULL, NULL, controller_prio, 0, K_NO_WAIT);

    terminal_reader_tid = k_thread_create(&terminal_reader_data, terminal_reader_stack,
                                          K_THREAD_STACK_SIZEOF(terminal_reader_stack), thread_terminal_reader,
                                          NULL, NULL, NULL, terminal_reader_prio, 0, K_NO_WAIT);

    return;
}
/**
 * Thread to read buttons
 * @param args  void
 */
void thread_button_reader(void *args)
{
    int64_t fin_time = 0, release_time = 0;
    release_time = k_uptime_get() + button_reader_period;
    // logic
    while (1)
    {
        fin_time = k_uptime_get();
        for (int btidx = 0; btidx < BUTTON_NUM; btidx++)
        {

            int val;

            val = 1 - gpio_pin_get(gpio0_dev[btidx], SWNODE[btidx]);

            if (val > 0)
            {
                buttondb[btidx].dcIndex += 1;
                if (buttondb[btidx].dcIndex == 3)
                {
                    buttondb[btidx].state = 1;
                    data_event.type = btidx;
                    eventHandler(btidx);
                }
            }
            else
            {
                buttondb[btidx].dcIndex = 0;
                buttondb[btidx].state = 0;
            }
        }

        if (fin_time < release_time)
        {

            k_msleep(release_time - fin_time);
            release_time += button_reader_period;
        }
        while (fin_time >= release_time)
            release_time += button_reader_period;
    }
}
/**
 * Thread filter values
 * @param args  void
 */
void thread_value_filter(void *args)
{
    int64_t fin_time = 0, release_time = 0;
    release_time = k_uptime_get() + value_filter_period;
    uint16_t adc_sample_buffer[BUFFER_SIZE];
    // logic

    while (1)
    {
        fin_time = k_uptime_get();

        int err = adc_sample(adc_sample_buffer);
        uint16_t avg_read = filter(adcbuffer);

        avg_value = 1023 - avg_read;

        if (fin_time < release_time)
        {
            k_msleep(release_time - fin_time);
            release_time += value_filter_period;
        }
        while (fin_time >= release_time)
            release_time += value_filter_period;
    }
}
/**
 * Thread Controller
 * @param args  void
 */
void thread_controller(void *args)
{
    int64_t fin_time = 0, release_time = 0;
    release_time = k_uptime_get() + controller_period;
    // logic

    int error = 0;
    int lastError = 0;
    float correctionP = 0, correctionD = 0;
    float current_value = 0;

    while (1)
    {
        fin_time = k_uptime_get();

        uint32_t desiredIntensity;
        if (manualMode)
        {
            desiredIntensity = manualIntensity;
        }
        else
        {
            k_mutex_lock(&rule_lock, K_FOREVER);
            desiredIntensity = getDesiredIntensity(fin_time);
            k_mutex_unlock(&rule_lock);
        }

        error = desiredIntensity - avg_value;
        int errorDiff = error - lastError;

        correctionP = error * 0.1;
        correctionD = errorDiff * 0.02;
        current_value += correctionP + correctionD;
        if (current_value < 0)
        {
            current_value = 0;
        }
        else if (current_value > 1023)
        {
            current_value = 1023;
        }

        if (manualMode)
        {
            printk("Avg value: %3u, Desired Intensity: %3u, Current led value: %u\n", avg_value, desiredIntensity, (int)current_value);
            int correction = (int)((correctionP + correctionD) * 100);
            printk("Correction = 0.1 x %d + 0.02 x %d ~= %d.%0d\n", (int)error, (int)errorDiff, correction / 100, abs(correction) % 100);
        }

        int ret = pwm_pin_set_usec(pwm, PWM_CHANNEL, 1023U, 1023 - ((int)current_value), PWM_FLAGS);

        lastError = error;

        if (fin_time < release_time)
        {
            k_msleep(release_time - fin_time);
            release_time += controller_period;
        }
        while (fin_time >= release_time)
            release_time += controller_period;
    }
}
/**
 * Thread to read terminal
 * @param args  void
 */
void thread_terminal_reader(void *args)
{
    uint8_t c;

    while (1)
    {
        c = console_getchar();
        if (manualMode)
        {
            continue;
        };
        console_putchar(c);

        if (c == '\r')
        {
            // read command
            console_putchar('\n');
            data_event.type = KEYBOARD_EVENT;
            commandHandler(terminalBuffer);
            memset(terminalBuffer, '\0', sizeof(terminalBuffer));
            idxBuffer = 0;
            printk("Insert rule in format hh:mm:ss-hh:mm:ss=iiii\n: ");
        }
        else
        {
            terminalBuffer[idxBuffer] = c;
            idxBuffer++;
        }
    }
}
