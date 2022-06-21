/**
 * @file EmbebbedStudioMain.c
 * @author João Ferreira (joaoprcf@ua.pt), Daniel Oliveira (danielsoliveira@ua.pt), João Carvalho
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

#define DEBUGREADINGS 1
#define GPIO0_NID DT_NODELABEL(gpio0) ///< Alias for the node label
#define SW4_NODE 0x1c                 ///< Alias for external button 0
#define SW5_NODE 0x1d                 ///< Alias for external button 1
#define SW6_NODE 0x1e                 ///< Alias for external button 2
#define SW7_NODE 0x1f                 ///< Alias for external button 3
#define BUTTON_NUM 4                  ///< Total number of buttons
#define KEYBOARD_EVENT 4

#define button_reader_prio 1
#define value_filter_prio 1
#define controller_prio 1
#define terminal_reader_prio 1

#define button_reader_period 20
#define value_filter_period 1000
#define controller_period 200

#define STACK_SIZE 1024
#define ARRAY_SIZE 5
#define BUFFER_SIZE 10
#define TERMINAL_BUFFER_SIZE 23
#define SECONDS_IN_A_DAY 86400

#define ADC_NID DT_NODELABEL(adc)
#define ADC_RESOLUTION 10
#define ADC_GAIN ADC_GAIN_1_4
#define ADC_REFERENCE ADC_REF_VDD_1_4
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40)
#define ADC_CHANNEL_ID 1
#define ADC_CHANNEL_INPUT NRF_SAADC_INPUT_AIN1

#define PWM_LED0_NODE DT_ALIAS(pwm_led0)

#if DT_NODE_HAS_STATUS(PWM_LED0_NODE, okay)
#define PWM_CTLR DT_PWMS_CTLR(PWM_LED0_NODE)
#define PWM_CHANNEL DT_PWMS_CHANNEL(PWM_LED0_NODE)
#define PWM_FLAGS DT_PWMS_FLAGS(PWM_LED0_NODE)
#else
#error "Unsupported board: pwm-led0 devicetree alias is not defined"
#define PWM_CTLR DT_INVALID_NODE
#define PWM_CHANNEL 0
#define PWM_FLAGS 0
#endif

struct k_fifo fifo_ab;

uint16_t adcbuffer[ARRAY_SIZE];
_Atomic uint16_t avg_value = 0;

_Atomic bool manualMode = true;
_Atomic uint32_t manualIntensity = 0;

struct data_item_t
{
    void *fifo_reserved; /* 1st word reserved for use by FIFO */
    uint8_t type;        /* Event type */
};

static char SWNODE[] = {
    SW4_NODE,
    SW5_NODE,
    SW6_NODE,
    SW7_NODE};

typedef struct debouncer
{
    unsigned int dcIndex;
    char state;
};

typedef struct st_rule
{
    uint32_t from;
    uint32_t to;
    uint32_t intensity;

} Rule;

bool isRuleValid(uint32_t seconds, Rule *rule)
{
    if (rule->from < rule->to && seconds >= rule->from && seconds < rule->to)
        return true;

    if (rule->from > rule->to && (seconds >= rule->from || seconds < rule->to))
        return true;

    return false;
}

static struct debouncer buttondb[BUTTON_NUM];
const struct device *gpio0_dev[BUTTON_NUM];
const struct device *pwm = NULL;

const struct device *adc_dev = NULL;
_Atomic uint8_t ptr = 0;

static Rule rules[20];
uint32_t rules_size = 0;
struct k_mutex rule_lock;

char terminalBuffer[TERMINAL_BUFFER_SIZE] = {0};
_Atomic int idxBuffer = 0;

K_THREAD_STACK_DEFINE(button_reader_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(value_filter_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(controller_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(terminal_reader_stack, STACK_SIZE);

struct k_thread button_reader_data;
struct k_thread value_filter_data;
struct k_thread controller_data;
struct k_thread terminal_reader_data;

/* Create task IDs */
k_tid_t button_reader_tid;
k_tid_t value_filter_tid;
k_tid_t controller_tid;
k_tid_t terminal_reader_tid;

void thread_button_reader(void *args);
void thread_value_filter(void *args);
void thread_controller(void *args);
void thread_terminal_reader(void *args);

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
        // printk("adc_sample(): error, must bind to adc first \n\r");
        return -1;
    }
    ret = adc_read(adc_dev, &sequence);
    if (ret)
    {
        // printk("adc_read() failed with code %d\n", ret);
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
 * @brief Filter array values from the adc buffer
 *
 * @return int16_t
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

const uint32_t DECIMAL_HOUR = 10 * 60 * 60;
const uint32_t UNIT_HOUR = 60 * 60;
const uint32_t DECIMAL_MINUTE = 10 * 60;
const uint32_t UNIT_MINUTE = 1 * 60;
const uint32_t DECIMAL_SECONDS = 10;
const uint32_t UNIT_SECONDS = 1;

uint32_t timeFormatToSeconds(char *cmd)
{
    return (cmd[0] - '0') * DECIMAL_HOUR + (cmd[1] - '0') * UNIT_HOUR + (cmd[3] - '0') * DECIMAL_MINUTE + (cmd[4] - '0') * UNIT_MINUTE + (cmd[6] - '0') * DECIMAL_SECONDS + (cmd[7] - '0') * UNIT_SECONDS;
}

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

    /* ADC setup: bind and initialize */
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

    k_fifo_init(&fifo_ab);

    printk("Manual mode is active\n");
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

void thread_button_reader(void *args)
{
    int64_t fin_time = 0, release_time = 0;
    release_time = k_uptime_get() + button_reader_period;
    // logic
    struct data_item_t data_event;
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
                    k_fifo_put(&fifo_ab, &data_event);
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

void thread_terminal_reader(void *args)
{
    uint8_t c;

    struct data_item_t data_event;

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
            // console_putchar()
            console_putchar('\n');
            data_event.type = KEYBOARD_EVENT;
            k_fifo_put(&fifo_ab, &data_event);
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
