#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>

#include <hal/nrf_radio.h>
#include <hal/nrf_ppi.h>
#include <hal/nrf_timer.h>
#include <hal/nrf_egu.h>

#include <nrfx_timer.h>

#include "rfx.h"

#define MAXSAMP (32*1024)
// #define MAXSAMP (512)

LOG_MODULE_REGISTER(iqcap, LOG_LEVEL_ERR);
// LOG_MODULE_REGISTER(iqcap, LOG_LEVEL_INF);

uint32_t iq_buf[MAXSAMP];

#define LED_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);

#define TRIG_NODE DT_ALIAS(trig)
static const struct gpio_dt_spec trig = GPIO_DT_SPEC_GET(TRIG_NODE, gpios);
static struct gpio_callback trig_cb_data;

const struct device *const uart_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);

static const nrfx_timer_t timer = NRFX_TIMER_INSTANCE(2);

typedef struct {
    uint32_t capture_len;
    uint32_t trigger_delay_cycles;
} config_t;

config_t config = {
    .capture_len = MAXSAMP,
    .trigger_delay_cycles = 1600,
};


static bool armed = false;

void trigger(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    if (!armed) {
        return;
    }

    if (config.trigger_delay_cycles == 0) {
        radio_trigger_iq_capture();
    } else {
        nrfx_timer_enable(&timer);
    }

    armed = false;
    LOG_INF("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
}

static void uputchar(uint8_t c)
{
    uart_poll_out(uart_dev, c);
}

static void uart_write(const uint8_t *buf, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        uputchar(buf[i]);
    }
}

static void print_wav_header()
{
    int channels = 2;
    int bits_per_sample = 16;
    int data_bytes = config.capture_len * 4;
    int file_size = 36 + data_bytes;
    int sample_rate = 16000000;
    int byte_rate = sample_rate * channels * bits_per_sample / 8;

    uputchar('R'); uputchar('I'); uputchar('F'); uputchar('F');
    uputchar(((uint8_t *)&file_size)[0]);
    uputchar(((uint8_t *)&file_size)[1]);
    uputchar(((uint8_t *)&file_size)[2]);
    uputchar(((uint8_t *)&file_size)[3]);
    uputchar('W'); uputchar('A'); uputchar('V'); uputchar('E');
    uputchar('f'); uputchar('m'); uputchar('t'); uputchar(' ');
    uputchar(16); uputchar(0); uputchar(0); uputchar(0); // Subchunk1Size
    uputchar(1); uputchar(0); // AudioFormat (PCM)
    uputchar(channels); uputchar(0); // NumChannels
    uputchar(((uint8_t *)&sample_rate)[0]);
    uputchar(((uint8_t *)&sample_rate)[1]);
    uputchar(((uint8_t *)&sample_rate)[2]);
    uputchar(((uint8_t *)&sample_rate)[3]);
    uputchar(((uint8_t *)&byte_rate)[0]);
    uputchar(((uint8_t *)&byte_rate)[1]);
    uputchar(((uint8_t *)&byte_rate)[2]);
    uputchar(((uint8_t *)&byte_rate)[3]);
    uputchar(4); uputchar(0); // BlockAlign
    uputchar(bits_per_sample); uputchar(0); // BitsPerSample
    uputchar('d'); uputchar('a'); uputchar('t'); uputchar('a');
    uputchar(((uint8_t *)&data_bytes)[0]);
    uputchar(((uint8_t *)&data_bytes)[1]);
    uputchar(((uint8_t *)&data_bytes)[2]);
    uputchar(((uint8_t *)&data_bytes)[3]);
}

void timer_event_handler(nrf_timer_event_t event_type, void *p_context)
{
    switch(event_type) {
        case NRF_TIMER_EVENT_COMPARE0:
            nrf_radio_task_trigger(NRF_RADIO, RFX_RADIO_TASK_IQCAP);
            break;
        
        default:
            break;
    }
}

static void config_timer() {

    // Count at 16 MHz (same as radio IQ capture rate)
    nrfx_timer_config_t timer_cfg = {
        .frequency = NRFX_MHZ_TO_HZ(16),
        .mode      = NRF_TIMER_MODE_TIMER,
        .bit_width = NRF_TIMER_BIT_WIDTH_32,
        .p_context = &timer,
    };
    // Uninit so we can change delay if already initialized
    nrfx_timer_uninit(&timer);
    // Initialize timer
    nrfx_timer_init(&timer, &timer_cfg, timer_event_handler);


    // Connect the nrfx irq handler which then calls our event handler
    IRQ_DIRECT_CONNECT(TIMER2_IRQn, 0, nrfx_timer_2_irq_handler, 0);
    irq_enable(TIMER2_IRQn);

    nrfx_timer_clear(&timer);
    // Use CC0 for trigger delay
    // Enable shortcut to stop and reset the timer
    nrfx_timer_extended_compare(&timer, NRF_TIMER_CC_CHANNEL0, config.trigger_delay_cycles,
            (NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK | NRF_TIMER_SHORT_COMPARE0_STOP_MASK), true);
}


int main(void)
{

    const struct device *const dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    uint32_t dtr = 0;

    gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    k_sleep(K_MSEC(300));
    gpio_pin_set_dt(&led, 0);
    k_sleep(K_MSEC(200));
    gpio_pin_set_dt(&led, 1);
    k_sleep(K_MSEC(300));
    gpio_pin_set_dt(&led, 0);
    k_sleep(K_MSEC(200));
    gpio_pin_set_dt(&led, 1);
    k_sleep(K_MSEC(300));
    gpio_pin_set_dt(&led, 0);

    // Wait for USB connection
    while (usb_enable(NULL)) { }

    for (int i = 0; !dtr && i < 100; i++) {
        uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
        k_sleep(K_MSEC(100));
    }

    k_sleep(K_MSEC(1000));

    LOG_INF("Console Connected!");

    init_radio(0, 2400);
    radio_set_iq_capture(&iq_buf[0], config.capture_len);

    int ret = radio_start_rx();
    k_sleep(K_MSEC(1000));
    LOG_INF("Radio started in RX mode");

    config_timer();
    LOG_INF("Timer configured");

    gpio_pin_configure_dt(&trig, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_interrupt_configure_dt(&trig, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&trig_cb_data, trigger, BIT(trig.pin));
    gpio_add_callback(trig.port, &trig_cb_data);
    LOG_INF("GPIO %d configured as trigger", trig.pin);

    int count = 0;

    for (;;) {
        k_sleep(K_MSEC(100));
        armed = true;
        LOG_INF("Armed for capture %d", ++count);
        gpio_pin_set_dt(&led, 1);
        
        memset(iq_buf, 0x55, sizeof(iq_buf));
        nrf_radio_event_clear(NRF_RADIO, RFX_RADIO_EVENT_IQCAPEND);
        nrf_radio_event_clear(NRF_RADIO, RFX_RADIO_EVENT_IQCAPSTART);

        if (nrf_radio_state_get(NRF_RADIO) != NRF_RADIO_STATE_RX) {
            LOG_WRN("Radio not in RX, restarting RX");
            ret = radio_start_rx();
            if (ret) {
                LOG_ERR("Failed to start RX\n");
                continue;
            }
            k_sleep(K_MSEC(100));
        }

        // Wait for capture
        while (!nrf_radio_event_check(NRF_RADIO, RFX_RADIO_EVENT_IQCAPSTART)) {
            k_sleep(K_USEC(100));
        }
        gpio_pin_set_dt(&led, 0);
        LOG_INF("IQ Capture Started");

        while (!nrf_radio_event_check(NRF_RADIO, RFX_RADIO_EVENT_IQCAPEND)) {
            k_sleep(K_USEC(100));
        }

        LOG_INF("Captured IQ samples");

        print_wav_header();
        for (int i = 0; i < config.capture_len; i++) {
            if (i % 16 == 0) {
                // Periodically yield to allow USB stack to process
                k_sleep(K_MSEC(1));
            }
            uint32_t val = iq_buf[i];
            // sign extend 12 bit samples
            int16_t i_sample = ((int)(val << 20)) / (1<<20);
            int16_t q_sample = ((int)(val << 8)) / (1<<20);

            uint8_t buf[] = {
                ((uint8_t *)&i_sample)[0], ((uint8_t *)&i_sample)[1],
                ((uint8_t *)&q_sample)[0], ((uint8_t *)&q_sample)[1],
            };

            uart_write(buf, sizeof(buf));
        }

        k_sleep(K_MSEC(1));
    }
}