#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <bluetooth/services/nus.h>

 /* EDA CHANNELS */
#define EDA_CHANNEL      3
#define EDA_RESOLUTION   12
#define EDA_OVERSAMPLING 6
#define EDA_SAMPLE_MS    20
#define EDA_EMA_SCALE    64
#define EDA_TX_WINDOW    100  /* 100 × 20 ms = 2000 ms transmit interval */

/* Pulse sensor (MAX30102 was used in this project) */
#define PPG_ADDR            0x57
#define PPG_REG_FIFO_WR_PTR 0x04
#define PPG_REG_OVF_COUNTER 0x05
#define PPG_REG_FIFO_RD_PTR 0x06
#define PPG_REG_FIFO_DATA   0x07
#define PPG_REG_FIFO_CONFIG 0x08
#define PPG_REG_MODE_CONFIG 0x09
#define PPG_REG_SPO2_CONFIG 0x0A
#define PPG_REG_LED1_PA     0x0C
#define PPG_REG_LED2_PA     0x0D

/* BPM averaging */
#define RATE_SIZE 8

#define BLE_TX_DELAY_MS 50

/* ADC (EDA) channel setup */
static const struct device *adc_dev;
static int16_t eda_buf;

static const struct adc_channel_cfg eda_ch_cfg = {
    .gain             = ADC_GAIN_1_4,
    .reference        = ADC_REF_VDD_1_4,
    .acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
    .channel_id       = EDA_CHANNEL,
    .input_positive   = SAADC_CH_PSELP_PSELP_AnalogInput3,
};

static struct adc_sequence eda_seq = {
    .channels    = BIT(EDA_CHANNEL),
    .buffer      = &eda_buf,
    .buffer_size = sizeof(eda_buf),
    .resolution  = EDA_RESOLUTION,
    .oversampling = EDA_OVERSAMPLING,
    .calibrate    = true,
};

/* PPG channel setup (uses I2C) */
static const struct device *i2c_dev;

static int PPG_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_write(i2c_dev, buf, 2, PPG_ADDR);
}

static int PPG_read_fifo(uint32_t *red_out, uint32_t *ir_out)
{
    uint8_t reg = PPG_REG_FIFO_DATA;
    uint8_t data[6];
    int ret = i2c_write_read(i2c_dev, PPG_ADDR, &reg, 1, data, 6);
    if (ret) return ret;
    *red_out = ((uint32_t)(data[0] & 0x03) << 16) | ((uint32_t)data[1] << 8) | data[2];
    *ir_out  = ((uint32_t)(data[3] & 0x03) << 16) | ((uint32_t)data[4] << 8) | data[5];
    return 0;
}

static uint8_t PPG_samples_available(void)
{
    uint8_t reg, wr, rd;
    reg = PPG_REG_FIFO_WR_PTR;
    i2c_write_read(i2c_dev, PPG_ADDR, &reg, 1, &wr, 1);
    reg = PPG_REG_FIFO_RD_PTR;
    i2c_write_read(i2c_dev, PPG_ADDR, &reg, 1, &rd, 1);
    return (wr - rd + 32) % 32;
}

static int PPG_init(void)
{
    int ret;
    ret = PPG_write(PPG_REG_MODE_CONFIG, 0x40); /* soft reset */
    if (ret) return ret;
    k_msleep(10);

    /* SMP_AVE=8, ROLLOVER, A_FULL=15 */
    ret = PPG_write(PPG_REG_FIFO_CONFIG, 0x7F);
    if (ret) return ret;

    ret = PPG_write(PPG_REG_MODE_CONFIG, 0x03); /* SpO2 mode */
    if (ret) return ret;

    /* ADC_RGE=16384, SR=100Hz, PW=411µs */
    ret = PPG_write(PPG_REG_SPO2_CONFIG, 0x67);
    if (ret) return ret;

    ret = PPG_write(PPG_REG_LED1_PA, 0x0A); /* Red low */
    if (ret) return ret;
    ret = PPG_write(PPG_REG_LED2_PA, 0x3F); /* IR ~12.6 mA */
    if (ret) return ret;

    PPG_write(PPG_REG_FIFO_WR_PTR, 0x00);
    PPG_write(PPG_REG_OVF_COUNTER, 0x00);
    PPG_write(PPG_REG_FIFO_RD_PTR, 0x00);
    return 0;
}

/* SparkFun PBA algorithm (adapted from Arduino example heartRate.h) */

/* DC estimator — same formula as SparkFun averageDCEstimator() */
static int32_t avg_dc_estimator(int32_t *p, uint32_t x)
{
    *p += (((int32_t)x << 15) - *p) >> 4;
    return *p >> 15;
}

/* Beat detector with zero-crossing on DC-removed signal, peak/trough tracking */
static bool check_for_beat(int32_t sample)
{
    static int32_t prev     = 0;
    static int32_t ac_max   = 0, ac_min = 0;
    static bool    pos_edge = false, neg_edge = false;

    bool beat = false;

    if (prev < 0 && sample >= 0) {          /* rising zero-crossing */
        if (ac_max - ac_min > 20 && ac_max - ac_min < 1000) beat = true;
        ac_max   = 0;
        pos_edge = true;
        neg_edge = false;
    }
    if (prev > 0 && sample <= 0) {          /* falling zero-crossing */
        ac_min   = 0;
        neg_edge = true;
        pos_edge = false;
    }
    if (pos_edge && sample > ac_max) ac_max = sample;
    if (neg_edge && sample < ac_min) ac_min = sample;

    prev = sample;
    return beat;
}

/* PPG cakcykatuibs */
#define PPG_STACK_SIZE 1024
#define PPG_PRIORITY   5

static volatile int32_t  bpm_avg;
static volatile uint32_t ir_latest;

static void ppg_thread_fn(void *p1, void *p2, void *p3)
{
    uint8_t  rates[RATE_SIZE] = {0};
    uint8_t  rate_spot = 0;
    int64_t  last_beat = 0;
    int32_t  avg = 0;
    int32_t  dc_reg = 0;
    int64_t  last_sample_ms = k_uptime_get();

    while (1) {
        if (PPG_samples_available() == 0) {
            if (k_uptime_get() - last_sample_ms > 1000) {
                PPG_init();
                last_sample_ms = k_uptime_get();
            }
            k_msleep(5);
            continue;
        }

        uint32_t red, ir;
        if (PPG_read_fifo(&red, &ir)) {
            k_msleep(10);
            continue;
        }

        last_sample_ms = k_uptime_get();
        ir_latest = ir;

        int32_t ir_ac = (int32_t)ir - avg_dc_estimator(&dc_reg, ir);

        if (check_for_beat(ir_ac)) {
            int64_t now   = k_uptime_get();
            int64_t delta = now - last_beat;
            last_beat = now;

            float bpm = 60000.0f / (float)delta;
            if (bpm > 20.0f && bpm < 255.0f) {
                rates[rate_spot++] = (uint8_t)bpm;
                rate_spot %= RATE_SIZE;

                int32_t sum = 0;
                for (int i = 0; i < RATE_SIZE; i++) sum += rates[i];
                avg = sum / RATE_SIZE;
            }
        }

        bpm_avg = (ir < 20000) ? 0 : avg;
    }
}

K_THREAD_DEFINE(ppg_tid, PPG_STACK_SIZE, ppg_thread_fn,
                NULL, NULL, NULL, PPG_PRIORITY, 0, 0);

/* Bluetooth helpers */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, "XIAO_PPG", sizeof("XIAO_PPG") - 1),
};

static struct bt_nus_cb nus_cb = { 0 };

static volatile bool ble_connected = false;

static void ble_dbg(const char *s)
{
    if (ble_connected) {
        bt_nus_send(NULL, s, strlen(s));
    }
}

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        printk("BLE connection failed: %u\n", err);
        ble_connected = false;
    } else {
        printk("BLE connected\n");
        ble_connected = true;
        bt_nus_send(NULL, "CONNECTED\n", 10);
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("BLE disconnected: %u\n", reason);
    ble_connected = false;
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected    = connected,
    .disconnected = disconnected,
};


int main(void)
{
    int ret;
    ret = bt_enable(NULL);
    if (ret) { printk("BT enable failed: %d\n", ret); return 0; }

    ret = bt_nus_init(&nus_cb);
    if (ret) { printk("NUS init failed: %d\n", ret); return 0; }

    k_msleep(500);
    ret = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), NULL, 0);
    if (ret) { printk("Advertising failed: %d\n", ret); return 0; }
    printk("Advertising as XIAO_PPG\n");

    adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc));
    if (!device_is_ready(adc_dev)) { printk("ADC not ready\n"); return 0; }
    adc_channel_setup(adc_dev, &eda_ch_cfg);

    i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
    if (!device_is_ready(i2c_dev)) {
        printk("I2C not ready\n");
    } else if (PPG_init()) {
        printk("PPG init failed\n");
    } else {
        printk("PPG ready\n");
    }

    adc_read(adc_dev, &eda_seq);
    int32_t ema_x   = (int32_t)eda_buf * EDA_EMA_SCALE;
    int     eda_count = 0;

    while (1) {
        adc_read(adc_dev, &eda_seq);
        ema_x = ema_x - (ema_x / EDA_EMA_SCALE) + eda_buf;

        if (++eda_count >= EDA_TX_WINDOW) {
            eda_count = 0;
            int32_t eda_raw = ema_x / EDA_EMA_SCALE;

            char msg[64];
            snprintf(msg, sizeof(msg), "IR:%u,BPM:%d,AvgBPM:%d,PPG:%d\n",
                     (unsigned int)ir_latest, (int)bpm_avg, (int)bpm_avg, (int)eda_raw);
            k_msleep(BLE_TX_DELAY_MS);
            if (ble_connected) {
                int err = bt_nus_send(NULL, msg, strlen(msg));
                if (err == -ENOMEM) {
                    printk("BLE no buffer, retrying...\n");
                    ble_dbg("ENOMEM retry\n");
                    k_msleep(50);
                    err = bt_nus_send(NULL, msg, strlen(msg));
                }
                if (err) {
                    printk("BLE send failed: %d\n", err);
                } else {
                    printk("BLE sent: %s", msg);
                }
            } else {
                printk("BLE not connected\n");
            }
        }

        k_msleep(EDA_SAMPLE_MS);
    }
    return 0;
}
