#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/conn.h>

#define SLEEP_TIME_MS 1000

#define INT0_NODE DT_ALIAS(int0)
#define I2C0_NODE DT_ALIAS(trackpad0)

#define TOUCH_STATE_ADDR 0x10

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
// static const struct gpio_dt_spec int0 = GPIO_DT_SPEC_GET(INT0_NODE, gpios);
static const struct i2c_dt_spec trackpad = I2C_DT_SPEC_GET(I2C0_NODE);

struct bt_conn *my_conn = NULL;
static struct k_work adv_work;

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static unsigned char url_data[] = {
    0x17, '/', '/', 'o', 'b', 'j', 'e', 'c', 't', '1', '0',
    '3', '7', '.', 'd', 'e', 'v'
};

static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_URI, url_data, sizeof(url_data)),
};

static void adv_work_handler(struct k_work *work) {
  int ret = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
  if (ret < 0) return 0;
}

static void advertising_start(void) {
  k_work_submit(&adv_work);
}

void on_connected(struct bt_conn *conn, uint8_t err) {
  if (err) return;

  my_conn = bt_conn_ref(conn);
  gpio_pin_set_dt(&led, 1);
}

void on_disconnected(struct bt_conn *conn, uint8_t reason) {
  bt_conn_unref(my_conn);
  gpio_pin_set_dt(&led, 0);
}

void on_recycled(void) {
  advertising_start();
}

struct bt_conn_cb connection_callbacks = {
    .connected = on_connected,
    .disconnected = on_disconnected,
    .recycled = on_recycled,
};

/*
void int0_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
  LOG_INF("Interrupt triggered on pin %d", int0.pin);
}

static struct gpio_callback int0_cb_data;
*/

int main(void) {
  int ret;

  if (!gpio_is_ready_dt(&led)) return 0;
  ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
  if (ret < 0) return 0;
  gpio_pin_set_dt(&led, 0);

  /*
  ret = gpio_pin_interrupt_configure_dt(&int0, GPIO_INT_EDGE_TO_ACTIVE);
  if (ret < 0) {
    return 0;
  }

  gpio_init_callback(&int0_cb_data, int0_callback, BIT(int0.pin));
  gpio_add_callback(int0.port, &int0_cb_data);
  */
  ret = bt_conn_cb_register(&connection_callbacks);
  if (ret < 0) return 0;

  ret = bt_enable(NULL);
  if (ret < 0) return 0;

  k_work_init(&adv_work, adv_work_handler);
  advertising_start();

  if (!device_is_ready(trackpad.bus)) return 0;

  while (1) {
    uint8_t touch_data[5] = {0};

    ret = i2c_burst_read_dt(&trackpad, TOUCH_STATE_ADDR, touch_data, sizeof(touch_data));
    if (ret < 0) return 0;

    k_msleep(SLEEP_TIME_MS);
  }
  return 0;
}
