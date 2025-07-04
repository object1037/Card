#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/conn.h>
#include <bluetooth/services/hids.h>

#include "mtch6102.h"

#define SLEEP_TIME_MS 20

#define INT0_NODE DT_ALIAS(int0)
#define I2C0_NODE DT_ALIAS(trackpad0)

#define DEVICE_NAME_LEN (sizeof(CONFIG_BT_DEVICE_NAME) - 1)

struct pointer_pos {
  int16_t x_val;
  int16_t y_val;
} position;
bool is_touched = false;

struct bt_conn *connection;
bool in_boot_mode;

BT_HIDS_DEF(hids_obj, 3);

K_MSGQ_DEFINE(hids_queue, sizeof(struct pointer_pos), 10, 4);

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct i2c_dt_spec trackpad = I2C_DT_SPEC_GET(I2C0_NODE);

static struct k_work adv_work, hids_work;
volatile bool is_adv_running = false;

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE, (BT_APPEARANCE_HID_TOUCHPAD >> 0) & 0xFF, (BT_APPEARANCE_HID_TOUCHPAD >> 8) & 0xFF),
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_HIDS_VAL)),
};

static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, DEVICE_NAME_LEN),
};

static void hids_pm_evt_handler(enum bt_hids_pm_evt evt, struct bt_conn *conn) {
  switch (evt) {
    case BT_HIDS_PM_EVT_BOOT_MODE_ENTERED:
      in_boot_mode = true;
      break;

    case BT_HIDS_PM_EVT_REPORT_MODE_ENTERED:
      in_boot_mode = false;
      break;

    default:
      break;
  }
}

static void hid_init(void) {
  int ret;
  struct bt_hids_init_param hids_init_param = {0};
  struct bt_hids_inp_rep *hids_inp_rep;

  static const uint8_t report_map[] = {
      0x05, 0x01, /* Usage Page (Generic Desktop) */
      0x09, 0x02, /* Usage (Mouse) */

      0xA1, 0x01, /* Collection (Application) */

      0x09, 0x01,       /* Usage (Pointer) */
      0xA1, 0x00,       /* Collection (Physical) */
      0x05, 0x01,       /* Usage Page (Generic Desktop) */
      0x09, 0x30,       /* Usage (X) */
      0x09, 0x31,       /* Usage (Y) */
      0x16, 0x01, 0xFE, /* Logical minimum (-511) */
      0x26, 0xFF, 0x01, /* Logical maximum (511) */
      0x75, 0x0A,       /* Report Size (10) */
      0x95, 0x02,       /* Report Count (2) */
      0x81, 0x06,       /* Input (Data, Variable, Relative) */

      0x05, 0x09, /* Usage Page (Buttons) */
      0x19, 0x01, /* Usage Minimum (01) */
      0x29, 0x03, /* Usage Maximum (03) */
      0x15, 0x00, /* Logical Minimum (0) */
      0x25, 0x01, /* Logical Maximum (1) */
      0x75, 0x01, /* Report Size (1) */
      0x95, 0x03, /* Report Count (3) */
      0x81, 0x02, /* Input (Data, Variable, Absolute) */
      0x75, 0x01, /* Report Size (1) */
      0x95, 0x01, /* Report Count (1) */
      0x81, 0x01, /* Input (Constant) for padding */
      0xC0,       /* End Collection (Physical) */
      0xC0,       /* End Collection (Application) */
  };

  hids_init_param.rep_map.data = report_map;
  hids_init_param.rep_map.size = sizeof(report_map);

  hids_init_param.info.bcd_hid = 0x0111;       // HID version 1.11
  hids_init_param.info.b_country_code = 0x00;  // No localization
  hids_init_param.info.flags = (BT_HIDS_REMOTE_WAKE | BT_HIDS_NORMALLY_CONNECTABLE);

  hids_inp_rep = &hids_init_param.inp_rep_group_init.reports[0];
  hids_inp_rep->size = 3;
  hids_inp_rep->id = 0;
  hids_init_param.inp_rep_group_init.cnt++;

  hids_init_param.is_mouse = true;
  hids_init_param.pm_evt_handler = hids_pm_evt_handler;

  ret = bt_hids_init(&hids_obj, &hids_init_param);
}

static void pointer_movement_send(int16_t x_delta, int16_t y_delta) {
  if (!connection) return;

  x_delta = MAX(MIN(x_delta, 511), -511);
  y_delta = MAX(MIN(y_delta, 511), -511);

  if (in_boot_mode) {
    bt_hids_boot_mouse_inp_rep_send(&hids_obj, connection, NULL, (int8_t)x_delta, (int8_t)y_delta, NULL);
  } else {
    uint8_t buffer[3];

    uint16_t ux = (uint16_t)(x_delta & 0x03FF);
    uint16_t uy = (uint16_t)(y_delta & 0x03FF);

    buffer[0] = (uint8_t)(ux & 0xFF);
    buffer[1] = (uint8_t)(((ux >> 8) & 0x03) | ((uy & 0x3F) << 2));
    buffer[2] = (uint8_t)(((uy >> 6) & 0x0F));

    bt_hids_inp_rep_send(&hids_obj, connection, 0, buffer, sizeof(buffer), NULL);
  }
}

static void mouse_handler(struct k_work *work) {
  struct pointer_pos pos;

  while (!k_msgq_get(&hids_queue, &pos, K_NO_WAIT)) {
    pointer_movement_send(pos.x_val, pos.y_val);
  }
}

static void adv_work_handler(struct k_work *work) {
  if (is_adv_running) return;

  int ret = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
  if (ret) return;
  is_adv_running = true;
}

static void advertising_start(void) {
  k_work_submit(&adv_work);
}

static void on_connected(struct bt_conn *conn, uint8_t err) {
  is_adv_running = false;
  if (err) return;

  err = bt_hids_connected(&hids_obj, conn);
  if (err) return;

  connection = conn;
  in_boot_mode = false;

  gpio_pin_set_dt(&led, 1);
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason) {
  bt_hids_disconnected(&hids_obj, conn);

  connection = NULL;

  advertising_start();
}

void on_recycled(void) {
  advertising_start();
}

static void pairing_complete(struct bt_conn *conn, bool bonded) {
  gpio_pin_set_dt(&led, 0);
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason) {
  for (size_t i = 0; i < 3; i++) {
    gpio_pin_set_dt(&led, 0);
    k_msleep(200);
    gpio_pin_set_dt(&led, 1);
    k_msleep(200);
  }
  gpio_pin_set_dt(&led, 0);
  return;
}

static void trackpad_get() {
  int ret;
  int16_t touch_x, touch_y;
  gesture_t gesture;

  bool was_touched = is_touched;

  ret = get_touch_data(&trackpad, &touch_x, &touch_y, &is_touched, &gesture);
  if (ret < 0) return;

  int16_t x_new = touch_y;
  int16_t y_new = TOUCH_X_MAX - touch_x;

  if (!was_touched && is_touched) {
    position.x_val = x_new;
    position.y_val = y_new;
  } else if (was_touched && is_touched) {
    int16_t x_delta = x_new - position.x_val;
    int16_t y_delta = y_new - position.y_val;

    if (x_delta || y_delta) {
      position.x_val = x_new;
      position.y_val = y_new;

      struct pointer_pos delta = {x_delta, y_delta};
      ret = k_msgq_put(&hids_queue, &delta, K_NO_WAIT);
      if (ret) return;

      if (k_msgq_num_used_get(&hids_queue) == 1) {
        k_work_submit(&hids_work);
      }
    }
  }
}

struct bt_conn_cb connection_callbacks = {
    .connected = on_connected,
    .disconnected = on_disconnected,
    .recycled = on_recycled,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
    .pairing_complete = pairing_complete,
    .pairing_failed = pairing_failed,
};

int main(void) {
  int ret;

  if (!gpio_is_ready_dt(&led)) return 0;
  ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
  if (ret < 0) return 0;
  gpio_pin_set_dt(&led, 0);

  ret = bt_conn_cb_register(&connection_callbacks);
  if (ret < 0) return 0;
  ret = bt_conn_auth_cb_register(NULL);
  if (ret < 0) return 0;
  ret = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
  if (ret < 0) return 0;

  hid_init();

  ret = bt_enable(NULL);
  if (ret < 0) return 0;

  k_work_init(&hids_work, mouse_handler);
  k_work_init(&adv_work, adv_work_handler);

  advertising_start();

  if (!device_is_ready(trackpad.bus)) return 0;

  while (1) {
    trackpad_get();

    k_msleep(SLEEP_TIME_MS);
  }
  return 0;
}
