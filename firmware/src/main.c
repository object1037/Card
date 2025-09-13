#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/settings/settings.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/conn.h>
#include <bluetooth/services/hids.h>

#include <zephyr/pm/device.h>
#include <zephyr/sys/poweroff.h>

#include "mtch6102.h"

#define LED0_NODE DT_ALIAS(led0)
#define INT0_NODE DT_ALIAS(int0)
#define I2C0_NODE DT_ALIAS(trackpad0)

#define DEVICE_NAME_LEN (sizeof(CONFIG_BT_DEVICE_NAME) - 1)

static void mouse_handler(struct k_work *work);
static void adv_work_handler(struct k_work *work);
static void trackpad_work_handler(struct k_work *work);
static void pointer_movement_send(int16_t x_delta, int16_t y_delta, gesture_t gesture);

static struct pos_gesture {
  int16_t x_val;
  int16_t y_val;
  gesture_t gesture;
} posi_gest;
static struct pos_gesture delta_ges_reset = {0};
static struct pos_gesture delta_ges_click = {0, 0, 1};
static bool is_touched = false;

static struct bt_conn *connection;
static bool in_boot_mode;
static volatile bool is_adv_running = false;

BT_HIDS_DEF(hids_obj, 3);

K_MSGQ_DEFINE(hids_queue, sizeof(struct pos_gesture), 5, 1);

static K_WORK_DEFINE(hids_work, mouse_handler);
static K_WORK_DEFINE(adv_work, adv_work_handler);
static K_WORK_DEFINE(trackpad_work, trackpad_work_handler);

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec int_pin = GPIO_DT_SPEC_GET(INT0_NODE, gpios);
static const struct i2c_dt_spec trackpad = I2C_DT_SPEC_GET(I2C0_NODE);

static struct gpio_callback int_cb_data;

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE, (BT_APPEARANCE_HID_TOUCHPAD >> 0) & 0xFF, (BT_APPEARANCE_HID_TOUCHPAD >> 8) & 0xFF),
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_HIDS_VAL)),
};

static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, DEVICE_NAME_LEN),
};

static void mouse_handler(struct k_work *work) {
  struct pos_gesture pos_ges;

  while (!k_msgq_get(&hids_queue, &pos_ges, K_NO_WAIT)) {
    pointer_movement_send(pos_ges.x_val, pos_ges.y_val, pos_ges.gesture);
  }
}

static void adv_work_handler(struct k_work *work) {
  if (is_adv_running) return;

  int ret = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
  if (ret) return;
  is_adv_running = true;
}

static void trackpad_work_handler(struct k_work *work) {
  int ret;
  int16_t x_new, y_new;
  struct pos_gesture delta_ges = {0};

  bool was_touched = is_touched;

  // flip x/y here
  ret = get_touch_data(&trackpad, &y_new, &x_new, &is_touched, &delta_ges.gesture);
  if (ret < 0) return;

  y_new = TOUCH_X_MAX - y_new;

  if (!was_touched && is_touched) {
    posi_gest.x_val = x_new;
    posi_gest.y_val = y_new;
  } else if (was_touched && is_touched) {
    delta_ges.x_val = x_new - posi_gest.x_val;
    delta_ges.y_val = y_new - posi_gest.y_val;

    if (delta_ges.x_val || delta_ges.y_val) {
      posi_gest.x_val = x_new;
      posi_gest.y_val = y_new;
    }
  }

  if (delta_ges.x_val == 0 && delta_ges.y_val == 0 && delta_ges.gesture == posi_gest.gesture) {
    return;
  }

  posi_gest.gesture = delta_ges.gesture;

  ret = k_msgq_put(&hids_queue, &delta_ges, K_NO_WAIT);
  if (ret) return;

  if (k_msgq_num_used_get(&hids_queue) == 1) {
    k_work_submit(&hids_work);
  }
}

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

static void pointer_movement_send(int16_t x_delta, int16_t y_delta, gesture_t gesture) {
  if (!connection) return;

  x_delta = MAX(MIN(x_delta, 511), -511);
  y_delta = MAX(MIN(y_delta, 511), -511);

  uint8_t buttons = 0;

  switch (gesture) {
    case GESTURE_CLICK:
      buttons |= 0x01;
      k_msgq_put(&hids_queue, &delta_ges_reset, K_NO_WAIT);
      break;
    case GESTURE_CLICK_HOLD:
      buttons |= 0x02;
      break;
    case GESTURE_DOUBLE_CLICK:
      buttons |= 0x01;
      k_msgq_put(&hids_queue, &delta_ges_reset, K_NO_WAIT);
      k_msgq_put(&hids_queue, &delta_ges_click, K_NO_WAIT);
      k_msgq_put(&hids_queue, &delta_ges_reset, K_NO_WAIT);
      break;
    default:
      break;
  }

  if (in_boot_mode) {
    bt_hids_boot_mouse_inp_rep_send(&hids_obj, connection, &buttons, (int8_t)x_delta, (int8_t)y_delta, NULL);
  } else {
    uint8_t buffer[3];

    uint16_t ux = (uint16_t)(x_delta & 0x03FF);
    uint16_t uy = (uint16_t)(y_delta & 0x03FF);

    buffer[0] = (uint8_t)(ux & 0xFF);
    buffer[1] = (uint8_t)(((ux >> 8) & 0x03) | ((uy & 0x3F) << 2));
    buffer[2] = (uint8_t)(((uy >> 6) & 0x0F) | (buttons << 4));

    bt_hids_inp_rep_send(&hids_obj, connection, 0, buffer, sizeof(buffer), NULL);
  }
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
  k_msleep(500);
  gpio_pin_set_dt(&led, 0);
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
  gpio_pin_set_dt(&led, 1);
  k_msleep(250);
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

static void int_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
  k_work_submit(&trackpad_work);
}

static struct bt_conn_cb connection_callbacks = {
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
  ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
  if (ret < 0) return 0;

  if (!gpio_is_ready_dt(&int_pin)) return 0;
  ret = gpio_pin_configure_dt(&int_pin, GPIO_INPUT);
  if (ret < 0) return 0;
  ret = gpio_pin_interrupt_configure_dt(&int_pin, GPIO_INT_EDGE_FALLING);
  if (ret < 0) return 0;

  ret = bt_conn_cb_register(&connection_callbacks);
  if (ret < 0) return 0;
  ret = bt_conn_auth_cb_register(NULL);
  if (ret < 0) return 0;
  ret = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
  if (ret < 0) return 0;

  gpio_init_callback(&int_cb_data, int_callback, BIT(int_pin.pin));
  gpio_add_callback(int_pin.port, &int_cb_data);

  hid_init();

  ret = bt_enable(NULL);
  if (ret < 0) return 0;

  settings_load();
  advertising_start();

  if (!device_is_ready(trackpad.bus)) return 0;

  ret = init_mtch6102(&trackpad);
  if (ret < 0) return 0;

  return 0;
}
