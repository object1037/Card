#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/conn.h>
#include <bluetooth/services/hids.h>

#define SLEEP_TIME_MS 20

#define INT0_NODE DT_ALIAS(int0)
#define I2C0_NODE DT_ALIAS(trackpad0)

#define TOUCH_STATE_ADDR 0x10

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

struct pointer_pos {
  int16_t x_val;
  int16_t y_val;
} position = {-1, -1};

static struct conn_mode {
  struct bt_conn *conn;
  bool in_boot_mode;
} conn_mode[CONFIG_BT_HIDS_MAX_CLIENT_COUNT];

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
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static void hids_pm_evt_handler(enum bt_hids_pm_evt evt, struct bt_conn *conn) {
  size_t i;

  for (i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
    if (conn_mode[i].conn == conn) {
      break;
    }
  }

  if (i >= CONFIG_BT_HIDS_MAX_CLIENT_COUNT) {
    return;
  }

  switch (evt) {
    case BT_HIDS_PM_EVT_BOOT_MODE_ENTERED:
      conn_mode[i].in_boot_mode = true;
      break;

    case BT_HIDS_PM_EVT_REPORT_MODE_ENTERED:
      conn_mode[i].in_boot_mode = false;
      break;

    default:
      break;
  }
}

static void hid_init(void) {
  int ret;
  struct bt_hids_init_param hids_init_param = {0};
  struct bt_hids_inp_rep *hids_inp_rep;
  static const uint8_t mouse_movement_mask[DIV_ROUND_UP(3, 8)] = {0};

  static const uint8_t report_map[] = {
      0x05, 0x01, /* Usage Page (Generic Desktop) */
      0x09, 0x02, /* Usage (Mouse) */

      0xA1, 0x01, /* Collection (Application) */

      0x85, 0x01, /* Report ID (1) */
      0x09, 0x01, /* Usage (Pointer) */
      0xA1, 0x00, /* Collection (Physical) */
      0x05, 0x09, /* Usage Page (Buttons) */
      0x19, 0x01, /* Usage Minimum (01) */
      0x29, 0x03, /* Usage Maximum (03) */
      0x15, 0x00, /* Logical Minimum (0) */
      0x25, 0x01, /* Logical Maximum (1) */
      0x75, 0x01, /* Report Size (1) */
      0x95, 0x03, /* Report Count (3) */
      0x81, 0x02, /* Input (Data, Variable, Absolute) */
      0x75, 0x05, /* Report Size (5) */
      0x95, 0x01, /* Report Count (1) */
      0x81, 0x01, /* Input (Constant) for padding */

      0x05, 0x01, /* Usage Page (Generic Desktop) */
      0x09, 0x30, /* Usage (X) */
      0x09, 0x31, /* Usage (Y) */
      0x15, 0x81, /* Logical minimum (-2047) */
      0x25, 0x7F, /* Logical maximum (2047) */
      0x75, 0x08, /* Report Size (8) */
      0x95, 0x02, /* Report Count (2) */
      0x81, 0x06, /* Input (Data, Variable, Relative) */
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
  hids_inp_rep->id = 1;
  hids_inp_rep->rep_mask = mouse_movement_mask;
  hids_init_param.inp_rep_group_init.cnt++;

  hids_init_param.is_mouse = true;
  hids_init_param.pm_evt_handler = hids_pm_evt_handler;

  ret = bt_hids_init(&hids_obj, &hids_init_param);
}

static void pointer_movement_send(int16_t x_delta, int16_t y_delta) {
  for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
    if (!conn_mode[i].conn) {
      continue;
    }

    x_delta = MAX(MIN(x_delta, SCHAR_MAX), SCHAR_MIN);
    y_delta = MAX(MIN(y_delta, SCHAR_MAX), SCHAR_MIN);

    if (conn_mode[i].in_boot_mode) {
      bt_hids_boot_mouse_inp_rep_send(&hids_obj, conn_mode[i].conn, NULL, (int8_t)x_delta, (int8_t)y_delta, NULL);
    } else {
      uint8_t buffer[3];

      buffer[0] = 0;  // Buttons, no buttons pressed
      buffer[1] = (uint8_t)x_delta;
      buffer[2] = (uint8_t)y_delta;

      int ret = bt_hids_inp_rep_send(&hids_obj, conn_mode[i].conn, 0, buffer, sizeof(buffer), NULL);
      if (ret < 0) {
        gpio_pin_toggle_dt(&led);
      }
    }
  }
}

static void mouse_handler(struct k_work *work) {
  struct pointer_pos pos;

  while (!k_msgq_get(&hids_queue, &pos, K_NO_WAIT)) {
    pointer_movement_send(pos.x_val, pos.y_val);
  }
}

static void insert_conn_object(struct bt_conn *conn) {
  for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
    if (!conn_mode[i].conn) {
      conn_mode[i].conn = conn;
      conn_mode[i].in_boot_mode = false;
      return;
    }
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

  insert_conn_object(conn);

  gpio_pin_set_dt(&led, 1);
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason) {
  int err = bt_hids_disconnected(&hids_obj, conn);

  for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
    if (conn_mode[i].conn == conn) {
      conn_mode[i].conn = NULL;
      break;
    }
  }

  gpio_pin_set_dt(&led, 0);
  advertising_start();
}

void on_recycled(void) {
  advertising_start();
}

static void security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err) {
  for (size_t i = 0; i < level; i++) {
    gpio_pin_set_dt(&led, 0);
    k_msleep(200);
    gpio_pin_set_dt(&led, 1);
    k_msleep(200);
  }
}

static void pairing_complete(struct bt_conn *conn, bool bonded) {
  return;
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason) {
  for (size_t i = 0; i < 5; i++) {
    gpio_pin_set_dt(&led, 0);
    k_msleep(100);
    gpio_pin_set_dt(&led, 1);
    k_msleep(50);
  }
  return;
}

static void trackpad_get() {
  int ret;
  uint8_t touch_data[5] = {0};

  ret = i2c_burst_read_dt(&trackpad, TOUCH_STATE_ADDR, touch_data, sizeof(touch_data));
  if (ret < 0) return;

  int x_new = (touch_data[1] << 4) | (touch_data[3] >> 4);
  int y_new = (touch_data[2] << 4) | (touch_data[3] & 0x0F);

  if (position.x_val == -1) {
    position.x_val = x_new;
    position.y_val = y_new;
  } else {
    int x_delta = x_new - position.x_val;
    int y_delta = y_new - position.y_val;

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
    .security_changed = security_changed,
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
