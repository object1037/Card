#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app, LOG_LEVEL_DBG);

#define SLEEP_TIME_MS 500

#define I2C0_NODE DT_ALIAS(trackpad0)

#define TOUCH_STATE 0x10

static const struct i2c_dt_spec trackpad = I2C_DT_SPEC_GET(I2C0_NODE);

int main(void) {
	int ret;

  if (!device_is_ready(trackpad.bus)) {
    return 0;
  }
  LOG_INF("Starting application...");

  while (1) {
    uint8_t touch_data[5] = {0};

    ret = i2c_burst_read_dt(&trackpad, TOUCH_STATE, touch_data, sizeof(touch_data));

    if (ret < 0) {
      LOG_ERR("I2C communication failed: %d", ret);
      return 0;
    }

    if (touch_data[0] & 0x04) {
      LOG_WRN("Lrg detected");
    }
    if (touch_data[0] & 0x02) {
      if (ret < 0) {
        return 0;
      }
      switch (touch_data[4]) {
        case 0x00:
          LOG_INF("Gesture: None");
          break;
        case 0x10:
          LOG_INF("Gesture: Click");
          break;
        case 0x11:
          LOG_INF("Gesture: Click and hold");
          break;
        case 0x20:
          LOG_INF("Gesture: Double click");
          break;
        case 0x31:
          LOG_INF("Gesture: Swipe down");
          break;
        case 0x32:
          LOG_INF("Gesture: Swipe down hold");
          break;
        case 0x41:
          LOG_INF("Gesture: Swipe right");
          break;
        case 0x42:
          LOG_INF("Gesture: Swipe right hold");
          break;
        case 0x51:
          LOG_INF("Gesture: Swipe up");
          break;
        case 0x52:
          LOG_INF("Gesture: Swipe up hold");
          break;
        case 0x61:
          LOG_INF("Gesture: Swipe left");
          break;
        case 0x62:
          LOG_INF("Gesture: Swipe left hold");
          break;
        default:
          LOG_INF("Gesture: Unknown gesture: 0x%02X", touch_data[4]);
          break;
      }
    }

    LOG_INF("0x%02X (%d, %d)", touch_data[0], ((touch_data[1] << 4) | (touch_data[3] >> 4)), ((touch_data[2] << 4) | (touch_data[3] & 0x0F)));
    k_msleep(SLEEP_TIME_MS);
  }
  return 0;
}
