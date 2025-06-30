#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app, LOG_LEVEL_DBG);

#define SLEEP_TIME_MS 500

#define I2C0_NODE DT_ALIAS(trackpad0)

#define TOUCH_STATE 0x10
#define TOUCHX 0x11
#define TOUCHY 0x12
#define GESTURE_STATE 0x14

static const struct i2c_dt_spec trackpad = I2C_DT_SPEC_GET(I2C0_NODE);

int main(void) {
	int ret;

  if (!device_is_ready(trackpad.bus)) {
    return 0;
  }
  LOG_INF("Starting application...");

  while (1) {
    uint8_t touch_state = 0;
    uint8_t gesture_state = 0;
    uint8_t touch_x = 0;
    uint8_t touch_y = 0;
    uint8_t addr[] = {TOUCH_STATE, TOUCHX, TOUCHY, GESTURE_STATE};
    ret = i2c_write_read_dt(&trackpad, &addr[0], 1, &touch_state, 1);
    ret = i2c_write_read_dt(&trackpad, &addr[1], 1, &touch_x, 1);
    ret = i2c_write_read_dt(&trackpad, &addr[2], 1, &touch_y, 1);

    if (ret < 0) {
      LOG_ERR("I2C communication failed: %d", ret);
      return 0;
    }

    if (touch_state & 0x04) {
      LOG_WRN("Lrg detected");
    }
    if (touch_state & 0x02) {
      ret = i2c_write_read_dt(&trackpad, &addr[3], 1, &gesture_state, 1);
      if (ret < 0) {
        return 0;
      }
      switch (gesture_state) {
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
          LOG_INF("Gesture: Unknown gesture: 0x%02X", gesture_state);
          break;
      }
    }

    LOG_INF("X,Y: (%d, %d)", (touch_x << 4), (touch_y << 4));
    k_msleep(SLEEP_TIME_MS);
  }
  return 0;
}
