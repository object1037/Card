#ifndef MTCH6102_H__
#define MTCH6102_H__

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/drivers/i2c.h>

#define CMD_ADDR 0x04

#define TOUCH_STATE_ADDR 0x10
#define TOUCH_X_ADDR 0x11
#define TOUCH_Y_ADDR 0x12
#define TOUCH_LSB_ADDR 0x13
#define GESTURE_STATE_ADDR 0x14

#define ACTIVE_PERIOD_L_ADDR 0x25
#define ACTIVE_PERIOD_H_ADDR 0x26
#define IDLE_PERIOD_L_ADDR 0x27
#define IDLE_PERIOD_H_ADDR 0x28

#define CMD_CFG 0x20

#define STATE_TOUCH_BM 0x01
#define STATE_GESTURE_BM 0x02

#define TOUCH_X_MAX 576
#define TOUCH_Y_MAX 384

typedef enum gesture_t {
  GESTURE_NONE = 0x00,
  GESTURE_CLICK = 0x10,
  GESTURE_CLICK_HOLD = 0x11,
  GESTURE_DOUBLE_CLICK = 0x20,
  GESTURE_SWIPE_DOWN = 0x31,
  GESTURE_SWIPE_DOWN_HOLD = 0x32,
  GESTURE_SWIPE_RIGHT = 0x41,
  GESTURE_SWIPE_RIGHT_HOLD = 0x42,
  GESTURE_SWIPE_UP = 0x51,
  GESTURE_SWIPE_UP_HOLD = 0x52,
  GESTURE_SWIPE_LEFT = 0x61,
  GESTURE_SWIPE_LEFT_HOLD = 0x62,
} gesture_t;

int init_mtch6102(const struct i2c_dt_spec *trackpad);
int get_touch_data(const struct i2c_dt_spec *trackpad, uint16_t *touch_x, uint16_t *touch_y, bool *is_touched, gesture_t *gesture);

#endif // MTCH6102_H__
