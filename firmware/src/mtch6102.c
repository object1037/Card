#include "mtch6102.h"

static int apply_config(const struct i2c_dt_spec *trackpad) {
  int ret;

  uint8_t cmd = CMD_CFG;
  ret = i2c_reg_write_byte_dt(trackpad, CMD_ADDR, cmd);
  if (ret < 0) return -1;

  return 0;
}

int init_mtch6102(const struct i2c_dt_spec *trackpad) {
  int ret;

  uint8_t idle_period_h = 0x0c;
  uint8_t idle_period_l = 0x99;  // 100 ms

  ret = i2c_reg_write_byte_dt(trackpad, IDLE_PERIOD_L_ADDR, idle_period_l);
  if (ret < 0) return -1;
  ret = i2c_reg_write_byte_dt(trackpad, IDLE_PERIOD_H_ADDR, idle_period_h);
  if (ret < 0) return -1;

  ret = apply_config(trackpad);

  return ret;
}

int get_touch_data(
  const struct i2c_dt_spec *trackpad,
  uint16_t *touch_x,
  uint16_t *touch_y,
  bool *is_touched,
  gesture_t *gesture
) {
  int ret;
  uint8_t touch_data[5] = {0};

  ret = i2c_burst_read_dt(trackpad, TOUCH_STATE_ADDR, touch_data, sizeof(touch_data));
  if (ret < 0) return -1;

  *is_touched = (touch_data[0] & STATE_TOUCH_BM) != 0;

  *touch_x = ((touch_data[1] << 4) | (touch_data[3] >> 4));
  *touch_y = (touch_data[2] << 4) | (touch_data[3] & 0x0F);

  *gesture = (gesture_t)touch_data[4];

  return 0;
}
