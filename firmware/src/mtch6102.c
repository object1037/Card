#include "mtch6102.h"

int get_touch_data(const struct i2c_dt_spec *trackpad, uint16_t *touch_x, uint16_t *touch_y, bool *is_touched, gesture_t *gesture) {
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
