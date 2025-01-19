#include "ports.h"

#include "driver/gpio.h"


#define GPIO_KEY  9
#define GPIO_LED  8


void GPIO_Init() {
  gpio_config_t io_conf = {};

  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = (1<<GPIO_LED);
  io_conf.pull_down_en = 0;
  io_conf.pull_up_en = 0;
  gpio_config(&io_conf);

  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pin_bit_mask = (1<<GPIO_KEY);
  io_conf.pull_down_en = 0;
  io_conf.pull_up_en = 0;
  gpio_config(&io_conf);
}

void LED_Write(uint8_t state) {
  gpio_set_level(GPIO_LED, state^1);
}

uint8_t Key_Read(void) {
  return gpio_get_level(GPIO_KEY);
}
