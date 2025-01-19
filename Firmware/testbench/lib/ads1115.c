#include <stdio.h>
#include "ads1115.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

// based on Adafruit_ADS1X15 library


#define I2C_MASTER_TIMEOUT_MS  1000


uint16_t MUX_BY_CHANNEL[] = {
    ADS1X15_REG_CONFIG_MUX_SINGLE_0, ///< Single-ended AIN0
    ADS1X15_REG_CONFIG_MUX_SINGLE_1, ///< Single-ended AIN1
    ADS1X15_REG_CONFIG_MUX_SINGLE_2, ///< Single-ended AIN2
    ADS1X15_REG_CONFIG_MUX_SINGLE_3  ///< Single-ended AIN3
};                                   ///< MUX config by channel


i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;

static void writeRegister(uint8_t reg, uint16_t value);
static uint16_t readRegister(uint8_t reg);

static void startADCReading(uint16_t mux, bool continuous);
static bool conversionComplete(void);
static int16_t getLastConversionResults(void);

static float computeVolts(int16_t counts);

uint8_t m_bitShift;            ///< bit shift amount
adsGain_t m_gain;              ///< ADC gain
uint16_t m_dataRate;           ///< Data rate
uint8_t buffer[3];

/*
ADS1015_Init() {
  m_bitShift = 4;
  m_gain = GAIN_TWOTHIRDS; // +/- 6.144V range (limited to VDD +0.3V max!) 
  m_dataRate = RATE_ADS1015_1600SPS;
}
*/

#define I2C_MASTER_SCL_IO           (21)       /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           (20)       /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          100000


static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ADS1X15_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
}



void ADS1115_Init(void) {

  m_bitShift = 0;
  m_gain = GAIN_TWOTHIRDS; /* +/- 6.144V range (limited to VDD +0.3V max!) */
  m_dataRate = RATE_ADS1115_128SPS;

  i2c_master_init(&bus_handle, &dev_handle);

}


/*
bool Adafruit_ADS1X15::begin(uint8_t i2c_addr, TwoWire *wire) {
  m_i2c_dev = new Adafruit_I2CDevice(i2c_addr, wire);
  return m_i2c_dev->begin();
}
*/

void setGain(adsGain_t gain) { m_gain = gain; }

adsGain_t getGain() { return m_gain; }

void setDataRate(uint16_t rate) { m_dataRate = rate; }

uint16_t getDataRate() { return m_dataRate; }

int16_t readADC_SingleEnded(uint8_t channel) {
  if (channel > 3) {
    return 0;
  }
  startADCReading(MUX_BY_CHANNEL[channel], /*continuous=*/false);
  while (!conversionComplete());
  return getLastConversionResults();
}


int16_t ADS1115_Read(uint8_t channel) {
  return readADC_SingleEnded(channel);
}

float ADS1115_ReadV(uint8_t channel) {
  return computeVolts(readADC_SingleEnded(channel));
}

bool conversionComplete() {
  return (readRegister(ADS1X15_REG_POINTER_CONFIG) & 0x8000) != 0;
}

int16_t readADC_Differential_0_1() {
  startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/false);
  while (!conversionComplete());
  return getLastConversionResults();
}

static int16_t readADC_Differential_0_3() {
  startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_3, /*continuous=*/false);
  while (!conversionComplete());
  return getLastConversionResults();
}

static int16_t readADC_Differential_1_3() {
  startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_1_3, /*continuous=*/false);
  while (!conversionComplete());
  return getLastConversionResults();
}

static int16_t readADC_Differential_2_3() {
  startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_2_3, /*continuous=*/false);
  while (!conversionComplete());
  return getLastConversionResults();
}

void startComparator_SingleEnded(uint8_t channel, int16_t threshold) {
  // Start with default values
  uint16_t config =
      ADS1X15_REG_CONFIG_CQUE_1CONV |   // Comparator enabled and asserts on 1
                                        // match
      ADS1X15_REG_CONFIG_CLAT_LATCH |   // Latching mode
      ADS1X15_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
      ADS1X15_REG_CONFIG_CMODE_TRAD |   // Traditional comparator (default val)
      ADS1X15_REG_CONFIG_MODE_CONTIN |  // Continuous conversion mode
      ADS1X15_REG_CONFIG_MODE_CONTIN;   // Continuous conversion mode

  // Set PGA/voltage range
  config |= m_gain;

  // Set data rate
  config |= m_dataRate;

  config |= MUX_BY_CHANNEL[channel];

  // Set the high threshold register
  // Shift 12-bit results left 4 bits for the ADS1015
  writeRegister(ADS1X15_REG_POINTER_HITHRESH, threshold << m_bitShift);

  // Write config register to the ADC
  writeRegister(ADS1X15_REG_POINTER_CONFIG, config);
}

static int16_t getLastConversionResults(void) {
  // Read the conversion results
  uint16_t res = readRegister(ADS1X15_REG_POINTER_CONVERT) >> m_bitShift;
  if (m_bitShift == 0) {
    return (int16_t)res;
  } else {
    // Shift 12-bit results right 4 bits for the ADS1015,
    // making sure we keep the sign bit intact
    if (res > 0x07FF) {
      // negative number - extend the sign to 16th bit
      res |= 0xF000;
    }
    return (int16_t)res;
  }
}

static float computeVolts(int16_t counts) {
  // see data sheet Table 3
  float fsRange;
  switch (m_gain) {
  case GAIN_TWOTHIRDS:
    fsRange = 6.144f;
    break;
  case GAIN_ONE:
    fsRange = 4.096f;
    break;
  case GAIN_TWO:
    fsRange = 2.048f;
    break;
  case GAIN_FOUR:
    fsRange = 1.024f;
    break;
  case GAIN_EIGHT:
    fsRange = 0.512f;
    break;
  case GAIN_SIXTEEN:
    fsRange = 0.256f;
    break;
  default:
    fsRange = 0.0f;
  }
  return counts * (fsRange / (32768 >> m_bitShift));
}

static void startADCReading(uint16_t mux, bool continuous) {
  // Start with default values
  uint16_t config =
      ADS1X15_REG_CONFIG_CQUE_1CONV |   // Set CQUE to any value other than
                                        // None so we can use it in RDY mode
      ADS1X15_REG_CONFIG_CLAT_NONLAT |  // Non-latching (default val)
      ADS1X15_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
      ADS1X15_REG_CONFIG_CMODE_TRAD;    // Traditional comparator (default val)

  if (continuous) {
    config |= ADS1X15_REG_CONFIG_MODE_CONTIN;
  } else {
    config |= ADS1X15_REG_CONFIG_MODE_SINGLE;
  }

  // Set PGA/voltage range
  config |= m_gain;

  // Set data rate
  config |= m_dataRate;

  // Set channels
  config |= mux;

  // Set 'start single-conversion' bit
  config |= ADS1X15_REG_CONFIG_OS_SINGLE;

  // Write config register to the ADC
  writeRegister(ADS1X15_REG_POINTER_CONFIG, config);

  // Set ALERT/RDY to RDY mode.
  writeRegister(ADS1X15_REG_POINTER_HITHRESH, 0x8000);
  writeRegister(ADS1X15_REG_POINTER_LOWTHRESH, 0x0000);
}



static void writeRegister(uint8_t reg, uint16_t value) {
  buffer[0] = reg;
  buffer[1] = value >> 8;
  buffer[2] = value & 0xFF;
  i2c_master_transmit(dev_handle, buffer, 3, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
  
}

static uint16_t readRegister(uint8_t reg) {
  buffer[0] = reg;
  i2c_master_transmit(dev_handle, &reg, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
  i2c_master_receive(dev_handle, buffer, 2, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
  return ((buffer[0] << 8) | buffer[1]);
}