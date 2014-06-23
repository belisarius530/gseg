#define MOSI 3
#define SS 2
#define SCK 5

#define L3GD20_ADDRESS                (0x6B)        // 1101001
#define L3GD20_POLL_TIMEOUT           (100)         // Maximum number of read attempts
#define L3GD20_ID                     (0b11010100)

#define L3GD20_SENSITIVITY_250DPS  (0.00875F)      // Roughly 22/256 for fixed point match // this was originally .00875, I embedded a *.02 term here.
#define L3GD20_SENSITIVITY_500DPS  (0.0175F)       // Roughly 45/256
#define L3GD20_SENSITIVITY_2000DPS (0.070F)        // Roughly 18/256
#define L3GD20_DPS_TO_RADS         (0.017453293F)  // degress/s to rad/s multiplier

#define L3GD20_REGISTER_WHO_AM_I             0x0F   // 11010100   r
#define L3GD20_REGISTER_CTRL_REG1            0x20   // 00000111   rw
#define L3GD20_REGISTER_CTRL_REG2            0x21   // 00000000   rw
#define L3GD20_REGISTER_CTRL_REG3            0x22   // 00000000   rw
#define L3GD20_REGISTER_CTRL_REG4            0x23   // 00000000   rw
#define L3GD20_REGISTER_CTRL_REG5            0x24   // 00000000   rw
#define L3GD20_REGISTER_REFERENCE            0x25   // 00000000   rw
#define L3GD20_REGISTER_OUT_TEMP             0x26   //            r
#define L3GD20_REGISTER_STATUS_REG           0x27   //            r
#define L3GD20_REGISTER_OUT_X_L              0x28   //            r
#define L3GD20_REGISTER_OUT_X_H              0x29   //            r
#define L3GD20_REGISTER_OUT_Y_L              0x2A   //            r
#define L3GD20_REGISTER_OUT_Y_H              0x2B   //            r
#define L3GD20_REGISTER_OUT_Z_L              0x2C   //            r
#define L3GD20_REGISTER_OUT_Z_H              0x2D   //            r
#define L3GD20_REGISTER_FIFO_CTRL_REG        0x2E   // 00000000   rw
#define L3GD20_REGISTER_FIFO_SRC_REG         0x2F   //            r
#define L3GD20_REGISTER_INT1_CFG             0x30   // 00000000   rw
#define L3GD20_REGISTER_INT1_SRC             0x31   //            r
#define L3GD20_REGISTER_TSH_XH               0x32   // 00000000   rw
#define L3GD20_REGISTER_TSH_XL               0x33   // 00000000   rw
#define L3GD20_REGISTER_TSH_YH               0x34   // 00000000   rw
#define L3GD20_REGISTER_TSH_YL               0x35   // 00000000   rw
#define L3GD20_REGISTER_TSH_ZH               0x36   // 00000000   rw
#define L3GD20_REGISTER_TSH_ZL               0x37   // 00000000   rw
#define L3GD20_REGISTER_INT1_DURATION        0x38   // 00000000   rw
#define cs 4
#define mosi 6
#define miso 5
#define clk 7

void spi_init(void);

void spi_write_sseg(uint8_t data);

void spi_write_dac(int data);

uint8_t spi_read_gyro(uint8_t address);

uint8_t spi_xfer(uint8_t x);
uint16_t spi_read16(uint8_t address);
void spi_write8(uint8_t address, uint8_t value);
uint8_t spi_read8(uint8_t address);
void spi_init_bitbang(void);
