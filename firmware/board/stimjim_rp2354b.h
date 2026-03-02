pico_board_cmake_set(PICO_PLATFORM, rp2350)

#define PICO_RP2350A  0

#define NLDAC_A       43u
#define CSA_A         45u
#define CSB_A         41u
#define OE0_A         38u
#define OE1_A         37u
#define LED_A         03u
#define CHANNEL_IO_A  33u
#define SPI_SCK_A     46u
#define SPI_MOSI_A    47u
#define SPI_MISO_A    44u
#define SPI_A         spi1

#define NLDAC_B       24u
#define CSA_B         21u
#define CSB_B         17u
#define OE0_B         19u
#define OE1_B         18u
#define LED_B         02u
#define CHANNEL_IO_B  34u
#define SPI_SCK_B     22u
#define SPI_MOSI_B    23u
#define SPI_MISO_B    20u
#define SPI_B         spi0

#define GPIO0         10u
#define GPIO1         09u
#define GPIO2         07u
#define GPIO3         06u
#define GPIO4         05u
#define GPIO5         04u
#define GPIO6         12u
#define GPIO7         13u

#define RISING_EDGE_INTERRUPT_BIT(gpio_number, value) (value << (4 * ((gpio_number) % 8) + 3))