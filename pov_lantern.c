#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "pico/cyw43_arch.h"

// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define SPI_PORT spi0
#define PIN_CIPO 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_COPI 19

#define DOTSTAR_HEIGHT 8
#define DOTSTAR_WIDTH 8

// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0
#define I2C_SDA 20
#define I2C_SCL 21
#define PCA9685_I2C_ADDR _u(0x60) // Default I2C address for the PCA9685 on the Adafruit DC motor FeatherWing as A5 is shorted high
// https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf

// PCA9685 registers
#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9
#define PCA9685_PRESCALE 0xFE
#define PCA9685_MODE1 0x0

// Mode 1 register setting bits
#define PCA9685_MODE_SLEEP 0x10
#define PCA9685_MODE_AUTO_INC 0x20
#define PCA9685_MODE_ALL_CALL 0x01

// TB6612FNG PWM pin mappings
#define TB6612FNG_IN1 0xa
#define TB6612FNG_IN2 0x9
#define TB6612FNG_PWM 0x8

// PCA9685 abstraction functions
void write_to_i2c(uint8_t addr, uint8_t *src, size_t len, bool nostop){
    i2c_write_blocking(I2C_PORT, addr, src, len, nostop);
}

void write_to_pca9685(uint8_t *src, size_t len, bool nostop){
    write_to_i2c(PCA9685_I2C_ADDR, src, len, nostop);
}

void set_pca9685_reg(uint8_t reg, uint8_t data){
    uint8_t buffer[2];
    buffer[0] = reg;
    buffer[1] = data;
    write_to_pca9685(buffer, count_of(buffer), false);
}

void set_pca9685_regs(uint8_t start_reg, uint8_t data[]){
    uint8_t buffer[count_of(data) + 1];
    buffer[0] = start_reg;
    for (size_t i = 1; i < count_of(buffer); i++){
        buffer[i] = data[i - 1];
    }
    write_to_pca9685(buffer, count_of(buffer), false);
}

int read_from_i2c(const uint8_t addr, uint8_t * dst, const size_t len, bool nostop){
    return i2c_read_blocking(I2C_PORT, addr, dst, len, nostop);
}

int read_from_pca9685(uint8_t * dst, const size_t len, bool nostop){
    return read_from_i2c(PCA9685_I2C_ADDR, dst, len, false);
}

uint8_t read_pca9685_reg(uint8_t reg){
    uint8_t buffer;
    write_to_pca9685(&reg, 1, true);
    read_from_pca9685(&buffer, 1, false);
    return buffer;
}

// Shamelessly copied from the Adafruit Arduino motor library
void set_pca9685_pwm(uint8_t pwm_num, uint16_t on, uint16_t off){
    uint8_t buffer[4];
    buffer[0] = on;
    buffer[1] = on >> 8;
    buffer[2] = off;
    buffer[3] = off >> 8;

    set_pca9685_regs(LED0_ON_L + pwm_num * 4, buffer);
}

void set_pca9685_pin(uint8_t pin_num, bool on_state){
    set_pca9685_pwm(pin_num, 0, on_state ? 4095 : 0);
}

void write_to_spi(const uint8_t *src, size_t len){
    spi_write_blocking(SPI_PORT, src, len);
}

uint32_t get_dotstar_pixel_data(uint8_t brightness, uint8_t red, uint8_t green, uint8_t blue){
    return ((((uint32_t) brightness) | 0xE0) << 24) | (((uint32_t) blue) << 16) | (((uint32_t) green) << 8) | ((uint32_t) red);
}

void set_all_dotstar_pixels(uint8_t brightness, uint8_t red, uint8_t green, uint8_t blue){
    uint8_t pixel_count = DOTSTAR_HEIGHT * DOTSTAR_WIDTH;
    uint8_t pixel_data_length = pixel_count * 4;
    uint8_t pixels_data[pixel_data_length];
    uint32_t pixel_data = get_dotstar_pixel_data(brightness, red, green, blue);
    for (size_t i = 0; i < pixel_data_length; i++){
        pixels_data[i] = pixel_data >> (i % 4) * 8;
    }
    write_to_spi(pixels_data, pixel_data_length);
}

void setup_pca9685(){
    // Get the current mode register
    uint8_t current_mode = read_pca9685_reg(PCA9685_MODE1);

    // WAKEY WAKEY
    uint8_t new_mode = current_mode & 0x6F;
    set_pca9685_reg(PCA9685_MODE1, new_mode);

    // Give it some time to wake up
    // "It takes 500 µs max for the oscillator to be up and
    // running once the SLEEP bit has been set to logic 0."
    sleep_ms(1);

    // Define settings
    uint8_t mode1_settings = PCA9685_MODE_ALL_CALL | PCA9685_MODE_AUTO_INC;
    // Apply our settings
    new_mode = new_mode | mode1_settings;
    // Write the register back
    set_pca9685_reg(PCA9685_MODE1, new_mode);
}

int main(){
    stdio_init_all();

    // Initialise the Wi-Fi chip
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
        return -1;
    }

    // SPI initialisation. This example will use SPI at 1MHz.
    spi_init(SPI_PORT, 1000 * 1000);
    gpio_set_function(PIN_CIPO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_COPI, GPIO_FUNC_SPI);
    spi_set_slave(SPI_PORT, false);
    
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
    // For more examples of SPI use see https://github.com/raspberrypi/pico-examples/tree/master/spi

    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400 * 1000);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    // For more examples of I2C use see https://github.com/raspberrypi/pico-examples/tree/master/i2c

    // Example to turn on the Pico W LED
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    setup_pca9685();

    bool clockwise = true;

    // Our main PWM
    set_pca9685_pwm(TB6612FNG_PWM, 0, 495);
    
    // IN1 IN2  EFFECT
    // L   H    CCW
    // H   L    CW
    set_pca9685_pin(TB6612FNG_IN1, clockwise);
    set_pca9685_pin(TB6612FNG_IN2, !clockwise);

    // set_all_dotstar_pixels(6, 0xFF, 0xFF, 0xFF);

    uint32_t pixel_data = get_dotstar_pixel_data(8 | 0xE0, 0xFF, 0xFF, 0xFF);
    uint8_t pixel_buffer[4];
    pixel_buffer[0] = pixel_data;
    pixel_buffer[1] = pixel_data >> 8;
    pixel_buffer[2] = pixel_data >> 16;
    pixel_buffer[3] = pixel_data >> 24;

    write_to_spi(pixel_buffer, 4);

    uint8_t index = 0;
    uint8_t register_offset = 0x26;

    bool light_status = false;

    while (true) {
        uint8_t register_index = register_offset + index;
        uint8_t register_value = read_pca9685_reg(register_index);
        printf("Register %02x: %02x\n", register_index, register_value);

        index++;
        index %= 4;

        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, light_status);
        light_status ^= true;

        sleep_ms(500);
    }
}
