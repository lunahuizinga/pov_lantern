#include <stdio.h>
#include <stdlib.h>
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

#define LED_FRAME_LENGTH 4

// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0
#define I2C_SDA 20
#define I2C_SCL 21
#define PCA9685_I2C_ADDR _u(0x60) // Default I2C address for the PCA9685 on the Adafruit DC motor FeatherWing as A5 is shorted high
// https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf
#define SHT30_I2C_ADDR _u(0x44) // Default I2C address for the SHT30
// https://sensirion.com/media/documents/213E6A3B/63A5A569/Datasheet_SHT3x_DIS.pdf
#define MPR121_I2C_ADDR _u(0x5A)

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

uint8_t dotstar_pixel_data[DOTSTAR_HEIGHT * DOTSTAR_WIDTH * 4];

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

void write_to_mpr121(uint8_t *src, size_t len, bool nostop){
    write_to_i2c(MPR121_I2C_ADDR, src, len, nostop);
}

void set_mpr121_reg(uint8_t reg, uint8_t data){
    uint8_t buffer[2];
    buffer[0] = reg;
    buffer[1] = data;
    write_to_mpr121(buffer, count_of(buffer), false);
}

int read_from_i2c(const uint8_t addr, uint8_t * dst, const size_t len, bool nostop){
    return i2c_read_blocking(I2C_PORT, addr, dst, len, nostop);
}

int read_from_pca9685(uint8_t * dst, const size_t len, bool nostop){
    return read_from_i2c(PCA9685_I2C_ADDR, dst, len, false);
}

int read_from_mpr121(uint8_t * dst, const size_t len){
    return read_from_i2c(MPR121_I2C_ADDR, dst, len, false);
}

uint8_t read_pca9685_reg(uint8_t reg){
    uint8_t buffer;
    write_to_pca9685(&reg, 1, true);
    read_from_pca9685(&buffer, 1, false);
    return buffer;
}

uint8_t read_mpr121_reg(uint8_t reg){
    uint8_t buffer;
    write_to_mpr121(&reg, 1, true);
    read_from_mpr121(&buffer, 1);
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

uint32_t format_dotstar_pixel_data(uint8_t brightness, uint8_t red, uint8_t green, uint8_t blue){
    return (((((uint32_t) brightness) & 0x1Fu) | 0xE0u)) | (((uint32_t) blue) << 8) | (((uint32_t) green) << 16) | ((uint32_t) red << 24);
}

void set_dotstar_pixel(uint8_t pixel_index, uint32_t pixel_data){
    for (size_t i = 0; i < LED_FRAME_LENGTH; i++){
        uint8_t pixel_blip = (uint8_t) (pixel_data >> (i * 8));
        dotstar_pixel_data[pixel_index * LED_FRAME_LENGTH + i] = pixel_blip;
    }
}

void push_dotstar_pixels(){
    uint8_t pixel_count = DOTSTAR_HEIGHT * DOTSTAR_WIDTH;
    uint16_t pixel_data_length = (pixel_count + 2) * LED_FRAME_LENGTH;
    uint8_t pixel_data[pixel_data_length];

    for (size_t i = 0; i < pixel_data_length; i++){
        if(i < LED_FRAME_LENGTH || i >= (pixel_count + 1) * LED_FRAME_LENGTH){
            pixel_data[i] = 0x0u;
            continue;
        }
        pixel_data[i] = dotstar_pixel_data[i - LED_FRAME_LENGTH];
    }
    
    write_to_spi(pixel_data, pixel_data_length);
}

void set_all_dotstar_pixels(uint8_t brightness, uint8_t red, uint8_t green, uint8_t blue){
    uint8_t pixel_count = DOTSTAR_HEIGHT * DOTSTAR_WIDTH;
    uint32_t pixel_data = format_dotstar_pixel_data(brightness, red, green, blue);

    printf("FORMATTED DATA: %02x\n", pixel_data);

    for (int i = 0; i < pixel_count; ++i){
        set_dotstar_pixel(i, pixel_data);
    }

    push_dotstar_pixels();
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

static void hsv2rgb(uint8_t h, uint8_t s, uint8_t v, uint8_t *r, uint8_t *g, uint8_t *b){
    uint8_t region, remainder;
    uint16_t p;
    uint16_t q;
    uint16_t t;

    if (s == 0) {                 /* achromatic (grey) */
        *r = *g = *b = v;
        return;
    }

    /* Hue is divided into six 60° sectors (0‑5).  Multiply by 6
       because we work in the 0‑255 range: 256 / 6 ≈ 42.666. */
    region   = h / 43;            /* 0‑5 */
    remainder = (h - (region * 43)) * 6;   /* 0‑255 */

    /* Compute intermediate values.
       All calculations stay in the 0‑255 range. */
    p = (v * (255 - s)) >> 8;                 /* = v * (1‑s)   */
    q = (v * (255 - ((s * remainder) >> 8))) >> 8;
    t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

    switch (region) {
        case 0: *r = v; *g = t; *b = p; break;   /* Red → Yellow */
        case 1: *r = q; *g = v; *b = p; break;   /* Yellow → Green */
        case 2: *r = p; *g = v; *b = t; break;   /* Green → Cyan   */
        case 3: *r = p; *g = q; *b = v; break;   /* Cyan → Blue    */
        case 4: *r = t; *g = p; *b = v; break;   /* Blue → Magenta*/
        default:*r = v; *g = p; *b = q; break;   /* Magenta → Red */
    }
}

static uint32_t format_dotstar_pixel_data_hsv(uint8_t hue, uint8_t saturation, uint8_t value){
    uint8_t r;
    uint8_t g;
    uint8_t b;
    hsv2rgb(hue, saturation, value, &r, &g, &b);
    return format_dotstar_pixel_data(0x1u, r, g, b);
}

static float lerp(float a, float b, float t){
    return a + (b - a) * t;
}

int main(){
    stdio_init_all();

    // Initialise the Wi-Fi chip
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
        return -1;
    }

    // SPI initialisation. This example will use SPI at 4MHz.
    spi_init(SPI_PORT, 4 * 1000 * 1000);
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

    // Initialise pin 22 for the PIR sensor
    gpio_init(22);
    gpio_pull_down(22);
    gpio_set_dir(22, GPIO_IN);

    // Example to turn on the Pico W LED
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    setup_pca9685();

    bool clockwise = true;

    // Our main PWM
    set_pca9685_pwm(TB6612FNG_PWM, 0, 0);
    
    // IN1 IN2  EFFECT
    // L   H    CCW
    // H   L    CW
    set_pca9685_pin(TB6612FNG_IN1, clockwise);
    set_pca9685_pin(TB6612FNG_IN2, !clockwise);

    // Set the touch sensitivity
    set_mpr121_reg(0x41, 0x10);
    set_mpr121_reg(0x42, 0x8);

    // Set MPR121 to start mode
    set_mpr121_reg(0x5E, 0x1);

    // Constants
    const uint8_t delta_time = 10;
    const uint16_t motor_pwm_on_amount = 2047;
    const int on_time = 5000;
    const int lerp_time = 2000;

    // Variables
    int counter = 0;
    uint8_t hue = 0;
    uint8_t saturation = 254;

    bool motor_status = false;
    int32_t motor_pwm_amount = 0;
    const int motor_pwm_change_speed = 30;
    bool last_cap_status = false;

    bool status = false;

    // Main loop
    while (true) {
        // Get sensor data
        bool pir_status = gpio_get(22);
        bool cap_status = read_mpr121_reg(0x0) & 0b1 == 0b1;

        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, pir_status);

        if (pir_status){
            counter = on_time;
            status = true;

            motor_status = true;
        }

        if (cap_status && !last_cap_status){
            saturation = (saturation + 16) % 255;
            last_cap_status = true;
        }

        if (!cap_status && last_cap_status) last_cap_status = false;

        if (counter <= 0) {
            status = false;

            motor_status = false;
        } else counter -= delta_time;

        if ((motor_status && motor_pwm_amount < motor_pwm_on_amount) || (!motor_status && motor_pwm_amount > 0)){
            if (motor_status) motor_pwm_amount += motor_pwm_change_speed;
            else motor_pwm_amount -= motor_pwm_change_speed;

            // Clamp
            if (motor_pwm_amount > motor_pwm_on_amount) motor_pwm_amount = motor_pwm_on_amount;
            else if (motor_pwm_amount < 0) motor_pwm_amount = 0;

            set_pca9685_pwm(TB6612FNG_PWM, 0, motor_pwm_amount);
        }

        hue = (hue + 1) % 255;

        for (size_t i = 0; i < DOTSTAR_HEIGHT * DOTSTAR_WIDTH; i++){
            uint32_t pixel_colour = format_dotstar_pixel_data_hsv((hue + i * 30) % 255, saturation, status ? 255 : 1);
            set_dotstar_pixel(i, pixel_colour);
        }
        
        push_dotstar_pixels();

        printf("%02x\n", (uint16_t) .5f * 240);

        sleep_ms(delta_time);
    }
}
