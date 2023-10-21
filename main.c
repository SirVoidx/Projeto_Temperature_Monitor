#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

//Controle botão

int b_state = 0; 

//Macros de display
#define LCD_LIMPA_TELA          0x01
#define LCD_INICIA              0x02
#define LCD_ENTRYMODESET        0x04
#define LCD_DISPLAY_CONTROL     0x08
#define LCD_DISPLAY_FUNCTIONSET 0x20

#define LCD_INICIO_ESQUERDA 0x02
#define LCD_LIGA_DISPLAY    0x04

#define LCD_16X2 0x08

#define LCD_BACKLIGHT   0x08
#define LCD_ENABLE_BIT  0x04

#define LCD_CARACTERE    1
#define LCD_COMANDO     0

#define MAX_LINHAS  2
#define MAX_COLUNAS 16

#define DISPLAY_BUS_ADDR 0X27

#define DELAY_US 600

#define ADDR _u(0x76) //Endereço de memória do sensor de temperatura

//Registradores de hardware
#define REG_CONFIG _u(0xF5)
#define REG_CTRL_MEAS _u(0xF4)
#define REG_RESET _u(0xE0)
#define REG_TEMP_XLSB _u(0xFC)
#define REG_TEMP_LSB _u(0xFB)
#define REG_TEMP_MSB _u(0xFA)
#define REG_PRESSURE_XLSB _u(0xF9)
#define REG_PRESSURE_LSB _u(0xF8)
#define REG_PRESSURE_MSB _u(0xF7)

//Registradores de calibração
#define REG_DIG_T1_LSB _u(0x88)
#define REG_DIG_T1_MSB _u(0x89)
#define REG_DIG_T2_LSB _u(0x8A)
#define REG_DIG_T2_MSB _u(0x8B)
#define REG_DIG_T3_LSB _u(0x8C)
#define REG_DIG_T3_MSB _u(0x8D)
#define REG_DIG_P1_LSB _u(0x8E)
#define REG_DIG_P1_MSB _u(0x8F)
#define REG_DIG_P2_LSB _u(0x90)
#define REG_DIG_P2_MSB _u(0x91)
#define REG_DIG_P3_LSB _u(0x92)
#define REG_DIG_P3_MSB _u(0x93)
#define REG_DIG_P4_LSB _u(0x94)
#define REG_DIG_P4_MSB _u(0x95)
#define REG_DIG_P5_LSB _u(0x96)
#define REG_DIG_P5_MSB _u(0x97)
#define REG_DIG_P6_LSB _u(0x98)
#define REG_DIG_P6_MSB _u(0x99)
#define REG_DIG_P7_LSB _u(0x9A)
#define REG_DIG_P7_MSB _u(0x9B)
#define REG_DIG_P8_LSB _u(0x9C)
#define REG_DIG_P8_MSB _u(0x9D)
#define REG_DIG_P9_LSB _u(0x9E)
#define REG_DIG_P9_MSB _u(0x9F)

//Número de registradores de calibração
#define CAL_TIMES 24

struct bmp_cal_parameters {
    uint16_t dig_t1;
    int16_t dig_t2;
    int16_t dig_t3;
    uint16_t dig_p1;
    int16_t dig_p2;
    int16_t dig_p3;
    int16_t dig_p4;
    int16_t dig_p5;
    int16_t dig_p6;
    int16_t dig_p7;
    int16_t dig_p8;
    int16_t dig_p9;
};

#ifdef i2c_default
void bmp280_init(){
    uint8_t buf[2];
    const uint8_t reg_config_val = ((0x04 << 5) | (0x05 << 2)) & 0xFC;
    buf[0]= REG_CONFIG;
    buf[1]= reg_config_val;
    i2c_write_blocking(i2c_default, ADDR, buf, 2, false);
    const uint8_t reg_ctrl_meas_val= (0x01 << 5) | (0x03 << 2) | (0x03);
    buf[0]= REG_CTRL_MEAS;
    buf[1]= reg_ctrl_meas_val; 
    i2c_write_blocking(i2c_default, ADDR, buf, 2, false);
}

void bmp280_read_raw(int32_t* temp, int32_t* pressure) {
    uint8_t buf[6];
    uint8_t reg = REG_PRESSURE_MSB;
    i2c_write_blocking(i2c_default, ADDR, &reg, 1, true); 
    i2c_read_blocking(i2c_default, ADDR, buf, 6, false);
    *pressure = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
    *temp = (buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4);
}

void bmp280_reset() {
    uint8_t buf[2] = { REG_RESET, 0xB6 };
    i2c_write_blocking(i2c_default, ADDR, buf, 2, false);
}

int32_t bmp280_convert(int32_t temp, struct bmp_cal_parameters* params) {
    int32_t var1, var2;
    var1 = ((((temp >> 3) - ((int32_t)params->dig_t1 << 1))) * ((int32_t)params->dig_t2)) >> 11;
    var2 = (((((temp >> 4) - ((int32_t)params->dig_t1)) * ((temp >> 4) - ((int32_t)params->dig_t1))) >> 12) * ((int32_t)params->dig_t3)) >> 14;
    return var1 + var2;
}

int32_t bmp280_convert_temp(int32_t temp, struct bmp_cal_parameters* params) {
    int32_t t_fine = bmp280_convert(temp, params);
    return (t_fine * 5 + 128) >> 8;
}

int32_t bmp280_convert_pressure(int32_t pressure, int32_t temp, struct bmp_cal_parameters* params) {
    // uses the BMP280 calibration parameters to compensate the pressure value read from its registers

    int32_t t_fine = bmp280_convert(temp, params);
    int32_t var1, var2;
    uint32_t converted = 0.0;
    var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)params->dig_p6);
    var2 += ((var1 * ((int32_t)params->dig_p5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)params->dig_p4) << 16);
    var1 = (((params->dig_p3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)params->dig_p2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t)params->dig_p1)) >> 15);
    if (var1 == 0) {
        return 0;
    }
    converted = (((uint32_t)(((int32_t)1048576) - pressure) - (var2 >> 12))) * 3125;
    if (converted < 0x80000000) {
        converted = (converted << 1) / ((uint32_t)var1);
    } else {
        converted = (converted / (uint32_t)var1) * 2;
    }
    var1 = (((int32_t)params->dig_p9) * ((int32_t)(((converted >> 3) * (converted >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(converted >> 2)) * ((int32_t)params->dig_p8)) >> 13;
    converted = (uint32_t)((int32_t)converted + ((var1 + var2 + params->dig_p7) >> 4));
    return converted;
}

void bmp280_get_calib_params(struct bmp_cal_parameters* params) {

    uint8_t buf[CAL_TIMES] = { 0 };
    uint8_t reg = REG_DIG_T1_LSB;
    i2c_write_blocking(i2c_default, ADDR, &reg, 1, true);
    i2c_read_blocking(i2c_default, ADDR, buf, CAL_TIMES, false);

    params->dig_t1 = (uint16_t)(buf[1] << 8) | buf[0];
    params->dig_t2 = (int16_t)(buf[3] << 8) | buf[2];   
    params->dig_t3 = (int16_t)(buf[5] << 8) | buf[4];

    params->dig_p1 = (uint16_t)(buf[7] << 8) | buf[6];
    params->dig_p2 = (int16_t)(buf[9] << 8) | buf[8];
    params->dig_p3 = (int16_t)(buf[11] << 8) | buf[10];
    params->dig_p4 = (int16_t)(buf[13] << 8) | buf[12];
    params->dig_p5 = (int16_t)(buf[15] << 8) | buf[14];
    params->dig_p6 = (int16_t)(buf[17] << 8) | buf[16];
    params->dig_p7 = (int16_t)(buf[19] << 8) | buf[18];
    params->dig_p8 = (int16_t)(buf[21] << 8) | buf[20];
    params->dig_p9 = (int16_t)(buf[23] << 8) | buf[22];
}

#endif

void lcd_envia_comando(uint8_t val){
    i2c_write_blocking(i2c_default, DISPLAY_BUS_ADDR, &val, 1, false);
}

void lcd_pulse_enable(uint8_t val){
    sleep_us(DELAY_US);
    lcd_envia_comando(val | LCD_ENABLE_BIT);
    sleep_us(DELAY_US);
    lcd_envia_comando(val & ~LCD_ENABLE_BIT);
    sleep_us(DELAY_US);
}

void lcd_envia_byte(uint8_t caractere, int dado_comando){
    uint8_t nible_high = dado_comando | (caractere & 0xF0) | LCD_BACKLIGHT;
    uint8_t nible_low = dado_comando |((caractere << 4) & 0xF0) | LCD_BACKLIGHT;

    lcd_envia_comando(nible_high);
    lcd_pulse_enable(nible_high);
    lcd_envia_comando(nible_low);
    lcd_pulse_enable(nible_low); 
}

void lcd_limpa_tela(){
    lcd_envia_byte(LCD_LIMPA_TELA, LCD_COMANDO);
}

void lcd_posiciona_cursor(int linha, int coluna){
    int aux = (linha ==0) ? 0x80 + coluna : 0xC0 + coluna;
    lcd_envia_byte(aux, LCD_COMANDO);
}

void lcd_envia_caractere(char caractere){
    lcd_envia_byte(caractere, LCD_CARACTERE);
}

void lcd_envia_string(const char *s){
    while(*s){
        lcd_envia_caractere(*s++);
    } 
}

void lcd_init(){
    lcd_envia_byte(LCD_INICIA, LCD_COMANDO);
    lcd_envia_byte(LCD_INICIA | LCD_LIMPA_TELA, LCD_COMANDO);
    lcd_envia_byte(LCD_ENTRYMODESET | LCD_INICIO_ESQUERDA, LCD_COMANDO);
    lcd_envia_byte(LCD_DISPLAY_FUNCTIONSET | LCD_16X2, LCD_COMANDO);
    lcd_envia_byte(LCD_DISPLAY_CONTROL | LCD_LIGA_DISPLAY, LCD_COMANDO);
    lcd_limpa_tela();
}



void gpio_callback(uint gpio, uint32_t events){
    if(events & GPIO_IRQ_EDGE_FALL){
        if (b_state <1){
            b_state++;
            //printf("Botão em: %d", b_state);
        }
        else{
            b_state=0;
            //printf("Botão em: %d", b_state);
        }
    }
    
}

 int main() {
    stdio_init_all();
    cyw43_arch_init();
    sleep_ms(5000);
    
    char mensagem[16];
    char header[16];

    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    gpio_init(15);
    gpio_set_dir(15, GPIO_IN);
    gpio_set_pulls(15,true,false);
    gpio_set_irq_enabled_with_callback(15, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    bmp280_init();

    struct bmp_cal_parameters params;
    bmp280_get_calib_params(&params);

    int32_t raw_temperature;
    int32_t raw_pressure;

    sleep_ms(250);

    lcd_init();
    lcd_posiciona_cursor(0,0);

    while (true) {
        bmp280_read_raw(&raw_temperature, &raw_pressure);
        int32_t temperature = bmp280_convert_temp(raw_temperature, &params);
        int32_t pressure = bmp280_convert_pressure(raw_pressure, raw_temperature, &params);
        printf("Pressure = %.3f kPa\n", pressure / 1000.f);
        printf("Temp. = %.2f C\n", temperature / 100.f);
        sleep_ms(500);
        switch(b_state){
            case(0):{
                lcd_limpa_tela();
                sprintf(header, "Temperature");
                lcd_posiciona_cursor(0,0);
                lcd_envia_string(header);
                lcd_posiciona_cursor(1,0);
                sprintf(mensagem, "%.2f Celsius", temperature / 100.f);
                lcd_envia_string(mensagem);
                break;    
            }
            case(1):{
                lcd_limpa_tela();
                sprintf(header, "Pressure");
                lcd_posiciona_cursor(0,0);
                lcd_envia_string(header);
                lcd_posiciona_cursor(1,0);
                sprintf(mensagem, "%.3f atm", pressure / 101325.f);
                lcd_envia_string(mensagem);
                break;
            }
        }
        
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        sleep_ms(500);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        sleep_ms(500);
    }
}

