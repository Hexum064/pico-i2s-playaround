#include <stdio.h>
#include <stdlib.h>
#include "string.h"
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pico/multicore.h"
#include "hardware/spi.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "i2s_32_441.pio.h"
#include "ws2812b.pio.h"

#define ALARM_1_NUM 0
#define ALARM_1_IRQ TIMER_IRQ_0

#define ALARM_2_NUM 1
#define ALARM_2_IRQ TIMER_IRQ_1

#define SAMPLE_RATE 44100
#define BITS_PER_SAMPLE 32
#define SPI_CLK_DIVISOR 90
#define SPI_FREQ (SAMPLE_RATE * BITS_PER_SAMPLE)
#define PICO_CLK_KHZ 133000 // ((SPI_FREQ * SPI_CLK_DIVISOR) / 1000)
#define AUDIO_BUFF_SIZE 1024 //Size of max Ocarina
#define MAX_VOL 254
#define VOL_AVG 8

#define ADC_VOL_GPIO 26

#define I2S_DATA_GPIO 17  
#define I2S_CLK_GPIO 18   
#define I2S_FS_GPIO 19   

#define PWM_LED_0_GPIO 2
#define PWM_LED_1_GPIO 3
#define PWM_LED_2_GPIO 4

#define NOTE_D_GPIO 5
#define NOTE_F_GPIO 6
#define NOTE_A_GPIO 7
#define NOTE_B_GPIO 8
#define NOTE_D2_GPIO 9

#define MODE_GPIO 10

#define MODE_LED_0_GPIO 11
#define MODE_LED_1_GPIO 12

#define RGB_LEDS_GPIO 13

#define PWM_CYCLES_LONG 10000

extern uint8_t song_of_storms[];
extern uint8_t a_loop[];
extern uint8_t b_loop[];
extern uint8_t d_loop[];
extern uint8_t d2_loop[];
extern uint8_t f_loop[];

const uint16_t sineLookupTable[] = {
0x8000, 0x8327, 0x864e, 0x8974, 0x8c98, 0x8fbb, 0x92db, 0x95f8,
0x9911, 0x9c27, 0x9f39, 0xa245, 0xa54c, 0xa84e, 0xab49, 0xae3d,
0xb12a, 0xb410, 0xb6ed, 0xb9c2, 0xbc8e, 0xbf51, 0xc20a, 0xc4b8,
0xc75c, 0xc9f4, 0xcc82, 0xcf03, 0xd178, 0xd3e0, 0xd63c, 0xd88a,
0xdaca, 0xdcfc, 0xdf1f, 0xe134, 0xe33a, 0xe530, 0xe717, 0xe8ed,
0xeab3, 0xec69, 0xee0e, 0xefa2, 0xf124, 0xf295, 0xf3f4, 0xf541,
0xf67b, 0xf7a4, 0xf8b9, 0xf9bc, 0xfaac, 0xfb89, 0xfc53, 0xfd09,
0xfdac, 0xfe3c, 0xfeb8, 0xff20, 0xff74, 0xffb5, 0xffe2, 0xfffa,
0xffff, 0xfff0, 0xffce, 0xff97, 0xff4c, 0xfeee, 0xfe7c, 0xfdf7,
0xfd5d, 0xfcb1, 0xfbf0, 0xfb1d, 0xfa37, 0xf93d, 0xf831, 0xf712,
0xf5e0, 0xf49c, 0xf346, 0xf1df, 0xf065, 0xeeda, 0xed3e, 0xeb90,
0xe9d2, 0xe804, 0xe625, 0xe437, 0xe239, 0xe02c, 0xde0f, 0xdbe5,
0xd9ab, 0xd764, 0xd510, 0xd2ae, 0xd03f, 0xcdc4, 0xcb3d, 0xc8aa,
0xc60b, 0xc362, 0xc0ae, 0xbdf1, 0xbb29, 0xb859, 0xb580, 0xb29e,
0xafb5, 0xacc4, 0xa9cc, 0xa6ce, 0xa3c9, 0xa0bf, 0x9db0, 0x9a9d,
0x9785, 0x946a, 0x914b, 0x8e2a, 0x8b06, 0x87e1, 0x84bb, 0x8194,
0x7e6c, 0x7b45, 0x781f, 0x74fa, 0x71d6, 0x6eb5, 0x6b96, 0x687b,
0x6563, 0x6250, 0x5f41, 0x5c37, 0x5932, 0x5634, 0x533c, 0x504b,
0x4d62, 0x4a80, 0x47a7, 0x44d7, 0x420f, 0x3f52, 0x3c9e, 0x39f5,
0x3756, 0x34c3, 0x323c, 0x2fc1, 0x2d52, 0x2af0, 0x289c, 0x2655,
0x241b, 0x21f1, 0x1fd4, 0x1dc7, 0x1bc9, 0x19db, 0x17fc, 0x162e,
0x1470, 0x12c2, 0x1126, 0xf9b, 0xe21, 0xcba, 0xb64, 0xa20,
0x8ee, 0x7cf, 0x6c3, 0x5c9, 0x4e3, 0x410, 0x34f, 0x2a3,
0x209, 0x184, 0x112, 0xb4, 0x69, 0x32, 0x10, 0x01,
0x06, 0x1e, 0x4b, 0x8c, 0xe0, 0x148, 0x1c4, 0x254,
0x2f7, 0x3ad, 0x477, 0x554, 0x644, 0x747, 0x85c, 0x985,
0xabf, 0xc0c, 0xd6b, 0xedc, 0x105e, 0x11f2, 0x1397, 0x154d,
0x1713, 0x18e9, 0x1ad0, 0x1cc6, 0x1ecc, 0x20e1, 0x2304, 0x2536,
0x2776, 0x29c4, 0x2c20, 0x2e88, 0x30fd, 0x337e, 0x360c, 0x38a4,
0x3b48, 0x3df6, 0x40af, 0x4372, 0x463e, 0x4913, 0x4bf0, 0x4ed6,
0x51c3, 0x54b7, 0x57b2, 0x5ab4, 0x5dbb, 0x60c7, 0x63d9, 0x66ef,
0x6a08, 0x6d25, 0x7045, 0x7368, 0x768c, 0x79b2, 0x7cd9};


typedef struct audio_buff_t {
    uint32_t buffer[AUDIO_BUFF_SIZE * 2];
    uint32_t length;
} audio_buff_t;

audio_buff_t audio_buffs[2];

//Maybe create a struct
uint8_t * current_sample;
uint32_t current_sample_len;
uint32_t current_sample_offset = 0;
uint8_t audio_dma_channel;
uint8_t current_volume = 0;
uint8_t current_buff_index = 0;
uint16_t vol_running_avg[VOL_AVG];
uint8_t vol_avg_index = 0;

dma_channel_config audio_dma_config;
dma_channel_config pwm_0_dma_config;
dma_channel_config pwm_1_dma_config;
dma_channel_config pwm_2_dma_config;

bool audio_button_down = false;

//TODO: Move logic to DMA
//TODO: Change paths in data.S to relative

//Loads the sample data, upconverts from 22.05K to 44.1K
//Only loads the left channel with data. Leave right at 0
//Returns true if a buffer was loaded
void load_audio_buffer(uint8_t * src, uint32_t src_len, uint32_t * src_offset, audio_buff_t * buff, uint8_t volume)
{
    buff->length =  (*src_offset + AUDIO_BUFF_SIZE) >= src_len ? (src_len - *src_offset) + 4 : AUDIO_BUFF_SIZE;
    uint32_t data;
    // printf("buffer addr: 0x%08x ", buff->buffer);
    // printf("volume: %d, len: 0x%04x\n", volume, buff->length);
    for (uint32_t i = 0; i < buff->length; i++)
    {
        int16_t s = (int16_t)(src[*src_offset + i]  - 0x7F);
        data = (uint32_t)(s * volume) << 16;
        // if (i == 0 || i == 1 || i == buff->length - 2 || i == buff->length - 1)
        // {
        // printf("%u -> %d -> 0x%08x\n", src[*src_offset + i], s, data);
        // }
        //This double load upconverts from 22.05k to 44.1K
        
        buff->buffer[i*2] = data;
        buff->buffer[(i*2)+1] = data;
    }

    (*src_offset) += buff->length;
    // printf("buff len: %u, new offset: %u\n", buff->length, *src_offset);
}

void audio_dma_start(int dma_channel, audio_buff_t * buff)
{
    // printf("starting transfer with buff at 0x%08x, len: %u\n", buff->buffer,  buff->length);
    dma_channel_configure(
        dma_channel,
        &audio_dma_config,
        &pio0_hw->txf[0], // Write address (only need to set this once)
        buff->buffer,             // Don't provide a read address yet
        buff->length * 2, // len is doubled because we upconverted
        true             // Don't start yet
    );   
}


void audio_dma_isr()
{

    // Clear the interrupt request.
    dma_hw->ints0 = 1u << audio_dma_channel;

    
    uint16_t len;
    //if the buffer to be output next has a len of 0, we are done
    if (audio_buffs[current_buff_index%2].length > 0)
    {
        //start the next output and load the next buffer
        audio_dma_start(audio_dma_channel, audio_buffs + (current_buff_index%2));
    }

// printf("playing sample for index %d\n", current_buff_index%2);
    uint64_t s_time;
    uint64_t e_time;
    s_time = time_us_64();
        current_buff_index++; //ok to wrap back to 0
        load_audio_buffer(current_sample, current_sample_len, &current_sample_offset, audio_buffs + (current_buff_index%2), current_volume); 
        
        if (audio_buffs[current_buff_index%2].length == 0 && audio_button_down)
        {
            //Auto restart
            // printf("Auto Restart\n");
            current_sample_offset = 4;            
            load_audio_buffer(current_sample, current_sample_len, &current_sample_offset, audio_buffs + (current_buff_index%2), current_volume); 
            //test to find in login analyzer data
            // for (uint x = 0; x< 8; x++)
            // {
            //     (audio_buffs + (current_buff_index%2))->buffer[x] = 0;
            // }
            // (audio_buffs + (current_buff_index%2))->length = 8;
        }
    e_time = time_us_64();
    // printf("time: %llu\n", e_time - s_time);






}

int audio_dma_init(audio_buff_t * first_buff)
{
    // Get a free channel, panic() if there are none
    int chan = dma_claim_unused_channel(true);
    audio_dma_config = dma_channel_get_default_config(chan);
    channel_config_set_transfer_data_size(&audio_dma_config, DMA_SIZE_32);
    channel_config_set_read_increment(&audio_dma_config, true);
    channel_config_set_write_increment(&audio_dma_config, false);
    channel_config_set_dreq(&audio_dma_config, DREQ_PIO0_TX0);
    
      
    
    return chan;
}

// int pwm_0_dma_init()
// {
//     uint slice0 = pwm_gpio_to_slice_num(PWM_LED_0_GPIO);
//     int chan = dma_claim_unused_channel(true);
//     pwm_0_dma_config = dma_channel_get_default_config(chan);
//     channel_config_set_transfer_data_size(&pwm_0_dma_config, DMA_SIZE_16);
//     channel_config_set_read_increment(&pwm_0_dma_config, true);
//     channel_config_set_write_increment(&pwm_0_dma_config, false);
//     channel_config_set_ring(&pwm_0_dma_config, false, 8);
//     channel_config_set_dreq(&pwm_0_dma_config, DREQ_DMA_TIMER0);
//     dma_channel_configure(chan, &pwm_0_dma_config, &pwm_hw->slice[slice0].cc, sineLookupTable, 256, false);
//     dma_timer_set_fraction(0, )
    
//     return chan;   
// }

void audio_irq_init(int channel)
{
    dma_channel_set_irq0_enabled(channel, true);
    irq_set_exclusive_handler(DMA_IRQ_0, audio_dma_isr);
    irq_set_enabled(DMA_IRQ_0, true);
}

void set_current_sample(uint8_t * sample)
{
    current_sample = sample;
    current_sample_len = (uint32_t)(current_sample[3] << 24) + (uint32_t)(current_sample[2] << 16) + (uint32_t)(current_sample[1] << 8) + (uint32_t)(current_sample[0] << 0);
    current_sample_offset = 4;
    printf("Curr Samp len: %lu\n", current_sample_len);
}

uint8_t get_vol_avg()
{
    uint16_t total = 0;
    for (uint8_t i = 0; i < VOL_AVG; i++)
    {        
        total += vol_running_avg[i];
    }

    return total / VOL_AVG;
}

void led_pwm_init()
{
    gpio_set_function(PWM_LED_0_GPIO, GPIO_FUNC_PWM);
    gpio_set_function(PWM_LED_1_GPIO, GPIO_FUNC_PWM);
    gpio_set_function(PWM_LED_2_GPIO, GPIO_FUNC_PWM);

    uint slice0 = pwm_gpio_to_slice_num(PWM_LED_0_GPIO);
    uint slice1 = pwm_gpio_to_slice_num(PWM_LED_1_GPIO);
    uint slice2 = pwm_gpio_to_slice_num(PWM_LED_2_GPIO);
    // uint slice1 = pwm_gpio_to_slice_num(PWM_LED_0_GPIO);
    // uint slice2 = pwm_gpio_to_slice_num(PWM_LED_0_GPIO);

    pwm_set_gpio_level(PWM_LED_0_GPIO, 0);
    pwm_set_gpio_level(PWM_LED_1_GPIO, 0);
    pwm_set_gpio_level(PWM_LED_2_GPIO, 0);
    pwm_set_enabled(slice0, true);
    pwm_set_enabled(slice1, true);
    pwm_set_enabled(slice2, true);

}

// static void alarm_1_isr(void) {
//     // Clear the alarm irq
//     hw_clear_bits(&timer_hw->intr, 1u << ALARM_1_NUM);

// }

// static void alarm_2_isr(void) {
//     // Clear the alarm irq
//     hw_clear_bits(&timer_hw->intr, 1u << ALARM_2_NUM);

// }


// static void alarm_1_in_us(uint32_t delay_us) {
//     // Enable the interrupt for our alarm (the timer outputs 4 alarm irqs)
//     hw_set_bits(&timer_hw->inte, 1u << ALARM_1_NUM);
//     // Set irq handler for alarm irq
//     irq_set_exclusive_handler(ALARM_1_IRQ, alarm_1_isr);
//     // Enable the alarm irq
//     irq_set_enabled(ALARM_1_IRQ, true);
//     // Enable interrupt in block and at processor


//     timer_hw->alarm[ALARM_1_NUM] = timer_hw->timerawl + delay_us;
// }

// static void alarm_2_in_us(uint32_t delay_us) {
//     // Enable the interrupt for our alarm (the timer outputs 4 alarm irqs)
//     hw_set_bits(&timer_hw->inte, 1u << ALARM_2_NUM);
//     // Set irq handler for alarm irq
//     irq_set_exclusive_handler(ALARM_2_IRQ, alarm_2_isr);
//     // Enable the alarm irq
//     irq_set_enabled(ALARM_2_IRQ, true);
//     // Enable interrupt in block and at processor


//     timer_hw->alarm[ALARM_2_NUM] = timer_hw->timerawl + delay_us;
// }
bool temp_led_on = true;
const uint LED_PIN = PICO_DEFAULT_LED_PIN;
repeating_timer_t timer0;
repeating_timer_t timer1;
repeating_timer_t pwm_0_timer;
repeating_timer_t pwm_1_timer;
repeating_timer_t pwm_2_timer;

alarm_pool_t *core_0_alarm;
PIO pio = pio0;
uint8_t blue = 0;
bool blink_cb(repeating_timer_t *rt)
{
    gpio_put(LED_PIN, temp_led_on);
    temp_led_on = !temp_led_on;
    return true;
}

bool rgb_leds_update_cb(repeating_timer_t *rt)
{
    
    for (uint8_t i = 0; i < 48; i++)
    {    

        pio_sm_put_blocking(pio, 1, 0x00003F00 + (blue % 128));
        blue++;        
    }
    return true;
}

void led_blink_test_init()
{
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    
    alarm_pool_add_repeating_timer_ms(core_0_alarm, 250, blink_cb, 0, &timer0);
}

void rgb_leds_update_timer_init()
{    
    alarm_pool_add_repeating_timer_ms(core_0_alarm, 100, rgb_leds_update_cb, 0, &timer1);
}

uint8_t pwm_0_index = 0;
uint8_t pwm_1_index = 0x55;
uint8_t pwm_2_index = 0xAA;

bool pwm_0_cb(repeating_timer_t *rt)
{    
    pwm_set_gpio_level(PWM_LED_0_GPIO, sineLookupTable[pwm_0_index++]);    
    return true;
}

void pwm_0_update_timer_init()
{    
    alarm_pool_add_repeating_timer_ms(core_0_alarm, 4, pwm_0_cb, 0, &pwm_0_timer);
}

bool pwm_1_cb(repeating_timer_t *rt)
{    
    pwm_set_gpio_level(PWM_LED_1_GPIO, sineLookupTable[pwm_1_index++]);    
    return true;
}

void pwm_1_update_timer_init()
{    
    alarm_pool_add_repeating_timer_ms(core_0_alarm, 4, pwm_1_cb, 0, &pwm_1_timer);
}

bool pwm_2_cb(repeating_timer_t *rt)
{    
    pwm_set_gpio_level(PWM_LED_2_GPIO, sineLookupTable[pwm_2_index++]);    
    return true;
}

void pwm_2_update_timer_init()
{    
    alarm_pool_add_repeating_timer_ms(core_0_alarm, 4, pwm_2_cb, 0, &pwm_2_timer);
}

void button_gpios_init()
{
    uint32_t mask = (1 << NOTE_D_GPIO)
        | (1 << NOTE_F_GPIO)
        | (1 << NOTE_A_GPIO)
        | (1 << NOTE_B_GPIO)
        | (1 << NOTE_D2_GPIO);

    gpio_set_dir_in_masked(mask);

    gpio_set_pulls(NOTE_D_GPIO, false, true);
    gpio_set_pulls(NOTE_F_GPIO, false, true);
    gpio_set_pulls(NOTE_A_GPIO, false, true);
    gpio_set_pulls(NOTE_B_GPIO, false, true);
    gpio_set_pulls(NOTE_D2_GPIO, false, true);


}

int main() {

    //set_sys_clock_khz(133000, true);   
    set_sys_clock_khz(PICO_CLK_KHZ, true);   

    

    stdio_init_all();

    adc_init();

    sleep_ms(4000);

    printf("Initializing\n");
    

    button_gpios_init();

    led_pwm_init();


    core_0_alarm = alarm_pool_create_with_unused_hardware_alarm(5);
    led_blink_test_init();

    adc_gpio_init(ADC_VOL_GPIO);

    adc_select_input(0);
    uint8_t vol;
    for (uint8_t i = 0; i < VOL_AVG; i++)
    {
        vol = adc_read() >> 4;
        vol = vol > MAX_VOL ? MAX_VOL : vol;
        vol_running_avg[i] = vol;
    }
    
    printf("volume: %d\n", current_volume);

    pwm_0_update_timer_init();
    pwm_1_update_timer_init();
    pwm_2_update_timer_init();

    // uint32_t colors[48];

    // for (uint8_t i = 0; i < 48; i++)
    // {
    //     colors[i] = 0x00003F1F + (i * 3);
    // }




    // const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    // gpio_init(LED_PIN);
    // gpio_set_dir(LED_PIN, GPIO_OUT);
    // while (true) {
    //     printf("blink\n");
    //     gpio_put(LED_PIN, 1);
    //     sleep_ms(250);
    //     gpio_put(LED_PIN, 0);
    //     sleep_ms(250);
    // }


    
   
    
    uint i2s_offset = pio_add_program(pio, &i2s_32_441_program);
    uint ws2812b_offset = pio_add_program(pio, &ws2812b_program);


    ws2812b_program_init(pio, 1, RGB_LEDS_GPIO, ws2812b_offset);


    rgb_leds_update_timer_init();


    int audio_dma_channel = audio_dma_init(audio_buffs);

    audio_irq_init(audio_dma_channel);

    

    i2s_32_441_program_init(pio, 0, I2S_CLK_GPIO, I2S_DATA_GPIO, I2S_FS_GPIO, i2s_offset);

    // while (1)
    // {
    //     for (uint32_t i = 0; i < data_size; i++)
    //     {   
    //         int16_t s = (int16_t)((uint16_t)(song_of_storms[i + 4] << 8) - 0x7FFF);
    //         uint32_t d = (uint32_t)(s << 16);
    //         // printf("Sample: 0x%08x\n", d);
    //         pio_sm_put_blocking(pio, sm, d);
    //         pio_sm_put_blocking(pio, sm, d);
    //     }
    // }
    

    //dma_init(stereo_data, data_size);



    

    // while(index < data_size)
    // {

    // }

    // for (uint8_t i = 0; i < 48; i++)
    // {    
    //     printf("Color 0x%08x\n", colors[i]);
    //     pio_sm_put_blocking(pio, 1, colors[i]);
    // }

    printf("Done.\n");
    printf("Goodbye\n");

    float vol_mult = (float)MAX_VOL / 128.0f;
    uint32_t mask = (1 << NOTE_D_GPIO)
        | (1 << NOTE_F_GPIO)
        | (1 << NOTE_A_GPIO)
        | (1 << NOTE_B_GPIO)
        | (1 << NOTE_D2_GPIO);
    bool start_song = false;

    while(1)
    {
        adc_select_input(0);
        vol = adc_read() >> 5;
        // printf("volume: %f\n", ((float)vol * vol_mult));
        vol = (uint8_t)((float)vol * vol_mult);
        // vol = vol > MAX_VOL ? MAX_VOL : vol;
        vol_running_avg[vol_avg_index%VOL_AVG] = vol;
        current_volume = get_vol_avg();
        vol_avg_index++;
        
        sleep_ms(25);

        uint32_t buttons = gpio_get_all();
        // printf("buttons: 0x%08x\n", buttons & ((1 << NOTE_D_GPIO)
        // | (1 << NOTE_F_GPIO)
        // | (1 << NOTE_A_GPIO)
        // | (1 << NOTE_B_GPIO)
        // | (1 << NOTE_D2_GPIO)));

        if (! audio_button_down)
        {
            

            switch (buttons & ((1 << NOTE_D_GPIO)
                | (1 << NOTE_F_GPIO)
                | (1 << NOTE_A_GPIO)
                | (1 << NOTE_B_GPIO)
                | (1 << NOTE_D2_GPIO)))
            {
                case (1 << NOTE_D_GPIO):
                printf("Note D Button\n");
                    audio_button_down = true;
                    set_current_sample(d_loop);
                    start_song = true;
                    break;
                case (1 << NOTE_F_GPIO):
                printf("Note F Button\n");
                    audio_button_down = true;
                    set_current_sample(f_loop);
                    start_song = true;            
                    break;
                case (1 << NOTE_A_GPIO):
                printf("Note A Button\n");
                    audio_button_down = true;
                    set_current_sample(a_loop);
                    start_song = true;            
                    break;
                case (1 << NOTE_B_GPIO):
                printf("Note B Button\n");
                    audio_button_down = true;
                    set_current_sample(b_loop);
                    start_song = true;            
                    break;
                case (1 << NOTE_D2_GPIO):
                printf("Note D2 Button\n");
                    audio_button_down = true;
                    set_current_sample(d2_loop);
                    start_song = true;            
                    break;                                           
            }

            if (start_song)
            {   
                printf("starting song\n");
                current_buff_index = 0;                             
                load_audio_buffer(current_sample, current_sample_len, &current_sample_offset, audio_buffs, current_volume); 
                start_song = false;
                audio_dma_isr();
            }

        }

        audio_button_down = ((buttons & (1 << NOTE_D_GPIO))
            | (buttons & (1 << NOTE_F_GPIO))
            | (buttons & (1 << NOTE_A_GPIO))
            | (buttons & (1 << NOTE_B_GPIO))
            | (buttons & (1 << NOTE_D2_GPIO)));

        if (!audio_button_down && dma_channel_is_busy(audio_dma_channel))
        {
            
            dma_channel_abort(audio_dma_channel);
            pio_sm_clear_fifos(pio, 0);
            pio_sm_restart(pio, 0);
        }
    }
}   


    // set_current_sample(song_of_storms);


    // load_audio_buffer(current_sample, current_sample_len, &current_sample_offset, audio_buffs, current_volume); 


    // 