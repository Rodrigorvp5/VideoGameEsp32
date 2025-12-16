#include "driver/gpio.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_spiffs.h"
#include "esp_timer.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "esp_adc/adc_continuous.h"
#include "esp_log.h"

#include "hal/gpio_types.h"
#include "lv_core/lv_obj.h"
#include "lv_core/lv_obj_style_dec.h"
#include "lv_font/lv_font.h"
#include "lv_font/lv_symbol_def.h"
#include "lv_misc/lv_area.h"
#include "lv_misc/lv_color.h"
#include "lv_widgets/lv_cont.h"
#include "lv_widgets/lv_label.h"
#include "lvgl_helpers.h"

#include "led_strip.h"

#include "driver/uart.h"

#include "driver/i2c.h"
#include "ssd1306.h"


#define UART_PORT UART_NUM_1
#define UART_TX 17


#define LED_GPIO    2
#define LED_COUNT   3

#define MUESTRAS_FILTRO 16 // Mientras más alto, más suave (pero con más "lag"). 16 o 32 está bien.

uint32_t suma_x = 0;
uint32_t suma_y = 0;
int indice_filtro = 0;

// CONFIGURACIÓN PRINCIPAL DE LA PANTALLA, DEFINICIÓN DE TAMAÑO PARA QUE COINCIDA CON EL DRIVER
#define LV_TICK_PERIOD_MS 1
#define SCREEN_WIDTH  240
#define SCREEN_HEIGHT 320

//ESTO ES EL PIXEL SIZE DE LAS NAVES
#define PIXEL_SIZE 2
#define PIXEL_SIZE_PLAYER 3

#define PIXEL_SIZE_MISIL 2

//ESTO SERÁ POR SI ME CONFUNDO DESPUÉS EN LOS PINES PODER CAMBIARLO AQUÍ Y NO FÍSICO 
//JOYSTICK
#define CANALY ADC_CHANNEL_3
#define CANALX ADC_CHANNEL_0

volatile int joystick_x = 1900;
volatile int joystick_y = 1900;

//EN FISICO LA CONFIGURACIÓN ACTUAL QUE TENGO ES:
//EL LADO DE LOS PINES ES EL LADO DE ABAJO 

//ENTONCES:

//X IZQUEIRDA ES 0 ------ X DERECHA 4095
//Y ABAJO ES CERO ----- Y ARRIBA ES 4095

//AHORITA MISMO EL MEDIO ES 1900X  Y   1800Y 

#define JUGADOR_T_X (11*PIXEL_SIZE_PLAYER)
#define JUGADOR_T_Y (13*PIXEL_SIZE_PLAYER)

#define ENEMIGO_T_X (11*PIXEL_SIZE)
#define ENEMIGO_T_Y ( 12*PIXEL_SIZE)

#define LED_STRIP_RMT_RES_HZ  (10 * 1000 * 1000)
#define LED_STRIP_MEMORY_BLOCK_WORDS 0
#define LED_STRIP_USE_DMA  0

#define ENEMIGO_H_T_X   (11 * PIXEL_SIZE)     // ancho sprite enemigo horizontal
#define ENEMIGO_H_T_Y   ( 8 * PIXEL_SIZE)     // alto sprite enemigo horizontal

#define MISIL_ENEMIGO_T_X (2 * PIXEL_SIZE_MISIL)
#define MISIL_ENEMIGO_T_Y (5 * PIXEL_SIZE_MISIL)

#define MAX_ENEMIGOS_H       3
#define MAX_BALAS_ENEMIGAS   2

#define VEL_ENEMIGO_H        2
#define VEL_BALA_ENEMIGA     4

#define I2C_SDA 27
#define I2C_SCL 25
#define I2C_PORT I2C_NUM_1
#define I2C_FREQ 400000

ssd1306_handle_t oled = NULL;

int CambioX = 2500;     //CAMBIO POSITIVO 
int CambioY = 2500;     //CAMBIO POSTIVO

int CambioXMenos = 1000;
int CambioYMenos = 1000;

int PixelesMovimiento = 5;

// Variable global que aumentará con el tiempo
int VelocidadGlobalEnemigosH = VEL_ENEMIGO_H;

// Variable para controlar la velocidad de los disparos enemigos
int VelocidadGlobalBalas = VEL_BALA_ENEMIGA;


// Posicion de spawm de la nave kamikace fuera de la pantalla que no sea vea 
int nave_x = (SCREEN_WIDTH / 2) - 20; // Centrado horizontalmente
int nave_y = SCREEN_HEIGHT;           // Empieza abajo (fuera de pantalla)

//Variables de la posicion de la nava de jugador
int posicionX = (SCREEN_WIDTH/2)-20;
int posicionY = (SCREEN_HEIGHT/2) - 140;

bool BanderaDeChoque = false;

int misilX = (SCREEN_WIDTH/2)-20;
int misilY = SCREEN_HEIGHT + 50;

#define BOTON_DISPARO GPIO_NUM_16

bool MisilDisparado = false;
int velocidad_misil = 5; 

//POsicion máxima que puede alcanzar el jugador en Y
//int PosMaxYPlayer = 220;

//AHORA VAMOS CON EL SISTEMA DE VELOCIDAD, PUNTOS Y 
static const char *TAG = "JUEGO";


int Puntuacion = 0;
volatile int vidas = 3;

typedef struct {
    int x;
    int y;
    int vel_x;
    bool activo;
} EnemigoHorizontal;

typedef struct {
    int x;
    int y;
    int vel_y;
    bool activo;
} BalaEnemiga;

EnemigoHorizontal enemigosH[MAX_ENEMIGOS_H];
BalaEnemiga balasEnemigas[MAX_BALAS_ENEMIGAS];


int VelocidadNaveKami = 3;

//AQUI VOY A CREAR EL ESTADO DEL JUEGO PARA HACER LA PANTALLA DE INICIO:

typedef enum {
    ESTADO_INICIO,
    ESTADO_JUGANDO
}EstadoJuego;

EstadoJuego estadoActual = ESTADO_INICIO;


#define MISIL_T_X (3 * PIXEL_SIZE_MISIL)
#define MISIL_T_Y (8 * PIXEL_SIZE_MISIL)

volatile bool ocultarPantallaInicio = false;



// ESTO ES EL SEMAFORO 
SemaphoreHandle_t xGuiSemaphore;
lv_obj_t * Pantalla;
lv_obj_t * PantallaInicio;
lv_obj_t * ContenedorNave; //ESTE CONTENEDOR AGRUPA A LA NAVE KAMIKACE
lv_obj_t * ContenedorNaveJugador;
lv_obj_t * ContenedorMisil;

lv_obj_t *ContenedoresEnemigosH[MAX_ENEMIGOS_H];
lv_obj_t *ContenedoresBalasEnemigas[MAX_BALAS_ENEMIGAS];


// ================= PROTOTIPOS =================
static void lv_tick_task(void *arg);
static void guiTask(void *pvParameter);
static void animacionNaveTask(void *pvParameter);
static void animacionNaveJugadorPruebaTask(void *pvParameter);
static void animacionMisil(void *pvParameter);
static void BotonTask(void *pvParameter);
static void dibujar_nave_pixelart(lv_obj_t * parent);
static void dibujar_nave_jugador(lv_obj_t * puntero);
static void dibujar_misil(lv_obj_t * puntero);
static void IniciarADC(void);
static void IniciarPines(void);
static void tareaADC(void *pv);
static bool FuncionVerificarColision(int jugadorX, int jugadorY, int EnemigoX, int EnemigoY);
static bool ColisionMisilEnemigo(int misilX, int misilY, int enemigoX, int enemigoY);
static void iniciarTimer30s();
static void Timer30s_Callback(void *arg);
static void send_audio_cmd(const char *cmd);
static void neopixel_init();
static void neopixel_VidasTask(void *pvParameter);
static void ReiniciarJuego();
static void dibujar_nave_horizontal(lv_obj_t *parent);
static void dibujar_misil_enemigo(lv_obj_t *parent, int idx);
static void animacionEnemigosHorizontales(void *pvParameter);
static void animacionBalasEnemigas(void *pvParameter);
static bool ColisionBalaJugador(int balaX, int balaY, int jugadorX, int jugadorY);
static void disparar_bala_enemiga(int ex, int ey);
static void inicializar_enemigos_y_balas();
static void spawn_enemigo_horizontal(int idx);
static bool ColisionJugadorEnemigoH(int jugadorX, int jugadorY, int enemigoX, int enemigoY);
static void i2c_init(void);
static void setup_storage(void);
static void setup_peripherals(void);
static void setup_uart(void);
static void start_game_tasks(void);
static void init_timer_service(void);

adc_continuous_handle_t handle = NULL;
esp_timer_handle_t timer30s;
led_strip_handle_t strip;

// ================= INIT HELPERS =================
static void setup_storage(void) {
    esp_vfs_spiffs_conf_t DiskConf = {
        .base_path = "/Datos",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = false
    };

    esp_vfs_spiffs_register(&DiskConf);
}

static void setup_peripherals(void) {
    IniciarADC();
    IniciarPines();
    neopixel_init();
    i2c_init();
}

static void setup_uart(void) {
    uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    uart_param_config(UART_PORT, &cfg);
    uart_set_pin(UART_PORT, UART_TX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_PORT, 1024, 0, 0, NULL, 0);
}

static void start_game_tasks(void) {
    xTaskCreate(tareaADC, "tareaADC", 4096, NULL, 5, NULL);

    xTaskCreatePinnedToCore(animacionNaveTask, "animacion", 2048, NULL, tskIDLE_PRIORITY+1, NULL, 0);
    xTaskCreatePinnedToCore(animacionNaveJugadorPruebaTask, "animacion2", 2048, NULL, tskIDLE_PRIORITY+1, NULL, 0);
    xTaskCreatePinnedToCore(animacionMisil, "animacion3", 2048, NULL, tskIDLE_PRIORITY+1, NULL, 0);

    xTaskCreatePinnedToCore(BotonTask, "disparar", 2048, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(neopixel_VidasTask, "vidas", 2048, NULL, 2, NULL, 0);

    xTaskCreatePinnedToCore(animacionEnemigosHorizontales, "enemigosH", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(animacionBalasEnemigas, "balasEnemigas", 4096, NULL, 2, NULL, 0);
}

static void init_timer_service(void) {
    const esp_timer_create_args_t timer_args = {
        .callback = &Timer30s_Callback,
        .name = "timer_30s"
    };

    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer30s));
}

// ================= APP MAIN =================
void app_main() {
    setup_storage();

    // 1. Tarea GUI (Pantalla)
    xTaskCreatePinnedToCore(guiTask, "gui", 4096*2, NULL, tskIDLE_PRIORITY+1, NULL, 1);

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    setup_peripherals();
    inicializar_enemigos_y_balas();

    OLED_MostrarPuntuacion(0);

    start_game_tasks();
    init_timer_service();
    setup_uart();

    vTaskDelete(NULL);
}

static void send_audio_cmd(const char *cmd)
{
    uart_write_bytes(UART_PORT, cmd, strlen(cmd));
}

static void IniciarADC(void)
{

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 1024,
        .conv_frame_size = 256,
    };

    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_digi_pattern_config_t pattern[2] = {0};

    pattern[0].atten = ADC_ATTEN_DB_11;      // 0–3.3V
    pattern[0].channel = CANALY;      // ADC1_CH0  = GPIO36
    pattern[0].unit = ADC_UNIT_1;            // usar ADC1
    pattern[0].bit_width = ADC_BITWIDTH_12;

    pattern[1].atten = ADC_ATTEN_DB_11;
    pattern[1].channel = CANALX;      // ADC1_CH0  = GPIO36
    pattern[1].unit = ADC_UNIT_1;            // usar ADC1
    pattern[1].bit_width = ADC_BITWIDTH_12;

    adc_continuous_config_t configADC = {
        .sample_freq_hz = 20 * 1000,              // 20 kHz
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,      // SOLO ADC1
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,   // formato correcto
        .pattern_num = 2,
        .adc_pattern = pattern,
    };

    ESP_ERROR_CHECK(adc_continuous_config(handle, &configADC));

    ESP_ERROR_CHECK(adc_continuous_start(handle));
}

static void IniciarPines(){
    gpio_set_direction(BOTON_DISPARO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BOTON_DISPARO, GPIO_PULLDOWN_ONLY);
}

static void i2c_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ
    };

    i2c_param_config(I2C_PORT, &conf);
    i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);

    
    oled = ssd1306_create(I2C_PORT, SSD1306_I2C_ADDRESS);
    if (oled == NULL) {
        ESP_LOGE("OLED", "Error creando SSD1306!");
        return;
    }

    ssd1306_init(oled);
    ssd1306_clear_screen(oled, 0x00);
    ssd1306_refresh_gram(oled);
}

void OLED_MostrarPuntuacion(int puntos)
{
    if (oled == NULL) return;  // seguridad extra

    char text[20];
    sprintf(text, "Puntos: %d", puntos);

    ssd1306_clear_screen(oled, 0x00);
    ssd1306_draw_string(oled, 0, 0, (uint8_t *)text, 16, 1);
    ssd1306_refresh_gram(oled);
}


static void inicializar_enemigos_y_balas()
{
    for (int i = 0; i < MAX_ENEMIGOS_H; i++) {
        enemigosH[i].activo = false;
        enemigosH[i].x = -100;
        enemigosH[i].y = 60 + i * 40; // filas distintas
        enemigosH[i].vel_x = VEL_ENEMIGO_H;
    }

    for (int i = 0; i < MAX_BALAS_ENEMIGAS; i++) {
        balasEnemigas[i].activo = false;
        balasEnemigas[i].x = -100;
        balasEnemigas[i].y = -100;
        balasEnemigas[i].vel_y = VEL_BALA_ENEMIGA;
    }
}

static void neopixel_init() {

    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_GPIO, // The GPIO that connected to the LED strip's data line
        .max_leds = LED_COUNT,      // The number of LEDs in the strip,
        .led_model = LED_MODEL_WS2812,        // LED strip model
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB, // The color order of the strip: GRB
        .flags = {
            .invert_out = false, // don't invert the output signal
        }
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,        // different clock source can lead to different power consumption
        .resolution_hz = LED_STRIP_RMT_RES_HZ, // RMT counter clock frequency
        .mem_block_symbols = LED_STRIP_MEMORY_BLOCK_WORDS, // the memory block size used by the RMT channel
        .flags = {
            .with_dma = LED_STRIP_USE_DMA,     // Using DMA can improve performance when driving more LEDs
        }
    };

    
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &strip));
    ESP_LOGI(TAG, "Created LED strip object with RMT backend");
    
}

static void neopixel_VidasTask(void *pvParameter) {
    int ultimaVida = -1;

    while (1) {

        if (vidas != ultimaVida) {

            // Limpiar tira
            ESP_ERROR_CHECK(led_strip_clear(strip));

            // Encender LEDs según vidas
            for (int i = 0; i < LED_COUNT; i++) {

                if (i < vidas) {
                    // Verde (0, 255, 0)
                    led_strip_set_pixel(strip, i, 0, 255, 0);
                } else {
                    
                    led_strip_set_pixel(strip, i, 255, 0, 0);
                    ESP_LOGI(TAG, "vidas: %d", vidas);
                }
            }

            // REFRESCAR
            ESP_ERROR_CHECK(led_strip_refresh(strip));

            ultimaVida = vidas;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}



static void BotonTask(void *pvParameter){

    bool estadoAnterior = false;
    uint32_t tiempoUltimo = 0;

    while(1){
        bool lectura = gpio_get_level(BOTON_DISPARO);

        uint32_t ahora = xTaskGetTickCount();

        if(lectura != estadoAnterior){
            if((ahora - tiempoUltimo)>pdMS_TO_TICKS(20)){
                estadoAnterior = lectura;

                if(lectura == true && !MisilDisparado){

                    if(estadoActual == ESTADO_INICIO){

                        estadoActual = ESTADO_JUGANDO;
                         // ejemplo: reproduce música 1
                        send_audio_cmd("PLAY JUGANDO\n");
                        ESP_LOGI(TAG, "CMD MANDADO");
                        ocultarPantallaInicio = true;
                        iniciarTimer30s();
                        ESP_LOGI(TAG, "ESTADO JUGANDO");


                    }else if(estadoActual == ESTADO_JUGANDO){

                        misilX = posicionX + 13;
                        misilY = posicionY - 10;

                        MisilDisparado = true;
                        ESP_LOGI(TAG, "DISPARO");
                        send_audio_cmd("PLAY DISPARO");
                    }
                    
                }
            }

            tiempoUltimo = ahora;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void tareaADC(void *pv)
{
    uint8_t raw_buffer[256];
    uint32_t bytes_leidos = 0;

    while (1)
    {
        // 1. Cambia el timeout a 0 o algo bajo, ya no queremos que bloquee tanto aquí
        esp_err_t ret = adc_continuous_read(handle, raw_buffer, sizeof(raw_buffer), &bytes_leidos, 0);

        if (ret == ESP_OK)
        {
            adc_digi_output_data_t *datos = (adc_digi_output_data_t *)raw_buffer;
            int num_muestras = bytes_leidos / sizeof(adc_digi_output_data_t);

          for (int i = 0; i < num_muestras; i++)
            {
                // Obtenemos canal y valor de ESTA muestra específica
                int chan_actual = datos[i].type1.channel;
                int val_actual = datos[i].type1.data;

                // Clasificamos
                if (chan_actual == CANALX) {
                    joystick_x = val_actual;
                } 
                else if (chan_actual == CANALY) {
                    joystick_y = val_actual;
                }

                // --- Filtrado simple ---
                static int x_ant = 0;
                static int y_ant = 0;

                joystick_x = (joystick_x + x_ant) / 2;
                joystick_y = (joystick_y + y_ant) / 2;

                x_ant = joystick_x;
                y_ant = joystick_y;
                }

            // Opcional: Imprimir solo de vez en cuando para no saturar
            //printf("X: %d | Y: %d\n", joystick_x, joystick_y);
        }

        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}


//DIBUJO DE LA NAVE (NAVES KAMIKACE)

static void dibujar_nave_pixelart(lv_obj_t * parent) {

    static const uint8_t sprite_map[12][11] = {
        {0,0,0,0,0,1,0,0,0,0,0}, // Punta
        {0,0,0,0,1,1,1,0,0,0,0},
        {0,0,0,0,1,2,1,0,0,0,0}, // Cabina (2)
        {0,0,0,1,1,1,1,1,0,0,0},
        {0,0,1,1,0,1,0,1,1,0,0},
        {0,1,1,1,1,1,1,1,1,1,0},
        {1,1,0,1,1,1,1,1,0,1,1}, // Alas
        {1,0,0,1,1,1,1,1,0,0,1},
        {1,0,0,1,0,0,0,1,0,0,1},
        {0,0,0,2,0,0,0,2,0,0,0}, // Motores
        {0,0,0,2,0,0,0,2,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0} 
    };

    static lv_style_t style_pixel_body;
    lv_style_init(&style_pixel_body);
    lv_style_set_bg_color(&style_pixel_body, LV_STATE_DEFAULT, LV_COLOR_WHITE); // Nave Blanca
    lv_style_set_border_width(&style_pixel_body, LV_STATE_DEFAULT, 0);
    lv_style_set_radius(&style_pixel_body, LV_STATE_DEFAULT, 0); // Cuadrados perfectos

    static lv_style_t style_pixel_fire;
    lv_style_init(&style_pixel_fire);
    lv_style_set_bg_color(&style_pixel_fire, LV_STATE_DEFAULT, LV_COLOR_RED); // Fuego Rojo
    lv_style_set_border_width(&style_pixel_fire, LV_STATE_DEFAULT, 0);

    for(int y=0; y<12; y++) {
        for(int x=0; x<11; x++) {
            uint8_t pixel = sprite_map[y][x];
            if(pixel > 0) {
                lv_obj_t * p = lv_obj_create(parent, NULL);
                lv_obj_set_size(p, PIXEL_SIZE, PIXEL_SIZE);
                lv_obj_set_pos(p, x * PIXEL_SIZE, y * PIXEL_SIZE);
                
                if(pixel == 1) lv_obj_add_style(p, LV_OBJ_PART_MAIN, &style_pixel_body);
                if(pixel == 2) lv_obj_add_style(p, LV_OBJ_PART_MAIN, &style_pixel_fire);
            }
        }
    }
    
    
    lv_obj_set_size(parent, 11 * PIXEL_SIZE, 12 * PIXEL_SIZE);
}

//AQUI IRÁ EL CODIGO PARA LA NAVE DEL JUGADOR:

static void dibujar_nave_jugador(lv_obj_t * parent){

    //PIXELART NAVE DEL JUGADOR
    static const uint8_t sprite_map[13][11] = {
        {0,0,0,0,0,3,0,0,0,0,0},
        {0,0,0,0,0,3,0,0,0,0,0},
        {2,0,0,0,3,3,3,0,0,0,2},
        {2,0,0,3,3,3,3,3,0,0,2},
        {2,0,0,3,3,3,3,3,0,0,2},
        {2,1,3,3,1,3,1,3,3,1,2},
        {2,2,1,1,1,3,1,1,1,2,2},
        {2,2,1,1,1,1,1,1,1,2,2},
        {0,0,2,1,0,1,0,1,2,0,0},
        {0,0,0,2,0,1,0,2,0,0,0},
        {0,0,0,0,0,1,0,0,0,0,0},
        {0,0,0,0,0,1,0,0,0,0,0},
        {0,0,0,0,0,1,0,0,0,0,0}
    };


    static lv_style_t style_pixel_1;
    lv_style_init(&style_pixel_1);
    lv_style_set_bg_color(&style_pixel_1, LV_STATE_DEFAULT, LV_COLOR_RED); 
    lv_style_set_border_width(&style_pixel_1, LV_STATE_DEFAULT, 0);
    lv_style_set_radius(&style_pixel_1, LV_STATE_DEFAULT, 0); // Cuadrados perfectos

    static lv_style_t style_pixel_2;
    lv_style_init(&style_pixel_2);
    lv_style_set_bg_color(&style_pixel_2, LV_STATE_DEFAULT, LV_COLOR_YELLOW); 
    lv_style_set_border_width(&style_pixel_2, LV_STATE_DEFAULT, 0);

    static lv_style_t style_pixel_3;
    lv_style_init(&style_pixel_3);
    lv_style_set_bg_color(&style_pixel_3, LV_STATE_DEFAULT, LV_COLOR_BLUE); 
    lv_style_set_border_width(&style_pixel_3, LV_STATE_DEFAULT, 0);

    for(int y=0; y<13; y++) {
        for(int x=0; x<11; x++) {
            uint8_t pixel = sprite_map[y][x];
            if(pixel > 0) {
                lv_obj_t * p = lv_obj_create(parent, NULL);
                lv_obj_set_size(p, PIXEL_SIZE_PLAYER, PIXEL_SIZE_PLAYER);
                lv_obj_set_pos(p, x * PIXEL_SIZE_PLAYER, y * PIXEL_SIZE_PLAYER);
                
                if(pixel == 1) lv_obj_add_style(p, LV_OBJ_PART_MAIN, &style_pixel_1);
                if(pixel == 2) lv_obj_add_style(p, LV_OBJ_PART_MAIN, &style_pixel_2);
                if(pixel == 3) lv_obj_add_style(p, LV_OBJ_PART_MAIN, &style_pixel_3);
            }
        }
    }

    lv_obj_set_size(parent, 11 * PIXEL_SIZE_PLAYER, 13 * PIXEL_SIZE_PLAYER);
}

static void dibujar_misil(lv_obj_t *parent){
    static const uint8_t misilpixel[8][3]={
        {0,2,0},
        {0,1,0},
        {0,1,0},
        {2,3,2},
        {1,3,1},
        {0,3,0},
        {0,3,0},
        {0,3,0}
    };

    static lv_style_t style1;
    lv_style_init(&style1);
    lv_style_set_bg_color(&style1, LV_STATE_DEFAULT, LV_COLOR_RED); 
    lv_style_set_border_width(&style1, LV_STATE_DEFAULT, 0);
    lv_style_set_radius(&style1, LV_STATE_DEFAULT, 0); // Cuadrados perfectos

    static lv_style_t style2;
    lv_style_init(&style2);
    lv_style_set_bg_color(&style2, LV_STATE_DEFAULT, LV_COLOR_YELLOW); 
    lv_style_set_border_width(&style2, LV_STATE_DEFAULT, 0);

    static lv_style_t style3;
    lv_style_init(&style3);
    lv_style_set_bg_color(&style3, LV_STATE_DEFAULT, LV_COLOR_CYAN); 
    lv_style_set_border_width(&style3, LV_STATE_DEFAULT, 0);

    for(int y=0; y<8; y++) {
        for(int x=0; x<3; x++) {
            uint8_t pixel = misilpixel[y][x];
            if(pixel > 0) {
                lv_obj_t * p = lv_obj_create(parent, NULL);

                lv_obj_set_size(p, PIXEL_SIZE_MISIL, PIXEL_SIZE_MISIL);
                lv_obj_set_pos(p, x * PIXEL_SIZE_MISIL, y * PIXEL_SIZE_MISIL);
                
                if(pixel == 1) lv_obj_add_style(p, LV_OBJ_PART_MAIN, &style1);
                if(pixel == 2) lv_obj_add_style(p, LV_OBJ_PART_MAIN, &style2);
                if(pixel == 3) lv_obj_add_style(p, LV_OBJ_PART_MAIN, &style3);
            }
        }
    }

    lv_obj_set_size(parent, 3 * PIXEL_SIZE_MISIL, 8 * PIXEL_SIZE_MISIL);
}

static void dibujar_nave_horizontal(lv_obj_t *parent)
{
    // Sprite 11x8
    static const uint8_t sprite[8][11] = {
        {0,0,0,0,0,6,6,0,0,0,0},
        {0,0,0,0,1,3,3,1,0,0,0},
        {0,0,0,1,1,3,3,1,1,0,0},
        {0,0,1,1,3,3,3,3,1,1,0},
        {0,1,1,3,3,3,3,3,3,1,1},
        {0,1,3,3,3,3,3,3,3,3,1},
        {0,0,1,1,1,3,3,1,1,1,0},
        {0,0,0,1,1,1,1,1,1,0,0}
    };

    // Canvas interno del enemigo
    lv_obj_t *canvas = lv_canvas_create(parent, NULL);

    // Tamaño real en pantalla
    int w = ENEMIGO_H_T_X;
    int h = ENEMIGO_H_T_Y;

    // Buffer donde se dibuja
    static lv_color_t *buf = NULL;
    if (buf == NULL)
        buf = heap_caps_malloc(w * h * sizeof(lv_color_t), MALLOC_CAP_DMA);

    // USAR CHROMA KEY
    lv_canvas_set_buffer(canvas, buf, w, h, LV_IMG_CF_TRUE_COLOR_CHROMA_KEYED);

    // LLENAR CON EL COLOR CHROMA KEY (verde)
    lv_canvas_fill_bg(canvas, LV_COLOR_LIME, LV_OPA_COVER);

    // Pinta pixel por pixel
    for (int y = 0; y < 8; y++) {
        for (int x = 0; x < 11; x++) {

            uint8_t pix = sprite[y][x];
            if (pix == 0)
                continue;

            lv_color_t c =
                (pix == 1 ? LV_COLOR_SILVER :
                 pix == 3 ? LV_COLOR_BLUE   :
                 pix == 6 ? LV_COLOR_RED    :
                             LV_COLOR_WHITE );

            // Expandir pixel al tamaño deseado
            for (int dy = 0; dy < PIXEL_SIZE; dy++) {
                for (int dx = 0; dx < PIXEL_SIZE; dx++) {
                    lv_canvas_set_px(canvas,
                                     x * PIXEL_SIZE + dx,
                                     y * PIXEL_SIZE + dy,
                                     c);
                }
            }
        }
    }

    lv_obj_set_size(canvas, w, h);
}


static void dibujar_misil_enemigo(lv_obj_t *parent, int idx)
{
    // Sprite 2x5 en codificación 3 = pixel rojo
    static const uint8_t misil[5][2] = {
        {0,3},
        {0,3},
        {3,3},
        {0,3},
        {0,3}
    };

    lv_obj_t *canvas = lv_canvas_create(parent, NULL);

    int w = MISIL_ENEMIGO_T_X;
    int h = MISIL_ENEMIGO_T_Y;

    // Cada bala tiene su propio buffer
    static lv_color_t *bufArr[MAX_BALAS_ENEMIGAS] = {0};

    if (bufArr[idx] == NULL)
        bufArr[idx] = heap_caps_malloc(w * h * sizeof(lv_color_t), MALLOC_CAP_DMA);

    // Formato con chroma key (transparencia)
    lv_canvas_set_buffer(canvas, bufArr[idx], w, h, LV_IMG_CF_TRUE_COLOR_CHROMA_KEYED);

    // Llenar con el color transparente (verde)
    lv_canvas_fill_bg(canvas, LV_COLOR_LIME, LV_OPA_COVER);

    // Dibujar los pixeles
    for (int y = 0; y < 5; y++)
    {
        for (int x = 0; x < 2; x++)
        {
            uint8_t pix = misil[y][x];
            if (pix != 3) continue;

            lv_color_t c = LV_COLOR_YELLOW;

            for (int dy = 0; dy < PIXEL_SIZE_MISIL; dy++) {
                for (int dx = 0; dx < PIXEL_SIZE_MISIL; dx++) {
                    lv_canvas_set_px(canvas,
                                     x * PIXEL_SIZE_MISIL + dx,
                                     y * PIXEL_SIZE_MISIL + dy,
                                     c);
                }
            }
        }
    }

    lv_obj_set_size(canvas, w, h);
}



static void animacionEnemigosHorizontales(void *pvParameter)
{
    while (1) {

        if (estadoActual != ESTADO_JUGANDO) {
            // Si no se está jugando, ocultar todos
            if (xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
                for (int i = 0; i < MAX_ENEMIGOS_H; i++) {
                    lv_obj_set_hidden(ContenedoresEnemigosH[i], true);
                    enemigosH[i].activo = false;
                }
                xSemaphoreGive(xGuiSemaphore);
            }
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        for (int i = 0; i < MAX_ENEMIGOS_H; i++) {

            if (!enemigosH[i].activo) {
                // Respawn automático
                spawn_enemigo_horizontal(i);
                continue;
            }

            // ======== COLISIÓN CON EL JUGADOR ========
            if (ColisionJugadorEnemigoH(posicionX, posicionY, enemigosH[i].x, enemigosH[i].y))
            {
                // Pantalla roja como la nave kamikaze
                if (xSemaphoreTake(xGuiSemaphore, portMAX_DELAY))
                {
                    lv_obj_set_style_local_bg_color(Pantalla, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_RED);
                    xSemaphoreGive(xGuiSemaphore);
                }

                vTaskDelay(pdMS_TO_TICKS(700));

                if (xSemaphoreTake(xGuiSemaphore, portMAX_DELAY))
                {
                    lv_obj_set_style_local_bg_color(Pantalla, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_BLACK);
                    

                    lv_obj_set_hidden(ContenedoresBalasEnemigas[i], true);
                    lv_obj_set_hidden(ContenedoresEnemigosH[i], true);

                    balasEnemigas[i].activo = false;
                    enemigosH[i].activo = false;

                    xSemaphoreGive(xGuiSemaphore);
                }

                // Restar vida
                if (vidas > 0)
                {
                    vidas--;
                    VelocidadNaveKami = 3;
                    VelocidadGlobalEnemigosH = VEL_ENEMIGO_H;
                    VelocidadGlobalBalas = VEL_BALA_ENEMIGA;
                    for (int i = 0; i < MAX_ENEMIGOS_H; i++) {
                        if (enemigosH[i].activo) {
                            if (enemigosH[i].vel_x > 0)
                                enemigosH[i].vel_x = 2;
                            else if (enemigosH[i].vel_x < 0)
                                enemigosH[i].vel_x = 2;
                        }
                    }
                    send_audio_cmd("PLAY EXPLOSION");
                }
                
                if(vidas == 0){
                    ESP_LOGI(TAG, "ESTO SE ESTA EJECUTANDO");
                    send_audio_cmd("PLAY OVER\n");        // Parar fondo
                    vTaskDelay(pdMS_TO_TICKS(400));   // Esperar
                    send_audio_cmd("PLAY STOP\n");   // Tocar Over
                    vTaskDelay(pdMS_TO_TICKS(100)); 
                    ReiniciarJuego();
                    
                    continue;
                }

                // Resetear enemigo fuera de pantalla
                enemigosH[i].activo = false;
                if (xSemaphoreTake(xGuiSemaphore, portMAX_DELAY))
                {
                    lv_obj_set_hidden(ContenedoresEnemigosH[i], true);
                    xSemaphoreGive(xGuiSemaphore);
                }

                continue;  // Saltar al siguiente enemigo
            }


            // Mover
            enemigosH[i].x += enemigosH[i].vel_x;

            // Salió de la pantalla, re-spawn
            if (enemigosH[i].x > SCREEN_WIDTH + ENEMIGO_H_T_X ||
                enemigosH[i].x < -ENEMIGO_H_T_X * 2) {
                enemigosH[i].activo = false;
                if (xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
                    lv_obj_set_hidden(ContenedoresEnemigosH[i], true);
                    xSemaphoreGive(xGuiSemaphore);
                }
                continue;
            }

            // Actualizar posición en LVGL
            if (xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
                lv_obj_set_pos(ContenedoresEnemigosH[i], enemigosH[i].x, enemigosH[i].y);
                xSemaphoreGive(xGuiSemaphore);
            }

            // Probabilidad de disparar
            if ((rand() % 1000) < 10) { // ~1% por ciclo
                disparar_bala_enemiga(enemigosH[i].x, enemigosH[i].y);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(25));
    }
}

static bool ColisionBalaJugador(int balaX, int balaY, int jugadorX, int jugadorY)
{
    if (balaX > jugadorX + JUGADOR_T_X) return false;
    if (balaX + MISIL_ENEMIGO_T_X < jugadorX) return false;
    if (balaY > jugadorY + JUGADOR_T_Y) return false;
    if (balaY + MISIL_ENEMIGO_T_Y < jugadorY) return false;
    return true;
}

static void animacionBalasEnemigas(void *pvParameter)
{
    while (1)
    {
        if (estadoActual != ESTADO_JUGANDO)
        {
            if (xSemaphoreTake(xGuiSemaphore, portMAX_DELAY))
            {
                for (int i = 0; i < MAX_BALAS_ENEMIGAS; i++)
                {
                    balasEnemigas[i].activo = false;
                    lv_obj_set_hidden(ContenedoresBalasEnemigas[i], true);
                }
                xSemaphoreGive(xGuiSemaphore);
            }
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        for (int i = 0; i < MAX_BALAS_ENEMIGAS; i++)
        {
            if (!balasEnemigas[i].activo) continue;

            // Mover enemigo hacia abajo
            balasEnemigas[i].y -= balasEnemigas[i].vel_y;

            // Fuera de pantalla → desactivar
            if (balasEnemigas[i].y < -20)
            {
                if (xSemaphoreTake(xGuiSemaphore, portMAX_DELAY))
                {
                    lv_obj_set_hidden(ContenedoresBalasEnemigas[i], true);
                    xSemaphoreGive(xGuiSemaphore);
                }

                balasEnemigas[i].activo = false;
                continue;
            }

            // Dibujar posición
            if (xSemaphoreTake(xGuiSemaphore, portMAX_DELAY))
            {
                lv_obj_set_pos(ContenedoresBalasEnemigas[i],
                               balasEnemigas[i].x,
                               balasEnemigas[i].y);
                xSemaphoreGive(xGuiSemaphore);
            }

            // COLISIÓN CON EL JUGADOR
            if (ColisionBalaJugador(balasEnemigas[i].x, balasEnemigas[i].y,
                                    posicionX, posicionY))
            {
                // Ocultar la bala SIEMPRE antes de desactivar
                if (xSemaphoreTake(xGuiSemaphore, portMAX_DELAY))
                {
                    lv_obj_set_hidden(ContenedoresBalasEnemigas[i], true);
                    xSemaphoreGive(xGuiSemaphore);
                }

                if(xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)){

                    nave_y = SCREEN_HEIGHT; // Reinicia abajo
                    // Efecto extra: cambiar posicion X aleatoria en cada vuelta
                    nave_x = (rand() % (SCREEN_WIDTH - 60)) + 10; 

                    // Aplicar posicion
                    lv_obj_set_pos(ContenedorNave, nave_x, nave_y);

                    xSemaphoreGive(xGuiSemaphore);
                }

                balasEnemigas[i].activo = false;

                // Lógica de daño al jugador
                if (vidas > 0)
                {
                    vidas--;

                    VelocidadNaveKami = 3;
                    VelocidadGlobalEnemigosH = VEL_ENEMIGO_H;
                    VelocidadGlobalBalas = VEL_BALA_ENEMIGA;

                    send_audio_cmd("PLAY EXPLOSION");
                    ESP_LOGI(TAG, "GOLPE POR BALA ENEMIGA, vidas: %d", vidas);

                    if (xSemaphoreTake(xGuiSemaphore, portMAX_DELAY))
                    {
                        lv_obj_set_style_local_bg_color(Pantalla, LV_OBJ_PART_MAIN,
                                                        LV_STATE_DEFAULT, LV_COLOR_RED);
                        xSemaphoreGive(xGuiSemaphore);
                    }

                    vTaskDelay(pdMS_TO_TICKS(600));

                    if (xSemaphoreTake(xGuiSemaphore, portMAX_DELAY))
                    {
                        lv_obj_set_style_local_bg_color(Pantalla, LV_OBJ_PART_MAIN,
                                                        LV_STATE_DEFAULT, LV_COLOR_BLACK);

                        lv_obj_set_hidden(ContenedoresBalasEnemigas[i], true);
                        lv_obj_set_hidden(ContenedoresEnemigosH[i], true);

                        balasEnemigas[i].activo = false;
                        enemigosH[i].activo = false;

                        nave_x = (SCREEN_WIDTH / 2) - 20;
                        nave_y = SCREEN_HEIGHT;
                    
                        lv_obj_set_pos(ContenedorNave, nave_x, nave_y);

                        xSemaphoreGive(xGuiSemaphore);

                    }

                }

                if(vidas == 0){

                    ESP_LOGI(TAG, "GAME OVER");
        
                    // 1. Detener música de fondo PRIMERO
                    send_audio_cmd("PLAY STOP\n"); 
                    
                    // 2. Darle tiempo al ESP de Audio para procesar el STOP (50ms es suficiente)
                    vTaskDelay(pdMS_TO_TICKS(50)); 
                    
                    // 3. Ahora sí, reproducir Game Over
                    send_audio_cmd("PLAY OVER\n");
                    
                    // 4. Esperar un poco para que no se reinicie instantáneamente (opcional)
                    vTaskDelay(pdMS_TO_TICKS(400)); 

                    ReiniciarJuego(); // Ya no envía STOP, así que OVER sigue sonando
                    ESP_LOGI(TAG, "JUEGO TERMINADO");

                }

            }

            for (int i = 0; i < MAX_ENEMIGOS_H; i++)
            {
                if (!enemigosH[i].activo) continue;

                if (misilX < enemigosH[i].x + ENEMIGO_H_T_X &&
                    misilX + MISIL_T_X > enemigosH[i].x &&
                    misilY < enemigosH[i].y + ENEMIGO_H_T_Y &&
                    misilY + MISIL_T_Y > enemigosH[i].y)
                {
                    // Desactivar misil
                    MisilDisparado = false;
                    misilY = SCREEN_HEIGHT + 50;

                    // Desactivar enemigo
                    send_audio_cmd("PLAY EXPLOSION2");
                    Puntuacion+=15;
                    OLED_MostrarPuntuacion(Puntuacion);

                    enemigosH[i].activo = false;
                    if (xSemaphoreTake(xGuiSemaphore, portMAX_DELAY))
                    {
                        lv_obj_set_hidden(ContenedoresEnemigosH[i], true);
                        xSemaphoreGive(xGuiSemaphore);
                    }

                    
                    send_audio_cmd("PLAY EXPLOSION");

                    // Salir del for para evitar doble detección
                    break;
                }
            }

        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}



// ================= TAREA DE ANIMACION =================
static void animacionNaveTask(void *pvParameter){
    while(1){

        if(estadoActual != ESTADO_JUGANDO){
            vTaskDelay(20);
            continue;
        }

        if(!BanderaDeChoque){
            // --- LOGICA VERTICAL ---
            nave_y -= VelocidadNaveKami; // Restamos Y para subir (0,0 esta arriba a la izquierda)
            

            // Si la nave sale por arriba de la pantalla
            if(nave_y < -50) { // -50 para dar margen a que salga completa
                nave_y = SCREEN_HEIGHT; // Reinicia abajo
                // Efecto extra: cambiar posicion X aleatoria en cada vuelta
                nave_x = (rand() % (SCREEN_WIDTH - 60)) + 10; 
            }

            if(xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)){
                // Aplicar posicion
                lv_obj_set_pos(ContenedorNave, nave_x, nave_y);

                xSemaphoreGive(xGuiSemaphore);
            }

            if(FuncionVerificarColision(posicionX, posicionY, nave_x, nave_y)){
                BanderaDeChoque = true;

                if (xSemaphoreTake(xGuiSemaphore, portMAX_DELAY))
                {
                    for (int i = 0; i < MAX_BALAS_ENEMIGAS; i++)
                    {       
                    lv_obj_set_hidden(ContenedoresBalasEnemigas[i], true);
                    balasEnemigas[i].activo = false;
                    }

                    xSemaphoreGive(xGuiSemaphore);
                }


                if (vidas > 0) {
                    vidas--;
                    VelocidadNaveKami = 3;
                    VelocidadGlobalEnemigosH = VEL_ENEMIGO_H;
                    VelocidadGlobalBalas = VEL_BALA_ENEMIGA;
                    
                    for (int i = 0; i < MAX_ENEMIGOS_H; i++) {
                        if (enemigosH[i].activo) {
                            if (enemigosH[i].vel_x > 0)
                                enemigosH[i].vel_x += 2;
                            else if (enemigosH[i].vel_x < 0)
                                enemigosH[i].vel_x -= 2;
                        }
                    }

                    ESP_LOGI(TAG, "VELOCIDAD REESTABLECIDA");
                    send_audio_cmd("PLAY EXPLOSION");
                    ESP_LOGI(TAG, "CHOCO");
                }

                if(vidas == 0){
                    ESP_LOGI(TAG, "GAME OVER");
        
                    // 1. Detener música de fondo PRIMERO
                    send_audio_cmd("PLAY STOP\n"); 
                    
                    // 2. Darle tiempo al ESP de Audio para procesar el STOP (50ms es suficiente)
                    vTaskDelay(pdMS_TO_TICKS(100)); 
                    
                    // 3. Ahora sí, reproducir Game Over
                    send_audio_cmd("PLAY OVER\n");
                    
                    // 4. Esperar un poco para que no se reinicie instantáneamente (opcional)
                    vTaskDelay(pdMS_TO_TICKS(400)); 

                    ReiniciarJuego(); // Ya no envía STOP, así que OVER sigue sonando
                    ESP_LOGI(TAG, "JUEGO TERMINADO");
                }
                    
                
            }
        }else{

            if(xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)){
                lv_obj_set_style_local_bg_color(Pantalla, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_RED);
                xSemaphoreGive(xGuiSemaphore);
            }

            vTaskDelay(pdMS_TO_TICKS(700));
            
            if(xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)){
                lv_obj_set_style_local_bg_color(Pantalla, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_BLACK);

                nave_x = (SCREEN_WIDTH / 2) - 20;
                nave_y = SCREEN_HEIGHT;
            
                lv_obj_set_pos(ContenedorNave, nave_x, nave_y);

                xSemaphoreGive(xGuiSemaphore);
            }
            
            BanderaDeChoque = false;

        }

         vTaskDelay(pdMS_TO_TICKS(20)); 
    }
}

static void animacionMisil(void *pvParameter){
    while(1){
        if(MisilDisparado){
            misilY += velocidad_misil;
        }

        if(misilY > SCREEN_HEIGHT){
            MisilDisparado = false;
            misilY = SCREEN_HEIGHT + 50;
        }

        if(ColisionMisilEnemigo(misilX, misilY, nave_x, nave_y)){
            MisilDisparado = false;
            misilY = SCREEN_HEIGHT + 50;

            send_audio_cmd("PLAY EXPLOSION");

            Puntuacion += 10;
            OLED_MostrarPuntuacion(Puntuacion);


            nave_x = (rand() % (SCREEN_WIDTH - 60)) + 10;
            nave_y = SCREEN_HEIGHT;

            ESP_LOGI(TAG, "Enemigo destruido, Puntuacion:  %d", Puntuacion);
            
        }

        if(xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)){
            lv_obj_set_pos(ContenedorMisil, misilX, misilY);
            xSemaphoreGive(xGuiSemaphore);
        }

        vTaskDelay(pdMS_TO_TICKS(15));
    }
}


static void animacionNaveJugadorPruebaTask(void *pvParameter){

    const int ANCHO_NAVE = 11 * PIXEL_SIZE_PLAYER; 
    const int ALTO_NAVE = 13 * PIXEL_SIZE_PLAYER;

    while(1){

        if(estadoActual != ESTADO_JUGANDO){
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)){

            if(BanderaDeChoque){
                xSemaphoreGive(xGuiSemaphore);
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }

            if(joystick_x > CambioX ){
                posicionX -= PixelesMovimiento;
            }else if(joystick_x < CambioXMenos){
                posicionX += PixelesMovimiento;    
            }
            if(joystick_y < CambioYMenos) {
                 posicionY -= PixelesMovimiento; // Bajar
            }
            // Tu Joystick: Y Arriba es 4095 -> Queremos que SUBA (Restar Y)
            else if(joystick_y > CambioY) {
                 posicionY += PixelesMovimiento; // Subir
            }

            if(posicionX < 0) posicionX = 0;
            
            if(posicionX > (SCREEN_WIDTH - ANCHO_NAVE)) posicionX = (SCREEN_WIDTH - ANCHO_NAVE);

            if(posicionY < 0) posicionY = 0;

            // Límite Inferior (Suelo)
            if(posicionY > (SCREEN_HEIGHT - ALTO_NAVE)) posicionY = (SCREEN_HEIGHT - ALTO_NAVE);

            // ---------------- APLICAR ----------------
            
            lv_obj_set_pos(ContenedorNaveJugador, posicionX, posicionY);
            
            xSemaphoreGive(xGuiSemaphore);
        }

        vTaskDelay(pdMS_TO_TICKS(35));
    }

}

static bool FuncionVerificarColision(int jugadorX, int jugadorY, int EnemigoX, int EnemigoY){

    if(jugadorX > EnemigoX + ENEMIGO_T_X) return false;

    if(jugadorX + JUGADOR_T_X < EnemigoX) return false;

    if(jugadorY > EnemigoY + ENEMIGO_T_Y) return false;

    if(jugadorY + JUGADOR_T_Y < EnemigoY) return false;

    return true;
}

static bool ColisionMisilEnemigo(int misilX, int misilY, int enemigoX, int enemigoY)
{
    // Si el misil está a la derecha del enemigo → no chocan
    if(misilX > enemigoX + ENEMIGO_T_X) return false;

    // Si el misil está a la izquierda del enemigo
    if(misilX + MISIL_T_X < enemigoX) return false;

    // Si el misil está abajo del enemigo
    if(misilY > enemigoY + ENEMIGO_T_Y) return false;

    // Si el misil está arriba del enemigo
    if(misilY + MISIL_T_Y < enemigoY) return false;

    return true;
}


// ================= TAREA GUI =================
static void guiTask(void *pvParameter) {
    xGuiSemaphore = xSemaphoreCreateMutex();
    
    lv_init();
    lvgl_driver_init();

    lv_color_t* buf1 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    lv_color_t* buf2 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    
    static lv_disp_buf_t disp_buf;
    lv_disp_buf_init(&disp_buf, buf1, buf2, DISP_BUF_SIZE); 
    
    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.flush_cb = disp_driver_flush;
    disp_drv.buffer = &disp_buf;
    
    // FORZAR RESOLUCION 
    disp_drv.hor_res = SCREEN_WIDTH; 
    disp_drv.ver_res = SCREEN_HEIGHT;

    lv_disp_drv_register(&disp_drv); 
   
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &lv_tick_task,
        .name = "periodic_gui"
    };
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000));

    // ----------------- INTERFAZ GRAFICA -----------------
    
    if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
        
        // 1. FONDO NEGRO COMPLETO
        Pantalla = lv_scr_act();
        static lv_style_t style_bg;
        lv_style_init(&style_bg);
        lv_style_set_bg_color(&style_bg, LV_STATE_DEFAULT, LV_COLOR_BLACK);
        lv_style_set_bg_opa(&style_bg, LV_STATE_DEFAULT, LV_OPA_COVER);
        lv_obj_add_style(Pantalla, LV_OBJ_PART_MAIN, &style_bg);
        
        // --- LIMPIEZA FORZADA DE MEMORIA DE VIDEO ---
        // Esto le dice a LVGL que redibuje TODO el objeto Pantalla ahora mismo
        // sobreescribiendo cualquier basura anterior en la GRAM del LCD.
        lv_obj_invalidate(Pantalla);
        
        PantallaInicio = lv_cont_create(Pantalla, NULL);
        lv_obj_add_style(PantallaInicio, LV_CONT_PART_MAIN, &style_bg);
        lv_obj_set_size(PantallaInicio, SCREEN_WIDTH, SCREEN_HEIGHT);

        // 2. CREAR CONTENEDOR DE LA NAVE
        ContenedorNave = lv_cont_create(Pantalla, NULL);
        // Hacemos el contenedor transparente
        static lv_style_t style_transp;
        lv_style_init(&style_transp);
        lv_style_set_bg_opa(&style_transp, LV_STATE_DEFAULT, LV_OPA_TRANSP);
        lv_style_set_border_width(&style_transp, LV_STATE_DEFAULT, 0);
        lv_obj_add_style(ContenedorNave, LV_CONT_PART_MAIN, &style_transp);
        
        // 3. DIBUJAR LOS PIXELES
        dibujar_nave_pixelart(ContenedorNave);
        
        // Posicion Inicial
        lv_obj_set_pos(ContenedorNave, nave_x, nave_y);

        ContenedorNaveJugador = lv_cont_create(Pantalla, NULL);
        lv_obj_add_style(ContenedorNaveJugador, LV_CONT_PART_MAIN, &style_transp);

        dibujar_nave_jugador(ContenedorNaveJugador);

        lv_obj_set_pos(ContenedorNaveJugador, posicionX, posicionY);

        ContenedorMisil = lv_cont_create(Pantalla, NULL);
        lv_obj_add_style(ContenedorMisil, LV_CONT_PART_MAIN, &style_transp);
        lv_obj_set_pos(ContenedorMisil, misilX, misilY);

        dibujar_misil(ContenedorMisil);

                // ===== Enemigos horizontales =====
        for (int i = 0; i < MAX_ENEMIGOS_H; i++) {
            ContenedoresEnemigosH[i] = lv_cont_create(Pantalla, NULL);
            lv_obj_add_style(ContenedoresEnemigosH[i], LV_CONT_PART_MAIN, &style_transp);

            dibujar_nave_horizontal(ContenedoresEnemigosH[i]);

            lv_obj_set_pos(ContenedoresEnemigosH[i], enemigosH[i].x, enemigosH[i].y);
            lv_obj_set_hidden(ContenedoresEnemigosH[i], true);
        }

        // ===== Balas enemigas =====
        for (int i = 0; i < MAX_BALAS_ENEMIGAS; i++) {
            ContenedoresBalasEnemigas[i] = lv_cont_create(Pantalla, NULL);
            lv_obj_add_style(ContenedoresBalasEnemigas[i], LV_CONT_PART_MAIN, &style_transp);

            dibujar_misil_enemigo(ContenedoresBalasEnemigas[i], i);


            lv_obj_set_pos(ContenedoresBalasEnemigas[i], balasEnemigas[i].x, balasEnemigas[i].y);
            lv_obj_set_hidden(ContenedoresBalasEnemigas[i], true);
        }

        xSemaphoreGive(xGuiSemaphore);
    }

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10));
        if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {

            if(ocultarPantallaInicio){
                lv_obj_set_hidden(PantallaInicio, true);
                ESP_LOGI(TAG, "SE DEBIO APAGAR LA PANTALLA");
                ocultarPantallaInicio = false;
            }
            lv_task_handler();
            xSemaphoreGive(xGuiSemaphore);
        }
    }

    free(buf1);
    free(buf2);
    vTaskDelete(NULL);
}

static void disparar_bala_enemiga(int ex, int ey)
{
    for (int i = 0; i < MAX_BALAS_ENEMIGAS; i++)
    {
        if (!balasEnemigas[i].activo)
        {
            balasEnemigas[i].activo = true;
            
            // AQUÍ EL CAMBIO: Usar la variable global
            balasEnemigas[i].vel_y = VelocidadGlobalBalas; 

            balasEnemigas[i].x = ex + ENEMIGO_H_T_X/2 - MISIL_ENEMIGO_T_X/2;
            balasEnemigas[i].y = ey + ENEMIGO_H_T_Y;

            // ... resto del código LVGL ...
            if (xSemaphoreTake(xGuiSemaphore, portMAX_DELAY))
            {
                lv_obj_set_pos(ContenedoresBalasEnemigas[i], balasEnemigas[i].x, balasEnemigas[i].y);
                lv_obj_set_hidden(ContenedoresBalasEnemigas[i], false);
                xSemaphoreGive(xGuiSemaphore);
            }
            return;
        }
    }
}

static bool ColisionJugadorEnemigoH(int jugadorX, int jugadorY, int enemigoX, int enemigoY)
{
    if (jugadorX > enemigoX + ENEMIGO_H_T_X) return false;
    if (jugadorX + JUGADOR_T_X < enemigoX) return false;
    if (jugadorY > enemigoY + ENEMIGO_H_T_Y) return false;
    if (jugadorY + JUGADOR_T_Y < enemigoY) return false;

    return true;
}


static void iniciarTimer30s()
{
    
    esp_timer_stop(timer30s);

    esp_timer_start_once(timer30s, 30 * 1000000);
}


static void Timer30s_Callback(void *arg)
{
    ESP_LOGI("TIMER", "¡DIFICULTAD AUMENTADA!");

    // 1. Naves
    VelocidadNaveKami += 1;
    VelocidadGlobalEnemigosH += 1;
    
    // 2. AUMENTAR VELOCIDAD DE BALAS ENEMIGAS
    VelocidadGlobalBalas += 1; // Aumentamos la velocidad base para las siguientes

    // --- Actualizar enemigos horizontales (código anterior) ---
    for (int i = 0; i < MAX_ENEMIGOS_H; i++) {
        if (enemigosH[i].activo) {
            if (enemigosH[i].vel_x > 0) enemigosH[i].vel_x = VelocidadGlobalEnemigosH;
            else enemigosH[i].vel_x = -VelocidadGlobalEnemigosH;
        }
    }

    // --- ACTUALIZAR BALAS YA DISPARADAS (EN EL AIRE) ---
    for (int i = 0; i < MAX_BALAS_ENEMIGAS; i++) {
        if (balasEnemigas[i].activo) {
            // Actualizamos su velocidad actual a la nueva
            balasEnemigas[i].vel_y = VelocidadGlobalBalas;
        }
    }

    iniciarTimer30s();
}




static void spawn_enemigo_horizontal(int idx)
{
    enemigosH[idx].activo = true;

    // Dirección aleatoria: 0 = izquierda->derecha, 1 = derecha->izquierda
    int dir = rand() % 2;

    if (dir == 0) {
        // USA LA VARIABLE GLOBAL AQUÍ
        enemigosH[idx].vel_x = VelocidadGlobalEnemigosH; 
        enemigosH[idx].x = -ENEMIGO_H_T_X;
    } else {
        // Y AQUÍ (negativa para ir a la izquierda)
        enemigosH[idx].vel_x = -VelocidadGlobalEnemigosH; 
        enemigosH[idx].x = SCREEN_WIDTH;
    }

    enemigosH[idx].y = 215 + idx * 40;

    // ... resto del código de lvgl ...
    if (xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
        lv_obj_set_pos(ContenedoresEnemigosH[idx], enemigosH[idx].x, enemigosH[idx].y);
        lv_obj_set_hidden(ContenedoresEnemigosH[idx], false);
        xSemaphoreGive(xGuiSemaphore);
    }
}


static void ReiniciarJuego(void)
{
    estadoActual = ESTADO_INICIO;
    send_audio_cmd("PLAY STOP\n");
    vTaskDelay(pdMS_TO_TICKS(20));
    send_audio_cmd("PLAY STOP\n");
    

    ESP_LOGI(TAG, "STOP MANDADO");

    vidas = 3;
    Puntuacion = 0;
    OLED_MostrarPuntuacion(Puntuacion);

    BanderaDeChoque = false;
    MisilDisparado = false;

    posicionX = (SCREEN_WIDTH/2)-20;
    posicionY = (SCREEN_HEIGHT/2) - 140;

    nave_x = (SCREEN_WIDTH / 2) - 20;
    nave_y = SCREEN_HEIGHT;

    misilX = posicionX + 13;
    misilY = SCREEN_HEIGHT + 50;

    VelocidadNaveKami = 3;
    VelocidadGlobalEnemigosH = VEL_ENEMIGO_H;
    VelocidadGlobalBalas = VEL_BALA_ENEMIGA;

    esp_timer_stop(timer30s);

    // Reset enemigos y balas
    for (int i = 0; i < MAX_ENEMIGOS_H; i++) {
        enemigosH[i].activo = false;
        enemigosH[i].x = -100;
    }
    for (int i = 0; i < MAX_BALAS_ENEMIGAS; i++) {
        balasEnemigas[i].activo = false;
        balasEnemigas[i].x = -100;
    }

    if (xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
        lv_obj_set_style_local_bg_color(Pantalla, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_BLACK);

        lv_obj_set_pos(ContenedorNave, nave_x, nave_y);
        lv_obj_set_pos(ContenedorNaveJugador, posicionX, posicionY);
        lv_obj_set_pos(ContenedorMisil, misilX, misilY);

        for (int i = 0; i < MAX_ENEMIGOS_H; i++) {
            lv_obj_set_hidden(ContenedoresEnemigosH[i], true);
        }
        for (int i = 0; i < MAX_BALAS_ENEMIGAS; i++) {
            lv_obj_set_hidden(ContenedoresBalasEnemigas[i], true);
        }

        lv_obj_set_hidden(PantallaInicio, false);

        xSemaphoreGive(xGuiSemaphore);
    }

    ESP_LOGI(TAG, "=== JUEGO REINICIADO ===");
}


static void lv_tick_task(void *arg) {
    (void) arg;
    lv_tick_inc(LV_TICK_PERIOD_MS);
}