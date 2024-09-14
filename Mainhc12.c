#include <driver/i2c.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "HD44780.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "esp_timer.h"
#include <time.h>
#include "esp_system.h"
#include "driver/gpio.h"
#include <unistd.h>
#include <driver/uart.h>

#define LCD_ADDR 0x27
#define SDA_PIN  23
#define SCL_PIN  22
#define LCD_COLS 16
#define LCD_ROWS 2
#define BOTAO_MAIS 15
#define BOTAO_MENOS 19
#define BOTAO_ENTER_FISICO 4
#define UART_NUM UART_NUM_2 // Define a UART que será utilizada
#define TX_PIN 17           // Pino TX do ESP32
#define RX_PIN 16           // Pino RX do ESP32 (alterado para não conflitar com BOTAO_ENTER)
#define BUF_SIZE 1024       // Tamanho do buffer
// Servo configuration
#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US 2500  // Maximum pulse width in microsecond
#define SERVO_MIN_DEGREE        0   // Minimum angle
#define SERVO_MAX_DEGREE        180    // Maximum angle
#define SERVO_PULSE_GPIO        13    // GPIO connects to the PWM signal line
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD   20000    // 20000 ticks, 20ms

#define TRIGGER_PIN GPIO_NUM_27
#define ECHO_PIN GPIO_NUM_26

static const float DISTANCE_MIN = 5.0; // Distância em centímetros


int tela_atual = 0;
static const char *TAG = "example";

// Servo handles
mcpwm_cmpr_handle_t comparator;
mcpwm_timer_handle_t timer;

static inline uint32_t example_angle_to_compare(int angle) {
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}

void servo_mover(int angle) {
    ESP_LOGI(TAG, "Move servo to %d degrees", angle);
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(angle)));
}

void servo_init(void) {
    ESP_LOGI(TAG, "Create timer and operator");
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    ESP_LOGI(TAG, "Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    ESP_LOGI(TAG, "Create comparator and generator from the operator");
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = SERVO_PULSE_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

    ESP_LOGI(TAG, "Set generator action on timer and compare event");
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}

void sensor_init(){
    gpio_set_direction(TRIGGER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);
}


float measure_distance() {
    // Gera o pulso de trigger
    gpio_set_level(TRIGGER_PIN, 0);
    usleep(2);
    gpio_set_level(TRIGGER_PIN, 1);
    usleep(10);
    gpio_set_level(TRIGGER_PIN, 0);

    // Variáveis para armazenar o tempo
    int64_t start_time = 0;
    int64_t end_time = 0;

    // Espera o início do pulso do Echo
    int64_t wait_time = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 0 && (esp_timer_get_time() - wait_time) < 1000000) {
        start_time = esp_timer_get_time();
    }

    // Espera o fim do pulso do Echo
    wait_time = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 1 && (esp_timer_get_time() - wait_time) < 1000000) {
        end_time = esp_timer_get_time();
    }

    // Calcula a duração do pulso
    int64_t duration = end_time - start_time;

    // Calcula a distância em centímetros
    float distance = duration / 58.0;

    // Retorna a distância calculada
    return distance;
}

void mostrar_introducao(){
    LCD_writeStr("Dispenser ");
    LCD_setCursor(1, 2);
    LCD_writeStr("Inteligente");
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    LCD_clearScreen();
}

void mostrar_tela(int tela) {
    LCD_clearScreen();
    switch (tela) {
        case 0:
            LCD_writeStr("Peso ");
            LCD_setCursor(1, 2);
            LCD_writeStr("250g");
            break;
        case 1:
            LCD_writeStr("Peso ");
            LCD_setCursor(1, 2);
            LCD_writeStr("500g");
            break;
        case 2:
            LCD_writeStr("Peso ");
            LCD_setCursor(1, 2);
            LCD_writeStr("750g");
            break;
        default:
            LCD_writeStr("Tela Inválida");
            break;
    }
}

void hc12_init() {
    // Configuração da UART
    const uart_config_t uart_config = {
        .baud_rate = 9600,  // Taxa de baud do HC-12 (padrão é 9600)
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, BUF_SIZE, 0, 0, NULL, 0);
}

bool verificar_hc12() {
    uint8_t data[BUF_SIZE];
    // Ler os dados recebidos pela UART
    int len = uart_read_bytes(UART_NUM, data, BUF_SIZE, 20 / portTICK_RATE_MS);
    if (len > 0) {
        // Se receber "ENTER", retorna true
        if (strstr((char*)data, "ENTER") != NULL) {
            ESP_LOGI(TAG, "Comando ENTER recebido via HC-12");
            return true;
        }
    }
    return false;
}

void verificar_botoes()  {
    // Verifica o botão ENTER físico ou o comando ENTER via HC-12
    if (gpio_get_level(BOTAO_ENTER_FISICO) == 0 || verificar_hc12()) {
        float distance = measure_distance();
        if(tela_atual == 0){
            LCD_clearScreen();
            LCD_writeStr("Pedido ");
            LCD_setCursor(0, 2);
            LCD_writeStr("Confirmado ");
            while (distance > DISTANCE_MIN) {
                ESP_LOGI(TAG, "Distância ainda maior que o mínimo: %f. Esperando...", distance);
                distance = measure_distance();  
                vTaskDelay(500 / portTICK_PERIOD_MS);  
            } 
            ESP_LOGI(TAG, "Distância menor ou igual ao mínimo, movendo servo...");
            servo_mover(180);  
            vTaskDelay(3000 / portTICK_PERIOD_MS);  
            servo_mover(0);  
            ESP_LOGI(TAG, "Servo movimento concluído, mostrando tela atual.");
            mostrar_tela(tela_atual);  
        } 
        else if(tela_atual == 1){
            LCD_clearScreen();
            LCD_writeStr("Pedido ");
            LCD_setCursor(0, 2);
            LCD_writeStr("Confirmado ");
            while(distance > DISTANCE_MIN){
                ESP_LOGI(TAG, "Distância ainda maior que o mínimo: %f. Esperando...", distance);
                distance = measure_distance();  
                vTaskDelay(500 / portTICK_PERIOD_MS);
            }
            ESP_LOGI(TAG, "Distância menor ou igual ao mínimo, movendo servo...");
            servo_mover(180);  
            vTaskDelay(4500 / portTICK_PERIOD_MS);  
            servo_mover(0);  
            ESP_LOGI(TAG, "Servo movimento concluído, mostrando tela atual.");
            mostrar_tela(tela_atual);  
        } 
        else if(tela_atual == 2){
            LCD_clearScreen();
            LCD_writeStr("Pedido ");
            LCD_setCursor(0, 2);
            LCD_writeStr("Confirmado ");
            while(distance > DISTANCE_MIN){
                ESP_LOGI(TAG, "Distância ainda maior que o mínimo: %f. Esperando...", distance);
                distance = measure_distance();  
                vTaskDelay(500 / portTICK_PERIOD_MS);
            }
            ESP_LOGI(TAG, "Distância menor ou igual ao mínimo, movendo servo...");
            servo_mover(180);  
            vTaskDelay(6000 / portTICK_PERIOD_MS); 
            servo_mover(0);  
            ESP_LOGI(TAG, "Servo movimento concluído, mostrando tela atual.");
            mostrar_tela(tela_atual);
        }
        else{
            LCD_clearScreen();
            LCD_writeStr("Acao  ");
            LCD_setCursor(0, 2);
            LCD_writeStr("Invalida ");
        }
        vTaskDelay(300 / portTICK_PERIOD_MS);  // Debounce
    }

    // Verificação dos outros botões (BOTAO_MAIS e BOTAO_MENOS)
    if (gpio_get_level(BOTAO_MAIS) == 0) {
        tela_atual = (tela_atual + 1) % 3;
        mostrar_tela(tela_atual);
        vTaskDelay(300 / portTICK_PERIOD_MS);  // Debounce
    }
    
    if (gpio_get_level(BOTAO_MENOS) == 0) {
        tela_atual = (tela_atual - 1 + 3) % 3;
        mostrar_tela(tela_atual);
        vTaskDelay(300 / portTICK_PERIOD_MS);  // Debounce
    }
}

void LCD_DemoTask(void* param) {
    mostrar_introducao();
    mostrar_tela(tela_atual);
    while (true) {
        verificar_botoes();
        vTaskDelay(100 / portTICK_PERIOD_MS);  // Verifica os botões a cada 100ms
    }
}

void app_main(void) {
    LCD_init(LCD_ADDR, SDA_PIN, SCL_PIN, LCD_COLS, LCD_ROWS);
    gpio_set_direction(BOTAO_MAIS, GPIO_MODE_INPUT);
    gpio_set_direction(BOTAO_MENOS, GPIO_MODE_INPUT);
    gpio_set_direction(BOTAO_ENTER_FISICO, GPIO_MODE_INPUT);  // Configura o novo pino para o botão ENTER físico

    gpio_pullup_en(BOTAO_MAIS);   // Ativa pull-up
    gpio_pullup_en(BOTAO_MENOS);  // Ativa pull-up
    gpio_pullup_en(BOTAO_ENTER_FISICO);  // Ativa pull-up no novo pino do botão ENTER físico

    servo_init();  // Inicializa o servo
    sensor_init();
    hc12_init();   // Inicializa o HC-12


    xTaskCreate(&LCD_DemoTask, "Demo Task", 2048, NULL, 5, NULL);
}
