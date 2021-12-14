/*********************************************************
* Projeto de trena eletrônica usando o sensor ultrasônico
* HCSR04 e o display OLED

* Por: Renan de Brito Leme e Roniere Rezende
*********************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/*Livrarias WiFi*/
#include "esp_event_loop.h" /*Inicializa o WiFi quando há um event handler*/
#include "esp_event.h"
#include "esp_wifi.h"
#include "nvs_flash.h" /*Inicializa os handlers do WiFi*/
#include "esp_http_client.h"

#include "ssd1306.h"
#include "fonts.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"

#include "sdkconfig.h"
#include "driver/i2c.h"
#include "esp_types.h"
#include "sys/time.h"

#include "ultrasonic.h"
#include "driver/dac.h"

#include "wifi.h"
#include "mqtt.h"

#define MAX_DISTANCE_CM 450 // 4.5m max
#define GPIO_TRIGGER	19
#define GPIO_ECHO	    18

#define GPIO_INCREMENTA 5
#define GPIO_DECREMENTA 4

#define QUEUE_SLOT      2

#define TAG "MQTT"

/*Protótipos das Tasks*/
void vTask_Ultrasonic (void *pvParamters);
void vTask_SSD1306 (void *pvParamters);
void vTask_INC_DEC (void *pvParamters);
void conectadoWifi(void *pvParameters);
void trataComunicacaoComServidor(void *pvParameters);

/*Protótipos de funções*/

/*Declaração das filas*/
QueueHandle_t xQueue_HCSR04_to_SSD1306; 
QueueHandle_t xQueue_INC_DEC_to_SSD1306; 

/*Declaração das semáforos*/
SemaphoreHandle_t xconexaoWifiSemaphore;
SemaphoreHandle_t xconexaoMQTTSemaphore;

/*Variáveis globais*/
uint32_t distancia = 0;
uint32_t inc_dec_value = 0;

void app_main()
{
	/* Inicialização do NVS*/
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	xconexaoWifiSemaphore = xSemaphoreCreateBinary();
	xconexaoMQTTSemaphore = xSemaphoreCreateBinary();
	
	wifi_start();
	SSD1306_Init();

    xQueue_HCSR04_to_SSD1306 = xQueueCreate(QUEUE_SLOT, sizeof(uint32_t));
	xQueue_INC_DEC_to_SSD1306 = xQueueCreate(QUEUE_SLOT, sizeof(uint32_t));
    
	xTaskCreate(vTask_Ultrasonic, "vTask_Ultrasonic", configMINIMAL_STACK_SIZE + 1024, NULL, 1, NULL);
	xTaskCreate(vTask_SSD1306, "vTask_SSD1306", configMINIMAL_STACK_SIZE + 1024, NULL, 2, NULL);
	xTaskCreate(vTask_INC_DEC, "vTask_INC_DEC", configMINIMAL_STACK_SIZE + 1024, NULL, 3, NULL);
	xTaskCreate(&conectadoWifi, "Conexão ao MQTT", 4096, NULL, 4, NULL);
	xTaskCreate(&trataComunicacaoComServidor, "Comunicação com o Broker", 4096, NULL, 5, NULL);
}

void vTask_Ultrasonic(void *pvParamters)
{
	xSemaphoreGive(xconexaoMQTTSemaphore);

	ultrasonic_sensor_t sensor = {
		.trigger_pin = GPIO_TRIGGER,
		.echo_pin = GPIO_ECHO
	};

	ultrasonic_init(&sensor);

	while (true) {

		xSemaphoreTake(xconexaoMQTTSemaphore, portMAX_DELAY);
        //ESP_LOGI("vTask_Ultrasonic","Pega o semáforo e lê o valor do sensor");

		uint32_t distance;
		esp_err_t res = ultrasonic_measure_cm(&sensor, MAX_DISTANCE_CM, &distance);
		if (res != ESP_OK) {
			printf("Error: ");
			switch (res) {
				case ESP_ERR_ULTRASONIC_PING:
					printf("Cannot ping (device is in invalid state)\n");
					break;
				case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
					printf("Ping timeout (no device found)\n");
					break;
				case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
					printf("Echo timeout (i.e. distance too big)\n");
					break;
				default:
					printf("%d\n", res);
			}
		} else {
			//printf("Distance: %d cm, %.02f m\n", (distance+1), (distance+1) / 100.0);
		}
		
		//ESP_LOGI("vTask_Ultrasonic","Envia o valor do sensor para a fila");
        xQueueSend(xQueue_HCSR04_to_SSD1306, &distance, portMAX_DELAY); /* Sends currente value of adc variable to queue */
        printf("Sensor -> Fila: %d centímetros\n", distance+1);
        
		//ESP_LOGI("vTask_Ultrasonic","Devolve o semáforo");
        xSemaphoreGive(xconexaoMQTTSemaphore);

		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

void vTask_SSD1306 (void *pvParameters)
{   
    (void)pvParameters;

    xSemaphoreGive(xconexaoMQTTSemaphore);

    char str [17];
    //uint32_t distancia = 0;
	uint32_t inc_dec = 0;

    SSD1306_Clear();    
    SSD1306_GotoXY(0,0);
    vTaskDelay(10);
    sprintf(str,"TRENA ELETRONICA");
    SSD1306_Puts(str, &Font_7x10, 1);

    SSD1306_UpdateScreen();

    while (1)
    {
        xSemaphoreTake(xconexaoMQTTSemaphore, portMAX_DELAY);
        //ESP_LOGI("vTask_SSD1306 ","Pega o semáforo e mostra o valor do sensor");

        xQueueReceive(xQueue_HCSR04_to_SSD1306, &distancia, portMAX_DELAY);
		xQueueReceive(xQueue_INC_DEC_to_SSD1306, &inc_dec, portMAX_DELAY);
        //ESP_LOGI("vTask_SSD1306 ","Recebe o valor do sensor via fila");
		printf("Fila -> para a variável distância: %d centímetros\n", distancia+1);
		printf("Fila -> para a variável valor desejado: %d valor desejado\n", inc_dec);

		SSD1306_GotoXY(0,20);
    	vTaskDelay(10);
    	sprintf(str,"Valor desej: %d", inc_dec);
    	SSD1306_Puts(str, &Font_7x10, 1);
    	//SSD1306_DrawLine(1,18, 127, 18, SSD1306_COLOR_WHITE);
        //SSD1306_UpdateScreen();

		SSD1306_GotoXY(0,40);
        vTaskDelay(10);
        sprintf(str, "Valor med: %d", distancia+1);
        SSD1306_Puts(str, &Font_7x10, 1);

		if (distancia+1 == inc_dec)
		{
			SSD1306_GotoXY(0,50);
    		vTaskDelay(10);
    		sprintf(str,"OK");
    		SSD1306_Puts(str, &Font_7x10, 1);
		}else{
			SSD1306_GotoXY(0,50);
    		vTaskDelay(10);
    		sprintf(str,"  ");
    		SSD1306_Puts(str, &Font_7x10, 1);

			SSD1306_GotoXY(50,50);
    		vTaskDelay(10);
    		sprintf(str,"NOK");
    		SSD1306_Puts(str, &Font_7x10, 1);
		}

        SSD1306_UpdateScreen();
             
        ESP_LOGI("vTask_SSD1306","Devolve o semáforo\n");
        xSemaphoreGive(xconexaoMQTTSemaphore);

        vTaskDelay(100/portTICK_PERIOD_MS);
    }  
}

void vTask_INC_DEC (void *pvParameters)
{   
    (void)pvParameters;

    //uint32_t inc_dec_value = 0;
	uint8_t debouncingTime1 = 0;
	uint8_t debouncingTime2 = 0;

	gpio_pad_select_gpio(GPIO_INCREMENTA);
    gpio_set_direction (GPIO_INCREMENTA, GPIO_MODE_INPUT);
	gpio_set_pull_mode(GPIO_INCREMENTA, GPIO_PULLUP_ONLY);
	
	gpio_pad_select_gpio(GPIO_DECREMENTA);
    gpio_set_direction (GPIO_DECREMENTA, GPIO_MODE_INPUT);
	gpio_set_pull_mode(GPIO_DECREMENTA, GPIO_PULLUP_ONLY);

    while (1)
    {
		if(gpio_get_level(GPIO_INCREMENTA) == 0)
		{
			debouncingTime1++;
			//ESP_LOGI("vTask_INC_DEC", "debouncingTime1 %d\n", debouncingTime1);
      		if(debouncingTime1 >= 2)
      		{
          		inc_dec_value++;
				//ESP_LOGI("vTask_INC_DEC", "Valor desejado %d\n", inc_dec_value);
      		}
    	}	
    	else{
      		debouncingTime1 = 0;
    	}

		if(gpio_get_level(GPIO_DECREMENTA) == 0)
		{
      		debouncingTime2++;
      		if(debouncingTime2 >= 2)
      		{
          		inc_dec_value--;
      		}
    	}	
    	else{
      		debouncingTime2 = 0;
    	}

		xQueueSend(xQueue_INC_DEC_to_SSD1306, &inc_dec_value, portMAX_DELAY);
		
		printf("Valor desejado %d\n", inc_dec_value);

        vTaskDelay(1/portTICK_PERIOD_MS);
    }  
}


void conectadoWifi(void *pvParameters)
{
	 while(1)
	{
		if(xSemaphoreTake(xconexaoWifiSemaphore, portMAX_DELAY))
		{
			ESP_LOGI(TAG, "Entrou no MQTT");
			mqtt_start();
		}
		vTaskDelay(1000/portTICK_PERIOD_MS);
	}
}

void trataComunicacaoComServidor(void *pvParameters)
{
	char mensagem[50];
	if(xSemaphoreTake(xconexaoMQTTSemaphore, portMAX_DELAY))
	{
		while(1)
		{
			sprintf(mensagem, "Distancia: %d", distancia);
			mqtt_envia_mensagem("trena/distancia", mensagem);
			
			sprintf(mensagem, "Valor desejado: %d", inc_dec_value);
			mqtt_envia_mensagem("trena/distancia", mensagem);

			xSemaphoreGive(xconexaoMQTTSemaphore);
			ESP_LOGI(TAG, "Teste");
			vTaskDelay(1000/portTICK_PERIOD_MS);
		}
	}
}