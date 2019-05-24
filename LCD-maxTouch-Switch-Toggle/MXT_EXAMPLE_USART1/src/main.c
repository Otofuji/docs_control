/**
 * \file
 *
 * \brief Example of usage of the maXTouch component with USART
 *
 * This example shows how to receive touch data from a maXTouch device
 * using the maXTouch component, and display them in a terminal window by using
 * the USART driver.
 *
 * Copyright (c) 2014-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */

/**
 * \mainpage
 *
 * \section intro Introduction
 * This simple example reads data from the maXTouch device and sends it over
 * USART as ASCII formatted text.
 *
 * \section files Main files:
 * - example_usart.c: maXTouch component USART example file
 * - conf_mxt.h: configuration of the maXTouch component
 * - conf_board.h: configuration of board
 * - conf_clock.h: configuration of system clock
 * - conf_example.h: configuration of example
 * - conf_sleepmgr.h: configuration of sleep manager
 * - conf_twim.h: configuration of TWI driver
 * - conf_usart_serial.h: configuration of USART driver
 *
 * \section apiinfo maXTouch low level component API
 * The maXTouch component API can be found \ref mxt_group "here".
 *
 * \section deviceinfo Device Info
 * All UC3 and Xmega devices with a TWI module can be used with this component
 *
 * \section exampledescription Description of the example
 * This example will read data from the connected maXTouch explained board
 * over TWI. This data is then processed and sent over a USART data line
 * to the board controller. The board controller will create a USB CDC class
 * object on the host computer and repeat the incoming USART data from the
 * main controller to the host. On the host this object should appear as a
 * serial port object (COMx on windows, /dev/ttyxxx on your chosen Linux flavour).
 *
 * Connect a terminal application to the serial port object with the settings
 * Baud: 57600
 * Data bits: 8-bit
 * Stop bits: 1 bit
 * Parity: None
 *
 * \section compinfo Compilation Info
 * This software was written for the GNU GCC and IAR for AVR.
 * Other compilers may or may not work.
 *
 * \section contactinfo Contact Information
 * For further information, visit
 * <A href="http://www.atmel.com/">Atmel</A>.\n
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#include <asf.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "conf_board.h"
#include "conf_example.h"
#include "conf_uart_serial.h"
#include "Icons/TELA_Prancheta 1.h"
#include "tfont.h"
#include "digital521.h"
#include "arial_72.h"



#define MAX_ENTRIES        3
#define STRING_LENGTH     70

#define USART_TX_MAX_LENGTH     0xff

#define AFEC_CHANNEL_TEMP_SENSOR 8
#define AFEC1_CHANNEL_TEMP_SENSOR 5

/** Reference voltage for AFEC,in mv. */
#define VOLT_REF        (3300)

/** The maximal digital value */
/** 2^12 - 1                  */
#define MAX_DIGITAL (4095)

#define LED_PIO					PIOC
#define LED_PIO_ID				ID_PIOC
#define LED_PIO_IDX				8u
#define LED_PIO_IDX_MASK		(1u<<LED_PIO_IDX)


struct ili9488_opt_t g_ili9488_display_opt;
const uint32_t BUTTON_W = 120;
const uint32_t BUTTON_H = 150;
const uint32_t BUTTON_BORDER = 2;
const uint32_t BUTTON_X = ILI9488_LCD_WIDTH/2;
const uint32_t BUTTON_Y = ILI9488_LCD_HEIGHT/2;

volatile Bool touched = false;
volatile Bool tc0_handle = false;
volatile uint32_t g_ul_value,g1_ul_value;
volatile Bool g_is_conversion_done,g1_conversion_done;

signed long SmoothDataINT;
signed long SmoothDataFP;
int Beta = 4; // Length of the filter < 16

signed long SmoothDataINT1;
signed long SmoothDataFP1;
int Beta1 = 4; // Length of the filter < 16
Bool isOn = false;
Bool lastBlu=false;


#ifdef DEBUG_SERIAL
#define UART_COMM USART1
#else
#define UART_COMM USART0
#endif

volatile long g_systimer = 0;

void SysTick_Handler() {
	g_systimer++;
}

void config_console(void) {
	usart_serial_options_t config;
	config.baudrate = 9600;
	config.charlength = US_MR_CHRL_8_BIT;
	config.paritytype = US_MR_PAR_NO;
	config.stopbits = false;
	usart_serial_init(USART1, &config);
	usart_enable_tx(USART1);
	usart_enable_rx(USART1);
}

void usart_put_string(Usart *usart, char str[]) {
	usart_serial_write_packet(usart, str, strlen(str));
}


int usart_get_string(Usart *usart, char buffer[], int bufferlen, int timeout_ms) {
	long timestart = g_systimer;
	uint32_t rx;
	uint32_t counter = 0;
	
	while(g_systimer - timestart < timeout_ms && counter < bufferlen - 1) {
		if(usart_read(usart, &rx) == 0) {
			//timestart = g_systimer; // reset timeout
			buffer[counter++] = rx;
		}
	}
	buffer[counter] = 0x00;
	return counter;
}

void usart_send_command(Usart *usart, char buffer_rx[], int bufferlen, char buffer_tx[], int timeout) {
	pio_clear(LED_PIO,LED_PIO_IDX_MASK);
	usart_put_string(usart, buffer_tx);
	usart_get_string(usart, buffer_rx, bufferlen, timeout);
	pio_set(LED_PIO,LED_PIO_IDX_MASK);
}

void usart_log(char* name, char* log) {
	usart_put_string(USART1, "[");
	usart_put_string(USART1, name);
	usart_put_string(USART1, "] ");
	usart_put_string(USART1, log);
	usart_put_string(USART1, "\r\n");
}

void hc05_config_server(void) {
	usart_serial_options_t config;
	config.baudrate = 9600;
	config.charlength = US_MR_CHRL_8_BIT;
	config.paritytype = US_MR_PAR_NO;
	config.stopbits = false;
	usart_serial_init(USART0, &config);
	usart_enable_tx(USART0);
	usart_enable_rx(USART0);
	
	// RX - PB0  TX - PB1
	pio_configure(PIOB, PIO_PERIPH_C, (1 << 0), PIO_DEFAULT);
	pio_configure(PIOB, PIO_PERIPH_C, (1 << 1), PIO_DEFAULT);
}

void hc05_disable_server(void) {
	isOn=false;
}

void hc05_en_server(void) {
	isOn = true;
}



static void AFEC_Temp_callback(void)
{
	g_ul_value = afec_channel_get_value(AFEC0, AFEC_CHANNEL_TEMP_SENSOR);
	g_is_conversion_done = true;
	
}
static void AFEC_Temp_callback1(void)
{
	g1_ul_value = afec_channel_get_value(AFEC1, AFEC1_CHANNEL_TEMP_SENSOR);
	g1_conversion_done = true;
	
}

int hc05_server_init(void) {
	char buffer_rx[128];
	usart_send_command(USART0, buffer_rx, 1000, "AT", 1000);
	usart_send_command(USART0, buffer_rx, 1000, "AT", 1000);
	usart_send_command(USART0, buffer_rx, 1000, "AT+NAMEmanchester", 1000);
	usart_log("hc05_server_init", buffer_rx);
	usart_send_command(USART0, buffer_rx, 1000, "AT", 1000);
	usart_send_command(USART0, buffer_rx, 1000, "AT+PIN0000", 1000);
	usart_log("hc05_server_init", buffer_rx);
}

void TC1_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrup??o foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	tc0_handle = true;
	pmc_disable_periph_clk(TC0);
}

	
static void configure_lcd(void){
	/* Initialize display parameter */
	g_ili9488_display_opt.ul_width = ILI9488_LCD_WIDTH;
	g_ili9488_display_opt.ul_height = ILI9488_LCD_HEIGHT;
	g_ili9488_display_opt.foreground_color = COLOR_CONVERT(COLOR_WHITE);
	g_ili9488_display_opt.background_color = COLOR_CONVERT(COLOR_WHITE);

	/* Initialize LCD */
	ili9488_init(&g_ili9488_display_opt);
}
void font_draw_text(tFont *font, const char *text, int x, int y, int spacing) {
	char *p = text;
	while(*p != NULL) {
		char letter = *p;
		int letter_offset = letter - font->start_char;
		if(letter <= font->end_char) {
			tChar *current_char = font->chars + letter_offset;
			ili9488_draw_pixmap(x, y, current_char->image->width, current_char->image->height, current_char->image->data);
			x += current_char->image->width + spacing;
		}
		p++;
	}
}

void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	uint32_t channel = 1;

	/* Configura o PMC */
	/* O TimerCounter ? meio confuso
	o uC possui 3 TCs, cada TC possui 3 canais
	TC0 : ID_TC0, ID_TC1, ID_TC2
	TC1 : ID_TC3, ID_TC4, ID_TC5
	TC2 : ID_TC6, ID_TC7, ID_TC8
	*/
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  4Mhz e interrup?c?o no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura e ativa interrup?c?o no TC canal 0 */
	/* Interrup??o no C */
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);

	/* Inicializa o canal 0 do TC */
	tc_start(TC, TC_CHANNEL);
}

/**
 * \brief Set maXTouch configuration
 *
 * This function writes a set of predefined, optimal maXTouch configuration data
 * to the maXTouch Xplained Pro.
 *
 * \param device Pointer to mxt_device struct
 */
static void mxt_init(struct mxt_device *device)
{
	enum status_code status;

	/* T8 configuration object data */
	uint8_t t8_object[] = {
		0x0d, 0x00, 0x05, 0x0a, 0x4b, 0x00, 0x00,
		0x00, 0x32, 0x19
	};

	/* T9 configuration object data */
	uint8_t t9_object[] = {
		0x8B, 0x00, 0x00, 0x0E, 0x08, 0x00, 0x80,
		0x32, 0x05, 0x02, 0x0A, 0x03, 0x03, 0x20,
		0x02, 0x0F, 0x0F, 0x0A, 0x00, 0x00, 0x00,
		0x00, 0x18, 0x18, 0x20, 0x20, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x02,
		0x02
	};

	/* T46 configuration object data */
	uint8_t t46_object[] = {
		0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x03,
		0x00, 0x00
	};
	
	/* T56 configuration object data */
	uint8_t t56_object[] = {
		0x02, 0x00, 0x01, 0x18, 0x1E, 0x1E, 0x1E,
		0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E,
		0x1E, 0x1E, 0x1E, 0x1E, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00
	};

	/* TWI configuration */
	twihs_master_options_t twi_opt = {
		.speed = MXT_TWI_SPEED,
		.chip  = MAXTOUCH_TWI_ADDRESS,
	};

	status = (enum status_code)twihs_master_setup(MAXTOUCH_TWI_INTERFACE, &twi_opt);
	Assert(status == STATUS_OK);

	/* Initialize the maXTouch device */
	status = mxt_init_device(device, MAXTOUCH_TWI_INTERFACE,
			MAXTOUCH_TWI_ADDRESS, MAXTOUCH_XPRO_CHG_PIO);
	Assert(status == STATUS_OK);

	/* Issue soft reset of maXTouch device by writing a non-zero value to
	 * the reset register */
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_COMMANDPROCESSOR_T6, 0)
			+ MXT_GEN_COMMANDPROCESSOR_RESET, 0x01);

	/* Wait for the reset of the device to complete */
	delay_ms(MXT_RESET_TIME);

	/* Write data to configuration registers in T7 configuration object */
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_POWERCONFIG_T7, 0) + 0, 0x20);
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_POWERCONFIG_T7, 0) + 1, 0x10);
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_POWERCONFIG_T7, 0) + 2, 0x4b);
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_POWERCONFIG_T7, 0) + 3, 0x84);

	/* Write predefined configuration data to configuration objects */
	mxt_write_config_object(device, mxt_get_object_address(device,
			MXT_GEN_ACQUISITIONCONFIG_T8, 0), &t8_object);
	mxt_write_config_object(device, mxt_get_object_address(device,
			MXT_TOUCH_MULTITOUCHSCREEN_T9, 0), &t9_object);
	mxt_write_config_object(device, mxt_get_object_address(device,
			MXT_SPT_CTE_CONFIGURATION_T46, 0), &t46_object);
	mxt_write_config_object(device, mxt_get_object_address(device,
			MXT_PROCI_SHIELDLESS_T56, 0), &t56_object);

	/* Issue recalibration command to maXTouch device by writing a non-zero
	 * value to the calibrate register */
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_COMMANDPROCESSOR_T6, 0)
			+ MXT_GEN_COMMANDPROCESSOR_CALIBRATE, 0x01);
}

void draw_screen(void) {
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_BLACK));
	ili9488_draw_filled_rectangle(0, 0, ILI9488_LCD_WIDTH-1, ILI9488_LCD_HEIGHT-1);
}

void draw_button(uint32_t clicked) {
	static uint32_t last_state = 255; // undefined
	if(clicked == last_state) return;
	
	ili9488_draw_pixmap(0,10,TELA_Prancheta1_correta.width,TELA_Prancheta1_correta.height,TELA_Prancheta1_correta.data);
	
	last_state = clicked;
}

uint32_t convert_axis_system_x(uint32_t touch_y) {
	// entrada: 4096 - 0 (sistema de coordenadas atual)
	// saida: 0 - 320
	return ILI9488_LCD_WIDTH - ILI9488_LCD_WIDTH*touch_y/4096;
}

uint32_t convert_axis_system_y(uint32_t touch_x) {
	// entrada: 0 - 4096 (sistema de coordenadas atual)
	// saida: 0 - 320
	return ILI9488_LCD_HEIGHT*touch_x/4096;
}

static int32_t convert_adc_to_temp(int32_t ADC_value){
	
	

	int32_t ul_vol;
	int FP_Shift = 5; 
	ul_vol = (ADC_value) * VOLT_REF / (float) MAX_DIGITAL;
	ul_vol <<= FP_Shift;
	SmoothDataFP = (SmoothDataFP<< Beta)-SmoothDataFP;
	SmoothDataFP += ul_vol;
	SmoothDataFP >>= Beta;
	SmoothDataINT = SmoothDataFP>> FP_Shift;
	
	
	return(SmoothDataINT);
}
static int32_t convert_adc_to_temp1(int32_t ADC_value){
	int32_t ul_vol;
	int FP_Shift = 5; 
	ul_vol = ADC_value * VOLT_REF / (float) MAX_DIGITAL;
	ul_vol <<= FP_Shift;
	SmoothDataFP1 = (SmoothDataFP1<< Beta1)-SmoothDataFP1;
	SmoothDataFP1 += ul_vol;
	SmoothDataFP1 >>= Beta1;
	SmoothDataINT1 = SmoothDataFP1>> FP_Shift;
	
	
	return(SmoothDataINT1*100/(3295-30));
}

static void config_ADC_TEMP(void){
	/*************************************
	* Ativa e configura AFEC
	*************************************/
	/* Ativa AFEC - 0 */
	afec_enable(AFEC0);
	afec_enable(AFEC1);

	/* struct de configuracao do AFEC */
	struct afec_config afec_cfg;
	struct afec_config afec_cfg1;

	/* Carrega parametros padrao */
	afec_get_config_defaults(&afec_cfg);
	afec_get_config_defaults(&afec_cfg1);
	

	/* Configura AFEC */
	afec_init(AFEC0, &afec_cfg);
	afec_init(AFEC1, &afec_cfg1);

	/* Configura trigger por software */
	afec_set_trigger(AFEC0, AFEC_TRIG_SW);
	afec_set_trigger(AFEC1, AFEC_TRIG_SW);

	/* configura call back */
	afec_set_callback(AFEC0, AFEC_INTERRUPT_EOC_8,	AFEC_Temp_callback, 3);
	afec_set_callback(AFEC1, AFEC_INTERRUPT_EOC_5,	AFEC_Temp_callback1, 2);

	/*** Configuracao espec?fica do canal AFEC ***/
	struct afec_ch_config afec_ch_cfg;
	struct afec_ch_config afec_ch_cfg1;
	afec_ch_get_config_defaults(&afec_ch_cfg);
	afec_ch_get_config_defaults(&afec_ch_cfg1);
	afec_ch_cfg.gain = AFEC_GAINVALUE_0;
	afec_ch_cfg1.gain = AFEC_GAINVALUE_0;
	afec_ch_set_config(AFEC0, AFEC_CHANNEL_TEMP_SENSOR, &afec_ch_cfg);
	afec_ch_set_config(AFEC1, AFEC1_CHANNEL_TEMP_SENSOR, &afec_ch_cfg1);

	/*
	* Calibracao:
	* Because the internal ADC offset is 0x200, it should cancel it and shift
	down to 0.
	*/
	afec_channel_set_analog_offset(AFEC0, AFEC_CHANNEL_TEMP_SENSOR, 0x200);
	afec_channel_set_analog_offset(AFEC1, AFEC1_CHANNEL_TEMP_SENSOR, 0x200);

	/***  Configura sensor de temperatura ***/
	struct afec_temp_sensor_config afec_temp_sensor_cfg;
	struct afec_temp_sensor_config afec_temp_sensor_cfg1;
	
	afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
	afec_temp_sensor_set_config(AFEC0, &afec_temp_sensor_cfg);
	afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg1);
	afec_temp_sensor_set_config(AFEC1, &afec_temp_sensor_cfg1);
	
	/* Selecina canal e inicializa convers?o */
	afec_channel_enable(AFEC0, AFEC_CHANNEL_TEMP_SENSOR);
	afec_channel_enable(AFEC1, AFEC1_CHANNEL_TEMP_SENSOR);
}
void config_io(void){
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_configure(LED_PIO, PIO_OUTPUT_0, LED_PIO_IDX_MASK, PIO_DEFAULT);
}

void update_screen(uint32_t tx, uint32_t ty) {
	while(!usart_is_tx_ready(UART_COMM));
	if(ty >= 30 && ty <= 130) {
		if(tx >= 20 && tx < 120) {
			//printf("\nLista Numerada\n");
			usart_write(UART_COMM, '1');
			} else if(tx > 120 && tx <= 210) {
			usart_write(UART_COMM, '2');
			//printf("\nLista de tópicos\n");
			}else if(tx > 210 && tx <= 320){
			usart_write(UART_COMM, '3');
			//printf("\nAdicionar Comentario\n");
		}
	}
	
	else if(ty > 130 && ty <= 240) {
		if(tx >= 20 && tx < 120) {
			usart_write(UART_COMM, '4');
			//printf("\nNota de Rodapé\n");
			} else if(tx > 120 && tx <= 210) {
			usart_write(UART_COMM, '5');
			//printf("\nHist. de versão\n");
			}else if(tx > 210 && tx <= 320){
			usart_write(UART_COMM, '6');
			//printf("\nDitado\n");
		}
	}
	
	else if(ty > 240 && ty <= 350) {
		if(tx >= 20 && tx < 120) {
			usart_write(UART_COMM, '7');
			//printf("\nEquação\n");
			} else if(tx > 120 && tx <= 210) {
			usart_write(UART_COMM, '8');
			//printf("\nDownload PDF\n");
			}else if(tx > 210 && tx <= 320){
			usart_write(UART_COMM, '9');
			//printf("\nE-mail a colaboradores\n");
		}
	}
	else if(ty > 350 && ty <= 480) {
		if(tx >= 20 && tx < 120) {
			usart_write(UART_COMM, 'a');
			//printf("\nInserir Imagem\n");
			} else if(tx > 120 && tx <= 210) {
			usart_write(UART_COMM, 'b');
			//printf("\nAnexo no e-mail\n");
			}else if(tx > 210 && tx <= 320){
			usart_write(UART_COMM, 'c');
			//printf("\nPublicar na WEB\n");
		}
	}
}

void mxt_handler(struct mxt_device *device)
{
	/* USART tx buffer initialized to 0 */
	char tx_buf[STRING_LENGTH * MAX_ENTRIES] = {0};
	uint8_t i = 0; /* Iterator */

	/* Temporary touch event data struct */
	struct mxt_touch_event touch_event;

	/* Collect touch events and put the data in a string,
	 * maximum 2 events at the time */
	do {
		/* Temporary buffer for each new touch event line */
		char buf[STRING_LENGTH];
	
		/* Read next next touch event in the queue, discard if read fails */
		if (mxt_read_touch_event(device, &touch_event) != STATUS_OK) {
			continue;
		}
		
		 // eixos trocados (quando na vertical LCD)
		uint32_t conv_x = convert_axis_system_x(touch_event.y);
		uint32_t conv_y = convert_axis_system_y(touch_event.x);
		
		
		if(touched==false){
			touched = true;
			pmc_enable_periph_clk(TC0);
			if(lastBlu){
			update_screen(conv_x, conv_y);
			}
		}
		/* Add the new string to the string buffer */
		strcat(tx_buf, buf);
		i++;

		/* Check if there is still messages in the queue and
		 * if we have reached the maximum numbers of events */
	} while ((mxt_is_message_pending(device)) & (i < MAX_ENTRIES));

	/* If there is any entries in the buffer, send them over USART */
	if (i > 0) {
//		usart_serial_write_packet(USART_SERIAL_EXAMPLE, (uint8_t *)tx_buf, strlen(tx_buf));
	}
}

int main(void)
{
	struct mxt_device device; /* Device data container */

	/* Initialize the USART configuration struct */
	/*const usart_serial_options_t usart_serial_options = {
		.baudrate     = USART_SERIAL_EXAMPLE_BAUDRATE,
		.charlength   = USART_SERIAL_CHAR_LENGTH,
		.paritytype   = USART_SERIAL_PARITY,
		.stopbits     = USART_SERIAL_STOP_BIT
	};*/

	sysclk_init(); /* Initialize system clocks */
	board_init();  /* Initialize board */
	configure_lcd();
	
	SysTick_Config(sysclk_get_cpu_hz() / 1000); // 1 ms
	config_console();
	config_io();
	
	#ifndef DEBUG_SERIAL
	usart_put_string(USART1, "Inicializando...\r\n");
	usart_put_string(USART1, "Config HC05 Server...\r\n");
	hc05_config_server();
	hc05_server_init();
	#endif
	
	char button1 = '0';
	char eof = 'X';
	char buffer[1024];
	
	
	
	draw_screen();
	draw_button(0);
	TC_init(TC0, ID_TC1, 1, 1);
	/* Initialize the mXT touch device */
	mxt_init(&device);
	config_ADC_TEMP();
	/* Initialize stdio on USART */
	//stdio_serial_init(USART_SERIAL_EXAMPLE, &usart_serial_options);

	//printf("\n\rmaXTouch data USART transmitter\n\r");
	afec_start_software_conversion(AFEC0);
	afec_start_software_conversion(AFEC1);

	while (true) {
		/* Check for any pending messages and run message handler if any
		 * message is found in the queue */
		
		if (mxt_is_message_pending(&device)) {
			mxt_handler(&device);
			
			while(!usart_is_tx_ready(UART_COMM));
			usart_write(UART_COMM, eof);
			delay_ms(200);
		}
		else{
			while(!usart_is_tx_ready(UART_COMM));
			usart_write(UART_COMM, button1);
			while(!usart_is_tx_ready(UART_COMM));
			usart_write(UART_COMM, eof);
		}
		
		if(tc0_handle){
			tc0_handle=false;
			touched = false;
		}
		if(g_is_conversion_done){
			g_is_conversion_done = false;
			//Bateria
			uint32_t cent = convert_adc_to_temp(g_ul_value);
			char temps[64];
			sprintf(&temps,"%3d %%",cent*100/(3291-25));
			font_draw_text(&digital52,temps,0,0,0.1);
			afec_start_software_conversion(AFEC0);
		}
		
		if(g1_conversion_done){
			// Controle Bluetooth
			g1_conversion_done=false;
			uint32_t cents = convert_adc_to_temp1(g1_ul_value);		
			if(cents<=50 && !lastBlu){
				lastBlu=true;
				ili9488_display_on();
				font_draw_text(&digital52,"  Wait  ",120,0,0);
				char buffer_x[128];
				usart_send_command(USART0,buffer_x,1000,"AT",1000)	;
				usart_send_command(USART0,buffer_x,1000,"AT",1000)	;
				usart_send_command(USART0,buffer_x,1000,"AT+RESET",1000);
				usart_send_command(USART0,buffer_x,1000,"AT+EXSNIFF",1000);
				font_draw_text(&digital52,"      On",120,0,0);
			}else if(cents>50 && lastBlu){
				lastBlu=false;
				font_draw_text(&digital52,"     Off",120,0,0);	
				char buffer_x[128];
				usart_send_command(USART0,buffer_x,1000,"AT",1000)	;
				usart_send_command(USART0,buffer_x,1000,"AT",1000)	;
				usart_send_command(USART0,buffer_x,1000,"AT+DISC",1000);
				usart_send_command(USART0,buffer_x,1000,"AT+RMAAD",1000);
				usart_send_command(USART0,buffer_x,1000,"AT+ENSNIFF",1000);
				ili9488_display_off();

			}
			afec_start_software_conversion(AFEC1);
			
		}
		
	}

	return 0;
}
