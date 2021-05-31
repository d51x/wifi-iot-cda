static const char* UTAG = "USR";
#define FW_VER "3.14"


/*
Количество настроек
Kotel1 gpio, Kotel2 gpio, Pump1 gpio, Pump2 gpio, ESC gpio, Vent gpio, Night(h), Day(h), BacklightTDelay, Kotel1LED, Kotel2LED, KotelWorkLed, PumpWorkLed, ScheduleLed, VentLed, GlobalTempSet

*/

#define TEMPSET_STEP 1
#define HYST_STEP 1
#define TEMPSET_MIN 100
#define TEMPSET_MAX 300
#define HYST_MIN 1
#define HYST_MAX 50

#define KOTEL1_GPIO sensors_param.cfgdes[0] //208
#define KOTEL2_GPIO sensors_param.cfgdes[1] //209
#define PUMP1_GPIO  sensors_param.cfgdes[2] //210
#define PUMP2_GPIO  sensors_param.cfgdes[3] //211
#define ESC_GPIO    sensors_param.cfgdes[4] //212
#define VENT_GPIO   sensors_param.cfgdes[5] //213

#define NIGHT_TIME  sensors_param.cfgdes[6] //23
#define DAY_TIME    sensors_param.cfgdes[7] //7

#define BACKLIGHT_TIMEOUT    sensors_param.cfgdes[8] // 30


#define KOTEL1_LED_GPIO sensors_param.cfgdes[9]
#define KOTEL2_LED_GPIO sensors_param.cfgdes[10]
#define KOTEL_LED_GPIO sensors_param.cfgdes[11]
#define PUMP_LED_GPIO sensors_param.cfgdes[12]
#define SCHEDULE_LED_GPIO sensors_param.cfgdes[13]
#define VENT_LED_GPIO sensors_param.cfgdes[14]

#define TEMPSET sensors_param.cfgdes[15]

#define current_temp        valdes[0]  // устанавливать через интерпретер или mqtt - valdes[0]
#define street_temp         valdes[1]  // устанавливать через интерпретер или mqtt - valdes[0]
#define work_mode           valdes[2]
#define schedule            valdes[3]

#define VALDES_INDEX_WORK_MODE                  2   //  
#define VALDES_INDEX_SCHEDULE                  3   //  

#define flow_temp data1wire[0]
#define return_temp data1wire[1]

#define KOTEL1_GPIO_DEFAULT 208
#define KOTEL2_GPIO_DEFAULT 209
#define PUMP1_GPIO_DEFAULT 210
#define PUMP2_GPIO_DEFAULT 211
#define ESC_GPIO_DEFAULT 212
#define VENT_GPIO_DEFAULT 213

#define BACKLIGHT_TIMEOUT_DEFAULT 30 //sec
#define NIGHT_TIME_DEFAULT 23
#define DAY_TIME_DEFAULT 7

#define BACKLIGHT_GPIO 199

#define MCP23017_INTA_PIN 4     // pin esp

#define B(bit_no)         (1 << (bit_no))
#define BIT_CLEAR(reg, bit_no)   (reg) &= ~B(bit_no)
#define BIT_SET(reg, bit_no)   (reg) |= B(bit_no)
#define BIT_CHECK(reg, bit_no)   ( (reg) & B(bit_no) )
#define BIT_TRIGGER(reg, bit_no)   (reg) ^= B(bit_no)


#if lcde
    #define LCD_BACKLIGHT_STATE BIT_CHECK(sensors_param.lcdled,0)
#else
    #define LCD_BACKLIGHT_STATE 1
#endif

#define THERMO_STATE(x)		BIT_CHECK(sensors_param.thermo[x-1][0],0)
#define THERMO_ON(x)  { if ( GPIO_ALL_GET(x+99) == 0 ) GPIO_ALL(99+x,1);}
#define THERMO_OFF(x) { if ( GPIO_ALL_GET(99+x) == 1 ) GPIO_ALL(99+x,0);}


#define THERMO_SETPOINT(x)      sensors_param.thermzn[x-1][0]
#define THERMO_HYSTERESIS(x)	sensors_param.thermzn[x-1][1]

#define THERMO_TEMP_SET(x,y)  { sensors_param.thermzn[x-1][0] = y; SAVEOPT; }
#define THERMO_HYST_SET(x,y)  { sensors_param.thermzn[x-1][1] = y; SAVEOPT; }

#define SYMBOL_DEGREE 0x01
#define SYMBOL_ARROW_RIGHT 126
#define SYMBOL_ARROW_LEFT 127

// NVSCURRENT_TEMP
#define SPACE_NAME "d51x"
#define WORK_MODE_PARAM "workmode"
#define SCHEDULE_PARAM "schedule"

#define MCP23017_GPIO0   1 << 0     //0x0001
#define MCP23017_GPIO1   1 << 1     //0x0002
#define MCP23017_GPIO2   1 << 2     //0x0004
#define MCP23017_GPIO3   1 << 3     //0x0008
#define MCP23017_GPIO4   1 << 4     //0x0010
#define MCP23017_GPIO5   1 << 5     //0x0020
#define MCP23017_GPIO6   1 << 6     //0x0040
#define MCP23017_GPIO7   1 << 7     //0x0080
#define MCP23017_GPIO8   1 << 8     //0x0100
#define MCP23017_GPIO9   1 << 9     //0x0200
#define MCP23017_GPIO10  1 << 10    //0x0400
#define MCP23017_GPIO11  1 << 11    //0x0800
#define MCP23017_GPIO12  1 << 12    //0x1000
#define MCP23017_GPIO13  1 << 13    //0x2000
#define MCP23017_GPIO14  1 << 14    //0x4000
#define MCP23017_GPIO15  1 << 15    //0x8000

#define IODIRA      0x00    // регистр, указыващий направления портов output/input
#define IODIRB      0x01
#define IPOLA       0x02    // инверсия ног
#define IPOLB       0x03
#define GPINTENA    0x04    // прерывания на ногах
#define GPINTENB    0x05
#define DEFVALA     0x06    // дефолтные значения ног, прерывание сработает, если на ноге сигнал отличается от дефолтного
#define DEFVALB     0x07
#define INTCONA     0x08    // условия сработки прерывания на ногах
#define INTCONB     0x09
#define IOCONA      0x0A    // конфигурационный регистр
#define IOCONB      0x0B
#define GPPUA       0x0C    // подтяжка ног 100к
#define GPPUB       0x0D
#define INTFA       0x0E    // регистр флагов прерываний, покажет на какой ноге было прерывание
#define INTFB       0x0F
#define INTCAPA     0x10    // покажет что было на ноге в момент прерывания на этой ноге
#define INTCAPB     0x11
#define GPIOA       0x12    // состояние ног, когда было прерывание на ноге может уже быть другое значение и надо читать INTCAP, если работаем с прерываниями
#define GPIOB       0x13
#define OLATA       0x14    
#define OLATB       0x15



#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c" 
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0') 

#define MCP23017_ISR_DELAY_MS 60

#define millis() (unsigned long) (esp_timer_get_time() / 1000ULL)

TaskHandle_t mcp23017_task;
QueueHandle_t mcp23017_queue;

TimerHandle_t  backlight_timer;

typedef void (*interrupt_cb)(void *arg, uint8_t *state);

typedef struct mcp23017_pin_isr {
        uint8_t pin;
        interrupt_cb pin_cb;
        void *args;

        interrupt_cb pin_cb2;
        void *args2;

        gpio_int_type_t intr_type;
        
        uint32_t up_delay_ms;
        uint32_t ts_down;
        uint32_t ts_up;

} mcp23017_pin_isr_t;

mcp23017_pin_isr_t *pin_isr; // указатель на массив коллбеков для пинов
uint8_t pin_isr_cnt;


typedef enum {
    MENU_PAGE_MAIN,
    MENU_PAGE_TEMPSET,
    //MENU_PAGE_HYST,
    MENU_PAGE_MAX
} menu_e;

menu_e menu_idx = MENU_PAGE_MAIN;

typedef enum {
    MODE_MANUAL,
    MODE_AUTO,
    MODE_KOTEL1,
    MODE_KOTEL2,
    MODE_MAX
} mode_e;

//mode_e work_mode = MODE_MANUAL; // TODO: переделать на valdes[2]

typedef enum {
    KOTEL_NONE,
    KOTEL_1,
    KOTEL_2
} active_kotel_e;

active_kotel_e active_kotel = KOTEL_NONE;

//uint16_t schedule = 0;      // TODO: переделать на valdes[3]

//uint16_t tempset = 240;
uint16_t shed_tempset = 0;

#define WORKMODE    work_mode
#define ADDLISTSENS {200,LSENSFL0,"WorkMode","workmode",&WORKMODE,NULL}, \
                    {201,LSENSFL1,"Temperature","temp",&current_temp,NULL}, \
                    {202,LSENSFL0,"Schedule","schedule",&schedule,NULL}, \
                    {203,LSENSFL0,"TempSet","tempset",&TEMPSET,NULL},


uint8_t display_error = 0;
TimerHandle_t  show_error_timer;
#define SHOW_ERROR_TIMEOUT 5000

uint32_t last_key_press = 0;
#define MENU_EXIT_TIMEOUT 10000 // 10 sec


esp_err_t nvs_param_save(const char* space_name, const char* key, void *param, uint16_t len)
{
    esp_err_t ret = ESP_ERR_INVALID_ARG;
    nvs_handle my_handle;
    ret = nvs_open(space_name, NVS_READWRITE, &my_handle);
    ret = nvs_set_blob(my_handle, key, param, len);
    ret = nvs_commit(my_handle);

SAVE_FINISH:
    nvs_close(my_handle);

OPEN_FAIL:
    return ret;
}

esp_err_t nvs_param_load(const char* space_name, const char* key, void* dest)
{
    esp_err_t ret = ESP_ERR_INVALID_ARG;
    nvs_handle my_handle;
    size_t required_size = 0;
    ret = nvs_open(space_name, NVS_READWRITE, &my_handle);
    ret = nvs_get_blob(my_handle, key, NULL, &required_size);
    if (required_size == 0) {
        ESP_LOGW(TAG, "the target you want to load has never been saved");
        ret = ESP_FAIL;
        goto LOAD_FINISH;
    }
    ret = nvs_get_blob(my_handle, key, dest, &required_size);

LOAD_FINISH:
    nvs_close(my_handle);

OPEN_FAIL:
    return ret;
}

static void IRAM_ATTR mcp23017_isr_handler(void *arg) {
    portBASE_TYPE HPTaskAwoken = pdFALSE;
    BaseType_t xHigherPriorityTaskWoken;

    uint16_t data[2];

    data[0] = MCPread_reg16(0, INTFA); // считываем данные с mcp23017
    data[1] = MCPread_reg16(0, INTCAPA); // считываем данные с mcp23017 // чтение снимка ножек при прерывании сбрасывает прерывание

    static uint32_t t = 0;

    if ( millis() - t >= MCP23017_ISR_DELAY_MS )
    {
        xQueueOverwriteFromISR(mcp23017_queue, &data, &xHigherPriorityTaskWoken);
        t = millis();
    }
    portEND_SWITCHING_ISR( HPTaskAwoken == pdTRUE );
}

static void mcp23017_isr_cb(void *arg) {
	while (1) {

            uint16_t data[2];
            if ( xQueueReceive(mcp23017_queue, &data, 0) == pdPASS) 
            {
                //ESP_LOGI(UTAG, " interrput: %4d \t 0x%04X \t " BYTE_TO_BINARY_PATTERN BYTE_TO_BINARY_PATTERN, data[0], data[0], BYTE_TO_BINARY(data[0] >> 8), BYTE_TO_BINARY(data[0]));
                //ESP_LOGI(UTAG, "gpio state: %4d \t 0x%04X \t " BYTE_TO_BINARY_PATTERN BYTE_TO_BINARY_PATTERN, data[1], data[1], BYTE_TO_BINARY(data[1] >> 8), BYTE_TO_BINARY(data[1]));

                // check pins with interrupts
                for ( uint8_t i = 0; i < 16; i++)
                {  
                    if ( BIT_CHECK( data[0], i) != 0)
                    {
                        // check pin state
                        uint8_t state = BIT_CHECK( data[1], i) != 0;                        
                        //ESP_LOGI(UTAG, "pin = %d, state = %d", i+1, state);                        
                        
                        // поиск коллбека для нажатия кнопок
                        for ( uint8_t j = 0; j < pin_isr_cnt; j++)
                        {
                            if ( pin_isr[ j ].pin == i )
                            {
                                //ESP_LOGI(UTAG, "pin = %d, state = %d, intr type: %d", i+1, state, pin_isr[ j ].intr_type);  
                                if ( ( state == 0 && pin_isr[ j ].intr_type == GPIO_INTR_POSEDGE) || 
                                     ( state == 1 && pin_isr[ j ].intr_type == GPIO_INTR_NEGEDGE))
                                {
                                    pin_isr[ j ].ts_down = millis();
                                    pin_isr[ j ].ts_up = millis();
                                }

                                if (( state == 1 && pin_isr[ j ].intr_type == GPIO_INTR_POSEDGE) || 
                                    ( state == 0 && pin_isr[ j ].intr_type == GPIO_INTR_NEGEDGE))
                                {
                                    pin_isr[ j ].ts_up = millis();
                                    // execute callback
                                    if ( ( pin_isr[ j ].ts_up - pin_isr[ j ].ts_down <= pin_isr[ j ].up_delay_ms ) 
                                         || 
                                         pin_isr[ j ].pin_cb2 == NULL)
                                    {
                                        pin_isr[ j ].pin_cb( pin_isr[ j ].args, &state );
                                    } else {
                                        if ( pin_isr[ j ].pin_cb2 != NULL )
                                            pin_isr[ j ].pin_cb2( pin_isr[ j ].args2, &state );
                                    }
                                } 
                                else if ( pin_isr[ j ].intr_type == GPIO_INTR_ANYEDGE )
                                {
                                    pin_isr[ j ].pin_cb( pin_isr[ j ].args, &state );
                                }
                            }
                        }                               
                    }
                }
            }
        vTaskDelay( 10 / portTICK_PERIOD_MS );
    }
    vTaskDelete(NULL);
}

esp_err_t mcp23017_isr_handler_add(uint8_t pin, gpio_int_type_t intr_type, interrupt_cb cb, void *args, interrupt_cb cb2, void *args2, uint32_t up_delay)
{
    if ( cb == NULL ) return ESP_FAIL;

    pin_isr_cnt++;
    pin_isr = (mcp23017_pin_isr_t *) realloc(pin_isr, pin_isr_cnt * sizeof(mcp23017_pin_isr_t));
    if ( pin_isr == NULL ) {
        pin_isr_cnt--;
        return ESP_FAIL;
    }

    pin_isr[ pin_isr_cnt-1 ].pin = pin;
    pin_isr[ pin_isr_cnt-1 ].pin_cb = cb;
    pin_isr[ pin_isr_cnt-1 ].intr_type = intr_type;
    pin_isr[ pin_isr_cnt-1 ].args = args;

    pin_isr[ pin_isr_cnt-1 ].pin_cb2 = cb2;
    pin_isr[ pin_isr_cnt-1 ].args2 = args2;

    pin_isr[ pin_isr_cnt-1 ].up_delay_ms = up_delay;

    return ESP_OK;     
}

void mcp23017_button_isr_cb(uint8_t pin, uint8_t *state)
{
    //ESP_LOGI(UTAG, "%s: pin %d, state %d", __func__, pin, *state);
    GPIO_ALL(pin, !GPIO_ALL_GET(pin));
}

void mcp23017_pir_sensor_cb(uint8_t pin, uint8_t *state)
{
    //ESP_LOGI(UTAG, "%s: pin %d, state %d", __func__, pin, *state);

    GPIO_ALL(pin, *state);
}


void lcd_print_(uint8_t line, const char *str)
{
    #if lcde
        LCD_print(line, str);
    #endif
}


void lcd_print(uint8_t line, const char *str)
{
    // если sens_state вздедена датчиками, то дисплей не выводит )))
    if ( display_error == 1 ) return;
    lcd_print_(line, str);
}

void show_display_error_cb(xTimerHandle tmr)   // rtos
{
    display_error = 0;
    xTimerStop( show_error_timer, 0);
    xTimerDelete(show_error_timer, 10);
    show_error_timer = NULL;
}

void print_error(const char *str)
{
    lcd_print_(0, "   *** ERROR ***    ");
    char err[21];
    if ( strlen(str) > 20 )
    {
        strncpy(err, str, 21);
        lcd_print_(1, err);  
        str += 20;
        
        if ( strlen(str) > 20 ) 
        {
            strncpy(err, str, 21);
            lcd_print_(2, err); 
            str += 20;
            lcd_print_(3, str);
        } else {
            lcd_print_(2, str);
            lcd_print_(3, "                    "); 
        }  

        
    } else {
        lcd_print_(1, "                    "); 
        lcd_print_(2, str); 
        lcd_print_(3, "                    "); 
    }
    
}

void show_display_error(const char *str)
{
    display_error = 1;

    if ( show_error_timer == NULL )
    {
        show_error_timer = xTimerCreate("dsplerr", SHOW_ERROR_TIMEOUT / portTICK_PERIOD_MS, pdFALSE, NULL, show_display_error_cb);
    }

    if ( xTimerIsTimerActive( show_error_timer ) == pdTRUE )
    {
        xTimerStop( show_error_timer, 0);
    }    

    xTimerStart( show_error_timer, 0);   

    // show error
    print_error(str);
}

void set_active_kotel(mode_e mode)
{
    switch ( mode ) {
        case MODE_MANUAL:
            active_kotel = KOTEL_NONE;
            break;
        case MODE_AUTO:
            // выбираем по времени
            active_kotel = ( time_loc.hour >= DAY_TIME && time_loc.hour < NIGHT_TIME) ? KOTEL_1 : KOTEL_2;             
            break;
        case MODE_KOTEL1: 
            active_kotel = KOTEL_1;          
            break;        
        case MODE_KOTEL2: 
            active_kotel = KOTEL_2;       
            break;
        default:      
            active_kotel = KOTEL_NONE;
    } 

    if ( mode != MODE_MANUAL) 
    {
        GPIO_ALL(100, active_kotel == KOTEL_1 );
        if ( active_kotel != KOTEL_1 ) GPIO_ALL(KOTEL1_GPIO, 0 );

        GPIO_ALL(101, active_kotel == KOTEL_2 );
        if ( active_kotel != KOTEL_2 ) GPIO_ALL(KOTEL2_GPIO, 0 );        
    } else {
        GPIO_ALL(100, 0 );
        GPIO_ALL(101, 0 );
    }
}

void change_work_mode()
{
    if ( display_error == 1 ) return;
    work_mode++;
    if ( work_mode >= MODE_MAX ) work_mode = MODE_MANUAL;
    nvs_param_save(SPACE_NAME, WORK_MODE_PARAM, &work_mode, sizeof(work_mode)); 
    set_active_kotel( work_mode );
}

void switch_menu()
{
    if ( display_error == 1 ) return;
    menu_idx++;
    if ( menu_idx >= MENU_PAGE_MAX ) menu_idx = MENU_PAGE_TEMPSET;
    last_key_press = millis();
}

int round_int_100(int val)
{
    int res = val / 100;
    int mod = val % 100;
    if ( mod >=50 ) {
        if ( res > 0) res++;
        else res--;
    }
    return res;
}

void show_main_page()
{
    char str[30];

    // tu, hh:mm    либо tu, dd.mm
    static uint32_t ii = 0;
    char weekday[2] = "";
    switch ( time_loc.dow ) {
        case 0: strcpy(weekday, "Mo"); break;
        case 1: strcpy(weekday, "Tu"); break;
        case 2: strcpy(weekday, "We"); break;
        case 3: strcpy(weekday, "Th"); break;
        case 4: strcpy(weekday, "Fr"); break;
        case 5: strcpy(weekday, "Sa"); break;
        case 6: strcpy(weekday, "Su"); break;
        default: break;
    }

    #define time_delay 20
    
    // t	u	,	1	1	:	2	2					T	→	:	2	4	.	2	°
    #define line1_pattern "%2s, %02d%s%02d   T%c:%2d.%1d%c"
    
    // K	O	T	E	L	1	(	*	)				T	←	:	3	1	.	2	°
    #define line2_pattern "%-10s  T%c:%2d.%1d%c"
    
    // S	с	h	d	:	O	F	F				>	T	s	:	2	2	.	2	°
    #define line3_pattern "Schd:%3s%4sTs:%2d.%1d%c"    
    
     // V	e	n	t	:	O	F	F					S	t	:	2	2	.	2	°
    #define line4_pattern "Vent:%3s    %2s:%2d.%1d%c"

    snprintf(str, 21, line1_pattern
                , weekday
                , ( ii % time_delay > 0) ? time_loc.hour : time_loc.day
                , ( ii % time_delay > 0) ? ( ii % 2 ? ":" : " ") : "."
                , ( ii % time_delay > 0) ? time_loc.min : time_loc.month
                , SYMBOL_ARROW_RIGHT            
                , flow_temp / 100
                , (flow_temp % 100)/10
                , SYMBOL_DEGREE  
            );

    lcd_print(0, str);

    // режим работы и котел активный
    char smode[10] = "";
    //char sactive[4];
    if ( work_mode == MODE_MANUAL ) 
    {
        strcpy(smode, "MANUAL");
        strcat(smode, "[");
        if ( GPIO_ALL_GET( KOTEL1_GPIO) )
            strcat(smode, "1");
        else 
            strcat(smode, "-" );     
        
        if ( GPIO_ALL_GET( KOTEL2_GPIO) )
            strcat(smode,  "2");
        else
            strcat(smode, "-");

        strcat(smode, "]");
    }
    else if ( work_mode == MODE_KOTEL1 ) {
        strcpy(smode, "KOTEL1");
        if ( GPIO_ALL_GET( KOTEL1_GPIO ) ) 
            strcat(smode,  "[*]");
        else
            strcat(smode,  "[-]");
    }
    else if ( work_mode == MODE_KOTEL2 ) 
    {
        strcpy(smode, "KOTEL2");
        if ( GPIO_ALL_GET( KOTEL2_GPIO ) ) 
            strcat(smode,  "[*]");
        else
            strcat(smode,  "[-]");            
    }    
    else if ( work_mode == MODE_AUTO ) 
    {
        strcpy(smode, "AUTO");
        if ( active_kotel == KOTEL_1 ) 
        {
            strcat(smode,  "[1");
            strcat(smode, THERMO_STATE(1) ? "*" : "");
            strcat(smode, "]"); 
        }
        else if ( active_kotel == KOTEL_2 )
        {
            strcat(smode,  "[2");
            strcat(smode, THERMO_STATE(2) ? "*" : "");
            strcat(smode, "]"); 
        }
        else 
            strcat( smode, "[-]"); 
    }
    else {
        strcpy(smode, "ERROR");
    }

    snprintf(str, 21, line2_pattern
                , smode
                , SYMBOL_ARROW_LEFT           
                , return_temp / 100
                , (return_temp % 100)/10
                , SYMBOL_DEGREE  
            );

    lcd_print(1, str);

    char sc[4] = "";
    if ( schedule ) 
    {
        static pos = 0;
        strcpy(sc, "");
        for ( uint8_t j = 0; j < 4; j++) 
        {
            strcat(sc, j == pos ? ">" : "-");
        }
        pos++;
        if ( pos > 4 ) pos = 0;
    } else 
        strcpy(sc, "   <");

    snprintf(str, 21, line3_pattern
                , schedule ? "ON " : "OFF"
                , sc
                , THERMO_SETPOINT(1) / 10
                , THERMO_SETPOINT(1) % 10
                , SYMBOL_DEGREE);
    lcd_print(2, str);
    
    static uint8_t show_cur_temp = 0;
    if ( ii % 5 == 0 ) show_cur_temp = 1 - show_cur_temp;

    snprintf(str, 21, line4_pattern
                , GPIO_ALL_GET( VENT_GPIO ) ? "ON " : "OFF"
                , show_cur_temp ?  "St" : "T#"
                , show_cur_temp ? ( street_temp / 10 ) : ( current_temp / 10 )
                , show_cur_temp ? ( street_temp % 10 ) : ( current_temp % 10 )
                , SYMBOL_DEGREE);
    lcd_print(3, str);

    ii++;
}

void show_page(uint8_t idx)
{
    switch ( idx ) 
    {
        case MENU_PAGE_MAIN: 
            show_main_page(); 
            break;
        case MENU_PAGE_TEMPSET: 
            //show_menu_thermostat(); 
            show_main_page();
            break;
        // case MENU_PAGE_HYST: 
        //     show_menu_hyst(); 
        //     break;
        case MENU_PAGE_MAX: 
            show_main_page(); 
            break;
        default: 
            show_main_page(); 
            break;
    } 
}

void tempset_dec()
{
    //uint16_t setpoint = THERMO_SETPOINT(1);
    TEMPSET -= TEMPSET_STEP;

    if ( TEMPSET < TEMPSET_MIN ) {
        TEMPSET = TEMPSET_MIN;
    } 

    THERMO_TEMP_SET(1, TEMPSET);
    THERMO_TEMP_SET(2, TEMPSET);
}

void tempset_inc()
{
    //uint16_t setpoint = THERMO_SETPOINT(1);
    TEMPSET += TEMPSET_STEP;

    if ( TEMPSET > TEMPSET_MAX ) {
        TEMPSET = TEMPSET_MAX;
    } 

    THERMO_TEMP_SET(1, TEMPSET);
    THERMO_TEMP_SET(2, TEMPSET);
}

void switch_schedule()
{
    if ( schedule ) 
    {
        

    } else {
        // расписание выключено, установим глобальную уставку
        THERMO_TEMP_SET(1, TEMPSET);
        THERMO_TEMP_SET(2, TEMPSET);   
    }
    schedule = 1 - schedule;

    nvs_param_save(SPACE_NAME, SCHEDULE_PARAM, &schedule, sizeof(schedule));

}

void backlight_timer_cb(xTimerHandle tmr)   // rtos
{
    uint8_t pin = (uint8_t)pvTimerGetTimerID(tmr); // rtos
    GPIO_ALL(pin, 0);
    xTimerStop(tmr, 10);
    xTimerDelete(tmr, 10);
    backlight_timer = NULL;
}

void turn_on_lcd_backlight(uint8_t pin, uint8_t *state)
{
    
   // каждое нажатие кнопки включает подсветку дисплея
    GPIO_ALL(pin, 1);

    // и запускает таймер на отключение подсветки дисплея на Х секунд, передается в аргументе
    // BACKLIGHT_TIMEOUT
    if ( backlight_timer == NULL )
    {
        backlight_timer = xTimerCreate("bcklght", BACKLIGHT_TIMEOUT * 1000 / portTICK_PERIOD_MS, pdFALSE, pin, backlight_timer_cb);
    }

    if ( xTimerIsTimerActive( backlight_timer ) == pdTRUE )
    {
        xTimerStop( backlight_timer, 0);
    }    

    xTimerStart( backlight_timer, 0);
}

void button1_short_press(void *args, uint8_t *state)
{
    uint8_t backlight = LCD_BACKLIGHT_STATE;
    turn_on_lcd_backlight( BACKLIGHT_GPIO, NULL);
    if ( backlight == 0 && sensors_param.lcden > 0) return;
    change_work_mode();
    last_key_press = millis();  
}

void button1_long_press(void *args, uint8_t *state)
{
    uint8_t backlight = LCD_BACKLIGHT_STATE;   
    turn_on_lcd_backlight( BACKLIGHT_GPIO, NULL);
    switch_schedule();
    last_key_press = millis();
}

void button2_short_press(uint8_t pin, uint8_t *state)
{  
    uint8_t backlight = LCD_BACKLIGHT_STATE;
    turn_on_lcd_backlight( BACKLIGHT_GPIO, NULL);
    if ( backlight == 0 && sensors_param.lcden > 0) return;
    if ( schedule ) {
        show_display_error("Schedule is enabled. Can't change temperature setpiont!");
    } else {
        tempset_dec();
    }
    last_key_press = millis();
}

void button2_long_press(uint8_t pin, uint8_t *state)
{  
    uint8_t backlight = LCD_BACKLIGHT_STATE;
    turn_on_lcd_backlight( BACKLIGHT_GPIO, NULL);

    if ( work_mode == MODE_AUTO || work_mode == MODE_KOTEL2) {
        show_display_error("Nelzya vklychat kotel1, mode != kotel1");
        return; // нельзя включать реле термостата котла 1, если режим Котел 2
        // TODO: индикация ошибки светодиодом 3-5 быстрых мигания  или показать ошибку на дисплее
    }
    // если на главной странице, то управляем термостатом 1
    GPIO_ALL(pin, !GPIO_ALL_GET(pin));
 
    last_key_press = millis();
}

void button3_short_press(uint8_t pin, uint8_t *state)
{
    uint8_t backlight = LCD_BACKLIGHT_STATE;
    turn_on_lcd_backlight( BACKLIGHT_GPIO, NULL);
    if ( backlight == 0 && sensors_param.lcden > 0) return;
    if ( schedule ) {
        show_display_error("Schedule is enabled. Can't change temperature setpiont!");
    } else {
        tempset_inc();
    }
    last_key_press = millis();
}

void button3_long_press(uint8_t pin, uint8_t *state)
{
    uint8_t backlight = LCD_BACKLIGHT_STATE;
    turn_on_lcd_backlight( BACKLIGHT_GPIO, NULL);
    if ( work_mode == MODE_AUTO || work_mode == MODE_KOTEL1) {
        show_display_error("Nelzya vklychat kotel2, mode != kotel2");
        return; // нельзя включать реле термостата котла 2, если режим Котел 1
    // TODO: индикация ошибки светодиодом 3-5 быстрых мигания или показать ошибку на дисплее
    }
    GPIO_ALL(pin, !GPIO_ALL_GET(pin));
    last_key_press = millis();
}

void button4_press(uint8_t pin, uint8_t *state)
{
    turn_on_lcd_backlight( BACKLIGHT_GPIO, NULL);
    GPIO_ALL(pin, !GPIO_ALL_GET(pin));
    last_key_press = millis(); 
}

void startfunc(){
    // выполняется один раз при старте модуля.

    ESP_LOGI(UTAG, "******************** VERSION = %s ****************", FW_VER);

    if ( nvs_param_load(SPACE_NAME, WORK_MODE_PARAM, &work_mode) != ESP_OK ) work_mode = MODE_MANUAL;
    ESP_LOGW(UTAG, "Loaded work mode = %d", work_mode);

    if ( nvs_param_load(SPACE_NAME, SCHEDULE_PARAM, &schedule) != ESP_OK ) schedule = 0;
    ESP_LOGW(UTAG, "Loaded schedule = %d", schedule);

    uint8_t err = 0;
    if ( KOTEL1_GPIO == 0 || KOTEL1_GPIO >=255 ) { KOTEL1_GPIO = KOTEL1_GPIO_DEFAULT ; err = 1; }
    if ( KOTEL2_GPIO == 0 || KOTEL2_GPIO >= 255 ) { KOTEL2_GPIO = KOTEL2_GPIO_DEFAULT ; err = 1; }
    if ( PUMP1_GPIO == 0 || PUMP1_GPIO >= 255 ) { PUMP1_GPIO = PUMP1_GPIO_DEFAULT ; err = 1; }
    if ( PUMP2_GPIO == 0 || PUMP2_GPIO >= 255 ) { PUMP2_GPIO = PUMP2_GPIO_DEFAULT ; err = 1; }
    if ( ESC_GPIO == 0 || ESC_GPIO >= 255 ) { ESC_GPIO = ESC_GPIO_DEFAULT ; err = 1; }
    if ( VENT_GPIO == 0 || VENT_GPIO >= 255 ) { VENT_GPIO = VENT_GPIO_DEFAULT ; err = 1; }

    if ( NIGHT_TIME >= 23 ) { NIGHT_TIME = NIGHT_TIME_DEFAULT ; err = 1; }
    if ( DAY_TIME >= 23 ) { DAY_TIME = DAY_TIME_DEFAULT ; err = 1; }

    if ( KOTEL1_LED_GPIO == 0 || KOTEL1_LED_GPIO >= 255 ) { KOTEL1_LED_GPIO = 255 ; err = 1; }
    if ( KOTEL2_LED_GPIO == 0 || KOTEL2_LED_GPIO >= 255 ) { KOTEL2_LED_GPIO = 255 ; err = 1; }
    if ( KOTEL_LED_GPIO == 0 || KOTEL_LED_GPIO >= 255 ) { KOTEL_LED_GPIO = 255 ; err = 1; }
    if ( PUMP_LED_GPIO == 0 || PUMP_LED_GPIO >= 255 ) { PUMP_LED_GPIO = 255 ; err = 1; }
    if ( SCHEDULE_LED_GPIO == 0 || SCHEDULE_LED_GPIO >= 255 ) { SCHEDULE_LED_GPIO = 255 ; err = 1; }
    if ( VENT_LED_GPIO == 0 || VENT_LED_GPIO >= 255 ) { VENT_LED_GPIO = 255 ; err = 1; }

    if ( TEMPSET < 100 || TEMPSET > 300 ) { TEMPSET = 240 ; err = 1; }

    if ( err == 1 ) SAVEOPT;

    // установить прерывания пинов
    //MCPwrite_reg16(0, GPINTENA, MCP23017_GPIO0 | MCP23017_GPIO1 | MCP23017_GPIO2 | MCP23017_GPIO3 | MCP23017_GPIO4 | MCP23017_GPIO5); // 0b0000111000000000
    //uint16_t inout = MCPread_reg16(0, IODIRA);
    //ESP_LOGI(UTAG, "%s: MCP23017 directions: " BYTE_TO_BINARY_PATTERN BYTE_TO_BINARY_PATTERN, __func__ , BYTE_TO_BINARY(inout >> 8), BYTE_TO_BINARY(inout));

    MCPwrite_reg16(0, GPINTENA, 0b1111111111111111); // 0b0000111000000000
    //MCPwrite_reg16(0, GPINTENA, inout); // 0b0000111000000000

    // условия сработки прерывания на ногах
    MCPwrite_reg16(0, INTCONA, 0);  // при нулях

    // дефолтные значения ног, прерывание сработает, если на ноге сигнал отличается от дефолтного, если на пинах значение отличается от  заданного ниже (DEFVAL  = 1 )
    MCPwrite_reg16(0, DEFVALA, 0b1111111111111111);    

    // инвертируем 13 пин
    //MCPwrite_reg16(0, IPOLA, 0b0001000100000000); // 0b0000111000000000

    // установить прерывания на GPIO
        // configure interrupts 
    gpio_config_t gpio_conf;
    gpio_conf.intr_type = GPIO_INTR_NEGEDGE; //GPIO_INTR_NEGEDGE; //GPIO_INTR_POSEDGE; // GPIO_INTR_ANYEDGE;           
    gpio_conf.mode = GPIO_MODE_INPUT;
    gpio_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_conf.pin_bit_mask = (1ULL << MCP23017_INTA_PIN);
    gpio_config(&gpio_conf);    
    gpio_install_isr_service(0);

    // прерывание на кнопки mcp23017
    gpio_isr_handler_add( MCP23017_INTA_PIN, mcp23017_isr_handler, NULL);  

        // 1 - сразу при нажатии         GPIO_INTR_POSEDGE 
        // 2 - только после отпускания   GPIO_INTR_NEGEDGE 
        // 3 - любое состояние           GPIO_INTR_ANYEDGE 
        // или наоборот, зависит от дефолтного состояния пина 1 = 1 или 0 = 2
    mcp23017_isr_handler_add( 0, GPIO_INTR_POSEDGE, button1_short_press, NULL,          button1_long_press,     NULL, 800);
    mcp23017_isr_handler_add( 1, GPIO_INTR_POSEDGE, button2_short_press, KOTEL1_GPIO,   button2_long_press,     KOTEL1_GPIO, 800);
    mcp23017_isr_handler_add( 2, GPIO_INTR_POSEDGE, button3_short_press, KOTEL2_GPIO,   button3_long_press,     KOTEL2_GPIO, 800);
    mcp23017_isr_handler_add( 3, GPIO_INTR_POSEDGE, button4_press, VENT_GPIO,     button4_press,     ESC_GPIO, 500);

    // датчик движения
    //mcp23017_isr_handler_add( 4, GPIO_INTR_ANYEDGE, mcp23017_pir_sensor_cb, 212, NULL, NULL, 0);
    
    // датчик геркон
    //mcp23017_isr_handler_add( 5, GPIO_INTR_ANYEDGE, mcp23017_pir_sensor_cb, 213, NULL, NULL, 0);

    //mcp23017_isr_handler_add( 5, GPIO_INTR_ANYEDGE, mcp23017_pir_sensor_cb, 199, NULL, NULL, 0); // display backlight

    mcp23017_queue = xQueueCreate(5, sizeof(uint16_t) * 2);
    xTaskCreate( mcp23017_isr_cb, "mcp23017_isr", 1024, NULL, 10, &mcp23017_task);

    set_active_kotel( work_mode );

    // выключить подсветку черех Х сек
    turn_on_lcd_backlight( BACKLIGHT_GPIO, NULL);
}

void timerfunc(uint32_t  timersrc) {
    // выполнение кода каждую 1 секунду
    if ( timersrc % 30 == 0 ) {
        // выполнение кода каждые 30 секунд
        set_active_kotel(work_mode);
    }

    if ( menu_idx != MENU_PAGE_MAIN && ( millis() - last_key_press >= MENU_EXIT_TIMEOUT )) 
    {
        menu_idx = MENU_PAGE_MAIN;
    }

    // управление уставками по расписанию
    if ( schedule ) {
        // проверить день недели

        // цикл по элементам
        // текущее время сравнить с установленным, 
        // если текущее меньше больше установленного, то взять текущую уставку
        // и так пройтись до конца расписания, перезаписывая уставку в термостаты
        // если не нашлось ни одной уставки в расписании, то выставить глобальную уставку
        // #maxscher - переменная прошивки, кол-во расписаний 

        uint16_t local_tempset = TEMPSET;  // по дефолту из глобальной уставки возьмем

        for ( uint8_t si = 0; si < maxscher; si++)
        {
            // проверяем день недели
            if ( BIT_CHECK( sensors_param.schweek[si], time_loc.dow ) ) 
            {
                // день недели включен в шедулере
                // теперь сравним время
                uint16_t sched_t = sensors_param.scheduler[si][1]*60 + sensors_param.scheduler[si][2];
                uint16_t loc_t = time_loc.hour * 60 + time_loc.min;
                
                if ( loc_t  >= sched_t ) 
                {
                    local_tempset = sensors_param.scheduler[si][3];
                } 

            }
        }

        shed_tempset = local_tempset;
        THERMO_TEMP_SET(1, local_tempset);
        THERMO_TEMP_SET(2, local_tempset);  
    }

    show_page( menu_idx );

    // управление термостатами воды
    if ( GPIO_ALL_GET(KOTEL1_GPIO) == 1 ) {
        // котел включен, включим термостат воды 
       THERMO_ON(3);
    } else {
        // котел выключили, ждем понижения температуры обратки
        // т.е отключения реле насоса и выключаем термостат воды
        if ( GPIO_ALL_GET(PUMP1_GPIO) == 0 || GPIO_ALL_GET(KOTEL2_GPIO) == 1) THERMO_OFF(3);
    }

    if ( GPIO_ALL_GET(KOTEL2_GPIO) == 1 ) {
        // котел включили
       THERMO_ON(4);
    } else {
        // котел выключили, ждем понижения температуры обратки
        // или отключения реле насоса и выключаем термостат воды
        if ( GPIO_ALL_GET(PUMP2_GPIO) == 0 || GPIO_ALL_GET(KOTEL1_GPIO) == 1) THERMO_OFF(4);
    }    


    // индикации 
    
    if ( work_mode != MODE_MANUAL )
    {
        // термостат котла 1
        GPIO_ALL( KOTEL1_LED_GPIO, THERMO_STATE(1) );

        // тормостат котла 2
        GPIO_ALL( KOTEL2_LED_GPIO, THERMO_STATE(2) );        
    } else {
        // в ручном режиме
        GPIO_ALL( KOTEL1_LED_GPIO, GPIO_ALL_GET( KOTEL1_GPIO ) );
        GPIO_ALL( KOTEL2_LED_GPIO, GPIO_ALL_GET( KOTEL2_GPIO ) );
    }
    



    // вентиляция
    GPIO_ALL( VENT_LED_GPIO, GPIO_ALL_GET( VENT_GPIO ) );

    // расписание
    GPIO_ALL( SCHEDULE_LED_GPIO, schedule );

    // индикация работы реле котлов, подсвечиваем, если хотя бы одно реле включено
    if ( GPIO_ALL_GET( KOTEL1_GPIO) == 0 && GPIO_ALL_GET( KOTEL2_GPIO ) == 0)
        GPIO_ALL( KOTEL_LED_GPIO, 0 );
    else
        GPIO_ALL( KOTEL_LED_GPIO, 1 );

    // индикация работы насоса, подсвечиваем, если хотя бы один насос включен
    if ( GPIO_ALL_GET( PUMP1_GPIO) == 0 && GPIO_ALL_GET( PUMP2_GPIO ) == 0)
        GPIO_ALL( PUMP_LED_GPIO, 0 );
    else
        GPIO_ALL( PUMP_LED_GPIO, 1 );

    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void webfunc(char *pbuf) 
{


   //********************************************************************************************
    os_sprintf(HTTPBUFF,"<table><tr><td>Mode:</td><td>");
 
    #define html_button_mode "<a href='#' onclick='wm(%d)'><div class='g_%d k kk fll wm' id='v%d'>%s</div></a>"
    os_sprintf(HTTPBUFF, html_button_mode, MODE_MANUAL, work_mode == MODE_MANUAL,   MODE_MANUAL, "Manual");
    os_sprintf(HTTPBUFF, html_button_mode, MODE_AUTO,   work_mode == MODE_AUTO,     MODE_AUTO, "Auto");
    os_sprintf(HTTPBUFF, html_button_mode, MODE_KOTEL1, work_mode == MODE_KOTEL1,   MODE_KOTEL1, "Kotel1");
    os_sprintf(HTTPBUFF, html_button_mode, MODE_KOTEL2, work_mode == MODE_KOTEL2,   MODE_KOTEL2, "Kotel2");
    os_sprintf(HTTPBUFF,"</td></tr><tr><td>Schedule:</td><td>");

    os_sprintf(HTTPBUFF, "<a id='ushd' href='#' data-val='%d' onclick='schd(this.dataset.val)'><div class='g_%d k kk fll' id='sch' data-text='%s'>%s</div></a><br>"
                        , !schedule
                        , schedule
                        , schedule ? "Off" : "On" //обратное значение, подставится после нажатия
                        , schedule ? "On" : "Off"
                        );   

    os_sprintf(HTTPBUFF,"</td></tr>");
    os_sprintf(HTTPBUFF,"<tr><td>Temperature:</td><td><b>%d.%d °C</b></td></tr>", current_temp / 10, current_temp % 10); 
    if ( schedule ) {
        os_sprintf(HTTPBUFF,"<tr><td>Schedule tempset:</td><td><b>%d.%d °C</b></td></tr>", shed_tempset / 10, shed_tempset % 10);     
    } else {
        os_sprintf(HTTPBUFF,"<tr><td>Global tempset:</td><td><b>%d.%d °C</b></td></tr>", TEMPSET / 10, TEMPSET % 10);     
    }
    
    os_sprintf(HTTPBUFF,"</table>");
    os_sprintf(HTTPBUFF,"<small>Version: %s</small>", FW_VER); 

    // SCRIPT
    os_sprintf(HTTPBUFF, "<script type='text/javascript'>"


                        "window.onload=function()"
                        "{"
                            "let e=document.createElement('style');"
                            "e.innerText='"
                                                ".kk{border-radius:4px;margin:-2px 2px 8px 4px;width:60px;}"
                                                "';"
                            "document.head.appendChild(e)"
                        "};"

                        "function wm(t)"
                        "{"
                            "ajax_request('/valdes?int=%d'+'&set='+t,"
                                "function(res)"
                                "{"
                                    "let v=document.getElementsByClassName('wm');"
                                    "for(let i=0;i<v.length;i++)v[i].classList.replace('g_1','g_0');"
                                    "document.getElementById('v'+t).classList.add('g_1')"
                                "}"
                            ")"
                        "};"

                        "function schd(t)"
                        "{"
                            "ajax_request("
                                "'/valdes?int=%d'+'&set='+t,"
                                "function(res)"
                                    "{"
                                        "var n=1-parseInt(t);"
                                        "var sc=document.getElementById('sch');"
                                        "sc.classList.replace('g_'+n,'g_'+t);"
                                        "sc.innerHTML=sc.getAttribute('data-text');"
                                        "document.getElementById('ushd').setAttribute('data-val',n);"
                                    "}"
                            ")"
                        "}"

                        "</script>"
                    , VALDES_INDEX_WORK_MODE
                    , VALDES_INDEX_SCHEDULE 
    );  
 



  
}