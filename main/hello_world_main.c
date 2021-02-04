#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#define SCL_LINE 27
#define SDA_LINE 26
#define I2C_PORT_NUMBER I2C_NUM_1
#define I2C_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define SLAVE_ADD 0x53
#define SIZE 8
#define READ_BIT 1
#define WRITE_BIT 0

#define DATAX0 0X32
#define DATAX1 0X33
#define DATAY0 0X34
#define DATAY1 0X35
#define DATAZ0 0X36
#define DATAZ1 0X37

#define POWER_CTL 0x2d
#define STANDBY_MASK 0X08 //MEASURE BIT CAN BE MASK TO ENTER STANDBY MODE. 
#define DEVID_FIXED  0xe5       //as per data sheet its should be 0xe5 but shows 0x43
#define DEVID_REG 0x00
#define BW_RATE 0x2c
#define BW_RATE_LOW_POWER 0x10


#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "hal/i2c_types.h"

TaskHandle_t led_handle;
TaskHandle_t Task2_handle;

TickType_t xLastWakeTime;
const TickType_t xFrequency=50;

gpio_config_t io_conf5;
i2c_config_t i2c_conf;


typedef struct acc_data {
    int8_t x;
    int16_t y;
    int16_t z;
}sensor_data_t;

static esp_err_t i2c_init()    {
    esp_err_t rt;
    int i2c_master_port = I2C_PORT_NUMBER;
    i2c_conf.mode = I2C_MODE_MASTER;
    i2c_conf.sda_io_num = SDA_LINE;
    i2c_conf.scl_io_num = SCL_LINE;
    i2c_conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_conf.master.clk_speed = I2C_FREQ_HZ;
    i2c_param_config(i2c_master_port,&i2c_conf);
    rt = i2c_driver_install(i2c_master_port,i2c_conf.mode,
            I2C_MASTER_RX_BUF_DISABLE,I2C_MASTER_TX_BUF_DISABLE,0);
    return rt;
}


static esp_err_t i2c_master_read_slave_reg(i2c_port_t i2c_num, uint8_t i2c_add, uint8_t i2c_reg, uint8_t *data_rd, size_t size)
{
    esp_err_t ret;
    if (size == 0) {
        return ESP_OK;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    //sending device address indicating registerto be written
    i2c_master_write_byte(cmd, (SLAVE_ADD << 1), 0x1); //write
    //send register we want
    i2c_master_write_byte(cmd, i2c_reg, 0x1); 
    //send repeated start
    i2c_master_start(cmd);
    //now send device address (indicating read) & read data

    i2c_master_write_byte(cmd, (SLAVE_ADD << 1 ) | READ_BIT, 0x1);//read
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, 0x1);
    }

    i2c_master_read_byte(cmd, data_rd + size - 1, 0x0);
    i2c_master_stop(cmd);

    i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);

    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_master_write_slave_reg(i2c_port_t i2c_num, uint8_t i2c_add, uint8_t i2c_reg, uint8_t *data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    //first send device address (W)& register to be written
    i2c_master_write_byte(cmd, (i2c_add << 1) | WRITE_BIT, 0x1);
    //send register we want
    i2c_master_write_byte(cmd, i2c_reg, 0x1);
    //write data 
    i2c_master_write(cmd, data_wr, size, 0x1);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}


esp_err_t adxl345_read(uint8_t reg, uint8_t *pdata, uint8_t count)  {
    return( i2c_master_read_slave_reg( I2C_PORT_NUMBER, SLAVE_ADD, reg, pdata, count) );
}

esp_err_t adxl345_write(uint8_t reg, uint8_t *pdata, uint8_t count)  {
    return( i2c_master_write_slave_reg( I2C_PORT_NUMBER, SLAVE_ADD, reg, pdata, count) );
}

static void adxl345_init() {
    
    uint8_t val;
    esp_err_t error;
    error = adxl345_read(POWER_CTL, &(val),1);
    ESP_LOGI("ADXL345 INIT task", "power control register (0x2d)value= 0x%X (OK)", val);

    //stand-by mode
    val |= (STANDBY_MASK);
        
    adxl345_write(POWER_CTL,&(val),1);
        ESP_LOGI("ADXL345 INIT task", "power control register = 0x%X (OK)", val);
    error = adxl345_read(BW_RATE, &(val),1);
        ESP_LOGI("ADXL345 INIT task", "BW_RATE(0x2c)value= 0x%X (OK)", val);
    val &= ~(BW_RATE_LOW_POWER);
    adxl345_write(BW_RATE,&(val),1);
        ESP_LOGI("ADXL345 INIT task", "BW_RATE(0x2c)LOW POWER cleared value= 0x%X (OK)", val);
    error = adxl345_read(DEVID_REG, &(val), 1);
        //error check
    if(error == ESP_FAIL) {
        ESP_LOGD("Error","ACC sending cmd error- %d",error);
    }
    else if(error == ESP_ERR_INVALID_ARG){
        ESP_LOGD("Error","ACC invalid param- %d",error);
    }
    else if(error == ESP_ERR_INVALID_STATE){
        ESP_LOGD("Error","ACC driver not installed or not in driver mode- %d",error);
    }
    else if(error == ESP_ERR_TIMEOUT){
        ESP_LOGD("Error","ACC BUS BUSY TIMEOUT- %d",error);
    }
    else if(error == ESP_OK)    {
        if(val == DEVID_FIXED)  {
            ESP_LOGI("ADXL345 INIT task", "ID = 0x%X (OK)", val);
        }
        else    {
            ESP_LOGE("ADXL345 INIT Task", "ID = 0x%X (not correct)(should be 0x%X)",val,DEVID_FIXED);
        }
    }

//read all registers
    error = adxl345_read(0x00,&(val),1);
        ESP_LOGI("result:CHECK","Accelerometer err:%d\t 0x00:%3d",error,val);
    error = adxl345_read(0x1d,&(val),1);
        ESP_LOGI("result:CHECK","Accelerometer err:%d\t 0x1d:%3d",error,val);
    error = adxl345_read(0x1e,&(val),1);
        ESP_LOGI("result:CHECK","Accelerometer err:%d\t 0x1e:%3d",error,val);
    error = adxl345_read(0x1f,&(val),1);
        ESP_LOGI("result:CHECK","Accelerometer err:%d\t 0x1f:%3d",error,val);
    error = adxl345_read(0x20,&(val),1);
        ESP_LOGI("result:CHECK","Accelerometer err:%d\t 0x20:%3d",error,val);
    error = adxl345_read(0x22,&(val),1);
        ESP_LOGI("result:CHECK","Accelerometer err:%d\t 0x22:%3d",error,val);
    error = adxl345_read(0x24,&(val),1);
        ESP_LOGI("result:CHECK","Accelerometer err:%d\t 0x24:%3d",error,val);
    error = adxl345_read(0x25,&(val),1);
        ESP_LOGI("result:CHECK","Accelerometer err:%d\t 0x25:%3d",error,val);
    error = adxl345_read(0x26,&(val),1);
        ESP_LOGI("result:CHECK","Accelerometer err:%d\t 0x26:%3d",error,val);
    error = adxl345_read(0x27,&(val),1);
        ESP_LOGI("result:CHECK","Accelerometer err:%d\t 0x27:%3d",error,val);
    error = adxl345_read(0x28,&(val),1);
        ESP_LOGI("result:CHECK","Accelerometer err:%d\t 0x28:%3d",error,val);
    error = adxl345_read(0x29,&(val),1);
        ESP_LOGI("result:CHECK","Accelerometer err:%d\t 0x29:%3d",error,val);
    error = adxl345_read(0x2a,&(val),1);
        ESP_LOGI("result:CHECK","Accelerometer err:%d\t 0x2a:%3d",error,val);
    error = adxl345_read(0x2b,&(val),1);
        ESP_LOGI("result:CHECK","Accelerometer err:%d\t 0x2b:%3d",error,val);
    error = adxl345_read(0x2c,&(val),1);
        ESP_LOGI("result:CHECK","Accelerometer err:%d\t 0x2c:%3d",error,val);
    error = adxl345_read(0x2d,&(val),1);
        ESP_LOGI("result:CHECK","Accelerometer err:%d\t 0x2d:%3d",error,val);
    error = adxl345_read(0x2e,&(val),1);
        ESP_LOGI("result:CHECK","Accelerometer err:%d\t 0x2e:%3d",error,val);
    error = adxl345_read(0x2f,&(val),1);
        ESP_LOGI("result:CHECK","Accelerometer err:%d\t 0x2f:%3d",error,val);
    error = adxl345_read(0x30,&(val),1);
        ESP_LOGI("result:CHECK","Accelerometer err:%d\t 0x30:%3d",error,val);
    error = adxl345_read(0x31,&(val),1);
        ESP_LOGI("result:CHECK","Accelerometer err:%d\t 0x31:%3d",error,val);
    error = adxl345_read(0x32,&(val),1);
        ESP_LOGI("result:CHECK","Accelerometer err:%d\t 0x32:%3d",error,val);
    error = adxl345_read(0x33,&(val),1);
        ESP_LOGI("result:CHECK","Accelerometer err:%d\t 0x33:%3d",error,val);
    error = adxl345_read(0x34,&(val),1);
        ESP_LOGI("result:CHECK","Accelerometer err:%d\t 0x34:%3d",error,val);
    error = adxl345_read(0x35,&(val),1);
        ESP_LOGI("result:CHECK","Accelerometer err:%d\t 0x35:%3d",error,val);
    error = adxl345_read(0x36,&(val),1);
        ESP_LOGI("result:CHECK","Accelerometer err:%d\t 0x36:%3d",error,val);
    error = adxl345_read(0x37,&(val),1);
        ESP_LOGI("result:CHECK","Accelerometer err:%d\t 0x37:%3d",error,val);
    error = adxl345_read(0x38,&(val),1);
        ESP_LOGI("result:CHECK","Accelerometer err:%d\t 0x38:%3d",error,val);
    error = adxl345_read(0x39,&(val),1);
        ESP_LOGI("result:CHECK","Accelerometer err:%d\t 0x39:%3d",error,val);
    error = adxl345_read(DATAX0,&(val),1);
        ESP_LOGI("result:CHECK","Accelerometer err:%d\t xL:%3d",error,val);
    error = adxl345_read(DATAX1,&(val),1);
        ESP_LOGI("result:CHECK","Accelerometer err:%d\t xH:%3d",error,val);
    error = adxl345_read(DATAY0,&(val),1);
        ESP_LOGI("result:CHECK","Accelerometer err:%d\t yL:%3d",error,val);
    error = adxl345_read(DATAY1,&(val),1);
        ESP_LOGI("result:CHECK","Accelerometer err:%d\t yH:%3d",error,val);
    error = adxl345_read(DATAZ0,&(val),1);
        ESP_LOGI("result:CHECK","Accelerometer err:%d\t zL:%3d",error,val);
    error = adxl345_read(DATAZ1,&(val),1);
        ESP_LOGI("result:CHECK","Accelerometer err:%d\t zH:%3d",error,val);
} 

uint16_t byte_swap(uint16_t data)   {
    return((data >> 8) | (data << 8));
}

void app_main(void) {

    ESP_LOGD("Main task--","Priority-%d\n",uxTaskPriorityGet(NULL));
//   xTaskCreate(blink_task,"BLINK_Task", 2048, NULL, 5, &led_handle);
    uint8_t xL,xH,yL,yH,zL,zH;
    uint8_t x,y,z;
    io_conf5.mode = GPIO_MODE_OUTPUT;
    io_conf5.pin_bit_mask = (1ULL<<25); //bit mask of the pins that you wnat to set, eg: GPIO18
    io_conf5.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf5.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf5);

    sensor_data_t acc;
    esp_err_t error;

    error = i2c_init();//initialization of i2c
    adxl345_init();

    if(error == ESP_OK)   {
        printf("i2c initialized successfully \n");    
        //write
        printf("start writing \n");
    
        while(1)    {
            //led blink to check if working 
            vTaskDelay(200/portTICK_PERIOD_MS);
            gpio_set_level(25,1);
            vTaskDelay(200/portTICK_PERIOD_MS);
            gpio_set_level(25,0);
            printf("bytes requested- %d\n",sizeof(acc));
            error = adxl345_read(DATAX0, (uint8_t *)&acc, sizeof(acc));
            /*error = adxl345_read(DATAX0, &xL, sizeof(xL));
            error = adxl345_read(DATAX1, &xH, sizeof(xH));
            x = (xH << 8) | (xL & 0xff);
            error = adxl345_read(DATAY0, &yL, sizeof(yL));
            error = adxl345_read(DATAY1, &yH, sizeof(yH));
            y = (yH << 8) | (yL & 0xff);
            error = adxl345_read(DATAZ0, &zL, sizeof(zL));
            error = adxl345_read(DATAZ1, &zH, sizeof(zH));
            z = (zH << 8) | (zL & 0xff);*/

            //ESP_LOGI("result:","Accelerometer err:%d\t x:%3d\t y:%3d\t z:%3d", error, x, y, z);

            if(error == ESP_OK) {
                /*
                //byte swap to make little-endian
                acc.x = byte_swap(acc.x);
                acc.y = byte_swap(acc.y);
                acc.z = byte_swap(acc.z);
                //shift each walue to align 14-bits in 16-bits ints
                acc.x/=4;
                acc.y/=4;
                acc.z/=4;
*/
                ESP_LOGI("result:","Accelerometer err:%d\t x:%3d\t y:%3d\t z:%3d", error, acc.x, acc.y, acc.z);
                //ESP_LOGI("result:","Accelerometer err:%d\t x:%3d",error,x);

            }
            else if(error == ESP_FAIL) {
                ESP_LOGD("Error","ACC sending cmd error- %d",error);
            }
            else if(error == ESP_ERR_INVALID_ARG){
                ESP_LOGD("Error","ACC invalid param- %d",error);
            }
            else if(error == ESP_ERR_INVALID_STATE){
                ESP_LOGD("Error","ACC driver not installed or not in driver mode- %d",error);
            }
            else if(error == ESP_ERR_TIMEOUT){
                ESP_LOGD("Error","ACC BUS BUSY TIMEOUT- %d",error);
            }
        }
    }

    else    {
        ESP_LOGD("install failed : ","couldn't install i2c\n");
    }
}