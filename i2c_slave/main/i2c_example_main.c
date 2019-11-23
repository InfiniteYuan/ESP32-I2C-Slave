/* i2c - Example

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "soc/dport_reg.h"
#include "soc/i2c_reg.h"
#include "soc/i2c_struct.h"
#include "soc/i2c_reg.h"
#include "driver/i2c.h"

static const char *TAG = "i2c-example";

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 512                  /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 128               /*!< Data length for r/w test, [0,DATA_LENGTH] */
#define DELAY_TIME_BETWEEN_ITEMS_MS 1000 /*!< delay time between different test items */

#define ESP_SLAVE_ADDR CONFIG_I2C_SLAVE_ADDRESS /*!< ESP32 slave address, you can set any 7bit value */

#ifdef CONFIG_EXAMPLE_I2C_SLAVE
    #define I2C_SLAVE_SCL_IO CONFIG_I2C_SLAVE_SCL               /*!< gpio number for i2c slave clock */
    #define I2C_SLAVE_SDA_IO CONFIG_I2C_SLAVE_SDA               /*!< gpio number for i2c slave data */
    #define I2C_SLAVE_NUM I2C_NUMBER(CONFIG_I2C_SLAVE_PORT_NUM) /*!< I2C port number for slave dev */
    #define I2C_SLAVE_TX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C slave tx buffer size */
    #define I2C_SLAVE_RX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C slave rx buffer size */
#endif

#ifdef CONFIG_EXAMPLE_I2C_MASTER
    #define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
    #define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */
    #define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
    #define I2C_MASTER_FREQ_HZ 100000        /*!< I2C master clock frequency */
    #define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
    #define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#endif

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

static DRAM_ATTR i2c_dev_t* const I2C[I2C_NUM_MAX] = { &I2C0, &I2C1 };

/**
 * @brief i2c_master_write_slave
 *
 * ___________________________________________________________________
 * | start | slave_addr + write_bit + ack | register address | write n bytes + ack  | stop |
 * --------|-----------------------------|----------------------|------------------|------|
 *
 */
static esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t reg_addr, uint8_t *data_wr, size_t size, TickType_t ticks_to_wait)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);	//写地址到从设备
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN); //写多个字节的
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, ticks_to_wait);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief i2c_master_read_slave
 *
 * ___________________________________________________________________
 * | start | slave_addr + write_bit + ack | register address |start | slave_addr + read_bit + ack |  read n-1 bytes + ack | read 1 byte + nack | stop |
 *
 */
static esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, uint8_t reg_addr, uint8_t* data_rd, size_t size, TickType_t ticks_to_wait)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t ret;
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);		//dummy write寄存器地址
    
    i2c_master_start(cmd);//起始位
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
	i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);// 读取字节不校验
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, ticks_to_wait);
    i2c_cmd_link_delete(cmd);
    return ret;
}
#ifdef CONFIG_EXAMPLE_I2C_MASTER
/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init()
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}
#endif

#ifdef CONFIG_EXAMPLE_I2C_SLAVE
typedef enum {
    SLAVE_WRITE = 0,
    SLAVE_READ = 1,
    SLAVE_IDLE = 2,
} slave_event_type;

uint8_t w_r_index = 0;
uint8_t slave_send_index = 0;
uint8_t slave_data[128] = {0x01};
slave_event_type slave_event = SLAVE_IDLE;

/* DRAM_ATTR is required to avoid I2C array placed in flash, due to accessed from ISR */
static DRAM_ATTR i2c_dev_t* const I2C_INSTANCE[I2C_NUM_MAX] = { &I2C0, &I2C1 };

static void IRAM_ATTR i2c_slave_isr_handler_default(void* arg)
{

    int i2c_num = I2C_SLAVE_NUM;
    uint32_t status = I2C_INSTANCE[i2c_num]->int_status.val;
    int idx = 0;

    portBASE_TYPE HPTaskAwoken = pdFALSE;
    while (status != 0) {
        status = I2C_INSTANCE[i2c_num]->int_status.val;
        if (status & I2C_ACK_ERR_INT_ST_M) {
            ets_printf("ae\n");
            I2C_INSTANCE[i2c_num]->int_ena.ack_err = 0;
            I2C_INSTANCE[i2c_num]->int_clr.ack_err = 1;
        } else if (status & I2C_TRANS_COMPLETE_INT_ST_M) { // receive data after receive device address + W/R bit and register address
            // ets_printf("tc, ");
            I2C_INSTANCE[i2c_num]->int_clr.trans_complete = 1;

            int rx_fifo_cnt = I2C_INSTANCE[i2c_num]->status_reg.rx_fifo_cnt;
            if (I2C_INSTANCE[i2c_num]->status_reg.slave_rw) { // read, slave should to send
                // ets_printf("R\n");
            } else { // write, slave should to recv
                // ets_printf("W ");
                ets_printf("Slave Recv");
                
                for (idx = 0; idx < rx_fifo_cnt; idx++) {
                    slave_data[w_r_index++] = I2C_INSTANCE[i2c_num]->fifo_data.data;
                }
                ets_printf("\n");
            }
            I2C_INSTANCE[i2c_num]->int_clr.rx_fifo_full = 1;
            slave_event = SLAVE_IDLE;
        } else if (status & I2C_SLAVE_TRAN_COMP_INT_ST_M) { // slave receive device address + W/R bit + register address
            if (I2C_INSTANCE[i2c_num]->status_reg.slave_rw) { // read, slave should to send
                ets_printf("sc, Slave Send\n");
            } else { // write, slave should to recv
                // ets_printf("sc W\n");
                w_r_index = I2C_INSTANCE[i2c_num]->fifo_data.data;
                switch (slave_event) {
                    case SLAVE_WRITE:
                        ets_printf("sc, W2W\n");
                        break;
                    case SLAVE_READ:
                        ets_printf("sc, R2W\n");
                        break;
                    case SLAVE_IDLE:
                        ets_printf("sc, I2W\n");
                        // reset tx fifo to avoid send last byte when master send read command next.
                        i2c_reset_tx_fifo(i2c_num);
                        // I2C_INSTANCE[i2c_num]->fifo_conf.tx_fifo_rst = 1;
                        // I2C_INSTANCE[i2c_num]->fifo_conf.tx_fifo_rst = 0;

                        slave_event = SLAVE_WRITE;
                        slave_send_index = w_r_index;

                        int tx_fifo_rem = I2C_FIFO_LEN - I2C_INSTANCE[i2c_num]->status_reg.tx_fifo_cnt;
                        for (size_t i = 0; i < tx_fifo_rem; i++) {
                            WRITE_PERI_REG(I2C_DATA_APB_REG(i2c_num), slave_data[slave_send_index++]);
                        }
                        I2C_INSTANCE[i2c_num]->int_ena.tx_fifo_empty = 1;
                        I2C_INSTANCE[i2c_num]->int_clr.tx_fifo_empty = 1;
                        break;
                }
            }

            I2C_INSTANCE[i2c_num]->int_clr.slave_tran_comp = 1;
        } else if (status & I2C_TXFIFO_EMPTY_INT_ST_M) {
            ets_printf("tfe, ");
            int tx_fifo_rem = I2C_FIFO_LEN - I2C_INSTANCE[i2c_num]->status_reg.tx_fifo_cnt;

            if (I2C_INSTANCE[i2c_num]->status_reg.slave_rw) { // read, slave should to send
                ets_printf("R\r\n");
                for (size_t i = 0; i < tx_fifo_rem; i++) {
                    WRITE_PERI_REG(I2C_DATA_APB_REG(i2c_num), slave_data[slave_send_index++]);
                }
                I2C_INSTANCE[i2c_num]->int_ena.tx_fifo_empty = 1;
                I2C_INSTANCE[i2c_num]->int_clr.tx_fifo_empty = 1;
            } else { // write, slave should to recv
                ets_printf("W\r\n");
            }
        } else if (status & I2C_RXFIFO_OVF_INT_ST_M) {
            ets_printf("rfo\n");
            I2C_INSTANCE[i2c_num]->int_clr.rx_fifo_ovf = 1;
        } else if (status & I2C_RXFIFO_FULL_INT_ST_M) {
            ets_printf("rff\n");
            int rx_fifo_cnt = I2C_INSTANCE[i2c_num]->status_reg.rx_fifo_cnt;
            for (idx = 0; idx < rx_fifo_cnt; idx++) {
                slave_data[w_r_index++] = I2C_INSTANCE[i2c_num]->fifo_data.data;
            }
            I2C_INSTANCE[i2c_num]->int_clr.rx_fifo_full = 1;
        } else {
            // ets_printf("%x\n", status);
            I2C_INSTANCE[i2c_num]->int_clr.val = status;
        }
    }
    //We only need to check here if there is a high-priority task needs to be switched.
    if(HPTaskAwoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

/**
 * @brief i2c slave initialization
 */
static esp_err_t i2c_slave_init()
{
    int i2c_slave_port = I2C_SLAVE_NUM;
    i2c_config_t conf_slave;
    conf_slave.sda_io_num = I2C_SLAVE_SDA_IO;
    conf_slave.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf_slave.scl_io_num = I2C_SLAVE_SCL_IO;
    conf_slave.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf_slave.mode = I2C_MODE_SLAVE;
    conf_slave.slave.addr_10bit_en = 0;
    conf_slave.slave.slave_addr = ESP_SLAVE_ADDR;
    i2c_param_config(i2c_slave_port, &conf_slave);
    i2c_driver_install(i2c_slave_port, conf_slave.mode,
                              I2C_SLAVE_RX_BUF_LEN,
                              I2C_SLAVE_TX_BUF_LEN, 0);
    if (i2c_isr_register(I2C_SLAVE_NUM, i2c_slave_isr_handler_default, NULL, 0, NULL) != ESP_OK) {
        ESP_LOGE(TAG, "i2c_isr_register");
    }
    for (size_t i = 0; i < 128; i++) {
        slave_data[i] = 128 - i;
    }
    
    return ESP_OK;
}
#endif
/**
 * @brief test function to show buffer
 */
static void disp_buf(uint8_t *buf, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        printf("%02x ", buf[i]);
        if ((i + 1) % 16 == 0) {
            printf("\n");
        }
    }
    printf("\n");
}

#ifdef CONFIG_EXAMPLE_I2C_MASTER
static void i2c_master_task(void *arg)
{
    int i = 0;
    uint8_t dat[128];
    uint8_t *data_rd = (uint8_t *)malloc(DATA_LENGTH);
	printf("power on\r\n");
	for(i = 0; i < 128; i++){
		dat[i] = i;
	}
    if (i2c_master_write_slave(I2C_MASTER_NUM, 0x00, dat, 33, 1000 / portTICK_RATE_MS) != ESP_OK) //ok 主发送数据正常
	    printf("master write error\r\n");

    while (1) {
        if (i2c_master_read_slave(I2C_MASTER_NUM, 0x02, data_rd, 65, 1000 / portTICK_RATE_MS) == ESP_OK) {
            disp_buf(data_rd, 65);
        } else {
    	    printf("master read error\r\n");
        }

        vTaskDelay(500 / portTICK_PERIOD_MS);

        memset(dat, i++, 33);
        if (i2c_master_write_slave(I2C_MASTER_NUM, 0x00, dat, 33, 1000 / portTICK_RATE_MS) != ESP_OK) //ok 主发送数据正常
            printf("master write error\r\n");
        
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}
#endif

void app_main()
{
#if CONFIG_EXAMPLE_I2C_MASTER
    ESP_ERROR_CHECK(i2c_master_init());
    xTaskCreate(i2c_master_task, "i2c_master_task", 1024 * 2, (void *)0, 10, NULL);
#else
    ESP_ERROR_CHECK(i2c_slave_init());
#endif
}
