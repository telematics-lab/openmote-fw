/**
 * @file       main.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "BoardImplementation.hpp"

#include "platform_types.hpp"

#include "I2c.hpp"

#include "Serial.hpp"

#include "Callback.hpp"
#include "Scheduler.hpp"
#include "Semaphore.hpp"
#include "Task.hpp"

#include "Ina226.hpp"
#include "Ina226_regs.h"

/*================================ define ===================================*/

#define GREEN_LED_TASK_PRIORITY         ( tskIDLE_PRIORITY + 0 )
#define PARSER_TASK_PRIORITY            ( tskIDLE_PRIORITY + 1 )
#define INA226_TASK_PRIORITY            ( tskIDLE_PRIORITY + 2 )

#define UART_BAUDRATE						        ( 1267200 )

#define SERIAL_BUFFER_LENGTH            ( 256 )

#define INA226_CMD_NONE                 ( 0x00 )
#define INA226_CMD_START                ( 0x01 )
#define INA226_CMD_STOP                 ( 0x02 )

#define INA226_IRQ_PORT                 ( GPIO_B_BASE )
#define INA226_IRQ_PIN                  ( GPIO_PIN_2 )
#define INA226_IRQ_IOC                  ( IOC_OVERRIDE_PUE )
#define INA226_IRQ_EDGE                 ( GPIO_FALLING_EDGE )
#define INA226_I2C_ADDRESS              ( 0x40 )

#define INA226_USER_CONFIG              ( INA226_CONFIG_DEFAULT | \
                                          INA226_CONFIG_AVG_BIT_1 | INA226_CONFIG_CT_VSH_1100US | \
                                          INA226_CONFIG_CT_VBUS_1100US | INA226_CONFIG_MODE_SV_CONT )
#define INA226_USER_CALIBRATION         ( 0x00 )

/*================================ typedef ==================================*/

typedef struct
{
  uint8_t cmd;
  uint16_t cfg;
  uint32_t timeout;
} ina226_cfg_t;

/*=============================== prototypes ================================*/

static void prvGreenLedTask(void *pvParameters);
static void prvIna226Task(void *pvParameters);
static void prvParserTask(void *pvParameters);

static bool ina226_cfg_parse(uint8_t* data, uint16_t length, ina226_cfg_t* ina226_cfg);

/*=============================== variables =================================*/

static Serial serial(uart0);

static uint8_t serial_rx_buffer[SERIAL_BUFFER_LENGTH];
static uint16_t serial_rx_buffer_len = sizeof(serial_rx_buffer);

static Task heartbeatTask{(const char *) "Heartbeat", 128, GREEN_LED_TASK_PRIORITY, prvGreenLedTask, nullptr};
static Task ina226Task{(const char *) "Ina226", 128, INA226_TASK_PRIORITY, prvIna226Task, nullptr};
static Task parserTask{(const char *) "Parser", 128, PARSER_TASK_PRIORITY, prvParserTask, nullptr};

static const GpioConfig ina226_irq_cfg {INA226_IRQ_PORT, INA226_IRQ_PIN, INA226_IRQ_IOC, INA226_IRQ_EDGE, 0};
static GpioIn ina226_irq {ina226_irq_cfg};
static Ina226 ina226 {i2c, ina226_irq};

static SemaphoreBinary run_semaphore {false};

static bool finished = false;
static uint32_t timeout = 0;

static Ina226Config ina226_config {.config = INA226_USER_CONFIG, .calibration = INA226_USER_CALIBRATION};

/*================================= public ==================================*/

int main (void)
{
  /* Initialize the board */
  board.init();
  
  /* Enable the UART interface */
  uart0.enable(UART_BAUDRATE);
  
  /* Initialize Serial interface */
  serial.init();
  
  /* Enable I2C */
  i2c.enable();

  /* Start the scheduler */
  Scheduler::run();
}

/*=============================== protected =================================*/

/*================================ private ==================================*/

static void prvParserTask(void *pvParameters)
{
  while(true)
  {
    ina226_cfg_t ina226_cfg;
    uint16_t length;
    bool result;
    
    /* Read buffer using Serial */
    length = serial.read(serial_rx_buffer, serial_rx_buffer_len);
    
    /* Check status */
    if (length > 0)
    {
      result = ina226_cfg_parse(serial_rx_buffer, serial_rx_buffer_len, &ina226_cfg);
    }
    else
    {
      result = false;
    }
    
    /* If result is successful */
    if (result)
    {
      /* Select command to execute */
      switch (ina226_cfg.cmd)
      {
        case INA226_CMD_START:
          /* Notify task to run with timeout */
          finished = false;
          timeout = ina226_cfg.timeout;
          ina226_config.config = ina226_cfg.cfg;
          run_semaphore.give();
          break;
        case INA226_CMD_STOP:
          /* Notify task to stop */
          finished = true;
          break;
        default:
            break;
      }
    }
  }
}

static void prvIna226Task(void *pvParameters)
{
  /* Initialize INA226 */
  ina226.init(INA226_I2C_ADDRESS);
    
  /* Forever */
  while (true)
  {
    
    uint32_t start_time, current_time;
    int32_t elapsed_time;
    bool check_timeout;
    bool status;
    
    /* Wait until the task can run */
    run_semaphore.take();
    
    /* Reset INA226 */
    ina226.reset();
    
    /* Configure INA226 */
    status = ina226.configure(ina226_config);
    
    /* Start INA226 */
    status = ina226.start();
    
    /* Get start_time and current_time */
    start_time   = Scheduler::get_ms();
    current_time = Scheduler::get_ms();
    
    /* If there is timeout, we need to check it */
    if (timeout > 0)
    {
      check_timeout = true;
    }
    else
    {
      check_timeout = false;
    }
    
    finished = false;
    
    /* Run until finished */
    while (!finished)
    {
      Ina226Data ina226_data;
      uint8_t buffer[2];

      /* Read data from INA226, blocking */
      status = ina226.read(ina226_data, INA226_MEASURE_SHUNT, 0);
      
      if (!status)
      {
        break;
      }
      
      /* Turn on orange LED */
      led_orange.on();
      
      /* Parse INA226 data to buffer */
      buffer[0] = ((ina226_data.shunt & 0x00FF) >> 0);
      buffer[1] = ((ina226_data.shunt & 0xFF00) >> 8);
      
      /* Write INA226 data to serial */
      serial.write(buffer, sizeof(buffer), true);
      
      /* Get current_time and calculate elapsed_time */
      if (check_timeout)
      {
        /* Update current and elapsed time */
        current_time = Scheduler::get_ms();
        elapsed_time = current_time - start_time;

        /* Check for timeout */
        if (elapsed_time >= timeout)
        {
          finished = true;
        }
      }
      
      /* Turn on orange LED */
      led_orange.off();
    }
    
    /* Stop INA226 */
    ina226.stop();
  }
}

static void prvGreenLedTask(void *pvParameters)
{
  /* Forever */
  while (true)
  {
    /* Turn off green LED for 950 ms */
    led_green.off();
    Scheduler::delay_ms(950);
    
    /* Turn on green LED for 50 ms */
    led_green.on();
    Scheduler::delay_ms(50);
  }
}

static bool ina226_cfg_parse(uint8_t* data, uint16_t length, ina226_cfg_t* ina226_cfg)
{
  /* Parse INA226 configuration message */
  ina226_cfg->cmd     = data[0];
  ina226_cfg->cfg     = data[2] <<  8 | data[1] << 0;
  ina226_cfg->timeout = data[3] << 24 | data[4] << 16 | data[5] << 8 | data[6] << 0;
  
  return true;
}
