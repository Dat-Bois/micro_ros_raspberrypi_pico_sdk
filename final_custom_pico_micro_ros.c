#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/imu.h>
#include <rmw_microros/rmw_microros.h>
#include <adafruit-gfx.h>
#include <adafruit-st7735.h>
#include "functions-st7735.h"


#include "pico/stdlib.h"
#include "pico_uart_transports.h"

#define TFT_CS        5
#define TFT_RST       7 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC        8
#define TFT_MOSI 3  // Data out
#define TFT_SCLK 2  // Clock out

const uint LED_PIN = 25;

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg_heartbeat;

rcl_subscription_t battery_subscriber;
sensor_msgs__msg__BatteryState volt_msg;

rcl_subscription_t imu_subscriber;
sensor_msgs__msg__Imu imu_msg;

rcl_subscription_t led_subscriber;
std_msgs__msg__Float32 led_msg;


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

double voltage = 0.0;
double roll = 0.0;
double pitch = 0.0;
double yaw = 0.0;
double led_test = 0.0;
bool kidnap = false;

void error_loop(){
  while(1){
    gpio_put(LED_PIN, !gpio_get(LED_PIN));
    sleep_ms(100);
  }
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    rcl_ret_t ret = rcl_publish(&publisher, &msg_heartbeat, NULL);
    msg_heartbeat.data++;
}

void led_subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Float32 * led_msg = (const std_msgs__msg__Float32 *)msgin;
  //digitalWrite(LED_PIN, (msg->data == 0) ? LOW : HIGH); 
  if(led_msg->data == 0) {gpio_put(LED_PIN, 0);} else{gpio_put(LED_PIN, 1);}
  //st7735 *st = oled_create(TFT_CS,TFT_DC,TFT_MOSI,TFT_SCLK,TFT_RST);
  //printData(st,msg_led->data); 
  led_test = led_msg->data;
}

int main()
{
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    gpio_init(LED_PIN);
    gpio_init(TFT_CS);
    gpio_init(TFT_DC);
    gpio_init(TFT_MOSI);
    gpio_init(TFT_SCLK);
    gpio_init(TFT_RST);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor_pub;
    rclc_executor_t executor_sub;
    rcl_timer_t timer;

    allocator = rcl_get_default_allocator();

    //custom stuff-----
    st7735 *st = oled_create(TFT_CS,TFT_DC,TFT_MOSI,TFT_SCLK,TFT_RST);
    oled_initR(st, INITR_144GREENTAB);
    gfx_fillScreen(st->gfx, ST77XX_GREEN);
    sleep_ms(5000);
    gfx_fillScreen(st->gfx, ST77XX_BLACK);
    //testlines(st, ST77XX_GREEN);
    printData(st, 23.2921, roll, pitch, yaw, kidnap);
    //-----------------

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    RCCHECK(rclc_node_init_default(&node, "pico_node", "", &support));

    //---------------Subscribers---------------
    RCCHECK(rclc_subscription_init_default(
      &led_subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "pico_subscriber"));

    RCCHECK(rclc_subscription_init_default(
      &battery_subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
      "battery_state"));
    
    RCCHECK(rclc_subscription_init_default(
      &imu_subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "imu"));
    //------------------------------------------

    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "pico_publisher"));

    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        timer_callback));

    RCCHECK(rclc_executor_init(&executor_pub, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));

    RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor_sub, &led_subscriber, &led_msg, &led_subscription_callback, ON_NEW_DATA));
    
    gpio_put(LED_PIN, 1);

    msg_heartbeat.data = 0;

    while (true)
    {
        RCCHECK(rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(100)));
        RCCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100)));
        printData(st,voltage, roll, pitch, yaw, kidnap);
    }
    return 0;
}
