/**
 * Copyright (c) 2018 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup zigbee_examples_multi_sensor main.c
 * @{
 * @ingroup zigbee_examples
 * @brief Zigbee Pressure and Temperature sensor
 */

#include "zboss_api.h"
#include "zb_mem_config_med.h"
#include "zb_error_handler.h"
#include "zigbee_helpers.h"
#include "app_timer.h"
#include "bsp.h"
#include "boards.h"
#include "sensorsim.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "zb_multi_sensor.h"

#define IEEE_CHANNEL_MASK                  (1l << ZIGBEE_CHANNEL)               /**< Scan only one, predefined channel to find the coordinator. */
#define ERASE_PERSISTENT_CONFIG            ZB_FALSE                             /**< Do not erase NVRAM to save the network parameters after device reboot or power-off. */

#define ZIGBEE_NETWORK_STATE_LED           BSP_BOARD_LED_2                      /**< LED indicating that light switch successfully joind Zigbee network. */

#define MIN_TEMPERATURE_VALUE              0                                    /**< Minimum temperature value as returned by the simulated measurement function. */
#define MAX_TEMPERATURE_VALUE              4000                                 /**< Maximum temperature value as returned by the simulated measurement function. */
#define TEMPERATURE_VALUE_INCREMENT        50                                   /**< Value by which the temperature value is incremented/decremented for each call to the simulated measurement function. */
#define MIN_PRESSURE_VALUE                 700                                  /**< Minimum pressure value as returned by the simulated measurement function. */
#define MAX_PRESSURE_VALUE                 1100                                 /**< Maximum pressure value as returned by the simulated measurement function. */
#define PRESSURE_VALUE_INCREMENT           5                                    /**< Value by which the temperature value is incremented/decremented for each call to the simulated measurement function. */

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE to compile End Device source code.
#endif

static sensor_device_ctx_t m_dev_ctx;

ZB_ZCL_DECLARE_IDENTIFY_ATTRIB_LIST(identify_attr_list, &m_dev_ctx.identify_attr.identify_time);

ZB_ZCL_DECLARE_BASIC_ATTRIB_LIST_EXT(basic_attr_list,
                                     &m_dev_ctx.basic_attr.zcl_version,
                                     &m_dev_ctx.basic_attr.app_version,
                                     &m_dev_ctx.basic_attr.stack_version,
                                     &m_dev_ctx.basic_attr.hw_version,
                                     m_dev_ctx.basic_attr.mf_name,
                                     m_dev_ctx.basic_attr.model_id,
                                     m_dev_ctx.basic_attr.date_code,
                                     &m_dev_ctx.basic_attr.power_source,
                                     m_dev_ctx.basic_attr.location_id,
                                     &m_dev_ctx.basic_attr.ph_env,
                                     m_dev_ctx.basic_attr.sw_ver);


ZB_ZCL_DECLARE_TEMP_MEASUREMENT_ATTRIB_LIST(temperature_attr_list, 
                                            &m_dev_ctx.temp_attr.measure_value,
                                            &m_dev_ctx.temp_attr.min_measure_value, 
                                            &m_dev_ctx.temp_attr.max_measure_value, 
                                            &m_dev_ctx.temp_attr.tolerance);

ZB_ZCL_DECLARE_PRES_MEASUREMENT_ATTRIB_LIST(pressure_attr_list, 
                                            &m_dev_ctx.pres_attr.measure_value, 
                                            &m_dev_ctx.pres_attr.min_measure_value, 
                                            &m_dev_ctx.pres_attr.max_measure_value, 
                                            &m_dev_ctx.pres_attr.tolerance);

ZB_DECLARE_MULTI_SENSOR_CLUSTER_LIST(multi_sensor_clusters,
                                     basic_attr_list,
                                     identify_attr_list,
                                     temperature_attr_list,
                                     pressure_attr_list);

ZB_ZCL_DECLARE_MULTI_SENSOR_EP(multi_sensor_ep,
                               MULTI_SENSOR_ENDPOINT,
                               multi_sensor_clusters);

ZBOSS_DECLARE_DEVICE_CTX_1_EP(multi_sensor_ctx, multi_sensor_ep);


static sensorsim_cfg_t   m_temperature_sim_cfg;                                 /**< Temperature sensor simulator configuration. */
static sensorsim_state_t m_temperature_sim_state;                               /**< Temperature sensor simulator state. */
static sensorsim_cfg_t   m_pressure_sim_cfg;                                    /**< Pressure sensor simulator configuration. */
static sensorsim_state_t m_pressure_sim_state;                                  /**< Pressure sensor simulator state. */

APP_TIMER_DEF(zb_app_timer);


void configure_attr_reporting(zb_bufid_t bufid);

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    ret_code_t err_code;

    // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing all clusters attributes.
 */
static void multi_sensor_clusters_attr_init(void)
{
    /* Basic cluster attributes data */
    m_dev_ctx.basic_attr.zcl_version   = ZB_ZCL_VERSION;
    m_dev_ctx.basic_attr.app_version   = SENSOR_INIT_BASIC_APP_VERSION;
    m_dev_ctx.basic_attr.stack_version = SENSOR_INIT_BASIC_STACK_VERSION;
    m_dev_ctx.basic_attr.hw_version    = SENSOR_INIT_BASIC_HW_VERSION;

    /* Use ZB_ZCL_SET_STRING_VAL to set strings, because the first byte should
     * contain string length without trailing zero.
     *
     * For example "test" string wil be encoded as:
     *   [(0x4), 't', 'e', 's', 't']
     */
    ZB_ZCL_SET_STRING_VAL(m_dev_ctx.basic_attr.mf_name,
                          SENSOR_INIT_BASIC_MANUF_NAME,
                          ZB_ZCL_STRING_CONST_SIZE(SENSOR_INIT_BASIC_MANUF_NAME));

    ZB_ZCL_SET_STRING_VAL(m_dev_ctx.basic_attr.model_id,
                          SENSOR_INIT_BASIC_MODEL_ID,
                          ZB_ZCL_STRING_CONST_SIZE(SENSOR_INIT_BASIC_MODEL_ID));

    ZB_ZCL_SET_STRING_VAL(m_dev_ctx.basic_attr.date_code,
                          SENSOR_INIT_BASIC_DATE_CODE,
                          ZB_ZCL_STRING_CONST_SIZE(SENSOR_INIT_BASIC_DATE_CODE));

    m_dev_ctx.basic_attr.power_source = SENSOR_INIT_BASIC_POWER_SOURCE;

    ZB_ZCL_SET_STRING_VAL(m_dev_ctx.basic_attr.location_id,
                          SENSOR_INIT_BASIC_LOCATION_DESC,
                          ZB_ZCL_STRING_CONST_SIZE(SENSOR_INIT_BASIC_LOCATION_DESC));


    m_dev_ctx.basic_attr.ph_env = SENSOR_INIT_BASIC_PH_ENV;

    /* Identify cluster attributes data */
    m_dev_ctx.identify_attr.identify_time        = ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE;

    /* Temperature measurement cluster attributes data */
    m_dev_ctx.temp_attr.measure_value            = ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_UNKNOWN;
    m_dev_ctx.temp_attr.min_measure_value        = ZB_ZCL_ATTR_TEMP_MEASUREMENT_MIN_VALUE_MIN_VALUE;
    m_dev_ctx.temp_attr.max_measure_value        = ZB_ZCL_ATTR_TEMP_MEASUREMENT_MAX_VALUE_MAX_VALUE;
    m_dev_ctx.temp_attr.tolerance                = ZB_ZCL_ATTR_TEMP_MEASUREMENT_TOLERANCE_MAX_VALUE;

    /* Pressure measurement cluster attributes data */
    m_dev_ctx.pres_attr.measure_value            = ZB_ZCL_ATTR_PRES_MEASUREMENT_VALUE_UNKNOWN;
    m_dev_ctx.pres_attr.min_measure_value        = ZB_ZCL_ATTR_PRES_MEASUREMENT_MIN_VALUE_MIN_VALUE;
    m_dev_ctx.pres_attr.max_measure_value        = ZB_ZCL_ATTR_PRES_MEASUREMENT_MAX_VALUE_MAX_VALUE;
    m_dev_ctx.pres_attr.tolerance                = ZB_ZCL_ATTR_PRES_MEASUREMENT_TOLERANCE_MAX_VALUE;
}

/**@brief Function for initializing LEDs.
 */
static zb_void_t leds_init(void)
{
    ret_code_t error_code;

    /* Initialize LEDs and buttons - use BSP to control them. */
    error_code = bsp_init(BSP_INIT_LEDS, NULL);
    APP_ERROR_CHECK(error_code);

    bsp_board_leds_off();
}

/**@brief Function for handling nrf app timer.
 * 
 * @param[IN]   context   Void pointer to context function is called with.
 * 
 * @details Function is called with pointer to sensor_device_ep_ctx_t as argument.
 */
static void zb_app_timer_handler(void * context)
{
    zb_zcl_status_t zcl_status;
    static zb_int16_t new_temp_value, new_pres_value;

    /* Get new temperature measured value */
    new_temp_value = (zb_int16_t)sensorsim_measure(&m_temperature_sim_state, &m_temperature_sim_cfg);
    zcl_status = zb_zcl_set_attr_val(MULTI_SENSOR_ENDPOINT, 
                                     ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, 
                                     ZB_ZCL_CLUSTER_SERVER_ROLE, 
                                     ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, 
                                     (zb_uint8_t *)&new_temp_value, 
                                     ZB_FALSE);
    if(zcl_status != ZB_ZCL_STATUS_SUCCESS)
    {
        NRF_LOG_INFO("Set temperature value fail. zcl_status: %d", zcl_status);
    }

    /* Get new pressure measured value */
    new_pres_value = (zb_int16_t)sensorsim_measure(&m_pressure_sim_state, &m_pressure_sim_cfg);
    zcl_status = zb_zcl_set_attr_val(MULTI_SENSOR_ENDPOINT,
                                     ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT, 
                                     ZB_ZCL_CLUSTER_SERVER_ROLE, 
                                     ZB_ZCL_ATTR_PRES_MEASUREMENT_VALUE_ID, 
                                     (zb_uint8_t *)&new_pres_value, 
                                     ZB_FALSE);
    if(zcl_status != ZB_ZCL_STATUS_SUCCESS)
    {
        NRF_LOG_INFO("Set pressure value fail. zcl_status: %d", zcl_status);
    }
}

/**@brief Function for initializing the sensor simulators.
 */
static void sensor_simulator_init(void)
{
    m_temperature_sim_cfg.min          = MIN_TEMPERATURE_VALUE;
    m_temperature_sim_cfg.max          = MAX_TEMPERATURE_VALUE;
    m_temperature_sim_cfg.incr         = TEMPERATURE_VALUE_INCREMENT;
    m_temperature_sim_cfg.start_at_max = false;

    sensorsim_init(&m_temperature_sim_state, &m_temperature_sim_cfg);

    m_pressure_sim_cfg.min          = MIN_PRESSURE_VALUE;
    m_pressure_sim_cfg.max          = MAX_PRESSURE_VALUE;
    m_pressure_sim_cfg.incr         = PRESSURE_VALUE_INCREMENT;
    m_pressure_sim_cfg.start_at_max = false;

    sensorsim_init(&m_pressure_sim_state, &m_pressure_sim_cfg);
}


/**@brief Zigbee stack event handler.
 *
 * @param[in]   bufid   Reference to the Zigbee stack buffer used to pass signal.
 */
void zboss_signal_handler(zb_bufid_t bufid)
{
    zb_zdo_app_signal_hdr_t  * p_sg_p      = NULL;
    zb_zdo_app_signal_type_t   sig         = zb_get_app_signal(bufid, &p_sg_p);
    zb_ret_t                   status      = ZB_GET_APP_SIGNAL_STATUS(bufid);

    /* Update network status LED */
    zigbee_led_status_update(bufid, ZIGBEE_NETWORK_STATE_LED);

    switch (sig)
    {
        case ZB_BDB_SIGNAL_DEVICE_REBOOT:
            /* fall-through */
        case ZB_BDB_SIGNAL_STEERING:
            /* Call default signal handler. */
            ZB_ERROR_CHECK(zigbee_default_signal_handler(bufid));
            if (status == RET_OK)
            {
                ret_code_t err_code = app_timer_start(zb_app_timer, APP_TIMER_TICKS(1000), NULL);
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            /* Call default signal handler. */
            ZB_ERROR_CHECK(zigbee_default_signal_handler(bufid));
            break;
    }

    if (bufid)
    {
        zb_buf_free(bufid);
    }
}

void configure_attr_reporting(zb_bufid_t bufid)
{
    zb_uint8_t * cmd_ptr;
    zb_uint16_t dst_addr = 0x0000;
    zb_uint8_t dst_ep = 64;
    ZB_ZCL_GENERAL_INIT_CONFIGURE_REPORTING_CLI_REQ(bufid,
                                                    cmd_ptr,
                                                    ZB_ZCL_ENABLE_DEFAULT_RESPONSE);
    ZB_ZCL_GENERAL_ADD_RECV_REPORT_CONFIGURE_REPORTING_REQ(cmd_ptr,
                                                           ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
                                                           0);
    ZB_ZCL_GENERAL_SEND_CONFIGURE_REPORTING_REQ(bufid,
                                                cmd_ptr,
                                                dst_addr,
                                                ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
                                                dst_ep, 
                                                MULTI_SENSOR_ENDPOINT,
                                                ZB_AF_HA_PROFILE_ID, 
                                                ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, 
                                                NULL);
    NRF_LOG_INFO("Attribute reporting configured");
}

/**@brief Function for application main entry.
 */
int main(void)
{
    zb_ret_t       zb_err_code;
    ret_code_t     err_code;
    zb_ieee_addr_t ieee_addr;

    /* Initialize loging system and GPIOs. */
    timers_init();
    log_init();
    sensor_simulator_init();
    leds_init();

    /* Create Timer for reporting attribute */
    err_code = app_timer_create(&zb_app_timer, APP_TIMER_MODE_REPEATED, zb_app_timer_handler);
    APP_ERROR_CHECK(err_code);

    /* Set Zigbee stack logging level and traffic dump subsystem. */
    ZB_SET_TRACE_LEVEL(ZIGBEE_TRACE_LEVEL);
    ZB_SET_TRACE_MASK(ZIGBEE_TRACE_MASK);
    ZB_SET_TRAF_DUMP_OFF();

    /* Initialize Zigbee stack. */
    ZB_INIT("multi_sensor");

    /* Set device address to the value read from FICR registers. */
    zb_osif_get_ieee_eui64(ieee_addr);
    zb_set_long_address(ieee_addr);

    /* Set static long IEEE address. */
    zb_set_network_ed_role(IEEE_CHANNEL_MASK);
    zigbee_erase_persistent_storage(ERASE_PERSISTENT_CONFIG);

    zb_set_ed_timeout(ED_AGING_TIMEOUT_64MIN);
    zb_set_keepalive_timeout(ZB_MILLISECONDS_TO_BEACON_INTERVAL(3000));
    zb_set_rx_on_when_idle(ZB_FALSE);

    /* Initialize application context structure. */
    UNUSED_RETURN_VALUE(ZB_MEMSET(&m_dev_ctx, 0, sizeof(m_dev_ctx)));

    /* Register temperature sensor device context (endpoints). */
    ZB_AF_REGISTER_DEVICE_CTX(&multi_sensor_ctx);

    /* Initialize sensor device attibutes */
    multi_sensor_clusters_attr_init();

    /** Start Zigbee Stack. */
    zb_err_code = zboss_start_no_autostart();
    ZB_ERROR_CHECK(zb_err_code);
    int i=0;
    while(1)
    {
        zboss_main_loop_iteration();
        UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
        if(i==50000){
            NRF_LOG_INFO("Configure attribute reporting");
            zb_buf_get_out_delayed(configure_attr_reporting);
        }
        i++;
    }
}


/**
 * @}
 */
