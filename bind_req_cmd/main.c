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
 * @defgroup zigbee_examples_light_switch main.c
 * @{
 * @ingroup zigbee_examples
 * @brief Dimmer switch for HA profile implementation.
 */

#include "zboss_api.h"
#include "zb_mem_config_custom.h"
#include "zb_error_handler.h"
#include "zigbee_helpers.h"

#include "app_timer.h"
#include "bsp.h"
#include "boards.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define IEEE_CHANNEL_MASK                   (1l << ZIGBEE_CHANNEL)              /**< Scan only one, predefined channel to find the coordinator. */
#define BIND_TEST_ENDPOINT                   1                                   /**< Source endpoint. */
#define MATCH_DESC_REQ_START_DELAY          (2 * ZB_TIME_ONE_SECOND)            /**< Delay between the config tool startup and multi sensor finding procedure. */
#define MATCH_DESC_REQ_TIMEOUT              (5 * ZB_TIME_ONE_SECOND)            /**< Timeout for finding procedure. */
#define MATCH_DESC_REQ_ROLE                 ZB_NWK_BROADCAST_ALL_DEVICES        /**< Find all devices. */
#define ERASE_PERSISTENT_CONFIG             ZB_FALSE                            /**< Do not erase NVRAM to save the network parameters after device reboot or power-off. NOTE: If this option is set to ZB_TRUE then do full device erase for all network devices before running other samples. */
#define ZIGBEE_NETWORK_STATE_LED            BSP_BOARD_LED_2                     /**< LED indicating that bind test successfully joind Zigbee network. */
#define BULB_FOUND_SENSOR                   BSP_BOARD_LED_3                     /**< LED indicating that bind test found a multi sensor. */
#define SLEEPY_ON_BUTTON                    BSP_BOARD_BUTTON_2                  /**< Button ID used to determine if we need the sleepy device behaviour (pressed means yes). */

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE to compile bind test (End Device) source code.
#endif


typedef struct multi_sensor_params_s
{
  zb_uint8_t  endpoint;
  zb_uint16_t short_addr;
  zb_addr_u nwk_addr;
} multi_sensor_params_t;

static zb_void_t find_multi_sensor_timeout(zb_bufid_t bufid);

static multi_sensor_params_t m_device_ctx;
static zb_uint8_t            m_attr_zcl_version   = ZB_ZCL_VERSION;
static zb_uint8_t            m_attr_power_source  = ZB_ZCL_BASIC_POWER_SOURCE_UNKNOWN;
static zb_uint16_t           m_attr_identify_time = 0;

/* Declare attribute list for Basic cluster. */
ZB_ZCL_DECLARE_BASIC_ATTRIB_LIST(basic_attr_list, &m_attr_zcl_version, &m_attr_power_source);

/* Declare attribute list for Identify cluster. */
ZB_ZCL_DECLARE_IDENTIFY_ATTRIB_LIST(identify_attr_list, &m_attr_identify_time);

/* Declare cluster list for Dimmer Switch device (Identify, Basic, Scenes, Groups, On Off, Level Control). */
/* Only clusters Identify and Basic have attributes. */
ZB_HA_DECLARE_CONFIGURATION_TOOL_CLUSTER_LIST(bind_test_clusters,
                                         basic_attr_list,
                                         identify_attr_list);

/* Declare endpoint for Dimmer Switch device. */
ZB_HA_DECLARE_CONFIGURATION_TOOL_EP(bind_test_ep,
                               BIND_TEST_ENDPOINT,
                               bind_test_clusters);

/* Declare application's device context (list of registered endpoints) for Dimmer Switch device. */
ZB_HA_DECLARE_CONFIGURATION_TOOL_CTX(bind_test_ctx, bind_test_ep);

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

static zb_void_t zb_bind_callback(zb_bufid_t bufid)
{
    zb_zdo_bind_resp_t * p_resp = (zb_zdo_bind_resp_t *)zb_buf_begin(bufid);

    if (p_resp->status == ZB_ZDP_STATUS_SUCCESS)
    {
        NRF_LOG_INFO("Bind ok");
    }
    else {
        /* Status can be found in zb_zdp_status_t */
        NRF_LOG_INFO("Error: Unable to modify binding. Status %d", p_resp->status);
    }
}

static zb_void_t bind_req()
{
    zb_ieee_addr_t               src_nwk_addr;
    zb_zdo_bind_req_param_t    * p_req;
    zb_bufid_t                   bufid;
    zb_ret_t                     zb_err_code;

    bufid = zb_buf_get_out();
    zb_osif_get_ieee_eui64(src_nwk_addr);
    
    
    p_req = ZB_BUF_GET_PARAM(bufid, zb_zdo_bind_req_param_t);
    ZB_MEMCPY(p_req->src_address, m_device_ctx.nwk_addr.addr_long, sizeof(zb_ieee_addr_t));
    p_req->src_endp = m_device_ctx.endpoint;
    p_req->cluster_id = ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT;
    p_req->dst_addr_mode = ZB_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
    ZB_MEMCPY(p_req->dst_address.addr_long, src_nwk_addr, sizeof(zb_ieee_addr_t));
    p_req->dst_endp = BIND_TEST_ENDPOINT;
    p_req->req_dst_addr = m_device_ctx.short_addr;

    zb_err_code = zb_zdo_bind_req(bufid, zb_bind_callback);
    ZB_ERROR_CHECK(zb_err_code);
}


/**@brief A callback called on EUI64 address response.
 *
 * @param[in] bufid Reference number to ZBOSS memory buffer.
 */
static zb_void_t zb_resolve_ieee_addr_cb(zb_bufid_t bufid)
{
    zb_zdo_ieee_addr_resp_t * p_resp = (zb_zdo_ieee_addr_resp_t *)zb_buf_begin(bufid);
    zb_char_t addr_buf[2 * 8 + 1];    /* 8 bytes (2 characters) plus one byte for null-terminator. */

    if (p_resp->status == ZB_ZDP_STATUS_SUCCESS)
    {
        ZB_LETOH64(m_device_ctx.nwk_addr.addr_long, p_resp->ieee_addr_remote_dev);
        UNUSED_RETURN_VALUE(ieee_addr_to_str(addr_buf, sizeof(addr_buf), m_device_ctx.nwk_addr.addr_long));
        NRF_LOG_INFO("Received EUI64 address for device 0x%04x -> %s.", m_device_ctx.short_addr, NRF_LOG_PUSH(addr_buf));
    }
    else
    {
        NRF_LOG_WARNING("Unable to resolve EUI64 source address. Status: %d\r\n",
                        p_resp->status);
    }

    zb_buf_free(bufid);
}


/**@brief Resolve EUI64 by sending IEEE address request.
 *
 * @param[in] bufid     Reference number to ZBOSS memory buffer.
 * @param[in] nwk_addr  Network address to be resolved.
 */
static zb_void_t zb_resolve_ieee_addr()
{
    zb_zdo_ieee_addr_req_param_t * p_req = NULL;
    zb_uint8_t                     tsn = 0;
    zb_bufid_t                     bufid;
    bufid = zb_buf_get_out();

    // Create new IEEE address request and fill with default values.
    p_req = ZB_BUF_GET_PARAM(bufid, zb_zdo_ieee_addr_req_param_t);
    p_req->start_index  = 0;
    p_req->request_type = 0;
    p_req->nwk_addr     = m_device_ctx.short_addr;
    p_req->dst_addr     = p_req->nwk_addr;
    tsn = zb_zdo_ieee_addr_req(bufid, zb_resolve_ieee_addr_cb);
    if (tsn == ZB_ZDO_INVALID_TSN)
    {
        NRF_LOG_WARNING("Failed to send IEEE address request for address: 0x%04x", m_device_ctx.short_addr);
        zb_buf_free(bufid);
    }
}


/**@brief Callback function receiving finding procedure results.
 *
 * @param[in]   bufid   Reference to Zigbee stack buffer used to pass received data.
 */
static zb_void_t find_multi_sensor_cb(zb_bufid_t bufid)
{
    zb_zdo_match_desc_resp_t   * p_resp = (zb_zdo_match_desc_resp_t *) zb_buf_begin(bufid);    // Get the beginning of the response
    zb_apsde_data_indication_t * p_ind  = ZB_BUF_GET_PARAM(bufid, zb_apsde_data_indication_t); // Get the pointer to the parameters buffer, which stores APS layer response
    zb_uint8_t                 * p_match_ep;
    zb_ret_t                     zb_err_code;

    if ((p_resp->status == ZB_ZDP_STATUS_SUCCESS) && (p_resp->match_len > 0) && (m_device_ctx.short_addr == 0xFFFF))
    {
        /* Match EP list follows right after response header */
        p_match_ep = (zb_uint8_t *)(p_resp + 1);

        /* We are searching for exact cluster, so only 1 EP may be found */
        m_device_ctx.endpoint   = *p_match_ep;
        m_device_ctx.short_addr = p_ind->src_addr;

        NRF_LOG_INFO("Found multi sensor addr: %d ep: %d", m_device_ctx.short_addr, m_device_ctx.endpoint);

        zb_err_code = ZB_SCHEDULE_APP_ALARM_CANCEL(find_multi_sensor_timeout, ZB_ALARM_ANY_PARAM);
        ZB_ERROR_CHECK(zb_err_code);
        
        bsp_board_led_on(BULB_FOUND_SENSOR);
        zb_resolve_ieee_addr();
        
    }

    if (bufid)
    {
        zb_buf_free(bufid);
    }
}

/**@brief Function for sending ON/OFF and Level Control find request.
 *
 * @param[in]   bufid   Non-zero reference to Zigbee stack buffer that will be used to construct find request.
 */
static zb_void_t find_multi_sensor(zb_bufid_t bufid)
{
    zb_zdo_match_desc_param_t * p_req;

    /* Initialize pointers inside buffer and reserve space for zb_zdo_match_desc_param_t request */
    p_req = zb_buf_initial_alloc(bufid, sizeof(zb_zdo_match_desc_param_t) + (1) * sizeof(zb_uint16_t));

    p_req->nwk_addr         = MATCH_DESC_REQ_ROLE;              // Send to devices specified by MATCH_DESC_REQ_ROLE
    p_req->addr_of_interest = MATCH_DESC_REQ_ROLE;              // Get responses from devices specified by MATCH_DESC_REQ_ROLE
    p_req->profile_id       = ZB_AF_HA_PROFILE_ID;              // Look for Home Automation profile clusters

    /* We are searching for 2 clusters: On/Off and Level Control Server */
    p_req->num_in_clusters  = 2;
    p_req->num_out_clusters = 0;
    /*lint -save -e415 // Suppress warning 415 "likely access of out-of-bounds pointer" */
    p_req->cluster_list[0]  = ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT;
    p_req->cluster_list[1]  = ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT;
    /*lint -restore */
    m_device_ctx.short_addr = 0xFFFF; // Set 0xFFFF to reset short address in order to parse only one response.
    UNUSED_RETURN_VALUE(zb_zdo_match_desc_req(bufid, find_multi_sensor_cb));
}

/**@brief Finding procedure timeout handler.
 *
 * @param[in]   bufid   Reference to Zigbee stack buffer that will be used to construct find request.
 */
static zb_void_t find_multi_sensor_timeout(zb_bufid_t bufid)
{
    zb_ret_t zb_err_code;

    if (bufid)
    {
        NRF_LOG_INFO("Multi sensor not found, try again");
        zb_err_code = ZB_SCHEDULE_APP_ALARM(find_multi_sensor, bufid, MATCH_DESC_REQ_START_DELAY);
        ZB_ERROR_CHECK(zb_err_code);
        zb_err_code = ZB_SCHEDULE_APP_ALARM(find_multi_sensor_timeout, 0, MATCH_DESC_REQ_TIMEOUT);
        ZB_ERROR_CHECK(zb_err_code);
    }
    else
    {
        zb_err_code = zb_buf_get_out_delayed(find_multi_sensor_timeout);
        ZB_ERROR_CHECK(zb_err_code);
    }
}

/**@brief Callback for button events.
 *
 * @param[in]   evt      Incoming event from the BSP subsystem.
 */
static void buttons_handler(bsp_event_t evt)
{
    //zb_ret_t zb_err_code;

    /* Inform default signal handler about user input at the device. */
    user_input_indicate();

    if (m_device_ctx.short_addr == 0xFFFF)
    {
        /* No bulb found yet. */
        return;
    }

    switch(evt)
    {
        case BSP_EVENT_KEY_0:
            break;

        case BSP_EVENT_KEY_1:
            bind_req();
            break;

        default:
            NRF_LOG_INFO("Unhandled BSP Event received: %d", evt);
            return;
    }
}

/**@brief Function for initializing LEDs and buttons.
 */
static zb_void_t leds_buttons_init(void)
{
    ret_code_t error_code;

    /* Initialize LEDs and buttons - use BSP to control them. */
    error_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, buttons_handler);
    APP_ERROR_CHECK(error_code);
    /* By default the bsp_init attaches BSP_KEY_EVENTS_{0-4} to the PUSH events of the corresponding buttons. */

    bsp_board_leds_off();
}

/**@brief Function to set the Sleeping Mode according to the SLEEPY_ON_BUTTON state.
*/
static zb_void_t sleepy_device_setup(void)
{
    zb_set_rx_on_when_idle(bsp_button_is_pressed(SLEEPY_ON_BUTTON) ? ZB_FALSE : ZB_TRUE);

#if ! defined DISABLE_POWER_CONSUMPTION_OPTIMIZATION
    /* If sleepy behaviour is enabled, power off unused RAM to save maximum energy */
    if (ZB_PIBCACHE_RX_ON_WHEN_IDLE() == ZB_FALSE)
    {
        zigbee_power_down_unused_ram();
    }
#endif /* ! defined DISABLE_POWER_CONSUMPTION_OPTIMIZATION */
}

/**@brief Zigbee stack event handler.
 *
 * @param[in]   bufid   Reference to the Zigbee stack buffer used to pass signal.
 */
void zboss_signal_handler(zb_bufid_t bufid)
{
    zb_zdo_app_signal_hdr_t      * p_sg_p = NULL;
    zb_zdo_app_signal_type_t       sig    = zb_get_app_signal(bufid, &p_sg_p);
    zb_ret_t                       status = ZB_GET_APP_SIGNAL_STATUS(bufid);
    zb_ret_t                       zb_err_code;

    /* Update network status LED */
    zigbee_led_status_update(bufid, ZIGBEE_NETWORK_STATE_LED);

    switch(sig)
    {
        case ZB_BDB_SIGNAL_DEVICE_REBOOT:
            /* fall-through */
        case ZB_BDB_SIGNAL_STEERING:
            /* Call default signal handler. */
            ZB_ERROR_CHECK(zigbee_default_signal_handler(bufid));
            if (status == RET_OK)
            {
                /* Check the multi sensor device address */
                if (m_device_ctx.short_addr == 0xFFFF)
                {
                    zb_err_code = ZB_SCHEDULE_APP_ALARM(find_multi_sensor, bufid, MATCH_DESC_REQ_START_DELAY);
                    ZB_ERROR_CHECK(zb_err_code);
                    zb_err_code = ZB_SCHEDULE_APP_ALARM(find_multi_sensor_timeout, 0, MATCH_DESC_REQ_TIMEOUT);
                    ZB_ERROR_CHECK(zb_err_code);
                    bufid = 0; // Do not free buffer - it will be reused by find_multi_sensor callback
                }
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


/**@brief Function for application main entry.
 */
int main(void)
{
    zb_ret_t       zb_err_code;
    zb_ieee_addr_t ieee_addr;

    /* Initialize timers, loging system and GPIOs. */
    timers_init();
    log_init();
    leds_buttons_init();

    /* Set Zigbee stack logging level and traffic dump subsystem. */
    ZB_SET_TRACE_LEVEL(ZIGBEE_TRACE_LEVEL);
    ZB_SET_TRACE_MASK(ZIGBEE_TRACE_MASK);
    ZB_SET_TRAF_DUMP_OFF();

    /* Initialize Zigbee stack. */
    ZB_INIT("bind test");

    /* Set device address to the value read from FICR registers. */
    zb_osif_get_ieee_eui64(ieee_addr);
    zb_set_long_address(ieee_addr);

    zb_set_network_ed_role(IEEE_CHANNEL_MASK);
    zigbee_erase_persistent_storage(ERASE_PERSISTENT_CONFIG);

    zb_set_ed_timeout(ED_AGING_TIMEOUT_64MIN);
    zb_set_keepalive_timeout(ZB_MILLISECONDS_TO_BEACON_INTERVAL(3000));
    sleepy_device_setup();

    /* Initialize application context structure. */
    UNUSED_RETURN_VALUE(ZB_MEMSET(&m_device_ctx, 0, sizeof(bind_test_ctx)));

    /* Set default bulb short_addr. */
    m_device_ctx.short_addr = 0xFFFF;

    /* Register dimmer switch device context (endpoints). */
    ZB_AF_REGISTER_DEVICE_CTX(&bind_test_ctx);

    /** Start Zigbee Stack. */
    zb_err_code = zboss_start_no_autostart();
    ZB_ERROR_CHECK(zb_err_code);

    while(1)
    {
        zboss_main_loop_iteration();
        UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
    }
}


/**
 * @}
 */
