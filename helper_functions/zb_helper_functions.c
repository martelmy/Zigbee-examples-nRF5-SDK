#include "zboss_api.h"
#include "nrf_log.h"
#include "zb_helper_functions.h"

static zb_void_t get_lqi()
{
    uint32_t i;
    zb_uint16_t addr;

    /* If gc_neighbor[i].used is 0, the entry is not used */
    /* If gc_neighbor[i].ext_neighbor is 1, this is ext neighbor record, else base neighbor */
    for (i = 0; i < gc_neighbor_table_size; i++) {
        if ((gc_neighbor[i].used == 0) ||
            (gc_neighbor[i].ext_neighbor != 0))
        {
            continue;
        }
        zb_address_short_by_ref(&addr, gc_neighbor[i].u.base.addr_ref);
        NRF_LOG_INFO("short_addr: 0x%04x, #LQI: %d", addr, gc_neighbor[i].lqi);
    }
    
}

/* Get neighbor table */
static zb_void_t get_neighbors()
{
    uint32_t i;
    zb_uint16_t addr;

    for (i = 0; i < gc_neighbor_table_size; i++)
    {
        /* If gc_neighbor[i].used is 0, the entry is not used */
        /* If gc_neighbor[i].ext_neighbor is 1, this is ext neighbor record, else base neighbor */
        if ((gc_neighbor[i].used == 0) ||
            (gc_neighbor[i].ext_neighbor != 0))
        {
            continue;
        }
        
        zb_address_short_by_ref(&addr, gc_neighbor[i].u.base.addr_ref);
        NRF_LOG_INFO("short_addr: 0x%04x", addr);

        switch (gc_neighbor[i].device_type) {
            case ZB_NWK_DEVICE_TYPE_COORDINATOR:
                NRF_LOG_INFO("device type: ZC ");
                break;
 
            case ZB_NWK_DEVICE_TYPE_ROUTER:
                NRF_LOG_INFO("device type: ZR  ");
                break;
 
            case ZB_NWK_DEVICE_TYPE_ED:
                NRF_LOG_INFO("device type: ZED ");
                break;
 
            default:
                NRF_LOG_INFO("device type: ??? ");
                break;
        }

        if (gc_neighbor[i].device_type != ZB_NWK_DEVICE_TYPE_ED)
        {
            /* The number of nwkLinkStatusPeriod intervals since a link status command was received */
            NRF_LOG_INFO("age: %03u ", gc_neighbor[i].u.base.age);
        }
        else
        {
            /* Time left of ED timeout */
            zb_time_t exp_time = ZB_TIME_SUBTRACT(gc_neighbor[i].u.base.time_to_expire, ZB_TIMER_GET());
            NRF_LOG_INFO("timeout: %07u", ZB_TIME_BEACON_INTERVAL_TO_MSEC(exp_time) / 1000);
        }

        if ((gc_neighbor[i].device_type != ZB_NWK_DEVICE_TYPE_ED) &&
            (gc_neighbor[i].u.base.outgoing_cost == 0))
        {   /* Outgoing cost of router or coordinator neighbor entry being 0 means the entry is stale */
            NRF_LOG_INFO("[STALE]");
        }
    }
}