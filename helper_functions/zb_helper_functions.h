#ifndef ZB_HELPER_FUNCTIONS_H__
#define ZB_HELPER_FUNCTIONS_H__

#include <stdint.h>
#include <stdbool.h>
#include "zboss_api.h"

/* Get neighbor table */
static zb_void_t get_neighbors();

/* Get LQI of all neighbors */
static zb_void_t get_lqi();

#endif /* ZB_HELPER_FUNCTIONS_H__ */
