#pragma once

// Debug to expose some function to use directly in main, 
// without RTIO & co

#include "vl53l8cx_api.h"
#include "vl53l8cx_platform.h"
#include "vl53l8cx_types.h"

static inline int test_read(const struct device *dev, uint8_t *buff, uint32_t size) {
    struct vl53l8cx_data *data = dev->data;
    int i = 0;
    int ret = 0;

    LOG_INF(" -- Start ranging");
    vl53l8cx_start_ranging(&data->vl53l8cx_private_config);

    while (true) 
    {
        uint8_t is_ready;
        vl53l8cx_check_data_ready(&data->vl53l8cx_private_config, &is_ready);
        if (is_ready) {
            VL53L8CX_ResultsData result_data;
            ret = vl53l8cx_get_ranging_data(&data->vl53l8cx_private_config, &result_data);
            LOG_INF("%d Got data: %d %d %d %d", 
                ret, result_data.distance_mm[0], result_data.distance_mm[1], result_data.distance_mm[2], result_data.distance_mm[3]);
            if (i>10) {
                break;
            }
            i++;
            //k_msleep(1000);
        }
        else {
            //LOG_INF("  not ready");
        }
        k_msleep(10);
    }



    LOG_INF(" -- Stop ranging");
    vl53l8cx_stop_ranging(&data->vl53l8cx_private_config);
    k_msleep(1000);
    return 0;
}