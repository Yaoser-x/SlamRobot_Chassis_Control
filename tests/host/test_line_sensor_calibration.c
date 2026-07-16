#include "line_sensor_calibration.h"

#include <stdio.h>
#include <stdlib.h>

static void require_int(int condition, const char *message)
{
    if (condition == 0)
    {
        (void)printf("FAIL: %s\n", message);
        exit(1);
    }
}

static void feed_constant(line_calibration_model_t *calibration, line_calibration_surface_t surface, uint16_t base)
{
    uint16_t raw[LINE_CALIBRATION_CHANNELS];

    require_int(LineSensorCalibration_Begin(calibration, surface, 4U) != 0U, "calibration collection starts");
    for (uint8_t sample = 0U; sample < 4U; ++sample)
    {
        for (uint8_t channel = 0U; channel < LINE_CALIBRATION_CHANNELS; ++channel)
        {
            raw[channel] = (uint16_t)(base + channel + sample);
        }
        LineSensorCalibration_Feed(calibration, raw);
    }
}

static void test_black_line_white_floor_calibrates_active_low(void)
{
    line_calibration_model_t calibration;
    uint16_t                 thresholds[LINE_CALIBRATION_CHANNELS];
    uint8_t                  active_low = 0U;

    LineSensorCalibration_Init(&calibration);
    feed_constant(&calibration, LINE_CALIBRATION_MODEL_SURFACE_FLOOR, 900U);
    feed_constant(&calibration, LINE_CALIBRATION_MODEL_SURFACE_LINE, 200U);
    require_int(LineSensorCalibration_Apply(&calibration, thresholds, &active_low) != 0U, "separated surfaces apply");
    require_int(active_low == 1U, "lower black line values select active-low polarity");
    require_int(thresholds[0] == 551U, "threshold is midpoint of sample means");
}

static void test_low_separation_is_rejected(void)
{
    line_calibration_model_t calibration;
    uint16_t                 thresholds[LINE_CALIBRATION_CHANNELS];
    uint8_t                  active_low;

    LineSensorCalibration_Init(&calibration);
    feed_constant(&calibration, LINE_CALIBRATION_MODEL_SURFACE_FLOOR, 500U);
    feed_constant(&calibration, LINE_CALIBRATION_MODEL_SURFACE_LINE, 480U);
    require_int(LineSensorCalibration_Apply(&calibration, thresholds, &active_low) == 0U,
                "ambiguous surfaces are rejected");
    require_int(calibration.fail_mask == 0xFFU, "all ambiguous channels are reported");
}

int main(void)
{
    test_black_line_white_floor_calibrates_active_low();
    test_low_separation_is_rejected();
    (void)printf("PASS: line calibration host tests\n");
    return 0;
}
