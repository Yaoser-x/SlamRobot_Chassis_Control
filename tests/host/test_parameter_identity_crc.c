#include "parameter_identity_crc.h"

#include <assert.h>
#include <stdio.h>

int main(void)
{
    param_model_t first = {0};
    param_model_t same;
    uint32_t      baseline;

    first.version               = PARAM_MODEL_VERSION;
    first.max_linear_mps        = 0.5f;
    first.motor_dir[0]          = -1;
    first.line_threshold_raw[0] = 1234U;
    same                        = first;
    baseline                    = ParameterIdentityCrc_Calculate(&first);
    assert(baseline != 0UL);
    assert(ParameterIdentityCrc_Calculate(&same) == baseline);
    same.max_linear_mps = 0.6f;
    assert(ParameterIdentityCrc_Calculate(&same) != baseline);
    puts("PASS: canonical parameter identity CRC");
    return 0;
}
