/* Link-smoke for the C ABI: include the header and call vfm_version / vfm_config_default. */
#include "CVardeFM/vardefm.h"
#include <stdio.h>
#include <string.h>

int main(void) {
    const char *ver = vfm_version();
    if (!ver || strncmp(ver, "VardeFM ", 8) != 0) {
        fprintf(stderr, "bad version: %s\n", ver ? ver : "(null)");
        return 1;
    }
    VfmConfig cfg;
    vfm_config_default(&cfg);
    if (cfg.mode != VFM_MODE_NARROW || cfg.max_level > 6) {
        fprintf(stderr, "bad defaults\n");
        return 1;
    }
    printf("%s\n", ver);
    return 0;
}
