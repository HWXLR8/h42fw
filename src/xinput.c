#include "xinput.h"
#include <string.h>

xinput_report xinput_data;

void xinput_init_report(void) {
  memset(&xinput_data, 0, sizeof(xinput_data));
  xinput_data.rid = 0x00;
  xinput_data.rsize = 0x14;
}
