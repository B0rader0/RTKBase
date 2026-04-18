#pragma once

#include <stdbool.h>

#include "frame_pool.h"

void ntrip_caster_init(void);
int  ntrip_caster_active_count(void);
void ntrip_caster_publish(pool_frame_t *f);
int  ntrip_caster_rover_count(void);
