#pragma once

#include "frame_pool.h"

void ntrip_client_init(void);
int  ntrip_client_active_count(void);
void ntrip_client_publish(pool_frame_t *f);
