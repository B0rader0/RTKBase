#include "include/rtk_base.h"
#include "esp_log.h"

static const char *TAG = "FRAME_POOL";

// ─── The pool itself ──────────────────────────────────────────────────────────
// Declared static: exists for the lifetime of the firmware, no heap involved.

static pool_frame_t s_pool[FRAME_POOL_SIZE];

// ─── pool_init ────────────────────────────────────────────────────────────────

void pool_init(void)
{
    for (int i = 0; i < FRAME_POOL_SIZE; i++) {
        atomic_store(&s_pool[i].ref_count, 0);
    }
    ESP_LOGI(TAG, "Frame pool ready: %d slots x %d bytes = %d bytes static RAM",
             FRAME_POOL_SIZE, (int)MAX_FRAME_SIZE,
             (int)(FRAME_POOL_SIZE * sizeof(pool_frame_t)));
}

// ─── pool_alloc ───────────────────────────────────────────────────────────────
// Scans for the first slot with ref_count == 0.
// Called only from the frame_splitter task, so no mutex is needed:
// ref_count transitions 0→N are done here (single writer),
// and N→0 transitions are done in consumer tasks via pool_release().
// The atomic load/store ensures visibility across cores.

pool_frame_t *pool_alloc(void)
{
    for (int i = 0; i < FRAME_POOL_SIZE; i++) {
        if (atomic_load(&s_pool[i].ref_count) == 0) {
            // Mark as tentatively in-use with ref_count = 1 so no other
            // path can grab this slot before pool_set_refs() is called.
            atomic_store(&s_pool[i].ref_count, 1);
            return &s_pool[i];
        }
    }
    // Pool exhausted: increase FRAME_POOL_SIZE or reduce FRAME_QUEUE_DEPTH.
    ESP_LOGE(TAG, "Pool exhausted! Increase FRAME_POOL_SIZE (currently %d)",
             FRAME_POOL_SIZE);
    return NULL;
}

// ─── pool_set_refs ────────────────────────────────────────────────────────────
// Call after pool_alloc(), before handing the pointer to any queue.
// Sets the final reference count (one per destination queue).

void pool_set_refs(pool_frame_t *f, int n)
{
    atomic_store(&f->ref_count, n);
}

// ─── pool_release ─────────────────────────────────────────────────────────────
// Called by each consumer task after it has finished with a frame.
// When the last reference is released, ref_count returns to 0 and the slot
// becomes available to pool_alloc() again.

void pool_release(pool_frame_t *f)
{
    int prev = atomic_fetch_sub(&f->ref_count, 1);
    if (prev <= 0) {
        // Should never happen; indicates a double-release bug.
        ESP_LOGE(TAG, "pool_release called on already-free slot!");
    }
    // When prev == 1, ref_count is now 0 → slot is free automatically.
}
