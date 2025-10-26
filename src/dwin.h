/*
 * DWIN Unified Core – Arduino/DGUS/T5L helper library
 * ----------------------------------------------------
 * Author: Recep Senbas
 * Version: 1.0.0
 *
 * Frame format (TX/RX):
 *   [0x5A][0xA5][LEN][CMD][PAYLOAD...] [CRC_L][CRC_H]  (CRC optional)
 *
 * Defaults in this library
 *   TX: LEN = 1 (CMD) + N (PAYLOAD) + (use_crc ? 2 : 0)
 *       CRC16(MODBUS) calculated over CMD+PAYLOAD only (matches T5L TX behavior).
 *   RX: CRC verification is permissive:
 *       - CRC may be over the whole frame (HDR..LEN..CMD..PAYLOAD) OR only CMD+PAYLOAD.
 *       - LEN may or may not include the CRC bytes. Both styles are accepted.
 */
#ifndef DWIN_H
#define DWIN_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stddef.h>
#include <stdint.h>
    /* DWIN protocol constants (1‑byte LEN framing) */

#define DWIN_HDR_0 0x5Au
#define DWIN_HDR_1 0xA5u

/* Common commands */
#define DWIN_CMD_WRITE_VP 0x82u
#define DWIN_CMD_READ_VP 0x83u
#define DWIN_VP_NOR_FLASH_RW_CMD 0x0008u
#define DWIN_VP_RTC 0x0010u           /* 8 bytes: Y(2-digit),M,D,Week(0=Sun),Hour,Min,Sec,0x00 */
#define DWIN_VP_BRIGHTNESS 0x0082u    /* 1 byte: 0..100 typical */
#define DWIN_VP_PIC_NOW 0x0014u       /* current page ID (read‑only), 1 word */
#define DWIN_VP_PIC_SET 0x0084u       /* PIC_Set: page ops */
#define DWIN_VP_PAGE_STACK_SW 0x00E8u /* Page stacking/overlay switch (E8_H/E8_L), next VP (0x00E9) holds overlay page ID */
/* ---- RTC_Set hardware RTC VPs (0x009C..0x009F) ---- */
#define DWIN_VP_RTC_SET_KEY 0x009Cu /* write 0x5A,0xA5 to commit RTC settings */
#define DWIN_VP_RTC_YM 0x009Du      /* [Year, Month] */
#define DWIN_VP_RTC_DH 0x009Eu      /* [Day, Hour] */
#define DWIN_VP_RTC_MS 0x009Fu      /* [Minute, Second] */

/* ---- Graph feature VPs (DGUS/T5L) ---- */
#define DWIN_VP_GRAPH_CMD 0x0310u     /* Graph command entry (expects 5 words) */
#define DWIN_VP_GRAPH_CH_BASE 0x0301u /* Graph channel base: 0..7 at 0x0301,0x0303,...0x030F */

#ifndef DWIN_MAX_FRAME
#define DWIN_MAX_FRAME 256
#endif

/* ISR RX queue (power-of-two for cheap modulo) */
#ifndef DWIN_RX_Q_SIZE
#define DWIN_RX_Q_SIZE 128u
#endif

    /* ---- Transport hooks (optional when ISR queue is enabled) ---- */
    typedef size_t (*dwin_write_fn)(const uint8_t *data, size_t len); /* blocking; return bytes written */
    typedef int (*dwin_read_fn)(uint8_t *data, size_t maxlen);        /* non-blocking; return 0..maxlen */
    typedef int (*dwin_available_fn)(void);                           /* bytes available to read */
    typedef uint32_t (*dwin_millis_fn)(void);                         /* monotonic ms tick */
    typedef void (*dwin_log_fn)(const char *msg);                     /* optional logger (can be NULL) */

    /* Forward decl */
    struct Dwin;

    typedef void (*dwin_frame_cb)(struct Dwin *ctx, const uint8_t *frame, size_t len);
    typedef void (*dwin_touch_cb)(struct Dwin *ctx, uint16_t vp, uint8_t event_code);

    /* ---- Context ---- */
    typedef struct Dwin
    {
        /* I/O hooks (ignored when use_isr_queue=1) */
        dwin_write_fn write;
        dwin_read_fn read;
        dwin_available_fn available;

        /* System services */
        dwin_millis_fn millis;
        dwin_log_fn log;

        /* Callbacks */
        dwin_frame_cb on_frame;
        dwin_touch_cb on_touch;

        /* Parser state (RX) */
        uint8_t rx_buf[DWIN_MAX_FRAME];
        size_t rx_len;       /* collected so far */
        size_t rx_target;    /* total frame length to expect */
        uint8_t sync;        /* 0=seek hdr0, 1=seek hdr1, 2=len+body */
        uint32_t last_rx_ms; /* for timeout */
        uint16_t timeout_ms; /* per-frame timeout (ms) */

        /* ISR RX queue (single-producer in ISR, single-consumer in poll) */
        volatile uint16_t q_head;
        volatile uint16_t q_tail;
        uint8_t q_buf[DWIN_RX_Q_SIZE];

        /* Feature flags */
        uint8_t use_isr_queue; /* 1: use ISR queue, ignore read/available */
        uint8_t use_crc;       /* 1: use CRC16 on TX/RX */
    } Dwin;

    /* ---- API ---- */
    /** Initialize context with I/O hooks and system callbacks. Returns 0 on success. */
    int dwin_init(Dwin *ctx,
                  dwin_write_fn write,         /* can be NULL if using ISR queue only */
                  dwin_read_fn read,           /* can be NULL if using ISR queue only */
                  dwin_available_fn available, /* can be NULL if using ISR queue only */
                  dwin_millis_fn millis,
                  dwin_log_fn log);

    void dwin_set_frame_callback(Dwin *ctx, dwin_frame_cb cb);
    void dwin_set_touch_callback(Dwin *ctx, dwin_touch_cb cb);

    /** Enable/disable ISR RX ring buffer mode (1=enable). */
    void dwin_use_isr_queue(Dwin *ctx, uint8_t enable);

    /* Enable/disable CRC16 (T5L-compatible TX rule: CRC over CMD+PAYLOAD) */
    static inline void dwin_use_crc(Dwin *ctx, uint8_t enable)
    {
        if (ctx)
            ctx->use_crc = (enable ? 1u : 0u);
    }

    /* Feed a byte from UART RX ISR (extremely fast; no blocking) */
    static inline void dwin_isr_feed(Dwin *ctx, uint8_t b)
    {
        if (!ctx || !ctx->use_isr_queue)
            return;
        uint16_t head = ctx->q_head;
        uint16_t next = (uint16_t)((head + 1u) & (DWIN_RX_Q_SIZE - 1u));
        if (next != ctx->q_tail)
        { /* drop if full */
            ctx->q_buf[head] = b;
            ctx->q_head = next;
        }
    }

    /* Optional: tick from a timer ISR to advance timebase finer than millis() */
    void dwin_tick_isr(Dwin *ctx, uint16_t ms_inc);

    /** Parse incoming bytes; call frequently from loop/task. Returns number of frames parsed. */
    int dwin_poll(Dwin *ctx);
    /** Clear the RX queue (ISR mode only). */
    void dwin_clean_rx_queue(Dwin *ctx);

    /** Low‑level send: packs a full DWIN frame with optional CRC. */
    int dwin_send(Dwin *ctx, uint8_t cmd, const uint8_t *payload, size_t plen);

    /* High-level helpers */
    /** Write arbitrary bytes to a VP address. */
    int dwin_write_vp(Dwin *ctx, uint16_t vp_addr, const uint8_t *data, size_t len);
    /** Read N words from a VP (this firmware expects 1‑byte word count). */
    int dwin_read_vp(Dwin *ctx, uint16_t vp_addr, uint16_t len);
    int dwin_write_u16(Dwin *ctx, uint16_t vp_addr, uint16_t value);
    int dwin_write_u32(Dwin *ctx, uint16_t vp_addr, uint32_t value);
    /**
     * Write object coordinates to a given SP/VP address.
     * Packs X and Y as two big‑endian 16‑bit words: [X_hi X_lo][Y_hi Y_lo].
     */
    int dwin_sp_set_xy(Dwin *ctx, uint16_t sp_addr, uint16_t x, uint16_t y);
    void dwin_beep_1s(Dwin *ctx);
    int dwin_set_visibility(Dwin *ctx, uint16_t sp_addr, uint16_t vp_addr, int show);
    int dwin_set_color565(Dwin *ctx, uint16_t sp_base_addr, uint16_t rgb565);
    int dwin_set_font_size(Dwin *ctx, uint16_t sp_base_addr, uint8_t size_x);
    int dwin_write_float(Dwin *ctx, uint16_t vp_addr, float value);
    /** Write a C string from RAM in small chunks (low SRAM); stops at NUL. */
#ifdef __cplusplus
    int dwin_write_text(Dwin *ctx, uint16_t vp_addr, const char *text);
    int dwin_write_text_P(Dwin *ctx, uint16_t vp_addr, const char *progmem_ptr);
#else
    int dwin_write_text(Dwin *ctx, uint16_t vp_addr, const char *text);
    int dwin_write_text_P(Dwin *ctx, uint16_t vp_addr, const char *progmem_ptr);
#endif
    int dwin_clear_text(Dwin *ctx, uint16_t vp_addr, size_t max_len);
    /** Set display brightness (0..100). */
    int dwin_set_brightness(Dwin *ctx, uint8_t br);
    /** Restart the panel via VP command. */
    int dwin_restart(Dwin *ctx);
    /** Store words from VP to NOR flash at a database address. */
    int dwin_nor_store(Dwin *ctx, uint32_t nor_head_addr, uint16_t src_vp, uint16_t words);
    /** Load words from NOR flash to VP. */
    int dwin_nor_load(Dwin *ctx, uint32_t nor_head_addr, uint16_t dst_vp, uint16_t words);
    /** Query GUI and OS version (blocking, returns 0 on success). */
    int dwin_get_versions_now(Dwin *ctx, uint8_t *out_gui, uint8_t *out_os, uint16_t timeout_ms);
    /** Write 8-byte software RTC (year, month, day, hour, min, sec). */
    int dwin_rtc_write(Dwin *ctx, uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second);
    /** Program hardware RTC (writes 009D..009F then commits via 009C=5A A5). */
    int dwin_rtc_hw_set(Dwin *ctx, uint16_t year, uint8_t month, uint8_t day,
                        uint8_t hour, uint8_t minute, uint8_t second);
    /** Read current page ID (blocking, returns page). */
    int dwin_get_page_now(Dwin *ctx, uint16_t timeout_ms);
    /** Set current page. */
    int dwin_set_page(Dwin *ctx, uint16_t page_id);
    /** Configure backlight standby/timeout. */
    int dwin_set_backlight_standby(Dwin *ctx, uint8_t on_brightness, uint8_t standby_brightness, uint16_t open_time_10ms);

    /* Graph helpers (DGUS/T5L) */
    /** Plot a value to a graph channel (0..7). */
    int dwin_graph_plot(Dwin *ctx, uint8_t channel, uint16_t value); /* channel 0..7 */
    /** Clear a graph channel (0..7), or all (8). */
    int dwin_graph_clear(Dwin *ctx, uint8_t channel); /* channel 0..7, 8=ALL */

    /* Page overlay controls (0x00E8 / 0x00E9). */
    /** Enable page overlay (0x00E8/0x00E9), optionally touch-only. */
    int dwin_overlay_enable(Dwin *ctx, uint16_t overlay_page_id, uint8_t overlay_touch_only);
    /** Disable overlay. */
    int dwin_overlay_disable(Dwin *ctx);

    /* Touch helpers */
    /** Disable all touches using overlay (by overlaying a blank page). */
    int dwin_touch_disable_all(Dwin *ctx, uint16_t blank_overlay_page_id);
    /** Enable all touches (remove overlay). */
    int dwin_touch_enable_all(Dwin *ctx);
    /* Utilities */
    static inline void dwin_put_be16(uint8_t *p, uint16_t v)
    {
        p[0] = (uint8_t)(v >> 8);
        p[1] = (uint8_t)v;
    }
    static inline void dwin_put_be32(uint8_t *p, uint32_t v)
    {
        p[0] = (uint8_t)(v >> 24);
        p[1] = (uint8_t)(v >> 16);
        p[2] = (uint8_t)(v >> 8);
        p[3] = (uint8_t)v;
    }

#ifdef __cplusplus
}
#endif
#ifdef __cplusplus
class __FlashStringHelper; /* forward */
static inline int dwin_write_text(Dwin *ctx, uint16_t vp_addr, const __FlashStringHelper *fs)
{
#if defined(ARDUINO_ARCH_AVR) || defined(__AVR__)
    return dwin_write_text_P(ctx, vp_addr, reinterpret_cast<const char *>(fs));
#else
    return dwin_write_text(ctx, vp_addr, reinterpret_cast<const char *>(fs));
#endif
}
#endif /* __cplusplus */
#endif /* DWIN_H */