/*
 * DWIN Unified Core – Arduino/DGUS/T5L helper library
 * ----------------------------------------------------
 * Author: Recep Senbas
 * Version: 1.0.0
 */
#include "dwin.h"
#if defined(ARDUINO_ARCH_AVR) || defined(__AVR__)
#include <avr/pgmspace.h>
#endif

/* -------- CRC16 (MODBUS/IBM) init=0xFFFF, poly=0xA001, LSB-first -------- */
static uint16_t dwin_crc16_t5l(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i];
        for (uint8_t b = 0; b < 8; b++)
        {
            uint16_t lsb = (uint16_t)(crc & 1u);
            crc >>= 1;
            if (lsb)
                crc ^= 0xA001u;
        }
    }
    return crc;
}

/* -------- internal log helper -------- */
static inline void log_if(Dwin *ctx, const char *msg)
{
    if (ctx && ctx->log)
        ctx->log(msg);
}

/* -------- init & callbacks -------- */
int dwin_init(Dwin *ctx,
              dwin_write_fn write,
              dwin_read_fn read,
              dwin_available_fn available,
              dwin_millis_fn millis,
              dwin_log_fn log)
{
    if (!ctx || !millis)
        return -1;
    ctx->write = write;
    ctx->read = read;
    ctx->available = available;
    ctx->millis = millis;
    ctx->log = log;

    ctx->on_frame = 0;
    ctx->on_touch = 0;

    ctx->rx_len = 0;
    ctx->rx_target = 0;
    ctx->sync = 0;
    ctx->last_rx_ms = 0;
    ctx->timeout_ms = 200;

    ctx->q_head = 0;
    ctx->q_tail = 0;
    ctx->use_isr_queue = 0;
    ctx->use_crc = 0; /* default: CRC disabled */
    return 0;
}

void dwin_set_frame_callback(Dwin *ctx, dwin_frame_cb cb)
{
    if (ctx)
        ctx->on_frame = cb;
}
void dwin_set_touch_callback(Dwin *ctx, dwin_touch_cb cb)
{
    if (ctx)
        ctx->on_touch = cb;
}

void dwin_use_isr_queue(Dwin *ctx, uint8_t enable)
{
    if (ctx)
        ctx->use_isr_queue = (enable ? 1u : 0u);
}

/* optional timer tick assist (no-op by default) */
void dwin_tick_isr(Dwin *ctx, uint16_t ms_inc)
{
    (void)ctx;
    (void)ms_inc;
}

/* internal busy-wait delay using ctx->millis() (dependency-free) */
// static void dwin_delay_ms(Dwin *ctx, uint16_t ms)
// {
//     if (!ctx || !ctx->millis || ms == 0)
//         return;
//     uint32_t t0 = ctx->millis();
//     while ((uint16_t)(ctx->millis() - t0) < ms)
//     { /* spin */
//     }
// }

/* -------- low-level send (1-byte LEN) --------
 * Frame: [5A][A5][LEN][CMD][PAYLOAD...][CRC_L][CRC_H?]
 * TX kuralı:
 *   LEN = CMD(1) + PAYLOAD(N) + (use_crc ? 2 : 0)
 *   CRC = CRC16(MODBUS) sadece CMD+PAYLOAD üstünden (T5L TX)
 */
int dwin_send(Dwin *ctx, uint8_t cmd, const uint8_t *payload, size_t plen)
{
    if (!ctx || !ctx->write)
        return -1;

    uint8_t frame[DWIN_MAX_FRAME];
    size_t idx = 0;
    frame[idx++] = DWIN_HDR_0;
    frame[idx++] = DWIN_HDR_1;

    size_t len_inside = 1 + plen + (ctx->use_crc ? 2u : 0u); /* LEN */
    size_t total = 3 + len_inside;                           /* hdr(2)+len(1)+LEN */
    if (total > DWIN_MAX_FRAME)
        return -2;

    frame[idx++] = (uint8_t)len_inside; /* LEN */
    frame[idx++] = cmd;                 /* CMD */

    for (size_t i = 0; i < plen; i++)
        frame[idx++] = payload[i];

    if (ctx->use_crc)
    {
        /* T5L TX rule: CRC over CMD + PAYLOAD only */
        uint16_t crc = dwin_crc16_t5l(&frame[3], (size_t)(1 + plen));
        frame[idx++] = (uint8_t)(crc & 0xFFu);        /* CRC_L */
        frame[idx++] = (uint8_t)((crc >> 8) & 0xFFu); /* CRC_H */
    }

    size_t written = ctx->write(frame, idx);
    return (written == idx) ? (int)idx : -3;
}

/* -------- high-level helpers -------- */

int dwin_write_vp(Dwin *ctx, uint16_t vp_addr, const uint8_t *data, size_t len)
{
    if (!ctx || !data || len == 0)
        return -1;
    if (len + 2 + 4 > DWIN_MAX_FRAME)
        return -2; /* vp(2) + header */

    uint8_t p[DWIN_MAX_FRAME];
    p[0] = (uint8_t)(vp_addr >> 8);
    p[1] = (uint8_t)(vp_addr & 0xFF);
    for (size_t i = 0; i < len; i++)
        p[2 + i] = data[i];

    return dwin_send(ctx, DWIN_CMD_WRITE_VP, p, 2 + len);
}

/* READ_VP request (this panel): payload = VP_H, VP_L, WORDS (1 byte) */
int dwin_read_vp(Dwin *ctx, uint16_t vp_addr, uint16_t len)
{
    /* Some DWIN firmwares expect 1‑byte WORD count in the request (matches your example: 5A A5 04 83 40 00 02). */
    if (!ctx || len == 0)
        return -1;
    uint8_t p[3];
    p[0] = (uint8_t)(vp_addr >> 8);
    p[1] = (uint8_t)(vp_addr & 0xFF);
    p[2] = (uint8_t)(len & 0xFF); /* WORDS (1 byte) */
    return dwin_send(ctx, DWIN_CMD_READ_VP, p, 3);
}

int dwin_write_u16(Dwin *ctx, uint16_t vp_addr, uint16_t value)
{
    uint8_t b[2];
    dwin_put_be16(b, value);
    return dwin_write_vp(ctx, vp_addr, b, 2);
}

int dwin_write_u32(Dwin *ctx, uint16_t vp_addr, uint32_t value)
{
    uint8_t b[4];
    dwin_put_be32(b, value);
    return dwin_write_vp(ctx, vp_addr, b, 4);
}

int dwin_sp_set_xy(Dwin *ctx, uint16_t sp_addr, uint16_t x, uint16_t y)
{
    /* Many DGUS/T5L widgets accept X,Y as two 16‑bit words at their SP/VP.
       Our u32 write uses big‑endian, so ((X<<16)|Y) yields [X_hi X_lo Y_hi Y_lo]. */
    uint32_t packed = ((uint32_t)x << 16) | (uint32_t)y;
    return dwin_write_u32(ctx, sp_addr + 1u, packed);
}

void dwin_beep_1s(Dwin *ctx)
{
    dwin_write_u16(ctx, 0x00A0, 0x007D);
}

int dwin_set_visibility(Dwin *ctx, uint16_t sp_addr, uint16_t vp_addr, int show)
{
    if (show)
    {
        return dwin_write_u16(ctx, sp_addr, vp_addr);
    }
    else
    {
        return dwin_write_u16(ctx, sp_addr, 0xFF00);
    }
}
/* Color 565: common convention uses SP+0x0003 */
int dwin_set_color565(Dwin *ctx, uint16_t sp_base_addr, uint16_t rgb565)
{
    uint16_t addr = (uint16_t)(sp_base_addr + 0x0003u);
    uint8_t b[2] = {(uint8_t)(rgb565 >> 8), (uint8_t)(rgb565 & 0xFF)};
    return dwin_write_vp(ctx, addr, b, 2);
}

int dwin_write_float(Dwin *ctx, uint16_t vp_addr, float value)
{
    union
    {
        float f;
        uint32_t u32;
    } conv;
    conv.f = value;
    uint8_t b[4];
    dwin_put_be32(b, conv.u32);
    return dwin_write_vp(ctx, vp_addr, b, 4);
}

int dwin_write_text(Dwin *ctx, uint16_t vp_addr, const char *text)
{
    if (!ctx || !text)
        return -1;

    const uint8_t *s = (const uint8_t *)text; /* RAM string */
    uint16_t addr = vp_addr;
    uint8_t buf[128]; /* Option B: 128‑byte chunk */
    int total = 0;

    for (;;)
    {
        uint8_t n = 0;
        while (n < sizeof(buf))
        {
            uint8_t c = *s++;
            if (c == 0)
                break;
            buf[n++] = c;
        }
        if (n)
        {
            int rc = dwin_write_vp(ctx, addr, buf, n);
            if (rc < 0)
                return rc;
            addr = (uint16_t)(addr + n);
            total += n;
        }
        if (n < sizeof(buf))
            break; /* hit NUL or finished last chunk */
    }
    return total; /* total bytes written */
}
int dwin_write_text_P(Dwin *ctx, uint16_t vp_addr, const char *progmem_ptr)
{
    if (!ctx || !progmem_ptr)
        return -1;

    uint16_t addr = vp_addr;
    uint8_t buf[128];
    int total = 0;

#if defined(ARDUINO_ARCH_AVR) || defined(__AVR__)
    /* AVR: PROGMEM’den pgm_read_byte ile chunk chunk oku */
    const char *p = progmem_ptr;
    for (;;)
    {
        uint8_t n = 0;
        while (n < sizeof(buf))
        {
            uint8_t c = pgm_read_byte(p++);
            if (c == 0)
                break;
            buf[n++] = c;
        }
        if (n)
        {
            int rc = dwin_write_vp(ctx, addr, buf, n);
            if (rc < 0)
                return rc;
            addr = (uint16_t)(addr + n);
            total += n;
        }
        if (n < sizeof(buf))
            break; /* NUL’e ulaştık */
    }
#else
    /* ESP32/others: flash literal normal pointer gibi okunur */
    const uint8_t *s = (const uint8_t *)progmem_ptr;
    for (;;)
    {
        uint8_t n = 0;
        while (n < sizeof(buf))
        {
            uint8_t c = *s++;
            if (c == 0)
                break;
            buf[n++] = c;
        }
        if (n)
        {
            int rc = dwin_write_vp(ctx, addr, buf, n);
            if (rc < 0)
                return rc;
            addr = (uint16_t)(addr + n);
            total += n;
        }
        if (n < sizeof(buf))
            break;
    }
#endif
    return total;
}

int dwin_clear_text(Dwin *ctx, uint16_t vp_addr, size_t max_len)
{
    if (!ctx || max_len == 0)
        return -1;

    uint16_t addr = vp_addr;
    uint8_t zeros[128] = {0}; /* small chunk buffer */
    int total = 0;

    while (max_len)
    {
        uint8_t n = (max_len > sizeof(zeros)) ? (uint8_t)sizeof(zeros) : (uint8_t)max_len;
        int rc = dwin_write_vp(ctx, addr, zeros, n);
        if (rc < 0)
            return rc;
        addr = (uint16_t)(addr + n);
        total += n;
        max_len -= n;
    }
    return total;
}

int dwin_set_brightness(Dwin *ctx, uint8_t br)
{
    if (br > 100)
        br = 100;
    uint8_t b = br;
    return dwin_write_vp(ctx, DWIN_VP_BRIGHTNESS, &b, 1);
}

/* Restart: write 0x55,0xAA,0x5A,0xA5 to VP 0x0004 (example) */
int dwin_restart(Dwin *ctx)
{
    uint8_t payload[6];
    payload[0] = 0x00; /* VP_H */
    payload[1] = 0x04; /* VP_L */
    payload[2] = 0x55;
    payload[3] = 0xAA;
    payload[4] = 0x5A;
    payload[5] = 0xA5;
    return dwin_send(ctx, DWIN_CMD_WRITE_VP, payload, 6);
}

/* -------- parser (1-byte LEN) -------- */
static inline void dwin_reset_sync(Dwin *ctx)
{
    ctx->rx_len = 0;
    ctx->rx_target = 0;
    ctx->sync = 0;
}

/* Consume one byte into state machine. Returns 1 if a frame completed, else 0. */
static int dwin_consume_byte(Dwin *ctx, uint8_t b)
{
    switch (ctx->sync)
    {
    case 0: /* seek 0x5A */
        if (b == DWIN_HDR_0)
        {
            ctx->sync = 1;
            ctx->rx_buf[0] = b;
            ctx->rx_len = 1;
        }
        break;

    case 1: /* seek 0xA5 */
        if (b == DWIN_HDR_1)
        {
            ctx->sync = 2;
            ctx->rx_buf[1] = b;
            ctx->rx_len = 2;
        }
        else
        {
            ctx->sync = 0;
            ctx->rx_len = 0;
        }
        break;

    case 2: /* LEN byte, then CMD + PAYLOAD (+CRC?) */
        if (ctx->rx_len == 2)
        {
            /* LEN at index 2 */
            ctx->rx_buf[2] = b;
            ctx->rx_len = 3;
            size_t len_inside = (size_t)ctx->rx_buf[2];
            size_t total = 3u + len_inside; /* minimal target; may extend by +2 if CRC is appended */
            ctx->rx_target = total;         /* total = hdr(2) + len(1) + LEN */
        }
        else
        {
            ctx->rx_buf[ctx->rx_len++] = b;

            if (ctx->rx_target && ctx->rx_len >= ctx->rx_target)
            {
                if (!ctx->use_crc)
                {
                    return 1; /* CRC yoksa minimal hedef yeterli */
                }

                size_t len_inside = (size_t)ctx->rx_buf[2];
                size_t minimal_total = 3u + len_inside;

                if (ctx->rx_len == minimal_total)
                {
                    /* Olası 1: LEN CRC'yi içeriyor → son 2 bayt CRC */
                    if (len_inside >= 3u)
                    {
                        uint16_t got = (uint16_t)ctx->rx_buf[minimal_total - 2] | ((uint16_t)ctx->rx_buf[minimal_total - 1] << 8);
                        uint16_t expect_cmd = dwin_crc16_t5l(&ctx->rx_buf[3], (size_t)(len_inside - 2u)); /* CMD+PAYLOAD */
                        if (expect_cmd == got)
                            return 1; /* tamamlandı */
                    }
                    /* Olası 2: CRC LEN’den ayrı geliyor → +2 bekle */
                    ctx->rx_target = (size_t)(minimal_total + 2u);
                    return 0;
                }
                else if (ctx->rx_len >= minimal_total + 2u)
                {
                    return 1; /* LEN sonrası appended CRC ile tamamlandı */
                }
            }
        }
        break;
    }
    return 0;
}

static void dwin_process_frame(Dwin *ctx, const uint8_t *f, size_t len)
{
    if (len < 4)
        return;
    const uint8_t cmd = f[3];

    /* CRC check (permissive) */
    size_t parse_len = len;
    if (ctx->use_crc)
    {
        if (len < 5)
            return; /* at least hdr+len+cmd+crc */

        uint16_t got = (uint16_t)f[len - 2] | ((uint16_t)f[len - 1] << 8);

        /* A) HDR..LEN..CMD..PAYLOAD (excluding CRC) */
        uint16_t expect_full = dwin_crc16_t5l(f, (size_t)(len - 2u));
        int pass = (expect_full == got);

        size_t len_inside = (size_t)f[2];

        /* B1) CMD+PAYLOAD, CRC inside LEN → (len_inside - 2) */
        if (!pass && len_inside >= 3u)
        {
            uint16_t expect_cmd_in_len = dwin_crc16_t5l(&f[3], (size_t)(len_inside - 2u));
            if (expect_cmd_in_len == got)
                pass = 1;
        }
        /* B2) CMD+PAYLOAD, CRC appended after LEN → len_inside */
        if (!pass && len_inside >= 1u)
        {
            uint16_t expect_cmd_app = dwin_crc16_t5l(&f[3], len_inside);
            if (expect_cmd_app == got)
                pass = 1;
        }

        if (!pass)
        {
            log_if(ctx, "DWIN: CRC ERROR (accepts HDR..PAYLOAD or CMD+PAYLOAD), frame dropped");
            return;
        }
        /* For parser limits treat length as len-2 (strip CRC bytes) */
        parse_len = len - 2u;
    }

    if (ctx->on_frame)
        ctx->on_frame(ctx, f, len);

    /* READ_VP response (typical):
       [5A][A5][LEN][83][VP_H][VP_L][LEN_L][LEN_H][DATA...] */
    if (cmd == DWIN_CMD_READ_VP && ctx->on_touch)
    {
        if (parse_len >= 9)
        {
            uint16_t vp = ((uint16_t)f[4] << 8) | f[5];
            uint16_t dlen = (uint16_t)f[6] | ((uint16_t)f[7] << 8); /* LE */
            uint8_t ev = 0;
            if (dlen >= 1 && (size_t)(8 + dlen) <= parse_len)
            {
                ev = f[8]; /* first data byte often used as event */
            }
            ctx->on_touch(ctx, vp, ev);
        }
        else if (parse_len >= 7)
        {
            /* fallback for non‑standard payloads */
            uint16_t vp = ((uint16_t)f[4] << 8) | f[5];
            uint8_t ev = f[6];
            ctx->on_touch(ctx, vp, ev);
        }
    }
}

/* Get a byte either from ISR queue or from hooks */
static int dwin_try_get_byte(Dwin *ctx, uint8_t *out)
{
    /* ISR queue mode */
    if (ctx->use_isr_queue)
    {
        uint16_t tail = ctx->q_tail;
        if (tail == ctx->q_head)
            return 0; /* empty */
        *out = ctx->q_buf[tail];
        ctx->q_tail = (uint16_t)((tail + 1u) & (DWIN_RX_Q_SIZE - 1u));
        return 1;
    }

    /* Hook/poll mode */
    if (!ctx->available || !ctx->read)
        return 0;

    int avail = ctx->available();
    if (avail > 0)
    {
        uint8_t tmp;
        int r = ctx->read(&tmp, 1); /* non-blocking single byte */
        if (r > 0)
        {
            *out = tmp;
            return 1;
        }
        log_if(ctx, "DWIN: avail>0 but read()=0");
    }
    return 0;
}

void dwin_clean_rx_queue(Dwin *ctx)
{
    if (!ctx)
        return;

    /* Drop any pending bytes in the ISR RX ring (if used) */
    ctx->q_head = 0;
    ctx->q_tail = 0;

    /* Also reset the parser state in case we were mid‑frame */
    dwin_reset_sync(ctx);

    /* In hook/poll mode (no ISR queue), actively drain the source so
       stale bytes do not accumulate in the UART/SoftwareSerial buffer. */
    if (!ctx->use_isr_queue && ctx->available && ctx->read)
    {
        while (ctx->available() > 0)
        {
            uint8_t tmp;
            if (ctx->read(&tmp, 1) <= 0)
                break;
        }
    }
}

int dwin_poll(Dwin *ctx)
{
    if (!ctx)
        return -1;

    int frames = 0;
    uint8_t b;

    /* Byte'ları boşalt, her gelişte zaman damgasını güncelle */
    while (dwin_try_get_byte(ctx, &b))
    {
        ctx->last_rx_ms = ctx->millis();

        if (dwin_consume_byte(ctx, b))
        {
            /* frame tamamlandı */
            size_t flen = ctx->rx_len; /* resetlemeden önce sakla */
            dwin_process_frame(ctx, ctx->rx_buf, flen);
            dwin_reset_sync(ctx);
            return 1; /* poll başına 1 frame işleyelim (istenirse arttırılabilir) */
        }
    }

    /* Timeout kontrolü (byte akışı bittikten sonra) */
    if (ctx->rx_len > 0)
    {
        uint32_t now = ctx->millis();
        if ((uint16_t)(now - ctx->last_rx_ms) > ctx->timeout_ms)
        {
            log_if(ctx, "DWIN: RX timeout, resetting parser");
            dwin_reset_sync(ctx);
        }
    }

    return frames;
}

/* NOR Flash R/W via VP=0x0008
 * OP = 0xA5 (store/write to NOR), 0x5A (load/read from NOR)
 * nor_head_addr: 24-bit database head address (even; we mask LSB=0)
 * vp: variable space adresi (BE)
 * words: 16-bit kelime sayısı (BE). Örn 2 kelime = 4 byte.
 */
static int dwin_nor_rw_cmd(Dwin *ctx, uint8_t op, uint32_t nor_head_addr, uint16_t vp, uint16_t words)
{
    if (!ctx)
        return -1;
    if (words == 0)
        return -2;

    /* Doküman: head address even olmalı */
    nor_head_addr &= ~1u;

    uint8_t payload[10];
    /* VP = 0x0008 */
    payload[0] = (uint8_t)(DWIN_VP_NOR_FLASH_RW_CMD >> 8);
    payload[1] = (uint8_t)(DWIN_VP_NOR_FLASH_RW_CMD & 0xFF);

    /* 8-byte komut alanı */
    payload[2] = op;                                      /* 0xA5 write, 0x5A read */
    payload[3] = (uint8_t)((nor_head_addr >> 16) & 0xFF); /* NOR_HI  */
    payload[4] = (uint8_t)((nor_head_addr >> 8) & 0xFF);  /* NOR_MID */
    payload[5] = (uint8_t)(nor_head_addr & 0xFF);         /* NOR_LO  */
    payload[6] = (uint8_t)(vp >> 8);                      /* VP_H    */
    payload[7] = (uint8_t)(vp & 0xFF);                    /* VP_L    */
    payload[8] = (uint8_t)(words >> 8);                   /* WORDS_H */
    payload[9] = (uint8_t)(words & 0xFF);                 /* WORDS_L */

    /* Bu bir VP yazma komutudur (CMD=0x82), payload uzunluğu = 10 byte */
    return dwin_send(ctx, DWIN_CMD_WRITE_VP, payload, sizeof(payload));
}

int dwin_nor_store(Dwin *ctx, uint32_t nor_head_addr, uint16_t src_vp, uint16_t words)
{
    return dwin_nor_rw_cmd(ctx, 0xA5u, nor_head_addr, src_vp, words);
}
int dwin_nor_load(Dwin *ctx, uint32_t nor_head_addr, uint16_t dst_vp, uint16_t words)
{
    return dwin_nor_rw_cmd(ctx, 0x5Au, nor_head_addr, dst_vp, words);
}

int dwin_get_versions_now(Dwin *ctx, uint8_t *out_gui, uint8_t *out_os, uint16_t timeout_ms)
{
    if (!ctx)
        return -1;

    /* Send: 5A A5 04 83 00 0F 01 (READ_VP VP=0x000F, words=1) */
    int s = dwin_read_vp(ctx, 0x000F, 1);
    if (s < 0)
        return s;

    /* Simple blocking wait for the specific response frame. We parse only
       the version reply format: LEN byte counts CMD+VP(2)+WORDS(1)+DATA(2) = 6. */
    uint8_t buf[DWIN_MAX_FRAME];
    size_t n = 0;
    size_t need = 0;   /* 3 + LEN */
    uint8_t phase = 0; /* 0:seek 5A, 1:seek A5, 2:get LEN, 3:collect body */

    uint32_t t0 = ctx->millis();
    for (;;)
    {
        /* timeout? */
        uint32_t now = ctx->millis();
        if ((uint16_t)(now - t0) > timeout_ms)
        {
            log_if(ctx, "DWIN: version read timeout");
            return -2;
        }

        uint8_t b;
        if (!dwin_try_get_byte(ctx, &b))
        {
            /* no data right now; continue polling */
            continue;
        }

        switch (phase)
        {
        case 0: /* seek 0x5A */
            if (b == DWIN_HDR_0)
            {
                buf[0] = b;
                n = 1;
                phase = 1;
            }
            break;
        case 1: /* seek 0xA5 */
            if (b == DWIN_HDR_1)
            {
                buf[1] = b;
                n = 2;
                phase = 2;
            }
            else
            {
                n = 0;
                phase = 0;
            }
            break;
        case 2: /* LEN */
            buf[2] = b;
            n = 3;
            need = (size_t)(3u + b);
            phase = 3;
            break;
        case 3: /* body */
            if (n < sizeof(buf))
                buf[n++] = b;
            else
            {
                n = 0;
                phase = 0;
                break;
            }

            if (n == need || n == need + 2u)
            {
                /* Try to parse as READ_VP reply for VP=0x000F with 1-word payload */
                if (n >= 9 && buf[3] == 0x83 && buf[4] == 0x00 && buf[5] == 0x0F)
                {
                    /* This reply uses 1-byte WORD count: buf[6] */
                    uint8_t words = buf[6];
                    if (words >= 1)
                    {
                        uint8_t gui = buf[7];
                        uint8_t os = buf[8];

                        /* Print via logger if available */
                        // if (ctx->log)
                        // {
                        //     char line[48];
                        //     /* e.g., "GUI=0x61 OS=0x21" */
                        //     snprintf(line, sizeof(line), "GUI=0x%02X OS=0x%02X", gui, os);
                        //     ctx->log(line);
                        // }
                        if (out_gui)
                            *out_gui = gui;
                        if (out_os)
                            *out_os = os;
                        return 0;
                    }
                }
                /* Not our frame; reset and continue searching */
                n = 0;
                phase = 0;
                need = 0;
                break;
            }
            break;
        }
    }
}

/* ---- RTC helpers (VP=0x0010) -------------------------------------------
 * Data layout (8 bytes, HEX not BCD):
 *   D7=Year (0..0x63)  D6=Month (1..12)  D5=Day (1..31)  D4=Week(0..6; 0=Sun)
 *   D3=Hour (0..23)    D2=Minute (0..59) D1=Second (0..59) D0=0x00 (unused)
 */
static uint8_t dwin_weekday_0sun(uint16_t y, uint8_t m, uint8_t d)
{
    /* Sakamoto algorithm; returns 0=Sunday..6=Saturday */
    static const uint8_t t[12] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
    if (m < 3)
        y -= 1;
    return (uint8_t)((y + y / 4 - y / 100 + y / 400 + t[m - 1] + d) % 7);
}

int dwin_rtc_write(Dwin *ctx, uint16_t year, uint8_t month, uint8_t day,
                   uint8_t hour, uint8_t minute, uint8_t second)
{
    if (!ctx)
        return -1;
    /* Basic range checks */
    if (month < 1 || month > 12)
        return -2;
    if (day < 1 || day > 31)
        return -3;
    if (hour > 23)
        return -4;
    if (minute > 59)
        return -5;
    if (second > 59)
        return -6;

    uint8_t y2 = (uint8_t)(year % 100u);
    uint8_t w = dwin_weekday_0sun(year, month, day); /* 0=Sun..6=Sat */

    uint8_t data[8];
    data[0] = y2;     /* Year (00..63) two-digit */
    data[1] = month;  /* Month */
    data[2] = day;    /* Day */
    data[3] = w;      /* Week: 0=SUN..6=SAT */
    data[4] = hour;   /* Hour */
    data[5] = minute; /* Minute */
    data[6] = second; /* Second */
    data[7] = 0x00;   /* Reserved */

    return dwin_write_vp(ctx, DWIN_VP_RTC, data, sizeof(data));
}

int dwin_rtc_hw_set(Dwin *ctx, uint16_t year, uint8_t month, uint8_t day,
                    uint8_t hour, uint8_t minute, uint8_t second)
{
    if (!ctx)
        return -1;
    /* range checks similar to dwin_rtc_write */
    if (month < 1 || month > 12)
        return -2;
    if (day < 1 || day > 31)
        return -3;
    if (hour > 23)
        return -4;
    if (minute > 59)
        return -5;
    if (second > 59)
        return -6;

    uint8_t y2 = (uint8_t)(year % 100u); /* 0..99 in HEX */

    /* Arrange pairs across 009D..009F: [Y,M], [D,H], [Min,Sec] */
    uint8_t ym[2] = {y2, month};
    uint8_t dh[2] = {day, hour};
    uint8_t ms[2] = {minute, second};

    int rc;
    rc = dwin_write_vp(ctx, DWIN_VP_RTC_YM, ym, 2);
    if (rc < 0)
        return rc;
    rc = dwin_write_vp(ctx, DWIN_VP_RTC_DH, dh, 2);
    if (rc < 0)
        return rc;
    rc = dwin_write_vp(ctx, DWIN_VP_RTC_MS, ms, 2);
    if (rc < 0)
        return rc;

    /* Commit: 009C = 0x5A,0xA5 (one-shot) */
    const uint8_t key[2] = {0x5A, 0xA5};
    rc = dwin_write_vp(ctx, DWIN_VP_RTC_SET_KEY, key, 2);
    if (rc < 0)
        return rc;

    return 0;
}

int dwin_get_page_now(Dwin *ctx, uint16_t timeout_ms)
{
    if (!ctx)
        return -1;

    /* Send: 5A A5 04 83 00 14 01 (READ_VP VP=0x0014, words=1) */
    int s = dwin_read_vp(ctx, DWIN_VP_PIC_NOW, 1);
    if (s < 0)
        return s;

    /* Blocking wait for the specific reply. Accepts frames with or without
       trailing CRC (n == need  or  n == need+2). */
    uint8_t buf[DWIN_MAX_FRAME];
    size_t n = 0;
    size_t need = 0;   /* 3 + LEN */
    uint8_t phase = 0; /* 0:seek 5A, 1:seek A5, 2:get LEN, 3:collect */

    uint32_t t0 = ctx->millis();
    for (;;)
    {
        uint32_t now = ctx->millis();
        if ((uint16_t)(now - t0) > timeout_ms)
        {
            log_if(ctx, "DWIN: get_page timeout");
            return -2;
        }

        uint8_t b;
        if (!dwin_try_get_byte(ctx, &b))
        {
            continue; /* no data yet */
        }

        switch (phase)
        {
        case 0: /* seek 0x5A */
            if (b == DWIN_HDR_0)
            {
                buf[0] = b;
                n = 1;
                phase = 1;
            }
            break;
        case 1: /* seek 0xA5 */
            if (b == DWIN_HDR_1)
            {
                buf[1] = b;
                n = 2;
                phase = 2;
            }
            else
            {
                n = 0;
                phase = 0;
            }
            break;
        case 2: /* LEN */
            buf[2] = b;
            n = 3;
            need = (size_t)(3u + b);
            phase = 3;
            break;
        case 3: /* body */
            if (n < sizeof(buf))
                buf[n++] = b;
            else
            {
                n = 0;
                phase = 0;
                break;
            }

            if (n == need || n == need + 2u)
            {
                /* Expect: [5A A5 LEN 83 00 14 01  PP_H PP_L] (+ optional CRC) */
                if (n >= 9 && buf[3] == DWIN_CMD_READ_VP && buf[4] == (uint8_t)(DWIN_VP_PIC_NOW >> 8) && buf[5] == (uint8_t)(DWIN_VP_PIC_NOW & 0xFF))
                {
                    uint8_t words = buf[6];
                    if (words >= 1)
                    {
                        uint16_t page = ((uint16_t)buf[7] << 8) | buf[8];
                        return (int)page;
                    }
                }
                /* Not our frame; reset and continue */
                n = 0;
                phase = 0;
                need = 0;
                break;
            }
            break;
        }
    }
}

int dwin_set_page(Dwin *ctx, uint16_t page_id)
{
    if (!ctx)
        return -1;

    uint8_t payload[6];
    payload[0] = (uint8_t)(DWIN_VP_PIC_SET >> 8);   /* VP_H */
    payload[1] = (uint8_t)(DWIN_VP_PIC_SET & 0xFF); /* VP_L */
    payload[2] = 0x5A;                              /* D3: enable once */
    payload[3] = 0x01;                              /* D2: mode = page switch */
    payload[4] = (uint8_t)(page_id >> 8);           /* PAGE_H (BE) */
    payload[5] = (uint8_t)(page_id & 0xFF);         /* PAGE_L */

    /* CMD=0x82 WRITE_VP, payload length = 6 */
    return dwin_send(ctx, DWIN_CMD_WRITE_VP, payload, sizeof(payload));
}

int dwin_set_backlight_standby(Dwin *ctx, uint8_t on_brightness, uint8_t standby_brightness, uint16_t open_time_10ms)
{
    if (!ctx)
        return -1;
    if (on_brightness > 0x64)
        on_brightness = 0x64; /* clamp per doc */
    if (standby_brightness > 0x64)
        standby_brightness = 0x64;

    /* VP=0x0082 payload: [D3=on][D2=standby][D1=open_hi][D0=open_lo] */
    uint8_t data[4];
    data[0] = on_brightness;
    data[1] = standby_brightness;
    data[2] = (uint8_t)(open_time_10ms >> 8);
    data[3] = (uint8_t)(open_time_10ms & 0xFF);
    return dwin_write_vp(ctx, DWIN_VP_BRIGHTNESS, data, sizeof(data));
}

int dwin_graph_plot(Dwin *ctx, uint8_t channel, uint16_t value)
{
    if (!ctx)
        return -1;
    if (channel > 7)
        return -2;

    /* Build 5 words @0x0310: [5A A5] [01 00] [00 CH] [02 00] [VAL VAL] */
    uint8_t p[10] = {0x5A, 0xA5, 0x01, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00};
    p[4] = channel;                 /* word3 low byte = channel */
    p[6] = (uint8_t)(value >> 8);   /* word5 hi */
    p[7] = (uint8_t)(value & 0xFF); /* word5 lo */
    p[8] = (uint8_t)(value >> 8);   /* duplicate value (as in your C code) */
    p[9] = (uint8_t)(value & 0xFF);

    return dwin_write_vp(ctx, DWIN_VP_GRAPH_CMD, p, sizeof(p));
}

int dwin_graph_clear(Dwin *ctx, uint8_t channel)
{
    if (!ctx)
        return -1;

    if (channel == 8)
    {
        int rc = 0;
        for (uint8_t ch = 0; ch < 8; ++ch)
        {
            uint16_t vp = (uint16_t)(DWIN_VP_GRAPH_CH_BASE + ((uint16_t)ch * 2u));
            int r = dwin_write_u16(ctx, vp, 0x0000);
            if (r < 0)
                rc = r;
        }
        return rc;
    }

    if (channel > 7)
        return -2;
    uint16_t vp = (uint16_t)(DWIN_VP_GRAPH_CH_BASE + ((uint16_t)channel * 2u));
    return dwin_write_u16(ctx, vp, 0x0000);
}

int dwin_overlay_enable(Dwin *ctx, uint16_t overlay_page_id, uint8_t overlay_touch_only)
{
    if (!ctx)
        return -1;
    /* Build a multi‑VP write starting at 0x00E8:
       [VP=00E8][0x5A][mode][PAGE_H][PAGE_L] */
    uint8_t payload[6];
    payload[0] = (uint8_t)(DWIN_VP_PAGE_STACK_SW >> 8);   /* VP_H (0x00) */
    payload[1] = (uint8_t)(DWIN_VP_PAGE_STACK_SW & 0xFF); /* VP_L (0xE8) */
    payload[2] = 0x5A;                                    /* E8_H: enable once */
    payload[3] = overlay_touch_only ? 0x01 : 0x00;        /* E8_L: touch routing */
    payload[4] = (uint8_t)(overlay_page_id >> 8);         /* E9_H */
    payload[5] = (uint8_t)(overlay_page_id & 0xFF);       /* E9_L */
    return dwin_send(ctx, DWIN_CMD_WRITE_VP, payload, sizeof(payload));
}

int dwin_overlay_disable(Dwin *ctx)
{
    if (!ctx)
        return -1;
    uint8_t zero = 0x00; /* any value other than 0x5A in E8_H disables */
    return dwin_write_vp(ctx, DWIN_VP_PAGE_STACK_SW, &zero, 1);
}

int dwin_touch_disable_all(Dwin *ctx, uint16_t blank_overlay_page_id)
{
    /* Overlay a page that has NO touch widgets and route touches only to overlay → effectively disables all touches. */
    return dwin_overlay_enable(ctx, blank_overlay_page_id, 1);
}

int dwin_touch_enable_all(Dwin *ctx)
{
    /* Simply turn overlay off */
    return dwin_overlay_disable(ctx);
}

/* Blocking read of a single 16-bit word from VP, returns 0 on success. */
static int dwin_read_word_now(Dwin *ctx, uint16_t vp, uint16_t timeout_ms, uint8_t *out_hi, uint8_t *out_lo)
{
    if (!ctx)
        return -1;

    int s = dwin_read_vp(ctx, vp, 1); /* request 1 word */
    if (s < 0)
        return s;

    uint8_t buf[DWIN_MAX_FRAME];
    size_t n = 0, need = 0; /* total bytes = 3+LEN (or +2 for CRC) */
    uint8_t phase = 0;      /* 0:seek 5A, 1:seek A5, 2:LEN, 3:body */
    uint32_t t0 = ctx->millis();

    for (;;)
    {
        uint32_t now = ctx->millis();
        if ((uint16_t)(now - t0) > timeout_ms)
        {
            log_if(ctx, "DWIN: read_word timeout");
            return -2;
        }

        uint8_t b;
        if (!dwin_try_get_byte(ctx, &b))
            continue;

        switch (phase)
        {
        case 0:
            if (b == DWIN_HDR_0)
            {
                buf[0] = b;
                n = 1;
                phase = 1;
            }
            break;
        case 1:
            if (b == DWIN_HDR_1)
            {
                buf[1] = b;
                n = 2;
                phase = 2;
            }
            else
            {
                n = 0;
                phase = 0;
            }
            break;
        case 2:
            buf[2] = b;
            n = 3;
            need = (size_t)(3u + b);
            phase = 3;
            break;
        case 3:
            if (n < sizeof(buf))
                buf[n++] = b;
            else
            {
                n = 0;
                phase = 0;
                break;
            }
            if (n == need || n == need + 2u)
            {
                /* Expect: [5A A5 LEN 83 VP_H VP_L 01  HI LO] (+opt CRC) */
                if (n >= 9 && buf[3] == DWIN_CMD_READ_VP && buf[4] == (uint8_t)(vp >> 8) && buf[5] == (uint8_t)(vp & 0xFF))
                {
                    uint8_t words = buf[6];
                    if (words >= 1)
                    {
                        if (out_hi)
                            *out_hi = buf[7];
                        if (out_lo)
                            *out_lo = buf[8];
                        return 0;
                    }
                }
                /* not our frame */
                n = 0;
                phase = 0;
                need = 0;
                break;
            }
            break;
        }
    }
}

int dwin_set_font_size(Dwin *ctx, uint16_t sp_base_addr, uint8_t size_x)
{
    if (!ctx)
        return -1;

    /* Read current word at SP+0x0004: [Lib_ID (H)][FontSize (L)] */
    uint8_t lib_id = 0, cur_sz = 0;
    int rr = dwin_read_word_now(ctx, (uint16_t)(sp_base_addr + 0x0004u), 300, &lib_id, &cur_sz);
    if (rr < 0)
    {
        /* Fallback: if read fails, try writing via L-byte odd address (may work on some panels) */
        uint16_t odd = (uint16_t)(sp_base_addr + 0x0004u + 1u);
        return dwin_write_vp(ctx, odd, &size_x, 1);
    }

    /* Preserve Lib_ID (H), update Font size (L) */
    uint8_t word[2] = {lib_id, size_x};
    return dwin_write_vp(ctx, (uint16_t)(sp_base_addr + 0x0004u), word, 2);
}
