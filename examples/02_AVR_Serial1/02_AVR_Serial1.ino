#include <Arduino.h>
#include "dwin.h"

/*
 * DWIN Unified — AVR boards with Serial1 (MEGA/Leonardo/Micro)
 * ------------------------------------------------------------
 * Load the HMI (panel side)(optional)
 *  This step is **not required** to use the library. The sample **DWIN_SET** image simply helps you
 *    - Download the HMI package (DWIN_SET) from Releases:
 *      https://github.com/recepsenbas/DWIN-Unified/releases/latest
 *    - Unzip; copy the folder **DWIN_SET/** to the **root** of a FAT32 microSD.
 *    - Power OFF the panel → insert the card → power ON → wait for update → remove card.
 *
 * Connection:
 *   - Dwin TX → RX1 (MEGA pin 19)
 *   - Dwin RX → TX1 (MEGA pin 18)
 *   - GND ↔ GND, Baud = 115200 (Display and sketch must match)
 *   ( For Leonardo/Micro, use Serial1 pins )
 *
 * CRC:
 *   - If CRC is active on Display: dwin_use_crc(&dwin, 1); (after init)
 *   - Otherwise keep it 0.
 */

static Dwin dwin;

static size_t tx_write(const uint8_t *d, size_t n) { return Serial1.write(d, n); }
static int rx_read(uint8_t *d, size_t)
{
    if (Serial1.available())
    {
        *d = Serial1.read();
        return 1;
    }
    return 0;
}
static int rx_avail() { return Serial1.available(); }
static uint32_t tick_ms() { return millis(); }
static void log_fn(const char *s) { Serial.println(s); }

// --- Touch callback ---
static void on_touch(Dwin *, uint16_t vp, uint8_t ev)
{
    Serial.print(F("[TOUCH] VP=0x"));
    Serial.print(vp, HEX);
    Serial.print(F(" EV="));
    Serial.println(ev);

    switch (vp)
    {
    case 0x2008:
        Serial.println(F("0x2008 touched!"));
        break;
    default:
        break;
    }
}

// --- Frame debug callback ---
static void on_frame(Dwin *, const uint8_t *f, size_t len)
{
    Serial.print(F("[FRAME] "));
    for (size_t i = 0; i < len; i++)
    {
        if (f[i] < 0x10)
            Serial.print('0');
        Serial.print(f[i], HEX);
        Serial.print(' ');
    }
    Serial.println();
}

void setup()
{
    Serial.begin(115200);  // Serial monitor
    Serial1.begin(115200); // DWIN Display

    dwin_init(&dwin, tx_write, rx_read, rx_avail, tick_ms, log_fn);
    dwin_use_isr_queue(&dwin, 0);
    dwin_use_crc(&dwin, 0); // set to 1 if needed

    /* Set touch and frame callbacks */
    dwin_set_touch_callback(&dwin, on_touch);
    dwin_set_frame_callback(&dwin, on_frame);

    // --- Minimal API tour ---
    dwin_set_page(&dwin, 0);                     // page 0
    dwin_set_brightness(&dwin, 80);              // 80% brightness
    dwin_write_u16(&dwin, 0x2000, 123);          // 16-bit
    dwin_write_u32(&dwin, 0x2002, 0x00112233UL); // 32-bit
    dwin_write_float(&dwin, 0x2006, 3.1415f);    // float
    dwin_write_text(&dwin, 0x5000, F("Hello DWIN!"));
    dwin_clear_text(&dwin, 0x5000, 100); // clear 100 bytes
    dwin_read_vp(&dwin, 0x2000, 1);      // read 1 word (reply via poll)

    int page = dwin_get_page_now(&dwin, 500);
    Serial1.print(F("Current page: "));
    Serial1.println(page);
    uint8_t gui = 0, os = 0;
    dwin_get_versions_now(&dwin, &gui, &os, 500);    // get GUI/OS versions
    dwin_rtc_write(&dwin, 2025, 10, 23, 12, 34, 56); // set display RTC

    dwin_set_color565(&dwin, 0x7000, 0xF800); // red
    dwin_sp_set_xy(&dwin, 0x7100, 100, 50);   // X=100, Y=50

    dwin_graph_plot(&dwin, 0, 500); // plot sample on CH0
    dwin_graph_clear(&dwin, 0);     // clear CH0

    // Optional:
    // dwin_overlay_enable(&dwin, 1, 1);  // overlay page=1 (touch-only)
    // dwin_overlay_disable(&dwin);
    // dwin_touch_disable_all(&dwin, 2);  // lock touches (blank page)
    // dwin_touch_enable_all(&dwin);
    // dwin_nor_store(&dwin, 0x000002, 0x6000, 2);
    // dwin_nor_load(&dwin,  0x000002, 0x6100, 2);
    // dwin_beep_1s(&dwin);
    // dwin_restart(&dwin);
}

void loop()
{
    dwin_poll(&dwin);
}