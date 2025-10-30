

#include <Arduino.h>
#include "dwin.h"

/*
 * DWIN Unified — Hardware Serial + ISR Queue (Arduino MEGA example)
 * =================================================================
 * Purpose: Demonstrate using a hardware UART (Serial1) while feeding
 *          the DWIN RX queue asynchronously via serialEvent1().
 *
 * Load the HMI (panel side)(optional) 
 *  This step is not required to use the library. The sample DWIN_SET image simply helps you
 *    - Download the HMI package (DWIN_SET) from Releases:
 *      https://github.com/recepsenbas/DWIN-Unified/releases/latest
 *    - Unzip; copy the folder DWIN_SET/ to the root of a FAT32 microSD.
 *    - Power OFF the panel → insert the card → power ON → wait for update → remove card.
 *
 *
 * Wiring (MEGA2560)
 *   - Display TX → RX1 (pin 19)
 *   - Display RX → TX1 (pin 18)
 *   - GND ↔ GND
 *
 * CRC on TX
 *   - If your HMI expects CRC on transmitted frames: call dwin_use_crc(&dwin, 1);
 *   - Otherwise keep it disabled (default 0).
 *
 * Notes
 *   - We enable the internal ISR queue: dwin_use_isr_queue(&dwin, 1).
 *   - Bytes are pushed from serialEvent1() using dwin_isr_feed(&dwin, byte).
 *   - The parser runs in loop() via dwin_poll(&dwin).
 */

static Dwin dwin;

// --- TX hook (HW UART) ---
static size_t tx_write(const uint8_t *d, size_t n)
{
    return Serial1.write(d, n);
}

// --- RX hooks are unused when ISR queue is enabled (return 0) ---
static int rx_read(uint8_t * /*d*/, size_t /*n*/) { return 0; }
static int rx_avail() { return 0; }

static uint32_t tick_ms() { return millis(); }
static void log_fn(const char *s) { Serial.println(s); }

// --- Optional callbacks ---
static void on_touch(Dwin *, uint16_t vp, uint8_t ev)
{
    Serial.print(F("[TOUCH] VP=0x"));
    Serial.print(vp, HEX);
    Serial.print(F(" EV="));
    Serial.println(ev);
}
static void on_frame(Dwin *, const uint8_t *f, size_t len)
{
#if 0
  Serial.print(F("[FRAME] "));
  for (size_t i=0;i<len;i++){ if(f[i]<0x10) Serial.print('0'); Serial.print(f[i],HEX); Serial.print(' ');} Serial.println();
#else
    (void)f;
    (void)len;
#endif
}

void setup()
{
    Serial.begin(115200);  // USB log
    Serial1.begin(115200); // DWIN display UART (set to 115200 if your HMI uses that)

    dwin_init(&dwin, tx_write, rx_read, rx_avail, tick_ms, log_fn);
    dwin_use_isr_queue(&dwin, 1); // <-- enable ISR queue mode
    dwin_use_crc(&dwin, 0);       // set to 1 if your HMI requires TX CRC

    dwin_set_touch_callback(&dwin, on_touch);
    dwin_set_frame_callback(&dwin, on_frame);

    // --- Minimal API tour (short, no long demos) ---
    dwin_set_page(&dwin, 0);
    dwin_set_brightness(&dwin, 80);

    dwin_write_u16(&dwin, 0x2000, 123);
    dwin_write_u32(&dwin, 0x2002, 0x00112233UL);
    dwin_write_text(&dwin, 0x5000, F("Hello from ISR queue!"));
    dwin_clear_text(&dwin, 0x5000, 100);

    int page = dwin_get_page_now(&dwin, 500);
    (void)page;
    uint8_t gui = 0, os = 0;
    dwin_get_versions_now(&dwin, &gui, &os, 500);

    dwin_set_color565(&dwin, 0x7000, 0xF800); // red
    dwin_sp_set_xy(&dwin, 0x7100, 100, 50);

    dwin_graph_plot(&dwin, 0, 500);
    dwin_graph_clear(&dwin, 0);
}

void loop()
{
    dwin_poll(&dwin); // consume bytes from ISR queue and parse frames
}

// --- Asynchronous RX feeder ---
// serialEvent1() is called between loop() iterations when Serial1 has data.
// We push all bytes into the library's RX queue.
void serialEvent1()
{
    while (Serial1.available())
    {
        uint8_t b = (uint8_t)Serial1.read();
        dwin_isr_feed(&dwin, b);
    }
}