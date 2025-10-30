#include <Arduino.h>
#include <SoftwareSerial.h>
#include "dwin.h"
#include <stdint.h>

/*
 * DWIN Unified — Nano Demo (SoftwareSerial D8/D9)
 * =================================================
 * Watch the demo in action: https://www.youtube.com/channel/UCYCO2hnhuEiL2108BpAENxQ
 *
 * 1) Load the HMI (panel side)
 *    - Download the HMI package (DWIN_SET) from Releases:
 *      https://github.com/recepsenbas/DWIN-Unified/releases/latest
 *    - Unzip; copy the folder DWIN_SET/ to the root of a FAT32 microSD.
 *    - Power OFF the panel → insert the card → power ON → wait for update → remove card.
 *
 * 2) Wire the Nano to the DWIN panel
 *    - DWIN TX → Nano D8 (SoftwareSerial RX)
 *    - DWIN RX → Nano D9 (SoftwareSerial TX)
 *    - GND ↔ GND (common ground)
 *    - Baud: 115200
 *      Recommendation: Because of high nano baudrate errors @115200, strictly recommended set baudrate
 *      @9600 both side, on nano and display side. Download :
 *      https://github.com/recepsenbas/DWIN-T5L-SDCC-Template/blob/main/artifacts/v0.1.1/T5L51_9600_CrcOff_ResponseOn_noRTC.bin
 *
 * 3) Upload this sketch
 *    - Open Serial Monitor at 115200 for logs (USB serial).
 *    - The demo will showcase: page switching, overlay/touch routing,
 *      text write & wipe clear, graphs, RTC/NOR flash, icon move,
 *      color & visibility control — so you can try every feature quickly.
 *
 * Notes
 *    - Keep `dwin_poll(&dwin)` running in `loop()`.
 *    - If you change wiring or baud, update `dwinSerial.begin(...)` and the HMI accordingly.
 */

SoftwareSerial dwinSerial(8, 9); // RX, TX
static Dwin dwin;

static uint16_t counter = 0;
static unsigned long lastUpdate = 0;
static bool ledState = false;

// --- Serial glue functions for DWIN core ---
static size_t tx_write(const uint8_t *d, size_t n)
{
    Serial.print(F("[TX] "));
    for (size_t i = 0; i < n; i++)
    {
        if (d[i] < 0x10)
            Serial.print('0');
        Serial.print(d[i], HEX);
        Serial.print(' ');
    }
    Serial.println();
    size_t w = dwinSerial.write(d, n);
    dwinSerial.flush();  // ensure TX completes
    dwinSerial.listen(); // re-enable RX immediately after TX
    return w;
}
static int rx_read(uint8_t *d, size_t)
{
    if (dwinSerial.available())
    {
        *d = dwinSerial.read();
        return 1;
    }
    return 0;
}
static int rx_avail() { return dwinSerial.available(); }
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
    case 0x1008:
        ledState = !ledState;
        digitalWrite(LED_BUILTIN, ledState);
        Serial.print(F("LED -> "));
        Serial.println(ledState ? F("ON") : F("OFF"));
        Serial.println(F("--------------------------------"));
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
static void nor_demo()
{
    dwin_clear_text(&dwin, 0x5000, 100);
    delay(1000);
    dwin_write_text(&dwin, 0x5000, F("NOR Flash Demo"));
    delay(1000);
    dwin_clear_text(&dwin, 0x5000, 100);
    delay(1000);
    /* Write 1.234.567.890 to VP 0x6000 (2 words, 4 bytes) */
    dwin_write_text(&dwin, 0x5000, F("Write 1.234.567.890 to VP 0x6000"));
    delay(1000);
    dwin_write_u32(&dwin, 0x6000, 1234567890);
    delay(1000);

    /* Save 2 words from 0x3000 to NOR address 0x000002*/
    dwin_clear_text(&dwin, 0x5000, 100);
    delay(1000);
    dwin_write_text(&dwin, 0x5000, F("Store 2 words from VP 0x6000 to NOR 0x000002"));
    delay(1000);
    dwin_nor_store(&dwin, 0x000002, 0x6000, 2);
    delay(1000);

    /* Load 2 words from NOR address 0x000002 to VP 0x4000 */
    dwin_clear_text(&dwin, 0x5000, 100);
    delay(1000);
    dwin_write_text(&dwin, 0x5000, F("Load 2 words from NOR 0x000002 to VP 0x6100"));
    delay(1000);
    dwin_nor_load(&dwin, 0x000002, 0x6100, 2);
    delay(1000);
    dwin_clean_rx_queue(&dwin);
}

static void switching_page(void)
{
    dwin_clear_text(&dwin, 0x5000, 100);
    delay(1000);
    dwin_write_text(&dwin, 0x5000, F("Switching Page Demo"));
    delay(1000);
    for (uint8_t i = 1; i < 3; i++)
    {
        dwin_set_page(&dwin, i);
        delay(1500);
    }
    dwin_set_page(&dwin, 0);
    delay(1000);
    dwin_clear_text(&dwin, 0x5000, 100);
    delay(1000);

    dwin_clean_rx_queue(&dwin);
}
void overlay_demo()
{
    dwin_set_page(&dwin, 1);
    delay(2000);
    dwin_write_text(&dwin, 0x5000, F("Overlay Demo"));
    delay(1000);
    dwin_clear_text(&dwin, 0x5000, 100);
    delay(1000);
    dwin_write_text(&dwin, 0x5000, F("Enable overlay on page 0, touchs are from page 0"));
    delay(1000);
    dwin_overlay_enable(&dwin, 0, 1); // Enable overlay on page 0 (main), touch events are from page 0
    // dwin_overlay_enable(&dwin, 0, 0); // Enable overlay on page 0 (main), touch events are from page 1
    delay(10000);
    dwin_overlay_disable(&dwin);
    delay(1000);
    dwin_set_page(&dwin, 0);
    delay(1000);
    dwin_clear_text(&dwin, 0x5000, 100);
    delay(1000);
    dwin_clean_rx_queue(&dwin);
}

// --- Sine wave drawer ------------------------------------------------------
static void graph_draw_sine(uint8_t channel,
                            uint16_t vmin,
                            uint16_t vmax,
                            uint16_t samples,
                            uint8_t cycles,
                            uint16_t per_point_delay_ms)
{
    if (samples < 2)
        samples = 2; // avoid div-by-zero
    float mid = 0.5f * (float)vmin + 0.5f * (float)vmax;
    float amp = 0.5f * ((float)vmax - (float)vmin);

    for (uint16_t n = 0; n < samples; ++n)
    {
        float phase = (TWO_PI * (float)cycles * (float)n) / (float)(samples - 1);
        float s = sin(phase);                        // Arduino provides sin(double) and TWO_PI
        int32_t y = (int32_t)(mid + amp * s + 0.5f); // round to nearest
        if (y < (int32_t)vmin)
            y = vmin;
        if (y > (int32_t)vmax)
            y = vmax;
        dwin_graph_plot(&dwin, channel, (uint16_t)y);
        if (per_point_delay_ms)
            delay(per_point_delay_ms);
        dwin_clean_rx_queue(&dwin);
    }
}
void draw_and_clear_graph_demo(void)
{
    dwin_clear_text(&dwin, 0x5000, 100);
    delay(1000);
    dwin_set_page(&dwin, 1);
    delay(1000);
    /* Sine wave graph drawing */
    dwin_write_text(&dwin, 0x5000, F("Draw Sine wave on graph channel 0"));
    delay(1000);
    graph_draw_sine(0, 0, 1000, 1000, 20, 5); // channel, vmin, vmax, samples, cycles, delay
    dwin_clear_text(&dwin, 0x5000, 100);
    delay(1000);
    dwin_write_text(&dwin, 0x5000, F("Clear graph channel 0"));
    delay(1000);
    dwin_graph_clear(&dwin, 0);
    delay(1000);
    dwin_clear_text(&dwin, 0x5000, 100);
    delay(1000);
    dwin_set_page(&dwin, 0);
    delay(1000);
    dwin_clean_rx_queue(&dwin);
}

void icon_demo_move(void)
{
    dwin_clear_text(&dwin, 0x5000, 100);
    delay(1000);
    dwin_set_page(&dwin, 2);
    delay(1000);
    dwin_write_text(&dwin, 0x5000, F("Icon Move Demo"));
    delay(1000);

    const int16_t maxX = 700; // 800-100
    const int16_t maxY = 380; // 480-100

    // Hızı/adımı biraz artır (gerekiyorsa değiştir):
    const int8_t stepX = 3;
    const int8_t stepY = 2;

    // İmzacıklar ve konumlar signed olsun; sınır kontrolü kolaylaşır
    int16_t x = 0;
    int16_t y = 0;
    int8_t dirX = 1;
    int8_t dirY = 1;

    unsigned long startTime = millis();

    while (millis() - startTime < 8000)
    {
        // SP=0x7100 → [X_hi X_lo][Y_hi Y_lo]
        dwin_sp_set_xy(&dwin, 0x7100, (uint16_t)x, (uint16_t)y);

        // Parser'ı canlı tut (aksi halde haberleşme tıkanabilir)
        dwin_clean_rx_queue(&dwin);
        delay(5); // 5 ms şart

        // Bir sonraki konumu hesapla
        int16_t nx = x + (int16_t)(dirX * stepX);
        int16_t ny = y + (int16_t)(dirY * stepY);

        // X ekseni sekme ve kenara sıkı yapışma (overshoot'u kes)
        if (nx <= 0)
        {
            nx = 0;
            dirX = 1;
        }
        else if (nx >= maxX)
        {
            nx = maxX;
            dirX = -1;
        }

        // Y ekseni sekme ve kenara sıkı yapışma
        if (ny <= 0)
        {
            ny = 0;
            dirY = 1;
        }
        else if (ny >= maxY)
        {
            ny = maxY;
            dirY = -1;
        }

        x = nx;
        y = ny;
    }
    dwin_clear_text(&dwin, 0x5000, 100);
    delay(1000);
    dwin_set_page(&dwin, 0);
    delay(1000);
}
void data_var_move_demo(void)
{
    dwin_clear_text(&dwin, 0x5000, 100);
    delay(1000);
    dwin_write_text(&dwin, 0x5000, F("Move 0x2000 Data Var"));
    delay(1000);
    for (uint16_t x = 111; x < 500; x += 20)
    {
        dwin_sp_set_xy(&dwin, 0x7000, x, 61);
        delay(50);
    }

    for (uint16_t y = 61; y < 300; y += 20)
    {
        dwin_sp_set_xy(&dwin, 0x7000, 500, y);
        delay(50);
    }
    for (uint16_t x = 500; x > 111; x -= 15)
    {
        dwin_sp_set_xy(&dwin, 0x7000, x, 300);
        delay(50);
    }
    for (uint16_t y = 300; y > 61; y -= 15)
    {
        dwin_sp_set_xy(&dwin, 0x7000, 111, y);
        delay(50);
    }
    dwin_sp_set_xy(&dwin, 0x7000, 111, 61);
    delay(50);
}

void change_color_demo(void)
{
    // Serial.println(F("Starting Color Change Demo"));
    // Serial.println(F("You can pick RGB565 colors from https://rgbcolorpicker.com/565"));
    // https://rgbcolorpicker.com/565
    dwin_clear_text(&dwin, 0x5000, 100);
    delay(1000);
    dwin_write_text(&dwin, 0x5000, F("Changing Colors Demo"));
    delay(1000);
    dwin_clear_text(&dwin, 0x5000, 100);
    delay(1000);
    dwin_write_text(&dwin, 0x5000, F("Colors value can be found at rgbcolorpicker.com"));
    delay(1500);
    dwin_set_color565(&dwin, 0x7000, 0xffe0); // yellow color
    delay(1500);
    dwin_set_color565(&dwin, 0x7000, 0x07E0); // green color
    delay(1500);
    dwin_set_color565(&dwin, 0x7000, 0x001F); // blue color
    delay(1500);
    dwin_set_color565(&dwin, 0x7000, 0x0000); // black color
    delay(1500);
    dwin_set_color565(&dwin, 0x7000, 0xfd00); // orange color
}

// --- Setup ---
void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);
    /**
     * @brief Initialize DWIN display
     * If you are using SoftwareSerial, ensure that the
     * baud rate is set appropriately. SoftwareSerial
     * may not handle very high baud rates reliably.
     * So a baud rate of 9600 is often a safe choice.
     * But don't forget to set the same baud rate on
     * the DWIN HMI side as well.
     */
    dwinSerial.begin(115200);
    dwinSerial.listen();
    delay(500);

    Serial.println(F("=== DWIN Unified Example – Nano (SoftwareSerial) ==="));

    /* init DWIN with direct read/write hooks (NOT ISR queue) */
    dwin_init(&dwin, tx_write, rx_read, rx_avail, tick_ms, log_fn);

    /**
     * @brief Transport mode selection
     * 0 = Hook/Poll mode (SoftwareSerial or any stream with read/available)
     * 1 = ISR Queue mode (feed bytes from UART RX ISR via dwin_isr_feed)
     * This example intentionally uses mode=0 because we bind to SoftwareSerial
     * (see rx_read/rx_avail above). To use Hardware UART with ISR mode instead:
     * 1) dwin_use_isr_queue(&dwin, 1);
     * 2) Provide only tx_write to dwin_init (read/available may be NULL)
     * 3) In your UART RX ISR: dwin_isr_feed(&dwin, received_byte);
     *
     * AVR quick snippet (for reference only):
     * ISR(USART_RX_vect) {
     *     uint8_t b = UDR0;           // AVR HW UART data register
     *     dwin_isr_feed(&dwin, b);    // enqueue for parser
     * }
     *
     * @note On Nano, using pins 0/1 for HW UART conflicts with USB Serial.
     * Use an external USB‑UART for logging or switch debug prints to a second
     * SoftwareSerial if you choose ISR mode with pins 0/1.
     *
     */
    dwin_use_isr_queue(&dwin, 0);
    /* Set CRC mode 0: disabled */
    dwin_use_crc(&dwin, 0);

    /* Set touch and frame callbacks */
    dwin_set_touch_callback(&dwin, on_touch);
    dwin_set_frame_callback(&dwin, on_frame);

    /* Restart DWIN Module */
    // Serial.println(F("Restarting DWIN"));
    dwin_restart(&dwin);
    delay(1000);

    /* Write Hello Dwin World! to Text VP 0x5000, then clear after 3s */
    Serial.println(F("Writing 'Hello Dwin World!'"));
    dwin_write_text(&dwin, 0x5000, F("Hello Dwin World!"));
    delay(2000);
    dwin_clear_text(&dwin, 0x5000, 100);
    delay(1000);
    /* Clear text at VP 0x5000 */
    dwin_write_text(&dwin, 0x5000, F("Clearing text..."));
    delay(2000);
    dwin_clear_text(&dwin, 0x5000, 100);
    delay(1000);
    /* Write integer value at VP 0x2000 */
    Serial.println(F("Writing integer 123 to VP 0x2000"));
    dwin_write_text(&dwin, 0x5000, F("Writing integer 123 to VP 0x2000"));
    delay(1000);
    dwin_write_u16(&dwin, 0x2000, 123);
    delay(1000);
    dwin_clean_rx_queue(&dwin);

    /* Write floating point value at VP 0x2004 */
    Serial.println(F("Writing float 45.67 to VP 0x2004"));
    dwin_clear_text(&dwin, 0x5000, 100);
    delay(50);
    dwin_write_text(&dwin, 0x5000, F("Writing float 45.67 to VP 0x2004"));
    delay(1000);
    dwin_write_float(&dwin, 0x2004, 45.67f);
    delay(1000);
    dwin_clean_rx_queue(&dwin);

    /* Set Data Variable Font Size */
    dwin_clear_text(&dwin, 0x5000, 100);
    delay(1000);
    Serial.println(F("Setting font size 22 at SP 0x7000"));
    dwin_write_text(&dwin, 0x5000, F("Setting font size 22 at SP 0x7000"));
    delay(1000);
    dwin_set_font_size(&dwin, 0x7000, 22);
    delay(1000);
    dwin_clean_rx_queue(&dwin);

    /* Get DWIN firmware versions */
    dwin_clear_text(&dwin, 0x5000, 100);
    delay(1000);
    Serial.println(F("Getting DWIN firmware versions"));
    dwin_write_text(&dwin, 0x5000, F("Getting DWIN firmware versions"));
    delay(1000);
    dwin_clear_text(&dwin, 0x5000, 100);
    delay(1000);
    Serial.println(F("Write versions on display"));
    dwin_write_text(&dwin, 0x5000, F("Write versions on display"));
    delay(1000);
    uint8_t gui_ver = 0;
    uint8_t core_ver = 0;
    dwin_get_versions_now(&dwin, &gui_ver, &core_ver, 500);
    char gui_str[4];
    char core_str[4];
    utoa(gui_ver, gui_str, 16);
    utoa(core_ver, core_str, 16);
    dwin_write_text(&dwin, 0x5100, gui_str);
    delay(1000);
    dwin_write_text(&dwin, 0x5200, core_str);
    delay(1000);
    dwin_clean_rx_queue(&dwin);

    /* Set Time And Date without hardware RTC */
    Serial.println(F("Set Display Date/Time"));
    dwin_clear_text(&dwin, 0x5000, 100);
    delay(1000);
    dwin_write_text(&dwin, 0x5000, F("Set Display Date/Time"));
    delay(1000);
    dwin_rtc_write(&dwin, 2025, 05, 21, 17, 25, 13); // 2025-05-21 17:25:13
    delay(1000);
    dwin_clean_rx_queue(&dwin);

    /* If panel has hardware RTC, program it as well (commit via 0x009C key): */
    // Serial.println(F("Setting DWIN Hardware RTC to 2025-05-21 17:25:13"));
    // dwin_rtc_hw_set(&dwin, 2025, 05, 21, 17, 25, 13);
    // delay(50);

    /* Set brightness */
    dwin_clear_text(&dwin, 0x5000, 100);
    delay(1000);
    dwin_write_text(&dwin, 0x5000, "Set brightness to 20%");
    delay(1000);
    dwin_set_brightness(&dwin, 20); // 20%
    delay(2000);
    dwin_write_text(&dwin, 0x5000, "Set brightness to 100%");
    dwin_set_brightness(&dwin, 100); // 100%
    delay(2000);
    dwin_clean_rx_queue(&dwin);

    /* NOR Flash Demo */
    nor_demo();

    /* Read pageID and Switch Page Demo */
    switching_page();

    dwin_clear_text(&dwin, 0x5000, 100);
    delay(1000);
    dwin_write_text(&dwin, 0x5000, F("Beep for 1 second"));
    delay(1000);
    dwin_beep_1s(&dwin); // 1s beep
    delay(1000);

    /* Set backlight standby on:100%, standby=20%, 5s */
    /* It must be open CFG settings */
    // dwin_set_backlight_standby(&dwin, 100, 20, 500);

    draw_and_clear_graph_demo();

    icon_demo_move();

    /* Change color of an element */
    change_color_demo();

    /* Data Variable Movement */
    data_var_move_demo();

    /* Change visibility of an element */
    dwin_clear_text(&dwin, 0x5000, 100);
    delay(1000);
    dwin_write_text(&dwin, 0x5000, F("Changing Visibility of an Element"));
    delay(1000);
    dwin_clear_text(&dwin, 0x5000, 100);
    delay(1000);
    dwin_write_text(&dwin, 0x5000, F("Make 0x2000 invisible for 3 seconds"));
    delay(1000);
    dwin_set_visibility(&dwin, 0x7000, 0x2000, 0); // hide
    delay(3000);
    dwin_clear_text(&dwin, 0x5000, 100);
    delay(1000);
    dwin_write_text(&dwin, 0x5000, F("Make 0x2000 visible again"));
    delay(1000);
    dwin_set_visibility(&dwin, 0x7000, 0x2000, 1); // show
    delay(3000);
    dwin_clear_text(&dwin, 0x5000, 100);
    delay(1000);
    dwin_clean_rx_queue(&dwin);

    dwin_write_text(&dwin, 0x5000, F("Change Icon"));
    delay(1000);
    for (uint8_t i = 0; i < 5; i++)
    {
        dwin_write_u16(&dwin, 0x200A, 1);
        delay(800);
        dwin_write_u16(&dwin, 0x200A, 0);
        delay(800);
    }
    dwin_clear_text(&dwin, 0x5000, 100);
    delay(1000);
    dwin_clean_rx_queue(&dwin);

    // /* Overlay Demo */
    overlay_demo();

    dwin_write_text(&dwin, 0x5000, F("It is not all, explore for more!"));
    delay(1000);
    Serial.println(F("Setup complete."));
}

// --- Loop ---
void loop()
{
    /* Poll DWIN for incoming frames */
    dwin_poll(&dwin);

    if (millis() - lastUpdate > 2000)
    {
        lastUpdate = millis();
        counter++;
        /* Update counter display on DWIN HMI */
        dwin_write_u16(&dwin, 0x2000, counter);
        ledState = !ledState;
        digitalWrite(LED_BUILTIN, ledState);
    }
}