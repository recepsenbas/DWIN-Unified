

# Changelog

All notable changes to this project will be documented in this file.

This project follows **Semantic Versioning**.

## [Unreleased]

### Added
- 

### Changed
- 

### Fixed
- 

---

## [1.0.0] - 2025-10-23

### Added
- Portable C core with Arduino glue.
- Robust RX parser with permissive CRC handling (accepts CMD+PAYLOAD or full-frame CRC).
- Optional CRC16 (MODBUS) on TX.
- VP helpers: `dwin_write_vp`, `dwin_read_vp`, `dwin_write_u16`, `dwin_write_u32`.
- Ultra‑low‑SRAM text I/O: chunked `dwin_write_text` with PROGMEM overload.
- Text utilities: `dwin_clear_text`.
- Page & UI: `dwin_set_page`, `dwin_get_page_now`, `dwin_set_brightness`, `dwin_set_visibility`.
- Overlay & touch routing: `dwin_overlay_enable/disable`, `dwin_touch_disable_all/enable`.
- Positioning & style: `dwin_sp_set_xy`, `dwin_set_color565`, `dwin_set_font_size`, `dwin_set_font_lib`.
- Time & storage: `dwin_rtc_write`, `dwin_rtc_hw_set`, `dwin_nor_store`, `dwin_nor_load`.
- Graph helpers: `dwin_graph_plot`, `dwin_graph_clear`.
- STM32 port scaffold under `ports/stm32/`.

### Notes
- No dynamic allocation; suitable for SoftwareSerial‑class MCUs.
- Text operations optimized for tiny SRAM via chunked writes.