
# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- 

### Changed
- 

### Fixed
- 

---

## [1.0.2] - 2025-10-30

### Fixed
- Minor library file improvements and bug fixes

---

## [1.0.0] - 2025-10-23

### Added
- **Core Library**
    - Portable C core with Arduino compatibility layer
    - Robust RX parser with permissive CRC handling (supports CMD+PAYLOAD or full-frame CRC)
    - Optional CRC16 (MODBUS) transmission validation

- **VP (Variable Pointer) Operations**
    - `dwin_write_vp()` - Write data to VP addresses
    - `dwin_read_vp()` - Read data from VP addresses  
    - `dwin_write_u16()` - Write 16-bit unsigned integers
    - `dwin_write_u32()` - Write 32-bit unsigned integers

- **Text I/O System**
    - Ultra-low SRAM text operations with chunked writes
    - `dwin_write_text()` with PROGMEM support for memory optimization
    - `dwin_clear_text()` for text field management

- **Display & UI Controls**
    - `dwin_set_page()` - Page navigation
    - `dwin_get_page_now()` - Current page retrieval
    - `dwin_set_brightness()` - Display brightness control
    - `dwin_set_visibility()` - Element visibility management

- **Touch & Overlay Management**
    - `dwin_overlay_enable()/disable()` - Overlay control
    - `dwin_touch_disable_all()/enable()` - Touch input routing

- **Styling & Positioning**
    - `dwin_sp_set_xy()` - Element positioning
    - `dwin_set_color565()` - RGB565 color management
    - `dwin_set_font_size()` - Font size control
    - `dwin_set_font_lib()` - Font library selection

- **Data & Time Management**
    - `dwin_rtc_write()` - Real-time clock operations
    - `dwin_rtc_hw_set()` - Hardware clock synchronization
    - `dwin_nor_store()/load()` - NOR flash storage operations

- **Graphics & Visualization**
    - `dwin_graph_plot()` - Graph plotting functionality
    - `dwin_graph_clear()` - Graph clearing operations

- **Platform Support**
    - STM32 port implementation in `ports/stm32/`
    - SoftwareSerial compatibility for resource-constrained MCUs

### Technical Highlights
- Zero dynamic memory allocation design
- Optimized for minimal SRAM usage through chunked operations
- Compatible with low-resource microcontrollers

