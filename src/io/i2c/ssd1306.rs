use core::cmp::{max, min};

use embassy_rp::{
    Peri,
    dma::Channel as DmaChannel,
    gpio::{Drive, Level, SlewRate},
    i2c::{Async as I2cAsync, I2c},
    peripherals::{I2C0, PIO0, PIO1},
    pio::{
        Common, Config as PioConfig, Direction as PioPinDirection, FifoJoin,
        Instance as PioInstance, Irq, Pin, PioPin, ShiftDirection, StateMachine, program::pio_asm,
    },
    pio_programs::clock_divider::calculate_pio_clock_divider,
};

use super::i2cpio::I2CPIO;

// Define the size of the display we have attached. This can vary, make sure you
// have the right size defined or the output will look rather odd!
// Code has been tested on 128x32 and 128x64 OLED displays
pub const SSD1306_HEIGHT: u8 = 32;
pub const SSD1306_WIDTH: u8 = 128;

const SSD1306_I2C_ADDR_DEFAULT: u8 = 0x3C;
const SSD1306_I2C_ADDR_SECONDARY: u8 = SSD1306_I2C_ADDR_DEFAULT + 1;

// 400 is usual, but often these can be overclocked to improve display response.
// Tested at 1000 on both 32 and 84 pixel height devices and it worked.
//const SSD1306_I2C_CLK: usize = 400;
const SSD1306_I2C_CLK: usize = 1000;

// -- control byte
const SSD1306_CONTROL_COMMAND: u8 = 0x80;
const SSD1306_CONTROL_DATA: u8 = 0x40;

// commands (see datasheet)
const SSD1306_SET_HIGH_COL_START_ADDR: u8 = 0x10;
const SSD1306_SET_MEM_MODE: u8 = 0x20;
const SSD1306_SET_COL_ADDR: u8 = 0x21;
const SSD1306_SET_PAGE_ADDR: u8 = 0x22;
const SSD1306_SET_HORIZ_SCROLL: u8 = 0x26;
const SSD1306_SET_SCROLL_DISABLE: u8 = 0x2E;
const SSD1306_SET_SCROLL_ENABLE: u8 = 0x2F;

const SSD1306_SET_DISP_START_LINE: u8 = 0x40;

const SSD1306_SET_CONTRAST: u8 = 0x81;
const SSD1306_SET_CHARGE_PUMP: u8 = 0x8D;

const SSD1306_SET_SEG_REMAP: u8 = 0xA0;
pub const SSD1306_SET_ENTIRE_ON_RAM: u8 = 0xA4;
pub const SSD1306_SET_ENTIRE_ON_ALL: u8 = 0xA5;
const SSD1306_SET_DISP_NORM: u8 = 0xA6;
pub const SSD1306_SET_DISP_INV: u8 = 0xA7;
const SSD1306_SET_MUX_RATIO: u8 = 0xA8;
pub const SSD1306_SET_DISP_OFF: u8 = 0xAE;
pub const SSD1306_SET_DISP_ON: u8 = 0xAF;
const SSD1306_SET_PAGE_ADDR_START_PAGE_MODE: u8 = 0xB0;
const SSD1306_SET_COM_OUT_SCAN_DIR_NORMAL: u8 = 0xC0;
const SSD1306_SET_COM_OUT_SCAN_DIR_REMAPPED: u8 = 0xC8;

const SSD1306_SET_DISP_OFFSET: u8 = 0xD3;
const SSD1306_SET_DISP_CLK_DIV: u8 = 0xD5;
const SSD1306_SET_PRECHARGE: u8 = 0xD9;
const SSD1306_SET_COM_PIN_CFG: u8 = 0xDA;
const SSD1306_SET_VCOM_DESEL: u8 = 0xDB;

const SSD1306_PAGE_HEIGHT: u8 = 8;
pub const SSD1306_NUM_PAGES: u8 = SSD1306_HEIGHT / SSD1306_PAGE_HEIGHT;
pub const SSD1306_BUF_LEN: usize = SSD1306_NUM_PAGES as usize * SSD1306_WIDTH as usize;

const SSD1306_WRITE_MODE: u8 = 0xFE;
const SSD1306_READ_MODE: u8 = 0xFF;

const COM_PIN_CFG_SEQ_NO_LR_REMAP: u8 = 0x02;
const COM_PIN_CFG_ALT_NO_LR_REMAP: u8 = 0x12;
const COM_PIN_CFG_SEQ_WITH_LR_REMAP: u8 = 0x22;
const COM_PIN_CFG_ALT_WITH_LR_REMAP: u8 = 0x32;
const CHARGE_PUMP_DISABLE: u8 = 0x04;
const CHARGE_PUMP_ENABLE: u8 = 0x14;
const ADDR_COLUMN_MAX: u8 = 0x7f;
const ADDR_COLUMN_MAX_PAGE_MODE: u8 = 0x0f;
const ADDR_PAGE_MAX: u8 = 0x07;
const CONTRAST_DEFAULT: u8 = 0x7f;
const DISPLAY_OFFSET_MAX: u8 = 0x3f;
const DIVIDE_RATIO_DEFAULT: u8 = 0x00;
const OSCILLATOR_FREQUENCY_DEFAULT: u8 = 0x08;
const MEMORY_ADDRESS_MODE_HORIZONTAL: u8 = 0;
const MEMORY_ADDRESS_MODE_VERTICAL: u8 = 1;
const MEMORY_ADDRESS_MODE_PAGE: u8 = 2;
const MUX_RATIO_MIN: u8 = 15;
const MUX_RATIO_MAX: u8 = 63;
const PHASE1_DEFAULT: u8 = 0x02;
const PHASE2_DEFAULT: u8 = 0x02;

const START_LINE_MAX: u8 = 64;
const VCOMH_LEVEL_V065: u8 = 0x00;
const VCOMH_LEVEL_V077: u8 = 0x20;
const VCOMH_LEVEL_V083: u8 = 0x30;
const VCOMH_LEVEL_AUTO: u8 = 0x40;

const ASCII_A: u8 = 65;
const ASCII_Z: u8 = 90;
const ASCII_0: u8 = 48;
const ASCII_9: u8 = 57;
const ASCII_A_LOWER: u8 = 97;
const ASCII_Z_LOWER: u8 = 122;

static FONT: [u8; 296] = [
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // nothing / space
    0x78, 0x14, 0x12, 0x11, 0x12, 0x14, 0x78, 0x00, //A
    0x7f, 0x49, 0x49, 0x49, 0x49, 0x49, 0x7f, 0x00, //B
    0x7e, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x00, //C
    0x7f, 0x41, 0x41, 0x41, 0x41, 0x41, 0x7e, 0x00, //D
    0x7f, 0x49, 0x49, 0x49, 0x49, 0x49, 0x49, 0x00, //E
    0x7f, 0x09, 0x09, 0x09, 0x09, 0x01, 0x01, 0x00, //F
    0x7f, 0x41, 0x41, 0x41, 0x51, 0x51, 0x73, 0x00, //G
    0x7f, 0x08, 0x08, 0x08, 0x08, 0x08, 0x7f, 0x00, //H
    0x00, 0x00, 0x00, 0x7f, 0x00, 0x00, 0x00, 0x00, //I
    0x21, 0x41, 0x41, 0x3f, 0x01, 0x01, 0x01, 0x00, //J
    0x00, 0x7f, 0x08, 0x08, 0x14, 0x22, 0x41, 0x00, //K
    0x7f, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x00, //L
    0x7f, 0x02, 0x04, 0x08, 0x04, 0x02, 0x7f, 0x00, //M
    0x7f, 0x02, 0x04, 0x08, 0x10, 0x20, 0x7f, 0x00, //N
    0x3e, 0x41, 0x41, 0x41, 0x41, 0x41, 0x3e, 0x00, //O
    0x7f, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0e, 0x00, //P
    0x3e, 0x41, 0x41, 0x49, 0x51, 0x61, 0x7e, 0x00, //Q
    0x7f, 0x11, 0x11, 0x11, 0x31, 0x51, 0x0e, 0x00, //R
    0x46, 0x49, 0x49, 0x49, 0x49, 0x30, 0x00, 0x00, //S
    0x01, 0x01, 0x01, 0x7f, 0x01, 0x01, 0x01, 0x00, //T
    0x3f, 0x40, 0x40, 0x40, 0x40, 0x40, 0x3f, 0x00, //U
    0x0f, 0x10, 0x20, 0x40, 0x20, 0x10, 0x0f, 0x00, //V
    0x7f, 0x20, 0x10, 0x08, 0x10, 0x20, 0x7f, 0x00, //W
    0x00, 0x41, 0x22, 0x14, 0x14, 0x22, 0x41, 0x00, //X
    0x01, 0x02, 0x04, 0x78, 0x04, 0x02, 0x01, 0x00, //Y
    0x41, 0x61, 0x59, 0x45, 0x43, 0x41, 0x00, 0x00, //Z
    0x3e, 0x41, 0x41, 0x49, 0x41, 0x41, 0x3e, 0x00, //0
    0x00, 0x00, 0x42, 0x7f, 0x40, 0x00, 0x00, 0x00, //1
    0x30, 0x49, 0x49, 0x49, 0x49, 0x46, 0x00, 0x00, //2
    0x49, 0x49, 0x49, 0x49, 0x49, 0x49, 0x36, 0x00, //3
    0x3f, 0x20, 0x20, 0x78, 0x20, 0x20, 0x00, 0x00, //4
    0x4f, 0x49, 0x49, 0x49, 0x49, 0x30, 0x00, 0x00, //5
    0x3f, 0x48, 0x48, 0x48, 0x48, 0x48, 0x30, 0x00, //6
    0x01, 0x01, 0x01, 0x61, 0x31, 0x0d, 0x03, 0x00, //7
    0x36, 0x49, 0x49, 0x49, 0x49, 0x49, 0x36, 0x00, //8
    0x06, 0x09, 0x09, 0x09, 0x09, 0x09, 0x7f, 0x00, //9
];

pub enum SSD1306MemoryAddressMode {
    Horizontal,
    Vertical,
    Page,
}

pub enum SSD1306VcomhDeselectLevel {
    V065,
    V077,
    V083,
    Auto,
}

pub struct SSD1306RenderArea {
    start_col: u8,
    end_col: u8,
    start_page: u8,
    end_page: u8,
    buflen: usize,
}

impl SSD1306RenderArea {
    pub fn new() -> Self {
        let start_col: u8 = 0;
        let end_col: u8 = SSD1306_WIDTH - 1;
        let start_page: u8 = 0;
        let end_page: u8 = SSD1306_NUM_PAGES - 1;
        let buflen = Self::calc_render_area_buflen(start_col, end_col, start_page, end_page);
        Self {
            start_col,
            end_col,
            start_page,
            end_page,
            buflen,
        }
    }

    fn calc_render_area_buflen(start_col: u8, end_col: u8, start_page: u8, end_page: u8) -> usize {
        // -- calculate how long the flattened buffer will be for a render area
        (end_col - start_col + 1) as usize * (end_page - start_page + 1) as usize
    }
}

pub enum SSD1306Addr {
    Default,
    Secondary,
}

pub struct SSD1306<'d> {
    i2cpio: I2CPIO<'d>,
    dev_addr: u8,
}

impl<'d> SSD1306<'d> {
    pub fn new(i2cpio: I2CPIO<'d>, addr: SSD1306Addr) -> Self {
        let dev_addr = match addr {
            SSD1306Addr::Default => SSD1306_I2C_ADDR_DEFAULT,
            SSD1306Addr::Secondary => SSD1306_I2C_ADDR_SECONDARY,
        };
        Self { i2cpio, dev_addr }
    }

    pub async fn send_cmd<const LEN: usize>(&mut self, cmd: [u8; LEN]) {
        // -- I2C write process expects a control byte followed by data
        // --this "data" can be a command or data to follow up a command
        // -- Co = 1, D/C = 0 => the driver expects a command
        //i2c_write_two_bytes(, self.dev_addr, SSD1306_CONTROL_COMMAND, cmd).await;
        self.i2cpio
            .i2c_write_byte_and_data(self.dev_addr, SSD1306_CONTROL_COMMAND, cmd)
            .await;
    }

    pub async fn send_data<const LEN: usize>(&mut self, data: [u8; LEN]) {
        // -- in horizontal addressing mode, the column address pointer auto-increments
        // -- and then wraps around to the next page, so we can send the entire frame
        // -- buffer in one gooooooo!
        self.i2cpio
            .i2c_write_byte_and_data(self.dev_addr, SSD1306_CONTROL_DATA, data)
            .await;
    }

    // ------------------------------------------------------------------------
    // -- fundamental commands
    // ------------------------------------------------------------------------

    pub async fn set_contrast(&mut self, contrast: u8) {
        // -- 256 contrast steps, 0x7f RESET
        let cmd = [SSD1306_SET_CONTRAST, contrast];
        self.send_cmd(cmd).await;
    }

    pub async fn disable_entire_on(&mut self) {
        let cmd = [SSD1306_SET_ENTIRE_ON_RAM];
        self.send_cmd(cmd).await;
    }

    pub async fn enable_entire_on_all(&mut self) {
        let cmd = [SSD1306_SET_ENTIRE_ON_ALL];
        self.send_cmd(cmd).await;
    }

    pub async fn set_display_normal(&mut self) {
        let cmd = [SSD1306_SET_DISP_NORM];
        self.send_cmd(cmd).await;
    }

    pub async fn set_display_inverted(&mut self) {
        let cmd = [SSD1306_SET_DISP_INV];
        self.send_cmd(cmd).await;
    }

    pub async fn set_display_off(&mut self) {
        let cmd = [SSD1306_SET_DISP_OFF];
        self.send_cmd(cmd).await;
    }

    pub async fn set_display_on(&mut self) {
        let cmd = [SSD1306_SET_DISP_ON];
        self.send_cmd(cmd).await;
    }

    // ------------------------------------------------------------------------
    // -- scrolling commands
    // ------------------------------------------------------------------------

    // pub async fn set_continuous_horizontal_scroll(
    //     &mut self,
    // ) {
    //     let cmd = [];
    //     self.send_cmd(cmd).await;
    // }

    pub async fn disable_horizontal_scrolling(&mut self) {
        let cmd = [SSD1306_SET_SCROLL_DISABLE];
        self.send_cmd(cmd).await;
    }

    pub async fn enable_horizontal_scrolling(&mut self) {
        let cmd = [SSD1306_SET_SCROLL_ENABLE];
        self.send_cmd(cmd).await;
    }

    // ------------------------------------------------------------------------
    // -- address setting commands
    // ------------------------------------------------------------------------

    pub async fn set_column_start_addr_for_page_mode(
        &mut self,
        lower_start_addr: u8,
        higher_start_addr: u8,
    ) {
        let lower_start_addr = min(lower_start_addr, ADDR_COLUMN_MAX_PAGE_MODE);
        let cmd = [lower_start_addr];
        self.send_cmd(cmd).await;
        let higher_start_addr = min(higher_start_addr, ADDR_COLUMN_MAX_PAGE_MODE);
        let cmd = [SSD1306_SET_HIGH_COL_START_ADDR | higher_start_addr];
        self.send_cmd(cmd).await;
    }

    pub async fn set_page_addr_start_for_page_mode(&mut self, start_addr: u8) {
        let start_addr = min(start_addr, ADDR_PAGE_MAX);
        let cmd = [SSD1306_SET_PAGE_ADDR_START_PAGE_MODE | start_addr];
        self.send_cmd(cmd).await;
    }

    pub async fn cmd_set_mem_addr_mode(&mut self, mode: SSD1306MemoryAddressMode) {
        // set memory address mode 0 = horizontal, 1 = vertical, 2 = page
        let mode = match mode {
            SSD1306MemoryAddressMode::Horizontal => MEMORY_ADDRESS_MODE_HORIZONTAL,
            SSD1306MemoryAddressMode::Vertical => MEMORY_ADDRESS_MODE_VERTICAL,
            SSD1306MemoryAddressMode::Page => MEMORY_ADDRESS_MODE_PAGE,
        };
        let cmd = [SSD1306_SET_MEM_MODE, mode];
        self.send_cmd(cmd).await;
    }

    pub async fn set_column_addr(&mut self, start_addr: u8, end_addr: u8) {
        let start_addr = min(start_addr, ADDR_COLUMN_MAX);
        let end_addr = min(end_addr, ADDR_COLUMN_MAX);
        let cmd = [SSD1306_SET_COL_ADDR, start_addr, end_addr];
        self.send_cmd(cmd).await;
    }

    pub async fn set_page_addr(&mut self, start_addr: u8, end_addr: u8) {
        let start_addr = min(start_addr, ADDR_PAGE_MAX);
        let end_addr = min(end_addr, ADDR_PAGE_MAX);
        let cmd = [SSD1306_SET_PAGE_ADDR, start_addr, end_addr];
        self.send_cmd(cmd).await;
    }

    // ------------------------------------------------------------------------
    // -- hardware configuration
    // ------------------------------------------------------------------------

    pub async fn set_display_start_line(&mut self, start_line: u8) {
        let start_line = min(start_line, START_LINE_MAX);
        let cmd = [SSD1306_SET_DISP_START_LINE | start_line];
        self.send_cmd(cmd).await;
    }

    pub async fn set_segment_remap(&mut self, remap: bool) {
        let remap = if remap { 1 } else { 0 };
        let cmd = [SSD1306_SET_SEG_REMAP | remap];
        self.send_cmd(cmd).await;
    }

    pub async fn set_mux_ratio(&mut self, ratio: u8) {
        let ratio = if ratio < MUX_RATIO_MIN {
            MUX_RATIO_MIN
        } else if ratio > MUX_RATIO_MAX {
            MUX_RATIO_MIN
        } else {
            ratio
        };
        let cmd = [SSD1306_SET_MUX_RATIO, ratio];
        self.send_cmd(cmd).await;
    }

    pub async fn set_com_out_scan_dir(&mut self, remap: bool) {
        let cmd = if remap {
            [SSD1306_SET_COM_OUT_SCAN_DIR_REMAPPED]
        } else {
            [SSD1306_SET_COM_OUT_SCAN_DIR_NORMAL]
        };
        self.send_cmd(cmd).await;
    }

    pub async fn set_display_offset(&mut self, offset: u8) {
        let offset = min(offset, DISPLAY_OFFSET_MAX);
        let cmd = [SSD1306_SET_DISP_OFFSET, offset];
        self.send_cmd(cmd).await;
    }

    pub async fn set_com_pin_cfg(&mut self, alternative_com_pin: bool, com_left_right_remap: bool) {
        let pin_cfg = if alternative_com_pin {
            if com_left_right_remap {
                COM_PIN_CFG_ALT_WITH_LR_REMAP
            } else {
                COM_PIN_CFG_ALT_NO_LR_REMAP
            }
        } else {
            if com_left_right_remap {
                COM_PIN_CFG_SEQ_WITH_LR_REMAP
            } else {
                COM_PIN_CFG_SEQ_NO_LR_REMAP
            }
        };
        let cmd = [SSD1306_SET_COM_PIN_CFG, pin_cfg];
        self.send_cmd(cmd).await;
    }

    // ------------------------------------------------------------------------
    // -- timing & driving scheme
    // ------------------------------------------------------------------------

    pub async fn set_disp_clk_div(&mut self, oscillator_frequency: u8, divide_ratio: u8) {
        let value = ((oscillator_frequency & 0xf) << 4) | (divide_ratio & 0xf);
        let cmd = [SSD1306_SET_DISP_CLK_DIV, value];
        self.send_cmd(cmd).await;
    }

    pub async fn set_precharge(&mut self, phase1: u8, phase2: u8) {
        let phase1 = max(1, phase1 & 0xf);
        let phase2 = max(1, phase2 & 0xf);
        let value = (phase2 << 4) | (phase1 & 0xf);
        let cmd = [SSD1306_SET_PRECHARGE, value];
        self.send_cmd(cmd).await;
    }

    pub async fn set_vcomh_deselect_level(&mut self, level: SSD1306VcomhDeselectLevel) {
        let level = match level {
            SSD1306VcomhDeselectLevel::V065 => VCOMH_LEVEL_V065,
            SSD1306VcomhDeselectLevel::V077 => VCOMH_LEVEL_V077,
            SSD1306VcomhDeselectLevel::V083 => VCOMH_LEVEL_V083,
            SSD1306VcomhDeselectLevel::Auto => VCOMH_LEVEL_AUTO,
        };
        let cmd = [SSD1306_SET_VCOM_DESEL, level];
        self.send_cmd(cmd).await;
    }

    pub async fn disable_charge_pump(&mut self) {
        let cmd = [SSD1306_SET_CHARGE_PUMP, CHARGE_PUMP_DISABLE];
        self.send_cmd(cmd).await;
    }

    pub async fn enable_charge_pump(&mut self) {
        let cmd = [SSD1306_SET_CHARGE_PUMP, CHARGE_PUMP_ENABLE];
        self.send_cmd(cmd).await;
    }

    pub async fn init(&mut self) {
        // Some of these commands are not strictly necessary as the reset
        // process defaults to some of these but they are shown here
        // to demonstrate what the initialization sequence looks like
        // Some configuration values are recommended by the board manufacturer
        self.i2cpio.enable();
        defmt::debug!("SSD1306 init 1");
        self.set_mux_ratio(SSD1306_HEIGHT - 1).await;
        defmt::debug!("SSD1306 init 2");
        self.set_display_offset(0).await;
        defmt::debug!("SSD1306 init 3");
        self.set_display_start_line(0).await;
        defmt::debug!("SSD1306 init 4");
        self.set_segment_remap(true).await; // set segment re-map, column address 127 is mapped to SEG0
        defmt::debug!("SSD1306 init 5");
        self.set_com_out_scan_dir(true).await;
        defmt::debug!("SSD1306 init 6");
        self.set_com_pin_cfg(false, false).await;
        defmt::debug!("SSD1306 init 7");
        self.set_contrast(CONTRAST_DEFAULT).await;
        defmt::debug!("SSD1306 init 8");
        self.disable_entire_on().await;
        defmt::debug!("SSD1306 init 9");
        self.set_display_normal().await;
        defmt::debug!("SSD1306 init 10");
        self.set_disp_clk_div(OSCILLATOR_FREQUENCY_DEFAULT, DIVIDE_RATIO_DEFAULT)
            .await; // -- divide ratio = 1, standard oscillator frequency
        defmt::debug!("SSD1306 init 11");
        self.cmd_set_mem_addr_mode(SSD1306MemoryAddressMode::Horizontal)
            .await;
        defmt::debug!("SSD1306 init 12");
        self.disable_horizontal_scrolling().await;
        defmt::debug!("SSD1306 init 13");
        self.enable_charge_pump().await;
        defmt::debug!("SSD1306 init 14");
        self.set_display_on().await;

        // defmt::debug!("SSD1306 init 1");
        // self.set_display_off().await;
        // defmt::debug!("SSD1306 init 2");
        // self.set_disp_clk_div(OSCILLATOR_FREQUENCY_DEFAULT, DIVIDE_RATIO_DEFAULT)
        //     .await; // -- standard oscillator frequency, divide ratio = 1
        // defmt::debug!("SSD1306 init 3");
        // self.set_mux_ratio(SSD1306_HEIGHT - 1).await;
        // defmt::debug!("SSD1306 init 4");
        // self.set_display_offset(0).await;
        // defmt::debug!("SSD1306 init 5");
        // self.set_display_start_line(0).await;
        // defmt::debug!("SSD1306 init 6");
        // self.disable_charge_pump().await;
        // defmt::debug!("SSD1306 init 7");
        // self.cmd_set_mem_addr_mode(SSD1306MemoryAddressMode::Horizontal)
        //     .await;
        // defmt::debug!("SSD1306 init 8");
        // self.set_segment_remap(true).await; // set segment re-map, column address 127 is mapped to SEG0
        // defmt::debug!("SSD1306 init 9");
        // self.set_com_out_scan_dir(true).await;
        // defmt::debug!("SSD1306 init 10");
        // self.set_com_pin_cfg(false, false).await;
        // defmt::debug!("SSD1306 init 11");
        // self.set_contrast(CONTRAST_DEFAULT).await;
        // defmt::debug!("SSD1306 init 12");
        // self.set_vcomh_deselect_level(SSD1306VcomhDeselectLevel::Auto)
        //     .await;
        // defmt::debug!("SSD1306 init 13");
        // self.set_precharge(PHASE1_DEFAULT, PHASE2_DEFAULT).await;
        // defmt::debug!("SSD1306 init 14");
        // self.set_entire_on_ram().await;
        // defmt::debug!("SSD1306 init 15");
        // self.set_display_normal().await;
        // defmt::debug!("SSD1306 init 16");
        // self.disable_horizontal_scrolling().await;
        // defmt::debug!("SSD1306 init 17");
        // self.set_display_on().await;
    }

    pub async fn render<const LEN: usize>(&mut self, data: [u8; LEN], area: &SSD1306RenderArea) {
        // -- update a portion of the display with a render area
        defmt::debug!("SSD1306 render column addr");
        self.set_column_addr(area.start_col, area.end_col).await;
        defmt::debug!("SSD1306 render page addr");
        self.set_page_addr(area.start_page, area.end_page).await;
        defmt::debug!("SSD1306 render data {}", LEN);
        self.send_data(data).await;
    }

    pub fn set_pixel(buf: &mut [u8; SSD1306_BUF_LEN], x: u8, y: u8, on: bool) {
        let x = min(x, SSD1306_WIDTH);
        let y = min(y, SSD1306_HEIGHT);

        // The calculation to determine the correct bit to set depends on which address
        // mode we are in. This code assumes horizontal

        // The video ram on the SSD1306 is split up in to 8 rows, one bit per pixel.
        // Each row is 128 long by 8 pixels high, each byte vertically arranged, so byte 0 is x=0, y=0->7,
        // byte 1 is x = 1, y=0->7 etc

        // This code could be optimised, but is like this for clarity. The compiler
        // should do a half decent job optimising it anyway.

        let bytes_per_row = SSD1306_WIDTH as usize; // x pixels, 1bpp, but each row is 8 pixel high, so (x / 8) * 8

        let byte_idx = (y as usize / 8) * bytes_per_row + x as usize;
        let mut byte = buf[byte_idx];

        if on {
            byte |= 1 << (y % 8);
        } else {
            byte &= !(1 << (y % 8));
        }
        buf[byte_idx] = byte;
    }

    // -- basic Bresenhams.
    pub fn draw_line(buf: &mut [u8; SSD1306_BUF_LEN], x0: u8, y0: u8, x1: u8, y1: u8, on: bool) {
        let sx: i16 = if x0 < x1 { 1 } else { -1 };
        let dx = if sx > 0 { x1 - x0 } else { x0 - x1 };
        let sy: i16 = if y0 < y1 { 1 } else { -1 };
        let dy = if sy > 0 { y0 - y1 } else { y1 - y0 };
        let mut x0 = x0 as i16;
        let mut y0 = y0 as i16;
        let x1 = x1 as i16;
        let y1 = y1 as i16;
        let mut err = dx + dy;
        let mut e2;

        loop {
            Self::set_pixel(buf, x0 as u8, y0 as u8, on);
            if x0 == x1 && y0 == y1 {
                break;
            }
            e2 = 2 * err;
            if e2 >= dy {
                err += dy;
                x0 += sx;
            }
            if e2 <= dx {
                err += dx;
                y0 += sy;
            }
        }
    }

    pub fn get_font_index(ch: u8) -> usize {
        if ch >= ASCII_A && ch <= ASCII_Z {
            return (ch - ASCII_A + 1) as usize;
        } else if ch >= ASCII_0 && ch <= ASCII_9 {
            return (ch - ASCII_0 + 27) as usize;
        }
        0 // -- char not in font table: nothing / space
    }

    pub fn write_char(buf: &mut [u8; SSD1306_BUF_LEN], x: u8, y: u8, ch: u8) {
        // -- check bounds
        if x > SSD1306_WIDTH - 8 || y > SSD1306_HEIGHT - 8 {
            // -- do not attempt to write char if it's out of bounds
            return;
        }
        // -- only write on Y row boundaries (every 8 vertical pixels)
        let y = y / 8;
        // -- toupper
        let ch = if ch >= ASCII_A_LOWER && ch <= ASCII_Z_LOWER {
            ch - (ASCII_A_LOWER - ASCII_A)
        } else {
            ch
        };
        let idx = Self::get_font_index(ch);
        let mut fb_idx = (y * 128 + x) as usize;

        for i in 0..8 {
            buf[fb_idx] = FONT[idx * 8 + i];
            fb_idx += 1;
        }
    }

    pub fn write_string(buf: &mut [u8; SSD1306_BUF_LEN], x: u8, y: u8, val: &str) {
        // -- check bounds
        if x > SSD1306_WIDTH - 8 || y > SSD1306_HEIGHT - 8 {
            // -- do not attempt to write if it's out of bounds
            return;
        }
        let mut x = x;
        for ch in val.chars() {
            let ch = ch as u8;
            Self::write_char(buf, x, y, ch);
            x += 8;
        }
    }
}
