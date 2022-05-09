#![no_std]
#![no_main]

extern crate panic_halt;

use atsamd_hal::{
    clock::GenericClockController,
    dmac::{DmaController, PriorityLevel, Transfer, TriggerAction, TriggerSource},
    gpio::Pins,
    pac::Peripherals,
    rtc::Rtc,
    sercom::{
        spi::{self, BitOrder, EightBit},
        Sercom0,
    },
    time::U32Ext,
};
use cortex_m_rt::entry;
use micromath::F32Ext;

mod bit_expand;

const N_LEDS: usize = 20;
static mut DMA_BUFFER: [u8; N_LEDS * 4 * 3 + 120] = [0u8; N_LEDS * 4 * 3 + 120];

const BRIGHTNESS: f32 = 48.0;

const LED_COORDS: [(f32, f32); 20] = [
    (2.25, 0.0),
    (3.0, 2.2),
    (2.25, 2.2),
    (1.4, 2.2),
    (0.7, 2.2),
    (0.0, 2.2),
    (0.0, 2.55),
    (0.35, 2.5),
    (0.7, 2.55),
    (1.15, 2.55),
    (1.4, 2.7),
    (1.7, 2.55),
    (2.45, 2.55),
    (2.75, 2.55),
    (2.9, 2.9),
    (2.3, 2.9),
    (1.7, 2.9),
    (1.15, 2.9),
    (0.7, 2.9),
    (0.0, 2.9),
];

fn write_pixel(n: usize, r: u8, g: u8, b: u8) {
    let base = n * 4 * 3;

    unsafe {
        DMA_BUFFER[base] = bit_expand::LOOKUP_A[g as usize];
        DMA_BUFFER[base + 1] = bit_expand::LOOKUP_B[g as usize];
        DMA_BUFFER[base + 2] = bit_expand::LOOKUP_C[g as usize];
        DMA_BUFFER[base + 3] = bit_expand::LOOKUP_D[g as usize];

        DMA_BUFFER[base + 4] = bit_expand::LOOKUP_A[r as usize];
        DMA_BUFFER[base + 5] = bit_expand::LOOKUP_B[r as usize];
        DMA_BUFFER[base + 6] = bit_expand::LOOKUP_C[r as usize];
        DMA_BUFFER[base + 7] = bit_expand::LOOKUP_D[r as usize];

        DMA_BUFFER[base + 8] = bit_expand::LOOKUP_A[b as usize];
        DMA_BUFFER[base + 9] = bit_expand::LOOKUP_B[b as usize];
        DMA_BUFFER[base + 10] = bit_expand::LOOKUP_C[b as usize];
        DMA_BUFFER[base + 11] = bit_expand::LOOKUP_D[b as usize];
    }
}

fn hsv_to_rgb(h: f32, s: f32, v: f32) -> (f32, f32, f32) {
    (
        ((((h * 6.0 - 3.0).abs() - 1.0).min(1.0).max(0.0) - 1.0) * s + 1.0) * v,
        (((2.0 - (h * 6.0 - 2.0).abs()).min(1.0).max(0.0) - 1.0) * s + 1.0) * v,
        (((2.0 - (h * 6.0 - 4.0).abs()).min(1.0).max(0.0) - 1.0) * s + 1.0) * v,
    )
}

fn anim(t: f32, x: f32, y: f32) -> (f32, f32, f32) {
    let x = x - 1.4;
    let y = y - 2.4;
    let hue = ((x * x + y * y).sqrt() * 0.25 - 0.5 * t).fract().abs();
    hsv_to_rgb(hue, 1.0, 1.0)
}

#[entry]
fn main() -> ! {
    let mut peripherals = Peripherals::take().unwrap();

    // Initialize clocks
    let mut clocks = GenericClockController::with_internal_32kosc(
        peripherals.GCLK,
        &mut peripherals.PM,
        &mut peripherals.SYSCTRL,
        &mut peripherals.NVMCTRL,
    );

    // Initialize GPIO
    let pins = Pins::new(peripherals.PORT);

    // Initialize SPI clock
    let gclk0 = clocks.gclk0();
    let spi_clock = clocks.sercom0_core(&gclk0).unwrap();

    // Initialize SPI
    let pads = spi::Pads::<Sercom0>::default()
        .data_out(pins.pa04)
        .sclk(pins.pa05);
    let spi = spi::Config::new(&peripherals.PM, peripherals.SERCOM0, pads, spi_clock.freq())
        .spi_mode(spi::MODE_0)
        .bit_order(BitOrder::MsbFirst)
        .baud(3200.khz())
        .char_size::<EightBit>()
        .enable();

    // Initialize DMA
    let mut dmac = DmaController::init(peripherals.DMAC, &mut peripherals.PM);
    let channels = dmac.split();

    // Initialize RTC
    let gclk1 = clocks.gclk1();
    let rtc_clock = clocks.rtc(&gclk1).unwrap();
    let mut rtc = Rtc::count32_mode(peripherals.RTC, rtc_clock.freq(), &mut peripherals.PM);

    // Start SPI DMA
    let ch0 = channels.0.init(PriorityLevel::LVL0);
    // ch0.enable_interrupts(InterruptFlags::new().with_tcmpl(true));
    let _spi_trans = unsafe { Transfer::new_unchecked(ch0, &mut DMA_BUFFER, spi, true) }
        .begin(TriggerSource::SERCOM0_TX, TriggerAction::BEAT);

    // Render loop
    let mut t = 0f32;
    loop {
        let sub = rtc.count32();
        if sub > 32_768 {
            rtc.set_count32(0);
            t += 1.0;
        }

        let t = t + (sub % 32_768) as f32 / 32_768.0;

        for i in 0..N_LEDS {
            let (x, y) = LED_COORDS[i];
            let v = anim(t, x, y);

            write_pixel(
                i,
                (v.0 * BRIGHTNESS) as u8,
                (v.1 * BRIGHTNESS) as u8,
                (v.2 * BRIGHTNESS) as u8,
            );
        }
    }
}
