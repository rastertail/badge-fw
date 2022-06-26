#![no_std]
#![no_main]

extern crate panic_halt;

use core::{
    mem::MaybeUninit,
    sync::atomic::{AtomicU8, AtomicUsize, Ordering},
};

use atsamd_hal::{
    clock::{ClockGenId, ClockSource, GenericClockController},
    dmac::{DmaController, PriorityLevel, Transfer, TriggerAction, TriggerSource},
    ehal::blocking::i2c::{Write, WriteRead},
    eic::{
        self,
        pin::{EicPin, Sense},
    },
    gpio::{self, pin, AlternateD, Pin, Pins},
    pac::{self, interrupt, Interrupt, Peripherals, ADC, NVIC},
    rtc::Rtc,
    sercom::{
        i2c,
        spi::{self, BitOrder, EightBit},
        Sercom0, Sercom2,
    },
    time::U32Ext,
};
use cortex_m_rt::entry;
use micromath::F32Ext;

type I2cPads = i2c::Pads<Sercom2, Pin<gpio::PA08, AlternateD>, Pin<gpio::PA09, AlternateD>>;
type I2c = i2c::I2c<i2c::Config<I2cPads>>;

#[repr(C)]
struct arm_cfft_instance_q15 {
    len: u16,
    twiddle: *const i16,
    bit_rev: *const u16,
    bit_rev_len: u16,
}

extern "C" {
    fn arm_cmplx_mag_q15(src: *const i16, dst: *mut i16, n: u32);
    fn arm_cfft_init_q15(s: *mut arm_cfft_instance_q15, len: u16);
    fn arm_cfft_q15(s: *const arm_cfft_instance_q15, d: *mut i16, ifft: u8, rev: u8);
    fn arm_scale_f32(src: *const f32, scale: f32, dst: *mut f32, n: u32);
    fn arm_q15_to_float(src: *const i16, dst: *mut f32, n: u32);
}

mod bit_expand;

const FFT_SAMPLES: usize = 256;
static mut ADC_BUFFER: [u16; FFT_SAMPLES] = [0u16; FFT_SAMPLES];
static ADC_END: AtomicUsize = AtomicUsize::new(0);
static mut FFT_BUFFER: [i16; FFT_SAMPLES * 2] = [0i16; FFT_SAMPLES * 2];
static mut FFT_F32: [f32; FFT_SAMPLES] = [0f32; FFT_SAMPLES];

const HAMMING_LUT: [f32; FFT_SAMPLES] = [
    0.0801385, 0.0805541, 0.0812464, 0.082215, 0.0834594, 0.0849788, 0.0867723, 0.0888388,
    0.091177, 0.0937856, 0.096663, 0.0998074, 0.103217, 0.10689, 0.110823, 0.115015, 0.119464,
    0.124165, 0.129117, 0.134316, 0.13976, 0.145445, 0.151367, 0.157524, 0.163911, 0.170525,
    0.177361, 0.184415, 0.191684, 0.199162, 0.206846, 0.214731, 0.222811, 0.231083, 0.23954,
    0.248179, 0.256993, 0.265978, 0.275128, 0.284438, 0.293901, 0.303513, 0.313267, 0.323158,
    0.333179, 0.343325, 0.353589, 0.363966, 0.374448, 0.385031, 0.395706, 0.406469, 0.417312,
    0.428229, 0.439213, 0.450258, 0.461358, 0.472504, 0.483691, 0.494912, 0.50616, 0.517429,
    0.528711, 0.54, 0.551289, 0.562571, 0.57384, 0.585088, 0.596309, 0.607496, 0.618642, 0.629742,
    0.640787, 0.651771, 0.662688, 0.673531, 0.684294, 0.694969, 0.705552, 0.716034, 0.726411,
    0.736675, 0.746821, 0.756842, 0.766733, 0.776487, 0.786099, 0.795562, 0.804872, 0.814022,
    0.823007, 0.831821, 0.84046, 0.848917, 0.857189, 0.865269, 0.873154, 0.880838, 0.888316,
    0.895585, 0.902639, 0.909475, 0.916089, 0.922476, 0.928633, 0.934555, 0.94024, 0.945684,
    0.950883, 0.955835, 0.960536, 0.964985, 0.969177, 0.97311, 0.976783, 0.980193, 0.983337,
    0.986214, 0.988823, 0.991161, 0.993228, 0.995021, 0.996541, 0.997785, 0.998754, 0.999446,
    0.999861, 1., 0.999861, 0.999446, 0.998754, 0.997785, 0.996541, 0.995021, 0.993228, 0.991161,
    0.988823, 0.986214, 0.983337, 0.980193, 0.976783, 0.97311, 0.969177, 0.964985, 0.960536,
    0.955835, 0.950883, 0.945684, 0.94024, 0.934555, 0.928633, 0.922476, 0.916089, 0.909475,
    0.902639, 0.895585, 0.888316, 0.880838, 0.873154, 0.865269, 0.857189, 0.848917, 0.84046,
    0.831821, 0.823007, 0.814022, 0.804872, 0.795562, 0.786099, 0.776487, 0.766733, 0.756842,
    0.746821, 0.736675, 0.726411, 0.716034, 0.705552, 0.694969, 0.684294, 0.673531, 0.662688,
    0.651771, 0.640787, 0.629742, 0.618642, 0.607496, 0.596309, 0.585088, 0.57384, 0.562571,
    0.551289, 0.54, 0.528711, 0.517429, 0.50616, 0.494912, 0.483691, 0.472504, 0.461358, 0.450258,
    0.439213, 0.428229, 0.417312, 0.406469, 0.395706, 0.385031, 0.374448, 0.363966, 0.353589,
    0.343325, 0.333179, 0.323158, 0.313267, 0.303513, 0.293901, 0.284438, 0.275128, 0.265978,
    0.256993, 0.248179, 0.23954, 0.231083, 0.222811, 0.214731, 0.206846, 0.199162, 0.191684,
    0.184415, 0.177361, 0.170525, 0.163911, 0.157524, 0.151367, 0.145445, 0.13976, 0.134316,
    0.129117, 0.124165, 0.119464, 0.115015, 0.110823, 0.10689, 0.103217, 0.0998074, 0.096663,
    0.0937856, 0.091177, 0.0888388, 0.0867723, 0.0849788, 0.0834594, 0.082215, 0.0812464,
    0.0805541, 0.0801385, 0.08,
];

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

const GRADIENT: [[f32; 3]; 4] = [
    [0.12, 0.09, 0.26],
    [0.54, 0.15, 0.37],
    [0.89, 0.33, 0.29],
    [1.0, 0.69, 0.02],
];

static EYE_MODE: AtomicU8 = AtomicU8::new(0);
static NAME_MODE: AtomicU8 = AtomicU8::new(0);
static NAME_SUBMODE: AtomicU8 = AtomicU8::new(0);

static mut CAP1206_I2C: Option<I2c> = None;
const CAP1206_ADDR: u8 = 0b0101000;

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

#[interrupt]
fn ADC() {
    let end = ADC_END.load(Ordering::SeqCst);
    ADC_END.store((end + 1) % FFT_SAMPLES, Ordering::SeqCst);

    unsafe {
        ADC::ptr()
            .as_ref()
            .unwrap()
            .intflag
            .modify(|_, w| w.resrdy().set_bit());
    }
}

#[interrupt]
fn EIC() {
    // Ack interrupt
    unsafe {
        CAP1206_I2C
            .as_mut()
            .unwrap()
            .write(CAP1206_ADDR, &[0x00, 0x00])
            .unwrap();
    }

    // Safety: Only this interrupt ever performs I2C transactions
    let status = unsafe {
        let mut buf = [0u8; 1];
        CAP1206_I2C
            .as_mut()
            .unwrap()
            .write_read(CAP1206_ADDR, &[0x03], &mut buf)
            .unwrap();
        buf[0]
    };

    let cur_name_mode = NAME_MODE.load(Ordering::SeqCst);
    if status & 0b00100000 > 0 {
        let mode = EYE_MODE.load(Ordering::SeqCst);
        EYE_MODE.store((mode + 1) % 3, Ordering::SeqCst);
    } else if status & 0b00010000 > 0 {
        NAME_MODE.store(0, Ordering::SeqCst);
        NAME_SUBMODE.store(0, Ordering::SeqCst);
    } else if status & 0b00001000 > 0 {
        if cur_name_mode == 1 {
            NAME_MODE.store(1, Ordering::SeqCst);
            let sub = NAME_SUBMODE.load(Ordering::SeqCst);
            NAME_SUBMODE.store((sub + 1) % 5, Ordering::SeqCst);
        } else {
            NAME_MODE.store(1, Ordering::SeqCst);
            NAME_SUBMODE.store(0, Ordering::SeqCst);
        }
    } else if status & 0b00000100 > 0 {
        NAME_MODE.store(2, Ordering::SeqCst);
        NAME_SUBMODE.store(0, Ordering::SeqCst);
    }

    // Assume we are only using EXTINT3
    unsafe {
        pac::EIC::ptr()
            .as_ref()
            .unwrap()
            .intflag
            .modify(|_, w| w.extint3().set_bit());
    }
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

    // Initialize serial clocks
    let gclk0 = clocks.gclk0();
    let spi_clock = clocks.sercom0_core(&gclk0).unwrap();
    let i2c_clock = clocks.sercom2_core(&gclk0).unwrap();

    // Initialize SPI
    let spi_pads = spi::Pads::<Sercom0>::default()
        .data_out(pins.pa04)
        .sclk(pins.pa05);
    let spi = spi::Config::new(
        &peripherals.PM,
        peripherals.SERCOM0,
        spi_pads,
        spi_clock.freq(),
    )
    .spi_mode(spi::MODE_0)
    .bit_order(BitOrder::MsbFirst)
    .baud(3200.khz())
    .char_size::<EightBit>()
    .enable();

    // Initialize IRQ pin
    let eic_clock = clocks.eic(&gclk0).unwrap();
    let mut eic = eic::EIC::init(&mut peripherals.PM, eic_clock, peripherals.EIC);
    let mut cap1206_irq = pins.pa03.into_floating_ei();
    cap1206_irq.sense(&mut eic, Sense::FALL);
    cap1206_irq.enable_interrupt(&mut eic);
    unsafe {
        NVIC::unmask(Interrupt::EIC);
    }

    // Initialize I2C
    let i2c_pads = i2c::Pads::<Sercom2, _, _>::new(pins.pa08, pins.pa09);
    let mut i2c = i2c::Config::new(
        &peripherals.PM,
        peripherals.SERCOM2,
        i2c_pads,
        i2c_clock.freq(),
    )
    .baud(50.khz())
    .enable();
    while i2c.write(CAP1206_ADDR, &[0x00, 0x00]).is_err() {} // Clear interrupts and wait for power up
    i2c.write(CAP1206_ADDR, &[0x28, 0x00]).unwrap(); // Disable repeats
    unsafe {
        CAP1206_I2C = Some(i2c);
    }

    // Mnually initialize ADC
    let gclk2 = clocks
        .configure_gclk_divider_and_source(ClockGenId::GCLK2, 1, ClockSource::OSC8M, false)
        .unwrap();
    let _adc_clock = clocks.adc(&gclk2).unwrap();
    peripherals.PM.apbcmask.modify(|_, w| w.adc_().set_bit());

    let adc_linearity = unsafe {
        (*(0x806020 as *const u32) >> 27) | (*(0x806021 as *const u32) << 5 & 0b11100000)
    } as u16;
    let adc_bias = unsafe { *(0x806021 as *const u32) >> 3 & 0b111 } as u16;
    while peripherals.ADC.status.read().syncbusy().bit() {}
    peripherals
        .ADC
        .calib
        .write(|w| unsafe { w.bits(adc_linearity & adc_bias << 8) });

    while peripherals.ADC.status.read().syncbusy().bit() {}
    peripherals.ADC.refctrl.write(|w| w.refsel().intvcc1());
    peripherals.ADC.avgctrl.write(|w| w.samplenum()._1());
    peripherals
        .ADC
        .ctrlb
        .write(|w| w.ressel()._12bit().prescaler().div256().freerun().set_bit());
    peripherals
        .ADC
        .inputctrl
        .write(|w| w.gain().div2().muxpos().pin0().muxneg().gnd());

    let _adc_pin = pins.pa02.into_alternate::<pin::B>();

    while peripherals.ADC.status.read().syncbusy().bit() {}
    peripherals.ADC.ctrla.modify(|_, w| w.enable().set_bit());
    while peripherals.ADC.status.read().syncbusy().bit() {}
    peripherals.ADC.swtrig.modify(|_, w| w.start().set_bit());

    // Initialize DMA
    let mut dmac = DmaController::init(peripherals.DMAC, &mut peripherals.PM);
    let channels = dmac.split();

    // Initialize RTC
    let gclk1 = clocks.gclk1();
    let rtc_clock = clocks.rtc(&gclk1).unwrap();
    let mut rtc = Rtc::count32_mode(peripherals.RTC, rtc_clock.freq(), &mut peripherals.PM);

    // Start SPI DMA
    let ch0 = channels.0.init(PriorityLevel::LVL0);
    let _spi_trans = unsafe { Transfer::new_unchecked(ch0, &mut DMA_BUFFER, spi, true) }
        .begin(TriggerSource::SERCOM0_TX, TriggerAction::BEAT);

    // Start ADC DMA
    let ch1 = channels.1.init(PriorityLevel::LVL0);
    let _adc_trans = unsafe {
        Transfer::new_unchecked(
            ch1,
            peripherals.ADC.result.as_ptr().as_mut().unwrap(),
            &mut ADC_BUFFER,
            true,
        )
    }
    .begin(TriggerSource::ADC_RESRDY, TriggerAction::BEAT);

    // Enable ADC interrupts
    peripherals.ADC.intflag.write(|w| w.resrdy().set_bit());
    peripherals.ADC.intenset.write(|w| w.resrdy().set_bit());
    unsafe {
        NVIC::unmask(Interrupt::ADC);
    }

    // Initialize FFT
    let fft = unsafe {
        let mut fft = MaybeUninit::<arm_cfft_instance_q15>::uninit();
        arm_cfft_init_q15(fft.as_mut_ptr(), FFT_SAMPLES as u16);
        fft.assume_init()
    };

    // Render loop
    let mut t = 0f32;
    let mut led_amp = [0f32; N_LEDS];
    loop {
        let name_mode = NAME_MODE.load(Ordering::SeqCst);
        let name_submode = NAME_SUBMODE.load(Ordering::SeqCst);

        let sub = rtc.count32();
        if sub > 32_768 {
            rtc.set_count32(0);
            t += 1.0;
        }
        let t = t + (sub % 32_768) as f32 / 32_768.0;

        if name_mode == 1 {
            // Copy into buffer
            unsafe {
                let gain = match name_submode {
                    0 => 20.0,
                    1 => 10.0,
                    2 => 5.0,
                    3 => 2.0,
                    4 => 1.0,
                    _ => 0.0,
                };

                let start = ADC_END.load(Ordering::SeqCst) + 1;
                for i in 0..FFT_SAMPLES {
                    let j = (start + i) % FFT_SAMPLES;
                    let v = ADC_BUFFER[j];
                    FFT_BUFFER[i * 2] = ((v as f32 - 2048.0) * HAMMING_LUT[i] * gain) as i16;
                    FFT_BUFFER[i * 2 + 1] = 0;
                }
            }

            // Evaluate FFT, get magnitudes, and convert to floats
            unsafe {
                arm_cfft_q15(&fft, FFT_BUFFER.as_mut_ptr(), 0, 1);
                arm_cmplx_mag_q15(
                    FFT_BUFFER.as_ptr(),
                    FFT_BUFFER.as_mut_ptr(),
                    FFT_SAMPLES as u32,
                );
                arm_q15_to_float(
                    FFT_BUFFER.as_ptr(),
                    FFT_F32.as_mut_ptr(),
                    FFT_SAMPLES as u32,
                );
                arm_scale_f32(
                    FFT_F32.as_ptr(),
                    2048.0,
                    FFT_F32.as_mut_ptr(),
                    FFT_SAMPLES as u32,
                );
            }
        }

        match EYE_MODE.load(Ordering::SeqCst) {
            0 => {
                write_pixel(0, 30, 15, 5);
            }
            1 => {
                let rgb = hsv_to_rgb((t * 0.1).fract(), 1.0, 1.0);
                write_pixel(
                    0,
                    (rgb.0 * BRIGHTNESS) as u8,
                    (rgb.1 * BRIGHTNESS) as u8,
                    (rgb.2 * BRIGHTNESS) as u8,
                );
            }
            2 => {
                write_pixel(0, 48, 0, 0);
            }
            _ => (),
        }

        for i in 1..N_LEDS {
            let (x, y) = LED_COORDS[i];

            match name_mode {
                0 => {
                    let v = anim(t, x, y);
                    write_pixel(
                        i,
                        (v.0 * BRIGHTNESS) as u8,
                        (v.1 * BRIGHTNESS) as u8,
                        (v.2 * BRIGHTNESS) as u8,
                    );
                }
                1 => {
                    let fft_bin = ((1.5 - (1.5 - x).abs()) / 2.0 * FFT_SAMPLES as f32) as i32;
                    let fft_min = (fft_bin - 16).max(1) as usize;
                    let fft_max = (fft_bin + 16).min(FFT_SAMPLES as i32 - 2) as usize;

                    let mut avg_amp = 0f32;
                    unsafe {
                        for i in fft_min..fft_max {
                            avg_amp += FFT_F32[i].sqrt();
                        }
                    }
                    avg_amp /= (fft_max - fft_min) as f32;
                    if avg_amp < 0.6 {
                        avg_amp = 0.0;
                    } else {
                        avg_amp = (avg_amp - 0.6) * 3.0;
                    }

                    let a = led_amp[i];
                    let smoothed = if avg_amp > a {
                        avg_amp
                    } else {
                        0.75 * a + 0.25 * avg_amp
                    };
                    led_amp[i] = smoothed;

                    let clamped = smoothed.min(1.0).max(0.0) * 3.0;
                    let stop = clamped as usize;
                    let next_stop = (stop + 1).min(GRADIENT.len() - 1);
                    let k = clamped.fract();
                    let [r1, g1, b1] = GRADIENT[stop];
                    let [r2, g2, b2] = GRADIENT[next_stop];
                    write_pixel(
                        i,
                        (((1.0 - k) * r1 + k * r2) * BRIGHTNESS) as u8,
                        (((1.0 - k) * g1 + k * g2) * BRIGHTNESS) as u8,
                        (((1.0 - k) * b1 + k * b2) * BRIGHTNESS) as u8,
                    );
                }
                2 => {
                    write_pixel(i, 48, 0, 0);
                }
                _ => (),
            }
        }
    }
}
