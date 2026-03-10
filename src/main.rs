#![no_std]
#![no_main]

use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};

use cortex_m_rt::entry;
use critical_section_lock_mut::LockMut;
use embedded_hal::{digital::InputPin, digital::OutputPin};
use hsv::Hsv;
use microbit::{
    board::Board,
    display::blocking::Display,
    hal::{
        Timer,
        gpio::{self, Level},
        pac::{self, interrupt},
        saadc::{Saadc, SaadcConfig},
    },
};

// const STEPS: sets how many steps there are in a frame (used by the PWM interrupt handler)
const STEPS: u32 = 100;
// const DURATION: sets how long each step lasts (used by the PWM interrupt handler)
const DURATION: u32 = 100;
// const DELAY: sets the delay interval for the superloop (not the PWM interrupt handler)
const DELAY: u32 = 10;
// const MAX_POT: sets the max value to be stored when reading the potentiometer
const MAX_POT: i16 = 0x3FFF;

// Frames used by Microbit LED matrix to denote currently selected HSV mode
type Frame = [[u8; 5]; 5];

#[rustfmt::skip]
const FRAME_H: Frame = [
    [1,0,0,0,1],
    [1,0,0,0,1],
    [1,1,1,1,1],
    [1,0,0,0,1],
    [1,0,0,0,1],
];

#[rustfmt::skip]
const FRAME_S: Frame = [
    [1,1,1,1,1],
    [1,0,0,0,0],
    [1,1,1,1,1],
    [0,0,0,0,1],
    [1,1,1,1,1],
];

#[rustfmt::skip]
const FRAME_V: Frame = [
    [1,0,0,0,1],
    [1,0,0,0,1],
    [1,0,0,0,1],
    [0,1,0,1,0],
    [0,0,1,0,0],
];

const FRAMES: [Frame; 3] = [FRAME_H, FRAME_S, FRAME_V];

static RGB: LockMut<RgbDisplay> = LockMut::new();

#[interrupt]
fn TIMER0() {
    RGB.with_lock(|rgb| rgb.step());
}

struct RgbDisplay {
    tick: usize,
    schedule_channels: [usize; 3],
    schedule_times: [u32; 3],
    next_schedule: Option<[u32; 3]>,
    rgb_pins: [gpio::Pin<gpio::Output<gpio::PushPull>>; 3],
    timer0: Timer<pac::TIMER0>,
}

impl RgbDisplay {
    fn new(
        rgb_pins: [gpio::Pin<gpio::Output<gpio::PushPull>>; 3],
        timer0: Timer<pac::TIMER0>,
    ) -> Self {
        Self {
            tick: 0,
            schedule_channels: [0, 1, 2],
            schedule_times: [10; 3],
            next_schedule: Some([10; 3]),
            rgb_pins,
            timer0,
        }
    }

    fn set(&mut self, hsv: &Hsv) {
        self.timer0.enable_interrupt();
        let rgb_obj = hsv.to_rgb();
        // Normalizes vaules and converts them to u32
        let r_n = (rgb_obj.r * 100.0) as u32;
        let g_n = (rgb_obj.g * 100.0) as u32;
        let b_n = (rgb_obj.b * 100.0) as u32;

        self.next_schedule = Some([r_n, g_n, b_n]);
    }

    fn step(&mut self) {
        let mut time_delay = 0;
        if self.tick == 0 {
            if let Some(n) = self.next_schedule.take() {
                let next = n;
                struct PinTime {
                    pin: usize,
                    duty: u32,
                }

                let mut pt = [
                    PinTime {
                        pin: 0,
                        duty: next[0],
                    },
                    PinTime {
                        pin: 1,
                        duty: next[1],
                    },
                    PinTime {
                        pin: 2,
                        duty: next[2],
                    },
                ];

                if pt[0].duty > pt[1].duty {
                    pt.swap(0, 1);
                }
                if pt[1].duty > pt[2].duty {
                    pt.swap(1, 2);
                }
                if pt[0].duty > pt[1].duty {
                    pt.swap(0, 1);
                }

                // Calculates steps
                let t1 = pt[0].duty;
                let t2 = pt[1].duty.saturating_sub(pt[0].duty);
                let t3 = pt[2].duty.saturating_sub(pt[1].duty);

                self.schedule_channels = [pt[0].pin, pt[1].pin, pt[2].pin];
                self.schedule_times = [t1, t2, t3];

                for p in self.rgb_pins.iter_mut() {
                    p.set_low().unwrap();
                }
                time_delay = self.schedule_times[0];
                self.tick = 1;
            }
        } else if (1..3).contains(&self.tick) {
            self.rgb_pins[self.schedule_channels[self.tick - 1]]
                .set_high()
                .unwrap();
            time_delay = self.schedule_times[self.tick];
            self.tick += 1;
        } else if self.tick == 3 {
            self.rgb_pins[self.schedule_channels[self.tick - 1]]
                .set_high()
                .unwrap();
            time_delay = STEPS.saturating_sub(
                self.schedule_times[0] + self.schedule_times[1] + self.schedule_times[2],
            );
            self.tick = 0;
        }
        self.timer0.reset_event();
        self.timer0.start((time_delay * DURATION).max(100));
    }
}

#[entry]
fn main() -> ! {
    rtt_init_print!();
    let board = Board::take().unwrap();
    let timer = Timer::new(board.TIMER0);
    let mut timer1 = Timer::new(board.TIMER1);
    let pin_r = board.edge.e08.into_push_pull_output(Level::Low);
    let pin_g = board.edge.e09.into_push_pull_output(Level::Low);
    let pin_b = board.edge.e16.into_push_pull_output(Level::Low);
    let pins = [pin_r.degrade(), pin_g.degrade(), pin_b.degrade()];

    let mut display = Display::new(board.display_pins);

    let mut button_a = board.buttons.button_a;
    let mut button_b = board.buttons.button_b;

    let mut button_a_state = false;
    let mut button_b_state = false;

    let mut current_mode: usize = 0;

    let mut saadc = Saadc::new(board.ADC, SaadcConfig::default());
    let mut pin_pot = board.edge.e02;

    let mut hsv_obj = Hsv {
        h: 0.4,
        s: 1.0,
        v: 1.0,
    };

    unsafe { pac::NVIC::unmask(pac::Interrupt::TIMER0) };
    pac::NVIC::unpend(pac::Interrupt::TIMER0);

    let rgb = RgbDisplay::new(pins, timer);
    RGB.init(rgb);

    RGB.with_lock(|rgb| rgb.set(&hsv_obj));

    RGB.with_lock(|rgb| rgb.step());

    rprintln!("Start HSV");
    loop {
        /*
            Captures a and b button presses on the microbit.
            - a button pressed: moves back one mode
            - b button pressed: moves forward one mode
            - both buttons pressed: ignored
            - held button press: counted only once
            * mode array wraps at both ends
        */
        if button_a.is_low().unwrap() && button_b.is_high().unwrap() && !button_a_state {
            current_mode = (current_mode + 3 - 1) % 3;
            button_a_state = true;
        } else if button_b.is_low().unwrap() && button_a.is_high().unwrap() && !button_b_state {
            current_mode = (current_mode + 1) % 3;
            button_b_state = true;
        }

        // Checks if the buttons are unpressed for at least one tick
        if button_a.is_high().unwrap() && button_a_state {
            button_a_state = false;
        }
        if button_b.is_high().unwrap() && button_b_state {
            button_b_state = false;
        }

        if let Ok(value) = saadc.read_channel(&mut pin_pot) {
            let value = value.clamp(0, MAX_POT) as f32 / MAX_POT as f32;
            if current_mode == 0 {
                hsv_obj.h = value;
            } else if current_mode == 1 {
                hsv_obj.s = value;
            } else if current_mode == 2 {
                hsv_obj.v = value;
            }
        }

        RGB.with_lock(|rgb| rgb.set(&hsv_obj));

        display.show(&mut timer1, FRAMES[current_mode], DELAY);
    }
}
