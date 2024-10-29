//! Serial interface loopback test
//!
//! You have to short the TX and RX pins to make this program work

#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m::asm;

use nb::block;
use pac::interrupt;

use core::mem::MaybeUninit;
use cortex_m_rt::entry;
use stm32f1xx_hal::gpio::*;
use stm32f1xx_hal::{pac, prelude::*, serial::Config};

static mut INT_PIN: MaybeUninit<stm32f1xx_hal::gpio::gpioa::PA6<Input<Floating>>> =
    MaybeUninit::uninit();
static mut DATA_PIN: MaybeUninit<stm32f1xx_hal::gpio::gpioa::PA7<Input<Floating>>> =
    MaybeUninit::uninit();


static mut DONE_MOVING: bool = false;
static mut CLOCK_COUNT: usize = 0;
static mut MOVEMENT_BUF: [u8; 33] = [0; 33];

#[interrupt]
fn EXTI9_5() {
    let int_pin = unsafe { &mut *INT_PIN.as_mut_ptr() };
    let data_pin = unsafe { &mut *DATA_PIN.as_mut_ptr() };

    if int_pin.check_interrupt() {
        if data_pin.is_high(){
            unsafe {MOVEMENT_BUF[CLOCK_COUNT] = b'1';}
        } else {
            unsafe {MOVEMENT_BUF[CLOCK_COUNT] = b'0';}
        }

        // if we don't clear this bit, the ISR would trigger indefinitely
        unsafe {
            if CLOCK_COUNT < 32 {
                CLOCK_COUNT += 1;
                DONE_MOVING = false;
            } else {
                if MOVEMENT_BUF[24] == 1 || MOVEMENT_BUF[25] == 1 {
                } else {
                }
                DONE_MOVING = true;
                CLOCK_COUNT = 0;
            }
        }
        int_pin.clear_interrupt_pending_bit();
    }
}


#[entry]
fn main() -> ! {
    // Get access to the device specific peripherals from the peripheral access crate
    let mut p = pac::Peripherals::take().unwrap();

    // Take ownership over the raw flash and rcc devices and convert them into the corresponding
    // HAL structs
    let mut flash = p.FLASH.constrain();
    let rcc = p.RCC.constrain();

    // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
    // `clocks`
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    // Prepare the alternate function I/O registers
    //let mut afio = p.AFIO.constrain();

    // Prepare the GPIOB peripheral
    let mut gpiob = p.GPIOB.split();

    // USART1
    // let tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
    // let rx = gpioa.pa10;

    // USART1
    // let tx = gpiob.pb6.into_alternate_push_pull(&mut gpiob.crl);
    // let rx = gpiob.pb7;

    // USART2
    // let tx = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
    // let rx = gpioa.pa3;

    // USART3
    // Configure pb10 as a push_pull output, this will be the tx pin
    let tx = gpiob.pb10.into_alternate_push_pull(&mut gpiob.crh);
    // Take ownership over pb11
    let rx = gpiob.pb11;

    // Set up the usart device. Take ownership over the USART register and tx/rx pins. The rest of
    // the registers are used to enable and configure the device.
    let mut serial = p
        .USART3
        .serial((tx, rx), Config::default().baudrate(115200.bps()), &clocks);

    {
        // the scope ensures that the int_pin reference is dropped before the first ISR can be executed.

        let mut gpioa = p.GPIOA.split();
        let mut afio = p.AFIO.constrain();

        let int_pin = unsafe { &mut *INT_PIN.as_mut_ptr() };
        *int_pin = gpioa.pa6.into_floating_input(&mut gpioa.crl);
        int_pin.make_interrupt_source(&mut afio);
        int_pin.trigger_on_edge(&mut p.EXTI, Edge::Falling);
        int_pin.enable_interrupt(&mut p.EXTI);
    } // initialization ends here

    unsafe {
        pac::NVIC::unmask(pac::Interrupt::EXTI9_5);
    }


    loop {
        unsafe {
            if DONE_MOVING == true {
                for i in 0..32 {
                    let sent = MOVEMENT_BUF[i];
                    block!(serial.tx.write_u8(sent)).unwrap();
                    DONE_MOVING = false;
                }
                block!(serial.tx.write_u8(10)).unwrap();
            }
        }
    }
}
