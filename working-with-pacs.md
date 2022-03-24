# References

- Blog post: [STM32L0 Rust Part 1 - Getting Started](https://craigjb.com/2019/12/31/stm32l0-rust/)
- Datasheet: [RM0351
Reference manual
STM32L47xxx, STM32L48xxx, STM32L49xxx and STM32L4Axxx
advanced Arm®-based 32-bit MCUs](https://www.st.com/resource/en/reference_manual/rm0351-stm32l47xxx-stm32l48xxx-stm32l49xxx-and-stm32l4axxx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- List: [Curated list of resources for Embedded and Low-level development in the Rust programming language](https://github.com/rust-embedded/awesome-embedded-rust)
- The Embedded Rust Book: [Recommendations for GPIO Interfaces](https://doc.rust-lang.org/beta/embedded-book/design-patterns/hal/gpio.html)
- Discovery (embedded Rust book): [Rust Embedded terminology](https://docs.rust-embedded.org/discovery/microbit/04-meet-your-hardware/terminology.html)
- Video: ["Type-Driven API Design in Rust" by Will Crichton](https://www.youtube.com/watch?v=bnnacleqg6k)
- Blog post: [Type-level Programming in Rust](https://willcrichton.net/notes/type-level-programming/)
- Blog post: [Typestates in Rust](https://yoric.github.io/post/rust-typestate/)
- Blog post: [Using Type-Level Programming in Rust to Make Safer Hardware Abstractions](https://blog.auxon.io/2019/10/25/type-level-registers/)
- Blog post: [The GPIO war: macro bunkers for typestate explosions](https://www.ecorax.net/macro-bunker-1/) and [part 2](https://www.ecorax.net/macro-bunker-2/)

# Mapping From SVD to Datasheet
## Blog Post

Post says:

> Peripheral Access Crate (PAC): These crates provide helpers for accessing the registers on parts using a more type-safe Rust API. They are generated using a tool called svd2rust, which takes in a System View Description (SVD) xml file that describes all the registers and their bit-fields. This file is actually required by ARM from licensees. The vendor SVD files are not always great, so several projects have patched them to clean-up the peripheral access APIs. For example, the stm32-rs project has PACs for a bunch of STM32 parts.

Contains example XML snippet of OTYPER:

```
<register>
    <name>OTYPER</name>
    <displayName>OTYPER</displayName>
    <description>GPIO port output type register</description>
    <addressOffset>0x4</addressOffset>
    <size>0x20</size>
    <access>read-write</access>
    <resetValue>0x00000000</resetValue>
    <fields>
    <field>
        <name>OT15</name>
        <description>Port x configuration bits (y =
        0..15)</description>
        <bitOffset>15</bitOffset>
        <bitWidth>1</bitWidth>
    </field>
    ...
    </fields>
</register>
```

## Datasheet

Datasheet P. 304:

![Screen Shot 2022-01-16 at 3 11 55 PM](https://user-images.githubusercontent.com/73720500/149676347-f57a70d0-52a7-47c2-bd8b-dc9f81b58fc1.png)

Note matching information between the XML fields and the datasheet diagram.

Datasheet P. 312-313 contains summary of GPIO register map:

![Screen Shot 2022-01-16 at 3 17 59 PM](https://user-images.githubusercontent.com/73720500/149676572-5c2dc5e2-2e5d-4bac-b249-5cd2e93ac65c.png)
![Screen Shot 2022-01-16 at 3 18 28 PM](https://user-images.githubusercontent.com/73720500/149676583-86b35442-d411-4189-8a92-a1689fbf7075.png)
![Screen Shot 2022-01-16 at 3 18 55 PM](https://user-images.githubusercontent.com/73720500/149676587-2d4b7369-85f7-4f91-92f3-0cc39c3d3145.png)

# Embedded HAL

- Crate [embedded-hal](https://crates.io/crates/embedded-hal)
- Docs [Crate embedded_hal](https://docs.rs/embedded-hal/0.2.6/embedded_hal/)

# NRF Crates

## NRF HAL

- [Crate nrf52_hal](https://docs.rs/nrf52-hal/0.0.1/nrf52_hal/)
  - Reexports [Crate nrf52](https://docs.rs/nrf52/0.1.0/nrf52/index.html)
  - How to use [Peripheral API](https://docs.rs/svd2rust/0.12.0/svd2rust/#peripheral-api)
    - Peripherals::take method
  - Contains [Struct nrf52::Peripherals](https://docs.rs/nrf52/0.1.0/nrf52/struct.Peripherals.html)
  - P0 GPIO

## NRF 52840

Nordic spec (datasheet): [nRF52840 Product Specification v1.7](https://infocenter.nordicsemi.com/pdf/nRF52840_PS_v1.7.pdf)

[All nRF52840 crates](https://crates.io/search?q=nrf52840)

- [adafruit-nrf52840-express](https://crates.io/crates/adafruit-nrf52840-express)
- [adafruit-nrf52840-sense](https://crates.io/crates/adafruit-nrf52840-sense)
- [Crate nrf52840_pac](https://docs.rs/nrf52840-pac/0.10.1/nrf52840_pac/)
  - Repo (includes SVD files): [PACs for nRF microcontrollers](https://github.com/nrf-rs/nrf-pacs)
    - Says: For a more user-friendly interface to the peripherals, the nrf-hal crates might be more appropriate.
  - [Module nrf52840_pac::p0](https://docs.rs/nrf52840-pac/0.10.1/nrf52840_pac/p0/index.html)
    - [Struct nrf52840_pac::p0::RegisterBlock](https://docs.rs/nrf52840-pac/0.10.1/nrf52840_pac/p0/struct.RegisterBlock.html)
  - [Struct nrf52840_pac::P0](https://docs.rs/nrf52840-pac/0.10.1/nrf52840_pac/struct.P0.html)
  - [Struct nrf52840_pac::P1](https://docs.rs/nrf52840-pac/0.10.1/nrf52840_pac/struct.P1.html)

# NRF HAL Crates

- Repo [nrf-hal](https://github.com/nrf-rs/nrf-hal/)

## Examples

### [Blinky button demo](https://github.com/nrf-rs/nrf-hal/blob/master/examples/blinky-button-demo)
- [main.rs](https://github.com/nrf-rs/nrf-hal/blob/master/examples/blinky-button-demo/src/main.rs)

```
#![no_main]
#![no_std]

use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;
use nrf52832_hal as hal;
use nrf52832_hal::gpio::Level;
use rtt_target::{rprintln, rtt_init_print};

#[panic_handler] // panicking behavior
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    rtt_init_print!();
    let p = hal::pac::Peripherals::take().unwrap();
    let port0 = hal::gpio::p0::Parts::new(p.P0);
    let button = port0.p0_13.into_pullup_input();
    let mut led = port0.p0_17.into_push_pull_output(Level::Low);

    rprintln!("Blinky button demo starting");
    loop {
        if button.is_high().unwrap() {
            led.set_high().unwrap();
        } else {
            led.set_low().unwrap();
        }
    }
}
```

### [SPI Master Demo](https://github.com/nrf-rs/nrf-hal/tree/master/examples/spi-demo)

[main.rs](https://github.com/nrf-rs/nrf-hal/blob/master/examples/spi-demo/src/main.rs)

```
#![no_std]
#![no_main]

extern crate cortex_m_rt as rt; // v0.5.x

extern crate embedded_hal_spy;
extern crate nrf52832_hal;
extern crate panic_halt;
use embedded_hal::blocking::spi::*;

use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
use nrf52832_hal::gpio;
use nrf52832_hal::gpio::p0::*;
use nrf52832_hal::gpio::Level;
use nrf52832_hal::gpio::*;
use nrf52832_hal::spim::Spim;

/// SPIM demonstation code.
/// connect a resistor between pin 22 and 23 on to feed MOSI direct back to MISO
///
/// If all tests Led1 to 4 will light up, in case of error only the failing test
/// one or more Led will remain off.
#[entry]
fn main() -> ! {
    let p = nrf52832_hal::pac::Peripherals::take().unwrap();
    let port0 = p0::Parts::new(p.P0);

    let cs: P0_21<gpio::Output<PushPull>> = port0.p0_21.into_push_pull_output(Level::Low);

    let mut led1: P0_17<gpio::Output<PushPull>> = port0.p0_17.into_push_pull_output(Level::High);
    let mut led2: P0_18<gpio::Output<PushPull>> = port0.p0_18.into_push_pull_output(Level::High);
    let mut led3: P0_19<gpio::Output<PushPull>> = port0.p0_19.into_push_pull_output(Level::High);
    let mut led4: P0_20<gpio::Output<PushPull>> = port0.p0_20.into_push_pull_output(Level::High);

    let _btn1 = port0.p0_13.into_pullup_input();
    let _btn2 = port0.p0_14.into_pullup_input();
    let _btn3 = port0.p0_15.into_pullup_input();
    let _btn4 = port0.p0_16.into_pullup_input();

    let spiclk = port0.p0_24.into_push_pull_output(Level::Low).degrade();
    let spimosi = port0.p0_23.into_push_pull_output(Level::Low).degrade();
    let spimiso = port0.p0_22.into_floating_input().degrade();

    let mut tests_ok = true;
    let pins = nrf52832_hal::spim::Pins {
        sck: spiclk,
        miso: Some(spimiso),
        mosi: Some(spimosi),
    };
    let mut spi = Spim::new(
        p.SPIM2,
        pins,
        nrf52832_hal::spim::Frequency::K500,
        nrf52832_hal::spim::MODE_0,
        0,
    );

    let reference_data = b"Hello,echo Loopback";
    // Read only test vector
    let test_vec1 = *reference_data;
    let mut readbuf = [0; 255];

    // This will write 8 bytes, then shift out ORC

    // Note: `spi.transfer_split_uneven(&mut cs.degrade(), reference_data, &mut readbuf)`
    //       will fail because reference data is in flash, the copy to an array
    //       will move it to RAM.

    match spi.transfer_split_uneven(&mut cs.degrade(), &test_vec1, &mut readbuf) {
        Ok(_) => {
            for i in 0..test_vec1.len() {
                tests_ok &= test_vec1[i] == readbuf[i];
            }
            if !tests_ok {
                led1.set_low().unwrap();
            } else {
                const ORC: u8 = 0;
                for i in test_vec1.len()..readbuf.len() {
                    if ORC != readbuf[i] {
                        tests_ok = false;
                        led1.set_low().unwrap();
                    }
                }
            }
        }
        Err(_) => {
            tests_ok = false;
            led1.set_low().unwrap();
        }
    }

    // Wrap interface in embedded-hal-spy to access embedded_hal traits
    let mut eh_spi = embedded_hal_spy::new(spi, |_| {});
    use embedded_hal::blocking::spi::Write;
    match eh_spi.write(reference_data) {
        Ok(_) => {}
        Err(_) => {
            tests_ok = false;
            led2.set_low().unwrap()
        }
    }

    let mut test_vec2 = *reference_data;
    match eh_spi.transfer(&mut test_vec2) {
        Ok(_) => {
            for i in 0..test_vec2.len() {
                if test_vec2[i] != reference_data[i] {
                    tests_ok = false;
                    led3.set_low().unwrap();
                }
            }
        }
        Err(_) => {
            tests_ok = false;
            led4.set_low().unwrap();
        }
    }

    if tests_ok {
        led1.set_low().unwrap();
        led2.set_low().unwrap();
        led3.set_low().unwrap();
        led4.set_low().unwrap();
    }

    loop {}
}
```
# Typestate

TODO: add explanation of typestate and type-level programming to illustrate "typestate machine" as used in above demo to transform pin.
