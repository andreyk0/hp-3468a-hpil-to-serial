#![no_std]
#![deny(unsafe_code)]
#![cfg_attr(not(doc), no_main)]

use panic_halt as _;

use cortex_m::asm;
//use cortex_m_semihosting::hprintln;

use stm32f1xx_hal::{
    gpio,
    gpio::Alternate,
    gpio::Edge,
    gpio::ExtiPin,
    gpio::Floating,
    gpio::Input,
    gpio::OpenDrain,
    i2c::{BlockingI2c, DutyCycle, Mode},
    prelude::*,
    usb,
};

use stm32f1xx_hal::pac::{EXTI, I2C1};

use embedded_hal::digital::v2::OutputPin;

use rtic::cyccnt::Duration;

use heapless::spsc::{Consumer, Producer, Queue};
use heapless::String;

use usb_device::bus;

use hpil_to_serial::{consts::*, display::*, pulse::*, types::*, usb_serial::UsbSerialDevice};

#[rtic::app(device = stm32f1xx_hal::stm32,
            peripherals = true,
            monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        exti: EXTI,
        led: LedPin,
        usb_serial: UsbSerial,
        hpil1: gpio::gpioa::PA0<Input<Floating>>,
        hpil2: gpio::gpioa::PA1<Input<Floating>>,
        pulse_producer: Producer<'static, Polarity, 512>,
        pulse_consumer: Consumer<'static, Polarity, 512>,
        display: Display<
            BlockingI2c<
                I2C1,
                (
                    gpio::gpiob::PB6<Alternate<OpenDrain>>,
                    gpio::gpiob::PB7<Alternate<OpenDrain>>,
                ),
            >,
        >,
    }

    #[init(schedule = [blink])]
    fn init(cx: init::Context) -> init::LateResources {
        static mut USB_BUS: Option<bus::UsbBusAllocator<usb::UsbBus<usb::Peripheral>>> = None;

        static mut PULSE_QUEUE: Queue<Polarity, 512> = Queue::new();
        let (pulse_producer, pulse_consumer) = PULSE_QUEUE.split();

        let mut core: rtic::Peripherals = cx.core;
        let mut device = cx.device;
        let mut flash = device.FLASH.constrain();
        let mut rcc = device.RCC.constrain();

        let clocks = rcc
            .cfgr
            .use_hse(8.mhz())
            .sysclk(SYS_FREQ)
            .pclk1(36.mhz())
            .freeze(&mut flash.acr);

        assert!(clocks.usbclk_valid());

        let mut afio = device.AFIO.constrain(&mut rcc.apb2);

        //hprintln!("clocks").unwrap();

        let mut gpioa = device.GPIOA.split(&mut rcc.apb2);

        let mut hpil1 = gpioa.pa0.into_floating_input(&mut gpioa.crl);
        hpil1.make_interrupt_source(&mut afio);
        hpil1.trigger_on_edge(&mut device.EXTI, Edge::RISING);

        let mut hpil2 = gpioa.pa1.into_floating_input(&mut gpioa.crl);
        hpil2.make_interrupt_source(&mut afio);
        hpil2.trigger_on_edge(&mut device.EXTI, Edge::RISING);

        // USB serial

        // BluePill board has a pull-up resistor on the D+ line.
        // Pull the D+ pin down to send a RESET condition to the USB bus.
        // This forced reset is needed only for development, without it host
        // will not reset your device when you upload new firmware.
        let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
        usb_dp.set_low().unwrap();
        asm::delay(SYS_FREQ.0 / 10);

        let usb_dm = gpioa.pa11;
        let usb_dp = usb_dp.into_floating_input(&mut gpioa.crh);

        let usbp = usb::Peripheral {
            usb: device.USB,
            pin_dm: usb_dm,
            pin_dp: usb_dp,
        };

        //hprintln!("usbp").unwrap();

        *USB_BUS = Some(usb::UsbBus::new(usbp));
        let usb_bus = USB_BUS.as_ref().unwrap();

        let usb_serial = UsbSerialDevice::new(usb_bus);

        //hprintln!("usb_bus").unwrap();

        //let mut gpiob = device.GPIOB.split(&mut rcc.apb2);
        let mut gpioc = device.GPIOC.split(&mut rcc.apb2);

        //hprintln!("gpio").unwrap();

        let led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

        // Initialize (enable) the monotonic timer (CYCCNT)
        core.DCB.enable_trace();
        // required on Cortex-M7 devices that software lock the DWT (e.g. STM32F7)
        cortex_m::peripheral::DWT::unlock();
        core.DWT.enable_cycle_counter();

        cx.schedule
            .blink(cx.start + Duration::from_cycles(SYS_FREQ.0 / 2))
            .unwrap();

        let mut gpiob = device.GPIOB.split(&mut rcc.apb2);
        let sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);
        let scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);

        let i2c = BlockingI2c::i2c1(
            device.I2C1,
            (scl, sda),
            &mut afio.mapr,
            Mode::Fast {
                frequency: 400_000.hz(),
                duty_cycle: DutyCycle::Ratio16to9,
            },
            clocks,
            &mut rcc.apb1,
            1000,
            10,
            1000,
            1000,
        );

        let display = Display::new(i2c);

        //hprintln!("init::LateResources").unwrap();
        init::LateResources {
            exti: device.EXTI,
            led,
            usb_serial,
            hpil1,
            hpil2,
            pulse_producer,
            pulse_consumer,
            display,
        }
    }

    #[idle(resources = [usb_serial, pulse_consumer, hpil1, hpil2, exti, display])]
    fn idle(mut cx: idle::Context) -> ! {
        let pulses = cx.resources.pulse_consumer;
        let mut usbs_lock = cx.resources.usb_serial;
        let mut ss: String<32> = String::new();
        let mut frame = Frame::new();

        cx.resources.display.update("Connecting").unwrap();

        asm::delay(SYS_FREQ.0);
        let mut exti = cx.resources.exti;
        cx.resources.hpil1.lock(|x| x.enable_interrupt(&mut exti));
        cx.resources.hpil2.lock(|x| x.enable_interrupt(&mut exti));

        loop {
            for p in pulses.dequeue() {
                for w in frame.pulse(p) {
                    let data = w & 0xFF;
                    for ch in char::from_u32(data as u32) {
                        ss.push(ch).map_err(|_| ss.clear()).ok();
                        if ch == '\n' {
                            usbs_lock.lock(|usbs| usbs.write(ss.as_bytes())).unwrap();
                            cx.resources.display.update(&ss).unwrap();
                            ss.clear();
                        }
                    }
                }
            }
        }
    }

    #[task(resources = [led],
           schedule = [blink],
           priority = 1)]
    fn blink(cx: blink::Context) {
        cx.resources.led.toggle().unwrap();
        cx.schedule
            .blink(cx.scheduled + Duration::from_cycles(SYS_FREQ.0 / 2))
            .unwrap();
    }

    #[task(binds = EXTI0,
            resources = [pulse_producer, hpil1],
            priority = 2)]
    fn pulse1(cx: pulse1::Context) {
        cx.resources.hpil1.clear_interrupt_pending_bit();
        cx.resources
            .pulse_producer
            .enqueue(Polarity::Negative)
            .unwrap();
    }

    #[task(binds = EXTI1,
            resources = [pulse_producer, hpil2],
            priority = 2)]
    fn pulse2(cx: pulse2::Context) {
        cx.resources.hpil2.clear_interrupt_pending_bit();
        cx.resources
            .pulse_producer
            .enqueue(Polarity::Positive)
            .unwrap();
    }

    #[task(binds = USB_LP_CAN_RX0,
           resources = [usb_serial],
           priority = 3)]
    fn usb_rx(mut cx: usb_rx::Context) {
        cx.resources.usb_serial.lock(|usbs| {
            usbs.poll();
            usbs.drain_input();
        });
    }

    #[task(binds = USB_HP_CAN_TX,
           resources = [usb_serial],
           priority = 4)]
    fn usb_tx(cx: usb_tx::Context) {
        cx.resources.usb_serial.poll();
    }

    // RTIC requires that unused interrupts are declared in an extern block when
    // using software tasks; these free interrupts will be used to dispatch the
    // software tasks.
    // Full list in  stm32f1::stm32f103::Interrupt
    extern "C" {
        fn FSMC();
        fn TAMPER();
        fn DMA1_CHANNEL1();
        fn DMA1_CHANNEL2();
        fn DMA1_CHANNEL3();
        fn DMA1_CHANNEL4();
        fn DMA1_CHANNEL5();
        fn DMA1_CHANNEL6();
        fn DMA1_CHANNEL7();
    }
};
