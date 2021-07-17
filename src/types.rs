use stm32f1xx_hal::{gpio::*, usb};

use crate::usb_serial::UsbSerialDevice;

pub type LedPin = gpioc::PC13<Output<PushPull>>;

pub type UsbSerial = UsbSerialDevice<'static, usb::UsbBus<usb::Peripheral>>;
