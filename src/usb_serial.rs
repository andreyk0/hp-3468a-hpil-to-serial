use usb_device::{bus, prelude::*, UsbError::WouldBlock};
use usbd_serial::{DefaultBufferStore, SerialPort};

pub struct UsbSerialDevice<'a, B>
where
    B: bus::UsbBus,
{
    serial_port: SerialPort<'a, B, DefaultBufferStore, DefaultBufferStore>,
    usb_device: UsbDevice<'a, B>,
}

impl<B> UsbSerialDevice<'_, B>
where
    B: bus::UsbBus,
{
    /// New usb serial
    pub fn new<'a>(usb_bus: &'a bus::UsbBusAllocator<B>) -> UsbSerialDevice<'a, B> {
        // this has to go before UsbDeviceBuilder, it mutably borrows from
        // refcells but doesn't exit scope and anything else trying to do
        // the same panics in refcell's borrow mut call
        let serial_port = SerialPort::new(&usb_bus);

        let usb_device = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27de))
            .manufacturer("DIY")
            .product("HP3468A")
            .serial_number("1")
            .device_class(usbd_serial::USB_CLASS_CDC)
            .build();

        UsbSerialDevice {
            serial_port,
            usb_device,
        }
    }

    /// Poll periodically
    #[inline]
    pub fn poll(&mut self) {
        if !self.usb_device.poll(&mut [&mut self.serial_port]) {
            // https://github.com/mvirkkunen/usb-device/issues/32
            usb_device::class::UsbClass::poll(&mut self.serial_port);
        }
    }

    /// Serial write all bytes out when possible
    pub fn write(&mut self, data: &[u8]) -> Result<(), UsbError> {
        if self.serial_port.dtr() {
            let mut n = 0;
            while n < data.len() - 1 {
                match self.serial_port.write(&data[n..]) {
                    Ok(s) => {
                        n += s;
                    }
                    Err(WouldBlock) => self.poll(),
                    e => {
                        e?;
                    }
                }
            }
        } else {
            // ignore quietly if USB is not set up
            self.poll()
        }

        Ok(())
    }

    pub fn drain_input(&mut self) {
        let mut buf: [u8; 8] = [0; 8];
        self.serial_port.read(&mut buf).ok();
    }
}
