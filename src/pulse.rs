#[derive(PartialEq, Eq, PartialOrd, Ord, Debug, Clone, Copy)]
pub enum Polarity {
    Positive,
    Negative,
}

/// Single +- or -+ wiggle
struct Pulse {
    p0: Option<Polarity>,
}

impl Pulse {
    pub fn new() -> Pulse {
        Pulse { p0: None }
    }

    pub fn pulse(&mut self, p: Polarity) -> Result<Option<Polarity>, ()> {
        match (self.p0, p) {
            (None, _) => {
                self.p0 = Some(p);
                Ok(None)
            }
            (Some(Polarity::Negative), Polarity::Positive) => {
                self.reset();
                Ok(Some(Polarity::Positive))
            }
            (Some(Polarity::Positive), Polarity::Negative) => {
                self.reset();
                Ok(Some(Polarity::Negative))
            }
            _ => {
                self.reset();
                Err(())
            }
        }
    }

    #[inline]
    pub fn reset(&mut self) {
        self.p0 = None;
    }
}

enum State {
    Initial,
    FirstBit(Polarity),
    Bit(u8),
}

pub struct Frame {
    bits: u16, // decoded bits: 3 control + 8 data bits
    state: State,
    pulse: Pulse,
}

impl Frame {
    pub fn new() -> Self {
        Frame {
            bits: 0,
            state: State::Initial,
            pulse: Pulse::new(),
        }
    }

    pub fn pulse(&mut self, pp: Polarity) -> Option<u16> {
        match self.pulse.pulse(pp) {
            Ok(None) => None,
            Err(_) => {
                self.reset();
                None
            }
            Ok(Some(p)) => match self.state {
                State::Initial => {
                    self.state = State::FirstBit(p);
                    None
                }
                State::FirstBit(fp) => match (fp, p) {
                    (Polarity::Positive, Polarity::Positive) => {
                        self.capture_bit(1);
                        self.state = State::Bit(1);
                        None
                    }
                    (Polarity::Negative, Polarity::Negative) => {
                        self.capture_bit(0);
                        self.state = State::Bit(1);
                        None
                    }
                    _ => {
                        self.reset();
                        None
                    }
                },
                State::Bit(10) => {
                    self.capture_pulse(p);
                    let res = self.bits;
                    self.reset();
                    Some(res)
                }
                State::Bit(n) => {
                    self.capture_pulse(p);
                    self.state = State::Bit(n + 1);
                    None
                }
            },
        }
    }

    #[inline]
    fn capture_pulse(&mut self, p: Polarity) {
        match p {
            Polarity::Positive => {
                self.capture_bit(1);
            }
            Polarity::Negative => {
                self.capture_bit(0);
            }
        }
    }

    #[inline]
    fn capture_bit(&mut self, b: u16) {
        self.bits = (self.bits << 1) | (b & 1)
    }

    fn reset(&mut self) {
        self.bits = 0;
        self.state = State::Initial;
        self.pulse.reset();
    }
}
