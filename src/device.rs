#![allow(unused)]

/// Number of RX pipes with configurable addresses
pub const PIPES_COUNT: usize = 6;
/// Minimum address length
pub const MIN_ADDR_BYTES: usize = 2;
/// Maximum address length
pub const MAX_ADDR_BYTES: usize = 5;

pub const R_REGISTER: u8 = 0b_0000_0000;
pub const W_REGISTER: u8 = 0b_0010_0000;
pub const R_RX_PAYLOAD: u8 = 0b_0110_0001;
pub const W_TX_PAYLOAD: u8 = 0b_1010_0000;
pub const FLUX_TX: u8 = 0b_1110_0001;
pub const FLUX_RX: u8 = 0b_1110_0010;
pub const REUSE_TX_PL: u8 = 0b_1110_0011;
pub const R_RX_PL_WID: u8 = 0b_0110_0000;
pub const W_ACK_PAYLOAD: u8 = 0b_1010_1000;
pub const W_TX_P_NO_ACK: u8 = 0b_1011_0000;
pub const NOP: u8 = 0b_1111_1111;

use bitfield::bitfield;

pub trait Register {
	/// Address in the register map
	fn new(value: u8) -> Self;
	fn addr() -> u8;
	fn value(&self) -> u8;
}

/// Common for all registers with 1 bytes of data
macro_rules! register {
	($name: ident, $addr: expr) => {
		register!($name, $addr, 0);
	};
	($name: ident, $addr: expr, $default: expr) => {
		impl Register for $name {
			fn new(value: u8) -> Self {
				Self(value)
			}
			fn addr() -> u8 {
				$addr
			}
			fn value(&self) -> u8 {
				self.0
			}
		}
		impl Default for $name {
			/// Could be 0 if undefined at register
			fn default() -> Self {
				Self($default)
			}
		}

		impl Clone for $name {
			fn clone(&self) -> Self {
				$name(self.0)
			}
		}
		impl Copy for $name {}

		impl PartialEq for $name {
			fn eq(&self, rhs: &Self) -> bool {
				self.0 == rhs.0
			}
		}
	};
}
macro_rules! pipes_accessors {
	($name: ident, $default: expr, $getter: ident, $setter: ident) => {
		impl $name {
			#[inline]
			pub fn $getter(&self, pipe_no: usize) -> bool {
				let mask = 1 << pipe_no;
				self.0 & mask == mask
			}

			#[inline]
			pub fn $setter(&mut self, pipe_no: usize, enable: bool) {
				let mask = 1 << pipe_no;
				if enable {
					self.0 |= mask;
				} else {
					self.0 &= !mask;
				}
			}

			pub fn from_bools(bools: &[bool; PIPES_COUNT]) -> Self {
				let mut register = $name($default);
				for (i, b) in bools.iter().enumerate() {
					register.$setter(i, *b);
				}
				register
			}

			pub fn to_bools(&self) -> [bool; PIPES_COUNT] {
				let mut bools = [true; PIPES_COUNT];
				for (i, b) in bools.iter_mut().enumerate() {
					*b = self.$getter(i);
				}
				bools
			}
		}
	};
}

bitfield! {
	/// Config register
	pub struct Config(u8);
	impl Debug;

	/// Mask rx data ready interrupt
	pub mask_rx_dr, set_mask_rx_dr: 6;
	/// Mask tx data ready interrupt
	pub mask_tx_ds, set_mask_tx_ds: 5;
	/// Mask max retransmissions interrupt
	pub mask_max_rt, set_mask_max_rt: 4;
	/// Enable CRC
	pub en_crc, set_en_crc: 3;
	/// CRC encoding scheme
	/// * `0`: 1 byte
	/// * `1`: 2 bytes
	pub crco, set_crco: 2;
	/// Power up
	pub pwr_up, set_pwr_up: 1;
	/// RX/TX control
	/// * `1`: PRX
	/// * `0`: PTX
	pub prim_rx, set_prim_rx: 0;
}
register!(Config, 0x00, 0b_0000_1000);

/// Enable Auto Acknowledgment
#[derive(Debug)]
pub struct EnAa(pub u8);
register!(EnAa, 0x01);
pipes_accessors!(EnAa, 0b0011_1111, enaa_p, set_enaa_p);

/// Enabled RX Addresses
#[derive(Debug)]
pub struct EnRxaddr(u8);
register!(EnRxaddr, 0x02);
pipes_accessors!(EnRxaddr, 0b0000_0011, erx_p, set_erx_p);

bitfield! {
	/// Setup Address Width
	pub struct SetupAw(u8);
	impl Debug;

	/// RX/TX address field width:
	/// * `0b01`: 3 bytes
	/// * `0b10`: 4 bytes
	/// * `0b11`: 5 bytes
	pub u8, aw, set_aw: 1, 0;
}
register!(SetupAw, 0x03);

bitfield! {
	/// Setup of Automatic Retransmission
	pub struct SetupRetr(u8);
	impl Debug;

	/// Auto Retransmit Delay `250 + (250 * ard) µS`
	pub u8, ard, set_ard: 7, 4;
	/// Auto Retransmit Count
	pub u8, arc, set_arc: 3, 0;
}
register!(SetupRetr, 0x04);

bitfield! {
	/// RF Channel
	pub struct RfCh(u8);
	impl Debug;

	/// Frequency, that is `2400 + rf_ch` Mhz
	/// 7 bits 0-127
	pub u8, rf_ch, set_rf_ch: 6, 0;
}
register!(RfCh, 0x05);

bitfield! {
	/// RF Setup
	pub struct RfSetup(u8);
	impl Debug;

	/// Enables continuous carrier transmit when high.
	pub cont_wave, set_cont_wave: 7;

	/// Set for 250kbps
	/// If this bit is set, then 250kbps is always used.
	/// Regardless of what rf_dr_high has.
	pub rf_dr_low, set_rf_dr_low: 5;
	/// Select between the high speed data rates. This bit
	/// is don’t care if rf_dr_low is set.
	pub rf_dr_high, set_rf_dr_high: 3;
	/// RF output power in TX mode
	/// * `00`: -18 dBm
	/// * `01`: -12 dBm
	/// * `10`: -6 dBm
	/// * `11`: 0 dBm
	pub u8, rf_pwr, set_rf_pwr: 2, 1;
}
register!(RfSetup, 0x06);


/// Interrupt flags for status register
pub const MAX_RT_FLAG: u8 = 1 << 4;
pub const TX_DS_FLAG: u8 = 1 << 5;
pub const RX_DR_FLAG: u8 = 1 << 6;
bitfield! {
	/// Status register, always received on MISO while command is sent
	/// on MOSI.
	pub struct Status(u8);
	impl Debug;

	/// Data ready RX FIFO interrupt. Write `true` to clear.
	pub rx_dr, set_rx_dr: 6;
	/// Data sent TX FIFO interrupt. Write `true` to clear.
	pub tx_ds, set_tx_ds: 5;
	/// Maximum number of TX retransmits interrupt. Write `true` to clear.
	pub max_rt, set_max_rt: 4;
	/// Data pipe number for reading from RX FIFO
	pub u8, rx_p_no, _: 3, 1;
	/// TX FIFO full flag
	pub tx_full, _: 0;
}
register!(Status, 0x07);

bitfield! {
	/// Transmit observe register
	pub struct ObserveTx(u8);
	impl Debug;

	/// Count lost packets. The counter is overflow protected
	/// to 15, and discontinues at max until reset.
	/// The counter is reset by writing to RF_CH.
	pub u8, plos_cnt, _: 7, 4;
	/// Count retransmitted packets. The counter is reset
	/// when transmission of a new packet starts.
	pub u8, arc_cnt, _: 3, 0;
}
register!(ObserveTx, 0x08);

bitfield! {
	/// Received Power Detect
	pub struct RPD(u8);
	impl Debug;

	/// Carrier Detect
	///
	/// Received Power Detector. This register is called
	/// CD (Carrier Detect) in the nRF24L01. The name is
	/// different in nRF24L01+ due to the different input
	/// power level threshold for this bit.
	pub rpd, _: 0;
}
register!(RPD, 0x09);

macro_rules! def_rx_pw {
	($name: ident, $addr: expr) => {
		bitfield! {
			/// Static payload length for RX
			pub struct $name(u8);
			impl Debug;

			/// Number of bytes in RX payload in data pipe (max: 32)
			pub u8, get, set: 5, 0;
		}
		register!($name, $addr);
	};
}

def_rx_pw!(RxPwP0, 0x11);
def_rx_pw!(RxPwP1, 0x12);
def_rx_pw!(RxPwP2, 0x13);
def_rx_pw!(RxPwP3, 0x14);
def_rx_pw!(RxPwP4, 0x15);
def_rx_pw!(RxPwP5, 0x16);

bitfield! {
	/// FIFO Status Register
	pub struct FifoStatus(u8);
	impl Debug;

	/// Used for a PTX device
	///
	/// Pulse the rfce high for at least 10µs to Reuse last
	/// transmitted payload. TX payload reuse is active
	/// until W_TX_PAYLOAD or FLUSH TX is executed.
	///
	/// - TX_REUSE is set by the SPI command
	/// - REUSE_TX_PL, and is reset by the SPI commands
	/// - W_TX_PAYLOAD or FLUSH TX
	pub tx_reuse, _: 6;
	/// TX FIFO full flag
	pub tx_full, _: 5;
	/// TX FIFO empty flag
	pub tx_empty, _: 4;
	/// RX FIFO full flag
	pub rx_full, _: 1;
	/// RX FIFO empty flag
	pub rx_empty, _: 0;
}
register!(FifoStatus, 0x17);

/// Enable Dynamic Payload length
pub struct Dynpd(pub u8);
register!(Dynpd, 0x1C);
pipes_accessors!(Dynpd, 0, dpl_p, set_dpl_p);

bitfield! {
	/// Enable features
	pub struct Feature(u8);
	impl Debug;

	/// Enables Dynamic Payload Length
	pub en_dpl, set_en_dpl: 2;
	/// Enables Payload with ACK
	pub en_ack_pay, set_en_ack_pay: 1;
	/// Enables the W_TX_PAYLOAD_NOACK command
	pub en_dyn_ack, set_en_dyn_ack: 0;
}
register!(Feature, 0x1D);

#[test]
fn test_pipe_accessors() {
	let mut en_rx_addr = EnRxaddr::from_bools(&[true, true, false, false, false, false]);
	assert_eq!(en_rx_addr.0, 0b_0000_0011);
	assert_eq!(en_rx_addr.erx_p(0), true);
	assert_eq!(en_rx_addr.erx_p(1), true);
	assert_eq!(en_rx_addr.erx_p(2), false);
	assert_eq!(en_rx_addr.erx_p(5), false);
	assert_eq!(en_rx_addr.erx_p(7), false);
	assert_eq!(
		en_rx_addr.to_bools(),
		[true, true, false, false, false, false]
	);
	en_rx_addr.set_erx_p(2, true);
	assert_eq!(en_rx_addr.erx_p(0), true);
	assert_eq!(en_rx_addr.erx_p(1), true);
	assert_eq!(en_rx_addr.erx_p(2), true);
	assert_eq!(en_rx_addr.erx_p(5), false);
	assert_eq!(en_rx_addr.erx_p(7), false);
	assert_eq!(
		en_rx_addr.to_bools(),
		[true, true, true, false, false, false]
	);
}
