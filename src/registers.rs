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
				*self
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
	($name: ident, $getter: ident, $setter: ident) => {
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
				let mut register = $name::default();
				for (i, b) in bools.iter().enumerate() {
					register.$setter(i, *b);
				}
				register
			}

			pub fn to_bools(self) -> [bool; PIPES_COUNT] {
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
register!(EnAa, 0x01, 0b_0011_1111);
pipes_accessors!(EnAa, enaa_p, set_enaa_p);

/// Enabled RX Addresses
#[derive(Debug)]
pub struct EnRxaddr(u8);
register!(EnRxaddr, 0x02, 0b_0000_0011);
pipes_accessors!(EnRxaddr, erx_p, set_erx_p);

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
register!(SetupAw, 0x03, 0b_0000_0011);

bitfield! {
	/// Setup of Automatic Retransmission
	pub struct SetupRetr(u8);
	impl Debug;

	/// Auto Retransmit Delay `250 + (250 * ard) µS`
	pub u8, ard, set_ard: 7, 4;
	/// Auto Retransmit Count
	pub u8, arc, set_arc: 3, 0;
}
register!(SetupRetr, 0x04, 0b_0000_0011);

bitfield! {
	/// RF Channel
	pub struct RfCh(u8);
	impl Debug;

	/// Frequency, that is `2400 + rf_ch` Mhz
	/// 7 bits 0-127
	pub u8, rf_ch, set_rf_ch: 6, 0;
}
register!(RfCh, 0x05, 0b0000_0010);

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
register!(RfSetup, 0x06, 0b_0000_1110);


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
	/// Data pipe number for the payload available for reading from `RX_FIFO`
	///
	/// 000-101: Data Pipe Number
	/// 110: Not used
	/// 111: RX FIFO Empty
	pub u8, rx_p_no, _: 3, 1;
	/// TX FIFO full flag
	pub tx_full, _: 0;
}
register!(Status, 0x07, 0b_0000_1110);

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
register!(ObserveTx, 0x08, 0b_0000_0000);

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
register!(RPD, 0x09, 0b_0000_0000);

/// Receive address data pipe 0 (5 bytes, LSB first)
pub const RX_ADDR_P0: u8 = 0x0A;
pub const RX_ADDR_P0_DEFAULT: [u8; 5] = [0xE7, 0xE7, 0xE7, 0xE7, 0xE7];
/// Receive address data pipe 1 (5 bytes, LSB first)
pub const RX_ADDR_P1: u8 = 0x0B;
pub const RX_ADDR_P1_DEFAULT: [u8; 5] = [0xC2, 0xC2, 0xC2, 0xC2, 0xC2];
/// Receive address data pipe 2 (1 byte LSB, rest is RX_ADDR_P1)
pub const RX_ADDR_P2: u8 = 0x0C;
pub const RX_ADDR_P2_DEFAULT: u8 = 0xC3;
/// Receive address data pipe 3 (1 byte LSB, rest is RX_ADDR_P1)
pub const RX_ADDR_P3: u8 = 0x0D;
pub const RX_ADDR_P3_DEFAULT: u8 = 0xC4;
/// Receive address data pipe 4 (1 byte LSB, rest is RX_ADDR_P1)
pub const RX_ADDR_P4: u8 = 0x0E;
pub const RX_ADDR_P4_DEFAULT: u8 = 0xC5;
/// Receive address data pipe 5 (1 byte LSB, rest is RX_ADDR_P1)
pub const RX_ADDR_P5: u8 = 0x0F;
pub const RX_ADDR_P5_DEFAULT: u8 = 0xC6;
/// Transmit adress. Used for a PTX device only. (LSB first)
///
/// Set RX_ADDR_P0 equal to this address to handle automatic
/// acknowledge if this is a PTX device with Enhanced Shockburst enabled.
pub const TX_ADDR: u8 = 0x10;
pub const RX_ADDR_DEFAULT: [u8; 5] = RX_ADDR_P0_DEFAULT;

// ----- Data Pipe Lengths -------
macro_rules! def_rx_pw {
	($name: ident, $addr: expr) => {
		bitfield! {
			/// Static payload length for RX
			///
			/// initialized to 0 by default
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
// -------------------------------

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
register!(FifoStatus, 0x17, 0b_0001_0001);

/// Enable Dynamic Payload length
pub struct Dynpd(pub u8);
register!(Dynpd, 0x1C, 0b_0000_0000);
pipes_accessors!(Dynpd, dpl_p, set_dpl_p);

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
register!(Feature, 0x1D, 0b_0000_0000);


#[test]
fn pipe_accessors() {
	let mut en_rx_addr = EnRxaddr::from_bools(&[true, true, false, true, false, false]);
	assert_eq!(en_rx_addr.0, 0b_0000_1011);
	assert_eq!(en_rx_addr.erx_p(0), true);
	assert_eq!(en_rx_addr.erx_p(1), true);
	assert_eq!(en_rx_addr.erx_p(2), false);
	assert_eq!(en_rx_addr.erx_p(3), true);
	assert_eq!(en_rx_addr.erx_p(5), false);
	assert_eq!(en_rx_addr.erx_p(7), false);
	assert_eq!(
		en_rx_addr.to_bools(),
		[true, true, false, true, false, false]
	);
	en_rx_addr.set_erx_p(2, true);
	assert_eq!(en_rx_addr.erx_p(0), true);
	assert_eq!(en_rx_addr.erx_p(1), true);
	assert_eq!(en_rx_addr.erx_p(2), true);
	assert_eq!(en_rx_addr.erx_p(3), true);
	assert_eq!(en_rx_addr.erx_p(5), false);
	assert_eq!(en_rx_addr.erx_p(7), false);
	assert_eq!(
		en_rx_addr.to_bools(),
		[true, true, true, true, false, false]
	);
}

#[test]
fn registers() {
	let mut config = Config::default();
	assert_eq!(config.mask_rx_dr(), false);
	assert_eq!(config.mask_tx_ds(), false);
	assert_eq!(config.mask_max_rt(), false);
	assert_eq!(config.en_crc(), true);
	assert_eq!(config.crco(), false);
	assert_eq!(config.pwr_up(), false);
	assert_eq!(config.prim_rx(), false);
	config.set_crco(true);
	config.set_pwr_up(true);
	assert_eq!(config.mask_rx_dr(), false);
	assert_eq!(config.mask_tx_ds(), false);
	assert_eq!(config.mask_max_rt(), false);
	assert_eq!(config.en_crc(), true);
	assert_eq!(config.crco(), true);
	assert_eq!(config.pwr_up(), true);
	assert_eq!(config.prim_rx(), false);

	let mut en_aa = EnAa::default();
	assert_eq!(en_aa.enaa_p(0), true);
	assert_eq!(en_aa.enaa_p(1), true);
	assert_eq!(en_aa.enaa_p(2), true);
	assert_eq!(en_aa.enaa_p(3), true);
	assert_eq!(en_aa.enaa_p(4), true);
	assert_eq!(en_aa.enaa_p(5), true);
	en_aa.set_enaa_p(0, false);
	en_aa.set_enaa_p(5, true);
	assert_eq!(en_aa.enaa_p(0), false);
	assert_eq!(en_aa.enaa_p(1), true);
	assert_eq!(en_aa.enaa_p(2), true);
	assert_eq!(en_aa.enaa_p(3), true);
	assert_eq!(en_aa.enaa_p(4), true);
	assert_eq!(en_aa.enaa_p(5), true);

	let mut en_rx_addr = EnRxaddr::default();
	assert_eq!(en_rx_addr.erx_p(0), true);
	assert_eq!(en_rx_addr.erx_p(1), true);
	assert_eq!(en_rx_addr.erx_p(2), false);
	assert_eq!(en_rx_addr.erx_p(3), false);
	assert_eq!(en_rx_addr.erx_p(4), false);
	assert_eq!(en_rx_addr.erx_p(5), false);
	en_rx_addr.set_erx_p(0, false);
	en_rx_addr.set_erx_p(5, true);
	assert_eq!(en_rx_addr.erx_p(0), false);
	assert_eq!(en_rx_addr.erx_p(1), true);
	assert_eq!(en_rx_addr.erx_p(2), false);
	assert_eq!(en_rx_addr.erx_p(3), false);
	assert_eq!(en_rx_addr.erx_p(4), false);
	assert_eq!(en_rx_addr.erx_p(5), true);


	let mut setup_aw = SetupAw::default();
	assert_eq!(setup_aw.aw(), 3);
	setup_aw.set_aw(0);
	assert_eq!(setup_aw.aw(), 0);

	let mut setup_retr = SetupRetr::default();
	assert_eq!(setup_retr.ard(), 0);
	assert_eq!(setup_retr.arc(), 3);
	setup_retr.set_ard(2);
	setup_retr.set_arc(5);
	assert_eq!(setup_retr.ard(), 2);
	assert_eq!(setup_retr.arc(), 5);

	let mut rf_ch = RfCh::default();
	assert_eq!(rf_ch.rf_ch(), 2);
	rf_ch.set_rf_ch(10);
	assert_eq!(rf_ch.rf_ch(), 10);

	let mut rf_setup = RfSetup::default();
	assert_eq!(rf_setup.cont_wave(), false);
	assert_eq!(rf_setup.rf_dr_low(), false);
	assert_eq!(rf_setup.rf_dr_high(), true);
	assert_eq!(rf_setup.rf_pwr(), 0b11);

	let mut status = Status::default();
	assert_eq!(status.rx_dr(), false);
	assert_eq!(status.tx_ds(), false);
	assert_eq!(status.max_rt(), false);
	assert_eq!(status.rx_p_no(), 0b111);
	assert_eq!(status.tx_full(), false);

	let mut observe_tx = ObserveTx::default();
	assert_eq!(observe_tx.plos_cnt(), 0);
	assert_eq!(observe_tx.arc_cnt(), 0);

	let mut rpd = RPD::default();
	assert_eq!(rpd.rpd(), false);

	let mut rx_pw5 = RxPwP5::default();
	assert_eq!(rx_pw5.0, 0);

	let mut fifo_status = FifoStatus::default();
	assert_eq!(fifo_status.tx_reuse(), false);
	assert_eq!(fifo_status.tx_full(), false);
	assert_eq!(fifo_status.tx_empty(), true);
	assert_eq!(fifo_status.rx_full(), false);
	assert_eq!(fifo_status.rx_empty(), true);

	let mut dynpd = Dynpd::default();
	assert_eq!(dynpd.dpl_p(0), false);

	let mut feature = Feature::default();
	assert_eq!(feature.en_dpl(), false);
	assert_eq!(feature.en_ack_pay(), false);
	assert_eq!(feature.en_dyn_ack(), false);
	feature.set_en_ack_pay(true);
	assert_eq!(feature.en_dpl(), false);
	assert_eq!(feature.en_ack_pay(), true);
	assert_eq!(feature.en_dyn_ack(), false);
}
