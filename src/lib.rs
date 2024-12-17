#![cfg_attr(not(feature = "std"), no_std)]

mod registers;

use core::fmt::Debug;

use embedded_hal::{
	delay::DelayNs,
	digital::OutputPin,
	spi::{Operation, SpiDevice},
};
use errors::Discriminant;
use registers::*;

use Error::Pin;
#[derive(Debug)]
#[repr(u8)]
pub enum Error<SPIE, PINE> {
	Spi(SPIE) = 0,
	Pin(PINE),
	// other custom errors
	TxFifoFull,
	RxFifoFull,
	NotInTxMode,
	NotInRxMode,
	PayloadWidthInvalid,
	PipeInvalid,
	AddrTooLong,
	AddrTooShort,
}
const ERROR_MAX: u8 = 20;

// So we can pull the u8 value out of it
impl<SPIE, PINE> Discriminant for Error<SPIE, PINE> {
	fn discriminant(&self) -> u8 {
		// SAFETY: Because `Self` is marked `repr(u8)`, its layout is a `repr(C)` `union`
		// between `repr(C)` structs, each of which has the `u8` discriminant as its first
		// field, so we can read the discriminant without offsetting the pointer.
		unsafe { *<*const _>::from(self).cast::<u8>() }
	}
	fn discriminant_max() -> u8 {
		ERROR_MAX
	}
}

#[derive(Debug)]
pub enum PayloadType {
	Payload,
	PayloadWithNoAck,
	AckPayload,
}
use PayloadType::*;
// We only implement From<SPIE> bc if PINE is implemented
// compiler complains about multiple implementations.
impl<SPIE, PINE> From<SPIE> for Error<SPIE, PINE> {
	fn from(value: SPIE) -> Self {
		Self::Spi(value)
	}
}

const CHECK_FIFO_OVERFLOW: bool = true;

pub struct NRF24L01<SPI, CE> {
	ce: CE,
	// pub so we can read reg values in samn HQ
	pub spi: SPI,
	config: Config,
}

impl<SPI: SpiDevice<u8>, CE: OutputPin> NRF24L01<SPI, CE> {
	/// Construct a new radio instance.
	pub fn new(ce: CE, spi: SPI) -> Self {
		Self {
			ce,
			spi,
			config: Default::default(),
		}
	}

	// ------------- Low Level Operations ---------------
	fn ce_enable(&mut self) -> Result<(), Error<SPI::Error, CE::Error>> {
		self.ce.set_high().map_err(Pin)
	}
	fn ce_disable(&mut self) -> Result<(), Error<SPI::Error, CE::Error>> {
		self.ce.set_low().map_err(Pin)
	}

	/// Executable in power down or standby modes only.
	fn write_register<R: Register>(
		&mut self,
		register: R,
	) -> Result<Status, Error<SPI::Error, CE::Error>> {
		let mut buf = [W_REGISTER | R::addr()];
		self.spi.transaction(&mut [
			Operation::TransferInPlace(&mut buf),
			Operation::Write(&[register.value()]),
		])?;
		Ok(Status(buf[0]))
	}
	fn read_register<R: Register>(&mut self) -> Result<(Status, R), Error<SPI::Error, CE::Error>> {
		let mut buf = [R_REGISTER | R::addr(), 0];
		self.spi.transfer_in_place(&mut buf)?;
		Ok((Status(buf[0]), R::new(buf[1])))
	}
	fn read_rx_payload(&mut self, buf_l: u8) -> Result<[u8; 32], Error<SPI::Error, CE::Error>> {
		let mut buf = [0u8; 32];
		self.spi.transaction(&mut [
			Operation::Write(&[R_RX_PAYLOAD]),
			Operation::Read(&mut buf[..buf_l as usize]),
		])?;
		Ok(buf)
	}

	fn write_payload(
		&mut self,
		buf: &[u8],
		payload_type: PayloadType,
	) -> Result<(), Error<SPI::Error, CE::Error>> {
		self.spi.transaction(&mut [
			Operation::Write(&[match payload_type {
				Payload => W_TX_PAYLOAD,
				PayloadWithNoAck => W_TX_P_NO_ACK,
				AckPayload => W_ACK_PAYLOAD,
			}]),
			Operation::Write(buf),
		])?;
		Ok(())
	}
	/// Remember that addr should start with LSB first.
	pub fn set_rx_addr(
		&mut self,
		pipe: u8,
		addr: &[u8],
	) -> Result<(), Error<SPI::Error, CE::Error>> {
		let pipe_addr = match pipe {
			0 => RX_ADDR_P0,
			1 => RX_ADDR_P1,
			2 => RX_ADDR_P2,
			3 => RX_ADDR_P3,
			4 => RX_ADDR_P4,
			5 => RX_ADDR_P5,
			_ => return Err(Error::PipeInvalid),
		};
		if addr.len() == 0 {
			return Err(Error::AddrTooShort);
		}
		if pipe <= 1 {
			if addr.len() > 5 {
				return Err(Error::AddrTooLong);
			}
			let mut buf = [0u8; 6];
			buf[0] = W_REGISTER | pipe_addr;
			let buf_len = addr.len() + 1; // max of 6 (1 command byte, max 5 addr bytes)
			buf[1..buf_len].copy_from_slice(addr);
			self.spi.write(&buf[..buf_len])?;
		} else {
			if addr.len() > 1 {
				return Err(Error::AddrTooLong);
			}
			let mut buf = [0u8; 2];
			buf[0] = W_REGISTER | pipe_addr;
			buf[1] = addr[0];
			self.spi.write(&buf)?;
		}

		Ok(())
	}
	/// Remember that addr should start with LSB first.
	pub fn set_tx_addr(&mut self, addr: &[u8]) -> Result<(), Error<SPI::Error, CE::Error>> {
		if addr.len() > 5 {
			Err(Error::AddrTooLong)
		} else {
			let mut buf = [0u8; 6];
			buf[0] = W_REGISTER | TX_ADDR;
			let buf_len = addr.len() + 1; // max of 6 (1 command byte, max 5 addr bytes)
			buf[1..buf_len].copy_from_slice(addr);
			Ok(self.spi.write(&buf[..buf_len])?)
		}
	}
	pub fn flush_rx(&mut self) -> Result<(), Error<SPI::Error, CE::Error>> {
		Ok(self.spi.write(&[FLUX_RX])?)
	}
	pub fn flush_tx(&mut self) -> Result<(), Error<SPI::Error, CE::Error>> {
		Ok(self.spi.write(&[FLUX_TX])?)
	}
	// fn reuse_tx_payload(&mut self) -> Result<(), Error<SPI::Error, CE::Error>> {
	// 	Ok(self.spi.write(&[REUSE_TX_PL])?)
	// }
	/// Sends a No Operation command, just reads Status register
	fn nop(&mut self) -> Result<Status, Error<SPI::Error, CE::Error>> {
		let mut buf = [NOP];
		self.spi.transfer_in_place(&mut buf)?;
		Ok(Status(buf[0]))
	}
	fn read_rx_payload_width(&mut self) -> Result<(Status, u8), Error<SPI::Error, CE::Error>> {
		let mut buf = [R_RX_PL_WID, 0];
		self.spi.transfer_in_place(&mut buf)?;
		Ok((Status(buf[0]), buf[1]))
	}
	fn set_prim_rx(&mut self, prim_rx: bool) -> Result<(), Error<SPI::Error, CE::Error>> {
		self.config.set_prim_rx(prim_rx);
		self.write_register(self.config)?;
		Ok(())
	}


	/// Is an in-band RF signal detected?
	///
	/// The internal carrier detect signal must be high for 40μs
	/// (NRF24L01+) or 128μs (NRF24L01) before the carrier detect
	/// register is set. Note that changing from standby to receive
	/// mode also takes 130μs.
	// fn has_carrier(&mut self) -> Result<bool, Error<SPI::Error, CE::Error>> {
	// 	self.read_register::<RPD>().map(|(_, rpd)| rpd.rpd())
	// }


	fn clear_interrupts(&mut self, flags: u8) -> Result<(), Error<SPI::Error, CE::Error>> {
		self.write_register(Status(flags & (MAX_RT_FLAG | RX_DR_FLAG | TX_DS_FLAG)))?;
		Ok(())
	}
	fn set_pwr_up(&mut self, pwr_up: bool) -> Result<(), Error<SPI::Error, CE::Error>> {
		self.config.set_pwr_up(pwr_up);
		self.write_register(self.config)?;
		Ok(())
	}

	/// Powers up the device
	fn power_up(&mut self) -> Result<(), Error<SPI::Error, CE::Error>> {
		self.set_pwr_up(true)
	}
	/// Powers down the device.
	///
	/// Can be used to stop a tx transmission while its underway
	fn power_down(&mut self) -> Result<(), Error<SPI::Error, CE::Error>> {
		self.set_pwr_up(false)
	}

	// --------------- Functional Operations ---------------

	// /// If RX FIFO is not empty, return the pipe number
	// pub fn data_available(&mut self) -> Result<Option<u8>, Error<SPI::Error, CE::Error>> {
	// 	self.read_register::<FifoStatus>()
	// 		.map(|(status, fifo_status)| {
	// 			if !fifo_status.rx_empty() {
	// 				Some(status.rx_p_no())
	// 			} else {
	// 				None
	// 			}
	// 		})
	// }

	/// Perform initialization sequence
	///
	/// Leaves the device in Standby mode.
	pub fn initialize<T: DelayNs>(
		&mut self,
		delay: &mut T,
	) -> Result<(), Error<SPI::Error, CE::Error>> {
		// If device is active, making sure it goes to a zeroed state
		self.ce_disable()?;
		self.power_down()?;
		delay.delay_ms(100); // Maximum time for power-on reset
					   // We can now assume the device is in power_down state
		self.power_up()?;
		delay.delay_ms(5); // Maximum time for power_down -> standby
		self.flush_rx()?;
		self.flush_tx()?;
		Ok(())
	}
}

impl<SPI: SpiDevice<u8>, CE: OutputPin> NRF24L01<SPI, CE> {
	/// All the things we can configure are:
	///
	/// - Config (masking IRQ flags, CRC, CRCO)
	/// - EN_AA (enabling auto-acknowledgement)
	/// - EN_RXADDR (enabling data pipes for RX)
	/// - DYNPD (enable dynamic payloads for pipes)
	/// - SETUP_AW (setup of address widths)
	/// - SETUP_RETR (ard [retransmission delay], arc [retransmission count])
	/// - RF_CH (rf channel)
	/// - RF_SETUP (data rate, rf power)
	/// - RX/TX_addr (addresses) & RX_PW (static # bytes in pipes)
	/// - Feature
	/// 	- enable dynamic payloads
	/// 	- enable ack payloads
	/// 	- enable dynamic ack (just enables the writing of a payloads with no ack apparently)
	pub fn configure(&mut self) -> Result<(), Error<SPI::Error, CE::Error>> {
		let mut config = Config::default();
		// Set crc of 2 bytes
		config.set_crco(true);
		// Won't mask any flags
		self.write_register(config)?;

		const PIPES:[bool;6] = [true, false, false, false, false, false];

		// en_aa is already set to true on all pipes, but we'll set it anyways
		self.write_register(EnAa::from_bools(&PIPES))?;
		// only enable first pipe
		self.write_register(EnRxaddr::from_bools(&PIPES))?;
		// Enable dynamic payloads on first pipe
		self.write_register(Dynpd::from_bools(&PIPES))?;

		let mut setup_aw = SetupAw::default();
		setup_aw.set_aw(0b11); // 5 byte addresses
		self.write_register(setup_aw)?;

		let mut setup_retr = SetupRetr::default();
		setup_retr.set_ard(5); // 1500 uS or 1.5ms
		setup_retr.set_arc(3); // 3 retransmissions
		self.write_register(setup_retr)?;

		let mut rf_ch = RfCh::default();
		rf_ch.set_rf_ch(24); // (2400 MHz + ch)
		self.write_register(rf_ch)?;

		let mut rf_setup = RfSetup::default();
		rf_setup.set_rf_dr_low(true); // enable 250KHz data rate
		rf_setup.set_rf_dr_high(false); // ik it's dont care, but for sake of it
		rf_setup.set_rf_pwr(0b11); // 0dBm MAX power
		self.write_register(rf_setup)?;

		// We won't set addresses or static number of bytes as defaults are ok
		// either way application should modify rx_addr and tx_addr themselves

		

		let mut feature = Feature::default();
		feature.set_en_dpl(true); // Enable dynamic payloads
		feature.set_en_ack_pay(false); // Disable acks with payloads
		feature.set_en_dyn_ack(true); // Enable writing payloads with no-ack.
		self.write_register(feature)?;

		Ok(())
	}

	pub fn set_auto_ack_pipes(
		&mut self,
		pipes: &[bool; PIPES_COUNT],
	) -> Result<(), Error<SPI::Error, CE::Error>> {
		self.write_register(EnAa::from_bools(pipes))?;
		Ok(())
	}
	pub fn set_rx_enabled_pipes(
		&mut self,
		pipes: &[bool; PIPES_COUNT],
	) -> Result<(), Error<SPI::Error, CE::Error>> {
		self.write_register(EnRxaddr::from_bools(pipes))?;
		Ok(())
	}
	pub fn set_dynamic_payload_pipes(
		&mut self,
		pipes: &[bool; PIPES_COUNT],
	) -> Result<(), Error<SPI::Error, CE::Error>> {
		self.write_register(Dynpd::from_bools(pipes))?;
		Ok(())
	}


	/// Start a transmission
	///
	/// If we're expecting an ack Shockburst,
	/// then we need to set RX_ADDR_P0 same as TX_ADDR
	/// this function currently doesn't do that.
	pub fn transmission_start<T: DelayNs>(
		&mut self,
		payload: &[u8],
		payload_type: PayloadType,
		delay: &mut T,
	) -> Result<(), Error<SPI::Error, CE::Error>> {
		if self.config.prim_rx() {
			return Err(Error::NotInTxMode);
		}
		if payload.len() > 32 || payload.is_empty() {
			return Err(Error::PayloadWidthInvalid);
		}
		if CHECK_FIFO_OVERFLOW && self.nop()?.tx_full() {
			return Err(Error::TxFifoFull);
		}
		self.clear_interrupts(MAX_RT_FLAG | TX_DS_FLAG)?;
		self.write_payload(payload, payload_type)?;
		self.ce_enable()?;
		delay.delay_us(100);
		self.ce_disable()?;
		Ok(())
	}

	/// Wait for transmission to end
	///
	/// Will clear tx interrupt flags
	pub fn transmission_ended(&mut self) -> nb::Result<bool, Error<SPI::Error, CE::Error>> {
		let status = self.nop()?;

		if status.tx_ds() {
			self.clear_interrupts(MAX_RT_FLAG | TX_DS_FLAG)?;
			Ok(true)
		} else if status.max_rt() {
			self.clear_interrupts(MAX_RT_FLAG | TX_DS_FLAG)?;
			self.flush_tx()?;
			Ok(false)
		} else {
			Err(nb::Error::WouldBlock)
		}
	}

	/// Receive a payload
	///
	/// This function will error if there's no payload available
	/// so make sure you only call this when you know there's a payload.
	fn receive(&mut self) -> Result<[u8; 32], Error<SPI::Error, CE::Error>> {
		let (_, payload_width) = self.read_rx_payload_width()?;
		if payload_width > 32 || payload_width == 0 {
			return Err(Error::PayloadWidthInvalid);
		}

		self.read_rx_payload(payload_width)
	}

	/// Receive a payload
	///
	/// This function will clear rx interrupt flags when no payload is available.
	///
	/// Returns Option<(pipe_number, payload)>
	pub fn receive_maybe(
		&mut self,
	) -> Result<Option<(u8, [u8; 32])>, Error<SPI::Error, CE::Error>> {
		if !self.config.prim_rx() {
			return Err(Error::NotInRxMode);
		}

		let status;
		if CHECK_FIFO_OVERFLOW {
			let (status_, fifo) = self.read_register::<FifoStatus>()?;
			status = status_;
			if fifo.rx_full() {
				// If fifo is full, then return error
				return Err(Error::RxFifoFull);
			}
		} else {
			status = self.nop()?;
		}

		if status.rx_p_no() == 0b111 {
			// If fifo is empty, clear the rx interrupt
			self.clear_interrupts(RX_DR_FLAG)?;
			Ok(None)
		} else {
			// If fifo isn't empty or full then receive the data
			self.receive()
				.map(|payload| Some((status.rx_p_no(), payload)))
		}
	}


	/// Switches to RX mode
	///
	/// keep in mind a transition to RX takes 130us of settling;
	/// enables ce
	pub fn rx(&mut self) -> Result<(), Error<SPI::Error, CE::Error>> {
		self.set_prim_rx(true)?;
		self.ce_enable()?;
		Ok(())
	}
	/// Switches to TX mode
	///
	/// keep in mind a transition to TX takes 130us of settling;
	/// disables ce first just in case we're still in rx
	pub fn tx(&mut self) -> Result<(), Error<SPI::Error, CE::Error>> {
		self.ce_disable()?;
		self.set_prim_rx(false)?;
		Ok(())
	}

	/// Disables CE
	pub fn idle(&mut self) -> Result<(), Error<SPI::Error, CE::Error>> {
		self.ce_disable()?;
		Ok(())
	}
}
