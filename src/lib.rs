#![cfg_attr(not(feature = "std"), no_std)]

mod device;

use core::fmt::Debug;

use device::*;
use embedded_hal::{
	delay::DelayNs,
	digital::OutputPin,
	spi::{Operation, SpiDevice},
};

use Error::Pin;
#[derive(Debug)]
pub enum Error<SPIE, PINE> {
	Spi(SPIE),
	Pin(PINE),
	// other custom errors
	TxFifoFull,
	RxFifoFull,
	NotInTxMode,
	NotInRxMode,
	PayloadWidthInvalid,
}
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
	spi: SPI,
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
		Ok(self.ce.set_high().map_err(Pin)?)
	}
	fn ce_disable(&mut self) -> Result<(), Error<SPI::Error, CE::Error>> {
		Ok(self.ce.set_low().map_err(Pin)?)
	}
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
	fn write_tx_payload(
		&mut self,
		buf: &[u8],
		no_ack: bool,
	) -> Result<(), Error<SPI::Error, CE::Error>> {
		self.spi.transaction(&mut [
			Operation::Write(&[if no_ack { W_TX_P_NO_ACK } else { W_TX_PAYLOAD }]),
			Operation::Write(buf),
		])?;
		Ok(())
	}
	fn flush_rx(&mut self) -> Result<(), Error<SPI::Error, CE::Error>> {
		Ok(self.spi.write(&[FLUX_RX])?)
	}
	fn flush_tx(&mut self) -> Result<(), Error<SPI::Error, CE::Error>> {
		Ok(self.spi.write(&[FLUX_TX])?)
	}
	fn reuse_tx_payload(&mut self) -> Result<(), Error<SPI::Error, CE::Error>> {
		Ok(self.spi.write(&[REUSE_TX_PL])?)
	}
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
	fn has_carrier(&mut self) -> Result<bool, Error<SPI::Error, CE::Error>> {
		self.read_register::<RPD>().map(|(_, rpd)| rpd.rpd())
	}


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

	/// If RX FIFO is not empty, return the pipe number
	pub fn data_available(&mut self) -> Result<Option<u8>, Error<SPI::Error, CE::Error>> {
		self.read_register::<FifoStatus>()
			.map(|(status, fifo_status)| {
				if !fifo_status.rx_empty() {
					Some(status.rx_p_no())
				} else {
					None
				}
			})
	}

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


	/// Start a transmission
	pub fn transmission_start<T: DelayNs>(
		&mut self,
		payload: &[u8],
		no_ack: bool,
		delay: &mut T,
	) -> Result<(), Error<SPI::Error, CE::Error>> {
		if self.config.prim_rx() == true {
			return Err(Error::NotInTxMode);
		}
		if payload.len() > 32 || payload.len() == 0 {
			return Err(Error::PayloadWidthInvalid);
		}
		if CHECK_FIFO_OVERFLOW {
			if self.nop()?.tx_full() {
				return Err(Error::TxFifoFull);
			}
		}
		self.clear_interrupts(MAX_RT_FLAG | TX_DS_FLAG)?;
		self.write_tx_payload(payload, no_ack)?;
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
	fn receive_raw(&mut self) -> Result<[u8; 32], Error<SPI::Error, CE::Error>> {
		let (_, payload_width) = self.read_rx_payload_width()?;
		if payload_width > 32 || payload_width == 0 {
			return Err(Error::PayloadWidthInvalid);
		}

		self.read_rx_payload(payload_width)
	}

	/// Receive a payload
	///
	/// This function will clear rx interrupt flags when no payload is available.
	pub fn receive(&mut self) -> Result<Option<[u8; 32]>, Error<SPI::Error, CE::Error>> {
		if self.config.prim_rx() == false {
			return Err(Error::NotInRxMode);
		}

		let (_, fifo) = self.read_register::<FifoStatus>()?;
		if fifo.rx_full() {
			// If fifo is full, then return error
			Err(Error::RxFifoFull)
		} else if fifo.rx_empty() {
			// If fifo is empty, clear the rx interrupt
			self.clear_interrupts(RX_DR_FLAG)?;
			Ok(None)
		} else {
			// If fifo isn't empty or full then receive the data
			self.receive_raw().map(|v| Some(v))
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
	/// disables ce first just in case we're still in rx;
	/// keep in mind a transition to TX takes 130us of settling
	pub fn tx(&mut self) -> Result<(), Error<SPI::Error, CE::Error>> {
		self.ce_disable()?;
		self.set_prim_rx(false)?;
		Ok(())
	}
}
