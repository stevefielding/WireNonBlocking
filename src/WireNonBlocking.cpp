/*
 * TwoWire.h - TWI/I2C library for Arduino Due
 * Copyright (c) 2011 Cristian Maglie <c.maglie@arduino.cc>
 * All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

extern "C" {
#include <string.h>
}

#include "WireNonBlocking.h"

static inline bool TWI_FailedAcknowledge(Twi *pTwi) {
	return pTwi->TWI_SR & TWI_SR_NACK;
}

static inline bool TWI_WaitTransferComplete(Twi *_twi, uint32_t _timeout) {
	uint32_t _status_reg = 0;
	while ((_status_reg & TWI_SR_TXCOMP) != TWI_SR_TXCOMP) {
		_status_reg = TWI_GetStatus(_twi);

		if (_status_reg & TWI_SR_NACK)
			return false;

		if (--_timeout == 0)
			return false;
	}
	return true;
}

static inline bool TWI_WaitByteSent(Twi *_twi, uint32_t _timeout) {
	uint32_t _status_reg = 0;
	while ((_status_reg & TWI_SR_TXRDY) != TWI_SR_TXRDY) {
		_status_reg = TWI_GetStatus(_twi);

		if (_status_reg & TWI_SR_NACK)
			return false;

		if (--_timeout == 0)
			return false;
	}

	return true;
}

static inline bool TWI_WaitByteReceived(Twi *_twi, uint32_t _timeout) {
	uint32_t _status_reg = 0;
	while ((_status_reg & TWI_SR_RXRDY) != TWI_SR_RXRDY) {
		_status_reg = TWI_GetStatus(_twi);

		if (_status_reg & TWI_SR_NACK)
			return false;

		if (--_timeout == 0)
			return false;
	}

	return true;
}

static inline bool TWI_STATUS_SVREAD(uint32_t status) {
	return (status & TWI_SR_SVREAD) == TWI_SR_SVREAD;
}

static inline bool TWI_STATUS_SVACC(uint32_t status) {
	return (status & TWI_SR_SVACC) == TWI_SR_SVACC;
}

static inline bool TWI_STATUS_GACC(uint32_t status) {
	return (status & TWI_SR_GACC) == TWI_SR_GACC;
}

static inline bool TWI_STATUS_EOSACC(uint32_t status) {
	return (status & TWI_SR_EOSACC) == TWI_SR_EOSACC;
}

static inline bool TWI_STATUS_NACK(uint32_t status) {
	return (status & TWI_SR_NACK) == TWI_SR_NACK;
}

TwoWire::TwoWire(Twi *_twi, void(*_beginCb)(void), void(*_endCb)(void)) :
	twi(_twi), rxBufferIndex(0), rxBufferLength(0), txAddress(0),
			txBufferLength(0), srvBufferIndex(0), srvBufferLength(0), status(
					UNINITIALIZED), onBeginCallback(_beginCb), 
						onEndCallback(_endCb), twiClock(TWI_CLOCK) {
}

void TwoWire::begin(void) {
	if (onBeginCallback)
		onBeginCallback();

	// Disable PDC channel
	twi->TWI_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;

	TWI_ConfigureMaster(twi, twiClock, VARIANT_MCK);
	status = MASTER_IDLE;
}

void TwoWire::begin(uint8_t address) {
	if (onBeginCallback)
		onBeginCallback();

	// Disable PDC channel
	twi->TWI_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;

	TWI_ConfigureSlave(twi, address);
	status = SLAVE_IDLE;
	TWI_EnableIt(twi, TWI_IER_SVACC);
	//| TWI_IER_RXRDY | TWI_IER_TXRDY	| TWI_IER_TXCOMP);
}

void TwoWire::begin(int address) {
	begin((uint8_t) address);
}

void TwoWire::end(void) {
	TWI_Disable(twi);

	// Enable PDC channel
	twi->TWI_PTCR &= ~(UART_PTCR_RXTDIS | UART_PTCR_TXTDIS);

	if (onEndCallback)
		onEndCallback();
}

void TwoWire::setClock(uint32_t frequency) {
	twiClock = frequency;
	TWI_SetClock(twi, twiClock, VARIANT_MCK);
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity, uint32_t iaddress, uint8_t isize, uint8_t sendStop) {
	if (quantity > BUFFER_LENGTH)
		quantity = BUFFER_LENGTH;

	// perform blocking read into buffer
	int readed = 0;
	TWI_StartRead(twi, address, iaddress, isize);
	do {
		// Stop condition must be set during the reception of last byte
		if (readed + 1 == quantity)
			TWI_SendSTOPCondition( twi);

		if (TWI_WaitByteReceived(twi, RECV_TIMEOUT))
			rxBuffer[readed++] = TWI_ReadByte(twi);
		else
			break;
	} while (readed < quantity);
	TWI_WaitTransferComplete(twi, RECV_TIMEOUT);

	// set rx buffer iterator vars
	rxBufferIndex = 0;
	rxBufferLength = readed;

	return readed;
}

// state machine version of requestFrom. Non-blocking, 2.3uS execution on atsam3x at 84MHz
// You must call this function in a sequence.
// On the first call resetStMach must be asserted.
// Then the function must be called repetitively until it returns false. 
// ie, processing is complete.
// All other parameters must be set correctly for every call.
// Polling for RX bytes, so run at i2c @ 100KHz to allow 100uS to service RX bytes.
enum rqStates {RQ_IDLE, RQ_START, RQ_WAIT_BYTE, RQ_WAIT_TRANSFER_COMPLETE};
int rqCurrState = RQ_IDLE;
int rqReaded = 0;
unsigned long rqStartMillis;
bool TwoWire::requestFrom_nb(bool resetStMach, bool *success,  uint8_t *bytesRead, uint8_t address, uint8_t quantity, uint32_t iaddress, uint8_t isize, uint8_t sendStop) {
  bool rqDone = false;
  *bytesRead = 0;
  *success = false;
  uint32_t status_reg;
  if (quantity > BUFFER_LENGTH)
    quantity = BUFFER_LENGTH;
  if (resetStMach)
    rqCurrState = RQ_START;

  switch (rqCurrState) {
    case RQ_IDLE:
      // do nothing. Wait for start request
      break;
    case RQ_START:
      rqReaded = 0;
      TWI_StartRead(twi, address, iaddress, isize);
      rqCurrState = RQ_WAIT_BYTE;
      rqStartMillis = millis();
      break;
    case RQ_WAIT_BYTE:
      status_reg = TWI_GetStatus(twi);
      if ((status_reg & TWI_SR_RXRDY) == TWI_SR_RXRDY && (status_reg & TWI_SR_NACK) != TWI_SR_NACK) {
        rxBuffer[rqReaded++] = TWI_ReadByte(twi);
	// Could there be a race condition here?
	// If this function is not called within one i2c byte period (25uS @ 400KHz or 100uS @ 100KHz)
	// then the stop condition may not be set correctly
	// This might be a good reason to keep the i2c bus at 100KHz
	// Not sure that this is true. The next byte may not be sent until TWI_ReadByte is executed?
        if (rqReaded + 1 == quantity)
          TWI_SendSTOPCondition(twi);
        if (rqReaded >= quantity)
          rqCurrState = RQ_WAIT_TRANSFER_COMPLETE;
	rqStartMillis = millis();
      }
      // timeout is 1mS per byte. Way too much time, but it should work
      else if ((millis() - rqStartMillis) > quantity + 2) {
        rqCurrState = RQ_IDLE;
	rqDone = true;
      }
      break;
    case RQ_WAIT_TRANSFER_COMPLETE:
      status_reg = TWI_GetStatus(twi);
      if ((status_reg & TWI_SR_TXCOMP) == TWI_SR_TXCOMP) {
        if (status_reg & TWI_SR_NACK) {
          *success = false; // error, finishing up
          rqDone = true;
          rqCurrState = RQ_IDLE;
        }
        else {
         rxBufferIndex = 0;
         rxBufferLength = rqReaded;
         rqDone = true;
         *success = true;
         *bytesRead = rqReaded;
         rqCurrState = RQ_IDLE;
       }
      }
      else if (rqStartMillis - millis() >= 2) {
        rqCurrState = RQ_IDLE;
        *success = false; // error, finishing up
        rqDone = true;
      }
      break; 
    default:
      break;
  }
  return rqDone;
}


uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop) {
	return requestFrom((uint8_t) address, (uint8_t) quantity, (uint32_t) 0, (uint8_t) 0, (uint8_t) sendStop);
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity) {
	return requestFrom((uint8_t) address, (uint8_t) quantity, (uint8_t) true);
}

uint8_t TwoWire::requestFrom(int address, int quantity) {
	return requestFrom((uint8_t) address, (uint8_t) quantity, (uint8_t) true);
}

uint8_t TwoWire::requestFrom(int address, int quantity, int sendStop) {
	return requestFrom((uint8_t) address, (uint8_t) quantity, (uint8_t) sendStop);
}

void TwoWire::beginTransmission(uint8_t address) {
	status = MASTER_SEND;

	// save address of target and empty buffer
	txAddress = address;
	txBufferLength = 0;
}

void TwoWire::beginTransmission(int address) {
	beginTransmission((uint8_t) address);
}

//
//	Originally, 'endTransmission' was an f(void) function.
//	It has been modified to take one parameter indicating
//	whether or not a STOP should be performed on the bus.
//	Calling endTransmission(false) allows a sketch to
//	perform a repeated start.
//
//	WARNING: Nothing in the library keeps track of whether
//	the bus tenure has been properly ended with a STOP. It
//	is very possible to leave the bus in a hung state if
//	no call to endTransmission(true) is made. Some I2C
//	devices will behave oddly if they do not see a STOP.
//
uint8_t TwoWire::endTransmission(uint8_t sendStop) {
	uint8_t error = 0;
	// transmit buffer (blocking)
	TWI_StartWrite(twi, txAddress, 0, 0, txBuffer[0]);
	if (!TWI_WaitByteSent(twi, XMIT_TIMEOUT))
		error = 2;	// error, got NACK on address transmit
	
	if (error == 0) {
		uint16_t sent = 1;
		while (sent < txBufferLength) {
			TWI_WriteByte(twi, txBuffer[sent++]);
			if (!TWI_WaitByteSent(twi, XMIT_TIMEOUT))
				error = 3;	// error, got NACK during data transmmit
		}
	}
	
	if (error == 0) {
		TWI_Stop(twi);
		if (!TWI_WaitTransferComplete(twi, XMIT_TIMEOUT))
			error = 4;	// error, finishing up
	}

	txBufferLength = 0;		// empty buffer
	status = MASTER_IDLE;
	return error;
}


// state machine version of endTransmission. Non-blocking, 2.3uS execution on atsam3x at 84MHz
// You must call this function in a sequence.
// On the first call resetStMach must be asserted.
// Then the function must be called repetitively until it returns false. 
// ie, processing is complete.
// Polling, so run at i2c @ 100KHz to allow 100uS to service .
enum etStates {ET_IDLE, ET_START, ET_WAIT_ADDR_SENT, ET_SEND_DATA, ET_WAIT_DATA_SENT, ET_WAIT_STOP_SENT};
int etCurrSt = ET_IDLE;
bool etDone;
unsigned long etStartMillis;
uint16_t etDataSent;
bool TwoWire::endTransmission_nb(bool resetStMach, uint8_t *trError) {
  etDone = false;
  uint32_t status_reg = 0;
    
  if (resetStMach)
    etCurrSt = ET_START;
  switch (etCurrSt) {
    case ET_IDLE:
     // Do nothing. Wait here for start request
     break;
    case ET_START:
      // transmit buffer (non-blocking)
      TWI_StartWrite(twi, txAddress, 0, 0, txBuffer[0]);
      etCurrSt = ET_WAIT_ADDR_SENT;
      etStartMillis = millis();
      *trError = 0;
      break;
    case ET_WAIT_ADDR_SENT:
      status_reg = TWI_GetStatus(twi);
      if ((status_reg & TWI_SR_TXRDY) == TWI_SR_TXRDY) {
        if (status_reg & TWI_SR_NACK) {
          *trError = 2; // error, got NACK or timeout on address transmit
          etCurrSt = ET_IDLE;
          etDone = true;
        }
        else {
          if (txBufferLength == 1) {
            TWI_Stop(twi);
            etCurrSt = ET_WAIT_STOP_SENT;
          }
	  else {
            etDataSent = 1;
            TWI_WriteByte(twi, txBuffer[etDataSent++]);
            etCurrSt = ET_WAIT_DATA_SENT;
	  }
          etStartMillis = millis();
        }
      }
      else if (etStartMillis - millis() >= 2) {
        etCurrSt = ET_IDLE;
        *trError = 2; // error, got NACK or timeout on address transmit
        etDone = true;
      }
      break;
    case ET_SEND_DATA:
      TWI_WriteByte(twi, txBuffer[etDataSent++]);
      etCurrSt = ET_WAIT_DATA_SENT;
      etStartMillis = millis();
      break;
    case ET_WAIT_DATA_SENT:
      status_reg = TWI_GetStatus(twi);
      if ((status_reg & TWI_SR_TXRDY) == TWI_SR_TXRDY) {
        if (status_reg & TWI_SR_NACK) {
          *trError = 3; // error, got NACK or timeout on data transmit
          etCurrSt = ET_IDLE;
          etDone = true;
        }
        else {
          etStartMillis = millis();
          if (etDataSent == txBufferLength) {
            etCurrSt = ET_WAIT_STOP_SENT;
            TWI_Stop(twi);
          }
          else
            etCurrSt = ET_SEND_DATA;
        }
      }
      else if (etStartMillis - millis() >= 2) {
        etCurrSt = ET_IDLE;
        *trError = 3; // error, got NACK or timeout on address transmit
        etDone = true;
      }
      break;
    case ET_WAIT_STOP_SENT:
      status_reg = TWI_GetStatus(twi);
      if ((status_reg & TWI_SR_TXCOMP) == TWI_SR_TXCOMP) {
        if (status_reg & TWI_SR_NACK) {
          *trError = 4; // error, finishing up
          etDone = true;
          etCurrSt = ET_IDLE;
        }
        else {
         etDone = true;
	 etCurrSt = ET_IDLE;
       }
      }
      else if (etStartMillis - millis() >= 2) {
        etCurrSt = ET_IDLE;
        *trError = 4; // error, finishing up
        etDone = true;
      }
    default:
      break;
  }
  if (etDone) {
    txBufferLength = 0; // empty buffer
    status = MASTER_IDLE;
  }
  return etDone;
}

//	This provides backwards compatibility with the original
//	definition, and expected behaviour, of endTransmission
//
uint8_t TwoWire::endTransmission(void)
{
	return endTransmission(true);
}

size_t TwoWire::write(uint8_t data) {
	if (status == MASTER_SEND) {
		if (txBufferLength >= BUFFER_LENGTH)
			return 0;
		txBuffer[txBufferLength++] = data;
		return 1;
	} else {
		if (srvBufferLength >= BUFFER_LENGTH)
			return 0;
		srvBuffer[srvBufferLength++] = data;
		return 1;
	}
}

size_t TwoWire::write(const uint8_t *data, size_t quantity) {
	if (status == MASTER_SEND) {
		for (size_t i = 0; i < quantity; ++i) {
			if (txBufferLength >= BUFFER_LENGTH)
				return i;
			txBuffer[txBufferLength++] = data[i];
		}
	} else {
		for (size_t i = 0; i < quantity; ++i) {
			if (srvBufferLength >= BUFFER_LENGTH)
				return i;
			srvBuffer[srvBufferLength++] = data[i];
		}
	}
	return quantity;
}

int TwoWire::available(void) {
	return rxBufferLength - rxBufferIndex;
}

int TwoWire::read(void) {
	if (rxBufferIndex < rxBufferLength)
		return rxBuffer[rxBufferIndex++];
	return -1;
}

int TwoWire::peek(void) {
	if (rxBufferIndex < rxBufferLength)
		return rxBuffer[rxBufferIndex];
	return -1;
}

void TwoWire::flush(void) {
	// Do nothing, use endTransmission(..) to force
	// data transfer.
}

void TwoWire::onReceive(void(*function)(int)) {
	onReceiveCallback = function;
}

void TwoWire::onRequest(void(*function)(void)) {
	onRequestCallback = function;
}

void TwoWire::onService(void) {
	// Retrieve interrupt status
	uint32_t sr = TWI_GetStatus(twi);

	if (status == SLAVE_IDLE && TWI_STATUS_SVACC(sr)) {
		TWI_DisableIt(twi, TWI_IDR_SVACC);
		TWI_EnableIt(twi, TWI_IER_RXRDY | TWI_IER_GACC | TWI_IER_NACK
				| TWI_IER_EOSACC | TWI_IER_SCL_WS | TWI_IER_TXCOMP);

		srvBufferLength = 0;
		srvBufferIndex = 0;

		// Detect if we should go into RECV or SEND status
		// SVREAD==1 means *master* reading -> SLAVE_SEND
		if (!TWI_STATUS_SVREAD(sr)) {
			status = SLAVE_RECV;
		} else {
			status = SLAVE_SEND;

			// Alert calling program to generate a response ASAP
			if (onRequestCallback)
				onRequestCallback();
			else
				// create a default 1-byte response
				write((uint8_t) 0);
		}
	}

	if (status != SLAVE_IDLE && TWI_STATUS_EOSACC(sr)) {
		if (status == SLAVE_RECV && onReceiveCallback) {
			// Copy data into rxBuffer
			// (allows to receive another packet while the
			// user program reads actual data)
			for (uint8_t i = 0; i < srvBufferLength; ++i)
				rxBuffer[i] = srvBuffer[i];
			rxBufferIndex = 0;
			rxBufferLength = srvBufferLength;

			// Alert calling program
			onReceiveCallback( rxBufferLength);
		}

		// Transfer completed
		TWI_EnableIt(twi, TWI_SR_SVACC);
		TWI_DisableIt(twi, TWI_IDR_RXRDY | TWI_IDR_GACC | TWI_IDR_NACK
				| TWI_IDR_EOSACC | TWI_IDR_SCL_WS | TWI_IER_TXCOMP);
		status = SLAVE_IDLE;
	}

	if (status == SLAVE_RECV) {
		if (TWI_STATUS_RXRDY(sr)) {
			if (srvBufferLength < BUFFER_LENGTH)
				srvBuffer[srvBufferLength++] = TWI_ReadByte(twi);
		}
	}

	if (status == SLAVE_SEND) {
		if (TWI_STATUS_TXRDY(sr) && !TWI_STATUS_NACK(sr)) {
			uint8_t c = 'x';
			if (srvBufferIndex < srvBufferLength)
				c = srvBuffer[srvBufferIndex++];
			TWI_WriteByte(twi, c);
		}
	}
}

#if WIRE_INTERFACES_COUNT > 0
static void Wire_Init(void) {
	pmc_enable_periph_clk(WIRE_INTERFACE_ID);
	PIO_Configure(
			g_APinDescription[PIN_WIRE_SDA].pPort,
			g_APinDescription[PIN_WIRE_SDA].ulPinType,
			g_APinDescription[PIN_WIRE_SDA].ulPin,
			g_APinDescription[PIN_WIRE_SDA].ulPinConfiguration);
	PIO_Configure(
			g_APinDescription[PIN_WIRE_SCL].pPort,
			g_APinDescription[PIN_WIRE_SCL].ulPinType,
			g_APinDescription[PIN_WIRE_SCL].ulPin,
			g_APinDescription[PIN_WIRE_SCL].ulPinConfiguration);

	NVIC_DisableIRQ(WIRE_ISR_ID);
	NVIC_ClearPendingIRQ(WIRE_ISR_ID);
	NVIC_SetPriority(WIRE_ISR_ID, 0);
	NVIC_EnableIRQ(WIRE_ISR_ID);
}

static void Wire_Deinit(void) {
	NVIC_DisableIRQ(WIRE_ISR_ID);
	NVIC_ClearPendingIRQ(WIRE_ISR_ID);

	pmc_disable_periph_clk(WIRE_INTERFACE_ID);

	// no need to undo PIO_Configure, 
	// as Peripheral A was enable by default before,
	// and pullups were not enabled
}

TwoWire Wire = TwoWire(WIRE_INTERFACE, Wire_Init, Wire_Deinit);

void WIRE_ISR_HANDLER(void) {
	Wire.onService();
}
#endif

#if WIRE_INTERFACES_COUNT > 1
static void Wire1_Init(void) {
	pmc_enable_periph_clk(WIRE1_INTERFACE_ID);
	PIO_Configure(
			g_APinDescription[PIN_WIRE1_SDA].pPort,
			g_APinDescription[PIN_WIRE1_SDA].ulPinType,
			g_APinDescription[PIN_WIRE1_SDA].ulPin,
			g_APinDescription[PIN_WIRE1_SDA].ulPinConfiguration);
	PIO_Configure(
			g_APinDescription[PIN_WIRE1_SCL].pPort,
			g_APinDescription[PIN_WIRE1_SCL].ulPinType,
			g_APinDescription[PIN_WIRE1_SCL].ulPin,
			g_APinDescription[PIN_WIRE1_SCL].ulPinConfiguration);

	NVIC_DisableIRQ(WIRE1_ISR_ID);
	NVIC_ClearPendingIRQ(WIRE1_ISR_ID);
	NVIC_SetPriority(WIRE1_ISR_ID, 0);
	NVIC_EnableIRQ(WIRE1_ISR_ID);
}

static void Wire1_Deinit(void) {
	NVIC_DisableIRQ(WIRE1_ISR_ID);
	NVIC_ClearPendingIRQ(WIRE1_ISR_ID);

	pmc_disable_periph_clk(WIRE1_INTERFACE_ID);

	// no need to undo PIO_Configure, 
	// as Peripheral A was enable by default before,
	// and pullups were not enabled
}

TwoWire Wire1 = TwoWire(WIRE1_INTERFACE, Wire1_Init, Wire1_Deinit);

void WIRE1_ISR_HANDLER(void) {
	Wire1.onService();
}
#endif
