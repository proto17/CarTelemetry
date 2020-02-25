/*
* GccApplication1.cpp
*
* Created: 2/22/2020 1:52:29 PM
* Author : DavidProtzman
*/

#define F_CPU 16000000

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

// Define the pin direction possibilities
enum class pin_dir {
	INPUT, OUTPUT
};

/************************************************************************/
/* Initialize the serial connection                                     */
/************************************************************************/
static inline void init_serial(const uint32_t baud_rate) {
	// There are two possibilities for setting the baud rate: single rate, or double rate
	// Calculate the register value for both single and double rate (equations come from 
	// Atmel docs for the ATMEGA328P)
	const float float_single_ubrrh_val = (F_CPU / (16.0f * baud_rate)) - 1;
	const float float_double_ubrrh_val = (F_CPU / (8.0f * baud_rate)) - 1;
	
	// The calculated values are going to likely be some amount off from a whole number.  In 
	// order to get the cleanest possible UART signal, we need to be as close to an integer 
	// multiple of the CPU clock as possible.
	
	// The first step is to see if the baud rate is simply too high for the single rate option.
	// If so, then we have to go with the double rate option.  Otherwise, we need to figure out
	// which of the two is closer to a whole number
	
	uint16_t ubrrh_val;
	if (float_single_ubrrh_val >= 0) {
		// Get the distance to the nearest whole number for both the single and double rates
		const auto single_rate_delta = fabs(round(float_single_ubrrh_val) - float_single_ubrrh_val);
		const auto double_rate_delta = fabs(round(float_double_ubrrh_val) - float_double_ubrrh_val);
		
		// Check which one is smaller (closer to a whole number) and use that one
		if (single_rate_delta < double_rate_delta) {
			ubrrh_val = static_cast<uint16_t>(round(float_single_ubrrh_val));
		} else {
			ubrrh_val = static_cast<uint16_t>(round(float_double_ubrrh_val));
			// Using the double rate means that we also need to enable the double rate register
			UCSR0A |= (1 << U2X0);
		}
		
		/*
		const float delta_single = fabs(round(float_single_ubrrh_val) - float_single_ubrrh_val);
		const float delta_double = fabs(round(float_double_ubrrh_val) - float_double_ubrrh_val);
		
		if (abs(round(delta_single) - delta_single) < abs(round(delta_double) - delta_double)) {
			ubrrh_val = static_cast<uint16_t>(round(float_single_ubrrh_val));
		} else {
			ubrrh_val = static_cast<uint16_t>(round(float_double_ubrrh_val));
			UCSR0A |= (1 << U2X0);
		}
		*/
	} else {
		// Getting here means that the single rate isn't an options due to the baud rate being too high
		ubrrh_val = static_cast<uint16_t>(round(float_double_ubrrh_val));
		// Set the double rate register
		UCSR0A |= (1 << U2X0);
	}
	
	// Set the upper and lower bytes for the rate
	UBRR0H = (uint8_t)(ubrrh_val >> 8);
	UBRR0L = (uint8_t)(ubrrh_val & 0xff);
	
	// Enable TX and RX
	UCSR0B |= (1 << TXEN0) | (1 << RXEN0);
	// 8 data bits, no parity
	UCSR0C |= (0 << UMSEL00) | (0 << UMSEL01) | (0 << UPM00) | (0 << UPM01) | (0 << USBS0) | (1 << UCSZ00) | (1 << UCSZ01) | (0 << UCSZ02);
}

static inline void send_serial(const char data) {
	// Wait for the write data register to be ready
	while (! (UCSR0A & (1 << UDRE0)));
	// Write the byte to the serial output (does not wait for the data to be sent)
	UDR0 = data;
}

static inline void send_serial(const char * data, const uint16_t data_len=0) {
	if (data_len != 0) {
		// There is a data length value, so only send up to that many bytes
		for (uint16_t data_idx = 0; data_idx < data_len; data_idx++) {
			send_serial(data[data_idx]);
		}
	} else {
		// There is not a data length value, so send bytes until the first null is encountered.
		uint16_t idx = 0;
		while (data[idx] != '\0') {
			send_serial(data[idx]);
			idx++;
		}
	}
	
}

struct pin_t {
	// Get a pointer to the PORTX register defined by the provided char (ex: input value of 'D' would return PORTD's pointer)
	// Must return volatile pointer due to address being to a register!
	static volatile uint8_t * get_port_ptr(const char port) {
		switch(port) {
			
			// For compatibility with boards that don't have all the same ports, using ifdef's to only allow a port
			// to be selected if it actually exists
			
			#ifdef PORTB
			case 'B':
				return &PORTB;
				break;
			#endif

			#ifdef PORTC
			case 'C':
				return &PORTC;
				break;
			#endif

			#ifdef PORTD
			case 'D':
				return &PORTD;
				break;
			#endif

			// Use a null to signify that there has been an error
			default:
				return 0x0;
		}
	}

	// Get a pointer to the PINX register defined by the provided char (ex: input value of 'D' would return PIND's pointer)
	// Must return volatile pointer due to address being to a register!
	static volatile uint8_t * get_pin_ptr(const char port) {
		switch(port) {
			// For compatibility with boards that don't have all the same ports, using ifdef's to only allow a port
			// to be selected if it actually exists
			
			#ifdef PINB
			case 'B':
				return &PINB;
				break;
			#endif

			#ifdef PINC
			case 'C':
				return &PINC;
				break;
			#endif

			#ifdef PIND
			case 'D':
				return &PIND;
				break;
			#endif

			// Use a null to signify that there has been an error
			default:
				return 0x0;
		}
	}

	// Get a pointer to the DDRX register defined by the provided char (ex: input value of 'D' would return DDRD's pointer)
	// Must return volatile pointer due to address being to a register!
	static volatile uint8_t * get_ddr_ptr(const char port) {
		// For compatibility with boards that don't have all the same ports, using ifdef's to only allow a port
		// to be selected if it actually exists
		
		switch(port) {
			#ifdef PORTB
			case 'B':
				return &DDRB;
				break;
			#endif

			#ifdef PORTC
			case 'C':
				return &DDRC;
				break;
			#endif

			#ifdef PORTD
			case 'D':
				return &DDRD;
				break;
			#endif

			// Use a null to signify that there has been an error
			default:
				return 0x0;
		}
	}
	
	pin_t(
		// PORTx pointer
		volatile uint8_t * const port_addr, 
		// DDRx pointer
	    volatile uint8_t * const ddr_addr, 
		// PINx pointer
		volatile uint8_t * const pin_addr, 
		// Bit offset in each of the pointers for this pin
		const uint8_t bit_offset, 
		// Initial direction for this pin (input or output)
		const pin_dir & dir=pin_dir::OUTPUT) :
			
			// Copy the various pointers and the offset
			port_addr_(port_addr), 
			ddr_addr_(ddr_addr), 
			pin_addr_(pin_addr), 
			bit_offset_(bit_offset), 
			// Pre-calculate the mask and inverse mask
			mask_(1 << bit_offset), 
			inv_mask_(~(1 << bit_offset)), 
			
			// Set the default direction
			dir_(dir) {
		// Nothing to do in the body of the constructor
	}

	pin_t(
		// Single character that defines the port (A, B, C, D, etc) MUST BE CAPS!!!
		const char port, 
		// Pin number in the specified port
		const int pin_num) : 
			// Call the main constructor
			pin_t(
				// Get the PORTx pointer
				get_port_ptr(port), 
				// Get the DDRx pointer
				get_ddr_ptr(port), 
				// Get the PINx pointer
				get_pin_ptr(port), 
				
				pin_num) {
		// Nothing to do in the body of the constructor
	}

	pin_t(
		// String of two characters that define the port and pin offset (ex: "D2" for PORTD pin 2)
		const char * const port_and_pin) : 
			// Call the above constructor
			pin_t(
				// Get the port letter
				port_and_pin[0], 
				// Convert the second character to an int
				(int)atoi(&port_and_pin[1])) {
		// Nothing to do in the body of the constructor
	}

	volatile uint8_t * const port_addr_;
	volatile uint8_t * const ddr_addr_;
	volatile uint8_t * const pin_addr_;
	const uint8_t bit_offset_;
	const uint8_t mask_;
	const uint8_t inv_mask_;
	pin_dir dir_;

	// Make sure that the PORTx, DDRx, and PINx pointers are valid
	bool is_valid() const {
		return ddr_addr_ != 0x0 && port_addr_ != 0x0 && pin_addr_ != 0x0;
	}

	// Setup the port for reading or writing
	inline void setup() {
		if (dir_ == pin_dir::OUTPUT) {
			*ddr_addr_ |= mask_;
		} else {
			*ddr_addr_ &= inv_mask_;
			// Disables the pullup by default
			*port_addr_ &= inv_mask_;
		}
	}

	// Change the pin direction (input or output)
	inline void set_dir(const pin_dir & dir) {
		dir_ = dir;
		setup();
	}
	
	// Set the pin value (high or low)
	inline void write(const bool high) {
		if (high) {
			*port_addr_ |= mask_;
		} else {
			*port_addr_ &= inv_mask_;
		}
	}
	
	// Get the current input value of the pin
	inline bool read() {
		return (((*pin_addr_) & mask_));
	}
};

bool validate_pins(pin_t * const pins, const uint8_t pin_count) {
	for (uint8_t pin_idx = 0; pin_idx < pin_count; pin_idx++) {
		if (! pins[pin_idx].is_valid()) {
			return false;
		}
	}

	return true;
}

void setup_pins(pin_t * const pins, const uint8_t pin_count) {
	for (uint8_t pin_idx = 0; pin_idx < pin_count; pin_idx++) {
		pins[pin_idx].setup();
	}
}


const uint8_t PIN_COUNT = 3;
pin_t pins[PIN_COUNT] = {
		{"C3"},
		{"D2"}, {"D3"}
};
auto & led_pin = pins[0];
auto & power_loss_input = pins[1];
auto & led2 = pins[2];

int main(void)
{	
	DDRC = 0xff;
	init_serial(2000000);
	uint8_t a = 0;
	char buff[5];
	

	if (! validate_pins(pins, PIN_COUNT)) {
		//ACK
		DDRC = 0xff;
		PORTC = 0x00;
	} else {

		setup_pins(pins, PIN_COUNT);
		led_pin.set_dir(pin_dir::OUTPUT);
		led_pin.write(true);
		led2.set_dir(pin_dir::OUTPUT);
		led2.write(true);
		power_loss_input.set_dir(pin_dir::INPUT);
		
		DDRD &= (~(1 << PORTD2));

		while (1)
		{
			led2.write(true);
			_delay_ms(100);
			led2.write(false);
			_delay_ms(100);
			
			const auto v = power_loss_input.read();
			led_pin.write(v);
			
			if ( v == false ) {
				send_serial("LOST POWER\n");
			} else {
				send_serial("This is a test ");
				itoa(a, buff, 10);
				send_serial(buff);
				send_serial("\n");
				a++;
			}
		}
	}
	



	
}
