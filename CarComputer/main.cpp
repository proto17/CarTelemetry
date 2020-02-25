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

enum class pin_dir {
	INPUT, OUTPUT
};

static inline void init_serial(const uint32_t baud_rate) {
	const float float_single_ubrrh_val = (F_CPU / (16.0f * baud_rate)) - 1;
	const float float_double_ubrrh_val = (F_CPU / (8.0f * baud_rate)) - 1;
	
	uint16_t ubrrh_val;
	if (float_single_ubrrh_val >= 0) {
		const float delta_single = fabs(round(float_single_ubrrh_val) - float_single_ubrrh_val);
		const float delta_double = fabs(round(float_double_ubrrh_val) - float_double_ubrrh_val);
		
		if (abs(round(delta_single) - delta_single) < abs(round(delta_double) - delta_double)) {
			ubrrh_val = static_cast<uint16_t>(round(float_single_ubrrh_val));
			} else {
				ubrrh_val = static_cast<uint16_t>(round(float_double_ubrrh_val));
				UCSR0A |= (1 << U2X0);
			}
	} else {
		ubrrh_val = static_cast<uint16_t>(round(float_double_ubrrh_val));
		UCSR0A |= (1 << U2X0);
	}
	
	UBRR0H = (uint8_t)(ubrrh_val >> 8);
	UBRR0L = (uint8_t)(ubrrh_val & 0xff);
	UCSR0B |= (1 << TXEN0);
	UCSR0C |= (0 << UMSEL00) | (0 << UMSEL01) | (0 << UPM00) | (0 << UPM01) | (0 << USBS0) | (1 << UCSZ00) | (1 << UCSZ01) | (0 << UCSZ02);
}

static inline void send_serial(const char data) {
	while (! (UCSR0A & (1 << UDRE0)));
	UDR0 = data;
}

static inline void send_serial(const char * data, const uint16_t data_len=0) {
	if (data_len != 0) {
		for (uint16_t data_idx = 0; data_idx < data_len; data_idx++) {
			send_serial(data[data_idx]);
		}
	} else {
		uint16_t idx = 0;
		do {
			send_serial(data[idx]);
			idx++;
		} while(data[idx] != '\0');
	}
	
}

struct pin_t {
	static volatile uint8_t * get_port_ptr(const char port) {
		switch(port) {
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

			default:
			return 0x0;
		}
	}

	static volatile uint8_t * get_pin_ptr(const char port) {
		switch(port) {
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

			default:
			return 0x0;
		}
	}

	static volatile uint8_t * get_ddr_ptr(const char port) {
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

			default:
			return 0x0;
		}
	}

	pin_t(volatile uint8_t * const port_addr, volatile uint8_t * const ddr_addr, volatile uint8_t * const pin_addr, const uint8_t bit_offset, const pin_dir & dir=pin_dir::OUTPUT) :
	port_addr_(port_addr), ddr_addr_(ddr_addr), pin_addr_(pin_addr), bit_offset_(bit_offset), mask_(1 << bit_offset), inv_mask_(~(1 << bit_offset)), dir_(dir) {}

	pin_t(const char port, const int pin_num) : pin_t(get_port_ptr(port), get_ddr_ptr(port), get_pin_ptr(port), pin_num) {}

	pin_t(const char * const port_and_pin) : pin_t(port_and_pin[0], (int)atoi(&port_and_pin[1])) {}

	volatile uint8_t * const port_addr_;
	volatile uint8_t * const ddr_addr_;
	volatile uint8_t * const pin_addr_;
	const uint8_t bit_offset_;
	const uint8_t mask_;
	const uint8_t inv_mask_;
	pin_dir dir_;

	bool is_valid() const {
		return ddr_addr_ != 0x0 && port_addr_ != 0x0;
	}

	inline void setup() {
		if (dir_ == pin_dir::OUTPUT) {
			*ddr_addr_ |= mask_;
		} else {
			*ddr_addr_ &= inv_mask_;
			*port_addr_ &= inv_mask_;
		}
	}

	inline void set_dir(const pin_dir & dir) {
		dir_ = dir;
		setup();
	}

	inline void write(const bool high) {
		if (high) {
			*port_addr_ |= mask_;
		} else {
			*port_addr_ &= inv_mask_;
		}
	}
	
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
