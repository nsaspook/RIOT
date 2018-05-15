#include "automio.h"

static void ble_gpio_set(struct ble_automation_io *aio, uint8_t value)
{
	uint32_t pin = aio->gpio.pin_number;

	(void) pin;
	if (value) {
		//		nrf_gpio_pin_set(pin);
	} else {
		//		nrf_gpio_pin_clear(pin);
	}
}

static uint8_t ble_gpio_get(struct ble_automation_io *aio)
{
	uint8_t val;
	
	(void) val;
	if (aio->gpio.output) {
		val = BLE_DIGITAL_OUTPUT_STATE;
	} else {
		//		val = nrf_gpio_pin_read(aio->gpio.pin_number);
	}

	return val;
}

void ble_automation_io_refresh(struct ble_automation_io *aio)
{
	uint8_t value = ble_gpio_get(aio);
	if (value != aio->value_digital) {
		// TODO: notify clients
	}
	aio->value_digital = value;
}

void ble_init_gpio(struct ble_automation_io *aio)
{
	uint32_t pin = aio->gpio.pin_number;
	bool output = aio->gpio.output;

	(void) pin;
	if (output) {
		//		nrf_gpio_cfg_output(pin);
		ble_gpio_set(aio, (int) aio->gpio.default_value);
		ble_automation_io_refresh(aio);
	} else {
		//		nrf_gpio_cfg_input(pin, NRF_GPIO_PIN_NOPULL);
	}
}
