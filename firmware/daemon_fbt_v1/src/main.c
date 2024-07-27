#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/drivers/flash/nrf_qspi_nor.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <inttypes.h>
#include <string.h>
#include <stddef.h>
#include "bhi360.h"
#include <stdio.h>
#include <nrf.h>
#include <esb.h>
/*

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)
const struct gpio_dt_spec battery_low = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, battery_low_gpios);

*/
#define _RADIO_SHORTS_COMMON                                                   \
	(RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk |         \
	 RADIO_SHORTS_ADDRESS_RSSISTART_Msk |                                  \
	 RADIO_SHORTS_DISABLED_RSSISTOP_Msk)


int clocks_start(void)
{
	int err;
	int res;
	struct onoff_manager *clk_mgr;
	struct onoff_client clk_cli;

	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	sys_notify_init_spinwait(&clk_cli.notify);
	onoff_request(clk_mgr, &clk_cli);

	do {
	    err = sys_notify_fetch_result(&clk_cli.notify, &res);
		if (!err && res) {
            // clock could not be started
			return res;
		}
	} while (err);

	return 0;
}

int esb_initialize(void)
{
	//These are arbitrary default addresses. In end user products
	//different addresses should be used for each set of devices.
	 
	uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
	uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
	uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8};

	struct esb_config config = ESB_DEFAULT_CONFIG;

	config.protocol = ESB_PROTOCOL_ESB_DPL;
	config.retransmit_delay = 600;
	config.bitrate = ESB_BITRATE_2MBPS;
	config.event_handler = NULL;
	config.mode = ESB_MODE_PTX;
	config.selective_auto_ack = true;

	esb_init(&config);
	esb_set_base_address_0(base_addr_0);
	esb_set_base_address_1(base_addr_1);
	esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));

	return 0;
}

static struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);

int main(void) {
	printk("build time: " __DATE__ " " __TIME__);
	// while (1) {
	// 		printk("AYOOOO!\n");
	// 		k_sleep(K_MSEC(1000));
	// }
    clocks_start();
	esb_initialize();

	struct spi_cs_control spi_cs = {
		//P1.08 as CS pin
		.gpio = {
			.port = DEVICE_DT_GET(DT_NODELABEL(imu_gpio)),
			.pin = 8,
			.dt_flags = GPIO_ACTIVE_LOW | GPIO_OPEN_DRAIN
		},
		.delay = 0
	};

	struct spi_config spi_cfg = {
		.frequency = 20000000,
		.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
		.slave = 0,
		.cs = spi_cs
	};

	struct spi_dt_spec spi_spec = {
		.bus = DEVICE_DT_GET(DT_NODELABEL(imu_spi)),
		.config = spi_cfg
	};

	// initialize BHI360
	reset_imu(&spi_spec);
	mask_all_interrupts(&spi_spec);
	read_interrupt_register(&spi_spec);
	wait_for_interface_ready(&spi_spec);
	read_fuser2_id(&spi_spec);
	read_fuser2_revision(&spi_spec);
	read_fuser2_rom_version(&spi_spec);
	write_firmware(&spi_spec);
	wait_for_firmware_verified(&spi_spec);
	boot_bhi360_program_ram(&spi_spec);
	wait_for_interface_ready(&spi_spec);
	read_wakeup_fifo_initialized(&spi_spec);
	read_non_wakeup_fifo_initialized(&spi_spec);
	read_interrupt_status_register(&spi_spec);

	// check if game rotation is available
	read_sensor_present(&spi_spec);
	k_msleep(100);
	read_interrupt_status_register(&spi_spec);

	// enable non-wakeup game rotation and read non-wakeup FIFO
	read_game_rotation_virtual_sensor_information(&spi_spec);
	k_msleep(100);
	read_interrupt_status_register(&spi_spec);
	read_game_rotation_virtual_sensor_configuration(&spi_spec);
	k_msleep(100);
	configure_game_rotation_sensor(&spi_spec);
	k_msleep(100);
	configure_linear_acceleration_sensor(&spi_spec);
	k_msleep(100);
	read_interrupt_status_register(&spi_spec);
	k_msleep(100);
	read_game_rotation_virtual_sensor_configuration(&spi_spec);
	k_msleep(100);
	read_interrupt_status_register(&spi_spec);

	// quat is 8 bytes
	struct quaternion quat;
	struct acc_vector vec;

	while (1) {
		read_non_wakeup_fifo(&spi_spec, &quat, &vec);
		esb_flush_tx();
		tx_payload.data[0] = quat.x & 0xFF;
		tx_payload.data[1] = (quat.x & 0xFF00) >> 8;
		tx_payload.data[2] = quat.y & 0xFF;
		tx_payload.data[3] = (quat.y & 0xFF00) >> 8;
		tx_payload.data[4] = quat.z & 0xFF;
		tx_payload.data[5] = (quat.z & 0xFF00) >> 8;
		tx_payload.data[6] = quat.w & 0xFF;
		tx_payload.data[7] = (quat.w & 0xFF00) >> 8;

		tx_payload.data[8] = vec.x & 0xFF;
		tx_payload.data[9] = (vec.x & 0xFF00) >> 8;
		tx_payload.data[10] = vec.y & 0xFF;
		tx_payload.data[11] = (vec.y & 0xFF00) >> 8;
		tx_payload.data[12] = vec.z & 0xFF;
		tx_payload.data[13] = (vec.z & 0xFF00) >> 8;
		esb_write_payload(&tx_payload);
		// better throughput with a little wait time
		k_sleep(K_MSEC(1));
	}

	return 0;
}
