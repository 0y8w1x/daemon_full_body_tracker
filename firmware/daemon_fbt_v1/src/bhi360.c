#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/spi.h>
#include <stdint.h>
#include "bhi360.h"

#define N_READ_DUMMY_BYTES 1

int reset_imu(struct spi_dt_spec *spi_spec) {
	uint8_t tx_buf[] = {WRITE_REG(0x14), 0x01};
	struct spi_buf tx_spi_buf		= {.buf = tx_buf, .len = sizeof(tx_buf)};
	struct spi_buf_set tx_spi_buf_set 	= {.buffers = &tx_spi_buf, .count = 1};

	int err = spi_write_dt(spi_spec, &tx_spi_buf_set);
	if (err < 0) {
		printk("spi_transceive_dt() failed, err: %d", err);
		return err;
	} else {
		printk("resetting BHI360.\n");
	}
    return 0;
}

int mask_all_interrupts(struct spi_dt_spec *spi_spec) {
	uint8_t tx_buf2[] = {WRITE_REG(0x07), 0x1F};	
	struct spi_buf	tx_spi_buf2 		= {.buf = tx_buf2, .len = sizeof(tx_buf2)};
	struct spi_buf_set tx_spi_buf_set2	= {.buffers = &tx_spi_buf2, .count = 1};

	int err = spi_write_dt(spi_spec, &tx_spi_buf_set2);
	if (err < 0) {
		printk("spi_write_dt() failed, err %d", err);
		return err;
	} else {
		printk("masked all interrupts.\n");
	}
    return 0;
}

int read_interrupt_register(struct spi_dt_spec *spi_spec) {
	uint8_t interrupt_register_read[1 + N_READ_DUMMY_BYTES];
	uint8_t tx_buffer11 = READ_REG(0x07);
	struct spi_buf tx_spi_buf11		= {.buf = &tx_buffer11, .len = 1};
	struct spi_buf_set tx_spi_buf_set11 	= {.buffers = &tx_spi_buf11, .count = 1};
	struct spi_buf rx_spi_buf11 		= {.buf = interrupt_register_read, .len = sizeof(interrupt_register_read)};
	struct spi_buf_set rx_spi_buf_set11	= {.buffers = &rx_spi_buf11, .count = 1};
	k_msleep(1000);
	int err = spi_transceive_dt(spi_spec, &tx_spi_buf_set11, &rx_spi_buf_set11);
	if (err < 0) {
		printk("spi_transceive_dt() failed, err: %d", err);
		return err;
	}
	printk("interrupt control register: %02x\n", interrupt_register_read[1]);
    return 0;
}

int wait_for_interface_ready(struct spi_dt_spec *spi_spec) {
	int size = 1 + N_READ_DUMMY_BYTES;
	uint8_t boot_status[size];

	uint8_t tx_buffer3 = READ_REG(0x25);
	struct spi_buf tx_spi_buf3		= {.buf = &tx_buffer3, .len = 1};
	struct spi_buf_set tx_spi_buf_set3 	= {.buffers = &tx_spi_buf3, .count = 1};
	struct spi_buf rx_spi_buf3 		= {.buf = boot_status, .len = size};
	struct spi_buf_set rx_spi_buf_set3	= {.buffers = &rx_spi_buf3, .count = 1};

	int err = 0;
	while (1) {
		err = spi_transceive_dt(spi_spec, &tx_spi_buf_set3, &rx_spi_buf_set3);
		if (err < 0) {
			printk("spi_transceive_dt() failed, err: %d", err);
			return err;
		}
		if (boot_status[1] & HOST_INTERFACE_READY_BIT)
			break;
		printk("waiting for SPI interface ready...\n");
		k_msleep(100);
	}
	printk("SPI interface ready.\n");
    return 0;
}

int read_fuser2_id(struct spi_dt_spec *spi_spec) {
	int size = 1 + N_READ_DUMMY_BYTES;
	uint8_t fuser2_id[size];
	uint8_t tx_buffer4 = READ_REG(0x1C);
	struct spi_buf tx_spi_buf4		= {.buf = &tx_buffer4, .len = 1};
	struct spi_buf_set tx_spi_buf_set4 	= {.buffers = &tx_spi_buf4, .count = 1};
	struct spi_buf rx_spi_buf4 		= {.buf = fuser2_id, .len = size};
	struct spi_buf_set rx_spi_buf_set4	= {.buffers = &rx_spi_buf4, .count = 1};
	int err = spi_transceive_dt(spi_spec, &tx_spi_buf_set4, &rx_spi_buf_set4);
	if (err < 0) {
		printk("spi_transceive_dt() failed, err: %d", err);
		return err;
	}
	printk("fuser id (should be 0x89): 0x%02x\n", fuser2_id[1]);
    return 0;
}

int read_fuser2_revision(struct spi_dt_spec *spi_spec) {
	int size = 1 + N_READ_DUMMY_BYTES;
	uint8_t fuser2_rev[size];
	uint8_t tx_buffer5 = READ_REG(0x1D);
	struct spi_buf tx_spi_buf5		= {.buf = &tx_buffer5, .len = 1};
	struct spi_buf_set tx_spi_buf_set5 	= {.buffers = &tx_spi_buf5, .count = 1};
	struct spi_buf rx_spi_buf5 		= {.buf = fuser2_rev, .len = size};
	struct spi_buf_set rx_spi_buf_set5	= {.buffers = &rx_spi_buf5, .count = 1};
	int err = spi_transceive_dt(spi_spec, &tx_spi_buf_set5, &rx_spi_buf_set5);
	if (err < 0) {
		printk("spi_transceive_dt() failed, err: %d", err);
		return err;
	}
	printk("fuser rev (should be 0x02): 0x%02x\n", fuser2_rev[1]);
    return 0;
}

int read_fuser2_rom_version(struct spi_dt_spec *spi_spec) {
	// read fuser ROM low byte
	int size = 1 + N_READ_DUMMY_BYTES;
	uint8_t fuser2_rom_low[size];
	uint8_t tx_buffer6 = READ_REG(0x1E);
	struct spi_buf tx_spi_buf6		= {.buf = &tx_buffer6, .len = 1};
	struct spi_buf_set tx_spi_buf_set6 	= {.buffers = &tx_spi_buf6, .count = 1};
	struct spi_buf rx_spi_buf6 		= {.buf = fuser2_rom_low, .len = size};
	struct spi_buf_set rx_spi_buf_set6	= {.buffers = &rx_spi_buf6, .count = 1};
	int err = spi_transceive_dt(spi_spec, &tx_spi_buf_set6, &rx_spi_buf_set6);
	if (err < 0) {
		printk("spi_transceive_dt() failed, err: %d", err);
		return err;
	}
	printk("fuser ROM low byte (should be 0x2E): 0x%02x\n", fuser2_rom_low[1]);

	// read fuser ROM high byte
	size = 1 + N_READ_DUMMY_BYTES;
	uint8_t fuser2_rom_high[size];
	uint8_t tx_buffer7 = READ_REG(0x1F);
	struct spi_buf tx_spi_buf7		= {.buf = &tx_buffer7, .len = 1};
	struct spi_buf_set tx_spi_buf_set7 	= {.buffers = &tx_spi_buf7, .count = 1};
	struct spi_buf rx_spi_buf7 		= {.buf = fuser2_rom_high, .len = size};
	struct spi_buf_set rx_spi_buf_set7	= {.buffers = &rx_spi_buf7, .count = 1};
	err = spi_transceive_dt(spi_spec, &tx_spi_buf_set7, &rx_spi_buf_set7);
	if (err < 0) {
		printk("spi_transceive_dt() failed, err: %d", err);
		return err;
	}
	printk("fuser ROM high byte (should be 0x14): 0x%02x\n", fuser2_rom_high[1]);
    return 0;
}

int write_firmware(struct spi_dt_spec *spi_spec) {
	const struct device *const qspi_dev = DEVICE_DT_GET(FLASH_NODE);
	if (!device_is_ready(qspi_dev)) {
		printf("%s: device not ready\n", qspi_dev->name);
		return -1;
	}

	// command "Upload to Program RAM" = 0x0002, and LSB = 0x02 0x00
	// length of firmware is 127228 bytes = 31807 words = 0x7C3F, and LSB = 0x3F 0x7C
	int total_length = 127228;
	// 68 is cleanly dividable by 4, and also cleanly divides 127228. nice properties, less code
	// the speed can be increase, but more codes is needed to check that everything aligns
	int firmware_packet_length = 68;
	// plus 1 for the register byte at the beginning
	uint8_t qspi_buf[firmware_packet_length+1];
	qspi_buf[0] = WRITE_REG(0x00);
	int offset = 0x00;
	int n_bytes_left = total_length;
	int packets_sent = 0;

	// command "Upload to Program RAM" = 0x0002, and LSB = 0x02 0x00
	// length is 127228, divided by 4 = 31807 = 0x7C3F, and LSB = 0x3F 0x7C
	uint8_t tx_buf[] = {WRITE_REG(0x00), 0x02, 0x00, 0x3F, 0x7C};
	struct spi_buf tx_spi_buf = 		{.buf = 	tx_buf, 		.len = 		sizeof(tx_buf)};
	struct spi_buf_set tx_spi_buf_set = {.buffers = &tx_spi_buf, 	.count = 	1};

	printk("writing firmware...\n");
	int err = spi_write_dt(spi_spec, &tx_spi_buf_set);
	if (err < 0) {
		printk("spi_write_dt() failed, err %d", err);
		return err;
	}

	while (n_bytes_left) {
		// read from qspi
		err = flash_read(qspi_dev, offset, &qspi_buf[1], firmware_packet_length);
		if (err != 0) {
			printk("Flash read failed! %d\n", err);
			return -1;
		}

		// write over spi
		// uint8_t tx_buf2[] = {WRITE_REG(0x00), qspi_buf[0], qspi_buf[1], qspi_buf[2], qspi_buf[3]};
		struct spi_buf tx_spi_buf2 = {.buf = qspi_buf, .len = sizeof(qspi_buf)};
		struct spi_buf_set tx_spi_buf_set2	= {.buffers = &tx_spi_buf2, .count = 1};
		err = spi_write_dt(spi_spec, &tx_spi_buf_set2);
		if (err < 0) {
			printk("spi_write_dt() failed, err %d", err);
			return err;
		} else {
			offset += firmware_packet_length;
			packets_sent++;
			n_bytes_left -= firmware_packet_length;
		}
		
		// printk("progress: %d/127228\n", packets_sent*firmware_packet_length);
		// k_msleep(1);
	}
    return 0;
}

int wait_for_firmware_verified(struct spi_dt_spec *spi_spec) {
	int size = 1 + N_READ_DUMMY_BYTES;
	uint8_t boot_status[size];

	uint8_t tx_buffer3 = READ_REG(0x25);
	struct spi_buf tx_spi_buf3		= {.buf = &tx_buffer3, .len = 1};
	struct spi_buf_set tx_spi_buf_set3 	= {.buffers = &tx_spi_buf3, .count = 1};
	struct spi_buf rx_spi_buf3 		= {.buf = boot_status, .len = size};
	struct spi_buf_set rx_spi_buf_set3	= {.buffers = &rx_spi_buf3, .count = 1};

	int err = 0;
	while (1) {
		err = spi_transceive_dt(spi_spec, &tx_spi_buf_set3, &rx_spi_buf_set3);
		if (err < 0) {
			printk("spi_transceive_dt() failed, err: %d", err);
			return err;
		}

		if (boot_status[1] & FIRMWARE_VERIFY_DONE_BIT)
			break;

		k_msleep(100);
		printk("waiting for firmware verification...\n");
	}
	printk("firmware verified.\n");
    return 0;
}

int boot_bhi360_program_ram(struct spi_dt_spec *spi_spec) {
	uint8_t tx_buf[] = {WRITE_REG(0x00), 0x03, 0x00, 0x00, 0x00};
	struct spi_buf tx_spi_buf = 		{.buf = 	tx_buf, 		.len = 		sizeof(tx_buf)};
	struct spi_buf_set tx_spi_buf_set = {.buffers = &tx_spi_buf, 	.count = 	1};

	int err = spi_write_dt(spi_spec, &tx_spi_buf_set);
	if (err < 0) {
		printk("spi_write_dt() failed, err %d", err);
		return err;
	}
	printk("bit set, to start BHI360 firmware.\n");
    return 0;
}

int read_wakeup_fifo_initialized(struct spi_dt_spec *spi_spec) {
	int size = 2 + N_READ_DUMMY_BYTES;
	uint8_t wakeup_fifo[size];
	uint8_t tx_buffer = READ_REG(0x01);
	struct spi_buf tx_spi_buf		= {.buf = &tx_buffer, .len = 1};
	struct spi_buf_set tx_spi_buf_set 	= {.buffers = &tx_spi_buf, .count = 1};
	struct spi_buf rx_spi_buf 		= {.buf = wakeup_fifo, .len = size};
	struct spi_buf_set rx_spi_buf_set	= {.buffers = &rx_spi_buf, .count = 1};
	int err = spi_transceive_dt(spi_spec, &tx_spi_buf_set, &rx_spi_buf_set);
	if (err < 0) {
		printk("spi_transceive_dt() failed, err: %d", err);
		return err;
	}
	uint16_t nbytes_in_wakeup_fifo = wakeup_fifo[1] | wakeup_fifo[2] << 8;
	printk("bytes in wakeup fifo: %d\n", nbytes_in_wakeup_fifo);

	if (nbytes_in_wakeup_fifo == 0) {
		printk("nothing in wakeup fifo.\n");
		return -1;
	}

	// 2 bytes less, because the initial read of the transfer length is 2 bytes.
	printk("reading %d bytes of wakeup fifo...\n", nbytes_in_wakeup_fifo);
	size = nbytes_in_wakeup_fifo + N_READ_DUMMY_BYTES;
	uint8_t wakeup_fifo_content[size];
	uint8_t tx_buffer2 = READ_REG(0x01);
	struct spi_buf tx_spi_buf2		= {.buf = &tx_buffer2, .len = 1};
	struct spi_buf_set tx_spi_buf_set2 	= {.buffers = &tx_spi_buf2, .count = 1};
	struct spi_buf rx_spi_buf2 		= {.buf = wakeup_fifo_content, .len = size};
	struct spi_buf_set rx_spi_buf_set2	= {.buffers = &rx_spi_buf2, .count = 1};
	err = spi_transceive_dt(spi_spec, &tx_spi_buf_set2, &rx_spi_buf_set2);

	// uint16_t delta_timestamp = wakeup_fifo_content[1] | wakeup_fifo_content[2] << 8;
	uint8_t fifo_event_id = wakeup_fifo_content[3];
	// ID for wakeup Meta Event
	if (fifo_event_id == 248) {
		uint8_t meta_event_type = wakeup_fifo_content[4];
		// meta event type for Spacer
		if (meta_event_type == 20) {
			uint16_t block_count = wakeup_fifo_content[5] | wakeup_fifo_content[6] << 8;
			printk("block count: %d\n", block_count);

			uint8_t fifo_event_id2 = wakeup_fifo_content[7];
			// event ID for Full Timestamp wakeup FIFO
			if (fifo_event_id2 == 247) {
				// int timestamp = wakeup_fifo_content[8] | wakeup_fifo_content[9] << 8
				// 					| wakeup_fifo_content[10] << 16 | wakeup_fifo_content[11] << 24
				// 					| wakeup_fifo_content[12] << 32;
				// printk("timestamp: %d\n", timestamp);

				uint8_t fifo_event_id3 = wakeup_fifo_content[13];
				// ID for wakeup Meta Event
				if (fifo_event_id3 == 248) {
					uint8_t meta_event_type2 = wakeup_fifo_content[14];
					// meta event type for Initialized
					if (meta_event_type2 == 16) {
						uint16_t ram_version = wakeup_fifo_content[15] | wakeup_fifo_content[16] << 8;
						printk("initialized with RAM version: %d\n", ram_version);
					}
				}
			}
		}
	}
    return 0;
}

int read_non_wakeup_fifo_initialized(struct spi_dt_spec *spi_spec) {
	int size = 2 + N_READ_DUMMY_BYTES;
	uint8_t non_wakeup_fifo[size];
	uint8_t tx_buffer = READ_REG(0x02);
	struct spi_buf tx_spi_buf		= {.buf = &tx_buffer, .len = 1};
	struct spi_buf_set tx_spi_buf_set 	= {.buffers = &tx_spi_buf, .count = 1};
	struct spi_buf rx_spi_buf 		= {.buf = non_wakeup_fifo, .len = size};
	struct spi_buf_set rx_spi_buf_set	= {.buffers = &rx_spi_buf, .count = 1};
	int err = spi_transceive_dt(spi_spec, &tx_spi_buf_set, &rx_spi_buf_set);
	if (err < 0) {
		printk("spi_transceive_dt() failed, err: %d", err);
		return err;
	}
	uint16_t nbytes_in_non_wakeup_fifo = non_wakeup_fifo[1] | non_wakeup_fifo[2] << 8;
	printk("bytes in non wakeup fifo: %d\n", nbytes_in_non_wakeup_fifo);

	if (nbytes_in_non_wakeup_fifo == 0) {
		printk("nothing in non-wakeup fifo.\n");
		return -1;
	}

	printk("reading %d bytes of non-wakeup fifo...\n", nbytes_in_non_wakeup_fifo);
	size = nbytes_in_non_wakeup_fifo + N_READ_DUMMY_BYTES;
	uint8_t non_wakeup_fifo_content[size];
	uint8_t tx_buffer2 = READ_REG(0x02);
	struct spi_buf tx_spi_buf2		= {.buf = &tx_buffer2, .len = 1};
	struct spi_buf_set tx_spi_buf_set2 	= {.buffers = &tx_spi_buf2, .count = 1};
	struct spi_buf rx_spi_buf2 		= {.buf = non_wakeup_fifo_content, .len = size};
	struct spi_buf_set rx_spi_buf_set2	= {.buffers = &rx_spi_buf2, .count = 1};
	err = spi_transceive_dt(spi_spec, &tx_spi_buf_set2, &rx_spi_buf_set2);

	// uint16_t delta_timestamp = non_wakeup_fifo_content[1] | non_wakeup_fifo_content[2] << 8;
	uint8_t fifo_event_id = non_wakeup_fifo_content[3];
	// ID for non-wakeup Meta Event
	if (fifo_event_id == 254) {
		uint8_t meta_event_type = non_wakeup_fifo_content[4];
		// meta event type for Spacer
		if (meta_event_type == 20) {
			uint16_t block_count = non_wakeup_fifo_content[5] | non_wakeup_fifo_content[6] << 8;
			printk("block count: %d\n", block_count);

			uint8_t fifo_event_id2 = non_wakeup_fifo_content[7];
			// event ID for Full Timestamp non-wakeup FIFO
			if (fifo_event_id2 == 253) {
				// int timestamp = non_wakeup_fifo_content[8] | non_wakeup_fifo_content[9] << 8
				// 					| non_wakeup_fifo_content[10] << 16 | non_wakeup_fifo_content[11] << 24
				// 					| non_wakeup_fifo_content[12] << 32;
				// printk("timestamp: %d\n", timestamp);

				uint8_t fifo_event_id3 = non_wakeup_fifo_content[13];
				// ID for wakeup Meta Event
				if (fifo_event_id3 == 254) {
					uint8_t meta_event_type2 = non_wakeup_fifo_content[14];
					// meta event type for Initialized
					if (meta_event_type2 == 16) {
						uint16_t ram_version = non_wakeup_fifo_content[15] | non_wakeup_fifo_content[16] << 8;
						printk("initialized with RAM version: %d\n", ram_version);
					}
				}
			}
		}
	}
    return 0;
}

int read_interrupt_status_register(struct spi_dt_spec *spi_spec) {
	int size = 1 + N_READ_DUMMY_BYTES;
	uint8_t interrupt_status[size];
	uint8_t tx_buffer = READ_REG(0x2D);
	struct spi_buf tx_spi_buf		= {.buf = &tx_buffer, .len = 1};
	struct spi_buf_set tx_spi_buf_set 	= {.buffers = &tx_spi_buf, .count = 1};
	struct spi_buf rx_spi_buf 		= {.buf = interrupt_status, .len = size};
	struct spi_buf_set rx_spi_buf_set	= {.buffers = &rx_spi_buf, .count = 1};
	int err = spi_transceive_dt(spi_spec, &tx_spi_buf_set, &rx_spi_buf_set);
	if (err < 0) {
		printk("spi_transceive_dt() failed, err: %d", err);
		return err;
	}
	printk("interrupt status: 0x%02x\n", interrupt_status[1]);
    return 0;
}

int read_sensor_present(struct spi_dt_spec *spi_spec) {
	// send "Virtual Sensors Present" (0x011F) System Parameter command to channel 0 (0x00)
	// and because we want to read, send 0x111F, and LSB = 0x1F 0x11
	// total number of bytes to follow is 0, and LSB = 0x00 0x00
	uint8_t tx_buf[] = {WRITE_REG(0x00), 0x1F, 0x11, 0x00, 0x00};
	struct spi_buf tx_spi_buf = 		{.buf = 	tx_buf, 		.len = 		sizeof(tx_buf)};
	struct spi_buf_set tx_spi_buf_set = {.buffers = &tx_spi_buf, 	.count = 	1};
	int err = spi_write_dt(spi_spec, &tx_spi_buf_set);
	if (err < 0) {
		printk("spi_write_dt() failed, err %d", err);
		return err;
	}
	printk("requested Virtual Sensors Present list.\n");

	// wait for it to be populated
	k_msleep(100);

	// get the response on channel 3 (0x03)
	// the way it's structured is:
	// 2 bytes parameter ID
	// 2 bytes length
	// and depending on the length, read the rest
	// so the plan is to only read the first 4 bytes and then decide,
	// based on ID and length what to do next
	int size = 4 + N_READ_DUMMY_BYTES;
	uint8_t parameter_response[size];
	uint8_t tx_buffer = READ_REG(0x03);
	struct spi_buf tx_spi_buf2		= {.buf = &tx_buffer, .len = 1};
	struct spi_buf_set tx_spi_buf_set2 	= {.buffers = &tx_spi_buf2, .count = 1};
	struct spi_buf rx_spi_buf2 		= {.buf = parameter_response, .len = size};
	struct spi_buf_set rx_spi_buf_set2	= {.buffers = &rx_spi_buf2, .count = 1};
	err = spi_transceive_dt(spi_spec, &tx_spi_buf_set2, &rx_spi_buf_set2);
	if (err < 0) {
		printk("spi_transceive_dt() failed, err: %d", err);
		return err;
	}
	uint16_t parameter_id = parameter_response[1] | parameter_response[2] << 8;
	uint16_t length = parameter_response[3] | parameter_response[4] << 8;

	if (parameter_id == 0x011F) {
		printk("length to read: %d bytes\n", length);
		// read length bytes, describing which "Virtual Sensor" is present
		size = length + N_READ_DUMMY_BYTES;
		uint8_t virtual_sensor_list[size];
		uint8_t tx_buffer3 = READ_REG(0x03);
		struct spi_buf tx_spi_buf3		= {.buf = &tx_buffer3, .len = 1};
		struct spi_buf_set tx_spi_buf_set3 	= {.buffers = &tx_spi_buf3, .count = 1};
		struct spi_buf rx_spi_buf3 		= {.buf = virtual_sensor_list, .len = size};
		struct spi_buf_set rx_spi_buf_set3	= {.buffers = &rx_spi_buf3, .count = 1};
		err = spi_transceive_dt(spi_spec, &tx_spi_buf_set3, &rx_spi_buf_set3);
		// virtual_sensor_list is a 256 bit map, packed in 32 bytes.
		// just check if the relevant sensor is available
		// for game rotation vector, it's ID 37 (non-wakeup fifo) and 38 (wakeup fifo)
		// bit 37 and 38 are in the 5th byte. each byte has position 0 to 7.
		// 37 and 38 are at bit 5 and 6 within the byte 5.
		uint8_t non_wakeup_game_rotation = 1 << 5;
		// uint8_t wakeup_game_rotation = 1 << 6;
		// we only care about non-wakeup fifo for now
		if (virtual_sensor_list[5] & non_wakeup_game_rotation) {
			printk("non-wakeup \"Game Rotation Vector\" supported.\n");
		}

		// for (int j=1; j<length; j++) {
		// 	printk("virtual_sensor_list[%d] = 0x%02x\n", j, virtual_sensor_list[j]);
		// 	k_msleep(10);
		// }
	}

	// for (int k = 1; k < size; k++) {
	// 	printk("parameter_response[%d] = 0x%02x\n", k, parameter_response[k]);
	// }
    return 0;
}

int read_game_rotation_virtual_sensor_information(struct spi_dt_spec *spi_spec) {
	// send "Virtual Sensor Information" for non-wakeup Game Rotation (0x0325)
	// System Parameter command to channel 0 (0x00)
	// and because we want to read, send 0x1325, and LSB = 0x25 0x13
	// total number of bytes to follow is 0, and LSB = 0x00 0x00
	uint8_t tx_buf[] = {WRITE_REG(0x00), 0x25, 0x13, 0x00, 0x00};
	struct spi_buf tx_spi_buf = 		{.buf = 	tx_buf, 		.len = 		sizeof(tx_buf)};
	struct spi_buf_set tx_spi_buf_set = {.buffers = &tx_spi_buf, 	.count = 	1};
	int err = spi_write_dt(spi_spec, &tx_spi_buf_set);
	if (err < 0) {
		printk("spi_write_dt() failed, err %d", err);
		return err;
	}
	printk("requested Virtual Sensor Information for \"Game Rotation Vector\".\n");

	// wait for it to be populated
	k_msleep(100);

	// response as defined in table 70 is returned
	// get the response on channel 3 (0x03)
	// so the plan is to only read the first 4 bytes and then decide,
	// based on ID and length what to do next
	int size = 4 + N_READ_DUMMY_BYTES;
	uint8_t parameter_response[size];
	uint8_t tx_buffer = READ_REG(0x03);
	struct spi_buf tx_spi_buf2		= {.buf = &tx_buffer, .len = 1};
	struct spi_buf_set tx_spi_buf_set2 	= {.buffers = &tx_spi_buf2, .count = 1};
	struct spi_buf rx_spi_buf2 		= {.buf = parameter_response, .len = size};
	struct spi_buf_set rx_spi_buf_set2	= {.buffers = &rx_spi_buf2, .count = 1};
	err = spi_transceive_dt(spi_spec, &tx_spi_buf_set2, &rx_spi_buf_set2);
	if (err < 0) {
		printk("spi_transceive_dt() failed, err: %d", err);
		return err;
	}
	// normally the parameter id is 0x0325 (we have requested 0x1325 for reading)
	// but to get the ID as per datasheet, we only care about the smaller byte (0x25 = 37)
	uint16_t parameter_id = parameter_response[1];
	uint16_t length = parameter_response[3] | parameter_response[4] << 8;

	printk("parameter_id = %d\n", parameter_id);
	// printk("length = %d\n", length);

	if (parameter_id == 37) {
		printk("length to read: %d bytes\n", length);
		// read length bytes, which represents the structure in table 70 in the datasheet
		size = length + N_READ_DUMMY_BYTES;
		uint8_t game_rotation_sensor_information[size];
		uint8_t tx_buffer3 = READ_REG(0x03);
		struct spi_buf tx_spi_buf3		= {.buf = &tx_buffer3, .len = 1};
		struct spi_buf_set tx_spi_buf_set3 	= {.buffers = &tx_spi_buf3, .count = 1};
		struct spi_buf rx_spi_buf3 		= {.buf = game_rotation_sensor_information, .len = size};
		struct spi_buf_set rx_spi_buf_set3	= {.buffers = &rx_spi_buf3, .count = 1};
		err = spi_transceive_dt(spi_spec, &tx_spi_buf_set3, &rx_spi_buf_set3);

		// for (int j=1; j<length; j++) {
		// 	printk("game_rotation_sensor_information[%d] = 0x%02x\n", j, game_rotation_sensor_information[j]);
		// 	k_msleep(10);
		// }
	}
    return 0;
}

int read_game_rotation_virtual_sensor_configuration(struct spi_dt_spec *spi_spec) {
	// send "Virtual Sensor Configuration" for non-wakeup Game Rotation (0x0525)
	// System Parameter command to channel 0 (0x00)
	// and because we want to read, send 0x1525, and LSB = 0x25 0x15
	// total number of bytes to follow is 0, and LSB = 0x00 0x00
	uint8_t tx_buf[] = {WRITE_REG(0x00), 0x25, 0x15, 0x00, 0x00};
	struct spi_buf tx_spi_buf = 		{.buf = 	tx_buf, 		.len = 		sizeof(tx_buf)};
	struct spi_buf_set tx_spi_buf_set = {.buffers = &tx_spi_buf, 	.count = 	1};
	int err = spi_write_dt(spi_spec, &tx_spi_buf_set);
	if (err < 0) {
		printk("spi_write_dt() failed, err %d", err);
		return err;
	}
	printk("requested Virtual Sensor Configuration for \"Game Rotation Vector\".\n");

	// wait for it to be populated
	k_msleep(100);

	// response as defined in table 71 is returned
	// get the response on channel 3 (0x03)
	// so the plan is to only read the first 4 bytes and then decide,
	// based on ID and length what to do next
	int size = 4 + N_READ_DUMMY_BYTES;
	uint8_t parameter_response[size];
	uint8_t tx_buffer = READ_REG(0x03);
	struct spi_buf tx_spi_buf2		= {.buf = &tx_buffer, .len = 1};
	struct spi_buf_set tx_spi_buf_set2 	= {.buffers = &tx_spi_buf2, .count = 1};
	struct spi_buf rx_spi_buf2 		= {.buf = parameter_response, .len = size};
	struct spi_buf_set rx_spi_buf_set2	= {.buffers = &rx_spi_buf2, .count = 1};
	err = spi_transceive_dt(spi_spec, &tx_spi_buf_set2, &rx_spi_buf_set2);
	if (err < 0) {
		printk("spi_transceive_dt() failed, err: %d", err);
		return err;
	}
	// normally the parameter id is 0x0525 (we have requested 0x1525 for reading)
	// but to get the ID as per datasheet, we only care about the smaller byte (0x25 = 37)
	uint16_t parameter_id = parameter_response[1];
	uint16_t length = parameter_response[3] | parameter_response[4] << 8;

	printk("parameter_id = %d\n", parameter_id);
	// printk("length = %d\n", length);

	if (parameter_id == 37) {
		printk("length to read: %d bytes\n", length);
		// read length bytes, which represents the structure in table 70 in the datasheet
		size = length + N_READ_DUMMY_BYTES;
		uint8_t game_rotation_sensor_configuration[size];
		uint8_t tx_buffer3 = READ_REG(0x03);
		struct spi_buf tx_spi_buf3		= {.buf = &tx_buffer3, .len = 1};
		struct spi_buf_set tx_spi_buf_set3 	= {.buffers = &tx_spi_buf3, .count = 1};
		struct spi_buf rx_spi_buf3 		= {.buf = game_rotation_sensor_configuration, .len = size};
		struct spi_buf_set rx_spi_buf_set3	= {.buffers = &rx_spi_buf3, .count = 1};
		err = spi_transceive_dt(spi_spec, &tx_spi_buf_set3, &rx_spi_buf_set3);

		for (int j=1; j<length; j++) {
			printk("game_rotation_sensor_configuration[%d] = 0x%02x\n", j, game_rotation_sensor_configuration[j]);
			k_msleep(10);
		}
	}
    return 0;
}

int configure_game_rotation_sensor(struct spi_dt_spec *spi_spec) {
	// command is "Configure Sensor" = 0x000D, and LSB = 0x0D, 0x00
	// length is 8 bytes = 0x0008, and LSB = 0x08, 0x00
	// sensor ID is 37 = 0x25, and LSB = 0x25
	// sample rate (in float) = 400Hz = 0x43C80000, and LSB = 0x00, 0x00, 0xC8, 0x42
	// latency = 0ms = 0x000000, and LSB = 0x00, 0x00, 0x00
	uint8_t tx_buf[] = {WRITE_REG(0x00), 0x0D, 0x00, 0x08, 0x00, 0x25, 0x00, 0x00, 0xC8, 0x43, 0x00, 0x00, 0x00};
	struct spi_buf tx_spi_buf = 		{.buf = 	tx_buf, 		.len = 		sizeof(tx_buf)};
	struct spi_buf_set tx_spi_buf_set = {.buffers = &tx_spi_buf, 	.count = 	1};
	int err = spi_write_dt(spi_spec, &tx_spi_buf_set);
	if (err < 0) {
		printk("spi_write_dt() failed, err %d", err);
		return err;
	}
	printk("configured game rotation sensor with 400Hz ODR.\n");
    return 0;
}

int configure_linear_acceleration_sensor(struct spi_dt_spec *spi_spec) {
	// command is "Configure Sensor" = 0x000D, and LSB = 0x0D, 0x00
	// length is 8 bytes = 0x0008, and LSB = 0x08, 0x00
	// sensor ID is 32 = 0x1F, and LSB = 0x1F
	// sample rate (in float) = 400Hz = 0x43C80000, and LSB = 0x00, 0x00, 0xC8, 0x42
	// latency = 0ms = 0x000000, and LSB = 0x00, 0x00, 0x00
	uint8_t tx_buf[] = {WRITE_REG(0x00), 0x0D, 0x00, 0x08, 0x00, 0x1F, 0x00, 0x00, 0xC8, 0x43, 0x00, 0x00, 0x00};
	struct spi_buf tx_spi_buf = 		{.buf = 	tx_buf, 		.len = 		sizeof(tx_buf)};
	struct spi_buf_set tx_spi_buf_set = {.buffers = &tx_spi_buf, 	.count = 	1};
	int err = spi_write_dt(spi_spec, &tx_spi_buf_set);
	if (err < 0) {
		printk("spi_write_dt() failed, err %d", err);
		return err;
	}
	printk("configured linear acceleration sensor with 400Hz ODR.\n");
    return 0;
}

// static int rofl = 0;

int read_non_wakeup_fifo(struct spi_dt_spec *spi_spec, struct quaternion *quat, struct acc_vector *vec) {
	int size = 50 + N_READ_DUMMY_BYTES;
	uint8_t non_wakeup_fifo[size];
	uint8_t tx_buffer = READ_REG(0x02);
	struct spi_buf tx_spi_buf		= {.buf = &tx_buffer, .len = 1};
	struct spi_buf_set tx_spi_buf_set 	= {.buffers = &tx_spi_buf, .count = 1};
	struct spi_buf rx_spi_buf 		= {.buf = non_wakeup_fifo, .len = size};
	struct spi_buf_set rx_spi_buf_set	= {.buffers = &rx_spi_buf, .count = 1};
	spi_transceive_dt(spi_spec, &tx_spi_buf_set, &rx_spi_buf_set);
	uint16_t nbytes_in_non_wakeup_fifo = non_wakeup_fifo[1] | non_wakeup_fifo[2] << 8;
	// printk("bytes in non wakeup fifo: %d\n", nbytes_in_non_wakeup_fifo);

	// I think the length is wrongly reported. it should be 33 bytes, but length is 30 bytes.
	// ignore all packets that are not 30 in length
	// we could handle them properly, but there is no reason to
	// 33 bytes is a healthy packet.
	// 1 byte dummy
	// 2 bytes transfer length
	// 2 bytes small delta timestamp
	// 4 bytes spacer meta event
	// 6 bytes full timestamp
	// 11 bytes quaternion+ format (game rotation sensor)
	// 7 bytes 3D vector formart (linear acceleration)
	// if (rofl++%1 == 0) {
	// 	printk("pkt: %d\n", nbytes_in_non_wakeup_fifo);
	// }
	if (nbytes_in_non_wakeup_fifo+3 == 33) {
		// printk("%d\n", rofl++%1000);
		quat->x = non_wakeup_fifo[16] | non_wakeup_fifo[17] << 8;
		quat->y = non_wakeup_fifo[18] | non_wakeup_fifo[19] << 8;
		quat->z = non_wakeup_fifo[20] | non_wakeup_fifo[21] << 8;
		quat->w = non_wakeup_fifo[22] | non_wakeup_fifo[23] << 8;

		vec->x = non_wakeup_fifo[27] | non_wakeup_fifo[28] << 8;
		vec->y = non_wakeup_fifo[29] | non_wakeup_fifo[30] << 8;
		vec->z = non_wakeup_fifo[31] | non_wakeup_fifo[32] << 8;
		return 0;
		// if (rofl++%1000 == 0) {
		// 	for (int h=0; h<nbytes_in_non_wakeup_fifo+3; h++) {
		// 		printk("non_wakeup_fifo[%d] = 0x%02x\n", h, non_wakeup_fifo[h]);
		// 		k_msleep(2);
		// 	}
		// }
	}
	return -1;
}
