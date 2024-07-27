#define FLASH_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(nordic_qspi_nor)

#define HOST_INTERFACE_READY_BIT 1 << 4
#define FIRMWARE_VERIFY_DONE_BIT 1 << 5
#define FIRMWARE_VERIFY_FAILED_BIT 1 << 6
#define N_READ_DUMMY_BYTES 1

#define READ_REG(reg) (reg | 1 << 7)
#define WRITE_REG(reg) (reg)

struct quaternion {
	uint16_t x;
	uint16_t y;
	uint16_t z;
	uint16_t w;
};

struct acc_vector {
	uint16_t x;
	uint16_t y;
	uint16_t z;
};

int reset_imu(struct spi_dt_spec *);
int mask_all_interrupts(struct spi_dt_spec *);
int read_interrupt_register(struct spi_dt_spec *);
int wait_for_interface_ready(struct spi_dt_spec *);
int read_fuser2_id(struct spi_dt_spec *);
int read_fuser2_revision(struct spi_dt_spec *);
int read_fuser2_rom_version(struct spi_dt_spec *);
int write_firmware(struct spi_dt_spec *);
int wait_for_firmware_verified(struct spi_dt_spec *);
int boot_bhi360_program_ram(struct spi_dt_spec *);
int read_wakeup_fifo_initialized(struct spi_dt_spec *);
int read_non_wakeup_fifo_initialized(struct spi_dt_spec *);
int read_interrupt_status_register(struct spi_dt_spec *);
int read_sensor_present(struct spi_dt_spec *);
int read_game_rotation_virtual_sensor_information(struct spi_dt_spec *);
int read_game_rotation_virtual_sensor_configuration(struct spi_dt_spec *);
int configure_game_rotation_sensor(struct spi_dt_spec *);
int configure_linear_acceleration_sensor(struct spi_dt_spec *);
int read_non_wakeup_fifo(struct spi_dt_spec *, struct quaternion *, struct acc_vector *);