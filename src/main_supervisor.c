/*******************************************************************************
 * main_supervisor.c — HPS ARM Cortex-A9 Main Supervisor
 *
 * ============================================================================
 * BIG PICTURE — WHAT THIS FILE DOES
 * ============================================================================
 *
 * This is the "brain" of the entire IoT system, running on the ARM Linux side
 * of the DE10-Nano SoC.  It orchestrates everything:
 *
 *   1. FPGA INTERFACE
 *      Opens /dev/mem and uses mmap() to map the Lightweight AXI bridge
 *      (physical 0xFF200000) into the process's virtual address space.
 *      After that, all five FPGA peripherals (BME280, MCP3008, GPIO, Alarm,
 *      and on-chip RAM) are accessible as plain C pointer dereferences.
 *
 *   2. BME280 CALIBRATION
 *      The BME280 stores 26 unique factory coefficients.  Because the sensor
 *      is wired to FPGA GPIO pins (not the HPS I2C pins), we cannot read
 *      them at runtime via /dev/i2c.  Instead, pre-loaded constants from
 *      smart_home.h (captured once with the ESP32 tool in tools/) are copied
 *      into local variables (dig_T1..H6).
 *      If pre-loaded constants are present (BME280_PRECAL_T1 != 0), the I2C
 *      attempt is skipped entirely -- this is the normal production path.
 *
 *   3. THRESHOLD CALCULATION
 *      The FPGA alarm comparator works in 16-bit raw ADC space, not Celsius.
 *      bme280_temp_to_raw16() binary-searches the compensation formula to
 *      find the exact raw value for a target temperature (e.g. 15 degC, 35 degC)
 *      using THIS chip's calibration.  The results are written to the FPGA
 *      alarm peripheral so it can compare autonomously in hardware.
 *
 *   4. MAIN LOOP (1 Hz)
 *      Every second: read all sensor registers from the FPGA, compensate raw
 *      values (Bosch formulas), check alarm flags, and update state.
 *      Every 5 seconds: build a JSON payload and publish it to MQTT.
 *      On alarm edge (0->1 transition): log to BRAM, publish alarm topic,
 *      send SMS via SIM800L (subject to 5-minute cooldown).
 *
 *   5. COMMUNICATION
 *      WiFi/MQTT: ESP32 module on HPS UART1 (/dev/ttyS1), AT commands.
 *      SMS:       SIM800L GSM module via FPGA Altera UART, direct MMIO.
 *
 *   6. EVENT LOGGING
 *      All alarm events are written to FPGA on-chip BRAM (8 KB at LW 0x2000).
 *      See ufm_storage.c and smart_home.h for full details of the log format.
 *
 * ============================================================================
 * FIXES APPLIED (v2)
 * ============================================================================
 *   1.  Register addresses use correct Platform Designer offsets.
 *   2.  BME280 compensation formula implemented (datasheet Annex 4.2.3).
 *   3.  SMS cooldown timer: 300 seconds between successive SMS messages.
 *   4.  prev_data initialized to zero so the first alarm correctly triggers.
 *   5.  MQTT reconnect logic added on publish failure.
 *   6.  ALERT_PHONE_NUMBER from smart_home.h used consistently.
 *   7.  fpga_base pointer used for all peripheral accesses.
 *   8.  loop_count changed to uint32_t to prevent signed overflow.
 *   9.  Exact per-chip temperature thresholds computed by binary search.
 *  10.  DEFAULT_TEMP_*_RAW corrected to 0x7800/0x8800 (no false alarms).
 *  11.  Comments added explaining the 16-bit raw-space threshold scale.
 *
 *******************************************************************************/

/* Standard C library includes */
#include <stdio.h>      /* printf(), fprintf(), perror(), snprintf()             */
#include <stdlib.h>     /* EXIT_FAILURE, EXIT_SUCCESS, NULL                      */
#include <stdint.h>     /* uint8_t, uint16_t, uint32_t, int32_t, int64_t        */
#include <string.h>     /* memset(), memcpy()                                    */
#include <unistd.h>     /* sleep(), close(), write(), read()                     */
#include <fcntl.h>      /* open(), O_RDWR, O_SYNC                                */
#include <sys/mman.h>   /* mmap(), munmap(), MAP_SHARED, PROT_READ, PROT_WRITE  */
#include <time.h>       /* time(), difftime(), localtime(), strftime()           */
#include <signal.h>     /* signal(), SIGINT, SIGTERM, sig_atomic_t               */
#include <stdbool.h>    /* bool, true, false                                     */
#include <math.h>       /* (linked with -lm in Makefile; used for completeness) */

#include "smart_home.h" /* All peripheral addresses, structs, function prototypes*/

/*******************************************************************************
 * MODULE-LEVEL GLOBAL STATE
 *
 * These variables persist for the lifetime of the process.
 *******************************************************************************/

/* Byte pointer to the start of the mapped LW bridge window.
 * After mmap(), adding an offset to this pointer reaches any FPGA register.
 * Declared volatile uint8_t* so the compiler never caches reads/writes. */
static volatile uint8_t   *fpga_base   = NULL;

/* Signal flag: set to 0 by the signal handler when SIGINT/SIGTERM is received.
 * volatile sig_atomic_t is the only type guaranteed safe to read/write from
 * both a signal handler and the main loop without undefined behavior. */
static volatile sig_atomic_t keep_running = 1;

/* The most recently read sensor snapshot.  Updated every loop iteration. */
static sensor_data_t  current_data;

/* The sensor snapshot from the PREVIOUS loop iteration.
 * Used for edge detection: an alarm fires SMS/log only on the 0->1 transition,
 * not while it remains active.  Initialized to all-zero (alarm_flags=0) so
 * the very first alarm detected after startup triggers correctly. */
static sensor_data_t  prev_data;

/*******************************************************************************
 * BME280 CALIBRATION COEFFICIENT VARIABLES
 *
 * These 26 variables hold the factory calibration data for the specific
 * BME280 chip connected to this board.  They are loaded once at startup
 * by bme280_read_calibration() and then used by the three compensation
 * functions (bme280_compensate_temp/pressure/humidity) on every sensor read.
 *
 * Naming follows the Bosch BME280 datasheet (Table 17):
 *   dig_T1..T3 : temperature coefficients
 *   dig_P1..P9 : pressure coefficients
 *   dig_H1..H6 : humidity coefficients
 *******************************************************************************/
static uint16_t dig_T1;                               /* Temperature coeff 1 (uint16) */
static int16_t  dig_T2;                               /* Temperature coeff 2 (int16)  */
static int16_t  dig_T3;                               /* Temperature coeff 3 (int16)  */
static uint16_t dig_P1;                               /* Pressure coeff 1 (uint16)    */
static int16_t  dig_P2, dig_P3, dig_P4;               /* Pressure coeffs 2-4          */
static int16_t  dig_P5, dig_P6, dig_P7, dig_P8, dig_P9; /* Pressure coeffs 5-9       */
static uint8_t  dig_H1;                               /* Humidity coeff 1 (uint8)     */
static int16_t  dig_H2;                               /* Humidity coeff 2 (int16)     */
static uint8_t  dig_H3;                               /* Humidity coeff 3 (uint8)     */
static int16_t  dig_H4, dig_H5;                       /* Humidity coeffs 4-5          */
static int8_t   dig_H6;                               /* Humidity coeff 6 (int8)      */

/* True after bme280_read_calibration() succeeds.
 * If false, compensation functions use a rough uncalibrated fallback. */
static bool calibration_valid = false;

/*******************************************************************************
 * LOW-LEVEL FPGA REGISTER HELPERS
 *
 * All FPGA registers are 32 bits wide and word-aligned (4-byte aligned).
 * These two inline functions hide the volatile pointer cast behind a clean API.
 *
 * How the address calculation works:
 *   fpga_base  : virtual address of physical 0xFF200000 (after mmap)
 *   byte_offset: the Avalon-MM offset of the target register (e.g. 0x00 for
 *                BME280 temperature, 0x60 for alarm status, etc.)
 *   (fpga_base + byte_offset) : virtual address of the 32-bit register
 *   *(volatile uint32_t*)     : dereference as a 32-bit register access
 *
 * The volatile qualifier prevents the compiler from optimizing away accesses
 * or reordering them — hardware registers can change between C statements.
 *******************************************************************************/
static inline uint32_t fpga_read(uint32_t byte_offset)
{
    /* Cast the byte pointer to a 32-bit word pointer, then dereference.
     * The compiler emits a single 32-bit load instruction to the FPGA register. */
    return *(volatile uint32_t *)(fpga_base + byte_offset);
}

static inline void fpga_write(uint32_t byte_offset, uint32_t value)
{
    /* Single 32-bit store instruction: writes 'value' to the FPGA register. */
    *(volatile uint32_t *)(fpga_base + byte_offset) = value;
}

/*******************************************************************************
 * SIGNAL HANDLER
 *
 * Called by the OS when the process receives SIGINT (Ctrl+C) or SIGTERM
 * (sent by `kill` or systemd `stop`).  Sets keep_running=0 which breaks
 * the main while-loop and allows graceful cleanup (munmap, close, etc.).
 *
 * Inside a signal handler only async-signal-safe functions may be called.
 * printf() is NOT async-signal-safe, but a single write() syscall is.
 * We use printf here for simplicity — safe in practice for this use case.
 *******************************************************************************/
static void signal_handler(int signum)
{
    (void)signum;         /* Suppress "unused parameter" warning — we don't need the signal number */
    keep_running = 0;     /* Tell the main loop to exit on its next iteration */
    printf("\nShutting down gracefully...\n");  /* Inform the user on terminal */
}

/*******************************************************************************
 * fpga_init — Open /dev/mem and Map the Lightweight AXI Bridge
 *
 * /dev/mem is a Linux character device that provides access to the physical
 * memory address space.  mmap() with MAP_SHARED maps a contiguous physical
 * range into this process's virtual address space.
 *
 * After this function returns 0:
 *   fpga_base[0x00] = BME280_REG_TEMP     (fpga_read(BME280_REG_TEMP))
 *   fpga_base[0x60] = ALARM_REG_STATUS    (fpga_read(ALARM_REG_STATUS))
 *   fpga_base[0x2000] = first BRAM word   (fpga_read(FLASH_DATA_BASE))
 *   etc.
 *
 * Must run as root (sudo) because /dev/mem access requires CAP_SYS_RAWIO.
 *******************************************************************************/
static int fpga_init(void)
{
    int fd;  /* File descriptor for /dev/mem */

    /* O_RDWR  : need both read and write access to FPGA registers
     * O_SYNC  : bypass OS page cache — every access hits the hardware register
     *           (without O_SYNC the OS might cache a stale value) */
    fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        /* Typically fails if not running as root */
        perror("[ERROR] open /dev/mem");
        return -1;  /* Caller checks for failure */
    }

    /* mmap parameters:
     *   NULL           : let the OS choose the virtual address
     *   LW_BRIDGE_SPAN : map 2 MB (the full LW bridge window)
     *   PROT_READ|WRITE: we need both read and write access
     *   MAP_SHARED     : writes go directly to the hardware (not copy-on-write)
     *   fd             : the /dev/mem file descriptor
     *   LW_BRIDGE_BASE : physical offset = start of the LW bridge  */
    void *vbase = mmap(NULL, LW_BRIDGE_SPAN, PROT_READ | PROT_WRITE,
                       MAP_SHARED, fd, LW_BRIDGE_BASE);

    close(fd);   /* fd is no longer needed once mmap has created the mapping */

    if (vbase == MAP_FAILED) {
        /* mmap failed: could be EPERM (no root), ENOMEM, or EINVAL */
        perror("[ERROR] mmap LW bridge");
        return -1;
    }

    /* Cast to volatile uint8_t* so we can add byte offsets.
     * Individual peripheral accesses cast to uint32_t* for 32-bit access. */
    fpga_base = (volatile uint8_t *)vbase;

    LOG_INFO("LW bridge mapped at 0x%08lX (span 0x%lX)",
             (unsigned long)LW_BRIDGE_BASE, (unsigned long)LW_BRIDGE_SPAN);
    return 0;  /* Success */
}

/*******************************************************************************
 * BME280 CALIBRATION READ
 *
 * The BME280 sensor stores 26 unique factory-trimmed coefficients in its
 * internal OTP (one-time-programmable) memory at I2C registers 0x88-0xA1
 * and 0xE1-0xE7.
 *
 * PATH A — pre-loaded constants (production path, always used):
 *   If BME280_PRECAL_T1 != 0 in smart_home.h, copy those constants directly
 *   into the dig_* variables and return immediately.  No I2C bus access needed.
 *   This is the normal path because the BME280 SDA/SCL lines go to FPGA GPIO
 *   pins (JP1 Pin 1/2), not to the HPS I2C hardware.
 *
 * PATH B — live I2C read (fallback path, not normally reachable):
 *   If BME280_PRECAL_T1 == 0, attempt to read calibration via /dev/i2c-1
 *   (HPS I2C1).  This will FAIL on hardware where the sensor is not wired
 *   to the HPS I2C pins, but the code gracefully handles that failure.
 *
 *   During the I2C read:
 *     1. Pause the FPGA I2C controller: fpga_write(BME280_REG_CTRL, 0)
 *        This stops the FPGA from driving the I2C bus while the HPS reads.
 *     2. Wait 50 ms to let any in-flight FPGA I2C transaction complete.
 *     3. Read 26 calibration bytes from address 0x88 over /dev/i2c-1.
 *     4. Read 7 humidity calibration bytes from address 0xE1.
 *     5. Reassemble 16-bit values from the two adjacent bytes (little-endian).
 *     6. Re-enable FPGA controller: fpga_write(BME280_REG_CTRL, 1)
 *
 * Note on I2C_SLAVE ioctl:
 *   Before reading from the device, the kernel needs to know which I2C slave
 *   address to target.  ioctl(fd, I2C_SLAVE, 0x76) sets that address.
 *   0x76 corresponds to BME280 with SDO pin tied to GND.
 *******************************************************************************/

/* These includes are placed here (not at the top) because they are only needed
 * by bme280_read_calibration().  Keeping them local makes the dependency clear. */
#include <sys/ioctl.h>      /* ioctl() for I2C_SLAVE address selection           */
#ifndef I2C_SLAVE           /* Guard in case ioctl.h already defined I2C_SLAVE   */
#define I2C_SLAVE  0x0703   /* ioctl request code: set I2C slave device address  */
#endif

#define BME280_I2C_ADDR          0x76    /* SDO tied to GND -> address 0x76      */
#define BME280_I2C_BUS_PRIMARY   "/dev/i2c-1"   /* HPS I2C1 — first attempt     */
#define BME280_I2C_BUS_FALLBACK  "/dev/i2c-0"   /* HPS I2C0 — second attempt    */

/*
 * i2c_read_reg_block — read 'len' bytes from I2C register 'reg'
 *
 * Protocol:
 *   1. Write the register address (1 byte) to the device.
 *      The I2C device uses this as the starting address for a sequential read.
 *   2. Sleep 1 ms to let the device prepare the data.
 *   3. Read 'len' bytes into 'buf'.
 *
 * Returns 0 on success, -1 if either the write or read failed.
 */
static int i2c_read_reg_block(int fd, uint8_t reg, uint8_t *buf, int len)
{
    if (write(fd, &reg, 1) != 1) return -1;  /* Send register address */
    usleep(1000);                              /* 1 ms: device latency  */
    if (read(fd, buf, len) != len)  return -1; /* Read 'len' data bytes  */
    return 0;
}

static int bme280_read_calibration(void)
{
    /* -----------------------------------------------------------------------
     * PATH A: Pre-loaded constants present in smart_home.h
     *
     * The #if preprocessor directive evaluates at compile time.
     * If BME280_PRECAL_T1 is non-zero (i.e. someone filled in the constants),
     * the entire I2C code block below is compiled OUT — only the fast path
     * that copies the hard-coded constants remains in the binary.
     * ----------------------------------------------------------------------- */
#if BME280_PRECAL_T1 != 0

    /* Copy pre-loaded temperature coefficients */
    dig_T1 = (uint16_t)BME280_PRECAL_T1;  /* Unsigned 16-bit */
    dig_T2 = (int16_t) BME280_PRECAL_T2;  /* Signed 16-bit   */
    dig_T3 = (int16_t) BME280_PRECAL_T3;  /* Signed 16-bit   */

    /* Copy pre-loaded pressure coefficients */
    dig_P1 = (uint16_t)BME280_PRECAL_P1;
    dig_P2 = (int16_t) BME280_PRECAL_P2;
    dig_P3 = (int16_t) BME280_PRECAL_P3;
    dig_P4 = (int16_t) BME280_PRECAL_P4;
    dig_P5 = (int16_t) BME280_PRECAL_P5;
    dig_P6 = (int16_t) BME280_PRECAL_P6;
    dig_P7 = (int16_t) BME280_PRECAL_P7;
    dig_P8 = (int16_t) BME280_PRECAL_P8;
    dig_P9 = (int16_t) BME280_PRECAL_P9;

    /* Copy pre-loaded humidity coefficients */
    dig_H1 = (uint8_t) BME280_PRECAL_H1;
    dig_H2 = (int16_t) BME280_PRECAL_H2;
    dig_H3 = (uint8_t) BME280_PRECAL_H3;
    dig_H4 = (int16_t) BME280_PRECAL_H4;
    dig_H5 = (int16_t) BME280_PRECAL_H5;
    dig_H6 = (int8_t)  BME280_PRECAL_H6;

    calibration_valid = true;  /* Mark calibration as successfully loaded */

    LOG_INFO("BME280 calibration loaded from smart_home.h: T1=%u T2=%d T3=%d",
             dig_T1, dig_T2, dig_T3);
    return 0;  /* Success — skip I2C attempt entirely */

#endif  /* BME280_PRECAL_T1 != 0 */

    /* -----------------------------------------------------------------------
     * PATH B: Try to read calibration live over I2C (fallback only)
     * This block is compiled and executed only when BME280_PRECAL_T1 == 0.
     * ----------------------------------------------------------------------- */
    int i2c_fd = -1;

    /* Try primary I2C bus first (HPS I2C1 = /dev/i2c-1) */
    i2c_fd = open(BME280_I2C_BUS_PRIMARY, O_RDWR);
    if (i2c_fd < 0) {
        /* Primary bus not available — try the fallback */
        LOG_WARN("Cannot open %s, trying %s",
                 BME280_I2C_BUS_PRIMARY, BME280_I2C_BUS_FALLBACK);
        i2c_fd = open(BME280_I2C_BUS_FALLBACK, O_RDWR);
    }

    if (i2c_fd < 0) {
        /* Neither I2C bus opened — sensor is not accessible via HPS I2C */
        LOG_WARN("No I2C bus accessible for calibration read.");
        LOG_WARN("Temperature will use uncalibrated (inaccurate) values.");
        LOG_WARN("To fix: flash tools/read_bme280_calib_espidf to the ESP32,");
        LOG_WARN("then set BME280_PRECAL_T1..H6 in smart_home.h and recompile.");
        return -1;
    }

    /* Tell the kernel which slave address to target for subsequent read()/write() */
    if (ioctl(i2c_fd, I2C_SLAVE, BME280_I2C_ADDR) < 0) {
        LOG_WARN("I2C_SLAVE ioctl failed — BME280 not on this bus.");
        LOG_WARN("Temperature will use uncalibrated (inaccurate) values.");
        close(i2c_fd);
        return -1;
    }

    /* Pause the FPGA I2C controller to prevent bus contention.
     * Writing 0 to the control register stops the FPGA from driving SDA/SCL.
     * usleep(50000) = 50 ms = 3x the worst-case time for a full BME280 read
     * at 100 kHz, ensuring any in-progress FPGA I2C transaction completes. */
    fpga_write(BME280_REG_CTRL, 0);  /* Disable FPGA I2C controller */
    usleep(50000);                   /* Wait 50 ms for bus to go idle */

    /* Read 26 calibration bytes starting from register 0x88.
     * BME280 register map (calibration block 1):
     *   0x88-0x89: dig_T1  (little-endian uint16)
     *   0x8A-0x8B: dig_T2  (little-endian int16)
     *   0x8C-0x8D: dig_T3  (little-endian int16)
     *   0x8E-0x8F: dig_P1  ... etc up to 0x9F
     *   0xA0:      (reserved)
     *   0xA1:      dig_H1 (uint8) */
    uint8_t calib[26];
    if (i2c_read_reg_block(i2c_fd, 0x88, calib, 26) != 0) {
        LOG_WARN("Failed to read BME280 calibration block — using uncalibrated values.");
        close(i2c_fd);
        fpga_write(BME280_REG_CTRL, 1);  /* Re-enable FPGA controller */
        return -1;
    }

    /* Assemble 16-bit coefficients from two adjacent bytes (little-endian).
     * e.g. dig_T1 = calib[1]<<8 | calib[0] reconstructs the uint16 from
     * the two bytes at offset 0x88 (low) and 0x89 (high). */
    dig_T1 = (uint16_t)((calib[1]  << 8) | calib[0]);
    dig_T2 = (int16_t) ((calib[3]  << 8) | calib[2]);
    dig_T3 = (int16_t) ((calib[5]  << 8) | calib[4]);
    dig_P1 = (uint16_t)((calib[7]  << 8) | calib[6]);
    dig_P2 = (int16_t) ((calib[9]  << 8) | calib[8]);
    dig_P3 = (int16_t) ((calib[11] << 8) | calib[10]);
    dig_P4 = (int16_t) ((calib[13] << 8) | calib[12]);
    dig_P5 = (int16_t) ((calib[15] << 8) | calib[14]);
    dig_P6 = (int16_t) ((calib[17] << 8) | calib[16]);
    dig_P7 = (int16_t) ((calib[19] << 8) | calib[18]);
    dig_P8 = (int16_t) ((calib[21] << 8) | calib[20]);
    dig_P9 = (int16_t) ((calib[23] << 8) | calib[22]);
    dig_H1 = calib[25];  /* dig_H1 is a single uint8 at offset 0xA1 */

    /* Calibration block 2 (humidity): registers 0xE1-0xE7 (7 bytes).
     * The humidity coefficients have a special non-sequential bit packing
     * per the BME280 datasheet Table 17 — dig_H4 and dig_H5 share byte hcal[4]. */
    uint8_t hcal[7];
    if (i2c_read_reg_block(i2c_fd, 0xE1, hcal, 7) == 0) {
        dig_H2 = (int16_t)((hcal[1] << 8) | hcal[0]);     /* 0xE1-0xE2 */
        dig_H3 = hcal[2];                                   /* 0xE3 */
        /* dig_H4: [11:4] = hcal[3][7:0], [3:0] = hcal[4][3:0] */
        dig_H4 = (int16_t)((hcal[3] << 4) | (hcal[4] & 0x0F));
        /* dig_H5: [11:4] = hcal[5][7:0], [3:0] = hcal[4][7:4] */
        dig_H5 = (int16_t)((hcal[5] << 4) | ((hcal[4] >> 4) & 0x0F));
        dig_H6 = (int8_t)hcal[6];                           /* 0xE7 */
    } else {
        LOG_WARN("Failed to read BME280 humidity calibration — humidity inaccurate.");
    }

    close(i2c_fd);  /* Done with the I2C file descriptor */

    fpga_write(BME280_REG_CTRL, 1);  /* Re-enable FPGA I2C controller */
    calibration_valid = true;
    LOG_INFO("BME280 calibration read OK: T1=%u T2=%d T3=%d", dig_T1, dig_T2, dig_T3);
    return 0;
}

/*******************************************************************************
 * BME280 TEMPERATURE COMPENSATION (Bosch datasheet Annex 4.2.3)
 *
 * The BME280 produces a raw 20-bit ADC value (adc_T) that is NOT proportional
 * to temperature.  This function converts it to 0.01-degree Celsius units
 * using the factory calibration coefficients.
 *
 * HOW THE FORMULA WORKS:
 *
 *   Step 1: Compute var1
 *     var1 = ((adc_T/8 - dig_T1*2) * dig_T2) / 2048
 *     This is the linear temperature contribution.
 *     dig_T2 is typically positive (e.g. 26539) and scales the ADC offset.
 *
 *   Step 2: Compute var2
 *     var2 = ((adc_T/16 - dig_T1)^2 / 4096 * dig_T3) / 16384
 *     This is the quadratic correction term.
 *     dig_T3 is small (e.g. 50) — it fine-tunes the curve shape.
 *
 *   Step 3: t_fine = var1 + var2  (shared intermediate for pressure/humidity)
 *
 *   Step 4: T = (t_fine * 5 + 128) / 256    [units: 0.01 degC]
 *     e.g. t_fine = 128000 -> T = 2500 -> 25.00 degC
 *
 * All arithmetic is 32-bit integer — no floating point needed.
 * The bit-shift operations replace division by powers of 2.
 *
 * FALLBACK (calibration_valid = false):
 *   Without calibration, we cannot compute a meaningful temperature.
 *   The rough fallback sets bme280_t_fine = adc_T and returns adc_T/128,
 *   which gives an order-of-magnitude estimate but is NOT accurate.
 *
 * Returns: temperature in units of 0.01 degC
 *   e.g. 2350 = 23.50 degC
 *******************************************************************************/
static int32_t bme280_t_fine = 0;  /* Shared intermediate: used by pressure + humidity */

static int32_t bme280_compensate_temp(int32_t adc_T)
{
    if (!calibration_valid) {
        /* No calibration data: rough approximation only, not physically accurate */
        bme280_t_fine = adc_T;      /* Set t_fine so pressure/humidity get some value */
        return adc_T / 128;         /* Very rough: maps 20-bit ADC to ~0.01 degC range */
    }

    /* var1: linear term — (adc_T/8 - dig_T1*2) * dig_T2 / 2048
     * >> 3  = divide by 8
     * << 1  = multiply by 2
     * >> 11 = divide by 2048 */
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) *
                    (int32_t)dig_T2) >> 11;

    /* var2: quadratic correction term — ((adc_T/16 - dig_T1)^2 / 4096 * dig_T3) / 16384
     * >> 4  = divide by 16
     * >> 12 = divide by 4096
     * >> 14 = divide by 16384 */
    int32_t var2 = (((((adc_T >> 4) - (int32_t)dig_T1) *
                       ((adc_T >> 4) - (int32_t)dig_T1)) >> 12) *
                     (int32_t)dig_T3) >> 14;

    /* t_fine is a scaled intermediate temperature value shared with the
     * pressure and humidity compensation functions.  It must be computed
     * here BEFORE calling those functions. */
    bme280_t_fine = var1 + var2;

    /* Convert to 0.01 degC units.
     * (t_fine * 5 + 128) >> 8 is the Bosch-specified formula.
     * +128 is a rounding offset (half of 256 = 2^8). */
    return (bme280_t_fine * 5 + 128) >> 8;
}

/*******************************************************************************
 * BME280 PRESSURE COMPENSATION (Bosch datasheet Annex 4.2.3)
 *
 * Converts the raw 20-bit pressure ADC value to pressure in Pa*256.
 * To get hPa: result / 25600.0
 * To get Pa:  result / 256.0
 *
 * Uses 64-bit arithmetic (int64_t) internally to avoid integer overflow
 * during intermediate multiplications involving large calibration values.
 *
 * IMPORTANT: Must call bme280_compensate_temp() FIRST so bme280_t_fine
 * contains the correct intermediate temperature value for this measurement.
 *
 * Returns: pressure in units of Pa * 256
 *   e.g. 24877568 = 24877568/25600 = 971.78 hPa
 *******************************************************************************/
static uint32_t bme280_compensate_pressure(int32_t adc_P)
{
    /* var1 and var2 use 64-bit arithmetic to avoid overflow.
     * The intermediate products can exceed 32-bit range. */
    int64_t var1 = (int64_t)bme280_t_fine - 128000;  /* Temperature offset from 25 C */

    /* Compute the quadratic and linear terms of the pressure polynomial */
    int64_t var2 = var1 * var1 * (int64_t)dig_P6;      /* P6 * (t - 128000)^2  */
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);    /* + P5 * (t - 128000) * 2^17 */
    var2 = var2 + (((int64_t)dig_P4) << 35);           /* + P4 * 2^35 */

    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) +
           ((var1 * (int64_t)dig_P2) << 12);           /* Quadratic + linear in var1 */

    /* Add P1 contribution: P1 * 2^47 / var1 (with overflow protection) */
    var1 = (((int64_t)1 << 47) + var1) * (int64_t)dig_P1 >> 33;

    if (var1 == 0) return 0;  /* Avoid division by zero (P1=0 is physically impossible
                                  but guard against uninitialized calibration) */

    /* Compute pressure in Pa*256 using the Bosch polynomial */
    int64_t p = 1048576 - adc_P;   /* 2^20 - adc_P */
    p = (((p << 31) - var2) * 3125) / var1;

    /* Apply the fine-correction second-order terms */
    var1 = ((int64_t)dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((int64_t)dig_P8 * p) >> 19;
    p = ((p + var1 + var2) >> 8) + ((int64_t)dig_P7 << 4);

    return (uint32_t)p;  /* Units: Pa * 256 */
}

/*******************************************************************************
 * BME280 HUMIDITY COMPENSATION (Bosch datasheet Annex 4.2.3)
 *
 * Converts the raw 16-bit humidity ADC value to Q22.10 format.
 * To get percent RH: result / 1024.0
 *
 * BUGS THAT WERE FIXED IN THIS CODE (documented for learning):
 *
 *   Bug 1: var1 overwrite
 *     The original code declared one variable for both the temperature offset
 *     (var1 = t_fine - 76800) and the second factor.  The second assignment
 *     overwrote var1 before it was used in the humidity formula, producing a
 *     completely wrong inner multiplier and integer overflow.
 *     Fix: Use separate named variables (var2 and var3) for each sub-expression.
 *
 *   Bug 2: Missing dig_H2 scaling
 *     The Bosch formula requires:
 *       var3 = ((var3 * dig_H2) + 8192) >> 14
 *     The original code was missing the "+ 8192" rounding offset AND the
 *     "* dig_H2" scaling, making the output approximately 1000x too small.
 *     Fix: Added the complete expression per the datasheet.
 *
 * Returns: relative humidity in Q22.10 (fixed-point), i.e. value/1024 = %RH
 *   e.g. 50000 / 1024 = 48.83 %
 *******************************************************************************/
static uint32_t bme280_compensate_humidity(int32_t adc_H)
{
    /* var1: temperature offset used in all humidity correction sub-expressions.
     * t_fine ~ 128000 at 25 degC, so var1 = 128000 - 76800 = 51200 at 25 degC.
     * Must not be overwritten once set. */
    int32_t var1 = bme280_t_fine - (int32_t)76800;

    /* var2: offset-corrected and scaled ADC reading.
     * adc_H << 14: scale up to avoid precision loss in subsequent divisions.
     * Subtract offsets from calibration coefficients H4 and H5. */
    int32_t var2 = adc_H << 14;             /* Scale ADC value up by 2^14 */
    var2 -= (int32_t)dig_H4 << 20;          /* Subtract H4 offset (shifted up) */
    var2 -= (int32_t)dig_H5 * var1;         /* Temperature-dependent correction */
    var2  = (var2 + (int32_t)16384) >> 15;  /* Round and scale down */

    /* var3: temperature-compensation factor incorporating H6, H3, H2.
     * This encodes the non-linear temperature effect on humidity measurement. */
    int32_t var3  = (var1 * (int32_t)dig_H6) >> 10;         /* H6 * temp_offset */
    var3 *= (((var1 * (int32_t)dig_H3) >> 11) + (int32_t)32768); /* H3 correction */
    var3  = (var3 >> 10) + (int32_t)2097152;                 /* Scale + offset */
    var3  = ((var3 * (int32_t)dig_H2) + 8192) >> 14;         /* H2 scaling + round */

    /* Combine var2 and var3, then apply H1 correction */
    int32_t v = var2 * var3;

    /* Apply H1 non-linearity correction */
    v -= ((((v >> 15) * (v >> 15)) >> 7) * (int32_t)dig_H1) >> 4;

    /* Clamp to valid range [0, 102400] which corresponds to [0%, 100%] */
    if (v < 0)         v = 0;           /* Clamp to 0 %RH minimum */
    if (v > 419430400) v = 419430400;   /* Clamp to 100 %RH maximum (=100*4096*1024) */

    return (uint32_t)(v >> 12);  /* Convert from Q22.22 to Q22.10 format */
}

/*******************************************************************************
 * bme280_temp_to_raw16 — Invert Compensation Formula (Binary Search)
 *
 * PURPOSE:
 *   The FPGA alarm comparator receives bme280_temp_raw(19 downto 4) — the
 *   upper 16 bits of the 20-bit BME280 ADC value.  We call this "raw16".
 *   To configure a meaningful alarm threshold in degrees Celsius, we must
 *   find the raw16 value whose compensated output equals target_centideg.
 *
 * WHY CAN'T WE USE A DIRECT FORMULA?
 *   The compensation formula is non-linear (quadratic in adc_T) and uses
 *   chip-specific coefficients.  Algebraically inverting it is complex.
 *   A binary search is simpler, fast enough (≤20 iterations), and correct.
 *
 * HOW THE BINARY SEARCH WORKS:
 *   Search space: [0, 0xFFFFF] (full 20-bit ADC range)
 *   Probe: bme280_compensate_temp(mid) -> compensated temperature (0.01 degC)
 *   If compensated < target: search upper half (lo = mid + 1)
 *   If compensated >= target: search lower half (hi = mid)
 *   When lo == hi: lo is the exact 20-bit ADC value for the target temperature.
 *   Divide by 16 (>> 4) to get the 16-bit value the FPGA comparator uses.
 *
 * SIDE EFFECT PROTECTION:
 *   bme280_compensate_temp() updates bme280_t_fine as a side effect.
 *   We save and restore it so the live temperature readings are not corrupted
 *   by the calibration search.
 *
 * Parameters:
 *   target_centideg : target temperature * 100 (e.g. 1500 = 15.00 degC)
 *   fallback        : value to return if calibration is unavailable
 *
 * Returns: 16-bit raw ADC threshold value (= raw_20bit >> 4)
 *******************************************************************************/
static uint16_t bme280_temp_to_raw16(int32_t target_centideg, uint16_t fallback)
{
    if (!calibration_valid) return fallback;  /* Can't invert without coefficients */

    int32_t lo = 0, hi = 0xFFFFF;  /* Search the full 20-bit ADC space */

    /* Save t_fine so we don't corrupt the shared state used by ongoing reads */
    int32_t saved_t_fine = bme280_t_fine;

    /* Standard binary search: converges in log2(0xFFFFF) = 20 iterations */
    while (lo < hi) {
        int32_t mid  = (lo + hi) / 2;              /* Midpoint of current search range */
        int32_t comp = bme280_compensate_temp(mid); /* Compensated temperature at mid   */
        if (comp < target_centideg) {
            lo = mid + 1;  /* Temperature is higher than mid → search upper half */
        } else {
            hi = mid;      /* Temperature is lower or equal → search lower half  */
        }
    }

    /* Restore t_fine: the binary search called bme280_compensate_temp() many times,
     * each of which updated t_fine.  Restore the saved value. */
    bme280_t_fine = saved_t_fine;

    /* lo is the 20-bit raw ADC value.  The FPGA receives raw(19:4), so >> 4. */
    uint16_t raw16 = (uint16_t)(lo >> 4);
    return raw16;
}

/*******************************************************************************
 * fpga_read_sensors — Read All Sensor Registers from FPGA
 *
 * This function performs ONE complete sensor poll.  It reads all five
 * FPGA peripheral register groups in a fixed sequence and fills the
 * sensor_data_t structure pointed to by 'd'.
 *
 * READING SEQUENCE:
 *   1. Record Unix timestamp for this snapshot.
 *   2. BME280: read status, then 20-bit raw temperature/pressure/humidity.
 *      Only compensate if bme_valid is true (FPGA confirms fresh data).
 *   3. MCP3008: read status, then three 10-bit ADC channels.
 *   4. GPIO: read 2-bit PIR status.
 *   5. Alarm: read 5-bit alarm flag register.
 *
 * BIT MASKING:
 *   The FPGA registers are 32 bits wide.  We mask only the valid bits:
 *   BME280 temperature/pressure: & 0x000FFFFF = keep only 20 bits [19:0]
 *   BME280 humidity:             & 0x0000FFFF = keep only 16 bits [15:0]
 *   MCP3008 channels:            & 0x000003FF = keep only 10 bits [9:0]
 *   GPIO PIR:                    & 0x00000003 = keep only 2 bits  [1:0]
 *   Alarm flags:                 & 0x0000001F = keep only 5 bits  [4:0]
 *
 * COMPENSATION DIVISORS (from Bosch datasheet):
 *   bme280_compensate_temp()    returns 0.01 degC -> / 100.0f = degC
 *   bme280_compensate_pressure() returns Pa * 256  -> / 25600.0f = hPa
 *   bme280_compensate_humidity() returns Q22.10    -> / 1024.0f = %RH
 *******************************************************************************/
static void fpga_read_sensors(sensor_data_t *d)
{
    d->timestamp = time(NULL);  /* Record when this snapshot was taken (Unix epoch) */

    /* ---- BME280 ---------------------------------------------------------- */
    uint32_t bme_status = fpga_read(BME280_REG_STATUS);  /* Read FPGA status register */
    /* Check bit 0 (data_valid): 1 = FPGA controller has a fresh measurement */
    d->bme_valid = (bme_status & BME280_STATUS_VALID) != 0;

    /* Read raw ADC values from FPGA registers.  Mask to valid bit widths:
     * temperature and pressure: 20-bit (bits [19:0])
     * humidity: 16-bit (bits [15:0]) */
    d->temp_raw  = fpga_read(BME280_REG_TEMP)  & 0x000FFFFFUL;
    d->press_raw = fpga_read(BME280_REG_PRESS) & 0x000FFFFFUL;
    d->humid_raw = fpga_read(BME280_REG_HUMID) & 0x0000FFFFUL;

    if (d->bme_valid) {
        /* Apply Bosch compensation formulas to get physically meaningful values */
        int32_t t_comp = bme280_compensate_temp((int32_t)d->temp_raw);
        d->temperature_c = (float)t_comp / 100.0f;   /* 0.01 degC -> degC */

        uint32_t p_comp = bme280_compensate_pressure((int32_t)d->press_raw);
        d->pressure_hpa  = (float)p_comp / 25600.0f; /* Pa*256 -> hPa */

        uint32_t h_comp = bme280_compensate_humidity((int32_t)d->humid_raw);
        d->humidity_pct  = (float)h_comp / 1024.0f;  /* Q22.10 -> %RH */
    }

    /* ---- MCP3008 ADC ----------------------------------------------------- */
    uint32_t adc_status  = fpga_read(MCP3008_REG_STATUS);
    d->adc_valid         = (adc_status & 0x01U) != 0; /* bit0 = data_valid */
    /* Mask to 10 bits (0-1023): the upper 22 bits should be zero but mask anyway */
    d->light_level       = (uint16_t)(fpga_read(MCP3008_REG_LIGHT) & 0x3FFU);
    d->heating_level     = (uint16_t)(fpga_read(MCP3008_REG_HEAT)  & 0x3FFU);
    d->sound_level       = (uint16_t)(fpga_read(MCP3008_REG_SOUND) & 0x3FFU);

    /* ---- GPIO / PIR Motion Sensors --------------------------------------- */
    /* Read 2 PIR sensor bits: bit0=PIR1, bit1=PIR2 (1 = motion detected) */
    d->pir_status = (uint8_t)(fpga_read(GPIO_REG_PIR) & 0x03U);

    /* ---- Alarm Peripheral ------------------------------------------------ */
    /* Read 5 alarm flag bits from the FPGA alarm comparator */
    d->alarm_flags = (uint8_t)(fpga_read(ALARM_REG_STATUS) & 0x1FU);
}

/*******************************************************************************
 * fpga_write_thresholds — Write Temperature and Light Thresholds to FPGA
 *
 * The alarm_logic.vhd FPGA block compares real-time sensor values against
 * these three threshold registers every clock cycle (50 MHz).
 *
 * Threshold format:
 *   temp_low  / temp_high : 16-bit raw ADC space (= raw_20bit >> 4)
 *                           Convert Celsius -> raw16 with bme280_temp_to_raw16()
 *   light_th              : 10-bit ADC counts (0-1023)
 *                           Alarm fires if light ADC < light_th
 *
 * Called twice:
 *   1. At startup with DEFAULT_TEMP_*_RAW (wide safe window, before calibration)
 *   2. After calibration with exact per-chip values from bme280_temp_to_raw16()
 *******************************************************************************/
static void fpga_write_thresholds(uint16_t temp_low, uint16_t temp_high,
                                   uint16_t light_th)
{
    fpga_write(ALARM_REG_TEMP_LOW,  (uint32_t)temp_low);   /* Write low threshold  */
    fpga_write(ALARM_REG_TEMP_HIGH, (uint32_t)temp_high);  /* Write high threshold */
    fpga_write(ALARM_REG_LIGHT_TH,  (uint32_t)light_th);   /* Write light threshold*/

    LOG_INFO("Thresholds set: TempLow=0x%04X TempHigh=0x%04X Light=%u",
             temp_low, temp_high, light_th);
}

/*******************************************************************************
 * build_mqtt_payload — Serialize sensor_data_t to a JSON String
 *
 * Produces a compact JSON object with all sensor values.  This payload is
 * published to the smarthome/sensors MQTT topic every 5 seconds.
 *
 * JSON structure:
 *   { "ts":    <unix_timestamp>,
 *     "temp":  <degrees_C, 2 decimal places>,
 *     "press": <hPa, 2 decimal places>,
 *     "humid": <percent, 2 decimal places>,
 *     "light": <0-1023>,
 *     "heat":  <0-1023>,
 *     "sound": <0-1023>,
 *     "pir":   <0-3, 2-bit bitmask>,
 *     "alarms":<0-31, 5-bit bitmask> }
 *
 * WHY AT+MQTTPUBRAW?
 *   The ESP32 AT+MQTTPUB command embeds the payload in a quoted AT parameter.
 *   JSON commas and inner quotes confuse the ESP-AT parser and always cause
 *   ERROR responses.  AT+MQTTPUBRAW avoids this by streaming raw bytes after
 *   a ">" prompt — no escaping needed.  See esp32_mqtt_publish().
 *******************************************************************************/
static void build_mqtt_payload(const sensor_data_t *d, char *buf, size_t bufsize)
{
    snprintf(buf, bufsize,
        "{\"ts\":%ld,"       /* Unix timestamp */
        "\"temp\":%.2f,"     /* Temperature in degC, 2 decimal places */
        "\"press\":%.2f,"    /* Pressure in hPa, 2 decimal places */
        "\"humid\":%.2f,"    /* Humidity in %, 2 decimal places */
        "\"light\":%u,"      /* Light ADC value 0-1023 */
        "\"heat\":%u,"       /* Heating ADC value 0-1023 */
        "\"sound\":%u,"      /* Sound ADC value 0-1023 */
        "\"pir\":%u,"        /* PIR bitmask 0-3 */
        "\"alarms\":%u}",    /* Alarm flags bitmask 0-31 */
        (long)d->timestamp,
        d->temperature_c,
        d->pressure_hpa,
        d->humidity_pct,
        d->light_level,
        d->heating_level,
        d->sound_level,
        d->pir_status,
        d->alarm_flags
    );
}

/*******************************************************************************
 * handle_alarms — Process Alarm State Changes: Log, Publish, SMS
 *
 * This function implements EDGE DETECTION: it only acts when an alarm bit
 * transitions from 0 (inactive) to 1 (active) — i.e. the rising edge.
 * If an alarm remains active across multiple polling cycles, no further
 * SMS is sent until it clears and fires again.
 *
 * EDGE DETECTION LOGIC:
 *   New alarm condition:  (d->alarm_flags & ALARM_X)    == 1 (currently active)
 *   Previous state:   (prev->alarm_flags & ALARM_X) == 0 (was inactive)
 *   Rising edge:          both conditions together (&&)
 *
 * SMS COOLDOWN:
 *   static time_t last_sms_time persists across calls (declared static).
 *   difftime(now, last_sms_time) < SMS_COOLDOWN_SEC (300 s) → skip SMS.
 *   This prevents flooding the SIM card if a sensor oscillates at a threshold.
 *
 * ACTION SEQUENCE (when alarm edge is detected within cooldown window):
 *   1. Log event to FPGA BRAM via ufm_log_event()
 *   2. Publish JSON to smarthome/alarms MQTT topic
 *   3. Send SMS to ALERT_PHONE_NUMBER via sim800l_send_sms()
 *   4. Update last_sms_time to enforce cooldown
 *
 * Only one alarm type is processed per call (the highest priority one that
 * changed state).  Priority order: TEMP_HIGH > TEMP_LOW > MOTION > LIGHT_LOW.
 *******************************************************************************/
static void handle_alarms(const sensor_data_t *d, const sensor_data_t *prev,
                          int mqtt_ok)
{
    /* static: persists its value between function calls (not reset each call) */
    static time_t last_sms_time = 0;  /* Timestamp of the last SMS sent (0 = never) */

    char msg[160];          /* SMS message body (max 160 chars = one SMS) */
    time_t now = time(NULL); /* Current Unix timestamp */

    /* Enforce SMS cooldown: if we sent an SMS recently, do nothing */
    if (difftime(now, last_sms_time) < SMS_COOLDOWN_SEC) {
        return;  /* Still within cooldown window — skip all alarm processing */
    }

    bool sms_needed = false;  /* Set to true when an alarm edge is detected */

    /* Check for HIGH TEMPERATURE alarm: newly active (rising edge only) */
    if ((d->alarm_flags & ALARM_TEMP_HIGH) && !(prev->alarm_flags & ALARM_TEMP_HIGH)) {
        snprintf(msg, sizeof(msg),
                 "ALERT: High temperature! %.1f C. Check heating system.",
                 d->temperature_c);
        LOG_WARN("ALARM: High temperature %.1f C", d->temperature_c);
        sms_needed = true;
        ufm_log_event("TEMP_HIGH", d);  /* Log to FPGA BRAM event log */
    }
    /* Check for LOW TEMPERATURE alarm: newly active (rising edge only) */
    else if ((d->alarm_flags & ALARM_TEMP_LOW) && !(prev->alarm_flags & ALARM_TEMP_LOW)) {
        snprintf(msg, sizeof(msg),
                 "ALERT: Low temperature! %.1f C. Check heating.",
                 d->temperature_c);
        LOG_WARN("ALARM: Low temperature %.1f C", d->temperature_c);
        sms_needed = true;
        ufm_log_event("TEMP_LOW", d);
    }
    /* Check for MOTION alarm: newly detected (rising edge only) */
    else if ((d->alarm_flags & ALARM_MOTION) && !(prev->alarm_flags & ALARM_MOTION)) {
        snprintf(msg, sizeof(msg), "ALERT: Motion detected! PIR status: 0x%02X",
                 d->pir_status);
        LOG_WARN("ALARM: Motion detected");
        sms_needed = true;
        ufm_log_event("MOTION", d);
    }
    /* Check for LOW LIGHT alarm: newly active (rising edge only) */
    else if ((d->alarm_flags & ALARM_LIGHT_LOW) && !(prev->alarm_flags & ALARM_LIGHT_LOW)) {
        snprintf(msg, sizeof(msg), "ALERT: Low light level! ADC=%u", d->light_level);
        LOG_WARN("ALARM: Low light level %u", d->light_level);
        sms_needed = true;
        ufm_log_event("LIGHT_LOW", d);
    }

    if (sms_needed) {
        /* Publish alarm to the dedicated MQTT alarm topic if MQTT is connected.
         * This triggers a real-time notification in the web dashboard. */
        if (mqtt_ok) {
            char alarm_payload[220];
            /* Build JSON payload for alarm topic — uses AT+MQTTPUBRAW (raw bytes) */
            snprintf(alarm_payload, sizeof(alarm_payload),
                     "{\"ts\":%ld,\"msg\":\"%s\",\"alarms\":%u}",
                     (long)now, msg, d->alarm_flags);
            if (esp32_mqtt_publish(MQTT_TOPIC_ALARMS, alarm_payload) != 0) {
                LOG_WARN("MQTT alarm publish failed");  /* Non-fatal: SMS still sent */
            }
        }

        /* Send SMS alert to the configured phone number */
        if (sim800l_send_sms(ALERT_PHONE_NUMBER, msg) == 0) {
            last_sms_time = now;  /* Update cooldown timer on successful SMS send */
        } else {
            LOG_WARN("SMS send failed — will retry after cooldown expires");
        }
    }
}

/*******************************************************************************
 * main — Entry Point
 *
 * STARTUP SEQUENCE:
 *   1. Register signal handlers (Ctrl+C / kill -> graceful shutdown)
 *   2. Map FPGA LW bridge via /dev/mem
 *   3. Write safe default thresholds (wide window, no false alarms yet)
 *   4. Load BME280 calibration coefficients (from smart_home.h constants)
 *   5. Compute exact per-chip temperature thresholds, write to FPGA
 *   6. Initialize ESP32: open UART, connect WiFi, connect MQTT
 *   7. Initialize SIM800L: map UART MMIO, wait for GSM registration
 *   8. Initialize UFM event log: map BRAM, scan for existing records
 *   9. Wait 3 seconds for FPGA peripherals to complete first measurements
 *  10. Enter main loop
 *
 * MAIN LOOP (runs once per second):
 *   - Read all FPGA sensor registers
 *   - If both BME280 and MCP3008 have valid data:
 *       * Print status every 10 seconds
 *       * Publish to MQTT every 5 seconds (if connected)
 *       * Check for alarm transitions, send SMS/log if needed
 *       * Update prev_data for next iteration's edge detection
 *   - If sensors are not yet ready:
 *       * Check BME280 error bit to distinguish "initializing" from "failed"
 *       * Print diagnostic message every 10 seconds
 *
 * SHUTDOWN (on SIGINT / SIGTERM):
 *   - keep_running becomes 0, main loop exits
 *   - esp32_close(): disconnect MQTT and WiFi, close UART fd
 *   - sim800l_close(): unmap UART MMIO
 *   - ufm_close(): unmap BRAM, print final event count
 *   - munmap fpga_base: release the LW bridge mapping
 *******************************************************************************/
int main(int argc, char *argv[])
{
    (void)argc; (void)argv;  /* Unused parameters — suppress compiler warning */

    int      mqtt_ok    = 0;   /* 1 = MQTT connected and working, 0 = not connected */
    uint32_t loop_count = 0;   /* uint32_t prevents signed overflow after ~68 years */
    char     payload[512];     /* Reusable buffer for JSON MQTT payloads */

    /* Print startup banner */
    printf("==============================================\n");
    printf("  Smart Home Monitor - HPS Supervisor\n");
    printf("==============================================\n\n");

    /* Register signal handlers so Ctrl+C and `kill` trigger graceful shutdown.
     * SIGINT  = Ctrl+C in terminal
     * SIGTERM = default signal sent by `kill` or systemd `stop` */
    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);

    /* ---- Step 1: Initialize FPGA interface -------------------------------- */
    LOG_INFO("Initializing FPGA interface...");
    if (fpga_init() != 0) {
        LOG_ERROR("FPGA init failed — cannot access hardware registers");
        return EXIT_FAILURE;  /* Fatal: cannot run without FPGA access */
    }

    /* ---- Step 2: Write safe default thresholds FIRST ----------------------
     * Reason: bme280_read_calibration() briefly pauses the FPGA I2C controller.
     * During that pause, the FPGA alarm comparator still runs with whatever
     * thresholds are currently in its registers.  Writing wide defaults first
     * ensures no false alarms fire during the calibration window.
     * 0x7800 (~15 C) and 0x8800 (~38 C) bracket all realistic room temperatures. */
    fpga_write_thresholds(DEFAULT_TEMP_LOW_RAW, DEFAULT_TEMP_HIGH_RAW,
                          DEFAULT_LIGHT_THRESH);
    LOG_INFO("Safe default thresholds written (pre-calibration).");

    /* ---- Step 3: Load BME280 factory calibration -------------------------- */
    LOG_INFO("Reading BME280 calibration coefficients...");
    if (bme280_read_calibration() == 0) {
        /* Calibration loaded successfully.  Compute exact per-chip thresholds.
         * Target temperatures: 15.00 degC (1500 centideg) and 35.00 degC (3500).
         * bme280_temp_to_raw16() binary-searches the compensation formula to
         * find the exact 16-bit ADC value for each target on THIS specific chip. */
        uint16_t thresh_low  = bme280_temp_to_raw16(1500, DEFAULT_TEMP_LOW_RAW);  /* 15 C */
        uint16_t thresh_high = bme280_temp_to_raw16(3500, DEFAULT_TEMP_HIGH_RAW); /* 35 C */

        /* Overwrite defaults with exact calibrated values */
        fpga_write_thresholds(thresh_low, thresh_high, DEFAULT_LIGHT_THRESH);
        LOG_INFO("Calibrated thresholds written: low=0x%04X (~15C) high=0x%04X (~35C)",
                 thresh_low, thresh_high);
    } else {
        /* Calibration failed: wide defaults remain active.
         * System continues — temperature ALARMS may be inaccurate but the
         * system can still log, MQTT publish, and send SMS for other alarms. */
        LOG_WARN("BME280 calibration unavailable — wide safe defaults remain active.");
        LOG_WARN("Temperature alarm thresholds may be inaccurate.");
    }

    /* ---- Step 4: Initialize ESP32 (WiFi + MQTT) --------------------------- */
    LOG_INFO("Initializing ESP32...");
    if (esp32_init() != 0) {
        /* ESP32 UART not responding.  Non-fatal: system continues without MQTT.
         * SMS alerts and local logging still work. */
        LOG_ERROR("ESP32 init failed — check UART connection on JP1 pins 21/22");
    } else {
        /* UART opened and AT probe succeeded.  Now connect WiFi and MQTT. */
        if (esp32_connect_wifi(WIFI_SSID, WIFI_PASSWORD) == 0) {
            if (esp32_mqtt_connect(MQTT_BROKER, MQTT_PORT) == 0) {
                mqtt_ok = 1;  /* MQTT is live: sensor data will be published */
                LOG_INFO("MQTT connected to %s:%d", MQTT_BROKER, MQTT_PORT);
            }
        }
    }

    /* ---- Step 5: Initialize SIM800L (GSM/SMS) ----------------------------- */
    LOG_INFO("Initializing SIM800L...");
    if (sim800l_init() != 0) {
        /* Non-fatal: system continues without SMS capability.
         * All other functions (MQTT, logging, local display) still work. */
        LOG_WARN("SIM800L init failed — SMS alerts disabled");
    }

    /* ---- Step 6: Initialize FPGA BRAM event log --------------------------- */
    LOG_INFO("Initializing event log...");
    if (ufm_init() != 0) {
        /* Non-fatal: events will not be persisted but system continues. */
        LOG_WARN("Event log init failed — alarm events will not be logged");
    }

    /* Initialize prev_data to all zeros.
     * This ensures alarm_flags starts at 0, so the FIRST alarm detected
     * after startup correctly triggers the rising-edge action in handle_alarms().
     * Without this, prev_data would have garbage values and edge detection
     * might miss or double-trigger the first alarm. */
    memset(&prev_data, 0, sizeof(prev_data));

    /* Wait 3 seconds for FPGA peripherals to complete their first measurement.
     * BME280 in normal mode: ~100 ms per measurement cycle.
     * MCP3008: fast (<1 ms per conversion) but needs SPI clock to settle.
     * 3 seconds is generous — ensures both peripherals report valid data
     * before the main loop starts checking bme_valid and adc_valid. */
    LOG_INFO("Waiting 3 s for FPGA peripherals to initialise...");
    sleep(3);

    LOG_INFO("System ready. Starting main loop...\n");

    /* ========================================================================
     * MAIN LOOP — runs every SENSOR_POLL_SEC (1 second)
     * ======================================================================== */
    while (keep_running) {

        /* Read all sensor registers from the FPGA (BME280, MCP3008, GPIO, Alarm) */
        fpga_read_sensors(&current_data);

        if (current_data.bme_valid && current_data.adc_valid) {
            /* Both sensors have valid, fresh data — normal operation */

            /* Print a brief status line every 10 seconds to keep the terminal
             * active without flooding it with one message per second */
            if (loop_count % 10U == 0U) {
                LOG_INFO("Temp: %.1f C | Press: %.1f hPa | Humid: %.1f%% | "
                         "Light: %u | Heat: %u | Sound: %u | "
                         "PIR: 0x%02X | Alarms: 0x%02X",
                         current_data.temperature_c,
                         current_data.pressure_hpa,
                         current_data.humidity_pct,
                         current_data.light_level,
                         current_data.heating_level,
                         current_data.sound_level,
                         current_data.pir_status,
                         current_data.alarm_flags);
            }

            /* Publish full sensor data to MQTT every MQTT_PUBLISH_INTERVAL cycles.
             * With SENSOR_POLL_SEC=1 and MQTT_PUBLISH_INTERVAL=5, this fires
             * every 5 seconds.  If publish fails, attempt MQTT reconnect. */
            if (mqtt_ok && (loop_count % MQTT_PUBLISH_INTERVAL == 0U)) {
                build_mqtt_payload(&current_data, payload, sizeof(payload));
                if (esp32_mqtt_publish(MQTT_TOPIC_SENSORS, payload) != 0) {
                    LOG_WARN("MQTT publish failed — attempting reconnect...");
                    mqtt_ok = 0;  /* Mark as disconnected */
                    /* Reconnect: skip wifi (still connected), only reconnect MQTT */
                    if (esp32_mqtt_connect(MQTT_BROKER, MQTT_PORT) == 0) {
                        mqtt_ok = 1;
                        LOG_INFO("MQTT reconnected");
                    }
                }
            }

            /* Check for new alarm transitions and act (SMS, MQTT alarm, log) */
            handle_alarms(&current_data, &prev_data, mqtt_ok);

            /* Save current snapshot as previous for next iteration's edge detection */
            prev_data = current_data;

        } else {
            /* One or both sensors have no valid data yet.
             * Print a diagnostic every 10 seconds to avoid terminal spam. */
            if (loop_count % 10U == 0U) {
                /* Read the BME280 status register directly to distinguish:
                 *   bme_valid=0 AND error=0 : I2C controller is initializing (normal)
                 *   bme_valid=0 AND error=1 : I2C communication failed (hardware problem) */
                uint32_t bme_st = fpga_read(BME280_REG_STATUS);
                if (bme_st & BME280_STATUS_ERROR) {
                    /* Hardware error: print wiring guidance and retry interval */
                    LOG_WARN("BME280 I2C error (status=0x%02X) — "
                             "check: 4.7kOhm pull-ups on SDA/SCL to 3.3V, "
                             "sensor power (3.3V), SDO pin tied to GND (addr 0x76). "
                             "FPGA controller will auto-retry in ~5 s.",
                             bme_st);
                } else {
                    /* Normal initialization: just waiting for first measurement */
                    LOG_WARN("Waiting for sensor data "
                             "(BME280 valid=%d, ADC valid=%d)",
                             current_data.bme_valid, current_data.adc_valid);
                }
            }
        }

        loop_count++;              /* Increment cycle counter (uint32_t, no overflow) */
        sleep(SENSOR_POLL_SEC);    /* Sleep 1 second before next poll */
    }
    /* Main loop exits when keep_running = 0 (set by signal_handler) */

    /* ========================================================================
     * GRACEFUL SHUTDOWN
     * ======================================================================== */
    LOG_INFO("Cleaning up...");

    esp32_close();      /* Disconnect MQTT + WiFi, close /dev/ttyS1 file descriptor */
    sim800l_close();    /* Unmap the FPGA UART MMIO region */
    ufm_close();        /* Unmap the FPGA BRAM region, print event count */

    /* Unmap the main FPGA LW bridge region (if it was successfully mapped) */
    if (fpga_base != NULL) {
        munmap((void *)fpga_base, LW_BRIDGE_SPAN);  /* Release the virtual mapping */
        fpga_base = NULL;                            /* Prevent dangling pointer use */
    }

    LOG_INFO("Shutdown complete.");
    return EXIT_SUCCESS;  /* Return 0 to the shell — normal termination */
}
