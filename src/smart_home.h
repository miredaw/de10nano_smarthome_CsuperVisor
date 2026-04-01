/*******************************************************************************
 * smart_home.h — Central Shared Header for the Smart Home Monitor
 *
 * PURPOSE:
 *   This single header file is included by ALL .c files in the project.
 *   It defines:
 *     1. Physical addresses of every FPGA peripheral (via the LW AXI bridge)
 *     2. Register offsets and bit masks for each peripheral
 *     3. The sensor_data_t structure that carries all live readings
 *     4. BME280 factory calibration constants (pre-loaded from the chip)
 *     5. Network / GSM configuration (WiFi SSID, MQTT broker, phone number)
 *     6. Timing constants (poll interval, MQTT publish rate, SMS cooldown)
 *     7. Logging macros (LOG_INFO / LOG_WARN / LOG_ERROR)
 *     8. Function declarations for esp32_handler.c, sim800l_handler.c,
 *        and ufm_storage.c so that main_supervisor.c can call them.
 *
 * HOW THE ADDRESS MAP WORKS:
 *   The Cyclone V SoC has a "Lightweight HPS-to-FPGA" AXI bridge that maps
 *   the FPGA Avalon-MM fabric into the HPS address space at 0xFF200000.
 *   Each custom IP block (BME280, MCP3008, GPIO, Alarm, UART) has a base
 *   offset assigned in Platform Designer (Qsys).  The HPS supervisor reads
 *   and writes these peripherals via direct 32-bit memory accesses after
 *   mapping the bridge with mmap(/dev/mem).
 *
 * IMPORTANT — FIXES APPLIED (v2):
 *   1. _GNU_SOURCE must be defined BEFORE any system header.  It unlocks
 *      POSIX/GNU extensions such as usleep() and CRTSCTS (needed in termios).
 *      Using #ifndef guards so including this header multiple times is safe.
 *   2. DEFAULT_TEMP_LOW_RAW / DEFAULT_TEMP_HIGH_RAW were wrong (0x1400/0x1E00).
 *      Those values sat below typical BME280 room-temperature readings, causing
 *      the FPGA alarm to fire the moment the system booted.  Corrected to
 *      0x7800/0x8800 (~15 C / ~38 C in the 16-bit raw ADC space).
 *
 *******************************************************************************/

#ifndef SMART_HOME_H   /* Include guard: prevents this header being processed twice */
#define SMART_HOME_H   /* Define the guard symbol so subsequent includes are skipped */

/* ---------------------------------------------------------------------------
 * _GNU_SOURCE must come before ANY system header (stdio, stdlib, termios, etc.)
 *
 * Without this macro, many POSIX/GNU symbols are hidden from the C standard
 * library headers:
 *   - usleep()        : microsecond sleep  (unistd.h)
 *   - CRTSCTS         : hardware RTS/CTS flow-control flag  (termios.h)
 *   - clock_gettime() : with CLOCK_MONOTONIC  (time.h)
 *   - strdup()        : string duplication  (string.h)
 *
 * Defining it here in the shared header guarantees it is set first in every
 * translation unit that includes smart_home.h, avoiding "implicit declaration"
 * compiler errors when compiling with -std=c99.
 * --------------------------------------------------------------------------- */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE   /* Unlock POSIX + GNU extensions across all included headers */
#endif

/* Standard library headers needed by the type definitions and macros below */
#include <stdint.h>   /* uint8_t, int16_t, uint32_t, int32_t, etc.               */
#include <stdbool.h>  /* C99 bool type with true and false constants              */
#include <time.h>     /* time_t (Unix timestamp), struct tm, time(), strftime()   */

/*******************************************************************************
 * SECTION 1 - Lightweight HPS-to-FPGA AXI Bridge
 *
 * The DE10-Nano has two HPS-to-FPGA bridges:
 *   - Full bridge  (2 GB window) — for large transfers, not used here
 *   - Lightweight bridge (2 MB window) — for memory-mapped I/O registers
 *
 * All five FPGA custom IP blocks are wired to the Lightweight bridge.
 * From the HPS ARM processor's perspective, the LW bridge appears as a
 * 2 MB window in the 32-bit physical address space starting at 0xFF200000.
 *
 * How to access from Linux user space (no kernel driver needed):
 *   1. fd = open("/dev/mem", O_RDWR | O_SYNC);
 *   2. base = mmap(NULL, LW_BRIDGE_SPAN, PROT_READ|PROT_WRITE,
 *                  MAP_SHARED, fd, LW_BRIDGE_BASE);
 *   3. close(fd);   // fd is no longer needed after mmap
 *   4. *(volatile uint32_t*)(base + peripheral_offset) = value;  // write reg
 *      value = *(volatile uint32_t*)(base + peripheral_offset);  // read reg
 *   5. munmap(base, LW_BRIDGE_SPAN);   // when done
 *
 * The volatile keyword is essential — it prevents the compiler from caching
 * the value in a CPU register, ensuring every access actually hits the hardware.
 *******************************************************************************/
#define LW_BRIDGE_BASE  0xFF200000UL  /* Physical base address of the LW bridge  */
#define LW_BRIDGE_SPAN  0x00200000UL  /* Window size: 2 MB (0x200000 bytes)      */

/*******************************************************************************
 * SECTION 2 - BME280 Environmental Sensor Peripheral
 *
 * Platform Designer component : bme280_i2c_0
 * Avalon-MM slave base offset  : 0x00   (absolute: 0xFF200000)
 *
 * The FPGA VHDL IP (bme280_controller.vhd + i2c_master.vhd) acts as an I2C
 * master connected to the Bosch BME280 sensor:
 *   SDA -> PIN_V12 (JP1 Pin 1)
 *   SCL -> PIN_E8  (JP1 Pin 2)
 *
 * The controller continuously polls the sensor and stores the raw results.
 * The HPS only reads these registers — it never drives the I2C bus itself.
 *
 * Register layout (each register is one 32-bit word = 4 bytes):
 *   +0x00  BME280_REG_TEMP   : raw temperature ADC [19:0], 20 bits valid
 *   +0x04  BME280_REG_PRESS  : raw pressure ADC    [19:0], 20 bits valid
 *   +0x08  BME280_REG_HUMID  : raw humidity ADC    [15:0], 16 bits valid
 *   +0x0C  BME280_REG_STATUS : status register (see bit masks below)
 *   +0x10  BME280_REG_CTRL   : control register (bit 0 = enable)
 *
 * IMPORTANT: The raw register values are NOT in physical units.  They must
 * be processed through the Bosch compensation formulas (Annex 4.2.3 of the
 * BME280 datasheet) to obtain degrees Celsius, hectopascals, and percent RH.
 * The compensation functions are implemented in main_supervisor.c.
 *******************************************************************************/
#define BME280_BASE         0x00UL            /* Avalon slave base offset        */

#define BME280_REG_TEMP     (BME280_BASE + 0x00UL)  /* Raw temperature ADC value */
#define BME280_REG_PRESS    (BME280_BASE + 0x04UL)  /* Raw pressure ADC value    */
#define BME280_REG_HUMID    (BME280_BASE + 0x08UL)  /* Raw humidity ADC value    */
#define BME280_REG_STATUS   (BME280_BASE + 0x0CUL)  /* Status: valid + error     */
#define BME280_REG_CTRL     (BME280_BASE + 0x10UL)  /* Control: bit0 = enable    */

/* Bit masks for BME280_REG_STATUS */
#define BME280_STATUS_VALID  (1U << 0)  /* 1 = fresh measurement is available    */
#define BME280_STATUS_ERROR  (1U << 1)  /* 1 = I2C communication error detected  */

/*******************************************************************************
 * SECTION 3 - MCP3008 SPI ADC Peripheral
 *
 * Platform Designer component : mcp3008_spi_adc_0
 * Avalon-MM slave base offset  : 0x20  (absolute: 0xFF200020)
 *
 * The FPGA VHDL IP (multi_channel_adc.vhd + spi_adc_mcp3008.vhd) drives a
 * Microchip MCP3008 8-channel 10-bit SPI ADC connected to JP1 pins 3-6:
 *   CLK  -> PIN_W12 (JP1 Pin 3)
 *   MOSI -> PIN_D11 (JP1 Pin 4)
 *   MISO -> PIN_D8  (JP1 Pin 5)
 *   CS_N -> PIN_AH13(JP1 Pin 6)
 *
 * Three analog channels are sampled and averaged:
 *   CH0 -> light sensor (LDR / photoresistor)       ADC range: 0-1023
 *   CH1 -> heating element (analog thermal sensor)  ADC range: 0-1023
 *   CH2 -> sound level (microphone + amplifier)     ADC range: 0-1023
 *
 * The data_valid bit is LATCHED (stays 1 until a new conversion starts),
 * so the HPS can safely read it at any polling rate without missing updates.
 *******************************************************************************/
#define MCP3008_BASE        0x20UL           /* Avalon slave base offset         */

#define MCP3008_REG_LIGHT   (MCP3008_BASE + 0x00UL)  /* CH0: light (0-1023)     */
#define MCP3008_REG_HEAT    (MCP3008_BASE + 0x04UL)  /* CH1: heating (0-1023)   */
#define MCP3008_REG_SOUND   (MCP3008_BASE + 0x08UL)  /* CH2: sound (0-1023)     */
#define MCP3008_REG_STATUS  (MCP3008_BASE + 0x0CUL)  /* bit0=data_valid         */
#define MCP3008_REG_CTRL    (MCP3008_BASE + 0x10UL)  /* bit0=start conversion   */

/*******************************************************************************
 * SECTION 4 - GPIO Controller Peripheral
 *
 * Platform Designer component : gpio_controller_0
 * Avalon-MM slave base offset  : 0x40  (absolute: 0xFF200040)
 *
 * Inputs (read by HPS):
 *   - 2 PIR motion sensors (active-high, HC-SR501 modules, 5V powered)
 *   - 4 push-buttons (active-low with internal pull-ups inside FPGA)
 *
 * Outputs (driven by alarm_logic.vhd, overrideable by HPS):
 *   - Red LED    : alarm state indicator
 *   - Green LED  : all-clear indicator
 *   - Yellow LED : motion indicator
 *   - Buzzer     : acoustic alarm
 *
 * The alarm_logic VHDL block drives the LED/buzzer combinationally in hardware.
 * The HPS supervisor reads PIR and alarm flags but does not need to write LEDs
 * unless it wants to override the hardware-driven state.
 *******************************************************************************/
#define GPIO_BASE           0x40UL           /* Avalon slave base offset         */

#define GPIO_REG_PIR        (GPIO_BASE + 0x00UL)  /* PIR inputs: bit0=PIR1, bit1=PIR2 */
#define GPIO_REG_BUTTON     (GPIO_BASE + 0x04UL)  /* Button inputs: bit0-3 = BTN0-3   */
#define GPIO_REG_LED        (GPIO_BASE + 0x08UL)  /* LED outputs (HPS override)        */
#define GPIO_REG_BUZZER     (GPIO_BASE + 0x0CUL)  /* Buzzer output (HPS override)      */

/* Bit masks for GPIO_REG_PIR register */
#define GPIO_PIR1_DETECT    (1U << 0)  /* bit0=1 means PIR sensor 1 sees motion  */
#define GPIO_PIR2_DETECT    (1U << 1)  /* bit1=1 means PIR sensor 2 sees motion  */

/* Bit masks for GPIO_REG_LED register */
#define GPIO_LED_RED        (1U << 0)  /* bit0: Red   LED  (1=on, temperature alarm) */
#define GPIO_LED_GREEN      (1U << 1)  /* bit1: Green LED  (1=on, all clear)         */
#define GPIO_LED_YELLOW     (1U << 2)  /* bit2: Yellow LED (1=on, motion detected)   */

/*******************************************************************************
 * SECTION 5 - Alarm Logic Peripheral
 *
 * Platform Designer component : alarm_logic_0
 * Avalon-MM slave base offset  : 0x60  (absolute: 0xFF200060)
 *
 * This is the REAL-TIME HARDWARE DECISION ENGINE.  Threshold comparisons run
 * at 50 MHz in FPGA fabric — completely independent of Linux scheduling latency.
 * LED and buzzer outputs are driven directly by this block; the HPS supervisor
 * only reads the alarm flags and acts on them (SMS, MQTT, log).
 *
 * !!! CRITICAL: THRESHOLD SCALE !!!
 *   The alarm comparator receives bme280_temp_raw(19 downto 4) from the VHDL.
 *   This means it sees the UPPER 16 BITS of the 20-bit BME280 ADC value.
 *   Formula: alarm_input = raw_20bit >> 4
 *
 *   Therefore all threshold values written to ALARM_REG_TEMP_* MUST be in
 *   this 16-bit space, NOT in degrees Celsius!
 *
 *   Example: 25 degC -> raw_20bit ~0x80000 -> alarm_input ~0x8000
 *   Default safe window: 0x7800 (~15 C) to 0x8800 (~38 C)
 *
 *   The correct per-chip threshold values are computed by bme280_temp_to_raw16()
 *   in main_supervisor.c using a binary search over the compensation formula.
 *
 * Alarm conditions and hardware outputs:
 *   TEMP_HIGH : raw > temp_high_thresh  -> Red LED on + buzzer pulses
 *   TEMP_LOW  : raw < temp_low_thresh   -> Red LED on + buzzer pulses
 *   LIGHT_LOW : light_adc < light_thresh -> Flag only (no LED)
 *   MOTION    : PIR1 or PIR2 active      -> Yellow LED on (5-sec latch)
 *   CRITICAL  : TEMP_HIGH OR TEMP_LOW    -> Buzzer: 250ms ON / 250ms OFF
 *   All clear : no alarms                -> Green LED on
 *******************************************************************************/
#define ALARM_BASE          0x60UL           /* Avalon slave base offset         */

#define ALARM_REG_STATUS    (ALARM_BASE + 0x00UL)  /* Alarm flag bits (read)     */
#define ALARM_REG_TEMP_LOW  (ALARM_BASE + 0x04UL)  /* Temp low threshold (write) */
#define ALARM_REG_TEMP_HIGH (ALARM_BASE + 0x08UL)  /* Temp high threshold (write)*/
#define ALARM_REG_LIGHT_TH  (ALARM_BASE + 0x0CUL)  /* Light threshold (write)    */
#define ALARM_REG_OUTPUTS   (ALARM_BASE + 0x10UL)  /* LED/buzzer states (read)   */

/* Bit masks for ALARM_REG_STATUS - 5 alarm bits */
#define ALARM_TEMP_HIGH     (1U << 0)  /* Temperature above upper threshold      */
#define ALARM_TEMP_LOW      (1U << 1)  /* Temperature below lower threshold      */
#define ALARM_LIGHT_LOW     (1U << 2)  /* Light ADC below light_thresh           */
#define ALARM_MOTION        (1U << 3)  /* PIR motion latched (5-second hold)     */
#define ALARM_CRITICAL      (1U << 4)  /* High-priority: any temperature alarm   */

/*******************************************************************************
 * SECTION 6 - Altera UART Peripheral (SIM800L GSM Module)
 *
 * Platform Designer component : uart_sim800l_0
 * Avalon-MM slave base offset  : 0x80  (absolute: 0xFF200080)
 *
 * Physical wiring (JP1 header):
 *   JP1 Pin 19 (PIN_D12)  = FPGA UART TX -> SIM800L RXD
 *   JP1 Pin 20 (PIN_AD20) = SIM800L TXD  -> FPGA UART RX
 *
 * This is an Altera/Intel UART IP synthesised in the FPGA fabric.
 * The altera_uart Linux kernel driver is NOT present in the DE10-Nano kernel
 * (v6.1.108-armv7-fpga), so /dev/ttyAL* does not appear.
 *
 * sim800l_handler.c therefore accesses this UART's registers directly via
 * /dev/mem + mmap, exactly like any other Avalon peripheral.
 *
 * The Altera UART register map (word-addressed, each reg = 4 bytes):
 *   +0x00  rxdata   [7:0]  : received byte       — read only when RRDY=1
 *   +0x04  txdata   [7:0]  : byte to transmit    — write only when TRDY=1
 *   +0x08  status          : bit7=RRDY, bit6=TRDY, bit5=TMT, bit3=ROE
 *   +0x0C  control         : interrupt-enable bits (kept 0 = polled mode)
 *   +0x10  divisor         : baud = f_clk / (divisor+1), may be fixed
 *   +0x14  endofpacket     : (not used in this project)
 *
 * RRDY = Receive  Data Ready (1 = a byte is waiting in the RX register)
 * TRDY = Transmit Ready      (1 = TX shift register has space for a byte)
 * TMT  = Transmit Empty      (1 = all TX bytes have been shifted out)
 * ROE  = Receive Overrun Error (1 = a byte was lost because RX was full)
 *******************************************************************************/
#define UART_SIM800L_BASE   0x80UL   /* Avalon offset; used by sim800l_handler.c  */

/*******************************************************************************
 * SECTION 7 - On-Chip Memory: UFM-style Event Log
 *
 * Platform Designer component : altera_avalon_onchip_memory2 (onchip_mem_0)
 * Avalon-MM slave base offset  : 0x2000  (absolute: 0xFF202000)
 * Total size                   : 8,192 bytes = 256 records x 32 bytes
 *
 * =========================================================================
 * WHERE IS THE LOG STORED?
 *   Inside FPGA Block RAM (BRAM) - on-chip flip-flop-based memory in the
 *   Cyclone V fabric.  It is NOT on the SD card, filesystem, or NAND flash.
 *   Physical address: 0xFF202000 (LW bridge 0xFF200000 + offset 0x2000)
 *
 * HOW IS DATA WRITTEN?
 *   ufm_storage.c calls mmap(/dev/mem) to map the LW bridge, then writes
 *   32-bit words directly with C pointer dereferences:
 *
 *     *(volatile uint32_t*)(lw_base + 0x2000 + word_offset * 4) = value;
 *
 *   No command protocol or busy-wait is needed: BRAM accepts direct
 *   word writes instantly (unlike SPI flash which needs an erase cycle).
 *
 * HOW IS DATA READ BACK?
 *   Method A - Via the running program:
 *     Call ufm_print_all_events() which scans from word 0 upward, checks
 *     each record's magic marker (0xCAFEBABE), reconstructs the struct,
 *     and prints a formatted ASCII table to stdout.
 *
 *   Method B - From the Linux shell (devmem2 tool):
 *     devmem2 0xFF202000 w   # Word 0 of event 0 (should be 0xCAFEBABE)
 *     devmem2 0xFF202004 w   # Word 1 of event 0 (Unix timestamp)
 *     devmem2 0xFF202008 w   # Word 2 of event 0 (event code + temp)
 *     ... and so on for each 4-byte word
 *
 *   Method C - Binary dump (all 8 KB):
 *     dd if=/dev/mem skip=$((0xFF202000)) count=8192 bs=1 of=ufm.bin
 *     hexdump -C ufm.bin
 *
 * EVENT RECORD FORMAT (32 bytes = 8 x 32-bit words, packed struct):
 *   Offset  Size  Field        Description
 *   +0x00   4 B   magic        0xCAFEBABE = valid entry, 0x00000000 = free slot
 *   +0x04   4 B   timestamp    Unix time (seconds since 1970-01-01 00:00:00 UTC)
 *   +0x08   2 B   event_code   0x0001=TEMP_HIGH, 0x0002=TEMP_LOW, etc.
 *   +0x0A   2 B   temp_x100    temperature_c * 100 as int16 (e.g. 2350 = 23.50 C)
 *   +0x0C   4 B   press_x100   pressure_hpa * 100 as uint32 (e.g. 96930 = 969.30)
 *   +0x10   4 B   humid_x100   humidity_pct * 100 as uint32 (e.g. 5500 = 55.00 %)
 *   +0x14   2 B   light        MCP3008 CH0 ADC value (0-1023)
 *   +0x16   2 B   heating      MCP3008 CH1 ADC value (0-1023)
 *   +0x18   2 B   sound        MCP3008 CH2 ADC value (0-1023)
 *   +0x1A   1 B   alarm_flags  5-bit bitmask of active alarms
 *   +0x1B   5 B   pad          Padding bytes (all 0x00) to reach 32 bytes total
 *
 * Why scale floats to integers?  BRAM stores raw bits; float representation
 * can differ between compiler versions and platforms.  Storing temp*100 as
 * an int16 is portable, reversible, and avoids endian/float ABI issues.
 *
 * STORAGE LAYOUT (word-addressed, each word = 4 bytes):
 *   Event 0  : words 0-7    (bytes 0x0000 - 0x001F)
 *   Event 1  : words 8-15   (bytes 0x0020 - 0x003F)
 *   Event N  : words N*8 .. N*8+7
 *   Event 255: words 2040-2047 (bytes 0x1FE0 - 0x1FFF) = last slot
 *
 * SCAN ALGORITHM (ufm_init):
 *   Walk from slot 0 upward reading the first word of each record:
 *     0xCAFEBABE -> valid event, count it, advance to next slot
 *     0x00000000 -> free slot, stop — this is where the next write goes
 *     anything else -> corrupted entry, skip
 *
 * PERSISTENCE:
 *   YES - survives HPS soft-reset (kernel panic, watchdog reboot)
 *   YES - survives HPS power off while FPGA stays configured
 *   NO  - lost when FPGA is reprogrammed (.sof / .rbf reload wipes BRAM)
 *   NO  - lost on full board power-off (BRAM is volatile flip-flop memory)
 *
 * HOW TO ERASE ALL LOGS:
 *   Option A (binary): sudo ./ufm_erase
 *   Option B (C code): ufm_clear_logs()
 *   Both write 0x00000000 to every word in the 8 KB region, restoring
 *   the BRAM to its power-on state so ufm_init() sees all free slots.
 * =========================================================================
 *******************************************************************************/
#define FLASH_DATA_BASE        0x2000UL   /* On-chip BRAM: LW bridge byte offset  */
#define FLASH_BYTES_PER_EVENT  32U        /* Each event record is exactly 32 bytes */
#define FLASH_MAX_EVENTS       256U       /* Max capacity: 256 * 32 = 8192 bytes   */

/*******************************************************************************
 * SECTION 8 - Default Alarm Thresholds (startup-safe wide window)
 *
 * These defaults are written to the FPGA alarm peripheral immediately at boot,
 * BEFORE the BME280 calibration is read.  They exist for one purpose: to
 * prevent false alarms during the ~100 ms calibration window.
 *
 * VALUES ARE IN 16-BIT RAW ADC SPACE (= raw_20bit >> 4):
 *   0x7800  ->  approximately 15 C  (cold lower bound)
 *   0x8800  ->  approximately 38 C  (hot upper bound)
 *
 * These are intentionally wide so no normal room temperature can trigger them.
 * Once calibration succeeds, bme280_temp_to_raw16() computes exact values for
 * THIS specific chip and fpga_write_thresholds() overwrites these defaults.
 *
 * HISTORY OF BUG: The original code used 0x1400 / 0x1E00 which correspond to
 * temperatures far below freezing.  The room-temperature sensor reading always
 * exceeded 0x1E00, so ALARM_TEMP_HIGH fired immediately on every boot,
 * causing false SMS messages and a perpetually lit red LED.
 *******************************************************************************/
#define DEFAULT_TEMP_LOW_RAW    0x7800U  /* ~15 C: lower alarm threshold (raw16) */
#define DEFAULT_TEMP_HIGH_RAW   0x8800U  /* ~38 C: upper alarm threshold (raw16) */
#define DEFAULT_LIGHT_THRESH    100U     /* Light alarm if ADC count < 100       */

/*******************************************************************************
 * SECTION 9 - Sensor Data Structure
 *
 * sensor_data_t is a point-in-time snapshot of all sensor readings.
 * It flows through the system as follows:
 *
 *   fpga_read_sensors()  -> fills raw fields + compensated floats
 *        |
 *        v
 *   build_mqtt_payload() -> serializes to JSON string
 *        |
 *        v
 *   esp32_mqtt_publish() -> sends to MQTT broker
 *
 *   handle_alarms() -> checks alarm_flags for transitions
 *        |
 *        +--> ufm_log_event()     -> stores to BRAM event log
 *        +--> sim800l_send_sms()  -> sends SMS alert
 *        +--> esp32_mqtt_publish()-> publishes to alarm topic
 *
 * Field explanations:
 *   temp_raw / press_raw / humid_raw:
 *     Direct 20-bit (or 16-bit for humid) values read from FPGA registers.
 *     NOT human-readable until passed through Bosch compensation formulas.
 *
 *   temperature_c / pressure_hpa / humidity_pct:
 *     Physical values after compensation.  Valid only when bme_valid = true.
 *
 *   pir_status:
 *     Two-bit field: bit0=PIR1, bit1=PIR2.  Set to 1 when motion is detected.
 *     The PIR sensors use a 5-second latch in the FPGA before deactivating.
 *
 *   alarm_flags:
 *     5-bit bitmask mirroring the FPGA ALARM_REG_STATUS register.
 *     Bit values defined by ALARM_TEMP_HIGH, ALARM_TEMP_LOW, etc. above.
 *******************************************************************************/
typedef struct {
    uint32_t temp_raw;       /* 20-bit raw temperature ADC (from FPGA register)  */
    uint32_t press_raw;      /* 20-bit raw pressure ADC (from FPGA register)     */
    uint32_t humid_raw;      /* 16-bit raw humidity ADC (from FPGA register)     */
    uint16_t light_level;    /* Light sensor: MCP3008 CH0, 0-1023               */
    uint16_t heating_level;  /* Heating sensor: MCP3008 CH1, 0-1023             */
    uint16_t sound_level;    /* Sound sensor: MCP3008 CH2, 0-1023               */
    uint8_t  pir_status;     /* PIR bitmask: bit0=PIR1, bit1=PIR2               */
    uint8_t  alarm_flags;    /* 5-bit alarm bitmask from FPGA alarm peripheral   */

    float    temperature_c;  /* Compensated temperature in degrees Celsius       */
    float    pressure_hpa;   /* Compensated pressure in hectopascals             */
    float    humidity_pct;   /* Compensated relative humidity in percent (0-100) */

    bool     bme_valid;      /* true = BME280 FPGA controller has valid data     */
    bool     adc_valid;      /* true = MCP3008 FPGA controller has valid data    */
    time_t   timestamp;      /* Unix time when this snapshot was taken           */
} sensor_data_t;

/*******************************************************************************
 * SECTION 10 - Function Declarations (Prototypes)
 *
 * These allow main_supervisor.c to call functions defined in the other
 * translation units.  The linker resolves actual addresses at link time.
 *******************************************************************************/

/* esp32_handler.c: ESP32 WiFi bridge + MQTT client via AT commands ----------- */
int  esp32_init(void);
  /* Open /dev/ttyS1 at 115200 baud, configure termios, send AT probe */
int  esp32_send_at_command(const char *cmd, const char *expected, int timeout_ms);
  /* Transmit AT command string + \r\n, wait up to timeout_ms for expected reply */
int  esp32_connect_wifi(const char *ssid, const char *password);
  /* AT+CWMODE=1, then AT+CWJAP="ssid","pass" — waits up to 20 s */
int  esp32_mqtt_connect(const char *broker, int port);
  /* AT+MQTTUSERCFG + AT+MQTTCONN — connects to authenticated broker */
int  esp32_mqtt_publish(const char *topic, const char *payload);
  /* AT+MQTTPUBRAW — streams raw JSON bytes after ">" prompt */
void esp32_close(void);
  /* AT+MQTTCLEAN + AT+CWQAP, then close(uart_fd) */

/* sim800l_handler.c: GSM/SMS via direct MMIO on Altera UART in FPGA ---------- */
int  sim800l_init(void);
  /* mmap LW bridge, set baud divisor, wait for SIM boot, configure SMS mode */
int  sim800l_send_at_command(const char *cmd, const char *expected, int timeout_ms);
  /* Write AT command to UART TXDATA register, poll RXDATA for response */
int  sim800l_send_sms(const char *phone_number, const char *message);
  /* AT+CMGS="number" -> wait for ">" -> write message body -> Ctrl+Z (0x1A) */
void sim800l_close(void);
  /* munmap the LW bridge region */

/* ufm_storage.c: FPGA BRAM event log (UFM emulation) ------------------------- */
int  ufm_init(void);
  /* mmap LW bridge, scan BRAM for existing events, find first free slot */
int  ufm_log_event(const char *event_type, const sensor_data_t *data);
  /* Pack sensor_data_t into flash_event_t, write 8 words to BRAM */
void ufm_print_all_events(void);
  /* Scan BRAM from slot 0, print each valid record as a table row to stdout */
void ufm_clear_logs(void);
  /* Write 0x00000000 to all 2048 words (8 KB) to erase all events */
void ufm_close(void);
  /* munmap the LW bridge region */

/*******************************************************************************
 * SECTION 11 - Network and GSM Configuration
 *
 * All credentials must match the settings in panel/.env for the dashboard
 * to receive MQTT messages.  The MQTT broker runs on a VPS (Virtual Private
 * Server) — it is accessible from both the DE10-Nano (via ESP32 WiFi) and
 * the Flask panel (which subscribes from the VPS itself or the internet).
 *
 * MQTT_USE_TLS: 0 = plain TCP (port 1883) — easiest setup, no certificates
 *               1 = TLS (port 8883) — secure, requires ESP-AT firmware v2.x+
 *                   and a valid server certificate (e.g. Let's Encrypt on VPS)
 *******************************************************************************/
#define WIFI_SSID           "SSID"            /* WiFi network name (SSID)     */
#define WIFI_PASSWORD       "PassWORD"          /* WiFi WPA2 passphrase         */
#define MQTT_BROKER         "111.222.333.444"    /* Mosquitto broker VPS IP      */
#define MQTT_PORT           1883                 /* Plain TCP MQTT port          */
#define MQTT_USER           "Username"             /* Mosquitto broker username    */
#define MQTT_PASS           "PassWORD"          /* Mosquitto broker password    */
#define MQTT_USE_TLS        0                    /* 0=plain TCP, 1=TLS           */
#define MQTT_CLIENT_ID      "DE10Nano_SmartHome" /* Unique MQTT client ID        */
#define MQTT_TOPIC_SENSORS  "smarthome/sensors"  /* Topic: periodic readings     */
#define MQTT_TOPIC_ALARMS   "smarthome/alarms"   /* Topic: alarm events          */
#define ALERT_PHONE_NUMBER  "+123456789"      /* SMS destination (E.164 fmt)  */

/*******************************************************************************
 * SECTION 12 - Timing Constants
 *
 * SENSOR_POLL_SEC (1 s):
 *   The main loop sleeps for this duration between sensor reads.  1 second
 *   gives responsive alarm detection without excessive CPU load.
 *
 * MQTT_PUBLISH_INTERVAL (5 cycles):
 *   Publish sensor data every 5th polling cycle = every 5 seconds.
 *   This is fast enough for a live dashboard and keeps MQTT traffic low.
 *   Alarm events are published immediately (not rate-limited).
 *
 * SMS_COOLDOWN_SEC (300 s = 5 min):
 *   Once an SMS is sent, no further SMS is sent for at least 300 seconds
 *   regardless of how many alarm transitions happen in that window.
 *   This prevents flooding the SIM card if a sensor oscillates around a
 *   threshold (e.g. temperature hovering at exactly the alarm level).
 *******************************************************************************/
#define SMS_COOLDOWN_SEC      300  /* Minimum seconds between two SMS messages   */
#define SENSOR_POLL_SEC       1    /* Main loop sleep interval in seconds        */
#define MQTT_PUBLISH_INTERVAL 5    /* Publish MQTT every N sensor poll cycles    */

/*******************************************************************************
 * SECTION 13 - BME280 Pre-loaded Factory Calibration Coefficients
 *
 * Every BME280 chip has unique factory-trimmed coefficients burned into its
 * internal OTP (one-time programmable) memory at the Bosch factory.  These 26
 * coefficients are used by the compensation formulas to convert raw ADC values
 * to physical units.
 *
 * Why are they hard-coded here instead of read at runtime?
 *   The BME280 is connected to FPGA GPIO pins (JP1), not to the HPS I2C bus.
 *   The ARM processor cannot reach the sensor's calibration registers directly.
 *   They were read ONCE using the ESP32 calibration tool (tools/read_bme280_calib)
 *   and are now embedded here permanently for this specific chip.
 *
 * Fast-path logic in bme280_read_calibration():
 *   If BME280_PRECAL_T1 != 0 (true after this chip was read), the function
 *   immediately copies these values into the dig_T1..H6 variables and returns
 *   WITHOUT attempting to open /dev/i2c-1 (which would fail anyway because
 *   the sensor is not wired to the HPS I2C pins).
 *
 * Coefficient types (Bosch BME280 datasheet, Table 17):
 *   T1: unsigned 16-bit,  T2-T3: signed 16-bit
 *   P1: unsigned 16-bit,  P2-P9: signed 16-bit
 *   H1: unsigned 8-bit,   H2: signed 16-bit
 *   H3: unsigned 8-bit,   H4-H5: signed 16-bit,  H6: signed 8-bit
 *******************************************************************************/
#define BME280_PRECAL_T1    28267U  /* dig_T1: uint16 */
#define BME280_PRECAL_T2    26539   /* dig_T2: int16  */
#define BME280_PRECAL_T3    50      /* dig_T3: int16  */
#define BME280_PRECAL_P1    36369U  /* dig_P1: uint16 */
#define BME280_PRECAL_P2    -10642  /* dig_P2: int16  */
#define BME280_PRECAL_P3    3024    /* dig_P3: int16  */
#define BME280_PRECAL_P4    7214    /* dig_P4: int16  */
#define BME280_PRECAL_P5    -35     /* dig_P5: int16  */
#define BME280_PRECAL_P6    -7      /* dig_P6: int16  */
#define BME280_PRECAL_P7    9900    /* dig_P7: int16  */
#define BME280_PRECAL_P8    -10230  /* dig_P8: int16  */
#define BME280_PRECAL_P9    4285    /* dig_P9: int16  */
#define BME280_PRECAL_H1    75U     /* dig_H1: uint8  */
#define BME280_PRECAL_H2    368     /* dig_H2: int16  */
#define BME280_PRECAL_H3    0U      /* dig_H3: uint8  */
#define BME280_PRECAL_H4    304     /* dig_H4: int16  */
#define BME280_PRECAL_H5    50      /* dig_H5: int16  */
#define BME280_PRECAL_H6    30      /* dig_H6: int8   */

/*******************************************************************************
 * SECTION 14 - Logging Macros
 *
 * Three severity levels, always print to terminal with a bracketed tag:
 *
 *   LOG_INFO(fmt, ...)  -> stdout  "[INFO]  message\n"  — normal status
 *   LOG_WARN(fmt, ...)  -> stdout  "[WARN]  message\n"  — degraded but running
 *   LOG_ERROR(fmt, ...) -> stderr  "[ERROR] message\n"  — fatal, may stop
 *
 * The ##__VA_ARGS__ syntax handles zero variadic arguments:
 *   LOG_INFO("Ready.");          // OK — no extra args
 *   LOG_INFO("Temp: %.1f", t);  // OK — one extra arg
 * Without ##, LOG_INFO("Ready.") would expand to printf("[INFO] " "Ready." "\n", )
 * with a trailing comma before empty args, which is a compile error in C99.
 *******************************************************************************/
#define LOG_INFO(fmt, ...)  printf("[INFO]  " fmt "\n", ##__VA_ARGS__)
#define LOG_WARN(fmt, ...)  printf("[WARN]  " fmt "\n", ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) fprintf(stderr, "[ERROR] " fmt "\n", ##__VA_ARGS__)

#endif /* SMART_HOME_H */
