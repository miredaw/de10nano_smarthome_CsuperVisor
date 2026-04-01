/*******************************************************************************
 * ufm_storage.c — FPGA On-Chip BRAM Event Logger (UFM Emulation)
 *
 * ============================================================================
 * BIG PICTURE — WHAT THIS FILE DOES
 * ============================================================================
 *
 * This module implements a persistent event log using FPGA Block RAM (BRAM).
 * It emulates a UFM (User Flash Memory) logger: events are written to a fixed
 * address region in the FPGA fabric, survive HPS resets, and can be read back
 * at any time.
 *
 * ============================================================================
 * WHERE IS THE LOG STORED?
 * ============================================================================
 *
 * Inside the Cyclone V FPGA as an "altera_avalon_onchip_memory2" IP block.
 *
 * Physical address:  0xFF202000
 *   = LW bridge base (0xFF200000) + Avalon-MM offset (0x2000)
 *
 * Size: 8,192 bytes = 256 records x 32 bytes per record.
 *
 * This is NOT the SD card, NOT the filesystem, NOT NAND flash.
 * It is on-chip SRAM flip-flops inside the FPGA — extremely fast (single
 * clock cycle read/write at 50 MHz), but volatile (cleared on FPGA reconfig).
 *
 * ============================================================================
 * HOW IS DATA WRITTEN?
 * ============================================================================
 *
 * After mmap(/dev/mem) maps the LW bridge, writing to the BRAM is as simple as:
 *
 *   *(volatile uint32_t*)(lw_base + 0x2000 + word_offset * 4) = value;
 *
 * The Avalon-MM interconnect translates this ARM store instruction into an
 * Avalon write transaction that updates the BRAM directly.  No protocol,
 * no command register, no busy-wait — it completes in one clock cycle.
 *
 * Each event record (32 bytes = 8 words) is written sequentially:
 *   flash_write_word(base + 0), flash_write_word(base + 1), ... x8
 *
 * ============================================================================
 * HOW IS DATA READ BACK?
 * ============================================================================
 *
 * METHOD A — C code (call from this running process):
 *   ufm_print_all_events()
 *   Scans from word 0 upward.  For each 8-word record:
 *     - Read word 0 (magic).
 *     - If magic == 0xCAFEBABE: valid record — read remaining 7 words,
 *       reconstruct the flash_event_t struct, print as a table row.
 *     - If magic == 0x00000000: end of written data — stop scan.
 *     - Any other value: corrupted/skipped entry.
 *
 * METHOD B — Linux shell (devmem2 utility):
 *   # Read word 0 of event 0 (should be 0xCAFEBABE if an event was logged):
 *   devmem2 0xFF202000 w
 *
 *   # Read word 1 of event 0 (Unix timestamp):
 *   devmem2 0xFF202004 w
 *
 *   # Read word 2 of event 0 (event_code + temp_x100):
 *   devmem2 0xFF202008 w
 *
 *   # Dump all 8 KB to a binary file (then use hexdump):
 *   dd if=/dev/mem of=ufm.bin bs=1 count=8192 skip=$((0xFF202000))
 *   hexdump -C ufm.bin | head -40
 *
 * METHOD C — Python script:
 *   import struct, mmap, os
 *   with open('/dev/mem', 'rb') as f:
 *       mm = mmap.mmap(f.fileno(), 0x2000, offset=0xFF202000)
 *       magic = struct.unpack('<I', mm.read(4))[0]
 *       print(hex(magic))  # Should be 0xCAFEBABE
 *
 * ============================================================================
 * EVENT RECORD FORMAT (32 bytes = 8 x 32-bit words, __attribute__((packed)))
 * ============================================================================
 *
 *   Byte offset  Size  Field         Description
 *   +0x00         4    magic         0xCAFEBABE = valid, 0x00000000 = free slot
 *   +0x04         4    timestamp     Unix time (seconds since 1970-01-01 00:00:00)
 *   +0x08         2    event_code    0x0001=TEMP_HIGH, 0x0002=TEMP_LOW,
 *                                    0x0003=MOTION,    0x0004=LIGHT_LOW
 *   +0x0A         2    temp_x100     temperature_c * 100 as int16
 *                                    e.g. 2350 means 23.50 degC, -500 means -5.00 degC
 *   +0x0C         4    press_x100    pressure_hpa * 100 as uint32
 *                                    e.g. 101325 means 1013.25 hPa
 *   +0x10         4    humid_x100    humidity_pct * 100 as uint32
 *                                    e.g. 5500 means 55.00 %RH
 *   +0x14         2    light         MCP3008 CH0 ADC value (0-1023)
 *   +0x16         2    heating       MCP3008 CH1 ADC value (0-1023)
 *   +0x18         2    sound         MCP3008 CH2 ADC value (0-1023)
 *   +0x1A         1    alarm_flags   5-bit bitmask of active alarm conditions
 *   +0x1B         5    pad           Padding zeros to reach exactly 32 bytes
 *
 * Why store floats as integers?
 *   BRAM stores raw bits.  Floating-point representation can vary between
 *   compiler versions and ABI settings.  Storing temp*100 as int16 is:
 *   - Portable: no float ABI dependence
 *   - Reversible: divide by 100.0f to recover the original value
 *   - Compact: int16 is sufficient for temperature (-327.68 to +327.67 degC)
 *
 * ============================================================================
 * STORAGE LAYOUT
 * ============================================================================
 *
 *   Address (from 0xFF202000)  Event  Words
 *   +0x0000 .. +0x001F          0      0-7
 *   +0x0020 .. +0x003F          1      8-15
 *   +0x0040 .. +0x005F          2      16-23
 *   ...
 *   +0x1FE0 .. +0x1FFF         255    2040-2047  (last slot)
 *
 * ============================================================================
 * PERSISTENCE
 * ============================================================================
 *
 *   YES: Survives HPS soft-reset (kernel panic, watchdog reboot, "reboot" cmd)
 *   YES: Survives HPS power-off if FPGA remains configured (e.g. battery-backed)
 *   NO:  Lost when FPGA is reprogrammed (new .sof loaded via JTAG or .rbf boot)
 *   NO:  Lost on full board power-off (BRAM = volatile SRAM flip-flops)
 *
 * ============================================================================
 * HOW TO ERASE ALL LOGS
 * ============================================================================
 *
 *   Option A — standalone binary:   sudo ./ufm_erase
 *   Option B — from C code:         ufm_clear_logs()
 *   Both write 0x00000000 to all 2048 words, restoring BRAM to its cleared
 *   state.  After erasing, ufm_init() will find no events and set
 *   write_word_idx = 0, event_count = 0.
 *
 *******************************************************************************/

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <stdio.h>      /* printf(), snprintf()                                  */
#include <stdlib.h>     /* NULL                                                  */
#include <stdint.h>     /* uint8_t, uint16_t, uint32_t, int16_t, int32_t        */
#include <string.h>     /* memset(), memcpy(), strcmp()                          */
#include <unistd.h>     /* close()                                               */
#include <fcntl.h>      /* open(), O_RDWR, O_SYNC                                */
#include <time.h>       /* time_t, localtime(), strftime()                       */
#include <assert.h>     /* _Static_assert (compile-time struct size check)       */
#include <sys/mman.h>   /* mmap(), munmap(), MAP_SHARED                          */

#include "smart_home.h" /* LW_BRIDGE_BASE/SPAN, FLASH_DATA_BASE, LOG_* macros   */

/*******************************************************************************
 * flash_event_t — The On-Disk Record Structure
 *
 * This struct is the exact layout of one 32-byte record in BRAM.
 * __attribute__((packed)) prevents the compiler from inserting padding bytes
 * between fields to satisfy alignment requirements.  Without this, the struct
 * would be larger than 32 bytes and the word-by-word write loop would be wrong.
 *
 * The _Static_assert below is a compile-time check that verifies the struct
 * is EXACTLY 32 bytes.  If a field is wrong type or padding sneaks in,
 * the build fails with a clear error message.
 *
 * Field details:
 *   magic      : 0xCAFEBABE for a valid record, 0x00000000 for a free/erased slot.
 *                The scan algorithm uses this to find the first free slot and
 *                distinguish valid records from cleared memory.
 *   timestamp  : uint32_t Unix time.  Sufficient until year 2106.
 *   event_code : uint16_t.  Identifies which alarm triggered: TEMP_HIGH, MOTION, etc.
 *   temp_x100  : int16_t.  Temperature in 0.01 degC units.
 *                Signed to handle temperatures below 0 (e.g. -5.00 C = -500).
 *   press_x100 : uint32_t.  Pressure in 0.01 hPa units.
 *                Sea-level pressure ~101325 Pa = 1013.25 hPa -> stored as 101325.
 *   humid_x100 : uint32_t.  Humidity in 0.01 %RH units.  Range: 0-10000.
 *   light      : uint16_t.  MCP3008 CH0 ADC value, 0-1023.
 *   heating    : uint16_t.  MCP3008 CH1 ADC value, 0-1023.
 *   sound      : uint16_t.  MCP3008 CH2 ADC value, 0-1023.
 *   alarm_flags: uint8_t.   5-bit bitmask of all active alarms at event time.
 *   pad[5]     : uint8_t[5]. Five zero-bytes to reach 32 bytes total.
 *******************************************************************************/
typedef struct {
    uint32_t magic;          /* +0x00  4 B  : 0xCAFEBABE=valid, 0x00=free    */
    uint32_t timestamp;      /* +0x04  4 B  : Unix timestamp (seconds)       */
    uint16_t event_code;     /* +0x08  2 B  : EVENT_CODE_* constant          */
    int16_t  temp_x100;      /* +0x0A  2 B  : temperature_c * 100 (int16)    */
    uint32_t press_x100;     /* +0x0C  4 B  : pressure_hpa * 100 (uint32)    */
    uint32_t humid_x100;     /* +0x10  4 B  : humidity_pct * 100 (uint32)    */
    uint16_t light;          /* +0x14  2 B  : light ADC 0-1023               */
    uint16_t heating;        /* +0x16  2 B  : heating ADC 0-1023             */
    uint16_t sound;          /* +0x18  2 B  : sound ADC 0-1023               */
    uint8_t  alarm_flags;    /* +0x1A  1 B  : 5-bit alarm bitmask            */
    uint8_t  pad[5];         /* +0x1B  5 B  : padding to 32 bytes total      */
} __attribute__((packed)) flash_event_t;

/* Compile-time assertion: if this fails, the struct is not exactly 32 bytes.
 * Fix by checking field types and the pad[] array size. */
_Static_assert(sizeof(flash_event_t) == 32,
               "flash_event_t must be exactly 32 bytes — check field types and padding");

/* Event type code constants (stored in flash_event_t.event_code) */
#define EVENT_TEMP_HIGH   0x0001U  /* Temperature above upper threshold */
#define EVENT_TEMP_LOW    0x0002U  /* Temperature below lower threshold */
#define EVENT_MOTION      0x0003U  /* PIR motion sensor triggered       */
#define EVENT_LIGHT_LOW   0x0004U  /* Light level below threshold       */
#define EVENT_UNKNOWN     0xFFFFU  /* Unrecognized event type string    */

/* Words per event record: 32 bytes / 4 bytes per word = 8 words */
#define WORDS_PER_EVENT   (FLASH_BYTES_PER_EVENT / 4U)   /* = 8 */

/* Magic value identifying a valid written event record.
 * Chosen to be extremely unlikely to appear in uninitialized/cleared memory. */
#define FLASH_MAGIC       0xCAFEBABEUL  /* Written at word 0 of every valid record */
#define FLASH_ERASED      0x00000000UL  /* BRAM power-on default = "free slot"     */

/*******************************************************************************
 * MODULE-PRIVATE STATE
 *
 * These three variables track the current log state and persist for the
 * lifetime of the process.
 *******************************************************************************/
static volatile uint8_t *ufm_lw_base   = NULL; /* mmap'd LW bridge base pointer */
static uint32_t          write_word_idx = 0;    /* Word index of next free slot  */
static int               event_count   = 0;    /* Number of valid stored events */

/*******************************************************************************
 * LOW-LEVEL REGISTER HELPERS
 *
 * flash_rd / flash_wr translate a BYTE OFFSET from the LW bridge base into
 * a 32-bit BRAM register access.  The byte offset includes FLASH_DATA_BASE
 * (0x2000) plus the word index * 4.
 *
 * Example: flash_rd(FLASH_DATA_BASE + 0 * 4) reads word 0 of event 0,
 *          which is at physical address 0xFF200000 + 0x2000 + 0 = 0xFF202000.
 *******************************************************************************/
static inline uint32_t flash_rd(uint32_t byte_offset)
{
    /* Read a 32-bit word from BRAM at the given byte offset from lw_base */
    return *(volatile uint32_t *)(ufm_lw_base + byte_offset);
}

static inline void flash_wr(uint32_t byte_offset, uint32_t value)
{
    /* Write a 32-bit word to BRAM at the given byte offset from lw_base */
    *(volatile uint32_t *)(ufm_lw_base + byte_offset) = value;
}

/*******************************************************************************
 * flash_erase_sector — Clear All 8 KB of BRAM to 0x00000000
 *
 * Iterates through all 2048 words (256 events * 8 words/event) and writes
 * FLASH_ERASED (0x00000000) to each.  This restores the BRAM to its power-on
 * state where every word reads as zero.
 *
 * After erase:
 *   - The scan in ufm_init() will see word 0 = 0x00000000 immediately
 *     and stop (first slot is free).
 *   - write_word_idx is reset to 0, event_count to 0 by the caller.
 *
 * This is not a true hardware "flash erase" — BRAM supports direct overwrite
 * without an erase cycle.  The "erase" here simply writes zeros to mimic the
 * erased state of real flash/UFM memory.
 *******************************************************************************/
static int flash_erase_sector(void)
{
    /* Total words in the BRAM log region: 256 events * 8 words = 2048 words */
    uint32_t total_words = FLASH_MAX_EVENTS * WORDS_PER_EVENT;

    LOG_INFO("UFM: erasing %u words (%u bytes) at LW offset 0x%04lX...",
             total_words, total_words * 4U, (unsigned long)FLASH_DATA_BASE);

    /* Write 0x00000000 to every word — loop through all 2048 words */
    for (uint32_t w = 0; w < total_words; w++)
        /* FLASH_DATA_BASE + w*4: byte address of word w from the LW bridge base
         * e.g. w=0 -> offset 0x2000, w=1 -> offset 0x2004, ..., w=2047 -> 0x3FFC */
        flash_wr(FLASH_DATA_BASE + w * 4U, FLASH_ERASED);

    LOG_INFO("UFM: erase complete — all %u words set to 0x00000000", total_words);
    return 0;
}

/*******************************************************************************
 * flash_write_word — Write One 32-bit Word to BRAM
 *
 * word_offset: position within the log data area (0 = first word of event 0,
 *              1 = second word of event 0, 8 = first word of event 1, etc.)
 *
 * Byte address calculation:
 *   FLASH_DATA_BASE + word_offset * 4
 *   = 0x2000 + word_offset * 4  (from LW bridge base)
 *
 * The Altera altera_avalon_onchip_memory2 IP accepts direct Avalon writes
 * with zero latency — no command register or busy-poll needed.
 *******************************************************************************/
static int flash_write_word(uint32_t word_offset, uint32_t value)
{
    /* Convert word index to byte offset and write the 32-bit value to BRAM */
    flash_wr(FLASH_DATA_BASE + word_offset * 4U, value);
    return 0;  /* Always succeeds for BRAM (no write-failure mechanism) */
}

/*******************************************************************************
 * flash_read_word — Read One 32-bit Word from BRAM
 *
 * Mirrors flash_write_word: converts word index to byte offset and reads.
 *******************************************************************************/
static uint32_t flash_read_word(uint32_t word_offset)
{
    /* Convert word index to byte offset and read the 32-bit value from BRAM */
    return flash_rd(FLASH_DATA_BASE + word_offset * 4U);
}

/*******************************************************************************
 * event_code_to_str — Map Event Code to Human-Readable String
 *
 * Used when printing the event log table (ufm_print_all_events).
 *******************************************************************************/
static const char *event_code_to_str(uint16_t code)
{
    switch (code) {
        case EVENT_TEMP_HIGH: return "TEMP_HIGH";   /* 0x0001 */
        case EVENT_TEMP_LOW:  return "TEMP_LOW";    /* 0x0002 */
        case EVENT_MOTION:    return "MOTION";      /* 0x0003 */
        case EVENT_LIGHT_LOW: return "LIGHT_LOW";   /* 0x0004 */
        default:              return "UNKNOWN";     /* 0xFFFF or anything else */
    }
}

/*******************************************************************************
 * str_to_event_code — Map Event Type String to Code
 *
 * Called by ufm_log_event() to convert the string-based event_type argument
 * (e.g. "TEMP_HIGH") into the corresponding uint16_t code for storage.
 *******************************************************************************/
static uint16_t str_to_event_code(const char *event_type)
{
    if (strcmp(event_type, "TEMP_HIGH")  == 0) return EVENT_TEMP_HIGH;
    if (strcmp(event_type, "TEMP_LOW")   == 0) return EVENT_TEMP_LOW;
    if (strcmp(event_type, "MOTION")     == 0) return EVENT_MOTION;
    if (strcmp(event_type, "LIGHT_LOW")  == 0) return EVENT_LIGHT_LOW;
    return EVENT_UNKNOWN;  /* Unrecognized type string */
}

/*******************************************************************************
 * ufm_init — Map LW Bridge and Scan BRAM for Existing Events
 *
 * WHAT THIS FUNCTION DOES:
 *   1. Opens /dev/mem and mmap's the LW bridge (2 MB window at 0xFF200000).
 *      The BRAM is at offset 0x2000 within this window.
 *   2. Reads word 0 of the BRAM to determine the initial state:
 *      a) 0xCAFEBABE: at least one event was previously written.
 *         Scan forward to count events and find the first free slot.
 *      b) 0x00000000: BRAM is in erased (cleared) state.
 *         Set write_word_idx = 0, event_count = 0.  Ready to log immediately.
 *      c) Anything else: unexpected data (first boot after FPGA config change?).
 *         Perform a full erase to restore known state, then set pointers to 0.
 *   3. After scanning, write_word_idx points to the first free word slot.
 *
 * SCAN ALGORITHM:
 *   Walk from event 0 to event 255, reading the FIRST word (magic) of each:
 *   - 0xCAFEBABE: valid event, increment event_count, advance write_word_idx.
 *   - 0x00000000: free slot, stop — this is where the next write goes.
 *   - Other:      corrupted/skipped entry, continue.
 *
 * Returns 0 on success, -1 if mmap fails.
 *******************************************************************************/
int ufm_init(void)
{
    /* Open the physical memory character device */
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        perror("UFM: open /dev/mem");
        return -1;
    }

    /* Map the full 2 MB LW bridge window.  The BRAM is at byte offset 0x2000
     * within this window, at physical address 0xFF202000. */
    void *vbase = mmap(NULL, LW_BRIDGE_SPAN,
                       PROT_READ | PROT_WRITE, MAP_SHARED,
                       fd, (off_t)LW_BRIDGE_BASE);

    close(fd);  /* fd is no longer needed once mmap establishes the mapping */

    if (vbase == MAP_FAILED) {
        perror("UFM: mmap LW bridge");
        return -1;
    }

    ufm_lw_base = (volatile uint8_t *)vbase;  /* Store byte pointer */
    LOG_INFO("UFM: LW bridge mapped — BRAM base at LW+0x%03lX (physical 0x%08lX)",
             (unsigned long)FLASH_DATA_BASE,
             (unsigned long)(LW_BRIDGE_BASE + FLASH_DATA_BASE));

    /* Read word 0 (first word of event 0's magic field) to determine state */
    uint32_t first_word = flash_read_word(0);

    if (first_word != FLASH_MAGIC && first_word != FLASH_ERASED) {
        /* Unexpected value — neither a valid magic nor a cleared zero.
         * This can happen if the FPGA was reconfigured with a different IP
         * at this address, or if the BRAM content was corrupted.
         * Safe response: erase and start fresh. */
        LOG_WARN("UFM: unexpected word 0 (0x%08X) — erasing BRAM for fresh start",
                 first_word);
        flash_erase_sector();  /* Write zeros to all 8 KB */
        write_word_idx = 0;    /* Next write goes to word 0 (event 0) */
        event_count    = 0;    /* No valid events */
        LOG_INFO("UFM: initialised (0 existing events after erase)");
        return 0;
    }

    /* Normal scan: walk through all 256 possible event slots */
    event_count    = 0;   /* Reset count before scanning */
    write_word_idx = 0;   /* Will be updated to the first free slot */

    for (uint32_t i = 0; i < FLASH_MAX_EVENTS; i++) {
        /* Each event occupies WORDS_PER_EVENT (8) consecutive words.
         * word_base is the word index of this event's first word (magic field). */
        uint32_t word_base = i * WORDS_PER_EVENT;
        uint32_t magic = flash_read_word(word_base);  /* Read magic field */

        if (magic == FLASH_ERASED) {
            /* This slot is free (cleared to 0x00000000).
             * All subsequent slots must also be free (we write sequentially).
             * Stop here — this is where the next event will be written. */
            write_word_idx = word_base;  /* Point to start of this free slot */
            break;
        }

        if (magic == FLASH_MAGIC) {
            /* Valid event record found */
            event_count++;                               /* Count it */
            write_word_idx = (i + 1U) * WORDS_PER_EVENT; /* Next slot = event i+1 */
        }
        /* Any other magic value: corrupted entry, skip silently */
    }

    LOG_INFO("UFM: initialised — %d existing events, next write at word %u (byte 0x%04X)",
             event_count, write_word_idx, write_word_idx * 4U);
    return 0;
}

/*******************************************************************************
 * ufm_log_event — Pack Sensor Data into a Record and Write to BRAM
 *
 * WHAT HAPPENS STEP BY STEP:
 *
 *   1. Validate: check ufm_lw_base != NULL and event_count < FLASH_MAX_EVENTS.
 *
 *   2. Pack the event into a flash_event_t struct:
 *      - magic      = 0xCAFEBABE
 *      - timestamp  = current Unix time (from data->timestamp)
 *      - event_code = str_to_event_code(event_type)
 *      - temp_x100  = (int16_t)(data->temperature_c * 100.0f)
 *        e.g. 23.45 degC -> 2345, -5.00 degC -> -500
 *      - press_x100 = (uint32_t)(data->pressure_hpa * 100.0f)
 *        e.g. 1013.25 hPa -> 101325
 *      - humid_x100 = (uint32_t)(data->humidity_pct * 100.0f)
 *        e.g. 55.00 %RH -> 5500
 *      - light, heating, sound: direct copies from sensor_data_t
 *      - alarm_flags: direct copy from sensor_data_t
 *      - pad[5]: zero (set by memset)
 *
 *   3. Copy the packed struct into a uint32_t words[8] array using memcpy().
 *      This gives us the 8 words to write without any struct pointer aliasing.
 *
 *   4. Write the 8 words to BRAM using flash_write_word() in order (word 0 first).
 *      Word 0 is the magic — written LAST to make the record "appear" atomically
 *      ... actually it's written FIRST here (index write_word_idx + 0).
 *      NOTE: BRAM does not have atomicity issues since the HPS is the only master.
 *
 *   5. Advance write_word_idx by WORDS_PER_EVENT (8) and increment event_count.
 *
 * Returns 0 on success, -1 if BRAM is full or not initialized.
 *******************************************************************************/
int ufm_log_event(const char *event_type, const sensor_data_t *data)
{
    if (ufm_lw_base == NULL) {
        LOG_WARN("UFM not initialised — skipping event log for '%s'", event_type);
        return -1;
    }

    /* Check capacity: if all 256 slots are used, the log is full */
    if (event_count >= (int)FLASH_MAX_EVENTS) {
        LOG_WARN("UFM full (%d/%u events) — event '%s' not logged",
                 event_count, FLASH_MAX_EVENTS, event_type);
        return -1;
    }

    /* ---- Pack the event into the fixed-size struct ----------------------- */
    flash_event_t rec;
    memset(&rec, 0x00, sizeof(rec));  /* Zero all fields including pad[] */

    rec.magic      = FLASH_MAGIC;                        /* 0xCAFEBABE */
    rec.timestamp  = (uint32_t)data->timestamp;          /* Unix time   */
    rec.event_code = str_to_event_code(event_type);      /* e.g. 0x0001 */

    /* Scale floating-point values to integers for portable BRAM storage */
    rec.temp_x100  = (int16_t)(data->temperature_c  * 100.0f); /* degC*100 as int16 */
    rec.press_x100 = (uint32_t)(data->pressure_hpa  * 100.0f); /* hPa*100 as uint32 */
    rec.humid_x100 = (uint32_t)(data->humidity_pct  * 100.0f); /* %RH*100 as uint32 */

    /* Copy ADC values directly — already in integer form */
    rec.light      = data->light_level;
    rec.heating    = data->heating_level;
    rec.sound      = data->sound_level;
    rec.alarm_flags= data->alarm_flags;
    /* pad[5] remains 0x00 from the memset above */

    /* ---- Copy struct to a word array for sequential writes --------------- */
    /* memcpy is required here to avoid strict aliasing undefined behavior.
     * Casting a struct pointer to uint32_t* would violate C aliasing rules. */
    uint32_t words[WORDS_PER_EVENT];                        /* 8 x 4 = 32 bytes */
    memcpy(words, &rec, sizeof(rec));  /* Copy packed struct bytes to word array */

    /* ---- Write 8 consecutive 32-bit words to BRAM ------------------------ */
    /* word 0 (words[0]) contains the magic 0xCAFEBABE.
     * All 8 words are written at consecutive word addresses starting at
     * write_word_idx.  The Avalon-MM interconnect handles each as a separate
     * 32-bit write transaction. */
    for (uint32_t w = 0; w < WORDS_PER_EVENT; w++)
        flash_write_word(write_word_idx + w, words[w]);

    /* ---- Update state pointers ------------------------------------------- */
    write_word_idx += WORDS_PER_EVENT;  /* Advance to next free slot (8 words later) */
    event_count++;                      /* Increment stored event count */

    /* Format a human-readable timestamp for the log line */
    char ts_str[32];
    struct tm *tm_info = localtime(&data->timestamp);
    strftime(ts_str, sizeof(ts_str), "%Y-%m-%dT%H:%M:%S", tm_info);

    LOG_INFO("Event logged [%d/%u]: %s @ %s (temp=%.2f C, alarms=0x%02X)",
             event_count, FLASH_MAX_EVENTS, event_type, ts_str,
             data->temperature_c, data->alarm_flags);
    return 0;
}

/*******************************************************************************
 * ufm_print_all_events — Scan BRAM and Print a Formatted Event Table
 *
 * SCAN ALGORITHM (matches ufm_init's scan):
 *   Walk from event 0 to event 255.
 *   Read word 0 (magic) of each event's 8-word record:
 *   - FLASH_ERASED (0x00000000): end of log — break.
 *   - FLASH_MAGIC  (0xCAFEBABE): valid record — read remaining 7 words,
 *     reconstruct the struct with memcpy(), format and print each field.
 *   - Other: corrupted entry — skip with continue.
 *
 * TABLE COLUMNS:
 *   Timestamp  : ISO 8601 format (YYYY-MM-DDTHH:MM:SS), from localtime()
 *   Event      : human-readable type string (TEMP_HIGH, MOTION, etc.)
 *   T(C)       : temperature in degC (temp_x100 / 100.0)
 *   P(hPa)     : pressure in hPa (press_x100 / 100.0)
 *   H(%)       : humidity in %RH (humid_x100 / 100.0)
 *   Light      : raw ADC value (0-1023)
 *   Heat       : raw ADC value (0-1023)
 *   Sound      : raw ADC value (0-1023)
 *   Alarms     : alarm bitmask in hex (0x00-0x1F)
 *******************************************************************************/
void ufm_print_all_events(void)
{
    if (ufm_lw_base == NULL) {
        LOG_WARN("UFM not initialised — cannot print events");
        return;
    }

    /* Print table header */
    printf("\n=== UFM Event Log (%d events) ===\n", event_count);
    printf("%-22s %-10s %7s %8s %7s %6s %6s %6s %8s\n",
           "Timestamp", "Event", "T(C)", "P(hPa)", "H(%)",
           "Light", "Heat", "Sound", "Alarms");
    /* Print separator line */
    printf("%-22s %-10s %7s %8s %7s %6s %6s %6s %8s\n",
           "----------------------", "----------",
           "-------", "--------", "-------",
           "------", "------", "------", "--------");

    int found = 0;  /* Count of valid records actually printed */

    for (uint32_t i = 0; i < FLASH_MAX_EVENTS; i++) {
        uint32_t word_base = i * WORDS_PER_EVENT;  /* Word index of this event's magic */
        uint32_t magic = flash_read_word(word_base);

        if (magic == FLASH_ERASED)
            break;  /* First zero-word = end of written data: no more events after this */

        if (magic != FLASH_MAGIC)
            continue;  /* Not a valid magic and not cleared: skip corrupted slot */

        /* ---- Read all 8 words of this valid event ----------------------- */
        uint32_t words[WORDS_PER_EVENT];
        words[0] = magic;  /* Already read above */
        for (uint32_t w = 1; w < WORDS_PER_EVENT; w++)
            words[w] = flash_read_word(word_base + w);  /* Read words 1-7 */

        /* ---- Reconstruct the struct from the raw words ------------------ */
        flash_event_t rec;
        memcpy(&rec, words, sizeof(rec));  /* Copy 32 bytes into the struct */

        /* ---- Format the Unix timestamp to human-readable string --------- */
        char ts_str[32];
        time_t t = (time_t)rec.timestamp;
        struct tm *tm_info = localtime(&t);
        strftime(ts_str, sizeof(ts_str), "%Y-%m-%dT%H:%M:%S", tm_info);

        /* ---- Print one table row ---------------------------------------- */
        printf("%-22s %-10s %7.2f %8.2f %7.2f %6u %6u %6u   0x%02X\n",
               ts_str,                                    /* Timestamp string  */
               event_code_to_str(rec.event_code),         /* Event type name   */
               (float)rec.temp_x100  / 100.0f,            /* degC (2 decimals) */
               (float)rec.press_x100 / 100.0f,            /* hPa  (2 decimals) */
               (float)rec.humid_x100 / 100.0f,            /* %RH  (2 decimals) */
               (unsigned)rec.light,                       /* ADC 0-1023        */
               (unsigned)rec.heating,                     /* ADC 0-1023        */
               (unsigned)rec.sound,                       /* ADC 0-1023        */
               (unsigned)rec.alarm_flags);                /* Alarm bitmask hex */
        found++;
    }

    printf("Total: %d valid events found\n\n", found);
}

/*******************************************************************************
 * ufm_clear_logs — Erase All Events from BRAM
 *
 * Calls flash_erase_sector() to write 0x00000000 to all 2048 words, then
 * resets the write pointer and event counter to 0.
 *
 * After this call, the BRAM is in the same state as immediately after FPGA
 * configuration (power-on default for BRAM = all zeros).
 * The next call to ufm_log_event() will write event 0 at word 0.
 *******************************************************************************/
void ufm_clear_logs(void)
{
    if (ufm_lw_base == NULL) {
        LOG_WARN("UFM not initialised — cannot clear logs");
        return;
    }

    LOG_INFO("UFM: clearing all %d events...", event_count);
    flash_erase_sector();  /* Write zeros to all 8 KB of BRAM log area */
    write_word_idx = 0;    /* Reset write pointer to start of log area  */
    event_count    = 0;    /* Reset event counter to zero               */
    LOG_INFO("UFM: cleared — log is empty");
}

/*******************************************************************************
 * ufm_close — Unmap the LW Bridge Virtual Memory Mapping
 *
 * Releases the 2 MB virtual address window mapped in ufm_init().
 * Sets ufm_lw_base to NULL to prevent any further BRAM accesses.
 * Prints the total number of events logged during this session.
 *******************************************************************************/
void ufm_close(void)
{
    if (ufm_lw_base != NULL) {
        LOG_INFO("UFM closed — %d total events in log", event_count);
        munmap((void *)ufm_lw_base, LW_BRIDGE_SPAN);  /* Release virtual mapping */
        ufm_lw_base = NULL;  /* Prevent dangling pointer use */
    }
}
