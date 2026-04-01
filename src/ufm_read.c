/*******************************************************************************
 * ufm_read.c — Standalone UFM/BRAM Event Log Reader
 *
 * Reads and pretty-prints all event records stored in the FPGA on-chip BRAM
 * (the "UFM" event log).  No dependency on smart_home.h — all constants are
 * embedded here so this file can be compiled and run independently.
 *
 * Compile:
 *   gcc -O2 -Wall -o ufm_read ufm_read.c
 *
 * Run (root required for /dev/mem):
 *   sudo ./ufm_read
 *
 * Optional flag:
 *   sudo ./ufm_read --all      Print ALL 256 slots (including free/corrupt ones)
 *   sudo ./ufm_read --hex      Also show the raw 8-word hex dump for each record
 *
 *******************************************************************************/

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>

/* ── Address constants ─────────────────────────────────────────────────────── */
#define LW_BRIDGE_BASE   0xFF200000UL  /* Lightweight HPS-to-FPGA AXI bridge   */
#define LW_BRIDGE_SPAN   0x00200000UL  /* 2 MB window                          */
#define FLASH_DATA_BASE  0x2000UL      /* BRAM offset inside LW bridge         */

/* ── Log geometry ──────────────────────────────────────────────────────────── */
#define BYTES_PER_EVENT  32U
#define MAX_EVENTS       256U
#define TOTAL_BRAM_BYTES (BYTES_PER_EVENT * MAX_EVENTS)  /* 8192 bytes         */

/* ── Record magic ──────────────────────────────────────────────────────────── */
#define MAGIC_VALID  0xCAFEBABEUL
#define MAGIC_FREE   0x00000000UL

/* ── Event codes ───────────────────────────────────────────────────────────── */
#define EVENT_TEMP_HIGH  0x0001U
#define EVENT_TEMP_LOW   0x0002U
#define EVENT_MOTION     0x0003U
#define EVENT_LIGHT_LOW  0x0004U

/* ── Alarm flag bits (alarm_flags field) ───────────────────────────────────── */
#define ALARM_TEMP_HIGH  (1U << 0)
#define ALARM_TEMP_LOW   (1U << 1)
#define ALARM_LIGHT_LOW  (1U << 2)
#define ALARM_MOTION     (1U << 3)

/* ── Record struct (must match ufm_storage.c exactly — 32 bytes packed) ────── */
typedef struct {
    uint32_t magic;        /* +0x00  4 B */
    uint32_t timestamp;    /* +0x04  4 B */
    uint16_t event_code;   /* +0x08  2 B */
    int16_t  temp_x100;    /* +0x0A  2 B */
    uint32_t press_x100;   /* +0x0C  4 B */
    uint32_t humid_x100;   /* +0x10  4 B */
    uint16_t light;        /* +0x14  2 B */
    uint16_t heating;      /* +0x16  2 B */
    uint16_t sound;        /* +0x18  2 B */
    uint8_t  alarm_flags;  /* +0x1A  1 B */
    uint8_t  pad[5];       /* +0x1B  5 B */
} __attribute__((packed)) flash_event_t;

_Static_assert(sizeof(flash_event_t) == 32, "flash_event_t must be exactly 32 bytes");

/* ── Helpers ───────────────────────────────────────────────────────────────── */

static const char *event_code_str(uint16_t code)
{
    switch (code) {
        case EVENT_TEMP_HIGH: return "TEMP_HIGH";
        case EVENT_TEMP_LOW:  return "TEMP_LOW ";
        case EVENT_MOTION:    return "MOTION   ";
        case EVENT_LIGHT_LOW: return "LIGHT_LOW";
        default:              return "UNKNOWN  ";
    }
}

/* Build a short string like "T+ L M" listing all active alarm flags. */
static void alarm_flags_str(uint8_t flags, char *buf, size_t bufsz)
{
    buf[0] = '\0';
    if (!flags) {
        snprintf(buf, bufsz, "(none)");
        return;
    }
    if (flags & ALARM_TEMP_HIGH) strncat(buf, "TEMP_HIGH ", bufsz - strlen(buf) - 1);
    if (flags & ALARM_TEMP_LOW)  strncat(buf, "TEMP_LOW ",  bufsz - strlen(buf) - 1);
    if (flags & ALARM_LIGHT_LOW) strncat(buf, "LIGHT_LOW ", bufsz - strlen(buf) - 1);
    if (flags & ALARM_MOTION)    strncat(buf, "MOTION ",    bufsz - strlen(buf) - 1);
    /* trim trailing space */
    size_t len = strlen(buf);
    if (len > 0 && buf[len - 1] == ' ')
        buf[len - 1] = '\0';
}

static void print_record(unsigned int idx, const flash_event_t *ev, int show_hex,
                          const volatile uint32_t *slot_words)
{
    /* Format timestamp */
    char timebuf[32];
    time_t ts = (time_t)ev->timestamp;
    struct tm *tm_info = localtime(&ts);
    if (tm_info)
        strftime(timebuf, sizeof(timebuf), "%Y-%m-%d %H:%M:%S", tm_info);
    else
        snprintf(timebuf, sizeof(timebuf), "(invalid ts)");

    /* Decode scaled integers back to floats */
    float temp_c     = ev->temp_x100   / 100.0f;
    float press_hpa  = ev->press_x100  / 100.0f;
    float humid_pct  = ev->humid_x100  / 100.0f;

    char alarm_buf[64];
    alarm_flags_str(ev->alarm_flags, alarm_buf, sizeof(alarm_buf));

    printf("┌─ Event #%-3u ───────────────────────────────────────────────────\n", idx);
    printf("│  Time        : %s  (unix: %u)\n", timebuf, ev->timestamp);
    printf("│  Trigger     : %s  (0x%04X)\n", event_code_str(ev->event_code), ev->event_code);
    printf("│  Temperature : %.2f °C\n", temp_c);
    printf("│  Pressure    : %.2f hPa\n", press_hpa);
    printf("│  Humidity    : %.2f %%RH\n", humid_pct);
    printf("│  Light ADC   : %u  (CH0, 0-1023)\n", ev->light);
    printf("│  Heating ADC : %u  (CH1, 0-1023)\n", ev->heating);
    printf("│  Sound ADC   : %u  (CH2, 0-1023)\n", ev->sound);
    printf("│  Alarm flags : 0x%02X  [%s]\n", ev->alarm_flags, alarm_buf);

    if (show_hex) {
        printf("│  Raw words   :");
        for (int w = 0; w < 8; w++)
            printf(" %08X", slot_words[w]);
        printf("\n");
    }

    printf("└───────────────────────────────────────────────────────────────\n");
}

/* ── Main ──────────────────────────────────────────────────────────────────── */

int main(int argc, char *argv[])
{
    int show_all = 0;
    int show_hex = 0;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--all") == 0) show_all = 1;
        else if (strcmp(argv[i], "--hex") == 0) show_hex = 1;
        else {
            fprintf(stderr, "Usage: %s [--all] [--hex]\n", argv[0]);
            return 1;
        }
    }

    /* Open /dev/mem */
    int fd = open("/dev/mem", O_RDONLY | O_SYNC);
    if (fd < 0) {
        perror("open /dev/mem");
        fprintf(stderr, "Hint: run as root (sudo ./ufm_read)\n");
        return 1;
    }

    /* Map the full LW bridge window */
    volatile void *lw_base = mmap(NULL, LW_BRIDGE_SPAN, PROT_READ,
                                  MAP_SHARED, fd, LW_BRIDGE_BASE);
    close(fd);
    if (lw_base == MAP_FAILED) {
        perror("mmap");
        return 1;
    }

    /* Pointer to the start of the BRAM region */
    volatile uint32_t *bram = (volatile uint32_t *)((uint8_t *)lw_base + FLASH_DATA_BASE);

    printf("╔══════════════════════════════════════════════════════════════════╗\n");
    printf("║         UFM / BRAM Event Log Reader — Smart Home Monitor         ║\n");
    printf("║  BRAM physical address: 0x%08lX                              ║\n",
           LW_BRIDGE_BASE + FLASH_DATA_BASE);
    printf("║  Capacity: %u events × %u bytes = %u bytes                   ║\n",
           MAX_EVENTS, BYTES_PER_EVENT, TOTAL_BRAM_BYTES);
    printf("╚══════════════════════════════════════════════════════════════════╝\n\n");

    unsigned int valid_count   = 0;
    unsigned int corrupt_count = 0;

    for (unsigned int slot = 0; slot < MAX_EVENTS; slot++) {
        /* Each record is 8 words; slot N starts at word offset N*8 */
        volatile uint32_t *slot_words = bram + slot * (BYTES_PER_EVENT / 4);

        uint32_t magic = slot_words[0];

        if (magic == MAGIC_FREE) {
            if (show_all)
                printf("[slot %3u] FREE (0x00000000) — end of log\n", slot);
            else
                break;  /* First free slot = end of written data */
            continue;
        }

        if (magic != MAGIC_VALID) {
            corrupt_count++;
            if (show_all)
                printf("[slot %3u] CORRUPT magic=0x%08X — skipped\n", slot, magic);
            continue;
        }

        /* Copy the 8 words into a local struct for safe field access */
        flash_event_t ev;
        memcpy(&ev, (const void *)slot_words, sizeof(ev));

        print_record(valid_count, &ev, show_hex, slot_words);
        valid_count++;
    }

    printf("\n");
    printf("── Summary ────────────────────────────────────────────────────────\n");
    printf("   Valid events  : %u\n", valid_count);
    if (corrupt_count)
        printf("   Corrupt slots : %u\n", corrupt_count);
    printf("   Free slots    : %u / %u\n", MAX_EVENTS - valid_count - corrupt_count, MAX_EVENTS);
    printf("───────────────────────────────────────────────────────────────────\n");

    munmap((void *)lw_base, LW_BRIDGE_SPAN);
    return 0;
}
