/*******************************************************************************
 * ufm_erase.c — Standalone FPGA On-Chip Memory (BRAM) Event Log Eraser
 *
 * PURPOSE
 * -------
 * This utility zeros out the entire 8 KB event-log region inside the FPGA's
 * on-chip SRAM (called "UFM" or "FLASH" in the project, though it is actually
 * volatile BRAM — not real non-volatile flash).  After running this tool,
 * ufm_storage.c will treat the region as empty and start writing from slot 0.
 *
 * WHY THIS IS NEEDED
 * ------------------
 * The main supervisor (main_supervisor.c) calls ufm_init() at startup to scan
 * the BRAM for existing records and finds the next free slot.  If old records
 * are present from a previous run they remain visible until overwritten.
 * Running ufm_erase wipes all 256 slots so the log is truly clean.
 *
 * PHYSICAL MEMORY MAP
 * -------------------
 *
 *   CPU physical address space:
 *   ┌──────────────────────────────────────────────────────┐
 *   │ 0xFF200000  LW HPS-to-FPGA AXI bridge base          │
 *   │   + 0x0000  bme280_i2c_0       (32 B)               │
 *   │   + 0x0020  mcp3008_spi_0      (32 B)               │
 *   │   + 0x0040  gpio_controller_0  (32 B)               │
 *   │   + 0x0060  alarm_logic_0      (32 B)               │
 *   │   + 0x0080  uart_sim800l_0     (32 B)               │
 *   │   + 0x2000  onchip_memory2_0   (8192 B = 8 KB) ◄── │
 *   └──────────────────────────────────────────────────────┘
 *
 *   Physical address of BRAM = 0xFF200000 + 0x2000 = 0xFF202000
 *
 *   The BRAM holds up to 256 event records, each 32 bytes (8 × 32-bit words):
 *     Record N occupies physical bytes 0xFF202000 + N*32 … + N*32 + 31
 *
 * HOW MMAP WORKS (brief recap)
 * ----------------------------
 *   /dev/mem exposes the raw physical address space as a file.
 *   mmap() maps a window of that file into the process's virtual address space.
 *   After mmap(), any pointer write to that virtual address is immediately
 *   visible to the FPGA fabric — no kernel driver, no ioctl needed.
 *
 *   MAP_SHARED is critical: it means writes go straight to the hardware and are
 *   NOT buffered in a private copy-on-write page.
 *
 *   O_SYNC on the open() call tells the kernel to disable CPU write buffering
 *   so that the ARM strongly-orders the writes to the peripheral bus.
 *
 * ERASE ALGORITHM
 * ---------------
 *   Total words = FLASH_MAX_EVENTS × WORDS_PER_EVENT = 256 × 8 = 2048 words
 *   Total bytes = 2048 × 4 = 8192 bytes (matches the Qsys IP depth × width)
 *
 *   We iterate w = 0 … 2047 and write 0x00000000 to word address:
 *       physical = 0xFF202000 + w*4
 *       virtual  = lw + 0x2000 + w*4
 *
 *   A word of 0x00000000 is safe because the magic marker for a valid record
 *   is 0xCAFEBABE (see ufm_storage.c).  ufm_init() skips any slot whose first
 *   word is not 0xCAFEBABE, so zeroed slots are treated as empty.
 *
 * PERSISTENCE NOTE
 * ----------------
 *   The BRAM is part of FPGA configuration RAM.  It is:
 *     • Cleared when the FPGA is re-programmed (new .sof/.rbf loaded)
 *     • Cleared when power is removed
 *     • PRESERVED across HPS (ARM) soft-resets — the FPGA keeps running
 *
 *   Therefore this tool is useful when you want to clear old logs while keeping
 *   the FPGA bitstream intact (i.e. the FPGA is already programmed and running).
 *
 * USAGE
 * -----
 *   gcc -std=c99 -O2 -o ufm_erase ufm_erase.c
 *   sudo ./ufm_erase
 *
 *   (root required to open /dev/mem)
 *
 * READING LOGS AFTER ERASING
 * --------------------------
 *   After running the supervisor again and generating new events, you can
 *   inspect the log without stopping the supervisor:
 *
 *   Method 1 — devmem2 (one word at a time):
 *     devmem2 0xFF202000 w        # magic of slot 0 (expect 0xCAFEBABE)
 *     devmem2 0xFF202004 w        # timestamp of slot 0
 *
 *   Method 2 — dd + hexdump (dump entire 8 KB):
 *     dd if=/dev/mem bs=32 count=256 skip=$((0xFF202000/32)) 2>/dev/null | hexdump -C
 *
 *   Method 3 — Python live reader:
 *     python3 - <<'EOF'
 *     import mmap, struct
 *     PAGE = 4096
 *     BASE = 0xFF202000
 *     with open("/dev/mem","rb") as f:
 *         m = mmap.mmap(f.fileno(), PAGE*2, mmap.MAP_SHARED, mmap.PROT_READ,
 *                       offset=BASE & ~(PAGE-1))
 *         off = BASE & (PAGE-1)
 *         for i in range(256):
 *             magic, ts, code = struct.unpack_from("<IIH", m, off + i*32)
 *             if magic == 0xCAFEBABE:
 *                 print(f"Slot {i:3d}: ts={ts}  code=0x{code:04X}")
 *     EOF
 *
 *******************************************************************************/

/* ── Feature-test macro ───────────────────────────────────────────────────── */
#ifndef _GNU_SOURCE
/* Enable GNU/POSIX extensions.  Required here mainly for off_t being 32- or
 * 64-bit depending on the toolchain.  Also keeps the file consistent with the
 * other sources in this project that use _GNU_SOURCE for usleep() etc.       */
#define _GNU_SOURCE
#endif

/* ── Standard headers ─────────────────────────────────────────────────────── */
#include <stdio.h>      /* printf(), perror()                                 */
#include <stdint.h>     /* uint8_t, uint32_t — fixed-width types              */
#include <fcntl.h>      /* open(), O_RDWR, O_SYNC                             */
#include <unistd.h>     /* close()                                            */
#include <sys/mman.h>   /* mmap(), munmap(), MAP_SHARED, PROT_READ/WRITE      */

/* ── Memory-map constants ─────────────────────────────────────────────────── */

/* Physical base of the Lightweight HPS-to-FPGA AXI bridge.
 * This is a fixed address in the Cyclone V SoC memory map (see
 * Cyclone V HPS Technical Reference Manual, Table 3-1).
 * The Linux kernel does NOT normally expose a driver for the FPGA fabric,
 * so we access it directly via /dev/mem.                                     */
#define LW_BRIDGE_BASE      0xFF200000UL

/* How many bytes to map.  We map the full 2 MB window so that a single mmap()
 * call covers every peripheral at once.  Only 8 KB of that window (at +0x2000)
 * is actually accessed by this tool, but mapping the full span matches what
 * the main supervisor does and avoids alignment issues.                      */
#define LW_BRIDGE_SPAN      0x00200000UL   /* 2 MB */

/* Offset within the LW bridge window where the on-chip SRAM starts.
 * Assigned in Platform Designer (Qsys).  Must match smart_home.h.           */
#define FLASH_DATA_BASE     0x2000UL

/* Maximum number of 32-byte event records that fit in the 8 KB BRAM:
 *   8192 bytes / 32 bytes per record = 256 records                           */
#define FLASH_MAX_EVENTS    256U

/* Each record is 32 bytes = 8 × 32-bit words.
 * We erase word-by-word (4 bytes at a time) because the Avalon-MM bus to the
 * on-chip memory has a 32-bit data width — byte-granularity writes would
 * require read-modify-write, which is unnecessary when zeroing everything.   */
#define FLASH_BYTES_PER_EVENT  32U
#define WORDS_PER_EVENT        (FLASH_BYTES_PER_EVENT / 4U)   /* = 8 */

/* ════════════════════════════════════════════════════════════════════════════
 * main()
 *
 * Steps:
 *   1. Open /dev/mem with O_SYNC to disable write-buffer coalescing.
 *   2. mmap() the full LW bridge window into process virtual address space.
 *   3. Compute total word count (2048).
 *   4. Loop: write 0x00000000 to every 32-bit word in the BRAM region.
 *   5. Unmap and exit.
 * ════════════════════════════════════════════════════════════════════════════ */
int main(void)
{
    /* ── Step 1: open /dev/mem ─────────────────────────────────────────────
     * O_RDWR  — we need write access to zero out the BRAM.
     * O_SYNC  — disables the CPU write buffer for this mapping.
     *            Without O_SYNC the ARM may reorder or batch writes, which can
     *            cause the peripheral to see data in the wrong order.  For a
     *            simple sequential erase this is not strictly critical, but it
     *            is good practice and matches the main supervisor.            */
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        /* perror() prints the string "open /dev/mem" followed by a colon and
         * the human-readable errno description, e.g.:
         *   open /dev/mem: Operation not permitted
         * This happens if the tool is run without root (sudo).               */
        perror("open /dev/mem");
        return 1;   /* non-zero exit → shell/make can detect failure         */
    }

    /* ── Step 2: mmap() the LW bridge ─────────────────────────────────────
     * Argument breakdown:
     *   NULL           — let the kernel choose the virtual address
     *   LW_BRIDGE_SPAN — 2 MB region to map
     *   PROT_READ|WRITE— we need both read (for pointer arithmetic) and write
     *   MAP_SHARED     — writes go directly to hardware, not a private page
     *   fd             — /dev/mem file descriptor
     *   LW_BRIDGE_BASE — physical address offset within /dev/mem
     *
     * On success, vbase points to a virtual address that aliases physical
     * 0xFF200000.  Writing to (vbase + 0x2000 + 4*w) writes physical word w
     * inside the BRAM.                                                        */
    void *vbase = mmap(NULL,
                       LW_BRIDGE_SPAN,
                       PROT_READ | PROT_WRITE,
                       MAP_SHARED,
                       fd,
                       (off_t)LW_BRIDGE_BASE);   /* cast needed: off_t may be
                                                     32-bit on armhf toolchain */

    /* Close the file descriptor immediately after mmap().
     * The mapping remains valid until munmap() is called — the kernel keeps
     * the underlying reference alive.  Closing fd early is good practice to
     * avoid leaking file descriptors in error paths below.                   */
    close(fd);

    /* Check for mmap failure.  MAP_FAILED is (void *)-1, a sentinel value
     * defined in <sys/mman.h>.  Common causes:
     *   • /dev/mem not readable (missing root or CONFIG_STRICT_DEVMEM=y)
     *   • Physical address not accessible (wrong SoC / address typo)        */
    if (vbase == MAP_FAILED) {
        perror("mmap LW bridge");
        return 1;
    }

    /* Cast to byte pointer so we can do byte-granularity pointer arithmetic.
     * "volatile" tells the compiler NOT to cache the value in a register or
     * optimise away the write — every store must actually reach the bus.
     * This is mandatory for MMIO.                                             */
    volatile uint8_t *lw = (volatile uint8_t *)vbase;

    /* ── Step 3: compute total word count ─────────────────────────────────
     *   256 records × 8 words/record = 2048 words
     *   2048 words × 4 bytes/word   = 8192 bytes  (= 8 KB BRAM depth)       */
    uint32_t total_words = FLASH_MAX_EVENTS * WORDS_PER_EVENT;   /* 2048 */

    /* Inform the user what is about to happen before doing any destructive
     * writes.  The %04lX format prints the offset as a 4-digit hex number
     * (e.g. "0x2000") for easy cross-reference with Qsys / smart_home.h.    */
    printf("UFM erase: clearing %u words (%u bytes) at LW offset 0x%04lX...\n",
           total_words,
           total_words * 4U,
           (unsigned long)FLASH_DATA_BASE);

    /* ── Step 4: zero all words ────────────────────────────────────────────
     * For each word index w in [0, 2047]:
     *   virtual address  = lw + FLASH_DATA_BASE + w*4
     *   physical address = 0xFF200000 + 0x2000 + w*4
     *                    = 0xFF202000 + w*4
     *
     * We write 0x00000000.  The ufm_init() scan in ufm_storage.c considers
     * a slot empty when its first word (magic field) != 0xCAFEBABE.
     * 0x00000000 satisfies that condition, so all slots will appear empty
     * after this loop.
     *
     * The cast chain:
     *   lw                    — volatile uint8_t *  (byte pointer into mmap)
     *   + FLASH_DATA_BASE      — skip to the BRAM region  (byte offset 0x2000)
     *   + w * 4U               — advance to word w         (4 bytes per word)
     *   (volatile uint32_t *)  — reinterpret as 32-bit word pointer
     *   * … = 0x00000000U      — write 32 bits of zero in one bus transaction
     *
     * Using uint32_t writes (not four uint8_t writes) ensures the Avalon-MM
     * bus sees a single 32-bit write strobe, which is what the on-chip memory
     * IP expects.                                                              */
    for (uint32_t w = 0; w < total_words; w++)
        *(volatile uint32_t *)(lw + FLASH_DATA_BASE + w * 4U) = 0x00000000U;

    /* Confirm completion. */
    printf("UFM erase: done — all %u event slots cleared.\n",
           FLASH_MAX_EVENTS);

    /* ── Step 5: unmap and exit ────────────────────────────────────────────
     * munmap() releases the virtual address range.  Any subsequent pointer
     * dereference through lw or vbase would cause a segfault.
     * Always unmap before exit — the OS will do it anyway, but being explicit
     * makes the intent clear and allows tools like valgrind to confirm clean
     * teardown.                                                               */
    munmap(vbase, LW_BRIDGE_SPAN);
    return 0;   /* 0 = success; checked by Makefile if called from a recipe  */
}
