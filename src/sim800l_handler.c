/*******************************************************************************
 * sim800l_handler.c — SIM800L GSM Module via Direct MMIO on FPGA Altera UART
 *
 * ============================================================================
 * BIG PICTURE — WHAT THIS FILE DOES
 * ============================================================================
 *
 * This module drives a SIM800L GSM/GPRS module to send SMS alert messages.
 * Instead of going through a Linux UART driver (/dev/ttyAL* or /dev/ttyS2),
 * it accesses the Altera UART IP registers DIRECTLY via /dev/mem + mmap —
 * the same mechanism used for all other FPGA peripherals in this project.
 *
 * WHY DIRECT MMIO (not a Linux driver)?
 *   The altera_uart kernel module is NOT present in the DE10-Nano kernel image
 *   (v6.1.108-armv7-fpga).  Without the driver, /dev/ttyAL0 never appears in
 *   the filesystem.  Direct register access sidesteps this entirely.
 *
 * PHYSICAL DATA PATH:
 *   ARM Linux process
 *       |
 *       | mmap /dev/mem -> LW bridge 0xFF200000
 *       |
 *   FPGA Altera UART IP (offset 0x80 from LW bridge base)
 *       | UART TX/RX at 19200 baud
 *       |
 *   JP1 Pin 19 (PIN_D12)  -> SIM800L RXD
 *   JP1 Pin 20 (PIN_AD20) <- SIM800L TXD
 *       |
 *   SIM800L EVB (GSM module with onboard LDO)
 *       |
 *       | GSM 900/1800 MHz radio
 *       |
 *   SMS to ALERT_PHONE_NUMBER
 *
 * ============================================================================
 * ALTERA UART REGISTER MAP (at base offset UART_SIM800L_BASE = 0x80)
 * ============================================================================
 *
 *   Offset  Name          Width  Description
 *   +0x00   rxdata        [7:0]  Received byte — valid only when RRDY=1
 *   +0x04   txdata        [7:0]  Byte to transmit — write only when TRDY=1
 *   +0x08   status        [31:0] bit7=RRDY  bit6=TRDY  bit5=TMT  bit3=ROE
 *   +0x0C   control       [31:0] Interrupt enables (kept 0 = polled mode)
 *   +0x10   divisor       [31:0] Baud divisor = f_clk/baud - 1  (2603 for 19200 @ 50MHz)
 *   +0x14   endofpacket   [7:0]  (not used in this project)
 *
 *   RRDY  (bit 7): Receive Data Ready  — 1 = a byte is waiting in rxdata
 *   TRDY  (bit 6): Transmit Ready      — 1 = space in TX FIFO for a new byte
 *   TMT   (bit 5): Transmit Empty      — 1 = all bytes have been shifted out
 *   ROE   (bit 3): Receive Overrun Err — 1 = a byte was lost (RX buffer full)
 *
 *   Baud divisor for 19200 baud at 50 MHz FPGA clock:
 *     divisor = 50,000,000 / 19,200 - 1 = 2603  (integer division)
 *
 * ============================================================================
 * SMS SEND PROTOCOL (Hayes/AT commands)
 * ============================================================================
 *
 *   Step 1: ATE0              -> disable echo (prevents reply contamination)
 *   Step 2: AT+CMGF=1         -> set SMS text mode (vs. PDU mode)
 *   Step 3: AT+CMGS="number"  -> start SMS to this phone number
 *   Step 4: (wait for ">")    -> SIM800L ready to accept message body
 *   Step 5: write message text
 *   Step 6: write 0x1A (Ctrl+Z) -> signals end of message, triggers send
 *   Step 7: (wait for "+CMGS:") -> confirms message was dispatched to SMSC
 *
 * ============================================================================
 * NOTES ON SIM (ThingsMobile IoT SIM)
 * ============================================================================
 *
 *   The SIM is a ThingsMobile roaming IoT SIM with home PLMN 23450 (UK).
 *   Special configuration required (done in sim800l_init):
 *   - AT+CSCA: set SMSC to ThingsMobile's UK gateway (+447797704000)
 *   - AT+CGDCONT: set APN to "TM" for ThingsMobile
 *   - AT+CBAND="ALL_MODE": ensure 900/1800 MHz bands are enabled for Italy
 *   - AT+COPS=0: force automatic operator selection (roaming in Italy)
 *
 *******************************************************************************/

#ifndef _GNU_SOURCE
#define _GNU_SOURCE   /* Unlock usleep() under -std=c99 */
#endif

#include <stdio.h>      /* printf(), perror(), snprintf()                        */
#include <stdlib.h>     /* NULL                                                  */
#include <string.h>     /* memset(), strstr()                                    */
#include <unistd.h>     /* usleep(), sleep()                                     */
#include <fcntl.h>      /* open(), O_RDWR, O_SYNC                                */
#include <sys/mman.h>   /* mmap(), munmap(), MAP_SHARED                          */
#include <errno.h>      /* errno, perror()                                       */
#include <stdbool.h>    /* bool, true, false                                     */

#include "smart_home.h" /* LW_BRIDGE_BASE/SPAN, UART_SIM800L_BASE, LOG_* macros  */

/*******************************************************************************
 * ALTERA UART REGISTER OFFSETS (bytes from LW bridge base address)
 *
 * Each register address = LW_BRIDGE_BASE + UART_SIM800L_BASE + register_offset.
 * For example: rxdata is at physical address 0xFF200000 + 0x80 + 0x00 = 0xFF200080.
 *******************************************************************************/
#define UART_RXDATA     (UART_SIM800L_BASE + 0x00UL)  /* Read: received byte [7:0]  */
#define UART_TXDATA     (UART_SIM800L_BASE + 0x04UL)  /* Write: byte to transmit     */
#define UART_STATUS     (UART_SIM800L_BASE + 0x08UL)  /* Status flags register       */
#define UART_CONTROL    (UART_SIM800L_BASE + 0x0CUL)  /* Interrupt enables (keep 0)  */
#define UART_DIVISOR    (UART_SIM800L_BASE + 0x10UL)  /* Baud rate divisor register  */

/* Status register bit masks */
#define UART_RRDY   (1U << 7)  /* Bit 7: Receive Ready  — byte waiting in rxdata  */
#define UART_TRDY   (1U << 6)  /* Bit 6: Transmit Ready — TX FIFO has space        */
#define UART_TMT    (1U << 5)  /* Bit 5: Transmit Empty — shift register drained   */
#define UART_ROE    (1U << 3)  /* Bit 3: Receive Overrun Error — byte was lost      */

/* Baud rate divisor for 19200 baud at 50 MHz FPGA system clock.
 * Formula: divisor = f_clk / baud - 1 = 50,000,000 / 19,200 - 1 = 2603
 * If the IP was synthesised with a fixed baud rate, writing this register
 * is silently ignored and the hardwired rate is used instead. */
#define BAUD_19200_DIVISOR   ((50000000UL / 19200UL) - 1UL)   /* = 2603 */

#define BUFFER_SIZE  512  /* Maximum bytes accumulated in rx_buffer per response */

/* Module-private state */
static volatile uint8_t  *lw_base  = NULL;       /* mmap base pointer to LW bridge */
static char               rx_buffer[BUFFER_SIZE]; /* Accumulates UART response bytes */

/*******************************************************************************
 * LOW-LEVEL UART REGISTER HELPERS
 *
 * uart_rd / uart_wr abstract the volatile pointer arithmetic needed to access
 * the Altera UART registers via the mmap'd LW bridge pointer.
 *
 * lw_base is a byte pointer.  Adding 'off' (a byte offset) and casting to
 * uint32_t* gives the address of the 32-bit UART register.  Dereferencing
 * with volatile ensures no compiler caching occurs.
 *******************************************************************************/
static inline uint32_t uart_rd(uint32_t off)
{
    /* Read a 32-bit UART register at byte offset 'off' from the LW bridge base.
     * e.g. uart_rd(UART_STATUS) reads the status register at 0xFF200088. */
    return *(volatile uint32_t *)(lw_base + off);
}

static inline void uart_wr(uint32_t off, uint32_t val)
{
    /* Write 'val' to a 32-bit UART register at byte offset 'off'. */
    *(volatile uint32_t *)(lw_base + off) = val;
}

/*******************************************************************************
 * uart_flush_rx — Discard All Bytes Currently in the RX Register
 *
 * The Altera UART stores the most recently received byte in the rxdata register.
 * This function reads (and discards) bytes as long as RRDY=1 (data available).
 * Used before sending an AT command to clear any leftover response bytes or
 * unsolicited notifications (e.g. "RING", "+CMTI:") from previous operations.
 *******************************************************************************/
static void uart_flush_rx(void)
{
    /* Keep reading and discarding bytes as long as RRDY bit is set.
     * (void) cast suppresses "unused value" compiler warning on the read. */
    while (uart_rd(UART_STATUS) & UART_RRDY)
        (void)uart_rd(UART_RXDATA);  /* Read and discard one byte */
}

/*******************************************************************************
 * uart_putc — Transmit One Byte Over the UART
 *
 * Spin-waits for TRDY (Transmit Ready) before writing to the txdata register.
 * A guard counter prevents an infinite loop if the UART is stuck.
 *
 * The usleep(10) = 10 microsecond idle between polls.
 * At 19200 baud, one character takes 1/19200 * 10 bits ≈ 520 microseconds.
 * So the guard allows roughly: 100,000 × 10 µs = 1 second of waiting.
 *******************************************************************************/
static void uart_putc(uint8_t c)
{
    int guard = 100000;  /* Limit spin iterations to prevent infinite loop */

    /* Wait until there is space in the TX register (TRDY=1).
     * TRDY goes low when the byte is loaded into the TX shift register. */
    while (!(uart_rd(UART_STATUS) & UART_TRDY) && --guard > 0)
        usleep(10);  /* 10 µs idle — avoids hammering the MMIO bus */

    /* Write the byte to the TX data register.
     * The FPGA UART IP will automatically start shifting it out at 19200 baud. */
    uart_wr(UART_TXDATA, (uint32_t)c);
}

/*******************************************************************************
 * uart_puts_raw — Transmit a Null-Terminated String Byte by Byte
 *
 * Iterates over each character in the string, calling uart_putc() for each.
 * Stops at the null terminator ('\0').  Does NOT append \r\n — callers must
 * add those explicitly when needed (e.g. for AT commands).
 *******************************************************************************/
static void uart_puts_raw(const char *s)
{
    while (*s)                /* Loop until null terminator */
        uart_putc((uint8_t)(*s++));  /* Transmit current char, advance pointer */
}

/*******************************************************************************
 * wait_for_response — Poll UART RX Until Expected String Found or Timeout
 *
 * This is the core receive loop.  It polls the RRDY status bit directly
 * (hardware polling, not blocking read()) and accumulates received bytes
 * into rx_buffer.  After each byte, it checks whether rx_buffer contains
 * the expected response string or "ERROR".
 *
 * Why polling instead of select()/read()?
 *   The Altera UART is accessed via MMIO — it is NOT a /dev/ttyXX file.
 *   There is no file descriptor to pass to select().  We must poll the
 *   RRDY bit in the status register to know when a byte is ready.
 *
 * Timeout mechanism:
 *   elapsed += 1 whenever no byte is available (usleep(1000) = 1 ms).
 *   This is not as accurate as clock_gettime() but is sufficient for
 *   AT command timeouts in the seconds range.
 *
 * ROE (Receive Overrun Error) handling:
 *   If the SIM800L sends data faster than we can read it (unlikely at 19200
 *   baud), the hardware sets the ROE bit.  We clear it by writing 0 to the
 *   status register.  This is a "write to clear" operation per Altera spec.
 *
 * Parameters:
 *   expected    : substring to search for in the accumulated response
 *   timeout_ms  : maximum wait time in milliseconds
 *
 * Returns 0 when expected string found, -1 on "ERROR" or timeout.
 * Side effect: rx_buffer contains whatever was received (useful for callers).
 *******************************************************************************/
static int wait_for_response(const char *expected, int timeout_ms)
{
    int total   = 0;    /* Bytes accumulated in rx_buffer */
    int elapsed = 0;    /* Approximate elapsed milliseconds */
    memset(rx_buffer, 0, BUFFER_SIZE);

    while (elapsed < timeout_ms) {
        uint32_t st = uart_rd(UART_STATUS);  /* Read UART status register */

        /* Check and clear Receive Overrun Error flag.
         * Writing 0 to the status register clears the error bits per Altera docs. */
        if (st & UART_ROE)
            uart_wr(UART_STATUS, 0);

        if (st & UART_RRDY) {
            /* A byte is available in the rxdata register — read it.
             * Mask to 8 bits: the register is 32-bit wide but only [7:0] is data. */
            uint8_t c = (uint8_t)(uart_rd(UART_RXDATA) & 0xFFU);

            /* Append to buffer if there is space (leave room for null terminator) */
            if (total < BUFFER_SIZE - 1) {
                rx_buffer[total++] = (char)c;  /* Store byte */
                rx_buffer[total]   = '\0';     /* Keep null-terminated */
            }

            /* Check for expected response (e.g. "OK", ">", "+CMGS:") */
            if (strstr(rx_buffer, expected)) return  0;  /* Found — success */

            /* Check for explicit ERROR response from SIM800L */
            if (strstr(rx_buffer, "ERROR"))  return -1;  /* Command rejected */

        } else {
            /* No byte available — sleep 1 ms to avoid busy-spinning */
            usleep(1000);  /* 1 ms sleep: 1000 iterations = ~1 second total */
            elapsed += 1;  /* Increment elapsed counter (1 unit = 1 ms) */
        }
    }

    return -1;  /* Timeout: expected response not received within timeout_ms */
}

/*******************************************************************************
 * sim800l_init — Map UART MMIO and Initialize the SIM800L Module
 *
 * INITIALIZATION SEQUENCE:
 *
 *   1. mmap /dev/mem: map the LW bridge into this process's address space.
 *      This is the same region used by main_supervisor.c and ufm_storage.c.
 *      Linux MAP_SHARED makes all views coherent; overlapping offsets won't
 *      conflict since UART is at 0x80 and other peripherals are at 0x00-0x70.
 *
 *   2. Configure baud rate: write divisor 2603 to UART_DIVISOR register.
 *      If the FPGA IP was synthesised with a fixed baud rate, this write is
 *      silently ignored and the hardware rate takes effect.
 *
 *   3. Disable interrupts: write 0 to UART_CONTROL (polled mode only).
 *
 *   4. Flush RX buffer: discard any bytes from module startup messages.
 *
 *   5. Boot wait + AT retry: SIM800L can take 10-20 seconds to power on
 *      depending on cold start vs warm start.  Wait 5 seconds, then retry
 *      "AT" every 2 seconds for up to 10 attempts (~25 seconds total).
 *
 *   6. ATE0: disable command echo.  By default (ATE1), the SIM800L echoes
 *      every character we send back to us.  This means when we send the SMS
 *      message body, all those characters fill rx_buffer BEFORE the "+CMGS:"
 *      confirmation arrives, causing wait_for_response() to time out.
 *
 *   7. AT+CPIN?: verify the SIM card is inserted and PIN-unlocked.
 *
 *   8. SIM-specific configuration for ThingsMobile IoT SIM:
 *      AT+CSCA: set SMSC (SMS Centre) address to ThingsMobile UK gateway.
 *      AT+CGDCONT: set APN to "TM" (needed for some roaming configurations).
 *      AT+CBAND: enable all GSM bands (900+1800 MHz for Italy).
 *      AT+COPS=0: force automatic operator selection (roam to any network).
 *
 *   9. Network registration: poll AT+CREG? until registered (stat=1 or 5).
 *      If registration times out, print diagnostics and continue
 *      (SMS will fail later, but system can still log and publish MQTT).
 *
 *  10. AT+CMGF=1: set SMS text mode (vs PDU mode).
 *      Retried up to 3 times because the SIM800L can reboot under high
 *      current draw (TX burst during GSM cell search), resetting to ATE1.
 *
 * Returns 0 on success, -1 on failure.
 *******************************************************************************/
int sim800l_init(void)
{
    /* ---- Step 1: Map the LW bridge via /dev/mem -------------------------- */
    int fd = open("/dev/mem", O_RDWR | O_SYNC);  /* Open physical memory device */
    if (fd < 0) {
        perror("SIM800L: open /dev/mem");
        return -1;
    }

    /* Map the full 2 MB LW bridge window.  The UART is at offset 0x80 within
     * this window, at absolute physical address 0xFF200080. */
    void *vbase = mmap(NULL, LW_BRIDGE_SPAN, PROT_READ | PROT_WRITE,
                       MAP_SHARED, fd, LW_BRIDGE_BASE);
    close(fd);  /* fd is no longer needed after mmap completes */

    if (vbase == MAP_FAILED) {
        perror("SIM800L: mmap LW bridge");
        return -1;
    }

    lw_base = (volatile uint8_t *)vbase;  /* Store as byte pointer for offset arithmetic */

    /* ---- Step 2: Configure baud rate divisor ----------------------------- */
    /* Write 2603 to the divisor register.  At 50 MHz FPGA clock:
     * baud_rate = f_clk / (divisor + 1) = 50,000,000 / 2604 ≈ 19,200 baud */
    uart_wr(UART_DIVISOR, BAUD_19200_DIVISOR);

    /* ---- Step 3: Disable all UART interrupts ----------------------------- */
    /* We use polling (checking RRDY/TRDY status bits), not interrupts.
     * Writing 0 to the control register disables all interrupt sources. */
    uart_wr(UART_CONTROL, 0);

    /* ---- Step 4: Flush stale bytes from RX buffer ------------------------ */
    uart_flush_rx();  /* Discard any bytes received before initialization */

    LOG_INFO("SIM800L UART mapped via /dev/mem at LW+0x%02lX, 19200 baud",
             (unsigned long)UART_SIM800L_BASE);

    /* ---- Step 5: Wait for SIM800L boot, then probe with AT --------------- */
    /* The SIM800L takes 10-20 seconds to power up, register on GSM, and
     * become responsive to AT commands.  We wait 5 seconds then try up to
     * 10 times with 2-second gaps (5 + 10×2 = 25 seconds max wait). */
    LOG_INFO("Waiting for SIM800L to boot (up to 25 s)...");
    sleep(5);  /* Initial wait: allow SIM800L power-on sequence to complete */

    bool at_ok = false;
    for (int attempt = 0; attempt < 10 && !at_ok; attempt++) {
        uart_flush_rx();  /* Clear any boot messages before sending AT */
        if (sim800l_send_at_command("AT", "OK", 2000) == 0) {
            at_ok = true;  /* Module responded — boot complete */
        } else {
            LOG_INFO("SIM800L not ready yet (attempt %d/10)...", attempt + 1);
            sleep(2);  /* Wait 2 more seconds before retrying */
        }
    }

    if (!at_ok) {
        LOG_ERROR("SIM800L not responding to AT after ~25 s — check power and wiring");
        munmap(vbase, LW_BRIDGE_SPAN);  /* Release the mmap before returning error */
        lw_base = NULL;
        return -1;
    }

    /* ---- Step 6: Disable command echo (ATE0) ----------------------------- */
    /* Factory default is ATE1 (echo ON).  With echo on, every character we
     * transmit is immediately echoed back to us over RX.  When sending the
     * SMS message body, the echo fills rx_buffer before "+CMGS:" arrives,
     * causing wait_for_response("+CMGS:") to time out (buffer full of echo).
     * ATE0 disables echo — critical for reliable SMS sending. */
    sim800l_send_at_command("ATE0", "OK", 2000);

    /* ---- Step 7: Verify SIM card is ready -------------------------------- */
    /* AT+CPIN? response:
     *   +CPIN: READY  -> SIM inserted, PIN not required (or already unlocked)
     *   +CPIN: SIM PIN -> SIM requires PIN (not handled in this code)
     *   ERROR          -> No SIM inserted */
    if (sim800l_send_at_command("AT+CPIN?", "READY", 5000) != 0) {
        LOG_ERROR("SIM card not ready — check SIM insertion and SIM800L power supply");
        return -1;
    }

    /* ---- Step 8: SIM-specific configuration (ThingsMobile IoT SIM) ------ */

    /* Set SMSC (SMS Service Centre) address.
     * ThingsMobile routes SMS via UK SMSC +447797704000.
     * Type=145 = international format (starts with '+').
     * Without this, AT+CMGS always fails because the module doesn't know
     * which SMS centre to route the message through. */
    sim800l_send_at_command("AT+CSCA=\"+447797704000\",145", "OK", 3000);

    /* Set APN for PDP context 1.  ThingsMobile uses APN "TM".
     * AT+CGDCONT=<cid>,"<type>","<apn>"
     * This is needed for some roaming SIM800L firmware versions that check
     * the APN during GSM network attachment in roaming configurations. */
    sim800l_send_at_command("AT+CGDCONT=1,\"IP\",\"TM\"", "OK", 3000);

    /* Configure standard +CREG response format (unsolicited disabled).
     * AT+CREG=0: only respond to direct queries, no unsolicited status URCs. */
    sim800l_send_at_command("AT+CREG=0", "OK", 2000);

    /* Enable all GSM frequency bands.
     * The SIM800L supports 850/900/1800/1900 MHz.  "ALL_MODE" ensures it
     * scans all bands, which is needed for correct roaming in Italy
     * (900 MHz + 1800 MHz are the primary Italian GSM bands). */
    sim800l_send_at_command("AT+CBAND=\"ALL_MODE\"", "OK", 3000);

    /* Force automatic operator selection.
     * The SIM's home PLMN is 23450 (UK).  Without this, the module may spend
     * minutes hunting for the UK home network before trying Italian roaming.
     * AT+COPS=0 instructs it to register on any available network automatically. */
    sim800l_send_at_command("AT+COPS=0", "OK", 5000);

    /* ---- Step 9: Wait for GSM network registration ----------------------- */
    /* Poll AT+CREG? up to 10 times (1 second apart).
     * +CREG: 0,1 = registered home network
     * +CREG: 0,5 = registered roaming
     * +CREG: 0,2 = not registered, searching
     * We look for ",1" or ",5" in the response buffer. */
    LOG_INFO("Waiting for GSM network registration...");
    bool registered = false;

    for (int i = 0; i < 10 && !registered; i++) {
        uart_flush_rx();                           /* Clear any pending bytes   */
        sim800l_send_at_command("ATE0", "OK", 1000); /* Re-send ATE0 defensively */
        uart_flush_rx();

        /* Send AT+CREG? directly and capture the full response in rx_buffer */
        uart_puts_raw("AT+CREG?\r\n");             /* Transmit query command    */
        wait_for_response("OK", 3000);             /* rx_buffer now has response*/

        LOG_INFO("CREG attempt %d/10: %.60s", i + 1, rx_buffer);

        /* Check for registration status codes in the response.
         * The response looks like: +CREG: 0,1\r\nOK
         * ",1" = home, ": 1" = home (older firmware), ",5" or ": 5" = roaming */
        if (strstr(rx_buffer, ",1") || strstr(rx_buffer, ": 1") ||
            strstr(rx_buffer, ",5") || strstr(rx_buffer, ": 5")) {
            LOG_INFO("GSM network registered successfully");
            registered = true;
        } else {
            sleep(1);  /* Not registered yet — wait 1 second before retrying */
        }
    }

    if (!registered) {
        /* Registration failed — print diagnostics but continue.
         * The system will still work for MQTT/logging; SMS will fail. */
        LOG_WARN("GSM registration timed out — SMS alerts may fail");

        /* Diagnostic: check signal strength and visible operators.
         * AT+CSQ returns RSSI and BER (signal quality).
         * AT+COPS? shows currently selected operator (if any). */
        LOG_INFO("--- GSM Registration Diagnostics ---");
        uart_flush_rx();
        sim800l_send_at_command("ATE0", "OK", 1000);
        uart_flush_rx();
        sim800l_send_at_command("AT+CSQ", "OK", 2000);      /* Signal strength */
        LOG_INFO("Signal quality (AT+CSQ): %.40s", rx_buffer);
        uart_flush_rx();
        sim800l_send_at_command("AT+COPS?", "OK", 5000);    /* Current operator */
        LOG_INFO("Current operator (AT+COPS?): %.80s", rx_buffer);
        LOG_INFO("--- End Diagnostics ---");
    }

    /* ---- Step 10: Set SMS text mode (AT+CMGF=1) with retry --------------- */
    /* The SIM800L can reboot under heavy GSM TX current spikes (especially
     * when powered from the board's 500 mA USB rail).  A reboot resets it to
     * ATE1 and default settings.  Retry AT+CMGF=1 up to 3 times to survive
     * a module that rebooted at the end of the registration window. */
    {
        bool cmgf_ok = false;
        for (int r = 0; r < 3 && !cmgf_ok; r++) {
            uart_flush_rx();  /* Clear any rogue bytes before sending command */
            if (sim800l_send_at_command("AT+CMGF=1", "OK", 3000) == 0) {
                cmgf_ok = true;  /* SMS text mode confirmed */
            } else {
                LOG_WARN("AT+CMGF=1 failed (attempt %d/3) — retrying in 3 s...", r + 1);
                sleep(3);  /* Wait for module to recover from potential reboot */
            }
        }
        if (!cmgf_ok) {
            /* After 3 failures, SMS is not usable — return error */
            LOG_ERROR("Failed to set SMS text mode after 3 attempts");
            return -1;
        }
    }

    LOG_INFO("SIM800L ready for SMS");
    return 0;  /* All initialization steps succeeded */
}

/*******************************************************************************
 * sim800l_send_at_command — Transmit AT Command, Wait for Response
 *
 * Flushes the RX buffer, transmits "cmd\r\n" byte by byte using uart_puts_raw()
 * and uart_putc(), then calls wait_for_response() to poll for the reply.
 *
 * The separate uart_flush_rx() + uart_puts_raw() + uart_putc('\r') + '\n'
 * sequence ensures precise control over the transmitted bytes.
 *
 * Returns 0 if expected_response found in reply, -1 on ERROR or timeout.
 *******************************************************************************/
int sim800l_send_at_command(const char *cmd, const char *expected_response,
                              int timeout_ms)
{
    if (lw_base == NULL) return -1;  /* Guard: UART MMIO must be mapped */

    uart_flush_rx();           /* Discard stale bytes before new command */
    uart_puts_raw(cmd);        /* Transmit the command string (no \r\n yet) */
    uart_putc('\r');           /* Carriage return — AT command delimiter     */
    uart_putc('\n');           /* Line feed — completes the \r\n termination */

    /* Wait for the expected response string in the UART RX stream */
    return wait_for_response(expected_response, timeout_ms);
}

/*******************************************************************************
 * sim800l_send_sms — Compose and Send an SMS Message
 *
 * FULL SMS SEND PROTOCOL:
 *
 *   1. ATE0:             Re-send echo disable (defensive — module may reboot).
 *      WHY: The SIM800L EVB can brown-out and reboot under TX-burst current.
 *      A reboot resets it to ATE1 (echo ON).  If echo is on when we send the
 *      message body, all those characters fill rx_buffer BEFORE "+CMGS:" arrives
 *      and wait_for_response() can never find the confirmation token.
 *      Sending ATE0 before every SMS is cheap (~50 ms) and harmless.
 *
 *   2. AT+CMGS="phone":  Start SMS composition to this number.
 *      SIM800L responds with "> " (greater-than + space) prompting for body.
 *
 *   3. (wait for ">")    If no ">" prompt in 5 seconds, GSM is not ready.
 *
 *   4. write message:    Transmit the SMS text body (max 160 chars for 1 SMS).
 *
 *   5. write 0x1A:       Ctrl+Z character signals "end of message, send now".
 *                        The SIM800L sends the SMS to the SMSC and responds
 *                        "+CMGS: <reference_number>" followed by "OK".
 *
 *   6. (wait for "+CMGS:") 30 second timeout: GSM transmission can be slow.
 *      Finding "+CMGS:" confirms the SMSC acknowledged the message.
 *
 * Parameters:
 *   phone_number : destination in E.164 format, e.g. "+393483190237"
 *   message      : SMS body text (plain ASCII, max ~160 chars for 1 SMS)
 *
 * Returns 0 on success, -1 on failure.
 *******************************************************************************/
int sim800l_send_sms(const char *phone_number, const char *message)
{
    if (lw_base == NULL) {
        LOG_WARN("SIM800L not initialised — cannot send SMS");
        return -1;
    }

    /* Step 1: Re-disable echo.  This call is cheap and prevents the echo
     * from filling rx_buffer before the "+CMGS:" confirmation arrives. */
    sim800l_send_at_command("ATE0", "OK", 2000);

    uart_flush_rx();  /* Clear any stale bytes (unsolicited URCs, etc.) */

    /* Step 2: Initiate SMS to the target phone number.
     * Format: AT+CMGS="<phone_number>"\r\n
     * The SIM800L will respond with ">\r\n" indicating it is ready for the body. */
    char cmd[100];
    snprintf(cmd, sizeof(cmd), "AT+CMGS=\"%s\"\r\n", phone_number);
    uart_puts_raw(cmd);  /* Transmit the CMGS command directly (includes \r\n) */

    /* Step 3: Wait for the ">" prompt — SIM800L is ready to receive the body */
    if (wait_for_response(">", 5000) != 0) {
        LOG_ERROR("SIM800L: no '>' prompt for SMS — check GSM registration");
        return -1;  /* Could not start SMS (no signal, not registered, etc.) */
    }

    /* Step 4: Transmit the message body character by character */
    uart_puts_raw(message);

    /* Step 5: Send Ctrl+Z (0x1A) to terminate the message and trigger send.
     * 0x1A is the ASCII "Substitute" character, used by Hayes AT protocol
     * to signal end-of-message in SMS composition mode. */
    uart_putc(0x1A);  /* 0x1A = 26 = Ctrl+Z = end-of-SMS marker */

    /* Clear the accumulation buffer so we start fresh waiting for "+CMGS:" */
    memset(rx_buffer, 0, BUFFER_SIZE);

    /* Step 6: Wait up to 30 seconds for "+CMGS: <ref>" confirmation.
     * "+CMGS:" means the SMSC (SMS Centre) has accepted the message for delivery.
     * This can take several seconds on a roaming SIM or under poor signal. */
    if (wait_for_response("+CMGS:", 30000) != 0) {
        LOG_ERROR("SMS send failed (response: %.80s)", rx_buffer);
        return -1;  /* Either timed out or got ERROR back */
    }

    LOG_INFO("SMS sent to %s: %s", phone_number, message);
    return 0;  /* Message delivered to SMSC */
}

/*******************************************************************************
 * sim800l_close — Unmap the LW Bridge MMIO Region
 *
 * Calls munmap() to release the virtual address mapping created in sim800l_init().
 * After this, any access to lw_base would be a segfault — set to NULL to prevent.
 *
 * Note: we do NOT send any AT command here because the process is shutting down
 * and the UART may already be in an indeterminate state.
 *******************************************************************************/
void sim800l_close(void)
{
    if (lw_base != NULL) {
        /* Release the 2 MB virtual mapping of the LW bridge */
        munmap((void *)lw_base, LW_BRIDGE_SPAN);
        lw_base = NULL;  /* Prevent dangling pointer use after unmap */
        LOG_INFO("SIM800L closed (MMIO unmapped)");
    }
}
