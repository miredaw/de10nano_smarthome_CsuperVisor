/*******************************************************************************
 * esp32_handler.c — ESP32 WiFi and MQTT Bridge via AT Commands
 *
 * ============================================================================
 * BIG PICTURE — WHAT THIS FILE DOES
 * ============================================================================
 *
 * This module controls an Espressif ESP32 module connected to the DE10-Nano
 * via HPS UART1 (/dev/ttyS1).  The ESP32 runs Espressif's official AT-command
 * firmware, which means it acts as a "serial-to-WiFi/MQTT" bridge:
 *
 *   DE10-Nano ARM Linux
 *       |
 *       | /dev/ttyS1 (UART1, 115200 baud, 8N1)
 *       |
 *   ESP32 (AT firmware)
 *       |
 *       | WiFi 802.11
 *       |
 *   MQTT Broker (Mosquitto on VPS)
 *
 * The ARM CPU sends text AT commands like:
 *   "AT+CWJAP=\"ssid\",\"pass\"\r\n"      -> join WiFi network
 *   "AT+MQTTCONN=0,\"ip\",1883,1\r\n"     -> connect to MQTT broker
 *   "AT+MQTTPUBRAW=0,\"topic\",N,0,0\r\n" -> publish message header
 *   followed by N raw JSON bytes           -> payload bytes
 *
 * The ESP32 responds with "OK" on success or "ERROR" on failure.
 *
 * ============================================================================
 * KEY DESIGN DECISIONS
 * ============================================================================
 *
 * 1. AT+MQTTPUBRAW instead of AT+MQTTPUB:
 *    AT+MQTTPUB embeds the JSON payload inside a quoted AT string parameter.
 *    JSON commas and braces confuse the ESP-AT parser. AT+MQTTPUBRAW sends
 *    the payload length first, then streams the raw bytes after a ">" prompt.
 *    No escaping is needed.
 *
 * 2. clock_gettime() for timeouts instead of sleep-counting:
 *    The old approach incremented an 'elapsed' counter by the usleep() duration.
 *    But each read() call can block for up to VTIME*100ms regardless of data.
 *    clock_gettime(CLOCK_MONOTONIC) measures actual wall-clock time, making
 *    timeouts behave exactly as specified.
 *
 * 3. AT+MQTTCLEAN before AT+MQTTUSERCFG:
 *    AT firmware v4.x returns ERROR on USERCFG if link_id=0 already exists.
 *    Always send AT+MQTTCLEAN=0 first. ERROR on first boot is expected/silent.
 *
 * Physical wiring (JP1):
 *   Pin 21 (PIN_C12)  = HPS UART1 TX -> ESP32 RX
 *   Pin 22 (PIN_AD17) = ESP32 TX -> HPS UART1 RX
 *
 *******************************************************************************/

/* Must be defined before any system header to unlock usleep() and CRTSCTS */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <stdio.h>      /* printf(), perror(), snprintf()                        */
#include <stdlib.h>     /* NULL                                                  */
#include <string.h>     /* memset(), strlen(), strstr()                          */
#include <unistd.h>     /* write(), read(), close(), sleep()                     */
#include <fcntl.h>      /* open(), O_RDWR, O_NOCTTY                              */
#include <termios.h>    /* struct termios, cfsetospeed(), tcsetattr(), etc.      */
#include <errno.h>      /* errno, perror()                                       */
#include <stdbool.h>    /* bool, true, false                                     */
#include <time.h>       /* clock_gettime(), struct timespec, CLOCK_MONOTONIC     */

#include "smart_home.h" /* LOG_INFO, LOG_WARN, LOG_ERROR macros                  */

/* /dev/ttyS0 = Linux console (reserved). /dev/ttyS1 = HPS UART1, free for use.
 * Make sure the kernel cmdline does NOT have console=ttyS1. */
#define UART_ESP32_DEVICE   "/dev/ttyS1"  /* HPS UART1 device node               */
#define BUFFER_SIZE         1024          /* AT response accumulation buffer size */

static int uart_fd = -1;            /* /dev/ttyS1 file descriptor (-1 = closed) */
static char rx_buffer[BUFFER_SIZE]; /* Accumulates bytes received from ESP32     */

/*******************************************************************************
 * esp32_init — Open and Configure UART, Probe ESP32
 *
 * Opens /dev/ttyS1, configures it for 115200 8N1 raw mode, then sends "AT"
 * and waits for "OK" to verify the ESP32 AT firmware is responsive.
 *
 * TERMIOS SETTINGS:
 *   c_cflag: baud=115200, 8 data bits, no parity, 1 stop bit, no HW flow ctrl
 *   c_lflag: raw mode (no echo, no canonical, no signals)
 *   c_iflag: no input translations, no SW flow control
 *   c_oflag: no output post-processing
 *   VTIME=1 / VMIN=0: read() returns after 100ms max with whatever data arrived
 *
 * Returns 0 on success, -1 on failure.
 *******************************************************************************/
int esp32_init(void)
{
    struct termios tty;  /* POSIX terminal settings structure */

    /* Open the UART: O_RDWR for bidirectional I/O, O_NOCTTY so this process
     * doesn't accidentally become the controlling terminal for the port */
    uart_fd = open(UART_ESP32_DEVICE, O_RDWR | O_NOCTTY);
    if (uart_fd < 0) {
        perror("ESP32: open UART");  /* e.g. "No such file or directory" or "Permission denied" */
        return -1;
    }

    /* Fetch current terminal attributes so we can modify only what we need */
    if (tcgetattr(uart_fd, &tty) != 0) {
        perror("ESP32: tcgetattr");
        close(uart_fd);
        uart_fd = -1;
        return -1;
    }

    /* Set TX and RX baud rate to 115200 bps (must match ESP32 AT firmware default) */
    cfsetospeed(&tty, B115200);  /* Output speed */
    cfsetispeed(&tty, B115200);  /* Input speed  */

    /* c_cflag — hardware configuration */
    tty.c_cflag &= ~PARENB;        /* No parity bit */
    tty.c_cflag &= ~CSTOPB;        /* 1 stop bit (not 2) */
    tty.c_cflag &= ~CSIZE;         /* Clear data size bits */
    tty.c_cflag |= CS8;            /* 8 data bits per character */
    tty.c_cflag &= ~CRTSCTS;       /* No hardware RTS/CTS flow control */
    tty.c_cflag |= CREAD | CLOCAL; /* Enable receiver; ignore modem control lines */

    /* c_lflag — line discipline: raw mode, no processing */
    tty.c_lflag &= ~(ICANON   /* No canonical (line-buffered) mode */
                   | ECHO     /* No character echo */
                   | ECHOE    /* No visual erase (BS SP BS) */
                   | ECHONL   /* No newline echo */
                   | ISIG);   /* No SIGINT/SIGQUIT from Ctrl+C / Ctrl+\ */

    /* c_iflag — input: disable all software flow control and byte translations */
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);  /* No XON/XOFF software flow control */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    /* ICRNL disabled: do NOT convert CR (\r) to NL (\n) on input —
     * AT responses use \r\n and we need to see the \r characters intact */

    /* c_oflag — output: no post-processing (send bytes as-is to UART TX) */
    tty.c_oflag &= ~(OPOST | ONLCR); /* No output processing, no NL->CR+NL */

    /* Timeout/blocking behavior for read():
     * VTIME=1: timer in 0.1s units -> return after 100ms if no new byte arrives
     * VMIN=0:  minimum 0 bytes -> read() can return 0 (don't block indefinitely) */
    tty.c_cc[VTIME] = 1;   /* 100ms timeout per read() call */
    tty.c_cc[VMIN]  = 0;   /* Non-blocking (rely on VTIME for timing) */

    /* Apply settings immediately (TCSANOW = no drain wait) */
    if (tcsetattr(uart_fd, TCSANOW, &tty) != 0) {
        perror("ESP32: tcsetattr");
        close(uart_fd);
        uart_fd = -1;
        return -1;
    }

    /* Discard any stale bytes in the OS UART RX/TX buffers */
    tcflush(uart_fd, TCIOFLUSH);
    LOG_INFO("ESP32 UART opened on %s at 115200 baud", UART_ESP32_DEVICE);

    sleep(1);  /* Allow ESP32 to finish any initialization after power-on */

    /* Basic AT probe: send "AT\r\n" and expect "OK" within 2 seconds */
    if (esp32_send_at_command("AT", "OK", 2000) != 0) {
        LOG_ERROR("ESP32 not responding — check wiring on JP1 pins 21/22");
        close(uart_fd);
        uart_fd = -1;
        return -1;
    }

    LOG_INFO("ESP32 AT probe OK");
    return 0;
}

/*******************************************************************************
 * esp32_send_at_command — Transmit AT Command, Wait for Expected Response
 *
 * Protocol:
 *   1. Clear rx_buffer and flush the UART OS buffers.
 *   2. Append \r\n to cmd and write to UART.
 *   3. Read bytes in a loop, accumulate in rx_buffer.
 *   4. After each chunk: check for expected_response (return 0) or "ERROR" (-1).
 *   5. If timeout_ms wall-clock time elapses: return -1.
 *
 * Timeout uses clock_gettime(CLOCK_MONOTONIC) — measures real elapsed time
 * correctly even when each read() blocks for up to 100ms (VTIME=1).
 *
 * elapsed_ms = (now.sec - start.sec)*1000 + (now.nsec - start.nsec)/1000000
 *
 * Returns 0 on success, -1 on timeout or ERROR.
 * Side effect: rx_buffer holds the received data (useful for callers that
 * need to parse the response content, e.g. AT+CREG? in sim800l).
 *******************************************************************************/
int esp32_send_at_command(const char *cmd, const char *expected_response,
                           int timeout_ms)
{
    if (uart_fd < 0) return -1;  /* Guard: UART must be open */

    memset(rx_buffer, 0, BUFFER_SIZE);  /* Clear previous response data */
    tcflush(uart_fd, TCIOFLUSH);        /* Discard stale RX bytes (unsolicited msgs) */

    /* Build "cmd\r\n" in a local buffer — AT commands require CR+LF termination */
    char cmd_crlf[300];
    snprintf(cmd_crlf, sizeof(cmd_crlf), "%s\r\n", cmd);

    /* Transmit the command to the ESP32 */
    if (write(uart_fd, cmd_crlf, strlen(cmd_crlf)) < 0) {
        perror("ESP32: write");
        return -1;
    }

    /* Start the monotonic clock for accurate timeout measurement */
    struct timespec t_start, t_now;
    clock_gettime(CLOCK_MONOTONIC, &t_start);

    int total = 0;  /* Bytes accumulated in rx_buffer */

    while (1) {
        /* Calculate real elapsed time in milliseconds */
        clock_gettime(CLOCK_MONOTONIC, &t_now);
        long elapsed_ms = (t_now.tv_sec  - t_start.tv_sec)  * 1000L
                        + (t_now.tv_nsec - t_start.tv_nsec) / 1000000L;
        if (elapsed_ms >= (long)timeout_ms) break;  /* Deadline exceeded */

        /* Read whatever bytes are available (up to buffer space remaining).
         * Returns 0 if VTIME expires with no data — just loop again. */
        int n = read(uart_fd, rx_buffer + total, BUFFER_SIZE - total - 1);
        if (n > 0) {
            total += n;
            rx_buffer[total] = '\0';  /* Null-terminate for strstr() searches */

            if (strstr(rx_buffer, expected_response) != NULL)
                return 0;  /* Found the expected response string */

            if (strstr(rx_buffer, "ERROR") != NULL) {
                LOG_WARN("ESP32 ERROR: %.80s", rx_buffer);
                return -1;  /* AT firmware rejected the command */
            }
        }
    }

    LOG_WARN("ESP32 timeout waiting for '%s'. Got: %.80s", expected_response, rx_buffer);
    return -1;  /* Timeout */
}

/*******************************************************************************
 * esp32_connect_wifi — Join a WiFi Access Point
 *
 * AT+CWMODE=1  : Station mode (connect to existing AP, not create a hotspot)
 * AT+CWJAP     : Join AP with given SSID and WPA2 password
 *                Timeout 20s: DHCP association can take up to ~15 seconds.
 *                ESP32 sends "WIFI CONNECTED", "WIFI GOT IP", then "OK".
 *
 * Returns 0 on success, -1 on failure.
 *******************************************************************************/
int esp32_connect_wifi(const char *ssid, const char *password)
{
    char cmd[256];

    /* Set station mode: mode 1 = client (connect to AP).
     * Mode 2 = softAP, Mode 3 = both. 5s timeout. */
    if (esp32_send_at_command("AT+CWMODE=1", "OK", 5000) != 0) return -1;

    /* Join the network.  Double-quote the SSID and password in the AT command.
     * The backslash-escaped quotes inside the format string produce:
     *   AT+CWJAP="Miredaw","Milad3633"  (sent over UART) */
    snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"", ssid, password);
    if (esp32_send_at_command(cmd, "OK", 20000) != 0) {
        LOG_ERROR("WiFi connection failed for SSID '%s'", ssid);
        return -1;
    }

    LOG_INFO("WiFi connected to '%s'", ssid);
    return 0;
}

/*******************************************************************************
 * esp32_mqtt_connect — Connect to the Authenticated MQTT Broker
 *
 * Steps:
 *   1. AT+MQTTCLEAN=0 : tear down any previous MQTT connection (silent).
 *   2. AT+MQTTUSERCFG : set client ID, username, password, TLS scheme.
 *   3. AT+MQTTCONN    : establish TCP connection to broker.
 *
 * TLS scheme: 1=plain TCP, 4=TLS (no CA verify), 5=TLS (verify server cert).
 * We use scheme 1 or 4 depending on MQTT_USE_TLS in smart_home.h.
 *
 * Returns 0 on success, -1 on failure.
 *******************************************************************************/
int esp32_mqtt_connect(const char *broker, int port)
{
    char cmd[300];

    /* Step 1: Clean up prior connection.
     * AT firmware v4.x fails MQTTUSERCFG if link_id=0 already exists.
     * On first boot there's no prior connection so ERROR is expected — handle
     * it silently by writing directly without calling the logging wrapper. */
    tcflush(uart_fd, TCIOFLUSH);
    {
        const char *clean_cmd = "AT+MQTTCLEAN=0\r\n";
        write(uart_fd, clean_cmd, strlen(clean_cmd));  /* Send cleanup */

        /* Wait up to 3s for OK or ERROR (either is acceptable) */
        struct timespec t0, tn;
        clock_gettime(CLOCK_MONOTONIC, &t0);
        char tmp[64]; int tot = 0;
        memset(tmp, 0, sizeof(tmp));
        while (1) {
            clock_gettime(CLOCK_MONOTONIC, &tn);
            if ((tn.tv_sec - t0.tv_sec) * 1000L +
                (tn.tv_nsec - t0.tv_nsec) / 1000000L >= 3000L) break;
            int n = read(uart_fd, tmp + tot, (int)sizeof(tmp) - tot - 1);
            if (n > 0) {
                tot += n; tmp[tot] = '\0';
                if (strstr(tmp, "OK") || strstr(tmp, "ERROR")) break;
            }
        }
    }  /* tmp goes out of scope here — cleanup result discarded */

    /* scheme 1 = plain TCP (port 1883), scheme 4 = TLS no-verify (port 8883) */
    int scheme = (MQTT_USE_TLS) ? 4 : 1;

    /* Step 2: Set MQTT client parameters.
     * AT+MQTTUSERCFG=<link_id>,<scheme>,<client_id>,<user>,<pass>,<certId>,<caId>,<path> */
    snprintf(cmd, sizeof(cmd),
             "AT+MQTTUSERCFG=0,%d,\"%s\",\"%s\",\"%s\",0,0,\"\"",
             scheme, MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS);
    if (esp32_send_at_command(cmd, "OK", 5000) != 0) {
        LOG_ERROR("MQTT user config failed — check MQTT_USER/MQTT_PASS in smart_home.h");
        return -1;
    }

    /* Step 3: Connect to broker.
     * AT+MQTTCONN=<link_id>,<host>,<port>,<reconnect=1>
     * reconnect=1 enables automatic reconnection if TCP drops */
    snprintf(cmd, sizeof(cmd), "AT+MQTTCONN=0,\"%s\",%d,1", broker, port);
    if (esp32_send_at_command(cmd, "OK", 15000) != 0) {
        LOG_ERROR("MQTT connect failed — check broker IP/port and VPS firewall");
        return -1;
    }

    LOG_INFO("MQTT connected to %s:%d (TLS=%d)", broker, port, MQTT_USE_TLS);
    return 0;
}

/*******************************************************************************
 * esp32_mqtt_publish — Publish a JSON Payload via AT+MQTTPUBRAW
 *
 * AT+MQTTPUBRAW uses a 2-phase protocol to avoid JSON parsing issues:
 *
 *   Phase 1 — Send command header with payload length:
 *     ARM -> ESP32: "AT+MQTTPUBRAW=0,"topic",<len>,0,0\r\n"
 *     ESP32 -> ARM: ">"   (ready to receive payload bytes)
 *
 *   Phase 2 — Stream raw JSON bytes:
 *     ARM -> ESP32: <exactly len bytes of JSON>
 *     ESP32 -> ARM: "OK"  (published to broker)
 *
 * Phase 3 (custom read loop) waits for "OK" without re-calling
 * esp32_send_at_command(), which would flush the UART and lose the "OK".
 *
 * Returns 0 on success, -1 on failure.
 *******************************************************************************/
int esp32_mqtt_publish(const char *topic, const char *payload)
{
    if (uart_fd < 0) return -1;

    int plen = (int)strlen(payload);  /* Exact byte count to send in phase 2 */

    /* Phase 1: send the PUBRAW header, wait for ">" prompt */
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "AT+MQTTPUBRAW=0,\"%s\",%d,0,0", topic, plen);
    if (esp32_send_at_command(cmd, ">", 5000) != 0) {
        LOG_WARN("MQTT publish: no '>' prompt for topic '%s'", topic);
        return -1;
    }

    /* Phase 2: stream the raw JSON bytes — no \r\n, no escaping */
    if (write(uart_fd, payload, (size_t)plen) != plen) {
        perror("ESP32: write MQTT payload");
        return -1;
    }

    /* Phase 3: custom read loop for "OK" — cannot reuse esp32_send_at_command()
     * because it calls tcflush() which would discard the incoming "OK" bytes */
    struct timespec t_start, t_now;
    clock_gettime(CLOCK_MONOTONIC, &t_start);
    memset(rx_buffer, 0, BUFFER_SIZE);
    int total = 0;

    while (1) {
        clock_gettime(CLOCK_MONOTONIC, &t_now);
        long elapsed_ms = (t_now.tv_sec  - t_start.tv_sec)  * 1000L
                        + (t_now.tv_nsec - t_start.tv_nsec) / 1000000L;
        if (elapsed_ms >= 5000L) break;  /* 5 second timeout */

        int n = read(uart_fd, rx_buffer + total, BUFFER_SIZE - total - 1);
        if (n > 0) {
            total += n;
            rx_buffer[total] = '\0';
            if (strstr(rx_buffer, "OK"))    return 0;   /* Published successfully */
            if (strstr(rx_buffer, "ERROR")) {
                LOG_WARN("MQTT pubraw ERROR for '%s': %.80s", topic, rx_buffer);
                return -1;
            }
        }
    }

    LOG_WARN("MQTT pubraw timeout for '%s'. Got: %.80s", topic, rx_buffer);
    return -1;
}

/*******************************************************************************
 * esp32_close — Gracefully Disconnect MQTT and WiFi, Release UART
 *
 * AT+MQTTCLEAN=0 : send MQTT DISCONNECT to broker (clean session end)
 * AT+CWQAP       : disassociate from WiFi AP
 * close()        : release the /dev/ttyS1 file descriptor
 *******************************************************************************/
void esp32_close(void)
{
    if (uart_fd >= 0) {
        esp32_send_at_command("AT+MQTTCLEAN=0", "OK", 3000);  /* Disconnect MQTT  */
        esp32_send_at_command("AT+CWQAP",       "OK", 3000);  /* Disconnect WiFi  */
        close(uart_fd);   /* Close the UART file descriptor */
        uart_fd = -1;     /* Mark as closed */
        LOG_INFO("ESP32 closed");
    }
}
