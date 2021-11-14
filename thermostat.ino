#include "rf69.h"

/*
 * Setup
 */

#define SERIAL_BAUD 57600

const uint8_t sync_val[] = {0x6c, 0xb6, 0xcb, 0x2c, 0x92, 0xd9};

void setup()
{
    Serial.begin(SERIAL_BAUD);
    while (!Serial)
        ;
    rf69_init(sizeof sync_val, 2, sync_val, 27);

    Serial.println("Danfoss thermostat transceiver");
}

/*
 * Packet definition and construction
 */
const static unsigned char thermostat_packet[] = {
    0xAA, 0xDD, 0x46, 0x00, 0x00, 0x00
};

/* Note that these positions applied to packets with the sync word included,
 * which isn't the case for those received over RF. */
#define THERMOSTAT_ID1_POS 3
#define THERMOSTAT_ID2_POS 4
#define COMMAND_POS 5

#define COMMAND_ON 0x33
#define COMMAND_LEARN 0x77
#define COMMAND_OFF 0xcc

/* out must be an array of length at least 3 * (sizeof thermostat_packet) */
static void mk_danfoss_packet(uint8_t *out, unsigned int thermid, uint8_t cmd)
{
    uint8_t packet[sizeof thermostat_packet];
    memcpy(packet, thermostat_packet, sizeof packet);
    packet[THERMOSTAT_ID1_POS] = thermid & 0xff;
    packet[THERMOSTAT_ID2_POS] = (thermid & 0xff00) >> 8;
    packet[COMMAND_POS] = cmd;
    encode_3b(packet, out, sizeof packet);
}

static void encode_3b(const uint8_t *in, uint8_t *out, size_t insz)
{
    unsigned int i;
    memset(out, 0, insz * 3);
    for (i = 0; i < insz * 8; i++) {
        unsigned int inval = in[i / 8] & (1 << (7 - (i % 8))) ? 1 : 0;
        unsigned int outbit = i * 3;
        /* Set out-value to 011 or 001: first bit already cleared,
         * last bit always 1: */
        out[(outbit + 1) / 8] |= inval << (7 - ((outbit + 1) % 8));
        out[(outbit + 2) / 8] |= 1 << (7 - ((outbit + 2) % 8));
    }
}

static void decode_3b(const uint8_t *in, uint8_t *out, size_t outsz)
{
    unsigned int i;
    memset(out, 0, outsz);
    for (i = 0; i < outsz * 8; i++) {
        unsigned int inbit = 1 + i * 3;
        uint8_t val = (in[inbit / 8] & (1 << (7 - (inbit % 8))))
                      ? 1 : 0;
        out[i / 8] |= val << (7 - (i % 8));
    }
}

/*
 * Command interface
 */

static const char *strcmd(char cmd)
{
    switch (cmd) {
        case 'O': return "ON";
        case 'X': return "OFF";
        case 'L': return "LEARN";
        default:  return "UNKNOWN";
    }
}

/* Command is O for on, X for off, L for learn. */
static void issue_command(char command, unsigned int thermid)
{
    uint8_t packet[6 * sizeof thermostat_packet];
    uint8_t packet_cmd = command == 'O' ? COMMAND_ON
                       : command == 'X' ? COMMAND_OFF
                       : command == 'L' ? COMMAND_LEARN : 0;
    mk_danfoss_packet(packet, thermid, packet_cmd);
    /* Transmit two copies back-to-back */
    memcpy(&packet[3 * sizeof thermostat_packet], packet,
           3 * sizeof thermostat_packet);
    rf69_transmit(packet, sizeof packet, true);
}

static void handle_command(const char *command)
{
    long thermostat_id;

    switch (toupper(command[0])) {
    case 'O': /* on */
    case 'X': /* off */
    case 'L': /* learn */
        thermostat_id = strtol(&command[1], NULL, 16);
        if (thermostat_id > 0 && thermostat_id < 0xffff) {
            /* valid thermostat ID, issue the command */
            Serial.print("ISSUE 0x");
            Serial.print(thermostat_id, HEX);
            Serial.print(" ");
            Serial.println(strcmd(toupper(command[0])));
            issue_command(toupper(command[0]), thermostat_id);
        }
        break;
    }
}

/* Build a line of input from serial, handling the command when it's done.
 * This avoids having a blocking readline call in loop(), which would
 * prevent us from receiving RF messages. */
#define SERIAL_RXBUF_SZ 16
char serial_rxbuf[SERIAL_RXBUF_SZ];
uint8_t serial_rxpos = 0;
static void handle_serial_char(char c)
{
    if (serial_rxpos < SERIAL_RXBUF_SZ) {
        serial_rxbuf[serial_rxpos++] = c == '\n' ? '\0' : c;
        if (c == '\n')
            handle_command(serial_rxbuf);
    }

    /* Command-delimination character always resets command state */
    if (c == '\n') {
        serial_rxpos = 0;
    }
}

void loop()
{
    uint8_t data[30];
    uint8_t data_sz = sizeof data;

    while (Serial.available() > 0) {
        int val = Serial.read();
        if (val >= 0)
            handle_serial_char((char)val);
    }

    if (rf69_receiveDone(data, &data_sz)) {
        uint8_t decoded[9], i;
        uint16_t thermostat_id;

        decode_3b(data, decoded, 9);

        /* Second half of message is repeated but may include an extra bit:
         * if the packet doesn't look correct already, then get rid of the
         * extra bit before checking for consistency. */
        if (decoded[3] != 0xAA && decoded[4] != 0xDD && decoded[5] != 0x46) {
          for (i = 3; i < 9; i++) {
              decoded[i] = decoded[i] << 1;
              if (i < 8)
                  decoded[i] |= decoded[i + 1] >> 7;
          }
        }

        /* Now decode it: check for message consistency */
        if (decoded[3] != 0xAA ||
            decoded[4] != 0xDD ||
            decoded[5] != 0x46 ||
            decoded[0] != decoded[6] ||
            decoded[1] != decoded[7])
            return;
        /* Check the thermostat ID */
        thermostat_id = (decoded[1] << 8) | decoded[0];
        Serial.print("RECV 0x");
        Serial.print(thermostat_id, HEX);
        if (decoded[2] == COMMAND_OFF)
            Serial.println(" OFF");
        else if (decoded[2] == COMMAND_LEARN)
            Serial.println(" LEARN");
        else if (decoded[2] == COMMAND_ON)
            Serial.println(" ON");
    }
}
