/* Simple driver for RF69 module for use with integration projects.  Note that
 * this assumes a single RF69 attached to the Arduino.
 *
 * Intended to be a very simple, easy to use implementation, that's enough to
 * provide necessary functionality to interact with Danfoss systems. */

#include <SPI.h>

/* SPI settings for RF69.  Note the 4MHz bus speed is lower than the 10MHz max
 * supported by the chip, but other libraries seem to operate stably at this
 * speed and since our data rate is low it seems wise to choose a safe value.
 **/
static SPISettings rf69_spi_settings(4000000, MSBFIRST, SPI_MODE0);

/* Relevant register numbers for the RF69 */
#define RF69_REG_FIFO          0x00
#define RF69_REG_OPMODE        0x01
#define RF69_REG_DATAMODUL     0x02
#define RF69_REG_BITRATE_MSB   0x03
#define RF69_REG_BITRATE_LSB   0x04
#define RF69_REG_FDEV_MSB      0x05
#define RF69_REG_FDEV_LSB      0x06
#define RF69_REG_FRF_MSB       0x07
#define RF69_REG_FRF_MID       0x08
#define RF69_REG_FRF_LSB       0x09
#define RF69_REG_RXBW          0x19

#define RF69_REG_IRQFLAGS1     0x27
#define RF69_REG_IRQFLAGS2     0x28

#define RF69_REG_PREAMBLE_LSB  0x2d
#define RF69_REG_SYNCCONFIG    0x2e
#define RF69_REG_SYNCVALUE1    0x2f
#define RF69_REG_PACKETCONFIG1 0x37
#define RF69_REG_PAYLOADLENGTH 0x38
#define RF69_REG_FIFOTHRESH    0x3c

enum mode {
    RF69_MODE_STANDBY  = 0x04,
    RF69_MODE_TRANSMIT = 0x0C,
    RF69_MODE_RECEIVE  = 0x10,
};

static void writeReg(uint8_t reg, uint8_t value)
{
    SPI.beginTransaction(rf69_spi_settings);
    digitalWrite(SS, LOW);
    SPI.transfer(reg | 0x80);
    SPI.transfer(value);
    digitalWrite(SS, HIGH);
    SPI.endTransaction();
}

static uint8_t readReg(uint8_t reg)
{
    uint8_t val;
    SPI.beginTransaction(rf69_spi_settings);
    digitalWrite(SS, LOW);
    SPI.transfer(reg);
    val = SPI.transfer(0);
    digitalWrite(SS, HIGH);
    SPI.endTransaction();
    return val;
}

void rf69_init(uint8_t sync_len, uint8_t sync_tol,
               const uint8_t *sync_val, uint8_t recv_packet_len)
{
    unsigned int i;

    SPI.begin();

    /* Ensure the module is initialised before we try to configure it: */
    do
        writeReg(RF69_REG_SYNCVALUE1, 0xaa);
    while (readReg(RF69_REG_SYNCVALUE1) != 0xaa);
    do
        writeReg(RF69_REG_SYNCVALUE1, 0x55);
    while (readReg(RF69_REG_SYNCVALUE1) != 0x55);

    /* Initialise registers */
    writeReg(RF69_REG_OPMODE,       (uint8_t)RF69_MODE_STANDBY);
    writeReg(RF69_REG_DATAMODUL,    0); /* pakcet mode, FSK, no shaping */
    writeReg(RF69_REG_BITRATE_MSB,  0x7d);
    writeReg(RF69_REG_BITRATE_LSB,  0x00);  /* 1000bps */
    writeReg(RF69_REG_FDEV_MSB,     0x01);
    writeReg(RF69_REG_FDEV_LSB,     0x9a);  /* 25Khz */
    writeReg(RF69_REG_FRF_MSB,      0x6c);
    writeReg(RF69_REG_FRF_MID,      0x7a);
    writeReg(RF69_REG_FRF_LSB,      0xff);  /* Approx 433.9Mhz */
    writeReg(RF69_REG_RXBW,         0x42);
    writeReg(RF69_REG_PACKETCONFIG1,0);     /* No packet filtering */
    writeReg(RF69_REG_IRQFLAGS2,    0x10);  /* Clear FIFO and flags */
    writeReg(RF69_REG_PREAMBLE_LSB, 0);     /* We generate our own preamble */
    writeReg(RF69_REG_SYNCCONFIG,   (1 << 7)   /* sync on */ |
                                    ((sync_len - 1) << 3) /* 6 bytes */ |
                                    sync_tol); /* error tolerance of 2 */
    for (i = 0; i < sync_len; i++) {
        writeReg(RF69_REG_SYNCVALUE1 + i, sync_val[i]);
    }
    /* Using fixed packet size for receive: */
    writeReg(RF69_REG_PAYLOADLENGTH, recv_packet_len);
    writeReg(RF69_REG_FIFOTHRESH,   0x80);  /* transmit as soon as FIFO
                                               non-empty */
    writeReg(0x6f, 0x20);

    /* Start in receive mode */
    writeReg(RF69_REG_OPMODE, (uint8_t)RF69_MODE_RECEIVE);
}

bool rf69_receiveDone(uint8_t *out, uint8_t *sz)
{
    if (readReg(RF69_REG_IRQFLAGS2) & 4 /* PayloadReady*/) {
        size_t max_sz = *sz, b = 0;
        /* Read data out of FIFO */
        while (b < max_sz &&
               readReg(RF69_REG_IRQFLAGS2) & 0x20 /* fifo not empty */) {
            out[b++] = readReg(RF69_REG_FIFO);
        }
        *sz = b;
        return true;
    }

    return false;
}

void rf69_transmit(const uint8_t *data, size_t len, bool no_sync)
{
    uint8_t orig_payload_len, orig_syncconf = 0, pos;

    /* Clear any current receive in progress */
    writeReg(RF69_REG_OPMODE, (uint8_t)RF69_MODE_STANDBY);
    writeReg(RF69_REG_IRQFLAGS2, 0x10);  /* Clear FIFO and flags */

    /* Disable sync word? */
    if (no_sync) {
        orig_syncconf = readReg(RF69_REG_SYNCCONFIG);
        writeReg(RF69_REG_SYNCCONFIG, 0);
    }

    /* Populate the FIFO with the data.  We assume len < size of fifo (66
     * bytes) */
    orig_payload_len = readReg(RF69_REG_PAYLOADLENGTH);
    writeReg(RF69_REG_PAYLOADLENGTH, len);
    SPI.beginTransaction(rf69_spi_settings);
    digitalWrite(SS, LOW);
    SPI.transfer(RF69_REG_FIFO | 0x80);
    for (pos = 0; pos < len; pos++) {
        SPI.transfer(data[pos]);
    }
    digitalWrite(SS, HIGH);
    SPI.endTransaction();

    /* Transition to transmit state, then poll for transmission complete */
    writeReg(RF69_REG_OPMODE, (uint8_t)RF69_MODE_TRANSMIT);
    while (!(readReg(RF69_REG_IRQFLAGS2) & 8 /* PacketSent */))
        ;

    /* Re-enable sync word? */
    if (no_sync) {
        writeReg(RF69_REG_OPMODE, (uint8_t)RF69_MODE_STANDBY);
        writeReg(RF69_REG_SYNCCONFIG, orig_syncconf);
    }

    /* Now back to receive mode */
    writeReg(RF69_REG_PAYLOADLENGTH, orig_payload_len);
    writeReg(RF69_REG_OPMODE, (uint8_t)RF69_MODE_RECEIVE);
}
