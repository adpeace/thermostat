#ifndef RF69_H
#define RF69_H

void rf69_init(uint8_t sync_len, uint8_t sync_tol,
               const uint8_t *sync_val, uint8_t recv_packet_len);

/* Returns True if a packet has been received.  The packet is written to *out.
 * *sz should be the size of the *out buffer, and will be updated with the
 * number of bytes read. */
bool rf69_receiveDone(uint8_t *out, uint8_t *sz);

/* Transmits data, which is of size len.  If no_sync is set, we clear the sync
 * word first (the caller is then responsible for adding it to the packet) */
void rf69_transmit(const uint8_t *data, size_t len, bool no_sync);

#endif
