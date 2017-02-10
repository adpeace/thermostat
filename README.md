# Danfoss Thermostat Emulation

The Arduino sketch provided here programs an RF69 receiver to transmit, and
receive and decode, RF packets to/from Danfoss RF thermostats similar to the
TP7000.

The sketch by default uses a baud-rate of 57600bps.  The interface is simple:
for each received RF packet a line will be printed like `RECV TTTT CMD` where
`TTTT` is the thermostat ID and `CMD` is either `ON`, `OFF`, or `LEARN`.

You can transmit packets by sending `CTTTT\n`, where `C` is `O` for On, `X` for
Off, or `L` for Learn, and `TTTT` is the thermostat ID.

See https://hackingathome.wordpress.com for more info.

## Example serial output

```
Danfoss thermostat transceiver
RECV 0x77B4 OFF
RECV 0x77B4 OFF
RECV 0x77B4 OFF
ISSUE 0x77B4 ON
RECV 0x77B4 OFF
RECV 0x53C0 OFF
RECV 0x53C0 OFF
RECV 0x77B4 OFF
RECV 0x77B4 OFF
RECV 0x53C0 OFF
RECV 0x77B4 OFF
RECV 0x53C0 OFF
RECV 0x77B4 OFF
```

