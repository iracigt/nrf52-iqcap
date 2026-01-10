# nRF52840 IQ Capture Tool

A tool using undocumented test features of the nRF52 RADIO core as a cheap one-shot 2.4GHz SDR

### Build

```sh
export ZEPHYR_BASE=.../ncs/v3.1.1/zephyr
west build -p --board promicro_nrf52840/nrf52840/uf2 --no-sysbuild
west build && udisksctl mount -b /dev/sd? && cp build/zephyr/zephyr.uf2 /run/media/*/NICENANO/
```

### Run

```sh
stty -F /dev/ttyACM0 raw
pio device monitor --quiet --raw --port /dev/ttyACM0 > test.wav
```

Status LED blinks three times at boot, then goes off until host attaches to the USB serial port.
LED comes on when armed. A falling edge on the trigger pin begins a capture. 
The status LED goes off on capture begin and back on once data transfer completes.

The data is a WAV file with I as the left and Q as the right channel.
Samples are 12 bits but sign extended to 16 bit linear PCM.
Discontinuities in the audio track, especially across channels, are indications of serial data being dropped or modified.
Common causes are POSIX stty flags such as `icrnl` or `igncr`.