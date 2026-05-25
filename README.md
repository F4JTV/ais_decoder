# AIS Decoder Module for SDR++

An AIS (Automatic Identification System / marine ship tracking, ITU-R M.1371)
decoder module for **SDR++**, with optional **TCP output** that streams decoded
contacts to an external collector (for example, a map backend).

## Features

- GMSK demodulation at 9600 baud on the two VHF AIS channels
  (AIS 1 = 161.975 MHz, AIS 2 = 162.025 MHz).
- Full decoding chain: NRZI decode -> HDLC flag sync -> bit de-stuffing ->
  CRC-16/X.25 check -> byte reconstruction -> field parsing (message types
  1-5, 18, 19, 21, 24).
- A movable/resizable **contacts window** showing each vessel with name/MMSI,
  reception time (UTC), latitude, longitude, SOG, COG, HDG, ship type, decoded
  message count, and navigational status.
- Quick-tune buttons for both AIS channels.
- A non-blocking **TCP client** (dedicated worker thread, automatic reconnect,
  bounded queue) that pushes one JSON record per positioned contact.
- Multiple instances are supported: run one module per channel, each with its
  own VFO and TCP connection.

## 1. Get SDR++ and add the module

The module is built in-tree, like the bundled `pager_decoder`.

```bash
git clone https://github.com/AlexandreRouma/SDRPlusPlus.git
cd SDRPlusPlus
cp -r /path/to/ais_decoder decoder_modules/ais_decoder
```

Wire the module into the build. The following edits are already applied if you
use the provided package together with `apply_to_sdrpp.sh`. Otherwise, add them
manually.

In the top-level `CMakeLists.txt`, in the first `# Decoders` section:

```cmake
option(OPT_BUILD_AIS_DECODER "Build the AIS decoder module with TCP output (no dependencies required)" ON)
```

In the second `# Decoders` section of the same file:

```cmake
if (OPT_BUILD_AIS_DECODER)
add_subdirectory("decoder_modules/ais_decoder")
endif (OPT_BUILD_AIS_DECODER)
```

In `core/src/core.cpp`, in the default module list:

```cpp
core::configManager.conf["modules"][modCount++] = "ais_decoder.so";
```

## 2. Dependencies (Ubuntu 24.04)

```bash
sudo apt update
sudo apt install -y build-essential cmake git pkg-config \
    libfftw3-dev libglfw3-dev libvolk-dev libzstd-dev \
    libairspy-dev libairspyhf-dev librtlsdr-dev libhackrf-dev \
    libiio-dev libad9361-dev libbladerf-dev liblimesuite-dev \
    libsoapysdr-dev libusb-1.0-0-dev
```

The AIS module itself has **no external dependencies**; all networking uses the
`net::` utility built into `sdrpp_core`. The packages above are only needed to
build SDR++ and its usual hardware source modules.

## 3. Build

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DOPT_BUILD_AIS_DECODER=ON
make -j$(nproc)
sudo make install
sudo ldconfig
```

The plugin is produced as `ais_decoder.so` and installed into
`/usr/lib/sdrpp/plugins/`.

## 4. Usage in SDR++

1. Launch SDR++.
2. If the module is not listed, open the **Module Manager**, select
   `ais_decoder`, give the instance a name (e.g. `AIS`), and click `+`.
3. In the module menu:
   - Use the **AIS 1 / AIS 2** buttons to tune the VFO to either channel.
   - Under **TCP output**, set the collector host and port and tick
     *Send decoded contacts*. The connection status and the Sent/Dropped
     counters are shown live.
   - Click **Show contacts window** to open the live contacts table in a
     separate, movable window.

To capture both channels at once, add **two instances** of the module via the
Module Manager (one per channel). Each has its own VFO and can point to the same
or different TCP collectors.

## 5. TCP output format

When TCP output is enabled, every decoded contact that carries a position is
sent as **one newline-terminated JSON object** (one record per line). Example:

```json
{"name":"EXAMPLE VESSEL","mmsi":227123456,"date":"2026-05-25","time":"14:58:25","lat":43.042958,"lon":7.019698,"type":"AIS","speed":14.7,"sog":14.7,"cog":253.8,"hdg":254,"shiptype":"Cargo","navstatus":"Under way (engine)","msgtype":1,"count":5,"info":"MMSI=227123456 msg=1 COG=253.8 HDG=254 nav=Under way (engine) ship=Cargo"}
```

Field reference:

| Key         | Type            | Description                                                       |
|-------------|-----------------|-------------------------------------------------------------------|
| `name`      | string          | Vessel name if known, otherwise `"MMSI:<number>"`                 |
| `mmsi`      | integer         | MMSI identifier                                                   |
| `date`      | string          | UTC reception date, `YYYY-MM-DD`                                  |
| `time`      | string          | UTC reception time, `HH:MM:SS`                                    |
| `lat`       | float           | Latitude in degrees (6 decimals)                                 |
| `lon`       | float           | Longitude in degrees (6 decimals)                                |
| `type`      | string          | Always `"AIS"`                                                   |
| `speed`     | float \| null   | Speed Over Ground in knots (backward-compatible alias of `sog`)  |
| `sog`       | float \| null   | Speed Over Ground in knots                                       |
| `cog`       | float \| null   | Course Over Ground (direction) in degrees                        |
| `hdg`       | integer \| null | True heading in degrees                                          |
| `shiptype`  | string \| null  | Ship type label (e.g. "Cargo", "Tanker", "Passenger")            |
| `navstatus` | string \| null  | Navigational status label (e.g. "Under way (engine)")            |
| `msgtype`   | integer         | Number of the last AIS message type received (1, 5, 18, ...)     |
| `count`     | integer         | Count of decoded messages for this MMSI since start              |
| `info`      | string          | Compact human-readable summary                                   |

Notes:
- Unavailable numeric/label fields are emitted as JSON `null`.
- Only messages that contain a position trigger a TCP record.
- Records are line-delimited: the collector should read and parse one line at
  a time.

## 6. Collector side

The module acts as a TCP **client**; the collector must run a TCP **server**.
The server must accept multiple simultaneous connections if you run more than
one module instance (one connection per instance). A minimal multi-client test
server (`ais_test_server.py`) is provided alongside this module.

```bash
# Quick local test (prints each line, labelled by client):
python3 ais_test_server.py 10110
```

Then point the module's TCP output to the server's IP and port (e.g.
`127.0.0.1:10110`).

> Tip: `nc -l` (and even `nc -k -l`) only relays one connection's output to the
> terminal at a time, so it is not a reliable way to test two module instances.
> Use the provided multi-client server instead.

## 7. Validation

Two test programs are included in the module folder (they are not part of the
CMake build):

- `test_decoder.cpp` -- encodes known AIS messages (type 1 position report,
  type 5 static/voyage) into HDLC+NRZI channel bits, decodes them, and verifies
  each parsed field, plus rejection of a CRC-corrupted frame:
  ```bash
  g++ -std=c++17 -I src -O2 test_decoder.cpp src/ais/decoder.cpp -o ais_test && ./ais_test
  ```
- `test_tcp.cpp` together with `test_server.py` -- exercises the TCP client
  (connect, queueing, reconnect).

## Technical notes

- **Bit order**: each AIS byte is transmitted LSB-first while the message is
  defined MSB-first; byte reconstruction packs the received LSB-first bits to
  recover the big-endian bitfield (matching gnuais / gr-ais).
- **NRZI**: decoded internally (transition = 0, no transition = 1), which makes
  decoding insensitive to FM discriminator polarity inversion.
- **CRC**: reflected HDLC FCS (polynomial 0x8408); a valid frame yields the
  magic residue `0xF0B8`.
- **Sampling**: 48 kHz (5 samples/symbol), RRC matched filter (beta = 0.4),
  Mueller & Muller symbol clock recovery.
- **TCP client**: the SDR++ socket is non-blocking. When the send buffer is
  momentarily full (`EWOULDBLOCK`), the worker keeps the connection and retries
  rather than treating it as a disconnect; partial writes are re-queued so
  framing is preserved.
