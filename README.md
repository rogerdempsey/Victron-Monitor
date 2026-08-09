# Victron BLE Monitor + Boat Systems Dashboard (ESP32)

Reads live data from a Victron battery shunt (SmartShunt / BMV) and a
SmartSolar/BlueSolar MPPT controller over Bluetooth Low Energy "Instant
Readout" broadcasts (no pairing, no cables to the Victron gear needed), and
serves a live dashboard at **http://192.168.8.66/**.

It also listens for fuel-flow/tank-level data pushed over UDP from a
diesel heater board, and fridge/crisper data pushed from an icebox
controller (separate small ESP boards on the boat's network, each with
their own local dashboard too), plus reads 3 DS18B20 temperature probes
wired directly to this ESP32 (Cabin1, Cabin2, Outside) — folding all of
it into the same dashboard.

The dashboard shows **GPS** and **AIS** pills near the top that turn
green when data is actively coming in and red when it isn't. Both are
pure listeners: this board doesn't have a wired GPS receiver and doesn't
broadcast anything itself — it listens for an existing NMEA0183-over-UDP
feed (GPS and AIS sentences together, from whatever upstream source is
already broadcasting them on the network) on port **10110**, the same
feed any chartplotter software like OpenCPN would listen to directly.
GPS sentences get parsed just enough to show local time (Pacific, with
automatic PST/PDT handling) straight from the GPS's own clock — no
internet/NTP needed. AIS sentences are never parsed at all, just noticed
— the AIS pill is a pure connectivity indicator.

The board is set up to recover on its own from most common failures
(stuck Wi-Fi connect, a genuinely wedged main loop) rather than needing
someone to notice and power-cycle it — see **Reliability** below.

Wi-Fi credentials and your Victron devices' MAC addresses/encryption keys
go in `include/secrets.h` (gitignored, kept out of version control) —
see step 2 below.

## 1. Install PlatformIO

Easiest path: install the **PlatformIO IDE extension** in VS Code
(Extensions -> search "PlatformIO IDE" -> Install). It will handle the
ESP32 toolchain and the required libraries (`scottp/victronble`,
`paulstoffregen/OneWire`, `milesburton/DallasTemperature`) automatically
based on `platformio.ini`.

## 2. Set up your credentials

Wi-Fi credentials and Victron device keys live in `include/secrets.h`,
which is gitignored so they never end up committed anywhere. Copy the
template and fill it in:

```bash
cp include/secrets.h.example include/secrets.h
```

Then open `include/secrets.h` and fill in:

```cpp
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
```

(Victron `SOLAR_MAC`/`SOLAR_KEY`/`SHUNT_MAC`/`SHUNT_KEY` are also set
there — see step 6 below for where to get those.)

## 3. Check your network range

The static IP is set to `192.168.8.66` with gateway `192.168.8.1` and
subnet `255.255.255.0`. If your router's network isn't `192.168.8.x`
(check your router admin page, or the IP your phone gets), update these
lines in `src/main.cpp`:

```cpp
IPAddress local_IP(192, 168, 8, 66);
IPAddress gateway(192, 168, 8, 1);
IPAddress subnet(255, 255, 255, 0);
```

Make sure `192.168.8.66` isn't already used by another device and isn't
inside your router's DHCP lease range (or set a DHCP reservation/exclusion
for it instead).

## 4. Board selection

`platformio.ini` targets a generic ESP32 dev module (`board = esp32dev`).
If you're using an ESP32-S3, ESP32-C3, or another variant, change the
`[env:esp32dev]` name and `board =` line to match your board
(e.g. `board = esp32-s3-devkitc-1`).

## 5. Build & upload

With the project folder open in VS Code / PlatformIO:

- Click the checkmark (Build) icon, then the right-arrow (Upload) icon in
  the PlatformIO toolbar, with the ESP32 plugged in via USB.
- Or from a terminal in this folder: `pio run -t upload`
- Open the Serial Monitor (115200 baud) to watch it connect to Wi-Fi and
  print its IP address.

## 6. Enable Instant Readout on both Victron devices

If you haven't already (you'll need this to have gotten the encryption
keys in the first place):

1. Open VictronConnect and connect to the device.
2. Settings -> Product Info.
3. Enable "Instant readout via Bluetooth".
4. Disconnect the VictronConnect app from the device afterwards — the
   BLE beacon and the ESP32's connectionless scan both work fine while
   disconnected, but if VictronConnect is actively connected it can
   interfere.

## 7. View the dashboard

Once the ESP32 has connected to Wi-Fi and picked up the first BLE
broadcasts (can take a few seconds), open:

**http://192.168.8.66/**

It shows all connected devices, auto-refreshing every 2 seconds:

- **Shunt**: voltage, current, power, state of charge, consumed Ah, time
  remaining, starter battery voltage, active alarms, signal strength.
- **Solar charger**: charge state (Bulk/Absorption/Float/etc.), battery
  voltage/current, max charge current (last 10h), yield today, plus the
  3 cabin/outside temperature probes (see below) under a "Temperatures"
  divider further down the same card — folded in together since they
  were built at the same time and there was room. Panel power isn't in
  this card, but is still shown live in the flow diagram at the top of
  the page.
- **Diesel heater**: fuel burn rate, litres used today, lifetime litres
  used, and fuel tank level (the highest reading seen in the last hour,
  read from the heater board's own NMEA2000 tank-level PGN).
- **Icebox**: fridge and crisper compartment temperatures, compressor
  on/off state, and duty cycle.

Above the cards, a small animated flow diagram shows current moving
between **Solar**, **Battery**, and **Loads** (with an amps/watts
readout under each), plus a fourth node, **Engine** — this one's
calculated, not measured. Whenever the shunt's charging current is
higher than what solar alone is providing, the difference is attributed
to the engine's alternator, and its flow line only ever goes to the
battery (it doesn't feed loads or interact with solar directly in the
diagram). Like the Loads estimate, it needs both the shunt and solar
charger reporting live data to compute anything, and shows "need both
devices" otherwise.

A status dot next to each device name turns green when data is fresh and
red if it hasn't updated in the last 15 seconds (e.g. out of BLE range,
or the heater/icebox board is offline). The 3 temperature probes don't
share a single dot — each shows its own "no probe" if that specific
sensor drops out, since they're wired independently.

The footer at the bottom of the page shows uptime and a heap health
snapshot: free heap at boot, free heap right now, and the largest
allocatable block. These matter for spotting trouble before it causes a
slowdown or crash — see **Reliability** below for what to watch for.

## Temperature probe wiring (Cabin1 / Cabin2 / Outside)

All 3 DS18B20 probes share a single wire on **ESP32 GPIO4**. Unlike the
GPS, these connect directly to the ESP32 — DS18B20 data lines are
already 3.3V-safe, no level converter needed. What they *do* need is a
pull-up resistor, since 1-Wire is an open-drain bus:

- Probe **VCC** -> ESP32 3.3V
- Probe **GND** -> ESP32 GND
- Probe **DATA** -> ESP32 GPIO4, **and** a ~4.7kΩ resistor from GPIO4 to
  3.3V (one resistor total, shared by all 3 probes on the bus — not one
  per probe)

All 3 probes wire to the same 3 points in parallel (VCC together, GND
together, DATA together) — that's the point of 1-Wire, each probe has a
unique factory-set address so they can share one bus.

**Probe identification**: each probe is matched to Cabin1/Cabin2/Outside
by its actual hardware address, not by the order it happens to enumerate
on the bus — so it doesn't matter what order they're wired in, and one
probe temporarily dropping off the bus can't cause the others to get
mislabeled. Each probe's 6-byte serial number (printed on the probe
itself) is hardcoded in `TEMP_PROBE_SERIAL` near the top of
`src/main.cpp`, matched to the `Cabin1`/`Cabin2`/`Outside` names in
`TEMP_PROBE_NAMES` right above it (same order, index for index). If you
ever replace a probe, update its serial number there and reflash — the
Serial Monitor at boot confirms whether each *named* probe was actually
found, rather than just listing whatever it happened to discover.

## How the heater and icebox boards connect

Unlike the GPS feed (broadcast to the whole network, since multiple
programs might want it) or the old design (this board polling each of
them over HTTP), the heater and icebox boards **push** their data here
over plain UDP unicast — fire-and-forget, no TCP connection or socket
lifecycle on either side, sent straight to this board's IP
(`192.168.8.66`) on port **2000**. This board just listens; there's
nothing to configure here for it to work, but it's worth understanding
for troubleshooting:

- Both boards send to the *same* port; this board tells them apart by
  which IP the packet came from (`HEATER_IP`/`ICEBOX_IP` near the top of
  `src/main.cpp`).
- If either board's Wi-Fi is down, or it can't reach this board's IP,
  its card here just goes stale (same "hasn't updated in 15s" red-dot
  behavior as everything else) — there's no retry from this end to
  configure, since it's the *other* board initiating each send.
- If this board's static IP (`192.168.8.66`) is ever changed, the
  matching `DASHBOARD_IP` constant needs updating on **both** the heater
  and icebox boards' own firmware too, or their telemetry has nowhere to
  go.
- Some routers/access points have a "client isolation" or "AP isolation"
  setting that blocks device-to-device traffic on the same Wi-Fi network
  (it's meant for public/guest networks). If telemetry never arrives
  despite everything looking fine on both ends, that setting is worth
  checking and temporarily disabling to test.

## How GPS and AIS reach this board

This board has no wired GPS receiver and doesn't originate or
re-broadcast anything itself — it's a pure listener for an existing
NMEA0183-over-UDP feed on port **10110**, the standard port for this
purpose. Both GPS sentences (`$...`) and AIS sentences (`!...`) are
expected on the same feed, told apart by that first character, which is
a standard NMEA0183 convention:

- GPS sentences get parsed just enough to pull out local time (from
  `$--RMC`) for the dashboard's clock, and drive the GPS pill.
- AIS sentences are never parsed — only noticed, purely to drive the AIS
  pill.

A single UDP packet can contain more than one sentence batched together
(some NMEA multiplexers/AIS receivers do this); that's handled
automatically.

Since this is a broadcast this board only listens to, whatever upstream
device is actually generating that feed (a GPS receiver already
configured for UDP broadcast, an AIS receiver in broadcast mode, a
multiplexer combining both, SignalK, etc.) needs to be broadcasting to
the subnet on port 10110 independently of this board — nothing here
causes that feed to exist. If you're setting up an upstream source for
the first time, see **Connecting OpenCPN** below, which also applies to
getting data flowing into this board in the first place, since both are
just listeners on the same broadcast.

If the port ever needs to change, `NAV_UDP_PORT` near the top of
`src/main.cpp` is the only place it's defined.

## Connecting OpenCPN

This board and OpenCPN are both just listeners on the same NMEA0183-over-
UDP broadcast now — there's no longer a "connect OpenCPN to this board"
step, since this board doesn't originate or relay anything. Point OpenCPN
at whatever's actually broadcasting the GPS/AIS feed on your network,
the same way this board listens to it:

1. **Options -> Connections -> Add Connection**
2. Type: **Network**
3. Protocol: **UDP**
4. Address: `0.0.0.0` (or leave blank) — this listens for the broadcast
   rather than connecting to one specific device
5. Port: `10110`
6. Apply, then check **Options -> Connections** shows data coming in
   (or watch the data monitor: **Options -> Connections -> Show NMEA
   Debug Window** or similar, depending on your OpenCPN version)

Any other program on the same Wi-Fi network that listens for NMEA0183
UDP broadcasts on port 10110 (nav apps on a phone/tablet, SignalK, etc.)
picks up the same feed simultaneously, this board included — that's the
advantage of a shared broadcast over each program needing its own direct
connection to the source.

## Reliability

This board runs unattended, so it's set up to recover from the most
common failure modes on its own rather than needing someone to notice
something's wrong and power-cycle it:

- **Task watchdog**: if a single pass through the main loop ever takes
  longer than 20 seconds — a genuinely stuck network call, a library
  deadlock, anything — the watchdog reboots the board automatically.
  Normal operation never comes close to 20s (nothing in the main loop
  blocks for any real length of time: BLE scanning and GPS/heater/icebox
  UDP reads are all non-blocking, and the DS18B20 temperature reads are
  spread across multiple loop() passes instead of blocking for their
  ~750ms conversion time), so this only fires on a real hang.
- **Wi-Fi connect timeout**: at boot, if Wi-Fi hasn't connected within
  60 seconds (e.g. the router isn't up yet after a full power-down), the
  board reboots and tries the whole boot sequence again, rather than
  sitting there forever with nothing running.
- **Classic Bluetooth memory released**: this project only ever does BLE
  scanning, never Classic Bluetooth, but the ESP32's Bluedroid stack
  reserves RAM for Classic BT by default regardless. That memory gets
  released back to the heap at boot, which matters a lot on a board this
  tight on RAM.
- **Internal logging disabled, at two separate layers**: the ESP32
  Arduino core logs certain internal events (like a dropped web server
  connection) straight to the same UART used for the Serial Monitor. If
  nothing's actively reading that UART (no Serial Monitor attached), and
  enough of these build up, the write can *block* until there's room -
  and since that can happen from inside the web server's own connection
  handling, a blocked log write stalls the whole dashboard along with
  it. This turned out to have two independent sources, not one:
  - `CORE_DEBUG_LEVEL=0` in `platformio.ini` covers the *Arduino-layer*
    logging (`log_e()`/`log_w()` calls, e.g. in `NetworkClient.cpp`).
  - `esp_log_level_set("*", ESP_LOG_NONE)`, called first thing in
    `setup()`, covers the *separate* ESP-IDF component logging
    underneath it - the WiFi driver, the Bluedroid/BLE stack, lwIP -
    none of which `CORE_DEBUG_LEVEL` touches at all.

  The first of these alone looked like it had fixed a "sluggish unless I
  have the Serial Monitor open" pattern (that had looked for a while
  like it might be a memory leak) - but a full lockup requiring a manual
  power cycle after several hours unmonitored turned out to still be
  happening, and only cleared the moment a Serial Monitor was reattached
  (with no page reload or power cycle), pointing at the same class of
  problem from the *other* logging source. Both are now off, which
  should mean nothing can queue up and block a UART write during normal
  operation regardless of which subsystem it would have come from - but
  given the first fix alone looked sufficient too until it wasn't, this
  is worth treating as the strongest lead so far rather than a
  guaranteed resolution until it's held up over a long unmonitored run.
- **Every web response asks the browser not to keep the connection
  alive** (`Connection: close`): the classic ESP32 `WebServer` library
  is documented to become unstable when a single HTTP connection gets
  reused for a very large number of requests over a long stretch -
  potentially relevant given the dashboard re-fetches `/data` every 2
  seconds, which could mean thousands of requests on one reused
  connection if a browser tab is left open for many hours. An earlier
  version of this fix also forced the ESP32 side of the connection
  closed after every response (`server.client().stop()`), but that call
  can itself block waiting on a TCP close handshake, and since the
  lockup this was meant to prevent was still happening with it in place,
  it was removed again rather than keep unproven blocking risk in a hot
  path - the logging fixes above are the more likely actual cause.

**Reading the heap footer**: the dashboard footer shows uptime, free
heap at boot, free heap right now, and the largest allocatable block.
Some drop from "boot" to "now" over the first while is normal (things
settle after the network stack, web server, etc. all come up) — what to
actually watch for:

- **Free heap slowly, steadily dropping over many hours with no sign of
  leveling off** — points to a genuine memory leak somewhere. Worth
  reporting the boot/now numbers a few hours apart so the rate can be
  worked out.
- **Free heap much bigger than the largest block** (e.g. 60kB free but
  only 8kB in the largest block) — heap fragmentation, not a leak. The
  board is more likely to act sluggish or fail to open new connections
  in this state even with plenty of "free" heap on paper.
- **Free heap and largest block low but close together and roughly
  flat** — just a tight-but-stable operating baseline, not a problem.

**If the board reboots on its own** (watchdog or otherwise), the Serial
log has the answer, but only if you're capturing it at the time — a
reboot clears whatever was in the log before. If you suspect a hang is
happening intermittently, leave a laptop connected via USB and run:

```bash
pio device monitor --filter log2file --filter time
```

`log2file` writes everything to a timestamped file under
`.pio/build/esp32dev/` (still shows live in the terminal too);  `time`
timestamps each line. A watchdog reboot logs a block starting with
something like `E (12345) task_wdt: Task watchdog got triggered`
followed by a `Backtrace:` line — if you catch one, that backtrace can
be decoded (with `addr2line` against the build's `.elf` file) to point
at the exact function that was stuck, which is much more useful for
tracking down a root cause than the reboot alone.

## Troubleshooting

- **No data for a device**: double check the MAC address and encryption
  key are current — Victron can rotate the key if you factory reset the
  device or re-pair. Get a fresh one from VictronConnect if needed.
- **Nothing shows up at all**: uncomment `victron.setDebug(true);` in
  `setup()`, re-flash, and watch the Serial Monitor for scan/decrypt logs.
- **BLE range**: keep the ESP32 within roughly 10-30 meters of both
  Victron devices, fewer obstructions is better.
- **Can't reach the web page**: confirm the ESP32's static IP didn't
  conflict with another device, and that your computer/phone is on the
  same network/subnet.
- **Dashboard fully locks up after many hours, needs a power cycle**:
  see the logging bullet under **Reliability** above - if it's still
  happening after that fix, the fact that a plain reboot is needed
  (rather than it recovering on its own, which the task watchdog should
  otherwise cause within 20s of a stuck loop) is itself an important
  clue that whatever's wrong isn't a simple stuck `loop()`. The most
  useful next step is capturing what actually happens at the moment of
  lockup, since guessing from symptoms alone has a real limit: run
  `pio device monitor --filter log2file --filter time` and leave it
  attached for as long as it takes to reproduce. A Guru Meditation/panic
  dump (not gated by any of the logging settings above - it's printed
  directly by the crash handler) would point at the exact function that
  faulted; a `task_wdt` block would mean the watchdog *did* fire but
  something about recovery afterward is failing; silence in the log
  right up to the lockup would point at something outside this board's
  own main loop entirely, most likely the WiFi/lwIP stack's own internal
  task, which isn't covered by the watchdog.
- **Heater or icebox card stuck on "no data"**: since these now push
  their data here (see **How the heater and icebox boards connect**
  above) rather than being polled, check that the *other* board is
  powered up, connected to Wi-Fi, and actually sending - its own Serial
  Monitor should show a "Telemetry UDP -> ..." line at boot. Also worth
  checking client/AP isolation on your router if it's never worked at
  all, and that `DASHBOARD_IP` on that board still matches this board's
  actual static IP.
- **A temperature probe shows "no probe"**: double-check the pull-up
  resistor is actually there (a very common miss — 1-Wire won't work at
  all without it, even though it's "just" a resistor) and that the
  probe's DATA line is wired to GPIO4. Check the Serial Monitor at boot
  — it prints how many probes it found vs. the expected 3, plus each
  found probe's address.
- **GPS or AIS pill stays red**: confirm whatever's actually broadcasting
  the NMEA0183/AIS feed on your network is powered up and genuinely
  broadcasting (to the subnet, e.g. `192.168.8.255`) on port **10110**,
  not sending to one specific unicast target. Check that OpenCPN (or
  whatever else consumes the feed) is also seeing it — if OpenCPN's
  blind too, the problem is upstream of this board entirely, not
  something to chase here. Also check for a "client/AP isolation"
  setting on your router, same as the heater/icebox troubleshooting
  above — it blocks broadcast traffic too, not just unicast.
- **GPS pill is green but no time shows on the dashboard**: the pill only
  means *some* `$`-prefixed sentence arrived recently — the clock
  specifically needs a `$--RMC` sentence with a valid fix. Confirm
  whatever's generating the feed actually includes RMC (not just GGA or
  other sentence types) and that the GPS itself has an actual satellite
  fix.

## Project structure

```
victron-monitor/
├── platformio.ini      # Board + library config
├── .gitignore          # Keeps include/secrets.h out of version control
├── include/
│   ├── secrets.h.example  # Template - copy to secrets.h and fill in
│   └── secrets.h          # Your real Wi-Fi/Victron credentials (gitignored)
├── src/
│   └── main.cpp        # WiFi + BLE scanning + GPS/AIS UDP listening + heater/icebox telemetry + temp probes + web dashboard
└── README.md
```
