# W5500-EVB-Pico2 — known issues / reliability notes

This board port is **functional but NOT recommended for production** based on
our testing.  The RP2350 side works fine; the problems are all on the WIZnet
**W5500 SPI-Ethernet** side.  We ultimately abandoned this board for our use
case and moved to a Wi-Fi (CYW43439 / Pico 2 W) design.  These notes are here
so the next person does not have to rediscover the same failure modes.

Hardware under test: a single W5500-EVB-Pico2 unit.  Some of this may be
unit-specific; treat it as "things that went wrong for us," not proven design
defects.

## 1. Unreliable link bring-up (the big one)

The W5500 does **not** reliably establish the Ethernet link on power-up.  On a
given boot it is roughly a coin-flip whether `wlan`/`eth0` comes up.  When it
fails, the RJ45 link LEDs stay **dark** and the interface never gets a DHCP
lease.  Recovery required a **full power-off of ~60 s** (cap discharge) before
it would link again — a warm reboot was not enough.

## 2. The RESET button does NOT reset the W5500

The RP2350 RUN/RESET button resets the *MCU only*.  The W5500 is reset solely
by its `RST` GPIO, which firmware toggles during `ifup`/driver init.  So after
a firmware crash or a wedged W5500, pressing RESET does nothing for the
Ethernet chip — you must remove power.  (Consider strapping W5500 `RST` to the
board reset on any respin.)

## 3. Link instability under connection churn

Under bursts of TCP connect/close (many short-lived sockets) the interface
would wedge — pings stop and stay stopped.  Part of this was a real driver bug
(see #4); part appears to be W5500 level-INT / SPI fragility that is documented
across other RTOSes too (e.g. Zephyr, ESP-IDF), and is why some projects move
the W5500 onto a PIO-SPI transport with a tunable MISO sample point.

## 4. TX-timeout used to fence the chip forever (FIXED upstream)

We found `drivers/net/w5500.c::w5500_txtimeout_work()` would put the chip into
permanent hardware reset on a TX timeout with no recovery, wedging the whole
stack until a power cycle.  A single stuck SEND (a missed `SEND_OK`, or a
desynced TX ring during a bursty TX storm) was enough to trigger it.

**This is already fixed on NuttX master** — `w5500_txtimeout_work()` now calls
`w5500_unfence()` and recovers.  If you are on an older tree, apply that fix.

## 5. SPI frequency sensitivity

The link was more stable at lower SPI clocks.  The defconfig ships
`CONFIG_RP23XX_W5500_SPI_FREQ` conservatively; pushing it higher made freezes
more likely on our unit.  If you see instability, lower it first.

## 6. RP2350-E9 input-latch erratum

The board init pulls the W5500 `INT` line up (`rp23xx_common_initialize.c`),
which is both required for the open-drain active-low INTn and the recommended
mitigation for RP2350-E9 (input pads can latch at ~2 V when floating / pulled
down).  Do not remove that pull-up.

## Bottom line

If you need reliable wired Ethernet on RP-class silicon, budget time for the
above, or consider a native-MAC MCU (e.g. STM32H7 + RMII PHY) or the Wi-Fi
route.  This port is provided as-is for those who want to work with the
W5500-EVB-Pico2 with eyes open.
