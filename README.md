# BMCU Firmware – Calibration and Compatibility Notes

This BMCU firmware has been tested and verified with the latest Bambu Lab A1 firmware.

IMPORTANT:
The printer must be configured as AMS, not AMS Lite.
Using AMS Lite will cause incompatibility issues.


# ❗ IMPORTANT - FIRST START (V10.3+) ❗

At the first startup after flashing, **all channels must be empty**.

From **V10.3**, the firmware calibrates empty-channel detection during first boot.

If you flashed it with filament inserted:
- remove all filament
- hold any one buffer for about **5 seconds** to re-calibrate

# ❗ Warning for 2nd generation printers
A lot of people make a mistake because the drawings are misleading, and it is not always clear from the diagrams whether they show the plug or the socket. As a result, Signal A and Signal B often get connected the wrong way around.

If your BMCU is not detected by a 2nd generation printer, try swapping Signal A and Signal B - but make sure you know exactly what you are doing.


# HMS WARNING STATUS

This firmware version **triggers an HMS warning immediately after printer startup**.

Important clarification:
- This HMS warning **does NOT block BMCU operation**
- It does **NOT require restarting the printer**
- It does **NOT affect printing**
- The printer works normally despite the warning
- The issue is **purely visual / informational** (HMS icon only)

At the moment, the HMS warning is known and accepted behavior in this firmware version.

If the HMS warning in Bambu Studio annoys you:
I made a Bambu Studio build that bypasses this specific AMS compatibility warning, so you do not see it anymore.
Other HMS warnings will still be visible (if they happen), so HMS remains useful.

https://github.com/jarczakpawel/BambuStudio-BMCU

---

## Supported printers

Correct operation has been confirmed on both 1st generation and 2nd generation printers.

### 1st generation printers
Support is confirmed for 1st generation printers.

### 2nd generation printers
Correct operation has been confirmed on:
- Bambu Lab P2S
- Bambu Lab H2D

At this point, it looks like it should work on all printers from both generations.

---

## Download

Please download ready-to-use firmware from the **"Releases"** section (right side of the GitHub page).
All firmware variants are generated there together with **.txt guides** that explain which build you should choose.

Start by selecting the correct printer mode folder first (standard(A1) or high_force_load(P1S)), then choose AUTOLOAD / RGB / slots as usual.

## Flashing

To flash any version of the BMCU (USB or TTL) on:

- Windows
- Linux
- macOS
- Android

use **BMCU Flasher**:

https://github.com/jarczakpawel/BMCU-Flasher

Precompiled binaries are available in the **Releases** section.

The flashing process is very simple and **does not require wchisptool**.

You can flash firmware in two ways:

- **Online flashing** directly from the built-in wizard (recommended)  
  → the flasher downloads the correct firmware automatically, so you **do not need to download any .bin files manually**.

- **Local flashing** using a firmware file you downloaded yourself.

The flasher also supports **Android**, so you can even flash the BMCU directly from your **phone** 🙂

IMPORTANT:
- Do **NOT** flash the BMCU while it is connected to the printer.
- Do **NOT** connect or disconnect the BMCU while the printer is powered on (risk of damaging the BMCU and/or the printer mainboard).

---

## SOLO firmware

Example file:

solo_0.095f.bin

This firmware is intended for single BMCU (SOLO) operation.

- Recommended for single-BMCU setups
- Filament retraction length: 9.5 cm

---

## Filament retraction explanation

Filament retraction must be calculated from the end of the AMS splitter inside the printer
(the plastic AMS part where four PTFE tubes enter).

Example:

- Distance from BMCU to the end of the AMS splitter: approximately 9.0 cm
- SOLO firmware retracts the filament about 0.5 cm past the splitter
- Total retraction length: 9.5 cm

When calculating your own retraction length:

- Always measure from the end of the AMS splitter
- Add the required distance plus approximately 9 cm, depending on your setup

---

## AMS_A / AMS_B / AMS_C / AMS_D firmware

These firmware versions are intended for:

- Multi-BMCU setups
- Longer filament retraction distances

If you want to use SOLO mode with a longer retraction, use AMS_A instead of SOLO.

---

## Calibration (first start)

Correct calibration is mandatory.
Without proper calibration, BMCU will not work correctly.

The calibration process is shown in the following video:

https://www.youtube.com/watch?v=Hn_DNzSmhuc

Follow the calibration steps shown in the video carefully.

---

## Re-calibration

You can recalibrate the BMCU at any time.

Steps:

1. Remove all filaments from all channels
2. Hold any one buffer in position for approximately 5 seconds

---

## Safety and usage notes

- Do not flash BMCU while it is connected to the printer
- Do not disconnect BMCU while the printer is powered on
- Do not update printer firmware while BMCU is connected
- Connect/disconnect the BMCU ONLY when the printer is completely powered off (unplugged). Doing this while powered can damage the BMCU and/or the printer mainboard.

These recommendations are based on community reports.
Not all failure scenarios have been tested.

Changing the printer mode from AMS Lite to AMS while BMCU was connected did not cause issues in testing, but this is not recommended.

---

## Disclaimer

You are using this firmware and performing any modifications at your own risk.
Make sure you understand what you are doing.
I am not responsible for any damage, failed prints, hardware issues, or data loss.

---

## Before opening a bug report

Please verify the basics first:

- Make sure you flashed the correct firmware variant and followed the flashing tutorial correctly.
- Make sure you really have **BMCU 370C with Hall sensors**.
    - The only reliable verification is to open the module and inspect the PCB.
    - Some sellers mix modules and try to get rid of older **370x** boards - sometimes 1-2 modules in the set can be 370x.
- If you have printer-side issues:
    - confirm you are on the latest printer firmware
    - do a factory reset (this often fixes weird AMS-related behavior)
- If filament detection behaves strangely:
    - boot the printer once without BMCU connected
    - then connect BMCU and test again
- Do a few real tests before creating a thread.
  Printers can have unrelated issues (rare, but happens) - some users cannot even update printer firmware automatically and must do it via SD card.

## Bug reports

If you encounter a real bug, you may report it.
This firmware has undergone solid testing, and no issues are expected.

---

# Changelog

## V10.5

### User-visible changes
  - Added unique UID (Serial Number) generation for each BMCU device to ensure distinct identification in Home Assistant and other multi-device setups.
  - 为每个 BMCU 设备生成唯一的 注册UID（序列号），确保在 Home Assistant 等多设备环境中具有独立的身份标识。

## V10.4

### User-visible changes
- Added support for **Bambu Lab P2S**.
  - Verified to work correctly in real tests.
  - The **H2 series** will most likely also work as well, because **1st generation AMS support** is confirmed there.

### Fixes
- **"filament in use"** flag is now cleared correctly when filament runs out during printing.
- Added support for **retraction when the buffer is pulled up manually**, even when there is **no filament inside**.

## V10.3

### User-visible changes
- Added new firmware mode: **soft_load(A1)**.
    - Intended mainly for **A1 / A1 Mini** users.
    - Uses lower filament loading force than **standard(A1)**.
    - Useful for some BMCU units with weaker lever springs, where stronger loading can cause clicking / grinding during filament load.
- Improved empty-channel detection calibration.
    - The firmware now calibrates and stores the "no filament" detection point separately for each channel.
    - This improves reliability on hardware variants where idle detection voltage differs between channels/modules.
- Improved calibration behavior:
    - calibration now also detects and saves **Hall polarity per channel**
    - magnet polarity is automatically detected during calibration and stored, so it no longer matters which way the magnet is inserted in the buffer

### Stability and behavior improvements
- Fixed PWM timer preload configuration on all motor channels.
    - PWM updates are now buffered correctly before timer update events.
- Improved AS5600 update timing.
    - Sensor polling is now rate-limited to about **1 ms**
    - more stable speed calculation
    - lower unnecessary CPU load
- Improved internal timing paths by reusing shared tick snapshots in the main motion loop.
    - less timing jitter
    - more consistent runtime behavior
- Improved high-load / jam timing logic during on_use.
    - high PWM accumulation now uses **microsecond precision** instead of millisecond buckets
- Improved motion loop time-step handling.
    - uses wrap-safe tick delta
    - clamps oversized time steps
    - avoids running motor control with invalid zero-step timing

### Notes
- `soft_load(A1)` is not meant as the default for everyone.
- If filament gets rejected because push force is too weak, switch back to `standard(A1)` and use a stronger lever spring.
- On some A1 / A1 Mini units, `soft_load(A1)` works very well and can be used permanently.

## V10.2

### User-visible changes
- Fixed a problem where **filament run-out could incorrectly trigger a jam condition**.  
  When filament ended, the motor could run continuously and eventually enter jam protection, which blocked the **automatic filament refill**.
- Reworked jam protection logic:
    - real filament jams are now detected separately from temporary motor stops
    - high motor load alone no longer falsely triggers a jam
- Improved flash persistence system (less unnecessary flash rewriting).
- Improved ADC/DMA processing:
    - faster value updates
    - lower CPU overhead
    - smoother runtime behavior
- Various timing and stability improvements.

### Technical changes
- **Filament metadata flash storage redesigned.**
    - append-only journal instead of rewriting a full flash page
    - each record: **40 bytes (10 words)**
    - **CRC32 validation**
    - **6 records per flash page**
    - page erase only when the page becomes full  
      This significantly reduces flash wear and makes writes power-loss safe.
- Loaded-channel persistence reworked into a lightweight **slot log** to reduce erase cycles.
- Added **skip-if-unchanged** logic to avoid unnecessary flash writes.
- Simplified and optimized **ADC DMA update/publish path**.
- CRC tables moved to **static compile-time tables** (no runtime generation).
- Cleanup of timing paths using **wrap-safe 32-bit timers**.
- Several other smaller fixes and internal optimizations.

## V10

### User-visible changes
- Improved spool jam handling: jam is detected immediately, the print is paused, the printer waits for you to fix the snag/tangle, then you can resume normally without ruining the print.

### Flash / persistence (wear + reliability)
- State (loaded channel) persistence reworked into an append-only slot log: 8 bytes per update, up to 192 updates before any page erase (~192x fewer erase cycles vs rewriting a whole 256B page per update).
- Filament metadata persistence reworked into a small CRC-protected log: 64B per update, 2 pages per filament (8 records) -> ~8x fewer erase cycles vs erasing a whole 256B page on every change.
- Per-filament saves: only the modified channel is written (reduces unnecessary flash writes).
- Power-loss safe commits: records are validated and partially-written data is ignored.

## V9

### User-visible changes
- Increased filament loading force for improved reliability during filament insertion.
- Improved filament loading behavior on some materials (e.g. **Sunlu PLA+** and similar filaments) where loading characteristics differ from standard PLA/PETG.
- Added protection against **spool jams**:
    - Lock mode activates if the buffer drops too low during printing.
    - Lock mode also activates if the motor runs at high speed continuously for ~8 seconds.
    - The lock is automatically released once the buffer returns to the neutral position.
    - Prevents prolonged motor overrun when filament movement is blocked.

## V8

### User-visible changes
- Supported print resume after a printer power reset / power loss (printing can be resumed properly).
- Improved behavior for **P1S** (loading problems due to long/bent PTFE path).
- Added AUTOLOAD support for **single-switch PCB** boards:
    - Triggered by pressing the buffer ("buffer tap").
    - Starts filament loading exactly like the external switch trigger.
- More stable loading process overall.
- Improved support for **low-torque BMCU** variants.

## V7

### User-visible changes
- **Remember loaded filaments (persistent state).**  
  You can load filament and safely power off the printer.  
  This allows you to disable the automatic unload-at-end behavior in G-code (if you often print with one filament),
  keeping filament loaded until you actually need to change it.  
  More info here: https://wiki.bambulab.com/en/ams/manual/ams-not-unloading-to-save-filament
- **100% solved filament loading problems.** The system is stable and consistent across hardware variants.
- **Filament RGB colors.** Modules/LEDs can display the configured filament color.

### AUTOLOAD (short)
**How AUTOLOAD works**
- **DM (two microswitches):**
    - Touch first switch → AUTOLOAD starts (you may need a light manual push until gears grab).
    - BMCU feeds filament until the second switch (behind extruder) confirms **fully inserted**.
    - Then it feeds 120 mm to make it print-ready.
    - **Anti-snag protection:** buffer position is monitored; if the filament catches on housing / PTFE edge, it retracts to safe position and retries (3 retries).
- **Single-switch boards:**
    - Stage 1 is manual (no second switch to confirm fully-in).
    - Once filament is fully in the extruder, Stage 2 behaves the same (incl. anti-snag protection).

### Technical changes
- **ADC_DMA upgraded (ADC1 + ADC2 in parallel):**
    - Regular simultaneous mode: ADC1+ADC2 scan channels in parallel to reduce noise and increase throughput.
    - Lower noise enabled smaller filtering and faster stable readout.
    - Full filtered update time: **~5 ms instead of ~28 ms**.
- **AS5600 reading correctness improved** (robustness and stability of reads).
- **Timer/tick safety (wrap safety):** all time comparisons reviewed to be correct under wrap-around.
- Final stabilization and cleanup: overall behavior is faster and more deterministic than previous releases.
- There were more fixes in V7 as well; easiest is to check the commit history.

**Final note:** all known issues were ultimately resolved. BMCU is fully stable and significantly faster vs older firmware.
At this moment I do not expect any further fixes.

---

## V6

## Framework
- Dropped Arduino Core (PlatformIO: framework = arduino) - the whole firmware was rewritten to pure CH32 (WCH SDK / noneos).
- Direct use of hardware timers, DMA and interrupts - no Arduino delays, no random timing, deterministic real-time behavior.
- Faster and correct flash operations (WCH Fast API) - stable writes, faster, without corrupting neighboring data.

## ADC_DMA
- Separated DMA writes from CPU reads - previously reads happened while DMA was overwriting the buffer.
- Filter is computed in the background (DMA half/full), not during readout - previously `get_value()` blocked CPU and broke timing.
- Constant CPU load - previously larger filter window slowed the system down.
- DMA error handling

## BUS (BambuBus + AHUB)
- Fixed RX/TX buffer race (reading and overwriting the same buffer at the same time).
- Snapshot-based parsing instead of working on a live buffer
- Deterministic frame handling timing - constant CPU cost, independent from packet length.
- Robustness against transmission errors - a bad packet does not break the whole system state.

## Flash / NVM
- Flash written page-by-page (256B) instead of erasing/programming the whole sector (4KB)
- Write only when data actually changed
- Hardware CRC for flash + verification on read
- AMS data split into separate records - changing one filament does not rewrite the whole structure.

## Soft-I2C / AS5600
- Rewritten from Arduino, removed timing bugs and Arduino "magic".
- Correct ACK/NACK, START/STOP, recovery handling
- Hard isolation of channels with errors

## Motion / mechanics
- Smoother motor control
- Added calibration buffers - filament stays in a neutral position, without unnecessary tension.
- Better state transitions - no jerks and no sudden braking.

## Misc
- CRC8 / CRC16 rewritten to simple C + lookup tables - faster, deterministic, no objects and no runtime init.
- Partially de-spaghettified includes
- and many more - firmware prepared for further development


## Final note

This firmware started as a personal CH32 learning project.
During development it grew far beyond the original scope because working on BMCU turned out to be genuinely enjoyable.

Many solutions are intentionally overengineered.
Everything was implemented primarily for personal use and experimentation.

The firmware has been used extensively during development,
and no practical issues were observed in real-world usage.
