# BMCU Firmware – Calibration and Compatibility Notes

<p align="center">If BMCU firmware or my other open-source projects are useful to you and you would like to support further development:
</p>
<p align="center">
  <a href="https://ko-fi.com/jarczakpawel">
    <img src="https://img.shields.io/badge/Support_on-Ko--fi-FF5E5B?style=for-the-badge&logo=kofi&logoColor=white" alt="Support on Ko-fi">
  </a>
  &nbsp;
  <a href="https://revolut.me/paweqxdkx">
    <img src="https://img.shields.io/badge/Support_via-Revolut-191C1F?style=for-the-badge" alt="Support via Revolut">
  </a>
</p>

> [!WARNING]
> Bambu Lab is limiting local BMCU interoperability through printer firmware updates.
>
> More information about the certification/authorization restrictions and how printer updates changed BMCU compatibility:
> [BMCU vs firmware locks](./bmcu-vs-firmware-locks.md)

## Printer firmware compatibility

> [!TIP]
> If you do not want the AMS/HMS compatibility error to block a project in the slicer, use **[OrcaStudio](https://github.com/jarczakpawel/OrcaStudio)**.
> On affected A1 / A1 mini firmware, OrcaStudio ignores the device compatibility error, keeps the project usable and retries sending the interrupted print.

| Printer | Printer firmware | BMCU compatibility                                                                                                                                                                                                                                                                                           |
|---|---|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| **A1 / A1 mini** | `01.05.00.00 – 01.07.02.00` | ✅ **Works normally.** A visible HMS warning may appear, but it does not affect printer operation in any way. The printer must be configured as **AMS**, not AMS Lite.                                                                                                                                                                                                 |
|| `>= 01.08.00.00` | ✅ **Works.** The printer sends a device compatibility error that blocks the project in the slicer. Use **[OrcaStudio](https://github.com/jarczakpawel/OrcaStudio)** — it ignores this error, does not block the project and retries sending the interrupted print, which resolves the compatibility problem. The printer must be configured as **AMS**, not AMS Lite. |
| **P1P / P1S** | `<= 01.08.01.00` | ✅ **Works normally.** This is the last firmware before the new authorization mechanism.                                                                                                                                                                                                                      |
|| `>= 01.08.02.00` | ⚠️ **The first print works and can run for any length of time.** On the next print, BMCU is rejected because it does not pass device certification/authorization. Before another print, reconnect/reset BMCU or restart the printer.                                                                         |
| **X1 / X1C** | `<= 01.08.02.00` | ✅ **Works normally.** This is the last firmware before the new authorization mechanism.                                                                                                                                                                                                                      |
|| `>= 01.08.03.00` | ⚠️ **The first print works and can run for any length of time.** On the next print, BMCU is rejected because it does not pass device certification/authorization. Before another print, reconnect/reset BMCU or restart the printer.                                                                         |
| **P2S** | `<= 01.01.03.00` | ⚠️ **The first print works and can run for any length of time.** It is best to start the print shortly after the printer starts or BMCU is reconnected. Before another print, reconnect/reset BMCU or restart the printer.                                                                                   |
|| `>= 01.02.00.00` | ❌ **Printing with BMCU does not work.** BMCU is detected and normal manual filament load/unload works, but when starting a print the printer stops at **Preparing AMS** because the required certification/authorization is not satisfied.                                                                   |
| **H2D** | `<= 01.02.00.00` | ⚠️ **The first print works and can run for any length of time.** It is best to start the print shortly after the printer starts or BMCU is reconnected. Before another print, reconnect/reset BMCU or restart the printer.                                                                                   |
|| `>= 01.03.00.00` | ❌ **Printing with BMCU does not work.** BMCU is detected and normal manual filament load/unload works, but when starting a print the printer stops at **Preparing AMS** because the required certification/authorization is not satisfied.                                                                   |
| **X2D** | `01.00.01.00` | ⚠️ **The first print works and can run for any length of time.** Before another print, a restart/reconnect is required. **If you want to use BMCU, do not update the printer — there is currently no normal way to downgrade back to this firmware.**                                                        |
|  | `>= 01.01.00.00` | ❌ **Printing with BMCU does not work.** BMCU can be detected and normal manual filament load/unload works, but the print stops at **Preparing AMS**.                                                                                                                                                         |
| **H2S** | Newer firmware | ❌ **Preparing AMS.** BMCU is detected, but printing does not start. The exact last compatible printer firmware has not yet been confirmed.                                                                                                                                                                   |
| **A2L** | All versions | ❌ **Does not work.**                                                                                                                                                                                                                                                                                         |

> [!NOTE]
> **P2S / H2D:** if you do not want to power-cycle the whole printer between prints, you can make a simple switch that disconnects **SigA and SigB at the same time**. Switching both signal lines off and back on allows BMCU to reconnect to the bus without unplugging the printer from power.

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

## Filament retraction explanation

Filament retraction must be calculated from the end of the AMS splitter inside the printer
(the plastic AMS part where four PTFE tubes enter).

Example:

- Distance from BMCU to the end of the AMS splitter: approximately 9.0 cm
- A 9.5 cm retraction variant is available for AMS_AUTO and AMS_A / AMS_B / AMS_C / AMS_D
- Choose the retraction length that matches your setup

When calculating your own retraction length:

- Always measure from the end of the AMS splitter
- Add the required distance plus approximately 9 cm, depending on your setup

---

## AMS_AUTO / AMS_A / AMS_B / AMS_C / AMS_D firmware

### AMS_AUTO - recommended

**AMS_AUTO is the recommended option for new installations.**

It works like the original Bambu AMS addressing system - connected BMCU units are **automatically enumerated and assigned their AMS number**.

In most cases, you should use **AMS_AUTO**.

### AMS_A / AMS_B / AMS_C / AMS_D - legacy

These are the old manually assigned firmware variants.

They are now considered **legacy** and are no longer the recommended configuration.

Use them only if **AMS_AUTO does not work correctly with your setup**.

---

## Calibration / Re-calibration

Correct calibration is important because after calibration BMCU knows the **exact movement limits of the buffer on every channel**.

Before starting calibration, remove filament from all channels.

To enter calibration mode:

1. Hold any one buffer for approximately **5 seconds**.
2. At first, **do not move anything**. BMCU records the current neutral / center position of every buffer.
3. When the channel LED starts **blinking blue**, push that channel's buffer **all the way in**, hold it there for approximately **1 second**, then release it.
4. When the channel LED starts **blinking red**, pull the buffer **all the way out**, hold it there for approximately **1 second**, then release it.
5. Repeat the same procedure for every channel.

After calibration, BMCU knows the exact **minimum, center and maximum buffer positions** for each individual channel.

You can repeat this calibration procedure at any time if necessary.

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
