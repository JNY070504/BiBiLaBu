---
name: "keil-cli-build"
description: "Compile and flash Keil MDK-ARM (STM32) projects from the command line via UV4.exe, then parse build_log.txt for errors/warnings and size. Invoke when the user asks to 编译/重新编译/烧录/下载 an STM32 Keil project (.uvprojx), or wants a build report."
---

# Keil CLI Build & Flash (STM32 / MDK-ARM)

Runs Keil MDK-ARM builds headlessly through `UV4.exe` and reports the result
without opening the IDE GUI.

## Toolchain

- **UV4.exe**: `F:\keil_5\UV4\UV4.exe`
- **Compiler**: ARMCC V5.06 update 5 (Keil ARM Compiler 5)
- Adjust the path if the user's Keil installation differs.

## Commands

Set `$PROJ` to the `.uvprojx` path and `$LOG` to a log file path.

| Action | Command |
| --- | --- |
| Incremental build | `& "F:\keil_5\UV4\UV4.exe" -b "$PROJ" -j0 -o "$LOG"` |
| Full rebuild | `& "F:\keil_5\UV4\UV4.exe" -r "$PROJ" -j0 -o "$LOG"` |
| Download / flash | `& "F:\keil_5\UV4\UV4.exe" -f "$PROJ" -j0 -o "$LOG"` |
| Build then flash | `& "F:\keil_5\UV4\UV4.exe" -b "$PROJ" -f -j0 -o "$LOG"` |

Flag notes:
- `-b` build (incremental), `-r` rebuild all, `-f` download to target
- `-j0` hides the GUI dialog / auto-close
- `-o <file>` writes the build log to that file
- `UV4.exe` returns immediately in some shells; the log keeps being written,
  so re-read `build_log.txt` if it looks truncated mid-compile

## Workflow

1. Locate the `.uvprojx` (usually `<project>\MDK-ARM\<name>.uvprojx`).
2. Run the build command with the Shell tool; use a generous timeout
   (a full rebuild of a small STM32 project takes a few seconds, a large
   one needs minutes).
3. Read `build_log.txt`.
4. Report the outcome using these three things:
   - Final line: `"<name>.axf" - N Error(s), M Warning(s).`
   - `Program Size: Code=... RO-data=... RW-data=... ZI-data=...`
   - The first actual error line if the build failed (`error:` / `#NNN:`)
   Do not dump the whole log — summarize.
5. If errors exist, fix the code and repeat from step 2.
6. Only flash (`-f`) when the user asks and a board is connected.

## Reading a failed build

- `#NNN: ... error` → compiler diagnostic; the number encodes the category
  (e.g. `#65` = expected a `;`).
- `L6218E: Undefined symbol X` → missing source file in the project's file
  groups, or missing `extern` declaration. Check the `.uvprojx` groups.
- `L6200E: Symbol X multiply defined` → same symbol defined twice; e.g. a
  peripheral init or IRQ handler written both by CubeMX-generated files and
  by a user driver.
- `Rebuild target` in the log means `-r` was used; `Build target` means `-b`.

## Rules

- Never open the Keil GUI to verify a build. Use this CLI flow instead.
- Prefer the dedicated Read tool over `cat`/`type` to inspect the log.
- A clean log must show `0 Error(s), 0 Warning(s)`; report anything else
  verbatim and fix it before declaring success.
