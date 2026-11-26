# CLAUDE.md

Guidance for Claude Code when working in the wm-tools (DSO3D12 flasher and viewer) repository.

## Read README.md first

**[README.md](README.md) is the single source of truth** for this repo: workspace layout and
package responsibilities, GUI modules, threads and serial-port ownership, the cursor model, iced
0.14 notes, device facts, the documentation map, build, test and lint commands, diagnostics, CI
and releases. Read the relevant sections before changing anything. This file adds only rules for
AI assistants.

## Cross-Tool Instruction Sync

This project keeps three parallel AI-assistant instruction files, one per tool:

- `CLAUDE.md` — Claude Code
- `AGENTS.md` — OpenAI Codex
- `.github/copilot-instructions.md` — GitHub Copilot

They are deliberately thin. **Shared project guidance belongs in `README.md`, not here** — add or
fix it there, and link to it from these files rather than copying it.

Their bodies are intentionally identical. **If any one of these files is updated by any tool or
contributor, the other two MUST be updated in the same change.** Only the title and the tool-name
framing may differ.

## Rules for AI assistants

### Before you finish

Every change must leave both of these passing; run them yourself and report the result:

```bash
cargo test --workspace                                        # unit tests
cargo clippy --workspace --all-targets -- -D warnings         # lint (must be warning-free)
```

- Do not silence lints with `#[allow(...)]`. If one is truly needed, scope it to the smallest
  item and say why in a comment.
- Run `rustfmt` on files you create. Do not reformat code you did not change.

### Tests

- Behaviour changes come with tests in the same crate. Update tests when the implementation
  changes; do not delete or weaken a test without an equivalent replacement.
- Parser changes need tests on concrete byte arrays or text, preferably taken from a real
  device, for every device variant the change touches.
- Tests must not need a serial port or an oscilloscope.

### Threads and the serial port

The serial listener and the firmware update run on background threads while the GUI keeps
working (README, "Threads and the serial port").

- **Never block the GUI thread.** No blocking serial I/O, `join` on a running thread, or sleeps
  in `update()`; do that work on the background thread and report through its channel.
- **One owner per serial port.** Stop the listener and wait for it before anything else opens
  the port, and never add a second reader for the same port.
- **Check cancel safety.** Every loop on a background thread must notice its stop flag within a
  read timeout; a stop, cancel, error or dropped handle must close the port and leave the GUI
  in a consistent state. Check async code the same way: an abandoned future must not leave
  half-done state behind.
- **Shared state is `Send + Sync`**, behind `Arc` with a `Mutex` or atomic. Never hold a lock
  across a blocking call or `.await`, and recover from poisoning with
  `.lock().unwrap_or_else(PoisonError::into_inner)`.

### Hardware safety

The flasher writes firmware to the device. A wrong byte, baud command or interrupted transfer
can leave the oscilloscope unbootable.

- Do not change XMODEM framing, the sync, baud-rate or reboot commands, erase handling or
  DTR/RTS reset handling without evidence: a verified capture, the reference SDK, or a live
  test. Say what the evidence is in the change.
- Record new packet or dump format facts in `docs/SCREENSHOT_PROTOCOL.md` in the same change.
- Do not assume one device variant or firmware: `dso3d12`, `dso2512g-v1` and `dso2512g-v2`
  differ, as do stock and ZeeTweak firmware.
- Placeholders must fail loudly. Never return fake captures or readings from production code.

### Testing on a real device

- Only when the user has the device connected and has agreed.
- Never flash or erase a device, or send reset or reboot commands, without the user's explicit
  consent for that run; use only firmware images the user names.
- Opening a port may reset the device through DTR/RTS. If the port is busy, ask the user: it may
  be held by their running GUI, and you must never stop their processes.

### GUI

- Keep measurement cursors and device cursors separate (README, "Cursor model").
- Add every new user-visible string to all five files in `gui/locales/`.
- Update `docs/GUI_DESIGN.md` and `docs/USER_MANUAL.md` in the same change as the GUI.

### Change discipline

- Prefer small, focused changes that keep existing APIs and behaviour.
- Keep CLI examples in the README in exact agreement with the `flash` and `capture` arguments.
- Do not change version numbers, the license or authorship without explicit instruction.
- Never commit secrets, local settings, firmware images or device captures.
- Commit messages use a short scoped prefix, as in the history: `GUI: ...`, `gui: serial: ...`.
  Commit or push only when the user asks.
- If a requirement is unclear, ask one concise question before making the change, and let the
  user type a free-form answer.
