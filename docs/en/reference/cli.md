# Command-Line Reference

> Category: Reference. The [Chinese source](../../zh/reference/cli.md) is authoritative. Console entry points are defined in `pyproject.toml`.

## `piper-pick-demo`

```bash
piper-pick-demo --config PATH [--query TEXT | --voice ...] [--mock]
```

`--config` is required. Non-voice execution requires `--query`; the hardware configuration does not contain a default
task. `--mock` uses the offline model and Mock environment.

Common temporary overrides include `--model`, `--server-url`, `--api-key`, `--max-iter`, `--workspace`, `--debug`,
`--no-skill`, and `--no-visual-feedback`. Voice mode supports `--voice`, `--voice-text`, `--voice-audio-file`,
`--voice-once`, `--no-wake`, `--tts`, and `--asr-device`.

## `jiuwensymbiosis-replay`

```bash
jiuwensymbiosis-replay TRACE_JSON [--text]
```

The default mode generates a self-contained HTML replay and prints its path. `--text` prints a terminal timeline and
frame paths without generating the visual report.

## `jiuwensymbiosis-gui`

```bash
jiuwensymbiosis-gui
# Equivalent module entry point:
python -m jiuwensymbiosis.gui
```

Starts the local NiceGUI browser service on `127.0.0.1`. Startup preflight reports the `.[gui]` installation command
when optional dependencies are missing.

## Hand-eye calibration

Installing `.[calib]` provides three entry points:

```bash
jiuwensymbiosis-calibrate-hand-eye --collect-poses OUTPUT --config RUNTIME_YAML
jiuwensymbiosis-calibrate-hand-eye --auto WAYPOINT_ARCHIVE --config RUNTIME_YAML --confirm-estop
jiuwensymbiosis-calibrate-hand-eye --replay STATION_ARCHIVE [--config RUNTIME_YAML]
```

`jiuwensymbiosis-calibrate-hand-eye` is the mount-neutral entry point; the camera mount comes from the runtime config or the archive and cannot be overridden on the command line. `jiuwensymbiosis-calibrate-eye-to-hand` runs the same flow but requires `eye_to_hand`, and `jiuwensymbiosis-calibrate-eye-in-hand` requires `eye_in_hand` for archive modes.

Exit codes: `0` published (or dry-run passed), `1` execution error, `2` preflight contract failure, `3` only an unloadable REVIEW/candidate report was produced.

The GUI's 「工具 → 手眼标定」 wizard drives these same workflows and additionally generates a printable board PDF and shows live corner detection while teaching; start there for a first calibration.
