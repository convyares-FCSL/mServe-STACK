# HyFleet Compression Module — Live Repo Todo

Work to be done after migration of `hyfleet_booster`, `hyfleet_compressor`, and
`hyfleet_sim` to the live project. Items are ordered roughly by dependency and
priority, not session-by-session.

See `architecture.md` for design decisions. All closed decisions still hold.

---

## Migration (do first)

- [ ] Fork live repo; create `feature/compression-module` branch
- [ ] Copy `hyfleet_booster`, `hyfleet_compressor`, `hyfleet_sim` into the live workspace
- [ ] Build clean: `colcon build --packages-select hyfleet_booster hyfleet_compressor hyfleet_sim`
- [ ] Absorb `CompressorTelemetry.msg`, `BoosterCmd.srv`, `CompressorCmd.srv`,
      `ControlBooster.action`, `ControlCompressor.action`, `SetMode.srv` into the live
      interfaces package; update all `#include` and `find_package` references
- [ ] Confirm PLC warn/alarm/lockout flags are present in `CompressorTelemetry.msg`
      (see `architecture.md` §Telemetry); add if missing
- [ ] Identify live equivalents of `mserve_utils` helpers (`get_or_declare_param`,
      range descriptors); update includes; build clean
- [ ] Rewrite `mserve_launch` as `hyfleet_launch`; `use_sim` arg swaps in `hyfleet_sim`
      instead of ADS bridge
- [ ] Per-instance YAML files for `low_booster` and `high_booster` (confirmed from
      ADS wiring docs — do not guess indices)
- [ ] No "using default" warnings at startup on any param

---

## Full integration tests — post-migration gate

Run against `hyfleet_sim` in the live workspace. All must pass before PR is raised.

- [ ] `test_low`, `test_high`, `test_parallel_both`, `test_setpoint_update`, `test_stop`,
      `test_sync` — all 6 pass against `hyfleet_sim`
- [ ] Clean SIGINT: `Ctrl-C` on launch, no nodes hard-killed mid-tree

---

## Booster rounding-out (Stage 4)

### Safety and monitoring

- [ ] `oil_healthy` subscription — `RosTopicSubNode` in each booster; fail-safe on stale:
      no fresh message within N s → treat as unhealthy, refuse/stop goal.
      Add a trivial fake `oil_healthy` publisher to `hyfleet_sim` for standalone tests.
- [ ] Compression ratio monitoring — `outlet / inlet`; per-instance params `ratio_limit`
      (1:30 low, 1:60 high); halt-restart on exceed; escalation guard: repeated exceedances
      in sliding window → abort/fault (circuit-breaker)
- [ ] Discharge temperature trend monitoring — rise rate; primary thermal health signal;
      protective over-temp trip is PLC, ROS monitors the trend
- [ ] Stall / no-progress detection — absence-of-rise in smoothed step-per-stroke over
      window → FAILURE; pairs with inlet-starvation gate (inlet < ~0.2 bar → stop);
      replaces the Stage 4 TODO hook in `OutletAtPressure::onRunning()`

### Diagnostics

- [ ] `DiagnosticsPublisher` on both nodes — status, last fault, mode, active tree
- [ ] Groot2 visualisation wired to both trees (live tick visualisation)

### Lifecycle manager

- [ ] Convert `LifecycleManager` to a lifecycle node (Nav2 pattern)
- [ ] Shutdown tree: deactivate → cleanup → shutdown in dependency order
- [ ] Verify clean SIGINT: no nodes hard-killed before shutdown tree completes

---

## Coordinator rounding-out (Stage 4)

### Mode service (done — verify in live repo)

`~/set_mode` service is implemented. Verify it works end-to-end after migration:
- [ ] `ros2 service call /hyfleet_compression/set_mode mserve_interfaces/srv/SetMode "{mode: 2}"`
      → logs "Compressor mode set to ECO"; subsequent fill uses `eco_cpm`

### Force stop robustness

- [ ] Recovery Fallback: graceful stop → force stop path in coordinator XML trees.
      Pattern: `Fallback(Sequence(STOP) → ForceFailure(Sequence(FORCE_STOP)))` — STOP
      attempt with timeout; fall through to FORCE_STOP if graceful fails.

### Re-goaling (deferred — complex)

- [ ] New LOW goal while LOW already RUNNING: relay updated `target_pressure` to in-flight
      booster (smooth update, no halt). Currently aborts and restarts. Depends on extended
      booster preemption semantics (booster must accept updated target on a live goal).

### Mode as stored state (done — may need review)

`mode` was removed from `ControlCompressor.action` goal and replaced with `~/set_mode`
service. If the live orchestrator previously relied on per-goal mode, update the caller.

---

## Simulator (Stage 5 remaining)

These are deferred from the mServe workspace — not needed for core functionality.

- [ ] VFD ramp: first-order lag `τ ≈ 0.5 s` on VFD start/stop (currently instantaneous)
- [ ] Pressure decay when PCSV off: `τ ≈ 10 s` (currently pressure holds exactly)
- [ ] Compression sawtooth ripple (cosmetic, not needed for logic testing)
- [ ] PLC protective layer: over-pressure / over-temp → alarm/lockout + autonomous stop;
      surfaces warn/alarm/lockout flags in `CompressorTelemetry.mode`

---

## End-of-run reporting (Stage 4)

- [ ] Extend `ControlBooster.Result`: end pressure, kg compressed (real-gas Z-factor
      required at 300–900 bar H₂), energy (kWh), specific energy (kWh/kg), peak ratio,
      cycle count, duration
- [ ] Cumulative counters: hours run (loaded vs idle), PCSV cycles, start count;
      fault/event log (last fault + history); diagnostics output

---

## PR gate

- [ ] No regressions in existing live bringup (non-compression nodes unaffected)
- [ ] All 6 integration tests pass in the live workspace against `hyfleet_sim`
- [ ] PR raised; merge dependency noted: ADS bridge work must land first
