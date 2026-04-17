# Reuse Map (prizm_firmware)

- Source project audited: `D:\LigaProgram\LigaRobotMainBoard`.

## Reused as-is
- `src/rc_crsf.h/.cpp` -> `lib/legacy_liga_adapter/rc_crsf.*`
- `src/drive_mecanum.h/.cpp` -> `lib/legacy_liga_adapter/drive_mecanum.*`
- `src/motors_prizm_exp.h/.cpp` -> `lib/legacy_liga_adapter/motors_prizm_exp.*`
- `src/odometry.h/.cpp` -> `lib/legacy_liga_adapter/odometry.*`

## Adapted in new architecture
- Main control loop replaced by `SystemController` + startup FSM + run lock.
- Channel semantics moved into config/mode manager layer.

## Not reused
- Legacy monolithic `main.cpp` flow (replaced by modular architecture).
