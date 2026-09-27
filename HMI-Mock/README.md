# HMI

The desktop Robot Motion Console is split into small modules so UI work does not leak into the real-time controller.

## Files

- `hmi_app.c/.h` — application layout, waypoint/profile fields, path selection and orchestration.
- `hmi_protocol.c/.h` — UDP RBT2 command packets and STA2 controller-status packets.
- `hmi_theme.c/.h` — shared fonts, colors, buttons, panels and numeric-field widgets.
- `waypoint_editor_3d.c/.h` — free-space 3D waypoint teaching, camera presets, XYZ gizmo and path preview.
- `hmi_types.h` — shared HMI-only data types.
- root `hmi.c` — intentionally tiny executable entry point.

## 3D waypoint teaching

The editor works in the robot base frame:

- robot `X` -> render `+X`
- robot `Y` -> render `-Z`
- robot `Z` -> render `+Y`

This keeps robot Z visually vertical while raylib continues using its normal Y-up rendering convention.

### Interaction

- Select waypoint `A`, `B` or `C`.
- Click the viewport to reposition the selected point in the current view plane.
- Drag the red X, green Y or blue Z gizmo to move along exactly one robot axis.
- Use `Top`, `Front` and `Side` views for unambiguous two-axis placement.
- Right-drag to orbit in Perspective view.
- Mouse wheel zooms the 3D camera.
- XYZ numeric fields provide exact coordinate entry.
- `APPLY XYZ` writes the taught positions back into the main HMI fields.

The main HMI still owns waypoint orientation (Yaw/Pitch/Roll). A future rotation gizmo can be added without changing the controller protocol.

## Geometry preview

The 3D editor reuses `ControlCore/Trajectory/circular_path.*` for circular arc and full-circle previews. The visualized circular path therefore uses the same three-point geometry as the live ControlCore planner rather than a separate decorative approximation.

## Controller boundary

The 3D editor only changes the Cartesian waypoint coordinates that the HMI sends. The validated live pipeline stays unchanged:

`HMI -> UDP RBT2 -> ControlCore -> streaming FIFO -> 1 ms CSP -> SOEM/EtherCAT`
