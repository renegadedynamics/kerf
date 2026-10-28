# Changelog

All notable changes to the OCCT.NET packages. Versions follow [SemVer](https://semver.org); before 1.0 a minor
version may break the API.

## 0.1.0 (unreleased)

Phase 0: groundwork, ported from OCCT V8_0_1 (b8f597c6).

- Licensed under LGPL 2.1 with the Open CASCADE exception, as OCCT.
- `OcctNet.Foundation`:
  - Standard: OCCT's exception hierarchy and its real and integer functions; Precision.
  - The gp package: points, vectors, directions, axes, coordinate systems, matrices, transformations,
    quaternions, conics and elementary surfaces, in 3D and 2D. Values are immutable to callers; each OCCT
    `Set` method has a `With` counterpart that returns a copy, and transformations have static factories
    (`gp_Trsf.Rotation(axis, angle)`).
  - `PackedList<T>`: an `IBentoList<T>` over one contiguous array, readable as a span.
- `OcctNet.Analyzers`: OCCTNET0001 reports a mutating call that would change a defensive copy.
- Tests: OCCT's gp GTests ported; oracle tests against native OCCT; Draw-lite, which parses all 17,237 cases
  of OCCT's Draw test corpus and lists the commands they need.
- `tools/port-status.cs` writes `docs/port-status.md`.
