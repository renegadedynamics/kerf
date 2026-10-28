# Changelog

All notable changes to the build123d.NET packages. Versions follow [SemVer](https://semver.org); before
1.0 a minor version may break the API.

## 0.1.0 (unreleased)

- Licensed under Apache-2.0.

The first packages, for Bento's 3D-models extension.

- Geometry: Vector, Axis, Plane, Location, Rotation, Matrix, Color, BoundBox.
- Topology: Shape, ShapeList and Vertex through Compound, with selectors and history.
- Objects and operations in algebra mode: the part, sketch and curve objects, location lists, and the
  generic, part and sketch operations (not yet `insert`, `add`, `make_brake_formed` or text).
- Native libraries for Windows x64 only.
- Operations no longer change their inputs or the geometry those inputs share, and can run concurrently
  on shared shapes:
  - meshing works on copies (`Shape.Mesh` is replaced by `Meshed`, which returns a meshed copy);
  - booleans run non-destructive;
  - ShapeFix, loft and tapered extrusion work on copies.
- Warnings are collected by `Warnings.Capture()` scopes, which replace the static `Warnings.Raised` event.
