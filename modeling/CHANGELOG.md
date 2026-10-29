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
- `Shape.Triangulate` gives a structured, immutable `ShapeMesh` for viewers and picking:
  - face meshes, with nodes shared within each face, surface normals, outward winding and the surface type;
  - edge polylines, taken from the face meshes, with the curve type;
  - vertices.
  Each list is in the order of `Faces()`, `Edges()` and `Vertices()`, and the whole mesh is copied out of
  the kernel in bulk. `MeshTolerances.For(shape)` gives tolerances in proportion to the shape.
  `TriangulateTree` gives the assembly tree, with labels, colours and locations, meshing each instanced
  geometry once.
- Native faults on Windows come back as typed `OcctException`s on any thread (`IsFault`). OCCT failures
  say what was being done (`OcctContext.Doing`, `OcctException.Operation`). Failed booleans name OCCT's
  alerts.
- New results tell the GC about their native memory (`GC.AddMemoryPressure`). Triangulation frees its
  temporary native mesh at once.
- Reading BRep data rejects truncated, garbled and overlong input instead of hanging or overrunning a
  buffer.
