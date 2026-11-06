# The clean-up: what Bento's branch has to change

The code clean-up after decision 1681 (kernel `docs/clean-up.md`) changed some public names. Results don't change. Bento's branch `kerf/0.2.0`
was searched for every name below; only these two lines touch one.

## What Bento has to change

- **`Extensions/Bento.Cad/IoFunctions.cs`, `ExportStl`:** `StlExportOptions` now says what the glTF and 3MF options say. Inside `new StlExportOptions { … }`:

  ```csharp
  LinearDeflection = options.Number("tolerance", 1e-3),
  AngularDeflection = options.Number("angularTolerance", 0.1),
  ```

  Bento's own option names (`tolerance`, `angularTolerance`) can stay, since they are what its users type.
- **`Extensions/Bento.Cad/ModelFunctions.cs`, line 376:** the help text says "(Kerf's SlotCurve, and SlotArc)". `SlotArc` is gone, so it should say "(Kerf's SlotCurve)".

## What it doesn't use, for reference

Bento uses none of these, so nothing else has to change: the rest are listed in the CHANGELOGs under `0.3.0`.

- `Kerf.Modeling`: `BoxHeight`, `RectangleHeight`, `ConeHeight`, `CylinderHeight`, `TrapezoidHeight` and `SlotHeight` (all now `Height`), `Polygon.Pts` (now `Points`), `DPrism` (now `DraftPrism`), `GeomAdaptor()` (now `AsCurve()` on edges and wires and `AsSurface()` on faces), `TopologyUtils` (now internal), `Triangle`'s parameters.
- `Kerf.Kernel`: the `With…` methods with OCCT's abbreviations, `EulerSequence`'s and `TransformForm`'s members, `Direction3D.D` and `Direction2D.D` (use `Direction3D.UnitX` and so on), and parameter names, which matter only to a call that names its arguments. Bento has no such call to a renamed parameter.
