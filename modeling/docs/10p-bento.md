# Unit 10P1: what Bento sees change

Unit 10P1 took Schematiq.Collections out of both Kerf layers (question 50: "standard .NET read-only types in both Kerf
layers, no Schematiq.Collections") and made the kernel's engine internal. Results are unchanged: the same calls give the
same shapes.

## The dependency

- `Kerf.Modeling` and `Kerf.Kernel.Foundation` no longer reference Schematiq.Collections. Bento's extension has nothing
  to share with the host here, and its load context needn't special-case that assembly.
- Bento converts to Schematiq.Collections only where it needs a modified collection (unit 10N).

## `ShapeList<T>`

No public signature changed: `Count`, the indexers, `Add`, `AddRange`, `Upcast`, `OfType`, `Intersect`, `Except`, `Expand`,
the sub-shape methods and the `Single…` methods are as they were, and it is still an `IReadOnlyList<T>`. It now holds an
`ImmutableArray<T>`, which changes what some of them cost:

| Member | Before (Schematiq's tree) | After |
|---|---|---|
| `list[i]`, enumeration | Tree walk | Array read |
| `Add`, `AddRange` | Cheap | Copies the list: build a sequence first and pass it to `new ShapeList<T>(…)` |
| `list[range]` | Shared the tree | Copies the slice |
| `Upcast<TBase>()` to a base class | A lazy view | Shares the array |
| `Upcast<TBase>()` to anything else | Failed when an element was read | Fails at the call |

The layer's own `GroupBy` used `Add` in a loop; it now builds each group once.

## The kernel's types

Bento should see none of these, since it is on the public API, but anything that reached past it now gets a compile error:

| Before | After |
|---|---|
| `Occt.PackedList<T>`, `ISchematiqList<T>` from the engine's methods | Removed; `IReadOnlyList<T>`, over an `ImmutableArray<T>` |
| `NCollection_*`, `TCollection_HAsciiString`, `TopoDS_HShape` and the rest of `Occt` | Internal |
| The rest of `Kerf.Engine.*`, including `MeshBuffers`, `IncrementalMesh` and `BrepTools` | Internal: use `Kerf.Meshing`, `Kerf.Exchange` and the other public namespaces |
| `Transient`'s `DynamicType`, `DynamicTypeName`, `IsKind`, `IsInstance`, `This`, `IsFrozen`, `CheckMutable` on every curve and surface | Internal. `Transient` itself stays public as the base of `Geometry3D` and `Geometry2D`, with no members |
