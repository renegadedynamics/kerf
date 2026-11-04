# Unit 10P2: what Bento sees change

Unit 10P2 finished the public failure surface (kernel decisions for unit 10P2). Results are unchanged: the same calls
give the same shapes and the same bytes.

## The exporters return results

Each export returns `Result` where it returned `bool`. Parameters past the stream or path are an options record.

| Before | After |
|---|---|
| `ExportBrep(shape, stream or path)`: `bool` | `Result` |
| `ExportStep(shape, stream or path, unit, writePcurves, precisionMode, timestamp, token)`: `bool` | `ExportStep(shape, stream or path, StepExportOptions? options, token)`: `Result` |
| `ExportStl(shape, stream or path, tolerance, angularTolerance, asciiFormat, token)`: `bool` | `ExportStl(shape, stream or path, StlExportOptions? options, token)`: `Result` (`Tolerance`, `AngularTolerance`, `Ascii`) |
| `ExportGltf(shape, stream or path, unit, binary, linearDeflection, angularDeflection, token)`: `bool` | `ExportGltf(shape, stream or path, GltfExportOptions? options, token)`: `Result` |
| `Export3mf(shape, stream or path, unit, linearDeflection, angularDeflection, token)`: `bool` | `Export3mf(shape, stream or path, ThreeMfExportOptions? options, token)`: `Result` |

| Failure | Before | After |
|---|---|---|
| An empty shape | `InvalidOperationException` | failed, `exchange.nothing-done`, the same message |
| STEP can't be written | `InvalidOperationException` "Failed to write STEP file" | failed, the writer's diagnostics |
| STL, glTF or 3MF can't be written | `false` | failed, the writer's diagnostics |
| A mesh can't be made | `InvalidOperationException` | failed, the mesher's diagnostics |
| A file or stream can't be written | `IOException`, `UnauthorizedAccessException` | failed, `exchange.write-failed` |
| A folder that doesn't exist | `DirectoryNotFoundException` | the same, as a missing file is for the importers |
| Null, a unit that isn't a length | `ArgumentNullException`, `ArgumentException` | the same |
| Cancellation | `OperationCanceledException` | the same |

Bento shows a failed export as it shows a failed import: `#VALUE!` with `Error.Message`.

## The kernel's values refuse with .NET's exceptions

A public value, geometry or shape of the kernel used to throw an internal exception on misuse. Now:

| Misuse | Before | After |
|---|---|---|
| `new Direction3D(0, 0, 0)`, a negative radius, a bad spline | internal `ConstructionErrorException`, `DomainErrorException` | `ArgumentException` |
| An index out of range | internal `OutOfRangeException` | `ArgumentOutOfRangeException` |
| A call the object's state refuses: `Vector3D.Normalized()` of a zero vector, `Period` of a curve that isn't periodic, an undefined derivative, a default shape | internal exceptions | `InvalidOperationException` |
| A trimmed curve or surface with a NaN parameter | the process died of a stack overflow | `ArgumentException` |

`Failures` in the modelling layer reads a refusal as it read the failure it replaces, so messages and codes in cells
are unchanged (`engine.construction-error`, in context). Code that catches an exception directly from a kernel value
sees the new message, with no OCCT names: "The geometry can't be constructed: input vector has zero norm."
`engine.locked-shape` and `engine.frozen-shape` are gone: they could only report a fault in the kernel, and report as
`engine.immutable-object` and `engine.domain-error`.
