# Unit 10M3: what Bento sees change

Unit 10M3 removed `KerfException` (the owner, 4 November 2026: "Let's get rid of that `KerfException`"; kernel
decisions for unit 10M3, after decision 1585). A failure a caller should expect is now a failed `Result<T>`, read rather
than caught. Results are unchanged: the same calls give the same shapes, and the messages Bento shows are the same
but where listed below.

## The kernel's `Result<T>`

| Before | After |
|---|---|
| `KerfException` (`Diagnostics`, `Error`) | Removed |
| `result.Value` on a failure: `KerfException` | `InvalidOperationException`, a programmer's error ("The result is a failure, with no value: modeling.fillet-failed: …") |
| `result.GetValueOrThrow()` | Removed: check `IsSuccess`, or `TryGetValue`, `Match` |
| `Result.ThrowIfFailed()` | Removed: `IsFailure`, `Error`, `Match` |
| (none) | `Match(success, failure)` on `Result<T>` and `Result`; `Select` and `SelectMany` for LINQ's query syntax |

## Operations that return results

Each returns `Result<T>` where it returned `T`; its `Try` twin is gone.

| Before | After |
|---|---|
| `Operations.Fillet(…)`: `Part` or `Shape`; `TryFillet` | `Result<Part>` (edges) or `Result<Shape>` (vertices) |
| `Operations.Chamfer(…)`: `Part` or `Shape`; `TryChamfer` | `Result<Part>` or `Result<Shape>` |
| `Operations.Extrude(…)` (four overloads): `Part`; `TryExtrude` | `Result<Part>` |
| `Operations.Loft(…)`: `Part`; `TryLoft` | `Result<Part>` |
| `Operations.Revolve(…)`: `Part`; `TryRevolve` | `Result<Part>` |
| `Operations.Sweep(…)`: `Compound`, `Part` or `Sketch`; `TrySweep` | `Result<Compound>`, `Result<Part>` or `Result<Sketch>` |
| `Operations.Offset(…)`: `Compound`, `Part`, `Sketch` or `Curve`; `TryOffset` | `Result<…>` of the same |
| `Operations.Thicken(…)`: `Part`; `TryThicken` | `Result<Part>` |
| `Operations.Draft(…)`: `Part`; `TryDraft` | `Result<Part>` |
| `part.Add`, `Subtract`, `Common` (Part, Sketch, Curve): the class; `TryAdd`, `TrySubtract`, `TryCommon` | `Result<Part>`, `Result<Sketch>`, `Result<Curve>` |
| `Importers.ImportBrep`, `ImportStep`, `ImportGltf`: `Shape` | `Result<Shape>` |
| `Importers.ImportStl`: `Face` | `Result<Face>` |

## Unchanged, as the shorthand

The operators (`a + b`, `a - b`, `a & b`, on any shape), the objects' constructors (`new Box(…)`, `new Spline(…)`), the
topological shapes' methods (`solid.Fillet(…)`, `Solid.Sweep(…)`) and the exporters still return the value. A failure
there throws `InvalidOperationException` (it was `KerfException`), with the same message: "box 0 × 1 × 1 failed: The
value must be a positive number". `ModelFunctions.Run` shows any exception's message, so these cells read as before.

## Failures whose type or message changed

| Before | After |
|---|---|
| STEP that isn't STEP: `ArgumentException` "Could not import STEP data: it isn't readable STEP data. …" | A failure, `exchange.failed`, the same message |
| STEP past a limit: `ArgumentException` | A failure, `exchange.failed` |
| STEP that refers to other files, read from a stream: `InvalidDataException` | A failure, `exchange.external-references`, the same message |
| STL or glTF that can't be read, or past a limit: `InvalidDataException` | A failure with the exchange's code (`exchange.malformed`…), the same message |
| BREP that isn't BREP: `KerfException` `exchange.malformed` | A failure, `exchange.malformed` |
| BREP past its byte limit: `InvalidDataException` | A failure, `exchange.limit-exceeded`, the same message |
| A thickening that fails: `InvalidOperationException` "Error applying thicken to given surface" | A failure, `modeling.thicken-failed`, "thicken failed: The surface can't be thickened." |
| A solid's offset that fails: `InvalidOperationException` "offset Error, an alternative kind may resolve this error" | A failure, `modeling.offset-failed`, "The offset failed; another kind of join may work." |
| A loft that can't be recovered: `InvalidOperationException` "Failed to create valid loft" | A failure, `modeling.loft-failed`, "The loft is invalid and couldn't be recovered." |

## In Bento

A cell turns a failed result into `#VALUE!` with the first error's message, as it showed the exception's:
`result.Match(value => …, error => …)`, or `TryGetValue`. A chain inside one function can stay a chain:

```csharp
Result<Part> part = from rounded in Fillet(edges, radius)
                    from cut in rounded.Subtract(holes)
                    select cut;
```
