# Unit 11R: no success for an invalid result, for Bento's branch

The owner's decision (kernel `docs/decisions.md`, 1742): no operation reports success for an invalid result, and a failure may carry the shape.

## What changes in a cell

**Nothing has to change for Bento to build, or to show the reason.** `ModelFunctions.Run` already turns an exception into `#VALUE!` with its message, and the shorthands (`a - b`, `Fillet(...).Value`) throw `InvalidOperationException` as for any other failure. What's different is which cells reach it:

| Cell | Before | Now |
|---|---|---|
| A sweep that folds over itself (`CAD_PT_SWEEP`) | A shape, and a warning | `#VALUE!`: "The sweep's result isn't valid: … A path with a sharp corner needs …" |
| A Boolean with an invalid input, or that comes out invalid | A shape that later cells trip over | `#VALUE!`: "The cut's result isn't valid: …" |
| A profile whose wires cross, or a face from them | A face or solid that's invalid | `#VALUE!`, at the first cell that makes it |
| A fillet past what the part takes | `#VALUE!` with `modeling.fillet-failed` | `#VALUE!` with `result.invalid`; "Try a smaller radius, or MaxFillet to find the largest that works" stays |

Eleven of the guide's examples, with their volumes, face counts and triangles, are unchanged. Search Bento's samples and workbooks for cells that relied on the warning: their shapes were already invalid, and now they say so.

## What Bento may want to do

**Still draw the shape.** A cell that failed this way can offer it to the viewer, marked as broken:

```csharp
catch (Exception ex) when (ex is not (OperationCanceledException or PendingCellException))
{
    // The error stays what the cell holds; what the viewer draws is up to Bento
    if (ModelingDiagnostics.TryGetInvalid(ex, out Shape invalid))
        ShowAsInvalid(call, invalid);
    return CellValue.FromError(CellError.Value, ex);
}
```

It shouldn't become the cell's value: formulas that use the cell should see the error, and `Value` throwing is the point. Meshing an invalid shape can fail or give holes, so the viewer should catch that as it does any other failure to mesh (`ShapeModels.cs`).

A function that returns `Result<T>` to Bento gets the same from `result.TryGetInvalid(out var invalid)`. `result.Error.Shapes` names the faulty sub-shapes, which a viewer can highlight.

## Cost

Bento pays once per operation: `Kerf.Modeling` checks what its Boolean's clean leaves, and a plain extrusion checks only its profile. It adds about 13% to a recalculation on a loaded machine, which is over what the brief allowed, so the owner is choosing how far to apply it: this note describes the branch as built. The figures are in kernel `docs/performance.md`, "Unit 11R". There's no setting to turn this off in `Kerf.Modeling`. The kernel's options have `CheckResult = false` for a caller that checks a later step itself.

## API

- `DiagnosticCode.InvalidResult` (`result.invalid`).
- `Result<T>.TryGetInvalid`, `Result<T>.Invalid`.
- `ModelingDiagnostics.TryGetInvalid(Exception, out Shape)`.
- `ValidationReport.ToDiagnostic(string)`.
- **Gone:** `HealingOperations.NotFixed` (`healing.not-fixed`): `Fix` fails with `result.invalid`, carrying the shape. `Warnings` no longer says "sweep created an invalid solid".
