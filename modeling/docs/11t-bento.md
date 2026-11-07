# Unit 11T: stopping a cell, for Bento's branch

Unit 11T looked for what can hang or end the application (kernel `docs/reliability.md`; decisions 1853 to 1858). One thing needs Bento.

## What Bento has to change

**Cells can't stop Kerf today.** `CalcEngine` abandons the cell it's working on when an edit arrives, by checking `reader.Cancellation` between steps of a formula. A modelling function takes no token (`a - b` can't), and `ModelFunctions.Run` passes none, so a cell inside a fillet carries on to the end: 12 s on a fillet that was going to fail, as measured below, with the user waiting and the processor busy. Only the file functions in `IoFunctions.cs` can stop today.

In `Extensions/Bento.Cad/ModelFunctions.cs`, `Run` needs one line:

```csharp
internal static CellValue Run(FunctionCall call, Func<ModelArguments, object?> body)
{
    using var cancellation = Kerf.Cancellation.Use(call.Reader.Cancellation);   // new
    try
    {
        return ObjectConverter.FromResult(body(new ModelArguments(call)));
    }
    ...
```

Every Kerf operation inside then obeys it, including in its parallel loops, and throws `OperationCanceledException`, which `Run` already lets through. Nothing Kerf hands back is left half-made, as shapes are immutable. Any other place that calls Kerf from a cell, such as the viewer's meshing if it runs for a cell, should do the same with its own token.

## What to expect

Time from cancelling to returning, on a loaded machine: see `reliability.md`, "Cancellation". The worst before was all of a fillet, 24 s; a typical stop now takes a few milliseconds.

## What else changes for Bento, with nothing to do

- **A failure where the application used to die.** Compounds and assemblies nest at most 64 levels. A chain of cells that wraps a compound in a compound fails at the 65th, with a message, rather than ending Bento with a stack overflow at about 200 to 3,000.
- **Very long loops of moves** no longer end the process, and stay linear.
- **`#VALUE!` where a request is too large to be meant.** A mesh of more than 10 million triangles (`MeshOptions.MaxTriangles`), a pattern of more than 10 million locations. Tessellating or exporting at a tiny tolerance fails at once rather than taking all the memory.
- **Results don't change** otherwise.

## Tests for Bento

A long fillet in a cell, with an edit part way through: the cell should restart within a moment, and the first calculation shouldn't go on using a processor. The case behind the 12 s: a loft of a 10 × 10 square to a circle of radius 4, 10 above it, all edges filleted at 3.94.
