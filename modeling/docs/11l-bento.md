# Units 11L and 11N: what Bento sees change

Units 11L and 11N change messages, STEP's header and one member's name, and add an option that's off by default (kernel
decisions 18401 to 18407). Shapes, volumes and face counts are unchanged, and so are STEP's entities apart from the product's
name. This is every change a caller meets. **A member not listed keeps its name and meaning.**

## What Bento has to change

| Where | Before | After |
|---|---|---|
| `Extensions/Bento.Cad/IoFunctions.cs`, the check on a read shape's label | `shape.Label.StartsWith("Open CASCADE STEP translator ", …)` | Both prefixes: `"Kerf STEP translator "` as well, since files already written keep the old name |
| `Extensions/Bento.Cad.Tests/IoFunctionTests.cs`, the header's contents | `'build123d'` in `FILE_NAME` | `'Kerf.Modeling'` |
| `docs/cad/build123d-in-bento.md`, the product names quoted | `Open CASCADE STEP translator 8.0 1.2` | Either, as above |
| Any call of `shape.BoundBox(…)` | `shape.BoundBox()`, `BoundBox(optimal: false)` | `shape.Bounds()`, `Bounds(optimal: false)`. `shape.BoundBox(tolerance)` has no equivalent: the tolerance did nothing |

Bento's own code doesn't call `BoundBox` today, so the table's last row only matters to new code.

## STEP's header

A file written by Kerf now starts:

```text
FILE_DESCRIPTION(('Kerf Model',
    'Kerf is derived from Open CASCADE Technology 8.0'),'2;1');
FILE_NAME('Kerf Shape Model','2026-11-05T14:32:36',('Author'),('Kerf'),
  'Kerf STEP processor 0.3.0','Kerf 0.3.0','Unknown');
```

`ExportStep` puts the shape's label and the time stamp in `FILE_NAME` as before, and `'Kerf.Modeling'` in place of
`'Kerf 0.3.0'`. A product with no name is `Kerf STEP translator 1`. Files naming Open CASCADE read as they did, and nothing
reads these strings back. Anything comparing written STEP with a stored file changes once.

## Messages

Messages in a cell can read differently. No code or exception type changes.

| Before | After |
|---|---|
| `Expected gp_Pln, Face, Location, Axis, VectorLike, or three points` (a zero `zDir` or `xDir`) | `A plane's directions can't have zero length` |
| `extrude not supported for TopoDS_Vertex`, `Invalid sewn shape TopoDS_Compound`, `SewedShape returned a TopoDS_Solid…` | `… for Vertex`, `… shape Compound`, `… returned a Solid…` |
| `Null TopoDS_Shape object` | `The shape is null` |
| `'x' is not defined as a named color in CSS3 or OCCT/X11` | `… in CSS3 or X11` |
| A diagnostic's `the algorithm couldn't complete: BRep_API: command not done` | `the algorithm couldn't complete: command not done` |

Any failure from the engine loses OCCT's member at its start, and a type named later on keeps only its last part. Its
`InnerException` still has OCCT's words, as the way back to OCCT's source: anything showing `ex.ToString()` to a user would
show them, so show `ex.Message`.

## A new option, off by default

`StepExportOptions.CleanDuplicates` (and the kernel's `StepWriteOptions.CleanDuplicates`) merges exactly equal points,
directions, vectors, lines, planes, circles and placements. Over the dataset's 289 files, files are 17.9% smaller and read back
with the same volumes and face counts ([performance.md](../../kernel/docs/performance.md)). Nothing changes unless it's asked
for: whether it's on by default is the owner's decision.
