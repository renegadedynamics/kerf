# Caching models in Bento: use cases and an outline

*For whoever builds this in Bento (unit 11F of Kerf's `kernel/docs/phase11-plan.md`). Written 6 November 2026 from Bento's branch `kerf/0.2.0` and Kerf's `main`. It's a starting point: nothing here has been prototyped, and no gain below has been measured. Check the code it cites before relying on it.*

## The idea

A Kerf shape is immutable, so a result can be reused whenever the same operation meets the same inputs. Bento recalculates more than it has to, and then meshes what it recalculated, so most of the work in an edit may be avoidable. Measure first (section 6): results already recalculate quickly, and an edit may turn out to be dominated by something else.

## 1. What happens today

- **An edit marks everything downstream dirty, and all of it runs.** `CalcEngine.MarkDirty` and `MarkDependents` mark dependents transitively before anything is calculated. `Complete` compares old and new values only for tracing (`CalcEngine.cs`, about line 861), so a cell that recalculates to the same value still recalculates all its dependents. **The plan's "Bento knows which cells really changed" isn't so yet:** it knows which were edited.
- **Every run makes new objects.** A CAD function called with the same arguments builds a new shape, and everything downstream then sees a shape that isn't the same.
- **Meshes are cached by object.** `ShapeModels.Models` is a `ConditionalWeakTable` on the value, so it helps when a cell's value is shown again, as the selection moves or stages are compared. A recalculated cell holds a new object and meshes from scratch.
- **The engine runs one cell at a time on its own thread,** and abandons a cell when another edit arrives. Volatile cells (`NOW`, `RAND`) are dirtied by every edit, and so is everything downstream of them.

## 2. Use cases

| | What the user does | Today | What would help |
|---|---|---|---|
| 1 | Undo or redo an edit to an early dimension | The whole chain runs again, and meshes again | Function results |
| 2 | Drag or retype a value, coming back to one used before; what-if and data tables that revisit values | Each visit is a full run | Function results |
| 3 | Edit a formula so that one argument changes, with the rest as before | All of it runs | Function results, for its unchanged parts |
| 4 | Many cells or `MAP` rows build the same thing, such as a hundred identical bolts | Built and meshed each time | Function results, then instancing in the viewer |
| 5 | Edit an input whose effect stops part-way: `MAX(A1, 10)` still 10, a selection that picks the same edge, a rounded dimension | Everything after it runs | Early cut-off |
| 6 | A workbook with `NOW()` or `RAND()` somewhere, with models downstream of a value that doesn't change | Runs on every edit anywhere | Early cut-off |
| 7 | Change only a part's colour or name, or move it | New object, meshed again | Reuse the mesh: moved, or under a new name |
| 8 | Change a late feature on a large part: one fillet of fifty faces | Only the last steps run, but the whole result is meshed | Meshes per face (section 5), if Kerf can offer it |
| 9 | Change the last step of a chain, editing the final fillet's radius | Only that step runs, as it should | **Nothing:** the baseline, and it must not get slower |
| 10 | Open a workbook | Everything runs once | Nothing here. A cache saved to disk is out of scope |

## 3. The outline, in order

**Stage 0: measure** (section 6), so that each stage has a before and after.

**Stage 1: reuse function results, inside `Bento.Cad`.** This covers use cases 1 to 4, needs no change to the engine, and is the place to start. `ModelFunctions.Add` wraps nearly every CAD function, so it's the one place to put this:

- **The key:** the function's definition and its arguments. Look up, return the same result object on a hit, and otherwise run and store it.
- **Hits carry on down a chain:** returning the same object means the next function's key matches too. The formulas still run, cheaply, and a hit also finds the mesh already cached by object, with no change to `ShapeModels`.
- **What to store:** results only. No errors at first, no abandoned or cancelled runs, and nothing quicker than a small threshold, such as 1 ms, since looking up a cheap result costs as much as running it.
- **Where it lives:** one per `CalcEngine`, or per workbook, which needs no locks as the engine has one thread. A process-wide table could share more between workbooks, but would need locks and a clear owner.
- **How it's bounded:** by a budget. The table holds results strongly, so it keeps them alive where the weak table doesn't. Evict the least recently used, and estimate a result's size (faces and triangles) as a first measure. Clear it when a workbook closes. A spreadsheet can't have an unbounded heap.

**Stage 2: early cut-off, in the engine.** This covers use cases 5 and 6, and every formula in Bento gains from it, not only models. It changes `Bento.Calc`'s core, so it's the larger risk, and the owner should decide on it after Stage 1's figures:

- Dirty still spreads eagerly, so snapshots show the right cells pending.
- Before running a dirty cell, check what it read last time (`_dependencies` holds it). If those cells are clean and their values are the same, keep the old value and clear the cell, without running it. If one is dirty, calculate that first, which the engine's pending mechanism already does.
- Stage 1 is what makes this work for models: a shape recalculated from equal arguments is then the same object.
- Volatile cells, cells that read visibility or `Now`, and spills need care: leave them out at first.

**Stage 3: reuse meshes under a change of placement or appearance** (use case 7). `CadPart` already has `Named` and `WithColor`, sharing its mesh. A key of the shape without its location, plus the location, could reuse a moved part's mesh. Instancing in the viewer belongs here too (`modeling/docs/bento-integration.md`, section 3, "Instances").

## 4. Traps

**Don't use `Equals` for cache keys, on shapes or geometry.** All of these are checked in Kerf's code:

- **`ModelShape.Equals` is `IsSame`:** it ignores orientation, and also label and colour (`Topology/ModelShape.cs`, line 252). A reversed shape, or one renamed or recoloured, compares equal, so a key on `Equals` returns the wrong one. **Compare shapes by reference.** It's stricter and always safe: a miss costs time, never a wrong answer.
- **Geometry's `Equals` rounds:** `Location`, `Plane` and `Axis` compare through `GeomKey.Round`, to `Constants.KeyDigits` decimals, so two different values can compare equal. A dimension changed in the last digits would return the old shape. Compare exact components instead.
- **Numbers by their bits:** `0.0 == -0.0` is true, and `NaN == NaN` is false. Compare `BitConverter.DoubleToInt64Bits` so that a key is exact and NaN can still hit.
- **Lists:** element by element, under the same rules. `ShapeList.Equals` compares as a set, ignoring order, so don't use it either. Compute the hash once, as lists can be long.
- **Option records:** every field, in a defined order.

**Some functions mustn't be cached as they stand:**

- **Imports** (`CAD_IO_IMPORT_STEP`, `_BREP`, `_GLTF`, `_STL`): the file can change under the same path. Leave them out, or add the file's length and last-write time to the key.
- **Exports** (`CAD_IO_EXPORT_*`): the write is the point, so never skip it.
- **Functions taking a function** (`MAP`, or a lambda passed to a CAD function): a closure's identity says little about what it captured. Leave them out.
- **Anything volatile,** and anything that reads the workbook's state rather than its arguments. Prefer a list of what may be cached over a list of what may not, so that a new function is safe by default.

**Sharing one object between cells has to be safe.** Check these before Stage 1:

- `ModelShape`'s `Label`, `Color` and `Children` have private setters. Confirm that nothing in Kerf or Bento changes a shape after it's handed out: anything that does breaks once two cells share a result.
- Look for anything in Bento that uses a value's identity as if it were the cell's. Picks and selectors map a face back through the shape's model, and two cells now holding one object could confuse the viewer's parts or a pick.
- Kerf gives no guarantee of a result being unaffected by cancellation unless it completes, which is why cancelled runs aren't stored.

**Other things to watch:**

- **The diff and history views:** `WorkbookDiff.SameValue` compares by `Equals`. More shapes will now be the same object across edits, which should only make it more accurate. Check it.
- **Determinism:** reuse assumes that a Kerf operation gives the same result from the same inputs. Kerf's rig checks this, including under parallel load, and Bento's tests rely on it, but a test here is cheap: for a sample of cached results, run the function again and compare volumes and face counts. Keep it as a debug check.
- **A failing operation:** when errors aren't stored, a formula that fails repeats its cost on every recalculation. Storing a failure is a reasonable later step, as long as the message is kept.

## 5. What Kerf could add, if the measurements ask for it

Bento shouldn't wait for these. Ask Kerf's maintainer if the figures point here:

- **A mesh per face (use case 8).** Booleans and fillets may keep unaffected faces' geometry, so a mesher that reuses a face's triangles would save meshing most of a large part. **Not checked:** whether faces keep their identity through a Boolean, whether edge meshes on shared boundaries allow this, or what it does to `MeshTolerances`. It needs its own investigation. Results would have to match a full mesh.
- **An exact key on geometry,** such as a public `ExactKey` on `Location`, `Plane` and `Axis`, so Bento needn't dig out components.
- **An estimate of a shape's size,** for the budget.

## 6. How to measure

The plan's measure of success is "recalculating an edit to one cell". Kerf's rig (`kernel/tools/bench.ps1`) times whole workbooks, so this needs a new benchmark, on Bento's side, on the examples in `Extensions/Bento.Cad/samples/introductory-examples.bento`, and on a long chain of about thirty features:

1. Each use case in section 2 as a script of edits, recording the time from the edit until the engine is idle and the mesh is ready, how many cells ran, and allocation.
2. How that time divides between formulas, Kerf's operations and meshing: this decides how much each stage can give. The engine's tracing (`TakeRecalculated`) already reports cells and depths.
3. Use case 9 and opening as the guards: neither may get slower, and memory after a long session of edits must stay within the budget.
4. Results compared with the cache on and off, over every example: the same values, volumes, face counts and triangles.

Use a quiet machine, and the runtime settings from `modeling/docs/11d-bento.md`.

## 7. Not in scope

A cache saved to disk; sharing across processes; parallel evaluation in the engine, though Stage 1's table would then need locks; and caching inside Kerf, whose operations stay pure.

## 8. Questions for the owner

1. **Stage 2 changes the core of `Bento.Calc`.** Do it if Stage 1's figures show use cases 5 and 6 still costing much? My recommendation is yes, on those figures, as every formula gains.
2. **The memory budget:** what's reasonable for a workbook? I'd start at a few hundred MB, measured against real sessions.
3. **The order of this against 0.3.0:** Bento's move to Kerf 0.3.0 will change `Bento.Cad` too (`modeling/docs/11l-bento.md`, `11d-bento.md`, `clean-up-bento.md`), so it's simplest to build this on top of that branch.
