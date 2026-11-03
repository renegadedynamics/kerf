# Collections plan: what 10J did and what is left

*Unit 10J, 3 November 2026. Phase10-plan.md 2.6 replaces OCCT's NCollection classes with .NET's collections. The whole of
it is far more than one unit (about 25,000 references, counted below), so 10J did the measured hot spots, the returned
collections, the constant tables and the analyser, and this file is the plan for the rest.*

## The counts

References to each type by name (`grep -w`), at 10J's start, in the kernel's sources (files in brackets), the kernel's
tests and benchmarks, and the modelling layer:

| Type | Sources | (files) | Tests | Modelling | Target |
|---|---:|---:|---:|---:|---|
| `NCollection_Array1` | 7,473 | (613) | 3,321 | 11 | `T[]`/`Span<T>` 0-based; `ReadOnlySpan<T>` in signatures |
| `NCollection_List` | 5,568 | (355) | 420 | 12 | `List<T>`; `LinkedList<T>` only where an algorithm splices |
| `NCollection_HArray1` | 3,434 | (687) | 200 | 4 | as `NCollection_Array1` (the handle goes) |
| `math_Vector` | 2,502 | (302) | 1,352 | 0 | `Span<double>` over `stackalloc`/pooled buffers where small and local |
| `NCollection_Sequence` | 1,816 | (331) | 220 | 1 | `List<T>`, a deque where it prepends |
| `NCollection_DataMap` | 1,044 | (205) | 77 | 3 | `Dictionary<,>`, frozen where built once |
| `NCollection_Array2` | 960 | (110) | 499 | 3 | `T[]` row-major with a small 2D view; `Grid<T>` in public |
| `NCollection_Map` | 836 | (157) | 78 | 4 | `HashSet<T>` |
| `NCollection_HSequence` | 725 | (114) | 79 | 1 | as `NCollection_Sequence` |
| `NCollection_IndexedMap` | 618 | (133) | 344 | 2 | `IndexedSet<T>` (below) |
| `NCollection_IndexedDataMap` | 465 | (115) | 54 | 7 | `IndexedMap<TKey, TValue>` (below) |
| `NCollection_DynamicArray` | 455 | (105) | 36 | 0 | `List<T>` with capacity |
| `NCollection_HArray2` | 418 | (87) | 40 | 0 | as `NCollection_Array2` |
| `NCollection_IntMap`, `IntDataMap`, `PackedMapOfInteger` | 348 | (58) | 43 | 0 | `HashSet<int>`/`Dictionary<int,>`; the packed map's bit blocks stay where they are measured |
| `NCollection_LinearVector`, `LocalArray`, `AliasedArray`, `Vector` | 143 | (37) | 5 | 0 | `List<T>`, `stackalloc` |

## What 10J did

- **Measured first** (decisions 1400–1401): the Booleans' profile has no `Transform3D` cost to fix; its hot spots
  were the shapes' identity hash, a rectangular array per elementary extremum, and the mesher's list removal.
- **Shape maps:** a TShape draws its hash once (`TShape._hash`); the maps iterate in insertion order, so nothing
  depends on its value. KERF0002 recognises Kerf's shape types again (it had matched nothing since the rename).
- **The mesher's circles** (`BRepMesh_CircleTool.Select`) are a `List<int>`, deleted entries overwritten rather than
  removed: the one splicing use that mattered (performance.md proposal 3), without a linked list.
- **Small arrays:** `Extrema_ExtElC`/`ExtElC2d` hold their results in `[InlineArray]` fields;
  `math_NewtonMinimum` keeps its `math_Jacobi` and `math_Gauss` (and their work vectors) across iterations
  (proposal 4).
- **Returned collections:** `BSplineSurface` and `BezierSurface` return `Grid<T>` (0-based) from `Poles()`,
  `Weights()` and `WeightsArray()`.
- **Constant tables:** STEP AP214's type tables and MoniTool's typed-value library are frozen dictionaries.
- **`NCollection_Map` is gone:** every use is a `HashSet<T>` (decision 1406), by a Roslyn rewrite from the compiler's
  symbols (`Extent()`/`Size()` to `Count`, `IsEmpty()` to `Count == 0`, copies keeping the comparer).

## What is left, in order

Each step is one type, one commit, measured with 9E's benchmarks, every suite keeping its counts. The rewrite tool of
10J (a Roslyn pass like kerf-rename, keyed by the wrapper's members) does the mechanical part; what it reports by hand.

1. **`NCollection_DataMap` → `Dictionary<,>`.** Its members map one to one except the `ref` returns (`ChangeFind`,
   `Bound`, `TryBound`, the indexer), which become `CollectionsMarshal.GetValueRefOrNullRef`/`GetValueRefOrAddDefault`
   helpers, and `Bind`'s returned bool. Iteration order is the same (both are a `Dictionary`).
2. **`NCollection_DynamicArray`/`Vector` → `List<T>`**, keeping the first capacity (9E's fix); the `ref` returns
   become `CollectionsMarshal.AsSpan`. Order unchanged.
3. **Indexed maps.** Algorithms depend on their order and on 1-based indices (the Booleans' data structure numbers
   every shape by its index). They need one small ordered type, not `Dictionary`:
   - `IndexedSet<T>`: a `List<T>` of keys and a `Dictionary<T, int>` of positions, 0-based; `Add` returns the
     position, `IndexOf` returns −1 when absent, `this[int]`, `Count`, `RemoveAtSwap` (OCCT's `RemoveFromIndex`: the
     last key takes the place), `Substitute`, `Swap`; enumerates in order.
   - `IndexedMap<TKey, TValue>`: the same with a parallel `List<TValue>` and `ref` access to values.
   - .NET 9's `OrderedDictionary<,>` was considered: its `RemoveAt` shifts the rest, where OCCT moves the last
     element, so the order after a removal would differ; `SetAt` and `RemoveAt` could emulate it, but every caller
     would have to.
   - The conversion is the delicate one: every `FindIndex`, `FindKey`, `FindFromIndex`, loop bound and stored index
     moves by one. Do it file by file, with the oracle and the corpus run per package (the Booleans first).
4. **`NCollection_List` → `List<T>`.** OCCT's list is linked; the port's is already a `List<T>`, so order and
   results stay. `Prepend`, `InsertBefore` and `Remove(Iterator)` are O(n): search the converted code for them in
   loops (9E found two; 10J fixed the mesher's) and use a deque or `LinkedList<T>` only where a profile shows one.
   The `Iterator` class (`More`/`Next`/`Value`) becomes `foreach` or an index.
5. **`NCollection_Sequence`/`HSequence` → `List<T>`**, 1-based to 0-based, with a deque where it prepends
   (`IntTools_FClass2d`, proposal 6).
6. **`NCollection_Array1`/`HArray1`/`Array2`/`HArray2`**, the largest (12,000 references in sources), last: arrays and
   spans, 0-based. Their bounds are part of OCCT's algorithms (poles from 1, knots from 1, arbitrary lower bounds in
   the approximations), so the conversion goes package by package with the arithmetic checked, and can stop at the
   engine's boundary where a package keeps OCCT's indexing internally.
7. **`math_Vector`/`math_Matrix`** in inner loops: span-based scratch over `stackalloc` (at most 32 elements, OCCT's
   inline buffer) in the hot ones of 9E's profile and the Booleans' (`math_FunctionSetRoot` built per
   `Extrema_GenLocateExtPS.Perform`: 18 vectors each, 2% of the Booleans' bytes; `math_Recipes`'s `LU_Decompose` and
   `Jacobi` work vectors; `AppParCurves`' matrices). `math_VectorBase` could also hold its array directly rather than
   through an `NCollection_Array1` (one object fewer per vector), once `Array1()`'s callers no longer rely on the
   shared view.

Also left: the static empty lists handed out by `BOPDS_DS`, `BRepFill`, `ChFi3d`, `TopOpeBRepDS` and
`TopoDS_Iterator` (shared mutable instances: they become `[]` returned as read-only, with their callers); the Bézier
classes' shared knot and multiplicity arrays (become `ReadOnlySpan<T>` with the BSplCLib signatures, step 6); and the
surfaces' public constructors, which still take `NCollection_Array2` (10L's builders take `Grid<T>`).

Question 55, the transient base: `NCollection_Array1`, `Array2` and `Sequence` derive from `Transient` only for OCCT's
handles; it goes with steps 5 and 6.
