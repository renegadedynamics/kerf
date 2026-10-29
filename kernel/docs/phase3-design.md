# Phase 3 design: immutable topology

Phase 3 ports OCCT's topology (TopoDS, BRep and their tools) and its BRep input/output. This document decides how
topology becomes immutable to callers while staying a mechanical, class-for-class port. It extends section 8 of
[translation-rules.md](translation-rules.md) and decision 92 ("geometry and topology objects freeze when shared").
[phase3-plan.md](phase3-plan.md) splits the work into units.

The short version:

- **`TopoDS_Shape` and its eight subtypes are structs**: a TShape reference, a `TopLoc_Location` and an
  orientation, with OCCT's value semantics.
- **TShapes, their representations and attached geometry are classes that freeze.** They freeze in two ways:
  - Geometry, triangulations and polygons freeze **eagerly**, as soon as `BRep_Builder` attaches them.
  - TShapes and their representation lists freeze **on publication**: when OCCT.NET hands a shape to code outside
    it. That happens when a public algorithm returns its result and when a shape is read from a file.
- **Frozen implies OCCT's `Locked`.** OCCT's own guards then do most of the work. `BRep_Builder` throws
  `TopoDS_LockedShape` on a locked shape, and the Booleans switch to non-destructive mode when an argument is locked.
- **An algorithm that must write into a frozen input works on a copy.** It copies the shape's structure and shares
  the geometry, through `BRepTools_Modifier` with `BRepTools_CopyModification`, and composes the copy's history with
  its own.
- **Triangulations stay where OCCT keeps them**, on faces and edges, and freeze with them. Meshing returns a meshed
  copy.

Sizes and counts below come from OCCT V8_0_1 (`b8f597c6`), measured on 29 October 2026. Comments were stripped
first.

## 1. What OCCT does

A `TopoDS_Shape` is a value of three fields: `occ::handle<TopoDS_TShape> myTShape`, `TopLoc_Location myLocation` and
`TopAbs_Orientation myOrient`. C++ copies it freely and changes the copies in place: `E.Orientation(...)`,
`E.Reverse()`, `S.Location(...)`, `S.Move(...)`, `S.Nullify()`. In ModelingAlgorithms, ModelingData and
DataExchange there are:

- 1,143 calls of `Orientation(x)`,
- 785 of `Reverse()`,
- 170 of `Location(x)`,
- 67 of `Move`.

ModelingAlgorithms and ModelingData also hold 5,362 downcasts `TopoDS::Edge(S)`, `TopoDS::Face(S)` and the like.

**`TopoDS_TShape`** (Standard_Transient) holds:

- a list of child `TopoDS_Shape`s, and
- a 16-bit state: the shape type plus the flags Free, Modified, Checked, Orientable, Closed, Infinite, Convex and
  Locked.

`BRep_TVertex`, `BRep_TEdge` and `BRep_TFace` add their geometry:

| TShape | Holds |
|---|---|
| `BRep_TVertex` | a point, a tolerance, and a list of `BRep_PointRepresentation`: points on curves, on surfaces, on curves on surfaces |
| `BRep_TEdge` | a tolerance, the flags SameParameter, SameRange and Degenerated, and a list of `BRep_CurveRepresentation`: the 3D curve, curves on surfaces, curves on two surfaces (regularity), 3D polygons, polygons on surfaces and on triangulations |
| `BRep_TFace` | a surface, a location, a tolerance, NaturalRestriction, and a list of `Poly_Triangulation`s with an active one |

The representations are Standard_Transient objects that `BRep_Builder` changes in place. For example, `UpdateEdge`
finds the edge's existing `BRep_GCurve` and calls `SetRange`, or replaces its pcurve.

### OCCT's own flags

- **Free.** `TopoDS_Builder::Add(S, C)` first sets `C.Free(false)` (comment: "From now the Component cannot be
  edited"). It then throws `TopoDS_FrozenShape` unless `S.Free()`. `Remove` checks the same. So OCCT already freezes a
  shape's child list once the shape becomes a child. Two things undermine it:
  - Algorithms call `Free(true)` to edit used shapes anyway: 36 places, 28 of them in BRepOffset, BRepFill and
    ShapeFix.
  - `BRep_Builder`'s update methods ignore Free. They must: building a face adds pcurves to edges that are already in
    its wire.
- **Locked.** `BRep_Builder` throws `TopoDS_LockedShape` from all its update methods (32 checks) when the target is
  locked. `BOPAlgo_PaveFiller` switches itself to non-destructive mode when any argument is locked
  (`BOPAlgo_PaveFiller_10.cxx:56`). `BRepTools::UpdateFaceUVPoints` stops at the first locked edge. Only reading a file sets
  Locked; BRepOffset clears it.
- **Modified and Checked** are cache-validity bits:
  - `Add` and `Remove` set Modified, and setting Modified clears Checked.
  - `BRepTools::Update(Face)` recomputes the UV end points of the face's pcurves and sets Checked.
- **Orientable, Closed, Infinite and Convex** are properties that algorithms set.
- **All eight flags are written** to `.brep` files, text and binary.

### Identity

- `IsPartner` compares TShape pointers.
- `IsSame` compares the TShape and the location.
- `IsEqual`, which is also `operator==`, compares the TShape, the location and the orientation.
- `std::hash<TopoDS_Shape>` combines the TShape's address with the location's hash, without the orientation, so it
  agrees with both IsSame and IsEqual.
- `TopTools_ShapeMapHasher` is IsSame with that hash. 2,622 map declarations name it explicitly.

### Who writes into topology they didn't create

The question for an immutable port is how often later algorithms write into TShapes they were given rather than
TShapes they built. A regex count over the code bodies of the toolkits that phases 4 to 7 port gives:

| Pattern | TopAlgo | Prim | BO | Bool | ShHealing | Fillet | Offset | Mesh | Feat |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `B.UpdateEdge` | 42 | 18 | 18 | 115 | 91 | 4 | 83 | 4 | 15 |
| `B.UpdateVertex` | 13 | 15 | 13 | 53 | 96 | 19 | 49 | 0 | 24 |
| `B.SameParameter/SameRange/Degenerated/Range/...` | 46 | 11 | 37 | 106 | 112 | 4 | 116 | 0 | 8 |
| Flag setters (Closed, Checked, Orientable, ...) | 24 | 9 | 17 | 56 | 18 | 3 | 39 | 0 | 23 |
| `Free(true)` | 0 | 0 | 2 | 16 | 6 | 0 | 12 | 0 | 0 |
| `ChangeCurves`/`ChangePoints`, direct `TE->Tolerance` | 11 | 0 | 20 | 11 | 26 | 0 | 18 | 0 | 0 |
| In-place utilities: `BRepLib::SameParameter`, `UpdateTolerances`, `BuildCurves3d`, ... | 36 | 3 | 4 | 16 | 8 | 5 | 48 | 0 | 5 |
| `BRepTools::Clean`/`Update`/`RemoveUnusedPCurves` | 0 | 4 | 0 | 2 | 12 | 0 | 8 | 1 | 2 |
| Triangulation or polygon stored | 27 | 0 | 2 | 0 | 0 | 0 | 0 | 51 | 0 |
| Copies (`EmptyCopied`, `BRepBuilderAPI_Copy`, NonDestructive) | 30 | 1 | 68 | 50 | 54 | 2 | 15 | 0 | 21 |
| `BRepTools_ReShape`/`ShapeBuild_ReShape` | 28 | 3 | 0 | 7 | 147 | 0 | 8 | 0 | 0 |

A heuristic classified the 1,133 `BRep_Builder` update calls in these toolkits by where the first argument comes
from (a throwaway script, not committed):

| Where the argument comes from | Calls | Share |
|---|---:|---:|
| Built in the same function | 198 | 17% |
| A function parameter | 207 | 18% |
| An explorer or container | 9 | 1% |
| Couldn't be classified | 719 | 63% |

Reading the unclassified ones shows mostly `MakeX` → `UpdateX` → `Add` sequences on new shapes, and edits of
shapes built earlier in the same algorithm. The parameter writes are concentrated:

| File | Writes to a parameter |
|---|---:|
| BRepOffset_MakeOffset | 18 |
| BRepFill | 16 |
| BRepOffset_MakeSimpleOffset | 15 |
| ShapeBuild_Edge | 13 |
| BRepLib_MakeShell | 12 |
| BRepFill_Sweep | 11 |
| ShapeFix | 30 |

So most writes are to an algorithm's own new topology, and writes into topology it was given come from six
recognisable families:

1. **In-place utilities.** These are the static functions that exist to edit a shape: `BRepLib::SameParameter`,
   `UpdateTolerances`, `UpdateInnerTolerances`, `BuildCurves3d`, `EncodeRegularity`, `OrientClosedSolid`,
   `BRepTools::Clean`, `CleanGeometry`, `RemoveUnusedPCurves`, `RemoveInternals` and `Update`, and
   `BRepLib::ContinuityOfFaces`. There are about 110 call sites.
2. **Healing.** ShapeFix, ShapeUpgrade and ShapeBuild edit in place, with a ReShape context recording the
   replacements.
3. **Booleans.** BOP grows the tolerances of argument vertices and edges and adds pcurves to argument edges, unless
   it is non-destructive. In non-destructive mode it works on `EmptyCopied` sub-shapes.
4. **Sweeps, offsets and drafts that share their input.**
   - `BRepSweep_Translation::SetPCurve` adds the pcurves of the lateral faces to the generating edges (build123d.NET's
     tapered-extrusion case).
   - BRepOffset and BRepFill unlock (`Free(true)`, `Locked(false)`) and edit shapes they were given.
5. **Meshing.** BRepMesh stores triangulations on the faces it is given and polygons on their edges.
   `BRepLib_ToolTriangulatedShape::ComputeNormals` writes normals into an attached triangulation.
6. **Cache completion.** `BRepTools::Update(Face)` (UV points and the Checked flag) and the Modified and Checked
   bookkeeping.

build123d.NET met families 1, 2, 3, 4 and 5 on native OCCT ("Immutability in the kernel" in its design notes). It
fixed them by copying (`TopoUtil.PrivateCopy`), by non-destructive Booleans and by ignoring meshes in bounding boxes.
Its `InputsUnchangedTests` compares inputs as BRep text and has to ignore only the Free and Checked flags.

The phase 3 algorithms themselves (BRepGProp, BRepBndLib, BRepCheck, BRepExtrema, BRepClass, BRepClass3d,
IntCurvesFace, BRepTopAdaptor) never write into their inputs. They only build new shells and solids of their own:
BRepCheck_Shell, BRepClass3d, BRepGProp.

## 2. TopoDS_Shape is a struct

`TopoDS_Shape` follows TopLoc_Location (decision 26) and the value types (rules, section 3). It is a
`public partial struct` with OCCT's three fields: `TopoDS_TShape? myTShape`, `TopLoc_Location myLocation` and
`TopAbs_Orientation myOrient`. That is 24 bytes.

- **Accessors are public:** `TShape()`, `Location()`, `Orientation()`, `ShapeType()`, `IsNull()`, `NbChildren()`,
  the flag getters (`Free()`, `Closed()`, ...), `IsPartner`, `IsSame` and `IsEqual`.
- **OCCT's non-mutating forms are public:** `Located`, `Oriented`, `Moved`, `Reversed`, `Complemented` and
  `Composed`. `EmptyCopied` is internal (changed in unit 3A, decision 306): it creates an unfrozen TShape, which
  nothing a caller holds may be (4.2), and only a builder can fill it.
- **The mutators are `internal`**, under the rules for value types: `Location(loc)`, `Orientation(o)`, `Move`,
  `Reverse`, `Complement`, `Compose`, `Nullify`, `EmptyCopy`, `TShape(t)` and the flag setters.
  - The OCCTNET0001 analyzer already catches a mutator called on a readonly copy.
  - The flag setters write to the shared TShape, so they also check it isn't frozen (section 4).
- **The constructor.** OCCT's default constructor gives a null shape with orientation `TopAbs_EXTERNAL`.
  `default(TopoDS_Shape)` would have `TopAbs_FORWARD`, so the struct gets an explicit parameterless constructor
  (as `gp_Dir` does). Ported code writes `new TopoDS_Shape()`, and NCollection's element construction (decision 15)
  does the same. `default` differs only in the orientation of a null shape.

**The eight subtypes are structs too.** `TopoDS_Vertex`, `Edge`, `Wire`, `Face`, `Shell`, `Solid`, `CompSolid` and
`Compound` each have the same three fields and all of TopoDS_Shape's members.

- **Conversions.** Each converts implicitly to `TopoDS_Shape`, which is the C++ upcast. Narrowing is
  `TopoDS.Edge(S)` and friends, which check the type and throw `Standard_TypeMismatch` as OCCT does.
- **Overloads** resolve as in C++. `BRep_Tool.Tolerance(TopoDS_Edge)` is chosen for an edge, and the
  `TopoDS_Shape` overload for a shape.
- **Writing into a subtype through a `ref TopoDS_Shape`** works through
  `ref Unsafe.As<TopoDS_Edge, TopoDS_Shape>(ref e)`, which the layouts make safe, wrapped in internal helpers
  (`TopoDS.AsShapeRef(ref e)`). It is needed only where C++ passes a derived variable to a non-const
  `TopoDS_Shape&` that changes the value's fields.
  - `TopoDS_Builder::Add` and `Remove` change only the TShape, so they take the shape by `in`.
- **The members are generated.** C# structs can't inherit, so a script (`tools/gen-topods-subtypes.cs`) writes the
  eight subtype files from TopoDS_Shape's members. They are regenerated, not edited.

Rejected:

- **A class hierarchy.** Every one of the 2,000-odd in-place changes of a shape value would become an aliasing bug
  or a hand rewrite. A null shape would become a null reference (`E.IsNull()` → NullReferenceException), and every
  explorer step would allocate.
- **One struct with `using` aliases for the subtypes.** OCCT's overload sets on Edge, Face and Vertex
  (`BRep_Tool::Tolerance`, `BRepTools::Update`, `TopExp::Vertices`, ...) would collide.

## 3. Identity, equality and hashing

| OCCT | Port |
|---|---|
| `IsPartner`, `IsSame`, `IsEqual` | the same, public |
| `operator==`, `!=` | `==`, `!=`, `Equals`, `IEquatable<T>`: **IsEqual** (TShape by reference, location by decision 27, orientation) |
| `std::hash<TopoDS_Shape>` | `GetHashCode()`: `RuntimeHelpers.GetHashCode(TShape)` combined with the location's hash, without the orientation. Consistent with both IsSame and IsEqual. |
| `TopTools_ShapeMapHasher` | an `IEqualityComparer<TopoDS_Shape>` singleton (`TopTools_ShapeMapHasher.Instance`): IsSame, same hash |
| typedefs `TopTools_MapOfShape`, `IndexedMapOfShape`, `DataMapOfShape*`, ... | written out with the comparer (rules, section 9) |

**Subtypes.** Each subtype's `Equals(object)` accepts any of the nine structs, so a boxed `TopoDS_Edge` equals the
boxed `TopoDS_Shape` that IsEqual matches.

**The comparer argument.** Forgetting the comparer gives IsEqual instead of IsSame: a silent change of meaning, since
OCCT names the hasher in 2,622 map declarations. New analyzer rule **OCCTNET0002** therefore requires an explicit
comparer on every `NCollection_Map`, `IndexedMap`, `DataMap`, `IndexedDataMap`, `HashSet` and `Dictionary` keyed by
a TopoDS type. A porter writes `TopTools_ShapeMapHasher.Instance` or, where OCCT really uses `std::hash`,
`EqualityComparer<TopoDS_Shape>.Default`.

**Hash values** are process-specific, as OCCT's are (addresses). Insertion-ordered maps are deterministic. The
iteration order of the unordered maps differs from OCCT's (decision 18) and, keyed by addresses, isn't stable even in
OCCT.

**What this gives Bento.** Bento gets the "cheap identity" it asked for (bento-integration.md, section 4):

- two cell values are the same shape iff `Equals`, and
- the hash can key its mesh cache.

Shapes computed again are new TShapes, so they compare unequal even when geometrically identical, as in native OCCT.

## 4. When topology freezes

Decision 92 gives every Standard_Transient a one-way frozen flag:

- `Freeze()` is internal,
- `IsFrozen()` is public,
- `CheckMutable()` is the first line of every mutator.

Phase 3 decides where TShapes, representations and attached objects freeze.

### 4.1 Geometry, triangulations and polygons freeze when attached

Every `BRep_Builder` method that stores an object freezes it. That applies to:

- a `Geom_Curve`, `Geom2d_Curve` or `Geom_Surface`,
- a `Poly_Triangulation`, `Poly_Polygon3D`, `Poly_Polygon2D` or `Poly_PolygonOnTriangulation`.

So does every `BRep_*Representation` constructor that keeps one, and `BRep_TFace`'s setters. This is rules section
8's "when topology takes it".

These objects are shared across shapes from the moment they're attached:

- the same surface on many faces,
- the same curve on an edge and its copies,
- `BRep_Tool::Curve` handing the stored curve to any algorithm.

Ported code changes them in place surprisingly often (`C2d->Translate(...)` after `BRep_Tool::CurveOnSurface` in
BRepOffset, for example). Eager freezing turns each such place into an exception, and the port copies first, as for
Geom (decisions 127, 132, 245).

**Poly.** Poly_Triangulation and the polygon classes get `CheckMutable()` in their mutators now. That is the
follow-up decision 63 promised. Their mutators stay public, because a mesher or an importer fills them as builders
(decision 63), but they throw once the object is attached. The cached range of a triangulation stays writable: it
is a cache of the frozen nodes (decision 316).

**The Active bit.** OCCT's `BRep_TFace` marks its active triangulation in the triangulation's purpose
(`Poly_MeshPurpose_Active`) and clears the bit of the one it replaces. A frozen triangulation may be shared by another
face, so the port leaves its purpose as it is (decision 309).

### 4.2 TShapes and representations freeze on publication

A TShape can't freeze when it is attached to a parent, as OCCT's Free does for child lists. OCCT builds a face and
then adds pcurves to edges already in its wire, and Booleans and sweeps update sub-shapes long after they're
attached. A TShape can't freeze when its constructor finishes either: that is decision 92's rejected option.

**The rule: every TShape that code outside OCCT.NET can reach is frozen. TShapes that only the running algorithm can
reach are not.** Concretely, a shape graph is frozen (`TopoDS_TShape.FreezeGraph()`, internal) at four points:

1. **A public result.** When a public method returns a shape it built, or a container of them, the graph is frozen
   first. This covers:
   - `Shape()`, `Edge()`, `Face()`, ... of the `BRepLib_Make*` and (from unit 3K) `BRepBuilderAPI_*` classes,
   - `BRepTools_Modifier.ModifiedShape`,
   - `BRepTools_ReShape.Apply` and `Value`,
   - `BRepTools_Quilt.Shells`,
   - `BRepTools.OuterWire` of a frozen face,
   - every result of a later phase's public algorithms.

   This is GC's `Value()` and GeomConvert's `BSplineCurve()` again (decisions 185, 243). An algorithm's
   `BRepTools_History` lists only shapes that are frozen by then.
2. **A read.** `BRepTools.Read`, `BinTools.Read`, and `Read` of the shape sets freeze the shapes they create, after
   the whole set has been read. Reading sets flags after adding children, so freezing must come last.
3. **Draw-lite.** A shape stored in a Draw variable is frozen (section 9).
4. **Explicit sharing inside the port.** An algorithm that keeps a shape and hands it to more than one consumer
   freezes it, as Geom's `BasisCurve()` does. Units note each place.

**Internal callers.** Ported code that continues to build on a sub-algorithm's result reads it through an internal
accessor that doesn't freeze. For example, `BRepLib_MakeFace` followed by `B.UpdateFace` reads the face through
`InternalShape()` (as `InternalCurve()`, decision 244). A missed place throws on the first test that reaches it.

**The walk.** `FreezeGraph` freezes in post-order:

1. the TShape's children,
2. its representations, and through them their geometry (already frozen by 4.1, so this costs nothing),
3. the TShape itself.

It stops at TShapes that are already frozen. That keeps the invariant **"a frozen TShape's whole sub-graph is
frozen"**, and makes the cost proportional to the TShapes created since the last freeze. Inputs, which are already
frozen, are never walked again. Concurrent walks of the same graph only repeat volatile writes of `true`, so they
are harmless.

**The freeze isn't recursive in general.** Freezing a `Geom_TrimmedCurve` doesn't freeze its basis, and nothing
needs it to. This answers decision 127's open point. The composite Geom and Geom2d objects already protect their
parts:

- the parts are reachable only through accessors that freeze them (`BasisCurve()`, `BasisSurface()`),
- the owner's own mutators throw once the owner is frozen,
- internal arrays are never handed to callers (decision 131).

So `Standard_Transient.Freeze()` stays non-virtual and shallow. The only deep structure is the topology graph, and
the one virtual is `TopoDS_TShape.FreezeRepresentations()`, which `BRep_TVertex`, `BRep_TEdge` and `BRep_TFace`
override.

### 4.3 Frozen implies Locked

`TopoDS_TShape.Locked()` returns `lockedBit || IsFrozen()`. That reuses every guard OCCT already has:

- `BRep_Builder`'s 32 checks throw `TopoDS_LockedShape` on a frozen target, with OCCT's message.
- `BOPAlgo_PaveFiller` goes non-destructive when an argument is frozen, so Booleans on frozen inputs work on copies
  without new code.
- `BRepTools::UpdateFaceUVPoints` stops at the first frozen edge (OCCT's `return`, occt-issues.md).

Three things need care:

- **Writers.** No file format stores the Locked bit: the text and binary writers write the seven flags Free to
  Convex. Only `TopTools_ShapeSet::Dump` prints it, and the port prints the raw bit (internal `LockedFlag()`), not
  `Locked()` (unit 3C, decision 381).
- **Unlocking.** `Locked(false)` on a frozen shape throws (BRepOffset will need a copy there).
- **Paths that bypass `BRep_Builder`** still throw `Standard_ImmutableObject` through `CheckMutable()`. These are
  `ChangeCurves()`, `ChangePoints()`, `TE->Tolerance(...)`, `UpdateTolerance`, the flag setters, the
  `TopoDS_Builder` child list, and the representations' setters (`SetRange`, `PCurve(...)`, `Location(...)`, ...).
- **The lists themselves** (`BRep_TEdge::Curves()`, `BRep_TVertex::Points()`, `BRep_TFace::Triangulations()`) are
  mutable `NCollection_List`s, so their accessors are internal; callers get read-only copies (decision 310).

### 4.4 The flags

| Flag | Rule |
|---|---|
| **Free** | Kept as OCCT for child lists: `Add` and `Remove` on a non-free parent throw `TopoDS_FrozenShape`. Its setter needs a mutable TShape, so `Free(true)` on a frozen shape throws. One departure: `TopoDS_Builder::Add` **doesn't clear Free on a frozen child**. A frozen shape keeps the Free value it had when frozen, so its BRep text never changes. Native OCCT clears it, so after a shape has been put into a compound, native OCCT writes `0` in its first flag column and the port writes `1`. Oracle comparisons build inputs that haven't been reused, or compare that column apart. This is the one flag build123d.NET's `InputsUnchanged` test ignores today (owner question 1 in phase3-plan.md). |
| **Modified, Checked** | Set only on unfrozen TShapes. `BRepTools::Update(Face)` and the other cache completions return at once on a frozen face: it can't have changed since it was frozen. Each such early return is marked `// OCCT.NET: frozen` and listed in decisions.md. |
| **Orientable, Closed, Infinite, Convex** | Setters are internal and check that the TShape isn't frozen. Algorithms that compute them for an input (`BRepLib::OrientClosedSolid`, `BRepTools::DetectClosedness` callers) do so on their copy. |
| **Locked** | As 4.3. |

### 4.5 BRep_Builder

`BRep_Builder` and `TopoDS_Builder` are mutators, so they are **internal** (rules, section 3). Their behaviour on a
frozen target:

| Situation | Result |
|---|---|
| The target is frozen | Throws `TopoDS_LockedShape` (4.3), or `Standard_ImmutableObject` on paths OCCT doesn't guard. Never a silent copy. |
| A frozen child is added to an unfrozen parent | Allowed. That is how new shapes share their inputs' sub-shapes. |
| A Make method is given a variable holding a frozen shape | `MakeEdge` and `MakeFace` throw `TopoDS_LockedShape`, as OCCT does for a locked one; ported code makes into a new variable. |
| Geometry or a mesh is attached | The object is frozen (4.1). |

Callers outside OCCT.NET build shapes through public algorithms. Phase 3 has the `BRepLib_Make*` classes and
`BRepBuilderAPI`'s makers (unit 3K). For compounds there is an addition, `TopoDS_Builder.MakeCompound(params
TopoDS_Shape[])` and an `IEnumerable` overload, returning a frozen compound. (`TopoDS_Builder` is therefore a public
class whose constructor and members are internal; `BRep_Builder` is an internal class. The Make methods take the
shape by `ref`, decision 307.) build123d.NET uses `BRep_Builder`
directly today, and moves to these when it switches over (phase 9).

Rejected: **copying silently in `BRep_Builder` when the target is frozen** (copy-on-write at TShape level). A
replaced TShape isn't seen by the parents that still hold the old one. OCCT's algorithms rely on an update being
visible to every parent: they add a pcurve, then explore the face and expect to find it. Results would be silently
wrong.

### 4.6 Writing into frozen inputs: copy first

When ported code (phases 4 to 7) must change a frozen input, it works on a **structural copy**:

- **What is copied:** new TShapes, and new representation objects pointing at the same frozen Geom and Poly objects.
- **How:** `BRepTools_Modifier` with `BRepTools_CopyModification(copyGeom: false, copyMesh: true)`, as
  `BRepBuilderAPI_Copy` does. It is wrapped in an internal helper `BRepTools.CopyForWrite(in TopoDS_Shape, out
  BRepTools_History, bool theCopyMesh = true)` in unit 3B.
  - The copy keeps the meshes, so that it has what the in-place operation would have seen (changed in unit 3B,
    decision 349): the shared, frozen triangulations, with each face's whole list and its active one (the
    modification carries the active one only), and the polygons.
  - Polygons on surfaces (Poly_Polygon2D) aren't carried, as in BRepBuilderAPI_Copy. The copy's vertices get their
    points on curves again from the modifier.
- **What happens next:** the algorithm runs OCCT's code unchanged on the copy. Its history is the copy's history
  composed with its own (`BRepTools_History.Merge`), so `Modified`, `Generated` and `IsDeleted` answer for the
  caller's original sub-shapes. build123d.NET does this for the tapered extrusion.

**The in-place utilities of family 1.** OCCT's `void f(const TopoDS_Shape& S, ...)` keeps its in-place form as an
internal method for ported code working on its own shapes. It gets a public functional twin returning the new shape,
and its history where OCCT keeps one. Every such twin goes in decisions.md, as each `With` twin does.

- In phase 3 these are `BRepTools.Clean`, `CleanGeometry`, `RemoveUnusedPCurves`, `RemoveInternals`, `Update`,
  `ActivateTriangulation`, `LoadTriangulation` and `UnloadTriangulation`.
- In unit 3I they are `BRepLib.SameParameter(shape)`, `UpdateTolerances`, `UpdateInnerTolerances`, `BuildCurves3d`,
  `EncodeRegularity`, `OrientClosedSolid` and `UpdateDeflection`.
- The twins copy up front. An optimistic form (run in place, and on the first frozen write start again on a copy)
  could avoid the copy when nothing changes. It is left until profiling asks for it.

**Families 2 to 5** are later phases' decisions, made on the same pattern:

- **Booleans:** already non-destructive on frozen inputs through Locked.
- **ShapeFix and offsets:** copy at the API entry.
- **Sweeps:** copy the profile when it would receive pcurves.
- **Meshing:** section 5.

Each place is recorded in decisions.md (rules, section 8).

## 5. Triangulations and polygons

Meshes stay where OCCT keeps them: `BRep_TFace`'s list of triangulations, and the polygon representations on
`BRep_TEdge`. They freeze with the shape.

- **Reading:** a `.brep` with triangles reads as a shape carrying them, frozen.
- **Writing:** `BRepTools.Write(..., withTriangles)` writes what the shape carries, so round trips are OCCT's.
- **Readers:** BRepBndLib (`useTriangulation`), BRepGProp's mesh properties, BRepExtrema's proximity tools, BRepCheck
  and later HLR read `BRep_Tool::Triangulation` unchanged. A shape value always gives the same answer.
- **Meshing (phase 4)** returns a **meshed copy**: copy for write (4.6), mesh the copy, freeze it, return it. That is
  build123d.NET's `Meshed`, and it is the only way to get a mesh onto a shape.
  - Bento's viewer keys its mesh cache by the input shape (`Equals`/`GetHashCode`).
  - `BRepTools.Clean`'s public twin returns a copy without meshes.
  - `ComputeNormals` runs on the triangulation before it is attached, or on a copy.
- **Thread safety:** a frozen shape and everything reachable from it are read-only. The remaining lazily computed
  state is already published safely:
  - Poly_Triangulation's cached range (a `Lock`, decision 69),
  - the B-spline `Resolution` caches (volatile, decisions 126, 135),
  - per-adaptor caches, which are never shared.

  Bento can calculate cells that share shapes on several threads with no locks. The meshing lock that
  build123d.NET needs (bento-integration.md, section 3) isn't needed.

Rejected: **a side cache of meshes** (a `ConditionalWeakTable<TopoDS_TShape, ...>` that `BRep_Tool::Triangulation`
consults).

- It makes results depend on what other cells have done: a bounding box would change because another cell meshed a
  shared TShape. That is the action at a distance this design removes.
- It needs its own locking.
- It changes every reader of `BRep_Tool::Triangulation`.
- `BRepTools::Write` would have to choose between the cache and the shape.

## 6. Unfrozen topology inside algorithms

**What stays the same.** Inside an algorithm everything is OCCT:

- TShapes it creates are mutable, shared between its intermediate shapes, and visible to every parent after an
  update.
- `Free`, `Modified` and `Checked` behave exactly as in C++.

**Where unfrozen shapes live.** Unfrozen shapes never leave OCCT.NET, and never cross threads except inside an
algorithm's own `Parallel.For`, as in OCCT, for example `BRepCheck_Analyzer`'s parallel mode.

**The cost.** It is one flag per object, one check per mutation, and one walk per published result, linear in the
TShapes created.

## 7. BRep input and output

**Streams and numbers.** The I/O builds on phase 2:

- C++ stream state lives beside the TextWriter and TextReader (`Standard_Stream`, decision 225).
- Numbers are formatted as the C runtime does.
- GeomTools sets freeze what they read (decision 227).
- Poly is written with Poly's writer (decision 68).

**Text.** `BRepTools_ShapeSet : TopTools_ShapeSet` with `TopTools_LocationSet` and the three GeomTools sets.

- Shapes, locations and geometry are numbered by insertion-ordered indexed maps, so the output order is OCCT's.
- Format versions 1 to 3, with or without triangles and normals, as `TopTools_FormatVersion`.
- **Line endings.** The stream overloads write `'\n'`, as decision 226 does. The file-path overloads also write
  `'\n'`, as native OCCT 8 does (it opens the file in binary mode; older releases wrote CRLF on Windows through
  `std::ofstream`'s text mode). Reading accepts CR.
- **Corpus files.** The `.brep` files in OCCT's corpus come from older releases, and reading normalises directions
  again. They round-trip in the sense of decision 233's F1/F2 scheme: native OCCT 8 reads the file and writes F1,
  reads F1 and writes F2; the port reads F1 and must write F2's bytes (unit 3C: every readable file does).

**Binary.**

| Format | Port |
|---|---|
| `BinTools_ShapeSet` (versions 1 to 3) | as OCCT |
| `BinTools_ShapeWriter`/`ShapeReader` (version 4: shared references by stream position) | as OCCT |
| `BinTools_IStream`/`OStream` | over `Stream` with `BinaryPrimitives` little-endian, OCCT's on-disk order |

**Reading freezes.** Every shape returned from a read is frozen (4.2), so a read shape and a computed one behave
alike.

**Writing doesn't write into the shape.** OCCT's writers only read, apart from `BRepTools::Write`'s optional
`CheckLocations`, which also only reads. The Locked column is the raw bit (4.3) and the Free column is the stored
value (4.4).

**Untrusted bytes** (bento-integration.md, section 7). A managed port can't crash the process the way native OCCT
can, but three failures remain: a stack overflow, a huge allocation and a long loop. The readers stay OCCT's, with
four additions, each marked `// OCCT.NET: untrusted input`:

- Counts read from the stream (shapes, nodes, triangles, poles) are checked against what the remaining input could
  hold before allocating.
- Nesting of offset and trimmed geometry is capped (GeomTools' readers recurse), as are compound depths.
- Words longer than OCCT's buffers are an error. OCCT's `char buffer[255]` overflows; the native shim checks for it
  today.
- Any failure surfaces as `Storage_StreamFormatError` (a Standard_Failure).

A fuzz test reads truncated and mutated corpus files.

**Draw's own formats** (`.rle`, DBRep's "DBRep_DrawableShape" header) are read by Draw-lite's `restore` through the
same shape sets, as DBRep does.

## 8. BRepGraph

OCCT 8's BRepGraph (44,700 lines) and BRepGraphInc (11,200) are a new incidence-table model of topology with UIDs,
layers, an editor and history. **Nothing in OCCT V8_0_1 uses them:**

- no algorithm in any toolkit,
- no Draw command,
- no test script.

The only users are their own 40,000 lines of GTests. They are **not ported in phase 3**, and can wait until an
upstream algorithm or a caller needs them. Their design, which keeps mutation behind an editor with version stamps,
is worth reading before any redesign of the model, but that isn't this port's business (design.md: a mechanical
port).

## 9. Draw-lite shapes

Draw keeps shapes in "drawable" variables (`DBRep_DrawableShape`), separate from Tcl's string variables:
`isdraw`, `dtyp`, `dname` and `whatis` query them, and `copy` and `renamevar` move them.

- **Storage.** Draw-lite adds a drawable table to `DrawInterpreter`, with names as Draw's, holding frozen
  `TopoDS_Shape` values and, later, geometry.
- **Commands that change a shape in place.** Some Draw commands change the shape a variable names:
  - `add` into an existing compound, `setflags`, `settolerance`, `purgeloc`, `emptycopy`;
  - `orientation`, `reverse` and `complement` change only the value.

  They replace the variable with a changed copy. OCCT's in-place change would also be seen by other variables
  naming the same TShape. Corpus scripts don't rely on that; Draw-lite notes it.
- **Output.** Commands print what Draw prints, so the DrawResources procedures (`checknbshapes`, `checkprops`,
  `checkmaxtol`, ...) parse it unchanged (decision 37).

## 10. Recommendation, alternatives and risks

**Recommended:** sections 2 to 7.

- Value-type shapes.
- Geometry frozen when attached.
- TShapes frozen when published or read, with "frozen implies Locked" reusing OCCT's guards.
- Structural copy for writers, with composed history.
- Meshes kept on the shapes.

It keeps the port mechanical:

- OCCT's code runs unchanged on unfrozen topology.
- The changes are confined to the publication points, the internal accessors and the functional twins, each
  recorded.
- A write into shared topology throws, so it can't go unnoticed.

**Alternatives considered:**

| Alternative | Why rejected |
|---|---|
| TopoDS_Shape as a class | Section 2: aliasing of the 2,000-odd in-place changes, null references, allocations |
| Freeze when attached to a parent (OCCT's Free) | Building faces and Booleans update attached edges and vertices |
| Freeze inputs on entry to public algorithms, instead of outputs on exit | The same internal-call problem in reverse (ported code passes its own intermediate shapes to helpers and keeps editing them), and an unfrozen result held by a caller could still be changed by a later internal path |
| Ownership tokens: each TShape records the operation that created it (an `AsyncLocal` context), and mutation is allowed only in that operation | Every public entry point must open a context, and parallel loops must carry it. A per-mutation context read. Hard to audit. Freezing gives the same guarantee with a flag. |
| No freezing; copy every input of every algorithm (PrivateCopy everywhere) | O(n) copies on read-only operations too. Breaks identity, so history-based selection and Bento's caching fail. A port bug writing to a shared shape stays silent. |
| Copy-on-write of single TShapes in BRep_Builder | Parents keep the old TShape, so updates aren't seen by them (4.5) |
| Allow append-only representation additions on frozen edges (pcurves for new faces) | Shared edges would accumulate pcurves for every face built on them over Bento's recalculations. The BRep bytes of a shared shape change. Appends race. |
| Meshes in a side cache | Section 5 |
| A separate frozen flag, not mapped to Locked | It would duplicate OCCT's 32 `BRep_Builder` checks and BOP's non-destructive switch, which Locked gives for free |
| Clear Free on frozen children (OCCT's exact bytes after reuse) | A shared shape's BRep text would change when another cell puts it into a compound, which breaks value semantics |
| Port BRepGraph now | Section 8: unused |

**Risks:**

1. **Writes into frozen topology are found at run time, not compile time.** A missed internal accessor or an
   unlisted in-place write throws `TopoDS_LockedShape` or `Standard_ImmutableObject` in a test, or, if no test
   reaches it, in a caller's operation.
   - *Mitigation:* every oracle and corpus test runs on frozen inputs (read from bytes), so they reach the writers.
   - *Mitigation:* each later unit greps its toolkit for the patterns of section 1 and lists the input writes it
     found.
2. **Copy cost.** Writers that copy pay O(size of input) per call: ShapeFix, offsets, sweeps on shared profiles,
   meshing, the in-place utilities' twins. Booleans already pay it in OCCT's non-destructive mode.
   - *Mitigation:* copy only where a write was found.
   - *Mitigation:* the optimistic retry of 4.6 where profiling shows it matters.
3. **History.** Every copy must be composed into the algorithm's history, or build123d.NET's history-based
   selection (`Select.Last`, `Select.New`) loses track of the caller's sub-shapes. Each copying algorithm needs an
   oracle test of `Modified`/`Generated`/`IsDeleted` against native OCCT.
4. **The Free column.** A frozen shape later put into a compound writes Free = 1 where native OCCT writes 0 (4.4).
   Oracle tests that compare BRep bytes after such reuse must account for it.
5. **The comparer.** A map ported without `TopTools_ShapeMapHasher` changes IsSame to IsEqual silently.
   *Mitigation:* OCCTNET0002 (section 3).
6. **Struct subtypes.** C++ code that passes a derived shape to a non-const `TopoDS_Shape&` and relies on the
   caller's variable changing needs `TopoDS.AsShapeRef`. C# rejects `ref` through an implicit conversion, so each
   such place is a compile error, not a silent copy. A porter who "fixes" it by copying into a `TopoDS_Shape` local
   and forgetting to copy back loses the change. *Mitigation:* review rule in the plan's notes for porters.
7. **Hostile input.** The checks of section 7 add code that isn't OCCT's. *Mitigation:* fuzz the readers against
   the native shim's rules.
8. **Scope.** BRepExtrema and BRepCheck reach into TKGeomAlgo (2D and curve-surface intersections) and BRepLib,
   which design.md placed in phase 5. Phase 3 therefore ports about 124,000 lines rather than the 34,000 of TKBRep
   alone. See phase3-plan.md.
