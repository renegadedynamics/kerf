# Decisions

Decisions made while porting, newest last, so they can be reviewed. Each says what was decided, why, and what
would change it. Bugs in OCCT itself are in [occt-issues.md](occt-issues.md) instead.

## Phase 0 (28 October 2026)

1. **Port from OCCT V8_0_1** (`b8f597c6`, 30 July 2026), the latest stable release and the one build123d.NET builds,
   so its native build is the oracle. Upstream fixes are carried across later by diffing.
2. **LGPL 2.1 with the Open CASCADE exception** for the whole port (owner's decision).
3. **Mechanical, class-for-class port** keeping OCCT's names and layout, rather than a redesign. Makes review
   against the C++ and carrying upstream fixes practical.
4. **Value types are structs with internal mutators**, not `readonly struct`s. OCCT mutates in place everywhere;
   keeping the mutators lets ported code stay line-for-line. Callers outside OCCT.NET see only `readonly`
   members, so values are immutable to them. The hazard (mutating a defensive copy) is a compile error through
   the OCCTNET0001 analyzer. Reconsider only if the analyzer proves leaky.
5. **Every mutator gets a public non-mutating twin**: `WithFoo` for `SetFoo`, the past participle otherwise, and
   static factories where a `Set` rebuilds the whole value (`gp_Trsf.Rotation`).
6. **Methods stay methods** (`p.X()`), not properties, to keep ported code identical to the C++.
7. **No implicit conversions** between gp types: C++'s converting constructors are written out (`new gp_Dir(v)`).
8. **Exact equality** (`Equals`, `==`, `GetHashCode`) on all value types, for Bento; OCCT's tolerant comparisons
   stay as `IsEqual(other, tol)`.
9. **Argument checks always on.** OCCT's release builds compile most `Raise_if` checks out (`No_Exception`); the
   port keeps them, preferring a clear exception to garbage geometry. Oracle tests use valid inputs.
10. **OCCT's bugs are reproduced**, marked `// OCCT` and listed in occt-issues.md, to be fixed only once
    consistent behaviour with native OCCT is proven (owner's decision).
11. **Returned collections are Bento.Collections interfaces**; numeric arrays use `PackedList<T>`, a contiguous
    `IBentoList<T>` readable as a span, because Bento's lists are trees of 32-element leaves.
12. **Oracle via build123d.NET's bindings** (a sibling checkout), not bindings of our own: they exist, are generated,
    and wrap the same OCCT build. Classes they lack are added to their whitelist when needed.
13. **Draw-lite** (a small Tcl interpreter) to run OCCT's own 17,237 Draw test scripts, rather than writing tests
    from scratch for the modelling algorithms.
14. **Coefficient arrays of gp quadrics** (`gp_Pln::Coefficients` and friends) use `out` parameters, and
    `gp_Torus::Coefficients` a `Span<double>` of 35 indexed from 0 (OCCT fills a 1-based `NCollection_Array1`).

## Phase 1 (28 October 2026, overnight)

15. **NCollection is reimplemented, not ported line for line**: OCCT's API (bounds, Value/ChangeValue, More/Next
    iterators, 1-based sequences and indexed maps) over .NET arrays, lists and dictionaries. OCCT's own
    implementation is allocator and memory plumbing that .NET does differently. Behaviour that ported code can see
    is kept: element default construction, the last key moving into a removed index of an indexed map, buffer
    views sharing storage.
16. **NCollection containers are classes**, so C++ copies must be written as copies (`new(b)`, `Assign`). Rejected
    alternative: structs, which would still alias their storage and hide the problem. The rule is in
    translation-rules.md section 9; a review step checks by-value uses.
17. **NCollection_HArray1/HArray2/HSequence derive from the plain containers** rather than from Standard_Transient,
    since C# has single inheritance and arrays are references already.
18. **The unordered maps don't keep OCCT's iteration order.** Ported code whose output depends on that order is
    noted where found.
19. **Raw array pointers become spans** (`double*`/`double&` plus pointer arithmetic → `Span<double>`).
20. **Progress is OCCT's API on .NET types**: `Message_ProgressIndicator(CancellationToken, IProgress<double>)`.
    This is how Bento will cancel long operations. Simplification: a range handed out by `Next()` and never closed
    isn't counted, where OCCT counts it on destruction.
21. **Low-level packages keep OCCT's mutable parameter types** (NCollection arrays, spans): BSplCLib, PLib and math
    are tools for ported code. Immutability applies to values callers hold (gp now, Geom and TopoDS later).
22. **Trimming**: default construction uses `Activator.CreateInstance<T>()`, with the trimming warning suppressed on
    the grounds that OCCT.NET's value types keep their constructors. Revisit if Bento publishes with full trimming.
23. **TKMath is verified by OCCT's GTests, not the oracle.** build123d.NET's native bindings expose only Bnd_Box,
    Bnd_OBB, TopLoc_Location and two Poly classes from TKMath, and widening them means changing and rebuilding
    that repository. Those classes get oracle tests now; the rest of TKMath gets oracle coverage indirectly through
    Geom, GeomAPI and BRep, which build123d.NET does bind, from phase 2.
24. **Parallel porting uses git worktrees made by hand** (in the session's scratch directory), one branch per
    package group off the phase branch, merged back with `--no-ff`. The harness's own worktree isolation refused
    this repository's path (a drive-letter case mismatch) and left three locked worktrees in `.claude/worktrees`,
    which are ignored and can be removed with `git worktree remove --force`.

## Phase 1: ElCLib, ElSLib, GeomAbs, TopLoc (28 October 2026)

25. **ElCLib and ElSLib output parameters are `out`**, and where OCCT fills an output with `SetXYZ`/`SetX`/... the
    port assigns a new value built from the same expressions in the same order, so results are unchanged.
    `ElCLib::AdjustPeriodic`'s U1 and U2 are read and written, so `ref`.
26. **TopLoc_Location is a struct** holding one reference to a shared, immutable list of nodes, exactly OCCT's
    layout. It has value semantics in OCCT (copied freely, embedded in every TopoDS_Shape), costs no allocation for
    the identity, and `default(TopLoc_Location)` is a valid identity, as OCCT's default constructor. A class would
    add an allocation and a null that OCCT doesn't have. TopLoc_SListOfItemLocation and TopLoc_ItemLocation are
    structs for the same reason; the list node and TopLoc_Datum3D are Standard_Transient classes, as in OCCT.
27. **TopLoc_Location equality is OCCT's `IsEqual`**: datums compared by reference, then powers, so `Equals`,
    `==` and `GetHashCode` agree with OCCT's maps of locations and shapes. Two locations built from equal
    transformations are different, as in OCCT. `HashCode()` keeps OCCT's structure (0 for the identity, then each
    datum and power folded in from a fixed seed) but returns an `int` from `System.HashCode` over the datums'
    `RuntimeHelpers.GetHashCode`, where OCCT MurmurHashes their addresses into a `size_t`; both are
    process-specific. The deprecated aliases TopLoc_MapOfLocation and TopLoc_IndexedMapOfLocation are written out
    as `NCollection_Map<TopLoc_Location>` and `NCollection_IndexedMap<TopLoc_Location>`.
28. **`operator gp_Trsf()` of TopLoc_Location is an explicit conversion**, and the non-explicit
    `TopLoc_Location(const gp_Trsf&)` is written out, as for gp's implicit conversions.
29. **ShallowDump keeps a TextWriter overload**; the object's address becomes its runtime hash code and C++'s default
    six-digit stream formatting becomes "G6".

## Phase 1: the math package (28 October 2026)

30. **`math_Vector` and `math_IntegerVector` are global using aliases** of the generic class
    `math_VectorBase<T>` (`where T : struct, INumber<T>`), as they are `using` aliases in OCCT. The aliases are declared once in
    `src/Directory.Build.props` and `tests/Directory.Build.props` (every port and test project references
    OcctNet.Foundation; the analyzer project is excluded). Rejected: subclasses, whose inherited operators would return the base type.
31. **`math_Vector`, `math_Matrix` and `math_DoubleTab` copy explicitly.** C++ copy construction is `new(x)`;
    C++ assignment, including move assignment from a temporary, is `Initialized(x)` (`Assign` for
    math_DoubleTab), which copies values, keeps the bounds and throws if the sizes differ, as OCCT does. The
    compound operators (`+=`, `-=`, `*=`, `/=`) are C# 14 instance compound operators, so they change the
    object in place as in C++. Moves are reference transfers, so the GTests' checks on moved-from objects are
    dropped. OCCT's inline buffers for small vectors and matrices are not reproduced.
32. **Function interfaces are abstract classes with `out` results**: `bool Value(double X, out double F)`,
    `bool Values(math_Vector X, out double F, math_Vector G)`. Vectors and matrices are passed as references
    and filled in place. An implementation that fails must still assign its `out` values, where C++ may leave
    them unchanged; the ported algorithms that rely on an unchanged value (`math_KronrodSingleIntegration`'s
    error, `math_GlobOptMin::computeLocalExtremum`) use `ref` internally.
33. **A `const math_Vector&` returned from a field returns the object itself**, documented "shared: don't
    modify", rather than a copy: these are low-level tools (decision 21) and copying would cost every caller.
34. **`Dump(Standard_OStream&)` is `Dump(TextWriter)`**, and `ToString()` returns what it writes, for every math
    class with a Dump.
35. **math_Recipes' free functions** (`LU_Decompose`, `SVD_Decompose`, `Jacobi`, ...) are static methods of a
    static class `math_Recipes`, imported with `using static Occt.math_Recipes;`.
36. **NCollection_CellFilter** takes its inspector as a type parameter constrained to
    `INCollection_CellFilter_Inspector<TTarget, TPoint>` (static abstract `Coord`, static virtual `Dimension`
    and `IsEqual`), in place of OCCT's duck-typed template parameter.

## Draw-lite: the Tcl core (29 October 2026)

37. **Draw-lite runs OCCT's own DrawResources scripts** (StandardCommands, Geometry, TestCommands, CheckCommands,
    Vector.tcl) rather than reimplementing the check commands in C#. `checkprops`, `checknbshapes`, `checkreal` and
    the rest then behave exactly as in OCCT, on top of a few C++ primitives (nbshapes, sprops/lprops/vprops,
    checkshape, tolerance, isdraw/dtyp/dname, ...) to be written as the geometry is ported.
38. **Script writes never touch the disk**: `open ... w`, `file mkdir/delete/copy` go to an in-memory overlay that
    later reads see. Integers are 64-bit (Tcl switches to bignums; Draw-lite to doubles). Regular expressions run on
    .NET's engine with Tcl's syntax translated; .NET takes the first matching alternative where Tcl takes the
    longest. Only the global namespace exists. `checkview` stays a no-op.
39. **Each top-level evaluation runs on a thread with a 256 MB stack**, because debug builds overflowed the default
    stack at about 270 nested procedure calls; the procedure-depth limit stays at Tcl's 1000. A `Deadline` stops a
    runaway script and can't be caught by the script.

## Phase 1: Bnd, BVH and the NCollection vectors (29 October 2026)

40. **NCollection_Vec2/3/4 and NCollection_Mat3/4 are generic structs** (`where T : unmanaged, INumber<T>`) over
    `[InlineArray]` storage, so `v[i]` and `myMat[i]` read as in the C++. They follow the value-type rules: the
    non-const `x()` and `operator[]` become an indexer with an internal setter and an internal `ChangeData()` span;
    `GetData()` becomes `CopyTo`; converting constructors become static `From<TOther>`. Mat3/Mat4 default to the
    identity (`new()`), `default` is zero. OCCT's float/double specialisation of `Vec3`/`Vec4 /= scalar`
    (multiply by the inverse) is kept. `Mat3.Inverted(ref, out)` takes `ref` because OCCT leaves the output
    untouched for a singular matrix.
41. **BVH is generic over the element type and the vector type**, `BVH_Box<T, TVec>` with `TVec` one of the
    NCollection vectors, instead of OCCT's `BVH_Box<T, N>`: C# has no integer type parameters, and the vector type
    fixes the dimension. The vectors implement `INCollection_Vec<T, TSelf>` (static abstract `Length`, `Create`,
    operators, component access); generic code writes a component through an internal `Unsafe` helper. The JIT
    specialises every instantiation, so there is no interface dispatch in the inner loops. Rejected: concrete
    classes per (T, N), six copies of every builder. `BVH_Box<double, 3>` is spelled
    `BVH_Box<double, NCollection_Vec3<double>>`; the tests have global aliases for OCCT's typedefs (`BVH_Vec3d`).
    N = 1, declared but never instantiated by OCCT, is not ported.
42. **BVH template specialisations and int template arguments**: the binary tree is `BVH_Tree<T, TVec>` and the quad
    tree `BVH_QuadTree<T, TVec>` (OCCT's arity tags dropped); `Child<K>(n)` is `Child(K, n)`; the binned builder's
    `Bins` is its constructor's last argument; `BVH_Transform<T, N>` exists only for N = 4, so it is
    `BVH_Transform<T>`; the 3D-only `BVH_Box::Transform`/`Transformed` are extension methods. OCCT's
    `BVHSetType = void` is a generic overload without the set type (`BVH_Traverse<T, TVec, TMetric>`), and
    `Select()` finds the tree of a `BVH_PrimitiveSet` or `BVH_Geometry` set by type. `RejectNode` returns its
    metric through `out`.
43. **BVH_Set is an interface**, because `BVH_PrimitiveSet` is both a set and a `BVH_Object` (OCCT's multiple
    inheritance); its default `Box()` is the static `BVH_Set.Box(set)`.
44. **BVH's classes keep OCCT's public mutators** (trees, sets, builders), like math and BSplCLib (decision 21): they
    are tools for ported code, and users extend them (custom sets, traversals). Only the value types (`BVH_Box`,
    `BVH_Ray`, the vectors) follow the internal-mutator rule.
45. **std::partition and std::sort.** The radix sorter's `std::partition` is ported as Microsoft's implementation,
    so the linear builder (used by `Bnd_OBB`) orders primitives as native OCCT does on Windows. `BVH_QuickSorter`
    uses .NET's span sort instead of `std::sort`: both are unstable introsorts, but primitives with equal centroids
    may end up in a different order, and trees built by the sweep-plane builder may differ.
46. **OSD_Parallel → Parallel.For/ForEach** with OCCT's sequential flag; `OSD_Thread` → `Thread`;
    `std::mutex` → `Lock`; atomics → `Interlocked`/`Volatile`.
47. **NCollection_LinearVector** is reimplemented over an array (decision 15), API subset only (no move operations,
    iterators as `AsSpan()`); its 912-line GTest is not ported yet.
48. **Bnd_B2/Bnd_B3 are ported once per instantiation** (`Bnd_B2d`/`Bnd_B2f`, `Bnd_B3d`/`Bnd_B3f`), the float files
    generated from the double ones by a file-level `using RealType`, to keep C++'s float/double promotions exactly.
    C++ `std::optional` returns become nullable structs, and multi-value returns `readonly record struct`s
    (`Bnd_Box.Limits`, `Bnd_Range.Bounds`, `Bnd_OBB.HalfSizes`) for structured bindings.
49. **Twins for Bnd's bare `Set` and `Add` methods**: `Bnd_Box.FromPoint`, `Bnd_Box.Void()`, `Bnd_OBB.FromPoints`,
    `Bnd_OBB.Void()` (static factories), and `Added`, `Enlarged`, `Opened*`, `Updated`, `Limited`, ... Methods whose
    output OCCT leaves untouched in some cases take `ref` rather than `out` (`Bnd_Range.GetMin/GetMax/GetBounds`).

## Phase 1: Poly (29 October 2026)

301. **Poly_ArrayOfNodes and Poly_ArrayOfUVNodes keep OCCT's run-time precision** through a port of
     `NCollection_AliasedArray` that holds a .NET array of the element type (`gp_Pnt[]` or
     `NCollection_Vec3<float>[]`, `gp_Pnt2d[]` or `NCollection_Vec2<float>[]`, chosen by the stride) plus a byte
     offset, and reads it through typed references (`Unsafe.As`, `MemoryMarshal`). Rejected: a `byte[]` buffer
     (MapNodeArray couldn't share it as a `gp_Pnt` array) and two typed arrays in one class (OCCT's stride-based
     API wouldn't carry over). The "wrapping" constructors take an array and a start index and share it, as OCCT's
     share memory; `Move` leaves the moved-from array referring to the storage, as in OCCT.
302. **Bulk reads for Bento** (additions on Poly_Triangulation): `NodeCoordinates()`/`NodeCoordinatesF()` (x, y, z
     per node, without copying, in the stored precision; the other precision throws `Standard_TypeMismatch`),
     `CopyNodeCoordinatesTo(Span<double>)`/`(Span<float>)` (either precision), `TriangleIndices()` (three 1-based
     indices per triangle), `TriangleSpan()`, `UVNodeCoordinates()`/`UVNodeCoordinatesF()`, `NormalComponents()`.
     The spans are valid until the arrays are resized. Poly_Triangle is exactly three `int`s so the triangle
     array casts to indices.
303. **Poly_Triangulation and the polygon classes keep public mutators**: they are mutable mesh containers filled
     by the mesher (decision 21). Their mutators are to become internal (or to throw once frozen) when
     triangulations are attached to shapes. Poly_Triangle follows the value-type rules: `Set` and `ChangeValue` are
     internal, the twins are the constructor (for `Set(n1, n2, n3)`) and `WithNode`.
304. **Poly_CoherentNode, Poly_CoherentTriangle and Poly_CoherentLink are classes**, not structs, although OCCT uses
     them by value: the coherent triangulation stores pointers to its triangles and links (neighbours, per-node ring
     lists) and compares addresses, so they need identity. C++ assignments of them (`theTriangle =
     Poly_CoherentTriangle(...)`) are an internal `Assign`. Poly_CoherentNode holds a gp_XYZ instead of deriving from
     it. Allocator arguments are dropped. The triangulation's iterators, like OCCT's, don't visit elements appended
     during the iteration.
305. **Poly_Connect, Poly_MakeLoops and Poly_MergeNodesTool are classes** (algorithms; OCCT's first two are used by
     value). Poly_MakeLoops' helpers are abstract classes; `OnAddLink`, called only by OCCT's debug builds, is
     never called. `Poly_MakeLoops.Link` is a struct with public fields as in OCCT, public `Reverse`/`Nullify`, and
     OCCT's unordered `operator==` as `Equals`/`==` (the equality its maps use).
306. **Poly_MergeNodesTool's node map is ported with OCCT's buckets**: NCollection_BaseMap's bucket counts
     (`NCollection_Primes`, added), resizing and rehash order, the DJB2 hash of the float position with MSVC's
     signed `char`, and the cell hash. When several merged nodes could match a new one, the first found depends on
     the bucket chains, so this keeps OCCT's choice. The element map, whose order doesn't matter, is an
     `NCollection_Map`.
307. **`NCollection_PackedMapOfInteger`** (OCCT's `NCollection_PackedMap<int>`, alias `TColStd_PackedMapOfInteger`)
     is ported with its buckets and iteration order, because Poly_MakeLoops takes its next start link from that
     order. 32-bit instantiation and set operations only (no boolean operations between maps).
308. **Poly's streams**: `Write`/`Dump` take a `TextWriter` and format numbers as a default C++ stream does (`%g`
     with six digits: .NET's "G6" with a lowercase exponent; `inf`, `nan`), `std::setw` as padding, so files are
     OCCT's byte for byte. `Read*` take a `TextReader` and follow C++ extraction: a failed read gives 0 and fails
     every later read. `Poly::PolygonProperties`, a template on the container, has overloads for
     NCollection_Array1 and NCollection_Sequence (the instantiations OCCT uses). `Poly::Intersect` and
     `IntersectTriLine` take their outputs by `ref` (unchanged on a miss).
309. **Left out of Poly_Triangulation**: the `OSD_FileSystem` argument of the deferred-loading methods (the methods
     are kept, without it), `DumpJson`, the `size_t` overloads of the node arrays. `MapNormalArray` returns a copy:
     a .NET float array can't alias an array of vectors. The cached range is a `StrongBox<Bnd_Box>` behind a `Lock`
     (OCCT: an atomic pointer and a shared mutex). Poly_MeshPurpose is a `[Flags]` enum over `uint`.
310. **`NCollection_IndexedMap.Substitute`** was added (Poly_MakeLoops replaces keys in place). Known difference left
     as is: OCCT's `NCollection_Array1::Resize(lower, upper)` throws for an empty range, the port's doesn't, so
     `Poly_Triangulation.ResizeTriangles(0)` succeeds in the port where OCCT throws.
