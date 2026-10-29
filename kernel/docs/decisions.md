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
## Phase 1: MathUtils and the new maths packages (29 October 2026)

OCCT 8's header-only packages MathUtils, MathLin, MathPoly, MathRoot, MathInteg, MathOpt and MathSys: C++
namespaces of function templates over duck-typed functors, returning result structs.

50. **A C++ namespace is a `public static partial class`** (`MathRoot.Brent(...)`), one C# file per header. Each
     file's `using namespace MathUtils;` is `using static Occt.MathUtils;`. Nested namespaces (`detail`,
     `Internal`) are nested `internal static partial class`es. Names that C++ code reaches through another
     namespace's `using` (`MathRoot::ScalarResult`) are written with their own class (`MathUtils.ScalarResult`).
     Where Standard and MathUtils both have a function (`IsEqual`, `Sign`), ported code qualifies `MathUtils.`.
51. **Functor templates are generic methods constrained on interfaces**, one interface per member set the C++
     body calls: `MathUtils.IScalarFunction` (`Value(double, out double)`), `IScalarFunctionWithDerivative`
     (`Values`; independent of `IScalarFunction`, since some solvers call only `Values`), `IVectorFunction`,
     `IVectorFunctionWithGradient`, `IVectorFunctionWithHessian`, `ISystemFunction` (`Value(x, F)`), and
     package-local ones (`MathSys.IFunctionSet[WithDerivatives]`, `MathSys.INewton2D/3D/4DFunction`,
     `MathInteg.ISetFunction`, MathRoot's adapter interfaces). A C++ `operator()` is a method `Invoke`. Why: a
     struct functor gets code specialised by the JIT with the call inlined, as a C++ template instantiation; a
     class still works through the interface. Rejected: delegates (an indirect call per evaluation, and
     `Func<>` can't have `out` parameters). Functors are passed by value, where C++ passes `Function&`: a
     functor whose evaluation changes its own state must be a class. The interfaces use `out` results, as in
     decision 32.
52. **The lambda wrappers (`MakeScalar`, `MakeVector`, `MakeSystem`, ...) hold delegates** of package-defined
     delegate types with C++'s signatures (`bool ScalarLambdaFunction(double, out double)`), so a C# lambda reads
     `MathUtils.MakeScalar((x, out y) => { y = x * x - 2.0; return true; })`. The library functors
     (`Polynomial`, `Rosenbrock`, `Sum<F, G>`, ...) are sealed classes; they copy the vectors and matrices they
     are given, as C++ does. Functor classes local to a C++ function template are private generic classes or
     structs of the package.
53. **Configurations and options are classes**: `Config`, `BoundedConfig : Config`, `NDimConfig : Config`,
     `PSOConfig : NDimConfig`, `GlobalConfig`, `IntegConfig`, `KronrodConfig`, `LMConfig`, `NewtonOptions`,
     `MinBracketOptions`, ..., with public fields as in C++ and a copy constructor. C++ passes derived
     configurations as `const Config&`; C# structs can't derive. A C++ copy is written `new(theConfig)`; a
     default argument `const Config& theConfig = Config()` is `Config? theConfig = null` and
     `theConfig ??= new Config();`. Rejected: structs with duplicated fields and conversion operators to the
     base (slicing), which don't scale to three levels.
54. **Result structs (`ScalarResult`, `PolyResult`, `VectorResult`, `IntegResult`, `NewtonResultN2`, ...) are
     structs with public fields**, as the C++ aggregates. `std::optional<double>` is `double?`,
     `std::optional<math_Vector>` a nullable reference, `size_t` is `int`. Their C++ default status is
     NotConverged, so they have an explicit parameterless constructor and ported code writes `new()`;
     `default(T)` has status OK. `explicit operator bool` is kept, and `operator true`/`false` make
     `if (aResult)` work as in C++. They have no `Equals`/`ToString`: they are transient solver outputs holding
     references. `MathUtils.Domain1D`/`Domain2D`, which are values, get the value-type members of section 3 of
     the rules but keep C++'s public fields. Results that hold containers (`MathRoot.MultipleResult`,
     `AllRootsResult`, `MathOpt.PSOSeedParticle`) share them when copied, where C++ copies them; ported helpers
     that change them take them `ref`.
55. **`std::array<T, N>` is `std_array2/3/4<T>`** (new file `Standard/std_array.cs`): `[InlineArray]` structs,
     copied as a whole, indexed from 0 with a range check, with `a[i][j]` working on nested arrays and
     `std_arrayN<T>.Of(...)` for brace initialisation. Not OCCT API, but OCCT 8 uses `std::array` widely.
     Larger one-off sizes are package-local inline arrays (`MathPoly.RootArray<T>` of 20, a 5-element row in
     MathSys).
56. **Templates over an integer (`NewtonResultN<N>`, `NewtonBoundsN<N>`)** are ported per instantiation OCCT
     uses: `NewtonResultN2`, `N3`, `N4`. The 2D/3D/4D functors take `ref std_arrayN<double> theF` and
     `ref std_arrayN<std_arrayN<double>> theJ`, so their bodies read as the C++ (`theJ[0][1] = ...`).
57. **Output parameters** are `out` when every path writes them, and `ref` when C++ may return without writing
     them, so the caller's value survives as in C++ (`MathUtils.CentralDifference`'s derivative,
     `BrentAlongCoordinate`'s `theFx`, MathSys's small solvers). Writes through `*aResult.Value` into an optional
     go through a temporary.
58. **`MathUtils.RandomGenerator` is a class**: its only purpose is its changing state, which a struct would lose
     to defensive copies. `MathOpt.PSOStats` is a class too (C++ fills it through an optional pointer), with
     `Assign`.
59. **`std::complex<double>` is `System.Numerics.Complex`** (same principal branches; division may differ in
     the last bits). `std::sort` is `Span.Sort` (not stable, like std::sort). `goto` is kept where OCCT uses it.
60. **Tests**: one xUnit class per GTest suite (a suite spread over two files is a `partial class`). C++
     `rand()` in the MathLin tests is the MSVC runtime's generator. The MathLin_EigenSearch random test uses a
     faithful `std::mt19937` but simple mappings in place of `std::uniform_*_distribution` (whose output differs
     between C++ libraries), so its random matrices differ from OCCT's; no check depends on the values.

## Phase 1: Poly (29 October 2026)

61. **Poly_ArrayOfNodes and Poly_ArrayOfUVNodes keep OCCT's run-time precision** through a port of
     `NCollection_AliasedArray` that holds a .NET array of the element type (`gp_Pnt[]` or
     `NCollection_Vec3<float>[]`, `gp_Pnt2d[]` or `NCollection_Vec2<float>[]`, chosen by the stride) plus a byte
     offset, and reads it through typed references (`Unsafe.As`, `MemoryMarshal`). Rejected: a `byte[]` buffer
     (MapNodeArray couldn't share it as a `gp_Pnt` array) and two typed arrays in one class (OCCT's stride-based
     API wouldn't carry over). The "wrapping" constructors take an array and a start index and share it, as OCCT's
     share memory; `Move` leaves the moved-from array referring to the storage, as in OCCT.
62. **Bulk reads for Bento** (additions on Poly_Triangulation): `NodeCoordinates()`/`NodeCoordinatesF()` (x, y, z
     per node, without copying, in the stored precision; the other precision throws `Standard_TypeMismatch`),
     `CopyNodeCoordinatesTo(Span<double>)`/`(Span<float>)` (either precision), `TriangleIndices()` (three 1-based
     indices per triangle), `TriangleSpan()`, `UVNodeCoordinates()`/`UVNodeCoordinatesF()`, `NormalComponents()`.
     The spans are valid until the arrays are resized. Poly_Triangle is exactly three `int`s so the triangle
     array casts to indices.
63. **Poly_Triangulation and the polygon classes keep public mutators**: they are mutable mesh containers filled
     by the mesher (decision 21). Their mutators are to become internal (or to throw once frozen) when
     triangulations are attached to shapes. Poly_Triangle follows the value-type rules: `Set` and `ChangeValue` are
     internal, the twins are the constructor (for `Set(n1, n2, n3)`) and `WithNode`.
64. **Poly_CoherentNode, Poly_CoherentTriangle and Poly_CoherentLink are classes**, not structs, although OCCT uses
     them by value: the coherent triangulation stores pointers to its triangles and links (neighbours, per-node ring
     lists) and compares addresses, so they need identity. C++ assignments of them (`theTriangle =
     Poly_CoherentTriangle(...)`) are an internal `Assign`. Poly_CoherentNode holds a gp_XYZ instead of deriving from
     it. Allocator arguments are dropped. The triangulation's iterators, like OCCT's, don't visit elements appended
     during the iteration.
65. **Poly_Connect, Poly_MakeLoops and Poly_MergeNodesTool are classes** (algorithms; OCCT's first two are used by
     value). Poly_MakeLoops' helpers are abstract classes; `OnAddLink`, called only by OCCT's debug builds, is
     never called. `Poly_MakeLoops.Link` is a struct with public fields as in OCCT, public `Reverse`/`Nullify`, and
     OCCT's unordered `operator==` as `Equals`/`==` (the equality its maps use).
66. **Poly_MergeNodesTool's node map is ported with OCCT's buckets**: NCollection_BaseMap's bucket counts
     (`NCollection_Primes`, added), resizing and rehash order, the DJB2 hash of the float position with MSVC's
     signed `char`, and the cell hash. When several merged nodes could match a new one, the first found depends on
     the bucket chains, so this keeps OCCT's choice. The element map, whose order doesn't matter, is an
     `NCollection_Map`.
67. **`NCollection_PackedMapOfInteger`** (OCCT's `NCollection_PackedMap<int>`, alias `TColStd_PackedMapOfInteger`)
     is ported with its buckets and iteration order, because Poly_MakeLoops takes its next start link from that
     order. 32-bit instantiation and set operations only (no boolean operations between maps).
68. **Poly's streams**: `Write`/`Dump` take a `TextWriter` and format numbers as a default C++ stream does (`%g`
     with six digits: .NET's "G6" with a lowercase exponent; `inf`, `nan`), `std::setw` as padding, so files are
     OCCT's byte for byte. `Read*` take a `TextReader` and follow C++ extraction: a failed read gives 0 and fails
     every later read. `Poly::PolygonProperties`, a template on the container, has overloads for
     NCollection_Array1 and NCollection_Sequence (the instantiations OCCT uses). `Poly::Intersect` and
     `IntersectTriLine` take their outputs by `ref` (unchanged on a miss).
69. **Left out of Poly_Triangulation**: the `OSD_FileSystem` argument of the deferred-loading methods (the methods
     are kept, without it), `DumpJson`, the `size_t` overloads of the node arrays. `MapNormalArray` returns a copy:
     a .NET float array can't alias an array of vectors. The cached range is a `StrongBox<Bnd_Box>` behind a `Lock`
     (OCCT: an atomic pointer and a shared mutex). Poly_MeshPurpose is a `[Flags]` enum over `uint`.
70. **`NCollection_IndexedMap.Substitute`** was added (Poly_MakeLoops replaces keys in place). Known difference left
     as is: OCCT's `NCollection_Array1::Resize(lower, upper)` throws for an empty range, the port's doesn't, so
     `Poly_Triangulation.ResizeTriangles(0)` succeeds in the port where OCCT throws.
71. **`NCollection_Array1.Resize` follows OCCT exactly**: resizing to bounds rejects an empty range
    (`Standard_RangeError`), resizing to a size allows zero. The first version allowed an empty range for both; the
    Poly port found the difference (OCCT's `Poly_Triangulation::ResizeTriangles(0)` throws).
## Phase 1: PLib, BSplCLib, CSLib (28-29 October 2026)

72. **Array starts become spans by use, not by C++ constness.** A `double&`/`double*`/`const double&` that is the
     start of an array is a `ReadOnlySpan<double>` when the function only reads through it (OCCT often omits
     `const`) and a `Span<double>` when it writes; a scalar `double&`/`int&` is `out` (`ref` when read first).
     Optional array pointers (`const NCollection_Array1<double>*`) are nullable NCollection arrays, and
     `PLib::NoWeights()`, `NoWeights2()`, `BSplCLib::NoWeights()`, `NoMults()` return null. Callers write
     `A.AsSpan()` / `A.AsSpan(i)` for `A(A.Lower())` / `A(i)`, and `MemoryMarshal.Cast<gp_Pnt, double>(...)` where
     OCCT casts an array of points to `double*`. Spans may alias, as OCCT's pointers do (`PLib::RationalDerivatives`
     is documented to accept the same array twice).
73. **`ExtrapMode` of the flat and gp `BSplCLib::Eval` is a `ReadOnlySpan<int>` of two modes**, not `ref int`:
     OCCT's `int&` is read as a two-element array (start and end). A C++ caller's `int mode[2]` becomes
     `stackalloc int[2]` or a collection expression.
74. **`NCollection_LocalArray` and C stack arrays become `stackalloc`** up to OCCT's own buffer size (128 doubles
     by default, or the template's N), and a heap array above it, never inside a loop. Stack memory is zeroed in
     .NET where OCCT's is not; nothing ported reads it before writing, except the OCCT bug in `BSplCLib::Eval`
     (occt-issues.md), where the port's zeros give the apparently intended result.
75. **BSplCLib_CurveComputation.pxx is ported once, generically**: methods
     `<Point, Vector, Traits> where Traits : struct, IBSplCLib_CurveTraits<Point, Vector>`, the interface's static
     abstract members carrying OCCT's traits operations and the calls the template resolves by overload (PLib's
     SetPoles/GetPoles, SetCoord, ...), implemented by `BSplCLib_CurveTraits3d` and `BSplCLib_CurveTraits2d`. The
     JIT specialises value-type generics, so this costs what the C++ template does. BSplCLib_1 (2D) and
     BSplCLib_3 (3D) keep OCCT's thin public wrappers. Single points written through `(double*)&aPoint` are
     computed in a stack buffer and built from it.
76. **PLib's polynomial evaluation specialises dimensions 1 to 4**, where OCCT's templates specialise 1 to 15:
     a type parameter with a static abstract constant lets the JIT keep the accumulators in registers. Higher
     dimensions run the general loops. The arithmetic is OCCT's, operation for operation.
77. **Constant tables are `ReadOnlySpan<double>` properties over RVA data** (the Jacobi tables, generated from
     OCCT's .pxx with every literal copied as written; the Bezier flat knots; the unit weights). `FlatBezierKnots`
     returns a `ReadOnlySpan<double>`: a caller needing an array writes `NCollection_Array1<double>.From(...)`,
     which copies. `UnitWeights(n)` returns, as OCCT, a non-owning view of a shared array of ones up to 2049
     elements (shared: don't modify), and a new array above; `UnitWeightsData()` is a `ReadOnlySpan<double>`.
78. **Small heap allocations remain where OCCT puts a math_Matrix on a stack buffer**: the flat `BSplCLib::Eval`
     allocates its (exactly sized) B-spline basis matrix per call (184 bytes for a cubic), `BuildBSpMatrix` its
     27 × 27 buffer and `Interpolate` its band matrix, because `math_Matrix` owns a `double[]`. The B-spline and
     Bezier evaluators (D0 to DN, Bohm, de Boor), PLib's evaluators and BSplCLib_Cache's evaluation allocate
     nothing; the Bezier evaluators share read-only per-degree knot arrays for that.
79. **BSplCLib_Cache is a Standard_Transient class whose `BuildCache` is internal** (a mutator); `IsCacheValid`
     and the evaluators are public. Its coefficients live in an `NCollection_Array2<double>` allocated once in
     the constructor, where OCCT wraps a 128-double member buffer anew in each `BuildCache`.
     **BSplCLib_CacheParams is a sealed class** (non-copyable in OCCT): its constant members are readonly fields,
     and the span state is properties with internal setters.
80. **PLib_JacobiPolynomial and PLib_HermitJacobi are sealed, immutable classes**: in V8_0_1 they are plain
     classes with only const members, used by value, and a struct's `default` would be invalid. Their
     `D0123` has an internal span overload so that PLib_HermitJacobi keeps OCCT's stack buffers.
81. **`CSLib::Normal`'s `theNormal` is `ref gp_Dir`**: OCCT leaves it unchanged when the status says no normal
     was computed. CSLib_Class2d is a sealed class (movable, not copyable in OCCT); its template constructor
     over the container type is one private `init` taking the bounds and an accessor.
82. **`out` parameters are always assigned**: `BSplCLib::PrepareInsertKnots` sets NbPoles and NbKnots to 0 when
     it returns false, where OCCT leaves them unchanged. Parameters named `Epsilon` in BSplCLib.hxx and
     `Tolerance` in BSplCLib.cxx take the .cxx name where `Epsilon` would hide `Standard.Epsilon()`.
83. **BSplCLib_EvaluatorFunction is an abstract class** with
     `Evaluate(int, ReadOnlySpan<double> theStartEnd, double, out double theResult, out int theErrorCode)`;
     its `operator()` is left out.
84. **BSplCLib is checked against native OCCT although decision 23 expected it only through Geom.** A throwaway
     harness (scratch, not committed: the oracle test project is not on this branch) drove build123d.NET's
     `Geom_BSplineCurve`, `Geom2d_BSplineCurve` and `Geom_BezierCurve` and the port's BSplCLib with the same
     inputs: D0, DN (orders 1-4), LocalD3, Resolution, InsertKnots, RemoveKnot, Unperiodize, MovePoint,
     IncreaseDegree (non-periodic, and 2D) and Bezier D0-D3 were bit-identical on 1,100 random curves (about
     200,000 values). MovePointAndTangent was identical except three degree-1 cases, where its 2 × 2 system is
     nearly singular. The harness should become oracle tests when that project reaches this code.
     BSplCLib_ConsistencyTests (OCCT.NET's own) checks the operations against each other on random curves.

## Phase 2: geometry (29 October 2026)

85. **Geometry and topology objects freeze when shared.** `Standard_Transient` has a one-way frozen flag
    (`Freeze()` internal, `IsFrozen()` public, `CheckMutable()` as the first line of every ported mutator of
    Geom, Geom2d, attached Poly objects and topology). Mutators stay internal, so callers can't change these
    objects at all; freezing protects shared objects from the port's own algorithms. Rejected: freezing every object
    when its constructor finishes (OCCT's algorithms routinely build an object and then modify it, so every such
    sequence would have to be rewritten) and no freezing (the shared-geometry mutations build123d.NET hit would
    stay silent). The cost is one flag per object and one check per mutation.
86. **A second assembly, OcctNet.ModelingData**, holds TKG2d, TKG3d, TKGeomBase and (phase 3) TKBRep, whose packages
    depend on each other in cycles and must therefore share an assembly. Test projects see every port assembly's
    internals (set once in src/Directory.Build.props).

## Phase 2: the abstract geometry layer (29 October 2026)

The bases of Geom, Geom2d and the adaptors, TopAbs, and the evaluation-representation helpers of GeomEval and
Geom2dEval, ported first so that the concrete classes can be ported in parallel (docs/phase2-plan.md).

501. **Abstract and virtual mutators are `protected internal`; the others are `internal`.** The abstract ones are
     `Transform`, `Reverse`, `UReverse`, `VReverse`, `SetDirection`, `Cross` and `CrossCross`. Callers outside
     OCCT.NET still can't change an object they are given, because `protected` access only works through their
     own derived type. They can still derive their own curves and surfaces, as OCCT allows (an override is
     `protected override` there, and `protected internal override` in an assembly that sees ModelingData's
     internals). Every implementation starts with `CheckMutable()`, and so do the non-virtual mutators of the bases
     (`Mirror`, `Rotate`, `Scale`, `Translate`, `SetAxis`, ...).
     Rejected: `internal abstract`, which would forbid subclasses outside OCCT.NET altogether.
502. **OCCT's return types are kept.** `Copy`, `Mirrored`, `Transformed`, `Reversed`, `Crossed`, ... return what
     OCCT returns (`Geom_Geometry`, `Geom_Curve`, `Geom_Vector`), not covariant types, so ported code reads as the
     C++ does (`(Geom_Line)L.Copy()` for `DownCast`). The `With` twins return the class that declares them
     (`Geom_Conic.WithAxis` returns a `Geom_Conic`); the concrete classes add their own. Revisit if callers find
     the casts heavy: covariant overrides of `Copy` would not break ported code.
503. **Geom_Transformation and Geom2d_Transformation are freezable like geometry.** They get internal mutators with
     `CheckMutable`, `With` twins, static factories (as gp_Trsf), and an added `PreMultiplied`. OCCT's in-place
     compound operators (`*=` of Geom2d_Transformation, and `+=`, `-=`, `*=`, `/=` of Geom2d_VectorWithMagnitude)
     are left out. The binary operators are kept.
504. **Evaluation results.** `Geom_Curve.ResD1/ResD2/ResD3` and `Geom_Surface.ResD1/...` (and the Geom2d ones) are
     nested structs with public fields, like decision 54's result structs. `D0` to `D3` take `out` parameters.
     `Bounds` takes `out`, `TransformParameters` `ref`, `Geom_Transformation.Transforms` `ref`.
505. **Adaptors' `GetType()` hides `object.GetType()`** (`public new virtual GeomAbs_CurveType GetType()`), so
     OCCT's name stays. Code wanting the .NET type uses `DynamicType()`. A ported GTest named `GetType` is renamed
     `GetType_` (xUnit rejects a test method hiding an `object` method).
506. **Placeholders for concrete classes.** The bases need a few concrete classes as return types:
     Geom_BezierCurve, Geom_BSplineCurve, Geom_OffsetCurve, Geom_BezierSurface, Geom_BSplineSurface,
     Geom2d_BezierCurve and Geom2d_BSplineCurve. Each is an empty abstract class in the file its port will replace,
     without a `Port of:` line, so port-status doesn't count it. Rejected: porting them here (14,000 lines, and
     BSplSLib isn't merged) and `object` return types.
507. **GeomEval_RepCurveDesc, GeomEval_RepSurfaceDesc and Geom2dEval_RepCurveDesc are in the abstract layer**,
     with RepUtils. Geom and Geom2d call them, and they need only the bases, so the Geom ⇄ GeomEval cycle
     disappears.
     - The C++ namespaces are static classes with nested types.
     - `std::size_t MaxDerivOrder` is an `int`, and `std::optional<Domain1d>` is `Domain1d?`.
     - The map and domain structs have OCCT's default values through explicit parameterless constructors.
     - RepUtils is internal; its `TryEval*` functions zero their `out` result when they return false.
     - The descriptors are mutable classes with public fields, as in OCCT. A descriptor attached to a frozen curve
       could therefore still be changed through a reference kept by whoever set it. The Geom porters keep
       `SetEvalRepresentation` internal, clone descriptors in `Copy` (as OCCT does), and don't return stored
       descriptors to callers.
508. **Adaptors are working objects** (decision 21). Their setters (`Load`) are public, and the base adaptors are
     concrete classes whose virtuals throw `Standard_NotImplemented`, as in OCCT. Adaptor3d_Curve/Surface's `D0`
     to `DN` are non-virtual wrappers over `EvalD*`, as in OCCT 8. In Adaptor2d_Curve2d they stay virtual, and
     `EvalD*` call them.
509. **`Geom_UndefinedDerivative_Raise_if` and `Geom_UndefinedValue_Raise_if`** (and the Geom2d ones) are not
     ported: OCCT defines them but never uses them. The exception classes are ported.
510. **The phase 2 plan** (docs/phase2-plan.md) sets the order of everything else. There are 21 units of 8,000 to
     15,000 lines in six waves, with the classes that close dependency cycles assigned explicitly: Geom_OffsetSurface
     and Adaptor3d_HSurfaceTool go with GeomAdaptor; GeomConvert_SurfToAnaSurf and Approx's SameParameter,
     CurveOnSurface and CurvlinFunc go with GeomLib; the elementary Extrema and ProjLib classes come early. The
     generated tables of AppCont and AdvApp2Var (148,000 lines) are produced by a generator script, as the PLib
     Jacobi tables were (decision 77).

## Phase 2: unit 1D, Eval geometries (29 October 2026)

GeomEval (SineWave and CircularHelix curves, TBezier and AHTBezier curves and surfaces, Ellipsoid, Hyperboloid,
HypParaboloid, Paraboloid and CircularHelicoid surfaces) and Geom2dEval (SineWave, Archimedean and logarithmic
spirals, CircleInvolute, TBezier and AHTBezier curves).

801. **Poles and weights are returned as `PackedList<T>`**, 0-based: `Poles()` and `Weights()` of the TBezier and
     AHTBezier curves, and of the surfaces row by row (the pole of U index i and V index j, from 0, is at
     `i * NbVPoles() + j`). The classes keep OCCT's `NCollection_Array1/Array2` fields, copied from the
     constructor's arguments with their bounds, so the evaluation code is OCCT's. Each call copies; nothing in
     OCCT calls these accessors in a loop. A non-rational TBezier returns no weights, as OCCT's empty array.
802. **The classes are `sealed`**: OCCT marks every one of their overrides `final`.
803. **Unsupported operations keep OCCT's behaviour.** `Reverse`, `UReverse`, `VReverse` and `Transform` start with
     `CheckMutable()` and then throw `Standard_NotImplemented`, as do `ReversedParameter`, `UIso` and `VIso`;
     so `Reversed`, `Transformed`, `Mirrored` and the other copying forms throw too.
804. **Setters of the elementary surfaces** (`SetSemiAxisA/B/C`, `SetR1`, `SetR2`, `SetMode`, `SetFocal`, `SetPitch`)
     are internal with `CheckMutable()`, and each has a public `With` twin returning the concrete type
     (`GeomEval_EllipsoidSurface.WithSemiAxisA`). `Coefficients` returns its ten values as `out double`, as the gp
     quadrics do (decision 14). The nested `GeomEval_HyperboloidSurface.SheetMode` enum stays nested.
805. **Templates over the derivative order** (`evalCurveBasis<theMaxOrder>`, `evalMixedAHT<theMaxOrder,
     theIsRational>`, `evalTrigAxisDerivs<theMaxOrder>`, `evalPolynomialSurface*<theMaxOrder>`, ...) are private
     static methods taking the order (and the rational flag) as parameters; `if constexpr` becomes `if`, and the
     arithmetic is unchanged. Their result structs are private nested structs. `NCollection_LocalArray<T, 16>` is a
     16-element `stackalloc` or, above that, a heap array (decision 74); a null weights pointer is a null array or
     an empty span. Where C# forbids a local hiding one of an enclosing scope, the inner one is renamed (`aS` →
     `aSr` in the rational branches of GeomEval_TBezierSurface; `aPoly*` in the polynomial branches of the AHTBezier
     curves). OCCT's private helpers that nothing calls (`evalBasis`, `evalBasisDeriv`, `evalTrigBasis`, ...) are
     ported anyway, for side-by-side review.
806. **The 3D TBezier and AHTBezier curves were ported from the 2D ports** by substituting the types, then
     aligned with the 3D C++ where it differs (the 3D AHTBezier indexes poles through `Value(Lower() + k)` and has no
     `evalBasis` helpers). The arithmetic of the two versions is the same.
807. **Tests.** The GTests are ported one xUnit class per suite (decision 60). GTests that compare with Geom classes
     not yet ported (Geom_Circle, Geom_Ellipse, Geom2d_Circle, Geom2d_Ellipse, Geom_SphericalSurface) compare with
     the gp value and ElCLib/ElSLib, which those classes evaluate through. Every class also gets a
     `Derivatives_MatchFiniteDifferences` test (tests/OcctNet.ModelingData.Tests/EvalTestUtils.cs): each
     derivative up to order 5 to 7 (curves) or 4 to 5 (surfaces) against the central difference of the one below,
     and EvalD1/D2/D3 against EvalD0 and EvalDN, for non-rational and rational poles and every combination of the
     AHT basis parts. There is no native oracle: build123d.NET doesn't bind these classes.
