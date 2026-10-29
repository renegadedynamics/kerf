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
     nearly singular. The harness should become oracle tests when that project reaches this code (done:
     decision 91, which also explains the degree-1 differences).
     BSplCLib_ConsistencyTests (OCCT.NET's own) checks the operations against each other on random curves.

## Phase 1: BSplSLib and Convert (29 October 2026)

85. **BSplSLib follows BSplCLib's conventions** (decisions 72-83): optional weights and multiplicities are
     nullable NCollection arrays (`BSplSLib.NoWeights()` is null), outputs are `out`, `Displ` is `in`, array starts
     are spans by use (`RationalDerivative(..., ReadOnlySpan<double> HDerivatives, Span<double> RDerivatives, ...)`).
     `(double*)&Poles(i, j)` is a private helper slicing `MemoryMarshal.Cast<gp_Pnt, double>(Poles.AsSpan())` at
     the element's linear offset; fixed C arrays (`local_poles_array[3][3][3]`) are flat stack spans indexed
     `i * 9 + j * 3 + k`; results written through `(double*)&aVec` are computed in spans and built with
     `new gp_Vec(...)`. `BSplSLib.UnitWeights` returns, as OCCT, a non-owning view over `BSplCLib.UnitWeights` up to
     2,049 weights (shared: don't modify) and a new array above. The 2D-array `CachePoles(i, j).SetX/SetY/SetZ`
     writes are a `new gp_Pnt` from the same products.
86. **BSplSLib's evaluation buffers are stack buffers sized by the degrees.** OCCT's `BSplSLib_DataContainer` is a
     2,852-double stack struct sized for degree 25. The port keeps it as a private `ref struct` of four spans,
     stackalloc'd in each evaluator (D0-D3, DN, the homogeneous forms, BuildCache) at 4(UDegree+1)(VDegree+1) poles,
     2·max(UDegree, VDegree) knots twice and 48 derivatives, after the degree check. It is a C stack array, so
     decision 74 allows stack use up to OCCT's size (21.6 KB at degree 25); sizing by the degrees zeroes only what is
     used (68 doubles for a bicubic), and evaluation allocates nothing. Rejected: a [ThreadStatic] buffer (hidden
     state, not reentrant). NCollection_LocalArray buffers (Iso, RationalDerivative, the cache evaluators,
     BSplSLib_Cache) follow decision 74: stackalloc up to 128 doubles, heap above.
87. **BSplSLib_Cache is BSplCLib_Cache's twin** (decision 79): a Standard_Transient class embedding two
     `BSplCLib_CacheParams`, an `NCollection_Array2<double>` of coefficients allocated once in the constructor (OCCT:
     an `NCollection_HArray2`), an internal `BuildCache` (a mutator) and public `IsCacheValid`, D0-D2 and
     D0Local-D2Local. OCCT's `const int&`/`const double&` parameters are by value. Its evaluation allocates nothing
     unless the smaller degree exceeds 9 (13 non-rational), where the derivatives' work array leaves
     NCollection_LocalArray's stack buffer in OCCT too.
88. **Convert's batch accessors return copies.** `Poles()`, `Weights()`, `Knots()`, `Multiplicities()`,
     `UKnots()`, ... return a new NCollection array where OCCT returns a const reference to a field, so a
     converter's result can't be changed through them. Every OCCT consumer (GeomConvert's BSplineCurveBuilder and
     BSplineSurfaceBuilder, Geom2dConvert, GeomConvert_CompBezierSurfacesToBSplineSurface, AdvApprox, AppDef,
     GeomFill) copies them into a new curve or surface anyway, and ported Geom code keeps passing NCollection arrays
     to the Geom constructors. Rejected: PackedList (the Geom constructors take NCollection arrays, so every caller
     would convert back) and returning the field (callers could change the converter, and two calls would share
     storage). The cost is one allocation per accessor call. The per-index accessors OCCT deprecates (`Pole(i)`,
     `Weight(i)`, ...) stay, without `[Obsolete]`, documented as deprecated.
89. **Convert classes are classes, built in their constructors**, with OCCT's hierarchy; base classes have
     protected constructors and protected fields with OCCT's names. The array outputs of the protected
     `BuildCosAndSin` (and of `GridPolynomialToPoles.BuildArray`), which OCCT reassigns through references, are
     `out`; the handle forms take `out NCollection_HArray1<...>` (`ref ...?` for the deprecated
     `CompPolynomialToPoles.Poles/Knots/Multiplicities(handle&)`, which OCCT leaves unchanged when not done).
     `Convert_ElementarySurfaceToBSplineSurface::Finalize` is `FinalizeArrays`, since `Finalize()` is .NET's
     finaliser. `Convert_CosAndSinEvalFunction` is a delegate with a `Span<double>` result;
     `BuildPolynomialCosAndSin` (not exported by OCCT) is internal. The builders
     (`Convert_CompBezierCurves(2d)ToBSplineCurve(2d)`: `AddCurve`, `Perform`) keep public mutators (decision 21);
     `AddCurve` copies its argument as C++ appends a copy.
90. **Convert_CompBezierCurvesToBSplineCurveBase is generic over a public traits struct**
     (`<PointType, VecType, Traits>`, `Convert_CompBezierCurvesTraits3d/2d` implementing
     `IConvert_CompBezierCurvesTraits`), as BSplCLib_CurveComputation (decision 75). The traits are public because a
     public class can't derive from a generic base over an internal type. `if constexpr (std::is_same_v<PointType,
     gp_Pnt>)` is `Traits.Is3d`, a constant the JIT folds. `NCollection_Array2.ResizeWithTrim` was added (OCCT API,
     used by Convert's FinalizeArrays): 2D-preserving copy, no reallocation when the dimensions don't change, empty
     range rejected. Known difference left as is: the port's `NCollection_Array2.Resize(..., copy: true)` keeps the
     2D layout, where OCCT V8_0_1's copies elements in linear order (different when the column count changes).
91. **BSplCLib, BSplSLib (with BSplSLib_Cache) and Convert are oracle-tested** (`tests/OcctNet.Oracle.Tests/
     {BSplCLib,BSplSLib,Convert}`), beyond what decision 23 expected. Native entry points, each mirrored exactly in
     the test: Geom_BSplineCurve/Geom2d_BSplineCurve/Geom_BezierCurve for BSplCLib (from decision 84's harness);
     Geom_BSplineSurface and Geom_BezierSurface for BSplSLib (OCCT 8 evaluates them through BSplSLib directly, not
     the cache); GeomAdaptor_Surface for BSplSLib_Cache (its evaluators rebuild one cache when IsCacheValid fails,
     and the test drives the port's cache with the same parameter sequence); GeomConvert/Geom2dConvert
     `CurveToBSplineCurve` and `GeomConvert::SurfaceToBSplineSurface` for Convert, redoing GeomConvert's
     post-processing in the test (2D poles lifted by `gp_Trsf.Transformation(Position, XOY)`, `SetPeriodic` for
     whole conics, the mirror of a left-handed `gp_Ax22d`). Every compared value is bit-identical, about 5.3 million
     in all, sin/cos/sqrt in Convert included. Left out: long RationalC1 arcs (GeomConvert joins two halves with
     the unported GeomConvert_CompCurveToBSplineCurve), periodic IncreaseDegree (broken in OCCT), inputs OCCT's
     release build doesn't check (a singular MovePointAndTangent system, MovePoint windows away from the point,
     repeated knots in InsertKnots with Add false; occt-issues.md). Decision 84's degree-1 MovePointAndTangent
     differences came from the harness, which built the deltas with `EvalDN` where Geom uses `Geom_Curve::D1`; the
     two differ in the last bit and the nearly singular system magnified it. BSplCLib_Cache, BSplSLib.Iso,
     CacheD0-D2 and FunctionMultiply have no native entry point in the bindings and are covered by unit tests only.
     The long RationalC1 arcs are compared since 2508, and the test calls the ported GeomConvert.
## Phase 2: geometry (29 October 2026)

92. **Geometry and topology objects freeze when shared.** `Standard_Transient` has a one-way frozen flag
    (`Freeze()` internal, `IsFrozen()` public, `CheckMutable()` as the first line of every ported mutator of
    Geom, Geom2d, attached Poly objects and topology). Mutators stay internal, so callers can't change these
    objects at all; freezing protects shared objects from the port's own algorithms. Rejected: freezing every object
    when its constructor finishes (OCCT's algorithms routinely build an object and then modify it, so every such
    sequence would have to be rewritten) and no freezing (the shared-geometry mutations build123d.NET hit would
    stay silent). The cost is one flag per object and one check per mutation.
93. **A second assembly, OcctNet.ModelingData**, holds TKG2d, TKG3d, TKGeomBase and (phase 3) TKBRep, whose packages
    depend on each other in cycles and must therefore share an assembly. Test projects see every port assembly's
    internals (set once in src/Directory.Build.props).

## Phase 2: the abstract geometry layer (29 October 2026)

The bases of Geom, Geom2d and the adaptors, TopAbs, and the evaluation-representation helpers of GeomEval and
Geom2dEval, ported first so that the concrete classes can be ported in parallel (docs/phase2-plan.md).

94. **Abstract and virtual mutators are `protected internal`; the others are `internal`.** The abstract ones are
     `Transform`, `Reverse`, `UReverse`, `VReverse`, `SetDirection`, `Cross` and `CrossCross`. Callers outside
     OCCT.NET still can't change an object they are given, because `protected` access only works through their
     own derived type. They can still derive their own curves and surfaces, as OCCT allows (an override is
     `protected override` there, and `protected internal override` in an assembly that sees ModelingData's
     internals). Every implementation starts with `CheckMutable()`, and so do the non-virtual mutators of the bases
     (`Mirror`, `Rotate`, `Scale`, `Translate`, `SetAxis`, ...).
     Rejected: `internal abstract`, which would forbid subclasses outside OCCT.NET altogether.
95. **OCCT's return types are kept.** `Copy`, `Mirrored`, `Transformed`, `Reversed`, `Crossed`, ... return what
     OCCT returns (`Geom_Geometry`, `Geom_Curve`, `Geom_Vector`), not covariant types, so ported code reads as the
     C++ does (`(Geom_Line)L.Copy()` for `DownCast`). The `With` twins return the class that declares them
     (`Geom_Conic.WithAxis` returns a `Geom_Conic`); the concrete classes add their own. Revisit if callers find
     the casts heavy: covariant overrides of `Copy` would not break ported code.
96. **Geom_Transformation and Geom2d_Transformation are freezable like geometry.** They get internal mutators with
     `CheckMutable`, `With` twins, static factories (as gp_Trsf), and an added `PreMultiplied`. OCCT's in-place
     compound operators (`*=` of Geom2d_Transformation, and `+=`, `-=`, `*=`, `/=` of Geom2d_VectorWithMagnitude)
     are left out. The binary operators are kept.
97. **Evaluation results.** `Geom_Curve.ResD1/ResD2/ResD3` and `Geom_Surface.ResD1/...` (and the Geom2d ones) are
     nested structs with public fields, like decision 54's result structs. `D0` to `D3` take `out` parameters.
     `Bounds` takes `out`, `TransformParameters` `ref`, `Geom_Transformation.Transforms` `ref`.
98. **Adaptors' `GetType()` hides `object.GetType()`** (`public new virtual GeomAbs_CurveType GetType()`), so
     OCCT's name stays. Code wanting the .NET type uses `DynamicType()`. A ported GTest named `GetType` is renamed
     `GetType_` (xUnit rejects a test method hiding an `object` method).
99. **Placeholders for concrete classes.** The bases need a few concrete classes as return types:
     Geom_BezierCurve, Geom_BSplineCurve, Geom_OffsetCurve, Geom_BezierSurface, Geom_BSplineSurface,
     Geom2d_BezierCurve and Geom2d_BSplineCurve. Each is an empty abstract class in the file its port will replace,
     without a `Port of:` line, so port-status doesn't count it. Rejected: porting them here (14,000 lines, and
     BSplSLib isn't merged) and `object` return types.
100. **GeomEval_RepCurveDesc, GeomEval_RepSurfaceDesc and Geom2dEval_RepCurveDesc are in the abstract layer**,
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
101. **Adaptors are working objects** (decision 21). Their setters (`Load`) are public, and the base adaptors are
     concrete classes whose virtuals throw `Standard_NotImplemented`, as in OCCT. Adaptor3d_Curve/Surface's `D0`
     to `DN` are non-virtual wrappers over `EvalD*`, as in OCCT 8. In Adaptor2d_Curve2d they stay virtual, and
     `EvalD*` call them.
102. **`Geom_UndefinedDerivative_Raise_if` and `Geom_UndefinedValue_Raise_if`** (and the Geom2d ones) are not
     ported: OCCT defines them but never uses them. The exception classes are ported.
103. **The phase 2 plan** (docs/phase2-plan.md) sets the order of everything else. There are 21 units of 8,000 to
     15,000 lines in six waves, with the classes that close dependency cycles assigned explicitly: Geom_OffsetSurface
     and Adaptor3d_HSurfaceTool go with GeomAdaptor; GeomConvert_SurfToAnaSurf and Approx's SameParameter,
     CurveOnSurface and CurvlinFunc go with GeomLib; the elementary Extrema and ProjLib classes come early. The
     generated tables of AppCont and AdvApp2Var (148,000 lines) are produced by a generator script, as the PLib
     Jacobi tables were (decision 77).

## Phase 2: unit 1D, Eval geometries (29 October 2026)

GeomEval (SineWave and CircularHelix curves, TBezier and AHTBezier curves and surfaces, Ellipsoid, Hyperboloid,
HypParaboloid, Paraboloid and CircularHelicoid surfaces) and Geom2dEval (SineWave, Archimedean and logarithmic
spirals, CircleInvolute, TBezier and AHTBezier curves).

104. **Poles and weights are returned as `PackedList<T>`**, 0-based: `Poles()` and `Weights()` of the TBezier and
     AHTBezier curves, and of the surfaces row by row (the pole of U index i and V index j, from 0, is at
     `i * NbVPoles() + j`). The classes keep OCCT's `NCollection_Array1/Array2` fields, copied from the
     constructor's arguments with their bounds, so the evaluation code is OCCT's. Each call copies; nothing in
     OCCT calls these accessors in a loop. A non-rational TBezier returns no weights, as OCCT's empty array.
105. **The classes are `sealed`**: OCCT marks every one of their overrides `final`.
106. **Unsupported operations keep OCCT's behaviour.** `Reverse`, `UReverse`, `VReverse` and `Transform` start with
     `CheckMutable()` and then throw `Standard_NotImplemented`, as do `ReversedParameter`, `UIso` and `VIso`;
     so `Reversed`, `Transformed`, `Mirrored` and the other copying forms throw too.
107. **Setters of the elementary surfaces** (`SetSemiAxisA/B/C`, `SetR1`, `SetR2`, `SetMode`, `SetFocal`, `SetPitch`)
     are internal with `CheckMutable()`, and each has a public `With` twin returning the concrete type
     (`GeomEval_EllipsoidSurface.WithSemiAxisA`). `Coefficients` returns its ten values as `out double`, as the gp
     quadrics do (decision 14). The nested `GeomEval_HyperboloidSurface.SheetMode` enum stays nested.
108. **Templates over the derivative order** (`evalCurveBasis<theMaxOrder>`, `evalMixedAHT<theMaxOrder,
     theIsRational>`, `evalTrigAxisDerivs<theMaxOrder>`, `evalPolynomialSurface*<theMaxOrder>`, ...) are private
     static methods taking the order (and the rational flag) as parameters; `if constexpr` becomes `if`, and the
     arithmetic is unchanged. Their result structs are private nested structs. `NCollection_LocalArray<T, 16>` is a
     16-element `stackalloc` or, above that, a heap array (decision 74); a null weights pointer is a null array or
     an empty span. Where C# forbids a local hiding one of an enclosing scope, the inner one is renamed (`aS` →
     `aSr` in the rational branches of GeomEval_TBezierSurface; `aPoly*` in the polynomial branches of the AHTBezier
     curves). OCCT's private helpers that nothing calls (`evalBasis`, `evalBasisDeriv`, `evalTrigBasis`, ...) are
     ported anyway, for side-by-side review.
109. **The 3D TBezier and AHTBezier curves were ported from the 2D ports** by substituting the types, then
     aligned with the 3D C++ where it differs (the 3D AHTBezier indexes poles through `Value(Lower() + k)` and has no
     `evalBasis` helpers). The arithmetic of the two versions is the same.
110. **Tests.** The GTests are ported one xUnit class per suite (decision 60). GTests that compare with Geom classes
     not yet ported (Geom_Circle, Geom_Ellipse, Geom2d_Circle, Geom2d_Ellipse, Geom_SphericalSurface) compare with
     the gp value and ElCLib/ElSLib, which those classes evaluate through. Every class also gets a
     `Derivatives_MatchFiniteDifferences` test (tests/OcctNet.ModelingData.Tests/EvalTestUtils.cs): each
     derivative up to order 5 to 7 (curves) or 4 to 5 (surfaces) against the central difference of the one below,
     and EvalD1/D2/D3 against EvalD0 and EvalDN, for non-rational and rational poles and every combination of the
     AHT basis parts. There is no native oracle: build123d.NET doesn't bind these classes.
## Phase 2: unit 1F, AdvApp2Var core (29 October 2026)

AdvApp2Var's numeric core is f2c output (Fortran translated to C): AdvApp2Var_SysBase, AdvApp2Var_MathBase and the
constant tables of AdvApp2Var_Data*.pxx. ApproxF2var, also f2c and Geom-free, belongs to unit 2D with the classes
that call it (phase2-plan.md).

111. **f2c array pointers are `F2cPtr<T>` and `F2cConstPtr<T>`** (AdvApp2Var_F2c.cs): ref structs holding a span and
     a bias, with `+`, `-`, `++`, `--` and a `ref` indexer. The f2c idioms then compile as written (`--point1;`,
     `crvold -= crvold_offset;`, `crvold[ii + nd * crvold_dim1]`), and `&crvold[k]` becomes `crvold + k`, so every
     subscript is the C one, operation for operation. The bias may point before the span, as a shifted C pointer
     does; only the elements accessed must exist, and an access outside the array throws where C would read or
     write outside it. `F2cConstPtr` is an input array (or a constant table, which is read-only data); `F2cPtr`
     converts to it implicitly, and arrays and spans convert to both. `F2c.Ref(ref x)` is `&x` passed as an array.
     Local Fortran arrays are `new F2cPtr<double>(stackalloc double[N])` (zeroed, where C leaves them
     uninitialised); Fortran DATA tables (`static double xmaxj[57] = {...}`) are `ReadOnlySpan<double>` locals over
     constant data. Rejected: plain spans with every subscript rewritten (`a[i - 1 - off]`), which would make
     10,000 lines unreviewable against the C, and unsafe pointers. AdvApp2Var_SysBase and AdvApp2Var_MathBase are
     internal (only AdvApp2Var uses them, in OCCT too), so decision 72's spans in signatures give way to these.
112. **AdvApp2Var's tables are generated by `tools/gen-advapp2var-data.cs`** (decision 77): one `ReadOnlySpan<double>`
     over constant data per .pxx, every literal copied as written except that C's `1.` and `1.e5` become `1.0` and
     `1.0e5`, which C# requires. OCCT returns each table as a struct cast over one C array; `Getmlgdrtl()` and the
     others return a ref struct of `F2cConstPtr` views of that one array, so call sites read as OCCT's
     (`AdvApp2Var_Data.Getmlgdrtl().rootab[k]`) and an index past a field reads the next field as in C. The small
     common blocks (`mdnombr`, `maovpar`, `maovpch`) are structs returned `ref readonly`; `maovpch`'s char arrays
     are strings. Nothing in OCCT writes these blocks.
113. **Statics and threads.** OCCT 8 already makes AdvApp2Var_SysBase's allocation table an instance member, and
     each routine that allocates makes its own instance: the port does the same (a sealed class, one per call), so
     allocation is reentrant. The other statics:
     - Fortran DATA tables and `static char nomprg[]` names: constant data, no state.
     - `mmprcsn_` (precisions) and `mblank__` (units): static, written only by the type initialiser (OCCT's
       start-up code) and read-only after. `mmwprcs_` and `macinit_`, which write them, are internal and called
       only there.
     - `mmcmher_`, the cache of Hermite coefficients that `mmherm0_` fills and `mmherm1_` reads, is
       `[ThreadStatic]` (created lazily). OCCT's is shared and racy (occt-issues.md). A per-thread cache computes
       exactly what OCCT computes on one thread, so results are unchanged, and approximations can run in parallel,
       as Bento runs them. Rejected: a lock (it would serialise every basis evaluation) and no cache (a change of
       cost for nothing). `Mmpobas_IsSafeAcrossThreads` exercises it.
114. **Dynamic allocation (`mcrrqst_`, `macrar8_`, ...) replaces the caller's array.** The Fortran returns an offset
     IOFSET from the caller's array T to a new zone, so that `T(IOFSET + I)` addresses it; a managed array can't be
     addressed from another one. The port's `mcrrqst_` takes T by `ref`, points it at a new managed block laid out
     as OCCT's (8-byte start flag, user zone rounded up to 8 bytes, 8-byte end flag) and sets IOFSET (1 for doubles,
     2 for ints) so that `T[IOFSET + I]` is the zone: callers keep OCCT's indexing. `mcrdelt_` finds the allocation
     by the block's identity, lets the collector free it and sets IOFSET to `int.MaxValue` (OCCT: the largest
     `intptr_t`); `intptr_t` offsets are `int`. The unit must be the element size (an `F2cPtr<double>` zone of
     4-byte units can't exist; OCCT never asks for one), else IERCOD = 2. Allocation failure throws, as OCCT's
     `Standard::Allocate` does. The byte-count routines (`mcrfill_`, `miraz_`, `mvriraz_`) keep sizes in bytes,
     with overloads for double and int arrays. The message, trace-back and logical-name routines are ported as
     OCCT has them: stubs that do nothing.
115. **Scalars, and the f2c converter.** f2c passes every scalar by pointer. A scalar parameter that the function
     (or a function it calls) never writes is passed by value; one it writes is `ref`, never `out`: f2c code
     returns on error paths without writing its outputs, and callers pass variables they read afterwards.
     MathBase was converted by `tools/f2c-to-cs.py`, which infers each pointer parameter's kind (array or scalar)
     and mode (read or written, through calls too, to a fixpoint), rewrites the bodies (dereferences, call
     arguments, declarations, comma expressions, labels, literals) and keeps the comments; its output was then
     fixed by hand (the NCollection_Array1 locals over C arrays, which become plain NCollection_Array1, the
     `mmgaus1_` callback, and a parameter named `is`, which is `@is`). Locals are zero-initialised for C#'s
     definite assignment; unreferenced labels are commented out. `int (*bfunx)(...)` is the delegate
     `AdvApp2Var_MathBase_bfunx`. Unit 2D can convert ApproxF2var the same way, giving the script MathBase's and
     SysBase's signatures (it prints the ones it infers as JSON).
116. **MathBase is checked against OCCT's C++ compiled natively** (a harness not committed, as in decision 84): the
     two .cxx files built with MSVC against stub Standard and NCollection_Array1 headers, with exported thunks
     called through P/Invoke beside the port on the same random inputs. 3,000 random cases of each of 35 public
     functions (mdsptpt_, mmapcmp_, mmaperx_, mmarcin_, mmbulld_, mmcdriv_, mmcglc1_, mmcvctx_, mmcvinv_, mmdrc11_,
     mmdrvck_, mmeps1_, mmfmca8_, mmfmca9_ also in place, mmfmcar_, mmfmcb5_, mmfmtb1_, mmhjcan_, mminltt_,
     mmjacan_, mmjaccv_, mmpobas_, mmpocrb_, mmmpocur_, mmmrslwd_, mmrtptt_, mmsrre2_, mmtrpjj_, mmunivt_, mmvncol_,
     mvsheld_, mzsnorm_, msc_, pow__di, mmveps3_; the private functions through them) gave bit-identical outputs and
     error codes, except where OCCT copies uninitialised memory (mmfmtb1_, occt-issues.md). The harness caught the
     converter's one inference miss (`++(*nbrlgn)` taken for a read-only scalar), since fixed. Not compared:
     mmresol_ (OCCT crashes below 101 dimensions and nothing calls it), mmposui_ (index structures; tested on a
     known profile) and mmgaus1_ (a callback). The committed tests check known values, some from that native run,
     and the generated tables against the .pxx.
## Phase 2: unit 1G, elementary analytics (29 October 2026)

IntAna2d, gce and the elementary Extrema classes (ExtElC, ExtElC2d, ExtPElC, ExtPElC2d, ExtPElS, ExtElSS, POnCurv,
POnCurv2d, POnSurf).

117. **What becomes a struct and what a class.**
      - Values used by copy are structs under the value-type rules: IntAna2d_IntPoint, IntAna2d_Conic,
        Extrema_POnCurv, Extrema_POnCurv2d, Extrema_POnSurf. Their setters (`SetValue`, `SetValues`, `SetParameters`)
        are internal; since each rebuilds the whole value, the constructors are their public twins. IntAna2d_IntPoint's
        `SetValue` is virtual in OCCT, but nothing derives from it. `new IntAna2d_IntPoint()` and
        `new Extrema_POnSurf()` have OCCT's non-zero defaults (RealLast, Precision::Infinite); `default` is zeros.
      - The algorithms are classes with OCCT's public `Perform` (working objects, decision 21):
        IntAna2d_AnaIntersection and the Extrema_Ext* classes. Extrema_ExtElC and Extrema_ExtElC2d have no Perform
        in OCCT and are computed by their constructors only. Their fixed C++ arrays are .NET arrays allocated with the
        object (`myPoint[6][2]` is `Extrema_POnCurv[6, 2]`); the small local arrays of the C++ functions are
        `stackalloc` or span collection expressions. Results are returned by value; `Points(N, P1&, P2&)` and
        `Extrema_POnSurf::Parameter` take `out`.
      - gce: `gce_Root` is a class with OCCT's protected `TheError`, and each gce_Make* a sealed class deriving from
        it, computed by its constructor. The transformation makers (MakeMirror, MakeRotation, MakeScale,
        MakeTranslation and their 2D twins) don't derive from it, as in OCCT. `Value()` and `Operator()` are kept;
        OCCT's implicit `operator gp_Lin()` (and the others) is an explicit conversion, as decision 28 did for
        TopLoc_Location. `Value()` throws StdFail_NotDone on a failed construction, where OCCT's release build
        returns the default object (decision 9).
118. **gce statuses that OCCT leaves unset.** `gce_MakeCirc(P1, P2, P3)` returns without setting `TheError` when
      the three points coincide or the computed center falls on P1 (both build a circle of radius 0), and
      `gce_MakeCirc2d(P1, P2, P3)` when the points are distinct but collinear (parallel bisectors). The status is
      then uninitialised memory in C++. The port's starts as gce_Done, a .NET field's default: that is what the
      zero-radius circles are meant to be, but for gce_MakeCirc2d it would report success with a default circle, so
      the port sets gce_IntersectionError there (marked `// OCCT`). Revisit if the oracle shows what native callers
      actually get.
119. **`gce_MakeCirc(Center, Plane, Radius)` reads the inner construction's circle directly** instead of calling
      `C.Value()`, which with a negative radius throws in OCCT's debug builds and in the port (decision 9) but
      returns the default circle in OCCT's release build. The construction then reports gce_NegativeRadius, as
      native OCCT does.
120. **File-private helpers.** IntAna2d_Outils.hxx/.cxx is an internal static class `IntAna2d_Outils`, imported
      with `using static`, holding the free functions and `MyDirectPolynomialRoots` as a nested class.
      Extrema_ExtElC.cxx's static `RefineDir` and its local class `ExtremaExtElC_TrigonometricRoots` (a copy of
      IntAna_IntQuadQuad's, which unit 2F will port separately) are private members of Extrema_ExtElC.
      `IntAna2d_AnaIntersection.empt` is set but never read in OCCT; it is kept, with CS0414 suppressed on that
      field only.
121. **Oracle coverage through native callers.** build123d.NET binds gce_MakeLin, IntAna2d_AnaIntersection,
      IntAna2d_IntPoint and Extrema_POnCurv, which are compared directly. Three more classes are compared through
      native classes that call them with known arguments: Extrema_ExtPElC through Extrema_ExtPC on lines, circles
      and ellipses (which passes analytic curves to ExtPElC with Precision::Confusion() and keeps its results in
      order), Extrema_ExtPElC2d through Geom2dAPI_ProjectPointOnCurve, and gce_MakeCirc(P1, P2, P3) with
      Extrema_ExtElC between two lines through GC_MakeArcOfCircle. Random directions handed to native OCCT are
      first `Oracle.Settled` (normalised until the port's normalisation leaves them unchanged), and the port's and
      native axis systems are built from the same inputs, so that differences come from the code under test.
## Phase 2: unit 1A, Geom curves (29 October 2026)

Geom_Line, Circle, Ellipse, Hyperbola, Parabola, TrimmedCurve, BezierCurve, BSplineCurve (with
Geom_BSplineCurve_1.cxx), OffsetCurve and Geom_OffsetCurveUtils.pxx.

122. **Arrays are returned to callers as `PackedList` copies; ported code reads the stored arrays through
     internal `Internal*` accessors.** Where OCCT returns a const reference to a curve's array, the public method
     of the same name returns a `PackedList<T>` copy, indexed from 0: `Poles()`, `Knots()`, `Multiplicities()`,
     `KnotSequence()`, `WeightsArray()` (always NbPoles values), and `Weights()`, which is null for a non-rational
     curve as OCCT's pointer is. The internal `InternalPoles()`, `InternalKnots()`, `InternalMultiplicities()`,
     `InternalKnotSequence()`, `InternalWeights()` (null when non-rational) and `InternalWeightsArray()` return the
     stored `NCollection_Array1`s (shared: don't modify), so a ported `BSplCLib::D0(U, C->Poles(), C->Weights(), ...)`
     becomes `BSplCLib.D0(U, C.InternalPoles(), C.InternalWeights(), ...)` without copying. OCCT's deprecated
     overloads that fill a caller's array (`Poles(NCollection_Array1<gp_Pnt>&)`, ...) stay public. Why: the
     NCollection arrays are mutable, so handing out the stored ones would let callers change a frozen curve, and
     copying on every internal call would cost the adaptors' per-span caches. Rejected: public NCollection copies
     (mutable values for callers, against section 9 of the rules) and `ReadOnlySpan`s (invalidated by later
     modifications, and not storable). The same names are proposed for Geom2d (1B) and the surfaces (1C:
     `InternalUKnots()`, ...).
123. **Evaluation representations.** `HasEvalRepresentation()` is public. `EvalRepresentation()` (the stored
     descriptor, which GeomAdaptor shares as OCCT does), `SetEvalRepresentation` and `ClearEvalRepresentation`
     are internal, the setters with `CheckMutable()`. Callers use `WithEvalRepresentation(desc)`, which validates
     the descriptor against the curve and stores a deep copy (`GeomEval_RepUtils.CloneCurveDesc`) in a copy of the
     curve, so the caller's descriptor can't reach the curve later; null removes it. `Copy` clones it and
     modifications drop it, as in OCCT.
124. **Twins.** `Set*` → `With*` (`WithPole`, `WithWeight`, `WithKnot`, `WithKnots`, `WithOrigin`, `WithTrim`,
     `WithBasisCurve`, `WithDirection`, `WithOffsetValue`, `WithRadius`, ...); `SetPeriodic`/`SetNotPeriodic` →
     `WithPeriodic`/`WithNotPeriodic`. Other mutators → past participles: `Increased`, `PoleInsertedAfter`,
     `PoleInsertedBefore`, `PoleRemoved`, `Segmented`, `DegreeIncreased`, `MultiplicityIncreased`,
     `MultiplicityIncremented`, `KnotInserted`, `KnotsInserted`, `PointMoved` and `PointAndTangentMoved` (with
     OCCT's output parameters), and `KnotRemoved`, which returns null where OCCT's `RemoveKnot` returns false.
     Each returns the concrete class. `InsertKnots` keeps OCCT's default `Add = false` (BSplCLib's is true).
125. **`final` members are `sealed override`** (including `Copy`); the classes themselves are not sealed, as in
     OCCT.
126. **`Resolution(Tolerance3D, out UTolerance)` of Bezier and B-spline curves is public and not a mutator**,
     although non-const in OCCT: it only caches 1 / max derivative, so it works on frozen curves. The "cached"
     value is computed into a local and published before its flag (`Volatile.Write`, decision 140), as in Geom2d;
     concurrent first calls compute the same value.
127. **Composite curves share their basis as OCCT does, frozen, and copy it on write** (as Geom2d, decision 132;
     changed in decision 140). `Geom_TrimmedCurve.BasisCurve()` and `Geom_OffsetCurve.BasisCurve()` return the
     stored basis curve (the constructors copy the curve they are given, as in OCCT), after freezing it, and the
     same object each time. The owners' own mutators (`Reverse`, `Transform`, and `SetTrim` through `Reverse`)
     copy a frozen basis curve before changing it, so a basis curve that was handed out never changes, and ported
     code that tries to change it through `BasisCurve()` throws (rules, section 8). The values computed are
     OCCT's: a copy is equal to its original. A trimmed curve built from a trimmed curve copies the inner basis
     without freezing it. First version: `BasisCurve()` returned the basis unfrozen, so ported code could change
     it.
128. **Geom_OffsetCurveUtils' templates are ported per instantiation** (rules, section 9): `AdjustDerivative` over
     `Geom_Curve` (Geom_OffsetCurve, GeomGridEval) and over `Adaptor3d_Curve` (GeomAdaptor_Curve's basis adaptor),
     with identical bodies, and `EvaluateD0` to `EvaluateDN` over `Adaptor3d_Curve` only, the one instantiation
     OCCT makes. Values read and written are `ref`, outputs `out`; the array of pointers to D2, D3 and D4 is a
     switch. Rejected: an interface implemented by both bases, which would change the abstract layer for one
     helper.
129. **Default arguments that are function calls** (`Precision::PConfusion()` in `Segment`, `gp::Resolution()` in
     `CalculateD0` to `D3`) are constants of the same value (`1e-7 * 0.01`, `RealSmallValue`); a unit test checks
     the first.
130. **Oracle tests of the curves** compare evaluation (D0 to D3, DN up to Degree + 1), properties, reversal,
     transformation and every Bezier and B-spline modification on random curves, mutating the native curves and
     going through the port's twins. Three choices: (a) transformed curves are evaluated at 1e-6, because native
     and port rotations differ in the last bits (sin and cos) and high-degree rational derivatives amplify that by
     up to 1e7, while what `Transform` computes (poles, axes) is compared at the default 1e-12; (b) `IsClosed` is
     not compared for infinite curves (an offset of a parabola is evaluated at 1e100, where the 1e-17 difference
     between a port axis and its native copy, which `Oracle.ToNative(gp_Ax2)` rebuilds, is multiplied by 1e200);
     (c) `MovePointAndTangent` is not tried on degree 1 curves, whose 2 × 2 system is singular: native OCCT
     inverts it without a check, the port throws (decisions 9 and 84).
     (a) and (b) withdrawn by 1705: with inputs built identically, transformed curves and the closure of infinite
     curves compare bit-identical.
## Phase 2: unit 1B, Geom2d curves (29 October 2026)

Geom2d_Line, Circle, Ellipse, Hyperbola, Parabola, TrimmedCurve, BezierCurve, BSplineCurve, OffsetCurve with
Geom2d_OffsetCurveUtils, and Adaptor2d_OffsetCurve.

131. **Pole, weight and knot arrays: `PackedList<T>` copies for callers, the arrays themselves for ported code.**
     Where OCCT returns a const reference to an internal array (`Poles()`, `Knots()`, `Multiplicities()`,
     `KnotSequence()`, `WeightsArray()`, and the `Weights()` pointer), the public method returns a new
     `PackedList<T>`. `Weights()` returns null for a non-rational curve, as OCCT returns `BSplCLib::NoWeights()`.
     A copy can't change when the curve is modified later, and it is what decision 11 asks for numeric arrays.
     Ported code gets the arrays themselves, without copying, through internal accessors named as in Geom
     (`InternalPoles()`, `InternalWeights()` (null if not rational), `InternalWeightsArray()`, `InternalKnots()`,
     `InternalMultiplicities()`, `InternalKnotSequence()`; first named with a `Ref` suffix, decision 140), to pass
     to BSplCLib as OCCT does. It must not modify them (the unit weights and the Bezier knot arrays are shared). A
     ported `C->Poles()` passed to BSplCLib doesn't compile until it is changed to `InternalPoles()`. OCCT's
     deprecated copy-out overloads (`Poles(NCollection_Array1&)`, ...)
     are public. The indexed accessors (`Pole(i)`, `Knot(i)`, `Weight(i)`) don't allocate.
     Rejected: returning the internal arrays publicly (callers could write into them) and a `PackedList` view of
     them (it would change when the curve is modified). Caching the copies in the curve is easy if profiling
     ever shows a need.
132. **Basis curves shared by Geom2d_TrimmedCurve and Geom2d_OffsetCurve are frozen, and copied on write.**
     OCCT's `BasisCurve()` returns the stored handle, and Geom2d_OffsetCurve stores the caller's curve without
     copying it (Geom2d_TrimmedCurve copies it in its constructor). The port keeps that sharing: `BasisCurve()`
     freezes the stored curve and returns it, and `Geom2d_OffsetCurve.SetBasisCurve` freezes the caller's curve
     that it keeps. The owners' own mutators (`Reverse`, `Transform`) copy a frozen basis curve before changing
     it, so a basis curve that was handed out or shared never changes. `BasisCurve()` returns the same object
     each time, as ported code may expect. Rejected: copying in `BasisCurve()` (an allocation per call, and a
     different object each time) and not freezing (the hazard of section 8 of the rules).
133. **Names of the added twins.** `Set` mutators get `With` twins (`WithRadius`, `WithPole`, `WithKnot`,
     `WithKnots`, `WithTrim`, `WithBasisCurve`, `WithOffsetValue`; `SetPeriodic`, `SetNotPeriodic` and
     `SetOrigin` give `WithPeriodic`, `WithNotPeriodic` and `WithOrigin`). The others get past participles:
     `Increased` (Bezier), `DegreeIncreased`, `MultiplicityIncreased`, `MultiplicityIncremented`, `KnotInserted`,
     `KnotsInserted` (Add defaults to false, as in Geom2d_BSplineCurve), `KnotRemoved` (null when the knot can't
     be removed within the tolerance), `PoleInsertedAfter`, `PoleInsertedBefore`, `PoleRemoved`, `Segmented`,
     `PointMoved` and `PointAndTangentMoved` (with OCCT's `out` results). `SetEvalRepresentation` gives
     `WithEvalRepresentation`, which stores a clone of the descriptor so that the caller's descriptor can't
     change the curve later, and `ClearEvalRepresentation` gives `WithoutEvalRepresentation`.
     `EvalRepresentation()` is internal (decision 507).
134. **`final` overrides are `sealed override`; the classes are not sealed**, as in OCCT, where the classes can
     be derived from but those methods can't be redefined.
135. **`Resolution` of the Bezier and B-spline curves stays public**: OCCT makes it non-const only because it
     fills a cache. The port writes the cached value before its flag (`Volatile.Write`), so concurrent callers
     on a shared curve compute it at most a few times and never read a half-written value.
136. **Geom2d_BezierCurve's `init` never writes into the shared unit weights.** It resizes and fills `myWeights`,
     which for a non-rational curve is a view of `BSplCLib::UnitWeights`, and a resize to the same size keeps the
     view. OCCT's callers always change the size, so this can't happen there; the port replaces a non-owning
     array first, in case a later port calls `init` otherwise.
137. **Geom2d_OffsetCurveUtils' templates are ported per instantiation**, as overloads for `Geom2d_Curve` and
     `Adaptor2d_Curve2d` (Geom2dAdaptor_Curve and Geom2dGridEval will use the second). In-out references are
     `ref`, outputs `out`; `AdjustDerivative`'s array of pointers to the derivatives is a switch over the `ref`
     parameters.
138. **`BSplCLib.InsertKnots` is always given `Add` explicitly** (Geom2d_BSplineCurve's default is false,
     BSplCLib's true): `InsertKnots` passes its own `Add`, `Segment` false, `IncreaseMultiplicity` and
     `IncrementMultiplicity` true. The tolerance parameter takes the .hxx name `ParametricTolerance`, not the
     .cxx name `Epsilon`, which would hide `Standard.Epsilon()` (as in decision 82).
139. **Oracle tests skip the inputs only the port rejects.** On random B-splines, `MovePointAndTangent` can meet
     a singular 2 × 2 system (mostly at degree 1): the port's `gp_Mat2d::Invert` throws, where native OCCT's
     release build divides by zero. Some edits of unclamped curves are invalid on both sides: native OCCT throws
     Standard_OutOfRange (its NCollection range checks are on), the port Standard_OutOfRange or an
     IndexOutOfRangeException. The edit chains stop at a curve that `IncreaseDegree` left inconsistent
     (occt-issues.md), and leave rotations out of composed transformations, whose last-bit sin/cos differences
     grow to about 1e-9 after several edits.
     Rotations are back in the edit chains since 1705 (bit-identical).

## Phase 2: curve consistency (29 October 2026)

Aligning the Geom (1A) and Geom2d (1B) curves where they made different choices for no OCCT reason.

140. **One set of conventions for both curve families.** (a) The internal array accessors are `InternalPoles()`,
      `InternalWeights()`, `InternalWeightsArray()`, `InternalKnots()`, `InternalMultiplicities()` and
      `InternalKnotSequence()` in both (Geom2d's `...Ref()` names renamed; decision 131), as for the surfaces
      (1C); the public `PackedList` accessors already had the same names. (b) Geom's trimmed and offset curves
      freeze their shared basis curve and copy it on write, as Geom2d's (decision 127, changed; 132). Every
      mutator path was checked against OCCT: only `Reverse` and `Transform` (and `SetTrim` through `Reverse`)
      change the basis, and a copy evaluates as the original, so results are OCCT's (oracle shares unchanged).
      (c) The non-mutating twins already had the same names in both families (decisions 124, 133);
      `WithoutEvalRepresentation` (Geom2d's twin of `ClearEvalRepresentation`) is added to Geom_BezierCurve,
      Geom_BSplineCurve and Geom_OffsetCurve. (d) Geom_OffsetCurveUtils' `CalculateD3` reproduces the same OCCT
      error as Geom2d's and is marked `// OCCT` the same way (occt-issues.md). (e) The evaluation
      representation rules differ between the families in OCCT (Geom2d_BSplineCurve never drops it,
      Geom_BSplineCurve always does, both Bezier curves only in `Transform` and `init`); the ports already
      matched OCCT, and tests now pin it in both. (f) `Resolution`'s cache is published the same way in both
      (decisions 126, 135); the default tolerance of `Segment` is the same named constant for
      `Precision::PConfusion()`; the Geom oracle tests use `Oracle`'s array converters instead of private
      copies, and Geom2dOracle builds native B-splines through `Oracle.ToNativeCurve`.
## Phase 2: unit 1E, approximation leaves (29 October 2026)

AdvApprox, AppParCurves (with its `.gxx` generics), FEmTool and GProp.

141. **AppParCurves_MultiPoint, MultiCurve and MultiBSpCurve are classes with OCCT's public API.** OCCT copies
      them by value, but their data are handles, so a C++ copy shares the points; a C# reference does the same. The
      setters stay public (decision 21): callers build MultiPoints as AppDef's input (AppDef_MultiPointConstraint
      derives from AppParCurves_MultiPoint), and the algorithms fill MultiCurves in place. Where C++ copies one
      (a by-value return, the slicing `(AppParCurves_MultiCurve)bspCurve`, a member assignment), the port writes
      the copy constructor `new(x)`, which shares the points, knots and multiplicities as C++ does and copies the
      rest (the B-spline degree). `MultiBSpCurve.Knots()` and `Multiplicities()` return copies (decision 88);
      `MultiCurve.Value(i)` returns the stored MultiPoint (OCCT returns a const reference; shared: don't modify).
      New MultiCurves fill their array with empty MultiPoints, as C++ default-constructs them. Dump takes a
      TextWriter and `ToString()` returns what it writes (decision 34). AppParCurves_ConstraintCouple is a value
      type (section 3 of the rules): internal setters, `WithIndex`/`WithConstraint` twins, default index -1.
142. **AppParCurves' `.gxx` files are generic classes over the MultiLine and its tool**:
      `AppParCurves_LeastSquare<MultiLine, ToolLine>`, `_ResolConstraint`, `_Function`, `_Gradient`,
      `_BSpFunction` and `_BSpGradient`, all `where ToolLine : struct, IAppParCurves_LineTool<MultiLine>`. The
      interface (new, `IAppParCurves_LineTool.cs`) has the static abstract members the templates call on
      `ToolLine`: FirstPoint, LastPoint, NbP3d, NbP2d, and three overloads each of Value, Tangency and Curvature
      (3d arrays, 2d arrays, both). As in decision 75, a struct tool makes the JIT specialise the code per
      instantiation, as OCCT's `_0.cxx` files do. The templates' other macro arguments (Squares, ResolCons,
      ParLeastSquare, ParFunction, Gradient_BFGS, BSpParLeastSquare, BSpParFunction, BSpGradient_BFGS) are, in
      every instantiation OCCT makes (AppDef, GeomInt, BRepApprox), the AppParCurves classes over the same
      MultiLine and tool, so the generic bodies name them directly. `AppParCurves_Gradient_BFGS` and
      `AppParCurves_BSpGradient_BFGS` have no `.gxx` (OCCT's generator wrote one identical copy per
      instantiation); they are generic classes ported from AppDef's copy. The public members are those of the
      instantiations' headers (AppDef_TheLeastSquares.hxx and the like, listed as ported files). This follows the
      phase 2 plan, which overrides section 9's "one port per instantiation" for these files.
      **How later units instantiate them** (2C AppDef/AppCont, 5A Approx, and later GeomInt and BRepApprox): write
      the MultiLine as a class and its tool as a struct implementing `IAppParCurves_LineTool<TheMultiLine>`
      (`tests/OcctNet.ModelingData.Tests/AppParCurves/AppParCurves_TestLine.cs` is a complete example). Tools
      with more members (WhatStatus, MakeMLBetween, MakeMLOneMorePoint, which Approx's ComputeLine and
      BSplComputeLine call) derive their own interface from this one. Where the public API needs OCCT's
      instantiation names (AppDef_TheLeastSquares, AppDef_MyBSplGradientOfBSplineCompute, ...), they can be sealed
      subclasses with forwarding constructors; inside the generic bodies the objects are of the AppParCurves types.
143. **AppParCurves' algorithms are working objects** (decision 21). The matrices and vectors they return
      (`FunctionMatrix`, `Poles`, `Distance`, `KIndex`, `NewParameters`, `ConstraintMatrix`, ...) are their own
      (decision 33; shared: don't modify), and so is `LeastSquare.BSplineValue()`, which OCCT returns by const
      reference. The by-value results (`BezierValue`, `Gradient.Value`, `BSpGradient.Value`,
      `BSpFunction.CurveValue`) are copies that share the poles, as OCCT's are. C++ `math_Vector`/`math_Matrix`
      assignments are `Initialized(...)` (decision 31). A function that fails assigns its `out` value 0
      (decision 32). Members OCCT writes but never reads (the functions' `Done`) or declares but never defines
      (`ResolConstraint::Error()`) are left out. The MultiLine is kept by reference where OCCT copies it into a
      member (neither changes it).
144. **AppParCurves' out-of-bounds reads throw.** Several of OCCT's index errors (occt-issues.md) read past an
      array in the release build, whose range checks the `.gxx` files also disable (`No_Standard_OutOfRange`). The
      port always checks (decision 9), so those paths throw Standard_OutOfRange where OCCT goes on with undefined
      values. The visible consequence: an interior passing or tangency constraint on a MultiLine with both 3d and
      2d points (AppParCurves_Function, BSpFunction, ResolConstraint) throws. Lines with only 3d or only 2d points,
      and every approximation without interior constraints, are unaffected.
145. **AdvApprox_EvaluatorFunction is an abstract class with by-value inputs.** Its one member is
      `Evaluate(int Dimension, ReadOnlySpan<double> StartEnd, double Parameter, int DerivativeRequest,
      Span<double> Result, out int ErrorCode)`. OCCT passes everything through pointers, but none of its 19
      implementations writes an input. `operator()` is left out, as for BSplCLib_EvaluatorFunction (decision 83).
      Implementations must always assign ErrorCode.
146. **AdvApprox_ApproxAFunction's batch accessors return copies** (`Poles1d()`, `Poles2d()`, `Poles()`,
      `Knots()`, `Multiplicities()`, `MaxError(D)`, `AverageError(D)`), as in decision 88; they return null where
      OCCT returns a null handle. The tolerance arguments are nullable `NCollection_HArray1<double>?`.
      AdvApprox_SimpleApprox is a working object whose accessors return its own arrays (shared: don't modify).
      `Dump` takes a TextWriter; `AdvApprox_Cutting.Value` has `out double cuttingvalue`.
147. **FEmTool keeps OCCT's mutable API** (decision 21). AppDef_Variational writes knots through
      `FEmTool_Curve.Knots()`, so it returns the curve's own array (shared). `FEmTool_ElementaryCriterion.Set(HArray2)`
      keeps the array it is given; `FEmTool_Assembly.AssemblyTable()` returns the table passed to the constructor;
      `FEmTool_SparseMatrix.ChangeValue` returns `ref double`. `OutM`/`OutS` take a TextWriter instead of writing
      to std::cout. `Length` and `ReduceDegree` use `out` parameters.
148. **FEmTool's linear criteria cache their reference integrals per constraint order, under a lock.** OCCT keeps
      one set in function-local statics and recomputes it whenever the order changes, which isn't thread-safe; the
      values are the same. The port always throws Standard_ConstructionError for a WorkDegree above 14, where OCCT
      checks only when it recomputes and otherwise reads past its 120-element table.
149. **GProp_GProps and its subclasses are classes with OCCT's public mutators.** `Add`, `AddPoint`, `SetLocation`
      and `Perform` stay public (decision 21): they are accumulators that BRepGProp and GeomGProp fill in place. The
      fields `g`, `loc`, `dim` and `inertia` are protected with OCCT's names; `GProp_GProps(GProp_GProps)` is the C++
      copy. **GProp_PrincipalProps is a struct** (section 3 of the rules): OCCT's class has only const members and
      is copied by value. Its parameterless constructor gives OCCT's undefined value (RealLast moments, radii and
      centre; the absolute axes); its full constructor, a private friend in OCCT, is internal; `Moments` and
      `RadiusOfGyration` have `out` parameters. **GProp_PEquation is a sealed, immutable class** computed in its
      constructor; its nested `enum class Type` stays nested and `GetType()` hides `object.GetType` (as in
      decision 98).
150. **GProp_CelGProps, GProp_SelGProps and GProp_VelGProps are ported with OCCT's errors** (occt-issues.md): no
      OCCT code uses them. Their tests check the masses and centres where OCCT is right and pin the reproduced
      errors elsewhere.
151. **GProp is oracle-tested through BRepGProp** (`tests/OcctNet.Oracle.Tests/GProp`), the only way to fill a
      native GProp_GProps. Native volume, surface and linear properties of 150 random primitives (box, cylinder,
      cone, sphere, torus, wedge) are loaded into the port's GProp_GProps (a test subclass setting the protected
      fields, the reference point at the centre of mass), and MatrixOfInertia, StaticMoments, MomentOfInertia,
      RadiusOfGyration, PrincipalProperties (moments, axes, radii, symmetry checks) and Add with random densities
      are compared: 20,700 values, all bit-identical. The oracle project now references OcctNet.ModelingData.
      AdvApprox, AppParCurves and FEmTool have no native entry point in the bindings (GeomConvert_ApproxCurve and
      GeomAPI_PointsToBSpline reach them indirectly, once their own units are ported); unit tests on known
      approximations cover them.

## Phase 2: unit 2F, quadric intersections (29 October 2026)

IntAna (Quadric, ResultType, Int3Pln, IntLinTorus, IntConicQuad, QuadQuadGeo, IntQuadQuad, Curve), Extrema_ExtElCS,
and the elementary ProjLib: ProjLib.cxx's `Project` overloads and `IsAnaSurf`, ProjLib_Projector, Plane, Cylinder,
Cone, Sphere and Torus.

152. **Algorithms and working objects are classes.** IntAna_Int3Pln, IntLinTorus, IntConicQuad, QuadQuadGeo,
      IntQuadQuad and Extrema_ExtElCS are algorithm classes with OCCT's public `Perform` (decision 117); results are
      returned by value and their fixed C++ arrays are .NET arrays allocated with the object. IntAna_Quadric is a
      sealed working class with public `SetQuadric` (it holds a list, and nothing copies it by value): `Coefficients`
      takes `out`, `NewCoefficients` `ref` (it transforms the values passed, not the quadric's own, as OCCT does), and
      `SpecialPoints()` returns a `PackedList<gp_Pnt>` copy, ported code reading the stored list through the internal
      `InternalSpecialPoints()` (decision 122's naming). As in OCCT, `SetQuadric` with a cone or a sphere appends to
      the special points without clearing them. Where C++ converts a gp quadric to an IntAna_Quadric implicitly
      (IntConicQuad's conic/plane overloads, Extrema_ExtElCS's line/sphere), the port writes `new IntAna_Quadric(...)`.
153. **IntAna_Curve is a value type.** OCCT copies it: IntQuadQuad's fixed array `TheCurve[12]`, `Curve(i)` returning
      a const reference, and IntPatch's lines keeping copies. The setters (`SetCylinderQuadValues`,
      `SetConeQuadValues`, `SetIsFirstOpen`, `SetIsLastOpen`, `SetDomain`) are internal, with public twins
      `WithCylinderQuadValues`, `WithConeQuadValues`, `WithIsFirstOpen`, `WithIsLastOpen` and `WithDomain`. `Value`
      and `D1u` are non-const in OCCT but change nothing, so they are `readonly`; `D1u` returns its point and vector as
      `out` (a zero vector when it returns false, where OCCT leaves the caller's unchanged). `FindParameter` appends to
      a caller's `NCollection_List<double>`, as in OCCT. `new IntAna_Curve()` is OCCT's empty curve (type
      GeomAbs_OtherSurface); `default` is all zeros (GeomAbs_Plane). `IntAna_IntQuadQuad.Curve(i)` returns a copy, and
      its array is filled with `new IntAna_Curve()`.
154. **IntAna_IntQuadQuad's file-local classes.** `TrigonometricRoots` is a private nested class: IntAna's own copy,
      which differs from Extrema_ExtElC's (decision 120): it fails on any inaccurate root instead of dropping it, and
      has no retry with zeroed coefficients. `MyTrigonometricFunction` is a private readonly struct (its unused
      `Derivative` kept). The template `AddSpecialPoints<gpSmth>` is one private method over `gp_Cylinder`, the only
      instantiation OCCT makes. `HasNextCurve`/`NextCurve` and the `Previous` pair take `out bool theOpposite`;
      `Parameters` is a no-op with `ref` arguments, as OCCT's only prints "not yet implemented"; the debug `DumpCurve`
      is left out.
155. **IntAna_QuadQuadGeo and IntAna_IntConicQuad call each other** (QuadQuadGeo refines plane/plane origins and
      intersects planes with cylinders and cones through IntConicQuad; IntConicQuad's circle/plane Perform uses
      QuadQuadGeo), and were ported together. The non-default constructors chain to the default one (`: this()`),
      which sets OCCT's member initialisers and calls `InitTolerances`, as each C++ constructor does. QuadQuadGeo's
      local class AxeOperator is a private nested class, and DirToAx2, EstimDist and RefineDir are private static
      methods; the function-local `static double EllipseLimit` and `HyperbolaLimit` are `const` locals; fixed local
      arrays are spans. QuadQuadGeo has no `IsParallel` (OCCT has none).
156. **ProjLib.** The static class is `public static partial class ProjLib`; `MakePCurveOfType` joins it with
      ProjLib_ProjectedCurve (unit 6A). `IsAnaSurf` takes an `Adaptor3d_Surface`. ProjLib_Projector and its
      subclasses are working classes (decision 21) with OCCT's public `Init`, `Project`, `SetType`, `SetPeriodic`,
      `SetBezier`, `SetBSpline`, `Done`, `UFrame`, `VFrame` and `SetInBounds`, and OCCT's protected fields. The
      `Project` overloads are virtual as in OCCT, and the subclasses' constructors call them, which dispatches to the
      subclass in C# as in C++. `GetType()` hides `object.GetType` (decision 98). `Bezier()` and `BSpline()` return the
      stored curves, shared as in OCCT (unit 6A, which fills them, decides whether to freeze them). A line normal to a
      plane makes `ProjLib_Plane` throw Standard_ConstructionError from gp_Dir2d, where OCCT's release build returns a
      line with a NaN direction (decision 9).
157. **Oracle coverage through native callers** (decision 121's approach):
      - IntAna_QuadQuadGeo through GeomAPI_IntSS for plane/plane, plane/cylinder, plane/sphere and sphere/sphere
        (IntPatch_ImpImpIntersection calls it with Tolang 1e-8 and the IntSS tolerance; the lines and conics GeomInt
        builds are compared; native AdjustToSeam turns plane/sphere circles, so their X axes aren't compared): 8,099
        of 8,136 values bit-identical, the rest within 1.7e-15.
      - IntAna_IntConicQuad (lines, circles, ellipses, parabolas and hyperbolas against planes, cylinders, cones and
        spheres) and IntAna_IntLinTorus through GeomAPI_IntCS, whose IntCurveSurface_HInter calls them with known
        tolerances and keeps each point's IntAna parameters (put in the period, bounded by the surface). Every point
        count matches. The conic/plane cases are 10,919 of 12,804 values bit-identical (within 1.1e-13), IntLinTorus
        2,815 of 3,468 (within 3.4e-14). Against quadrics the parameters are roots of math_DirectPolynomialRoots and
        differ by up to 6e-11 relative (32,384 of 52,410 bit-identical), so those tests compare at 1e-9. The
        line/quadric code is OCCT's line for line, so the differences come from the polynomial solver (cos, acos,
        cbrt) or from the axis systems rebuilt natively; math_DirectPolynomialRoots has no oracle of its own yet.
      - ProjLib_Plane through GeomAPI::To2d (ProjLib_ProjectedCurve on a plane): 7,010 of 9,300 values
        bit-identical, within 8.9e-16 (gp_Ax22d renormalises the directions).
      - No native path reaches IntAna_IntQuadQuad, IntAna_Int3Pln, Extrema_ExtElCS or the other ProjLib projectors
        directly (IntPatch, Extrema_ExtCS and ProjLib_ProjectedCurve on quadrics add their own processing). They have
        property tests: points on both surfaces, parameters mapped back through ElCLib and ElSLib.
## Phase 2: unit 1C, Geom surfaces

Geom_Plane, the cylindrical, conical, spherical and toroidal surfaces, Geom_RectangularTrimmedSurface,
Geom_SurfaceOfRevolution and Geom_SurfaceOfLinearExtrusion with Geom_RevolutionUtils and Geom_ExtrusionUtils
(1101-1109); Geom_BezierSurface and Geom_BSplineSurface (1110-1113).

158. **The surfaces follow unit 1A's conventions for the curves**: OCCT's `final` members are `sealed override`
      (`Copy` included) and the classes are not sealed (decision 125); every `Set` method is internal with
      `CheckMutable()` and has a public `With` twin returning the concrete class: `WithPln`; `WithCylinder`,
      `WithRadius`; `WithCone`, `WithRadius`, `WithSemiAngle`; `WithSphere`, `WithRadius`; `WithTorus`,
      `WithMajorRadius`, `WithMinorRadius`; `WithAxis`, `WithDirection`, `WithBasisCurve`, `WithLocation` (revolution);
      `WithDirection`, `WithBasisCurve` (extrusion); and `WithTrim` with OCCT's two public `SetTrim` signatures (the
      eight-argument `SetTrim` stays private, without the check, as a helper reached only from mutators and
      constructors). Geom_ElementarySurface's `WithAxis`, `WithLocation`, `WithPosition` and the bases' `UReversed`,
      `VReversed`, `Transformed`, ... cover the rest.
159. **Composite surfaces share their basis as OCCT does, frozen, and copy it on write** (as the curves, decision 127; changed in 1702). `Geom_RectangularTrimmedSurface.BasisSurface()` and `Geom_SweptSurface.BasisCurve()`
      freeze the stored object (which the constructors and `SetBasisCurve` copy from the caller's, as in OCCT) and
      return it, the same object each time. The owners' own mutators that change the basis copy a frozen basis first:
      the trimmed surface's `UReverse`, `VReverse` (both also reached through `SetTrim`) and `Transform`, the surface
      of revolution's `VReverse` and `Transform` (its `UReverse` only reverses the axis direction), the extrusion's
      `UReverse` and `Transform` (its `VReverse` only reverses the direction). A basis that was handed out never
      changes, and ported code that tries to change it through `BasisSurface()`/`BasisCurve()` throws (rules,
      section 8). A trimmed surface built from a trimmed surface copies the inner basis without freezing it. Every
      path was checked against OCCT: a copy evaluates as its original, so the values are OCCT's (oracle shares
      unchanged, all bit-identical). First version: the basis was returned unfrozen, and the owners' mutators changed
      it in place, so a caller holding it saw it change.
160. **Iso-curves that build unported curves throw until unit 1A is merged.** `UIso`/`VIso` of the elementary
      surfaces (Geom_Line, Geom_Circle, Geom_TrimmedCurve), `VIso` of Geom_SurfaceOfRevolution (Geom_Circle), `UIso` of
      Geom_SurfaceOfLinearExtrusion (Geom_Line) and the trimmed-direction iso-curves of Geom_RectangularTrimmedSurface
      (Geom_TrimmedCurve) throw `Standard_NotImplemented`, with OCCT's code in a `// TODO(1A)` comment (see also 1112).
      Revolution `UIso` and extrusion `VIso` (copies of the basis, rotated or translated) work. Unit tests named
      `*_NotImplementedUntil1A` flag them for the merge. Geom_RectangularTrimmedSurface's constructors turn an
      offset basis into an offset of a trimmed surface; Geom_OffsetSurface is unit 2A's, so that code is a
      `// TODO(2A)` comment (until then no surface can be an offset surface).
      Superseded by 1701: unit 1A is merged, and the iso-curves are OCCT's.
161. **Geom_RevolutionUtils and Geom_ExtrusionUtils are internal static classes.** Their `Calculate*` functions are
      ported as written (`in` inputs, `out` results). Their evaluator templates over the curve type are ported for
      `Adaptor3d_Curve` only, the one instantiation OCCT makes (GeomAdaptor_Surface; GeomGridEval uses the
      `Calculate*` functions), as decision 128.
162. **Small translations.** `std::remainder` (closure and periodicity of trimmed surfaces) is `Math.IEEERemainder`,
      which has the same definition (quotient rounded to even). `Geom_RectangularTrimmedSurface::Copy` returns a null
      handle when neither direction is trimmed; no constructor or `SetTrim` can produce that state, so the port
      throws `Standard_ProgramError` there instead of returning null from a non-nullable `Copy`. OCCT's
      `Geom_SurfaceOfRevolution::ReferencePlane` throws `Standard_NotImplemented`, and so does the port's.
163. **Evaluation representations of the swept surfaces** follow decision 123: `HasEvalRepresentation()` is public;
      `EvalRepresentation()`, `SetEvalRepresentation` and `ClearEvalRepresentation` are internal (the setters with
      `CheckMutable()`); callers use `WithEvalRepresentation(desc)`, which validates the descriptor and stores a
      deep copy in a copy of the surface. Every modification drops it, as in OCCT, and `Copy` clones it.
164. **Stand-in basis curves in the tests.** Until unit 1A's Geom_Line, Geom_Circle and Geom_BSplineCurve are merged,
      the unit tests revolve and extrude `TestLine` (the abstract layer's) and `TestCircle`, and the oracle tests
      `PortCircle` and `PortBSplineCurve` (non-periodic), which mirror OCCT 8's Geom_Circle and Geom_BSplineCurve
      (ElCLib's evaluators; BSplCLib's D0-D3 on the knots with multiplicities after LocateParameter, DN on the flat
      knots; BSplCLib's Reverse; poles transformed one by one). They don't mirror Geom_BSplineCurve's continuity or
      closure, which the oracle tests then don't compare. Replace them with the real classes after the merge.
      Superseded by 1703: the stand-ins are deleted.
165. **Oracle inputs are built identically on both sides.** `Oracle.ToNative(gp_Dir)` and `ToNative(gp_Ax3)` pass
      the port's normalised (and orthogonalised) values to native constructors that normalise them again, which can
      change the last bit of the input. The Geom oracle tests therefore build directions, axes, coordinate systems
      and transformations on both sides from the same raw coordinates through the same constructors
      (`tests/OcctNet.Oracle.Tests/Geom/GeomOracleInputs.cs`). With that, every value is bit-identical, rotations
      (sin, cos) included: the differences first seen after rotations and mirrors (up to 2.1e-13) were all input
      differences. docs/oracle-testing.md says so.
166. **The elementary, swept and trimmed surfaces are oracle-tested** (`tests/OcctNet.Oracle.Tests/Geom/
      Geom_SurfacesOracleTests.cs`): 300 random elementary surfaces (each kind, left-handed frames half the time,
      cones of zero reference radius, tori with minor radius above major), 300 surfaces of revolution and extrusion
      of circles and B-spline curves, and 600 trimmed surfaces of every basis kind (B-spline surfaces included),
      trimmed in one or both directions with both senses, trimmed again, and copied. Compared: D0-D3, DN up to order
      4, bounds, closure, periodicity and periods, continuity and IsCN, reversed parameters, U and V reversal, random
      transformations (rotation, translation, positive and negative scale, mirror) with TransformParameters and
      ParametricTransformation, and each class's queries (position, radii, Coefficients, Apex, Area, Volume, basis
      curve, iso-curves). 1.33 million values, all bit-identical.

167. **Geom_BSplineSurface and Geom_BezierSurface return their arrays as immutable values.** The one-dimensional
      arrays (`UKnots()`, `VKnots()`, `UMultiplicities()`, `VMultiplicities()`, `UKnotSequence()`, `VKnotSequence()`)
      are `PackedList<T>`s, as unit 1A's Geom_BSplineCurve returns its knots (index 0 is OCCT's index 1). The
      two-dimensional ones (`Poles()`, `Weights()`, `WeightsArray()`) are new copies of the `NCollection_Array2`,
      since PackedList has one dimension, and by decision 88's reasoning: ported code passes them back to the Geom
      constructors. `Weights()` is null unless the surface is rational in U or V (OCCT 8's pointer). OCCT's
      deprecated fill forms (`Poles(NCollection_Array2&)`, `UKnots(NCollection_Array1&)`, ...) stay public and
      copy into the caller's array. Ported code uses internal accessors returning the stored arrays without copy
      (`InternalPoles()`, `InternalWeights()` (null when non-rational), `InternalWeightsArray()`, `InternalUKnots()`,
      `InternalUMultiplicities()`, `InternalUKnotSequence()` and the V ones). Geom_BezierSurface's knots,
      multiplicities and flat knots are, as in OCCT, static arrays shared per degree (`Internal*`), copied into a
      PackedList for callers.
168. **Twins of the surface mutators.** Every `SetFoo` has `WithFoo` (`WithPole`, `WithPoleRow/Col`, `WithWeight`,
      `WithWeightRow/Col`, `WithUKnot(s)`, `WithVKnot(s)`, `WithUPeriodic`, `WithUNotPeriodic`, `WithUOrigin`,
      the V ones, `WithEvalRepresentation`), returning the concrete class. The other mutators take OCCT's past
      participle: `ExchangedUV`, `DegreeIncreased` (B-spline) and `Increased` (Bezier, OCCT's `Increase`),
      `UKnotsInserted`, `UKnotInserted`, `UKnotRemoved` (null when OCCT's `RemoveUKnot` returns false),
      `UMultiplicityIncreased`, `UMultiplicityIncremented`, `Segmented`, `CheckedAndSegmented`, `PointMoved` (with
      OCCT's out indices), `PoleColInsertedAfter/Before`, `PoleRowInsertedAfter/Before`, `PoleColRemoved`,
      `PoleRowRemoved`, and the V forms. `UReversed`, `VReversed` and `Transformed` come from the bases.
      `Resolution` caches its result, as OCCT's non-const method does; it is public and works on a frozen surface
      (the cache is not part of the value: computed into locals, then published before the volatile flag).
      OCCT's protected helpers stay protected (`segment` and Geom_BezierSurface's `init`, both with
      `CheckMutable`; `updateUKnots`/`updateVKnots`, which only recompute derived fields). The copy constructors
      are public, as in OCCT.
169. **The iso-curves of the B-spline and Bezier surfaces wait for unit 1A.** `UIso` and `VIso` (and the
      B-spline `UIso(U, CheckRational)`, `VIso(V, CheckRational)`) compute the iso-curve's poles with BSplSLib::Iso
      as OCCT does, then throw `Standard_NotImplemented`, with OCCT's Geom_BSplineCurve/Geom_BezierCurve
      construction in a `// TODO(1A)` comment. Geom_BSplineSurface's `IsUClosed`/`IsVClosed`, which OCCT computes
      as `UIso(U1)->IsEqual(UIso(U2), Precision::Confusion())`, can't wait: a private `IsoCurvesEqual` does what
      that does on the iso-curves' data (Geom_BSplineCurve's constructor checks: CheckCurveData, weights above
      gp::Resolution(), rationality; then IsEqual on poles and weights, the knots, multiplicities and degree being
      shared). The oracle tests find it identical to OCCT, throwing included. Replace both with OCCT's code once
      the curves are merged.
      Superseded by 1701: the iso-curves and IsUClosed/IsVClosed are OCCT's code.
170. **Geom_BSplineSurface and Geom_BezierSurface are oracle-tested** (`tests/OcctNet.Oracle.Tests/Geom`): random
      surfaces (B-spline: degrees 1 to 5, every periodicity, rational in U, V or both, some closed; Bezier: degrees 1
      to 8), every field and D0-D3, DN, local evaluation, LocateU/V, Resolution, PeriodicNormalization and the
      reversed parameters, before and after random edits (B-spline: 20 kinds, from knot insertion and removal to
      Segment, periodicity, origin, ExchangeUV, MovePoint, reverses and Transform; Bezier: 14). 8.37 million values,
      all bit-identical once the transformations are built identically on both sides (1108; the differences after
      rotations and mirrors first reported, up to 2.1e-13, came from the inputs).
      Left out, because OCCT's release build reads out of range there (occt-issues.md): MovePoint on periodic
      surfaces or with a window away from the point, RemoveUKnot/RemoveVKnot of the end knots of a periodic
      direction, CheckAndSegment over the whole range of a periodic direction, and periodic IncreaseDegree
      (decision 91).

## Phase 2: surfaces against the real curves (29 October 2026)

Unit 1C's surfaces finished against unit 1A's curves, and aligned with their conventions.

171. **The iso-curves are OCCT's.** Every `// TODO(1A)` of the surfaces is replaced by OCCT's code: `UIso`/`VIso`
      of the elementary surfaces build Geom_Line, Geom_Circle and (sphere `UIso`) a Geom_TrimmedCurve on
      [-PI/2, PI/2], which Geom_TrimmedCurve brings into [3PI/2, 5PI/2]; Geom_SurfaceOfRevolution's `VIso` the
      circle of the basis curve's point; Geom_SurfaceOfLinearExtrusion's `UIso` a line; Geom_RectangularTrimmedSurface
      a Geom_TrimmedCurve in a trimmed direction; Geom_BezierSurface and Geom_BSplineSurface (with the
      `CheckRational` overloads) Bezier and B-spline curves from BSplSLib::Iso's poles. Geom_BSplineSurface's
      `IsUClosed`/`IsVClosed` are OCCT's `UIso(U1)->IsEqual(UIso(U2), Precision::Confusion())`; the private
      `IsoCurvesEqual` (decision 169), which reproduced it on the iso-curves' data, is deleted (the oracle tests had
      found it identical, and still do). The `*_NotImplementedUntil1A` unit tests are OCCT's GTests again (Geom_Plane
      `UIso`, `VIso`; Geom_BSplineSurface `UIso`, `VIso`, `RationalSurface_UIso`; Geom_BezierSurface `UIso`, `VIso`,
      `RationalSurface_UIso`, `VIso_Rational`). Still open: the offset basis of Geom_RectangularTrimmedSurface's
      constructors (`// TODO(2A)`, Geom_OffsetSurface), and the GTests that need GeomAdaptor or Geom_OffsetSurface.
172. **Surfaces freeze their shared basis and copy it on write** (decision 159, rewritten), for the same reasons as
      the curves (127, 132): `BasisSurface()`/`BasisCurve()` freeze and return the stored object; `UReverse`,
      `VReverse` and `Transform`, where they change the basis, copy it first if it is frozen
      (`Geom_SweptSurface.OwnBasisCurve()`, private protected, for both swept surfaces). The iso-curves that reuse
      the basis curve (revolution `UIso`, extrusion `VIso`) already copy it, as in OCCT, so they are never frozen.
      Unit tests check that a handed-out basis is frozen, the same object each time, unchanged by its owner's
      reversal and transformation, and that the owner's values are unchanged.
173. **The tests use the real curves.** The stand-ins of decision 164 (`TestCircle` in the unit tests, `PortCircle`
      and `PortBSplineCurve` in the oracle tests) are deleted: the swept surfaces revolve and extrude Geom_Line,
      Geom_Circle and Geom_BSplineCurve, and the oracle tests now compare the swept surfaces' continuity on B-spline
      curves too. `TestLine` stays: it tests the abstract layer (Geom_Curve's own non-virtual members).
174. **The iso-curves are oracle-tested** (`GeomIsoCurveCompare`): the curve's class, its definition (line axis;
      circle radius and axes; trimmed curve's basis, recursively; Bezier and B-spline poles, weights, knots,
      multiplicities, degree, rationality), parameters, closure, periodicity, continuity, and D0-D3 and DN at the
      ends and inside. `UIso` and `VIso` at the bounds, inside, and a period on in a periodic direction, of every
      elementary surface, of surfaces of revolution and extrusion of circles and B-spline curves, of B-spline
      surfaces, of their U and V reversed and transformed copies and of trimmed ones (Geom_SurfacesOracleTests
      `Iso_curves`, 4.24 million values), of random B-spline surfaces with and without `CheckRational` (0.67 million)
      and random Bezier surfaces (0.37 million), and in the swept surfaces' test. Every value is bit-identical. The
      inputs are built as decision 165 says.
175. **The curve oracle tests build their inputs as the surface ones do** (decision 165). GeomCurveOracleTests'
      directions, axes and transformations come from `GeomOracleInputs` (`Dirs`, `Ax1s`, and `Ax2sFromDirection`,
      gp_Ax2 from a point and a main direction); the Geom2d tests' from its new 2D functions (`Dir2ds`, `Ax2ds`). Each
      draws the same random numbers as the `Oracle` function it replaces, so the test cases are unchanged.
      Bit-identical shares of the Geom curve tests, before and after: line and conics 77.8% → 100% (292,372 values),
      Geom_TrimmedCurve 85.5% → 100% (340,084), Geom_OffsetCurve 75.8% → 100% (594,219), Bezier evaluation (the rest)
      97.0% → 100% (234,003; largest difference before 6.5e-9), B-spline evaluation (the rest) 96.8% → 100% (397,581);
      the other five tests were already 100%. In all, 2,780,858 of 3,058,431 values (90.9%) → all of them. The "trig"
      differences of decision 130 (a) were differences of the inputs: native and port sin and cos agree on them. So
      the 1e-6 tolerance of transformed curves is gone (default 1e-12, and still bit-identical), and `IsClosed` is
      compared on infinite curves again (130 (b)). The Geom2d tests were already 100% (their directions were settled
      with `Oracle.Settled`) and stay so; their B-spline edit chains now include rotations (decision 139),
      bit-identical too (304,072 values).
## Phase 2: unit 2B, adaptors on surfaces (29 October 2026)

Geom2dAdaptor (Geom2dAdaptor_Curve, MakeCurve), Adaptor3d_CurveOnSurface, Adaptor3d_IsoCurve, Adaptor3d_TopolTool and
Hermit.

176. **Geom2dAdaptor_Curve's `std::variant` of curve data is an `object?` field**: null (monostate), a boxed gp
      value (`gp_Lin2d`, `gp_Circ2d`, ...; never changed once boxed, so a shallow copy shares it where C++ copies
      it) or one of the nested data classes `OffsetData`, `BezierData`, `BSplineData`, read with type patterns where
      OCCT uses `std::get`/`get_if`. The data classes are internal and the field `private protected`: they hold the
      evaluation caches and the stored evaluation-representation descriptors, which callers must not reach
      (decision 100). OCCT's protected fields `myCurve`, `myTypeCurve`, `myFirst` and `myLast` stay protected
      (BRepAdaptor_Curve2d derives from the class). `ShallowCopy` builds new data objects, so caches are never
      shared, as in OCCT. `final` evaluators are `sealed override`. Rejected: one field per alternative (more code
      to diff against OCCT, for no gain).
177. **Geom2dAdaptor_Curve reads curves as OCCT does, without copying.** B-spline paths pass `InternalKnots()`,
      `InternalMultiplicities()`, `InternalKnotSequence()`, `InternalPoles()` and `InternalWeights()` to BSplCLib and
      BSplCLib_Cache (decisions 131, 140). The Bezier cache's flat knots, which OCCT wraps from
      `BSplCLib::FlatBezierKnots(deg)` on every rebuild, are the curve's `InternalKnotSequence()`: the same values,
      shared per degree, without an allocation. `IsBoundary` and `BSplCLib.LocateParameter` return their spans
      through `out`. `Bezier()` and `BSpline()` keep the base's non-nullable signatures and return null for another
      curve type, as OCCT returns a null handle; `Curve()` is nullable. The adaptor keeps the caller's curve, as
      OCCT's handle does (no copy, no freeze: adaptors are working objects). `Geom2dAdaptor` is a static class;
      `MakeCurve` returns the adapted Bezier, B-spline or offset curve itself, as OCCT does.
178. **Geom2dAdaptor_Curve is oracle-tested** (`tests/OcctNet.Oracle.Tests/Geom2dAdaptor`) on the curve types
      native OCCT binds: random B-splines (degrees 1 to 5, rational or not, periodic or not; over their whole range,
      a random part and, when periodic, a range shifted across the period start), lines, circles and ellipses, each
      also through a Geom2d_TrimmedCurve. Compared: D0 to D3, Value, EvalD0, DN and EvalDN up to degree + 1 at the
      ends (the B-spline boundary-span path), at the knots and at random parameters; the type, range, closure,
      periodicity, period, continuity, NbIntervals and Intervals for C0 to CN, Resolution, NbSamples, the B-spline
      counts and the elementary values; on the adaptor, its Trim and its ShallowCopy, and after a reload with
      another range. 1,029,496 values, all bit-identical. Hyperbolas, parabolas, Bezier and offset curves are not
      bound natively; the unit tests compare them with the curves' own evaluation.
179. **Adaptor3d_IsoCurve.** OCCT's implicit copy constructor (used by `Trim`, sharing the surface) is a private
      constructor; `ShallowCopy` shallow-copies the surface, as in OCCT. `Bezier()` and `BSpline()` on Bezier and
      B-spline surfaces go through `Geom_BezierSurface/Geom_BSplineSurface.UIso/VIso`, which still throw
      `Standard_NotImplemented` behind unit 1C's `TODO(1A)` (decision 169); they will work once those are replaced,
      with no change here. On surfaces of revolution and extrusion they copy the basis curve before rotating or
      translating it, as OCCT does, so a frozen basis curve is never changed.
180. **Adaptor3d_CurveOnSurface** is a working object: `Load(Surface)`, `Load(Curve2d)` and `Load(Curve2d, Surface)`
      are public; `GetCurve()`/`GetSurface()` return the stored adaptors. OCCT's `ChangeCurve()`/`ChangeSurface()`
      (a `handle&` to the field) are `public ref Adaptor2d_Curve2d? ChangeCurve() => ref myCurve;` (a reference to a
      class field needs no `[UnscopedRef]`); assigning through them doesn't recompute the special cases, as in OCCT.
      The interval cache that the const `NbIntervals` writes through `const_cast` is two plain fields;
      `ShallowCopy` shares the sequence as OCCT shares the handle, which is safe because `NbIntervals` replaces it
      rather than changing it. `Standard_ASSERT_RAISE` in `Intervals` throws `Standard_ProgramError`, as OCCT does in
      every build. The `constexpr Precision::PConfusion() / 10` tolerances are computed, not written as literals, so
      they match OCCT's to the bit. Curve and surface arrays are read through the `Internal*` accessors. The private
      member `FindBounds` declared in the .hxx is neither defined nor used in OCCT and is not ported.
181. **Adaptor3d_TopolTool** is a working object: OCCT's virtuals stay public virtual and its protected fields
      (`myS`, `myNbSamplesU/V`, `myUPars/myVPars`) protected, for BRepTopAdaptor_TopolTool. `Edge()`, a `void*` in
      OCCT, returns `object?` (null here). `SamplePoint` and `GetConeApexParam` return through `out`; `UParameters`
      and `VParameters` copy into the caller's array with `Assign` (OCCT's array assignment). `ComputeSamplePoints`
      reads poles through `InternalPoles()`. The `GeomAdaptor_Surface.hxx` include is unused: no GeomAdaptor is
      needed.
182. **Hermit is a static class.** `Solutionbis` takes `Knotmin` and `Knotmax` as `ref`: OCCT writes only one of
      them in some branches. Hermit reads the caller's curve through the `Internal*` accessors (copying the knots
      before `BSplCLib::Reparametrize`, as OCCT does) and mutates only the two Geom2d_BSplineCurves it builds
      (`InsertKnot`, `SetPole`), so frozen input curves work and the returned curve is unfrozen. The duplicated 3D
      and 2D helpers stay overloads, as in OCCT; `std::sort` of the knots is `Span.Sort` (no ties that matter).
      Hermit is not bound natively: GTests only.
183. **Test adaptors until unit 2A.** `tests/OcctNet.ModelingData.Tests/TestAdaptors.cs` has `TestCurveAdaptor` and
      `TestSurfaceAdaptor`, minimal adaptors over Geom curves and surfaces (evaluation through the Geom object, the
      type queries, one interval per direction unless interior bounds are given), so the adaptors on surfaces can be
      tested before GeomAdaptor_Curve/Surface exist. Adaptor3d_CurveOnSurface, IsoCurve and TopolTool are tested
      against the surface evaluated directly (a curve on a surface's derivatives by the chain rule); none of them can
      be built through the native bindings, so they have no oracle tests. Replace the stand-ins with GeomAdaptor
      once 2A is merged.
## Phase 2: unit 2E, constructions and sampling

GC (the Geom-returning makers, 3D and 2D), GCE2d, CPnts and GCPnts.

184. **GC follows gce's conventions (decision 117).** GC_Root is a class with OCCT's protected `TheError`; each
      GC_Make* is a sealed class computed by its constructor. `Value()` throws StdFail_NotDone on a failed
      construction, where OCCT's release build returns a null handle. OCCT's implicit `operator const
      occ::handle<T>&()` is an explicit conversion; GC has no `Operator()`. The transformation makers (Mirror,
      Rotation, Scale, Translation and their 2D twins) don't derive from GC_Root and can't fail, as in OCCT.
      Where GC converts a gce construction implicitly on a path that can fail, the port does what OCCT's release
      build does when that is well defined: GC_MakeArcOfCircle2d(P1, V, P2) with P1 = P2 reads gce_MakeLin2d's
      default line, as OCCT does (it then throws Standard_ConstructionError from gp_Dir2d, where OCCT computes NaN
      parameters). GC_MakeSegment(P1, P2) with P1 = P2 throws StdFail_NotDone from `GC_MakeLine.Value()`, where
      OCCT's release build crashes on a null line (occt-issues.md).
185. **`Value()` of a GC maker returns its result frozen** (rules, section 8), the same object at every call, as
      OCCT's const-reference return is. A maker's result is a long-lived object handed to a caller, who may share
      it between shapes. This covers the Geom_Transformation and Geom2d_Transformation results (decision 96).
      Ported code that changes a GC result in place must `Copy()` it first; GC itself only changes a result before
      handing it out (GC_MakeCylindricalSurface's `SetRadius`).
186. **GCE2d is not ported as types.** In OCCT 8 its headers are only deprecated `using` aliases of the GC 2D makers
      (`GCE2d_Root` = `GC_Root`, `GCE2d_MakeArcOfCircle` = `GC_MakeArcOfCircle2d`, ...), and C# can't export a type
      alias. Callers use the GC_*2d classes; a caller wanting the old names can declare its own `using` aliases.
187. **GCPnts's and CPnts's templates over the curve type are ported once per instantiation**, as overloads for
      Adaptor3d_Curve and Adaptor2d_Curve2d with identical bodies (rules, section 9; as decisions 128 and 137).
      GCPnts_TCurveTypes, the traits resolving the point, Bezier, B-spline and distance-function types, is
      replaced by those types written out in each instantiation. The 2D bodies were generated from the 3D ones by
      substituting the types, and a unit test (`GCPnts_PortTests.TwoD_MatchesThreeD`) checks that every algorithm
      gives bit-identical parameters and points on a planar 3D curve and the 2D curve built from the same numbers.
      Rejected: one generic body over a traits interface with static abstract members wrapping each adaptor
      method: the two adaptors share no base, their points and vectors differ (gp_Pnt/gp_Pnt2d), and the wrapper
      would add a layer between the port and OCCT's text for code that runs once per curve, not in an inner loop.
188. **CPnts's function pointers and `void*` curves.** `CPnts_RealFunction` (`double (*)(double, void*)`) is a
      delegate taking the client data as an object; the two evaluators of CPnts_AbscissaPoint are allocated once.
      CPnts_UniformDeflection keeps its curve as an object with OCCT's `my3d` flag. As in OCCT, the CPnts and
      GCPnts objects that take a curve (CPnts_AbscissaPoint after `Init`, CPnts_UniformDeflection,
      GCPnts_DistFunction) keep a reference to it. Functions that fail assign their `out` values (decision 32).
189. **GCPnts's algorithms are sealed working classes** (decision 21) with OCCT's API: `Initialize` recomputes,
      results are read by index (`NbPoints`, `Parameter(i)`, `Value(i)`), and GCPnts_TangentialDeflection keeps its
      public `AddPoint`. No collection accessors are added: the results are small working values, and a Bento
      wrapper can copy them out. `NbPoints`, `Parameter` and `Value` check `IsDone`, which OCCT's release build
      compiles out; GCPnts_QuasiUniformAbscissa, which reads a GCPnts_UniformAbscissa's results without checking,
      uses internal unchecked accessors so that a failed computation is copied as OCCT copies it.
190. **Test stand-ins for the adaptors.** GeomAdaptor_Curve (unit 2A) and Geom2dAdaptor_Curve (unit 2B) are not
      ported yet, and nothing in GCPnts constructs them. The tests use `TestGeomAdaptor_Curve` and
      `TestGeom2dAdaptor_Curve` (tests/OcctNet.ModelingData.Tests/TestAdaptors.cs, compiled into the oracle tests
      too), which mirror OCCT's adaptors for lines, conics, Bezier and B-spline curves (BSplCLib_Cache evaluation,
      local evaluation at the bounds, continuity, intervals, resolution); offset curves and evaluation
      representations go through the curve's own evaluation. An oracle test compares them with the native adaptors:
      405,817 values, all bit-identical. Units 2A and 2B replace them with the real adaptors and keep that test as a
      regression check.
191. **Oracle coverage.** build123d.NET binds GC_MakeArcOfCircle, ArcOfEllipse, ArcOfHyperbola and ArcOfParabola,
      GCPnts_AbscissaPoint, GCPnts_UniformDeflection and GCPnts_QuasiUniformDeflection. On random lines, conics,
      Bezier and B-spline curves (rational, periodic, with C0 to C2 breaks), 2D and 3D, the arc makers' status,
      bounds, points and circles, and the lengths, abscissa points (every constructor) and both deflection
      samplings (every point and parameter, with and without control, at C0, C1 and C2, on whole curves and on
      reversed sub-ranges) are compared: 191,740 values, all bit-identical. GC_MakeSegment and the other makers,
      GCPnts_UniformAbscissa, GCPnts_QuasiUniformAbscissa and GCPnts_TangentialDeflection are not bound and are
      unit-tested only.
## Phase 2: unit 2D, AdvApp2Var

AdvApp2Var_ApproxF2var (f2c), the working classes (Context, Criterion, Node, Iso, Patch, Network, Framework; OCCT's
"strips" are sequences of isos, with no class of their own), AdvApp2Var_EvaluatorFunc2Var and
AdvApp2Var_ApproxAFunc2Var. With unit 1F, all 35 files of the package.

192. **ApproxF2var is converted by `tools/f2c-to-cs.py`** (decision 115), given MathBase's signatures as JSON (read off
      AdvApp2Var_MathBase.cs, one letter per parameter). The script now also knows the evaluator parameter
      (`const AdvApp2Var_EvaluatorFunc2Var&`, passed through), `int const*` parameters, the AdvApp2Var_ApproxF2var
      prefix, and the evaluator calls (`(*const_cast<AdvApp2Var_EvaluatorFunc2Var*>(&foncnp)).Evaluate(...)`,
      rewritten as a call with the evaluator's parameter modes). Fixed by hand: the three work pointers OCCT builds
      from an offset alone (`reinterpret_cast<double*>(iofwr * sizeof(double))`), which are `wrkar + iofwr` after the
      port's `mcrrqst_` (decision 114); a `(int*)ndimen` cast; two comma expressions spread over several lines. The
      class is internal and static, with OCCT's public and private functions. It has no state: its Fortran DATA
      tables (`xmax2`, `xmax4`, `xmax6`) are constant spans, each routine that allocates makes its own
      AdvApp2Var_SysBase, and MathBase's Hermite cache is per thread (decision 113), so approximations run in parallel
      (`Approximations_RunInParallel_AsSequentially`).
193. **AdvApp2Var_EvaluatorFunc2Var is a public abstract class with by-value inputs** (as decision 145):
      `Evaluate(int theDimension, ReadOnlySpan<double> theUStartEnd, ReadOnlySpan<double> theVStartEnd, int
      theFavorIso, double theConstParam, int theNbParams, ReadOnlySpan<double> theParameters, int theUOrder, int
      theVOrder, Span<double> theResult, out int theErrorCode)`. None of OCCT's implementations writes an input.
      `operator()` is left out. ApproxF2var calls it through a private extension method taking f2c pointers, so its
      call sites keep OCCT's arguments; the spans run to the end of the caller's arrays (a C pointer has no length),
      and an implementation writes `theNbParams * theDimension` values.
194. **The working classes are public classes with OCCT's public mutators** (decision 21): GeomPlate's criteria and
      GeomFill use them. Where C++ copies by value: AdvApp2Var_Network's and AdvApp2Var_Framework's constructors copy
      their sequences (the Framework its strips too), sharing the patches, nodes and isos as OCCT's handles do;
      AdvApp2Var_Node's `operator=` is `Assign`, which ApproxAFunc2Var's ComputeConstraints uses as OCCT does
      (`N1 = *myConstraints.Node(indN1)`, `*myConstraints.Node(indN1) = N1`); AdvApp2Var_Context has no mutators and
      is shared. Their accessors return their own arrays (shared: don't modify), nullable where OCCT's handles may be
      null (a default context, an iso or patch not yet approximated). Network's `operator()` are indexers;
      `FirstNotApprox` takes `ref` indices (left unchanged when it finds nothing); `SameDegree` has `out`
      parameters. `Dump` takes a TextWriter and `ToString()` returns what it writes (decision 34).
195. **Small translations.** Iso::MakeApprox's moving pointers (`CRBAPP += SZCRB`) are offsets into the arrays'
      spans, and its `goto FINISH` (which only freed two arrays) a return. The Context's tolerance arrays of a
      dimension without subspaces may be null: OCCT passes the handles to helpers that read them only for each
      subspace, as the port does. `pow` is `Math.Pow`.
196. **Only 3D, as OCCT documents.** Iso and Patch read and write three coordinates whatever the dimension, and pass
      3 as the subspace dimension to the Fortran: with 1D or 2D subspaces OCCT reads and writes outside its arrays,
      and the port throws (IndexOutOfRangeException in MathBase's `mmtrpj4_` for a 1D function).
197. **ApproxAFunc2Var's results.** `Surface(i)` returns the stored Geom_BSplineSurface, frozen when it is built
      (section 8 of the rules): callers keep it, and ported code that would change it copies it first. The error
      accessors `MaxError(Dimension)`, `AverageError`, `UFrontError` and `VFrontError` return copies, null where
      OCCT returns a null handle (decision 146); the `(Dimension, Index)` forms read the stored values. The tolerance
      arguments are nullable.
198. **mma2ce1_'s stray write is kept in its work zone.** OCCT writes the Jacobi maxima of both directions at the
      same place (occt-issues.md); when the U Jacobi degree exceeds the V one by more than about 13, the U values run
      past the work zone, and native OCCT corrupts its heap (the native harness crashes). The port allocates the
      excess (`iszwr += Max(0, isz5 - isz6 - isz7)`), so the stray values land in the zone, unread: results are
      OCCT's wherever OCCT doesn't crash, and defined where it does. Rejected: throwing, which would make
      GeomConvert_ApproxSurface fail for valid, if lopsided, degree limits. Fixing the bug itself (writing the U
      maxima at ipt4) waits, as decision 10 says: it changes every approximation's result.
199. **AdvApp2Var is checked against native OCCT** (a harness not committed, as decisions 84 and 116): a DLL built
      with MSVC against build123d.NET's OCCT 8.0.1 install (`.deps/install/win-x64`: headers and TKernel, TKMath,
      TKG3d, TKGeomBase), whose evaluator calls back into the same C# function as the port's, so both sides
      approximate identical values. End to end, AdvApp2Var_ApproxAFunc2Var on 15,100 random cases (polynomial,
      trigonometric, mixed, spherical and planar functions; random domains, tolerances 1e-2 to 1e-9, frontier
      tolerances, favoured iso U, V or none, continuities C0 to C2 in each direction, precision codes 0 to 3, degree
      limits 2·order+1 to 14, 1 to 200 patches, without and with a criterion of each type and repartition): every
      result (IsDone, HasResult, the errors, CritError, degrees, knots, multiplicities and poles; 19.6 million
      values) is bit-identical in the 15,062 cases native OCCT survives; the other 38 overflow OCCT's work zone
      (1907) and are run by the port only. Function by function, the 13 public routines of AdvApp2Var_ApproxF2var
      (mma2roo_, mma2jmx_, mmapptt_, mma1her_, mma2can_, mma2ac1_ to mma2ac3_, mma2fx6_, mma2cdi_, mma2ds1_,
      mma2fnc_, mma2ce1_), including the order -1 paths that ApproxAFunc2Var never takes, on 12,977 random calls
      over buffers prefilled with the same random values: all outputs and error codes bit-identical. Not compared
      natively: the failure paths (no random case failed on either side) and Patch's `Poles`.
200. **For unit 4C (GeomConvert_ApproxSurface).** It passes `nul1` (one zero) and `nul2` (four zeros) for the 1D and
      2D tolerances, an AdvApprox_PrefAndRec in each direction, and an evaluator over an Adaptor3d_Surface that
      re-trims the adaptor on each call (`myAdaptor = myAdaptor->UTrim(...)` on a mutable member: the port's
      evaluator holds a reassignable field). Its `MaxError()` is `approx.MaxError(3, 1)`, which inherits the error
      underestimate of occt-issues.md, and its surface is ApproxAFunc2Var's frozen one. The native harness's approach
      (a callback evaluator) carries over to an oracle test of GeomConvert_ApproxSurface once the bindings whitelist
      it.
## Phase 2: unit 2C, discrete approximation (29 October 2026)

AppCont, AppDef, Approx_ComputeLine and Approx_BSplComputeLine (the `.gxx` AppDef instantiates), with
Approx_ParametrizationType, Approx_Status and Approx_MCurvesToBSpCurve.

201. **AppDef_MultiPointConstraint and AppDef_MultiLine are classes with OCCT's public API**, as
      AppParCurves_MultiPoint (decision 141): OCCT copies them by value, but their data are handles, so a C++ copy
      shares the points, tangents and MultiPointConstraints, and a C# reference does the same. The setters
      (`SetValue`, `SetPoint`, `SetTang`, `SetCurv`, ...) stay public (decision 21): callers fill a MultiLine before
      approximating it. `AppDef_MultiLine.Value(i)` returns the stored MultiPointConstraint (shared). OCCT declares
      `AppDef_MultiLine::SetParameter` but never defines it: left out. `Dump` takes a TextWriter, and `ToString()`
      returns what it writes. GeomAPI's implicit conversion of a point array to a MultiLine is written out
      (`new AppDef_MultiLine(points)`).
202. **Approx_ComputeLine and Approx_BSplComputeLine are generic classes over the MultiLine and its tool**,
      `Approx_ComputeLine<MultiLine, LineTool>` and `Approx_BSplComputeLine<MultiLine, LineTool>`, `where LineTool :
      struct, IApprox_LineTool<MultiLine>`. The new interface `IApprox_LineTool<MultiLine>` (in Approx) derives from
      `IAppParCurves_LineTool<MultiLine>` (decision 142) and adds the static abstract `WhatStatus`, `MakeMLBetween`
      and `MakeMLOneMorePoint`; the last takes `out MultiLine OtherLine` (OCCT passes a default-constructed line by
      reference), and implementations must always assign it. The `.gxx` helper macros (Approx_MyGradient,
      Approx_ParLeastSquareOfMyGradient, Approx_MyBSplGradient, Approx_MyGradientbis, ...) are, in every
      instantiation OCCT makes (AppDef, GeomInt, BRepApprox), the AppParCurves generics over the same MultiLine and
      tool, so the generic bodies name those directly. AppDef_MyLineTool is a `readonly struct` implementing the
      interface. OCCT's 24 AppDef instantiation classes (`*_0.cxx`) are sealed subclasses of the generics with
      OCCT's constructors forwarded: AppDef_Compute and AppDef_BSplineCompute (the public API that GeomAPI, AppBlend
      and HLRTopoBRep use), and the 22 helper names (AppDef_TheLeastSquares, AppDef_MyGradientOfCompute,
      AppDef_BSpParLeastSquareOfMyBSplGradientOfBSplineCompute, ...), written once by a scratch script. Files
      already listed as ported by AppParCurves (decision 142) aren't listed again. Rejected: a non-generic port per
      instantiation (GeomInt and BRepApprox instantiate the same `.gxx` over their own lines).
203. **Approx_MCurvesToBSpCurve is ported in this unit**, ahead of the rest of Approx (unit 5A): ComputeLine's
      `SplineValue` uses it. Its `myDone` (written, never read) is left out (decision 143); `Append` stores a copy
      sharing the poles, as OCCT's sequence copies the MultiCurve.
204. **Ownership of the results** (decision 143). `Approx_BSplComputeLine.Value()` and `ChangeValue()` return
      the stored MultiBSpCurve, `Approx_ComputeLine.Value(i)` and `ChangeValue(i)` the stored MultiCurve (OCCT's
      const and non-const references; through ChangeValue a caller may modify it, as in OCCT), and `SplineValue()`
      the object's own joined curve; `Parameters()` and `Parameters(i)` return the object's own arrays. Shared:
      don't modify. Where the C++ copies a MultiCurve into a member or a sequence (`TheMultiBSpCurve =
      SQ.BSplineValue()`, `myMultiCurves.Append(TheMultiCurve)`), the port writes the copy constructor `new(x)`,
      which shares the poles as C++ does. `Error` has `out` parameters. OCCT leaves ComputeLine's `tolreached`
      uninitialised until an approximation sets it: false in the port. The private `Compute`'s tolerances and bad
      index are `out` (assigned 0 first); C++ `aParams = Para` and `V1 = 0.5 * (V1 + V2)` are `Initialized(...)`
      (decision 31). C# forbids a nested local named as a later outer one, so ComputeCurve's two-point branch names
      its curve and parameters `mySCU2` and `ThePar2`.
205. **AppCont's tables are generated by `tools/gen-appcont-matrices.cs`** (decision 77): one
      `ReadOnlySpan<double>` property per `.pxx` (`THE_BBMATRIX_DATA`, `THE_INV_MMATRIX_DATA`, `THE_IBPMATRIX_DATA`,
      `THE_IBTMATRIX_DATA`, `THE_VBMATRIX_DATA`: 6,200, 6,200, 4,900, 3,795 and 105,000 values) on the internal
      static partial class `AppCont_ContMatrices`, which also holds the `.pxx` functions (MMatrix, InvMMatrix,
      IBPMatrix, IBTMatrix, VBernstein; OCCT's namespace is internal to TKGeomBase, and every OCCT.NET assembly sees
      internals). Every literal is copied as written ("1." becomes "1.0"); the three IBP entries OCCT writes as
      `a - b` stay expressions, which both compilers fold the same way. The generator checks each table's size
      against its offset formula. **AppCont_Function** is an abstract class with OCCT's protected counts,
      `PeriodInformation(int, out bool, out double)`, and `Value` and `D1` filling NCollection arrays.
      **AppCont_LeastSquare** is a working object: `Value()` returns its own MultiCurve, rebuilt on each call as in
      OCCT; `Error` has `out` parameters; OCCT's global struct `PeriodicityInfo`, used only there, is a private
      nested struct.
206. **FEmTool_Curve.D2 reproduces OCCT's read for elements of degree 1.** At C0, `FEmTool_Curve::ReduceDegree`
      can lower an element to degree 1; `D2` then evaluates a polynomial of degree -1, whose value is the Dimension
      coefficients just before the element's block of second-derivative coefficients (occt-issues.md). The port
      (which threw from a span slice) now reads them as OCCT does for a later element (the previous element's
      highest coefficients) and uses zeros for the first element, where OCCT reads before the array. Native OCCT
      reads uninitialised memory there (and for a previous element whose highest coefficients were never
      computed), so its C0 variational approximations can change from run to run: the oracle tests use C1 and C2,
      and a unit test pins the port's values. Found by the AppDef_Variational oracle test.
207. **AppDef_Variational's API.** A working object with OCCT's public setters (decision 21). `Value()` returns a
      copy sharing the poles (OCCT's by-value return, decision 141); `Parameters()` and `Knots()` return its own
      HArray1s (shared: don't modify, decision 143); `Criterium` and `CriteriumWeight` have `out` parameters. `Dump`
      (iostream formatting) is left out, and so are OCCT's checks that the bools WithMinMax and WithCutting are 0 or
      1. **AppDef_SmoothCriterion** is an abstract Standard_Transient class (`EstLength()` returns `ref double`;
      `GetCurve(out FEmTool_Curve?)` beside OCCT's non-virtual `Curve()`; `QualityValues`, `ErrorValues`,
      `GetEstimation` and `GetWeight` with `out`); **AppDef_LinearCriteria** derives from it and shares the
      MultiLine, parameters and curve it is given.
208. **AppDef_Variational's internals.** The private helpers take `ref` or `out` as their C++ use requires
      (TheMotor's curve and Adjusting's criterion, weights and curve `ref`; SplitCurve's OutCurve `ref`, since OCCT
      leaves it unchanged when nothing is cut; InitCutting's curve `out`). `std::stable_sort` and `std::sort` on the
      parameter and knot arrays are a private stable insertion sort with `<` (the same order; the arrays are short).
      `math_Vector` and `NCollection_Array1` views over raw pointers use the buffer constructors; `FEmTool_Assembly*`
      with `new` and `delete` are plain references. Where OCCT tests a constraint's point before the constraint
      count (and so reads one past the constraints), the port tests the count first; the results differ only when
      OCCT's out-of-range value equals the point index. OCCT clamps MaxDegree to 30, but FEmTool's linear criteria
      accept at most 14 (decision 148): above 14 the port throws Standard_ConstructionError where OCCT reads past its
      table.
209. **AppDef is oracle-tested through GeomAPI_PointsToBSpline** (`tests/OcctNet.Oracle.Tests/AppDef`), mirroring
      its three `Init` forms exactly: AppDef_BSplineCompute with the settings Init derives (two iterations and
      Squares for Tol3D <= 1e-3, else the gradient; the continuity switch; `Perform` on the points); the form with
      parameters (normalised to [0, 1], iso-parametric, Squares, no iterations, knots reparametrised back with
      `BSplCLib.Reparametrize`); and the weighted form through AppDef_Variational (1,000 segments, no min-max, the
      iteration count, the criterion weights, failures in `Approximate` caught as OCCT does, setup failures thrown
      on both sides). Point sets are random points or samples of random polynomials, helices and sines (2 to 40
      points, with and without noise, unevenly spaced), degrees 1 to 8, continuities C0 to C3, tolerances 1e-6 to
      1. Poles, knots, multiplicities and degree are compared: 64,971 values for the three parametrisations, 26,004
      with parameters and 15,905 for the variational form (161 curves, C1 and C2 only: decision 206), all
      bit-identical. AppDef_Compute, AppCont_LeastSquare and the Approx classes have no native entry point in the
      bindings; unit tests cover them.

## Phase 2: unit 2A, offset surfaces and GeomAdaptor

Geom_OffsetSurface with Geom_OffsetSurfaceUtils.pxx and Geom_OsculatingSurface.pxx; GeomAdaptor (GeomAdaptor,
GeomAdaptor_Curve, GeomAdaptor_Surface, GeomAdaptor_SurfaceOfRevolution, GeomAdaptor_SurfaceOfLinearExtrusion,
GeomAdaptor_TransformedCurve, GeomAdaptor_TransformedSurface); Adaptor3d_HSurfaceTool.

210. **The adaptors' `std::variant` of evaluation data is an `object?` field.** GeomAdaptor_Curve's
      `myCurveData` and GeomAdaptor_Surface's `mySurfaceData` hold null (`std::monostate`), a boxed gp value
      (gp_Lin, gp_Circ, ..., gp_Pln, gp_Torus) or one of OCCT's data structs, which are internal sealed nested
      classes with OCCT's names and public fields (`OffsetData`, `BezierData`, `BSplineData`, `ExtrusionData`,
      `RevolutionData`; the cache is a plain mutable field where OCCT marks it `mutable`). `std::get<T>` is a cast,
      `std::get_if<T>` an `is` pattern, so the code reads as OCCT's. Boxed values are never changed in place, so
      `ShallowCopy` shares them as OCCT copies them. No other package uses the structs (checked across OCCT), so
      they are internal; GeomAdaptor_Surface's field is `private protected` for the two derived adaptors, its other
      fields `protected` as in OCCT. Rejected: one typed field per alternative (eleven fields, and every
      `Load`/`ShallowCopy` resetting them) and a class hierarchy (a redesign). Unboxing copies a gp value per
      evaluation, without allocating.
211. **Geom_OffsetSurface shares its basis frozen, and copies it on write** (as the other composite surfaces,
      decisions 159 and 172). The constructor and `SetBasisSurface` copy the caller's surface (flattening trimmed
      and offset surfaces as OCCT does); `BasisSurface()` freezes the stored basis and returns it, the same object
      each time; `UReverse`, `VReverse` and `Transform` copy a frozen basis before changing it. `Surface()` returns
      that frozen basis for a zero offset (OCCT returns the handle), and a new surface otherwise. The osculating
      surfaces are frozen when built: copies of an offset surface share them, as OCCT's copy constructor shares the
      handles, and `UOsculatingSurface`/`VOsculatingSurface` hand them out. Every path was checked against OCCT; the
      values are OCCT's (oracle, all bit-identical).
212. **Geom_OffsetSurfaceUtils' templates are generic methods over a struct basis.** OCCT instantiates them over
      `Geom_Surface*` (Geom_OffsetSurface) and `handle<GeomAdaptor_Surface>` (GeomAdaptor_Surface); the bodies are
      1,800 lines, so they are written once, `where TBasis : struct, IBasisSurface`, with two readonly struct
      wrappers (`GeomSurfaceBasis`, `AdaptorBasis`) that the JIT specialises (rules, section 9; decision 41's
      pattern). The osculating-surface query is a nullable `Geom_OsculatingSurface` in both: OCCT's
      GeomAdaptor_Surface asks the Geom_OffsetSurface, whose answer is `myOscSurf && myOscSurf->...`, so it passes
      the offset surface's internal `OsculatingSurface()`. The query's `bool&`/`handle&` results are `ref` (left
      unchanged when the query is false, decision 57). OCCT's NCollection_LocalArray buffers under NCollection_Array2
      are NCollection_Array2 allocations (decision 78): only in D2, D3, DN and the singular paths. Rejected:
      overloads per instantiation as decision 128 did for the 300-line curve helpers (two copies of 1,800 lines).
213. **Geom_OffsetSurface's API follows the other surfaces** (decisions 158, 163): `SetBasisSurface`,
      `SetOffsetValue`, `SetEvalRepresentation` and `ClearEvalRepresentation` are internal with `CheckMutable()`;
      the twins are `WithBasisSurface`, `WithOffsetValue`, `WithEvalRepresentation` (stores a clone) and
      `WithoutEvalRepresentation`; `EvalRepresentation()` is internal (GeomAdaptor shares it, as OCCT does). The
      iso-curve evaluators are private nested `AdvApprox_EvaluatorFunction`s. Geom_OsculatingSurface, a private
      .pxx class in OCCT, is internal sealed, immutable after `Init`.
214. **Adaptors are working objects** (decision 101): `Load`, `Reset`, `SetTrsf` and `LoadCurveOnSurface` are
      public. `Curve()`, `Surface()`, `BSpline()`, `Bezier()`, `OffsetCurve()` return the loaded geometry itself
      (OCCT's handles): frozen when it came out of a composite (`Load` of a trimmed curve or surface reads
      `BasisCurve()`/`BasisSurface()`), the caller's object otherwise; the adaptors never change it. `Curve()` and
      `Surface()` are nullable (null before a `Load`). The embedded adaptors of GeomAdaptor_TransformedCurve and
      GeomAdaptor_TransformedSurface (C++ members by value) are owned objects, and `Curve()`, `ChangeCurve()` and
      `AdaptorSurfaceOriginal()` return them; OCCT's deprecated `Surface()` and `GeomSurface()` are `[Obsolete]`.
      `ShallowCopy` never shares a cache, as in OCCT; where OCCT copies a GeomAdaptor_Surface by value (the
      transformed surface's cached adaptor, sharing its cache handle), the port shallow-copies it, so no two
      adaptors share a cache. Evaluation results are OCCT's either way.
215. **GeomAdaptor_TransformedCurve's curve on surface is typed `Adaptor3d_Curve`** until unit 2B's
      Adaptor3d_CurveOnSurface is merged (`TODO(2B)` on the field, `LoadCurveOnSurface` and `CurveOnSurface()`):
      every call it makes is an Adaptor3d_Curve virtual, so only those three declarations change then.
216. **Adaptor3d_HSurfaceTool is a static class over `Adaptor3d_Surface`** (OCCT passes handles). `IsSurfG1`'s
      default `Precision::Angular()` is a constant (decision 129). `GetType(Adaptor3d_Surface)` is a static
      overload, so it doesn't hide `object.GetType`.
217. **Oracle tests** (`tests/OcctNet.Oracle.Tests/GeomAdaptor`, `Geom/Geom_OffsetSurfaceOracleTests.cs`; shared
      random inputs in `AdaptorOracleInputs`, built identically on both sides as decision 165 says).
      GeomAdaptor_Curve on lines, conics, Bezier, B-spline, periodic B-spline, offset and trimmed curves, whole and
      bounded, reloaded: parameters, continuity, NbIntervals/Intervals for every GeomAbs_Shape (throwing on both
      sides or neither), resolution, type and its data, degree, poles, knots, closure, periodicity, D0-D3 and DN on
      shuffled sequences through spans, knots and bounds (the cache and the boundary `Local*` evaluation), Trim and
      ShallowCopy: 1,496,907 values. GeomAdaptor_Surface on every surface kind, whole and bounded with snapping
      tolerances: the same queries, U and V intervals, resolutions, D0-D3 and DN at bounds, within tolerances and on
      knots, UTrim/VTrim, BasisCurve/BasisSurface: 2,074,589 values. Geom_OffsetSurface on offsets of every basis
      (equivalent surfaces with flipped and degenerate radii, C0 bases with and without the check, nested and
      trimmed), reversed, transformed (TransformParameters, ParametricTransformation), with a new offset, trimmed,
      through GeomAdaptor_Surface, and their iso-curves (AdvApprox): 567,104 values; at singular points (collapsed
      pole rows of B-spline and Bezier surfaces with their osculating surfaces, at one end and at both; sphere-like
      revolutions; a cone apex through a surface of revolution), through the surface and the adaptor, and
      reversed: 1,082,775 values. All 5,221,375 values are bit-identical. GeomAdaptor_SurfaceOfRevolution,
      GeomAdaptor_SurfaceOfLinearExtrusion, the transformed adaptors, GeomAdaptor and Adaptor3d_HSurfaceTool have no
      native binding: the ported GTests and OCCT.NET's own unit tests cover them.

## Phase 2: unit 3B, curve extrema

Extrema's curve templates (Extrema_GGExtPC, GGenExtPC, GFuncExtPC, GLocateExtPC, GenLocateExtPC, GFuncExtCC,
GGenExtCC, GenLocateExtCC) in their 3D and 2D instantiations, Extrema_CurveTool, Extrema_Curve2dTool,
Extrema_ExtCC, Extrema_ExtCC2d, Extrema_LocateExtCC, Extrema_LocateExtCC2d and Extrema_GlobOptFuncCC.

218. **The templates are ported once per instantiation, the 2D files generated from the 3D ones** (rules, section 9;
      as GCPnts, decision 187). Every template is instantiated exactly twice, over (Adaptor3d_Curve,
      Extrema_CurveTool, gp_Pnt, gp_Vec, Extrema_POnCurv, Extrema_ExtPElC) and their 2D counterparts: its parameters
      are the 3D/2D axis and nothing else. The 2D files are the output of `tools/extrema-to2d.sed` on the 3D ones
      (no hand edits; re-running it is the check that they are in step), and
      `Extrema_CurvePortTests.TwoD_MatchesThreeD` checks that ExtPC, LocateExtPC, ExtCC (through ECC) and
      LocateExtCC give bit-identical parameters, points and distances on planar 3D curves and the 2D curves built
      from the same numbers. Rejected: one generic body over a static-abstract tool interface. gp_Pnt/gp_Pnt2d and
      gp_Vec/gp_Vec2d share no interface, so the interface would also have to wrap the point and vector operations
      (`TheVector(P, Q)`, `Dot`, `Magnitude`, `SquareDistance`, the scalar operators), and the elementary solvers
      (ExtPElC/ExtPElC2d, ExtElC/ExtElC2d) have different overload sets: the port would no longer read as the C++
      does, for no speed gain (the per-instantiation code calls the adaptors directly, as OCCT's inlined tools do).
      Extrema_ExtCC/ExtCC2d and LocateExtCC/LocateExtCC2d are separate classes in OCCT and are ported by hand.
219. **One class per distinct instantiation, under OCCT's first name.** OCCT gives some instantiations several
      `using` names for the same type: Extrema_ExtPC is also Extrema_ELPCOfLocateExtPC, Extrema_EPCOfExtPC is also
      Extrema_EPCOfELPCOfLocateExtPC, and Extrema_PCFOfEPCOfExtPC is also Extrema_PCFOfEPCOfELPCOfLocateExtPC and
      Extrema_PCLocFOfLocEPCOfLocateExtPC (the same in 2D). C# can't export a type alias (decision 186), so the
      other names don't exist; their headers are in the `Port of:` lines of the class they name, and the template
      headers in those of both instantiations. The anonymous-namespace helpers of Extrema_GGenExtCC.hxx are the
      internal static class `Extrema_GGenExtCC`: the comparator (and a `Comparison<gp_XY>` over it for
      `Span.Sort`, which replaces `std::sort`; points that compare equal are identical, so the order can't depend
      on the sort algorithm), `ChangeIntervals` (taking the handle by `ref`), the cell-filter inspector as a struct
      implementing `INCollection_CellFilter_Inspector<gp_XY, gp_XY>`, and `ProjPOnC` as two overloads. The
      file-static functions of Extrema_GlobOptFuncCC.cxx are the internal static class `Extrema_GlobOptFuncCC`;
      Extrema_GlobOptFuncCCC0/C1/C2 keep OCCT's 3D and 2D constructors and type switch.
220. **Algorithm classes with OCCT's API** (decision 117): public `Initialize`/`Perform`/`SetCurve`/`SetRange`/
      `SetTolerance`, results by value (`const ThePOnC&` returns a copy), `out` for `Points`, `Point(P1, P2)` and
      `TrimmedSquareDistances`. By-value members are allocated with the object (`= new()`). The curves (OCCT's
      `TheCurve*`, `void*` in GFuncExtCC and GGenExtCC) are adaptor references kept as OCCT keeps the pointers:
      the caller's adaptor must outlive the algorithm, as in OCCT. `CurvePtr` returns the adaptor and
      `SearchOfTolerance` takes one, compared by reference as OCCT compares the pointers. Extrema_LocECC(2d) and
      Extrema_LocateExtCC(2d) are computed by their constructors (readonly fields). Extrema_ExtCC2d's
      `std::shared_ptr` locals are plain references; Extrema_ExtCC::PrepareParallelResult's two flags, `double` in
      OCCT, are `bool`. The curve tools are public static classes; `DeflCurvIntervals` returns a
      NCollection_HArray1.
221. **Function results on failure.** Where Extrema_PCFOfEPCOfExtPC(2d)'s `Value`, `Values` and `Derivative` and
      Extrema_GlobOptFuncCC's `Value` return false without writing their `out` results, the port writes 0
      (decision 32). Their callers that ignore the status (math_FunctionRoots, Extrema_ECC's Lipschitz sampling)
      reach this only where a derivative vanishes or the parameters leave the range; the oracle saw no difference.
      Extrema_CCLocFOfLocECC's results are math_Vectors filled in place, so it leaves them unchanged as OCCT does.
      `myInit` (GGenExtPC) and `myD1Init` (GFuncExtPC) are set and never read in OCCT; they are kept, with CS0414
      suppressed on those fields only (as decision 120).
222. **Left to other units.** Extrema_GCurveLocator (a template instantiated only by Geom2dInt and HLRBRep, outside
      TKGeomBase), Extrema_GlobOptFuncCS, GlobOptFuncConicS and GlobOptFuncCQuadric (used by Extrema_ExtCS and
      GenExtCS: units 4A/4B) and the enums Extrema_ExtFlag, ExtAlgo and ElementType (surface extrema). None of the
      classes of this unit constructs a GeomAdaptor_Curve, so the code has no `TODO(2A)`; the tests use the
      stand-in adaptors (decision 190) and mark it there.
223. **Oracle coverage** (`tests/OcctNet.Oracle.Tests/Extrema/ExtremaCurveOracleTests.cs`), on random lines, conics,
      Bezier and B-spline curves (rational, periodic, with C0/C1 breaks) and ranges, with points on, near and far
      from the curves:
      - Extrema_ExtPC, bound natively, compared directly (whole range, sub-ranges with random function tolerances
        through `Initialize`, an object reused for several points; every extremum's parameter, point, square
        distance and IsMin, and the trimmed distances and points), also on offsets of smooth curves (the general
        path): 44,029 of 45,057 values bit-identical. All the differences (up to 1.6e-12) are on the offset curves,
        which the test stand-in evaluates through Geom_OffsetCurve where native GeomAdaptor_Curve has its own
        evaluation (TODO(2A)); without them every value was bit-identical.
      - Extrema_ExtPC through GeomAPI_ProjectPointOnCurve (its `Load` + `Initialize(myC, First, Last)` + `Perform`
        path mirrored): 19,741 values, all bit-identical.
      - Extrema_ExtPC2d through Geom2dAPI_ProjectPointOnCurve, on the real Geom2dAdaptor_Curve: 19,766 values, all
        bit-identical.
      - Extrema_ExtCC through BRepExtrema_DistShapeShape between two edges (GeomAPI_ExtremaCurveCurve isn't bound).
        BRepExtrema_ExtCC sets the curves on the edge ranges and the tolerances max(Resolution(Confusion),
        PConfusion), curve 2 first; the port is driven the same way. DistShapeShape's solutions inside both edges
        are Extrema_ExtCC's extrema unchanged, and each must be one of the port's: 1,147 of 1,152 values
        bit-identical over 128 solutions (200 edge pairs, two thirds of them moved to touch or nearly touch); the 5
        others (2.2e-15) are one line/circle pair, solved by unit 1G's Extrema_ExtElC (trigonometric roots).
        Solutions at vertices or from DistanceSS's PERFORM_C0 projections are skipped.
      Not compared natively: Extrema_LocateExtPC(2d), LocateExtCC(2d), ExtCC2d and the ECC classes on their own
      (no binding reaches them directly); the unit tests check them against known answers, dense sampling and the
      3D/2D identity.
224. **For later units.** 4A (surface extrema): Extrema_GenExtCS/ExtCS use Extrema_GlobOptFuncCS/ConicS/CQuadric
      (not ported here) and Extrema_ExtPC/ExtCC; Extrema_ExtPExtS and ExtPRevS build GeomAdaptor_Curves and call
      Extrema_ExtPC. 4B: Extrema_ExtCS calls Extrema_ExtCC and Extrema_ExtPC on adaptors it constructs. 5A/6A
      (GeomLib, Approx_SameParameter, ProjLib_CompProjectedCurve/ProjectedCurve): they use Extrema_ExtPC,
      LocateExtPC and their 2D twins on adaptors that must stay alive as long as the algorithm (decision 220).
      Geom2dAPI_ProjectPointOnCurve (TKGeomAlgo) copies an Extrema_ExtPC2d by value (`myExtPC = theExtPC2d`); the
      port has no copy constructor, so that port should keep the constructed object instead. Once unit 2A is
      merged, the tests' `TestGeomAdaptor_Curve` should become GeomAdaptor_Curve, and the offset tolerance of the
      ExtPC oracle test should go back to the default.
## Phase 2: unit 3C, properties, persistence, hashes

LProp and GeomLProp (local properties of curves and surfaces: OCCT 8's templates `GeomLProp_CLPropsBase` and
`GeomLProp_SLPropsBase` with `LProp_CurveUtils` and `LProp_SurfaceUtils`; GeomLProp::Continuity; GeomLProp_CurAndInf2d
with its numeric helpers), GeomTools (OCCT's text format of curves, 2D curves and surfaces, the geometry of a `.brep`
file), GeomHash and Geom2dHash, and `Standard_Stream` in Foundation.

225. **C++ stream state lives beside the TextWriter and TextReader** (`Standard_Stream`, new, in Foundation). OCCT's
      `Standard_OStream&`/`Standard_IStream&` stay `TextWriter`/`TextReader` (rules, section 9); the state a C++ stream
      carries and .NET's don't is kept per writer or reader in a `ConditionalWeakTable`: the output precision
      (`Standard_Stream.Precision(OS)` and `Precision(OS, n)`, OCCT's `OS.precision()`, 6 by default) and the input's
      fail and end-of-file flags. So the GeomTools sets set precision 17 and restore it as OCCT does, a caller can set
      the precision for `GeomTools::Write`, and several reads on one reader share its failure state (phase 3's
      BRepTools reads geometry and topology from one stream). `OS << double` is `Standard_Stream.Write`:
      `printf("%.*g")` as the Windows C runtime writes it, which is .NET's "G" format with a lowercase exponent: exact
      digits, ties to even (the C runtime's "standard rounding", on in programs built with a recent SDK such as native
      OCCT; a first version rounded ties away from zero, the C runtime's legacy rounding, after a test that called it
      without that option; the oracle showed native OCCT writing 229.121246337890625 as 229.12124633789062). `inf`,
      `-inf`, `nan`, `-nan(ind)` as before (decision 68); Poly's writer now calls the same function (same output).
      Extraction follows C++: white space skipped, `>> int` stops at the first non-digit, `>> bool` reads 0 or 1 (other
      numbers give true and fail), a failed stream extracts nothing and leaves the value, overflow clamps; `ReadWord`
      with a width reads width - 1 characters. OCCT's `Strtod` (C's strtod in the C locale) is `Standard_Stream.Strtod`,
      with the end position; hexadecimal reals, which OCCT never writes, read as 0. Unit tests compare the formatting
      (about 170,000 values at precisions 1 to 20, exact ties included) and strtod (10,000 texts) with ucrtbase.dll
      through P/Invoke. Rejected: a `Standard_OStream` wrapper class (OCCT's signatures and every phase 3 caller would
      change), a precision parameter on each Print method (OCCT's callers set it on the stream), and a new extractor
      per call as Poly's (decision 68), which loses the failure state between calls.
226. **GeomTools writes `'\n'`**, as a C++ stream does; OCCT's files have CRLF on Windows only because `std::ofstream`
      opens them in text mode. Phase 3's file-level `BRepTools::Write` decides what a file gets. Reading treats CR as
      white space, as C++ does.
227. **The GeomTools sets keep OCCT's map semantics, null included.** `NCollection_IndexedMap<handle<Standard_Transient>>`
      is an `NCollection_IndexedMap<object>` with reference equality (handles hash by address) and a sentinel key for
      the null handle, which OCCT's map can hold once: `GeomTools_Curve2dSet.Add(null)` and `GeomTools_SurfaceSet.Add(null)`
      add it (the 3D set returns 0, as OCCT does), and every curve or surface that `Read` can't build is that one null
      entry, so the indices of the later ones shift, as in OCCT (occt-issues.md). The sets are classes (decision 16).
      `Read` freezes the geometry it creates (the set hands it to every caller that asks, and BRepTools shares it
      between shapes); `Add` doesn't freeze the caller's object, and the static `ReadCurve`/`ReadCurve2d`/`ReadSurface`
      return new, unfrozen geometry. A trimmed, offset or swept object whose basis couldn't be read makes OCCT
      dereference a null handle, which `OCC_CATCH_SIGNALS` turns into a caught exception where signals are converted:
      the port catches the NullReferenceException there, so the object reads as null. OCCT's console messages ("Not a
      Curve table", and the unknown-type line that the compact form writes to std::cout) go to `Console.Out`.
228. **GeomTools' API.** `GeomTools.Read(out Geom_Curve?, TextReader)` and its 2D and surface overloads (OCCT's
      `handle&` output), `GetReal(TextReader, out double)` (a word of at most 30 characters, as OCCT's `width(31)`, then
      Strtod). `GeomTools_UndefinedTypeHandler` is a public class with OCCT's virtuals; its `Read*` methods take the
      result by `ref` and return the reader. The installed handler is a process-wide static, as OCCT's is (volatile;
      `SetUndefinedTypeHandler(null)` is ignored). The type tests are on the exact type (`DynamicType() == typeof(...)`,
      OCCT's `STANDARD_TYPE` comparison), so derived classes go to the handler. GeomTools_Debug.cxx (a function for
      the debugger's command window) is not ported.
229. **The local-property templates are ported per instantiation** (rules, section 9; as decisions 128 and 187):
      `GeomLProp_CLProps` (Geom_Curve), `GeomLProp_CLProps2d` (Geom2d_Curve) and `LProp_CLProps3d` (Adaptor3d_Curve),
      `GeomLProp_SLProps` (Geom_Surface) and `LProp_SLProps3d` (Adaptor3d_Surface), each a class with the template's
      body and `LProp_CurveUtils`'s or `LProp_SurfaceUtils`'s wrappers written out with OCCT's `DirectAccess`; the 2D
      and adaptor classes are generated from the Geom ones by substituting the types (the surface bounds differ: Geom's
      `Bounds`, the adaptor's first and last parameters). The helpers that don't depend on the curve or surface are the
      internal static classes `LProp_CurveUtils` (3D and 2D overloads) and `LProp_SurfaceUtils` (in
      GeomLProp_SurfaceUtils.cs, OCCT's file). Phase 3 adds `BRepLProp_CLProps`/`BRepLProp_SLProps` (on BRepAdaptor,
      an Adaptor3d in the port, so the `LProp_*3d` bodies apply unchanged) and, later, HLRBRep's `ToolAccess`
      instantiations the same way. Rejected: one generic class over a static-abstract access interface (the gp types
      share no interface, and the bodies are short).
230. **The wrappers' reference and value arguments are reproduced.** Where OCCT binds a `const&` to a field, the port
      reads the field after the calls that update it (`IsTangentUDefined` evaluates D1U and D2U, then reads both; a D2
      evaluation can change D1's last bits). Where OCCT passes a field by value before computing it, the port copies
      it first: `MaxCurvature`, `MinCurvature`, `MeanCurvature`, `GaussianCurvature` and `IsUmbilic` return or
      compare the previous values when `IsCurvatureDefined` hasn't run (occt-issues.md; native OCCT confirms it).
      Outputs are `out` (`Tangent`, `Normal`, `CentreOfCurvature`, `TangentU/V`, `CurvatureDirections`); `Value()`,
      `D1()` and the like return values. The classes are working objects with public setters (decision 21).
      `LProp_NotDefined_Raise_if` always throws (decision 9). `LProp_BadContinuity` is ported; its macro, never used,
      isn't (decision 102). GeomLProp_NumericCurInf2d and the two function classes, private .pxx classes in OCCT, are
      internal; their `out` results are 0 where OCCT leaves them unset.
231. **The hashers are `IEqualityComparer<T>` classes** (GeomHash_CurveHasher, GeomHash_SurfaceHasher, the four mesh
      hashers, Geom2dHash_CurveHasher). OCCT's `aHasher(x)` is `GetHashCode(x)` (0 for null, as in OCCT) and
      `aHasher(a, b)` is `Equals(a, b)`, so they plug into Dictionary and HashSet. `CompTolerance` and `HashTolerance`
      are public readonly fields (mutable in OCCT, but changing them would corrupt a collection using the hasher); the
      default arguments are constants equal to Precision::Angular(), Confusion() and Computational() (decision 129; a
      test checks them). The .pxx helpers are internal readonly structs with `Hash` (a `ulong`, OCCT's `size_t`) and
      `IsEqual`, without default arguments (a struct's parameterless `new` would skip them); `PolygonOnTriHashKey` is a
      public `readonly record struct (Poly, TriRepId)`, whose own equality compares the polygon by reference (the
      hasher compares it by value).
232. **The hashers keep OCCT's semantics but not its hash values.** The same fields are compared with the same
      tolerances, dispatch is on the exact type, a type the hasher doesn't know hashes by its name and compares by
      reference, numbers are rounded to multiples of `HashTolerance` before hashing (`std::round`: away from zero), and
      Bezier and B-spline objects hash their degrees and counts only. So equal objects hash equally, and objects equal
      within the tolerance can hash differently across a rounding boundary, as in OCCT. `opencascade::hash` of an
      integer is the integer; `hashBytes` is 64-bit FNV-1a over the hashes' bytes (`GeomHash_HashUtils`), folded to 32
      bits for `GetHashCode`: deterministic across runs and platforms, unlike `System.HashCode`, so the GTests'
      inequality checks always give the same result.
233. **Test data.** The ModelingData and oracle test projects get OCCT's source directory as assembly metadata, as
      the Draw-lite tests do, and fail when it is missing. `GeomTools_CorpusTests` reads the Curve2ds, Curves and
      Surfaces sections of the 37 `.brep` files of data/occ (18,108 curves and surfaces) and writes them back at each
      file's precision: the files come from older OCCT releases (15 digits in some, three-digit exponents, CRLF,
      directions normalised differently), so the test compares word for word, numbers within two units of the last
      digit; 905,752 of 910,971 words are identical. The byte-for-byte comparisons are against native OCCT 8 (2410).
234. **Oracle tests** (`tests/OcctNet.Oracle.Tests/GeomTools`, `GeomLProp`). GeomTools against native
      `BRepTools::Write` of an edge, a face, or an edge with a curve on a plane, each holding one random object built
      identically on both sides (decision 165): 1,000 3D curves of the ten kinds, 1,100 surfaces of the eleven kinds
      and 300 more offset surfaces, 1,000 2D curves (lines, circles, ellipses, B-splines and trimmed curves, the 2D
      kinds native OCCT binds); every section byte-identical, and the port, reading that file, writes what native OCCT
      writes after reading it (reading normalises directions again, so that text can differ from the first in the last
      digits); 2.7 MB in all. The corpus: native OCCT reads each data/occ file and writes it (F1), then reads F1 and
      writes it (F2); the port reads F1's geometry and writes it: F2's text, byte for byte, for all 37 files (18,108
      objects, 8,732,494 bytes). Surface properties against `BRepLProp_SLProps` on a BRepAdaptor_Surface: the port's
      LProp_SLProps3d on a GeomAdaptor_Surface, on random surfaces of every kind at random and boundary parameters with
      derivative orders 0 to 2, every query (and the curvatures asked first, before IsCurvatureDefined, after
      construction and after SetParameters): 262,620 values, all bit-identical; GeomLProp_SLProps on the Geom_Surface
      itself: within 2.4e-13 (54,391 of 68,030 bit-identical; the adaptor's caches evaluate B-splines differently).
      Not bound natively, so unit-tested only: the curve properties (GeomLProp_CLProps and its 2D and adaptor forms,
      which a unit test finds bit-identical to each other on analytic curves), GeomLProp::Continuity,
      GeomLProp_CurAndInf2d and the hashers.
## Phase 2: unit 3A, grid evaluation

GeomGridEval (Line, Circle, Ellipse, Hyperbola, Parabola, BezierCurve, BSplineCurve, OffsetCurve, OtherCurve,
Curve; Plane, Cylinder, Sphere, Cone, Torus, BezierSurface, BSplineSurface, OffsetSurface, SurfaceOfRevolution,
SurfaceOfExtrusion, OtherSurface, Surface) and Geom2dGridEval (the curve evaluators in 2D).

235. **Each evaluator is a sealed class with OCCT's API.** OCCT's evaluators are non-copyable value classes held in
      a `std::variant`; the port's are `public sealed class GeomGridEval_X`, built from a nullable geometry
      (`new GeomGridEval_Circle(Geom_Circle?)`; null gives empty results, as OCCT's null handle), with `Geometry()`
      and OCCT's `EvaluateGrid`, `EvaluateGridD1/D2/D3` and `EvaluateGridDN`, which return new 1-based
      `NCollection_Array1`/`NCollection_Array2` arrays as OCCT does. The bodies are OCCT's line for line, in OCCT's
      order of operations. As in OCCT, the parameters of a non-periodic B-spline direction must be ascending (the 3D
      evaluators skip locating a parameter below the current span's end); the class remarks say so.
236. **Interfaces for the evaluators** (additions): `GeomGridEval.ICurveEvaluator`, `GeomGridEval.ISurfaceEvaluator`
      and `Geom2dGridEval.ICurveEvaluator`, implemented by every evaluator. The dispatchers' `std::variant` is an
      interface field, null for `std::monostate`, and `std::visit` is a call through it (decisions 176 and 210 used an
      `object?` for data variants; here every alternative has the same five methods). Ported templates over "a grid
      evaluator" (ExtremaPC_GridEvaluator::BuildGrid, unit 4A) take a type parameter constrained on the interface.
      `GetType()` is `public new` (decision 98); the dispatchers' protected `initialization` overloads are private
      (the classes are sealed); GeomGridEval_Surface's `HasTransformation`/`GetTransformation` use `gp_Trsf?` for
      `std::optional<gp_Trsf>`. GeomGridEval_OtherCurve and the 2D one keep the adaptor object where OCCT keeps a
      reference (so OCCT's "must outlive the evaluator" caveat is gone); GeomGridEval_OtherSurface's
      `std::variant<const Adaptor3d_Surface*, handle<Geom_Surface>>` is one `object` field read with a type pattern.
237. **Result types.** OCCT's aliases `GeomGridEval::CurveD1..SurfD3` are written as `Geom_Curve.ResD1..D3` and
      `Geom_Surface.ResD1..D3` (decision 97); Geom2dGridEval's own `CurveD1..D3` structs, which have the same members,
      as `Geom2d_Curve.ResD1..D3`. Brace initialisations are object initialisers; results computed through `out`
      parameters are written straight into the array element (`ref aGrid.ChangeValue(i, j)`).
238. **Templates over functors** (the B-spline `evaluateGridCached`/`evaluateGridDirect` with their lambdas) are
      generic methods over a private interface with static abstract members (`CacheEval`, `DirectEval`), implemented
      by private structs per derivative order (decision 51's pattern), which the JIT specialises; the B-spline
      surface DN functor gets NU and NV as arguments where OCCT's lambda captures them. GeomGridEval.hxx's five
      `EvaluateGrid*Helper` templates (unused in OCCT) take a functor constrained on `GeomGridEval.IGridFunctor<T>`
      and share one body.
239. **Allocations.** As in OCCT, each call returns new arrays and builds one BSplCLib_Cache/BSplSLib_Cache (B-spline,
      Bezier), a basis GeomAdaptor_Surface and `Geom_OffsetSurface.Surface()` (offset surfaces), or a basis grid
      (offsets, swept surfaces); the inner loops allocate nothing (functor structs, `ref` writes, no boxing, no
      delegates). Span-based overloads were not added: the units that call these evaluators (Extrema's GenExt*,
      ExtremaPC, GeomBndLib, later the mesher) consume OCCT's arrays; an overload writing into a caller's span is
      a small addition if profiling asks for it.
240. **Smaller translations.** GeomGridEval_BezierSurface's V-isoline path ports `OCC_CATCH_SIGNALS` and
      `catch (const Standard_Failure&)` as `catch (Standard_Failure)` and falls back to the cache, as OCCT does. The
      swept evaluators keep OCCT's fields (including `myAxisDirection`, which OCCT never reads), at OCCT's defaults
      for a null geometry. GeomGridEval_OffsetSurface passes `Geom_OffsetSurface.OsculatingSurface()` as the
      osculating query (decision 212) and checks the equivalent surface with `ReferenceEquals`.
      GeomGridEval_OffsetCurve's DN keeps OCCT's missing empty-grid check (occt-issues.md): the port throws
      Standard_OutOfRange where OCCT's behaviour is undefined. The 3D OtherCurve calls the adaptor's
      `Value`/`D1`-`D3`/`DN`, the 2D one its `EvalD*`, as the two OCCT files do.
241. **Consistency tests replace the oracle** (no native binding exists). `GeomGridEval_CurveConsistencyTest`,
      `GeomGridEval_AnalyticSurfaceConsistencyTest`, `GeomGridEval_SurfaceConsistencyTest` and
      `Geom2dGridEval_ConsistencyTest` evaluate random geometry of every kind (directly, through the adaptors,
      trimmed, transformed, offset) and compare every grid value with point evaluation (Geom EvalD*/EvalDN and the
      adaptors), which the oracle tests found bit-identical to native OCCT. Each comparison has a stated
      expectation: bit-identical where OCCT's grid runs the point evaluation's algorithm, else a relative tolerance
      (1e-13 to 1e-12) with the reason, because OCCT's own grid is not bit-identical to its point evaluation there.
      - Bit-identical: lines; ellipse and hyperbola derivatives; parabola D2, D3 and higher; plane derivatives;
        Bezier curves and surfaces against the adaptors (the same single-span cache) and Bezier DN against Geom;
        B-spline curves on a run of two or more parameters in a span against the adaptor; B-spline surfaces at a
        lone grid point against Geom, and their D3 and DN against both; offset surfaces' D2 and D3 (and all of a
        Bezier basis against the adaptor); swept surfaces against Geom where the basis curve's grid is exact;
        OtherCurve/OtherSurface against what they wrap; transformed B-spline D3/DN against the transformed adaptor.
      - Last bits (up to 4e-15 for elementary formulas, 6e-14 for B-spline caches): OCCT's elementary formulas
        group terms differently from ElCLib/ElSLib (`R·(cos X + sin Y)` against `R cos X + R sin Y`; the centre
        added first; `u·(1/2F)` against `u/2F`); B-spline surface patches are evaluated through
        `BSplSLib_Cache.D0Local/D1Local/D2Local` on `(u - mid)/halfLen`, where GeomAdaptor_Surface uses the cache's
        `D0/D1/D2` (inverse span lengths) and Geom BSplSLib directly; a lone B-spline curve parameter in its span is
        evaluated directly on flat knots.
      OCCT's grid errors are compared separately and pinned by named tests (occt-issues.md).
242. **GTests.** All are ported (GeomGridEval 15 files, Geom2dGridEval 5, and GeomAdaptor_TransformedSurface's
      `GridEvalUsesTransformedGeometryOnlyOnce`, left out by unit 2A). Suites spread over several OCCT files are
      partial classes (decision 60): GeomGridEval_Surface_Test.cxx holds tests of the Sphere, Cylinder, Cone and
      Torus suites; GeomGridEval_Curve_Test.cxx and Geom2dGridEval_Curve_Test.cxx hold several suites each. Where a
      helper needed a test's name, the helper was renamed.

## Phase 2: unit 4C, conversion (29 October 2026)

Geom2dConvert (all of it: the statics, ApproxArcsSegments with PPoint, ApproxCurve, BSplineCurveKnotSplitting,
BSplineCurveToBezierCurve, CompCurveToBSplineCurve) and GeomConvert except SurfToAnaSurf (unit 5A): the statics of
GeomConvert.cxx and GeomConvert_1.cxx, ApproxCurve, ApproxSurface, CompCurveToBSplineCurve, the curve and surface
knot splitting and Bezier conversions, CompBezierSurfacesToBSplineSurface, CurveToAnaCurve with GeomConvert_ConvType
(in OCCT 8 an enum of Target, Simplest and MinGap only), FuncConeLSDist, FuncCylinderLSDist, FuncSphereLSDist and
Units.

243. **What a conversion algorithm keeps is frozen when handed out; what it builds per call is not** (as GC,
      decision 185). `GeomConvert_CompCurveToBSplineCurve.BSplineCurve()`, `GeomConvert_ApproxCurve.Curve()`, their 2D
      twins and `Geom2dConvert_ApproxArcsSegments.GetResult()` freeze the stored objects and return the same ones at
      every call. `CompCurveToBSplineCurve.Add` raises the degree of its current curve in place, as OCCT does, so it
      copies a frozen current curve first: a curve already handed out never changes. `GeomConvert_ApproxSurface.Surface()`
      is AdvApp2Var's surface, frozen when built (decision 197). `Arc(i)`, `Arcs`, `Patch(i, j)` and `Patches` of the
      Bezier conversions build new objects at each call, as OCCT does, and return them unfrozen.
244. **The statics return objects the caller owns, unfrozen.** Ported code (GeomLib, GeomFill, ShapeUpgrade, ...)
      modifies what `CurveToBSplineCurve`, `SplitBSpline*`, `Concat*` and `C0BSplineToC1BSplineCurve` return, as OCCT
      does. Where OCCT takes the result from a local algorithm object (CompCurveToBSplineCurve, ApproxCurve), the port
      reads it through internal accessors that don't freeze it (`InternalBSplineCurve()`, `InternalCurve()`): the
      object is discarded at once, so nothing is copied. `SurfaceToBSplineSurface` returns a copy of ApproxSurface's
      frozen surface. As in OCCT, a result can be one of the caller's own curves: a group of one curve in
      ConcatG1/ConcatC1, a curve without C0 knots in C0BSplineToArrayOfC1BSplineCurve, the input of a
      CurveToAnaCurve `Compute*` that already has the requested type (a frozen `BasisCurve()` when reached through a
      trimmed curve).
245. **The caller's geometry is never modified.** OCCT changes it in four places; the port changes copies, with the
      same values: Pretreatment of ConcatG1/ConcatC1 (2D and 3D) divides the weights of the caller's curves in place,
      and the port puts normalised copies into the caller's array, which the caller sees as OCCT's array does; with a
      single closed curve, ConcatG1/ConcatC1 call SetPeriodic and RemoveKnot on it, and the port on a copy;
      `C0BSplineToC1BSplineCurve(ref BS, ...)` gives BS a new curve (GeomConvert always does, through
      C0BSplineToArrayOfC1BSplineCurve and a concatenation; Geom2dConvert only when there are C0 knots);
      CompBezierSurfacesToBSplineSurface raises the degree of copies of the caller's patches.
246. **Signatures.** Arrays of curves are `NCollection_Array1<Geom_BSplineCurve>`; OCCT's handle outputs are
      `out NCollection_HArray1<Geom_BSplineCurve>` and `out NCollection_HArray1<int>`; the closed flags are
      `ref bool` (`ClosedG1Flag` in 3D, `ClosedFlag` in 2D, as OCCT names them). Accessors that fill a caller's array
      (`Arcs`, `Knots`, `Patches`, `Splitting`, ...) keep OCCT's form. CompBezierSurfacesToBSplineSurface's
      `Poles()`, `UKnots()`, `VKnots()`, `UMultiplicities()` and `VMultiplicities()` return `NCollection_HArray1/2`
      copies (decision 88: GeomFill and ShapeUpgrade pass them to Geom constructors). `ApproxArcsSegments.GetResult()`
      returns an `IBentoList<Geom2d_Curve>`, a PackedList of the frozen curves. CurveToAnaCurve's outputs that OCCT
      leaves unchanged on some paths are `ref` (`cf`, `cl` and `Deviation` of ComputeLine, ComputeCircle and
      ComputeEllipse, IsLinear's `Deviation`, GetCircle's circle, ComputeCurve's `theGap`, ConvertToAnalytical's result),
      the others `out` (assigned 0 where OCCT leaves them unset, decision 32).
247. **Small classes.** Geom2dConvert_PPoint is a public struct: exact `==`/`Equals`, OCCT's tolerant `operator==` as
      `IsEqual(in)`, its `operator!=` as internal `IsNotEqual`, `WithD1` for SetD1, OCCT's RealLast parameter from
      the parameterless constructor. The Func*LSDist classes are public sealed subclasses of
      math_MultipleVarFunction(WithGradient) with public `SetPoints`/`SetDir`, keeping the caller's point array as
      OCCT keeps the handle. GeomConvert_Units is a static class returning new curves.
248. **Evaluators and small translations.** The AdvApprox and AdvApp2Var evaluators of ApproxCurve and ApproxSurface
      are private nested classes holding their adaptor in a reassignable field, re-trimmed as OCCT's mutable handle is
      (decision 200). The law and reparameterise evaluators of the statics are private nested classes; the latter
      evaluates into a stack buffer of `order + 1` values (OCCT's single `double&`, occt-issues.md). `Dump` takes a
      TextWriter, and `ToString()` returns what it writes. GeomConvert's ConcatG1/ConcatC1 read
      `Geom2d_BSplineCurve::MaxDegree()`, as OCCT does (the same 25). GeomConvert uses GeomLProp::Continuity and
      Geom2dConvert its own private Continuity, as in OCCT. `calculateLines`, which OCCT never calls, is ported.
249. **Tests.** GTests: GeomConvert_Test (3), GeomConvert_CompCurveToBSplineCurve_Test (8),
      Geom2dConvert_CompCurveToBSplineCurve_Test (1) and TKGeomAlgo's Geom2dConvert_BSplineCurveToBezierCurve_Test (1;
      Geom2dAPI_Interpolate isn't ported, so its periodic curve is native GeomAPI_Interpolate's result for the same
      points, hard-coded at full precision). 72 tests of OCCT.NET's own cover every class and static, the rules of
      2501-2503, and each OCCT issue found (occt-issues.md).
250. **Oracle tests** (`tests/OcctNet.Oracle.Tests/{GeomConvert,Geom2dConvert,Convert}`; inputs built identically,
      decision 165): GeomConvert::CurveToBSplineCurve on every curve kind, whole and trimmed with both senses and every
      parameterisation (offsets through ApproxCurve), long RationalC1 arcs (400 3D, 300 2D, whole turns included),
      SplitBSplineCurve and all four SplitBSplineSurface overloads (throwing on both sides or neither),
      SurfaceToBSplineSurface on every surface kind, trimmed or not, and on offset surfaces (22 approximated by
      AdvApp2Var), GeomConvert_ApproxCurve (both constructors, C0 to C2, tolerances to 1e-11), and
      GeomConvert_CompCurveToBSplineCurve on curves cut into pieces, some reversed, added in random order with random
      After, WithRatio and MinM (2,312 added, 1,169 refused; BSplineCurve() read between adds). Geom2dConvert's
      CurveToBSplineCurve on lines, conics, B-splines and, through GeomAPI::To2d, parabolas and hyperbolas; its
      SplitBSplineCurve. ConvertOracleTests now calls the ported GeomConvert and Geom2dConvert instead of re-doing
      their post-processing (decision 91), with the long RationalC1 arcs it had to leave out. 1,232,862 values, all
      bit-identical. Not bound natively, so unit-tested only: ConcatG1, ConcatC1, the C0BSplineTo* functions, the
      knot splittings and Bezier conversions, CompBezierSurfacesToBSplineSurface, CurveToAnaCurve, the LSDist
      functions, Units, ApproxArcsSegments, ApproxSurface's other parameters, and the 2D Bezier and offset paths.
## Phase 2: unit 4A, surface extrema

Extrema's point/surface classes (Extrema_ExtPS, GenExtPS, ExtPExtS, ExtPRevS, GenLocateExtPS, FuncPSNorm, FuncPSDist,
POnSurfParams), curve/surface and surface/surface classes but Extrema_ExtCS (GenExtCS, GenLocateExtCS, FuncExtCS,
GlobOptFuncCS, GlobOptFuncConicS, GlobOptFuncCQuadric, ExtSS, GenExtSS, GenLocateExtSS, FuncExtSS), the enums
Extrema_ExtFlag, ExtAlgo and ElementType; OCCT 8's ExtremaPC package; and, for Extrema_GenExtPS's sphere tree,
NCollection_UBTree, NCollection_UBTreeFiller and `std_mt19937` (Foundation).

251. **Algorithm classes with OCCT's API** (decisions 117 and 220). Every class is a working object with OCCT's
      public `Initialize`/`Perform`/`SetFlag`/`SetAlgo`; results are returned by value (`const Extrema_POnSurf&` is a
      copy), `Points` and `TrimmedSquareDistances` take `out`. Surfaces and curves are adaptor references kept as
      OCCT keeps its pointers and references: the caller's adaptor must outlive the algorithm. Extrema_ExtPExtS and
      Extrema_ExtPRevS are Standard_Transient classes (OCCT holds them by handle) taking a
      GeomAdaptor_SurfaceOfLinearExtrusion/SurfaceOfRevolution directly; Extrema_ExtPS builds them as OCCT does
      (OCCT's copy of a temporary adaptor is a plain construction). By-value C++ members are allocated with the
      object; the fixed arrays of ExtPExtS (4) and ExtPRevS (8) are .NET arrays filled with `new Extrema_POnSurf()`.
      A `Perform` on a default-constructed GenExtCS, GenExtSS or ExtSS throws StdFail_NotDone where OCCT
      dereferences a null pointer. Extrema_FuncExtCS's `SquareDistances()`, `PointsOnCurve()` and `PointsOnSurf()`
      return the stored sequences, which GenExtCS edits as OCCT edits the non-const references.
252. **Extrema_POnSurfParams is a struct embedding an Extrema_POnSurf** (Extrema_POnSurf is a struct, decision 117,
      and C# structs can't derive). It forwards `Value`, `Parameter` and the internal `SetParameters`, and
      `POnSurf()` is C++'s slicing conversion. Its setters are internal with the twins `WithSqrDistance`,
      `WithElementType`, `WithIndices`; `new()` has OCCT's defaults (an indefinite point), `default` is zeros.
      Extrema_GenExtPS's `ComputeEdgeParameters`, which returns a reference to one of its arguments or to the member
      `myGridParam`, returns a copy (its callers copy it into an array).
253. **File-local code.** Extrema_GenExtPS.cxx's selector classes `Bnd_SphereUBTreeSelector(Min/Max)` are private
      nested classes holding the solution themselves (OCCT holds a reference to the caller's local and to the sphere
      array handle); `fillParams`, `LengthOfIso` and `CorrectNbSamples` are private static methods (`int&` is `ref`);
      `NCollection_Handle<NCollection_UBTree>` and the HArray handles are nullable references. The file-static
      functions of ExtPS, ExtPExtS, ExtPRevS, GenLocateExtPS and GenExtCS (`IsoIsDeg`, `ProjectPnt`,
      `IsOriginalPnt`, `GetPosition`, `PerformExtPElC`, `IsCaseAnalyticallyComputable`, `IsExtremum`, `CorrectTol`,
      `IsQuadric`, `IsConic`, ...) are private static methods of their class; GenExtSS's local class
      `Extrema_FuncDistSS` is a private nested sealed class. The small local arrays (`double aU[2]`) are
      `stackalloc` spans, hoisted out of the loop where C++ declares them inside (CA2014). GenExtSS's unread locals
      `PP1min`/`PP2min`/`PP1max`/`PP2max` are left out; its `myInit` is set and never read and kept with CS0414
      suppressed on that field (decision 221).
254. **Implicit gp_Vec to gp_Dir conversions are written out where C++ makes them.** Extrema_ExtPExtS keeps OCCT's
      `gp_Vec myDirection` and passes it where a gp_Dir is expected (`ProjectPnt`, `gp_Ax2(P, myDirection)`,
      `IsCaseAnalyticallyComputable`): each place is `new gp_Dir(myDirection)`, which normalises again as C++'s
      converting constructor does, while `gp_Vec(E, Pe) * gp_Vec(myDirection)` and `myDirection.IsParallel(...)` stay
      gp_Vec operations (gp_Vec::IsParallel, not gp_Dir's). Extrema_ExtPRevS's `N.IsParallel(OO, ...)` and
      `N ^= OO` (a gp_Dir with a gp_Vec) convert OO likewise. The oracle tests confirm the choice (bit-identical).
255. **NCollection_UBTree's bounding type is the interface `INCollection_UBTreeBnd<T>`** (`Add(in T)`, `IsOut(in T)`,
      `SquareExtent()`), constrained `where T : struct`. Bnd_Sphere and Bnd_Box implement it, `Add` explicitly over
      their internal mutator: through the interface on a boxed copy it changes only the copy, so callers' values stay
      immutable, while the tree changes its own nodes' values in place through the constraint as C++ does.
      `Selector` is an abstract class (its `Reject`, const in C++, is not readonly: OCCT's own selectors cast the
      const away to lower their distance); `TreeNode` is a sealed class holding its two children in an array, with
      OCCT's `Gemmate`/`Kill` internal. The allocators are dropped (rules, section 9); the move constructor and
      move assignment are a constructor taking a tree and `Move(tree)`, which empty the source. `Clear` also resets
      the last node, which OCCT leaves dangling. `Select` keeps OCCT's explicit stack (an array doubled as needed).
      Rejected: a third type parameter of static traits (the tree would no longer read as
      `NCollection_UBTree<int, Bnd_Sphere>`), and making Bnd's `Add` public.
256. **NCollection_UBTreeFiller is IDisposable, and shuffles with `std_mt19937`.** OCCT's destructor fills the tree
      with the objects not yet added; `Dispose` does it (`using var` where C++ relies on the scope). The shuffle
      decides the tree's shape and so, on ties, which sphere a selector keeps: `std_mt19937` (Foundation/Standard, new;
      decision 55's naming) is the standard Mersenne Twister, checked against the C++ standard's 10000th value, so the
      port's trees are OCCT's. It is a class, as in decision 58. `CheckTree` writes to a TextWriter in `Sprintf`'s
      format.
257. **Failure values** (decision 221). Where Extrema_FuncPSDist's `Value`/`Values` and the GlobOptFunc classes'
      `Value` return false outside the parameter range without writing `F`, the port writes 0. Extrema_GenLocateExtPS's
      default tolerances `Precision::PConfusion()` are the constant `1e-7 * 0.01` (decision 129).
258. **ExtremaPC is OCCT's API on classes.**
      - `ExtremaPC.hxx` is `public static partial class ExtremaPC` (decision 50): the `THE_*` constants (`const`;
        `THE_DEFAULT_TOLERANCE` and `THE_PARAM_TOLERANCE` written as the literals 1e-7 and 1e-9), the `Status` and
        `SearchMode` enums, `ExtremumResult`, `Result`, `Config` and `AddEndpointExtrema`. The alias
        `ExtremaPC::Domain1D` is `MathUtils.Domain1D` (decision 186). `ExtremumResult` is a struct with public fields
        (decision 54; `new()` has `IsMinimum = true`, `default` false); `Config` is a class with public fields and a
        copy constructor (decision 53), `std::optional<Domain1D>` a `Domain1D?`.
      - `ExtremaPC.Result` is a sealed class shared as OCCT shares it: each evaluator returns a reference to its own
        buffer, overwritten by the next `Perform`, and the port returns that object. `Status` and
        `InfiniteSquareDistance` have public getters and internal setters, `Clear()` is internal, the extrema
        (`NCollection_DynamicArray`) an internal field read through `NbExt()`, the indexer, `MinIndex`/`MaxIndex` and
        `Min/MaxSquareDistance`. `ExtremaPC_Curve`'s static not-done result is one shared instance.
      - The evaluators (Line, Circle, Ellipse, Hyperbola, Parabola, BezierCurve, BSplineCurve, OffsetCurve,
        OtherCurve) are sealed classes implementing `ExtremaPC.ICurveEvaluator` (an addition: `Value`, `Perform`,
        `PerformWithEndpoints`; the constraint of the generic `AddEndpointExtrema<CurveEvaluator>`).
        `ExtremaPC_Curve`'s `std::variant` is an `ICurveEvaluator?` field (decision 236's pattern); `mutable` members
        are ordinary fields; offset and other curves keep the caller's adaptor (decision 220).
      - `ExtremaPC_GridEvaluator.BuildGrid<GridEval>` is constrained on `GeomGridEval.ICurveEvaluator` (decision 236);
        the B-spline grid reads `InternalKnots()` and its parameters ascend (decision 235).
        `ExtremaPC_DistanceFunction` is a readonly struct functor (`MathUtils.IScalarFunction`,
        `IScalarFunctionWithDerivative`) for `MathRoot.NewtonBounded` (decision 51). The `std::sort` of candidates is
        `Span.Sort`, unstable like std::sort: candidates of equal estimated distance may be visited in another order
        than MSVC's. Lambdas with captured state are private methods; `std::clamp` is written out (`Math.Clamp`
        throws when min > max).
259. **Tests.** Every ExtremaPC GTest is ported (12 files, 305 tests; the random tests' `std::mt19937` is exact but
      `uniform_real_distribution` is a linear mapping, decision 60), and NCollection_UBTree's (9 tests; the file's
      NCollection_EBTree tests wait for EBTree), with tests of the filler, the parent links and `std_mt19937`. OCCT has
      no GTests of the surface extrema but GeomAPI_ProjectPointOnSurf's Bug867, ported on the Extrema_ExtPS path
      GeomAPI takes. Unit tests (`Extrema_SurfacePSTests`, `Extrema_SurfaceCSSSTests`) check the rest against known
      answers, the elementary solvers, finite differences and dense sampling; the generic solvers return stationary
      points refined from a grid (interior, or clamped by bounded solvers), so their tests put the extremum inside the
      domain (bumps).
260. **Oracle coverage** (`tests/OcctNet.Oracle.Tests/Extrema/ExtremaSurfaceOracleTests.cs`; every value
      bit-identical). Extrema_ExtPS is not bound; its callers are, and the port's side mirrors their call paths:
      - Extrema_ExtPS through GeomAPI_ProjectPointOnSurf on every surface kind (elementary, revolution, extrusion,
        Bezier, B-spline, offset, trimmed), whole (infinite bounds become ±1e10) or on a sub-range, OCCT's default
        or a random tolerance, gradient or tree algorithm, the three flags, one projector reused for up to three
        points (GeomAPI loads a GeomAdaptor_Surface on the range, then calls SetAlgo, Initialize and Perform on a
        default Extrema_ExtPS): 4,712 extrema of 2,186 projections, 34,830 values.
      - Extrema_ExtPS through BRepExtrema_DistShapeShape between a vertex and a face (BRepExtrema_ExtPF: the whole
        surface as BRepAdaptor_Surface, which is a GeomAdaptor_TransformedSurface, the face's UV bounds and resolution
        tolerances, MINMAX and Grad): 347 solutions, 2,082 values. The port reads BRepTools::UVBounds and the
        tolerance natively, and builds the surface BRepLib_MakeFace stores (it trims offsets of swept surfaces:
        without that, 17 values of offset faces differed in the last bits).
      - Extrema_GenExtCS through DistShapeShape between an edge and a face, on Bezier and smooth B-spline curves and on
        conics Extrema_ExtCS doesn't solve analytically, so that GenExtCS's general, conic and quadric paths all run
        (Extrema_ExtCS's generic path mirrored: Initialize(S, 10, 10), 12 or 13 samples, hyperbolas within ±20,
        AddSolution's periods; the edge's range read natively, since BRepLib_MakeEdge moves a periodic curve's range
        into the period): 580 solutions, 5,800 values.
      - Extrema_ExtSS (ExtElSS and GenExtSS) through DistShapeShape between two faces (BRepExtrema_ExtFF: Initialize
        with face 2, Perform with face 1), surface 2 turned so that the normals at two random points are opposite and
        moved to put them face to face, so that solutions fall inside both faces: 103 solutions, 1,133 values.
      Each native solution inside the shapes must be one of the port's extrema. Not compared natively: the
      Extrema_*Locate* classes, GenExtSS on its own, ExtPExtS/ExtPRevS on their own and ExtremaPC (no binding); the
      unit tests and ExtremaPC's comparison GTests (against Extrema_ExtPC) cover them.
261. **For later units.** 4B (Extrema_ExtCS): `Extrema_GenExtCS` has OCCT's API (`new()`, `Initialize(S, NbU, NbV,
      Tol2)` and the bounded overload, `Perform(C, NbT, Tol1)` and `Perform(C, NbT, tmin, tsup, Tol1)`, `IsDone`,
      `NbExt`, `SquareDistance`, `PointOnCurve`, `PointOnSurface`), and `Extrema_ExtPS` projects the sharp points;
      `GenExtCS_through_BRepExtrema_DistShapeShape_EdgeFace` mirrors ExtCS's generic path and can compare the port's
      Extrema_ExtCS directly once it exists. NCollection_UBTree (with Bnd_Box) is ready for BRepClass3d, Sewing,
      ShapeAnalysis and IntPolyh. 5A/6A (GeomLib, ProjLib_CompProjectedCurve, ProjLib_PrjResolve, GeomAPI) use
      Extrema_ExtPS, GenLocateExtPS and GenExtPS on adaptors that must outlive them; a port of
      GeomAPI_ProjectPointOnSurf should be the oracle test's port side (a default Extrema_ExtPS, SetAlgo before each
      Initialize, the flag kept across Init).

## Phase 2: unit 5A, GeomLib and Approx (29 October 2026)

GeomLib (all of it: GeomLib.cxx's statics, GeomLib_Check2dBSplineCurve, CheckBSplineCurve, CheckCurveOnSurface,
DenominatorMultiplier, Interpolate with InterpolationErrors, IsPlanarSurface, LogSample, MakeCurvefromApprox, PolyFunc,
Tool); the rest of Approx (Approx_ComputeCLine.gxx with its instantiations FitAndDivide and FitAndDivide2d, Curve2d,
Curve3d, CurveOnSurface, CurvlinFunc, CurvilinearParameter, SameParameter, SweepFunction, SweepApproximation); and
GeomConvert_SurfToAnaSurf. With unit 2C, all of Approx.

2801. **GeomLib's statics never change the caller's geometry, and return objects the caller owns** (decisions 244
      and 245). Where OCCT writes a handle argument, the port takes it by `ref` and assigns a new object:
      `ExtendSurfByLength` raises the degree of, and segments, the caller's B-spline surface (or ApproxSurface's frozen
      one) in place in OCCT, and the port changes a copy; `CancelDenominatorDerivative` (used only by a Draw command)
      changes a copy as soon as a direction is requested; `AdjustExtremity` and `buildC3dOnIsoLine` change the result
      of GeomConvert::CurveToBSplineCurve, copied first if frozen; `ExtendCurveToPoint` reads its concatenation's curve
      through `InternalBSplineCurve()` (unfrozen: the concatenation is discarded). As in OCCT, `SameRange` returns the
      caller's own curve when the ranges already agree, and `BuildCurve3d`'s plane branch builds the 3D curve of the
      adaptor's curve (the basis of a trimmed pcurve, as Geom2dAdaptor_Curve loads it). Signatures: OCCT's outputs
      left unchanged on some paths are `ref` (`SameRange`'s and `BuildCurve3d`'s curve, `NormEstim`'s normal), the
      others `out` (`Inertia`, `AxeOfInertia`, `RemovePointsFromArray`, `DensifyArray1OfReal`, the `EvalMax*`
      distances, `IsClosed`, and `isIsoLine`, which assigns false and 0 before any early return, decision 32);
      handles that may be null are nullable (`GTransform`, `buildC3dOnIsoLine`). `SameRange` throws Standard_Failure
      on a null curve, as OCCT does. GeomLib is a `static partial class`.
2802. **GeomLib's file-local code and small classes.** ComputeLambda, ExtendKPart, CanBeTreated, CheckIfKnotExists,
      AddAKnot, BuildFlatKnot, FunctionMultiply, CancelDenominatorDerivative1D and CompareWeightPoles are private
      statics (handle references `ref`, outputs `out`); the evaluators `GeomLib_CurveOnSurfaceEvaluator` (an
      AdvApprox_EvaluatorFunction holding the trimmed adaptor in a field) and `law_evaluator` (a
      BSplSLib_EvaluatorFunction, writing 0 with its error code where OCCT leaves the result unset) are private nested
      classes. OCCT's raw pointers into math_Matrix, math_Vector and NCollection arrays are their spans; the
      two-element `extrap_mode` is a `stackalloc` hoisted out of its loop. `GeomLib_DenominatorMultiplierPtr` is a
      plain reference. GeomLib_DenominatorMultiplier keeps the caller's surface (OCCT's handle; it only reads it);
      GeomLib_LogSample and GeomLib_PolyFunc derive from math_FunctionSample and math_FunctionWithDerivative.
      **GeomLib_MakeCurvefromApprox** keeps the caller's AdvApprox_ApproxAFunction, which OCCT copies by value and
      never changes, and builds a new, unfrozen curve at each call.
2803. **GeomLib_IsPlanarSurface** is a sealed class computed in its constructor. `Plan()` returns the gp_Pln by value
      and throws StdFail_NotDone when the surface isn't planar, as OCCT does. The two file-static `Controle` functions
      are private static overloads (the points overload takes the plane by `ref`). The implicit gp_Vec to gp_Dir
      conversions are written out (decision 254).
2804. **GeomConvert_SurfToAnaSurf** is a working object with OCCT's public `Init`, `SetConvType` and `SetTarget`
      (decision 21). It keeps the caller's surface and never changes it: B-spline and Bezier sub-ranges are segmented
      on a `Copy()`, so a frozen input works. `ConvertToAnalytical` returns new, unfrozen surfaces the caller owns, or
      null (decision 244). CurveToAnaCurve is called with `out cf, cl` and `ref theGap` (decision 246), the gap locals
      starting at 0 where OCCT leaves them uninitialised; `GetCircle(ref gp_Circ, ...)` starts from `new gp_Circ()`.
      GetCylByLS minimises GeomConvert_FuncCylinderLSDist with math_PSO then math_Powell, as OCCT does (only `Value`
      is called, so FuncCylinderLSDist's gradient error doesn't matter). `ComputeGap`'s default `RealLast()` is the
      constant `double.MaxValue`; the fixed arrays `newSurf[5]`, `dd[5]` and `aSTypes[5]` are an array and two spans.
2805. **GeomLib_CheckCurveOnSurface is a working object with OCCT's API.** It keeps the caller's adaptors, which must
      outlive it; `Perform(null)` sets error 1; the default `theTolRange` is `1.0e-7 * 0.01` (decision 257).
      OSD_ThreadPool is `Parallel.For` over `min(intervals, Environment.ProcessorCount)` threads, each with its own
      shallow-copied adaptors, taking the elements `Lower + t, Lower + t + n, ...` as OCCT's launcher does; each
      element is computed on its own, so parallel and sequential results are identical (tested). An
      AggregateException wrapping a Standard_Failure is rethrown, so the outer catch sets status 3 as in OCCT. The
      file-local TargetFunc and Local are private nested classes (`operator()` is `Run`); `Derive`'s optional
      `double*` is a flag and an `out`; `ElementaryMaximum`'s lambdas are private methods; `Value` writes 0 on
      failure (decision 221); FillSubIntervals' optional array pointer is a nullable array.
2806. **GeomLib_Tool is a public static class.** `Parameter` and `Parameters` have `out` results, assigned 0 also when
      the geometry is null (decision 32). ComputeDeviation's optional pointers are overloads (`(curve, f, l, double
      start, int iters = 100)`, `(..., int iters, ref prm, ref pnt, ref vec, ref line)` leaving each output unchanged
      where OCCT doesn't write it, `(curve, f, l, int nbSub, int iters = 10)` and `(..., int iters, ref prm)`), chosen
      by C# as C++ chooses them. FuncSolveDeviation is a private nested class keeping the caller's
      Geom2dAdaptor_Curve, which OCCT copies and only evaluates.
2807. **The tangent checks and the interpolation.** GeomLib_CheckBSplineCurve's and GeomLib_Check2dBSplineCurve's
      `FixTangent` changes the checked curve in place, as OCCT does, but is internal: its OCCT callers
      (IntTools_FaceFace, GeomInt_IntSS, TopOpeBRepTool) pass curves they have just built, and a frozen curve throws
      Standard_ImmutableObject (tested). The public API is `FixedTangent`, which returns a new, unfrozen copy, or null
      when nothing needs fixing; `NeedTangentFix` has `out` parameters. GeomLib_Interpolate's `Curve()` freezes the
      stored curve and returns it at each call (decision 243), null on failure; `Error()` is GeomLib_NoError after a
      success, where OCCT leaves it uninitialised (occt-issues.md). GeomLib_InterpolationErrors keeps OCCT's names.
2808. **Approx_Curve2d, Approx_Curve3d and Approx_CurveOnSurface are working objects whose results are frozen when
      handed out** (decision 243): `Curve()`, `Curve3d()` and `Curve2d()` freeze the stored B-spline and return the
      same object at each call; the internal `InternalCurve()`, `InternalCurve3d()` and `InternalCurve2d()` return it
      unfrozen for ported code that discards the algorithm (ProjLib, BRepLib). The AdvApprox evaluators are private
      nested classes with a reassignable adaptor field (decision 248). Approx_Curve2d's constructor calls
      `C2D.Trim(...)` and discards the result, as OCCT does. Approx_Curve3d's `Dump` throws StdFail_NotDone without a
      result, where OCCT dereferences a null handle. Approx_CurveOnSurface isn't sealed, so OCCT's protected
      `isIsoLine` and `buildC3dOnIsoLine` stay protected (`out` outputs assigned first, decision 32); its
      `buildC3dOnIsoLine` copies a frozen conversion result before reversing and re-knotting it (decision 244); its
      deprecated constructor is `[Obsolete]`.
2809. **Approx_SameParameter.** Its private struct `Approx_SameParameter_Data` is a private nested class owning four
      `double[1000]` arrays where OCCT has stack arrays; `Swap` swaps the arrays as OCCT swaps the pointers. OCCT saves
      and restores the curve on surface by copying it (`ACS = aData.myCOnS; ... aData.myCOnS = ACS`); only its 2D curve
      changes in between, so the port keeps that curve and loads it again. The evaluator keeps references to the
      caller's poles and flat knots (OCCT's const references) and evaluates into a two-element stack buffer, as OCCT
      does (AdvApprox asks for at most the first derivative: the continuity is at most C1). `ComputeTangents` takes
      `ref` arguments left unchanged on failure; `CheckSameParameter`'s squared distance and `ProjectPointOnCurve`'s
      outputs are `out`. `Curve2d()` freezes only curves the algorithm built: when it falls back to
      Geom2dAdaptor::MakeCurve of the input, the result can be the caller's own pcurve, returned unfrozen as OCCT
      returns it; `InternalCurve2d()` never freezes. `Curve3d()` returns the caller's adaptor; `CurveOnSurface()` the
      algorithm's shallow-copied adaptor, or null when the computation stopped early (OCCT's null handle).
2810. **Approx_ComputeCLine.gxx is one non-generic body.** OCCT instantiates it twice, as Approx_FitAndDivide and
      Approx_FitAndDivide2d, both over AppCont_Function, and the `.gxx` calls no line tool (AppCont_LeastSquare takes
      the AppCont_Function itself), so the static tool interface decision 202 anticipated isn't needed. The body is a
      public abstract class `Approx_ComputeCLine` with protected constructors; the two instantiations are sealed
      subclasses forwarding OCCT's constructors and defaults. It is a working object (decision 21: `Perform` and the
      `Set*` methods are public); `Value(i)` returns a copy sharing the poles, as OCCT's by-value return does
      (decision 141), and where the C++ copies a MultiCurve into a member or a sequence the port writes `new(x)`.
      `Parameters` and `Error` have `out` parameters, the private `Compute` takes its tolerances by `ref`. Compute's
      write of `aPrevCurve`/`aPrevTol*` just before a `break` is left out: they are never read again.
2811. **Approx_CurvlinFunc is a sealed Standard_Transient working object.** It keeps the caller's adaptors as OCCT
      keeps its handles; `Trim` replaces them with trimmed adaptors and never changes the caller's. OCCT's const_cast
      writes of `myPrevS`/`myPrevU` are plain field writes; the private `Init(C, Si, Ui)` has `out` arrays;
      `findfourpoints` and `cubic` are private statics over stackalloc spans; Trim's case 3 `[[fallthrough]]` is
      `goto case 2`; EvalCurOnSur's second, unread computation of `d2U_dS2` is left out. Two OCCT errors in the case
      of two curves on two surfaces are reproduced and marked (occt-issues.md).
2812. **Approx_CurvilinearParameter, Approx_SweepFunction and Approx_SweepApproximation.** Approx_CurvilinearParameter's
      `Curve3d()`, `Curve2d1()` and `Curve2d2()` freeze the stored B-splines and return the same objects at each call
      (null without a result); its three AdvApprox evaluators are private nested classes; OCCT leaves `myMaxError2d2`
      uninitialised in the one-surface constructor, 0 in the port. Approx_SweepFunction is a public abstract
      Standard_Transient whose optional members (D1, D2, Resolution, BarycentreOfSurf, MaximalSection,
      GetMinimalWeight) throw Standard_NotImplemented as in OCCT; `SectionShape` and `Resolution` use `out`.
      Approx_SweepApproximation is a sealed working object: its surface and 2D accessors (SurfPoles, SurfWeights,
      SurfU/VKnots, SurfU/VMults, Curve2dPoles, Curves2dKnots, Curves2dMults) return its own arrays (shared: don't
      modify), `Surface` and `Curve2d` copy into the caller's arrays; `Eval` writes into a `Span<double>`; AdvApprox's
      `Poles()` and `Poles1d()` return copies in the port (decision 146), so they are read once before the loops where
      OCCT reads the handle per element (the same values); `Dump` writes to its TextWriter the line end and
      " Not Done " that OCCT writes to std::cout. Every `Dump` takes a TextWriter and `ToString()` returns what it
      writes.
2813. **Oracle coverage** (all values bit-identical; inputs built identically on both sides, decision 165):
      - GeomLib_IsPlanarSurface (bound) on 2,000 random surfaces: elementary surfaces, planar and nearly planar
        B-splines and Beziers (noise 0 to 1e-2, some rational), random B-splines, revolutions, extrusions, trimmed and
        offset surfaces (native offsets built with `isNotCheckC0`), OCCT's default tolerance or 1e-9 to 1e-1; IsPlanar
        and the plane's location and axes: 11,408 values (784 planar).
      - Approx_SameParameter through BRepLib::SameParameter(edge, tolerance, newTol, false) on 600 edges (a 3D curve
        and a pcurve on a plane, cylinder or B-spline surface: rational circles under an angular circle, lines and
        generators run at uneven speed, B-spline isolines, perturbed B-spline images; tolerances 1e-7 to 1e-3),
        mirroring BRepLib's path (adaptors on the edge range, BRepLib's ComputeTol copied, its choice of pcurve and
        tolerance): the pcurve's structure and values, its range, the new and the edge tolerance: 144,638 values. Not
        mirrored: BRepLib's C0 pcurve branch (Approx_CurvilinearParameter) and its failure branch (BRepCheck), never
        reached.
      - GeomLib::BuildCurve3d (To3d, the isoline path and AdvApprox) through BRepLib::BuildCurve3d on 440 edges with
        only a pcurve, on every surface kind (on planes, BRepLib's own To3d call on lines, circles, ellipses and
        B-splines, trimmed or not; elsewhere isolines as lines or degree-1 B-splines in either sense, diagonal lines,
        circle arcs and free B-splines), C0 to C2, degrees 8 to 14, MaxSegment 0 (BRepLib's evaluateMaxSegment,
        copied), 10 or 30: the 3D curve's structure and D1 at 11 points: 61,524 values.
      - GeomLib::SameRange through BRepLib::SameRange on 600 edges (a pcurve on a plane, a line, circle, ellipse or
        B-spline, trimmed or not, whose range is moved and possibly stretched), throwing on both sides or neither:
        43,671 values.
      - GeomLib_CheckCurveOnSurface through BRepTools::EvalAndUpdateTol (its path mirrored: range clamped to the
        non-periodic curves, the adaptors, the 22-sample fallback, the edge tolerance as the floor) on 600 cases (plane
        and cylinder with perturbed B-splines, conics for the trigonometric path, lines): 600 values.
      Not bound, so unit-tested only: GeomLib's other statics, GeomLib_Tool, the Check classes, Interpolate, the
      FitAndDivide classes, CurvlinFunc, CurvilinearParameter, the sweep classes, Curve2d, Curve3d and
      GeomConvert_SurfToAnaSurf (ShapeAnalysis_CanonicalRecognition isn't bound, and ShapeCustom doesn't reach it).
2814. **Tests.** OCCT's only GTest for these packages, GeomLib_CheckCurveOnSurface_Test (7 tests), is ported. 82 tests of
      OCCT.NET's own cover every class: known answers (curves on planes and cylinders, isolines, conic and helix
      approximations, quadrics and tori recognised from their B-spline conversions and from swept surfaces, planar
      surfaces of every kind), the copies made instead of OCCT's in-place changes (frozen inputs), CheckCurveOnSurface
      run in parallel and sequentially, and each OCCT issue that a test can reach, pinned. Not reached by any test:
      SurfToAnaSurf's Gauss-field cylinder path (`TryCylinderByGaussField`).
2815. **For later units.** 6A (ProjLib): `new Approx_CurveOnSurface(C2D, Surf, First, Last, Tol)` then
      `Perform(MaxSegments, MaxDegree, Continuity, only3d, only2d)`, results through `InternalCurve3d()` and
      `InternalCurve2d()` (unfrozen), `MaxError3d()`, `IsDone()`, `HasResult()`; `new Approx_FitAndDivide2d(F,
      degmin, degmax, tol3d, tol2d, cutting)` then `SetMaxSegments`, `SetInvOrder`, `SetHangChecking`, `Perform`,
      `Value(i)`, `Parameters(i, out, out)`, `Error(i, out, out)`; `GeomLib.SameRange(..., ref newCurve)`; GeomLib_Tool
      for ProjectOnPlane. The oracle tests' edge builders (SameRange, BuildCurve3d, SameParameter) can be reused for
      BRepProj and To2d comparisons. Phase 3 (BRepLib): BuildCurve3d's plane branch calls `GeomLib.To3d` on the stored
      pcurve itself; SameParameter's C0 B-spline pcurve branch needs `GeomConvert.C0BSplineToC1BSplineCurve` and
      Approx_CurvilinearParameter; the oracle tests' port sides (`ComputeTol`, `evaluateMaxSegment`) become BRepLib's
      own code then.
