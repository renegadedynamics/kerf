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
