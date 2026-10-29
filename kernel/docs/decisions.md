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

## Phase 1: MathUtils and the new maths packages (29 October 2026)

OCCT 8's header-only packages MathUtils, MathLin, MathPoly, MathRoot, MathInteg, MathOpt and MathSys: C++
namespaces of function templates over duck-typed functors, returning result structs.

201. **A C++ namespace is a `public static partial class`** (`MathRoot.Brent(...)`), one C# file per header. Each
     file's `using namespace MathUtils;` is `using static Occt.MathUtils;`. Nested namespaces (`detail`,
     `Internal`) are nested `internal static partial class`es. Names that C++ code reaches through another
     namespace's `using` (`MathRoot::ScalarResult`) are written with their own class (`MathUtils.ScalarResult`).
     Where Standard and MathUtils both have a function (`IsEqual`, `Sign`), ported code qualifies `MathUtils.`.
202. **Functor templates are generic methods constrained on interfaces**, one interface per member set the C++
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
203. **The lambda wrappers (`MakeScalar`, `MakeVector`, `MakeSystem`, ...) hold delegates** of package-defined
     delegate types with C++'s signatures (`bool ScalarLambdaFunction(double, out double)`), so a C# lambda reads
     `MathUtils.MakeScalar((x, out y) => { y = x * x - 2.0; return true; })`. The library functors
     (`Polynomial`, `Rosenbrock`, `Sum<F, G>`, ...) are sealed classes; they copy the vectors and matrices they
     are given, as C++ does. Functor classes local to a C++ function template are private generic classes or
     structs of the package.
204. **Configurations and options are classes**: `Config`, `BoundedConfig : Config`, `NDimConfig : Config`,
     `PSOConfig : NDimConfig`, `GlobalConfig`, `IntegConfig`, `KronrodConfig`, `LMConfig`, `NewtonOptions`,
     `MinBracketOptions`, ..., with public fields as in C++ and a copy constructor. C++ passes derived
     configurations as `const Config&`; C# structs can't derive. A C++ copy is written `new(theConfig)`; a
     default argument `const Config& theConfig = Config()` is `Config? theConfig = null` and
     `theConfig ??= new Config();`. Rejected: structs with duplicated fields and conversion operators to the
     base (slicing), which don't scale to three levels.
205. **Result structs (`ScalarResult`, `PolyResult`, `VectorResult`, `IntegResult`, `NewtonResultN2`, ...) are
     structs with public fields**, as the C++ aggregates. `std::optional<double>` is `double?`,
     `std::optional<math_Vector>` a nullable reference, `size_t` is `int`. Their C++ default status is
     NotConverged, so they have an explicit parameterless constructor and ported code writes `new()`;
     `default(T)` has status OK. `explicit operator bool` is kept, and `operator true`/`false` make
     `if (aResult)` work as in C++. They have no `Equals`/`ToString`: they are transient solver outputs holding
     references. `MathUtils.Domain1D`/`Domain2D`, which are values, get the value-type members of section 3 of
     the rules but keep C++'s public fields. Results that hold containers (`MathRoot.MultipleResult`,
     `AllRootsResult`, `MathOpt.PSOSeedParticle`) share them when copied, where C++ copies them; ported helpers
     that change them take them `ref`.
206. **`std::array<T, N>` is `std_array2/3/4<T>`** (new file `Standard/std_array.cs`): `[InlineArray]` structs,
     copied as a whole, indexed from 0 with a range check, with `a[i][j]` working on nested arrays and
     `std_arrayN<T>.Of(...)` for brace initialisation. Not OCCT API, but OCCT 8 uses `std::array` widely.
     Larger one-off sizes are package-local inline arrays (`MathPoly.RootArray<T>` of 20, a 5-element row in
     MathSys).
207. **Templates over an integer (`NewtonResultN<N>`, `NewtonBoundsN<N>`)** are ported per instantiation OCCT
     uses: `NewtonResultN2`, `N3`, `N4`. The 2D/3D/4D functors take `ref std_arrayN<double> theF` and
     `ref std_arrayN<std_arrayN<double>> theJ`, so their bodies read as the C++ (`theJ[0][1] = ...`).
208. **Output parameters** are `out` when every path writes them, and `ref` when C++ may return without writing
     them, so the caller's value survives as in C++ (`MathUtils.CentralDifference`'s derivative,
     `BrentAlongCoordinate`'s `theFx`, MathSys's small solvers). Writes through `*aResult.Value` into an optional
     go through a temporary.
209. **`MathUtils.RandomGenerator` is a class**: its only purpose is its changing state, which a struct would lose
     to defensive copies. `MathOpt.PSOStats` is a class too (C++ fills it through an optional pointer), with
     `Assign`.
210. **`std::complex<double>` is `System.Numerics.Complex`** (same principal branches; division may differ in
     the last bits). `std::sort` is `Span.Sort` (not stable, like std::sort). `goto` is kept where OCCT uses it.
211. **Tests**: one xUnit class per GTest suite (a suite spread over two files is a `partial class`). C++
     `rand()` in the MathLin tests is the MSVC runtime's generator. The MathLin_EigenSearch random test uses a
     faithful `std::mt19937` but simple mappings in place of `std::uniform_*_distribution` (whose output differs
     between C++ libraries), so its random matrices differ from OCCT's; no check depends on the values.
