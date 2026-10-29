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

## Phase 1: Bnd, BVH and the NCollection vectors (29 October 2026)

23. **NCollection_Vec2/3/4 and NCollection_Mat3/4 are generic structs** (`where T : unmanaged, INumber<T>`) over
    `[InlineArray]` storage, so `v[i]` and `myMat[i]` read as in the C++. They follow the value-type rules: the
    non-const `x()` and `operator[]` become an indexer with an internal setter and an internal `ChangeData()` span;
    `GetData()` becomes `CopyTo`; converting constructors become static `From<TOther>`. Mat3/Mat4 default to the
    identity (`new()`), `default` is zero. OCCT's float/double specialisation of `Vec3`/`Vec4 /= scalar`
    (multiply by the inverse) is kept. `Mat3.Inverted(ref, out)` takes `ref` because OCCT leaves the output
    untouched for a singular matrix.
24. **BVH is generic over the element type and the vector type**, `BVH_Box<T, TVec>` with `TVec` one of the
    NCollection vectors, instead of OCCT's `BVH_Box<T, N>`: C# has no integer type parameters, and the vector type
    fixes the dimension. The vectors implement `INCollection_Vec<T, TSelf>` (static abstract `Length`, `Create`,
    operators, component access); generic code writes a component through an internal `Unsafe` helper. The JIT
    specialises every instantiation, so there is no interface dispatch in the inner loops. Rejected: concrete
    classes per (T, N), six copies of every builder. `BVH_Box<double, 3>` is spelled
    `BVH_Box<double, NCollection_Vec3<double>>`; the tests have global aliases for OCCT's typedefs (`BVH_Vec3d`).
    N = 1, declared but never instantiated by OCCT, is not ported.
25. **BVH template specialisations and int template arguments**: the binary tree is `BVH_Tree<T, TVec>` and the quad
    tree `BVH_QuadTree<T, TVec>` (OCCT's arity tags dropped); `Child<K>(n)` is `Child(K, n)`; the binned builder's
    `Bins` is its constructor's last argument; `BVH_Transform<T, N>` exists only for N = 4, so it is
    `BVH_Transform<T>`; the 3D-only `BVH_Box::Transform`/`Transformed` are extension methods. OCCT's
    `BVHSetType = void` is a generic overload without the set type (`BVH_Traverse<T, TVec, TMetric>`), and
    `Select()` finds the tree of a `BVH_PrimitiveSet` or `BVH_Geometry` set by type. `RejectNode` returns its
    metric through `out`.
26. **BVH_Set is an interface**, because `BVH_PrimitiveSet` is both a set and a `BVH_Object` (OCCT's multiple
    inheritance); its default `Box()` is the static `BVH_Set.Box(set)`.
27. **BVH's classes keep OCCT's public mutators** (trees, sets, builders), like math and BSplCLib (decision 21): they
    are tools for ported code, and users extend them (custom sets, traversals). Only the value types (`BVH_Box`,
    `BVH_Ray`, the vectors) follow the internal-mutator rule.
28. **std::partition and std::sort.** The radix sorter's `std::partition` is ported as Microsoft's implementation,
    so the linear builder (used by `Bnd_OBB`) orders primitives as native OCCT does on Windows. `BVH_QuickSorter`
    uses .NET's span sort instead of `std::sort`: both are unstable introsorts, but primitives with equal centroids
    may end up in a different order, and trees built by the sweep-plane builder may differ.
29. **OSD_Parallel → Parallel.For/ForEach** with OCCT's sequential flag; `OSD_Thread` → `Thread`;
    `std::mutex` → `Lock`; atomics → `Interlocked`/`Volatile`.
30. **NCollection_LinearVector** is reimplemented over an array (decision 15), API subset only (no move operations,
    iterators as `AsSpan()`); its 912-line GTest is not ported yet.
31. **Bnd_B2/Bnd_B3 are ported once per instantiation** (`Bnd_B2d`/`Bnd_B2f`, `Bnd_B3d`/`Bnd_B3f`), the float files
    generated from the double ones by a file-level `using RealType`, to keep C++'s float/double promotions exactly.
    C++ `std::optional` returns become nullable structs, and multi-value returns `readonly record struct`s
    (`Bnd_Box.Limits`, `Bnd_Range.Bounds`, `Bnd_OBB.HalfSizes`) for structured bindings.
32. **Twins for Bnd's bare `Set` and `Add` methods**: `Bnd_Box.FromPoint`, `Bnd_Box.Void()`, `Bnd_OBB.FromPoints`,
    `Bnd_OBB.Void()` (static factories), and `Added`, `Enlarged`, `Opened*`, `Updated`, `Limited`, ... Methods whose
    output OCCT leaves untouched in some cases take `ref` rather than `out` (`Bnd_Range.GetMin/GetMax/GetBounds`).
