# Translating OCCT C++ to C#

These rules turn OCCT's C++ into OCCT.NET's C#. They aim for two things at once:

- **The port stays mechanical.** A C++ file and its C# file can be read side by side, line for line, and a
  fix in OCCT can be carried across by diffing. Don't restructure, rename or "improve" ported code.
- **The public API is functional.** Values that callers see never change. Only code inside OCCT.NET may
  mutate, and only values it owns.

`src/OcctNet.Foundation/gp` is the reference implementation of these rules. When in doubt, copy what it does.

## 1. Files and provenance

- One C# file per OCCT class, merging its `.hxx`, `.lxx` and `.cxx`, at
  `src/<Assembly>/<OCCT package>/<Class>.cs` (for example `src/OcctNet.Foundation/gp/gp_Pnt.cs`).
- Every ported file starts with its provenance. `tools/port-status` reads the `Port of:` lines.

  ```csharp
  // Port of: src/FoundationClasses/TKMath/gp/gp_Pnt.hxx
  // Port of: src/FoundationClasses/TKMath/gp/gp_Pnt.cxx
  // Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
  // Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
  ```

- Port from the commit in `occt-version.json` (V8_0_1), which `tools/get-occt.ps1` checks out in `.deps/occt-src`.
- Everything is in the namespace `Occt`. OCCT's prefixed names are already unique.

## 2. Names

Keep OCCT's names exactly: types (`gp_Pnt`, `BRepAlgoAPI_Cut`), methods (`Transformed`, `SetCoord`), fields
(`coord`, `myLocation`), parameters (`theP`, `theAng`) and local variables. Methods stay methods: `p.X()`, not
`p.X`. The analyzers' naming rules are off for this reason.

Additions that OCCT doesn't have (the `With` methods, static factories, `CopyTo`, `ToString`) use OCCT's
style, so they don't stand out.

## 3. Value types (gp and other copyable classes)

A C++ class used by value, not through `Handle()`, becomes a **`public partial struct`**.

| C++ | C# |
|---|---|
| private fields | `private` fields, OCCT's names. `internal` when a C++ `friend` reads them. |
| `double myMat[3][3]` | nine fields `m00 … m22`; `myMat[i][j]` is `m{i}{j}` |
| const member function | `public readonly` method |
| mutating member function (`Transform`, `SetX`, `Normalize`) | **`internal`** method |
| `[[nodiscard]] Transformed()` and the like | `public readonly` method, implemented as OCCT does (copy, mutate the copy, return) |
| default constructor with non-zero values | explicit parameterless constructor (`public gp_Dir() => …`) |
| `constexpr` | nothing; C# has no equivalent, and the JIT inlines small methods |
| `noexcept` | nothing |

**Why internal mutators?** OCCT is written in mutate-in-place style throughout. Keeping the mutators lets
ported algorithms call them exactly as the C++ does. Callers outside OCCT.NET only see `readonly` members,
so for them every value is immutable. All OCCT.NET assemblies see each other's internals.

**The one danger, and its guard.** In C#, calling a mutating method on a read-only value (a `readonly`
field, an `in` parameter, `this` in a `readonly` member, or a temporary like `ax.Location()`) silently
mutates a hidden copy. The **OCCTNET0001 analyzer** (`src/OcctNet.Analyzers`) makes that a compile error.
When it fires, copy into a local, mutate the local, and assign back, or use the `-ed` form:

```csharp
// C++:  myAxis.Location().Translate(V);   // (a bug in C++ too)
gp_Pnt aLoc = myAxis.Location();
aLoc.Translate(V);
myAxis.SetLocation(aLoc);
```

**Every internal mutator has a public, non-mutating twin.** If OCCT has one (`Transform`/`Transformed`),
port it. If not, add it:

- `SetFoo(args)` → `public readonly T WithFoo(args)`: copy, call `SetFoo`, return the copy.
- Any other mutator `Foo()` → the past participle, as OCCT names its own twins (`XReverse` → `XReversed`,
  `BaryCenter` → `BaryCentered`).
- Where a `Set` method rebuilds the whole value regardless of its state (`gp_Trsf::SetRotation`), also add a
  static factory named without `Set` (`gp_Trsf.Rotation(axis, angle)`).

**Every value type also gets**, below its ported members:

- `IEquatable<T>`, `Equals`, `GetHashCode`, `==` and `!=`, comparing fields exactly. Bento compares cell
  values this way. OCCT's tolerant comparisons stay as `IsEqual(other, tolerance)`.
- `ToString()` via `gp.Format`, which writes round-trippable, culture-invariant numbers.
- `CopyTo(Span<double>)` where the value is a block of numbers (`gp_XYZ`, `gp_Mat`).

`default(T)` is all zeros. For types whose OCCT default isn't zero (`gp_Dir` is (1, 0, 0), `gp_Trsf` is the
identity, `gp_Ax2` is the global system), say so in the type's `<remarks>`. Ported code must write `new T()`,
never `default`. Arrays of such types must be filled explicitly (OCCT's arrays default-construct their
elements; C# arrays don't).

## 4. Signatures

| C++ | C# |
|---|---|
| `const T&` (struct) | `in T` |
| `T&` written but not read (an output) | `out T` |
| `T&` read and written | `ref T` |
| `const double`, `const int`, `bool` | `double`, `int`, `bool` |
| returns `const T&` of a field | returns `T` by value |
| returns `T&` of a field (`ChangeCoord`, `ChangeValue`) | `[UnscopedRef] internal ref T` |
| static returning `const T&` of a static (`gp::Origin()`) | `public static ref readonly T` |
| `operator()(r, c)` const | `public readonly double this[int r, int c]` |
| default arguments | default arguments when constant, overloads otherwise |
| `Standard_Real`, `Standard_Integer`, `Standard_Boolean` | `double`, `int`, `bool` |

## 5. Expressions

- **Free functions from Standard** (`RealSmall()`, `Square()`, `IsOdd()`, `Epsilon()`, `Min`, `Max`, `M_PI`)
  are in `Occt.Standard`, which every OCCT.NET project imports with `using static`. `Min`/`Max` behave like
  `std::min`/`std::max`, including with NaN, so use them rather than `Math.Min`/`Math.Max` in ported code.
- `std::abs`, `std::sqrt`, `sin`, `cos`, `atan2`, `pow` → `Math.Abs`, `Math.Sqrt`, ... (same argument order).
- **Raise_if macros** → the functions of the same name in `Standard` (`Standard_OutOfRange_Raise_if(cond, msg)`).
  OCCT's release builds compile most of them out (`No_Exception`); the port always checks. Add a function for a
  new exception type as you need it.
- **Exceptions** keep OCCT's types and hierarchy (`Standard_ConstructionError : Standard_DomainError :
  Standard_Failure : Exception`). `throw Standard_Failure("msg")` → `throw new Standard_Failure("msg")`.
- **Implicit conversions.** C++ converts silently through non-explicit constructors (`gp_Vec` → `gp_Dir`). C#
  has none of these: write the constructor out (`new gp_Dir(theVec)`). The compiler finds every place.
- **Operators.** C++ operators become C# operators with the same meaning, including `*` for the dot product and
  `^` for the cross product. Compound assignment (`+=`) then works on locals as in C++.
- **Integer arithmetic.** `/` and `%` truncate as in C++. Keep C++'s `int`/`double` promotions exactly.
- **Unscoped enums** keep their member names (`gp_TrsfForm.gp_Rotation`). Files that use them write
  `using static Occt.gp_TrsfForm;` so the code reads as the C++ does. A nested `enum class D` stays nested.
- **Comments.** Keep OCCT's comments that explain the code; drop the ones that only repeat the method's name.
  Public members get a `/// <summary>` condensed from OCCT's `//!` documentation where it says something useful.

## 6. Leave out

- `DumpJson`, `InitFromJson`, `operator<<` (streams): `ToString` replaces them.
- `DEFINE_STANDARD_ALLOC`, `DEFINE_STANDARD_RTTIEXT` and other memory and RTTI macros.
- `std::hash` and `std::equal_to` specialisations: `GetHashCode` and `Equals` replace them.
- `#ifdef` branches for other platforms and for `OCCT_DEBUG`.
- `GetData()`/`ChangeData()` raw pointers: use `CopyTo(Span<double>)`, or a `ref` accessor if a port needs it.

Record each omission that changes behaviour in the file, with a one-line comment.

## 7. OCCT's own bugs

Port what OCCT does, not what it should do, so results match the reference. Mark the line with a comment that
starts `// OCCT` and add it to [occt-issues.md](occt-issues.md). Fixes are a decision for later, made once the
oracle tests can show what changes.

## 8. Handle classes

- `Handle(T)`/`occ::handle<T>` → a reference to class `T` (nullable). `IsNull()` → `is null`;
  `Nullify()` → `= null`; `Handle(T)::DownCast(x)` → `x as T`; `occ::handle<T> h = new T(...)` → `T h = new T(...)`.
- `Standard_Transient` subclasses → classes deriving from `Standard_Transient`. `DEFINE_STANDARD_RTTIEXT` and
  `IMPLEMENT_STANDARD_RTTIEXT` go; `DynamicType()`, `IsKind` and `IsInstance` work through .NET types.
- **Geometry and topology classes** (Geom, Geom2d, Poly's triangulations and polygons once attached, TopoDS's
  TShapes, BRep's representations) are immutable to callers and freezable inside the port:
  - Their mutators (`SetRadius`, `SetPole`, `InsertKnot`, `Transform`, `Mirror`, ...) are `internal`, and the
    **first line of each is `CheckMutable();`**, which throws `Standard_ImmutableObject` on a frozen object.
    Mutators reached only through another mutator (private helpers) don't need the check.
  - Constructors and queries are public. Callers transform with OCCT's own non-mutating forms
    (`Geom_Geometry::Transformed`, `Mirrored`, `Rotated`, ..., which copy and then mutate the copy), and every
    `SetFoo` gets a public `WithFoo` returning a mutated `Copy()` (as for value types).
  - `Copy()` returns an unfrozen object, so algorithms can keep doing what OCCT does: copy, then modify.
  - An object is **frozen** (`Freeze()`, internal, one-way) when it becomes shared: when topology takes it
    (BRep_Builder storing a curve, surface or triangulation; phase 3 decides exactly where), and wherever a port
    hands a long-lived internal object to a caller. Then any OCCT algorithm that tries to change shared geometry
    in place throws instead of silently changing every shape that uses it — the hazard build123d.NET had to work
    around. When that happens in ported code, the port copies first, and the place is noted in decisions.md.
- Other transient classes (algorithms, caches, tools, math_Function implementations) keep OCCT's mutators
  `internal` or public as their package decides (decision 21): they are working objects, not values.
- Abstract interfaces (math_Function and the like, pure virtual classes) → `abstract class` with `abstract`
  members; virtual → `virtual`/`override`. Keep C++ multiple inheritance of interfaces as C# interfaces only when
  a class really has two bases; otherwise a single base class.

## 9. Collections, pointers and performance

- **NCollection containers** are ported in `src/OcctNet.Foundation/NCollection` with OCCT's API over .NET storage:
  `NCollection_Array1/Array2/HArray1/HArray2` (arbitrary bounds), `NCollection_Sequence/HSequence` (1-based),
  `NCollection_List` (with its `Iterator`: More/Next/Value), `NCollection_DynamicArray`/`NCollection_Vector`,
  `NCollection_LocalArray`, `NCollection_Map/IndexedMap/DataMap/IndexedDataMap`. Ported code uses them unchanged.
  - **They are classes.** A C++ copy (`NCollection_Array1<double> a(b);`, `a = b;`, pass or return by value) must be
    written as a copy: `new(b)` or `a.Assign(b)`. A plain C# assignment shares the container. Check every
    by-value use of a container when porting.
  - Elements are default-constructed as in C++ (`gp_Dir` elements are (1, 0, 0)).
  - `NCollection_Array1`, `NCollection_Array2` and `NCollection_Sequence` derive from `Standard_Transient`, so the
    H-collections are transients as in OCCT (a handle to an `NCollection_HArray1` stored or sent as a
    `occ::handle<Standard_Transient>`, `STANDARD_TYPE(NCollection_HArray1<...>)`: decision 12801). Nothing else of
    `Standard_Transient` applies to them (they aren't frozen).
  - `NCollection_Array1<T>::Iterator` → `NCollection_Array1<T>.Iterator` (More/Next/Value).
  - `a(i)`, `a[i]`, `a.ChangeValue(i)` → `a[i]` (a `ref` indexer, for reading and writing); `a.Value(i)` stays.
  - The "buffer" constructors (`NCollection_Array1<T> a(b(i), lower, upper)`, an array over another's memory) →
    `new(b, i, lower, upper)`, which shares `b`'s storage.
  - Allocator arguments (`NCollection_BaseAllocator`, `NCollection_IncAllocator`) are dropped.
  - Hasher template arguments become an `IEqualityComparer<TKey>` constructor argument. The unordered maps don't
    iterate in OCCT's order; if a ported algorithm's result depends on that order, note it.
  - Typedefs (`TColStd_Array1OfReal`, `TColgp_Array1OfPnt`) are written out as the NCollection type.
- **Raw pointers and array references.** OCCT often passes the start of an array as `double*` or `double&` and does
  pointer arithmetic (PLib, BSplCLib). Port these as `Span<double>` (`ReadOnlySpan<double>` when const);
  `p + k` → `p[k..]` or `p.Slice(k)`; `&a(i)` / `a.ChangeValue(i)` passed as a start → `a.AsSpan(i)`. A pointer that
  is moved along (`p++`, `p += n`) becomes an index into the span.
- **Templates** → generics where the parameter is a type the code works on (`math_VectorBase<T>`); where C++ uses a
  template only for speed over a few fixed types, port the instantiations OCCT uses. `.gxx` "generic" files
  (included with macros defining the types) are ported once per instantiation OCCT makes, named as OCCT names them.
- `TCollection_AsciiString`, `TCollection_ExtendedString` → `string`. `Standard_OStream`/`Standard_SStream` → `TextWriter`/`StringBuilder`.
- `OSD_Parallel::For` → `Parallel.For` (and keep any `theIsSequential` flag). Mutexes (`Standard_Mutex`) → `Lock`.
- **Progress**: `Message_ProgressRange` and `Message_ProgressScope` keep OCCT's API (More, Next, UserBreak).
  A C# caller makes a `Message_ProgressIndicator` from a `CancellationToken` and an `IProgress<double>`.
  Where the C++ relies on a scope's destructor, use `using var aPS = new Message_ProgressScope(...)`.
- **Returned immutable collections** are `IBentoList<T>` and the other Bento.Collections interfaces. For
  numeric arrays (poles, knots, mesh nodes) return a `PackedList<T>`: one contiguous array implementing
  `IBentoList<T>`, readable as a `ReadOnlySpan<T>` without copying. Low-level APIs (BSplCLib, PLib, math) keep
  OCCT's NCollection and span parameters: they are tools for ported code, not values for callers.
- Pass structs `in`; use `stackalloc` for small fixed scratch buffers; no LINQ or allocations in inner loops.
- Floating-point parity with OCCT isn't required everywhere, but keeping the C++ order of operations costs
  nothing and keeps most results bit-identical. Don't reassociate or fuse arithmetic.
- Anonymous-namespace and file-`static` functions → `private static` (or `internal static` if several files need
  them) methods on the class of the file that holds them, or on a static class named after the file.

## 10. Tests

- Port the package's GTests (`src/<Module>/<Toolkit>/GTests/<Class>_Test.cxx`) to xUnit in
  `tests/OcctNet.Foundation.Tests/<package>/<Class>Tests.cs`, keeping each test's name. `EXPECT_NEAR(a, b, tol)`
  → `Assert.Equal(b, a, tol)`; `EXPECT_THROW(x, T)` → `Assert.ThrowsAny<T>(() => x)`.
- Where OCCT's release build skips a check that the port makes (`No_Exception`), a test that expects no
  exception may need the port's exception instead. Note it in the test.
- Add oracle tests (`tests/OcctNet.Oracle.Tests`) that compare the port against native OCCT on random inputs.
  See [oracle-testing.md](oracle-testing.md).
