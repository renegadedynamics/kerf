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

## 8. Handle classes (from phase 2)

These rules are provisional until the Geom package is ported:

- `Handle(T)`/`occ::handle<T>` → a reference to class `T` (nullable). `IsNull()` → `is null`;
  `Handle(T)::DownCast(x)` → `x as T`.
- `Standard_Transient` subclasses → classes. Their mutators are `internal` too. Objects a caller can see
  (curves, surfaces, shapes) are frozen once built, and a mutator called on a frozen object throws
  `Standard_ImmutableObject`. That keeps OCCT's algorithms from changing shared input geometry, which is the
  hazard build123d.NET had to work around.

## 9. Collections and performance

- Inside algorithms, NCollection containers become .NET arrays, `Span<T>` and lists. 1-based OCCT arrays keep
  their bounds through `NCollection_Array1<T>` (phase 1), so index arithmetic ports unchanged.
- **Returned immutable collections** are `IBentoList<T>` and the other Bento.Collections interfaces. For
  numeric arrays (poles, knots, mesh nodes) return a `PackedList<T>`: one contiguous array implementing
  `IBentoList<T>`, readable as a `ReadOnlySpan<T>` without copying.
- Pass structs `in`; use `stackalloc` for small fixed scratch buffers; no LINQ or allocations in inner loops.
- Floating-point parity with OCCT isn't required everywhere, but keeping the C++ order of operations costs
  nothing and keeps most results bit-identical. Don't reassociate or fuse arithmetic.

## 10. Tests

- Port the package's GTests (`src/<Module>/<Toolkit>/GTests/<Class>_Test.cxx`) to xUnit in
  `tests/OcctNet.Foundation.Tests/<package>/<Class>Tests.cs`, keeping each test's name. `EXPECT_NEAR(a, b, tol)`
  → `Assert.Equal(b, a, tol)`; `EXPECT_THROW(x, T)` → `Assert.ThrowsAny<T>(() => x)`.
- Where OCCT's release build skips a check that the port makes (`No_Exception`), a test that expects no
  exception may need the port's exception instead. Note it in the test.
- Add oracle tests (`tests/OcctNet.Oracle.Tests`) that compare the port against native OCCT on random inputs.
  See [oracle-testing.md](oracle-testing.md).
