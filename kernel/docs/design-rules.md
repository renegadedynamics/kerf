# Design rules

The rules for new and redesigned code in Kerf, from phase 10 on: the kernel's public API, the engine as units 10E to
10L reshape it, and the modelling layer. They replace [translation-rules.md](translation-rules.md), which stays as the
rules the port followed until phase 10; code not yet redesigned still reads by those.

The goal is phase10-plan.md's: a library designed for .NET, immutable, without copies at entry, reporting expected
failures as results, on .NET 10 and the latest C#. Where a rule below and the plan disagree, the plan wins and this
file is corrected.

## 1. Names

### Namespaces

| Namespace | What |
|---|---|
| `Kerf.Geometry` | Points, vectors, directions, axes, transforms, curves, surfaces |
| `Kerf.Topology` | Shapes, sub-shapes, traversal |
| `Kerf.Construction` | Primitives, sweeps, lofts |
| `Kerf.Booleans` | Fuse, cut, common, section, split |
| `Kerf.Meshing` | Triangulation and meshes |
| `Kerf.Exchange` | STEP, STL, glTF, 3MF, BREP |
| `Kerf.Fillets`, `Kerf.Offsets`, `Kerf.Features`, `Kerf.Healing`, `Kerf.Measurement` | The areas decision 1349 (65) added |
| `Kerf.Engine.<Area>` | Everything else, internal: the algorithms (`Kerf.Engine.Booleans.PaveFiller`) |
| `Kerf.Modeling` | The modelling layer |

Removed types (NCollection, TCollection) keep their OCCT names in the namespace `Occt` until unit 10J
replaces them (decision 1349, 58). Unit 10E applied these names: [api/kerf.map](api/kerf.map) is the map, and
`OcctTypeNames` (Foundation) gives each renamed type's and enum member's OCCT name at run time.

- **No namespace segment that is also a common type name.** Inside `Kerf.Engine.*`, a namespace `Kerf.Engine.Math`
  would make every `Math.Sqrt` mean the namespace: OCCT's `math` package goes to `Kerf.Engine.Numerics`. Likewise no
  type is called `Geometry`, `Topology`, `Kerf` or after its own namespace.
- **No public type whose name is a common .NET type's** (`Math`, `Vector<T>`, `Matrix4x4`, `Point`, `Timer`), even in
  another namespace: two `using`s make it ambiguous. Engine types follow the same rule for the namespaces the code
  imports (`Stream`, `ConsoleColor`, `Parallel`, `Vector3`), since a type in the code's own namespace hides .NET's.
- **No two types with one name**, even in different namespaces: a file that imports both can't name either. 10E gave
  every engine type of ModelingAlgorithms and DataExchange a name of its own (Foundation's and ModelingData's few
  remaining twins are qualified where they meet). A type isn't named after its base type, a member, or a type of an
  enclosing namespace either: each would bind a reference to the wrong one.

### Types

- PascalCase, without OCCT's package prefix: the namespace says where a type belongs. `gp_Pnt` becomes a point type in
  `Kerf.Geometry`, `BOPAlgo_PaveFiller` becomes `Kerf.Engine.Booleans.PaveFiller`.
- Acronyms of two letters stay capitals (`XY`), longer ones are words (`Svd`, `Bfgs`, `Step`, `Iges`).
- Exceptions end in `Exception`; attributes in `Attribute`; interfaces start with `I`; type parameters are `T` or
  `TName`.
- One public type per file, named after the type. Derived files keep their `// Port of:` header, which names the OCCT
  files they came from (the licence requires it, and it lets a fix in OCCT be found).
- **`2D`/`3D` where both versions exist** (`Point3D`, `Vector2D`, `CircleCurve3D`), none where only one does
  (`Cylinder`, `Sphere`, `CoordinateSystem`, `BSplineSurface`); `Plane3D` keeps its suffix because `Plane` is
  System.Numerics's (decision 1349, 57).
- Interfaces start with `I` (`ICurveTool`); exceptions end in `Exception` (`DomainErrorException`).

### Members

- PascalCase for methods, properties, events and constants; verb phrases for methods, nouns for properties.
- **Properties for values:** a count is `PoleCount`, not `NbPoles()`; a cheap yes/no is `IsClosed`; a value read
  without work is a property. Work, or a result that can fail, stays a method (`ComputeArea()`, `TryGetPlane(out …)`).
  Never a `Get` prefix on a property.
- No OCCT protocol names in the public API: no `Perform`, `IsDone`, `Build`, `Init`/`More`/`Next`/`Value` (section 4).
- Enum members without the enum's prefix: `TransformForm.Rotation`, not `gp_Rotation`. Code names them with their enum
  (`ShapeType.Edge`), not bare through `using static`: several enums and types share names such as `Edge`, `Cylinder`
  and `In`.
- A private method isn't named so that it overloads a public one: `LengthCore`, not `Length` beside `Length(…)`.

### Fields, parameters and locals

| Kind | Style | Example |
|---|---|---|
| Parameters, locals | camelCase | `point`, `tolerance` |
| Private, internal and protected instance fields | `_camelCase` | `_location` |
| Private, internal and protected static fields | `s_camelCase` | `s_cache` (and section 10: rarely needed) |
| Constants, static read-only fields | PascalCase | `MaxDegree` |
| Public fields | PascalCase, and read-only | (only in structs with exposed data) |

These are the rules [.editorconfig](../../.editorconfig) states and section 14 enforces. The renamer applies them
mechanically to ported code (`theX` → `x`, `myX` → `_x`, `aX` → `x`): tools/kerf-rename/README.md.

## 2. Value types and reference types

- **Small mathematical values are `readonly struct`s:** points, vectors, directions, axes, frames, transforms, boxes,
  intervals. They are passed `in` when larger than two doubles, and returned by value.
  - Fields are `readonly`; there are no mutators. A changed copy is a `With…` method or an operator.
  - `default(T)` must be a valid value, or the type documents that it isn't and every API rejects it. A direction whose
    default is the zero vector is invalid: its constructors normalise and its API never accepts `default`.
  - Equality is exact (`==`, `Equals`, `GetHashCode` over the fields), because Bento compares cell values. Tolerant
    comparison is a method with the tolerance spelt out: `IsEqual(other, tolerance)`, `IsParallel(other, angle)`.
- **Geometry and topology are sealed immutable classes**, or `readonly struct`s where a value is a reference plus a few
  fields, as a shape (a TShape, a location and an orientation) is.
- **Records for options, results and diagnostics:** `sealed record` (or `readonly record struct` when small and
  copied often), with `required` and `init` members. Not for geometry: records' equality and `ToString` are wrong for
  values compared with tolerances and printed for users.
- **Classes for working objects** inside the engine: an algorithm's state while it runs. They aren't published.

## 3. Immutability

- **No public setters** on public types: `init` instead. No public `Set…` methods on published types: a `With…` copy, or
  a builder.
- **Construction is the only time something changes.** A B-spline is built, then published; a builder (`…Builder`)
  holds what's being built, and its `Build()`/`ToX()` returns the immutable result.
- **Algorithms never write into their inputs** (phase10-plan.md 2.4). Where OCCT changes an input in place, the
  operation records a replacement in its working overlay; the result shares every sub-shape it didn't change.
- **Collections handed out are read-only** (section 6), and never a view of something that will change.
- `InternalsVisibleTo` stays between the four kernel assemblies, one product released together (decision 1349, 54):
  the engine types they share stay internal to the kernel.
- **Values are `readonly struct`s** (10G): the gp values have no mutating member; `Transformed`, `WithX` and the other
  returning forms are the only way to a changed value. Engine structs that are scratch storage (`Xyz`, `Xy`, `Matrix2x2`,
  `Quaternion3`, inline buffers) stay mutable, and KERF0001 guards them.

## 4. Operations

- **An operation is a static method, or an immutable operation object, that takes inputs and an options record and
  returns a result.** `BooleanOperations.Fuse(a, b, options)`, also callable as `a.Fuse(b)` (decision 1349, 53). No
  `Perform()`/`IsDone()`, no `Build()` on a mutable algorithm object, no `Init`/`More`/`Next` iteration.
- **Options are records** with defaults that are the common case: `new FuseOptions { Tolerance = 1e-6 }`. No long lists
  of optional parameters (more than three is a sign).
- **Cancellation and progress:** `CancellationToken cancellationToken = default` last, and `IProgress<double>?` where an
  operation is long. `Message_ProgressRange` stays inside the engine.
- **No ambient state:** no thread-static context, no global switches. What an operation needs, it is given (section
  10).

## 5. Results and errors

- **Expected failures return results; exceptions are for misuse.**
  - Expected: a fillet that can't be built, a Boolean on invalid input, an unreadable file, an intersection that
    doesn't converge. These return `Result<T>` (unit 10D designs it, 10F introduces it): a value or a failure, with
    `Diagnostics` (warnings and errors, each with a code, a message and the sub-shapes concerned).
  - Misuse: a null argument, a negative radius, an index out of range. These throw .NET's own exceptions
    (`ArgumentNullException.ThrowIfNull`, `ArgumentOutOfRangeException.ThrowIfNegative`), not OCCT's `Standard_…` types.
- **`Try…(out T)`** where a single bool says enough (`TryGetPlane`), with `[NotNullWhen(true)]`.
- **Callers who want exceptions** use the result's `Value` (it throws with the diagnostics), or `GetValueOrThrow()`.
- **Never catch and continue silently.** The engine's OCCT exceptions-as-statuses are replaced on hot paths and at the
  boundary first (2.5); none may cross the public API.
- **Messages** say what failed and why, in plain English, without OCCT's names.

## 6. Collections

- **Indices are 0-based** everywhere public.
- **Parameters** take the least they need: `ReadOnlySpan<T>` for numbers and small values, `IEnumerable<T>` when
  enumerated once, `IReadOnlyList<T>` when indexed. `params ReadOnlySpan<T>` for variable arguments.
- **Returns:** `ReadOnlySpan<T>` (or `ReadOnlyMemory<T>` when kept) for numeric arrays such as poles, knots and mesh
  nodes; `ImmutableArray<T>` or `IReadOnlyList<T>` for the rest; `IReadOnlyDictionary`/`IReadOnlySet` for maps and sets.
  No Schematiq.Collections in either layer (question 50).
- **Inside:** `List<T>` with a capacity when the size is known; `HashSet<T>`, `Dictionary<TKey,TValue>`;
  `FrozenDictionary`/`FrozenSet` for tables built once; `Queue<T>`, `Stack<T>`, `PriorityQueue<T,TPriority>`; a linked
  list only where an algorithm splices. NCollection's classes aren't used in new code and go in 10J.
- **Maps and sets of shapes name their comparer**, `IsSame` or `IsEqual` (KERF0002, section 14).
- **Order:** don't depend on a hash set's or dictionary's iteration order for a result; sort, or keep an ordered list
  beside it.

## 7. Spans, memory and speed

- **Small fixed-size scratch** (a few vectors, a 3×3 matrix) is `stackalloc` into a `Span<double>`, or an
  `[InlineArray]` struct when the size is a small constant; larger or variable buffers come from `ArrayPool<T>.Shared`
  and are returned in a `finally`.
- **`ref struct` views** over spans for matrices and grids (`MatrixView` over a `Span<double>` with a stride), with
  `scoped` where a span mustn't escape.
- **No allocation in inner loops:** no LINQ, no lambdas capturing locals, no boxing, no `params T[]`.
- **Measure:** a change made for speed comes with numbers from 9E's benchmarks (Bento's examples, the modelling layer's
  suite), before and after.
- **Engine arithmetic keeps OCCT's order of operations** unless a fix needs otherwise (phase10-plan.md 3.2's tests tell the difference).

## 8. Nullability

- Nullable reference types are on and warnings are errors, as today.
- `null` means "absent" only where the type says so (`T?`). Results never use `null` for failure; they use `Result<T>`.
- No `!` to silence the compiler except where the reason is written beside it.
- `[NotNullWhen]`, `[MaybeNullWhen]`, `[MemberNotNull]` where the flow is real.

## 9. Modern C#

Use them where they make the code simpler or faster; don't convert working code only to use them.

| Feature | Use for |
|---|---|
| `record`, `record struct` | Options, results, diagnostics (section 2) |
| `required`, `init` | Construction of options and results; immutable public types |
| Collection expressions `[..]` | Building lists, arrays, spans, immutable arrays; `[]` for empty |
| `field` in accessors | A property that validates or caches without a hand-written backing field |
| Extension members (`extension(T t) { … }`) | The modelling layer's conveniences over kernel types; conversions to `System.Numerics` |
| `params ReadOnlySpan<T>` | Variable arguments without an array |
| `Lock` | The rare lock left (one-time initialisation, section 10); never `lock (object)` |
| `FrozenDictionary`, `FrozenSet` | Tables built once and read many times (STEP's type tables, the message table) |
| `[InlineArray(n)]` | Fixed small buffers (a curve's poles up to degree 3, a 4×4 matrix) |
| `ref readonly` parameters | Large structs where `in` would let a caller pass a temporary unintentionally |
| `allows ref struct` | Generic helpers over span views |
| Primary constructors | Small classes whose parameters are only stored; not for public types with validation |
| File-scoped namespaces, `static` lambdas, pattern matching, raw string literals | Everywhere they fit |
| `ArgumentNullException.ThrowIfNull` and the `ThrowIf…` helpers | Argument checks (section 5) |

## 10. State and threading

- **No mutable static state.** What OCCT keeps in statics (message tables, ShapeProcess's registry, `Resource_Manager`,
  debug flags, scratch buffers) becomes per-operation context passed down, or a local.
- **Constant tables stay static**, `static readonly` and frozen, initialised once (a static constructor or `Lazy<T>`).
- **No locks** except around one-time initialisation, and then a `Lock`. Immutable values need none.
- **Every public type is safe to use from several threads at once**, because it can't change.

## 11. OCCT's code and OCCT's bugs

- The engine keeps OCCT's mathematics: renamed and retyped, not re-derived (phase10-plan.md 2.1).
- A fix to an OCCT bug gets a test of the corrected behaviour and a line in [occt-issues.md](occt-issues.md); its
  `// OCCT` marker goes (2.8).
- The name map (the applied map 10E keeps) says what each OCCT name became, so a fix in OCCT can
  still be found and ported by hand.

## 12. Documentation

- Every public member has an XML comment: what it is or does, its units, its failures. Examples for the main entry
  points of each area.
- Comments say why, not what. OCCT's comments that explain the mathematics stay.
- A divergence from OCCT's behaviour is noted where it is made and in occt-issues.md.

## 13. Tests

- Every public member has a test of its own (question 49), and each engine package at least 80% line coverage.
- Per area, invariant and property-based tests (phase10-plan.md 3.2): a Boolean's volume, mass properties under
  transforms, round trips.
- Recorded comparisons say their level: identical, geometrically equal, or an intended difference with its reason.

### Corpus cases that read their own process

OCCT's harness runs each case in a new DRAWEXE, and some scripts read that process's figures. Draw-lite runs most cases
side by side in one process, where those figures would mean nothing, so `DrawCorpus.RunAndCheck` sends such a case to a
process of its own (`DrawCaseProcess`, decisions 19101 to 19104):

| What the script reads | Which cases | What its process does |
|---|---|---|
| `meminfo h`, the live heap | `bug7287_*`, `bug24863_1`, `bug31075`, `bug31120` and any others that come to run | Runs the case twice and judges the second; parses every time rather than caching; gives the log its room at the start; takes one reading before the script. The reading is two full collections about the finalizers, to a multiple of 1 kB |
| `bbuild -t` against a fixed figure | `bug24157_*` | Reports the process's processor time, which other work on the machine doesn't add to |
| Other `meminfo` counters, or a section timed against a figure | `bug26980`, `bug83` and the like | Only starts afresh |

**How sensitive a trend is.** `checktrend` fails when the mean step passes the tolerance plus two standard deviations.
The process's own growth is the list of readings, 64 to 250 bytes a cycle, and with a 1 kB reading that can't fail. A
retained box is about 15 kB a cycle and a Boolean's result the same. Keeping 1.4 kB a cycle failed 40 runs of 40 on its
own. A step of up to 10 kB, which comes and goes in some runs, can't fail a case but can hide a leak in one short run.
Under about 1 kB a cycle may or may not be seen. `DrawCaseProcessTests` holds on to objects deliberately to keep
this true. Anything to do with a heap reading belongs in that process: a reading taken in the tests' process can't be
trusted, so a case run with a `theSetup` action (the "before unit X" reports, which assert nothing about statuses) stays
in-process and can still flip there.

**Time.** Don't assert elapsed time where another measure says the same thing. A limit that stays in a test (the elapsed
limits, `HangGuard`) has to be there to catch a hang, far above what the work takes.

## 14. Enforcement

### Naming: .editorconfig and IDE1006

[.editorconfig](../../.editorconfig) at the repository's root states the rules of section 1, and IDE1006 is an error
in the build: kernel/ and modeling/Directory.Build.props set `EnforceCodeStyleInBuild` (unit 10E). The file's last
sections turn it off, by folder, where the code doesn't obey yet:

- **OCCT's names, on purpose:** NCollection and TCollection (removed types, kept in `Occt` until 10J) and
  AdvApp2Var's f2c translation of OCCT's Fortran.
- **Tests, Draw-lite, benchmarks, samples and tools:** 10E renamed the kernel's symbols wherever they are used, not
  these projects' own.
- **158 folders of the kernel and the modelling layer** (3,337 violations when 10E measured): mostly parameters and
  locals kerf-rename left because their .NET name was taken in the same body (`thePoint` beside a local `point`),
  then type parameters (`TheCurve`), fields, local functions and constants.

**Shrinking the list:** a unit that renames a folder's remaining names (kerf-rename with a map of them, or by hand) removes
the folder from the list in the same commit; its naming is then enforced. When the list holds only OCCT's names, its
last section goes. CS8981 (lower-case type names) stays in `NoWarn` while AdvApp2Var's commons keep theirs.

### Analyzers

The kernel's analyzers (src/Kerf.Analyzers; 10E renamed their ids from `OCCTNET…` to `KERF…`):

| Rule | What | After the redesign |
|---|---|---|
| KERF0001 | A mutating call on a read-only struct value, which would change a hidden copy | **Kept.** The gp types become `readonly struct`s, so it stops firing for them, but `[InlineArray]` buffers and `ref struct` views (section 7) are mutable structs. Retire it only if no mutable struct is left. |
| KERF0002 | A map or set of shapes created without an explicit comparer | **Kept.** 10E pointed it at the shapes' new names (`Kerf.Topology.Shape` and its eight subtypes; NCollection's maps keep theirs). It still finds them by name, so a later rename must update it; a marker attribute on the shape types would end that. Its map list shrinks to .NET's collections when NCollection goes (10J). |
| KERF0003 | A way to change a public type of a public namespace (`Kerf`, `Kerf.<Area>`; not `Kerf.Engine.*`): a public or protected `set` accessor, a public field that isn't `readonly`, a public member of a struct that isn't `readonly`, a public `void` method named `Set…` or after one of OCCT's in-place operations (`Transform`, `Reverse`…) | **Written in 10G**, an error: KERF0101 to KERF0103 below, as one rule. Geom's `ResD1`…`ResD3` are internal since 10L1 (`CurveDerivatives` replaces them); enumerators are exempt (a `#pragma`), as they move by contract. |

Proposed, for 10L (not written yet; KERF0101 to KERF0103 are KERF0003 since 10G):

| Rule | Reports |
|---|---|
| KERF0101 | A public or protected `set` accessor on a public type in a public namespace (`init` is allowed) |
| KERF0102 | A public method named `Set…` on a public type in a public namespace |
| KERF0103 | A public field that isn't `readonly` or `const`; a public struct that isn't `readonly` |
| KERF0104 | A public member returning a mutable collection (`List<T>`, `T[]`, `Dictionary<,>`, NCollection) |
| KERF0105 | A mutable static field, or `[ThreadStatic]`, outside an allow-list of caches (section 10) |
| KERF0106 | An OCCT exception type (`Standard_Failure` and derived) thrown from a public member, or declared in its docs |
| KERF0107 | `InternalsVisibleTo` to anything but the assembly's own tests |

Each starts as a warning on the public namespaces only, and becomes an error once its area is clean.
