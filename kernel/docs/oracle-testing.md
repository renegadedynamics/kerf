# Oracle testing

`tests/OcctNet.Oracle.Tests` runs the port on the inputs native OCCT was given and compares the results with native's,
as recorded. Native OCCT went in unit 10B, with build123d.NET's native backend that built it: the recordings are frozen.

## Native OCCT, recorded (units 10A and 10B)

Since unit 10A the tests don't need native OCCT to run (decisions 1336 to 1338). They call it through `N.`, which
names the facade `OcctNative` (`Recorded/Native.g.cs`): the members of build123d.NET's native wrappers
(`Build123d.Occt`) that the tests used, each a step on the running test's **tape** (`Recorded/NativeTape.cs`).

Each call reads its results from the test's recording, embedded in the assembly (`Recordings/<test class>.tape.gz`,
one tape per test method or theory row), after checking that the test asks the same thing: the member, and a check
byte of a running hash of every call's arguments (numbers to the bit, facade objects by their number on the tape,
rooted paths by their file's content). The results: values, the types of the objects returned, the exception thrown,
the files written (`BRepTools::Write` to a path, a STEP file). The port's side always runs, so a test compares the port
with native's results as recorded, exactly as it did live. If the port's results change what the test asks native
OCCT, the replay fails at that call ("the recording has ... here"): compare differently (phase10-plan.md 3.2), as
native OCCT can't be asked again.

**Frozen** (unit 10B, decision 15102). Until unit 10B the tests could also record (`OCCTNET_ORACLE_RECORD=1`, built
with native OCCT: `HAS_ORACLE`, `Recorded/Native.Record.g.cs`) and run live (`-p:OracleDirect=true`), and
`tools/oracle-facade.cs` generated the facade from a live build. All went with native OCCT: the facade is committed
code now, and a test can only use the members it has, replayed as recorded.

build123d.NET's own scenarios (`Build123d/`) ran build123d.NET on native OCCT when recording: what they produced (the
calls captured, their operands' native shapes, build123d.NET's message details) is recorded as values (`Tape.Value`),
and `Build123dConstructionOracleTests` runs the cases recorded (kind, operands as native OCCT wrote them, data). The
scenarios themselves went in unit 10B.

**Budgets** (decision 1337). Recording everything native returned came to 2.3 GB compressed: whole corpora (data/occ,
OCCT's test dataset) written as BRep text, meshes read triangle by triangle, thousands of random cases. The loops that
made them ran under `Tape.Budget`: when recording, items were taken while the loop's recording stayed under a budget,
a corpus's in a spread order (every seventh first); the replay takes the items recorded. The random tests' case counts
are `Tape.Cases(live, recorded)`, the recorded count.

## Writing a test

```csharp
var anOracle = new Oracle(output);          // seeded: the same inputs every run
for (int i = 0; i < 500; i++)
{
    gp_Ax1 anAxis = anOracle.Ax1();
    using var aNative = Oracle.ToNative(anAxis);      // a facade object, replayed
    ...
    anOracle.Same(nativeResult, portResult, "what is compared");
}
anOracle.Report("gp_Trsf composition");     // "17879/24000 values bit-identical, largest relative difference 2.84E-014"
```

`Same` passes bit-identical values, and otherwise checks the relative difference (absolute below 1) against a
tolerance, 1e-12 by default. Loosen it per comparison where the computation amplifies rounding (powers, Euler
angles), and say why.

## Things to know

- Build the inputs on both sides from the same raw values through the same constructors. `Oracle.ToNative(gp_Dir)`
  and `ToNative(gp_Ax3)` hand native OCCT the port's normalised values, which the native constructors normalise
  (and orthogonalise) again: the inputs can then differ in the last bit, and so does everything computed from them.
  `GeomOracleInputs` builds directions, axes and transformations that way, in 3D and 2D, for the Geom and Geom2d
  tests (curves and surfaces); with it, even rotations (sin, cos) compare bit-identical, and the curve tests went
  from 90.9% to 100% bit-identical values (decision 175).
- OCCT's release builds compile most argument checks out (`No_Exception`), and the port always checks. Keep oracle
  inputs valid; test invalid inputs in the unit tests instead.
- Native results differ from the port's in the last bits mostly where transcendental functions are involved
  (`acos`, `asin`, `sin`, `cos`): MSVC's maths library and .NET's don't always round the same way. Check the
  inputs first: the differences the curve tests blamed on sin and cos were differences of the inputs.
- When native OCCT computes an input itself (a Geom method that evaluates the curve first, like
  MovePointAndTangent's `Geom_Curve::D1`), build the port's input with the same native call: `EvalDN` and `D1` can
  differ in the last bit, and an ill-conditioned step magnifies it.
- Mirror the native call path exactly: read the Geom (or adaptor) source to see which BSplCLib/BSplSLib function it
  calls with which arguments (flat knots or knots with multiplicities, located span, periodic normalisation, the
  rationality flags it derives), and call the port the same way. Done that way, the B-spline packages are
  bit-identical.
- All oracle tests are in the namespace `OcctNet.Oracle.Tests`, whatever their folder: a namespace named after an OCCT
  package (`...Tests.BSplCLib`, `...Tests.gp`) hides the OCCT class of the same name from every other test file.
- Build native axes and directions from the same raw components as the port's, rather than converting the port's
  normalised `gp_Dir` with `Oracle.ToNative`: native OCCT normalises the unit direction again, which can change its
  last bit (GProp's oracle tests saw it on about 700 values).

## build123d.NET's own operations (unit 6M)

`tests/OcctNet.Oracle.Tests/Build123d` replays the native calls build123d.NET's own tests made (plan section 5,
milestones 4 and 6). They were captured by transcribing the tests' constructions as scenarios run with build123d.NET's
public API on native OCCT (decision 947), when recording; the scenarios went with native OCCT in unit 10B, and their
calls are recorded:

- `Build123dBooleanCapture.cs`: the `BRepAlgoAPI_*` calls and the `UnifySameDomain` clean that build123d.NET's Boolean
  path made for each `Sub`, `Add`, `And`, `Fuse`, `Cut`, `Intersect`, `Split` and `Clean` (its dispatch mirrored,
  decision 948), with build123d.NET's own result.
- `Build123dBooleanOracleTests` runs each recorded call through the Boolean oracle's `Compare` (unit 6F/6P: bytes with
  the Free column masked, canonically, histories, the port's parallel runs) and each clean through the UnifySameDomain
  oracle's (unit 6U), and checks the transcription against build123d.NET's own results.
- `Build123dConstructionOracleTests` does the same for the sweeps, lofts, filling, projections, constraint solvers,
  `Chamfer2D` and `Offset2D` (decision 952).

## The differential run (units 9D2 and 10A)

`tools/b3d-differential.ps1` runs build123d.NET's suite on OCCT.NET with its shape log on, and
`Build123dDifferentialTests` compares the log with native's, recorded in `Recordings/Build123dDifferential.native.gz`
(decision 1338: per entry the digests of its BRep text as is and masked, its properties, and its text where OCCT.NET's
differed when recorded, from six native runs). It is frozen since unit 10B. Without `B3D_DIFF_LOGS` the test only
checks the recording.
