# Oracle testing

`tests/OcctNet.Oracle.Tests` runs the port and native OCCT on the same inputs and compares the results.

## Native OCCT, recorded (unit 10A)

Since unit 10A the tests don't need native OCCT to run (decisions 1336 to 1338). They call it through `N.`, which
names the facade `OcctNative` (`Recorded/Native.g.cs`): the members of build123d.NET's native wrappers
(`Build123d.Occt`) that the tests use, each a step on the running test's **tape** (`Recorded/NativeTape.cs`).

- **Replaying** (the default, and the only mode once native OCCT is gone): each call reads its results from the test's
  recording, embedded in the assembly (`Recordings/<test class>.tape.gz`, one tape per test method or theory row), after
  checking that the test asks the same thing: the member, and a check byte of a running hash of every call's arguments
  (numbers to the bit, facade objects by their number on the tape, rooted paths by their file's content). The results:
  values, the types of the objects returned, the exception thrown, the files written (`BRepTools::Write` to a path, a
  STEP file). The port's side always runs, so a test compares the port with native's results as recorded, exactly as
  it did live. If the port's results change what the test asks native OCCT, the replay fails at that call ("the
  recording has ... here"): re-record while native exists, or compare differently (phase10-plan.md 3.2).
- **Recording** (while native OCCT exists): build with native OCCT (the properties below: `HAS_ORACLE`) and set
  `OCCTNET_ORACLE_RECORD=1`. Each call then runs natively (`Recorded/Native.Record.g.cs`) and is written to the tape,
  which is saved into the source tree's `Recordings` when the test ends (replacing that test's earlier tape only, so a
  filter re-records a subset):

  ```
  $env:OCCTNET_ORACLE_RECORD = '1'
  dotnet test tests/OcctNet.Oracle.Tests -c Release -p:Build123dNativeRoot=<build123d.NET checkout with its native build>
  Remove-Item Env:OCCTNET_ORACLE_RECORD
  dotnet test tests/OcctNet.Oracle.Tests -c Release      # the replay, from the new recordings
  ```
- **Live** (`-p:OracleDirect=true`, with native OCCT): `N` names `Build123d.Occt` itself, as before 10A; nothing is
  recorded and the budgets below don't apply. `tools/oracle-facade.cs` reads that build to generate the facade again
  when tests use members it lacks:
  `dotnet run tools/oracle-facade.cs -- tests/OcctNet.Oracle.Tests/bin/Release/net10.0/OcctNet.Oracle.Tests.dll tests/OcctNet.Oracle.Tests/Recorded`.

Native objects are recorded per test, so none can live in a static field or be made in a constructor. build123d.NET's
own scenarios (`Build123d/`) run build123d.NET on native OCCT, so only when recording: what they produce (the calls
captured, their operands' native shapes, build123d.NET's message details) is recorded as values (`Tape.Value`), and
`Build123dConstructionOracleTests` records its cases (kind, operands as native OCCT writes them, data) and runs them.

**Budgets** (decision 1337). Recording everything native returned came to 2.3 GB compressed: whole corpora (data/occ,
OCCT's test dataset) written as BRep text, meshes read triangle by triangle, thousands of random cases. The loops that
made them run under `Tape.Budget`: when recording, items are taken while the loop's recording stays under 1.5 MB
(uncompressed), a corpus's in a spread order (every seventh first) and with an item that alone takes over a quarter of
that dropped again; the replay takes the items recorded. The random tests' case counts are `Tape.Cases(live,
recorded)`. Live, every loop runs in full.

## Setup for recording and live runs

The native side comes from a build123d.NET checkout (`Build123dNetRoot`, the repository's `modeling` folder) with its
native build (`Build123dNativeRoot`, `native/out` and `.deps/install`: `native/build-deps.ps1` and
`native/build-shim.ps1` there), which builds the same OCCT release (V8_0_1). The test project then references its
`Build123d.Occt` and `Build123d` projects, which copy the native libraries into the test output. Without them the
project replays.

## Writing a test

```csharp
var anOracle = new Oracle(output);          // seeded: the same inputs every run
for (int i = 0; i < 500; i++)
{
    gp_Ax1 anAxis = anOracle.Ax1();
    using var aNative = Oracle.ToNative(anAxis);      // native wrappers must be disposed
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

`tests/OcctNet.Oracle.Tests/Build123d` replays the native calls build123d.NET's own tests make (plan section 5,
milestones 4 and 6). Open question 20's capture hook in build123d.NET isn't there, so the project references
build123d.NET's `Build123d` project (as well as `Build123d.Occt`) and rebuilds the tests' operands with its public API
(decision 947):

- `Build123dBooleanCapture.cs` transcribes the tests' constructions as scenarios. Its `Sub`, `Add`, `And`, `Fuse`,
  `Cut`, `Intersect`, `Split` and `Clean` record the `BRepAlgoAPI_*` calls and the `UnifySameDomain` clean that
  build123d.NET's Boolean path makes for each (its dispatch mirrored, decision 948), then return build123d.NET's own
  result so the scenario goes on. The benchmarks link the same file.
- `Build123dBooleanOracleTests` runs each recorded call through the Boolean oracle's `Compare` (unit 6F/6P: bytes with
  the Free column masked, canonically, histories, the port's parallel runs) and each clean through the UnifySameDomain
  oracle's (unit 6U), measures native's repeatability on every call, and checks the transcription against
  build123d.NET's own results.
- `Build123dConstructionOracleTests` does the same for the sweeps, lofts, filling, projections, constraint solvers,
  `Chamfer2D` and `Offset2D` (decision 952).

A new build123d.NET test with Booleans is added to the replay by transcribing its construction as one more scenario,
then recording (while native OCCT exists).

## The differential run (units 9D2 and 10A)

`tools/b3d-differential.ps1` runs build123d.NET's suite on OCCT.NET with its shape log on, and
`Build123dDifferentialTests` compares the log with native's, recorded in `Recordings/Build123dDifferential.native.gz`
(decision 1338: per entry the digests of its BRep text as is and masked, its properties, and its text where OCCT.NET's
differed when recorded). `-Record -NativeRoot <checkout>` records native's log again from six native runs, while
native OCCT exists. Without `B3D_DIFF_LOGS` the test only checks the recording.
