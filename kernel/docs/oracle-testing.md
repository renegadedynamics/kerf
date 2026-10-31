# Oracle testing

`tests/OcctNet.Oracle.Tests` runs the port and native OCCT on the same inputs and compares the results.

## Setup

The native side comes from a sibling checkout of build123d.NET (`../build123d.net`, or set the MSBuild property
`Build123dNetRoot`), which builds the same OCCT release (V8_0_1) and generates .NET bindings for the classes it
whitelists. That checkout needs its native build: `native/build-deps.ps1` and `native/build-shim.ps1` there. The
test project references its `Build123d.Occt` project, which copies the native libraries into the test output.

Without that checkout the project still builds, and its one test fails with a message saying what's missing, so
a missing oracle never passes as "no differences".

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

A new build123d.NET test with Booleans is added to the replay by transcribing its construction as one more scenario.
