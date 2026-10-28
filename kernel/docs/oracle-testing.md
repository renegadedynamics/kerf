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

- OCCT's release builds compile most argument checks out (`No_Exception`), and the port always checks. Keep oracle
  inputs valid; test invalid inputs in the unit tests instead.
- Native results differ from the port's in the last bits mostly where transcendental functions are involved
  (`acos`, `asin`, `sin`, `cos`): MSVC's maths library and .NET's don't always round the same way.
