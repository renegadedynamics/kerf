# Public members no test calls

Written by `tools/public-members.cs` from the coverage run of `tools/coverage.ps1` (phase10-plan.md 3.2, question 49:
every public member tested). The public API is every exported type outside `Kerf.Engine` in the kernel's assemblies,
and `Kerf.Modeling`. A member counts as called if a line of it ran in any test run, directly or through another
member; "no data" means coverage has no entry for it. Abstract members, fields and compiler-written members without
lines are not measured, and are counted below.

Measured: 4,417 members; called 4,417; not called 0; no data 0. Not measured: 36 abstract, 6 fields, 805 compiler-written.

| Assembly | Members | Called | Not called | No data |
|---|---:|---:|---:|---:|
| Kerf.Kernel.Foundation | 1,276 | 1,276 | 0 | 0 |
| Kerf.Kernel.ModelingData | 1,033 | 1,033 | 0 | 0 |
| Kerf.Kernel.ModelingAlgorithms | 781 | 781 | 0 | 0 |
| Kerf.Kernel.DataExchange | 114 | 114 | 0 | 0 |
| Kerf.Modeling | 1,213 | 1,213 | 0 | 0 |
