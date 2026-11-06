# Public members no test calls

Written by `tools/public-members.cs` from the coverage run of `tools/coverage.ps1` (phase10-plan.md 3.2, question 49:
every public member tested). The public API is every exported type outside `Kerf.Engine` in the kernel's assemblies,
and `Kerf.Modeling`. A member counts as called if a line of it ran in any test run, directly or through another
member; "no data" means coverage has no entry for it. Abstract members, fields and compiler-written members without
lines are not measured, and are counted below.

Measured: 4,455 members; called 4,455; not called 0; no data 0. Not measured: 32 abstract, 6 fields, 845 compiler-written.

| Assembly | Members | Called | Not called | No data |
|---|---:|---:|---:|---:|
| Kerf.Kernel.Foundation | 1,267 | 1,267 | 0 | 0 |
| Kerf.Kernel.ModelingData | 1,010 | 1,010 | 0 | 0 |
| Kerf.Kernel.ModelingAlgorithms | 783 | 783 | 0 | 0 |
| Kerf.Kernel.DataExchange | 116 | 116 | 0 | 0 |
| Kerf.Modeling | 1,279 | 1,279 | 0 | 0 |
