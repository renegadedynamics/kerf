# Line coverage

The floor of question 49 (phase10-plan.md 3.2): every public member tested, at least 80% line coverage per engine
package, lower where the Draw-lite corpus covers a package and the gap is listed. Written by `tools/coverage.ps1`
(`tools/coverage-report.cs`): line coverage of the kernel's and the modeling layer's assemblies, merged (a line is
covered if any run covered it): the kernel's unit tests, the public API's tests, the Draw-lite tests as they run by
default (the corpus classes on their samples, `KERF_CORPUS_FULL` unset), the modeling layer's suite, and the
oracle tests' replays with `-WithOracle`. A package is the folder under the assembly (OCCT's package for the kernel).
Lines are the coverable lines Microsoft's code coverage counts. The public members no test calls are in
[public-members.md](public-members.md).

Measured on 6 November 2026.
The runs are the whole run at main's head (86344c5e4) with unit 11I's new tests and a fresh Foundation run added: the lines of only two files moved (`MathRoot_Bisection.cs`, `BVH/DistanceField.cs`), and those come from the fresh run alone.
Test projects merged: `Kerf.Kernel.DataExchange.Tests`, `Kerf.Kernel.DrawLite.Tests`, `Kerf.Kernel.Foundation.Tests`, `Kerf.Kernel.ModelingAlgorithms.Laws`, `Kerf.Kernel.ModelingAlgorithms.Tests`, `Kerf.Kernel.ModelingData.Tests`, `Kerf.Kernel.Oracle.Tests`, `Kerf.Kernel.PublicApi.Tests`, `Kerf.Modeling.Direct`, `Kerf.Modeling.Tests`.

## By assembly

| Assembly | Packages | Lines | Covered | % | Packages under 80% |
|---|---:|---:|---:|---:|---:|
| Kerf.Kernel.DataExchange | 62 | 85,586 | 49,939 | 58.3 | 41 |
| Kerf.Kernel.Foundation | 33 | 61,793 | 56,613 | 91.6 | 0 |
| Kerf.Kernel.ModelingAlgorithms | 119 | 259,115 | 223,738 | 86.3 | 13 |
| Kerf.Kernel.ModelingData | 50 | 143,926 | 131,357 | 91.3 | 3 |
| Kerf.Modeling | 8 | 8,153 | 7,507 | 92.1 | 0 |
| **Total** | 272 | 558,573 | 469,154 | 84.0 | 57 |

## The gaps

Every package under the floor, with the reason (`docs/coverage-gaps.tsv`; written from what each package does, not from
a line-by-line audit). "With corpus" adds the whole Draw-lite corpus (`-FullCorpus`) to the runs above: where it
equals the percentage beside it, the corpus's sample already reaches all the corpus does, so question 49's allowance
for a package the corpus covers doesn't apply, and these stay open gaps for the owner.

| Assembly | Package | Lines | Uncovered | % | With corpus % | Reason |
|---|---|---:|---:|---:|---:|---|
| Kerf.Kernel.DataExchange | RWStepAP214 | 13,030 | 5,328 | 59.1 | 59.1 | One read and write tool per STEP entity; the rest are for entities in no STEP file we test with. Real tests need STEP files holding them (kinematics, FEA, tolerances): owner's call |
| Kerf.Kernel.DataExchange | RWStepKinematics | 4,448 | 4,448 | 0.0 | 0.0 | One read and write tool per STEP entity; the rest are for entities in no STEP file we test with. Real tests need STEP files holding them (kinematics, FEA, tolerances): owner's call |
| Kerf.Kernel.DataExchange | RWStepDimTol | 2,624 | 2,248 | 14.3 | 14.3 | One read and write tool per STEP entity; the rest are for entities in no STEP file we test with. Real tests need STEP files holding them (kinematics, FEA, tolerances): owner's call |
| Kerf.Kernel.DataExchange | IFSelect | 3,935 | 1,824 | 53.6 | 53.6 | OCCT's exchange framework: selections, sessions and printing that Draw's data-exchange commands drive and Draw-lite doesn't port; only what a read or write passes through runs |
| Kerf.Kernel.DataExchange | StepData | 3,602 | 1,750 | 51.4 | 51.4 | OCCT's exchange framework: selections, sessions and printing that Draw's data-exchange commands drive and Draw-lite doesn't port; only what a read or write passes through runs |
| Kerf.Kernel.DataExchange | RWStepGeom | 3,446 | 1,567 | 54.5 | 54.5 | One read and write tool per STEP entity; the rest are for entities in no STEP file we test with. Real tests need STEP files holding them (kinematics, FEA, tolerances): owner's call |
| Kerf.Kernel.DataExchange | RWStepBasic | 3,182 | 1,376 | 56.8 | 56.8 | One read and write tool per STEP entity; the rest are for entities in no STEP file we test with. Real tests need STEP files holding them (kinematics, FEA, tolerances): owner's call |
| Kerf.Kernel.DataExchange | Interface | 4,139 | 1,281 | 69.1 | 69.1 | OCCT's exchange framework: selections, sessions and printing that Draw's data-exchange commands drive and Draw-lite doesn't port; only what a read or write passes through runs |
| Kerf.Kernel.DataExchange | RWStepShape | 2,390 | 1,254 | 47.5 | 47.5 | One read and write tool per STEP entity; the rest are for entities in no STEP file we test with. Real tests need STEP files holding them (kinematics, FEA, tolerances): owner's call |
| Kerf.Kernel.DataExchange | RWStepVisual | 2,952 | 1,209 | 59.0 | 59.0 | One read and write tool per STEP entity; the rest are for entities in no STEP file we test with. Real tests need STEP files holding them (kinematics, FEA, tolerances): owner's call |
| Kerf.Kernel.DataExchange | RWStepFEA | 1,431 | 922 | 35.6 | 35.6 | One read and write tool per STEP entity; the rest are for entities in no STEP file we test with. Real tests need STEP files holding them (kinematics, FEA, tolerances): owner's call |
| Kerf.Kernel.DataExchange | RWStepRepr | 1,703 | 906 | 46.8 | 46.8 | One read and write tool per STEP entity; the rest are for entities in no STEP file we test with. Real tests need STEP files holding them (kinematics, FEA, tolerances): owner's call |
| Kerf.Kernel.DataExchange | XSControl | 2,072 | 846 | 59.2 | 59.2 | OCCT's exchange framework: selections, sessions and printing that Draw's data-exchange commands drive and Draw-lite doesn't port; only what a read or write passes through runs |
| Kerf.Kernel.DataExchange | StepToTopoDS | 2,231 | 813 | 63.6 | 63.6 | The translators' rarer forms and write modes; follows the STEP test data |
| Kerf.Kernel.DataExchange | STEPControl | 2,689 | 771 | 71.3 | 71.3 | The translators' rarer forms and write modes; follows the STEP test data |
| Kerf.Kernel.DataExchange | StepBasic | 1,878 | 684 | 63.6 | 63.6 | STEP's entity classes: the fields of entities in no STEP file we test with, as the RWStep packages |
| Kerf.Kernel.DataExchange | Transfer | 2,109 | 676 | 67.9 | 67.9 | OCCT's exchange framework: selections, sessions and printing that Draw's data-exchange commands drive and Draw-lite doesn't port; only what a read or write passes through runs |
| Kerf.Kernel.DataExchange | StepElement | 894 | 662 | 26.0 | 26.0 | STEP's entity classes: the fields of entities in no STEP file we test with, as the RWStep packages |
| Kerf.Kernel.DataExchange | StepAP214 | 1,703 | 578 | 66.1 | 66.1 | STEP's entity classes: the fields of entities in no STEP file we test with, as the RWStep packages |
| Kerf.Kernel.DataExchange | STEPConstruct | 2,209 | 546 | 75.3 | 75.3 | The translators' rarer forms and write modes; follows the STEP test data |
| Kerf.Kernel.DataExchange | StepKinematics | 1,257 | 503 | 60.0 | 60.0 | STEP's entity classes: the fields of entities in no STEP file we test with, as the RWStep packages |
| Kerf.Kernel.DataExchange | StepVisual | 1,482 | 497 | 66.5 | 66.5 | STEP's entity classes: the fields of entities in no STEP file we test with, as the RWStep packages |
| Kerf.Kernel.DataExchange | StepToGeom | 1,242 | 435 | 65.0 | 65.0 | The translators' rarer forms and write modes; follows the STEP test data |
| Kerf.Kernel.DataExchange | RWStepElement | 523 | 423 | 19.1 | 19.1 | One read and write tool per STEP entity; the rest are for entities in no STEP file we test with. Real tests need STEP files holding them (kinematics, FEA, tolerances): owner's call |
| Kerf.Kernel.DataExchange | StepShape | 1,090 | 398 | 63.5 | 63.5 | STEP's entity classes: the fields of entities in no STEP file we test with, as the RWStep packages |
| Kerf.Kernel.DataExchange | TopoDSToStep | 1,469 | 356 | 75.8 | 75.8 | The translators' rarer forms and write modes; follows the STEP test data |
| Kerf.Kernel.DataExchange | StepGeom | 1,281 | 343 | 73.2 | 73.2 | STEP's entity classes: the fields of entities in no STEP file we test with, as the RWStep packages |
| Kerf.Kernel.DataExchange | StepFEA | 748 | 311 | 58.4 | 58.4 | STEP's entity classes: the fields of entities in no STEP file we test with, as the RWStep packages |
| Kerf.Kernel.DataExchange | StepRepr | 760 | 241 | 68.3 | 68.3 | STEP's entity classes: the fields of entities in no STEP file we test with, as the RWStep packages |
| Kerf.Kernel.DataExchange | RWStepAP203 | 341 | 217 | 36.4 | 36.4 | One read and write tool per STEP entity; the rest are for entities in no STEP file we test with. Real tests need STEP files holding them (kinematics, FEA, tolerances): owner's call |
| Kerf.Kernel.DataExchange | STEPSelections | 229 | 209 | 8.7 | 8.7 | OCCT's exchange framework: selections, sessions and printing that Draw's data-exchange commands drive and Draw-lite doesn't port; only what a read or write passes through runs |
| Kerf.Kernel.DataExchange | MoniTool | 786 | 208 | 73.5 | 73.5 | OCCT's exchange framework: selections, sessions and printing that Draw's data-exchange commands drive and Draw-lite doesn't port; only what a read or write passes through runs |
| Kerf.Kernel.DataExchange | StepDimTol | 560 | 184 | 67.1 | 67.1 | STEP's entity classes: the fields of entities in no STEP file we test with, as the RWStep packages |
| Kerf.Kernel.DataExchange | IFGraph | 379 | 156 | 58.8 | 58.8 | OCCT's exchange framework: selections, sessions and printing that Draw's data-exchange commands drive and Draw-lite doesn't port; only what a read or write passes through runs |
| Kerf.Kernel.DataExchange | StepAP203 | 285 | 146 | 48.8 | 48.8 | STEP's entity classes: the fields of entities in no STEP file we test with, as the RWStep packages |
| Kerf.Kernel.DataExchange | APIHeaderSection | 299 | 116 | 61.2 | 61.2 | OCCT's exchange framework: selections, sessions and printing that Draw's data-exchange commands drive and Draw-lite doesn't port; only what a read or write passes through runs |
| Kerf.Kernel.DataExchange | STEPEdit | 156 | 98 | 37.2 | 37.2 | OCCT's exchange framework: selections, sessions and printing that Draw's data-exchange commands drive and Draw-lite doesn't port; only what a read or write passes through runs |
| Kerf.Kernel.DataExchange | StepSelect | 167 | 71 | 57.5 | 57.5 | OCCT's exchange framework: selections, sessions and printing that Draw's data-exchange commands drive and Draw-lite doesn't port; only what a read or write passes through runs |
| Kerf.Kernel.DataExchange | StepAP242 | 144 | 70 | 51.4 | 51.4 | STEP's entity classes: the fields of entities in no STEP file we test with, as the RWStep packages |
| Kerf.Kernel.DataExchange | RWStepAP242 | 150 | 46 | 69.3 | 69.3 | One read and write tool per STEP entity; the rest are for entities in no STEP file we test with. Real tests need STEP files holding them (kinematics, FEA, tolerances): owner's call |
| Kerf.Kernel.DataExchange | UnitsMethods | 82 | 29 | 64.6 | 64.6 | Conversions for units no test file uses |
| Kerf.Kernel.ModelingAlgorithms | HLRBRep | 6,844 | 1,592 | 76.7 | 76.7 | Hidden-line paths past the few hlr cases |
| Kerf.Kernel.ModelingAlgorithms | TopOpeBRepBuild | 1,302 | 921 | 29.3 | 29.3 | The legacy Booleans' remains: only what fillets' and HLR's callers need is alive. Candidates for removal, not tests |
| Kerf.Kernel.ModelingAlgorithms | TopOpeBRepTool | 988 | 719 | 27.2 | 27.2 | The legacy Booleans' remains: only what fillets' and HLR's callers need is alive. Candidates for removal, not tests |
| Kerf.Kernel.ModelingAlgorithms | IntCurveSurface | 1,949 | 597 | 69.4 | 69.4 | About 400 lines are dead in OCCT too: the limits for infinite surfaces, cut off by its clamp to 1e50, and two helpers nothing calls (occt-issues.md); the floor is out of reach by honest tests |
| Kerf.Kernel.ModelingAlgorithms | TopOpeBRepDS | 2,009 | 536 | 73.3 | 73.3 | The legacy Booleans' remains: only what fillets' and HLR's callers need is alive. Candidates for removal, not tests |
| Kerf.Kernel.ModelingAlgorithms | BiTgte | 1,131 | 382 | 66.2 | 66.2 | Forms past the corpus's few cases; both are small OCCT features |
| Kerf.Kernel.ModelingAlgorithms | Draft | 1,429 | 351 | 75.4 | 75.4 | Forms past the corpus's few cases; both are small OCCT features |
| Kerf.Kernel.ModelingAlgorithms | Geom2dHatch | 994 | 220 | 77.9 | 77.9 | Intersection forms by curve and surface type the tests don't reach |
| Kerf.Kernel.ModelingAlgorithms | TopTrans | 400 | 102 | 74.5 | 74.5 | The legacy Booleans' remains: only what fillets' and HLR's callers need is alive. Candidates for removal, not tests |
| Kerf.Kernel.ModelingAlgorithms | BRepApprox | 135 | 90 | 33.3 | 33.3 | A small approximation wrapper, partly run |
| Kerf.Kernel.ModelingAlgorithms | FilletSurf | 374 | 80 | 78.6 | 78.6 | Blends on restrictions and points, and rarely reached walking branches; 884 fillet cases already run |
| Kerf.Kernel.ModelingAlgorithms | Geom2dInt | 183 | 53 | 71.0 | 71.0 | Intersection forms by curve and surface type the tests don't reach |
| Kerf.Kernel.ModelingAlgorithms | TopBas | 41 | 41 | 0.0 | 0.0 | A single value type nothing calls (HLR's interference): a candidate for removal, not tests |
| Kerf.Kernel.ModelingData | AppParCurves | 2,688 | 727 | 73.0 | 73.0 | Least-squares forms the tests' approximations don't reach |
| Kerf.Kernel.ModelingData | GeomBndLib | 2,756 | 593 | 78.5 | 78.6 | no reason recorded |
| Kerf.Kernel.ModelingData | Hermit | 518 | 160 | 69.1 | 69.1 | 11I's tests hold a(u) to its promise under steep weights and tight tolerances. Left: pole-moving branches for ratios beyond the tolerance whose end values stay within it, which no weights tried reach |

## Kerf.Kernel.DataExchange

Packages under 80%, least covered first (uncovered lines in brackets); the rest after.

| Package | Files | Lines | Covered | % |
|---|---:|---:|---:|---:|
| **RWStepKinematics** | 74 | 4,448 | 0 (4,448) | 0.0 |
| **STEPSelections** | 1 | 229 | 20 (209) | 8.7 |
| **RWStepDimTol** | 49 | 2,624 | 376 (2,248) | 14.3 |
| **RWStepElement** | 15 | 523 | 100 (423) | 19.1 |
| **StepElement** | 27 | 894 | 232 (662) | 26.0 |
| **RWStepFEA** | 52 | 1,431 | 509 (922) | 35.6 |
| **RWStepAP203** | 11 | 341 | 124 (217) | 36.4 |
| **STEPEdit** | 1 | 156 | 58 (98) | 37.2 |
| **RWStepRepr** | 70 | 1,703 | 797 (906) | 46.8 |
| **RWStepShape** | 92 | 2,390 | 1,136 (1,254) | 47.5 |
| **StepAP203** | 21 | 285 | 139 (146) | 48.8 |
| **StepAP242** | 6 | 144 | 74 (70) | 51.4 |
| **StepData** | 33 | 3,602 | 1,852 (1,750) | 51.4 |
| **IFSelect** | 40 | 3,935 | 2,111 (1,824) | 53.6 |
| **RWStepGeom** | 89 | 3,446 | 1,879 (1,567) | 54.5 |
| **RWStepBasic** | 123 | 3,182 | 1,806 (1,376) | 56.8 |
| **StepSelect** | 3 | 167 | 96 (71) | 57.5 |
| **StepFEA** | 62 | 748 | 437 (311) | 58.4 |
| **IFGraph** | 11 | 379 | 223 (156) | 58.8 |
| **RWStepVisual** | 102 | 2,952 | 1,743 (1,209) | 59.0 |
| **RWStepAP214** | 29 | 13,030 | 7,702 (5,328) | 59.1 |
| **XSControl** | 11 | 2,072 | 1,226 (846) | 59.2 |
| **StepKinematics** | 85 | 1,257 | 754 (503) | 60.0 |
| **APIHeaderSection** | 2 | 299 | 183 (116) | 61.2 |
| **StepShape** | 103 | 1,090 | 692 (398) | 63.5 |
| **StepToTopoDS** | 18 | 2,231 | 1,418 (813) | 63.6 |
| **StepBasic** | 144 | 1,878 | 1,194 (684) | 63.6 |
| **UnitsMethods** | 1 | 82 | 53 (29) | 64.6 |
| **StepToGeom** | 1 | 1,242 | 807 (435) | 65.0 |
| **StepAP214** | 46 | 1,703 | 1,125 (578) | 66.1 |
| **StepVisual** | 136 | 1,482 | 985 (497) | 66.5 |
| **StepDimTol** | 56 | 560 | 376 (184) | 67.1 |
| **Transfer** | 30 | 2,109 | 1,433 (676) | 67.9 |
| **StepRepr** | 85 | 760 | 519 (241) | 68.3 |
| **Interface** | 43 | 4,139 | 2,858 (1,281) | 69.1 |
| **RWStepAP242** | 4 | 150 | 104 (46) | 69.3 |
| **STEPControl** | 5 | 2,689 | 1,918 (771) | 71.3 |
| **StepGeom** | 91 | 1,281 | 938 (343) | 73.2 |
| **MoniTool** | 10 | 786 | 578 (208) | 73.5 |
| **STEPConstruct** | 11 | 2,209 | 1,663 (546) | 75.3 |
| **TopoDSToStep** | 17 | 1,469 | 1,113 (356) | 75.8 |
| STEPCAFControl | 6 | 1,832 | 1,540 (292) | 84.1 |
| LibCtl | 1 | 95 | 80 (15) | 84.2 |
| XSAlgo | 3 | 437 | 372 (65) | 85.1 |
| TransferBRep | 8 | 526 | 463 (63) | 88.0 |
| RWHeaderSection | 6 | 244 | 217 (27) | 88.9 |
| GeomToStep | 33 | 1,185 | 1,067 (118) | 90.0 |
| XdeDoc | 12 | 1,100 | 991 (109) | 90.1 |
| StepTidy | 4 | 326 | 294 (32) | 90.2 |
| Exchange | 5 | 393 | 366 (27) | 93.1 |
| RWObj | 1 | 121 | 113 (8) | 93.4 |
| HeaderSection | 5 | 79 | 74 (5) | 93.7 |
| DESTEP | 1 | 99 | 93 (6) | 93.9 |
| RWGltf | 4 | 814 | 769 (45) | 94.5 |
| StlAPI | 3 | 74 | 70 (4) | 94.6 |
| RWStl | 3 | 472 | 447 (25) | 94.7 |
| Resource | 2 | 367 | 349 (18) | 95.1 |
| RW3mf | 1 | 247 | 236 (11) | 95.5 |
| StepFile | 5 | 875 | 847 (28) | 96.8 |
| RWMesh | 2 | 138 | 135 (3) | 97.8 |
| DE | 1 | 55 | 55 (0) | 100.0 |
| Results | 1 | 10 | 10 (0) | 100.0 |

## Kerf.Kernel.Foundation

Packages under 80%, least covered first (uncovered lines in brackets); the rest after.

| Package | Files | Lines | Covered | % |
|---|---:|---:|---:|---:|
| MathSys | 6 | 940 | 760 (180) | 80.9 |
| MathUtils | 14 | 1,331 | 1,081 (250) | 81.2 |
| MathPoly | 4 | 341 | 278 (63) | 81.5 |
| MathOpt | 8 | 1,793 | 1,474 (319) | 82.2 |
| MathRoot | 8 | 1,196 | 984 (212) | 82.3 |
| TCollection | 2 | 186 | 155 (31) | 83.3 |
| NCollection | 27 | 2,503 | 2,090 (413) | 83.5 |
| MathInteg | 5 | 616 | 522 (94) | 84.7 |
| Standard | 12 | 893 | 758 (135) | 84.9 |
| Message | 19 | 1,205 | 1,028 (177) | 85.3 |
| BSplCLib | 8 | 4,642 | 3,972 (670) | 85.6 |
| Resource | 1 | 189 | 164 (25) | 86.8 |
| MathLin | 7 | 862 | 748 (114) | 86.8 |
| gp | 44 | 5,996 | 5,287 (709) | 88.2 |
| Poly | 17 | 2,280 | 2,049 (231) | 89.9 |
| TopLoc | 5 | 175 | 158 (17) | 90.3 |
| Bnd | 11 | 3,205 | 2,943 (262) | 91.8 |
| BVH | 31 | 1,439 | 1,323 (116) | 91.9 |
| math | 48 | 10,849 | 10,022 (827) | 92.4 |
| CSLib | 3 | 422 | 393 (29) | 93.1 |
| Convert | 16 | 1,675 | 1,565 (110) | 93.4 |
| OSD | 1 | 46 | 43 (3) | 93.5 |
| Context | 1 | 56 | 53 (3) | 94.6 |
| Precision | 1 | 20 | 19 (1) | 95.0 |
| Results | 3 | 175 | 169 (6) | 96.6 |
| Quantity | 3 | 966 | 941 (25) | 97.4 |
| Geometry | 5 | 130 | 127 (3) | 97.7 |
| BSplSLib | 2 | 2,329 | 2,298 (31) | 98.7 |
| ElCLib | 1 | 646 | 639 (7) | 98.9 |
| PLib | 5 | 13,724 | 13,612 (112) | 99.2 |
| ElSLib | 1 | 916 | 911 (5) | 99.5 |
| Colors | 1 | 44 | 44 (0) | 100.0 |
| Storage | 1 | 3 | 3 (0) | 100.0 |

## Kerf.Kernel.ModelingAlgorithms

Packages under 80%, least covered first (uncovered lines in brackets); the rest after.

| Package | Files | Lines | Covered | % |
|---|---:|---:|---:|---:|
| **TopBas** | 1 | 41 | 0 (41) | 0.0 |
| **TopOpeBRepTool** | 10 | 988 | 269 (719) | 27.2 |
| **TopOpeBRepBuild** | 14 | 1,302 | 381 (921) | 29.3 |
| **BRepApprox** | 25 | 135 | 45 (90) | 33.3 |
| **BiTgte** | 3 | 1,131 | 749 (382) | 66.2 |
| **IntCurveSurface** | 20 | 1,949 | 1,352 (597) | 69.4 |
| **Geom2dInt** | 12 | 183 | 130 (53) | 71.0 |
| **TopOpeBRepDS** | 30 | 2,009 | 1,473 (536) | 73.3 |
| **TopTrans** | 2 | 400 | 298 (102) | 74.5 |
| **Draft** | 6 | 1,429 | 1,078 (351) | 75.4 |
| **HLRBRep** | 54 | 6,844 | 5,252 (1,592) | 76.7 |
| **Geom2dHatch** | 7 | 994 | 774 (220) | 77.9 |
| **FilletSurf** | 2 | 374 | 294 (80) | 78.6 |
| Intrv | 2 | 234 | 188 (46) | 80.3 |
| HatchGen | 4 | 297 | 239 (58) | 80.5 |
| LocalAnalysis | 3 | 590 | 476 (114) | 80.7 |
| Intf | 9 | 2,158 | 1,745 (413) | 80.9 |
| GeomInt | 29 | 1,708 | 1,386 (322) | 81.1 |
| BRepPrim | 10 | 1,456 | 1,184 (272) | 81.3 |
| BRepBuilderAPI | 23 | 4,230 | 3,446 (784) | 81.5 |
| BRepFeat | 13 | 6,582 | 5,365 (1,217) | 81.5 |
| ShapeCustom | 12 | 1,916 | 1,562 (354) | 81.5 |
| ChFi3d | 29 | 16,770 | 13,745 (3,025) | 82.0 |
| IntCurve | 16 | 4,341 | 3,564 (777) | 82.1 |
| Prs3d | 2 | 68 | 56 (12) | 82.4 |
| ShapeExtend | 7 | 879 | 725 (154) | 82.5 |
| ShapeConstruct | 4 | 1,924 | 1,592 (332) | 82.7 |
| LocOpe | 23 | 5,970 | 4,945 (1,025) | 82.8 |
| NLPlate | 9 | 258 | 215 (43) | 83.3 |
| Contap | 13 | 2,170 | 1,809 (361) | 83.4 |
| IntRes2d | 5 | 464 | 388 (76) | 83.6 |
| BRepBlend | 25 | 6,074 | 5,086 (988) | 83.7 |
| IntStart | 2 | 628 | 526 (102) | 83.8 |
| IntPolyh | 11 | 2,757 | 2,310 (447) | 83.8 |
| ShapeFix | 27 | 12,585 | 10,657 (1,928) | 84.7 |
| BRepFilletAPI | 5 | 350 | 297 (53) | 84.9 |
| BRepClass3d | 7 | 1,005 | 853 (152) | 84.9 |
| BRepAlgoAPI | 12 | 552 | 469 (83) | 85.0 |
| Plate | 13 | 1,888 | 1,606 (282) | 85.1 |
| HelixBRep | 1 | 298 | 255 (43) | 85.6 |
| AppBlend | 2 | 599 | 514 (85) | 85.8 |
| IntWalk | 7 | 3,464 | 2,973 (491) | 85.8 |
| IntPatch | 47 | 14,854 | 12,753 (2,101) | 85.9 |
| Hatch | 3 | 166 | 143 (23) | 86.1 |
| HLRAlgo | 18 | 2,281 | 1,967 (314) | 86.2 |
| BRepAlgo | 6 | 1,592 | 1,374 (218) | 86.3 |
| ShapeUpgrade | 31 | 5,928 | 5,117 (811) | 86.3 |
| BRepOffset | 12 | 12,480 | 10,773 (1,707) | 86.3 |
| ShapeAlgo | 3 | 350 | 304 (46) | 86.9 |
| BRepMesh | 61 | 5,859 | 5,114 (745) | 87.3 |
| BOPTools | 12 | 2,521 | 2,203 (318) | 87.4 |
| ShapeBuild | 5 | 570 | 499 (71) | 87.5 |
| ChFiDS | 12 | 1,485 | 1,302 (183) | 87.7 |
| BRepLib | 19 | 4,718 | 4,157 (561) | 88.1 |
| IntImpParGen | 3 | 592 | 522 (70) | 88.2 |
| BRepCheck | 9 | 2,725 | 2,410 (315) | 88.4 |
| ShapeProcessAPI | 1 | 26 | 23 (3) | 88.5 |
| Geom2dAPI | 5 | 700 | 620 (80) | 88.6 |
| TopCnx | 1 | 35 | 31 (4) | 88.6 |
| GccEnt | 4 | 53 | 47 (6) | 88.7 |
| BRepTopAdaptor | 4 | 674 | 598 (76) | 88.7 |
| GeomPlate | 9 | 2,130 | 1,890 (240) | 88.7 |
| ShapeAnalysis | 19 | 6,474 | 5,770 (704) | 89.1 |
| IntSurf | 11 | 651 | 581 (70) | 89.2 |
| BRepOffsetAPI | 13 | 2,353 | 2,106 (247) | 89.5 |
| Bisector | 10 | 2,079 | 1,863 (216) | 89.6 |
| ChFi2d | 7 | 2,022 | 1,816 (206) | 89.8 |
| BRepGProp | 13 | 2,257 | 2,030 (227) | 89.9 |
| BRepSweep | 9 | 968 | 877 (91) | 90.6 |
| BRepExtrema | 17 | 2,552 | 2,313 (239) | 90.6 |
| BlendFunc | 21 | 4,959 | 4,496 (463) | 90.7 |
| BRepPrimAPI | 12 | 667 | 605 (62) | 90.7 |
| Sweep | 3 | 97 | 88 (9) | 90.7 |
| ShapeProcess | 5 | 927 | 841 (86) | 90.7 |
| GeomAPI | 11 | 1,494 | 1,361 (133) | 91.1 |
| StdPrs | 1 | 34 | 31 (3) | 91.2 |
| GeomFill | 58 | 10,349 | 9,458 (891) | 91.4 |
| IntTools | 25 | 6,401 | 5,857 (544) | 91.5 |
| Drawing | 9 | 1,289 | 1,183 (106) | 91.8 |
| ApproxInt | 7 | 1,547 | 1,421 (126) | 91.9 |
| MAT2d | 7 | 1,570 | 1,443 (127) | 91.9 |
| HLRTopoBRep | 6 | 804 | 740 (64) | 92.0 |
| IMeshTools | 10 | 189 | 174 (15) | 92.1 |
| IntCurvesFace | 2 | 378 | 349 (29) | 92.3 |
| BRepClass | 7 | 554 | 512 (42) | 92.4 |
| GccAna | 47 | 4,008 | 3,712 (296) | 92.6 |
| BRepFill | 31 | 12,927 | 11,984 (943) | 92.7 |
| Geom2dGcc | 26 | 6,253 | 5,800 (453) | 92.8 |
| IMeshData | 10 | 87 | 81 (6) | 93.1 |
| TopClass | 2 | 146 | 136 (10) | 93.2 |
| BOPAlgo | 44 | 12,685 | 11,863 (822) | 93.5 |
| Features | 1 | 124 | 116 (8) | 93.5 |
| BOPDS | 17 | 1,623 | 1,524 (99) | 93.9 |
| IntImp | 6 | 706 | 665 (41) | 94.2 |
| Law | 10 | 1,603 | 1,511 (92) | 94.3 |
| Results | 4 | 164 | 155 (9) | 94.5 |
| BRepBndLib | 2 | 555 | 525 (30) | 94.6 |
| Fillets | 1 | 230 | 218 (12) | 94.8 |
| MAT | 8 | 550 | 522 (28) | 94.9 |
| HelixGeom | 6 | 347 | 330 (17) | 95.1 |
| BRepIntCurveSurface | 1 | 112 | 107 (5) | 95.5 |
| ChFiKPart | 14 | 1,877 | 1,796 (81) | 95.7 |
| Geometry | 4 | 543 | 521 (22) | 95.9 |
| Meshing | 2 | 162 | 156 (6) | 96.3 |
| Blend | 11 | 395 | 381 (14) | 96.5 |
| Measurement | 2 | 173 | 167 (6) | 96.5 |
| Construction | 6 | 562 | 544 (18) | 96.8 |
| BRepMAT2d | 3 | 390 | 381 (9) | 97.7 |
| FairCurve | 11 | 1,577 | 1,549 (28) | 98.2 |
| Offsets | 1 | 116 | 114 (2) | 98.3 |
| BRepProj | 1 | 126 | 124 (2) | 98.4 |
| Booleans | 1 | 69 | 68 (1) | 98.6 |
| Healing | 2 | 267 | 265 (2) | 99.3 |
| BRepMeshData | 6 | 120 | 120 (0) | 100.0 |
| BRepPreviewAPI | 1 | 50 | 50 (0) | 100.0 |
| GccInt | 7 | 36 | 36 (0) | 100.0 |
| HLRAppli | 1 | 30 | 30 (0) | 100.0 |
| MeshExtraction | 1 | 147 | 147 (0) | 100.0 |
| XBRepMesh | 1 | 7 | 7 (0) | 100.0 |

## Kerf.Kernel.ModelingData

Packages under 80%, least covered first (uncovered lines in brackets); the rest after.

| Package | Files | Lines | Covered | % |
|---|---:|---:|---:|---:|
| **Hermit** | 1 | 518 | 358 (160) | 69.1 |
| **AppParCurves** | 13 | 2,688 | 1,961 (727) | 73.0 |
| **GeomBndLib** | 36 | 2,756 | 2,163 (593) | 78.5 |
| CPnts | 4 | 382 | 308 (74) | 80.6 |
| LProp | 6 | 398 | 321 (77) | 80.7 |
| ExtremaPC | 13 | 1,086 | 878 (208) | 80.8 |
| Geom2dConvert | 7 | 1,342 | 1,096 (246) | 81.7 |
| GProp | 9 | 561 | 460 (101) | 82.0 |
| GeomLProp | 9 | 770 | 632 (138) | 82.1 |
| GCPnts | 8 | 2,368 | 1,949 (419) | 82.3 |
| TopoDS | 25 | 1,129 | 933 (196) | 82.6 |
| Geom2d | 23 | 2,746 | 2,295 (451) | 83.6 |
| AppDef | 30 | 2,086 | 1,744 (342) | 83.6 |
| Extrema | 53 | 8,797 | 7,372 (1,425) | 83.8 |
| Geom2dEval | 8 | 1,362 | 1,145 (217) | 84.1 |
| Adaptor3d | 8 | 2,220 | 1,894 (326) | 85.3 |
| BndLib | 4 | 871 | 751 (120) | 86.2 |
| ProjLib | 15 | 4,809 | 4,152 (657) | 86.3 |
| BRepLProp | 4 | 88 | 76 (12) | 86.4 |
| gce | 24 | 846 | 746 (100) | 88.2 |
| GeomProjLib | 1 | 111 | 98 (13) | 88.3 |
| GeomLib | 11 | 2,653 | 2,350 (303) | 88.6 |
| BRep | 21 | 1,870 | 1,661 (209) | 88.8 |
| BRepTools | 19 | 4,652 | 4,151 (501) | 89.2 |
| GeomAdaptor | 7 | 2,294 | 2,050 (244) | 89.4 |
| Geom | 47 | 8,965 | 8,059 (906) | 89.9 |
| GeomEval | 14 | 3,292 | 2,974 (318) | 90.3 |
| IntAna | 7 | 2,583 | 2,343 (240) | 90.7 |
| GeomHash | 31 | 731 | 665 (66) | 91.0 |
| GeomConvert | 15 | 3,188 | 2,903 (285) | 91.1 |
| IntAna2d | 4 | 647 | 590 (57) | 91.2 |
| Approx | 14 | 4,072 | 3,723 (349) | 91.4 |
| AdvApp2Var | 21 | 24,819 | 22,934 (1,885) | 92.4 |
| FEmTool | 8 | 965 | 892 (73) | 92.4 |
| Geom2dAdaptor | 2 | 615 | 576 (39) | 93.7 |
| Topology | 8 | 455 | 429 (26) | 94.3 |
| GeomGridEval | 23 | 3,532 | 3,361 (171) | 95.2 |
| TopTools | 5 | 505 | 482 (23) | 95.4 |
| BinTools | 11 | 2,876 | 2,759 (117) | 95.9 |
| GeomTools | 5 | 1,520 | 1,459 (61) | 96.0 |
| BRepAdaptor | 4 | 349 | 335 (14) | 96.0 |
| Geom2dGridEval | 10 | 1,151 | 1,107 (44) | 96.2 |
| Geom2dHash | 13 | 240 | 232 (8) | 96.7 |
| AdvApprox | 5 | 621 | 603 (18) | 97.1 |
| Adaptor2d | 3 | 307 | 302 (5) | 98.4 |
| GC | 33 | 881 | 878 (3) | 99.7 |
| AppCont | 8 | 31,896 | 31,894 (2) | 100.0 |
| Results | 2 | 44 | 44 (0) | 100.0 |
| TopAbs | 1 | 46 | 46 (0) | 100.0 |
| TopExp | 2 | 223 | 223 (0) | 100.0 |

## Kerf.Modeling

Packages under 80%, least covered first (uncovered lines in brackets); the rest after.

| Package | Files | Lines | Covered | % |
|---|---:|---:|---:|---:|
| Build | 1 | 91 | 75 (16) | 82.4 |
| Objects | 3 | 811 | 724 (87) | 89.3 |
| Exchange | 3 | 277 | 250 (27) | 90.3 |
| Geometry | 14 | 1,242 | 1,141 (101) | 91.9 |
| Kernel | 4 | 99 | 91 (8) | 91.9 |
| Topology | 31 | 4,911 | 4,544 (367) | 92.5 |
| Operations | 5 | 709 | 669 (40) | 94.4 |
| (root) | 1 | 13 | 13 (0) | 100.0 |
