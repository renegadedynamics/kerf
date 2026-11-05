# Line coverage

The floor of question 49 (phase10-plan.md 3.2): every public member tested, at least 80% line coverage per engine
package, lower where the Draw-lite corpus covers a package and the gap is listed. Written by `tools/coverage.ps1`
(`tools/coverage-report.cs`): line coverage of the kernel's and the modeling layer's assemblies, merged (a line is
covered if any run covered it): the kernel's unit tests, the public API's tests, the Draw-lite tests as they run by
default (the corpus classes on their samples, `KERF_CORPUS_FULL` unset), the modeling layer's suite, and the
oracle tests' replays with `-WithOracle`. A package is the folder under the assembly (OCCT's package for the kernel).
Lines are the coverable lines Microsoft's code coverage counts. The public members no test calls are in
[public-members.md](public-members.md).

Measured on 5 November 2026.
Test projects merged: `Kerf.Kernel.DataExchange.Tests`, `Kerf.Kernel.DrawLite.Tests`, `Kerf.Kernel.Foundation.Tests`, `Kerf.Kernel.ModelingAlgorithms.Tests`, `Kerf.Kernel.ModelingData.Tests`, `Kerf.Kernel.Oracle.Tests`, `Kerf.Kernel.PublicApi.Tests`, `Kerf.Modeling.Tests`.

## By assembly

| Assembly | Packages | Lines | Covered | % | Packages under 80% |
|---|---:|---:|---:|---:|---:|
| Kerf.Kernel.DataExchange | 61 | 85,242 | 49,583 | 58.2 | 41 |
| Kerf.Kernel.Foundation | 33 | 61,544 | 50,188 | 81.5 | 11 |
| Kerf.Kernel.ModelingAlgorithms | 119 | 259,248 | 218,068 | 84.1 | 23 |
| Kerf.Kernel.ModelingData | 50 | 143,883 | 130,190 | 90.5 | 6 |
| Kerf.Modeling | 8 | 8,146 | 7,411 | 91.0 | 0 |
| **Total** | 271 | 558,063 | 455,440 | 81.6 | 81 |

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
| Kerf.Kernel.DataExchange | IFSelect | 3,948 | 1,830 | 53.6 | 53.6 | OCCT's exchange framework: selections, sessions and printing that Draw's data-exchange commands drive and Draw-lite doesn't port; only what a read or write passes through runs |
| Kerf.Kernel.DataExchange | StepData | 3,602 | 1,750 | 51.4 | 51.4 | OCCT's exchange framework: selections, sessions and printing that Draw's data-exchange commands drive and Draw-lite doesn't port; only what a read or write passes through runs |
| Kerf.Kernel.DataExchange | RWStepGeom | 3,446 | 1,575 | 54.3 | 54.3 | One read and write tool per STEP entity; the rest are for entities in no STEP file we test with. Real tests need STEP files holding them (kinematics, FEA, tolerances): owner's call |
| Kerf.Kernel.DataExchange | RWStepBasic | 3,182 | 1,376 | 56.8 | 56.8 | One read and write tool per STEP entity; the rest are for entities in no STEP file we test with. Real tests need STEP files holding them (kinematics, FEA, tolerances): owner's call |
| Kerf.Kernel.DataExchange | Interface | 4,113 | 1,283 | 68.8 | 68.8 | OCCT's exchange framework: selections, sessions and printing that Draw's data-exchange commands drive and Draw-lite doesn't port; only what a read or write passes through runs |
| Kerf.Kernel.DataExchange | RWStepShape | 2,390 | 1,254 | 47.5 | 47.5 | One read and write tool per STEP entity; the rest are for entities in no STEP file we test with. Real tests need STEP files holding them (kinematics, FEA, tolerances): owner's call |
| Kerf.Kernel.DataExchange | RWStepVisual | 2,952 | 1,209 | 59.0 | 59.0 | One read and write tool per STEP entity; the rest are for entities in no STEP file we test with. Real tests need STEP files holding them (kinematics, FEA, tolerances): owner's call |
| Kerf.Kernel.DataExchange | RWStepFEA | 1,431 | 922 | 35.6 | 35.6 | One read and write tool per STEP entity; the rest are for entities in no STEP file we test with. Real tests need STEP files holding them (kinematics, FEA, tolerances): owner's call |
| Kerf.Kernel.DataExchange | RWStepRepr | 1,703 | 906 | 46.8 | 46.8 | One read and write tool per STEP entity; the rest are for entities in no STEP file we test with. Real tests need STEP files holding them (kinematics, FEA, tolerances): owner's call |
| Kerf.Kernel.DataExchange | XSControl | 2,072 | 846 | 59.2 | 59.2 | OCCT's exchange framework: selections, sessions and printing that Draw's data-exchange commands drive and Draw-lite doesn't port; only what a read or write passes through runs |
| Kerf.Kernel.DataExchange | StepToTopoDS | 2,231 | 813 | 63.6 | 63.6 | The translators' rarer forms and write modes; follows the STEP test data |
| Kerf.Kernel.DataExchange | STEPControl | 2,687 | 771 | 71.3 | 71.3 | The translators' rarer forms and write modes; follows the STEP test data |
| Kerf.Kernel.DataExchange | StepBasic | 1,878 | 684 | 63.6 | 63.6 | STEP's entity classes: the fields of entities in no STEP file we test with, as the RWStep packages |
| Kerf.Kernel.DataExchange | Transfer | 2,111 | 678 | 67.9 | 67.9 | OCCT's exchange framework: selections, sessions and printing that Draw's data-exchange commands drive and Draw-lite doesn't port; only what a read or write passes through runs |
| Kerf.Kernel.DataExchange | StepElement | 894 | 662 | 26.0 | 26.0 | STEP's entity classes: the fields of entities in no STEP file we test with, as the RWStep packages |
| Kerf.Kernel.DataExchange | StepAP214 | 1,703 | 578 | 66.1 | 66.1 | STEP's entity classes: the fields of entities in no STEP file we test with, as the RWStep packages |
| Kerf.Kernel.DataExchange | STEPConstruct | 2,209 | 546 | 75.3 | 75.3 | The translators' rarer forms and write modes; follows the STEP test data |
| Kerf.Kernel.DataExchange | StepKinematics | 1,257 | 503 | 60.0 | 60.0 | STEP's entity classes: the fields of entities in no STEP file we test with, as the RWStep packages |
| Kerf.Kernel.DataExchange | StepVisual | 1,482 | 501 | 66.2 | 66.2 | STEP's entity classes: the fields of entities in no STEP file we test with, as the RWStep packages |
| Kerf.Kernel.DataExchange | StepToGeom | 1,242 | 435 | 65.0 | 65.0 | The translators' rarer forms and write modes; follows the STEP test data |
| Kerf.Kernel.DataExchange | RWStepElement | 523 | 423 | 19.1 | 19.1 | One read and write tool per STEP entity; the rest are for entities in no STEP file we test with. Real tests need STEP files holding them (kinematics, FEA, tolerances): owner's call |
| Kerf.Kernel.DataExchange | StepShape | 1,090 | 405 | 62.8 | 62.8 | STEP's entity classes: the fields of entities in no STEP file we test with, as the RWStep packages |
| Kerf.Kernel.DataExchange | StepGeom | 1,281 | 357 | 72.1 | 72.1 | STEP's entity classes: the fields of entities in no STEP file we test with, as the RWStep packages |
| Kerf.Kernel.DataExchange | TopoDSToStep | 1,469 | 356 | 75.8 | 75.8 | The translators' rarer forms and write modes; follows the STEP test data |
| Kerf.Kernel.DataExchange | StepFEA | 748 | 311 | 58.4 | 58.4 | STEP's entity classes: the fields of entities in no STEP file we test with, as the RWStep packages |
| Kerf.Kernel.DataExchange | StepRepr | 760 | 243 | 68.0 | 68.0 | STEP's entity classes: the fields of entities in no STEP file we test with, as the RWStep packages |
| Kerf.Kernel.DataExchange | RWStepAP203 | 341 | 217 | 36.4 | 36.4 | One read and write tool per STEP entity; the rest are for entities in no STEP file we test with. Real tests need STEP files holding them (kinematics, FEA, tolerances): owner's call |
| Kerf.Kernel.DataExchange | STEPSelections | 229 | 209 | 8.7 | 8.7 | OCCT's exchange framework: selections, sessions and printing that Draw's data-exchange commands drive and Draw-lite doesn't port; only what a read or write passes through runs |
| Kerf.Kernel.DataExchange | MoniTool | 787 | 208 | 73.6 | 73.6 | OCCT's exchange framework: selections, sessions and printing that Draw's data-exchange commands drive and Draw-lite doesn't port; only what a read or write passes through runs |
| Kerf.Kernel.DataExchange | StepDimTol | 560 | 184 | 67.1 | 67.1 | STEP's entity classes: the fields of entities in no STEP file we test with, as the RWStep packages |
| Kerf.Kernel.DataExchange | IFGraph | 379 | 156 | 58.8 | 58.8 | OCCT's exchange framework: selections, sessions and printing that Draw's data-exchange commands drive and Draw-lite doesn't port; only what a read or write passes through runs |
| Kerf.Kernel.DataExchange | StepAP203 | 285 | 146 | 48.8 | 48.8 | STEP's entity classes: the fields of entities in no STEP file we test with, as the RWStep packages |
| Kerf.Kernel.DataExchange | APIHeaderSection | 299 | 116 | 61.2 | 61.2 | OCCT's exchange framework: selections, sessions and printing that Draw's data-exchange commands drive and Draw-lite doesn't port; only what a read or write passes through runs |
| Kerf.Kernel.DataExchange | STEPEdit | 156 | 98 | 37.2 | 37.2 | OCCT's exchange framework: selections, sessions and printing that Draw's data-exchange commands drive and Draw-lite doesn't port; only what a read or write passes through runs |
| Kerf.Kernel.DataExchange | StepSelect | 167 | 71 | 57.5 | 57.5 | OCCT's exchange framework: selections, sessions and printing that Draw's data-exchange commands drive and Draw-lite doesn't port; only what a read or write passes through runs |
| Kerf.Kernel.DataExchange | StepAP242 | 144 | 70 | 51.4 | 51.4 | STEP's entity classes: the fields of entities in no STEP file we test with, as the RWStep packages |
| Kerf.Kernel.DataExchange | RWStepAP242 | 150 | 46 | 69.3 | 69.3 | One read and write tool per STEP entity; the rest are for entities in no STEP file we test with. Real tests need STEP files holding them (kinematics, FEA, tolerances): owner's call |
| Kerf.Kernel.DataExchange | UnitsMethods | 82 | 29 | 64.6 | 64.6 | Conversions for units no test file uses |
| Kerf.Kernel.Foundation | PLib | 13,724 | 3,604 | 73.7 | 73.7 | Polynomial bases at degrees and constraint orders the tests' approximations don't reach |
| Kerf.Kernel.Foundation | Bnd | 3,205 | 1,215 | 62.1 | 62.1 | Box variants and transformed forms the tests rarely reach |
| Kerf.Kernel.Foundation | MathOpt | 1,777 | 736 | 58.6 | 58.6 | OCCT's newer numerics: solver variants and failure branches, reached only through the callers the corpus runs. Shallow tests wouldn't judge them |
| Kerf.Kernel.Foundation | MathUtils | 1,331 | 567 | 57.4 | 57.4 | OCCT's newer numerics: solver variants and failure branches, reached only through the callers the corpus runs. Shallow tests wouldn't judge them |
| Kerf.Kernel.Foundation | NCollection | 2,437 | 555 | 77.2 | 77.3 | OCCT's collections, going with 10J and 10P1; not worth new tests |
| Kerf.Kernel.Foundation | MathRoot | 1,183 | 429 | 63.7 | 63.7 | OCCT's newer numerics: solver variants and failure branches, reached only through the callers the corpus runs. Shallow tests wouldn't judge them |
| Kerf.Kernel.Foundation | BVH | 1,436 | 397 | 72.4 | 72.4 | Tree builders and traversal forms the tests don't reach |
| Kerf.Kernel.Foundation | MathSys | 940 | 263 | 72.0 | 72.0 | OCCT's newer numerics: solver variants and failure branches, reached only through the callers the corpus runs. Shallow tests wouldn't judge them |
| Kerf.Kernel.Foundation | MathInteg | 616 | 238 | 61.4 | 61.4 | OCCT's newer numerics: solver variants and failure branches, reached only through the callers the corpus runs. Shallow tests wouldn't judge them |
| Kerf.Kernel.Foundation | MathLin | 861 | 213 | 75.3 | 75.3 | OCCT's newer numerics: solver variants and failure branches, reached only through the callers the corpus runs. Shallow tests wouldn't judge them |
| Kerf.Kernel.Foundation | MathPoly | 341 | 88 | 74.2 | 74.2 | OCCT's newer numerics: solver variants and failure branches, reached only through the callers the corpus runs. Shallow tests wouldn't judge them |
| Kerf.Kernel.ModelingAlgorithms | ShapeFix | 12,709 | 3,282 | 74.2 | 74.2 | Repairs for kinds of broken input neither the unit tests nor the corpus hold |
| Kerf.Kernel.ModelingAlgorithms | LocOpe | 5,971 | 2,028 | 66.0 | 66.0 | Feature forms (prisms and pipes from and until faces, ribs) past what the corpus's feature cases use |
| Kerf.Kernel.ModelingAlgorithms | Geom2dGcc | 6,253 | 1,997 | 68.1 | 68.1 | Tangent-circle solvers by qualifier and curve type: many combinations, few tested |
| Kerf.Kernel.ModelingAlgorithms | HLRBRep | 6,879 | 1,714 | 75.1 | 75.1 | Hidden-line paths past the few hlr cases |
| Kerf.Kernel.ModelingAlgorithms | BRepFeat | 6,612 | 1,621 | 75.5 | 75.5 | Feature forms (prisms and pipes from and until faces, ribs) past what the corpus's feature cases use |
| Kerf.Kernel.ModelingAlgorithms | BRepBlend | 6,123 | 1,239 | 79.8 | 79.8 | Blends on restrictions and points, and rarely reached walking branches; 884 fillet cases already run |
| Kerf.Kernel.ModelingAlgorithms | TopOpeBRepBuild | 1,310 | 922 | 29.6 | 29.6 | The legacy Booleans' remains: only what fillets' and HLR's callers need is alive. Candidates for removal, not tests |
| Kerf.Kernel.ModelingAlgorithms | TopOpeBRepTool | 988 | 719 | 27.2 | 27.2 | The legacy Booleans' remains: only what fillets' and HLR's callers need is alive. Candidates for removal, not tests |
| Kerf.Kernel.ModelingAlgorithms | IntCurveSurface | 1,942 | 654 | 66.3 | 66.3 | Intersection forms by curve and surface type the tests don't reach |
| Kerf.Kernel.ModelingAlgorithms | TopOpeBRepDS | 2,009 | 536 | 73.3 | 73.3 | The legacy Booleans' remains: only what fillets' and HLR's callers need is alive. Candidates for removal, not tests |
| Kerf.Kernel.ModelingAlgorithms | ShapeConstruct | 1,924 | 402 | 79.1 | 79.1 | Healing support, following ShapeFix's gap |
| Kerf.Kernel.ModelingAlgorithms | Plate | 1,890 | 398 | 78.9 | 78.9 | Constraint kinds the fill tests and cases don't reach |
| Kerf.Kernel.ModelingAlgorithms | BiTgte | 1,131 | 382 | 66.2 | 66.2 | Forms past the corpus's few cases; both are small OCCT features |
| Kerf.Kernel.ModelingAlgorithms | Draft | 1,429 | 368 | 74.2 | 74.2 | Forms past the corpus's few cases; both are small OCCT features |
| Kerf.Kernel.ModelingAlgorithms | Geom2dHatch | 998 | 270 | 72.9 | 72.9 | Intersection forms by curve and surface type the tests don't reach |
| Kerf.Kernel.ModelingAlgorithms | ShapeAlgo | 344 | 210 | 39.0 | 39.0 | Healing support, following ShapeFix's gap |
| Kerf.Kernel.ModelingAlgorithms | Blend | 395 | 138 | 65.1 | 65.1 | Blends on restrictions and points, and rarely reached walking branches; 884 fillet cases already run |
| Kerf.Kernel.ModelingAlgorithms | BRepFilletAPI | 350 | 125 | 64.3 | 64.3 | Blends on restrictions and points, and rarely reached walking branches; 884 fillet cases already run |
| Kerf.Kernel.ModelingAlgorithms | FilletSurf | 374 | 104 | 72.2 | 72.2 | Blends on restrictions and points, and rarely reached walking branches; 884 fillet cases already run |
| Kerf.Kernel.ModelingAlgorithms | TopTrans | 400 | 102 | 74.5 | 74.5 | The legacy Booleans' remains: only what fillets' and HLR's callers need is alive. Candidates for removal, not tests |
| Kerf.Kernel.ModelingAlgorithms | BRepApprox | 135 | 90 | 33.3 | 33.3 | A small approximation wrapper, partly run |
| Kerf.Kernel.ModelingAlgorithms | Geom2dInt | 183 | 53 | 71.0 | 71.0 | Intersection forms by curve and surface type the tests don't reach |
| Kerf.Kernel.ModelingAlgorithms | TopBas | 41 | 41 | 0.0 | 0.0 | A single value type nothing calls (HLR's interference): a candidate for removal, not tests |
| Kerf.Kernel.ModelingData | AppParCurves | 2,689 | 727 | 73.0 | 73.0 | Least-squares forms the tests' approximations don't reach |
| Kerf.Kernel.ModelingData | GC | 881 | 459 | 47.9 | 47.9 | Factories' overloads and failure statuses the tests don't reach |
| Kerf.Kernel.ModelingData | gce | 846 | 320 | 62.2 | 62.2 | Factories' overloads and failure statuses the tests don't reach |
| Kerf.Kernel.ModelingData | Hermit | 518 | 299 | 42.3 | 42.3 | Hermite conversions the tests rarely reach |
| Kerf.Kernel.ModelingData | BndLib | 871 | 293 | 66.4 | 66.6 | Boxes of conic and elementary pieces in forms the tests don't reach |
| Kerf.Kernel.ModelingData | Adaptor2d | 307 | 86 | 72.0 | 72.0 | Base-class defaults, mostly overridden |

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
| **IFSelect** | 40 | 3,948 | 2,118 (1,830) | 53.6 |
| **RWStepGeom** | 89 | 3,446 | 1,871 (1,575) | 54.3 |
| **RWStepBasic** | 123 | 3,182 | 1,806 (1,376) | 56.8 |
| **StepSelect** | 3 | 167 | 96 (71) | 57.5 |
| **StepFEA** | 62 | 748 | 437 (311) | 58.4 |
| **IFGraph** | 11 | 379 | 223 (156) | 58.8 |
| **RWStepVisual** | 102 | 2,952 | 1,743 (1,209) | 59.0 |
| **RWStepAP214** | 29 | 13,030 | 7,702 (5,328) | 59.1 |
| **XSControl** | 11 | 2,072 | 1,226 (846) | 59.2 |
| **StepKinematics** | 85 | 1,257 | 754 (503) | 60.0 |
| **APIHeaderSection** | 2 | 299 | 183 (116) | 61.2 |
| **StepShape** | 103 | 1,090 | 685 (405) | 62.8 |
| **StepToTopoDS** | 18 | 2,231 | 1,418 (813) | 63.6 |
| **StepBasic** | 144 | 1,878 | 1,194 (684) | 63.6 |
| **UnitsMethods** | 1 | 82 | 53 (29) | 64.6 |
| **StepToGeom** | 1 | 1,242 | 807 (435) | 65.0 |
| **StepAP214** | 46 | 1,703 | 1,125 (578) | 66.1 |
| **StepVisual** | 136 | 1,482 | 981 (501) | 66.2 |
| **StepDimTol** | 56 | 560 | 376 (184) | 67.1 |
| **Transfer** | 31 | 2,111 | 1,433 (678) | 67.9 |
| **StepRepr** | 85 | 760 | 517 (243) | 68.0 |
| **Interface** | 42 | 4,113 | 2,830 (1,283) | 68.8 |
| **RWStepAP242** | 4 | 150 | 104 (46) | 69.3 |
| **STEPControl** | 5 | 2,687 | 1,916 (771) | 71.3 |
| **StepGeom** | 91 | 1,281 | 924 (357) | 72.1 |
| **MoniTool** | 10 | 787 | 579 (208) | 73.6 |
| **STEPConstruct** | 11 | 2,209 | 1,663 (546) | 75.3 |
| **TopoDSToStep** | 17 | 1,469 | 1,113 (356) | 75.8 |
| STEPCAFControl | 6 | 1,827 | 1,537 (290) | 84.1 |
| LibCtl | 1 | 95 | 80 (15) | 84.2 |
| XSAlgo | 3 | 437 | 372 (65) | 85.1 |
| TransferBRep | 8 | 526 | 463 (63) | 88.0 |
| RWHeaderSection | 6 | 244 | 217 (27) | 88.9 |
| GeomToStep | 33 | 1,185 | 1,067 (118) | 90.0 |
| XdeDoc | 12 | 1,100 | 991 (109) | 90.1 |
| HeaderSection | 5 | 79 | 73 (6) | 92.4 |
| Exchange | 5 | 390 | 363 (27) | 93.1 |
| RWObj | 1 | 121 | 113 (8) | 93.4 |
| DESTEP | 1 | 99 | 93 (6) | 93.9 |
| RWGltf | 4 | 814 | 769 (45) | 94.5 |
| RWStl | 3 | 472 | 447 (25) | 94.7 |
| StlAPI | 3 | 76 | 72 (4) | 94.7 |
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
| **MathUtils** | 14 | 1,331 | 764 (567) | 57.4 |
| **MathOpt** | 8 | 1,777 | 1,041 (736) | 58.6 |
| **MathInteg** | 5 | 616 | 378 (238) | 61.4 |
| **Bnd** | 11 | 3,205 | 1,990 (1,215) | 62.1 |
| **MathRoot** | 8 | 1,183 | 754 (429) | 63.7 |
| **MathSys** | 6 | 940 | 677 (263) | 72.0 |
| **BVH** | 31 | 1,436 | 1,039 (397) | 72.4 |
| **PLib** | 5 | 13,724 | 10,120 (3,604) | 73.7 |
| **MathPoly** | 4 | 341 | 253 (88) | 74.2 |
| **MathLin** | 7 | 861 | 648 (213) | 75.3 |
| **NCollection** | 27 | 2,437 | 1,882 (555) | 77.2 |
| TCollection | 2 | 186 | 155 (31) | 83.3 |
| Message | 19 | 1,203 | 1,022 (181) | 85.0 |
| Standard | 12 | 901 | 766 (135) | 85.0 |
| BSplCLib | 8 | 4,642 | 3,972 (670) | 85.6 |
| Resource | 1 | 189 | 164 (25) | 86.8 |
| gp | 44 | 5,996 | 5,295 (701) | 88.3 |
| Poly | 17 | 2,301 | 2,061 (240) | 89.6 |
| TopLoc | 5 | 169 | 152 (17) | 89.9 |
| math | 48 | 10,697 | 9,869 (828) | 92.3 |
| CSLib | 3 | 415 | 386 (29) | 93.0 |
| Convert | 16 | 1,675 | 1,565 (110) | 93.4 |
| OSD | 1 | 46 | 43 (3) | 93.5 |
| Context | 1 | 56 | 53 (3) | 94.6 |
| Precision | 1 | 20 | 19 (1) | 95.0 |
| Results | 3 | 163 | 157 (6) | 96.3 |
| Quantity | 3 | 966 | 941 (25) | 97.4 |
| Geometry | 5 | 130 | 127 (3) | 97.7 |
| BSplSLib | 2 | 2,329 | 2,298 (31) | 98.7 |
| ElCLib | 1 | 646 | 639 (7) | 98.9 |
| ElSLib | 1 | 916 | 911 (5) | 99.5 |
| Colors | 1 | 44 | 44 (0) | 100.0 |
| Storage | 1 | 3 | 3 (0) | 100.0 |

## Kerf.Kernel.ModelingAlgorithms

Packages under 80%, least covered first (uncovered lines in brackets); the rest after.

| Package | Files | Lines | Covered | % |
|---|---:|---:|---:|---:|
| **TopBas** | 1 | 41 | 0 (41) | 0.0 |
| **TopOpeBRepTool** | 10 | 988 | 269 (719) | 27.2 |
| **TopOpeBRepBuild** | 14 | 1,310 | 388 (922) | 29.6 |
| **BRepApprox** | 25 | 135 | 45 (90) | 33.3 |
| **ShapeAlgo** | 3 | 344 | 134 (210) | 39.0 |
| **BRepFilletAPI** | 5 | 350 | 225 (125) | 64.3 |
| **Blend** | 11 | 395 | 257 (138) | 65.1 |
| **LocOpe** | 23 | 5,971 | 3,943 (2,028) | 66.0 |
| **BiTgte** | 3 | 1,131 | 749 (382) | 66.2 |
| **IntCurveSurface** | 20 | 1,942 | 1,288 (654) | 66.3 |
| **Geom2dGcc** | 26 | 6,253 | 4,256 (1,997) | 68.1 |
| **Geom2dInt** | 12 | 183 | 130 (53) | 71.0 |
| **FilletSurf** | 2 | 374 | 270 (104) | 72.2 |
| **Geom2dHatch** | 7 | 998 | 728 (270) | 72.9 |
| **TopOpeBRepDS** | 30 | 2,009 | 1,473 (536) | 73.3 |
| **ShapeFix** | 27 | 12,709 | 9,427 (3,282) | 74.2 |
| **Draft** | 6 | 1,429 | 1,061 (368) | 74.2 |
| **TopTrans** | 2 | 400 | 298 (102) | 74.5 |
| **HLRBRep** | 54 | 6,879 | 5,165 (1,714) | 75.1 |
| **BRepFeat** | 13 | 6,612 | 4,991 (1,621) | 75.5 |
| **Plate** | 13 | 1,890 | 1,492 (398) | 78.9 |
| **ShapeConstruct** | 4 | 1,924 | 1,522 (402) | 79.1 |
| **BRepBlend** | 25 | 6,123 | 4,884 (1,239) | 79.8 |
| Intrv | 2 | 234 | 188 (46) | 80.3 |
| ChFi3d | 28 | 16,673 | 13,401 (3,272) | 80.4 |
| HatchGen | 4 | 297 | 239 (58) | 80.5 |
| Intf | 9 | 2,145 | 1,730 (415) | 80.7 |
| LocalAnalysis | 3 | 590 | 476 (114) | 80.7 |
| GeomInt | 29 | 1,708 | 1,386 (322) | 81.1 |
| BRepPrim | 10 | 1,456 | 1,184 (272) | 81.3 |
| BRepBuilderAPI | 23 | 4,230 | 3,446 (784) | 81.5 |
| ShapeCustom | 12 | 1,916 | 1,562 (354) | 81.5 |
| IntCurve | 16 | 4,343 | 3,563 (780) | 82.0 |
| Prs3d | 2 | 68 | 56 (12) | 82.4 |
| ShapeExtend | 7 | 879 | 725 (154) | 82.5 |
| Contap | 13 | 2,170 | 1,804 (366) | 83.1 |
| NLPlate | 9 | 258 | 215 (43) | 83.3 |
| IntRes2d | 5 | 464 | 388 (76) | 83.6 |
| IntStart | 2 | 628 | 526 (102) | 83.8 |
| IntPolyh | 11 | 2,757 | 2,310 (447) | 83.8 |
| BRepClass3d | 7 | 1,005 | 853 (152) | 84.9 |
| BRepAlgoAPI | 12 | 552 | 469 (83) | 85.0 |
| HelixBRep | 1 | 298 | 255 (43) | 85.6 |
| ShapeUpgrade | 31 | 5,926 | 5,084 (842) | 85.8 |
| AppBlend | 2 | 599 | 514 (85) | 85.8 |
| IntWalk | 7 | 3,464 | 2,973 (491) | 85.8 |
| IntPatch | 47 | 14,862 | 12,765 (2,097) | 85.9 |
| Hatch | 3 | 166 | 143 (23) | 86.1 |
| HLRAlgo | 18 | 2,281 | 1,967 (314) | 86.2 |
| BRepOffset | 12 | 12,486 | 10,776 (1,710) | 86.3 |
| BRepAlgo | 6 | 1,594 | 1,376 (218) | 86.3 |
| ShapeBuild | 5 | 556 | 482 (74) | 86.7 |
| ChFiDS | 12 | 1,484 | 1,287 (197) | 86.7 |
| BOPTools | 12 | 2,525 | 2,203 (322) | 87.2 |
| BRepMesh | 61 | 5,853 | 5,107 (746) | 87.3 |
| BRepLib | 19 | 4,718 | 4,157 (561) | 88.1 |
| IntImpParGen | 3 | 592 | 522 (70) | 88.2 |
| GeomPlate | 9 | 2,142 | 1,891 (251) | 88.3 |
| ShapeAnalysis | 19 | 6,482 | 5,734 (748) | 88.5 |
| ShapeProcessAPI | 1 | 26 | 23 (3) | 88.5 |
| BRepCheck | 9 | 2,725 | 2,411 (314) | 88.5 |
| Geom2dAPI | 5 | 700 | 620 (80) | 88.6 |
| TopCnx | 1 | 35 | 31 (4) | 88.6 |
| GccEnt | 4 | 53 | 47 (6) | 88.7 |
| IntSurf | 11 | 651 | 581 (70) | 89.2 |
| BRepOffsetAPI | 13 | 2,353 | 2,101 (252) | 89.3 |
| ChFi2d | 7 | 2,036 | 1,821 (215) | 89.4 |
| Bisector | 10 | 2,079 | 1,863 (216) | 89.6 |
| BRepGProp | 13 | 2,247 | 2,020 (227) | 89.9 |
| BlendFunc | 21 | 4,959 | 4,473 (486) | 90.2 |
| BRepSweep | 9 | 968 | 877 (91) | 90.6 |
| BRepExtrema | 17 | 2,564 | 2,325 (239) | 90.7 |
| BRepPrimAPI | 12 | 667 | 605 (62) | 90.7 |
| Sweep | 3 | 97 | 88 (9) | 90.7 |
| ShapeProcess | 5 | 927 | 841 (86) | 90.7 |
| GeomAPI | 11 | 1,494 | 1,361 (133) | 91.1 |
| GeomFill | 58 | 10,381 | 9,458 (923) | 91.1 |
| StdPrs | 1 | 34 | 31 (3) | 91.2 |
| Drawing | 9 | 1,243 | 1,137 (106) | 91.5 |
| IntTools | 25 | 6,410 | 5,866 (544) | 91.5 |
| HLRTopoBRep | 6 | 804 | 737 (67) | 91.7 |
| BRepTopAdaptor | 4 | 682 | 626 (56) | 91.8 |
| ApproxInt | 7 | 1,549 | 1,423 (126) | 91.9 |
| MAT2d | 7 | 1,570 | 1,443 (127) | 91.9 |
| IMeshTools | 10 | 189 | 174 (15) | 92.1 |
| IntCurvesFace | 2 | 378 | 349 (29) | 92.3 |
| BRepClass | 7 | 556 | 514 (42) | 92.4 |
| GccAna | 47 | 4,008 | 3,712 (296) | 92.6 |
| BRepFill | 31 | 12,936 | 11,982 (954) | 92.6 |
| IMeshData | 10 | 87 | 81 (6) | 93.1 |
| TopClass | 2 | 146 | 136 (10) | 93.2 |
| BOPAlgo | 44 | 12,671 | 11,845 (826) | 93.5 |
| Features | 1 | 124 | 116 (8) | 93.5 |
| BOPDS | 17 | 1,623 | 1,524 (99) | 93.9 |
| IntImp | 6 | 706 | 665 (41) | 94.2 |
| Law | 10 | 1,603 | 1,511 (92) | 94.3 |
| BRepBndLib | 2 | 523 | 493 (30) | 94.3 |
| Results | 4 | 164 | 155 (9) | 94.5 |
| Fillets | 1 | 230 | 218 (12) | 94.8 |
| MAT | 8 | 550 | 522 (28) | 94.9 |
| HelixGeom | 6 | 347 | 330 (17) | 95.1 |
| BRepIntCurveSurface | 1 | 112 | 107 (5) | 95.5 |
| ChFiKPart | 14 | 1,877 | 1,796 (81) | 95.7 |
| Geometry | 4 | 543 | 521 (22) | 95.9 |
| Meshing | 2 | 160 | 154 (6) | 96.2 |
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
| **Hermit** | 1 | 518 | 219 (299) | 42.3 |
| **GC** | 33 | 881 | 422 (459) | 47.9 |
| **gce** | 24 | 846 | 526 (320) | 62.2 |
| **BndLib** | 4 | 871 | 578 (293) | 66.4 |
| **Adaptor2d** | 3 | 307 | 221 (86) | 72.0 |
| **AppParCurves** | 13 | 2,689 | 1,962 (727) | 73.0 |
| Geom2dConvert | 7 | 1,356 | 1,089 (267) | 80.3 |
| CPnts | 4 | 382 | 307 (75) | 80.4 |
| LProp | 6 | 398 | 321 (77) | 80.7 |
| ExtremaPC | 13 | 1,086 | 878 (208) | 80.8 |
| GCPnts | 8 | 2,368 | 1,916 (452) | 80.9 |
| GeomBndLib | 36 | 2,726 | 2,227 (499) | 81.7 |
| GProp | 9 | 561 | 460 (101) | 82.0 |
| GeomLProp | 9 | 770 | 632 (138) | 82.1 |
| TopoDS | 25 | 1,133 | 937 (196) | 82.7 |
| Geom2dEval | 8 | 1,376 | 1,145 (231) | 83.2 |
| Extrema | 53 | 8,783 | 7,314 (1,469) | 83.3 |
| Geom2d | 23 | 2,746 | 2,295 (451) | 83.6 |
| AppDef | 30 | 2,086 | 1,744 (342) | 83.6 |
| Adaptor3d | 8 | 2,220 | 1,894 (326) | 85.3 |
| ProjLib | 15 | 4,805 | 4,148 (657) | 86.3 |
| BRepLProp | 4 | 88 | 76 (12) | 86.4 |
| GeomProjLib | 1 | 111 | 98 (13) | 88.3 |
| GeomLib | 11 | 2,653 | 2,350 (303) | 88.6 |
| BRep | 21 | 1,834 | 1,625 (209) | 88.6 |
| BRepTools | 19 | 4,640 | 4,137 (503) | 89.2 |
| GeomAdaptor | 7 | 2,296 | 2,050 (246) | 89.3 |
| Geom | 47 | 8,965 | 8,056 (909) | 89.9 |
| GeomEval | 14 | 3,306 | 2,974 (332) | 90.0 |
| IntAna | 7 | 2,583 | 2,343 (240) | 90.7 |
| GeomHash | 31 | 731 | 665 (66) | 91.0 |
| GeomConvert | 15 | 3,188 | 2,903 (285) | 91.1 |
| Approx | 14 | 4,077 | 3,713 (364) | 91.1 |
| IntAna2d | 4 | 664 | 607 (57) | 91.4 |
| AdvApp2Var | 21 | 24,819 | 22,934 (1,885) | 92.4 |
| FEmTool | 8 | 965 | 892 (73) | 92.4 |
| Geom2dAdaptor | 2 | 615 | 576 (39) | 93.7 |
| Topology | 8 | 451 | 425 (26) | 94.2 |
| GeomGridEval | 23 | 3,534 | 3,363 (171) | 95.2 |
| TopTools | 5 | 505 | 482 (23) | 95.4 |
| BinTools | 11 | 2,876 | 2,759 (117) | 95.9 |
| GeomTools | 5 | 1,520 | 1,459 (61) | 96.0 |
| BRepAdaptor | 4 | 349 | 335 (14) | 96.0 |
| Geom2dGridEval | 10 | 1,151 | 1,107 (44) | 96.2 |
| Geom2dHash | 13 | 240 | 232 (8) | 96.7 |
| AdvApprox | 5 | 621 | 603 (18) | 97.1 |
| AppCont | 8 | 31,896 | 31,894 (2) | 100.0 |
| Results | 2 | 44 | 44 (0) | 100.0 |
| TopAbs | 1 | 46 | 46 (0) | 100.0 |
| TopExp | 2 | 207 | 207 (0) | 100.0 |

## Kerf.Modeling

Packages under 80%, least covered first (uncovered lines in brackets); the rest after.

| Package | Files | Lines | Covered | % |
|---|---:|---:|---:|---:|
| Build | 1 | 91 | 75 (16) | 82.4 |
| Objects | 3 | 813 | 711 (102) | 87.5 |
| Exchange | 3 | 275 | 248 (27) | 90.2 |
| Topology | 31 | 4,904 | 4,477 (427) | 91.3 |
| Geometry | 14 | 1,242 | 1,137 (105) | 91.5 |
| Kernel | 4 | 99 | 91 (8) | 91.9 |
| Operations | 5 | 709 | 659 (50) | 92.9 |
| (root) | 1 | 13 | 13 (0) | 100.0 |
