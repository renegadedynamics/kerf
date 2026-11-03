# Line coverage

Unit 10A's measurement (phase10-plan.md 3.2; the floor of question 49: every public member tested, at least 80% line
coverage per engine package, lower where the Draw-lite corpus covers a package and the gap is listed). Written by
`tools/coverage.ps1` (`tools/coverage-report.cs`): line coverage of the kernel's and the modeling layer's assemblies
under the tests that don't need native OCCT, merged (a line is covered if any run covered it): the kernel's unit
tests, the Draw-lite tests as they run by default (the corpus classes on their samples, `OCCTNET_CORPUS_FULL` unset)
and the modeling layer's suite on OCCT.NET; the oracle tests only with `-WithOracle`. A package is the folder under
the assembly (OCCT's package for the kernel). Lines are the coverable lines Microsoft's code coverage counts.
Measured on 3 November, before units 10B and 10E: `Build123d.Occt` and `Build123d.OcctNet`, the native runtime and the
forwarding layer, are gone (10B), and the assemblies have been renamed since (10E: `OcctNet.*` is `Kerf.Kernel.*`,
`Build123d` is `Kerf.Modeling`). `tools/coverage.ps1` measures under the new names when run again.

Test projects merged: `Build123d.Tests`, `OcctNet.DataExchange.Tests`, `OcctNet.DrawLite.Tests`, `OcctNet.Foundation.Tests`, `OcctNet.ModelingAlgorithms.Tests`, `OcctNet.ModelingData.Tests`.

## By assembly

| Assembly | Packages | Lines | Covered | % | Packages under 80% |
|---|---:|---:|---:|---:|---:|
| Build123d | 7 | 8,975 | 7,972 | 88.8 | 3 |
| Build123d.Occt | 1 | 23 | 13 | 56.5 | 1 |
| Build123d.OcctNet | 2 | 24,455 | 4,138 | 16.9 | 2 |
| OcctNet.DataExchange | 59 | 84,798 | 49,141 | 58.0 | 41 |
| OcctNet.Foundation | 30 | 61,532 | 47,623 | 77.4 | 13 |
| OcctNet.ModelingAlgorithms | 108 | 258,543 | 213,658 | 82.6 | 28 |
| OcctNet.ModelingData | 48 | 143,467 | 128,035 | 89.2 | 13 |
| **Total** | 255 | 581,793 | 450,580 | 77.4 | 101 |

## Build123d

Packages under 80%, least covered first (uncovered lines in brackets); the rest after.

| Package | Files | Lines | Covered | % |
|---|---:|---:|---:|---:|
| **(root)** | 1 | 13 | 0 (13) | 0.0 |
| **Build** | 1 | 90 | 37 (53) | 41.1 |
| **Objects** | 3 | 808 | 576 (232) | 71.3 |
| Exchange | 3 | 225 | 187 (38) | 83.1 |
| Topology | 30 | 5,847 | 5,268 (579) | 90.1 |
| Operations | 3 | 618 | 583 (35) | 94.3 |
| Geometry | 13 | 1,374 | 1,321 (53) | 96.1 |

## Build123d.Occt

Packages under 80%, least covered first (uncovered lines in brackets); the rest after.

| Package | Files | Lines | Covered | % |
|---|---:|---:|---:|---:|
| **Runtime** | 2 | 23 | 13 (10) | 56.5 |

## Build123d.OcctNet

Packages under 80%, least covered first (uncovered lines in brackets); the rest after.

| Package | Files | Lines | Covered | % |
|---|---:|---:|---:|---:|
| **Generated** | 257 | 22,798 | 3,116 (19,682) | 13.7 |
| **Runtime** | 11 | 1,657 | 1,022 (635) | 61.7 |

## OcctNet.DataExchange

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
| **StepSelect** | 3 | 154 | 87 (67) | 56.5 |
| **RWStepBasic** | 123 | 3,182 | 1,806 (1,376) | 56.8 |
| **StepFEA** | 62 | 748 | 437 (311) | 58.4 |
| **RWStepAP214** | 29 | 13,016 | 7,655 (5,361) | 58.8 |
| **IFGraph** | 11 | 379 | 223 (156) | 58.8 |
| **RWStepVisual** | 102 | 2,952 | 1,743 (1,209) | 59.0 |
| **XSControl** | 11 | 2,076 | 1,232 (844) | 59.3 |
| **StepKinematics** | 85 | 1,257 | 760 (497) | 60.5 |
| **APIHeaderSection** | 2 | 299 | 183 (116) | 61.2 |
| **StepShape** | 103 | 1,090 | 679 (411) | 62.3 |
| **StepBasic** | 144 | 1,878 | 1,193 (685) | 63.5 |
| **StepToTopoDS** | 18 | 2,232 | 1,418 (814) | 63.5 |
| **UnitsMethods** | 1 | 82 | 53 (29) | 64.6 |
| **StepToGeom** | 1 | 1,242 | 806 (436) | 64.9 |
| **StepAP214** | 46 | 1,704 | 1,126 (578) | 66.1 |
| **StepVisual** | 136 | 1,481 | 980 (501) | 66.2 |
| **StepDimTol** | 56 | 560 | 376 (184) | 67.1 |
| **Transfer** | 31 | 2,109 | 1,433 (676) | 67.9 |
| **StepRepr** | 85 | 760 | 517 (243) | 68.0 |
| **Interface** | 42 | 4,105 | 2,827 (1,278) | 68.9 |
| **RWStepAP242** | 4 | 150 | 104 (46) | 69.3 |
| **STEPControl** | 5 | 2,686 | 1,915 (771) | 71.3 |
| **StepGeom** | 91 | 1,281 | 924 (357) | 72.1 |
| **MoniTool** | 10 | 792 | 583 (209) | 73.6 |
| **STEPConstruct** | 11 | 2,211 | 1,663 (548) | 75.2 |
| **TopoDSToStep** | 17 | 1,469 | 1,113 (356) | 75.8 |
| STEPCAFControl | 6 | 1,826 | 1,536 (290) | 84.1 |
| LibCtl | 1 | 95 | 80 (15) | 84.2 |
| XSAlgo | 3 | 437 | 372 (65) | 85.1 |
| TransferBRep | 8 | 526 | 463 (63) | 88.0 |
| RWHeaderSection | 6 | 244 | 217 (27) | 88.9 |
| GeomToStep | 33 | 1,181 | 1,060 (121) | 89.8 |
| XdeDoc | 12 | 1,103 | 994 (109) | 90.1 |
| HeaderSection | 5 | 79 | 73 (6) | 92.4 |
| RWObj | 1 | 121 | 113 (8) | 93.4 |
| DESTEP | 1 | 99 | 93 (6) | 93.9 |
| RWGltf | 4 | 814 | 769 (45) | 94.5 |
| RWStl | 3 | 468 | 443 (25) | 94.7 |
| StlAPI | 3 | 76 | 72 (4) | 94.7 |
| Resource | 2 | 367 | 349 (18) | 95.1 |
| RW3mf | 1 | 247 | 236 (11) | 95.5 |
| StepFile | 5 | 863 | 839 (24) | 97.2 |
| RWMesh | 2 | 138 | 135 (3) | 97.8 |
| DE | 1 | 55 | 55 (0) | 100.0 |

## OcctNet.Foundation

Packages under 80%, least covered first (uncovered lines in brackets); the rest after.

| Package | Files | Lines | Covered | % |
|---|---:|---:|---:|---:|
| **Storage** | 1 | 3 | 1 (2) | 33.3 |
| **gp** | 41 | 6,333 | 3,480 (2,853) | 55.0 |
| **Bnd** | 11 | 3,205 | 1,838 (1,367) | 57.3 |
| **MathUtils** | 14 | 1,331 | 764 (567) | 57.4 |
| **MathOpt** | 8 | 1,777 | 1,041 (736) | 58.6 |
| **MathInteg** | 5 | 616 | 378 (238) | 61.4 |
| **MathRoot** | 8 | 1,183 | 754 (429) | 63.7 |
| **MathSys** | 6 | 940 | 677 (263) | 72.0 |
| **BVH** | 31 | 1,436 | 1,039 (397) | 72.4 |
| **PLib** | 5 | 13,733 | 10,062 (3,671) | 73.3 |
| **MathPoly** | 4 | 341 | 253 (88) | 74.2 |
| **MathLin** | 7 | 861 | 648 (213) | 75.3 |
| **NCollection** | 23 | 2,513 | 1,952 (561) | 77.7 |
| TCollection | 2 | 186 | 155 (31) | 83.3 |
| Standard | 10 | 830 | 700 (130) | 84.3 |
| Message | 19 | 1,192 | 1,012 (180) | 84.9 |
| BSplCLib | 8 | 4,642 | 3,945 (697) | 85.0 |
| Resource | 1 | 184 | 158 (26) | 85.9 |
| Quantity | 3 | 966 | 839 (127) | 86.9 |
| Collections | 1 | 49 | 43 (6) | 87.8 |
| TopLoc | 5 | 166 | 146 (20) | 88.0 |
| Poly | 17 | 2,310 | 2,068 (242) | 89.5 |
| CSLib | 3 | 415 | 377 (38) | 90.8 |
| math | 48 | 10,675 | 9,837 (838) | 92.1 |
| Convert | 16 | 1,692 | 1,577 (115) | 93.2 |
| OSD | 1 | 46 | 43 (3) | 93.5 |
| Precision | 1 | 20 | 19 (1) | 95.0 |
| BSplSLib | 2 | 2,328 | 2,270 (58) | 97.5 |
| ElCLib | 1 | 646 | 639 (7) | 98.9 |
| ElSLib | 1 | 913 | 908 (5) | 99.5 |

## OcctNet.ModelingAlgorithms

Packages under 80%, least covered first (uncovered lines in brackets); the rest after.

| Package | Files | Lines | Covered | % |
|---|---:|---:|---:|---:|
| **TopBas** | 1 | 41 | 0 (41) | 0.0 |
| **TopOpeBRepTool** | 11 | 993 | 269 (724) | 27.1 |
| **TopOpeBRepBuild** | 14 | 1,324 | 401 (923) | 30.3 |
| **BRepApprox** | 25 | 135 | 45 (90) | 33.3 |
| **ShapeAlgo** | 3 | 343 | 133 (210) | 38.8 |
| **Geom2dGcc** | 26 | 6,255 | 2,572 (3,683) | 41.1 |
| **BRepFilletAPI** | 5 | 350 | 210 (140) | 60.0 |
| **Blend** | 11 | 395 | 257 (138) | 65.1 |
| **IntCurveSurface** | 20 | 1,942 | 1,276 (666) | 65.7 |
| **LocOpe** | 23 | 5,999 | 3,976 (2,023) | 66.3 |
| **BiTgte** | 3 | 1,143 | 759 (384) | 66.4 |
| **Geom2dInt** | 12 | 183 | 131 (52) | 71.6 |
| **FilletSurf** | 2 | 372 | 268 (104) | 72.0 |
| **Draft** | 6 | 1,429 | 1,033 (396) | 72.3 |
| **Geom2dHatch** | 7 | 998 | 726 (272) | 72.7 |
| **TopOpeBRepDS** | 30 | 2,009 | 1,473 (536) | 73.3 |
| **ShapeFix** | 27 | 12,691 | 9,378 (3,313) | 73.9 |
| **TopTrans** | 2 | 400 | 298 (102) | 74.5 |
| **BRepFeat** | 13 | 6,599 | 4,933 (1,666) | 74.8 |
| **HLRBRep** | 54 | 6,889 | 5,173 (1,716) | 75.1 |
| **Sweep** | 3 | 97 | 76 (21) | 78.4 |
| **Plate** | 13 | 1,890 | 1,492 (398) | 78.9 |
| **ShapeConstruct** | 4 | 1,921 | 1,519 (402) | 79.1 |
| **Intf** | 9 | 2,145 | 1,703 (442) | 79.4 |
| **Prs3d** | 2 | 68 | 54 (14) | 79.4 |
| **BRepBuilderAPI** | 23 | 4,272 | 3,401 (871) | 79.6 |
| **BRepBlend** | 25 | 6,123 | 4,884 (1,239) | 79.8 |
| **ChFi2d** | 7 | 2,036 | 1,627 (409) | 79.9 |
| ChFi3d | 28 | 16,659 | 13,366 (3,293) | 80.2 |
| Intrv | 2 | 234 | 188 (46) | 80.3 |
| HatchGen | 4 | 297 | 239 (58) | 80.5 |
| LocalAnalysis | 3 | 590 | 476 (114) | 80.7 |
| GeomInt | 29 | 1,708 | 1,382 (326) | 80.9 |
| BRepPrim | 10 | 1,457 | 1,185 (272) | 81.3 |
| ShapeCustom | 12 | 1,921 | 1,566 (355) | 81.5 |
| IntCurve | 16 | 4,344 | 3,556 (788) | 81.9 |
| ShapeExtend | 7 | 881 | 726 (155) | 82.4 |
| BRepGProp | 13 | 2,247 | 1,863 (384) | 82.9 |
| Contap | 13 | 2,167 | 1,801 (366) | 83.1 |
| NLPlate | 9 | 258 | 215 (43) | 83.3 |
| IntStart | 2 | 628 | 524 (104) | 83.4 |
| IntRes2d | 5 | 464 | 388 (76) | 83.6 |
| IntPolyh | 11 | 2,757 | 2,308 (449) | 83.7 |
| BRepClass3d | 7 | 1,005 | 849 (156) | 84.5 |
| GeomPlate | 9 | 2,143 | 1,813 (330) | 84.6 |
| HelixBRep | 1 | 298 | 253 (45) | 84.9 |
| BRepAlgoAPI | 12 | 552 | 469 (83) | 85.0 |
| IntPatch | 47 | 14,863 | 12,690 (2,173) | 85.4 |
| AppBlend | 2 | 599 | 512 (87) | 85.5 |
| IntWalk | 7 | 3,464 | 2,963 (501) | 85.5 |
| ShapeUpgrade | 31 | 5,939 | 5,097 (842) | 85.8 |
| BRepOffset | 12 | 12,501 | 10,737 (1,764) | 85.9 |
| BRepExtrema | 17 | 2,564 | 2,205 (359) | 86.0 |
| Hatch | 3 | 166 | 143 (23) | 86.1 |
| HLRAlgo | 18 | 2,281 | 1,967 (314) | 86.2 |
| BRepAlgo | 6 | 1,603 | 1,385 (218) | 86.4 |
| ShapeBuild | 5 | 553 | 479 (74) | 86.6 |
| ChFiDS | 12 | 1,482 | 1,285 (197) | 86.7 |
| BRepLib | 19 | 4,749 | 4,123 (626) | 86.8 |
| BRepMesh | 61 | 5,802 | 5,041 (761) | 86.9 |
| BRepOffsetAPI | 13 | 2,361 | 2,054 (307) | 87.0 |
| BRepPrimAPI | 12 | 672 | 585 (87) | 87.1 |
| GccAna | 46 | 6,749 | 5,893 (856) | 87.3 |
| ShapeAnalysis | 19 | 6,480 | 5,679 (801) | 87.6 |
| GeomAPI | 11 | 1,495 | 1,311 (184) | 87.7 |
| BOPTools | 12 | 2,527 | 2,216 (311) | 87.7 |
| IntImpParGen | 3 | 594 | 523 (71) | 88.0 |
| BRepCheck | 9 | 2,722 | 2,397 (325) | 88.1 |
| ShapeProcessAPI | 1 | 26 | 23 (3) | 88.5 |
| Geom2dAPI | 5 | 700 | 620 (80) | 88.6 |
| TopCnx | 1 | 35 | 31 (4) | 88.6 |
| GccEnt | 4 | 53 | 47 (6) | 88.7 |
| BRepSweep | 9 | 968 | 860 (108) | 88.8 |
| ShapeProcess | 5 | 1,012 | 902 (110) | 89.1 |
| IntSurf | 11 | 651 | 581 (70) | 89.2 |
| Bisector | 10 | 2,079 | 1,862 (217) | 89.6 |
| BRepBndLib | 2 | 523 | 469 (54) | 89.7 |
| BlendFunc | 21 | 4,961 | 4,474 (487) | 90.2 |
| IntTools | 25 | 6,410 | 5,815 (595) | 90.7 |
| GeomFill | 58 | 10,382 | 9,439 (943) | 90.9 |
| StdPrs | 1 | 34 | 31 (3) | 91.2 |
| BRepFill | 31 | 12,981 | 11,853 (1,128) | 91.3 |
| HLRTopoBRep | 6 | 813 | 743 (70) | 91.4 |
| BRepTopAdaptor | 4 | 682 | 626 (56) | 91.8 |
| MAT2d | 7 | 1,567 | 1,439 (128) | 91.8 |
| ApproxInt | 7 | 1,549 | 1,423 (126) | 91.9 |
| IMeshTools | 10 | 190 | 175 (15) | 92.1 |
| IntCurvesFace | 2 | 378 | 349 (29) | 92.3 |
| BRepClass | 7 | 556 | 514 (42) | 92.4 |
| BOPAlgo | 44 | 12,654 | 11,767 (887) | 93.0 |
| IMeshData | 10 | 86 | 80 (6) | 93.0 |
| TopClass | 2 | 146 | 136 (10) | 93.2 |
| BOPDS | 17 | 1,617 | 1,519 (98) | 93.9 |
| IntImp | 6 | 706 | 665 (41) | 94.2 |
| Law | 10 | 1,603 | 1,511 (92) | 94.3 |
| MAT | 8 | 550 | 522 (28) | 94.9 |
| HelixGeom | 6 | 347 | 330 (17) | 95.1 |
| BRepProj | 1 | 130 | 124 (6) | 95.4 |
| BRepIntCurveSurface | 1 | 112 | 107 (5) | 95.5 |
| ChFiKPart | 14 | 1,879 | 1,796 (83) | 95.6 |
| BRepMAT2d | 3 | 390 | 380 (10) | 97.4 |
| FairCurve | 11 | 1,577 | 1,548 (29) | 98.2 |
| BRepMeshData | 6 | 118 | 118 (0) | 100.0 |
| BRepPreviewAPI | 1 | 50 | 50 (0) | 100.0 |
| GccInt | 7 | 36 | 36 (0) | 100.0 |
| HLRAppli | 1 | 27 | 27 (0) | 100.0 |
| OcctNet | 1 | 137 | 137 (0) | 100.0 |
| XBRepMesh | 1 | 10 | 10 (0) | 100.0 |

## OcctNet.ModelingData

Packages under 80%, least covered first (uncovered lines in brackets); the rest after.

| Package | Files | Lines | Covered | % |
|---|---:|---:|---:|---:|
| **GC** | 33 | 881 | 370 (511) | 42.0 |
| **Hermit** | 1 | 518 | 219 (299) | 42.3 |
| **Adaptor2d** | 3 | 307 | 182 (125) | 59.3 |
| **gce** | 24 | 846 | 518 (328) | 61.2 |
| **BndLib** | 4 | 875 | 582 (293) | 66.5 |
| **TopoDS** | 25 | 1,107 | 787 (320) | 71.1 |
| **LProp** | 6 | 397 | 287 (110) | 72.3 |
| **AppParCurves** | 13 | 2,689 | 1,956 (733) | 72.7 |
| **Geom2d** | 22 | 2,747 | 2,014 (733) | 73.3 |
| **GCPnts** | 8 | 2,368 | 1,790 (578) | 75.6 |
| **CPnts** | 4 | 382 | 298 (84) | 78.0 |
| **ExtremaPC** | 13 | 1,086 | 852 (234) | 78.5 |
| **Geom2dConvert** | 7 | 1,358 | 1,074 (284) | 79.1 |
| GeomLProp | 9 | 768 | 625 (143) | 81.4 |
| GeomBndLib | 36 | 2,704 | 2,208 (496) | 81.7 |
| Geom | 42 | 8,760 | 7,198 (1,562) | 82.2 |
| Extrema | 53 | 8,786 | 7,222 (1,564) | 82.2 |
| Geom2dEval | 8 | 1,376 | 1,145 (231) | 83.2 |
| AppDef | 30 | 2,086 | 1,739 (347) | 83.4 |
| Adaptor3d | 8 | 2,221 | 1,894 (327) | 85.3 |
| GeomConvert | 15 | 3,189 | 2,746 (443) | 86.1 |
| ProjLib | 15 | 4,751 | 4,096 (655) | 86.2 |
| BRepLProp | 4 | 88 | 76 (12) | 86.4 |
| GeomAdaptor | 7 | 2,289 | 1,995 (294) | 87.2 |
| BRep | 21 | 1,805 | 1,577 (228) | 87.4 |
| GeomLib | 11 | 2,653 | 2,339 (314) | 88.2 |
| GeomProjLib | 1 | 111 | 98 (13) | 88.3 |
| BRepTools | 19 | 4,565 | 4,053 (512) | 88.8 |
| IntAna2d | 4 | 661 | 593 (68) | 89.7 |
| GeomEval | 14 | 3,310 | 2,978 (332) | 90.0 |
| GProp | 9 | 1,023 | 921 (102) | 90.0 |
| IntAna | 7 | 2,586 | 2,342 (244) | 90.6 |
| Approx | 14 | 4,069 | 3,689 (380) | 90.7 |
| GeomHash | 31 | 731 | 665 (66) | 91.0 |
| FEmTool | 8 | 968 | 894 (74) | 92.4 |
| AdvApp2Var | 21 | 24,846 | 22,956 (1,890) | 92.4 |
| Geom2dAdaptor | 2 | 615 | 576 (39) | 93.7 |
| BRepAdaptor | 4 | 349 | 331 (18) | 94.8 |
| GeomGridEval | 23 | 3,534 | 3,363 (171) | 95.2 |
| TopTools | 5 | 505 | 481 (24) | 95.2 |
| BinTools | 11 | 2,877 | 2,759 (118) | 95.9 |
| GeomTools | 5 | 1,520 | 1,459 (61) | 96.0 |
| Geom2dGridEval | 10 | 1,151 | 1,107 (44) | 96.2 |
| Geom2dHash | 13 | 240 | 232 (8) | 96.7 |
| AdvApprox | 5 | 621 | 603 (18) | 97.1 |
| AppCont | 8 | 31,895 | 31,893 (2) | 100.0 |
| TopAbs | 1 | 46 | 46 (0) | 100.0 |
| TopExp | 2 | 207 | 207 (0) | 100.0 |
