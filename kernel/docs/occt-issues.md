# Suspected bugs in OCCT

The port reproduces OCCT's behaviour, bugs included, so results match the reference. Each suspected bug is
marked in the code with a comment starting `// OCCT` and listed here. Whether to fix one in the port (or report
it upstream) is a later decision, made with the oracle tests showing what changes.

**Triage (phase 10, unit 10K; [occt-issues-triage.md](occt-issues-triage.md)).** The last column of every table says
what happens to the row: `fix K1` to `fix K8` (fixed by the batch of that name, in the triage's order), `touch`
(fixed when its code is next rewritten), `leave` (harmless, a test script's or Draw's, or in code that goes), or
`settled` (the port already doesn't reproduce the bug). A row that is fixed gets `fixed (decision N)` there, in the
same commit as the fix and its test.

| Where | What | Effect | Triage |
|---|---|---|---|
| `gp_Dir::Mirror(const gp_Ax1&)` (gp_Dir.cxx) | Takes the axis direction's third component from `Y()` instead of `Z()` | Mirroring a direction about an axis is wrong unless the axis's Y and Z components are equal. `gp_Vec`'s and `gp_Pnt`'s versions are correct, so a mirrored axis or system can get inconsistent directions. | fixed (decision 1485) |
| `gp_Mat2d::Multiply(const gp_Mat2d&)` used as `M.Multiply(M)` (gp_Mat2d.cxx `Power`; gp_Trsf2d.cxx and gp_GTrsf2d.cxx `Power`) | `Multiply` writes `myMat[0][1]` before reading `theOther.myMat[0][1]`; squaring a matrix in place passes itself as `theOther` | Most 2D matrix and transformation powers of 2 or more are wrong. The port reproduces it, because an `in` parameter aliases a local just as a C++ reference does. | fixed (decision 1485) |
| `gp_Trsf2d::Multiply`, rotation × rotation | Tests this transformation's translation for zero where gp_Trsf tests the other's | When this rotation has no translation, the other rotation's translation is dropped | fixed (decision 1485) |
| `gp_Trsf2d::Power`, compound branch | Scales only the diagonal (`SetDiagonal(scale*m11, scale*m22)`), then applies the scale again in the loop | Powers of a scaled 2D transformation are wrong | fixed (decision 1485) |
| `gp_Hypr2d::Asymptote1`, `Asymptote2` | Build the direction from YDirection alone (Y ± (b/a)·Y), where gp_Hypr uses X ± (b/a)·Y | The asymptotes point along Y; `Asymptote2` throws when the radii are equal | fixed (decision 1485) |
| `gp_Hypr2d::Directrix2` | Placed at Focus1 + (Parameter/e)·X, where gp_Hypr uses Location − (a/e)·X | The second directrix is in the wrong place | fixed (decision 1485) |
| `gp_GTrsf::Power`, `gp_Other` form | Treats any negative N other than -1 as \|N\|, without inverting | A general transformation raised to a negative power below -1 is not inverted | fixed (decision 1485) |
| `gp_Hypr::gp_Hypr()` | Sets the minor radius to `RealFirst()`, which is negative and which no constructor or Set method accepts | A default hyperbola is invalid; harmless until it is used | leave |
| `gp_Hypr::Asymptote2` | Its error message says "Asymptote1" | Cosmetic | leave |
| `ElSLib::TorusDN`, mixed branch (Nu > 0 and Nv > 0) | The odd/odd and even/even sub-branches never change sign, where the derivative's sign depends on (Nu + Nv) mod 4 | Mixed torus derivatives are reversed when Nu and Nv are both odd with Nu + Nv ≡ 0 (mod 4), e.g. (1, 3), (3, 1), or both even with Nu + Nv ≡ 2 (mod 4), e.g. (2, 4). (1, 1), (2, 2) and the odd/even orders are right. | fixed (decision 1486) |
| `math_Matrix::Multiply(const math_Matrix&)` (math_Matrix.lxx) | Writes the product's columns at Right's column indexes | In-place multiplication by a matrix whose column range differs from this one's (non-square, or other bounds) writes the wrong columns or out of range (the port throws `Standard_OutOfRange`) | touch |
| `math_VectorBase<int>::Divided` (math_VectorBase.lxx) | Multiplies by `1. / theRight` converted to the item type | For `math_IntegerVector`, division by any integer other than ±1 gives zeros. Unused in OCCT | leave |
| `math_Uzawa::Perform` (math_Uzawa.cxx) | `Errinit` has as many values as unknowns but is indexed by equation | With more equations than unknowns OCCT writes past the vector (range checks compiled out); the port throws `Standard_OutOfRange` | touch |
| `math_NewtonFunctionSetRoot::FunctionSetErrors` (both overloads) | Return `DeltaX`, the last Newton step, not `FValues` | Callers asking for the function errors get the step | touch |
| `math_Householder(A, B: math_Matrix, EPS)` and `Perform` (math_Householder.cxx) | Q gets A's column range but is indexed from 1; B's columns are indexed from 1; dimensions are checked only after copying B | A with LowerCol ≠ 1, or B with LowerCol ≠ 1, reads out of range; a mismatched B fails on the index check before the `DimensionError` check | touch |
| `math_SVD::PseudoInverse` (math_SVD.cxx) | Writes `Result(i, j)` from 1 whatever Result's bounds | A result matrix with other bounds is wrong or out of range | touch |
| `math_FunctionSetRoot` two-point `MinimizeDirection` (math_FunctionSetRoot.cxx) | Ignores the return value of `F.Value(tsol, fsol)` | On failure `fsol` is uninitialised in C++ (0 in the port) | touch |
| `math_GaussMultipleIntegration` recursion (math_GaussMultipleIntegration.cxx) | Keeps only the status of the last loop iteration | A failure of the function at an earlier Gauss point is ignored: that point is left out of the sum and the result still reports done | touch |
| `math_ComputeKronrodPointsAndWeights(0)` (math_ComputeKronrodPointsAndWeights.cxx) | Reads `b[Number + 1]` past its buffer | Undefined behaviour in C++; the port throws `IndexOutOfRangeException`, which the constructor's `catch (Standard_Failure)` doesn't catch. OCCT never passes 0 | leave |
| `math_FunctionRoots` constructor, maximum with f(x) < 0 (math_FunctionRoots.cxx) | The second estimate repeats the one from Xi-1, where the positive branch uses Xi+1 | The second test can never succeed where the first failed, so some near-roots beside a negative maximum are not re-sampled | touch |
| `math_TrigonometricFunctionRoots::Perform`, "Cas particulier de PI" | Checks for PI already being a solution only on the first pass of the loop | When A−C+E≈0 and PI wasn't found earlier, PI is added once per free slot: duplicate solutions | touch |
| `math_GlobOptMin::ComputeInitSol` (math_GlobOptMin.cxx) | Calls `computeLocalExtremum(aPnt, aVal, aPnt)`: the start and output points are the same vector | A local method that fails after writing a point outside the box leaves the next method starting from that point, so initial solutions depend on the method order | touch |
| `math_NewtonMinimum::IsConvex()` (math_NewtonMinimum.hxx) | Declared, never defined | Calling it fails to link in C++; left out of the port | leave |
| `Bnd_Sphere::Add` (Bnd_Sphere.cxx) | Treats a negative radius as "not initialised yet", but the default constructor sets the radius to 0 | Adding a sphere to a default-constructed `Bnd_Sphere` also encloses the origin instead of copying the other sphere | touch |
| `Bnd_OBB::ReBuild`, two coincident points (Bnd_OBB.cxx) | Calls `ProcessOnePoint`, which resets `myHDims` to 0, then enlarges by `myHDims[1] + Precision::Confusion()` | The point tolerances are dropped: the half-sizes are `Precision::Confusion()` instead of the larger tolerance plus Confusion | touch |
| `NCollection_Mat3::Adjoint` | Documented as the adjugate, returns the cofactor matrix (its transpose); `Inverted` transposes it back | Callers expecting the adjugate get its transpose | touch |
| `NCollection_Mat4::Adjoint` | `crossVec4` returns the negated minors, so the result is the negated cofactor matrix, not the adjugate and not Mat3's convention | Wrong sign and transposition; nothing in OCCT calls it | leave |
| `BVH::SquareDistanceToObject`, `SquareDistanceToGeomerty` (BVH_DistanceField.lxx) | Return `false` (a zero distance) when the tree is null | Dead code (`BVH()` never returns null); cosmetic | leave |
| `MathLin::LU`, `Solve`, `SolveMultiple`, `Invert` (MathLin_Gauss.hxx) | Row indexes are also used as column indexes; `aFirstNonZero == 0` doubles as "no non-zero found yet" | Wrong or out-of-range results when LowerCol ≠ LowerRow; with a row range starting at 0 or below, a first non-zero at index 0 is lost and the forward substitution is wrong | touch |
| `MathLin::SolveCrout` (MathLin_Crout.hxx) | Indexes the inverse's columns from LowerRow, though it has A's column bounds | Wrong or out-of-range results when LowerCol ≠ LowerRow | touch |
| `MathLin::MatrixPower` (MathLin_Jacobi.hxx) | Takes the eigenvector matrix's column indexes from the eigenvalue (row) range | Wrong or out-of-range results when LowerCol ≠ LowerRow | touch |
| `MathLin::OptimalRegularization` (MathLin_LeastSquares.hxx) | Documented as leave-one-out cross-validation, but scores each lambda by the plain residual of the regularized fit | The residual grows with lambda, so the smallest lambda of the grid is (almost) always returned | touch |
| `MathLin::ConditionNumber` (MathLin_SVD.hxx) | Skips zero singular values when taking the minimum | A singular matrix gets a finite condition number instead of the documented infinity | touch |
| MathLin_Jacobi.hxx | Uses `EigenResult` meaning MathUtils::EigenResult, which works only because it doesn't include MathLin_EigenSearch.hxx; included after that header, MathLin::EigenResult hides it | A translation unit including both headers in that order fails to compile; the port writes `MathUtils.EigenResult` | leave |
| `MathPoly::Quintic`, `MathPoly::Sextic` (MathPoly_Laguerre.hxx) | Copy at most 4 real roots into a PolyResult, which holds 4 | A fifth or sixth real root is silently dropped (`MathPoly::Laguerre` returns all of them) | touch |
| `MathRoot::Trigonometric`, A = B = 0 with C ≠ 0 and D ≠ 0 (MathRoot_Trig.hxx) | Tests `aPoly.Status == InfiniteSolutions` after already returning when `!aPoly.IsDone()` | Dead code: InfiniteRoots can't be set on this path. Cosmetic | leave |
| `MathRoot::Trigonometric`, A = E = 0 with C = 0 or D = 0 (MathRoot_Trig.hxx) | Adds roots without the duplicate removal and sorting of the general case | When \|D/2B\| or \|C/2B\| is about 1 the result has duplicate roots (0 or PI/2 twice) and may be unsorted. (These branches also shift a root by 2PI and then by trunc(Mod)·2PI, as math_TrigonometricFunctionRoots does, so a lower bound of 2PI or more may push it out of range; unverified) | touch |
| `MathOpt::NewtonNumericalHessian` (MathOpt_Newton.hxx) via `MathUtils::NumericalHessian` | NumericalHessian fills the matrix from index 1, but Newton's Hessian has the starting point's bounds | With a starting point whose lower bound isn't 1, the Hessian is shifted or out of range (the port throws `Standard_OutOfRange`) | touch |
| `MathOpt::MultiStart` (MathOpt_GlobOpt.hxx) | Divides `MaxIterations` by `NbStarts` unguarded | NbStarts = 0 is undefined behaviour in C++; the port throws `DivideByZeroException` | touch |
| `MathOpt::PSO` (MathOpt_PSO.hxx) | Checks only the upper bounds' length, then indexes them with the lower bounds' indexes | Bounds vectors with different lower indexes are read wrongly or out of range | touch |
| `MathSys::Newton`, `MathSys::NewtonBounded` (MathSys_Newton.hxx) | On success return the Jacobian evaluated at the start of the last iteration, before the final step | `VectorResult.Jacobian` is not the Jacobian at the returned `Solution` | touch |
| `MathSys::LevenbergMarquardtBounded` (MathSys_LevenbergMarquardt.hxx) | The X-convergence test uses the unclamped step, not the step taken after clamping | At an active bound the X criterion never succeeds: the solver stops only on the F or gradient tests or at MaxIterations | touch |
| `MathSys::Solve2D` (MathSys_Newton2D.hxx) | Clamps with `(false, 0.0)`, ignoring `NewtonOptions::AllowSoftBounds` | Soft bounds have no effect in Solve2D, though Solve2DSymmetric, Solve3D and Solve4D honour them | touch |
| `Poly_Triangulation(Nodes, Triangles)` and `(Nodes, UVNodes, Triangles)`; `Poly_PolygonOnTriangulation(Nodes[, Parameters])` | Copy the tables with `NCollection_Array1::operator=`, which takes the source's bounds | A triangle array not starting at 1 gives a triangulation whose `Triangle(i)` is shifted (and out of range at 1) | touch |
| `Poly_ArrayOfNodes`, `Poly_ArrayOfUVNodes` (NCollection_AliasedArray allocation) | Allocate without initialising | Nodes of `Poly_Triangulation(n, m, ...)`, and nodes added by `ResizeNodes` or `AddUVNodes`, are garbage until set; the port's are zero | leave |
| `Poly_Connect` (Poly_Connect.hxx) | Documents the adjacent triangles of T as sharing n2-n3, n3-n1, n1-n2; the code gives them across n1-n2, n2-n3, n3-n1 | Documentation only; the port documents what the code does | leave |
| `Poly_CoherentNode::Poly_CoherentNode()` | Leaves the normal uninitialised | `HasNormal()` of a default node is random; the port's is false | leave |
| `Poly_CoherentNode::Clear` | Frees the list of triangles but keeps the pointer | The node isn't free afterwards and iterating its triangles reads freed memory; the port keeps the list | settled |
| `Poly_CoherentTriangulation::ComputeLinks` | Clears the links without clearing the triangles' pointers to them | Recomputing without `ClearLinks()` first leaves dangling link pointers, and sides that had a link get no new one | touch |
| `Poly_CoherentTriangulation(const occ::handle<Poly_Triangulation>&)` with a null triangulation | Leaves the deflection uninitialised | Random deflection; the port's is 0 | leave |
| `Poly_Polygon3D::Parameters`, `Poly_PolygonOnTriangulation::ChangeParameters` (deprecated) | Dereference a null handle when there are no parameters | Crash in OCCT; the port throws `Standard_NullObject` (Parameters) or a NullReferenceException (the deprecated method) | touch |
| `Poly_MergeNodesTool::MergedNodesMap::hashCode` | Hashes the bytes of the position as `char`, signed on MSVC and unsigned on ARM/GCC | The bucket chains, and so which of several close nodes a new one merges with, can differ between platforms; the port follows MSVC | leave |
| `Poly_MergeNodesTool::Result` with no node added | Calls `ResizeNodes(0)`, which throws (an aliased array can't be empty) | `Standard_RangeError` instead of an empty result, in OCCT and the port | touch |
| `NCollection_AliasedArray::ChangeLast` | Returns `Value<>()`, a const reference, as a non-const one | Doesn't compile if used; nothing in OCCT uses it. The port returns the element for writing | leave |
| `PLib::CoefficientsPoles` (gp_Pnt and gp_Pnt2d overloads, PLib.cxx) | Fills its pole work array from `Coefs`, not `Poles`; only the array's size comes from `Poles` | None when both have the same length, the only meaningful use (the values are overwritten). With fewer poles than coefficients C++ writes past the array; the port throws | touch |
| `BSplCLib::IncreaseDegree`, periodic curves (BSplCLib.cxx) | Not analysed: found by comparing with native OCCT through `Geom_BSplineCurve::IncreaseDegree`, whose poles the port reproduces exactly | For most periodic curves whose knot multiplicities are not all 1 (115 of 200 random ones), the raised curve is a different curve. Geom_BSplineCurve and Geom2d_BSplineCurve inherit it | fixed (decision 1484) |
| `BSplCLib::Eval` (flat poles, both overloads), extrapolation branch (BSplCLib.cxx) | Sizes the Taylor array for `LocalRequest + 1` rows, then evaluates a polynomial of degree `Degree` (always larger there) on it | Extrapolating beyond the ends with an extrapolation mode below the degree reads uninitialised stack memory. The port reads zeros instead, which gives the Taylor polynomial of order ExtrapMode, apparently the intent | settled |
| `BSplCLib::FunctionMultiply`, `FunctionReparameterise` (flat poles, BSplCLib_2.cxx) | Pass a single `int` as `ExtrapMode` to `Eval`, which reads two | `ExtrapMode[1]` is read past the variable, but only consulted for a parameter beyond the last knot, which the Schoenberg points never are. The port passes the degree for both | leave |
| `BSplCLib::InsertKnots` (flat poles, BSplCLib.cxx) | `np = NewKnots.Lower() + (index + 1) * Dimension` uses the knots' lower bound for an index into the poles | Wrong pole placement when the two lower bounds differ; harmless when both are 1, as in all OCCT callers | touch |
| `BSplCLib::IncreaseDegree` (flat poles, BSplCLib.cxx) | Reads `wpoles(NewPoles.Lower() + ...)`, but `wpoles` is indexed from 1 | Wrong when NewPoles.Lower() ≠ 1 | touch |
| `BSplCLib::MovePointAndTangent` (flat poles, BSplCLib_2.cxx) | Chooses the second bump function by comparing U with `FlatKnots(index)` and `FlatKnots(index + 1)`, where `index` indexes the Schoenberg points | The second bump function may peak on the farther Schoenberg point: the deformation is less local than intended, but valid | leave |
| `BSplCLib::MergeBSplineKnots` (BSplCLib_2.cxx) | Indexes `Mults1` and `Mults2` from 1, whatever their lower bounds | Wrong multiplicities for arrays not starting at 1 (OCCT's range checks are compiled out); the port throws | touch |
| `BSplCLib::DN(U, N, Poles, Weights, P, VN)` (Bezier, BSplCLib.hxx) | Declared, never defined | Calling it fails to link in C++; left out of the port | leave |
| `CSLib::Normal`, the overload with `theMaxOrder` and `theDerNUV` (CSLib.cxx) | In the sign-change loop, `aFirst = aIntervalIdx` is 0 for the first interval, so the next interval also takes the "first" branch, which resets the sign state | A sign change between the first two root intervals is never detected: `CSLib_Defined` with a normal can be reported where `CSLib_InfinityOfSolutions` is due | touch |
| `TopAbs::Print(TopAbs_State, Standard_OStream&)` (TopAbs.cxx) | Its name table is in the order ON, IN, OUT, UNKNOWN, but TopAbs_State is IN, OUT, ON, UNKNOWN | IN prints "ON", OUT prints "IN", ON prints "OUT" | touch |
| `Geom_AxisPlacement::SetAxis` on a `Geom_Axis2Placement` | Only assigns the axis; Geom_Axis2Placement doesn't redefine it, although the documentation says the X and Y directions are recomputed (and that a parallel X direction throws) | The X and Y directions are left unchanged, no longer perpendicular to the new main direction | touch |
| `Adaptor3d_HVertex::Adaptor3d_HVertex()` | Leaves the orientation uninitialised | `Orientation()` of a default vertex is random; the port's is FORWARD | leave |
| `BSplCLib::PrepareInsertKnots` with `Add` false (BSplCLib.cxx) | Counts equal entries of AddKnots once, with the first entry's multiplicity, where `BSplCLib::InsertKnots` processes each entry | The predicted pole and knot counts can be wrong. Example: a periodic degree-7 curve, its last knot added three times with multiplicities 2, 2 and 6: 30 poles predicted, unchanged, but InsertKnots raises both end multiplicities to 6 and the shape is lost. `Geom_BSplineCurve::InsertKnots` hides it by returning early when the pole count doesn't change. Found by the oracle tests | touch |
| `BSplSLib::CoefsD0/D1/D2` (BSplSLib.lxx) | Pass `Poles.RowLength() - 1` (the V degree) as UDegree and `ColLength() - 1` as VDegree, while `PolesCoefficients` returns the coefficients indexed (U, V) like the poles | With at least as many U poles as V poles they evaluate the Bezier surface at (V, U), the U and V derivatives exchanged; with fewer U poles the result is meaningless. No OCCT code calls them | leave |
| `BSplSLib::CacheD2`, rational (BSplSLib.cxx) | Zeroes the work array above row `MinIndMax` for the poles' dimension, then reuses it for the weights with the smaller dimension `min_degree + 1` | When both degrees are below 2, the weights' third row holds pole values, taken as the second derivative of the weights: the second derivatives (and `CoefsD2`'s) of a rational bilinear surface are wrong. BSplSLib_Cache, which OCCT's own evaluators use, is not affected | fixed (decision 1497) |
| `BSplSLib::CacheD0/D1/D2` (BSplSLib.cxx) | Take the cache's start as `PolesArray(LowerCol(), LowerRow())`, row and column swapped | Harmless for every cache OCCT builds (both lower bounds 1); wrong or out of range otherwise | leave |
| `BSplSLib::IsRational` (BSplSLib.cxx) | Takes the default tolerance from `Weights(I1, I2)`, a row bound used as a column index, instead of `Weights(I1, J1)`; compares the weights along U only | A tolerance from the wrong weight, out of the row's range when I2 exceeds the column count (OCCT's release build reads that storage element unchecked; the port reads the same element) | touch |
| `BSplSLib::FunctionMultiply` (BSplSLib.cxx) | Calls `HomogeneousD0` with URat = VRat = true whatever `Weights` is | A null `Weights` is dereferenced (crash in OCCT, `NullReferenceException` in the port) | touch |
| `BSplSLib::MovePoint` (BSplSLib.cxx) | When [UIndex1, UIndex2] (or the V window) holds none of the poles acting at (U, V), `ukk1` (`vkk1`) stays 0 and `UBSplineBasis(1, ukk1 - UFirstNonZeroBsplineIndex + 2)` is read out of the matrix's range | OCCT's release build reads past the basis matrix and goes on with a meaningless pole choice; the port throws `Standard_OutOfRange`. Seen through `Geom_BSplineSurface::MovePoint` with a window away from the point | touch |
| `Convert_ConicToBSplineCurve::BuildCosAndSin` (Convert_ConicToBSplineCurve.cxx) | For a `Convert_ParameterisationType` outside the enumeration, calls through a null function pointer | Crash in OCCT; `NullReferenceException` in the port. Only reachable with an invalid enum value | leave |
| `GeomEval_TBezierSurface::IsUClosed`, `IsVClosed` | Compare the first and last rows (columns) of poles. In the trigonometric basis these are the constant and the highest cosine terms; the boundary iso-curves are S(0, v) = Σ (P0j + P2j + P4j + ...) Bv_j and S(π/α, v) = Σ (P0j − P2j + P4j − ...) Bv_j | The result says nothing about closure: a surface can be reported closed when it isn't (test `IsClosed_ComparesPoleRows_OcctIssue`), and the reverse | touch |
| `GeomEval_AHTBezierCurve`, `Geom2dEval_AHTBezierCurve`, `GeomEval_AHTBezierSurface`, non-rational constructors | Allocate a one-element weights array (`myWeights(1, 1)`, `(1, 1, 1, 1)`) and leave it uninitialised; `Weights()` returns it | A non-rational curve or surface reports one random weight; the port's is 0. A non-rational TBezier returns an empty array instead | leave |
| `AdvApp2Var_MathBase::mdsptpt_` (AdvApp2Var_MathBase.cxx) | The work array `differ` is a null pointer (the Fortran had `DIFFER(100)`); it is allocated only when NDIMEN > 100 | For NDIMEN <= 100 C++ dereferences null. Nothing in OCCT calls it. The port throws `IndexOutOfRangeException` | leave |
| `AdvApp2Var_MathBase::mmresol_` (AdvApp2Var_MathBase.cxx) | Its work arrays (`v1` to `v4`, `hchole`, `matsym`, `mcho`) are null pointers that `macrar8_` allocates only above 100 elements | Any dimension <= 100 dereferences null. Nothing in OCCT calls it. The port throws | leave |
| `AdvApp2Var_MathBase::mmfmtb1_` (AdvApp2Var_MathBase.cxx) | Fills only the first JSIZE1 elements of each line of its work zone, then copies MAXSZ2 per line into TABLE2 | When MAXSZ2 > JSIZE1, the unused part of TABLE2 gets uninitialised heap memory. The port, whose zones are zeroed, writes zeros | leave |
| `mmcmher_`, the Hermite cache of `AdvApp2Var_MathBase::mmherm0_`/`mmherm1_` (AdvApp2Var_MathBase.cxx) | A static struct shared by every thread; `mmherm0_` zeroes then refills it while `mmherm1_` reads it on the strength of three keys written earlier | Concurrent approximations race on it: a thread can read a half-filled table (at the first use, or when threads use different intervals). The port keeps one cache per thread (decision 113) | settled |
| `MyDirectPolynomialRoots` (IntAna2d_Outils.cxx) | The extra candidate roots named `MATH_A210` are those of `A3 x² + A2 x + A1`, not `A2 x² + A1 x + A0` as the name and its siblings (A4321, A3210) suggest | When the quartic's roots look unreliable, the wrong quadratic's roots are added as candidates; they are usually sorted out by their residuals | leave |
| `IntAna2d_AnaIntersection::Perform(gp_Circ2d, gp_Circ2d)` (IntAna2d_AnaIntersection_2.cxx) | Measures both parameters counterclockwise from each circle's X axis (`gp_Vec2d::Angle`), whatever the circle's sense; the external-tangency parameter on the second circle is also not brought into [0, 2PI[ | `ParamOnFirst`/`ParamOnSecond` are wrong on an indirect circle (reproduced by a unit test; the oracle shows native OCCT gives the same) | fixed (decision 1464) |
| `IntAna2d_AnaIntersection::Perform(gp_Hypr2d, IntAna2d_Conic)` and `(gp_Parab2d, ...)` | `MyDirectPolynomialRoots` reports "not done" when the quartic has no real root; the hyperbola case then returns `IsDone() == false`, while the parabola case, lacking a `return`, ends with `done = true` | A hyperbola that misses the conic gives "not done" rather than an empty result; a parabola gives an empty result | touch |
| `Extrema_ExtPElC2d::Perform` for ellipses, hyperbolas and parabolas (Extrema_ExtPElC2d.cxx) | Sets `myIsMin = (NoSol == 0)` in a loop where NoSol starts at 1; for circles it sets true for both extrema | `IsMin` is always false for these conics, and true for the far point of a circle. The 3D Extrema_ExtPElC computes IsMin properly | touch |
| `Extrema_ExtElSS::Points` (Extrema_ExtElSS.cxx) | No Perform fills `myPOnS1`/`myPOnS2` | `Points` dereferences null handles: a crash in OCCT for two parallel planes (the only case with a result); a NullReferenceException in the port. The other surface pairs throw Standard_NotImplemented | touch |
| `Extrema_ExtElC(gp_Lin, gp_Lin, AngTol)`, `(gp_Lin, gp_Circ, Tol)`; `Extrema_ExtElC2d(gp_Lin2d, gp_Lin2d, AngTol)`, `(gp_Lin2d, gp_Circ2d, Tol)`; `Extrema_ExtPElC::Perform(gp_Parab)`; `Extrema_ExtPElS::Perform(gp_Pln)` | Ignore their tolerance argument, though the line-line ones document it as the parallelism test | Precision::Angular() (or no tolerance) is used whatever the caller passes | touch |
| `gce_MakeCirc(P1, P2, P3)` (gce_MakeCirc.cxx) | Returns without setting `TheError` when the three points coincide, and when the computed center is within gp::Resolution() of P1 | The status is uninitialised; the port's is gce_Done (decision 118) | settled |
| `gce_MakeCirc2d(P1, P2, P3)` (gce_MakeCirc2d.cxx) | Doesn't set `TheError` when the bisectors don't intersect (distinct collinear points) | The status is uninitialised, and `Value()` may return a default circle as a success; the port reports gce_IntersectionError (decision 118) | settled |
| `gce_MakeCirc(Center, gp_Pln, Radius)` (gce_MakeCirc.cxx) | Calls `Value()` on its inner construction without checking it | With a negative radius, StdFail_NotDone in debug builds; the release build copies the default circle and reports gce_NegativeRadius. The port does the latter (decision 119) | leave |
| `gce_MakeLin2d(const gp_Lin2d&, double Dist)` (gce_MakeLin2d.hxx) | Documented as placing the line on the right of `Lin` for a positive distance; the code offsets along the left normal (-Dy, Dx) | Documentation only; the port documents what the code does | leave |
| `Geom_BSplineCurve::EvalD0` to `EvalD3` (Geom_BSplineCurve_1.cxx) | Normalize a copy of U with `PeriodicNormalization`, then pass U itself to `LocateParameter`, which overwrites the copy | None: `LocateParameter` handles periodicity itself. Dead code | leave |
| `Geom_BSplineCurve::LocalDN` | Its error message says "LocalD3" | Cosmetic | leave |
| `Geom_BezierCurve::InsertPoleAfter`, `InsertPoleBefore` | Refuse a curve of `MaxDegree()` = 25 poles (`nbpoles >= MaxDegree()`), though the constructors and `Increase` accept MaxDegree() + 1 = 26 poles | A pole can't be inserted to reach degree 25 | touch |
| `Geom_BezierCurve(Poles, Weights)` and its `Rational` helper | Index the weights from 1 whatever the array's lower bound | Weights not indexed from 1 are read shifted or out of range (range checks compiled out in OCCT; the port throws) | touch |
| `Geom2d_OffsetCurveUtils::CalculateD3` and `Geom_OffsetCurveUtils::CalculateD3` (Geom2d_OffsetCurveUtils.pxx, Geom_OffsetCurveUtils.pxx: the same code, both branches) | The second derivative it returns subtracts the `Ndir` term that `CalculateD2` adds, and the third derivative's `DNdir` and `Ndir` coefficients are not the derivative of the second (`-3(D2r/R³ + Dr²/R⁵)` for `-3(D2r/R³ - 3Dr²/R⁵)`; `6Dr²/R⁵ + 6Dr·D2r/R⁵ - 15Dr³/R⁷ - D3r` for `9Dr·D2r/R⁵ - 15Dr³/R⁷ - D3r/R³`) | `Geom2d_OffsetCurve::EvalD3` and `Geom_OffsetCurve::EvalD3` (so `D3`, `DN(U, 3)`), `Adaptor2d_OffsetCurve::D3` and (through `EvaluateD3`) GeomAdaptor's offset curves return wrong D2 and D3 wherever the basis curve's speed varies; right on circles and lines. Offset 0.5 of an ellipse (3, 1) at u = 0.7: EvalD3's D2 is (-2.597, -3.131) where EvalD2's is (-1.794, -1.102), and its D3 is (8.29, 11.12) where finite differences give (0.627, 1.171); the same values in 3D for the ellipse in the XY plane offset along Z | fixed (decision 1487) |
| `Geom2d_BSplineCurve` modifiers (Geom2d_BSplineCurve.cxx, _1.cxx) | None of them clears an attached evaluation representation, where every `Geom_BSplineCurve` modifier that changes the curve calls `ClearEvalRepresentation` (Geom_BSplineCurve.cxx, _1.cxx) | After `Transform`, `SetPole`, `InsertKnot`, ... a 2D curve still evaluates through the old representation, although its poles and knots changed; a 3D one drops it. The ports keep each family's behaviour (tests pin both) | touch |
| `Geom2d_BezierCurve::SetPole`, `SetWeight`, `Reverse`, `Segment` and the same in `Geom_BezierCurve` | Don't clear the evaluation representation; only `init` (constructors, `Increase`, pole insertion and removal) and `Transform` do, in both families | The modified curve still evaluates through the old representation (tests pin it in both families) | touch |
| `Geom2d_BSplineCurve::IncreaseDegree` (non-periodic) | Sizes the new poles as `NbPoles + Step * (LastUKnotIndex - FirstUKnotIndex)`, counting only the knots of the parameter range, while `BSplCLib::IncreaseDegree` raises the end multiplicities too | On a curve not clamped at its ends the result is inconsistent: degree 3, multiplicities (4, 2, 2, 2, 2) and 8 poles give degree 4, multiplicities (5, 3, 3, 3, 5) and 11 poles where 14 are needed (native OCCT gives the same curve). Unclamped at both ends, `BSplCLib::IncreaseDegree` writes past the poles (Standard_OutOfRange in the port and in native OCCT, whose array checks are on) | touch |
| `Geom2d_TrimmedCurve::SetTrim`; `Geom2d_BSplineCurve::LocalDN` | Error messages name `Geom_TrimmedCurve` and `LocalD3` | Cosmetic | leave |
| `Geom2d_BezierCurve(Poles, Weights)` and its `Rational` helper (Geom2d_BezierCurve.cxx) | Index the weights from 1, whatever the array's lower bound | Weights indexed from another bound are read out of range (garbage in OCCT's release build; Standard_OutOfRange in the port) | touch |
| `AppParCurves::SecondDerivativeBernstein` (AppParCurves.cxx) | Allocates `B(1, deg - 1)` and writes `B(1)` whatever the degree | For degree 1 the write is past an empty vector (unchecked in OCCT's release build); the port allocates one element. The result is right either way | leave |
| `AppParCurves_MultiCurve::Pole`, `Pole2d` (AppParCurves_MultiCurve.cxx) | Range check `(CuIndex <= 0) && Nieme <= 0`, `&&` where `\|\|` is meant | The check almost never fires; the MultiPoint's own check catches a bad curve index | leave |
| `AppParCurves_LeastSquare::Init` (AppParCurves_LeastSquare.gxx) | The test that fixes the last pole ends with `FirstConstraint == AppParCurves_CurvaturePoint` where `LastConstraint` is meant | A curvature constraint at the first point also copies the last point into the last pole; harmless when the last point is unconstrained, since that pole is then solved for | leave |
| `AppParCurves_ResolConstraint` constructor (AppParCurves_ResolConstraint.gxx) | The second member of a 2d curve reads `tabP2d(i - nb3d)`, the passing point index `i` where the curve `k - nb3d` is meant | Out of bounds (garbage in OCCT's release build, Standard_OutOfRange in the port) unless `i - nb3d` happens to be a 2d curve index: interior constraints on 2d curves are broken | touch |
| `AppParCurves_ResolConstraint` constructor, tangency relations (AppParCurves_ResolConstraint.gxx) | For a 3d curve whose largest tangent component is Y, `T1 = V.Y()` is missing | The relation between the curves' tangents uses the previous T1; only with two or more curves and an interior tangency | touch |
| `AppParCurves_ResolConstraint::ConstraintDerivative` (AppParCurves_ResolConstraint.gxx) | Indexes the 2d tangents by `k` where `k - nb3d` is meant, and writes the tangent relations' derivatives into `Cont` (the constraint matrix) instead of `DeCont`; T1 and T2 are uninitialised | Out of bounds for 2d curves after 3d ones (the port throws); with two or more curves and an interior tangency, the constraint matrix is overwritten by derivatives and the derivative matrix lacks those rows | touch |
| `AppParCurves_Function` and `AppParCurves_BSpFunction` constructors (AppParCurves_Function.gxx, AppParCurves_BSpFunction.gxx) | With interior constraints, read the 2d points as `TabP2d(j)`, the curve index, where `j - nb3d` is meant | Out of bounds for a line with 3d and 2d points (garbage in OCCT, Standard_OutOfRange in the port): constrained approximation of such lines doesn't work | touch |
| `AppParCurves_Function::Perform`, constrained gradient (AppParCurves_Function.gxx) | `DPTCO(j) -= RESTM(j, k) * DPTCO1(Pi, j + Inc)` reads column `j` where `k` is meant; the correction term is `Scal = ...` where `+=` is meant, and its z part uses `Erruzax` | The gradient of F with interior constraints is wrong (only the last pole counts, x stands in for z); the BFGS minimisation still runs, stopping on the error tolerances | touch |
| `AdvApprox_SimpleApprox::Perform` (AdvApprox_SimpleApprox.cxx) | With an odd number of Gauss points (15 or 25), the loop adding the t = 0 contribution doesn't restart `Sum` for each coefficient | Each even Jacobi coefficient is a running sum carried over from the previous one: an error of about 0.9 on sin(3u) where 20 or 30 points give 1e-15. AdvApprox_ApproxAFunction always uses 30 or more points; only direct callers of SimpleApprox, or the static `Approximation` with code ≤ -3, are affected | touch |
| `FEmTool_Curve::GetElement` (FEmTool_Curve.cxx) | Rescales the derivative columns `j1 + 1 .. myDimension`, with `j1 = Coeffs.LowerCol() - 1`, where `j1 + myDimension` is meant | With LowerCol > 1 the last columns aren't rescaled; with LowerCol < 1 OCCT writes out of bounds (the port throws). AppDef passes 1-based arrays | touch |
| `FEmTool_Assembly::AddConstraint` (FEmTool_Assembly.cxx) | When the line already has segments and the new index range sticks out past every segment at both ends, `Coeff` stays null and is dereferenced | Crash in OCCT; NullReferenceException in the port | touch |
| `GProp_CelGProps`, `GProp_SelGProps`, `GProp_VelGProps` (all `Perform` overloads) | Build the inertia matrix as P·V·Λ (each eigenvector scaled by its value) instead of P·V·Λ·Vᵀ·Pᵀ; `V.Multiply(Passage, V)` also reads V while writing it | Wrong, non-symmetric inertia matrices (an axis-aligned full circle gets (1,3) = 2πR³, (2,1) = (3,2) = πR³ and a zero diagonal). No OCCT code uses these classes | fixed (decision 1471) |
| `GProp_CelGProps`, `GProp_SelGProps`, `GProp_VelGProps` | Store the centre of mass `g` as an absolute point, while GProp_GProps reads it relative to `loc` | `CentreOfMass` is shifted by the location point when it isn't the origin: a unit circle at the origin, with location (1, 2, 3), reports its centre at (1, 2, 3) | fixed (decision 1471) |
| `GProp_CelGProps::Perform(gp_Lin)` (GProp_CelGProps.cxx) | Computes Izz with Ixx's coefficients, where `alfa2 = Vx·X0 + Vy·Y0` and `alfa3 = X0² + Y0²` are meant | For a rod along X from (1, 2, 3) of length 6, the central Izz is 30 where it should be 18 | fixed (decision 1471) |
| `GProp_CelGProps::Perform(gp_Circ)` (GProp_CelGProps.cxx) | Uses the centre's Z0 for the z of the centre of mass | Ignores the tilt of the circle's plane (covered by the inertia and centre issues above) | fixed (decision 1471) |
| `GProp_SelGProps::Perform(gp_Cone)` (GProp_SelGProps.cxx) | The area has an extra cos(SemiAngle) factor, and ISn2 is a copy of ICn2 | Wrong area, and the sin·cos inertia terms have the wrong sign | fixed (decision 1471) |
| `GProp_VelGProps::Perform(gp_Cone)` (GProp_VelGProps.cxx) | Computes π L² cos t sin t (2R + L sin t) as the volume, not the frustum's; ISn2 is again a copy of ICn2 | Wrong volume and inertia | fixed (decision 1471) |
| `GProp_VelGProps::Perform(gp_Cylinder)` (GProp_VelGProps.cxx) | The axial moment is R²·dim where R²·dim/2 is meant | Axial inertia twice too large | fixed (decision 1471) |
| `GProp_VelGProps::Perform(gp_Torus)` (GProp_VelGProps.cxx) | Reads cos(Alpha2) and sin(Alpha2) as Cnt2 and Snt2 where Teta2 is meant | With Teta in [0, π] and Alpha full, the centre stays at y = 0 (SelGProps gives 2R/π) | fixed (decision 1471) |
| `IntAna_Curve::InternalUVValue` (IntAna_Curve.cxx) | The derivative `aDA` has `aCos2t * (Z2CosSin * Z2CosSin)` where `aDB` and `aDC` have `(Z?CosSin + Z?CosSin)` | Only the discriminant tolerance `aTolD` is affected | leave |
| `IntAna_IntQuadQuad::InternalSetNextAndPrevious` (IntAna_IntQuadQuad.cxx) | `for (c2 = 0; (c2 < NbCurves) && (c2 != c1); c2++)` stops at c2 == c1 instead of skipping it | Each curve is compared only with the curves before it. Every pair is still tested once and the updates are symmetric, so the links are probably the same | leave |
| `TrigonometricRoots` (IntAna_IntQuadQuad.cxx) | `NbRoots` is left uninitialised when math_TrigonometricFunctionRoots fails or has infinite roots | `IsARoot` reads it; harmless while callers check `IsDone` first. 0 in the port | leave |
| `IntAna_IntQuadQuad::NbPnt`, `Point`, `Parameters` | `Thepoints`/`Nbpoints` are never filled (the code that would is commented out), and `Parameters` only prints "not yet implemented" | `NbPnt` is always 0; `Parameters` returns nothing | leave |
| `AxeOperator::Distance` (IntAna_QuadQuadGeo.cxx) | Leaves its outputs unset when the axes are parallel | Its one caller (skew cylinder/cylinder) never passes parallel axes. 0 in the port | leave |
| `Extrema_ExtElCS::Points` (Extrema_ExtElCS.cxx) | A parallel result (`IsParallel`) fills only the square distance | `Points` then reads null handles (a crash) or the arrays of an earlier Perform on the same object; the port throws NullReferenceException or returns those stale points likewise | touch |
| `ProjLib_Cone::Project(const gp_Circ&)` (ProjLib_Cone.cxx) | Sets the type to `GeomAbs_Line` before checking that the circle is normal to the axis | For a tilted circle `IsDone()` is false but `Line()` returns the default line instead of throwing, and `ProjLib::Project(gp_Cone, gp_Circ)` returns it silently | touch |
| `ProjLib_Projector::ProjLib_Projector()` | Documented as setting the type to OtherCurve; sets BSplineCurve | Documentation only | leave |

## Phase 2: unit 1C, Geom surfaces

| Where | What | Effect | Triage |
|---|---|---|---|
| `Geom_ConicalSurface::Coefficients` (Geom_ConicalSurface.cxx) | Computes C1-C3 as `T1j * T14 + T2j * T24 + radius * T3j` and D with `+ 2.0 * radius * T34`, where gp_Cone::Coefficients has `- T3j * (radius + T34)` and `- 2.0 * radius * T34` (T3j scaled by tan(semiAngle)) | The implicit equation is wrong unless the cone's origin is the global origin and its reference radius is 0: the surface's points don't satisfy it. The port computes what OCCT computes (oracle-tested); a unit test shows the error | fixed (decision 1494) |
| `Geom_RectangularTrimmedSurface(S, Param1, Param2, UTrim, Sense)` (Geom_RectangularTrimmedSurface.cxx) | Calls `SetTrim(Param1, Param2, UTrim, Sense)`, which reads `isutrimmed`/`isvtrimmed` (and the other direction's trims), before anything initialises them | Undefined in C++; OCCT's allocator clears new objects by default (MMGT_CLEAR), so they read false, as the port's defaults do | leave |
| `Geom_BSplineSurface::MovePoint` on a periodic surface (Geom_BSplineSurface_1.cxx) | Passes the periodic flat knots and the stored poles to `BSplSLib::MovePoint`, which indexes poles and weights by the B-spline basis index, beyond NbPoles near the end of a periodic direction | OCCT's release build reads (and moves) storage past the pole and weight arrays; the port throws `Standard_OutOfRange`. Found by the oracle tests on V-periodic surfaces | touch |
| `Geom_BSplineSurface::CheckAndSegment` over the whole range of a periodic direction (Geom_BSplineSurface.cxx, `segment`) | The direction is not segmented, but the knots are still rebuilt from `index1U` to `index2U`, and `NewU1 + deltaU` (one period on) locates on the first knot again, so `index2U` becomes `index1U + 1` | The direction keeps two knots and stays periodic with all its poles: knots and poles no longer match. Evaluation then throws (`PeriodicNormalization`) or reads out of range, and the iso-curves (closure tests) fail to build. Example: a U-periodic degree-1 surface with 6 knots of multiplicity 1, `CheckAndSegment` on its whole parametric rectangle | touch |
| `Geom_BSplineSurface::RemoveUKnot`, `RemoveVKnot` on a periodic direction (Geom_BSplineSurface_1.cxx) | Accept `Index` equal to FirstUKnotIndex (1) or LastUKnotIndex, the knots that close the period | Removing one leaves knots and multiplicities inconsistent with periodicity; `BSplCLib::KnotSequence` then reads out of range (OCCT's release build reads past the array; the port throws `IndexOutOfRangeException`). Example: V-periodic degree 2, knots multiplicities (2, 1, 2), `RemoveVKnot(3, 0, 100)` | touch |
| `Geom_BezierSurface` edits (Geom_BezierSurface.cxx) | Only `Transform`, `UReverse` and `VReverse` clear the evaluation representation; `SetPole`, `SetWeight`, `Increase`, `Segment`, `ExchangeUV`, the pole row and column edits keep it (Geom_BSplineSurface clears it on every edit) | A Bezier surface with an evaluation representation goes on evaluating the old shape after those edits | touch |
| `Geom_BezierSurface::Increase`, V direction (Geom_BezierSurface.cxx) | Passes `UKnots()` as the V knots | Harmless: a Bezier surface's knots are (0, 1) in both directions | leave |
| `Geom_BezierSurface::VIso` (Geom_BezierSurface.cxx) | Passes `UKnots()` (with `VMultiplicities()`) to `BSplSLib::Iso` for the V direction | Harmless, as for `Increase`: the knots are (0, 1) in both directions. The port passes the same arrays | leave |
| `Adaptor3d_CurveOnSurface::EvalD3`, general path (Adaptor3d_CurveOnSurface.cxx, file-static `SetLinearForm`) | The third derivative of S(u(t), v(t)) weights `D1U` by u' (`DW.X()`) where the chain rule has u''' (`D3W.X()`), and gives `D3UVV` the coefficient 5u'v'² instead of 3u'v'² (the second `V32.SetLinearForm` repeats the `DW.X()*DW.Y()*DW.Y()` term, and V32 is then doubled) | `D3` and `DN(U, 3)` of a curve on a surface are wrong outside the line and circle special cases, whenever u' ≠ u''' or the surface has a non-zero D3UVV. A 2D line of slope (1, 2)/√5 on a cylinder of radius 2: D3 is off by u'·Su, about 0.9, where a finite difference of D2 agrees with the chain rule (`Adaptor3d_CurveOnSurfaceTests.EvalD3_OcctFormula_DiffersFromDerivative`) | fixed (decision 1488) |
| `Adaptor3d_CurveOnSurface.cxx`, file-static `Hunt` | When the coordinate matches no knot, the loop leaves `i = Arr.Upper() + 1` and `Arr(i)` reads past the array (release build) before throwing `Standard_NotImplemented` | Not reachable from the callers, which only pass knots; the port would throw IndexOutOfRangeException there | leave |
| `Adaptor3d_TopolTool::Classify` and `IsThePointOn`, infinite lower bound (Adaptor3d_TopolTool.cxx) | U is OUT when `U >= Usup + Tol`, V when `V > Vsup + Tol` | A point exactly at the upper bound + Tol is OUT in U and ON in V. Harmless in practice | leave |
| `Adaptor3d_TopolTool::BSplSamplePnts` | With fewer than 4 candidate parameters in a direction (e.g. degree 1, one span, minimum below 4), `nn = nbsu / 4` is 0 and no parameter is flagged, but the count is still set to 4 | The sample parameters end in zeros (e.g. [0, 1, 0, 0]) and the samples include parameter 0 where no knot is | touch |
| `Adaptor3d_TopolTool(const handle<Adaptor3d_Surface>&)` | Leaves `myNbSamplesV`, `nbVtx` and `idVtx` uninitialised | Harmless: nothing reads them before they are set (C# zeroes them) | leave |

## Phase 2: unit 2E, constructions and sampling

| Where | What | Effect | Triage |
|---|---|---|---|
| `GC_MakeSegment(const gp_Pnt&, const gp_Pnt&)` (GC_MakeSegment.cxx) | Converts `GC_MakeLine(P1, P2)` to a handle without checking its status | With coincident points OCCT's release build passes a null line to Geom_TrimmedCurve and crashes; the port throws StdFail_NotDone | touch |
| `GC_MakeArcOfCircle2d(P1, V, P2)` (GC_MakeArcOfCircle2d.cxx) | Converts `gce_MakeLin2d(P1, P2)` without checking its status | With P1 = P2 the release build goes on with the default line, finds the center at P1 with radius 0 and computes NaN parameters; the port throws Standard_ConstructionError from `ElCLib::Parameter` | touch |
| `GCPnts_QuasiUniformAbscissa::initialize` (GCPnts_QuasiUniformAbscissa.cxx) | Reads `NbPoints()` and `Parameter()` of its GCPnts_UniformAbscissa without checking `IsDone()` | The debug build throws StdFail_NotDone on curves other than Bezier and B-spline whose abscissa computation fails (a zero-length curve); the release build copies the failed result. The port copies it, as the release build does | touch |
| `GCPnts_TangentialDeflection::initialize` (GCPnts_TangentialDeflection.cxx) | `Standard_ConstructionError_Raise_if(...)` is followed by `myParameters.Clear();` without a semicolon in between | Harmless: both expansions of the macro still clear the parameters | leave |
| `GCPnts_UniformDeflection::Value`, `GCPnts_QuasiUniformDeflection::Value` | Their StdFail_NotDone messages name `GCPnts_UniformAbscissa::Parameter()` and `GCPnts_QuasiUniformAbscissa::Parameter()` | Cosmetic | leave |

## Phase 2: unit 2D, AdvApp2Var

| Where | What | Effect | Triage |
|---|---|---|---|
| `AdvApp2Var_ApproxF2var::mma2ce1_` (AdvApp2Var_ApproxF2var.cxx) | Calls `mma2jmx_` for U and for V with the same output `&wrkar_off[ipt5]`, where the U maxima belong at `ipt4` | The maxima of the U Jacobi polynomials are never computed and stay 0 (the work zone is calloc'ed), so every estimate of the error of dropping coefficients (`mma2er1_`, `mma2er2_`, which multiply by them) is 0. Degree reduction is then unchecked and `MaxError` underestimates: sin(u)·cos(v) on [0, 2π]², C0, tolerance 1e-4, gives IsDone with MaxError 1.03e-5 for a surface of degree 1 in V that is more than 1 away from the function (native OCCT and the port alike; `SinCos_C0_ReproducesOcctsUnderestimatedError`). GeomConvert_ApproxSurface inherits it | touch (decision 1482) |
| `AdvApp2Var_ApproxF2var::mma2ce1_`, the same calls | The U call writes `ndjacu + 1 - 2(iordru + 1)` values at `ipt5`, where `ndjacv + 1 - 2(iordrv + 1)` plus `4·ndimen` remain | When the U Jacobi degree is much larger than the V one (C0 in U with MaxDegInU 14, C2 in V with MaxDegInV 6, precision code 1), OCCT writes past its work zone and corrupts the heap; native OCCT crashed on every such case. The port keeps the stray values in the zone (decision 198) | settled |
| `AdvApp2Var_Iso::MakeApprox`, `AdvApp2Var_Patch` (Discretise, AddConstraints, MakeApprox) | Read and write three coordinates per point, and pass 3 as the subspace dimension, whatever the dimension (OCCT documents "works only in 3D") | With 1D or 2D subspaces OCCT reads and writes outside its arrays; the port throws (decision 196) | touch |
| `Approx_BSplComputeLine::FindRealConstraints` (Approx_BSplComputeLine.gxx) | For a CurvaturePoint end constraint, tests `LineTool::Tangency` a second time where `Curvature` is meant | A line with tangents but no curvatures gets CurvaturePoint ends (more poles counted, curvature constraints asked of the least squares) | touch |
| `Approx_ComputeLine` CheckMultiCurve, 2d case (Approx_ComputeLine.gxx) | Normalises the first pole vector by `sqrt(aSqNormToler)` (Epsilon(1)), not by its norm | The first scalar product is scaled by about 1e16, so any turn of more than 90 degrees between the first two pole segments counts as a loop | touch |
| `Approx_ComputeLine` CheckMultiCurve (Approx_ComputeLine.gxx) | `int indbads[4]` is written at `indbads[indcur]` for every curve | Past the array for a line with more than three curves (one 3d and three 2d, or four 2d): undefined in OCCT, IndexOutOfRangeException in the port | touch |
| `Approx_ComputeLine` constructors (Approx_ComputeLine.gxx) | `tolreached` is never initialised | `IsToleranceReached()` before any approximation sets it returns garbage; false in the port | leave |
| `FEmTool_Curve::D2` (FEmTool_Curve.cxx) | For an element of degree 1 (which `ReduceDegree` allows at C0), evaluates a polynomial of degree -1: the Dimension coefficients just before the element's second-derivative block | The previous element's highest coefficients, or memory before the array for the first element (uninitialised in either case unless computed): native C0 variational approximations (GeomAPI_PointsToBSpline with weights) change from run to run. The port reads the same coefficients and zeros before the array (decision 206) | touch |
| `AppCont_LeastSquare` constructor (AppCont_LeastSquare.cxx) | `myDone` is set only after solving a constrained system | `IsDone()` is false when both ends are free (the poles are computed) and when the constraints fix every pole; Approx_ComputeCLine copies it, so an unconstrained continuous approximation reports "not done" | fixed (decision 1470) |
| `AppCont_LeastSquare` constructor, period jump at the last point (AppCont_LeastSquare.cxx) | The loop runs `aDimIdx <= 2` where the first point's runs to `aMaxDim` | Periodic coordinates after the second aren't unwrapped at the last pole for a PassPoint or TangencyPoint last constraint | touch |
| `AppDef_Variational::EstTangent`, `EstSecnd`, `ACR` (AppDef_Variational.cxx) | Test `myTypConstraints(2*IdCnt-1) < ipnt` before `IdCnt <= NbConstr` | Reads index 2*NbConstr+1, past the constraints, for every point after the last constraint (ACR reads constraint NbCntr+1). The port tests the count first (decision 208) | settled |
| `AppDef_Variational::Project`, `TheMotor` (AppDef_Variational.cxx) | `NumPoints` is set only where a distance is positive, then shifted by `myFirstPoint - 1` on every call; TheMotor never initialises it | An uninitialised (or doubly shifted) index of the farthest point when no distance is positive; the port starts at 0 | settled |
| `AppDef_Variational::EstTangent`, last point (AppDef_Variational.cxx) | Addresses `myTabPoints` as `(myLastPoint - 3) * myDimension + 1` | Wrong point unless `myFirstPoint == 1` | touch |
| `AppDef_Variational::InitCriterionEstimations` (AppDef_Variational.cxx) | Calls `EstTangent(myFirstPoint + 2)` for a line of two points | Reads past the points; the port throws Standard_OutOfRange | touch |
| `AppDef_Variational::Init` (AppDef_Variational.cxx) | Never resets `myIsOverConstr` to false | After an over-constrained setup, `SetConstraints` with feasible constraints returns false and `IsOverConstrained()` stays true, although the object approximates | touch |
| `AppDef_LinearCriteria::Gradient` (AppDef_LinearCriteria.cxx) | Takes `&myCache->Value(1)` before its loop, even for an element without points | Dereferences a null cache; the port reads the cache inside the loop only | settled |

## Phase 2: unit 2A, offset surfaces and GeomAdaptor

| Where | What | Effect | Triage |
|---|---|---|---|
| `Geom_OffsetSurface::TransformParameters` (Geom_OffsetSurface.cxx) | Transforms the parameters with the basis surface, then again with the equivalent surface (the evaluation representation) when there is one, although the offset surface has the basis surface's parametrization | Parameters are transformed twice: for an offset of a plane and a scale of 2, U and V are multiplied by 4; it disagrees with `ParametricTransformation`, which uses the basis only. Confirmed against native OCCT (oracle); a unit test pins it | fixed (decision 1489) |
| `Geom_OffsetSurface::UReverse`, `VReverse`, `Transform` (Geom_OffsetSurface.cxx) | Clear the evaluation representation (the equivalent plane, cylinder, ... built from `Surface()`) and don't rebuild it, where `SetOffsetValue` and `SetBasisSurface` do | After a reversal or a transformation, the surface is evaluated through the general offset formula instead of the equivalent surface (equal to rounding), `UIso`/`VIso` return AdvApprox approximations instead of exact lines and circles, and `TransformParameters` stops transforming twice | touch |
| `GeomAdaptor_SurfaceOfLinearExtrusion::VTrim`, `UTrim` (GeomAdaptor_SurfaceOfLinearExtrusion.cxx) | `VTrim` trims the basis curve, which carries U; `UTrim` ignores its parameters and returns the untrimmed extrusion | Trimming in V restricts U instead (V stays unbounded), and trimming in U does nothing. A unit test pins it | fixed (decision 1496) |

## Phase 2: unit 3B, curve extrema

| Where | What | Effect | Triage |
|---|---|---|---|
| `Extrema_GGenExtCC::Perform` (Extrema_GGenExtCC.hxx; Extrema_ECC, Extrema_ECC2d) | Never clears `myPoints1`/`myPoints2` | A second `Perform` on the same object appends its solutions to the first one's (`Extrema_CurvePortTests.ECC_PerformTwice_AccumulatesSolutions_AsInOcct`). Extrema_ExtCC keeps one Extrema_ECC and calls `Perform` on it at each of its own `Perform`s, so an Extrema_ExtCC reused (Initialize + Perform) on curves taking the ECC path reports the earlier curves' solutions too (those inside the new ranges pass its filter) | fixed (decision 1461) |
| `Extrema_GenLocateExtCC` constructor (Extrema_GenLocateExtCC.hxx; Extrema_LocECC, Extrema_LocECC2d) | Accepts the root when `Ff(1) < Tolf && Ff(2) < Tolf`, the signed function values, not their absolute values | Any negative residual passes: a root the solver didn't converge to is reported as done when both functions are negative there | touch |
| `Extrema_ExtCC2d::Results(const Extrema_ExtElC2d&, ...)` (Extrema_ExtCC2d.cxx) | Calls `AlgExt.IsParallel()` before testing `AlgExt.IsDone()` | When Extrema_ExtElC2d is not done (a circle and a concentric ellipse of equal radii), Extrema_ExtCC2d throws StdFail_NotDone (a real throw in OCCT's release build too) instead of reporting not done (`Extrema_CurvePortTests.ExtCC2d_NotDoneElementary_Throws_AsInOcct`) | touch |
| `Extrema_ExtCC::PrepareParallelResult`, circle/circle (Extrema_ExtCC.cxx) | `continue` when `ExtPCir.NbExt() < 1` skips `aProjRng1.Shift(M_PI)` at the end of the loop body | The next iterations test the same projected range again instead of the shifted ones. Reached only when Extrema_ExtPElC finds no extremum (a NotDone Extrema_ExtPElC throws from `NbExt` first) | touch |
| `Extrema_CurveTool::DeflCurvIntervals`, `Extrema_Curve2dTool::DeflCurvIntervals` | `L += aP.Distance(aP1)` without moving `aP` to `aP1` | L is the sum of the distances from the first point to 22 others, not the length of the polygon, so the deflection and minimum length given to GCPnts_TangentialDeflection (and the "short curve" test) are not what the comments say | touch |
| `Extrema_GFuncExtPC::Value` and `SearchOfTolerance` (Extrema_GFuncExtPC.hxx) | Test only `D1c.X()` and `D1c.Y()` for infinity | A 3D first derivative that is infinite only in Z is not caught. Harmless in practice | leave |
## Phase 2: unit 3C, properties, persistence, hashes

| Where | What | Effect | Triage |
|---|---|---|---|
| `LProp_SurfaceUtils::RequireCurvature`, `IsUmbilic` (GeomLProp_SurfaceUtils.hxx), called by `GeomLProp_SLPropsBase::MaxCurvature`, `MinCurvature`, `MeanCurvature`, `GaussianCurvature`, `IsUmbilic` | Take the curvature fields by value, evaluated before the call to `IsCurvatureDefined()` that computes them | Asked before `IsCurvatureDefined()`, the curvatures are the previous parameters' (0 on a new object: a sphere of radius 2 reports MaxCurvature 0, then -0.5), and IsUmbilic compares the previous values (true on a new object). Confirmed against native BRepLProp_SLProps (oracle, 4,315 cases) | fixed (decision 1467) |
| `GeomLProp::Continuity` (GeomLProp.cxx) | For a B-spline at an inner knot, sets `cont = Degree() - Multiplicity()`, the order of continuity there, and then reads it on `GeomAbsToInteger`'s scale (2 for C1, 4 for C2) | A junction at a knot is reported lower than it is: a cubic's simple knot (C2) compares first derivatives only and gives at most C1, a double knot (C1) compares nothing and gives C0. A unit test pins it (not bound natively) | fixed (decision 1466) |
| `LProp_CurveUtils::CentreOfCurvature` (LProp_CurveUtils.hxx) | When the first derivative is null, `Curvature()` returns RealLast() without updating `myCurvature`, and the centre is computed from it and from the null normal D2 (D1.D1) - D1 (D1.D2) | Release builds compute a NaN centre with a stale curvature (0 on a new object); the port's `gp_Vec::Normalize` throws Standard_ConstructionError | touch |
| `GeomLProp_CurAndInf2d::Perform` (GeomLProp_CurAndInf2d.cxx) | Runs the curvature-extrema search and then the inflection search, each of which sets `myIsDone` | A failure of the curvature search on a B-spline or other curve is hidden when the inflection search succeeds | touch |
| `GeomTools_Curve2dSet::Add`, `GeomTools_SurfaceSet::Add` and the three sets' `Read` (GeomTools_*Set.cxx) | Add a null handle to the indexed map, which keeps one null entry; `Read` adds the null result of every object it can't read | When several objects of a table can't be read, they share one index and every later object gets an index one lower per extra failure, so the shapes of a `.brep` file refer to the wrong geometry. `Dump` and `Write` then dereference the null entry | touch |
| `GeomTools_UndefinedTypeHandler::PrintCurve`, `PrintCurve2d`, `PrintSurface` (GeomTools_UndefinedTypeHandler.cxx) | In the compact (file) form, write their message to std::cout instead of the stream | A table with an object of an unknown type is written with fewer objects than its count says, so it can't be read back | touch |
| `GeomTools_*Set` readers of trimmed, offset and swept objects | Pass a null basis (one that couldn't be read) to the constructors, which dereference it | A crash unless signals are converted to exceptions (`OCC_CATCH_SIGNALS`); the port reads the object as null | settled |
| `GeomHash_BSplineCurveHasher`, `Geom2dHash_BSplineCurveHasher`, `GeomHash_BSplineSurfaceHasher` comparison (.pxx) | Compare degree, knots, multiplicities and rationality, but neither periodicity nor pole counts, then loop over the first object's poles | With the same knots, a periodic B-spline has more poles than a non-periodic one: comparing the non-periodic one first finds them equal after its own poles; comparing the periodic one first reads past the other's poles (undefined behaviour in a `noexcept` operator). The port's `Pole` throws Standard_OutOfRange. A unit test pins it | touch |
## Phase 2: unit 3A, grid evaluation

| Where | What | Effect | Triage |
|---|---|---|---|
| `GeomGridEval_BSplineCurve::EvaluateGridDN`, `Geom2dGridEval_BSplineCurve::EvaluateGridDN` | Locate the span in the flat knots (`BSplCLib::LocateParameter` without multiplicities), then pass that flat index to `BSplCLib::DN` together with the knots and multiplicities, where it is read as a knot index; `LocateParameter` keeps a hint that is a valid knot index | DN is evaluated on another span's polynomial, extrapolated to U (errors of order 1); when the flat index is past the knots, building the span's knots reads outside the array (garbage in OCCT, IndexOutOfRangeException in the port). Surfaces of revolution and extrusion of B-spline curves inherit it in their grid DN. Pinned by `BSplineDN_UsesFlatSpanIndexWithKnots` (3D and 2D) | touch |
| `EvaluateGridDN` of GeomGridEval_BezierCurve, _BSplineCurve, _BezierSurface, _BSplineSurface, Geom2dGridEval_BezierCurve, _BSplineCurve | Return zero when the order exceeds the degree (in U or V for surfaces), also for rational geometry | A rational curve's or surface's derivatives above the degree are in general not zero (Geom and the adaptors return them): the grid differs by 100%. Pinned by `RationalDN_AboveDegreeIsZero` (curves 3D and 2D, B-spline surface) and in the Bezier surface consistency test | touch |
| `Geom2dGridEval_Parabola` (every method) | Builds the Y direction as the mirror axis turned by +90 degrees, ignoring the sense of the parabola's gp_Ax22d | A parabola with an indirect frame is evaluated mirrored (errors of order 1). Pinned by `IndirectParabola_GridIsMirrored`. The 3D evaluator takes the frame's Y direction and is right | fixed (decision 1495) |
| `GeomGridEval_Parabola`, `Geom2dGridEval_Parabola` (every method) | Divide by the focal length (1/(4F), 1/(2F)); Geom_Parabola and Geom2d_Parabola allow a focal length of 0, which ElCLib evaluates as the line Location + U·XDir | The grid of a zero-focal parabola is NaN at U = 0 and infinite elsewhere. Pinned by `ZeroFocalParabola_GridIsNotFinite` (3D and 2D) | leave |
| `GeomGridEval_OffsetCurve::EvaluateGridDN`, orders 1 to 3 | Reads the D1, D2 or D3 grid without checking that it is not empty; the grid is empty when the offset fails at a parameter (basis tangent zero or parallel to the offset direction). The 2D evaluator checks | Reads an empty array: undefined in OCCT, Standard_OutOfRange in the port. Pinned by `OffsetDN_ReadsEmptyGridAfterFailedOffset` | touch |
| `Geom_OffsetCurveUtils::EvaluateDN`, `Geom2d_OffsetCurveUtils::EvaluateDN` (both instantiations), case 3 (Geom_OffsetCurveUtils.pxx, Geom2d_OffsetCurveUtils.pxx) | Calls `EvaluateD3(..., aPnt, aDummy, aDummy, theDN)`: one variable receives both D1 and D2, so the basis D2 overwrites the basis D1 before CalculateD3 uses them | GeomAdaptor_Curve's and Geom2dAdaptor_Curve's DN(3) on an offset curve differ from Geom_OffsetCurve/Geom2d_OffsetCurve's (up to 1.9 relative) and can throw an undefined-derivative exception. Found by the grid consistency tests, which compare offset DN3 with the curve only | fixed (decision 1487) |
| `GeomConvert::ConcatG1`, `Geom2dConvert::ConcatG1` and `Geom2dConvert::ConcatC1` (GeomConvert.cxx, Geom2dConvert.cxx) | With ClosedFlag set and more than one G1 group, they reorder the curves and a local copy of the tolerances, then merge each multi-curve group with the caller's un-reordered `ArrayOfToler(j-1)` (GeomConvert::ConcatC1 reads the local copy) | Each merge uses another junction's tolerance, and the closure is never merged with ClosedTolerance: segments (0,0)→(1,0)→(2,0)→(2,1)→(1e-5,0) with tolerances {1e-7, 1e-12, 1e-7} and ClosedTolerance 1e-4 throw "Concatenation Error" (`ConcatG1_ClosedWire_MergesWithTheReorderedTolerances`, 2D and 3D) | fixed (decision 1831) |
| `Geom2dConvert::C0BSplineToArrayOfC1BSplineCurve` (Geom2dConvert.cxx) | Tests closure with `SquareDistance < Tolerance`, where C0BSplineToC1BSplineCurve uses `Tolerance * Tolerance` | A curve whose ends are farther apart than Tolerance (but within its square root) is taken as closed | touch |
| `Geom2dConvert_ApproxArcsSegments::makeApproximation` (Geom2dConvert_ApproxArcsSegments.cxx) | Appends makeLine's result even when it is null (two parameters farther apart than PConfusion whose points are within Confusion) | `GetResult()` can hold a null curve; the port leaves it out. Found by reading; 3,000 random curves never reached it | settled |
| `Geom2dConvert_reparameterise_evaluator` / `GeomConvert_reparameterise_evaluator` (Geom2dConvert.cxx, GeomConvert.cxx) | Pass a single `double&` as PLib::EvalPolynomial's result array | Would write past it if a derivative were requested; BSplCLib::FunctionReparameterise only asks for order 0, so latent | leave |
| `GeomConvert::SplitBSplineSurface(S, FromParam1, ToParam2, USplit = false, ...)` (GeomConvert_1.cxx) | The V branch tests `S->IsUPeriodic()` and calls `UReverse()` | A split in V with FromParam1 > ToParam2 (or SameOrientation false on a U-periodic surface) reverses U instead of V (`SplitBSplineSurface_InV_ReversesU_OCCT`; the oracle agrees) | fixed (decision 1490) |
| `GeomConvert_BSplineSurfaceKnotSplitting` (GeomConvert_BSplineSurfaceKnotSplitting.cxx) | When the surface already has continuity VContinuityRange in V, assigns `usplitIndexes` where it means `vsplitIndexes` | The U result is overwritten with the first and last V knot indices, and `NbVSplits`, `VSplitValue` and `Splitting` dereference a null handle (the port throws NullReferenceException; `KnotSplitting_SmoothV_ReproducesOcctError`) | fixed (decision 1491) |
| `GeomConvert_CompBezierSurfacesToBSplineSurface(Beziers, Tolerance, RemoveKnots)` (GeomConvert_CompBezierSurfacesToBSplineSurface.cxx) | The V knot removal loop tests `multU` (left over from the U loop), not `multV` | No V knot is removed when the U removals ended normally or with one U patch (`CompBezierSurfaces_WithTolerance_RemovesUKnotsOnly`). If the last U removal failed with multU > minus, the V loop runs until a removal fails; once a V knot is removed entirely, RemoveVKnot is called with a negative multiplicity, returns true, and the loop never ends (OCCT and the port; not tested) | touch |
| `GeomConvert_FuncCylinderLSDist::Gradient` (GeomConvert_FuncCylinderLSDist.cxx) | Scales `G(6) *= -2R` where it means `G(4)` | OCCT's release build writes past the 4-element vector and returns an unscaled radius derivative; the port throws on the index (`Cylinder_ValueValues_AndGradientReproducesOcctError`). `Value`/`Values`' value is right, and SurfToAnaSurf only calls `Value` | touch |
| `GeomConvert_CurveToAnaCurve::ComputeEllipse` through `ConicDefinition` (GeomConvert_CurveToAnaCurve.cxx) | Takes `cos2t = 1/sqrt(1 + t2d²)` and `sint = sqrt((1 - cos2t)/2)` both positive | The sign of the rotation is lost: the major axis always comes out within [0°, 45°] of the fitting frame's X axis, so ellipses oriented otherwise are rejected at any tolerance (an arc of radii 3 and 1.2 turned in 15° steps is recognised at 0–30° and 180–210° only; `ComputeEllipse_MajorAxisAt60Degrees_ReproducesOcctError`). By reading and the port's behaviour; no native binding reaches it. Probably affects ShapeAnalysis_CanonicalRecognition::IsEllipse | fixed (decision 1492) |

## Phase 2: unit 4A, surface extrema

| Where | What | Effect | Triage |
|---|---|---|---|
| `Extrema_GenExtPS` tree search, `Bnd_SphereUBTreeSelectorMax` (Extrema_GenExtPS.cxx) | Starts `myMaxDist` at 0 and rejects with `Bnd_Sphere::IsOut(P, myMaxDist)`, which rejects a sphere whose minimal distance exceeds that value (the test of the minimum selector) | Every sphere is rejected, so the search for the maximum (Extrema_ExtAlgo_Tree with MAX or MINMAX) refines from the first grid node, not the farthest one, and misses the maximum. Pinned by `GenExtPS_TreeMax_StartsFromTheFirstNode_AsInOcct` | touch |
| `CorrectNbSamples` (Extrema_GenExtPS.cxx) | When the U steps are more than ten times the V steps, multiplies `theNbV` again, where `theNbU` was meant | The tree search's sampling is refined in V instead of U for surfaces much longer in U | touch |
| `Extrema_ExtPRevS::Initialize` | Resets `myIsAnalyticallyComputable` to false before `if (myS != theS)`, which alone sets it | Initialising again with the same surface (handle) loses the analytic path: Extrema_GenExtPS (then initialised) answers instead. Extrema_ExtPS never does this | touch |
| `Extrema_ExtPS::TrimmedSquareDistances` | Returns `d11` ... `P22`, which nothing computes | Zeros and the origin after the default constructor, uninitialised memory after the others (zeros in the port) | leave |
| `Extrema_GlobOptFuncCQuadric::QuadricParameters`, default branch | Writes `theUV(2) = myUl` where `myVf` was presumably meant | On a surface that is not a quadric (LoadQuad never called with one), V is the last U parameter. GenExtCS calls it only on quadrics | leave |
| `Extrema_ExtSS` on parallel surfaces (Extrema_ExtSS.cxx) | `NbExt()` is 1 (the distance is kept) but no points are stored | `Points(1, ...)` reads past empty sequences (undefined in OCCT's release build, Standard_OutOfRange in the port); BRepExtrema_ExtFF tests `IsParallel` first. Pinned by `ExtSS_ParallelPlanes` | touch |
| `Extrema_ExtSS::Perform` | Resets `myIsPar` only on the elementary path (only `Initialize` resets it otherwise) | After a parallel result, a later Perform taking the generic path (Extrema_GenExtSS) with the same Initialize still reports parallel | touch |

## Phase 2: unit 4B, bounding boxes

| Where | What | Effect | Triage |
|---|---|---|---|
| `BndLib::Add(gp_Parab, P1, P2, ...)`, `BndLib::Add(gp_Parab2d, ...)`, `BndLib::Add(gp_Hypr2d, ...)`, finite arcs (BndLib.cxx) | Add only the two ends and, when P1·P2 < 0, the vertex (parameter 0); the coordinate extrema of a parabola or hyperbola turned in its plane are elsewhere. The 3D hyperbola adds them (`ComputeBox`) | The box does not contain the arc: a parabola with axes (1,1)/√2, (−1,1)/√2, focal 2 on [−6, 6] gets Xmin −1.0607 where the curve reaches −1.4142; a 2D hyperbola (3, 2) at 45 degrees on [−1, 1] gets Xmin 1.6114 for 1.5811. BndLib_Add3dCurve/Add2dCurve go through GeomBndLib, not these functions; direct callers are affected | fixed (decision 1429) |
| `GeomBndLib_Cone::Box` with an infinite V bound (GeomBndLib_Cone.cxx) | Adds the circles at the finite bound and at V = 0, then opens the box only along the axis (GeomBndLib_InfiniteHelpers), as for a cylinder; a cone widens away from its apex | For a cone whose axis is a coordinate axis the radial sides stay closed, so the box misses the surface away from V = 0 (an axis-aligned cone on V in [0, +inf) gets X and Y within the V = 0 circle). Pinned by `GeomBndLib_SurfaceIssueTest.Cone_InfiniteV_RadialDirectionsStayClosed` | touch |
| `GeomBndLib_OffsetSurface::Box`, fallback without an analytic equivalent (GeomBndLib_OffsetSurface.cxx) | Enlarges the basis box by \|offset\| and then by the tolerance; `Bnd_Box::Enlarge` keeps the larger gap | The gap is max(\|offset\|, Tol) instead of \|offset\| + Tol: the tolerance is lost whenever it is smaller than the offset. Pinned by `GeomBndLib_SurfaceIssueTest.OffsetSurface_Fallback_ToleranceNotAddedToOffset` | touch |
| `GeomBndLib_OtherSurface::Box`, `GeomBndLib_BezierSurface::Box` on a strict sub-patch, `GeomBndLib_BSplineSurface::Box` outside the surface's bounds | Add the points of a sample grid (at most 50 x 50, from the full-range sample counts for Bezier and B-spline) and enlarge by the tolerance only, where the curve evaluators enlarge their samples by 1.5 times a deflection estimate | The box can miss extrema between samples: a quadratic Bezier patch z = 20u(1-u) on U in [0.1, 0.65] gets Zmax 4.968 for a true 5.0. Pinned by `GeomBndLib_SurfaceIssueTest.BezierSurface_TrimmedPatch_SampledWithoutDeflection` | touch |
| `GeomBndLib_Parabola2d::Box`, finite arcs (GeomBndLib_Parabola2d.cxx) | As the BndLib parabola: adds only the two ends and, when U1·U2 < 0, the vertex; the coordinate extrema of a parabola turned in its plane (at U = -2F·Yk/Xk) are missed | The box does not contain the arc: a parabola with mirror axis (1,1)/√2, focal 2, on [−6, 6] gets Xmin −1.0607 where the curve reaches −1.4142. Reaches `GeomBndLib_Curve2d` and `BndLib_Add2dCurve` (for conic arcs, and Geom2dAdaptor_Curve of parabolas). Pinned by `GeomBndLib_Curve2dIssueTest.Parabola2d_TurnedArc_MissesCoordinateExtremum` | fixed (decision 1429) |
| `GeomBndLib_Parabola::Box(gp_Parab, U1, U2, ...)`, finite arcs (GeomBndLib_Parabola.cxx) | Adds only the two ends and, when U1·U2 < 0, the vertex (as BndLib::Add(gp_Parab)); the coordinate extrema of a parabola turned in its plane, at U = −2F·Yk/Xk, are missed | The box does not contain the arc: axes (1,1,0)/√2, (−1,1,0)/√2, focal 2, on [−6, 6] gives Xmin −1.0607 where the curve reaches −1.4142 at U = 4. Reaches GeomBndLib_Curve, BndLib_Add3dCurve, and so BRepBndLib on parabolic edges. Pinned by `GeomBndLib_CurveIssueTests.TurnedParabola_BoxMissesCoordinateExtremum` | fixed (decision 1429) |
| `Extrema_ExtCS::Perform`, a line against a plane (not parallel), torus, cone, Bezier, B-spline, swept, offset or other surface, when the line's range, clipped to the projections of the surface's box corners, is shorter than Confusion or empty (Extrema_ExtCS.cxx) | Projects the middle point aPm with Extrema_ExtPS, then passes `PC.Value()` of a default-constructed Extrema_POnCurv (the origin) to AddSolution as the point on the curve, instead of aPm | Each such extremum reports the origin as its point on the curve (its parameter and square distance are right). Reached by short lines and by lines whose range lies beyond the surface's box | fixed (decision 1430) |
| `GeomBndLib_BSplineCurve::Box` (GeomBndLib_BSplineCurve.cxx) with `Geom_BSplineCurve::Segment` (Geom_BSplineCurve.cxx), periodic curves | When [U1, U2] differs from the curve's range but U2 ends a whole period from U1 to the last bit (typically U2 = Last - 1 ulp, as BRepTools::UVBounds returns for a face on an extrusion or revolution of a periodic B-spline), `aMinDiff` is 0 and the segment tolerance becomes 0; `Segment(U1, U2, 0)` then leaves one pole of the result unwritten (seen: knots ×3, ×1, ×1, ×3, five poles, the fourth never set) | Native OCCT reads uninitialised memory for that pole (values like 6.95e-310, or a leftover coordinate): the box of the edge or face changes from run to run (seen in BRepBndLib::Add and AddOBB on a periodic degree-2 B-spline with 3 poles). The port reads 0, so its box is deterministic but also takes in the origin, bounded by the full curve's pole hull. The oracle test skips these cases (`BndLibOracleTests.SegmentsWithZeroTolerance`) | touch |

## Phase 3: unit 3D, 2D intersection bases

| Where | What | Effect | Triage |
|---|---|---|---|
| `Intf_SectionPoint::IsOnSameEdge`, both points on vertices of the first argument (Intf_SectionPoint.cxx) | Compares `IndexT1` (the second argument's address) where the other branches compare `IndexO1` | Two points on the same vertex of the first argument are not "on the same edge" unless their second-argument addresses agree (the tool-side test then decides). Pinned by `IntfTests.SectionPoint_IsOnSameEdge` | touch |
| `Intf_InterferencePolygon2d::Clean` (Intf_InterferencePolygon2d.cxx) | `Only1Seg` is set before the loop over the tangent zones and only ever reset by an EDGE/EDGE pair, so a zone inherits `true` from an earlier short zone | A later zone spanning several segments can be turned into a single section point when an earlier zone was short. Not reproduced | settled |
| `Intf_Tool::HyprBox` (3D, Intf_Tool.cxx) | Writes `beginOnCurve[nbSeg]`/`endOnCurve[nbSeg]` without the `nbSeg < 6` guard that the fix of bugs #75/#76 added to Hypr2dBox, Parab2dBox and ParabBox | More than six crossings would overflow the fixed arrays in C++ (the port throws IndexOutOfRangeException). Not reproduced | settled |
| `IntImpParGen_Intersector::Perform` (IntImpParGen_Intersector.gxx) | `Resultat1`/`Resultat2` are sized `2 + 2·(1 + n)` for the n solution intervals before the intervals shifted by a period on a closed implicit curve (`nb_segments_crees`, up to 2 per interval) are added, and And_Domaine_Objet1_Intersections may write two values per interval | With three or more shifted intervals C++ writes out of bounds (the port throws Standard_OutOfRange). Not reproduced; reached through IntCurve_IntImpConicParConic and Geom2dInt (unit 3E) | settled |
## Phase 2: unit 5A, GeomLib and Approx

| Where | What | Effect | Triage |
|---|---|---|---|
| `GeomLib::CancelDenominatorDerivative` through `CancelDenominatorDerivative1D` (GeomLib.cxx) | Reparametrises the surface's U knots to [0, 1] before testing whether it can be treated, and restores them only after a treatment | A surface that can't be treated (non-rational, periodic, or already treated) comes back reparametrised to [0, 1]. OCCT changes the caller's surface; the port changes a copy (`CancelDenominatorDerivative_Untreatable_LeftReparametrised_OCCT`) | touch |
| `GeomLib_Interpolate::GeomLib_Interpolate` (GeomLib_Interpolate.cxx) | Never sets `myError` on success | `Error()` returns an uninitialised value after a successful interpolation; the port returns `GeomLib_NoError` | settled |
| `Adaptor3d_CurveOnSurface::Load(C)` and `Load(S)` (Adaptor3d_CurveOnSurface.cxx), as Approx_CurvlinFunc uses it | Keep `myIntervals`/`myIntCont`, so a reloaded adaptor returns the intervals cached for the previous curve | Approx_CurvlinFunc's case 3 (two curves on two surfaces) reuses one adaptor in NbIntervals, Intervals and Init: the second curve's discontinuities are lost and its parameter tables are built on the first curve's C3 intervals (`CurvlinFunc_TwoCurves_ReusesTheFirstCurvesIntervals_OCCT`: knots at 0.3 and 0.6 fuse to 2 intervals, not 3) | touch |
| `Approx_CurvlinFunc::Trim`, case 3 (Approx_CurvlinFunc.cxx) | Computes the second curve's parameters with `GetUParameter(CurOnSur, First, 1)`, the first curve's tables (myUi_1, mySi_1, myLength1) | The second curve is trimmed at the first curve's parameters. Latent: Approx_CurvilinearParameter's case 3 evaluator never calls Trim (commented out in OCCT). By reading | leave |
| `Approx_SweepApproximation::Dump` | Writes the line end after the 2D errors, and " Not Done ", to std::cout instead of its stream | Cosmetic; the port writes them to the TextWriter | leave |
| `GeomConvert_SurfToAnaSurf::ConvertToAnalytical` (GeomConvert_SurfToAnaSurf.cxx) | The "diagonal" that caps the tolerance evaluates `mySurf->Value((U1 + U2), (V1 + V2) / 2)`, at U1 + U2 rather than the middle | The cap (diagonal / 1000) is measured to a point that can be outside the range (B-splines are extrapolated), so the effective tolerance depends on where the parameter range sits. By reading | touch |
| `GeomConvert_SurfToAnaSurf::TryCylinerCone` | When the first or last iso is not a circle (a cone's apex, R = 0), its point is `iso->Value((Last - First) / 2)`, half the range rather than its middle | The cone's axis goes through the wrong point unless the iso's range starts at 0. By reading | leave (decision 1493) |
| `GeomConvert_SurfToAnaSurf::TryCylinderByGaussField` | Sizes the least-squares point array `theNbU * theNbU` rather than `theNbU * theNbV` | Latent: its only caller passes 7 × 7 | leave |
## Phase 3: unit 3A, topology core

| Where | What | Effect | Triage |
|---|---|---|---|
| `BRep_Builder::UpdateEdge(E, P1, P2, S, L)` (BRep_Builder.cxx), polygons on a closed surface | Stores the `BRep_PolygonOnClosedSurface` with the identity location, where the one-polygon form stores `L.Predivided(E.Location())` and `BRep_Tool::PolygonOnSurface(E, S, L)` looks for that | On a face at a location (or an edge with one), the two polygons are stored but never found again by face or surface; only the index form `PolygonOnSurface(E, P, S, L, i)` returns them. Pinned by `BRep_ToolTests.PolygonOnClosedSurface_is_stored_without_the_location_as_in_OCCT`; not checked natively (Poly_Polygon2D isn't bound) | touch |
| `BRepTools::UpdateFaceUVPoints` (BRepTools.cxx) | `if (TE->Locked()) return;` stops at the first locked edge; `continue` was presumably meant (the edges' own UV points are independent) | The edges after a locked one keep stale UV points. In the port every frozen edge is locked, so this happens when a new face is built on edges shared with a published shape. Pinned by `BRep_ToolTests.UpdateFaceUVPoints_stops_at_the_first_locked_edge_as_in_OCCT` | touch |
| `BRep_Tool::Range(E, S, L, First, Last)` (BRep_Tool.cxx) | A query that ends with `E.TShape()->Modified(true)` | Asking for a range marks the edge modified and so unchecked; `BRep_Tool::Parameter(V, E, S, L)` and `Range(E, F, ...)` reach it. The port does the same on unfrozen edges and nothing on frozen ones (decision 312) | touch |

## Phase 3: unit 3E, 2D curve intersection

| Where | What | Effect | Triage |
|---|---|---|---|
| `SET_BOUNDED_DOMAIN` (IntCurve_IntConicConic.cxx) | Compares `bsup` with `domain.FirstParameter()` instead of `LastParameter()` | A last bound is replaced by the domain's even when it is already inside it. Reached only by parabola and hyperbola intersections with a half-bounded domain. Not reproduced | settled |
| `IntCurve_IntConicConic::Perform(gp_Lin2d, gp_Parab2d)` (IntCurve_IntConicConic.cxx) | Offsets the line by `(maxtol·Dy, maxtol·Dx)`, which is not normal to it (`(-Dy, Dx)` was meant) | The band in which the analytic solutions are searched is skewed; the results are refined afterwards. Not reproduced | settled |
| `IntCurve_IntConicConic::Perform(gp_Lin2d, gp_Elips2d)`, segment ends (IntCurve_IntConicConic_1.cxx) | Evaluates the end of a solution segment at `SolutionEllipse[i].Binf` instead of `.Bsup`, and on `Pos1b == End` sets `Esup = DL.LastParameter()` (the line's domain) instead of `DE.LastParameter()` | A line/ellipse segment (tangency along a stretch within the tolerance) can get the wrong end point and transition and, at the ellipse domain's end, the line's parameter as its ellipse parameter. Not reproduced | touch |
| `IntCurve_IntPolyPolyGen::findIntersect`, recursion on tangent zones (IntCurve_IntPolyPolyGen.gxx) | Calls `Perform(C1, RecursD1, C2, RecursD2, Tol, TolConf, ...)` with the two tolerances swapped | The refinement treats Tol as the confusion tolerance and TolConf as the tolerance. No effect when they are equal, as Geom2dAPI_InterCurveCurve passes them | leave |
| `HeadOrEndPoint` (IntCurve_IntPolyPolyGen.gxx) | Computes `SP2` when the second domain's end matches but never uses it: P2 is not moved to that end, as P1 is moved to `SP1` | A solution snapped to an end of the second curve keeps the unsnapped point. Not reproduced | settled |
| `IntCurve_PConicTool::D2` on a line (IntCurve_PConicTool.cxx) | Clears `Tan` (which LineD1 then writes) and never writes `Norm` | The caller's normal is left as it was; OCCT's callers pass default-constructed (0, 0) vectors, which the port writes | leave |

## Phase 2: unit 6A, projection

| Where | What | Effect | Triage |
|---|---|---|---|
| `ProjLib_CompProjectedCurve::Perform` | Sets `approx2d = false` for an isoparametric part inside the loop over parts and never restores it | Every later part that isn't isoparametric gets no 2D approximation: its 2D result is the previous part's iso line (`Perform_IsoThenFreePart_ReusesIsoLine_OcctBug`) | fixed (decision 1460) |
| `ProjLib_CompProjectedCurve` (file-static FindSplitPoint) | Reads `Point(aMinIdx)` with `aMinIdx = -1` when Extrema_ExtPS is done without an extremum | Out-of-range read in OCCT; the port throws Standard_OutOfRange. Not reached by a test | touch |
| `ProjLib_ComputeApproxOnPolarSurface::ProjectUsingInitialCurve2d` | Tests `BSS->MaxDegree() == 1` (and `BS->MaxDegree()` for Bezier surfaces), the static maximum degree (25), to detect bilinear surfaces | The pole-wise bilinear branches are dead code; inside them, square distances truncated to int, extrema started from coordinates instead of parameters, and `myProjIsDone` reset after every pole (`BilinearSurface_PoleWiseBranchIsDead`) | leave |
| `ProjLib_ComputeApproxOnPolarSurface::Perform` (C0 B-spline path) and `BuildInitialCurve2d` | Can read `Inter2d(nbInter2d + 2)`; dereferences a null `G2dC` for an initial curve of type OtherCurve; reads `ext.SquareDistance(1)` without checking `NbExt() > 0` | Out-of-range reads and a null dereference in OCCT; the port throws. Not reached by a test | touch |
| `ProjLib_ComputeApprox` (file-static Function_SetUVBounds, cylinder branch; the same logic for spheres, cones and tori) | When the sampled U range is outside [0, 2π], replaces the window by mid ± π, a width of 2π, even for a curve spanning more | A curve winding more than once around the axis gets a wrong pcurve (off by up to the diameter) while the reached tolerance stays tiny (`Cylinder_HelixOfMoreThanATurn_WrongResult_OcctBug`; ProjLib_ProjectedCurve reaches it for B-spline helices) | fixed (decision 1459) |
| `ProjLib_ComputeApprox` (Function_Value, sphere branch) | Tests `S > U1 \|\| S < U2`, always true (`S < U1 \|\| S > U2` was meant) | Harmless: ElCLib::InPeriod leaves values already in the window unchanged. By reading | leave |
| `ProjLib_ComputeApprox` (Function_SetUVBounds, sphere/circle branch) | Adjusts U2 to the seam, then overwrites it with the U of the point at `W1 + PI/8`, a fixed parameter offset | The bounds depend on the circle's parameterisation. By reading | touch |
| `ProjLib_ComputeApprox` (Function_SetUVBounds, cylinder/closed ellipse) | Calls `mySurface->D1(U1, U2, ...)` with two U parameters | Harmless on a cylinder (D1U doesn't depend on V). By reading | leave |
| `ProjLib_ProjectOnPlane::Load`, parabola | Gives the projected parabola the focal `Focal / \|f(Xc)\|`; the right one is `Focal * \|f(Yc)\|² / \|f(Xc)\|` | Wrong focal when the projection shortens the parabola's Y direction (`Parabola_TiltedAboutItsXAxis_WrongFocal_OcctBug`) | fixed (decision 1465) |
| `ProjLib_ProjectOnPlane::Load`, parabola projected to a line | Builds a Geom_Line, then calls GetTrimmedResult with the null parabola | Null dereference in OCCT, NullReferenceException in the port (`Parabola_ProjectedToLine_OcctDereferencesNull`) | touch |
| `ProjLib_ProjectOnPlane` (callers of PerformApprox) | Dereferences a null curve when the approximation fails (`!IsAllApproximated`) | Null dereference. Not reached by a test | touch |
| `ProjLib_ProjectOnSurface::Load` | `MC.Curve(1, Poles)` fills the result array instead of `LocalPoles`, which stays at the origin; and passes the increment `Inc` where `BSplCLib::IncreaseDegree` takes the new degree | The result curve's poles are lost: it ends at the origin (`Load_PolesLost_OcctBug`). No OCCT code calls this class | leave |

## Phase 3: unit 3F, curve-surface intersection

| Where | What | Effect | Triage |
|---|---|---|---|
| `GeomAPI_ExtremaCurveCurve::TotalPerform` (GeomAPI_ExtremaCurveCurve.cxx) | Compares each pair of end points with the first total distance (`aTotalDist2`), which it never updates | A later pair can replace a nearer one: segments [1, 2] on the X and Y axes give a `TotalLowerDistance` of √8 instead of √2. Pinned by `ExtremaCurveCurve_TotalReachesTheEnds` | fixed (decision 1462) |
| `GeomAPI_ExtremaCurveCurve::Init` with ranges (GeomAPI_ExtremaCurveCurve.cxx) | Loads the whole curves (`myC1.Load(C1)`, no range) for the total distance | The Total methods report the curves' natural end parameters with the ranges' end points and project onto the whole curves: ranges [1, 2] of the segments [0, 3] give a total distance 1 at parameters (0, 0), where the answer is √2. Pinned by `ExtremaCurveCurve_TotalOnRangesReportsTheCurveEnds` | fixed (decision 1463) |
| `GeomAPI_Interpolate` (`BuildParameters`, `Interpolate`), `GeomAPI_PointsToBSpline::Init(Points, Params, ...)` (GeomAPI_Interpolate.cxx, GeomAPI_PointsToBSpline.cxx) | Index the caller's arrays from 1 (`ParametersPtr->Value(ii)` at a point index, `Value(1)`, `SetValue(1)`, `Params(i)`) | Arrays with another lower bound are read at the wrong places, or out of range. Not reproduced | settled |
| `GeomAPI_ExtremaCurveSurface::Init` (GeomAPI_ExtremaCurveSurface.cxx) | Hands Extrema_ExtCS a pointer to a local GeomAdaptor_Surface | The pointer dangles after Init (Extrema_ExtCS reads it again in later calls). Harmless in the port, which keeps a reference | leave |
## Phase 3: unit 3B, modification and adaptors

| Where | What | Effect | Triage |
|---|---|---|---|
| `BRepTools_NurbsConvertModification::NewCurve2d`, a seam edge without a 3D curve (BRepTools_NurbsConvertModification.cxx) | Calls `BRepTools::EvalAndUpdateTol(newE, C3d, ...)` with the null `C3d`, which dereferences it (`C3d->IsPeriodic()`) | A crash in OCCT when a pcurve-only seam on a non-B-spline surface or pcurve is converted; the port throws NullReferenceException. By reading | touch |
| `BRepTools_GTrsfModification::NewTriangulation` (BRepTools_GTrsfModification.cxx) | Tests `myGScale < 0.0` to reverse the triangles of a mirroring transformation, but `myGScale` is a maximum of absolute values | The triangles of a mirrored triangulation keep their orientation (TrsfModification tests `ScaleFactor() < 0`). By reading | touch |
| `BRepTools_GTrsfModification::NewPolygonOnTriangulation` | Returns true when the face has a triangulation even if the edge has no polygon on it | The modifier then calls `UpdateEdge` with a null polygon on the new edge. By reading | touch |
| `BRepTools_Modifier::Rebuild` with a modification whose `NewCurve2d` returns true without setting the curve (BRepTools_Modifier.cxx; `BRepTools_TrsfModification::NewCurve2d` through `GeomLib::SameRange` on a zero-length range) | The pcurve handle is declared once for all the edges of the face | The edge gets the previous edge's pcurve. The port's `out` parameter is null instead (decision 346). Not reproduced | settled |
| `BRepTools_History::AddModified`/`prepareModified` (BRepTools_History.cxx) | Removing a shape from another's generated list leaves the list bound when it becomes empty | `HasGenerated()` stays true, and Dump counts the shape, with nothing generated. Kept; pinned by `BRepTools_ModifierTests.History_records_modifications_generations_and_removals` | touch |
## Phase 3: unit 3H, global properties

| Where | What | Effect | Triage |
|---|---|---|---|
| `BRepGProp_Gauss::convert(Inertia, Coeff, IsByPoint, ...)` (BRepGProp_Gauss.cxx) | The test `std::abs(Mass) >= EPS_DIM && theIsByPoint` sends every region bounded by a plane (`theIsByPoint` false) to the else branch, which sets the mass to 0 and the centre to the origin; the inner `if (theIsByPoint)` and its plane branch are dead | `BRepGProp_Vinert` by a plane (its Perform and constructors with a gp_Pln) always reports a null volume and centre, with the matrix of inertia computed. BRepGProp's statics never ask for it (VolumePropertiesGK by plane uses BRepGProp_VinertGK, which is right). Pinned by `BRepGPropKnownAnswerTests.Vinert_by_plane_returns_a_null_mass_as_OCCT_does`; Vinert isn't bound natively | fixed (decision 1468) |
| `BRepGProp_Face::VKnots` (BRepGProp_Face.cxx), planes, cylinders and cones | Writes the face's U bounds (`FirstUParameter`, `LastUParameter`) as its V knots | On a natural face of these surfaces the adaptive integration (`Sinert`/`Vinert` with Eps) starts its V subdivision from U values: knots outside the V range are ignored, inside ones split it at an arbitrary place. Changes the subdivision, so the error and the last digits, not the converged result. Ported as OCCT | fixed (decision 1469) |
| `BRepGProp_VinertGK::GetAbsolutError` (BRepGProp_VinertGK.hxx) | Declared, never defined | A link error for a caller in C++; the port returns the absolute error that Perform returns | settled |
| `BRepGProp_Face::GetTKnots` (BRepGProp_Face.cxx) | Copies the B-spline face's V knots and doesn't use them; the documentation promises arc parameters whose points have V close to the face's V knots | With IsUseSpan the outer Gauss-Kronrod integral is split only at the pcurve's own knots, not at the surface's V knots. Accuracy only. By reading | touch |
| `BRepCheck_Shell::Closed` (BRepCheck_Shell.cxx) | Reports BRepCheck_RedundantFace when `!mapS.Add(aF)`; `NCollection_IndexedMap::Add` returns the index, never 0 | A face repeated in a shell is never reported as redundant by Closed (Minimum reports NotConnected, or Closed InvalidMultiConnexity); only Orientation reports RedundantFace. Pinned by `BRepCheckTests.Shell_RedundantFace` | touch |
| `BRepCheck_Shell::NbConnectedSet` (BRepCheck_Shell.cxx) | When the face that empties the set of free faces starts a new connected set, `break` leaves before the shell being built is appended | A last connected set of one face is lost from `theSets` and the count. By reading; not pinned | touch |
## Phase 3: unit 3C, BRep I/O

The input bugs are found by reading and pinned by the port's untrusted-input tests; they aren't run natively, where
they hang, overflow the stack or read outside memory and would take the test host down.

| Where | What | Effect | Triage |
|---|---|---|---|
| `TopTools_LocationSet::Read` (TopTools_LocationSet.cxx) | `while (l1 != 0) { IS >> p; ... IS >> l1; }` never checks the stream; `myMap(l1)` isn't range-checked in a release build; `L1.Powered(p)` of a complex location recurses `p` times | A location chain cut short loops for ever (the failed extraction leaves `l1` as it was), growing the location; a bad index reads outside the map; a large power of a complex location overflows the stack. The port throws `Storage_StreamFormatError` (decision 383) | settled |
| `TopTools_ShapeSet::Read` (TopTools_ShapeSet.cxx) | The sub-shape loop `do { Read(SS, IS, nbShapes); ... } while (!SS.IsNull())` never checks the stream, and `Read(S, IS, nbshapes)` indexes `myShapes(nbshapes - atoi(buffer + 1) + 1)` without a range check | A list cut short loops for ever (the uninitialised or stale buffer keeps naming a shape) or reads outside the map; so does a reference to a shape not read yet | settled |
| `TopTools_ShapeSet::Read`, `BRepTools_ShapeSet` readers, GeomTools' set readers | `char buffer[255]; IS >> buffer;` with no width | A word of 255 characters or more writes past the buffer | settled |
| `BRepTools_ShapeSet::ReadGeometry`, vertex and edge (BRepTools_ShapeSet.cxx) | The representation loops `do { ... IS >> val; ... } while (val > 0)` never check the stream | On input cut short the last `val` stays positive and the loop never ends; for a vertex it appends the same point representation each time, without bound | settled |
| `BRepTools_ShapeSet::ReadGeometry`, a face of a triangulation only | `myTriangulations.FindKey(s)` without a range check (the other branch checks `s`) | A bad index reads outside the map | settled |
| `BRepTools_ShapeSet::ReadPolygon3D`, `ReadPolygonOnTriangulation`, `ReadTriangulation` | The node and triangle counts, declared outside the loops, are read without checks and allocated for; on a failed stream they keep the last value | A huge or negative count allocates without bound or fails; after a failure every remaining polygon or triangulation allocates again with the last count | settled |
| `BRepTools_ShapeSet::AddGeometry` (also BinTools_ShapeSet) | Adds an edge's polygon on triangulation and its triangulation to the maps without checking for null; `WritePolygonOnTriangulation` and `WriteTriangulation` dereference them | Writing such an edge crashes. The port throws `Standard_NullObject` when the edge is added (decision 384; in BinTools in `AddShape`) | settled |
| `TopTools_ShapeSet::Add` (and `BinTools_ShapeSet::AddShape`, `BinTools_ShapeWriter::WriteShape`) | Recurse once per level of nesting | A shape nested deeply enough overflows the stack and ends the process. The port throws `InsufficientExecutionStackException` | settled |
| GeomTools' `ReadCurve`, `ReadCurve2d`, `ReadSurface` (also BinTools' sets) | Trimmed, offset, extrusion and revolution objects read their basis recursively, without a limit | Deeply nested geometry in a file overflows the stack (the port caps it at 64 levels) | settled |
| `BinTools_ShapeSet::WriteShape` / `ReadShape`, `BinTools_ShapeWriter` / `BinTools_ShapeReader`, a point on a surface | Write `Parameter2()` before `Parameter()`; both readers take the first value as the first parameter | The parameters of a point on a surface are swapped on every binary round trip (BinTools tests `Point_representations_follow_OCCT`) | fixed (decision 1500) |
| `BinTools_ShapeReader::ReadShape`, a point on a curve on a surface (BinTools_ShapeReader.cxx) | Tests `aSurface.IsNull()` where `!aSurface.IsNull()` was meant | The representation is dropped when the surface is there, or built with a null surface when it isn't | touch |
| `BinTools_SurfaceSet::Write`, `BinTools_Curve2dSet::Write` | Dereference the null entry the sets can hold, outside their `try` | Writing a set holding a null surface or 2D curve (from a point representation read with a missing one) crashes; the port throws `Standard_Failure` (found by fuzzing) | touch |
| `BinTools_ShapeSet::ReadShape`, vertices of versions 1, 2 and 4 | Reads a real before the type byte and steps back unless the next byte is 1 to 3, for files written with the parameter first; version 4 files write the type first | A parameter whose last stored byte is 1, 2 or 3 (a positive real below about 1e-303) is misread. Ported as is | leave |
| BinTools readers: `ShapeReader` references and locations, `LocationSet::Read` | Stream positions and location indices aren't checked; `Powered` of a complex location recurses per power | A reference cycle recurses until the stack overflows; bad indices read outside the maps | settled |
## Phase 3: unit 3G, classification and boxes

| Where | What | Effect | Triage |
|---|---|---|---|
| `IntCurvesFace_ShapeIntersector::Load` (IntCurvesFace_ShapeIntersector.cxx) | Appends the new faces' intersectors without clearing those of a shape loaded before, and sets `myNbFaces` to the new count | A second Load intersects the first shape's faces (the first `myNbFaces` of the sequence), not the new shape's. Pinned by `IntCurvesFaceTests.ShapeIntersector_second_Load_keeps_the_first_shape_OCCT_issue` | touch |
| `IntCurvesFace_ShapeIntersector::PerformNearest` | A face is skipped (not intersected again) once the nearest parameter found reaches `theParMin`, but `SortResult` still reads its intersector | The skipped faces report the points of their previous Perform, for another line. By reading; reached only when a point is found exactly at `theParMin` | touch |
| `IntCurvesFace_Intersector::InternalCall` (IntCurvesFace_Intersector.cxx) | Projects the point on `BRep_Tool::Curve` of every edge of the face, without checking for a null curve | On a face with a degenerated edge (or a pcurve-only edge) whose point the larger tolerance classifies ON, the projection gets a null curve and throws (Standard_NullObject in OCCT; the port throws too). By reading | touch |
| `BRepBndLib.cxx`, `FindExactUVBounds` | Calls `ElCLib::InPeriod(u, umin, umax)` and `InPeriod(v, vmin, vmax)` as if they changed their argument; OCCT 8's InPeriod returns the value | On periodic surfaces the mid-points of the pcurves aren't brought into the surface's period, so a face bounded by its iso-lines outside the first period isn't recognised as a natural restriction: AddOptimal then bounds the face from its pcurves' UV box and the edges' boxes (AdjustFaceBox) instead of the surface's natural bounds. By reading | touch |
| `BRepTopAdaptor_FClass2d::BRepTopAdaptor_FClass2d` (BRepTopAdaptor_FClass2d.cxx) | Returns from the constructor when an edge has no pcurve on the face | The classifier keeps the polygons of the wires before it, and the UV box as far as it got: points are classified against part of the face. By reading | touch |
| `BRepTopAdaptor_FClass2d::BRepTopAdaptor_FClass2d`, a wire of fewer than 4 sample points | Reads `SeqPnt2d(1)` and `SeqPnt2d(2)` whatever the number of points (out of range with `No_Standard_OutOfRange`) | Undefined in OCCT for a wire that gives fewer than two points (all its edges shorter than 1e-9 in parameter but one, or 3D-degenerate); the port throws Standard_OutOfRange. By reading | touch |
| `TopClass_FaceClassifier.pxx` with `BRepClass_FaceExplorer::CurrentEdge` | One `BRepClass_Edge` is reused for all the edges, and `BRepClass_Edge::SetNextEdge` returns without writing when the edge's last vertex has no single other edge (a closed edge, three edges at a vertex) | That edge keeps the previous edge's next edge, which `BRepClass_Intersector`'s CheckSkip (vertex tolerance above the gap tolerance) then uses as its neighbour. Kept (the port's `ITopClass_FaceExplorer.CurrentEdge` takes the edge by `ref`, decision 390). By reading | touch |

## Phase 3: unit 3L1, Draw-lite shapes

Found while porting Draw's commands (Draw-lite reproduces them, so the corpus sees OCCT's behaviour).

| Where | What | Effect | Triage |
|---|---|---|---|
| `tcopy` (BRepTest_BasicCommands.cxx) | The pairs loop reads `a[i + iFirst]` and `a[i + iFirst + 1]` with `i` the pair number, where `2 * i` was meant | `tcopy a b c d` copies a to b, then b to c; d is never set. Pinned by `DrawShapeCommandsTests.Tcopy_copies_the_shapes` | leave |
| `ShapeAnalysis_ShapeTolerance::InTolerance` (ShapeAnalysis_ShapeTolerance.cxx) | Vertices are kept when `tol >= valmax` where faces and edges test `tol <= valmax` | `tolerance shape [mode] min max` lists every vertex over the upper bound among the shapes "below" or "between", and none under it (each vertex once per edge that holds it). Pinned by `DrawShapeCommandsTests.Tolerance_and_maxtolerance` | leave |
| `nexplode` (DBRep.cxx) | Sizes its arrays as the number of distinct sub-shapes minus one, assuming the shape itself is in the map and not of the type | `nexplode` of an edge into edges (or a face into faces, a vertex into vertices) writes past the array: Standard_OutOfRange in a build with range checks | leave |
## Phase 3: unit 3I, BRepLib

| Where | What | Effect | Triage |
|---|---|---|---|
| `BRepLib_MakeEdge2d::Init` (the main one) | Never sets `myError = BRepLib_EdgeDone`, and the field is uninitialised in every constructor but the default one | A maker that never failed reports an undefined error. The port's field starts at EdgeDone. By reading | settled |
| `BRepLib_MakeEdge2d.cxx`, static `Project` | Returns true with `p` unset when Extrema_ExtPC2d is done but finds no extremum | The parameter is undefined in OCCT; the port gives 0. By reading | settled |
| `BRepLib_MakeWire::Add(const TopoDS_Edge&)` | Ends with `myError = BRepLib_WireDone` unconditionally | `BRepLib_NonManifoldWire` is never reported; only VF and VL, set to null, keep the information. Kept | touch |
| `BRepLib_BndBoxVertexSelector::SetCurrentVertex` (BRepLib_MakeWire_1.cxx) | Never resets `myVBox` | The box grows with every vertex: more boxes pass Reject, and only the running time suffers. Kept | leave |
| `BRepLib_MakeFace::Init(S, UMin, UMax, VMin, VMax, TolDegen)` | Checks the surface's natural range only when the parameters come reversed | Ordered parameters outside the natural range are accepted. Kept | touch |
| `BRepLib_MakeShell` | Adapts a null 3D curve (`IsDegenerated` on a GeomAdaptor_Curve) when BuildCurves3d failed on an edge | Throws in OCCT and in the port. By reading | touch |
| `BRepLib::UpdateEdgeTol` on an edge without a 3D curve | The first loop tests `an_iterator.More()` but reads `second_iterator.Value()` and advances neither | Only the first representation is looked at; the loop never ends if it isn't a curve on a surface, and `curve_on_surface_index` is left at 1 instead of 0. If the edge is also flagged SameParameter, `EvalMaxParametricDistance` measures against `geom_reference_curve`, never loaded: OCCT dereferences a null curve (the native binding throws), the port throws `NullReferenceException`. Reproduced in the oracle, which accepts both | touch |
| `BRepLib::OrientClosedSolid` on a solid without faces (not a bug, pinned) | `BRepClass3d_SClassifier::PerformInfinitePoint` classifies IN for a faceless solid | Returns true and reverses the solid. Pinned by `BRepLibStaticsTests.OrientClosedSolid_reverses_an_empty_solid` | leave |
| `BRepLib::ExtendFace`, a face on a bounded (B-spline, Bezier) surface | Calls `GeomLib::ExtendSurfByLength` on the face's own surface handle (`BRep_Tool::Surface(theF)`), which raises the degree and inserts knots in place | The face passed in changes, and so does every face sharing the surface. Reproduced: the native input's BRep text changed in 8 of 20 lofts. The port's GeomLib extends a copy, so the input is unchanged and the extended face is OCCT's | settled |
| `BRepLib::ExtendFace`, the general branch | Takes `BRep_Tool::Surface(theF)` without the face's location | A located face on a non-analytic surface loses its placement when extended (the analytic branch uses the transformed surface). Kept | settled (decision 1477) |
| `BRepLib::ExtendFace` | Tests `!Precision::IsInfinite(aSVMax)` for the VMin extension, where `aSVMin` is meant | VMin is extended when VMax is finite. Kept | fixed (decision 1478) |
| `BRepLib_PointCloudShape::NbPointsByDensity` | Divides each face's area by the argument `theDensity` instead of the computed `aDensity` | With the default density 0 the quotient is infinite, its conversion to int gives INT_MIN on x86, and every face gets one point. Kept, with x86's conversion emulated; pinned by `BRepLibMeshTests.PointCloudShape_with_the_computed_density_gives_one_point_per_face` | touch |
| `BRepLib::EnsureNormalConsistency` | Uses `BRep_Tool::PolygonOnTriangulation`'s result without a null check | An edge between two meshed faces without a polygon on one of their triangulations crashes OCCT; the port throws `NullReferenceException`. BRepMesh output always has the polygons. By reading | touch |
| `BRepLib_FuseEdges.cxx`, static `MakeClosedCurve` | When a fused closed B-spline edge must start at a vertex other than the curve's origin, makes the curve periodic, moves its origin and segments it in place | Without a transformation, that curve is the first old edge's own 3D curve (or its trimmed curve's basis): the input shape's edge geometry changes. The port works on a copy | settled |
| `BRepLib_FuseEdges::SameSupport`, B-splines | The knot check `(K1(k) - K2(k)) > tollin` is signed, and the multiplicities are compared as reals against a length tolerance | A knot of the first curve smaller than the second's by any amount passes. Kept | touch |
| `BRepLib_FuseEdges::BRepLib_FuseEdges` | The `PerformNow` argument is ignored (commented out) | Nothing is computed until an accessor asks. Kept | touch |
| `BRepLib_MakeFace(W)` with `BRepLib::UpdateTolerances` (not a bug, a consequence) | UpdateTolerances adds `2 * Epsilon(tol)` to every vertex tolerance it computes, and MakeFace runs it on the face holding the caller's wire | Making a face from a wire raises the tolerances of the caller's own vertices by two ulps, in place. The port keeps the caller's frozen vertices instead (decision 417) | settled |
## Phase 3: unit 3J, BRepCheck

| Where | What | Effect | Triage |
|---|---|---|---|
| `BRepCheck_Analyzer` (BRepCheck_Analyzer.hxx), its map of results | `NCollection_IndexedDataMap<TopoDS_Shape, handle<BRepCheck_Result>>` without a hasher: the default compares with `operator==` (IsEqual), so a shape met in two orientations (a seam edge, a shared vertex) has two results; the face task skips shapes already seen with IsSame, so only the first orientation's result gets the face as a context; `ValidSub` stops (`break`) at the result without that context | Whether a face (or edge) is valid depends on the order of its sub-shapes: a fault on an edge met after both occurrences of a seam isn't seen by `IsValid(face)`. Ported as is (decision 430); found by the oracle, checked natively over all 5,040 orderings of a face's wire | touch |
| `BRepCheck_Solid::Minimum` (BRepCheck_Solid.cxx) | After "too many growths" breaks out of the loop that calls `Init` on each closed shell's tool, the remaining tools aren't initialised, and the `IsOut` loop then uses their default point (-1, -1, -1) and dereferences their null classifier | When the second shell that isn't a hole comes before the last two closed shells, later shells are tested with a point that isn't on them, and the loop reaches an uninitialised tool and crashes (outside the analyzer's `try`). By reading | touch |
| `BRepCheck_Edge::InContext` | When the reference curve is a curve on a surface (no 3D curve) and the edge has no other pcurve on a planar face, down-casts the reference adaptor to GeomAdaptor_Curve and dereferences the null result | An access violation, caught by the analyzer under OCC_CATCH_SIGNALS as CheckFail. The port throws Standard_NullObject, also CheckFail (pinned by `BRepCheckAnalyzerTests.Edge_minimum_finds_bad_flags_ranges_and_curves`) | touch |
| `BRepCheck_Edge::InContext` in parallel mode | The edge's reference adaptor (`myHCurve`) is shared by the threads that check the edge in its different faces; a GeomAdaptor_Curve on a B-spline or Bezier curve fills an evaluation cache on each call | A data race in OCCT's parallel mode, seen natively: on `brep/bug29502_split_faces.brep` of the test dataset, 1 native parallel run in 40 under load reports a valid face (and its compounds) invalid; native serial and the port, serial and parallel (40 of 40), don't. The port evaluates a shallow copy per call in parallel mode (decision 431) | settled |
| `BRepCheck_Edge::Tolerance` | The surface of the second pcurve on a closed surface is placed with the representation's location only (`cr->Location()`), the first one's with the edge's location too | On a located edge with a seam, the second pcurve is measured in the wrong place, and the tolerance comes out too large. By reading | touch |
| `BRepCheck_Vertex::Tolerance` | A point on a curve whose curve is null leaves `Controlp` as the previous representation left it, and transforms it again | That point is counted at a wrong place. By reading | touch |
| `BRepCheck_Face::OrientationOfWires` | Searches the map of wires for each wire and reads the key without checking the iterator ("No control on More()"); with `Update`, adds `myOrires` after a fault (BRepCheck::Add appends NoError to a list holding another status) | Undefined if a wire isn't found (the port throws Standard_NoSuchObject); a face without surface checked by the analyzer has NoSurface then NoError (harmless: IsValid reads the first) | touch |
| `BRepCheck_Face.cxx`, `IsInside` | Evaluates the 3D curve at the pcurve's first parameter (`aCurve->D0(aFirst, ...)`) to measure the edge's length | Measures the wrong point on an edge whose pcurve and 3D curve aren't parametrised alike; harmless on same-parameter edges. By reading | leave |
| `BRepCheck_Wire::SelfIntersect` | Returns NoCurveOnSurface or InvalidRange for an edge after the first with no pcurve or an empty range, without recording it even with `Update` | The wire's context status stays NoError while the call returns a fault. Ported as is | touch |
| `BRepCheck_Wire::SetStatus` | Takes no lock, unlike Edge's and Face's | Nothing parallel calls it | leave |

## Phase 3: unit 3L2a, Draw-lite checks

Found while porting Draw's checking commands (Draw-lite reproduces them, so the corpus sees OCCT's behaviour).

| Where | What | Effect | Triage |
|---|---|---|---|
| `isbbinterf` (BRepTest_BasicCommands.cxx) | Checks `theNArg < 2`, then reads `theArgVal[2]` | `isbbinterf a` passes a null name to DBRep::Get. The port gives the usage message for fewer than two shapes. By reading | leave |
| `checksection` (BRepTest_CheckCommands.cxx) | `-r` reads `a[++anAI]` without checking that a value follows | `checksection s -r` calls Draw::Atoi on a null pointer. The port compares with 0. By reading | leave |
| `checkshape` structural dump (`StructuralDump`, BRepTest_CheckCommands.cxx) | Writes `"%s_v"` (and `_e`, `_w`, `_f`, `_s`, `_o`) with the user's prefix into `char aName[20]` | A prefix of 18 characters or more overflows the buffer. The port has no limit. By reading | leave |
| `checksection` (not a bug, a consequence) | Lists the lone vertices from a `NCollection_Map` hashed by TopTools_ShapeMapHasher, whose order follows the TShapes' addresses | The names `alone_1`, `alone_2`, ... aren't given in a reproducible order. The port names them in the order first met (decision 439) | leave |
| `computetolerance` | Prints `(void*)&(*S.TShape())`, and calls `S.ShapeType()` without checking for a null shape | The output holds addresses; a null shape throws Standard_NullObject (as in the port). The port prints a per-run number instead of an address (decision 440) | leave |

## Phase 3: unit 3K, distances and makers

| Where | What | Effect | Triage |
|---|---|---|---|
| `BRepExtrema_DistShapeShape::Perform`, the final filter ("Modified by Sergey KHROMOV") | Removes a solution whose distance exceeds `myDistRef + myEps` and then increments the index | The solution after a removed one is never checked. No input found that reaches it (every collected solution is within the deflection of the distance its task ended with). By reading | leave |
| `BRepExtrema_DistanceSS.cxx`, static `TRIM_INFINIT_FACE` | `double arrU[8], arrV[8]` are read for every corner of the edge's box, but written only when `GeomAPI_ProjectPointOnSurf` is done | A failed projection takes the bounds from uninitialised memory in OCCT; the port's arrays start at zero. By reading | settled |
| `BRepExtrema_DistShapeShape`, multi-threaded `SolidTreatment` (a property, not a bug) | Each task stops at its first vertex inside the solid and appends it under a mutex; the others stop only when they next read the done flag | Which vertices are returned as inner solutions, and how many, depends on the thread scheduling, in OCCT and in the port. The oracle checks those solutions as valid | leave |
| `BRepExtrema_DistShapeShape` (a property, not a bug) | The work is split into tasks by the default thread pool's size in single-threaded mode too, each task stopping early on its own best distance | The solutions, their order and (where several pairs tie within the deflection) their number can depend on the machine's processor count. The port uses the same count (`Environment.ProcessorCount`) | leave |
| `BRepExtrema_TriangleSet::Clear` (and so `Init`) | Clears the triangles and vertices but not `myShapeIdxOfVtxVec`, `myNumVtxInShapeVec`, `myNumTrgInShapeVec` | After a second `Init` (ShapeProximity's or SelfIntersection's reload) `GetShapeIDOfVtx`, `GetVtxIdxInShape` and the proximity statuses read the first set's entries. Pinned by `BRepExtremaTests.OcctIssue_TriangleSet_InitAgainKeepsTheFirstShapesVertexIds` | touch |
| `BRepExtrema_TriangleSet::Init` | `isOK` ends the loop at the first shape without a mesh, but the method returns true whenever the BVH exists | The set silently holds only the shapes before the first unmeshed one, and the tools report it as loaded. Pinned by `BRepExtremaTests.OcctIssue_TriangleSet_InitIsTrueWithoutMeshes` | touch |
| `BRepExtrema_OverlapTool::LoadTriangleSets` / `Perform` | The result maps `myOverlapSubShapes1/2` are never cleared | A ShapeProximity reloaded and performed again keeps the first run's pairs (and reports not done if the new sets don't overlap). Pinned by `BRepExtremaTests.OcctIssue_OverlapTool_KeepsOldPairs` | touch |
| `BRepExtrema_ShapeProximity::Perform` | Passes `myTolerance` by reference to `BRepExtrema_ProximityValueTool::Perform`, which writes the proximity value into it | After computing a proximity value, `Tolerance()` is that value, and the next Load and Perform run the overlap test with it instead of computing a proximity value. Pinned by `BRepExtremaTests.OcctIssue_ShapeProximity_ValueBecomesTheTolerance` | touch |
| `BRepExtrema_VertexInspector::Inspect` (BRepExtrema_ProximityValueTool.cxx) | Sets `myIsNeedAdd = true` at the start of every call | Only the last target inspected in the cells decides whether a bisection point is new: a coincident point found earlier is forgotten, and duplicates are added to the refinement points. Kept; by reading | touch |
| `BRepBuilderAPI_Copy::Perform` (and every `BRepBuilderAPI_ModifyShape` reused on a shape it modified before) | `NotDone()` is meant to force a new copy, but `BRepTools_Modifier::Init` doesn't clear its map, and `Rebuild` returns the images it already has; `CreateNewVertices` makes the vertices again | The second copy returns the first result, and `ModifiedShape`/`Modified` of a vertex then answer a new vertex that isn't in it. Draw's `tcopy a b` reuses one framework, so copying a shape twice in one command shares the result. Kept; pinned by `BRepBuilderAPITests.Copy_makes_new_TShapes_and_can_be_reused` and compared with native OCCT by `BRepBuilderAPIModificationsOracleTests.Copy_answers_as_native_OCCT` | touch |
| `BRepTools_Modifier::Rebuild` (face) with `BRepTools_NurbsConvertModification` | The NURBS conversion keeps a face on a B-spline or Bezier surface (`NewSurface` returns false) but copies its triangulation (`NewTriangulation` returns true), and the modifier then makes the face "with bare triangulation" | `BRepBuilderAPI_NurbsConvert` and `BRepBuilderAPI_GTransform` of a meshed shape give its B-spline and Bezier faces no surface (a meshed loft: six of its eight faces). Kept; pinned by `BRepBuilderAPITests.OcctIssue_NurbsConvert_of_a_meshed_BSpline_face_drops_its_surface`, and the same on native shapes in `BRepBuilderAPIModificationsOracleTests` | fixed (decision 1476) |

## Phase 3: unit 3M, primitives

| Where | What | Effect | Triage |
|---|---|---|---|
| `BRepSweep_Rotation::MakeEmptyGeneratingEdge` | Calls `BRep_Tool::Curve(E, L, f, l)->Copy()` before testing the curve for null | Revolving an edge without a 3D curve dereferences a null handle in OCCT; the port throws NullReferenceException there. Marked `// OCCT`; by reading | touch |
| `BRepPrim_GWedge::Vertex` | When the top is degenerated (Z2Min = Z2Max or X2Min = X2Max), the vertex built is copied to its partners in pairs, a partner not built yet as a null vertex, and both are marked built | A later request for that partner returns a null vertex instead of building it. The box and wedge makers' build order doesn't reach it (oracle). Ported as is; by reading | leave |
| `BRepPrim_Cone(Angle, Axes)` (the infinite cone on axes) | Leaves `myRadius` uninitialised | The lateral face's radius is garbage. No maker uses this constructor; the port's field is 0 | leave |
| `BRepPrimAPI_MakeRevolution`, static `Project` | Passes GeomProjLib::Curve2d's result on without checking it | A meridian whose projection fails gives a null 2D meridian, which BRepPrim_Revolution dereferences when the shape is built. Ported as is; by reading | touch |
| `BRepPrimAPI_MakeCylinder`, `MakeSphere`, `MakeTorus` (documentation) | Document Standard_DomainError for a radius or height not over Precision::Confusion() | Nothing checks them: a null or negative radius builds a shape (BRepPrim_Cone alone checks its parameters). Ported as is | touch |
| `BRepPrim_GWedge::Face`, `BRepPrimAPI_MakeBox::BottomFace` ... (a property, not a bug) | Return the face as built so far; building the other faces adds pcurves to its edges later, in place | A face asked for first changes after it was handed out. The port publishes the face once all faces are built (decision 467) | settled |
| `BRepPrimAPI_MakeRevol::Build`, the degenerated edges (a property) | Iterates an NCollection_DataMap of faces hashed by TShape address to decide which face keeps the original degenerated edge | Not reproducible from run to run; the port iterates in binding order (decision 470). No difference seen in the oracle | leave |
| `pcone`, `psphere`, `ptorus` (BRepTest_PrimitiveCommands.cxx) | The forms whose first number may be a plane read it with `DrawTrSurf::GetPlane` and use the handle without checking it | A word that isn't a plane dereferences a null handle in DRAWEXE; Draw-lite reports "DrawTrSurf::Get: not a plane". By reading | leave |
## Phase 3: unit 3L2b, Draw-lite construction and distances

Found while porting Draw's construction, distance and modification commands. Draw-lite reproduces the command bugs, so
the corpus sees OCCT's behaviour; all were found by reading unless a test is named.

| Where | What | Effect | Triage |
|---|---|---|---|
| `setorigin` (GeomliteTest_CurveCommands.cxx) | The `else { return 1; }` pairs with the 2D B-spline test | On a 3D B-spline curve the command sets the origin and then fails with an empty message. Kept; unit test | leave |
| `2dbeziercurve` (GeomliteTest_CurveCommands.cxx) | Counts the words per pole as `(n - 2) / np` where the 3D command uses `n - 3` | A curve of one pole without weight is read as weighted. Kept | leave |
| `extsurf`, `revsurf`, `trim` (GeomliteTest_SurfaceCommands.cxx), `gcarc ... cir` (GeometryTest_ConstraintCommands.cxx) | Read `a[5]`, `a[8]`, `a[8]` and `a[5]` without checking the count | With one word fewer they pass Tcl's null `argv[argc]` to Draw::Atof or DrawTrSurf::GetPoint. The port reads an empty word | leave |
| `gcarc` (GeometryTest_ConstraintCommands.cxx) | Returns 1 after binding the segment or arc, 0 after printing its usage | A successful gcarc is a Tcl error with an empty message: OCCT's scripts (samples/tcl/bottle.tcl) call it inside `catch`. Kept; unit test | leave |
| `pmirror`, `pscale`, `2dpmirror`, `2dpscale` (DrawTrSurf.cxx, `transform`, `d2transform`) | Don't take the point's coordinates off the list of names, as `lmirror`, `smirror`, `translate` and `rotate` do | The coordinates are looked up as names (harmlessly) and printed in the result (`pmirror p 0 0 0` returns `p 0 0 0 `). Kept; unit test | leave |
| `visoedge` (BRepTest_CurveCommands.cxx, `isoedge`) | `UMax = std::max(VMax, std::max(p1, p2))` | The trimmed surface's U range ends at the face's V maximum. Kept | leave |
| `vertex name P` (BRepTest_CurveCommands.cxx), `mksurface`, `pcurve edge face` (BRepTest_SurfaceCommands.cxx) | Dereference the down-cast point, the face's surface or the edge's pcurve without a test | A name that isn't a point, a face without surface or an edge without pcurve on the face is an access violation. The port throws Standard_NullObject | leave |
| `length` (GeomliteTest_CurveCommands.cxx) | `di << a[1] << "is not a curve\n"` | No space before "is". Kept | leave |
| `BRepLib::UpdateTolerances` in place (`InternalUpdateTolerances` with `IsMutableInput`, then `UpdShTol`) | A vertex TShape reached at several locations is a key per location in an NCollection_DataMap hashed by TopTools_ShapeMapHasher (the TShape's address); with `theVForceUpdate` each key's tolerance is written into the shared TShape | The vertex keeps the tolerance of the key iterated last, in an order set by addresses: the result isn't reproducible. The port iterates in insertion order (decision 18). data/occ's solid.brep, mal_tige.brep and MODERN_MVCooker_1.brep (`updatetolerance s 1`) differ from native OCCT only there (DrawLiteConstructionOracleTests); the reshaper form makes a copy per location and is reproducible | leave |
| `proximity`, `selfintersect` (BRepTest_ExtremaCommands.cxx; a property, not a bug) | List the overlapping sub-shapes by iterating an `NCollection_DataMap<int, ...>` | The order of the printed names is the map's bucket order. The port prints them in ascending order; the names are the same | leave |
| lowalgos/extss/bug29712_44 (a test, not the library) | Computes `extrema s1 s1` and `extrema s2 s2`, each plane with itself, and expects the distance 10 between the two parallel planes | The distance is 0, so the case fails in DRAWEXE as in Draw-lite (`extrema s1 s2` was meant). By reading | leave |
## Phase 4: unit 4L1, Draw-lite mesh readers

Found while porting Draw's mesh readers (tclean, trinfo, triepoints, trmergenodes, correctnormals, triarea, tricheck,
mpparallel). Draw-lite reproduces the command bugs, so the corpus sees OCCT's behaviour; by reading unless a test is
named.

| Where | What | Effect | Triage |
|---|---|---|---|
| `BRepLib::EnsureNormalConsistency` (`correctnormals`) | Computes the missing normals from `aPT->UVNode(i)` without checking that the triangulation has UV nodes | On a triangulation without UV nodes (Poly_MergeNodesTool's result, so after `trmergenodes`) OCCT reads past an empty array. The port throws Standard_OutOfRange, which Draw-lite reports as CommandCmd does. Marked `// OCCT` in the command; unit test | touch |
| `BRepLib::EnsureNormalConsistency` (already in 3I) | Uses a null polygon on triangulation | The oracle confirms it natively: OSD_Exception_ACCESS_VIOLATION on several data/occ and dataset files, where the port fails too (DrawLiteMeshOracleTests counts them as failing on both sides) | touch |
| `tricheck` (MeshTest_PluginCommands.cxx), the check of the meshes inside the faces | `if (aBoundaryEdgeMap.Size() == 0) break;` | The first face with no polygon on its triangulation (a face meshed without its edges, or not meshed) ends the loop: the faces after it are never checked for "Not connected mesh inside face". Kept; unit test | leave |
| `MeshTest_CheckTopology::Perform`, cross-face errors | Records `i1` and `i2`, the indices in the edge's polygons, where tricheck prints them as "node1" and "node2" | The numbers printed aren't node numbers. Kept | leave |
| `tricheck -small` | Takes `const gp_Trsf& aTrsf = aLoc.Transformation()` before `BRep_Tool::Triangulation(aFace, aLoc)` fills `aLoc` | The null-area triangles are drawn untransformed. Display only; Draw-lite draws nothing | leave |
| `trmergenodes` without `-oneFace` (MeshTest.cxx) | Replaces each face's triangulation (`UpdateFace(F, T, false)`) and leaves the edges' polygons on the former triangulation | The edges keep polygons on triangulations no face carries: `tricheck` finds no boundary on the face and stops (above), `tclean` doesn't remove those polygons (it walks the faces' triangulations). Kept; unit test | leave |
| `correctnormals` (MeshTest.cxx) | Reads `argv[1]` without checking the count | Without an argument DBRep::Get gets Tcl's null `argv[argc]`. The port reads an empty word: no shape, "Normals have not been changed!" | leave |
| `triepoints` (MeshTest.cxx; a property, not a bug) | Numbers the edges by iterating an `NCollection_Map` of shapes, hashed by TShape address | The edge numbers in the vertex names aren't reproducible. The port numbers the edges in TopExp_Explorer's order | leave |

## Phase 4: unit 4S, ShapeAnalysis

Found while porting ShapeExtend, ShapeAnalysis's wire analysis and ShapeBuild. Reproduced in the port, each marked
`// OCCT`.

| Where | What | Effect | Triage |
|---|---|---|---|
| `ShapeAnalysis_Curve::ValidateRange` (ShapeAnalysis_Curve.cxx) | Tests `IsPeriodic` on the basis curve (through trimmed and offset curves), but passes the curve's own `[cf, cl]` to `ElCLib::AdjustPeriodic` as the period | On a trimmed arc of a circle, the trimmed range is taken as the period: the reversed range (2, 1) on the arc [0, 3] becomes (2, 4), past the arc's end. By reading; pinned by `ValidateRange_on_a_trimmed_arc_wraps_with_the_trimmed_range_ReproducesOcctError`; the oracle matches native on trimmed circles | touch |
| `ShapeAnalysis_Edge::CheckOverlapping` through the static `IsOverlapPartEdges` (ShapeAnalysis_Edge.cxx) | The sampling step is half the shorter edge's length | With a degenerated (zero-length) edge whose vertex is within tolerance of the other edge, the step is 0 and the loop never ends. Found when the first oracle run hung; the oracle skips degenerated edges. Not pinned (it would hang) | fixed (decision 1431) |
| `ShapeAnalysis_Edge::CheckOverlapping` | Doesn't reset the status; in the domain branch, the `aStartLength < 0` case sets `aEndLength`, which the next line overwrites | Status flags add up across calls. By reading | touch |
| `ShapeAnalysis_Edge::CheckVerticesWithCurve3d` and `CheckVerticesWithPCurve` | OR the enumerator `ShapeExtend_DONE1` into the status, not its encoding | Harmless: equal for DONE1 and DONE2. By reading | leave |
| `ShapeExtend_WireData::Add(const Handle(ShapeExtend_WireData)&, int)` | In non-manifold mode adds `wire->Edge(i)` where it means `wire->NonmanifoldEdge(i)` | The other wire's INTERNAL/EXTERNAL edges are lost and ordinary edges added twice. Pinned by `Add_wire_data_in_non_manifold_mode_adds_the_wrong_edges` | touch |
| `ShapeAnalysis_WireOrder::SetChains` | Also checks the wrap-around from the last edge to the start of its own chain | On an open chain the last chain's start is appended twice: 3 chains, the second the empty range (3, 2). Pinned by `Chains_of_an_open_and_a_closed_wire` | touch |
| `ShapeAnalysis_WireOrder::Gap(0)`, `SetCouples` | `myGap` is never computed, and SetCouples is empty | `Gap()` is always 0 and `NbCouples()` always 0. Pinned | touch |
| `ShapeAnalysis_TransferParameters` constructors | Leave `myMaxTolerance` uninitialised | Undefined until `SetMaxTolerance`; the port starts it at 0 | leave |
| `ShapeAnalysis_Wire::CheckLoop` (ShapeAnalysis_Wire.cxx) | Sets the analyser's precision to `Precision::Infinite()` and, on the FAIL2 return for an edge without vertices, doesn't restore it | Later checks on the same analyser run with an infinite precision and pass everything. By reading | touch |
| `ShapeAnalysis::TotCross2D` | With no pcurve of the wire on the face, the points stay at their defaults | The result is 0, so `IsOuterBound` counts such a wire as outer. By reading | touch |

## Phase 4: unit 4A, mesh foundations

Found while porting Message, NCollection_MapAlgo, IMeshData, IMeshTools, BRepMeshData and BRepMesh's geometry tools.
All are reproduced and pinned by unit tests.

| Where | What | Effect | Triage |
|---|---|---|---|
| `BRepMesh_GeomTool::IntSegSeg` (BRepMesh_GeomTool.cxx) | `aPointHash[0]`/`[1]` classify the second segment's start/end on the first segment, `[2]`/`[3]` the first's on the second; the `PointOnSegment` branch returns the first segment's start for `[0]`, its end for `[1]`, the second's start for `[2]` and its end for `[3]` | The point given with `PointOnSegment` is an end point of the wrong segment. BRepMesh_Delaun, its only caller asking for it, recomputes the chopping point and ignores it, so meshes aren't affected. Kept (`// OCCT`); BRepMeshGeomToolTests | leave |
| `Message_Algorithm::AddStatus` (Message_Algorithm.cxx) | Copies the other algorithm's strings with `for (n = 1; n < Length; n++)` | The last string of each status is dropped. Kept; unit test | touch |
| `Message_Msg::Get` (Message_Msg.cxx) | Replaces the unfilled placeholders but keeps their positions | A second `Get` garbles the text ("a %s b" → "a UNKNOWN b" → "a UNKNOWNKNOWN b"); `SendStatusMessages` calls `Get` on a stored custom message, so sending it twice garbles it. Kept; unit test | touch |
| `Message_Msg::Set` (Message_Msg.cxx) | After "%%", reads the next characters as a format | "100%% done" gets a placeholder "% d": `Arg(7)` gives "100 7one". Kept; unit test | touch |
| `NCollection_MapAlgo::Differ` (NCollection_MapAlgo.hxx; a property, not a bug) | Reports a change by comparing the counts before and after | Swapping one key for another returns false. Kept; unit test | leave |

## Phase 4: unit 4C, model and edges

Found while porting BRepMesh's model building, edge discretisation, healer and pre- and post-processors. Reproduced in
the port, each marked `// OCCT`.

| Where | What | Effect | Triage |
|---|---|---|---|
| `BRepMesh_FaceChecker`'s selector, `Accept` (BRepMesh_FaceChecker.cxx) | When a wire is checked against itself, a crossing is kept only if the loop it closes has an area; the area sums the cross products of each segment's second point, in the order of each pcurve's parameter, not the wire's | Along reversed edges the second points are the wire's first ones: a bow tie bounded by reversed edges measures zero area, its crossing is taken for a tangency, and the face is meshed with a self-intersecting wire (the same bow tie with forward edges fails with SelfIntersectingWire). Pinned by `Face_checker_misses_a_crossing_of_reversed_edges_as_OCCT_does` | touch |
| `BRepMesh_ModelPostProcessor`'s PolygonCommitter, `commitPolygons` (BRepMesh_ModelPostProcessor.cxx) | Stores a seam's first discrete pcurve on the face as the polygon of the forward edge and the last as the reversed edge's, whatever their orientations | The model's pcurves follow the wire order of ShapeAnalysis_Wire::CheckOrder. On BRepPrimAPI's cylinder (top, seam reversed at u = 0, bottom, seam forward at u = 2pi) `BRep_Tool::PolygonOnTriangulation` of the forward seam gives the nodes at u = 0, the reversed seam's side. Same 3D points; the UV side is swapped. Pinned by `Seam_polygons_are_stored_in_the_model_order_as_OCCT_does`; the oracle compares native polygons in this order | touch |
## Phase 4: unit 4B, Delaunay core

Found while porting the Delaunay core and Delabella. Reproduced in the port, each marked `// OCCT`. Of the
BRepMesh_Delaun and DataStructureOfDelaun entries, only the first is on the mesher's path, where it sizes the
circles' cells.

| Where | What | Effect | Triage |
|---|---|---|---|
| `CDelaBella::Triangulate` (delabella.cpp), all points cocircular | With more than 3 points, all cocircular ("[NFO] all input points are cocircular."), the last point is retracted and made the cone's top, which is coplanar with the base; no point is left for the main loop, and the final loop over `2 * points - 4` faces reads the adjacency (`f[0..2]`) of faces this run never allocated | Native (MSVC, release) crashes with an access violation, confirmed on the unit square's corners and on the twelve integer points of the circle of radius 5. The port throws NullReferenceException. Pinned by `Exactly_cocircular_points_fail_as_in_OCCT_ReproducesOcctError`. BRepMesh_DelabellaBaseMeshAlgo adds the four corners of an enlarged box to the face's nodes, so it can't reach this with distinct nodes | touch |
| `CDelaBella::Triangulate`, 0 points | The duplicate removal starts with `w = 0, r = 1` and ends with `w = 1`: it reports "detected -1 dups" and sets `points = 1`, then the single-point branch writes `vert_alloc[0]` of an empty allocation (null on a first call) | Undefined behaviour natively; the port throws NullReferenceException. Pinned by `No_points_fail_as_in_OCCT_ReproducesOcctError` | touch |
| `CDelaBella::Triangulate`, `assert(0)` and `assert(_f != face_alloc)` | Release builds compile the asserts out: a coplanar point that is on neither side of the contour is taken as `last` without being linked in; the emergency search for a visible face runs below `face_alloc` when none is visible | Undefined behaviour natively (the port throws IndexOutOfRangeException in the second case). By reading; not reached by any test | leave |
| `IDelaBella::Triangulate` (a property, not a bug) | The header says triangles are returned clockwise "in abc"; the Delaunay triangles (lifted faces with `n.z < 0`) are clockwise in `v[0..2]`, and the hull contour counter-clockwise | BRepMesh_DelabellaBaseMeshAlgo takes `v[0], v[2], v[1]` to get counter-clockwise triangles. Tested | leave |
| `BRepMesh_Delaun::initCirclesTool` (BRepMesh_Delaun.cxx) | `if (NbNodes() > 100) aScaler = 5; else if (NbNodes() > 1000) aScaler = 7;` | The second branch is unreachable: the circles' cells are at most a fifth of the box for any number of nodes above 100. Kept; by reading | leave |
| `BRepMesh_DataStructureOfDelaun::SubstituteElement` and `clearDeletedLinks` | `myElements(theIndex) = ...` on an `NCollection_DynamicArray` indexed from 0, where `GetElement(theIndex)` reads `theIndex - 1` | The triangle after the one meant is replaced; past the last triangle the write goes into unused capacity (or, at a block boundary, a missing block). The port replaces the next triangle and drops a write past the end. The mesher calls neither. Pinned by `SubstituteElement_writes_one_triangle_too_far_ReproducesOcctError` | touch |
| `BRepMesh_VertexTool::RemoveLast`, through `BRepMesh_DataStructureOfDelaun::clearDeletedNodes` | `RemoveLast` is `DeleteVertex(Extent())`, which marks the vertex deleted but doesn't shrink the vector | `clearDeletedNodes` calls it once per moved node: from the second on it deletes the same last vertex again instead of the current last one, which stays live as a duplicate of the node copied from it, and `NbNodes()` never decreases. The mesher doesn't call `ClearDeleted`. Pinned by `ClearDeleted_renumbers_links_and_nodes_ReproducesOcctError` | touch |
| `BRepMesh_Triangulator::prepareMeshStructure` | Maps the triangles' nodes back to the wires' points with `myTmpMap.Bind(aNumNode + 1, ...)`, while the triangles use the indices `AddNode` returns, and `AddNode` merges points within `Precision::Confusion()` | With two coincident wire points the indices shift: triangles get the wrong points, or `Find` throws and `Perform` returns false. By reading | touch |
| `NCollection_Sequence::Remove(from, to)` in `BRepMesh_Delaun::meshPolygon` (a property of release builds) | When the crossing link's start is the closer end and the two frontier links are consecutive, the range is `(aPolyIt + 1, aPolyIt)` | OCCT's range check is compiled out in release builds and the call removes nothing. The port's `Remove` checks, so `meshPolygon` skips that call (`removeRange`). By reading | settled |
| `BRepMesh_CircleInspector::Inspect` (OCCT's own comment) | Compares `d² - r²` with the square of the linear tolerance | The circles are effectively exact. OCCT keeps it because the correct test hangs "perf mesh bug27119". Kept | leave |
| `BRepMesh_Delaun` with free nodes only (a property, not a bug) | `frontierAdjust`'s `cleanupMesh` removes every triangle with a free link whose nodes no frontier holds, until none is left to remove | Triangulating points without a frontier keeps only an inner part of their Delaunay triangulation (nothing for three points). The mesher always has a frontier. Tested | leave |
| `BRepMesh_Delaun::createTrianglesOnNewVertices` (a property, not a bug) | `Contains` reports a non-free link when the node is within a fifth of its length, and a free node there is skipped | A free node closer to a frontier link than a fifth of the link's length is never inserted. Tested | leave |

## Phase 4: unit 4D, face meshing and the incremental mesh

Found while porting the face algorithms, range splitters and factories. Reproduced in the port, each marked `// OCCT`.
Native BRepMesh_IncrementalMesh (Watson) itself showed no bug in the oracle: every compared face is bit-identical.

| Where | What | Effect | Triage |
|---|---|---|---|
| `BRepMesh_DelabellaMeshAlgoFactory::GetAlgo`, `default:` (BRepMesh_DelabellaMeshAlgoFactory.cxx) | Surfaces of extrusion, offset and other surfaces fall to the default branch, which uses `BRepMesh_NURBSRangeSplitter` (the Watson factory gives extrusions BRepMesh_ExtrusionRangeSplitter). Its `getUndefinedIntervalNb` asks the surface for `NbVPoles()`, and `GeomAdaptor_Surface::NbVPoles` throws Standard_NoSuchObject for a surface of extrusion | With `-algo 1` (Delabella) a face on a surface of extrusion is left without a mesh: BRepMesh_BaseMeshAlgo's `catch (Standard_Failure)` swallows the exception without setting a status, so GetStatusFlags() is 0. By reading OCCT's code; the port does the same. Pinned by `Delabella_leaves_an_extrusion_unmeshed_as_OCCT_does_ReproducesOcctError` | touch |
| `BRepMesh_BaseMeshAlgo::Perform` (a property) | A failure caught while a face is meshed leaves the face without a triangulation and without a status; only FaceDiscret's own catch sets IMeshData_Failure | A face can be silently unmeshed (see the row above). Kept | touch |

## Phase 4: unit 4L2, incmesh and the corpus

Found while porting incmesh, tessellate, the mesh factory's commands and AIS's meshing. By reading unless a test is
named; Draw-lite keeps OCCT's behaviour. The oracle found no difference from native incmesh or AIS meshing.

| Where | What | Effect | Triage |
|---|---|---|---|
| `incmesh` (MeshTest.cxx), argument parsing | Any word that parses fully as a real (`IsRealValue(true)`) is taken as the deflection, before shape names are looked up | A shape named like a number can't be meshed by name. Kept | leave |
| `incmesh` | Returns 0 whatever the statuses (Failure, SelfIntersectingWire, ...) | A failed mesh is only visible in the text; the mesh group's TODOs match it there. Kept | leave |
| `incmesh -prs` with a deflection | The deflection given becomes the drawer's deviation coefficient, so `incmesh s 1 -prs` meshes at 4 times the shape's size | The help text calls it "0.001 of bounding box linear deflection"; the value is taken as written. Kept; unit test | leave |
| `StdPrs_ToolTriangulatedShape::GetDeflection` (a property) | Stores the absolute deflection it computes into the object's drawer as its own maximal chordial deviation | An object displayed with a relative deflection keeps that absolute value: a later `vdefaults -absDefl` doesn't reach it. Kept (Prs3d_Drawer is a working object) | leave |
| mesh group, standard_shading/M4 (a test reference) | The script expects an area difference of 1.9456% when meshed for shading, under `TODO ?OCC22687` | The port's AIS mesh, bit-identical to native's by the oracle, gives 1.918%: the optional TODO makes the case BAD, not FAILED. The reference may come from another OCCT version's shading deflection | leave |
| v3d/bugs/bug29020 (a test of the viewer) | Reads the maximal texture size from `vglinfo` | Can't run without OpenGL; FAILED in Draw-lite, which ignores vglinfo | leave |
## Phase 4: unit 4W, viewer extraction, benchmarks and WebAssembly

No OCCT bug found. The extraction calls BRep_Tool, BRepLib_ToolTriangulatedShape::ComputeNormals,
BRepAdaptor_Surface/Curve and GCPnts_TangentialDeflection as build123d.NET's native shim does, and its buffers are
float-identical to native's on every compared face and edge.

| Where | What | Effect | Triage |
|---|---|---|---|
| build123d.NET's 3d_mesh_extract (not OCCT) | Calls BRepLib_ToolTriangulatedShape::ComputeNormals on the face's attached triangulation, which writes the normals into the meshed shape | Harmless there (the shape is build123d.NET's private meshed copy). The port computes them on an unattached copy (decision 569), with the same values | leave |
| BRepLib_ToolTriangulatedShape::ComputeNormals (a property) | Normals are stored in single precision (Poly_Triangulation::SetNormal), then read back as gp_Dir (normalised again in double) before the location's rotation | The viewer's normals go through float twice. Kept: the port must give native's floats | leave |

## Phase 5: unit 5E, IntPolyh

All reproduced. The ones that change results are parity-sensitive for units 5D and 5G (IntPatch_PrmPrmIntersection
starts its walking lines from IntPolyh's section lines and tangent zones).

| Where | What | Effect | Triage |
|---|---|---|---|
| `IntPolyh_Intersection::PerformMaillage` (standard), enlarged-zone retry | When no couple is found, the retry refills the points on the enlarged sampling and recomputes everything on the same `IntPolyh_MaillageAffinage`. `IntPolyh_Array::Init` keeps the items already there, and `FillArrayOfTriangles` only ever clears the intersection-possible flag | The first pass's triangles keep their flags (out of the common box, split, non-interfering), their cached bounding boxes (computed from the unenlarged points), deflections and edges; triangles beyond the first pass's count are reused with stale state when refining. A first pass that rejected everything (disjoint boxes, the usual reason for no couple) makes the retry find nothing; a fresh object on the enlarged sampling would. Reproduced; test `BSplinesBeyondBounds_EnlargedRetry_KeepsTheFirstPassTriangles` | touch |
| `IntPolyh_MaillageAffinage::FillArrayOfPnt(SurfID, isShiftFwd, Upars, Vpars, theDeflTol)` | Calls the point-normal overload with `1` instead of `SurfID` | Always fills the first surface's points. IntPolyh_Intersection doesn't call it (it uses the point-normal overload directly); `FillArrayOfPnt(SurfID, isShiftFwd)` does. Test `FillArrayOfPnt_ShiftedWithSampling_FillsTheFirstSurface` | touch |
| `IsDegenerated` (IntPolyh_MaillageAffinage.cxx) | Samples the isoline at `i * dU` (and `i * dV`), not `aU1 + i * dU` | The middle sample is off the surface's range unless it starts at 0, so a degenerated bound isoline can be missed (its points then aren't flagged, and its triangles get a deflection instead of being degenerated) | touch |
| `CalculPtsInterTriEdgeCoplanaires`, the edge not parallel to the side, second point, `TriSurfID == 2` | Takes the side's orientation from `Tri1` where the first point's branch reads `Tri2` | The second start point's `Lambda2` may be `1 - alpha` instead of `alpha` | touch |
| `CalculPtsInterTriEdgeCoplanaires`, the edge parallel to the side | For `TriSurfID == 2`, `SetUV1(PEP.U(), PTP.V())` and `SetUV2(PTP.U(), PEP.V())` mix the two points' U and V; one branch builds `PEP2` from `PE2 + Edge * lambda2` where the others start from `PE1`; the second point takes `alpha1` and `lambda1`, not `alpha2` and `lambda2`, and for `TriSurfID == 2` the orientations from the other triangles | Coplanar overlapping edges give start points with wrong parameters and positions on the edges (the GTest's sphere/plane and sphere/cylinder cases only check that they are finite) | touch |
| `IntPolyh_MaillageAffinage::TriangleEdgeContact`, coplanar case | One of the duplicate checks compares `SP1.V2()` with `SP2.V1()` | Two distinct points can be merged, or a duplicate kept | touch |
| `IntPolyh_MaillageAffinage::TriangleEdgeContact`, `TriSurfID == 2`, on the second side (alpha 1) | Sets `Lambda2` from `alpha`, where the first surface's branch uses `beta` | The start point's position on the edge is 1 (or 0) whatever beta is | touch |
| `IntPolyh_MaillageAffinage::TriContact` | Leaves the angle unchanged when a triangle's normal vanishes; TriangleCompare passes one variable for all couples | Such a couple gets the previous couple's angle (or -2), which feeds `IsAdvRequired`, `AnalyzeIntersection` and the start points' incidence | touch |
| `IntPolyh_StartPoint::CheckSameSP` | Prints both points and "Can't Check" to standard output when both edges are unknown; TriangleEdgeContact prints "Cote PT2PT3 nul1" | Debug output in a release library. Left out of the port (decision 596) | leave |
| `IntPolyh_MaillageAffinage::CommonBox(B1, B2, ...)` | Ignores its box arguments and uses its own | Harmless (its only caller passes its own boxes) | leave |
| `IntPolyh_MaillageAffinage::GetFinTE`, `GetFinTT` | Declared, never defined | A link error for a caller; left out (decision 597) | leave |
## Phase 6: unit 6S, Sewing

Found while porting BRepBuilderAPI_Sewing and FastSewing. Reproduced in the port, each marked `// OCCT`.

| Where | What | Effect | Triage |
|---|---|---|---|
| `BRepBuilderAPI_Sewing::Perform` → `EdgeRegularity` (BRepBuilderAPI_Sewing.cxx) | EdgeRegularity ends with `myMergedEdges.Clear()`, and CreateOutputInformations, called after it, binds a section to its boundary only if the section is in `myMergedEdges` | With sewing on (the default), `IsSectionBound` is always false and `SectionToBoundary` always throws; they work only in analysis mode (`option1 = false`). Pinned by `A_bound_is_cut_at_a_T_junction_and_its_sections_know_their_boundary`; the oracle compares the number of bound sections with native's | touch |
| `BRepBuilderAPI_Sewing::SameParameterEdge` (the two-edge form) | When the second edge has only the reversed pcurve of a seam on a face (`c2d2` null, `c2d21` not), dereferences the null `c2d2` | A crash in native OCCT; a NullReferenceException in the port. By reading | touch |
| `FreeEdge`, `MultipleEdge`, `ContigousEdge`, `ContigousEdgeCouple`, `DeletedFace`, `DegeneratedShape` | The range checks accept index 0 (`index < 0 \|\| index > Nb...`) | Index 0 then fails in the map (Standard_OutOfRange) instead of the method's own check. Kept | touch |
| `BRepBuilderAPI_Sewing::EdgeRegularity` | Iterates `myMergedEdges` (a map keyed by shapes) in address order | Harmless: each edge's regularity is encoded on its own. The port iterates in insertion order | leave |
| `BRepBuilderAPI_Sewing::Dump` | Writes to `std::cout` only | The port adds `Dump(TextWriter)` | leave |
| `BRepBuilderAPI_Sewing` (native runs, found by the oracle) | The sewn shape's sub-shapes come out in an order that can change from run to run of the same sewing in one process (address-ordered maps; plan 2.4) | tests/sewing tol_0_01/Q5 and CNP001_projoiei.rle at 1e-6 gave two native orders over seven runs; the port's order is one of them, the same every run. The oracle compares with every native variant | leave |
| `BRepBuilderAPI_Sewing` (in place, found by the oracle) | Sews its inputs in place: in a few cases (tests/sewing G3, G4 at 0.01, Y7 at 100; CCH_propag.rle, CCH_r2.rle at 1e-6) it writes into input edges or vertices (SameParameterShape runs BRepLib::SameParameter on every edge of the result, the inputs' own edges included), and then answers that they aren't modified (`IsModified`, `IsModifiedSubShape`) | The port never writes a frozen input: it sews a copy (decision 600) and answers that those inputs are modified, giving the changed copy. Expected; the oracle detects the native write and compares everything else | settled |
## Phase 5: unit 5A, intersection foundations

Found while porting IntSurf, IntImp, IntStart and IntPatch's data model and tools. By reading OCCT's code unless a
test is named; the port reproduces each (`// OCCT` at the line). No class of this unit is bound natively, so none
is confirmed by the oracle yet (5G).

| Where | What | Effect | Triage |
|---|---|---|---|
| `IntPatch_GLine::ComputeVertexParameters`, the third filter (points on S2 only) | Tests `VTXi.IsVertexOnS1()` where `IsVertexOnS2` is meant; always false for these points | Of two coincident points on S2, point i is always the one removed, even when it is the vertex. Kept | touch |
| `IntPatch_GLine::Replace` | Appends the point and ignores Index (the replacement is commented out) | A "replaced" vertex is added at the end and the old one stays. Kept; pinned by `IntPatch_GLineTests` | touch |
| `IntPatch_GLine::ComputeVertexParameters` (a property) | Sets `indf = 1` and `indl = nbvtx` without setting `fipt`/`lapt` | HasFirstPoint/HasLastPoint stay as they were. Kept; pinned | touch |
| `IntPatch_ALine::ComputeVertexParameters`, the periodic-line block | The `Prepend` branch is inside `VTX0 == ParamMinOnLine` but requires `VTX0 != ParamMinOnLine` (and measures from VTX0 where VTXN seems meant) | A closed line whose last vertex is at its end never gets the matching vertex at its start. Kept | touch |
| `IntPatch_WLine::ComputeVertexParameters`, the insertion of a vertex before the line | The loop that shifts the other vertices tests `j != 1` where `j != i` is meant | Vertex 1 keeps parameter 1 while its point moves to index 2. Kept; pinned by `ComputeVertexParameters_reproduces_OCCTs_unshifted_vertex_before_the_line` | touch |
| `IntPatch_WLine::ComputeVertexParameters`, the seam phase | The S2 creation block runs even after the S1 block has appended a vertex; its search loop is skipped and its else branch resets `encoreunefois` | The loops continue past the new vertex. Kept | touch |
| `IntPatch_WLine::Dump`, `IntPatch_RLine::Dump`, mode 0 | Print `Vertex(pol)` instead of `Point(pol)`, and only while `pol <= NbVertex` | The dump shows vertices where points are announced. Kept | leave |
| `IntPatch_RstInt.cxx`, `Recadre`, torus case | The V2 loop compares `V2 < V1p - 1.5π` where V2p is meant | A torus's V2 can be left a period off. Kept | touch |
| `IntPatch_PolyArc::SetOffset` | `Get` returns the bounds with the gap, and the box is reset to the unshifted bounds before the shifted ones are added | The box covers both positions and grows by the gap on each call. Kept | touch |
| `IntPatch_HInterTool::SamplePoint`, perturbation 1 | `dv *= 1.001; dv *= 0.999;` scales dv twice (du was presumably meant first) | The sample is moved by a factor 0.999999 in v only. Kept; pinned by `IntPatch_HInterToolTests` | touch |
| `IntStart_SearchOnBoundaries.gxx`, `BoundedArc` (a site, decision 610) | `std::sort` of the roots (SolInfo) by parameter: ties keep their order with MSVC on up to 32 roots (insertion sort) | The port sorts as MSVC on up to 32; with more roots, tied parameters could come out in another order than native's | leave |
| `IntImp_ZerImpFunc`, `IntImp_ZerParFunc`, `IntPatch_CSFunction` (a property, decision 608) | Keep `void*` pointers to the caller's handles (and quadric) | A later reassignment of the caller's handle variable would change the function's surface in OCCT; the port keeps the object. No caller does it | leave |
| `IntSurf_Transition::SetValue()` and `SetValue(Tangent, Type)` (a property) | Leave the other fields (situation, opposite flag, tangency) as they were | Two in transitions can differ in fields their queries don't show; MakeTransition passes the caller's objects (`ref` in the port). Kept | leave |
## Phase 6: unit 6A, BOP data and reports

Each bug is reproduced by a test and marked `// OCCT` at the line, and ported as OCCT has it unless the entry says
otherwise.

| Where | What | Effect | Test | Triage |
|---|---|---|---|---|
| Message_Report::AddAlert | When `Message_CompositeAlerts::AddAlert` merges the new alert into one of the same type, it returns false. The report then tests `anAlerts.Extent() > myLimit`, which is always true with the default limit of -1, and removes the first alert of that gravity. The limit is never applied to an appended alert | Simple alerts merge by default, so adding the same simple alert twice with the same gravity leaves none; a third add appends it again. In BOPAlgo, `AddError(new BOPAlgo_AlertTooFewArguments)` twice gives `HasErrors() == false`, and merging two reports that hold the same simple alert type loses alerts. If another alert is first in the list, that one is removed instead | `AddAlert_MergingRemovesTheFirstAlert_OcctIssue`, `AddAlert_LimitAppliesOnlyOnMerge_OcctIssue`, `SimpleErrorAddedTwice_OcctIssue` | touch |
| Message_CompositeAlerts::Clear(type) | The loop's else branch calls `anIt.More()` instead of `anIt.Next()` | An alert not of the cleared type makes the loop run forever, so `Message_Report::Clear(type)` hangs. **Fixed minimally** (`Next()`), because a hang can't be shipped | `Clear_AllGravityAndType` | settled |
| IntTools_Curve::D0 | The bounds test is `thePar >= First \|\| thePar <= Last`, where `&&` is meant | Every parameter counts as inside the curve and is evaluated | `Curve_D0_OcctBug_EvaluatesOutsideTheBounds` | touch |
| IntTools_CommonPrt copy constructor | Sets `myAllNullFlag(false)` instead of copying it (`Assign` does copy it) | A copied common part loses its all-null flag | `CommonPrt_OcctBug_CopyConstructorDropsTheFlag` | touch |
| IntTools_CommonPrt::Assign | Appends the other part's Ranges2 to its own instead of replacing them | `a = b` accumulates ranges. BOPDS_InterfEE/EF::SetCommonPart goes through it, so setting a common part twice accumulates the second edge's ranges | `CommonPrt_OcctBug_AssignAppendsTheRanges`, `InterfEE_SetCommonPartTwice_AccumulatesTheSecondEdgeRanges` | touch |
| IntTools::RemoveIdenticalRoots | After `aSR.Remove(k)` the loop goes on at k+1, so the root moved into slot k is never compared with root j | With roots 1.0, 1.0001 and 1.0002 and eps 1, the result is 1.0 and 1.0002, though 1.0002 is within eps of 1.0 | `RemoveIdenticalRoots_OcctBug_SkipsTheRootAfterARemoval` | touch |
| BOPDS_FaceInfo::Clear | Clears the In and On maps but not the Sc maps | A cleared face info keeps its section pave blocks and vertices | a BOPDS_DSTests case | touch |
| BOPDS_DS::Paves | Sizes its array as the number of pave blocks plus one. If fewer distinct paves are found, the default paves left in the array (index -1, parameter 99) are sorted and appended too; OCCT's assert only fires in debug builds | Spurious paves in the list for an edge whose pave blocks share paves | — (kept; reached by 6E) | touch |
| BOPDS_DS::BuildBndBoxSolid | Called after `Init` with one argument, it returns a void box: `prepareSolids` has already replaced the solid's shells by faces, edges and vertices, so no shell is found | A void box for such a solid | a BOPDS_DSTests case | touch |
| BOPAlgo_Builder::SetArguments (stub, OCCT's code) | Clears `myArguments` but not the fence map `myMapFence` | Setting again arguments that were added before leaves none | `BuilderSetArguments_Twice_KeepsNothing_OcctIssue` | touch |
| BOPAlgo_ToolsProvider::SetTools | The same, with `myMapTools` | As above, for tools | — (ported, not yet marked or tested) | touch |

**Order sites (decisions 580 and 633).** Native's NCollection_Map iterates in hash-bucket order. For integer and
pair keys that order is deterministic but scrambled; the port iterated in insertion order. Unit 6E's measurements
showed it decides the pave filler's results, and BOPDS's integer maps now iterate in OCCT's order (decision 784). It
decided:

- `BOPDS_DS::prepareFaces`/`prepareSolids`: the order of each face's and solid's sub-shape list, and through it
  FaceInfoOn, SharedEdges and AloneVertices;
- `UpdateFaceInfoIn/On(NCollection_Map<int>)`;
- `UpdateCommonBlock`: the order in which the new common blocks are made (a `DataMap<BOPDS_Pair, …>`);
- `IntTools_CurveRangeLocalizeData`/`SurfaceRangeLocalizeData::ListRangeOut` (hash order of samples).

In BOPDS, no map keyed by pave block's address is iterated: FaceInfo's In/On/Sc maps are indexed maps (insertion
order in native too), and `myMapPBCB` is only looked up.

Not bugs, but worth knowing:

- IntTools_ShrunkRange never resets its bounding box. `SetData`, `Perform` and `SetShrunkRange` only add to it, so an
  object reused for a second edge keeps the first edge's box. OCCT's callers make one object per pave block.
- The BOPAlgo message file's key `BOPAlgo_AlertBOPIsNotSet` doesn't match the class `BOPAlgo_AlertBOPNotSet`, and
  `BOPAlgo_AlertNullFace` has no class.

## Phase 6: unit 6H1, healing foundations

Found while porting ShapeExtend's rest, ShapeBuild_ReShape, ShapeConstruct, ShapeAnalysis's rest and
Geom2dAPI_Interpolate/PointsToBSpline. Each is reproduced in the port and marked `// OCCT`; "pinned" names the unit
test that shows it. None of these classes but ShapeAnalysis_Shell and FreeBounds is bound natively, and those two
showed no difference from native.

| Where | What | Effect | Triage |
|---|---|---|---|
| `ShapeExtend_CompositeSurface::Copy` | Initializes the copy with `Init(patches)`, whose parametrisation defaults to natural | A copy (and so `Transformed`, `Translated`, ...) loses uniform, unitary or custom joint values. Pinned by `CompositeSurface_Copy_LosesJointValues`; the port's `With*` twins keep them | touch |
| `ShapeConstruct::JoinPCurves` | Returns `i <= edges->Length()`: false when every pcurve was joined, true when the loop broke early (or on a plane) | The inverse of its documentation. No OCCT caller reads it. Pinned by `JoinPCurves_TwinJoinsTwoEdgesOnACylinder` | leave |
| `ShapeConstruct::ConvertSurfaceToBSpline`, the continuity-lowering loop | `if (cnt > 0) cnt--; continue;` retries C0 as long as GeomConvert_ApproxSurface throws | Can loop forever. By reading | fixed (decision 1432) |
| `ShapeAnalysis_ShapeTolerance::InTolerance` | Tests vertices with `tol >= valmax` where faces and edges test `<=` | `OverTolerance(s, 0)` returns every vertex and nothing else; a vertex inside [min, max] isn't found. Pinned | touch |
| `ShapeAnalysis_Geom::PositionTrsf` | `SetScale` resets the whole transformation | Any scale other than 1 discards the rotation computed before it. Pinned | touch |
| `ShapeAnalysis_WireVertex::Analyze` | Compares vertices with `V1 == V2`, orientation included | An edge's REVERSED last vertex never equals the next edge's FORWARD first one: status 0 ("same vertex") is never recorded, shared vertices get 1. Pinned | touch |
| `ShapeAnalysis_WireVertex::SetStart` | Stores the parameter on the preceding edge in `myUFol` instead of `myUPre` | The preceding parameter is lost. Pinned | touch |
| `ShapeAnalysis_WireVertex::NextCriter(1)` | Tests `stat > 0` where the documentation says `>= 0` | By reading | touch |
| `ShapeAnalysis_CheckSmallFace::CheckPin` | Reports `whatrow = nbu` when row 1 of U is the pin; the status and the DONE3 equal-poles check come from the last bound tested only | By reading | touch |
| `ShapeAnalysis_CheckSmallFace::CheckTwisted` | Computes the V step from the U range (`dv = (umax-umin)/nbint`) | The V sampling is wrong on non-square ranges. Pinned (a twist is still found) | touch |
| `ShapeAnalysis_CheckSmallFace::CheckPinEdges` | Tests `(M_PI - angle2)` twice; the second is probably meant for angle1 | By reading | touch |
| `ShapeAnalysis_CheckSmallFace` constructor | Leaves myStatus, myStatusPinFace, myStatusPinEdges and myPrecision uninitialised | The port starts them at OK and -1 | leave |
| `ShapeAnalysis_CheckSmallFace::IsSpotFace` | Returns 1 ("spot") for a face without wires and leaves spot and spotol unwritten | Pinned | touch |
| `ShapeAnalysis_CanonicalRecognition::GetSurface` (shell form) | The loop recomputes the parameters from the first face's surface instead of `anElemSurf1` | Every later face compares equal: any shell whose first face fits is accepted. By reading | touch |
| `ShapeAnalysis_CanonicalRecognition::GetCurve` | When the edge's 3D curve already is the target conic and the edge has a location, transforms the edge's own (shared) curve in place | Native OCCT changes shared geometry, and repeated calls accumulate the transformation. The port copies the curve first (decision 660). Pinned (three calls give the same result) | settled |
| `ShapeAnalysis_CanonicalRecognition`'s FillSolverData (cone) | The semi-angle's upper bound uses the radius step `aDR` instead of `aDAng` | By reading | touch |

## Phase 5: unit 5H, hatching

Found while porting Hatch, HatchGen, Geom2dHatch and GeomAPI_PointsToBSplineSurface. By reading OCCT's code unless a
test is named; the port reproduces each (`// OCCT` at the line). The hatcher has no native binding: unit 6B checks
it through IntTools_Context and BOPTools.

| Where | What | Effect | Triage |
|---|---|---|---|
| `Geom2dHatch_Hatcher::RemElement` | `HatchGen_PointOnHatching PntH = Hatching.ChangePoint(IPntH);` copies the point, so the element's points on element are removed from the copy | The hatching's points keep the removed element's points on element; only a point that had no other element is removed. As the trimming stays done, a later `ComputeDomains` reuses those points and their transitions. Kept; pinned by `RemElement_KeepsTheOtherElementsPointsOnHatching` | touch |
| `Geom2dHatch_Elements` copy constructor ("magic constructor") | Copies nothing: the new set is empty, with current edge 0 and probing parameter 0 (the default constructor sets 1 and 0.123) | A copied set of elements classifies as if it had none. Kept; pinned by `Elements_CopyConstructorCopiesNothing` | touch |
| `Geom2dHatch_Hatcher::GlobalTransition` | The parameter chosen from the point's position on the element (its first or last parameter at an end) is overwritten by the point's parameter (OCCT's own "TODO: finding #25") | The local geometry at an element's end is taken at the intersection's parameter, not at the end. Kept | touch |
| `Geom2dHatch_Hatcher::Trim(IndH, IndE)` | Writes " Intersector -> Done = False " to std::cout when an intersection fails | Output from a library; left out of the port (decision 666) | settled |
| `Geom2dHatch_Hatcher::IsDone()` (no index) | Declared, never defined | A link error for a caller; left out | leave |
| `Geom2dHatch_Element` default constructor | Leaves the orientation uninitialised | The port's is FORWARD | leave |
| `Hatch_Hatcher::StartIndex`, `EndIndex` | Don't check the oriented mode where `Start` and `End` do | On an unoriented hatcher, a line without intersections gives index 0 and parameter 0 for interval 1 where `Start` throws. Kept | touch |
| `GeomAPI_PointsToBSplineSurface::Init` (all forms) | Sets each multi-point at the grid's own row index (`MP.SetPoint(i, Points(i, j))` from `LowerRow()`) and each line entry at its column index (`Line.SetValue(j, MP)` from `LowerCol()`); the periodic form closes with `Points(1, j)` | A grid not indexed from 1 throws Standard_OutOfRange (or closes on the wrong point). Kept | touch |

## Phase 5: unit 5B, analytic intersections

Found while porting IntPatch_ImpImpIntersection, by reading OCCT's code; the port reproduces each (`// OCCT` at the
line). The conics are confirmed bit-identical by the early oracle (decision 676); the rest is 5G's to confirm.

| Where | What | Effect | Triage |
|---|---|---|---|
| `Perform`, the cone/torus case (`case 35`/`53`) | Doesn't set `bEmpty = empt` after IntCoTo, as every other pair does | An empty cone/torus intersection still runs the boundary search; with points on the boundaries it reports a non-empty result. Kept | touch |
| `IntCyCo`, the `IntAna_Point` case | `Quad1.Parameters(psol, U2, V2)` where Quad2 is meant | The isolated point's parameters on the second surface are those on the first. Kept | touch |
| `ProcessBounds` | The loop over `slin` advances `j` only on analytic lines | A non-analytic line in `slin` while a bound is still unprocessed loops forever. Its callers put only ALines in `slin` first, so it isn't reached. Kept | touch |
| `WorkWithBoundaries::BoundariesComputing`, `mB < 0` | Tests `mB <= -mB - 1` where its comment's derivation gives `mB <= -mC - 1` | The U1 range for that sign of B is chosen on a wrong bound. Kept | **fixed**; no Boolean could reach it (decision 1814) |
| `WorkWithBoundaries::BoundariesComputing`, `mB > 0` | `mB + \|mC\| < -1` can never be true for `mB > 0` | The "no solution" branch is dead. Kept | leave |
| `ProcessSegments` (a property) | `ptvtx` lives across the segments and the vertex loop; an end point no line carries is built from the last vertex read, setting only its value, parameters and (for a vertex) one arc | The vertex can keep the arc, vertex and multiple flag of an unrelated vertex on the other surface. Kept | touch |
| `IntCyCo`, `IntCoCo`, `IntCoSp` (a property) | `ptf`/`ptl` are declared outside the loop over the IntAna curves | Where a curve's bound is open, ProcessBounds gets the previous curve's end point (unused while that bound is open). Kept | touch |
| `CyCyAnalyticalIntersect` (a property) | Doesn't reset `Multpoint` | Harmless: IntCyCy resets it before | leave |

## Phase 5: unit 5F, approximating intersection lines

Found while porting ApproxInt, GeomInt_WLApprox and BRepApprox. The port reproduces each (`// OCCT` at the line), and
a native program built against OCCT 8.0.1 (decision 680) confirmed the first and the third; `ApproxIntTests` pins
the native values.

| Where | What | Effect | Triage |
|---|---|---|---|
| `ApproxInt_PrmPrmSvSurfaces::Compute`, `ApproxInt_ImpPrmSvSurfaces::Compute`, the two-entry cache | On a hit (the same parameters as one of the last two computations) they return the cached flag without writing their outputs: the point, the tangents and the refined parameters stay as the caller gave them | PrmPrm's `SeekPoint` called twice at the same parameters gives the origin with the unrefined parameters the second time, as a valid point (native: `(0, 0, 0)`); `MakeMLBetween` would insert a stale point if a resampled parameter set repeated one of the last two. Kept; pinned by `PrmPrmSvSurfaces_SeekPointTwice_ReturnsTheOriginTheSecondTime` and `ImpPrmSvSurfaces_ComputeTwice_LeavesTheOutputsTheSecondTime` | **fixed** (decision 1812) |
| `ApproxInt_ImpPrmSvSurfaces::Compute`, the parametric surface first | Stores the cache key `MyParOnS2.SetCoord(tu2, tu2)` where `(tu2, tv2)` is meant | The point is never found in the cache (unless tu2 = tv2) and is computed again: the same values, more work. Kept | **fixed** (decision 1812) |
| `ApproxInt_Approx::Perform(line, ...)` (the line alone) | With two knots and more than 2·NbPntMax points it adds a middle knot `(indicemax - indicemin) / 2`, an index count rather than `indicemin + ...` | From a first index above 1 the knots go backwards: on a straight line of 300 points, `Perform(WL, ..., 150, 300)` gives a curve from point 75, and from 200 a curve from point 50 (native too). Harmless from index 1 (BRepApprox's callers). Kept; pinned by `WLApprox_PerformLine_MiddleKnotIsAnIndexCount` | touch |

## Phase 5: unit 5C, walking

Found while porting IntWalk, IntPatch_ImpPrmIntersection and IntPatch_SpecialPoints. By reading OCCT's code; the port
reproduces each (`// OCCT` at the line). None changes the nine walks recorded natively (decision 687), which the port
matches bit for bit.

| Where | What | Effect | Triage |
|---|---|---|---|
| `IntWalk_IWalking::Perform(Pnts1, Func, S, Reversed)` (the overload without interior points) | Doesn't call `Clear()`, unlike the other overload: the previous lines are kept, and on a new object the vectors lack their dummy item 0 | The walking reads start point I + 1's state as point I's, and past the vectors' end for the last one (out of bounds in OCCT; Standard_OutOfRange in the port). No OCCT caller uses this overload | leave |
| `IntWalk_IWalking::TestArretCadre` (IntWalk_IWalking.gxx, at a stop found on the line) | Computes the function's direction, orients it against the line, then sets the line's tangent to the previous direction instead | The computed direction is unused; `Direction3d` still throws at a tangent point. Kept | touch |
| `IntPatch_ImpPrmIntersection.cxx`, `ToSmooth` (line 2422) | The loop runs `ip` from `startp` to `NbTestPnts`, where `startp + NbTestPnts` is meant | At the end of a line (`startp = NbPoints - NbTestPnts - 2`) the loop is empty unless the line is short; the averages stay 0, so AddVertices never adds the last vertex. Kept | touch |
| `IntPatch_ImpPrmIntersection.cxx`, `DecomposeResult` (line 3716) | The adaptor `anArc->Trim(...)` returns is discarded | The arc is never trimmed. Kept | touch |
| `IntPatch_ImpPrmIntersection.cxx`, `ComputeTangency` (lines 365–373) | On an INTERNAL or EXTERNAL arc, the complex-transition branch never assigns the vertex handle `vtx` | It keeps an earlier point's vertex, or is null, which would crash OCCT in `Identical` (a NullReferenceException in the port). Kept | touch |
| `IntPatch_ImpPrmIntersection.cxx`, `DecomposeResult` (lines 3188–3195; a property) | `aSSLine` is a second handle to the input line's points | AdjustLine and InsertSeamVertices edit the input line in place, even when it is replaced afterwards. Kept | touch |
| `IntWalk_PWalking` (the constructor with the bounds; a property) | Leaves `tgfirst`, `tglast`, `myTangentIdx` and `previoustg` uninitialised | Perform writes them before reading them; harmless | leave |
## Phase 6: unit 6H2, ShapeFix edges and wires

Found while porting ShapeFix's statics, Root, Edge, EdgeProjAux, Wire, WireSegment, WireVertex, IntersectionTool,
ShapeTolerance, SplitCommonVertex, EdgeConnect and SplitTool. Each is reproduced in the port and marked `// OCCT`
unless the table says otherwise; "pinned" names the unit test that shows it. Native ShapeFix_Shape on wires and edges
(the oracle) showed no difference from native.

| Where | What | Effect | Triage |
|---|---|---|---|
| `ShapeFix_ShapeTolerance::LimitTolerance`, TopAbs_WIRE | The edges' result isn't added to `fait` | A change to the edges' tolerances alone returns false. Pinned by `LimitTolerance_Wire_DoesntCountTheEdges` | touch |
| `ShapeFix_EdgeProjAux::Init2d`, `UpdateParam2d` | `theCurve2d->Reverse()` on the handle `BRep_Tool::CurveOnSurface` returns | Computing two parameters reverses the edge's own pcurve, and every shape's sharing it. The port puts a reversed copy in place of the curve on this edge only, unless the edge is frozen (decision 1263; the caller's range is set on the reversed curve, so it is read afterwards) | settled |
| `ShapeFix_Edge::FixReversed2d` | Reverses the stored pcurve in place | Other edges sharing the pcurve are reversed too. The port puts a reversed copy in place of the curve on this edge only (decision 700) | settled |
| `ShapeFix_WireVertex::FixSame` | Compares `V1 == V2` (orientation included); `B.Add`s the shared vertex without removing the edges' old ones; doesn't make the edges Free first, as `Fix` does | A shared vertex is never detected; the edges end with three or four vertices; on the edges of a wire `Add` throws `TopoDS_FrozenShape`. Pinned | touch |
| `ShapeFix_IntersectionTool::FixIntersectingWires` | Calls `sewd->Wire()` three times per modified wire | The face gets one new wire, the context records a second as the replacement and the loop goes on with a third: `Context()->Value(oldWire)` isn't the face's wire | touch |
| `ShapeFix_Wire::FixTails` | Ends with `myStatusNotches = myLastFixStatus` | `StatusFixTails()` always reports OK, and the notches' status is overwritten by the tails' | touch |
| `ShapeFix_Wire::FixGap2d`, clipping an extended pcurve at the surface's bounds | Uses `jj` (which end of the segment) where `j` (which curve) is meant to pick `reversed1`/`!reversed2` and `cfirst1`/`clast2` | The segment ends are clipped on the wrong curve's side. By reading | touch |
| `ShapeFix_Edge::FixAddPCurve`, seam branch | Dereferences the projected pcurve, which can be null | Inside OCC_CATCH_SIGNALS it ends as FAIL2. The port throws `Standard_NullObject` inside the same `try`, with the same status | settled |

Map order: `ShapeFix_EdgeConnect::Build` walks `myLists` (a DataMap keyed by vertex) in native's hash order; the port
walks it in insertion order (decision 18). Each shared vertex is computed on its own, so only the order of the writes
changes.

## Phase 5: unit 5D, parametric intersections

Found while porting IntPatch_PrmPrmIntersection, the polyhedra and their interference. By reading OCCT's code; the
port reproduces each (`// OCCT` at the line). None changes the seven intersections recorded natively (decision 711),
which the port matches bit for bit.

| Where | What | Effect | Triage |
|---|---|---|---|
| `IntPatch_InterferencePolyhedron::Perform(const IntPatch_Polyhedron&)` | Calls the self-interference `Interference(Obje)`, whose body is empty; the constructor of the self-interference calls `Interference(Obje, Obje)` instead | `Perform` on one polyhedron finds nothing (only the mode and tolerance are set). OCCT's own callers use the constructor. Kept, and pinned by a test | leave |
| `IntPatch_Polyhedron.hxx` | Declares `Perform`, `UMinSingularity`... (the four setters) and `HasUMinSingularity`... (the four getters), which no .cxx defines | A caller would fail to link. Left out of the port | leave |
| `IntPatch_Polyhedron::PlaneEquation` | On a triangle with an edge shorter than 1e-7, sets the normal to (1, 0, 0) and returns without writing `PolarDistance` | The caller's variable keeps its value (uninitialised in a typical caller). Nothing in OCCT calls it; kept (`ref` in the port) | leave |
| `IntPatch_Polyhedron::TriConnex` (the end; OCCT's own "BUG NON CORRIGE") | When the other point of the connected triangle coincides with `Pedge`, returns 0 instead of looking further, with `TriCon` and `OtherP` set | InterferencePolyhedron reads `TriCon` (into `pivo`) to tell whether the neighbouring triangle has already given the segment of an edge lying in the other triangle's plane; at such a vertex it decides on the triangle found so far. Kept | touch |
| `IntPatch_PrmPrmIntersection::NewLine` (line 1555) | Tests `nab > ds`, where `nab = 1 / (a + b)` is the inverse of the segment's length and `ds` the arc-length step | A point is inserted only in segments shorter than `1 / ds` (in parameter units, so the test depends on the parametrisation's scale). Nothing in OCCT calls NewLine. Kept | leave |
| `IntPatch_PrmPrmIntersection::Perform` (the sampled one, the branch for more than 2,500 samples) | Doesn't clear the lines, whatever `ClearFlag` says (the other branch clears them when it is true) | A second sampled Perform on the same object keeps the first one's lines. IntPatch_Intersection uses a fresh object (or wants them kept). Kept | touch |
| `IntPatch_PrmPrmIntersection::Perform` (with the list of points; a property) | Appends the points' images across the seams of closed surfaces to the caller's list | The caller's list grows. Kept | touch |
| `IntPatch_PrmPrmIntersection::PointDepart` (lines 3689-3718; a property) | Converts `(P.X() - x0 + dx2) / dx` to `int` without a range check (undefined behaviour in C++ out of range; MSVC gives `INT_MIN`) | Out of range and NaN samples fall outside the grid. The port reproduces MSVC's result (decision 708) | leave |
## Phase 6: unit 6B, context and BOP tools

Found while porting IntTools_Context, FClass2d, TopolTool and Tools and BOPTools' AlgoTools (all three files),
AlgoTools2D and AlgoTools3D, and auditing the context's caches (plan 2.4, decision 714). Each is marked `// OCCT` at the
line (or commented) and ported as OCCT has it; "pinned" names the unit test that shows it.

| Where | What | Effect | Triage |
|---|---|---|---|
| `IntTools_FClass2d::Init` | Doesn't clear TabClass/TabOrien or reset the face explorer | A second Init appends the new face's polygons to the old ones, and a point the polygons can't decide is classified against the old face. IntTools_Context never re-initialises a classifier. By reading | touch |
| `IntTools_FClass2d::Init` | Returns on an edge without a pcurve, with the classifier half built (the wires sampled so far, partial bounds, U1..V2 unset) | The classifier answers from the partial polygons. The port's unset fields are 0 | touch |
| `IntTools_FClass2d::Init`, refinement loop | Never recomputes the polygon's area and perimeter, so `anExpThick` stays the first polygon's | By reading | touch |
| `IntTools_FClass2d::IsHole` | Reports the orientation of the last wire with a non-zero area | A face whose hole is drawn after its outer wire reports `true`. Pinned | touch |
| `IntTools_TopolTool::SamplePoint` | Maps index 1 to the second sample in U (`iu = 1 + Index - (iv-1)*n`, as if the index started at 0) | Index n·m gives `v = vsup`, on the boundary, and the first grid node is never sampled. Pinned | touch |
| `IntTools_Tools::IsVertex(Edge, t)` | Computes the vertex tolerance, then overwrites it with 1e-12 (squared) | The vertex's tolerance is ignored. By reading | touch |
| `IntTools_Tools::RejectLines` | Writes into its const input through a const_cast (a trimmed curve replaced by its basis) | The caller's sequence changes. Kept | touch |
| `BOPTools_AlgoTools::ComputeStateByOnePoint` | Discards the state of the recursive call on a container's first sub-shape | Returns TopAbs_UNKNOWN for any compound, shell, solid. Pinned | touch |
| `BOPTools_AlgoTools_1.cxx`, `IntersectCurves2d` | `aLen2 = MapEdgeLength(*theEData1.Edge, …)` uses the first edge twice | The distance threshold ignores the second edge's length. By reading | touch |
| `BOPTools_AlgoTools2D_1.cxx`, file-static `IsClosed` | The loop condition `(aExp.More() \|\| iCnt == 2)` keeps going once a seam is counted twice at the end; `TopExp_Explorer::Next()` on an empty stack pushes the face's iterator again, so the count reaches 4 | A true seam gives false, so `AttachExistingPCurve` never calls `UpdateClosedPCurve`, and the split of a seam gets one pcurve. Pinned (`IsClosed_of_AlgoTools2D_1_reproduces_OCCTs_restarting_loop`) | touch |
| `BOPTools_AlgoTools3D::GetNormalToSurface` | Calls `IsDirsCoinside(aDD1U, aDD1U)`, comparing DU with itself | That check always passes. By reading | touch |
| `BOPTools_AlgoTools3D.cxx`, `HasGeometry` (IsEmptyShape) | Calls `CR->Polygon3D()` on every representation that isn't a 3D curve, curve on surface or regularity; the base class throws Standard_DomainError | An edge whose first such representation is a polygon on triangulation or on surface throws instead of reporting geometry. By reading | touch |

**Call-history and thread-timing dependence (plan 2.4, decisions 580 and 714).**

- **IntTools_SurfaceRangeLocalizeData's box cache** (the context's `SurfaceData(face)`). IntTools_BeanFaceIntersector's
  `ComputeLocalized` resets the out ranges and the grid at each run (`RemoveRangeOutAll`, `ClearGrid`) but not the
  boxes. They are keyed by sample range (depth and indices), not by parameters or tolerance, and enlarged by
  `myCriteria` (the edge's tolerance plus the face's, BeanFaceIntersector.cxx lines 177 and 199; `GetSurfaceBox` and
  `BndLib_AddSurface::Add` at lines 1532–1569, and the root box at 1832–1869 for B-spline faces). So the first edge
  intersected with a face decides the enlargement of the boxes every later edge on that face uses, and the boxes
  only reject (`IsOut`): a box made with a smaller tolerance can reject a range where a later, more tolerant edge
  touches. Native's per-thread contexts make that order thread-dependent, so native's parallel edge/face results
  can differ from its serial ones. Unit 6C decides how the port keeps serial = parallel (decision 723).
- **BRepClass3d_SolidClassifier reused by the context** (`SolidClassifier`, `ComputeState`). BRepClass3d_SolidExplorer's
  `myParamOnEdge` advances when a whole pass over the faces finds no ray and is never reset (Segment resets only
  `myFirstFace`); that pass also replaces cached face intersectors whose infinite point is IN with unrestricted ones,
  permanently. IntCurvesFace_Intersector's `myIsParallel` is set only when the line meets nothing and never reset, so
  SClassifier's parallel-line ON check can read a flag left by an earlier line. A reused classifier can then answer
  differently from a fresh one. Rare paths; none was hit on the tests' solids, and the oracle (same call order) matched
  native.
- **BOPTools_AlgoTools' tolerance corrections in parallel** (CorrectPointOnCurve, CorrectCurveOnSurface,
  CorrectShapeTolerances with `bRunParallel`). Parallel items share vertices and edges and update their tolerances with
  unsynchronised read-compare-writes whose outcome depends on the tolerance read first (CheckEdge's `0.1 * Tol` margin,
  CorrectWires', CorrectEdgeTolerance's comparisons). A vertex whose two faces need √dA < √dB < 1.01√dA ends with
  1.01√dA or 1.01√dB depending on which face runs first, and a concurrent max can be lost. Native's parallel result
  depends on thread timing; the port runs these loops serially (decision 720).

Not bugs, but worth knowing: `BOPTools_AlgoTools::GetFaceDir`'s `Standard_ASSERT_RETURN(aProjPL.IsDone(), …, false)`
returns false in release builds, and the port does the same.
## Phase 6: unit 6H3, ShapeFix faces to shapes

Found while porting ShapeFix_Face, ComposeShell, Shell, Solid, Shape, Wireframe, FixSmallFace, FixSmallSolid,
FaceConnect and FreeBounds. Each is reproduced in the port and marked `// OCCT` unless the table says otherwise;
"pinned" names the test that shows it.

| Where | What | Effect | Triage |
|---|---|---|---|
| `ShapeFix_Face::FixMissingSeam`, two open wires of the same orientation | `isdeg2 = isdeg` stores the flag (1) where the other branch stores the wire's index | A later `ws.Remove(isdeg ? i : isdeg2 ? isdeg2 : isdeg1)` can remove wire 1 instead of the degenerated one. By reading | touch |
| `ShapeFix_Face::FixSmallAreaWire` | Tests `ShapeType() != WIRE && Orientation() != FORWARD && != REVERSED` where `\|\|` is meant | FORWARD/REVERSED vertices and INTERNAL wires reach `TopoDS::Wire`; OCCT's release build reinterprets a vertex as a wire without edges, which is kept. The port does the same instead of throwing. Pinned by a ShapeFix_FaceTests case | touch |
| `ShapeFix_Face::Perform` (a property) | `BRepTools::Update` sets the Checked bit of the caller's face in place | The port never writes the caller's face; the oracle masks the Checked column with the Free one | leave |
| `ShapeFix_Wireframe::FixWireGaps`, `FixSmallEdges` on a compound | Each recursive call resets the status | The statuses are the last child's only. Reproduced by the oracle | touch |
| `ShapeFix_Wireframe::MergeSmallEdges`, the free-wire block | Reads `anExpf2.Current()` after that face explorer is exhausted | It yields the explored shape; the port has the same semantics | leave |
| `ShapeFix_Shell::Perform` | When FixFaceOrientation gives up (Möbius-like faces) it sets FAIL but not DONE2, and the new shell isn't recorded in the context | Perform returns false. Pinned by the Möbius band test | touch |
| `ShapeFix_Shell::FixFaceOrientation` | Always rebuilds the shell, even when nothing is reoriented, and then records nothing | `Shape()` is a new shell holding the same faces, while the context maps the input to itself | touch |
| `ShapeFix_Shape::Perform` | The early return (a shape already fixed or made by the context) and the abort paths don't restore the face tool's FixSmallAreaWireMode | A later Perform on a non-face can keep the forced mode | touch |
| `ShapeFix_FreeBounds::Perform` | Removes and adds an edge's vertices while a TopoDS_Iterator walks them | The list iterator reads a freed node; the vertex order inside the edge then depends on freed memory. The port walks a snapshot (the same vertices) | settled |
| `ShapeFix_FreeBounds::Perform` | `BRep_Builder::Remove`/`Add` on edges that sit in wires (not Free) | `TopoDS_FrozenShape` is compiled out in OCCT's release build but raised by its debug build and by the port. The port sets the Free flag around the edit, as ShapeFix_EdgeConnect does in OCCT | settled |
| `ShapeFix_FixSmallFace::SplitOneFace` | Removes and adds edges of `wireonface` while a TopExp_Explorer walks it | The same freed-node read; the port explores a snapshot first | settled |

Map order (decision 18; native iterates hashed maps in address order, the port in insertion order):
ShapeFix_Shell's GlueClosedCandidate, AddMultiConexityFaces and CreateNonManifoldShells (`mapmerge`; the result can't
change there, `IndexedDataMap::Add` keeps the first value), ShapeFix_Solid's CollectSolids, ShapeFix_Wireframe's edge
maps, ShapeFix_FixSmallSolid's MergeShells (the face order in the merged shell) and ShapeFix_FaceConnect's
`theFreeEdges`, `myConnected`, `myOriFreeEdges`, `theRepEdges` and `theNewVertices` (the order of the sewing inputs and
replacements). Only the order of faces, shells or writes can differ; the oracle saw no case.

## Phase 5: unit 5G, surface/surface intersection

Found while porting IntPatch_Intersection, ALineToWLine, WLineTool, LineConstructor, GeomInt and Draw's `intersect`. By
reading OCCT's code; the port reproduces each (`// OCCT` at the line). None shows in the oracle's GeomAPI_IntSS
comparisons (decision 748), where every pair is bit-identical.

| Where | What | Effect | Triage |
|---|---|---|---|
| `IntPatch_Intersection::Perform(S1, D1, S2, D2, TolArc, TolTang, ...)`, two tori | `bToCheck = aTor2.MajorRadius() > aTor2.MinorRadius()` overwrites the first torus's test; the list-of-points Perform combines both and also accepts equal radii within TolTang | A self-crossing first torus is sent to ImpImp by one overload and not by the other. Kept | touch |
| `IntPatch_Intersection.cxx`, `FUN_GetUiso` (offset surface whose basis U-iso is a line, parabola or hyperbola) | `Geom_RectangularTrimmedSurface(gos, VmTr, VMTr, true)` gives the V range as a U trim; `FUN_GetViso`'s call is right | The U-iso of the trimmed offset keeps its infinite V range. Kept | touch |
| `IntPatch_Intersection.cxx`, `FUN_NewFirstLast` | The second test isn't `else if` | A line, parabola or hyperbola range on the positive side (both ends ≥ 0) falls to the final `else` and is replaced by the symmetric [-TrVal, TrVal]; a hyperbola then also misses the `Fst > 4` early return. Kept | touch |
| `IntPatch_LineConstructor::Perform`, the GLine branch and the "RLine on S2" branch | `myDom1->Init(); if (myDom2->More())` tests the other domain's iteration state | The restriction loop runs, or not, on the state myDom2 was left in. Kept | touch |
| `IntPatch_LineConstructor::Perform`, the same branches | Read `Tol3d(...)` from the domain, which a plain `Adaptor3d_TopolTool` doesn't have (Standard_DomainError) | LineConstructor on GLines or one-sided RLines works only with a face's topology tool. OCCT reaches the class only from TopOpeBRep (and `IntPatch_Intersection::Dump`, compiled out). Pinned by a test | touch |
| `IntPatch_LineConstructor::Perform`, GLine branch, closed conics | The arc from the last vertex to the first is classified with the `Tol` of the loop's last iteration | Kept | touch |
| `IntPatch_WLineTool::ComputePurgedWLine` | `nb` is reassigned inside the equal-points loop; the later `nb < aNbSingleBezier` test uses the count before the last removal | Kept | touch |
| `GeomInt_IntSS::InternalPerform` | Clears `sline` but not `slineS1`/`slineS2` | On a GeomInt_IntSS performed twice, `LineOnS1/S2(i)` and `HasLineOnS1/S2(i)` return the first Perform's 2D curves. GeomAPI_IntSS never asks for them. Kept | touch |
| `GeomInt_IntSS::InternalPerform`, start-point mode | Skips `myIntersector.Perform` when the start point is outside a domain | `IsDone()` and the lines come from the previous Perform on the same object (or "not done"). Kept | touch |
| `GeomInt_IntSS::BuildPCurves` (GeomInt_IntSS_1.cxx) | Calls `theCurve2d->DynamicType()` after a second `GeomProjLib::Curve2d` that may return null | Access violation in OCCT, NullReferenceException in the port. Kept | touch |
| `GeomInt_LineTool::DecompositionOfWLine` | Its two neighbour searches step one index past a part before testing | When all points of an end part coincide in UV, `Point(i)` is read past the line (unchecked in OCCT's release build; Standard_OutOfRange in the port). Kept | touch |
| Draw's `intersect`, curve/surface branch (GeometryTest_CurveCommands.cxx) | The points' loop calls `Inters.Point(i)` with `i` running from `nblines + 1` | With segments and points both, the points read are past the last one (Standard_OutOfRange, reported by Draw) and the first points are skipped. Kept in Draw-lite | leave |

## Phase 6: unit 6C, edge intersectors

Found while porting IntTools_EdgeEdge, IntTools_EdgeFace and IntTools_BeanFaceIntersector. Each is ported as OCCT has
it, except the first (decision 753).

| Where | What | Effect | Triage |
|---|---|---|---|
| `IntTools_BeanFaceIntersector::LocalizeSolutions`, `ComputeLocalized` with `IntTools_Context::SurfaceData` | The face's sample boxes are cached across runs, keyed by sample only, enlarged by the first edge's criteria, and a cached box skips the check against the run's main curve box (6B's entry) | A run's result depends on what its context computed before, so native's parallel edge/face results depend on thread order. Measured (tools/edgerec `study`): 1 run in 25,308 (8,436 pairs on localising faces, in order, reversed and with alternating fuzzy values) differs from a fresh context, by 32 ulps in a range. **The port keys the cache and checks cached boxes as fresh ones (decision 753)**, so it always gives the fresh-context result; that run is the residual difference from native's serial result | leave |
| `IntTools_BeanFaceIntersector`, `myMinSqDistance` | Only `ComputeAroundExactIntersection` and `ComputeUsingExtremum` update it; the line/plane path, `FastComputeAnalytic`, the coincidence test and `ComputeLocalized` never do | `IntTools_EdgeFace::MinimalDistance()` is RealLast for edges on B-spline faces even with common parts (all 177 such recorded runs), and BOPAlgo_PaveFiller_5's distance check is skipped for them. Recorded | touch |
| `IntTools_EdgeFace::CheckTouchVertex` | Builds its surface adaptor on the surface's natural bounds (`Surface->Bounds`), where `CheckTouch` uses the face's | On a plane or another infinite surface the extrema run on an unbounded surface. By reading | touch |
| `IntTools_EdgeFace::CheckData` | Sets status 2 for a degenerated edge, then overwrites it with 3 if the edge has no geometry | The degenerated status is lost for such edges. By reading | touch |
| `IntTools_BeanFaceIntersector::ComputeAroundExactIntersection` | With more than one exact intersection point it lowers `myCriteria` to 3·Confusion for the rest of the run (meant for the range search) | The extremum and boundary passes that follow use the lowered criteria too. By reading | touch |
| `IntTools_CurveRangeLocalizeData::ListRangeOut` (decision 629) | Native lists the curve's out ranges in hash order, the port in insertion order; `ComputeLocalized` inserts them into the range manager with flag 1 | The ranges are disjoint and share one flag, so only zero-length ranges at shared ends can come out in a different order. No difference in the 1,562 recorded runs on localising faces | leave |
## Phase 6: unit 6D, face/face intersection

Found while porting IntTools_FaceFace and IntTools_WLineTool. By reading OCCT's code unless said; the port reproduces
each (`// OCCT` at the line). None shows in the comparison with native IntTools_FaceFace (decision 761), where every
pair is bit-identical.

| Where | What | Effect | Triage |
|---|---|---|---|
| `IntTools_FaceFace::Perform` | Clears the curves but never `myPnts` (5G's `InternalPerform` bug in FaceFace's form) | A second Perform on the same object appends its points to the first's. Pinned by a test. The pave filler makes one object per face pair, so no Boolean is affected | touch |
| `IntTools_FaceFace.cxx`, `PerformPlanes` | Without the second 2D curve (`theApprox2` false) it clears the first (`SetFirstCurve2d(H1)` in the second branch) | A plane/plane line asked for with the first 2D curve only has none. The pave filler asks for both or neither. Pinned by a test | touch |
| `IntTools_FaceFace.cxx`, `PerformPlanes` | Two planar faces without wires (unbounded) give `ClassifyLin2d` the infinite bounds; `Geom_TrimmedCurve` on the resulting parameters throws | Standard_ConstructionError, natively too (three such pairs in the recordings). Pinned by a test | touch |
| `IntTools_FaceFace::MakeCurve`, walking lines | `myTolApprox = 1.e-5` (cylinder/sphere pairs where `ApproxWithPCurves` is false, and the re-approximation after an invalid 2D curve) writes the member | The looser tolerance stays for the following lines and for later Performs on the object (until SetParameters) | touch |
| `IntTools_FaceFace::MakeCurve`, `goto reapprox` | Restarts the whole line although the curves of its earlier parts (or multicurves) are already appended | Those curves are appended twice. Kept | touch |
| `IntTools_FaceFace::MakeCurve`, lines, parabolas, hyperbolas | Checks `C2d.IsNull()` after `GeomInt_IntSS::BuildPCurves`, which has already dereferenced a null second projection (5G's issue) | The check can't help: OCCT crashes, the port throws NullReferenceException. Kept | touch |
| `IntTools_FaceFace::SetList` and Perform | The start points are kept between Performs; on a reversed pair Perform swaps each point's parameters in place | A second reversed Perform swaps them back to the wrong order; the list also grows (decision 706) | touch |
| `IntTools_FaceFace.cxx`, `CheckPCurve` | The sample parameter runs on from the curve's first parameter by `(NPoints - 1) * dT` per interval | With several C∞ intervals the samples drift from the intervals and the end of the last one is never checked | touch |
| `IntTools_WLineTool.cxx`, `IsPointInDegeneratedZone` | Reads the first surface's bounds for the second (`aS1->Bounds(US21, ...)`), and compares V on both surfaces, and U on the second, with `aDelta` (1e-7) instead of the resolution `aD` it has just computed | The degenerated-zone test of the second surface uses the first's bounds and a 3D tolerance as a parametric one | touch |
| `IntTools_WLineTool::DecompositionOfWLine`, "correction of projected coordinates" | `X < umin && X > umax && Y < vmin && Y > vmax` can't hold (and uses the other surface's bounds) | The rough re-projection never runs | touch |
## Phase 6: unit 6U, UnifySameDomain

Found while porting ShapeUpgrade_UnifySameDomain. Each is reproduced in the port (on its copy of the input) and marked
`// OCCT` unless the table says otherwise.

| Where | What | Effect | Triage |
|---|---|---|---|
| `IntUnifyFaces` → `AddPCurves` → `TransformPCurves` | Adds the reference surface's pcurves to the edges of the faces it unifies, and resets their ranges (`BRep_Builder::Range`), on the input's own edges, whatever `SetSafeInputMode` says ("the input shape can't be modified") | The caller's edges change in place, and the result shares them. The port runs on a copy (decision 766); its history lists those edges as modified into their changed copies, where native's has no entry (decision 643). Reproduced by the oracle (37 of the 480 Clean runs have such entries) | settled |
| `ReconstructMissedSeam` | `UpdateEdge` swaps the two pcurves of a removed seam edge of the input | The same in-place write into the input | settled |
| `IntUnifyFaces`, ConcatBSplines with an edge closed on the face but not a seam | `SetUPeriodic`/`SetVPeriodic` on the face's own B-spline surface | Every face sharing that surface, the input's included, becomes periodic. The port makes a periodic copy and puts it in the original's place in the shape it works on (decision 769), so its result is native's byte for byte (`bowl_noremove.brep`) and the caller's surface is unchanged. On a rectangular trimmed surface native edits the shared basis; the port copies the basis only | settled |
| `IntUnifyFaces`, the wire builder | `LastVertexOfSeam` and `StartOfNextEdge` are declared and tested but never set | The end-of-wire test after ReconstructMissedSeam never holds. Kept | touch |
| `UpdateMapOfShapes` | Adds to the NCollection_Map it iterates | Whether the new shapes are visited depends on the buckets; they map to themselves, so nothing changes. The port iterates a snapshot | settled |
| `UnifyEdges` with `mySafeInputMode` | `UpdateMapOfShapes(myKeepShapes, ...)` adds the context's images to the member map of shapes to keep | A second `Build` without `Initialize` keeps them too. The port restores the caller's map after Build (the images are of its copy) | touch |
| `SetSafeInputMode(false)` | Builds pcurves on planes in the input's edges and lets ShapeFix_Face edit them without a context | The documented behaviour; in the port it acts on the copy of a frozen input | leave |

Not bugs, but worth knowing: native UnifySameDomain iterates hashed maps only for membership (`myKeepShapes`, the shells of
a face), so its result doesn't depend on addresses; every oracle run was byte-identical or Free-masked.

## Phase 5: unit 5K1, analytic constraints

Found while porting GccAna. Each is ported as OCCT has it and marked `// OCCT` at the line; the native recordings
(tools/gccrec, decision 780) agree with the port on all of them. "Unset" means an NCollection_Array1 element OCCT
never writes: uninitialised natively, 0 in the port (decision 781). Out-of-range indices throw Standard_OutOfRange
natively and in the port (decision 778).

| Where | What | Effect | Triage |
|---|---|---|---|
| `GccAna_Circ2d3Tan.cxx` (three circles) | `distcc2` and `distcc3` are measured from `center1` | WhichQualifier's computed qualifiers 2 and 3 are wrong for unqualified circles | fixed (decision 1540) |
| `GccAna_Circ2d3Tan.cxx` (three circles) | `rSol` has 16 slots where `xSol`/`ySol` have 64 | More than 16 candidate triplets throw Standard_OutOfRange | fixed (decision 1540) |
| `GccAna_Circ2d3Tan_1.cxx` (circle, circle, line) | `ok` is not reset when the second circle's check fails, and the distance to the line is never compared with the radius | Qualified runs return circles tangent to the first circle only (56 of 144 qualifier combinations in a sweep). Pinned by a test | fixed (decision 1540) |
| `GccAna_Circ2d3Tan_1.cxx` | The line's qualifier branches test `Qualified2` where `Qualified3` is meant; `distcc2` is measured from `center1`; the second tangency's guard tests `pnttg1sol` and writes `par1sol` | Wrong computed qualifiers and tangency parameters | fixed (decision 1540) |
| `GccAna_Circ2d3Tan_3.cxx` (three lines) | `cote2` assigned where `cote3` is meant (all unqualified: harmless, that branch returns every solution); `cote3(1)` assigned twice and `cote3(2)` never, in the branches (unqualified, enclosed, unqualified) and (enclosed, unqualified, unqualified) | The solutions of those two branches are filtered with an uninitialised side: native's count varies (the recordings skip those cases) | fixed (decision 1540) |
| `GccAna_Circ2d3Tan_3.cxx` | The qualified filtering moves the circle to its new slot but not `qualifier1..3` | Wrong qualifiers after filtering | fixed (decision 1540) |
| `GccAna_Circ2d3Tan_4.cxx` (circle, circle, point) | The duplicate-removal loop doesn't shift `TheSame1..3` | IsTheSame of the solutions after a removed duplicate is that of their predecessors | fixed (decision 1540) |
| `GccAna_Circ2d3Tan_5.cxx` (circle, line, point) | A 1999 change reversed `dc` | The first tangency point is on the far side of the solution unless the solution is enclosed in C1. Pinned by a test | fixed (decision 1540) |
| `GccAna_Circ2d2TanOn.cxx` (two circles, centre on a line) | Up to four boundary cases or matches write `Radius(3..4)` into a 2-element array; the qualifiers of unqualified circles use `Radius(i)`, `i` the bisector's index, instead of `Radius(k)`; `Rbid(1)` is written twice and `Rbid(2)` never; `pararg1` is computed from `pnttg2sol` | Standard_OutOfRange in 189 of 1,500 random cases; otherwise wrong qualifiers and parameter | fixed (decision 1541) |
| `GccAna_Circ2d2TanOn.cxx` | `CenterOn3` returns `pnttg1sol` (a tangency point), not the centre; `ThisSolution` checks neither IsDone nor the index, `NbSolutions` not IsDone | Callers reading the centre get a tangency point | fixed (decision 1541) |
| `GccAna_Circ2d2TanOn_1.cxx` (circle, line, centre on a line) | The "other side" retry for `pinterm` recomputes the same point | The other side is never tried | fixed (decision 1541) |
| `GccAna_Circ2d2TanOn_3.cxx` (circle, point, centre on a line) | `if (std::abs(popradius - dist1))` tests a double as a boolean; `pararg1(i)` uses the bisector's index; `pararg2` is unset on the boundary path; with C1 qualified the radius follows the qualifier whichever hyperbola branch the centre is on | Some solutions don't pass through the point (pinned by a test); unset and misplaced parameters | fixed (decision 1541) |
| `GccAna_Circ2d2TanOn_6.cxx` (two circles, centre on a circle) | As `GccAna_Circ2d2TanOn.cxx`: `Rbid(2)` unset, `Radius` overrun | As above | fixed (decision 1541) |
| `GccAna_Circ2d2TanOn_7.cxx` (circle, line, centre on a circle) | The retry uses the same `distcco - Ron` point and doesn't recompute `d1`/`d2`; the boundary test checks `Qualified1` against the line's side, with the same sign for enclosed and outside; the boundary case never sets `pntcen` | Missed solutions; `parcen3` is the parameter of the default origin | fixed (decision 1541) |
| `GccAna_Circ2d2TanOn_8.cxx` (circle, point, centre on a circle) | The radius follows `Qualified1` only (both radii when unqualified), whichever branch the centre is on; `SameCenter` is always false | Some solutions don't pass through the point (pinned by a test) | fixed (decision 1541) |
| `GccAna_Circ2d2TanOn_9.cxx` (two lines, centre on a circle) | `D1.Angle(D2) <= Resolution` is signed, so lines turning clockwise take the "parallel" branch, where the second line's qualifier test overwrites the first's; the centre is `Intp.Point(i)` (the first bisector's index) instead of `Point(l)`; the tolerance is unused | Wrong branch for half the line pairs; Standard_OutOfRange when that point doesn't exist | fixed (decision 1541) |
| `GccAna_Circ2d2TanOn_10.cxx` (line, point, centre on a circle) | The boundary `pinterm` is `distpc + Ron` from the point but compared with `distpc - Ron`; `parcen3` is computed then overwritten with 0; tangency point 1 is at distance `dist1` instead of `Radius` | Wrong boundary detection, centre parameter and tangency point | fixed (decision 1541) |
| `GccAna_Circ2d2TanOn_11.cxx` (two points, centre on a circle) | The `C1` enlargement branch repeats conditions that returned earlier | Dead code | leave |
| `GccAna_Circ2d2TanRad.cxx` (two circles) | `C2 = gp_Circ2d(C1)` overwrites C2 in the enclosed/enclosed boundary case | `pararg2` is a parameter on C1 | fixed (decision 1542) |
| `GccAna_Circ2d2TanRad.cxx` | `std::abs(R1 - dist - R2) > 0.0` (meant `<= Tol`) always holds | Enclosing + unqualified always returns one circle, not tangent to the second; `TheSame2(1)` unset when the solution is the first circle only | fixed (decision 1542) |
| `GccAna_Circ2d2TanRad.cxx` | Outside + unqualified builds `C(2)` on `C(1).Location()` | The pair is concentric and 2 solutions are lost | fixed (decision 1542) |
| `GccAna_Circ2d2TanRad_4.cxx` (two lines) | `normL1`/`normL2` are the lines' directions, not their normals | Wrong computed qualifiers for unqualified lines | fixed (decision 1542) |
| `GccAna_Circ2dTanOnRad.cxx` (circle, centre on a line) | `CenterOn3` returns `pnttg1sol`, not `pntcen3`; the tangency point's sign is swapped in the general branch | The tangency point is diametrically opposite; enclosed with R1 = 2R puts it at C1's centre, whose parameter throws Standard_ConstructionError | fixed (decision 1543) |
| `GccAna_Circ2dTanOnRad_1.cxx` (line, centre on a line) | `pntcen3(NbrSol) = cirsol(1).Location()` | Solution 2 reports solution 1's centre | fixed (decision 1543) |
| `GccAna_Circ2dTanOnRad_2.cxx` (point, centre on a line) | With the point on OnLine, 2 solutions are found but WellDone stays false; `qualifier1` unset there | The accessors throw StdFail_NotDone | fixed (decision 1543) |
| `GccAna_Circ2dTanOnRad_3.cxx` (circle, centre on a circle) | `qualifier1(NbrSol)` written while NbrSol = 0 | Standard_OutOfRange (a hand-picked recorded case) | fixed (decision 1543) |
| `GccAna_Circ2dTanOnRad_4.cxx` (line, centre on a circle) | `sign = (int)dc1.Dot(normL1)` truncates to 0, so the tangency point is the centre | Every generic case throws Standard_ConstructionError (the parameter of the centre): 44 of 120 recorded cases | fixed (decision 1543) |
| `GccAna_Circ2dTanOnRad_5.cxx` (point, centre on a circle) | `Location - signe * Point1` is a sum when `signe = -1`; the loop writes index 1, not NbrSol | Wrong boundary circle; solution 2's qualifier, IsTheSame and tangency data unset | fixed (decision 1543) |
| `GccAna_Lin2d2Tan.cxx` (two circles) | `qualifier2(...) = Qualified1.Qualifier()` in almost every branch; for unqualified circles WhichQualifier returns GccEnt_unqualified, not a computed qualifier (also circle + point); the mixed branches write solution 2's qualifiers into `qualifier1(1)`/`qualifier2(1)`; unqualified/enclosing gives solution 2 the first's tangency point | Wrong or unset qualifiers; a wrong tangency point (pinned by a test) | fixed (decision 1544) |
| `GccAna_Lin2dTanObl.cxx` | The second intersection is computed again with `linsol(1)`; a solution parallel to TheLine gets the intersection data of a default point | Intersection2(2) = Intersection2(1); (0, 0) data for parallel solutions | fixed (decision 1544) |
| `GccAna_Lin2dTanPer.cxx` | `qualifier1` never set by the (point, line), (point, circle) and (circle, line) constructors; `maxdist = RealLast()` never lowered, so the last intersection point is kept, not the nearest; (circle, circle) always gives 2 solutions whatever the qualifier, tangent to C1 and perpendicular to the line of centres (they don't pass through TheCircle's centre); no BadQualifier check for noqualifier; ThisSolution throws Standard_RangeError | Unset qualifiers; wrong lines in the circle/circle case (pinned by a test) | fixed (decision 1544) |

## Phase 6: unit 6E, the pave filler

Found while porting BOPAlgo_PaveFiller, by reading OCCT's code. The port reproduces each (`// OCCT` at the line) except
where the entry says otherwise. None shows in the comparison with native (decision 785), where all 914 runs are
bit-identical.

| Where | What | Effect | Triage |
|---|---|---|---|
| `BOPAlgo_PaveFiller::PutPaveOnCurve` (PaveFiller_6), new-pave branch | Raises the tolerance of the vertex put on a section curve with `BRep_Builder::UpdateVertex`, without the `IsNewShape`/same-domain check `UpdateVertex(nV, tol)` makes in the non-destructive mode; `PutPavesOnCurve` lets the common vertices of the two faces through | In the non-destructive mode an argument vertex's tolerance can be raised in place. On a frozen argument the port throws (BOPAlgo_AlertIntersectionFailed, decision 787). Not reached by the recordings | touch |
| `BOPAlgo_PaveFiller::FilterPavesOnCurves` and `MakeBlocks` (PaveFiller_6) | Write the saved tolerances of the vertices of `theMVTol` straight into their TShapes (`BRep_TVertex::Tolerance`) | As above: the vertices of site 1 and the vertices of used paves, argument ones included | touch |
| `BOPAlgo_PaveFiller::CorrectToleranceOfSE` (PaveFiller_6), step 2.2 | Lowers the tolerance of vertices connected to section edges whose tolerance was reduced, whether or not the vertex is new | In the non-destructive mode an argument vertex can be changed (lowered) in place; the port throws on a frozen one. Not reached by the recordings | touch |
| `BOPAlgo_MPC::Perform` / file-static `UpdateVertices` (PaveFiller_7) | Raises the tolerances of the vertices of section edges in place, from parallel solvers, on vertices several section edges share; `BRep_TVertex::UpdateTolerance` is an unsynchronised read-compare-write | Natively a larger tolerance can be lost to a concurrent smaller one, so the parallel result can differ from the serial one. **Fixed** (decision 789): the port takes a lock around the update, which gives the serial result. In the non-destructive mode the vertex can be an argument's whose tolerance didn't grow before (the port throws on a frozen one, caught as BOPAlgo_AlertBuildingPCurveFailed) | settled |
| `BOPAlgo_PaveFiller::PutClosingPaveOnCurve` (PaveFiller_6) | Sets the index of a local copy of the pave (`aPave.SetIndex(nVn)`) that is never used again | No effect. Kept, not marked | leave |
| `BOPAlgo_PaveFiller::PerformVF` (PaveFiller_4) | Adds to the face's VerticesIn only the last same-domain vertex `nVx` of the loop over a pair's vertices | Possibly intended. Kept, not marked (matches native in every recorded case) | leave |

**Order sites** (decisions 580, 633, 784). The filler iterates integer maps (the faces' sub-shape and vertex sets,
`ShapesSD`, MakeBlocks' maps of vertices and their tolerances, the same-domain maps of MakeSDVerticesFF, PostTreatFF,
UpdateFaceInfo and UpdatePaveBlocks, ProcessExistingPaveBlocks' `aDMBV`) in OCCT's bucket order, which decides the
indices of new shapes; the port reproduces it with `NCollection_IntMap` and `NCollection_IntDataMap` (decision 784). It
iterates no map keyed by address, so native's filler is reproducible.
## Phase 5: unit 5O1, faces from wires

Found while porting BRepAlgo and the TopOpeBRepBuild/DS/Tool members `TopOpeBRepBuild_WireToFace::MakeFaces` runs.
Each is reproduced in the port and marked `// OCCT` unless the table says otherwise.

| Where | What | Effect | Triage |
|---|---|---|---|
| `BRepAlgo_FaceRestrictor::Perform`, `ChangePCurve` | Returns `C2.IsNull()` (true when the edge has *no* pcurve), which the caller stores as `YaPCurve` and tests `if (!YaPCurve)` | Without projection, an edge without a pcurve on the surface but with one elsewhere gets that pcurve moved onto the surface, then its 3D curve projected as well; without a 3D curve Perform returns not done, having written the pcurve into the caller's edge. Reproduced by the recordings (scenario 14: done 0, no face) | touch |
| `BRepAlgo_FaceRestrictor::PerformWithCorrection`, `IsInside` | Evaluates the first edge's pcurve on the face without checking it | Native crashes on an edge without a pcurve on the face (and not on a plane with a 3D curve); the port throws NullReferenceException | touch |
| `BRepAlgo_FaceRestrictor` | The default constructor leaves `myDone`, `modeProj` and `myCorrection` uninitialised | `IsDone` before `Perform` reads garbage; the port's are false | leave |
| `TopOpeBRepTool_2d.cxx`, `FC2D_FancestorE` | Without a 3D curve, looks the edge's face up in a map only `FC2D_Prepare` (the Booleans) creates, dereferencing a null pointer otherwise | Native crashes when WireToFace meets an edge with neither a pcurve on the face nor a 3D curve; the port throws Standard_NullObject (decision 792) | touch |
| `TopOpeBRepTool_2d.cxx`, the pcurve cache | Kept in `thread_local` statics that survive between calls; `FC2D_FancestorE` allocates the cache map before crashing | In a native process that ran a legacy Boolean on the thread, WireToFace may find pcurves cached by it. The port never has the cache (decision 792) | settled |
| `TopOpeBRepTool_ShapeClassifier` | The default constructor leaves `mySameDomain`, `mymren`, `mymredone`, `myState` and `myPAvLS` uninitialised; the shared classifier (`FSC_GetPSC`) is made with it | Its callers set them before reading; the port's are zero, false and null | leave |
| `TopOpeBRepBuild_WireEdgeSet::MakeNeighboursList` | `VertexConnectsEdges` leaves its orientations unwritten when the vertex isn't shared, and the caller reads `oVE` | Undefined on closing edges at a vertex they don't share; the port reads FORWARD | leave |
| `TopOpeBRepBuild_Merge.cxx`, `CorrectUnclosedWire` | Removes a wire's children while a TopoDS_Iterator is on them (the iterator's node is freed) | Undefined; in practice the next child is visited. The port iterates a snapshot. Only new wires (blocks of edges) get there, never WireToFace's | settled |
| `TopOpeBRepBuild_Builder::MakeFaces` | `myBuildTool.Closed(newWire, true)` on old wires too, and `AddFaceWire` | WireToFace flags the caller's wires closed (and not free). The port works on copies of frozen wires and gives the caller's back where nothing changed (decision 793) | settled |
| `BRepAlgo_Image::Image` | For a shape without images, appends it to a function-level `static` list and returns the list | The list keeps every shape so asked for: the second call returns two shapes. Shared between threads natively (racy); per thread in the port | touch |
| `BRepAlgo_AsDes::Ascendant`, `Descendant`, `ChangeDescendant` | Return a function-level `static` empty list for an unknown shape; `ChangeDescendant` hands it out writable | A caller filling it changes what every later call returns for unknown shapes. Per thread in the port | touch |
## Phase 5: unit 5K2, geometric constraints

Found while porting Geom2dGcc. Each is ported as OCCT has it and marked `// OCCT` at the line (58 sites). "Unset"
means an NCollection_Array1 element or a local OCCT never writes: uninitialised natively, 0 in the port (decision
781). The oracle (decision 802) masks or attributes every one whose native value depends on memory.

| Where | What | Effect | Triage |
|---|---|---|---|
| `Geom2dGcc_FunctionTanCuCuCu.cxx` | `Derivatives` and `Values` compute nothing and return true | Circ2d3TanIter solves with math_FunctionSetRoot, which calls `Values`: the search starts from the solver's own F and DF, uninitialised natively (stack and heap bytes), zeros in the port, which stops at the starting point. Native's results on free-form Circ2d3Tan (curve, curve, point) vary from run to run: 323 of 2,000 oracle cases differ, none repeating on a second native run | leave |
| `Geom2dGcc_FunctionTanCuCuOnCu.cxx` | The (circle, curve, centre on a circle) constructor sets `Geom2dGcc_CuCuOnCi`, not `CiCuOnCi` | `InitDerivative` evaluates the curveless `Curv1`: a null dereference natively, NullReferenceException in the port. Only the public `Geom2dGcc_Circ2d2TanOnIter(QualifiedCirc, QCurve, gp_Circ2d, ...)` reaches it; Geom2dGcc_Circ2d2TanOn's dispatch never does | touch |
| `Geom2dGcc_Circ2d2TanOn.cxx` (two curves, centre on a line or circle) | No `else`: the generic Iter on the OnCurve adaptor always runs after the line or circle Iter and replaces its result; the circle branch never calls WhichQualifier | The specialised Iter's work is discarded | touch |
| `Geom2dGcc_Circ2d2TanOn.cxx` (every Iter branch) | CenterOn3 isn't copied | `parcen3(1)` unset, `pntcen(1)` the origin (the oracle masks CenterOn3's parameter on these paths) | fixed (decision 1545) |
| `Geom2dGcc_Circ2d2TanOn.cxx` (curve, point, centre on a line or circle) | `TheSame2(1)` not set | Read by Tangency2 and IsTheSame2; unreachable in practice (the Iter throws, below) | fixed (decision 1545) |
| `Geom2dGcc_Circ2d2TanOnIter.cxx` (curve, point, on a line, circle or curve) | math_FunctionSetRoot is given 3-vectors for the 4-variable FunctionTanCuCuOnCu | Every `Geom2dGcc_Circ2d2TanOn(free-form curve, point, ...)` throws. Natively Standard_DimensionError: math_FunctionSetRoot.cxx defines `No_Standard_OutOfRange`, so SetTolerance reads past the vector unchecked and Perform rejects the length. In the port Standard_OutOfRange at SetTolerance (decision 803) | fixed (decision 1545) |
| `Geom2dGcc_Circ2d2TanOnIter.cxx` | Tangency1, Tangency2 and CenterOn3 return 0 for ParSol, ParArg and the centre's parameter instead of the stored values; IsTheSame2 always false | Through Geom2dGcc_Circ2d2TanOn, every Iter parameter is 0 | fixed (decision 1545) |
| `Geom2dGcc_Circ2d2TanOnIter.cxx` (circle, curve, line) | The centre is `ElCLib::Value(Ufirst(1), OnLine)`, Ufirst(1) being the parameter on C1 | Wrong centre (public constructor only) | fixed (decision 1545) |
| `Geom2dGcc_Circ2d2TanOnIter.cxx` (curve, curve, circle or curve) | `par1sol = 0`, `par2sol` the distance between the tangency points | Hidden by the accessors returning 0 | fixed (decision 1545) |
| `Geom2dGcc_Circ2d2TanOnIter.cxx` (point constructors) | `parcen3` is the radius `Ufirst(3)`; the line case stores the centre's line parameter as `pararg2` | Moot: these constructors throw | fixed (decision 1545) |
| `Geom2dGcc_Circ2d2TanOnGeo.cxx` (two circles, centre on a curve) | For an unqualified first circle `Rbid(1)` is overwritten and `Rbid(2)` never set; `RBid(2)` set only for an unqualified second circle; both read whenever nbsol is 2 | Native's radii depend on memory: 11 of 2,000 oracle cases differ (9 repeating natively) | leave |
| `Geom2dGcc_Circ2d2TanOnGeo.cxx` | `Radius(nnsol)` reaches 4 in an array of 2; the qualifier tests use `Radius(i)` with the bisector's index | Standard_OutOfRange, natively too | fixed (decision 1545) |
| `Geom2dGcc_Circ2d2TanRad.cxx`, `Results(GccAna_Circ2d2TanRad)` | Asks for Tangency1 and Tangency2 even when IsTheSame, where GccAna's accessors throw StdFail_NotDone | The constructor throws for any analytic case whose solution is an argument circle; with GccAna's unset `TheSame2(1)` (decision 781) it may throw natively only. No oracle case did | fixed (decision 1545) |
| `Geom2dGcc_Circ2d2TanRadGeo.cxx` (curve, curve) | `qualifier1(NbrSol)` assigned twice, `qualifier2` never | WhichQualifier's second qualifier unset when neither argument is a line or circle (not bound natively, so not compared) | touch |
| `Geom2dGcc_Circ2d3TanIter.cxx` | `IsTheSame3` always returns true once done; `IsTheSame2` reads TheSame3 | Every iterative Circ2d3Tan solution reports IsTheSame3 and its Tangency3 throws StdFail_NotDone (cirtang and callers asking for the third tangency of a free-form case get the exception natively too) | fixed (decision 1546) |
| `Geom2dGcc_Circ2d3TanIter.cxx` (curve, curve, point), (circle, circle, curve) | The second qualifier test checks Qualified1 again | Wrong qualifier filtering | fixed (decision 1546) |
| `Geom2dGcc_Circ2d3TanIter.cxx` (line, line, curve), (line, curve, point), (circle, line, curve) | The side of a line is `centre · normal` from the origin, not from a point of the line | Wrong qualifier filtering unless the line passes through the origin | fixed (decision 1546) |
| `Geom2dGcc_Circ2d3TanIter.cxx` (line, curve, point), (circle, curve, point) | Cu2 is evaluated at `Ufirst(2)` (the line's or the point-circle's parameter) instead of `Ufirst(3)`; the point-circle's tangent at `Ufirst(3)` | Wrong tangency checks (public constructors only) | fixed (decision 1546) |
| `Geom2dGcc_Circ2dTanCenGeo.cxx` | The `Index` array is never written, and `Index(1) == Index(2)` decides between one solution (the nearest extremum) and two (the farthest too) | Native has one or two solutions depending on memory (the port always one): 387 of 2,000 oracle cases differ, 251 repeating natively, native always the port's solutions plus others | touch |
| `Geom2dGcc_Circ2dTanOnRadGeo.cxx` (all six constructors) | `par1sol` is `ElCLib::Parameter(cirsol(n), pnttg1sol(n))` before `pnttg1sol(n)` is set | The parameter of the default point (0, 0); Standard_ConstructionError when the solution is centred at the origin | fixed (decision 1547) |
| `Geom2dGcc_Circ2dTanOnRadGeo.cxx` | `CenterOn3` returns `pnttg1sol`, not `pntcen3` | As GccAna_Circ2dTanOnRad's: a tangency point for the centre | fixed (decision 1547) |
| `Geom2dGcc_Circ2dTanOnRadGeo.cxx` (curve, centre on a curve) | `Adaptor2d_OffsetCurve(HOnCurv)` without an offset has the range [0, 0] | The curve to centre on is one point: no solution | fixed (decision 1547) |
| `Geom2dGcc_Circ2dTanOnRadGeo.cxx` (line, centre on a curve) | The offset line goes through `dir1.XY() + cote * norm1`, the direction, not the origin; `pararg1` is a parameter on that line evaluated on L1 | Wrong solutions unless the line passes through its direction vector | fixed (decision 1547) |
| `Geom2dGcc_Lin2dTanObl.cxx` (curve, line, param, angle) | Passes TolAng to the Iter as the start parameter and Param1 as the angular tolerance | Draw's `lintan` with an angle starts from TolAng | fixed (decision 1544) |
| `Geom2dGcc_Lin2dTanObl.cxx` (free-form curve) | `Add` calls the Iter's Intersection2, which throws Geom2dGcc_IsParallel when the angle is 0 or π | The constructor throws | fixed (decision 1544) |
| `Geom2dGcc_Lin2dTanObl.hxx` | `IsParallel2` declared, not defined (build123d.NET's binding excludes it) | Left out with its write-only `Paral2` | leave |

## Phase 5: unit 5L1, laws and trihedrons

Found while porting Law and GeomFill's trihedron and location laws. Each is ported as OCCT has it, except the array
bounds of Law_BSpline (decision 808).

| Where | What | Effect | Triage |
|---|---|---|---|
| `Law_BSpline::IncreaseMultiplicity` (both overloads), `IncrementMultiplicity` | Call `InsertKnots` with the default `Add = false`; Geom_BSplineCurve passes true | The multiplicity becomes Max(old, M - old) instead of M (or Max(old, Step) instead of old + Step), so `SetKnot(Index, K, M)` misses M. Kept, pinned by a test | **fixed** (decision 1813) |
| `Law_BSpline::Segment` | Locates the end at `NewU1 + (U2 - U1)`, which can round one ulp below the knot just inserted at U2; the strict `< Eps` test misses it | The segment ends at the previous knot and keeps poles of a span whose end knot is not of multiplicity Degree: a wrong curve, in a few percent of random segments. Kept, pinned by a test | **fixed** (decision 1813) |
| `Law_BSpline` constructors and `SetPeriodic` | OCCT 8's `NCollection_Array1::operator=` copies the source's bounds into the (1, n) arrays they allocate | Arrays given with other bounds, and the knots after `SetNotPeriodic` then `SetPeriodic` with FirstUKnotIndex > 1, keep their bounds; later 1-based reads are out of bounds (undefined behaviour). **The port copies into 1-based arrays (decision 808)** | touch |
| `Law_BSpline` (`Rational(W)`), `Law_Interpolate::BuildParameters` | Read W from index 1, and index the parameters by the points' index, whatever the arrays' lower bounds | Wrong only for arrays not indexed from 1. Kept | touch |
| `GeomFill_CorrectedFrenet::Copy` | Copies with the default constructor | The copy of a law made `ForEvaluation` searches the mean plane again (it is not "for evaluation"). Kept | touch |
| `GeomFill_CorrectedFrenet.cxx`, `smoothlaw` | After a successful first pass `Law = BS` shares the handle, and the second pass removes more knots from BS | The returned law is the second pass's even when that pass's control fails. Kept | touch |
| `GeomFill_CorrectedFrenet::InitInterval` | The next step is `max(\|S(t)\| / 2, LengthMin) / \|S'(t)\|` with S = C' ^ C'' (GeomFill_SnglrFunc), capped by the default step but not bounded below | On offset curves the steps fall near 1e-6, so one SetCurve takes hundreds of thousands of Frenet evaluations: seconds to over ten minutes natively on random offsets of B-splines and Bezier curves (the port, in a Debug build, about 30 times longer). Kept; the oracle leaves those spines out (decision 811) | fixed (decision 1440) |
| `GeomFill_CorrectedFrenet::Intervals` | No `return` after `EvolAroundT->Intervals(T, S)` when Frenet has one interval | The fusion that follows overwrites T with the same values. Harmless. Kept | leave |
| `GeomFill_GuideTrihedronAC::D2` | Uses `TN2 = \|TN\|^2` (= 1) where the second derivative of a unit vector needs `\|DTN\|^2` | D2BiNormal, and with it D2Tangent, are off by (\|DTN\|^2 - 1) BiNormal. Kept, pinned by a test | touch |
| `GeomFill_GuideTrihedronAC::D0/D1` | No guard against zero magnitudes (OCCT's own TODO "finding #8") | Division by zero where the path meets the guide. Kept | touch |
| `GeomFill_DiscreteTrihedron::Init` | Appends to myKnots and myTrihedrons without clearing them | A second SetCurve on a free-form curve keeps the first curve's samples. Kept | touch |
| `GeomFill_GuideTrihedronPlan` | Declares its own `myTrimmed`, hiding the base's, so the inherited `GetInterval` reads a null handle; `D2` computes only the path's Frenet frame and returns false | Access violation natively, NullReferenceException in the port; no second derivatives. Kept | touch |
| `GeomFill_LocationGuide::Copy` | Runs `Set(..., rotation, ...)` before the copy has a path; passes `myPoles2d(1,1).X()` (the first path parameter) as the start angle, as SetCurve does | Copying a law with a rotation throws Standard_ConstructionError. Kept, pinned by a test | touch |
| `GeomFill_LocationGuide::D2` | Multiplies the caller's incoming matrices by the transformation before setting them from the trihedron | D2 never applies SetTrsf, while D0 and D1 do. Kept, pinned by a test | touch |
| `GeomFill_LocationGuide::GetAverageLaw` | Averages myNbPts + 1 points with a step of a tenth of the interval | Most of the 22 points lie past the interval's end. Kept, pinned by a test | touch |
| `GeomFill_LocationGuide::SetRotation` | After the "rescue" solve `Pc` is still the default point | w = 0 is stored. By reading | touch |
| `GeomFill_FunctionDraft::DerivT` | Omits the w cos(a) B' term | LocationDraft's D1/D2 trace derivatives are wrong where the binormal turns (about 1.6 times the finite difference along a circle; exact along a line). Kept, pinned by a test | **fixed**, with `Deriv2T` and `DerivTX` (decision 1815) |
| `GeomFill_LocationDraft`, the seed search of D0/D1/D2 | For a parameter before the first milestone reads the poles at indices -1 and 0 | Out of bounds natively; Standard_OutOfRange in the port | touch |
| `GeomFill_CurveAndTrihedron`, `LocationDraft`, `LocationGuide::GetAverageLaw` | The matrix's columns are (T, N, B), where D0 to D2 give (N, B, T) | The average law is not the average of M(t). Kept, pinned by a test (CurveAndTrihedron) | touch |
| `GeomFill_UniformSection::BarycentreOfSurf` | Starts at the section's first parameter but steps to the converted B-spline's last parameter | A wrong sample range when the two differ. By reading | touch |
| `GeomFill_LocFunction::DN` | Any order other than 0 to 2 dereferences a null pointer | Access violation natively; Standard_OutOfRange in the port (tested) | touch |
## Phase 5: unit 5M, medial axis

Found while porting Bisector, MAT, MAT2d and BRepMAT2d. Each is ported as OCCT has it and marked `// OCCT` at the
line; the native recordings (tools/bisrec, tools/matrec, decision 823) agree with the port wherever they reach.
"Unset" means a local or field OCCT never writes on that path: uninitialised natively, 0 or false in the port.

| Where | What | Effect | Triage |
|---|---|---|---|
| `Bisector_Inter::Perform` | Each piece is stored in SBis/SD at its interval index, but the loop runs from 1 to the piece count | When an interval outside the domain is skipped, it reads an unset slot (a null handle) and misses the last piece | touch |
| `Bisector_Inter::Perform` | The second bisector's end extension is tested with `IB2 == Bis1->NbIntervals()` (Bis2 meant) | Wrong extension test when the bisectors' interval counts differ | touch |
| `Bisector_Inter::NeighbourPerform` | UMax and UMin come from `BisecCC::ValueAndDist`, which can leave them unset | Unset bounds (0 in the port) | leave |
| `Bisector_BisecPC(Cu, P, Side, UMin, UMax)` | Leaves shiftParameter, distMax, isEmpty and currentInterval uninitialised, then evaluates `Value(UMin)` with the unset shiftParameter | Uses 0 and false in the port | leave |
| `Bisector_BisecPC::Values` | No case for N = 3 | EvalD3 returns the given point rather than the bisector's point, with zero derivatives | touch |
| `Bisector_BisecPC::ComputeIntervals` | The periodic branch is dead (one interval appended at that point) and would call `Period()`, which throws for a non-periodic bisector | None reachable | leave |
| `Bisector_BisecCC::Values` | No `return` after the extension when the polygon has at most one point; it then compares with `First()` of an empty polygon | Compares with a default point of parameter 0 | touch |
| `Bisector_BisecCC` (TestExtension) | On the branch testing the curve's last point, T1 isn't normalised before `T1·T2 > 1 - Confusion` | The tangency test depends on T1's length | touch |
| `Bisector_BisecAna::Perform` | Can leave `thebisector` unset: the circle/circle, circle/line and circle/point cases when no solution is kept, the point/point case when `!ok && oncurve` | Bisector_Bisec dereferences it: a native crash, a NullReferenceException in the port | touch |
| `Bisector_BisecCC::ValueAndDist`, `Extension`, `PointByInt` | Can leave U1, U2, Distance (UOnB, Dist) unwritten (reached from SupLastParameter) | Unset outputs (`ref` in the port, decision 817) | touch |
| `Bisector_BisecCC::ChangeGuide` | Shares curve1 and curve2 with the original (not a bug) | Transforming one changes the other | leave |
| `MAT2d_Mat2d::CreateMat` | A face bounded by one closed edge (a full circle): the main loop never runs and the roots come from the empty list of a bisector never built; `roots->Current()` is a null handle | Native: access violation in `BRepMAT2d_BisectingLocus::Compute`. Port: NullReferenceException (marked in `MAT2d_Mat2d.Roots`) | touch (guarded, decision 1438) |
| `MAT2d_Tool2d::BisecFusion` | Assumes a bisector that isn't curve/curve is analytic: a Bisector_BisecPC gives a null `down_cast`, then `BAna->SetTrim` | Native crash, NullReferenceException in the port; reached when contours are merged, e.g. a holed face with Bézier sides (a matrec preset) | touch (decision 1441) |
| `BRepMAT2d_BisectingLocus::RenumerationAndFusion` | Reads `MergeArc1`, `MergeArc2` and `GeomIndexArc1..4`, which `MAT_Graph::FusionOfBasicElts` leaves unset when the two elements are one, and `MergeArc2` unset when the start and end arcs are one | Uninitialised or stale values; the port starts them at false/0 and keeps OCCT's reuse of stale values | touch |
| `MAT_Bisector`, `MAT2d_Mat2d` | MAT_Bisector's index, issue and end points, vectors, sense and distance, and Mat2d's `intersectionpoint`, uninitialised until set | Harmless (always set before read on the recorded paths); 0 in the port | leave |
| `MAT2d_CutCurve` | `Perform(C, aSide)` and `PerformInf` declared, never defined; `thenumberofedges` never read | None | leave |

## Phase 6: unit 6F, builders and Booleans

Found while porting the builders, by reading OCCT's code. The port reproduces each (`// OCCT` at the line). None shows
in the oracle (decision 835).

| Where | What | Effect | Triage |
|---|---|---|---|
| `BOPAlgo_BOP::BuildSolid` | Scans the face-to-solids map with `for (i = 1; i < aNb; ++i)`: the last face is never checked for being shared by two argument solids | A solid whose only face shared with another is that last one is wrongly taken as possibly untouched | fixed (decision 1428) |
| `BOPAlgo_Builder::BuildDraftFace` (Builder_2) | Changes the orientation of the edge splits stored in `myImages` in place (`ChangeValue` through a const reference) | Later readers of the images see each edge split's last orientation | touch |
| `BOPAlgo_Builder::FillInternalVertices` (Builder_2) | Adds an internal vertex into a face split, which can be another argument's own face kept as its same-domain group's face | In the non-destructive mode an argument face can be changed in place. On a frozen argument the port throws (BOPAlgo_AlertBuilderFailed, decision 829). Not reached by the oracle | touch |
| `BOPAlgo_Builder::FillInternalShapes` (Builder_3) | Adds internal vertices and edges into a split solid, which can be an argument's solid kept as the same-domain solid | As above, for solids | touch |
| `BOPAlgo_Tools::WiresToFaces` | Adds p-curves and corrects tolerances on the input's edges | The input changes; the port copies a frozen input at the entry (decision 830) | settled |
| `BOPAlgo_ToolsProvider::SetTools` | Clears `myTools` but not the fence `myMapTools` (as 6A's SetArguments) | Setting again tools added before leaves none | touch |

**Order sites (decision 580).** Two builder loops iterate a shape-keyed map, in address order natively and in insertion
order in the port: `FillSameDomainFaces` over `myImages` (to propagate each solid parent to its face pieces) and
`BOPAlgo_Tools::FillInternals` over the solids' internal faces. Neither order is observable: each piece is bound under
its own key, and each solid receives only its own shells. The builders' integer maps iterate in OCCT's order (decision
784), and no map keyed by a pave block or common block is iterated, so native's builders are reproducible: native gave
the same result serially and in parallel on every oracle case.

## Phase 5: unit 5L2, sections and sweep surfaces

Found while porting GeomFill's sweeps, sections and pipes. Each is ported as OCCT has it, except the in-place
reparametrisation of NSections' sections (decision 838); the native recordings (decision 840) reproduce the ones that
change a result.

| Where | What | Effect | Triage |
|---|---|---|---|
| `GeomFill_Pipe::Init(Path, FirstSect, LastSect)` | Treats `myLoc->SetCurve(myAdpPath)`'s false as a failure, but CorrectedFrenet's SetCurve returns whether the plain Frenet frame needs no correction | Pipes evolving between two sections fail with GeomFill_ImpossibleContact on most 3D paths and on planar paths with inflections (27 of 30 random 3D B-spline paths; `tuyau r path s1 s2`). Kept, pinned by a test and the recordings | fixed (decision 1436) |
| `GeomFill_SweepSectionGenerator::Perform` (types 1 to 3) | `TR = Rot * Trans` sits in the branch for opposite tangents, so TR stays the identity otherwise | The rotations and translations between successive sections are dropped: every section of a circle, constant or evolving section along a non-circular path stays at the first one's place. Kept (GeomFill_Pipe uses only type 0) | fixed (decision 1481) |
| `GeomFill_SweepSectionGenerator::GetShape` (type 0) | Gives the quasi-angular shape (7 poles, degree 6) whatever `Perform(Polynomial)` asked; `Section` then assigns the polynomial conversion's 8 poles | `GeomFill_Pipe::Perform(WithParameters, Polynomial = true)` on a pipe on an edge throws Standard_OutOfRange natively and in the port. Kept, pinned | touch |
| `GeomFill_QuasiAngularConvertor::Section` (second derivatives) | `D2Vw(5)` has `(b + 11)` where the derivative of `beta^4 (2c + b^2)` needs `(b + 1)` | The weights' second derivative, and with it D2Poles, are wrong in the quasi-angular conversion (D2 of circular blends). Kept | **fixed** (decision 1815) |
| `GeomFill::GetCircle` (second derivatives, rational branch) | `d2np2 = d2nplan ^ ns1 + nplan ^ dn2w`, where the second derivative of `nplan ^ ns1` needs `nplan ^ d2n1w` | Wrong second derivatives of the interior poles of arcs of more than one span. Kept | **fixed**, with a second `dn2w` for `d2n1w` in the same loop (decision 1815) |
| `GeomFill::GetCircle` (rational branch) | Builds the interior poles as `pts1 + R ((cos a - 1) ns1 + sin a (nplan ^ ns1))`, right for normals from the centre to the ends, while the header calls them inward normals ("Normal rentrente") and CircularBlendFunc passes inward ones | With inward normals, arcs of two or three spans (TgtThetaOver2_2/_3) get interior poles off the circle (radius 3.49 for 2 at 4.5 radians). CircularBlendFunc never asks for more than one span (it switches to quasi-angular above 0.65 pi). Kept; the test uses outward normals | touch |
| `GeomFill_NSections::ComputeSurface` | `SetKnots` on the caller's B-spline sections | Reparametrises the caller's curves in place. **The port reparametrises copies (decision 838)** | settled |
| `GeomFill_NSections::BarycentreOfSurf` | U is not reset for each V row | Only the first row samples the u range; the others evaluate past it. Kept | touch |
| `GeomFill_EvolvedSection::BarycentreOfSurf` | As UniformSection's: starts at the section's first parameter, steps to the converted B-spline's last | A wrong sample range when the two differ. By reading | touch |
| `GeomFill_Sweep::Build` | The call to `BuildProduct` (the approximation of the location law composed with the section's surface) is commented out, and BuildProduct's tensorial product too | `GeomFill_Location` approximates globally, as `GeomFill_Section` does. Kept | leave |

## Phase 5: unit 5L3, surfaces from curves

Found while porting GeomFill's filling half and AppBlend. Each is ported as OCCT has it, except the reversal of the
caller's curve in BSplineCurves and BezierCurves (decision 846). None was found in GordonUtilities, NetworkSurface or
Gordon.

| Where | What | Effect | Triage |
|---|---|---|---|
| `GeomFill_Coons::Init`, rational form | The poles put P2 in column 1 and P4 in column NU, the weights W4 in column 1 and W2 in column NU | Through BSplineCurves/BezierCurves, which pass (P1, P4, P3, P2, W1, W4, W3, W2), rational Coons fillings get the left and right boundary weights swapped. Kept, pinned by a test | fixed (decision 1479) |
| `GeomFill_CoonsAlgPatch::Value` | Evaluates bounds 0 and 2 at V instead of U | A wrong point; `D1U`, `D1V` and `DUV` are right, and ConstrainedFilling uses only those. Kept, pinned by a test | **fixed** (decision 1815) |
| `GeomFill_BSplineCurves::Init`, `GeomFill_BezierCurves::Init` (two curves, CurvedStyle) | When CC1's end is CC2's start, calls `C1->Reverse()` on the caller's curve, not on its copy | The result is built from the unreversed copy, so it is right, but the caller's curve is reversed (a second call sees it so). The port leaves the caller's curve alone (decision 846) | settled |
| `GeomFill_ConstrainedFilling::MatchKnots` | After SetDomain, the `ind[]` searches compare `dom` with the multiplicities `nm[]->Value(i)` instead of the knots `nk[]` | The knot is not found and the tangency mixing index keeps its default. Kept | touch |
| `GeomFill_ConstrainedFilling` | `stcor`, `tgalg` and `dom` are never reset by Init | A second Init on the same object starts sortbounds from the previous corner gaps and keeps tangency fields of bounds that no longer have normals. Kept | touch |
| `GeomFill_ConstrainedFilling::MatchKnots` | Reads `nbnp`/`nbnk` after `PrepareInsertKnots` returns false | Uninitialised natively; the port's out parameters are assigned | leave |
| `GeomFill_Profiler::UnifyByInsertingAllKnots` | Inserts every curve's knots into the others with `Add = false`; on periodic sections `BSplCLib::PrepareInsertKnots` mispredicts the pole count (Phase 1's entry above) | Natively `BSplCLib::InsertKnots` writes past its arrays: heap corruption, a crash in the recorder; the port throws Standard_RangeError. Reached by every Generator, SectionGenerator and loft whose sections are all periodic B-splines with different knots | fixed (decision 1439) |
| `GeomFill_Profiler::Perform` | When knot insertion does not unify the curves, `UnifyBySettingMiddleKnots` only moves knots | The curves keep different numbers of poles; `Poles`, GeomFill_Generator and AppBlend_AppSurf then read and write past arrays sized for the first curve natively (unchecked) where the port throws. Seen with all-periodic sections | fixed (decision 1439) |
| `GeomFill_SectionGenerator::SetParam` | Keeps the caller's array, then writes `myParams(i) = Params(Lower() + i - 1)` into it for i from 1 | For an array not indexed from 1 the caller's values are shifted and the last write is past the end (unchecked natively; Standard_OutOfRange in the port, tested) | touch |
| `GeomFill_SectionGenerator` constructor | Sets default parameters only if `mySequence` has curves, which it never has there | `Parameter` without `SetParam` dereferences a null handle (an access violation natively, NullReferenceException in the port) | touch |
| `AppBlend_AppSurf::InternalPerform`, spline approximation | `if (C0) SetContinuity(0); if (C1) ... else if (C2) ... else SetContinuity(3)` | GeomAbs_C0 ends as continuity 3. Kept (recorded) | touch |
| `AppBlend_AppSurf::InternalPerform`, Bezier pieces (not SpApprox) | Computes the tolerances reached but never stores them | `TolReached` and `TolCurveOnSurf` give the previous run's values (0 at first). Kept | touch |
| `AppBlend_AppSurf` (both Perform) | The two-section rule (IFV occ13904) writes `dmin = 1` and possibly `dmax = 1` into the object's settings | They stay for the next pieces of `Perform(..., NbMaxP)` and for later runs: a piece of two sections lowers every later piece's degrees, and later pieces can fail (StdFail_NotDone). Kept (recorded) | touch |
| `AppBlend_AppSurf::Perform(..., NbMaxP)`, known parameters | Measures each Bezier piece by `mucu.Pole(Index, 1)` to `Pole(Index, NbPoles)`, Index being the piece's number | The distances, hence the interior knots, come from the wrong curve of the multi-curve. Kept | touch |
| `AppBlend_AppSurf::Curves2dShape` | `NbPoles = tabPoles->ColLength()`, the number of U poles | The 2d curves have the V poles. Kept | touch |
| `AppDef_Variational::InitCriterionEstimations` through `AppBlend_AppSurf::PerformSmoothing` | The entry above (EstTangent past a two-point line) | Smoothing two sections: native reads past the points and throws Standard_OutOfRange later; the port's range check throws Standard_RangeError first (decision 849) | touch |
## Phase 5: unit 5R, 2D fillets and chamfers

ChFi2d and BRepFilletAPI_MakeFillet2d, reproduced as OCCT has them (decision 10), each marked `// OCCT`. None made
the port differ from native in the oracle or the recordings (decision 856).

| Where | What | Effect | Triage |
|---|---|---|---|
| `ChFi2d_Builder.cxx`, `BuildNewWire` | Every edge of the face being built goes into one new wire, closed as the face's first wire | A face with holes comes back with one wire holding the outer and inner edges (build123d.NET chamfers each wire on its own for this reason) | fixed (decision 1474) |
| `BRepFilletAPI_MakeFillet2d.cxx`, `Modified` | `DescendantEdge` reads the history with `Find` | Standard_NoSuchObject for any edge MakeFillet2d didn't trim, where BRepBuilderAPI_MakeShape's contract is an empty list | touch |
| `ChFi2d_Builder.cxx`, `BuildFilletEdge` | `vec1` built from the vertices and overwritten at once; `Ebid1`/`Ebid2` built and unused; the basis curves down-cast to Geom2d_Line unchecked | Dead code; a curve neither line nor circle would be a null dereference (IsLineOrCircle rejects it first) | leave |
| `ChFi2d_Builder_0.cxx`, `ComputePoint(F, L, E, Param)` | `Param` unset and the point at the plane's origin when the chamfer's line meets nothing | An angle chamfer that misses its second edge ends at the origin of the plane with an uninitialised parameter (0 in the port) | touch |
| `ChFi2d_Builder_0.cxx`, `ModifyChamfer(Chamfer, E1, E2, D1, D2)` | `E1` unused | Only `E2` orients the new chamfer | touch |
| `ChFi2d_AnaFilletAlgo.cxx`, `Init(wire)` | `e1` and `e2` are only set when null | A second Init (ChFi2d_FilletAPI reuses its algorithm) keeps the first wire's edges | touch |
| `ChFi2d_AnaFilletAlgo.cxx`, `isCW` | The middle point is evaluated on the edge's curve without the edge's location, the ends with it | Wrong orientation for an arc edge with a location | fixed (decision 1472) |
| `ChFi2d_AnaFilletAlgo.cxx`, `Perform` | The right neighbour's ends are chosen by `e1.Orientation()` | Wrong trimming when the two edges have different orientations | fixed (decision 1473) |
| `ChFi2d_AnaFilletAlgo.cxx`, `ArcFilletSegment` | Compares a squared distance difference with `SquareConfusion`, where SegmentFilletArc uses `Confusion` | The arc's end point is found less often; the parameter path is used instead | touch |
| `ChFi2d_FilletAlgo.cxx`, `IsRadiusIntersected` | `LowerDistanceParameter()` (a parameter) compared with `Precision::Confusion()` as if it were a distance | Intersections are judged on the edge by their parameter | touch |
| `ChFi2d_FilletAlgo.cxx`, `FilletPoint::Copy`, `FilterPoints` | `Copy` doesn't copy `myNear`; FilterPoints removes `myV(b)` with `b` an index into `myNear` | Values and their near indices can fall out of step | touch |
## Phase 5: unit 5Q, fair curves

Found while porting FairCurve. Each is ported as OCCT has it and marked `// OCCT` at the line; the native recordings
(tools/fairrec, decision 862) agree with the port bit for bit.

| Where | What | Effect | Triage |
|---|---|---|---|
| `FairCurve_EnergyOfBatten::Compute`, `FairCurve_EnergyOfMVC::Compute` | `MyBattenLaw.SetSliding(MyLengthSliding)` updates the energy's own copy of the law, which nothing reads; the distributions keep their copies with the initial sliding | With free sliding and a non-zero slope, the height law `H(t) = h + (t - 0.5) slope L` keeps the initial L while the sliding changes. Kept | touch |
| `FairCurve_Energy::Hessian1` | Sets the terms between the first end's and the second end's constraint variables to 0 ("correct if there are less than 3 nodes") | With the battens' single-span degree 9 curves every pole interacts, so those hessian terms are wrong (finite differences give 1e-3 to 2e-2); the Newton steps are approximate, the minimum is the same. Kept, pinned by a test | touch |
| `FairCurve_Energy::Value` | Reads `Energie(0)` of a fresh `math_Vector(0, 0)` that Compute leaves unwritten when it stops on an infinite sliding | Uninitialised natively, 0 in the port; math_NewtonMinimum's step halving and its step reduction (which ignores the return value) read it. No case of the recordings differs. Kept | leave |
| `FairCurve_EnergyOfBatten::Compute` | When a sagging integration fails, goes on to the next interval and returns that interval's result | With more than 12 poles, a failure on one interval can be lost. By reading | touch |

## Phase 5: unit 5P, plates and filling

Found while porting Plate, NLPlate, GeomPlate, LocalAnalysis and BRepFill_Filling. Each is ported as OCCT has it,
except the two of decision 869 and the evaluation cache of decision 870, which change nothing native computes
reliably.

| Where | What | Effect | Triage |
|---|---|---|---|
| `NLPlate_NLPlate::Solve`, `Solve2` | After the loop over constraint orders, `OK = true` overwrites the `OK = false` an iteration's failure set | `IsDone()` is true even when no plate was solved. Kept, pinned by a test (three constraints at one UV: every plate singular, the surface still the initial plane) | touch |
| `NLPlate_NLPlate::Solve2` | Solves its first, polynomial iteration with `ord`, not the `ordre` it raises to `maxOrder + 2` | Possibly intended. Kept | leave |
| `Plate_Plate::Init` | Leaves `OK = true` | An empty plate, and a GeomPlate_BuildPlateSurface with no constraint, report `IsDone()`. OCCT's own GTests pin it with a TODO (fixing it broke blend and filling Draw tests). Kept | touch |
| `Plate_GtoCConstraint` (G3 constructor with a direction) | `nP` is unused (its name commented out in the signature) | Equal to the constructor without it. Kept, pinned by a test | leave |
| `Plate_LinearXYZConstraint`, `Plate_LinearScalarConstraint` (from arrays) | OCCT 8's `operator=` copies the source's bounds | A pinpoint array not indexed from 1 makes Plate's 1-based reads go out of range: Standard_OutOfRange on both sides. No caller passes one | leave |
| `LocalAnalysis_CurveContinuity`, `LocalAnalysis_SurfaceContinuity` | `myErrorStatus` is never set on success; the tolerance-only surface constructor leaves `myTypeCont` unset until `ComputeAnalysis` | `StatusError()` after a successful analysis and `ContinuityStatus()` before one read indeterminate memory; the port returns `LocalAnalysis_NullFirstDerivative` and `GeomAbs_C0`. No OCCT caller reads them then | leave |
| `GeomPlate_BuildPlateSurface::Perform` | When a curve can't be projected on the initial surface, approximates `myGeomPlateSurface`, which Perform nulled at its start (ComputeSurfInit sets it only on its non-linear branch) | Access violation natively (the corpus's bug23343, an OCCT TODO); the port throws Standard_NullObject (decision 869) | touch |
| `GeomPlate_BuildPlateSurface::G0Error(Index)`, `G1Error(Index)`, `G2Error(Index)` | Arrays of NbPtsOnCur values, of which EcartContraintesMil writes one per interval of the curve's last discretisation | More intervals (MakeFilling's usual case once Perform raised the points): Standard_OutOfRange on both sides. Fewer: native reads uninitialised memory, and returns values that change from call to call (seen on the oracle's 4- to 7-edge contours, several per test run); the port's arrays are zeroed (decision 869) | touch |
| `GeomPlate_BuildPlateSurface::G0Error()`, `G1Error()`, `G2Error()` | Set only by VerifSurface, which runs only with curve constraints | With point constraints only, native returns uninitialised values; the port 0. BRepFill_Filling always has curves | leave |
| `GeomPlate_MakeApprox` (criterion order other than -1, 0, 1) | No branch approximates | A null surface, errors unset (the port's 0). Kept | touch |
| `GeomPlate_Surface::UPeriod`, `VPeriod` | Return `false` | 0. Harmless (never periodic) | leave |
| `BRepFill_Filling::AddConstraints` (through MakeFilling's `Add(edge, face, order)`) | Passes the GeomAbs_Shape value as the plate's order (0 G0, 1 G1, 2 G2) | GeomAbs_C1 (2) gives a G2 constraint, and GeomAbs_G2 (3) throws "BRepFill : The continuity is not G0 G1 or G2" (Standard_Failure, on both sides: 10 oracle cases). build123d.NET's `MakeSurfacePatch` maps its C2 level to G2, so it always fails natively | fixed (decision 1434) |
| `BRepFill_Filling::Build` | Adds the bounds it finds for free constraints to `myBoundary`, and keeps `myOldNewMap` from build to build | A second Build rebuilds each boundary edge from the first build's edge (its empty copy keeps the pcurve on the first surface) and keeps the free constraints' edges. Kept; the oracle compares second builds | touch |
| `BRepFill_Filling::G0Error()` and the others before `Build` | Dereference the null builder | Access violation natively; NullReferenceException in the port | touch |
| `tests/bugs/filling/bug23380` | Draw reads `i` and `f` (faces) as free constraints; the case expects 4 edges | Native OCCT 8.0.1, through build123d.NET's binding, gives 3 edges, as the port does (byte-identical face): the case fails in Draw-lite, and natively too unless DRAWEXE reads its words otherwise | leave |
## Phase 6: unit 6P, BRepAlgoAPI and the public API

Found by OCCT's GTests and the Boolean oracle. The port reproduces each, or says where it can't (`// OCCT` or
`// OCCT.NET: frozen` at the line).

| Where | What | Effect | Triage |
|---|---|---|---|
| `BOPAlgo_PaveFiller::MakeBlocks` (PaveFiller_6, "back to previous tolerance values for unused vertices") | Writes each saved tolerance back into the vertex's TShape, also when it is the vertex's own (a vertex only used as an existing pave) and also for an argument's vertex in the non-destructive mode. `UpdateSavedTolerance` can raise a saved value to a section edge's tolerance, and then the write grows an argument vertex in place | The no-op write is skipped in the port (it would throw on a frozen vertex: GTest `bcut_simple/M2`, a cut of a cut result by its own argument, failed with BOPAlgo_AlertIntersectionFailed). A real change of a frozen argument vertex still throws there (not seen; decision 877) | settled |
| `BRepTools_History::Merge` | Appends the composed images from a hashed `NCollection_Map` (`add(aL12, aAdditions[aI])`), so the order of a merged Modified or Generated list is the map's address order natively, insertion order in the port | `SimplifyResult`'s merged history lists the same images as native's, in another order, on 37 of the oracle's 393 simplified Booleans (and on different cases from one native run to the next); the sets are equal | leave |
| `BRepAlgoAPI_BuilderAlgo::SimplifyResult` | Runs UnifySameDomain with `SetSafeInputMode(myNonDestructive)`: in OCCT's default (destructive) mode the unifier writes into the result in place, and the result shares the arguments' untouched sub-shapes, so the caller's arguments change (pcurves added, ranges reset) | Native writes into its arguments; the port's are frozen and the unifier copies at its entry (decision 766), so they never change. Results are the same bytes; the history has decision 643's in-place entries where native has none | settled |
| OCCT's destructive mode (`SetNonDestructive(false)`, Draw's default) | The pave filler grows tolerances and adds pcurves on the arguments' own sub-shapes, which the result then shares | The port's frozen arguments always run non-destructively (decision 577); where native's destructive result differs, native's non-destructive result is the port's (oracle) | settled |

## Phase 6: unit 6G, checks and the other builders

Found by the native recordings of tools/borec and the unit tests. The port reproduces each, or says where it can't
(`// OCCT` or `// OCCT.NET: frozen` at the line).

| Where | What | Effect | Triage |
|---|---|---|---|
| `BOPAlgo_CheckerSI` (through PaveFiller_6's `PutPaveOnCurve` and `MakeBlocks`) | The self-check sets the non-destructive mode, but the filler still raises the tolerance of argument vertices a face/face section curve passes through (`BRep_Builder::UpdateVertex`), and MakeBlocks writes saved values back | Native `bopcheck`/`bopargcheck` can leave their input with raised vertex tolerances (bottle, hammer, CrankArm against a moved copy). The port checks a copy of a frozen argument (decision 884) | settled |
| `BOPAlgo_PaveFiller::FilterPavesOnCurves` (as MakeBlocks, unit 6P) | Writes a vertex tolerance even when it equals the current value, for an argument vertex used as an existing pave (every self-check of a box) | A no-op natively; skipped in the port, where it would throw on a frozen vertex (decision 885) | settled |
| `bopapicheck` (BOPTest_CheckCommands.cxx) | `-op` as the last word reads `a[n]`, a null pointer, through `strcmp` | DRAWEXE crashes; Draw-lite treats the missing operation as unknown | leave |
| `BOPAlgo_MakePeriodic::SetXTrimmed/SetYTrimmed/SetZTrimmed` | `(bool theIsTrimmed, const bool theFirst = 0.0)` declares the period's start as a `bool` | The start can only be 0 or 1; reproduced with a bool parameter | touch |
| `BOPAlgo_MakePeriodic::UpdateTwins` | Iterates `myRepeatedTwins` (shape-keyed: address order natively) and skips a group once any member is done; twins aren't transitive | The twin groups after a repetition depend on the iteration order; native isn't reproducible there. The port iterates in insertion order | leave |
| `BOPAlgo_MakePeriodic` trim and repetition, `BOPAlgo_MakeConnected::MakePeriodic` | The trim's Common and the repetition's gluing run destructively | Natively they change the caller's input, the periodic shape returned before and the connected shape in place (pcurves added to shared edges). The port copies frozen parts first (decision 891) | settled |
| `BOPAlgo_RemoveFeatures::RemoveInternalWires` | Removes INTERNAL wires and edges from the rebuilt solids' faces in place (`aF.Free(true); BRep_Builder::Remove`, through a `const_cast` of the explorer's face) | When the face is an untouched input face shared into the result, native changes the caller's input. The port copies (decision 892) | settled |

## Phase 5: unit 5S1, sweep laws and sections

Found by the native recordings of tools/brepfillrec and tools/cwrec, the oracle and reading. The port reproduces each
(`// OCCT` at the line).

| Where | What | Effect | Triage |
|---|---|---|---|
| `BRepFill_LocationLaw::Abscissa` | When the lengths aren't computed yet it calls `CurvilinearBounds(Index, bid, Length)` and starts from the edge's last bound instead of its first | The first call on a fresh law is too long by the edge's own length (a 4 + 3 polyline: `Abscissa(2, mid)` gives 8.5 the first time, 5.5 afterwards); reproduced natively, pinned by a test | touch |
| `BRepFill_ACRLaw` constructor | `OrigParam` is indexed over all the wire's edges, `ipath` counts only the non-degenerated ones | After a degenerated spine edge each edge gets an earlier edge's origin (found by reading, not reproduced) | settled |
| `BRepFill_LocationLaw::IsG1`, closed-path branch | The `D1` on law 1 isn't assigned to `Ok_D1`, unlike the inner branch | A failed evaluation there goes unseen | touch |
| `BRepFill_CompatibleWires` SearchRoot (from SameNumberByPolarMethod) | Returns the first key of RMap or MapVLV, iterated in TShape-address order, whose list holds the vertex | With looping sections (the first repeated last) native's result varies from run to run (five runs, five outputs: status Done or Failed, cuts moved); the port iterates in binding order and is deterministic | settled |
| `BRepFill_CompatibleWires::SearchOrigin`, `SameNumberByACR` | `newwire.Oriented(TopAbs_FORWARD);` discards its result; `double AllLengthsNull = true` holds a bool | No effect | leave |
| `BRepFill::Face`, `BRepFill::Shell`, `BRepFill_Pipe` | Write pcurves, ranges, flags and tolerances into the caller's edges and profile | Natively the caller's shapes change; the port writes into copies of frozen inputs (decisions 897, 910) | settled |

## Phase 5: unit 5O2, the 2D wire offset

Found by the MakeOffset oracle, tools/evorec and reading. The port reproduces each (`// OCCT` at the line), unless the
last column says otherwise.

| Where | What | Effect | Triage |
|---|---|---|---|
| `BRepFill_OffsetWire::GeneratedShapes` | Merges the lists of `myMap` by iterating `myMapSpine`, a map keyed by shapes and hashed by TShape address | The shapes in a generated list come in an address-dependent order natively; the port iterates in insertion order. MakeOffset's `Generated` held the same shapes in another order in 74 of 1,124 oracle results | leave |
| `BRepFill_OffsetWire` constructors | Leave `myCallGen` uninitialised; only `Perform` sets it, and `GeneratedShapes` reads it | After `PerformWithBiLo` alone native reads garbage; harmless since `myMapSpine` is then empty. The port starts it false | leave |
| `BRepFill_TrimSurfaceTool::IntersectWith` | `SetTrim` on the trimmed bisector it is given, which is the medial axis's own curve | The locus's bisector stays lengthened by the tolerance at both ends for every later call | touch |
| `BRepFill_Evolved` (`ElementaryPerform`, `PrepareProfile`, `Add`, `Transfert`) | Iterates `myMap` and `MapVevo`, keyed by shapes, in TShape-address order | Natively the parallel edges are built, and the result's faces listed, in an address-dependent order; the port uses insertion order. The bindings don't depend on it | leave |
| `BRepOffsetAPI_MakeOffset::BuildDomains` (via BRepFill_OffsetWire and the medial axis) | Bisector_BisecAna can leave its curve unset (5M) and Bisector_Bisec dereferences it, on some arc profiles with intersection joins | An access violation: with OCCT's signal translator (DRAWEXE, build123d.NET) a Standard_Failure caught as "not done"; without it a crash. The port: a NullReferenceException caught as "not done" (decision 917) | touch (decision 1441) |
| `BRepOffsetAPI_MakeOffset::BuildDomains` | `double Found = false` holds a bool | No effect | leave |

## Phase 5: unit 5S2, pipe shells

Found by the MakePipeShell oracle, tools/piperec and reading. The port reproduces each (`// OCCT` at the line), unless
the last column says otherwise.

| Where | What | Effect | Triage |
|---|---|---|---|
| MakePipeShell with an auxiliary spine and `BRepFill_Contact` on polyline spines (the site not isolated) | Dereferences a null handle | An access violation natively; Standard_NullObject in the port (4 oracle cases, decision 930) | touch |
| `BRepFill_Pipe::Edge` (through `BRepOffsetAPI_MakePipe::Generated(spine edge, profile vertex)`) | Reads `myEdges`, which is null when the profile is a vertex | An access violation natively (OSD_Exception_ACCESS_VIOLATION under OSD::SetSignal, 18 of tools/piperec's cases); NullReferenceException in the port | fixed (decision 1435) |
| `BRepFill_PipeShell::Set(bool IsFrenet)` | Both branches set `myTrihedron = GeomFill_IsFrenet`; the corrected Frenet one is labelled Frenet | No effect seen (`myTrihedron` only chooses the continuity for the discrete trihedron and the contact laws) | leave |
| `BRepFill_PipeShell::Prepare` | Clears `WSeq` and `myEdgeNewEdges` but not `myIndOfSec` | After a second Prepare (a section added or removed after a build) the history reads the first build's section indices | touch |
| `BRepFill_PipeShell::BuildHistory` | The loop looking for the section's inde-th edge tests `Degenerated(anEdge)` (the section's edge) instead of `theEdge` | Degenerated edges of the placed wire are counted; no effect on the oracle's sections | leave |
| `BRepFill_PipeShell` constructor | `mySpine.Closed(true)` on the caller's wire when its ends meet | Natively the caller's spine changes; the port closes a new wire of the caller's edges (decision 926) | settled |
| `BRepFill_PipeShell::MakeSolid` | Adds the end faces to the shell `Shape()` already returned | Natively the shape the caller holds changes; the port adds them to a new shell of the same faces when the shell was published (decision 926) | settled |
| `BRepFill_Sweep` BuildFace | `BRepLib_FindSurface` can return the plane of a face already built (the four edges' pcurves lie on it), and BuildFace calls `UReverse` on it | The other face's surface is reversed too, natively. **Fixed** in the port: it reverses a copy (decision 929); the results differ from native's there | settled |
| `BRepFill_Sweep::BuildWire` | With a last bound, stores `FirstShape` (a wire) at column NbPath of the vertex table instead of the last vertex | Unreachable: the pipes give no bounds to a vertex's sweep | leave |
| `BRepFill_Sweep::PerformCorner`, tangent case | Evaluates the next law's frame at its last parameter instead of its first | The corner of a tangent but twisted junction is placed with the next edge's end frame | touch |
| `BRepFill_Sweep::Build` | Substitutes the edges of `myVEdgesModified`, a map keyed by shapes, in TShape-address order; the first substitution of a shared vertex wins | Natively the order can change which new vertex replaces a shared one; the port iterates in insertion order. Not observed in the oracle | leave |
| `BRepFill_TrimShellCorner::HasSection` | `myHasSection` is set nowhere | Always false; the sweep's fallback for a failed right corner depends only on the transition | touch |
| `BRepOffsetAPI_MiddlePath::Build` | Writes "Interpolation failed" to `std::cout` when GeomAPI_Interpolate fails, then calls `Curve()` | StdFail_NotDone on both sides; the port doesn't print | leave |

## Phase 5: unit 5T, lofts and projection

Found by the ThruSections and BRepProj_Projection oracles and reading. The port reproduces each (`// OCCT` at the
line), unless the last column says otherwise.

| Where | What | Effect | Triage |
|---|---|---|---|
| `BRepFill_Generator::Perform` | `anOrEdge1`/`anOrEdge2` are references to the wire explorers' current edge; after `ex1.Next()` they name the next edge (a null edge after the last), and `myMap` is keyed by it | `GeneratedShapes(E)` gives the face built on the edge before E; the last face is keyed by a null edge. ThruSections doesn't read this map. Reproduced (decision 936) | touch |
| `BRepOffsetAPI_ThruSections::Init` | Sets the maximal degree to 6 where the constructor sets 8 | A loft re-initialised approximates with a lower degree. Reproduced | touch |
| `BRepOffsetAPI_ThruSections::SetCriteriumWeight` | A negative weight sets the status to Failed; `Build` resets the status | No effect but a transient status. Reproduced | leave |
| `BRepOffsetAPI_ThruSections` (mutable input, the default) and `BRepFill_Generator` | Write pcurves, ranges, flags and tolerances into the caller's section edges and vertices | Natively the caller's sections change. The port copies frozen sections at Build and gives back the unchanged sub-shapes (decisions 936, 937) | settled |
| `BRepFill_CompatibleWires` through ThruSections | Maps keyed by shapes iterated in TShape-address order | Native's status and result vary from run to run on lofts whose first section is also the last, and rarely otherwise (1 of 630 random cases); the port is deterministic (decision 940) | settled |
| `BRepProj_Projection::BuildSection` | A compound target's faces go into a new, unlocked compound, so the section runs destructively and writes pcurves into the caller's target edges | Natively the target changes. On frozen targets the port's section ran destructively into frozen edges and failed (IntersectionFailed); it now runs non-destructively when the target holds frozen shapes (`// OCCT.NET`, decision 939) | settled |

## Phases 5 and 6: integration

Found by full Oracle runs (test classes in parallel).

| Where | What | Effect | Triage |
|---|---|---|---|
| `FEmTool_LinearFlexion`, `FEmTool_LinearJerk` and `FEmTool_LinearTension` constructors (FEmTool_Linear*.cxx) | Keep the reference integrals of the last constraint order in function statics `Order` and `MatrixElemts`, unsynchronised; `Order` is set before `MatrixElemts` is recomputed | Concurrent AppDef_Variational runs with different continuities (GeomAPI_PointsToBSpline and PointsToBSplineSurface with weights, ThruSections with smoothing) read the other order's integrals: 2 of 32 C2 approximations run beside C1 ones gave other poles (1e-4 relative), one of them the value a full Oracle run failed on. The port caches per order under a lock (decision 148); the oracle serialises these native calls (decision 945) | settled |
## Phase 6: unit 6M, build123d.NET's Booleans

Found by replaying build123d.NET's own tests' operations. The Booleans and cleans showed no new issue: all 497 calls
are byte-identical with native, and native repeated itself on every one.

| Where | What | Effect | Triage |
|---|---|---|---|
| `BRepProj_Projection::BuildSection` (through build123d.NET's `Project` and `ProjectToShape`) | Never sets the section non-destructive, so any unlocked target, not only the compound of 5T's entry, is sectioned destructively | Natively the caller's target changes: in build123d.NET's ProjectEdge case a box edge on the projection cylinder's seam had its tolerance raised (1e-7 to 1.00000000666e-07) and a pcurve added. The port's frozen targets keep the section non-destructive (decision 939), so its split edge is a copy whose vertices come in the other order: canonically identical, byte-identical on unfrozen operands (decision 953) | settled |
## Phase 6: unit 6L, Draw-lite Booleans and healing

Found by porting BOPTest's and SWDRAW's commands and running the corpus. Draw-lite reproduces each (`// OCCT` at the
line), unless the last column says otherwise.

| Where | What | Effect | Triage |
|---|---|---|---|
| BOPTest_TolerCommands `ReduceFaceTolerance` (`btolx`) | The edge map isn't cleared between faces | A face's tolerance is the smallest of its own edges' and every earlier face's edges' | leave |
| BOPTest_TolerCommands `breducetolerance` | The usage message says `bupdatetolerance` | Text only | leave |
| BOPTest_DebugCommands `bopfsd` | Adds the face itself to the compound, not its same-domain face | `f_sd` holds copies of `f`; the count is right | leave |
| BOPTest_DebugCommands `bopindex` | Index 0 (the first argument) counts as "not found" (`ind > 0`) | " DS does not contain the shape" for the first argument | leave |
| BOPTest_DebugCommands `bopwho`, `bopsd`, `bopfon`/`bopfin`/`bopfsc`, `bopfav` | The range check `ind > NbShapes()` (or `NbSourceShapes()`) lets the one-past-the-end index through | Natively reads past the table; in Draw-lite the index check of the port's table throws, and the case logs the exception | leave |
| SWDRAW_ShapeAnalysis `fbclose` | Reads `a[3]` for splitclosed and `a[4]` for splitopen (off by one) | The options shift | leave |
| SWDRAW_ShapeAnalysis `statshape` | `strstr("bigspl", arg3)`: the option matches as a substring of the keyword | An empty or one-letter word turns modes on | leave |
| SWDRAW_ShapeAnalysis `fbprops` | Labels the open wires "closed wires"; its usage message says `freeprops` | Text only | leave |
| SWDRAW `settolerance` (argc equal to premarg), `samerange` (prints `argv[2]` with 2 words), `fastsewing -tol` without a value | Read past `argv` | Natively undefined; Draw-lite reads an empty word | leave |
| SWDRAW_ShapeFix `fixshape +?` | Lists its message counts in NCollection_DataMap hash order | Draw-lite lists them in first-seen order | leave |
| SWDRAW_ShapeAnalysis `fbprops` on a 10×10 square | `1-16k` comes out marginally negative, so ratio and width print 0 (the formula gives 1 and 10) | Reproduced; not checked against native | leave |
| bugs/heal/bug29382_3 (UnifySameDomain with `+b` on a Bézier prism's common with a box) | The script expects 5 vertices | The port and native OCCT (build123d.NET's build, on the port's input and on its own Boolean) both give 6; the case fails on both | leave |
## Phase 5: unit 5W, Draw-lite construction

Found by reading Draw's command sources while porting them. Draw-lite reproduces each, unless the last column says
otherwise.

| Where | What | Effect | Triage |
|---|---|---|---|
| `qcircle`, `qline` (GeometryTest_CurveTanCommands.cxx) | The help gives the qualifier as `-enclosing`, `-enclosed`...; `GccEnt::PositionFromString` matches only the names without the dash | A qualifier written as the help shows is ignored: the curve is unqualified. Reproduced | leave |
| `geompipe` (BRepTest_SweepCommands.cxx) | The first edge (`spineedge`) is used as the guide and the second (`profileedge`) as the path; the circle's frame is taken on the path at the guide's first parameter (`ProfileCurve->D1(aSpFirst, ...)`) | The arguments mean the opposite of their names, and the section is misplaced when the two edges' ranges differ. Reproduced | leave |
| `mksweep` | Accepts 2 or 5 words, so `mksweep wire -C` without tolerances (the help's `[-C [AngTol LinTol]]`) fails with no message | Only `mksweep wire -C AngTol LinTol` corrects the spine. Reproduced | leave |
| `errorsweep`, `deletesweep` | Dereference the sweep maker without checking that `mksweep` made one | An access violation natively; a .NET NullReferenceException (a Tcl error) in Draw-lite | leave |
| `tanginterpol` | Checks `n < 3 * nbpoints + num_read` where reading the points needs one word more | With exactly one word too few the last coordinate is read from `argv[argc]`, a null pointer. Draw-lite throws (a Tcl error) | leave |
| `thrusections` | The `Null3DCurve` status message lacks the final newline the others have | Cosmetic. Reproduced | leave |

## Phase 7: unit 7A, ShapeCustom

Found by porting ShapeCustom and comparing it with native OCCT (decisions 973 to 978). The port reproduces each
(`// OCCT` at the line), unless the last column says otherwise.

| Where | What | Effect | Triage |
|---|---|---|---|
| ShapeCustom_SweptToElementary `NewCurve2d` | Calls `BRep_Tool::Surface(NewF, L)` on the null face BRepTools_Modifier passes from `CreateOtherVertices` (to ask whether an edge's vertices must be copied), which reaches it for edges without a new 3D curve (degenerated edges of revolutions) | An access violation natively (`OSD_Exception_ACCESS_VIOLATION` under the signal handler) on 16 of the heal group's and data/occ's shapes: `ShapeCustom::SweptToElementary` fails on any shape with such a face. The port throws a NullReferenceException there (decision 977) | fixed (decision 1437) |
| ShapeCustom_SweptToElementary `NewCurve2d` | Dereferences `SS` (null) when the face's surface isn't swept and only the edge was copied, if the new face is a torus or a sphere | An access violation natively; the port skipped the shift (`is` on null) until unit 7C, and now throws a NullReferenceException there (decision 1024): ShapeProcess's SweptToElementary, BRepTools_Modifier in its mutable-input mode, meets it on three heal inputs | fixed (decision 1437) |
| ShapeCustom_SweptToElementary `IsToConvert` | For a trimmed or offset surface, tests `S->IsKind(Geom_SweptSurface)` (known false) instead of its basis `Stmp` | Trimmed and offset surfaces of revolution or extrusion are never converted | touch |
| ShapeCustom_Surface `ConvertToAnalytical` | `if (substitute && !NHS.IsNull()) Init(newSurf)`: the adaptor is never null, so a surface rejected by the final check (null) replaces the tool's surface | With `substitute`, a failed conversion leaves the tool with no surface; a later `ConvertToPeriodic` returns null | touch |
| ShapeCustom_Surface `ConvertToAnalytical` | Computes the orientation check with `iso->D1(0., ...)` on the last iso-curve built (V at the middle), at parameter 0 rather than U1 | Reproduced | touch |
| ShapeCustom_Curve, ShapeCustom_Surface `ConvertToPeriodic` | Make the given B-spline periodic in place when its knots needn't be rearranged | Changes the caller's (shared) geometry; the port changes a copy (decision 974) | settled |
| ShapeCustom_Curve2d `SimplifyBSpline2d` | Removes knots from the caller's curve in place | As above; the port copies a frozen curve (decision 974) | settled |
| ShapeCustom_BSplineRestriction `ConvertCurve2d` | For an offset pcurve whose basis converts to C0, converts the curve into `ResCurve`, not `C`, and returns true | The caller's C is left as it was (null in NewCurve2d), and the modifier stores a null pcurve. Not met by the tests | touch |
| ShapeCustom_BSplineRestriction `ConvertCurve`, `ConvertCurve2d` | The `continue` meant to restart with the global limits (`aC = aC1`) is inside the inner `for (j = 1; j <= 2; j++)` loop: it retries once (ConvertCurve2d with the continuity computed before `aC = aC1`), and when the retry fails too and the curve isn't to be copied anyway, falls through to `myConvert = true; return true` | A failed approximation is returned as the converted curve, with its error as the tolerance. Reproduced | touch |
| ShapeCustom_BSplineRestriction `IsConvertCurve3d` / `IsConvertCurve2d` | A 3D B-spline is converted from `NbKnots - 1 >= NbSeg` spans, a 2D one from `> NbSeg` | Reproduced | touch |
| ShapeCustom_BSplineRestriction `NewPoint` | Answers from `myConvert`, the state of the last face or edge converted before the call; BRepTools_Modifier asks for points first | The vertices of a shape get new points only when the same modification converted something before (a compound's later children) | touch |
| ShapeCustom_RestrictionParameters | `ConvertToroidalSurf` returns the cylinders' field and `ConvertCylindricalSurf` the tori's | Harmless: each accessor always uses the same field | leave |
| BRepTools_Modifier `SetShapeFlags(S, S)` (reached by every ShapeCustom function) | Copies the flags of a sub-shape left alone onto itself; `Modified(true)` clears its Checked bit | Writes into the caller's shape natively; the port's frozen input keeps its flags (decision 347), the oracles mask the Checked column | settled |
| ShapeExtend::Init with Message_MsgFile::Msg | Init loads SHAPE.us only when `ShapeFix.FixSmallSolid.MSG0` is missing, and `Msg` stores an unknown keyword's error text under the keyword: a ShapeFix_FixSmallSolid warning sent before Init (ShapeAlgo::Init or ShapeProcess) leaves every other healing message unknown for the rest of the process | Reproduced; the message tests load the texts themselves, since the tests share one process | leave |
## Phase 7: unit 7W, Draw-lite's cheap gaps

Found by investigating the four cases decision 972 made runnable and by porting Draw's commands. Draw-lite reproduces
each (`// OCCT` at the line where it is code), unless the last column says otherwise.

| Where | What | Effect | Triage |
|---|---|---|---|
| bugs/modalg_7/bug26034 (`bopcheck` after `ttranslate Revolve002 0 0 -0.5`) | IntPatch_ImpImpIntersection of the sphere and a cone reaches `Standard_NumericError_Raise_if` in math_FunctionAllRoots (IntStart_SearchOnBoundaries::BoundedArc); BOPAlgo_CheckerSI turns the exception into the warning "Intersection of pair of shapes has failed", which the script reads as a fault | OCCT's default Release build compiles the check out (`No_Exception`), so OCCT's CI passes. build123d.NET's native build keeps the check, as the port does (decision 9): tools/cksirec (native BOPAlgo_CheckerSI) gives the same warning at -0.5 and none at 0, +0.5 and -0.25, as the port. The case fails on both | leave |
| `insertpole` (GeomliteTest_CurveCommands.cxx) | Returns 1 below 6 words, so the 2D form `insertpole name index x y` can't be given; with 6 words the 2D form takes `a[5]` as the weight | A 2D pole is always inserted with a weight. Reproduced | leave |
| `dmode`, `discr`, `defle` (DrawTrSurf.cxx) | With names, every iteration sets the drawable of `a[1]`, not `a[i]`; `dmode` with one word sets the uniform mode whatever the word | Display only: Draw-lite keeps the modal values and changes no drawable | leave |
| `lastrep` (Draw_VariableCommands.cxx) | With Z, divides the pick by `dout.Zoom`, which is `false` (0) in batch mode | X, Y and Z are NaN when no view exists. Reproduced | leave |
| `crvpoints`, `crvtpoints` (GeometryTest_CurveCommands.cxx) | Read `a[2]` and `a[3]` without checking the word count | Natively undefined; Draw-lite reads an empty word | leave |
| `2dintersect` (GeomliteTest_API2dCommands.cxx) | `-tol` as the last word reads `a[++i]` past the end; the output spells "parameter on the fist" | Draw-lite reads an empty word (0); the text is reproduced | leave |
| `xbounds`, `removeinternals` (BRepTest_OtherCommands.cxx, DBRep.cxx) | Messages without a space: " shape<name> must be a face", "<name>is a null shape" | Text only. Reproduced | leave |
| `XProgress` | Sets Draw_ProgressIndicator's process-wide defaults; the corpus's progress cases read the indicator's output in the commands' results | Draw-lite keeps the modes per interpreter and its commands make no progress indicator (decision 954), so those cases fail (decisions 984, 988) | leave |
| BRepPrimAPI_MakePrism's history on a sphere band from 0 to 2π (bugs/modalg_7/bug30346_2, `prism` then `savehistory`) | The port's IsDeleted says one edge of the base is deleted; native (build123d.NET's MakePrism on the same face) says none | Not OCCT's: the port's copy for write of the generating shape, changed in place after the sweep (decision 981). **Fixed** by unit 9F (decision 1307); the case passes | settled |

## Phase 7: unit 7W's follow-up, the Boolean corpus in full

| Where | What | Effect | Triage |
|---|---|---|---|
| bugs/modalg_7/bug83 (`bsection` of two planar B-spline sheets from BUC60912_sec_slow.brep) | Not OCCT's: a cold start, 0.41 s and 0.43 s. Warm the port takes about 6 ms and 60 ms, native about 16 ms and 55 ms, on a loaded machine (unit 11G, decision 1723; it was 0.20 s against 0.03 s at phase 7) | The case's 0.1 s limit fails, on the JIT, not a result difference | leave |

## Phase 7: unit 7D, helix (TKHelix)

Found by porting TKHelix and BRepTest_HelixCommands.cxx. The port reproduces each (`// OCCT` at the line where it is
code), unless the last column says otherwise.

| Where | What | Effect | Triage |
|---|---|---|---|
| HelixGeom_HelixCurve `Load` | `myTgBeta` is set only when \|taper\| > 1e-4, never reset: reloading a tapered adaptor with a (near) zero taper keeps the old taper | The second helix is tapered. Not reached by TKHelix's builders, which load a new adaptor each time. Reproduced, and pinned by a test | touch |
| HelixBRep_BuilderHelix `SetParameters` (spiral, by heights) | Reads `theHeights(1)` where the loop uses `theHeights.Lower()` | An array not starting at 1 is read at the wrong place or throws. Reproduced | touch |
| HelixBRep_BuilderHelix (class documentation) | Error 13 is documented for "TaperAngle < 0 or > π/2 - TolAng"; `BuildPart` checks only the upper bound, and returns 3 (undocumented) when an edge can't be made | Negative tapers (diameters decreasing) are built, as the corpus's spirals need | leave |
| HelixBRep_BuilderHelix `SmoothingEdges` | Changes the edges' curves in place; when no degree lets the tangent move, the curve keeps the raised degree though `UpdateEdge` isn't called | Native shares no curve with a caller, so harmless; the port changes copies (decision 994) | settled |
| HelixGeom_Tools `HelixGeom_Tools_Eval::Evaluate` | Sets error code 1 for a dimension other than 3, then writes three values anyway | Never reached (the dimension is always 3). Reproduced | leave |
| `helix2`, `spiral2` (BRepTest_HelixCommands.cxx) | The usage text for a wrong argument count names H and PF arguments the commands don't take | Text only. Reproduced | leave |
| helix/standard/D6, D7 | `CheckSpiral` compares lprops' centre and moments as strings, centre coordinates near 1e-15 included | OCCT's own TODO (OCC11111) on all platforms; BAD in Draw-lite as in DRAWEXE | leave |
## Phase 7: unit 7B, ShapeUpgrade and ShapeAlgo

The port reproduces each, unless the last column says otherwise.

| Where | What | Effect | Triage |
|---|---|---|---|
| ShapeUpgrade_SplitSurfaceContinuity `Compute` | For a surface of linear extrusion whose basis curve had knots removed, `SetBasisCurve` on the surface it was given: the face's own surface (for a trimmed one, the basis shared by its faces) | Changes the caller's geometry (heal's BUC60173.rle with C2 criteria); the port changes a copy (decision 998), the one attributed oracle difference | settled |
| ShapeUpgrade_SplitSurfaceContinuity `Compute` | For a surface of revolution, the knots removed from the basis curve aren't set into the surface (unlike the extrusion); DONE3 is reported anyway | Reproduced | touch |
| ShapeUpgrade::C0BSplineToSequenceOfC1BSplineCurve | `SetNotPeriodic` on the caller's curve | The port splits a copy of a frozen curve (decision 998) | settled |
| ShapeUpgrade_ShapeConvertToBezier `Perform` | Segments the edges' Bezier pcurves in place; where a seam's two pcurves are one curve, it is segmented twice (an arc ending at π/4 gets 0.49, data/occ's nuts and many heal inputs in curves-only mode) | Reproduced on copies (decision 998) | touch |
| ShapeUpgrade_FixSmallBezierCurves `Approx` | For the seam's second pcurve, compares the bounds with the first pcurve's range and lowers `First` with `if (First > f)` | Reproduced | touch |
| ShapeUpgrade_FixSmallBezierCurves `Approx` | Re-initializes the wire divider's own split tools, which WireDivide is still using | Reproduced | touch |
| ShapeUpgrade_ShapeDivide `Perform` | Dereferences the split face tool for the free wires without checking it | Reproduced (NullReferenceException) | touch |
| ShapeUpgrade_ShellSewing `ApplySewing` | Documented to return the input when no shell is sewn; returns a null shape | Reproduced | touch |
| ShapeUpgrade_ShapeDivideClosedEdges `SetNbSplitPoints` | Ignores the number | Reproduced | touch |
| ShapeUpgrade_SplitCurve2dContinuity `Compute` | After removing a knot completely, doesn't lower the last index as the 3D tool does | Reproduced | touch |

## Phase 7: unit 7E, the offset foundations

Found by porting BRepOffset (but MakeOffset) and ChFi3d.cxx. The port reproduces each (`// OCCT` at the line), unless
the last column says otherwise.

| Where | What | Effect | Triage |
|---|---|---|---|
| BRepOffset_Tool `IsOnSurface` (:577) | The sampling step is `(f - l)`, negative | Samples outside the curve's range. Reproduced | touch |
| BRepOffset_Tool `AngleWireEdge` (:931) | `Angle` is uninitialised when the edges aren't B-splines | Undefined natively; the port uses 0 | leave |
| BRepOffset_Tool `PutInBounds` (:113), `BuildPCurves` (:397) | The bounds are uninitialised when `FindPeriod` writes nothing; the pcurve can be null when `PutInBounds` dereferences it | Undefined natively; the port uses 0, and throws on the null curve | touch |
| BRepOffset_Tool `CheckBounds` (:3146) | `L.First()` on an empty list | Undefined in native release builds; the port throws Standard_NoSuchObject | touch |
| BRepOffset_Tool `Inter3D`, the closed-wire branch | Could loop forever if the edges don't chain | Looks unreachable (a built wire's Closed flag is never set). Reproduced | leave |
| BRepOffset_Inter2d `RefEdgeInter` ("keep Vmin") | `Remove` moves the iterators on, then the loop's `Next()` skips an element | A vertex after a removed one is kept. Reproduced | touch |
| BRepOffset_Inter2d `ExtentEdge` | `MinSurf` is replaced by a transformed copy, so `theSurf == MinSurf` never holds | The pcurve the 3D curve was built from is checked (and may be re-projected) too. Reproduced | touch |
| BRepOffset_MakeLoops (three places) | `NewTV->ChangePoints().Append(TV->ChangePoints())` moves the list | The replaced vertex loses its point representations, which matters only if it survives outside the images. Reproduced | touch |
| BRepOffset_Inter3d `ContextIntByInt` | `LOE` and `MV` are built and never used; on the vertex path `Generated(E)` gets a null edge | Harmless. Reproduced | leave |
| BRepOffset_SimpleOffset `FillVertexData` | A vertex with no offset edge divides by zero | Its new point is NaN. Reproduced | touch |
| BRepOffset_MakeSimpleOffset `GetSafeOffset` | Divides by `myMaxAngle`, which is 0 for a smooth shape | Infinite or NaN. Reproduced | touch |
| BRepOffset_MakeSimpleOffset `BuildMissingWalls` | `IsClosed` on a null shell when the quilt makes none (an edge in solid mode) | An access violation natively; Standard_NullObject in the port (decision 1010) | touch |
| BRepOffset_MakeSimpleOffset, solid mode | Writes the walls' pcurves into the input's free edges and raises their vertices' tolerances | A second run on the same input gives another result natively (decision 1010). The port copies at the entry (decision 1006), so the input is never written | settled |
## Phase 7: unit 7H, LocOpe

Found by porting LocOpe and recording it natively (tools/locoperec). The port reproduces each, unless the last column
says otherwise.

| Where | What | Effect | Triage |
|---|---|---|---|
| LocOpe_SplitShape, WiresOnShape, Spliter, Generator, Gluer, SplitDrafts, the sweeps | Write into the shapes they are given: vertices' point representations and tolerances, edges' pcurves and continuities, the split shape's faces (plan 2.2) | The caller's shapes change. The port works on copies of frozen shapes and gives back the unchanged ones (decisions 1013, 1014) | settled |
| LocOpe_SplitDrafts `Perform` | Builds a GeomFill_Pipe on the two drafted planes' intersection, an infinite Geom_Line, which GeomConvert can't convert ("No such curve") | Throws Standard_DomainError whenever the planes meet (natively on all 229 such recorded cases; the 133 others aren't done): the class can't draft a plane. Reproduced | touch |
| LocOpe_SplitDrafts `Perform` | After the right edge is built, tests the left one (`NewEdgg`) again; the tolerance compared is NewEdgg's twice | Reproduced | touch |
| LocOpe_DPrism (through BRepFill_Evolved) | The order of an edge's two pcurves in the result, and the last bits of the faces' areas, follow addresses | Native's BRep text varies run to run; the oracle compares the shapes canonically (decision 1017) | leave |
| LocOpe_CSIntersector `LocalizeBefore(I, FromInd, ...)` | No index version of `LocBefore` exists: the index converts to the double parameter `From` | Searches before the parameter equal to the index. Reproduced | touch |
| LocOpe_WiresOnShape `Add` | Puts each argument's box at the argument's index but its edges at the next free indices; an argument with several edges (a wire) leaves boxes and edges out of step | Edges tested against another edge's box. Reproduced | touch |
| LocOpe_WiresOnShape `Project` | The V resolution is computed with `UResolution` | Reproduced | touch |
| LocOpe_WiresOnShape `PutPCurve` | The branch for internal vertices (`!V1.IsNull() && V2.IsNull()`) reads the tolerance and point of the null V2 | Throws when reached. Reproduced | touch |
| LocOpe_FindEdges `Set` | Compares B-spline knots with `Kf(k) - Kt(k) > Tol`, not in absolute value | Knots smaller than the other's pass. Reproduced | touch |
| LocOpe_Pipe | Corrects the orientation of a fused planar face's normal with the reference plane's `Direct()`, not the face's own plane's | Reproduced | touch |
| LocOpe_Pipe `Curves`, `BarycCurve` | A straight spine's pipe lines are infinite lines, which GeomConvert can't convert; `BarycCurve` with no curve appends a null curve and goes on to read the first | Throws Standard_DomainError / Standard_OutOfRange. Reproduced | touch |
| LocOpe_Generator `Perform` | Computes whether a face's wire touches the removed edges (`isIncludedInW`) and doesn't use it | Reproduced | leave |
| LocOpe_Gluer `AddEdges` | Computes, for each face of the result, the new shape's edges touching it, and keeps nothing | Reproduced (for its cost) | leave |
| LocOpe_Revol, LocOpe_RevolutionForm | Declare constructors taking the base and the axis without defining them; RevolutionForm never sets its rotation (`myIsTrans` stays false) | The constructors are left out (decision 1015) | leave |
| LocOpe_LinearForm | Stores the two points it is given and never reads them | Reproduced | leave |
| LocOpe_Spliter `Perform` | Builds the left part from maps keyed by addresses | `Left()`'s order varies natively; the oracle compares it as a set | leave |
## Phase 7: unit 7C, ShapeProcess and the healing Draw commands

Found by porting ShapeProcess, ShapeProcessAPI, `Resource_Manager` and SWDRAW's ShapeUpgrade, ShapeCustom and
ShapeProcess commands. The port reproduces each, unless the last column says otherwise.

| Where | What | Effect | Triage |
|---|---|---|---|
| ShapeProcess_OperLibrary `Init` | Sets its static `done` flag before registering the operators, without a lock | A second thread calling `Init` meanwhile returns at once and may find no operator. The port registers under a lock (decision 1021) | settled |
| ShapeProcess `RegisterOperator`, `FindOperator` | The static operator map has no lock | Registering while another thread performs can corrupt it. The port takes a lock (decision 1021) | settled |
| ShapeProcess_Context `LoadResourceManager` | `stat` of a file that doesn't exist isn't checked: the modification time is read from an uninitialised buffer; the cache compares the name only, not the directories | A missing file may look modified. The port takes 0 for a missing file and keys the cache by the files too (decision 1023) | settled |
| ShapeProcess_Context `GetReal`, `GetInteger` | Read the first character of a value that isn't a number (`Value(1)`), which throws Standard_OutOfRange for an empty value | The exception leaves the operator; the performer reports "failed with exception". Reproduced | touch |
| Resource_Manager (constructors) | The user resource file is loaded into the reference map (`Load(aUserPath, myRefMap)`), so the "User file loaded" trace can't be printed and `GetMap(false)` holds only what `SetResource` set | Reproduced | touch |
| ShapeProcess `settol` (SetTolerance), `sameparam` (SameParameter) | Change tolerances and pcurves in place and record nothing in the context ("WARNING: no update of context yet!") | The history has no entry for them. Reproduced; in the port the result holds changed copies (decision 1022) | touch |
| BRepLib `UpdateTolerances` (in place, from SetTolerance) | Sets the tolerances computed per (sub-shape, location) by iterating a map keyed by shape in hash order (TShape addresses); a TShape the shape holds at two locations, with different tolerances at each, ends with whichever OCCT iterates last | The port iterates in insertion order: 28 of the oracle's heal inputs (instances) get the other tolerance on a few vertices and edges, attributed | leave |
| ShapeProcessAPI_ApplySequence `PrepareShape`, `ClearMap` | `fillmap` and `until` are ignored; `ClearMap` does nothing | Reproduced | touch |
| SWDRAW `DT_SplitCurve`, `DT_SplitCurve2d`, `DT_SplitSurface` | Read `argv[argc]` (a null pointer) when the optional split flag is missing | Draw-lite takes a missing flag as 0 | leave |
| SWDRAW `splitface` | Prints ", " before every U value (`i > j` where `j > 1` is meant) | Reproduced | leave |
| SWDRAW `expshape` | Counts a seam twice per face | Reproduced | leave |
| SWDRAW `offset2dcurve` | On a C0 curve, Geom2d_OffsetCurve throws Standard_ConstructionError ("Offset on C0 curve") | Reproduced | leave |

## Phase 7: unit 7F, the 3D offset

Found by porting BRepOffset_MakeOffset and the offset API, and by its oracle. The port reproduces each (`// OCCT` at
the line), unless the last column says otherwise.

| Where | What | Effect | Triage |
|---|---|---|---|
| BRepOffset_MakeOffset `TrimEdges` | `if (theNewEdges.Add(NE))` tests the index NCollection_IndexedMap::Add returns, never 0 | An edge already trimmed is trimmed again. Reproduced | touch |
| BRepOffset_BuildOffsetFaces `BuildSplitsOfFaces` | In the artificial case, `*pLFIm = aLFImages` when the face has no splits yet (`pLFIm` null) | An access violation natively; a NullReferenceException in the port. Not met by the oracle | touch |
| BRepOffset_MakeOffset (MakeLoops on the closing faces, `MakeMissingWalls`, `UpdateTolerance` with `Locked(false)`) | Writes into its input: new vertices on the closing faces' edges, the walls' pcurves on a thickened shell's edges, tolerances | Native changed its own input in 82 of the oracle's 724 runs; a second run on it differs. The port copies at the entry (decision 1030) | settled |
| BRepOffset_MakeOffset, the Arc join with Intersection on | Iterates shape-keyed maps by address (decision 1032) | The result's text varies from run to run natively (about 40 of the oracle's runs on primitives). The port's order is insertion order | leave |
| BRepOffset_MakeOffset, BRepOffsetAPI_MakeThickSolid | `IsDone()` with a null `Shape()` (open shells with nothing left to offset, a hollow sphere) | Reproduced (16 oracle runs); build123d.NET checks for it | touch |
| BSplCLib `AntiBoorScheme` through `RemoveKnot` (GeomLib::ExtendSurfByLength, the offset's face extension) | Steps outside its NCollection_LocalArray buffers, reading and writing the stack around them | Undefined natively (the tolerance test then fails); the port's `RemoveKnot` reports the removal impossible instead of throwing (decision 1034) | settled |
| BRepOffset_MakeOffset `SelfInter` | Throws Standard_NotImplemented | Never called. Reproduced | leave |

## Phase 7: unit 7J, the blend functions

Found by porting Blend, BlendFunc and ChFiDS and by tools/blendrec. The port reproduces each (`// OCCT` at the line).

| Where | What | Effect | Triage |
|---|---|---|---|
| BlendFunc_ConstRad, BlendFunc_EvolRad `ComputeValues` | The scratch values (the guide's point and derivatives, the surfaces' third derivatives) are `static thread_local`, shared by every instance on a thread, while the cache guards are per instance | An instance whose parameter is unchanged can read the guide point another instance computed last. Reproduced with `[ThreadStatic]` (decision 1037) | touch |
| BlendFunc_ConstRadInv `Derivatives` | `dnplan.SetLinearForm(theD, nplan, d2cur)` where `Values` has `-nplan.Dot(d2cur)` | The derivatives in the guide's parameter (rows 2 to 4) differ from `Values`' and from finite differences off the origin (decision 1038) | touch |
| BlendFunc_EvolRad `ComputeValues` | The order-2 positioning doesn't recompute `dns1u1`, `dns1v1`, `dns1u2`, `dns1v2` (ConstRad's does) | The second derivatives use the last order-1 values (decision 1039) | touch |
| BlendFunc_ConstThroatInv, ConstThroatWithPenetrationInv `Derivatives` | `temp1`, `temp2`, `tempmid`/`temp3` are computed from `pts1`, `pts2` before `D1` updates them | They are the previous call's points (the same point when `Values` calls `Value` first) | touch |
| BlendFunc_ChAsym `Section` (first order) | Writes `PointOnS1()`/`PointOnS2()` into the poles before computing the points | The poles are the previous computation's points | touch |
| BlendFunc_ChAsymInv `ComputeValues` | On the first surface's restriction, `DX(4,2)` uses `dnplan.Dot(tempVec)` with `tempVec = 2 * nps1` | Disagrees with finite differences on a curved guide (2 against 0 on the plane/cylinder case) | touch |
| BlendFunc_CSCircular `GetTolerance` | Passes the signed radius to GeomFill::GetTolerance (CSConstRad passes its absolute value) | A negative radius makes gp_Circ throw Standard_ConstructionError | touch |
| BlendFunc_EvolRad `GetTolerance` | `lengthmin / maxang` before any solution (both still at RealLast/RealFirst) gives a negative radius | gp_Circ throws Standard_ConstructionError | touch |
| BlendFunc_Chamfer, ConstThroat, ChAsym, Ruled (GenChamfer) `GetTolerance` | Leave `Tol1D` unset | The caller's vector keeps whatever it held (uninitialised memory natively) | leave |
| BlendFunc `Knots`, `Mults` | Declared, never defined | Left out | leave |
| ChFiDS_Stripe `Reset` | `pcrv1.Nullify()` twice | The last pcurve is kept | touch |
| ChFiDS_FilSpine `SetRadius(Law, IinC)` | Builds a composite law from the law, drops it, and clears the radii | The law is lost | fixed (decision 1433) |
| Blend_CurvPointFuncInv `NbVariables` | Documented as 3, returns 2 | Reproduced | leave |
## Phase 7: unit 7G, drafts and the rest of TKOffset

Found by porting Draft, BRepOffsetAPI's draft and projection makers, BRepFill_Draft, BiTgte, BRepFill_AdvancedEvolved
and their Draw commands, and by their oracle and recordings. The port reproduces each (`// OCCT` at the line), unless
the last column says otherwise.

| Where | What | Effect | Triage |
|---|---|---|---|
| Draft_Modification `Remove` | A face whose `Add` failed was never added, so `Remove` throws Standard_NoSuchObject, though the documentation says to call it after a failed `Add` | Reproduced; `Clear` resets | touch |
| Draft_Modification `NewCurve2d` | Translates by a period the pcurve it returns, which may be the one it keeps for the edge | A second call for the same edge and face would translate it again. The port translates a frozen one on a copy (decision 1044) | settled |
| Draft_Modification `NewParameter` | `GC = down_cast<Geom_TrimmedCurve>(GC)` keeps the trimmed curve (its basis was meant) | Reproduced | touch |
| Draft_Modification (some drafts of several faces) | Dereferences a null handle | An access violation natively (24 oracle runs), a NullReferenceException in the port | touch (decision 1441) |
| BRepOffsetAPI_DraftAngle `CorrectWires` | Frees and edits the modifier's result, which shares the input's unchanged sub-shapes | Native writes into its input (324 of the oracle's 870 runs changed native's input); build123d.NET copies ExtrudeTaper's profile for it. The port copies the result first (decision 1044) | settled |
| BRepAlgo_NormalProjection `Build` | The edges the face bounds trim (BRepAlgoAPI_Section) are added to the result but not to the list `Generated` returns | `Generated` is empty for them. Reproduced | touch |
| BRepOffsetAPI_FindContigousEdges `NbEdges` | Declared, never defined | Left out | leave |
| BRepFill_Draft `Generated` | For a vertex, `TopoDS::Edge` throws; the vertex branch runs only for a null shape, whose comparisons fail | Never answers for a vertex. Reproduced | touch |
| BiTgte_Blend `ContactType` | Two faces give BiTgte_FaceEdge, never BiTgte_FaceFace | Reproduced | touch |
| BiTgte_Blend `ContactType` (`SupportShape1`) | Throws Standard_NoSuchObject on some center lines (the recording's Boolean result and bug29861's cylinders) | Reproduced (17 recorded runs) | touch |
| BRepFill_AdvancedEvolved `GetSpineAndProfile` | Evaluates both tangents with the first edge's adaptor (`anAC1.D1(aPar2, ...)`), and takes `anExtr.Value()` for every solution | Reproduced | touch |
| BRepFill_AdvancedEvolved `RemoveExcessSolids` | `TopoDS::Solid(anExpSo.Current())` after the explorer passed its only solid: the explored compound, cast unchecked in release builds | The result is the compound. The port keeps it without the cast (decision 1047) | settled |
| BRepTest `offsetload`, `offsetonface`, `rollingball` | Read words past the arguments (`a[2]` with two words, `a[i + 1]` for an odd count, `a[2]`, `a[3]`, `a[n]`) | Draw-lite reads empty words | leave |
## Phase 7: unit 7I, BRepFeat

Found by porting BRepFeat and the feature Draw commands, and by their oracle. The port reproduces each (`// OCCT` at
the line), unless the last column says otherwise.

| Where | What | Effect | Triage |
|---|---|---|---|
| BRepFeat_MakePrism, MakeDPrism, MakeRevol | Each declares its own private `myStatusError`, hiding BRepFeat_Form's; `CurrentStatusError` (the form's, not virtual) never reads it | A status the maker sets (IncTypes, NoIntersectU/F, NullToolU/F, InvFirstShape, IncParameter...) is never reported; a construction that never reaches GlobalPerform (a prism of a length without glued faces) reports NotInitialized. Reproduced (the oracle's statuses agree) | touch |
| BRepFeat_MakePipe | Declares its own `mySkface`, hiding the form's, which stays null | Reproduced | touch |
| BRepFeat_MakeRevolutionForm | Declares its own `mySliding`, hiding BRepFeat_RibSlot's, which stays false | Reproduced | touch |
| BRepFeat_MakePrism, MakeDPrism `PerformUntilHeight`, BRepFeat_MakeRevol `PerformUntilAngle` | `if (Until.IsNull()) Perform(Length);` and `if (Length == 0) Perform(Until);` don't return | A null Until throws Standard_ConstructionError after the prism of the length; a zero length builds the prism twice. Reproduced | touch |
| BRepFeat_MakeRevol `PerformUntilAngle` | The branches are the other way round from `Perform(Until)`: a finite until face is cut, a support glued | Reproduced | fixed (decision 1480) |
| BRepFeat_MakePipe `Perform(Until)`, `Perform(From, Until)` | `myFShape` becomes the pipe's first shape, which the map doesn't hold | `FirstShape()` then throws Standard_NoSuchObject. Reproduced | touch |
| BRepFeat_Form `GlobalPerform` | `if (myMap.IsEmpty())` where `myMap(fac).IsEmpty()` is meant (the until faces outside the shape) | An until face cut away entirely keeps an empty list. Reproduced | touch |
| BRepFeat_MakeRevolutionForm `Init` | Tests `proj.NbPoints()` again where `proj1` is meant | Reproduced | touch |
| BRepFeat_MakeRevol, RevolutionForm `ToFuse` | Down-casts the face's surface to a plane after testing the basis surface | A trimmed plane gives a null plane: an access violation natively, a NullReferenceException in the port | touch |
| BRepFeat_MakeCylindricalHole `PerformThruNext` | The `else` meant for `parbar > Last` hangs on `First - parbar < dmin` | Reproduced | touch |
| BRepFeat_Form `UpdateDescendants`, BRepFeat_RibSlot, BRepFeat_MakeDPrism `BossEdges` | Iterate maps keyed by shape address into the history lists and the lateral edges | Natively in address order; the port in insertion order (decision 1054) | leave |
| BRepFeat_Builder, BRepFeat_Form (the Booleans), BRepFeat_MakeLinearForm `Init` (`BB.UpdateVertex`, `myPln->Transform`), BRepFeat_RibSlot `LFPerform` (`BRepLib::SameParameter` on the glued result) | Write into the shapes and the plane the caller gives | The port works on copies (decision 1052) | settled |

## Phase 7: unit 7K, the walking (BRepBlend)

Found by porting BRepBlend and by tools/blendrec. The port reproduces each (`// OCCT` at the line) except where said.

| Where | What | Effect | Triage |
|---|---|---|---|
| BRepBlend_CSWalking `IsDone`, `Line` | Declared inline in the header, never defined (no .lxx) | The class can't be used natively (a link error); ChFi3d doesn't use it. The port defines them (decision 1059) | settled |
| BRepBlend_RstRstConstRad, RstRstEvolRad `IsSolution` | `math_Vector DEDT(1, 3); DEDT = secmember;` for the 2x2 system when Gauss fails | Release builds copy the two values and solve on them (the dimension checks are compiled out); the port's vector has two entries | leave |
| BRepBlend_RstRstConstRad, RstRstEvolRad `IsSolution` | The second restriction's 2D tangent is computed from `tgrst1` | `Tangent2dOnRst2` is the first restriction's tangent in the second surface's parameters | touch |
| BRepBlend_RstRstLineBuilder `PerformFirstSection`, `InternalPerform` | Leaving the first restriction's domain, `IsVtxrst2`/`Vtxrst2` take the vertex found on the first restriction | The second extremity can be marked on the first restriction's vertex | touch |
| BRepBlend_RstRstLineBuilder `MakeExtremity` | The second extremity's tangent is `TangentOnC1()` | The extremity on the second restriction has the first's tangent | touch |
| BRepBlend_SurfRstLineBuilder `InternalPerform` | On both restrictions at once, `sol(3) = solinvs(3)` (the parameter on the face's arc) where `PerformFirstSection` takes `solinvs(2)` (on the restriction) | The section is set at a wrong restriction parameter before `TestArret` | touch |
| BRepBlend_SurfRstLineBuilder `Perform` | The restriction's start extremity is built with the surface's (U, V) | Its parameter is the surface's U, not the restriction's | touch |
| BRepBlend_Walking `Perform`, `PerformFirstSection`, `InternalPerform`; ChFi3d's `UpdateLine` | Four-argument `BRepBlend_Extremity(P, sol(1), sol(2), tol)` and `SetValue` are the curve overload (P, W, Param, Tol) | The extremity's U is the surface's u and its guide parameter the surface's v | touch |
| BRepBlend_SurfRstConstRad, SurfRstEvolRad, RstRstConstRad, RstRstEvolRad | `myTConv` is not initialised by the constructors | Read only after `GetShape` sets it in OCCT's call order; the port starts it at its first value | leave |
| BRepBlend_SurfRstLineBuilder `ArcToRecadre` | The header names the 2D points (pt2d, lastpt2d), the definition (lastpt2d, pt2d) | The port follows the definition | leave |
| BRepBlend_CurvPointRadInv | `choix` is set and never read | None | leave |

## Phase 7: unit 7L, the legacy data structure's core

Found by porting TopOpeBRepDS's core and TopOpeBRepTool_ShapeTool. The port reproduces each (`// OCCT` or a remark at
the place).

| Where | What | Effect | Triage |
|---|---|---|---|
| TopOpeBRepDS_SurfaceIterator constructor | Sets the SURFACE condition after the base constructor has matched, without matching again | The first `Value()` is the list's first interference whatever its geometry (`HDataStructure::SolidSurfaces`); `Next()` then skips the others | touch |
| TopOpeBRepDS_DataStructure `Change*Interferences`, `ChangeCurve`, `ChangePoint`, `ChangeSurface` | Return the data structure's own empty list or value, for writing, when the shape or geometry is missing | What is written there is every later miss's list or value (decision 1066) | touch |
| TopOpeBRepDS_Point(shape) | Leaves `myKeep` unset; the default constructor leaves the tolerance unset | Uninitialised natively; the port keeps the point and uses 0 | leave |
| TopOpeBRepDS_Interference() | Leaves the indices and kinds unset | Uninitialised natively; the port uses 0 and POINT | leave |
| TopOpeBRepDS_SurfaceExplorer() | Leaves `myFindKeep` unset | Uninitialised natively; the port uses false | leave |
| TopOpeBRepDS_CurveIterator `PCurve`, TopOpeBRepDS_Curve `Curve1`/`Curve2` | Cast the interference to a surface/curve interference without checking | Undefined natively on another class; the port throws InvalidCastException | touch |
| TopOpeBRepDS_Surface | A new surface is not kept, a new point or curve is | `SurfaceExplorer` with its default `FindOnlyKeep` finds none of a fillet's surfaces | touch |
| TopOpeBRepDS_DataStructure `IsSectionEdge`, `SectionEdge` | Need the edge among the data structure's shapes too, not only among the section edges | A section edge added alone is not one | touch |
| TopOpeBRepDS_HDataStructure `FUN_TopOpeBRepDS_SortOnParameter` | Counts all interferences but finds only point ones | With other interferences in the list, the last point found is appended again (a null one first if none was found, then `T(0)` out of range: undefined natively, the port throws) | touch |
| TopOpeBRepTool_ShapeTool `EdgeData` | Compares the curvature with `max(tol, Epsilon(0), RealLast())` | The normal is never computed | touch |
| TopOpeBRepTool_ShapeTool `SurfacesSameOriented`, `CurvesSameOriented` | Only planes and cylinders, and lines, are compared ("NYI") | Every other pair is "same oriented" | touch |
## Phase 7: unit 7X, Quantity_Color and build123d.NET's phase 7 calls

Found by porting Quantity_Color and Quantity_ColorRGBA and by replaying build123d.NET's phase 7 calls. The port
reproduces each (`// OCCT` at the line), unless the last column says otherwise.

| Where | What | Effect | Triage |
|---|---|---|---|
| Quantity_Color `ColorFromName` | The bare prefix `"Quantity_NOC_"` (any case) makes `SubString(14, 13)` raise Standard_OutOfRange inside a `noexcept` function | Native OCCT terminates the process (std::terminate). The port throws Standard_OutOfRange (decision 1070) | touch |
| Quantity_Color `Argb2color` | Reads the bytes as sRGB, where `Color2argb` writes the linear values | A colour doesn't come back from its ARGB integer; documented by OCCT. Reproduced | leave |

The replay found nothing new in the offsets, drafts and features: the null result after a reported success and native's
run-to-run variation with the Arc join are unit 7F's (decisions 1031 and 1032).

## Phase 7: unit 7P, the foundations of HLR

Found by porting Intrv, TopBas, TopCnx, HLRAlgo and Contap and by tools/hlrrec. The port reproduces each (`// OCCT` at
the line where it is code).

| Where | What | Effect | Triage |
|---|---|---|---|
| HLRAlgo_Projector `Project(const gp_Pnt&, gp_Pnt2d&)` | For the first hard-coded axonometric view (`TrsfType` 0) computes X and Y and never writes `Pout` | The caller's point comes back unchanged (decision 1078); the other `Project` forms are right | touch |
| Contap_ArcFunction `Value`, `Values`; Contap_SurfFunction `Value`, `Derivatives`, `Values` | Contap_DraftPrs (a draft seen from an eye) computes nothing | The value is left unset (the port's ArcFunction returns 0, decision 1076; SurfFunction leaves the caller's vector and matrix); the derivatives are 0 | touch |
| Contap_HCurve2dTool `NbSamples` | `nbs *= (Last - First) / (U1 - U0)`, the inverse of IntPatch_HCurve2dTool's corrected ratio | More samples on a short range of a B-spline arc than on the whole arc | touch |
| HLRAlgo_EdgeIterator `InitHidden` | On a status neither wholly hidden nor with visible intervals (a default status) reads the first interval of an empty sequence | An access violation natively; Standard_OutOfRange in the port | touch |
| Intrv_Interval's default constructor | Its tolerances are `Epsilon(RealFirst())` and `Epsilon(RealLast())`, both 0 | The whole line has null tolerances (the port's `Epsilon` gave infinity there, decision 1080) | settled |
| TopCnx_EdgeFaceTransition `BoundaryTransition` | Both branches of the even/odd test return TopAbs_EXTERNAL | Reproduced | touch |
| Contap_Contour `LineConstructor` | `u2`, `v2` unset when a line or circle lies on a surface other than a cylinder, cone or sphere | Unreachable (ContAna only makes those); the port starts them at 0 | leave |
| HLRAlgo_EdgeStatus `Hide` | Ignores `OnBoundary` | Reproduced | touch |

## Phase 7: unit 7N1, the fillet construction data

Found by porting ChFiKPart and ChFi3d_Builder_0. The port reproduces each (`// OCCT` or a remark at the place).

| Where | What | Effect | Triage |
|---|---|---|---|
| ChFiKPart_ComputeData_ChAsymPlnCyl `MakeChAsym` (straight guide) | Projects the guide's point on the cylinder's axis with `gp_XYZ::Multiplied(gp_XYZ)`, a product coordinate by coordinate, where a dot product is meant | The point on the axis is right only for an axis parallel to a coordinate axis; on any other the distance-angle chamfer along a generatrix is built from a wrong point | fixed (decision 1475) |
| ChFiKPart_ComputeData_FilPlnCyl `MakeFillet` (circle, sphere case) | Evaluates the sphere with the cylinder's radius | The two radii are equal within the confusion there: no effect | leave |
| ChFi3d_Builder_0.hxx | Declares `ChFi3d_EnlargeFace`, defined nowhere | Not ported | leave |
| ChFi3d_Builder_0 `ChFi3d_IndexPointInDS` | Writes the point and tolerance into the common point's vertex ("it is necessary to implement a mechanism controlling tolerance") | Changes the input's vertices natively (plan 2.2); the port writes the builder's working copy (decision 1087) | settled |
| ChFi3d_Builder_0 `ChFi3d_Orientation`, `ChFi3d_FilDS` | The orientation is left unchanged when no interference matches; FilDS goes on with the previous transition | A fillet surface data on a surface without its solid interference takes the previous one's transitions | touch |
| ChFi3d_Builder_0 `ChFi3d_PerformElSpine` | Declares three arrays (`ExtrapPole`, `ExtraCoeffs`, `Cont`) it never uses | None | leave |
| ChFi3d_Builder_0 `ChFi3d_StripeEdgeInter` | Takes the data structure and never reads it | None | leave |

## Phase 7: unit 7Q1, the data model of HLR

Found by porting HLRTopoBRep and HLRBRep's data model and by tools/hlrrec. The port reproduces each (`// OCCT` at the
line where it is code).

| Where | What | Effect | Triage |
|---|---|---|---|
| HLRTopoBRep_DSFiller `ProcessEdges`, HLRTopoBRep_FaceIsoLiner `MakeIsoLine` | Update the point representations of the input shape's vertices (the ends of the edges split, an isoline's ends) | Writes into the caller's shape; the port's outliner copies a frozen shape first (decision 1095) | settled |
| HLRTopoBRep_DSFiller `InsertFace` | `if (Maxz > Maxx) Maxx = Maxy;` where `Maxz` is meant | The approximation's 3D tolerance of a walking contour ignores its extent in Z | touch |
| HLRBRep_SurfaceTool, HLRBRep_BSurfaceTool `NbSamplesU(S, u1, u2)`, `NbSamplesV` | Divide by `first - last`, so the ratio is negative | Above 10 samples, the count is always 5 | touch |
| HLRBRep_ThePolyhedronOfInterCSurf | Its constructors leave the four singularity flags uninitialised and don't check the parameter arrays (IntCurveSurface_ThePolyhedronOfHInter does both) | The port's flags are false | leave |
| HLRBRep_Data `Write` | The copied faces share the source's wires blocks, whose edge indices are then shifted | The source data's faces point to the wrong edges afterwards | touch |
| HLRBRep_Data `NextInterference` | `if (AutoIntersectionDone()) { AutoIntersectionDone(true); ... }` sets the flag only when it is already set | A self-intersection is rejected only for edges marked done by `Update` (simple faces) | touch |
| HLRBRep_Data's rejection table `Set` | Tests `IndUV[i0][i] != -1` before `i < nbUV[i0]` | Reads one element past a full row; the port tests the bound first (same result) | leave |
| HLRBRep_Intersector `Perform` (two edges sharing a vertex) | The stop test is `aDecalageb2 <= 0` where the three others are `< 0` | Reproduced | touch |
| HLRBRep_ShapeToHLR `ExploreShape` | The closedness of an open shell counts the original shell's edges in the outlined shape's edge map; a split edge is not there (index 0) | Its orientations are counted in an unused slot: split edges don't make a shell open | touch |
| HLRBRep_Curve | `D3` writes nothing, `DN` returns the null vector, `Hyperbola` and `Parabola` default ones | Reproduced (HLRBRep_CLProps' third derivative keeps its previous value) | touch |
| HLRBRep_Surface | Declares `UIntervalContinuity` and `VIntervalContinuity` without defining them | Left out | leave |
## Phase 7: unit 7Y, follow-ups on the differences from native

Found by unit 7Y's investigations. The port reproduces each except where said.

| Where | What | Effect | Triage |
|---|---|---|---|
| BRepOffset_Tool `EnlargeGeometry` (through `EnLargeFace` with `ChangeGeom`) | On a Geom_OffsetSurface, `SetBasisSurface` on the face's own surface; on a B-spline basis, `ExtendSurfByLength` raises its degree in place | The face given lies on the enlarged surface afterwards, its bounds grown (BRepOffset_MakeOffset's `ExtentFace` relies on it, OCC5805's thick solid); a caller's face changes too. The port does the first on its working faces (decision 1101), never on a caller's, and not the degree raise | touch |
| BRepFill_Filling `AddConstraints` | An edge without a pcurve on its support face leaves the 2D adaptor unloaded, then evaluated (OCCT's comment says so) | A null dereference natively; the port takes the support's own edge on the same curve for a caller's frozen edge (decision 1103) | settled |
| LocOpe_DPrism (BRepFeat_MakeDPrism with a taper) | `BRepLib::UpdateTolerances` on the result raises the profile's vertex tolerances (1.06e-5), the profile being shared | The caller's profile changes; a later DPrism on it starts from the raised tolerances (decision 1105) | settled |
| build123d.NET's native binding, from several threads at once | (see decision 1109) | | leave |

## Phase 7: unit 7R, the fillets' reconstruction

Found by comparing the reconstruction with native OCCT's fillets (tools/dsrec --recon). The reconstruction replaces
TopOpeBRepBuild (decision 992), so these are native results it does not reproduce; each is attributed in
`ReconstructionNativeTests.KnownDifferences`.

| Where | What | Effect | Triage |
|---|---|---|---|
| TopOpeBRepBuild (a fillet reaching a cone's apex, blend/simple H8) | The degenerated edge at the apex is left whole although the fillet's curve reaches it inside its parametric range | Native's cone face fails BRepCheck (UnorientableShape); the reconstruction cuts the edge there (decision 1113) and its face is valid, the geometry equal | settled |
| ChFi3d / TopOpeBRepBuild (blend/complex A1, OCC22817, "TEST INCOMPLETE" in OCCT) | The result fails BRepCheck (a sub-shape badly oriented) and the face on one fillet surface has a negative area | The reconstruction builds no face on that surface and its result is valid; volumes differ | leave |
| TopOpeBRepBuild (a Bezier face cut, blend/complex A2) | The pieces are built on a B-spline copy of the Bezier surface | The reconstruction's pieces keep the Bezier surface; same geometry | leave |
| BRepFilletAPI_MakeFillet (all twelve edges of a cube of 10, radius 6) | `IsDone` is true and the result degenerate: negative volume, an area of -6e29, B-spline faces far outside the cube | Not compared; build123d.NET rejects the result | touch |
| TopOpeBRepBuild_HBuilder::Perform (blend/complex A3) | Its filters and reducers change the data structure before building (the only recorded case where they do) | The reconstruction reads the data structure as ChFi3d left it and gives native's result | leave |

## Phase 7: unit 7N2, the fillet builder

Found by porting ChFi3d_Builder, ChFi3d_FilBuilder and ChFi3d_ChBuilder. The port reproduces each (`// OCCT` or a
remark at the place).

| Where | What | Effect | Triage |
|---|---|---|---|
| ChFi3d_Builder_1 `PerformExtremity` | Tests `NCollection_IndexedMap::Add`, which returns an index, as a boolean | Always true: the reversed copy of a closed edge is always taken as one of the vertex's edges | touch |
| ChFi3d_Builder_SpKP `FillSD` | Keeps a pointer to a local copy of the domain's point that leaves its scope | Undefined natively; the port uses the copy | leave |
| ChFi3d_Builder_SpKP `Tri` | Casts the constness of the hatcher's domain and point away and edits them | The hatcher's domains change under it (reproduced through `ChangeDomain`) | touch |
| ChFi3d_Builder_CnCrn (OCCT .cxx:3280) | `tpt2.Tolerance(tpt1.Tolerance() + error)` | The second point takes the first's tolerance | touch |
| ChFi3d_Builder_C1 | `Crv.Tolerance(...)` on a local copy of a curve already added to the data structure | The raised tolerance is lost | touch |
| ChFi3d_Builder_C1, `_C2` | Booleans kept in integers and doubles (`proledge`, `sens`, `sameparam`) | None (0 and 1) | leave |
| ChFi3d_ChBuilder `SimulSurf` (constant throat with penetration) | Prints "Construction of offset guide failed!" to std::cout and goes on with a null guide | The port does not print; the null guide fails as natively | leave |
| ChFi3d_ChBuilder `SetDistAngle` | Initializes two surface adaptors it never reads | None | leave |

## Phase 7: unit 7R2, the reconstruction's follow-up

| Where | What | Effect | Triage |
|---|---|---|---|
| ChFi3d (four base edges of a square pyramid, radius 5) | Native's result fails BRepCheck: a face UnorientableShape, a wire SelfIntersectingWire | The port (ChFi3d and the reconstruction) gives the same result and the same faults (decision 1125) | leave |
| BRepFilletAPI_MakeFillet (a box of 10, its top edges, radius 6) | `IsDone` is true; two corner faces have negative areas and the result fails BRepCheck (BadOrientationOfSubshape, IntersectingWires) | The reconstruction builds no face on the degenerate surfaces and keeps IntersectingWires; not compared further | leave |
## Phase 7: unit 7Q2, the hidden-line removal algorithms

Found by porting HLRBRep's algorithms, HLRAppli and the HLR Draw commands. The port reproduces each except where said.

| Where | What | Effect | Triage |
|---|---|---|---|
| HLRBRep_Hider `Hide` (an ON interference with an INTERNAL transition and an UNKNOWN state before it) | `case TopAbs_UNKNOWN: break;` leaves the list's iterator where it is | The loop over the ON interferences never ends; the port moves to the next interference (the only fix: a hang) | settled |
| HLRBRep_EdgeBuilder.cxx | Defines `No_Exception`, compiling out its checks of an empty vertex list and of a list without interferences | The builder then dereferences a null area limit (caught by the hider's OCC_CATCH_SIGNALS); the port does the same | touch |
| HLRBRep_InternalAlgo `Hide(I, J)` | The shapes' min-max test checks both directions for indices 0 to 5 but one only for 6 and 7 (14 conditions, not 16) | Some pairs of shapes that cannot overlap are hidden by each other anyway (slower, same result) | leave |
| HLRBRep_InternalAlgo `Update` | The shape's min-max box starts from its first edge's: a shape without edges adds its faces' boxes to unset values | The port's start is zeros | leave |
| HLRBRep_PolyAlgo `InsertOnOutLine` | Keeps a reference to a segment of the array that `UpdateLinks` may reallocate | Not read after the reallocation: no effect | leave |
| HLRBRep_PolyAlgo's copy constructor | Shares the other's HLRAlgo_PolyAlgo | `Update` on either clears the other's results | touch |
| HLRBRep_HLRToShape `InternalCompound` | Scales the data structure's projector during the call (`Scaled(true)`, then `Scaled(false)`) | The algorithm's projector is unscaled afterwards whatever it was | touch |
| HLRBRep `MakeEdge3d`, HLRAppli_ReflectLines `GetCompoundOf3dEdges` | The 3D edges share the input edges' vertices, and `BRepLib::SameParameter` on the compound may raise their tolerances | Writes into the caller's vertices natively; the port's are the outliner's working copy's for a frozen input (decision 1128) | settled |
| HLRTest `houtl` | Reads `a[2]` after checking only `n < 2` | A null name with one argument; the port checks | leave |
| ViewerTest `vcomputehlr` | The projector's location is the negated centre of the near plane | On the view axis: the 2D results don't depend on it but through rounding | leave |

## Phase 7: unit 7N3, the public fillet and chamfer API

Found by porting BRepFilletAPI, FilletSurf and the fillet and chamfer Draw commands. The port reproduces each, but
for the hang in `chamf` (decision 1146).

| Where | What | Effect | Triage |
|---|---|---|---|
| ChFiDS_FilSpine::SetRadius(Law_Function, IinC) (BRepFilletAPI_MakeFillet's `Add(L, E)` and `SetRadius(L, IC, IinC)`) | Builds a Law_Composite holding the law, drops it, and clears the radii | The law is lost; Build throws Standard_OutOfRange on the empty radii | fixed (decision 1433) |
| FilletSurf_InternalBuilder::Add | The loop over the edge's faces re-initializes the iterator of the loop over the edges | Only the first edge is checked for being sharp and on the shape; the outer loop then runs on over the first edge's faces | touch |
| FilletSurf_Builder::EndSectionStatus | Its StdFail_NotDone says "StartSectionStatus" | None but the message | leave |
| BRepTest_ChamferCommands.cxx, `chamf` | When an edge is followed by a face and fewer than two more words (`chamf r s e f 1`), neither branch advances `i` and the `while (i + 1 < narg)` loop never ends | Draw-lite stops reading the words there (decision 1146); no corpus case reaches it | leave |
| BRepTest_FilletCommands.cxx, `buildevol` and `blend` | `Rake`/`Rakk` are deleted only after a Build that returns | On an exception the maker stays; `checkhist` then says "Active Builder Not Done" (reproduced) | leave |
| Corpus cases that depend on the reconstruction's order (not OCCT bugs, for unit 7R) | blend/complex/G8 and bugs/modalg_7/bug27711_3 name the edges of a fillet's result by `explode` index for a second fillet | The reconstruction's faces and edges come in another order (decision 1114), so another edge is filleted and `blend` fails | leave |

## Phase 7: unit 7R3, the reconstruction's tail

| Where | What | Effect | Triage |
|---|---|---|---|
| ChFi3d (blend/simple Y1) | A fillet ending exactly at a vertex is recorded as a new point, with an interference at the edge's end whose transition says the edge is OUT | The reconstruction takes the point for the vertex and leaves the edge whole (decision 1157) | leave |
| ChFi3d (tolblend_simple C4, C7, C8, bug354_1) | A fillet's end section overflows the edge: the point's parameter lies past the edge's range | The part kept is extended to it, as TopOpeBRepBuild does (decision 1160) | leave |
| TopOpeBRepBuild (blend/simple V7, tolblend_simple C5, complex A2) | A Bezier face the fillet runs past is rebuilt on a B-spline | Not reproduced: the reconstruction's piece keeps the Bezier surface (decision 1162) | settled |
| BRepFilletAPI_MakeFillet `IsDeleted` | A solid the fillets do not touch is "deleted" (`myMap` holds the result's faces only) | Reproduced | touch |

## Phase 8: unit 8H1, STL

Found by porting RWStl and StlAPI and recording native OCCT on hand-made and derived inputs (tools/stlrec). The port
reproduces each, but for the allocation in the last row but two (decision 1170).

| Where | What | Effect | Triage |
|---|---|---|---|
| RWStl_Reader::Read | Returns `!stream.fail()`, and the line buffer's last short block sets the fail flag | False for nearly every ASCII file read whole (`RWStl::ReadFile` ignores it) | touch |
| RWStl_Reader::Read, multi-domain ASCII | The line buffer reads ahead in 1 KB blocks, but the loop tests the stream's flags and skips white space (`>> std::ws`) at the stream's position, inside data the buffer hasn't returned | Domains after the first are lost once the last block has been read (files under 1 KB: twosolids.stl reads one solid); the white space skipped can join lines or numbers at a block boundary | fixed (decision 1499) |
| RWStl_Reader::Read, multi-domain | A domain not closed by "endsolid" fails, so `AddSolid` isn't called for it | `RWStl::ReadFile(…, list)` drops it; the single-triangulation form keeps it | touch |
| RWStl_Reader::ReadVertex | Skips letters before the numbers | `vertex nan 0 0` and `vertex inf …` fail to read ("cannot read vertex coordinates") | leave |
| RWStl::ReadStream, StlAPI_Reader::Read(stream) | ASCII if the data start with "solid" (case-sensitive), binary otherwise | A binary file whose header starts "solid", or an ASCII one starting "SOLID", fails; `ReadFile` recognises both | fixed (decision 1498) |
| RWStl_Reader::ReadBinary | Reads the facets in blocks of 80, the first sized by the header's count | A header claiming more facets than the data hold reads nothing if the shortfall is in the first block | touch |
| RWStl_Reader::ReadBinary | Sizes Poly_MergeNodesTool's map by the header's facet count (`2 * count` buckets, an `int`) | Standard_OutOfMemory when text is read as binary (`ReadBinaryStream` on ASCII); not reproduced (decision 1170) | settled |
| Standard_ReadLineBuffer::ReadLine | The last line, without an end of line, is returned without its terminating `'\0'`; in multi-line mode, reads `myReadBuffer[pos + 2]` past the bytes read | Callers parsing it as a C string may read past it; the port's spans end at the line | leave |
| RWStl::WriteAscii(path), StlAPI_Writer::Write(path) | Open the file in text mode | CR LF line ends on Windows only (decision 1168) | leave |
## Phase 8: unit 8P, the STEP parser

Found by rewriting StepFile's flex scanner and bison parser and comparing them with OCCT's on the corpus and on hostile
inputs. The port reproduces each, but for the crash in FinalOfScope.

| Where | What | Effect | Triage |
|---|---|---|---|
| step.lex (flex) | The variable trailing context of `#[0-9]+/[ \t]*=` makes flex keep a state per byte, so its buffer can't grow: a token (a string with what `yymore` collected, a name, a comment's run, the blanks after a label) can't reach its 16,384th byte | "input buffer overflow, can't enlarge buffer because scanner uses REJECT": StepFile_Read fails on a file holding a string of 16,383 bytes or more | fixed (decision 1511) |
| step.lex, the end of a string | `[']/[" "\n\r]*[\)\,]`: the class holds the double quote, and not the tab | `'a' "",` ends the string; `'a'` followed by a tab and a comma doesn't, and the string runs on to the next quote followed by `)` or `,` | fixed (decision 1512) |
| step.lex, numbers | The real rules need two characters before the exponent; ties go to the first rule | `1E3` is a name (TYPE: in a list, `1E3,` is a syntax error), `12E3` a real; `-` and `+` alone are integers, `...` a real, `T.` an enumeration | touch |
| step.tab.cxx `parser::error` | Adds 1 to the scanner's line counter, which starts at 1 | Errors are reported one line after their own | touch |
| StepFile_ReadData::FinalOfScope | Reads `mySubArg[0]` while `mySubArg` is null, until a first list is closed | Crash on an empty header followed by a scope as the first entity; the port takes it for a scope without an export list | settled |
| StepFile_ReadData::CreateNewRecord | Allocates records uninitialized | A record never given a type (reachable through syntax errors) has a garbage type; null in the port | settled |
| StepFile_ReadData::CreateNewText | The CARTESIAN_POINT shortcut comes before the error argument's concatenation, which it skips | An error argument's text drops what came before a `CARTESIAN_POINT` | touch |
| StepFile_ReadData texts | C strings | A NUL byte in a quoted string ends its text | leave |
| StepFile_ReadData::PrintRecord | The two-letter codes of the argument types (`(IF#TnEHBx`, `)nlIxdnxix`) don't follow Interface_ParamType's order | A text prints `nd`, a sub-list `Bi` (print modes only) | leave |
| step.yacc `stepf1`, `stepf2`, `stepf3` | The data section must be followed by `ENDSTEP;` and the end of the input, unless `END-ISO-10303-21;` (whose start condition skips the rest); `stepf3` accepts a data section ended by an error at the end of the input, but only after an empty header | Text on a line after `ENDSTEP;` fails the read; a truncated file is read if its header is empty, fails otherwise | touch |
| step.yacc `finvide` | Expects `' '` tokens, which the scanner never returns (it skips blanks) | None | leave |
| step.yacc `finscop` | Prints "Export List not yet processed" with printf | On the standard output; the port sends it to the messenger | settled |
## Phase 8: unit 8A1, XSBase's foundation

Found by porting MoniTool, Interface, IFGraph and LibCtl, and by recording them natively (tools/xsbaserec). The port
reproduces each, but for `EntityCluster::Remove(num)` (decision 1190).

| Where | What | Effect | Triage |
|---|---|---|---|
| Interface_EntityCluster::Remove(num) | The shift loop runs `j--` while `j < n` | Out of bounds below the array for an inner rank; fixed in the port (decision 1190) | settled |
| Interface_EntityCluster::Value, SetValue | Walking the clusters, subtract the first cluster's count, not the current one's; SetValue tests the first cluster's `thenext` | Once `Interface_EntityList::Add` has put a cluster that isn't full in front, the entities past it throw Standard_OutOfRange (recorded natively) | touch |
| Interface_EntityCluster::Remove(ent) | Returns false even when the entity was removed further on (via an `int res`) | Interface_EntityList::Remove(ent) doesn't drop an emptied list | touch |
| Interface_EntityList::TypedEntity | Counts each cluster backwards | The n-th entity of a type isn't the n-th in order | touch |
| MoniTool_TypedValue::ClearValue | Nullifies the value's string, which IsSetValue, SetCStringValue and RealValue then dereference | A crash after ClearValue (a NullReferenceException in the port) | touch |
| MoniTool_Stat | Its copy constructor leaves the counters null; the constructor initialises `thetot` three times; Percent divides integers | A copied Stat fails at Open; the sub-level share is 0 until done | touch |
| Interface_Static | Redeclares `thesatisf`, `thesatisn`, which nothing sets | PrintStatic never prints the satisfies function | leave |
| Interface_Static::IsSet(name, false) | Takes the wild-card without checking it is attached | A crash for an unset static without a wild-card | touch |
| Interface_Static::IDef | `EnumDef`'s outputs are uninitialised for a static that isn't an enumeration; "eval" with no word reads an uninitialised buffer | Garbage "estart", "ecount"; the port gives 0, 0 and "start - 1" | settled |
| Interface_Graph::Name | `if (themodel->Number(ent)) return str;`: the test is inverted | No name for an entity of the model | touch |
| Interface_Category | The categories are numbered from 0 (an NCollection_DynamicArray) | "Shape" is 0, the number of an unknown name too; AddCategory("Shape") would append it again | leave |
| Interface_BitMap::AddFlag, SetFlagName | Append the name to a free (".") or existing name instead of replacing it | Names like ".f2" | touch |
| Interface_Check::Complies, Remove | The fails' branch loops over the warnings (copied code) | Fails are never matched nor removed | touch |
| Interface_CheckTool's check lists | Check without try/catch (`errh = 0`), and that path returns before the "Shared Items unknown" fail | The lists don't report entities sharing items out of the model (`Check(num)` does) | touch |
| Interface_CheckTool::WarningCheckList | Adds its working check, then clears it and sets the next entity on it | The recorded checks end empty, with the last entity | touch |
| Interface_CopyTool::Transferred | Increments the nesting level after the copy, so the entities a copy transfers count at level 0 | RootResult lists them as roots, some twice | touch |
| Interface_InterfaceModel::AddReportEntity | `semantic` true adds to the main reports, false to the semantic ones | The reverse of its documentation | touch |
| Interface_InterfaceModel::ChangeOrder | Assigns `rep1` twice and never `rep2` | Reports aren't swapped correctly | touch |
| Interface_ParamSet::Append | Past the reservation, each parameter replaces the next set | Only the last overflowing parameter is kept (the readers reserve enough) | leave |
| IFGraph_AllConnected::GetFromEntity | `if (!thegraph.IsPresent(...)) return;`: the test is inverted | Finds nothing; IFGraph_ConnectedComponants' parts are all empty | touch |
| IFGraph_Cumulate::GetFromEntity, IFGraph_Compare::GetFromEntity | Iterate an IFGraph_AllShared without evaluating it (a graph content evaluates at Begin, not Start) | Add nothing (GetFromIter with an evaluated content works) | touch |
| IFGraph_StrongComponants::Evaluate | Puts each entity in a part of its own | No strong components: IFGraph_Cycles finds no cycle | touch |
| LibCtl_Library::Clear | Leaves an empty node, which Select, Module and Protocol dereference | A crash if the library isn't refilled | touch |
| Interface_MSG::Blanks | Return a pointer before their buffer of 76 blanks for wider fields | Garbage natively; the port returns the blanks | settled |

## Phase 8: unit 8B, StepData and the STEP header

Found by porting StepData, the header packages and Resource_Unicode's conversions, and by recording them natively
(tools/stepdatarec). The port reproduces each, but for the first (decision 1198).

| Where | What | Effect | Triage |
|---|---|---|---|
| StepData_Field::CopyFrom | For a SelectNamed, copies `*this` (still holding the member) into the copy's field | Endless recursion; fixed in the port | settled |
| StepData_Field::CopyFrom | Names the SelectNamed copy with its own (empty) name | The copy loses the member's name | touch |
| StepData_Field::CopyFrom, SetEntity(num) | Build the copied or converted list, then drop it | Lists are shared, not copied; SetEntity on a list of integers, reals or strings changes nothing | touch |
| StepData_Field::Set | For an HArray2 of reals sets the kind Integer, and reads `hi2` (null) for the row length of reals and transients | A crash (NullReferenceException) | touch |
| StepData_Field::ItemKind | `if (!ht.IsNull()) return kind;`: the test is inverted | "Any" for a list of transients; a null dereference for others | touch |
| StepData_Field::Real(n1, n2) | Returns a member's integer value for a list of lists | Reals truncated | touch |
| StepData_SelectMember::SetLogical | Stores 0 for every value | A logical member reads back False | touch |
| StepData_EnumTool::AddDefinition | After a blank, the next term starts with that blank; "$" gets a dot too ("$."); a term ending with a blank reads before the buffer | ". B." for "A B"; NullValue never finds "$" and returns -1 | touch |
| StepData_EnumTool::NullValue | Returns `false` (0) where it documents -1 when not optional | 0 | touch |
| StepData_StepReaderData::ReadAny | An integer parameter read into a given member is set as a real | | touch |
| StepData_StepReaderData::cleanText | Lead bytes near the end make the UTF-8 decoder read the terminating zero and the memory past it; an empty decoding (an overlong zero) makes `Trunc(-1)` throw | Garbage or Standard_OutOfRange from ReadString | touch |
| Resource_Unicode::ConvertGBKToUnicode, ConvertBig5ToUnicode | Insert the bytes read so far "at 0" (`Insert(0, ...)`) when a lead byte is followed by ASCII | Standard_OutOfRange | touch |
| Resource_Unicode::ConvertGBKToUnicode | After a 4-byte sequence, keeps its three bytes, so every following byte reads as a fourth; the 4-byte code point indexes the 2-byte table | Garbage after the first 4-byte character | touch |
| RWHeaderSection::Init | Registers the header protocol again at each call | From the second call, the header protocol is a FileProtocol | touch |
| APIHeaderSection_MakeHeader::SetDescription | Tests `fs`, then sets `fd` | A crash if there is a schema and no description | touch |
| StepData_StepWriter (undefined entities) | A sub-list's type is the parser's "/* (SUB) */", which OpenTypedSub writes | Written sub-lists start with a comment (valid STEP) | leave |
| StepData_StepDumper::Dump | Lists the proper idents under `nbi < 0`, never true; `nbu` and `nbq` are reset to 0 instead of counted | Never lists entities without ident or with ident = number | leave |
## Phase 8: unit 8A2, Transfer, TransferBRep and XSAlgo

Found by porting Transfer, TransferBRep and XSAlgo and testing them on the toy norm. The port reproduces each.

| Where | What | Effect | Triage |
|---|---|---|---|
| Transfer_ProcessForTransient/ForFinder::Unbind | Puts `thelastbnd` back in the map, which FindAndMask has just set to the former binder | Nothing is unbound (the roots are updated and true returned); Clean finds no unbound item | touch |
| Transfer_ProcessForTransient/ForFinder::ResultOne, CheckListOne, IsCheckListEmpty, RemoveResult | Their array of indices is never filled since the scopes were removed | An empty iterator and check list, true, and nothing removed, for any mapped object | touch |
| Transfer_ProcessForTransient/ForFinder::Transferring | Root management records `therootl` at the end of the first transfer done, nested ones included | With root management, nested transfers become roots too (innermost first) | touch |
| Transfer_TransferIterator::SelectResult | Compares each item with `ResultType()`, the current item's type, not the item's | Selects on the current item's result type (a crash natively when the current item is multiple) | touch |
| Transfer_Finder::SameAttributes | Documented as sharing the other finder's attributes, but assigns the map (a copy in C++) | The attributes are copied, not shared | touch |
| Transfer_ResultFromModel::Results(1) | Appends the main result before and after the sub-results | The main result is listed twice at level 1 | touch |
| TransferBRep::Shapes(TP, list) | `if (TP.IsNull() && list.IsNull())` where `\|\|` is meant | A crash when only one of them is null | touch |
| TransferBRep_Reader::Transfer, TransferList | Don't clear the former shapes (TransferRoots does), though TransferList's documentation says so | The shapes of successive transfers accumulate | touch |
| TransferBRep::PrintResultInfo | Its message keys (Result.Print.MSG30 to MSG48) are in no message file of OCCT 8 | Prints "Unknown message invoked with the keyword ..." lines | touch |
| XSAlgo_AlgoContainer::ProcessShape | `if (!aSeq) aSeq = thePseq;` never applies: Interface_Static::CVal returns "" for an undeclared static | An undeclared sequence static gives the sequence "": the default ShapeFix on read, `.exec.op` = DirectFaces on write | touch |
| Transfer_ActorOfProcessForTransient/ForFinder | The default constructor leaves `thelast` uninitialised | False under OCCT's default (zeroing) allocator, as in the port | leave |
| Transfer_ActorDispatch::Transfer, Transfer_TransferDispatch::Copy | The actor's TransferEntity copies through the process, which calls the actor again, with nothing bound in between | An endless recursion (a stack overflow) for any entity the dispatch actor transfers; fixed in the port (decision 1207) | settled |

## Phase 8: unit 8C1, the STEP entity generator

Found by generating StepBasic, StepGeom, StepShape and StepRepr and their RW packages from OCCT's C++. The port
reproduces it (decision 1209).

| Where | What | Effect | Triage |
|---|---|---|---|
| RWStepRepr_RWBooleanRepresentationItem::WriteStep | `SW.Send(theEnt->Value())` with a `bool`: C++ promotes it to `int`, so `Send(int)` is called | BOOLEAN_REPRESENTATION_ITEM's value is written 1 or 0, not .T. or .F., and reading it back (`ReadBoolean`) fails | fixed (decision 1504) |

## Phase 8: unit 8C2, the other entity packages and StepAP214's protocol

Found by generating StepVisual, StepAP214/203/242, StepDimTol, StepKinematics, StepFEA, StepElement, their RW
packages and StepAP214's dispatch from OCCT's C++, and by the corpus's round trips (decision 1219). The port
reproduces them all; the generated code carries no `// OCCT` marker (it is regenerated), the tests name them.

| Where | What | Effect | Triage |
|---|---|---|---|
| RWStepVisual_RWSurfaceStyleRenderingWithProperties::ReadStep | Tests `if (strcmp(text, ".CONSTANT_SHADING."))`, true when the text differs, and so on down the list | Every rendering method but .CONSTANT_SHADING. is read as .CONSTANT_SHADING., and .CONSTANT_SHADING. as .COLOUR_SHADING. (545 records of the corpus) | fixed (decision 1501) |
| RWStepVisual_RWComplexTriangulatedFace::WriteStep, RWStepVisual_RWCubicBezierTriangulatedFace::WriteStep (normals, control triangles) | Loop the rows up to `RowLength()` and the columns up to `ColLength()`, which are the numbers of columns and rows | Out of range unless the array is square: OCCT's release build writes values from the wrong places, a checked build (build123d.NET's, the port's) throws Standard_OutOfRange. 552 faces of the corpus can't be written | touch |
| StepVisual_StyledItem::Item, RWStepVisual_RWStyledItem::WriteStep | Init keeps the target as a transient and as a representation item (null if it isn't one); the writer sends `Item()`, the representation item | A styled item whose target is a representation (or anything but a representation item) is written with `$` for its item (68 records) | fixed (decision 1502) |
| RWStepAP214_ReadWriteModule::IsComplex | Lists every case of ComplexType but 800 (ActuatedKinPairAndOrderKinPair) and 802 (RepositionedTessellatedGeometricSet) | The writer starts them as simple records, of type `?` (StepType's default), followed by the components: `?() GEOMETRIC_REPRESENTATION_ITEM() ...` (30 records) | fixed (decision 1503) |
| RWStepAP214_ReadWriteModule::ComplexType and CaseStep (cases 694-697, 705, 706, 800) | ComplexType gives the generic GEOMETRIC_TOLERANCE (or kinematic pair) components, CaseStep wants a specific kind | The components ComplexType gives aren't recognised back (no effect on reading or writing files) | leave |
| RWStepAP214_GeneralModule::NewVoid case 106 | Creates a StepVisual_AnnotationOccurrence for DRAUGHTING_ANNOTATION_OCCURRENCE | Such a record is read into an annotation occurrence and written back ANNOTATION_OCCURRENCE (none in the corpus as a simple record) | touch |
| RWStepAP214_ReadWriteModule, cases 825 and 826 (SurfaceStyleReflectanceAmbientDiffuse(Specular)) and 708 (TESSELLATED_ITEM) | 825 and 826 have no type name in StepType or the type map; 708 is named by StepType but missing from the type map | SURFACE_STYLE_REFLECTANCE_AMBIENT_DIFFUSE(_SPECULAR) can be neither read nor written (written `?`); TESSELLATED_ITEM can be written, not read | fixed (decision 1505) |
| StepElement_VolumeElementPurposeMember and StepElement_SurfaceElementPurposeMember, `CompareNames` | Returns case 1 for both the enumerated purpose and APPLICATION_DEFINED_ELEMENT_PURPOSE (the curve one gives 2) | An application-defined purpose is written back as the enumerated one (2 records) | touch |
| StepBasic_SizeMember | Always names itself POSITIVE_LENGTH_MEASURE | An untyped curve width is written typed (15 records); POSITIVE_LENGTH_MEASURE(LENGTH_MEASURE(x)) keeps its outer type only (16) | touch |
| RWStepVisual_RWCharacterizedObjAndRepresentationAndDraughtingModel::WriteStep | Writes CHARACTERIZED_OBJECT's name and description as derived | `CHARACTERIZED_OBJECT(*,*)` for the read values (3 records) | touch |
| StepAP214_RepItemGroup, StepAP214_ExternallyDefinedClass, StepAP214_ExternallyDefinedGeneralProperty, StepDimTol_CommonDatum: Init with all fields | Call Init on their second supertype's field, which the default constructor leaves null | A null dereference: their readers call it, so such a record is read as an erroneous entity (the reader's per-entity recovery, as OCCT's signal handler); none in the corpus | touch |
| RWStepVisual_RWSurfaceStyleParameterLine::WriteStep, RWStepVisual_RWTextStyleWithBoxCharacteristics::WriteStep | `SW.Send(ent->DirectionCounts())`, `SW.Send(ent->Characteristics())` (arrays of SELECTs) through `Send(occ::handle<Standard_Transient>)` | The array, not an entity of the model, is written `$ /* UNKNOWN REF */` with a fail (none in the corpus) | touch |
| StepFEA_SymmetricTensor43d | Declares SetFeaIsotropicSymmetricTensor43d, never defined | Left out | leave |

## Phase 8: unit 8A3, IFSelect, XSControl and APIHeaderSection_EditHeader

Found by porting IFSelect (as traced) and XSControl and testing them on the toy norm and the corpus's STEP files
(decision 1228). The port reproduces each but the one noted.

| Where | What | Effect | Triage |
|---|---|---|---|
| Interface_CheckTool::VerifyCheckList and CheckList (8A1's code, met here) | One Interface_Check object is cleared and refilled for each entity and recorded for every entity with messages; when the model has semantic checks or report entities, it becomes the model's own check, which the next entity clears | Every recorded check ends with the last entity's messages (none if the last has none): IFSelect_WorkSession::ComputeCheck's semantic checks lose all but the last entity's, and the model's stored checks are cleared by the next lists. Reproduced; the native comparison agrees on the corpus | touch |
| IFSelect_CheckCounter::Analyse | Keeps `thesign->Text(ent, model).ToCString()`, the C string of a temporary destroyed at once | With a signature, the type in each message is garbage or empty natively. The port gives the text (not comparable natively) | leave |
| IFSelect_WorkSession::GiveListCombined | Never creates the list it appends to | A null dereference for any two lists | touch |
| IFSelect_WorkSession::SendAll, SendSelected | Clear `errhand` (a file static, shared by every session) and restore it on an exception only | After one successful call, every session's later calls run without the exception guard. The port keeps the flag per session | settled |
| IFSelect_WorkSession::RemoveName | Re-adds the item to the indexed map of items to drop its name; adding an existing key changes nothing | The item keeps its name as its attribute (HasName, Name, ItemLabel's lists), though the name is unbound | touch |
| IFSelect_WorkSession::ReadFile with OCCT 8's StepSelect_WorkLibrary | The library reads into the model it is given; the session gives a null one | XSControl_Reader::ReadFile on STEP fails (a null dereference caught as a signal: RetFail); STEPControl_Reader::ReadFile makes its model itself | fixed (decision 1506) |
| IFSelect_SignatureList::PrintList | Tests `if (num == IFSelect_ShortByItem)`, an entity number against the enumeration's value 2 | Entity n0 2 is printed "??" in the lists | touch |
| IFSelect_Functions::GiveDispatch | Sets '\0' at the positions of '(' and ')' (TCollection_AsciiString::SetValue) before testing that there are any | A name that is neither a dispatch nor has parentheses raises Standard_OutOfRange | touch |
| XSControl_TransferWriter::CheckList, ResultCheckList | Skip a check `if (NbFails() == 0 \|\| NbWarnings() == 0)` | Only checks with both fails and warnings are listed | touch |
| XSControl_TransferReader::EntitiesFromShapeList (modes 2 to 4) | Reads `list->Value(i)` (the entity number) in the loop over `ir` | The results compared are the wrong ones (or out of range) | touch |
| APIHeaderSection_EditHeader, IsTimeStamp | `[[fallthrough]]` after each separator test into the next digit test | A separator must also be a digit: no time stamp is ever accepted, so the editor can't set one | touch |
| IFSelect_EditForm::NbTouched, IFSelect_ContextModif::Search, IFSelect_IntParam::StaticName | Declared, never defined | The first two left out; StaticName ported | leave |
## Phase 8: unit 8T, the assembly tree and the fresh mesh formats

Nothing of OCCT's is ported in this unit (decision 1229). Two behaviours of XCAF's that `XdeDoc_Document` doesn't
copy, for unit 8G (decision 1231):

| Where | What OCCT does | What the port does | Triage |
|---|---|---|---|
| XCAFDoc_ShapeTool::GetShape | Returns an empty compound for a label marked as an external reference, even one holding a shape (STEPCAFControl_Reader::AddShape marks the extern file's label so) | Records the references; the shape stays | touch |
| XCAFDoc_ShapeTool::AddComponent | Doesn't check that the component doesn't contain the assembly: a cycle makes the shape tool recurse without end | Returns the null label | touch |

## Phase 8: unit 8D, geometry (StepToGeom, GeomToStep)

Found by porting StepToGeom and GeomToStep and by recording them natively (tools/geomsteprec). The port reproduces
each, but for the first (decision 1240).

| Where | What | Effect | Triage |
|---|---|---|---|
| StepToGeom::MakeCurve, MakeCurve2d, MakeSurface | Recurse along replicas, offsets, surface and trimmed curves, guarding only against an entity referring to itself | A cycle of two entities overflows the stack; the port throws InsufficientExecutionStackException (fixed, decision 1240) | settled |
| StepToGeom::MakeCartesianPoint2d, MakeDirection, MakeDirection2d | Don't test their entity for null (MakeCartesianPoint does) | A null dereference for a line or placement without its point or direction | touch |
| StepToGeom (ExtractParameter) | Dereferences MakeCartesianPoint's result, null for a 2D trimming point | A null dereference trimming by a 2D point | touch |
| StepToGeom::MakeBSplineCurve, MakeBSplineCurve2d | Test the closed_curve logical as a C++ bool | UNKNOWN counts as closed (periodicity forced on closed curves) | touch |
| GeomToStep_MakeCurve (Geom2d) | Tests Geom2d_BoundedCurve before Geom2d_TrimmedCurve, a bounded curve: the trimmed-curve branch is never reached | Every 2D trimmed curve goes to MakeBoundedCurve, which doesn't map it: StdFail_NotDone | fixed (decision 1507) |
| GeomToStep_MakeBoundedCurve (Geom2d) | Converts a 2D Bezier curve to a B-spline and writes it with MakeBSplineCurveWithKnots, without the rational case | A rational 2D Bezier curve loses its weights | fixed (decision 1508) |
| GeomToStep_MakeAxis2Placement2d(gp_Ax2) | Writes the 3D location and main direction | An axis2_placement_2d with 3D coordinates | leave (decision 1509) |
| GeomToStep_MakeRectangularTrimmedSurface, MakeConicalSurface | Write angles in degrees (180/π) and in radians, whatever the plane angle factor; StepToGeom reads both with the factor | With the default factors, a rectangular trimmed surface's angular bounds don't read back (degrees read as radians) | fixed (decision 1510) |
| RWStepRepr_RWBooleanRepresentationItem::WriteStep | `SW.Send(theEnt->Value())` with a `bool`: C++ promotes it to `int`, so `Send(int)` is called | BOOLEAN_REPRESENTATION_ITEM's value is written 1 or 0, not .T. or .F., and reading it back (`ReadBoolean`) fails | fixed (decision 1504) |

## Phase 8: unit 8E, StepToTopoDS and TopoDSToStep

| Where | What | Effect | Triage |
|---|---|---|---|
| Transfer_ProcessForTransient/ForFinder::Unbind | Puts `thelastbnd` back in the map, which FindAndMask has just set to the former binder | Nothing is unbound (the roots are updated and true returned); Clean finds no unbound item | touch |
| Transfer_ProcessForTransient/ForFinder::ResultOne, CheckListOne, IsCheckListEmpty, RemoveResult | Their array of indices is never filled since the scopes were removed | An empty iterator and check list, true, and nothing removed, for any mapped object | touch |
| Transfer_ProcessForTransient/ForFinder::Transferring | Root management records `therootl` at the end of the first transfer done, nested ones included | With root management, nested transfers become roots too (innermost first) | touch |
| Transfer_TransferIterator::SelectResult | Compares each item with `ResultType()`, the current item's type, not the item's | Selects on the current item's result type (a crash natively when the current item is multiple) | touch |
| Transfer_Finder::SameAttributes | Documented as sharing the other finder's attributes, but assigns the map (a copy in C++) | The attributes are copied, not shared | touch |
| Transfer_ResultFromModel::Results(1) | Appends the main result before and after the sub-results | The main result is listed twice at level 1 | touch |
| TransferBRep::Shapes(TP, list) | `if (TP.IsNull() && list.IsNull())` where `\|\|` is meant | A crash when only one of them is null | touch |
| TransferBRep_Reader::Transfer, TransferList | Don't clear the former shapes (TransferRoots does), though TransferList's documentation says so | The shapes of successive transfers accumulate | touch |
| TransferBRep::PrintResultInfo | Its message keys (Result.Print.MSG30 to MSG48) are in no message file of OCCT 8 | Prints "Unknown message invoked with the keyword ..." lines | touch |
| XSAlgo_AlgoContainer::ProcessShape | `if (!aSeq) aSeq = thePseq;` never applies: Interface_Static::CVal returns "" for an undeclared static | An undeclared sequence static gives the sequence "": the default ShapeFix on read, `.exec.op` = DirectFaces on write | touch |
| Transfer_ActorOfProcessForTransient/ForFinder | The default constructor leaves `thelast` uninitialised | False under OCCT's default (zeroing) allocator, as in the port | leave |
| Transfer_ActorDispatch::Transfer, Transfer_TransferDispatch::Copy | The actor's TransferEntity copies through the process, which calls the actor again, with nothing bound in between | An endless recursion (a stack overflow) for any entity the dispatch actor transfers; fixed in the port (decision 1207) | settled |

## Phase 8: unit 8G1, the XDE Draw-lite commands

Seen while porting XDEDRAW's commands onto Draw-lite's documents; the commands keep OCCT's behaviour where it shows.

| Where | What | Effect | Triage |
|---|---|---|---|
| TDF_Tool::Label (entry) | Skips the entry's first tag without checking it is 0 | "5:1:1" names 0:1:1, so a shape named "1" or "7:1" passed to an XDE command is taken as a label, not the shape (the port's label lookup does the same) | leave |
| XCAFDoc_ShapeTool::RemoveShape, RemoveComponent | Forget the label's attributes but leave it in the tool's maps (myShapeLabels, mySubShapes) | SearchUsingMap can return a removed label; the port's document forgets the label in its maps too (decision 1258) | settled |
| XDEDRAW_Layers.cxx setVisibility; XDEDRAW.cxx XDumpAssemblyGraph | The usage line misses a space ("Use: XSetVisibilityDocName"); the graph dump's usage names `-verbose`, the command parses `-names` | Messages only; reproduced | leave |

## Phase 8: unit 8F, STEPControl

| Where | What | Effect | Triage |
|---|---|---|---|
| STEPControl_ActorRead (file level) | `NM_DETECTED`, a global flag the actor sets and reads across calls | Not reentrant: readers in parallel share it. A field in the port (decision 1268) | settled |
| STEPControl_ActorRead::TransferEntity (product definition, NAUO) | Follows assembly links without a guard | A file whose assembly links form a cycle overflows the stack. The port checks the stack (decision 1268) | settled |
| STEPConstruct_Assembly::MakeRelationship | Numbers occurrences from a function-static counter | The identifiers written depend on what the process wrote before (decision 1266) | leave |
| STEPControl_Controller (write actor) | The controller recorded process-wide holds the write actor: tolerance, shape fix parameters, product index | A writer's settings leak into the next writer's output; not thread-safe (decision 1264) | settled |
| STEPControl_ActorWrite (non-manifold mode) | `aSubShell.Closed(true)` on the input's shells | The writer changes the shape it writes. A copy in the port (decision 1269) | settled |
| StepSelect_WorkLibrary::WriteFile | `filemod->Label()` after a downcast that may give null | A file modifier of another kind is dereferenced null | touch |
| STEPSelections_SelectDerived::Matches | `atype->SubType(checker)` with `atype` null for an unknown type name | A null dereference; false in the port | settled |
| STEPSelections_SelectGSCurves, SelectInstances | Thread-local state shared by all instances of the class | A selection's result depends on other instances; the selection's own in the port (decision 1270) | settled |
| STEPConstruct_AP203Context | Date, time zone, user name and host address read from the system | AP203 output is not repeatable; settable through the context (decision 1267) | settled |
| STEPControl_Writer (AP203 assemblies) | Native omits the AP203 management entities of the last occurrences of data/occ's assemblies | 12 of 720 writes differ: cause not found, attributed (decision 1272) | leave |
| IFSelect_WorkSession::ModelCheckList (bug25013_25013faces.stp) | Native throws Standard_OutOfRange (NCollection_Sequence::Value) walking the check list after the transfer | The port's check list is complete; attributed (decision 1272) | leave |

## Phase 8: unit 8G2, STEPCAFControl

| Where | What | Effect | Triage |
|---|---|---|---|
| STEPCAFControl_Controller | Replaces the write actor STEPControl_Controller's constructor configures with a fresh STEPCAFControl_ActorWrite, without its shape fix parameters and processing flags | Once STEPCAFControl is initialised (it records itself under "STEP"), STEPControl_Writer in the same process writes shapes unprocessed unless its caller sets the parameters (decision 1277) | touch |
| STEPCAFControl_Writer::ExternFile (by name) | `if (!myFiles.IsEmpty() \|\| !myFiles.IsBound(theName)) return false;` | Fails whenever there are external files: the test is inverted. Kept (decision 1276) | touch |
| STEPCAFControl_Writer (map of styles' representations) | Adds the MDGPRs to the model in the order of a map keyed by shapes' addresses | The entity order of a written file with several styled parts is not repeatable (decision 1280) | leave |
| STEPCAFControl_Reader::ReadNames | A NAUO no PRODUCT_DEFINITION_SHAPE refers to takes the name found for the previous entity, or a null handle at the first | A null dereference for a file whose first NAUO has no shape definition; the port names it "" | settled |
| STEPCAFControl_Reader (bug31670_russian.stp) | A product name of the file is read past its end (bytes of other memory in the name) | The document's name and the text written back differ from run to run; attributed in the comparisons | leave |
| STEPCAFControl_Reader::collectRepresentationItems | Follows representation relationships without a guard | A cycle of relationships recurses without end; the port stops after 1,000 levels (decision 1280) | settled |
| XCAFDoc_ShapeTool::GetExternRefs | Returns the names of the label's children, not the external references SetExternRefs keeps in a child label of its own | Its callers (none in STEPCAFControl) get the components' names of an assembly; the port's GetExternRefs returns the references | touch |
| (port) XSControl_TransferReader::ShapeResult | The port encoded the regularity of a result holding frozen shapes into a copy (decision 1224) | An assembly whose parts were published before was copied whole, so its parts were no longer its product definitions' results and STEPCAFControl_Reader read it as one part (trj4_s1-ai-214.stp written and read back). Fixed: an unfrozen shape holding frozen ones is encoded in place around them; the STEPControl comparisons are unchanged | settled |

## Phase 8: unit 8Z, STEP external references

| Where | What | Effect | Triage |
|---|---|---|---|
| STEPCAFControl_Reader::ReadExternFile | Records a file as read (`myFiles.Bind`) only after its transfer, which reads the file's own references; the "same main file" check compares with the referencing file only | Files referring to each other (A to B to A) recurse without end, a stack overflow. Fixed: the files being read are kept and a reference back to one stays unresolved with a warning (decision 1284-1287) | settled |
| STEPCAFControl_Reader::Transfer, ReadExternFile; STEPConstruct_ExternRefs::FileName | External files read from the disk relative to the loaded file's name or the current directory, or at any absolute or UNC path the file names, without the caller's limits or cancellation, with no way to turn it off or redirect it | A file opens arbitrary paths. The port's resolver, limits and budget (decisions 1284-1287) | settled |
| STEPCAFControl_Writer::Write (multifile) | The external files' names are the labels' names, unchecked | A document's names can place files outside the main file's folder. Kept for Write(path); the stream form hands the names to the caller (decision 1288) | touch |
## Phase 8: unit 8W, the data-exchange corpus

| Where | What | Effect | Triage |
|---|---|---|---|
| DEBREP_Provider::Write (`write.binary : 0`) | Writes the text form with BRepTools::Write, without Draw's `DBRep_DrawableShape` line, while DEBREP_Provider::Read takes a file for text only when it starts with that line | A text BRep file `writefile` wrote is read back as binary by `readfile`, which fails. Kept in Draw-lite's stand-in (decision 1289) | leave |
| DE_ConfigurationContext (WhatKindOfLine) | `!aLine.EndsWith(' ') \|\| !aLine.EndsWith('\t') \|\| !aLine.EndsWith('\n')` is always true | A space is always appended to the line before it is split; harmless (the value is trimmed) | leave |

## Phase 9: unit 9F, the triage of decision 1165's five cases

| Where | What | Effect | Triage |
|---|---|---|---|
| BRepPrimAPI_MakeSweep's copy for write (OCCT.NET's, `KeepUnchangedGenerating`) | Not OCCT's: giving the caller's unchanged sub-shapes back changed the generating shape's copy in place, which the sweep reads again for its history (the seam test of BRepSweep_Rotation::HasShape) | `Generated` built a face from a seam on demand and wrote into the published result (TopoDS_LockedShape, bugs/modalg_6/bug26406); `IsDeleted` of a seam was true (bugs/modalg_7/bug30346_2). **Fixed** (decision 1307) | settled |
| BRepSweep_Rotation, BRepSweep_Translation SetDirectingParameter | A history query builds on demand the edge a vertex generates, and calls `UpdateVertex` on the result's vertices; natively that changes only the edge's ranges | The port threw on the frozen vertex. Port bug, **fixed** (decision 1307): a write that changes nothing is skipped | settled |
| BRepFill_Filling::BuildWires with frozen boundary edges | Not OCCT's: MakeWire moves a wire vertex in place natively, and goes on with a copy of the wire in the port; the filling built its final edges from the caller's edges, not the copies | The face lost an edge (3 for 4, bugs/filling/bug23380). **Fixed** (decision 1308) | settled |
| tests/lowalgos/extss/bug29712_44 | Runs `extrema s1 s1` and `extrema s2 s2` (each plane against itself) and expects 10, the distance between s1 and s2 | Fails natively too (distance 0). Attributed (decision 1309) | leave |
| resources/samples/tcl/ANC101.tcl (demo/samples/anc101) | Blends `_model_27`, then `_model_28` from the same `explode`, an edge the first blend replaced | "There are no suitable edges for chamfer or fillet", natively too on the same Booleans. Attributed (decision 1310) | leave |

## Phase 10: unit 10H5, offsets, fillets and features

| Where | What | Effect | Triage |
|---|---|---|---|
| BRepOffset_MakeOffset `Initialize`, `Clear` | Leave BRepOffset_Analyse's edge types and ancestors from the last run; its next `Perform` appends to them | A second run on one object, on the same shapes, fails (`UnknownError`); the port's copies at the entry hid it (each run had new shapes). The port clears the analysis at `Initialize` (decision 1422) | fix |

## Phase 10: unit 10K3, found while fixing

| Where | What | Effect | Triage |
|---|---|---|---|
| `gp_Trsf::Power`, compound branch (gp_Trsf.cxx; and gp_Trsf2d's, beside its row above) | Squares `Tempscale` before scaling the square's translation with it | Powers of 2 or more of a scaled rotation (or any compound transformation with a scale other than 1) put the translation in the wrong place. Found by unit 10K3's tests of the 2D row | fixed (decision 1485) |
| RWStepVisual_RWSurfaceStyleRendering::ReadStep | The same inverted `strcmp` tests as its subclass's row above | Rendering methods of SURFACE_STYLE_RENDERING read wrong, as SURFACE_STYLE_RENDERING_WITH_PROPERTIES's. Found by unit 10K3 beside that row | fixed (decision 1501) |

## Phase 10: unit 10K4, found by the 2D constraint laws

Bugs the laws of unit 10K4 (decision 1548) found in the classes of batch K4 beyond the rows above, fixed with their
classes.

| Where | What | Effect | Triage |
|---|---|---|---|
| `GccAna_Circ2d3Tan_2.cxx` (circle, two lines), `_6.cxx` (two lines, point) | The flag is not reset when a check fails; a centre on the wrong side of a qualified line is kept; enclosing takes \|r - d\| | Circles not tangent to all arguments, or on the wrong side | fixed (decision 1540) |
| `GccAna_Circ2d3Tan_4.cxx` (two circles, point) | Not done when IntAna2d reports two hyperbolas that don't meet as not done | 42 of 1,500 random cases not done | fixed (decision 1540) |
| `GccAna_Circ2d2TanRad_2.cxx` (circle, point) | Takes the tangency parameter of the unset point of a solution that is the circle | Standard_ConstructionError when the circle is at the origin | fixed (decision 1542) |
| `Geom2dGcc_Lin2dTanObl.cxx` (curve, line, param, angle) | NbrSol is never set for the iterative solution, and its tangency isn't checked | No solution reported; with the parameters fixed, non-tangent lines | fixed (decision 1544) |
| `Geom2dGcc_Circ2d3TanIter.cxx` | par1sol..par3sol are 0 or distances between tangency points | Wrong parameters on the solution (build123d.NET's arcs) | fixed (decision 1546) |
| `Geom2dGcc_Circ2dTanOnRadGeo.cxx` | Offset-curve intersections give circles off the argument or the curve, and on the wrong side of a qualified circle | Wrong solutions (once row 1068 lets curves be found) | fixed (decision 1547) |

## Found by unit 10P2's run of the public members

| Where | What | Effect | Triage |
|---|---|---|---|
| `Geom_TrimmedCurve.cxx`, `Geom2d_TrimmedCurve.cxx`, `Geom_RectangularTrimmedSurface.cxx` (`SetTrim`, `Reverse`) | With a NaN parameter on a basis that is not periodic, `U1 < U2` is false, so the sense is reversed, and `Reverse` calls `SetTrim` with NaN again | Unbounded recursion: the stack overflows | fixed: a NaN is refused (decision 1604) |

## Phase 11: the healing benchmark's failures

| Where | What | Effect | Triage |
|---|---|---|---|
| `ShapeUpgrade_UnifySameDomain.cxx`, `IsMergingPossible` | `Diff1.Angle(Diff2)` on two edges' end tangents, one of zero length (a B-spline with its end pole repeated) | `gp_VectorWithNullMagnitude` out of `Build`, where the checks are compiled in; where they aren't, a NaN angle passes the test and the edges can merge whatever the angle. bug28366_fuselage.stp, t4mec.stp | fixed (decision 1652) |
| `ShapeUpgrade_UnifySameDomain.cxx`, `CheckSharedVertices` | Counts a second missing vertex as a shared vertex, then looks it up in a map that has no null key | `Standard_NoSuchObject` out of `Build` on any shape with two missing vertex ends (one infinite line). bug27333_simple.stp, OCC22459-TEST2.stp | fixed (decision 1653) |
## Found by unit 11N's port of StepTidy

| Where | What | Effect | Triage |
|---|---|---|---|
| `StepTidy_CartesianPointHasher`, `StepTidy_DirectionHasher` | `hashBytes(aCoords.data(), size)` passes the 3 coordinates as 3 bytes | Only low bytes of the first coordinate are hashed: values within the documented 1e-12 rarely merge, and round first coordinates share a bucket | settled (decision 1665: exact merging, with a proper hash) |
| `StepTidy_VectorHasher`, `StepTidy_CircleHasher` | Hash the magnitude and radius by their integer parts | Vectors and circles under 1 share a bucket | settled (decision 1665) |
| `StepTidy_*Hasher` comparisons | `Name()->IsSameString` is called when both names are null | A null dereference | settled (decision 1666) |
| `StepTidy_Axis2Placement3dReducer::replaceCircle`, `replaceEllipse` | `SetValue` on a copy of the conic's position, and returns true | The conic keeps the duplicate, and `AddWithRefs` brings it back | settled (decision 1666) |
| `StepTidy_DirectionReducer::replaceAxis2Placement3d` | Returns after the axis | A duplicate that's also the reference direction stays | settled (decision 1666) |
| `StepTidy_DuplicateCleaner::removeEntities` | The new models have no parameters and no header | The written file names the default schema, whichever was asked for | settled (decision 1666) |
| `STEPCAFControl_Writer` stream write | Only `Write(file)` cleans duplicates | `SetCleanDuplicates` is ignored when writing to a stream | settled (decision 1666) |

## Found by unit 11E

| Where | What | Effect | Status |
|---|---|---|---|
| `BRepBndLib.cxx`, `IsModifySize` and `AdjustFaceBox` | The search for whether a side of a surface's box is reached inside the face uses a plane patch set by the sides cut back before it, and takes one extremum where a whole curve may touch | A box larger than the face: on `shading_172.brep` the offset face's top is 301.6 where its mesh reaches 288.6, and the whole shape's box is 10 units too tall. It still holds the shape. Skipping the sides that couldn't matter made it right by chance, so the port keeps the order | kept, as OCCT (decision 1672) |
| `GeomBndLib_Torus::BoxOptimal` | Searches a surface whose extremes have a closed form, and starts from a coarse grid | About 10 ms a patch, most of a filleted part's optimal box | fixed (decision 1671) |
## Found by unit 11I's coverage tests

| Where | What | Effect | State |
|---|---|---|---|
| `ChFiDS_FilSpine::Reset` | Reads the first and last radii when there may be none (`BRepFilletAPI_MakeFillet::Add(E)`, then `Reset`) | Undefined in a release build | fixed (decision 1676) |
| `ShapeAlgo_AlgoContainer::ApproxBSplineCurve`, both forms | The part after the last repeated pole is never appended | A degree-1 spline with a repeated pole comes back without its end | fixed (decision 1677) |
| `MathLin::Crout` | Inverts its working L in place, then copies it to the result's `L` | The result's `L` is L's inverse, against its comment | fixed (decision 1678) |
| `PLib_JacobiPolynomial::AverageError` | Sums the coefficients only up to `myDegree`, where `MaxError` goes to `myWorkDegree`, and from one index later | Terms above q count for nothing, so the average understates the error. Open for the owner: the port keeps it. Approximations only report it (AdvApprox's average errors), and don't steer by it | leave |
| `gce_MakeLin2d(Lin, Dist)` | The header says a positive distance is to the right of the line; the code adds `Dist * (-dy, dx)`, to the left | Only the comment is wrong, as far as callers go; the port follows the code | leave |
| `BndLib::Add(gp_Cone, ...)` with an infinite V bound | Opens the box along the axis only, though the cone widens without limit | The box doesn't hold the cone. Open for the owner: the port still does this | leave |
| `BndLib::Add(gp_Cylinder, ...)` and `gp_Cone`, both V bounds infinite | Adds nothing but the open sides | The box is still void, so `Get` throws and `IsOut` says yes, though the surface is everywhere along its axis. Open for the owner: the port still does this | leave |

## Found by unit 11I's healing tests

| Where | What | Effect | State |
|---|---|---|---|
| `ShapeConstruct_Curve::ConvertToBSpline`, both forms | A Bezier is trimmed to [first, last] before converting, which reparametrises it over [0, 1], and the converted curve is then segmented to [first, last] as well | Part of a Bezier comes back as a part of that part, on the wrong parameters, in anything joining edges (`JoinCurves`, `JoinPCurves`) | fixed (decision 1713) |
| `ShapeFix_Wire::FixGap3d`, by conversion, on one edge | With a single edge there's no second curve, so "take the curve's end if it couldn't be converted" always applies | The curve closes at its own start, not at the vertex as with two edges. Harmless, as the vertex's tolerance is then set; the port keeps it | kept, with a test |
| `ShapeConstruct_ProjectCurveOnSurface`, a spline with very uneven parameter speed | On an interval 10^5 times as fast as the next, the standard projector's approximation doesn't follow the curve | The pcurve at t = 0 was 1.4 from the curve's start on a cylinder of radius 5, with the status "done"; `SameParameter` raises the tolerance later. Native 8.0.1 gives the same to the digits, 1.413 at t = 0 and 10.04 at t = 0.001 | **improved in the port** (decision 1737): the projector's result is checked on every knot span, and otherwise the sampled path runs, or the projection fails |

## Found by the known defects unit

| Where | What | Effect | State |
|---|---|---|---|
| `ChFi3d_Builder`, a chamfer's corner, equal_dist_sequence/B5 step 2 and five other steps | The curves in the data structure don't close a loop on a face | Natively a face with an open wire: invalid, as OCCT's TODOs expect | **the port refuses** (decision 1735); the chamfer itself isn't fixed |
| `BRepFilletAPI_MakeFillet`, radius 11 on all edges of a 20 mm box | Done, with an invalid shape: native 26 faces and an area of 1.7e47 | The port's engine is invalid too (29 faces, 3686). `Kerf.Modeling` refuses (decision 1740). `FilletOperations.Fillet` still returns it as a success: **for the owner**, whether the kernel's operation should check too | open |

## Found by unit 11P's flaky tests

| Where | What | Effect | Status |
|---|---|---|---|
| tests/bugs/modalg_7/bug31120 | `set ratio (${memval}-{$memval1})/${memval1}` makes a text, not a number, so `if { ${ratio} > 0.05}` compares strings, and `(` sorts before `0` | The check can't fail: run with the heap five times larger it stays silent, so the script doesn't test the leak it names | leave: a test script's. `DrawCaseProcessTests.Repeating_an_offset_keeps_nothing` asks the question (decision 1703) |

## Found by unit 11I's tests of features, intersectors and solvers

Native has gone, so "as OCCT" here comes from reading OCCT's source beside the port's.

| Where | What | Effect | State |
|---|---|---|---|
| `Geom2dGcc_FunctionTanCuCuOnCu`, circle, curve, centre on a circle | The constructor sets `CuCuOnCi`, so `InitDerivative` evaluates the unset first curve | That form of the iterative solver can't run | fixed (decision 1749) |
| `Geom2dGcc_Circ2d2TanOnIter`, curve, point, centre on a curve | Bounds of the centre's parameter and the radius swapped, and the radius read as the centre's parameter after solving | Done only by chance | fixed (decision 1750) |
| `MathRoot::Bisection`, `BisectionNewton` | No check of the ends: with `f(a) = 0`, every midpoint counts as a's sign | Returns the far end as a root, with status OK | fixed (decision 1751) |
| `BVH_DistanceField::Build`, parallel | The trees are built lazily by whichever slice asks first, and building reorders the elements | Wrong distances or an exception on another thread | fixed (decision 1752) |
| `IntCurveSurface_InterImpl::PerformBounds` (bug26525) | Clamps the surface's parameters to 1e50 before `PerformConicSurf` asks `Precision::IsInfinite`, which needs 1e100 | The estimates for infinite surfaces (`EstLimForInfExtr`, `Revl`, `Offs`, `Surf`, `ProjectIntersectAndEstLim`) are dead code. A line parallel to the rulings of an infinite extrusion, which the estimate would have refused, is sampled out to 1e50, and rounding puts spurious points on it with \|W\| from 1e32 up (48 in the test). Harmless below 1e30, and left for the owner: fixing it would change the limits every line against an infinite surface uses | leave |
| `IntCurveSurface_InterUtils`, `DoSurface`, `DoNewBounds` | Called by nothing in OCCT: they served a method of `IntCurveSurface_HInter` that's never called | Dead code | leave |
| `Geom2dGcc_Circ2d3TanIter`, the forms with a point | The point is a circle of radius 0, whose parameter stays at pi since nothing depends on it, and the solution is then checked with the tangent there, (0, -1), against the way from the point to the centre | A correct solution is refused unless the point lies below its centre (with the plane's y up) | leave |
| `Geom2dGcc_Lin2d2TanIter`, circle and curve | `par2sol` is the distance from the first tangency to the second, unsigned, and the line can run the other way | `Tangency2`'s parameter can be on the wrong side of the line's origin; the point is right | leave |
| `LocOpe_Pipe::Curves`, `BarycCurve`, a spine with a line | `GeomConvert` has no form for a whole line, and the swept edges carry one | `BRepFeat_MakePipe::Perform(Until)` and `Perform(From, Until)` throw along any spine with a line in it; the unlimited pipe doesn't ask for the curves | leave |
| `BRepFill_Pipe::PipeLine`, along a circle | Seen on the port only, the same code by reading: the sweep of a point gives no edge, where a Bezier spine gives one | `LocOpe_Pipe::BarycCurve` then reads the first of no curves, so a limited `MakePipe` along an arc throws | leave |
| `BRepFeat::ParametricBarycenter` | The loop over the vertices counts each one in `nbp` without adding its parameter | The mean is biased towards 0: 23.44 for 25 on a box. Features only compare it, so the port keeps it, for the owner | leave |
| `FilletSurf_InternalBuilder::Perform`, `Simulate` | No catch round the fillet's start | A radius too large for its faces throws "StartSol echec", though the class has `IsDone` and `StatusError` for failures | leave |
| `HLRBRep_Algo`, a sphere in perspective | Seen on the port only: its outline is walked rather than analytic | Up to 0.9% short of the circle it should be, and shortest at great focal distances (31.15 against 31.42 at a focus of 10,000) | leave |
| `BRepFeat_MakeCylindricalHole`, `PerformThruNext` on a compound of two solids | Seen on the port only: the drilled solid is cut, and the plug stays in the result as a third solid | The volume doesn't change. On a single solid the same hole is right. Not attributed | open |
| `BRepFeat_MakeRevol::Perform(From, Until)` with plane faces | Seen on the port only: `InvShape` whichever planes were tried | Its 55 lines after the check weren't reached. Not attributed | open |
| `BRepFeat_MakePrism`, `MakeDraftPrism`, mode 2 (the feature alone), with an Until | Seen on the port only: `GlobalPerform` leaves no shape | Only `Perform(length)` gives the feature alone. Not attributed | open |
| `GCPnts_AbscissaPoint::Length`, `CPnts_AbscissaPoint`, `BRepGProp_Cinert` | A few fixed Gauss points over each span: 2 x degree for a Bezier, 10 for an ellipse | 1.5% out on an ellipse of 10 by 1. Fixed for the public API (decision 1768). The engine's own callers keep it, as they only compare and as native comparisons pin what they decide | fixed in public; engine left |
| `BRepPrimAPI_MakeTorus` with a part of the minor circle | Its base class closes any revolved primitive by flat discs to the axis | The outer half of a tube's section gives 210.6 and not the 53.5 a slice would, on R = 5, r = 1. It is valid and documented now, but not what the name suggests. For the owner: a real tube sector would be a new option or a change of meaning | leave |
| `BRepOffsetAPI_MakePipeShell`, a fixed normal lying along the path | Seen on the port: a section turned to contain the path sweeps no volume | A success with a volume of 1e-16 for `Normal` square to a straight path. Not pinned by any test, as unit 11R decides how such results report | open |
## Found or fixed by unit 11S

The list and its ranking are in [occt-defects.md](occt-defects.md).

| Where | What | Effect | State |
|---|---|---|---|
| `GeomGridEval_BSplineSurface`, the cache | Built from a parameter that can locate to the next span | A wrong point just below a knot (#1577) | fixed (decision 1775) |
| `GeomFill_GuideTrihedronAC::D2` | `TN2` for `\|DTN\|^2` | A sweep with a guide distorted (#1681) | fixed (decision 1776) |
| `GeomBndLib_*Surface::BoxOptimal` | Cell centres aren't seen, and the search's gate reads the corners only | A box smaller than the surface (#1604, #1679) | fixed (decision 1777) |
| `gp_Trsf::Invert`, `gp_Trsf2d::Invert` | A point mirror's translation negated | Wrong inverse | fixed from upstream (decision 1778) |
| `Geom_OffsetSurface::Surface` | An indirect plane's offset direction | A plane on the wrong side, and evaluation through it | **fixed** (decision 1811) |
| `BRepBndLib::AddOptimal` | Adds the face's tolerance whatever the flag | (#1698) | fixed (decision 1779) |
| `GeomAdaptor_SurfaceOfRevolution::GetType` | `major > r` with no tolerance | An invalid solid (#1566) | fixed (decision 1780) |
| `ElCLib::ParabolaParameter`, `Extrema_ExtPElC` | Focal 0 isn't the line evaluation makes it | (#1617) | fixed (decision 1781) |
| `gp`, `Geom`, `Geom2d` constructors | NaN passes `r < 0` | (#1618) | fixed for radii and angles; directions **open**, as `ExtPElS` on a cone reaches one (decision 1782) |
| `IntPatch` cylinder/cylinder walk | Status stays "absent" after a step in and out over both V bounds | Empty Boolean (#1633) | fixed (decision 1783) |
| `BRepGProp_Gauss::Compute` | U isn't refined, and its error isn't counted | Wrong precise properties (#1675) | fixed (decision 1784) |
| `GeomBndLib_BSplineCurve2d::Box` | Trims a periodic arc to the first period | A face's area short (#1685) | fixed (decision 1785) |
| `BRepFill_OffsetWire` | Concentric circles reach the medial axis; a collapsed circle falls through | A crash natively (#1606, #1452) | fixed (decision 1788) |
| `BRepFill_OffsetWire`, a positive offset of a face with a hole | Treats each wire alone, so the hole grows | An outward offset's hole is larger, not smaller. The same whether the hole is concentric or not | **open**: to be checked with what a user expects |
| Default sweep (`GeomFill_CorrectedFrenet`) on some planar splines | The sweep's approximation fails where Frenet and discrete succeed: #1630's spine | No pipe; natively a crash | **open**, worked round: the public sweep tries Frenet, then discrete, and warns (decision 1805) |
| `BRepPrimAPI_MakeTorus`, part of the minor circle | Makes the solid under the arc, closed by flat discs to the axis, and not a slice of the tube | 210.6 and not 53.5 on the outer half with R = 5, r = 1 | **kept in the engine for the corpus; the public `Solid.Torus` makes the slice** (decision 1804) |
| OCCT's corpus references for `checkprops -s` | From the path with #1675's bug | blend/simple/J6 expects 79953.8 for a true 77932 | see decision 1784 |
## Found by unit 11R, checking every result
| Where | What | Effect | Status |
|---|---|---|---|
| `ShapeUpgrade_UnifySameDomain`, after a good Boolean (bopfuse_simple ZL7, ZL8, ZL8-fuzzy, ZL9; bfuse_complex H8, I5, N1; bcut_complex B2) | The Boolean is valid and its cleaned result isn't: 8 of 600 cases of the Boolean suites, and a ninth, bcut_complex O5, is invalid before it too | Not compared with native, so whether it's OCCT's or the port's isn't known | **fixed** for 7 of the 8; native is the same on all 8 (measured). bcut_complex B2 is left, an invalid flag on a free face (decision 1816) |
| `ShapeCustom_BSplineRestriction` (`ApproximateBSplines`), a cylinder with its defaults; a sphere through `ToSplines` | Invalid: an edge's curve on its surface, its same-parameter flag and the face's orientation | Inferred from the check's messages: approximating the surface leaves edges off it. Not compared with native | **fixed** in the public operation; native is invalid too (measured). `bsplres` stays as native's (decision 1817) |
| `BRepFilletAPI_MakeFillet2d` (`Chamfer2D`), distance past half a side of the face | Done, with an invalid face | Not compared with native | **Now a failure** |
| `BRepBuilderAPI_MakeFace` on crossing wires; prism and ThruSections on them | Done, with an invalid shape | Found on the port. OCCT doesn't check here either, by its documentation | **Now a failure** |
| `IntPatch_Intersection`, `IntAna_QuadQuadGeo`, a sphere of the minor radius centred on a torus's centre circle (#1567) | Walked, not analytic: five curves of tolerance up to 0.93 | Fuse empty, Cut removes nothing, no failure | **fixed**, and an empty fuse fails (decision 1818) |
| `IntTools_FaceFace::MaxDistance`, far along a hyperbola | A point of `inf - inf` is projected | A NaN direction in `ExtPElS`, on a cone | **fixed**, with directions refusing NaN (decision 1820) |
| `TopoDSToStep_MakeStepFace`, near a horn torus (#1461) | A bare `R < r` | Congruent faces written as different surfaces by noise | **fixed** (decision 1821) |
| `Intf_InterferencePolygon2d::Intersect`, a closed polygon | Skips every pair with its first or last segment | A self-intersection of the first segment lost | **fixed**, from upstream (decision 1822) |
| `AppParCurves_LeastSquare::Init`, the last constraint | Tests `FirstConstraint` for curvature | No difference found in tests | **fixed**, from upstream (decision 1822) |
| `GeomFill::GetCircle` | `dn2w` for `d2n1w` a second time, in the interior poles' second derivatives | Wrong second derivatives over several spans | **fixed** (decision 1815) |
## Found or fixed by unit 11S, third pass

| Where | What | Effect | State |
|---|---|---|---|
| `ShapeBuild_Edge::CopyRanges` | Shifts each periodic curve alone and clears the flags | Unify makes a valid shape invalid (#1340) | fixed in Unify; other callers keep OCCT's (decision 1830) |
| `Geom2dConvert::ConcatC1`, multi-group branch | `Add` without `After` | A closed loop half a turn out in Unify (#1541) | fixed for Unify; other callers keep OCCT's order (decision 1831) |
| `BOPAlgo_Builder::FillImagesCompound` | Orients a solid's splits by its source's orientation | A fuse drops a compound's reversed solids (#1613) | fixed (decision 1832) |
| `IntPatch_ImpPrmIntersection::Perform`, `IntPatch_RstInt::PutVertexOnLine` | Pieces lose their bound vertices; a period counts as apart | A cut removes nothing (#1543) | fixed for pieces crossing the face (decision 1834) |
| `BRepTopAdaptor_TopolTool::Classify` | Keeps the first tolerance's classifier | A reused classifier's answers depend on order (#1692) | fixed (decision 1835) |
| `BRepAlgoAPI_Section`, approximation off | Loses most of a curve on `disc.brp` | A section silently short (#1124) | public default changed (decision 1836); **cause open** |
| `ShapeUpgrade_UnifySameDomain`, a surface closed both ways | A face with no wires | Valid, and no mesh (#1310) | fixed (decision 1837) |
| `StepToGeom::MakeSurface`, offset surface | No surface if the basis stays C0 | A face lost on reading (#994) | fixed (decision 1838) |
| `ChFi3d`, a two-edge corner | A 3D curve 190 from its pcurves, by the shell's face order | Volume 10596 for 29800, valid (#1639) | **open**; public failure (decision 1833) |
| `BRepMesh`, a cone from `Sfacciata.step` | The structure ends with no elements | No triangles on a valid face (#572) | fixed (decision 1930) |
| `ChFi3d_Builder::SplitKPart`, a contact line on a face's boundary | At a radius equal to the face's width, the analytic surface is right, but the contact line lies along the far edge and the hatcher finds 0 domains on it, so `SplitKPart` fails. The general walk then can't start either: "no starting solution". OCCT #1177, #172; upstream's `f713b1305f` accepts it, with 12 of its files in the TopOpeBRepBuild family that Kerf replaced | Radius 10 on a 10 mm cube fails where 9.999999 works | **Worked around, for straight edges between planes** (decisions 1847, 1848); ChFi3d is unchanged. fillet-design.md |
| `ChFi3d`, just below a limit | A radius 1e-8 under a face's width leaves a face 1e-8 wide, below the tolerance, and it passes the check | 7 faces where 5 are right | **Left**, a question for the owner (decision 1851) |
## Found by the check of a Boolean's volume

| What | Where | Status |
|---|---|---|
| The Booleans of `boolean/bopcut_complex/L8` and `M9`, `bugs/modalg_7/bug23187` and `boolean/opensolid/A2` have inputs that aren't valid: a volume integrated from such an input can be far off (L8's tool: 222537, a mesh's 371354), which is why the check asks for valid inputs before it says a rule is broken | Decision 1862 | leave |
| #1629's fuse and common of the block and the tool are also wrong, which the issue doesn't say: the fuse is 8023.19 where the inputs together are 7993.21 (a mesh gives 8023.16), and the common is empty. The cut's solids are 7735.45 and −29.97, so the fuse has the same extra groove | `BOPAlgo`; decision 1863 | open, reported as a failure |
| #1659 in Kerf, narrowed: the face/face intersector gives the whole loop, but the cap's section of the upper face comes in two curves, (795.403, 90.047, 496.186) to (796.105, 90.653, 495.623) and (796.104, 90.653, 495.623) to (819.288, 91.831, 495.402), about 1e-3 apart at the join, with a tolerance of 3e-4. The pave filler makes no pave block from the first, so its 1.07 of section is lost and the loop stays open. The issue took this for the intersection stopping short. Moving the cutter by 0.1 gives one curve. Not fixed: a change to the intersections needs the whole corpus | `BOPAlgo_PaveFiller_6`, `IntPatch` | open, reported as a failure |
| #1643 in Kerf: `Common(body, lip)` is empty, `Cut(body, lip)` is the body and `Fuse(body, lip)` is the body too; the other order is right, 219351.87, 1559.60, 1742218.07. The located sections give the same. The fuse is short of the truth too, by the lip's part outside the body, and no rule can see that. The issue's empty fuse no longer happens | `BOPAlgo`; decision 1866 | open, reported as a failure |
| `IntAna_QuadQuadGeo`, `IntPatch_ImpImpIntersection`, equal cylinders whose axes are under 1e-7 from crossing | Ellipses where the true curves turn away within sqrt(2 R d), so edges through the crossing points disagree with them. On native 8.0.1: a round notch wrong in 88 of 267 steps, with no failure | Fixed where a crossing point is at a face's end (decision 1885); radius 500 still fails |
| `BRepBlend_AppFuncRoot`, `Approx_SweepApproximation` | A blend surface that can't be fitted takes up to 320,000 solves, and the fillet then fails | Bounded at 20,000 (decision 1884) |
| `BRepTopAdaptor_FClass2d`, `IntTools_FClass2d` | A NaN or infinite parameter never returns | Fixed (decision 1883) |

Found by unit 12C (fillet-design.md, section 8), each a loud failure:

| Where | What | Effect | Triage |
|---|---|---|---|
| `ChFi3d_Builder`, a next face within 0.1 radians | Just below the face's width, by up to about 1e-6 at small angles and 1e-8 at 5 degrees, the fillet comes out invalid. **Since unit 12M the public `Fillet` builds these exactly instead (decision 1910),** and the engine's own remains as it was. Above and below that it's right | `result.invalid`, carrying the shape | leave, until a user meets it: it wants a stage of fillet-design.md's |
| `ChFi3d_Builder`, a contact line reaching a tangent edge | From the radius where the contact line meets a tangent edge to as much as 1e-5 above it, it fails, though it's right on both sides | `fillet.*` failure at a radius that should work | leave, as above |
| `BRepOffset_MakeOffset`, a cone with an apex | Offset inwards with round joins, or an inverted cone with either join, gives a compound of faces, called valid, with a volume to 3.7e101 and a box to 3.8 million. With mitres an upright cone is right. Same `BRepOffset::Surface` by reading; native not run | A public success, now a failure holding the shape | Decision 1918; the geometry is left |
## Found by units 12E and 12G

| What | Where | Status |
|---|---|---|
| Two fitted curves meeting at a tiny corner, whose end tangents agree by chance, are glued into a spline with half its length on 1 part in 133 of its parameters. In Kerf, by reading the same code in OCCT (not run on native) | `ShapeUpgrade_UnifySameDomain` | Refused (decision 1922) |
| Points spread by parameter step over a fast stretch of a spline whose speeds are under the limit of 1500, and the pcurve is 0.27 from the curve, with the status "done" and no larger tolerance | `ShapeConstruct_ProjectCurveOnSurface` | Left: the merge that made it is refused; a check changed `de/step_2/T9` (decision 1922) |
| `BRepCheck`'s 23 points by parameter don't see such a pcurve: valid at a tolerance of 1e-7 | `BRepCheck_Edge` | Left |
| `maxDist = 1e4 * tol` for extending a vertex: absolute, where a touch spans sqrt(2 R tol) | `BOPAlgo_PaveFiller::PerformEF` | Scaled by the touch (decision 1923) |
| `sqMagnFDer < 1e-8` has the fourth power of the size in it, so `JoinWLines` decides by curvature at a large radius and by a polygon at a small one | `IntPatch_PointLine::CurvatureRadiusOfIntersLine` | Left |
Found by unit 12J:

| Where | What | Effect | Triage |
|---|---|---|---|
| `BRepExtrema_DistShapeShape` | Only stationary points from a grid: faces crossing along a closed curve inside both aren't found | A positive distance between parts that intersect (#1123) | fixed (decision 1926) |
| `BRepMesh`, a torus's grid | Parameters from the boundary's points, and no control of the deviation | A mesh 1.42 from the surface for 0.45 asked (#1444) | fixed (decision 1929) |
| `BRepMesh`, a sphere's grid | Long triangles beside the seam, and no control of the deviation | 1.69 times the deflection on 3% of the triangles, at some deflections | **open**: the same fix doubles those meshes and grows others by 28%; for the owner (decision 1929) |
| `BRepMesh_ModelPreProcessor`, `SeamEdgeAmplifier` | Chooses a seam's pcurve with the curve's first parameter, far off on a line | A cone with no triangles (#572) | fixed (decision 1930) |
| `IntWalk_IWalking::IsTangentExtCheck` | Close at four points a step away counts as tangent, whatever the sign | A section's closed curve lost, and a wrong Common (#1039) | fixed (decision 1934) |
| `BOPAlgo_PaveFiller::PutBoundPaveOnCurve` | A curve's end a little off a face takes no vertex, so the curve has no pave block, and is lost | An open section and a Cut that removes nothing (#1659) | fixed (decision 1935); the walk stopping short remains |
| `BRepTools_GTrsfModification::NewTriangulation` | Tests a scale that's never negative; takes normals through the matrix with its diagonal 1; ignores the face's placement | A mirrored mesh inside out, its normals wrong, a located mesh moved twice | fixed (decision 1936) |
| `BRepOffset_MakeOffset`, a cone with an apex | Offset inwards with round joins, or an inverted cone with either join, gives a compound of faces, called valid, with a volume to 3.7e101 and a box to 3.8 million. With mitres an upright cone is right. Same `BRepOffset::Surface` by reading; native not run | A public success, now a failure holding the shape; an upright cone's round joins are retried with mitres, which are right there | Decisions 1918 and 1938; the geometry is left, and the inverted cone still fails |

## Found by unit 12S

| Where | What | Effect | Triage |
|---|---|---|---|
| `BRepFill_OffsetWire::Perform`, `CheckBadEdges` | An edge curving tighter than the offset is replaced by a polyline of deflection `0.01 * |offset|`, whose chords lie inside the curve | An inward offset past a curve's sharpest bend up to 1% of the distance off, silently: an ellipse 10 by 5 at -3, 39.556 for 40.035 | **open**, for the owner (decision 23204) |
| `BRepFill_OffsetWire::MakeOffset` | A B-spline edge with a kink (C0) inside it goes to `Geom2d_OffsetCurve`, which refuses it | "Offset on C0 curve": 39 of the 77 offset/wire_* failures, OCCT's known ones; splitting first rescues 4 | leave (decision 23203) |
| `BRepOffsetAPI_MakeOffset`, the tangent join | Not a 2D join OCCT documents; by reading, the code tests `== Arc` in some places and `== Intersection` in others, so the tangent join takes neither branch in some | Nothing outwards from a triangle or star; mitres' results elsewhere | **open**, for the owner (decision 23205) |
