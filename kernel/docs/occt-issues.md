# Suspected bugs in OCCT

The port reproduces OCCT's behaviour, bugs included, so results match the reference. Each suspected bug is
marked in the code with a comment starting `// OCCT` and listed here. Whether to fix one in the port (or report
it upstream) is a later decision, made with the oracle tests showing what changes.

| Where | What | Effect |
|---|---|---|
| `gp_Dir::Mirror(const gp_Ax1&)` (gp_Dir.cxx) | Takes the axis direction's third component from `Y()` instead of `Z()` | Mirroring a direction about an axis is wrong unless the axis's Y and Z components are equal. `gp_Vec`'s and `gp_Pnt`'s versions are correct, so a mirrored axis or system can get inconsistent directions. |
| `gp_Mat2d::Multiply(const gp_Mat2d&)` used as `M.Multiply(M)` (gp_Mat2d.cxx `Power`; gp_Trsf2d.cxx and gp_GTrsf2d.cxx `Power`) | `Multiply` writes `myMat[0][1]` before reading `theOther.myMat[0][1]`; squaring a matrix in place passes itself as `theOther` | Most 2D matrix and transformation powers of 2 or more are wrong. The port reproduces it, because an `in` parameter aliases a local just as a C++ reference does. |
| `gp_Trsf2d::Multiply`, rotation × rotation | Tests this transformation's translation for zero where gp_Trsf tests the other's | When this rotation has no translation, the other rotation's translation is dropped |
| `gp_Trsf2d::Power`, compound branch | Scales only the diagonal (`SetDiagonal(scale*m11, scale*m22)`), then applies the scale again in the loop | Powers of a scaled 2D transformation are wrong |
| `gp_Hypr2d::Asymptote1`, `Asymptote2` | Build the direction from YDirection alone (Y ± (b/a)·Y), where gp_Hypr uses X ± (b/a)·Y | The asymptotes point along Y; `Asymptote2` throws when the radii are equal |
| `gp_Hypr2d::Directrix2` | Placed at Focus1 + (Parameter/e)·X, where gp_Hypr uses Location − (a/e)·X | The second directrix is in the wrong place |
| `gp_GTrsf::Power`, `gp_Other` form | Treats any negative N other than -1 as \|N\|, without inverting | A general transformation raised to a negative power below -1 is not inverted |
| `gp_Hypr::gp_Hypr()` | Sets the minor radius to `RealFirst()`, which is negative and which no constructor or Set method accepts | A default hyperbola is invalid; harmless until it is used |
| `gp_Hypr::Asymptote2` | Its error message says "Asymptote1" | Cosmetic |
| `ElSLib::TorusDN`, mixed branch (Nu > 0 and Nv > 0) | The odd/odd and even/even sub-branches never change sign, where the derivative's sign depends on (Nu + Nv) mod 4 | Mixed torus derivatives are reversed when Nu and Nv are both odd with Nu + Nv ≡ 0 (mod 4), e.g. (1, 3), (3, 1), or both even with Nu + Nv ≡ 2 (mod 4), e.g. (2, 4). (1, 1), (2, 2) and the odd/even orders are right. |
| `math_Matrix::Multiply(const math_Matrix&)` (math_Matrix.lxx) | Writes the product's columns at Right's column indexes | In-place multiplication by a matrix whose column range differs from this one's (non-square, or other bounds) writes the wrong columns or out of range (the port throws `Standard_OutOfRange`) |
| `math_VectorBase<int>::Divided` (math_VectorBase.lxx) | Multiplies by `1. / theRight` converted to the item type | For `math_IntegerVector`, division by any integer other than ±1 gives zeros. Unused in OCCT |
| `math_Uzawa::Perform` (math_Uzawa.cxx) | `Errinit` has as many values as unknowns but is indexed by equation | With more equations than unknowns OCCT writes past the vector (range checks compiled out); the port throws `Standard_OutOfRange` |
| `math_NewtonFunctionSetRoot::FunctionSetErrors` (both overloads) | Return `DeltaX`, the last Newton step, not `FValues` | Callers asking for the function errors get the step |
| `math_Householder(A, B: math_Matrix, EPS)` and `Perform` (math_Householder.cxx) | Q gets A's column range but is indexed from 1; B's columns are indexed from 1; dimensions are checked only after copying B | A with LowerCol ≠ 1, or B with LowerCol ≠ 1, reads out of range; a mismatched B fails on the index check before the `DimensionError` check |
| `math_SVD::PseudoInverse` (math_SVD.cxx) | Writes `Result(i, j)` from 1 whatever Result's bounds | A result matrix with other bounds is wrong or out of range |
| `math_FunctionSetRoot` two-point `MinimizeDirection` (math_FunctionSetRoot.cxx) | Ignores the return value of `F.Value(tsol, fsol)` | On failure `fsol` is uninitialised in C++ (0 in the port) |
| `math_GaussMultipleIntegration` recursion (math_GaussMultipleIntegration.cxx) | Keeps only the status of the last loop iteration | A failure of the function at an earlier Gauss point is ignored: that point is left out of the sum and the result still reports done |
| `math_ComputeKronrodPointsAndWeights(0)` (math_ComputeKronrodPointsAndWeights.cxx) | Reads `b[Number + 1]` past its buffer | Undefined behaviour in C++; the port throws `IndexOutOfRangeException`, which the constructor's `catch (Standard_Failure)` doesn't catch. OCCT never passes 0 |
| `math_FunctionRoots` constructor, maximum with f(x) < 0 (math_FunctionRoots.cxx) | The second estimate repeats the one from Xi-1, where the positive branch uses Xi+1 | The second test can never succeed where the first failed, so some near-roots beside a negative maximum are not re-sampled |
| `math_TrigonometricFunctionRoots::Perform`, "Cas particulier de PI" | Checks for PI already being a solution only on the first pass of the loop | When A−C+E≈0 and PI wasn't found earlier, PI is added once per free slot: duplicate solutions |
| `math_GlobOptMin::ComputeInitSol` (math_GlobOptMin.cxx) | Calls `computeLocalExtremum(aPnt, aVal, aPnt)`: the start and output points are the same vector | A local method that fails after writing a point outside the box leaves the next method starting from that point, so initial solutions depend on the method order |
| `math_NewtonMinimum::IsConvex()` (math_NewtonMinimum.hxx) | Declared, never defined | Calling it fails to link in C++; left out of the port |
| `Bnd_Sphere::Add` (Bnd_Sphere.cxx) | Treats a negative radius as "not initialised yet", but the default constructor sets the radius to 0 | Adding a sphere to a default-constructed `Bnd_Sphere` also encloses the origin instead of copying the other sphere |
| `Bnd_OBB::ReBuild`, two coincident points (Bnd_OBB.cxx) | Calls `ProcessOnePoint`, which resets `myHDims` to 0, then enlarges by `myHDims[1] + Precision::Confusion()` | The point tolerances are dropped: the half-sizes are `Precision::Confusion()` instead of the larger tolerance plus Confusion |
| `NCollection_Mat3::Adjoint` | Documented as the adjugate, returns the cofactor matrix (its transpose); `Inverted` transposes it back | Callers expecting the adjugate get its transpose |
| `NCollection_Mat4::Adjoint` | `crossVec4` returns the negated minors, so the result is the negated cofactor matrix, not the adjugate and not Mat3's convention | Wrong sign and transposition; nothing in OCCT calls it |
| `BVH::SquareDistanceToObject`, `SquareDistanceToGeomerty` (BVH_DistanceField.lxx) | Return `false` (a zero distance) when the tree is null | Dead code (`BVH()` never returns null); cosmetic |
| `MathLin::LU`, `Solve`, `SolveMultiple`, `Invert` (MathLin_Gauss.hxx) | Row indexes are also used as column indexes; `aFirstNonZero == 0` doubles as "no non-zero found yet" | Wrong or out-of-range results when LowerCol ≠ LowerRow; with a row range starting at 0 or below, a first non-zero at index 0 is lost and the forward substitution is wrong |
| `MathLin::SolveCrout` (MathLin_Crout.hxx) | Indexes the inverse's columns from LowerRow, though it has A's column bounds | Wrong or out-of-range results when LowerCol ≠ LowerRow |
| `MathLin::MatrixPower` (MathLin_Jacobi.hxx) | Takes the eigenvector matrix's column indexes from the eigenvalue (row) range | Wrong or out-of-range results when LowerCol ≠ LowerRow |
| `MathLin::OptimalRegularization` (MathLin_LeastSquares.hxx) | Documented as leave-one-out cross-validation, but scores each lambda by the plain residual of the regularized fit | The residual grows with lambda, so the smallest lambda of the grid is (almost) always returned |
| `MathLin::ConditionNumber` (MathLin_SVD.hxx) | Skips zero singular values when taking the minimum | A singular matrix gets a finite condition number instead of the documented infinity |
| MathLin_Jacobi.hxx | Uses `EigenResult` meaning MathUtils::EigenResult, which works only because it doesn't include MathLin_EigenSearch.hxx; included after that header, MathLin::EigenResult hides it | A translation unit including both headers in that order fails to compile; the port writes `MathUtils.EigenResult` |
| `MathPoly::Quintic`, `MathPoly::Sextic` (MathPoly_Laguerre.hxx) | Copy at most 4 real roots into a PolyResult, which holds 4 | A fifth or sixth real root is silently dropped (`MathPoly::Laguerre` returns all of them) |
| `MathRoot::Trigonometric`, A = B = 0 with C ≠ 0 and D ≠ 0 (MathRoot_Trig.hxx) | Tests `aPoly.Status == InfiniteSolutions` after already returning when `!aPoly.IsDone()` | Dead code: InfiniteRoots can't be set on this path. Cosmetic |
| `MathRoot::Trigonometric`, A = E = 0 with C = 0 or D = 0 (MathRoot_Trig.hxx) | Adds roots without the duplicate removal and sorting of the general case | When \|D/2B\| or \|C/2B\| is about 1 the result has duplicate roots (0 or PI/2 twice) and may be unsorted. (These branches also shift a root by 2PI and then by trunc(Mod)·2PI, as math_TrigonometricFunctionRoots does, so a lower bound of 2PI or more may push it out of range; unverified) |
| `MathOpt::NewtonNumericalHessian` (MathOpt_Newton.hxx) via `MathUtils::NumericalHessian` | NumericalHessian fills the matrix from index 1, but Newton's Hessian has the starting point's bounds | With a starting point whose lower bound isn't 1, the Hessian is shifted or out of range (the port throws `Standard_OutOfRange`) |
| `MathOpt::MultiStart` (MathOpt_GlobOpt.hxx) | Divides `MaxIterations` by `NbStarts` unguarded | NbStarts = 0 is undefined behaviour in C++; the port throws `DivideByZeroException` |
| `MathOpt::PSO` (MathOpt_PSO.hxx) | Checks only the upper bounds' length, then indexes them with the lower bounds' indexes | Bounds vectors with different lower indexes are read wrongly or out of range |
| `MathSys::Newton`, `MathSys::NewtonBounded` (MathSys_Newton.hxx) | On success return the Jacobian evaluated at the start of the last iteration, before the final step | `VectorResult.Jacobian` is not the Jacobian at the returned `Solution` |
| `MathSys::LevenbergMarquardtBounded` (MathSys_LevenbergMarquardt.hxx) | The X-convergence test uses the unclamped step, not the step taken after clamping | At an active bound the X criterion never succeeds: the solver stops only on the F or gradient tests or at MaxIterations |
| `MathSys::Solve2D` (MathSys_Newton2D.hxx) | Clamps with `(false, 0.0)`, ignoring `NewtonOptions::AllowSoftBounds` | Soft bounds have no effect in Solve2D, though Solve2DSymmetric, Solve3D and Solve4D honour them |
| `Poly_Triangulation(Nodes, Triangles)` and `(Nodes, UVNodes, Triangles)`; `Poly_PolygonOnTriangulation(Nodes[, Parameters])` | Copy the tables with `NCollection_Array1::operator=`, which takes the source's bounds | A triangle array not starting at 1 gives a triangulation whose `Triangle(i)` is shifted (and out of range at 1) |
| `Poly_ArrayOfNodes`, `Poly_ArrayOfUVNodes` (NCollection_AliasedArray allocation) | Allocate without initialising | Nodes of `Poly_Triangulation(n, m, ...)`, and nodes added by `ResizeNodes` or `AddUVNodes`, are garbage until set; the port's are zero |
| `Poly_Connect` (Poly_Connect.hxx) | Documents the adjacent triangles of T as sharing n2-n3, n3-n1, n1-n2; the code gives them across n1-n2, n2-n3, n3-n1 | Documentation only; the port documents what the code does |
| `Poly_CoherentNode::Poly_CoherentNode()` | Leaves the normal uninitialised | `HasNormal()` of a default node is random; the port's is false |
| `Poly_CoherentNode::Clear` | Frees the list of triangles but keeps the pointer | The node isn't free afterwards and iterating its triangles reads freed memory; the port keeps the list |
| `Poly_CoherentTriangulation::ComputeLinks` | Clears the links without clearing the triangles' pointers to them | Recomputing without `ClearLinks()` first leaves dangling link pointers, and sides that had a link get no new one |
| `Poly_CoherentTriangulation(const occ::handle<Poly_Triangulation>&)` with a null triangulation | Leaves the deflection uninitialised | Random deflection; the port's is 0 |
| `Poly_Polygon3D::Parameters`, `Poly_PolygonOnTriangulation::ChangeParameters` (deprecated) | Dereference a null handle when there are no parameters | Crash in OCCT; the port throws `Standard_NullObject` (Parameters) or a NullReferenceException (the deprecated method) |
| `Poly_MergeNodesTool::MergedNodesMap::hashCode` | Hashes the bytes of the position as `char`, signed on MSVC and unsigned on ARM/GCC | The bucket chains, and so which of several close nodes a new one merges with, can differ between platforms; the port follows MSVC |
| `Poly_MergeNodesTool::Result` with no node added | Calls `ResizeNodes(0)`, which throws (an aliased array can't be empty) | `Standard_RangeError` instead of an empty result, in OCCT and the port |
| `NCollection_AliasedArray::ChangeLast` | Returns `Value<>()`, a const reference, as a non-const one | Doesn't compile if used; nothing in OCCT uses it. The port returns the element for writing |
| `PLib::CoefficientsPoles` (gp_Pnt and gp_Pnt2d overloads, PLib.cxx) | Fills its pole work array from `Coefs`, not `Poles`; only the array's size comes from `Poles` | None when both have the same length, the only meaningful use (the values are overwritten). With fewer poles than coefficients C++ writes past the array; the port throws |
| `BSplCLib::IncreaseDegree`, periodic curves (BSplCLib.cxx) | Not analysed: found by comparing with native OCCT through `Geom_BSplineCurve::IncreaseDegree`, whose poles the port reproduces exactly | For most periodic curves whose knot multiplicities are not all 1 (115 of 200 random ones), the raised curve is a different curve. Geom_BSplineCurve and Geom2d_BSplineCurve inherit it |
| `BSplCLib::Eval` (flat poles, both overloads), extrapolation branch (BSplCLib.cxx) | Sizes the Taylor array for `LocalRequest + 1` rows, then evaluates a polynomial of degree `Degree` (always larger there) on it | Extrapolating beyond the ends with an extrapolation mode below the degree reads uninitialised stack memory. The port reads zeros instead, which gives the Taylor polynomial of order ExtrapMode, apparently the intent |
| `BSplCLib::FunctionMultiply`, `FunctionReparameterise` (flat poles, BSplCLib_2.cxx) | Pass a single `int` as `ExtrapMode` to `Eval`, which reads two | `ExtrapMode[1]` is read past the variable, but only consulted for a parameter beyond the last knot, which the Schoenberg points never are. The port passes the degree for both |
| `BSplCLib::InsertKnots` (flat poles, BSplCLib.cxx) | `np = NewKnots.Lower() + (index + 1) * Dimension` uses the knots' lower bound for an index into the poles | Wrong pole placement when the two lower bounds differ; harmless when both are 1, as in all OCCT callers |
| `BSplCLib::IncreaseDegree` (flat poles, BSplCLib.cxx) | Reads `wpoles(NewPoles.Lower() + ...)`, but `wpoles` is indexed from 1 | Wrong when NewPoles.Lower() ≠ 1 |
| `BSplCLib::MovePointAndTangent` (flat poles, BSplCLib_2.cxx) | Chooses the second bump function by comparing U with `FlatKnots(index)` and `FlatKnots(index + 1)`, where `index` indexes the Schoenberg points | The second bump function may peak on the farther Schoenberg point: the deformation is less local than intended, but valid |
| `BSplCLib::MergeBSplineKnots` (BSplCLib_2.cxx) | Indexes `Mults1` and `Mults2` from 1, whatever their lower bounds | Wrong multiplicities for arrays not starting at 1 (OCCT's range checks are compiled out); the port throws |
| `BSplCLib::DN(U, N, Poles, Weights, P, VN)` (Bezier, BSplCLib.hxx) | Declared, never defined | Calling it fails to link in C++; left out of the port |
| `CSLib::Normal`, the overload with `theMaxOrder` and `theDerNUV` (CSLib.cxx) | In the sign-change loop, `aFirst = aIntervalIdx` is 0 for the first interval, so the next interval also takes the "first" branch, which resets the sign state | A sign change between the first two root intervals is never detected: `CSLib_Defined` with a normal can be reported where `CSLib_InfinityOfSolutions` is due |
| `TopAbs::Print(TopAbs_State, Standard_OStream&)` (TopAbs.cxx) | Its name table is in the order ON, IN, OUT, UNKNOWN, but TopAbs_State is IN, OUT, ON, UNKNOWN | IN prints "ON", OUT prints "IN", ON prints "OUT" |
| `Geom_AxisPlacement::SetAxis` on a `Geom_Axis2Placement` | Only assigns the axis; Geom_Axis2Placement doesn't redefine it, although the documentation says the X and Y directions are recomputed (and that a parallel X direction throws) | The X and Y directions are left unchanged, no longer perpendicular to the new main direction |
| `Adaptor3d_HVertex::Adaptor3d_HVertex()` | Leaves the orientation uninitialised | `Orientation()` of a default vertex is random; the port's is FORWARD |
| `BSplCLib::PrepareInsertKnots` with `Add` false (BSplCLib.cxx) | Counts equal entries of AddKnots once, with the first entry's multiplicity, where `BSplCLib::InsertKnots` processes each entry | The predicted pole and knot counts can be wrong. Example: a periodic degree-7 curve, its last knot added three times with multiplicities 2, 2 and 6: 30 poles predicted, unchanged, but InsertKnots raises both end multiplicities to 6 and the shape is lost. `Geom_BSplineCurve::InsertKnots` hides it by returning early when the pole count doesn't change. Found by the oracle tests |
| `BSplSLib::CoefsD0/D1/D2` (BSplSLib.lxx) | Pass `Poles.RowLength() - 1` (the V degree) as UDegree and `ColLength() - 1` as VDegree, while `PolesCoefficients` returns the coefficients indexed (U, V) like the poles | With at least as many U poles as V poles they evaluate the Bezier surface at (V, U), the U and V derivatives exchanged; with fewer U poles the result is meaningless. No OCCT code calls them |
| `BSplSLib::CacheD2`, rational (BSplSLib.cxx) | Zeroes the work array above row `MinIndMax` for the poles' dimension, then reuses it for the weights with the smaller dimension `min_degree + 1` | When both degrees are below 2, the weights' third row holds pole values, taken as the second derivative of the weights: the second derivatives (and `CoefsD2`'s) of a rational bilinear surface are wrong. BSplSLib_Cache, which OCCT's own evaluators use, is not affected |
| `BSplSLib::CacheD0/D1/D2` (BSplSLib.cxx) | Take the cache's start as `PolesArray(LowerCol(), LowerRow())`, row and column swapped | Harmless for every cache OCCT builds (both lower bounds 1); wrong or out of range otherwise |
| `BSplSLib::IsRational` (BSplSLib.cxx) | Takes the default tolerance from `Weights(I1, I2)`, a row bound used as a column index, instead of `Weights(I1, J1)`; compares the weights along U only | A tolerance from the wrong weight, out of the row's range when I2 exceeds the column count (OCCT's release build reads that storage element unchecked; the port reads the same element) |
| `BSplSLib::FunctionMultiply` (BSplSLib.cxx) | Calls `HomogeneousD0` with URat = VRat = true whatever `Weights` is | A null `Weights` is dereferenced (crash in OCCT, `NullReferenceException` in the port) |
| `BSplSLib::MovePoint` (BSplSLib.cxx) | When [UIndex1, UIndex2] (or the V window) holds none of the poles acting at (U, V), `ukk1` (`vkk1`) stays 0 and `UBSplineBasis(1, ukk1 - UFirstNonZeroBsplineIndex + 2)` is read out of the matrix's range | OCCT's release build reads past the basis matrix and goes on with a meaningless pole choice; the port throws `Standard_OutOfRange`. Seen through `Geom_BSplineSurface::MovePoint` with a window away from the point |
| `Convert_ConicToBSplineCurve::BuildCosAndSin` (Convert_ConicToBSplineCurve.cxx) | For a `Convert_ParameterisationType` outside the enumeration, calls through a null function pointer | Crash in OCCT; `NullReferenceException` in the port. Only reachable with an invalid enum value |
| `GeomEval_TBezierSurface::IsUClosed`, `IsVClosed` | Compare the first and last rows (columns) of poles. In the trigonometric basis these are the constant and the highest cosine terms; the boundary iso-curves are S(0, v) = Σ (P0j + P2j + P4j + ...) Bv_j and S(π/α, v) = Σ (P0j − P2j + P4j − ...) Bv_j | The result says nothing about closure: a surface can be reported closed when it isn't (test `IsClosed_ComparesPoleRows_OcctIssue`), and the reverse |
| `GeomEval_AHTBezierCurve`, `Geom2dEval_AHTBezierCurve`, `GeomEval_AHTBezierSurface`, non-rational constructors | Allocate a one-element weights array (`myWeights(1, 1)`, `(1, 1, 1, 1)`) and leave it uninitialised; `Weights()` returns it | A non-rational curve or surface reports one random weight; the port's is 0. A non-rational TBezier returns an empty array instead |
| `AdvApp2Var_MathBase::mdsptpt_` (AdvApp2Var_MathBase.cxx) | The work array `differ` is a null pointer (the Fortran had `DIFFER(100)`); it is allocated only when NDIMEN > 100 | For NDIMEN <= 100 C++ dereferences null. Nothing in OCCT calls it. The port throws `IndexOutOfRangeException` |
| `AdvApp2Var_MathBase::mmresol_` (AdvApp2Var_MathBase.cxx) | Its work arrays (`v1` to `v4`, `hchole`, `matsym`, `mcho`) are null pointers that `macrar8_` allocates only above 100 elements | Any dimension <= 100 dereferences null. Nothing in OCCT calls it. The port throws |
| `AdvApp2Var_MathBase::mmfmtb1_` (AdvApp2Var_MathBase.cxx) | Fills only the first JSIZE1 elements of each line of its work zone, then copies MAXSZ2 per line into TABLE2 | When MAXSZ2 > JSIZE1, the unused part of TABLE2 gets uninitialised heap memory. The port, whose zones are zeroed, writes zeros |
| `mmcmher_`, the Hermite cache of `AdvApp2Var_MathBase::mmherm0_`/`mmherm1_` (AdvApp2Var_MathBase.cxx) | A static struct shared by every thread; `mmherm0_` zeroes then refills it while `mmherm1_` reads it on the strength of three keys written earlier | Concurrent approximations race on it: a thread can read a half-filled table (at the first use, or when threads use different intervals). The port keeps one cache per thread (decision 113) |
| `MyDirectPolynomialRoots` (IntAna2d_Outils.cxx) | The extra candidate roots named `MATH_A210` are those of `A3 x² + A2 x + A1`, not `A2 x² + A1 x + A0` as the name and its siblings (A4321, A3210) suggest | When the quartic's roots look unreliable, the wrong quadratic's roots are added as candidates; they are usually sorted out by their residuals |
| `IntAna2d_AnaIntersection::Perform(gp_Circ2d, gp_Circ2d)` (IntAna2d_AnaIntersection_2.cxx) | Measures both parameters counterclockwise from each circle's X axis (`gp_Vec2d::Angle`), whatever the circle's sense; the external-tangency parameter on the second circle is also not brought into [0, 2PI[ | `ParamOnFirst`/`ParamOnSecond` are wrong on an indirect circle (reproduced by a unit test; the oracle shows native OCCT gives the same) |
| `IntAna2d_AnaIntersection::Perform(gp_Hypr2d, IntAna2d_Conic)` and `(gp_Parab2d, ...)` | `MyDirectPolynomialRoots` reports "not done" when the quartic has no real root; the hyperbola case then returns `IsDone() == false`, while the parabola case, lacking a `return`, ends with `done = true` | A hyperbola that misses the conic gives "not done" rather than an empty result; a parabola gives an empty result |
| `Extrema_ExtPElC2d::Perform` for ellipses, hyperbolas and parabolas (Extrema_ExtPElC2d.cxx) | Sets `myIsMin = (NoSol == 0)` in a loop where NoSol starts at 1; for circles it sets true for both extrema | `IsMin` is always false for these conics, and true for the far point of a circle. The 3D Extrema_ExtPElC computes IsMin properly |
| `Extrema_ExtElSS::Points` (Extrema_ExtElSS.cxx) | No Perform fills `myPOnS1`/`myPOnS2` | `Points` dereferences null handles: a crash in OCCT for two parallel planes (the only case with a result); a NullReferenceException in the port. The other surface pairs throw Standard_NotImplemented |
| `Extrema_ExtElC(gp_Lin, gp_Lin, AngTol)`, `(gp_Lin, gp_Circ, Tol)`; `Extrema_ExtElC2d(gp_Lin2d, gp_Lin2d, AngTol)`, `(gp_Lin2d, gp_Circ2d, Tol)`; `Extrema_ExtPElC::Perform(gp_Parab)`; `Extrema_ExtPElS::Perform(gp_Pln)` | Ignore their tolerance argument, though the line-line ones document it as the parallelism test | Precision::Angular() (or no tolerance) is used whatever the caller passes |
| `gce_MakeCirc(P1, P2, P3)` (gce_MakeCirc.cxx) | Returns without setting `TheError` when the three points coincide, and when the computed center is within gp::Resolution() of P1 | The status is uninitialised; the port's is gce_Done (decision 118) |
| `gce_MakeCirc2d(P1, P2, P3)` (gce_MakeCirc2d.cxx) | Doesn't set `TheError` when the bisectors don't intersect (distinct collinear points) | The status is uninitialised, and `Value()` may return a default circle as a success; the port reports gce_IntersectionError (decision 118) |
| `gce_MakeCirc(Center, gp_Pln, Radius)` (gce_MakeCirc.cxx) | Calls `Value()` on its inner construction without checking it | With a negative radius, StdFail_NotDone in debug builds; the release build copies the default circle and reports gce_NegativeRadius. The port does the latter (decision 119) |
| `gce_MakeLin2d(const gp_Lin2d&, double Dist)` (gce_MakeLin2d.hxx) | Documented as placing the line on the right of `Lin` for a positive distance; the code offsets along the left normal (-Dy, Dx) | Documentation only; the port documents what the code does |
| `Geom_BSplineCurve::EvalD0` to `EvalD3` (Geom_BSplineCurve_1.cxx) | Normalize a copy of U with `PeriodicNormalization`, then pass U itself to `LocateParameter`, which overwrites the copy | None: `LocateParameter` handles periodicity itself. Dead code |
| `Geom_BSplineCurve::LocalDN` | Its error message says "LocalD3" | Cosmetic |
| `Geom_BezierCurve::InsertPoleAfter`, `InsertPoleBefore` | Refuse a curve of `MaxDegree()` = 25 poles (`nbpoles >= MaxDegree()`), though the constructors and `Increase` accept MaxDegree() + 1 = 26 poles | A pole can't be inserted to reach degree 25 |
| `Geom_BezierCurve(Poles, Weights)` and its `Rational` helper | Index the weights from 1 whatever the array's lower bound | Weights not indexed from 1 are read shifted or out of range (range checks compiled out in OCCT; the port throws) |
| `Geom2d_OffsetCurveUtils::CalculateD3` and `Geom_OffsetCurveUtils::CalculateD3` (Geom2d_OffsetCurveUtils.pxx, Geom_OffsetCurveUtils.pxx: the same code, both branches) | The second derivative it returns subtracts the `Ndir` term that `CalculateD2` adds, and the third derivative's `DNdir` and `Ndir` coefficients are not the derivative of the second (`-3(D2r/R³ + Dr²/R⁵)` for `-3(D2r/R³ - 3Dr²/R⁵)`; `6Dr²/R⁵ + 6Dr·D2r/R⁵ - 15Dr³/R⁷ - D3r` for `9Dr·D2r/R⁵ - 15Dr³/R⁷ - D3r/R³`) | `Geom2d_OffsetCurve::EvalD3` and `Geom_OffsetCurve::EvalD3` (so `D3`, `DN(U, 3)`), `Adaptor2d_OffsetCurve::D3` and (through `EvaluateD3`) GeomAdaptor's offset curves return wrong D2 and D3 wherever the basis curve's speed varies; right on circles and lines. Offset 0.5 of an ellipse (3, 1) at u = 0.7: EvalD3's D2 is (-2.597, -3.131) where EvalD2's is (-1.794, -1.102), and its D3 is (8.29, 11.12) where finite differences give (0.627, 1.171); the same values in 3D for the ellipse in the XY plane offset along Z |
| `Geom2d_BSplineCurve` modifiers (Geom2d_BSplineCurve.cxx, _1.cxx) | None of them clears an attached evaluation representation, where every `Geom_BSplineCurve` modifier that changes the curve calls `ClearEvalRepresentation` (Geom_BSplineCurve.cxx, _1.cxx) | After `Transform`, `SetPole`, `InsertKnot`, ... a 2D curve still evaluates through the old representation, although its poles and knots changed; a 3D one drops it. The ports keep each family's behaviour (tests pin both) |
| `Geom2d_BezierCurve::SetPole`, `SetWeight`, `Reverse`, `Segment` and the same in `Geom_BezierCurve` | Don't clear the evaluation representation; only `init` (constructors, `Increase`, pole insertion and removal) and `Transform` do, in both families | The modified curve still evaluates through the old representation (tests pin it in both families) |
| `Geom2d_BSplineCurve::IncreaseDegree` (non-periodic) | Sizes the new poles as `NbPoles + Step * (LastUKnotIndex - FirstUKnotIndex)`, counting only the knots of the parameter range, while `BSplCLib::IncreaseDegree` raises the end multiplicities too | On a curve not clamped at its ends the result is inconsistent: degree 3, multiplicities (4, 2, 2, 2, 2) and 8 poles give degree 4, multiplicities (5, 3, 3, 3, 5) and 11 poles where 14 are needed (native OCCT gives the same curve). Unclamped at both ends, `BSplCLib::IncreaseDegree` writes past the poles (Standard_OutOfRange in the port and in native OCCT, whose array checks are on) |
| `Geom2d_TrimmedCurve::SetTrim`; `Geom2d_BSplineCurve::LocalDN` | Error messages name `Geom_TrimmedCurve` and `LocalD3` | Cosmetic |
| `Geom2d_BezierCurve(Poles, Weights)` and its `Rational` helper (Geom2d_BezierCurve.cxx) | Index the weights from 1, whatever the array's lower bound | Weights indexed from another bound are read out of range (garbage in OCCT's release build; Standard_OutOfRange in the port) |
| `AppParCurves::SecondDerivativeBernstein` (AppParCurves.cxx) | Allocates `B(1, deg - 1)` and writes `B(1)` whatever the degree | For degree 1 the write is past an empty vector (unchecked in OCCT's release build); the port allocates one element. The result is right either way |
| `AppParCurves_MultiCurve::Pole`, `Pole2d` (AppParCurves_MultiCurve.cxx) | Range check `(CuIndex <= 0) && Nieme <= 0`, `&&` where `\|\|` is meant | The check almost never fires; the MultiPoint's own check catches a bad curve index |
| `AppParCurves_LeastSquare::Init` (AppParCurves_LeastSquare.gxx) | The test that fixes the last pole ends with `FirstConstraint == AppParCurves_CurvaturePoint` where `LastConstraint` is meant | A curvature constraint at the first point also copies the last point into the last pole; harmless when the last point is unconstrained, since that pole is then solved for |
| `AppParCurves_ResolConstraint` constructor (AppParCurves_ResolConstraint.gxx) | The second member of a 2d curve reads `tabP2d(i - nb3d)`, the passing point index `i` where the curve `k - nb3d` is meant | Out of bounds (garbage in OCCT's release build, Standard_OutOfRange in the port) unless `i - nb3d` happens to be a 2d curve index: interior constraints on 2d curves are broken |
| `AppParCurves_ResolConstraint` constructor, tangency relations (AppParCurves_ResolConstraint.gxx) | For a 3d curve whose largest tangent component is Y, `T1 = V.Y()` is missing | The relation between the curves' tangents uses the previous T1; only with two or more curves and an interior tangency |
| `AppParCurves_ResolConstraint::ConstraintDerivative` (AppParCurves_ResolConstraint.gxx) | Indexes the 2d tangents by `k` where `k - nb3d` is meant, and writes the tangent relations' derivatives into `Cont` (the constraint matrix) instead of `DeCont`; T1 and T2 are uninitialised | Out of bounds for 2d curves after 3d ones (the port throws); with two or more curves and an interior tangency, the constraint matrix is overwritten by derivatives and the derivative matrix lacks those rows |
| `AppParCurves_Function` and `AppParCurves_BSpFunction` constructors (AppParCurves_Function.gxx, AppParCurves_BSpFunction.gxx) | With interior constraints, read the 2d points as `TabP2d(j)`, the curve index, where `j - nb3d` is meant | Out of bounds for a line with 3d and 2d points (garbage in OCCT, Standard_OutOfRange in the port): constrained approximation of such lines doesn't work |
| `AppParCurves_Function::Perform`, constrained gradient (AppParCurves_Function.gxx) | `DPTCO(j) -= RESTM(j, k) * DPTCO1(Pi, j + Inc)` reads column `j` where `k` is meant; the correction term is `Scal = ...` where `+=` is meant, and its z part uses `Erruzax` | The gradient of F with interior constraints is wrong (only the last pole counts, x stands in for z); the BFGS minimisation still runs, stopping on the error tolerances |
| `AdvApprox_SimpleApprox::Perform` (AdvApprox_SimpleApprox.cxx) | With an odd number of Gauss points (15 or 25), the loop adding the t = 0 contribution doesn't restart `Sum` for each coefficient | Each even Jacobi coefficient is a running sum carried over from the previous one: an error of about 0.9 on sin(3u) where 20 or 30 points give 1e-15. AdvApprox_ApproxAFunction always uses 30 or more points; only direct callers of SimpleApprox, or the static `Approximation` with code ≤ -3, are affected |
| `FEmTool_Curve::GetElement` (FEmTool_Curve.cxx) | Rescales the derivative columns `j1 + 1 .. myDimension`, with `j1 = Coeffs.LowerCol() - 1`, where `j1 + myDimension` is meant | With LowerCol > 1 the last columns aren't rescaled; with LowerCol < 1 OCCT writes out of bounds (the port throws). AppDef passes 1-based arrays |
| `FEmTool_Assembly::AddConstraint` (FEmTool_Assembly.cxx) | When the line already has segments and the new index range sticks out past every segment at both ends, `Coeff` stays null and is dereferenced | Crash in OCCT; NullReferenceException in the port |
| `GProp_CelGProps`, `GProp_SelGProps`, `GProp_VelGProps` (all `Perform` overloads) | Build the inertia matrix as P·V·Λ (each eigenvector scaled by its value) instead of P·V·Λ·Vᵀ·Pᵀ; `V.Multiply(Passage, V)` also reads V while writing it | Wrong, non-symmetric inertia matrices (an axis-aligned full circle gets (1,3) = 2πR³, (2,1) = (3,2) = πR³ and a zero diagonal). No OCCT code uses these classes |
| `GProp_CelGProps`, `GProp_SelGProps`, `GProp_VelGProps` | Store the centre of mass `g` as an absolute point, while GProp_GProps reads it relative to `loc` | `CentreOfMass` is shifted by the location point when it isn't the origin: a unit circle at the origin, with location (1, 2, 3), reports its centre at (1, 2, 3) |
| `GProp_CelGProps::Perform(gp_Lin)` (GProp_CelGProps.cxx) | Computes Izz with Ixx's coefficients, where `alfa2 = Vx·X0 + Vy·Y0` and `alfa3 = X0² + Y0²` are meant | For a rod along X from (1, 2, 3) of length 6, the central Izz is 30 where it should be 18 |
| `GProp_CelGProps::Perform(gp_Circ)` (GProp_CelGProps.cxx) | Uses the centre's Z0 for the z of the centre of mass | Ignores the tilt of the circle's plane (covered by the inertia and centre issues above) |
| `GProp_SelGProps::Perform(gp_Cone)` (GProp_SelGProps.cxx) | The area has an extra cos(SemiAngle) factor, and ISn2 is a copy of ICn2 | Wrong area, and the sin·cos inertia terms have the wrong sign |
| `GProp_VelGProps::Perform(gp_Cone)` (GProp_VelGProps.cxx) | Computes π L² cos t sin t (2R + L sin t) as the volume, not the frustum's; ISn2 is again a copy of ICn2 | Wrong volume and inertia |
| `GProp_VelGProps::Perform(gp_Cylinder)` (GProp_VelGProps.cxx) | The axial moment is R²·dim where R²·dim/2 is meant | Axial inertia twice too large |
| `GProp_VelGProps::Perform(gp_Torus)` (GProp_VelGProps.cxx) | Reads cos(Alpha2) and sin(Alpha2) as Cnt2 and Snt2 where Teta2 is meant | With Teta in [0, π] and Alpha full, the centre stays at y = 0 (SelGProps gives 2R/π) |
| `IntAna_Curve::InternalUVValue` (IntAna_Curve.cxx) | The derivative `aDA` has `aCos2t * (Z2CosSin * Z2CosSin)` where `aDB` and `aDC` have `(Z?CosSin + Z?CosSin)` | Only the discriminant tolerance `aTolD` is affected |
| `IntAna_IntQuadQuad::InternalSetNextAndPrevious` (IntAna_IntQuadQuad.cxx) | `for (c2 = 0; (c2 < NbCurves) && (c2 != c1); c2++)` stops at c2 == c1 instead of skipping it | Each curve is compared only with the curves before it. Every pair is still tested once and the updates are symmetric, so the links are probably the same |
| `TrigonometricRoots` (IntAna_IntQuadQuad.cxx) | `NbRoots` is left uninitialised when math_TrigonometricFunctionRoots fails or has infinite roots | `IsARoot` reads it; harmless while callers check `IsDone` first. 0 in the port |
| `IntAna_IntQuadQuad::NbPnt`, `Point`, `Parameters` | `Thepoints`/`Nbpoints` are never filled (the code that would is commented out), and `Parameters` only prints "not yet implemented" | `NbPnt` is always 0; `Parameters` returns nothing |
| `AxeOperator::Distance` (IntAna_QuadQuadGeo.cxx) | Leaves its outputs unset when the axes are parallel | Its one caller (skew cylinder/cylinder) never passes parallel axes. 0 in the port |
| `Extrema_ExtElCS::Points` (Extrema_ExtElCS.cxx) | A parallel result (`IsParallel`) fills only the square distance | `Points` then reads null handles (a crash) or the arrays of an earlier Perform on the same object; the port throws NullReferenceException or returns those stale points likewise |
| `ProjLib_Cone::Project(const gp_Circ&)` (ProjLib_Cone.cxx) | Sets the type to `GeomAbs_Line` before checking that the circle is normal to the axis | For a tilted circle `IsDone()` is false but `Line()` returns the default line instead of throwing, and `ProjLib::Project(gp_Cone, gp_Circ)` returns it silently |
| `ProjLib_Projector::ProjLib_Projector()` | Documented as setting the type to OtherCurve; sets BSplineCurve | Documentation only |

## Phase 2: unit 1C, Geom surfaces

| Where | What | Effect |
|---|---|---|
| `Geom_ConicalSurface::Coefficients` (Geom_ConicalSurface.cxx) | Computes C1-C3 as `T1j * T14 + T2j * T24 + radius * T3j` and D with `+ 2.0 * radius * T34`, where gp_Cone::Coefficients has `- T3j * (radius + T34)` and `- 2.0 * radius * T34` (T3j scaled by tan(semiAngle)) | The implicit equation is wrong unless the cone's origin is the global origin and its reference radius is 0: the surface's points don't satisfy it. The port computes what OCCT computes (oracle-tested); a unit test shows the error |
| `Geom_RectangularTrimmedSurface(S, Param1, Param2, UTrim, Sense)` (Geom_RectangularTrimmedSurface.cxx) | Calls `SetTrim(Param1, Param2, UTrim, Sense)`, which reads `isutrimmed`/`isvtrimmed` (and the other direction's trims), before anything initialises them | Undefined in C++; OCCT's allocator clears new objects by default (MMGT_CLEAR), so they read false, as the port's defaults do |
| `Geom_BSplineSurface::MovePoint` on a periodic surface (Geom_BSplineSurface_1.cxx) | Passes the periodic flat knots and the stored poles to `BSplSLib::MovePoint`, which indexes poles and weights by the B-spline basis index, beyond NbPoles near the end of a periodic direction | OCCT's release build reads (and moves) storage past the pole and weight arrays; the port throws `Standard_OutOfRange`. Found by the oracle tests on V-periodic surfaces |
| `Geom_BSplineSurface::CheckAndSegment` over the whole range of a periodic direction (Geom_BSplineSurface.cxx, `segment`) | The direction is not segmented, but the knots are still rebuilt from `index1U` to `index2U`, and `NewU1 + deltaU` (one period on) locates on the first knot again, so `index2U` becomes `index1U + 1` | The direction keeps two knots and stays periodic with all its poles: knots and poles no longer match. Evaluation then throws (`PeriodicNormalization`) or reads out of range, and the iso-curves (closure tests) fail to build. Example: a U-periodic degree-1 surface with 6 knots of multiplicity 1, `CheckAndSegment` on its whole parametric rectangle |
| `Geom_BSplineSurface::RemoveUKnot`, `RemoveVKnot` on a periodic direction (Geom_BSplineSurface_1.cxx) | Accept `Index` equal to FirstUKnotIndex (1) or LastUKnotIndex, the knots that close the period | Removing one leaves knots and multiplicities inconsistent with periodicity; `BSplCLib::KnotSequence` then reads out of range (OCCT's release build reads past the array; the port throws `IndexOutOfRangeException`). Example: V-periodic degree 2, knots multiplicities (2, 1, 2), `RemoveVKnot(3, 0, 100)` |
| `Geom_BezierSurface` edits (Geom_BezierSurface.cxx) | Only `Transform`, `UReverse` and `VReverse` clear the evaluation representation; `SetPole`, `SetWeight`, `Increase`, `Segment`, `ExchangeUV`, the pole row and column edits keep it (Geom_BSplineSurface clears it on every edit) | A Bezier surface with an evaluation representation goes on evaluating the old shape after those edits |
| `Geom_BezierSurface::Increase`, V direction (Geom_BezierSurface.cxx) | Passes `UKnots()` as the V knots | Harmless: a Bezier surface's knots are (0, 1) in both directions |
| `Geom_BezierSurface::VIso` (Geom_BezierSurface.cxx) | Passes `UKnots()` (with `VMultiplicities()`) to `BSplSLib::Iso` for the V direction | Harmless, as for `Increase`: the knots are (0, 1) in both directions. The port passes the same arrays |
| `Adaptor3d_CurveOnSurface::EvalD3`, general path (Adaptor3d_CurveOnSurface.cxx, file-static `SetLinearForm`) | The third derivative of S(u(t), v(t)) weights `D1U` by u' (`DW.X()`) where the chain rule has u''' (`D3W.X()`), and gives `D3UVV` the coefficient 5u'v'² instead of 3u'v'² (the second `V32.SetLinearForm` repeats the `DW.X()*DW.Y()*DW.Y()` term, and V32 is then doubled) | `D3` and `DN(U, 3)` of a curve on a surface are wrong outside the line and circle special cases, whenever u' ≠ u''' or the surface has a non-zero D3UVV. A 2D line of slope (1, 2)/√5 on a cylinder of radius 2: D3 is off by u'·Su, about 0.9, where a finite difference of D2 agrees with the chain rule (`Adaptor3d_CurveOnSurfaceTests.EvalD3_OcctFormula_DiffersFromDerivative`) |
| `Adaptor3d_CurveOnSurface.cxx`, file-static `Hunt` | When the coordinate matches no knot, the loop leaves `i = Arr.Upper() + 1` and `Arr(i)` reads past the array (release build) before throwing `Standard_NotImplemented` | Not reachable from the callers, which only pass knots; the port would throw IndexOutOfRangeException there |
| `Adaptor3d_TopolTool::Classify` and `IsThePointOn`, infinite lower bound (Adaptor3d_TopolTool.cxx) | U is OUT when `U >= Usup + Tol`, V when `V > Vsup + Tol` | A point exactly at the upper bound + Tol is OUT in U and ON in V. Harmless in practice |
| `Adaptor3d_TopolTool::BSplSamplePnts` | With fewer than 4 candidate parameters in a direction (e.g. degree 1, one span, minimum below 4), `nn = nbsu / 4` is 0 and no parameter is flagged, but the count is still set to 4 | The sample parameters end in zeros (e.g. [0, 1, 0, 0]) and the samples include parameter 0 where no knot is |
| `Adaptor3d_TopolTool(const handle<Adaptor3d_Surface>&)` | Leaves `myNbSamplesV`, `nbVtx` and `idVtx` uninitialised | Harmless: nothing reads them before they are set (C# zeroes them) |

## Phase 2: unit 2E, constructions and sampling

| Where | What | Effect |
|---|---|---|
| `GC_MakeSegment(const gp_Pnt&, const gp_Pnt&)` (GC_MakeSegment.cxx) | Converts `GC_MakeLine(P1, P2)` to a handle without checking its status | With coincident points OCCT's release build passes a null line to Geom_TrimmedCurve and crashes; the port throws StdFail_NotDone |
| `GC_MakeArcOfCircle2d(P1, V, P2)` (GC_MakeArcOfCircle2d.cxx) | Converts `gce_MakeLin2d(P1, P2)` without checking its status | With P1 = P2 the release build goes on with the default line, finds the center at P1 with radius 0 and computes NaN parameters; the port throws Standard_ConstructionError from `ElCLib::Parameter` |
| `GCPnts_QuasiUniformAbscissa::initialize` (GCPnts_QuasiUniformAbscissa.cxx) | Reads `NbPoints()` and `Parameter()` of its GCPnts_UniformAbscissa without checking `IsDone()` | The debug build throws StdFail_NotDone on curves other than Bezier and B-spline whose abscissa computation fails (a zero-length curve); the release build copies the failed result. The port copies it, as the release build does |
| `GCPnts_TangentialDeflection::initialize` (GCPnts_TangentialDeflection.cxx) | `Standard_ConstructionError_Raise_if(...)` is followed by `myParameters.Clear();` without a semicolon in between | Harmless: both expansions of the macro still clear the parameters |
| `GCPnts_UniformDeflection::Value`, `GCPnts_QuasiUniformDeflection::Value` | Their StdFail_NotDone messages name `GCPnts_UniformAbscissa::Parameter()` and `GCPnts_QuasiUniformAbscissa::Parameter()` | Cosmetic |

## Phase 2: unit 2D, AdvApp2Var

| Where | What | Effect |
|---|---|---|
| `AdvApp2Var_ApproxF2var::mma2ce1_` (AdvApp2Var_ApproxF2var.cxx) | Calls `mma2jmx_` for U and for V with the same output `&wrkar_off[ipt5]`, where the U maxima belong at `ipt4` | The maxima of the U Jacobi polynomials are never computed and stay 0 (the work zone is calloc'ed), so every estimate of the error of dropping coefficients (`mma2er1_`, `mma2er2_`, which multiply by them) is 0. Degree reduction is then unchecked and `MaxError` underestimates: sin(u)·cos(v) on [0, 2π]², C0, tolerance 1e-4, gives IsDone with MaxError 1.03e-5 for a surface of degree 1 in V that is more than 1 away from the function (native OCCT and the port alike; `SinCos_C0_ReproducesOcctsUnderestimatedError`). GeomConvert_ApproxSurface inherits it |
| `AdvApp2Var_ApproxF2var::mma2ce1_`, the same calls | The U call writes `ndjacu + 1 - 2(iordru + 1)` values at `ipt5`, where `ndjacv + 1 - 2(iordrv + 1)` plus `4·ndimen` remain | When the U Jacobi degree is much larger than the V one (C0 in U with MaxDegInU 14, C2 in V with MaxDegInV 6, precision code 1), OCCT writes past its work zone and corrupts the heap; native OCCT crashed on every such case. The port keeps the stray values in the zone (decision 198) |
| `AdvApp2Var_Iso::MakeApprox`, `AdvApp2Var_Patch` (Discretise, AddConstraints, MakeApprox) | Read and write three coordinates per point, and pass 3 as the subspace dimension, whatever the dimension (OCCT documents "works only in 3D") | With 1D or 2D subspaces OCCT reads and writes outside its arrays; the port throws (decision 196) |
| `Approx_BSplComputeLine::FindRealConstraints` (Approx_BSplComputeLine.gxx) | For a CurvaturePoint end constraint, tests `LineTool::Tangency` a second time where `Curvature` is meant | A line with tangents but no curvatures gets CurvaturePoint ends (more poles counted, curvature constraints asked of the least squares) |
| `Approx_ComputeLine` CheckMultiCurve, 2d case (Approx_ComputeLine.gxx) | Normalises the first pole vector by `sqrt(aSqNormToler)` (Epsilon(1)), not by its norm | The first scalar product is scaled by about 1e16, so any turn of more than 90 degrees between the first two pole segments counts as a loop |
| `Approx_ComputeLine` CheckMultiCurve (Approx_ComputeLine.gxx) | `int indbads[4]` is written at `indbads[indcur]` for every curve | Past the array for a line with more than three curves (one 3d and three 2d, or four 2d): undefined in OCCT, IndexOutOfRangeException in the port |
| `Approx_ComputeLine` constructors (Approx_ComputeLine.gxx) | `tolreached` is never initialised | `IsToleranceReached()` before any approximation sets it returns garbage; false in the port |
| `FEmTool_Curve::D2` (FEmTool_Curve.cxx) | For an element of degree 1 (which `ReduceDegree` allows at C0), evaluates a polynomial of degree -1: the Dimension coefficients just before the element's second-derivative block | The previous element's highest coefficients, or memory before the array for the first element (uninitialised in either case unless computed): native C0 variational approximations (GeomAPI_PointsToBSpline with weights) change from run to run. The port reads the same coefficients and zeros before the array (decision 206) |
| `AppCont_LeastSquare` constructor (AppCont_LeastSquare.cxx) | `myDone` is set only after solving a constrained system | `IsDone()` is false when both ends are free (the poles are computed) and when the constraints fix every pole; Approx_ComputeCLine copies it, so an unconstrained continuous approximation reports "not done" |
| `AppCont_LeastSquare` constructor, period jump at the last point (AppCont_LeastSquare.cxx) | The loop runs `aDimIdx <= 2` where the first point's runs to `aMaxDim` | Periodic coordinates after the second aren't unwrapped at the last pole for a PassPoint or TangencyPoint last constraint |
| `AppDef_Variational::EstTangent`, `EstSecnd`, `ACR` (AppDef_Variational.cxx) | Test `myTypConstraints(2*IdCnt-1) < ipnt` before `IdCnt <= NbConstr` | Reads index 2*NbConstr+1, past the constraints, for every point after the last constraint (ACR reads constraint NbCntr+1). The port tests the count first (decision 208) |
| `AppDef_Variational::Project`, `TheMotor` (AppDef_Variational.cxx) | `NumPoints` is set only where a distance is positive, then shifted by `myFirstPoint - 1` on every call; TheMotor never initialises it | An uninitialised (or doubly shifted) index of the farthest point when no distance is positive; the port starts at 0 |
| `AppDef_Variational::EstTangent`, last point (AppDef_Variational.cxx) | Addresses `myTabPoints` as `(myLastPoint - 3) * myDimension + 1` | Wrong point unless `myFirstPoint == 1` |
| `AppDef_Variational::InitCriterionEstimations` (AppDef_Variational.cxx) | Calls `EstTangent(myFirstPoint + 2)` for a line of two points | Reads past the points; the port throws Standard_OutOfRange |
| `AppDef_Variational::Init` (AppDef_Variational.cxx) | Never resets `myIsOverConstr` to false | After an over-constrained setup, `SetConstraints` with feasible constraints returns false and `IsOverConstrained()` stays true, although the object approximates |
| `AppDef_LinearCriteria::Gradient` (AppDef_LinearCriteria.cxx) | Takes `&myCache->Value(1)` before its loop, even for an element without points | Dereferences a null cache; the port reads the cache inside the loop only |

## Phase 2: unit 2A, offset surfaces and GeomAdaptor

| Where | What | Effect |
|---|---|---|
| `Geom_OffsetSurface::TransformParameters` (Geom_OffsetSurface.cxx) | Transforms the parameters with the basis surface, then again with the equivalent surface (the evaluation representation) when there is one, although the offset surface has the basis surface's parametrization | Parameters are transformed twice: for an offset of a plane and a scale of 2, U and V are multiplied by 4; it disagrees with `ParametricTransformation`, which uses the basis only. Confirmed against native OCCT (oracle); a unit test pins it |
| `Geom_OffsetSurface::UReverse`, `VReverse`, `Transform` (Geom_OffsetSurface.cxx) | Clear the evaluation representation (the equivalent plane, cylinder, ... built from `Surface()`) and don't rebuild it, where `SetOffsetValue` and `SetBasisSurface` do | After a reversal or a transformation, the surface is evaluated through the general offset formula instead of the equivalent surface (equal to rounding), `UIso`/`VIso` return AdvApprox approximations instead of exact lines and circles, and `TransformParameters` stops transforming twice |
| `GeomAdaptor_SurfaceOfLinearExtrusion::VTrim`, `UTrim` (GeomAdaptor_SurfaceOfLinearExtrusion.cxx) | `VTrim` trims the basis curve, which carries U; `UTrim` ignores its parameters and returns the untrimmed extrusion | Trimming in V restricts U instead (V stays unbounded), and trimming in U does nothing. A unit test pins it |

## Phase 2: unit 3B, curve extrema

| Where | What | Effect |
|---|---|---|
| `Extrema_GGenExtCC::Perform` (Extrema_GGenExtCC.hxx; Extrema_ECC, Extrema_ECC2d) | Never clears `myPoints1`/`myPoints2` | A second `Perform` on the same object appends its solutions to the first one's (`Extrema_CurvePortTests.ECC_PerformTwice_AccumulatesSolutions_AsInOcct`). Extrema_ExtCC keeps one Extrema_ECC and calls `Perform` on it at each of its own `Perform`s, so an Extrema_ExtCC reused (Initialize + Perform) on curves taking the ECC path reports the earlier curves' solutions too (those inside the new ranges pass its filter) |
| `Extrema_GenLocateExtCC` constructor (Extrema_GenLocateExtCC.hxx; Extrema_LocECC, Extrema_LocECC2d) | Accepts the root when `Ff(1) < Tolf && Ff(2) < Tolf`, the signed function values, not their absolute values | Any negative residual passes: a root the solver didn't converge to is reported as done when both functions are negative there |
| `Extrema_ExtCC2d::Results(const Extrema_ExtElC2d&, ...)` (Extrema_ExtCC2d.cxx) | Calls `AlgExt.IsParallel()` before testing `AlgExt.IsDone()` | When Extrema_ExtElC2d is not done (a circle and a concentric ellipse of equal radii), Extrema_ExtCC2d throws StdFail_NotDone (a real throw in OCCT's release build too) instead of reporting not done (`Extrema_CurvePortTests.ExtCC2d_NotDoneElementary_Throws_AsInOcct`) |
| `Extrema_ExtCC::PrepareParallelResult`, circle/circle (Extrema_ExtCC.cxx) | `continue` when `ExtPCir.NbExt() < 1` skips `aProjRng1.Shift(M_PI)` at the end of the loop body | The next iterations test the same projected range again instead of the shifted ones. Reached only when Extrema_ExtPElC finds no extremum (a NotDone Extrema_ExtPElC throws from `NbExt` first) |
| `Extrema_CurveTool::DeflCurvIntervals`, `Extrema_Curve2dTool::DeflCurvIntervals` | `L += aP.Distance(aP1)` without moving `aP` to `aP1` | L is the sum of the distances from the first point to 22 others, not the length of the polygon, so the deflection and minimum length given to GCPnts_TangentialDeflection (and the "short curve" test) are not what the comments say |
| `Extrema_GFuncExtPC::Value` and `SearchOfTolerance` (Extrema_GFuncExtPC.hxx) | Test only `D1c.X()` and `D1c.Y()` for infinity | A 3D first derivative that is infinite only in Z is not caught. Harmless in practice |
## Phase 2: unit 3C, properties, persistence, hashes

| Where | What | Effect |
|---|---|---|
| `LProp_SurfaceUtils::RequireCurvature`, `IsUmbilic` (GeomLProp_SurfaceUtils.hxx), called by `GeomLProp_SLPropsBase::MaxCurvature`, `MinCurvature`, `MeanCurvature`, `GaussianCurvature`, `IsUmbilic` | Take the curvature fields by value, evaluated before the call to `IsCurvatureDefined()` that computes them | Asked before `IsCurvatureDefined()`, the curvatures are the previous parameters' (0 on a new object: a sphere of radius 2 reports MaxCurvature 0, then -0.5), and IsUmbilic compares the previous values (true on a new object). Confirmed against native BRepLProp_SLProps (oracle, 4,315 cases) |
| `GeomLProp::Continuity` (GeomLProp.cxx) | For a B-spline at an inner knot, sets `cont = Degree() - Multiplicity()`, the order of continuity there, and then reads it on `GeomAbsToInteger`'s scale (2 for C1, 4 for C2) | A junction at a knot is reported lower than it is: a cubic's simple knot (C2) compares first derivatives only and gives at most C1, a double knot (C1) compares nothing and gives C0. A unit test pins it (not bound natively) |
| `LProp_CurveUtils::CentreOfCurvature` (LProp_CurveUtils.hxx) | When the first derivative is null, `Curvature()` returns RealLast() without updating `myCurvature`, and the centre is computed from it and from the null normal D2 (D1.D1) - D1 (D1.D2) | Release builds compute a NaN centre with a stale curvature (0 on a new object); the port's `gp_Vec::Normalize` throws Standard_ConstructionError |
| `GeomLProp_CurAndInf2d::Perform` (GeomLProp_CurAndInf2d.cxx) | Runs the curvature-extrema search and then the inflection search, each of which sets `myIsDone` | A failure of the curvature search on a B-spline or other curve is hidden when the inflection search succeeds |
| `GeomTools_Curve2dSet::Add`, `GeomTools_SurfaceSet::Add` and the three sets' `Read` (GeomTools_*Set.cxx) | Add a null handle to the indexed map, which keeps one null entry; `Read` adds the null result of every object it can't read | When several objects of a table can't be read, they share one index and every later object gets an index one lower per extra failure, so the shapes of a `.brep` file refer to the wrong geometry. `Dump` and `Write` then dereference the null entry |
| `GeomTools_UndefinedTypeHandler::PrintCurve`, `PrintCurve2d`, `PrintSurface` (GeomTools_UndefinedTypeHandler.cxx) | In the compact (file) form, write their message to std::cout instead of the stream | A table with an object of an unknown type is written with fewer objects than its count says, so it can't be read back |
| `GeomTools_*Set` readers of trimmed, offset and swept objects | Pass a null basis (one that couldn't be read) to the constructors, which dereference it | A crash unless signals are converted to exceptions (`OCC_CATCH_SIGNALS`); the port reads the object as null |
| `GeomHash_BSplineCurveHasher`, `Geom2dHash_BSplineCurveHasher`, `GeomHash_BSplineSurfaceHasher` comparison (.pxx) | Compare degree, knots, multiplicities and rationality, but neither periodicity nor pole counts, then loop over the first object's poles | With the same knots, a periodic B-spline has more poles than a non-periodic one: comparing the non-periodic one first finds them equal after its own poles; comparing the periodic one first reads past the other's poles (undefined behaviour in a `noexcept` operator). The port's `Pole` throws Standard_OutOfRange. A unit test pins it |
## Phase 2: unit 3A, grid evaluation

| Where | What | Effect |
|---|---|---|
| `GeomGridEval_BSplineCurve::EvaluateGridDN`, `Geom2dGridEval_BSplineCurve::EvaluateGridDN` | Locate the span in the flat knots (`BSplCLib::LocateParameter` without multiplicities), then pass that flat index to `BSplCLib::DN` together with the knots and multiplicities, where it is read as a knot index; `LocateParameter` keeps a hint that is a valid knot index | DN is evaluated on another span's polynomial, extrapolated to U (errors of order 1); when the flat index is past the knots, building the span's knots reads outside the array (garbage in OCCT, IndexOutOfRangeException in the port). Surfaces of revolution and extrusion of B-spline curves inherit it in their grid DN. Pinned by `BSplineDN_UsesFlatSpanIndexWithKnots` (3D and 2D) |
| `EvaluateGridDN` of GeomGridEval_BezierCurve, _BSplineCurve, _BezierSurface, _BSplineSurface, Geom2dGridEval_BezierCurve, _BSplineCurve | Return zero when the order exceeds the degree (in U or V for surfaces), also for rational geometry | A rational curve's or surface's derivatives above the degree are in general not zero (Geom and the adaptors return them): the grid differs by 100%. Pinned by `RationalDN_AboveDegreeIsZero` (curves 3D and 2D, B-spline surface) and in the Bezier surface consistency test |
| `Geom2dGridEval_Parabola` (every method) | Builds the Y direction as the mirror axis turned by +90 degrees, ignoring the sense of the parabola's gp_Ax22d | A parabola with an indirect frame is evaluated mirrored (errors of order 1). Pinned by `IndirectParabola_GridIsMirrored`. The 3D evaluator takes the frame's Y direction and is right |
| `GeomGridEval_Parabola`, `Geom2dGridEval_Parabola` (every method) | Divide by the focal length (1/(4F), 1/(2F)); Geom_Parabola and Geom2d_Parabola allow a focal length of 0, which ElCLib evaluates as the line Location + U·XDir | The grid of a zero-focal parabola is NaN at U = 0 and infinite elsewhere. Pinned by `ZeroFocalParabola_GridIsNotFinite` (3D and 2D) |
| `GeomGridEval_OffsetCurve::EvaluateGridDN`, orders 1 to 3 | Reads the D1, D2 or D3 grid without checking that it is not empty; the grid is empty when the offset fails at a parameter (basis tangent zero or parallel to the offset direction). The 2D evaluator checks | Reads an empty array: undefined in OCCT, Standard_OutOfRange in the port. Pinned by `OffsetDN_ReadsEmptyGridAfterFailedOffset` |
| `Geom_OffsetCurveUtils::EvaluateDN`, `Geom2d_OffsetCurveUtils::EvaluateDN` (both instantiations), case 3 (Geom_OffsetCurveUtils.pxx, Geom2d_OffsetCurveUtils.pxx) | Calls `EvaluateD3(..., aPnt, aDummy, aDummy, theDN)`: one variable receives both D1 and D2, so the basis D2 overwrites the basis D1 before CalculateD3 uses them | GeomAdaptor_Curve's and Geom2dAdaptor_Curve's DN(3) on an offset curve differ from Geom_OffsetCurve/Geom2d_OffsetCurve's (up to 1.9 relative) and can throw an undefined-derivative exception. Found by the grid consistency tests, which compare offset DN3 with the curve only |
| `GeomConvert::ConcatG1`, `Geom2dConvert::ConcatG1` and `Geom2dConvert::ConcatC1` (GeomConvert.cxx, Geom2dConvert.cxx) | With ClosedFlag set and more than one G1 group, they reorder the curves and a local copy of the tolerances, then merge each multi-curve group with the caller's un-reordered `ArrayOfToler(j-1)` (GeomConvert::ConcatC1 reads the local copy) | Each merge uses another junction's tolerance, and the closure is never merged with ClosedTolerance: segments (0,0)→(1,0)→(2,0)→(2,1)→(1e-5,0) with tolerances {1e-7, 1e-12, 1e-7} and ClosedTolerance 1e-4 throw "Concatenation Error" (`ConcatG1_ClosedWire_ReadsTheCallersTolerances_OCCT`, 2D and 3D) |
| `Geom2dConvert::C0BSplineToArrayOfC1BSplineCurve` (Geom2dConvert.cxx) | Tests closure with `SquareDistance < Tolerance`, where C0BSplineToC1BSplineCurve uses `Tolerance * Tolerance` | A curve whose ends are farther apart than Tolerance (but within its square root) is taken as closed |
| `Geom2dConvert_ApproxArcsSegments::makeApproximation` (Geom2dConvert_ApproxArcsSegments.cxx) | Appends makeLine's result even when it is null (two parameters farther apart than PConfusion whose points are within Confusion) | `GetResult()` can hold a null curve; the port leaves it out. Found by reading; 3,000 random curves never reached it |
| `Geom2dConvert_reparameterise_evaluator` / `GeomConvert_reparameterise_evaluator` (Geom2dConvert.cxx, GeomConvert.cxx) | Pass a single `double&` as PLib::EvalPolynomial's result array | Would write past it if a derivative were requested; BSplCLib::FunctionReparameterise only asks for order 0, so latent |
| `GeomConvert::SplitBSplineSurface(S, FromParam1, ToParam2, USplit = false, ...)` (GeomConvert_1.cxx) | The V branch tests `S->IsUPeriodic()` and calls `UReverse()` | A split in V with FromParam1 > ToParam2 (or SameOrientation false on a U-periodic surface) reverses U instead of V (`SplitBSplineSurface_InV_ReversesU_OCCT`; the oracle agrees) |
| `GeomConvert_BSplineSurfaceKnotSplitting` (GeomConvert_BSplineSurfaceKnotSplitting.cxx) | When the surface already has continuity VContinuityRange in V, assigns `usplitIndexes` where it means `vsplitIndexes` | The U result is overwritten with the first and last V knot indices, and `NbVSplits`, `VSplitValue` and `Splitting` dereference a null handle (the port throws NullReferenceException; `KnotSplitting_SmoothV_ReproducesOcctError`) |
| `GeomConvert_CompBezierSurfacesToBSplineSurface(Beziers, Tolerance, RemoveKnots)` (GeomConvert_CompBezierSurfacesToBSplineSurface.cxx) | The V knot removal loop tests `multU` (left over from the U loop), not `multV` | No V knot is removed when the U removals ended normally or with one U patch (`CompBezierSurfaces_WithTolerance_RemovesUKnotsOnly`). If the last U removal failed with multU > minus, the V loop runs until a removal fails; once a V knot is removed entirely, RemoveVKnot is called with a negative multiplicity, returns true, and the loop never ends (OCCT and the port; not tested) |
| `GeomConvert_FuncCylinderLSDist::Gradient` (GeomConvert_FuncCylinderLSDist.cxx) | Scales `G(6) *= -2R` where it means `G(4)` | OCCT's release build writes past the 4-element vector and returns an unscaled radius derivative; the port throws on the index (`Cylinder_ValueValues_AndGradientReproducesOcctError`). `Value`/`Values`' value is right, and SurfToAnaSurf only calls `Value` |
| `GeomConvert_CurveToAnaCurve::ComputeEllipse` through `ConicDefinition` (GeomConvert_CurveToAnaCurve.cxx) | Takes `cos2t = 1/sqrt(1 + t2d²)` and `sint = sqrt((1 - cos2t)/2)` both positive | The sign of the rotation is lost: the major axis always comes out within [0°, 45°] of the fitting frame's X axis, so ellipses oriented otherwise are rejected at any tolerance (an arc of radii 3 and 1.2 turned in 15° steps is recognised at 0–30° and 180–210° only; `ComputeEllipse_MajorAxisAt60Degrees_ReproducesOcctError`). By reading and the port's behaviour; no native binding reaches it. Probably affects ShapeAnalysis_CanonicalRecognition::IsEllipse |

## Phase 2: unit 4A, surface extrema

| Where | What | Effect |
|---|---|---|
| `Extrema_GenExtPS` tree search, `Bnd_SphereUBTreeSelectorMax` (Extrema_GenExtPS.cxx) | Starts `myMaxDist` at 0 and rejects with `Bnd_Sphere::IsOut(P, myMaxDist)`, which rejects a sphere whose minimal distance exceeds that value (the test of the minimum selector) | Every sphere is rejected, so the search for the maximum (Extrema_ExtAlgo_Tree with MAX or MINMAX) refines from the first grid node, not the farthest one, and misses the maximum. Pinned by `GenExtPS_TreeMax_StartsFromTheFirstNode_AsInOcct` |
| `CorrectNbSamples` (Extrema_GenExtPS.cxx) | When the U steps are more than ten times the V steps, multiplies `theNbV` again, where `theNbU` was meant | The tree search's sampling is refined in V instead of U for surfaces much longer in U |
| `Extrema_ExtPRevS::Initialize` | Resets `myIsAnalyticallyComputable` to false before `if (myS != theS)`, which alone sets it | Initialising again with the same surface (handle) loses the analytic path: Extrema_GenExtPS (then initialised) answers instead. Extrema_ExtPS never does this |
| `Extrema_ExtPS::TrimmedSquareDistances` | Returns `d11` ... `P22`, which nothing computes | Zeros and the origin after the default constructor, uninitialised memory after the others (zeros in the port) |
| `Extrema_GlobOptFuncCQuadric::QuadricParameters`, default branch | Writes `theUV(2) = myUl` where `myVf` was presumably meant | On a surface that is not a quadric (LoadQuad never called with one), V is the last U parameter. GenExtCS calls it only on quadrics |
| `Extrema_ExtSS` on parallel surfaces (Extrema_ExtSS.cxx) | `NbExt()` is 1 (the distance is kept) but no points are stored | `Points(1, ...)` reads past empty sequences (undefined in OCCT's release build, Standard_OutOfRange in the port); BRepExtrema_ExtFF tests `IsParallel` first. Pinned by `ExtSS_ParallelPlanes` |
| `Extrema_ExtSS::Perform` | Resets `myIsPar` only on the elementary path (only `Initialize` resets it otherwise) | After a parallel result, a later Perform taking the generic path (Extrema_GenExtSS) with the same Initialize still reports parallel |

## Phase 2: unit 4B, bounding boxes

| Where | What | Effect |
|---|---|---|
| `BndLib::Add(gp_Parab, P1, P2, ...)`, `BndLib::Add(gp_Parab2d, ...)`, `BndLib::Add(gp_Hypr2d, ...)`, finite arcs (BndLib.cxx) | Add only the two ends and, when P1·P2 < 0, the vertex (parameter 0); the coordinate extrema of a parabola or hyperbola turned in its plane are elsewhere. The 3D hyperbola adds them (`ComputeBox`) | The box does not contain the arc: a parabola with axes (1,1)/√2, (−1,1)/√2, focal 2 on [−6, 6] gets Xmin −1.0607 where the curve reaches −1.4142; a 2D hyperbola (3, 2) at 45 degrees on [−1, 1] gets Xmin 1.6114 for 1.5811. BndLib_Add3dCurve/Add2dCurve go through GeomBndLib, not these functions; direct callers are affected |
| `GeomBndLib_Cone::Box` with an infinite V bound (GeomBndLib_Cone.cxx) | Adds the circles at the finite bound and at V = 0, then opens the box only along the axis (GeomBndLib_InfiniteHelpers), as for a cylinder; a cone widens away from its apex | For a cone whose axis is a coordinate axis the radial sides stay closed, so the box misses the surface away from V = 0 (an axis-aligned cone on V in [0, +inf) gets X and Y within the V = 0 circle). Pinned by `GeomBndLib_SurfaceIssueTest.Cone_InfiniteV_RadialDirectionsStayClosed` |
| `GeomBndLib_OffsetSurface::Box`, fallback without an analytic equivalent (GeomBndLib_OffsetSurface.cxx) | Enlarges the basis box by \|offset\| and then by the tolerance; `Bnd_Box::Enlarge` keeps the larger gap | The gap is max(\|offset\|, Tol) instead of \|offset\| + Tol: the tolerance is lost whenever it is smaller than the offset. Pinned by `GeomBndLib_SurfaceIssueTest.OffsetSurface_Fallback_ToleranceNotAddedToOffset` |
| `GeomBndLib_OtherSurface::Box`, `GeomBndLib_BezierSurface::Box` on a strict sub-patch, `GeomBndLib_BSplineSurface::Box` outside the surface's bounds | Add the points of a sample grid (at most 50 x 50, from the full-range sample counts for Bezier and B-spline) and enlarge by the tolerance only, where the curve evaluators enlarge their samples by 1.5 times a deflection estimate | The box can miss extrema between samples: a quadratic Bezier patch z = 20u(1-u) on U in [0.1, 0.65] gets Zmax 4.968 for a true 5.0. Pinned by `GeomBndLib_SurfaceIssueTest.BezierSurface_TrimmedPatch_SampledWithoutDeflection` |
| `GeomBndLib_Parabola2d::Box`, finite arcs (GeomBndLib_Parabola2d.cxx) | As the BndLib parabola: adds only the two ends and, when U1·U2 < 0, the vertex; the coordinate extrema of a parabola turned in its plane (at U = -2F·Yk/Xk) are missed | The box does not contain the arc: a parabola with mirror axis (1,1)/√2, focal 2, on [−6, 6] gets Xmin −1.0607 where the curve reaches −1.4142. Reaches `GeomBndLib_Curve2d` and `BndLib_Add2dCurve` (for conic arcs, and Geom2dAdaptor_Curve of parabolas). Pinned by `GeomBndLib_Curve2dIssueTest.Parabola2d_TurnedArc_MissesCoordinateExtremum` |
| `GeomBndLib_Parabola::Box(gp_Parab, U1, U2, ...)`, finite arcs (GeomBndLib_Parabola.cxx) | Adds only the two ends and, when U1·U2 < 0, the vertex (as BndLib::Add(gp_Parab)); the coordinate extrema of a parabola turned in its plane, at U = −2F·Yk/Xk, are missed | The box does not contain the arc: axes (1,1,0)/√2, (−1,1,0)/√2, focal 2, on [−6, 6] gives Xmin −1.0607 where the curve reaches −1.4142 at U = 4. Reaches GeomBndLib_Curve, BndLib_Add3dCurve, and so BRepBndLib on parabolic edges. Pinned by `GeomBndLib_CurveIssueTests.TurnedParabola_BoxMissesCoordinateExtremum` |
| `Extrema_ExtCS::Perform`, a line against a plane (not parallel), torus, cone, Bezier, B-spline, swept, offset or other surface, when the line's range, clipped to the projections of the surface's box corners, is shorter than Confusion or empty (Extrema_ExtCS.cxx) | Projects the middle point aPm with Extrema_ExtPS, then passes `PC.Value()` of a default-constructed Extrema_POnCurv (the origin) to AddSolution as the point on the curve, instead of aPm | Each such extremum reports the origin as its point on the curve (its parameter and square distance are right). Reached by short lines and by lines whose range lies beyond the surface's box |
| `GeomBndLib_BSplineCurve::Box` (GeomBndLib_BSplineCurve.cxx) with `Geom_BSplineCurve::Segment` (Geom_BSplineCurve.cxx), periodic curves | When [U1, U2] differs from the curve's range but U2 ends a whole period from U1 to the last bit (typically U2 = Last - 1 ulp, as BRepTools::UVBounds returns for a face on an extrusion or revolution of a periodic B-spline), `aMinDiff` is 0 and the segment tolerance becomes 0; `Segment(U1, U2, 0)` then leaves one pole of the result unwritten (seen: knots ×3, ×1, ×1, ×3, five poles, the fourth never set) | Native OCCT reads uninitialised memory for that pole (values like 6.95e-310, or a leftover coordinate): the box of the edge or face changes from run to run (seen in BRepBndLib::Add and AddOBB on a periodic degree-2 B-spline with 3 poles). The port reads 0, so its box is deterministic but also takes in the origin, bounded by the full curve's pole hull. The oracle test skips these cases (`BndLibOracleTests.SegmentsWithZeroTolerance`) |

## Phase 3: unit 3D, 2D intersection bases

| Where | What | Effect |
|---|---|---|
| `Intf_SectionPoint::IsOnSameEdge`, both points on vertices of the first argument (Intf_SectionPoint.cxx) | Compares `IndexT1` (the second argument's address) where the other branches compare `IndexO1` | Two points on the same vertex of the first argument are not "on the same edge" unless their second-argument addresses agree (the tool-side test then decides). Pinned by `IntfTests.SectionPoint_IsOnSameEdge` |
| `Intf_InterferencePolygon2d::Clean` (Intf_InterferencePolygon2d.cxx) | `Only1Seg` is set before the loop over the tangent zones and only ever reset by an EDGE/EDGE pair, so a zone inherits `true` from an earlier short zone | A later zone spanning several segments can be turned into a single section point when an earlier zone was short. Not reproduced |
| `Intf_Tool::HyprBox` (3D, Intf_Tool.cxx) | Writes `beginOnCurve[nbSeg]`/`endOnCurve[nbSeg]` without the `nbSeg < 6` guard that the fix of bugs #75/#76 added to Hypr2dBox, Parab2dBox and ParabBox | More than six crossings would overflow the fixed arrays in C++ (the port throws IndexOutOfRangeException). Not reproduced |
| `IntImpParGen_Intersector::Perform` (IntImpParGen_Intersector.gxx) | `Resultat1`/`Resultat2` are sized `2 + 2·(1 + n)` for the n solution intervals before the intervals shifted by a period on a closed implicit curve (`nb_segments_crees`, up to 2 per interval) are added, and And_Domaine_Objet1_Intersections may write two values per interval | With three or more shifted intervals C++ writes out of bounds (the port throws Standard_OutOfRange). Not reproduced; reached through IntCurve_IntImpConicParConic and Geom2dInt (unit 3E) |
## Phase 2: unit 5A, GeomLib and Approx

| Where | What | Effect |
|---|---|---|
| `GeomLib::CancelDenominatorDerivative` through `CancelDenominatorDerivative1D` (GeomLib.cxx) | Reparametrises the surface's U knots to [0, 1] before testing whether it can be treated, and restores them only after a treatment | A surface that can't be treated (non-rational, periodic, or already treated) comes back reparametrised to [0, 1]. OCCT changes the caller's surface; the port changes a copy (`CancelDenominatorDerivative_Untreatable_LeftReparametrised_OCCT`) |
| `GeomLib_Interpolate::GeomLib_Interpolate` (GeomLib_Interpolate.cxx) | Never sets `myError` on success | `Error()` returns an uninitialised value after a successful interpolation; the port returns `GeomLib_NoError` |
| `Adaptor3d_CurveOnSurface::Load(C)` and `Load(S)` (Adaptor3d_CurveOnSurface.cxx), as Approx_CurvlinFunc uses it | Keep `myIntervals`/`myIntCont`, so a reloaded adaptor returns the intervals cached for the previous curve | Approx_CurvlinFunc's case 3 (two curves on two surfaces) reuses one adaptor in NbIntervals, Intervals and Init: the second curve's discontinuities are lost and its parameter tables are built on the first curve's C3 intervals (`CurvlinFunc_TwoCurves_ReusesTheFirstCurvesIntervals_OCCT`: knots at 0.3 and 0.6 fuse to 2 intervals, not 3) |
| `Approx_CurvlinFunc::Trim`, case 3 (Approx_CurvlinFunc.cxx) | Computes the second curve's parameters with `GetUParameter(CurOnSur, First, 1)`, the first curve's tables (myUi_1, mySi_1, myLength1) | The second curve is trimmed at the first curve's parameters. Latent: Approx_CurvilinearParameter's case 3 evaluator never calls Trim (commented out in OCCT). By reading |
| `Approx_SweepApproximation::Dump` | Writes the line end after the 2D errors, and " Not Done ", to std::cout instead of its stream | Cosmetic; the port writes them to the TextWriter |
| `GeomConvert_SurfToAnaSurf::ConvertToAnalytical` (GeomConvert_SurfToAnaSurf.cxx) | The "diagonal" that caps the tolerance evaluates `mySurf->Value((U1 + U2), (V1 + V2) / 2)`, at U1 + U2 rather than the middle | The cap (diagonal / 1000) is measured to a point that can be outside the range (B-splines are extrapolated), so the effective tolerance depends on where the parameter range sits. By reading |
| `GeomConvert_SurfToAnaSurf::TryCylinerCone` | When the first or last iso is not a circle (a cone's apex, R = 0), its point is `iso->Value((Last - First) / 2)`, half the range rather than its middle | The cone's axis goes through the wrong point unless the iso's range starts at 0. By reading |
| `GeomConvert_SurfToAnaSurf::TryCylinderByGaussField` | Sizes the least-squares point array `theNbU * theNbU` rather than `theNbU * theNbV` | Latent: its only caller passes 7 × 7 |
## Phase 3: unit 3A, topology core

| Where | What | Effect |
|---|---|---|
| `BRep_Builder::UpdateEdge(E, P1, P2, S, L)` (BRep_Builder.cxx), polygons on a closed surface | Stores the `BRep_PolygonOnClosedSurface` with the identity location, where the one-polygon form stores `L.Predivided(E.Location())` and `BRep_Tool::PolygonOnSurface(E, S, L)` looks for that | On a face at a location (or an edge with one), the two polygons are stored but never found again by face or surface; only the index form `PolygonOnSurface(E, P, S, L, i)` returns them. Pinned by `BRep_ToolTests.PolygonOnClosedSurface_is_stored_without_the_location_as_in_OCCT`; not checked natively (Poly_Polygon2D isn't bound) |
| `BRepTools::UpdateFaceUVPoints` (BRepTools.cxx) | `if (TE->Locked()) return;` stops at the first locked edge; `continue` was presumably meant (the edges' own UV points are independent) | The edges after a locked one keep stale UV points. In the port every frozen edge is locked, so this happens when a new face is built on edges shared with a published shape. Pinned by `BRep_ToolTests.UpdateFaceUVPoints_stops_at_the_first_locked_edge_as_in_OCCT` |
| `BRep_Tool::Range(E, S, L, First, Last)` (BRep_Tool.cxx) | A query that ends with `E.TShape()->Modified(true)` | Asking for a range marks the edge modified and so unchecked; `BRep_Tool::Parameter(V, E, S, L)` and `Range(E, F, ...)` reach it. The port does the same on unfrozen edges and nothing on frozen ones (decision 312) |

## Phase 3: unit 3E, 2D curve intersection

| Where | What | Effect |
|---|---|---|
| `SET_BOUNDED_DOMAIN` (IntCurve_IntConicConic.cxx) | Compares `bsup` with `domain.FirstParameter()` instead of `LastParameter()` | A last bound is replaced by the domain's even when it is already inside it. Reached only by parabola and hyperbola intersections with a half-bounded domain. Not reproduced |
| `IntCurve_IntConicConic::Perform(gp_Lin2d, gp_Parab2d)` (IntCurve_IntConicConic.cxx) | Offsets the line by `(maxtol·Dy, maxtol·Dx)`, which is not normal to it (`(-Dy, Dx)` was meant) | The band in which the analytic solutions are searched is skewed; the results are refined afterwards. Not reproduced |
| `IntCurve_IntConicConic::Perform(gp_Lin2d, gp_Elips2d)`, segment ends (IntCurve_IntConicConic_1.cxx) | Evaluates the end of a solution segment at `SolutionEllipse[i].Binf` instead of `.Bsup`, and on `Pos1b == End` sets `Esup = DL.LastParameter()` (the line's domain) instead of `DE.LastParameter()` | A line/ellipse segment (tangency along a stretch within the tolerance) can get the wrong end point and transition and, at the ellipse domain's end, the line's parameter as its ellipse parameter. Not reproduced |
| `IntCurve_IntPolyPolyGen::findIntersect`, recursion on tangent zones (IntCurve_IntPolyPolyGen.gxx) | Calls `Perform(C1, RecursD1, C2, RecursD2, Tol, TolConf, ...)` with the two tolerances swapped | The refinement treats Tol as the confusion tolerance and TolConf as the tolerance. No effect when they are equal, as Geom2dAPI_InterCurveCurve passes them |
| `HeadOrEndPoint` (IntCurve_IntPolyPolyGen.gxx) | Computes `SP2` when the second domain's end matches but never uses it: P2 is not moved to that end, as P1 is moved to `SP1` | A solution snapped to an end of the second curve keeps the unsnapped point. Not reproduced |
| `IntCurve_PConicTool::D2` on a line (IntCurve_PConicTool.cxx) | Clears `Tan` (which LineD1 then writes) and never writes `Norm` | The caller's normal is left as it was; OCCT's callers pass default-constructed (0, 0) vectors, which the port writes |

## Phase 2: unit 6A, projection

| Where | What | Effect |
|---|---|---|
| `ProjLib_CompProjectedCurve::Perform` | Sets `approx2d = false` for an isoparametric part inside the loop over parts and never restores it | Every later part that isn't isoparametric gets no 2D approximation: its 2D result is the previous part's iso line (`Perform_IsoThenFreePart_ReusesIsoLine_OcctBug`) |
| `ProjLib_CompProjectedCurve` (file-static FindSplitPoint) | Reads `Point(aMinIdx)` with `aMinIdx = -1` when Extrema_ExtPS is done without an extremum | Out-of-range read in OCCT; the port throws Standard_OutOfRange. Not reached by a test |
| `ProjLib_ComputeApproxOnPolarSurface::ProjectUsingInitialCurve2d` | Tests `BSS->MaxDegree() == 1` (and `BS->MaxDegree()` for Bezier surfaces), the static maximum degree (25), to detect bilinear surfaces | The pole-wise bilinear branches are dead code; inside them, square distances truncated to int, extrema started from coordinates instead of parameters, and `myProjIsDone` reset after every pole (`BilinearSurface_PoleWiseBranchIsDead`) |
| `ProjLib_ComputeApproxOnPolarSurface::Perform` (C0 B-spline path) and `BuildInitialCurve2d` | Can read `Inter2d(nbInter2d + 2)`; dereferences a null `G2dC` for an initial curve of type OtherCurve; reads `ext.SquareDistance(1)` without checking `NbExt() > 0` | Out-of-range reads and a null dereference in OCCT; the port throws. Not reached by a test |
| `ProjLib_ComputeApprox` (file-static Function_SetUVBounds, cylinder branch; the same logic for spheres, cones and tori) | When the sampled U range is outside [0, 2π], replaces the window by mid ± π, a width of 2π, even for a curve spanning more | A curve winding more than once around the axis gets a wrong pcurve (off by up to the diameter) while the reached tolerance stays tiny (`Cylinder_HelixOfMoreThanATurn_WrongResult_OcctBug`; ProjLib_ProjectedCurve reaches it for B-spline helices) |
| `ProjLib_ComputeApprox` (Function_Value, sphere branch) | Tests `S > U1 \|\| S < U2`, always true (`S < U1 \|\| S > U2` was meant) | Harmless: ElCLib::InPeriod leaves values already in the window unchanged. By reading |
| `ProjLib_ComputeApprox` (Function_SetUVBounds, sphere/circle branch) | Adjusts U2 to the seam, then overwrites it with the U of the point at `W1 + PI/8`, a fixed parameter offset | The bounds depend on the circle's parameterisation. By reading |
| `ProjLib_ComputeApprox` (Function_SetUVBounds, cylinder/closed ellipse) | Calls `mySurface->D1(U1, U2, ...)` with two U parameters | Harmless on a cylinder (D1U doesn't depend on V). By reading |
| `ProjLib_ProjectOnPlane::Load`, parabola | Gives the projected parabola the focal `Focal / \|f(Xc)\|`; the right one is `Focal * \|f(Yc)\|² / \|f(Xc)\|` | Wrong focal when the projection shortens the parabola's Y direction (`Parabola_TiltedAboutItsXAxis_WrongFocal_OcctBug`) |
| `ProjLib_ProjectOnPlane::Load`, parabola projected to a line | Builds a Geom_Line, then calls GetTrimmedResult with the null parabola | Null dereference in OCCT, NullReferenceException in the port (`Parabola_ProjectedToLine_OcctDereferencesNull`) |
| `ProjLib_ProjectOnPlane` (callers of PerformApprox) | Dereferences a null curve when the approximation fails (`!IsAllApproximated`) | Null dereference. Not reached by a test |
| `ProjLib_ProjectOnSurface::Load` | `MC.Curve(1, Poles)` fills the result array instead of `LocalPoles`, which stays at the origin; and passes the increment `Inc` where `BSplCLib::IncreaseDegree` takes the new degree | The result curve's poles are lost: it ends at the origin (`Load_PolesLost_OcctBug`). No OCCT code calls this class |

## Phase 3: unit 3F, curve-surface intersection

| Where | What | Effect |
|---|---|---|
| `GeomAPI_ExtremaCurveCurve::TotalPerform` (GeomAPI_ExtremaCurveCurve.cxx) | Compares each pair of end points with the first total distance (`aTotalDist2`), which it never updates | A later pair can replace a nearer one: segments [1, 2] on the X and Y axes give a `TotalLowerDistance` of √8 instead of √2. Pinned by `ExtremaCurveCurve_TotalReachesTheEnds` |
| `GeomAPI_ExtremaCurveCurve::Init` with ranges (GeomAPI_ExtremaCurveCurve.cxx) | Loads the whole curves (`myC1.Load(C1)`, no range) for the total distance | The Total methods report the curves' natural end parameters with the ranges' end points and project onto the whole curves: ranges [1, 2] of the segments [0, 3] give a total distance 1 at parameters (0, 0), where the answer is √2. Pinned by `ExtremaCurveCurve_TotalOnRangesReportsTheCurveEnds` |
| `GeomAPI_Interpolate` (`BuildParameters`, `Interpolate`), `GeomAPI_PointsToBSpline::Init(Points, Params, ...)` (GeomAPI_Interpolate.cxx, GeomAPI_PointsToBSpline.cxx) | Index the caller's arrays from 1 (`ParametersPtr->Value(ii)` at a point index, `Value(1)`, `SetValue(1)`, `Params(i)`) | Arrays with another lower bound are read at the wrong places, or out of range. Not reproduced |
| `GeomAPI_ExtremaCurveSurface::Init` (GeomAPI_ExtremaCurveSurface.cxx) | Hands Extrema_ExtCS a pointer to a local GeomAdaptor_Surface | The pointer dangles after Init (Extrema_ExtCS reads it again in later calls). Harmless in the port, which keeps a reference |
## Phase 3: unit 3B, modification and adaptors

| Where | What | Effect |
|---|---|---|
| `BRepTools_NurbsConvertModification::NewCurve2d`, a seam edge without a 3D curve (BRepTools_NurbsConvertModification.cxx) | Calls `BRepTools::EvalAndUpdateTol(newE, C3d, ...)` with the null `C3d`, which dereferences it (`C3d->IsPeriodic()`) | A crash in OCCT when a pcurve-only seam on a non-B-spline surface or pcurve is converted; the port throws NullReferenceException. By reading |
| `BRepTools_GTrsfModification::NewTriangulation` (BRepTools_GTrsfModification.cxx) | Tests `myGScale < 0.0` to reverse the triangles of a mirroring transformation, but `myGScale` is a maximum of absolute values | The triangles of a mirrored triangulation keep their orientation (TrsfModification tests `ScaleFactor() < 0`). By reading |
| `BRepTools_GTrsfModification::NewPolygonOnTriangulation` | Returns true when the face has a triangulation even if the edge has no polygon on it | The modifier then calls `UpdateEdge` with a null polygon on the new edge. By reading |
| `BRepTools_Modifier::Rebuild` with a modification whose `NewCurve2d` returns true without setting the curve (BRepTools_Modifier.cxx; `BRepTools_TrsfModification::NewCurve2d` through `GeomLib::SameRange` on a zero-length range) | The pcurve handle is declared once for all the edges of the face | The edge gets the previous edge's pcurve. The port's `out` parameter is null instead (decision 346). Not reproduced |
| `BRepTools_History::AddModified`/`prepareModified` (BRepTools_History.cxx) | Removing a shape from another's generated list leaves the list bound when it becomes empty | `HasGenerated()` stays true, and Dump counts the shape, with nothing generated. Kept; pinned by `BRepTools_ModifierTests.History_records_modifications_generations_and_removals` |
## Phase 3: unit 3H, global properties

| Where | What | Effect |
|---|---|---|
| `BRepGProp_Gauss::convert(Inertia, Coeff, IsByPoint, ...)` (BRepGProp_Gauss.cxx) | The test `std::abs(Mass) >= EPS_DIM && theIsByPoint` sends every region bounded by a plane (`theIsByPoint` false) to the else branch, which sets the mass to 0 and the centre to the origin; the inner `if (theIsByPoint)` and its plane branch are dead | `BRepGProp_Vinert` by a plane (its Perform and constructors with a gp_Pln) always reports a null volume and centre, with the matrix of inertia computed. BRepGProp's statics never ask for it (VolumePropertiesGK by plane uses BRepGProp_VinertGK, which is right). Pinned by `BRepGPropKnownAnswerTests.Vinert_by_plane_returns_a_null_mass_as_OCCT_does`; Vinert isn't bound natively |
| `BRepGProp_Face::VKnots` (BRepGProp_Face.cxx), planes, cylinders and cones | Writes the face's U bounds (`FirstUParameter`, `LastUParameter`) as its V knots | On a natural face of these surfaces the adaptive integration (`Sinert`/`Vinert` with Eps) starts its V subdivision from U values: knots outside the V range are ignored, inside ones split it at an arbitrary place. Changes the subdivision, so the error and the last digits, not the converged result. Ported as OCCT |
| `BRepGProp_VinertGK::GetAbsolutError` (BRepGProp_VinertGK.hxx) | Declared, never defined | A link error for a caller in C++; the port returns the absolute error that Perform returns |
| `BRepGProp_Face::GetTKnots` (BRepGProp_Face.cxx) | Copies the B-spline face's V knots and doesn't use them; the documentation promises arc parameters whose points have V close to the face's V knots | With IsUseSpan the outer Gauss-Kronrod integral is split only at the pcurve's own knots, not at the surface's V knots. Accuracy only. By reading |
| `BRepCheck_Shell::Closed` (BRepCheck_Shell.cxx) | Reports BRepCheck_RedundantFace when `!mapS.Add(aF)`; `NCollection_IndexedMap::Add` returns the index, never 0 | A face repeated in a shell is never reported as redundant by Closed (Minimum reports NotConnected, or Closed InvalidMultiConnexity); only Orientation reports RedundantFace. Pinned by `BRepCheckTests.Shell_RedundantFace` |
| `BRepCheck_Shell::NbConnectedSet` (BRepCheck_Shell.cxx) | When the face that empties the set of free faces starts a new connected set, `break` leaves before the shell being built is appended | A last connected set of one face is lost from `theSets` and the count. By reading; not pinned |
## Phase 3: unit 3C, BRep I/O

The input bugs are found by reading and pinned by the port's untrusted-input tests; they aren't run natively, where
they hang, overflow the stack or read outside memory and would take the test host down.

| Where | What | Effect |
|---|---|---|
| `TopTools_LocationSet::Read` (TopTools_LocationSet.cxx) | `while (l1 != 0) { IS >> p; ... IS >> l1; }` never checks the stream; `myMap(l1)` isn't range-checked in a release build; `L1.Powered(p)` of a complex location recurses `p` times | A location chain cut short loops for ever (the failed extraction leaves `l1` as it was), growing the location; a bad index reads outside the map; a large power of a complex location overflows the stack. The port throws `Storage_StreamFormatError` (decision 383) |
| `TopTools_ShapeSet::Read` (TopTools_ShapeSet.cxx) | The sub-shape loop `do { Read(SS, IS, nbShapes); ... } while (!SS.IsNull())` never checks the stream, and `Read(S, IS, nbshapes)` indexes `myShapes(nbshapes - atoi(buffer + 1) + 1)` without a range check | A list cut short loops for ever (the uninitialised or stale buffer keeps naming a shape) or reads outside the map; so does a reference to a shape not read yet |
| `TopTools_ShapeSet::Read`, `BRepTools_ShapeSet` readers, GeomTools' set readers | `char buffer[255]; IS >> buffer;` with no width | A word of 255 characters or more writes past the buffer |
| `BRepTools_ShapeSet::ReadGeometry`, vertex and edge (BRepTools_ShapeSet.cxx) | The representation loops `do { ... IS >> val; ... } while (val > 0)` never check the stream | On input cut short the last `val` stays positive and the loop never ends; for a vertex it appends the same point representation each time, without bound |
| `BRepTools_ShapeSet::ReadGeometry`, a face of a triangulation only | `myTriangulations.FindKey(s)` without a range check (the other branch checks `s`) | A bad index reads outside the map |
| `BRepTools_ShapeSet::ReadPolygon3D`, `ReadPolygonOnTriangulation`, `ReadTriangulation` | The node and triangle counts, declared outside the loops, are read without checks and allocated for; on a failed stream they keep the last value | A huge or negative count allocates without bound or fails; after a failure every remaining polygon or triangulation allocates again with the last count |
| `BRepTools_ShapeSet::AddGeometry` (also BinTools_ShapeSet) | Adds an edge's polygon on triangulation and its triangulation to the maps without checking for null; `WritePolygonOnTriangulation` and `WriteTriangulation` dereference them | Writing such an edge crashes. The port throws `Standard_NullObject` when the edge is added (decision 384; in BinTools in `AddShape`) |
| `TopTools_ShapeSet::Add` (and `BinTools_ShapeSet::AddShape`, `BinTools_ShapeWriter::WriteShape`) | Recurse once per level of nesting | A shape nested deeply enough overflows the stack and ends the process. The port throws `InsufficientExecutionStackException` |
| GeomTools' `ReadCurve`, `ReadCurve2d`, `ReadSurface` (also BinTools' sets) | Trimmed, offset, extrusion and revolution objects read their basis recursively, without a limit | Deeply nested geometry in a file overflows the stack (the port caps it at 64 levels) |
| `BinTools_ShapeSet::WriteShape` / `ReadShape`, `BinTools_ShapeWriter` / `BinTools_ShapeReader`, a point on a surface | Write `Parameter2()` before `Parameter()`; both readers take the first value as the first parameter | The parameters of a point on a surface are swapped on every binary round trip (BinTools tests `Point_representations_follow_OCCT`) |
| `BinTools_ShapeReader::ReadShape`, a point on a curve on a surface (BinTools_ShapeReader.cxx) | Tests `aSurface.IsNull()` where `!aSurface.IsNull()` was meant | The representation is dropped when the surface is there, or built with a null surface when it isn't |
| `BinTools_SurfaceSet::Write`, `BinTools_Curve2dSet::Write` | Dereference the null entry the sets can hold, outside their `try` | Writing a set holding a null surface or 2D curve (from a point representation read with a missing one) crashes; the port throws `Standard_Failure` (found by fuzzing) |
| `BinTools_ShapeSet::ReadShape`, vertices of versions 1, 2 and 4 | Reads a real before the type byte and steps back unless the next byte is 1 to 3, for files written with the parameter first; version 4 files write the type first | A parameter whose last stored byte is 1, 2 or 3 (a positive real below about 1e-303) is misread. Ported as is |
| BinTools readers: `ShapeReader` references and locations, `LocationSet::Read` | Stream positions and location indices aren't checked; `Powered` of a complex location recurses per power | A reference cycle recurses until the stack overflows; bad indices read outside the maps |
## Phase 3: unit 3G, classification and boxes

| Where | What | Effect |
|---|---|---|
| `IntCurvesFace_ShapeIntersector::Load` (IntCurvesFace_ShapeIntersector.cxx) | Appends the new faces' intersectors without clearing those of a shape loaded before, and sets `myNbFaces` to the new count | A second Load intersects the first shape's faces (the first `myNbFaces` of the sequence), not the new shape's. Pinned by `IntCurvesFaceTests.ShapeIntersector_second_Load_keeps_the_first_shape_OCCT_issue` |
| `IntCurvesFace_ShapeIntersector::PerformNearest` | A face is skipped (not intersected again) once the nearest parameter found reaches `theParMin`, but `SortResult` still reads its intersector | The skipped faces report the points of their previous Perform, for another line. By reading; reached only when a point is found exactly at `theParMin` |
| `IntCurvesFace_Intersector::InternalCall` (IntCurvesFace_Intersector.cxx) | Projects the point on `BRep_Tool::Curve` of every edge of the face, without checking for a null curve | On a face with a degenerated edge (or a pcurve-only edge) whose point the larger tolerance classifies ON, the projection gets a null curve and throws (Standard_NullObject in OCCT; the port throws too). By reading |
| `BRepBndLib.cxx`, `FindExactUVBounds` | Calls `ElCLib::InPeriod(u, umin, umax)` and `InPeriod(v, vmin, vmax)` as if they changed their argument; OCCT 8's InPeriod returns the value | On periodic surfaces the mid-points of the pcurves aren't brought into the surface's period, so a face bounded by its iso-lines outside the first period isn't recognised as a natural restriction: AddOptimal then bounds the face from its pcurves' UV box and the edges' boxes (AdjustFaceBox) instead of the surface's natural bounds. By reading |
| `BRepTopAdaptor_FClass2d::BRepTopAdaptor_FClass2d` (BRepTopAdaptor_FClass2d.cxx) | Returns from the constructor when an edge has no pcurve on the face | The classifier keeps the polygons of the wires before it, and the UV box as far as it got: points are classified against part of the face. By reading |
| `BRepTopAdaptor_FClass2d::BRepTopAdaptor_FClass2d`, a wire of fewer than 4 sample points | Reads `SeqPnt2d(1)` and `SeqPnt2d(2)` whatever the number of points (out of range with `No_Standard_OutOfRange`) | Undefined in OCCT for a wire that gives fewer than two points (all its edges shorter than 1e-9 in parameter but one, or 3D-degenerate); the port throws Standard_OutOfRange. By reading |
| `TopClass_FaceClassifier.pxx` with `BRepClass_FaceExplorer::CurrentEdge` | One `BRepClass_Edge` is reused for all the edges, and `BRepClass_Edge::SetNextEdge` returns without writing when the edge's last vertex has no single other edge (a closed edge, three edges at a vertex) | That edge keeps the previous edge's next edge, which `BRepClass_Intersector`'s CheckSkip (vertex tolerance above the gap tolerance) then uses as its neighbour. Kept (the port's `ITopClass_FaceExplorer.CurrentEdge` takes the edge by `ref`, decision 390). By reading |

## Phase 3: unit 3L1, Draw-lite shapes

Found while porting Draw's commands (Draw-lite reproduces them, so the corpus sees OCCT's behaviour).

| Where | What | Effect |
|---|---|---|
| `tcopy` (BRepTest_BasicCommands.cxx) | The pairs loop reads `a[i + iFirst]` and `a[i + iFirst + 1]` with `i` the pair number, where `2 * i` was meant | `tcopy a b c d` copies a to b, then b to c; d is never set. Pinned by `DrawShapeCommandsTests.Tcopy_copies_the_shapes` |
| `ShapeAnalysis_ShapeTolerance::InTolerance` (ShapeAnalysis_ShapeTolerance.cxx) | Vertices are kept when `tol >= valmax` where faces and edges test `tol <= valmax` | `tolerance shape [mode] min max` lists every vertex over the upper bound among the shapes "below" or "between", and none under it (each vertex once per edge that holds it). Pinned by `DrawShapeCommandsTests.Tolerance_and_maxtolerance` |
| `nexplode` (DBRep.cxx) | Sizes its arrays as the number of distinct sub-shapes minus one, assuming the shape itself is in the map and not of the type | `nexplode` of an edge into edges (or a face into faces, a vertex into vertices) writes past the array: Standard_OutOfRange in a build with range checks |
## Phase 3: unit 3I, BRepLib

| Where | What | Effect |
|---|---|---|
| `BRepLib_MakeEdge2d::Init` (the main one) | Never sets `myError = BRepLib_EdgeDone`, and the field is uninitialised in every constructor but the default one | A maker that never failed reports an undefined error. The port's field starts at EdgeDone. By reading |
| `BRepLib_MakeEdge2d.cxx`, static `Project` | Returns true with `p` unset when Extrema_ExtPC2d is done but finds no extremum | The parameter is undefined in OCCT; the port gives 0. By reading |
| `BRepLib_MakeWire::Add(const TopoDS_Edge&)` | Ends with `myError = BRepLib_WireDone` unconditionally | `BRepLib_NonManifoldWire` is never reported; only VF and VL, set to null, keep the information. Kept |
| `BRepLib_BndBoxVertexSelector::SetCurrentVertex` (BRepLib_MakeWire_1.cxx) | Never resets `myVBox` | The box grows with every vertex: more boxes pass Reject, and only the running time suffers. Kept |
| `BRepLib_MakeFace::Init(S, UMin, UMax, VMin, VMax, TolDegen)` | Checks the surface's natural range only when the parameters come reversed | Ordered parameters outside the natural range are accepted. Kept |
| `BRepLib_MakeShell` | Adapts a null 3D curve (`IsDegenerated` on a GeomAdaptor_Curve) when BuildCurves3d failed on an edge | Throws in OCCT and in the port. By reading |
| `BRepLib::UpdateEdgeTol` on an edge without a 3D curve | The first loop tests `an_iterator.More()` but reads `second_iterator.Value()` and advances neither | Only the first representation is looked at; the loop never ends if it isn't a curve on a surface, and `curve_on_surface_index` is left at 1 instead of 0. If the edge is also flagged SameParameter, `EvalMaxParametricDistance` measures against `geom_reference_curve`, never loaded: OCCT dereferences a null curve (the native binding throws), the port throws `NullReferenceException`. Reproduced in the oracle, which accepts both |
| `BRepLib::OrientClosedSolid` on a solid without faces (not a bug, pinned) | `BRepClass3d_SClassifier::PerformInfinitePoint` classifies IN for a faceless solid | Returns true and reverses the solid. Pinned by `BRepLibStaticsTests.OrientClosedSolid_reverses_an_empty_solid` |
| `BRepLib::ExtendFace`, a face on a bounded (B-spline, Bezier) surface | Calls `GeomLib::ExtendSurfByLength` on the face's own surface handle (`BRep_Tool::Surface(theF)`), which raises the degree and inserts knots in place | The face passed in changes, and so does every face sharing the surface. Reproduced: the native input's BRep text changed in 8 of 20 lofts. The port's GeomLib extends a copy, so the input is unchanged and the extended face is OCCT's |
| `BRepLib::ExtendFace`, the general branch | Takes `BRep_Tool::Surface(theF)` without the face's location | A located face on a non-analytic surface loses its placement when extended (the analytic branch uses the transformed surface). Kept |
| `BRepLib::ExtendFace` | Tests `!Precision::IsInfinite(aSVMax)` for the VMin extension, where `aSVMin` is meant | VMin is extended when VMax is finite. Kept |
| `BRepLib_PointCloudShape::NbPointsByDensity` | Divides each face's area by the argument `theDensity` instead of the computed `aDensity` | With the default density 0 the quotient is infinite, its conversion to int gives INT_MIN on x86, and every face gets one point. Kept, with x86's conversion emulated; pinned by `BRepLibMeshTests.PointCloudShape_with_the_computed_density_gives_one_point_per_face` |
| `BRepLib::EnsureNormalConsistency` | Uses `BRep_Tool::PolygonOnTriangulation`'s result without a null check | An edge between two meshed faces without a polygon on one of their triangulations crashes OCCT; the port throws `NullReferenceException`. BRepMesh output always has the polygons. By reading |
| `BRepLib_FuseEdges.cxx`, static `MakeClosedCurve` | When a fused closed B-spline edge must start at a vertex other than the curve's origin, makes the curve periodic, moves its origin and segments it in place | Without a transformation, that curve is the first old edge's own 3D curve (or its trimmed curve's basis): the input shape's edge geometry changes. The port works on a copy |
| `BRepLib_FuseEdges::SameSupport`, B-splines | The knot check `(K1(k) - K2(k)) > tollin` is signed, and the multiplicities are compared as reals against a length tolerance | A knot of the first curve smaller than the second's by any amount passes. Kept |
| `BRepLib_FuseEdges::BRepLib_FuseEdges` | The `PerformNow` argument is ignored (commented out) | Nothing is computed until an accessor asks. Kept |
| `BRepLib_MakeFace(W)` with `BRepLib::UpdateTolerances` (not a bug, a consequence) | UpdateTolerances adds `2 * Epsilon(tol)` to every vertex tolerance it computes, and MakeFace runs it on the face holding the caller's wire | Making a face from a wire raises the tolerances of the caller's own vertices by two ulps, in place. The port keeps the caller's frozen vertices instead (decision 417) |
## Phase 3: unit 3J, BRepCheck

| Where | What | Effect |
|---|---|---|
| `BRepCheck_Analyzer` (BRepCheck_Analyzer.hxx), its map of results | `NCollection_IndexedDataMap<TopoDS_Shape, handle<BRepCheck_Result>>` without a hasher: the default compares with `operator==` (IsEqual), so a shape met in two orientations (a seam edge, a shared vertex) has two results; the face task skips shapes already seen with IsSame, so only the first orientation's result gets the face as a context; `ValidSub` stops (`break`) at the result without that context | Whether a face (or edge) is valid depends on the order of its sub-shapes: a fault on an edge met after both occurrences of a seam isn't seen by `IsValid(face)`. Ported as is (decision 430); found by the oracle, checked natively over all 5,040 orderings of a face's wire |
| `BRepCheck_Solid::Minimum` (BRepCheck_Solid.cxx) | After "too many growths" breaks out of the loop that calls `Init` on each closed shell's tool, the remaining tools aren't initialised, and the `IsOut` loop then uses their default point (-1, -1, -1) and dereferences their null classifier | When the second shell that isn't a hole comes before the last two closed shells, later shells are tested with a point that isn't on them, and the loop reaches an uninitialised tool and crashes (outside the analyzer's `try`). By reading |
| `BRepCheck_Edge::InContext` | When the reference curve is a curve on a surface (no 3D curve) and the edge has no other pcurve on a planar face, down-casts the reference adaptor to GeomAdaptor_Curve and dereferences the null result | An access violation, caught by the analyzer under OCC_CATCH_SIGNALS as CheckFail. The port throws Standard_NullObject, also CheckFail (pinned by `BRepCheckAnalyzerTests.Edge_minimum_finds_bad_flags_ranges_and_curves`) |
| `BRepCheck_Edge::InContext` in parallel mode | The edge's reference adaptor (`myHCurve`) is shared by the threads that check the edge in its different faces; a GeomAdaptor_Curve on a B-spline or Bezier curve fills an evaluation cache on each call | A data race in OCCT's parallel mode, seen natively: on `brep/bug29502_split_faces.brep` of the test dataset, 1 native parallel run in 40 under load reports a valid face (and its compounds) invalid; native serial and the port, serial and parallel (40 of 40), don't. The port evaluates a shallow copy per call in parallel mode (decision 431) |
| `BRepCheck_Edge::Tolerance` | The surface of the second pcurve on a closed surface is placed with the representation's location only (`cr->Location()`), the first one's with the edge's location too | On a located edge with a seam, the second pcurve is measured in the wrong place, and the tolerance comes out too large. By reading |
| `BRepCheck_Vertex::Tolerance` | A point on a curve whose curve is null leaves `Controlp` as the previous representation left it, and transforms it again | That point is counted at a wrong place. By reading |
| `BRepCheck_Face::OrientationOfWires` | Searches the map of wires for each wire and reads the key without checking the iterator ("No control on More()"); with `Update`, adds `myOrires` after a fault (BRepCheck::Add appends NoError to a list holding another status) | Undefined if a wire isn't found (the port throws Standard_NoSuchObject); a face without surface checked by the analyzer has NoSurface then NoError (harmless: IsValid reads the first) |
| `BRepCheck_Face.cxx`, `IsInside` | Evaluates the 3D curve at the pcurve's first parameter (`aCurve->D0(aFirst, ...)`) to measure the edge's length | Measures the wrong point on an edge whose pcurve and 3D curve aren't parametrised alike; harmless on same-parameter edges. By reading |
| `BRepCheck_Wire::SelfIntersect` | Returns NoCurveOnSurface or InvalidRange for an edge after the first with no pcurve or an empty range, without recording it even with `Update` | The wire's context status stays NoError while the call returns a fault. Ported as is |
| `BRepCheck_Wire::SetStatus` | Takes no lock, unlike Edge's and Face's | Nothing parallel calls it |

## Phase 3: unit 3L2a, Draw-lite checks

Found while porting Draw's checking commands (Draw-lite reproduces them, so the corpus sees OCCT's behaviour).

| Where | What | Effect |
|---|---|---|
| `isbbinterf` (BRepTest_BasicCommands.cxx) | Checks `theNArg < 2`, then reads `theArgVal[2]` | `isbbinterf a` passes a null name to DBRep::Get. The port gives the usage message for fewer than two shapes. By reading |
| `checksection` (BRepTest_CheckCommands.cxx) | `-r` reads `a[++anAI]` without checking that a value follows | `checksection s -r` calls Draw::Atoi on a null pointer. The port compares with 0. By reading |
| `checkshape` structural dump (`StructuralDump`, BRepTest_CheckCommands.cxx) | Writes `"%s_v"` (and `_e`, `_w`, `_f`, `_s`, `_o`) with the user's prefix into `char aName[20]` | A prefix of 18 characters or more overflows the buffer. The port has no limit. By reading |
| `checksection` (not a bug, a consequence) | Lists the lone vertices from a `NCollection_Map` hashed by TopTools_ShapeMapHasher, whose order follows the TShapes' addresses | The names `alone_1`, `alone_2`, ... aren't given in a reproducible order. The port names them in the order first met (decision 439) |
| `computetolerance` | Prints `(void*)&(*S.TShape())`, and calls `S.ShapeType()` without checking for a null shape | The output holds addresses; a null shape throws Standard_NullObject (as in the port). The port prints a per-run number instead of an address (decision 440) |

## Phase 3: unit 3K, distances and makers

| Where | What | Effect |
|---|---|---|
| `BRepExtrema_DistShapeShape::Perform`, the final filter ("Modified by Sergey KHROMOV") | Removes a solution whose distance exceeds `myDistRef + myEps` and then increments the index | The solution after a removed one is never checked. No input found that reaches it (every collected solution is within the deflection of the distance its task ended with). By reading |
| `BRepExtrema_DistanceSS.cxx`, static `TRIM_INFINIT_FACE` | `double arrU[8], arrV[8]` are read for every corner of the edge's box, but written only when `GeomAPI_ProjectPointOnSurf` is done | A failed projection takes the bounds from uninitialised memory in OCCT; the port's arrays start at zero. By reading |
| `BRepExtrema_DistShapeShape`, multi-threaded `SolidTreatment` (a property, not a bug) | Each task stops at its first vertex inside the solid and appends it under a mutex; the others stop only when they next read the done flag | Which vertices are returned as inner solutions, and how many, depends on the thread scheduling, in OCCT and in the port. The oracle checks those solutions as valid |
| `BRepExtrema_DistShapeShape` (a property, not a bug) | The work is split into tasks by the default thread pool's size in single-threaded mode too, each task stopping early on its own best distance | The solutions, their order and (where several pairs tie within the deflection) their number can depend on the machine's processor count. The port uses the same count (`Environment.ProcessorCount`) |
| `BRepExtrema_TriangleSet::Clear` (and so `Init`) | Clears the triangles and vertices but not `myShapeIdxOfVtxVec`, `myNumVtxInShapeVec`, `myNumTrgInShapeVec` | After a second `Init` (ShapeProximity's or SelfIntersection's reload) `GetShapeIDOfVtx`, `GetVtxIdxInShape` and the proximity statuses read the first set's entries. Pinned by `BRepExtremaTests.OcctIssue_TriangleSet_InitAgainKeepsTheFirstShapesVertexIds` |
| `BRepExtrema_TriangleSet::Init` | `isOK` ends the loop at the first shape without a mesh, but the method returns true whenever the BVH exists | The set silently holds only the shapes before the first unmeshed one, and the tools report it as loaded. Pinned by `BRepExtremaTests.OcctIssue_TriangleSet_InitIsTrueWithoutMeshes` |
| `BRepExtrema_OverlapTool::LoadTriangleSets` / `Perform` | The result maps `myOverlapSubShapes1/2` are never cleared | A ShapeProximity reloaded and performed again keeps the first run's pairs (and reports not done if the new sets don't overlap). Pinned by `BRepExtremaTests.OcctIssue_OverlapTool_KeepsOldPairs` |
| `BRepExtrema_ShapeProximity::Perform` | Passes `myTolerance` by reference to `BRepExtrema_ProximityValueTool::Perform`, which writes the proximity value into it | After computing a proximity value, `Tolerance()` is that value, and the next Load and Perform run the overlap test with it instead of computing a proximity value. Pinned by `BRepExtremaTests.OcctIssue_ShapeProximity_ValueBecomesTheTolerance` |
| `BRepExtrema_VertexInspector::Inspect` (BRepExtrema_ProximityValueTool.cxx) | Sets `myIsNeedAdd = true` at the start of every call | Only the last target inspected in the cells decides whether a bisection point is new: a coincident point found earlier is forgotten, and duplicates are added to the refinement points. Kept; by reading |
| `BRepBuilderAPI_Copy::Perform` (and every `BRepBuilderAPI_ModifyShape` reused on a shape it modified before) | `NotDone()` is meant to force a new copy, but `BRepTools_Modifier::Init` doesn't clear its map, and `Rebuild` returns the images it already has; `CreateNewVertices` makes the vertices again | The second copy returns the first result, and `ModifiedShape`/`Modified` of a vertex then answer a new vertex that isn't in it. Draw's `tcopy a b` reuses one framework, so copying a shape twice in one command shares the result. Kept; pinned by `BRepBuilderAPITests.Copy_makes_new_TShapes_and_can_be_reused` and compared with native OCCT by `BRepBuilderAPIModificationsOracleTests.Copy_answers_as_native_OCCT` |
| `BRepTools_Modifier::Rebuild` (face) with `BRepTools_NurbsConvertModification` | The NURBS conversion keeps a face on a B-spline or Bezier surface (`NewSurface` returns false) but copies its triangulation (`NewTriangulation` returns true), and the modifier then makes the face "with bare triangulation" | `BRepBuilderAPI_NurbsConvert` and `BRepBuilderAPI_GTransform` of a meshed shape give its B-spline and Bezier faces no surface (a meshed loft: six of its eight faces). Kept; pinned by `BRepBuilderAPITests.OcctIssue_NurbsConvert_of_a_meshed_BSpline_face_drops_its_surface`, and the same on native shapes in `BRepBuilderAPIModificationsOracleTests` |

## Phase 3: unit 3M, primitives

| Where | What | Effect |
|---|---|---|
| `BRepSweep_Rotation::MakeEmptyGeneratingEdge` | Calls `BRep_Tool::Curve(E, L, f, l)->Copy()` before testing the curve for null | Revolving an edge without a 3D curve dereferences a null handle in OCCT; the port throws NullReferenceException there. Marked `// OCCT`; by reading |
| `BRepPrim_GWedge::Vertex` | When the top is degenerated (Z2Min = Z2Max or X2Min = X2Max), the vertex built is copied to its partners in pairs, a partner not built yet as a null vertex, and both are marked built | A later request for that partner returns a null vertex instead of building it. The box and wedge makers' build order doesn't reach it (oracle). Ported as is; by reading |
| `BRepPrim_Cone(Angle, Axes)` (the infinite cone on axes) | Leaves `myRadius` uninitialised | The lateral face's radius is garbage. No maker uses this constructor; the port's field is 0 |
| `BRepPrimAPI_MakeRevolution`, static `Project` | Passes GeomProjLib::Curve2d's result on without checking it | A meridian whose projection fails gives a null 2D meridian, which BRepPrim_Revolution dereferences when the shape is built. Ported as is; by reading |
| `BRepPrimAPI_MakeCylinder`, `MakeSphere`, `MakeTorus` (documentation) | Document Standard_DomainError for a radius or height not over Precision::Confusion() | Nothing checks them: a null or negative radius builds a shape (BRepPrim_Cone alone checks its parameters). Ported as is |
| `BRepPrim_GWedge::Face`, `BRepPrimAPI_MakeBox::BottomFace` ... (a property, not a bug) | Return the face as built so far; building the other faces adds pcurves to its edges later, in place | A face asked for first changes after it was handed out. The port publishes the face once all faces are built (decision 467) |
| `BRepPrimAPI_MakeRevol::Build`, the degenerated edges (a property) | Iterates an NCollection_DataMap of faces hashed by TShape address to decide which face keeps the original degenerated edge | Not reproducible from run to run; the port iterates in binding order (decision 470). No difference seen in the oracle |
| `pcone`, `psphere`, `ptorus` (BRepTest_PrimitiveCommands.cxx) | The forms whose first number may be a plane read it with `DrawTrSurf::GetPlane` and use the handle without checking it | A word that isn't a plane dereferences a null handle in DRAWEXE; Draw-lite reports "DrawTrSurf::Get: not a plane". By reading |
## Phase 3: unit 3L2b, Draw-lite construction and distances

Found while porting Draw's construction, distance and modification commands. Draw-lite reproduces the command bugs, so
the corpus sees OCCT's behaviour; all were found by reading unless a test is named.

| Where | What | Effect |
|---|---|---|
| `setorigin` (GeomliteTest_CurveCommands.cxx) | The `else { return 1; }` pairs with the 2D B-spline test | On a 3D B-spline curve the command sets the origin and then fails with an empty message. Kept; unit test |
| `2dbeziercurve` (GeomliteTest_CurveCommands.cxx) | Counts the words per pole as `(n - 2) / np` where the 3D command uses `n - 3` | A curve of one pole without weight is read as weighted. Kept |
| `extsurf`, `revsurf`, `trim` (GeomliteTest_SurfaceCommands.cxx), `gcarc ... cir` (GeometryTest_ConstraintCommands.cxx) | Read `a[5]`, `a[8]`, `a[8]` and `a[5]` without checking the count | With one word fewer they pass Tcl's null `argv[argc]` to Draw::Atof or DrawTrSurf::GetPoint. The port reads an empty word |
| `gcarc` (GeometryTest_ConstraintCommands.cxx) | Returns 1 after binding the segment or arc, 0 after printing its usage | A successful gcarc is a Tcl error with an empty message: OCCT's scripts (samples/tcl/bottle.tcl) call it inside `catch`. Kept; unit test |
| `pmirror`, `pscale`, `2dpmirror`, `2dpscale` (DrawTrSurf.cxx, `transform`, `d2transform`) | Don't take the point's coordinates off the list of names, as `lmirror`, `smirror`, `translate` and `rotate` do | The coordinates are looked up as names (harmlessly) and printed in the result (`pmirror p 0 0 0` returns `p 0 0 0 `). Kept; unit test |
| `visoedge` (BRepTest_CurveCommands.cxx, `isoedge`) | `UMax = std::max(VMax, std::max(p1, p2))` | The trimmed surface's U range ends at the face's V maximum. Kept |
| `vertex name P` (BRepTest_CurveCommands.cxx), `mksurface`, `pcurve edge face` (BRepTest_SurfaceCommands.cxx) | Dereference the down-cast point, the face's surface or the edge's pcurve without a test | A name that isn't a point, a face without surface or an edge without pcurve on the face is an access violation. The port throws Standard_NullObject |
| `length` (GeomliteTest_CurveCommands.cxx) | `di << a[1] << "is not a curve\n"` | No space before "is". Kept |
| `BRepLib::UpdateTolerances` in place (`InternalUpdateTolerances` with `IsMutableInput`, then `UpdShTol`) | A vertex TShape reached at several locations is a key per location in an NCollection_DataMap hashed by TopTools_ShapeMapHasher (the TShape's address); with `theVForceUpdate` each key's tolerance is written into the shared TShape | The vertex keeps the tolerance of the key iterated last, in an order set by addresses: the result isn't reproducible. The port iterates in insertion order (decision 18). data/occ's solid.brep, mal_tige.brep and MODERN_MVCooker_1.brep (`updatetolerance s 1`) differ from native OCCT only there (DrawLiteConstructionOracleTests); the reshaper form makes a copy per location and is reproducible |
| `proximity`, `selfintersect` (BRepTest_ExtremaCommands.cxx; a property, not a bug) | List the overlapping sub-shapes by iterating an `NCollection_DataMap<int, ...>` | The order of the printed names is the map's bucket order. The port prints them in ascending order; the names are the same |
| lowalgos/extss/bug29712_44 (a test, not the library) | Computes `extrema s1 s1` and `extrema s2 s2`, each plane with itself, and expects the distance 10 between the two parallel planes | The distance is 0, so the case fails in DRAWEXE as in Draw-lite (`extrema s1 s2` was meant). By reading |
## Phase 4: unit 4L1, Draw-lite mesh readers

Found while porting Draw's mesh readers (tclean, trinfo, triepoints, trmergenodes, correctnormals, triarea, tricheck,
mpparallel). Draw-lite reproduces the command bugs, so the corpus sees OCCT's behaviour; by reading unless a test is
named.

| Where | What | Effect |
|---|---|---|
| `BRepLib::EnsureNormalConsistency` (`correctnormals`) | Computes the missing normals from `aPT->UVNode(i)` without checking that the triangulation has UV nodes | On a triangulation without UV nodes (Poly_MergeNodesTool's result, so after `trmergenodes`) OCCT reads past an empty array. The port throws Standard_OutOfRange, which Draw-lite reports as CommandCmd does. Marked `// OCCT` in the command; unit test |
| `BRepLib::EnsureNormalConsistency` (already in 3I) | Uses a null polygon on triangulation | The oracle confirms it natively: OSD_Exception_ACCESS_VIOLATION on several data/occ and dataset files, where the port fails too (DrawLiteMeshOracleTests counts them as failing on both sides) |
| `tricheck` (MeshTest_PluginCommands.cxx), the check of the meshes inside the faces | `if (aBoundaryEdgeMap.Size() == 0) break;` | The first face with no polygon on its triangulation (a face meshed without its edges, or not meshed) ends the loop: the faces after it are never checked for "Not connected mesh inside face". Kept; unit test |
| `MeshTest_CheckTopology::Perform`, cross-face errors | Records `i1` and `i2`, the indices in the edge's polygons, where tricheck prints them as "node1" and "node2" | The numbers printed aren't node numbers. Kept |
| `tricheck -small` | Takes `const gp_Trsf& aTrsf = aLoc.Transformation()` before `BRep_Tool::Triangulation(aFace, aLoc)` fills `aLoc` | The null-area triangles are drawn untransformed. Display only; Draw-lite draws nothing |
| `trmergenodes` without `-oneFace` (MeshTest.cxx) | Replaces each face's triangulation (`UpdateFace(F, T, false)`) and leaves the edges' polygons on the former triangulation | The edges keep polygons on triangulations no face carries: `tricheck` finds no boundary on the face and stops (above), `tclean` doesn't remove those polygons (it walks the faces' triangulations). Kept; unit test |
| `correctnormals` (MeshTest.cxx) | Reads `argv[1]` without checking the count | Without an argument DBRep::Get gets Tcl's null `argv[argc]`. The port reads an empty word: no shape, "Normals have not been changed!" |
| `triepoints` (MeshTest.cxx; a property, not a bug) | Numbers the edges by iterating an `NCollection_Map` of shapes, hashed by TShape address | The edge numbers in the vertex names aren't reproducible. The port numbers the edges in TopExp_Explorer's order |

## Phase 4: unit 4S, ShapeAnalysis

Found while porting ShapeExtend, ShapeAnalysis's wire analysis and ShapeBuild. Reproduced in the port, each marked
`// OCCT`.

| Where | What | Effect |
|---|---|---|
| `ShapeAnalysis_Curve::ValidateRange` (ShapeAnalysis_Curve.cxx) | Tests `IsPeriodic` on the basis curve (through trimmed and offset curves), but passes the curve's own `[cf, cl]` to `ElCLib::AdjustPeriodic` as the period | On a trimmed arc of a circle, the trimmed range is taken as the period: the reversed range (2, 1) on the arc [0, 3] becomes (2, 4), past the arc's end. By reading; pinned by `ValidateRange_on_a_trimmed_arc_wraps_with_the_trimmed_range_ReproducesOcctError`; the oracle matches native on trimmed circles |
| `ShapeAnalysis_Edge::CheckOverlapping` through the static `IsOverlapPartEdges` (ShapeAnalysis_Edge.cxx) | The sampling step is half the shorter edge's length | With a degenerated (zero-length) edge whose vertex is within tolerance of the other edge, the step is 0 and the loop never ends. Found when the first oracle run hung; the oracle skips degenerated edges. Not pinned (it would hang) |
| `ShapeAnalysis_Edge::CheckOverlapping` | Doesn't reset the status; in the domain branch, the `aStartLength < 0` case sets `aEndLength`, which the next line overwrites | Status flags add up across calls. By reading |
| `ShapeAnalysis_Edge::CheckVerticesWithCurve3d` and `CheckVerticesWithPCurve` | OR the enumerator `ShapeExtend_DONE1` into the status, not its encoding | Harmless: equal for DONE1 and DONE2. By reading |
| `ShapeExtend_WireData::Add(const Handle(ShapeExtend_WireData)&, int)` | In non-manifold mode adds `wire->Edge(i)` where it means `wire->NonmanifoldEdge(i)` | The other wire's INTERNAL/EXTERNAL edges are lost and ordinary edges added twice. Pinned by `Add_wire_data_in_non_manifold_mode_adds_the_wrong_edges` |
| `ShapeAnalysis_WireOrder::SetChains` | Also checks the wrap-around from the last edge to the start of its own chain | On an open chain the last chain's start is appended twice: 3 chains, the second the empty range (3, 2). Pinned by `Chains_of_an_open_and_a_closed_wire` |
| `ShapeAnalysis_WireOrder::Gap(0)`, `SetCouples` | `myGap` is never computed, and SetCouples is empty | `Gap()` is always 0 and `NbCouples()` always 0. Pinned |
| `ShapeAnalysis_TransferParameters` constructors | Leave `myMaxTolerance` uninitialised | Undefined until `SetMaxTolerance`; the port starts it at 0 |
| `ShapeAnalysis_Wire::CheckLoop` (ShapeAnalysis_Wire.cxx) | Sets the analyser's precision to `Precision::Infinite()` and, on the FAIL2 return for an edge without vertices, doesn't restore it | Later checks on the same analyser run with an infinite precision and pass everything. By reading |
| `ShapeAnalysis::TotCross2D` | With no pcurve of the wire on the face, the points stay at their defaults | The result is 0, so `IsOuterBound` counts such a wire as outer. By reading |

## Phase 4: unit 4A, mesh foundations

Found while porting Message, NCollection_MapAlgo, IMeshData, IMeshTools, BRepMeshData and BRepMesh's geometry tools.
All are reproduced and pinned by unit tests.

| Where | What | Effect |
|---|---|---|
| `BRepMesh_GeomTool::IntSegSeg` (BRepMesh_GeomTool.cxx) | `aPointHash[0]`/`[1]` classify the second segment's start/end on the first segment, `[2]`/`[3]` the first's on the second; the `PointOnSegment` branch returns the first segment's start for `[0]`, its end for `[1]`, the second's start for `[2]` and its end for `[3]` | The point given with `PointOnSegment` is an end point of the wrong segment. BRepMesh_Delaun, its only caller asking for it, recomputes the chopping point and ignores it, so meshes aren't affected. Kept (`// OCCT`); BRepMeshGeomToolTests |
| `Message_Algorithm::AddStatus` (Message_Algorithm.cxx) | Copies the other algorithm's strings with `for (n = 1; n < Length; n++)` | The last string of each status is dropped. Kept; unit test |
| `Message_Msg::Get` (Message_Msg.cxx) | Replaces the unfilled placeholders but keeps their positions | A second `Get` garbles the text ("a %s b" → "a UNKNOWN b" → "a UNKNOWNKNOWN b"); `SendStatusMessages` calls `Get` on a stored custom message, so sending it twice garbles it. Kept; unit test |
| `Message_Msg::Set` (Message_Msg.cxx) | After "%%", reads the next characters as a format | "100%% done" gets a placeholder "% d": `Arg(7)` gives "100 7one". Kept; unit test |
| `NCollection_MapAlgo::Differ` (NCollection_MapAlgo.hxx; a property, not a bug) | Reports a change by comparing the counts before and after | Swapping one key for another returns false. Kept; unit test |

## Phase 4: unit 4C, model and edges

Found while porting BRepMesh's model building, edge discretisation, healer and pre- and post-processors. Reproduced in
the port, each marked `// OCCT`.

| Where | What | Effect |
|---|---|---|
| `BRepMesh_FaceChecker`'s selector, `Accept` (BRepMesh_FaceChecker.cxx) | When a wire is checked against itself, a crossing is kept only if the loop it closes has an area; the area sums the cross products of each segment's second point, in the order of each pcurve's parameter, not the wire's | Along reversed edges the second points are the wire's first ones: a bow tie bounded by reversed edges measures zero area, its crossing is taken for a tangency, and the face is meshed with a self-intersecting wire (the same bow tie with forward edges fails with SelfIntersectingWire). Pinned by `Face_checker_misses_a_crossing_of_reversed_edges_as_OCCT_does` |
| `BRepMesh_ModelPostProcessor`'s PolygonCommitter, `commitPolygons` (BRepMesh_ModelPostProcessor.cxx) | Stores a seam's first discrete pcurve on the face as the polygon of the forward edge and the last as the reversed edge's, whatever their orientations | The model's pcurves follow the wire order of ShapeAnalysis_Wire::CheckOrder. On BRepPrimAPI's cylinder (top, seam reversed at u = 0, bottom, seam forward at u = 2pi) `BRep_Tool::PolygonOnTriangulation` of the forward seam gives the nodes at u = 0, the reversed seam's side. Same 3D points; the UV side is swapped. Pinned by `Seam_polygons_are_stored_in_the_model_order_as_OCCT_does`; the oracle compares native polygons in this order |
## Phase 4: unit 4B, Delaunay core

Found while porting the Delaunay core and Delabella. Reproduced in the port, each marked `// OCCT`. Of the
BRepMesh_Delaun and DataStructureOfDelaun entries, only the first is on the mesher's path, where it sizes the
circles' cells.

| Where | What | Effect |
|---|---|---|
| `CDelaBella::Triangulate` (delabella.cpp), all points cocircular | With more than 3 points, all cocircular ("[NFO] all input points are cocircular."), the last point is retracted and made the cone's top, which is coplanar with the base; no point is left for the main loop, and the final loop over `2 * points - 4` faces reads the adjacency (`f[0..2]`) of faces this run never allocated | Native (MSVC, release) crashes with an access violation, confirmed on the unit square's corners and on the twelve integer points of the circle of radius 5. The port throws NullReferenceException. Pinned by `Exactly_cocircular_points_fail_as_in_OCCT_ReproducesOcctError`. BRepMesh_DelabellaBaseMeshAlgo adds the four corners of an enlarged box to the face's nodes, so it can't reach this with distinct nodes |
| `CDelaBella::Triangulate`, 0 points | The duplicate removal starts with `w = 0, r = 1` and ends with `w = 1`: it reports "detected -1 dups" and sets `points = 1`, then the single-point branch writes `vert_alloc[0]` of an empty allocation (null on a first call) | Undefined behaviour natively; the port throws NullReferenceException. Pinned by `No_points_fail_as_in_OCCT_ReproducesOcctError` |
| `CDelaBella::Triangulate`, `assert(0)` and `assert(_f != face_alloc)` | Release builds compile the asserts out: a coplanar point that is on neither side of the contour is taken as `last` without being linked in; the emergency search for a visible face runs below `face_alloc` when none is visible | Undefined behaviour natively (the port throws IndexOutOfRangeException in the second case). By reading; not reached by any test |
| `IDelaBella::Triangulate` (a property, not a bug) | The header says triangles are returned clockwise "in abc"; the Delaunay triangles (lifted faces with `n.z < 0`) are clockwise in `v[0..2]`, and the hull contour counter-clockwise | BRepMesh_DelabellaBaseMeshAlgo takes `v[0], v[2], v[1]` to get counter-clockwise triangles. Tested |
| `BRepMesh_Delaun::initCirclesTool` (BRepMesh_Delaun.cxx) | `if (NbNodes() > 100) aScaler = 5; else if (NbNodes() > 1000) aScaler = 7;` | The second branch is unreachable: the circles' cells are at most a fifth of the box for any number of nodes above 100. Kept; by reading |
| `BRepMesh_DataStructureOfDelaun::SubstituteElement` and `clearDeletedLinks` | `myElements(theIndex) = ...` on an `NCollection_DynamicArray` indexed from 0, where `GetElement(theIndex)` reads `theIndex - 1` | The triangle after the one meant is replaced; past the last triangle the write goes into unused capacity (or, at a block boundary, a missing block). The port replaces the next triangle and drops a write past the end. The mesher calls neither. Pinned by `SubstituteElement_writes_one_triangle_too_far_ReproducesOcctError` |
| `BRepMesh_VertexTool::RemoveLast`, through `BRepMesh_DataStructureOfDelaun::clearDeletedNodes` | `RemoveLast` is `DeleteVertex(Extent())`, which marks the vertex deleted but doesn't shrink the vector | `clearDeletedNodes` calls it once per moved node: from the second on it deletes the same last vertex again instead of the current last one, which stays live as a duplicate of the node copied from it, and `NbNodes()` never decreases. The mesher doesn't call `ClearDeleted`. Pinned by `ClearDeleted_renumbers_links_and_nodes_ReproducesOcctError` |
| `BRepMesh_Triangulator::prepareMeshStructure` | Maps the triangles' nodes back to the wires' points with `myTmpMap.Bind(aNumNode + 1, ...)`, while the triangles use the indices `AddNode` returns, and `AddNode` merges points within `Precision::Confusion()` | With two coincident wire points the indices shift: triangles get the wrong points, or `Find` throws and `Perform` returns false. By reading |
| `NCollection_Sequence::Remove(from, to)` in `BRepMesh_Delaun::meshPolygon` (a property of release builds) | When the crossing link's start is the closer end and the two frontier links are consecutive, the range is `(aPolyIt + 1, aPolyIt)` | OCCT's range check is compiled out in release builds and the call removes nothing. The port's `Remove` checks, so `meshPolygon` skips that call (`removeRange`). By reading |
| `BRepMesh_CircleInspector::Inspect` (OCCT's own comment) | Compares `d² - r²` with the square of the linear tolerance | The circles are effectively exact. OCCT keeps it because the correct test hangs "perf mesh bug27119". Kept |
| `BRepMesh_Delaun` with free nodes only (a property, not a bug) | `frontierAdjust`'s `cleanupMesh` removes every triangle with a free link whose nodes no frontier holds, until none is left to remove | Triangulating points without a frontier keeps only an inner part of their Delaunay triangulation (nothing for three points). The mesher always has a frontier. Tested |
| `BRepMesh_Delaun::createTrianglesOnNewVertices` (a property, not a bug) | `Contains` reports a non-free link when the node is within a fifth of its length, and a free node there is skipped | A free node closer to a frontier link than a fifth of the link's length is never inserted. Tested |

## Phase 4: unit 4D, face meshing and the incremental mesh

Found while porting the face algorithms, range splitters and factories. Reproduced in the port, each marked `// OCCT`.
Native BRepMesh_IncrementalMesh (Watson) itself showed no bug in the oracle: every compared face is bit-identical.

| Where | What | Effect |
|---|---|---|
| `BRepMesh_DelabellaMeshAlgoFactory::GetAlgo`, `default:` (BRepMesh_DelabellaMeshAlgoFactory.cxx) | Surfaces of extrusion, offset and other surfaces fall to the default branch, which uses `BRepMesh_NURBSRangeSplitter` (the Watson factory gives extrusions BRepMesh_ExtrusionRangeSplitter). Its `getUndefinedIntervalNb` asks the surface for `NbVPoles()`, and `GeomAdaptor_Surface::NbVPoles` throws Standard_NoSuchObject for a surface of extrusion | With `-algo 1` (Delabella) a face on a surface of extrusion is left without a mesh: BRepMesh_BaseMeshAlgo's `catch (Standard_Failure)` swallows the exception without setting a status, so GetStatusFlags() is 0. By reading OCCT's code; the port does the same. Pinned by `Delabella_leaves_an_extrusion_unmeshed_as_OCCT_does_ReproducesOcctError` |
| `BRepMesh_BaseMeshAlgo::Perform` (a property) | A failure caught while a face is meshed leaves the face without a triangulation and without a status; only FaceDiscret's own catch sets IMeshData_Failure | A face can be silently unmeshed (see the row above). Kept |

## Phase 4: unit 4L2, incmesh and the corpus

Found while porting incmesh, tessellate, the mesh factory's commands and AIS's meshing. By reading unless a test is
named; Draw-lite keeps OCCT's behaviour. The oracle found no difference from native incmesh or AIS meshing.

| Where | What | Effect |
|---|---|---|
| `incmesh` (MeshTest.cxx), argument parsing | Any word that parses fully as a real (`IsRealValue(true)`) is taken as the deflection, before shape names are looked up | A shape named like a number can't be meshed by name. Kept |
| `incmesh` | Returns 0 whatever the statuses (Failure, SelfIntersectingWire, ...) | A failed mesh is only visible in the text; the mesh group's TODOs match it there. Kept |
| `incmesh -prs` with a deflection | The deflection given becomes the drawer's deviation coefficient, so `incmesh s 1 -prs` meshes at 4 times the shape's size | The help text calls it "0.001 of bounding box linear deflection"; the value is taken as written. Kept; unit test |
| `StdPrs_ToolTriangulatedShape::GetDeflection` (a property) | Stores the absolute deflection it computes into the object's drawer as its own maximal chordial deviation | An object displayed with a relative deflection keeps that absolute value: a later `vdefaults -absDefl` doesn't reach it. Kept (Prs3d_Drawer is a working object) |
| mesh group, standard_shading/M4 (a test reference) | The script expects an area difference of 1.9456% when meshed for shading, under `TODO ?OCC22687` | The port's AIS mesh, bit-identical to native's by the oracle, gives 1.918%: the optional TODO makes the case BAD, not FAILED. The reference may come from another OCCT version's shading deflection |
| v3d/bugs/bug29020 (a test of the viewer) | Reads the maximal texture size from `vglinfo` | Can't run without OpenGL; FAILED in Draw-lite, which ignores vglinfo |
## Phase 4: unit 4W, viewer extraction, benchmarks and WebAssembly

No OCCT bug found. The extraction calls BRep_Tool, BRepLib_ToolTriangulatedShape::ComputeNormals,
BRepAdaptor_Surface/Curve and GCPnts_TangentialDeflection as build123d.NET's native shim does, and its buffers are
float-identical to native's on every compared face and edge.

| Where | What | Effect |
|---|---|---|
| build123d.NET's 3d_mesh_extract (not OCCT) | Calls BRepLib_ToolTriangulatedShape::ComputeNormals on the face's attached triangulation, which writes the normals into the meshed shape | Harmless there (the shape is build123d.NET's private meshed copy). The port computes them on an unattached copy (decision 569), with the same values |
| BRepLib_ToolTriangulatedShape::ComputeNormals (a property) | Normals are stored in single precision (Poly_Triangulation::SetNormal), then read back as gp_Dir (normalised again in double) before the location's rotation | The viewer's normals go through float twice. Kept: the port must give native's floats |

## Phase 5: unit 5E, IntPolyh

All reproduced. The ones that change results are parity-sensitive for units 5D and 5G (IntPatch_PrmPrmIntersection
starts its walking lines from IntPolyh's section lines and tangent zones).

| Where | What | Effect |
|---|---|---|
| `IntPolyh_Intersection::PerformMaillage` (standard), enlarged-zone retry | When no couple is found, the retry refills the points on the enlarged sampling and recomputes everything on the same `IntPolyh_MaillageAffinage`. `IntPolyh_Array::Init` keeps the items already there, and `FillArrayOfTriangles` only ever clears the intersection-possible flag | The first pass's triangles keep their flags (out of the common box, split, non-interfering), their cached bounding boxes (computed from the unenlarged points), deflections and edges; triangles beyond the first pass's count are reused with stale state when refining. A first pass that rejected everything (disjoint boxes, the usual reason for no couple) makes the retry find nothing; a fresh object on the enlarged sampling would. Reproduced; test `BSplinesBeyondBounds_EnlargedRetry_KeepsTheFirstPassTriangles` |
| `IntPolyh_MaillageAffinage::FillArrayOfPnt(SurfID, isShiftFwd, Upars, Vpars, theDeflTol)` | Calls the point-normal overload with `1` instead of `SurfID` | Always fills the first surface's points. IntPolyh_Intersection doesn't call it (it uses the point-normal overload directly); `FillArrayOfPnt(SurfID, isShiftFwd)` does. Test `FillArrayOfPnt_ShiftedWithSampling_FillsTheFirstSurface` |
| `IsDegenerated` (IntPolyh_MaillageAffinage.cxx) | Samples the isoline at `i * dU` (and `i * dV`), not `aU1 + i * dU` | The middle sample is off the surface's range unless it starts at 0, so a degenerated bound isoline can be missed (its points then aren't flagged, and its triangles get a deflection instead of being degenerated) |
| `CalculPtsInterTriEdgeCoplanaires`, the edge not parallel to the side, second point, `TriSurfID == 2` | Takes the side's orientation from `Tri1` where the first point's branch reads `Tri2` | The second start point's `Lambda2` may be `1 - alpha` instead of `alpha` |
| `CalculPtsInterTriEdgeCoplanaires`, the edge parallel to the side | For `TriSurfID == 2`, `SetUV1(PEP.U(), PTP.V())` and `SetUV2(PTP.U(), PEP.V())` mix the two points' U and V; one branch builds `PEP2` from `PE2 + Edge * lambda2` where the others start from `PE1`; the second point takes `alpha1` and `lambda1`, not `alpha2` and `lambda2`, and for `TriSurfID == 2` the orientations from the other triangles | Coplanar overlapping edges give start points with wrong parameters and positions on the edges (the GTest's sphere/plane and sphere/cylinder cases only check that they are finite) |
| `IntPolyh_MaillageAffinage::TriangleEdgeContact`, coplanar case | One of the duplicate checks compares `SP1.V2()` with `SP2.V1()` | Two distinct points can be merged, or a duplicate kept |
| `IntPolyh_MaillageAffinage::TriangleEdgeContact`, `TriSurfID == 2`, on the second side (alpha 1) | Sets `Lambda2` from `alpha`, where the first surface's branch uses `beta` | The start point's position on the edge is 1 (or 0) whatever beta is |
| `IntPolyh_MaillageAffinage::TriContact` | Leaves the angle unchanged when a triangle's normal vanishes; TriangleCompare passes one variable for all couples | Such a couple gets the previous couple's angle (or -2), which feeds `IsAdvRequired`, `AnalyzeIntersection` and the start points' incidence |
| `IntPolyh_StartPoint::CheckSameSP` | Prints both points and "Can't Check" to standard output when both edges are unknown; TriangleEdgeContact prints "Cote PT2PT3 nul1" | Debug output in a release library. Left out of the port (decision 596) |
| `IntPolyh_MaillageAffinage::CommonBox(B1, B2, ...)` | Ignores its box arguments and uses its own | Harmless (its only caller passes its own boxes) |
| `IntPolyh_MaillageAffinage::GetFinTE`, `GetFinTT` | Declared, never defined | A link error for a caller; left out (decision 597) |
## Phase 6: unit 6S, Sewing

Found while porting BRepBuilderAPI_Sewing and FastSewing. Reproduced in the port, each marked `// OCCT`.

| Where | What | Effect |
|---|---|---|
| `BRepBuilderAPI_Sewing::Perform` → `EdgeRegularity` (BRepBuilderAPI_Sewing.cxx) | EdgeRegularity ends with `myMergedEdges.Clear()`, and CreateOutputInformations, called after it, binds a section to its boundary only if the section is in `myMergedEdges` | With sewing on (the default), `IsSectionBound` is always false and `SectionToBoundary` always throws; they work only in analysis mode (`option1 = false`). Pinned by `A_bound_is_cut_at_a_T_junction_and_its_sections_know_their_boundary`; the oracle compares the number of bound sections with native's |
| `BRepBuilderAPI_Sewing::SameParameterEdge` (the two-edge form) | When the second edge has only the reversed pcurve of a seam on a face (`c2d2` null, `c2d21` not), dereferences the null `c2d2` | A crash in native OCCT; a NullReferenceException in the port. By reading |
| `FreeEdge`, `MultipleEdge`, `ContigousEdge`, `ContigousEdgeCouple`, `DeletedFace`, `DegeneratedShape` | The range checks accept index 0 (`index < 0 \|\| index > Nb...`) | Index 0 then fails in the map (Standard_OutOfRange) instead of the method's own check. Kept |
| `BRepBuilderAPI_Sewing::EdgeRegularity` | Iterates `myMergedEdges` (a map keyed by shapes) in address order | Harmless: each edge's regularity is encoded on its own. The port iterates in insertion order |
| `BRepBuilderAPI_Sewing::Dump` | Writes to `std::cout` only | The port adds `Dump(TextWriter)` |
| `BRepBuilderAPI_Sewing` (native runs, found by the oracle) | The sewn shape's sub-shapes come out in an order that can change from run to run of the same sewing in one process (address-ordered maps; plan 2.4) | tests/sewing tol_0_01/Q5 and CNP001_projoiei.rle at 1e-6 gave two native orders over seven runs; the port's order is one of them, the same every run. The oracle compares with every native variant |
| `BRepBuilderAPI_Sewing` (in place, found by the oracle) | Sews its inputs in place: in a few cases (tests/sewing G3, G4 at 0.01, Y7 at 100; CCH_propag.rle, CCH_r2.rle at 1e-6) it writes into input edges or vertices (SameParameterShape runs BRepLib::SameParameter on every edge of the result, the inputs' own edges included), and then answers that they aren't modified (`IsModified`, `IsModifiedSubShape`) | The port never writes a frozen input: it sews a copy (decision 600) and answers that those inputs are modified, giving the changed copy. Expected; the oracle detects the native write and compares everything else |
## Phase 5: unit 5A, intersection foundations

Found while porting IntSurf, IntImp, IntStart and IntPatch's data model and tools. By reading OCCT's code unless a
test is named; the port reproduces each (`// OCCT` at the line). No class of this unit is bound natively, so none
is confirmed by the oracle yet (5G).

| Where | What | Effect |
|---|---|---|
| `IntPatch_GLine::ComputeVertexParameters`, the third filter (points on S2 only) | Tests `VTXi.IsVertexOnS1()` where `IsVertexOnS2` is meant; always false for these points | Of two coincident points on S2, point i is always the one removed, even when it is the vertex. Kept |
| `IntPatch_GLine::Replace` | Appends the point and ignores Index (the replacement is commented out) | A "replaced" vertex is added at the end and the old one stays. Kept; pinned by `IntPatch_GLineTests` |
| `IntPatch_GLine::ComputeVertexParameters` (a property) | Sets `indf = 1` and `indl = nbvtx` without setting `fipt`/`lapt` | HasFirstPoint/HasLastPoint stay as they were. Kept; pinned |
| `IntPatch_ALine::ComputeVertexParameters`, the periodic-line block | The `Prepend` branch is inside `VTX0 == ParamMinOnLine` but requires `VTX0 != ParamMinOnLine` (and measures from VTX0 where VTXN seems meant) | A closed line whose last vertex is at its end never gets the matching vertex at its start. Kept |
| `IntPatch_WLine::ComputeVertexParameters`, the insertion of a vertex before the line | The loop that shifts the other vertices tests `j != 1` where `j != i` is meant | Vertex 1 keeps parameter 1 while its point moves to index 2. Kept; pinned by `ComputeVertexParameters_reproduces_OCCTs_unshifted_vertex_before_the_line` |
| `IntPatch_WLine::ComputeVertexParameters`, the seam phase | The S2 creation block runs even after the S1 block has appended a vertex; its search loop is skipped and its else branch resets `encoreunefois` | The loops continue past the new vertex. Kept |
| `IntPatch_WLine::Dump`, `IntPatch_RLine::Dump`, mode 0 | Print `Vertex(pol)` instead of `Point(pol)`, and only while `pol <= NbVertex` | The dump shows vertices where points are announced. Kept |
| `IntPatch_RstInt.cxx`, `Recadre`, torus case | The V2 loop compares `V2 < V1p - 1.5π` where V2p is meant | A torus's V2 can be left a period off. Kept |
| `IntPatch_PolyArc::SetOffset` | `Get` returns the bounds with the gap, and the box is reset to the unshifted bounds before the shifted ones are added | The box covers both positions and grows by the gap on each call. Kept |
| `IntPatch_HInterTool::SamplePoint`, perturbation 1 | `dv *= 1.001; dv *= 0.999;` scales dv twice (du was presumably meant first) | The sample is moved by a factor 0.999999 in v only. Kept; pinned by `IntPatch_HInterToolTests` |
| `IntStart_SearchOnBoundaries.gxx`, `BoundedArc` (a site, decision 610) | `std::sort` of the roots (SolInfo) by parameter: ties keep their order with MSVC on up to 32 roots (insertion sort) | The port sorts as MSVC on up to 32; with more roots, tied parameters could come out in another order than native's |
| `IntImp_ZerImpFunc`, `IntImp_ZerParFunc`, `IntPatch_CSFunction` (a property, decision 608) | Keep `void*` pointers to the caller's handles (and quadric) | A later reassignment of the caller's handle variable would change the function's surface in OCCT; the port keeps the object. No caller does it |
| `IntSurf_Transition::SetValue()` and `SetValue(Tangent, Type)` (a property) | Leave the other fields (situation, opposite flag, tangency) as they were | Two in transitions can differ in fields their queries don't show; MakeTransition passes the caller's objects (`ref` in the port). Kept |
## Phase 6: unit 6A, BOP data and reports

Each bug is reproduced by a test and marked `// OCCT` at the line, and ported as OCCT has it unless the entry says
otherwise.

| Where | What | Effect | Test |
|---|---|---|---|
| Message_Report::AddAlert | When `Message_CompositeAlerts::AddAlert` merges the new alert into one of the same type, it returns false. The report then tests `anAlerts.Extent() > myLimit`, which is always true with the default limit of -1, and removes the first alert of that gravity. The limit is never applied to an appended alert | Simple alerts merge by default, so adding the same simple alert twice with the same gravity leaves none; a third add appends it again. In BOPAlgo, `AddError(new BOPAlgo_AlertTooFewArguments)` twice gives `HasErrors() == false`, and merging two reports that hold the same simple alert type loses alerts. If another alert is first in the list, that one is removed instead | `AddAlert_MergingRemovesTheFirstAlert_OcctIssue`, `AddAlert_LimitAppliesOnlyOnMerge_OcctIssue`, `SimpleErrorAddedTwice_OcctIssue` |
| Message_CompositeAlerts::Clear(type) | The loop's else branch calls `anIt.More()` instead of `anIt.Next()` | An alert not of the cleared type makes the loop run forever, so `Message_Report::Clear(type)` hangs. **Fixed minimally** (`Next()`), because a hang can't be shipped | `Clear_AllGravityAndType` |
| IntTools_Curve::D0 | The bounds test is `thePar >= First \|\| thePar <= Last`, where `&&` is meant | Every parameter counts as inside the curve and is evaluated | `Curve_D0_OcctBug_EvaluatesOutsideTheBounds` |
| IntTools_CommonPrt copy constructor | Sets `myAllNullFlag(false)` instead of copying it (`Assign` does copy it) | A copied common part loses its all-null flag | `CommonPrt_OcctBug_CopyConstructorDropsTheFlag` |
| IntTools_CommonPrt::Assign | Appends the other part's Ranges2 to its own instead of replacing them | `a = b` accumulates ranges. BOPDS_InterfEE/EF::SetCommonPart goes through it, so setting a common part twice accumulates the second edge's ranges | `CommonPrt_OcctBug_AssignAppendsTheRanges`, `InterfEE_SetCommonPartTwice_AccumulatesTheSecondEdgeRanges` |
| IntTools::RemoveIdenticalRoots | After `aSR.Remove(k)` the loop goes on at k+1, so the root moved into slot k is never compared with root j | With roots 1.0, 1.0001 and 1.0002 and eps 1, the result is 1.0 and 1.0002, though 1.0002 is within eps of 1.0 | `RemoveIdenticalRoots_OcctBug_SkipsTheRootAfterARemoval` |
| BOPDS_FaceInfo::Clear | Clears the In and On maps but not the Sc maps | A cleared face info keeps its section pave blocks and vertices | a BOPDS_DSTests case |
| BOPDS_DS::Paves | Sizes its array as the number of pave blocks plus one. If fewer distinct paves are found, the default paves left in the array (index -1, parameter 99) are sorted and appended too; OCCT's assert only fires in debug builds | Spurious paves in the list for an edge whose pave blocks share paves | — (kept; reached by 6E) |
| BOPDS_DS::BuildBndBoxSolid | Called after `Init` with one argument, it returns a void box: `prepareSolids` has already replaced the solid's shells by faces, edges and vertices, so no shell is found | A void box for such a solid | a BOPDS_DSTests case |
| BOPAlgo_Builder::SetArguments (stub, OCCT's code) | Clears `myArguments` but not the fence map `myMapFence` | Setting again arguments that were added before leaves none | `BuilderSetArguments_Twice_KeepsNothing_OcctIssue` |
| BOPAlgo_ToolsProvider::SetTools | The same, with `myMapTools` | As above, for tools | — (ported, not yet marked or tested) |

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

| Where | What | Effect |
|---|---|---|
| `ShapeExtend_CompositeSurface::Copy` | Initializes the copy with `Init(patches)`, whose parametrisation defaults to natural | A copy (and so `Transformed`, `Translated`, ...) loses uniform, unitary or custom joint values. Pinned by `CompositeSurface_Copy_LosesJointValues`; the port's `With*` twins keep them |
| `ShapeConstruct::JoinPCurves` | Returns `i <= edges->Length()`: false when every pcurve was joined, true when the loop broke early (or on a plane) | The inverse of its documentation. No OCCT caller reads it. Pinned by `JoinPCurves_TwinJoinsTwoEdgesOnACylinder` |
| `ShapeConstruct::ConvertSurfaceToBSpline`, the continuity-lowering loop | `if (cnt > 0) cnt--; continue;` retries C0 as long as GeomConvert_ApproxSurface throws | Can loop forever. By reading |
| `ShapeAnalysis_ShapeTolerance::InTolerance` | Tests vertices with `tol >= valmax` where faces and edges test `<=` | `OverTolerance(s, 0)` returns every vertex and nothing else; a vertex inside [min, max] isn't found. Pinned |
| `ShapeAnalysis_Geom::PositionTrsf` | `SetScale` resets the whole transformation | Any scale other than 1 discards the rotation computed before it. Pinned |
| `ShapeAnalysis_WireVertex::Analyze` | Compares vertices with `V1 == V2`, orientation included | An edge's REVERSED last vertex never equals the next edge's FORWARD first one: status 0 ("same vertex") is never recorded, shared vertices get 1. Pinned |
| `ShapeAnalysis_WireVertex::SetStart` | Stores the parameter on the preceding edge in `myUFol` instead of `myUPre` | The preceding parameter is lost. Pinned |
| `ShapeAnalysis_WireVertex::NextCriter(1)` | Tests `stat > 0` where the documentation says `>= 0` | By reading |
| `ShapeAnalysis_CheckSmallFace::CheckPin` | Reports `whatrow = nbu` when row 1 of U is the pin; the status and the DONE3 equal-poles check come from the last bound tested only | By reading |
| `ShapeAnalysis_CheckSmallFace::CheckTwisted` | Computes the V step from the U range (`dv = (umax-umin)/nbint`) | The V sampling is wrong on non-square ranges. Pinned (a twist is still found) |
| `ShapeAnalysis_CheckSmallFace::CheckPinEdges` | Tests `(M_PI - angle2)` twice; the second is probably meant for angle1 | By reading |
| `ShapeAnalysis_CheckSmallFace` constructor | Leaves myStatus, myStatusPinFace, myStatusPinEdges and myPrecision uninitialised | The port starts them at OK and -1 |
| `ShapeAnalysis_CheckSmallFace::IsSpotFace` | Returns 1 ("spot") for a face without wires and leaves spot and spotol unwritten | Pinned |
| `ShapeAnalysis_CanonicalRecognition::GetSurface` (shell form) | The loop recomputes the parameters from the first face's surface instead of `anElemSurf1` | Every later face compares equal: any shell whose first face fits is accepted. By reading |
| `ShapeAnalysis_CanonicalRecognition::GetCurve` | When the edge's 3D curve already is the target conic and the edge has a location, transforms the edge's own (shared) curve in place | Native OCCT changes shared geometry, and repeated calls accumulate the transformation. The port copies the curve first (decision 660). Pinned (three calls give the same result) |
| `ShapeAnalysis_CanonicalRecognition`'s FillSolverData (cone) | The semi-angle's upper bound uses the radius step `aDR` instead of `aDAng` | By reading |

## Phase 5: unit 5H, hatching

Found while porting Hatch, HatchGen, Geom2dHatch and GeomAPI_PointsToBSplineSurface. By reading OCCT's code unless a
test is named; the port reproduces each (`// OCCT` at the line). The hatcher has no native binding: unit 6B checks
it through IntTools_Context and BOPTools.

| Where | What | Effect |
|---|---|---|
| `Geom2dHatch_Hatcher::RemElement` | `HatchGen_PointOnHatching PntH = Hatching.ChangePoint(IPntH);` copies the point, so the element's points on element are removed from the copy | The hatching's points keep the removed element's points on element; only a point that had no other element is removed. As the trimming stays done, a later `ComputeDomains` reuses those points and their transitions. Kept; pinned by `RemElement_KeepsTheOtherElementsPointsOnHatching` |
| `Geom2dHatch_Elements` copy constructor ("magic constructor") | Copies nothing: the new set is empty, with current edge 0 and probing parameter 0 (the default constructor sets 1 and 0.123) | A copied set of elements classifies as if it had none. Kept; pinned by `Elements_CopyConstructorCopiesNothing` |
| `Geom2dHatch_Hatcher::GlobalTransition` | The parameter chosen from the point's position on the element (its first or last parameter at an end) is overwritten by the point's parameter (OCCT's own "TODO: finding #25") | The local geometry at an element's end is taken at the intersection's parameter, not at the end. Kept |
| `Geom2dHatch_Hatcher::Trim(IndH, IndE)` | Writes " Intersector -> Done = False " to std::cout when an intersection fails | Output from a library; left out of the port (decision 666) |
| `Geom2dHatch_Hatcher::IsDone()` (no index) | Declared, never defined | A link error for a caller; left out |
| `Geom2dHatch_Element` default constructor | Leaves the orientation uninitialised | The port's is FORWARD |
| `Hatch_Hatcher::StartIndex`, `EndIndex` | Don't check the oriented mode where `Start` and `End` do | On an unoriented hatcher, a line without intersections gives index 0 and parameter 0 for interval 1 where `Start` throws. Kept |
| `GeomAPI_PointsToBSplineSurface::Init` (all forms) | Sets each multi-point at the grid's own row index (`MP.SetPoint(i, Points(i, j))` from `LowerRow()`) and each line entry at its column index (`Line.SetValue(j, MP)` from `LowerCol()`); the periodic form closes with `Points(1, j)` | A grid not indexed from 1 throws Standard_OutOfRange (or closes on the wrong point). Kept |

## Phase 5: unit 5B, analytic intersections

Found while porting IntPatch_ImpImpIntersection, by reading OCCT's code; the port reproduces each (`// OCCT` at the
line). The conics are confirmed bit-identical by the early oracle (decision 676); the rest is 5G's to confirm.

| Where | What | Effect |
|---|---|---|
| `Perform`, the cone/torus case (`case 35`/`53`) | Doesn't set `bEmpty = empt` after IntCoTo, as every other pair does | An empty cone/torus intersection still runs the boundary search; with points on the boundaries it reports a non-empty result. Kept |
| `IntCyCo`, the `IntAna_Point` case | `Quad1.Parameters(psol, U2, V2)` where Quad2 is meant | The isolated point's parameters on the second surface are those on the first. Kept |
| `ProcessBounds` | The loop over `slin` advances `j` only on analytic lines | A non-analytic line in `slin` while a bound is still unprocessed loops forever. Its callers put only ALines in `slin` first, so it isn't reached. Kept |
| `WorkWithBoundaries::BoundariesComputing`, `mB < 0` | Tests `mB <= -mB - 1` where its comment's derivation gives `mB <= -mC - 1` | The U1 range for that sign of B is chosen on a wrong bound. Kept |
| `WorkWithBoundaries::BoundariesComputing`, `mB > 0` | `mB + |mC| < -1` can never be true for `mB > 0` | The "no solution" branch is dead. Kept |
| `ProcessSegments` (a property) | `ptvtx` lives across the segments and the vertex loop; an end point no line carries is built from the last vertex read, setting only its value, parameters and (for a vertex) one arc | The vertex can keep the arc, vertex and multiple flag of an unrelated vertex on the other surface. Kept |
| `IntCyCo`, `IntCoCo`, `IntCoSp` (a property) | `ptf`/`ptl` are declared outside the loop over the IntAna curves | Where a curve's bound is open, ProcessBounds gets the previous curve's end point (unused while that bound is open). Kept |
| `CyCyAnalyticalIntersect` (a property) | Doesn't reset `Multpoint` | Harmless: IntCyCy resets it before |

## Phase 5: unit 5F, approximating intersection lines

Found while porting ApproxInt, GeomInt_WLApprox and BRepApprox. The port reproduces each (`// OCCT` at the line), and
a native program built against OCCT 8.0.1 (decision 680) confirmed the first and the third; `ApproxIntTests` pins
the native values.

| Where | What | Effect |
|---|---|---|
| `ApproxInt_PrmPrmSvSurfaces::Compute`, `ApproxInt_ImpPrmSvSurfaces::Compute`, the two-entry cache | On a hit (the same parameters as one of the last two computations) they return the cached flag without writing their outputs: the point, the tangents and the refined parameters stay as the caller gave them | PrmPrm's `SeekPoint` called twice at the same parameters gives the origin with the unrefined parameters the second time, as a valid point (native: `(0, 0, 0)`); `MakeMLBetween` would insert a stale point if a resampled parameter set repeated one of the last two. Kept; pinned by `PrmPrmSvSurfaces_SeekPointTwice_ReturnsTheOriginTheSecondTime` and `ImpPrmSvSurfaces_ComputeTwice_LeavesTheOutputsTheSecondTime` |
| `ApproxInt_ImpPrmSvSurfaces::Compute`, the parametric surface first | Stores the cache key `MyParOnS2.SetCoord(tu2, tu2)` where `(tu2, tv2)` is meant | The point is never found in the cache (unless tu2 = tv2) and is computed again: the same values, more work. Kept |
| `ApproxInt_Approx::Perform(line, ...)` (the line alone) | With two knots and more than 2·NbPntMax points it adds a middle knot `(indicemax - indicemin) / 2`, an index count rather than `indicemin + ...` | From a first index above 1 the knots go backwards: on a straight line of 300 points, `Perform(WL, ..., 150, 300)` gives a curve from point 75, and from 200 a curve from point 50 (native too). Harmless from index 1 (BRepApprox's callers). Kept; pinned by `WLApprox_PerformLine_MiddleKnotIsAnIndexCount` |

## Phase 5: unit 5C, walking

Found while porting IntWalk, IntPatch_ImpPrmIntersection and IntPatch_SpecialPoints. By reading OCCT's code; the port
reproduces each (`// OCCT` at the line). None changes the nine walks recorded natively (decision 687), which the port
matches bit for bit.

| Where | What | Effect |
|---|---|---|
| `IntWalk_IWalking::Perform(Pnts1, Func, S, Reversed)` (the overload without interior points) | Doesn't call `Clear()`, unlike the other overload: the previous lines are kept, and on a new object the vectors lack their dummy item 0 | The walking reads start point I + 1's state as point I's, and past the vectors' end for the last one (out of bounds in OCCT; Standard_OutOfRange in the port). No OCCT caller uses this overload |
| `IntWalk_IWalking::TestArretCadre` (IntWalk_IWalking.gxx, at a stop found on the line) | Computes the function's direction, orients it against the line, then sets the line's tangent to the previous direction instead | The computed direction is unused; `Direction3d` still throws at a tangent point. Kept |
| `IntPatch_ImpPrmIntersection.cxx`, `ToSmooth` (line 2422) | The loop runs `ip` from `startp` to `NbTestPnts`, where `startp + NbTestPnts` is meant | At the end of a line (`startp = NbPoints - NbTestPnts - 2`) the loop is empty unless the line is short; the averages stay 0, so AddVertices never adds the last vertex. Kept |
| `IntPatch_ImpPrmIntersection.cxx`, `DecomposeResult` (line 3716) | The adaptor `anArc->Trim(...)` returns is discarded | The arc is never trimmed. Kept |
| `IntPatch_ImpPrmIntersection.cxx`, `ComputeTangency` (lines 365–373) | On an INTERNAL or EXTERNAL arc, the complex-transition branch never assigns the vertex handle `vtx` | It keeps an earlier point's vertex, or is null, which would crash OCCT in `Identical` (a NullReferenceException in the port). Kept |
| `IntPatch_ImpPrmIntersection.cxx`, `DecomposeResult` (lines 3188–3195; a property) | `aSSLine` is a second handle to the input line's points | AdjustLine and InsertSeamVertices edit the input line in place, even when it is replaced afterwards. Kept |
| `IntWalk_PWalking` (the constructor with the bounds; a property) | Leaves `tgfirst`, `tglast`, `myTangentIdx` and `previoustg` uninitialised | Perform writes them before reading them; harmless |
## Phase 6: unit 6H2, ShapeFix edges and wires

Found while porting ShapeFix's statics, Root, Edge, EdgeProjAux, Wire, WireSegment, WireVertex, IntersectionTool,
ShapeTolerance, SplitCommonVertex, EdgeConnect and SplitTool. Each is reproduced in the port and marked `// OCCT`
unless the table says otherwise; "pinned" names the unit test that shows it. Native ShapeFix_Shape on wires and edges
(the oracle) showed no difference from native.

| Where | What | Effect |
|---|---|---|
| `ShapeFix_ShapeTolerance::LimitTolerance`, TopAbs_WIRE | The edges' result isn't added to `fait` | A change to the edges' tolerances alone returns false. Pinned by `LimitTolerance_Wire_DoesntCountTheEdges` |
| `ShapeFix_EdgeProjAux::Init2d`, `UpdateParam2d` | `theCurve2d->Reverse()` on the handle `BRep_Tool::CurveOnSurface` returns | Computing two parameters reverses the edge's own pcurve, and every shape's sharing it. The port reverses a copy and never writes the edge (the reversed curve isn't read afterwards) |
| `ShapeFix_Edge::FixReversed2d` | Reverses the stored pcurve in place | Other edges sharing the pcurve are reversed too. The port puts a reversed copy in place of the curve on this edge only (decision 700) |
| `ShapeFix_WireVertex::FixSame` | Compares `V1 == V2` (orientation included); `B.Add`s the shared vertex without removing the edges' old ones; doesn't make the edges Free first, as `Fix` does | A shared vertex is never detected; the edges end with three or four vertices; on the edges of a wire `Add` throws `TopoDS_FrozenShape`. Pinned |
| `ShapeFix_IntersectionTool::FixIntersectingWires` | Calls `sewd->Wire()` three times per modified wire | The face gets one new wire, the context records a second as the replacement and the loop goes on with a third: `Context()->Value(oldWire)` isn't the face's wire |
| `ShapeFix_Wire::FixTails` | Ends with `myStatusNotches = myLastFixStatus` | `StatusFixTails()` always reports OK, and the notches' status is overwritten by the tails' |
| `ShapeFix_Wire::FixGap2d`, clipping an extended pcurve at the surface's bounds | Uses `jj` (which end of the segment) where `j` (which curve) is meant to pick `reversed1`/`!reversed2` and `cfirst1`/`clast2` | The segment ends are clipped on the wrong curve's side. By reading |
| `ShapeFix_Edge::FixAddPCurve`, seam branch | Dereferences the projected pcurve, which can be null | Inside OCC_CATCH_SIGNALS it ends as FAIL2. The port throws `Standard_NullObject` inside the same `try`, with the same status |

Map order: `ShapeFix_EdgeConnect::Build` walks `myLists` (a DataMap keyed by vertex) in native's hash order; the port
walks it in insertion order (decision 18). Each shared vertex is computed on its own, so only the order of the writes
changes.

## Phase 5: unit 5D, parametric intersections

Found while porting IntPatch_PrmPrmIntersection, the polyhedra and their interference. By reading OCCT's code; the
port reproduces each (`// OCCT` at the line). None changes the seven intersections recorded natively (decision 711),
which the port matches bit for bit.

| Where | What | Effect |
|---|---|---|
| `IntPatch_InterferencePolyhedron::Perform(const IntPatch_Polyhedron&)` | Calls the self-interference `Interference(Obje)`, whose body is empty; the constructor of the self-interference calls `Interference(Obje, Obje)` instead | `Perform` on one polyhedron finds nothing (only the mode and tolerance are set). OCCT's own callers use the constructor. Kept, and pinned by a test |
| `IntPatch_Polyhedron.hxx` | Declares `Perform`, `UMinSingularity`... (the four setters) and `HasUMinSingularity`... (the four getters), which no .cxx defines | A caller would fail to link. Left out of the port |
| `IntPatch_Polyhedron::PlaneEquation` | On a triangle with an edge shorter than 1e-7, sets the normal to (1, 0, 0) and returns without writing `PolarDistance` | The caller's variable keeps its value (uninitialised in a typical caller). Nothing in OCCT calls it; kept (`ref` in the port) |
| `IntPatch_Polyhedron::TriConnex` (the end; OCCT's own "BUG NON CORRIGE") | When the other point of the connected triangle coincides with `Pedge`, returns 0 instead of looking further, with `TriCon` and `OtherP` set | InterferencePolyhedron reads `TriCon` (into `pivo`) to tell whether the neighbouring triangle has already given the segment of an edge lying in the other triangle's plane; at such a vertex it decides on the triangle found so far. Kept |
| `IntPatch_PrmPrmIntersection::NewLine` (line 1555) | Tests `nab > ds`, where `nab = 1 / (a + b)` is the inverse of the segment's length and `ds` the arc-length step | A point is inserted only in segments shorter than `1 / ds` (in parameter units, so the test depends on the parametrisation's scale). Nothing in OCCT calls NewLine. Kept |
| `IntPatch_PrmPrmIntersection::Perform` (the sampled one, the branch for more than 2,500 samples) | Doesn't clear the lines, whatever `ClearFlag` says (the other branch clears them when it is true) | A second sampled Perform on the same object keeps the first one's lines. IntPatch_Intersection uses a fresh object (or wants them kept). Kept |
| `IntPatch_PrmPrmIntersection::Perform` (with the list of points; a property) | Appends the points' images across the seams of closed surfaces to the caller's list | The caller's list grows. Kept |
| `IntPatch_PrmPrmIntersection::PointDepart` (lines 3689-3718; a property) | Converts `(P.X() - x0 + dx2) / dx` to `int` without a range check (undefined behaviour in C++ out of range; MSVC gives `INT_MIN`) | Out of range and NaN samples fall outside the grid. The port reproduces MSVC's result (decision 708) |
## Phase 6: unit 6B, context and BOP tools

Found while porting IntTools_Context, FClass2d, TopolTool and Tools and BOPTools' AlgoTools (all three files),
AlgoTools2D and AlgoTools3D, and auditing the context's caches (plan 2.4, decision 714). Each is marked `// OCCT` at the
line (or commented) and ported as OCCT has it; "pinned" names the unit test that shows it.

| Where | What | Effect |
|---|---|---|
| `IntTools_FClass2d::Init` | Doesn't clear TabClass/TabOrien or reset the face explorer | A second Init appends the new face's polygons to the old ones, and a point the polygons can't decide is classified against the old face. IntTools_Context never re-initialises a classifier. By reading |
| `IntTools_FClass2d::Init` | Returns on an edge without a pcurve, with the classifier half built (the wires sampled so far, partial bounds, U1..V2 unset) | The classifier answers from the partial polygons. The port's unset fields are 0 |
| `IntTools_FClass2d::Init`, refinement loop | Never recomputes the polygon's area and perimeter, so `anExpThick` stays the first polygon's | By reading |
| `IntTools_FClass2d::IsHole` | Reports the orientation of the last wire with a non-zero area | A face whose hole is drawn after its outer wire reports `true`. Pinned |
| `IntTools_TopolTool::SamplePoint` | Maps index 1 to the second sample in U (`iu = 1 + Index - (iv-1)*n`, as if the index started at 0) | Index n·m gives `v = vsup`, on the boundary, and the first grid node is never sampled. Pinned |
| `IntTools_Tools::IsVertex(Edge, t)` | Computes the vertex tolerance, then overwrites it with 1e-12 (squared) | The vertex's tolerance is ignored. By reading |
| `IntTools_Tools::RejectLines` | Writes into its const input through a const_cast (a trimmed curve replaced by its basis) | The caller's sequence changes. Kept |
| `BOPTools_AlgoTools::ComputeStateByOnePoint` | Discards the state of the recursive call on a container's first sub-shape | Returns TopAbs_UNKNOWN for any compound, shell, solid. Pinned |
| `BOPTools_AlgoTools_1.cxx`, `IntersectCurves2d` | `aLen2 = MapEdgeLength(*theEData1.Edge, …)` uses the first edge twice | The distance threshold ignores the second edge's length. By reading |
| `BOPTools_AlgoTools2D_1.cxx`, file-static `IsClosed` | The loop condition `(aExp.More() \|\| iCnt == 2)` keeps going once a seam is counted twice at the end; `TopExp_Explorer::Next()` on an empty stack pushes the face's iterator again, so the count reaches 4 | A true seam gives false, so `AttachExistingPCurve` never calls `UpdateClosedPCurve`, and the split of a seam gets one pcurve. Pinned (`IsClosed_of_AlgoTools2D_1_reproduces_OCCTs_restarting_loop`) |
| `BOPTools_AlgoTools3D::GetNormalToSurface` | Calls `IsDirsCoinside(aDD1U, aDD1U)`, comparing DU with itself | That check always passes. By reading |
| `BOPTools_AlgoTools3D.cxx`, `HasGeometry` (IsEmptyShape) | Calls `CR->Polygon3D()` on every representation that isn't a 3D curve, curve on surface or regularity; the base class throws Standard_DomainError | An edge whose first such representation is a polygon on triangulation or on surface throws instead of reporting geometry. By reading |

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

| Where | What | Effect |
|---|---|---|
| `ShapeFix_Face::FixMissingSeam`, two open wires of the same orientation | `isdeg2 = isdeg` stores the flag (1) where the other branch stores the wire's index | A later `ws.Remove(isdeg ? i : isdeg2 ? isdeg2 : isdeg1)` can remove wire 1 instead of the degenerated one. By reading |
| `ShapeFix_Face::FixSmallAreaWire` | Tests `ShapeType() != WIRE && Orientation() != FORWARD && != REVERSED` where `\|\|` is meant | FORWARD/REVERSED vertices and INTERNAL wires reach `TopoDS::Wire`; OCCT's release build reinterprets a vertex as a wire without edges, which is kept. The port does the same instead of throwing. Pinned by a ShapeFix_FaceTests case |
| `ShapeFix_Face::Perform` (a property) | `BRepTools::Update` sets the Checked bit of the caller's face in place | The port never writes the caller's face; the oracle masks the Checked column with the Free one |
| `ShapeFix_Wireframe::FixWireGaps`, `FixSmallEdges` on a compound | Each recursive call resets the status | The statuses are the last child's only. Reproduced by the oracle |
| `ShapeFix_Wireframe::MergeSmallEdges`, the free-wire block | Reads `anExpf2.Current()` after that face explorer is exhausted | It yields the explored shape; the port has the same semantics |
| `ShapeFix_Shell::Perform` | When FixFaceOrientation gives up (Möbius-like faces) it sets FAIL but not DONE2, and the new shell isn't recorded in the context | Perform returns false. Pinned by the Möbius band test |
| `ShapeFix_Shell::FixFaceOrientation` | Always rebuilds the shell, even when nothing is reoriented, and then records nothing | `Shape()` is a new shell holding the same faces, while the context maps the input to itself |
| `ShapeFix_Shape::Perform` | The early return (a shape already fixed or made by the context) and the abort paths don't restore the face tool's FixSmallAreaWireMode | A later Perform on a non-face can keep the forced mode |
| `ShapeFix_FreeBounds::Perform` | Removes and adds an edge's vertices while a TopoDS_Iterator walks them | The list iterator reads a freed node; the vertex order inside the edge then depends on freed memory. The port walks a snapshot (the same vertices) |
| `ShapeFix_FreeBounds::Perform` | `BRep_Builder::Remove`/`Add` on edges that sit in wires (not Free) | `TopoDS_FrozenShape` is compiled out in OCCT's release build but raised by its debug build and by the port. The port sets the Free flag around the edit, as ShapeFix_EdgeConnect does in OCCT |
| `ShapeFix_FixSmallFace::SplitOneFace` | Removes and adds edges of `wireonface` while a TopExp_Explorer walks it | The same freed-node read; the port explores a snapshot first |

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

| Where | What | Effect |
|---|---|---|
| `IntPatch_Intersection::Perform(S1, D1, S2, D2, TolArc, TolTang, ...)`, two tori | `bToCheck = aTor2.MajorRadius() > aTor2.MinorRadius()` overwrites the first torus's test; the list-of-points Perform combines both and also accepts equal radii within TolTang | A self-crossing first torus is sent to ImpImp by one overload and not by the other. Kept |
| `IntPatch_Intersection.cxx`, `FUN_GetUiso` (offset surface whose basis U-iso is a line, parabola or hyperbola) | `Geom_RectangularTrimmedSurface(gos, VmTr, VMTr, true)` gives the V range as a U trim; `FUN_GetViso`'s call is right | The U-iso of the trimmed offset keeps its infinite V range. Kept |
| `IntPatch_Intersection.cxx`, `FUN_NewFirstLast` | The second test isn't `else if` | A line, parabola or hyperbola range on the positive side (both ends ≥ 0) falls to the final `else` and is replaced by the symmetric [-TrVal, TrVal]; a hyperbola then also misses the `Fst > 4` early return. Kept |
| `IntPatch_LineConstructor::Perform`, the GLine branch and the "RLine on S2" branch | `myDom1->Init(); if (myDom2->More())` tests the other domain's iteration state | The restriction loop runs, or not, on the state myDom2 was left in. Kept |
| `IntPatch_LineConstructor::Perform`, the same branches | Read `Tol3d(...)` from the domain, which a plain `Adaptor3d_TopolTool` doesn't have (Standard_DomainError) | LineConstructor on GLines or one-sided RLines works only with a face's topology tool. OCCT reaches the class only from TopOpeBRep (and `IntPatch_Intersection::Dump`, compiled out). Pinned by a test |
| `IntPatch_LineConstructor::Perform`, GLine branch, closed conics | The arc from the last vertex to the first is classified with the `Tol` of the loop's last iteration | Kept |
| `IntPatch_WLineTool::ComputePurgedWLine` | `nb` is reassigned inside the equal-points loop; the later `nb < aNbSingleBezier` test uses the count before the last removal | Kept |
| `GeomInt_IntSS::InternalPerform` | Clears `sline` but not `slineS1`/`slineS2` | On a GeomInt_IntSS performed twice, `LineOnS1/S2(i)` and `HasLineOnS1/S2(i)` return the first Perform's 2D curves. GeomAPI_IntSS never asks for them. Kept |
| `GeomInt_IntSS::InternalPerform`, start-point mode | Skips `myIntersector.Perform` when the start point is outside a domain | `IsDone()` and the lines come from the previous Perform on the same object (or "not done"). Kept |
| `GeomInt_IntSS::BuildPCurves` (GeomInt_IntSS_1.cxx) | Calls `theCurve2d->DynamicType()` after a second `GeomProjLib::Curve2d` that may return null | Access violation in OCCT, NullReferenceException in the port. Kept |
| `GeomInt_LineTool::DecompositionOfWLine` | Its two neighbour searches step one index past a part before testing | When all points of an end part coincide in UV, `Point(i)` is read past the line (unchecked in OCCT's release build; Standard_OutOfRange in the port). Kept |
| Draw's `intersect`, curve/surface branch (GeometryTest_CurveCommands.cxx) | The points' loop calls `Inters.Point(i)` with `i` running from `nblines + 1` | With segments and points both, the points read are past the last one (Standard_OutOfRange, reported by Draw) and the first points are skipped. Kept in Draw-lite |

## Phase 6: unit 6C, edge intersectors

Found while porting IntTools_EdgeEdge, IntTools_EdgeFace and IntTools_BeanFaceIntersector. Each is ported as OCCT has
it, except the first (decision 753).

| Where | What | Effect |
|---|---|---|
| `IntTools_BeanFaceIntersector::LocalizeSolutions`, `ComputeLocalized` with `IntTools_Context::SurfaceData` | The face's sample boxes are cached across runs, keyed by sample only, enlarged by the first edge's criteria, and a cached box skips the check against the run's main curve box (6B's entry) | A run's result depends on what its context computed before, so native's parallel edge/face results depend on thread order. Measured (tools/edgerec `study`): 1 run in 25,308 (8,436 pairs on localising faces, in order, reversed and with alternating fuzzy values) differs from a fresh context, by 32 ulps in a range. **The port keys the cache and checks cached boxes as fresh ones (decision 753)**, so it always gives the fresh-context result; that run is the residual difference from native's serial result |
| `IntTools_BeanFaceIntersector`, `myMinSqDistance` | Only `ComputeAroundExactIntersection` and `ComputeUsingExtremum` update it; the line/plane path, `FastComputeAnalytic`, the coincidence test and `ComputeLocalized` never do | `IntTools_EdgeFace::MinimalDistance()` is RealLast for edges on B-spline faces even with common parts (all 177 such recorded runs), and BOPAlgo_PaveFiller_5's distance check is skipped for them. Recorded |
| `IntTools_EdgeFace::CheckTouchVertex` | Builds its surface adaptor on the surface's natural bounds (`Surface->Bounds`), where `CheckTouch` uses the face's | On a plane or another infinite surface the extrema run on an unbounded surface. By reading |
| `IntTools_EdgeFace::CheckData` | Sets status 2 for a degenerated edge, then overwrites it with 3 if the edge has no geometry | The degenerated status is lost for such edges. By reading |
| `IntTools_BeanFaceIntersector::ComputeAroundExactIntersection` | With more than one exact intersection point it lowers `myCriteria` to 3·Confusion for the rest of the run (meant for the range search) | The extremum and boundary passes that follow use the lowered criteria too. By reading |
| `IntTools_CurveRangeLocalizeData::ListRangeOut` (decision 629) | Native lists the curve's out ranges in hash order, the port in insertion order; `ComputeLocalized` inserts them into the range manager with flag 1 | The ranges are disjoint and share one flag, so only zero-length ranges at shared ends can come out in a different order. No difference in the 1,562 recorded runs on localising faces |
## Phase 6: unit 6D, face/face intersection

Found while porting IntTools_FaceFace and IntTools_WLineTool. By reading OCCT's code unless said; the port reproduces
each (`// OCCT` at the line). None shows in the comparison with native IntTools_FaceFace (decision 761), where every
pair is bit-identical.

| Where | What | Effect |
|---|---|---|
| `IntTools_FaceFace::Perform` | Clears the curves but never `myPnts` (5G's `InternalPerform` bug in FaceFace's form) | A second Perform on the same object appends its points to the first's. Pinned by a test. The pave filler makes one object per face pair, so no Boolean is affected |
| `IntTools_FaceFace.cxx`, `PerformPlanes` | Without the second 2D curve (`theApprox2` false) it clears the first (`SetFirstCurve2d(H1)` in the second branch) | A plane/plane line asked for with the first 2D curve only has none. The pave filler asks for both or neither. Pinned by a test |
| `IntTools_FaceFace.cxx`, `PerformPlanes` | Two planar faces without wires (unbounded) give `ClassifyLin2d` the infinite bounds; `Geom_TrimmedCurve` on the resulting parameters throws | Standard_ConstructionError, natively too (three such pairs in the recordings). Pinned by a test |
| `IntTools_FaceFace::MakeCurve`, walking lines | `myTolApprox = 1.e-5` (cylinder/sphere pairs where `ApproxWithPCurves` is false, and the re-approximation after an invalid 2D curve) writes the member | The looser tolerance stays for the following lines and for later Performs on the object (until SetParameters) |
| `IntTools_FaceFace::MakeCurve`, `goto reapprox` | Restarts the whole line although the curves of its earlier parts (or multicurves) are already appended | Those curves are appended twice. Kept |
| `IntTools_FaceFace::MakeCurve`, lines, parabolas, hyperbolas | Checks `C2d.IsNull()` after `GeomInt_IntSS::BuildPCurves`, which has already dereferenced a null second projection (5G's issue) | The check can't help: OCCT crashes, the port throws NullReferenceException. Kept |
| `IntTools_FaceFace::SetList` and Perform | The start points are kept between Performs; on a reversed pair Perform swaps each point's parameters in place | A second reversed Perform swaps them back to the wrong order; the list also grows (decision 706) |
| `IntTools_FaceFace.cxx`, `CheckPCurve` | The sample parameter runs on from the curve's first parameter by `(NPoints - 1) * dT` per interval | With several C∞ intervals the samples drift from the intervals and the end of the last one is never checked |
| `IntTools_WLineTool.cxx`, `IsPointInDegeneratedZone` | Reads the first surface's bounds for the second (`aS1->Bounds(US21, ...)`), and compares V on both surfaces, and U on the second, with `aDelta` (1e-7) instead of the resolution `aD` it has just computed | The degenerated-zone test of the second surface uses the first's bounds and a 3D tolerance as a parametric one |
| `IntTools_WLineTool::DecompositionOfWLine`, "correction of projected coordinates" | `X < umin && X > umax && Y < vmin && Y > vmax` can't hold (and uses the other surface's bounds) | The rough re-projection never runs |
## Phase 6: unit 6U, UnifySameDomain

Found while porting ShapeUpgrade_UnifySameDomain. Each is reproduced in the port (on its copy of the input) and marked
`// OCCT` unless the table says otherwise.

| Where | What | Effect |
|---|---|---|
| `IntUnifyFaces` → `AddPCurves` → `TransformPCurves` | Adds the reference surface's pcurves to the edges of the faces it unifies, and resets their ranges (`BRep_Builder::Range`), on the input's own edges, whatever `SetSafeInputMode` says ("the input shape can't be modified") | The caller's edges change in place, and the result shares them. The port runs on a copy (decision 766); its history lists those edges as modified into their changed copies, where native's has no entry (decision 643). Reproduced by the oracle (37 of the 480 Clean runs have such entries) |
| `ReconstructMissedSeam` | `UpdateEdge` swaps the two pcurves of a removed seam edge of the input | The same in-place write into the input |
| `IntUnifyFaces`, ConcatBSplines with an edge closed on the face but not a seam | `SetUPeriodic`/`SetVPeriodic` on the face's own B-spline surface | Every face sharing that surface, the input's included, becomes periodic. The port makes a periodic copy and puts it in the original's place in the shape it works on (decision 769), so its result is native's byte for byte (`bowl_noremove.brep`) and the caller's surface is unchanged. On a rectangular trimmed surface native edits the shared basis; the port copies the basis only |
| `IntUnifyFaces`, the wire builder | `LastVertexOfSeam` and `StartOfNextEdge` are declared and tested but never set | The end-of-wire test after ReconstructMissedSeam never holds. Kept |
| `UpdateMapOfShapes` | Adds to the NCollection_Map it iterates | Whether the new shapes are visited depends on the buckets; they map to themselves, so nothing changes. The port iterates a snapshot |
| `UnifyEdges` with `mySafeInputMode` | `UpdateMapOfShapes(myKeepShapes, ...)` adds the context's images to the member map of shapes to keep | A second `Build` without `Initialize` keeps them too. The port restores the caller's map after Build (the images are of its copy) |
| `SetSafeInputMode(false)` | Builds pcurves on planes in the input's edges and lets ShapeFix_Face edit them without a context | The documented behaviour; in the port it acts on the copy of a frozen input |

Not bugs, but worth knowing: native UnifySameDomain iterates hashed maps only for membership (`myKeepShapes`, the shells of
a face), so its result doesn't depend on addresses; every oracle run was byte-identical or Free-masked.

## Phase 5: unit 5K1, analytic constraints

Found while porting GccAna. Each is ported as OCCT has it and marked `// OCCT` at the line; the native recordings
(tools/gccrec, decision 780) agree with the port on all of them. "Unset" means an NCollection_Array1 element OCCT
never writes: uninitialised natively, 0 in the port (decision 781). Out-of-range indices throw Standard_OutOfRange
natively and in the port (decision 778).

| Where | What | Effect |
|---|---|---|
| `GccAna_Circ2d3Tan.cxx` (three circles) | `distcc2` and `distcc3` are measured from `center1` | WhichQualifier's computed qualifiers 2 and 3 are wrong for unqualified circles |
| `GccAna_Circ2d3Tan.cxx` (three circles) | `rSol` has 16 slots where `xSol`/`ySol` have 64 | More than 16 candidate triplets throw Standard_OutOfRange |
| `GccAna_Circ2d3Tan_1.cxx` (circle, circle, line) | `ok` is not reset when the second circle's check fails, and the distance to the line is never compared with the radius | Qualified runs return circles tangent to the first circle only (56 of 144 qualifier combinations in a sweep). Pinned by a test |
| `GccAna_Circ2d3Tan_1.cxx` | The line's qualifier branches test `Qualified2` where `Qualified3` is meant; `distcc2` is measured from `center1`; the second tangency's guard tests `pnttg1sol` and writes `par1sol` | Wrong computed qualifiers and tangency parameters |
| `GccAna_Circ2d3Tan_3.cxx` (three lines) | `cote2` assigned where `cote3` is meant (all unqualified: harmless, that branch returns every solution); `cote3(1)` assigned twice and `cote3(2)` never, in the branches (unqualified, enclosed, unqualified) and (enclosed, unqualified, unqualified) | The solutions of those two branches are filtered with an uninitialised side: native's count varies (the recordings skip those cases) |
| `GccAna_Circ2d3Tan_3.cxx` | The qualified filtering moves the circle to its new slot but not `qualifier1..3` | Wrong qualifiers after filtering |
| `GccAna_Circ2d3Tan_4.cxx` (circle, circle, point) | The duplicate-removal loop doesn't shift `TheSame1..3` | IsTheSame of the solutions after a removed duplicate is that of their predecessors |
| `GccAna_Circ2d3Tan_5.cxx` (circle, line, point) | A 1999 change reversed `dc` | The first tangency point is on the far side of the solution unless the solution is enclosed in C1. Pinned by a test |
| `GccAna_Circ2d2TanOn.cxx` (two circles, centre on a line) | Up to four boundary cases or matches write `Radius(3..4)` into a 2-element array; the qualifiers of unqualified circles use `Radius(i)`, `i` the bisector's index, instead of `Radius(k)`; `Rbid(1)` is written twice and `Rbid(2)` never; `pararg1` is computed from `pnttg2sol` | Standard_OutOfRange in 189 of 1,500 random cases; otherwise wrong qualifiers and parameter |
| `GccAna_Circ2d2TanOn.cxx` | `CenterOn3` returns `pnttg1sol` (a tangency point), not the centre; `ThisSolution` checks neither IsDone nor the index, `NbSolutions` not IsDone | Callers reading the centre get a tangency point |
| `GccAna_Circ2d2TanOn_1.cxx` (circle, line, centre on a line) | The "other side" retry for `pinterm` recomputes the same point | The other side is never tried |
| `GccAna_Circ2d2TanOn_3.cxx` (circle, point, centre on a line) | `if (std::abs(popradius - dist1))` tests a double as a boolean; `pararg1(i)` uses the bisector's index; `pararg2` is unset on the boundary path; with C1 qualified the radius follows the qualifier whichever hyperbola branch the centre is on | Some solutions don't pass through the point (pinned by a test); unset and misplaced parameters |
| `GccAna_Circ2d2TanOn_6.cxx` (two circles, centre on a circle) | As `GccAna_Circ2d2TanOn.cxx`: `Rbid(2)` unset, `Radius` overrun | As above |
| `GccAna_Circ2d2TanOn_7.cxx` (circle, line, centre on a circle) | The retry uses the same `distcco - Ron` point and doesn't recompute `d1`/`d2`; the boundary test checks `Qualified1` against the line's side, with the same sign for enclosed and outside; the boundary case never sets `pntcen` | Missed solutions; `parcen3` is the parameter of the default origin |
| `GccAna_Circ2d2TanOn_8.cxx` (circle, point, centre on a circle) | The radius follows `Qualified1` only (both radii when unqualified), whichever branch the centre is on; `SameCenter` is always false | Some solutions don't pass through the point (pinned by a test) |
| `GccAna_Circ2d2TanOn_9.cxx` (two lines, centre on a circle) | `D1.Angle(D2) <= Resolution` is signed, so lines turning clockwise take the "parallel" branch, where the second line's qualifier test overwrites the first's; the centre is `Intp.Point(i)` (the first bisector's index) instead of `Point(l)`; the tolerance is unused | Wrong branch for half the line pairs; Standard_OutOfRange when that point doesn't exist |
| `GccAna_Circ2d2TanOn_10.cxx` (line, point, centre on a circle) | The boundary `pinterm` is `distpc + Ron` from the point but compared with `distpc - Ron`; `parcen3` is computed then overwritten with 0; tangency point 1 is at distance `dist1` instead of `Radius` | Wrong boundary detection, centre parameter and tangency point |
| `GccAna_Circ2d2TanOn_11.cxx` (two points, centre on a circle) | The `C1` enlargement branch repeats conditions that returned earlier | Dead code |
| `GccAna_Circ2d2TanRad.cxx` (two circles) | `C2 = gp_Circ2d(C1)` overwrites C2 in the enclosed/enclosed boundary case | `pararg2` is a parameter on C1 |
| `GccAna_Circ2d2TanRad.cxx` | `std::abs(R1 - dist - R2) > 0.0` (meant `<= Tol`) always holds | Enclosing + unqualified always returns one circle, not tangent to the second; `TheSame2(1)` unset when the solution is the first circle only |
| `GccAna_Circ2d2TanRad.cxx` | Outside + unqualified builds `C(2)` on `C(1).Location()` | The pair is concentric and 2 solutions are lost |
| `GccAna_Circ2d2TanRad_4.cxx` (two lines) | `normL1`/`normL2` are the lines' directions, not their normals | Wrong computed qualifiers for unqualified lines |
| `GccAna_Circ2dTanOnRad.cxx` (circle, centre on a line) | `CenterOn3` returns `pnttg1sol`, not `pntcen3`; the tangency point's sign is swapped in the general branch | The tangency point is diametrically opposite; enclosed with R1 = 2R puts it at C1's centre, whose parameter throws Standard_ConstructionError |
| `GccAna_Circ2dTanOnRad_1.cxx` (line, centre on a line) | `pntcen3(NbrSol) = cirsol(1).Location()` | Solution 2 reports solution 1's centre |
| `GccAna_Circ2dTanOnRad_2.cxx` (point, centre on a line) | With the point on OnLine, 2 solutions are found but WellDone stays false; `qualifier1` unset there | The accessors throw StdFail_NotDone |
| `GccAna_Circ2dTanOnRad_3.cxx` (circle, centre on a circle) | `qualifier1(NbrSol)` written while NbrSol = 0 | Standard_OutOfRange (a hand-picked recorded case) |
| `GccAna_Circ2dTanOnRad_4.cxx` (line, centre on a circle) | `sign = (int)dc1.Dot(normL1)` truncates to 0, so the tangency point is the centre | Every generic case throws Standard_ConstructionError (the parameter of the centre): 44 of 120 recorded cases |
| `GccAna_Circ2dTanOnRad_5.cxx` (point, centre on a circle) | `Location - signe * Point1` is a sum when `signe = -1`; the loop writes index 1, not NbrSol | Wrong boundary circle; solution 2's qualifier, IsTheSame and tangency data unset |
| `GccAna_Lin2d2Tan.cxx` (two circles) | `qualifier2(...) = Qualified1.Qualifier()` in almost every branch; for unqualified circles WhichQualifier returns GccEnt_unqualified, not a computed qualifier (also circle + point); the mixed branches write solution 2's qualifiers into `qualifier1(1)`/`qualifier2(1)`; unqualified/enclosing gives solution 2 the first's tangency point | Wrong or unset qualifiers; a wrong tangency point (pinned by a test) |
| `GccAna_Lin2dTanObl.cxx` | The second intersection is computed again with `linsol(1)`; a solution parallel to TheLine gets the intersection data of a default point | Intersection2(2) = Intersection2(1); (0, 0) data for parallel solutions |
| `GccAna_Lin2dTanPer.cxx` | `qualifier1` never set by the (point, line), (point, circle) and (circle, line) constructors; `maxdist = RealLast()` never lowered, so the last intersection point is kept, not the nearest; (circle, circle) always gives 2 solutions whatever the qualifier, tangent to C1 and perpendicular to the line of centres (they don't pass through TheCircle's centre); no BadQualifier check for noqualifier; ThisSolution throws Standard_RangeError | Unset qualifiers; wrong lines in the circle/circle case (pinned by a test) |

## Phase 6: unit 6E, the pave filler

Found while porting BOPAlgo_PaveFiller, by reading OCCT's code. The port reproduces each (`// OCCT` at the line) except
where the entry says otherwise. None shows in the comparison with native (decision 785), where all 914 runs are
bit-identical.

| Where | What | Effect |
|---|---|---|
| `BOPAlgo_PaveFiller::PutPaveOnCurve` (PaveFiller_6), new-pave branch | Raises the tolerance of the vertex put on a section curve with `BRep_Builder::UpdateVertex`, without the `IsNewShape`/same-domain check `UpdateVertex(nV, tol)` makes in the non-destructive mode; `PutPavesOnCurve` lets the common vertices of the two faces through | In the non-destructive mode an argument vertex's tolerance can be raised in place. On a frozen argument the port throws (BOPAlgo_AlertIntersectionFailed, decision 787). Not reached by the recordings |
| `BOPAlgo_PaveFiller::FilterPavesOnCurves` and `MakeBlocks` (PaveFiller_6) | Write the saved tolerances of the vertices of `theMVTol` straight into their TShapes (`BRep_TVertex::Tolerance`) | As above: the vertices of site 1 and the vertices of used paves, argument ones included |
| `BOPAlgo_PaveFiller::CorrectToleranceOfSE` (PaveFiller_6), step 2.2 | Lowers the tolerance of vertices connected to section edges whose tolerance was reduced, whether or not the vertex is new | In the non-destructive mode an argument vertex can be changed (lowered) in place; the port throws on a frozen one. Not reached by the recordings |
| `BOPAlgo_MPC::Perform` / file-static `UpdateVertices` (PaveFiller_7) | Raises the tolerances of the vertices of section edges in place, from parallel solvers, on vertices several section edges share; `BRep_TVertex::UpdateTolerance` is an unsynchronised read-compare-write | Natively a larger tolerance can be lost to a concurrent smaller one, so the parallel result can differ from the serial one. **Fixed** (decision 789): the port takes a lock around the update, which gives the serial result. In the non-destructive mode the vertex can be an argument's whose tolerance didn't grow before (the port throws on a frozen one, caught as BOPAlgo_AlertBuildingPCurveFailed) |
| `BOPAlgo_PaveFiller::PutClosingPaveOnCurve` (PaveFiller_6) | Sets the index of a local copy of the pave (`aPave.SetIndex(nVn)`) that is never used again | No effect. Kept, not marked |
| `BOPAlgo_PaveFiller::PerformVF` (PaveFiller_4) | Adds to the face's VerticesIn only the last same-domain vertex `nVx` of the loop over a pair's vertices | Possibly intended. Kept, not marked (matches native in every recorded case) |

**Order sites** (decisions 580, 633, 784). The filler iterates integer maps (the faces' sub-shape and vertex sets,
`ShapesSD`, MakeBlocks' maps of vertices and their tolerances, the same-domain maps of MakeSDVerticesFF, PostTreatFF,
UpdateFaceInfo and UpdatePaveBlocks, ProcessExistingPaveBlocks' `aDMBV`) in OCCT's bucket order, which decides the
indices of new shapes; the port reproduces it with `NCollection_IntMap` and `NCollection_IntDataMap` (decision 784). It
iterates no map keyed by address, so native's filler is reproducible.
## Phase 5: unit 5O1, faces from wires

Found while porting BRepAlgo and the TopOpeBRepBuild/DS/Tool members `TopOpeBRepBuild_WireToFace::MakeFaces` runs.
Each is reproduced in the port and marked `// OCCT` unless the table says otherwise.

| Where | What | Effect |
|---|---|---|
| `BRepAlgo_FaceRestrictor::Perform`, `ChangePCurve` | Returns `C2.IsNull()` (true when the edge has *no* pcurve), which the caller stores as `YaPCurve` and tests `if (!YaPCurve)` | Without projection, an edge without a pcurve on the surface but with one elsewhere gets that pcurve moved onto the surface, then its 3D curve projected as well; without a 3D curve Perform returns not done, having written the pcurve into the caller's edge. Reproduced by the recordings (scenario 14: done 0, no face) |
| `BRepAlgo_FaceRestrictor::PerformWithCorrection`, `IsInside` | Evaluates the first edge's pcurve on the face without checking it | Native crashes on an edge without a pcurve on the face (and not on a plane with a 3D curve); the port throws NullReferenceException |
| `BRepAlgo_FaceRestrictor` | The default constructor leaves `myDone`, `modeProj` and `myCorrection` uninitialised | `IsDone` before `Perform` reads garbage; the port's are false |
| `TopOpeBRepTool_2d.cxx`, `FC2D_FancestorE` | Without a 3D curve, looks the edge's face up in a map only `FC2D_Prepare` (the Booleans) creates, dereferencing a null pointer otherwise | Native crashes when WireToFace meets an edge with neither a pcurve on the face nor a 3D curve; the port throws Standard_NullObject (decision 792) |
| `TopOpeBRepTool_2d.cxx`, the pcurve cache | Kept in `thread_local` statics that survive between calls; `FC2D_FancestorE` allocates the cache map before crashing | In a native process that ran a legacy Boolean on the thread, WireToFace may find pcurves cached by it. The port never has the cache (decision 792) |
| `TopOpeBRepTool_ShapeClassifier` | The default constructor leaves `mySameDomain`, `mymren`, `mymredone`, `myState` and `myPAvLS` uninitialised; the shared classifier (`FSC_GetPSC`) is made with it | Its callers set them before reading; the port's are zero, false and null |
| `TopOpeBRepBuild_WireEdgeSet::MakeNeighboursList` | `VertexConnectsEdges` leaves its orientations unwritten when the vertex isn't shared, and the caller reads `oVE` | Undefined on closing edges at a vertex they don't share; the port reads FORWARD |
| `TopOpeBRepBuild_Merge.cxx`, `CorrectUnclosedWire` | Removes a wire's children while a TopoDS_Iterator is on them (the iterator's node is freed) | Undefined; in practice the next child is visited. The port iterates a snapshot. Only new wires (blocks of edges) get there, never WireToFace's |
| `TopOpeBRepBuild_Builder::MakeFaces` | `myBuildTool.Closed(newWire, true)` on old wires too, and `AddFaceWire` | WireToFace flags the caller's wires closed (and not free). The port works on copies of frozen wires and gives the caller's back where nothing changed (decision 793) |
| `BRepAlgo_Image::Image` | For a shape without images, appends it to a function-level `static` list and returns the list | The list keeps every shape so asked for: the second call returns two shapes. Shared between threads natively (racy); per thread in the port |
| `BRepAlgo_AsDes::Ascendant`, `Descendant`, `ChangeDescendant` | Return a function-level `static` empty list for an unknown shape; `ChangeDescendant` hands it out writable | A caller filling it changes what every later call returns for unknown shapes. Per thread in the port |
## Phase 5: unit 5K2, geometric constraints

Found while porting Geom2dGcc. Each is ported as OCCT has it and marked `// OCCT` at the line (58 sites). "Unset"
means an NCollection_Array1 element or a local OCCT never writes: uninitialised natively, 0 in the port (decision
781). The oracle (decision 802) masks or attributes every one whose native value depends on memory.

| Where | What | Effect |
|---|---|---|
| `Geom2dGcc_FunctionTanCuCuCu.cxx` | `Derivatives` and `Values` compute nothing and return true | Circ2d3TanIter solves with math_FunctionSetRoot, which calls `Values`: the search starts from the solver's own F and DF, uninitialised natively (stack and heap bytes), zeros in the port, which stops at the starting point. Native's results on free-form Circ2d3Tan (curve, curve, point) vary from run to run: 323 of 2,000 oracle cases differ, none repeating on a second native run |
| `Geom2dGcc_FunctionTanCuCuOnCu.cxx` | The (circle, curve, centre on a circle) constructor sets `Geom2dGcc_CuCuOnCi`, not `CiCuOnCi` | `InitDerivative` evaluates the curveless `Curv1`: a null dereference natively, NullReferenceException in the port. Only the public `Geom2dGcc_Circ2d2TanOnIter(QualifiedCirc, QCurve, gp_Circ2d, ...)` reaches it; Geom2dGcc_Circ2d2TanOn's dispatch never does |
| `Geom2dGcc_Circ2d2TanOn.cxx` (two curves, centre on a line or circle) | No `else`: the generic Iter on the OnCurve adaptor always runs after the line or circle Iter and replaces its result; the circle branch never calls WhichQualifier | The specialised Iter's work is discarded |
| `Geom2dGcc_Circ2d2TanOn.cxx` (every Iter branch) | CenterOn3 isn't copied | `parcen3(1)` unset, `pntcen(1)` the origin (the oracle masks CenterOn3's parameter on these paths) |
| `Geom2dGcc_Circ2d2TanOn.cxx` (curve, point, centre on a line or circle) | `TheSame2(1)` not set | Read by Tangency2 and IsTheSame2; unreachable in practice (the Iter throws, below) |
| `Geom2dGcc_Circ2d2TanOnIter.cxx` (curve, point, on a line, circle or curve) | math_FunctionSetRoot is given 3-vectors for the 4-variable FunctionTanCuCuOnCu | Every `Geom2dGcc_Circ2d2TanOn(free-form curve, point, ...)` throws. Natively Standard_DimensionError: math_FunctionSetRoot.cxx defines `No_Standard_OutOfRange`, so SetTolerance reads past the vector unchecked and Perform rejects the length. In the port Standard_OutOfRange at SetTolerance (decision 803) |
| `Geom2dGcc_Circ2d2TanOnIter.cxx` | Tangency1, Tangency2 and CenterOn3 return 0 for ParSol, ParArg and the centre's parameter instead of the stored values; IsTheSame2 always false | Through Geom2dGcc_Circ2d2TanOn, every Iter parameter is 0 |
| `Geom2dGcc_Circ2d2TanOnIter.cxx` (circle, curve, line) | The centre is `ElCLib::Value(Ufirst(1), OnLine)`, Ufirst(1) being the parameter on C1 | Wrong centre (public constructor only) |
| `Geom2dGcc_Circ2d2TanOnIter.cxx` (curve, curve, circle or curve) | `par1sol = 0`, `par2sol` the distance between the tangency points | Hidden by the accessors returning 0 |
| `Geom2dGcc_Circ2d2TanOnIter.cxx` (point constructors) | `parcen3` is the radius `Ufirst(3)`; the line case stores the centre's line parameter as `pararg2` | Moot: these constructors throw |
| `Geom2dGcc_Circ2d2TanOnGeo.cxx` (two circles, centre on a curve) | For an unqualified first circle `Rbid(1)` is overwritten and `Rbid(2)` never set; `RBid(2)` set only for an unqualified second circle; both read whenever nbsol is 2 | Native's radii depend on memory: 11 of 2,000 oracle cases differ (9 repeating natively) |
| `Geom2dGcc_Circ2d2TanOnGeo.cxx` | `Radius(nnsol)` reaches 4 in an array of 2; the qualifier tests use `Radius(i)` with the bisector's index | Standard_OutOfRange, natively too |
| `Geom2dGcc_Circ2d2TanRad.cxx`, `Results(GccAna_Circ2d2TanRad)` | Asks for Tangency1 and Tangency2 even when IsTheSame, where GccAna's accessors throw StdFail_NotDone | The constructor throws for any analytic case whose solution is an argument circle; with GccAna's unset `TheSame2(1)` (decision 781) it may throw natively only. No oracle case did |
| `Geom2dGcc_Circ2d2TanRadGeo.cxx` (curve, curve) | `qualifier1(NbrSol)` assigned twice, `qualifier2` never | WhichQualifier's second qualifier unset when neither argument is a line or circle (not bound natively, so not compared) |
| `Geom2dGcc_Circ2d3TanIter.cxx` | `IsTheSame3` always returns true once done; `IsTheSame2` reads TheSame3 | Every iterative Circ2d3Tan solution reports IsTheSame3 and its Tangency3 throws StdFail_NotDone (cirtang and callers asking for the third tangency of a free-form case get the exception natively too) |
| `Geom2dGcc_Circ2d3TanIter.cxx` (curve, curve, point), (circle, circle, curve) | The second qualifier test checks Qualified1 again | Wrong qualifier filtering |
| `Geom2dGcc_Circ2d3TanIter.cxx` (line, line, curve), (line, curve, point), (circle, line, curve) | The side of a line is `centre · normal` from the origin, not from a point of the line | Wrong qualifier filtering unless the line passes through the origin |
| `Geom2dGcc_Circ2d3TanIter.cxx` (line, curve, point), (circle, curve, point) | Cu2 is evaluated at `Ufirst(2)` (the line's or the point-circle's parameter) instead of `Ufirst(3)`; the point-circle's tangent at `Ufirst(3)` | Wrong tangency checks (public constructors only) |
| `Geom2dGcc_Circ2dTanCenGeo.cxx` | The `Index` array is never written, and `Index(1) == Index(2)` decides between one solution (the nearest extremum) and two (the farthest too) | Native has one or two solutions depending on memory (the port always one): 387 of 2,000 oracle cases differ, 251 repeating natively, native always the port's solutions plus others |
| `Geom2dGcc_Circ2dTanOnRadGeo.cxx` (all six constructors) | `par1sol` is `ElCLib::Parameter(cirsol(n), pnttg1sol(n))` before `pnttg1sol(n)` is set | The parameter of the default point (0, 0); Standard_ConstructionError when the solution is centred at the origin |
| `Geom2dGcc_Circ2dTanOnRadGeo.cxx` | `CenterOn3` returns `pnttg1sol`, not `pntcen3` | As GccAna_Circ2dTanOnRad's: a tangency point for the centre |
| `Geom2dGcc_Circ2dTanOnRadGeo.cxx` (curve, centre on a curve) | `Adaptor2d_OffsetCurve(HOnCurv)` without an offset has the range [0, 0] | The curve to centre on is one point: no solution |
| `Geom2dGcc_Circ2dTanOnRadGeo.cxx` (line, centre on a curve) | The offset line goes through `dir1.XY() + cote * norm1`, the direction, not the origin; `pararg1` is a parameter on that line evaluated on L1 | Wrong solutions unless the line passes through its direction vector |
| `Geom2dGcc_Lin2dTanObl.cxx` (curve, line, param, angle) | Passes TolAng to the Iter as the start parameter and Param1 as the angular tolerance | Draw's `lintan` with an angle starts from TolAng |
| `Geom2dGcc_Lin2dTanObl.cxx` (free-form curve) | `Add` calls the Iter's Intersection2, which throws Geom2dGcc_IsParallel when the angle is 0 or π | The constructor throws |
| `Geom2dGcc_Lin2dTanObl.hxx` | `IsParallel2` declared, not defined (build123d.NET's binding excludes it) | Left out with its write-only `Paral2` |

## Phase 5: unit 5L1, laws and trihedrons

Found while porting Law and GeomFill's trihedron and location laws. Each is ported as OCCT has it, except the array
bounds of Law_BSpline (decision 808).

| Where | What | Effect |
|---|---|---|
| `Law_BSpline::IncreaseMultiplicity` (both overloads), `IncrementMultiplicity` | Call `InsertKnots` with the default `Add = false`; Geom_BSplineCurve passes true | The multiplicity becomes Max(old, M - old) instead of M (or Max(old, Step) instead of old + Step), so `SetKnot(Index, K, M)` misses M. Kept, pinned by a test |
| `Law_BSpline::Segment` | Locates the end at `NewU1 + (U2 - U1)`, which can round one ulp below the knot just inserted at U2; the strict `< Eps` test misses it | The segment ends at the previous knot and keeps poles of a span whose end knot is not of multiplicity Degree: a wrong curve, in a few percent of random segments. Kept, pinned by a test |
| `Law_BSpline` constructors and `SetPeriodic` | OCCT 8's `NCollection_Array1::operator=` copies the source's bounds into the (1, n) arrays they allocate | Arrays given with other bounds, and the knots after `SetNotPeriodic` then `SetPeriodic` with FirstUKnotIndex > 1, keep their bounds; later 1-based reads are out of bounds (undefined behaviour). **The port copies into 1-based arrays (decision 808)** |
| `Law_BSpline` (`Rational(W)`), `Law_Interpolate::BuildParameters` | Read W from index 1, and index the parameters by the points' index, whatever the arrays' lower bounds | Wrong only for arrays not indexed from 1. Kept |
| `GeomFill_CorrectedFrenet::Copy` | Copies with the default constructor | The copy of a law made `ForEvaluation` searches the mean plane again (it is not "for evaluation"). Kept |
| `GeomFill_CorrectedFrenet.cxx`, `smoothlaw` | After a successful first pass `Law = BS` shares the handle, and the second pass removes more knots from BS | The returned law is the second pass's even when that pass's control fails. Kept |
| `GeomFill_CorrectedFrenet::InitInterval` | The next step is `max(|S(t)| / 2, LengthMin) / |S'(t)|` with S = C' ^ C'' (GeomFill_SnglrFunc), capped by the default step but not bounded below | On offset curves the steps fall near 1e-6, so one SetCurve takes hundreds of thousands of Frenet evaluations: seconds to over ten minutes natively on random offsets of B-splines and Bezier curves (the port, in a Debug build, about 30 times longer). Kept; the oracle leaves those spines out (decision 811) |
| `GeomFill_CorrectedFrenet::Intervals` | No `return` after `EvolAroundT->Intervals(T, S)` when Frenet has one interval | The fusion that follows overwrites T with the same values. Harmless. Kept |
| `GeomFill_GuideTrihedronAC::D2` | Uses `TN2 = |TN|^2` (= 1) where the second derivative of a unit vector needs `|DTN|^2` | D2BiNormal, and with it D2Tangent, are off by (|DTN|^2 - 1) BiNormal. Kept, pinned by a test |
| `GeomFill_GuideTrihedronAC::D0/D1` | No guard against zero magnitudes (OCCT's own TODO "finding #8") | Division by zero where the path meets the guide. Kept |
| `GeomFill_DiscreteTrihedron::Init` | Appends to myKnots and myTrihedrons without clearing them | A second SetCurve on a free-form curve keeps the first curve's samples. Kept |
| `GeomFill_GuideTrihedronPlan` | Declares its own `myTrimmed`, hiding the base's, so the inherited `GetInterval` reads a null handle; `D2` computes only the path's Frenet frame and returns false | Access violation natively, NullReferenceException in the port; no second derivatives. Kept |
| `GeomFill_LocationGuide::Copy` | Runs `Set(..., rotation, ...)` before the copy has a path; passes `myPoles2d(1,1).X()` (the first path parameter) as the start angle, as SetCurve does | Copying a law with a rotation throws Standard_ConstructionError. Kept, pinned by a test |
| `GeomFill_LocationGuide::D2` | Multiplies the caller's incoming matrices by the transformation before setting them from the trihedron | D2 never applies SetTrsf, while D0 and D1 do. Kept, pinned by a test |
| `GeomFill_LocationGuide::GetAverageLaw` | Averages myNbPts + 1 points with a step of a tenth of the interval | Most of the 22 points lie past the interval's end. Kept, pinned by a test |
| `GeomFill_LocationGuide::SetRotation` | After the "rescue" solve `Pc` is still the default point | w = 0 is stored. By reading |
| `GeomFill_FunctionDraft::DerivT` | Omits the w cos(a) B' term | LocationDraft's D1/D2 trace derivatives are wrong where the binormal turns (about 1.6 times the finite difference along a circle; exact along a line). Kept, pinned by a test |
| `GeomFill_LocationDraft`, the seed search of D0/D1/D2 | For a parameter before the first milestone reads the poles at indices -1 and 0 | Out of bounds natively; Standard_OutOfRange in the port |
| `GeomFill_CurveAndTrihedron`, `LocationDraft`, `LocationGuide::GetAverageLaw` | The matrix's columns are (T, N, B), where D0 to D2 give (N, B, T) | The average law is not the average of M(t). Kept, pinned by a test (CurveAndTrihedron) |
| `GeomFill_UniformSection::BarycentreOfSurf` | Starts at the section's first parameter but steps to the converted B-spline's last parameter | A wrong sample range when the two differ. By reading |
| `GeomFill_LocFunction::DN` | Any order other than 0 to 2 dereferences a null pointer | Access violation natively; Standard_OutOfRange in the port (tested) |
## Phase 5: unit 5M, medial axis

Found while porting Bisector, MAT, MAT2d and BRepMAT2d. Each is ported as OCCT has it and marked `// OCCT` at the
line; the native recordings (tools/bisrec, tools/matrec, decision 823) agree with the port wherever they reach.
"Unset" means a local or field OCCT never writes on that path: uninitialised natively, 0 or false in the port.

| Where | What | Effect |
|---|---|---|
| `Bisector_Inter::Perform` | Each piece is stored in SBis/SD at its interval index, but the loop runs from 1 to the piece count | When an interval outside the domain is skipped, it reads an unset slot (a null handle) and misses the last piece |
| `Bisector_Inter::Perform` | The second bisector's end extension is tested with `IB2 == Bis1->NbIntervals()` (Bis2 meant) | Wrong extension test when the bisectors' interval counts differ |
| `Bisector_Inter::NeighbourPerform` | UMax and UMin come from `BisecCC::ValueAndDist`, which can leave them unset | Unset bounds (0 in the port) |
| `Bisector_BisecPC(Cu, P, Side, UMin, UMax)` | Leaves shiftParameter, distMax, isEmpty and currentInterval uninitialised, then evaluates `Value(UMin)` with the unset shiftParameter | Uses 0 and false in the port |
| `Bisector_BisecPC::Values` | No case for N = 3 | EvalD3 returns the given point rather than the bisector's point, with zero derivatives |
| `Bisector_BisecPC::ComputeIntervals` | The periodic branch is dead (one interval appended at that point) and would call `Period()`, which throws for a non-periodic bisector | None reachable |
| `Bisector_BisecCC::Values` | No `return` after the extension when the polygon has at most one point; it then compares with `First()` of an empty polygon | Compares with a default point of parameter 0 |
| `Bisector_BisecCC` (TestExtension) | On the branch testing the curve's last point, T1 isn't normalised before `T1·T2 > 1 - Confusion` | The tangency test depends on T1's length |
| `Bisector_BisecAna::Perform` | Can leave `thebisector` unset: the circle/circle, circle/line and circle/point cases when no solution is kept, the point/point case when `!ok && oncurve` | Bisector_Bisec dereferences it: a native crash, a NullReferenceException in the port |
| `Bisector_BisecCC::ValueAndDist`, `Extension`, `PointByInt` | Can leave U1, U2, Distance (UOnB, Dist) unwritten (reached from SupLastParameter) | Unset outputs (`ref` in the port, decision 817) |
| `Bisector_BisecCC::ChangeGuide` | Shares curve1 and curve2 with the original (not a bug) | Transforming one changes the other |
| `MAT2d_Mat2d::CreateMat` | A face bounded by one closed edge (a full circle): the main loop never runs and the roots come from the empty list of a bisector never built; `roots->Current()` is a null handle | Native: access violation in `BRepMAT2d_BisectingLocus::Compute`. Port: NullReferenceException (marked in `MAT2d_Mat2d.Roots`) |
| `MAT2d_Tool2d::BisecFusion` | Assumes a bisector that isn't curve/curve is analytic: a Bisector_BisecPC gives a null `down_cast`, then `BAna->SetTrim` | Native crash, NullReferenceException in the port; reached when contours are merged, e.g. a holed face with Bézier sides (a matrec preset) |
| `BRepMAT2d_BisectingLocus::RenumerationAndFusion` | Reads `MergeArc1`, `MergeArc2` and `GeomIndexArc1..4`, which `MAT_Graph::FusionOfBasicElts` leaves unset when the two elements are one, and `MergeArc2` unset when the start and end arcs are one | Uninitialised or stale values; the port starts them at false/0 and keeps OCCT's reuse of stale values |
| `MAT_Bisector`, `MAT2d_Mat2d` | MAT_Bisector's index, issue and end points, vectors, sense and distance, and Mat2d's `intersectionpoint`, uninitialised until set | Harmless (always set before read on the recorded paths); 0 in the port |
| `MAT2d_CutCurve` | `Perform(C, aSide)` and `PerformInf` declared, never defined; `thenumberofedges` never read | None |

## Phase 6: unit 6F, builders and Booleans

Found while porting the builders, by reading OCCT's code. The port reproduces each (`// OCCT` at the line). None shows
in the oracle (decision 835).

| Where | What | Effect |
|---|---|---|
| `BOPAlgo_BOP::BuildSolid` | Scans the face-to-solids map with `for (i = 1; i < aNb; ++i)`: the last face is never checked for being shared by two argument solids | A solid whose only face shared with another is that last one is wrongly taken as possibly untouched |
| `BOPAlgo_Builder::BuildDraftFace` (Builder_2) | Changes the orientation of the edge splits stored in `myImages` in place (`ChangeValue` through a const reference) | Later readers of the images see each edge split's last orientation |
| `BOPAlgo_Builder::FillInternalVertices` (Builder_2) | Adds an internal vertex into a face split, which can be another argument's own face kept as its same-domain group's face | In the non-destructive mode an argument face can be changed in place. On a frozen argument the port throws (BOPAlgo_AlertBuilderFailed, decision 829). Not reached by the oracle |
| `BOPAlgo_Builder::FillInternalShapes` (Builder_3) | Adds internal vertices and edges into a split solid, which can be an argument's solid kept as the same-domain solid | As above, for solids |
| `BOPAlgo_Tools::WiresToFaces` | Adds p-curves and corrects tolerances on the input's edges | The input changes; the port copies a frozen input at the entry (decision 830) |
| `BOPAlgo_ToolsProvider::SetTools` | Clears `myTools` but not the fence `myMapTools` (as 6A's SetArguments) | Setting again tools added before leaves none |

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

| Where | What | Effect |
|---|---|---|
| `GeomFill_Pipe::Init(Path, FirstSect, LastSect)` | Treats `myLoc->SetCurve(myAdpPath)`'s false as a failure, but CorrectedFrenet's SetCurve returns whether the plain Frenet frame needs no correction | Pipes evolving between two sections fail with GeomFill_ImpossibleContact on most 3D paths and on planar paths with inflections (27 of 30 random 3D B-spline paths; `tuyau r path s1 s2`). Kept, pinned by a test and the recordings |
| `GeomFill_SweepSectionGenerator::Perform` (types 1 to 3) | `TR = Rot * Trans` sits in the branch for opposite tangents, so TR stays the identity otherwise | The rotations and translations between successive sections are dropped: every section of a circle, constant or evolving section along a non-circular path stays at the first one's place. Kept (GeomFill_Pipe uses only type 0) |
| `GeomFill_SweepSectionGenerator::GetShape` (type 0) | Gives the quasi-angular shape (7 poles, degree 6) whatever `Perform(Polynomial)` asked; `Section` then assigns the polynomial conversion's 8 poles | `GeomFill_Pipe::Perform(WithParameters, Polynomial = true)` on a pipe on an edge throws Standard_OutOfRange natively and in the port. Kept, pinned |
| `GeomFill_QuasiAngularConvertor::Section` (second derivatives) | `D2Vw(5)` has `(b + 11)` where the derivative of `beta^4 (2c + b^2)` needs `(b + 1)` | The weights' second derivative, and with it D2Poles, are wrong in the quasi-angular conversion (D2 of circular blends). Kept |
| `GeomFill::GetCircle` (second derivatives, rational branch) | `d2np2 = d2nplan ^ ns1 + nplan ^ dn2w`, where the second derivative of `nplan ^ ns1` needs `nplan ^ d2n1w` | Wrong second derivatives of the interior poles of arcs of more than one span. Kept |
| `GeomFill::GetCircle` (rational branch) | Builds the interior poles as `pts1 + R ((cos a - 1) ns1 + sin a (nplan ^ ns1))`, right for normals from the centre to the ends, while the header calls them inward normals ("Normal rentrente") and CircularBlendFunc passes inward ones | With inward normals, arcs of two or three spans (TgtThetaOver2_2/_3) get interior poles off the circle (radius 3.49 for 2 at 4.5 radians). CircularBlendFunc never asks for more than one span (it switches to quasi-angular above 0.65 pi). Kept; the test uses outward normals |
| `GeomFill_NSections::ComputeSurface` | `SetKnots` on the caller's B-spline sections | Reparametrises the caller's curves in place. **The port reparametrises copies (decision 838)** |
| `GeomFill_NSections::BarycentreOfSurf` | U is not reset for each V row | Only the first row samples the u range; the others evaluate past it. Kept |
| `GeomFill_EvolvedSection::BarycentreOfSurf` | As UniformSection's: starts at the section's first parameter, steps to the converted B-spline's last | A wrong sample range when the two differ. By reading |
| `GeomFill_Sweep::Build` | The call to `BuildProduct` (the approximation of the location law composed with the section's surface) is commented out, and BuildProduct's tensorial product too | `GeomFill_Location` approximates globally, as `GeomFill_Section` does. Kept |

## Phase 5: unit 5L3, surfaces from curves

Found while porting GeomFill's filling half and AppBlend. Each is ported as OCCT has it, except the reversal of the
caller's curve in BSplineCurves and BezierCurves (decision 846). None was found in GordonUtilities, NetworkSurface or
Gordon.

| Where | What | Effect |
|---|---|---|
| `GeomFill_Coons::Init`, rational form | The poles put P2 in column 1 and P4 in column NU, the weights W4 in column 1 and W2 in column NU | Through BSplineCurves/BezierCurves, which pass (P1, P4, P3, P2, W1, W4, W3, W2), rational Coons fillings get the left and right boundary weights swapped. Kept, pinned by a test |
| `GeomFill_CoonsAlgPatch::Value` | Evaluates bounds 0 and 2 at V instead of U | A wrong point; `D1U`, `D1V` and `DUV` are right, and ConstrainedFilling uses only those. Kept, pinned by a test |
| `GeomFill_BSplineCurves::Init`, `GeomFill_BezierCurves::Init` (two curves, CurvedStyle) | When CC1's end is CC2's start, calls `C1->Reverse()` on the caller's curve, not on its copy | The result is built from the unreversed copy, so it is right, but the caller's curve is reversed (a second call sees it so). The port leaves the caller's curve alone (decision 846) |
| `GeomFill_ConstrainedFilling::MatchKnots` | After SetDomain, the `ind[]` searches compare `dom` with the multiplicities `nm[]->Value(i)` instead of the knots `nk[]` | The knot is not found and the tangency mixing index keeps its default. Kept |
| `GeomFill_ConstrainedFilling` | `stcor`, `tgalg` and `dom` are never reset by Init | A second Init on the same object starts sortbounds from the previous corner gaps and keeps tangency fields of bounds that no longer have normals. Kept |
| `GeomFill_ConstrainedFilling::MatchKnots` | Reads `nbnp`/`nbnk` after `PrepareInsertKnots` returns false | Uninitialised natively; the port's out parameters are assigned |
| `GeomFill_Profiler::UnifyByInsertingAllKnots` | Inserts every curve's knots into the others with `Add = false`; on periodic sections `BSplCLib::PrepareInsertKnots` mispredicts the pole count (Phase 1's entry above) | Natively `BSplCLib::InsertKnots` writes past its arrays: heap corruption, a crash in the recorder; the port throws Standard_RangeError. Reached by every Generator, SectionGenerator and loft whose sections are all periodic B-splines with different knots |
| `GeomFill_Profiler::Perform` | When knot insertion does not unify the curves, `UnifyBySettingMiddleKnots` only moves knots | The curves keep different numbers of poles; `Poles`, GeomFill_Generator and AppBlend_AppSurf then read and write past arrays sized for the first curve natively (unchecked) where the port throws. Seen with all-periodic sections |
| `GeomFill_SectionGenerator::SetParam` | Keeps the caller's array, then writes `myParams(i) = Params(Lower() + i - 1)` into it for i from 1 | For an array not indexed from 1 the caller's values are shifted and the last write is past the end (unchecked natively; Standard_OutOfRange in the port, tested) |
| `GeomFill_SectionGenerator` constructor | Sets default parameters only if `mySequence` has curves, which it never has there | `Parameter` without `SetParam` dereferences a null handle (an access violation natively, NullReferenceException in the port) |
| `AppBlend_AppSurf::InternalPerform`, spline approximation | `if (C0) SetContinuity(0); if (C1) ... else if (C2) ... else SetContinuity(3)` | GeomAbs_C0 ends as continuity 3. Kept (recorded) |
| `AppBlend_AppSurf::InternalPerform`, Bezier pieces (not SpApprox) | Computes the tolerances reached but never stores them | `TolReached` and `TolCurveOnSurf` give the previous run's values (0 at first). Kept |
| `AppBlend_AppSurf` (both Perform) | The two-section rule (IFV occ13904) writes `dmin = 1` and possibly `dmax = 1` into the object's settings | They stay for the next pieces of `Perform(..., NbMaxP)` and for later runs: a piece of two sections lowers every later piece's degrees, and later pieces can fail (StdFail_NotDone). Kept (recorded) |
| `AppBlend_AppSurf::Perform(..., NbMaxP)`, known parameters | Measures each Bezier piece by `mucu.Pole(Index, 1)` to `Pole(Index, NbPoles)`, Index being the piece's number | The distances, hence the interior knots, come from the wrong curve of the multi-curve. Kept |
| `AppBlend_AppSurf::Curves2dShape` | `NbPoles = tabPoles->ColLength()`, the number of U poles | The 2d curves have the V poles. Kept |
| `AppDef_Variational::InitCriterionEstimations` through `AppBlend_AppSurf::PerformSmoothing` | The entry above (EstTangent past a two-point line) | Smoothing two sections: native reads past the points and throws Standard_OutOfRange later; the port's range check throws Standard_RangeError first (decision 849) |
## Phase 5: unit 5R, 2D fillets and chamfers

ChFi2d and BRepFilletAPI_MakeFillet2d, reproduced as OCCT has them (decision 10), each marked `// OCCT`. None made
the port differ from native in the oracle or the recordings (decision 856).

| Where | What | Effect |
|---|---|---|
| `ChFi2d_Builder.cxx`, `BuildNewWire` | Every edge of the face being built goes into one new wire, closed as the face's first wire | A face with holes comes back with one wire holding the outer and inner edges (build123d.NET chamfers each wire on its own for this reason) |
| `BRepFilletAPI_MakeFillet2d.cxx`, `Modified` | `DescendantEdge` reads the history with `Find` | Standard_NoSuchObject for any edge MakeFillet2d didn't trim, where BRepBuilderAPI_MakeShape's contract is an empty list |
| `ChFi2d_Builder.cxx`, `BuildFilletEdge` | `vec1` built from the vertices and overwritten at once; `Ebid1`/`Ebid2` built and unused; the basis curves down-cast to Geom2d_Line unchecked | Dead code; a curve neither line nor circle would be a null dereference (IsLineOrCircle rejects it first) |
| `ChFi2d_Builder_0.cxx`, `ComputePoint(F, L, E, Param)` | `Param` unset and the point at the plane's origin when the chamfer's line meets nothing | An angle chamfer that misses its second edge ends at the origin of the plane with an uninitialised parameter (0 in the port) |
| `ChFi2d_Builder_0.cxx`, `ModifyChamfer(Chamfer, E1, E2, D1, D2)` | `E1` unused | Only `E2` orients the new chamfer |
| `ChFi2d_AnaFilletAlgo.cxx`, `Init(wire)` | `e1` and `e2` are only set when null | A second Init (ChFi2d_FilletAPI reuses its algorithm) keeps the first wire's edges |
| `ChFi2d_AnaFilletAlgo.cxx`, `isCW` | The middle point is evaluated on the edge's curve without the edge's location, the ends with it | Wrong orientation for an arc edge with a location |
| `ChFi2d_AnaFilletAlgo.cxx`, `Perform` | The right neighbour's ends are chosen by `e1.Orientation()` | Wrong trimming when the two edges have different orientations |
| `ChFi2d_AnaFilletAlgo.cxx`, `ArcFilletSegment` | Compares a squared distance difference with `SquareConfusion`, where SegmentFilletArc uses `Confusion` | The arc's end point is found less often; the parameter path is used instead |
| `ChFi2d_FilletAlgo.cxx`, `IsRadiusIntersected` | `LowerDistanceParameter()` (a parameter) compared with `Precision::Confusion()` as if it were a distance | Intersections are judged on the edge by their parameter |
| `ChFi2d_FilletAlgo.cxx`, `FilletPoint::Copy`, `FilterPoints` | `Copy` doesn't copy `myNear`; FilterPoints removes `myV(b)` with `b` an index into `myNear` | Values and their near indices can fall out of step |
## Phase 5: unit 5Q, fair curves

Found while porting FairCurve. Each is ported as OCCT has it and marked `// OCCT` at the line; the native recordings
(tools/fairrec, decision 862) agree with the port bit for bit.

| Where | What | Effect |
|---|---|---|
| `FairCurve_EnergyOfBatten::Compute`, `FairCurve_EnergyOfMVC::Compute` | `MyBattenLaw.SetSliding(MyLengthSliding)` updates the energy's own copy of the law, which nothing reads; the distributions keep their copies with the initial sliding | With free sliding and a non-zero slope, the height law `H(t) = h + (t - 0.5) slope L` keeps the initial L while the sliding changes. Kept |
| `FairCurve_Energy::Hessian1` | Sets the terms between the first end's and the second end's constraint variables to 0 ("correct if there are less than 3 nodes") | With the battens' single-span degree 9 curves every pole interacts, so those hessian terms are wrong (finite differences give 1e-3 to 2e-2); the Newton steps are approximate, the minimum is the same. Kept, pinned by a test |
| `FairCurve_Energy::Value` | Reads `Energie(0)` of a fresh `math_Vector(0, 0)` that Compute leaves unwritten when it stops on an infinite sliding | Uninitialised natively, 0 in the port; math_NewtonMinimum's step halving and its step reduction (which ignores the return value) read it. No case of the recordings differs. Kept |
| `FairCurve_EnergyOfBatten::Compute` | When a sagging integration fails, goes on to the next interval and returns that interval's result | With more than 12 poles, a failure on one interval can be lost. By reading |

## Phase 5: unit 5P, plates and filling

Found while porting Plate, NLPlate, GeomPlate, LocalAnalysis and BRepFill_Filling. Each is ported as OCCT has it,
except the two of decision 869 and the evaluation cache of decision 870, which change nothing native computes
reliably.

| Where | What | Effect |
|---|---|---|
| `NLPlate_NLPlate::Solve`, `Solve2` | After the loop over constraint orders, `OK = true` overwrites the `OK = false` an iteration's failure set | `IsDone()` is true even when no plate was solved. Kept, pinned by a test (three constraints at one UV: every plate singular, the surface still the initial plane) |
| `NLPlate_NLPlate::Solve2` | Solves its first, polynomial iteration with `ord`, not the `ordre` it raises to `maxOrder + 2` | Possibly intended. Kept |
| `Plate_Plate::Init` | Leaves `OK = true` | An empty plate, and a GeomPlate_BuildPlateSurface with no constraint, report `IsDone()`. OCCT's own GTests pin it with a TODO (fixing it broke blend and filling Draw tests). Kept |
| `Plate_GtoCConstraint` (G3 constructor with a direction) | `nP` is unused (its name commented out in the signature) | Equal to the constructor without it. Kept, pinned by a test |
| `Plate_LinearXYZConstraint`, `Plate_LinearScalarConstraint` (from arrays) | OCCT 8's `operator=` copies the source's bounds | A pinpoint array not indexed from 1 makes Plate's 1-based reads go out of range: Standard_OutOfRange on both sides. No caller passes one |
| `LocalAnalysis_CurveContinuity`, `LocalAnalysis_SurfaceContinuity` | `myErrorStatus` is never set on success; the tolerance-only surface constructor leaves `myTypeCont` unset until `ComputeAnalysis` | `StatusError()` after a successful analysis and `ContinuityStatus()` before one read indeterminate memory; the port returns `LocalAnalysis_NullFirstDerivative` and `GeomAbs_C0`. No OCCT caller reads them then |
| `GeomPlate_BuildPlateSurface::Perform` | When a curve can't be projected on the initial surface, approximates `myGeomPlateSurface`, which Perform nulled at its start (ComputeSurfInit sets it only on its non-linear branch) | Access violation natively (the corpus's bug23343, an OCCT TODO); the port throws Standard_NullObject (decision 869) |
| `GeomPlate_BuildPlateSurface::G0Error(Index)`, `G1Error(Index)`, `G2Error(Index)` | Arrays of NbPtsOnCur values, of which EcartContraintesMil writes one per interval of the curve's last discretisation | More intervals (MakeFilling's usual case once Perform raised the points): Standard_OutOfRange on both sides. Fewer: native reads uninitialised memory, and returns values that change from call to call (seen on the oracle's 4- to 7-edge contours, several per test run); the port's arrays are zeroed (decision 869) |
| `GeomPlate_BuildPlateSurface::G0Error()`, `G1Error()`, `G2Error()` | Set only by VerifSurface, which runs only with curve constraints | With point constraints only, native returns uninitialised values; the port 0. BRepFill_Filling always has curves |
| `GeomPlate_MakeApprox` (criterion order other than -1, 0, 1) | No branch approximates | A null surface, errors unset (the port's 0). Kept |
| `GeomPlate_Surface::UPeriod`, `VPeriod` | Return `false` | 0. Harmless (never periodic) |
| `BRepFill_Filling::AddConstraints` (through MakeFilling's `Add(edge, face, order)`) | Passes the GeomAbs_Shape value as the plate's order (0 G0, 1 G1, 2 G2) | GeomAbs_C1 (2) gives a G2 constraint, and GeomAbs_G2 (3) throws "BRepFill : The continuity is not G0 G1 or G2" (Standard_Failure, on both sides: 10 oracle cases). build123d.NET's `MakeSurfacePatch` maps its C2 level to G2, so it always fails natively |
| `BRepFill_Filling::Build` | Adds the bounds it finds for free constraints to `myBoundary`, and keeps `myOldNewMap` from build to build | A second Build rebuilds each boundary edge from the first build's edge (its empty copy keeps the pcurve on the first surface) and keeps the free constraints' edges. Kept; the oracle compares second builds |
| `BRepFill_Filling::G0Error()` and the others before `Build` | Dereference the null builder | Access violation natively; NullReferenceException in the port |
| `tests/bugs/filling/bug23380` | Draw reads `i` and `f` (faces) as free constraints; the case expects 4 edges | Native OCCT 8.0.1, through build123d.NET's binding, gives 3 edges, as the port does (byte-identical face): the case fails in Draw-lite, and natively too unless DRAWEXE reads its words otherwise |
## Phase 6: unit 6P, BRepAlgoAPI and the public API

Found by OCCT's GTests and the Boolean oracle. The port reproduces each, or says where it can't (`// OCCT` or
`// OCCT.NET: frozen` at the line).

| Where | What | Effect |
|---|---|---|
| `BOPAlgo_PaveFiller::MakeBlocks` (PaveFiller_6, "back to previous tolerance values for unused vertices") | Writes each saved tolerance back into the vertex's TShape, also when it is the vertex's own (a vertex only used as an existing pave) and also for an argument's vertex in the non-destructive mode. `UpdateSavedTolerance` can raise a saved value to a section edge's tolerance, and then the write grows an argument vertex in place | The no-op write is skipped in the port (it would throw on a frozen vertex: GTest `bcut_simple/M2`, a cut of a cut result by its own argument, failed with BOPAlgo_AlertIntersectionFailed). A real change of a frozen argument vertex still throws there (not seen; decision 877) |
| `BRepTools_History::Merge` | Appends the composed images from a hashed `NCollection_Map` (`add(aL12, aAdditions[aI])`), so the order of a merged Modified or Generated list is the map's address order natively, insertion order in the port | `SimplifyResult`'s merged history lists the same images as native's, in another order, on 37 of the oracle's 393 simplified Booleans (and on different cases from one native run to the next); the sets are equal |
| `BRepAlgoAPI_BuilderAlgo::SimplifyResult` | Runs UnifySameDomain with `SetSafeInputMode(myNonDestructive)`: in OCCT's default (destructive) mode the unifier writes into the result in place, and the result shares the arguments' untouched sub-shapes, so the caller's arguments change (pcurves added, ranges reset) | Native writes into its arguments; the port's are frozen and the unifier copies at its entry (decision 766), so they never change. Results are the same bytes; the history has decision 643's in-place entries where native has none |
| OCCT's destructive mode (`SetNonDestructive(false)`, Draw's default) | The pave filler grows tolerances and adds pcurves on the arguments' own sub-shapes, which the result then shares | The port's frozen arguments always run non-destructively (decision 577); where native's destructive result differs, native's non-destructive result is the port's (oracle) |

## Phase 6: unit 6G, checks and the other builders

Found by the native recordings of tools/borec and the unit tests. The port reproduces each, or says where it can't
(`// OCCT` or `// OCCT.NET: frozen` at the line).

| Where | What | Effect |
|---|---|---|
| `BOPAlgo_CheckerSI` (through PaveFiller_6's `PutPaveOnCurve` and `MakeBlocks`) | The self-check sets the non-destructive mode, but the filler still raises the tolerance of argument vertices a face/face section curve passes through (`BRep_Builder::UpdateVertex`), and MakeBlocks writes saved values back | Native `bopcheck`/`bopargcheck` can leave their input with raised vertex tolerances (bottle, hammer, CrankArm against a moved copy). The port checks a copy of a frozen argument (decision 884) |
| `BOPAlgo_PaveFiller::FilterPavesOnCurves` (as MakeBlocks, unit 6P) | Writes a vertex tolerance even when it equals the current value, for an argument vertex used as an existing pave (every self-check of a box) | A no-op natively; skipped in the port, where it would throw on a frozen vertex (decision 885) |
| `bopapicheck` (BOPTest_CheckCommands.cxx) | `-op` as the last word reads `a[n]`, a null pointer, through `strcmp` | DRAWEXE crashes; Draw-lite treats the missing operation as unknown |
| `BOPAlgo_MakePeriodic::SetXTrimmed/SetYTrimmed/SetZTrimmed` | `(bool theIsTrimmed, const bool theFirst = 0.0)` declares the period's start as a `bool` | The start can only be 0 or 1; reproduced with a bool parameter |
| `BOPAlgo_MakePeriodic::UpdateTwins` | Iterates `myRepeatedTwins` (shape-keyed: address order natively) and skips a group once any member is done; twins aren't transitive | The twin groups after a repetition depend on the iteration order; native isn't reproducible there. The port iterates in insertion order |
| `BOPAlgo_MakePeriodic` trim and repetition, `BOPAlgo_MakeConnected::MakePeriodic` | The trim's Common and the repetition's gluing run destructively | Natively they change the caller's input, the periodic shape returned before and the connected shape in place (pcurves added to shared edges). The port copies frozen parts first (decision 891) |
| `BOPAlgo_RemoveFeatures::RemoveInternalWires` | Removes INTERNAL wires and edges from the rebuilt solids' faces in place (`aF.Free(true); BRep_Builder::Remove`, through a `const_cast` of the explorer's face) | When the face is an untouched input face shared into the result, native changes the caller's input. The port copies (decision 892) |

## Phase 5: unit 5S1, sweep laws and sections

Found by the native recordings of tools/brepfillrec and tools/cwrec, the oracle and reading. The port reproduces each
(`// OCCT` at the line).

| Where | What | Effect |
|---|---|---|
| `BRepFill_LocationLaw::Abscissa` | When the lengths aren't computed yet it calls `CurvilinearBounds(Index, bid, Length)` and starts from the edge's last bound instead of its first | The first call on a fresh law is too long by the edge's own length (a 4 + 3 polyline: `Abscissa(2, mid)` gives 8.5 the first time, 5.5 afterwards); reproduced natively, pinned by a test |
| `BRepFill_ACRLaw` constructor | `OrigParam` is indexed over all the wire's edges, `ipath` counts only the non-degenerated ones | After a degenerated spine edge each edge gets an earlier edge's origin (found by reading, not reproduced) |
| `BRepFill_LocationLaw::IsG1`, closed-path branch | The `D1` on law 1 isn't assigned to `Ok_D1`, unlike the inner branch | A failed evaluation there goes unseen |
| `BRepFill_CompatibleWires` SearchRoot (from SameNumberByPolarMethod) | Returns the first key of RMap or MapVLV, iterated in TShape-address order, whose list holds the vertex | With looping sections (the first repeated last) native's result varies from run to run (five runs, five outputs: status Done or Failed, cuts moved); the port iterates in binding order and is deterministic |
| `BRepFill_CompatibleWires::SearchOrigin`, `SameNumberByACR` | `newwire.Oriented(TopAbs_FORWARD);` discards its result; `double AllLengthsNull = true` holds a bool | No effect |
| `BRepFill::Face`, `BRepFill::Shell`, `BRepFill_Pipe` | Write pcurves, ranges, flags and tolerances into the caller's edges and profile | Natively the caller's shapes change; the port writes into copies of frozen inputs (decisions 897, 910) |

## Phase 5: unit 5O2, the 2D wire offset

Found by the MakeOffset oracle, tools/evorec and reading. The port reproduces each (`// OCCT` at the line), unless the
last column says otherwise.

| Where | What | Effect |
|---|---|---|
| `BRepFill_OffsetWire::GeneratedShapes` | Merges the lists of `myMap` by iterating `myMapSpine`, a map keyed by shapes and hashed by TShape address | The shapes in a generated list come in an address-dependent order natively; the port iterates in insertion order. MakeOffset's `Generated` held the same shapes in another order in 74 of 1,124 oracle results |
| `BRepFill_OffsetWire` constructors | Leave `myCallGen` uninitialised; only `Perform` sets it, and `GeneratedShapes` reads it | After `PerformWithBiLo` alone native reads garbage; harmless since `myMapSpine` is then empty. The port starts it false |
| `BRepFill_TrimSurfaceTool::IntersectWith` | `SetTrim` on the trimmed bisector it is given, which is the medial axis's own curve | The locus's bisector stays lengthened by the tolerance at both ends for every later call |
| `BRepFill_Evolved` (`ElementaryPerform`, `PrepareProfile`, `Add`, `Transfert`) | Iterates `myMap` and `MapVevo`, keyed by shapes, in TShape-address order | Natively the parallel edges are built, and the result's faces listed, in an address-dependent order; the port uses insertion order. The bindings don't depend on it |
| `BRepOffsetAPI_MakeOffset::BuildDomains` (via BRepFill_OffsetWire and the medial axis) | Bisector_BisecAna can leave its curve unset (5M) and Bisector_Bisec dereferences it, on some arc profiles with intersection joins | An access violation: with OCCT's signal translator (DRAWEXE, build123d.NET) a Standard_Failure caught as "not done"; without it a crash. The port: a NullReferenceException caught as "not done" (decision 917) |
| `BRepOffsetAPI_MakeOffset::BuildDomains` | `double Found = false` holds a bool | No effect |

## Phase 5: unit 5S2, pipe shells

Found by the MakePipeShell oracle, tools/piperec and reading. The port reproduces each (`// OCCT` at the line), unless
the last column says otherwise.

| Where | What | Effect |
|---|---|---|
| MakePipeShell with an auxiliary spine and `BRepFill_Contact` on polyline spines (the site not isolated) | Dereferences a null handle | An access violation natively; Standard_NullObject in the port (4 oracle cases, decision 930) |
| `BRepFill_Pipe::Edge` (through `BRepOffsetAPI_MakePipe::Generated(spine edge, profile vertex)`) | Reads `myEdges`, which is null when the profile is a vertex | An access violation natively (OSD_Exception_ACCESS_VIOLATION under OSD::SetSignal, 18 of tools/piperec's cases); NullReferenceException in the port |
| `BRepFill_PipeShell::Set(bool IsFrenet)` | Both branches set `myTrihedron = GeomFill_IsFrenet`; the corrected Frenet one is labelled Frenet | No effect seen (`myTrihedron` only chooses the continuity for the discrete trihedron and the contact laws) |
| `BRepFill_PipeShell::Prepare` | Clears `WSeq` and `myEdgeNewEdges` but not `myIndOfSec` | After a second Prepare (a section added or removed after a build) the history reads the first build's section indices |
| `BRepFill_PipeShell::BuildHistory` | The loop looking for the section's inde-th edge tests `Degenerated(anEdge)` (the section's edge) instead of `theEdge` | Degenerated edges of the placed wire are counted; no effect on the oracle's sections |
| `BRepFill_PipeShell` constructor | `mySpine.Closed(true)` on the caller's wire when its ends meet | Natively the caller's spine changes; the port closes a new wire of the caller's edges (decision 926) |
| `BRepFill_PipeShell::MakeSolid` | Adds the end faces to the shell `Shape()` already returned | Natively the shape the caller holds changes; the port adds them to a new shell of the same faces when the shell was published (decision 926) |
| `BRepFill_Sweep` BuildFace | `BRepLib_FindSurface` can return the plane of a face already built (the four edges' pcurves lie on it), and BuildFace calls `UReverse` on it | The other face's surface is reversed too, natively. **Fixed** in the port: it reverses a copy (decision 929); the results differ from native's there |
| `BRepFill_Sweep::BuildWire` | With a last bound, stores `FirstShape` (a wire) at column NbPath of the vertex table instead of the last vertex | Unreachable: the pipes give no bounds to a vertex's sweep |
| `BRepFill_Sweep::PerformCorner`, tangent case | Evaluates the next law's frame at its last parameter instead of its first | The corner of a tangent but twisted junction is placed with the next edge's end frame |
| `BRepFill_Sweep::Build` | Substitutes the edges of `myVEdgesModified`, a map keyed by shapes, in TShape-address order; the first substitution of a shared vertex wins | Natively the order can change which new vertex replaces a shared one; the port iterates in insertion order. Not observed in the oracle |
| `BRepFill_TrimShellCorner::HasSection` | `myHasSection` is set nowhere | Always false; the sweep's fallback for a failed right corner depends only on the transition |
| `BRepOffsetAPI_MiddlePath::Build` | Writes "Interpolation failed" to `std::cout` when GeomAPI_Interpolate fails, then calls `Curve()` | StdFail_NotDone on both sides; the port doesn't print |

## Phase 5: unit 5T, lofts and projection

Found by the ThruSections and BRepProj_Projection oracles and reading. The port reproduces each (`// OCCT` at the
line), unless the last column says otherwise.

| Where | What | Effect |
|---|---|---|
| `BRepFill_Generator::Perform` | `anOrEdge1`/`anOrEdge2` are references to the wire explorers' current edge; after `ex1.Next()` they name the next edge (a null edge after the last), and `myMap` is keyed by it | `GeneratedShapes(E)` gives the face built on the edge before E; the last face is keyed by a null edge. ThruSections doesn't read this map. Reproduced (decision 936) |
| `BRepOffsetAPI_ThruSections::Init` | Sets the maximal degree to 6 where the constructor sets 8 | A loft re-initialised approximates with a lower degree. Reproduced |
| `BRepOffsetAPI_ThruSections::SetCriteriumWeight` | A negative weight sets the status to Failed; `Build` resets the status | No effect but a transient status. Reproduced |
| `BRepOffsetAPI_ThruSections` (mutable input, the default) and `BRepFill_Generator` | Write pcurves, ranges, flags and tolerances into the caller's section edges and vertices | Natively the caller's sections change. The port copies frozen sections at Build and gives back the unchanged sub-shapes (decisions 936, 937) |
| `BRepFill_CompatibleWires` through ThruSections | Maps keyed by shapes iterated in TShape-address order | Native's status and result vary from run to run on lofts whose first section is also the last, and rarely otherwise (1 of 630 random cases); the port is deterministic (decision 940) |
| `BRepProj_Projection::BuildSection` | A compound target's faces go into a new, unlocked compound, so the section runs destructively and writes pcurves into the caller's target edges | Natively the target changes. On frozen targets the port's section ran destructively into frozen edges and failed (IntersectionFailed); it now runs non-destructively when the target holds frozen shapes (`// OCCT.NET`, decision 939) |

## Phases 5 and 6: integration

Found by full Oracle runs (test classes in parallel).

| Where | What | Effect |
|---|---|---|
| `FEmTool_LinearFlexion`, `FEmTool_LinearJerk` and `FEmTool_LinearTension` constructors (FEmTool_Linear*.cxx) | Keep the reference integrals of the last constraint order in function statics `Order` and `MatrixElemts`, unsynchronised; `Order` is set before `MatrixElemts` is recomputed | Concurrent AppDef_Variational runs with different continuities (GeomAPI_PointsToBSpline and PointsToBSplineSurface with weights, ThruSections with smoothing) read the other order's integrals: 2 of 32 C2 approximations run beside C1 ones gave other poles (1e-4 relative), one of them the value a full Oracle run failed on. The port caches per order under a lock (decision 148); the oracle serialises these native calls (decision 945) |
## Phase 6: unit 6M, build123d.NET's Booleans

Found by replaying build123d.NET's own tests' operations. The Booleans and cleans showed no new issue: all 497 calls
are byte-identical with native, and native repeated itself on every one.

| Where | What | Effect |
|---|---|---|
| `BRepProj_Projection::BuildSection` (through build123d.NET's `Project` and `ProjectToShape`) | Never sets the section non-destructive, so any unlocked target, not only the compound of 5T's entry, is sectioned destructively | Natively the caller's target changes: in build123d.NET's ProjectEdge case a box edge on the projection cylinder's seam had its tolerance raised (1e-7 to 1.00000000666e-07) and a pcurve added. The port's frozen targets keep the section non-destructive (decision 939), so its split edge is a copy whose vertices come in the other order: canonically identical, byte-identical on unfrozen operands (decision 953) |
## Phase 6: unit 6L, Draw-lite Booleans and healing

Found by porting BOPTest's and SWDRAW's commands and running the corpus. Draw-lite reproduces each (`// OCCT` at the
line), unless the last column says otherwise.

| Where | What | Effect |
|---|---|---|
| BOPTest_TolerCommands `ReduceFaceTolerance` (`btolx`) | The edge map isn't cleared between faces | A face's tolerance is the smallest of its own edges' and every earlier face's edges' |
| BOPTest_TolerCommands `breducetolerance` | The usage message says `bupdatetolerance` | Text only |
| BOPTest_DebugCommands `bopfsd` | Adds the face itself to the compound, not its same-domain face | `f_sd` holds copies of `f`; the count is right |
| BOPTest_DebugCommands `bopindex` | Index 0 (the first argument) counts as "not found" (`ind > 0`) | " DS does not contain the shape" for the first argument |
| BOPTest_DebugCommands `bopwho`, `bopsd`, `bopfon`/`bopfin`/`bopfsc`, `bopfav` | The range check `ind > NbShapes()` (or `NbSourceShapes()`) lets the one-past-the-end index through | Natively reads past the table; in Draw-lite the index check of the port's table throws, and the case logs the exception |
| SWDRAW_ShapeAnalysis `fbclose` | Reads `a[3]` for splitclosed and `a[4]` for splitopen (off by one) | The options shift |
| SWDRAW_ShapeAnalysis `statshape` | `strstr("bigspl", arg3)`: the option matches as a substring of the keyword | An empty or one-letter word turns modes on |
| SWDRAW_ShapeAnalysis `fbprops` | Labels the open wires "closed wires"; its usage message says `freeprops` | Text only |
| SWDRAW `settolerance` (argc equal to premarg), `samerange` (prints `argv[2]` with 2 words), `fastsewing -tol` without a value | Read past `argv` | Natively undefined; Draw-lite reads an empty word |
| SWDRAW_ShapeFix `fixshape +?` | Lists its message counts in NCollection_DataMap hash order | Draw-lite lists them in first-seen order |
| SWDRAW_ShapeAnalysis `fbprops` on a 10×10 square | `1-16k` comes out marginally negative, so ratio and width print 0 (the formula gives 1 and 10) | Reproduced; not checked against native |
| bugs/heal/bug29382_3 (UnifySameDomain with `+b` on a Bézier prism's common with a box) | The script expects 5 vertices | The port and native OCCT (build123d.NET's build, on the port's input and on its own Boolean) both give 6; the case fails on both |
## Phase 5: unit 5W, Draw-lite construction

Found by reading Draw's command sources while porting them. Draw-lite reproduces each, unless the last column says
otherwise.

| Where | What | Effect |
|---|---|---|
| `qcircle`, `qline` (GeometryTest_CurveTanCommands.cxx) | The help gives the qualifier as `-enclosing`, `-enclosed`...; `GccEnt::PositionFromString` matches only the names without the dash | A qualifier written as the help shows is ignored: the curve is unqualified. Reproduced |
| `geompipe` (BRepTest_SweepCommands.cxx) | The first edge (`spineedge`) is used as the guide and the second (`profileedge`) as the path; the circle's frame is taken on the path at the guide's first parameter (`ProfileCurve->D1(aSpFirst, ...)`) | The arguments mean the opposite of their names, and the section is misplaced when the two edges' ranges differ. Reproduced |
| `mksweep` | Accepts 2 or 5 words, so `mksweep wire -C` without tolerances (the help's `[-C [AngTol LinTol]]`) fails with no message | Only `mksweep wire -C AngTol LinTol` corrects the spine. Reproduced |
| `errorsweep`, `deletesweep` | Dereference the sweep maker without checking that `mksweep` made one | An access violation natively; a .NET NullReferenceException (a Tcl error) in Draw-lite |
| `tanginterpol` | Checks `n < 3 * nbpoints + num_read` where reading the points needs one word more | With exactly one word too few the last coordinate is read from `argv[argc]`, a null pointer. Draw-lite throws (a Tcl error) |
| `thrusections` | The `Null3DCurve` status message lacks the final newline the others have | Cosmetic. Reproduced |

## Phase 7: unit 7A, ShapeCustom

Found by porting ShapeCustom and comparing it with native OCCT (decisions 973 to 978). The port reproduces each
(`// OCCT` at the line), unless the last column says otherwise.

| Where | What | Effect |
|---|---|---|
| ShapeCustom_SweptToElementary `NewCurve2d` | Calls `BRep_Tool::Surface(NewF, L)` on the null face BRepTools_Modifier passes from `CreateOtherVertices` (to ask whether an edge's vertices must be copied), which reaches it for edges without a new 3D curve (degenerated edges of revolutions) | An access violation natively (`OSD_Exception_ACCESS_VIOLATION` under the signal handler) on 16 of the heal group's and data/occ's shapes: `ShapeCustom::SweptToElementary` fails on any shape with such a face. The port throws a NullReferenceException there (decision 977) |
| ShapeCustom_SweptToElementary `NewCurve2d` | Dereferences `SS` (null) when the face's surface isn't swept and only the edge was copied, if the new face is a torus or a sphere | An access violation natively; the port skipped the shift (`is` on null) until unit 7C, and now throws a NullReferenceException there (decision 1024): ShapeProcess's SweptToElementary, BRepTools_Modifier in its mutable-input mode, meets it on three heal inputs |
| ShapeCustom_SweptToElementary `IsToConvert` | For a trimmed or offset surface, tests `S->IsKind(Geom_SweptSurface)` (known false) instead of its basis `Stmp` | Trimmed and offset surfaces of revolution or extrusion are never converted |
| ShapeCustom_Surface `ConvertToAnalytical` | `if (substitute && !NHS.IsNull()) Init(newSurf)`: the adaptor is never null, so a surface rejected by the final check (null) replaces the tool's surface | With `substitute`, a failed conversion leaves the tool with no surface; a later `ConvertToPeriodic` returns null |
| ShapeCustom_Surface `ConvertToAnalytical` | Computes the orientation check with `iso->D1(0., ...)` on the last iso-curve built (V at the middle), at parameter 0 rather than U1 | Reproduced |
| ShapeCustom_Curve, ShapeCustom_Surface `ConvertToPeriodic` | Make the given B-spline periodic in place when its knots needn't be rearranged | Changes the caller's (shared) geometry; the port changes a copy (decision 974) |
| ShapeCustom_Curve2d `SimplifyBSpline2d` | Removes knots from the caller's curve in place | As above; the port copies a frozen curve (decision 974) |
| ShapeCustom_BSplineRestriction `ConvertCurve2d` | For an offset pcurve whose basis converts to C0, converts the curve into `ResCurve`, not `C`, and returns true | The caller's C is left as it was (null in NewCurve2d), and the modifier stores a null pcurve. Not met by the tests |
| ShapeCustom_BSplineRestriction `ConvertCurve`, `ConvertCurve2d` | The `continue` meant to restart with the global limits (`aC = aC1`) is inside the inner `for (j = 1; j <= 2; j++)` loop: it retries once (ConvertCurve2d with the continuity computed before `aC = aC1`), and when the retry fails too and the curve isn't to be copied anyway, falls through to `myConvert = true; return true` | A failed approximation is returned as the converted curve, with its error as the tolerance. Reproduced |
| ShapeCustom_BSplineRestriction `IsConvertCurve3d` / `IsConvertCurve2d` | A 3D B-spline is converted from `NbKnots - 1 >= NbSeg` spans, a 2D one from `> NbSeg` | Reproduced |
| ShapeCustom_BSplineRestriction `NewPoint` | Answers from `myConvert`, the state of the last face or edge converted before the call; BRepTools_Modifier asks for points first | The vertices of a shape get new points only when the same modification converted something before (a compound's later children) |
| ShapeCustom_RestrictionParameters | `ConvertToroidalSurf` returns the cylinders' field and `ConvertCylindricalSurf` the tori's | Harmless: each accessor always uses the same field |
| BRepTools_Modifier `SetShapeFlags(S, S)` (reached by every ShapeCustom function) | Copies the flags of a sub-shape left alone onto itself; `Modified(true)` clears its Checked bit | Writes into the caller's shape natively; the port's frozen input keeps its flags (decision 347), the oracles mask the Checked column |
| ShapeExtend::Init with Message_MsgFile::Msg | Init loads SHAPE.us only when `ShapeFix.FixSmallSolid.MSG0` is missing, and `Msg` stores an unknown keyword's error text under the keyword: a ShapeFix_FixSmallSolid warning sent before Init (ShapeAlgo::Init or ShapeProcess) leaves every other healing message unknown for the rest of the process | Reproduced; the message tests load the texts themselves, since the tests share one process |
## Phase 7: unit 7W, Draw-lite's cheap gaps

Found by investigating the four cases decision 972 made runnable and by porting Draw's commands. Draw-lite reproduces
each (`// OCCT` at the line where it is code), unless the last column says otherwise.

| Where | What | Effect |
|---|---|---|
| bugs/modalg_7/bug26034 (`bopcheck` after `ttranslate Revolve002 0 0 -0.5`) | IntPatch_ImpImpIntersection of the sphere and a cone reaches `Standard_NumericError_Raise_if` in math_FunctionAllRoots (IntStart_SearchOnBoundaries::BoundedArc); BOPAlgo_CheckerSI turns the exception into the warning "Intersection of pair of shapes has failed", which the script reads as a fault | OCCT's default Release build compiles the check out (`No_Exception`), so OCCT's CI passes. build123d.NET's native build keeps the check, as the port does (decision 9): tools/cksirec (native BOPAlgo_CheckerSI) gives the same warning at -0.5 and none at 0, +0.5 and -0.25, as the port. The case fails on both |
| `insertpole` (GeomliteTest_CurveCommands.cxx) | Returns 1 below 6 words, so the 2D form `insertpole name index x y` can't be given; with 6 words the 2D form takes `a[5]` as the weight | A 2D pole is always inserted with a weight. Reproduced |
| `dmode`, `discr`, `defle` (DrawTrSurf.cxx) | With names, every iteration sets the drawable of `a[1]`, not `a[i]`; `dmode` with one word sets the uniform mode whatever the word | Display only: Draw-lite keeps the modal values and changes no drawable |
| `lastrep` (Draw_VariableCommands.cxx) | With Z, divides the pick by `dout.Zoom`, which is `false` (0) in batch mode | X, Y and Z are NaN when no view exists. Reproduced |
| `crvpoints`, `crvtpoints` (GeometryTest_CurveCommands.cxx) | Read `a[2]` and `a[3]` without checking the word count | Natively undefined; Draw-lite reads an empty word |
| `2dintersect` (GeomliteTest_API2dCommands.cxx) | `-tol` as the last word reads `a[++i]` past the end; the output spells "parameter on the fist" | Draw-lite reads an empty word (0); the text is reproduced |
| `xbounds`, `removeinternals` (BRepTest_OtherCommands.cxx, DBRep.cxx) | Messages without a space: " shape<name> must be a face", "<name>is a null shape" | Text only. Reproduced |
| `XProgress` | Sets Draw_ProgressIndicator's process-wide defaults; the corpus's progress cases read the indicator's output in the commands' results | Draw-lite keeps the modes per interpreter and its commands make no progress indicator (decision 954), so those cases fail (decisions 984, 988) |
| BRepPrimAPI_MakePrism's history on a sphere band from 0 to 2π (bugs/modalg_7/bug30346_2, `prism` then `savehistory`) | The port's IsDeleted says one edge of the base is deleted; native (build123d.NET's MakePrism on the same face) says none | Not OCCT's: a port difference in what BRepSweep marks used, found by decision 981 and not yet fixed. The case fails |

## Phase 7: unit 7W's follow-up, the Boolean corpus in full

| Where | What | Effect |
|---|---|---|
| bugs/modalg_7/bug83 (`bsection` of two planar B-spline sheets from BUC60912_sec_slow.brep) | Not OCCT's: the port's section takes 0.20 s warm in Release where native takes 0.03 s (and 0.34 s on a cold start, the JIT) | The case's 0.1 s limit fails. A performance gap to profile (plan 2.7), not a result difference |

## Phase 7: unit 7D, helix (TKHelix)

Found by porting TKHelix and BRepTest_HelixCommands.cxx. The port reproduces each (`// OCCT` at the line where it is
code), unless the last column says otherwise.

| Where | What | Effect |
|---|---|---|
| HelixGeom_HelixCurve `Load` | `myTgBeta` is set only when \|taper\| > 1e-4, never reset: reloading a tapered adaptor with a (near) zero taper keeps the old taper | The second helix is tapered. Not reached by TKHelix's builders, which load a new adaptor each time. Reproduced, and pinned by a test |
| HelixBRep_BuilderHelix `SetParameters` (spiral, by heights) | Reads `theHeights(1)` where the loop uses `theHeights.Lower()` | An array not starting at 1 is read at the wrong place or throws. Reproduced |
| HelixBRep_BuilderHelix (class documentation) | Error 13 is documented for "TaperAngle < 0 or > π/2 - TolAng"; `BuildPart` checks only the upper bound, and returns 3 (undocumented) when an edge can't be made | Negative tapers (diameters decreasing) are built, as the corpus's spirals need |
| HelixBRep_BuilderHelix `SmoothingEdges` | Changes the edges' curves in place; when no degree lets the tangent move, the curve keeps the raised degree though `UpdateEdge` isn't called | Native shares no curve with a caller, so harmless; the port changes copies (decision 994) |
| HelixGeom_Tools `HelixGeom_Tools_Eval::Evaluate` | Sets error code 1 for a dimension other than 3, then writes three values anyway | Never reached (the dimension is always 3). Reproduced |
| `helix2`, `spiral2` (BRepTest_HelixCommands.cxx) | The usage text for a wrong argument count names H and PF arguments the commands don't take | Text only. Reproduced |
| helix/standard/D6, D7 | `CheckSpiral` compares lprops' centre and moments as strings, centre coordinates near 1e-15 included | OCCT's own TODO (OCC11111) on all platforms; BAD in Draw-lite as in DRAWEXE |
## Phase 7: unit 7B, ShapeUpgrade and ShapeAlgo

The port reproduces each, unless the last column says otherwise.

| Where | What | Effect |
|---|---|---|
| ShapeUpgrade_SplitSurfaceContinuity `Compute` | For a surface of linear extrusion whose basis curve had knots removed, `SetBasisCurve` on the surface it was given: the face's own surface (for a trimmed one, the basis shared by its faces) | Changes the caller's geometry (heal's BUC60173.rle with C2 criteria); the port changes a copy (decision 998), the one attributed oracle difference |
| ShapeUpgrade_SplitSurfaceContinuity `Compute` | For a surface of revolution, the knots removed from the basis curve aren't set into the surface (unlike the extrusion); DONE3 is reported anyway | Reproduced |
| ShapeUpgrade::C0BSplineToSequenceOfC1BSplineCurve | `SetNotPeriodic` on the caller's curve | The port splits a copy of a frozen curve (decision 998) |
| ShapeUpgrade_ShapeConvertToBezier `Perform` | Segments the edges' Bezier pcurves in place; where a seam's two pcurves are one curve, it is segmented twice (an arc ending at π/4 gets 0.49, data/occ's nuts and many heal inputs in curves-only mode) | Reproduced on copies (decision 998) |
| ShapeUpgrade_FixSmallBezierCurves `Approx` | For the seam's second pcurve, compares the bounds with the first pcurve's range and lowers `First` with `if (First > f)` | Reproduced |
| ShapeUpgrade_FixSmallBezierCurves `Approx` | Re-initializes the wire divider's own split tools, which WireDivide is still using | Reproduced |
| ShapeUpgrade_ShapeDivide `Perform` | Dereferences the split face tool for the free wires without checking it | Reproduced (NullReferenceException) |
| ShapeUpgrade_ShellSewing `ApplySewing` | Documented to return the input when no shell is sewn; returns a null shape | Reproduced |
| ShapeUpgrade_ShapeDivideClosedEdges `SetNbSplitPoints` | Ignores the number | Reproduced |
| ShapeUpgrade_SplitCurve2dContinuity `Compute` | After removing a knot completely, doesn't lower the last index as the 3D tool does | Reproduced |

## Phase 7: unit 7E, the offset foundations

Found by porting BRepOffset (but MakeOffset) and ChFi3d.cxx. The port reproduces each (`// OCCT` at the line), unless
the last column says otherwise.

| Where | What | Effect |
|---|---|---|
| BRepOffset_Tool `IsOnSurface` (:577) | The sampling step is `(f - l)`, negative | Samples outside the curve's range. Reproduced |
| BRepOffset_Tool `AngleWireEdge` (:931) | `Angle` is uninitialised when the edges aren't B-splines | Undefined natively; the port uses 0 |
| BRepOffset_Tool `PutInBounds` (:113), `BuildPCurves` (:397) | The bounds are uninitialised when `FindPeriod` writes nothing; the pcurve can be null when `PutInBounds` dereferences it | Undefined natively; the port uses 0, and throws on the null curve |
| BRepOffset_Tool `CheckBounds` (:3146) | `L.First()` on an empty list | Undefined in native release builds; the port throws Standard_NoSuchObject |
| BRepOffset_Tool `Inter3D`, the closed-wire branch | Could loop forever if the edges don't chain | Looks unreachable (a built wire's Closed flag is never set). Reproduced |
| BRepOffset_Inter2d `RefEdgeInter` ("keep Vmin") | `Remove` moves the iterators on, then the loop's `Next()` skips an element | A vertex after a removed one is kept. Reproduced |
| BRepOffset_Inter2d `ExtentEdge` | `MinSurf` is replaced by a transformed copy, so `theSurf == MinSurf` never holds | The pcurve the 3D curve was built from is checked (and may be re-projected) too. Reproduced |
| BRepOffset_MakeLoops (three places) | `NewTV->ChangePoints().Append(TV->ChangePoints())` moves the list | The replaced vertex loses its point representations, which matters only if it survives outside the images. Reproduced |
| BRepOffset_Inter3d `ContextIntByInt` | `LOE` and `MV` are built and never used; on the vertex path `Generated(E)` gets a null edge | Harmless. Reproduced |
| BRepOffset_SimpleOffset `FillVertexData` | A vertex with no offset edge divides by zero | Its new point is NaN. Reproduced |
| BRepOffset_MakeSimpleOffset `GetSafeOffset` | Divides by `myMaxAngle`, which is 0 for a smooth shape | Infinite or NaN. Reproduced |
| BRepOffset_MakeSimpleOffset `BuildMissingWalls` | `IsClosed` on a null shell when the quilt makes none (an edge in solid mode) | An access violation natively; Standard_NullObject in the port (decision 1010) |
| BRepOffset_MakeSimpleOffset, solid mode | Writes the walls' pcurves into the input's free edges and raises their vertices' tolerances | A second run on the same input gives another result natively (decision 1010). The port copies at the entry (decision 1006), so the input is never written |
## Phase 7: unit 7H, LocOpe

Found by porting LocOpe and recording it natively (tools/locoperec). The port reproduces each, unless the last column
says otherwise.

| Where | What | Effect |
|---|---|---|
| LocOpe_SplitShape, WiresOnShape, Spliter, Generator, Gluer, SplitDrafts, the sweeps | Write into the shapes they are given: vertices' point representations and tolerances, edges' pcurves and continuities, the split shape's faces (plan 2.2) | The caller's shapes change. The port works on copies of frozen shapes and gives back the unchanged ones (decisions 1013, 1014) |
| LocOpe_SplitDrafts `Perform` | Builds a GeomFill_Pipe on the two drafted planes' intersection, an infinite Geom_Line, which GeomConvert can't convert ("No such curve") | Throws Standard_DomainError whenever the planes meet (natively on all 229 such recorded cases; the 133 others aren't done): the class can't draft a plane. Reproduced |
| LocOpe_SplitDrafts `Perform` | After the right edge is built, tests the left one (`NewEdgg`) again; the tolerance compared is NewEdgg's twice | Reproduced |
| LocOpe_DPrism (through BRepFill_Evolved) | The order of an edge's two pcurves in the result, and the last bits of the faces' areas, follow addresses | Native's BRep text varies run to run; the oracle compares the shapes canonically (decision 1017) |
| LocOpe_CSIntersector `LocalizeBefore(I, FromInd, ...)` | No index version of `LocBefore` exists: the index converts to the double parameter `From` | Searches before the parameter equal to the index. Reproduced |
| LocOpe_WiresOnShape `Add` | Puts each argument's box at the argument's index but its edges at the next free indices; an argument with several edges (a wire) leaves boxes and edges out of step | Edges tested against another edge's box. Reproduced |
| LocOpe_WiresOnShape `Project` | The V resolution is computed with `UResolution` | Reproduced |
| LocOpe_WiresOnShape `PutPCurve` | The branch for internal vertices (`!V1.IsNull() && V2.IsNull()`) reads the tolerance and point of the null V2 | Throws when reached. Reproduced |
| LocOpe_FindEdges `Set` | Compares B-spline knots with `Kf(k) - Kt(k) > Tol`, not in absolute value | Knots smaller than the other's pass. Reproduced |
| LocOpe_Pipe | Corrects the orientation of a fused planar face's normal with the reference plane's `Direct()`, not the face's own plane's | Reproduced |
| LocOpe_Pipe `Curves`, `BarycCurve` | A straight spine's pipe lines are infinite lines, which GeomConvert can't convert; `BarycCurve` with no curve appends a null curve and goes on to read the first | Throws Standard_DomainError / Standard_OutOfRange. Reproduced |
| LocOpe_Generator `Perform` | Computes whether a face's wire touches the removed edges (`isIncludedInW`) and doesn't use it | Reproduced |
| LocOpe_Gluer `AddEdges` | Computes, for each face of the result, the new shape's edges touching it, and keeps nothing | Reproduced (for its cost) |
| LocOpe_Revol, LocOpe_RevolutionForm | Declare constructors taking the base and the axis without defining them; RevolutionForm never sets its rotation (`myIsTrans` stays false) | The constructors are left out (decision 1015) |
| LocOpe_LinearForm | Stores the two points it is given and never reads them | Reproduced |
| LocOpe_Spliter `Perform` | Builds the left part from maps keyed by addresses | `Left()`'s order varies natively; the oracle compares it as a set |
## Phase 7: unit 7C, ShapeProcess and the healing Draw commands

Found by porting ShapeProcess, ShapeProcessAPI, `Resource_Manager` and SWDRAW's ShapeUpgrade, ShapeCustom and
ShapeProcess commands. The port reproduces each, unless the last column says otherwise.

| Where | What | Effect |
|---|---|---|
| ShapeProcess_OperLibrary `Init` | Sets its static `done` flag before registering the operators, without a lock | A second thread calling `Init` meanwhile returns at once and may find no operator. The port registers under a lock (decision 1021) |
| ShapeProcess `RegisterOperator`, `FindOperator` | The static operator map has no lock | Registering while another thread performs can corrupt it. The port takes a lock (decision 1021) |
| ShapeProcess_Context `LoadResourceManager` | `stat` of a file that doesn't exist isn't checked: the modification time is read from an uninitialised buffer; the cache compares the name only, not the directories | A missing file may look modified. The port takes 0 for a missing file and keys the cache by the files too (decision 1023) |
| ShapeProcess_Context `GetReal`, `GetInteger` | Read the first character of a value that isn't a number (`Value(1)`), which throws Standard_OutOfRange for an empty value | The exception leaves the operator; the performer reports "failed with exception". Reproduced |
| Resource_Manager (constructors) | The user resource file is loaded into the reference map (`Load(aUserPath, myRefMap)`), so the "User file loaded" trace can't be printed and `GetMap(false)` holds only what `SetResource` set | Reproduced |
| ShapeProcess `settol` (SetTolerance), `sameparam` (SameParameter) | Change tolerances and pcurves in place and record nothing in the context ("WARNING: no update of context yet!") | The history has no entry for them. Reproduced; in the port the result holds changed copies (decision 1022) |
| BRepLib `UpdateTolerances` (in place, from SetTolerance) | Sets the tolerances computed per (sub-shape, location) by iterating a map keyed by shape in hash order (TShape addresses); a TShape the shape holds at two locations, with different tolerances at each, ends with whichever OCCT iterates last | The port iterates in insertion order: 28 of the oracle's heal inputs (instances) get the other tolerance on a few vertices and edges, attributed |
| ShapeProcessAPI_ApplySequence `PrepareShape`, `ClearMap` | `fillmap` and `until` are ignored; `ClearMap` does nothing | Reproduced |
| SWDRAW `DT_SplitCurve`, `DT_SplitCurve2d`, `DT_SplitSurface` | Read `argv[argc]` (a null pointer) when the optional split flag is missing | Draw-lite takes a missing flag as 0 |
| SWDRAW `splitface` | Prints ", " before every U value (`i > j` where `j > 1` is meant) | Reproduced |
| SWDRAW `expshape` | Counts a seam twice per face | Reproduced |
| SWDRAW `offset2dcurve` | On a C0 curve, Geom2d_OffsetCurve throws Standard_ConstructionError ("Offset on C0 curve") | Reproduced |

## Phase 7: unit 7F, the 3D offset

Found by porting BRepOffset_MakeOffset and the offset API, and by its oracle. The port reproduces each (`// OCCT` at
the line), unless the last column says otherwise.

| Where | What | Effect |
|---|---|---|
| BRepOffset_MakeOffset `TrimEdges` | `if (theNewEdges.Add(NE))` tests the index NCollection_IndexedMap::Add returns, never 0 | An edge already trimmed is trimmed again. Reproduced |
| BRepOffset_BuildOffsetFaces `BuildSplitsOfFaces` | In the artificial case, `*pLFIm = aLFImages` when the face has no splits yet (`pLFIm` null) | An access violation natively; a NullReferenceException in the port. Not met by the oracle |
| BRepOffset_MakeOffset (MakeLoops on the closing faces, `MakeMissingWalls`, `UpdateTolerance` with `Locked(false)`) | Writes into its input: new vertices on the closing faces' edges, the walls' pcurves on a thickened shell's edges, tolerances | Native changed its own input in 82 of the oracle's 724 runs; a second run on it differs. The port copies at the entry (decision 1030) |
| BRepOffset_MakeOffset, the Arc join with Intersection on | Iterates shape-keyed maps by address (decision 1032) | The result's text varies from run to run natively (about 40 of the oracle's runs on primitives). The port's order is insertion order |
| BRepOffset_MakeOffset, BRepOffsetAPI_MakeThickSolid | `IsDone()` with a null `Shape()` (open shells with nothing left to offset, a hollow sphere) | Reproduced (16 oracle runs); build123d.NET checks for it |
| BSplCLib `AntiBoorScheme` through `RemoveKnot` (GeomLib::ExtendSurfByLength, the offset's face extension) | Steps outside its NCollection_LocalArray buffers, reading and writing the stack around them | Undefined natively (the tolerance test then fails); the port's `RemoveKnot` reports the removal impossible instead of throwing (decision 1034) |
| BRepOffset_MakeOffset `SelfInter` | Throws Standard_NotImplemented | Never called. Reproduced |

## Phase 7: unit 7J, the blend functions

Found by porting Blend, BlendFunc and ChFiDS and by tools/blendrec. The port reproduces each (`// OCCT` at the line).

| Where | What | Effect |
|---|---|---|
| BlendFunc_ConstRad, BlendFunc_EvolRad `ComputeValues` | The scratch values (the guide's point and derivatives, the surfaces' third derivatives) are `static thread_local`, shared by every instance on a thread, while the cache guards are per instance | An instance whose parameter is unchanged can read the guide point another instance computed last. Reproduced with `[ThreadStatic]` (decision 1037) |
| BlendFunc_ConstRadInv `Derivatives` | `dnplan.SetLinearForm(theD, nplan, d2cur)` where `Values` has `-nplan.Dot(d2cur)` | The derivatives in the guide's parameter (rows 2 to 4) differ from `Values`' and from finite differences off the origin (decision 1038) |
| BlendFunc_EvolRad `ComputeValues` | The order-2 positioning doesn't recompute `dns1u1`, `dns1v1`, `dns1u2`, `dns1v2` (ConstRad's does) | The second derivatives use the last order-1 values (decision 1039) |
| BlendFunc_ConstThroatInv, ConstThroatWithPenetrationInv `Derivatives` | `temp1`, `temp2`, `tempmid`/`temp3` are computed from `pts1`, `pts2` before `D1` updates them | They are the previous call's points (the same point when `Values` calls `Value` first) |
| BlendFunc_ChAsym `Section` (first order) | Writes `PointOnS1()`/`PointOnS2()` into the poles before computing the points | The poles are the previous computation's points |
| BlendFunc_ChAsymInv `ComputeValues` | On the first surface's restriction, `DX(4,2)` uses `dnplan.Dot(tempVec)` with `tempVec = 2 * nps1` | Disagrees with finite differences on a curved guide (2 against 0 on the plane/cylinder case) |
| BlendFunc_CSCircular `GetTolerance` | Passes the signed radius to GeomFill::GetTolerance (CSConstRad passes its absolute value) | A negative radius makes gp_Circ throw Standard_ConstructionError |
| BlendFunc_EvolRad `GetTolerance` | `lengthmin / maxang` before any solution (both still at RealLast/RealFirst) gives a negative radius | gp_Circ throws Standard_ConstructionError |
| BlendFunc_Chamfer, ConstThroat, ChAsym, Ruled (GenChamfer) `GetTolerance` | Leave `Tol1D` unset | The caller's vector keeps whatever it held (uninitialised memory natively) |
| BlendFunc `Knots`, `Mults` | Declared, never defined | Left out |
| ChFiDS_Stripe `Reset` | `pcrv1.Nullify()` twice | The last pcurve is kept |
| ChFiDS_FilSpine `SetRadius(Law, IinC)` | Builds a composite law from the law, drops it, and clears the radii | The law is lost |
| Blend_CurvPointFuncInv `NbVariables` | Documented as 3, returns 2 | Reproduced |

## Phase 7: unit 7L, the legacy data structure's core

Found by porting TopOpeBRepDS's core and TopOpeBRepTool_ShapeTool. The port reproduces each (`// OCCT` or a remark at
the place).

| Where | What | Effect |
|---|---|---|
| TopOpeBRepDS_SurfaceIterator constructor | Sets the SURFACE condition after the base constructor has matched, without matching again | The first `Value()` is the list's first interference whatever its geometry (`HDataStructure::SolidSurfaces`); `Next()` then skips the others |
| TopOpeBRepDS_DataStructure `Change*Interferences`, `ChangeCurve`, `ChangePoint`, `ChangeSurface` | Return the data structure's own empty list or value, for writing, when the shape or geometry is missing | What is written there is every later miss's list or value (decision 9905) |
| TopOpeBRepDS_Point(shape) | Leaves `myKeep` unset; the default constructor leaves the tolerance unset | Uninitialised natively; the port keeps the point and uses 0 |
| TopOpeBRepDS_Interference() | Leaves the indices and kinds unset | Uninitialised natively; the port uses 0 and POINT |
| TopOpeBRepDS_SurfaceExplorer() | Leaves `myFindKeep` unset | Uninitialised natively; the port uses false |
| TopOpeBRepDS_CurveIterator `PCurve`, TopOpeBRepDS_Curve `Curve1`/`Curve2` | Cast the interference to a surface/curve interference without checking | Undefined natively on another class; the port throws InvalidCastException |
| TopOpeBRepDS_Surface | A new surface is not kept, a new point or curve is | `SurfaceExplorer` with its default `FindOnlyKeep` finds none of a fillet's surfaces |
| TopOpeBRepDS_DataStructure `IsSectionEdge`, `SectionEdge` | Need the edge among the data structure's shapes too, not only among the section edges | A section edge added alone is not one |
| TopOpeBRepDS_HDataStructure `FUN_TopOpeBRepDS_SortOnParameter` | Counts all interferences but finds only point ones | With other interferences in the list, the last point found is appended again (a null one first if none was found, then `T(0)` out of range: undefined natively, the port throws) |
| TopOpeBRepTool_ShapeTool `EdgeData` | Compares the curvature with `max(tol, Epsilon(0), RealLast())` | The normal is never computed |
| TopOpeBRepTool_ShapeTool `SurfacesSameOriented`, `CurvesSameOriented` | Only planes and cylinders, and lines, are compared ("NYI") | Every other pair is "same oriented" |
