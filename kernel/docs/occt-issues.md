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
