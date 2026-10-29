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
| `Bnd_Sphere::Add` (Bnd_Sphere.cxx) | Treats a negative radius as "not initialised yet", but the default constructor sets the radius to 0 | Adding a sphere to a default-constructed `Bnd_Sphere` also encloses the origin instead of copying the other sphere |
| `Bnd_OBB::ReBuild`, two coincident points (Bnd_OBB.cxx) | Calls `ProcessOnePoint`, which resets `myHDims` to 0, then enlarges by `myHDims[1] + Precision::Confusion()` | The point tolerances are dropped: the half-sizes are `Precision::Confusion()` instead of the larger tolerance plus Confusion |
| `NCollection_Mat3::Adjoint` | Documented as the adjugate, returns the cofactor matrix (its transpose); `Inverted` transposes it back | Callers expecting the adjugate get its transpose |
| `NCollection_Mat4::Adjoint` | `crossVec4` returns the negated minors, so the result is the negated cofactor matrix, not the adjugate and not Mat3's convention | Wrong sign and transposition; nothing in OCCT calls it |
| `BVH::SquareDistanceToObject`, `SquareDistanceToGeomerty` (BVH_DistanceField.lxx) | Return `false` (a zero distance) when the tree is null | Dead code (`BVH()` never returns null); cosmetic |
