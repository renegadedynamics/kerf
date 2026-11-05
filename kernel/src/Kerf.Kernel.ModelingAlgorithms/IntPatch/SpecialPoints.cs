// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_SpecialPoints.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_SpecialPoints.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Walking;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Engine.Intersection.Patches.SpecPntType;

namespace Kerf.Engine.Intersection.Patches;

/// <summary>
/// Special points of intersection lines: the apex of a cone, the poles of a sphere, points on the seams of a quadric.
/// Finds them and adds them (with correct parameters) to walking lines.
/// </summary>
/// <remarks>
/// OCCT's static class. Its file-local function <c>FuncPreciseSeam</c> is a private nested class, and
/// <c>AdjustPointAndVertex</c>'s optional vertex pointer is an overload taking the vertex by <c>ref</c>. The
/// protected members (ProcessSphere, ProcessCone, GetTangentToIntLineForCone) are internal.
/// </remarks>
internal static class SpecialPoints
{
    /// <summary>The function whose zero is an intersection point lying on a seam (isoline) of the quadric.</summary>
    private sealed class FuncPreciseSeam : FunctionSetWithDerivatives
    {
        private readonly SurfaceAdaptorBase _qSurf;
        private readonly SurfaceAdaptorBase _pSurf;

        // 1 for U-coordinate, 0 - for V one.
        private readonly int _seamCoordInd;

        // Constant parameter of iso-line
        private readonly double _isoParameter;

        public FuncPreciseSeam(SurfaceAdaptorBase qSurf, SurfaceAdaptorBase pSurf, bool isTheUSeam,
                               double isoParameter)
        {
            _qSurf = qSurf;
            _pSurf = pSurf;
            _seamCoordInd = isTheUSeam ? 1 : 0; // Defines, U- or V-seam is used
            _isoParameter = isoParameter;
        }

        public override int NbVariables() => 3;

        public override int NbEquations() => 3;

        public override bool Value(math_Vector x, math_Vector f)
        {
            try
            {
                int indX = x.Lower(), indF = f.Lower();
                Span<double> uv = [_isoParameter, _isoParameter];
                uv[_seamCoordInd] = x[indX + 2];
                Point3D p1 = _pSurf.Value(x[indX], x[indX + 1]);
                Point3D p2 = _qSurf.Value(uv[0], uv[1]);

                (p1.XYZ() - p2.XYZ()).Coord(out f[indF], out f[indF + 1], out f[indF + 2]);
            }
            catch (Exception caught) when (Refusal.IsFailure(caught))
            {
                return false;
            }

            return true;
        }

        public override bool Derivatives(math_Vector x, Matrix d)
        {
            try
            {
                int indX = x.Lower(), indRD = d.LowerRow(), indCD = d.LowerCol();
                Span<double> uv = [_isoParameter, _isoParameter];
                uv[_seamCoordInd] = x[indX + 2];

                // 0 for U-coordinate, 1 - for V one
                _pSurf.D1(x[indX], x[indX + 1], out _, out Vector3D d10, out Vector3D d11);
                _qSurf.D1(uv[0], uv[1], out _, out Vector3D d20, out Vector3D d21);

                // d/dX1
                d10.Coord(out d[indRD, indCD], out d[indRD + 1, indCD], out d[indRD + 2, indCD]);

                // d/dX2
                d11.Coord(out d[indRD, indCD + 1], out d[indRD + 1, indCD + 1],
                           out d[indRD + 2, indCD + 1]);

                // d/dX3
                (_seamCoordInd == 0 ? d20 : d21).Reversed().Coord(out d[indRD, indCD + 2],
                                                                    out d[indRD + 1, indCD + 2],
                                                                    out d[indRD + 2, indCD + 2]);
            }
            catch (Exception caught) when (Refusal.IsFailure(caught))
            {
                return false;
            }

            return true;
        }

        public override bool Values(math_Vector x, math_Vector f, Matrix d)
        {
            if (!Value(x, f))
            {
                return false;
            }

            if (!Derivatives(x, d))
            {
                return false;
            }

            return true;
        }
    }

    // Computes tangent having the given parameter. See calling method(s) for detailed information.
    private static Xyz GetTangent(double coneSemiAngle, double parameter)
    {
        double w2 = parameter * parameter;
        double cosUn = (1.0 - w2) / (1.0 + w2);
        double sinUn = 2.0 * parameter / (1.0 + w2);

        double tanA = Math.Tan(coneSemiAngle);
        return new Xyz(tanA * cosUn, tanA * sinUn, 1.0);
    }

    // Checks if thePt is in theSurf (with given tolerance). Returns the foot of projection (theProjPt) and its
    // parameters on theSurf.
    private static bool IsPointOnSurface(SurfaceAdaptorBase surf, in Point3D pt, double tol,
                                         ref Point3D projPt, ref double upar, ref double vpar)
    {
        bool retVal = false;

        switch (surf.GetType())
        {
            case Plane:
            case SurfaceKind.Cylinder:
            case SurfaceKind.Cone:
            case SurfaceKind.Sphere:
            case SurfaceKind.Torus:
            case SurfaceOfExtrusion:
            case SurfaceOfRevolution:
            {
                ExtPS extr = new(pt, surf, surf.UResolution(tol), surf.VResolution(tol),
                                           ExtFlag.ExtFlag_MIN);
                if (!extr.IsDone || (extr.ExtCount < 1))
                {
                    retVal = false;
                }
                else
                {
                    int extrIndex = 1;
                    double sqDistMin = extr.SquareDistance(extrIndex);
                    for (int i = extrIndex + 1; i <= extr.ExtCount; i++)
                    {
                        double sqD = extr.SquareDistance(i);
                        if (sqD < sqDistMin)
                        {
                            sqDistMin = sqD;
                            extrIndex = i;
                        }
                    }

                    if (sqDistMin > tol * tol)
                    {
                        retVal = false;
                    }
                    else
                    {
                        projPt = new Point3D(extr.Point(extrIndex).Value().XYZ());
                        extr.Point(extrIndex).Parameter(out upar, out vpar);
                        retVal = true;
                    }
                }
            }
            break;
            default:
            {
                GenLocateExtPS extr = new(surf);
                extr.Perform(pt, upar, vpar);
                if (!extr.IsDone || (extr.SquareDistance() > tol * tol))
                {
                    retVal = false;
                }
                else
                {
                    extr.Point().Parameter(out upar, out vpar);
                    projPt = new Point3D(extr.Point().Value().XYZ());
                    retVal = true;
                }
            }
            break;
        }

        return retVal;
    }

    /// <summary>
    /// Computes the point at the intersection of the isolines U = 0 and V = 0 of <paramref name="qSurf"/> lying on
    /// <paramref name="pSurf"/>, adjusted to <paramref name="refPt"/>'s periods. If
    /// <paramref name="isReversed"/>, theQSurf is the second surface of the point.
    /// </summary>
    public static bool AddCrossUVIsoPoint(SurfaceAdaptorBase qSurf, SurfaceAdaptorBase pSurf,
                                          in PntOn2S refPt, double tol,
                                          ref PntOn2S addedPoint, bool isReversed = false)
    {
        Span<double> arrOfPeriod = [0.0, 0.0, 0.0, 0.0];
        global::Kerf.Engine.Intersection.Walking.IntSurf.SetPeriod(isReversed ? pSurf : qSurf, isReversed ? qSurf : pSurf, arrOfPeriod);

        // Not quadric point
        double u0, v0;
        if (isReversed)
        {
            refPt.ParametersOnS1(out u0, out v0);
        }
        else
        {
            refPt.ParametersOnS2(out u0, out v0);
        }

        // Quadric point
        double uquad = 0.0, vquad = 0.0;

        qSurf.D0(uquad, vquad, out Point3D pQuad);

        GenLocateExtPS extr = new(pSurf);
        extr.Perform(pQuad, u0, v0);

        if (!extr.IsDone)
        {
            return false;
        }

        if (extr.SquareDistance() > tol * tol)
        {
            return false;
        }

        extr.Point().Parameter(out u0, out v0);
        Point3D p0 = extr.Point().Value();

        if (isReversed)
        {
            addedPoint.SetValue(new Point3D(0.5 * (p0.XYZ() + pQuad.XYZ())), u0, v0, uquad, vquad);
        }
        else
        {
            addedPoint.SetValue(new Point3D(0.5 * (p0.XYZ() + pQuad.XYZ())), uquad, vquad, u0, v0);
        }

        AdjustPointAndVertex(refPt, arrOfPeriod, ref addedPoint);

        return true;
    }

    /// <summary>
    /// Computes the point lying strictly on the isoline U = <paramref name="isoParameter"/> (if
    /// <paramref name="isU"/>) or V = theIsoParameter of <paramref name="qSurf"/> and on
    /// <paramref name="pSurf"/>. The vectors hold the U and V parameters on thePSurf, then the free parameter on
    /// theQSurf.
    /// </summary>
    public static bool AddPointOnUorVIso(SurfaceAdaptorBase qSurf, SurfaceAdaptorBase pSurf,
                                         in PntOn2S refPt, bool isU, double isoParameter,
                                         math_Vector toler, math_Vector initPoint, math_Vector infBound,
                                         math_Vector supBound, ref PntOn2S addedPoint,
                                         bool isReversed = false)
    {
        Span<double> arrOfPeriod = [0.0, 0.0, 0.0, 0.0];
        global::Kerf.Engine.Intersection.Walking.IntSurf.SetPeriod(isReversed ? pSurf : qSurf, isReversed ? qSurf : pSurf, arrOfPeriod);

        FuncPreciseSeam f = new(qSurf, pSurf, isU, isoParameter);

        FunctionSetRoot srf = new(f, toler);
        srf.Perform(f, initPoint, infBound, supBound);

        if (!srf.IsDone)
        {
            return false;
        }

        math_Vector roots = new(initPoint.Lower(), initPoint.Upper());
        srf.Root(roots);

        // On parametric
        double u0 = roots[1], v0 = roots[2];

        // On quadric
        double uquad = isU ? 0.0 : roots[3];
        double vquad = isU ? roots[3] : 0.0;
        Point3D pQuad = qSurf.Value(uquad, vquad);
        Point3D p0 = pSurf.Value(u0, v0);

        if (isReversed)
        {
            addedPoint.SetValue(new Point3D(0.5 * (p0.XYZ() + pQuad.XYZ())), u0, v0, uquad, vquad);
        }
        else
        {
            addedPoint.SetValue(new Point3D(0.5 * (p0.XYZ() + pQuad.XYZ())), uquad, vquad, u0, v0);
        }

        AdjustPointAndVertex(refPt, arrOfPeriod, ref addedPoint);
        return true;
    }

    /// <summary>
    /// Computes the "special point" on a sphere (the U parameter at a pole), in [0, 2*PI]; the caller adjusts it.
    /// </summary>
    internal static bool ProcessSphere(in PntOn2S ptIso, in Vector3D dUofPSurf, in Vector3D dVofPSurf,
                                       bool isReversed, double vquad, ref double uquad,
                                       out bool isIsoChoosen)
    {
        isIsoChoosen = false;

        // Vector with {cos(Uq), sin(Uq)} coordinates. It is always normalized.
        Vector2D v1 = new();

        if ((Math.Abs(dUofPSurf.Z) < global::Kerf.Engine.Foundation.Precision.PConfusion()) && (Math.Abs(dVofPSurf.Z) < global::Kerf.Engine.Foundation.Precision.PConfusion()))
        {
            // Example of this case is an intersection of a plane with a sphere when the plane tangents the sphere in
            // some pole (i.e. only one intersection point, not line). In this case, U-coordinate of the sphere is
            // undefined (can be really anything). Another reason is that we have tangent zone around the pole (see bug
            // #26576). Computation of correct value of theUquad is impossible. Therefore, (in order to return
            // something) we will consider the intersection line goes along some isoline in neighborhood of the pole.
            double uIso, vIso;
            if (isReversed)
            {
                ptIso.ParametersOnS2(out uIso, out vIso);
            }
            else
            {
                ptIso.ParametersOnS1(out uIso, out vIso);
            }

            uquad = uIso;
            isIsoChoosen = true;
        }
        else
        {
            if (Math.Abs(dUofPSurf.Z) > Math.Abs(dVofPSurf.Z))
            {
                double dusDvs = dVofPSurf.Z / dUofPSurf.Z;
                v1 = v1.WithCoord(dUofPSurf.X * dusDvs - dVofPSurf.X, dUofPSurf.Y * dusDvs - dVofPSurf.Y);
            }
            else
            {
                double dvsDus = dUofPSurf.Z / dVofPSurf.Z;
                v1 = v1.WithCoord(dVofPSurf.X * dvsDus - dUofPSurf.X, dVofPSurf.Y * dvsDus - dUofPSurf.Y);
            }

            v1 = v1.Normalized();

            if (Math.Abs(v1.X) > Math.Abs(v1.Y))
            {
                uquad = Math.CopySign(Math.Asin(v1.Y), vquad);
            }
            else
            {
                uquad = Math.CopySign(Math.Acos(v1.X), vquad);
            }
        }

        return true;
    }

    /// <summary>
    /// Computes the "special point" on a cone (the U parameter at the apex), in [0, 2*PI]; the caller adjusts it.
    /// </summary>
    internal static bool ProcessCone(in PntOn2S ptIso, in Vector3D dUofPSurf, in Vector3D dVofPSurf,
                                     in Cone3D cone, bool isReversed, ref double uquad,
                                     out bool isIsoChoosen)
    {
        isIsoChoosen = false;

        // A plane tangent to 2nd (intersected) surface. Its normal.
        Xyz tgPlaneZ = dUofPSurf.Crossed(dVofPSurf).XYZ();
        double sqModTg = tgPlaneZ.SquareModulus();
        if (sqModTg < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            isIsoChoosen = true;
        }

        Xyz[] tgILine = new Xyz[2];
        int nbTangent = !isIsoChoosen
                             ? GetTangentToIntLineForCone(cone.SemiAngle, tgPlaneZ.Divided(Math.Sqrt(sqModTg)),
                                                          tgILine)
                             : 0;

        if (nbTangent == 0)
        {
            isIsoChoosen = true;
        }
        else
        {
            double period = M_PI + M_PI;
            double uIso, vIso;
            if (isReversed)
            {
                ptIso.ParametersOnS2(out uIso, out vIso);
            }
            else
            {
                ptIso.ParametersOnS1(out uIso, out vIso);
            }

            uIso = global::Kerf.Engine.Geometry.ElCLib.InPeriod(uIso, 0.0, period);

            // Sought U-parameter in the apex point

            // For 2 possible parameters value, one will be chosen which is nearer to aUIso.
            double minDelta = RealLast();
            for (int idx = 0; idx < nbTangent; idx++)
            {
                // Vector {cos(a), sin(a)}
                Vector2D vecCS = new(tgILine[idx].X(), tgILine[idx].Y());
                double sqMod = vecCS.SquareMagnitude;
                if (sqMod < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                {
                    isIsoChoosen = true;
                    break;
                }

                // Normalize
                vecCS = vecCS.Divided(Math.Sqrt(sqMod));

                // Angle in range [0, PI/2]
                double uq = (Math.Abs(vecCS.X) < Math.Abs(vecCS.Y)) ? Math.Acos(Math.Abs(vecCS.X))
                                                                            : Math.Asin(Math.Abs(vecCS.Y));

                // Convert angles to the range [0, 2*PI]
                if (vecCS.Y < 0.0)
                {
                    if (vecCS.X > 0.0)
                    {
                        uq = -uq;
                    }
                    else
                    {
                        uq += M_PI;
                    }
                }
                else if (vecCS.X < 0.0)
                {
                    uq = M_PI - uq;
                }

                // Select the parameter the nearest to aUIso
                uq = global::Kerf.Engine.Geometry.ElCLib.InPeriod(uq, 0.0, period);
                double delta = Math.Abs(uq - uIso);
                if (delta > M_PI)
                {
                    delta = period - delta;
                }

                if (delta < minDelta)
                {
                    minDelta = delta;
                    uquad = uq;
                }
            }
        }

        if (isIsoChoosen)
        {
            isIsoChoosen = true;

            double uIso, vIso;
            if (isReversed)
            {
                ptIso.ParametersOnS2(out uIso, out vIso);
            }
            else
            {
                ptIso.ParametersOnS1(out uIso, out vIso);
            }

            uquad = uIso;
            return true;
        }
        else
        {
            return true;
        }
    }

    /// <summary>
    /// The tangents (at most two, in <paramref name="result"/>) at the apex of a canonical cone of the
    /// intersection with the plane through the apex of normal <paramref name="plnNormal"/> (normalized). Returns
    /// their number.
    /// </summary>
    internal static int GetTangentToIntLineForCone(double coneSemiAngle, in Xyz plnNormal, Xyz[] result)
    {
        double nullTol = Epsilon(1.0);
        double tanA = Math.Tan(coneSemiAngle);
        double a = plnNormal.Z() / tanA - plnNormal.X();
        double b = plnNormal.Y();
        double c = plnNormal.Z() / tanA + plnNormal.X();

        if (Math.Abs(a) < nullTol)
        {
            if (Math.Abs(b) > nullTol)
            {
                // The plane goes along the cone generatrix.
                result[0] = GetTangent(coneSemiAngle, -c / (b + b));
                return 1;
            }

            // The cone and the plane have only one common point. It is the cone apex.
            return 0;
        }

        // Discriminant of this equation is equal to
        double discr = plnNormal.Z() / Math.Sin(coneSemiAngle);
        discr = 1.0 - discr * discr;

        if (Math.Abs(discr) < nullTol)
        {
            // The plane goes along the cone generatrix. Mathematically, this condition is equivalent to the one above
            // (|aA| < aNullTol and |aB| > aNullTol), but is separated for numerical stability.
            result[0] = GetTangent(coneSemiAngle, -b / a);
            return 1;
        }
        else if (discr > 0.0)
        {
            double rd = Math.Sqrt(discr);
            result[0] = GetTangent(coneSemiAngle, (-b + rd) / a);
            result[1] = GetTangent(coneSemiAngle, (-b - rd) / a);
            return 2;
        }

        // We will never come here.
        return 0;
    }

    /// <summary>
    /// Computes the pole of a sphere or the apex of a cone (<paramref name="qSurf"/>) as an intersection point with
    /// <paramref name="pSurf"/>, in <paramref name="addedPoint"/> (not added to any line).
    /// <paramref name="ptIso"/> gives the isoline on which the pole is placed; <paramref name="vertex"/> must be
    /// initialized (it may be updated). If <paramref name="isReqRefCheck"/>, the vertex must be the pole. Throws
    /// Standard_TypeMismatch for other quadrics.
    /// </summary>
    public static bool AddSingularPole(SurfaceAdaptorBase qSurf, SurfaceAdaptorBase pSurf,
                                       in PntOn2S ptIso, ref PatchPoint vertex,
                                       ref PntOn2S addedPoint, bool isReversed = false,
                                       bool isReqRefCheck = false)
    {
        // On parametric
        double u0, v0;
        // aPQuad is Pole
        Point3D p0 = new();
        double uquad, vquad;
        if (isReversed)
        {
            vertex.Parameters(out u0, out v0, out uquad, out vquad);
        }
        else
        {
            vertex.Parameters(out uquad, out vquad, out u0, out v0);
        }

        uquad = 0.0;

        if (qSurf.GetType() == SurfaceKind.Sphere)
        {
            vquad = Math.CopySign(M_PI_2, vquad);
        }
        else if (qSurf.GetType() == SurfaceKind.Cone)
        {
            Cone3D co = qSurf.Cone();
            double radius = co.RefRadius;
            double semiAngle = co.SemiAngle;
            vquad = -radius / Math.Sin(semiAngle);
        }
        else
        {
            throw new TypeMismatchException("IntPatch_SpecialPoints::AddSingularPole(),Unsupported quadric with Pole");
        }

        qSurf.D0(uquad, vquad, out Point3D pQuad);
        double tol = vertex.Tolerance();
        if (isReqRefCheck && (pQuad.SquareDistanceTo(vertex.Value()) >= tol * tol))
        {
            return false;
        }

        if (!IsPointOnSurface(pSurf, pQuad, tol, ref p0, ref u0, ref v0))
        {
            return false;
        }

        // Pole is an intersection point (lies in the quadric and the parametric surface)

        if (isReversed)
        {
            addedPoint.SetValue(new Point3D(0.5 * (p0.XYZ() + pQuad.XYZ())), u0, v0, uquad, vquad);
        }
        else
        {
            addedPoint.SetValue(new Point3D(0.5 * (p0.XYZ() + pQuad.XYZ())), uquad, vquad, u0, v0);
        }

        bool isSame = addedPoint.IsSame(vertex.PntOn2S(), global::Kerf.Engine.Foundation.Precision.Confusion());

        // Found pole does not exist in the Walking-line. It must be added there (with correct 2D-parameters).

        // 2D-parameters of thePSurf surface have already been found (aU0, aV0). Let find 2D-parameters on the quadric.
        // The algorithm depends on the type of the quadric (sphere or cone). The derivatives of thePSurf are
        // transformed into the coordinate system of the quadric.

        pSurf.D1(u0, v0, out _, out Vector3D vecDu, out Vector3D vecDv);

        // Transforms parametric surface in coordinate-system of the quadric
        Transform3D tr = new();
        tr = tr.WithTransformation((qSurf.GetType() == SurfaceKind.Sphere) ? qSurf.Sphere().Position
                                                                      : qSurf.Cone().Position);

        // Derivatives of transformed thePSurf
        vecDu = vecDu.Transformed(tr);
        vecDv = vecDv.Transformed(tr);

        bool isIsoChoosen;

        if (qSurf.GetType() == SurfaceKind.Sphere)
        {
            if (!ProcessSphere(ptIso, vecDu, vecDv, isReversed, vquad, ref uquad, out isIsoChoosen))
            {
                return false;
            }
        }
        else // if(theQSurf->GetType() == GeomAbs_Cone)
        {
            if (!ProcessCone(ptIso, vecDu, vecDv, qSurf.Cone(), isReversed, ref uquad, out isIsoChoosen))
            {
                return false;
            }
        }

        if (isReversed)
        {
            addedPoint.SetValue(new Point3D(0.5 * (p0.XYZ() + pQuad.XYZ())), u0, v0, uquad, vquad);
        }
        else
        {
            addedPoint.SetValue(new Point3D(0.5 * (p0.XYZ() + pQuad.XYZ())), uquad, vquad, u0, v0);
        }

        if (isSame)
        {
            vertex.SetValue(addedPoint);
            return true;
        }

        if (!isIsoChoosen)
        {
            Span<double> arrOfPeriod = stackalloc double[4];
            if (isReversed)
            {
                global::Kerf.Engine.Intersection.Walking.IntSurf.SetPeriod(pSurf, qSurf, arrOfPeriod);
            }
            else
            {
                global::Kerf.Engine.Intersection.Walking.IntSurf.SetPeriod(qSurf, pSurf, arrOfPeriod);
            }

            AdjustPointAndVertex(vertex.PntOn2S(), arrOfPeriod, ref addedPoint);
        }
        else
        {
            vertex.SetValue(addedPoint);
        }

        return true;
    }

    /// <summary>
    /// After a special point added to a line: the point (<paramref name="newPoint"/>, in and out) with which the
    /// line continues or a new line starts, adjusted to <paramref name="refPt"/>.
    /// </summary>
    public static bool ContinueAfterSpecialPoint(SurfaceAdaptorBase qSurf, SurfaceAdaptorBase pSurf,
                                                 in PntOn2S refPt, SpecPntType spType,
                                                 double tol2D, ref PntOn2S newPoint,
                                                 bool isReversed = false)
    {
        if (spType == SpecPntType.SPntNone)
        {
            return false;
        }

        if (newPoint.IsSame(refPt, global::Kerf.Engine.Foundation.Precision.Confusion(), tol2D))
        {
            return false;
        }

        if ((spType == SpecPntType.SPntPole) && (qSurf.GetType() == SurfaceKind.Cone))
        {
            // Check if the condition b) is satisfied. Repeat the same steps as in AddSingularPole.

            // On parametric
            double u0, v0;
            // On quadric
            double uquad, vquad;

            if (isReversed)
            {
                newPoint.Parameters(out u0, out v0, out uquad, out vquad);
            }
            else
            {
                newPoint.Parameters(out uquad, out vquad, out u0, out v0);
            }

            pSurf.D1(u0, v0, out _, out Vector3D vecDu, out Vector3D vecDv);

            // Transforms parametric surface in coordinate-system of the quadric
            Transform3D tr = new Transform3D().WithTransformation(qSurf.Cone().Position);

            // Derivatives of transformed thePSurf
            vecDu = vecDu.Transformed(tr);
            vecDv = vecDv.Transformed(tr);

            ProcessCone(refPt, vecDu, vecDv, qSurf.Cone(), isReversed, ref uquad, out _);

            newPoint.SetValue(!isReversed, uquad, vquad);
        }

        // In case of going through the Pole/Apex, U-parameter of the quadric surface will change to +/-PI, with
        // exceptions (C0 second surface; huge curvature): so the U-parameter of the quadric is adjusted with period
        // PI/2 instead of 2PI, forbidding "jumps" between two neighbor Walking-points greater than PI/4.

        double period = (spType == SpecPntType.SPntPole) ? M_PI_2 : 2.0 * M_PI;

        double upPeriod = pSurf.IsUPeriodic ? pSurf.UPeriod() : 0.0;
        double uqPeriod = qSurf.IsUPeriodic ? period : 0.0;
        double vpPeriod = pSurf.IsVPeriodic ? pSurf.VPeriod() : 0.0;
        double vqPeriod = qSurf.IsVPeriodic ? period : 0.0;

        Span<double> arrOfPeriod =
        [
            isReversed ? upPeriod : uqPeriod,
            isReversed ? vpPeriod : vqPeriod,
            isReversed ? uqPeriod : upPeriod,
            isReversed ? vqPeriod : vpPeriod,
        ];

        AdjustPointAndVertex(refPt, arrOfPeriod, ref newPoint);
        return true;
    }

    /// <summary>
    /// Sets <paramref name="newPoint"/>'s parameters closest to <paramref name="refPoint"/>'s by adding or
    /// subtracting the periods (U and V of the first surface, then of the second; 0 where not periodic).
    /// </summary>
    public static void AdjustPointAndVertex(in PntOn2S refPoint, ReadOnlySpan<double> arrPeriods,
                                            ref PntOn2S newPoint)
    {
        Adjust(refPoint, arrPeriods, ref newPoint, out _);
    }

    /// <summary>
    /// As <see cref="AdjustPointAndVertex(in PntOn2S, ReadOnlySpan{double}, ref PntOn2S)"/>, and sets
    /// <paramref name="vertex"/>'s parameters as the new point's (OCCT's non-null vertex pointer).
    /// </summary>
    public static void AdjustPointAndVertex(in PntOn2S refPoint, ReadOnlySpan<double> arrPeriods,
                                            ref PntOn2S newPoint, ref PatchPoint vertex)
    {
        Adjust(refPoint, arrPeriods, ref newPoint, out (double, double, double, double) par);
        vertex.SetParameters(par.Item1, par.Item2, par.Item3, par.Item4);
    }

    private static void Adjust(in PntOn2S refPoint, ReadOnlySpan<double> arrPeriods,
                               ref PntOn2S newPoint, out (double, double, double, double) theParams)
    {
        Span<double> refPar = [0.0, 0.0];
        Span<double> par = [0.0, 0.0, 0.0, 0.0];
        newPoint.Parameters(out par[0], out par[1], out par[2], out par[3]);

        for (int i = 0; i < 4; i++)
        {
            if (arrPeriods[i] == 0)
            {
                continue;
            }

            double period = arrPeriods[i], halfPeriod = 0.5 * arrPeriods[i];

            if (i < 2)
            { // 1st surface is used
                refPoint.ParametersOnS1(out refPar[0], out refPar[1]);
            }
            else
            {
                refPoint.ParametersOnS2(out refPar[0], out refPar[1]);
            }

            int refInd = i % 2;

            {
                double deltaPar = refPar[refInd] - par[i];
                double incr = Math.CopySign(period, deltaPar);
                while ((deltaPar > halfPeriod) || (deltaPar < -halfPeriod))
                {
                    par[i] += incr;
                    deltaPar = refPar[refInd] - par[i];
                }
            }
        }

        theParams = (par[0], par[1], par[2], par[3]);
        newPoint.SetValue(par[0], par[1], par[2], par[3]);
    }
}
