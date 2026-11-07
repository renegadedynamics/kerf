// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_BSplineRestriction.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_BSplineRestriction.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_BSplineRestriction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Healing.Custom;

/// <summary>
/// The modification that approximates surfaces, curves and pcurves with a given degree, number of spans and
/// tolerances (3D and 2D), as B-splines; the continuity asked for is lowered where the approximation fails with it.
/// <see cref="RestrictionParameters"/> says which kinds of geometry to convert.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21). Its errors and number of spans accumulate over the faces and edges it converts,
/// and, as in OCCT, <see cref="NewPoint"/> answers from whether anything was converted before it is called, so a
/// modification reused (a compound's children, <see cref="Customize.ApplyModifier"/>) gives new points to later
/// children's vertices.
/// </para>
/// <para>
/// It writes only into the modifier's new edges (<see cref="NewCurve2d"/>'s range on <c>NewE</c>, never the
/// caller's edge) and into the B-splines its approximations create (decision 973).
/// </para>
/// </remarks>
internal class BSplineRestriction : CustomModification
{
    private Continuity _continuity3d;
    private Continuity _continuity2d;
    private int _maxDegree;
    private int _nbMaxSeg;
    private double _tol3d;
    private double _tol2d;
    private double _surfaceError;
    private double _curve3dError;
    private double _curve2dError;
    private int _nbOfSpan;
    private bool _approxSurfaceFlag;
    private bool _approxCurve3dFlag;
    private bool _approxCurve2dFlag;
    private bool _deg;
    private bool _convert;
    private bool _rational;
    private RestrictionParameters _parameters;

    // OCC_CATCH_SIGNALS: OCCT catches Standard_Failure, and access violations as failures (decision 917); never a write
    // into a frozen shape or geometry (decision 423)
    private static bool IsCaughtFailure(Exception exception)
        => exception is not ImmutableObjectException
           && (Refusal.IsFailure(exception) || exception is NullReferenceException);

    private static Continuity IntegerToGeomAbsShape(int i)
    {
        Continuity result = C0;
        switch (i)
        {
            case 0:
                result = C0;
                break;
            case 1:
                result = C1;
                break;
            case 2:
                result = C2;
                break;
            case 3:
                result = C3;
                break;
            default:
                result = CN;
                break;
        }
        return result;
    }

    private static int ContToInteger(Continuity cont)
    {
        int result = 0;
        switch (cont)
        {
            case C0:
            case G1:
                result = 0;
                break;
            case C1:
            case G2:
                result = 1;
                break;
            case C2:
                result = 2;
                break;
            case C3:
                result = 3;
                break;
            default:
                result = 4;
                break;
        }
        return result;
    }

    private static bool IsConvertCurve3d(Curve3D? aCurve,
                                         int degree,
                                         int nbSeg,
                                         bool myRational,
                                         RestrictionParameters aParameters)
    {
        if (aCurve is null)
        {
            return false;
        }
        if (aParameters.ConvertCurve3d())
        {
            return true;
        }
        if (aCurve is TrimmedCurve3D tmp)
        {
            Curve3D basCurve = tmp.BasisCurve;
            return IsConvertCurve3d(basCurve, degree, nbSeg, myRational, aParameters);
        }

        if (aCurve is OffsetCurve3D tmpO)
        {
            if (aParameters.ConvertOffsetCurv3d())
            {
                return true;
            }
            Curve3D basCurve = tmpO.BasisCurve;
            return IsConvertCurve3d(basCurve, degree, nbSeg, myRational, aParameters);
        }
        if (aCurve is BSplineCurve3D bsC)
        {
            // OCCT: '>=' here, '>' for 2D curves (occt-issues.md)
            if (bsC.Degree > degree || ((bsC.KnotCount - 1) >= nbSeg))
            {
                return true;
            }
            return myRational && bsC.IsRational;
        }
        if (aCurve is BezierCurve3D bz
            && (bz.Degree > degree || (myRational && bz.IsRational)))
        {
            return true;
        }
        // else return false;
        return false;
    }

    private static bool IsConvertSurface(Surface? aSurface,
                                         int degree,
                                         int nbSeg,
                                         bool myRational,
                                         RestrictionParameters aParameters)
    {
        if (aSurface is null)
        {
            return false;
        }
        if (aSurface is PlaneSurface)
        {
            return aParameters.ConvertPlane();
        }
        else if (aSurface is ConicalSurface)
        {
            return aParameters.ConvertConicalSurf();
        }
        else if (aSurface is SphericalSurface)
        {
            return aParameters.ConvertSphericalSurf();
        }
        else if (aSurface is CylindricalSurface)
        {
            return aParameters.ConvertCylindricalSurf();
        }
        else if (aSurface is ToroidalSurface)
        {
            return aParameters.ConvertToroidalSurf();
        }

        // else if(aSurface->IsKind(STANDARD_TYPE(Geom_ElementarySurface)))    {
        //   return aParameters->ConvertElementarySurf();
        //  }
        if (aSurface is SweptSurface swept)
        {
            if (aSurface is RevolutionSurface && aParameters.ConvertRevolutionSurf())
            {
                return true;
            }
            if (aSurface is ExtrusionSurface && aParameters.ConvertExtrusionSurf())
            {
                return true;
            }
            Curve3D basCurve = swept.BasisCurve;
            return IsConvertCurve3d(basCurve, degree, nbSeg, myRational, aParameters);
        }
        if (aSurface is TrimmedSurface rts)
        {
            Surface theSurf = rts.BasisSurface;
            return IsConvertSurface(theSurf, degree, nbSeg, myRational, aParameters);
        }
        if (aSurface is OffsetSurface os)
        {
            if (aParameters.ConvertOffsetSurf())
            {
                return true;
            }
            Surface theSurf = os.BasisSurface;
            return IsConvertSurface(theSurf, degree, nbSeg, myRational, aParameters);
        }
        if (aSurface is BSplineSurface theBSurf)
        {
            if (theBSurf.UDegree > degree || theBSurf.VDegree > degree)
            {
                return true;
            }
            if ((theBSurf.UKnotCount - 1) * (theBSurf.VKnotCount - 1) > nbSeg)
            {
                return true;
            }
            if (myRational && (theBSurf.IsURational || theBSurf.IsVRational))
            {
                return true;
            }
            return false;
        }

        if (aSurface is BezierSurface theBzSurf)
        {
            if (aParameters.ConvertBezierSurf())
            {
                return true;
            }
            if (theBzSurf.UDegree > degree || theBzSurf.VDegree > degree)
            {
                return true;
            }
            if (myRational && (theBzSurf.IsURational || theBzSurf.IsVRational))
            {
                return true;
            }
            return false;
        }
        return false;
    }

    private static bool IsConvertCurve2d(Curve2D? aCurve,
                                         int degree,
                                         int nbSeg,
                                         bool myRational,
                                         RestrictionParameters aParameters)
    {
        if (aCurve is null)
        {
            return false;
        }
        if (aParameters.ConvertCurve2d())
        {
            return true;
        }
        if (aCurve is TrimmedCurve2D tmp)
        {
            Curve2D basCurve = tmp.BasisCurve;
            return IsConvertCurve2d(basCurve, degree, nbSeg, myRational, aParameters);
        }
        if (aCurve is OffsetCurve2D tmpO)
        {
            if (aParameters.ConvertOffsetCurv2d())
            {
                return true;
            }
            Curve2D basCurve = tmpO.BasisCurve;
            return IsConvertCurve2d(basCurve, degree, nbSeg, myRational, aParameters);
        }
        if (aCurve is BSplineCurve2D bs
            && ((bs.Degree > degree || ((bs.KnotCount - 1) > nbSeg))
                || (myRational && bs.IsRational)))
        {
            return true;
        }
        if (aCurve is BezierCurve2D bz
            && ((bz.Degree > degree) || (myRational && bz.IsRational)))
        {
            return true;
        }
        // else return false;
        return false;
    }

    /// <summary>
    /// A modification approximating surfaces, 3D curves and pcurves: tolerances 0.01 (3D) and 1e-6 (2D), C1 (3D) and
    /// C2 (2D), degree up to 9, up to 10000 spans, the degree first, rational geometry not converted.
    /// </summary>
    public BSplineRestriction()
    {
        _approxSurfaceFlag = true;
        _approxCurve3dFlag = true;
        _approxCurve2dFlag = true;
        _tol3d = 0.01;
        _tol2d = 1E-6;
        _continuity3d = C1;
        _continuity2d = C2;
        _maxDegree = 9;
        _nbMaxSeg = 10000;
        _surfaceError = global::Kerf.Engine.Foundation.Precision.Confusion();
        _curve3dError = global::Kerf.Engine.Foundation.Precision.Confusion();
        _curve2dError = global::Kerf.Engine.Foundation.Precision.PConfusion();
        _nbOfSpan = 0;
        _convert = false;
        _deg = true;
        _rational = false;
        _parameters = new RestrictionParameters();
    }

    /// <summary>A modification with the given parameters of approximation and the default restriction parameters.</summary>
    public BSplineRestriction(bool anApproxSurfaceFlag,
                                          bool anApproxCurve3dFlag,
                                          bool anApproxCurve2dFlag,
                                          double aTol3d,
                                          double aTol2d,
                                          Continuity aContinuity3d,
                                          Continuity aContinuity2d,
                                          int aMaxDegree,
                                          int aNbMaxSeg,
                                          bool deg,
                                          bool rational)
    {
        _approxSurfaceFlag = anApproxSurfaceFlag;
        _approxCurve3dFlag = anApproxCurve3dFlag;
        _approxCurve2dFlag = anApproxCurve2dFlag;
        _tol3d = aTol3d;
        _tol2d = aTol2d;
        _maxDegree = aMaxDegree;
        _continuity3d = aContinuity3d;
        _continuity2d = aContinuity2d;
        // Kerf: 0 or below would double for ever, looking for a surface that fits (decision 22501)
        Refusal.ArgumentIf(aNbMaxSeg <= 0, FailureKind.ConstructionError, "ShapeCustom_BSplineRestriction: the most segments must be above 0");
        _nbMaxSeg = aNbMaxSeg;
        _surfaceError = global::Kerf.Engine.Foundation.Precision.Confusion();
        _curve3dError = global::Kerf.Engine.Foundation.Precision.Confusion();
        _curve2dError = global::Kerf.Engine.Foundation.Precision.PConfusion();
        _nbOfSpan = 0;
        _convert = false;
        _deg = deg;
        _rational = rational;
        _parameters = new RestrictionParameters();
    }

    /// <summary>A modification with the given parameters of approximation and restriction parameters.</summary>
    public BSplineRestriction(bool anApproxSurfaceFlag,
                                          bool anApproxCurve3dFlag,
                                          bool anApproxCurve2dFlag,
                                          double aTol3d,
                                          double aTol2d,
                                          Continuity aContinuity3d,
                                          Continuity aContinuity2d,
                                          int aMaxDegree,
                                          int aNbMaxSeg,
                                          bool deg,
                                          bool rational,
                                          RestrictionParameters aModes)
    {
        _approxSurfaceFlag = anApproxSurfaceFlag;
        _approxCurve3dFlag = anApproxCurve3dFlag;
        _approxCurve2dFlag = anApproxCurve2dFlag;
        _tol3d = aTol3d;
        _tol2d = aTol2d;
        _maxDegree = aMaxDegree;
        _continuity3d = aContinuity3d;
        _continuity2d = aContinuity2d;
        // Kerf: 0 or below would double for ever, looking for a surface that fits (decision 22501)
        Refusal.ArgumentIf(aNbMaxSeg <= 0, FailureKind.ConstructionError, "ShapeCustom_BSplineRestriction: the most segments must be above 0");
        _nbMaxSeg = aNbMaxSeg;
        _surfaceError = global::Kerf.Engine.Foundation.Precision.Confusion();
        _curve3dError = global::Kerf.Engine.Foundation.Precision.Confusion();
        _curve2dError = global::Kerf.Engine.Foundation.Precision.PConfusion();
        _nbOfSpan = 0;
        _convert = false;
        _deg = deg;
        _rational = rational;
        _parameters = aModes;
    }

    /// <summary>
    /// True if the face's surface is approximated: then S is the approximation (over the face's UV bounds in the
    /// segment mode, or where the surface is infinite).
    /// </summary>
    public override bool NewSurface(in Face f,
                                    out Surface? s,
                                    out Placement l,
                                    out double tol,
                                    out bool revWires,
                                    out bool revFace)
    {
        s = null;
        tol = 0.0;
        revWires = revFace = false;
        if (!_approxSurfaceFlag)
        {
            l = new Placement();
            return false;
        }
        revWires = false;
        revFace = false;
        _convert = false;
        Surface? surface = Tool.Surface(f, out l);
        if (surface is null)
        {
            return false;
        }
        bool isOf = true;
        if (_parameters.ConvertOffsetSurf())
        {
            isOf = false;
        }
        surface.ParameterBounds(out double uf, out double ul, out double vf, out double vl);
        BrepTools.UVBounds(f, out double umin, out double umax, out double vmin, out double vmax);
        if (_parameters.SegmentSurfaceMode())
        {
            uf = umin;
            ul = umax;
            vf = vmin;
            vl = vmax;
        }
        else
        {
            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(uf) || global::Kerf.Engine.Foundation.Precision.IsInfinite(ul))
            {
                uf = umin;
                ul = umax;
            }
            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(vf) || global::Kerf.Engine.Foundation.Precision.IsInfinite(vl))
            {
                vf = vmin;
                vl = vmax;
            }
        }

        bool isConv = ConvertSurface(surface, ref s, uf, ul, vf, vl, isOf);
        tol = global::Kerf.Engine.Foundation.Precision.Confusion(); // mySurfaceError;

        if (isConv)
        {
            bool wasBSpline = surface is BSplineSurface;
            if (surface is TrimmedSurface rts)
            {
                wasBSpline = rts.BasisSurface is BSplineSurface;
            }

            if (wasBSpline)
            {
                SendMsg(f, new Msg("BSplineRestriction.NewSurface.MSG1"));
            }
            else
            {
                SendMsg(f, new Msg("BSplineRestriction.NewSurface.MSG0"));
            }
        }

        return isConv;
    }

    private static void ConvertExtrusion(Curve3D? c, /*const gp_Dir& direction,*/
                                         in Transform3D shiftF,
                                         in Transform3D shiftL,
                                         double vf,
                                         double vl,
                                         out Surface bspline)
    {
        // OCCT: dereferences C, null when the approximation of the basis curve failed (an access violation natively)
        BSplineCurve3D bspl = (BSplineCurve3D)c!;
        int nbPoles = bspl.PoleCount;
        NCollection_Array1<Point3D> poles = bspl.InternalPoles();
        NCollection_Array1<double> weights = bspl.InternalWeightsArray();
        NCollection_Array1<double> knots = bspl.InternalKnots();
        NCollection_Array1<int> mults = bspl.InternalMultiplicities();

        NCollection_Array2<Point3D> resPoles = new(1, nbPoles, 1, 2);
        NCollection_Array2<double> resWeigth = new(1, nbPoles, 1, 2);
        for (int j = 1; j <= nbPoles; j++)
        {
            resPoles[j, 1] = poles[j].Transformed(shiftF);
            resPoles[j, 2] = poles[j].Transformed(shiftL);
            resWeigth[j, 1] = weights[j];
            resWeigth[j, 2] = weights[j];
        }

        NCollection_Array1<double> vknots = new(1, 2);
        NCollection_Array1<int> vmults = new(1, 2);
        vknots[1] = vf;
        vknots[2] = vl;
        vmults[1] = vmults[2] = 2;

        bspline = new BSplineSurface(resPoles,
                                          resWeigth,
                                          knots,
                                          vknots,
                                          mults,
                                          vmults,
                                          bspl.Degree,
                                          1,
                                          bspl.IsPeriodic,
                                          false);
    }

    /// <summary>
    /// True if the surface is converted: then S is the result (left unchanged otherwise). With <paramref name="isOf"/>,
    /// offset surfaces are approximated as offsets of approximated bases, else as B-splines.
    /// </summary>
    public bool ConvertSurface(Surface aSurface,
                               ref Surface? s,
                               double uf,
                               double ul,
                               double vf,
                               double vl,
                               bool isOf = true)
    {
        if (!IsConvertSurface(aSurface, _maxDegree, _nbMaxSeg, _rational, _parameters))
        {
            return false;
        }

        Surface surf = aSurface;
        if (surf is PlaneSurface pln && _parameters.ConvertPlane())
        {
            NCollection_Array2<Point3D> poles = new(1, 2, 1, 2);
            NCollection_Array1<double> uknots = new(1, 2);
            NCollection_Array1<int> umults = new(1, 2);
            NCollection_Array1<double> vknots = new(1, 2);
            NCollection_Array1<int> vmults = new(1, 2);

            poles[1, 1] = pln.Value(uf, vf);
            poles[1, 2] = pln.Value(uf, vl);
            poles[2, 1] = pln.Value(ul, vf);
            poles[2, 2] = pln.Value(ul, vl);
            uknots[1] = uf;
            uknots[2] = ul;
            vknots[1] = vf;
            vknots[2] = vl;
            umults[1] = umults[2] = vmults[1] = vmults[2] = 2;
            s = new BSplineSurface(poles, uknots, vknots, umults, vmults, 1, 1, false, false);
            return true;
        }
        if (surf is RevolutionSurface surfaceR)
        {
            Curve3D basCurve = surfaceR.BasisCurve;
            Curve3D? resCurve = null;
            double tolS = global::Kerf.Engine.Foundation.Precision.Confusion();
            if (_parameters.ConvertRevolutionSurf())
            {
                if (basCurve is OffsetCurve3D)
                {
                    Continuity cnt = basCurve.Continuity;
                    cnt = (cnt > C2 ? C2 : cnt);
                    if (ConvertCurve(basCurve,
                                     ref resCurve,
                                     false,
                                     Max(vf, basCurve.FirstParameter),
                                     Min(vl, basCurve.LastParameter),
                                     ref tolS,
                                     false))
                    {
                        RevolutionSurface newRevol = new RevolutionSurface(resCurve!, surfaceR.Axis);
                        surf = newRevol;
                    }
                }
            }
            else
            {
                if (ConvertCurve(basCurve,
                                 ref resCurve,
                                 false,
                                 Max(vf, basCurve.FirstParameter),
                                 Min(vl, basCurve.LastParameter),
                                 ref tolS,
                                 isOf))
                {
                    s = new RevolutionSurface(resCurve!, surfaceR.Axis);
                    return true;
                }
                else
                {
                    return false;
                }
            }
        }
        if (surf is ExtrusionSurface surfaceE)
        {
            Curve3D? basCurve = surfaceE.BasisCurve;
            Curve3D? resCurve = null;
            double tolS = global::Kerf.Engine.Foundation.Precision.Confusion();
            if (_parameters.ConvertExtrusionSurf())
            {
                Continuity cnt = surfaceE.Continuity;
                cnt = (cnt > C2 ? C2 : cnt);
                BSplineCurve3D? bspl =
                    global::Kerf.Engine.Healing.Construct.ShapeConstruct.InternalConvertCurveToBSpline(basCurve, uf, ul, tolS, cnt, _nbMaxSeg, _maxDegree);
                basCurve = bspl;
                // OCCT: dereferences BasCurve, null when the conversion failed (an access violation natively)
                ConvertCurve(basCurve!,
                             ref resCurve,
                             true,
                             Max(uf, basCurve!.FirstParameter),
                             Min(ul, basCurve.LastParameter),
                             ref tolS,
                             isOf);
                Transform3D shiftF = new Transform3D(), shiftL = new Transform3D();
                shiftF = shiftF.WithTranslation(surfaceE.Value(uf, 0), surfaceE.Value(uf, vf));
                shiftL = shiftL.WithTranslation(surfaceE.Value(uf, 0), surfaceE.Value(uf, vl));
                ConvertExtrusion(resCurve, /*Surface->Direction(),*/ shiftF, shiftL, vf, vl, out Surface bSpline);
                s = bSpline;
                return true;
            }
            else
            {
                if (ConvertCurve(basCurve,
                                 ref resCurve,
                                 false,
                                 Max(uf, basCurve.FirstParameter),
                                 Min(ul, basCurve.LastParameter),
                                 ref tolS,
                                 isOf))
                {
                    s = new ExtrusionSurface(resCurve!, surfaceE.Direction);
                    return true;
                }
                else
                {
                    return false;
                }
            }
        }
        if (surf is TrimmedSurface tmpR)
        {
            tmpR.ParameterBounds(out double U1, out double U2, out double V1, out double V2);
            Surface theSurf = tmpR.BasisSurface;
            Surface? resSurface = null;
            if (ConvertSurface(theSurf, ref resSurface, U1, U2, V1, V2, isOf))
            {
                // S = new Geom_RectangularTrimmedSurface(ResSurface,U1,U2,V1,V2);
                s = resSurface;
                return true;
            }
            else
            {
                return false;
            }
        }
        if (surf is OffsetSurface tmpO && isOf)
        {
            Surface theSurf = tmpO.BasisSurface;
            Surface? resSurface = null;
            if (ConvertSurface(theSurf, ref resSurface, uf, ul, vf, vl))
            {
                if (resSurface!.Continuity != C0)
                {
                    s = new OffsetSurface(resSurface, tmpO.Offset);
                    return true;
                }
                else if (ConvertSurface(surf, ref s, uf, ul, vf, vl, false))
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }
            else
            {
                return false;
            }
        }
        if (surf is BezierSurface bezier && _parameters.ConvertBezierSurf())
        {
            int uDegree = bezier.UDegree;
            int vDegree = bezier.VDegree;
            NCollection_Array2<Point3D> aPoles = bezier.InternalPoles();
            NCollection_Array2<double> weights = bezier.InternalWeightsArray();
            NCollection_Array1<double> uKnots = new(1, 2), vKnots = new(1, 2);
            uKnots[1] = 0;
            uKnots[2] = 1;
            vKnots[1] = 0;
            vKnots[2] = 1;
            NCollection_Array1<int> uMults = new(1, 2), vMults = new(1, 2);
            uMults.Init(uDegree + 1);
            vMults.Init(vDegree + 1);
            BSplineSurface bspline =
                new BSplineSurface(aPoles, weights, uKnots, vKnots, uMults, vMults, uDegree, vDegree);

            if (!ConvertSurface(bspline, ref s, uf, ul, vf, vl, isOf))
            {
                s = bspline;
            }
            return true;
        }

        int nbSeg = 1;
        bool uRat = false;
        bool vRat = false;
        int uDeg = 1, vDeg = 1;
        if (surf is BSplineSurface bsS)
        {
            uDeg = bsS.UDegree;
            vDeg = bsS.VDegree;
            nbSeg = (bsS.UKnotCount - 1) * (bsS.VKnotCount - 1);
            uRat = bsS.IsURational;
            vRat = bsS.IsVRational;
            bool isR = (_rational && (uRat || vRat));
            if (uDeg <= _maxDegree && vDeg <= _maxDegree && nbSeg <= _nbMaxSeg && !isR)
            {
                return false;
            }
        }
        if (surf is BezierSurface bsZ)
        {
            uDeg = bsZ.UDegree;
            vDeg = bsZ.VDegree;
            nbSeg = 1;
            uRat = bsZ.IsURational;
            vRat = bsZ.IsVRational;
            bool isR = (_rational && (uRat || vRat));
            if (uDeg <= _maxDegree && vDeg <= _maxDegree && nbSeg <= _nbMaxSeg && !isR)
            {
                return false;
            }
        }
        Continuity cont = _continuity3d;
        if (surf is OffsetSurface)
        {
            cont = C0;
        }

        double tol3d;
        int nbOfSpan, imax = 10;
        int maxSeg = _nbMaxSeg;
        int maxDeg = _maxDegree;
        surf.ParameterBounds(out double u1, out double u2, out double v1, out double v2);
        double shiftU = 0, shiftV = 0;
        if (Math.Abs(u1 - uf) > global::Kerf.Engine.Foundation.Precision.PConfusion() || Math.Abs(u2 - ul) > global::Kerf.Engine.Foundation.Precision.PConfusion()
            || Math.Abs(v1 - vf) > global::Kerf.Engine.Foundation.Precision.PConfusion() || Math.Abs(v2 - vl) > global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            bool isTrim = false;
            if (!surf.IsUPeriodic)
            { // else {
                u1 = Max(u1, uf);
                u2 = Min(u2, ul);
                isTrim = true;
            }
            if (!surf.IsVPeriodic)
            { // else
                v1 = Max(v1, vf);
                v2 = Min(v2, vl);
                isTrim = true;
            }

            if (isTrim && (u1 != u2) && (v1 != v2))
            {
                TrimmedSurface trSurface = new TrimmedSurface(surf, u1, u2, v1, v2);
                trSurface.ParameterBounds(out double ur1, out double ur2, out double vr1, out double vr2);
                shiftU = u1 - ur1;
                shiftV = v1 - vr1;
                surf = trSurface;
            }
        }
        int cu = Min(ContToInteger(cont), ContToInteger(surf.Continuity));
        int cv = Min(ContToInteger(cont), ContToInteger(surf.Continuity));
        if (cu == 0)
        {
            cu = ContToInteger(cont);
        }
        if (cv == 0)
        {
            cv = ContToInteger(cont);
        }

        for (;;)
        {
            double prevTol = RealLast(), newTol = 0;
            for (int i = 1; i <= imax; i++)
            {
                tol3d = _tol3d * i / 2;
                while (cu >= 0 || cv >= 0)
                {
                    try
                    {
                        Continuity contV = IntegerToGeomAbsShape(cv);
                        Continuity contU = IntegerToGeomAbsShape(cu);

                        ApproxSurface approx = new ApproxSurface(surf, tol3d, contU, contV, maxDeg, maxDeg, maxSeg, 0);
                        bool done = approx.IsDone;
                        newTol = approx.MaxError();
                        if (approx.MaxError() <= _tol3d && done)
                        {
                            nbOfSpan = (approx.Surface()!.UKnotCount - 1) * (approx.Surface()!.VKnotCount - 1);
                            s = approx.Surface();
                            BSplineSurface bsc = (BSplineSurface)s!;
                            // Kerf: the approximation's surface is frozen (decision 197); OCCT changes it in place
                            // below, the port changes a copy (decision 974)
                            if (aSurface.IsUPeriodic || aSurface.IsVPeriodic || Math.Abs(shiftU) > global::Kerf.Engine.Foundation.Precision.PConfusion()
                                || Math.Abs(shiftV) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                            {
                                bsc = (BSplineSurface)bsc.Copy();
                                s = bsc;
                            }
                            if (aSurface.IsUPeriodic)
                            {
                                bsc.SetUPeriodic();
                            }
                            if (aSurface.IsVPeriodic)
                            {
                                bsc.SetVPeriodic();
                            }
                            _convert = true;
                            _nbOfSpan = _nbOfSpan + nbOfSpan;
                            _surfaceError = Max(_surfaceError, approx.MaxError());
                            if (Math.Abs(shiftU) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                            {
                                NCollection_Array1<double> uknots = new(bsc.InternalUKnots());
                                for (int j = 1; j <= uknots.Length(); j++)
                                {
                                    uknots[j] += shiftU;
                                }
                                bsc.SetUKnots(uknots);
                            }
                            if (Math.Abs(shiftV) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                            {
                                NCollection_Array1<double> vknots = new(bsc.InternalVKnots());
                                for (int j = 1; j <= vknots.Length(); j++)
                                {
                                    vknots[j] += shiftV;
                                }
                                bsc.SetVKnots(vknots);
                            }

                            return true;
                        }
                        else
                        {
                            break;
                        }
                    }
                    catch (Exception exception) when (IsCaughtFailure(exception))
                    {
                        // szv: protection against loop
                        if (cu == 0 && cv == 0)
                        {
                            break;
                        }
                        if (cu > 0)
                        {
                            cu--;
                        }
                        if (cv > 0)
                        {
                            cv--;
                        }
                    }
                }
                if (prevTol <= newTol)
                {
                    break;
                }
                else
                {
                    prevTol = newTol;
                }
            }
            // int GMaxDegree = 15;//Geom_BSplineSurface::MaxDegree();

            if (_deg)
            {
                if (maxSeg < _parameters.GMaxSeg())
                {
                    if (cv != 0 || cu != 0)
                    {
                        if (cv > 0)
                        {
                            cv--;
                        }
                        if (cu > 0)
                        {
                            cu--;
                        }
                    }
                    else
                    {
                        maxSeg = 2 * maxSeg; // myGMaxSeg;
                    }
                    if (maxSeg > _parameters.GMaxSeg())
                    {
                        maxSeg = _parameters.GMaxSeg();
                    }
                    else
                    {
                        continue;
                    }
                }
                else
                {
                    return false;
                }
            }
            else
            {
                if (maxDeg < _parameters.GMaxDegree())
                {
                    maxDeg = _parameters.GMaxDegree();
                    continue;
                }
                else
                {
                    return false;
                }
            }
        }
    }

    /// <summary>
    /// True if the edge's 3D curve is approximated, or (C then a copy of the curve) if a surface or pcurve of the
    /// edge is.
    /// </summary>
    public override bool NewCurve(in Edge e, out Curve3D? c, out Placement l, out double tol)
    {
        c = null;
        tol = 0.0;
        if (!_approxCurve3dFlag)
        {
            l = new Placement();
            return false;
        }
        Curve3D? curve = Tool.Curve(e, out l, out double first, out double last);
        double tolCur = Tool.Tolerance(e);
        // if(aCurve.IsNull()) return false;
        bool isConvert = false;
        BrepTEdge te = (BrepTEdge)e.TShape()!;
        // iterate on pcurves
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(te.Curves());
        for (; itcr.More(); itcr.Next())
        {
            if (itcr.Value() is not GCurve gc || !gc.IsCurveOnSurface())
            {
                continue;
            }
            Surface? surface = gc.Surface();
            Curve2D? curve2d = gc.PCurve();
            if ((_approxSurfaceFlag
                 && IsConvertSurface(surface, _maxDegree, _nbMaxSeg, _rational, _parameters))
                || (_approxCurve2dFlag
                    && IsConvertCurve2d(curve2d, _maxDegree, _nbMaxSeg, _rational, _parameters)))
            {
                isConvert = true;
                break;
            }
        }
        if (curve is null)
        {
            if (isConvert)
            {
                c = curve;
                tol = tolCur;
                return true;
            }
            else
            {
                return false;
            }
        }
        bool isOf = true;
        if (_parameters.ConvertOffsetCurv3d())
        {
            isOf = false;
        }
        bool isConv = ConvertCurve(curve, ref c, isConvert, first, last, ref tolCur, isOf);
        tol = Tool.Tolerance(e); // TolCur;

        if (isConv)
        {
            bool wasBSpline = curve is BSplineCurve3D;
            if (curve is TrimmedCurve3D tc)
            {
                wasBSpline = tc.BasisCurve is BSplineCurve3D;
            }

            if (wasBSpline)
            {
                SendMsg(e, new Msg("BSplineRestriction.NewCurve.MSG1"));
            }
            else
            {
                SendMsg(e, new Msg("BSplineRestriction.NewCurve.MSG0"));
            }
        }
        return isConv;
    }

    /// <summary>
    /// True if the curve is converted, or copied (<paramref name="isConvert"/>): then C is the result (left unchanged
    /// otherwise) and TolCur its error. With <paramref name="isOf"/>, offset curves are approximated as offsets of
    /// approximated bases, else as B-splines.
    /// </summary>
    public bool ConvertCurve(Curve3D aCurve,
                             ref Curve3D? c,
                             bool isConvert,
                             double first,
                             double last,
                             ref double tolCur,
                             bool isOf = true)
    {
        //  TolCur =  Precision::Confusion();
        if (aCurve is TrimmedCurve3D tmpT)
        {
            Curve3D basCurve = tmpT.BasisCurve;
            Curve3D? resCurve = null;
            if (ConvertCurve(basCurve, ref resCurve, isConvert, first, last, ref tolCur, isOf))
            {
                c = resCurve;
                return true;
            }
            else
            {
                if (isConvert)
                {
                    c = (Curve3D)aCurve.Copy();
                    tolCur = global::Kerf.Engine.Foundation.Precision.Confusion();
                    return true;
                }

                return false;
            }
        }

        if (aCurve is LineCurve3D line && _parameters.ConvertCurve3d())
        {
            NCollection_Array1<Point3D> poles = new(1, 2);
            poles[1] = line.Value(first);
            poles[2] = line.Value(last);
            NCollection_Array1<double> knots = new(1, 2);
            knots[1] = first;
            knots[2] = last;
            NCollection_Array1<int> mults = new(1, 2);
            mults.Init(2);
            BSplineCurve3D res = new BSplineCurve3D(poles, knots, mults, 1);
            c = res;
            return true;
        }

        if (aCurve is ConicCurve3D && _parameters.ConvertCurve3d())
        {
            BSplineCurve3D bSpline;
            Curve3D tcurve = new TrimmedCurve3D(aCurve, first, last); // protection against parabols ets
            ApproxCurve approx = new ApproxCurve(tcurve,
                                                                        _tol3d /*Precision::Approximation()*/,
                                                                        _continuity2d,
                                                                        _nbMaxSeg,
                                                                        6);
            if (approx.HasResult)
            {
                bSpline = approx.InternalCurve()!;
            }
            else
            {
                bSpline = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(tcurve, QuasiAngular);
            }

            double shift = first - bSpline.FirstParameter;
            if (Math.Abs(shift) > global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                NCollection_Array1<double> newKnots = new(bSpline.InternalKnots());
                for (int i = 1; i <= newKnots.Length(); i++)
                {
                    newKnots[i] += shift;
                }
                bSpline.SetKnots(newKnots);
            }
            Curve3D? resCurve = null;
            if (ConvertCurve(bSpline, ref resCurve, isConvert, first, last, ref tolCur, false))
            {
                c = resCurve;
                return true;
            }
            else
            {
                c = bSpline;
                tolCur = global::Kerf.Engine.Foundation.Precision.PConfusion();
                return true;
            }
        }

        if (aCurve is BezierCurve3D && _parameters.ConvertCurve3d())
        {
            Curve3D bSpline = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(aCurve, QuasiAngular);
            Curve3D? resCurve = null;
            if (ConvertCurve(bSpline, ref resCurve, isConvert, first, last, ref tolCur, false))
            {
                c = resCurve;
                return true;
            }
            else
            {
                c = bSpline;
                tolCur = global::Kerf.Engine.Foundation.Precision.PConfusion();
                return true;
            }
        }

        if (aCurve is OffsetCurve3D tmpO && isOf)
        {
            Curve3D basCurve = tmpO.BasisCurve;
            Curve3D? resCurve = null;
            if (ConvertCurve(basCurve, ref resCurve, isConvert, first, last, ref tolCur))
            {
                if (resCurve!.Continuity != C0)
                {
                    c = new OffsetCurve3D(resCurve, tmpO.Offset, tmpO.Direction);
                    return true;
                }
                else if (ConvertCurve(aCurve, ref c, isConvert, first, last, ref tolCur, false))
                {
                    return true;
                }
                else
                {
                    if (isConvert)
                    {
                        c = (Curve3D)aCurve.Copy();
                        tolCur = global::Kerf.Engine.Foundation.Precision.Confusion();
                        return true;
                    }
                    return false;
                }
            }
            else
            {
                if (isConvert)
                {
                    c = (Curve3D)aCurve.Copy();
                    tolCur = global::Kerf.Engine.Foundation.Precision.Confusion();
                    return true;
                }
                return false;
            }
        }
        if (aCurve is BSplineCurve3D
            || aCurve is BezierCurve3D
            || (aCurve is OffsetCurve3D && !isOf))
        {
            int deg = 1;

            if (aCurve is BSplineCurve3D bsC)
            {
                deg = bsC.Degree;
                bool isR = (_rational && bsC.IsRational);
                if (!isR && deg <= _maxDegree && (bsC.KnotCount - 1) <= _nbMaxSeg)
                {
                    if (isConvert)
                    {
                        c = (Curve3D)aCurve.Copy();
                        tolCur = global::Kerf.Engine.Foundation.Precision.Confusion();
                        return true;
                    }
                    else
                    {
                        return false;
                    }
                }
            }
            if (aCurve is BezierCurve3D bzC)
            {
                deg = bzC.Degree;
                bool isR = (_rational && bzC.IsRational);
                if (!isR && deg <= _maxDegree)
                {
                    if (isConvert)
                    {
                        c = (Curve3D)aCurve.Copy();
                        tolCur = global::Kerf.Engine.Foundation.Precision.Confusion();
                        return true;
                    }
                    else
                    {
                        return false;
                    }
                }
            }
            Curve3D curve1;
            double pf = aCurve.FirstParameter, pl = aCurve.LastParameter;
            // 15.11.2002 PTV OCC966
            if (CurveAnalysis.IsPeriodic(aCurve) && (first != last))
            {
                curve1 = new TrimmedCurve3D(aCurve, first, last);
            }
            else if (pf < (first - global::Kerf.Engine.Foundation.Precision.PConfusion()) || pl > (last + global::Kerf.Engine.Foundation.Precision.PConfusion()))
            {
                double f = Max(first, pf), l = Min(last, pl);
                if (f != l)
                {
                    curve1 = new TrimmedCurve3D(aCurve, f, l);
                }
                else
                {
                    curve1 = aCurve;
                }
            }
            else
            {
                curve1 = aCurve;
            }
            int aC = Min(ContToInteger(_continuity3d), ContToInteger(aCurve.Continuity));
            if (aC == 0)
            {
                aC = ContToInteger(_continuity3d);
            }
            // aC = std::min(aC,(Deg -1));
            int maxSeg = _nbMaxSeg;
            int maxDeg = _maxDegree;
            int c1 = aC;
            for (; aC >= 0; aC--)
            {
                try
                {
                    for (int j = 1; j <= 2; j++)
                    {
                        Continuity cont = IntegerToGeomAbsShape(aC);
                        ApproxCurve anApprox = new ApproxCurve(curve1, _tol3d, cont, maxSeg, maxDeg);
                        bool done = anApprox.IsDone;
                        c = anApprox.InternalCurve();
                        // OCCT: dereferences the curve, null when the approximation has no result (an access
                        // violation natively, caught as a failure: decision 917)
                        int nbseg = ((BSplineCurve3D)c!).KnotCount - 1;
                        int degC = ((BSplineCurve3D)c).Degree;
                        if (_deg
                            && ((degC > maxDeg) || !done || (anApprox.MaxError() >= Max(tolCur, _tol3d))))
                        {
                            if (maxSeg < _parameters.GMaxSeg())
                            {
                                maxSeg = _parameters.GMaxSeg();
                                aC = c1;
                                // OCCT: continues the inner loop (one retry), and a failed retry falls through to
                                // success below (occt-issues.md)
                                continue;
                            }
                            else
                            {
                                if (isConvert)
                                {
                                    c = (Curve3D)aCurve.Copy();
                                    tolCur = global::Kerf.Engine.Foundation.Precision.Confusion();
                                    return true;
                                }
                            }
                        }
                        if (!_deg
                            && ((nbseg > _parameters.GMaxSeg()) || !done
                                || (anApprox.MaxError() >= Max(tolCur, _tol3d))))
                        {
                            if (maxDeg < _parameters.GMaxDegree())
                            {
                                maxDeg = _parameters.GMaxDegree();
                                aC = c1;
                                continue;
                            }
                            else
                            {
                                if (isConvert)
                                {
                                    c = (Curve3D)aCurve.Copy();
                                    tolCur = global::Kerf.Engine.Foundation.Precision.Confusion();
                                    return true;
                                }
                            }
                        }
                        _convert = true;
                        tolCur = anApprox.MaxError();
                        _curve3dError = Max(_curve3dError, anApprox.MaxError());
                        return true;
                    }
                }
                catch (Exception exception) when (IsCaughtFailure(exception))
                {
                    continue;
                }
            }
            return false;
        }
        else
        {
            if (isConvert)
            {
                c = (Curve3D)aCurve.Copy();
                tolCur = global::Kerf.Engine.Foundation.Precision.Confusion();
                return true;
            }
            return false;
        }
    }

    /// <summary>
    /// True if the edge's pcurve on the face is approximated, or (C then a copy of the pcurve) if the face's surface or
    /// the edge's 3D curve or another pcurve is.
    /// </summary>
    public override bool NewCurve2d(in Edge e,
                                    in Face f,
                                    in Edge newE,
                                    in Face newF,
                                    out Curve2D? c,
                                    out double tol)
    {
        c = null;
        tol = 0.0;
        if (!_approxCurve2dFlag && !_approxSurfaceFlag)
        {
            return false;
        }
        Surface? surface = Tool.Surface(f, out Placement l);
        SurfaceAdaptor adS = new SurfaceAdaptor(surface!);
        double tolCur =
            Min(adS.UResolution(Tool.Tolerance(e)), adS.VResolution(Tool.Tolerance(e)));
        Curve2D? curve = Tool.CurveOnSurface(e, f, out double first, out double last);
        if (curve is null)
        {
            return false;
        }
        Curve3D? cur3d = Tool.Curve(e, out Placement loc1, out double f1, out double l1);
        //  bool IsConvert = (IsConvertSurface(aSurface,myMaxDegree,myNbMaxSeg) ||
        //  !E.IsSame(NewE));

        bool isConvert =
            ((_approxSurfaceFlag
              && IsConvertSurface(surface, _maxDegree, _nbMaxSeg, _rational, _parameters))
             || (_approxCurve3dFlag
                 && IsConvertCurve3d(cur3d, _maxDegree, _nbMaxSeg, _rational, _parameters)));

        if (!isConvert)
        {
            BrepTEdge te = (BrepTEdge)e.TShape()!;
            // iterate on pcurves
            var itcr = new NCollection_List<CurveRepresentation>.Iterator(te.Curves());
            for (; itcr.More(); itcr.Next())
            {
                if (itcr.Value() is not GCurve gc || !gc.IsCurveOnSurface())
                {
                    continue;
                }
                Surface? surf = gc.Surface();
                Curve2D? cur2d = gc.PCurve();
                if ((_approxSurfaceFlag
                     && IsConvertSurface(surf, _maxDegree, _nbMaxSeg, _rational, _parameters))
                    || (_approxCurve2dFlag
                        && IsConvertCurve2d(cur2d, _maxDegree, _nbMaxSeg, _rational, _parameters)))
                {
                    isConvert = true;
                    break;
                }
            }
        }
        if (!_approxCurve2dFlag)
        {
            if (isConvert)
            {
                c = (Curve2D)curve.Copy();
                return true;
            }
            else
            {
                return false;
            }
        }
        bool isOf = true;
        if (_parameters.ConvertOffsetCurv2d())
        {
            isOf = false;
        }
        bool isConv = ConvertCurve2d(curve, ref c, isConvert, first, last, ref tolCur, isOf);

        tol = Tool.Tolerance(e); // TolCur;
        BrepBuilder b = new BrepBuilder();
        if (!isConv && !newE.IsSame(e))
        {
            // Kerf: NewE is the modifier's new edge (or its dummy), never the caller's (decision 973)
            b.Range(newE, first, last);
        }
        return isConv;
    }

    /// <summary>
    /// True if the pcurve is converted, or copied (<paramref name="isConvert"/>): then C is the result (left
    /// unchanged otherwise) and TolCur its error. With <paramref name="isOf"/>, offset pcurves are approximated as
    /// offsets of approximated bases, else as B-splines.
    /// </summary>
    public bool ConvertCurve2d(Curve2D aCurve,
                               ref Curve2D? c,
                               bool isConvert,
                               double first,
                               double last,
                               ref double tolCur,
                               bool isOf = true)
    {
        // TolCur = Precision::PConfusion();
        if (aCurve is TrimmedCurve2D tmpT)
        {
            Curve2D basCurve = tmpT.BasisCurve;
            Curve2D? resCurve = null;
            if (ConvertCurve2d(basCurve, ref resCurve, isConvert, first, last, ref tolCur, isOf))
            {
                c = resCurve;
                return true;
            }
            else
            {
                if (isConvert)
                {
                    c = (Curve2D)aCurve.Copy();
                    tolCur = global::Kerf.Engine.Foundation.Precision.PConfusion();
                    return true;
                }
                else
                {
                    return false;
                }
            }
        }

        if (aCurve is LineCurve2D line2d && _parameters.ConvertCurve2d())
        {
            NCollection_Array1<Point2D> poles = new(1, 2);
            poles[1] = line2d.Value(first);
            poles[2] = line2d.Value(last);
            NCollection_Array1<double> knots = new(1, 2);
            knots[1] = first;
            knots[2] = last;
            NCollection_Array1<int> mults = new(1, 2);
            mults.Init(2);
            BSplineCurve2D res = new BSplineCurve2D(poles, knots, mults, 1);
            c = res;
            return true;
        }

        if (aCurve is ConicCurve2D && _parameters.ConvertCurve2d())
        {
            BSplineCurve2D bSpline2d;
            Curve2D tcurve = new TrimmedCurve2D(aCurve, first, last); // protection against parabols ets
            ApproxCurve2D approx = new ApproxCurve2D(tcurve, _tol2d, _continuity2d, _nbMaxSeg, 6);
            if (approx.HasResult)
            {
                bSpline2d = approx.InternalCurve()!;
            }
            else
            {
                bSpline2d = Geom2dConvert2D.CurveToBSplineCurve(tcurve, QuasiAngular);
            }

            double shift = first - bSpline2d.FirstParameter;
            if (Math.Abs(shift) > global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                NCollection_Array1<double> newKnots = new(bSpline2d.InternalKnots());
                for (int i = 1; i <= newKnots.Length(); i++)
                {
                    newKnots[i] += shift;
                }
                bSpline2d.SetKnots(newKnots);
            }
            Curve2D? resCurve = null;
            if (ConvertCurve2d(bSpline2d, ref resCurve, isConvert, first, last, ref tolCur, false))
            {
                c = resCurve;
                return true;
            }
            else
            {
                c = bSpline2d;
                tolCur = global::Kerf.Engine.Foundation.Precision.PConfusion();
                return true;
            }
        }

        if (aCurve is BezierCurve2D && _parameters.ConvertCurve2d())
        {
            Curve2D bSpline2d = Geom2dConvert2D.CurveToBSplineCurve(aCurve, QuasiAngular);
            Curve2D? resCurve = null;
            if (ConvertCurve2d(bSpline2d, ref resCurve, isConvert, first, last, ref tolCur, false))
            {
                c = resCurve;
                return true;
            }
            else
            {
                c = bSpline2d;
                tolCur = global::Kerf.Engine.Foundation.Precision.PConfusion();
                return true;
            }
        }

        if (aCurve is OffsetCurve2D tmpO && isOf)
        {
            Curve2D basCurve = tmpO.BasisCurve;
            Curve2D? resCurve = null;
            if (ConvertCurve2d(basCurve, ref resCurve, isConvert, first, last, ref tolCur))
            {
                if (resCurve!.Continuity != C0)
                {
                    c = new OffsetCurve2D(resCurve, tmpO.Offset);
                    return true;
                }
                // OCCT: converts into ResCurve, not C, so C is left unchanged with true returned (occt-issues.md)
                else if (ConvertCurve2d(aCurve, ref resCurve, isConvert, first, last, ref tolCur, false))
                {
                    return true;
                }
                else
                {
                    if (isConvert)
                    {
                        c = (Curve2D)aCurve.Copy();
                        tolCur = global::Kerf.Engine.Foundation.Precision.PConfusion();
                        return true;
                    }
                    else
                    {
                        return false;
                    }
                }
            }
            else
            {
                if (isConvert)
                {
                    c = (Curve2D)aCurve.Copy();
                    tolCur = global::Kerf.Engine.Foundation.Precision.PConfusion();
                    return true;
                }
                else
                {
                    return false;
                }
            }
        }
        if (aCurve is BSplineCurve2D
            || aCurve is BezierCurve2D
            || ((aCurve is OffsetCurve2D) && !isOf))
        {
            int deg = 1;

            if (aCurve is BSplineCurve2D bsC)
            {
                deg = bsC.Degree;
                bool isR = (_rational && bsC.IsRational);
                if (!isR && deg <= _maxDegree && (bsC.KnotCount - 1) <= _nbMaxSeg)
                {
                    if (isConvert)
                    {
                        c = (Curve2D)aCurve.Copy();
                        tolCur = global::Kerf.Engine.Foundation.Precision.PConfusion();
                        return true;
                    }
                    else
                    {
                        return false;
                    }
                }
            }
            if (aCurve is BezierCurve2D bzC)
            {
                deg = bzC.Degree;
                bool isR = (_rational && bzC.IsRational);
                if (!isR && deg <= _maxDegree)
                {
                    if (isConvert)
                    {
                        c = (Curve2D)aCurve.Copy();
                        tolCur = global::Kerf.Engine.Foundation.Precision.PConfusion();
                        return true;
                    }
                    else
                    {
                        return false;
                    }
                }
            }
            Curve2D curve1;
            double pf = aCurve.FirstParameter, pl = aCurve.LastParameter;
            // 15.11.2002 PTV OCC966
            if (CurveAnalysis.IsPeriodic(aCurve) && (first != last))
            {
                curve1 = new TrimmedCurve2D(aCurve, first, last);
            }
            else if (aCurve.FirstParameter < (first - global::Kerf.Engine.Foundation.Precision.PConfusion())
                     || aCurve.LastParameter > (last + global::Kerf.Engine.Foundation.Precision.PConfusion()))
            {
                double f = Max(first, pf), l = Min(last, pl);
                if (f != l)
                {
                    curve1 = new TrimmedCurve2D(aCurve, f, l);
                }
                else
                {
                    curve1 = aCurve;
                }
            }
            else
            {
                curve1 = aCurve;
            }
            int aC = Min(ContToInteger(_continuity2d), ContToInteger(aCurve.Continuity));
            if (aC == 0)
            {
                aC = ContToInteger(_continuity2d);
            }
            // aC = std::min(aC,(Deg -1));
            int c1 = aC;
            int maxSeg = _nbMaxSeg;
            int maxDeg = _maxDegree;
            for (; aC >= 0; aC--)
            {
                try
                {
                    Continuity cont = IntegerToGeomAbsShape(aC);
                    for (int j = 1; j <= 2; j++)
                    {
                        ApproxCurve2D anApprox = new ApproxCurve2D(curve1, _tol2d, cont, maxSeg, maxDeg);
                        bool done = anApprox.IsDone;
                        c = anApprox.InternalCurve();
                        // OCCT: dereferences the curve, null when the approximation has no result (an access
                        // violation natively, caught as a failure: decision 917)
                        int nbseg = ((BSplineCurve2D)c!).KnotCount - 1;
                        int degC = ((BSplineCurve2D)c).Degree;

                        if (_deg
                            && ((degC > maxDeg) || !done || (anApprox.MaxError() >= Max(_tol2d, tolCur))))
                        {
                            if (maxSeg < _parameters.GMaxSeg())
                            {
                                maxSeg = _parameters.GMaxSeg();
                                aC = c1;
                                // OCCT: continues the inner loop with the same aCont (occt-issues.md)
                                continue;
                            }
                            else
                            {
                                if (isConvert)
                                {
                                    c = (Curve2D)aCurve.Copy();
                                    tolCur = global::Kerf.Engine.Foundation.Precision.PConfusion();
                                    return true;
                                }
                            }
                        }

                        if (!_deg
                            && ((nbseg >= maxSeg) || !done || (anApprox.MaxError() >= Max(_tol2d, tolCur))))
                        {
                            if (maxDeg < _parameters.GMaxDegree())
                            {
                                maxDeg = _parameters.GMaxDegree();
                                aC = c1;
                                continue;
                            }
                            else
                            {
                                if (isConvert)
                                {
                                    c = (Curve2D)aCurve.Copy();
                                    tolCur = global::Kerf.Engine.Foundation.Precision.PConfusion();
                                    return true;
                                }
                            }
                        }
                        _convert = true;
                        tolCur = anApprox.MaxError();
                        _curve2dError = Max(_curve2dError, anApprox.MaxError());
                        return true;
                    }
                }
                catch (Exception exception) when (IsCaughtFailure(exception))
                {
                    continue;
                }
            }
            return false;
        }
        else
        {
            if (isConvert)
            {
                c = (Curve2D)aCurve.Copy();
                tolCur = global::Kerf.Engine.Foundation.Precision.PConfusion();
                return true;
            }
            else
            {
                return false;
            }
        }
    }

    /// <summary>
    /// True (P the vertex's point, Tol its tolerance) if anything was converted before the call, false otherwise.
    /// </summary>
    public override bool NewPoint(in Vertex v, out Point3D p, out double tol)
    {
        p = new Point3D();
        tol = Tool.Tolerance(v);
        // OCCT: myConvert is the state of the last conversion before the call (occt-issues.md)
        if (_convert)
        {
            Point3D p1 = new Point3D(Tool.Pnt(v).XYZ());
            p = p1;
            return true;
        }
        else
        {
            return false;
        }
    }

    /// <summary>False: parameters are never modified.</summary>
    public override bool NewParameter(in Vertex v, in Edge e, ref double p, ref double tol)
    {
        return false;
    }

    /// <summary>The continuity of the edge between the faces, unchanged.</summary>
    public override Continuity Continuity(in Edge e,
                                             in Face f1,
                                             in Face f2,
                                             in Edge newE,
                                             in Face newF1,
                                             in Face newF2)
    {
        return Tool.Continuity(e, f1, f2);
    }

    /// <summary>The largest errors of the approximations of surfaces (returned), 3D curves and pcurves.</summary>
    public double MaxErrors(out double aCurve3dErr, out double aCurve2dErr)
    {
        aCurve3dErr = _curve3dError;
        aCurve2dErr = _curve2dError;
        return _surfaceError;
    }

    /// <summary>The number of spans of the approximated surfaces.</summary>
    public int OfSpanCount
    {
        get
        {
            return _nbOfSpan;
        }
    }

    /// <summary>Sets the tolerance of approximation of 3D curves and surfaces.</summary>
    public void SetTol3d(double tol3d)
    {
        _tol3d = tol3d;
    }

    /// <summary>Sets the tolerance of approximation of pcurves.</summary>
    public void SetTol2d(double tol2d)
    {
        _tol2d = tol2d;
    }

    /// <summary>Whether surfaces are approximated, for reading and writing.</summary>
    public ref bool ModifyApproxSurfaceFlag() => ref _approxSurfaceFlag;

    /// <summary>Whether 3D curves are approximated, for reading and writing.</summary>
    public ref bool ModifyApproxCurve3dFlag() => ref _approxCurve3dFlag;

    /// <summary>Whether pcurves are approximated, for reading and writing.</summary>
    public ref bool ModifyApproxCurve2dFlag() => ref _approxCurve2dFlag;

    /// <summary>Sets the continuity of the approximation of 3D curves and surfaces.</summary>
    public void SetContinuity3d(Continuity continuity3d)
    {
        _continuity3d = continuity3d;
    }

    /// <summary>Sets the continuity of the approximation of pcurves.</summary>
    public void SetContinuity2d(Continuity continuity2d)
    {
        _continuity2d = continuity2d;
    }

    /// <summary>Sets the maximal degree of approximation.</summary>
    public void SetMaxDegree(int maxDegree)
    {
        _maxDegree = maxDegree;
    }

    /// <summary>Sets the maximal number of spans of approximation.</summary>
    public void SetMaxNbSegments(int maxNbSegments)
    {
        _nbMaxSeg = maxNbSegments;
    }

    /// <summary>
    /// Sets the priority: true keeps the degree under the maximal one at the expense of the number of spans, false the
    /// number of spans at the expense of the degree.
    /// </summary>
    public void SetPriority(bool degree)
    {
        _deg = degree;
    }

    /// <summary>Sets whether rational B-splines and Beziers are approximated by polynomial ones even within the limits.</summary>
    public void SetConvRational(bool rational)
    {
        _rational = rational;
    }

    /// <summary>The parameters saying which geometry to convert.</summary>
    public RestrictionParameters GetRestrictionParameters()
    {
        return _parameters;
    }

    /// <summary>Sets the parameters saying which geometry to convert.</summary>
    public void SetRestrictionParameters(RestrictionParameters aModes)
    {
        _parameters = aModes;
    }

    /// <summary>The largest error of approximation of 3D curves.</summary>
    public double Curve3dError()
    {
        return _curve3dError;
    }

    /// <summary>The largest error of approximation of pcurves.</summary>
    public double Curve2dError()
    {
        return _curve2dError;
    }

    /// <summary>The largest error of approximation of surfaces.</summary>
    public double SurfaceError()
    {
        return _surfaceError;
    }
}
