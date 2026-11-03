// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_BSplineRestriction.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_BSplineRestriction.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_BSplineRestriction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>
/// The modification that approximates surfaces, curves and pcurves with a given degree, number of spans and
/// tolerances (3D and 2D), as B-splines; the continuity asked for is lowered where the approximation fails with it.
/// <see cref="ShapeCustom_RestrictionParameters"/> says which kinds of geometry to convert.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21). Its errors and number of spans accumulate over the faces and edges it converts,
/// and, as in OCCT, <see cref="NewPoint"/> answers from whether anything was converted before it is called, so a
/// modification reused (a compound's children, <see cref="ShapeCustom.ApplyModifier"/>) gives new points to later
/// children's vertices.
/// </para>
/// <para>
/// It writes only into the modifier's new edges (<see cref="NewCurve2d"/>'s range on <c>NewE</c>, never the
/// caller's edge) and into the B-splines its approximations create (decision 973).
/// </para>
/// </remarks>
public class ShapeCustom_BSplineRestriction : ShapeCustom_Modification
{
    private Continuity myContinuity3d;
    private Continuity myContinuity2d;
    private int myMaxDegree;
    private int myNbMaxSeg;
    private double myTol3d;
    private double myTol2d;
    private double mySurfaceError;
    private double myCurve3dError;
    private double myCurve2dError;
    private int myNbOfSpan;
    private bool myApproxSurfaceFlag;
    private bool myApproxCurve3dFlag;
    private bool myApproxCurve2dFlag;
    private bool myDeg;
    private bool myConvert;
    private bool myRational;
    private ShapeCustom_RestrictionParameters myParameters;

    // OCC_CATCH_SIGNALS: OCCT catches Standard_Failure, and access violations as failures (decision 917); never a write
    // into a frozen shape or geometry (decision 423)
    private static bool IsCaughtFailure(Exception theException)
        => theException is not (ImmutableObjectException or LockedShapeException)
           && theException is FailureException or NullReferenceException;

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

    private static int ContToInteger(Continuity Cont)
    {
        int result = 0;
        switch (Cont)
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
                                         int Degree,
                                         int NbSeg,
                                         bool myRational,
                                         ShapeCustom_RestrictionParameters aParameters)
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
            Curve3D BasCurve = tmp.BasisCurve();
            return IsConvertCurve3d(BasCurve, Degree, NbSeg, myRational, aParameters);
        }

        if (aCurve is OffsetCurve3D tmpO)
        {
            if (aParameters.ConvertOffsetCurv3d())
            {
                return true;
            }
            Curve3D BasCurve = tmpO.BasisCurve();
            return IsConvertCurve3d(BasCurve, Degree, NbSeg, myRational, aParameters);
        }
        if (aCurve is BSplineCurve3D BsC)
        {
            // OCCT: '>=' here, '>' for 2D curves (occt-issues.md)
            if (BsC.Degree() > Degree || ((BsC.KnotCount - 1) >= NbSeg))
            {
                return true;
            }
            return myRational && BsC.IsRational;
        }
        if (aCurve is BezierCurve3D aBz
            && (aBz.Degree() > Degree || (myRational && aBz.IsRational)))
        {
            return true;
        }
        // else return false;
        return false;
    }

    private static bool IsConvertSurface(Surface? aSurface,
                                         int Degree,
                                         int NbSeg,
                                         bool myRational,
                                         ShapeCustom_RestrictionParameters aParameters)
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
        if (aSurface is SweptSurface aSwept)
        {
            if (aSurface is RevolutionSurface && aParameters.ConvertRevolutionSurf())
            {
                return true;
            }
            if (aSurface is ExtrusionSurface && aParameters.ConvertExtrusionSurf())
            {
                return true;
            }
            Curve3D BasCurve = aSwept.BasisCurve();
            return IsConvertCurve3d(BasCurve, Degree, NbSeg, myRational, aParameters);
        }
        if (aSurface is TrimmedSurface aRTS)
        {
            Surface theSurf = aRTS.BasisSurface();
            return IsConvertSurface(theSurf, Degree, NbSeg, myRational, aParameters);
        }
        if (aSurface is OffsetSurface aOS)
        {
            if (aParameters.ConvertOffsetSurf())
            {
                return true;
            }
            Surface theSurf = aOS.BasisSurface();
            return IsConvertSurface(theSurf, Degree, NbSeg, myRational, aParameters);
        }
        if (aSurface is BSplineSurface theBSurf)
        {
            if (theBSurf.UDegree() > Degree || theBSurf.VDegree() > Degree)
            {
                return true;
            }
            if ((theBSurf.UKnotCount - 1) * (theBSurf.VKnotCount - 1) > NbSeg)
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
            if (theBzSurf.UDegree() > Degree || theBzSurf.VDegree() > Degree)
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
                                         int Degree,
                                         int NbSeg,
                                         bool myRational,
                                         ShapeCustom_RestrictionParameters aParameters)
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
            Curve2D BasCurve = tmp.BasisCurve();
            return IsConvertCurve2d(BasCurve, Degree, NbSeg, myRational, aParameters);
        }
        if (aCurve is OffsetCurve2D tmpO)
        {
            if (aParameters.ConvertOffsetCurv2d())
            {
                return true;
            }
            Curve2D BasCurve = tmpO.BasisCurve();
            return IsConvertCurve2d(BasCurve, Degree, NbSeg, myRational, aParameters);
        }
        if (aCurve is BSplineCurve2D aBs
            && ((aBs.Degree() > Degree || ((aBs.KnotCount - 1) > NbSeg))
                || (myRational && aBs.IsRational)))
        {
            return true;
        }
        if (aCurve is BezierCurve2D aBz
            && ((aBz.Degree() > Degree) || (myRational && aBz.IsRational)))
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
    public ShapeCustom_BSplineRestriction()
    {
        myApproxSurfaceFlag = true;
        myApproxCurve3dFlag = true;
        myApproxCurve2dFlag = true;
        myTol3d = 0.01;
        myTol2d = 1E-6;
        myContinuity3d = C1;
        myContinuity2d = C2;
        myMaxDegree = 9;
        myNbMaxSeg = 10000;
        mySurfaceError = global::Kerf.Engine.Foundation.Precision.Confusion();
        myCurve3dError = global::Kerf.Engine.Foundation.Precision.Confusion();
        myCurve2dError = global::Kerf.Engine.Foundation.Precision.PConfusion();
        myNbOfSpan = 0;
        myConvert = false;
        myDeg = true;
        myRational = false;
        myParameters = new ShapeCustom_RestrictionParameters();
    }

    /// <summary>A modification with the given parameters of approximation and the default restriction parameters.</summary>
    public ShapeCustom_BSplineRestriction(bool anApproxSurfaceFlag,
                                          bool anApproxCurve3dFlag,
                                          bool anApproxCurve2dFlag,
                                          double aTol3d,
                                          double aTol2d,
                                          Continuity aContinuity3d,
                                          Continuity aContinuity2d,
                                          int aMaxDegree,
                                          int aNbMaxSeg,
                                          bool Deg,
                                          bool Rational)
    {
        myApproxSurfaceFlag = anApproxSurfaceFlag;
        myApproxCurve3dFlag = anApproxCurve3dFlag;
        myApproxCurve2dFlag = anApproxCurve2dFlag;
        myTol3d = aTol3d;
        myTol2d = aTol2d;
        myMaxDegree = aMaxDegree;
        myContinuity3d = aContinuity3d;
        myContinuity2d = aContinuity2d;
        myNbMaxSeg = aNbMaxSeg;
        mySurfaceError = global::Kerf.Engine.Foundation.Precision.Confusion();
        myCurve3dError = global::Kerf.Engine.Foundation.Precision.Confusion();
        myCurve2dError = global::Kerf.Engine.Foundation.Precision.PConfusion();
        myNbOfSpan = 0;
        myConvert = false;
        myDeg = Deg;
        myRational = Rational;
        myParameters = new ShapeCustom_RestrictionParameters();
    }

    /// <summary>A modification with the given parameters of approximation and restriction parameters.</summary>
    public ShapeCustom_BSplineRestriction(bool anApproxSurfaceFlag,
                                          bool anApproxCurve3dFlag,
                                          bool anApproxCurve2dFlag,
                                          double aTol3d,
                                          double aTol2d,
                                          Continuity aContinuity3d,
                                          Continuity aContinuity2d,
                                          int aMaxDegree,
                                          int aNbMaxSeg,
                                          bool Deg,
                                          bool Rational,
                                          ShapeCustom_RestrictionParameters aModes)
    {
        myApproxSurfaceFlag = anApproxSurfaceFlag;
        myApproxCurve3dFlag = anApproxCurve3dFlag;
        myApproxCurve2dFlag = anApproxCurve2dFlag;
        myTol3d = aTol3d;
        myTol2d = aTol2d;
        myMaxDegree = aMaxDegree;
        myContinuity3d = aContinuity3d;
        myContinuity2d = aContinuity2d;
        myNbMaxSeg = aNbMaxSeg;
        mySurfaceError = global::Kerf.Engine.Foundation.Precision.Confusion();
        myCurve3dError = global::Kerf.Engine.Foundation.Precision.Confusion();
        myCurve2dError = global::Kerf.Engine.Foundation.Precision.PConfusion();
        myNbOfSpan = 0;
        myConvert = false;
        myDeg = Deg;
        myRational = Rational;
        myParameters = aModes;
    }

    /// <summary>
    /// True if the face's surface is approximated: then S is the approximation (over the face's UV bounds in the
    /// segment mode, or where the surface is infinite).
    /// </summary>
    public override bool NewSurface(in Face F,
                                    out Surface? S,
                                    out Placement L,
                                    out double Tol,
                                    out bool RevWires,
                                    out bool RevFace)
    {
        S = null;
        Tol = 0.0;
        RevWires = RevFace = false;
        if (!myApproxSurfaceFlag)
        {
            L = new Placement();
            return false;
        }
        RevWires = false;
        RevFace = false;
        myConvert = false;
        Surface? aSurface = Tool.Surface(F, out L);
        if (aSurface is null)
        {
            return false;
        }
        bool IsOf = true;
        if (myParameters.ConvertOffsetSurf())
        {
            IsOf = false;
        }
        aSurface.Bounds(out double UF, out double UL, out double VF, out double VL);
        BrepTools.UVBounds(F, out double Umin, out double Umax, out double Vmin, out double Vmax);
        if (myParameters.SegmentSurfaceMode())
        {
            UF = Umin;
            UL = Umax;
            VF = Vmin;
            VL = Vmax;
        }
        else
        {
            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(UF) || global::Kerf.Engine.Foundation.Precision.IsInfinite(UL))
            {
                UF = Umin;
                UL = Umax;
            }
            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(VF) || global::Kerf.Engine.Foundation.Precision.IsInfinite(VL))
            {
                VF = Vmin;
                VL = Vmax;
            }
        }

        bool IsConv = ConvertSurface(aSurface, ref S, UF, UL, VF, VL, IsOf);
        Tol = global::Kerf.Engine.Foundation.Precision.Confusion(); // mySurfaceError;

        if (IsConv)
        {
            bool wasBSpline = aSurface is BSplineSurface;
            if (aSurface is TrimmedSurface rts)
            {
                wasBSpline = rts.BasisSurface() is BSplineSurface;
            }

            if (wasBSpline)
            {
                SendMsg(F, new Msg("BSplineRestriction.NewSurface.MSG1"));
            }
            else
            {
                SendMsg(F, new Msg("BSplineRestriction.NewSurface.MSG0"));
            }
        }

        return IsConv;
    }

    private static void ConvertExtrusion(Curve3D? C, /*const gp_Dir& direction,*/
                                         in Transform3D shiftF,
                                         in Transform3D shiftL,
                                         double VF,
                                         double VL,
                                         out Surface bspline)
    {
        // OCCT: dereferences C, null when the approximation of the basis curve failed (an access violation natively)
        BSplineCurve3D bspl = (BSplineCurve3D)C!;
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
        vknots[1] = VF;
        vknots[2] = VL;
        vmults[1] = vmults[2] = 2;

        bspline = new BSplineSurface(resPoles,
                                          resWeigth,
                                          knots,
                                          vknots,
                                          mults,
                                          vmults,
                                          bspl.Degree(),
                                          1,
                                          bspl.IsPeriodic,
                                          false);
    }

    /// <summary>
    /// True if the surface is converted: then S is the result (left unchanged otherwise). With <paramref name="IsOf"/>,
    /// offset surfaces are approximated as offsets of approximated bases, else as B-splines.
    /// </summary>
    public bool ConvertSurface(Surface aSurface,
                               ref Surface? S,
                               double UF,
                               double UL,
                               double VF,
                               double VL,
                               bool IsOf = true)
    {
        if (!IsConvertSurface(aSurface, myMaxDegree, myNbMaxSeg, myRational, myParameters))
        {
            return false;
        }

        Surface aSurf = aSurface;
        if (aSurf is PlaneSurface pln && myParameters.ConvertPlane())
        {
            NCollection_Array2<Point3D> poles = new(1, 2, 1, 2);
            NCollection_Array1<double> uknots = new(1, 2);
            NCollection_Array1<int> umults = new(1, 2);
            NCollection_Array1<double> vknots = new(1, 2);
            NCollection_Array1<int> vmults = new(1, 2);

            poles[1, 1] = pln.Value(UF, VF);
            poles[1, 2] = pln.Value(UF, VL);
            poles[2, 1] = pln.Value(UL, VF);
            poles[2, 2] = pln.Value(UL, VL);
            uknots[1] = UF;
            uknots[2] = UL;
            vknots[1] = VF;
            vknots[2] = VL;
            umults[1] = umults[2] = vmults[1] = vmults[2] = 2;
            S = new BSplineSurface(poles, uknots, vknots, umults, vmults, 1, 1, false, false);
            return true;
        }
        if (aSurf is RevolutionSurface SurfaceR)
        {
            Curve3D BasCurve = SurfaceR.BasisCurve();
            Curve3D? ResCurve = null;
            double TolS = global::Kerf.Engine.Foundation.Precision.Confusion();
            if (myParameters.ConvertRevolutionSurf())
            {
                if (BasCurve is OffsetCurve3D)
                {
                    Continuity cnt = BasCurve.Continuity();
                    cnt = (cnt > C2 ? C2 : cnt);
                    if (ConvertCurve(BasCurve,
                                     ref ResCurve,
                                     false,
                                     Max(VF, BasCurve.FirstParameter()),
                                     Min(VL, BasCurve.LastParameter()),
                                     ref TolS,
                                     false))
                    {
                        RevolutionSurface newRevol = new RevolutionSurface(ResCurve!, SurfaceR.Axis());
                        aSurf = newRevol;
                    }
                }
            }
            else
            {
                if (ConvertCurve(BasCurve,
                                 ref ResCurve,
                                 false,
                                 Max(VF, BasCurve.FirstParameter()),
                                 Min(VL, BasCurve.LastParameter()),
                                 ref TolS,
                                 IsOf))
                {
                    S = new RevolutionSurface(ResCurve!, SurfaceR.Axis());
                    return true;
                }
                else
                {
                    return false;
                }
            }
        }
        if (aSurf is ExtrusionSurface SurfaceE)
        {
            Curve3D? BasCurve = SurfaceE.BasisCurve();
            Curve3D? ResCurve = null;
            double TolS = global::Kerf.Engine.Foundation.Precision.Confusion();
            if (myParameters.ConvertExtrusionSurf())
            {
                Continuity cnt = SurfaceE.Continuity();
                cnt = (cnt > C2 ? C2 : cnt);
                BSplineCurve3D? bspl =
                    ShapeConstruct.InternalConvertCurveToBSpline(BasCurve, UF, UL, TolS, cnt, myNbMaxSeg, myMaxDegree);
                BasCurve = bspl;
                // OCCT: dereferences BasCurve, null when the conversion failed (an access violation natively)
                ConvertCurve(BasCurve!,
                             ref ResCurve,
                             true,
                             Max(UF, BasCurve!.FirstParameter()),
                             Min(UL, BasCurve.LastParameter()),
                             ref TolS,
                             IsOf);
                Transform3D shiftF = new Transform3D(), shiftL = new Transform3D();
                shiftF.SetTranslation(SurfaceE.Value(UF, 0), SurfaceE.Value(UF, VF));
                shiftL.SetTranslation(SurfaceE.Value(UF, 0), SurfaceE.Value(UF, VL));
                ConvertExtrusion(ResCurve, /*Surface->Direction(),*/ shiftF, shiftL, VF, VL, out Surface aBSpline);
                S = aBSpline;
                return true;
            }
            else
            {
                if (ConvertCurve(BasCurve,
                                 ref ResCurve,
                                 false,
                                 Max(UF, BasCurve.FirstParameter()),
                                 Min(UL, BasCurve.LastParameter()),
                                 ref TolS,
                                 IsOf))
                {
                    S = new ExtrusionSurface(ResCurve!, SurfaceE.Direction());
                    return true;
                }
                else
                {
                    return false;
                }
            }
        }
        if (aSurf is TrimmedSurface tmpR)
        {
            tmpR.Bounds(out double U1, out double U2, out double V1, out double V2);
            Surface theSurf = tmpR.BasisSurface();
            Surface? ResSurface = null;
            if (ConvertSurface(theSurf, ref ResSurface, U1, U2, V1, V2, IsOf))
            {
                // S = new Geom_RectangularTrimmedSurface(ResSurface,U1,U2,V1,V2);
                S = ResSurface;
                return true;
            }
            else
            {
                return false;
            }
        }
        if (aSurf is OffsetSurface tmpO && IsOf)
        {
            Surface theSurf = tmpO.BasisSurface();
            Surface? ResSurface = null;
            if (ConvertSurface(theSurf, ref ResSurface, UF, UL, VF, VL))
            {
                if (ResSurface!.Continuity() != C0)
                {
                    S = new OffsetSurface(ResSurface, tmpO.Offset());
                    return true;
                }
                else if (ConvertSurface(aSurf, ref S, UF, UL, VF, VL, false))
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
        if (aSurf is BezierSurface bezier && myParameters.ConvertBezierSurf())
        {
            int uDegree = bezier.UDegree();
            int vDegree = bezier.VDegree();
            NCollection_Array2<Point3D> aPoles = bezier.InternalPoles();
            NCollection_Array2<double> aWeights = bezier.InternalWeightsArray();
            NCollection_Array1<double> uKnots = new(1, 2), vKnots = new(1, 2);
            uKnots[1] = 0;
            uKnots[2] = 1;
            vKnots[1] = 0;
            vKnots[2] = 1;
            NCollection_Array1<int> uMults = new(1, 2), vMults = new(1, 2);
            uMults.Init(uDegree + 1);
            vMults.Init(vDegree + 1);
            BSplineSurface bspline =
                new BSplineSurface(aPoles, aWeights, uKnots, vKnots, uMults, vMults, uDegree, vDegree);

            if (!ConvertSurface(bspline, ref S, UF, UL, VF, VL, IsOf))
            {
                S = bspline;
            }
            return true;
        }

        int NbSeg = 1;
        bool URat = false;
        bool VRat = false;
        int UDeg = 1, VDeg = 1;
        if (aSurf is BSplineSurface BsS)
        {
            UDeg = BsS.UDegree();
            VDeg = BsS.VDegree();
            NbSeg = (BsS.UKnotCount - 1) * (BsS.VKnotCount - 1);
            URat = BsS.IsURational;
            VRat = BsS.IsVRational;
            bool IsR = (myRational && (URat || VRat));
            if (UDeg <= myMaxDegree && VDeg <= myMaxDegree && NbSeg <= myNbMaxSeg && !IsR)
            {
                return false;
            }
        }
        if (aSurf is BezierSurface BsZ)
        {
            UDeg = BsZ.UDegree();
            VDeg = BsZ.VDegree();
            NbSeg = 1;
            URat = BsZ.IsURational;
            VRat = BsZ.IsVRational;
            bool IsR = (myRational && (URat || VRat));
            if (UDeg <= myMaxDegree && VDeg <= myMaxDegree && NbSeg <= myNbMaxSeg && !IsR)
            {
                return false;
            }
        }
        Continuity Cont = myContinuity3d;
        if (aSurf is OffsetSurface)
        {
            Cont = C0;
        }

        double aTol3d;
        int nbOfSpan, imax = 10;
        int MaxSeg = myNbMaxSeg;
        int MaxDeg = myMaxDegree;
        aSurf.Bounds(out double u1, out double u2, out double v1, out double v2);
        double ShiftU = 0, ShiftV = 0;
        if (Math.Abs(u1 - UF) > global::Kerf.Engine.Foundation.Precision.PConfusion() || Math.Abs(u2 - UL) > global::Kerf.Engine.Foundation.Precision.PConfusion()
            || Math.Abs(v1 - VF) > global::Kerf.Engine.Foundation.Precision.PConfusion() || Math.Abs(v2 - VL) > global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            bool isTrim = false;
            if (!aSurf.IsUPeriodic)
            { // else {
                u1 = Max(u1, UF);
                u2 = Min(u2, UL);
                isTrim = true;
            }
            if (!aSurf.IsVPeriodic)
            { // else
                v1 = Max(v1, VF);
                v2 = Min(v2, VL);
                isTrim = true;
            }

            if (isTrim && (u1 != u2) && (v1 != v2))
            {
                TrimmedSurface trSurface = new TrimmedSurface(aSurf, u1, u2, v1, v2);
                trSurface.Bounds(out double ur1, out double ur2, out double vr1, out double vr2);
                ShiftU = u1 - ur1;
                ShiftV = v1 - vr1;
                aSurf = trSurface;
            }
        }
        int aCU = Min(ContToInteger(Cont), ContToInteger(aSurf.Continuity()));
        int aCV = Min(ContToInteger(Cont), ContToInteger(aSurf.Continuity()));
        if (aCU == 0)
        {
            aCU = ContToInteger(Cont);
        }
        if (aCV == 0)
        {
            aCV = ContToInteger(Cont);
        }

        for (;;)
        {
            double prevTol = RealLast(), newTol = 0;
            for (int i = 1; i <= imax; i++)
            {
                aTol3d = myTol3d * i / 2;
                while (aCU >= 0 || aCV >= 0)
                {
                    try
                    {
                        Continuity aContV = IntegerToGeomAbsShape(aCV);
                        Continuity aContU = IntegerToGeomAbsShape(aCU);

                        ApproxSurface anApprox = new ApproxSurface(aSurf, aTol3d, aContU, aContV, MaxDeg, MaxDeg, MaxSeg, 0);
                        bool Done = anApprox.IsDone;
                        newTol = anApprox.MaxError();
                        if (anApprox.MaxError() <= myTol3d && Done)
                        {
                            nbOfSpan = (anApprox.Surface()!.UKnotCount - 1) * (anApprox.Surface()!.VKnotCount - 1);
                            S = anApprox.Surface();
                            BSplineSurface Bsc = (BSplineSurface)S!;
                            // OCCT.NET: the approximation's surface is frozen (decision 197); OCCT changes it in place
                            // below, the port changes a copy (decision 974)
                            if (aSurface.IsUPeriodic || aSurface.IsVPeriodic || Math.Abs(ShiftU) > global::Kerf.Engine.Foundation.Precision.PConfusion()
                                || Math.Abs(ShiftV) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                            {
                                Bsc = (BSplineSurface)Bsc.Copy();
                                S = Bsc;
                            }
                            if (aSurface.IsUPeriodic)
                            {
                                Bsc.SetUPeriodic();
                            }
                            if (aSurface.IsVPeriodic)
                            {
                                Bsc.SetVPeriodic();
                            }
                            myConvert = true;
                            myNbOfSpan = myNbOfSpan + nbOfSpan;
                            mySurfaceError = Max(mySurfaceError, anApprox.MaxError());
                            if (Math.Abs(ShiftU) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                            {
                                NCollection_Array1<double> uknots = new(Bsc.InternalUKnots());
                                for (int j = 1; j <= uknots.Length(); j++)
                                {
                                    uknots[j] += ShiftU;
                                }
                                Bsc.SetUKnots(uknots);
                            }
                            if (Math.Abs(ShiftV) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                            {
                                NCollection_Array1<double> vknots = new(Bsc.InternalVKnots());
                                for (int j = 1; j <= vknots.Length(); j++)
                                {
                                    vknots[j] += ShiftV;
                                }
                                Bsc.SetVKnots(vknots);
                            }

                            return true;
                        }
                        else
                        {
                            break;
                        }
                    }
                    catch (Exception anException) when (IsCaughtFailure(anException))
                    {
                        // szv: protection against loop
                        if (aCU == 0 && aCV == 0)
                        {
                            break;
                        }
                        if (aCU > 0)
                        {
                            aCU--;
                        }
                        if (aCV > 0)
                        {
                            aCV--;
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

            if (myDeg)
            {
                if (MaxSeg < myParameters.GMaxSeg())
                {
                    if (aCV != 0 || aCU != 0)
                    {
                        if (aCV > 0)
                        {
                            aCV--;
                        }
                        if (aCU > 0)
                        {
                            aCU--;
                        }
                    }
                    else
                    {
                        MaxSeg = 2 * MaxSeg; // myGMaxSeg;
                    }
                    if (MaxSeg > myParameters.GMaxSeg())
                    {
                        MaxSeg = myParameters.GMaxSeg();
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
                if (MaxDeg < myParameters.GMaxDegree())
                {
                    MaxDeg = myParameters.GMaxDegree();
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
    public override bool NewCurve(in Edge E, out Curve3D? C, out Placement L, out double Tol)
    {
        C = null;
        Tol = 0.0;
        if (!myApproxCurve3dFlag)
        {
            L = new Placement();
            return false;
        }
        Curve3D? aCurve = Tool.Curve(E, out L, out double First, out double Last);
        double TolCur = Tool.Tolerance(E);
        // if(aCurve.IsNull()) return false;
        bool IsConvert = false;
        BrepTEdge TE = (BrepTEdge)E.TShape()!;
        // iterate on pcurves
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(TE.Curves());
        for (; itcr.More(); itcr.Next())
        {
            if (itcr.Value() is not GCurve GC || !GC.IsCurveOnSurface())
            {
                continue;
            }
            Surface? aSurface = GC.Surface();
            Curve2D? aCurve2d = GC.PCurve();
            if ((myApproxSurfaceFlag
                 && IsConvertSurface(aSurface, myMaxDegree, myNbMaxSeg, myRational, myParameters))
                || (myApproxCurve2dFlag
                    && IsConvertCurve2d(aCurve2d, myMaxDegree, myNbMaxSeg, myRational, myParameters)))
            {
                IsConvert = true;
                break;
            }
        }
        if (aCurve is null)
        {
            if (IsConvert)
            {
                C = aCurve;
                Tol = TolCur;
                return true;
            }
            else
            {
                return false;
            }
        }
        bool IsOf = true;
        if (myParameters.ConvertOffsetCurv3d())
        {
            IsOf = false;
        }
        bool IsConv = ConvertCurve(aCurve, ref C, IsConvert, First, Last, ref TolCur, IsOf);
        Tol = Tool.Tolerance(E); // TolCur;

        if (IsConv)
        {
            bool wasBSpline = aCurve is BSplineCurve3D;
            if (aCurve is TrimmedCurve3D tc)
            {
                wasBSpline = tc.BasisCurve() is BSplineCurve3D;
            }

            if (wasBSpline)
            {
                SendMsg(E, new Msg("BSplineRestriction.NewCurve.MSG1"));
            }
            else
            {
                SendMsg(E, new Msg("BSplineRestriction.NewCurve.MSG0"));
            }
        }
        return IsConv;
    }

    /// <summary>
    /// True if the curve is converted, or copied (<paramref name="IsConvert"/>): then C is the result (left unchanged
    /// otherwise) and TolCur its error. With <paramref name="IsOf"/>, offset curves are approximated as offsets of
    /// approximated bases, else as B-splines.
    /// </summary>
    public bool ConvertCurve(Curve3D aCurve,
                             ref Curve3D? C,
                             bool IsConvert,
                             double First,
                             double Last,
                             ref double TolCur,
                             bool IsOf = true)
    {
        //  TolCur =  Precision::Confusion();
        if (aCurve is TrimmedCurve3D tmpT)
        {
            Curve3D BasCurve = tmpT.BasisCurve();
            Curve3D? ResCurve = null;
            if (ConvertCurve(BasCurve, ref ResCurve, IsConvert, First, Last, ref TolCur, IsOf))
            {
                C = ResCurve;
                return true;
            }
            else
            {
                if (IsConvert)
                {
                    C = (Curve3D)aCurve.Copy();
                    TolCur = global::Kerf.Engine.Foundation.Precision.Confusion();
                    return true;
                }

                return false;
            }
        }

        if (aCurve is LineCurve3D aLine && myParameters.ConvertCurve3d())
        {
            NCollection_Array1<Point3D> poles = new(1, 2);
            poles[1] = aLine.Value(First);
            poles[2] = aLine.Value(Last);
            NCollection_Array1<double> knots = new(1, 2);
            knots[1] = First;
            knots[2] = Last;
            NCollection_Array1<int> mults = new(1, 2);
            mults.Init(2);
            BSplineCurve3D res = new BSplineCurve3D(poles, knots, mults, 1);
            C = res;
            return true;
        }

        if (aCurve is ConicCurve3D && myParameters.ConvertCurve3d())
        {
            BSplineCurve3D aBSpline;
            Curve3D tcurve = new TrimmedCurve3D(aCurve, First, Last); // protection against parabols ets
            ApproxCurve approx = new ApproxCurve(tcurve,
                                                                        myTol3d /*Precision::Approximation()*/,
                                                                        myContinuity2d,
                                                                        myNbMaxSeg,
                                                                        6);
            if (approx.HasResult)
            {
                aBSpline = approx.InternalCurve()!;
            }
            else
            {
                aBSpline = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(tcurve, QuasiAngular);
            }

            double Shift = First - aBSpline.FirstParameter();
            if (Math.Abs(Shift) > global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                NCollection_Array1<double> newKnots = new(aBSpline.InternalKnots());
                for (int i = 1; i <= newKnots.Length(); i++)
                {
                    newKnots[i] += Shift;
                }
                aBSpline.SetKnots(newKnots);
            }
            Curve3D? ResCurve = null;
            if (ConvertCurve(aBSpline, ref ResCurve, IsConvert, First, Last, ref TolCur, false))
            {
                C = ResCurve;
                return true;
            }
            else
            {
                C = aBSpline;
                TolCur = global::Kerf.Engine.Foundation.Precision.PConfusion();
                return true;
            }
        }

        if (aCurve is BezierCurve3D && myParameters.ConvertCurve3d())
        {
            Curve3D aBSpline = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(aCurve, QuasiAngular);
            Curve3D? ResCurve = null;
            if (ConvertCurve(aBSpline, ref ResCurve, IsConvert, First, Last, ref TolCur, false))
            {
                C = ResCurve;
                return true;
            }
            else
            {
                C = aBSpline;
                TolCur = global::Kerf.Engine.Foundation.Precision.PConfusion();
                return true;
            }
        }

        if (aCurve is OffsetCurve3D tmpO && IsOf)
        {
            Curve3D BasCurve = tmpO.BasisCurve();
            Curve3D? ResCurve = null;
            if (ConvertCurve(BasCurve, ref ResCurve, IsConvert, First, Last, ref TolCur))
            {
                if (ResCurve!.Continuity() != C0)
                {
                    C = new OffsetCurve3D(ResCurve, tmpO.Offset(), tmpO.Direction());
                    return true;
                }
                else if (ConvertCurve(aCurve, ref C, IsConvert, First, Last, ref TolCur, false))
                {
                    return true;
                }
                else
                {
                    if (IsConvert)
                    {
                        C = (Curve3D)aCurve.Copy();
                        TolCur = global::Kerf.Engine.Foundation.Precision.Confusion();
                        return true;
                    }
                    return false;
                }
            }
            else
            {
                if (IsConvert)
                {
                    C = (Curve3D)aCurve.Copy();
                    TolCur = global::Kerf.Engine.Foundation.Precision.Confusion();
                    return true;
                }
                return false;
            }
        }
        if (aCurve is BSplineCurve3D
            || aCurve is BezierCurve3D
            || (aCurve is OffsetCurve3D && !IsOf))
        {
            int Deg = 1;

            if (aCurve is BSplineCurve3D BsC)
            {
                Deg = BsC.Degree();
                bool IsR = (myRational && BsC.IsRational);
                if (!IsR && Deg <= myMaxDegree && (BsC.KnotCount - 1) <= myNbMaxSeg)
                {
                    if (IsConvert)
                    {
                        C = (Curve3D)aCurve.Copy();
                        TolCur = global::Kerf.Engine.Foundation.Precision.Confusion();
                        return true;
                    }
                    else
                    {
                        return false;
                    }
                }
            }
            if (aCurve is BezierCurve3D BzC)
            {
                Deg = BzC.Degree();
                bool IsR = (myRational && BzC.IsRational);
                if (!IsR && Deg <= myMaxDegree)
                {
                    if (IsConvert)
                    {
                        C = (Curve3D)aCurve.Copy();
                        TolCur = global::Kerf.Engine.Foundation.Precision.Confusion();
                        return true;
                    }
                    else
                    {
                        return false;
                    }
                }
            }
            Curve3D aCurve1;
            double pf = aCurve.FirstParameter(), pl = aCurve.LastParameter();
            // 15.11.2002 PTV OCC966
            if (ShapeAnalysis_Curve.IsPeriodic(aCurve) && (First != Last))
            {
                aCurve1 = new TrimmedCurve3D(aCurve, First, Last);
            }
            else if (pf < (First - global::Kerf.Engine.Foundation.Precision.PConfusion()) || pl > (Last + global::Kerf.Engine.Foundation.Precision.PConfusion()))
            {
                double F = Max(First, pf), L = Min(Last, pl);
                if (F != L)
                {
                    aCurve1 = new TrimmedCurve3D(aCurve, F, L);
                }
                else
                {
                    aCurve1 = aCurve;
                }
            }
            else
            {
                aCurve1 = aCurve;
            }
            int aC = Min(ContToInteger(myContinuity3d), ContToInteger(aCurve.Continuity()));
            if (aC == 0)
            {
                aC = ContToInteger(myContinuity3d);
            }
            // aC = std::min(aC,(Deg -1));
            int MaxSeg = myNbMaxSeg;
            int MaxDeg = myMaxDegree;
            int aC1 = aC;
            for (; aC >= 0; aC--)
            {
                try
                {
                    for (int j = 1; j <= 2; j++)
                    {
                        Continuity aCont = IntegerToGeomAbsShape(aC);
                        ApproxCurve anApprox = new ApproxCurve(aCurve1, myTol3d, aCont, MaxSeg, MaxDeg);
                        bool Done = anApprox.IsDone;
                        C = anApprox.InternalCurve();
                        // OCCT: dereferences the curve, null when the approximation has no result (an access
                        // violation natively, caught as a failure: decision 917)
                        int Nbseg = ((BSplineCurve3D)C!).KnotCount - 1;
                        int DegC = ((BSplineCurve3D)C).Degree();
                        if (myDeg
                            && ((DegC > MaxDeg) || !Done || (anApprox.MaxError() >= Max(TolCur, myTol3d))))
                        {
                            if (MaxSeg < myParameters.GMaxSeg())
                            {
                                MaxSeg = myParameters.GMaxSeg();
                                aC = aC1;
                                // OCCT: continues the inner loop (one retry), and a failed retry falls through to
                                // success below (occt-issues.md)
                                continue;
                            }
                            else
                            {
                                if (IsConvert)
                                {
                                    C = (Curve3D)aCurve.Copy();
                                    TolCur = global::Kerf.Engine.Foundation.Precision.Confusion();
                                    return true;
                                }
                            }
                        }
                        if (!myDeg
                            && ((Nbseg > myParameters.GMaxSeg()) || !Done
                                || (anApprox.MaxError() >= Max(TolCur, myTol3d))))
                        {
                            if (MaxDeg < myParameters.GMaxDegree())
                            {
                                MaxDeg = myParameters.GMaxDegree();
                                aC = aC1;
                                continue;
                            }
                            else
                            {
                                if (IsConvert)
                                {
                                    C = (Curve3D)aCurve.Copy();
                                    TolCur = global::Kerf.Engine.Foundation.Precision.Confusion();
                                    return true;
                                }
                            }
                        }
                        myConvert = true;
                        TolCur = anApprox.MaxError();
                        myCurve3dError = Max(myCurve3dError, anApprox.MaxError());
                        return true;
                    }
                }
                catch (Exception anException) when (IsCaughtFailure(anException))
                {
                    continue;
                }
            }
            return false;
        }
        else
        {
            if (IsConvert)
            {
                C = (Curve3D)aCurve.Copy();
                TolCur = global::Kerf.Engine.Foundation.Precision.Confusion();
                return true;
            }
            return false;
        }
    }

    /// <summary>
    /// True if the edge's pcurve on the face is approximated, or (C then a copy of the pcurve) if the face's surface or
    /// the edge's 3D curve or another pcurve is.
    /// </summary>
    public override bool NewCurve2d(in Edge E,
                                    in Face F,
                                    in Edge NewE,
                                    in Face NewF,
                                    out Curve2D? C,
                                    out double Tol)
    {
        C = null;
        Tol = 0.0;
        if (!myApproxCurve2dFlag && !myApproxSurfaceFlag)
        {
            return false;
        }
        Surface? aSurface = Tool.Surface(F, out Placement L);
        SurfaceAdaptor AdS = new SurfaceAdaptor(aSurface!);
        double TolCur =
            Min(AdS.UResolution(Tool.Tolerance(E)), AdS.VResolution(Tool.Tolerance(E)));
        Curve2D? aCurve = Tool.CurveOnSurface(E, F, out double First, out double Last);
        if (aCurve is null)
        {
            return false;
        }
        Curve3D? aCur3d = Tool.Curve(E, out Placement Loc1, out double F1, out double L1);
        //  bool IsConvert = (IsConvertSurface(aSurface,myMaxDegree,myNbMaxSeg) ||
        //  !E.IsSame(NewE));

        bool IsConvert =
            ((myApproxSurfaceFlag
              && IsConvertSurface(aSurface, myMaxDegree, myNbMaxSeg, myRational, myParameters))
             || (myApproxCurve3dFlag
                 && IsConvertCurve3d(aCur3d, myMaxDegree, myNbMaxSeg, myRational, myParameters)));

        if (!IsConvert)
        {
            BrepTEdge TE = (BrepTEdge)E.TShape()!;
            // iterate on pcurves
            var itcr = new NCollection_List<CurveRepresentation>.Iterator(TE.Curves());
            for (; itcr.More(); itcr.Next())
            {
                if (itcr.Value() is not GCurve GC || !GC.IsCurveOnSurface())
                {
                    continue;
                }
                Surface? aSurf = GC.Surface();
                Curve2D? aCur2d = GC.PCurve();
                if ((myApproxSurfaceFlag
                     && IsConvertSurface(aSurf, myMaxDegree, myNbMaxSeg, myRational, myParameters))
                    || (myApproxCurve2dFlag
                        && IsConvertCurve2d(aCur2d, myMaxDegree, myNbMaxSeg, myRational, myParameters)))
                {
                    IsConvert = true;
                    break;
                }
            }
        }
        if (!myApproxCurve2dFlag)
        {
            if (IsConvert)
            {
                C = (Curve2D)aCurve.Copy();
                return true;
            }
            else
            {
                return false;
            }
        }
        bool IsOf = true;
        if (myParameters.ConvertOffsetCurv2d())
        {
            IsOf = false;
        }
        bool IsConv = ConvertCurve2d(aCurve, ref C, IsConvert, First, Last, ref TolCur, IsOf);

        Tol = Tool.Tolerance(E); // TolCur;
        BrepBuilder B = new BrepBuilder();
        if (!IsConv && !NewE.IsSame(E))
        {
            // OCCT.NET: NewE is the modifier's new edge (or its dummy), never the caller's (decision 973)
            B.Range(NewE, First, Last);
        }
        return IsConv;
    }

    /// <summary>
    /// True if the pcurve is converted, or copied (<paramref name="IsConvert"/>): then C is the result (left
    /// unchanged otherwise) and TolCur its error. With <paramref name="IsOf"/>, offset pcurves are approximated as
    /// offsets of approximated bases, else as B-splines.
    /// </summary>
    public bool ConvertCurve2d(Curve2D aCurve,
                               ref Curve2D? C,
                               bool IsConvert,
                               double First,
                               double Last,
                               ref double TolCur,
                               bool IsOf = true)
    {
        // TolCur = Precision::PConfusion();
        if (aCurve is TrimmedCurve2D tmpT)
        {
            Curve2D BasCurve = tmpT.BasisCurve();
            Curve2D? ResCurve = null;
            if (ConvertCurve2d(BasCurve, ref ResCurve, IsConvert, First, Last, ref TolCur, IsOf))
            {
                C = ResCurve;
                return true;
            }
            else
            {
                if (IsConvert)
                {
                    C = (Curve2D)aCurve.Copy();
                    TolCur = global::Kerf.Engine.Foundation.Precision.PConfusion();
                    return true;
                }
                else
                {
                    return false;
                }
            }
        }

        if (aCurve is LineCurve2D aLine2d && myParameters.ConvertCurve2d())
        {
            NCollection_Array1<Point2D> poles = new(1, 2);
            poles[1] = aLine2d.Value(First);
            poles[2] = aLine2d.Value(Last);
            NCollection_Array1<double> knots = new(1, 2);
            knots[1] = First;
            knots[2] = Last;
            NCollection_Array1<int> mults = new(1, 2);
            mults.Init(2);
            BSplineCurve2D res = new BSplineCurve2D(poles, knots, mults, 1);
            C = res;
            return true;
        }

        if (aCurve is ConicCurve2D && myParameters.ConvertCurve2d())
        {
            BSplineCurve2D aBSpline2d;
            Curve2D tcurve = new TrimmedCurve2D(aCurve, First, Last); // protection against parabols ets
            ApproxCurve2D approx = new ApproxCurve2D(tcurve, myTol2d, myContinuity2d, myNbMaxSeg, 6);
            if (approx.HasResult)
            {
                aBSpline2d = approx.InternalCurve()!;
            }
            else
            {
                aBSpline2d = Geom2dConvert2D.CurveToBSplineCurve(tcurve, QuasiAngular);
            }

            double Shift = First - aBSpline2d.FirstParameter();
            if (Math.Abs(Shift) > global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                NCollection_Array1<double> newKnots = new(aBSpline2d.InternalKnots());
                for (int i = 1; i <= newKnots.Length(); i++)
                {
                    newKnots[i] += Shift;
                }
                aBSpline2d.SetKnots(newKnots);
            }
            Curve2D? ResCurve = null;
            if (ConvertCurve2d(aBSpline2d, ref ResCurve, IsConvert, First, Last, ref TolCur, false))
            {
                C = ResCurve;
                return true;
            }
            else
            {
                C = aBSpline2d;
                TolCur = global::Kerf.Engine.Foundation.Precision.PConfusion();
                return true;
            }
        }

        if (aCurve is BezierCurve2D && myParameters.ConvertCurve2d())
        {
            Curve2D aBSpline2d = Geom2dConvert2D.CurveToBSplineCurve(aCurve, QuasiAngular);
            Curve2D? ResCurve = null;
            if (ConvertCurve2d(aBSpline2d, ref ResCurve, IsConvert, First, Last, ref TolCur, false))
            {
                C = ResCurve;
                return true;
            }
            else
            {
                C = aBSpline2d;
                TolCur = global::Kerf.Engine.Foundation.Precision.PConfusion();
                return true;
            }
        }

        if (aCurve is OffsetCurve2D tmpO && IsOf)
        {
            Curve2D BasCurve = tmpO.BasisCurve();
            Curve2D? ResCurve = null;
            if (ConvertCurve2d(BasCurve, ref ResCurve, IsConvert, First, Last, ref TolCur))
            {
                if (ResCurve!.Continuity() != C0)
                {
                    C = new OffsetCurve2D(ResCurve, tmpO.Offset());
                    return true;
                }
                // OCCT: converts into ResCurve, not C, so C is left unchanged with true returned (occt-issues.md)
                else if (ConvertCurve2d(aCurve, ref ResCurve, IsConvert, First, Last, ref TolCur, false))
                {
                    return true;
                }
                else
                {
                    if (IsConvert)
                    {
                        C = (Curve2D)aCurve.Copy();
                        TolCur = global::Kerf.Engine.Foundation.Precision.PConfusion();
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
                if (IsConvert)
                {
                    C = (Curve2D)aCurve.Copy();
                    TolCur = global::Kerf.Engine.Foundation.Precision.PConfusion();
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
            || ((aCurve is OffsetCurve2D) && !IsOf))
        {
            int Deg = 1;

            if (aCurve is BSplineCurve2D BsC)
            {
                Deg = BsC.Degree();
                bool IsR = (myRational && BsC.IsRational);
                if (!IsR && Deg <= myMaxDegree && (BsC.KnotCount - 1) <= myNbMaxSeg)
                {
                    if (IsConvert)
                    {
                        C = (Curve2D)aCurve.Copy();
                        TolCur = global::Kerf.Engine.Foundation.Precision.PConfusion();
                        return true;
                    }
                    else
                    {
                        return false;
                    }
                }
            }
            if (aCurve is BezierCurve2D BzC)
            {
                Deg = BzC.Degree();
                bool IsR = (myRational && BzC.IsRational);
                if (!IsR && Deg <= myMaxDegree)
                {
                    if (IsConvert)
                    {
                        C = (Curve2D)aCurve.Copy();
                        TolCur = global::Kerf.Engine.Foundation.Precision.PConfusion();
                        return true;
                    }
                    else
                    {
                        return false;
                    }
                }
            }
            Curve2D aCurve1;
            double pf = aCurve.FirstParameter(), pl = aCurve.LastParameter();
            // 15.11.2002 PTV OCC966
            if (ShapeAnalysis_Curve.IsPeriodic(aCurve) && (First != Last))
            {
                aCurve1 = new TrimmedCurve2D(aCurve, First, Last);
            }
            else if (aCurve.FirstParameter() < (First - global::Kerf.Engine.Foundation.Precision.PConfusion())
                     || aCurve.LastParameter() > (Last + global::Kerf.Engine.Foundation.Precision.PConfusion()))
            {
                double F = Max(First, pf), L = Min(Last, pl);
                if (F != L)
                {
                    aCurve1 = new TrimmedCurve2D(aCurve, F, L);
                }
                else
                {
                    aCurve1 = aCurve;
                }
            }
            else
            {
                aCurve1 = aCurve;
            }
            int aC = Min(ContToInteger(myContinuity2d), ContToInteger(aCurve.Continuity()));
            if (aC == 0)
            {
                aC = ContToInteger(myContinuity2d);
            }
            // aC = std::min(aC,(Deg -1));
            int aC1 = aC;
            int MaxSeg = myNbMaxSeg;
            int MaxDeg = myMaxDegree;
            for (; aC >= 0; aC--)
            {
                try
                {
                    Continuity aCont = IntegerToGeomAbsShape(aC);
                    for (int j = 1; j <= 2; j++)
                    {
                        ApproxCurve2D anApprox = new ApproxCurve2D(aCurve1, myTol2d, aCont, MaxSeg, MaxDeg);
                        bool Done = anApprox.IsDone;
                        C = anApprox.InternalCurve();
                        // OCCT: dereferences the curve, null when the approximation has no result (an access
                        // violation natively, caught as a failure: decision 917)
                        int Nbseg = ((BSplineCurve2D)C!).KnotCount - 1;
                        int DegC = ((BSplineCurve2D)C).Degree();

                        if (myDeg
                            && ((DegC > MaxDeg) || !Done || (anApprox.MaxError() >= Max(myTol2d, TolCur))))
                        {
                            if (MaxSeg < myParameters.GMaxSeg())
                            {
                                MaxSeg = myParameters.GMaxSeg();
                                aC = aC1;
                                // OCCT: continues the inner loop with the same aCont (occt-issues.md)
                                continue;
                            }
                            else
                            {
                                if (IsConvert)
                                {
                                    C = (Curve2D)aCurve.Copy();
                                    TolCur = global::Kerf.Engine.Foundation.Precision.PConfusion();
                                    return true;
                                }
                            }
                        }

                        if (!myDeg
                            && ((Nbseg >= MaxSeg) || !Done || (anApprox.MaxError() >= Max(myTol2d, TolCur))))
                        {
                            if (MaxDeg < myParameters.GMaxDegree())
                            {
                                MaxDeg = myParameters.GMaxDegree();
                                aC = aC1;
                                continue;
                            }
                            else
                            {
                                if (IsConvert)
                                {
                                    C = (Curve2D)aCurve.Copy();
                                    TolCur = global::Kerf.Engine.Foundation.Precision.PConfusion();
                                    return true;
                                }
                            }
                        }
                        myConvert = true;
                        TolCur = anApprox.MaxError();
                        myCurve2dError = Max(myCurve2dError, anApprox.MaxError());
                        return true;
                    }
                }
                catch (Exception anException) when (IsCaughtFailure(anException))
                {
                    continue;
                }
            }
            return false;
        }
        else
        {
            if (IsConvert)
            {
                C = (Curve2D)aCurve.Copy();
                TolCur = global::Kerf.Engine.Foundation.Precision.PConfusion();
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
    public override bool NewPoint(in Vertex V, out Point3D P, out double Tol)
    {
        P = new Point3D();
        Tol = Tool.Tolerance(V);
        // OCCT: myConvert is the state of the last conversion before the call (occt-issues.md)
        if (myConvert)
        {
            Point3D p1 = new Point3D(Tool.Pnt(V).XYZ());
            P = p1;
            return true;
        }
        else
        {
            return false;
        }
    }

    /// <summary>False: parameters are never modified.</summary>
    public override bool NewParameter(in Vertex V, in Edge E, ref double P, ref double Tol)
    {
        return false;
    }

    /// <summary>The continuity of the edge between the faces, unchanged.</summary>
    public override Continuity Continuity(in Edge E,
                                             in Face F1,
                                             in Face F2,
                                             in Edge NewE,
                                             in Face NewF1,
                                             in Face NewF2)
    {
        return Tool.Continuity(E, F1, F2);
    }

    /// <summary>The largest errors of the approximations of surfaces (returned), 3D curves and pcurves.</summary>
    public double MaxErrors(out double aCurve3dErr, out double aCurve2dErr)
    {
        aCurve3dErr = myCurve3dError;
        aCurve2dErr = myCurve2dError;
        return mySurfaceError;
    }

    /// <summary>The number of spans of the approximated surfaces.</summary>
    public int NbOfSpan()
    {
        return myNbOfSpan;
    }

    /// <summary>Sets the tolerance of approximation of 3D curves and surfaces.</summary>
    public void SetTol3d(double Tol3d)
    {
        myTol3d = Tol3d;
    }

    /// <summary>Sets the tolerance of approximation of pcurves.</summary>
    public void SetTol2d(double Tol2d)
    {
        myTol2d = Tol2d;
    }

    /// <summary>Whether surfaces are approximated, for reading and writing.</summary>
    public ref bool ModifyApproxSurfaceFlag() => ref myApproxSurfaceFlag;

    /// <summary>Whether 3D curves are approximated, for reading and writing.</summary>
    public ref bool ModifyApproxCurve3dFlag() => ref myApproxCurve3dFlag;

    /// <summary>Whether pcurves are approximated, for reading and writing.</summary>
    public ref bool ModifyApproxCurve2dFlag() => ref myApproxCurve2dFlag;

    /// <summary>Sets the continuity of the approximation of 3D curves and surfaces.</summary>
    public void SetContinuity3d(Continuity Continuity3d)
    {
        myContinuity3d = Continuity3d;
    }

    /// <summary>Sets the continuity of the approximation of pcurves.</summary>
    public void SetContinuity2d(Continuity Continuity2d)
    {
        myContinuity2d = Continuity2d;
    }

    /// <summary>Sets the maximal degree of approximation.</summary>
    public void SetMaxDegree(int MaxDegree)
    {
        myMaxDegree = MaxDegree;
    }

    /// <summary>Sets the maximal number of spans of approximation.</summary>
    public void SetMaxNbSegments(int MaxNbSegments)
    {
        myNbMaxSeg = MaxNbSegments;
    }

    /// <summary>
    /// Sets the priority: true keeps the degree under the maximal one at the expense of the number of spans, false the
    /// number of spans at the expense of the degree.
    /// </summary>
    public void SetPriority(bool Degree)
    {
        myDeg = Degree;
    }

    /// <summary>Sets whether rational B-splines and Beziers are approximated by polynomial ones even within the limits.</summary>
    public void SetConvRational(bool Rational)
    {
        myRational = Rational;
    }

    /// <summary>The parameters saying which geometry to convert.</summary>
    public ShapeCustom_RestrictionParameters GetRestrictionParameters()
    {
        return myParameters;
    }

    /// <summary>Sets the parameters saying which geometry to convert.</summary>
    public void SetRestrictionParameters(ShapeCustom_RestrictionParameters aModes)
    {
        myParameters = aModes;
    }

    /// <summary>The largest error of approximation of 3D curves.</summary>
    public double Curve3dError()
    {
        return myCurve3dError;
    }

    /// <summary>The largest error of approximation of pcurves.</summary>
    public double Curve2dError()
    {
        return myCurve2dError;
    }

    /// <summary>The largest error of approximation of surfaces.</summary>
    public double SurfaceError()
    {
        return mySurfaceError;
    }
}
