// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_BSplineRestriction.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_BSplineRestriction.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_BSplineRestriction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Convert_ParameterisationType;
using static Occt.GeomAbs_Shape;

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
/// caller's edge) and into the B-splines its approximations create (decision 8101).
/// </para>
/// </remarks>
public class ShapeCustom_BSplineRestriction : ShapeCustom_Modification
{
    private GeomAbs_Shape myContinuity3d;
    private GeomAbs_Shape myContinuity2d;
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
        => theException is not (Standard_ImmutableObject or TopoDS_LockedShape)
           && theException is Standard_Failure or NullReferenceException;

    private static GeomAbs_Shape IntegerToGeomAbsShape(int i)
    {
        GeomAbs_Shape result = GeomAbs_C0;
        switch (i)
        {
            case 0:
                result = GeomAbs_C0;
                break;
            case 1:
                result = GeomAbs_C1;
                break;
            case 2:
                result = GeomAbs_C2;
                break;
            case 3:
                result = GeomAbs_C3;
                break;
            default:
                result = GeomAbs_CN;
                break;
        }
        return result;
    }

    private static int ContToInteger(GeomAbs_Shape Cont)
    {
        int result = 0;
        switch (Cont)
        {
            case GeomAbs_C0:
            case GeomAbs_G1:
                result = 0;
                break;
            case GeomAbs_C1:
            case GeomAbs_G2:
                result = 1;
                break;
            case GeomAbs_C2:
                result = 2;
                break;
            case GeomAbs_C3:
                result = 3;
                break;
            default:
                result = 4;
                break;
        }
        return result;
    }

    private static bool IsConvertCurve3d(Geom_Curve? aCurve,
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
        if (aCurve is Geom_TrimmedCurve tmp)
        {
            Geom_Curve BasCurve = tmp.BasisCurve();
            return IsConvertCurve3d(BasCurve, Degree, NbSeg, myRational, aParameters);
        }

        if (aCurve is Geom_OffsetCurve tmpO)
        {
            if (aParameters.ConvertOffsetCurv3d())
            {
                return true;
            }
            Geom_Curve BasCurve = tmpO.BasisCurve();
            return IsConvertCurve3d(BasCurve, Degree, NbSeg, myRational, aParameters);
        }
        if (aCurve is Geom_BSplineCurve BsC)
        {
            // OCCT: '>=' here, '>' for 2D curves (occt-issues.md)
            if (BsC.Degree() > Degree || ((BsC.NbKnots() - 1) >= NbSeg))
            {
                return true;
            }
            return myRational && BsC.IsRational();
        }
        if (aCurve is Geom_BezierCurve aBz
            && (aBz.Degree() > Degree || (myRational && aBz.IsRational())))
        {
            return true;
        }
        // else return false;
        return false;
    }

    private static bool IsConvertSurface(Geom_Surface? aSurface,
                                         int Degree,
                                         int NbSeg,
                                         bool myRational,
                                         ShapeCustom_RestrictionParameters aParameters)
    {
        if (aSurface is null)
        {
            return false;
        }
        if (aSurface is Geom_Plane)
        {
            return aParameters.ConvertPlane();
        }
        else if (aSurface is Geom_ConicalSurface)
        {
            return aParameters.ConvertConicalSurf();
        }
        else if (aSurface is Geom_SphericalSurface)
        {
            return aParameters.ConvertSphericalSurf();
        }
        else if (aSurface is Geom_CylindricalSurface)
        {
            return aParameters.ConvertCylindricalSurf();
        }
        else if (aSurface is Geom_ToroidalSurface)
        {
            return aParameters.ConvertToroidalSurf();
        }

        // else if(aSurface->IsKind(STANDARD_TYPE(Geom_ElementarySurface)))    {
        //   return aParameters->ConvertElementarySurf();
        //  }
        if (aSurface is Geom_SweptSurface aSwept)
        {
            if (aSurface is Geom_SurfaceOfRevolution && aParameters.ConvertRevolutionSurf())
            {
                return true;
            }
            if (aSurface is Geom_SurfaceOfLinearExtrusion && aParameters.ConvertExtrusionSurf())
            {
                return true;
            }
            Geom_Curve BasCurve = aSwept.BasisCurve();
            return IsConvertCurve3d(BasCurve, Degree, NbSeg, myRational, aParameters);
        }
        if (aSurface is Geom_RectangularTrimmedSurface aRTS)
        {
            Geom_Surface theSurf = aRTS.BasisSurface();
            return IsConvertSurface(theSurf, Degree, NbSeg, myRational, aParameters);
        }
        if (aSurface is Geom_OffsetSurface aOS)
        {
            if (aParameters.ConvertOffsetSurf())
            {
                return true;
            }
            Geom_Surface theSurf = aOS.BasisSurface();
            return IsConvertSurface(theSurf, Degree, NbSeg, myRational, aParameters);
        }
        if (aSurface is Geom_BSplineSurface theBSurf)
        {
            if (theBSurf.UDegree() > Degree || theBSurf.VDegree() > Degree)
            {
                return true;
            }
            if ((theBSurf.NbUKnots() - 1) * (theBSurf.NbVKnots() - 1) > NbSeg)
            {
                return true;
            }
            if (myRational && (theBSurf.IsURational() || theBSurf.IsVRational()))
            {
                return true;
            }
            return false;
        }

        if (aSurface is Geom_BezierSurface theBzSurf)
        {
            if (aParameters.ConvertBezierSurf())
            {
                return true;
            }
            if (theBzSurf.UDegree() > Degree || theBzSurf.VDegree() > Degree)
            {
                return true;
            }
            if (myRational && (theBzSurf.IsURational() || theBzSurf.IsVRational()))
            {
                return true;
            }
            return false;
        }
        return false;
    }

    private static bool IsConvertCurve2d(Geom2d_Curve? aCurve,
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
        if (aCurve is Geom2d_TrimmedCurve tmp)
        {
            Geom2d_Curve BasCurve = tmp.BasisCurve();
            return IsConvertCurve2d(BasCurve, Degree, NbSeg, myRational, aParameters);
        }
        if (aCurve is Geom2d_OffsetCurve tmpO)
        {
            if (aParameters.ConvertOffsetCurv2d())
            {
                return true;
            }
            Geom2d_Curve BasCurve = tmpO.BasisCurve();
            return IsConvertCurve2d(BasCurve, Degree, NbSeg, myRational, aParameters);
        }
        if (aCurve is Geom2d_BSplineCurve aBs
            && ((aBs.Degree() > Degree || ((aBs.NbKnots() - 1) > NbSeg))
                || (myRational && aBs.IsRational())))
        {
            return true;
        }
        if (aCurve is Geom2d_BezierCurve aBz
            && ((aBz.Degree() > Degree) || (myRational && aBz.IsRational())))
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
        myContinuity3d = GeomAbs_C1;
        myContinuity2d = GeomAbs_C2;
        myMaxDegree = 9;
        myNbMaxSeg = 10000;
        mySurfaceError = Precision.Confusion();
        myCurve3dError = Precision.Confusion();
        myCurve2dError = Precision.PConfusion();
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
                                          GeomAbs_Shape aContinuity3d,
                                          GeomAbs_Shape aContinuity2d,
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
        mySurfaceError = Precision.Confusion();
        myCurve3dError = Precision.Confusion();
        myCurve2dError = Precision.PConfusion();
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
                                          GeomAbs_Shape aContinuity3d,
                                          GeomAbs_Shape aContinuity2d,
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
        mySurfaceError = Precision.Confusion();
        myCurve3dError = Precision.Confusion();
        myCurve2dError = Precision.PConfusion();
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
    public override bool NewSurface(in TopoDS_Face F,
                                    out Geom_Surface? S,
                                    out TopLoc_Location L,
                                    out double Tol,
                                    out bool RevWires,
                                    out bool RevFace)
    {
        S = null;
        Tol = 0.0;
        RevWires = RevFace = false;
        if (!myApproxSurfaceFlag)
        {
            L = new TopLoc_Location();
            return false;
        }
        RevWires = false;
        RevFace = false;
        myConvert = false;
        Geom_Surface? aSurface = BRep_Tool.Surface(F, out L);
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
        BRepTools.UVBounds(F, out double Umin, out double Umax, out double Vmin, out double Vmax);
        if (myParameters.SegmentSurfaceMode())
        {
            UF = Umin;
            UL = Umax;
            VF = Vmin;
            VL = Vmax;
        }
        else
        {
            if (Precision.IsInfinite(UF) || Precision.IsInfinite(UL))
            {
                UF = Umin;
                UL = Umax;
            }
            if (Precision.IsInfinite(VF) || Precision.IsInfinite(VL))
            {
                VF = Vmin;
                VL = Vmax;
            }
        }

        bool IsConv = ConvertSurface(aSurface, ref S, UF, UL, VF, VL, IsOf);
        Tol = Precision.Confusion(); // mySurfaceError;

        if (IsConv)
        {
            bool wasBSpline = aSurface is Geom_BSplineSurface;
            if (aSurface is Geom_RectangularTrimmedSurface rts)
            {
                wasBSpline = rts.BasisSurface() is Geom_BSplineSurface;
            }

            if (wasBSpline)
            {
                SendMsg(F, new Message_Msg("BSplineRestriction.NewSurface.MSG1"));
            }
            else
            {
                SendMsg(F, new Message_Msg("BSplineRestriction.NewSurface.MSG0"));
            }
        }

        return IsConv;
    }

    private static void ConvertExtrusion(Geom_Curve? C, /*const gp_Dir& direction,*/
                                         in gp_Trsf shiftF,
                                         in gp_Trsf shiftL,
                                         double VF,
                                         double VL,
                                         out Geom_Surface bspline)
    {
        // OCCT: dereferences C, null when the approximation of the basis curve failed (an access violation natively)
        Geom_BSplineCurve bspl = (Geom_BSplineCurve)C!;
        int nbPoles = bspl.NbPoles();
        NCollection_Array1<gp_Pnt> poles = bspl.InternalPoles();
        NCollection_Array1<double> weights = bspl.InternalWeightsArray();
        NCollection_Array1<double> knots = bspl.InternalKnots();
        NCollection_Array1<int> mults = bspl.InternalMultiplicities();

        NCollection_Array2<gp_Pnt> resPoles = new(1, nbPoles, 1, 2);
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

        bspline = new Geom_BSplineSurface(resPoles,
                                          resWeigth,
                                          knots,
                                          vknots,
                                          mults,
                                          vmults,
                                          bspl.Degree(),
                                          1,
                                          bspl.IsPeriodic(),
                                          false);
    }

    /// <summary>
    /// True if the surface is converted: then S is the result (left unchanged otherwise). With <paramref name="IsOf"/>,
    /// offset surfaces are approximated as offsets of approximated bases, else as B-splines.
    /// </summary>
    public bool ConvertSurface(Geom_Surface aSurface,
                               ref Geom_Surface? S,
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

        Geom_Surface aSurf = aSurface;
        if (aSurf is Geom_Plane pln && myParameters.ConvertPlane())
        {
            NCollection_Array2<gp_Pnt> poles = new(1, 2, 1, 2);
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
            S = new Geom_BSplineSurface(poles, uknots, vknots, umults, vmults, 1, 1, false, false);
            return true;
        }
        if (aSurf is Geom_SurfaceOfRevolution SurfaceR)
        {
            Geom_Curve BasCurve = SurfaceR.BasisCurve();
            Geom_Curve? ResCurve = null;
            double TolS = Precision.Confusion();
            if (myParameters.ConvertRevolutionSurf())
            {
                if (BasCurve is Geom_OffsetCurve)
                {
                    GeomAbs_Shape cnt = BasCurve.Continuity();
                    cnt = (cnt > GeomAbs_C2 ? GeomAbs_C2 : cnt);
                    if (ConvertCurve(BasCurve,
                                     ref ResCurve,
                                     false,
                                     Max(VF, BasCurve.FirstParameter()),
                                     Min(VL, BasCurve.LastParameter()),
                                     ref TolS,
                                     false))
                    {
                        Geom_SurfaceOfRevolution newRevol = new Geom_SurfaceOfRevolution(ResCurve!, SurfaceR.Axis());
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
                    S = new Geom_SurfaceOfRevolution(ResCurve!, SurfaceR.Axis());
                    return true;
                }
                else
                {
                    return false;
                }
            }
        }
        if (aSurf is Geom_SurfaceOfLinearExtrusion SurfaceE)
        {
            Geom_Curve? BasCurve = SurfaceE.BasisCurve();
            Geom_Curve? ResCurve = null;
            double TolS = Precision.Confusion();
            if (myParameters.ConvertExtrusionSurf())
            {
                GeomAbs_Shape cnt = SurfaceE.Continuity();
                cnt = (cnt > GeomAbs_C2 ? GeomAbs_C2 : cnt);
                Geom_BSplineCurve? bspl =
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
                gp_Trsf shiftF = new gp_Trsf(), shiftL = new gp_Trsf();
                shiftF.SetTranslation(SurfaceE.Value(UF, 0), SurfaceE.Value(UF, VF));
                shiftL.SetTranslation(SurfaceE.Value(UF, 0), SurfaceE.Value(UF, VL));
                ConvertExtrusion(ResCurve, /*Surface->Direction(),*/ shiftF, shiftL, VF, VL, out Geom_Surface aBSpline);
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
                    S = new Geom_SurfaceOfLinearExtrusion(ResCurve!, SurfaceE.Direction());
                    return true;
                }
                else
                {
                    return false;
                }
            }
        }
        if (aSurf is Geom_RectangularTrimmedSurface tmpR)
        {
            tmpR.Bounds(out double U1, out double U2, out double V1, out double V2);
            Geom_Surface theSurf = tmpR.BasisSurface();
            Geom_Surface? ResSurface = null;
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
        if (aSurf is Geom_OffsetSurface tmpO && IsOf)
        {
            Geom_Surface theSurf = tmpO.BasisSurface();
            Geom_Surface? ResSurface = null;
            if (ConvertSurface(theSurf, ref ResSurface, UF, UL, VF, VL))
            {
                if (ResSurface!.Continuity() != GeomAbs_C0)
                {
                    S = new Geom_OffsetSurface(ResSurface, tmpO.Offset());
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
        if (aSurf is Geom_BezierSurface bezier && myParameters.ConvertBezierSurf())
        {
            int uDegree = bezier.UDegree();
            int vDegree = bezier.VDegree();
            NCollection_Array2<gp_Pnt> aPoles = bezier.InternalPoles();
            NCollection_Array2<double> aWeights = bezier.InternalWeightsArray();
            NCollection_Array1<double> uKnots = new(1, 2), vKnots = new(1, 2);
            uKnots[1] = 0;
            uKnots[2] = 1;
            vKnots[1] = 0;
            vKnots[2] = 1;
            NCollection_Array1<int> uMults = new(1, 2), vMults = new(1, 2);
            uMults.Init(uDegree + 1);
            vMults.Init(vDegree + 1);
            Geom_BSplineSurface bspline =
                new Geom_BSplineSurface(aPoles, aWeights, uKnots, vKnots, uMults, vMults, uDegree, vDegree);

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
        if (aSurf is Geom_BSplineSurface BsS)
        {
            UDeg = BsS.UDegree();
            VDeg = BsS.VDegree();
            NbSeg = (BsS.NbUKnots() - 1) * (BsS.NbVKnots() - 1);
            URat = BsS.IsURational();
            VRat = BsS.IsVRational();
            bool IsR = (myRational && (URat || VRat));
            if (UDeg <= myMaxDegree && VDeg <= myMaxDegree && NbSeg <= myNbMaxSeg && !IsR)
            {
                return false;
            }
        }
        if (aSurf is Geom_BezierSurface BsZ)
        {
            UDeg = BsZ.UDegree();
            VDeg = BsZ.VDegree();
            NbSeg = 1;
            URat = BsZ.IsURational();
            VRat = BsZ.IsVRational();
            bool IsR = (myRational && (URat || VRat));
            if (UDeg <= myMaxDegree && VDeg <= myMaxDegree && NbSeg <= myNbMaxSeg && !IsR)
            {
                return false;
            }
        }
        GeomAbs_Shape Cont = myContinuity3d;
        if (aSurf is Geom_OffsetSurface)
        {
            Cont = GeomAbs_C0;
        }

        double aTol3d;
        int nbOfSpan, imax = 10;
        int MaxSeg = myNbMaxSeg;
        int MaxDeg = myMaxDegree;
        aSurf.Bounds(out double u1, out double u2, out double v1, out double v2);
        double ShiftU = 0, ShiftV = 0;
        if (Math.Abs(u1 - UF) > Precision.PConfusion() || Math.Abs(u2 - UL) > Precision.PConfusion()
            || Math.Abs(v1 - VF) > Precision.PConfusion() || Math.Abs(v2 - VL) > Precision.PConfusion())
        {
            bool isTrim = false;
            if (!aSurf.IsUPeriodic())
            { // else {
                u1 = Max(u1, UF);
                u2 = Min(u2, UL);
                isTrim = true;
            }
            if (!aSurf.IsVPeriodic())
            { // else
                v1 = Max(v1, VF);
                v2 = Min(v2, VL);
                isTrim = true;
            }

            if (isTrim && (u1 != u2) && (v1 != v2))
            {
                Geom_RectangularTrimmedSurface trSurface = new Geom_RectangularTrimmedSurface(aSurf, u1, u2, v1, v2);
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
                        GeomAbs_Shape aContV = IntegerToGeomAbsShape(aCV);
                        GeomAbs_Shape aContU = IntegerToGeomAbsShape(aCU);

                        GeomConvert_ApproxSurface anApprox = new GeomConvert_ApproxSurface(aSurf, aTol3d, aContU, aContV, MaxDeg, MaxDeg, MaxSeg, 0);
                        bool Done = anApprox.IsDone();
                        newTol = anApprox.MaxError();
                        if (anApprox.MaxError() <= myTol3d && Done)
                        {
                            nbOfSpan = (anApprox.Surface()!.NbUKnots() - 1) * (anApprox.Surface()!.NbVKnots() - 1);
                            S = anApprox.Surface();
                            Geom_BSplineSurface Bsc = (Geom_BSplineSurface)S!;
                            // OCCT.NET: the approximation's surface is frozen (decision 197); OCCT changes it in place
                            // below, the port changes a copy (decision 8102)
                            if (aSurface.IsUPeriodic() || aSurface.IsVPeriodic() || Math.Abs(ShiftU) > Precision.PConfusion()
                                || Math.Abs(ShiftV) > Precision.PConfusion())
                            {
                                Bsc = (Geom_BSplineSurface)Bsc.Copy();
                                S = Bsc;
                            }
                            if (aSurface.IsUPeriodic())
                            {
                                Bsc.SetUPeriodic();
                            }
                            if (aSurface.IsVPeriodic())
                            {
                                Bsc.SetVPeriodic();
                            }
                            myConvert = true;
                            myNbOfSpan = myNbOfSpan + nbOfSpan;
                            mySurfaceError = Max(mySurfaceError, anApprox.MaxError());
                            if (Math.Abs(ShiftU) > Precision.PConfusion())
                            {
                                NCollection_Array1<double> uknots = new(Bsc.InternalUKnots());
                                for (int j = 1; j <= uknots.Length(); j++)
                                {
                                    uknots[j] += ShiftU;
                                }
                                Bsc.SetUKnots(uknots);
                            }
                            if (Math.Abs(ShiftV) > Precision.PConfusion())
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
    public override bool NewCurve(in TopoDS_Edge E, out Geom_Curve? C, out TopLoc_Location L, out double Tol)
    {
        C = null;
        Tol = 0.0;
        if (!myApproxCurve3dFlag)
        {
            L = new TopLoc_Location();
            return false;
        }
        Geom_Curve? aCurve = BRep_Tool.Curve(E, out L, out double First, out double Last);
        double TolCur = BRep_Tool.Tolerance(E);
        // if(aCurve.IsNull()) return false;
        bool IsConvert = false;
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
        // iterate on pcurves
        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(TE.Curves());
        for (; itcr.More(); itcr.Next())
        {
            if (itcr.Value() is not BRep_GCurve GC || !GC.IsCurveOnSurface())
            {
                continue;
            }
            Geom_Surface? aSurface = GC.Surface();
            Geom2d_Curve? aCurve2d = GC.PCurve();
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
        Tol = BRep_Tool.Tolerance(E); // TolCur;

        if (IsConv)
        {
            bool wasBSpline = aCurve is Geom_BSplineCurve;
            if (aCurve is Geom_TrimmedCurve tc)
            {
                wasBSpline = tc.BasisCurve() is Geom_BSplineCurve;
            }

            if (wasBSpline)
            {
                SendMsg(E, new Message_Msg("BSplineRestriction.NewCurve.MSG1"));
            }
            else
            {
                SendMsg(E, new Message_Msg("BSplineRestriction.NewCurve.MSG0"));
            }
        }
        return IsConv;
    }

    /// <summary>
    /// True if the curve is converted, or copied (<paramref name="IsConvert"/>): then C is the result (left unchanged
    /// otherwise) and TolCur its error. With <paramref name="IsOf"/>, offset curves are approximated as offsets of
    /// approximated bases, else as B-splines.
    /// </summary>
    public bool ConvertCurve(Geom_Curve aCurve,
                             ref Geom_Curve? C,
                             bool IsConvert,
                             double First,
                             double Last,
                             ref double TolCur,
                             bool IsOf = true)
    {
        //  TolCur =  Precision::Confusion();
        if (aCurve is Geom_TrimmedCurve tmpT)
        {
            Geom_Curve BasCurve = tmpT.BasisCurve();
            Geom_Curve? ResCurve = null;
            if (ConvertCurve(BasCurve, ref ResCurve, IsConvert, First, Last, ref TolCur, IsOf))
            {
                C = ResCurve;
                return true;
            }
            else
            {
                if (IsConvert)
                {
                    C = (Geom_Curve)aCurve.Copy();
                    TolCur = Precision.Confusion();
                    return true;
                }

                return false;
            }
        }

        if (aCurve is Geom_Line aLine && myParameters.ConvertCurve3d())
        {
            NCollection_Array1<gp_Pnt> poles = new(1, 2);
            poles[1] = aLine.Value(First);
            poles[2] = aLine.Value(Last);
            NCollection_Array1<double> knots = new(1, 2);
            knots[1] = First;
            knots[2] = Last;
            NCollection_Array1<int> mults = new(1, 2);
            mults.Init(2);
            Geom_BSplineCurve res = new Geom_BSplineCurve(poles, knots, mults, 1);
            C = res;
            return true;
        }

        if (aCurve is Geom_Conic && myParameters.ConvertCurve3d())
        {
            Geom_BSplineCurve aBSpline;
            Geom_Curve tcurve = new Geom_TrimmedCurve(aCurve, First, Last); // protection against parabols ets
            GeomConvert_ApproxCurve approx = new GeomConvert_ApproxCurve(tcurve,
                                                                        myTol3d /*Precision::Approximation()*/,
                                                                        myContinuity2d,
                                                                        myNbMaxSeg,
                                                                        6);
            if (approx.HasResult())
            {
                aBSpline = approx.InternalCurve()!;
            }
            else
            {
                aBSpline = GeomConvert.CurveToBSplineCurve(tcurve, Convert_QuasiAngular);
            }

            double Shift = First - aBSpline.FirstParameter();
            if (Math.Abs(Shift) > Precision.PConfusion())
            {
                NCollection_Array1<double> newKnots = new(aBSpline.InternalKnots());
                for (int i = 1; i <= newKnots.Length(); i++)
                {
                    newKnots[i] += Shift;
                }
                aBSpline.SetKnots(newKnots);
            }
            Geom_Curve? ResCurve = null;
            if (ConvertCurve(aBSpline, ref ResCurve, IsConvert, First, Last, ref TolCur, false))
            {
                C = ResCurve;
                return true;
            }
            else
            {
                C = aBSpline;
                TolCur = Precision.PConfusion();
                return true;
            }
        }

        if (aCurve is Geom_BezierCurve && myParameters.ConvertCurve3d())
        {
            Geom_Curve aBSpline = GeomConvert.CurveToBSplineCurve(aCurve, Convert_QuasiAngular);
            Geom_Curve? ResCurve = null;
            if (ConvertCurve(aBSpline, ref ResCurve, IsConvert, First, Last, ref TolCur, false))
            {
                C = ResCurve;
                return true;
            }
            else
            {
                C = aBSpline;
                TolCur = Precision.PConfusion();
                return true;
            }
        }

        if (aCurve is Geom_OffsetCurve tmpO && IsOf)
        {
            Geom_Curve BasCurve = tmpO.BasisCurve();
            Geom_Curve? ResCurve = null;
            if (ConvertCurve(BasCurve, ref ResCurve, IsConvert, First, Last, ref TolCur))
            {
                if (ResCurve!.Continuity() != GeomAbs_C0)
                {
                    C = new Geom_OffsetCurve(ResCurve, tmpO.Offset(), tmpO.Direction());
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
                        C = (Geom_Curve)aCurve.Copy();
                        TolCur = Precision.Confusion();
                        return true;
                    }
                    return false;
                }
            }
            else
            {
                if (IsConvert)
                {
                    C = (Geom_Curve)aCurve.Copy();
                    TolCur = Precision.Confusion();
                    return true;
                }
                return false;
            }
        }
        if (aCurve is Geom_BSplineCurve
            || aCurve is Geom_BezierCurve
            || (aCurve is Geom_OffsetCurve && !IsOf))
        {
            int Deg = 1;

            if (aCurve is Geom_BSplineCurve BsC)
            {
                Deg = BsC.Degree();
                bool IsR = (myRational && BsC.IsRational());
                if (!IsR && Deg <= myMaxDegree && (BsC.NbKnots() - 1) <= myNbMaxSeg)
                {
                    if (IsConvert)
                    {
                        C = (Geom_Curve)aCurve.Copy();
                        TolCur = Precision.Confusion();
                        return true;
                    }
                    else
                    {
                        return false;
                    }
                }
            }
            if (aCurve is Geom_BezierCurve BzC)
            {
                Deg = BzC.Degree();
                bool IsR = (myRational && BzC.IsRational());
                if (!IsR && Deg <= myMaxDegree)
                {
                    if (IsConvert)
                    {
                        C = (Geom_Curve)aCurve.Copy();
                        TolCur = Precision.Confusion();
                        return true;
                    }
                    else
                    {
                        return false;
                    }
                }
            }
            Geom_Curve aCurve1;
            double pf = aCurve.FirstParameter(), pl = aCurve.LastParameter();
            // 15.11.2002 PTV OCC966
            if (ShapeAnalysis_Curve.IsPeriodic(aCurve) && (First != Last))
            {
                aCurve1 = new Geom_TrimmedCurve(aCurve, First, Last);
            }
            else if (pf < (First - Precision.PConfusion()) || pl > (Last + Precision.PConfusion()))
            {
                double F = Max(First, pf), L = Min(Last, pl);
                if (F != L)
                {
                    aCurve1 = new Geom_TrimmedCurve(aCurve, F, L);
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
                        GeomAbs_Shape aCont = IntegerToGeomAbsShape(aC);
                        GeomConvert_ApproxCurve anApprox = new GeomConvert_ApproxCurve(aCurve1, myTol3d, aCont, MaxSeg, MaxDeg);
                        bool Done = anApprox.IsDone();
                        C = anApprox.InternalCurve();
                        // OCCT: dereferences the curve, null when the approximation has no result (an access
                        // violation natively, caught as a failure: decision 917)
                        int Nbseg = ((Geom_BSplineCurve)C!).NbKnots() - 1;
                        int DegC = ((Geom_BSplineCurve)C).Degree();
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
                                    C = (Geom_Curve)aCurve.Copy();
                                    TolCur = Precision.Confusion();
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
                                    C = (Geom_Curve)aCurve.Copy();
                                    TolCur = Precision.Confusion();
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
                C = (Geom_Curve)aCurve.Copy();
                TolCur = Precision.Confusion();
                return true;
            }
            return false;
        }
    }

    /// <summary>
    /// True if the edge's pcurve on the face is approximated, or (C then a copy of the pcurve) if the face's surface or
    /// the edge's 3D curve or another pcurve is.
    /// </summary>
    public override bool NewCurve2d(in TopoDS_Edge E,
                                    in TopoDS_Face F,
                                    in TopoDS_Edge NewE,
                                    in TopoDS_Face NewF,
                                    out Geom2d_Curve? C,
                                    out double Tol)
    {
        C = null;
        Tol = 0.0;
        if (!myApproxCurve2dFlag && !myApproxSurfaceFlag)
        {
            return false;
        }
        Geom_Surface? aSurface = BRep_Tool.Surface(F, out TopLoc_Location L);
        GeomAdaptor_Surface AdS = new GeomAdaptor_Surface(aSurface!);
        double TolCur =
            Min(AdS.UResolution(BRep_Tool.Tolerance(E)), AdS.VResolution(BRep_Tool.Tolerance(E)));
        Geom2d_Curve? aCurve = BRep_Tool.CurveOnSurface(E, F, out double First, out double Last);
        if (aCurve is null)
        {
            return false;
        }
        Geom_Curve? aCur3d = BRep_Tool.Curve(E, out TopLoc_Location Loc1, out double F1, out double L1);
        //  bool IsConvert = (IsConvertSurface(aSurface,myMaxDegree,myNbMaxSeg) ||
        //  !E.IsSame(NewE));

        bool IsConvert =
            ((myApproxSurfaceFlag
              && IsConvertSurface(aSurface, myMaxDegree, myNbMaxSeg, myRational, myParameters))
             || (myApproxCurve3dFlag
                 && IsConvertCurve3d(aCur3d, myMaxDegree, myNbMaxSeg, myRational, myParameters)));

        if (!IsConvert)
        {
            BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
            // iterate on pcurves
            var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(TE.Curves());
            for (; itcr.More(); itcr.Next())
            {
                if (itcr.Value() is not BRep_GCurve GC || !GC.IsCurveOnSurface())
                {
                    continue;
                }
                Geom_Surface? aSurf = GC.Surface();
                Geom2d_Curve? aCur2d = GC.PCurve();
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
                C = (Geom2d_Curve)aCurve.Copy();
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

        Tol = BRep_Tool.Tolerance(E); // TolCur;
        BRep_Builder B = new BRep_Builder();
        if (!IsConv && !NewE.IsSame(E))
        {
            // OCCT.NET: NewE is the modifier's new edge (or its dummy), never the caller's (decision 8101)
            B.Range(NewE, First, Last);
        }
        return IsConv;
    }

    /// <summary>
    /// True if the pcurve is converted, or copied (<paramref name="IsConvert"/>): then C is the result (left
    /// unchanged otherwise) and TolCur its error. With <paramref name="IsOf"/>, offset pcurves are approximated as
    /// offsets of approximated bases, else as B-splines.
    /// </summary>
    public bool ConvertCurve2d(Geom2d_Curve aCurve,
                               ref Geom2d_Curve? C,
                               bool IsConvert,
                               double First,
                               double Last,
                               ref double TolCur,
                               bool IsOf = true)
    {
        // TolCur = Precision::PConfusion();
        if (aCurve is Geom2d_TrimmedCurve tmpT)
        {
            Geom2d_Curve BasCurve = tmpT.BasisCurve();
            Geom2d_Curve? ResCurve = null;
            if (ConvertCurve2d(BasCurve, ref ResCurve, IsConvert, First, Last, ref TolCur, IsOf))
            {
                C = ResCurve;
                return true;
            }
            else
            {
                if (IsConvert)
                {
                    C = (Geom2d_Curve)aCurve.Copy();
                    TolCur = Precision.PConfusion();
                    return true;
                }
                else
                {
                    return false;
                }
            }
        }

        if (aCurve is Geom2d_Line aLine2d && myParameters.ConvertCurve2d())
        {
            NCollection_Array1<gp_Pnt2d> poles = new(1, 2);
            poles[1] = aLine2d.Value(First);
            poles[2] = aLine2d.Value(Last);
            NCollection_Array1<double> knots = new(1, 2);
            knots[1] = First;
            knots[2] = Last;
            NCollection_Array1<int> mults = new(1, 2);
            mults.Init(2);
            Geom2d_BSplineCurve res = new Geom2d_BSplineCurve(poles, knots, mults, 1);
            C = res;
            return true;
        }

        if (aCurve is Geom2d_Conic && myParameters.ConvertCurve2d())
        {
            Geom2d_BSplineCurve aBSpline2d;
            Geom2d_Curve tcurve = new Geom2d_TrimmedCurve(aCurve, First, Last); // protection against parabols ets
            Geom2dConvert_ApproxCurve approx = new Geom2dConvert_ApproxCurve(tcurve, myTol2d, myContinuity2d, myNbMaxSeg, 6);
            if (approx.HasResult())
            {
                aBSpline2d = approx.InternalCurve()!;
            }
            else
            {
                aBSpline2d = Geom2dConvert.CurveToBSplineCurve(tcurve, Convert_QuasiAngular);
            }

            double Shift = First - aBSpline2d.FirstParameter();
            if (Math.Abs(Shift) > Precision.PConfusion())
            {
                NCollection_Array1<double> newKnots = new(aBSpline2d.InternalKnots());
                for (int i = 1; i <= newKnots.Length(); i++)
                {
                    newKnots[i] += Shift;
                }
                aBSpline2d.SetKnots(newKnots);
            }
            Geom2d_Curve? ResCurve = null;
            if (ConvertCurve2d(aBSpline2d, ref ResCurve, IsConvert, First, Last, ref TolCur, false))
            {
                C = ResCurve;
                return true;
            }
            else
            {
                C = aBSpline2d;
                TolCur = Precision.PConfusion();
                return true;
            }
        }

        if (aCurve is Geom2d_BezierCurve && myParameters.ConvertCurve2d())
        {
            Geom2d_Curve aBSpline2d = Geom2dConvert.CurveToBSplineCurve(aCurve, Convert_QuasiAngular);
            Geom2d_Curve? ResCurve = null;
            if (ConvertCurve2d(aBSpline2d, ref ResCurve, IsConvert, First, Last, ref TolCur, false))
            {
                C = ResCurve;
                return true;
            }
            else
            {
                C = aBSpline2d;
                TolCur = Precision.PConfusion();
                return true;
            }
        }

        if (aCurve is Geom2d_OffsetCurve tmpO && IsOf)
        {
            Geom2d_Curve BasCurve = tmpO.BasisCurve();
            Geom2d_Curve? ResCurve = null;
            if (ConvertCurve2d(BasCurve, ref ResCurve, IsConvert, First, Last, ref TolCur))
            {
                if (ResCurve!.Continuity() != GeomAbs_C0)
                {
                    C = new Geom2d_OffsetCurve(ResCurve, tmpO.Offset());
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
                        C = (Geom2d_Curve)aCurve.Copy();
                        TolCur = Precision.PConfusion();
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
                    C = (Geom2d_Curve)aCurve.Copy();
                    TolCur = Precision.PConfusion();
                    return true;
                }
                else
                {
                    return false;
                }
            }
        }
        if (aCurve is Geom2d_BSplineCurve
            || aCurve is Geom2d_BezierCurve
            || ((aCurve is Geom2d_OffsetCurve) && !IsOf))
        {
            int Deg = 1;

            if (aCurve is Geom2d_BSplineCurve BsC)
            {
                Deg = BsC.Degree();
                bool IsR = (myRational && BsC.IsRational());
                if (!IsR && Deg <= myMaxDegree && (BsC.NbKnots() - 1) <= myNbMaxSeg)
                {
                    if (IsConvert)
                    {
                        C = (Geom2d_Curve)aCurve.Copy();
                        TolCur = Precision.PConfusion();
                        return true;
                    }
                    else
                    {
                        return false;
                    }
                }
            }
            if (aCurve is Geom2d_BezierCurve BzC)
            {
                Deg = BzC.Degree();
                bool IsR = (myRational && BzC.IsRational());
                if (!IsR && Deg <= myMaxDegree)
                {
                    if (IsConvert)
                    {
                        C = (Geom2d_Curve)aCurve.Copy();
                        TolCur = Precision.PConfusion();
                        return true;
                    }
                    else
                    {
                        return false;
                    }
                }
            }
            Geom2d_Curve aCurve1;
            double pf = aCurve.FirstParameter(), pl = aCurve.LastParameter();
            // 15.11.2002 PTV OCC966
            if (ShapeAnalysis_Curve.IsPeriodic(aCurve) && (First != Last))
            {
                aCurve1 = new Geom2d_TrimmedCurve(aCurve, First, Last);
            }
            else if (aCurve.FirstParameter() < (First - Precision.PConfusion())
                     || aCurve.LastParameter() > (Last + Precision.PConfusion()))
            {
                double F = Max(First, pf), L = Min(Last, pl);
                if (F != L)
                {
                    aCurve1 = new Geom2d_TrimmedCurve(aCurve, F, L);
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
                    GeomAbs_Shape aCont = IntegerToGeomAbsShape(aC);
                    for (int j = 1; j <= 2; j++)
                    {
                        Geom2dConvert_ApproxCurve anApprox = new Geom2dConvert_ApproxCurve(aCurve1, myTol2d, aCont, MaxSeg, MaxDeg);
                        bool Done = anApprox.IsDone();
                        C = anApprox.InternalCurve();
                        // OCCT: dereferences the curve, null when the approximation has no result (an access
                        // violation natively, caught as a failure: decision 917)
                        int Nbseg = ((Geom2d_BSplineCurve)C!).NbKnots() - 1;
                        int DegC = ((Geom2d_BSplineCurve)C).Degree();

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
                                    C = (Geom2d_Curve)aCurve.Copy();
                                    TolCur = Precision.PConfusion();
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
                                    C = (Geom2d_Curve)aCurve.Copy();
                                    TolCur = Precision.PConfusion();
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
                C = (Geom2d_Curve)aCurve.Copy();
                TolCur = Precision.PConfusion();
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
    public override bool NewPoint(in TopoDS_Vertex V, out gp_Pnt P, out double Tol)
    {
        P = new gp_Pnt();
        Tol = BRep_Tool.Tolerance(V);
        // OCCT: myConvert is the state of the last conversion before the call (occt-issues.md)
        if (myConvert)
        {
            gp_Pnt p1 = new gp_Pnt(BRep_Tool.Pnt(V).XYZ());
            P = p1;
            return true;
        }
        else
        {
            return false;
        }
    }

    /// <summary>False: parameters are never modified.</summary>
    public override bool NewParameter(in TopoDS_Vertex V, in TopoDS_Edge E, ref double P, ref double Tol)
    {
        return false;
    }

    /// <summary>The continuity of the edge between the faces, unchanged.</summary>
    public override GeomAbs_Shape Continuity(in TopoDS_Edge E,
                                             in TopoDS_Face F1,
                                             in TopoDS_Face F2,
                                             in TopoDS_Edge NewE,
                                             in TopoDS_Face NewF1,
                                             in TopoDS_Face NewF2)
    {
        return BRep_Tool.Continuity(E, F1, F2);
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
    public void SetContinuity3d(GeomAbs_Shape Continuity3d)
    {
        myContinuity3d = Continuity3d;
    }

    /// <summary>Sets the continuity of the approximation of pcurves.</summary>
    public void SetContinuity2d(GeomAbs_Shape Continuity2d)
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
