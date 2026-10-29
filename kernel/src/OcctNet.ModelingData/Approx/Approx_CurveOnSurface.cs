// Port of: src/ModelingData/TKGeomBase/Approx/Approx_CurveOnSurface.hxx
// Port of: src/ModelingData/TKGeomBase/Approx/Approx_CurveOnSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Convert_ParameterisationType;
using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_Shape;
using static Occt.GeomAbs_SurfaceType;

namespace Occt;

/// <summary>
/// Approximates a curve on a surface (a 2D curve and the surface it lies on) with a 3D B-spline curve, a 2D B-spline
/// curve, or both, by AdvApprox. Isolines of a GeomAdaptor_Surface are taken from the surface when only the 3D curve
/// is asked for.
/// </summary>
public class Approx_CurveOnSurface
{
    private sealed class Approx_CurveOnSurface_Eval : AdvApprox_EvaluatorFunction
    {
        // Reassigned when AdvApprox asks for another interval, as OCCT's mutable handles are.
        private Adaptor3d_Curve fonct;
        private Adaptor2d_Curve2d fonct2d;
        private double StartEndSav0;
        private double StartEndSav1;

        public Approx_CurveOnSurface_Eval(Adaptor3d_Curve theFunc, Adaptor2d_Curve2d theFunc2d, double First,
                                          double Last)
        {
            fonct = theFunc;
            fonct2d = theFunc2d;
            StartEndSav0 = First;
            StartEndSav1 = Last;
        }

        public override void Evaluate(int Dimension, ReadOnlySpan<double> StartEnd, double Param, int Order,
                                      Span<double> Result, out int ErrorCode)
        {
            ErrorCode = 0;
            double par = Param;

            // Dimension is incorrect
            if (Dimension != 5)
            {
                ErrorCode = 1;
            }

            // Parameter is incorrect
            if (StartEnd[0] != StartEndSav0 || StartEnd[1] != StartEndSav1)
            {
                fonct = fonct.Trim(StartEnd[0], StartEnd[1], Precision.PConfusion());
                fonct2d = fonct2d.Trim(StartEnd[0], StartEnd[1], Precision.PConfusion());
                StartEndSav0 = StartEnd[0];
                StartEndSav1 = StartEnd[1];
            }
            gp_Pnt pnt;

            gp_Pnt2d pnt2d;

            switch (Order)
            {
                case 0:
                {
                    fonct2d.D0(par, out pnt2d);
                    fonct.D0(par, out pnt);
                    Result[0] = pnt2d.X();
                    Result[1] = pnt2d.Y();
                    Result[2] = pnt.X();
                    Result[3] = pnt.Y();
                    Result[4] = pnt.Z();
                    break;
                }
                case 1:
                {
                    gp_Vec v1;
                    gp_Vec2d v21;
                    fonct2d.D1(par, out pnt2d, out v21);
                    fonct.D1(par, out pnt, out v1);
                    Result[0] = v21.X();
                    Result[1] = v21.Y();
                    Result[2] = v1.X();
                    Result[3] = v1.Y();
                    Result[4] = v1.Z();
                    break;
                }
                case 2:
                {
                    gp_Vec v1, v2;
                    gp_Vec2d v21, v22;
                    fonct2d.D2(par, out pnt2d, out v21, out v22);
                    fonct.D2(par, out pnt, out v1, out v2);
                    Result[0] = v22.X();
                    Result[1] = v22.Y();
                    Result[2] = v2.X();
                    Result[3] = v2.Y();
                    Result[4] = v2.Z();
                    break;
                }
                default:
                    Result[0] = Result[1] = Result[2] = Result[3] = Result[4] = 0.0;
                    ErrorCode = 3;
                    break;
            }
        }
    }

    private sealed class Approx_CurveOnSurface_Eval3d : AdvApprox_EvaluatorFunction
    {
        private Adaptor3d_Curve fonct;
        private double StartEndSav0;
        private double StartEndSav1;

        public Approx_CurveOnSurface_Eval3d(Adaptor3d_Curve theFunc, double First, double Last)
        {
            fonct = theFunc;
            StartEndSav0 = First;
            StartEndSav1 = Last;
        }

        public override void Evaluate(int Dimension, ReadOnlySpan<double> StartEnd, double Param, int Order,
                                      Span<double> Result, out int ErrorCode)
        {
            ErrorCode = 0;
            double par = Param;

            // Dimension is incorrect
            if (Dimension != 3)
            {
                ErrorCode = 1;
            }

            // Parameter is incorrect
            if (StartEnd[0] != StartEndSav0 || StartEnd[1] != StartEndSav1)
            {
                fonct = fonct.Trim(StartEnd[0], StartEnd[1], Precision.PConfusion());
                StartEndSav0 = StartEnd[0];
                StartEndSav1 = StartEnd[1];
            }

            gp_Pnt pnt;

            switch (Order)
            {
                case 0:
                    pnt = fonct.Value(par);
                    Result[0] = pnt.X();
                    Result[1] = pnt.Y();
                    Result[2] = pnt.Z();
                    break;
                case 1:
                {
                    gp_Vec v1;
                    fonct.D1(par, out pnt, out v1);
                    Result[0] = v1.X();
                    Result[1] = v1.Y();
                    Result[2] = v1.Z();
                    break;
                }
                case 2:
                {
                    gp_Vec v1, v2;
                    fonct.D2(par, out pnt, out v1, out v2);
                    Result[0] = v2.X();
                    Result[1] = v2.Y();
                    Result[2] = v2.Z();
                    break;
                }
                default:
                    Result[0] = Result[1] = Result[2] = 0.0;
                    ErrorCode = 3;
                    break;
            }
        }
    }

    private sealed class Approx_CurveOnSurface_Eval2d : AdvApprox_EvaluatorFunction
    {
        private Adaptor2d_Curve2d fonct2d;
        private double StartEndSav0;
        private double StartEndSav1;

        public Approx_CurveOnSurface_Eval2d(Adaptor2d_Curve2d theFunc2d, double First, double Last)
        {
            fonct2d = theFunc2d;
            StartEndSav0 = First;
            StartEndSav1 = Last;
        }

        public override void Evaluate(int Dimension, ReadOnlySpan<double> StartEnd, double Param, int Order,
                                      Span<double> Result, out int ErrorCode)
        {
            ErrorCode = 0;
            double par = Param;

            // Dimension is incorrect
            if (Dimension != 2)
            {
                ErrorCode = 1;
            }

            // Parameter is incorrect
            if (StartEnd[0] != StartEndSav0 || StartEnd[1] != StartEndSav1)
            {
                fonct2d = fonct2d.Trim(StartEnd[0], StartEnd[1], Precision.PConfusion());
                StartEndSav0 = StartEnd[0];
                StartEndSav1 = StartEnd[1];
            }

            gp_Pnt2d pnt;

            switch (Order)
            {
                case 0:
                {
                    pnt = fonct2d.Value(par);
                    Result[0] = pnt.X();
                    Result[1] = pnt.Y();
                    break;
                }
                case 1:
                {
                    gp_Vec2d v1;
                    fonct2d.D1(par, out pnt, out v1);
                    Result[0] = v1.X();
                    Result[1] = v1.Y();
                    break;
                }
                case 2:
                {
                    gp_Vec2d v1, v2;
                    fonct2d.D2(par, out pnt, out v1, out v2);
                    Result[0] = v2.X();
                    Result[1] = v2.Y();
                    break;
                }
                default:
                    Result[0] = Result[1] = 0.0;
                    ErrorCode = 3;
                    break;
            }
        }
    }

    // Input curve.
    private readonly Adaptor2d_Curve2d myC2D;
    // Input surface.
    private readonly Adaptor3d_Surface mySurf;
    // First parameter of the result.
    private readonly double myFirst;
    // Last parameter of the result.
    private readonly double myLast;
    // Tolerance.
    private readonly double myTol;
    private Geom2d_BSplineCurve? myCurve2d;
    private Geom_BSplineCurve? myCurve3d;
    private bool myIsDone;
    private bool myHasResult;
    private double myError3d;
    private double myError2dU;
    private double myError2dV;

    /// <summary>Approximates at once (calls <see cref="Perform"/>). OCCT deprecates this constructor.</summary>
    [Obsolete("This constructor is deprecated. Use other constructor and perform method instead.")]
    public Approx_CurveOnSurface(Adaptor2d_Curve2d C2D, Adaptor3d_Surface Surf, double First, double Last, double Tol,
                                 GeomAbs_Shape S, int MaxDegree, int MaxSegments, bool only3d = false,
                                 bool only2d = false)
    {
        myC2D = C2D;
        mySurf = Surf;
        myFirst = First;
        myLast = Last;
        myTol = Tol;
        myIsDone = false;
        myHasResult = false;
        myError3d = 0.0;
        myError2dU = 0.0;
        myError2dV = 0.0;
        Perform(MaxSegments, MaxDegree, S, only3d, only2d);
    }

    /// <summary>
    /// Sets up the approximation of theC2D on theSurf over [theFirst, theLast] within theTol, without computing it.
    /// The adaptors are kept, as OCCT keeps the handles.
    /// </summary>
    public Approx_CurveOnSurface(Adaptor2d_Curve2d theC2D, Adaptor3d_Surface theSurf, double theFirst, double theLast,
                                 double theTol)
    {
        myC2D = theC2D;
        mySurf = theSurf;
        myFirst = theFirst;
        myLast = theLast;
        myTol = theTol;
        myIsDone = false;
        myHasResult = false;
        myError3d = 0.0;
        myError2dU = 0.0;
        myError2dV = 0.0;
    }

    /// <summary>
    /// Builds the curves: the 3D one only (theOnly3d), the 2D one only (theOnly2d), or both. When only the 3D curve
    /// is asked for and the 2D curve is an isoline, the isoline of the surface is taken and the other arguments are
    /// ignored.
    /// </summary>
    /// <exception cref="Standard_ConstructionError">theOnly3d and theOnly2d are both true.</exception>
    public void Perform(int theMaxSegments, int theMaxDegree, GeomAbs_Shape theContinuity, bool theOnly3d = false,
                        bool theOnly2d = false)
    {
        myIsDone = false;
        myHasResult = false;
        myError2dU = 0.0;
        myError2dV = 0.0;
        myError3d = 0.0;

        if (theOnly3d && theOnly2d)
        {
            throw new Standard_ConstructionError();
        }

        GeomAbs_Shape aContinuity = theContinuity;
        if (aContinuity == GeomAbs_G1)
        {
            aContinuity = GeomAbs_C1;
        }
        else if (aContinuity == GeomAbs_G2)
        {
            aContinuity = GeomAbs_C2;
        }
        else if (aContinuity > GeomAbs_C2)
        {
            aContinuity = GeomAbs_C2; // Restriction of AdvApprox_ApproxAFunction
        }

        Adaptor2d_Curve2d TrimmedC2D = myC2D.Trim(myFirst, myLast, Precision.PConfusion());

        bool isU, isForward;
        double aParam;
        if (theOnly3d && isIsoLine(TrimmedC2D, out isU, out aParam, out isForward))
        {
            if (buildC3dOnIsoLine(TrimmedC2D, isU, aParam, isForward))
            {
                myIsDone = true;
                myHasResult = true;
                return;
            }
        }

        Adaptor3d_CurveOnSurface HCOnS = new(TrimmedC2D, mySurf);

        int Num1DSS = 0, Num2DSS = 0, Num3DSS = 0;
        NCollection_HArray1<double>? OneDTol = null;
        NCollection_HArray1<double>? TwoDTolNul = null;
        NCollection_HArray1<double>? ThreeDTol = null;

        // create evaluators and choose appropriate one
        Approx_CurveOnSurface_Eval3d Eval3dCvOnSurf = new(HCOnS, myFirst, myLast);
        Approx_CurveOnSurface_Eval2d Eval2dCvOnSurf = new(TrimmedC2D, myFirst, myLast);
        Approx_CurveOnSurface_Eval EvalCvOnSurf = new(HCOnS, TrimmedC2D, myFirst, myLast);
        AdvApprox_EvaluatorFunction EvalPtr;
        if (theOnly3d)
        {
            EvalPtr = Eval3dCvOnSurf;
        }
        else if (theOnly2d)
        {
            EvalPtr = Eval2dCvOnSurf;
        }
        else
        {
            EvalPtr = EvalCvOnSurf;
        }

        // Initialization for 2d approximation
        if (!theOnly3d)
        {
            Num1DSS = 2;
            OneDTol = new NCollection_HArray1<double>(1, Num1DSS);

            double TolU, TolV;

            TolU = mySurf.UResolution(myTol) / 2.0;
            TolV = mySurf.VResolution(myTol) / 2.0;

            if (mySurf.UContinuity() == GeomAbs_C0)
            {
                if (!Adaptor3d_HSurfaceTool.IsSurfG1(mySurf, true, Precision.Angular()))
                {
                    TolU = Min(1.0e-3, 1.0e3 * TolU);
                }
                if (!Adaptor3d_HSurfaceTool.IsSurfG1(mySurf, true, Precision.Confusion()))
                {
                    TolU = Min(1.0e-3, 1.0e2 * TolU);
                }
            }

            if (mySurf.VContinuity() == GeomAbs_C0)
            {
                if (!Adaptor3d_HSurfaceTool.IsSurfG1(mySurf, false, Precision.Angular()))
                {
                    TolV = Min(1.0e-3, 1.0e3 * TolV);
                }
                if (!Adaptor3d_HSurfaceTool.IsSurfG1(mySurf, false, Precision.Confusion()))
                {
                    TolV = Min(1.0e-3, 1.0e2 * TolV);
                }
            }

            OneDTol.SetValue(1, TolU);
            OneDTol.SetValue(2, TolV);
        }

        if (!theOnly2d)
        {
            Num3DSS = 1;
            ThreeDTol = new NCollection_HArray1<double>(1, Num3DSS);
            ThreeDTol.Init(myTol / 2);
        }

        AdvApprox_Cutting CutTool;

        if (aContinuity <= myC2D.Continuity() && aContinuity <= mySurf.UContinuity()
            && aContinuity <= mySurf.VContinuity())
        {
            CutTool = new AdvApprox_DichoCutting();
        }
        else if (aContinuity == GeomAbs_C1)
        {
            int NbInterv_C1 = HCOnS.NbIntervals(GeomAbs_C1);
            NCollection_Array1<double> CutPnts_C1 = new(1, NbInterv_C1 + 1);
            HCOnS.Intervals(CutPnts_C1, GeomAbs_C1);
            int NbInterv_C2 = HCOnS.NbIntervals(GeomAbs_C2);
            NCollection_Array1<double> CutPnts_C2 = new(1, NbInterv_C2 + 1);
            HCOnS.Intervals(CutPnts_C2, GeomAbs_C2);

            CutTool = new AdvApprox_PrefAndRec(CutPnts_C1, CutPnts_C2);
        }
        else
        {
            int NbInterv_C2 = HCOnS.NbIntervals(GeomAbs_C2);
            NCollection_Array1<double> CutPnts_C2 = new(1, NbInterv_C2 + 1);
            HCOnS.Intervals(CutPnts_C2, GeomAbs_C2);
            int NbInterv_C3 = HCOnS.NbIntervals(GeomAbs_C3);
            NCollection_Array1<double> CutPnts_C3 = new(1, NbInterv_C3 + 1);
            HCOnS.Intervals(CutPnts_C3, GeomAbs_C3);

            CutTool = new AdvApprox_PrefAndRec(CutPnts_C2, CutPnts_C3);
        }

        AdvApprox_ApproxAFunction aApprox = new(Num1DSS,
                                                Num2DSS,
                                                Num3DSS,
                                                OneDTol,
                                                TwoDTolNul,
                                                ThreeDTol,
                                                myFirst,
                                                myLast,
                                                aContinuity,
                                                theMaxDegree,
                                                theMaxSegments,
                                                EvalPtr,
                                                CutTool);

        myIsDone = aApprox.IsDone();
        myHasResult = aApprox.HasResult();

        if (myHasResult)
        {
            NCollection_HArray1<double> Knots = aApprox.Knots()!;
            NCollection_HArray1<int> Mults = aApprox.Multiplicities()!;
            int Degree = aApprox.Degree();

            int aNbPoles = aApprox.NbPoles();
            if (!theOnly2d)
            {
                NCollection_Array1<gp_Pnt> Poles = new(1, aNbPoles);
                aApprox.Poles(1, Poles);
                myCurve3d = new Geom_BSplineCurve(Poles, Knots.Array1(), Mults.Array1(), Degree);
                myError3d = aApprox.MaxError(3, 1);
            }
            if (!theOnly3d)
            {
                NCollection_Array1<gp_Pnt2d> Poles2d = new(1, aNbPoles);
                NCollection_Array1<double> Poles1dU = new(1, aNbPoles);
                aApprox.Poles1d(1, Poles1dU);
                NCollection_Array1<double> Poles1dV = new(1, aNbPoles);
                aApprox.Poles1d(2, Poles1dV);
                for (int i = 1; i <= aNbPoles; i++)
                {
                    Poles2d.SetValue(i, new gp_Pnt2d(Poles1dU.Value(i), Poles1dV.Value(i)));
                }
                myCurve2d = new Geom2d_BSplineCurve(Poles2d, Knots.Array1(), Mults.Array1(), Degree);

                myError2dU = aApprox.MaxError(1, 1);
                myError2dV = aApprox.MaxError(1, 2);
            }
        }
    }

    /// <summary>Whether the approximation is within the required tolerance.</summary>
    public bool IsDone() => myIsDone;

    /// <summary>Whether the approximation has a result, not necessarily within the required tolerance.</summary>
    public bool HasResult() => myHasResult;

    /// <summary>The 3D curve (frozen: the same object until the next Perform), or null if none was built.</summary>
    public Geom_BSplineCurve? Curve3d()
    {
        myCurve3d?.Freeze();
        return myCurve3d;
    }

    /// <summary>The 3D curve without freezing it, for ported code that discards this object.</summary>
    internal Geom_BSplineCurve? InternalCurve3d() => myCurve3d;

    /// <summary>The largest 3D error.</summary>
    public double MaxError3d() => myError3d;

    /// <summary>The 2D curve (frozen: the same object until the next Perform), or null if none was built.</summary>
    public Geom2d_BSplineCurve? Curve2d()
    {
        myCurve2d?.Freeze();
        return myCurve2d;
    }

    /// <summary>The 2D curve without freezing it, for ported code that discards this object.</summary>
    internal Geom2d_BSplineCurve? InternalCurve2d() => myCurve2d;

    /// <summary>The largest error of the U component of the 2D curve.</summary>
    public double MaxError2dU() => myError2dU;

    /// <summary>The largest error of the V component of the 2D curve.</summary>
    public double MaxError2dV() => myError2dV;

    /// <summary>
    /// Checks whether the 2D curve is an isoline with natural parametrisation: a line, or a B-spline or Bezier curve
    /// of degree 1 with two poles, along U or V. Leaves the outputs unchanged when it is not.
    /// </summary>
    protected bool isIsoLine(Adaptor2d_Curve2d theC2D, out bool theIsU, out double theParam, out bool theIsForward)
    {
        // OCCT leaves the outputs unset when it returns false; the port assigns them first (decision 32).
        theIsU = false;
        theParam = 0.0;
        theIsForward = false;

        // These variables are used to check line state (vertical or horizontal).
        bool isAppropriateType = false;
        gp_Pnt2d aLoc2d = new();
        gp_Dir2d aDir2d = new();

        // Test type.
        GeomAbs_CurveType aType = theC2D.GetType();
        if (aType == GeomAbs_Line)
        {
            gp_Lin2d aLin2d = theC2D.Line();
            aLoc2d = aLin2d.Location();
            aDir2d = aLin2d.Direction();
            isAppropriateType = true;
        }
        else if (aType == GeomAbs_BSplineCurve)
        {
            Geom2d_BSplineCurve aBSpline2d = theC2D.BSpline();
            if (aBSpline2d.Degree() != 1 || aBSpline2d.NbPoles() != 2)
            {
                return false; // Not a line or uneven parameterization.
            }

            aLoc2d = aBSpline2d.Pole(1);

            // Vector should be non-degenerated.
            gp_Vec2d aVec2d = new(aBSpline2d.Pole(1), aBSpline2d.Pole(2));
            if (aVec2d.SquareMagnitude() < Precision.Confusion())
            {
                return false; // Degenerated spline.
            }
            aDir2d = new gp_Dir2d(aVec2d);

            isAppropriateType = true;
        }
        else if (aType == GeomAbs_BezierCurve)
        {
            Geom2d_BezierCurve aBezier2d = theC2D.Bezier();
            if (aBezier2d.Degree() != 1 || aBezier2d.NbPoles() != 2)
            {
                return false; // Not a line or uneven parameterization.
            }

            aLoc2d = aBezier2d.Pole(1);

            // Vector should be non-degenerated.
            gp_Vec2d aVec2d = new(aBezier2d.Pole(1), aBezier2d.Pole(2));
            if (aVec2d.SquareMagnitude() < Precision.Confusion())
            {
                return false; // Degenerated spline.
            }
            aDir2d = new gp_Dir2d(aVec2d);

            isAppropriateType = true;
        }

        if (!isAppropriateType)
        {
            return false;
        }

        // Check line to be vertical or horizontal.
        if (aDir2d.IsParallel(gp.DX2d(), Precision.Angular()))
        {
            // Horizontal line. V = const.
            theIsU = false;
            theParam = aLoc2d.Y();
            theIsForward = aDir2d.Dot(gp.DX2d()) > 0.0;
            return true;
        }
        else if (aDir2d.IsParallel(gp.DY2d(), Precision.Angular()))
        {
            // Vertical line. U = const.
            theIsU = true;
            theParam = aLoc2d.X();
            theIsForward = aDir2d.Dot(gp.DY2d()) > 0.0;
            return true;
        }

        return false;
    }

    /// <summary>
    /// Builds the 3D curve of an isoline from the surface's own isoline (GeomAdaptor_Surface only, not spheres),
    /// reparametrised as the 2D curve. True when it is within the tolerance.
    /// </summary>
    protected bool buildC3dOnIsoLine(Adaptor2d_Curve2d theC2D, bool theIsU, double theParam, bool theIsForward)
    {
        // Convert adapter to the appropriate type.
        GeomAdaptor_Surface? aGeomAdapter = mySurf as GeomAdaptor_Surface;
        if (aGeomAdapter is null)
        {
            return false;
        }

        if (mySurf.GetType() == GeomAbs_Sphere)
        {
            return false;
        }

        // Extract isoline
        Geom_Surface aSurf = aGeomAdapter.Surface()!;
        Geom_Curve aC3d;

        gp_Pnt2d aF2d = theC2D.Value(theC2D.FirstParameter());
        gp_Pnt2d aL2d = theC2D.Value(theC2D.LastParameter());

        bool isToTrim = true;
        double U1, U2, V1, V2;
        aSurf.Bounds(out U1, out U2, out V1, out V2);

        if (theIsU)
        {
            double aV1Param = Min(aF2d.Y(), aL2d.Y());
            double aV2Param = Max(aF2d.Y(), aL2d.Y());
            if (aV2Param < V1 - myTol || aV1Param > V2 + myTol)
            {
                return false;
            }
            else if (Precision.IsInfinite(V1) || Precision.IsInfinite(V2))
            {
                if (Math.Abs(aV2Param - aV1Param) < Precision.PConfusion())
                {
                    return false;
                }
                aSurf = new Geom_RectangularTrimmedSurface(aSurf, U1, U2, aV1Param, aV2Param);
                isToTrim = false;
            }
            else
            {
                aV1Param = Max(aV1Param, V1);
                aV2Param = Min(aV2Param, V2);
                if (Math.Abs(aV2Param - aV1Param) < Precision.PConfusion())
                {
                    return false;
                }
            }
            aC3d = aSurf.UIso(theParam);
            if (isToTrim)
            {
                aC3d = new Geom_TrimmedCurve(aC3d, aV1Param, aV2Param);
            }
        }
        else
        {
            double aU1Param = Min(aF2d.X(), aL2d.X());
            double aU2Param = Max(aF2d.X(), aL2d.X());
            if (aU2Param < U1 - myTol || aU1Param > U2 + myTol)
            {
                return false;
            }
            else if (Precision.IsInfinite(U1) || Precision.IsInfinite(U2))
            {
                if (Math.Abs(aU2Param - aU1Param) < Precision.PConfusion())
                {
                    return false;
                }
                aSurf = new Geom_RectangularTrimmedSurface(aSurf, aU1Param, aU2Param, V1, V2);
                isToTrim = false;
            }
            else
            {
                aU1Param = Max(aU1Param, U1);
                aU2Param = Min(aU2Param, U2);
                if (Math.Abs(aU2Param - aU1Param) < Precision.PConfusion())
                {
                    return false;
                }
            }
            aC3d = aSurf.VIso(theParam);
            if (isToTrim)
            {
                aC3d = new Geom_TrimmedCurve(aC3d, aU1Param, aU2Param);
            }
        }

        // Convert arbitrary curve type to the b-spline.
        Geom_BSplineCurve aCurve3d = GeomConvert.CurveToBSplineCurve(aC3d, Convert_QuasiAngular);
        // The conversion can return a frozen curve that others share (decision 244): change a copy.
        if (aCurve3d.IsFrozen())
        {
            aCurve3d = (Geom_BSplineCurve)aCurve3d.Copy();
        }
        myCurve3d = aCurve3d;
        if (!theIsForward)
        {
            aCurve3d.Reverse();
        }

        // Rebuild parameterization for the 3d curve to have the same parameterization with
        // a two-dimensional curve.
        NCollection_Array1<double> aKnots = new(aCurve3d.InternalKnots());
        BSplCLib.Reparametrize(theC2D.FirstParameter(), theC2D.LastParameter(), aKnots);
        aCurve3d.SetKnots(aKnots);

        // Evaluate error.
        myError3d = 0.0;

        double aParF = myFirst;
        double aParL = myLast;
        const int aNbPnt = 23;
        for (int anIdx = 0; anIdx <= aNbPnt; ++anIdx)
        {
            double aPar = aParF + ((aParL - aParF) * anIdx) / aNbPnt;

            gp_Pnt2d aPnt2d = theC2D.Value(aPar);

            gp_Pnt aPntC3D = aCurve3d.Value(aPar);
            gp_Pnt aPntC2D = mySurf.Value(aPnt2d.X(), aPnt2d.Y());

            double aSqDeviation = aPntC3D.SquareDistance(aPntC2D);
            myError3d = Max(aSqDeviation, myError3d);
        }

        myError3d = Math.Sqrt(myError3d);

        // Target tolerance is not obtained. This situation happens for isolines on the sphere.
        // OCCT is unable to convert it keeping original parameterization, while the geometric
        // form of the result is entirely identical. In that case, it is better to utilize
        // a general-purpose approach.
        return myError3d <= myTol;
    }
}
