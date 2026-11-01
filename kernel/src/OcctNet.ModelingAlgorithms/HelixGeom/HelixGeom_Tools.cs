// Port of: src/ModelingAlgorithms/TKHelix/HelixGeom/HelixGeom_Tools.hxx
// Port of: src/ModelingAlgorithms/TKHelix/HelixGeom/HelixGeom_Tools.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Approximation of helix curves (and of any 3D curve adaptor) by B-spline curves.</summary>
/// <remarks>
/// The outputs are <c>ref</c>: OCCT leaves them unchanged when the approximation fails. The curves returned are new
/// and unfrozen (decision 244).
/// </remarks>
public static class HelixGeom_Tools
{
    private sealed class HelixGeom_Tools_Eval : AdvApprox_EvaluatorFunction
    {
        private readonly Adaptor3d_Curve fonct;

        public HelixGeom_Tools_Eval(Adaptor3d_Curve theFunc)
        {
            fonct = theFunc;
        }

        public override void Evaluate(int Dimension, ReadOnlySpan<double> StartEnd, double Param, int Order,
                                      Span<double> Result, out int ErrorCode)
        {
            ErrorCode = 0;
            double par = Param;

            // Dimension is incorrect
            // OCCT: the error is set, but the three values are written anyway (occt-issues.md, unit 7D)
            if (Dimension != 3)
            {
                ErrorCode = 1;
            }
            gp_Pnt pnt;
            gp_Vec v1, v2;

            switch (Order)
            {
                case 0:
                    pnt = fonct.Value(par);
                    Result[0] = pnt.X();
                    Result[1] = pnt.Y();
                    Result[2] = pnt.Z();
                    break;
                case 1:
                    fonct.D1(par, out pnt, out v1);
                    Result[0] = v1.X();
                    Result[1] = v1.Y();
                    Result[2] = v1.Z();
                    break;
                case 2:
                    fonct.D2(par, out pnt, out v1, out v2);
                    Result[0] = v2.X();
                    Result[1] = v2.Y();
                    Result[2] = v2.Z();
                    break;
                default:
                    Result[0] = Result[1] = Result[2] = 0.0;
                    ErrorCode = 3;
                    break;
            }
        }
    }

    /// <summary>
    /// Approximates a 3D curve by a B-spline within <paramref name="theTol"/>, with continuity
    /// <paramref name="theCont"/>, at most <paramref name="theMaxSeg"/> segments and degree
    /// <paramref name="theMaxDeg"/>. Returns 0 on success (with the curve and the maximum error), 1 if the
    /// approximation failed, 2 if it has no result.
    /// </summary>
    public static int ApprCurve3D(Adaptor3d_Curve theHC,
                                  double theTol,
                                  GeomAbs_Shape theCont,
                                  int theMaxSeg,
                                  int theMaxDeg,
                                  ref Geom_BSplineCurve? theBSpl,
                                  ref double theMaxError)
    {
        bool anIsDone, aHasResult;
        int Num1DSS, Num2DSS, Num3DSS;
        double First, Last;
        NCollection_HArray1<double>? OneDTolNul = null, TwoDTolNul = null, ThreeDTol;
        AdvApprox_DichoCutting aCutTool = new();
        // Setup approximation dimensions and tolerances
        Num1DSS = 0;
        Num2DSS = 0;
        Num3DSS = 1;
        ThreeDTol = new NCollection_HArray1<double>(1, Num3DSS);
        ThreeDTol.Init(theTol);
        // Get curve parameter range
        First = theHC.FirstParameter();
        Last = theHC.LastParameter();
        // Setup approximation function and perform approximation
        HelixGeom_Tools_Eval ev = new(theHC);
        AdvApprox_ApproxAFunction aApprox = new(Num1DSS,
                                                Num2DSS,
                                                Num3DSS,
                                                OneDTolNul,
                                                TwoDTolNul,
                                                ThreeDTol,
                                                First,
                                                Last,
                                                theCont,
                                                theMaxDeg,
                                                theMaxSeg,
                                                ev,
                                                aCutTool);
        // Check if approximation was successful
        anIsDone = aApprox.IsDone();
        if (!anIsDone)
        {
            return 1;
        }
        // Initialize error and check for results
        theMaxError = 0.0;
        // Verify approximation has results
        aHasResult = aApprox.HasResult();
        if (!aHasResult)
        {
            return 2;
        }
        // Extract B-spline curve data from approximation
        NCollection_Array1<gp_Pnt> Poles = new(1, aApprox.NbPoles());
        aApprox.Poles(1, Poles);
        NCollection_HArray1<double> Knots = aApprox.Knots()!;
        NCollection_HArray1<int> Mults = aApprox.Multiplicities()!;
        int Degree = aApprox.Degree();
        theBSpl = new Geom_BSplineCurve(Poles, Knots.Array1(), Mults.Array1(), Degree);
        theMaxError = aApprox.MaxError(3, 1);
        // Return success
        return 0;
    }

    /// <summary>
    /// Approximates a helix (see <see cref="HelixGeom_HelixCurve.Load(double, double, double, double, double, bool)"/>)
    /// by a B-spline within <paramref name="theTol"/>, C2, degree at most 8 and at most 150 segments. Returns 0 on
    /// success.
    /// </summary>
    /// <exception cref="Standard_ConstructionError">The helix parameters are invalid.</exception>
    public static int ApprHelix(double aT1,
                                double aT2,
                                double aPitch,
                                double aRStart,
                                double aTaperAngle,
                                bool aIsCW,
                                double theTol,
                                ref Geom_BSplineCurve? theBSpl,
                                ref double theMaxError)
    {
        int iErr, aMaxDegree, aMaxSeg;
        GeomAbs_Shape aCont;
        HelixGeom_HelixCurve aAdaptor = new();
        HelixGeom_HelixCurve aHAdaptor;
        // Load helix parameters and create adaptor handle
        aAdaptor.Load(aT1, aT2, aPitch, aRStart, aTaperAngle, aIsCW);
        aHAdaptor = new HelixGeom_HelixCurve(aAdaptor);
        // Set default approximation parameters
        aCont = GeomAbs_Shape.GeomAbs_C2;
        aMaxDegree = 8;
        aMaxSeg = 150;
        // Perform curve approximation
        iErr = ApprCurve3D(aHAdaptor,
                           theTol,
                           aCont,
                           aMaxSeg,
                           aMaxDegree,
                           ref theBSpl,
                           ref theMaxError);
        return iErr;
    }
}
