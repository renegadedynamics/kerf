// Port of: src/ModelingAlgorithms/TKHelix/HelixGeom/HelixGeom_Tools.hxx
// Port of: src/ModelingAlgorithms/TKHelix/HelixGeom/HelixGeom_Tools.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation.Advanced;
using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Helix.Geometry;

/// <summary>Approximation of helix curves (and of any 3D curve adaptor) by B-spline curves.</summary>
/// <remarks>
/// The outputs are <c>ref</c>: OCCT leaves them unchanged when the approximation fails. The curves returned are new
/// and unfrozen (decision 244).
/// </remarks>
public static class GeometryTools
{
    private sealed class HelixGeom_Tools_Eval : EvaluatorFunction
    {
        private readonly CurveAdaptorBase _fonct;

        public HelixGeom_Tools_Eval(CurveAdaptorBase func)
        {
            _fonct = func;
        }

        public override void Evaluate(int dimension, ReadOnlySpan<double> startEnd, double param, int order,
                                      Span<double> result, out int errorCode)
        {
            errorCode = 0;
            double par = param;

            // Dimension is incorrect
            // OCCT: the error is set, but the three values are written anyway (occt-issues.md, unit 7D)
            if (dimension != 3)
            {
                errorCode = 1;
            }
            Point3D pnt;
            Vector3D v1, v2;

            switch (order)
            {
                case 0:
                    pnt = _fonct.Value(par);
                    result[0] = pnt.X;
                    result[1] = pnt.Y;
                    result[2] = pnt.Z;
                    break;
                case 1:
                    _fonct.D1(par, out pnt, out v1);
                    result[0] = v1.X;
                    result[1] = v1.Y;
                    result[2] = v1.Z;
                    break;
                case 2:
                    _fonct.D2(par, out pnt, out v1, out v2);
                    result[0] = v2.X;
                    result[1] = v2.Y;
                    result[2] = v2.Z;
                    break;
                default:
                    result[0] = result[1] = result[2] = 0.0;
                    errorCode = 3;
                    break;
            }
        }
    }

    /// <summary>
    /// Approximates a 3D curve by a B-spline within <paramref name="tol"/>, with continuity
    /// <paramref name="cont"/>, at most <paramref name="maxSeg"/> segments and degree
    /// <paramref name="maxDeg"/>. Returns 0 on success (with the curve and the maximum error), 1 if the
    /// approximation failed, 2 if it has no result.
    /// </summary>
    public static int ApprCurve3D(CurveAdaptorBase hc,
                                  double tol,
                                  Continuity cont,
                                  int maxSeg,
                                  int maxDeg,
                                  ref BSplineCurve3D? bSpl,
                                  ref double maxError)
    {
        bool isDone, hasResult;
        int num1DSS, num2DSS, num3DSS;
        double first, last;
        NCollection_HArray1<double>? oneDTolNul = null, twoDTolNul = null, threeDTol;
        DichoCutting cutTool = new();
        // Setup approximation dimensions and tolerances
        num1DSS = 0;
        num2DSS = 0;
        num3DSS = 1;
        threeDTol = new NCollection_HArray1<double>(1, num3DSS);
        threeDTol.Init(tol);
        // Get curve parameter range
        first = hc.FirstParameter();
        last = hc.LastParameter();
        // Setup approximation function and perform approximation
        HelixGeom_Tools_Eval ev = new(hc);
        ApproxAFunction approx = new(num1DSS,
                                                num2DSS,
                                                num3DSS,
                                                oneDTolNul,
                                                twoDTolNul,
                                                threeDTol,
                                                first,
                                                last,
                                                cont,
                                                maxDeg,
                                                maxSeg,
                                                ev,
                                                cutTool);
        // Check if approximation was successful
        isDone = approx.IsDone;
        if (!isDone)
        {
            return 1;
        }
        // Initialize error and check for results
        maxError = 0.0;
        // Verify approximation has results
        hasResult = approx.HasResult;
        if (!hasResult)
        {
            return 2;
        }
        // Extract B-spline curve data from approximation
        NCollection_Array1<Point3D> poles = new(1, approx.NbPoles());
        approx.Poles(1, poles);
        NCollection_HArray1<double> knots = approx.Knots()!;
        NCollection_HArray1<int> mults = approx.Multiplicities()!;
        int degree = approx.Degree();
        bSpl = new BSplineCurve3D(poles, knots.Array1(), mults.Array1(), degree);
        maxError = approx.MaxError(3, 1);
        // Return success
        return 0;
    }

    /// <summary>
    /// Approximates a helix (see <see cref="HelixCurve.Load(double, double, double, double, double, bool)"/>)
    /// by a B-spline within <paramref name="tol"/>, C2, degree at most 8 and at most 150 segments. Returns 0 on
    /// success.
    /// </summary>
    /// <exception cref="ConstructionErrorException">The helix parameters are invalid.</exception>
    public static int ApprHelix(double aT1,
                                double aT2,
                                double aPitch,
                                double aRStart,
                                double aTaperAngle,
                                bool aIsCW,
                                double tol,
                                ref BSplineCurve3D? bSpl,
                                ref double maxError)
    {
        int iErr, maxDegree, maxSeg;
        Continuity cont;
        HelixCurve adaptor = new();
        HelixCurve hAdaptor;
        // Load helix parameters and create adaptor handle
        adaptor.Load(aT1, aT2, aPitch, aRStart, aTaperAngle, aIsCW);
        hAdaptor = new HelixCurve(adaptor);
        // Set default approximation parameters
        cont = Continuity.C2;
        maxDegree = 8;
        maxSeg = 150;
        // Perform curve approximation
        iErr = ApprCurve3D(hAdaptor,
                           tol,
                           cont,
                           maxSeg,
                           maxDegree,
                           ref bSpl,
                           ref maxError);
        return iErr;
    }
}
