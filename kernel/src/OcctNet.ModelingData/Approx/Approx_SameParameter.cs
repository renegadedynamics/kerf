// Port of: src/ModelingData/TKGeomBase/Approx/Approx_SameParameter.hxx
// Port of: src/ModelingData/TKGeomBase/Approx/Approx_SameParameter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>
/// Approximates a pcurve on a surface so that its parameter is the same as that of a given 3D reference curve:
/// evaluated on the surface at a parameter, the new pcurve gives the point of the 3D curve at that parameter, within
/// the tolerance. The computation is made by the constructor. The 3D curve and the pcurve must have the same
/// parametric range.
/// </summary>
public sealed class Approx_SameParameter
{
    // Evaluates the pcurve through the 1D B-spline (degree 3) of the change of parameter.
    private sealed class Approx_SameParameter_Evaluator : AdvApprox_EvaluatorFunction
    {
        // OCCT keeps const references to the caller's arrays, which outlive the approximation.
        private readonly NCollection_Array1<double> FlatKnots;
        private readonly NCollection_Array1<double> Poles;
        private readonly Adaptor2d_Curve2d HCurve2d;

        public Approx_SameParameter_Evaluator(NCollection_Array1<double> theFlatKnots,
                                              NCollection_Array1<double> thePoles,
                                              Adaptor2d_Curve2d theHCurve2d)
        {
            FlatKnots = theFlatKnots;
            Poles = thePoles;
            HCurve2d = theHCurve2d;
        }

        public override void Evaluate(int Dimension, ReadOnlySpan<double> StartEnd, double Parameter,
                                      int DerivativeRequest, Span<double> Result, out int ReturnCode)
        {
            const int aDegree = 3;
            ReadOnlySpan<int> extrap_mode = [aDegree, aDegree];
            // OCCT's two-element buffer: AdvApprox asks for at most the first derivative here (continuity C1 at most).
            Span<double> eval_result = stackalloc double[2];
            ReadOnlySpan<double> PolesArray = Poles.AsSpan();

            // Evaluate the 1D B-Spline that represents the change in parameterization.
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Eval(Parameter,
                          false,
                          DerivativeRequest,
                          extrap_mode,
                          aDegree,
                          FlatKnots,
                          1,
                          PolesArray,
                          eval_result);

            Point2D aPoint;
            Vector2D aVector;
            if (DerivativeRequest == 0)
            {
                HCurve2d.D0(eval_result[0], out aPoint);
                aPoint.Coord(out Result[0], out Result[1]);
            }
            else if (DerivativeRequest == 1)
            {
                HCurve2d.D1(eval_result[0], out aPoint, out aVector);
                aVector.Multiply(eval_result[1]);
                aVector.Coord(out Result[0], out Result[1]);
            }

            ReturnCode = 0;
        }
    }

    // Internal data structure to unify access to the most actively used data. OCCT's points into four stack arrays
    // of myMaxArraySize values; the port owns four arrays, swapped as OCCT swaps the pointers.
    private sealed class Approx_SameParameter_Data
    {
        public Adaptor3d_CurveOnSurface myCOnS = null!; // Curve on surface.
        public int myNbPnt;                             // Number of points.
        public double[] myPC3d = null!;                 // Parameters on 3d curve.
        public double[] myPC2d = null!;                 // Parameters on 2d curve.

        // Second data arrays. Used in loop over poles.
        public double[] myNewPC3d = null!; // Parameters on 3d curve.
        public double[] myNewPC2d = null!; // Parameters on 2d curve.

        // Parameters ranges.
        public double myC3dPF; // Curve 3d Parameter First.
        public double myC3dPL; // Curve 3d Parameter Last.
        public double myC2dPF; // Curve 2d Parameter First.
        public double myC2dPL; // Curve 2d Parameter Last.

        public double myTol; // Working tolerance.

        // Swap data arrays and update number of points.
        public void Swap(int theNewNbPoints)
        {
            myNbPnt = theNewNbPoints;
            double[] temp;

            // 3-D
            temp = myPC3d;
            myPC3d = myNewPC3d;
            myNewPC3d = temp;

            // 2-D
            temp = myPC2d;
            myPC2d = myNewPC2d;
            myNewPC2d = temp;
        }
    }

    private const int myNbSamples = 22; // To be consistent with "checkshape".
    private const int myMaxArraySize = 1000;
    private readonly double myDeltaMin;
    private bool mySameParameter;
    private bool myDone;
    private double myTolReached;
    private Geom2d_Curve? myCurve2d;
    // Whether myCurve2d was built here (and so may be frozen when handed out), not the caller's own pcurve.
    private bool myCurve2dIsBuilt;
    private Adaptor2d_Curve2d myHCurve2d;
    private readonly Adaptor3d_Curve myC3d;
    private readonly Adaptor3d_Surface mySurf;
    private Adaptor3d_CurveOnSurface? myCurveOnSurface;

    /// <summary>Makes C2D on S have the parameter of C3D, within Tol. The curves must have the same range.</summary>
    public Approx_SameParameter(Geom_Curve C3D, Geom2d_Curve C2D, Geom_Surface S, double Tol)
    {
        myDeltaMin = global::Kerf.Engine.Foundation.Precision.PConfusion();
        mySameParameter = true;
        myDone = false;
        myHCurve2d = new Geom2dAdaptor_Curve(C2D);
        myC3d = new GeomAdaptor_Curve(C3D);
        mySurf = new GeomAdaptor_Surface(S);
        Build(Tol);
    }

    /// <summary>As the other constructors, for curves and a surface given by adaptors (kept by reference).</summary>
    public Approx_SameParameter(Adaptor3d_Curve C3D, Geom2d_Curve C2D, Adaptor3d_Surface S, double Tol)
    {
        myDeltaMin = global::Kerf.Engine.Foundation.Precision.PConfusion();
        mySameParameter = true;
        myDone = false;
        myC3d = C3D;
        mySurf = S;
        myHCurve2d = new Geom2dAdaptor_Curve(C2D);
        Build(Tol);
    }

    /// <summary>As the other constructors, for curves and a surface given by adaptors (kept by reference).</summary>
    public Approx_SameParameter(Adaptor3d_Curve C3D, Adaptor2d_Curve2d C2D, Adaptor3d_Surface S, double Tol)
    {
        myDeltaMin = global::Kerf.Engine.Foundation.Precision.PConfusion();
        mySameParameter = true;
        myDone = false;
        myC3d = C3D;
        mySurf = S;
        myHCurve2d = C2D;
        Build(Tol);
    }

    /// <summary>False if the computation failed, true if it succeeded.</summary>
    public bool IsDone() => myDone;

    /// <summary>The tolerance (maximal distance) between the 3D curve and the curve on surface of the result.</summary>
    public double TolReached() => myTolReached;

    /// <summary>Whether the data already had the same parameter within the tolerance (then nothing is built).</summary>
    public bool IsSameParameter() => mySameParameter;

    /// <summary>
    /// The 2D curve that has the parameter of the 3D curve on the surface within the tolerance, or null when none was
    /// computed. A curve built by the algorithm is frozen (the same object at each call); otherwise it is the
    /// caller's pcurve (or a trimmed curve on it), as OCCT returns it.
    /// </summary>
    public Geom2d_Curve? Curve2d()
    {
        if (myCurve2dIsBuilt)
        {
            myCurve2d?.Freeze();
        }
        return myCurve2d;
    }

    /// <summary>The 2D curve without freezing it, for ported code that discards this object.</summary>
    internal Geom2d_Curve? InternalCurve2d() => myCurve2d;

    /// <summary>The 3D curve (the caller's adaptor).</summary>
    public Adaptor3d_Curve Curve3d() => myC3d;

    /// <summary>
    /// The curve on surface of the result (the algorithm's own adaptor), or null when the computation stopped early.
    /// </summary>
    public Adaptor3d_CurveOnSurface? CurveOnSurface() => myCurveOnSurface;

    private static void ProjectPointOnCurve(double InitValue,
                                            in Point3D APoint,
                                            double Tolerance,
                                            int NumIteration,
                                            Adaptor3d_Curve Curve,
                                            out bool Status,
                                            out double Result)
    {
        int num_iter = 0, not_done = 1;

        Point3D a_point;
        Vector3D vector, d1, d2;
        double func, func_derivative, param = InitValue;
        Status = false;
        do
        {
            num_iter++;
            Curve.D2(param, out a_point, out d1, out d2);
            vector = new Vector3D(a_point, APoint);

            func = vector.Dot(d1);
            if (Math.Abs(func) < Tolerance * d1.Magnitude())
            {
                not_done = 0;
                Status = true;
            }
            else
            {
                func_derivative = vector.Dot(d2) - d1.Dot(d1);

                // Avoid division by zero.
                const double Toler = 1.0e-12;
                if (Math.Abs(func_derivative) > Toler)
                {
                    param -= func / func_derivative;
                }

                param = Max(param, Curve.FirstParameter());
                param = Min(param, Curve.LastParameter());
            }
        } while (not_done != 0 && num_iter <= NumIteration);

        Result = param;
    }

    private static double ComputeTolReached(Adaptor3d_Curve c3d, Adaptor3d_CurveOnSurface cons, int nbp)
    {
        double d2 = 0.0; // Square max discrete deviation.
        double first = c3d.FirstParameter();
        double last = c3d.LastParameter();
        for (int i = 0; i <= nbp; i++)
        {
            double t = IntToReal(i) / IntToReal(nbp);
            double u = first * (1.0 - t) + last * t;
            Point3D Pc3d, Pcons;
            try
            {
                Pc3d = c3d.Value(u);
                Pcons = cons.Value(u);
            }
            catch (FailureException)
            {
                d2 = global::Kerf.Engine.Foundation.Precision.Infinite();
                break;
            }
            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(Pcons.X()) || global::Kerf.Engine.Foundation.Precision.IsInfinite(Pcons.Y())
                || global::Kerf.Engine.Foundation.Precision.IsInfinite(Pcons.Z()))
            {
                d2 = global::Kerf.Engine.Foundation.Precision.Infinite();
                break;
            }
            d2 = Max(d2, Pc3d.SquareDistance(Pcons));
        }

        const double aMult = 1.0 + 0.05; //
        double aDeviation = aMult * Math.Sqrt(d2);
        aDeviation = Max(aDeviation, global::Kerf.Engine.Foundation.Precision.Confusion()); // Tolerance in modeling space.
        return aDeviation;
    }

    private static bool Check(NCollection_Array1<double> FlatKnots,
                              NCollection_Array1<double> Poles,
                              int nbp,
                              double[] pc3d,
                              Adaptor3d_Curve c3d,
                              Adaptor3d_CurveOnSurface cons,
                              ref double tol,
                              double oldtol)
    {
        const int aDegree = 3;
        ReadOnlySpan<int> extrap_mode = [aDegree, aDegree];
        Span<double> tcons = stackalloc double[1];

        // Correction of the interval of valid values. This condition has no sensible
        // grounds. But it is better then the old one (which is commented out) because
        // it fixes the bug OCC5898. To develop more or less sensible criterion it is
        // necessary to deeply investigate this problem which is not possible in frames
        // of debugging.
        double aParamFirst = 3.0 * pc3d[0] - 2.0 * pc3d[nbp - 1];
        double aParamLast = 3.0 * pc3d[nbp - 1] - 2.0 * pc3d[0];

        double FirstPar = cons.FirstParameter();
        double LastPar = cons.LastParameter();
        if (aParamFirst < FirstPar)
        {
            aParamFirst = FirstPar;
        }
        if (aParamLast > LastPar)
        {
            aParamLast = LastPar;
        }

        double d2 = 0.0; // Maximum square deviation on the samples.
        double d = tol;
        int nn = 2 * nbp;
        double unsurnn = 1.0 / nn;
        double tprev = aParamFirst;
        for (int i = 0; i <= nn; i++)
        {
            // Compute corresponding parameter on 2d curve.
            // It should be inside of 3d curve parameter space.
            double t = unsurnn * i;
            double tc3d = pc3d[0] * (1.0 - t) + pc3d[nbp - 1] * t; // weight function.
            Point3D Pc3d = c3d.Value(tc3d);
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Eval(tc3d, false, 0, extrap_mode, aDegree, FlatKnots, 1, Poles.AsSpan(1), tcons);

            if (tcons[0] < tprev || tcons[0] > aParamLast)
            {
                tol = global::Kerf.Engine.Foundation.Precision.Infinite();
                return false;
            }
            tprev = tcons[0];
            Point3D Pcons = cons.Value(tcons[0]);
            double temp = Pc3d.SquareDistance(Pcons);
            if (temp > d2)
            {
                d2 = temp;
            }
        }
        tol = Math.Sqrt(d2);

        // Check poles parameters to be ordered.
        for (int i = Poles.Lower() + 1; i <= Poles.Upper(); ++i)
        {
            double aPreviousParam = Poles[i - 1];
            double aCurrentParam = Poles[i];

            if (aPreviousParam > aCurrentParam)
            {
                return false;
            }
        }

        return (tol <= d || tol > 0.8 * oldtol);
    }

    // Computes the pcurve (internal use only).
    private void Build(double Tolerance)
    {
        // Algorithm:
        // 1) Build initial distribution. Increase number of samples in case of C0 continuity.
        // 2.1) Check same parameter state on samples.
        // 2.2) Compute parameters in 2d space if not same parameter.
        // 3) Loop over poles number and try to interpolate 2d curve.
        // 4) If loop is failed build 2d curve forcibly or use original pcurve.
        double[] qpcons = new double[myMaxArraySize], qnewpcons = new double[myMaxArraySize],
                 qpc3d = new double[myMaxArraySize], qnewpc3d = new double[myMaxArraySize];

        // Create and fill data structure.
        Approx_SameParameter_Data aData = new();
        aData.myCOnS = new Adaptor3d_CurveOnSurface(myHCurve2d, mySurf);
        aData.myC2dPF = aData.myCOnS.FirstParameter();
        aData.myC2dPL = aData.myCOnS.LastParameter();
        aData.myC3dPF = myC3d.FirstParameter();
        aData.myC3dPL = myC3d.LastParameter();
        aData.myNbPnt = 0; // No points initially.
        aData.myPC2d = qpcons;
        aData.myPC3d = qpc3d;
        aData.myNewPC2d = qnewpcons;
        aData.myNewPC3d = qnewpc3d;
        aData.myTol = Tolerance;

        // Build initial distribution.
        if (!BuildInitialDistribution(aData))
        {
            mySameParameter = false;
            myDone = false;
            return;
        }

        // Check same parameter state on distribution.
        double aMaxSqDeviation = 0.0;
        const double aPercentOfBadProj = 0.3;
        int aNbPnt = aData.myNbPnt - RealToInt(aPercentOfBadProj * aData.myNbPnt);
        mySameParameter = CheckSameParameter(aData, out aMaxSqDeviation);
        if (mySameParameter)
        {
            myTolReached = ComputeTolReached(myC3d, aData.myCOnS, 2 * myNbSamples);
            myDone = true;
            return;
        }
        else
        {
            // Control number of sample points after checking sameparameter
            // If number of points is less then initial one, it means that there are
            // problems with projection
            if (aData.myNbPnt < aNbPnt)
            {
                myTolReached = ComputeTolReached(myC3d, aData.myCOnS, 2 * myNbSamples);
                myCurve2d = Geom2dAdaptor.MakeCurve(myHCurve2d);
                myCurve2dIsBuilt = false;
                myDone = false;
                return;
            }
        }

        // Control tangents at the extremities to know if the
        // reparametring is possible and calculate the tangents
        // at the extremities of the function of change of variable.
        double tangent0 = 0.0, tangent1 = 0.0;
        if (!ComputeTangents(aData.myCOnS, ref tangent0, ref tangent1))
        {
            // Cannot compute tangents.
            mySameParameter = false;
            myDone = false;
            myTolReached = ComputeTolReached(myC3d, aData.myCOnS, 2 * myNbSamples);
            return;
        }

        // There is at least one point where same parameter is broken.
        // Try to build B-spline approximation curve using interpolation with degree 3.
        // The loop is organized over number of poles.
        Continuity aContinuity = myHCurve2d.Continuity();
        if (aContinuity > C1)
        {
            aContinuity = C1;
        }

        double besttol2 = aData.myTol * aData.myTol, tolsov = global::Kerf.Engine.Foundation.Precision.Infinite();
        bool interpolok = false, hasCountChanged = false;
        do
        {
            // Interpolation data.
            int num_knots = aData.myNbPnt + 7;
            int num_poles = aData.myNbPnt + 3;
            NCollection_Array1<double> Poles = new(1, num_poles);
            NCollection_Array1<double> FlatKnots = new(1, num_knots);

            if (!Interpolate(aData, tangent0, tangent1, Poles, FlatKnots))
            {
                // Interpolation fails.
                mySameParameter = false;
                myDone = false;
                myTolReached = ComputeTolReached(myC3d, aData.myCOnS, 2 * myNbSamples);
                return;
            }

            double algtol = Math.Sqrt(besttol2);
            interpolok =
              Check(FlatKnots, Poles, aData.myNbPnt + 1, aData.myPC3d, myC3d, aData.myCOnS, ref algtol, tolsov);
            tolsov = algtol;

            // Try to build 2d curve and check it for validity.
            if (interpolok)
            {
                double besttol = Math.Sqrt(besttol2);

                NCollection_HArray1<double> tol1d;
                NCollection_HArray1<double>? tol2d = null, tol3d = null;
                tol1d = new NCollection_HArray1<double>(1, 2);
                tol1d.SetValue(1, mySurf.UResolution(besttol));
                tol1d.SetValue(2, mySurf.VResolution(besttol));

                Approx_SameParameter_Evaluator ev = new(FlatKnots, Poles, myHCurve2d);
                int aMaxDeg = 11, aMaxSeg = 1000;
                AdvApprox_ApproxAFunction anApproximator = new(2,
                                                               0,
                                                               0,
                                                               tol1d,
                                                               tol2d,
                                                               tol3d,
                                                               aData.myC3dPF,
                                                               aData.myC3dPL,
                                                               aContinuity,
                                                               aMaxDeg,
                                                               aMaxSeg,
                                                               ev);

                if (anApproximator.IsDone() || anApproximator.HasResult())
                {
                    // OCCT copies the curve on surface to restore it below; only its 2D curve changes, so the port
                    // keeps that curve and loads it back.
                    Adaptor2d_Curve2d ACS = aData.myCOnS.GetCurve()!;
                    GeomLib_MakeCurvefromApprox aCurveBuilder = new(anApproximator);
                    Geom2d_BSplineCurve aC2d = aCurveBuilder.Curve2dFromTwo1d(1, 2);
                    Adaptor2d_Curve2d aHCurve2d = new Geom2dAdaptor_Curve(aC2d);
                    aData.myCOnS.Load(aHCurve2d);
                    myTolReached = ComputeTolReached(myC3d, aData.myCOnS, 2 * myNbSamples);

                    const double aMult = 250.0; // To be tolerant with discrete tolerance.
                    if (myTolReached < aMult * besttol)
                    {
                        myCurve2d = aC2d;
                        myCurve2dIsBuilt = true;
                        myHCurve2d = aHCurve2d;
                        myDone = true;
                        break;
                    }
                    else if (aData.myNbPnt < myMaxArraySize - 1)
                    {
                        interpolok = false;
                        aData.myCOnS.Load(ACS);
                    }
                    else
                    {
                        break;
                    }
                }
            }

            if (!interpolok)
            {
                hasCountChanged = IncreaseNbPoles(Poles, FlatKnots, aData, ref besttol2);
            }
        } while (!interpolok && hasCountChanged);

        if (!myDone)
        {
            // Loop is finished unsuccessfully. Fix tolerance by maximal deviation,
            // using data from the last loop iteration or initial data. Use data set with minimal
            // deflection.

            // Original 2d curve.
            aData.myCOnS.Load(myHCurve2d);
            myTolReached = ComputeTolReached(myC3d, aData.myCOnS, 2 * myNbSamples);
            myCurve2d = Geom2dAdaptor.MakeCurve(myHCurve2d);
            myCurve2dIsBuilt = false;

            // Approximation curve.
            int num_knots = aData.myNbPnt + 7;
            int num_poles = aData.myNbPnt + 3;
            NCollection_Array1<double> Poles = new(1, num_poles);
            NCollection_Array1<double> FlatKnots = new(1, num_knots);

            Interpolate(aData, tangent0, tangent1, Poles, FlatKnots);

            double besttol = Math.Sqrt(besttol2);
            NCollection_HArray1<double> tol1d;
            NCollection_HArray1<double>? tol2d = null, tol3d = null;
            tol1d = new NCollection_HArray1<double>(1, 2);
            tol1d.SetValue(1, mySurf.UResolution(besttol));
            tol1d.SetValue(2, mySurf.VResolution(besttol));

            Approx_SameParameter_Evaluator ev = new(FlatKnots, Poles, myHCurve2d);
            AdvApprox_ApproxAFunction anApproximator = new(2,
                                                           0,
                                                           0,
                                                           tol1d,
                                                           tol2d,
                                                           tol3d,
                                                           aData.myC3dPF,
                                                           aData.myC3dPL,
                                                           aContinuity,
                                                           11,
                                                           40,
                                                           ev);

            if (!anApproximator.IsDone() && !anApproximator.HasResult())
            {
                myDone = false;
                return;
            }

            GeomLib_MakeCurvefromApprox aCurveBuilder = new(anApproximator);
            Geom2d_BSplineCurve aC2d = aCurveBuilder.Curve2dFromTwo1d(1, 2);
            Adaptor2d_Curve2d aHCurve2d = new Geom2dAdaptor_Curve(aC2d);
            aData.myCOnS.Load(aHCurve2d);

            double anApproxTol = ComputeTolReached(myC3d, aData.myCOnS, 2 * myNbSamples);
            if (anApproxTol < myTolReached)
            {
                myTolReached = anApproxTol;
                myCurve2d = aC2d;
                myCurve2dIsBuilt = true;
                myHCurve2d = aHCurve2d;
            }
            myDone = true;
        }

        myCurveOnSurface = (Adaptor3d_CurveOnSurface)aData.myCOnS.ShallowCopy();
    }

    // Computes initial point distribution.
    private bool BuildInitialDistribution(Approx_SameParameter_Data theData)
    {
        // Take a multiple of the sample pof CheckShape,
        // at least the control points will be correct.
        // Take parameters with constant step on the curve on surface
        // and on curve 3d.
        double deltacons = (theData.myC2dPL - theData.myC2dPF) / myNbSamples;
        double deltac3d = (theData.myC3dPL - theData.myC3dPF) / myNbSamples;
        double wcons = theData.myC2dPF;
        double wc3d = theData.myC3dPF;
        for (int ii = 0; ii < myNbSamples; ii++)
        {
            theData.myPC2d[ii] = wcons;
            theData.myPC3d[ii] = wc3d;
            wcons += deltacons;
            wc3d += deltac3d;
        }
        theData.myNbPnt = myNbSamples;
        theData.myPC2d[theData.myNbPnt] = theData.myC2dPL;
        theData.myPC3d[theData.myNbPnt] = theData.myC3dPL;

        // Change number of points in case of C0 continuity.
        Continuity Continuity = myHCurve2d.Continuity();
        if (Continuity < C1)
        {
            if (!IncreaseInitialNbSamples(theData))
            {
                // Number of samples is too big.
                return false;
            }
        }

        return true;
    }

    // Get number of C1 intervals and build new distribution on them.
    // Sub-method in BuildInitialDistribution.
    private bool IncreaseInitialNbSamples(Approx_SameParameter_Data theData)
    {
        int NbInt = myHCurve2d.NbIntervals(C1) + 1;
        NCollection_Array1<double> aC1Intervals = new(1, NbInt);
        myHCurve2d.Intervals(aC1Intervals, C1);

        int inter = 1;
        while (inter <= NbInt && aC1Intervals[inter] <= theData.myC3dPF + myDeltaMin)
        {
            inter++;
        }
        while (NbInt > 0 && aC1Intervals[NbInt] >= theData.myC3dPL - myDeltaMin)
        {
            NbInt--;
        }

        // Compute new parameters.
        NCollection_Sequence<double> aNewPar = new();
        aNewPar.Append(theData.myC3dPF);
        int ii = 1;
        while (inter <= NbInt || (ii < myNbSamples && inter <= aC1Intervals.Length()))
        {
            if (aC1Intervals[inter] < theData.myPC2d[ii])
            {
                aNewPar.Append(aC1Intervals[inter]);
                if ((theData.myPC2d[ii] - aC1Intervals[inter]) <= myDeltaMin)
                {
                    ii++;
                    if (ii > myNbSamples)
                    {
                        ii = myNbSamples;
                    }
                }
                inter++;
            }
            else
            {
                if ((aC1Intervals[inter] - theData.myPC2d[ii]) > myDeltaMin)
                {
                    aNewPar.Append(theData.myPC2d[ii]);
                }
                ii++;
            }
        }
        // Simple protection if theNewNbPoints > allocated elements in array but one
        // myMaxArraySize - 1 index may be filled after projection.
        theData.myNbPnt = aNewPar.Length();
        if (theData.myNbPnt > myMaxArraySize - 1)
        {
            return false;
        }

        for (ii = 1; ii < theData.myNbPnt; ii++)
        {
            // Copy only internal points.
            theData.myPC2d[ii] = theData.myPC3d[ii] = aNewPar.Value(ii + 1);
        }
        theData.myPC3d[theData.myNbPnt] = theData.myC3dPL;
        theData.myPC2d[theData.myNbPnt] = theData.myC2dPL;

        return true;
    }

    // Checks the same parameter state and builds the dependency between the 2d and 3d curves.
    private bool CheckSameParameter(Approx_SameParameter_Data theData, out double theSqDist)
    {
        double Tol2 = theData.myTol * theData.myTol;
        bool isSameParam = true;

        // Compute initial distance on boundary points.
        Point3D Pcons, Pc3d;
        theData.myCOnS.D0(theData.myC2dPF, out Pcons);
        myC3d.D0(theData.myC3dPF, out Pc3d);
        double dist2 = Pcons.SquareDistance(Pc3d);
        double dmax2 = dist2;

        theData.myCOnS.D0(theData.myC2dPL, out Pcons);
        myC3d.D0(theData.myC3dPL, out Pc3d);
        dist2 = Pcons.SquareDistance(Pc3d);
        dmax2 = Max(dmax2, dist2);

        Extrema_LocateExtPC Projector = new();
        Projector.Initialize(myC3d, theData.myC3dPF, theData.myC3dPL, theData.myTol);

        int count = 1;
        double previousp = theData.myC3dPF, initp = 0, curp = 0;
        double bornesup = theData.myC3dPL - myDeltaMin;
        bool isProjOk = false;
        for (int ii = 1; ii < theData.myNbPnt; ii++)
        {
            theData.myCOnS.D0(theData.myPC2d[ii], out Pcons);
            myC3d.D0(theData.myPC3d[ii], out Pc3d);
            dist2 = Pcons.SquareDistance(Pc3d);

            // Same parameter point.
            bool isUseParam = (dist2 <= Tol2 && // Good distance.
                               (theData.myPC3d[ii] > theData.myPC3d[count - 1]
                                                       + myDeltaMin)); // Point is separated from previous.
            if (isUseParam)
            {
                if (dmax2 < dist2)
                {
                    dmax2 = dist2;
                }
                initp = previousp = theData.myPC3d[count] = theData.myPC3d[ii];
                theData.myPC2d[count] = theData.myPC2d[ii];
                count++;
                continue;
            }

            // Local search: local extrema and iterative projection algorithm.
            if (!isProjOk)
            {
                initp = theData.myPC3d[ii];
            }
            isProjOk = isSameParam = false;
            Projector.Perform(Pcons, initp);
            if (Projector.IsDone())
            {
                // Local extrema is found.
                curp = Projector.Point().Parameter();
                isProjOk = true;
            }
            else
            {
                ProjectPointOnCurve(initp, Pcons, theData.myTol, 30, myC3d, out isProjOk, out curp);
            }
            isProjOk = isProjOk &&                      // Good projection.
                       curp > previousp + myDeltaMin && // Point is separated from previous.
                       curp < bornesup;                 // Inside of parameter space.
            if (isProjOk)
            {
                initp = previousp = theData.myPC3d[count] = curp;
                theData.myPC2d[count] = theData.myPC2d[ii];
                count++;
                continue;
            }

            // Whole parameter space search using general extrema.
            Extrema_ExtPC PR = new(Pcons, myC3d, theData.myC3dPF, theData.myC3dPL, theData.myTol);
            if (!PR.IsDone() || PR.NbExt() == 0)
            { // Lazy evaluation is used.
                continue;
            }

            int aNbExt = PR.NbExt();
            int anIndMin = 0;
            double aCurDistMin = RealLast();
            for (int i = 1; i <= aNbExt; i++)
            {
                Point3D aP = PR.Point(i).Value();
                double aDist2 = aP.SquareDistance(Pcons);
                if (aDist2 < aCurDistMin)
                {
                    aCurDistMin = aDist2;
                    anIndMin = i;
                }
            }
            if (anIndMin != 0)
            {
                curp = PR.Point(anIndMin).Parameter();
                if (curp > previousp + myDeltaMin && curp < bornesup)
                {
                    initp = previousp = theData.myPC3d[count] = curp;
                    theData.myPC2d[count] = theData.myPC2d[ii];
                    count++;
                    isProjOk = true;
                }
            }
        }
        theData.myNbPnt = count;
        theData.myPC2d[theData.myNbPnt] = theData.myC2dPL;
        theData.myPC3d[theData.myNbPnt] = theData.myC3dPL;

        theSqDist = dmax2;
        return isSameParam;
    }

    // Computes the tangents on the boundary points; false if they are null. The outputs are left unchanged then, as
    // in OCCT.
    private bool ComputeTangents(Adaptor3d_CurveOnSurface theCOnS, ref double theFirstTangent,
                                 ref double theLastTangent)
    {
        const double aSmallMagnitude = 1.0e-12;
        // Check tangency on curve border.
        Point3D aPnt, aPntCOnS;
        Vector3D aVec, aVecConS;

        // First point.
        double aParamFirst = myC3d.FirstParameter();
        theCOnS.D1(aParamFirst, out aPntCOnS, out aVecConS);
        myC3d.D1(aParamFirst, out aPnt, out aVec);
        double aMagnitude = aVecConS.Magnitude();
        if (aMagnitude > aSmallMagnitude)
        {
            theFirstTangent = aVec.Magnitude() / aMagnitude;
        }
        else
        {
            return false;
        }

        // Last point.
        double aParamLast = myC3d.LastParameter();
        theCOnS.D1(aParamLast, out aPntCOnS, out aVecConS);
        myC3d.D1(aParamLast, out aPnt, out aVec);

        aMagnitude = aVecConS.Magnitude();
        if (aMagnitude > aSmallMagnitude)
        {
            theLastTangent = aVec.Magnitude() / aMagnitude;
        }
        else
        {
            return false;
        }

        return true;
    }

    // Computes interpolated values; false if the computation failed.
    private static bool Interpolate(Approx_SameParameter_Data theData,
                                    double aTangFirst,
                                    double aTangLast,
                                    NCollection_Array1<double> thePoles,
                                    NCollection_Array1<double> theFlatKnots)
    {
        int num_poles = theData.myNbPnt + 3;
        NCollection_Array1<int> ContactOrder = new(1, num_poles);
        NCollection_Array1<double> aParameters = new(1, num_poles);

        // Fill tables taking attention to end values.
        ContactOrder.Init(0);
        ContactOrder[2] = ContactOrder[num_poles - 1] = 1;

        theFlatKnots[1] = theFlatKnots[2] = theFlatKnots[3] = theFlatKnots[4] = theData.myC3dPF;
        theFlatKnots[num_poles + 1] = theFlatKnots[num_poles + 2] = theFlatKnots[num_poles + 3] =
          theFlatKnots[num_poles + 4] = theData.myC3dPL;

        thePoles[1] = theData.myC2dPF;
        thePoles[num_poles] = theData.myC2dPL;
        thePoles[2] = aTangFirst;
        thePoles[num_poles - 1] = aTangLast;

        aParameters[1] = aParameters[2] = theData.myC3dPF;
        aParameters[num_poles - 1] = aParameters[num_poles] = theData.myC3dPL;

        for (int ii = 3; ii <= num_poles - 2; ii++)
        {
            thePoles[ii] = theData.myPC2d[ii - 2];
            aParameters[ii] = theFlatKnots[ii + 2] = theData.myPC3d[ii - 2];
        }
        int inversion_problem;
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Interpolate(3,
                             theFlatKnots,
                             aParameters,
                             ContactOrder,
                             1,
                             thePoles.AsSpan(1),
                             out inversion_problem);
        return inversion_problem == 0;
    }

    // Increases the number of poles in the poles loop; true if the distribution changed.
    private bool IncreaseNbPoles(NCollection_Array1<double> thePoles,
                                 NCollection_Array1<double> theFlatKnots,
                                 Approx_SameParameter_Data theData,
                                 ref double theBestSqTol)
    {
        Extrema_LocateExtPC Projector = new();
        Projector.Initialize(myC3d, myC3d.FirstParameter(), myC3d.LastParameter(), theData.myTol);
        double curp = 0.0;
        bool projok = false;

        // Project middle point to fix parameterization and check projection existence.
        const int aDegree = 3;
        const int DerivativeRequest = 0;
        ReadOnlySpan<int> extrap_mode = [aDegree, aDegree];
        Span<double> eval_result = stackalloc double[1];
        ReadOnlySpan<double> PolesArray = thePoles.AsSpan();
        int newcount = 0;
        for (int ii = 0; ii < theData.myNbPnt; ii++)
        {
            theData.myNewPC2d[newcount] = theData.myPC2d[ii];
            theData.myNewPC3d[newcount] = theData.myPC3d[ii];
            newcount++;

            if (theData.myNbPnt - ii + newcount == myMaxArraySize)
            {
                continue;
            }

            global::Kerf.Engine.Geometry.Splines.BSplCLib.Eval(0.5 * (theData.myPC3d[ii] + theData.myPC3d[ii + 1]),
                          false,
                          DerivativeRequest,
                          extrap_mode,
                          3,
                          theFlatKnots,
                          1,
                          PolesArray,
                          eval_result);

            if (eval_result[0] < theData.myPC2d[ii] || eval_result[0] > theData.myPC2d[ii + 1])
            {
                double ucons = 0.5 * (theData.myPC2d[ii] + theData.myPC2d[ii + 1]);
                double uc3d = 0.5 * (theData.myPC3d[ii] + theData.myPC3d[ii + 1]);

                Point3D Pcons;
                theData.myCOnS.D0(ucons, out Pcons);
                Projector.Perform(Pcons, uc3d);
                if (Projector.IsDone())
                {
                    curp = Projector.Point().Parameter();
                    double dist_2 = Projector.SquareDistance();
                    if (dist_2 > theBestSqTol)
                    {
                        theBestSqTol = dist_2;
                    }
                    projok = true;
                }
                else
                {
                    ProjectPointOnCurve(uc3d, Pcons, theData.myTol, 30, myC3d, out projok, out curp);
                }
                if (projok)
                {
                    if (curp > theData.myPC3d[ii] + myDeltaMin && curp < theData.myPC3d[ii + 1] - myDeltaMin)
                    {
                        theData.myNewPC3d[newcount] = curp;
                        theData.myNewPC2d[newcount] = ucons;
                        newcount++;
                    }
                }
            }
        } //     for (ii = 0; ii < count; ii++)
        theData.myNewPC3d[newcount] = theData.myPC3d[theData.myNbPnt];
        theData.myNewPC2d[newcount] = theData.myPC2d[theData.myNbPnt];

        if ((theData.myNbPnt != newcount) && newcount < myMaxArraySize - 1)
        {
            // Distribution is changed.
            theData.Swap(newcount);
            return true;
        }

        // Increase number of samples in two times.
        newcount = 0;
        for (int n = 0; n < theData.myNbPnt; n++)
        {
            theData.myNewPC3d[newcount] = theData.myPC3d[n];
            theData.myNewPC2d[newcount] = theData.myPC2d[n];
            newcount++;

            if (theData.myNbPnt - n + newcount == myMaxArraySize)
            {
                continue;
            }

            double ucons = 0.5 * (theData.myPC2d[n] + theData.myPC2d[n + 1]);
            double uc3d = 0.5 * (theData.myPC3d[n] + theData.myPC3d[n + 1]);

            Point3D Pcons;
            theData.myCOnS.D0(ucons, out Pcons);
            Projector.Perform(Pcons, uc3d);
            if (Projector.IsDone())
            {
                curp = Projector.Point().Parameter();
                double dist_2 = Projector.SquareDistance();
                if (dist_2 > theBestSqTol)
                {
                    theBestSqTol = dist_2;
                }
                projok = true;
            }
            else
            {
                ProjectPointOnCurve(uc3d, Pcons, theData.myTol, 30, myC3d, out projok, out curp);
            }
            if (projok)
            {
                if (curp > theData.myPC3d[n] + myDeltaMin && curp < theData.myPC3d[n + 1] - myDeltaMin)
                {
                    theData.myNewPC3d[newcount] = curp;
                    theData.myNewPC2d[newcount] = ucons;
                    newcount++;
                }
            }
        }
        theData.myNewPC3d[newcount] = theData.myPC3d[theData.myNbPnt];
        theData.myNewPC2d[newcount] = theData.myPC2d[theData.myNbPnt];

        if (theData.myNbPnt != newcount)
        {
            // Distribution is changed.
            theData.Swap(newcount);
            return true;
        }

        return false;
    }
}
