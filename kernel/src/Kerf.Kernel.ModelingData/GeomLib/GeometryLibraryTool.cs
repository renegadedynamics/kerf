// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_Tool.hxx
// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_Tool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Kerf.Engine.Extrema.ExtFlag;

namespace Kerf.Engine.Geometry.Library;

/// <summary>
/// Computes the parameters of a point on a curve or a surface (the point must lie on it, or within MaxDist, and the
/// result must be unambiguous), and the deviation of a 2D curve from the chord between two of its points.
/// </summary>
internal static class GeometryLibraryTool
{
    // The functions Parameter(s) are used to compute parameter(s) of point
    // on curves and surfaces. The main rule is that tested point must lied
    // on curves or surfaces otherwise the resulted parameter(s) may be wrong.
    // To make search process more common the MaxDist value is used to define
    // the proximity of point to curve or surface. It is clear that this MaxDist
    // value can't be too high to be not in conflict with previous rule.
    private const double Partolerance = 1.0e-9;

    /// <summary>
    /// The parameter U of a 3D point on a 3D curve, or at a distance at most MaxDist. Returns false (U = 0) if the
    /// point is farther or the computation fails.
    /// </summary>
    public static bool Parameter(Curve3D? curve, in Point3D point, double maxDist, out double u)
    {
        u = 0.0;
        if (curve is null)
        {
            return false;
        }
        //
        u = 0.0;
        double tol = maxDist * maxDist;
        //
        CurveAdaptor gac = new(curve);
        CurvePointExtrema extrema = new(point, gac);
        //
        if (!extrema.IsDone)
        {
            return false;
        }
        //
        int n = extrema.ExtCount;
        if (n <= 0)
        {
            return false;
        }
        //
        int i = 0, iMin = 0;
        double dist2Min = RealLast();
        for (i = 1; i <= n; i++)
        {
            if (extrema.SquareDistance(i) < dist2Min)
            {
                iMin = i;
                dist2Min = extrema.SquareDistance(i);
            }
        }
        if (iMin != 0 && dist2Min <= tol)
        {
            u = (extrema.Point(iMin)).Parameter();
        }
        else
        {
            return false;
        }

        return true;
    }

    /// <summary>
    /// The parameters U, V of a 3D point on a surface, or at a distance at most MaxDist. Returns false (U = V = 0)
    /// if the point is farther or the computation fails.
    /// </summary>
    public static bool Parameters(Surface? surface, in Point3D point, double maxDist, out double u, out double v)
    {
        u = 0.0;
        v = 0.0;
        if (surface is null)
        {
            return false;
        }
        //
        u = 0.0;
        v = 0.0;
        double tol = maxDist * maxDist;
        //
        SurfaceAdaptor gas = new(surface);
        double tolU = Partolerance, tolV = Partolerance;
        //
        ExtPS extrema = new(point, gas, tolU, tolV, ExtFlag_MIN);
        //
        if (!extrema.IsDone)
        {
            return false;
        }
        //
        int n = extrema.ExtCount;
        if (n <= 0)
        {
            return false;
        }
        //
        double dist2Min = RealLast();
        int i = 0, iMin = 0;
        for (i = 1; i <= n; i++)
        {
            if (extrema.SquareDistance(i) < dist2Min)
            {
                dist2Min = extrema.SquareDistance(i);
                iMin = i;
            }
        }
        if (iMin != 0 && dist2Min <= tol)
        {
            extrema.Point(iMin).Parameter(out u, out v);
        }
        else
        {
            return false;
        }

        return true;
    }

    /// <summary>
    /// The parameter U of a 2D point on a 2D curve, or at a distance at most MaxDist. Returns false (U = 0) if the
    /// point is farther or the computation fails.
    /// </summary>
    public static bool Parameter(Curve2D? curve, in Point2D point, double maxDist, out double u)
    {
        u = 0.0;
        if (curve is null)
        {
            return false;
        }
        //
        u = 0.0;
        double tol = maxDist * maxDist;
        //
        CurveAdaptor2D gac = new(curve);
        ExtPC2d extrema = new(point, gac);
        if (!extrema.IsDone)
        {
            return false;
        }
        int n = extrema.ExtCount;
        if (n <= 0)
        {
            return false;
        }
        int i = 0, iMin = 0;
        double dist2Min = RealLast();
        for (i = 1; i <= n; i++)
        {
            if (extrema.SquareDistance(i) < dist2Min)
            {
                dist2Min = extrema.SquareDistance(i);
                iMin = i;
            }
        }
        if (iMin != 0 && dist2Min <= tol)
        {
            u = (extrema.Point(iMin)).Parameter();
        }
        else
        {
            return false;
        }

        return true;
    }

    /// <summary>
    /// Target function to compute deviation of the source 2D-curve. It is one-variate function. Its parameter is a
    /// parameter on the curve. Deviation is a maximal distance between any point in the curve and the given line.
    /// </summary>
    private sealed class FuncSolveDeviation : MultipleVarFunction
    {
        //! The curve (OCCT copies the adaptor; the port keeps the caller's, which it only evaluates)
        private readonly CurveAdaptor2D _curve;

        //! Square modulus of myDirRef (it is constant)
        private readonly double _sqMod;

        //! TRUE if *this is initialized correctly
        private readonly bool _isValid;

        //! Sets the given line
        private readonly Xy _pRef, _dirRef;

        //! Last computed point in the curve
        private Point2D _pointOnCurve = new();

        //! Always directed from myPointOnCurve to the line
        private Vector2D _vecCurvLine = new();

        //! Constructor. Initializes the curve and the line going through two given points.
        public FuncSolveDeviation(CurveAdaptor2D curve, in Xy pf, in Xy pl)
        {
            _curve = curve;
            _pRef = pf;
            _dirRef = pl - pf;
            _sqMod = _dirRef.SquareModulus();
            _isValid = (_sqMod > global::Kerf.Engine.Foundation.Precision.SquarePConfusion());
        }

        //! Compute additional parameters depending on the argument of *this
        public void UpdateFields(double param)
        {
            _curve.D0(param, out _pointOnCurve);
            Xy vt = _pointOnCurve.XY() - _pRef;
            _vecCurvLine = new Vector2D(vt.Dot(_dirRef) * _dirRef / _sqMod - vt);
        }

        //! Returns value of *this (square deviation) and its 1st and 2nd derivative.
        public void ValueAndDerives(double param, out double val, out double d1, out double d2)
        {
            Vector2D aD1;
            Vector2D aD2;
            _curve.D2(param, out _pointOnCurve, out aD1, out aD2);

            Xy vt = _pointOnCurve.XY() - _pRef;
            // C++ converts myDirRef to gp_Vec2d for gp_Vec2d::Crossed
            Vector2D dirRef = new(_dirRef);
            val = vt.Crossed(_dirRef);
            d1 = aD1.Crossed(dirRef);
            d2 = 2.0 * (d1 * d1 + val * aD2.Crossed(dirRef));
            d1 *= 2.0 * val;
            val *= val / _sqMod;
        }

        //! Returns TRUE if the function has been initializes correctly.
        public bool IsValid => _isValid;

        public override int VariableCount => 1;

        //! Returns last computed Point in the given curve.
        public Point2D PointOnCurve() => _pointOnCurve;

        //! Returns last computed vector directed from some point on the curve to the given line.
        public Vector2D VecCurveLine() => _vecCurvLine;

        //! Returns the given line
        public void GetLine(ref Line2D line)
        {
            line = line.WithDirection(new Direction2D(_dirRef));
            line = line.WithLocation(new Point2D(_pRef));
        }

        //! Returns value of *this (square deviation)
        public override bool Value(math_Vector prm, out double val)
        {
            double d1;
            double d2;
            ValueAndDerives(prm.Value(prm.Lower()), out val, out d1, out d2);
            val = -val;
            return true;
        }

        //! Always returns 0. It is used for compatibility with the parent class.
        public override int GetStateNumber() => 0;
    }

    /// <summary>
    /// The maximal deviation between theCurve and the chord joining its points at theFPar and theLPar, found by
    /// Newton iterations from theStartParameter (see the other overload for a start). Returns a negative value if
    /// the deviation cannot be computed.
    /// </summary>
    public static double ComputeDeviation(CurveAdaptor2D curve, double fPar, double lPar,
                                          double startParameter, int nbIters = 100)
    {
        double prm = 0.0;
        Point2D pnt = new();
        Vector2D vec = new();
        Line2D lin = new();
        return ComputeDeviation(curve, fPar, lPar, startParameter, nbIters, ref prm, ref pnt,
                                ref vec, ref lin, false);
    }

    /// <summary>
    /// As the other overload, also returning the parameter and the point where the deviation is reached (in
    /// [theFPar, theLPar]), the vector from that point to the chord (perpendicular to it) and the chord's line.
    /// Each output is left unchanged where OCCT doesn't write it (the line is set once the chord is valid, the others
    /// on success).
    /// </summary>
    public static double ComputeDeviation(CurveAdaptor2D curve, double fPar, double lPar,
                                          double startParameter, int nbIters, ref double prmOnCurve,
                                          ref Point2D ptOnCurve, ref Vector2D vecCurvLine,
                                          ref Line2D line)
    {
        return ComputeDeviation(curve, fPar, lPar, startParameter, nbIters, ref prmOnCurve,
                                ref ptOnCurve, ref vecCurvLine, ref line, true);
    }

    // OCCT's body; theOutputs says whether its optional pointers are given (all of them, or none).
    private static double ComputeDeviation(CurveAdaptor2D curve, double fPar, double lPar,
                                           double startParameter, int nbIters, ref double prmOnCurve,
                                           ref Point2D ptOnCurve, ref Vector2D vecCurvLine,
                                           ref Line2D line, bool outputs)
    {
        // Computed maximal deflection
        if ((startParameter < fPar) || (startParameter > lPar))
        {
            return -1.0;
        }

        Point2D pf = curve.Value(fPar);
        Point2D pl = curve.Value(lPar);

        FuncSolveDeviation func = new(curve, pf.XY(), pl.XY());

        if (!func.IsValid)
        {
            return -1.0;
        }

        if (outputs)
        {
            func.GetLine(ref line);
        }

        double tolDefl = global::Kerf.Engine.Foundation.Precision.PConfusion();

        double d1 = 0.0;
        double d2 = 0.0;
        double u0 = startParameter;
        double umax = startParameter;
        double sqDefl;
        func.ValueAndDerives(u0, out sqDefl, out d1, out d2);
        for (int itr = 1; itr <= nbIters; itr++)
        {
            if (Math.Abs(d2) < global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                break;
            }
            double delta = d1 / d2;
            double u1 = u0 - delta;

            if ((u1 < fPar) || (u1 > lPar))
            {
                break;
            }
            double sqD;
            func.ValueAndDerives(u1, out sqD, out d1, out d2);
            if (sqD > sqDefl)
            {
                umax = u1;
                double dd =
                    sqDefl > 0.0 ? Math.Abs(Math.Sqrt(sqD) - Math.Sqrt(sqDefl)) : Math.Sqrt(sqD);
                sqDefl = sqD;
                if (dd < tolDefl)
                {
                    break;
                }
            }

            if (Math.Abs(u0 - u1) < global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                break;
            }
            u0 = u1;
        }
        if (sqDefl < 0.0)
        {
            return sqDefl;
        }
        if (outputs)
        {
            prmOnCurve = umax;
            func.UpdateFields(umax);
            ptOnCurve = ptOnCurve.WithXY(func.PointOnCurve().XY());
            vecCurvLine = vecCurvLine.WithXY(func.VecCurveLine().XY());
        }
        return Math.Sqrt(sqDefl);
    }

    /// <summary>
    /// A rough maximal deviation between theCurve and the chord joining its points at theFPar and theLPar, found
    /// by a particle swarm over theNbSubIntervals sub-intervals; a start for the Newton overload. Returns a negative
    /// value if the deviation cannot be computed.
    /// </summary>
    public static double ComputeDeviation(CurveAdaptor2D curve, double fPar, double lPar,
                                          int nbSubIntervals, int nbIters = 10)
    {
        double prm = 0.0;
        return ComputeDeviation(curve, fPar, lPar, nbSubIntervals, nbIters, ref prm);
    }

    /// <summary>As the other overload, also returning the parameter of the deviation (unchanged on failure).</summary>
    public static double ComputeDeviation(CurveAdaptor2D curve, double fPar, double lPar,
                                          int nbSubIntervals, int nbIters, ref double prmOnCurve)
    {
        // Computed maximal deflection
        Point2D pf = curve.Value(fPar);
        Point2D pl = curve.Value(lPar);

        FuncSolveDeviation func = new(curve, pf.XY(), pl.XY());

        if (!func.IsValid)
        {
            return -1.0;
        }
        math_Vector aFPar = new(1, 1, fPar);
        math_Vector aLPar = new(1, 1, lPar);
        math_Vector step = new(1, 1, (lPar - fPar) / (10.0 * nbSubIntervals));
        math_Vector outputPnt = new(1, 1, fPar);
        Pso mpso = new(func, aFPar, aLPar, step, nbSubIntervals, nbIters);

        double sqDefl;
        mpso.Perform(step, out sqDefl, outputPnt, nbIters);

        if (sqDefl == RealLast())
        {
            return -1.0;
        }
        prmOnCurve = outputPnt[1];
        return Math.Sqrt(Math.Abs(sqDefl));
    }
}
