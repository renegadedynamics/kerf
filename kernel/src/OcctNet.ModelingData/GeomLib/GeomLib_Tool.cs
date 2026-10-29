// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_Tool.hxx
// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_Tool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Extrema_ExtFlag;

namespace Occt;

/// <summary>
/// Computes the parameters of a point on a curve or a surface (the point must lie on it, or within MaxDist, and the
/// result must be unambiguous), and the deviation of a 2D curve from the chord between two of its points.
/// </summary>
public static class GeomLib_Tool
{
    // The functions Parameter(s) are used to compute parameter(s) of point
    // on curves and surfaces. The main rule is that tested point must lied
    // on curves or surfaces otherwise the resulted parameter(s) may be wrong.
    // To make search process more common the MaxDist value is used to define
    // the proximity of point to curve or surface. It is clear that this MaxDist
    // value can't be too high to be not in conflict with previous rule.
    private const double PARTOLERANCE = 1.0e-9;

    /// <summary>
    /// The parameter U of a 3D point on a 3D curve, or at a distance at most MaxDist. Returns false (U = 0) if the
    /// point is farther or the computation fails.
    /// </summary>
    public static bool Parameter(Geom_Curve? Curve, in gp_Pnt Point, double MaxDist, out double U)
    {
        U = 0.0;
        if (Curve is null)
        {
            return false;
        }
        //
        U = 0.0;
        double aTol = MaxDist * MaxDist;
        //
        GeomAdaptor_Curve aGAC = new(Curve);
        Extrema_ExtPC extrema = new(Point, aGAC);
        //
        if (!extrema.IsDone())
        {
            return false;
        }
        //
        int n = extrema.NbExt();
        if (n <= 0)
        {
            return false;
        }
        //
        int i = 0, iMin = 0;
        double Dist2Min = RealLast();
        for (i = 1; i <= n; i++)
        {
            if (extrema.SquareDistance(i) < Dist2Min)
            {
                iMin = i;
                Dist2Min = extrema.SquareDistance(i);
            }
        }
        if (iMin != 0 && Dist2Min <= aTol)
        {
            U = (extrema.Point(iMin)).Parameter();
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
    public static bool Parameters(Geom_Surface? Surface, in gp_Pnt Point, double MaxDist, out double U, out double V)
    {
        U = 0.0;
        V = 0.0;
        if (Surface is null)
        {
            return false;
        }
        //
        U = 0.0;
        V = 0.0;
        double aTol = MaxDist * MaxDist;
        //
        GeomAdaptor_Surface aGAS = new(Surface);
        double aTolU = PARTOLERANCE, aTolV = PARTOLERANCE;
        //
        Extrema_ExtPS extrema = new(Point, aGAS, aTolU, aTolV, Extrema_ExtFlag_MIN);
        //
        if (!extrema.IsDone())
        {
            return false;
        }
        //
        int n = extrema.NbExt();
        if (n <= 0)
        {
            return false;
        }
        //
        double Dist2Min = RealLast();
        int i = 0, iMin = 0;
        for (i = 1; i <= n; i++)
        {
            if (extrema.SquareDistance(i) < Dist2Min)
            {
                Dist2Min = extrema.SquareDistance(i);
                iMin = i;
            }
        }
        if (iMin != 0 && Dist2Min <= aTol)
        {
            extrema.Point(iMin).Parameter(out U, out V);
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
    public static bool Parameter(Geom2d_Curve? Curve, in gp_Pnt2d Point, double MaxDist, out double U)
    {
        U = 0.0;
        if (Curve is null)
        {
            return false;
        }
        //
        U = 0.0;
        double aTol = MaxDist * MaxDist;
        //
        Geom2dAdaptor_Curve aGAC = new(Curve);
        Extrema_ExtPC2d extrema = new(Point, aGAC);
        if (!extrema.IsDone())
        {
            return false;
        }
        int n = extrema.NbExt();
        if (n <= 0)
        {
            return false;
        }
        int i = 0, iMin = 0;
        double Dist2Min = RealLast();
        for (i = 1; i <= n; i++)
        {
            if (extrema.SquareDistance(i) < Dist2Min)
            {
                Dist2Min = extrema.SquareDistance(i);
                iMin = i;
            }
        }
        if (iMin != 0 && Dist2Min <= aTol)
        {
            U = (extrema.Point(iMin)).Parameter();
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
    private sealed class FuncSolveDeviation : math_MultipleVarFunction
    {
        //! The curve (OCCT copies the adaptor; the port keeps the caller's, which it only evaluates)
        private readonly Geom2dAdaptor_Curve myCurve;

        //! Square modulus of myDirRef (it is constant)
        private readonly double mySqMod;

        //! TRUE if *this is initialized correctly
        private readonly bool myIsValid;

        //! Sets the given line
        private readonly gp_XY myPRef, myDirRef;

        //! Last computed point in the curve
        private gp_Pnt2d myPointOnCurve = new();

        //! Always directed from myPointOnCurve to the line
        private gp_Vec2d myVecCurvLine = new();

        //! Constructor. Initializes the curve and the line going through two given points.
        public FuncSolveDeviation(Geom2dAdaptor_Curve theCurve, in gp_XY thePf, in gp_XY thePl)
        {
            myCurve = theCurve;
            myPRef = thePf;
            myDirRef = thePl - thePf;
            mySqMod = myDirRef.SquareModulus();
            myIsValid = (mySqMod > Precision.SquarePConfusion());
        }

        //! Compute additional parameters depending on the argument of *this
        public void UpdateFields(double theParam)
        {
            myCurve.D0(theParam, out myPointOnCurve);
            gp_XY aVt = myPointOnCurve.XY() - myPRef;
            myVecCurvLine = new gp_Vec2d(aVt.Dot(myDirRef) * myDirRef / mySqMod - aVt);
        }

        //! Returns value of *this (square deviation) and its 1st and 2nd derivative.
        public void ValueAndDerives(double theParam, out double theVal, out double theD1, out double theD2)
        {
            gp_Vec2d aD1;
            gp_Vec2d aD2;
            myCurve.D2(theParam, out myPointOnCurve, out aD1, out aD2);

            gp_XY aVt = myPointOnCurve.XY() - myPRef;
            // C++ converts myDirRef to gp_Vec2d for gp_Vec2d::Crossed
            gp_Vec2d aDirRef = new(myDirRef);
            theVal = aVt.Crossed(myDirRef);
            theD1 = aD1.Crossed(aDirRef);
            theD2 = 2.0 * (theD1 * theD1 + theVal * aD2.Crossed(aDirRef));
            theD1 *= 2.0 * theVal;
            theVal *= theVal / mySqMod;
        }

        //! Returns TRUE if the function has been initializes correctly.
        public bool IsValid() => myIsValid;

        public override int NbVariables() => 1;

        //! Returns last computed Point in the given curve.
        public gp_Pnt2d PointOnCurve() => myPointOnCurve;

        //! Returns last computed vector directed from some point on the curve to the given line.
        public gp_Vec2d VecCurveLine() => myVecCurvLine;

        //! Returns the given line
        public void GetLine(ref gp_Lin2d theLine)
        {
            theLine.SetDirection(new gp_Dir2d(myDirRef));
            theLine.SetLocation(new gp_Pnt2d(myPRef));
        }

        //! Returns value of *this (square deviation)
        public override bool Value(math_Vector thePrm, out double theVal)
        {
            double aD1;
            double aD2;
            ValueAndDerives(thePrm.Value(thePrm.Lower()), out theVal, out aD1, out aD2);
            theVal = -theVal;
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
    public static double ComputeDeviation(Geom2dAdaptor_Curve theCurve, double theFPar, double theLPar,
                                          double theStartParameter, int theNbIters = 100)
    {
        double aPrm = 0.0;
        gp_Pnt2d aPnt = new();
        gp_Vec2d aVec = new();
        gp_Lin2d aLin = new();
        return ComputeDeviation(theCurve, theFPar, theLPar, theStartParameter, theNbIters, ref aPrm, ref aPnt,
                                ref aVec, ref aLin, false);
    }

    /// <summary>
    /// As the other overload, also returning the parameter and the point where the deviation is reached (in
    /// [theFPar, theLPar]), the vector from that point to the chord (perpendicular to it) and the chord's line.
    /// Each output is left unchanged where OCCT doesn't write it (the line is set once the chord is valid, the others
    /// on success).
    /// </summary>
    public static double ComputeDeviation(Geom2dAdaptor_Curve theCurve, double theFPar, double theLPar,
                                          double theStartParameter, int theNbIters, ref double thePrmOnCurve,
                                          ref gp_Pnt2d thePtOnCurve, ref gp_Vec2d theVecCurvLine,
                                          ref gp_Lin2d theLine)
    {
        return ComputeDeviation(theCurve, theFPar, theLPar, theStartParameter, theNbIters, ref thePrmOnCurve,
                                ref thePtOnCurve, ref theVecCurvLine, ref theLine, true);
    }

    // OCCT's body; theOutputs says whether its optional pointers are given (all of them, or none).
    private static double ComputeDeviation(Geom2dAdaptor_Curve theCurve, double theFPar, double theLPar,
                                           double theStartParameter, int theNbIters, ref double thePrmOnCurve,
                                           ref gp_Pnt2d thePtOnCurve, ref gp_Vec2d theVecCurvLine,
                                           ref gp_Lin2d theLine, bool theOutputs)
    {
        // Computed maximal deflection
        if ((theStartParameter < theFPar) || (theStartParameter > theLPar))
        {
            return -1.0;
        }

        gp_Pnt2d aPf = theCurve.Value(theFPar);
        gp_Pnt2d aPl = theCurve.Value(theLPar);

        FuncSolveDeviation aFunc = new(theCurve, aPf.XY(), aPl.XY());

        if (!aFunc.IsValid())
        {
            return -1.0;
        }

        if (theOutputs)
        {
            aFunc.GetLine(ref theLine);
        }

        double aTolDefl = Precision.PConfusion();

        double aD1 = 0.0;
        double aD2 = 0.0;
        double aU0 = theStartParameter;
        double aUmax = theStartParameter;
        double aSqDefl;
        aFunc.ValueAndDerives(aU0, out aSqDefl, out aD1, out aD2);
        for (int anItr = 1; anItr <= theNbIters; anItr++)
        {
            if (Math.Abs(aD2) < Precision.PConfusion())
            {
                break;
            }
            double aDelta = aD1 / aD2;
            double aU1 = aU0 - aDelta;

            if ((aU1 < theFPar) || (aU1 > theLPar))
            {
                break;
            }
            double aSqD;
            aFunc.ValueAndDerives(aU1, out aSqD, out aD1, out aD2);
            if (aSqD > aSqDefl)
            {
                aUmax = aU1;
                double aDD =
                    aSqDefl > 0.0 ? Math.Abs(Math.Sqrt(aSqD) - Math.Sqrt(aSqDefl)) : Math.Sqrt(aSqD);
                aSqDefl = aSqD;
                if (aDD < aTolDefl)
                {
                    break;
                }
            }

            if (Math.Abs(aU0 - aU1) < Precision.PConfusion())
            {
                break;
            }
            aU0 = aU1;
        }
        if (aSqDefl < 0.0)
        {
            return aSqDefl;
        }
        if (theOutputs)
        {
            thePrmOnCurve = aUmax;
            aFunc.UpdateFields(aUmax);
            thePtOnCurve.SetXY(aFunc.PointOnCurve().XY());
            theVecCurvLine.SetXY(aFunc.VecCurveLine().XY());
        }
        return Math.Sqrt(aSqDefl);
    }

    /// <summary>
    /// A rough maximal deviation between theCurve and the chord joining its points at theFPar and theLPar, found
    /// by a particle swarm over theNbSubIntervals sub-intervals; a start for the Newton overload. Returns a negative
    /// value if the deviation cannot be computed.
    /// </summary>
    public static double ComputeDeviation(Geom2dAdaptor_Curve theCurve, double theFPar, double theLPar,
                                          int theNbSubIntervals, int theNbIters = 10)
    {
        double aPrm = 0.0;
        return ComputeDeviation(theCurve, theFPar, theLPar, theNbSubIntervals, theNbIters, ref aPrm);
    }

    /// <summary>As the other overload, also returning the parameter of the deviation (unchanged on failure).</summary>
    public static double ComputeDeviation(Geom2dAdaptor_Curve theCurve, double theFPar, double theLPar,
                                          int theNbSubIntervals, int theNbIters, ref double thePrmOnCurve)
    {
        // Computed maximal deflection
        gp_Pnt2d aPf = theCurve.Value(theFPar);
        gp_Pnt2d aPl = theCurve.Value(theLPar);

        FuncSolveDeviation aFunc = new(theCurve, aPf.XY(), aPl.XY());

        if (!aFunc.IsValid())
        {
            return -1.0;
        }
        math_Vector aFPar = new(1, 1, theFPar);
        math_Vector aLPar = new(1, 1, theLPar);
        math_Vector aStep = new(1, 1, (theLPar - theFPar) / (10.0 * theNbSubIntervals));
        math_Vector anOutputPnt = new(1, 1, theFPar);
        math_PSO aMPSO = new(aFunc, aFPar, aLPar, aStep, theNbSubIntervals, theNbIters);

        double aSqDefl;
        aMPSO.Perform(aStep, out aSqDefl, anOutputPnt, theNbIters);

        if (aSqDefl == RealLast())
        {
            return -1.0;
        }
        thePrmOnCurve = anOutputPnt[1];
        return Math.Sqrt(Math.Abs(aSqDefl));
    }
}
