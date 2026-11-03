// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_Curve2d.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_Curve2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Occt;

/// <summary>Converts 2D curves to analytical form within a tolerance, or simplifies them.</summary>
public static class ShapeCustom_Curve2d
{
    private static Line2D GetLine(in Point2D P1, in Point2D P2, double c1, out double cf, out double cl)
    {
        Vector2D avec = new Vector2D(P1, P2);
        Direction2D adir = new Direction2D(avec);
        Line2D alin = new Line2D(P1, adir);
        alin.SetLocation(global::Kerf.Engine.Geometry.ElCLib.Value(c1, alin));
        cf = global::Kerf.Engine.Geometry.ElCLib.Parameter(alin, P1);
        cl = global::Kerf.Engine.Geometry.ElCLib.Parameter(alin, P2);
        return alin;
    }

    /// <summary>
    /// True if the poles lie on a line within the tolerance; then <paramref name="Deviation"/> is their largest
    /// distance to it (left unchanged otherwise).
    /// </summary>
    public static bool IsLinear(NCollection_Array1<Point2D> thePoles, double tolerance, ref double Deviation)
    {
        int nbPoles = thePoles.Length();
        if (nbPoles < 2)
        {
            return false;
        }

        double dMax = 0;
        int iMax1 = 0, iMax2 = 0;

        int i;
        for (i = 1; i < nbPoles; i++)
        {
            for (int j = i + 1; j <= nbPoles; j++)
            {
                double dist = thePoles[i].SquareDistance(thePoles[j]);
                if (dist > dMax)
                {
                    dMax = dist;
                    iMax1 = i;
                    iMax2 = j;
                }
            }
        }

        double dPreci = global::Kerf.Engine.Foundation.Precision.PConfusion() * global::Kerf.Engine.Foundation.Precision.PConfusion();
        if (dMax < dPreci)
        {
            return false;
        }

        double tol2 = tolerance * tolerance;
        Vector2D avec = new Vector2D(thePoles[iMax1], thePoles[iMax2]);
        Direction2D adir = new Direction2D(avec);
        Line2D alin = new Line2D(thePoles[iMax1], adir);

        double aMax = 0.0;
        for (i = 1; i <= nbPoles; i++)
        {
            double dist = alin.SquareDistance(thePoles[i]);
            if (dist > tol2)
            {
                return false;
            }
            if (dist > aMax)
            {
                aMax = dist;
            }
        }
        Deviation = Math.Sqrt(aMax);

        return true;
    }

    /// <summary>
    /// The B-spline or Bezier curve as a 2D line between parameters <paramref name="c1"/> and <paramref name="c2"/>,
    /// if its poles are linear within the tolerance: then <paramref name="cf"/> and <paramref name="cl"/> are the
    /// line's parameters of the curve's points at c1 and c2, and <paramref name="theDeviation"/> the poles' largest
    /// distance to the line. Null otherwise (the outputs then left unchanged).
    /// </summary>
    public static LineCurve2D? ConvertToLine2d(Curve2D theCurve,
                                               double c1,
                                               double c2,
                                               double theTolerance,
                                               ref double cf,
                                               ref double cl,
                                               ref double theDeviation)
    {
        LineCurve2D? aLine2d = null;
        Point2D P1 = theCurve.Value(c1);
        Point2D P2 = theCurve.Value(c2);
        double dPreci = theTolerance * theTolerance;
        if (P1.SquareDistance(P2) < dPreci)
        {
            return aLine2d; // it is not a line
        }

        if (theCurve is BSplineCurve2D bsc)
        {
            if (!ShapeCustom_Curve2d.IsLinear(bsc.InternalPoles(), theTolerance, ref theDeviation))
            {
                return aLine2d; // non
            }
            Line2D alin = GetLine(P1, P2, c1, out cf, out cl);
            aLine2d = new LineCurve2D(alin);
            return aLine2d;
        }

        if (theCurve is BezierCurve2D bzc)
        {
            if (!ShapeCustom_Curve2d.IsLinear(bzc.InternalPoles(), theTolerance, ref theDeviation))
            {
                return aLine2d; // non
            }
            Line2D alin = GetLine(P1, P2, c1, out cf, out cl);
            aLine2d = new LineCurve2D(alin);
            return aLine2d;
        }

        return aLine2d;
    }

    /// <summary>
    /// Removes knots of the B-spline where its local derivatives are the same, within the tolerance. True if knots
    /// were removed.
    /// </summary>
    /// <remarks>
    /// OCCT removes the knots from the caller's curve in place. The port removes them from a copy of a frozen curve,
    /// and gives the copy back through <paramref name="theBSpline2d"/> (decision 974); an unfrozen curve is changed in
    /// place, as in OCCT.
    /// </remarks>
    public static bool SimplifyBSpline2d(ref BSplineCurve2D theBSpline2d, double theTolerance)
    {
        // OCCT.NET: copy on write of a frozen curve (decision 974); the copy is given back only if a knot was removed
        BSplineCurve2D anOriginal = theBSpline2d;
        BSplineCurve2D aBSpline2d = anOriginal.IsFrozen ? (BSplineCurve2D)anOriginal.Copy() : anOriginal;
        int aInitNbK;
        int NbK = aInitNbK = aBSpline2d.KnotCount;
        // search knot to remove
        bool IsToRemove = true;
        int aKnotIndx = NbK - 1;
        while (IsToRemove && NbK > 2)
        {
            int aMult = aBSpline2d.Multiplicity(aKnotIndx);
            int DegMult = aBSpline2d.Degree() - aMult;
            if ((DegMult > 1) && aBSpline2d.IsCN(DegMult))
            {
                double U = aBSpline2d.Knot(aKnotIndx);
                Vector2D aVec1 = aBSpline2d.LocalDN(U, aKnotIndx - 1, aKnotIndx, DegMult);
                Vector2D aVec2 = aBSpline2d.LocalDN(U, aKnotIndx, aKnotIndx + 1, DegMult);
                // check the derivations are have the "same" angle
                if (aVec1.IsParallel(aVec2, global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    // remove knot
                    try
                    {
                        aBSpline2d.RemoveKnot(aKnotIndx, aMult - 1, theTolerance);
                    }
                    catch (FailureException)
                    {
                    }
                }
            }
            aKnotIndx--;

            NbK = aBSpline2d.KnotCount;
            if (aKnotIndx == 1 || aKnotIndx == NbK)
            {
                IsToRemove = false;
            }
        }
        if (aInitNbK > NbK)
        {
            theBSpline2d = aBSpline2d;
        }
        return (aInitNbK > NbK);
    }
}
