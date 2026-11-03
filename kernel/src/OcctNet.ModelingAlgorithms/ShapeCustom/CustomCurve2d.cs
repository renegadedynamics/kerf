// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_Curve2d.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_Curve2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Healing.Custom;

/// <summary>Converts 2D curves to analytical form within a tolerance, or simplifies them.</summary>
public static class CustomCurve2d
{
    private static Line2D GetLine(in Point2D p1, in Point2D p2, double c1, out double cf, out double cl)
    {
        Vector2D avec = new Vector2D(p1, p2);
        Direction2D adir = new Direction2D(avec);
        Line2D alin = new Line2D(p1, adir);
        alin.SetLocation(global::Kerf.Engine.Geometry.ElCLib.Value(c1, alin));
        cf = global::Kerf.Engine.Geometry.ElCLib.Parameter(alin, p1);
        cl = global::Kerf.Engine.Geometry.ElCLib.Parameter(alin, p2);
        return alin;
    }

    /// <summary>
    /// True if the poles lie on a line within the tolerance; then <paramref name="deviation"/> is their largest
    /// distance to it (left unchanged otherwise).
    /// </summary>
    public static bool IsLinear(NCollection_Array1<Point2D> poles, double tolerance, ref double deviation)
    {
        int nbPoles = poles.Length();
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
                double dist = poles[i].SquareDistance(poles[j]);
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
        Vector2D avec = new Vector2D(poles[iMax1], poles[iMax2]);
        Direction2D adir = new Direction2D(avec);
        Line2D alin = new Line2D(poles[iMax1], adir);

        double max = 0.0;
        for (i = 1; i <= nbPoles; i++)
        {
            double dist = alin.SquareDistance(poles[i]);
            if (dist > tol2)
            {
                return false;
            }
            if (dist > max)
            {
                max = dist;
            }
        }
        deviation = Math.Sqrt(max);

        return true;
    }

    /// <summary>
    /// The B-spline or Bezier curve as a 2D line between parameters <paramref name="c1"/> and <paramref name="c2"/>,
    /// if its poles are linear within the tolerance: then <paramref name="cf"/> and <paramref name="cl"/> are the
    /// line's parameters of the curve's points at c1 and c2, and <paramref name="deviation"/> the poles' largest
    /// distance to the line. Null otherwise (the outputs then left unchanged).
    /// </summary>
    public static LineCurve2D? ConvertToLine2d(Curve2D curve,
                                               double c1,
                                               double c2,
                                               double tolerance,
                                               ref double cf,
                                               ref double cl,
                                               ref double deviation)
    {
        LineCurve2D? line2d = null;
        Point2D p1 = curve.Value(c1);
        Point2D p2 = curve.Value(c2);
        double dPreci = tolerance * tolerance;
        if (p1.SquareDistance(p2) < dPreci)
        {
            return line2d; // it is not a line
        }

        if (curve is BSplineCurve2D bsc)
        {
            if (!CustomCurve2d.IsLinear(bsc.InternalPoles(), tolerance, ref deviation))
            {
                return line2d; // non
            }
            Line2D alin = GetLine(p1, p2, c1, out cf, out cl);
            line2d = new LineCurve2D(alin);
            return line2d;
        }

        if (curve is BezierCurve2D bzc)
        {
            if (!CustomCurve2d.IsLinear(bzc.InternalPoles(), tolerance, ref deviation))
            {
                return line2d; // non
            }
            Line2D alin = GetLine(p1, p2, c1, out cf, out cl);
            line2d = new LineCurve2D(alin);
            return line2d;
        }

        return line2d;
    }

    /// <summary>
    /// Removes knots of the B-spline where its local derivatives are the same, within the tolerance. True if knots
    /// were removed.
    /// </summary>
    /// <remarks>
    /// OCCT removes the knots from the caller's curve in place. The port removes them from a copy of a frozen curve,
    /// and gives the copy back through <paramref name="bSpline2d"/> (decision 974); an unfrozen curve is changed in
    /// place, as in OCCT.
    /// </remarks>
    public static bool SimplifyBSpline2d(ref BSplineCurve2D bSpline2d, double tolerance)
    {
        // OCCT.NET: copy on write of a frozen curve (decision 974); the copy is given back only if a knot was removed
        BSplineCurve2D original = bSpline2d;
        BSplineCurve2D aBSpline2d = original.IsFrozen ? (BSplineCurve2D)original.Copy() : original;
        int initNbK;
        int nbK = initNbK = aBSpline2d.KnotCount;
        // search knot to remove
        bool isToRemove = true;
        int knotIndx = nbK - 1;
        while (isToRemove && nbK > 2)
        {
            int mult = aBSpline2d.Multiplicity(knotIndx);
            int degMult = aBSpline2d.Degree() - mult;
            if ((degMult > 1) && aBSpline2d.IsCN(degMult))
            {
                double u = aBSpline2d.Knot(knotIndx);
                Vector2D vec1 = aBSpline2d.LocalDN(u, knotIndx - 1, knotIndx, degMult);
                Vector2D vec2 = aBSpline2d.LocalDN(u, knotIndx, knotIndx + 1, degMult);
                // check the derivations are have the "same" angle
                if (vec1.IsParallel(vec2, global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    // remove knot
                    try
                    {
                        aBSpline2d.RemoveKnot(knotIndx, mult - 1, tolerance);
                    }
                    catch (FailureException)
                    {
                    }
                }
            }
            knotIndx--;

            nbK = aBSpline2d.KnotCount;
            if (knotIndx == 1 || knotIndx == nbK)
            {
                isToRemove = false;
            }
        }
        if (initNbK > nbK)
        {
            bSpline2d = aBSpline2d;
        }
        return (initNbK > nbK);
    }
}
