// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GGenExtCC.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;

using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Extrema;

/// <summary>
/// The anonymous-namespace helpers of OCCT's template Extrema_GGenExtCC, shared by its two instantiations
/// <see cref="Ecc"/> and <see cref="Ecc2d"/>.
/// </summary>
internal static class GGenExtCC
{
    // Comparator, used in std::sort.
    internal static bool Extrema_GGenExtCC_comp(in Xy a, in Xy b)
    {
        if (a.X() < b.X())
        {
            return true;
        }
        else
        {
            if (a.X() == b.X())
            {
                if (a.Y() < b.Y())
                {
                    return true;
                }
            }
        }
        return false;
    }

    /// <summary>The comparator as a <see cref="Comparison{T}"/>. The points are sorted by (X, Y); two points that
    /// compare equal are identical (NaN excepted), so the result does not depend on the sort algorithm.</summary>
    internal static int Extrema_GGenExtCC_Compare(Xy a, Xy b) =>
        Extrema_GGenExtCC_comp(a, b) ? -1 : (Extrema_GGenExtCC_comp(b, a) ? 1 : 0);

    /// <summary>Splits the intervals theInts into theNbInts: one interval evenly, otherwise by halving the longest
    /// interval repeatedly.</summary>
    internal static void Extrema_GGenExtCC_ChangeIntervals(ref NCollection_HArray1<double> ints, int nbInts)
    {
        int aNbInts = ints.Length() - 1;
        int nbAdd = nbInts - aNbInts;
        NCollection_HArray1<double> newInts = new(1, nbInts + 1);
        int nbLast = ints.Length();
        int i;
        if (aNbInts == 1)
        {
            newInts.SetValue(1, ints.First());
            newInts.SetValue(nbInts + 1, ints.Last());
            double dt = (ints.Last() - ints.First()) / nbInts;
            double t = ints.First() + dt;
            for (i = 2; i <= nbInts; ++i, t += dt)
            {
                newInts.SetValue(i, t);
            }
            ints = newInts;
            return;
        }
        for (i = 1; i <= nbLast; ++i)
        {
            newInts.SetValue(i, ints.Value(i));
        }
        while (nbAdd > 0)
        {
            double lIntMax = -1.0;
            int maxInd = -1;
            for (i = 1; i < nbLast; ++i)
            {
                double l = newInts.Value(i + 1) - newInts.Value(i);
                if (l > lIntMax)
                {
                    lIntMax = l;
                    maxInd = i;
                }
            }

            double t = (newInts.Value(maxInd + 1) + newInts.Value(maxInd)) / 2.0;
            for (i = nbLast; i > maxInd; --i)
            {
                newInts.SetValue(i + 1, newInts.Value(i));
            }
            nbLast++;
            nbAdd--;
            newInts.SetValue(maxInd + 1, t);
        }
        ints = newInts;
    }

    /// <summary>The inspector of the cell filter that drops solutions closer than the tolerance to a kept one.</summary>
    internal struct Extrema_GGenExtCC_PointsInspector : ICellFilterInspector<Xy, Xy>
    {
        public static int Dimension => 2;

        public static double Coord(int i, Xy pnt) => pnt.Coord(i + 1);

        public static Xy Shift(in Xy pnt, double tol) => new(pnt.X() + tol, pnt.Y() + tol);

        private readonly double _tol;
        private Xy _current;
        private bool _isFind;

        public Extrema_GGenExtCC_PointsInspector(double tol)
        {
            _tol = tol * tol;
            _isFind = false;
            _current = new Xy();
        }

        public void ClearFind() => _isFind = false;

        public readonly bool IsFind() => _isFind;

        public void SetCurrent(in Xy curPnt) => _current = curPnt;

        public CellFilterAction Inspect(Xy theObject)
        {
            Xy pt = _current.Subtracted(theObject);
            double sqDist = pt.SquareModulus();
            if (sqDist < _tol)
            {
                _isFind = true;
            }
            return CellFilterAction.CellFilter_Keep;
        }
    }

    /// <summary>The least square distance from theP to the curve of theProjTool (RealLast if there is no extremum).</summary>
    internal static double Extrema_GGenExtCC_ProjPOnC(in Point3D p, CurvePointExtrema projTool)
    {
        double dist = RealLast();
        projTool.Perform(p);
        if (projTool.IsDone && projTool.ExtCount != 0)
        {
            for (int i = 1; i <= projTool.ExtCount; ++i)
            {
                double d = projTool.SquareDistance(i);
                if (d < dist)
                {
                    dist = d;
                }
            }
        }
        return dist;
    }

    /// <summary>The least square distance from theP to the curve of theProjTool (RealLast if there is no extremum).</summary>
    internal static double Extrema_GGenExtCC_ProjPOnC(in Point2D p, ExtPC2d projTool)
    {
        double dist = RealLast();
        projTool.Perform(p);
        if (projTool.IsDone && projTool.ExtCount != 0)
        {
            for (int i = 1; i <= projTool.ExtCount; ++i)
            {
                double d = projTool.SquareDistance(i);
                if (d < dist)
                {
                    dist = d;
                }
            }
        }
        return dist;
    }
}
