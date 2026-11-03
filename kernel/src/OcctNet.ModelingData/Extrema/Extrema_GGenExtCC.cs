// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GGenExtCC.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The anonymous-namespace helpers of OCCT's template Extrema_GGenExtCC, shared by its two instantiations
/// <see cref="Extrema_ECC"/> and <see cref="Extrema_ECC2d"/>.
/// </summary>
internal static class Extrema_GGenExtCC
{
    // Comparator, used in std::sort.
    internal static bool Extrema_GGenExtCC_comp(in Xy theA, in Xy theB)
    {
        if (theA.X() < theB.X())
        {
            return true;
        }
        else
        {
            if (theA.X() == theB.X())
            {
                if (theA.Y() < theB.Y())
                {
                    return true;
                }
            }
        }
        return false;
    }

    /// <summary>The comparator as a <see cref="Comparison{T}"/>. The points are sorted by (X, Y); two points that
    /// compare equal are identical (NaN excepted), so the result does not depend on the sort algorithm.</summary>
    internal static int Extrema_GGenExtCC_Compare(Xy theA, Xy theB) =>
        Extrema_GGenExtCC_comp(theA, theB) ? -1 : (Extrema_GGenExtCC_comp(theB, theA) ? 1 : 0);

    /// <summary>Splits the intervals theInts into theNbInts: one interval evenly, otherwise by halving the longest
    /// interval repeatedly.</summary>
    internal static void Extrema_GGenExtCC_ChangeIntervals(ref NCollection_HArray1<double> theInts, int theNbInts)
    {
        int aNbInts = theInts.Length() - 1;
        int aNbAdd = theNbInts - aNbInts;
        NCollection_HArray1<double> aNewInts = new(1, theNbInts + 1);
        int aNbLast = theInts.Length();
        int i;
        if (aNbInts == 1)
        {
            aNewInts.SetValue(1, theInts.First());
            aNewInts.SetValue(theNbInts + 1, theInts.Last());
            double dt = (theInts.Last() - theInts.First()) / theNbInts;
            double t = theInts.First() + dt;
            for (i = 2; i <= theNbInts; ++i, t += dt)
            {
                aNewInts.SetValue(i, t);
            }
            theInts = aNewInts;
            return;
        }
        for (i = 1; i <= aNbLast; ++i)
        {
            aNewInts.SetValue(i, theInts.Value(i));
        }
        while (aNbAdd > 0)
        {
            double anLIntMax = -1.0;
            int aMaxInd = -1;
            for (i = 1; i < aNbLast; ++i)
            {
                double anL = aNewInts.Value(i + 1) - aNewInts.Value(i);
                if (anL > anLIntMax)
                {
                    anLIntMax = anL;
                    aMaxInd = i;
                }
            }

            double t = (aNewInts.Value(aMaxInd + 1) + aNewInts.Value(aMaxInd)) / 2.0;
            for (i = aNbLast; i > aMaxInd; --i)
            {
                aNewInts.SetValue(i + 1, aNewInts.Value(i));
            }
            aNbLast++;
            aNbAdd--;
            aNewInts.SetValue(aMaxInd + 1, t);
        }
        theInts = aNewInts;
    }

    /// <summary>The inspector of the cell filter that drops solutions closer than the tolerance to a kept one.</summary>
    internal struct Extrema_GGenExtCC_PointsInspector : ICellFilterInspector<Xy, Xy>
    {
        public static int Dimension => 2;

        public static double Coord(int i, Xy thePnt) => thePnt.Coord(i + 1);

        public static Xy Shift(in Xy thePnt, double theTol) => new(thePnt.X() + theTol, thePnt.Y() + theTol);

        private readonly double myTol;
        private Xy myCurrent;
        private bool myIsFind;

        public Extrema_GGenExtCC_PointsInspector(double theTol)
        {
            myTol = theTol * theTol;
            myIsFind = false;
            myCurrent = new Xy();
        }

        public void ClearFind() => myIsFind = false;

        public readonly bool isFind() => myIsFind;

        public void SetCurrent(in Xy theCurPnt) => myCurrent = theCurPnt;

        public CellFilterAction Inspect(Xy theObject)
        {
            Xy aPt = myCurrent.Subtracted(theObject);
            double aSQDist = aPt.SquareModulus();
            if (aSQDist < myTol)
            {
                myIsFind = true;
            }
            return CellFilterAction.CellFilter_Keep;
        }
    }

    /// <summary>The least square distance from theP to the curve of theProjTool (RealLast if there is no extremum).</summary>
    internal static double Extrema_GGenExtCC_ProjPOnC(in Point3D theP, Extrema_ExtPC theProjTool)
    {
        double aDist = RealLast();
        theProjTool.Perform(theP);
        if (theProjTool.IsDone() && theProjTool.NbExt() != 0)
        {
            for (int i = 1; i <= theProjTool.NbExt(); ++i)
            {
                double aD = theProjTool.SquareDistance(i);
                if (aD < aDist)
                {
                    aDist = aD;
                }
            }
        }
        return aDist;
    }

    /// <summary>The least square distance from theP to the curve of theProjTool (RealLast if there is no extremum).</summary>
    internal static double Extrema_GGenExtCC_ProjPOnC(in Point2D theP, Extrema_ExtPC2d theProjTool)
    {
        double aDist = RealLast();
        theProjTool.Perform(theP);
        if (theProjTool.IsDone() && theProjTool.NbExt() != 0)
        {
            for (int i = 1; i <= theProjTool.NbExt(); ++i)
            {
                double aD = theProjTool.SquareDistance(i);
                if (aD < aDist)
                {
                    aDist = aD;
                }
            }
        }
        return aDist;
    }
}
