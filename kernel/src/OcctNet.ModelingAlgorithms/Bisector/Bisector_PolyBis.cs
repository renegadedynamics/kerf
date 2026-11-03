// Port of: src/ModelingAlgorithms/TKTopAlgo/Bisector/Bisector_PolyBis.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/Bisector/Bisector_PolyBis.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace Occt;

/// <summary>The polygon of a Bisector_BisecCC: its points (Bisector_PointOnBis), indexed from 1.</summary>
/// <remarks>
/// OCCT's class is a value holding a C array of 30 points (index 0 unused). The port makes it a class, a working
/// object of Bisector_BisecCC, with internal mutators; OCCT's copies (BisecCC's <c>Polygon(P)</c> setter) are
/// written as copies (the copy constructor). An index past the array throws IndexOutOfRangeException where OCCT
/// would read or write past it (OCCT's polygons hold at most 21 points).
/// </remarks>
public sealed class Bisector_PolyBis
{
    private readonly Bisector_PointOnBis[] thePoints = new Bisector_PointOnBis[30];
    private int nbPoints;

    /// <summary>An empty polygon.</summary>
    public Bisector_PolyBis()
    {
        for (int i = 0; i < thePoints.Length; i++)
        {
            thePoints[i] = new Bisector_PointOnBis();
        }

        nbPoints = 0;
    }

    /// <summary>A copy of theOther.</summary>
    public Bisector_PolyBis(Bisector_PolyBis theOther)
    {
        Array.Copy(theOther.thePoints, thePoints, thePoints.Length);
        nbPoints = theOther.nbPoints;
    }

    internal void Append(in Bisector_PointOnBis P)
    {
        nbPoints++;
        thePoints[nbPoints] = P;
    }

    /// <summary>The number of points.</summary>
    public int Length() => nbPoints;

    /// <summary>True if the polygon has no point.</summary>
    public bool IsEmpty() => (nbPoints == 0);

    /// <summary>The point Index (from 1).</summary>
    public Bisector_PointOnBis Value(int Index) => thePoints[Index];

    /// <summary>The first point.</summary>
    public Bisector_PointOnBis First() => thePoints[1];

    /// <summary>The last point.</summary>
    public Bisector_PointOnBis Last() => thePoints[nbPoints];

    /// <summary>The index of the polygon's segment that contains the parameter U on the bisector.</summary>
    public int Interval(double U)
    {
        if (Last().ParamOnBis() - U < Constants.Resolution())
        {
            return nbPoints - 1;
        }

        double dU = (Last().ParamOnBis() - First().ParamOnBis()) / (nbPoints - 1);

        if (dU <= Constants.Resolution())
        {
            return 1;
        }

        int IntU = (int)(Math.Abs(U - First().ParamOnBis()) / dU);
        IntU++;

        if (thePoints[IntU].ParamOnBis() >= U)
        {
            for (int i = IntU; i >= 1; i--)
            {
                if (thePoints[i].ParamOnBis() <= U)
                {
                    IntU = i;
                    break;
                }
            }
        }
        else
        {
            for (int i = IntU; i <= nbPoints - 1; i++)
            {
                if (thePoints[i].ParamOnBis() >= U)
                {
                    IntU = i - 1;
                    break;
                }
            }
        }

        return IntU;
    }

    internal void Transform(in Transform2D T)
    {
        for (int i = 1; i <= nbPoints; i++)
        {
            Point2D P = thePoints[i].Point();
            P.Transform(T);
            thePoints[i].Point(P);
        }
    }

    // OCCT's Dump (std::cout) is left out.
}
