// Port of: src/ModelingAlgorithms/TKGeomAlgo/HatchGen/HatchGen_PointOnHatching.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/HatchGen/HatchGen_PointOnHatching.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.IntRes2d_Position;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>An intersection point on a hatching, with the points on the elements that meet the hatching there.</summary>
/// <remarks>
/// A value class of OCCT, copied where OCCT copies it (decision 664). A copy has its own list of points on
/// element, which share the (never changed) point objects. The mutators (<see cref="AddPoint"/>,
/// <see cref="RemPoint"/>, <see cref="ClrPoints"/> and the inherited setters) are internal.
/// </remarks>
public sealed class HatchGen_PointOnHatching : HatchGen_IntersectionPoint
{
    private readonly NCollection_Sequence<HatchGen_PointOnElement> myPoints;

    /// <summary>An empty point.</summary>
    public HatchGen_PointOnHatching()
    {
        myPoints = new NCollection_Sequence<HatchGen_PointOnElement>();
    }

    /// <summary>A copy of <paramref name="Other"/>, with its own list of points on element.</summary>
    public HatchGen_PointOnHatching(HatchGen_PointOnHatching Other)
        : base(Other)
    {
        myPoints = new NCollection_Sequence<HatchGen_PointOnElement>(Other.myPoints);
    }

    /// <summary>The point on the hatching (the first curve of <paramref name="Point"/>), with no point on element.</summary>
    public HatchGen_PointOnHatching(in IntRes2d_IntersectionPoint Point)
    {
        myPoints = new NCollection_Sequence<HatchGen_PointOnElement>();
        myIndex = 0;
        myParam = Point.ParamOnFirst();
        switch (Point.TransitionOfFirst().PositionOnCurve())
        {
            case IntRes2d_Head:
                myPosit = TopAbs_FORWARD;
                break;
            case IntRes2d_Middle:
                myPosit = TopAbs_INTERNAL;
                break;
            case IntRes2d_End:
                myPosit = TopAbs_REVERSED;
                break;
        }
        myBefore = TopAbs_UNKNOWN;
        myAfter = TopAbs_UNKNOWN;
        mySegBeg = false;
        mySegEnd = false;
        myPoints.Clear();
    }

    /// <summary>Adds a point on element, unless an identical one (within <paramref name="Confusion"/>) is there.</summary>
    internal void AddPoint(HatchGen_PointOnElement Point, double Confusion)
    {
        int NbPnt = myPoints.Length();
        // for (int IPnt = 1 ;
        int IPnt;
        for (IPnt = 1; IPnt <= NbPnt && myPoints[IPnt].IsDifferent(Point, Confusion); IPnt++)
        {
        }
        if (IPnt > NbPnt)
        {
            myPoints.Append(Point);
        }
    }

    /// <summary>The number of elements intersecting the hatching at this point.</summary>
    public int NbPoints() => myPoints.Length();

    /// <summary>The <paramref name="Index"/>-th point on element.</summary>
    /// <exception cref="Standard_OutOfRange"><paramref name="Index"/> is out of range.</exception>
    public HatchGen_PointOnElement Point(int Index) => myPoints.Value(Index);

    /// <summary>Removes the <paramref name="Index"/>-th point on element.</summary>
    internal void RemPoint(int Index)
    {
        myPoints.Remove(Index);
    }

    /// <summary>Removes all the points on element.</summary>
    internal void ClrPoints()
    {
        myPoints.Clear();
    }

    /// <summary>True if this point is lower than <paramref name="Point"/>: Point's parameter − this one's > Confusion.</summary>
    public bool IsLower(HatchGen_PointOnHatching Point, double Confusion) => (Point.myParam - myParam > Confusion);

    /// <summary>True if the parameters of the two points are within <paramref name="Confusion"/>.</summary>
    public bool IsEqual(HatchGen_PointOnHatching Point, double Confusion) =>
        (Math.Abs(Point.myParam - myParam) <= Confusion);

    /// <summary>True if this point is greater than <paramref name="Point"/>: this parameter − Point's > Confusion.</summary>
    public bool IsGreater(HatchGen_PointOnHatching Point, double Confusion) => (myParam - Point.myParam > Confusion);

    /// <summary>Writes the point and its points on element to <paramref name="theOS"/>, as OCCT writes them to std::cout.</summary>
    public override void Dump(TextWriter theOS, int Index = 0)
    {
        theOS.Write("--- Point on hatching ");
        if (Index > 0)
        {
            theOS.Write($"# {Index,3} ");
        }
        else
        {
            theOS.Write("------");
        }
        theOS.Write("------------------\n");
        theOS.Write($"    Index of the hatching = {myIndex}\n");
        theOS.Write($"    Parameter on hatching = {Number(myParam)}\n");
        theOS.Write($"    Position  on hatching = {PositionName(myPosit)}\n");
        theOS.Write($"    State Before          = {StateName(myBefore)}\n");
        theOS.Write($"    State After           = {StateName(myAfter)}\n");
        theOS.Write($"    Beginning of segment  = {(mySegBeg ? "TRUE" : "FALSE")}\n");
        theOS.Write($"    End       of segment  = {(mySegEnd ? "TRUE" : "FALSE")}\n");
        int NbPnt = myPoints.Length();
        if (NbPnt == 0)
        {
            theOS.Write("    No points on element\n");
        }
        else
        {
            theOS.Write($"    Contains {NbPnt} points on element\n");
            for (int IPnt = 1; IPnt <= NbPnt; IPnt++)
            {
                HatchGen_PointOnElement Point = myPoints.Value(IPnt);
                Point.Dump(theOS, IPnt);
            }
        }
        theOS.Write("----------------------------------------------\n");
    }
}
