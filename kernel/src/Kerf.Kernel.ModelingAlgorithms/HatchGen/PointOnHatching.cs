// Port of: src/ModelingAlgorithms/TKGeomAlgo/HatchGen/HatchGen_PointOnHatching.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/HatchGen/HatchGen_PointOnHatching.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Intersection.TwoD;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Classification.Hatching;

/// <summary>An intersection point on a hatching, with the points on the elements that meet the hatching there.</summary>
/// <remarks>
/// A value class of OCCT, copied where OCCT copies it (decision 664). A copy has its own list of points on
/// element, which share the (never changed) point objects. The mutators (<see cref="AddPoint"/>,
/// <see cref="RemPoint"/>, <see cref="ClrPoints"/> and the inherited setters) are internal.
/// </remarks>
internal sealed class PointOnHatching : IntersectionPoint
{
    private readonly NCollection_Sequence<PointOnElement> _points;

    /// <summary>An empty point.</summary>
    public PointOnHatching()
    {
        _points = new NCollection_Sequence<PointOnElement>();
    }

    /// <summary>A copy of <paramref name="other"/>, with its own list of points on element.</summary>
    public PointOnHatching(PointOnHatching other)
        : base(other)
    {
        _points = new NCollection_Sequence<PointOnElement>(other._points);
    }

    /// <summary>The point on the hatching (the first curve of <paramref name="point"/>), with no point on element.</summary>
    public PointOnHatching(in IntersectionPoint2D point)
    {
        _points = new NCollection_Sequence<PointOnElement>();
        _index = 0;
        _param = point.ParamOnFirst();
        switch (point.TransitionOfFirst().PositionOnCurve())
        {
            case Position2D.Head:
                _posit = Forward;
                break;
            case Position2D.Middle:
                _posit = Internal;
                break;
            case Position2D.End:
                _posit = Reversed;
                break;
        }
        _before = Unknown;
        _after = Unknown;
        _segBeg = false;
        _segEnd = false;
        _points.Clear();
    }

    /// <summary>Adds a point on element, unless an identical one (within <paramref name="confusion"/>) is there.</summary>
    internal void AddPoint(PointOnElement point, double confusion)
    {
        int nbPnt = _points.Length();
        // for (int IPnt = 1 ;
        int iPnt;
        for (iPnt = 1; iPnt <= nbPnt && _points[iPnt].IsDifferent(point, confusion); iPnt++)
        {
        }
        if (iPnt > nbPnt)
        {
            _points.Append(point);
        }
    }

    /// <summary>The number of elements intersecting the hatching at this point.</summary>
    public int PointCount => _points.Length();

    /// <summary>The <paramref name="index"/>-th point on element.</summary>
    /// <exception cref="OutOfRangeException"><paramref name="index"/> is out of range.</exception>
    public PointOnElement Point(int index) => _points.Value(index);

    /// <summary>Removes the <paramref name="index"/>-th point on element.</summary>
    internal void RemPoint(int index)
    {
        _points.Remove(index);
    }

    /// <summary>Removes all the points on element.</summary>
    internal void ClrPoints()
    {
        _points.Clear();
    }

    /// <summary>True if this point is lower than <paramref name="point"/>: Point's parameter − this one's > Confusion.</summary>
    public bool IsLower(PointOnHatching point, double confusion) => (point._param - _param > confusion);

    /// <summary>True if the parameters of the two points are within <paramref name="confusion"/>.</summary>
    public bool IsEqual(PointOnHatching point, double confusion) =>
        (Math.Abs(point._param - _param) <= confusion);

    /// <summary>True if this point is greater than <paramref name="point"/>: this parameter − Point's > Confusion.</summary>
    public bool IsGreater(PointOnHatching point, double confusion) => (_param - point._param > confusion);

    /// <summary>Writes the point and its points on element to <paramref name="os"/>, as OCCT writes them to std::cout.</summary>
    public override void Dump(TextWriter os, int index = 0)
    {
        os.Write("--- Point on hatching ");
        if (index > 0)
        {
            os.Write($"# {index,3} ");
        }
        else
        {
            os.Write("------");
        }
        os.Write("------------------\n");
        os.Write($"    Index of the hatching = {_index}\n");
        os.Write($"    Parameter on hatching = {Number(_param)}\n");
        os.Write($"    Position  on hatching = {PositionName(_posit)}\n");
        os.Write($"    State Before          = {StateName(_before)}\n");
        os.Write($"    State After           = {StateName(_after)}\n");
        os.Write($"    Beginning of segment  = {(_segBeg ? "TRUE" : "FALSE")}\n");
        os.Write($"    End       of segment  = {(_segEnd ? "TRUE" : "FALSE")}\n");
        int nbPnt = _points.Length();
        if (nbPnt == 0)
        {
            os.Write("    No points on element\n");
        }
        else
        {
            os.Write($"    Contains {nbPnt} points on element\n");
            for (int iPnt = 1; iPnt <= nbPnt; iPnt++)
            {
                PointOnElement point = _points.Value(iPnt);
                point.Dump(os, iPnt);
            }
        }
        os.Write("----------------------------------------------\n");
    }
}
