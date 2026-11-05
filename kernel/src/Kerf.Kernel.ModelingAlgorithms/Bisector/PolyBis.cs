// Port of: src/ModelingAlgorithms/TKTopAlgo/Bisector/Bisector_PolyBis.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/Bisector/Bisector_PolyBis.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Offsets.Bisectors;

/// <summary>The polygon of a Bisector_BisecCC: its points (Bisector_PointOnBis), indexed from 1.</summary>
/// <remarks>
/// OCCT's class is a value holding a C array of 30 points (index 0 unused). The port makes it a class, a working
/// object of Bisector_BisecCC, with internal mutators; OCCT's copies (BisecCC's <c>Polygon(P)</c> setter) are
/// written as copies (the copy constructor). An index past the array throws IndexOutOfRangeException where OCCT
/// would read or write past it (OCCT's polygons hold at most 21 points).
/// </remarks>
internal sealed class PolyBis
{
    private readonly PointOnBis[] _points = new PointOnBis[30];
    private int _nbPoints;

    /// <summary>An empty polygon.</summary>
    public PolyBis()
    {
        for (int i = 0; i < _points.Length; i++)
        {
            _points[i] = new PointOnBis();
        }

        _nbPoints = 0;
    }

    /// <summary>A copy of theOther.</summary>
    public PolyBis(PolyBis other)
    {
        Array.Copy(other._points, _points, _points.Length);
        _nbPoints = other._nbPoints;
    }

    internal void Append(in PointOnBis p)
    {
        _nbPoints++;
        _points[_nbPoints] = p;
    }

    /// <summary>The number of points.</summary>
    public int Length() => _nbPoints;

    /// <summary>True if the polygon has no point.</summary>
    public bool IsEmpty => (_nbPoints == 0);

    /// <summary>The point Index (from 1).</summary>
    public PointOnBis Value(int index) => _points[index];

    /// <summary>The first point.</summary>
    public PointOnBis First() => _points[1];

    /// <summary>The last point.</summary>
    public PointOnBis Last() => _points[_nbPoints];

    /// <summary>The index of the polygon's segment that contains the parameter U on the bisector.</summary>
    public int Interval(double u)
    {
        if (Last().ParamOnBis() - u < Constants.Resolution())
        {
            return _nbPoints - 1;
        }

        double dU = (Last().ParamOnBis() - First().ParamOnBis()) / (_nbPoints - 1);

        if (dU <= Constants.Resolution())
        {
            return 1;
        }

        int intU = (int)(Math.Abs(u - First().ParamOnBis()) / dU);
        intU++;

        if (_points[intU].ParamOnBis() >= u)
        {
            for (int i = intU; i >= 1; i--)
            {
                if (_points[i].ParamOnBis() <= u)
                {
                    intU = i;
                    break;
                }
            }
        }
        else
        {
            for (int i = intU; i <= _nbPoints - 1; i++)
            {
                if (_points[i].ParamOnBis() >= u)
                {
                    intU = i - 1;
                    break;
                }
            }
        }

        return intU;
    }

    internal void Transform(in Transform2D t)
    {
        for (int i = 1; i <= _nbPoints; i++)
        {
            Point2D p = _points[i].Point();
            p = p.Transformed(t);
            _points[i].Point(p);
        }
    }

    // OCCT's Dump (std::cout) is left out.
}
