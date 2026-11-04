// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_Intersection.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_Intersection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Intersection.CurveSurface;

/// <summary>The result of a curve/surface intersection: points and segments, and whether the curve is "parallel".</summary>
/// <remarks>
/// An abstract class (OCCT's constructor is protected), with OCCT's protected fields and methods. A point equal to
/// one already stored (parameters within 1e-8, same transition) is not appended. OCCT's <c>Dump</c> (to std::cout)
/// is <see cref="ToString"/>.
/// </remarks>
internal abstract class IntersectionCurveSurfaceIntersection
{
    protected bool _done;
    protected bool _isParallel; // Curve is "parallel" surface
    private readonly NCollection_Sequence<CurveSurfaceIntersectionPoint> _lpnt = new();
    private readonly NCollection_Sequence<IntersectionSegment> _lseg = new();

    protected IntersectionCurveSurfaceIntersection()
    {
        _done = false;
        _isParallel = false;
    }

    private static bool PARAMEQUAL(double a, double b) => Math.Abs((a) - (b)) < (1e-8);

    /// <summary>True if the computation was successful.</summary>
    public bool IsDone => (_done);

    /// <summary>True if the curve is on or parallel to the surface (detected in the analytic cases only).</summary>
    public bool IsParallel => (_isParallel);

    /// <summary>The number of intersection points.</summary>
    public int PointCount
    {
        get
        {
            if (!_done)
            {
                throw new NotDoneException();
            }
            return _lpnt.Length();
        }
    }

    /// <summary>The number of intersection segments.</summary>
    public int SegmentCount
    {
        get
        {
            if (!_done)
            {
                throw new NotDoneException();
            }
            return _lseg.Length();
        }
    }

    /// <summary>The point of range <paramref name="n"/>.</summary>
    public CurveSurfaceIntersectionPoint Point(int n)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        return _lpnt.Value(n);
    }

    /// <summary>The segment of range <paramref name="n"/>.</summary>
    public IntersectionSegment Segment(int n)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        return _lseg.Value(n);
    }

    /// <summary>Copies the results of <paramref name="other"/>.</summary>
    protected void SetValues(IntersectionCurveSurfaceIntersection other)
    {
        if (other._done)
        {
            _lseg.Clear();
            _lpnt.Clear();
            int n = other._lpnt.Length();
            int i;
            for (i = 1; i <= n; i++)
            {
                _lpnt.Append(other._lpnt.Value(i));
            }
            n = other._lseg.Length();
            for (i = 1; i <= n; i++)
            {
                _lseg.Append(other._lseg.Value(i));
            }
            _done = true;
            _isParallel = other._isParallel;
        }
        else
        {
            _done = false;
            _isParallel = false;
        }
    }

    /// <summary>Appends the points and segments of <paramref name="other"/> (the parameters are unused, as in OCCT).</summary>
    protected void Append(IntersectionCurveSurfaceIntersection other, double firstParamOnCurve, double lastParamOnCurve)
    {
        _ = firstParamOnCurve;
        _ = lastParamOnCurve;
        int i, ni;
        if (other._done)
        {
            ni = other._lpnt.Length();
            for (i = 1; i <= ni; i++)
            {
                Append(other.Point(i));
            }
            ni = other._lseg.Length();
            for (i = 1; i <= ni; i++)
            {
                Append(other.Segment(i));
            }
        }
    }

    /// <summary>Appends a point, unless one with the same parameters and transition is already stored.</summary>
    protected void Append(in CurveSurfaceIntersectionPoint otherPoint)
    {
        int i, ni;
        ni = _lpnt.Length();
        for (i = 1; i <= ni; i++)
        {
            otherPoint.Values(out _, out double u, out double v, out double w,
                              out TransitionOnCurve TrOnCurve);
            _lpnt[i].Values(out _, out double anu, out double anv, out double anw,
                           out TransitionOnCurve trOnCurve);
            if (PARAMEQUAL(u, anu))
            {
                if (PARAMEQUAL(v, anv))
                {
                    if (PARAMEQUAL(w, anw))
                    {
                        if (trOnCurve == TrOnCurve)
                        {
                            return;
                        }
                    }
                }
            }
        }
        _lpnt.Append(otherPoint);
    }

    protected void Append(in IntersectionSegment otherSegment) => _lseg.Append(otherSegment);

    /// <summary>Forgets the results.</summary>
    protected void ResetFields()
    {
        if (_done)
        {
            _lseg.Clear();
            _lpnt.Clear();
            _done = false;
            _isParallel = false;
        }
    }

    public override string ToString()
    {
        if (!_done)
        {
            return " Intersection NotDone";
        }
        var text = new System.Text.StringBuilder();
        int i, ni;
        ni = _lpnt.Length();
        for (i = 1; i <= ni; i++)
        {
            text.AppendLine(Point(i).ToString());
        }
        ni = _lseg.Length();
        for (i = 1; i <= ni; i++)
        {
            text.AppendLine(Segment(i).ToString());
        }
        return text.ToString();
    }
}
