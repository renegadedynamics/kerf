// Port of: src/ModelingData/TKG2d/Adaptor2d/Adaptor2d_Line2d.hxx
// Port of: src/ModelingData/TKG2d/Adaptor2d/Adaptor2d_Line2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Geometry.Adaptors;

/// <summary>A 2D line segment as an adapted curve; used by Adaptor3d_TopolTool to trim a surface.</summary>
/// <remarks>An adaptor is a working object: <see cref="Load(in Line2D)"/> is public, as in OCCT.</remarks>
internal class Line2d : Curve2DAdaptorBase
{
    private double _ufirst;
    private double _ulast;
    private Axis2D _ax2d = new Axis2D();

    /// <summary>An empty adaptor (the X axis, parameters 0 to 0).</summary>
    public Line2d()
    {
        _ufirst = 0.0;
        _ulast = 0.0;
    }

    /// <summary>The line through P with direction D, between UFirst and ULast.</summary>
    public Line2d(in Point2D p, in Direction2D d, double uFirst, double uLast)
    {
        _ufirst = uFirst;
        _ulast = uLast;
        _ax2d = new Axis2D(p, d);
    }

    public override Curve2DAdaptorBase ShallowCopy()
    {
        Line2d copy = new Line2d();
        copy._ufirst = _ufirst;
        copy._ulast = _ulast;
        copy._ax2d = _ax2d;
        return copy;
    }

    /// <summary>Loads the infinite line L.</summary>
    public void Load(in Line2D l)
    {
        _ax2d = l.Position;
        _ufirst = -global::Kerf.Engine.Foundation.Precision.Infinite();
        _ulast = global::Kerf.Engine.Foundation.Precision.Infinite();
    }

    /// <summary>Loads the line L between Fi and La.</summary>
    public void Load(in Line2D l, double fi, double la)
    {
        _ax2d = l.Position;
        _ufirst = fi;
        _ulast = la;
    }

    public override double FirstParameter() => _ufirst;

    public override double LastParameter() => _ulast;

    public override Continuity Continuity() => CN;

    public override int NbIntervals(Continuity s) => 1;

    public override void Intervals(NCollection_Array1<double> t, Continuity s)
    {
        t[t.Lower()] = _ufirst;
        t[t.Lower() + 1] = _ulast;
    }

    public override Curve2DAdaptorBase Trim(double first, double last, double tol)
    {
        Line2d hl = new Line2d();
        hl.Load(new Line2D(_ax2d), first, last);
        return hl;
    }

    public override bool IsClosed => false;

    public override bool IsPeriodic => false;

    public override double Period() => throw new NoSuchObjectException();

    public override Point2D Value(double x) => global::Kerf.Engine.Geometry.ElCLib.LineValue(x, _ax2d);

    public override void D0(double x, out Point2D p) => p = global::Kerf.Engine.Geometry.ElCLib.LineValue(x, _ax2d);

    public override void D1(double x, out Point2D p, out Vector2D v) => global::Kerf.Engine.Geometry.ElCLib.LineD1(x, _ax2d, out p, out v);

    public override void D2(double x, out Point2D p, out Vector2D v1, out Vector2D v2)
    {
        global::Kerf.Engine.Geometry.ElCLib.LineD1(x, _ax2d, out p, out v1);
        v2 = new Vector2D(0.0, 0.0);
    }

    public override void D3(double x, out Point2D p, out Vector2D v1, out Vector2D v2, out Vector2D v3)
    {
        global::Kerf.Engine.Geometry.ElCLib.LineD1(x, _ax2d, out p, out v1);
        v2 = new Vector2D(0.0, 0.0);
        v3 = new Vector2D(0.0, 0.0);
    }

    /// <summary>The direction for N = 1, a null vector for N &gt; 1.</summary>
    /// <exception cref="OutOfRangeException">N &lt;= 0.</exception>
    public override Vector2D DN(double u, int n)
    {
        if (n <= 0)
        {
            throw new OutOfRangeException();
        }
        if (n == 1)
        {
            return new Vector2D(_ax2d.Direction);
        }
        return new Vector2D(0.0, 0.0);
    }

    public override double Resolution(double r3d) => r3d; // nul !!!!

    public override CurveKind GetType() => global::Kerf.Geometry.CurveKind.Line;

    public override Line2D Line() => new Line2D(_ax2d);

    public override Circle2D Circle() => throw new NoSuchObjectException();

    public override Ellipse2D Ellipse() => throw new NoSuchObjectException();

    public override Hyperbola2D Hyperbola() => throw new NoSuchObjectException();

    public override Parabola2D Parabola() => throw new NoSuchObjectException();

    public override int Degree() => throw new NoSuchObjectException();

    public override bool IsRational => throw new NoSuchObjectException();

    public override int PoleCount => throw new NoSuchObjectException();

    public override int KnotCount => throw new NoSuchObjectException();

    public override BezierCurve2D Bezier() => throw new NoSuchObjectException();

    public override BSplineCurve2D BSpline() => throw new NoSuchObjectException();
}
