// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_CartesianPoint.hxx
// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_CartesianPoint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Objects;

/// <summary>A point in the plane defined by its Cartesian coordinates (a shareable <see cref="Point2D"/>).</summary>
internal class CartesianPoint2 : Point2
{
    private Point2D _gpPnt2d;

    /// <summary>A point equal to P.</summary>
    public CartesianPoint2(in Point2D p) => _gpPnt2d = p;

    /// <summary>The point (X, Y).</summary>
    public CartesianPoint2(double x, double y) => _gpPnt2d = new Point2D(x, y);

    /// <summary>A new, unfrozen point equal to this one.</summary>
    public override Geometry2D Copy()
    {
        CartesianPoint2 p;
        p = new CartesianPoint2(_gpPnt2d);
        return p;
    }

    internal void SetCoord(double x, double y)
    {
        CheckMutable();
        _gpPnt2d = _gpPnt2d.WithCoord(x, y);
    }

    public override void Coord(out double x, out double y) => _gpPnt2d.Coord(out x, out y);

    internal void SetPnt2d(in Point2D p)
    {
        CheckMutable();
        _gpPnt2d = p;
    }

    internal void SetX(double x)
    {
        CheckMutable();
        _gpPnt2d = _gpPnt2d.WithX(x);
    }

    internal void SetY(double y)
    {
        CheckMutable();
        _gpPnt2d = _gpPnt2d.WithY(y);
    }

    public override Point2D Pnt2d() => _gpPnt2d;

    public override double X() => _gpPnt2d.X;

    public override double Y() => _gpPnt2d.Y;

    /// <summary>Applies T to this point.</summary>
    protected internal override void Transform(in Transform2D t)
    {
        CheckMutable();
        _gpPnt2d = _gpPnt2d.Transformed(t);
    }

    // Non-mutating twins of the Set methods (OCCT.NET)

    public CartesianPoint2 WithCoord(double x, double y)
    {
        CartesianPoint2 copy = (CartesianPoint2)Copy();
        copy.SetCoord(x, y);
        return copy;
    }

    public CartesianPoint2 WithPnt2d(in Point2D p)
    {
        CartesianPoint2 copy = (CartesianPoint2)Copy();
        copy.SetPnt2d(p);
        return copy;
    }

    public CartesianPoint2 WithX(double x)
    {
        CartesianPoint2 copy = (CartesianPoint2)Copy();
        copy.SetX(x);
        return copy;
    }

    public CartesianPoint2 WithY(double y)
    {
        CartesianPoint2 copy = (CartesianPoint2)Copy();
        copy.SetY(y);
        return copy;
    }
}
