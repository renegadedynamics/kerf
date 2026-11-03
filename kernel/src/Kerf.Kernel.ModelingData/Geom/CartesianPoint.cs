// Port of: src/ModelingData/TKG3d/Geom/Geom_CartesianPoint.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_CartesianPoint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Objects;

/// <summary>A point in 3D space defined by its Cartesian coordinates (a shareable <see cref="Point3D"/>).</summary>
public class CartesianPoint : Point
{
    private Point3D _gpPnt;

    /// <summary>A point equal to P.</summary>
    public CartesianPoint(in Point3D p) => _gpPnt = p;

    /// <summary>The point (X, Y, Z).</summary>
    public CartesianPoint(double x, double y, double z) => _gpPnt = new Point3D(x, y, z);

    /// <summary>A new, unfrozen point equal to this one.</summary>
    public override Geometry3D Copy()
    {
        CartesianPoint p;
        p = new CartesianPoint(_gpPnt);
        return p;
    }

    internal void SetCoord(double x, double y, double z)
    {
        CheckMutable();
        _gpPnt = _gpPnt.WithCoord(x, y, z);
    }

    internal void SetPnt(in Point3D p)
    {
        CheckMutable();
        _gpPnt = p;
    }

    internal void SetX(double x)
    {
        CheckMutable();
        _gpPnt = _gpPnt.WithX(x);
    }

    internal void SetY(double y)
    {
        CheckMutable();
        _gpPnt = _gpPnt.WithY(y);
    }

    internal void SetZ(double z)
    {
        CheckMutable();
        _gpPnt = _gpPnt.WithZ(z);
    }

    public override void Coord(out double x, out double y, out double z) => _gpPnt.Coord(out x, out y, out z);

    public override Point3D Pnt() => _gpPnt;

    public override double X() => _gpPnt.X;

    public override double Y() => _gpPnt.Y;

    public override double Z() => _gpPnt.Z;

    /// <summary>Applies T to this point.</summary>
    protected internal override void Transform(in Transform3D t)
    {
        CheckMutable();
        _gpPnt = _gpPnt.Transformed(t);
    }

    // Non-mutating twins of the Set methods (OCCT.NET)

    public CartesianPoint WithCoord(double x, double y, double z)
    {
        CartesianPoint copy = (CartesianPoint)Copy();
        copy.SetCoord(x, y, z);
        return copy;
    }

    public CartesianPoint WithPnt(in Point3D p)
    {
        CartesianPoint copy = (CartesianPoint)Copy();
        copy.SetPnt(p);
        return copy;
    }

    public CartesianPoint WithX(double x)
    {
        CartesianPoint copy = (CartesianPoint)Copy();
        copy.SetX(x);
        return copy;
    }

    public CartesianPoint WithY(double y)
    {
        CartesianPoint copy = (CartesianPoint)Copy();
        copy.SetY(y);
        return copy;
    }

    public CartesianPoint WithZ(double z)
    {
        CartesianPoint copy = (CartesianPoint)Copy();
        copy.SetZ(z);
        return copy;
    }
}
