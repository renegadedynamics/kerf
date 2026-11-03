// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_AxisPlacement.hxx
// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_AxisPlacement.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Objects;

/// <summary>An axis in the plane, an origin and a unit vector (a shareable <see cref="Axis2D"/>).</summary>
public class AxisPlacement2 : Geometry2D
{
    private Axis2D _axis;

    /// <summary>An axis equal to A.</summary>
    public AxisPlacement2(in Axis2D a) => _axis = a;

    /// <summary>The axis of origin P and direction V.</summary>
    public AxisPlacement2(in Point2D p, in Direction2D v) => _axis = new Axis2D(p, v);

    /// <summary>A new, unfrozen axis equal to this one.</summary>
    public override Geometry2D Copy()
    {
        AxisPlacement2 a;
        a = new AxisPlacement2(_axis);
        return a;
    }

    /// <summary>The axis as a <see cref="Axis2D"/>.</summary>
    public Axis2D Ax2d() => _axis;

    /// <summary>The direction.</summary>
    public Direction2D Direction() => _axis.Direction;

    /// <summary>The origin.</summary>
    public Point2D Location() => _axis.Location;

    /// <summary>Reverses the direction in place.</summary>
    internal void Reverse()
    {
        CheckMutable();
        _axis = _axis.Reversed();
    }

    /// <summary>A copy with the direction reversed.</summary>
    public AxisPlacement2 Reversed()
    {
        Axis2D a = _axis;
        a = a.Reversed();
        AxisPlacement2 temp = new AxisPlacement2(a);
        return temp;
    }

    /// <summary>Applies T to this axis.</summary>
    protected internal override void Transform(in Transform2D t)
    {
        CheckMutable();
        _axis = _axis.Transformed(t);
    }

    internal void SetAxis(in Axis2D a)
    {
        CheckMutable();
        _axis = a;
    }

    internal void SetLocation(in Point2D p)
    {
        CheckMutable();
        _axis = _axis.WithLocation(p);
    }

    internal void SetDirection(in Direction2D v)
    {
        CheckMutable();
        _axis = _axis.WithDirection(v);
    }

    /// <summary>The angle, in [-PI, PI], between the directions of this axis and Other.</summary>
    public double Angle(AxisPlacement2 other) => _axis.AngleTo(other.Ax2d());

    // Non-mutating twins of the Set methods (OCCT.NET)

    public AxisPlacement2 WithAxis(in Axis2D a)
    {
        AxisPlacement2 copy = (AxisPlacement2)Copy();
        copy.SetAxis(a);
        return copy;
    }

    public AxisPlacement2 WithLocation(in Point2D p)
    {
        AxisPlacement2 copy = (AxisPlacement2)Copy();
        copy.SetLocation(p);
        return copy;
    }

    public AxisPlacement2 WithDirection(in Direction2D v)
    {
        AxisPlacement2 copy = (AxisPlacement2)Copy();
        copy.SetDirection(v);
        return copy;
    }
}
