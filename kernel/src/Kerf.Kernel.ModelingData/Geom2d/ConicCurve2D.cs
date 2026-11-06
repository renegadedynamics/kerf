// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Conic.hxx
// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Conic.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Kerf.Geometry.Continuity;

namespace Kerf.Geometry;

/// <summary>
/// The common behaviour of conics in the plane (circles, ellipses, hyperbolas, parabolas), placed by a local
/// coordinate system (gp_Ax22d, right- or left-handed) whose origin is the centre (the apex of a parabola) and
/// whose X axis is the origin of the parameter. The sense of the coordinate system orients the conic.
/// </summary>
public abstract class ConicCurve2D : Curve2D
{
    /// <summary>Only the kernel derives from it.</summary>
    private protected ConicCurve2D()
    {
    }

    private protected Frame2D _pos = new Frame2D();

    internal void SetAxis(in Frame2D a)
    {
        CheckMutable();
        _pos = _pos.WithAxis(a);
    }

    internal void SetXAxis(in Axis2D ax)
    {
        CheckMutable();
        _pos = _pos.WithXAxis(ax);
    }

    internal void SetYAxis(in Axis2D ay)
    {
        CheckMutable();
        _pos = _pos.WithYAxis(ay);
    }

    internal void SetLocation(in Point2D p)
    {
        CheckMutable();
        _pos = _pos.WithLocation(p);
    }

    /// <summary>A copy with the local coordinate system theA.</summary>
    public ConicCurve2D WithAxis(in Frame2D frame)
    {
        ConicCurve2D copy = (ConicCurve2D)Copy();
        copy.SetAxis(frame);
        return copy;
    }

    /// <summary>A copy with the X axis theAX; the Y axis is recomputed, keeping the sense of the coordinate system.</summary>
    public ConicCurve2D WithXAxis(in Axis2D xAxis)
    {
        ConicCurve2D copy = (ConicCurve2D)Copy();
        copy.SetXAxis(xAxis);
        return copy;
    }

    /// <summary>A copy with the Y axis theAY; the X axis is recomputed, keeping the sense of the coordinate system.</summary>
    public ConicCurve2D WithYAxis(in Axis2D yAxis)
    {
        ConicCurve2D copy = (ConicCurve2D)Copy();
        copy.SetYAxis(yAxis);
        return copy;
    }

    /// <summary>A copy located at theP.</summary>
    public ConicCurve2D WithLocation(in Point2D point)
    {
        ConicCurve2D copy = (ConicCurve2D)Copy();
        copy.SetLocation(point);
        return copy;
    }

    /// <summary>The X axis: the origin of the parametrization.</summary>
    public Axis2D XAxis => new Axis2D(_pos.Location, _pos.XDirection);

    /// <summary>The Y axis, perpendicular to the X axis.</summary>
    public Axis2D YAxis => new Axis2D(_pos.Location, _pos.YDirection);

    /// <summary>The eccentricity: 0 for a circle, in [0, 1) for an ellipse, more than 1 for a hyperbola, 1 for a parabola.</summary>
    public abstract double Eccentricity { get; }

    /// <summary>The centre (circle, ellipse, hyperbola) or apex (parabola).</summary>
    public Point2D Location => _pos.Location;

    /// <summary>The local coordinate system.</summary>
    public Frame2D Position => _pos;

    /// <summary>Reverses the parametrization by reversing the Y direction of the local coordinate system.</summary>
    protected internal override void Reverse()
    {
        CheckMutable();
        Direction2D temp = _pos.YDirection;
        temp = temp.Reversed();
        _pos = _pos.WithAxis(new Frame2D(_pos.Location, _pos.XDirection, temp));
    }

    /// <summary>CN: conics are infinitely smooth.</summary>
    public override Continuity Continuity => CN;

    /// <summary>True.</summary>
    internal override bool IsCN(int n) => true;
}
