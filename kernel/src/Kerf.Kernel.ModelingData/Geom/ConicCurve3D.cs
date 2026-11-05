// Port of: src/ModelingData/TKG3d/Geom/Geom_Conic.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_Conic.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Geometry.Continuity;

namespace Kerf.Geometry;

/// <summary>
/// The common behaviour of conics in 3D space (circles, ellipses, hyperbolas, parabolas). A conic is placed by a
/// right-handed coordinate system (gp_Ax2) whose origin is its centre (the apex of a parabola), whose X and Y
/// directions define its plane and whose X axis is the origin of its parameter; the main direction orients it.
/// </summary>
public abstract class ConicCurve3D : Curve3D
{
    /// <summary>Only the kernel derives from it (unit 10L1).</summary>
    private protected ConicCurve3D()
    {
    }

    private protected Frame3D _pos = new Frame3D();

    /// <summary>Changes the main axis (the normal to the plane); X and Y axes are recomputed.</summary>
    /// <exception cref="ConstructionErrorException">theA1 is parallel to the X axis of the conic.</exception>
    internal void SetAxis(in Axis3D a1)
    {
        CheckMutable();
        _pos = _pos.WithAxis(a1);
    }

    internal void SetLocation(in Point3D p)
    {
        CheckMutable();
        _pos = _pos.WithLocation(p);
    }

    internal void SetPosition(in Frame3D a2)
    {
        CheckMutable();
        _pos = a2;
    }

    /// <summary>A copy with the main axis theA1 (X and Y axes recomputed).</summary>
    public ConicCurve3D WithAxis(in Axis3D a1)
    {
        ConicCurve3D copy = (ConicCurve3D)Copy();
        copy.SetAxis(a1);
        return copy;
    }

    /// <summary>A copy located at theP.</summary>
    public ConicCurve3D WithLocation(in Point3D p)
    {
        ConicCurve3D copy = (ConicCurve3D)Copy();
        copy.SetLocation(p);
        return copy;
    }

    /// <summary>A copy with the local coordinate system theA2.</summary>
    public ConicCurve3D WithPosition(in Frame3D a2)
    {
        ConicCurve3D copy = (ConicCurve3D)Copy();
        copy.SetPosition(a2);
        return copy;
    }

    /// <summary>The main axis, normal to the plane of the conic.</summary>
    public Axis3D Axis => _pos.Axis;

    /// <summary>The centre (circle, ellipse, hyperbola) or apex (parabola).</summary>
    public Point3D Location => _pos.Location;

    /// <summary>The local coordinate system; its X direction is the origin of the parameter.</summary>
    public Frame3D Position => _pos;

    /// <summary>The eccentricity: 0 for a circle, in [0, 1) for an ellipse, more than 1 for a hyperbola, 1 for a parabola.</summary>
    /// <exception cref="DomainErrorException">A hyperbola with a null major radius.</exception>
    public abstract double Eccentricity { get; }

    /// <summary>The X axis: the origin of the parametrization.</summary>
    public Axis3D XAxis => new Axis3D(_pos.Location, _pos.XDirection);

    /// <summary>The Y axis, perpendicular to the X axis in the plane of the conic.</summary>
    public Axis3D YAxis => new Axis3D(_pos.Location, _pos.YDirection);

    /// <summary>Reverses the parametrization by reversing the main direction of the local coordinate system.</summary>
    protected internal override void Reverse()
    {
        CheckMutable();
        Direction3D vz = _pos.Direction;
        vz = vz.Reversed();
        _pos = _pos.WithDirection(vz);
    }

    /// <summary>CN: conics are infinitely smooth.</summary>
    public override Continuity Continuity => CN;

    /// <summary>True.</summary>
    internal override bool IsCN(int n) => true;
}
