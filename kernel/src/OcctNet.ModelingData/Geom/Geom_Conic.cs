// Port of: src/ModelingData/TKG3d/Geom/Geom_Conic.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_Conic.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>
/// The common behaviour of conics in 3D space (circles, ellipses, hyperbolas, parabolas). A conic is placed by a
/// right-handed coordinate system (gp_Ax2) whose origin is its centre (the apex of a parabola), whose X and Y
/// directions define its plane and whose X axis is the origin of its parameter; the main direction orients it.
/// </summary>
public abstract class Geom_Conic : Geom_Curve
{
    protected Frame3D pos = new Frame3D();

    /// <summary>Changes the main axis (the normal to the plane); X and Y axes are recomputed.</summary>
    /// <exception cref="ConstructionErrorException">theA1 is parallel to the X axis of the conic.</exception>
    internal void SetAxis(in Axis3D theA1)
    {
        CheckMutable();
        pos.SetAxis(theA1);
    }

    internal void SetLocation(in Point3D theP)
    {
        CheckMutable();
        pos.SetLocation(theP);
    }

    internal void SetPosition(in Frame3D theA2)
    {
        CheckMutable();
        pos = theA2;
    }

    /// <summary>A copy with the main axis theA1 (X and Y axes recomputed).</summary>
    public Geom_Conic WithAxis(in Axis3D theA1)
    {
        Geom_Conic aCopy = (Geom_Conic)Copy();
        aCopy.SetAxis(theA1);
        return aCopy;
    }

    /// <summary>A copy located at theP.</summary>
    public Geom_Conic WithLocation(in Point3D theP)
    {
        Geom_Conic aCopy = (Geom_Conic)Copy();
        aCopy.SetLocation(theP);
        return aCopy;
    }

    /// <summary>A copy with the local coordinate system theA2.</summary>
    public Geom_Conic WithPosition(in Frame3D theA2)
    {
        Geom_Conic aCopy = (Geom_Conic)Copy();
        aCopy.SetPosition(theA2);
        return aCopy;
    }

    /// <summary>The main axis, normal to the plane of the conic.</summary>
    public Axis3D Axis() => pos.Axis();

    /// <summary>The centre (circle, ellipse, hyperbola) or apex (parabola).</summary>
    public Point3D Location() => pos.Location();

    /// <summary>The local coordinate system; its X direction is the origin of the parameter.</summary>
    public Frame3D Position() => pos;

    /// <summary>The eccentricity: 0 for a circle, in [0, 1) for an ellipse, more than 1 for a hyperbola, 1 for a parabola.</summary>
    /// <exception cref="DomainErrorException">A hyperbola with a null major radius.</exception>
    public abstract double Eccentricity();

    /// <summary>The X axis: the origin of the parametrization.</summary>
    public Axis3D XAxis() => new Axis3D(pos.Location(), pos.XDirection());

    /// <summary>The Y axis, perpendicular to the X axis in the plane of the conic.</summary>
    public Axis3D YAxis() => new Axis3D(pos.Location(), pos.YDirection());

    /// <summary>Reverses the parametrization by reversing the main direction of the local coordinate system.</summary>
    protected internal override void Reverse()
    {
        CheckMutable();
        Direction3D Vz = pos.Direction();
        Vz.Reverse();
        pos.SetDirection(Vz);
    }

    /// <summary>CN: conics are infinitely smooth.</summary>
    public override Continuity Continuity() => CN;

    /// <summary>True.</summary>
    public override bool IsCN(int N) => true;
}
