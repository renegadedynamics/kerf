// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Conic.hxx
// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Conic.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>
/// The common behaviour of conics in the plane (circles, ellipses, hyperbolas, parabolas), placed by a local
/// coordinate system (gp_Ax22d, right- or left-handed) whose origin is the centre (the apex of a parabola) and
/// whose X axis is the origin of the parameter. The sense of the coordinate system orients the conic.
/// </summary>
public abstract class Geom2d_Conic : Geom2d_Curve
{
    protected Frame2D pos = new Frame2D();

    internal void SetAxis(in Frame2D theA)
    {
        CheckMutable();
        pos.SetAxis(theA);
    }

    internal void SetXAxis(in Axis2D theAX)
    {
        CheckMutable();
        pos.SetXAxis(theAX);
    }

    internal void SetYAxis(in Axis2D theAY)
    {
        CheckMutable();
        pos.SetYAxis(theAY);
    }

    internal void SetLocation(in Point2D theP)
    {
        CheckMutable();
        pos.SetLocation(theP);
    }

    /// <summary>A copy with the local coordinate system theA.</summary>
    public Geom2d_Conic WithAxis(in Frame2D theA)
    {
        Geom2d_Conic aCopy = (Geom2d_Conic)Copy();
        aCopy.SetAxis(theA);
        return aCopy;
    }

    /// <summary>A copy with the X axis theAX; the Y axis is recomputed, keeping the sense of the coordinate system.</summary>
    public Geom2d_Conic WithXAxis(in Axis2D theAX)
    {
        Geom2d_Conic aCopy = (Geom2d_Conic)Copy();
        aCopy.SetXAxis(theAX);
        return aCopy;
    }

    /// <summary>A copy with the Y axis theAY; the X axis is recomputed, keeping the sense of the coordinate system.</summary>
    public Geom2d_Conic WithYAxis(in Axis2D theAY)
    {
        Geom2d_Conic aCopy = (Geom2d_Conic)Copy();
        aCopy.SetYAxis(theAY);
        return aCopy;
    }

    /// <summary>A copy located at theP.</summary>
    public Geom2d_Conic WithLocation(in Point2D theP)
    {
        Geom2d_Conic aCopy = (Geom2d_Conic)Copy();
        aCopy.SetLocation(theP);
        return aCopy;
    }

    /// <summary>The X axis: the origin of the parametrization.</summary>
    public Axis2D XAxis() => new Axis2D(pos.Location(), pos.XDirection());

    /// <summary>The Y axis, perpendicular to the X axis.</summary>
    public Axis2D YAxis() => new Axis2D(pos.Location(), pos.YDirection());

    /// <summary>The eccentricity: 0 for a circle, in [0, 1) for an ellipse, more than 1 for a hyperbola, 1 for a parabola.</summary>
    public abstract double Eccentricity();

    /// <summary>The centre (circle, ellipse, hyperbola) or apex (parabola).</summary>
    public Point2D Location() => pos.Location();

    /// <summary>The local coordinate system.</summary>
    public Frame2D Position() => pos;

    /// <summary>Reverses the parametrization by reversing the Y direction of the local coordinate system.</summary>
    protected internal override void Reverse()
    {
        CheckMutable();
        Direction2D Temp = pos.YDirection();
        Temp.Reverse();
        pos.SetAxis(new Frame2D(pos.Location(), pos.XDirection(), Temp));
    }

    /// <summary>CN: conics are infinitely smooth.</summary>
    public override Continuity Continuity() => CN;

    /// <summary>True.</summary>
    public override bool IsCN(int N) => true;
}
