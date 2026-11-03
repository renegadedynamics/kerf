// Port of: src/ModelingData/TKG3d/Geom/Geom_ElementarySurface.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_ElementarySurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>
/// A surface with a simple parametric equation in a local coordinate system (gp_Ax3): the plane, the cylinder,
/// the cone, the sphere and the torus. The surface is oriented: its normal is always defined.
/// </summary>
/// <remarks>
/// The local coordinate system need not be right-handed: if it is left-handed, the main direction is
/// -(XDirection ^ YDirection) and the trigonometric senses defined by the two are opposite.
/// </remarks>
public abstract class Geom_ElementarySurface : Geom_Surface
{
    protected CoordinateSystem pos = new CoordinateSystem();

    /// <summary>Changes the main axis (Z axis).</summary>
    /// <exception cref="ConstructionErrorException">theA1 is parallel to the X axis of the coordinate system.</exception>
    internal void SetAxis(in Axis3D theA1)
    {
        CheckMutable();
        pos.SetAxis(theA1);
    }

    internal void SetLocation(in Point3D theLoc)
    {
        CheckMutable();
        pos.SetLocation(theLoc);
    }

    internal void SetPosition(in CoordinateSystem theAx3)
    {
        CheckMutable();
        pos = theAx3;
    }

    /// <summary>A copy with the main axis theA1.</summary>
    public Geom_ElementarySurface WithAxis(in Axis3D theA1)
    {
        Geom_ElementarySurface aCopy = (Geom_ElementarySurface)Copy();
        aCopy.SetAxis(theA1);
        return aCopy;
    }

    /// <summary>A copy whose local coordinate system is located at theLoc.</summary>
    public Geom_ElementarySurface WithLocation(in Point3D theLoc)
    {
        Geom_ElementarySurface aCopy = (Geom_ElementarySurface)Copy();
        aCopy.SetLocation(theLoc);
        return aCopy;
    }

    /// <summary>A copy with the local coordinate system theAx3.</summary>
    public Geom_ElementarySurface WithPosition(in CoordinateSystem theAx3)
    {
        Geom_ElementarySurface aCopy = (Geom_ElementarySurface)Copy();
        aCopy.SetPosition(theAx3);
        return aCopy;
    }

    /// <summary>The main axis (Z axis) of the surface.</summary>
    public Axis3D Axis() => pos.Axis();

    /// <summary>The origin of the local coordinate system.</summary>
    public Point3D Location() => pos.Location();

    /// <summary>The local coordinate system.</summary>
    public CoordinateSystem Position() => pos;

    /// <summary>Reverses the U direction (reverses the Y direction of the local coordinate system).</summary>
    protected internal override void UReverse()
    {
        CheckMutable();
        pos.YReverse();
    }

    /// <summary>Reverses the V direction (reverses the main direction of the local coordinate system).</summary>
    protected internal override void VReverse()
    {
        CheckMutable();
        pos.ZReverse();
    }

    /// <summary>CN, the continuity of every elementary surface.</summary>
    public override Continuity Continuity() => CN;

    /// <summary>True.</summary>
    public override bool IsCNu(int N) => true;

    /// <summary>True.</summary>
    public override bool IsCNv(int N) => true;
}
