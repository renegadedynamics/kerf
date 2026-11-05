// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeRotation.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeRotation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Construction.Curves;

/// <summary>A rotation about an axis, as a Geom_Transformation.</summary>
/// <remarks>Not a GC_Root: the construction can't fail, as in OCCT.</remarks>
internal sealed class MakeRotation
{
    private readonly GeometryTransformation _theRotation;

    /// <summary>The rotation of angle Angle about the line Line.</summary>
    public MakeRotation(in Line3D line, double angle)
    {
        _theRotation = new GeometryTransformation();
        _theRotation.SetRotation(new Axis3D(line.Location, line.Direction), angle);
    }

    /// <summary>The rotation of angle Angle about the axis Axis.</summary>
    public MakeRotation(in Axis3D axis, double angle)
    {
        _theRotation = new GeometryTransformation();
        _theRotation.SetRotation(axis, angle);
    }

    /// <summary>The rotation of angle Angle about the axis through Point with direction Direc.</summary>
    public MakeRotation(in Point3D point, in Direction3D direc, double angle)
    {
        _theRotation = new GeometryTransformation();
        _theRotation.SetRotation(new Axis3D(point, direc), angle);
    }

    /// <summary>The constructed transformation (frozen).</summary>
    public GeometryTransformation Value()
    {
        _theRotation.Freeze();
        return _theRotation;
    }

    /// <summary>The constructed transformation (OCCT's implicit conversion).</summary>
    public static explicit operator GeometryTransformation(MakeRotation maker) => maker.Value();
}
