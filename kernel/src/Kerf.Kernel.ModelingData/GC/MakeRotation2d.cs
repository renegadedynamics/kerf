// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeRotation2d.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeRotation2d.cxx
// Port of: src/ModelingData/TKGeomBase/GCE2d/GCE2d_MakeRotation.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Curves;

/// <summary>A rotation about a point in 2D, as a Geom2d_Transformation.</summary>
/// <remarks>Not a GC_Root: the construction can't fail, as in OCCT.</remarks>
internal sealed class MakeRotation2d
{
    private readonly Transformation2 _theRotation;

    /// <summary>The rotation of angle Angle about Point.</summary>
    public MakeRotation2d(in Point2D point, double angle)
    {
        _theRotation = new Transformation2();
        _theRotation.SetRotation(point, angle);
    }

    /// <summary>The constructed transformation (frozen).</summary>
    public Transformation2 Value()
    {
        _theRotation.Freeze();
        return _theRotation;
    }

    /// <summary>The constructed transformation (OCCT's implicit conversion).</summary>
    public static explicit operator Transformation2(MakeRotation2d maker) => maker.Value();
}
