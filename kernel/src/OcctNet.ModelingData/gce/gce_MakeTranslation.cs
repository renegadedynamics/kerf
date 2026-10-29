// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeTranslation.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeTranslation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A translation in 3D space, as a gp_Trsf.</summary>
public sealed class gce_MakeTranslation
{
    private gp_Trsf TheTranslation = new();

    /// <summary>A translation by a vector.</summary>
    public gce_MakeTranslation(in gp_Vec Vec) => TheTranslation.SetTranslation(Vec);

    /// <summary>A translation from Point1 to Point2.</summary>
    public gce_MakeTranslation(in gp_Pnt Point1, in gp_Pnt Point2) => TheTranslation.SetTranslation(new gp_Vec(Point1, Point2));

    /// <summary>The constructed transformation.</summary>
    public gp_Trsf Value() => TheTranslation;

    /// <summary>Alias for Value().</summary>
    public gp_Trsf Operator() => Value();

    /// <summary>The constructed transformation (OCCT's implicit conversion operator).</summary>
    public static explicit operator gp_Trsf(gce_MakeTranslation theMaker) => theMaker.Operator();
}
