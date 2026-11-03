// Port of: src/ModelingData/TKGeomBase/BndLib/BndLib_Add3dCurve.hxx
// Port of: src/ModelingData/TKGeomBase/BndLib/BndLib_Add3dCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;

namespace Occt;

/// <summary>
/// Adds the bounding box of a 3D curve to a box (through <see cref="GeomBndLib_Curve"/>). The box is enlarged by the
/// tolerance; an infinite curve opens it.
/// </summary>
public static class BndLib_Add3dCurve
{
    /// <summary>Adds the bounding box of the curve C, over its own range, enlarged by Tol, to B.</summary>
    public static void Add(Adaptor3d_Curve C, double Tol, ref Box B)
    {
        Add(C, C.FirstParameter(), C.LastParameter(), Tol, ref B);
    }

    /// <summary>Adds the bounding box of the arc [U1, U2] of C, enlarged by Tol, to B.</summary>
    public static void Add(Adaptor3d_Curve C, double U1, double U2, double Tol, ref Box B)
    {
        new GeomBndLib_Curve(C).Add(U1, U2, Tol, ref B);
    }

    /// <summary>Adds the precise bounding box of the curve C to B (numerical search of the extrema; slower).</summary>
    public static void AddOptimal(Adaptor3d_Curve C, double Tol, ref Box B)
    {
        AddOptimal(C, C.FirstParameter(), C.LastParameter(), Tol, ref B);
    }

    /// <summary>Adds the precise bounding box of the arc [U1, U2] of C to B.</summary>
    public static void AddOptimal(Adaptor3d_Curve C, double U1, double U2, double Tol, ref Box B)
    {
        new GeomBndLib_Curve(C).AddOptimal(U1, U2, Tol, ref B);
    }
}
