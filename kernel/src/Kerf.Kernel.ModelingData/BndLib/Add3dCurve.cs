// Port of: src/ModelingData/TKGeomBase/BndLib/BndLib_Add3dCurve.hxx
// Port of: src/ModelingData/TKGeomBase/BndLib/BndLib_Add3dCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Geometry.Bounds.Computation;
using Occt;

namespace Kerf.Engine.Geometry.Bounds;

/// <summary>
/// Adds the bounding box of a 3D curve to a box (through <see cref="CurveBounds"/>). The box is enlarged by the
/// tolerance; an infinite curve opens it.
/// </summary>
internal static class Add3dCurve
{
    /// <summary>Adds the bounding box of the curve C, over its own range, enlarged by Tol, to B.</summary>
    public static void Add(CurveAdaptorBase c, double tol, ref Box b)
    {
        Add(c, c.FirstParameter(), c.LastParameter(), tol, ref b);
    }

    /// <summary>Adds the bounding box of the arc [U1, U2] of C, enlarged by Tol, to B.</summary>
    public static void Add(CurveAdaptorBase c, double u1, double u2, double tol, ref Box b)
    {
        new CurveBounds(c).Add(u1, u2, tol, ref b);
    }

    /// <summary>Adds the precise bounding box of the curve C to B (numerical search of the extrema; slower).</summary>
    public static void AddOptimal(CurveAdaptorBase c, double tol, ref Box b)
    {
        AddOptimal(c, c.FirstParameter(), c.LastParameter(), tol, ref b);
    }

    /// <summary>Adds the precise bounding box of the arc [U1, U2] of C to B.</summary>
    public static void AddOptimal(CurveAdaptorBase c, double u1, double u2, double tol, ref Box b)
    {
        new CurveBounds(c).AddOptimal(u1, u2, tol, ref b);
    }
}
