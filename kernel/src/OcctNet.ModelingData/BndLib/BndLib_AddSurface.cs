// Port of: src/ModelingData/TKGeomBase/BndLib/BndLib_AddSurface.hxx
// Port of: src/ModelingData/TKGeomBase/BndLib/BndLib_AddSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Adds the bounding box of a surface to a box (through <see cref="GeomBndLib_Surface"/>). The box is enlarged by
/// the tolerance; an infinite surface opens it.
/// </summary>
public static class BndLib_AddSurface
{
    /// <summary>Adds the bounding box of the surface S, over its own bounds, enlarged by Tol, to B.</summary>
    public static void Add(Adaptor3d_Surface S, double Tol, ref Bnd_Box B)
    {
        Add(S, S.FirstUParameter(), S.LastUParameter(), S.FirstVParameter(), S.LastVParameter(), Tol, ref B);
    }

    /// <summary>Adds the bounding box of the patch [UMin, UMax] x [VMin, VMax] of S, enlarged by Tol, to B.</summary>
    public static void Add(Adaptor3d_Surface S, double UMin, double UMax, double VMin, double VMax, double Tol, ref Bnd_Box B)
    {
        new GeomBndLib_Surface(S).Add(UMin, UMax, VMin, VMax, Tol, ref B);
    }

    /// <summary>Adds the precise bounding box of the surface S to B (numerical search of the extrema; slower).</summary>
    public static void AddOptimal(Adaptor3d_Surface S, double Tol, ref Bnd_Box B)
    {
        AddOptimal(S, S.FirstUParameter(), S.LastUParameter(), S.FirstVParameter(), S.LastVParameter(), Tol, ref B);
    }

    /// <summary>Adds the precise bounding box of the patch [UMin, UMax] x [VMin, VMax] of S to B.</summary>
    public static void AddOptimal(Adaptor3d_Surface S, double UMin, double UMax, double VMin, double VMax, double Tol, ref Bnd_Box B)
    {
        new GeomBndLib_Surface(S).AddOptimal(UMin, UMax, VMin, VMax, Tol, ref B);
    }
}
