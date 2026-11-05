// Port of: src/ModelingData/TKBRep/BRepLProp/BRepLProp_SurfaceTool.hxx
// Port of: src/ModelingData/TKBRep/BRepLProp/BRepLProp_SurfaceTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Topology.Properties;

/// <summary>Access to a <see cref="FaceSurfaceAdaptor"/> for the local-property algorithms.</summary>
internal static class SurfaceTool
{
    /// <summary>The point of parameters (U, V).</summary>
    public static void Value(FaceSurfaceAdaptor s, double u, double v, out Point3D p)
    {
        p = s.Value(u, v);
    }

    /// <summary>The point and first derivatives at (U, V).</summary>
    public static void D1(FaceSurfaceAdaptor s, double u, double v, out Point3D p, out Vector3D d1U, out Vector3D d1V)
    {
        s.D1(u, v, out p, out d1U, out d1V);
    }

    /// <summary>The point, first and second derivatives at (U, V).</summary>
    public static void D2(FaceSurfaceAdaptor s,
                          double u,
                          double v,
                          out Point3D p,
                          out Vector3D d1U,
                          out Vector3D d1V,
                          out Vector3D d2U,
                          out Vector3D d2V,
                          out Vector3D duv)
    {
        s.D2(u, v, out p, out d1U, out d1V, out d2U, out d2V, out duv);
    }

    /// <summary>The derivative of orders IU in U and IV in V.</summary>
    public static Vector3D DN(FaceSurfaceAdaptor s, double u, double v, int iu, int iv) => s.DN(u, v, iu, iv);

    /// <summary>The order of continuity of the surface: 1 if only first derivatives are computable, and so on.</summary>
    public static int Continuity(FaceSurfaceAdaptor S)
    {
        Continuity u = S.UContinuity(), v = S.VContinuity();
        Continuity s = v < u ? v : u; // std::min
        switch (s)
        {
            case C0:
                return 0;
            case C1:
                return 1;
            case C2:
                return 2;
            case C3:
                return 3;
            case G1:
                return 0;
            case G2:
                return 0;
            case CN:
                return 3;
        }
        return 0;
    }

    /// <summary>The parameter bounds of the surface.</summary>
    public static void Bounds(FaceSurfaceAdaptor s, out double u1, out double v1, out double u2, out double v2)
    {
        u1 = s.FirstUParameter();
        v1 = s.FirstVParameter();
        u2 = s.LastUParameter();
        v2 = s.LastVParameter();
    }
}
