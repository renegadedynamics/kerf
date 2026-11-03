// Port of: src/ModelingData/TKBRep/BRepLProp/BRepLProp_SurfaceTool.hxx
// Port of: src/ModelingData/TKBRep/BRepLProp/BRepLProp_SurfaceTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>Access to a <see cref="BRepAdaptor_Surface"/> for the local-property algorithms.</summary>
public static class BRepLProp_SurfaceTool
{
    /// <summary>The point of parameters (U, V).</summary>
    public static void Value(BRepAdaptor_Surface S, double U, double V, out Point3D P)
    {
        P = S.Value(U, V);
    }

    /// <summary>The point and first derivatives at (U, V).</summary>
    public static void D1(BRepAdaptor_Surface S, double U, double V, out Point3D P, out Vector3D D1U, out Vector3D D1V)
    {
        S.D1(U, V, out P, out D1U, out D1V);
    }

    /// <summary>The point, first and second derivatives at (U, V).</summary>
    public static void D2(BRepAdaptor_Surface S,
                          double U,
                          double V,
                          out Point3D P,
                          out Vector3D D1U,
                          out Vector3D D1V,
                          out Vector3D D2U,
                          out Vector3D D2V,
                          out Vector3D DUV)
    {
        S.D2(U, V, out P, out D1U, out D1V, out D2U, out D2V, out DUV);
    }

    /// <summary>The derivative of orders IU in U and IV in V.</summary>
    public static Vector3D DN(BRepAdaptor_Surface S, double U, double V, int IU, int IV) => S.DN(U, V, IU, IV);

    /// <summary>The order of continuity of the surface: 1 if only first derivatives are computable, and so on.</summary>
    public static int Continuity(BRepAdaptor_Surface S)
    {
        Continuity aU = S.UContinuity(), aV = S.VContinuity();
        Continuity s = aV < aU ? aV : aU; // std::min
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
    public static void Bounds(BRepAdaptor_Surface S, out double U1, out double V1, out double U2, out double V2)
    {
        U1 = S.FirstUParameter();
        V1 = S.FirstVParameter();
        U2 = S.LastUParameter();
        V2 = S.LastVParameter();
    }
}
