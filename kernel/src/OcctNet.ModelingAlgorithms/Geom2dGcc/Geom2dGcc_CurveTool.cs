// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_CurveTool.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_CurveTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>The curve services the Geom2dGcc solvers use, over Geom2dAdaptor_Curve.</summary>
public static class Geom2dGcc_CurveTool
{
    public static double EpsX(Geom2dAdaptor_Curve C, double Tol)
    {
        return C.Resolution(Tol);
    }

    public static int NbSamples(Geom2dAdaptor_Curve C)
    {
        return 20;
    }

    public static Point2D Value(Geom2dAdaptor_Curve C, double U)
    {
        return C.Value(U);
    }

    public static double FirstParameter(Geom2dAdaptor_Curve C)
    {
        return C.FirstParameter();
    }

    public static double LastParameter(Geom2dAdaptor_Curve C)
    {
        return C.LastParameter();
    }

    public static void D1(Geom2dAdaptor_Curve C, double U, out Point2D P, out Vector2D T)
    {
        C.D1(U, out P, out T);
    }

    public static void D2(Geom2dAdaptor_Curve C, double U, out Point2D P, out Vector2D T, out Vector2D N)
    {
        C.D2(U, out P, out T, out N);
    }

    public static void D3(Geom2dAdaptor_Curve C, double U, out Point2D P, out Vector2D T, out Vector2D N, out Vector2D dN)
    {
        C.D3(U, out P, out T, out N, out dN);
    }
}
