// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_CurveTool.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_CurveTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Constraints;

/// <summary>The curve services the Geom2dGcc solvers use, over Geom2dAdaptor_Curve.</summary>
internal static class ConstraintCurveTool
{
    public static double EpsX(CurveAdaptor2D c, double tol)
    {
        return c.Resolution(tol);
    }

    public static int NbSamples(CurveAdaptor2D c)
    {
        return 20;
    }

    public static Point2D Value(CurveAdaptor2D c, double u)
    {
        return c.Value(u);
    }

    public static double FirstParameter(CurveAdaptor2D c)
    {
        return c.FirstParameter();
    }

    public static double LastParameter(CurveAdaptor2D c)
    {
        return c.LastParameter();
    }

    public static void D1(CurveAdaptor2D c, double u, out Point2D p, out Vector2D t)
    {
        c.D1(u, out p, out t);
    }

    public static void D2(CurveAdaptor2D c, double u, out Point2D p, out Vector2D t, out Vector2D n)
    {
        c.D2(u, out p, out t, out n);
    }

    public static void D3(CurveAdaptor2D c, double u, out Point2D p, out Vector2D t, out Vector2D n, out Vector2D dN)
    {
        c.D3(u, out p, out t, out n, out dN);
    }
}
