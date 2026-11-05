// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_EdgeTool.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_EdgeTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Measurement;

/// <summary>
/// The curve tool of <see cref="Cinert"/>: the methods GProp's CurveTool template requires, on the curve of an
/// edge.
/// </summary>
internal static class EdgeTool
{
    /// <summary>The parameter of the start point of the curve.</summary>
    public static double FirstParameter(EdgeCurveAdaptor c) => c.FirstParameter();

    /// <summary>The parameter of the end point of the curve.</summary>
    public static double LastParameter(EdgeCurveAdaptor c) => c.LastParameter();

    /// <summary>
    /// The number of Gauss points for an accurate integration: for a polynomial curve of degree n, 2n - 1.
    /// </summary>
    public static int IntegrationOrder(EdgeCurveAdaptor bac)
    {
        switch (bac.GetType())
        {
            case Line:
                return 2;

            case Parabola:
                return 5;

            case BezierCurve:
            case BSplineCurve:
            {
                // Use the adaptor's own NbPoles(), which handles the curve-on-surface case;
                // Curve() is null there and down-casting it segfaults.
                int n = 2 * bac.PoleCount - 1;
                return n;
            }

            default:
                return 10;
        }
    }

    /// <summary>The point of parameter U.</summary>
    public static Point3D Value(EdgeCurveAdaptor c, double u) => c.Value(u);

    /// <summary>The point of parameter U and the first derivative there.</summary>
    public static void D1(EdgeCurveAdaptor c, double u, out Point3D p, out Vector3D v1) => c.D1(u, out p, out v1);

    /// <summary>The number of intervals of continuity <paramref name="s"/>.</summary>
    public static int NbIntervals(EdgeCurveAdaptor c, Continuity s) => c.NbIntervals(s);

    /// <summary>The parameters bounding the intervals of continuity <paramref name="s"/> (T.Length() > NbIntervals).</summary>
    public static void Intervals(EdgeCurveAdaptor c, NCollection_Array1<double> t, Continuity s) => c.Intervals(t, s);
}
