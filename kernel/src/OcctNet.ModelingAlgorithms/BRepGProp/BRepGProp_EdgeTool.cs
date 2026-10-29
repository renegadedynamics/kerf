// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_EdgeTool.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_EdgeTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;

namespace Occt;

/// <summary>
/// The curve tool of <see cref="BRepGProp_Cinert"/>: the methods GProp's CurveTool template requires, on the curve of an
/// edge.
/// </summary>
/// <remarks>
/// OCCT's methods take a BRepAdaptor_Curve. They take its base class, <see cref="GeomAdaptor_TransformedCurve"/>, until
/// unit 3B ports BRepAdaptor_Curve (TODO(3B)); a BRepAdaptor_Curve will then be accepted unchanged.
/// </remarks>
public static class BRepGProp_EdgeTool
{
    /// <summary>The parameter of the start point of the curve.</summary>
    public static double FirstParameter(GeomAdaptor_TransformedCurve C) => C.FirstParameter();

    /// <summary>The parameter of the end point of the curve.</summary>
    public static double LastParameter(GeomAdaptor_TransformedCurve C) => C.LastParameter();

    /// <summary>
    /// The number of Gauss points for an accurate integration: for a polynomial curve of degree n, 2n - 1.
    /// </summary>
    public static int IntegrationOrder(GeomAdaptor_TransformedCurve BAC)
    {
        switch (BAC.GetType())
        {
            case GeomAbs_Line:
                return 2;

            case GeomAbs_Parabola:
                return 5;

            case GeomAbs_BezierCurve:
            case GeomAbs_BSplineCurve:
            {
                // Use the adaptor's own NbPoles(), which handles the curve-on-surface case;
                // Curve() is null there and down-casting it segfaults.
                int n = 2 * BAC.NbPoles() - 1;
                return n;
            }

            default:
                return 10;
        }
    }

    /// <summary>The point of parameter U.</summary>
    public static gp_Pnt Value(GeomAdaptor_TransformedCurve C, double U) => C.Value(U);

    /// <summary>The point of parameter U and the first derivative there.</summary>
    public static void D1(GeomAdaptor_TransformedCurve C, double U, out gp_Pnt P, out gp_Vec V1) => C.D1(U, out P, out V1);

    /// <summary>The number of intervals of continuity <paramref name="S"/>.</summary>
    public static int NbIntervals(GeomAdaptor_TransformedCurve C, GeomAbs_Shape S) => C.NbIntervals(S);

    /// <summary>The parameters bounding the intervals of continuity <paramref name="S"/> (T.Length() > NbIntervals).</summary>
    public static void Intervals(GeomAdaptor_TransformedCurve C, NCollection_Array1<double> T, GeomAbs_Shape S) => C.Intervals(T, S);
}
