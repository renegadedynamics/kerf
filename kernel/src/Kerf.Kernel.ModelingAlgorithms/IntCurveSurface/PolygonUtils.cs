// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_PolygonUtils.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;

namespace Kerf.Engine.Intersection.CurveSurface;

/// <summary>
/// The discretization of a curve into the polygons of the curve/surface intersections (OCCT's namespace
/// IntCurveSurface_PolygonUtils, the former IntCurveSurface_Polygon.gxx).
/// </summary>
internal static class PolygonUtils
{
    /// <summary>
    /// Samples <paramref name="nbPntIn"/> points of the curve at equal parameter steps from
    /// <paramref name="binf"/> to <paramref name="bsup"/>, adds them to the box, and estimates the maximum
    /// deflection (the box is enlarged by 1.5 times it).
    /// </summary>
    public static void InitUniform<CurveType, CurveTool>(in CurveType curve, double binf, double bsup,
                                                         int nbPntIn, NCollection_Array1<Point3D> pnts,
                                                         ref Box bnd, out double deflection)
        where CurveTool : IIntersectionCurveTool<CurveType>
    {
        double du = (bsup - binf) / (double)(nbPntIn - 1);
        double u = binf;
        Point3D p;

        for (int i = 1; i <= nbPntIn; ++i)
        {
            CurveTool.D0(curve, u, out p);
            bnd.Add(p);
            pnts.SetValue(i, p);
            u += du;
        }

        // Calculate deflection estimate by measuring distance from midpoints to chord lines
        deflection = 0.0;
        if (nbPntIn > 3)
        {
            u = binf + du * 0.5;
            for (int i = 1; i < nbPntIn; ++i)
            {
                Point3D pm = CurveTool.Value(curve, u);
                Point3D p1 = pnts.Value(i);
                Point3D p2 = pnts.Value(i + 1);
                // Kerf: a chord of no length is measured from its point, as OCCT does since 71658f6c8f (decision 34004);
                // 8.0.1 made a direction of it and threw
                Vector3D chord = new(p1, p2);
                double t = chord.Magnitude <= global::Kerf.Engine.Geometry.Constants.Resolution()
                    ? p1.DistanceTo(pm)
                    : new Line3D(p1, new Direction3D(chord)).DistanceTo(pm);
                if (t > deflection)
                {
                    deflection = t;
                }
                u += du;
            }
            bnd.Enlarge(1.5 * deflection);
        }
        else
        {
            bnd.Enlarge(1e-10);
        }
    }

    /// <summary>
    /// Samples the curve at the parameters <paramref name="upars"/>, adds the points to the box, keeps a copy of
    /// the parameters in <paramref name="theParams"/>, and estimates the maximum deflection.
    /// </summary>
    public static void InitWithParams<CurveType, CurveTool>(in CurveType curve, NCollection_Array1<double> upars,
                                                            int nbPntIn, NCollection_Array1<Point3D> pnts,
                                                            ref Box bnd, out double deflection,
                                                            out NCollection_HArray1<double> theParams)
        where CurveTool : IIntersectionCurveTool<CurveType>
    {
        theParams = new NCollection_HArray1<double>(1, upars.Length());
        int i0 = upars.Lower() - 1;

        for (int i = 1; i <= nbPntIn; ++i)
        {
            theParams.SetValue(i, upars[i + i0]);
            CurveTool.D0(curve, upars[i + i0], out Point3D p);
            bnd.Add(p);
            pnts.SetValue(i, p);
        }

        // Calculate deflection estimate
        deflection = 0.0;
        if (nbPntIn > 3)
        {
            for (int i = 1; i < nbPntIn; ++i)
            {
                double u = 0.5 * (upars[i0 + i] + upars[i0 + i + 1]);
                Point3D pm = CurveTool.Value(curve, u);
                Point3D p1 = pnts.Value(i);
                Point3D p2 = pnts.Value(i + 1);
                // Kerf: a chord of no length is measured from its point, as OCCT does since 71658f6c8f (decision 34004);
                // 8.0.1 made a direction of it and threw
                Vector3D chord = new(p1, p2);
                double t = chord.Magnitude <= global::Kerf.Engine.Geometry.Constants.Resolution()
                    ? p1.DistanceTo(pm)
                    : new Line3D(p1, new Direction3D(chord)).DistanceTo(pm);
                if (t > deflection)
                {
                    deflection = t;
                }
            }
            bnd.Enlarge(1.5 * deflection);
        }
        else
        {
            bnd.Enlarge(1e-10);
        }
    }

    /// <summary>
    /// The approximate parameter on the curve of the point at <paramref name="paramOnLine"/> (0 to 1) along the
    /// segment <paramref name="index"/> of the polygon.
    /// </summary>
    public static double ApproxParamOnCurve(int index, double paramOnLine, double binf, double bsup,
                                            int nbPntIn, NCollection_HArray1<double>? theParams)
    {
        if (paramOnLine < 0.0 || paramOnLine > 1.0)
        {
            return binf + (paramOnLine * (bsup - binf)) / (double)(nbPntIn - 1);
        }

        int Index = index;
        double ParamOnLine = paramOnLine;

        if ((Index == nbPntIn) && (ParamOnLine == 0.0))
        {
            Index--;
            ParamOnLine = 1.0;
        }

        double du, u;
        if (theParams is null)
        {
            du = (bsup - binf) / (double)(nbPntIn - 1);
            u = binf + du * (double)(Index - 1);
        }
        else
        {
            du = theParams.Value(Index + 1) - theParams.Value(Index);
            u = theParams.Value(Index);
        }

        u += du * ParamOnLine;
        return u;
    }
}
