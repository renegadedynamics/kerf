// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_PolygonUtils.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The discretization of a curve into the polygons of the curve/surface intersections (OCCT's namespace
/// IntCurveSurface_PolygonUtils, the former IntCurveSurface_Polygon.gxx).
/// </summary>
internal static class IntCurveSurface_PolygonUtils
{
    /// <summary>
    /// Samples <paramref name="theNbPntIn"/> points of the curve at equal parameter steps from
    /// <paramref name="theBinf"/> to <paramref name="theBsup"/>, adds them to the box, and estimates the maximum
    /// deflection (the box is enlarged by 1.5 times it).
    /// </summary>
    public static void InitUniform<CurveType, CurveTool>(in CurveType theCurve, double theBinf, double theBsup,
                                                         int theNbPntIn, NCollection_Array1<Point3D> thePnts,
                                                         ref Box theBnd, out double theDeflection)
        where CurveTool : IIntCurveSurface_CurveTool<CurveType>
    {
        double du = (theBsup - theBinf) / (double)(theNbPntIn - 1);
        double u = theBinf;
        Point3D P;

        for (int i = 1; i <= theNbPntIn; ++i)
        {
            CurveTool.D0(theCurve, u, out P);
            theBnd.Add(P);
            thePnts.SetValue(i, P);
            u += du;
        }

        // Calculate deflection estimate by measuring distance from midpoints to chord lines
        theDeflection = 0.0;
        if (theNbPntIn > 3)
        {
            u = theBinf + du * 0.5;
            for (int i = 1; i < theNbPntIn; ++i)
            {
                Point3D Pm = CurveTool.Value(theCurve, u);
                Point3D P1 = thePnts.Value(i);
                Point3D P2 = thePnts.Value(i + 1);
                Line3D L = new(P1, new Direction3D(new Vector3D(P1, P2)));
                double t = L.Distance(Pm);
                if (t > theDeflection)
                {
                    theDeflection = t;
                }
                u += du;
            }
            theBnd.Enlarge(1.5 * theDeflection);
        }
        else
        {
            theBnd.Enlarge(1e-10);
        }
    }

    /// <summary>
    /// Samples the curve at the parameters <paramref name="theUpars"/>, adds the points to the box, keeps a copy of
    /// the parameters in <paramref name="theParams"/>, and estimates the maximum deflection.
    /// </summary>
    public static void InitWithParams<CurveType, CurveTool>(in CurveType theCurve, NCollection_Array1<double> theUpars,
                                                            int theNbPntIn, NCollection_Array1<Point3D> thePnts,
                                                            ref Box theBnd, out double theDeflection,
                                                            out NCollection_HArray1<double> theParams)
        where CurveTool : IIntCurveSurface_CurveTool<CurveType>
    {
        theParams = new NCollection_HArray1<double>(1, theUpars.Length());
        int i0 = theUpars.Lower() - 1;

        for (int i = 1; i <= theNbPntIn; ++i)
        {
            theParams.SetValue(i, theUpars[i + i0]);
            CurveTool.D0(theCurve, theUpars[i + i0], out Point3D P);
            theBnd.Add(P);
            thePnts.SetValue(i, P);
        }

        // Calculate deflection estimate
        theDeflection = 0.0;
        if (theNbPntIn > 3)
        {
            for (int i = 1; i < theNbPntIn; ++i)
            {
                double u = 0.5 * (theUpars[i0 + i] + theUpars[i0 + i + 1]);
                Point3D Pm = CurveTool.Value(theCurve, u);
                Point3D P1 = thePnts.Value(i);
                Point3D P2 = thePnts.Value(i + 1);
                Line3D L = new(P1, new Direction3D(new Vector3D(P1, P2)));
                double t = L.Distance(Pm);
                if (t > theDeflection)
                {
                    theDeflection = t;
                }
            }
            theBnd.Enlarge(1.5 * theDeflection);
        }
        else
        {
            theBnd.Enlarge(1e-10);
        }
    }

    /// <summary>
    /// The approximate parameter on the curve of the point at <paramref name="theParamOnLine"/> (0 to 1) along the
    /// segment <paramref name="theIndex"/> of the polygon.
    /// </summary>
    public static double ApproxParamOnCurve(int theIndex, double theParamOnLine, double theBinf, double theBsup,
                                            int theNbPntIn, NCollection_HArray1<double>? theParams)
    {
        if (theParamOnLine < 0.0 || theParamOnLine > 1.0)
        {
            return theBinf + (theParamOnLine * (theBsup - theBinf)) / (double)(theNbPntIn - 1);
        }

        int Index = theIndex;
        double ParamOnLine = theParamOnLine;

        if ((Index == theNbPntIn) && (ParamOnLine == 0.0))
        {
            Index--;
            ParamOnLine = 1.0;
        }

        double du, u;
        if (theParams is null)
        {
            du = (theBsup - theBinf) / (double)(theNbPntIn - 1);
            u = theBinf + du * (double)(Index - 1);
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
