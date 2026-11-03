// Port of: src/ModelingAlgorithms/TKGeomAlgo/GTests/GeomFill_BSplineCurves_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Occt.GeomFill_FillingStyle;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// OCCT's GeomFill_BSplineCurves GTest (suite GeomFill_BSplineCurvesTest; OCC28131). Its two offset tests
/// (OCC28131_SimpleOffsetOfFilledFace, OCC28131_StandardOffsetOfFilledFace) need BRepOffset (phase 7) and are not
/// ported yet.
/// </summary>
public class GeomFill_BSplineCurves_Test
{
    // Builds the filled face from 3 boundary BSpline/Bezier curves (common setup for all OCC28131 tests).
    internal static TopoDS_Shape createOCC28131Face()
    {
        const double aHeight = 8.5;
        Point3D aV0 = new(-17.6, 0.0, 0.0);
        Point3D aV1 = new(0, 32.8, 0.0);

        // Outline Bezier curve
        NCollection_Array1<Point3D> aBezierPoles = new(1, 4);
        aBezierPoles[1] = aV0;
        aBezierPoles[4] = aV1;
        aBezierPoles[2] = new Point3D(aV0.X(), (5.4 / 13.2) * aV1.Y(), 0);
        aBezierPoles[3] = new Point3D((6.0 / 6.8) * aV0.X(), aV1.Y(), 0);

        Geom_BezierCurve aBezier = new(aBezierPoles);
        Geom_BSplineCurve anOutlineCurve = GeomConvert.CurveToBSplineCurve(aBezier);

        // First side curve from 2D interpolation projected on YZ plane
        Geom_BSplineCurve aCurve1;
        {
            NCollection_HArray1<Point2D> aHArray = new(1, 2);
            aHArray.SetValue(1, new Point2D(-aV1.Y(), 0));
            aHArray.SetValue(2, new Point2D(0, aHeight + aHeight / 2));
            Geom2dAPI_Interpolate anInterp = new(aHArray, false, 1e-6);
            anInterp.Load(new Vector2D(0, 1), new Vector2D(1, 0));
            anInterp.Perform();
            Plane3D aPln = new(new CoordinateSystem(new Point3D(), new Direction3D(Direction3D.D.X), new Direction3D(Direction3D.D.NY)));
            aCurve1 = (Geom_BSplineCurve)GeomAPI.To3d(anInterp.Curve(), aPln);
        }

        // Second side curve from 2D interpolation projected on XZ plane
        Geom_BSplineCurve aCurve2;
        {
            NCollection_HArray1<Point2D> aHArray = new(1, 3);
            aHArray.SetValue(1, new Point2D(-aV0.X(), 0));
            aHArray.SetValue(2, new Point2D(-aV0.X() - 2.6, aHeight));
            aHArray.SetValue(3, new Point2D(0, aHeight + aHeight / 2));
            Geom2dAPI_Interpolate anInterp = new(aHArray, false, 1e-6);
            anInterp.Perform();
            Plane3D aPln = new(new CoordinateSystem(new Point3D(), new Direction3D(Direction3D.D.NY), new Direction3D(Direction3D.D.NX)));
            aCurve2 = (Geom_BSplineCurve)GeomAPI.To3d(anInterp.Curve(), aPln);
        }

        GeomFill_BSplineCurves aFill = new();
        aFill.Init(anOutlineCurve, aCurve1, aCurve2, GeomFill_CoonsStyle);

        BRepBuilderAPI_MakeFace aFaceBuilder = new(aFill.Surface()!, 0);
        return aFaceBuilder.IsDone() ? aFaceBuilder.Shape() : new TopoDS_Shape();
    }

    [Fact]
    public void OCC28131_FillSurfaceFromBezierAndInterpolatedCurves()
    {
        TopoDS_Shape aFace = createOCC28131Face();
        Assert.False(aFace.IsNull());
        Assert.True(new BRepCheck_Analyzer(aFace).IsValid());
    }
}
