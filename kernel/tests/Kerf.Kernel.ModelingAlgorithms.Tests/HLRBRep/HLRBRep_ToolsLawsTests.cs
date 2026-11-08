// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.HiddenLines;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// The hidden-line algorithm's surface and curve tools by what they answer (unit 12Q, decision 1753): each kind of face
/// and edge gives back its own geometry, its bounds, periods and degrees, and as many samples as its kind asks for.
/// </summary>
public class HLRBRep_ToolsLawsTests
{
    private static FaceSurfaceAdaptor FaceOf(Shape theSolid, SurfaceKind theKind)
    {
        foreach (Shape aF in BRepOffset_MakeOffsetTests.SubShapes(theSolid, ShapeType.Face))
        {
            var anAdaptor = new FaceSurfaceAdaptor(ShapeCasts.Face(aF));
            if (anAdaptor.GetType() == theKind)
            {
                return anAdaptor;
            }
        }
        throw new InvalidOperationException(theKind.ToString());
    }

    private static EdgeCurveAdaptor EdgeOf(Shape theShape, CurveKind theKind)
    {
        foreach (Shape anE in BRepOffset_MakeOffsetTests.SubShapes(theShape, ShapeType.Edge))
        {
            var anAdaptor = new EdgeCurveAdaptor(ShapeCasts.Edge(anE));
            if (anAdaptor.GetType() == theKind && !global::Kerf.Engine.Topology.Brep.Tool.Degenerated(ShapeCasts.Edge(anE)))
            {
                return anAdaptor;
            }
        }
        throw new InvalidOperationException(theKind.ToString());
    }

    [Fact]
    public void Each_elementary_face_gives_back_its_surface()
    {
        FaceSurfaceAdaptor aPlane = FaceOf(new MakeBox(1, 2, 3).Shape(), SurfaceKind.Plane);
        Assert.Equal(SurfaceKind.Plane, BSurfaceTool.GetType(aPlane));
        Assert.True(BSurfaceTool.Plane(aPlane).Axis.Direction.IsParallel(Direction3D.UnitX, 1e-12)
                    || BSurfaceTool.Plane(aPlane).Axis.Direction.IsParallel(Direction3D.UnitY, 1e-12)
                    || BSurfaceTool.Plane(aPlane).Axis.Direction.IsParallel(Direction3D.UnitZ, 1e-12));
        Assert.Equal(2, BSurfaceTool.NbSamplesU(aPlane));
        Assert.Equal(2, BSurfaceTool.NbSamplesV(aPlane));

        FaceSurfaceAdaptor aCylinder = FaceOf(new ApiMakeCylinder(2, 10).Shape(), SurfaceKind.Cylinder);
        Assert.Equal(2, BSurfaceTool.Cylinder(aCylinder).Radius);
        Assert.True(BSurfaceTool.IsUPeriodic(aCylinder) && BSurfaceTool.IsUClosed(aCylinder));
        Assert.Equal(2 * Math.PI, BSurfaceTool.UPeriod(aCylinder), 12);
        Assert.False(BSurfaceTool.IsVPeriodic(aCylinder) || BSurfaceTool.IsVClosed(aCylinder));
        Assert.Equal(10, BSurfaceTool.LastVParameter(aCylinder) - BSurfaceTool.FirstVParameter(aCylinder), 12);
        Assert.Equal(2 * Math.PI, BSurfaceTool.LastUParameter(aCylinder) - BSurfaceTool.FirstUParameter(aCylinder), 12);
        Assert.Equal(10, BSurfaceTool.NbSamplesU(aCylinder));

        FaceSurfaceAdaptor aCone = FaceOf(new ApiMakeCone(5, 2, 10).Shape(), SurfaceKind.Cone);
        Assert.Equal(Math.Atan(0.3), Math.Abs(BSurfaceTool.Cone(aCone).SemiAngle), 12);
        FaceSurfaceAdaptor aSphere = FaceOf(new MakeSphere(4).Shape(), SurfaceKind.Sphere);
        Assert.Equal(4, BSurfaceTool.Sphere(aSphere).Radius);
        Assert.True(BSurfaceTool.IsVPeriodic(aSphere) == false);
        FaceSurfaceAdaptor aTorus = FaceOf(new MakeTorus(6, 2).Shape(), SurfaceKind.Torus);
        Assert.Equal((6.0, 2.0), (BSurfaceTool.Torus(aTorus).MajorRadius, BSurfaceTool.Torus(aTorus).MinorRadius));
        Assert.True(BSurfaceTool.IsVPeriodic(aTorus));
        Assert.Equal(2 * Math.PI, BSurfaceTool.VPeriod(aTorus), 12);
        // a torus 20 times round its axis, a revolved surface 15 times along its other parameter
        Assert.Equal(20, BSurfaceTool.NbSamplesU(aTorus));
        Assert.Equal(15, BSurfaceTool.NbSamplesV(aTorus));
        Assert.Equal(15, BSurfaceTool.NbSamplesV(aCylinder));
        // over part of its range, a surface sampled more than 10 times is sampled 5 times (OCCT's ratio is negative)
        Assert.Equal(5, BSurfaceTool.NbSamplesU(aTorus, 0, 1));
        Assert.Equal(5, BSurfaceTool.NbSamplesV(aTorus, 0, 1));
        Assert.Equal(10, BSurfaceTool.NbSamplesU(aCylinder, 0, 1));
    }

    [Fact]
    public void A_face_evaluates_as_its_surface()
    {
        FaceSurfaceAdaptor aCylinder = FaceOf(new ApiMakeCylinder(2, 10).Shape(), SurfaceKind.Cylinder);
        Point3D aP = BSurfaceTool.Value(aCylinder, 1.0, 3.0);
        Assert.True(aP.DistanceTo(new Point3D(2 * Math.Cos(1.0), 2 * Math.Sin(1.0), 3.0)) < 1e-12);
        BSurfaceTool.D0(aCylinder, 1.0, 3.0, out Point3D aP0);
        Assert.Equal(aP, aP0);
        BSurfaceTool.D1(aCylinder, 1.0, 3.0, out _, out Vector3D aDu, out Vector3D aDv);
        Assert.Equal(2, aDu.Magnitude, 12);
        Assert.True(aDv.IsParallel(Vector3D.UnitZ, 1e-12));
        BSurfaceTool.D2(aCylinder, 1.0, 3.0, out _, out _, out _, out Vector3D aD2u, out Vector3D aD2v, out Vector3D aDuv);
        Assert.Equal(2, aD2u.Magnitude, 12);
        Assert.Equal(0, aD2v.Magnitude, 12);
        Assert.Equal(0, aDuv.Magnitude, 12);
        BSurfaceTool.D3(aCylinder, 1.0, 3.0, out _, out _, out _, out _, out _, out _, out Vector3D aD3u, out _, out _, out _);
        Assert.Equal(2, aD3u.Magnitude, 12);
        Assert.Equal(aD2u, BSurfaceTool.DN(aCylinder, 1.0, 3.0, 2, 0));
        // the parameter a chord of 1 spans on the radius 2
        Assert.Equal(2 * Math.Asin(0.25), BSurfaceTool.UResolution(aCylinder, 1.0), 12);
        Assert.Equal(1.0, BSurfaceTool.VResolution(aCylinder, 1.0), 12);
        Assert.Equal(Continuity.CN, BSurfaceTool.UContinuity(aCylinder));
        Assert.Equal(Continuity.CN, BSurfaceTool.VContinuity(aCylinder));
        Assert.Equal(1, BSurfaceTool.NbUIntervals(aCylinder, Continuity.C2));
        Assert.Equal(1, BSurfaceTool.NbVIntervals(aCylinder, Continuity.C2));
        var aTab = new NCollection_Array1<double>(1, 2);
        BSurfaceTool.VIntervals(aCylinder, aTab, Continuity.C2);
        Assert.Equal(10, aTab[2] - aTab[1], 12);
        BSurfaceTool.UIntervals(aCylinder, aTab, Continuity.C2);
        Assert.Equal(2 * Math.PI, aTab[2] - aTab[1], 12);
        // trimmed, a face's surface keeps its kind and takes the new bounds
        SurfaceAdaptorBase aTrimmed = BSurfaceTool.VTrim(aCylinder, 1, 2, 1e-9);
        Assert.Equal(1, aTrimmed.LastVParameter() - aTrimmed.FirstVParameter(), 12);
        Assert.Equal(1, BSurfaceTool.UTrim(aCylinder, 0, 1, 1e-9).LastUParameter(), 12);
    }

    [Fact]
    public void A_spline_face_gives_back_its_degrees_and_poles_and_is_sampled_by_them()
    {
        // a box face converted to a B-spline of degree 1 by 1, with 2 by 2 poles
        Shape aBox = new MakeBox(1, 2, 3).Shape();
        Face aFace = ShapeCasts.Face(BRepOffset_MakeOffsetTests.SubShapes(aBox, ShapeType.Face)[0]);
        Shape aSpline = new global::Kerf.Engine.Construction.Api.NurbsConvert(aFace).Shape();
        var anAdaptor = new FaceSurfaceAdaptor(ShapeCasts.Face(BRepOffset_MakeOffsetTests.SubShapes(aSpline, ShapeType.Face)[0]));
        Assert.Equal(SurfaceKind.BSplineSurface, BSurfaceTool.GetType(anAdaptor));
        Assert.Equal((1, 1), (BSurfaceTool.UDegree(anAdaptor), BSurfaceTool.VDegree(anAdaptor)));
        Assert.Equal((2, 2), (BSurfaceTool.NbUPoles(anAdaptor), BSurfaceTool.NbVPoles(anAdaptor)));
        Assert.Equal((2, 2), (BSurfaceTool.NbUKnots(anAdaptor), BSurfaceTool.NbVKnots(anAdaptor)));
        Assert.False(BSurfaceTool.IsURational(anAdaptor) || BSurfaceTool.IsVRational(anAdaptor));
        Assert.Equal(BSurfaceTool.UDegree(anAdaptor), BSurfaceTool.BSpline(anAdaptor).UDegree);
        // knots times degree, and at least 2
        Assert.Equal(2, BSurfaceTool.NbSamplesU(anAdaptor));
        Assert.Equal(2, BSurfaceTool.NbSamplesV(anAdaptor));
    }

    [Fact]
    public void Each_kind_of_edge_gives_back_its_curve_and_samples()
    {
        EdgeCurveAdaptor aLine = EdgeOf(new MakeBox(1, 2, 3).Shape(), CurveKind.Line);
        Assert.Equal(CurveKind.Line, BCurveTool.GetType(aLine));
        Assert.Equal(2, BCurveTool.NbSamples(aLine, BCurveTool.FirstParameter(aLine), BCurveTool.LastParameter(aLine)));
        Assert.False(BCurveTool.IsClosed(aLine) || BCurveTool.IsPeriodic(aLine));
        Assert.Equal(1, BCurveTool.NbIntervals(aLine, Continuity.C2));
        Assert.Equal(Continuity.CN, BCurveTool.Continuity(aLine));
        Assert.True(BCurveTool.Line(aLine).Direction.IsParallel(Direction3D.UnitX, 1e-12)
                    || BCurveTool.Line(aLine).Direction.IsParallel(Direction3D.UnitY, 1e-12)
                    || BCurveTool.Line(aLine).Direction.IsParallel(Direction3D.UnitZ, 1e-12));

        EdgeCurveAdaptor aCircle = EdgeOf(new ApiMakeCylinder(2, 10).Shape(), CurveKind.Circle);
        Assert.Equal(2, BCurveTool.Circle(aCircle).Radius);
        Assert.True(BCurveTool.IsPeriodic(aCircle) && BCurveTool.IsClosed(aCircle));
        Assert.Equal(2 * Math.PI, BCurveTool.Period(aCircle), 12);
        Assert.Equal(10, BCurveTool.NbSamples(aCircle, 0, 1));
        Assert.Equal(2 * Math.Asin(0.25), BCurveTool.Resolution(aCircle, 1.0), 12);
        BCurveTool.D1(aCircle, 0, out Point3D aP, out Vector3D aT);
        Assert.Equal(2, aT.Magnitude, 12);
        BCurveTool.D2(aCircle, 0, out _, out _, out Vector3D aN);
        Assert.Equal(2, aN.Magnitude, 12);
        BCurveTool.D3(aCircle, 0, out _, out _, out _, out Vector3D aD3);
        Assert.Equal(2, aD3.Magnitude, 12);
        Assert.Equal(aN, BCurveTool.DN(aCircle, 0, 2));
        BCurveTool.D0(aCircle, 0, out Point3D aP0);
        Assert.Equal(aP, aP0);
        Assert.Equal(aP, BCurveTool.Value(aCircle, 0));

        Edge anEllipseEdge = new MakeEdge(new Ellipse3D(new Frame3D(Point3D.Origin, Direction3D.UnitZ), 5, 3)).Edge();
        EdgeCurveAdaptor anEllipse = new(anEllipseEdge);
        Assert.Equal((5.0, 3.0), (BCurveTool.Ellipse(anEllipse).MajorRadius, BCurveTool.Ellipse(anEllipse).MinorRadius));
        Edge aParabolaEdge = new MakeEdge(new Parabola3D(new Frame3D(Point3D.Origin, Direction3D.UnitZ), 2), -1, 1).Edge();
        Assert.Equal(2, BCurveTool.Parabola(new EdgeCurveAdaptor(aParabolaEdge)).Focal);
        Edge aHyperbolaEdge = new MakeEdge(new Hyperbola3D(new Frame3D(Point3D.Origin, Direction3D.UnitZ), 3, 1), -1, 1).Edge();
        Assert.Equal(3, BCurveTool.Hyperbola(new EdgeCurveAdaptor(aHyperbolaEdge)).MajorRadius);
    }

    [Fact]
    public void A_spline_edge_gives_back_its_poles_and_weights()
    {
        var aPoles = new NCollection_Array1<Point3D>(1, 3);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(1, 2, 0);
        aPoles[3] = new Point3D(3, 0, 1);
        var aBezier = new BezierCurve3D(aPoles);
        foreach (Curve3D aCurve in new Curve3D[] { aBezier, global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(aBezier) })
        {
            var anAdaptor = new EdgeCurveAdaptor(new MakeEdge(aCurve).Edge());
            Assert.Equal(2, BCurveTool.Degree(anAdaptor));
            Assert.Equal(3, BCurveTool.NbPoles(anAdaptor));
            Assert.False(BCurveTool.IsRational(anAdaptor));
            var aGot = new NCollection_Array1<Point3D>(1, 3);
            BCurveTool.Poles(anAdaptor, aGot);
            var aGotToo = new NCollection_Array1<Point3D>(1, 3);
            var aWeights = new NCollection_Array1<double>(1, 3);
            BCurveTool.PolesAndWeights(anAdaptor, aGotToo, aWeights);
            for (int i = 1; i <= 3; i++)
            {
                Assert.Equal(aPoles[i], aGot[i]);
                Assert.Equal(aPoles[i], aGotToo[i]);
                Assert.Equal(1, aWeights[i]);
            }
        }
        var aBezierAdaptor = new EdgeCurveAdaptor(new MakeEdge(aBezier).Edge());
        Assert.Equal(6, BCurveTool.NbSamples(aBezierAdaptor, 0, 1));
        Assert.Equal(3, BCurveTool.Bezier(aBezierAdaptor).PoleCount);
        var aSplineAdaptor = new EdgeCurveAdaptor(new MakeEdge(global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(aBezier)).Edge());
        Assert.Equal(2, BCurveTool.NbKnots(aSplineAdaptor));
        Assert.Equal(3, BCurveTool.BSpline(aSplineAdaptor).PoleCount);
        // knots times degree over the part asked: 4 on the whole, 8 on half
        Assert.Equal(4, BCurveTool.NbSamples(aSplineAdaptor, 0, 1));
        Assert.Equal(8, BCurveTool.NbSamples(aSplineAdaptor, 0, 0.5));
    }
}
