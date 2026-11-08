// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Approximation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// The surface tool of the approximation of a face/face intersection by what it answers (unit 12Q, decision 1753):
/// each kind of face gives back its surface, bounds, periods and derivatives, and is sampled ten times each way.
/// </summary>
public class BRepApprox_SurfaceToolLawsTests
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

    [Fact]
    public void A_cylinder_face_answers_as_its_surface()
    {
        FaceSurfaceAdaptor s = FaceOf(new ApiMakeCylinder(2, 10).Shape(), SurfaceKind.Cylinder);
        Assert.Equal(SurfaceKind.Cylinder, ApproximationSurfaceTool.GetType(s));
        Assert.Equal(2, ApproximationSurfaceTool.Cylinder(s).Radius);
        Assert.Equal(2 * Math.PI, ApproximationSurfaceTool.LastUParameter(s) - ApproximationSurfaceTool.FirstUParameter(s), 12);
        Assert.Equal(10, ApproximationSurfaceTool.LastVParameter(s) - ApproximationSurfaceTool.FirstVParameter(s), 12);
        Assert.True(ApproximationSurfaceTool.IsUClosed(s) && ApproximationSurfaceTool.IsUPeriodic(s));
        Assert.False(ApproximationSurfaceTool.IsVClosed(s) || ApproximationSurfaceTool.IsVPeriodic(s));
        Assert.Equal(2 * Math.PI, ApproximationSurfaceTool.UPeriod(s), 12);
        Assert.Equal(1, ApproximationSurfaceTool.NbUIntervals(s, Continuity.C2));
        Assert.Equal(1, ApproximationSurfaceTool.NbVIntervals(s, Continuity.C2));
        var aTab = new NCollection_Array1<double>(1, 2);
        ApproximationSurfaceTool.UIntervals(s, aTab, Continuity.C2);
        Assert.Equal(2 * Math.PI, aTab[2] - aTab[1], 12);
        ApproximationSurfaceTool.VIntervals(s, aTab, Continuity.C2);
        Assert.Equal(10, aTab[2] - aTab[1], 12);
        Assert.Equal(1, ApproximationSurfaceTool.UTrim(s, 0, 1, 1e-9).LastUParameter(), 12);
        Assert.Equal(2, ApproximationSurfaceTool.VTrim(s, 1, 2, 1e-9).LastVParameter(), 12);

        // on the radius 2: the point, its derivatives, and the parameter a chord of 1 spans
        Point3D aP = ApproximationSurfaceTool.Value(s, 1.0, 3.0);
        Assert.True(aP.DistanceTo(new Point3D(2 * Math.Cos(1.0), 2 * Math.Sin(1.0), 3)) < 1e-12);
        ApproximationSurfaceTool.D0(s, 1.0, 3.0, out Point3D aP0);
        Assert.Equal(aP, aP0);
        ApproximationSurfaceTool.D1(s, 1.0, 3.0, out _, out Vector3D aDu, out Vector3D aDv);
        Assert.Equal(2, aDu.Magnitude, 12);
        Assert.Equal(1, aDv.Magnitude, 12);
        ApproximationSurfaceTool.D2(s, 1.0, 3.0, out _, out _, out _, out Vector3D aD2u, out Vector3D aD2v, out Vector3D aDuv);
        Assert.Equal((2.0, 0.0, 0.0), (Math.Round(aD2u.Magnitude, 12), aD2v.Magnitude, aDuv.Magnitude));
        ApproximationSurfaceTool.D3(s, 1.0, 3.0, out _, out _, out _, out _, out _, out _, out Vector3D aD3u, out Vector3D aD3v, out _, out _);
        Assert.Equal(2, aD3u.Magnitude, 12);
        Assert.Equal(0, aD3v.Magnitude, 12);
        Assert.Equal(aD2u, ApproximationSurfaceTool.DN(s, 1.0, 3.0, 2, 0));
        Assert.Equal(2 * Math.Asin(0.25), ApproximationSurfaceTool.UResolution(s, 1.0), 12);
        Assert.Equal(1, ApproximationSurfaceTool.VResolution(s, 1.0), 12);
        Assert.Equal(10, ApproximationSurfaceTool.NbSamplesU(s));
        Assert.Equal(10, ApproximationSurfaceTool.NbSamplesV(s));
        Assert.Equal(10, ApproximationSurfaceTool.NbSamplesU(s, 0, 1));
        Assert.Equal(10, ApproximationSurfaceTool.NbSamplesV(s, 0, 1));
    }

    [Fact]
    public void Each_kind_of_face_gives_back_its_surface()
    {
        Direction3D aNormal = ApproximationSurfaceTool.Plane(FaceOf(new MakeBox(1, 2, 3).Shape(), SurfaceKind.Plane)).Axis.Direction;
        Assert.True(aNormal.IsParallel(Direction3D.UnitX, 1e-12) || aNormal.IsParallel(Direction3D.UnitY, 1e-12) || aNormal.IsParallel(Direction3D.UnitZ, 1e-12));
        Assert.Equal(Math.Atan(0.3), Math.Abs(ApproximationSurfaceTool.Cone(FaceOf(new ApiMakeCone(5, 2, 10).Shape(), SurfaceKind.Cone)).SemiAngle), 12);
        Assert.Equal(4, ApproximationSurfaceTool.Sphere(FaceOf(new MakeSphere(4).Shape(), SurfaceKind.Sphere)).Radius);
        Torus3D aTorus = ApproximationSurfaceTool.Torus(FaceOf(new MakeTorus(6, 2).Shape(), SurfaceKind.Torus));
        Assert.Equal((6.0, 2.0), (aTorus.MajorRadius, aTorus.MinorRadius));
        FaceSurfaceAdaptor aTorusFace = FaceOf(new MakeTorus(6, 2).Shape(), SurfaceKind.Torus);
        Assert.True(ApproximationSurfaceTool.IsVPeriodic(aTorusFace));
        Assert.Equal(2 * Math.PI, ApproximationSurfaceTool.VPeriod(aTorusFace), 12);

        // a revolved and an extruded face give back their axis, direction and profile: an ellipse off the axis, turned
        // about it, and one pushed along it
        Edge aProfile = new MakeEdge(new Ellipse3D(new Frame3D(new Point3D(8, 0, 0), Direction3D.UnitY), 2, 1)).Edge();
        Shape aRevolved = new ApiMakeRevol(aProfile, new Axis3D(Point3D.Origin, Direction3D.UnitZ)).Shape();
        FaceSurfaceAdaptor aRevolution = FaceOf(aRevolved, SurfaceKind.SurfaceOfRevolution);
        Assert.True(ApproximationSurfaceTool.AxeOfRevolution(aRevolution).Direction.IsParallel(Direction3D.UnitZ, 1e-12));
        Assert.Equal(CurveKind.Ellipse, ApproximationSurfaceTool.BasisCurve(aRevolution).GetType());
        Edge anArc = new MakeEdge(new Ellipse3D(new Frame3D(Point3D.Origin, Direction3D.UnitZ), 5, 3)).Edge();
        Shape anExtruded = new ApiMakePrism(anArc, new Vector3D(0, 0, 4)).Shape();
        FaceSurfaceAdaptor anExtrusion = FaceOf(anExtruded, SurfaceKind.SurfaceOfExtrusion);
        Assert.True(ApproximationSurfaceTool.Direction(anExtrusion).IsParallel(Direction3D.UnitZ, 1e-12));
        Assert.Equal(CurveKind.Ellipse, ApproximationSurfaceTool.BasisCurve(anExtrusion).GetType());
    }

    [Fact]
    public void A_spline_face_gives_back_its_spline()
    {
        Shape aBox = new MakeBox(1, 2, 3).Shape();
        Shape aSpline = new NurbsConvert(ShapeCasts.Face(BRepOffset_MakeOffsetTests.SubShapes(aBox, ShapeType.Face)[0])).Shape();
        var s = new FaceSurfaceAdaptor(ShapeCasts.Face(BRepOffset_MakeOffsetTests.SubShapes(aSpline, ShapeType.Face)[0]));
        Assert.Equal(SurfaceKind.BSplineSurface, ApproximationSurfaceTool.GetType(s));
        Assert.Equal(1, ApproximationSurfaceTool.BSpline(s).UDegree);
    }
}
