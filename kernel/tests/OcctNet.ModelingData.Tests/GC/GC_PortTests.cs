// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Occt.gce_ErrorType;

namespace OcctNet.ModelingData.Tests;

/// <summary>Tests of the GC port beyond OCCT's GTests: failure statuses, freezing, and the gce failure paths.</summary>
public class GC_PortTests
{
    private static readonly Frame3D theZAxes = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));

    [Fact]
    public void FailedConstructions_ReportStatus_AndValueThrows()
    {
        GC_MakeCircle aCircle = new(theZAxes, -1.0);
        Assert.Equal(gce_NegativeRadius, aCircle.Status());
        Assert.True(aCircle.IsError());
        Assert.Throws<NotDoneException>(() => aCircle.Value());
        Assert.Throws<NotDoneException>(() => (Geom_Circle)aCircle);

        Assert.Equal(gce_InvertAxis, new GC_MakeEllipse(theZAxes, 1.0, 2.0).Status());
        Assert.Equal(gce_NegativeRadius, new GC_MakeEllipse(theZAxes, 1.0, -2.0).Status());
        Assert.Equal(gce_NegativeRadius, new GC_MakeHyperbola(theZAxes, -1.0, 2.0).Status());
        Assert.Equal(gce_ConfusedPoints, new GC_MakeLine(new Point3D(1, 2, 3), new Point3D(1, 2, 3)).Status());
        Assert.Equal(gce_BadEquation, new GC_MakePlane(0.0, 0.0, 0.0, 1.0).Status());
        Assert.Equal(gce_NegativeRadius, new GC_MakeCylindricalSurface(theZAxes, -1.0).Status());
        Assert.Equal(gce_NegativeRadius, new GC_MakeConicalSurface(theZAxes, 0.3, -1.0).Status());
        Assert.Equal(gce_BadAngle, new GC_MakeConicalSurface(theZAxes, M_PI / 2.0, 1.0).Status());
        Assert.Equal(gce_NullFocusLength,
                     new GC_MakeParabola2d(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), -1.0, true).Status());
        Assert.Equal(gce_ConfusedPoints,
                     new GC_MakeArcOfCircle(new Point3D(1, 0, 0), new Vector3D(0, 1, 0), new Point3D(1, 0, 0)).Status());
        Assert.False(new GC_MakeTrimmedCylinder(new Point3D(0, 0, 0), new Point3D(0, 0, 0), new Point3D(1, 0, 0)).IsDone());
    }

    [Fact]
    public void Segment_ConfusedPoints_Throws()
    {
        // OCCT's release build dereferences a null line here; the port throws from GC_MakeLine's Value().
        Assert.Throws<NotDoneException>(() => new GC_MakeSegment(new Point3D(1, 2, 3), new Point3D(1, 2, 3)));
    }

    [Fact]
    public void Value_IsFrozen_AndTheSameObject()
    {
        GC_MakeSegment aSegment = new(new Point3D(0, 0, 0), new Point3D(3, 4, 0));
        Geom_TrimmedCurve aCurve = aSegment.Value();
        Assert.True(aCurve.IsFrozen);
        Assert.Same(aCurve, aSegment.Value());
        Assert.Same(aCurve, (Geom_TrimmedCurve)aSegment);
        Assert.Equal(5.0, aCurve.LastParameter(), 1e-15);
        Assert.Throws<ImmutableObjectException>(() => aCurve.SetTrim(0.0, 1.0));
        Assert.False(((Geom_TrimmedCurve)aCurve.Copy()).IsFrozen);

        GC_MakeRotation aRotation = new(new Axis3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 0.5);
        Assert.True(aRotation.Value().IsFrozen);
        Assert.True(new GC_MakeTranslation2d(new Vector2D(1, 2)).Value().IsFrozen);
        Assert.True(new GC_MakePlane(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)).Value().IsFrozen);
    }

    [Fact]
    public void ArcOfCircle2d_FromConfusedPoints_Throws()
    {
        // OCCT converts gce_MakeLin2d(P1, P1) implicitly: its release build goes on with the default line, finds
        // the center at P1 and computes the parameters of P1 on a circle of radius 0, where ElCLib makes a gp_Dir2d
        // of a null vector (NaN parameters in the release build). The port reads the same default line, and throws
        // there instead.
        Point2D aP = new(1.0, 2.0);
        Assert.Throws<ConstructionErrorException>(() => new GC_MakeArcOfCircle2d(aP, new Vector2D(0.0, 1.0), aP));
    }

    [Fact]
    public void ArcOfCircle2d_FromTangent_FollowsTheTangent()
    {
        Point2D aP1 = new(1.0, 0.0);
        Point2D aP2 = new(-1.0, 0.0);
        GC_MakeArcOfCircle2d aMaker = new(aP1, new Vector2D(0.0, 1.0), aP2);
        Assert.True(aMaker.IsDone());
        Geom2d_TrimmedCurve anArc = aMaker.Value();
        Assert.True(anArc.StartPoint().IsEqual(aP1, 1e-12));
        Assert.True(anArc.EndPoint().IsEqual(aP2, 1e-12));
        Assert.True(anArc.Value(0.5 * (anArc.FirstParameter() + anArc.LastParameter())).IsEqual(new Point2D(0, 1), 1e-12));
    }

    [Fact]
    public void Surfaces_AndTrimmedSurfaces()
    {
        Cylinder aCyl = new(new CoordinateSystem(theZAxes), 5.0);
        Assert.Equal(3.0, new GC_MakeCylindricalSurface(aCyl, 2.0).Value().Radius());
        Assert.Equal(2.0, new GC_MakeCylindricalSurface(aCyl, new Point3D(0.0, 2.0, 7.0)).Value().Radius(), 1e-15);

        GC_MakeTrimmedCylinder aTrimmed = new(new Axis3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 2.0, 3.0);
        aTrimmed.Value().Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
        Assert.Equal((0.0, 2.0 * M_PI, 0.0, 3.0), (aU1, aU2, aV1, aV2));

        GC_MakeTrimmedCone aCone = new(new Point3D(0, 0, 0), new Point3D(0, 0, 4), 3.0, 1.0);
        Assert.True(aCone.IsDone());
        aCone.Value().Bounds(out _, out _, out _, out double aConeV2);
        Assert.Equal(Math.Sqrt(4.0 * 4.0 + 2.0 * 2.0), aConeV2, 1e-12);
    }

    [Fact]
    public void Transformations()
    {
        Point3D aP = new(1.0, 2.0, 3.0);
        Point3D aO = new(0.0, 0.0, 0.0);
        Assert.True(aP.Mirrored(aO).IsEqual(aP.Transformed(new GC_MakeMirror(aO).Value().Trsf()), 1e-15));
        Assert.True(new Point3D(3, 6, 9).IsEqual(aP.Transformed(new GC_MakeScale(aO, 3.0).Value().Trsf()), 1e-15));
        Assert.True(new Point3D(2, 4, 6).IsEqual(aP.Transformed(new GC_MakeTranslation(aO, aP).Value().Trsf()), 1e-15));
        Point2D aP2 = new(1.0, 0.0);
        Point2D aO2 = new(0.0, 0.0);
        Assert.True(new Point2D(0, 1).IsEqual(aP2.Transformed(new GC_MakeRotation2d(aO2, M_PI / 2.0).Value().Trsf2d()), 1e-15));
        Assert.True(aP2.IsEqual(aP2.Transformed(new GC_MakeMirror2d(new Axis2D(aO2, new Direction2D(1, 0))).Value().Trsf2d()), 1e-15));
    }
}
