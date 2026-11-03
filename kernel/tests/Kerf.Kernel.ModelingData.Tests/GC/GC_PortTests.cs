// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Curves;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>Tests of the GC port beyond OCCT's GTests: failure statuses, freezing, and the gce failure paths.</summary>
public class GC_PortTests
{
    private static readonly Frame3D theZAxes = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));

    [Fact]
    public void FailedConstructions_ReportStatus_AndValueThrows()
    {
        MakeCircle aCircle = new(theZAxes, -1.0);
        Assert.Equal(NegativeRadius, aCircle.Status());
        Assert.True(aCircle.IsError);
        Assert.Throws<NotDoneException>(() => aCircle.Value());
        Assert.Throws<NotDoneException>(() => (CircleCurve3D)aCircle);

        Assert.Equal(InvertAxis, new MakeEllipse(theZAxes, 1.0, 2.0).Status());
        Assert.Equal(NegativeRadius, new MakeEllipse(theZAxes, 1.0, -2.0).Status());
        Assert.Equal(NegativeRadius, new MakeHyperbola(theZAxes, -1.0, 2.0).Status());
        Assert.Equal(ConfusedPoints, new MakeLine(new Point3D(1, 2, 3), new Point3D(1, 2, 3)).Status());
        Assert.Equal(BadEquation, new MakePlane(0.0, 0.0, 0.0, 1.0).Status());
        Assert.Equal(NegativeRadius, new MakeCylindricalSurface(theZAxes, -1.0).Status());
        Assert.Equal(NegativeRadius, new MakeConicalSurface(theZAxes, 0.3, -1.0).Status());
        Assert.Equal(BadAngle, new MakeConicalSurface(theZAxes, M_PI / 2.0, 1.0).Status());
        Assert.Equal(NullFocusLength,
                     new MakeParabola2d(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), -1.0, true).Status());
        Assert.Equal(ConfusedPoints,
                     new MakeArcOfCircle(new Point3D(1, 0, 0), new Vector3D(0, 1, 0), new Point3D(1, 0, 0)).Status());
        Assert.False(new MakeTrimmedCylinder(new Point3D(0, 0, 0), new Point3D(0, 0, 0), new Point3D(1, 0, 0)).IsDone);
    }

    [Fact]
    public void Segment_ConfusedPoints_Throws()
    {
        // OCCT's release build dereferences a null line here; the port throws from GC_MakeLine's Value().
        Assert.Throws<NotDoneException>(() => new MakeSegment(new Point3D(1, 2, 3), new Point3D(1, 2, 3)));
    }

    [Fact]
    public void Value_IsFrozen_AndTheSameObject()
    {
        MakeSegment aSegment = new(new Point3D(0, 0, 0), new Point3D(3, 4, 0));
        TrimmedCurve3D aCurve = aSegment.Value();
        Assert.True(aCurve.IsFrozen);
        Assert.Same(aCurve, aSegment.Value());
        Assert.Same(aCurve, (TrimmedCurve3D)aSegment);
        Assert.Equal(5.0, aCurve.LastParameter(), 1e-15);
        Assert.Throws<ImmutableObjectException>(() => aCurve.SetTrim(0.0, 1.0));
        Assert.False(((TrimmedCurve3D)aCurve.Copy()).IsFrozen);

        MakeRotation aRotation = new(new Axis3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 0.5);
        Assert.True(aRotation.Value().IsFrozen);
        Assert.True(new MakeTranslation2d(new Vector2D(1, 2)).Value().IsFrozen);
        Assert.True(new MakePlane(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)).Value().IsFrozen);
    }

    [Fact]
    public void ArcOfCircle2d_FromConfusedPoints_Throws()
    {
        // OCCT converts gce_MakeLin2d(P1, P1) implicitly: its release build goes on with the default line, finds
        // the center at P1 and computes the parameters of P1 on a circle of radius 0, where ElCLib makes a gp_Dir2d
        // of a null vector (NaN parameters in the release build). The port reads the same default line, and throws
        // there instead.
        Point2D aP = new(1.0, 2.0);
        Assert.Throws<ConstructionErrorException>(() => new MakeArcOfCircle2d(aP, new Vector2D(0.0, 1.0), aP));
    }

    [Fact]
    public void ArcOfCircle2d_FromTangent_FollowsTheTangent()
    {
        Point2D aP1 = new(1.0, 0.0);
        Point2D aP2 = new(-1.0, 0.0);
        MakeArcOfCircle2d aMaker = new(aP1, new Vector2D(0.0, 1.0), aP2);
        Assert.True(aMaker.IsDone);
        TrimmedCurve2D anArc = aMaker.Value();
        Assert.True(anArc.StartPoint().IsEqual(aP1, 1e-12));
        Assert.True(anArc.EndPoint().IsEqual(aP2, 1e-12));
        Assert.True(anArc.Value(0.5 * (anArc.FirstParameter() + anArc.LastParameter())).IsEqual(new Point2D(0, 1), 1e-12));
    }

    [Fact]
    public void Surfaces_AndTrimmedSurfaces()
    {
        Cylinder aCyl = new(new CoordinateSystem(theZAxes), 5.0);
        Assert.Equal(3.0, new MakeCylindricalSurface(aCyl, 2.0).Value().Radius());
        Assert.Equal(2.0, new MakeCylindricalSurface(aCyl, new Point3D(0.0, 2.0, 7.0)).Value().Radius(), 1e-15);

        MakeTrimmedCylinder aTrimmed = new(new Axis3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 2.0, 3.0);
        aTrimmed.Value().Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
        Assert.Equal((0.0, 2.0 * M_PI, 0.0, 3.0), (aU1, aU2, aV1, aV2));

        MakeTrimmedCone aCone = new(new Point3D(0, 0, 0), new Point3D(0, 0, 4), 3.0, 1.0);
        Assert.True(aCone.IsDone);
        aCone.Value().Bounds(out _, out _, out _, out double aConeV2);
        Assert.Equal(Math.Sqrt(4.0 * 4.0 + 2.0 * 2.0), aConeV2, 1e-12);
    }

    [Fact]
    public void Transformations()
    {
        Point3D aP = new(1.0, 2.0, 3.0);
        Point3D aO = new(0.0, 0.0, 0.0);
        Assert.True(aP.Mirrored(aO).IsEqual(aP.Transformed(new MakeMirror(aO).Value().Trsf()), 1e-15));
        Assert.True(new Point3D(3, 6, 9).IsEqual(aP.Transformed(new MakeScale(aO, 3.0).Value().Trsf()), 1e-15));
        Assert.True(new Point3D(2, 4, 6).IsEqual(aP.Transformed(new MakeTranslation(aO, aP).Value().Trsf()), 1e-15));
        Point2D aP2 = new(1.0, 0.0);
        Point2D aO2 = new(0.0, 0.0);
        Assert.True(new Point2D(0, 1).IsEqual(aP2.Transformed(new MakeRotation2d(aO2, M_PI / 2.0).Value().Trsf2d()), 1e-15));
        Assert.True(aP2.IsEqual(aP2.Transformed(new MakeMirror2d(new Axis2D(aO2, new Direction2D(1, 0))).Value().Trsf2d()), 1e-15));
    }
}
