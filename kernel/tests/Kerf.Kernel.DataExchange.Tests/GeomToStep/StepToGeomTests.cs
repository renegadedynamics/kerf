// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using static Kerf.Engine.Exchange.Step.Data.Logical;
using static Kerf.Engine.Exchange.Step.Geometry.StepBSplineCurveForm;
using static Kerf.Engine.Exchange.Step.Geometry.StepKnotType;
using static Kerf.Engine.Exchange.Step.Geometry.StepTrimmingPreference;

namespace Kerf.Kernel.DataExchange.Tests.GeomToStep;

/// <summary>Unit tests of StepToGeom (unit 8D); tools/geomsteprec compares it with native OCCT on the corpus.</summary>
public sealed class StepToGeomTests
{
    private static readonly Factors Inches = new Factors().InitializedFactors(25.4, M_PI / 180.0, 1.0);

    private static TCollection_HAsciiString Name() => new("");

    private static StepCartesianPoint Point(double x, double y, double z)
    {
        var P = new StepCartesianPoint();
        P.Init3D(Name(), x, y, z);
        return P;
    }

    private static StepCartesianPoint Point2d(double x, double y)
    {
        var P = new StepCartesianPoint();
        P.Init2D(Name(), x, y);
        return P;
    }

    private static StepDirection Direction(params double[] theRatios)
    {
        var D = new StepDirection();
        var aRatios = new NCollection_HArray1<double>(1, theRatios.Length);
        for (int i = 0; i < theRatios.Length; i++)
        {
            aRatios.SetValue(i + 1, theRatios[i]);
        }

        D.Init(Name(), aRatios);
        return D;
    }

    private static StepAxis2Placement3d Placement(StepDirection? theAxis = null, StepDirection? theRef = null)
    {
        var A = new StepAxis2Placement3d();
        A.Init(Name(), Point(1, 2, 3), theAxis is not null, theAxis!, theRef is not null, theRef!);
        return A;
    }

    private static StepAxis2Placement Select(global::Kerf.Engine.Foundation.Transient thePlacement)
    {
        var S = new StepAxis2Placement();
        S.SetValue(thePlacement);
        return S;
    }

    [Fact]
    public void Points_are_scaled_by_the_length_factor()
    {
        CartesianPoint? P = global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeCartesianPoint(Point(1, -2, 0.5), Inches);
        Assert.NotNull(P);
        Assert.Equal(new Point3D(25.4, -50.8, 12.7), P.Pnt());
        Assert.Null(global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeCartesianPoint(Point2d(1, 2)));
        Assert.Null(global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeCartesianPoint(null!));

        // 2D points are parameters: no factor
        Assert.Equal(new Point2D(1, 2), global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeCartesianPoint2d(Point2d(1, 2), Inches)!.Pnt2d());
        Assert.Null(global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeCartesianPoint2d(Point(1, 2, 3)));
    }

    [Fact]
    public void Null_and_infinite_directions_are_not_made()
    {
        Assert.Null(global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeDirection(Direction(0, 0, 0)));
        Assert.Null(global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeDirection(Direction(0, 1e300, 0)));
        Assert.Null(global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeDirection(Direction(1, 0)));
        Assert.Equal(new Direction3D(0, 0, 1), global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeDirection(Direction(0, 0, 5))!.Dir());
        Assert.Null(global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeDirection2d(Direction(0, 0, 1)));
        Assert.Equal(new Direction2D(1, 0), global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeDirection2d(Direction(3, 0, 7))!.Dir2d());
    }

    [Fact]
    public void A_placement_without_directions_takes_the_defaults()
    {
        Axis2Placement? A = global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeAxis2Placement(Placement());
        Assert.NotNull(A);
        Assert.Equal(new Point3D(1, 2, 3), A.Ax2().Location);
        Assert.Equal(new Direction3D(0, 0, 1), A.Ax2().Direction);

        // a reference direction parallel to the axis is ignored
        Axis2Placement? B = global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeAxis2Placement(Placement(Direction(1, 0, 0), Direction(-2, 0, 0)));
        Assert.Equal(new Direction3D(1, 0, 0), B!.Ax2().Direction);
        Assert.True(B.Ax2().XDirection.IsNormal(new Direction3D(1, 0, 0), 1e-12));
    }

    [Fact]
    public void An_ellipse_with_its_first_semi_axis_the_smaller_is_turned()
    {
        var E = new StepEllipse();
        E.Init(Name(), Select(Placement(Direction(0, 0, 1), Direction(1, 0, 0))), 2, 5);
        EllipseCurve3D? G = global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeEllipse(E);
        Assert.NotNull(G);
        Assert.Equal(5, G.MajorRadius);
        Assert.Equal(2, G.MinorRadius);
        Assert.True(G.XAxis.Direction.IsParallel(new Direction3D(0, 1, 0), 1e-12));
    }

    [Fact]
    public void A_cone_takes_the_angle_factor_and_a_minimum_semi_angle()
    {
        var C = new StepConicalSurface();
        C.Init(Name(), Placement(), 2, 30);
        ConicalSurface? G = global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeConicalSurface(C, Inches);
        Assert.NotNull(G);
        Assert.Equal(30 * (M_PI / 180.0), G.SemiAngle);
        Assert.Equal(2 * 25.4, G.RefRadius);

        C.SetSemiAngle(0);
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Angular(), global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeConicalSurface(C)!.SemiAngle);
    }

    [Fact]
    public void A_circle_trimmed_by_degrees()
    {
        var aCircle = new StepCircle();
        aCircle.Init(Name(), Select(Placement(Direction(0, 0, 1), Direction(1, 0, 0))), 10);
        var T1 = new StepTrimmingSelect();
        T1.SetParameterValue(90);
        var T2 = new StepTrimmingSelect();
        T2.SetParameterValue(180);
        var aTrim1 = new NCollection_HArray1<StepTrimmingSelect>(1, 1);
        aTrim1.SetValue(1, T1);
        var aTrim2 = new NCollection_HArray1<StepTrimmingSelect>(1, 1);
        aTrim2.SetValue(1, T2);
        var T = new StepTrimmedCurve();
        T.Init(Name(), aCircle, aTrim1, aTrim2, true, StepTrimmingPreference.TpParameter);

        TrimmedCurve3D? G = global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeTrimmedCurve(T, Inches);
        Assert.NotNull(G);
        Assert.Equal(M_PI / 2, G.FirstParameter, 1e-14);
        Assert.Equal(M_PI, G.LastParameter, 1e-14);
        Assert.Equal(254, (G.BasisCurve as CircleCurve3D)!.Radius);
    }

    [Fact]
    public void Excess_end_multiplicities_drop_poles()
    {
        // degree 1 with end multiplicities 3: one pole dropped at each end
        var aPoles = new NCollection_HArray1<StepCartesianPoint>(1, 4);
        for (int i = 1; i <= 4; i++)
        {
            aPoles.SetValue(i, Point(i, i * i, 0));
        }

        var aMults = new NCollection_HArray1<int>(1, 2);
        aMults.SetValue(1, 3);
        aMults.SetValue(2, 3);
        var aKnots = new NCollection_HArray1<double>(1, 2);
        aKnots.SetValue(1, 0);
        aKnots.SetValue(2, 1);
        var B = new StepBSplineCurveWithKnots();
        B.Init(Name(), 1, aPoles, StepBSplineCurveForm.BscfUnspecified, Logical.LFalse, Logical.LFalse, aMults, aKnots, StepKnotType.KtUnspecified);

        BSplineCurve3D? G = global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeBSplineCurve(B);
        Assert.NotNull(G);
        Assert.Equal(2, G.PoleCount);
        Assert.Equal(new Point3D(2, 4, 0), G.Pole(1));
        Assert.Equal(new Point3D(3, 9, 0), G.Pole(2));
    }

    [Fact]
    public void Cyclic_replicas_end_without_a_stack_overflow()
    {
        // two curve replicas of each other: OCCT recurses until the stack overflows; the port throws a catchable
        // exception (decision 1240), and MakeSurface's catch-all turns a surface cycle into null as OCCT does
        var anOperator = new StepCartesianTransformationOperator3d();
        anOperator.SetLocalOrigin(Point(0, 0, 0));
        var A = new StepCurveReplica();
        var B = new StepCurveReplica();
        A.Init(Name(), B, anOperator);
        B.Init(Name(), A, anOperator);
        Assert.ThrowsAny<InsufficientExecutionStackException>(() => global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeCurve(A));

        var S = new StepSurfaceReplica();
        var R = new StepSurfaceReplica();
        S.Init(Name(), R, anOperator);
        R.Init(Name(), S, anOperator);
        Assert.Null(global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeSurface(S));
    }

    [Fact]
    public void A_transformation_operator_is_inverted()
    {
        var anOperator = new StepCartesianTransformationOperator3d();
        anOperator.SetLocalOrigin(Point(1, 2, 3));
        anOperator.SetAxis1(Direction(0, 1, 0));
        anOperator.SetAxis2(Direction(-1, 0, 0));
        Transform3D T = new Transform3D();
        Assert.True(global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeTransformation3d(anOperator, ref T));
        // the inverse of the change of coordinates into the operator's system: its origin is the global origin's
        // image, and its first axis the X axis's
        Assert.True(new Point3D(0, 0, 0).Transformed(T).DistanceTo(new Point3D(1, 2, 3)) < 1e-12);
        Assert.True(new Direction3D(1, 0, 0).Transformed(T).IsEqual(new Direction3D(0, 1, 0), 1e-12));

        var aNoOrigin = new StepCartesianTransformationOperator3d();
        aNoOrigin.SetLocalOrigin(Point2d(1, 2));
        Transform3D U = new Transform3D();
        Assert.False(global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeTransformation3d(aNoOrigin, ref U));
    }
}
