// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Occt.StepData_Logical;
using static Occt.StepGeom_BSplineCurveForm;
using static Occt.StepGeom_KnotType;
using static Occt.StepGeom_TrimmingPreference;

namespace OcctNet.DataExchange.Tests.GeomToStep;

/// <summary>Unit tests of StepToGeom (unit 8D); tools/geomsteprec compares it with native OCCT on the corpus.</summary>
public sealed class StepToGeomTests
{
    private static readonly StepData_Factors Inches = new StepData_Factors().InitializedFactors(25.4, M_PI / 180.0, 1.0);

    private static TCollection_HAsciiString Name() => new("");

    private static StepGeom_CartesianPoint Point(double x, double y, double z)
    {
        var P = new StepGeom_CartesianPoint();
        P.Init3D(Name(), x, y, z);
        return P;
    }

    private static StepGeom_CartesianPoint Point2d(double x, double y)
    {
        var P = new StepGeom_CartesianPoint();
        P.Init2D(Name(), x, y);
        return P;
    }

    private static StepGeom_Direction Direction(params double[] theRatios)
    {
        var D = new StepGeom_Direction();
        var aRatios = new NCollection_HArray1<double>(1, theRatios.Length);
        for (int i = 0; i < theRatios.Length; i++)
        {
            aRatios.SetValue(i + 1, theRatios[i]);
        }

        D.Init(Name(), aRatios);
        return D;
    }

    private static StepGeom_Axis2Placement3d Placement(StepGeom_Direction? theAxis = null, StepGeom_Direction? theRef = null)
    {
        var A = new StepGeom_Axis2Placement3d();
        A.Init(Name(), Point(1, 2, 3), theAxis is not null, theAxis!, theRef is not null, theRef!);
        return A;
    }

    private static StepGeom_Axis2Placement Select(global::Kerf.Engine.Foundation.Transient thePlacement)
    {
        var S = new StepGeom_Axis2Placement();
        S.SetValue(thePlacement);
        return S;
    }

    [Fact]
    public void Points_are_scaled_by_the_length_factor()
    {
        Geom_CartesianPoint? P = StepToGeom.MakeCartesianPoint(Point(1, -2, 0.5), Inches);
        Assert.NotNull(P);
        Assert.Equal(new Point3D(25.4, -50.8, 12.7), P.Pnt());
        Assert.Null(StepToGeom.MakeCartesianPoint(Point2d(1, 2)));
        Assert.Null(StepToGeom.MakeCartesianPoint(null!));

        // 2D points are parameters: no factor
        Assert.Equal(new Point2D(1, 2), StepToGeom.MakeCartesianPoint2d(Point2d(1, 2), Inches)!.Pnt2d());
        Assert.Null(StepToGeom.MakeCartesianPoint2d(Point(1, 2, 3)));
    }

    [Fact]
    public void Null_and_infinite_directions_are_not_made()
    {
        Assert.Null(StepToGeom.MakeDirection(Direction(0, 0, 0)));
        Assert.Null(StepToGeom.MakeDirection(Direction(0, 1e300, 0)));
        Assert.Null(StepToGeom.MakeDirection(Direction(1, 0)));
        Assert.Equal(new Direction3D(0, 0, 1), StepToGeom.MakeDirection(Direction(0, 0, 5))!.Dir());
        Assert.Null(StepToGeom.MakeDirection2d(Direction(0, 0, 1)));
        Assert.Equal(new Direction2D(1, 0), StepToGeom.MakeDirection2d(Direction(3, 0, 7))!.Dir2d());
    }

    [Fact]
    public void A_placement_without_directions_takes_the_defaults()
    {
        Geom_Axis2Placement? A = StepToGeom.MakeAxis2Placement(Placement());
        Assert.NotNull(A);
        Assert.Equal(new Point3D(1, 2, 3), A.Ax2().Location());
        Assert.Equal(new Direction3D(0, 0, 1), A.Ax2().Direction());

        // a reference direction parallel to the axis is ignored
        Geom_Axis2Placement? B = StepToGeom.MakeAxis2Placement(Placement(Direction(1, 0, 0), Direction(-2, 0, 0)));
        Assert.Equal(new Direction3D(1, 0, 0), B!.Ax2().Direction());
        Assert.True(B.Ax2().XDirection().IsNormal(new Direction3D(1, 0, 0), 1e-12));
    }

    [Fact]
    public void An_ellipse_with_its_first_semi_axis_the_smaller_is_turned()
    {
        var E = new StepGeom_Ellipse();
        E.Init(Name(), Select(Placement(Direction(0, 0, 1), Direction(1, 0, 0))), 2, 5);
        Geom_Ellipse? G = StepToGeom.MakeEllipse(E);
        Assert.NotNull(G);
        Assert.Equal(5, G.MajorRadius());
        Assert.Equal(2, G.MinorRadius());
        Assert.True(G.XAxis().Direction().IsParallel(new Direction3D(0, 1, 0), 1e-12));
    }

    [Fact]
    public void A_cone_takes_the_angle_factor_and_a_minimum_semi_angle()
    {
        var C = new StepGeom_ConicalSurface();
        C.Init(Name(), Placement(), 2, 30);
        Geom_ConicalSurface? G = StepToGeom.MakeConicalSurface(C, Inches);
        Assert.NotNull(G);
        Assert.Equal(30 * (M_PI / 180.0), G.SemiAngle());
        Assert.Equal(2 * 25.4, G.RefRadius());

        C.SetSemiAngle(0);
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Angular(), StepToGeom.MakeConicalSurface(C)!.SemiAngle());
    }

    [Fact]
    public void A_circle_trimmed_by_degrees()
    {
        var aCircle = new StepGeom_Circle();
        aCircle.Init(Name(), Select(Placement(Direction(0, 0, 1), Direction(1, 0, 0))), 10);
        var T1 = new StepGeom_TrimmingSelect();
        T1.SetParameterValue(90);
        var T2 = new StepGeom_TrimmingSelect();
        T2.SetParameterValue(180);
        var aTrim1 = new NCollection_HArray1<StepGeom_TrimmingSelect>(1, 1);
        aTrim1.SetValue(1, T1);
        var aTrim2 = new NCollection_HArray1<StepGeom_TrimmingSelect>(1, 1);
        aTrim2.SetValue(1, T2);
        var T = new StepGeom_TrimmedCurve();
        T.Init(Name(), aCircle, aTrim1, aTrim2, true, StepGeom_tpParameter);

        Geom_TrimmedCurve? G = StepToGeom.MakeTrimmedCurve(T, Inches);
        Assert.NotNull(G);
        Assert.Equal(M_PI / 2, G.FirstParameter(), 1e-14);
        Assert.Equal(M_PI, G.LastParameter(), 1e-14);
        Assert.Equal(254, (G.BasisCurve() as Geom_Circle)!.Radius());
    }

    [Fact]
    public void Excess_end_multiplicities_drop_poles()
    {
        // degree 1 with end multiplicities 3: one pole dropped at each end
        var aPoles = new NCollection_HArray1<StepGeom_CartesianPoint>(1, 4);
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
        var B = new StepGeom_BSplineCurveWithKnots();
        B.Init(Name(), 1, aPoles, StepGeom_bscfUnspecified, StepData_LFalse, StepData_LFalse, aMults, aKnots, StepGeom_ktUnspecified);

        Geom_BSplineCurve? G = StepToGeom.MakeBSplineCurve(B);
        Assert.NotNull(G);
        Assert.Equal(2, G.NbPoles());
        Assert.Equal(new Point3D(2, 4, 0), G.Pole(1));
        Assert.Equal(new Point3D(3, 9, 0), G.Pole(2));
    }

    [Fact]
    public void Cyclic_replicas_end_without_a_stack_overflow()
    {
        // two curve replicas of each other: OCCT recurses until the stack overflows; the port throws a catchable
        // exception (decision 1240), and MakeSurface's catch-all turns a surface cycle into null as OCCT does
        var anOperator = new StepGeom_CartesianTransformationOperator3d();
        anOperator.SetLocalOrigin(Point(0, 0, 0));
        var A = new StepGeom_CurveReplica();
        var B = new StepGeom_CurveReplica();
        A.Init(Name(), B, anOperator);
        B.Init(Name(), A, anOperator);
        Assert.ThrowsAny<InsufficientExecutionStackException>(() => StepToGeom.MakeCurve(A));

        var S = new StepGeom_SurfaceReplica();
        var R = new StepGeom_SurfaceReplica();
        S.Init(Name(), R, anOperator);
        R.Init(Name(), S, anOperator);
        Assert.Null(StepToGeom.MakeSurface(S));
    }

    [Fact]
    public void A_transformation_operator_is_inverted()
    {
        var anOperator = new StepGeom_CartesianTransformationOperator3d();
        anOperator.SetLocalOrigin(Point(1, 2, 3));
        anOperator.SetAxis1(Direction(0, 1, 0));
        anOperator.SetAxis2(Direction(-1, 0, 0));
        Transform3D T = new Transform3D();
        Assert.True(StepToGeom.MakeTransformation3d(anOperator, ref T));
        // the inverse of the change of coordinates into the operator's system: its origin is the global origin's
        // image, and its first axis the X axis's
        Assert.True(new Point3D(0, 0, 0).Transformed(T).Distance(new Point3D(1, 2, 3)) < 1e-12);
        Assert.True(new Direction3D(1, 0, 0).Transformed(T).IsEqual(new Direction3D(0, 1, 0), 1e-12));

        var aNoOrigin = new StepGeom_CartesianTransformationOperator3d();
        aNoOrigin.SetLocalOrigin(Point2d(1, 2));
        Transform3D U = new Transform3D();
        Assert.False(StepToGeom.MakeTransformation3d(aNoOrigin, ref U));
    }
}
