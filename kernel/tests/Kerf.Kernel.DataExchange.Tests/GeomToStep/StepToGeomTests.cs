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
        // exception (decision 1240), which MakeSurface now lets through too, so that the read fails (decision 1859)
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
        Assert.ThrowsAny<InsufficientExecutionStackException>(() => global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeSurface(S));
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

    private static NCollection_HArray1<StepTrimmingSelect> Trim(params object[] theValues)
    {
        var aTrim = new NCollection_HArray1<StepTrimmingSelect>(1, theValues.Length);
        for (int i = 0; i < theValues.Length; i++)
        {
            var aSelect = new StepTrimmingSelect();
            if (theValues[i] is double aParameter)
            {
                aSelect.SetParameterValue(aParameter);
            }
            else
            {
                aSelect.SetValue((StepCartesianPoint)theValues[i]);
            }
            aTrim.SetValue(i + 1, aSelect);
        }
        return aTrim;
    }

    private static StepCircle Circle(double theRadius, bool theWithRef = true)
    {
        var aCircle = new StepCircle();
        aCircle.Init(Name(), Select(theWithRef ? Placement(Direction(0, 0, 1), Direction(1, 0, 0)) : Placement(Direction(0, 0, 1))), theRadius);
        return aCircle;
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void A_circle_trimmed_by_points_runs_between_them(bool theCartesian)
    {
        StepTrimmingPreference thePreference = theCartesian ? StepTrimmingPreference.TpCartesian : StepTrimmingPreference.TpUnspecified;
        // the circle of radius 10 about (1, 2, 3), from its point on X to its point on Y: a quarter turn
        var T = new StepTrimmedCurve();
        T.Init(Name(), Circle(10), Trim(Point(11, 2, 3)), Trim(Point(1, 12, 3)), true, thePreference);
        TrimmedCurve3D? G = global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeTrimmedCurve(T);
        Assert.NotNull(G);
        Assert.Equal(0, G.FirstParameter, 1e-9);
        Assert.Equal(M_PI / 2, G.LastParameter, 1e-9);
        Assert.True(G.Value(G.LastParameter).DistanceTo(new Point3D(1, 12, 3)) < 1e-9);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void A_circle_with_no_reference_direction_trimmed_by_parameters_is_whole(bool theSense)
    {
        // with no X direction the parameters mean nothing: the whole circle, in the sense asked
        var T = new StepTrimmedCurve();
        T.Init(Name(), Circle(10, false), Trim(30.0), Trim(60.0), theSense, StepTrimmingPreference.TpParameter);
        TrimmedCurve3D? G = global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeTrimmedCurve(T, Inches);
        Assert.NotNull(G);
        Assert.Equal(2 * M_PI, Math.Abs(G.LastParameter - G.FirstParameter), 1e-12);
        // its points: the whole circle, from one end round to the other
        Assert.True(G.Value(G.FirstParameter).DistanceTo(G.Value(G.LastParameter)) < 1e-9);
    }

    [Fact]
    public void A_line_trimmed_by_parameters_counts_them_in_its_vector()
    {
        // from (1, 2, 3) along a vector of length 2 in X: parameters 1 and 3 are 2 and 6 along it
        var aVector = new StepVector();
        aVector.Init(Name(), Direction(1, 0, 0), 2);
        var aLine = new StepLine();
        aLine.Init(Name(), Point(1, 2, 3), aVector);
        var T = new StepTrimmedCurve();
        T.Init(Name(), aLine, Trim(1.0), Trim(3.0), true, StepTrimmingPreference.TpParameter);
        TrimmedCurve3D? G = global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeTrimmedCurve(T);
        Assert.NotNull(G);
        Assert.True(G.Value(G.FirstParameter).DistanceTo(new Point3D(3, 2, 3)) < 1e-12);
        Assert.True(G.Value(G.LastParameter).DistanceTo(new Point3D(7, 2, 3)) < 1e-12);
        // and by points, wherever the vector's length
        var P = new StepTrimmedCurve();
        P.Init(Name(), aLine, Trim(Point(2, 2, 3)), Trim(Point(5, 2, 3)), true, StepTrimmingPreference.TpCartesian);
        TrimmedCurve3D? H = global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeTrimmedCurve(P);
        Assert.Equal(3, H!.LastParameter - H.FirstParameter, 1e-12);
    }

    [Fact]
    public void An_ellipse_trimmed_by_parameters_is_turned_with_its_axes()
    {
        // its first semi-axis the smaller: the axes are swapped, so its parameters move by a quarter turn
        var E = new StepEllipse();
        E.Init(Name(), Select(Placement(Direction(0, 0, 1), Direction(1, 0, 0))), 2, 5);
        var T = new StepTrimmedCurve();
        T.Init(Name(), E, Trim(0.0), Trim(90.0), true, StepTrimmingPreference.TpParameter);
        TrimmedCurve3D? G = global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeTrimmedCurve(T, new Factors().InitializedFactors(1.0, M_PI / 180.0, 1.0));
        Assert.NotNull(G);
        // STEP's parameter 0 is on its first axis (X, at 2): the point (3, 2, 3); 90 degrees on the second (Y, at 5)
        Assert.True(G.Value(G.FirstParameter).DistanceTo(new Point3D(3, 2, 3)) < 1e-9, G.Value(G.FirstParameter).ToString());
        Assert.True(G.Value(G.LastParameter).DistanceTo(new Point3D(1, 7, 3)) < 1e-9, G.Value(G.LastParameter).ToString());
    }

    [Fact]
    public void A_2d_transformation_operator_is_inverted()
    {
        // as in 3D: the operator's origin is the image of the global origin, its first axis the X axis's
        var anOperator = new StepCartesianTransformationOperator2d();
        anOperator.SetLocalOrigin(Point2d(1, 2));
        anOperator.SetAxis1(Direction(0, 1));
        Transform2D T = new Transform2D();
        Assert.True(global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeTransformation2d(anOperator, ref T));
        Assert.True(new Point2D(0, 0).Transformed(T).DistanceTo(new Point2D(1, 2)) < 1e-12);
        Assert.True(new Point2D(1, 0).Transformed(T).DistanceTo(new Point2D(1, 3)) < 1e-12);
        // without its first axis, X; without a 2D origin, nothing
        var aNoAxis = new StepCartesianTransformationOperator2d();
        aNoAxis.SetLocalOrigin(Point2d(3, 4));
        Transform2D U = new Transform2D();
        Assert.True(global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeTransformation2d(aNoAxis, ref U));
        Assert.True(new Point2D(1, 0).Transformed(U).DistanceTo(new Point2D(4, 4)) < 1e-12);
        var aNoOrigin = new StepCartesianTransformationOperator2d();
        aNoOrigin.SetLocalOrigin(Point(1, 2, 3));
        Assert.False(global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeTransformation2d(aNoOrigin, ref U));
    }

    [Fact]
    public void Su_parameters_place_a_frame_by_their_turns_and_offsets()
    {
        StepSuParameters Su(double a, double alpha, double b, double beta, double c, double gamma)
        {
            var aSu = new StepSuParameters();
            aSu.SetA(a);
            aSu.SetAlpha(alpha);
            aSu.SetB(b);
            aSu.SetBeta(beta);
            aSu.SetC(c);
            aSu.SetGamma(gamma);
            return aSu;
        }
        // none: the world frame; c alone lifts it, a alone moves it along X
        Frame3D aNone = global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeAxis2Placement(Su(0, 0, 0, 0, 0, 0)).Ax2();
        Assert.True(aNone.Location.DistanceTo(Point3D.Origin) < 1e-12 && aNone.Direction.IsEqual(Direction3D.UnitZ, 1e-12) && aNone.XDirection.IsEqual(Direction3D.UnitX, 1e-12));
        Assert.True(global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeAxis2Placement(Su(0, 0, 0, 0, 5, 0)).Ax2().Location.DistanceTo(new Point3D(0, 0, 5)) < 1e-12);
        Assert.True(global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeAxis2Placement(Su(2, 0, 0, 0, 0, 0)).Ax2().Location.DistanceTo(new Point3D(2, 0, 0)) < 1e-12);
        // gamma turns everything about Z, beta the X direction about the axis
        Frame3D aTurned = global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeAxis2Placement(Su(2, 0, 0, Math.PI / 2, 0, Math.PI / 2)).Ax2();
        Assert.True(aTurned.Location.DistanceTo(new Point3D(0, 2, 0)) < 1e-12);
        Assert.True(aTurned.XDirection.IsEqual(new Direction3D(-1, 0, 0), 1e-12));
        // alpha tilts the axis, and b runs along the tilted axis
        Frame3D aTilted = global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeAxis2Placement(Su(0, Math.PI / 2, 3, 0, 0, 0)).Ax2();
        Assert.True(aTilted.Direction.IsEqual(new Direction3D(0, -1, 0), 1e-12));
        Assert.True(aTilted.Location.DistanceTo(new Point3D(0, -3, 0)) < 1e-12);
    }

    [Fact]
    public void A_circle_about_a_diameter_is_revolved_from_pole_to_pole()
    {
        // a circle of radius 2 about (0, 0, 0) in the plane XZ, turned about Z, which runs through its centre in its
        // plane: the half circle from pole to pole is taken, so the surface is the sphere, once over
        var aCircle = new StepCircle();
        aCircle.Init(Name(), Select(Placement(Direction(0, 1, 0), Direction(1, 0, 0))), 2);
        var aCentre = new StepCartesianPoint();
        aCentre.Init3D(Name(), 1, 2, 3);
        var anAxis = new StepAxis1Placement();
        anAxis.Init(Name(), aCentre, true, Direction(0, 0, 1));
        var aRevolution = new StepSurfaceOfRevolution();
        aRevolution.Init(Name(), aCircle, anAxis);
        RevolutionSurface? G = global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeSurfaceOfRevolution(aRevolution);
        Assert.NotNull(G);
        TrimmedCurve3D aHalf = Assert.IsType<TrimmedCurve3D>(G.BasisCurve);
        Assert.Equal(M_PI, aHalf.LastParameter - aHalf.FirstParameter, 12);
        for (int i = 0; i <= 4; i++)
        {
            for (int j = 0; j <= 4; j++)
            {
                Point3D aP = G.Value(2 * M_PI * i / 4, aHalf.FirstParameter + (M_PI * j / 4));
                Assert.Equal(2, aP.DistanceTo(new Point3D(1, 2, 3)), 12);
            }
        }
        // its poles are on the axis
        Assert.Equal(0, Math.Sqrt(Math.Pow(G.Value(0, aHalf.FirstParameter).X - 1, 2) + Math.Pow(G.Value(0, aHalf.FirstParameter).Y - 2, 2)), 9);
    }
}
