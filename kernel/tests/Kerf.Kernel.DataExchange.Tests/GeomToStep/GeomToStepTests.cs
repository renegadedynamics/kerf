// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Export.Geometry;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Kerf.Kernel.DataExchange.Tests.GeomToStep;

/// <summary>Unit tests of GeomToStep (unit 8D); tools/geomsteprec compares it with native OCCT.</summary>
public sealed class GeomToStepTests
{
    private static readonly Factors Inches = new Factors().InitializedFactors(25.4, M_PI / 180.0, 1.0);

    private static readonly Frame3D Tilted = new(new Point3D(1, 2, 3), new Direction3D(0, 0.6, 0.8), new Direction3D(1, 0, 0));

    [Fact]
    public void Points_are_divided_by_the_length_factor()
    {
        StepCartesianPoint P = new MakeCartesianPoint(new Point3D(25.4, 50.8, -12.7), Inches.LengthFactor()).Value();
        Assert.Equal(3, P.CoordinateCount);
        Assert.Equal([1.0, 2.0, -0.5], P.Coordinates().ToArray());

        StepCartesianPoint Q = new MakeCartesianPoint(new Point2D(3, 4), Inches.LengthFactor()).Value();
        Assert.Equal(2, Q.CoordinateCount);
        Assert.Equal(4.0, Q.CoordinatesValue(2));
    }

    [Fact]
    public void A_line_keeps_a_unit_vector_divided_by_the_length_factor()
    {
        StepLine L = new GeometryMakeLine(new LineCurve3D(new Point3D(0, 0, 25.4), new Direction3D(0, 1, 0)), Inches).Value();
        Assert.Equal(1.0, L.Pnt().CoordinatesValue(3));
        Assert.Equal(1.0 / 25.4, L.Dir().Magnitude());

        // 2D lines take no factor
        StepLine L2 = new GeometryMakeLine(new LineCurve2D(new Point2D(1, 1), new Direction2D(1, 0)), Inches).Value();
        Assert.Equal(1.0, L2.Dir().Magnitude());
    }

    [Fact]
    public void Curves_and_surfaces_convert_back()
    {
        var aCircle = new CircleCurve3D(Tilted, 7.5);
        var C = new MakeCurve(aCircle, Inches);
        Assert.True(C.IsDone);
        var aBack = global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeCurve(C.Value(), Inches) as CircleCurve3D;
        Assert.NotNull(aBack);
        Assert.Equal(7.5, aBack.Radius, 1e-12);
        Assert.True(aBack.Location.DistanceTo(Tilted.Location) < 1e-12);

        var aTorus = new ToroidalSurface(new CoordinateSystem(Tilted), 6, 1.5);
        var S = new MakeSurface(aTorus);
        Assert.True(S.IsDone);
        var aTorusBack = global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeSurface(S.Value()) as ToroidalSurface;
        Assert.NotNull(aTorusBack);
        Assert.Equal(6, aTorusBack.MajorRadius);
        Assert.Equal(1.5, aTorusBack.MinorRadius);
    }

    [Fact]
    public void A_rectangular_trimmed_surface_is_written_in_the_plane_angle_unit()
    {
        // degrees with these factors; radians with the defaults, read back as written (OCCT wrote degrees whatever
        // the unit, which the defaults read as radians: occt-issues.md, GeomToStep_MakeRectangularTrimmedSurface;
        // Kerf decision 1510)
        var aCylinder = new CylindricalSurface(new CoordinateSystem(Tilted), 2);
        var aTrimmed = new TrimmedSurface(aCylinder, 0, M_PI / 2, -25.4, 50.8);
        StepRectangularTrimmedSurface T = new MakeRectangularTrimmedSurface(aTrimmed, Inches).Value();
        Assert.Equal(90.0, T.U2(), 1e-12);
        Assert.Equal(-1.0, T.V1(), 1e-12);
        Assert.Equal(2.0, T.V2(), 1e-12);
        Assert.IsType<StepCylindricalSurface>(T.BasisSurface());

        foreach (Factors aFactors in new[] { new Factors(), Inches })
        {
            StepRectangularTrimmedSurface aWritten = new MakeRectangularTrimmedSurface(aTrimmed, aFactors).Value();
            TrimmedSurface? aBack = global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeRectangularTrimmedSurface(aWritten, aFactors);
            Assert.NotNull(aBack);
            aBack.ParameterBounds(out double u1, out double u2, out double v1, out double v2);
            Assert.Equal(0.0, u1, 1e-12);
            Assert.Equal(M_PI / 2, u2, 1e-12);
            Assert.Equal((-25.4, 50.8), (Math.Round(v1, 9), Math.Round(v2, 9)));
        }

        // a cone's semi-angle likewise (OCCT wrote radians whatever the unit)
        var aCone = new ConicalSurface(new CoordinateSystem(Tilted), 0.3, 2);
        foreach (Factors aFactors in new[] { new Factors(), Inches })
        {
            var aWrittenCone = new GeometryMakeConicalSurface(aCone, aFactors).Value();
            Assert.Equal(aFactors.PlaneAngleFactor() == 1.0 ? 0.3 : 0.3 * 180.0 / M_PI, aWrittenCone.SemiAngle(), 1e-12);
            Assert.Equal(0.3, global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeConicalSurface(aWrittenCone, aFactors)!.SemiAngle, 1e-12);
        }
    }

    [Fact]
    public void Unmapped_geometry_is_not_done()
    {
        var aLine = new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        var anOffset = new OffsetCurve3D(aLine, 1, new Direction3D(0, 0, 1));
        var C = new MakeCurve(anOffset);
        Assert.False(C.IsDone);
        EngineAssert.Throws<NotDoneException>(() => C.Value());

        // a 2D trimmed curve is written as its basis (OCCT tested it as a bounded curve first, which MakeBoundedCurve
        // doesn't map, and threw: occt-issues.md, GeomToStep_MakeCurve; Kerf decision 1507)
        var aTrimmed2d = new TrimmedCurve2D(new LineCurve2D(new Point2D(0, 3), new Direction2D(1, 0)), 0, 1);
        var aLine2d = Assert.IsType<StepLine>(new MakeCurve(aTrimmed2d).Value());
        Assert.Equal(3.0, aLine2d.Pnt().CoordinatesValue(2));
        var anArc2d = new TrimmedCurve2D(new CircleCurve2D(new Axis2D(new Point2D(1, 1), new Direction2D(1, 0)), 2), 0.5, 2);
        var aCircleBack = global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeCurve2d(new MakeCurve(anArc2d).Value()) as CircleCurve2D;
        Assert.NotNull(aCircleBack);
        Assert.Equal(2.0, aCircleBack.Radius, 1e-12);
    }

    [Fact]
    public void A_rational_2d_Bezier_curve_keeps_its_weights()
    {
        // occt-issues.md, GeomToStep_MakeBoundedCurve (Geom2d; Kerf decision 1508): OCCT wrote it without its weights
        var aPoles = NCollection_Array1<Point2D>.From([new Point2D(0, 0), new Point2D(1, 2), new Point2D(3, 0)]);
        var aWeights = NCollection_Array1<double>.From([1.0, 0.5, 1.0]);
        var aBezier = new BezierCurve2D(aPoles, aWeights);
        StepCurve aWritten = new MakeCurve(aBezier).Value();
        Curve2D? aBack = global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeCurve2d(aWritten);
        Assert.NotNull(aBack);
        foreach (double t in new[] { 0.0, 0.25, 0.6, 1.0 })
        {
            double u = aBack.FirstParameter + t * (aBack.LastParameter - aBack.FirstParameter);
            Assert.True(aBezier.Value(t).DistanceTo(aBack.Value(u)) < 1e-12, $"at {t}");
        }
    }

    [Fact]
    public void A_cone_with_a_negative_semi_angle_is_refused()
    {
        var aCone = new ConicalSurface(new CoordinateSystem(Tilted), -0.3, 2);
        EngineAssert.Throws<DomainErrorException>(() => new GeometryMakeConicalSurface(aCone));
    }

    [Fact]
    public void Frozen_periodic_input_is_converted_on_a_copy()
    {
        var aPoles = new NCollection_Array1<Point3D>(1, 4);
        aPoles.SetValue(1, new Point3D(0, 0, 0));
        aPoles.SetValue(2, new Point3D(1, 0, 0));
        aPoles.SetValue(3, new Point3D(1, 1, 0));
        aPoles.SetValue(4, new Point3D(0, 1, 0));
        var aKnots = new NCollection_Array1<double>(1, 5);
        var aMults = new NCollection_Array1<int>(1, 5);
        for (int i = 1; i <= 5; i++)
        {
            aKnots.SetValue(i, i - 1);
            aMults.SetValue(i, 1);
        }

        var aCurve = new BSplineCurve3D(aPoles, aKnots, aMults, 2, true);
        aCurve.Freeze();
        var B = new MakeBoundedCurve(aCurve);
        Assert.True(B.IsDone);
        Assert.True(aCurve.IsPeriodic);
        Assert.IsType<StepBSplineCurveWithKnots>(B.Value());
    }

    [Fact]
    public void A_transformation_operator_reads_back_as_the_transformation()
    {
        Transform3D T = new Transform3D().WithRotation(new Axis3D(new Point3D(1, 0, 0), new Direction3D(0, 0, 1)), 0.5);
        Transform3D aTranslation = new Transform3D().WithTranslation(new Vector3D(3, -2, 1));
        T = T.PreMultiplied(aTranslation);

        StepCartesianTransformationOperator3d anOperator = new MakeCartesianTransformationOperator(T).Value();
        Transform3D aBack = new Transform3D();
        Assert.True(global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeTransformation3d(anOperator, ref aBack));
        Point3D P = new Point3D(0.25, 4, -1);
        Assert.True(P.Transformed(T).DistanceTo(P.Transformed(aBack)) < 1e-12);
    }
}
