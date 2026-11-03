// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace OcctNet.DataExchange.Tests.GeomToStep;

/// <summary>Unit tests of GeomToStep (unit 8D); tools/geomsteprec compares it with native OCCT.</summary>
public sealed class GeomToStepTests
{
    private static readonly StepData_Factors Inches = new StepData_Factors().InitializedFactors(25.4, M_PI / 180.0, 1.0);

    private static readonly Frame3D Tilted = new(new Point3D(1, 2, 3), new Direction3D(0, 0.6, 0.8), new Direction3D(1, 0, 0));

    [Fact]
    public void Points_are_divided_by_the_length_factor()
    {
        StepGeom_CartesianPoint P = new GeomToStep_MakeCartesianPoint(new Point3D(25.4, 50.8, -12.7), Inches.LengthFactor()).Value();
        Assert.Equal(3, P.NbCoordinates());
        Assert.Equal([1.0, 2.0, -0.5], P.Coordinates().ToArray());

        StepGeom_CartesianPoint Q = new GeomToStep_MakeCartesianPoint(new Point2D(3, 4), Inches.LengthFactor()).Value();
        Assert.Equal(2, Q.NbCoordinates());
        Assert.Equal(4.0, Q.CoordinatesValue(2));
    }

    [Fact]
    public void A_line_keeps_a_unit_vector_divided_by_the_length_factor()
    {
        StepGeom_Line L = new GeomToStep_MakeLine(new LineCurve3D(new Point3D(0, 0, 25.4), new Direction3D(0, 1, 0)), Inches).Value();
        Assert.Equal(1.0, L.Pnt().CoordinatesValue(3));
        Assert.Equal(1.0 / 25.4, L.Dir().Magnitude());

        // 2D lines take no factor
        StepGeom_Line L2 = new GeomToStep_MakeLine(new LineCurve2D(new Point2D(1, 1), new Direction2D(1, 0)), Inches).Value();
        Assert.Equal(1.0, L2.Dir().Magnitude());
    }

    [Fact]
    public void Curves_and_surfaces_convert_back()
    {
        var aCircle = new CircleCurve3D(Tilted, 7.5);
        var C = new GeomToStep_MakeCurve(aCircle, Inches);
        Assert.True(C.IsDone());
        var aBack = StepToGeom.MakeCurve(C.Value(), Inches) as CircleCurve3D;
        Assert.NotNull(aBack);
        Assert.Equal(7.5, aBack.Radius(), 1e-12);
        Assert.True(aBack.Location().Distance(Tilted.Location()) < 1e-12);

        var aTorus = new ToroidalSurface(new CoordinateSystem(Tilted), 6, 1.5);
        var S = new GeomToStep_MakeSurface(aTorus);
        Assert.True(S.IsDone());
        var aTorusBack = StepToGeom.MakeSurface(S.Value()) as ToroidalSurface;
        Assert.NotNull(aTorusBack);
        Assert.Equal(6, aTorusBack.MajorRadius());
        Assert.Equal(1.5, aTorusBack.MinorRadius());
    }

    [Fact]
    public void A_rectangular_trimmed_surface_is_written_in_degrees()
    {
        var aCylinder = new CylindricalSurface(new CoordinateSystem(Tilted), 2);
        var aTrimmed = new TrimmedSurface(aCylinder, 0, M_PI / 2, -25.4, 50.8);
        StepGeom_RectangularTrimmedSurface T = new GeomToStep_MakeRectangularTrimmedSurface(aTrimmed, Inches).Value();
        Assert.Equal(90.0, T.U2(), 1e-12);
        Assert.Equal(-1.0, T.V1(), 1e-12);
        Assert.Equal(2.0, T.V2(), 1e-12);
        Assert.IsType<StepGeom_CylindricalSurface>(T.BasisSurface());
    }

    [Fact]
    public void Unmapped_geometry_is_not_done()
    {
        var aLine = new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        var anOffset = new OffsetCurve3D(aLine, 1, new Direction3D(0, 0, 1));
        var C = new GeomToStep_MakeCurve(anOffset);
        Assert.False(C.IsDone());
        Assert.Throws<NotDoneException>(() => C.Value());

        // OCCT tests a 2D trimmed curve as a bounded curve first, which MakeBoundedCurve doesn't map
        var aTrimmed2d = new TrimmedCurve2D(new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0)), 0, 1);
        Assert.Throws<NotDoneException>(() => new GeomToStep_MakeCurve(aTrimmed2d));
    }

    [Fact]
    public void A_cone_with_a_negative_semi_angle_is_refused()
    {
        var aCone = new ConicalSurface(new CoordinateSystem(Tilted), -0.3, 2);
        Assert.Throws<DomainErrorException>(() => new GeomToStep_MakeConicalSurface(aCone));
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
        var B = new GeomToStep_MakeBoundedCurve(aCurve);
        Assert.True(B.IsDone());
        Assert.True(aCurve.IsPeriodic);
        Assert.IsType<StepGeom_BSplineCurveWithKnots>(B.Value());
    }

    [Fact]
    public void A_transformation_operator_reads_back_as_the_transformation()
    {
        Transform3D T = new Transform3D();
        T.SetRotation(new Axis3D(new Point3D(1, 0, 0), new Direction3D(0, 0, 1)), 0.5);
        Transform3D aTranslation = new Transform3D();
        aTranslation.SetTranslation(new Vector3D(3, -2, 1));
        T.PreMultiply(aTranslation);

        StepGeom_CartesianTransformationOperator3d anOperator = new GeomToStep_MakeCartesianTransformationOperator(T).Value();
        Transform3D aBack = new Transform3D();
        Assert.True(StepToGeom.MakeTransformation3d(anOperator, ref aBack));
        Point3D P = new Point3D(0.25, 4, -1);
        Assert.True(P.Transformed(T).Distance(P.Transformed(aBack)) < 1e-12);
    }
}
