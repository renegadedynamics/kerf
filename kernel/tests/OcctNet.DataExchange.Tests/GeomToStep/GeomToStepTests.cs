// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.DataExchange.Tests.GeomToStep;

/// <summary>Unit tests of GeomToStep (unit 8D); tools/geomsteprec compares it with native OCCT.</summary>
public sealed class GeomToStepTests
{
    private static readonly StepData_Factors Inches = new StepData_Factors().InitializedFactors(25.4, M_PI / 180.0, 1.0);

    private static readonly gp_Ax2 Tilted = new(new gp_Pnt(1, 2, 3), new gp_Dir(0, 0.6, 0.8), new gp_Dir(1, 0, 0));

    [Fact]
    public void Points_are_divided_by_the_length_factor()
    {
        StepGeom_CartesianPoint P = new GeomToStep_MakeCartesianPoint(new gp_Pnt(25.4, 50.8, -12.7), Inches.LengthFactor()).Value();
        Assert.Equal(3, P.NbCoordinates());
        Assert.Equal([1.0, 2.0, -0.5], P.Coordinates().ToArray());

        StepGeom_CartesianPoint Q = new GeomToStep_MakeCartesianPoint(new gp_Pnt2d(3, 4), Inches.LengthFactor()).Value();
        Assert.Equal(2, Q.NbCoordinates());
        Assert.Equal(4.0, Q.CoordinatesValue(2));
    }

    [Fact]
    public void A_line_keeps_a_unit_vector_divided_by_the_length_factor()
    {
        StepGeom_Line L = new GeomToStep_MakeLine(new Geom_Line(new gp_Pnt(0, 0, 25.4), new gp_Dir(0, 1, 0)), Inches).Value();
        Assert.Equal(1.0, L.Pnt().CoordinatesValue(3));
        Assert.Equal(1.0 / 25.4, L.Dir().Magnitude());

        // 2D lines take no factor
        StepGeom_Line L2 = new GeomToStep_MakeLine(new Geom2d_Line(new gp_Pnt2d(1, 1), new gp_Dir2d(1, 0)), Inches).Value();
        Assert.Equal(1.0, L2.Dir().Magnitude());
    }

    [Fact]
    public void Curves_and_surfaces_convert_back()
    {
        var aCircle = new Geom_Circle(Tilted, 7.5);
        var C = new GeomToStep_MakeCurve(aCircle, Inches);
        Assert.True(C.IsDone());
        var aBack = StepToGeom.MakeCurve(C.Value(), Inches) as Geom_Circle;
        Assert.NotNull(aBack);
        Assert.Equal(7.5, aBack.Radius(), 1e-12);
        Assert.True(aBack.Location().Distance(Tilted.Location()) < 1e-12);

        var aTorus = new Geom_ToroidalSurface(new gp_Ax3(Tilted), 6, 1.5);
        var S = new GeomToStep_MakeSurface(aTorus);
        Assert.True(S.IsDone());
        var aTorusBack = StepToGeom.MakeSurface(S.Value()) as Geom_ToroidalSurface;
        Assert.NotNull(aTorusBack);
        Assert.Equal(6, aTorusBack.MajorRadius());
        Assert.Equal(1.5, aTorusBack.MinorRadius());
    }

    [Fact]
    public void A_rectangular_trimmed_surface_is_written_in_degrees()
    {
        var aCylinder = new Geom_CylindricalSurface(new gp_Ax3(Tilted), 2);
        var aTrimmed = new Geom_RectangularTrimmedSurface(aCylinder, 0, M_PI / 2, -25.4, 50.8);
        StepGeom_RectangularTrimmedSurface T = new GeomToStep_MakeRectangularTrimmedSurface(aTrimmed, Inches).Value();
        Assert.Equal(90.0, T.U2(), 1e-12);
        Assert.Equal(-1.0, T.V1(), 1e-12);
        Assert.Equal(2.0, T.V2(), 1e-12);
        Assert.IsType<StepGeom_CylindricalSurface>(T.BasisSurface());
    }

    [Fact]
    public void Unmapped_geometry_is_not_done()
    {
        var aLine = new Geom_Line(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0));
        var anOffset = new Geom_OffsetCurve(aLine, 1, new gp_Dir(0, 0, 1));
        var C = new GeomToStep_MakeCurve(anOffset);
        Assert.False(C.IsDone());
        Assert.Throws<StdFail_NotDone>(() => C.Value());

        // OCCT tests a 2D trimmed curve as a bounded curve first, which MakeBoundedCurve doesn't map
        var aTrimmed2d = new Geom2d_TrimmedCurve(new Geom2d_Line(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), 0, 1);
        Assert.Throws<StdFail_NotDone>(() => new GeomToStep_MakeCurve(aTrimmed2d));
    }

    [Fact]
    public void A_cone_with_a_negative_semi_angle_is_refused()
    {
        var aCone = new Geom_ConicalSurface(new gp_Ax3(Tilted), -0.3, 2);
        Assert.Throws<Standard_DomainError>(() => new GeomToStep_MakeConicalSurface(aCone));
    }

    [Fact]
    public void Frozen_periodic_input_is_converted_on_a_copy()
    {
        var aPoles = new NCollection_Array1<gp_Pnt>(1, 4);
        aPoles.SetValue(1, new gp_Pnt(0, 0, 0));
        aPoles.SetValue(2, new gp_Pnt(1, 0, 0));
        aPoles.SetValue(3, new gp_Pnt(1, 1, 0));
        aPoles.SetValue(4, new gp_Pnt(0, 1, 0));
        var aKnots = new NCollection_Array1<double>(1, 5);
        var aMults = new NCollection_Array1<int>(1, 5);
        for (int i = 1; i <= 5; i++)
        {
            aKnots.SetValue(i, i - 1);
            aMults.SetValue(i, 1);
        }

        var aCurve = new Geom_BSplineCurve(aPoles, aKnots, aMults, 2, true);
        aCurve.Freeze();
        var B = new GeomToStep_MakeBoundedCurve(aCurve);
        Assert.True(B.IsDone());
        Assert.True(aCurve.IsPeriodic());
        Assert.IsType<StepGeom_BSplineCurveWithKnots>(B.Value());
    }

    [Fact]
    public void A_transformation_operator_reads_back_as_the_transformation()
    {
        gp_Trsf T = new gp_Trsf();
        T.SetRotation(new gp_Ax1(new gp_Pnt(1, 0, 0), new gp_Dir(0, 0, 1)), 0.5);
        gp_Trsf aTranslation = new gp_Trsf();
        aTranslation.SetTranslation(new gp_Vec(3, -2, 1));
        T.PreMultiply(aTranslation);

        StepGeom_CartesianTransformationOperator3d anOperator = new GeomToStep_MakeCartesianTransformationOperator(T).Value();
        gp_Trsf aBack = new gp_Trsf();
        Assert.True(StepToGeom.MakeTransformation3d(anOperator, ref aBack));
        gp_Pnt P = new gp_Pnt(0.25, 4, -1);
        Assert.True(P.Transformed(T).Distance(P.Transformed(aBack)) < 1e-12);
    }
}
