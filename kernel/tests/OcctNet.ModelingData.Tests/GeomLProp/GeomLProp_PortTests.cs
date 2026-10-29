// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// The port's own tests of LProp and GeomLProp: the instantiations agree, GeomLProp::Continuity, the surface
/// properties on known surfaces, and OCCT's quirks that the port keeps (occt-issues.md).
/// </summary>
public class GeomLProp_PortTests
{
    private static void SameVec(in gp_Vec theA, in gp_Vec theB)
    {
        Assert.Equal(theA.X(), theB.X());
        Assert.Equal(theA.Y(), theB.Y());
        Assert.Equal(theA.Z(), theB.Z());
    }

    [Fact]
    public void The_three_curve_instantiations_agree()
    {
        var aRand = new Random(20261029);
        int aCompared = 0;
        for (int i = 0; i < 440; i++)
        {
            Geom_Curve aCurve = GeomHash_PortTests.Curve(aRand, i % 11);
            var anAdaptor = new GeomAdaptor_Curve(aCurve);
            double aFirst = Max(aCurve.FirstParameter(), -10.0), aLast = Min(aCurve.LastParameter(), 10.0);
            for (int j = 0; j <= 4; j++)
            {
                double aU = aFirst + (aLast - aFirst) * j / 4.0;
                int aN = j % 4;
                var aGeom = new GeomLProp_CLProps(aCurve, aU, aN, 1e-7);
                var anAdapt = new LProp_CLProps3d(anAdaptor, aU, aN, 1e-7);

                // the adaptor's B-spline and Bezier caches may differ from the curve's evaluation in the last bits
                bool isExact = aCurve is not (Geom_BSplineCurve or Geom_BezierCurve or Geom_OffsetCurve);
                Assert.Equal(aGeom.IsTangentDefined(), anAdapt.IsTangentDefined());
                if (isExact)
                {
                    Assert.Equal(aGeom.Value(), anAdapt.Value());
                    SameVec(aGeom.D1(), anAdapt.D1());
                    SameVec(aGeom.D3(), anAdapt.D3());
                    Assert.Equal(aGeom.Curvature(), anAdapt.Curvature());
                }
                else
                {
                    Assert.Equal(aGeom.Curvature(), anAdapt.Curvature(), 1e-9 * Max(1.0, Math.Abs(aGeom.Curvature())));
                }

                aCompared++;
            }
        }

        // 2D: a planar 3D curve and the same 2D curve
        var aCircle2d = new GeomLProp_CLProps2d(new Geom2d_Circle(new gp_Ax2d(new gp_Pnt2d(1, 2), new gp_Dir2d(1, 0)), 3), 0.7, 3, 1e-7);
        var aCircle3d = new GeomLProp_CLProps(new Geom_Circle(new gp_Ax2(new gp_Pnt(1, 2, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 3), 0.7, 3, 1e-7);
        Assert.Equal(aCircle2d.Curvature(), aCircle3d.Curvature());
        aCircle2d.CentreOfCurvature(out gp_Pnt2d aC2);
        aCircle3d.CentreOfCurvature(out gp_Pnt aC3);
        Assert.Equal(aC2.X(), aC3.X());
        Assert.Equal(aC2.Y(), aC3.Y());
        aCircle2d.Normal(out gp_Dir2d aN2);
        aCircle3d.Normal(out gp_Dir aN3);
        Assert.Equal(aN2.X(), aN3.X());
        Assert.Equal(aN2.Y(), aN3.Y());
        Assert.True(aCompared > 2000);
    }

    [Fact]
    public void Tangent_from_a_higher_derivative()
    {
        // a cubic Bezier curve whose first two poles coincide: D1 = 0 at u = 0, so the tangent comes from D2, with the
        // sign of the chord to a nearby point, and the curvature is infinite
        var aPoles = new NCollection_Array1<gp_Pnt>(1, 4);
        aPoles[1] = new gp_Pnt(0, 0, 0);
        aPoles[2] = new gp_Pnt(0, 0, 0);
        aPoles[3] = new gp_Pnt(1, 1, 0);
        aPoles[4] = new gp_Pnt(2, 0, 0);
        var aCurve = new Geom_BezierCurve(aPoles);
        var aProps = new GeomLProp_CLProps(aCurve, 0.0, 2, 1e-7);
        Assert.True(aProps.IsTangentDefined());
        aProps.Tangent(out gp_Dir aTangent);
        Assert.Equal(1.0 / Math.Sqrt(2.0), aTangent.X(), 1e-15);
        Assert.Equal(1.0 / Math.Sqrt(2.0), aTangent.Y(), 1e-15);
        Assert.Equal(RealLast(), aProps.Curvature());
        Assert.ThrowsAny<LProp_NotDefined>(() => aProps.Normal(out _));

        // OCCT goes on with the null normal D2 (D1.D1) - D1 (D1.D2) and the stale curvature 0 (occt-issues.md): its
        // release build computes NaN, the port's gp_Vec::Normalize throws
        Assert.ThrowsAny<Standard_ConstructionError>(() => aProps.CentreOfCurvature(out _));

        // a point (all derivatives null): no tangent
        var aPoint = new Geom_BezierCurve(new NCollection_Array1<gp_Pnt>(1, 2));
        var aPointProps = new GeomLProp_CLProps(aPoint, 0.5, 1, 1e-7);
        Assert.False(aPointProps.IsTangentDefined());
        Assert.ThrowsAny<LProp_NotDefined>(() => aPointProps.Tangent(out _));
        Assert.ThrowsAny<LProp_NotDefined>(() => aPointProps.Curvature());

        Assert.ThrowsAny<Standard_OutOfRange>(() => new GeomLProp_CLProps(aCurve, 4, 1e-7));
        Assert.ThrowsAny<Standard_OutOfRange>(() => new GeomLProp_SLProps(3, 1e-7));
    }

    [Fact]
    public void Continuity_of_joined_curves()
    {
        var aLine1 = new Geom_Line(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0));
        var aLine2 = new Geom_Line(new gp_Pnt(5, 0, 0), new gp_Dir(1, 0, 0));
        Assert.Equal(GeomAbs_C2, GeomLProp.Continuity(aLine1, aLine2, 5.0, 0.0, false, false));
        Assert.Equal(GeomAbs_C0, GeomLProp.Continuity(aLine1, aLine2, 5.0, 0.0, true, false));
        Assert.Equal(GeomAbs_C2, GeomLProp.Continuity(aLine1, aLine2, 5.0, 0.0, true, true));

        // same direction, different speed: G1
        var aCircle = new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 2, 0), new gp_Dir(0, 0, 1)), 2.0);
        Assert.Equal(GeomAbs_G1, GeomLProp.Continuity(aLine1, aCircle, 0.0, -Math.PI / 2, false, false));

        // at an angle: C0
        var aLine3 = new Geom_Line(new gp_Pnt(5, 0, 0), new gp_Dir(0, 1, 0));
        Assert.Equal(GeomAbs_C0, GeomLProp.Continuity(aLine1, aLine3, 5.0, 0.0, false, false));

        Assert.ThrowsAny<Standard_Failure>(() => GeomLProp.Continuity(aLine1, aLine3, 4.0, 0.0, false, false));
    }

    private static Geom_BSplineCurve CubicWithKnot(int theMult)
    {
        int aNbPoles = 4 + theMult;
        var aPoles = new NCollection_Array1<gp_Pnt>(1, aNbPoles);
        for (int i = 1; i <= aNbPoles; i++)
        {
            aPoles[i] = new gp_Pnt(i, Math.Sin(i), 0.1 * i * i);
        }

        var aKnots = new NCollection_Array1<double>(1, 3);
        aKnots[1] = 0.0;
        aKnots[2] = 0.5;
        aKnots[3] = 1.0;
        var aMults = new NCollection_Array1<int>(1, 3);
        aMults[1] = 4;
        aMults[2] = theMult;
        aMults[3] = 4;
        return new Geom_BSplineCurve(aPoles, aKnots, aMults, 3);
    }

    /// <summary>
    /// OCCT compares Degree - Multiplicity (the order of continuity at a knot) with its own scale, on which 2 is C1 and
    /// 4 is C2 (occt-issues.md): a C2 knot is reported C1 at best, a C1 knot C0.
    /// </summary>
    [Fact]
    public void Continuity_at_a_BSpline_knot_is_reported_lower_as_in_OCCT()
    {
        // a cubic with a simple knot is C2 there: OCCT compares first derivatives only
        Geom_BSplineCurve aC2 = CubicWithKnot(1);
        Assert.Equal(GeomAbs_C1, GeomLProp.Continuity(aC2, aC2, 0.5, 0.5, false, false));

        // with a double knot it is C1: OCCT compares nothing
        Geom_BSplineCurve aC1 = CubicWithKnot(2);
        Assert.Equal(GeomAbs_C0, GeomLProp.Continuity(aC1, aC1, 0.5, 0.5, false, false));

        // away from the knot (not a B-spline knot: cont 5, n 3) both are C2
        Assert.Equal(GeomAbs_C2, GeomLProp.Continuity(aC1, aC1, 0.25, 0.25, false, false));
    }

    [Fact]
    public void Surface_curvatures_of_known_surfaces()
    {
        var anAx3 = new gp_Ax3(new gp_Pnt(1, 2, 3), new gp_Dir(0, 0, 1));
        var aSphere = new GeomLProp_SLProps(new Geom_SphericalSurface(anAx3, 2.0), 0.4, 0.3, 2, 1e-7);
        Assert.True(aSphere.IsCurvatureDefined());
        Assert.Equal(-0.5, aSphere.MaxCurvature(), 1e-15);
        Assert.Equal(-0.5, aSphere.MinCurvature(), 1e-15);
        Assert.Equal(0.25, aSphere.GaussianCurvature(), 1e-15);

        var aCylinder = new GeomLProp_SLProps(new Geom_CylindricalSurface(anAx3, 4.0), 1.0, 5.0, 2, 1e-7);
        Assert.True(aCylinder.IsCurvatureDefined());
        Assert.Equal(0.0, aCylinder.MaxCurvature(), 1e-15);
        Assert.Equal(-0.25, aCylinder.MinCurvature(), 1e-15);
        Assert.Equal(-0.125, aCylinder.MeanCurvature(), 1e-15);
        Assert.Equal(0.0, aCylinder.GaussianCurvature(), 1e-15);
        Assert.False(aCylinder.IsUmbilic());
        aCylinder.CurvatureDirections(out gp_Dir aMax, out gp_Dir aMin);
        Assert.Equal(1.0, Math.Abs(aMax.Z()), 1e-15);
        Assert.Equal(0.0, aMin.Z(), 1e-15);
        Assert.Equal(new gp_Dir(Math.Cos(1.0), Math.Sin(1.0), 0.0).X(), aCylinder.Normal().X(), 1e-15);

        // a cone at its apex: no normal, no curvature
        var aCone = new GeomLProp_SLProps(new Geom_ConicalSurface(anAx3, 0.5, 0.0), 0.0, 0.0, 2, 1e-7);
        Assert.False(aCone.IsNormalDefined());
        Assert.False(aCone.IsCurvatureDefined());
        Assert.ThrowsAny<LProp_NotDefined>(() => aCone.Normal());
        Assert.ThrowsAny<LProp_NotDefined>(() => aCone.MaxCurvature());

        // the adaptor instantiation agrees on the sphere
        var anAdapted = new LProp_SLProps3d(new GeomAdaptor_Surface(new Geom_SphericalSurface(anAx3, 2.0)), 0.4, 0.3, 2, 1e-7);
        Assert.True(anAdapted.IsCurvatureDefined());
        Assert.Equal(aSphere.MeanCurvature(), anAdapted.MeanCurvature());
        Assert.Equal(aSphere.Normal(), anAdapted.Normal());
    }

    /// <summary>
    /// OCCT passes the curvature fields by value before computing them (occt-issues.md): asked before
    /// IsCurvatureDefined, MaxCurvature and the others return the previous value, 0 on a new object, and IsUmbilic
    /// compares the previous values. The oracle tests confirm it against native OCCT.
    /// </summary>
    [Fact]
    public void Curvatures_asked_first_return_the_previous_value_as_in_OCCT()
    {
        var aSphere = new Geom_SphericalSurface(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 2.0);
        var aProps = new GeomLProp_SLProps(aSphere, 0.4, 0.3, 2, 1e-7);
        Assert.Equal(0.0, aProps.MaxCurvature());
        Assert.Equal(-0.5, aProps.MaxCurvature(), 1e-15);

        var aCylinder = new Geom_CylindricalSurface(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 4.0);
        var aCylProps = new GeomLProp_SLProps(aCylinder, 0.4, 0.3, 2, 1e-7);
        Assert.True(aCylProps.IsUmbilic()); // compares 0 and 0
        Assert.False(aCylProps.IsUmbilic());
        aCylProps.SetParameters(1.0, 1.0);
        Assert.Equal(-0.25, aCylProps.MinCurvature(), 1e-15); // the previous parameters' value, the same here
    }
}
