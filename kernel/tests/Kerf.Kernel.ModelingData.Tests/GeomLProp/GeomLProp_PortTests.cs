// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using static Kerf.Geometry.Continuity;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// The port's own tests of LProp and GeomLProp: the instantiations agree, GeomLProp::Continuity, the surface
/// properties on known surfaces, and OCCT's quirks that the port keeps (occt-issues.md).
/// </summary>
public class GeomLProp_PortTests
{
    private static void SameVec(in Vector3D theA, in Vector3D theB)
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
            Curve3D aCurve = GeomHash_PortTests.Curve(aRand, i % 11);
            var anAdaptor = new CurveAdaptor(aCurve);
            double aFirst = Max(aCurve.FirstParameter(), -10.0), aLast = Min(aCurve.LastParameter(), 10.0);
            for (int j = 0; j <= 4; j++)
            {
                double aU = aFirst + (aLast - aFirst) * j / 4.0;
                int aN = j % 4;
                var aGeom = new global::Kerf.Engine.Geometry.Properties.CLProps(aCurve, aU, aN, 1e-7);
                var anAdapt = new global::Kerf.Engine.Geometry.Properties.CLProps3d(anAdaptor, aU, aN, 1e-7);

                // the adaptor's B-spline and Bezier caches may differ from the curve's evaluation in the last bits
                bool isExact = aCurve is not (BSplineCurve3D or BezierCurve3D or OffsetCurve3D);
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
        var aCircle2d = new global::Kerf.Engine.Geometry.Properties.CLProps2d(new CircleCurve2D(new Axis2D(new Point2D(1, 2), new Direction2D(1, 0)), 3), 0.7, 3, 1e-7);
        var aCircle3d = new global::Kerf.Engine.Geometry.Properties.CLProps(new CircleCurve3D(new Frame3D(new Point3D(1, 2, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 3), 0.7, 3, 1e-7);
        Assert.Equal(aCircle2d.Curvature(), aCircle3d.Curvature());
        aCircle2d.CentreOfCurvature(out Point2D aC2);
        aCircle3d.CentreOfCurvature(out Point3D aC3);
        Assert.Equal(aC2.X(), aC3.X());
        Assert.Equal(aC2.Y(), aC3.Y());
        aCircle2d.Normal(out Direction2D aN2);
        aCircle3d.Normal(out Direction3D aN3);
        Assert.Equal(aN2.X(), aN3.X());
        Assert.Equal(aN2.Y(), aN3.Y());
        Assert.True(aCompared > 2000);
    }

    [Fact]
    public void Tangent_from_a_higher_derivative()
    {
        // a cubic Bezier curve whose first two poles coincide: D1 = 0 at u = 0, so the tangent comes from D2, with the
        // sign of the chord to a nearby point, and the curvature is infinite
        var aPoles = new NCollection_Array1<Point3D>(1, 4);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(0, 0, 0);
        aPoles[3] = new Point3D(1, 1, 0);
        aPoles[4] = new Point3D(2, 0, 0);
        var aCurve = new BezierCurve3D(aPoles);
        var aProps = new global::Kerf.Engine.Geometry.Properties.CLProps(aCurve, 0.0, 2, 1e-7);
        Assert.True(aProps.IsTangentDefined());
        aProps.Tangent(out Direction3D aTangent);
        Assert.Equal(1.0 / Math.Sqrt(2.0), aTangent.X(), 1e-15);
        Assert.Equal(1.0 / Math.Sqrt(2.0), aTangent.Y(), 1e-15);
        Assert.Equal(RealLast(), aProps.Curvature());
        Assert.ThrowsAny<global::Kerf.Engine.Geometry.Properties.NotDefinedException>(() => aProps.Normal(out _));

        // OCCT goes on with the null normal D2 (D1.D1) - D1 (D1.D2) and the stale curvature 0 (occt-issues.md): its
        // release build computes NaN, the port's gp_Vec::Normalize throws
        Assert.ThrowsAny<ConstructionErrorException>(() => aProps.CentreOfCurvature(out _));

        // a point (all derivatives null): no tangent
        var aPoint = new BezierCurve3D(new NCollection_Array1<Point3D>(1, 2));
        var aPointProps = new global::Kerf.Engine.Geometry.Properties.CLProps(aPoint, 0.5, 1, 1e-7);
        Assert.False(aPointProps.IsTangentDefined());
        Assert.ThrowsAny<global::Kerf.Engine.Geometry.Properties.NotDefinedException>(() => aPointProps.Tangent(out _));
        Assert.ThrowsAny<global::Kerf.Engine.Geometry.Properties.NotDefinedException>(() => aPointProps.Curvature());

        Assert.ThrowsAny<OutOfRangeException>(() => new global::Kerf.Engine.Geometry.Properties.CLProps(aCurve, 4, 1e-7));
        Assert.ThrowsAny<OutOfRangeException>(() => new global::Kerf.Engine.Geometry.Properties.SLProps(3, 1e-7));
    }

    [Fact]
    public void Continuity_of_joined_curves()
    {
        var aLine1 = new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        var aLine2 = new LineCurve3D(new Point3D(5, 0, 0), new Direction3D(1, 0, 0));
        Assert.Equal(C2, global::Kerf.Engine.Geometry.Properties.GeomLProp.Continuity(aLine1, aLine2, 5.0, 0.0, false, false));
        Assert.Equal(C0, global::Kerf.Engine.Geometry.Properties.GeomLProp.Continuity(aLine1, aLine2, 5.0, 0.0, true, false));
        Assert.Equal(C2, global::Kerf.Engine.Geometry.Properties.GeomLProp.Continuity(aLine1, aLine2, 5.0, 0.0, true, true));

        // same direction, different speed: G1
        var aCircle = new CircleCurve3D(new Frame3D(new Point3D(0, 2, 0), new Direction3D(0, 0, 1)), 2.0);
        Assert.Equal(G1, global::Kerf.Engine.Geometry.Properties.GeomLProp.Continuity(aLine1, aCircle, 0.0, -Math.PI / 2, false, false));

        // at an angle: C0
        var aLine3 = new LineCurve3D(new Point3D(5, 0, 0), new Direction3D(0, 1, 0));
        Assert.Equal(C0, global::Kerf.Engine.Geometry.Properties.GeomLProp.Continuity(aLine1, aLine3, 5.0, 0.0, false, false));

        Assert.ThrowsAny<FailureException>(() => global::Kerf.Engine.Geometry.Properties.GeomLProp.Continuity(aLine1, aLine3, 4.0, 0.0, false, false));
    }

    private static BSplineCurve3D CubicWithKnot(int theMult)
    {
        int aNbPoles = 4 + theMult;
        var aPoles = new NCollection_Array1<Point3D>(1, aNbPoles);
        for (int i = 1; i <= aNbPoles; i++)
        {
            aPoles[i] = new Point3D(i, Math.Sin(i), 0.1 * i * i);
        }

        var aKnots = new NCollection_Array1<double>(1, 3);
        aKnots[1] = 0.0;
        aKnots[2] = 0.5;
        aKnots[3] = 1.0;
        var aMults = new NCollection_Array1<int>(1, 3);
        aMults[1] = 4;
        aMults[2] = theMult;
        aMults[3] = 4;
        return new BSplineCurve3D(aPoles, aKnots, aMults, 3);
    }

    /// <summary>
    /// At an inner knot a cubic is C(3 - multiplicity): the junction of the curve with itself there is reported so
    /// (OCCT compared Degree - Multiplicity with its own scale, on which 2 is C1 and 4 is C2, and reported a C2 knot C1
    /// at best, a C1 knot C0: occt-issues.md, decision 16708).
    /// </summary>
    [Fact]
    public void Continuity_at_a_BSpline_knot_is_the_knots()
    {
        // a cubic with a simple knot is C2 there
        BSplineCurve3D aC2 = CubicWithKnot(1);
        Assert.Equal(C2, global::Kerf.Engine.Geometry.Properties.GeomLProp.Continuity(aC2, aC2, 0.5, 0.5, false, false));

        // with a double knot, C1
        BSplineCurve3D aC1 = CubicWithKnot(2);
        Assert.Equal(C1, global::Kerf.Engine.Geometry.Properties.GeomLProp.Continuity(aC1, aC1, 0.5, 0.5, false, false));

        // with a triple knot, C0
        BSplineCurve3D aC0 = CubicWithKnot(3);
        Assert.Equal(C0, global::Kerf.Engine.Geometry.Properties.GeomLProp.Continuity(aC0, aC0, 0.5, 0.5, false, false));

        // away from the knot (not a B-spline knot: cont 5, n 3) both are C2
        Assert.Equal(C2, global::Kerf.Engine.Geometry.Properties.GeomLProp.Continuity(aC1, aC1, 0.25, 0.25, false, false));
    }

    [Fact]
    public void Surface_curvatures_of_known_surfaces()
    {
        var anAx3 = new CoordinateSystem(new Point3D(1, 2, 3), new Direction3D(0, 0, 1));
        var aSphere = new global::Kerf.Engine.Geometry.Properties.SLProps(new SphericalSurface(anAx3, 2.0), 0.4, 0.3, 2, 1e-7);
        Assert.True(aSphere.IsCurvatureDefined());
        Assert.Equal(-0.5, aSphere.MaxCurvature(), 1e-15);
        Assert.Equal(-0.5, aSphere.MinCurvature(), 1e-15);
        Assert.Equal(0.25, aSphere.GaussianCurvature(), 1e-15);

        var aCylinder = new global::Kerf.Engine.Geometry.Properties.SLProps(new CylindricalSurface(anAx3, 4.0), 1.0, 5.0, 2, 1e-7);
        Assert.True(aCylinder.IsCurvatureDefined());
        Assert.Equal(0.0, aCylinder.MaxCurvature(), 1e-15);
        Assert.Equal(-0.25, aCylinder.MinCurvature(), 1e-15);
        Assert.Equal(-0.125, aCylinder.MeanCurvature(), 1e-15);
        Assert.Equal(0.0, aCylinder.GaussianCurvature(), 1e-15);
        Assert.False(aCylinder.IsUmbilic());
        aCylinder.CurvatureDirections(out Direction3D aMax, out Direction3D aMin);
        Assert.Equal(1.0, Math.Abs(aMax.Z()), 1e-15);
        Assert.Equal(0.0, aMin.Z(), 1e-15);
        Assert.Equal(new Direction3D(Math.Cos(1.0), Math.Sin(1.0), 0.0).X(), aCylinder.Normal().X(), 1e-15);

        // a cone at its apex: no normal, no curvature
        var aCone = new global::Kerf.Engine.Geometry.Properties.SLProps(new ConicalSurface(anAx3, 0.5, 0.0), 0.0, 0.0, 2, 1e-7);
        Assert.False(aCone.IsNormalDefined);
        Assert.False(aCone.IsCurvatureDefined());
        Assert.ThrowsAny<global::Kerf.Engine.Geometry.Properties.NotDefinedException>(() => aCone.Normal());
        Assert.ThrowsAny<global::Kerf.Engine.Geometry.Properties.NotDefinedException>(() => aCone.MaxCurvature());

        // the adaptor instantiation agrees on the sphere
        var anAdapted = new global::Kerf.Engine.Geometry.Properties.SLProps3d(new SurfaceAdaptor(new SphericalSurface(anAx3, 2.0)), 0.4, 0.3, 2, 1e-7);
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
        var aSphere = new SphericalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 2.0);
        var aProps = new global::Kerf.Engine.Geometry.Properties.SLProps(aSphere, 0.4, 0.3, 2, 1e-7);
        Assert.Equal(0.0, aProps.MaxCurvature());
        Assert.Equal(-0.5, aProps.MaxCurvature(), 1e-15);

        var aCylinder = new CylindricalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 4.0);
        var aCylProps = new global::Kerf.Engine.Geometry.Properties.SLProps(aCylinder, 0.4, 0.3, 2, 1e-7);
        Assert.True(aCylProps.IsUmbilic()); // compares 0 and 0
        Assert.False(aCylProps.IsUmbilic());
        aCylProps.SetParameters(1.0, 1.0);
        Assert.Equal(-0.25, aCylProps.MinCurvature(), 1e-15); // the previous parameters' value, the same here
    }
}
