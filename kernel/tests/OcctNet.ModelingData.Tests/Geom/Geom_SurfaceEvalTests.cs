// Port of OCCT's src/ModelingData/TKG3d/GTests/Geom_SurfaceEval_Test.cxx (V8_0_1).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// The EvalD* methods of the Geom surfaces.
/// </summary>
public class Geom_SurfaceEvalTests
{
    [Fact]
    public void Plane_EvalD0_ReturnsValidPoint()
    {
        Geom_Plane aSurf = new(new gp_Pln(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0)));

        gp_Pnt aResult = aSurf.EvalD0(1.0, 2.0);

        // For a standard XY plane, the point should be at (U, V, 0)
        Assert.Equal(1.0, aResult.X(), Precision.Confusion());
        Assert.Equal(2.0, aResult.Y(), Precision.Confusion());
        Assert.Equal(0.0, aResult.Z(), Precision.Confusion());
    }

    [Fact]
    public void Plane_EvalD1_ConstantPartials()
    {
        Geom_Plane aSurf = new(new gp_Pln(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0)));

        Geom_Surface.ResD1 aD1 = aSurf.EvalD1(1.0, 2.0);

        // Partial derivatives of a plane are constant
        Assert.Equal(1.0, aD1.D1U.Magnitude(), Precision.Confusion());
        Assert.Equal(1.0, aD1.D1V.Magnitude(), Precision.Confusion());
        Assert.Equal(0.0, aD1.D1U.Dot(aD1.D1V), Precision.Confusion());
    }

    [Fact]
    public void Sphere_EvalD1_PartialDerivatives()
    {
        double aRadius = 5.0;
        gp_Sphere aSphere = new(new gp_Ax3(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0)), aRadius);
        Geom_SphericalSurface aSurf = new(aSphere);

        double aU = M_PI / 4.0; // longitude
        double aV = M_PI / 6.0; // latitude

        Geom_Surface.ResD1 aD1 = aSurf.EvalD1(aU, aV);

        // Point should lie on sphere of given radius
        double aDist = aD1.Point.Distance(new gp_Pnt(0.0, 0.0, 0.0));
        Assert.Equal(aRadius, aDist, Precision.Confusion());

        // D1U and D1V should be non-zero and perpendicular on a sphere (at non-pole points)
        Assert.True(aD1.D1U.Magnitude() > Precision.Confusion());
        Assert.True(aD1.D1V.Magnitude() > Precision.Confusion());
    }

    [Fact]
    public void BSplineSurface_EvalD2_ConsistentWithOldAPI()
    {
        // Create a simple BSpline surface - a flat quad
        NCollection_Array2<gp_Pnt> aPoles = new(1, 3, 1, 3);
        for (int i = 1; i <= 3; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                aPoles[i, j] = new gp_Pnt((double)(i - 1), (double)(j - 1), 0.1 * (double)((i - 2) * (j - 2)));
            }
        }

        NCollection_Array1<double> aUKnots = new(1, 2);
        aUKnots[1] = 0.0;
        aUKnots[2] = 1.0;
        NCollection_Array1<double> aVKnots = new(1, 2);
        aVKnots[1] = 0.0;
        aVKnots[2] = 1.0;
        NCollection_Array1<int> aUMults = new(1, 2);
        aUMults[1] = 3;
        aUMults[2] = 3;
        NCollection_Array1<int> aVMults = new(1, 2);
        aVMults[1] = 3;
        aVMults[2] = 3;

        Geom_BSplineSurface aSurf = new(aPoles, aUKnots, aVKnots, aUMults, aVMults, 2, 2);

        double aU = 0.5;
        double aV = 0.5;

        Geom_Surface.ResD2 aEvalResult = aSurf.EvalD2(aU, aV);

        aSurf.D2(aU, aV, out gp_Pnt aOldP, out gp_Vec aOldD1U, out gp_Vec aOldD1V, out gp_Vec aOldD2U, out gp_Vec aOldD2V, out gp_Vec aOldD2UV);

        Assert.Equal(0.0, aEvalResult.Point.Distance(aOldP), Precision.Confusion());
        Assert.Equal(0.0, (aEvalResult.D1U - aOldD1U).Magnitude(), Precision.Confusion());
        Assert.Equal(0.0, (aEvalResult.D1V - aOldD1V).Magnitude(), Precision.Confusion());
        Assert.Equal(0.0, (aEvalResult.D2U - aOldD2U).Magnitude(), Precision.Confusion());
        Assert.Equal(0.0, (aEvalResult.D2V - aOldD2V).Magnitude(), Precision.Confusion());
        Assert.Equal(0.0, (aEvalResult.D2UV - aOldD2UV).Magnitude(), Precision.Confusion());
    }

    [Fact]
    public void CylindricalSurface_EvalD1_ValidResults()
    {
        double aRadius = 4.0;
        Geom_CylindricalSurface aSurf = new(new gp_Ax3(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0)), aRadius);

        double aU = M_PI / 4.0;
        double aV = 3.0;

        Geom_Surface.ResD1 aD1 = aSurf.EvalD1(aU, aV);

        // Point should lie on cylinder of given radius at height V
        double aDistXY = Math.Sqrt(aD1.Point.X() * aD1.Point.X() + aD1.Point.Y() * aD1.Point.Y());
        Assert.Equal(aRadius, aDistXY, Precision.Confusion());
        Assert.Equal(aV, aD1.Point.Z(), Precision.Confusion());

        // D1U should be tangent to the circle (perpendicular to radial direction in XY)
        Assert.True(aD1.D1U.Magnitude() > Precision.Confusion());
        // D1V should be along the axis (Z direction)
        Assert.Equal(0.0, aD1.D1V.X(), Precision.Confusion());
        Assert.Equal(0.0, aD1.D1V.Y(), Precision.Confusion());
        Assert.Equal(1.0, aD1.D1V.Z(), Precision.Confusion());
    }

    [Fact]
    public void BezierSurface_EvalD2_ConsistentWithOldAPI()
    {
        // Create a simple 3x3 Bezier surface with some curvature
        NCollection_Array2<gp_Pnt> aPoles = new(1, 3, 1, 3);
        aPoles[1, 1] = new gp_Pnt(0.0, 0.0, 0.0);
        aPoles[1, 2] = new gp_Pnt(0.0, 1.0, 0.0);
        aPoles[1, 3] = new gp_Pnt(0.0, 2.0, 0.0);
        aPoles[2, 1] = new gp_Pnt(1.0, 0.0, 0.5);
        aPoles[2, 2] = new gp_Pnt(1.0, 1.0, 1.0);
        aPoles[2, 3] = new gp_Pnt(1.0, 2.0, 0.5);
        aPoles[3, 1] = new gp_Pnt(2.0, 0.0, 0.0);
        aPoles[3, 2] = new gp_Pnt(2.0, 1.0, 0.0);
        aPoles[3, 3] = new gp_Pnt(2.0, 2.0, 0.0);

        Geom_BezierSurface aSurf = new(aPoles);

        double aU = 0.3;
        double aV = 0.7;

        Geom_Surface.ResD2 aEvalResult = aSurf.EvalD2(aU, aV);

        aSurf.D2(aU, aV, out gp_Pnt aOldP, out gp_Vec aOldD1U, out gp_Vec aOldD1V, out gp_Vec aOldD2U, out gp_Vec aOldD2V, out gp_Vec aOldD2UV);

        Assert.Equal(0.0, aEvalResult.Point.Distance(aOldP), Precision.Confusion());
        Assert.Equal(0.0, (aEvalResult.D1U - aOldD1U).Magnitude(), Precision.Confusion());
        Assert.Equal(0.0, (aEvalResult.D1V - aOldD1V).Magnitude(), Precision.Confusion());
        Assert.Equal(0.0, (aEvalResult.D2U - aOldD2U).Magnitude(), Precision.Confusion());
        Assert.Equal(0.0, (aEvalResult.D2V - aOldD2V).Magnitude(), Precision.Confusion());
        Assert.Equal(0.0, (aEvalResult.D2UV - aOldD2UV).Magnitude(), Precision.Confusion());
    }

    [Fact]
    public void ExtrusionSurface_EvalD0_ValidResults()
    {
        // Extrude a circle along Z axis
        gp_Circ aCirc = new(new gp_Ax2(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0)), 3.0);
        Geom_Circle aBasis = new(aCirc);

        Geom_SurfaceOfLinearExtrusion aSurf = new(aBasis, new gp_Dir(0.0, 0.0, 1.0));

        double aU = M_PI / 3.0;
        double aV = 5.0;

        gp_Pnt aResult = aSurf.EvalD0(aU, aV);

        // Point should lie on circle of radius 3 at height V
        double aDistXY = Math.Sqrt(aResult.X() * aResult.X() + aResult.Y() * aResult.Y());
        Assert.Equal(3.0, aDistXY, Precision.Confusion());
        Assert.Equal(aV, aResult.Z(), Precision.Confusion());
    }

    [Fact]
    public void RevolutionSurface_EvalD1_ValidResults()
    {
        // Revolve a line around Z axis to create a cone-like surface
        Geom_Line aGeneratrix = new(new gp_Pnt(1.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0));

        gp_Ax1 anAxis = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0));
        Geom_SurfaceOfRevolution aSurf = new(aGeneratrix, anAxis);

        double aU = M_PI / 4.0;
        double aV = 2.0;

        Geom_Surface.ResD1 aD1 = aSurf.EvalD1(aU, aV);

        // Point should lie on a circle of radius 1 at height V
        double aDistXY = Math.Sqrt(aD1.Point.X() * aD1.Point.X() + aD1.Point.Y() * aD1.Point.Y());
        Assert.Equal(1.0, aDistXY, Precision.Confusion());
        Assert.Equal(aV, aD1.Point.Z(), Precision.Confusion());

        // D1U should be tangential (circumferential direction)
        Assert.True(aD1.D1U.Magnitude() > Precision.Confusion());
        // D1V should be along the axis (Z direction) since generatrix is vertical
        Assert.Equal(0.0, aD1.D1V.X(), Precision.Confusion());
        Assert.Equal(0.0, aD1.D1V.Y(), Precision.Confusion());
        Assert.Equal(1.0, aD1.D1V.Z(), Precision.Confusion());
    }

    [Fact]
    public void OffsetSurface_EvalD0_ValidOnRegularSurface()
    {
        // Create a plane offset by a distance
        Geom_Plane aPlane = new(new gp_Pln(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0)));

        Geom_OffsetSurface anOffset = new(aPlane, 3.0);

        gp_Pnt aResult = anOffset.EvalD0(1.0, 2.0);

        // Offset plane at distance 3 in Z direction
        Assert.Equal(1.0, aResult.X(), Precision.Confusion());
        Assert.Equal(2.0, aResult.Y(), Precision.Confusion());
        Assert.Equal(3.0, aResult.Z(), Precision.Confusion());
    }
}
