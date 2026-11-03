// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Occt.IntPatch_SpecPntType;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// IntPatch_SpecialPoints: the pole of a sphere and the apex of a cone as intersection points, points on a quadric's
/// seam and at the crossing of its isolines, the tangents at a cone's apex, and the adjustment of parameters by
/// periods. The expected values are those of OCCT's algorithm worked by hand (the geometry is exact); no native
/// binding covers the class (oracle: indirect, 5G).
/// </summary>
public class IntPatchSpecialPointsTests
{
    private static readonly CoordinateSystem Origin = new(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0));

    private static GeomAdaptor_Surface Plane(double x, double y, double z, double nx, double ny, double nz) =>
        new(new Geom_Plane(new Point3D(x, y, z), new Direction3D(nx, ny, nz)), -10, 10, -10, 10);

    private static IntSurf_PntOn2S Pnt(Point3D P, double u1, double v1, double u2, double v2) =>
        new IntSurf_PntOn2S().WithValue(P, u1, v1, u2, v2);

    [Fact]
    public void GetTangentToIntLineForCone_TwoOrNone()
    {
        Xyz[] aRes = new Xyz[2];

        // A plane through the apex containing the axis: the two generatrices at u = -pi/2 and u = pi/2.
        int aNb = IntPatch_SpecialPoints.GetTangentToIntLineForCone(M_PI / 4, new Xyz(1, 0, 0), aRes);
        Assert.Equal(2, aNb);
        double aTan = Math.Tan(M_PI / 4);
        Assert.Equal(0.0, aRes[0].X(), 15);
        Assert.Equal(-aTan, aRes[0].Y(), 15);
        Assert.Equal(1.0, aRes[0].Z());
        Assert.Equal(0.0, aRes[1].X(), 15);
        Assert.Equal(aTan, aRes[1].Y(), 15);

        // A plane normal to the axis meets the cone at its apex only.
        Assert.Equal(0, IntPatch_SpecialPoints.GetTangentToIntLineForCone(M_PI / 4, new Xyz(0, 0, 1), aRes));
    }

    [Fact]
    public void AddSingularPole_SphereNorthPole()
    {
        // Sphere of radius 2; a plane through the north pole (0, 0, 2), normal (1, 0, 1): it cuts the sphere in a
        // circle whose tangent at the pole is the Y axis, so the sphere's U there is pi/2 (V = pi/2).
        GeomAdaptor_Surface aQ = new(new Geom_SphericalSurface(Origin, 2.0));
        GeomAdaptor_Surface aP = Plane(0, 0, 2, 1, 0, 1);

        IntPatch_Point aVertex = new();
        aVertex.SetValue(Pnt(new Point3D(0, 1e-4, 2), M_PI / 2 + 0.1, M_PI / 2 - 1e-4, 0.0, 0.0));
        aVertex.SetTolerance(1e-3);
        IntSurf_PntOn2S aPtIso = aVertex.PntOn2S();
        IntSurf_PntOn2S anAdded = new();

        Assert.True(IntPatch_SpecialPoints.AddSingularPole(aQ, aP, aPtIso, ref aVertex, ref anAdded));
        Assert.True(anAdded.Value().Distance(new Point3D(0, 0, 2)) < 1e-9);
        anAdded.Parameters(out double aUq, out double aVq, out double aUp, out double aVp);
        Assert.Equal(M_PI_2, aVq);
        Assert.Equal(M_PI_2, aUq, 9);
        Assert.True(aP.Value(aUp, aVp).Distance(new Point3D(0, 0, 2)) < 1e-9);

        // The pole is found on the sphere's other side too (reversed: the quadric is the second surface).
        IntPatch_Point aVertexR = new();
        aVertexR.SetValue(Pnt(new Point3D(0, 1e-4, 2), 0.0, 0.0, M_PI / 2 + 0.1, M_PI / 2 - 1e-4));
        aVertexR.SetTolerance(1e-3);
        IntSurf_PntOn2S anAddedR = new();
        Assert.True(IntPatch_SpecialPoints.AddSingularPole(aQ, aP, aVertexR.PntOn2S(), ref aVertexR, ref anAddedR, true));
        anAddedR.Parameters(out _, out _, out double aUq2, out double aVq2);
        Assert.Equal(M_PI_2, aVq2);
        Assert.Equal(M_PI_2, aUq2, 9);
    }

    [Fact]
    public void AddSingularPole_ConeApexAndRefCheck()
    {
        // Cone of semi-angle pi/4 and reference radius 1: the apex is at V = -sqrt(2), point (0, 0, -1). The plane
        // x = 0 through the apex contains the generatrices at U = pi/2 and 3*pi/2; the vertex's U chooses pi/2.
        GeomAdaptor_Surface aQ = new(new Geom_ConicalSurface(Origin, M_PI / 4, 1.0));
        GeomAdaptor_Surface aP = Plane(0, 0, -1, 1, 0, 0);

        IntPatch_Point aVertex = new();
        aVertex.SetValue(Pnt(new Point3D(0, 1e-4, -1 + 1e-4), M_PI / 2 + 0.05, -Math.Sqrt(2.0) + 1e-4, 0.0, 0.0));
        aVertex.SetTolerance(1e-3);
        IntSurf_PntOn2S anAdded = new();
        Assert.True(IntPatch_SpecialPoints.AddSingularPole(aQ, aP, aVertex.PntOn2S(), ref aVertex, ref anAdded));
        Assert.True(anAdded.Value().Distance(new Point3D(0, 0, -1)) < 1e-9);
        anAdded.ParametersOnS1(out double aUq, out double aVq);
        Assert.Equal(-1.0 / Math.Sin(M_PI / 4), aVq);
        Assert.Equal(M_PI_2, aUq, 9);

        // With the reference check, a vertex far from the apex is refused.
        IntPatch_Point aFar = new();
        aFar.SetValue(Pnt(new Point3D(0, 0.5, -0.5), M_PI / 2, -0.7, 0.0, 0.0));
        aFar.SetTolerance(1e-3);
        IntSurf_PntOn2S anUnused = new();
        Assert.False(IntPatch_SpecialPoints.AddSingularPole(aQ, aP, aFar.PntOn2S(), ref aFar, ref anUnused, false, true));
    }

    [Fact]
    public void AddSingularPole_OtherQuadric_Throws()
    {
        GeomAdaptor_Surface aQ = new(new Geom_CylindricalSurface(Origin, 1.0));
        GeomAdaptor_Surface aP = Plane(0, 0, 0, 0, 0, 1);
        IntPatch_Point aVertex = new();
        aVertex.SetTolerance(1e-3);
        IntSurf_PntOn2S anAdded = new();
        Assert.Throws<TypeMismatchException>(() =>
            IntPatch_SpecialPoints.AddSingularPole(aQ, aP, aVertex.PntOn2S(), ref aVertex, ref anAdded));
    }

    [Fact]
    public void AddPointOnUorVIso_CylinderSeamAndPlane()
    {
        // The cylinder's seam U = 0 is the line x = 1, y = 0; the plane z = 0.5 cuts it at (1, 0, 0.5): on the
        // plane at (U, V) = (1, 0), on the cylinder at (0, 0.5), adjusted to the reference point's U = 6.2 (2*pi).
        GeomAdaptor_Surface aQ = new(new Geom_CylindricalSurface(Origin, 1.0), 0, 2 * M_PI, -10, 10);
        GeomAdaptor_Surface aP = Plane(0, 0, 0.5, 0, 0, 1);
        IntSurf_PntOn2S aRef = Pnt(new Point3D(1, 0, 0.5), 6.2, 0.5, 1.0, -0.08);

        math_Vector aTol = new(1, 3), aStart = new(1, 3), anInf = new(1, 3), aSup = new(1, 3);
        aTol[1] = 1e-9;
        aTol[2] = 1e-9;
        aTol[3] = 1e-9;
        aStart[1] = 0.9;
        aStart[2] = 0.1;
        aStart[3] = 0.4;
        anInf[1] = -10;
        anInf[2] = -10;
        anInf[3] = -10;
        aSup[1] = 10;
        aSup[2] = 10;
        aSup[3] = 10;
        IntSurf_PntOn2S anAdded = new();
        Assert.True(IntPatch_SpecialPoints.AddPointOnUorVIso(aQ, aP, aRef, true, 0.0, aTol, aStart, anInf, aSup,
                                                             ref anAdded));
        anAdded.Parameters(out double aUq, out double aVq, out double aUp, out double aVp);
        Assert.Equal(2 * M_PI, aUq);
        Assert.Equal(0.5, aVq, 9);
        Assert.Equal(1.0, aUp, 9);
        Assert.Equal(0.0, aVp, 9);
        Assert.True(anAdded.Value().Distance(new Point3D(1, 0, 0.5)) < 1e-9);
    }

    [Fact]
    public void AddCrossUVIsoPoint_CylinderOrigin()
    {
        // The cylinder's point (U, V) = (0, 0) is (1, 0, 0), on the plane z = 0 at (1, 0).
        GeomAdaptor_Surface aQ = new(new Geom_CylindricalSurface(Origin, 1.0), 0, 2 * M_PI, -10, 10);
        GeomAdaptor_Surface aP = Plane(0, 0, 0, 0, 0, 1);
        IntSurf_PntOn2S aRef = Pnt(new Point3D(1, 0, 0), 6.2, 0.01, 0.9, 0.1);
        IntSurf_PntOn2S anAdded = new();
        Assert.True(IntPatch_SpecialPoints.AddCrossUVIsoPoint(aQ, aP, aRef, 1e-7, ref anAdded));
        anAdded.Parameters(out double aUq, out double aVq, out double aUp, out double aVp);
        Assert.Equal(2 * M_PI, aUq);
        Assert.Equal(0.0, aVq);
        Assert.Equal(1.0, aUp, 9);
        Assert.Equal(0.0, aVp, 9);

        // Too far from the plane: no point.
        GeomAdaptor_Surface aFar = Plane(0, 0, 1, 0, 0, 1);
        Assert.False(IntPatch_SpecialPoints.AddCrossUVIsoPoint(aQ, aFar, aRef, 1e-7, ref anAdded));
    }

    [Fact]
    public void AdjustPointAndVertex_ByPeriods()
    {
        IntSurf_PntOn2S aRef = Pnt(new Point3D(), 0.1, 0.2, 6.2, 0.3);
        IntSurf_PntOn2S aNew = Pnt(new Point3D(), 6.3, 0.2, 0.0, 0.3 + 4 * M_PI);
        double[] aPeriods = [2 * M_PI, 0.0, 2 * M_PI, 2 * M_PI];
        IntPatch_Point aVertex = new();
        IntPatch_SpecialPoints.AdjustPointAndVertex(aRef, aPeriods, ref aNew, ref aVertex);

        aNew.Parameters(out double u1, out double v1, out double u2, out double v2);
        Assert.Equal(6.3 - 2 * M_PI, u1);
        Assert.Equal(0.2, v1);
        Assert.Equal(2 * M_PI, u2);
        Assert.Equal(0.3 + 4 * M_PI - 2 * M_PI - 2 * M_PI, v2);
        aVertex.Parameters(out double w1, out double x1, out double w2, out double x2);
        Assert.Equal((u1, v1, u2, v2), (w1, x1, w2, x2));

        // No period: unchanged.
        IntSurf_PntOn2S aSame = Pnt(new Point3D(), 100, 100, 100, 100);
        IntPatch_SpecialPoints.AdjustPointAndVertex(aRef, [0.0, 0.0, 0.0, 0.0], ref aSame);
        aSame.Parameters(out double a, out double b, out double c, out double d);
        Assert.Equal((100.0, 100.0, 100.0, 100.0), (a, b, c, d));
    }

    [Fact]
    public void ContinueAfterSpecialPoint_NoneOrSame_ReturnsFalse()
    {
        GeomAdaptor_Surface aQ = new(new Geom_SphericalSurface(Origin, 2.0));
        GeomAdaptor_Surface aP = Plane(0, 0, 2, 1, 0, 1);
        IntSurf_PntOn2S aRef = Pnt(new Point3D(0, 0, 2), 1.0, M_PI_2, 0.0, 0.0);
        IntSurf_PntOn2S aNew = aRef;
        Assert.False(IntPatch_SpecialPoints.ContinueAfterSpecialPoint(aQ, aP, aRef, IntPatch_SPntNone, 1e-9, ref aNew));
        Assert.False(IntPatch_SpecialPoints.ContinueAfterSpecialPoint(aQ, aP, aRef, IntPatch_SPntPole, -1.0, ref aNew));

        // Through the pole, the quadric's U is adjusted with period pi/2 (OCCT's rule).
        IntSurf_PntOn2S aJump = Pnt(new Point3D(0, 0.01, 2), 1.0 + M_PI, M_PI_2, 0.5, 0.0);
        Assert.True(IntPatch_SpecialPoints.ContinueAfterSpecialPoint(aQ, aP, aRef, IntPatch_SPntPole, -1.0, ref aJump));
        aJump.ParametersOnS1(out double aU, out _);
        Assert.True(Math.Abs(aU - 1.0) <= M_PI_2 / 2, $"U = {aU}");
    }
}
