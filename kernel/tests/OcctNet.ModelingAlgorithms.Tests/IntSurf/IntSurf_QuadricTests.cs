// Port of: src/ModelingAlgorithms/TKGeomAlgo/GTests/IntSurf_Quadric_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>OCCT's IntSurf_Quadric GTest.</summary>
public class IntSurf_Quadric_Test
{
    [Fact]
    public void ConeApexGradientRemainsFinite()
    {
        Cone aCone = new(new CoordinateSystem(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0), new Direction3D(1.0, 0.0, 0.0)),
                            0.5, 0.0);
        SurfaceQuadric aQuadric = new(aCone);
        Point3D anApex = aCone.Apex();

        Vector3D aGradient = aQuadric.Gradient(anApex);
        Assert.True(aGradient.SquareMagnitude() <= global::Kerf.Engine.Foundation.Precision.SquareConfusion());

        double aDist = 1.0;
        Vector3D aGrad = new(1.0, 0.0, 0.0);
        aQuadric.ValAndGrad(anApex, ref aDist, ref aGrad);
        Assert.Equal(0.0, aDist, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(aGrad.SquareMagnitude() <= global::Kerf.Engine.Foundation.Precision.SquareConfusion());
    }
}

/// <summary>Known answers of IntSurf_Quadric and IntSurf_QuadricTool.</summary>
public class IntSurf_QuadricTests
{
    private static readonly CoordinateSystem Frame = new(new Point3D(1.0, -2.0, 0.5), new Direction3D(0.3, 0.4, 0.866), new Direction3D(1.0, 0.0, 0.0).Crossed(new Direction3D(0.3, 0.4, 0.866)));

    public static TheoryData<int> Kinds => new() { 0, 1, 2, 3, 4 };

    private static SurfaceQuadric Make(int theKind) => theKind switch
    {
        0 => new SurfaceQuadric(new Plane3D(Frame)),
        1 => new SurfaceQuadric(new Cylinder(Frame, 2.0)),
        2 => new SurfaceQuadric(new Sphere(Frame, 3.0)),
        3 => new SurfaceQuadric(new Cone(Frame, 0.4, 1.5)),
        _ => new SurfaceQuadric(new Torus(Frame, 4.0, 1.0)),
    };

    [Fact]
    public void DefaultIsAnEmptyQuadric()
    {
        SurfaceQuadric aQ = new();
        Assert.Equal(SurfaceKind.OtherSurface, aQ.TypeQuadric());
        Assert.Equal(0.0, aQ.Distance(new Point3D(1, 2, 3)));
        double aDist = 7.0;
        Vector3D aGrad = new(1, 2, 3);
        aQ.ValAndGrad(new Point3D(1, 2, 3), ref aDist, ref aGrad);
        Assert.Equal(7.0, aDist);
        Assert.Equal(new Vector3D(1, 2, 3), aGrad);
    }

    [Theory]
    [MemberData(nameof(Kinds))]
    public void PointsOnTheSurfaceAreAtZeroDistance(int theKind)
    {
        SurfaceQuadric aQ = Make(theKind);
        for (double u = 0.1; u < 6.0; u += 0.7)
        {
            for (double v = -0.9; v < 1.0; v += 0.3)
            {
                Point3D aP = aQ.Value(u, v);
                Assert.Equal(0.0, aQ.Distance(aP), 1e-12);
                Assert.Equal(0.0, QuadricTool.Value(aQ, aP.X(), aP.Y(), aP.Z()), 1e-12);
            }
        }
    }

    [Theory]
    [MemberData(nameof(Kinds))]
    public void GradientIsTheNormalAndMatchesFiniteDifferences(int theKind)
    {
        SurfaceQuadric aQ = Make(theKind);
        double h = 1e-6;
        foreach (Point3D aP in new[] { new Point3D(3.0, 1.0, 2.0), new Point3D(-2.0, 0.5, -1.0), new Point3D(0.7, 4.0, 1.5) })
        {
            Vector3D aGrad = aQ.Gradient(aP);
            Vector3D aFD = new((aQ.Distance(new Point3D(aP.X() + h, aP.Y(), aP.Z())) - aQ.Distance(new Point3D(aP.X() - h, aP.Y(), aP.Z()))) / (2 * h),
                             (aQ.Distance(new Point3D(aP.X(), aP.Y() + h, aP.Z())) - aQ.Distance(new Point3D(aP.X(), aP.Y() - h, aP.Z()))) / (2 * h),
                             (aQ.Distance(new Point3D(aP.X(), aP.Y(), aP.Z() + h)) - aQ.Distance(new Point3D(aP.X(), aP.Y(), aP.Z() - h))) / (2 * h));
            if (theKind != 3)
            {
                // the cone's "distance" is measured along the axis' normal, not orthogonally: only its sign agrees
                Assert.True(aGrad.IsEqual(aFD, 1e-5, 1e-5), $"{aGrad} vs {aFD}");
            }

            double aDist = 0.0;
            Vector3D aG2 = new();
            aQ.ValAndGrad(aP, ref aDist, ref aG2);
            Assert.Equal(aQ.Distance(aP), aDist, 1e-14);
            Assert.True(aG2.IsEqual(aGrad, 1e-14, 1e-12));
            QuadricTool.Gradient(aQ, aP.X(), aP.Y(), aP.Z(), out Vector3D aG3);
            Assert.Equal(aGrad, aG3);
        }
    }

    [Theory]
    [MemberData(nameof(Kinds))]
    public void ParametersInvertValue(int theKind)
    {
        SurfaceQuadric aQ = Make(theKind);
        double u0 = 1.2, v0 = theKind == 0 ? 0.6 : 0.4;
        Point3D aP = aQ.Value(u0, v0);
        double u = 0, v = 0;
        aQ.Parameters(aP, ref u, ref v);
        Assert.Equal(u0, u, 1e-12);
        Assert.Equal(v0, v, 1e-12);

        Point3D aP1 = new();
        Vector3D aD1U = new(), aD1V = new();
        aQ.D1(u0, v0, ref aP1, ref aD1U, ref aD1V);
        Assert.True(aP1.IsEqual(aP, 1e-14));
        Assert.True(aQ.DN(u0, v0, 1, 0).IsEqual(aD1U, 1e-14, 1e-14));
        Assert.True(aQ.DN(u0, v0, 0, 1).IsEqual(aD1V, 1e-14, 1e-14));

        // The normal at (u, v) is along the gradient at the point
        Vector3D aN = aQ.Normale(u0, v0);
        Vector3D aG = aQ.Gradient(aP);
        Assert.True(aN.IsParallel(aG, 1e-9));
        Assert.True(aQ.Normale(aP).IsParallel(aN, 1e-9));
    }

    [Fact]
    public void QuadricsKeepTheirDefinition()
    {
        Cylinder aCyl = new(Frame, 2.0);
        SurfaceQuadric aQ = new(aCyl);
        Assert.Equal(SurfaceKind.Cylinder, aQ.TypeQuadric());
        Assert.Equal(2.0, aQ.Cylinder().Radius());
        Assert.Equal(4.0e-6, QuadricTool.Tolerance(aQ));
        Assert.Equal(6.0e-6, QuadricTool.Tolerance(new SurfaceQuadric(new Sphere(Frame, 3.0))));
        Assert.Equal(1.0e-6, QuadricTool.Tolerance(new SurfaceQuadric(new Plane3D(Frame))));

        Cone aCone = new(Frame, 0.4, 1.5);
        SurfaceQuadric aQc = new(aCone);
        Assert.Equal(0.4, aQc.Cone().SemiAngle());
        Assert.Equal(1.5, aQc.Cone().RefRadius());

        // SetValue rebuilds the whole quadric, as the constructor does
        SurfaceQuadric aQs = new(new Plane3D(Frame));
        aQs.SetValue(aCyl);
        Assert.Equal(aQ, aQs);
        SurfaceQuadric aQt = new(new Torus(Frame, 4.0, 1.0));
        Assert.Equal(4.0, aQt.Torus().MajorRadius());
        Assert.Equal(1.0, aQt.Torus().MinorRadius());
    }
}
