// Port of: src/ModelingAlgorithms/TKGeomAlgo/GTests/IntSurf_Quadric_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>OCCT's IntSurf_Quadric GTest.</summary>
public class IntSurf_Quadric_Test
{
    [Fact]
    public void ConeApexGradientRemainsFinite()
    {
        gp_Cone aCone = new(new gp_Ax3(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0), new gp_Dir(1.0, 0.0, 0.0)),
                            0.5, 0.0);
        IntSurf_Quadric aQuadric = new(aCone);
        gp_Pnt anApex = aCone.Apex();

        gp_Vec aGradient = aQuadric.Gradient(anApex);
        Assert.True(aGradient.SquareMagnitude() <= Precision.SquareConfusion());

        double aDist = 1.0;
        gp_Vec aGrad = new(1.0, 0.0, 0.0);
        aQuadric.ValAndGrad(anApex, ref aDist, ref aGrad);
        Assert.Equal(0.0, aDist, Precision.Confusion());
        Assert.True(aGrad.SquareMagnitude() <= Precision.SquareConfusion());
    }
}

/// <summary>Known answers of IntSurf_Quadric and IntSurf_QuadricTool.</summary>
public class IntSurf_QuadricTests
{
    private static readonly gp_Ax3 Frame = new(new gp_Pnt(1.0, -2.0, 0.5), new gp_Dir(0.3, 0.4, 0.866), new gp_Dir(1.0, 0.0, 0.0).Crossed(new gp_Dir(0.3, 0.4, 0.866)));

    public static TheoryData<int> Kinds => new() { 0, 1, 2, 3, 4 };

    private static IntSurf_Quadric Make(int theKind) => theKind switch
    {
        0 => new IntSurf_Quadric(new gp_Pln(Frame)),
        1 => new IntSurf_Quadric(new gp_Cylinder(Frame, 2.0)),
        2 => new IntSurf_Quadric(new gp_Sphere(Frame, 3.0)),
        3 => new IntSurf_Quadric(new gp_Cone(Frame, 0.4, 1.5)),
        _ => new IntSurf_Quadric(new gp_Torus(Frame, 4.0, 1.0)),
    };

    [Fact]
    public void DefaultIsAnEmptyQuadric()
    {
        IntSurf_Quadric aQ = new();
        Assert.Equal(GeomAbs_SurfaceType.GeomAbs_OtherSurface, aQ.TypeQuadric());
        Assert.Equal(0.0, aQ.Distance(new gp_Pnt(1, 2, 3)));
        double aDist = 7.0;
        gp_Vec aGrad = new(1, 2, 3);
        aQ.ValAndGrad(new gp_Pnt(1, 2, 3), ref aDist, ref aGrad);
        Assert.Equal(7.0, aDist);
        Assert.Equal(new gp_Vec(1, 2, 3), aGrad);
    }

    [Theory]
    [MemberData(nameof(Kinds))]
    public void PointsOnTheSurfaceAreAtZeroDistance(int theKind)
    {
        IntSurf_Quadric aQ = Make(theKind);
        for (double u = 0.1; u < 6.0; u += 0.7)
        {
            for (double v = -0.9; v < 1.0; v += 0.3)
            {
                gp_Pnt aP = aQ.Value(u, v);
                Assert.Equal(0.0, aQ.Distance(aP), 1e-12);
                Assert.Equal(0.0, IntSurf_QuadricTool.Value(aQ, aP.X(), aP.Y(), aP.Z()), 1e-12);
            }
        }
    }

    [Theory]
    [MemberData(nameof(Kinds))]
    public void GradientIsTheNormalAndMatchesFiniteDifferences(int theKind)
    {
        IntSurf_Quadric aQ = Make(theKind);
        double h = 1e-6;
        foreach (gp_Pnt aP in new[] { new gp_Pnt(3.0, 1.0, 2.0), new gp_Pnt(-2.0, 0.5, -1.0), new gp_Pnt(0.7, 4.0, 1.5) })
        {
            gp_Vec aGrad = aQ.Gradient(aP);
            gp_Vec aFD = new((aQ.Distance(new gp_Pnt(aP.X() + h, aP.Y(), aP.Z())) - aQ.Distance(new gp_Pnt(aP.X() - h, aP.Y(), aP.Z()))) / (2 * h),
                             (aQ.Distance(new gp_Pnt(aP.X(), aP.Y() + h, aP.Z())) - aQ.Distance(new gp_Pnt(aP.X(), aP.Y() - h, aP.Z()))) / (2 * h),
                             (aQ.Distance(new gp_Pnt(aP.X(), aP.Y(), aP.Z() + h)) - aQ.Distance(new gp_Pnt(aP.X(), aP.Y(), aP.Z() - h))) / (2 * h));
            if (theKind != 3)
            {
                // the cone's "distance" is measured along the axis' normal, not orthogonally: only its sign agrees
                Assert.True(aGrad.IsEqual(aFD, 1e-5, 1e-5), $"{aGrad} vs {aFD}");
            }

            double aDist = 0.0;
            gp_Vec aG2 = new();
            aQ.ValAndGrad(aP, ref aDist, ref aG2);
            Assert.Equal(aQ.Distance(aP), aDist, 1e-14);
            Assert.True(aG2.IsEqual(aGrad, 1e-14, 1e-12));
            IntSurf_QuadricTool.Gradient(aQ, aP.X(), aP.Y(), aP.Z(), out gp_Vec aG3);
            Assert.Equal(aGrad, aG3);
        }
    }

    [Theory]
    [MemberData(nameof(Kinds))]
    public void ParametersInvertValue(int theKind)
    {
        IntSurf_Quadric aQ = Make(theKind);
        double u0 = 1.2, v0 = theKind == 0 ? 0.6 : 0.4;
        gp_Pnt aP = aQ.Value(u0, v0);
        double u = 0, v = 0;
        aQ.Parameters(aP, ref u, ref v);
        Assert.Equal(u0, u, 1e-12);
        Assert.Equal(v0, v, 1e-12);

        gp_Pnt aP1 = new();
        gp_Vec aD1U = new(), aD1V = new();
        aQ.D1(u0, v0, ref aP1, ref aD1U, ref aD1V);
        Assert.True(aP1.IsEqual(aP, 1e-14));
        Assert.True(aQ.DN(u0, v0, 1, 0).IsEqual(aD1U, 1e-14, 1e-14));
        Assert.True(aQ.DN(u0, v0, 0, 1).IsEqual(aD1V, 1e-14, 1e-14));

        // The normal at (u, v) is along the gradient at the point
        gp_Vec aN = aQ.Normale(u0, v0);
        gp_Vec aG = aQ.Gradient(aP);
        Assert.True(aN.IsParallel(aG, 1e-9));
        Assert.True(aQ.Normale(aP).IsParallel(aN, 1e-9));
    }

    [Fact]
    public void QuadricsKeepTheirDefinition()
    {
        gp_Cylinder aCyl = new(Frame, 2.0);
        IntSurf_Quadric aQ = new(aCyl);
        Assert.Equal(GeomAbs_SurfaceType.GeomAbs_Cylinder, aQ.TypeQuadric());
        Assert.Equal(2.0, aQ.Cylinder().Radius());
        Assert.Equal(4.0e-6, IntSurf_QuadricTool.Tolerance(aQ));
        Assert.Equal(6.0e-6, IntSurf_QuadricTool.Tolerance(new IntSurf_Quadric(new gp_Sphere(Frame, 3.0))));
        Assert.Equal(1.0e-6, IntSurf_QuadricTool.Tolerance(new IntSurf_Quadric(new gp_Pln(Frame))));

        gp_Cone aCone = new(Frame, 0.4, 1.5);
        IntSurf_Quadric aQc = new(aCone);
        Assert.Equal(0.4, aQc.Cone().SemiAngle());
        Assert.Equal(1.5, aQc.Cone().RefRadius());

        // SetValue rebuilds the whole quadric, as the constructor does
        IntSurf_Quadric aQs = new(new gp_Pln(Frame));
        aQs.SetValue(aCyl);
        Assert.Equal(aQ, aQs);
        IntSurf_Quadric aQt = new(new gp_Torus(Frame, 4.0, 1.0));
        Assert.Equal(4.0, aQt.Torus().MajorRadius());
        Assert.Equal(1.0, aQt.Torus().MinorRadius());
    }
}
