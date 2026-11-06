// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Laws;
using Kerf.Engine.Fillets.Blend;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// The inverse blend functions at an extremity (a fillet through a point or a curve, tangent to a surface), which the
/// fillet corpus rarely reaches. Their analytic derivatives against central differences of their values, their
/// combined form against their separate ones, and a known solution: a rolling ball on a plane.
/// </summary>
public class BRepBlend_InverseFunctionTests
{
    private const double R = 2.0;

    // The surface: a cylinder, so the derivatives aren't trivially zero
    private static SurfaceAdaptor Cylinder() =>
        new(new CylindricalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 5));

    private static SurfaceAdaptor Plane() =>
        new(new PlaneSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0))));

    private static CurveAdaptor Guide() =>
        new(new BezierCurve3D(new[] { new Point3D(7, -3, 1), new Point3D(8, 0, 2), new Point3D(6, 3, 4) }), 0, 1);

    private static CurveAdaptor2D Restriction() =>
        new(new LineCurve2D(new Point2D(0.1, 0.5), new Direction2D(1, 2)), 0, 5);

    private static LawsFunction Evolving()
    {
        var aLaw = new Linear();
        aLaw.Set(0, 1.5, 1, 2.5);
        return aLaw;
    }

    private static math_Vector Vec(double a, double b, double c)
    {
        var aV = new math_Vector(1, 3);
        aV[1] = a;
        aV[2] = b;
        aV[3] = c;
        return aV;
    }

    // Derivatives against central differences of Value; Values against Value and Derivatives
    private static void AssertDerivatives(FunctionSetWithDerivatives theF, math_Vector theX)
    {
        Assert.Equal(3, theF.NbEquations());
        var aD = new Matrix(1, 3, 1, 3);
        Assert.True(theF.Derivatives(theX, aD));
        var aValue = new math_Vector(1, 3);
        Assert.True(theF.Value(theX, aValue));

        const double H = 1e-6;
        for (int j = 1; j <= 3; j++)
        {
            var aXp = Vec(theX[1], theX[2], theX[3]);
            var aXm = Vec(theX[1], theX[2], theX[3]);
            aXp[j] += H;
            aXm[j] -= H;
            var aFp = new math_Vector(1, 3);
            var aFm = new math_Vector(1, 3);
            theF.Value(aXp, aFp);
            theF.Value(aXm, aFm);
            for (int i = 1; i <= 3; i++)
            {
                double aFd = (aFp[i] - aFm[i]) / (2 * H);
                Assert.True(Math.Abs(aFd - aD[i, j]) < 1e-4 * (1 + Math.Abs(aFd)), $"d f{i} / d x{j}: analytic {aD[i, j]}, differences {aFd}");
            }
        }

        var aF2 = new math_Vector(1, 3);
        var aD2 = new Matrix(1, 3, 1, 3);
        Assert.True(theF.Values(theX, aF2, aD2));
        for (int i = 1; i <= 3; i++)
        {
            Assert.Equal(aValue[i], aF2[i], 12);
            for (int j = 1; j <= 3; j++)
            {
                Assert.Equal(aD[i, j], aD2[i, j], 10);
            }
        }
    }

    [Theory]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    public void SurfPointConstRadInv_DerivativesAreThoseOfItsValues(int theChoice)
    {
        var aF = new SurfPointConstRadInv(Cylinder(), Guide());
        aF.Set(R, theChoice);
        aF.Set(new Point3D(3, 4, 5));
        AssertDerivatives(aF, Vec(0.4, 0.3, 2));
    }

    [Theory]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    public void SurfPointEvolRadInv_DerivativesAreThoseOfItsValues(int theChoice)
    {
        var aF = new SurfPointEvolRadInv(Cylinder(), Guide(), Evolving());
        aF.Set(theChoice);
        aF.Set(new Point3D(3, 4, 5));
        AssertDerivatives(aF, Vec(0.4, 0.3, 2));
    }

    [Theory]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    public void SurfCurvConstRadInv_DerivativesAreThoseOfItsValues(int theChoice)
    {
        var aF = new SurfCurvConstRadInv(Cylinder(), Guide(), Guide());
        aF.Set(R, theChoice);
        aF.Set(Restriction());
        AssertDerivatives(aF, Vec(0.4, 0.6, 1));
    }

    [Theory]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    public void SurfCurvEvolRadInv_DerivativesAreThoseOfItsValues(int theChoice)
    {
        var aF = new SurfCurvEvolRadInv(Cylinder(), Guide(), Guide(), Evolving());
        aF.Set(theChoice);
        aF.Set(Restriction());
        AssertDerivatives(aF, Vec(0.4, 0.6, 1));
    }

    // A ball of radius R rolling along a straight guide over the plane z = 0: the guide runs along X, and a section is
    // the plane x = w. The ball's centre is at (w, 0, R) for choices 1 and 2 and at (w, 0, -R) for 3 and 4.
    [Theory]
    [InlineData(1, 1)]
    [InlineData(2, 1)]
    [InlineData(3, -1)]
    [InlineData(4, -1)]
    public void SurfPointConstRadInv_HasTheRollingBallForItsSolution(int theChoice, int theSide)
    {
        var aGuide = new CurveAdaptor(new LineCurve3D(new Point3D(0, 0, R), new Direction3D(1, 0, 0)), 0, 10);
        var aF = new SurfPointConstRadInv(Plane(), aGuide);
        aF.Set(R, theChoice);
        aF.Set(new Point3D(4, R, theSide * R)); // on the ball's section
        Assert.True(aF.IsSolution(Vec(4, 4, 0), 1e-9));
        Assert.False(aF.IsSolution(Vec(4.1, 4, 0), 1e-9));
        Assert.False(aF.IsSolution(Vec(4, 4, 0.1), 1e-9));

        var aInf = new math_Vector(1, 3);
        var aSup = new math_Vector(1, 3);
        aF.GetBounds(aInf, aSup);
        Assert.Equal(0, aInf[1]);
        Assert.Equal(10, aSup[1]);
        var aTol = new math_Vector(1, 3);
        aF.GetTolerance(aTol, 1e-3);
        Assert.Equal(1e-3, aTol[1], 9);
        Assert.Equal(1e-3, aTol[2], 9);
        Assert.Equal(1e-3, aTol[3], 9);
    }

    [Theory]
    [InlineData(1, 1)]
    [InlineData(3, -1)]
    public void SurfPointEvolRadInv_HasTheRollingBallForItsSolution_AtTheLawsRadius(int theChoice, int theSide)
    {
        // the radius is 1.5 + w / 10 along the guide: at w = 5, it's 2
        var aGuide = new CurveAdaptor(new LineCurve3D(new Point3D(0, 0, R), new Direction3D(1, 0, 0)), 0, 10);
        var aLaw = new Linear();
        aLaw.Set(0, 1.5, 10, 2.5);
        var aF = new SurfPointEvolRadInv(Plane(), aGuide, aLaw);
        aF.Set(theChoice);
        aF.Set(new Point3D(5, R, theSide * R));
        Assert.True(aF.IsSolution(Vec(5, 5, 0), 1e-9));
        // the same point isn't on the ball of another section
        Assert.False(aF.IsSolution(Vec(4, 5, 0), 1e-9));
        var aInf = new math_Vector(1, 3);
        var aSup = new math_Vector(1, 3);
        aF.GetBounds(aInf, aSup);
        Assert.Equal(10, aSup[1]);
        var aTol = new math_Vector(1, 3);
        aF.GetTolerance(aTol, 1e-3);
        Assert.True(aTol[1] > 0);
    }

    [Theory]
    [InlineData(1, 1)]
    [InlineData(3, -1)]
    public void SurfCurvConstRadInv_HasTheRollingBallForItsSolution(int theChoice, int theSide)
    {
        // the curve passes through the ball's section at (4, R, side R), at its parameter 4; the restriction is the
        // line v = 0 of the plane, so the contact point (4, 0, 0) is at its parameter 4
        var aGuide = new CurveAdaptor(new LineCurve3D(new Point3D(0, 0, R), new Direction3D(1, 0, 0)), 0, 10);
        var aCurve = new CurveAdaptor(new LineCurve3D(new Point3D(0, R, theSide * R), new Direction3D(1, 0, 0)), 0, 10);
        var aRst = new CurveAdaptor2D(new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0)), 0, 10);
        var aF = new SurfCurvConstRadInv(Plane(), aCurve, aGuide);
        aF.Set(R, theChoice);
        aF.Set(aRst);
        Assert.True(aF.IsSolution(Vec(4, 4, 4), 1e-9));
        // the whole line of contact solves it too: moving only the guide's section loses it
        Assert.False(aF.IsSolution(Vec(4.1, 4, 4), 1e-9));
        var aInf = new math_Vector(1, 3);
        var aSup = new math_Vector(1, 3);
        aF.GetBounds(aInf, aSup);
        Assert.Equal(10, aSup[3]);
        var aTol = new math_Vector(1, 3);
        aF.GetTolerance(aTol, 1e-3);
        Assert.Equal(1e-3, aTol[3], 9);
    }

    [Fact]
    public void SurfCurvEvolRadInv_HasTheRollingBallForItsSolution_AtTheLawsRadius()
    {
        var aGuide = new CurveAdaptor(new LineCurve3D(new Point3D(0, 0, R), new Direction3D(1, 0, 0)), 0, 10);
        var aCurve = new CurveAdaptor(new LineCurve3D(new Point3D(0, R, R), new Direction3D(1, 0, 0)), 0, 10);
        var aRst = new CurveAdaptor2D(new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0)), 0, 10);
        var aLaw = new Linear();
        aLaw.Set(0, 1.5, 10, 2.5);
        var aF = new SurfCurvEvolRadInv(Plane(), aCurve, aGuide, aLaw);
        aF.Set(1);
        aF.Set(aRst);
        Assert.True(aF.IsSolution(Vec(5, 5, 5), 1e-9));
        Assert.False(aF.IsSolution(Vec(4, 5, 5), 1e-9));
        var aInf = new math_Vector(1, 3);
        var aSup = new math_Vector(1, 3);
        aF.GetBounds(aInf, aSup);
        Assert.Equal(10, aSup[2]);
        var aTol = new math_Vector(1, 3);
        aF.GetTolerance(aTol, 1e-3);
        Assert.True(aTol[3] > 0);
    }
}
