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
        aF.SetTangent(new Vector3D(1, 2, -1)); // the restriction's, at the point (decision 2158)
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
        aF.SetTangent(new Vector3D(1, 0, 0)); // a restriction along the guide (decision 2158)
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

    // The ball between two edges (decision 2162): edge 1 on the plane z = 0, edge 2 on the plane y = 0, neither along
    // the guide (the X axis), so OCCT's section in the guide's plane is off the ball there.
    private static SurfaceAdaptor PlaneY() =>
        new(new PlaneSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 1, 0), new Direction3D(1, 0, 0))));

    private static CurveAdaptor2D Edge1() => new(new LineCurve2D(new Point2D(0, 1), new Direction2D(1, 0.3)), -5, 15);

    private static CurveAdaptor2D Edge2() => new(new LineCurve2D(new Point2D(0, -1), new Direction2D(1, -0.2)), -5, 15);

    private static CurveAdaptor AlongX() => new(new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), 0, 10);

    private static RstRstConstRad BetweenEdges(int theChoice, double theParam)
    {
        var aF = new RstRstConstRad(Plane(), Edge1(), PlaneY(), Edge2(), AlongX());
        aF.Set(Plane(), Edge1(), PlaneY(), Edge2());
        aF.Set(R, theChoice);
        aF.Set(theParam);
        return aF;
    }

    private static math_Vector Vec(double a, double b)
    {
        var aV = new math_Vector(1, 2);
        aV[1] = a;
        aV[2] = b;
        return aV;
    }

    // As AssertDerivatives, for the functions of two variables
    private static void AssertDerivatives2(FunctionSetWithDerivatives theF, math_Vector theX)
    {
        Assert.Equal(2, theF.NbEquations());
        var aD = new Matrix(1, 2, 1, 2);
        Assert.True(theF.Derivatives(theX, aD));
        const double H = 1e-6;
        for (int j = 1; j <= 2; j++)
        {
            var aXp = Vec(theX[1], theX[2]);
            var aXm = Vec(theX[1], theX[2]);
            aXp[j] += H;
            aXm[j] -= H;
            var aFp = new math_Vector(1, 2);
            var aFm = new math_Vector(1, 2);
            theF.Value(aXp, aFp);
            theF.Value(aXm, aFm);
            for (int i = 1; i <= 2; i++)
            {
                double aFd = (aFp[i] - aFm[i]) / (2 * H);
                Assert.True(Math.Abs(aFd - aD[i, j]) < 1e-5 * (1 + Math.Abs(aFd)), $"d f{i} / d x{j}: analytic {aD[i, j]}, differences {aFd}");
            }
        }
        var aValue = new math_Vector(1, 2);
        var aF2 = new math_Vector(1, 2);
        var aD2 = new Matrix(1, 2, 1, 2);
        theF.Value(theX, aValue);
        Assert.True(theF.Values(theX, aF2, aD2));
        for (int i = 1; i <= 2; i++)
        {
            Assert.Equal(aValue[i], aF2[i], 12);
            for (int j = 1; j <= 2; j++)
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
    public void RstRstConstRad_DerivativesAreThoseOfItsValues(int theChoice)
    {
        AssertDerivatives2(BetweenEdges(theChoice, 4), Vec(4.2, 3.7));
    }

    [Theory]
    [InlineData(1, true)]
    [InlineData(3, true)]
    [InlineData(2, false)]
    [InlineData(4, false)]
    public void CurvPointRadInv_DerivativesAreThoseOfItsValues(int theChoice, bool thePointOnFirst)
    {
        var aF = new CurvPointRadInv(Guide(), new CurveOnSurface(Edge2(), PlaneY()));
        aF.Set(R, theChoice, thePointOnFirst);
        aF.Set(new Point3D(7.5, 1.2, 0.4));
        aF.SetTangent(new Vector3D(1, 0.3, 0)); // edge 1's
        AssertDerivatives2(aF, Vec(0.4, 7.2));
    }

    // Solved, the section is the ball's: the centre at R from both edges' points, its chord to each square to that edge,
    // the points' midpoint in the guide's plane; and the centre is off that plane, where OCCT put it
    [Theory]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    public void RstRstConstRad_HasTheBallBetweenTheEdgesForItsSolution(int theChoice)
    {
        RstRstConstRad aF = BetweenEdges(theChoice, 4);
        var aTol = new math_Vector(1, 2);
        var aInf = new math_Vector(1, 2);
        var aSup = new math_Vector(1, 2);
        aF.GetTolerance(aTol, 1e-10);
        aF.GetBounds(aInf, aSup);
        var aRoot = new FunctionSetRoot(aF, aTol, 50);
        aRoot.Perform(aF, Vec(4, 4), aInf, aSup);
        Assert.True(aRoot.IsDone);
        var aSol = new math_Vector(1, 2);
        aRoot.Root(aSol);
        Assert.True(aF.IsSolution(aSol, 1e-9));

        Point3D aP1 = aF.PointOnRst1(), aP2 = aF.PointOnRst2();
        var aCircle = new Circle3D();
        aF.Section(4, aSol[1], aSol[2], out _, out _, ref aCircle);
        Point3D aC = aCircle.Location;
        Assert.Equal(R, aCircle.Radius, 12);
        Assert.Equal(R, aC.DistanceTo(aP1), 9);
        Assert.Equal(R, aC.DistanceTo(aP2), 9);
        Assert.Equal(0, new Vector3D(aP1, aC).Dot(new Vector3D(1, 0.3, 0).Normalized()), 9);
        Assert.Equal(0, new Vector3D(aP2, aC).Dot(new Vector3D(1, 0, 0.2).Normalized()), 9);
        Assert.Equal(4, (aP1.X + aP2.X) / 2, 9);
        Assert.True(Math.Abs(aC.X - 4) > 1e-2, $"the centre at {aC.X}, in the guide's plane");
        // The circle's plane holds both points
        Assert.Equal(0, new Vector3D(aC, aP1).Dot(new Vector3D(aCircle.Axis.Direction)), 9);
        Assert.Equal(0, new Vector3D(aC, aP2).Dot(new Vector3D(aCircle.Axis.Direction)), 9);
    }

    // The evolving radius between the two edges (decision 2216): 1.5 + w / 10 along the guide, 1.9 at w = 4
    private static LawsFunction AlongTheGuide()
    {
        var aLaw = new Linear();
        aLaw.Set(0, 1.5, 10, 2.5);
        return aLaw;
    }

    private static RstRstEvolRad BetweenEdgesEvolving(int theChoice, double theParam)
    {
        var aF = new RstRstEvolRad(Plane(), Edge1(), PlaneY(), Edge2(), AlongX(), AlongTheGuide());
        aF.Set(Plane(), Edge1(), PlaneY(), Edge2());
        aF.Set(theChoice);
        aF.Set(theParam);
        return aF;
    }

    [Theory]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    public void RstRstEvolRad_DerivativesAreThoseOfItsValues(int theChoice)
    {
        AssertDerivatives2(BetweenEdgesEvolving(theChoice, 4), Vec(4.2, 3.7));
    }

    [Theory]
    [InlineData(1, true)]
    [InlineData(3, true)]
    [InlineData(2, false)]
    [InlineData(4, false)]
    public void CurvPointRadInv_DerivativesAreThoseOfItsValues_WithAnEvolvingRadius(int theChoice, bool thePointOnFirst)
    {
        var aF = new CurvPointRadInv(Guide(), new CurveOnSurface(Edge2(), PlaneY()));
        aF.Set(Evolving(), theChoice, thePointOnFirst);
        aF.Set(new Point3D(7.5, 1.2, 0.4));
        aF.SetTangent(new Vector3D(1, 0.3, 0)); // edge 1's
        AssertDerivatives2(aF, Vec(0.4, 7.2));
    }

    // Solved, the section is on the ball of the law's radius there: the centre at that radius from both edges' points, its
    // chord to each square to that edge; and the walk's tangent, which takes the radius's change, is the solutions' own.
    // The section's circle is the envelope's (unit 15AK): smaller than the ball's great circle as the radius changes,
    // through both points, the ball's centre on its axis
    [Theory]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    public void RstRstEvolRad_HasTheBallOfTheLawsRadiusBetweenTheEdgesForItsSolution(int theChoice)
    {
        (math_Vector Sol, Point3D P1, Point3D P2) Solve(RstRstEvolRad theF)
        {
            var aTol = new math_Vector(1, 2);
            var aInf = new math_Vector(1, 2);
            var aSup = new math_Vector(1, 2);
            theF.GetTolerance(aTol, 1e-12);
            theF.GetBounds(aInf, aSup);
            var aRoot = new FunctionSetRoot(theF, aTol, 50);
            aRoot.Perform(theF, Vec(4, 4), aInf, aSup);
            Assert.True(aRoot.IsDone);
            var aSol = new math_Vector(1, 2);
            aRoot.Root(aSol);
            Assert.True(theF.IsSolution(aSol, 1e-9));
            return (aSol, theF.PointOnRst1(), theF.PointOnRst2());
        }

        RstRstEvolRad aF = BetweenEdgesEvolving(theChoice, 4);
        (math_Vector aSol, Point3D aP1, Point3D aP2) = Solve(aF);
        var aCircle = new Circle3D();
        aF.Section(4, aSol[1], aSol[2], out _, out _, ref aCircle);
        Assert.InRange(aCircle.Radius, 1.8, 1.9 - 1e-6);
        Assert.Equal(aCircle.Radius, aCircle.Location.DistanceTo(aP1), 9);
        Assert.Equal(aCircle.Radius, aCircle.Location.DistanceTo(aP2), 9);
        Assert.Equal(0, new Vector3D(aCircle.Location, aP1).Dot(new Vector3D(aCircle.Axis.Direction)), 9);
        Assert.Equal(0, new Vector3D(aCircle.Location, aP2).Dot(new Vector3D(aCircle.Axis.Direction)), 9);
        // The ball's centre, on the circle's axis on one side or the other
        double aOff = Math.Sqrt(1.9 * 1.9 - aCircle.Radius * aCircle.Radius);
        Assert.Contains(new[] { 1.0, -1.0 }, s =>
        {
            Point3D aC = aCircle.Location.Translated(s * aOff * new Vector3D(aCircle.Axis.Direction));
            return Math.Abs(new Vector3D(aP1, aC).Dot(new Vector3D(1, 0.3, 0).Normalized())) < 1e-9
                   && Math.Abs(new Vector3D(aP2, aC).Dot(new Vector3D(1, 0, 0.2).Normalized())) < 1e-9;
        });
        Vector3D aTangent1 = aF.TangentOnRst1(), aTangent2 = aF.TangentOnRst2();

        const double H = 1e-5;
        (_, Point3D aP1p, Point3D aP2p) = Solve(BetweenEdgesEvolving(theChoice, 4 + H));
        (_, Point3D aP1m, Point3D aP2m) = Solve(BetweenEdgesEvolving(theChoice, 4 - H));
        Vector3D aFd1 = new Vector3D(aP1m, aP1p) / (2 * H), aFd2 = new Vector3D(aP2m, aP2p) / (2 * H);
        Assert.True((aFd1 - aTangent1).Magnitude < 1e-5 * (1 + aFd1.Magnitude), $"first: {aTangent1} against {aFd1}");
        Assert.True((aFd2 - aTangent2).Magnitude < 1e-5 * (1 + aFd2.Magnitude), $"second: {aTangent2} against {aFd2}");
    }

    // Unit 15AK: the derivative of the envelope's section along the walk, which takes the second derivative of the centre's
    // path (by differences of the first), is that of the sections at the walk's solutions either side
    [Theory]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    public void RstRstEvolRad_SectionsDerivativeIsThatOfItsSections(int theChoice)
    {
        (Point3D[] Poles, double[] Weights, Vector3D[] DPoles, double[] DWeights) Section(double theParam)
        {
            RstRstEvolRad aF = BetweenEdgesEvolving(theChoice, theParam);
            var aTol = new math_Vector(1, 2);
            var aInf = new math_Vector(1, 2);
            var aSup = new math_Vector(1, 2);
            aF.GetTolerance(aTol, 1e-12);
            aF.GetBounds(aInf, aSup);
            var aRoot = new FunctionSetRoot(aF, aTol, 50);
            aRoot.Perform(aF, Vec(4, 4), aInf, aSup);
            Assert.True(aRoot.IsDone);
            var aSol = new math_Vector(1, 2);
            aRoot.Root(aSol);
            Assert.True(aF.IsSolution(aSol, 1e-9));
            aF.GetShape(out int aNbPoles, out _, out _, out _);
            var aPoles = new NCollection_Array1<Point3D>(1, aNbPoles);
            var aDPoles = new NCollection_Array1<Vector3D>(1, aNbPoles);
            var aPoles2d = new NCollection_Array1<Point2D>(1, 2);
            var aDPoles2d = new NCollection_Array1<Vector2D>(1, 2);
            var aWeights = new NCollection_Array1<double>(1, aNbPoles);
            var aDWeights = new NCollection_Array1<double>(1, aNbPoles);
            BlendPoint aPoint = new();
            aPoint.SetValue(aF.PointOnRst1(), aF.PointOnRst2(), theParam, aSol[1], aSol[2]);
            Assert.True(aF.Section(aPoint, aPoles, aDPoles, aPoles2d, aDPoles2d, aWeights, aDWeights));
            return ([.. Enumerable.Range(1, aNbPoles).Select(k => aPoles[k])], [.. Enumerable.Range(1, aNbPoles).Select(k => aWeights[k])],
                    [.. Enumerable.Range(1, aNbPoles).Select(k => aDPoles[k])], [.. Enumerable.Range(1, aNbPoles).Select(k => aDWeights[k])]);
        }

        const double H = 1e-4;
        var aHere = Section(4);
        var aAfter = Section(4 + H);
        var aBefore = Section(4 - H);
        Assert.Equal(aHere.Poles.Length, aAfter.Poles.Length);
        Assert.Equal(aHere.Poles.Length, aBefore.Poles.Length);
        for (int k = 0; k < aHere.Poles.Length; k++)
        {
            Vector3D aFd = new Vector3D(aBefore.Poles[k], aAfter.Poles[k]) / (2 * H);
            Assert.True((aFd - aHere.DPoles[k]).Magnitude < 1e-6 * (1 + aFd.Magnitude), $"pole {k}: {aHere.DPoles[k]} against {aFd}");
            Assert.Equal((aAfter.Weights[k] - aBefore.Weights[k]) / (2 * H), aHere.DWeights[k], 6);
        }
    }
}
