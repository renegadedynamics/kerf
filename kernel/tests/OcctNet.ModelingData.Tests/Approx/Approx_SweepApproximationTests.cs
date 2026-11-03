// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Kerf.Geometry.Continuity;

namespace OcctNet.ModelingData.Tests;

// Approx_SweepApproximation on simple sweep functions (OCCT has no GTests for it; GeomFill's functions aren't ported).
public class Approx_SweepApproximationTests
{
    // A rational circle (nine poles, degree 2) of radius 1 + 0.5 v, centred at (0, 0, v), translated along Z; with
    // one 2d curve (v, 2 v) when With2d.
    private sealed class GrowingCircle : SweepFunction
    {
        private static readonly double[] theQx = [1, 1, 0, -1, -1, -1, 0, 1, 1];
        private static readonly double[] theQy = [0, 1, 1, 1, 0, -1, -1, -1, 0];
        private readonly bool myWith2d;
        private readonly bool myDerivatives;
        public int NbSetInterval;

        public GrowingCircle(bool theWith2d, bool theDerivatives)
        {
            myWith2d = theWith2d;
            myDerivatives = theDerivatives;
        }

        public static double Weight(int i) => i % 2 == 1 ? 1.0 : Math.Sqrt(2.0) / 2.0;

        public static double Radius(double v) => 1.0 + 0.5 * v;

        public static Point3D Pole(int i, double v) =>
            new(Radius(v) * theQx[i - 1], Radius(v) * theQy[i - 1], v);

        public static BSplineCurve3D Section(double v)
        {
            NCollection_Array1<Point3D> aPoles = new(1, 9);
            NCollection_Array1<double> aWeights = new(1, 9);
            for (int i = 1; i <= 9; i++)
            {
                aPoles[i] = Pole(i, v);
                aWeights[i] = Weight(i);
            }
            NCollection_Array1<double> aKnots = new(1, 5);
            NCollection_Array1<int> aMults = new(1, 5);
            SectionKnots(aKnots, aMults);
            return new BSplineCurve3D(aPoles, aWeights, aKnots, aMults, 2);
        }

        private static void SectionKnots(NCollection_Array1<double> theKnots, NCollection_Array1<int> theMults)
        {
            for (int i = 1; i <= 5; i++)
            {
                theKnots[i] = (i - 1) * 0.5 * Math.PI;
                theMults[i] = (i == 1 || i == 5) ? 3 : 2;
            }
        }

        public override bool D0(double Param, double First, double Last, NCollection_Array1<Point3D> Poles,
                                NCollection_Array1<Point2D> Poles2d, NCollection_Array1<double> Weigths)
        {
            for (int i = 1; i <= 9; i++)
            {
                Poles[i] = Pole(i, Param);
                Weigths[i] = Weight(i);
            }
            if (myWith2d)
            {
                Poles2d[1] = new Point2D(Param, 2.0 * Param);
            }
            return true;
        }

        public override bool D1(double Param, double First, double Last, NCollection_Array1<Point3D> Poles,
                                NCollection_Array1<Vector3D> DPoles, NCollection_Array1<Point2D> Poles2d,
                                NCollection_Array1<Vector2D> DPoles2d, NCollection_Array1<double> Weigths,
                                NCollection_Array1<double> DWeigths)
        {
            if (!myDerivatives)
            {
                return false;
            }
            D0(Param, First, Last, Poles, Poles2d, Weigths);
            for (int i = 1; i <= 9; i++)
            {
                DPoles[i] = new Vector3D(0.5 * theQx[i - 1], 0.5 * theQy[i - 1], 1.0);
                DWeigths[i] = 0.0;
            }
            if (myWith2d)
            {
                DPoles2d[1] = new Vector2D(1.0, 2.0);
            }
            return true;
        }

        public override bool D2(double Param, double First, double Last, NCollection_Array1<Point3D> Poles,
                                NCollection_Array1<Vector3D> DPoles, NCollection_Array1<Vector3D> D2Poles,
                                NCollection_Array1<Point2D> Poles2d, NCollection_Array1<Vector2D> DPoles2d,
                                NCollection_Array1<Vector2D> D2Poles2d, NCollection_Array1<double> Weigths,
                                NCollection_Array1<double> DWeigths, NCollection_Array1<double> D2Weigths)
        {
            if (!D1(Param, First, Last, Poles, DPoles, Poles2d, DPoles2d, Weigths, DWeigths))
            {
                return false;
            }
            for (int i = 1; i <= 9; i++)
            {
                D2Poles[i] = new Vector3D(0.0, 0.0, 0.0);
                D2Weigths[i] = 0.0;
            }
            if (myWith2d)
            {
                D2Poles2d[1] = new Vector2D(0.0, 0.0);
            }
            return true;
        }

        public override int Nb2dCurves() => myWith2d ? 1 : 0;

        public override void SectionShape(out int NbPoles, out int NbKnots, out int Degree)
        {
            NbPoles = 9;
            NbKnots = 5;
            Degree = 2;
        }

        public override void Knots(NCollection_Array1<double> TKnots) =>
            SectionKnots(TKnots, new NCollection_Array1<int>(1, 5));

        public override void Mults(NCollection_Array1<int> TMults) =>
            SectionKnots(new NCollection_Array1<double>(1, 5), TMults);

        public override bool IsRational => true;

        public override int NbIntervals(Continuity S) => 1;

        public override void Intervals(NCollection_Array1<double> T, Continuity S)
        {
            T[1] = 0.0;
            T[2] = 2.0;
        }

        public override void SetInterval(double First, double Last) => NbSetInterval++;

        public override void Resolution(int Index, double Tol, out double TolU, out double TolV)
        {
            TolU = Tol;
            TolV = Tol / 2.0;
        }

        public override void GetTolerance(double BoundTol, double SurfTol, double AngleTol,
                                          NCollection_Array1<double> Tol3d) => Tol3d.Init(SurfTol);

        public override void SetTolerance(double Tol3d, double Tol2d)
        {
        }

        public override Point3D BarycentreOfSurf() => new(0.0, 0.0, 1.0);

        public override double MaximalSection() => 2.0 * Math.PI * Radius(2.0);

        public override void GetMinimalWeight(NCollection_Array1<double> Weigths)
        {
            for (int i = 1; i <= 9; i++)
            {
                Weigths[i] = Weight(i);
            }
        }
    }

    // A non-rational quadratic section (three poles) moved along a sine, without derivatives.
    private sealed class SineSection : SweepFunction
    {
        public static Point3D Pole(int i, double v) => new(i, Math.Sin(v) * (i == 2 ? 1.0 : 0.25), v);

        public override bool D0(double Param, double First, double Last, NCollection_Array1<Point3D> Poles,
                                NCollection_Array1<Point2D> Poles2d, NCollection_Array1<double> Weigths)
        {
            for (int i = 1; i <= 3; i++)
            {
                Poles[i] = Pole(i, Param);
                Weigths[i] = 1.0;
            }
            return true;
        }

        public override int Nb2dCurves() => 0;

        public override void SectionShape(out int NbPoles, out int NbKnots, out int Degree)
        {
            NbPoles = 3;
            NbKnots = 2;
            Degree = 2;
        }

        public override void Knots(NCollection_Array1<double> TKnots)
        {
            TKnots[1] = 0.0;
            TKnots[2] = 1.0;
        }

        public override void Mults(NCollection_Array1<int> TMults)
        {
            TMults[1] = 3;
            TMults[2] = 3;
        }

        public override bool IsRational => false;

        public override int NbIntervals(Continuity S) => 1;

        public override void Intervals(NCollection_Array1<double> T, Continuity S)
        {
            T[1] = 0.0;
            T[2] = 6.0;
        }

        public override void SetInterval(double First, double Last)
        {
        }

        public override void GetTolerance(double BoundTol, double SurfTol, double AngleTol,
                                          NCollection_Array1<double> Tol3d) => Tol3d.Init(SurfTol);

        public override void SetTolerance(double Tol3d, double Tol2d)
        {
        }
    }

    private static BSplineSurface BuildSurface(SweepApproximation theApprox)
    {
        theApprox.SurfShape(out int aUDeg, out int aVDeg, out int aNbUP, out int aNbVP, out int aNbUK, out int aNbVK);
        NCollection_Array2<Point3D> aPoles = new(1, aNbUP, 1, aNbVP);
        NCollection_Array2<double> aWeights = new(1, aNbUP, 1, aNbVP);
        NCollection_Array1<double> aUKnots = new(1, aNbUK), aVKnots = new(1, aNbVK);
        NCollection_Array1<int> aUMults = new(1, aNbUK), aVMults = new(1, aNbVK);
        theApprox.Surface(aPoles, aWeights, aUKnots, aVKnots, aUMults, aVMults);
        Assert.Equal(aUDeg, theApprox.UDegree());
        Assert.Equal(aVDeg, theApprox.VDegree());
        return new BSplineSurface(aPoles, aWeights, aUKnots, aVKnots, aUMults, aVMults, aUDeg, aVDeg);
    }

    [Theory]
    [InlineData(C0, true)]
    [InlineData(C1, true)]
    [InlineData(C2, true)]
    [InlineData(C2, false)]
    public void RationalGrowingCircle_WithA2dCurve(Continuity theContinuity, bool theDerivatives)
    {
        GrowingCircle aFunc = new(true, theDerivatives);
        SweepApproximation anApprox = new(aFunc);
        Assert.False(anApprox.IsDone);
        Assert.Throws<NotDoneException>(() => anApprox.UDegree());
        anApprox.Perform(0.0, 2.0, 1.0e-6, 1.0e-6, 1.0e-5, 1.0e-4, theContinuity, 11, 50);

        Assert.True(anApprox.IsDone);
        Assert.True(anApprox.SurfWeights()[1, 1] > 0.0);
        Assert.Equal(2, anApprox.UDegree());
        Assert.True(anApprox.MaxErrorOnSurf() < 1.0e-5);
        Assert.True(anApprox.AverageErrorOnSurf() <= anApprox.MaxErrorOnSurf());
        Assert.True(aFunc.NbSetInterval > 0);

        BSplineSurface aSurf = BuildSurface(anApprox);
        for (int j = 0; j <= 10; j++)
        {
            double v = 0.2 * j;
            BSplineCurve3D aSection = GrowingCircle.Section(v);
            for (int i = 0; i <= 8; i++)
            {
                double u = 2.0 * Math.PI * i / 8.0;
                Assert.True(aSurf.Value(u, v).Distance(aSection.Value(u)) < 1.0e-5);
            }
        }

        Assert.Equal(1, anApprox.Curves2dCount);
        anApprox.Curves2dShape(out int aDeg2d, out int aNbPoles2d, out int aNbKnots2d);
        Assert.Equal(anApprox.VDegree(), aDeg2d);
        NCollection_Array1<Point2D> aPoles2d = new(1, aNbPoles2d);
        NCollection_Array1<double> aKnots2d = new(1, aNbKnots2d);
        NCollection_Array1<int> aMults2d = new(1, aNbKnots2d);
        anApprox.Curve2d(1, aPoles2d, aKnots2d, aMults2d);
        Assert.Equal(aPoles2d[1], anApprox.Curve2dPoles(1)[1]);
        BSplineCurve2D aC2d = new(aPoles2d, aKnots2d, aMults2d, aDeg2d);
        for (int j = 0; j <= 10; j++)
        {
            double v = 0.2 * j;
            Assert.True(aC2d.Value(v).Distance(new Point2D(v, 2.0 * v)) < 1.0e-6);
        }
        Assert.True(anApprox.Max2dError(1) <= 1.0e-5);
        Assert.True(anApprox.Average2dError(1) <= anApprox.Max2dError(1));
        Assert.True(anApprox.TolCurveOnSurf(1) >= 0.0);
        Assert.Contains("Error 2d = ", anApprox.ToString());
    }

    [Fact]
    public void NonRationalSine_WithoutDerivatives()
    {
        SineSection aFunc = new();
        SweepApproximation anApprox = new(aFunc);
        anApprox.Perform(0.0, 6.0, 1.0e-7, 1.0e-7, 1.0e-5, 1.0e-4);

        Assert.True(anApprox.IsDone);
        Assert.Equal(0, anApprox.Curves2dCount);
        Assert.Throws<DomainErrorException>(() => anApprox.Curves2dDegree());
        Assert.True(anApprox.MaxErrorOnSurf() < 1.0e-6);
        Assert.Equal(1.0, anApprox.SurfWeights()[2, 2]);

        BSplineSurface aSurf = BuildSurface(anApprox);
        Assert.False(aSurf.IsURational);
        for (int j = 0; j <= 12; j++)
        {
            double v = 0.5 * j;
            Point3D P1 = SineSection.Pole(1, v), P2 = SineSection.Pole(2, v), P3 = SineSection.Pole(3, v);
            for (int i = 0; i <= 4; i++)
            {
                double u = 0.25 * i;
                Xyz anExpected = P1.XYZ() * ((1 - u) * (1 - u)) + P2.XYZ() * (2 * u * (1 - u)) + P3.XYZ() * (u * u);
                Assert.True(aSurf.Value(u, v).Distance(new Point3D(anExpected)) < 1.0e-6);
            }
        }
        Assert.Contains("Segment(s) of degree", anApprox.ToString());
    }

    [Fact]
    public void C1WithoutD1_ThrowsNotImplemented()
    {
        // OCCT checks D1 by calling it; the base class's D1 throws.
        SweepApproximation anApprox = new(new SineSection());
        Assert.Throws<MethodNotImplementedException>(
            () => anApprox.Perform(0.0, 6.0, 1.0e-7, 1.0e-7, 1.0e-5, 1.0e-4, C1));
        Assert.Contains(" Not Done ", anApprox.ToString());
    }
}
