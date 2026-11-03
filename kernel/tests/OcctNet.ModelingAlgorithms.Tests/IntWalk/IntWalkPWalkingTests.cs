// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Sanity checks of IntWalk_PWalking on simple pairs (the step-by-step comparison with native recordings is in
/// the unit's native-step tests): every point on both surfaces, the line reaching the domains' boundaries, and the
/// same line bit for bit on every run and from parallel threads (decision 490).
/// </summary>
public class IntWalkPWalkingTests
{
    private const double TolTangency = 1.0e-8;
    private const double Epsilon = 1.0e-6;
    private const double Deflection = 0.1;
    private const double Increment = 0.1;

    private static Adaptor3d_Surface PlaneXY() =>
        new GeomAdaptor_Surface(new Geom_Plane(new CoordinateSystem()), -10.0, 10.0, -10.0, 10.0);

    private static Adaptor3d_Surface PlaneYZ() =>
        new GeomAdaptor_Surface(new Geom_Plane(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), -10.0, 10.0, -10.0, 10.0);

    private static readonly Plane3D PlaneX2 = new(new Point3D(2, 0, 0), new Direction3D(1, 0, 0));

    private static readonly Cylinder Cylinder5 = new(new CoordinateSystem(), 5.0);

    private static (Adaptor3d_Surface Plane, Adaptor3d_Surface Cylinder) PlaneAndCylinder() =>
        (new GeomAdaptor_Surface(new Geom_Plane(PlaneX2), -20.0, 20.0, -20.0, 20.0),
         new GeomAdaptor_Surface(new Geom_CylindricalSurface(Cylinder5), 0.0, 2.0 * Math.PI, -10.0, 10.0));

    private static NCollection_Array1<double> StartOnPlaneAndCylinder()
    {
        Point3D aP = new(2.0, Math.Sqrt(21.0), 0.0);
        global::Kerf.Engine.Geometry.ElSLib.Parameters(PlaneX2, aP, out double u1, out double v1);
        global::Kerf.Engine.Geometry.ElSLib.Parameters(Cylinder5, aP, out double u2, out double v2);
        NCollection_Array1<double> aPar = new(1, 4);
        aPar[1] = u1;
        aPar[2] = v1;
        aPar[3] = u2;
        aPar[4] = v2;
        return aPar;
    }

    private static NCollection_Array1<double> Origin()
    {
        NCollection_Array1<double> aPar = new(1, 4);
        aPar.Init(0.0);
        return aPar;
    }

    private static void AssertOnSurfaces(IntWalk_PWalking theWalk, Adaptor3d_Surface theS1, Adaptor3d_Surface theS2,
                                         double theTol)
    {
        for (int i = 1; i <= theWalk.NbPoints(); i++)
        {
            IntSurf_PntOn2S aP = theWalk.Value(i);
            aP.Parameters(out double u1, out double v1, out double u2, out double v2);
            Assert.True(theS1.Value(u1, v1).Distance(aP.Value()) < theTol, $"point {i} off surface 1");
            Assert.True(theS2.Value(u2, v2).Distance(aP.Value()) < theTol, $"point {i} off surface 2");
        }
    }

    [Fact]
    public void TwoPlanesGiveTheirStraightLine()
    {
        Adaptor3d_Surface aS1 = PlaneXY(), aS2 = PlaneYZ();
        IntWalk_PWalking aWalk = new(aS1, aS2, TolTangency, Epsilon, Deflection, Increment);
        aWalk.Perform(Origin());

        Assert.True(aWalk.IsDone());
        Assert.True(aWalk.NbPoints() >= 2);
        Assert.False(aWalk.IsClosed());
        AssertOnSurfaces(aWalk, aS1, aS2, 1.0e-7);
        for (int i = 1; i <= aWalk.NbPoints(); i++)
        {
            Point3D aP = aWalk.Value(i).Value();
            Assert.True(Math.Abs(aP.X()) < 1.0e-7 && Math.Abs(aP.Z()) < 1.0e-7, $"point {i} off the Y axis");
        }

        // The line runs along Y in both directions from the origin, to the planes' borders at |y| = 10
        double aYmin = double.MaxValue, aYmax = double.MinValue;
        for (int i = 1; i <= aWalk.NbPoints(); i++)
        {
            double y = aWalk.Value(i).Value().Y();
            aYmin = Math.Min(aYmin, y);
            aYmax = Math.Max(aYmax, y);
        }
        Assert.True(aYmin < -9.0 && aYmax > 9.0, $"y in [{aYmin}, {aYmax}]");
    }

    [Fact]
    public void PlaneAndCylinderReachTheBoundary()
    {
        (Adaptor3d_Surface aPlane, Adaptor3d_Surface aCyl) = PlaneAndCylinder();
        IntWalk_PWalking aWalk = new(aPlane, aCyl, TolTangency, Epsilon, Deflection, Increment);
        aWalk.Perform(StartOnPlaneAndCylinder());

        Assert.True(aWalk.IsDone());
        Assert.True(aWalk.NbPoints() >= 2);
        AssertOnSurfaces(aWalk, aPlane, aCyl, 1.0e-6);

        // The line is the generator x = 2, y = sqrt(21): it ends near the cylinder's limits z = -10 and z = 10
        double aZmin = double.MaxValue, aZmax = double.MinValue;
        for (int i = 1; i <= aWalk.NbPoints(); i++)
        {
            Point3D aP = aWalk.Value(i).Value();
            Assert.True(Math.Abs(aP.X() - 2.0) < 1.0e-6 && Math.Abs(aP.Y() - Math.Sqrt(21.0)) < 1.0e-6,
                        $"point {i} off the generator");
            aZmin = Math.Min(aZmin, aP.Z());
            aZmax = Math.Max(aZmax, aP.Z());
        }
        Assert.True(aZmin < -9.0 && aZmax > 9.0, $"z in [{aZmin}, {aZmax}]");

        // PutToBoundary leaves a line already on the boundary within 1e-3 of the range as it is or completes it
        aWalk.PutToBoundary(aPlane, aCyl);
        AssertOnSurfaces(aWalk, aPlane, aCyl, 1.0e-6);
    }

    [Fact]
    public void StepTraceSeesEveryTestedStep()
    {
        (Adaptor3d_Surface aPlane, Adaptor3d_Surface aCyl) = PlaneAndCylinder();
        IntWalk_PWalking aWalk = new(aPlane, aCyl, TolTangency, Epsilon, Deflection, Increment);
        List<(IntWalk_StatusDeflection Status, IntSurf_PntOn2S Point)> aSteps = [];
        aWalk.StepTrace = (s, p) => aSteps.Add((s, p));
        aWalk.Perform(StartOnPlaneAndCylinder());

        Assert.NotEmpty(aSteps);
        // Every point of the line but the first was accepted after a deflection test
        Assert.True(aSteps.Count >= aWalk.NbPoints() - 1);
    }

    private static List<double> Signature(Adaptor3d_Surface theS1, Adaptor3d_Surface theS2,
                                          NCollection_Array1<double> theStart)
    {
        IntWalk_PWalking aWalk = new(theS1, theS2, TolTangency, Epsilon, Deflection, Increment);
        aWalk.Perform(theStart);
        List<double> aSig = [aWalk.NbPoints()];
        for (int i = 1; i <= aWalk.NbPoints(); i++)
        {
            IntSurf_PntOn2S aP = aWalk.Value(i);
            aP.Parameters(out double u1, out double v1, out double u2, out double v2);
            aSig.AddRange([aP.Value().X(), aP.Value().Y(), aP.Value().Z(), u1, v1, u2, v2]);
        }
        return aSig;
    }

    [Fact]
    public void TheSameLineEveryRunAndInParallel()
    {
        List<double> Run()
        {
            (Adaptor3d_Surface aPlane, Adaptor3d_Surface aCyl) = PlaneAndCylinder();
            return Signature(aPlane, aCyl, StartOnPlaneAndCylinder());
        }

        List<double> aRef = Run();
        for (int k = 0; k < 3; k++)
        {
            Assert.Equal(aRef, Run());
        }

        List<double>[] aParallel = new List<double>[16];
        Parallel.For(0, aParallel.Length, i => aParallel[i] = Run());
        foreach (List<double> aSig in aParallel)
        {
            Assert.Equal(aRef.Count, aSig.Count);
            for (int i = 0; i < aRef.Count; i++)
            {
                Assert.Equal(BitConverter.DoubleToInt64Bits(aRef[i]), BitConverter.DoubleToInt64Bits(aSig[i]));
            }
        }
    }

    [Fact]
    public void AccessorsThrowBeforeDone()
    {
        IntWalk_PWalking aWalk = new(PlaneXY(), PlaneXY(), TolTangency, Epsilon, Deflection, Increment);
        // Coincident planes: tangent everywhere, the walking stops before marching and is not done
        aWalk.Perform(Origin());
        Assert.False(aWalk.IsDone());
        Assert.Throws<NotDoneException>(() => aWalk.NbPoints());
        Assert.Equal(aWalk.MaxStep(0), aWalk.MaxStep(0));
        Assert.ThrowsAny<OutOfRangeException>(() => aWalk.MaxStep(4));
    }
}
