// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// GProp_CelGProps, GProp_SelGProps, GProp_VelGProps, GProp_GProps and GProp_PrincipalProps against analytical
/// results (OCCT has no GTests for them).
/// </summary>
public class GProp_ElementaryTests
{
    private const double Tol = 1e-9;

    private static void AssertMat(Matrix3x3 theMat, double theXX, double theYY, double theZZ,
                                  double theXY = 0.0, double theXZ = 0.0, double theYZ = 0.0, double theTol = Tol)
    {
        Assert.Equal(theXX, theMat.Value(1, 1), theTol);
        Assert.Equal(theYY, theMat.Value(2, 2), theTol);
        Assert.Equal(theZZ, theMat.Value(3, 3), theTol);
        Assert.Equal(theXY, theMat.Value(1, 2), theTol);
        Assert.Equal(theXY, theMat.Value(2, 1), theTol);
        Assert.Equal(theXZ, theMat.Value(1, 3), theTol);
        Assert.Equal(theXZ, theMat.Value(3, 1), theTol);
        Assert.Equal(theYZ, theMat.Value(2, 3), theTol);
        Assert.Equal(theYZ, theMat.Value(3, 2), theTol);
    }

    private static void AssertPnt(Point3D theP, double theX, double theY, double theZ, double theTol = Tol)
    {
        Assert.Equal(theX, theP.X, theTol);
        Assert.Equal(theY, theP.Y, theTol);
        Assert.Equal(theZ, theP.Z, theTol);
    }

    // ---- HOperator and GProp_GProps -------------------------------------------------------------------------

    [Fact]
    public void HOperator_PointMass()
    {
        global::Kerf.Engine.Measurement.GProp.HOperator(new Point3D(1.0, 2.0, 3.0), Constants.Origin(), 2.0, out Matrix3x3 aH);
        AssertMat(aH, 2.0 * 13.0, 2.0 * 10.0, 2.0 * 5.0, -4.0, -6.0, -12.0);
    }

    [Fact]
    public void Add_TranslatesInertia()
    {
        // Two unit points composed through systems with different reference points.
        var aP1 = new global::Kerf.Engine.Measurement.PGProps();
        aP1.AddPoint(new Point3D(1.0, 0.0, 0.0));
        var aP2 = new global::Kerf.Engine.Measurement.PGProps();
        aP2.AddPoint(new Point3D(-1.0, 0.0, 0.0));

        var aSystem = new global::Kerf.Engine.Measurement.Properties(new Point3D(5.0, 5.0, 5.0));
        aSystem.Add(aP1, 2.0);
        aSystem.Add(aP2, 2.0);

        Assert.Equal(4.0, aSystem.Mass(), Tol);
        AssertPnt(aSystem.CentreOfMass(), 0.0, 0.0, 0.0);
        AssertMat(aSystem.MatrixOfInertia(), 0.0, 4.0, 4.0);
        Assert.Equal(4.0, aSystem.MomentOfInertia(new Axis3D(Constants.Origin(), new Direction3D(0.0, 0.0, 1.0))), Tol);
        Assert.Equal(1.0, aSystem.RadiusOfGyration(new Axis3D(Constants.Origin(), new Direction3D(0.0, 1.0, 0.0))), Tol);
        // About a parallel axis through (0, 3, 0): 4 + 4 * 9.
        Assert.Equal(40.0, aSystem.MomentOfInertia(new Axis3D(new Point3D(0.0, 3.0, 0.0), new Direction3D(0.0, 0.0, 1.0))), Tol);
        aSystem.StaticMoments(out double aIx, out double aIy, out double aIz);
        Assert.Equal(0.0, aIx, Tol);
        Assert.Equal(0.0, aIy, Tol);
        Assert.Equal(0.0, aIz, Tol);

        EngineAssert.ThrowsAny<DomainErrorException>(() => aSystem.Add(aP1, 0.0));
    }

    [Fact]
    public void Copy_IsIndependent()
    {
        var aP = new global::Kerf.Engine.Measurement.PGProps();
        aP.AddPoint(new Point3D(1.0, 2.0, 3.0));
        var aCopy = new global::Kerf.Engine.Measurement.Properties(aP);
        aP.AddPoint(new Point3D(3.0, 2.0, 1.0));
        Assert.Equal(1.0, aCopy.Mass());
        AssertPnt(aCopy.CentreOfMass(), 1.0, 2.0, 3.0);
    }

    [Fact]
    public void PrincipalProperties_Symmetries()
    {
        // Six points on the axes: a centre of symmetry.
        var aPnts = new NCollection_Array1<Point3D>(1, 6);
        aPnts[1] = new Point3D(1.0, 0.0, 0.0);
        aPnts[2] = new Point3D(-1.0, 0.0, 0.0);
        aPnts[3] = new Point3D(0.0, 1.0, 0.0);
        aPnts[4] = new Point3D(0.0, -1.0, 0.0);
        aPnts[5] = new Point3D(0.0, 0.0, 1.0);
        aPnts[6] = new Point3D(0.0, 0.0, -1.0);
        global::Kerf.Engine.Measurement.PrincipalProperties aPP = new global::Kerf.Engine.Measurement.PGProps(aPnts).PrincipalProperties();
        Assert.True(aPP.HasSymmetryPoint());
        Assert.True(aPP.HasSymmetryAxis());
        aPP.Moments(out double aI1, out double aI2, out double aI3);
        Assert.Equal(4.0, aI1, Tol);
        Assert.Equal(4.0, aI2, Tol);
        Assert.Equal(4.0, aI3, Tol);
        aPP.RadiusOfGyration(out double aR1, out _, out _);
        Assert.Equal(Math.Sqrt(4.0 / 6.0), aR1, Tol);

        // A rotated rectangle of points: no symmetry axis, principal axes along the rectangle.
        var aRot = new Transform3D();
        aRot = aRot.WithRotation(new Axis3D(Constants.Origin(), new Direction3D(0.0, 0.0, 1.0)), 0.3);
        var aRect = new NCollection_Array1<Point3D>(1, 4);
        aRect[1] = new Point3D(2.0, 1.0, 0.0).Transformed(aRot);
        aRect[2] = new Point3D(-2.0, 1.0, 0.0).Transformed(aRot);
        aRect[3] = new Point3D(2.0, -1.0, 0.0).Transformed(aRot);
        aRect[4] = new Point3D(-2.0, -1.0, 0.0).Transformed(aRot);
        aPP = new global::Kerf.Engine.Measurement.PGProps(aRect).PrincipalProperties();
        Assert.False(aPP.HasSymmetryAxis());
        Assert.False(aPP.HasSymmetryPoint());
        aPP.Moments(out aI1, out aI2, out aI3);
        double[] aMoments = [aI1, aI2, aI3];
        Array.Sort(aMoments);
        Assert.Equal(4.0, aMoments[0], Tol);  // about the long axis: 4 * 1
        Assert.Equal(16.0, aMoments[1], Tol); // about the short axis: 4 * 4
        Assert.Equal(20.0, aMoments[2], Tol); // about z
        Vector3D aLong = new Vector3D(1.0, 0.0, 0.0).Transformed(aRot);
        Vector3D[] anAxes = [aPP.FirstAxisOfInertia(), aPP.SecondAxisOfInertia(), aPP.ThirdAxisOfInertia()];
        double[] aRaw = [aI1, aI2, aI3];
        for (int i = 0; i < 3; i++)
        {
            if (Math.Abs(aRaw[i] - 4.0) < Tol)
            {
                Assert.Equal(1.0, Math.Abs(anAxes[i].Dot(aLong)), Tol);
            }
        }
    }

    [Fact]
    public void PrincipalProps_Default()
    {
        var aPP = new global::Kerf.Engine.Measurement.PrincipalProperties();
        aPP.Moments(out double aI1, out _, out _);
        Assert.Equal(RealLast(), aI1);
        Assert.Equal(new Vector3D(0.0, 1.0, 0.0), aPP.SecondAxisOfInertia());
        Assert.Equal(aPP, new global::Kerf.Engine.Measurement.PrincipalProperties());
    }

    // ---- Curves, surfaces and solids ---------------------------------------------------------------------------
    //
    // GProp_CelGProps, GProp_SelGProps and GProp_VelGProps against closed forms (rings, rods, shells and solids of
    // cylinders, cones, spheres and tori) and, for partial patches, against the integrals computed here by Simpson's
    // rule. OCCT got their matrices of inertia wrong (P V L for P V L V^T P^T), kept the centres absolute where
    // GProp_GProps reads them relative to the location point, and had wrong line Izz, cone area and volume, cylinder
    // axial moment and torus centre (occt-issues.md, unit 1's GProp rows; decision 1471).

    private static readonly CoordinateSystem Tilted = new(new Point3D(1.0, -2.0, 0.5), new Direction3D(0.2, -0.3, 1.0), new Direction3D(1.0, 0.0, -0.2));
    private static readonly Point3D Loc = new(3.0, 1.0, -2.0);

    private static Axis3D AxisOf(CoordinateSystem theFrame) => new(theFrame.Location, theFrame.Direction);

    // An axis through the centre of mass perpendicular to the frame's axis.
    private static Axis3D Across(CoordinateSystem theFrame, Point3D theCentre) => new(theCentre, theFrame.XDirection);

    // The mass, centre and central matrix of inertia (absolute axes) of the measure theJ on the parameter box, by
    // Simpson's rule with theN intervals on each side that isn't degenerate.
    private static (double M, Point3D G, double[,] I) Integrate(Func<double, double, double, Point3D> thePoint,
                                                               Func<double, double, double, double> theJ,
                                                               double theA1, double theA2, double theB1, double theB2,
                                                               double theC1, double theC2, int theN = 60)
    {
        static (double[] X, double[] W) Rule(double t1, double t2, int n)
        {
            if (t1 == t2)
            {
                return ([t1], [1.0]);
            }
            double h = (t2 - t1) / n;
            double[] x = new double[n + 1], w = new double[n + 1];
            for (int i = 0; i <= n; i++)
            {
                x[i] = t1 + i * h;
                w[i] = h / 3.0 * (i == 0 || i == n ? 1.0 : i % 2 == 1 ? 4.0 : 2.0);
            }
            return (x, w);
        }

        (double[] aX, double[] aWa) = Rule(theA1, theA2, theN);
        (double[] aY, double[] aWb) = Rule(theB1, theB2, theN);
        (double[] aZ, double[] aWc) = Rule(theC1, theC2, theN);
        double aM = 0.0;
        double[] aS1 = new double[3];
        double[,] aS2 = new double[3, 3];
        for (int i = 0; i < aX.Length; i++)
        {
            for (int j = 0; j < aY.Length; j++)
            {
                for (int k = 0; k < aZ.Length; k++)
                {
                    double w = aWa[i] * aWb[j] * aWc[k] * theJ(aX[i], aY[j], aZ[k]);
                    Point3D p = thePoint(aX[i], aY[j], aZ[k]);
                    double[] q = [p.X, p.Y, p.Z];
                    aM += w;
                    for (int r = 0; r < 3; r++)
                    {
                        aS1[r] += w * q[r];
                        for (int c = 0; c < 3; c++)
                        {
                            aS2[r, c] += w * q[r] * q[c];
                        }
                    }
                }
            }
        }
        double[] g = [aS1[0] / aM, aS1[1] / aM, aS1[2] / aM];
        double[,] aCentral = new double[3, 3];
        for (int r = 0; r < 3; r++)
        {
            for (int c = 0; c < 3; c++)
            {
                aCentral[r, c] = aS2[r, c] - aM * g[r] * g[c];
            }
        }
        double aTrace = aCentral[0, 0] + aCentral[1, 1] + aCentral[2, 2];
        double[,] anI = new double[3, 3];
        for (int r = 0; r < 3; r++)
        {
            for (int c = 0; c < 3; c++)
            {
                anI[r, c] = (r == c ? aTrace : 0.0) - aCentral[r, c];
            }
        }
        return (aM, new Point3D(g[0], g[1], g[2]), anI);
    }

    private static void AssertSame(global::Kerf.Engine.Measurement.Properties theProps,
                                   (double M, Point3D G, double[,] I) theRef, double theRelTol = 1e-6)
    {
        double aScale = Math.Abs(theRef.M);
        Assert.Equal(theRef.M, theProps.Mass(), theRelTol * aScale);
        Assert.True(theProps.CentreOfMass().DistanceTo(theRef.G) < 1e-5, $"centre {theProps.CentreOfMass().X} {theProps.CentreOfMass().Y} {theProps.CentreOfMass().Z}");
        Matrix3x3 aMat = theProps.MatrixOfInertia();
        double aNorm = Math.Abs(theRef.I[0, 0]) + Math.Abs(theRef.I[1, 1]) + Math.Abs(theRef.I[2, 2]);
        for (int r = 0; r < 3; r++)
        {
            for (int c = 0; c < 3; c++)
            {
                Assert.True(Math.Abs(theRef.I[r, c] - aMat.Value(r + 1, c + 1)) < theRelTol * aNorm,
                            $"I({r + 1}, {c + 1}): {aMat.Value(r + 1, c + 1)}, expected {theRef.I[r, c]}");
            }
        }
    }

    // The point (w cos u, w sin u, h) of the frame
    private static Point3D InFrame(CoordinateSystem theFrame, double theW, double theU, double theH)
        => theFrame.Location.Translated(new Vector3D(theFrame.XDirection) * (theW * Math.Cos(theU))
                                          + new Vector3D(theFrame.YDirection) * (theW * Math.Sin(theU))
                                          + new Vector3D(theFrame.Direction) * theH);

    [Fact]
    public void Circle_ring_and_arc()
    {
        const double R = 2.0;
        var aCirc = new Circle3D(new Frame3D(Tilted.Location, Tilted.Direction, Tilted.XDirection), R);
        var aProps = new global::Kerf.Engine.Measurement.CelGProps(aCirc, Loc);
        double m = 2.0 * Math.PI * R;
        Assert.Equal(m, aProps.Mass(), Tol);
        Assert.True(aProps.CentreOfMass().DistanceTo(Tilted.Location) < Tol);
        // a ring: m R^2 about its axis, m R^2 / 2 about a diameter
        Assert.Equal(m * R * R, aProps.MomentOfInertia(AxisOf(Tilted)), Tol);
        Assert.Equal(m * R * R / 2.0, aProps.MomentOfInertia(Across(Tilted, Tilted.Location)), Tol);

        // a quarter arc: its centroid at 2 R sqrt(2) / pi from the centre, on the bisector
        var anArc = new global::Kerf.Engine.Measurement.CelGProps(new Circle3D(Constants.XOY(), 3.0), 0.0, Math.PI / 2.0, Loc);
        Assert.Equal(1.5 * Math.PI, anArc.Mass(), Tol);
        AssertPnt(anArc.CentreOfMass(), 6.0 / Math.PI, 6.0 / Math.PI, 0.0);

        aProps.Perform(aCirc, 0.4, 2.9);
        AssertSame(aProps, Integrate((u, _, _) => InFrame(Tilted, R, u, 0.0), (_, _, _) => R, 0.4, 2.9, 0, 0, 0, 0, 200));
    }

    [Fact]
    public void Line_rod()
    {
        // a rod of length 6 along X from (1, 2, 3): central moments 0 about its axis, L^3 / 12 = 18 across it (OCCT
        // computed Izz with Ixx's coefficients: 30)
        var aLin = new Line3D(new Point3D(1.0, 2.0, 3.0), new Direction3D(1.0, 0.0, 0.0));
        var aProps = new global::Kerf.Engine.Measurement.CelGProps(aLin, 0.0, 6.0, Loc);
        Assert.Equal(6.0, aProps.Mass(), Tol);
        AssertPnt(aProps.CentreOfMass(), 4.0, 2.0, 3.0);
        AssertMat(aProps.MatrixOfInertia(), 0.0, 18.0, 18.0);

        // a tilted one
        var aTilted = new Line3D(Tilted.Location, Tilted.Direction);
        var aT = new global::Kerf.Engine.Measurement.CelGProps(aTilted, -1.0, 3.0, Loc);
        Assert.Equal(4.0, aT.Mass(), Tol);
        Assert.True(aT.CentreOfMass().DistanceTo(Tilted.Location.Translated(new Vector3D(Tilted.Direction))) < Tol);
        Assert.Equal(0.0, aT.MomentOfInertia(new Axis3D(aT.CentreOfMass(), Tilted.Direction)), Tol);
        Assert.Equal(64.0 / 12.0, aT.MomentOfInertia(new Axis3D(aT.CentreOfMass(), Tilted.XDirection)), Tol);
    }

    [Fact]
    public void Cylinder_shell_and_solid()
    {
        const double R = 2.0, H = 5.0;
        var aCyl = new Cylinder3D(Tilted, R);
        Point3D aCentre = Tilted.Location.Translated(new Vector3D(Tilted.Direction) * (H / 2.0));

        var aShell = new global::Kerf.Engine.Measurement.SelGProps(aCyl, 0.0, 2.0 * Math.PI, 0.0, H, Loc);
        double m = 2.0 * Math.PI * R * H;
        Assert.Equal(m, aShell.Mass(), Tol);
        Assert.True(aShell.CentreOfMass().DistanceTo(aCentre) < Tol);
        Assert.Equal(m * R * R, aShell.MomentOfInertia(AxisOf(Tilted)), 1e-8);
        Assert.Equal(m * (R * R / 2.0 + H * H / 12.0), aShell.MomentOfInertia(Across(Tilted, aCentre)), 1e-8);

        // the solid's axial moment is m R^2 / 2 (OCCT: m R^2)
        var aSolid = new global::Kerf.Engine.Measurement.VelGProps(aCyl, 0.0, 2.0 * Math.PI, 0.0, H, Loc);
        double v = Math.PI * R * R * H;
        Assert.Equal(v, aSolid.Mass(), Tol);
        Assert.True(aSolid.CentreOfMass().DistanceTo(aCentre) < Tol);
        Assert.Equal(v * R * R / 2.0, aSolid.MomentOfInertia(AxisOf(Tilted)), 1e-8);
        Assert.Equal(v * (3.0 * R * R + H * H) / 12.0, aSolid.MomentOfInertia(Across(Tilted, aCentre)), 1e-8);

        aShell.Perform(aCyl, 0.5, 2.5, -1.0, 2.0);
        AssertSame(aShell, Integrate((u, z, _) => InFrame(Tilted, R, u, z), (_, _, _) => R, 0.5, 2.5, -1.0, 2.0, 0, 0));
        aSolid.Perform(aCyl, 0.5, 2.5, -1.0, 2.0);
        AssertSame(aSolid, Integrate((u, z, r) => InFrame(Tilted, r, u, z), (_, _, r) => r, 0.5, 2.5, -1.0, 2.0, 0.0, R, 40));
    }

    [Fact]
    public void Cone_shell_and_solid()
    {
        // a cone from its apex (reference radius 0) along its generatrix to L: height h = L cos t, base radius
        // a = L sin t
        double t = Math.PI / 6.0, L = 2.0, h = L * Math.Cos(t), a = L * Math.Sin(t);
        var aCone = new Cone3D(Tilted, t, 0.0);
        Point3D Axial(double theZ) => Tilted.Location.Translated(new Vector3D(Tilted.Direction) * theZ);

        // the lateral area pi a L (OCCT multiplied it by cos t), its centroid at 2 h / 3
        var aShell = new global::Kerf.Engine.Measurement.SelGProps(aCone, 0.0, 2.0 * Math.PI, 0.0, L, Loc);
        Assert.Equal(Math.PI * a * L, aShell.Mass(), Tol);
        Assert.True(aShell.CentreOfMass().DistanceTo(Axial(2.0 * h / 3.0)) < Tol);
        Assert.Equal(aShell.Mass() * a * a / 2.0, aShell.MomentOfInertia(AxisOf(Tilted)), 1e-9);

        // the solid: pi a^2 h / 3, its centroid at 3 h / 4, moments 3 m a^2 / 10 about the axis and
        // m (3 a^2 / 20 + 3 h^2 / 80) across it (OCCT's volume was pi L^2 cos t sin t (2R + L sin t))
        var aSolid = new global::Kerf.Engine.Measurement.VelGProps(aCone, 0.0, 2.0 * Math.PI, 0.0, L, Loc);
        double m = Math.PI * a * a * h / 3.0;
        Assert.Equal(m, aSolid.Mass(), Tol);
        Point3D aCentre = Axial(3.0 * h / 4.0);
        Assert.True(aSolid.CentreOfMass().DistanceTo(aCentre) < Tol);
        Assert.Equal(3.0 * m * a * a / 10.0, aSolid.MomentOfInertia(AxisOf(Tilted)), 1e-9);
        Assert.Equal(m * (3.0 * a * a / 20.0 + 3.0 * h * h / 80.0), aSolid.MomentOfInertia(Across(Tilted, aCentre)), 1e-9);

        // a frustum's patch
        double R0 = 1.0, sn = Math.Sin(t), cs = Math.Cos(t);
        var aFrustum = new Cone3D(Tilted, t, R0);
        aShell.Perform(aFrustum, 0.5, 2.5, -0.5, 1.5);
        AssertSame(aShell, Integrate((u, v, _) => InFrame(Tilted, R0 + v * sn, u, v * cs), (_, v, _) => R0 + v * sn, 0.5, 2.5, -0.5, 1.5, 0, 0));
        aSolid.Perform(aFrustum, 0.5, 2.5, -0.5, 1.5);
        AssertSame(aSolid, Integrate((u, v, s) => InFrame(Tilted, s * (R0 + v * sn), u, v * cs),
                                     (_, v, s) => (R0 + v * sn) * (R0 + v * sn) * s * cs, 0.5, 2.5, -0.5, 1.5, 0.0, 1.0, 40));
    }

    [Fact]
    public void Sphere_shell_and_solid()
    {
        const double R = 1.5;
        var aSph = new Sphere3D(Tilted, R);
        var aShell = new global::Kerf.Engine.Measurement.SelGProps(aSph, 0.0, 2.0 * Math.PI, -Math.PI / 2.0, Math.PI / 2.0, Loc);
        double m = 4.0 * Math.PI * R * R;
        Assert.Equal(m, aShell.Mass(), Tol);
        Assert.True(aShell.CentreOfMass().DistanceTo(Tilted.Location) < Tol);
        Assert.Equal(2.0 / 3.0 * m * R * R, aShell.MomentOfInertia(AxisOf(Tilted)), 1e-9);
        Assert.Equal(2.0 / 3.0 * m * R * R, aShell.MomentOfInertia(Across(Tilted, Tilted.Location)), 1e-9);

        // the upper hemisphere's centroid at R / 2
        aShell.Perform(aSph, 0.0, 2.0 * Math.PI, 0.0, Math.PI / 2.0);
        Assert.True(aShell.CentreOfMass().DistanceTo(Tilted.Location.Translated(new Vector3D(Tilted.Direction) * (R / 2.0))) < Tol);

        var aSolid = new global::Kerf.Engine.Measurement.VelGProps(aSph, 0.0, 2.0 * Math.PI, -Math.PI / 2.0, Math.PI / 2.0, Loc);
        double v = 4.0 / 3.0 * Math.PI * R * R * R;
        Assert.Equal(v, aSolid.Mass(), Tol);
        Assert.True(aSolid.CentreOfMass().DistanceTo(Tilted.Location) < Tol);
        Assert.Equal(0.4 * v * R * R, aSolid.MomentOfInertia(AxisOf(Tilted)), 1e-9);

        aShell.Perform(aSph, 0.5, 2.5, -0.3, 1.2);
        AssertSame(aShell, Integrate((u, w, _) => InFrame(Tilted, R * Math.Cos(w), u, R * Math.Sin(w)), (_, w, _) => R * R * Math.Cos(w),
                                     0.5, 2.5, -0.3, 1.2, 0, 0));
        aSolid.Perform(aSph, 0.5, 2.5, -0.3, 1.2);
        AssertSame(aSolid, Integrate((u, w, r) => InFrame(Tilted, r * Math.Cos(w), u, r * Math.Sin(w)), (_, w, r) => r * r * Math.Cos(w),
                                     0.5, 2.5, -0.3, 1.2, 0.0, R, 40));
    }

    [Fact]
    public void Torus_shell_and_solid()
    {
        const double RMaj = 3.0, RMin = 1.0;
        var aTor = new Torus3D(Tilted, RMaj, RMin);

        // the shell: 4 pi^2 R r, moments m (R^2 + 3 r^2 / 2) about the axis, m (R^2 / 2 + 5 r^2 / 4) across it
        var aShell = new global::Kerf.Engine.Measurement.SelGProps(aTor, 0.0, 2.0 * Math.PI, 0.0, 2.0 * Math.PI, Loc);
        double m = 4.0 * Math.PI * Math.PI * RMaj * RMin;
        Assert.Equal(m, aShell.Mass(), 1e-9);
        Assert.True(aShell.CentreOfMass().DistanceTo(Tilted.Location) < Tol);
        Assert.Equal(m * (RMaj * RMaj + 1.5 * RMin * RMin), aShell.MomentOfInertia(AxisOf(Tilted)), 1e-8);
        Assert.Equal(m * (RMaj * RMaj / 2.0 + 1.25 * RMin * RMin), aShell.MomentOfInertia(Across(Tilted, Tilted.Location)), 1e-8);

        // the solid: 2 pi^2 R r^2, moments m (R^2 + 3 r^2 / 4) and m (R^2 / 2 + 5 r^2 / 8)
        var aSolid = new global::Kerf.Engine.Measurement.VelGProps(aTor, 0.0, 2.0 * Math.PI, 0.0, 2.0 * Math.PI, Loc);
        double v = 2.0 * Math.PI * Math.PI * RMaj * RMin * RMin;
        Assert.Equal(v, aSolid.Mass(), 1e-9);
        Assert.True(aSolid.CentreOfMass().DistanceTo(Tilted.Location) < Tol);
        Assert.Equal(v * (RMaj * RMaj + 0.75 * RMin * RMin), aSolid.MomentOfInertia(AxisOf(Tilted)), 1e-8);
        Assert.Equal(v * (RMaj * RMaj / 2.0 + 0.625 * RMin * RMin), aSolid.MomentOfInertia(Across(Tilted, Tilted.Location)), 1e-8);

        // half of it, u in [0, pi]: the shell's centroid at 2 (R + r^2 / (2R)) / pi from the axis, the solid's at
        // 2 (R + r^2 / (4R)) / pi (OCCT's solid kept it on the axis, reading cos and sin of alpha2 for teta2)
        Vector3D aY = new(Tilted.YDirection);
        aShell.Perform(aTor, 0.0, Math.PI, 0.0, 2.0 * Math.PI);
        Assert.True(aShell.CentreOfMass().DistanceTo(Tilted.Location.Translated(aY * (2.0 * (RMaj + RMin * RMin / (2.0 * RMaj)) / Math.PI))) < Tol);
        aSolid.Perform(aTor, 0.0, Math.PI, 0.0, 2.0 * Math.PI);
        Assert.True(aSolid.CentreOfMass().DistanceTo(Tilted.Location.Translated(aY * (2.0 * (RMaj + RMin * RMin / (4.0 * RMaj)) / Math.PI))) < Tol);

        aShell.Perform(aTor, 0.5, 2.5, -0.7, 2.0);
        AssertSame(aShell, Integrate((u, w, _) => InFrame(Tilted, RMaj + RMin * Math.Cos(w), u, RMin * Math.Sin(w)),
                                     (_, w, _) => RMin * (RMaj + RMin * Math.Cos(w)), 0.5, 2.5, -0.7, 2.0, 0, 0));
        aSolid.Perform(aTor, 0.5, 2.5, -0.7, 2.0);
        AssertSame(aSolid, Integrate((u, w, r) => InFrame(Tilted, RMaj + r * Math.Cos(w), u, r * Math.Sin(w)),
                                     (_, w, r) => r * (RMaj + r * Math.Cos(w)), 0.5, 2.5, -0.7, 2.0, 0.0, RMin, 40));
    }
}
