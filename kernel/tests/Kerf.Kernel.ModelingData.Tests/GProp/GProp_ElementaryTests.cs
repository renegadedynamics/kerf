// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

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

        Assert.ThrowsAny<DomainErrorException>(() => aSystem.Add(aP1, 0.0));
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
    // OCCT never uses GProp_CelGProps, GProp_SelGProps or GProp_VelGProps, and their inertia is wrong (see
    // docs/occt-issues.md): the port reproduces it. These tests check the masses and centres of mass against
    // the analytical values where OCCT gets them right, and pin the reproduced errors elsewhere.

    [Fact]
    public void Circle_Full()
    {
        const double R = 2.0;
        var aCirc = new Circle3D(Constants.XOY(), R);
        var aProps = new global::Kerf.Engine.Measurement.CelGProps(aCirc, Constants.Origin());
        Assert.Equal(2.0 * Math.PI * R, aProps.Mass(), Tol);
        AssertPnt(aProps.CentreOfMass(), 0.0, 0.0, 0.0);
        // The right matrix is diag(pi R^3, pi R^3, 2 pi R^3). OCCT builds P V L (eigenvectors scaled by their
        // values) instead of P V L V^T P^T, and math_Jacobi sorts the values, so the columns come out permuted.
        Matrix3x3 aMat = aProps.MatrixOfInertia();
        double aR3 = R * R * R;
        Assert.Equal(2.0 * Math.PI * aR3, aMat.Value(1, 3), Tol);
        Assert.Equal(Math.PI * aR3, aMat.Value(2, 1), Tol);
        Assert.Equal(Math.PI * aR3, aMat.Value(3, 2), Tol);
        Assert.Equal(0.0, aMat.Value(1, 1), Tol);
    }

    [Fact]
    public void Circle_QuarterArc()
    {
        const double R = 3.0;
        var aCirc = new Circle3D(Constants.XOY(), R);
        var aProps = new global::Kerf.Engine.Measurement.CelGProps(aCirc, 0.0, Math.PI / 2.0, Constants.Origin());
        Assert.Equal(Math.PI * R / 2.0, aProps.Mass(), Tol);
        // Centroid of a quarter arc: 2R/pi on each axis.
        AssertPnt(aProps.CentreOfMass(), 2.0 * R / Math.PI, 2.0 * R / Math.PI, 0.0);
    }

    [Fact]
    public void Circle_LocationNotOrigin()
    {
        // OCCT stores the absolute centre in g, which GProp_GProps reads as relative to the location point, so the
        // centre of mass comes out shifted by the location.
        var aCirc = new Circle3D(Constants.XOY(), 1.0);
        var aProps = new global::Kerf.Engine.Measurement.CelGProps(aCirc, new Point3D(1.0, 2.0, 3.0));
        AssertPnt(aProps.CentreOfMass(), 1.0, 2.0, 3.0);
    }

    [Fact]
    public void Line_Segment()
    {
        var aLin = new Line3D(new Point3D(1.0, 2.0, 3.0), new Direction3D(1.0, 0.0, 0.0));
        var aProps = new global::Kerf.Engine.Measurement.CelGProps(aLin, 0.0, 6.0, Constants.Origin());
        Assert.Equal(6.0, aProps.Mass(), Tol);
        AssertPnt(aProps.CentreOfMass(), 4.0, 2.0, 3.0);
        // Thin rod of length L = 6 along x through (., 2, 3): central moments 0, L^3/12 + ..., about the centre.
        Matrix3x3 aMat = aProps.MatrixOfInertia();
        Assert.Equal(0.0, aMat.Value(1, 1), Tol);
        Assert.Equal(18.0, aMat.Value(2, 2), Tol);
        Assert.Equal(0.0, aMat.Value(1, 2), Tol);
        // The rod's central Izz is L^3/12 = 18. OCCT computes the u^2 and u terms of Izz with Ixx's y and z
        // coefficients (alfa2 = Vy Y0 + Vz Z0, alfa3 = Y0^2 + Z0^2), which gives 30 here.
        Assert.Equal(30.0, aMat.Value(3, 3), Tol);
    }

    [Fact]
    public void Cylinder_Lateral()
    {
        const double R = 2.0;
        const double H = 5.0;
        var aCyl = new Cylinder(new CoordinateSystem(Constants.XOY()), R);
        var aProps = new global::Kerf.Engine.Measurement.SelGProps(aCyl, 0.0, 2.0 * Math.PI, 0.0, H, Constants.Origin());
        Assert.Equal(2.0 * Math.PI * R * H, aProps.Mass(), Tol);
        AssertPnt(aProps.CentreOfMass(), 0.0, 0.0, H / 2.0);
    }

    [Fact]
    public void Sphere_Area()
    {
        const double R = 1.5;
        var aSph = new Sphere(new CoordinateSystem(Constants.XOY()), R);
        var aProps = new global::Kerf.Engine.Measurement.SelGProps(aSph, 0.0, 2.0 * Math.PI, -Math.PI / 2.0, Math.PI / 2.0, Constants.Origin());
        Assert.Equal(4.0 * Math.PI * R * R, aProps.Mass(), Tol);
        AssertPnt(aProps.CentreOfMass(), 0.0, 0.0, 0.0);

        // Upper hemisphere: centroid at R/2.
        aProps.Perform(aSph, 0.0, 2.0 * Math.PI, 0.0, Math.PI / 2.0);
        Assert.Equal(2.0 * Math.PI * R * R, aProps.Mass(), Tol);
        AssertPnt(aProps.CentreOfMass(), 0.0, 0.0, R / 2.0);
    }

    [Fact]
    public void Torus_Area()
    {
        const double RMaj = 4.0;
        const double RMin = 1.0;
        var aTor = new Torus(new CoordinateSystem(Constants.XOY()), RMaj, RMin);
        var aProps = new global::Kerf.Engine.Measurement.SelGProps(aTor, 0.0, 2.0 * Math.PI, 0.0, 2.0 * Math.PI, Constants.Origin());
        Assert.Equal(4.0 * Math.PI * Math.PI * RMaj * RMin, aProps.Mass(), Tol);
        AssertPnt(aProps.CentreOfMass(), 0.0, 0.0, 0.0);
    }

    [Fact]
    public void Cone_Lateral()
    {
        // A cone of reference radius R and semi-angle t, from v = 0 to v = L along its generatrix.
        const double R = 1.0;
        const double L = 2.0;
        double t = Math.PI / 6.0;
        var aCone = new Cone(new CoordinateSystem(Constants.XOY()), t, R);
        var aProps = new global::Kerf.Engine.Measurement.SelGProps(aCone, 0.0, 2.0 * Math.PI, 0.0, L, Constants.Origin());
        // The area is pi (R + R2) L; OCCT multiplies it by cos(t).
        double aR2 = R + L * Math.Sin(t);
        Assert.Equal(Math.PI * (R + aR2) * L * Math.Cos(t), aProps.Mass(), Tol);
    }

    [Fact]
    public void Cylinder_Volume()
    {
        const double R = 2.0;
        const double H = 3.0;
        var aCyl = new Cylinder(new CoordinateSystem(Constants.XOY()), R);
        var aProps = new global::Kerf.Engine.Measurement.VelGProps(aCyl, 0.0, 2.0 * Math.PI, 0.0, H, Constants.Origin());
        double aM = Math.PI * R * R * H;
        Assert.Equal(aM, aProps.Mass(), Tol);
        AssertPnt(aProps.CentreOfMass(), 0.0, 0.0, H / 2.0);
        // The axial moment is M R^2 / 2; OCCT gives M R^2.
        Assert.Equal(aM * R * R, aProps.MatrixOfInertia().Value(3, 3), Tol);
    }

    [Fact]
    public void Sphere_Volume()
    {
        const double R = 2.0;
        var aSph = new Sphere(new CoordinateSystem(Constants.XOY()), R);
        var aProps = new global::Kerf.Engine.Measurement.VelGProps(aSph, 0.0, 2.0 * Math.PI, -Math.PI / 2.0, Math.PI / 2.0, Constants.Origin());
        Assert.Equal(4.0 / 3.0 * Math.PI * R * R * R, aProps.Mass(), Tol);
        AssertPnt(aProps.CentreOfMass(), 0.0, 0.0, 0.0);
    }

    [Fact]
    public void Torus_Volume()
    {
        const double RMaj = 3.0;
        const double RMin = 1.0;
        var aTor = new Torus(new CoordinateSystem(Constants.XOY()), RMaj, RMin);
        var aProps = new global::Kerf.Engine.Measurement.VelGProps(aTor, 0.0, 2.0 * Math.PI, 0.0, 2.0 * Math.PI, Constants.Origin());
        Assert.Equal(2.0 * Math.PI * Math.PI * RMaj * RMin * RMin, aProps.Mass(), Tol);
        AssertPnt(aProps.CentreOfMass(), 0.0, 0.0, 0.0);

        // Half a torus (Teta in [0, pi]): the surface class puts the centre at y = 2 RMaj / pi; the volume class
        // reads cos(Alpha2) and sin(Alpha2) for Teta2, so with Alpha2 = 2 pi its centre stays at y = 0.
        var aHalfS = new global::Kerf.Engine.Measurement.SelGProps(aTor, 0.0, Math.PI, 0.0, 2.0 * Math.PI, Constants.Origin());
        Assert.Equal(2.0 * RMaj / Math.PI, aHalfS.CentreOfMass().Y, Tol);
        var aHalfV = new global::Kerf.Engine.Measurement.VelGProps(aTor, 0.0, Math.PI, 0.0, 2.0 * Math.PI, Constants.Origin());
        Assert.Equal(0.0, aHalfV.CentreOfMass().Y, Tol);
    }

    [Fact]
    public void Cone_Volume()
    {
        const double R = 1.0;
        const double L = 2.0;
        double t = Math.PI / 6.0;
        var aCone = new Cone(new CoordinateSystem(Constants.XOY()), t, R);
        var aProps = new global::Kerf.Engine.Measurement.VelGProps(aCone, 0.0, 2.0 * Math.PI, 0.0, L, Constants.Origin());
        // The frustum's volume is pi h (R^2 + R R2 + R2^2) / 3 with h = L cos(t); OCCT computes
        // pi L^2 cos(t) sin(t) (2R + L sin(t)), the lateral area's formula times L sin(t).
        double aExpected = Math.PI * L * L * Math.Cos(t) * Math.Sin(t) * (2.0 * R + L * Math.Sin(t));
        Assert.Equal(aExpected, aProps.Mass(), Tol);
    }
}
