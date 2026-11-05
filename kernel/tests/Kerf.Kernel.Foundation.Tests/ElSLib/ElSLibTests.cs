// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Kernel.Foundation.Tests;

/// <summary>
/// OCCT has no GTests for ElSLib. These check the port against the parametrisations it documents: round trips
/// through Parameters, derivatives against finite differences and closed forms, and the isoparametric curves.
/// </summary>
public class ElSLibTests
{
    private const double Tol = 1e-9;

    private const double SAngle = 0.4;
    private const double Radius = 3.0;
    private const double MajorRadius = 5.0;
    private const double MinorRadius = 1.5;

    /// <summary>A few coordinate systems, two of them left-handed.</summary>
    public static TheoryData<int> Systems => new() { 0, 1, 2, 3 };

    private static CoordinateSystem System(int theIndex)
    {
        CoordinateSystem anAx3 = theIndex switch
        {
            0 => new CoordinateSystem(),
            1 => new CoordinateSystem(new Point3D(1.0, -2.0, 3.5), new Direction3D(0.3, 0.5, 0.8), new Direction3D(1.0, 0.2, -0.5)),
            2 => new CoordinateSystem(new Point3D(-4.0, 0.5, 2.0), new Direction3D(-0.6, 0.1, 0.4), new Direction3D(0.2, 1.0, 0.1)),
            _ => new CoordinateSystem(new Point3D(7.0, 7.0, -1.0), new Direction3D(0.0, 0.0, 1.0), new Direction3D(1.0, 0.0, 0.0)),
        };
        return theIndex >= 2 ? anAx3.YReversed() : anAx3;
    }

    /// <summary>
    /// Angles equal modulo 2*PI: Parameters returns a U just below 2*PI for a point at U = 0 when rounding makes
    /// atan2 slightly negative (below -Precision::Computational()), as OCCT does.
    /// </summary>
    private static void AssertSameAngle(double theExpected, double theActual)
    {
        double aDiff = Math.IEEERemainder(theActual - theExpected, 2 * Math.PI);
        Assert.True(Math.Abs(aDiff) <= Tol, $"expected {theExpected}, got {theActual}");
        Assert.InRange(theActual, 0.0, 2 * Math.PI);
    }

    private static void AssertNear(in global::Kerf.Engine.Geometry.Xyz theExpected, in global::Kerf.Engine.Geometry.Xyz theActual, double theTol = Tol)
    {
        Assert.Equal(theExpected.X(), theActual.X(), theTol);
        Assert.Equal(theExpected.Y(), theActual.Y(), theTol);
        Assert.Equal(theExpected.Z(), theActual.Z(), theTol);
    }

    // Each surface as a value function, so the tests can loop over them

    private delegate Point3D SurfaceValue(double theU, double theV);

    private static IEnumerable<(string Name, SurfaceValue Value)> Surfaces(CoordinateSystem thePos)
    {
        yield return ("plane", (u, v) => global::Kerf.Engine.Geometry.ElSLib.PlaneValue(u, v, thePos));
        yield return ("cylinder", (u, v) => global::Kerf.Engine.Geometry.ElSLib.CylinderValue(u, v, thePos, Radius));
        yield return ("cone", (u, v) => global::Kerf.Engine.Geometry.ElSLib.ConeValue(u, v, thePos, Radius, SAngle));
        yield return ("sphere", (u, v) => global::Kerf.Engine.Geometry.ElSLib.SphereValue(u, v, thePos, Radius));
        yield return ("torus", (u, v) => global::Kerf.Engine.Geometry.ElSLib.TorusValue(u, v, thePos, MajorRadius, MinorRadius));
    }

    [Theory]
    [MemberData(nameof(Systems))]
    public void Value_then_Parameters_round_trip(int theSystem)
    {
        CoordinateSystem aPos = System(theSystem);
        double[] aUs = [0.0, 0.3, 1.7, 3.0, 4.4, 6.1];
        double[] aVs = [-1.2, -0.4, 0.0, 0.5, 1.3];
        foreach (double aU in aUs)
        {
            foreach (double aV in aVs)
            {
                global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(aPos, global::Kerf.Engine.Geometry.ElSLib.PlaneValue(aU, aV, aPos), out double aPU, out double aPV);
                Assert.Equal(aU, aPU, Tol);
                Assert.Equal(aV, aPV, Tol);

                global::Kerf.Engine.Geometry.ElSLib.CylinderParameters(aPos, Radius, global::Kerf.Engine.Geometry.ElSLib.CylinderValue(aU, aV, aPos, Radius), out aPU, out aPV);
                AssertSameAngle(aU, aPU);
                Assert.Equal(aV, aPV, Tol);

                global::Kerf.Engine.Geometry.ElSLib.ConeParameters(aPos, Radius, SAngle, global::Kerf.Engine.Geometry.ElSLib.ConeValue(aU, aV, aPos, Radius, SAngle), out aPU, out aPV);
                AssertSameAngle(aU, aPU);
                Assert.Equal(aV, aPV, Tol);

                global::Kerf.Engine.Geometry.ElSLib.SphereParameters(aPos, Radius, global::Kerf.Engine.Geometry.ElSLib.SphereValue(aU, aV, aPos, Radius), out aPU, out aPV);
                AssertSameAngle(aU, aPU);
                Assert.Equal(aV, aPV, Tol);

                // The torus' V is returned in [0, 2*PI]
                global::Kerf.Engine.Geometry.ElSLib.TorusParameters(aPos, MajorRadius, MinorRadius,
                                       global::Kerf.Engine.Geometry.ElSLib.TorusValue(aU, aV, aPos, MajorRadius, MinorRadius), out aPU, out aPV);
                AssertSameAngle(aU, aPU);
                AssertSameAngle(aV, aPV);
            }
        }
    }

    [Fact]
    public void Parameters_normalise_and_handle_poles()
    {
        CoordinateSystem aPos = new();

        // A sphere's poles have U = 0
        global::Kerf.Engine.Geometry.ElSLib.SphereParameters(aPos, Radius, new Point3D(0, 0, Radius), out double aU, out double aV);
        Assert.Equal(0.0, aU);
        Assert.Equal(M_PI_2, aV);
        global::Kerf.Engine.Geometry.ElSLib.SphereParameters(aPos, Radius, new Point3D(0, 0, -Radius), out aU, out aV);
        Assert.Equal(0.0, aU);
        Assert.Equal(-M_PI_2, aV);

        // The cone's apex has U = 0, and a point beyond the apex (on the other nappe) keeps its U
        double anApexV = -Radius / Math.Sin(SAngle);
        global::Kerf.Engine.Geometry.ElSLib.ConeParameters(aPos, Radius, SAngle, global::Kerf.Engine.Geometry.ElSLib.ConeValue(1.0, anApexV, aPos, Radius, SAngle), out aU, out aV);
        Assert.Equal(0.0, aU);
        Assert.Equal(anApexV, aV, Tol);
        Point3D aBeyond = global::Kerf.Engine.Geometry.ElSLib.ConeValue(1.0, anApexV - 2.0, aPos, Radius, SAngle);
        global::Kerf.Engine.Geometry.ElSLib.ConeParameters(aPos, Radius, SAngle, aBeyond, out aU, out aV);
        Assert.Equal(1.0, aU, Tol);
        Assert.Equal(anApexV - 2.0, aV, Tol);

        // A tiny negative angle becomes 0 rather than 2*PI; 2*PI itself is kept
        global::Kerf.Engine.Geometry.ElSLib.CylinderParameters(aPos, Radius, new Point3D(Radius, -1e-17, 0.0), out aU, out _);
        Assert.Equal(0.0, aU);
        global::Kerf.Engine.Geometry.ElSLib.CylinderParameters(aPos, Radius, new Point3D(Radius, -1e-3, 0.0), out aU, out _);
        Assert.Equal(2 * Math.PI - 1e-3 / Radius, aU, 1e-9);
    }

    [Fact]
    public void TorusParameters_with_major_radius_below_minor_radius()
    {
        // A spindle torus: the point is nearer the tube centred on the opposite side of the axis
        CoordinateSystem aPos = new();
        const double aMajor = 1.0, aMinor = 3.0;
        foreach (double aU in new[] { 0.2, 2.0, 4.0 })
        {
            foreach (double aV in new[] { 0.1, 1.0, 2.0 })
            {
                Point3D aP = global::Kerf.Engine.Geometry.ElSLib.TorusValue(aU, aV, aPos, aMajor, aMinor);
                global::Kerf.Engine.Geometry.ElSLib.TorusParameters(aPos, aMajor, aMinor, aP, out double aPU, out double aPV);
                AssertNear(aP.XYZ(), global::Kerf.Engine.Geometry.ElSLib.TorusValue(aPU, aPV, aPos, aMajor, aMinor).XYZ());
            }
        }
    }

    [Theory]
    [MemberData(nameof(Systems))]
    public void D0_and_gp_overloads_match_Value(int theSystem)
    {
        CoordinateSystem aPos = System(theSystem);
        const double aU = 1.1, aV = 0.7;

        global::Kerf.Engine.Geometry.ElSLib.PlaneD0(aU, aV, aPos, out Point3D aP);
        Assert.Equal(global::Kerf.Engine.Geometry.ElSLib.PlaneValue(aU, aV, aPos), aP);
        global::Kerf.Engine.Geometry.ElSLib.D0(aU, aV, new Plane3D(aPos), out aP);
        Assert.Equal(global::Kerf.Engine.Geometry.ElSLib.PlaneValue(aU, aV, aPos), aP);
        Assert.Equal(global::Kerf.Engine.Geometry.ElSLib.PlaneValue(aU, aV, aPos), global::Kerf.Engine.Geometry.ElSLib.Value(aU, aV, new Plane3D(aPos)));

        global::Kerf.Engine.Geometry.ElSLib.CylinderD0(aU, aV, aPos, Radius, out aP);
        Assert.Equal(global::Kerf.Engine.Geometry.ElSLib.CylinderValue(aU, aV, aPos, Radius), aP);
        global::Kerf.Engine.Geometry.ElSLib.D0(aU, aV, new Cylinder3D(aPos, Radius), out aP);
        Assert.Equal(global::Kerf.Engine.Geometry.ElSLib.CylinderValue(aU, aV, aPos, Radius), aP);
        Assert.Equal(global::Kerf.Engine.Geometry.ElSLib.CylinderValue(aU, aV, aPos, Radius), global::Kerf.Engine.Geometry.ElSLib.Value(aU, aV, new Cylinder3D(aPos, Radius)));

        global::Kerf.Engine.Geometry.ElSLib.ConeD0(aU, aV, aPos, Radius, SAngle, out aP);
        Assert.Equal(global::Kerf.Engine.Geometry.ElSLib.ConeValue(aU, aV, aPos, Radius, SAngle), aP);
        global::Kerf.Engine.Geometry.ElSLib.D0(aU, aV, new Cone3D(aPos, SAngle, Radius), out aP);
        Assert.Equal(global::Kerf.Engine.Geometry.ElSLib.ConeValue(aU, aV, aPos, Radius, SAngle), aP);
        Assert.Equal(global::Kerf.Engine.Geometry.ElSLib.ConeValue(aU, aV, aPos, Radius, SAngle), global::Kerf.Engine.Geometry.ElSLib.Value(aU, aV, new Cone3D(aPos, SAngle, Radius)));

        global::Kerf.Engine.Geometry.ElSLib.SphereD0(aU, aV, aPos, Radius, out aP);
        Assert.Equal(global::Kerf.Engine.Geometry.ElSLib.SphereValue(aU, aV, aPos, Radius), aP);
        global::Kerf.Engine.Geometry.ElSLib.D0(aU, aV, new Sphere3D(aPos, Radius), out aP);
        Assert.Equal(global::Kerf.Engine.Geometry.ElSLib.SphereValue(aU, aV, aPos, Radius), aP);
        Assert.Equal(global::Kerf.Engine.Geometry.ElSLib.SphereValue(aU, aV, aPos, Radius), global::Kerf.Engine.Geometry.ElSLib.Value(aU, aV, new Sphere3D(aPos, Radius)));

        global::Kerf.Engine.Geometry.ElSLib.TorusD0(aU, aV, aPos, MajorRadius, MinorRadius, out aP);
        Assert.Equal(global::Kerf.Engine.Geometry.ElSLib.TorusValue(aU, aV, aPos, MajorRadius, MinorRadius), aP);
        global::Kerf.Engine.Geometry.ElSLib.D0(aU, aV, new Torus3D(aPos, MajorRadius, MinorRadius), out aP);
        Assert.Equal(global::Kerf.Engine.Geometry.ElSLib.TorusValue(aU, aV, aPos, MajorRadius, MinorRadius), aP);
        Assert.Equal(global::Kerf.Engine.Geometry.ElSLib.TorusValue(aU, aV, aPos, MajorRadius, MinorRadius),
                     global::Kerf.Engine.Geometry.ElSLib.Value(aU, aV, new Torus3D(aPos, MajorRadius, MinorRadius)));

        // and the gp overloads of D1 and the Parameters
        global::Kerf.Engine.Geometry.ElSLib.D1(aU, aV, new Torus3D(aPos, MajorRadius, MinorRadius), out aP, out Vector3D aVu, out Vector3D aVv);
        global::Kerf.Engine.Geometry.ElSLib.TorusD1(aU, aV, aPos, MajorRadius, MinorRadius, out Point3D aP2, out Vector3D aVu2, out Vector3D aVv2);
        Assert.Equal(aP2, aP);
        Assert.Equal(aVu2, aVu);
        Assert.Equal(aVv2, aVv);
        global::Kerf.Engine.Geometry.ElSLib.Parameters(new Cone3D(aPos, SAngle, Radius), aP, out double aPU, out double aPV);
        global::Kerf.Engine.Geometry.ElSLib.ConeParameters(aPos, Radius, SAngle, aP, out double aPU2, out double aPV2);
        Assert.Equal(aPU2, aPU);
        Assert.Equal(aPV2, aPV);
    }

    [Theory]
    [MemberData(nameof(Systems))]
    public void D1_matches_finite_differences(int theSystem)
    {
        CoordinateSystem aPos = System(theSystem);
        const double aU = 0.9, aV = 0.35, h = 1e-6;

        void Check(SurfaceValue theValue, in Point3D theP, in Vector3D theVu, in Vector3D theVv)
        {
            AssertNear(theValue(aU, aV).XYZ(), theP.XYZ());
            global::Kerf.Engine.Geometry.Xyz aDu = (theValue(aU + h, aV).XYZ() - theValue(aU - h, aV).XYZ()) / (2 * h);
            global::Kerf.Engine.Geometry.Xyz aDv = (theValue(aU, aV + h).XYZ() - theValue(aU, aV - h).XYZ()) / (2 * h);
            AssertNear(aDu, theVu.XYZ(), 1e-7);
            AssertNear(aDv, theVv.XYZ(), 1e-7);
        }

        var aSurfaces = Surfaces(aPos).ToDictionary(s => s.Name, s => s.Value);
        global::Kerf.Engine.Geometry.ElSLib.PlaneD1(aU, aV, aPos, out Point3D aP, out Vector3D aVu, out Vector3D aVv);
        Check(aSurfaces["plane"], aP, aVu, aVv);
        global::Kerf.Engine.Geometry.ElSLib.CylinderD1(aU, aV, aPos, Radius, out aP, out aVu, out aVv);
        Check(aSurfaces["cylinder"], aP, aVu, aVv);
        global::Kerf.Engine.Geometry.ElSLib.ConeD1(aU, aV, aPos, Radius, SAngle, out aP, out aVu, out aVv);
        Check(aSurfaces["cone"], aP, aVu, aVv);
        global::Kerf.Engine.Geometry.ElSLib.SphereD1(aU, aV, aPos, Radius, out aP, out aVu, out aVv);
        Check(aSurfaces["sphere"], aP, aVu, aVv);
        global::Kerf.Engine.Geometry.ElSLib.TorusD1(aU, aV, aPos, MajorRadius, MinorRadius, out aP, out aVu, out aVv);
        Check(aSurfaces["torus"], aP, aVu, aVv);
    }

    // The derivative of order (Nu, Nv) in closed form: the n-th derivative of cos(x) is cos(x + n*PI/2), and of
    // sin(x) is sin(x + n*PI/2).
    private static global::Kerf.Engine.Geometry.Xyz ReferenceDN(string theSurface, double U, double V, in CoordinateSystem Pos, int Nu, int Nv)
    {
        global::Kerf.Engine.Geometry.Xyz X = Pos.XDirection.XYZ(), Y = Pos.YDirection.XYZ(), Z = Pos.Direction.XYZ();
        double CosUn = Math.Cos(U + Nu * M_PI_2), SinUn = Math.Sin(U + Nu * M_PI_2);
        double CosVn = Math.Cos(V + Nv * M_PI_2), SinVn = Math.Sin(V + Nv * M_PI_2);
        global::Kerf.Engine.Geometry.Xyz Radial = X * CosUn + Y * SinUn;
        global::Kerf.Engine.Geometry.Xyz Zero = new(0, 0, 0);
        switch (theSurface)
        {
            case "plane":
                return Nu == 1 && Nv == 0 ? X : Nu == 0 && Nv == 1 ? Y : Zero;
            case "cylinder":
                return Nv == 0 ? Radial * Radius : Nu == 0 && Nv == 1 ? Z : Zero;
            case "cone":
                // (Radius + V sinA) Radial(U) + V cosA Z
                return Nv == 0 ? Radial * (Radius + V * Math.Sin(SAngle))
                       : Nv == 1 ? Radial * Math.Sin(SAngle) + (Nu == 0 ? Z * Math.Cos(SAngle) : Zero)
                       : Zero;
            case "sphere":
                return Radial * (Radius * CosVn) + (Nu == 0 ? Z * (Radius * SinVn) : Zero);
            default: // torus
                return Radial * ((Nv == 0 ? MajorRadius : 0.0) + MinorRadius * CosVn)
                       + (Nu == 0 ? Z * (MinorRadius * SinVn) : Zero);
        }
    }

    private static Vector3D PortDN(string theSurface, double U, double V, in CoordinateSystem Pos, int Nu, int Nv) => theSurface switch
    {
        "plane" => global::Kerf.Engine.Geometry.ElSLib.PlaneDN(U, V, Pos, Nu, Nv),
        "cylinder" => global::Kerf.Engine.Geometry.ElSLib.CylinderDN(U, V, Pos, Radius, Nu, Nv),
        "cone" => global::Kerf.Engine.Geometry.ElSLib.ConeDN(U, V, Pos, Radius, SAngle, Nu, Nv),
        "sphere" => global::Kerf.Engine.Geometry.ElSLib.SphereDN(U, V, Pos, Radius, Nu, Nv),
        _ => global::Kerf.Engine.Geometry.ElSLib.TorusDN(U, V, Pos, MajorRadius, MinorRadius, Nu, Nv),
    };

    [Theory]
    [MemberData(nameof(Systems))]
    public void DN_matches_closed_form(int theSystem)
    {
        CoordinateSystem aPos = System(theSystem);
        const double aU = 1.3, aV = 0.6;
        foreach (string aSurface in new[] { "plane", "cylinder", "cone", "sphere", "torus" })
        {
            for (int aNu = 0; aNu <= 5; aNu++)
            {
                for (int aNv = 0; aNv <= 5; aNv++)
                {
                    if (aNu + aNv == 0)
                    {
                        continue;
                    }

                    global::Kerf.Engine.Geometry.Xyz anExpected = ReferenceDN(aSurface, aU, aV, aPos, aNu, aNv);
                    AssertNear(anExpected, PortDN(aSurface, aU, aV, aPos, aNu, aNv).XYZ());
                }
            }
        }
    }

    [Fact]
    public void TorusDN_mixed_orders_change_sign_with_the_order()
    {
        // d4P/du dv3 = -d2P/du dv and d6P/du2 dv4 = -d4P/du2 dv2 for a torus (OCCT returned them unreversed;
        // occt-issues.md, ElSLib::TorusDN; Kerf decision 1486)
        CoordinateSystem aPos = new();
        Vector3D aDuv = global::Kerf.Engine.Geometry.ElSLib.TorusDN(0.5, 0.4, aPos, MajorRadius, MinorRadius, 1, 1);
        Vector3D aDuvvv = global::Kerf.Engine.Geometry.ElSLib.TorusDN(0.5, 0.4, aPos, MajorRadius, MinorRadius, 1, 3);
        Assert.Equal(aDuv.Reversed(), aDuvvv);
        Vector3D aDuuvv = global::Kerf.Engine.Geometry.ElSLib.TorusDN(0.5, 0.4, aPos, MajorRadius, MinorRadius, 2, 2);
        Vector3D aDuuvvvv = global::Kerf.Engine.Geometry.ElSLib.TorusDN(0.5, 0.4, aPos, MajorRadius, MinorRadius, 2, 4);
        Assert.Equal(aDuuvv.Reversed(), aDuuvvvv);
    }

    [Theory]
    [MemberData(nameof(Systems))]
    public void D2_and_D3_match_DN(int theSystem)
    {
        CoordinateSystem aPos = System(theSystem);
        const double aU = 2.2, aV = -0.3;

        void Check(string theSurface, Vector3D[] theDerivatives)
        {
            // Vu, Vv, Vuu, Vvv, Vuv, Vuuu, Vvvv, Vuuv, Vuvv
            (int, int)[] anOrders = [(1, 0), (0, 1), (2, 0), (0, 2), (1, 1), (3, 0), (0, 3), (2, 1), (1, 2)];
            for (int i = 0; i < theDerivatives.Length; i++)
            {
                (int aNu, int aNv) = anOrders[i];
                AssertNear(ReferenceDN(theSurface, aU, aV, aPos, aNu, aNv), theDerivatives[i].XYZ());
            }
        }

        Point3D aP;
        Vector3D Vu, Vv, Vuu, Vvv, Vuv, Vuuu, Vvvv, Vuuv, Vuvv;

        global::Kerf.Engine.Geometry.ElSLib.CylinderD2(aU, aV, aPos, Radius, out aP, out Vu, out Vv, out Vuu, out Vvv, out Vuv);
        Check("cylinder", [Vu, Vv, Vuu, Vvv, Vuv]);
        global::Kerf.Engine.Geometry.ElSLib.ConeD2(aU, aV, aPos, Radius, SAngle, out aP, out Vu, out Vv, out Vuu, out Vvv, out Vuv);
        Check("cone", [Vu, Vv, Vuu, Vvv, Vuv]);
        global::Kerf.Engine.Geometry.ElSLib.SphereD2(aU, aV, aPos, Radius, out aP, out Vu, out Vv, out Vuu, out Vvv, out Vuv);
        Check("sphere", [Vu, Vv, Vuu, Vvv, Vuv]);
        global::Kerf.Engine.Geometry.ElSLib.TorusD2(aU, aV, aPos, MajorRadius, MinorRadius, out aP, out Vu, out Vv, out Vuu, out Vvv, out Vuv);
        Check("torus", [Vu, Vv, Vuu, Vvv, Vuv]);
        AssertNear(global::Kerf.Engine.Geometry.ElSLib.TorusValue(aU, aV, aPos, MajorRadius, MinorRadius).XYZ(), aP.XYZ());

        global::Kerf.Engine.Geometry.ElSLib.CylinderD3(aU, aV, aPos, Radius, out aP, out Vu, out Vv, out Vuu, out Vvv, out Vuv, out Vuuu, out Vvvv, out Vuuv, out Vuvv);
        Check("cylinder", [Vu, Vv, Vuu, Vvv, Vuv, Vuuu, Vvvv, Vuuv, Vuvv]);
        global::Kerf.Engine.Geometry.ElSLib.ConeD3(aU, aV, aPos, Radius, SAngle, out aP, out Vu, out Vv, out Vuu, out Vvv, out Vuv, out Vuuu, out Vvvv, out Vuuv, out Vuvv);
        Check("cone", [Vu, Vv, Vuu, Vvv, Vuv, Vuuu, Vvvv, Vuuv, Vuvv]);
        global::Kerf.Engine.Geometry.ElSLib.SphereD3(aU, aV, aPos, Radius, out aP, out Vu, out Vv, out Vuu, out Vvv, out Vuv, out Vuuu, out Vvvv, out Vuuv, out Vuvv);
        Check("sphere", [Vu, Vv, Vuu, Vvv, Vuv, Vuuu, Vvvv, Vuuv, Vuvv]);
        global::Kerf.Engine.Geometry.ElSLib.TorusD3(aU, aV, aPos, MajorRadius, MinorRadius, out aP, out Vu, out Vv, out Vuu, out Vvv, out Vuv, out Vuuu, out Vvvv, out Vuuv, out Vuvv);
        Check("torus", [Vu, Vv, Vuu, Vvv, Vuv, Vuuu, Vvvv, Vuuv, Vuvv]);
        AssertNear(global::Kerf.Engine.Geometry.ElSLib.TorusValue(aU, aV, aPos, MajorRadius, MinorRadius).XYZ(), aP.XYZ());
    }

    private static Point3D OnLine(in Line3D theL, double theT)
        => new(theL.Location.XYZ() + theL.Direction.XYZ() * theT);

    private static Point3D OnCircle(in Circle3D theC, double theT)
        => new(theC.Location.XYZ()
               + theC.Position.XDirection.XYZ() * (theC.Radius * Math.Cos(theT))
               + theC.Position.YDirection.XYZ() * (theC.Radius * Math.Sin(theT)));

    [Theory]
    [MemberData(nameof(Systems))]
    public void Iso_curves_follow_the_surface(int theSystem)
    {
        CoordinateSystem aPos = System(theSystem);
        const double aU = 0.8, aV = 0.45;
        foreach (double aT in new[] { -1.0, 0.0, 0.6, 2.5 })
        {
            AssertNear(global::Kerf.Engine.Geometry.ElSLib.PlaneValue(aU, aT, aPos).XYZ(), OnLine(global::Kerf.Engine.Geometry.ElSLib.PlaneUIso(aPos, aU), aT).XYZ());
            AssertNear(global::Kerf.Engine.Geometry.ElSLib.PlaneValue(aT, aV, aPos).XYZ(), OnLine(global::Kerf.Engine.Geometry.ElSLib.PlaneVIso(aPos, aV), aT).XYZ());

            AssertNear(global::Kerf.Engine.Geometry.ElSLib.CylinderValue(aU, aT, aPos, Radius).XYZ(), OnLine(global::Kerf.Engine.Geometry.ElSLib.CylinderUIso(aPos, Radius, aU), aT).XYZ());
            AssertNear(global::Kerf.Engine.Geometry.ElSLib.CylinderValue(aT, aV, aPos, Radius).XYZ(), OnCircle(global::Kerf.Engine.Geometry.ElSLib.CylinderVIso(aPos, Radius, aV), aT).XYZ());

            AssertNear(global::Kerf.Engine.Geometry.ElSLib.ConeValue(aU, aT, aPos, Radius, SAngle).XYZ(),
                       OnLine(global::Kerf.Engine.Geometry.ElSLib.ConeUIso(aPos, Radius, SAngle, aU), aT).XYZ());
            AssertNear(global::Kerf.Engine.Geometry.ElSLib.ConeValue(aT, aV, aPos, Radius, SAngle).XYZ(),
                       OnCircle(global::Kerf.Engine.Geometry.ElSLib.ConeVIso(aPos, Radius, SAngle, aV), aT).XYZ());

            AssertNear(global::Kerf.Engine.Geometry.ElSLib.SphereValue(aU, aT, aPos, Radius).XYZ(), OnCircle(global::Kerf.Engine.Geometry.ElSLib.SphereUIso(aPos, Radius, aU), aT).XYZ());
            AssertNear(global::Kerf.Engine.Geometry.ElSLib.SphereValue(aT, aV, aPos, Radius).XYZ(), OnCircle(global::Kerf.Engine.Geometry.ElSLib.SphereVIso(aPos, Radius, aV), aT).XYZ());

            AssertNear(global::Kerf.Engine.Geometry.ElSLib.TorusValue(aU, aT, aPos, MajorRadius, MinorRadius).XYZ(),
                       OnCircle(global::Kerf.Engine.Geometry.ElSLib.TorusUIso(aPos, MajorRadius, MinorRadius, aU), aT).XYZ());
            AssertNear(global::Kerf.Engine.Geometry.ElSLib.TorusValue(aT, aV, aPos, MajorRadius, MinorRadius).XYZ(),
                       OnCircle(global::Kerf.Engine.Geometry.ElSLib.TorusVIso(aPos, MajorRadius, MinorRadius, aV), aT).XYZ());
        }
    }

    [Fact]
    public void VIso_with_negative_radius_is_reversed()
    {
        // Past the cone's apex the parallel's radius would be negative: the circle is reversed instead, and
        // still follows the surface
        CoordinateSystem aPos = new();
        double aV = -Radius / Math.Sin(SAngle) - 2.0;
        Circle3D aC = global::Kerf.Engine.Geometry.ElSLib.ConeVIso(aPos, Radius, SAngle, aV);
        Assert.Equal(-(Radius + aV * Math.Sin(SAngle)), aC.Radius, Tol);
        AssertNear(global::Kerf.Engine.Geometry.ElSLib.ConeValue(1.0, aV, aPos, Radius, SAngle).XYZ(), OnCircle(aC, 1.0).XYZ());

        // A sphere's parallel just past the pole (#23170)
        Circle3D aS = global::Kerf.Engine.Geometry.ElSLib.SphereVIso(aPos, Radius, M_PI_2 + 0.1);
        Assert.Equal(Radius * Math.Sin(0.1), aS.Radius, Tol);
        Assert.Equal(-1.0, aS.Position.Direction.Z, Tol);
    }
}
