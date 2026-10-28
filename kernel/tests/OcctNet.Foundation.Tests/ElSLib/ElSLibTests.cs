// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

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

    private static gp_Ax3 System(int theIndex)
    {
        gp_Ax3 anAx3 = theIndex switch
        {
            0 => new gp_Ax3(),
            1 => new gp_Ax3(new gp_Pnt(1.0, -2.0, 3.5), new gp_Dir(0.3, 0.5, 0.8), new gp_Dir(1.0, 0.2, -0.5)),
            2 => new gp_Ax3(new gp_Pnt(-4.0, 0.5, 2.0), new gp_Dir(-0.6, 0.1, 0.4), new gp_Dir(0.2, 1.0, 0.1)),
            _ => new gp_Ax3(new gp_Pnt(7.0, 7.0, -1.0), new gp_Dir(0.0, 0.0, 1.0), new gp_Dir(1.0, 0.0, 0.0)),
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

    private static void AssertNear(in gp_XYZ theExpected, in gp_XYZ theActual, double theTol = Tol)
    {
        Assert.Equal(theExpected.X(), theActual.X(), theTol);
        Assert.Equal(theExpected.Y(), theActual.Y(), theTol);
        Assert.Equal(theExpected.Z(), theActual.Z(), theTol);
    }

    // Each surface as a value function, so the tests can loop over them

    private delegate gp_Pnt SurfaceValue(double theU, double theV);

    private static IEnumerable<(string Name, SurfaceValue Value)> Surfaces(gp_Ax3 thePos)
    {
        yield return ("plane", (u, v) => ElSLib.PlaneValue(u, v, thePos));
        yield return ("cylinder", (u, v) => ElSLib.CylinderValue(u, v, thePos, Radius));
        yield return ("cone", (u, v) => ElSLib.ConeValue(u, v, thePos, Radius, SAngle));
        yield return ("sphere", (u, v) => ElSLib.SphereValue(u, v, thePos, Radius));
        yield return ("torus", (u, v) => ElSLib.TorusValue(u, v, thePos, MajorRadius, MinorRadius));
    }

    [Theory]
    [MemberData(nameof(Systems))]
    public void Value_then_Parameters_round_trip(int theSystem)
    {
        gp_Ax3 aPos = System(theSystem);
        double[] aUs = [0.0, 0.3, 1.7, 3.0, 4.4, 6.1];
        double[] aVs = [-1.2, -0.4, 0.0, 0.5, 1.3];
        foreach (double aU in aUs)
        {
            foreach (double aV in aVs)
            {
                ElSLib.PlaneParameters(aPos, ElSLib.PlaneValue(aU, aV, aPos), out double aPU, out double aPV);
                Assert.Equal(aU, aPU, Tol);
                Assert.Equal(aV, aPV, Tol);

                ElSLib.CylinderParameters(aPos, Radius, ElSLib.CylinderValue(aU, aV, aPos, Radius), out aPU, out aPV);
                AssertSameAngle(aU, aPU);
                Assert.Equal(aV, aPV, Tol);

                ElSLib.ConeParameters(aPos, Radius, SAngle, ElSLib.ConeValue(aU, aV, aPos, Radius, SAngle), out aPU, out aPV);
                AssertSameAngle(aU, aPU);
                Assert.Equal(aV, aPV, Tol);

                ElSLib.SphereParameters(aPos, Radius, ElSLib.SphereValue(aU, aV, aPos, Radius), out aPU, out aPV);
                AssertSameAngle(aU, aPU);
                Assert.Equal(aV, aPV, Tol);

                // The torus' V is returned in [0, 2*PI]
                ElSLib.TorusParameters(aPos, MajorRadius, MinorRadius,
                                       ElSLib.TorusValue(aU, aV, aPos, MajorRadius, MinorRadius), out aPU, out aPV);
                AssertSameAngle(aU, aPU);
                AssertSameAngle(aV, aPV);
            }
        }
    }

    [Fact]
    public void Parameters_normalise_and_handle_poles()
    {
        gp_Ax3 aPos = new();

        // A sphere's poles have U = 0
        ElSLib.SphereParameters(aPos, Radius, new gp_Pnt(0, 0, Radius), out double aU, out double aV);
        Assert.Equal(0.0, aU);
        Assert.Equal(M_PI_2, aV);
        ElSLib.SphereParameters(aPos, Radius, new gp_Pnt(0, 0, -Radius), out aU, out aV);
        Assert.Equal(0.0, aU);
        Assert.Equal(-M_PI_2, aV);

        // The cone's apex has U = 0, and a point beyond the apex (on the other nappe) keeps its U
        double anApexV = -Radius / Math.Sin(SAngle);
        ElSLib.ConeParameters(aPos, Radius, SAngle, ElSLib.ConeValue(1.0, anApexV, aPos, Radius, SAngle), out aU, out aV);
        Assert.Equal(0.0, aU);
        Assert.Equal(anApexV, aV, Tol);
        gp_Pnt aBeyond = ElSLib.ConeValue(1.0, anApexV - 2.0, aPos, Radius, SAngle);
        ElSLib.ConeParameters(aPos, Radius, SAngle, aBeyond, out aU, out aV);
        Assert.Equal(1.0, aU, Tol);
        Assert.Equal(anApexV - 2.0, aV, Tol);

        // A tiny negative angle becomes 0 rather than 2*PI; 2*PI itself is kept
        ElSLib.CylinderParameters(aPos, Radius, new gp_Pnt(Radius, -1e-17, 0.0), out aU, out _);
        Assert.Equal(0.0, aU);
        ElSLib.CylinderParameters(aPos, Radius, new gp_Pnt(Radius, -1e-3, 0.0), out aU, out _);
        Assert.Equal(2 * Math.PI - 1e-3 / Radius, aU, 1e-9);
    }

    [Fact]
    public void TorusParameters_with_major_radius_below_minor_radius()
    {
        // A spindle torus: the point is nearer the tube centred on the opposite side of the axis
        gp_Ax3 aPos = new();
        const double aMajor = 1.0, aMinor = 3.0;
        foreach (double aU in new[] { 0.2, 2.0, 4.0 })
        {
            foreach (double aV in new[] { 0.1, 1.0, 2.0 })
            {
                gp_Pnt aP = ElSLib.TorusValue(aU, aV, aPos, aMajor, aMinor);
                ElSLib.TorusParameters(aPos, aMajor, aMinor, aP, out double aPU, out double aPV);
                AssertNear(aP.XYZ(), ElSLib.TorusValue(aPU, aPV, aPos, aMajor, aMinor).XYZ());
            }
        }
    }

    [Theory]
    [MemberData(nameof(Systems))]
    public void D0_and_gp_overloads_match_Value(int theSystem)
    {
        gp_Ax3 aPos = System(theSystem);
        const double aU = 1.1, aV = 0.7;

        ElSLib.PlaneD0(aU, aV, aPos, out gp_Pnt aP);
        Assert.Equal(ElSLib.PlaneValue(aU, aV, aPos), aP);
        ElSLib.D0(aU, aV, new gp_Pln(aPos), out aP);
        Assert.Equal(ElSLib.PlaneValue(aU, aV, aPos), aP);
        Assert.Equal(ElSLib.PlaneValue(aU, aV, aPos), ElSLib.Value(aU, aV, new gp_Pln(aPos)));

        ElSLib.CylinderD0(aU, aV, aPos, Radius, out aP);
        Assert.Equal(ElSLib.CylinderValue(aU, aV, aPos, Radius), aP);
        ElSLib.D0(aU, aV, new gp_Cylinder(aPos, Radius), out aP);
        Assert.Equal(ElSLib.CylinderValue(aU, aV, aPos, Radius), aP);
        Assert.Equal(ElSLib.CylinderValue(aU, aV, aPos, Radius), ElSLib.Value(aU, aV, new gp_Cylinder(aPos, Radius)));

        ElSLib.ConeD0(aU, aV, aPos, Radius, SAngle, out aP);
        Assert.Equal(ElSLib.ConeValue(aU, aV, aPos, Radius, SAngle), aP);
        ElSLib.D0(aU, aV, new gp_Cone(aPos, SAngle, Radius), out aP);
        Assert.Equal(ElSLib.ConeValue(aU, aV, aPos, Radius, SAngle), aP);
        Assert.Equal(ElSLib.ConeValue(aU, aV, aPos, Radius, SAngle), ElSLib.Value(aU, aV, new gp_Cone(aPos, SAngle, Radius)));

        ElSLib.SphereD0(aU, aV, aPos, Radius, out aP);
        Assert.Equal(ElSLib.SphereValue(aU, aV, aPos, Radius), aP);
        ElSLib.D0(aU, aV, new gp_Sphere(aPos, Radius), out aP);
        Assert.Equal(ElSLib.SphereValue(aU, aV, aPos, Radius), aP);
        Assert.Equal(ElSLib.SphereValue(aU, aV, aPos, Radius), ElSLib.Value(aU, aV, new gp_Sphere(aPos, Radius)));

        ElSLib.TorusD0(aU, aV, aPos, MajorRadius, MinorRadius, out aP);
        Assert.Equal(ElSLib.TorusValue(aU, aV, aPos, MajorRadius, MinorRadius), aP);
        ElSLib.D0(aU, aV, new gp_Torus(aPos, MajorRadius, MinorRadius), out aP);
        Assert.Equal(ElSLib.TorusValue(aU, aV, aPos, MajorRadius, MinorRadius), aP);
        Assert.Equal(ElSLib.TorusValue(aU, aV, aPos, MajorRadius, MinorRadius),
                     ElSLib.Value(aU, aV, new gp_Torus(aPos, MajorRadius, MinorRadius)));

        // and the gp overloads of D1 and the Parameters
        ElSLib.D1(aU, aV, new gp_Torus(aPos, MajorRadius, MinorRadius), out aP, out gp_Vec aVu, out gp_Vec aVv);
        ElSLib.TorusD1(aU, aV, aPos, MajorRadius, MinorRadius, out gp_Pnt aP2, out gp_Vec aVu2, out gp_Vec aVv2);
        Assert.Equal(aP2, aP);
        Assert.Equal(aVu2, aVu);
        Assert.Equal(aVv2, aVv);
        ElSLib.Parameters(new gp_Cone(aPos, SAngle, Radius), aP, out double aPU, out double aPV);
        ElSLib.ConeParameters(aPos, Radius, SAngle, aP, out double aPU2, out double aPV2);
        Assert.Equal(aPU2, aPU);
        Assert.Equal(aPV2, aPV);
    }

    [Theory]
    [MemberData(nameof(Systems))]
    public void D1_matches_finite_differences(int theSystem)
    {
        gp_Ax3 aPos = System(theSystem);
        const double aU = 0.9, aV = 0.35, h = 1e-6;

        void Check(SurfaceValue theValue, in gp_Pnt theP, in gp_Vec theVu, in gp_Vec theVv)
        {
            AssertNear(theValue(aU, aV).XYZ(), theP.XYZ());
            gp_XYZ aDu = (theValue(aU + h, aV).XYZ() - theValue(aU - h, aV).XYZ()) / (2 * h);
            gp_XYZ aDv = (theValue(aU, aV + h).XYZ() - theValue(aU, aV - h).XYZ()) / (2 * h);
            AssertNear(aDu, theVu.XYZ(), 1e-7);
            AssertNear(aDv, theVv.XYZ(), 1e-7);
        }

        var aSurfaces = Surfaces(aPos).ToDictionary(s => s.Name, s => s.Value);
        ElSLib.PlaneD1(aU, aV, aPos, out gp_Pnt aP, out gp_Vec aVu, out gp_Vec aVv);
        Check(aSurfaces["plane"], aP, aVu, aVv);
        ElSLib.CylinderD1(aU, aV, aPos, Radius, out aP, out aVu, out aVv);
        Check(aSurfaces["cylinder"], aP, aVu, aVv);
        ElSLib.ConeD1(aU, aV, aPos, Radius, SAngle, out aP, out aVu, out aVv);
        Check(aSurfaces["cone"], aP, aVu, aVv);
        ElSLib.SphereD1(aU, aV, aPos, Radius, out aP, out aVu, out aVv);
        Check(aSurfaces["sphere"], aP, aVu, aVv);
        ElSLib.TorusD1(aU, aV, aPos, MajorRadius, MinorRadius, out aP, out aVu, out aVv);
        Check(aSurfaces["torus"], aP, aVu, aVv);
    }

    // The derivative of order (Nu, Nv) in closed form: the n-th derivative of cos(x) is cos(x + n*PI/2), and of
    // sin(x) is sin(x + n*PI/2).
    private static gp_XYZ ReferenceDN(string theSurface, double U, double V, in gp_Ax3 Pos, int Nu, int Nv)
    {
        gp_XYZ X = Pos.XDirection().XYZ(), Y = Pos.YDirection().XYZ(), Z = Pos.Direction().XYZ();
        double CosUn = Math.Cos(U + Nu * M_PI_2), SinUn = Math.Sin(U + Nu * M_PI_2);
        double CosVn = Math.Cos(V + Nv * M_PI_2), SinVn = Math.Sin(V + Nv * M_PI_2);
        gp_XYZ Radial = X * CosUn + Y * SinUn;
        gp_XYZ Zero = new(0, 0, 0);
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

    private static gp_Vec PortDN(string theSurface, double U, double V, in gp_Ax3 Pos, int Nu, int Nv) => theSurface switch
    {
        "plane" => ElSLib.PlaneDN(U, V, Pos, Nu, Nv),
        "cylinder" => ElSLib.CylinderDN(U, V, Pos, Radius, Nu, Nv),
        "cone" => ElSLib.ConeDN(U, V, Pos, Radius, SAngle, Nu, Nv),
        "sphere" => ElSLib.SphereDN(U, V, Pos, Radius, Nu, Nv),
        _ => ElSLib.TorusDN(U, V, Pos, MajorRadius, MinorRadius, Nu, Nv),
    };

    /// <summary>
    /// TorusDN's odd/odd and even/even mixed branches never change sign (an OCCT bug, see ElSLib.cs), so those
    /// derivatives come out reversed when (Nu + Nv) % 4 is 0 (odd/odd) or 2 (even/even).
    /// </summary>
    private static bool TorusDNReversed(int Nu, int Nv)
        => Nu > 0 && Nv > 0
           && ((IsOdd(Nu) && IsOdd(Nv) && (Nu + Nv) % 4 == 0) || (IsEven(Nu) && IsEven(Nv) && (Nu + Nv) % 4 == 2));

    [Theory]
    [MemberData(nameof(Systems))]
    public void DN_matches_closed_form(int theSystem)
    {
        gp_Ax3 aPos = System(theSystem);
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

                    gp_XYZ anExpected = ReferenceDN(aSurface, aU, aV, aPos, aNu, aNv);
                    if (aSurface == "torus" && TorusDNReversed(aNu, aNv))
                    {
                        anExpected = anExpected.Reversed(); // OCCT behaviour, reproduced
                    }

                    AssertNear(anExpected, PortDN(aSurface, aU, aV, aPos, aNu, aNv).XYZ());
                }
            }
        }
    }

    [Fact]
    public void TorusDN_reproduces_OCCT_sign_error()
    {
        // d4P/du dv3 = -d2P/du dv for a torus, but OCCT returns +d2P/du dv
        gp_Ax3 aPos = new();
        gp_Vec aDuv = ElSLib.TorusDN(0.5, 0.4, aPos, MajorRadius, MinorRadius, 1, 1);
        gp_Vec aDuvvv = ElSLib.TorusDN(0.5, 0.4, aPos, MajorRadius, MinorRadius, 1, 3);
        Assert.Equal(aDuv, aDuvvv);
    }

    [Theory]
    [MemberData(nameof(Systems))]
    public void D2_and_D3_match_DN(int theSystem)
    {
        gp_Ax3 aPos = System(theSystem);
        const double aU = 2.2, aV = -0.3;

        void Check(string theSurface, gp_Vec[] theDerivatives)
        {
            // Vu, Vv, Vuu, Vvv, Vuv, Vuuu, Vvvv, Vuuv, Vuvv
            (int, int)[] anOrders = [(1, 0), (0, 1), (2, 0), (0, 2), (1, 1), (3, 0), (0, 3), (2, 1), (1, 2)];
            for (int i = 0; i < theDerivatives.Length; i++)
            {
                (int aNu, int aNv) = anOrders[i];
                AssertNear(ReferenceDN(theSurface, aU, aV, aPos, aNu, aNv), theDerivatives[i].XYZ());
            }
        }

        gp_Pnt aP;
        gp_Vec Vu, Vv, Vuu, Vvv, Vuv, Vuuu, Vvvv, Vuuv, Vuvv;

        ElSLib.CylinderD2(aU, aV, aPos, Radius, out aP, out Vu, out Vv, out Vuu, out Vvv, out Vuv);
        Check("cylinder", [Vu, Vv, Vuu, Vvv, Vuv]);
        ElSLib.ConeD2(aU, aV, aPos, Radius, SAngle, out aP, out Vu, out Vv, out Vuu, out Vvv, out Vuv);
        Check("cone", [Vu, Vv, Vuu, Vvv, Vuv]);
        ElSLib.SphereD2(aU, aV, aPos, Radius, out aP, out Vu, out Vv, out Vuu, out Vvv, out Vuv);
        Check("sphere", [Vu, Vv, Vuu, Vvv, Vuv]);
        ElSLib.TorusD2(aU, aV, aPos, MajorRadius, MinorRadius, out aP, out Vu, out Vv, out Vuu, out Vvv, out Vuv);
        Check("torus", [Vu, Vv, Vuu, Vvv, Vuv]);
        AssertNear(ElSLib.TorusValue(aU, aV, aPos, MajorRadius, MinorRadius).XYZ(), aP.XYZ());

        ElSLib.CylinderD3(aU, aV, aPos, Radius, out aP, out Vu, out Vv, out Vuu, out Vvv, out Vuv, out Vuuu, out Vvvv, out Vuuv, out Vuvv);
        Check("cylinder", [Vu, Vv, Vuu, Vvv, Vuv, Vuuu, Vvvv, Vuuv, Vuvv]);
        ElSLib.ConeD3(aU, aV, aPos, Radius, SAngle, out aP, out Vu, out Vv, out Vuu, out Vvv, out Vuv, out Vuuu, out Vvvv, out Vuuv, out Vuvv);
        Check("cone", [Vu, Vv, Vuu, Vvv, Vuv, Vuuu, Vvvv, Vuuv, Vuvv]);
        ElSLib.SphereD3(aU, aV, aPos, Radius, out aP, out Vu, out Vv, out Vuu, out Vvv, out Vuv, out Vuuu, out Vvvv, out Vuuv, out Vuvv);
        Check("sphere", [Vu, Vv, Vuu, Vvv, Vuv, Vuuu, Vvvv, Vuuv, Vuvv]);
        ElSLib.TorusD3(aU, aV, aPos, MajorRadius, MinorRadius, out aP, out Vu, out Vv, out Vuu, out Vvv, out Vuv, out Vuuu, out Vvvv, out Vuuv, out Vuvv);
        Check("torus", [Vu, Vv, Vuu, Vvv, Vuv, Vuuu, Vvvv, Vuuv, Vuvv]);
        AssertNear(ElSLib.TorusValue(aU, aV, aPos, MajorRadius, MinorRadius).XYZ(), aP.XYZ());
    }

    private static gp_Pnt OnLine(in gp_Lin theL, double theT)
        => new(theL.Location().XYZ() + theL.Direction().XYZ() * theT);

    private static gp_Pnt OnCircle(in gp_Circ theC, double theT)
        => new(theC.Location().XYZ()
               + theC.Position().XDirection().XYZ() * (theC.Radius() * Math.Cos(theT))
               + theC.Position().YDirection().XYZ() * (theC.Radius() * Math.Sin(theT)));

    [Theory]
    [MemberData(nameof(Systems))]
    public void Iso_curves_follow_the_surface(int theSystem)
    {
        gp_Ax3 aPos = System(theSystem);
        const double aU = 0.8, aV = 0.45;
        foreach (double aT in new[] { -1.0, 0.0, 0.6, 2.5 })
        {
            AssertNear(ElSLib.PlaneValue(aU, aT, aPos).XYZ(), OnLine(ElSLib.PlaneUIso(aPos, aU), aT).XYZ());
            AssertNear(ElSLib.PlaneValue(aT, aV, aPos).XYZ(), OnLine(ElSLib.PlaneVIso(aPos, aV), aT).XYZ());

            AssertNear(ElSLib.CylinderValue(aU, aT, aPos, Radius).XYZ(), OnLine(ElSLib.CylinderUIso(aPos, Radius, aU), aT).XYZ());
            AssertNear(ElSLib.CylinderValue(aT, aV, aPos, Radius).XYZ(), OnCircle(ElSLib.CylinderVIso(aPos, Radius, aV), aT).XYZ());

            AssertNear(ElSLib.ConeValue(aU, aT, aPos, Radius, SAngle).XYZ(),
                       OnLine(ElSLib.ConeUIso(aPos, Radius, SAngle, aU), aT).XYZ());
            AssertNear(ElSLib.ConeValue(aT, aV, aPos, Radius, SAngle).XYZ(),
                       OnCircle(ElSLib.ConeVIso(aPos, Radius, SAngle, aV), aT).XYZ());

            AssertNear(ElSLib.SphereValue(aU, aT, aPos, Radius).XYZ(), OnCircle(ElSLib.SphereUIso(aPos, Radius, aU), aT).XYZ());
            AssertNear(ElSLib.SphereValue(aT, aV, aPos, Radius).XYZ(), OnCircle(ElSLib.SphereVIso(aPos, Radius, aV), aT).XYZ());

            AssertNear(ElSLib.TorusValue(aU, aT, aPos, MajorRadius, MinorRadius).XYZ(),
                       OnCircle(ElSLib.TorusUIso(aPos, MajorRadius, MinorRadius, aU), aT).XYZ());
            AssertNear(ElSLib.TorusValue(aT, aV, aPos, MajorRadius, MinorRadius).XYZ(),
                       OnCircle(ElSLib.TorusVIso(aPos, MajorRadius, MinorRadius, aV), aT).XYZ());
        }
    }

    [Fact]
    public void VIso_with_negative_radius_is_reversed()
    {
        // Past the cone's apex the parallel's radius would be negative: the circle is reversed instead, and
        // still follows the surface
        gp_Ax3 aPos = new();
        double aV = -Radius / Math.Sin(SAngle) - 2.0;
        gp_Circ aC = ElSLib.ConeVIso(aPos, Radius, SAngle, aV);
        Assert.Equal(-(Radius + aV * Math.Sin(SAngle)), aC.Radius(), Tol);
        AssertNear(ElSLib.ConeValue(1.0, aV, aPos, Radius, SAngle).XYZ(), OnCircle(aC, 1.0).XYZ());

        // A sphere's parallel just past the pole (#23170)
        gp_Circ aS = ElSLib.SphereVIso(aPos, Radius, M_PI_2 + 0.1);
        Assert.Equal(Radius * Math.Sin(0.1), aS.Radius(), Tol);
        Assert.Equal(-1.0, aS.Position().Direction().Z(), Tol);
    }
}
