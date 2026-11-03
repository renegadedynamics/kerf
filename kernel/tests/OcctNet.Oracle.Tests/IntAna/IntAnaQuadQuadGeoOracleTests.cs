// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Xunit.Abstractions;
using static Occt.IntAna_ResultType;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// IntAna_QuadQuadGeo against native OCCT through GeomAPI_IntSS, which intersects two elementary surfaces with
/// IntPatch_ImpImpIntersection: plane/plane (IntPP), plane/cylinder (IntPCy), plane/sphere (IntPSp) and
/// sphere/sphere (IntSpSp) call IntAna_QuadQuadGeo with the angular tolerance 1e-8 and GeomAPI_IntSS's tolerance,
/// and GeomInt_IntSS builds a Geom_Line, Geom_Circle or Geom_Ellipse on the gp curve it returns (trimmed when the
/// surfaces bound it). Native OCCT moves a plane/sphere circle's X axis to the seam (AdjustToSeam), so only the
/// circles' centres, radii and normals are compared there.
/// </summary>
public class IntAnaQuadQuadGeoOracleTests(ITestOutputHelper output)
{
    private const int Cases = 300;
    private const double TolAng = 1.0e-8;
    private const double Tol = 1.0e-7;

    private static N.Geom_Curve Basis(N.Geom_Curve theC)
    {
        using var aTrimmed = N.Geom_TrimmedCurve.DownCast(theC);
        return aTrimmed is null ? theC : aTrimmed.BasisCurve()!;
    }

    private static CoordinateSystem Settled(Oracle theOracle)
    {
        CoordinateSystem anAx = theOracle.Ax3();
        var aSettled = new CoordinateSystem(anAx.Location(), Oracle.Settled(anAx.Direction()), Oracle.Settled(anAx.XDirection()));
        return anAx.Direct() ? aSettled : aSettled.YReversed();
    }

    private static void SameAxisDirection(Oracle theOracle, N.gp_Dir theNative, in Direction3D thePort, string theWhat)
    {
        // A circle's normal may be reversed by the native builder
        double aSign = theNative.X() * thePort.X() + theNative.Y() * thePort.Y() + theNative.Z() * thePort.Z() < 0 ? -1 : 1;
        theOracle.Same(aSign * theNative.X(), thePort.X(), $"{theWhat} normal X");
        theOracle.Same(aSign * theNative.Y(), thePort.Y(), $"{theWhat} normal Y");
        theOracle.Same(aSign * theNative.Z(), thePort.Z(), $"{theWhat} normal Z");
    }

    private static void Compare(Oracle theOracle, N.Geom_Surface theS1, N.Geom_Surface theS2, IntAna_QuadQuadGeo thePort,
                                string theWhat)
    {
        using var anIntSS = new N.GeomAPI_IntSS(theS1, theS2, Tol);
        theOracle.Same(anIntSS.IsDone(), thePort.IsDone(), $"{theWhat} IsDone");
        if (!anIntSS.IsDone() || !thePort.IsDone())
        {
            return;
        }

        IntAna_ResultType aType = thePort.TypeInter();
        bool aCurve = aType == IntAna_Line || aType == IntAna_Circle || aType == IntAna_Ellipse;
        theOracle.Same(aCurve, anIntSS.NbLines() > 0, $"{theWhat} has curves");
        if (!aCurve || anIntSS.NbLines() == 0)
        {
            return;
        }

        using var aLine = anIntSS.Line(1)!;
        using var aBasis = Basis(aLine);
        switch (aType)
        {
            case IntAna_Line:
            {
                using var aNLine = N.Geom_Line.DownCast(aBasis)!;
                using var aLin = aNLine.Lin();
                using var aLoc = aLin.Location();
                using var aDir = aLin.Direction();
                Line3D aPort = thePort.Line(1);
                theOracle.Same(aLoc, aPort.Location(), $"{theWhat} line location");
                theOracle.Same(aDir.X(), aPort.Direction().X(), $"{theWhat} line direction X");
                theOracle.Same(aDir.Y(), aPort.Direction().Y(), $"{theWhat} line direction Y");
                theOracle.Same(aDir.Z(), aPort.Direction().Z(), $"{theWhat} line direction Z");
                break;
            }
            case IntAna_Circle:
            {
                using var aNCircle = N.Geom_Circle.DownCast(aBasis)!;
                using var aCirc = aNCircle.Circ();
                using var aLoc = aCirc.Location();
                using var anAxis = aCirc.Axis();
                using var aNormal = anAxis.Direction();
                Circle3D aPort = thePort.Circle(1);
                theOracle.Same(aLoc, aPort.Location(), $"{theWhat} circle centre");
                theOracle.Same(aCirc.Radius(), aPort.Radius(), $"{theWhat} circle radius");
                SameAxisDirection(theOracle, aNormal, aPort.Axis().Direction(), $"{theWhat} circle");
                break;
            }
            case IntAna_Ellipse:
            {
                using var aNEllipse = N.Geom_Ellipse.DownCast(aBasis)!;
                using var aLoc = aNEllipse.Location();
                Ellipse3D aPort = thePort.Ellipse(1);
                theOracle.Same(aLoc, aPort.Location(), $"{theWhat} ellipse centre");
                theOracle.Same(aNEllipse.MajorRadius(), aPort.MajorRadius(), $"{theWhat} ellipse major radius");
                theOracle.Same(aNEllipse.MinorRadius(), aPort.MinorRadius(), $"{theWhat} ellipse minor radius");
                break;
            }
        }
    }

    [Fact]
    public void Plane_plane()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            CoordinateSystem anAx1 = Settled(anOracle), anAx2 = Settled(anOracle);
            using var aNAx1 = Oracle.ToNative(anAx1);
            using var aNAx2 = Oracle.ToNative(anAx2);
            using var aS1 = new N.Geom_Plane(aNAx1);
            using var aS2 = new N.Geom_Plane(aNAx2);
            var aPort = new IntAna_QuadQuadGeo(new Plane3D(anAx1), new Plane3D(anAx2), TolAng, Tol);
            Compare(anOracle, aS1, aS2, aPort, $"case {i}");
        }
        anOracle.Report("IntAna_QuadQuadGeo plane/plane");
    }

    [Fact]
    public void Plane_cylinder()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            CoordinateSystem anAx1 = Settled(anOracle), anAx2 = Settled(anOracle);
            double aR = anOracle.Real(0.5, 20);
            using var aNAx1 = Oracle.ToNative(anAx1);
            using var aNAx2 = Oracle.ToNative(anAx2);
            using var aS1 = new N.Geom_Plane(aNAx1);
            using var aS2 = new N.Geom_CylindricalSurface(aNAx2, aR);
            var aPort = new IntAna_QuadQuadGeo(new Plane3D(anAx1), new Cylinder(anAx2, aR), TolAng, Tol);
            Compare(anOracle, aS1, aS2, aPort, $"case {i}");
        }
        anOracle.Report("IntAna_QuadQuadGeo plane/cylinder");
    }

    [Fact]
    public void Plane_sphere()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            CoordinateSystem anAx1 = Settled(anOracle), anAx2 = Settled(anOracle);
            double aR = anOracle.Real(10, 150);
            using var aNAx1 = Oracle.ToNative(anAx1);
            using var aNAx2 = Oracle.ToNative(anAx2);
            using var aS1 = new N.Geom_Plane(aNAx1);
            using var aS2 = new N.Geom_SphericalSurface(aNAx2, aR);
            var aPort = new IntAna_QuadQuadGeo(new Plane3D(anAx1), new Sphere(anAx2, aR));
            Compare(anOracle, aS1, aS2, aPort, $"case {i}");
        }
        anOracle.Report("IntAna_QuadQuadGeo plane/sphere");
    }

    [Fact]
    public void Sphere_sphere()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            CoordinateSystem anAx1 = Settled(anOracle), anAx2 = Settled(anOracle);
            double aR1 = anOracle.Real(10, 150), aR2 = anOracle.Real(10, 150);
            using var aNAx1 = Oracle.ToNative(anAx1);
            using var aNAx2 = Oracle.ToNative(anAx2);
            using var aS1 = new N.Geom_SphericalSurface(aNAx1, aR1);
            using var aS2 = new N.Geom_SphericalSurface(aNAx2, aR2);
            var aPort = new IntAna_QuadQuadGeo(new Sphere(anAx1, aR1), new Sphere(anAx2, aR2), Tol);
            Compare(anOracle, aS1, aS2, aPort, $"case {i}");
        }
        anOracle.Report("IntAna_QuadQuadGeo sphere/sphere");
    }
}
