// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Curves;
using Kerf.Geometry;
using Xunit.Abstractions;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// GC_MakeArcOfCircle, GC_MakeArcOfEllipse, GC_MakeArcOfHyperbola and GC_MakeArcOfParabola against native OCCT, on
/// random inputs built from the same raw numbers on both sides: the status, the trimming parameters, the basis conic
/// and points of the arc.
/// </summary>
public class GCOracleTests(ITestOutputHelper output)
{
    private const int Cases = 500;

    private static void SameArc(Oracle theOracle, N.Geom_TrimmedCurve theNative, TrimmedCurve3D thePort,
                                string theWhat, double theTolerance = Oracle.Tolerance)
    {
        double aNF = theNative.FirstParameter(), aNL = theNative.LastParameter();
        theOracle.Same(aNF, thePort.FirstParameter(), theWhat + " FirstParameter", theTolerance);
        theOracle.Same(aNL, thePort.LastParameter(), theWhat + " LastParameter", theTolerance);
        // Points at the native parameters, so that a parameter difference doesn't count twice
        foreach (double aT in new[] { 0.0, 0.25, 0.5, 1.0 })
        {
            double aU = aNF + aT * (aNL - aNF);
            using var aNP = theNative.Value(aU);
            theOracle.Same(aNP, thePort.Value(aU), $"{theWhat} point at {aT}", theTolerance);
        }
    }

    private static void SameStatus(Oracle theOracle, N.GC_Root theNative, Root thePort, string theWhat) =>
        theOracle.Same((int)theNative.Status(), (int)thePort.Status(), theWhat + " status");

    private static void SameCirc(Oracle theOracle, N.Geom_TrimmedCurve theNative, TrimmedCurve3D thePort,
                                 string theWhat, double theTolerance = Oracle.Tolerance)
    {
        using var aNBasis = theNative.BasisCurve();
        using var aNCircle = N.Geom_Circle.DownCast(aNBasis);
        using var aNCirc = aNCircle!.Circ();
        Circle3D aCirc = ((CircleCurve3D)thePort.BasisCurve()).Circ();
        theOracle.Same(aNCirc.Radius(), aCirc.Radius(), theWhat + " radius", theTolerance);
        using var aNPos = aNCirc.Position();
        using var aNLoc = aNPos.Location();
        using var aNZ = aNPos.Direction();
        using var aNX = aNPos.XDirection();
        using var aNZXYZ = aNZ.XYZ();
        using var aNXXYZ = aNX.XYZ();
        theOracle.Same(Oracle.FromNative(aNLoc).XYZ(), aCirc.Location().XYZ(), theWhat + " center", theTolerance);
        theOracle.Same(Oracle.FromNative(aNZXYZ), aCirc.Axis().Direction().XYZ(), theWhat + " normal", theTolerance);
        theOracle.Same(Oracle.FromNative(aNXXYZ), aCirc.XAxis().Direction().XYZ(), theWhat + " X direction",
                       theTolerance);
    }

    [Fact]
    public void ArcOfCircle_three_points()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            Point3D aP1 = anOracle.Pnt(), aP2 = anOracle.Pnt(), aP3 = anOracle.Pnt();
            using var aNP1 = Oracle.ToNative(aP1);
            using var aNP2 = Oracle.ToNative(aP2);
            using var aNP3 = Oracle.ToNative(aP3);

            using var aNative = new N.GC_MakeArcOfCircle(aNP1, aNP2, aNP3);
            var aPort = new MakeArcOfCircle(aP1, aP2, aP3);
            SameStatus(anOracle, aNative, aPort, "three points");
            if (!aNative.IsDone() || !aPort.IsDone)
            {
                continue;
            }
            using var aNArc = aNative.Value();
            SameCirc(anOracle, aNArc!, aPort.Value(), "three points");
            SameArc(anOracle, aNArc!, aPort.Value(), "three points");
        }
        anOracle.Report("GC_MakeArcOfCircle(P1, P2, P3)");
    }

    [Fact]
    public void ArcOfCircle_points_and_tangent()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            Point3D aP1 = anOracle.Pnt(), aP2 = anOracle.Pnt();
            Vector3D aV = anOracle.Vec();
            using var aNP1 = Oracle.ToNative(aP1);
            using var aNP2 = Oracle.ToNative(aP2);
            using var aNV = Oracle.ToNative(aV);

            using var aNative = new N.GC_MakeArcOfCircle(aNP1, aNV, aNP2);
            var aPort = new MakeArcOfCircle(aP1, aV, aP2);
            SameStatus(anOracle, aNative, aPort, "tangent");
            if (!aNative.IsDone() || !aPort.IsDone)
            {
                continue;
            }
            using var aNArc = aNative.Value();
            SameCirc(anOracle, aNArc!, aPort.Value(), "tangent");
            SameArc(anOracle, aNArc!, aPort.Value(), "tangent");
        }
        anOracle.Report("GC_MakeArcOfCircle(P1, V, P2)");
    }

    [Fact]
    public void ArcOfCircle_on_circle()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            (N.gp_Ax2 aNA2, Frame3D aA2) = GeomOracleInputs.Ax2s(anOracle);
            using var _ = aNA2;
            double aR = anOracle.Real(0.1, 50);
            using var aNC = new N.gp_Circ(aNA2, aR);
            Circle3D aC = new(aA2, aR);
            Point3D aP1 = anOracle.Pnt(), aP2 = anOracle.Pnt();
            double aAlpha1 = anOracle.Angle(), aAlpha2 = anOracle.Angle();
            bool aSense = anOracle.Bool();
            using var aNP1 = Oracle.ToNative(aP1);
            using var aNP2 = Oracle.ToNative(aP2);

            using (var aN = new N.GC_MakeArcOfCircle(aNC, aNP1, aNP2, aSense))
            using (var aNArc = aN.Value())
            {
                SameArc(anOracle, aNArc!, new MakeArcOfCircle(aC, aP1, aP2, aSense).Value(), "two points");
            }
            using (var aN = new N.GC_MakeArcOfCircle(aNC, aNP1, aAlpha2, aSense))
            using (var aNArc = aN.Value())
            {
                SameArc(anOracle, aNArc!, new MakeArcOfCircle(aC, aP1, aAlpha2, aSense).Value(), "point, angle");
            }
            using (var aN = new N.GC_MakeArcOfCircle(aNC, aAlpha1, aAlpha2, aSense))
            using (var aNArc = aN.Value())
            {
                SameArc(anOracle, aNArc!, new MakeArcOfCircle(aC, aAlpha1, aAlpha2, aSense).Value(), "angles");
            }
        }
        anOracle.Report("GC_MakeArcOfCircle on a circle");
    }

    [Fact]
    public void ArcOfEllipse()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            (N.gp_Ax2 aNA2, Frame3D aA2) = GeomOracleInputs.Ax2s(anOracle);
            using var _ = aNA2;
            double aMinor = anOracle.Real(0.1, 20);
            double aMajor = aMinor + anOracle.Real(0, 20);
            using var aNE = new N.gp_Elips(aNA2, aMajor, aMinor);
            Ellipse3D aE = new(aA2, aMajor, aMinor);
            Point3D aP1 = anOracle.Pnt(), aP2 = anOracle.Pnt();
            double aAlpha1 = anOracle.Angle(), aAlpha2 = anOracle.Angle();
            bool aSense = anOracle.Bool();
            using var aNP1 = Oracle.ToNative(aP1);
            using var aNP2 = Oracle.ToNative(aP2);

            using (var aN = new N.GC_MakeArcOfEllipse(aNE, aNP1, aNP2, aSense))
            using (var aNArc = aN.Value())
            {
                SameArc(anOracle, aNArc!, new MakeArcOfEllipse(aE, aP1, aP2, aSense).Value(), "two points");
            }
            using (var aN = new N.GC_MakeArcOfEllipse(aNE, aNP1, aAlpha2, aSense))
            using (var aNArc = aN.Value())
            {
                SameArc(anOracle, aNArc!, new MakeArcOfEllipse(aE, aP1, aAlpha2, aSense).Value(), "point, angle");
            }
            using (var aN = new N.GC_MakeArcOfEllipse(aNE, aAlpha1, aAlpha2, aSense))
            using (var aNArc = aN.Value())
            {
                SameArc(anOracle, aNArc!, new MakeArcOfEllipse(aE, aAlpha1, aAlpha2, aSense).Value(), "angles");
            }
        }
        anOracle.Report("GC_MakeArcOfEllipse");
    }

    [Fact]
    public void ArcOfHyperbola()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            (N.gp_Ax2 aNA2, Frame3D aA2) = GeomOracleInputs.Ax2s(anOracle);
            using var _ = aNA2;
            double aMajor = anOracle.Real(0.1, 20), aMinor = anOracle.Real(0.1, 20);
            using var aNH = new N.gp_Hypr(aNA2, aMajor, aMinor);
            Hyperbola3D aH = new(aA2, aMajor, aMinor);
            Point3D aP1 = anOracle.Pnt(), aP2 = anOracle.Pnt();
            double aAlpha1 = anOracle.Real(-3, 3), aAlpha2 = anOracle.Real(-3, 3);
            bool aSense = anOracle.Bool();
            using var aNP1 = Oracle.ToNative(aP1);
            using var aNP2 = Oracle.ToNative(aP2);

            using (var aN = new N.GC_MakeArcOfHyperbola(aNH, aNP1, aNP2, aSense))
            using (var aNArc = aN.Value())
            {
                SameArc(anOracle, aNArc!, new MakeArcOfHyperbola(aH, aP1, aP2, aSense).Value(), "two points");
            }
            using (var aN = new N.GC_MakeArcOfHyperbola(aNH, aNP1, aAlpha2, aSense))
            using (var aNArc = aN.Value())
            {
                SameArc(anOracle, aNArc!, new MakeArcOfHyperbola(aH, aP1, aAlpha2, aSense).Value(), "point, angle");
            }
            using (var aN = new N.GC_MakeArcOfHyperbola(aNH, aAlpha1, aAlpha2, aSense))
            using (var aNArc = aN.Value())
            {
                SameArc(anOracle, aNArc!, new MakeArcOfHyperbola(aH, aAlpha1, aAlpha2, aSense).Value(), "angles");
            }
        }
        anOracle.Report("GC_MakeArcOfHyperbola");
    }

    [Fact]
    public void ArcOfParabola()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            (N.gp_Ax2 aNA2, Frame3D aA2) = GeomOracleInputs.Ax2s(anOracle);
            using var _ = aNA2;
            double aFocal = anOracle.Real(0.1, 20);
            using var aNP = new N.gp_Parab(aNA2, aFocal);
            Parabola3D aPb = new(aA2, aFocal);
            Point3D aP1 = anOracle.Pnt(), aP2 = anOracle.Pnt();
            double aAlpha1 = anOracle.Real(), aAlpha2 = anOracle.Real();
            bool aSense = anOracle.Bool();
            using var aNP1 = Oracle.ToNative(aP1);
            using var aNP2 = Oracle.ToNative(aP2);

            using (var aN = new N.GC_MakeArcOfParabola(aNP, aNP1, aNP2, aSense))
            using (var aNArc = aN.Value())
            {
                SameArc(anOracle, aNArc!, new MakeArcOfParabola(aPb, aP1, aP2, aSense).Value(), "two points");
            }
            using (var aN = new N.GC_MakeArcOfParabola(aNP, aNP1, aAlpha2, aSense))
            using (var aNArc = aN.Value())
            {
                SameArc(anOracle, aNArc!, new MakeArcOfParabola(aPb, aP1, aAlpha2, aSense).Value(), "point, angle");
            }
            using (var aN = new N.GC_MakeArcOfParabola(aNP, aAlpha1, aAlpha2, aSense))
            using (var aNArc = aN.Value())
            {
                SameArc(anOracle, aNArc!, new MakeArcOfParabola(aPb, aAlpha1, aAlpha2, aSense).Value(), "angles");
            }
        }
        anOracle.Report("GC_MakeArcOfParabola");
    }
}
