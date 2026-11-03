// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Xunit.Abstractions;

namespace Kerf.Kernel.Oracle.Tests;

/// <summary>
/// ProjLib_Plane against native OCCT through GeomAPI::To2d, which projects a curve on a plane with
/// ProjLib_ProjectedCurve: for a line or a conic that is ProjLib_Plane, and Geom2dAdaptor::MakeCurve builds a
/// Geom2d_Line or Geom2d_Conic on its result. The port's ProjLib.Project overloads are compared with it.
/// </summary>
public class ProjLibOracleTests(ITestOutputHelper output)
{
    private const int Cases = 300;

    private static CoordinateSystem SettledAx3(Oracle theOracle)
    {
        CoordinateSystem anAx = theOracle.Ax3();
        var aSettled = new CoordinateSystem(anAx.Location, Oracle.Settled(anAx.Direction), Oracle.Settled(anAx.XDirection));
        return anAx.IsDirect ? aSettled : aSettled.YReversed();
    }

    private static Frame3D SettledAx2(Oracle theOracle)
    {
        Frame3D anAx = theOracle.Ax2();
        return new Frame3D(anAx.Location, Oracle.Settled(anAx.Direction), Oracle.Settled(anAx.XDirection));
    }

    private static void SamePosition(Oracle theOracle, N.Geom2d_Curve theNative, in Frame2D thePort, string theWhat)
    {
        using var aConic = N.Geom2d_Conic.DownCast(theNative)!;
        using var aPos = aConic.Position();
        using var aLoc = aPos.Location();
        using var aX = aPos.XDirection();
        using var aY = aPos.YDirection();
        theOracle.Same(aLoc, thePort.Location, $"{theWhat} location");
        theOracle.Same(aX.X(), thePort.XDirection.X, $"{theWhat} X direction X");
        theOracle.Same(aX.Y(), thePort.XDirection.Y, $"{theWhat} X direction Y");
        theOracle.Same(aY.X(), thePort.YDirection.X, $"{theWhat} Y direction X");
        theOracle.Same(aY.Y(), thePort.YDirection.Y, $"{theWhat} Y direction Y");
    }

    [Fact]
    public void Plane_conics()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            CoordinateSystem aPlAx = SettledAx3(anOracle);
            var aPln = new Plane3D(aPlAx);
            using var aNPlAx = Oracle.ToNative(aPlAx);
            using var aNPln = new N.gp_Pln(aNPlAx);
            Frame3D aCAx = SettledAx2(anOracle);
            using var aNCAx = Oracle.ToNative(aCAx);
            double aMaj = anOracle.Real(0.5, 20), aMin = anOracle.Real(0.2, aMaj);

            // Line
            {
                var aLin = new Line3D(aCAx.Axis);
                using var aNAx1 = Oracle.ToNative(aCAx.Axis);
                using var aNC = new N.Geom_Line(aNAx1);
                using var aN2d = N.GeomAPI.To2d(aNC, aNPln)!;
                using var aNLine = N.Geom2d_Line.DownCast(aN2d)!;
                using var aNLoc = aNLine.Location();
                using var aNDir = aNLine.Direction();
                Line2D aPort = global::Kerf.Engine.Geometry.Projection.ProjLib.Project(aPln, aLin);
                anOracle.Same(aNLoc, aPort.Location, $"case {i} line location");
                anOracle.Same(aNDir.X(), aPort.Direction.X, $"case {i} line direction X");
                anOracle.Same(aNDir.Y(), aPort.Direction.Y, $"case {i} line direction Y");
            }

            // Circle
            {
                using var aNC = new N.Geom_Circle(aNCAx, aMaj);
                using var aN2d = N.GeomAPI.To2d(aNC, aNPln)!;
                using var aNCirc = N.Geom2d_Circle.DownCast(aN2d)!;
                Circle2D aPort = global::Kerf.Engine.Geometry.Projection.ProjLib.Project(aPln, new Circle3D(aCAx, aMaj));
                SamePosition(anOracle, aN2d, aPort.Position, $"case {i} circle");
                anOracle.Same(aNCirc.Radius(), aPort.Radius, $"case {i} circle radius");
            }

            // Ellipse
            {
                using var aNC = new N.Geom_Ellipse(aNCAx, aMaj, aMin);
                using var aN2d = N.GeomAPI.To2d(aNC, aNPln)!;
                using var aNElips = N.Geom2d_Ellipse.DownCast(aN2d)!;
                Ellipse2D aPort = global::Kerf.Engine.Geometry.Projection.ProjLib.Project(aPln, new Ellipse3D(aCAx, aMaj, aMin));
                SamePosition(anOracle, aN2d, aPort.Axis, $"case {i} ellipse");
                anOracle.Same(aNElips.MajorRadius(), aPort.MajorRadius, $"case {i} ellipse major radius");
                anOracle.Same(aNElips.MinorRadius(), aPort.MinorRadius, $"case {i} ellipse minor radius");
            }

            // Parabola and hyperbola: Geom2d_Parabola and Geom2d_Hyperbola aren't bound, so only the position
            {
                using var aNC = new N.Geom_Parabola(aNCAx, aMin);
                using var aN2d = N.GeomAPI.To2d(aNC, aNPln)!;
                Parabola2D aPort = global::Kerf.Engine.Geometry.Projection.ProjLib.Project(aPln, new Parabola3D(aCAx, aMin));
                SamePosition(anOracle, aN2d, aPort.Axis, $"case {i} parabola");
            }
            {
                using var aNC = new N.Geom_Hyperbola(aNCAx, aMaj, aMin);
                using var aN2d = N.GeomAPI.To2d(aNC, aNPln)!;
                Hyperbola2D aPort = global::Kerf.Engine.Geometry.Projection.ProjLib.Project(aPln, new Hyperbola3D(aCAx, aMaj, aMin));
                SamePosition(anOracle, aN2d, aPort.Axis, $"case {i} hyperbola");
            }
        }
        anOracle.Report("ProjLib_Plane through GeomAPI::To2d");
    }
}
