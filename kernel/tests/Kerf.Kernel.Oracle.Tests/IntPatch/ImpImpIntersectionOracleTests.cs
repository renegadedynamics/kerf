// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Patches;
using Kerf.Geometry;
using Xunit.Abstractions;
using static Kerf.Engine.Intersection.Patches.PatchLineType;

namespace Kerf.Kernel.Oracle.Tests;

/// <summary>
/// An early, partial oracle of IntPatch_ImpImpIntersection (unit 5B) through native GeomAPI_IntSS, before the port
/// has GeomInt (unit 5G). For two quadrics GeomInt_IntSS runs IntPatch_ImpImpIntersection on GeomAdaptor_Surfaces with
/// natural bounds and Adaptor3d_TopolTools (TolArc = TolTang = 1e-7), and makes each conic line (IntPatch_GLine) a
/// Geom_Line, Geom_Circle, ... built from the GLine's conic, trimmed by GeomInt_LineConstructor: the basis curve of
/// a native line is the GLine's conic itself. So every native line whose basis is a conic must be, bit for bit, the
/// conic of one of the port's GLines; native lines approximated from analytic or walking lines (B-splines) are left
/// to 5G, as are the trimming, the vertices and the count (the line constructor splits and drops lines).
/// </summary>
/// <remarks>
/// The pairs are random planes, cylinders, cones, spheres and tori (a torus reaches ImpImp only with a quadric that
/// shares its axis, or a sphere on it; IntPatch_Intersection sends the others to ImpPrm), and the special cases ImpImp treats apart: coaxial and
/// parallel axes, planes through the apex of a cone or parallel to its generatrices, tangent planes and cylinders,
/// spheres on an axis. The surfaces' positions are read back from the native surfaces; a pair whose native position
/// doesn't round-trip bit for bit into the port's gp_Ax3 is skipped (counted).
/// </remarks>
public class ImpImpIntersectionOracleTests(ITestOutputHelper output)
{
    private enum Kind { Plane, Cylinder, Cone, Sphere, Torus }

    // A surface position: random or, for the special cases, sharing the axis of the first surface.
    private static CoordinateSystem RandomAx3(Oracle theO, double theRange)
    {
        Direction3D aN = Oracle.Settled(theO.Dir());
        Direction3D aX = Oracle.Settled(new Direction3D(new Vector3D(aN).Crossed(new Vector3D(theO.Dir()))));
        return new CoordinateSystem(new Point3D(theO.Real(-theRange, theRange), theO.Real(-theRange, theRange),
                                     theO.Real(-theRange, theRange)), aN, aX);
    }

    private static N.Geom_Surface Native(Kind theKind, in CoordinateSystem theAx, double theR, double theAng)
    {
        using var aP = Oracle.ToNative(theAx.Location);
        using var aN = Oracle.ToNative(theAx.Direction);
        using var aX = Oracle.ToNative(theAx.XDirection);
        using var anAx = new N.gp_Ax3(aP, aN, aX);
        return theKind switch
        {
            Kind.Plane => new N.Geom_Plane(anAx),
            Kind.Cylinder => new N.Geom_CylindricalSurface(anAx, theR),
            Kind.Cone => new N.Geom_ConicalSurface(anAx, theAng, theR),
            Kind.Sphere => new N.Geom_SphericalSurface(anAx, theR),
            _ => new N.Geom_ToroidalSurface(anAx, theR + 1.0, 0.5 * theR),
        };
    }

    // The port's surface on the native surface's own position (null if it doesn't round-trip).
    private static Surface? Port(Kind theKind, in CoordinateSystem theAx, N.Geom_Surface theNative, double theR, double theAng)
    {
        using var anElem = N.Geom_ElementarySurface.DownCast(theNative)!;
        using var aPos = anElem.Position();
        using var aLoc = aPos.Location();
        using var aZ = aPos.Direction();
        using var aX = aPos.XDirection();
        using var aY = aPos.YDirection();
        // The same construction as the native one, from the same values.
        CoordinateSystem anAx = new(theAx.Location, theAx.Direction, theAx.XDirection);
        double[] aNative = [aLoc.X(), aLoc.Y(), aLoc.Z(), aZ.X(), aZ.Y(), aZ.Z(), aX.X(), aX.Y(), aX.Z(), aY.X(), aY.Y(), aY.Z()];
        double[] aPort = [.. Components(anAx.Location), .. Components(anAx.Direction), .. Components(anAx.XDirection),
                          .. Components(anAx.YDirection)];
        if (!aNative.SequenceEqual(aPort))
        {
            return null;
        }
        return theKind switch
        {
            Kind.Plane => new PlaneSurface(anAx),
            Kind.Cylinder => new CylindricalSurface(anAx, theR),
            Kind.Cone => new ConicalSurface(anAx, theAng, theR),
            Kind.Sphere => new SphericalSurface(anAx, theR),
            _ => new ToroidalSurface(anAx, theR + 1.0, 0.5 * theR),
        };
    }

    private static double[] Components(in Direction3D theD) => [theD.X, theD.Y, theD.Z];

    private static double[] Components(in Point3D theP) => [theP.X, theP.Y, theP.Z];

    private static double[] Components(in Frame3D theA) =>
        [.. Components(theA.Location), .. Components(theA.Direction), .. Components(theA.XDirection)];

    private static double[] NativeAx2(N.gp_Ax2 theA)
    {
        using var aL = theA.Location();
        using var aZ = theA.Direction();
        using var aX = theA.XDirection();
        return [aL.X(), aL.Y(), aL.Z(), aZ.X(), aZ.Y(), aZ.Z(), aX.X(), aX.Y(), aX.Z()];
    }

    // The type and values of a native line's basis conic, or null for a B-spline (an approximated line).
    private static (string Type, double[] Values)? NativeConic(N.Geom_Curve theLine)
    {
        using var aTrimmed = N.Geom_TrimmedCurve.DownCast(theLine);
        using N.Geom_Curve aBasis = aTrimmed is null ? N.Geom_Curve.DownCast(theLine)! : aTrimmed.BasisCurve()!;
        using (var aLine = N.Geom_Line.DownCast(aBasis))
        {
            if (aLine is not null)
            {
                using var aLin = aLine.Lin();
                using var aL = aLin.Location();
                using var aD = aLin.Direction();
                return ("Line", [aL.X(), aL.Y(), aL.Z(), aD.X(), aD.Y(), aD.Z()]);
            }
        }
        using var aConic = N.Geom_Conic.DownCast(aBasis);
        if (aConic is null)
        {
            return null;
        }
        using var aPos = aConic.Position();
        double[] anAx2 = NativeAx2(aPos);
        using (var aC = N.Geom_Circle.DownCast(aBasis))
        {
            if (aC is not null)
            {
                return ("Circle", [.. anAx2, aC.Radius()]);
            }
        }
        using (var anE = N.Geom_Ellipse.DownCast(aBasis))
        {
            if (anE is not null)
            {
                return ("Ellipse", [.. anAx2, anE.MajorRadius(), anE.MinorRadius()]);
            }
        }
        using (var aP = N.Geom_Parabola.DownCast(aBasis))
        {
            if (aP is not null)
            {
                return ("Parabola", [.. anAx2, aP.Focal()]);
            }
        }
        using var aH = N.Geom_Hyperbola.DownCast(aBasis)!;
        return ("Hyperbola", [.. anAx2, aH.MajorRadius(), aH.MinorRadius()]);
    }

    private static (string Type, double[] Values)? PortConic(PatchLine theLine)
    {
        if (theLine is not GLine aG)
        {
            return null;
        }
        return aG.ArcType() switch
        {
            PatchLineType.Lin => ("Line", [.. Components(aG.Line().Location), .. Components(aG.Line().Direction)]),
            PatchLineType.Circle => ("Circle", [.. Components(aG.Circle().Position), aG.Circle().Radius]),
            PatchLineType.Ellipse => ("Ellipse", [.. Components(aG.Ellipse().Position), aG.Ellipse().MajorRadius,
                                             aG.Ellipse().MinorRadius]),
            PatchLineType.Parabola => ("Parabola", [.. Components(aG.Parabola().Position), aG.Parabola().Focal]),
            _ => ("Hyperbola", [.. Components(aG.Hyperbola().Position), aG.Hyperbola().MajorRadius,
                                aG.Hyperbola().MinorRadius]),
        };
    }

    private static double MaxRelativeDifference(double[] theA, double[] theB)
    {
        double aMax = 0.0;
        for (int i = 0; i < theA.Length; i++)
        {
            aMax = Math.Max(aMax, Math.Abs(theA[i] - theB[i]) / Math.Max(1.0, Math.Abs(theA[i])));
        }
        return aMax;
    }

    [Fact]
    public void Native_conic_lines_are_the_ports_GLines_bit_for_bit()
    {
        var anO = new Oracle(output, 50502);
        int aPairs = 0, aSkipped = 0, aConics = 0, anIdentical = 0, aBSplines = 0;
        double aMaxDiff = 0.0;
        List<string> aDifferent = [];
        for (int i = 0; i < 625; i++)
        {
            var aK1 = (Kind)(i % 5);
            var aK2 = (Kind)((i / 5) % 5);
            int aSpecial = (i / 25) % 5;
            CoordinateSystem anAx1 = RandomAx3(anO, 1.0);
            CoordinateSystem anAx2 = RandomAx3(anO, 1.0);
            double aR1 = Math.Round(anO.Real(0.5, 3), 3), aR2 = Math.Round(anO.Real(0.5, 3), 3);
            double anA1 = anO.Real(0.1, 1.2), anA2 = anO.Real(0.1, 1.2);
            switch (aSpecial)
            {
                case 1: // coaxial
                    anAx2 = new CoordinateSystem(anAx1.Location.Translated(new Vector3D(anAx1.Direction) * anO.Real(-2, 2)),
                                       anAx1.Direction, anAx1.XDirection);
                    break;
                case 2: // parallel axes
                    anAx2 = new CoordinateSystem(anAx1.Location.Translated(new Vector3D(anAx1.XDirection) * anO.Real(-2, 2)),
                                       anAx1.Direction, anAx1.XDirection);
                    break;
                case 3: // the same location (the apex of a cone with radius 0, the centre of a sphere)
                    anAx2 = new CoordinateSystem(anAx1.Location, anAx2.Direction, anAx2.XDirection);
                    aR1 = aK1 == Kind.Cone ? 0.0 : aR1;
                    aR2 = aK2 == Kind.Cone ? 0.0 : aR2;
                    anAx2 = aK1 == Kind.Torus || aK2 == Kind.Torus ? new CoordinateSystem(anAx1.Location, anAx1.Direction, anAx1.XDirection) : anAx2;
                    break;
                case 4: // tangent: a cylinder's or sphere's radius reaches the other's location or axis
                    anAx2 = new CoordinateSystem(anAx1.Location.Translated(new Vector3D(anAx1.XDirection) * (aR1 + aR2)),
                                       anAx1.Direction, anAx1.XDirection);
                    break;
            }

            using N.Geom_Surface aN1 = Native(aK1, anAx1, aR1, anA1);
            using N.Geom_Surface aN2 = Native(aK2, anAx2, aR2, anA2);
            Surface? aP1 = Port(aK1, anAx1, aN1, aR1, anA1), aP2 = Port(aK2, anAx2, aN2, aR2, anA2);
            if (aP1 is null || aP2 is null)
            {
                aSkipped++;
                continue;
            }

            List<(string Type, double[] Values)> aNativeConics = [];
            using (var anInter = new N.GeomAPI_IntSS(aN1, aN2, 1.0e-7))
            {
                if (!anInter.IsDone())
                {
                    continue;
                }
                for (int j = 1; j <= anInter.NbLines(); j++)
                {
                    using N.Geom_Curve aLine = anInter.Line(j)!;
                    var aConic = NativeConic(aLine);
                    if (aConic is null)
                    {
                        aBSplines++;
                    }
                    else
                    {
                        aNativeConics.Add(aConic.Value);
                    }
                }
            }

            SurfaceAdaptor aS1 = new(aP1), aS2 = new(aP2);
            var aPort = new ImpImpIntersection(aS1, new TopolTool(aS1), aS2,
                                                        new TopolTool(aS2), 1.0e-7, 1.0e-7);
            Assert.True(aPort.IsDone || aNativeConics.Count == 0, $"pair {i}: the port failed");
            aPairs++;
            List<(string Type, double[] Values)> aPortConics = [];
            if (aPort.IsDone && !aPort.IsEmpty)
            {
                for (int j = 1; j <= aPort.LineCount; j++)
                {
                    var aConic = PortConic(aPort.Line(j));
                    if (aConic is not null)
                    {
                        aPortConics.Add(aConic.Value);
                    }
                }
            }

            foreach (var aNC in aNativeConics)
            {
                aConics++;
                if (aPortConics.Any(p => p.Type == aNC.Type && p.Values.SequenceEqual(aNC.Values)))
                {
                    anIdentical++;
                    continue;
                }
                var aSameType = aPortConics.Where(p => p.Type == aNC.Type).ToList();
                Assert.True(aSameType.Count > 0, $"pair {i} ({aK1}/{aK2}, case {aSpecial}): no {aNC.Type} in the port");
                double aDiff = aSameType.Min(p => MaxRelativeDifference(aNC.Values, p.Values));
                aMaxDiff = Math.Max(aMaxDiff, aDiff);
                aDifferent.Add($"pair {i} ({aK1}/{aK2}, case {aSpecial}): {aNC.Type} differs by {aDiff:E2}");
            }
        }

        output.WriteLine($"{aPairs} pairs ({aSkipped} skipped), {aConics} native conic lines, {anIdentical} bit-identical "
                         + $"to a GLine of the port, {aBSplines} approximated lines (5G); largest difference {aMaxDiff:E2}");
        foreach (string aLine in aDifferent)
        {
            output.WriteLine(aLine);
        }
        Assert.True(aConics > 100, $"only {aConics} conic lines");
        Assert.True(aMaxDiff <= 1.0e-9, $"largest difference {aMaxDiff:E2}");
        Assert.True(anIdentical >= aConics * 0.95, $"{anIdentical} of {aConics} bit-identical");
    }
}
