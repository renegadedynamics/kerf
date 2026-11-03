// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using Xunit.Abstractions;

namespace Kerf.Kernel.Oracle.Tests;

/// <summary>Bnd_OBB against native OCCT, on random inputs.</summary>
/// <remarks>
/// The native binding exposes only <c>ReBuild(points)</c>, so the tolerance and optimal (BVH) modes of ReBuild are
/// covered by the unit tests alone. Rebuilding from the vertices of two boxes (<c>Add</c>) differs from native OCCT in the
/// last bits for a few cases, though rebuilding from the same points is bit-identical: the vertices themselves differ.
/// </remarks>
public class BndOBBOracleTests(ITestOutputHelper output)
{
    private const int Cases = 500;

    [Fact]
    public void OBB_from_point_sets()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            NCollection_Array1<Point3D> aPoints = RandomPoints(anOracle);
            using var aNPoints = ToNative(aPoints);
            using var aNative = new N.Bnd_OBB();
            aNative.ReBuild(aNPoints);
            OrientedBox anOBB = OrientedBox.FromPoints(aPoints);
            SameOBB(anOracle, aNative, anOBB, "ReBuild");

            // Every point inside
            for (int k = aPoints.Lower(); k <= aPoints.Upper(); k++)
            {
                using var aNP = Oracle.ToNative(aPoints[k]);
                anOracle.Same(aNative.IsOut(aNP), anOBB.IsOut(aPoints[k]), "IsOut(own point)");
            }

            // Random queries
            for (int k = 0; k < 4; k++)
            {
                Point3D aP = anOracle.Pnt();
                using var aNP = Oracle.ToNative(aP);
                anOracle.Same(aNative.IsOut(aNP), anOBB.IsOut(aP), "IsOut(point)");
                anOracle.Same(aNative.Contains(aNP), anOBB.Contains(aP), "Contains");
            }

            anOracle.Same(aNative.SquareExtent(), anOBB.SquareExtent(), "SquareExtent");
        }

        anOracle.Report("Bnd_OBB ReBuild from points");
    }

    [Fact]
    public void OBB_from_box_enlarge_and_add()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            Point3D aP1 = anOracle.Pnt(), aP2 = anOracle.Pnt();
            using var aNP1 = Oracle.ToNative(aP1);
            using var aNP2 = Oracle.ToNative(aP2);
            using var aNBox = new N.Bnd_Box();
            aNBox.Add(aNP1);
            aNBox.Add(aNP2);
            Box aBox = Box.FromPoint(aP1).Added(aP2);

            using var aNative = new N.Bnd_OBB(aNBox);
            OrientedBox anOBB = new(aBox);
            SameOBB(anOracle, aNative, anOBB, "from Bnd_Box");

            double aGap = anOracle.Real(-3, 3);
            aNative.Enlarge(aGap);
            anOBB = anOBB.Enlarged(aGap);
            SameOBB(anOracle, aNative, anOBB, "Enlarge");

            // Add a point (inside or outside)
            Point3D aP = new(anOracle.Real(-150, 150), anOracle.Real(-150, 150), anOracle.Real(-150, 150));
            using var aNP = Oracle.ToNative(aP);
            aNative.Add(aNP);
            anOBB = anOBB.Added(aP);
            SameOBB(anOracle, aNative, anOBB, "Add(point)");

            // Add another (oriented) box
            Frame3D anAx = anOracle.Ax2();
            double aHX = anOracle.Real(0, 30), aHY = anOracle.Real(0, 30), aHZ = anOracle.Real(0, 30);
            using var aNOther = ToNative(anAx, aHX, aHY, aHZ);
            OrientedBox anOther = FromNative(aNOther);
            aNative.Add(aNOther);
            anOBB = anOBB.Added(anOther);
            SameOBB(anOracle, aNative, anOBB, "Add(OBB)");

            // Adding to a void box copies the other
            using var aNVoid = new N.Bnd_OBB();
            aNVoid.Add(aNOther);
            SameOBB(anOracle, aNVoid, new OrientedBox().Added(anOther), "Add(OBB) to void");
        }

        anOracle.Report("Bnd_OBB from Bnd_Box, Enlarge, Add");
    }

    [Fact]
    public void OBB_interference()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            Frame3D anAx1 = anOracle.Ax2(), anAx2 = anOracle.Ax2();
            double[] aH = [anOracle.Real(0, 60), anOracle.Real(0, 60), anOracle.Real(0, 60),
                           anOracle.Real(0, 60), anOracle.Real(0, 60), anOracle.Real(0, 60)];
            using var aN1 = ToNative(anAx1, aH[0], aH[1], aH[2]);
            using var aN2 = ToNative(anAx2, aH[3], aH[4], aH[5]);
            OrientedBox anOBB1 = FromNative(aN1);
            OrientedBox anOBB2 = FromNative(aN2);

            anOracle.Same(aN1.IsOut(aN2), anOBB1.IsOut(anOBB2), "IsOut(OBB)");
            anOracle.Same(aN1.Intersects(aN2), anOBB1.Intersects(anOBB2), "Intersects");
            anOracle.Same(aN1.IsCompletelyInside(aN2), anOBB1.IsCompletelyInside(anOBB2), "IsCompletelyInside");

            // A box inside the other (shrunk copy) and an axis-aligned pair
            using var aNSmall = ToNative(anAx1, aH[0] / 2, aH[1] / 2, aH[2] / 2);
            OrientedBox aSmall = FromNative(aNSmall);
            anOracle.Same(aN1.IsCompletelyInside(aNSmall), anOBB1.IsCompletelyInside(aSmall), "IsCompletelyInside(shrunk)");

            aN1.SetAABox(true);
            aN2.SetAABox(true);
            anOracle.Same(aN1.IsOut(aN2), anOBB1.WithAABox(true).IsOut(anOBB2.WithAABox(true)), "IsOut(AABox)");
        }

        anOracle.Report("Bnd_OBB interference");
    }

    // Helpers

    private static NCollection_Array1<Point3D> RandomPoints(Oracle theOracle)
    {
        int aKind = theOracle.Int(0, 4);
        int aNb = aKind == 4 ? theOracle.Int(1, 3) : theOracle.Int(3, 60);
        NCollection_Array1<Point3D> aPoints = new(1, aNb);
        Frame3D anAx = theOracle.Ax2();
        for (int k = 1; k <= aNb; k++)
        {
            double aU = theOracle.Real(-20, 20), aV = theOracle.Real(-5, 5), aW = theOracle.Real(-1, 1);
            aPoints[k] = aKind switch
            {
                0 => theOracle.Pnt(),                                                      // a cloud
                1 => anAx.Location.Translated(aU * new Vector3D(anAx.XDirection)      // flat
                                                 + aV * new Vector3D(anAx.YDirection)),
                2 => anAx.Location.Translated(aU * new Vector3D(anAx.XDirection)),     // collinear
                _ => anAx.Location.Translated(aU * new Vector3D(anAx.XDirection)       // a thin slab
                                                 + aV * new Vector3D(anAx.YDirection)
                                                 + aW * new Vector3D(anAx.Direction)),
            };
        }

        return aPoints;
    }

    private static N.TColgp_Array1OfPnt ToNative(NCollection_Array1<Point3D> thePoints)
    {
        var aNative = new N.TColgp_Array1OfPnt(thePoints.Lower(), thePoints.Upper());
        for (int k = thePoints.Lower(); k <= thePoints.Upper(); k++)
        {
            using var aNP = Oracle.ToNative(thePoints[k]);
            aNative.SetValue(k, aNP);
        }

        return aNative;
    }

    private static N.Bnd_OBB ToNative(in Frame3D theAx, double theHX, double theHY, double theHZ)
    {
        using var aC = Oracle.ToNative(theAx.Location);
        using var aX = Oracle.ToNative(theAx.XDirection);
        using var aY = Oracle.ToNative(theAx.YDirection);
        using var aZ = Oracle.ToNative(theAx.Direction);
        return new N.Bnd_OBB(aC, aX, aY, aZ, theHX, theHY, theHZ);
    }

    /// <summary>
    /// The port's box with exactly the native box's values: the native gp_Dir constructor normalises the components again
    /// (the port keeps a vector within gp::Resolution of unit length), which can change the last bits.
    /// </summary>
    private static OrientedBox FromNative(N.Bnd_OBB theNative)
    {
        using var aC = theNative.Center();
        using var aX = theNative.XDirection();
        using var aY = theNative.YDirection();
        using var aZ = theNative.ZDirection();
        return new OrientedBox().WithCenter(new Point3D(Oracle.FromNative(aC)))
                            .WithXComponent(new Direction3D(Oracle.FromNative(aX)), theNative.XHSize())
                            .WithYComponent(new Direction3D(Oracle.FromNative(aY)), theNative.YHSize())
                            .WithZComponent(new Direction3D(Oracle.FromNative(aZ)), theNative.ZHSize())
                            .WithAABox(theNative.IsAABox());
    }

    private static void SameOBB(Oracle theOracle, N.Bnd_OBB theNative, in OrientedBox thePort, string theWhat)
    {
        theOracle.Same(theNative.IsVoid(), thePort.IsVoid, $"{theWhat}: IsVoid");
        theOracle.Same(theNative.IsAABox(), thePort.IsAABox, $"{theWhat}: IsAABox");
        using (var aC = theNative.Center())
        using (var aX = theNative.XDirection())
        using (var aY = theNative.YDirection())
        using (var aZ = theNative.ZDirection())
        {
            theOracle.Same(Oracle.FromNative(aC), thePort.Center(), $"{theWhat}: Center");
            theOracle.Same(Oracle.FromNative(aX), thePort.XDirection(), $"{theWhat}: XDirection");
            theOracle.Same(Oracle.FromNative(aY), thePort.YDirection(), $"{theWhat}: YDirection");
            theOracle.Same(Oracle.FromNative(aZ), thePort.ZDirection(), $"{theWhat}: ZDirection");
        }

        theOracle.Same(theNative.XHSize(), thePort.XHSize(), $"{theWhat}: XHSize");
        theOracle.Same(theNative.YHSize(), thePort.YHSize(), $"{theWhat}: YHSize");
        theOracle.Same(theNative.ZHSize(), thePort.ZHSize(), $"{theWhat}: ZHSize");
    }
}
