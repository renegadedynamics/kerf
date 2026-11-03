// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Xunit.Abstractions;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// BRepFill.Axe (unit 5O2; BRepOffsetAPI_MakeEvolved's referential when theIsAxeProf is false) against native OCCT's:
/// random planar polygon spines (as a wire and as a face, in random planes) and random profiles (polylines near the
/// spine, some starting on one of its vertices, which takes OCCT's vertex/vertex branch). Both sides read the same BRep
/// file (the port's shapes frozen, as a caller's); the computed axis (location, direction, X direction) and the
/// ProfOnSpine flag must be bit-identical. BRepOffsetAPI_MakeEvolved itself isn't bound natively.
/// </summary>
public sealed class BRepFillAxeOracleTests(ITestOutputHelper output) : IDisposable
{
    private readonly string myDirectory = Directory.CreateTempSubdirectory("occtnet-axe-").FullName;
    private int myFileNumber;

    public void Dispose()
    {
        try
        {
            Directory.Delete(myDirectory, true);
        }
        catch (IOException)
        {
        }
    }

    // Writes the port's shape and reads it on both sides (the port's frozen)
    private (N.TopoDS_Shape N, TopoDS_Shape P) Both(in TopoDS_Shape theShape)
    {
        string aFile = Path.Combine(myDirectory, $"s{myFileNumber++}.brep");
        Assert.True(BRepTools.Write(theShape, aFile));
        var aNShape = new N.TopoDS_Shape();
        using (var aBuilder = new N.BRep_Builder())
        {
            Assert.True(N.BRepTools.Read(aNShape, aFile, aBuilder));
        }
        Assert.True(BRepTools.Read(out TopoDS_Shape aPShape, aFile));
        aPShape.TShape()!.FreezeGraph();
        return (aNShape, aPShape);
    }

    private static TopoDS_Wire Polygon(IReadOnlyList<Point3D> thePoints, bool theClosed)
    {
        BRepBuilderAPI_MakePolygon aMP = new();
        foreach (Point3D aP in thePoints)
        {
            aMP.Add(aP);
        }
        if (theClosed)
        {
            aMP.Close();
        }
        return aMP.Wire();
    }

    [Fact]
    public void Axe_on_random_spines_and_profiles()
    {
        var anOracle = new Oracle(output, 20261105);
        int aCases = 0, aOnVertex = 0, aOnSpine = 0, anExceptions = 0;
        List<string> aDifferences = [];
        for (int aCase = 0; aCase < 300; aCase++)
        {
            // a convex or star-shaped polygon in a random plane
            CoordinateSystem aPlane = anOracle.Bool() ? new CoordinateSystem() : anOracle.Ax3();
            int aNb = anOracle.Int(3, 8);
            List<Point3D> aSpinePoints = [];
            for (int i = 0; i < aNb; i++)
            {
                double anAngle = 2 * Math.PI * i / aNb + anOracle.Real(-0.2, 0.2);
                double aRadius = anOracle.Real(5, 20);
                Point3D aP = new(aRadius * Math.Cos(anAngle), aRadius * Math.Sin(anAngle), 0);
                aSpinePoints.Add(aP.Transformed(TrsfFrom(aPlane)));
            }
            TopoDS_Wire aSpineWire = Polygon(aSpinePoints, true);
            bool isFace = anOracle.Bool();
            TopoDS_Shape aSpine = isFace ? new BRepBuilderAPI_MakeFace(aSpineWire, true).Face() : aSpineWire;

            // a profile of 1 to 3 segments, starting on a spine vertex or somewhere near the spine
            bool isOnVertex = anOracle.Int(0, 3) == 0;
            Point3D aStart = isOnVertex ? aSpinePoints[anOracle.Int(0, aNb - 1)]
                                       : aSpinePoints[0].Translated(new Vector3D(anOracle.Real(-3, 3), anOracle.Real(-3, 3), anOracle.Real(-3, 3)));
            List<Point3D> aProfilePoints = [aStart];
            int aNbSeg = anOracle.Int(1, 3);
            for (int i = 0; i < aNbSeg; i++)
            {
                aProfilePoints.Add(aProfilePoints[^1].Translated(new Vector3D(anOracle.Real(-4, 4), anOracle.Real(-4, 4), anOracle.Real(-4, 4))));
            }
            TopoDS_Wire aProfile = Polygon(aProfilePoints, false);
            double aTol = anOracle.Bool() ? 1.0e-7 : anOracle.Real(0.01, 3);

            (N.TopoDS_Shape aNSpine, TopoDS_Shape aPSpine) = Both(aSpine);
            (N.TopoDS_Shape aNProf, TopoDS_Shape aPProf) = Both(aProfile);
            aCases++;
            aOnVertex += isOnVertex ? 1 : 0;

            string? aNError = null, aPError = null;
            var aNAxe = new N.gp_Ax3();
            bool aNOn = false;
            try
            {
                N.BRepFill.Axe(aNSpine, N.TopoDS.Wire(aNProf), aNAxe, ref aNOn, aTol);
            }
            catch (N.OcctException e)
            {
                aNError = e.Message.IndexOf(':') is int c and > 0 ? e.Message[..c] : e.Message;
            }
            CoordinateSystem aPAxe = new();
            bool aPOn = false;
            try
            {
                BRepFill.Axe(aPSpine, TopoDS.Wire(aPProf), out aPAxe, out aPOn, aTol);
            }
            catch (FailureException f)
            {
                aPError = global::Kerf.Engine.Foundation.OcctTypeNames.Of(f.GetType());
            }
            string aWhat = $"case {aCase} ({(isFace ? "face" : "wire")}{(isOnVertex ? ", on a vertex" : "")})";
            if (aNError is not null || aPError is not null)
            {
                if (aNError != aPError)
                {
                    aDifferences.Add($"{aWhat}: {aNError ?? "no exception"} natively, {aPError ?? "no exception"} in the port");
                }
                anExceptions++;
                continue;
            }
            aOnSpine += aPOn ? 1 : 0;
            if (aNOn != aPOn)
            {
                aDifferences.Add($"{aWhat}: ProfOnSpine {aNOn} natively, {aPOn} in the port");
            }
            Point3D aNL = Oracle.FromNative(aNAxe.Location());
            Xyz aND = Oracle.FromNative(aNAxe.Direction().XYZ()), aNX = Oracle.FromNative(aNAxe.XDirection().XYZ());
            if (aNL.XYZ() != aPAxe.Location().XYZ() || aND != aPAxe.Direction().XYZ() || aNX != aPAxe.XDirection().XYZ())
            {
                aDifferences.Add($"{aWhat}: native {aNL} {aND} {aNX}, port {aPAxe.Location()} {aPAxe.Direction()} {aPAxe.XDirection()}");
            }
        }
        output.WriteLine($"BRepFill.Axe: {aCases} cases ({aOnVertex} profiles starting on a spine vertex, {aOnSpine} on the spine, "
                         + $"{anExceptions} the same exception), {aDifferences.Count} different");
        foreach (string aD in aDifferences.Take(20))
        {
            output.WriteLine("  " + aD);
        }
        Assert.Empty(aDifferences);
    }

    private static Transform3D TrsfFrom(in CoordinateSystem theAx3)
    {
        Transform3D aT = new();
        aT.SetTransformation(theAx3);
        return aT.Inverted();
    }
}
