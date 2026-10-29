#if HAS_ORACLE
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Text;
using System.Text.RegularExpressions;
using Xunit.Abstractions;
using N = Build123d.Occt;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// The port's BRepPrimAPI_MakePrism and MakeRevol against native OCCT's on the same random profiles (vertices, line and
/// arc edges, open and closed polygons, planar faces with and without holes, disks, cylinder patches, shells and
/// compounds), vectors, directions, axes and angles, with and without copy and canonization. The results are compared
/// through BRep text: the port's profiles are frozen (made by the public makers), so the port sweeps a copy and the
/// result shares the profile's unchanged sub-shapes, which keep Free = 1 where native OCCT clears it (design 4.4); files
/// that differ are compared again with the Free column masked. The histories (IsDeleted, Generated, FirstShape and
/// LastShape of every sub-shape of the profile, and the whole FirstShape and LastShape) are compared as indices and
/// orientations in the result's map of sub-shapes.
/// </summary>
public sealed class BRepPrimAPISweepOracleTests(ITestOutputHelper output) : IDisposable
{
    private readonly string myDirectory = Directory.CreateTempSubdirectory("occtnet-sweep-").FullName;
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

    private string NewFile() => Path.Combine(myDirectory, $"s{myFileNumber++}.brep");

    private string NativeText(N.TopoDS_Shape theShape)
    {
        string aFile = NewFile();
        Assert.True(N.BRepTools.Write(theShape, aFile));
        return File.ReadAllText(aFile, Encoding.Latin1);
    }

    private string PortText(in TopoDS_Shape theShape)
    {
        string aFile = NewFile();
        Assert.True(BRepTools.Write(theShape, aFile));
        return File.ReadAllText(aFile, Encoding.Latin1);
    }

    // The Free flag is the first of the seven flag digits on their own line.
    private static string MaskFree(string theText) => Regex.Replace(theText, "(?m)^[01]([01]{6})$", "x$1");

    private sealed class Tally(string theWhat)
    {
        public int Files;
        public int Identical;
        public int MaskedEqual;
        public int Failed;
        public int Invalid;
        public int HistoryValues;
        public readonly Dictionary<string, int> Kinds = [];
        public readonly List<string> Differences = [];

        public string Report() =>
            $"{theWhat}: {Files} files, {Identical} byte-identical, {MaskedEqual} equal with the Free column masked, "
            + $"{Failed} different; {Invalid} invalid on both sides; {HistoryValues} history values equal; by profile: "
            + string.Join(", ", Kinds.OrderBy(k => k.Key).Select(k => $"{k.Key} {k.Value}"));
    }

    private void Compare(Tally theTally, string theWhat, N.TopoDS_Shape theNative, in TopoDS_Shape thePort)
    {
        string aN = NativeText(theNative);
        string aP = PortText(thePort);
        theTally.Files++;
        if (aN == aP)
        {
            theTally.Identical++;
        }
        else if (MaskFree(aN) == MaskFree(aP))
        {
            theTally.MaskedEqual++;
        }
        else
        {
            theTally.Failed++;
            if (theTally.Differences.Count < 8)
            {
                // the first lines that differ, with the Free column masked
                string[] aNL = MaskFree(aN).Split('\n'), aPL = MaskFree(aP).Split('\n');
                var aDiff = new StringBuilder();
                for (int i = 0, n = 0; i < Math.Max(aNL.Length, aPL.Length) && n < 6; i++)
                {
                    string a = i < aNL.Length ? aNL[i] : "", b = i < aPL.Length ? aPL[i] : "";
                    if (a != b)
                    {
                        aDiff.Append($"  line {i + 1}: native [{a.TrimEnd()}] port [{b.TrimEnd()}]\n");
                        n++;
                    }
                }
                theTally.Differences.Add($"{theWhat} ({aNL.Length} and {aPL.Length} lines):\n{aDiff}");
            }
        }
    }

    private void Finish(Tally theTally)
    {
        output.WriteLine(theTally.Report());
        foreach (string aD in theTally.Differences)
        {
            output.WriteLine(aD);
        }
        Assert.Equal(0, theTally.Failed);
    }

    // ------------------------------------------------------------ settled inputs (as BRepLibMakersOracleTests)

    private static bool Same(N.gp_Dir theN, in gp_Dir theP) => theN.X() == theP.X() && theN.Y() == theP.Y() && theN.Z() == theP.Z();

    private static gp_Dir FromNative(N.gp_Dir theD) => new(theD.X(), theD.Y(), theD.Z());

    private static gp_Dir Dir(Oracle theO)
    {
        while (true)
        {
            gp_Dir aD = Oracle.Settled(theO.Dir());
            using var aND = Oracle.ToNative(aD);
            if (Same(aND, aD))
            {
                return aD;
            }
        }
    }

    private static gp_Ax3 Ax3(Oracle theO)
    {
        while (true)
        {
            gp_Pnt aP = new(theO.Real(-20, 20), theO.Real(-20, 20), theO.Real(-20, 20));
            using var aNP = Oracle.ToNative(aP);
            using var aN0 = Oracle.ToNative(theO.Dir());
            using var aX0 = Oracle.ToNative(theO.Dir());
            var aNAx = new N.gp_Ax2(aNP, aN0, aX0);
            for (int i = 0; i < 4; i++)
            {
                using var aD = aNAx.Direction();
                using var aX = aNAx.XDirection();
                aNAx = new N.gp_Ax2(aNP, aD, aX);
            }
            using var aND = aNAx.Direction();
            using var aNX = aNAx.XDirection();
            var anAx = new gp_Ax3(new gp_Ax2(aP, FromNative(aND), FromNative(aNX)));
            using var aN3 = Oracle.ToNative(anAx);
            using var aD3 = aN3.Direction();
            using var aX3 = aN3.XDirection();
            using var aY3 = aN3.YDirection();
            if (Same(aD3, anAx.Direction()) && Same(aX3, anAx.XDirection()) && Same(aY3, anAx.YDirection()))
            {
                return anAx;
            }
        }
    }

    // A placement of a circle that native OCCT holds with the same directions, or false.
    private static bool SettledAx2(in gp_Ax2 theAx)
    {
        using var aN = Oracle.ToNative(theAx);
        using var aD = aN.Direction();
        using var aX = aN.XDirection();
        using var aY = aN.YDirection();
        return Same(aD, theAx.Direction()) && Same(aX, theAx.XDirection()) && Same(aY, theAx.YDirection());
    }

    // A random simple polygon in the XY plane of the placement: points at increasing angles around its origin.
    private static gp_Pnt[] StarPolygon(Oracle theO, in gp_Ax3 theAx, int theN, double theScale)
    {
        var aPts = new gp_Pnt[theN];
        double anAngle = theO.Real(0, 1);
        for (int i = 0; i < theN; i++)
        {
            anAngle += theO.Real(0.3, 2 * Math.PI / theN);
            double r = theScale * theO.Real(0.5, 1.0);
            aPts[i] = ElSLib.PlaneValue(r * Math.Cos(anAngle), r * Math.Sin(anAngle), theAx);
        }
        return aPts;
    }

    private static (N.TopoDS_Wire, TopoDS_Wire) Polygon(gp_Pnt[] thePoints, bool theClosed)
    {
        var aNP = new N.BRepBuilderAPI_MakePolygon();
        var aPP = new BRepBuilderAPI_MakePolygon();
        foreach (gp_Pnt aP in thePoints)
        {
            using var aN = Oracle.ToNative(aP);
            aNP.Add(aN);
            aPP.Add(aP);
        }
        if (theClosed)
        {
            aNP.Close();
            aPP.Close();
        }
        return (aNP.Wire(), aPP.Wire());
    }

    private const int NbKinds = 12;

    /// <summary>
    /// A profile in the XY plane of <paramref name="theAx"/>, within <paramref name="theScale"/> of its origin (kind 11:
    /// a triangle with an edge on the placement's Y axis), built the same way on both sides; null when a circle's
    /// placement can't be settled.
    /// </summary>
    private static (N.TopoDS_Shape N, TopoDS_Shape P, string What)? Profile(Oracle theO, int theKind, in gp_Ax3 theAx, double theScale)
    {
        switch (theKind)
        {
            case 0:
            {
                gp_Pnt aP = ElSLib.PlaneValue(theO.Real(-theScale, theScale) * 0.7, theO.Real(-theScale, theScale) * 0.7, theAx);
                using var aN = Oracle.ToNative(aP);
                return (new N.BRepBuilderAPI_MakeVertex(aN).Shape(), new BRepBuilderAPI_MakeVertex(aP).Shape(), "vertex");
            }
            case 1:
            {
                gp_Pnt[] aPts = StarPolygon(theO, theAx, 2, theScale);
                using var aN1 = Oracle.ToNative(aPts[0]);
                using var aN2 = Oracle.ToNative(aPts[1]);
                return (new N.BRepBuilderAPI_MakeEdge(aN1, aN2).Shape(), new BRepBuilderAPI_MakeEdge(aPts[0], aPts[1]).Shape(), "line");
            }
            case 2:
            {
                gp_Pnt aC = ElSLib.PlaneValue(theO.Real(-0.3, 0.3) * theScale, theO.Real(-0.3, 0.3) * theScale, theAx);
                var anAx2 = new gp_Ax2(aC, theAx.Direction(), theAx.XDirection());
                if (!SettledAx2(anAx2))
                {
                    return null;
                }
                double r = theO.Real(0.2, 0.6) * theScale;
                double a1 = theO.Real(0, 3), a2 = a1 + theO.Real(0.3, 3);
                var aCirc = new gp_Circ(anAx2, r);
                using var aNAx = Oracle.ToNative(anAx2);
                using var aNC = new N.gp_Circ(aNAx, r);
                return (new N.BRepBuilderAPI_MakeEdge(aNC, a1, a2).Shape(), new BRepBuilderAPI_MakeEdge(aCirc, a1, a2).Shape(), "arc");
            }
            case 3:
            case 4:
            {
                gp_Pnt[] aPts = StarPolygon(theO, theAx, theO.Int(3, 6), theScale);
                (N.TopoDS_Wire aNW, TopoDS_Wire aPW) = Polygon(aPts, theKind == 4);
                return (aNW, aPW, theKind == 4 ? "closed polygon" : "open polygon");
            }
            case 5:
            case 6:
            {
                gp_Pnt[] aPts = StarPolygon(theO, theAx, theO.Int(3, 7), theScale);
                if (theO.Bool())
                {
                    Array.Reverse(aPts);
                }
                (N.TopoDS_Wire aNW, TopoDS_Wire aPW) = Polygon(aPts, true);
                N.TopoDS_Face aNF = new N.BRepBuilderAPI_MakeFace(aNW, true).Face();
                TopoDS_Face aPF = new BRepBuilderAPI_MakeFace(aPW, true).Face();
                if (theKind == 6)
                {
                    gp_Pnt[] aHole = StarPolygon(theO, theAx, theO.Int(3, 6), 0.2 * theScale);
                    (N.TopoDS_Wire aNH, TopoDS_Wire aPH) = Polygon(aHole, true);
                    aNF = new N.BRepBuilderAPI_MakeFace(aNF, N.TopoDS.Wire(aNH.Reversed())).Face();
                    aPF = new BRepBuilderAPI_MakeFace(aPF, TopoDS.Wire(aPH.Reversed())).Face();
                }
                return (aNF, aPF, theKind == 6 ? "face with hole" : "polygon face");
            }
            case 7:
            {
                gp_Pnt aC = ElSLib.PlaneValue(theO.Real(-0.3, 0.3) * theScale, theO.Real(-0.3, 0.3) * theScale, theAx);
                var anAx2 = new gp_Ax2(aC, theAx.Direction(), theAx.XDirection());
                if (!SettledAx2(anAx2))
                {
                    return null;
                }
                double r = theO.Real(0.2, 0.6) * theScale;
                var aCirc = new gp_Circ(anAx2, r);
                using var aNAx = Oracle.ToNative(anAx2);
                using var aNC = new N.gp_Circ(aNAx, r);
                N.TopoDS_Wire aNW = new N.BRepBuilderAPI_MakeWire(new N.BRepBuilderAPI_MakeEdge(aNC).Edge()).Wire();
                TopoDS_Wire aPW = new BRepBuilderAPI_MakeWire(new BRepBuilderAPI_MakeEdge(aCirc).Edge()).Wire();
                return (new N.BRepBuilderAPI_MakeFace(aNW, true).Face(), new BRepBuilderAPI_MakeFace(aPW, true).Face(), "disk");
            }
            case 8:
            {
                // a cylinder patch around the placement's normal, near its origin
                double r = theO.Real(0.2, 0.6) * theScale;
                double u1 = theO.Real(0, 3), u2 = u1 + theO.Real(0.3, 3);
                double v1 = theO.Real(-0.3, 0) * theScale, v2 = v1 + theO.Real(0.1, 0.3) * theScale;
                using var aNAx = Oracle.ToNative(theAx);
                using var aNCyl = new N.gp_Cylinder(aNAx, r);
                return (new N.BRepBuilderAPI_MakeFace(aNCyl, u1, u2, v1, v2).Face(),
                        new BRepBuilderAPI_MakeFace(new gp_Cylinder(theAx, r), u1, u2, v1, v2).Face(), "cylinder patch");
            }
            case 9:
            {
                double u1 = theO.Real(-0.7, 0) * theScale, u2 = u1 + theO.Real(0.2, 0.7) * theScale;
                double v1 = theO.Real(-0.7, 0) * theScale, v2 = v1 + theO.Real(0.2, 0.7) * theScale;
                using var aNAx = Oracle.ToNative(theAx);
                return (new N.BRepBuilderAPI_MakeShell(new N.Geom_Plane(aNAx), u1, u2, v1, v2).Shape(),
                        new BRepBuilderAPI_MakeShell(new Geom_Plane(theAx), u1, u2, v1, v2).Shape(), "shell");
            }
            case 10:
            {
                // a compound of two polygon faces
                gp_Pnt[] aPts1 = StarPolygon(theO, theAx, theO.Int(3, 5), 0.4 * theScale);
                gp_Pnt[] aPts2 = StarPolygon(theO, theAx, theO.Int(3, 5), 0.4 * theScale);
                var aT = new gp_Vec(theAx.XDirection()).Multiplied(0.5 * theScale);
                for (int i = 0; i < aPts2.Length; i++)
                {
                    aPts2[i] = aPts2[i].Translated(aT);
                }
                (N.TopoDS_Wire aNW1, TopoDS_Wire aPW1) = Polygon(aPts1, true);
                (N.TopoDS_Wire aNW2, TopoDS_Wire aPW2) = Polygon(aPts2, true);
                var aNC = new N.TopoDS_Compound();
                var aNB = new N.BRep_Builder();
                aNB.MakeCompound(aNC);
                aNB.Add(aNC, new N.BRepBuilderAPI_MakeFace(aNW1, true).Face());
                aNB.Add(aNC, new N.BRepBuilderAPI_MakeFace(aNW2, true).Face());
                TopoDS_Compound aPC = TopoDS_Builder.MakeCompound(new BRepBuilderAPI_MakeFace(aPW1, true).Face(),
                                                                  new BRepBuilderAPI_MakeFace(aPW2, true).Face());
                return (aNC, aPC, "compound");
            }
            default:
            {
                // a triangle with an edge on the placement's Y axis
                double y1 = theO.Real(-0.7, 0) * theScale, y2 = theO.Real(0.1, 0.7) * theScale;
                gp_Pnt[] aPts =
                [
                    ElSLib.PlaneValue(0, y1, theAx), ElSLib.PlaneValue(theO.Real(0.2, 0.7) * theScale, theO.Real(y1, y2), theAx),
                    ElSLib.PlaneValue(0, y2, theAx),
                ];
                (N.TopoDS_Wire aNW, TopoDS_Wire aPW) = Polygon(aPts, true);
                if (theO.Bool())
                {
                    return (aNW, aPW, "triangle wire on axis");
                }
                return (new N.BRepBuilderAPI_MakeFace(aNW, true).Face(), new BRepBuilderAPI_MakeFace(aPW, true).Face(),
                        "triangle on axis");
            }
        }
    }

    // ------------------------------------------------------------ histories

    private static (int, int) Index(N.TopTools_IndexedMapOfShape theMap, N.TopoDS_Shape theS) =>
        theS.IsNull() ? (-1, -1) : (theMap.FindIndex(theS), (int)theS.Orientation());

    private static (int, int) Index(NCollection_IndexedMap<TopoDS_Shape> theMap, in TopoDS_Shape theS) =>
        theS.IsNull() ? (-1, -1) : (theMap.FindIndex(theS), (int)theS.Orientation());

    private interface ISweepPair
    {
        bool IsDeleted(N.TopoDS_Shape theN, in TopoDS_Shape theP, out bool thePort);
        (N.TopTools_ListOfShape, IEnumerable<TopoDS_Shape>) Generated(N.TopoDS_Shape theN, in TopoDS_Shape theP);
        (N.TopoDS_Shape, TopoDS_Shape) First(N.TopoDS_Shape theN, in TopoDS_Shape theP);
        (N.TopoDS_Shape, TopoDS_Shape) Last(N.TopoDS_Shape theN, in TopoDS_Shape theP);
        (N.TopoDS_Shape, TopoDS_Shape) First();
        (N.TopoDS_Shape, TopoDS_Shape) Last();
    }

    private sealed class PrismPair(N.BRepPrimAPI_MakePrism theN, BRepPrimAPI_MakePrism theP) : ISweepPair
    {
        public bool IsDeleted(N.TopoDS_Shape theNS, in TopoDS_Shape thePS, out bool thePort)
        {
            thePort = theP.IsDeleted(thePS);
            return theN.IsDeleted(theNS);
        }

        public (N.TopTools_ListOfShape, IEnumerable<TopoDS_Shape>) Generated(N.TopoDS_Shape theNS, in TopoDS_Shape thePS) =>
            (theN.Generated(theNS), theP.Generated(thePS));

        public (N.TopoDS_Shape, TopoDS_Shape) First(N.TopoDS_Shape theNS, in TopoDS_Shape thePS) =>
            (theN.FirstShape(theNS), theP.FirstShape(thePS));

        public (N.TopoDS_Shape, TopoDS_Shape) Last(N.TopoDS_Shape theNS, in TopoDS_Shape thePS) =>
            (theN.LastShape(theNS), theP.LastShape(thePS));

        public (N.TopoDS_Shape, TopoDS_Shape) First() => (theN.FirstShape(), theP.FirstShape());

        public (N.TopoDS_Shape, TopoDS_Shape) Last() => (theN.LastShape(), theP.LastShape());
    }

    private sealed class RevolPair(N.BRepPrimAPI_MakeRevol theN, BRepPrimAPI_MakeRevol theP) : ISweepPair
    {
        public bool IsDeleted(N.TopoDS_Shape theNS, in TopoDS_Shape thePS, out bool thePort)
        {
            thePort = theP.IsDeleted(thePS);
            return theN.IsDeleted(theNS);
        }

        public (N.TopTools_ListOfShape, IEnumerable<TopoDS_Shape>) Generated(N.TopoDS_Shape theNS, in TopoDS_Shape thePS) =>
            (theN.Generated(theNS), theP.Generated(thePS));

        public (N.TopoDS_Shape, TopoDS_Shape) First(N.TopoDS_Shape theNS, in TopoDS_Shape thePS) =>
            (theN.FirstShape(theNS), theP.FirstShape(thePS));

        public (N.TopoDS_Shape, TopoDS_Shape) Last(N.TopoDS_Shape theNS, in TopoDS_Shape thePS) =>
            (theN.LastShape(theNS), theP.LastShape(thePS));

        public (N.TopoDS_Shape, TopoDS_Shape) First() => (theN.FirstShape(), theP.FirstShape());

        public (N.TopoDS_Shape, TopoDS_Shape) Last() => (theN.LastShape(), theP.LastShape());
    }

    // Compares the histories of every sub-shape of the profile as indices and orientations in the result's map.
    private static void CompareHistories(Tally theTally, string theWhat, ISweepPair theSweep, N.TopoDS_Shape theNProfile,
                                         in TopoDS_Shape thePProfile, N.TopoDS_Shape theNResult, in TopoDS_Shape thePResult)
    {
        using var aNP = new N.TopTools_IndexedMapOfShape();
        N.TopExp.MapShapes(theNProfile, aNP);
        var aPP = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(thePProfile, aPP);
        Assert.Equal(aNP.Extent(), aPP.Extent());

        using var aNR = new N.TopTools_IndexedMapOfShape();
        var aPR = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        if (!theNResult.IsNull())
        {
            N.TopExp.MapShapes(theNResult, aNR);
            TopExp.MapShapes(thePResult, aPR);
        }
        Assert.Equal(aNR.Extent(), aPR.Extent());

        (N.TopoDS_Shape aNF, TopoDS_Shape aPF) = theSweep.First();
        Assert.True(Index(aNR, aNF) == Index(aPR, aPF), $"{theWhat}: FirstShape");
        (N.TopoDS_Shape aNL, TopoDS_Shape aPL) = theSweep.Last();
        Assert.True(Index(aNR, aNL) == Index(aPR, aPL), $"{theWhat}: LastShape");
        theTally.HistoryValues += 2;

        for (int i = 1; i <= aNP.Extent(); i++)
        {
            N.TopoDS_Shape aNS = aNP.FindKey(i);
            TopoDS_Shape aPS = aPP[i];
            bool aNDel = theSweep.IsDeleted(aNS, aPS, out bool aPDel);
            Assert.True(aNDel == aPDel, $"{theWhat}: IsDeleted of sub-shape {i}");

            (N.TopTools_ListOfShape aNG, IEnumerable<TopoDS_Shape> aPG) = theSweep.Generated(aNS, aPS);
            var aNGI = aNG.Select(s => Index(aNR, s)).ToList();
            var aPGI = aPG.Select(s => Index(aPR, s)).ToList();
            Assert.True(aNGI.SequenceEqual(aPGI), $"{theWhat}: Generated of sub-shape {i}");

            (N.TopoDS_Shape aNSF, TopoDS_Shape aPSF) = theSweep.First(aNS, aPS);
            Assert.True(Index(aNR, aNSF) == Index(aPR, aPSF), $"{theWhat}: FirstShape of sub-shape {i}");
            (N.TopoDS_Shape aNSL, TopoDS_Shape aPSL) = theSweep.Last(aNS, aPS);
            Assert.True(Index(aNR, aNSL) == Index(aPR, aPSL), $"{theWhat}: LastShape of sub-shape {i}");
            theTally.HistoryValues += 4 + aNGI.Count;
        }
    }

    private static void Count(Tally theTally, string theWhat) =>
        theTally.Kinds[theWhat] = theTally.Kinds.TryGetValue(theWhat, out int n) ? n + 1 : 1;

    // ------------------------------------------------------------ prisms

    [Fact]
    public void MakePrism_builds_the_prisms_native_OCCT_builds()
    {
        var anO = new Oracle(output, 3201);
        var aT = new Tally("prisms");
        for (int aCase = 0; aCase < 480; aCase++)
        {
            gp_Ax3 anAx = Ax3(anO);
            double aScale = anO.Real(1, 10);
            var aProfile = Profile(anO, aCase % NbKinds, anAx, aScale);
            if (aProfile is null)
            {
                continue;
            }
            (N.TopoDS_Shape aNS, TopoDS_Shape aPS, string aWhat) = aProfile.Value;
            bool aCopy = anO.Int(0, 3) == 0;
            bool aCanonize = anO.Int(0, 3) != 0;

            N.BRepPrimAPI_MakePrism aNM;
            BRepPrimAPI_MakePrism aPM;
            int aMode = anO.Int(0, 3);
            if (aMode == 0)
            {
                // a semi-infinite or infinite prism
                gp_Dir aD = Dir(anO);
                bool anInf = anO.Bool();
                using var aND = Oracle.ToNative(aD);
                aNM = new N.BRepPrimAPI_MakePrism(aNS, aND, anInf, aCopy, aCanonize);
                aPM = new BRepPrimAPI_MakePrism(aPS, aD, anInf, aCopy, aCanonize);
                aWhat += anInf ? ", infinite" : ", semi-infinite";
            }
            else
            {
                // a vector: random, or along the normal of the profile's plane (a planar prism of a planar face)
                gp_Vec aV = aMode == 1
                              ? new gp_Vec(anAx.Direction()).Multiplied(anO.Real(0.5, 10) * (anO.Bool() ? 1 : -1))
                              : new gp_Vec(anO.Real(-10, 10), anO.Real(-10, 10), anO.Real(-10, 10));
                using var aNV = Oracle.ToNative(aV);
                aNM = new N.BRepPrimAPI_MakePrism(aNS, aNV, aCopy, aCanonize);
                aPM = new BRepPrimAPI_MakePrism(aPS, aV, aCopy, aCanonize);
            }
            aWhat += (aCopy ? ", copy" : "") + (aCanonize ? "" : ", not canonized");
            Assert.Equal(aNM.IsDone(), aPM.IsDone());
            N.TopoDS_Shape aNR = aNM.Shape();
            TopoDS_Shape aPR = aPM.Shape();
            Compare(aT, aWhat, aNR, aPR);
            Count(aT, aWhat.Split(',')[0]);
            CompareHistories(aT, aWhat, new PrismPair(aNM, aPM), aNS, aPS, aNR, aPR);
        }
        Finish(aT);
    }

    // ------------------------------------------------------------ revolutions

    [Fact]
    public void MakeRevol_builds_the_revolutions_native_OCCT_builds()
    {
        var anO = new Oracle(output, 3202);
        var aT = new Tally("revolutions");
        for (int aCase = 0; aCase < 480; aCase++)
        {
            gp_Ax3 anAx = Ax3(anO);
            double aScale = anO.Real(1, 10);
            int aKind = aCase % NbKinds;
            var aProfile = Profile(anO, aKind, anAx, aScale);
            if (aProfile is null)
            {
                continue;
            }
            (N.TopoDS_Shape aNS, TopoDS_Shape aPS, string aWhat) = aProfile.Value;
            bool aCopy = anO.Int(0, 3) == 0;

            // the axis: the placement's Y axis for the triangles on it; otherwise parallel to it beyond the profile,
            // or (about a tenth of the time each) a random axis through the placement's origin or its Y axis, which may
            // cross the profile
            // (native gp_Dir normalises the Y direction again: one that doesn't come out the same is settled first)
            gp_Dir aYD = anAx.YDirection();
            using (var aNYD = Oracle.ToNative(aYD))
            {
                if (!Same(aNYD, aYD))
                {
                    aYD = Oracle.Settled(aYD);
                    using var aNYD2 = Oracle.ToNative(aYD);
                    if (!Same(aNYD2, aYD))
                    {
                        continue;
                    }
                }
            }
            gp_Ax1 anAxis;
            if (aKind == NbKinds - 1)
            {
                anAxis = new gp_Ax1(anAx.Location(), aYD);
            }
            else if (anO.Int(0, 9) == 0)
            {
                anAxis = new gp_Ax1(anAx.Location(), Dir(anO));
                aWhat += ", random axis";
            }
            else if (anO.Int(0, 9) == 0)
            {
                anAxis = new gp_Ax1(anAx.Location(), aYD);
                aWhat += ", axis across";
            }
            else
            {
                anAxis = new gp_Ax1(ElSLib.PlaneValue(-aScale * anO.Real(1.1, 2), 0, anAx), aYD);
            }
            using var aNAxis = Oracle.ToNative(anAxis);

            N.BRepPrimAPI_MakeRevol aNM;
            BRepPrimAPI_MakeRevol aPM;
            int aMode = anO.Int(0, 3);
            if (aMode == 0)
            {
                aNM = new N.BRepPrimAPI_MakeRevol(aNS, aNAxis, aCopy);
                aPM = new BRepPrimAPI_MakeRevol(aPS, anAxis, aCopy);
                aWhat += ", whole turn";
            }
            else
            {
                double anAngle = aMode == 1 ? anO.Real(0.05, 2 * Math.PI) : aMode == 2 ? -anO.Real(0.05, 2 * Math.PI) : anO.Real(2 * Math.PI, 9);
                aNM = new N.BRepPrimAPI_MakeRevol(aNS, aNAxis, anAngle, aCopy);
                aPM = new BRepPrimAPI_MakeRevol(aPS, anAxis, anAngle, aCopy);
                aWhat += $", angle {anAngle:R}";
            }
            aWhat += aCopy ? ", copy" : "";
            Assert.Equal(aNM.IsDone(), aPM.IsDone());
            Count(aT, aWhat.Split(',')[0]);
            if (!aNM.IsDone())
            {
                aT.Invalid++;
                continue;
            }
            N.TopoDS_Shape aNR = aNM.Shape();
            TopoDS_Shape aPR = aPM.Shape();
            Compare(aT, aWhat, aNR, aPR);
            Assert.Equal(aNM.HasDegenerated(), aPM.HasDegenerated());
            CompareHistories(aT, aWhat, new RevolPair(aNM, aPM), aNS, aPS, aNR, aPR);
        }
        Finish(aT);
    }
}
#endif
