#if HAS_ORACLE
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Reflection;
using Xunit.Abstractions;
using N = Build123d.Occt;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// IntTools_FClass2d against native OCCT on every face of the brep files of data/occ: random points in and around each
/// face's UV box, points near its edges' pcurves, and the same points shifted by periods; Perform with and without the
/// periodic adjustment, TestOnRestriction at two tolerances, PerformInfinitePoint and IsHole.
/// </summary>
/// <remarks>
/// Both sides read the same file, and explore its faces in the same order. The points are computed once (with the
/// port's UV bounds and pcurves) and given to both sides as the same doubles. One classifier per face and side is
/// reused for all its points, as IntTools_Context reuses it.
/// </remarks>
public class IntTools_FClass2dOracleTests(ITestOutputHelper output)
{
    private const int MaxFacesPerFile = 400;
    private const int PointsPerFace = 24;

    private static string Metadata(string theKey)
        => typeof(IntTools_FClass2dOracleTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>()
                                                .Single(a => a.Key == theKey).Value!;

    private readonly List<string> myMismatches = [];
    private int myComparisons;

    private void Same(int theNative, int thePort, string theWhat)
    {
        myComparisons++;
        if (theNative != thePort)
        {
            myMismatches.Add($"{theWhat}: native {theNative}, port {thePort}");
        }
    }

    private static IEnumerable<gp_Pnt2d> Points(Random theRandom, in TopoDS_Face theF)
    {
        var aList = new List<gp_Pnt2d>();
        BRepTools.UVBounds(theF, out double u1, out double u2, out double v1, out double v2);
        double du = u2 - u1, dv = v2 - v1;
        var aS = new BRepAdaptor_Surface(theF, false);
        // In and around the UV box
        for (int i = 0; i < PointsPerFace / 2; i++)
        {
            aList.Add(new gp_Pnt2d(u1 - 0.2 * du + 1.4 * du * theRandom.NextDouble(), v1 - 0.2 * dv + 1.4 * dv * theRandom.NextDouble()));
        }
        // Near the pcurves, at distances from 0 to 1e-3 of the box
        var anEdges = new List<TopoDS_Edge>();
        for (var anExp = new TopExp_Explorer(theF, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            anEdges.Add(TopoDS.Edge(anExp.Current()));
        }
        for (int i = 0; i < PointsPerFace / 2 && anEdges.Count > 0; i++)
        {
            TopoDS_Edge anE = anEdges[theRandom.Next(anEdges.Count)];
            Geom2d_Curve? aC = BRep_Tool.CurveOnSurface(anE, theF, out double f, out double l);
            if (aC is null)
            {
                continue;
            }
            gp_Pnt2d aP = aC.Value(f + (l - f) * theRandom.NextDouble());
            double aScale = theRandom.Next(4) switch { 0 => 0.0, 1 => 1e-9, 2 => 1e-6, _ => 1e-3 };
            aList.Add(new gp_Pnt2d(aP.X() + (theRandom.NextDouble() - 0.5) * aScale * du,
                                   aP.Y() + (theRandom.NextDouble() - 0.5) * aScale * dv));
        }
        // Shifted by periods
        int n = aList.Count;
        for (int i = 0; i < n; i += 3)
        {
            double x = aList[i].X(), y = aList[i].Y();
            if (aS.IsUPeriodic())
            {
                x += aS.UPeriod() * theRandom.Next(-2, 3);
            }
            if (aS.IsVPeriodic())
            {
                y += aS.VPeriod() * theRandom.Next(-2, 3);
            }
            if (aS.IsUPeriodic() || aS.IsVPeriodic())
            {
                aList.Add(new gp_Pnt2d(x, y));
            }
        }
        return aList;
    }

    private int myFaces, myPoints;
    private readonly int[] myStates = new int[4];

    private void CompareFile(string theFile, Random theRandom, int theMaxFaces)
    {
        string aName = Path.GetFileName(theFile);
        var aNShape = new N.TopoDS_Shape();
        using (var aBuilder = new N.BRep_Builder())
        {
            if (!N.BRepTools.Read(aNShape, theFile, aBuilder))
            {
                return;
            }
        }
        if (!BRepTools.Read(out TopoDS_Shape aPShape, theFile))
        {
            myMismatches.Add($"{aName}: the port can't read it");
            return;
        }

        var aNExp = new N.TopExp_Explorer(aNShape, N.TopAbs_ShapeEnum.TopAbs_FACE);
        var aPExp = new TopExp_Explorer(aPShape, TopAbs_FACE);
        int iF = 0;
        for (; aNExp.More() && iF < theMaxFaces; aNExp.Next(), aPExp.Next(), iF++)
        {
            Assert.True(aPExp.More());
            N.TopoDS_Face aNF = N.TopoDS.Face(aNExp.Current());
            TopoDS_Face aPF = TopoDS.Face(aPExp.Current());
            if (BRep_Tool.Surface(aPF) is null)
            {
                continue; // a face without a surface: neither side can classify on it
            }
            double aTol = BRep_Tool.Tolerance(aPF);
            string aWhat = $"{aName} face {iF}";
            IntTools_FClass2d aP;
            try
            {
                aP = new IntTools_FClass2d(aPF, aTol);
            }
            catch (Exception e)
            {
                myMismatches.Add($"{aWhat}: port Init throws {e.GetType().Name}");
                continue;
            }
            using var aN = new N.IntTools_FClass2d(aNF, aTol);
            myFaces++;
            Same(aN.IsHole() ? 1 : 0, aP.IsHole() ? 1 : 0, $"{aWhat} IsHole");
            Same((int)aN.PerformInfinitePoint(), (int)aP.PerformInfinitePoint(), $"{aWhat} PerformInfinitePoint");
            foreach (gp_Pnt2d aPnt in Points(theRandom, aPF))
            {
                myPoints++;
                string aAt = $"{aWhat} at ({aPnt.X():R}, {aPnt.Y():R})";
                using N.gp_Pnt2d aNPnt = Oracle.ToNative(aPnt);
                int aState = (int)aP.Perform(aPnt);
                myStates[aState]++;
                Same((int)aN.Perform(aNPnt), aState, $"{aAt} Perform");
                Same((int)aN.Perform(aNPnt, false), (int)aP.Perform(aPnt, false), $"{aAt} Perform without recadre");
                Same((int)aN.TestOnRestriction(aNPnt, 1e-3), (int)aP.TestOnRestriction(aPnt, 1e-3), $"{aAt} TestOnRestriction 1e-3");
                Same((int)aN.TestOnRestriction(aNPnt, 1e-7, false), (int)aP.TestOnRestriction(aPnt, 1e-7, false),
                     $"{aAt} TestOnRestriction 1e-7 without recadre");
            }
        }
    }

    private void ReportAndCheck(string theWhat, int theFiles)
    {
        output.WriteLine($"{theWhat}: {theFiles} files, {myFaces} faces, {myPoints} points, {myComparisons} comparisons, "
                         + $"{myMismatches.Count} mismatches; port's Perform states IN {myStates[0]}, OUT {myStates[1]}, "
                         + $"ON {myStates[2]}, UNKNOWN {myStates[3]}");
        foreach (string aM in myMismatches.Take(40))
        {
            output.WriteLine(aM);
        }
        Assert.Empty(myMismatches);
    }

    [Fact]
    public void FClass2d_matches_native_OCCT_on_the_faces_of_data_occ()
    {
        string[] aFiles = Directory.GetFiles(Path.Combine(Metadata("OcctSourceDir"), "data", "occ"), "*.brep")
                                   .Order(StringComparer.Ordinal).ToArray();
        var aRandom = new Random(64110);
        foreach (string aFile in aFiles)
        {
            CompareFile(aFile, aRandom, MaxFacesPerFile);
        }
        ReportAndCheck("data/occ", aFiles.Length);
    }

    /// <summary>
    /// Every fourth brep file of OCCT's test dataset under 300 KB (the corpus's inputs, many of them faulty: bad wires,
    /// missing pcurves, degenerated edges), up to 60 faces each.
    /// </summary>
    [Fact]
    public void FClass2d_matches_native_OCCT_on_the_test_dataset()
    {
        string aRoot = Metadata("OcctTestDataDir");
        if (string.IsNullOrEmpty(aRoot) || !Directory.Exists(Path.Combine(aRoot, "brep")))
        {
            output.WriteLine("OcctTestDataDir isn't set: nothing to check");
            return;
        }
        string[] aFiles = Directory.GetFiles(Path.Combine(aRoot, "brep"), "*.brep")
                                   .Where(f => new FileInfo(f).Length < 300_000)
                                   .Order(StringComparer.Ordinal).Where((_, i) => i % 4 == 0).ToArray();
        var aRandom = new Random(64111);
        foreach (string aFile in aFiles)
        {
            CompareFile(aFile, aRandom, 60);
        }
        ReportAndCheck("test dataset", aFiles.Length);
    }
}
#endif
