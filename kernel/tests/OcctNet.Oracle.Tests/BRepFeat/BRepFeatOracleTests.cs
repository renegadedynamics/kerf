// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Brep;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Diagnostics;
using OcctNet.TestHarness;
using Xunit.Abstractions;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// BRepFeat's makers that build123d.NET binds (unit 7I), against native OCCT: <c>BRepFeat_MakeDPrism</c> as
/// build123d.NET's <c>DPrism</c> calls it (through all, by a depth, up to a face; with and without a taper; fusing and
/// cutting) and <c>BRepFeat_SplitShape</c> as its <c>SplitByPerimeter</c> (a sequence of edges on a face, then Left and
/// Right). The inputs (primitives, a Boolean result, and profiles on their planar faces) are written by the port and read
/// by both sides, the port's frozen as a caller's; the results are compared as BRep text with the Free column masked,
/// then canonically (native DPrism varies run to run, decision 1017), and the histories (IsDeleted and the number of
/// Modified faces of every face of the basis shape, Generated of the profile's edges), the status and the exceptions.
/// </summary>
public class BRepFeatOracleTests(ITestOutputHelper output)
{
    private static string? ErrorOf(Exception e) => ShapeFixOracleRun.ErrorOf(e) ?? (e is NullReferenceException ? "OSD_Exception_ACCESS_VIOLATION" : null);

    // The port's shape written and read by both sides; the port's frozen, as a caller's
    private static (N.TopoDS_Shape N, Shape P) Both(string theDirectory, ref int theNumber, in Shape theShape)
        => Both(theDirectory, ref theNumber, theShape, out _);

    private static (N.TopoDS_Shape N, Shape P) Both(string theDirectory, ref int theNumber, in Shape theShape,
                                                         out string theFile)
    {
        string aFile = Path.Combine(theDirectory, $"in{theNumber++}.brep");
        theFile = aFile;
        Assert.True(BrepTools.Write(theShape, aFile, true, false, FormatVersion.FormatVersion_VERSION_3));
        N.TopoDS_Shape aN = ShapeFixOracleRun.NativeRead(aFile)!;
        Assert.True(BrepTools.Read(out Shape aP, aFile));
        aP.TShape()!.FreezeGraph();
        return (aN, aP);
    }

    private static List<Shape> Faces(in Shape theShape)
    {
        var aList = new List<Shape>();
        var aMap = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, ShapeType.Face, aMap);
        for (int i = 1; i <= aMap.Extent(); i++)
        {
            aList.Add(aMap[i]);
        }
        return aList;
    }

    private static List<N.TopoDS_Shape> Faces(N.TopoDS_Shape theShape)
    {
        using var aMap = new N.TopTools_IndexedMapOfShape();
        N.TopExp.MapShapes(theShape, N.TopAbs_ShapeEnum.TopAbs_FACE, aMap);
        var aList = new List<N.TopoDS_Shape>();
        for (int i = 1; i <= aMap.Extent(); i++)
        {
            aList.Add(aMap.FindKey(i));
        }
        return aList;
    }

    internal static IEnumerable<(string Name, Shape Shape)> Bases()
    {
        yield return ("box", new BRepPrimAPI_MakeBox(10, 20, 30).Shape());
        yield return ("cylinder", new BRepPrimAPI_MakeCylinder(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 8, 20).Shape());
        Shape aBox = new BRepPrimAPI_MakeBox(new Point3D(-10, -10, 0), 20, 20, 12).Shape();
        Shape aHole = new BRepPrimAPI_MakeCylinder(new Frame3D(new Point3D(6, 6, -1), new Direction3D(0, 0, 1)), 2, 14).Shape();
        yield return ("box with a hole", new BRepAlgoAPI_Cut(aBox, aHole).Shape());
        var aHexagon = new BRepBuilderAPI_MakePolygon();
        for (int i = 0; i < 6; i++)
        {
            aHexagon.Add(new Point3D(12 * Math.Cos(i * Math.PI / 3), 12 * Math.Sin(i * Math.PI / 3), 0));
        }
        aHexagon.Close();
        Face aHexFace = new BRepBuilderAPI_MakeFace(aHexagon.Wire(), true).Face();
        yield return ("hexagonal prism", new BRepPrimAPI_MakePrism(aHexFace, new Vector3D(0, 0, 15)).Shape());
    }

    // A rectangle or a circle around the centre of a planar face, in its plane, of a quarter of its size
    internal static Face? Profile(in Face theFace, bool theCircle)
    {
        var anAdaptor = new FaceSurfaceAdaptor(theFace);
        if (anAdaptor.GetType() != SurfaceKind.Plane)
        {
            return null;
        }
        Plane3D aPln = anAdaptor.Plane();
        var aProps = new Properties();
        BRepGProp.SurfaceProperties(theFace, aProps);
        Point3D aC = aProps.CentreOfMass();
        double aSize = Math.Sqrt(aProps.Mass()) / 6;
        Direction3D aN = aPln.Axis().Direction(), aX = aPln.XAxis().Direction();
        if (theFace.Orientation() == Orientation.Reversed)
        {
            aN = aN.Reversed();
        }
        var aPlane = new Plane3D(new CoordinateSystem(aC, aN, aX));
        if (theCircle)
        {
            Edge anEdge = new BRepBuilderAPI_MakeEdge(new Circle3D(new Frame3D(aC, aN, aX), aSize)).Edge();
            return new BRepBuilderAPI_MakeFace(aPlane, new BRepBuilderAPI_MakeWire(anEdge).Wire(), true).Face();
        }
        Direction3D aY = aN.Crossed(aX);
        Point3D P(double u, double v) => aC.Translated(new Vector3D(aX) * u + new Vector3D(aY) * v);
        var aPoly = new BRepBuilderAPI_MakePolygon(P(-aSize, -aSize / 2), P(aSize, -aSize / 2), P(aSize, aSize / 2), P(-aSize, aSize / 2), true);
        return new BRepBuilderAPI_MakeFace(aPlane, aPoly.Wire(), true).Face();
    }

    internal enum Mode { Depth, ThruAll, Until }

    private static string Histories(Func<Shape, bool> theDeleted, Func<Shape, int> theModified,
                                    Func<Shape, int> theGenerated, List<Shape> theFaces, List<Shape> theEdges)
        => string.Join(" ", theFaces.Select(f => theDeleted(f) ? "d" : theModified(f).ToString()))
           + " | " + string.Join(" ", theEdges.Select(e => theGenerated(e).ToString()));

    // The rounding of decision 1017's canonical record
    private static string Q(double theValue) =>
        ((long)Math.Round(theValue * 1e6, MidpointRounding.AwayFromZero)).ToString(System.Globalization.CultureInfo.InvariantCulture) + ";";

    // Decision 1017's canonical record of a DPrism result, whose BRep text natively varies run to run: the sorted faces'
    // areas and edge counts, edges' tolerances and ends, vertices' points and tolerances, rounded to 1e-6; and the volume
    internal static string Coarse(in Shape theShape)
    {
        var aLines = new List<string>();
        foreach (Shape aF in CanonicalShapeComparer.SubShapes(theShape, ShapeType.Face))
        {
            var aProps = new Properties();
            BRepGProp.SurfaceProperties(aF, aProps);
            aLines.Add($"F {Q(aProps.Mass())} {CanonicalShapeComparer.SubShapes(aF, ShapeType.Edge).Count()}");
        }
        foreach (Shape anE in CanonicalShapeComparer.SubShapes(theShape, ShapeType.Edge))
        {
            Exploration.Vertices(ShapeCasts.Edge(anE), out Vertex aV1, out Vertex aV2);
            string[] anEnds = [aV1.IsNull ? "" : P(Tool.Pnt(aV1)), aV2.IsNull ? "" : P(Tool.Pnt(aV2))];
            Array.Sort(anEnds, StringComparer.Ordinal);
            aLines.Add($"E {Q(Tool.Tolerance(ShapeCasts.Edge(anE)))} {anEnds[0]} {anEnds[1]}");
        }
        foreach (Shape aV in CanonicalShapeComparer.SubShapes(theShape, ShapeType.Vertex))
        {
            aLines.Add($"V {P(Tool.Pnt(ShapeCasts.Vertex(aV)))} {Q(Tool.Tolerance(ShapeCasts.Vertex(aV)))}");
        }
        aLines.Sort(StringComparer.Ordinal);
        var aVolume = new Properties();
        BRepGProp.VolumeProperties(theShape, aVolume);
        aLines.Add($"volume {Q(aVolume.Mass())} valid {new BRepCheck_Analyzer(theShape).IsValid()}");
        return string.Join("\n", aLines);

        static string P(Point3D p) => Q(p.X()) + Q(p.Y()) + Q(p.Z());
    }

    private static void CheckDPrism(ShapeFixOracleRun theRun, string theDir, ref int theNumber, string theWhat,
                                    N.TopoDS_Shape? theNRes, string? theNErr, Shape? thePRes, string? thePErr,
                                    N.TopoDS_Shape theNIn, Shape thePIn, string thePInText, ref int theCoarse)
    {
        int aDifferences = theRun.Differences.Count;
        theRun.Check(theWhat, theNRes, theNErr, "", thePRes, thePErr, "", theNIn, thePIn, thePInText);
        if (theRun.Differences.Count == aDifferences || theNRes is null || thePRes is null
            || !theRun.Differences[^1].StartsWith(theWhat + ": BRep texts", StringComparison.Ordinal))
        {
            return;
        }
        // native DPrism's text varies run to run (decision 1017): the coarse record decides
        string aFile = Path.Combine(theDir, $"res{theNumber++}.brep");
        File.WriteAllText(aFile, theRun.NativeText(theNRes));
        Assert.True(BrepTools.Read(out Shape aNP, aFile));
        string aN = Coarse(aNP), aP = Coarse(thePRes.Value);
        if (aN == aP)
        {
            theRun.Differences.RemoveAt(theRun.Differences.Count - 1);
            theCoarse++;
        }
        else
        {
            string[] aNL = aN.Split('\n'), aPL = aP.Split('\n');
            theRun.Differences[^1] = $"{theWhat}: coarse records differ: native only {string.Join(" | ", aNL.Except(aPL).Take(4))}; port only {string.Join(" | ", aPL.Except(aNL).Take(4))}";
        }
    }

    [Fact]
    public void DPrism_as_build123d_calls_it_as_native()
    {
        var aWatch = Stopwatch.StartNew();
        using var aRun = new ShapeFixOracleRun(output, "feat");
        string aDir = Path.Combine(Path.GetTempPath(), "occtnet-feat-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(aDir);
        int aNumber = 0, aHistories = 0, aStatuses = 0, aCoarse = 0;
        try
        {
            foreach ((string aName, Shape aBase) in Bases())
            {
                List<Shape> aBaseFaces = Faces(aBase);
                for (int iF = 0; iF < aBaseFaces.Count && iF < 6; iF++)
                {
                    foreach (bool isCircle in new[] { false, true })
                    {
                        Face? aProfile = Profile(ShapeCasts.Face(aBaseFaces[iF]), isCircle);
                        if (aProfile is null)
                        {
                            continue;
                        }
                        // the base and the profile in one compound, read by both sides
                        var aComp = new Compound();
                        var aB = new BrepBuilder();
                        aB.MakeCompound(ref aComp);
                        aB.Add(aComp, aBase);
                        aB.Add(aComp, aProfile.Value);
                        (N.TopoDS_Shape aNIn, Shape aPIn) = Both(aDir, ref aNumber, aComp, out string anInFile);
                        string aPInText = aRun.PortText(aPIn);
                        var aPIt = new Iterator(aPIn);
                        Shape aPBase = aPIt.Value();
                        aPIt.Next();
                        Face aPProf = ShapeCasts.Face(aPIt.Value());
                        N.TopoDS_Shape aNBase = new();
                        N.TopoDS_Face aNProf = new();
                        List<N.TopoDS_Shape> aNFaces = [];
                        var aNEdges = new List<N.TopoDS_Shape>();
                        // Native writes into its input (the profile's vertex tolerances, raised by a tapered fuse):
                        // each native run reads its input again, as the port's is the same frozen shape every time
                        // (decision 1105)
                        void ReadNative(bool theAgain)
                        {
                            if (theAgain)
                            {
                                aNIn = ShapeFixOracleRun.NativeRead(anInFile)!;
                            }
                            using var aNIt = new N.TopoDS_Iterator(aNIn);
                            aNBase = aNIt.Value();
                            aNIt.Next();
                            aNProf = N.TopoDS.Face(aNIt.Value());
                            aNFaces = Faces(aNBase);
                            aNEdges.Clear();
                            using var anExp = new N.TopExp_Explorer(aNProf, N.TopAbs_ShapeEnum.TopAbs_EDGE);
                            for (; anExp.More(); anExp.Next())
                            {
                                aNEdges.Add(anExp.Current());
                            }
                        }
                        ReadNative(false);
                        List<Shape> aPFaces = Faces(aPBase);
                        var aPEdges = new List<Shape>();
                        for (var anExp = new Explorer(aPProf, ShapeType.Edge); anExp.More(); anExp.Next())
                        {
                            aPEdges.Add(anExp.Current());
                        }
                        bool isFirstRun = true;
                        // the face opposite (for Until): the last planar face but this one
                        int iUntil = Enumerable.Range(0, aPFaces.Count).Reverse()
                                               .FirstOrDefault(k => k != iF && Profile(ShapeCasts.Face(aPFaces[k]), false) is not null, -1);

                        foreach (Mode aMode in Enum.GetValues<Mode>())
                        {
                            foreach (int aFuse in new[] { 0, 1 })
                            {
                                foreach (double aTaper in new[] { 0.0, 5.0 })
                                {
                                    if (aMode == Mode.Until && iUntil < 0)
                                    {
                                        continue;
                                    }
                                    ReadNative(!isFirstRun);
                                    isFirstRun = false;
                                    bool isSketch = (iF + aFuse) % 2 == 0;
                                    double aDepth = aFuse == 1 ? 4.0 : -4.0;
                                    string aWhat = $"{aName} face {iF} {(isCircle ? "circle" : "rectangle")} {aMode} fuse {aFuse} taper {aTaper}{(isSketch ? " on its face" : "")}";

                                    N.TopoDS_Shape? aNRes = null;
                                    string? aNErr = null;
                                    string aNStatus = "", aNHist = "";
                                    try
                                    {
                                        using var aNoFace = new N.TopoDS_Face();
                                        using var aN = new N.BRepFeat_MakeDPrism(aNBase, aNProf, isSketch ? N.TopoDS.Face(aNFaces[iF]) : aNoFace,
                                                                                aTaper * Math.PI / 180, aFuse, false);
                                        switch (aMode)
                                        {
                                            case Mode.Depth: aN.Perform(aDepth); break;
                                            case Mode.ThruAll: aN.PerformThruAll(); break;
                                            default: aN.Perform(aNFaces[iUntil]); break;
                                        }
                                        aNStatus = aN.CurrentStatusError().ToString();
                                        if (aN.IsDone())
                                        {
                                            aNRes = aN.Shape();
                                            aNHist = string.Join(" ", aNFaces.Select(f => aN.IsDeleted(f) ? "d" : aN.Modified(f).Extent().ToString()))
                                                     + " | " + string.Join(" ", aNEdges.Select(e => aN.Generated(e).Extent().ToString()));
                                        }
                                    }
                                    catch (Exception e) when (ErrorOf(e) is not null)
                                    {
                                        aNErr = ErrorOf(e);
                                    }

                                    Shape? aPRes = null;
                                    string? aPErr = null;
                                    string aPStatus = "", aPHist = "";
                                    try
                                    {
                                        var aP = new BRepFeat_MakeDPrism(aPBase, aPProf, isSketch ? ShapeCasts.Face(aPFaces[iF]) : new Face(),
                                                                         aTaper * Math.PI / 180, aFuse, false);
                                        switch (aMode)
                                        {
                                            case Mode.Depth: aP.Perform(aDepth); break;
                                            case Mode.ThruAll: aP.PerformThruAll(); break;
                                            default: aP.Perform(aPFaces[iUntil]); break;
                                        }
                                        aPStatus = aP.CurrentStatusError().ToString();
                                        if (aP.IsDone())
                                        {
                                            aPRes = aP.Shape();
                                            aPHist = Histories(f => aP.IsDeleted(f), f => aP.Modified(f).Count, e => aP.Generated(e).Count, aPFaces, aPEdges);
                                        }
                                    }
                                    catch (Exception e) when (ErrorOf(e) is not null)
                                    {
                                        aPErr = ErrorOf(e);
                                    }

                                    CheckDPrism(aRun, aDir, ref aNumber, aWhat, aNRes, aNErr, aPRes, aPErr, aNIn, aPIn, aPInText, ref aCoarse);
                                    if (aNStatus != aPStatus)
                                    {
                                        aRun.Differences.Add($"{aWhat}: status {aNStatus} natively, {aPStatus} in the port");
                                    }
                                    else
                                    {
                                        aStatuses++;
                                    }
                                    if (aNHist != aPHist)
                                    {
                                        aRun.Differences.Add($"{aWhat}: history {aNHist} natively, {aPHist} in the port");
                                    }
                                    else
                                    {
                                        aHistories++;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        finally
        {
            Directory.Delete(aDir, true);
        }
        output.WriteLine($"statuses equal {aStatuses}, histories equal {aHistories}, results equal only by the coarse record {aCoarse}");
        aRun.Report("BRepFeat_MakeDPrism as build123d.NET's DPrism", aWatch.Elapsed);
        // Attributed (decisions 1057, 1105): rectangles fused by a depth with a taper. Native's LocOpe_DPrism trims the
        // tapered walls' pcurves at 1.0640513e-5 where the port's start at 0, and raises the four profile vertices to
        // 1.06e-5 in place (BRepLib::UpdateTolerances); native's result written as text then fails BRepCheck, natively
        // as in the port (in memory it passes). The same geometry otherwise; the trim's cause is open. The six cases cut
        // through all that differed by those vertices' tolerance were native's input written by the earlier tapered fuse:
        // each native run now reads its input again.
        int anInvalidNative = aRun.Differences.Count(d => d.Contains("valid False; port only volume", StringComparison.Ordinal)
                                                          && d.Contains("fuse 1 taper 5", StringComparison.Ordinal));
        // At most 11: native's DPrism varies with addresses (decision 1017), and under a loaded merge gate one of them
        // once came out as the port's (1 November 2026); every difference must still be of this attributed kind
        Assert.InRange(anInvalidNative, 0, 11);
        Assert.Equal(anInvalidNative, aRun.Differences.Count);
    }

    [Fact]
    public void SplitShape_as_SplitByPerimeter_as_native()
    {
        var aWatch = Stopwatch.StartNew();
        using var aRun = new ShapeFixOracleRun(output, "split");
        string aDir = Path.Combine(Path.GetTempPath(), "occtnet-split-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(aDir);
        int aNumber = 0, aSides = 0;
        try
        {
            foreach ((string aName, Shape aBase) in Bases())
            {
                List<Shape> aBaseFaces = Faces(aBase);
                for (int iF = 0; iF < aBaseFaces.Count; iF++)
                {
                    foreach (bool isCircle in new[] { false, true })
                    {
                        Face? aProfile = Profile(ShapeCasts.Face(aBaseFaces[iF]), isCircle);
                        if (aProfile is null)
                        {
                            continue;
                        }
                        // SplitByPerimeter splits a face (here the base's face) by the perimeter's edges
                        var aComp = new Compound();
                        var aB = new BrepBuilder();
                        aB.MakeCompound(ref aComp);
                        aB.Add(aComp, aBaseFaces[iF]);
                        aB.Add(aComp, BrepTools.OuterWire(aProfile.Value));
                        (N.TopoDS_Shape aNIn, Shape aPIn) = Both(aDir, ref aNumber, aComp);
                        string aPInText = aRun.PortText(aPIn);
                        var aPIt = new Iterator(aPIn);
                        Shape aPFace = aPIt.Value();
                        aPIt.Next();
                        Shape aPWire = aPIt.Value();
                        using var aNIt = new N.TopoDS_Iterator(aNIn);
                        N.TopoDS_Shape aNFace = aNIt.Value();
                        aNIt.Next();
                        N.TopoDS_Shape aNWire = aNIt.Value();
                        string aWhat = $"{aName} face {iF} {(isCircle ? "circle" : "rectangle")}";

                        N.TopoDS_Shape? aNRes = null;
                        string? aNErr = null, aNSides = "";
                        try
                        {
                            using var aN = new N.BRepFeat_SplitShape(aNFace);
                            using var anEdges = new N.TopTools_SequenceOfShape();
                            using (var anExp = new N.TopExp_Explorer(aNWire, N.TopAbs_ShapeEnum.TopAbs_EDGE))
                            {
                                for (; anExp.More(); anExp.Next())
                                {
                                    anEdges.Append(anExp.Current());
                                }
                            }
                            aN.Add(anEdges);
                            aN.Build();
                            if (aN.IsDone())
                            {
                                aNRes = aN.Shape();
                                using var aL = aN.Left();
                                using var aR = aN.Right();
                                aNSides = $"left {aL.Extent()} right {aR.Extent()} modified {aN.Modified(aNFace).Extent()}";
                            }
                        }
                        catch (Exception e) when (ErrorOf(e) is not null)
                        {
                            aNErr = ErrorOf(e);
                        }

                        Shape? aPRes = null;
                        string? aPErr = null, aPSides = "";
                        try
                        {
                            var aP = new BRepFeat_SplitShape(aPFace);
                            var anEdges = new NCollection_Sequence<Shape>();
                            for (var anExp = new Explorer(aPWire, ShapeType.Edge); anExp.More(); anExp.Next())
                            {
                                anEdges.Append(anExp.Current());
                            }
                            aP.Add(anEdges);
                            aP.Build();
                            if (aP.IsDone())
                            {
                                aPRes = aP.Shape();
                                aPSides = $"left {aP.Left().Count} right {aP.Right().Count} modified {aP.Modified(aPFace).Count}";
                            }
                        }
                        catch (Exception e) when (ErrorOf(e) is not null)
                        {
                            aPErr = ErrorOf(e);
                        }

                        aRun.Check(aWhat, aNRes, aNErr, "", aPRes, aPErr, "", aNIn, aPIn, aPInText);
                        if (aNSides != aPSides)
                        {
                            aRun.Differences.Add($"{aWhat}: {aNSides} natively, {aPSides} in the port");
                        }
                        else
                        {
                            aSides++;
                        }
                    }
                }
            }
        }
        finally
        {
            Directory.Delete(aDir, true);
        }
        output.WriteLine($"left, right and modified equal {aSides}");
        aRun.Report("BRepFeat_SplitShape as build123d.NET's SplitByPerimeter", aWatch.Elapsed);
        Assert.Empty(aRun.Differences);
    }
}
