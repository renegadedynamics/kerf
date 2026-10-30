#if HAS_ORACLE
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Diagnostics;
using System.Reflection;
using System.Text.RegularExpressions;
using OcctNet.TestHarness;
using Xunit.Abstractions;
using N = Build123d.Occt;
using static Occt.ShapeExtend_Status;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// ShapeFix_Wire, ShapeFix_Edge and ShapeFix's SameParameter against native OCCT, through native
/// <c>ShapeFix_Shape</c> on a wire or an edge (build123d.NET's <c>MakeFace</c> fixes its wires this way).
/// </summary>
/// <remarks>
/// <para>
/// ShapeFix_Shape is unit 6H3's. On a wire or an edge it only reaches this unit's classes, so the port's side is a
/// transcription of its wire and edge branches (<see cref="PortFixShape"/>): a context considering locations, then on a
/// wire ShapeFix_Wire with topology changes allowed and the closed mode off for an open wire, no face, the context set
/// after the load, the wire replaced by the fixed one; on an edge ShapeFix_Edge's FixVertexTolerance; then the context
/// applied, ShapeFix::SameParameter (not enforced) and the context applied again. It runs two ways on the frozen input:
/// with a copy at the entry of the whole shape (as ShapeFix_Shape will, decision 583), and with ShapeFix_Wire's own copy
/// at the entry and the SameParameter twin.
/// </para>
/// <para>
/// Inputs: the wires and edges of the heal group's data files and data/occ (up to 1 MB, a sample per shape), and
/// random wires as build123d.NET makes faces from: lines, arcs and B-splines, with gaps between edges from 0 to 10^-2,
/// tiny edges, shuffled and reversed edges, open and closed, at the default precision and at 10^-3. Native OCCT writes
/// each input to a file that both sides read. Results are compared as BRep text with the Free column masked (frozen
/// sub-shapes keep the Free flag they were frozen with, open question 1), then canonically where the texts differ.
/// </para>
/// </remarks>
public class ShapeFixWireOracleTests(ITestOutputHelper output) : IDisposable
{
    private readonly string myDirectory = Directory.CreateTempSubdirectory("occtnet-sfw-").FullName;
    private int myFileNumber;
    private readonly List<string> myDifferences = [];
    private int myCompared, myIdentical, myFreeMasked, myCanonical, myExceptions, myUnchanged, myFixed, myChanged;

    public void Dispose()
    {
        Directory.Delete(myDirectory, true);
        GC.SuppressFinalize(this);
    }

    private static string Metadata(string theKey)
        => typeof(ShapeFixWireOracleTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>().Single(a => a.Key == theKey).Value!;

    private string NewFile() => Path.Combine(myDirectory, $"s{myFileNumber++}.brep");

    private string NativeText(N.TopoDS_Shape theShape)
    {
        string aFile = NewFile();
        Assert.True(N.BRepTools.Write(theShape, aFile, true, false, N.TopTools_FormatVersion.TopTools_FormatVersion_VERSION_3));
        string aText = File.ReadAllText(aFile);
        File.Delete(aFile);
        return aText;
    }

    private string PortText(in TopoDS_Shape theShape)
    {
        string aFile = NewFile();
        Assert.True(BRepTools.Write(theShape, aFile, true, false, TopTools_FormatVersion.TopTools_FormatVersion_VERSION_3));
        string aText = File.ReadAllText(aFile);
        File.Delete(aFile);
        return aText;
    }

    /// <summary>Native writes the shape; both read it. Null if native OCCT can't write or read it.</summary>
    private (N.TopoDS_Shape N, TopoDS_Shape P)? Read(N.TopoDS_Shape theShape)
    {
        string aFile = NewFile();
        if (!N.BRepTools.Write(theShape, aFile, true, false, N.TopTools_FormatVersion.TopTools_FormatVersion_VERSION_3))
        {
            return null;
        }
        var aNShape = new N.TopoDS_Shape();
        using (var aBuilder = new N.BRep_Builder())
        {
            if (!N.BRepTools.Read(aNShape, aFile, aBuilder))
            {
                return null;
            }
        }
        Assert.True(BRepTools.Read(out TopoDS_Shape aPShape, aFile), aFile);
        File.Delete(aFile);
        if (!aPShape.IsNull())
        {
            aPShape.TShape()!.FreezeGraph(); // a caller's shape
        }
        return (aNShape, aPShape);
    }

    // ---- the port's ShapeFix_Shape on a wire or an edge (ShapeFix_Shape.cxx, Init and Perform) ----

    private static TopoDS_Shape PortFixShape(in TopoDS_Shape theShape, double thePrecision, double theMaxTol, bool theEntryOfShape,
                                             out bool theStatus, out bool theDone1, out bool theDone2)
    {
        var aContext = new ShapeBuild_ReShape();
        aContext.ModeConsiderLocation(true);
        BRepTools_CopyAtEntry? anEntry = theEntryOfShape ? new BRepTools_CopyAtEntry(theShape, aContext) : null;
        TopoDS_Shape myShape = anEntry is null ? theShape : anEntry.Shape();

        // ShapeFix_Shape's tools as its constructor makes them, the precision passed down as SetPrecision does
        var sfw = new ShapeFix_Wire();
        sfw.SetPrecision(thePrecision);
        sfw.SetMaxTolerance(theMaxTol);
        ShapeFix_Edge sfe = sfw.FixEdgeTool();

        theStatus = theDone1 = theDone2 = false;
        TopoDS_Shape S = aContext.InternalApply(myShape);
        switch (S.ShapeType())
        {
            case TopAbs_WIRE:
            {
                sfw.ModifyTopologyMode() = true;
                if (!S.Closed())
                {
                    sfw.ClosedWireMode() = false;
                }
                sfw.SetFace(new TopoDS_Face());
                sfw.Load(TopoDS.Wire(S));
                sfw.SetContext(aContext);
                if (sfw.Perform())
                {
                    theStatus = true;
                    aContext.Replace(S, anEntry is null ? sfw.Wire() : sfw.InternalWire());
                }
                theDone2 = true;
                break;
            }
            case TopAbs_EDGE:
            {
                sfe.SetContext(aContext);
                if (sfe.FixVertexTolerance(TopoDS.Edge(S)))
                {
                    theDone1 = true;
                }
                break;
            }
        }

        TopoDS_Shape myResult = aContext.InternalApply(S);
        if (anEntry is not null)
        {
            ShapeFix.SameParameter(myResult, false, 0.0);
            myResult = aContext.InternalApply(myResult);
            myResult = anEntry.Finish(myResult);
        }
        else
        {
            myResult = ShapeFix.SameParametered(myResult, false, out _);
            myResult = aContext.InternalApply(myResult);
        }
        if (!myResult.IsNull())
        {
            myResult.TShape()!.FreezeGraph();
        }
        return myResult;
    }

    private static string? ErrorOf(Exception theException) => theException switch
    {
        N.OcctException e => e.Message.IndexOf(':') is int c and > 0 ? e.Message[..c] : e.Message,
        Standard_Failure f => f.GetType().Name,
        _ => null,
    };

    private void Compare(N.TopoDS_Shape theShape, string theWhat, double thePrecision = 1.0e-7, double theMaxTol = 1.0e-7)
    {
        if (Read(theShape) is not (N.TopoDS_Shape aNS, TopoDS_Shape aPS))
        {
            return;
        }
        string anInputText = PortText(aPS); // the frozen input, which nothing may change

        N.TopoDS_Shape? aNRes = null;
        bool aNStatus = false, aND1 = false, aND2 = false;
        string? aNErr = null;
        try
        {
            using var aFix = new N.ShapeFix_Shape(aNS);
            if (thePrecision != 1.0e-7 || theMaxTol != 1.0e-7)
            {
                aFix.SetPrecision(thePrecision);
                aFix.SetMaxTolerance(theMaxTol);
            }
            aNStatus = aFix.Perform();
            aNRes = aFix.Shape();
            myFixed += aNStatus ? 1 : 0;
            myChanged += NativeText(aNRes) != NativeText(aNS) ? 1 : 0;
            aND1 = aFix.Status(N.ShapeExtend_Status.ShapeExtend_DONE1);
            aND2 = aFix.Status(N.ShapeExtend_Status.ShapeExtend_DONE2);
        }
        catch (N.OcctException e)
        {
            aNErr = ErrorOf(e);
        }

        foreach (bool anEntryOfShape in new[] { true, false })
        {
            string w = $"{theWhat} ({(anEntryOfShape ? "copy of the shape" : "ShapeFix_Wire's copy")})";
            TopoDS_Shape aPRes = new();
            bool aPStatus = false, aPD1 = false, aPD2 = false;
            string? aPErr = null;
            try
            {
                aPRes = PortFixShape(aPS, thePrecision, theMaxTol, anEntryOfShape, out aPStatus, out aPD1, out aPD2);
            }
            catch (Standard_Failure e)
            {
                aPErr = ErrorOf(e);
            }

            myCompared++;
            if (aNErr is not null || aPErr is not null)
            {
                if (aNErr == aPErr)
                {
                    myIdentical++;
                    myExceptions++;
                }
                else
                {
                    myDifferences.Add($"{w}: {aNErr ?? "no exception"} natively, {aPErr ?? "no exception"} in the port");
                }
                continue;
            }

            if (aNStatus != aPStatus || aND1 != aPD1 || aND2 != aPD2)
            {
                myDifferences.Add($"{w}: status {aNStatus}/{aND1}/{aND2} natively, {aPStatus}/{aPD1}/{aPD2} in the port");
            }
            string aNT = NativeText(aNRes!), aPT = PortText(aPRes);
            BRepTextDiff aDiff = BRepTextComparison.Compare(aNT, aPT);
            if (aDiff.IsIdentical)
            {
                myIdentical++;
            }
            else if (aDiff.IsIdenticalMaskingFree)
            {
                myIdentical++;
                myFreeMasked++;
            }
            else
            {
                // both read from the native result's text: the canonical comparison of decision 581
                string aFile = NewFile();
                File.WriteAllText(aFile, aNT);
                Assert.True(BRepTools.Read(out TopoDS_Shape aNP, aFile));
                File.Delete(aFile);
                CanonicalComparison aCanon = CanonicalShapeComparer.Compare(aNP, aPRes, new CanonicalOptions { CheckValidity = false });
                if (aCanon.IsIdentical)
                {
                    myCanonical++;
                }
                else
                {
                    myDifferences.Add($"{w}: {aDiff}; canonically {aCanon}");
                }
            }

            if (PortText(aPS) == anInputText)
            {
                myUnchanged++;
            }
            else
            {
                myDifferences.Add($"{w}: the input changed");
            }
        }
    }

    // ---- inputs ----

    private static N.TopoDS_Shape? NativeRead(string theFile)
    {
        var aShape = new N.TopoDS_Shape();
        using var aBuilder = new N.BRep_Builder();
        try
        {
            return N.BRepTools.Read(aShape, theFile, aBuilder) && !aShape.IsNull() ? aShape : null;
        }
        catch (Exception)
        {
            return null;
        }
    }

    // Up to theMax sub-shapes of the type, evenly sampled, distinct by TShape (as an indexed map gives them).
    private static List<N.TopoDS_Shape> Sample(N.TopoDS_Shape theShape, N.TopAbs_ShapeEnum theType, int theMax)
    {
        using var aMap = new N.TopTools_IndexedMapOfShape();
        N.TopExp.MapShapes(theShape, theType, aMap);
        int aNb = aMap.Extent();
        var aList = new List<N.TopoDS_Shape>();
        int aStep = Math.Max(1, aNb / theMax);
        for (int i = 1; i <= aNb && aList.Count < theMax; i += aStep)
        {
            aList.Add(aMap.FindKey(i));
        }
        return aList;
    }

    private void RunShape(N.TopoDS_Shape theShape, string theWhat, int theMaxWires, int theMaxEdges)
    {
        int i = 0;
        foreach (N.TopoDS_Shape aWire in Sample(theShape, N.TopAbs_ShapeEnum.TopAbs_WIRE, theMaxWires))
        {
            Compare(aWire, $"{theWhat}, wire {i++}");
        }
        i = 0;
        foreach (N.TopoDS_Shape anEdge in Sample(theShape, N.TopAbs_ShapeEnum.TopAbs_EDGE, theMaxEdges))
        {
            Compare(anEdge, $"{theWhat}, edge {i++}");
        }
    }

    private static N.gp_Pnt P(in gp_Pnt theP) => new(theP.X(), theP.Y(), theP.Z());

    // A random wire as a sketch makes it: lines, arcs and B-splines around a loop in a random plane, each edge with
    // its own vertices, the ends moved apart by a gap; sometimes with tiny edges, shuffled, reversed or open.
    private static N.TopoDS_Shape RandomWire(Oracle theOracle, int theIndex, out string theWhat)
    {
        int aNb = theOracle.Int(3, 8);
        double[] aGaps = [0.0, 0.0, 1.0e-8, 1.0e-6, 1.0e-5, 1.0e-4, 1.0e-3, 1.0e-2];
        double aGap = aGaps[theIndex % aGaps.Length];
        gp_Ax2 anAx = theOracle.Ax2();
        double aRadius = theOracle.Real(1.0, 50.0);
        var aPoints = new gp_Pnt[aNb];
        for (int i = 0; i < aNb; i++)
        {
            double anAngle = 2.0 * Math.PI * (i + 0.3 * theOracle.Real(-1.0, 1.0)) / aNb;
            double r = aRadius * theOracle.Real(0.6, 1.4);
            gp_Pnt aP = anAx.Location();
            aP = aP.Translated(new gp_Vec(anAx.XDirection()) * (r * Math.Cos(anAngle)));
            aP = aP.Translated(new gp_Vec(anAx.YDirection()) * (r * Math.Sin(anAngle)));
            aPoints[i] = aP;
        }
        bool isOpen = theOracle.Int(0, 4) == 0;
        bool isTiny = theOracle.Int(0, 3) == 0;
        bool isShuffled = theOracle.Int(0, 3) == 0;
        bool isReversed = theOracle.Int(0, 3) == 0;
        bool isPolygon = aGap == 0.0 && theOracle.Bool();
        theWhat = $"{aNb} edges, gap {aGap}{(isOpen ? ", open" : "")}{(isTiny ? ", tiny edge" : "")}{(isShuffled ? ", shuffled" : "")}"
                  + $"{(isReversed ? ", reversed" : "")}{(isPolygon ? ", polygon" : "")}";

        var anEdges = new List<N.TopoDS_Shape>();
        if (isPolygon)
        {
            using var aPoly = new N.BRepBuilderAPI_MakePolygon();
            foreach (gp_Pnt aP in aPoints)
            {
                aPoly.Add(P(aP));
            }
            if (!isOpen)
            {
                aPoly.Close();
            }
            for (var anExp = new N.TopExp_Explorer(aPoly.Wire(), N.TopAbs_ShapeEnum.TopAbs_EDGE); anExp.More(); anExp.Next())
            {
                anEdges.Add(anExp.Current());
            }
        }
        else
        {
            int aNbEdges = isOpen ? aNb - 1 : aNb;
            gp_Pnt aPrevEnd = aPoints[0];
            for (int i = 0; i < aNbEdges; i++)
            {
                gp_Pnt aStart = aPrevEnd;
                gp_Pnt anEnd = aPoints[(i + 1) % aNb];
                if (aGap > 0.0)
                {
                    anEnd = anEnd.Translated(new gp_Vec(theOracle.Dir()) * aGap);
                }
                if (i == aNbEdges - 1 && !isOpen)
                {
                    anEnd = aPoints[0].Translated(new gp_Vec(theOracle.Dir()) * aGap);
                }
                N.TopoDS_Edge? anEdge = null;
                int aKind = theOracle.Int(0, 9);
                gp_Pnt aMid = new gp_Pnt((aStart.XYZ() + anEnd.XYZ()) * 0.5);
                gp_Vec aSide = new gp_Vec(anAx.Direction()).Crossed(new gp_Vec(aStart, anEnd));
                if (aKind >= 7 && aSide.Magnitude() > 1.0e-6)
                {
                    gp_Pnt aBulge = aMid.Translated(aSide.Normalized() * (aStart.Distance(anEnd) * theOracle.Real(-0.3, 0.3)));
                    try
                    {
                        if (aKind == 9)
                        {
                            using var aPnts = new N.TColgp_HArray1OfPnt(1, 3);
                            aPnts.SetValue(1, P(aStart));
                            aPnts.SetValue(2, P(aBulge));
                            aPnts.SetValue(3, P(anEnd));
                            using var anInterp = new N.GeomAPI_Interpolate(aPnts, false, 1.0e-9);
                            anInterp.Perform();
                            if (anInterp.IsDone())
                            {
                                anEdge = new N.BRepBuilderAPI_MakeEdge(anInterp.Curve()).Edge();
                            }
                        }
                        else
                        {
                            using var anArc = new N.GC_MakeArcOfCircle(P(aStart), P(aBulge), P(anEnd));
                            anEdge = new N.BRepBuilderAPI_MakeEdge(anArc.Value()).Edge();
                        }
                    }
                    catch (N.OcctException)
                    {
                        anEdge = null;
                    }
                }
                anEdge ??= new N.BRepBuilderAPI_MakeEdge(P(aStart), P(anEnd)).Edge();
                anEdges.Add(anEdge);
                if (isTiny && i == 0)
                {
                    gp_Pnt aTinyEnd = anEnd.Translated(new gp_Vec(theOracle.Dir()) * theOracle.Real(1.0e-8, 1.0e-4));
                    anEdges.Add(new N.BRepBuilderAPI_MakeEdge(P(anEnd), P(aTinyEnd)).Edge());
                    anEnd = aTinyEnd;
                }
                aPrevEnd = anEnd;
            }
        }

        if (isShuffled)
        {
            for (int i = anEdges.Count - 1; i > 0; i--)
            {
                int j = theOracle.Int(0, i);
                (anEdges[i], anEdges[j]) = (anEdges[j], anEdges[i]);
            }
        }
        var aWire = new N.TopoDS_Wire();
        using var aB = new N.BRep_Builder();
        aB.MakeWire(aWire);
        for (int i = 0; i < anEdges.Count; i++)
        {
            N.TopoDS_Shape anEdge = anEdges[i];
            aB.Add(aWire, isReversed && i % 2 == 1 ? anEdge.Reversed() : anEdge);
        }
        aWire.Closed(!isOpen);
        return aWire;
    }

    private void Report(string theWhat, TimeSpan theTime)
    {
        output.WriteLine($"{theWhat}: {myIdentical}/{myCompared} results identical as BRep text ({myFreeMasked} with the Free column masked, "
                         + $"{myExceptions} the same exception on both sides), {myCanonical} canonically identical, {myDifferences.Count} different; "
                         + $"inputs unchanged {myUnchanged}; native fixes reported {myFixed}, native results differing from their input {myChanged}; {theTime.TotalSeconds:F0} s");
        foreach (string aD in myDifferences.Take(60))
        {
            output.WriteLine("  " + aD);
        }
    }

    [Fact]
    public void ShapeFix_Shape_on_random_wires_matches_native_OCCT()
    {
        var aWatch = Stopwatch.StartNew();
        var anOracle = new Oracle(output, 62201);
        for (int i = 0; i < 400; i++)
        {
            N.TopoDS_Shape aWire = RandomWire(anOracle, i, out string aWhat);
            Compare(aWire, $"random wire {i} ({aWhat})");
            if (i % 4 == 0)
            {
                Compare(aWire, $"random wire {i} ({aWhat}), precision 1e-3", 1.0e-3, 1.0e-2);
            }
            if (i % 5 == 0)
            {
                var anExp = new N.TopExp_Explorer(aWire, N.TopAbs_ShapeEnum.TopAbs_EDGE);
                if (anExp.More())
                {
                    Compare(anExp.Current(), $"random wire {i} ({aWhat}), an edge");
                }
            }
        }
        Report("ShapeFix_Shape on random wires", aWatch.Elapsed);
        Assert.Empty(myDifferences);
    }

    [Fact]
    public void ShapeFix_Shape_on_the_wires_of_the_heal_group_and_data_occ_matches_native_OCCT()
    {
        var aWatch = Stopwatch.StartNew();
        int aShapes = 0;
        var aFiles = new List<string>();
        foreach (string aFile in Directory.GetFiles(Path.Combine(Metadata("OcctSourceDir"), "data", "occ"), "*.brep").Order(StringComparer.Ordinal))
        {
            aFiles.Add(aFile);
        }

        string aRoot = Metadata("OcctTestDataDir");
        if (!string.IsNullOrEmpty(aRoot) && Directory.Exists(aRoot))
        {
            var aNames = new SortedSet<string>(StringComparer.Ordinal);
            var aLocate = new Regex(@"locate_data_file\s+([^\]\s]+)");
            foreach (string aCase in Directory.EnumerateFiles(Path.Combine(Metadata("OcctSourceDir"), "tests", "heal"), "*", SearchOption.AllDirectories))
            {
                foreach (Match aMatch in aLocate.Matches(File.ReadAllText(aCase)))
                {
                    aNames.Add(aMatch.Groups[1].Value);
                }
            }
            var aByName = new Dictionary<string, string>(StringComparer.Ordinal);
            foreach (string aFile in Directory.EnumerateFiles(aRoot, "*", SearchOption.AllDirectories))
            {
                aByName.TryAdd(Path.GetFileName(aFile), aFile);
            }
            foreach (string aName in aNames)
            {
                if (aByName.TryGetValue(aName, out string? aFile) && (aName.EndsWith(".brep", StringComparison.OrdinalIgnoreCase)
                                                                      || aName.EndsWith(".rle", StringComparison.OrdinalIgnoreCase))
                    && new FileInfo(aFile).Length <= 1_000_000)
                {
                    aFiles.Add(aFile);
                }
            }
        }
        else
        {
            output.WriteLine("OcctTestDataDir isn't set: data/occ only");
        }

        foreach (string aFile in aFiles)
        {
            if (NativeRead(aFile) is N.TopoDS_Shape aShape)
            {
                RunShape(aShape, Path.GetFileName(aFile), 12, 6);
                aShapes++;
            }
        }
        output.WriteLine($"{aShapes} shapes");
        Report("ShapeFix_Shape on the wires and edges of the heal group's and data/occ's shapes", aWatch.Elapsed);
        Assert.Empty(myDifferences);
    }
}
#endif
