#if HAS_ORACLE
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.TestHarness;
using Xunit.Abstractions;
using N = Build123d.Occt;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// BRepFilletAPI_MakeFillet2d (and through it ChFi2d_Builder, Geom2dGcc_Circ2d2TanRad and GccAna) against native
/// OCCT, unit 5R.
/// </summary>
/// <remarks>
/// <para>Inputs: planar faces built by the port and written to a BRep file that both sides read (the port's copy frozen,
/// as a caller's): random polygons and profiles of segments and arcs of circles in random planes, rectangles as
/// build123d.NET's <c>Face.MakeRect</c> makes them, and rectangles with a rectangular hole.</para>
/// <para>Each case runs the same operations on both sides: fillets, chamfers by two distances (build123d.NET's
/// <c>Chamfer2D</c>: the two edges of a vertex as MapShapesAndAncestors lists them) and by a distance and an angle,
/// then sometimes a modification or removal of the last one. Two patterns are build123d.NET's own: every corner of a
/// rectangle filleted (<c>RectangleRounded</c>, which build123d.NET builds through its own Geom2dGcc fillet, not
/// MakeFillet2d) and every corner chamfered (<c>Chamfer2D</c>). Compared: each operation's status and whether it
/// returned an edge; then IsDone, NbFillet, NbChamfer, NbCurves; the result as BRep text (the Free column masked, then
/// canonically); for every edge of the face IsModified, HasDescendant and Modified (as the index of the descendant in
/// the result, or the exception); NewEdges and BasisEdge as indices; and the input left unchanged.</para>
/// </remarks>
public class BRepFilletAPIMakeFillet2dOracleTests(ITestOutputHelper output) : IDisposable
{
    private readonly string myDirectory = Directory.CreateTempSubdirectory("occtnet-f2d-").FullName;
    private int myFileNumber;
    private readonly List<string> myDifferences = [];
    private int myCases, myIdentical, myFreeMasked, myCanonical, myNotDone, myOps, myOpsDone, myHistory;

    public void Dispose()
    {
        Directory.Delete(myDirectory, true);
        GC.SuppressFinalize(this);
    }

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

    private static string? ErrorOf(Exception theException) => theException switch
    {
        N.OcctException e => e.Message.IndexOf(':') is int c and > 0 ? e.Message[..c] : e.Message,
        Standard_Failure f => f.GetType().Name,
        _ => null,
    };

    // ---- inputs ----

    private static TopoDS_Face Face(in TopoDS_Wire theWire) => new BRepBuilderAPI_MakeFace(theWire, true).Face();

    private static gp_Pnt OnPlane(in gp_Ax2 theAx, double theU, double theV)
        => theAx.Location().Translated(new gp_Vec(theAx.XDirection()) * theU + new gp_Vec(theAx.YDirection()) * theV);

    // A polygon around the origin of the plane (theArcs: some sides arcs of circles)
    private static TopoDS_Face Profile(Oracle theOracle, bool theArcs, out double theSize)
    {
        gp_Ax2 anAx = theOracle.Bool() ? new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)) : theOracle.Ax2();
        int aNb = theOracle.Int(3, 8);
        double aRadius = theOracle.Real(1.0, 50.0);
        theSize = aRadius;
        var aPoints = new gp_Pnt[aNb];
        for (int i = 0; i < aNb; i++)
        {
            double anAngle = 2.0 * Math.PI * (i + 0.3 * theOracle.Real(-1.0, 1.0)) / aNb;
            double r = aRadius * theOracle.Real(0.6, 1.4);
            aPoints[i] = OnPlane(anAx, r * Math.Cos(anAngle), r * Math.Sin(anAngle));
        }
        if (!theArcs)
        {
            var aPoly = new BRepBuilderAPI_MakePolygon();
            foreach (gp_Pnt aP in aPoints)
            {
                aPoly.Add(aP);
            }
            aPoly.Close();
            return Face(aPoly.Wire());
        }
        var aWire = new BRepBuilderAPI_MakeWire();
        for (int i = 0; i < aNb; i++)
        {
            gp_Pnt a = aPoints[i], b = aPoints[(i + 1) % aNb];
            if (theOracle.Int(0, 9) < 4)
            {
                // an arc through a point off the middle of the side
                var aSide = new gp_Vec(a, b);
                gp_Vec aNormal = new gp_Vec(anAx.Direction()).Crossed(aSide);
                gp_Pnt aMid = a.Translated(aSide * 0.5).Translated(aNormal * theOracle.Real(-0.3, 0.3));
                aWire.Add(new BRepBuilderAPI_MakeEdge(new GC_MakeArcOfCircle(a, aMid, b).Value()).Edge());
            }
            else
            {
                aWire.Add(new BRepBuilderAPI_MakeEdge(a, b).Edge());
            }
        }
        return Face(aWire.Wire());
    }

    // build123d.NET's Face.MakeRect: a rectangle centred on the origin of Plane.XY
    private static TopoDS_Wire Rect(double theW, double theH, double theX = 0.0, double theY = 0.0)
    {
        var aPoly = new BRepBuilderAPI_MakePolygon(new gp_Pnt(theX - theW / 2, theY - theH / 2, 0), new gp_Pnt(theX + theW / 2, theY - theH / 2, 0),
                                                   new gp_Pnt(theX + theW / 2, theY + theH / 2, 0), new gp_Pnt(theX - theW / 2, theY + theH / 2, 0), true);
        return aPoly.Wire();
    }

    // ---- the operations, run alike on both sides ----

    private enum OpKind { Fillet, ChamferDD, ChamferDA, ModifyFillet, RemoveFillet, ModifyChamfer, RemoveChamfer }

    private sealed record Op(OpKind Kind, int Vertex, double A, double B, bool Swap);

    private static List<Op> RandomOps(Oracle theOracle, int theNbVertices, double theSize)
    {
        var anOps = new List<Op>();
        List<int> aVertices = Enumerable.Range(1, theNbVertices).OrderBy(_ => theOracle.Int(0, 1000)).Take(theOracle.Int(1, Math.Min(4, theNbVertices))).ToList();
        double[] aFactors = [0.01, 0.05, 0.1, 0.2, 0.35, 0.6, 1.5];
        foreach (int v in aVertices)
        {
            int k = theOracle.Int(0, 9);
            double a = theSize * aFactors[theOracle.Int(0, aFactors.Length - 1)];
            double b = theSize * aFactors[theOracle.Int(0, aFactors.Length - 1)];
            OpKind aKind = k < 5 ? OpKind.Fillet : (k < 8 ? OpKind.ChamferDD : OpKind.ChamferDA);
            if (aKind == OpKind.ChamferDA)
            {
                b = theOracle.Real(0.1, 1.4);
            }
            anOps.Add(new Op(aKind, v, a, b, theOracle.Bool()));
        }
        int m = theOracle.Int(0, 19);
        if (m < 3)
        {
            anOps.Add(new Op(m == 0 ? OpKind.ModifyFillet : OpKind.ModifyChamfer, 0, theSize * theOracle.Real(0.02, 0.3), theSize * theOracle.Real(0.02, 0.3), false));
        }
        else if (m < 5)
        {
            anOps.Add(new Op(m == 3 ? OpKind.RemoveFillet : OpKind.RemoveChamfer, 0, 0, 0, false));
        }
        return anOps;
    }

    private sealed class PortRun
    {
        public readonly List<string> Steps = [];
        public BRepFilletAPI_MakeFillet2d Maker = null!;
    }

    private static PortRun RunPort(in TopoDS_Face theFace, List<Op> theOps)
    {
        var aRun = new PortRun();
        var aMaker = new BRepFilletAPI_MakeFillet2d(theFace);
        aRun.Maker = aMaker;
        aRun.Steps.Add("init " + (int)aMaker.Status());
        NCollection_IndexedMap<TopoDS_Shape> aVertices = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theFace, TopAbs_VERTEX, aVertices);
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> anAncestors = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(theFace, TopAbs_VERTEX, TopAbs_EDGE, anAncestors);
        TopoDS_Edge aLastFillet = new(), aLastChamfer = new();
        foreach (Op anOp in theOps)
        {
            string aStep;
            try
            {
                TopoDS_Edge anEdge = new();
                TopoDS_Vertex aV = anOp.Vertex > 0 ? TopoDS.Vertex(aVertices.FindKey(anOp.Vertex)) : new TopoDS_Vertex();
                NCollection_List<TopoDS_Shape>? aList = anOp.Vertex > 0 ? anAncestors.FindFromKey(aV) : null;
                switch (anOp.Kind)
                {
                    case OpKind.Fillet:
                        anEdge = aMaker.AddFillet(aV, anOp.A);
                        if (!anEdge.IsNull())
                        {
                            aLastFillet = anEdge;
                        }
                        break;
                    case OpKind.ChamferDD:
                        {
                            TopoDS_Edge E1 = TopoDS.Edge(aList!.First()), E2 = TopoDS.Edge(aList.Last());
                            anEdge = anOp.Swap ? aMaker.AddChamfer(E2, E1, anOp.A, anOp.B) : aMaker.AddChamfer(E1, E2, anOp.A, anOp.B);
                            break;
                        }
                    case OpKind.ChamferDA:
                        anEdge = aMaker.AddChamfer(TopoDS.Edge(anOp.Swap ? aList!.Last() : aList!.First()), aV, anOp.A, anOp.B);
                        break;
                    case OpKind.ModifyFillet:
                        anEdge = aMaker.ModifyFillet(aLastFillet, anOp.A);
                        break;
                    case OpKind.RemoveFillet:
                        aStep = aMaker.RemoveFillet(aLastFillet).IsNull() ? "v0" : "v1";
                        aRun.Steps.Add($"{anOp.Kind} {aStep} {(int)aMaker.Status()}");
                        continue;
                    case OpKind.ModifyChamfer:
                        if (aMaker.NbChamfer() > 0)
                        {
                            aLastChamfer = TopoDS.Edge(aMaker.ChamferEdges()[aMaker.NbChamfer() - 1]);
                        }
                        anEdge = aMaker.ModifyChamfer(aLastChamfer, aLastChamfer, anOp.A, Math.Min(1.4, anOp.B));
                        break;
                    case OpKind.RemoveChamfer:
                        if (aMaker.NbChamfer() > 0)
                        {
                            aLastChamfer = TopoDS.Edge(aMaker.ChamferEdges()[aMaker.NbChamfer() - 1]);
                        }
                        aStep = aMaker.RemoveChamfer(aLastChamfer).IsNull() ? "v0" : "v1";
                        aRun.Steps.Add($"{anOp.Kind} {aStep} {(int)aMaker.Status()}");
                        continue;
                }
                aStep = $"{anOp.Kind} {(anEdge.IsNull() ? "e0" : "e1")} {(int)aMaker.Status()}";
            }
            catch (Standard_Failure e)
            {
                aStep = $"{anOp.Kind} !{e.GetType().Name}";
            }
            aRun.Steps.Add(aStep);
        }
        return aRun;
    }

    private sealed class NativeRun
    {
        public readonly List<string> Steps = [];
        public N.BRepFilletAPI_MakeFillet2d Maker = null!;
    }

    private static NativeRun RunNative(N.TopoDS_Face theFace, List<Op> theOps)
    {
        var aRun = new NativeRun();
        var aMaker = new N.BRepFilletAPI_MakeFillet2d(theFace);
        aRun.Maker = aMaker;
        aRun.Steps.Add("init " + (int)aMaker.Status());
        var aVertices = new N.TopTools_IndexedMapOfShape();
        N.TopExp.MapShapes(theFace, N.TopAbs_ShapeEnum.TopAbs_VERTEX, aVertices);
        var anAncestors = new N.TopTools_IndexedDataMapOfShapeListOfShape();
        N.TopExp.MapShapesAndAncestors(theFace, N.TopAbs_ShapeEnum.TopAbs_VERTEX, N.TopAbs_ShapeEnum.TopAbs_EDGE, anAncestors);
        N.TopoDS_Edge aLastFillet = new(), aLastChamfer = new();
        foreach (Op anOp in theOps)
        {
            string aStep;
            try
            {
                N.TopoDS_Edge anEdge = new();
                N.TopoDS_Vertex aV = anOp.Vertex > 0 ? N.TopoDS.Vertex(aVertices.FindKey(anOp.Vertex)) : new N.TopoDS_Vertex();
                N.TopTools_ListOfShape? aList = anOp.Vertex > 0 ? anAncestors.FindFromKey(aV) : null;
                switch (anOp.Kind)
                {
                    case OpKind.Fillet:
                        anEdge = aMaker.AddFillet(aV, anOp.A);
                        if (!anEdge.IsNull())
                        {
                            aLastFillet = anEdge;
                        }
                        break;
                    case OpKind.ChamferDD:
                        {
                            N.TopoDS_Edge E1 = N.TopoDS.Edge(aList!.First()), E2 = N.TopoDS.Edge(aList.Last());
                            anEdge = anOp.Swap ? aMaker.AddChamfer(E2, E1, anOp.A, anOp.B) : aMaker.AddChamfer(E1, E2, anOp.A, anOp.B);
                            break;
                        }
                    case OpKind.ChamferDA:
                        anEdge = aMaker.AddChamfer(N.TopoDS.Edge(anOp.Swap ? aList!.Last() : aList!.First()), aV, anOp.A, anOp.B);
                        break;
                    case OpKind.ModifyFillet:
                        anEdge = aMaker.ModifyFillet(aLastFillet, anOp.A);
                        break;
                    case OpKind.RemoveFillet:
                        aStep = aMaker.RemoveFillet(aLastFillet).IsNull() ? "v0" : "v1";
                        aRun.Steps.Add($"{anOp.Kind} {aStep} {(int)aMaker.Status()}");
                        continue;
                    case OpKind.ModifyChamfer:
                        if (aMaker.NbChamfer() > 0)
                        {
                            aLastChamfer = N.TopoDS.Edge(aMaker.ChamferEdges().Value(aMaker.NbChamfer()));
                        }
                        anEdge = aMaker.ModifyChamfer(aLastChamfer, aLastChamfer, anOp.A, Math.Min(1.4, anOp.B));
                        break;
                    case OpKind.RemoveChamfer:
                        if (aMaker.NbChamfer() > 0)
                        {
                            aLastChamfer = N.TopoDS.Edge(aMaker.ChamferEdges().Value(aMaker.NbChamfer()));
                        }
                        aStep = aMaker.RemoveChamfer(aLastChamfer).IsNull() ? "v0" : "v1";
                        aRun.Steps.Add($"{anOp.Kind} {aStep} {(int)aMaker.Status()}");
                        continue;
                }
                aStep = $"{anOp.Kind} {(anEdge.IsNull() ? "e0" : "e1")} {(int)aMaker.Status()}";
            }
            catch (N.OcctException e)
            {
                aStep = $"{anOp.Kind} !{ErrorOf(e)}";
            }
            aRun.Steps.Add(aStep);
        }
        return aRun;
    }

    // ---- the comparison ----

    private void Compare(in TopoDS_Face theFace, List<Op> theOps, string theWhat)
    {
        string aFile = NewFile();
        Assert.True(BRepTools.Write(theFace, aFile, true, false, TopTools_FormatVersion.TopTools_FormatVersion_VERSION_3));
        var aNShape = new N.TopoDS_Shape();
        using (var aBuilder = new N.BRep_Builder())
        {
            Assert.True(N.BRepTools.Read(aNShape, aFile, aBuilder));
        }
        Assert.True(BRepTools.Read(out TopoDS_Shape aPShape, aFile));
        File.Delete(aFile);
        aPShape.TShape()!.FreezeGraph(); // a caller's face
        string anInputText = BRepTextComparison.Write(aPShape);
        N.TopoDS_Face aNFace = N.TopoDS.Face(aNShape);
        TopoDS_Face aPFace = TopoDS.Face(aPShape);
        myCases++;

        NativeRun aN = RunNative(aNFace, theOps);
        PortRun aP = RunPort(aPFace, theOps);
        myOps += theOps.Count;
        myOpsDone += aN.Steps.Count(s => s.EndsWith(" 5", StringComparison.Ordinal));
        if (!aN.Steps.SequenceEqual(aP.Steps))
        {
            myDifferences.Add($"{theWhat}: steps native [{string.Join("; ", aN.Steps)}], port [{string.Join("; ", aP.Steps)}]");
            return;
        }

        aN.Maker.Build();
        aP.Maker.Build();
        bool isDone = aN.Maker.IsDone();
        string aCounts(int f, int c, int n) => $"{f}/{c}/{n}";
        if (isDone != aP.Maker.IsDone()
            || aCounts(aN.Maker.NbFillet(), aN.Maker.NbChamfer(), aN.Maker.NbCurves()) != aCounts(aP.Maker.NbFillet(), aP.Maker.NbChamfer(), aP.Maker.NbCurves()))
        {
            myDifferences.Add($"{theWhat}: IsDone or counts differ");
            return;
        }
        if (!isDone)
        {
            myNotDone++;
            return;
        }

        N.TopoDS_Shape aNRes = aN.Maker.Shape();
        TopoDS_Shape aPRes = aP.Maker.Shape();
        string aNT = NativeText(aNRes), aPT = PortText(aPRes);
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
            string aTextFile = NewFile();
            File.WriteAllText(aTextFile, aNT);
            Assert.True(BRepTools.Read(out TopoDS_Shape aNP, aTextFile));
            File.Delete(aTextFile);
            CanonicalComparison aCanon = CanonicalShapeComparer.Compare(aNP, aPRes);
            if (aCanon.IsIdentical)
            {
                myCanonical++;
            }
            else
            {
                myDifferences.Add($"{theWhat}: {aDiff}; canonically {aCanon}");
            }
        }

        // the history, as indices in the result
        var aNResEdges = new N.TopTools_IndexedMapOfShape();
        N.TopExp.MapShapes(aNRes, N.TopAbs_ShapeEnum.TopAbs_EDGE, aNResEdges);
        NCollection_IndexedMap<TopoDS_Shape> aPResEdges = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(aPRes, TopAbs_EDGE, aPResEdges);
        var aNEdges = new N.TopTools_IndexedMapOfShape();
        N.TopExp.MapShapes(aNFace, N.TopAbs_ShapeEnum.TopAbs_EDGE, aNEdges);
        NCollection_IndexedMap<TopoDS_Shape> aPEdges = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(aPFace, TopAbs_EDGE, aPEdges);
        var aNH = new List<string>();
        var aPH = new List<string>();
        for (int i = 1; i <= aNEdges.Extent(); i++)
        {
            N.TopoDS_Edge aNE = N.TopoDS.Edge(aNEdges.FindKey(i));
            TopoDS_Edge aPE = TopoDS.Edge(aPEdges.FindKey(i));
            aNH.Add($"{i}:{aN.Maker.IsModified(aNE)},{aN.Maker.HasDescendant(aNE)}");
            aPH.Add($"{i}:{aP.Maker.IsModified(aPE)},{aP.Maker.HasDescendant(aPE)}");
            try
            {
                aNH.Add(string.Join(",", aN.Maker.Modified(aNE).Select(s => aNResEdges.FindIndex(s))));
            }
            catch (N.OcctException e)
            {
                aNH.Add("!" + ErrorOf(e));
            }
            try
            {
                aPH.Add(string.Join(",", aP.Maker.Modified(aPE).Select(s => aPResEdges.FindIndex(s))));
            }
            catch (Standard_Failure e)
            {
                aPH.Add("!" + e.GetType().Name);
            }
        }
        for (int k = 1; k <= aN.Maker.NbCurves(); k++)
        {
            aNH.Add("new" + string.Join(",", aN.Maker.NewEdges(k).Select(s => aNResEdges.FindIndex(s))));
            aPH.Add("new" + string.Join(",", aP.Maker.NewEdges(k).Select(s => aPResEdges.FindIndex(s))));
        }
        for (int k = 1; k <= aNResEdges.Extent(); k++)
        {
            aNH.Add("basis" + aNEdges.FindIndex(aN.Maker.BasisEdge(N.TopoDS.Edge(aNResEdges.FindKey(k)))));
            aPH.Add("basis" + aPEdges.FindIndex(aP.Maker.BasisEdge(TopoDS.Edge(aPResEdges.FindKey(k)))));
        }
        if (aNH.SequenceEqual(aPH))
        {
            myHistory++;
        }
        else
        {
            myDifferences.Add($"{theWhat}: history native [{string.Join(" ", aNH)}], port [{string.Join(" ", aPH)}]");
        }

        if (BRepTextComparison.Write(aPShape) != anInputText)
        {
            myDifferences.Add($"{theWhat}: the input changed");
        }
    }

    private static int NbVertices(in TopoDS_Shape theShape)
    {
        NCollection_IndexedMap<TopoDS_Shape> aMap = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, TopAbs_VERTEX, aMap);
        return aMap.Extent();
    }

    private void Report()
    {
        output.WriteLine($"{myCases} cases, {myOps} operations ({myOpsDone} done natively); {myNotDone} not done on both sides; "
                         + $"results: {myIdentical} identical BRep text ({myFreeMasked} with the Free column masked), "
                         + $"{myCanonical} canonically identical; histories identical: {myHistory}; differences: {myDifferences.Count}");
        foreach (string aDiff in myDifferences.Take(30))
        {
            output.WriteLine("  " + aDiff);
        }
    }

    [Fact]
    public void Random_polygons_and_arc_profiles()
    {
        var anOracle = new Oracle(output, 5931);
        for (int i = 0; i < 400; i++)
        {
            bool isArcs = i % 2 == 1;
            TopoDS_Face aFace;
            double aSize;
            try
            {
                aFace = Profile(anOracle, isArcs, out aSize);
            }
            catch (Standard_Failure)
            {
                continue; // a profile the makers reject (a degenerate arc)
            }
            List<Op> anOps = RandomOps(anOracle, NbVertices(aFace), aSize);
            Compare(aFace, anOps, $"{(isArcs ? "arcs" : "polygon")} #{i} [{string.Join(", ", anOps)}]");
        }
        Report();
        Assert.Empty(myDifferences);
    }

    [Fact]
    public void Build123d_RectangleRounded_and_Chamfer2D_patterns()
    {
        var anOracle = new Oracle(output, 5932);
        for (int i = 0; i < 120; i++)
        {
            double w = anOracle.Real(1.0, 100.0), h = anOracle.Real(1.0, 100.0);
            bool isHole = i % 4 == 3;
            TopoDS_Face aFace;
            if (isHole)
            {
                var aMaker = new BRepBuilderAPI_MakeFace(Rect(w, h), true);
                aMaker.Add(TopoDS.Wire(Rect(w / 3, h / 3, w / 10, -h / 10).Reversed()));
                aFace = aMaker.Face();
            }
            else
            {
                aFace = Face(Rect(w, h));
            }
            int aNb = NbVertices(aFace);
            double aMin = Math.Min(w, h);
            List<Op> anOps;
            if (i % 2 == 0)
            {
                // RectangleRounded: every corner at one radius below half the smaller side (sometimes at it)
                double r = i % 10 == 0 ? aMin / 2 : aMin * anOracle.Real(0.01, 0.49);
                anOps = Enumerable.Range(1, aNb).Select(v => new Op(OpKind.Fillet, v, r, 0, false)).ToList();
            }
            else
            {
                // Chamfer2D: every corner, length and length2, the edges in the ancestors' order
                double d = aMin * anOracle.Real(0.01, 0.45), d2 = i % 3 == 0 ? d : aMin * anOracle.Real(0.01, 0.45);
                anOps = Enumerable.Range(1, aNb).Select(v => new Op(OpKind.ChamferDD, v, d, d2, false)).ToList();
            }
            Compare(aFace, anOps, $"rectangle #{i} {w}x{h}{(isHole ? " with a hole" : "")} [{anOps[0]}]");
        }
        Report();
        Assert.Empty(myDifferences);
    }
}
#endif
