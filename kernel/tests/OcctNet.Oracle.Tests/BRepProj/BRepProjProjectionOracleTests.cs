#if HAS_ORACLE
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.TestHarness;
using Xunit.Abstractions;
using N = Build123d.Occt;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// BRepProj_Projection (unit 5T; through it BRepFill_Generator's cone, BRepSweep_Prism's prism, BRepAlgoAPI_Section
/// and ShapeAnalysis_FreeBounds) against native OCCT, which build123d.NET binds for its projections.
/// </summary>
/// <remarks>
/// Inputs built by the port and written to one BRep file that both sides read (the port's frozen): wires and edges
/// (circles, polygons, ellipses, arcs, B-splines, open polylines) projected along a direction (cylindrical) or from a
/// point (conical) onto boxes, a box with a pocket, cylinders, spheres, cones, tori, planar and extruded faces and
/// compounds of faces with a vertex, as build123d.NET's <c>ProjectToShape</c> and <c>Project</c> do. Compared:
/// exceptions, IsDone, the wires (in order, else as a set) and the compound as BRep text (then with the Free column
/// masked, then canonically); the inputs left unchanged.
/// </remarks>
public sealed class BRepProjProjectionOracleTests(ITestOutputHelper output) : IDisposable
{
    private readonly string myDirectory = Directory.CreateTempSubdirectory("occtnet-brepproj-").FullName;
    private int myFileNumber;
    private readonly List<string> myDifferences = [];
    private int myCases, myDone, myNotDone, myExceptions, myWires, myIdentical, myFreeMasked, myCanonical, myOtherOrder;

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

    private (N.TopoDS_Shape[] N, TopoDS_Shape[] P) Both(params TopoDS_Shape[] theShapes)
    {
        BRep_Builder aB = new();
        TopoDS_Compound aComp = new();
        aB.MakeCompound(ref aComp);
        foreach (TopoDS_Shape aS in theShapes)
        {
            aB.Add(aComp, aS);
        }
        string aFile = NewFile();
        Assert.True(BRepTools.Write(aComp, aFile, true, false, TopTools_FormatVersion.TopTools_FormatVersion_VERSION_3));
        var aNShape = new N.TopoDS_Shape();
        using (var aBuilder = new N.BRep_Builder())
        {
            Assert.True(N.BRepTools.Read(aNShape, aFile, aBuilder));
        }
        Assert.True(BRepTools.Read(out TopoDS_Shape aPShape, aFile));
        File.Delete(aFile);
        aPShape.TShape()!.FreezeGraph();
        var aN = new List<N.TopoDS_Shape>();
        for (var anIt = new N.TopoDS_Iterator(aNShape); anIt.More(); anIt.Next())
        {
            aN.Add(anIt.Value());
        }
        var aP = new List<TopoDS_Shape>();
        for (TopoDS_Iterator anIt = new(aPShape); anIt.More(); anIt.Next())
        {
            aP.Add(anIt.Value());
        }
        return ([.. aN], [.. aP]);
    }

    // 0: identical text, 1: Free masked, 2: canonical, -1: different
    private int Compare(N.TopoDS_Shape theN, in TopoDS_Shape theP, out string theDiff)
    {
        theDiff = "";
        string aNT = NativeText(theN), aPT = PortText(theP);
        BRepTextDiff aDiff = BRepTextComparison.Compare(aNT, aPT);
        if (aDiff.IsIdentical)
        {
            return 0;
        }
        if (aDiff.IsIdenticalMaskingFree)
        {
            return 1;
        }
        string aTextFile = NewFile();
        File.WriteAllText(aTextFile, aNT);
        Assert.True(BRepTools.Read(out TopoDS_Shape aNP, aTextFile));
        File.WriteAllText(aTextFile, aPT);
        Assert.True(BRepTools.Read(out TopoDS_Shape aPP, aTextFile));
        File.Delete(aTextFile);
        CanonicalComparison aCanon = CanonicalShapeComparer.Compare(aNP, aPP);
        if (aCanon.IsIdentical)
        {
            return 2;
        }
        theDiff = $"{aDiff}; canonically {aCanon}";
        return -1;
    }

    private void Count(int theOutcome)
    {
        myIdentical += theOutcome is 0 or 1 ? 1 : 0;
        myFreeMasked += theOutcome == 1 ? 1 : 0;
        myCanonical += theOutcome == 2 ? 1 : 0;
    }

    // Projects input 0 onto input 1 on both sides, along theDir or from thePnt
    private void Run(string theName, N.TopoDS_Shape[] theNIn, TopoDS_Shape[] thePIn, gp_Dir? theDir, gp_Pnt? thePnt)
    {
        myCases++;
        string[] anInputTexts = [.. thePIn.Select(s => PortText(s))];
        string? aNError = null, aPError = null;
        N.BRepProj_Projection? aN = null;
        BRepProj_Projection? aP = null;
        try
        {
            aN = theDir is { } d ? new N.BRepProj_Projection(theNIn[0], theNIn[1], Oracle.ToNative(d))
                                 : new N.BRepProj_Projection(theNIn[0], theNIn[1], Oracle.ToNative(thePnt!.Value));
        }
        catch (N.OcctException e)
        {
            aNError = e.OcctType;
        }
        try
        {
            aP = theDir is { } d ? new BRepProj_Projection(thePIn[0], thePIn[1], d)
                                 : new BRepProj_Projection(thePIn[0], thePIn[1], thePnt!.Value);
        }
        catch (Standard_Failure f)
        {
            aPError = f.GetType().Name;
        }
        catch (Exception e)
        {
            myDifferences.Add($"{theName}: native {aNError ?? "no exception"}, the port throws {e.GetType().Name} {e.StackTrace?.Split('\n')[0]}");
            return;
        }
        for (int i = 0; i < thePIn.Length; i++)
        {
            if (PortText(thePIn[i]) != anInputTexts[i])
            {
                myDifferences.Add($"{theName}: input {i} changed");
            }
        }
        if (aNError is not null || aPError is not null)
        {
            if (aNError is null || aPError is null || !aNError.StartsWith(aPError, StringComparison.Ordinal))
            {
                myDifferences.Add($"{theName}: {aNError ?? "no exception"} natively, {aPError ?? "no exception"} in the port");
            }
            else
            {
                myExceptions++;
            }
            return;
        }
        using (aN)
        {
            if (aN!.IsDone() != aP!.IsDone())
            {
                myDifferences.Add($"{theName}: IsDone native {aN.IsDone()}, port {aP.IsDone()}");
                return;
            }
            if (!aP.IsDone())
            {
                myNotDone++;
                return;
            }
            myDone++;
            var aNW = new List<N.TopoDS_Wire>();
            for (aN.Init(); aN.More(); aN.Next())
            {
                aNW.Add(aN.Current());
            }
            var aPW = new List<TopoDS_Wire>();
            for (aP.Init(); aP.More(); aP.Next())
            {
                aPW.Add(aP.Current());
                Assert.True(aP.Current().TShape()!.IsFrozen());
            }
            if (aNW.Count != aPW.Count)
            {
                myDifferences.Add($"{theName}: {aNW.Count} wires natively, {aPW.Count} in the port");
                return;
            }
            myWires += aPW.Count;
            int[] anOutcomes = new int[aPW.Count];
            bool isInOrder = true;
            for (int i = 0; i < aPW.Count && isInOrder; i++)
            {
                anOutcomes[i] = Compare(aNW[i], aPW[i], out _);
                isInOrder = anOutcomes[i] >= 0;
            }
            if (!isInOrder)
            {
                // as a set: each native wire matched with a port wire
                var aFree = Enumerable.Range(0, aPW.Count).ToList();
                for (int i = 0; i < aNW.Count; i++)
                {
                    int aMatch = -1;
                    string aDiff = "";
                    foreach (int j in aFree)
                    {
                        int anOutcome = Compare(aNW[i], aPW[j], out aDiff);
                        if (anOutcome >= 0)
                        {
                            aMatch = j;
                            anOutcomes[i] = anOutcome;
                            break;
                        }
                    }
                    if (aMatch < 0)
                    {
                        myDifferences.Add($"{theName}: wire {i}: {aDiff}");
                        return;
                    }
                    aFree.Remove(aMatch);
                }
                myOtherOrder++;
            }
            if (anOutcomes.Any(o => o == 2) && NativeRerunMatches(theNIn, aPW, theDir, thePnt))
            {
                // the same wires as BRep text on another native run: the order of the section's edges natively follows
                // TShape addresses (decision 939)
                anOutcomes = [.. anOutcomes.Select(_ => 1)];
                myNativeVaries.Add(theName);
            }
            if (anOutcomes.Any(o => o == 2))
            {
                myCanonicalCases.Add(theName);
                if (!DestructiveMatches(aNW, thePIn, theDir, thePnt))
                {
                    myDifferences.Add($"{theName}: canonically identical only, and not byte-identical on unfrozen inputs either");
                }
            }
            foreach (int anOutcome in anOutcomes)
            {
                Count(anOutcome);
            }
            if (isInOrder && Compare(aN.Shape(), aP.Shape(), out string aShapeDiff) < 0)
            {
                myDifferences.Add($"{theName}: Shape(): {aShapeDiff}");
            }
        }
    }

    private readonly List<string> myNativeVaries = [];
    private readonly List<string> myCanonicalCases = [];

    // Native run again on new TShapes of the same inputs, at other heap addresses: true if one run gives the port's
    // wires as BRep text, in order
    private bool NativeRerunMatches(N.TopoDS_Shape[] theNIn, List<TopoDS_Wire> thePW, gp_Dir? theDir, gp_Pnt? thePnt)
    {
        List<N.TopoDS_Shape> aBallast = [];
        for (int anAttempt = 0; anAttempt < 8; anAttempt++)
        {
            for (int i = 0; i < 997 * (anAttempt % 4 + 1) + 13 * anAttempt; i++)
            {
                aBallast.Add(new N.BRepBuilderAPI_MakeVertex(new N.gp_Pnt(i, anAttempt, 0)).Vertex());
            }
            var aComp = new N.TopoDS_Compound();
            using var aB = new N.BRep_Builder();
            aB.MakeCompound(aComp);
            foreach (N.TopoDS_Shape aS in theNIn)
            {
                aB.Add(aComp, aS);
            }
            string aFile = NewFile();
            Assert.True(N.BRepTools.Write(aComp, aFile, true, false, N.TopTools_FormatVersion.TopTools_FormatVersion_VERSION_3));
            var aRead = new N.TopoDS_Shape();
            Assert.True(N.BRepTools.Read(aRead, aFile, aB));
            File.Delete(aFile);
            var aIn = new List<N.TopoDS_Shape>();
            for (var anIt = new N.TopoDS_Iterator(aRead); anIt.More(); anIt.Next())
            {
                aIn.Add(anIt.Value());
            }
            using N.BRepProj_Projection aN = theDir is { } d ? new N.BRepProj_Projection(aIn[0], aIn[1], Oracle.ToNative(d))
                                                             : new N.BRepProj_Projection(aIn[0], aIn[1], Oracle.ToNative(thePnt!.Value));
            var aNW = new List<N.TopoDS_Wire>();
            for (aN.Init(); aN.More(); aN.Next())
            {
                aNW.Add(aN.Current());
            }
            if (aNW.Count == thePW.Count && Enumerable.Range(0, aNW.Count).All(i => Compare(aNW[i], thePW[i], out _) is 0 or 1))
            {
                return true;
            }
        }
        return false;
    }

    // The port on unfrozen copies of its inputs (so its section runs destructively, as native's): true if every wire is
    // then the same BRep text as native's
    private bool DestructiveMatches(List<N.TopoDS_Wire> theNW, TopoDS_Shape[] thePIn, gp_Dir? theDir, gp_Pnt? thePnt)
    {
        var aIn = thePIn.Select(s => BRepTools.ExactCopyForWrite(s, out _)).ToArray();
        BRepProj_Projection aP = theDir is { } d ? new BRepProj_Projection(aIn[0], aIn[1], d)
                                                 : new BRepProj_Projection(aIn[0], aIn[1], thePnt!.Value);
        var aPW = new List<TopoDS_Wire>();
        for (aP.Init(); aP.More(); aP.Next())
        {
            aPW.Add(aP.Current());
        }
        return aPW.Count == theNW.Count && Enumerable.Range(0, aPW.Count).All(i => Compare(theNW[i], aPW[i], out _) is 0 or 1);
    }

    private void Report(string theWhat)
    {
        output.WriteLine($"{theWhat}: {myCases} cases; {myExceptions} same exception on both sides, {myNotDone} not done on both "
                         + $"sides, {myDone} done with {myWires} wires: {myIdentical} identical BRep text ({myFreeMasked} with "
                         + $"the Free column masked), {myCanonical} canonically identical, each byte-identical when the port's section runs destructively as native's does on unfrozen copies (decision 577: {string.Join("; ", myCanonicalCases)}); {myOtherOrder} cases with the wires "
                         + $"in another order; {myNativeVaries.Count} cases matched on another native run ({string.Join("; ", myNativeVaries)}); "
                         + $"differences: {myDifferences.Count}");
        foreach (string aDiff in myDifferences.Take(40))
        {
            output.WriteLine("  " + aDiff);
        }
    }

    // ---- inputs ----

    private static TopoDS_Wire Wire(params TopoDS_Edge[] theEdges)
    {
        BRepBuilderAPI_MakeWire aMW = new();
        foreach (TopoDS_Edge anE in theEdges)
        {
            aMW.Add(anE);
        }
        return aMW.Wire();
    }

    private static gp_Pnt OnPlane(in gp_Ax2 theAx, double theU, double theV) =>
        theAx.Location().Translated(new gp_Vec(theAx.XDirection()) * theU + new gp_Vec(theAx.YDirection()) * theV);

    // A wire (or an edge) to project, in the plane theAx
    private static (TopoDS_Shape Shape, string Kind) Projected(Oracle theOracle, in gp_Ax2 theAx, double theSize)
    {
        double r = theSize;
        switch (theOracle.Int(0, 6))
        {
            case 0:
                return (Wire(new BRepBuilderAPI_MakeEdge(new gp_Circ(theAx, r)).Edge()), "circle");
            case 1:
            {
                int aNb = theOracle.Int(3, 6);
                BRepBuilderAPI_MakePolygon aP = new();
                for (int i = 0; i < aNb; i++)
                {
                    double a = 2 * Math.PI * i / aNb + 0.3;
                    aP.Add(OnPlane(theAx, r * Math.Cos(a), r * Math.Sin(a)));
                }
                aP.Close();
                return (aP.Wire(), $"polygon{aNb}");
            }
            case 2:
                return (Wire(new BRepBuilderAPI_MakeEdge(new gp_Elips(theAx, r, r * 0.6)).Edge()), "ellipse");
            case 3:
                return (new BRepBuilderAPI_MakeEdge(new GC_MakeArcOfCircle(new gp_Circ(theAx, r), 0.2, 2.5, true).Value()).Edge(), "arc edge");
            case 4:
            {
                NCollection_HArray1<gp_Pnt> aPts = new(1, 5);
                for (int i = 1; i <= 5; i++)
                {
                    aPts.SetValue(i, OnPlane(theAx, -r + r * (i - 1) / 2, theOracle.Real(-0.5, 0.5) * r));
                }
                GeomAPI_Interpolate anI = new(aPts, false, 1.0e-7);
                anI.Perform();
                return (Wire(new BRepBuilderAPI_MakeEdge(anI.Curve()).Edge()), "bspline");
            }
            case 5:
            {
                BRepBuilderAPI_MakePolygon aP = new();
                aP.Add(OnPlane(theAx, -r, -r * 0.3));
                aP.Add(OnPlane(theAx, 0, r * 0.4));
                aP.Add(OnPlane(theAx, r, -r * 0.2));
                return (aP.Wire(), "open polyline");
            }
            default:
                return (new BRepBuilderAPI_MakeEdge(OnPlane(theAx, -r, 0), OnPlane(theAx, r, r * 0.2)).Edge(), "segment edge");
        }
    }

    // A target around the origin
    private static (TopoDS_Shape Shape, string Kind) Target(Oracle theOracle)
    {
        switch (theOracle.Int(0, 8))
        {
            case 0:
                return (new BRepPrimAPI_MakeBox(new gp_Pnt(-5, -5, -2), 10, 10, 4).Shape(), "box");
            case 1:
            {
                TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(new gp_Pnt(-5, -5, -2), 10, 10, 4).Shape();
                TopoDS_Shape aPocket = new BRepPrimAPI_MakeBox(new gp_Pnt(-2.5, -2.5, 0), 5, 5, 2).Shape();
                return (new BRepAlgoAPI_Cut(aBox, aPocket).Shape(), "pocketed box");
            }
            case 2:
                return (new BRepPrimAPI_MakeCylinder(new gp_Ax2(new gp_Pnt(0, -6, 0), new gp_Dir(0, 1, 0)), 4, 12).Shape(), "cylinder");
            case 3:
                return (new BRepPrimAPI_MakeSphere(5).Shape(), "sphere");
            case 4:
                return (new BRepPrimAPI_MakeCone(new gp_Ax2(new gp_Pnt(0, 0, -4), new gp_Dir(0, 0, 1)), 6, 1, 6).Shape(), "cone");
            case 5:
                return (new BRepPrimAPI_MakeTorus(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 4, 1.5).Shape(), "torus");
            case 6:
            {
                // a cylindrical face alone (build123d.NET projects onto faces)
                TopoDS_Shape aCyl = new BRepPrimAPI_MakeCylinder(new gp_Ax2(new gp_Pnt(0, -6, 0), new gp_Dir(0, 1, 0)), 4, 12).Shape();
                TopExp_Explorer anExp = new(aCyl, TopAbs_FACE);
                return (anExp.Current(), "cylindrical face");
            }
            case 7:
            {
                // an extruded elliptical arc (build123d.NET's tip-arc test)
                Geom_TrimmedCurve anArc = new GC_MakeArcOfEllipse(new gp_Elips(new gp_Ax2(), 6, 3), 3.5, 5.9, true).Value();
                TopoDS_Edge anEdge = new BRepBuilderAPI_MakeEdge(anArc).Edge();
                TopoDS_Shape aFace = new BRepPrimAPI_MakePrism(anEdge, new gp_Vec(0, 0, 4)).Shape();
                gp_Trsf aT = new();
                aT.SetTranslation(new gp_Vec(0, 0, -2));
                return (aFace.Moved(new TopLoc_Location(aT)), "extruded elliptical face");
            }
            default:
            {
                // a compound of faces of a box and a vertex (the section takes the faces only)
                BRep_Builder aB = new();
                TopoDS_Compound aC = new();
                aB.MakeCompound(ref aC);
                TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(new gp_Pnt(-4, -4, -1), 8, 8, 2).Shape();
                int i = 0;
                for (TopExp_Explorer anExp = new(aBox, TopAbs_FACE); anExp.More(); anExp.Next(), i++)
                {
                    if (i % 2 == 0)
                    {
                        aB.Add(aC, anExp.Current());
                    }
                }
                aB.Add(aC, new BRepBuilderAPI_MakeVertex(new gp_Pnt(9, 9, 9)).Vertex());
                return (aC, "compound of faces");
            }
        }
    }

    // ---- the cases ----

    [Fact]
    public void Cylindrical_projections_along_a_direction()
    {
        var anOracle = new Oracle(output, 20261111);
        for (int aCase = 0; aCase < 140; aCase++)
        {
            (TopoDS_Shape aTarget, string aTK) = Target(anOracle);
            gp_Dir aDir = Oracle.Settled(new gp_Dir(anOracle.Real(-0.4, 0.4), anOracle.Real(-0.4, 0.4), -1));
            gp_Ax2 aPlane = new(new gp_Pnt(anOracle.Real(-1, 1), anOracle.Real(-1, 1), 10), new gp_Dir(0, 0, 1));
            (TopoDS_Shape aWire, string aWK) = Projected(anOracle, aPlane, anOracle.Real(0.5, 3));
            (N.TopoDS_Shape[] aNIn, TopoDS_Shape[] aPIn) = Both(aWire, aTarget);
            Run($"case {aCase} {aWK} onto {aTK}", aNIn, aPIn, aDir, null);
        }
        Report("cylindrical");
        Assert.Empty(myDifferences);
    }

    [Fact]
    public void Conical_projections_from_a_point()
    {
        var anOracle = new Oracle(output, 20261112);
        for (int aCase = 0; aCase < 120; aCase++)
        {
            (TopoDS_Shape aTarget, string aTK) = Target(anOracle);
            gp_Pnt aCenter = new(anOracle.Real(-1, 1), anOracle.Real(-1, 1), anOracle.Real(14, 20));
            gp_Ax2 aPlane = new(new gp_Pnt(anOracle.Real(-1, 1), anOracle.Real(-1, 1), 10), new gp_Dir(0, 0, 1));
            (TopoDS_Shape aWire, string aWK) = Projected(anOracle, aPlane, anOracle.Real(0.5, 2));
            (N.TopoDS_Shape[] aNIn, TopoDS_Shape[] aPIn) = Both(aWire, aTarget);
            Run($"case {aCase} {aWK} onto {aTK}", aNIn, aPIn, null, aCenter);
        }
        Report("conical");
        Assert.Empty(myDifferences);
    }

    [Fact]
    public void Build123d_projection_cases_and_errors()
    {
        // build123d.NET's ProjectionTests: a circle onto a box with a pocket (two wires and more), a circle edge onto a
        // unit box from below, a circle onto a sphere from its centre; then the errors
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(new gp_Pnt(-5, -5, -2), 10, 10, 4).Shape();
        TopoDS_Shape aPocket = new BRepPrimAPI_MakeBox(new gp_Pnt(-2.5, -2.5, 0), 5, 5, 2).Shape();
        TopoDS_Shape aTarget = new BRepAlgoAPI_Cut(aBox, aPocket).Shape();
        TopoDS_Wire aCircle = Wire(new BRepBuilderAPI_MakeEdge(new gp_Circ(new gp_Ax2(new gp_Pnt(0, 0, 10), new gp_Dir(0, 0, 1)), 3)).Edge());
        (N.TopoDS_Shape[] aNIn, TopoDS_Shape[] aPIn) = Both(aCircle, aTarget);
        Run("circle onto a pocketed box", aNIn, aPIn, new gp_Dir(0, 0, -1), null);

        TopoDS_Edge aCircleEdge = new BRepBuilderAPI_MakeEdge(new gp_Circ(new gp_Ax2(new gp_Pnt(0, 0, -5), new gp_Dir(0, 0, 1)), 1)).Edge();
        (aNIn, aPIn) = Both(aCircleEdge, new BRepPrimAPI_MakeBox(1, 1, 1).Shape());
        Run("circle edge onto a unit box", aNIn, aPIn, new gp_Dir(0, 0, 1), null);

        (aNIn, aPIn) = Both(Wire(new BRepBuilderAPI_MakeEdge(new gp_Circ(new gp_Ax2(), 1)).Edge()), new BRepPrimAPI_MakeSphere(50).Shape());
        Run("circle onto a sphere from its centre", aNIn, aPIn, null, new gp_Pnt(0, 0, 0.5));

        // errors: a vertex projected, a target without faces, the point on the wire's first vertex
        TopoDS_Vertex aV = new BRepBuilderAPI_MakeVertex(new gp_Pnt(0, 0, 10)).Vertex();
        (aNIn, aPIn) = Both(aV, aTarget);
        Run("a vertex projected", aNIn, aPIn, new gp_Dir(0, 0, -1), null);
        (aNIn, aPIn) = Both(aCircle, aCircle);
        Run("onto a wire", aNIn, aPIn, new gp_Dir(0, 0, -1), null);
        (aNIn, aPIn) = Both(aCircle, aTarget);
        TopExp_Explorer anExp = new(aCircle, TopAbs_VERTEX);
        Run("from the first vertex", aNIn, aPIn, null, BRep_Tool.Pnt(TopoDS.Vertex(anExp.Current())));
        Report("build123d.NET cases");
        Assert.Empty(myDifferences);
    }
}
#endif
