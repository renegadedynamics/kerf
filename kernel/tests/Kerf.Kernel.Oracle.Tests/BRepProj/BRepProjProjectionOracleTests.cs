// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Curves;
using Kerf.Engine.Exchange.Brep;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.TestHarness;
using Xunit.Abstractions;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.Oracle.Tests;

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

    private string PortText(in Shape theShape)
    {
        string aFile = NewFile();
        Assert.True(BrepTools.Write(theShape, aFile, true, false, FormatVersion.FormatVersion_VERSION_3));
        string aText = File.ReadAllText(aFile);
        File.Delete(aFile);
        return aText;
    }

    private (N.TopoDS_Shape[] N, Shape[] P) Both(params Shape[] theShapes)
    {
        BrepBuilder aB = new();
        Compound aComp = new();
        aB.MakeCompound(ref aComp);
        foreach (Shape aS in theShapes)
        {
            aB.Add(aComp, aS);
        }
        string aFile = NewFile();
        Assert.True(BrepTools.Write(aComp, aFile, true, false, FormatVersion.FormatVersion_VERSION_3));
        string aText = File.ReadAllText(aFile);
        var aNShape = new N.TopoDS_Shape();
        using (var aBuilder = new N.BRep_Builder())
        {
            Assert.True(N.BRepTools.Read(aNShape, aFile, aBuilder));
        }
        Assert.True(BrepTools.Read(out Shape aPShape, aFile));
        File.Delete(aFile);
        aPShape.TShape()!.FreezeGraph();
        var aN = new List<N.TopoDS_Shape>();
        for (var anIt = new N.TopoDS_Iterator(aNShape); anIt.More(); anIt.Next())
        {
            aN.Add(anIt.Value());
        }
        GC.KeepAlive(aNShape); // native TopoDS_Iterator holds no reference to the shape it iterates (decision 1109)
        var aP = new List<Shape>();
        for (Iterator anIt = new(aPShape); anIt.More(); anIt.Next())
        {
            aP.Add(anIt.Value());
        }
        N.TopoDS_Shape[] aNIn = [.. aN];
        mySourceTexts[aNIn] = aText;
        return (aNIn, [.. aP]);
    }

    // The text each native input array was read from, for the native reruns (decision 944)
    private readonly Dictionary<N.TopoDS_Shape[], string> mySourceTexts = new(ReferenceEqualityComparer.Instance);

    // 0: identical text, 1: Free masked, 2: canonical, -1: different
    private int Compare(N.TopoDS_Shape theN, in Shape theP, out string theDiff)
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
        Assert.True(BrepTools.Read(out Shape aNP, aTextFile));
        File.WriteAllText(aTextFile, aPT);
        Assert.True(BrepTools.Read(out Shape aPP, aTextFile));
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
    private void Run(string theName, N.TopoDS_Shape[] theNIn, Shape[] thePIn, Direction3D? theDir, Point3D? thePnt)
    {
        myCases++;
        string[] anInputTexts = [.. thePIn.Select(s => PortText(s))];
        string? aNError = null, aPError = null;
        N.BRepProj_Projection? aN = null;
        Projection? aP = null;
        try
        {
            aN = Oracle.NativeWithRetry(() => theDir is { } d ? new N.BRepProj_Projection(theNIn[0], theNIn[1], Oracle.ToNative(d))
                                                              : new N.BRepProj_Projection(theNIn[0], theNIn[1], Oracle.ToNative(thePnt!.Value)));
        }
        catch (N.OcctException e)
        {
            aNError = e.OcctType;
        }
        try
        {
            aP = theDir is { } d ? new Projection(thePIn[0], thePIn[1], d)
                                 : new Projection(thePIn[0], thePIn[1], thePnt!.Value);
        }
        catch (FailureException f)
        {
            aPError = global::Kerf.Engine.Foundation.OcctTypeNames.Of(f.GetType());
        }
        catch (Exception e)
        {
            myDifferences.Add($"{theName}: native {aNError ?? "no exception"}, the port throws {Kerf.Engine.Foundation.OcctTypeNames.Of(e.GetType())} {e.StackTrace?.Split('\n')[0]}");
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
            if (aN!.IsDone() != aP!.IsDone)
            {
                myDifferences.Add($"{theName}: IsDone native {aN.IsDone()}, port {aP.IsDone}");
                return;
            }
            if (!aP.IsDone)
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
            var aPW = new List<Wire>();
            for (aP.Init(); aP.More(); aP.Next())
            {
                aPW.Add(aP.Current());
                Assert.True(aP.Current().TShape()!.IsFrozen);
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
    // wires as BRep text, in order. The new TShapes are read from the text both sides read, not from a write of
    // native's shapes, which moves curve and surface directions by an ulp (decision 944).
    private bool NativeRerunMatches(N.TopoDS_Shape[] theNIn, List<Wire> thePW, Direction3D? theDir, Point3D? thePnt)
    {
        List<N.TopoDS_Shape> aBallast = [];
        for (int anAttempt = 0; anAttempt < 8; anAttempt++)
        {
            for (int i = 0; i < 997 * (anAttempt % 4 + 1) + 13 * anAttempt; i++)
            {
                aBallast.Add(new N.BRepBuilderAPI_MakeVertex(new N.gp_Pnt(i, anAttempt, 0)).Vertex());
            }
            using var aB = new N.BRep_Builder();
            string aFile = NewFile();
            File.WriteAllText(aFile, mySourceTexts[theNIn]);
            var aRead = new N.TopoDS_Shape();
            Assert.True(N.BRepTools.Read(aRead, aFile, aB));
            File.Delete(aFile);
            var aIn = new List<N.TopoDS_Shape>();
            for (var anIt = new N.TopoDS_Iterator(aRead); anIt.More(); anIt.Next())
            {
                aIn.Add(anIt.Value());
            }
            GC.KeepAlive(aRead); // native TopoDS_Iterator holds no reference to the shape it iterates (decision 1109)
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
    private bool DestructiveMatches(List<N.TopoDS_Wire> theNW, Shape[] thePIn, Direction3D? theDir, Point3D? thePnt)
    {
        var aIn = thePIn.Select(s => BrepTools.ExactCopyForWrite(s, out _)).ToArray();
        Projection aP = theDir is { } d ? new Projection(aIn[0], aIn[1], d)
                                                 : new Projection(aIn[0], aIn[1], thePnt!.Value);
        var aPW = new List<Wire>();
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

    private static Wire Wire(params Edge[] theEdges)
    {
        MakeWire aMW = new();
        foreach (Edge anE in theEdges)
        {
            aMW.Add(anE);
        }
        return aMW.Wire();
    }

    private static Point3D OnPlane(in Frame3D theAx, double theU, double theV) =>
        theAx.Location().Translated(new Vector3D(theAx.XDirection()) * theU + new Vector3D(theAx.YDirection()) * theV);

    // A wire (or an edge) to project, in the plane theAx
    private static (Shape Shape, string Kind) Projected(Oracle theOracle, in Frame3D theAx, double theSize)
    {
        double r = theSize;
        switch (theOracle.Int(0, 6))
        {
            case 0:
                return (Wire(new MakeEdge(new Circle3D(theAx, r)).Edge()), "circle");
            case 1:
            {
                int aNb = theOracle.Int(3, 6);
                MakePolygon aP = new();
                for (int i = 0; i < aNb; i++)
                {
                    double a = 2 * Math.PI * i / aNb + 0.3;
                    aP.Add(OnPlane(theAx, r * Math.Cos(a), r * Math.Sin(a)));
                }
                aP.Close();
                return (aP.Wire(), $"polygon{aNb}");
            }
            case 2:
                return (Wire(new MakeEdge(new Ellipse3D(theAx, r, r * 0.6)).Edge()), "ellipse");
            case 3:
                return (new MakeEdge(new MakeArcOfCircle(new Circle3D(theAx, r), 0.2, 2.5, true).Value()).Edge(), "arc edge");
            case 4:
            {
                NCollection_HArray1<Point3D> aPts = new(1, 5);
                for (int i = 1; i <= 5; i++)
                {
                    aPts.SetValue(i, OnPlane(theAx, -r + r * (i - 1) / 2, theOracle.Real(-0.5, 0.5) * r));
                }
                ApiInterpolate anI = new(aPts, false, 1.0e-7);
                anI.Perform();
                return (Wire(new MakeEdge(anI.Curve()).Edge()), "bspline");
            }
            case 5:
            {
                MakePolygon aP = new();
                aP.Add(OnPlane(theAx, -r, -r * 0.3));
                aP.Add(OnPlane(theAx, 0, r * 0.4));
                aP.Add(OnPlane(theAx, r, -r * 0.2));
                return (aP.Wire(), "open polyline");
            }
            default:
                return (new MakeEdge(OnPlane(theAx, -r, 0), OnPlane(theAx, r, r * 0.2)).Edge(), "segment edge");
        }
    }

    // A target around the origin
    private static (Shape Shape, string Kind) Target(Oracle theOracle)
    {
        switch (theOracle.Int(0, 8))
        {
            case 0:
                return (new MakeBox(new Point3D(-5, -5, -2), 10, 10, 4).Shape(), "box");
            case 1:
            {
                Shape aBox = new MakeBox(new Point3D(-5, -5, -2), 10, 10, 4).Shape();
                Shape aPocket = new MakeBox(new Point3D(-2.5, -2.5, 0), 5, 5, 2).Shape();
                return (new Cut(aBox, aPocket).Shape(), "pocketed box");
            }
            case 2:
                return (new ApiMakeCylinder(new Frame3D(new Point3D(0, -6, 0), new Direction3D(0, 1, 0)), 4, 12).Shape(), "cylinder");
            case 3:
                return (new MakeSphere(5).Shape(), "sphere");
            case 4:
                return (new ApiMakeCone(new Frame3D(new Point3D(0, 0, -4), new Direction3D(0, 0, 1)), 6, 1, 6).Shape(), "cone");
            case 5:
                return (new MakeTorus(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 4, 1.5).Shape(), "torus");
            case 6:
            {
                // a cylindrical face alone (build123d.NET projects onto faces)
                Shape aCyl = new ApiMakeCylinder(new Frame3D(new Point3D(0, -6, 0), new Direction3D(0, 1, 0)), 4, 12).Shape();
                Explorer anExp = new(aCyl, ShapeType.Face);
                return (anExp.Current(), "cylindrical face");
            }
            case 7:
            {
                // an extruded elliptical arc (build123d.NET's tip-arc test)
                TrimmedCurve3D anArc = new MakeArcOfEllipse(new Ellipse3D(new Frame3D(), 6, 3), 3.5, 5.9, true).Value();
                Edge anEdge = new MakeEdge(anArc).Edge();
                Shape aFace = new ApiMakePrism(anEdge, new Vector3D(0, 0, 4)).Shape();
                Transform3D aT = new();
                aT.SetTranslation(new Vector3D(0, 0, -2));
                return (aFace.Moved(new Placement(aT)), "extruded elliptical face");
            }
            default:
            {
                // a compound of faces of a box and a vertex (the section takes the faces only)
                BrepBuilder aB = new();
                Compound aC = new();
                aB.MakeCompound(ref aC);
                Shape aBox = new MakeBox(new Point3D(-4, -4, -1), 8, 8, 2).Shape();
                int i = 0;
                for (Explorer anExp = new(aBox, ShapeType.Face); anExp.More(); anExp.Next(), i++)
                {
                    if (i % 2 == 0)
                    {
                        aB.Add(aC, anExp.Current());
                    }
                }
                aB.Add(aC, new MakeVertex(new Point3D(9, 9, 9)).Vertex());
                return (aC, "compound of faces");
            }
        }
    }

    // ---- the cases ----

    [Fact]
    public void Cylindrical_projections_along_a_direction()
    {
        var anOracle = new Oracle(output, 20261111);
        foreach (int aCase in Tape.Budget(140))
        {
            (Shape aTarget, string aTK) = Target(anOracle);
            Direction3D aDir = Oracle.Settled(new Direction3D(anOracle.Real(-0.4, 0.4), anOracle.Real(-0.4, 0.4), -1));
            Frame3D aPlane = new(new Point3D(anOracle.Real(-1, 1), anOracle.Real(-1, 1), 10), new Direction3D(0, 0, 1));
            (Shape aWire, string aWK) = Projected(anOracle, aPlane, anOracle.Real(0.5, 3));
            (N.TopoDS_Shape[] aNIn, Shape[] aPIn) = Both(aWire, aTarget);
            Run($"case {aCase} {aWK} onto {aTK}", aNIn, aPIn, aDir, null);
        }
        Report("cylindrical");
        Assert.Empty(myDifferences);
    }

    [Fact]
    public void Conical_projections_from_a_point()
    {
        var anOracle = new Oracle(output, 20261112);
        foreach (int aCase in Tape.Budget(120))
        {
            (Shape aTarget, string aTK) = Target(anOracle);
            Point3D aCenter = new(anOracle.Real(-1, 1), anOracle.Real(-1, 1), anOracle.Real(14, 20));
            Frame3D aPlane = new(new Point3D(anOracle.Real(-1, 1), anOracle.Real(-1, 1), 10), new Direction3D(0, 0, 1));
            (Shape aWire, string aWK) = Projected(anOracle, aPlane, anOracle.Real(0.5, 2));
            (N.TopoDS_Shape[] aNIn, Shape[] aPIn) = Both(aWire, aTarget);
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
        Shape aBox = new MakeBox(new Point3D(-5, -5, -2), 10, 10, 4).Shape();
        Shape aPocket = new MakeBox(new Point3D(-2.5, -2.5, 0), 5, 5, 2).Shape();
        Shape aTarget = new Cut(aBox, aPocket).Shape();
        Wire aCircle = Wire(new MakeEdge(new Circle3D(new Frame3D(new Point3D(0, 0, 10), new Direction3D(0, 0, 1)), 3)).Edge());
        (N.TopoDS_Shape[] aNIn, Shape[] aPIn) = Both(aCircle, aTarget);
        Run("circle onto a pocketed box", aNIn, aPIn, new Direction3D(0, 0, -1), null);

        Edge aCircleEdge = new MakeEdge(new Circle3D(new Frame3D(new Point3D(0, 0, -5), new Direction3D(0, 0, 1)), 1)).Edge();
        (aNIn, aPIn) = Both(aCircleEdge, new MakeBox(1, 1, 1).Shape());
        Run("circle edge onto a unit box", aNIn, aPIn, new Direction3D(0, 0, 1), null);

        (aNIn, aPIn) = Both(Wire(new MakeEdge(new Circle3D(new Frame3D(), 1)).Edge()), new MakeSphere(50).Shape());
        Run("circle onto a sphere from its centre", aNIn, aPIn, null, new Point3D(0, 0, 0.5));

        // errors: a vertex projected, a target without faces, the point on the wire's first vertex
        Vertex aV = new MakeVertex(new Point3D(0, 0, 10)).Vertex();
        (aNIn, aPIn) = Both(aV, aTarget);
        Run("a vertex projected", aNIn, aPIn, new Direction3D(0, 0, -1), null);
        (aNIn, aPIn) = Both(aCircle, aCircle);
        Run("onto a wire", aNIn, aPIn, new Direction3D(0, 0, -1), null);
        (aNIn, aPIn) = Both(aCircle, aTarget);
        Explorer anExp = new(aCircle, ShapeType.Vertex);
        Run("from the first vertex", aNIn, aPIn, null, Tool.Pnt(ShapeCasts.Vertex(anExp.Current())));
        Report("build123d.NET cases");
        Assert.Empty(myDifferences);
    }
}
