// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Curves;
using Kerf.Engine.Construction.Filling;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Text;
using Kerf.Kernel.TestHarness;
using Xunit.Abstractions;

namespace Kerf.Kernel.Oracle.Tests;

/// <summary>
/// The port's BRepFill package statics (unit 5S1) against native OCCT's, which build123d.NET binds: <c>Face</c> and
/// <c>Shell</c> (build123d.NET's ruled faces, <c>MakeSurfaceFromCurves</c>), <c>Axe</c>, <c>ComputeACR</c> and
/// <c>InsertACR</c>. The inputs are built on the port's side with the public makers (so they are frozen, as callers
/// hold them), written as BRep text and read by native OCCT, so both sides run on the same shapes. Shapes are compared
/// through BRep text (byte for byte, then with the Free column masked, then canonically: decision 581), doubles bit for
/// bit, exceptions by type. The port's inputs are checked unchanged after every call, and its results frozen.
/// </summary>
public sealed class BRepFillOracleTests(ITestOutputHelper output) : IDisposable
{
    private readonly string myDirectory = Directory.CreateTempSubdirectory("occtnet-brepfill-").FullName;
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

    private sealed class Tally(string theWhat)
    {
        public int Cases;
        public int Identical;
        public int FreeMasked;
        public int Canonical;
        public int Different;
        public int Throws;
        public int Values;
        public int ValuesExact;
        public readonly List<string> Differences = [];

        public string Report() =>
            $"{theWhat}: {Cases} cases, {Identical} byte-identical, {FreeMasked} equal with the Free column masked, "
            + $"{Canonical} canonically identical, {Different} different; {Throws} throw on both sides (same type); "
            + $"{ValuesExact}/{Values} doubles bit-identical";
    }

    private void Finish(Tally theTally)
    {
        output.WriteLine(theTally.Report());
        foreach (string aD in theTally.Differences.Take(20))
        {
            output.WriteLine(aD);
        }
        Assert.Empty(theTally.Differences);
    }

    // The input compound written as BRep text and read back by both sides (the port's reading frozen, as callers hold
    // shapes), with its children as TopoDS_Iterator gives them. Reading the same text gives both sides bit-identical
    // inputs: BRep text writes locations with 15 significant digits, so a shape built in memory and its reading differ.
    private (List<N.TopoDS_Shape> N, List<Shape> P, Shape PInput) ReadBoth(in Shape theCompound)
    {
        string aFile = NewFile();
        Assert.True(BrepTools.Write(theCompound, aFile));
        var aNShape = new N.TopoDS_Shape();
        using (var aBuilder = new N.BRep_Builder())
        {
            Assert.True(N.BRepTools.Read(aNShape, aFile, aBuilder));
        }
        var aList = new List<N.TopoDS_Shape>();
        for (var anIt = new N.TopoDS_Iterator(aNShape); anIt.More(); anIt.Next())
        {
            aList.Add(anIt.Value());
        }
        GC.KeepAlive(aNShape); // native TopoDS_Iterator holds no reference to the shape it iterates (decision 1109)
        Assert.True(BrepTools.Read(out Shape aPShape, aFile));
        aPShape.TShape()!.FreezeGraph();
        var aPList = new List<Shape>();
        for (var anIt = new Iterator(aPShape); anIt.More(); anIt.Next())
        {
            aPList.Add(anIt.Value());
        }
        return (aList, aPList, aPShape);
    }

    private static Compound Holder(params Shape[] theShapes)
    {
        BrepBuilder B = new();
        Compound aC = new();
        B.MakeCompound(ref aC);
        foreach (Shape aS in theShapes)
        {
            B.Add(aC, aS);
        }
        return aC;
    }

    private void CompareShapes(Tally theTally, string theWhat, N.TopoDS_Shape theNative, in Shape thePort)
    {
        if (!thePort.IsNull && !thePort.TShape()!.IsFrozen)
        {
            theTally.Differences.Add($"{theWhat}: the port's result isn't frozen");
        }
        if (theNative.IsNull() || thePort.IsNull)
        {
            if (theNative.IsNull() != thePort.IsNull)
            {
                theTally.Different++;
                theTally.Differences.Add($"{theWhat}: null on one side only (native {theNative.IsNull()})");
            }
            else
            {
                theTally.Identical++;
            }
            return;
        }

        string aNFile = NewFile(), aPFile = NewFile();
        Assert.True(N.BRepTools.Write(theNative, aNFile));
        Assert.True(BrepTools.Write(thePort, aPFile));
        string aN = File.ReadAllText(aNFile, Encoding.Latin1), aP = File.ReadAllText(aPFile, Encoding.Latin1);
        BRepTextDiff aDiff = BRepTextComparison.Compare(aN, aP);
        if (aDiff.IsIdentical)
        {
            theTally.Identical++;
            return;
        }
        if (aDiff.IsIdenticalMaskingFree)
        {
            theTally.FreeMasked++;
            return;
        }

        Assert.True(BrepTools.Read(out Shape aNP, aNFile));
        Assert.True(BrepTools.Read(out Shape aPP, aPFile));
        CanonicalComparison aCanon = CanonicalShapeComparer.Compare(aNP, aPP, new CanonicalOptions { CheckValidity = false });
        if (aCanon.IsIdentical)
        {
            theTally.Canonical++;
            return;
        }
        theTally.Different++;
        theTally.Differences.Add($"{theWhat}: {aDiff}\n{aCanon}");
    }

    private static void CompareThrow(Tally theTally, string theWhat, N.OcctException? theN, Exception? theP)
    {
        if (theN is null && theP is null)
        {
            return;
        }
        if (theN is not null && theP is not null && theN.OcctType == global::Kerf.Engine.Foundation.OcctTypeNames.Of(theP.GetType()))
        {
            theTally.Throws++;
            return;
        }
        theTally.Differences.Add($"{theWhat}: native threw {theN?.OcctType ?? "nothing"} ({theN?.OcctMessage}), "
                                 + $"port {theP?.GetType().Name ?? "nothing"} ({theP?.Message})");
    }

    private static void CompareDouble(Tally theTally, string theWhat, double theN, double theP)
    {
        theTally.Values++;
        if (BitConverter.DoubleToInt64Bits(theN) == BitConverter.DoubleToInt64Bits(theP))
        {
            theTally.ValuesExact++;
            return;
        }
        theTally.Differences.Add($"{theWhat}: native {theN:R} port {theP:R}");
    }

    private static void Unchanged(Tally theTally, string theWhat, string theBefore, in Shape theInput)
    {
        if (theBefore != BRepTextComparison.Write(theInput))
        {
            theTally.Differences.Add($"{theWhat}: the port's input changed");
        }
    }

    // ---------------------------------------------------------------------------------------------- random inputs

    private static Placement Location(Oracle theO)
    {
        Transform3D aT = Transform3D.Rotation(theO.Ax1(), theO.Angle());
        aT = aT.WithTranslationPart(theO.Vec());
        return new Placement(aT);
    }

    private static NCollection_Array1<Point3D> Poles(Oracle theO, in Point3D theStart, int theNb, double theScale)
    {
        var aPoles = new NCollection_Array1<Point3D>(1, theNb);
        Point3D aP = theStart;
        for (int i = 1; i <= theNb; i++)
        {
            aPoles[i] = aP;
            aP = new Point3D(aP.X + theO.Real(-theScale, theScale), aP.Y + theO.Real(-theScale, theScale),
                            aP.Z + theO.Real(-theScale, theScale));
        }
        return aPoles;
    }

    // A random bounded curve and the range of an edge on it; theClosed asks for a closed one where the kind has it.
    private static Curve3D Curve(Oracle theO, int theKind, out double theU1, out double theU2, out bool theClosed)
    {
        theClosed = false;
        switch (theKind)
        {
            case 0:
                theU1 = theO.Real(-20, 20);
                theU2 = theU1 + theO.Real(1, 40);
                return new LineCurve3D(theO.Pnt(), theO.Dir());
            case 1:
            case 2:
                {
                    var aC = new CircleCurve3D(theO.Ax2(), theO.Real(1, 30));
                    if (theKind == 2 || theO.Int(0, 3) == 0)
                    {
                        theClosed = true;
                        theU1 = 0;
                        theU2 = 2 * Math.PI;
                    }
                    else
                    {
                        theU1 = theO.Real(-3, 3);
                        theU2 = theU1 + theO.Real(0.3, 6);
                    }
                    return aC;
                }
            case 3:
                {
                    double aMaj = theO.Real(2, 30);
                    var aE = new EllipseCurve3D(theO.Ax2(), aMaj, aMaj * theO.Real(0.2, 1));
                    if (theO.Int(0, 3) == 0)
                    {
                        theClosed = true;
                        theU1 = 0;
                        theU2 = 2 * Math.PI;
                    }
                    else
                    {
                        theU1 = theO.Real(-3, 3);
                        theU2 = theU1 + theO.Real(0.3, 6);
                    }
                    return aE;
                }
            case 4:
                {
                    int aDeg = theO.Int(1, 4);
                    theO.BSplineKnots(aDeg, false, out NCollection_Array1<double> aKnots, out NCollection_Array1<int> aMults);
                    int aNbPoles = -aDeg - 1;
                    for (int i = 1; i <= aMults.Length(); i++)
                    {
                        aNbPoles += aMults[i];
                    }
                    NCollection_Array1<Point3D> aPoles = Poles(theO, theO.Pnt(), aNbPoles, 10);
                    var aC = theO.Bool()
                                 ? new BSplineCurve3D(aPoles, theO.Weights(aNbPoles, true), aKnots, aMults, aDeg)
                                 : new BSplineCurve3D(aPoles, aKnots, aMults, aDeg);
                    theU1 = aC.FirstParameter;
                    theU2 = aC.LastParameter;
                    if (theO.Bool())
                    {
                        double aLen = theU2 - theU1;
                        theU1 += theO.Real(0, 0.3) * aLen;
                        theU2 -= theO.Real(0, 0.3) * aLen;
                    }
                    return aC;
                }
            case 5:
                {
                    int aNb = theO.Int(2, 6);
                    NCollection_Array1<Point3D> aPoles = Poles(theO, theO.Pnt(), aNb, 10);
                    var aC = theO.Bool() ? new BezierCurve3D(aPoles, theO.Weights(aNb, true)) : new BezierCurve3D(aPoles);
                    theU1 = 0;
                    theU2 = 1;
                    return aC;
                }
            default:
                {
                    // a trimmed curve of another kind
                    Curve3D aBasis = Curve(theO, theO.Int(0, 4), out double a1, out double a2, out _);
                    var aT = new TrimmedCurve3D(aBasis, a1, a2);
                    theU1 = aT.FirstParameter;
                    theU2 = aT.LastParameter;
                    return aT;
                }
        }
    }

    private static Edge Edge(Oracle theO, int theKind, out bool theClosed)
    {
        Curve3D aC = Curve(theO, theKind, out double aU1, out double aU2, out theClosed);
        return new MakeEdge(aC, aU1, aU2).Edge();
    }

    private static Shape Dressed(Oracle theO, in Shape theShape, Placement? theLoc)
    {
        Shape aS = theShape;
        if (theLoc is not null)
        {
            aS = aS.Moved(theLoc.Value);
        }
        if (theO.Int(0, 3) == 0)
        {
            aS = aS.Reversed();
        }
        return aS;
    }

    // ----------------------------------------------------------------------------------------------------- Face

    [Fact]
    public void Face_MatchesNative()
    {
        var anO = new Oracle(output, 5851);
        var aTally = new Tally("BRepFill::Face");
        var aKinds = new Dictionary<string, int>();
        foreach (int aCase in Tape.Budget(700))
        {
            Edge aE1, aE2;
            string aKind;
            int aMode = anO.Int(0, 9);
            if (aMode == 0)
            {
                // edges sharing their first vertex: the ruled face degenerates there
                var aA = new MakeVertex(anO.Pnt()).Vertex();
                var aB = new MakeVertex(anO.Pnt()).Vertex();
                var aC = new MakeVertex(anO.Pnt()).Vertex();
                aE1 = new MakeEdge(aA, aB).Edge();
                aE2 = new MakeEdge(aA, aC).Edge();
                aKind = "shared vertex";
            }
            else if (aMode == 1)
            {
                // two edges sharing both vertices (a lens)
                var aA = new MakeVertex(anO.Pnt()).Vertex();
                var aB = new MakeVertex(anO.Pnt()).Vertex();
                aE1 = new MakeEdge(aA, aB).Edge();
                Point3D aPA = Tool.Pnt(aA), aPB = Tool.Pnt(aB);
                Point3D aMid = new((aPA.X + aPB.X) / 2 + anO.Real(1, 5), (aPA.Y + aPB.Y) / 2, (aPA.Z + aPB.Z) / 2 + 3);
                var aArc = new MakeArcOfCircle(aPA, aMid, aPB).Value();
                aE2 = new MakeEdge(aArc, aA, aB).Edge();
                aKind = "shared ends";
            }
            else if (aMode == 2)
            {
                aE1 = Edge(anO, 2, out _);
                aE2 = Edge(anO, 2, out _);
                aKind = "closed";
            }
            else
            {
                int aK1 = anO.Int(0, 6), aK2 = anO.Bool() ? aK1 : anO.Int(0, 6);
                aE1 = Edge(anO, aK1, out _);
                aE2 = Edge(anO, aK2, out _);
                aKind = $"kinds {aK1} {aK2}";
            }
            Placement? aL1 = null, aL2 = null;
            int aLoc = anO.Int(0, 3);
            if (aLoc == 1)
            {
                aL1 = aL2 = Location(anO);
            }
            else if (aLoc == 2)
            {
                aL1 = Location(anO);
                aL2 = anO.Bool() ? Location(anO) : null;
            }
            aE1 = ShapeCasts.Edge(Dressed(anO, aE1, aL1));
            aE2 = ShapeCasts.Edge(Dressed(anO, aE2, aL2));
            aKinds[aKind.Split(' ')[0]] = aKinds.GetValueOrDefault(aKind.Split(' ')[0]) + 1;

            (List<N.TopoDS_Shape> aN, List<Shape> aP, Shape anInput) = ReadBoth(Holder(aE1, aE2));
            aE1 = ShapeCasts.Edge(aP[0]);
            aE2 = ShapeCasts.Edge(aP[1]);
            string aBefore = BRepTextComparison.Write(anInput);
            string aWhat = $"Face case {aCase} ({aKind})";
            aTally.Cases++;

            N.TopoDS_Shape? aNR = null;
            Shape? aPR = null;
            N.OcctException? aNEx = null;
            Exception? aPEx = null;
            try
            {
                aNR = N.BRepFill.Face(N.TopoDS.Edge(aN[0]), N.TopoDS.Edge(aN[1]));
            }
            catch (N.OcctException anEx)
            {
                aNEx = anEx;
            }
            try
            {
                aPR = Fill.Face(aE1, aE2);
            }
            catch (Exception anEx) when (Refusal.IsFailure(anEx))
            {
                aPEx = anEx;
            }
            Unchanged(aTally, aWhat, aBefore, anInput);
            if (aNEx is not null || aPEx is not null)
            {
                CompareThrow(aTally, aWhat, aNEx, aPEx);
                continue;
            }
            CompareShapes(aTally, aWhat, aNR!, aPR!.Value);
        }
        output.WriteLine(string.Join(", ", aKinds.OrderBy(k => k.Key).Select(k => $"{k.Key} {k.Value}")));
        Finish(aTally);
    }

    // ---------------------------------------------------------------------------------------------------- Shell

    private static Point3D[] PolygonPoints(Oracle theO, int theNb)
    {
        CoordinateSystem anAx = theO.Ax3();
        double aR = theO.Real(5, 30);
        var aPnts = new Point3D[theNb];
        for (int i = 0; i < theNb; i++)
        {
            double anA = 2 * Math.PI * (i + theO.Real(0, 0.6)) / theNb;
            double aRi = aR * theO.Real(0.6, 1.2);
            aPnts[i] = new Point3D(anAx.Location.XYZ() + anAx.XDirection.XYZ() * (aRi * Math.Cos(anA))
                                  + anAx.YDirection.XYZ() * (aRi * Math.Sin(anA)) + anAx.Direction.XYZ() * theO.Real(-2, 2));
        }
        return aPnts;
    }

    private static Wire Wire(Oracle theO, int theKind, int theNb, bool theClosed)
    {
        Point3D[] aPnts = PolygonPoints(theO, theNb + (theClosed ? 0 : 1));
        switch (theKind)
        {
            case 0:
                {
                    var aMP = new MakePolygon();
                    foreach (Point3D aP in aPnts)
                    {
                        aMP.Add(aP);
                    }
                    if (theClosed)
                    {
                        aMP.Close();
                    }
                    return aMP.Wire();
                }
            case 1:
                {
                    // arcs of one circle (a closed wire of theNb arcs) or a chain of arcs through the points
                    var aMW = new MakeWire();
                    if (theClosed)
                    {
                        var aC = new CircleCurve3D(theO.Ax2(), theO.Real(3, 30));
                        double a0 = theO.Real(-1, 1);
                        Vertex aFirst = new MakeVertex(aC.Value(a0)).Vertex(), aPrev = aFirst;
                        for (int i = 1; i <= theNb; i++)
                        {
                            double a1 = a0 + 2 * Math.PI * (i - 1) / theNb, a2 = a0 + 2 * Math.PI * i / theNb;
                            Vertex aNext = i == theNb ? aFirst : new MakeVertex(aC.Value(a2)).Vertex();
                            aMW.Add(new MakeEdge(aC, aPrev, aNext, a1, a2).Edge());
                            aPrev = aNext;
                        }
                    }
                    else
                    {
                        for (int i = 0; i + 1 < aPnts.Length; i++)
                        {
                            Point3D aM = new((aPnts[i].X + aPnts[i + 1].X) / 2 + theO.Real(-2, 2),
                                            (aPnts[i].Y + aPnts[i + 1].Y) / 2 + theO.Real(-2, 2),
                                            (aPnts[i].Z + aPnts[i + 1].Z) / 2 + theO.Real(-2, 2));
                            aMW.Add(new MakeEdge(new MakeArcOfCircle(aPnts[i], aM, aPnts[i + 1]).Value()).Edge());
                        }
                    }
                    return aMW.Wire();
                }
            default:
                {
                    // a chain of Bezier curves through the points
                    var aMW = new MakeWire();
                    int aNbE = theClosed ? aPnts.Length : aPnts.Length - 1;
                    for (int i = 0; i < aNbE; i++)
                    {
                        Point3D a1 = aPnts[i], a2 = aPnts[(i + 1) % aPnts.Length];
                        var aPoles = new NCollection_Array1<Point3D>(1, 4);
                        aPoles[1] = a1;
                        aPoles[2] = new Point3D(a1.X + theO.Real(-3, 3), a1.Y + theO.Real(-3, 3), a1.Z + theO.Real(-3, 3));
                        aPoles[3] = new Point3D(a2.X + theO.Real(-3, 3), a2.Y + theO.Real(-3, 3), a2.Z + theO.Real(-3, 3));
                        aPoles[4] = a2;
                        aMW.Add(new MakeEdge(new BezierCurve3D(aPoles)).Edge());
                    }
                    return aMW.Wire();
                }
        }
    }

    [Fact]
    public void Shell_MatchesNative()
    {
        Tape.AcceptOtherArguments(Tape.WireKeepsCallersEdges); // plan 3.2's second level (unit 10H3)
        var anO = new Oracle(output, 5852);
        var aTally = new Tally("BRepFill::Shell");
        foreach (int aCase in Tape.Budget(300))
        {
            int aNb = anO.Int(1, 6);
            bool aClosed1 = anO.Int(0, 2) != 0, aClosed2 = anO.Int(0, 4) == 0 ? !aClosed1 : aClosed1;
            if (aNb < 3 && (aClosed1 || aClosed2))
            {
                aNb = 3;
            }
            int aK1 = anO.Int(0, 2), aK2 = anO.Bool() ? aK1 : anO.Int(0, 2);
            Wire aW1 = Wire(anO, aK1, aNb, aClosed1);
            Wire aW2 = Wire(anO, aK2, aNb, aClosed2);
            Placement? aL1 = null, aL2 = null;
            int aLoc = anO.Int(0, 3);
            if (aLoc == 1)
            {
                aL1 = aL2 = Location(anO);
            }
            else if (aLoc == 2)
            {
                aL1 = Location(anO);
            }
            aW1 = ShapeCasts.Wire(Dressed(anO, aW1, aL1));
            aW2 = ShapeCasts.Wire(Dressed(anO, aW2, aL2));

            (List<N.TopoDS_Shape> aN, List<Shape> aP, Shape anInput) = ReadBoth(Holder(aW1, aW2));
            aW1 = ShapeCasts.Wire(aP[0]);
            aW2 = ShapeCasts.Wire(aP[1]);
            string aBefore = BRepTextComparison.Write(anInput);
            string aWhat = $"Shell case {aCase} (kinds {aK1} {aK2}, {aNb} edges, closed {aClosed1} {aClosed2})";
            aTally.Cases++;

            N.TopoDS_Shape? aNR = null;
            Shape? aPR = null;
            N.OcctException? aNEx = null;
            Exception? aPEx = null;
            try
            {
                aNR = N.BRepFill.Shell(N.TopoDS.Wire(aN[0]), N.TopoDS.Wire(aN[1]));
            }
            catch (N.OcctException anEx)
            {
                aNEx = anEx;
            }
            try
            {
                aPR = Fill.Shell(aW1, aW2);
            }
            catch (Exception anEx) when (Refusal.IsFailure(anEx))
            {
                aPEx = anEx;
            }
            Unchanged(aTally, aWhat, aBefore, anInput);
            if (aNEx is not null || aPEx is not null)
            {
                CompareThrow(aTally, aWhat, aNEx, aPEx);
                continue;
            }
            CompareShapes(aTally, aWhat, aNR!, aPR!.Value);
        }
        Finish(aTally);
    }

    // ------------------------------------------------------------------------------------------------------ Axe

    [Fact]
    public void Axe_MatchesNative()
    {
        var anO = new Oracle(output, 5853);
        var aTally = new Tally("BRepFill::Axe");
        var aModes = new int[3];
        foreach (int aCase in Tape.Budget(300))
        {
            CoordinateSystem anAx = anO.Ax3();
            int aNb = anO.Int(3, 7);
            double aR = anO.Real(5, 30);
            var aMP = new MakePolygon();
            var aSpinePnts = new Point3D[aNb];
            for (int i = 0; i < aNb; i++)
            {
                double anA = 2 * Math.PI * (i + anO.Real(0, 0.5)) / aNb;
                aSpinePnts[i] = new Point3D(anAx.Location.XYZ() + anAx.XDirection.XYZ() * (aR * Math.Cos(anA))
                                           + anAx.YDirection.XYZ() * (aR * Math.Sin(anA)));
                aMP.Add(aSpinePnts[i]);
            }
            aMP.Close();
            Shape aSpine = aMP.Wire();
            if (anO.Int(0, 9) == 0)
            {
                // a face that isn't planar: OCCT throws Standard_NoSuchObject
                aSpine = new MakeFace(new CylindricalSurface(anAx, aR), 0, anO.Real(1, 6), 0, anO.Real(1, 20), 1e-7).Face();
            }
            else if (anO.Bool())
            {
                aSpine = new MakeFace(aMP.Wire(), true).Face();
            }

            // the profile: from a spine vertex, from a point on a spine edge, or away from the spine
            int aMode = anO.Int(0, 2);
            aModes[aMode]++;
            Point3D aStart;
            if (aMode == 0)
            {
                aStart = aSpinePnts[anO.Int(0, aNb - 1)];
            }
            else
            {
                int i = anO.Int(0, aNb - 1);
                double t = anO.Real(0.1, 0.9);
                Point3D a = aSpinePnts[i], b = aSpinePnts[(i + 1) % aNb];
                aStart = new Point3D(a.XYZ() * (1 - t) + b.XYZ() * t);
                if (aMode == 2)
                {
                    aStart = new Point3D(aStart.XYZ() + anAx.Direction.XYZ() * anO.Real(0.5, 5));
                }
            }
            var aPP = new MakePolygon();
            aPP.Add(aStart);
            int aNbP = anO.Int(1, 3);
            for (int i = 0; i < aNbP; i++)
            {
                aPP.Add(new Point3D(aStart.X + anO.Real(-5, 5), aStart.Y + anO.Real(-5, 5), aStart.Z + anO.Real(-5, 5)));
            }
            Wire aProfile = aPP.Wire();
            double aTol = anO.Int(0, 2) switch { 0 => 1e-7, 1 => 1e-3, _ => 1.0 };

            (List<N.TopoDS_Shape> aN, List<Shape> aP, Shape anInput) = ReadBoth(Holder(aSpine, aProfile));
            aSpine = aP[0];
            aProfile = ShapeCasts.Wire(aP[1]);
            string aBefore = BRepTextComparison.Write(anInput);
            string aWhat = $"Axe case {aCase} (mode {aMode}, {Kerf.Engine.Foundation.OcctTypeNames.Of(aSpine.Type)})";
            aTally.Cases++;

            using var aNAx = new N.gp_Ax3();
            bool aNOn = false;
            CoordinateSystem aPAx = new();
            bool aPOn = false;
            N.OcctException? aNEx = null;
            Exception? aPEx = null;
            try
            {
                N.BRepFill.Axe(aN[0], N.TopoDS.Wire(aN[1]), aNAx, ref aNOn, aTol);
            }
            catch (N.OcctException anEx)
            {
                aNEx = anEx;
            }
            try
            {
                Fill.Axe(aSpine, aProfile, out aPAx, out aPOn, aTol);
            }
            catch (Exception anEx) when (Refusal.IsFailure(anEx))
            {
                aPEx = anEx;
            }
            Unchanged(aTally, aWhat, aBefore, anInput);
            if (aNEx is not null || aPEx is not null)
            {
                CompareThrow(aTally, aWhat, aNEx, aPEx);
                continue;
            }
            aTally.Identical++;
            if (aNOn != aPOn)
            {
                aTally.Differences.Add($"{aWhat}: ProfOnSpine native {aNOn} port {aPOn}");
            }
            using N.gp_Pnt aNL = aNAx.Location();
            using N.gp_Dir aND = aNAx.Direction(), aNX = aNAx.XDirection(), aNY = aNAx.YDirection();
            Point3D aPL = aPAx.Location;
            Direction3D aPD = aPAx.Direction, aPX = aPAx.XDirection, aPY = aPAx.YDirection;
            CompareDouble(aTally, aWhat + " Loc.X", aNL.X(), aPL.X);
            CompareDouble(aTally, aWhat + " Loc.Y", aNL.Y(), aPL.Y);
            CompareDouble(aTally, aWhat + " Loc.Z", aNL.Z(), aPL.Z);
            CompareDouble(aTally, aWhat + " Dir.X", aND.X(), aPD.X);
            CompareDouble(aTally, aWhat + " Dir.Y", aND.Y(), aPD.Y);
            CompareDouble(aTally, aWhat + " Dir.Z", aND.Z(), aPD.Z);
            CompareDouble(aTally, aWhat + " XDir.X", aNX.X(), aPX.X);
            CompareDouble(aTally, aWhat + " XDir.Y", aNX.Y(), aPX.Y);
            CompareDouble(aTally, aWhat + " XDir.Z", aNX.Z(), aPX.Z);
            CompareDouble(aTally, aWhat + " YDir.X", aNY.X(), aPY.X);
            CompareDouble(aTally, aWhat + " YDir.Y", aNY.Y(), aPY.Y);
            CompareDouble(aTally, aWhat + " YDir.Z", aNY.Z(), aPY.Z);
        }
        output.WriteLine($"profile from a vertex {aModes[0]}, from an edge {aModes[1]}, away {aModes[2]}");
        Finish(aTally);
    }

    // ------------------------------------------------------------------------------------- ComputeACR, InsertACR

    private static Wire AcrWire(Oracle theO, out int theNbEdges)
    {
        int aNb = theO.Int(1, 6);
        bool aClosed = aNb >= 3 && theO.Bool();
        Wire aW = Wire(theO, theO.Int(0, 2), aNb, aClosed);
        theNbEdges = 0;
        for (var anExp = new WireExplorer(aW); anExp.More(); anExp.Next())
        {
            theNbEdges++;
        }
        Placement? aL = theO.Int(0, 3) == 0 ? Location(theO) : null;
        return ShapeCasts.Wire(Dressed(theO, aW, aL));
    }

    [Fact]
    public void ComputeACR_MatchesNative()
    {
        Tape.AcceptOtherArguments(Tape.WireKeepsCallersEdges); // plan 3.2's second level (unit 10H3)
        var anO = new Oracle(output, 5854);
        var aTally = new Tally("BRepFill::ComputeACR");
        foreach (int aCase in Tape.Budget(300))
        {
            Wire aW = AcrWire(anO, out int aNbEdges);
            (List<N.TopoDS_Shape> aN, List<Shape> aP, Shape anInput) = ReadBoth(Holder(aW));
            aW = ShapeCasts.Wire(aP[0]);
            string aBefore = BRepTextComparison.Write(anInput);
            string aWhat = $"ComputeACR case {aCase} ({aNbEdges} edges)";
            aTally.Cases++;
            using var aNACR = new N.TColStd_Array1OfReal(0, aNbEdges);
            var aPACR = new NCollection_Array1<double>(0, aNbEdges);
            N.BRepFill.ComputeACR(N.TopoDS.Wire(aN[0]), aNACR);
            Fill.ComputeACR(aW, aPACR);
            Unchanged(aTally, aWhat, aBefore, anInput);
            aTally.Identical++;
            for (int i = 0; i <= aNbEdges; i++)
            {
                CompareDouble(aTally, $"{aWhat} ACR({i})", aNACR.Value(i), aPACR[i]);
            }
        }
        Finish(aTally);
    }

    [Fact]
    public void InsertACR_MatchesNative()
    {
        Tape.AcceptOtherArguments(Tape.WireKeepsCallersEdges); // plan 3.2's second level (unit 10H3)
        var anO = new Oracle(output, 5855);
        var aTally = new Tally("BRepFill::InsertACR");
        int aCut = 0;
        foreach (int aCase in Tape.Budget(300))
        {
            Wire aW = AcrWire(anO, out int aNbEdges);
            int aNbCuts = anO.Int(1, 6);
            var aCuts = new double[aNbCuts];
            var aACR = new NCollection_Array1<double>(0, aNbEdges);
            Fill.ComputeACR(aW, aACR);
            for (int i = 0; i < aNbCuts; i++)
            {
                // random abscissae, some on the edges' ends
                aCuts[i] = anO.Int(0, 4) == 0 ? aACR[anO.Int(1, aNbEdges)] : anO.Real(0, 1);
            }
            Array.Sort(aCuts);
            var aPCuts = new NCollection_Array1<double>(1, aNbCuts);
            for (int i = 0; i < aNbCuts; i++)
            {
                aPCuts[i + 1] = aCuts[i];
            }
            double aPrec = anO.Int(0, 2) switch { 0 => 1e-9, 1 => 1e-4, _ => 0.05 };

            (List<N.TopoDS_Shape> aN, List<Shape> aP, Shape anInput) = ReadBoth(Holder(aW));
            aW = ShapeCasts.Wire(aP[0]);
            string aBefore = BRepTextComparison.Write(anInput);
            string aWhat = $"InsertACR case {aCase} ({aNbEdges} edges, {aNbCuts} cuts, prec {aPrec})";
            aTally.Cases++;

            N.TopoDS_Shape? aNR = null;
            Shape? aPR = null;
            N.OcctException? aNEx = null;
            Exception? aPEx = null;
            using N.TColStd_Array1OfReal aNCuts = Oracle.ToNative(aPCuts);
            try
            {
                aNR = N.BRepFill.InsertACR(N.TopoDS.Wire(aN[0]), aNCuts, aPrec);
            }
            catch (N.OcctException anEx)
            {
                aNEx = anEx;
            }
            try
            {
                aPR = Fill.InsertACR(aW, aPCuts, aPrec);
            }
            catch (Exception anEx) when (Refusal.IsFailure(anEx))
            {
                aPEx = anEx;
            }
            Unchanged(aTally, aWhat, aBefore, anInput);
            if (aNEx is not null || aPEx is not null)
            {
                CompareThrow(aTally, aWhat, aNEx, aPEx);
                continue;
            }
            int aNbResult = 0;
            for (var anExp = new Explorer(aPR!.Value, ShapeType.Edge); anExp.More(); anExp.Next())
            {
                aNbResult++;
            }
            if (aNbResult > aNbEdges)
            {
                aCut++;
            }
            CompareShapes(aTally, aWhat, aNR!, aPR!.Value);
        }
        output.WriteLine($"{aCut} wires cut");
        Finish(aTally);
    }
}
