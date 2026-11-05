// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Xunit.Abstractions;
using static Kerf.Geometry.CurveKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.Oracle.Tests;

/// <summary>
/// BRepTools_WireExplorer, BRepAdaptor_Curve, BRepAdaptor_Curve2d, BRepAdaptor_CompCurve, BRepLProp::Continuity and
/// BRepLProp_SLProps against native OCCT, on shapes built identically on both sides (<see cref="TwinShapes"/>):
/// the topology oracle's random scenes (boxes, cylinder faces with seams and located geometry, faces with holes,
/// compounds sharing sub-shapes at locations and orientations), planar faces whose edges have only pcurves, and open
/// or closed wires mixing B-spline, circle and line edges, located and reversed. The port's shapes are frozen first.
/// </summary>
public class BRepAdaptorOracleTests(ITestOutputHelper output)
{
    private const int Scenes = 48;

    private static readonly Continuity[] theShapes =
        [Continuity.C0, Continuity.C1, Continuity.C2, Continuity.C3, Continuity.CN];

    private sealed class Counts
    {
        public int Wires;
        public int Edges;
        public int EdgeFaces;
        public int Faces;
        public int Junctions;
    }

    [Fact]
    public void Adaptors_on_random_shapes_evaluate_as_native_OCCT()
    {
        var anOracle = new Oracle(output);
        var aCounts = new Counts();
        foreach (int aScene in Tape.Budget(Scenes))
        {
            Twin<N.TopoDS_Shape, Shape> aShape = (aScene % 6) switch
            {
                4 => PCurveScene(anOracle),
                5 => WireScene(anOracle),
                _ => TopologyOracleTests.RandomScene(anOracle, aScene),
            };
            aShape.P.TShape()!.FreezeGraph();
            Compare(anOracle, aShape, aCounts);
        }

        output.WriteLine($"{Scenes} scenes: {aCounts.Wires} wires, {aCounts.Edges} edges, {aCounts.EdgeFaces} edge-face pairs, "
                         + $"{aCounts.Faces} faces, {aCounts.Junctions} junctions");
        anOracle.Report("BRepTools_WireExplorer, BRepAdaptor_Curve/Curve2d/CompCurve, BRepLProp");
    }

    // Scenes

    /// <summary>A planar face on a random plane whose edges have only pcurves, at a random location, maybe reversed.</summary>
    private static Twin<N.TopoDS_Shape, Shape> PCurveScene(Oracle theOracle)
    {
        var aTwins = new TwinShapes();
        Point3D anOrigin = theOracle.Pnt();
        Xyz aNormal = GeomOracleInputs.RawDir(theOracle);
        Xyz anX = aNormal.Crossed(GeomOracleInputs.RawDir(theOracle));
        while (anX.Modulus() < 0.1)
        {
            anX = aNormal.Crossed(GeomOracleInputs.RawDir(theOracle));
        }

        double a = theOracle.Real(2, 8), b = theOracle.Real(2, 8);
        Twin<N.TopoDS_Shape, Shape> aFace = TwinShapes.Up(
            aTwins.PCurveOnlyFace(anOrigin, aNormal, anX, (0, 0), (a, theOracle.Real(-1, 1)), (a, b), (theOracle.Real(-1, 1), b)));
        return Placed(theOracle, aFace);
    }

    /// <summary>
    /// A wire of a line, a B-spline and a circular arc (closed by a fourth line half the time), the edges' order and
    /// orientations shuffled in the wire, at a random location.
    /// </summary>
    private static Twin<N.TopoDS_Shape, Shape> WireScene(Oracle theOracle)
    {
        var aTwins = new TwinShapes();

        // a B-spline curve from raw numbers
        int aDegree = theOracle.Int(2, 3);
        int aNbPoles = theOracle.Int(aDegree + 1, 7);
        var aPoles = new NCollection_Array1<Point3D>(1, aNbPoles);
        for (int i = 1; i <= aNbPoles; i++)
        {
            aPoles[i] = new Point3D(i * 2.0 + theOracle.Real(-0.5, 0.5), theOracle.Real(-2, 2), theOracle.Real(-2, 2));
        }

        int aNbKnots = aNbPoles - aDegree + 1;
        var aKnots = new NCollection_Array1<double>(1, aNbKnots);
        var aMults = new NCollection_Array1<int>(1, aNbKnots);
        double aK = theOracle.Real(-1, 1);
        for (int i = 1; i <= aNbKnots; i++)
        {
            aKnots[i] = aK;
            aK += theOracle.Real(0.3, 2);
            aMults[i] = i == 1 || i == aNbKnots ? aDegree + 1 : 1;
        }

        NCollection_Array1<double>? aWeights = theOracle.Bool() ? theOracle.Weights(aNbPoles, true) : null;
        N.Geom_BSplineCurve aNBS = Oracle.ToNativeCurve(aPoles, aWeights, aKnots, aMults, aDegree, false);
        BSplineCurve3D aBS = aWeights is null
            ? new BSplineCurve3D(aPoles, aKnots, aMults, aDegree)
            : new BSplineCurve3D(aPoles, aWeights, aKnots, aMults, aDegree);
        double f = aKnots[1], l = aKnots[aNbKnots];

        // the vertices where the port's curves put them (both sides use these coordinates)
        Point3D aP1 = aBS.Value(f), aP2 = aBS.Value(l);
        Point3D aP0 = new Point3D(aP1.X - theOracle.Real(1, 3), aP1.Y + theOracle.Real(-1, 1), aP1.Z);
        Twin<N.TopoDS_Vertex, Vertex> aV0 = aTwins.Vertex(aP0), aV1 = aTwins.Vertex(aP1, 1e-6), aV2 = aTwins.Vertex(aP2, 1e-6);
        Twin<N.TopoDS_Edge, Edge> aLine = aTwins.Segment(aV0, aP0, aV1, aP1);
        Twin<N.TopoDS_Edge, Edge> aSpline = aTwins.EdgeOn(aNBS, aBS, new Placement(), null, f, l, aV1, aV2);

        // an arc of a circle in a plane through aP2: centre aP2 + r X, from angle Pi
        double r = theOracle.Real(0.5, 3), anArc = theOracle.Real(0.5, 2.5);
        Point3D aCentre = new Point3D(aP2.X + r, aP2.Y, aP2.Z);
        using var aNC = Oracle.ToNative(aCentre);
        using var aNZ = new N.gp_Dir(0, 0, 1);
        using var aNXd = new N.gp_Dir(1, 0, 0);
        using var aNAx2 = new N.gp_Ax2(aNC, aNZ, aNXd);
        var aCircle = new CircleCurve3D(new Frame3D(aCentre, new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), r);
        Point3D aP3 = aCircle.Value(Math.PI + anArc);
        Twin<N.TopoDS_Vertex, Vertex> aV3 = aTwins.Vertex(aP3, 1e-6);
        Twin<N.TopoDS_Edge, Edge> anArcEdge =
            aTwins.EdgeOn(new N.Geom_Circle(aNAx2, r), aCircle, new Placement(), null, Math.PI, Math.PI + anArc, aV2, aV3);

        var anEdges = new List<Twin<N.TopoDS_Edge, Edge>> { aLine, aSpline, anArcEdge };
        bool isClosed = theOracle.Bool();
        if (isClosed)
        {
            anEdges.Add(aTwins.Segment(aV3, aP3, aV0, aP0));
        }

        // shuffle, and reverse some edges (a reversed edge in the wire is the chain's edge run backwards: the wire
        // explorer must then find the chain in the other direction, so reverse all or none)
        int aShift = theOracle.Int(0, anEdges.Count - 1);
        var aShuffled = new List<Twin<N.TopoDS_Edge, Edge>>();
        for (int i = 0; i < anEdges.Count; i++)
        {
            aShuffled.Add(anEdges[(i + aShift) % anEdges.Count]);
        }

        if (theOracle.Bool())
        {
            aShuffled = [.. aShuffled.Select(e => new Twin<N.TopoDS_Edge, Edge>(N.TopoDS.Edge(e.N.Reversed()), ShapeCasts.Edge(e.P.Reversed())))];
        }

        Twin<N.TopoDS_Wire, Wire> aWire = aTwins.Wire(isClosed, [.. aShuffled]);
        return Placed(theOracle, TwinShapes.Up(aWire));
    }

    private static Twin<N.TopoDS_Shape, Shape> Placed(Oracle theOracle, Twin<N.TopoDS_Shape, Shape> theShape)
    {
        Twin<N.TopoDS_Shape, Shape> aShape = theShape;
        if (theOracle.Bool())
        {
            (N.TopLoc_Location aNL, Placement aL) = TopologyOracleTests.RandomLocation(theOracle);
            aShape = new(aShape.N.Moved(aNL), aShape.P.Moved(aL));
        }

        return theOracle.Bool() ? new(aShape.N.Reversed(), aShape.P.Reversed()) : aShape;
    }

    // Comparisons

    private static void Compare(Oracle theOracle, Twin<N.TopoDS_Shape, Shape> theShape, Counts theCounts)
    {
        // edges alone
        foreach (Twin<N.TopoDS_Shape, Shape> anE in TopologyOracleTests.Explore(theShape, ShapeType.Edge))
        {
            theCounts.Edges++;
            N.TopoDS_Edge aNE = N.TopoDS.Edge(anE.N);
            Edge aPE = ShapeCasts.Edge(anE.P);
            using var aNC = new N.BRepAdaptor_Curve(aNE);
            var aPC = new EdgeCurveAdaptor(aPE);
            theOracle.Same(aNC.Is3DCurve(), aPC.Is3DCurve(), "Is3DCurve");
            theOracle.Same(aNC.Tolerance(), aPC.Tolerance(), "Tolerance");
            CompareCurve(theOracle, theOracle.Int(0, 1000), aNC, aPC, "BRepAdaptor_Curve");
            CompareTrim(theOracle, aNC, aPC);
            using N.Adaptor3d_Curve aNCopy = aNC.ShallowCopy()!;
            CompareCurve(theOracle, 1, aNCopy, aPC.ShallowCopy(), "BRepAdaptor_Curve.ShallowCopy");
        }

        // edges on faces
        foreach (Twin<N.TopoDS_Shape, Shape> aF in TopologyOracleTests.Explore(theShape, ShapeType.Face))
        {
            theCounts.Faces++;
            N.TopoDS_Face aNF = N.TopoDS.Face(aF.N);
            Face aPF = ShapeCasts.Face(aF.P);
            foreach (Twin<N.TopoDS_Shape, Shape> anE in TopologyOracleTests.Explore(aF, ShapeType.Edge))
            {
                theCounts.EdgeFaces++;
                N.TopoDS_Edge aNE = N.TopoDS.Edge(anE.N);
                Edge aPE = ShapeCasts.Edge(anE.P);
                using var aNC = new N.BRepAdaptor_Curve(aNE, aNF);
                var aPC = new EdgeCurveAdaptor(aPE, aPF);
                CompareCurve(theOracle, theOracle.Int(0, 1000), aNC, aPC, "BRepAdaptor_Curve(E, F)");
                CompareTrim(theOracle, aNC, aPC);
                using var aNC2 = new N.BRepAdaptor_Curve2d(aNE, aNF);
                var aPC2 = new EdgeCurve2DAdaptor(aPE, aPF);
                CompareCurve2d(theOracle, aNC2, aPC2);
            }

            CompareSurfaceProps(theOracle, aNF, aPF);
            foreach (Twin<N.TopoDS_Shape, Shape> aW in TopologyOracleTests.Explore(aF, ShapeType.Wire))
            {
                CompareWireExplorer(theOracle, aW, aF, theCounts);
            }
        }

        // wires (with no face), and the composite curves
        foreach (Twin<N.TopoDS_Shape, Shape> aW in TopologyOracleTests.Explore(theShape, ShapeType.Wire))
        {
            theCounts.Wires++;
            CompareWireExplorer(theOracle, aW, null, theCounts);
            CompareCompCurve(theOracle, aW);
        }
    }

    private static double[] Samples(Oracle theOracle, double f, double l, int theNb)
    {
        var aList = new List<double> { f, l, 0.5 * (f + l) };
        for (int i = 0; i < theNb; i++)
        {
            aList.Add(f + (l - f) * theOracle.Real(0, 1));
        }

        return [.. aList];
    }

    private static void CompareCurve(Oracle theOracle, int theSeed, N.Adaptor3d_Curve theN, CurveAdaptorBase theP, string theWhat)
    {
        double f = theN.FirstParameter(), l = theN.LastParameter();
        theOracle.Same(f, theP.FirstParameter(), theWhat + " First");
        theOracle.Same(l, theP.LastParameter(), theWhat + " Last");
        theOracle.Same((int)theN.Continuity(), (int)theP.Continuity(), theWhat + " Continuity");
        theOracle.Same((int)theN.GetType_(), (int)theP.GetType(), theWhat + " GetType");
        theOracle.Same(theN.IsClosed(), theP.IsClosed, theWhat + " IsClosed");
        theOracle.Same(theN.IsPeriodic(), theP.IsPeriodic, theWhat + " IsPeriodic");
        if (theN.IsPeriodic())
        {
            theOracle.Same(theN.Period(), theP.Period(), theWhat + " Period");
        }

        theOracle.SameRelative(theN.Resolution(1e-3), theP.Resolution(1e-3), theWhat + " Resolution");
        foreach (Continuity aS in theShapes)
        {
            int n = theN.NbIntervals((N.GeomAbs_Shape)(int)aS);
            theOracle.Same(n, theP.NbIntervals(aS), theWhat + " NbIntervals");
            using var aNT = new N.TColStd_Array1OfReal(1, n + 1);
            var aT = new NCollection_Array1<double>(1, n + 1);
            theN.Intervals(aNT, (N.GeomAbs_Shape)(int)aS);
            theP.Intervals(aT, aS);
            for (int i = 1; i <= n + 1; i++)
            {
                theOracle.Same(aNT.Value(i), aT[i], theWhat + " Intervals");
            }
        }

        switch (theP.GetType())
        {
            case Line:
            {
                using N.gp_Lin aNL = theN.Line();
                Line3D aL = theP.Line();
                using N.gp_Pnt aNLoc = aNL.Location();
                theOracle.Same(aNLoc, aL.Location, theWhat + " Line.Location");
                using N.gp_Dir aND = aNL.Direction();
                theOracle.Same(aND.X(), aL.Direction.X, theWhat + " Line.Direction");
                theOracle.Same(aND.Y(), aL.Direction.Y, theWhat + " Line.Direction");
                theOracle.Same(aND.Z(), aL.Direction.Z, theWhat + " Line.Direction");
                break;
            }
            case Circle:
            {
                using N.gp_Circ aNCi = theN.Circle();
                Circle3D aCi = theP.Circle();
                using N.gp_Pnt aNLoc = aNCi.Location();
                theOracle.Same(aNLoc, aCi.Location, theWhat + " Circle.Location");
                theOracle.Same(aNCi.Radius(), aCi.Radius, theWhat + " Circle.Radius");
                break;
            }
            case BSplineCurve:
            case BezierCurve:
                theOracle.Same(theN.Degree(), theP.Degree(), theWhat + " Degree");
                theOracle.Same(theN.NbPoles(), theP.PoleCount, theWhat + " NbPoles");
                theOracle.Same(theN.IsRational(), theP.IsRational, theWhat + " IsRational");
                break;
        }

        var aRandom = new Oracle(null!, theSeed);
        foreach (double u in Samples(aRandom, f, l, 4))
        {
            using var aNP = new N.gp_Pnt();
            using var aNV1 = new N.gp_Vec();
            using var aNV2 = new N.gp_Vec();
            using var aNV3 = new N.gp_Vec();
            using N.gp_Pnt aNP0 = theN.EvalD0(u);
            theOracle.Same(aNP0, theP.EvalD0(u), theWhat + " D0");
            theN.D1(u, aNP, aNV1);
            theP.D1(u, out Point3D aP, out Vector3D aV1);
            theOracle.Same(aNP, aP, theWhat + " D1.P");
            theOracle.Same(aNV1, aV1, theWhat + " D1.V1");
            theN.D2(u, aNP, aNV1, aNV2);
            theP.D2(u, out aP, out aV1, out Vector3D aV2);
            theOracle.Same(aNP, aP, theWhat + " D2.P");
            theOracle.Same(aNV1, aV1, theWhat + " D2.V1");
            theOracle.Same(aNV2, aV2, theWhat + " D2.V2");
            theN.D3(u, aNP, aNV1, aNV2, aNV3);
            theP.D3(u, out aP, out aV1, out aV2, out Vector3D aV3);
            theOracle.Same(aNP, aP, theWhat + " D3.P");
            theOracle.Same(aNV1, aV1, theWhat + " D3.V1");
            theOracle.Same(aNV2, aV2, theWhat + " D3.V2");
            theOracle.Same(aNV3, aV3, theWhat + " D3.V3");
            for (int n = 1; n <= 3; n++)
            {
                using N.gp_Vec aNDN = theN.EvalDN(u, n);
                theOracle.Same(aNDN, theP.EvalDN(u, n), theWhat + " DN");
            }
        }
    }

    private static void CompareTrim(Oracle theOracle, N.Adaptor3d_Curve theN, CurveAdaptorBase theP)
    {
        double f = theN.FirstParameter(), l = theN.LastParameter();
        double a = f + (l - f) * theOracle.Real(0, 0.4), b = f + (l - f) * theOracle.Real(0.6, 1);
        using N.Adaptor3d_Curve aNT = theN.Trim(a, b, 1e-9)!;
        CompareCurve(theOracle, theOracle.Int(0, 1000), aNT, theP.Trim(a, b, 1e-9), "Trim");
    }

    private static void CompareCurve2d(Oracle theOracle, N.BRepAdaptor_Curve2d theN, EdgeCurve2DAdaptor theP)
    {
        double f = theN.FirstParameter(), l = theN.LastParameter();
        theOracle.Same(f, theP.FirstParameter(), "Curve2d First");
        theOracle.Same(l, theP.LastParameter(), "Curve2d Last");
        theOracle.Same((int)theN.GetType_(), (int)theP.GetType(), "Curve2d GetType");
        theOracle.Same((int)theN.Continuity(), (int)theP.Continuity(), "Curve2d Continuity");
        theOracle.Same(theN.IsPeriodic(), theP.IsPeriodic, "Curve2d IsPeriodic");
        theOracle.SameRelative(theN.Resolution(1e-3), theP.Resolution(1e-3), "Curve2d Resolution");
        foreach (double u in Samples(theOracle, f, l, 3))
        {
            using var aNP = new N.gp_Pnt2d();
            using var aNV1 = new N.gp_Vec2d();
            using var aNV2 = new N.gp_Vec2d();
            using var aNV3 = new N.gp_Vec2d();
            theN.D3(u, aNP, aNV1, aNV2, aNV3);
            theP.D3(u, out Point2D aP, out Vector2D aV1, out Vector2D aV2, out Vector2D aV3);
            theOracle.Same(aNP, aP, "Curve2d D3.P");
            theOracle.Same(aNV1, aV1, "Curve2d D3.V1");
            theOracle.Same(aNV2, aV2, "Curve2d D3.V2");
            theOracle.Same(aNV3, aV3, "Curve2d D3.V3");
        }

        using N.Adaptor2d_Curve2d aNCopy = theN.ShallowCopy()!;
        Curve2DAdaptorBase aCopy = theP.ShallowCopy();
        double m = 0.5 * (f + l);
        using N.gp_Pnt2d aNM = aNCopy.Value(m);
        theOracle.Same(aNM, aCopy.Value(m), "Curve2d ShallowCopy");
    }

    /// <summary>The wire explorer's order, orientations and vertices, with or without the face.</summary>
    private static void CompareWireExplorer(Oracle theOracle, Twin<N.TopoDS_Shape, Shape> theWire,
                                            Twin<N.TopoDS_Shape, Shape>? theFace, Counts theCounts)
    {
        List<Twin<N.TopoDS_Shape, Shape>> anEdges = TopologyOracleTests.Explore(theWire, ShapeType.Edge);
        N.TopoDS_Wire aNW = N.TopoDS.Wire(theWire.N);
        Wire aPW = ShapeCasts.Wire(theWire.P);
        using N.BRepTools_WireExplorer aNExp = theFace is { } aF ? new N.BRepTools_WireExplorer(aNW, N.TopoDS.Face(aF.N)) : new N.BRepTools_WireExplorer(aNW);
        WireExplorer anExp = theFace is { } aF2 ? new WireExplorer(aPW, ShapeCasts.Face(aF2.P)) : new WireExplorer(aPW);
        N.BRepAdaptor_Curve? aNPrev = null;
        EdgeCurveAdaptor? aPrev = null;
        for (; anExp.More(); anExp.Next(), aNExp.Next())
        {
            theOracle.Same(true, aNExp.More(), "WireExplorer.More");
            N.TopoDS_Edge aNE = aNExp.Current();
            Edge aPE = anExp.Current();
            int anIndex = anEdges.FindIndex(e => e.N.IsEqual(aNE));
            Assert.True(anIndex >= 0);
            theOracle.Same(true, anEdges[anIndex].P.IsEqual(aPE), "WireExplorer.Current");
            theOracle.Same((int)aNExp.Orientation(), (int)anExp.Orientation(), "WireExplorer.Orientation");
            using N.TopoDS_Vertex aNV = aNExp.CurrentVertex();
            Vertex aV = anExp.CurrentVertex();
            theOracle.Same(aNV.IsNull(), aV.IsNull, "WireExplorer.CurrentVertex is null");
            if (!aV.IsNull)
            {
                using N.gp_Pnt aNPnt = N.BRep_Tool.Pnt(aNV);
                theOracle.Same(aNPnt, Tool.Pnt(aV), "WireExplorer.CurrentVertex");
            }

            // BRepLProp::Continuity at the junction with the previous edge (wires without a face only)
            if (theFace is null && !N.BRep_Tool.Degenerated(aNE))
            {
                var aNC = new N.BRepAdaptor_Curve(aNE);
                var aPC = new EdgeCurveAdaptor(aPE);
                if (aPrev is not null)
                {
                    theCounts.Junctions++;
                    double u1 = aPrev.Edge().Orientation == Reversed ? aPrev.FirstParameter() : aPrev.LastParameter();
                    double u2 = aPE.Orientation == Reversed ? aPC.LastParameter() : aPC.FirstParameter();
                    bool isNThrow = false, isPThrow = false;
                    N.GeomAbs_Shape aNCont = N.GeomAbs_Shape.GeomAbs_C0;
                    Continuity aPCont = Continuity.C0;
                    try
                    {
                        aNCont = N.BRepLProp.Continuity(aNPrev!, aNC, u1, u2);
                    }
                    catch (Exception)
                    {
                        isNThrow = true;
                    }

                    try
                    {
                        aPCont = global::Kerf.Engine.Topology.Properties.BRepLProp.Continuity(aPrev, aPC, u1, u2);
                    }
                    catch (Exception caught) when (Refusal.IsFailure(caught))
                    {
                        isPThrow = true;
                    }

                    theOracle.Same(isNThrow, isPThrow, "BRepLProp::Continuity throws");
                    theOracle.Same((int)aNCont, (int)aPCont, "BRepLProp::Continuity");
                }

                aNPrev?.Dispose();
                aNPrev = aNC;
                aPrev = aPC;
            }
        }

        aNPrev?.Dispose();
        theOracle.Same(false, aNExp.More(), "WireExplorer.More at the end");
    }

    private static void CompareCompCurve(Oracle theOracle, Twin<N.TopoDS_Shape, Shape> theWire)
    {
        N.TopoDS_Wire aNW = N.TopoDS.Wire(theWire.N);
        Wire aPW = ShapeCasts.Wire(theWire.P);
        // a wire of only degenerated edges has no curve
        bool hasEdge = false;
        for (var anExp = new WireExplorer(aPW); anExp.More(); anExp.Next())
        {
            hasEdge |= !Tool.Degenerated(anExp.Current());
        }

        if (!hasEdge)
        {
            return;
        }

        foreach (bool isAC in new[] { false, true })
        {
            using var aNC = new N.BRepAdaptor_CompCurve(aNW, isAC);
            var aPC = new WireCurveAdaptor(aPW, isAC);
            string aWhat = isAC ? "CompCurve(AC)" : "CompCurve";
            CompareCurve(theOracle, theOracle.Int(0, 1000), aNC, aPC, aWhat);
            CompareCompEdges(theOracle, aNC, aPC, aWhat);

            // trimmed, through Trim and through the trimming constructor
            double f = aNC.FirstParameter(), l = aNC.LastParameter();
            double a = f + (l - f) * theOracle.Real(0, 0.45), b = f + (l - f) * theOracle.Real(0.55, 1);
            using N.Adaptor3d_Curve aNT = aNC.Trim(a, b, 1e-7)!;
            CompareCurve(theOracle, theOracle.Int(0, 1000), aNT, aPC.Trim(a, b, 1e-7), aWhat + ".Trim");
            using var aNT2 = new N.BRepAdaptor_CompCurve(aNW, isAC, a, b, 1e-7);
            var aPT2 = new WireCurveAdaptor(aPW, isAC, a, b, 1e-7);
            CompareCurve(theOracle, theOracle.Int(0, 1000), aNT2, aPT2, aWhat + "(trimmed)");
            CompareCompEdges(theOracle, aNT2, aPT2, aWhat + "(trimmed)");

            using N.Adaptor3d_Curve aNCopy = aNC.ShallowCopy()!;
            CompareCurve(theOracle, 2, aNCopy, aPC.ShallowCopy(), aWhat + ".ShallowCopy");
        }
    }

    private static void CompareCompEdges(Oracle theOracle, N.BRepAdaptor_CompCurve theN, WireCurveAdaptor theP, string theWhat)
    {
        double f = theN.FirstParameter(), l = theN.LastParameter();
        var aParams = new List<double>(Samples(theOracle, f, l, 4));
        for (double k = Math.Ceiling(f); k <= l; k += 1.0)
        {
            aParams.Add(k);
        }

        foreach (double u in aParams)
        {
            var aNE = new N.TopoDS_Edge();
            double aNU = 0;
            theN.Edge(u, aNE, ref aNU);
            theP.Edge(u, out Edge aPE, out double aPU);
            theOracle.Same(aNU, aPU, theWhat + " Edge.UonE");
            using N.gp_Pnt aNP = N.BRep_Tool.Pnt(N.TopExp.FirstVertex(aNE, true));
            theOracle.Same(aNP, Tool.Pnt(Exploration.FirstVertex(aPE, true)), theWhat + " Edge");
            theOracle.Same((int)aNE.Orientation(), (int)aPE.Orientation, theWhat + " Edge.Orientation");
        }
    }

    /// <summary>BRepLProp_SLProps on the face's adaptor, at random parameters inside its UV bounds.</summary>
    private static void CompareSurfaceProps(Oracle theOracle, N.TopoDS_Face theNF, in Face theF)
    {
        using var aNS = new N.BRepAdaptor_Surface(theNF);
        var aS = new FaceSurfaceAdaptor(theF);
        double u1 = aS.FirstUParameter(), u2 = aS.LastUParameter(), v1 = aS.FirstVParameter(), v2 = aS.LastVParameter();
        for (int i = 0; i < 3; i++)
        {
            double u = u1 + (u2 - u1) * theOracle.Real(0, 1), v = v1 + (v2 - v1) * theOracle.Real(0, 1);
            int aN = theOracle.Int(0, 2);
            using var aNP = new N.BRepLProp_SLProps(aNS, u, v, aN, 1e-9);
            var aP = new global::Kerf.Engine.Topology.Properties.SLProps(aS, u, v, aN, 1e-9);
            using N.gp_Pnt aNV = aNP.Value();
            theOracle.Same(aNV, aP.Value(), "SLProps.Value");
            using N.gp_Vec aND1U = aNP.D1U();
            theOracle.Same(aND1U, aP.D1U(), "SLProps.D1U");
            using N.gp_Vec aND1V = aNP.D1V();
            theOracle.Same(aND1V, aP.D1V(), "SLProps.D1V");
            using N.gp_Vec aND2U = aNP.D2U();
            theOracle.Same(aND2U, aP.D2U(), "SLProps.D2U");
            using N.gp_Vec aNDUV = aNP.DUV();
            theOracle.Same(aNDUV, aP.DUV(), "SLProps.DUV");
            bool isN = aNP.IsNormalDefined();
            theOracle.Same(isN, aP.IsNormalDefined, "SLProps.IsNormalDefined");
            if (isN)
            {
                using N.gp_Dir aNN = aNP.Normal();
                Direction3D aPN = aP.Normal();
                theOracle.Same(aNN.X(), aPN.X, "SLProps.Normal");
                theOracle.Same(aNN.Y(), aPN.Y, "SLProps.Normal");
                theOracle.Same(aNN.Z(), aPN.Z, "SLProps.Normal");
            }

            bool isC = aNP.IsCurvatureDefined();
            theOracle.Same(isC, aP.IsCurvatureDefined(), "SLProps.IsCurvatureDefined");
            if (isC)
            {
                theOracle.Same(aNP.MaxCurvature(), aP.MaxCurvature(), "SLProps.MaxCurvature");
                theOracle.Same(aNP.MinCurvature(), aP.MinCurvature(), "SLProps.MinCurvature");
                theOracle.Same(aNP.MeanCurvature(), aP.MeanCurvature(), "SLProps.MeanCurvature");
                theOracle.Same(aNP.GaussianCurvature(), aP.GaussianCurvature(), "SLProps.GaussianCurvature");
                theOracle.Same(aNP.IsUmbilic(), aP.IsUmbilic(), "SLProps.IsUmbilic");
            }
        }
    }
}
