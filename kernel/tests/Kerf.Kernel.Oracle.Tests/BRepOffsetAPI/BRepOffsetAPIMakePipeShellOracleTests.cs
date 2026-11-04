// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Curves;
using Kerf.Engine.Construction.Filling;
using Kerf.Engine.Exchange.Brep;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Diagnostics;
using Kerf.Kernel.TestHarness;
using Xunit.Abstractions;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.Oracle.Tests;

/// <summary>
/// BRepOffsetAPI_MakePipeShell (and through it BRepFill_PipeShell, BRepFill_Sweep, BRepFill_TrimShellCorner and the
/// laws of unit 5S1) against native OCCT, unit 5S2.
/// </summary>
/// <remarks>
/// <para>Inputs built by the port and written to one BRep file that both sides read (the port's copy frozen, as a
/// caller's): random spines (polylines with corners, G1 chains of lines and tangent arcs, interpolated B-splines,
/// closed circles and polygons, helix-like B-splines), profiles in the plane normal to the spine at its start (circles,
/// ellipses, polygons, closed B-splines, open polylines and arcs), sometimes off the spine or tilted (with contact and
/// correction), several sections at the spine's vertices or along it, punctual sections (a vertex at an end).</para>
/// <para>Every trihedron mode: corrected Frenet (by default and through <c>SetMode(false)</c>), Frenet, discrete,
/// fixed, constant binormal, Darboux on a support (box faces, a cylinder's lateral face), an auxiliary spine
/// (curvilinear equivalence or plane, without contact, with contact, with contact on border); every transition
/// (transformed, right corner, round corner); shells and solids. Compared: exceptions, IsReady, GetStatus, IsDone,
/// ErrorOnSurface (bits), the result, FirstShape and LastShape as BRep text (the Free column masked, then
/// canonically), MakeSolid, Generated for every edge, vertex and wire of the profiles and the spine (as indices in the
/// result), Simulate's sections; the inputs left unchanged.</para>
/// </remarks>
public sealed class BRepOffsetAPIMakePipeShellOracleTests(ITestOutputHelper output) : IDisposable
{
    private readonly string myDirectory = Directory.CreateTempSubdirectory("occtnet-pipeshell-").FullName;
    private int myFileNumber;
    private readonly List<string> myDifferences = [];
    private int myCases, myBuilt, myNotDone, myExceptions, myIdentical, myFreeMasked, myCanonical, mySolids,
                myHistory, myHistoryOrder, mySimulated, myBoundsIdentical;

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

    // Writes the port's shapes in one compound and reads it on both sides (the port's frozen); the children in order
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
        for (global::Kerf.Engine.Topology.Iterator anIt = new(aPShape); anIt.More(); anIt.Next())
        {
            aP.Add(anIt.Value());
        }
        return ([.. aN], [.. aP]);
    }

    // ---- inputs ----

    private enum Mode { Default, CorrectedFrenet, Frenet, Discrete, Fixed, BiNormal, Auxiliary, Darboux }

    private sealed record Section(Shape Profile, int SpineVertex, bool WithContact, bool WithCorrection);

    private sealed class Case
    {
        public required string Name;
        public required Wire Spine;
        public required List<Section> Sections;
        public Mode Mode;
        public Frame3D Axe = new();
        public Direction3D BiNormal = new();
        public Shape Aux = new(); // auxiliary spine, or Darboux support
        public bool Curvilinear;
        public TypeOfContact Contact;
        public global::Kerf.Construction.TransitionMode Transition;
        public bool Solid;
        public int Simulate;
    }

    private static Wire Wire(params Edge[] theEdges)
    {
        MakeWire aMW = new();
        foreach (Edge anE in theEdges)
        {
            aMW.Add(anE);
        }
        return aMW.Wire();
    }

    private static Wire Polyline(bool theClosed, params Point3D[] thePoints)
    {
        MakePolygon aP = new();
        foreach (Point3D aQ in thePoints)
        {
            aP.Add(aQ);
        }
        if (theClosed)
        {
            aP.Close();
        }
        return aP.Wire();
    }

    // A random spine and its kind
    private static (Wire Wire, string Kind) RandomSpine(Oracle theOracle, int theKind)
    {
        switch (theKind)
        {
            case 0:
            {
                // a polyline with corners
                int aNb = theOracle.Int(1, 4);
                var aPoints = new List<Point3D> { new(0, 0, 0) };
                Vector3D aDir = new(0, 0, 1);
                for (int i = 0; i < aNb; i++)
                {
                    Vector3D aTurn = new(theOracle.Real(-0.8, 0.8), theOracle.Real(-0.8, 0.8), theOracle.Real(-0.3, 0.3));
                    aDir = (aDir + aTurn).Normalized();
                    aPoints.Add(aPoints[^1].Translated(aDir * theOracle.Real(6, 15)));
                }
                return (Polyline(false, [.. aPoints]), "polyline");
            }
            case 1:
            {
                // a G1 chain: a line, then tangent arcs
                Point3D aP0 = new(0, 0, 0), aP1 = new(0, 0, theOracle.Real(5, 10));
                List<Edge> anEdges = [new MakeEdge(aP0, aP1).Edge()];
                Vector3D aTangent = new(0, 0, 1);
                int aNb = theOracle.Int(1, 2);
                for (int i = 0; i < aNb; i++)
                {
                    Point3D aNext = aP1.Translated(aTangent * theOracle.Real(4, 8)
                                                  + new Vector3D(theOracle.Real(-6, 6), theOracle.Real(-6, 6), 0));
                    TrimmedCurve3D anArc = new MakeArcOfCircle(aP1, aTangent, aNext).Value();
                    anEdges.Add(new MakeEdge(anArc).Edge());
                    anArc.D1(anArc.LastParameter, out _, out aTangent);
                    aTangent = aTangent.Normalized();
                    aP1 = aNext;
                }
                return (Wire([.. anEdges]), "g1chain");
            }
            case 2:
            {
                // an interpolated B-spline
                int aNb = theOracle.Int(3, 6);
                NCollection_HArray1<Point3D> aPoints = new(1, aNb);
                for (int i = 1; i <= aNb; i++)
                {
                    aPoints.SetValue(i, new Point3D(theOracle.Real(-4, 4), theOracle.Real(-4, 4), 6.0 * (i - 1)));
                }
                ApiInterpolate anInt = new(aPoints, false, 1e-7);
                anInt.Perform();
                return (Wire(new MakeEdge(anInt.Curve()).Edge()), "bspline");
            }
            case 3:
            {
                // a closed circle in the plane XZ (the profile at (r, 0, 0) normal to it)
                double aR = theOracle.Real(8, 15);
                Circle3D aCirc = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, -1, 0), new Direction3D(1, 0, 0)), aR);
                return (Wire(new MakeEdge(aCirc).Edge()), "circle");
            }
            case 4:
            {
                // a closed polygon
                double aS = theOracle.Real(8, 15);
                return (Polyline(true, new Point3D(0, 0, 0), new Point3D(0, 0, aS), new Point3D(aS, 0, aS), new Point3D(aS, 0, 0)), "square");
            }
            default:
            {
                // a helix-like B-spline
                int aNb = theOracle.Int(8, 12);
                double aR = theOracle.Real(3, 6), aPitch = theOracle.Real(4, 8);
                NCollection_HArray1<Point3D> aPoints = new(1, aNb);
                for (int i = 1; i <= aNb; i++)
                {
                    double t = 0.5 * Math.PI * (i - 1);
                    aPoints.SetValue(i, new Point3D(aR * Math.Sin(t), aR * (1 - Math.Cos(t)), aPitch * t / (2 * Math.PI)));
                }
                ApiInterpolate anInt = new(aPoints, false, 1e-7);
                anInt.Perform();
                return (Wire(new MakeEdge(anInt.Curve()).Edge()), "helix");
            }
        }
    }

    // The frame normal to the spine at its start
    private static Frame3D StartFrame(in Wire theSpine)
    {
        WireCurveAdaptor aC = new(theSpine);
        aC.D1(aC.FirstParameter(), out Point3D aP, out Vector3D aV);
        return new Frame3D(aP, new Direction3D(aV));
    }

    private static Point3D OnPlane(in Frame3D theAx, double theU, double theV)
        => theAx.Location.Translated(new Vector3D(theAx.XDirection) * theU + new Vector3D(theAx.YDirection) * theV);

    // A random profile in the plane theAx, its kind
    private static (Shape Shape, string Kind) RandomProfile(Oracle theOracle, in Frame3D theAx, double theSize, bool theClosedOnly)
    {
        int aKind = theOracle.Int(0, theClosedOnly ? 4 : 6);
        switch (aKind)
        {
            case 0:
                return (Wire(new MakeEdge(new Circle3D(theAx, theSize)).Edge()), "circle");
            case 1:
            {
                double aMajor = theSize * theOracle.Real(1.0, 1.6);
                return (Wire(new MakeEdge(new Ellipse3D(theAx, aMajor, theSize * theOracle.Real(0.4, 0.99))).Edge()), "ellipse");
            }
            case 2:
            {
                double aW = theSize * theOracle.Real(0.6, 1.4), aH = theSize * theOracle.Real(0.6, 1.4);
                return (Polyline(true, OnPlane(theAx, -aW, -aH), OnPlane(theAx, aW, -aH), OnPlane(theAx, aW, aH), OnPlane(theAx, -aW, aH)), "rectangle");
            }
            case 3:
            {
                int aNb = theOracle.Int(3, 6);
                var aPoints = new Point3D[aNb];
                for (int i = 0; i < aNb; i++)
                {
                    double anAngle = 2.0 * Math.PI * (i + 0.2 * theOracle.Real(-1, 1)) / aNb;
                    double r = theSize * theOracle.Real(0.7, 1.3);
                    aPoints[i] = OnPlane(theAx, r * Math.Cos(anAngle), r * Math.Sin(anAngle));
                }
                return (Polyline(true, aPoints), "polygon");
            }
            case 4:
            {
                int aNb = theOracle.Int(4, 6);
                NCollection_HArray1<Point3D> aPoints = new(1, aNb);
                for (int i = 1; i <= aNb; i++)
                {
                    double anAngle = 2.0 * Math.PI * (i - 1) / aNb;
                    double r = theSize * theOracle.Real(0.7, 1.3);
                    aPoints.SetValue(i, OnPlane(theAx, r * Math.Cos(anAngle), r * Math.Sin(anAngle)));
                }
                ApiInterpolate anInt = new(aPoints, true, 1e-7);
                anInt.Perform();
                return (Wire(new MakeEdge(anInt.Curve()).Edge()), "periodic bspline");
            }
            case 5:
                return (Polyline(false, OnPlane(theAx, -theSize, 0), OnPlane(theAx, 0, theSize * theOracle.Real(0.2, 1)),
                                 OnPlane(theAx, theSize, 0)), "open polyline");
            default:
                return (Wire(new MakeEdge(new MakeArcOfCircle(OnPlane(theAx, -theSize, 0), OnPlane(theAx, 0, theSize * 0.6),
                                                                                OnPlane(theAx, theSize, 0)).Value()).Edge()), "open arc");
        }
    }

    // ---- the comparison ----

    private static N.gp_Pnt ToN(in Point3D theP) => Oracle.ToNative(theP);

    // A direction as native receives it: gp_Dir(x, y, z) normalises the coordinates again
    private static Direction3D Renormalised(in Direction3D theD) => new(theD.X, theD.Y, theD.Z);

    private static N.gp_Dir ToN(in Direction3D theD) => Oracle.ToNative(theD);

    private void Run(Case theCase, bool theRerun = false)
    {
        myCases++;
        // the inputs: spine, auxiliary shape, profiles
        List<Shape> anInputs = [theCase.Spine, theCase.Aux.IsNull ? theCase.Spine : theCase.Aux];
        anInputs.AddRange(theCase.Sections.Select(s => s.Profile));
        (N.TopoDS_Shape[] aNIn, Shape[] aPIn) = Both([.. anInputs]);
        string[] anInputTexts = [.. aPIn.Select(s => PortText(s))];
        N.TopoDS_Wire aNSpine = N.TopoDS.Wire(aNIn[0]);
        Wire aPSpine = global::Kerf.Engine.Topology.ShapeCasts.Wire(aPIn[0]);

        string aWhat = theCase.Name;
        var aWatch = Stopwatch.StartNew();
        string? aNError = null, aPError = null;
        using var aN = new N.BRepOffsetAPI_MakePipeShell(aNSpine);
        MakePipeShell? aP = null;
        bool aNSolid = false, aPSolid = false;

        // native
        try
        {
            switch (theCase.Mode)
            {
                case Mode.CorrectedFrenet: aN.SetMode(false); break;
                case Mode.Frenet: aN.SetMode(true); break;
                case Mode.Discrete: aN.SetDiscreteMode(); break;
                case Mode.Fixed: aN.SetMode(new N.gp_Ax2(ToN(theCase.Axe.Location), ToN(theCase.Axe.Direction), ToN(theCase.Axe.XDirection))); break;
                case Mode.BiNormal: aN.SetMode(ToN(theCase.BiNormal)); break;
                case Mode.Auxiliary: aN.SetMode(N.TopoDS.Wire(aNIn[1]), theCase.Curvilinear, (N.BRepFill_TypeOfContact)(int)theCase.Contact); break;
                case Mode.Darboux: aN.SetMode(aNIn[1]); break;
            }
            aN.SetTransitionMode((N.BRepBuilderAPI_TransitionMode)(int)theCase.Transition);
            for (int i = 0; i < theCase.Sections.Count; i++)
            {
                Section aS = theCase.Sections[i];
                if (aS.SpineVertex >= 0)
                {
                    aN.Add(aNIn[2 + i], NativeVertex(aNSpine, aS.SpineVertex), aS.WithContact, aS.WithCorrection);
                }
                else
                {
                    aN.Add(aNIn[2 + i], aS.WithContact, aS.WithCorrection);
                }
            }
            aN.Build();
            if (aN.IsDone() && theCase.Solid)
            {
                aNSolid = aN.MakeSolid();
            }
        }
        catch (N.OcctException e)
        {
            aNError = e.OcctType;
        }

        double aNTime = aWatch.Elapsed.TotalSeconds;
        aWatch.Restart();
        // the port
        try
        {
            aP = new MakePipeShell(aPSpine);
            switch (theCase.Mode)
            {
                case Mode.CorrectedFrenet: aP.SetMode(false); break;
                case Mode.Frenet: aP.SetMode(true); break;
                case Mode.Discrete: aP.SetDiscreteMode(); break;
                // built as native builds them from the coordinates (gp_Ax2(P, N, Vx) and gp_Dir(x, y, z) normalise again)
                case Mode.Fixed: aP.SetMode(new Frame3D(theCase.Axe.Location, Renormalised(theCase.Axe.Direction), Renormalised(theCase.Axe.XDirection))); break;
                case Mode.BiNormal: aP.SetMode(Renormalised(theCase.BiNormal)); break;
                case Mode.Auxiliary: aP.SetMode(global::Kerf.Engine.Topology.ShapeCasts.Wire(aPIn[1]), theCase.Curvilinear, theCase.Contact); break;
                case Mode.Darboux: aP.SetMode(aPIn[1]); break;
            }
            aP.SetTransitionMode(theCase.Transition);
            for (int i = 0; i < theCase.Sections.Count; i++)
            {
                Section aS = theCase.Sections[i];
                if (aS.SpineVertex >= 0)
                {
                    aP.Add(aPIn[2 + i], PortVertex(aPSpine, aS.SpineVertex), aS.WithContact, aS.WithCorrection);
                }
                else
                {
                    aP.Add(aPIn[2 + i], aS.WithContact, aS.WithCorrection);
                }
            }
            aP.Build();
            if (aP.IsDone && theCase.Solid)
            {
                aPSolid = aP.MakeSolid();
            }
        }
        catch (Exception f) when (Refusal.IsFailure(f))
        {
            aPError = global::Kerf.Engine.Foundation.OcctTypeNames.Of(Refusal.Unwrapped(f).GetType());
            if (f is ImmutableObjectException or global::Kerf.Engine.Topology.LockedShapeException)
            {
                aPError += " " + string.Join(" / ", (f.StackTrace ?? "").Split('\n').Take(4).Select(l => l.Trim()));
            }
        }
        catch (Exception e)
        {
            myDifferences.Add($"{aWhat}: native {aNError ?? "no exception"}, the port throws {Kerf.Engine.Foundation.OcctTypeNames.Of(Refusal.Unwrapped(e).GetType())} {e.StackTrace?.Split('\n')[0]}");
            return;
        }

        double aPTime = aWatch.Elapsed.TotalSeconds;
        if (aNTime + aPTime > 5)
        {
            output.WriteLine($"  slow: {aWhat}: native {aNTime:F1} s, port {aPTime:F1} s");
        }
        for (int i = 0; i < aPIn.Length; i++)
        {
            if (PortText(aPIn[i]) != anInputTexts[i])
            {
                myDifferences.Add($"{aWhat}: input {i} changed");
            }
        }

        if (aNError is not null || aPError is not null)
        {
            // Native reads uninitialised memory on a few cases and then crashes only now and then (case 220 once in a
            // Release gate on 1 November 2026, never before): run such a case again once, and keep the difference only
            // if native crashes again
            if (aNError == "OSD_Exception_ACCESS_VIOLATION" && aPError is null && !theRerun)
            {
                output.WriteLine($"  rerun: {aWhat}: {aNError} natively, no exception in the port");
                myCases--;
                Run(theCase, true);
                return;
            }
            if (aNError == "OSD_Exception_ACCESS_VIOLATION" && aPError is "Standard_NullObject" or "NullReferenceException")
            {
                myExceptions++; // a null handle natively (decision 930)
            }
            else if (aNError is null || aPError is null || !aNError.StartsWith(aPError, StringComparison.Ordinal))
            {
                myDifferences.Add($"{aWhat}: {aNError ?? "no exception"} natively, {aPError ?? "no exception"} in the port");
            }
            else
            {
                myExceptions++;
            }
            return;
        }

        if ((int)aN.GetStatus() != (int)aP!.GetStatus() || aN.IsDone() != aP.IsDone || aN.IsReady() != aP.IsReady)
        {
            myDifferences.Add($"{aWhat}: status native {aN.GetStatus()}/{aN.IsDone()}, port {aP.GetStatus()}/{aP.IsDone}");
            return;
        }
        if (!aN.IsDone())
        {
            myNotDone++;
            return;
        }
        myBuilt++;
        if (BitConverter.DoubleToInt64Bits(aN.ErrorOnSurface()) != BitConverter.DoubleToInt64Bits(aP.ErrorOnSurface()))
        {
            myDifferences.Add($"{aWhat}: ErrorOnSurface native {aN.ErrorOnSurface():R}, port {aP.ErrorOnSurface():R}");
        }
        if (aNSolid != aPSolid)
        {
            myDifferences.Add($"{aWhat}: MakeSolid native {aNSolid}, port {aPSolid}");
            return;
        }
        mySolids += aPSolid ? 1 : 0;

        N.TopoDS_Shape aNRes = aN.Shape();
        Shape aPRes = aP.Shape();
        if (!CompareShapes(aWhat + " result", aNRes, aPRes, true))
        {
            return;
        }
        bool aBounds = CompareShapes(aWhat + " FirstShape", aN.FirstShape(), aP.FirstShape(), false)
                       & CompareShapes(aWhat + " LastShape", aN.LastShape(), aP.LastShape(), false);
        myBoundsIdentical += aBounds ? 1 : 0;

        // Generated, as indices in the result
        var aNResMap = new N.TopTools_IndexedMapOfShape();
        N.TopExp.MapShapes(aNRes, aNResMap);
        IndexedSet<Shape> aPResMap = new(global::Kerf.Engine.Topology.ShapeHasher.Instance);
        global::Kerf.Engine.Topology.Exploration.MapShapes(aPRes, aPResMap);
        var aNH = new List<string>();
        var aPH = new List<string>();
        for (int k = 0; k < aPIn.Length; k++)
        {
            if (k == 1)
            {
                continue; // the auxiliary spine or the support
            }
            var aNInMap = new N.TopTools_IndexedMapOfShape();
            N.TopExp.MapShapes(aNIn[k], aNInMap);
            IndexedSet<Shape> aPInMap = new(global::Kerf.Engine.Topology.ShapeHasher.Instance);
            global::Kerf.Engine.Topology.Exploration.MapShapes(aPIn[k], aPInMap);
            for (int i = 1; i <= aNInMap.Extent(); i++)
            {
                aNH.Add($"{k}.{i}:" + string.Join(",", aN.Generated(aNInMap.FindKey(i)).Select(s => aNResMap.FindIndex(s))));
                aPH.Add($"{k}.{i}:" + string.Join(",", aP.Generated(aPInMap[i - 1]).Select(s => (aPResMap.IndexOf(s) + 1))));
            }
        }
        static string Sorted(string theEntry)
        {
            string[] aParts = theEntry.Split(':');
            return aParts[0] + ":" + string.Join(",", aParts[1].Split(',', StringSplitOptions.RemoveEmptyEntries).Select(int.Parse).Order());
        }
        if (aNH.SequenceEqual(aPH))
        {
            myHistory++;
        }
        else if (aNH.Select(Sorted).SequenceEqual(aPH.Select(Sorted)))
        {
            myHistory++;
            myHistoryOrder++;
        }
        else
        {
            int aFirst = Enumerable.Range(0, Math.Min(aNH.Count, aPH.Count)).FirstOrDefault(i => aNH[i] != aPH[i]);
            myDifferences.Add($"{aWhat}: Generated differ, first at {aFirst}: native {aNH.ElementAtOrDefault(aFirst)}, port {aPH.ElementAtOrDefault(aFirst)}");
        }

        // Simulate, on new makers
        if (theCase.Simulate > 0)
        {
            using var aNSim = new N.BRepOffsetAPI_MakePipeShell(aNSpine);
            var aPSim = new MakePipeShell(aPSpine);
            if (theCase.Mode == Mode.Frenet)
            {
                aNSim.SetMode(true);
                aPSim.SetMode(true);
            }
            aNSim.Add(aNIn[2]);
            aPSim.Add(aPIn[2]);
            using var aNList = new N.TopTools_ListOfShape();
            aNSim.Simulate(theCase.Simulate, aNList);
            NCollection_List<Shape> aPList = new();
            aPSim.Simulate(theCase.Simulate, aPList);
            N.TopoDS_Shape[] aNSecs = [.. aNList];
            List<Shape> aPSecs = [];
            for (NCollection_List<Shape>.Iterator anIt = new(aPList); anIt.More(); anIt.Next())
            {
                aPSecs.Add(anIt.Value());
            }
            bool isSame = aNSecs.Length == aPSecs.Count;
            for (int i = 0; isSame && i < aNSecs.Length; i++)
            {
                isSame = CompareShapes($"{aWhat} Simulate {i}", aNSecs[i], aPSecs[i], false);
            }
            if (aNSecs.Length != aPSecs.Count)
            {
                myDifferences.Add($"{aWhat}: Simulate gives {aNSecs.Length} sections natively, {aPSecs.Count} in the port");
            }
            mySimulated += isSame ? 1 : 0;
        }
    }

    private static N.TopoDS_Vertex NativeVertex(N.TopoDS_Wire theSpine, int theIndex)
    {
        var aMap = new N.TopTools_IndexedMapOfShape();
        N.TopExp.MapShapes(theSpine, N.TopAbs_ShapeEnum.TopAbs_VERTEX, aMap);
        return N.TopoDS.Vertex(aMap.FindKey(Math.Min(theIndex + 1, aMap.Extent())));
    }

    private static Vertex PortVertex(in Wire theSpine, int theIndex)
    {
        IndexedSet<Shape> aMap = new(global::Kerf.Engine.Topology.ShapeHasher.Instance);
        global::Kerf.Engine.Topology.Exploration.MapShapes(theSpine, ShapeType.Vertex, aMap);
        return global::Kerf.Engine.Topology.ShapeCasts.Vertex(aMap[Math.Min(theIndex + 1, aMap.Count) - 1]);
    }

    // Compares the shapes: BRep text, then with the Free column masked, then canonically. Counts the results' outcomes.
    private bool CompareShapes(string theWhat, N.TopoDS_Shape theN, in Shape theP, bool theCount)
    {
        if (theN.IsNull() || theP.IsNull)
        {
            if (theN.IsNull() != theP.IsNull)
            {
                myDifferences.Add($"{theWhat}: null natively {theN.IsNull()}, in the port {theP.IsNull}");
                return false;
            }
            return true;
        }
        string aNT = NativeText(theN), aPT = PortText(theP);
        BRepTextDiff aDiff = BRepTextComparison.Compare(aNT, aPT);
        if (aDiff.IsIdentical)
        {
            myIdentical += theCount ? 1 : 0;
            return true;
        }
        if (aDiff.IsIdenticalMaskingFree)
        {
            myIdentical += theCount ? 1 : 0;
            myFreeMasked += theCount ? 1 : 0;
            return true;
        }
        // canonically, both read back from their texts (the text rounds coordinates to 15 digits)
        string aTextFile = NewFile();
        File.WriteAllText(aTextFile, aNT);
        Assert.True(BrepTools.Read(out Shape aNP, aTextFile));
        File.WriteAllText(aTextFile, aPT);
        Assert.True(BrepTools.Read(out Shape aPP, aTextFile));
        File.Delete(aTextFile);
        CanonicalComparison aCanon = CanonicalShapeComparer.Compare(aNP, aPP);
        if (aCanon.IsIdentical)
        {
            myCanonical += theCount ? 1 : 0;
            return true;
        }
        myDifferences.Add($"{theWhat}: {aDiff}; canonically {aCanon}");
        return false;
    }

    private void Report(string theWhat)
    {
        output.WriteLine($"{theWhat}: {myCases} cases; {myExceptions} same exception on both sides, {myNotDone} not done on "
                         + $"both sides, {myBuilt} built ({mySolids} solids): {myIdentical} identical BRep text "
                         + $"({myFreeMasked} with the Free column masked), {myCanonical} canonically identical; first and "
                         + $"last shapes identical {myBoundsIdentical}; Generated identical {myHistory} ({myHistoryOrder} "
                         + $"in another order); Simulate identical {mySimulated}; differences: {myDifferences.Count}");
        foreach (string aDiff in myDifferences.Take(40))
        {
            output.WriteLine("  " + aDiff);
        }
    }

    // ---- the cases ----

    private static global::Kerf.Construction.TransitionMode RandomTransition(Oracle theOracle) => (global::Kerf.Construction.TransitionMode)theOracle.Int(0, 2);

    [Fact]
    public void Every_mode_and_transition_on_random_spines()
    {
        var anOracle = new Oracle(output, 20261031);
        Mode[] aModes = [Mode.Default, Mode.CorrectedFrenet, Mode.Frenet, Mode.Discrete, Mode.Fixed, Mode.BiNormal];
        int aCase = 0;
        foreach (int aRound in Tape.Budget(7))
        {
            for (int aSpineKind = 0; aSpineKind < 6; aSpineKind++)
            {
                foreach (Mode aMode in aModes)
                {
                    (Wire aSpine, string aKind) = RandomSpine(anOracle, aSpineKind);
                    Frame3D aFrame = StartFrame(aSpine);
                    (Shape aProfile, string aProfKind) = RandomProfile(anOracle, aFrame, anOracle.Real(0.3, 1.5), false);
                    Case c = new()
                    {
                        Name = $"case {aCase++} {aKind} {aProfKind} {aMode}",
                        Spine = aSpine,
                        Sections = [new Section(aProfile, -1, false, anOracle.Int(0, 4) == 0)],
                        Mode = aMode,
                        Axe = aFrame,
                        BiNormal = new Direction3D(anOracle.Real(-1, 1), anOracle.Real(-1, 1), anOracle.Real(-0.2, 0.2) + 2),
                        Transition = RandomTransition(anOracle),
                        Solid = anOracle.Bool(),
                        Simulate = anOracle.Int(0, 3) == 0 ? anOracle.Int(2, 6) : 0,
                    };
                    c.Name += $" {c.Transition}{(c.Solid ? " solid" : "")}";
                    Run(c);
                }
            }
        }
        Report("modes");
        Assert.Empty(myDifferences);
    }

    [Fact]
    public void Transitions_at_sharp_corners()
    {
        var anOracle = new Oracle(output, 20261101);
        int aCase = 0;
        foreach (int i in Tape.Budget(40))
        {
            // two or three segments with corners of 20 to 120 degrees
            int aNb = anOracle.Int(2, 3);
            List<Point3D> aPoints = [new(0, 0, 0)];
            Vector3D aDir = new(0, 0, 1);
            for (int k = 0; k < aNb; k++)
            {
                aPoints.Add(aPoints[^1].Translated(aDir * anOracle.Real(8, 14)));
                double anAngle = anOracle.Real(20, 120) * Math.PI / 180;
                Axis3D anAxis = new(new Point3D(), new Direction3D(anOracle.Real(-1, 1), anOracle.Real(-1, 1), 0.01));
                aDir = aDir.Rotated(anAxis, anAngle);
            }
            Wire aSpine = Polyline(false, [.. aPoints]);
            Frame3D aFrame = StartFrame(aSpine);
            foreach (global::Kerf.Construction.TransitionMode aTransition in new[] { global::Kerf.Construction.TransitionMode.Transformed,
                                                                          global::Kerf.Construction.TransitionMode.RightCorner,
                                                                          global::Kerf.Construction.TransitionMode.RoundCorner })
            {
                (Shape aProfile, string aProfKind) = RandomProfile(anOracle, aFrame, anOracle.Real(0.5, 2), true);
                Run(new Case
                {
                    Name = $"corner case {aCase++} {aNb} segments {aProfKind} {aTransition}",
                    Spine = aSpine,
                    Sections = [new Section(aProfile, -1, false, false)],
                    Mode = anOracle.Bool() ? Mode.Default : Mode.Frenet,
                    Transition = aTransition,
                    Solid = anOracle.Bool(),
                });
            }
        }
        Report("corners");
        Assert.Empty(myDifferences);
    }

    [Fact]
    public void Auxiliary_spines_with_and_without_contact()
    {
        var anOracle = new Oracle(output, 20261102);
        int aCase = 0;
        foreach (int i in Tape.Budget(30))
        {
            (Wire aSpine, string aKind) = RandomSpine(anOracle, anOracle.Int(0, 2) switch { 0 => 0, 1 => 2, _ => 3 });
            Frame3D aFrame = StartFrame(aSpine);
            // the guide: the spine moved sideways (build123d.NET's twisted extrusion uses a twisted copy)
            Transform3D aT = new Transform3D().WithTranslation(new Vector3D(aFrame.XDirection) * anOracle.Real(2, 5));
            Wire aGuide = global::Kerf.Engine.Topology.ShapeCasts.Wire(new Transform(aSpine, aT, true).Shape());
            foreach (bool aCurvilinear in new[] { true, false })
            {
                TypeOfContact aContact = (TypeOfContact)anOracle.Int(0, 2);
                if (aKind == "polyline" && aContact == TypeOfContact.Contact)
                {
                    // contact along a polyline takes minutes natively (370 s for one case) and ten times that in a
                    // Debug port; four such cases met native's access violation (decision 930)
                    aContact = TypeOfContact.NoContact;
                }
                (Shape aProfile, string aProfKind) = RandomProfile(anOracle, aFrame, anOracle.Real(0.5, 1.5), true);
                Run(new Case
                {
                    Name = $"aux case {aCase++} {aKind} {aProfKind} curvilinear {aCurvilinear} {aContact}",
                    Spine = aSpine,
                    Sections = [new Section(aProfile, -1, false, false)],
                    Mode = Mode.Auxiliary,
                    Aux = aGuide,
                    Curvilinear = aCurvilinear,
                    Contact = aContact,
                    Transition = RandomTransition(anOracle),
                    Solid = anOracle.Bool(),
                });
            }
        }
        Report("auxiliary");
        Assert.Empty(myDifferences);
    }

    [Fact]
    public void Darboux_on_supports()
    {
        var anOracle = new Oracle(output, 20261103);
        int aCase = 0;
        for (int i = 0; i < 20; i++)
        {
            Shape aSupport;
            Wire aSpine;
            if (anOracle.Bool())
            {
                // a box's face, the spine its outer wire or a part of it
                Shape aBox = new MakeBox(anOracle.Real(5, 15), anOracle.Real(5, 15), anOracle.Real(5, 15)).Shape();
                IndexedSet<Shape> aFaces = new(global::Kerf.Engine.Topology.ShapeHasher.Instance);
                global::Kerf.Engine.Topology.Exploration.MapShapes(aBox, ShapeType.Face, aFaces);
                Face aFace = global::Kerf.Engine.Topology.ShapeCasts.Face(aFaces[anOracle.Int(1, 6) - 1]);
                aSupport = anOracle.Bool() ? aFace : aBox;
                aSpine = BrepTools.OuterWire(aFace);
            }
            else
            {
                // a cylinder's lateral face, the spine its top circle
                Shape aCyl = new ApiMakeCylinder(anOracle.Real(3, 8), anOracle.Real(4, 10)).Shape();
                IndexedSet<Shape> aFaces = new(global::Kerf.Engine.Topology.ShapeHasher.Instance);
                global::Kerf.Engine.Topology.Exploration.MapShapes(aCyl, ShapeType.Face, aFaces);
                aSupport = aFaces[0];
                IndexedSet<Shape> anEdges = new(global::Kerf.Engine.Topology.ShapeHasher.Instance);
                global::Kerf.Engine.Topology.Exploration.MapShapes(aSupport, ShapeType.Edge, anEdges);
                Edge aCircle = new();
                for (int k = 0; k < anEdges.Count; k++)
                {
                    if (!Tool.IsClosed(global::Kerf.Engine.Topology.ShapeCasts.Edge(anEdges[k]), global::Kerf.Engine.Topology.ShapeCasts.Face(aSupport)))
                    {
                        aCircle = global::Kerf.Engine.Topology.ShapeCasts.Edge(anEdges[k]);
                        break;
                    }
                }
                aSpine = Wire(aCircle);
            }
            Frame3D aFrame = StartFrame(aSpine);
            (Shape aProfile, string aProfKind) = RandomProfile(anOracle, aFrame, anOracle.Real(0.3, 1.0), true);
            Run(new Case
            {
                Name = $"darboux case {aCase++} {Kerf.Engine.Foundation.OcctTypeNames.Of(aSupport.Type)} {aProfKind}",
                Spine = aSpine,
                Sections = [new Section(aProfile, -1, false, false)],
                Mode = Mode.Darboux,
                Aux = aSupport,
                Transition = RandomTransition(anOracle),
                Solid = anOracle.Bool(),
            });
        }
        Report("darboux");
        Assert.Empty(myDifferences);
    }

    [Fact]
    public void Several_sections_punctual_sections_contact_and_correction()
    {
        var anOracle = new Oracle(output, 20261104);
        int aCase = 0;
        foreach (int i in Tape.Budget(40))
        {
            (Wire aSpine, string aKind) = RandomSpine(anOracle, anOracle.Int(0, 2));
            IndexedSet<Shape> aVertices = new(global::Kerf.Engine.Topology.ShapeHasher.Instance);
            global::Kerf.Engine.Topology.Exploration.MapShapes(aSpine, ShapeType.Vertex, aVertices);
            WireCurveAdaptor aC = new(aSpine);
            List<Section> aSections = [];
            int aShape = anOracle.Int(0, 3);
            if (aShape == 0)
            {
                // a punctual section at the end
                Frame3D aFrame = StartFrame(aSpine);
                aSections.Add(new Section(RandomProfile(anOracle, aFrame, anOracle.Real(0.5, 1.5), true).Shape, -1, false, false));
                aC.D0(aC.LastParameter(), out Point3D anEnd);
                aSections.Add(new Section(new MakeVertex(anEnd).Vertex(), -1, false, false));
            }
            else
            {
                // sections along the spine: at its start, at its vertices or between, sometimes off it or tilted
                int aNb = anOracle.Int(2, 3);
                for (int k = 0; k < aNb; k++)
                {
                    double t = k == 0 ? aC.FirstParameter() : aC.FirstParameter() + (aC.LastParameter() - aC.FirstParameter()) * k / (aNb - 1);
                    aC.D1(t, out Point3D aP, out Vector3D aV);
                    bool isOff = aShape == 3;
                    Frame3D aFrame = new(isOff ? aP.Translated(new Vector3D(anOracle.Real(-0.5, 0.5), anOracle.Real(-0.5, 0.5), anOracle.Real(-0.5, 0.5))) : aP,
                                        new Direction3D(isOff ? aV + new Vector3D(anOracle.Real(-0.2, 0.2), anOracle.Real(-0.2, 0.2), 0) * aV.Magnitude : aV));
                    int aVertex = aShape == 2 && k > 0 ? (k == aNb - 1 ? aVertices.Count - 1 : anOracle.Int(0, aVertices.Count - 1)) : -1;
                    aSections.Add(new Section(RandomProfile(anOracle, aFrame, anOracle.Real(0.5, 1.5), true).Shape, aVertex,
                                              isOff && anOracle.Bool(), isOff && anOracle.Bool()));
                }
            }
            Run(new Case
            {
                Name = $"sections case {aCase++} {aKind} kind {aShape} {aSections.Count} sections",
                Spine = aSpine,
                Sections = aSections,
                Mode = anOracle.Int(0, 2) switch { 0 => Mode.Default, 1 => Mode.Frenet, _ => Mode.Discrete },
                Transition = RandomTransition(anOracle),
                Solid = anOracle.Bool(),
            });
        }
        Report("sections");
        Assert.Empty(myDifferences);
    }

    [Fact]
    public void Build123d_sweeps()
    {
        // build123d.NET's sweep (SetMode(isFrenet), SetTransitionMode, Add(wire, false, rotate), Build, MakeSolid), its
        // sweep with a binormal (a fixed frame at the path's start: SetMode(gp_Ax2)), with an auxiliary wire
        // (SetMode(wire, true)), and its twisted extrusion (SetMode(auxSpine, false)).
        var anOracle = new Oracle(output, 20261105);
        int aCase = 0;
        foreach (int i in Tape.Budget(30))
        {
            (Wire aSpine, string aKind) = RandomSpine(anOracle, anOracle.Int(0, 2));
            Frame3D aFrame = StartFrame(aSpine);
            (Shape aProfile, string aProfKind) = RandomProfile(anOracle, aFrame, anOracle.Real(0.3, 1.2), true);
            int aStyle = anOracle.Int(0, 3);
            Case c = new()
            {
                Name = $"build123d case {aCase++} {aKind} {aProfKind} style {aStyle}",
                Spine = aSpine,
                Sections = [new Section(aProfile, -1, false, aStyle == 1)],
                Mode = aStyle switch { 0 => anOracle.Bool() ? Mode.Frenet : Mode.CorrectedFrenet, 1 => Mode.Fixed, _ => Mode.Auxiliary },
                Axe = new Frame3D(aFrame.Location, new Direction3D(anOracle.Real(-1, 1), anOracle.Real(-1, 1), 3)),
                Transition = RandomTransition(anOracle),
                Solid = true,
            };
            if (c.Mode == Mode.Auxiliary)
            {
                Transform3D aT = new Transform3D().WithTranslation(new Vector3D(aFrame.XDirection) * anOracle.Real(2, 5));
                c.Aux = global::Kerf.Engine.Topology.ShapeCasts.Wire(new Transform(aSpine, aT, true).Shape());
                c.Curvilinear = aStyle == 2;
            }
            Run(c);
        }
        Report("build123d");
        Assert.Empty(myDifferences);
    }

    [Fact]
    public void CorrectedFrenet_on_an_offset_free_form_spine_timing()
    {
        // The slow case of unit 5L1's notes: corrected Frenet along an offset B-spline. One representative case, timed on
        // both sides (the Release figure is recorded in decision 932).
        // a twisted B-spline through random points, offset along Z (5L1's slow kind: random offsets of B-splines)
        var anOracle = new Oracle(output, 20261106);
        NCollection_HArray1<Point3D> aPoints = new(1, 6);
        for (int i = 1; i <= 6; i++)
        {
            aPoints.SetValue(i, new Point3D(5.0 * (i - 1), anOracle.Real(-6, 6), anOracle.Real(-3, 3)));
        }
        ApiInterpolate anInt = new(aPoints, false, 1e-7);
        anInt.Perform();
        OffsetCurve3D anOffset = new(anInt.Curve(), 4.0, new Direction3D(0, 0, 1));
        Wire aSpine = Wire(new MakeEdge(anOffset).Edge());
        Frame3D aFrame = StartFrame(aSpine);
        Shape aProfile = Wire(new MakeEdge(new Circle3D(aFrame, 0.8)).Edge());
        (N.TopoDS_Shape[] aNIn, Shape[] aPIn) = Both(aSpine, aProfile);

        var aWatch = Stopwatch.StartNew();
        using var aN = new N.BRepOffsetAPI_MakePipeShell(N.TopoDS.Wire(aNIn[0]));
        aN.SetMode(false);
        aN.Add(aNIn[1]);
        aN.Build();
        double aNative = aWatch.Elapsed.TotalSeconds;
        aWatch.Restart();
        MakePipeShell aP = new(global::Kerf.Engine.Topology.ShapeCasts.Wire(aPIn[0]));
        aP.SetMode(false);
        aP.Add(aPIn[1]);
        aP.Build();
        double aPort = aWatch.Elapsed.TotalSeconds;
        output.WriteLine($"corrected Frenet on an offset B-spline: native {aNative:F2} s, port {aPort:F2} s");
        Assert.Equal(aN.IsDone(), aP.IsDone);
        if (aN.IsDone())
        {
            SameTopology(aN.Shape(), aP.Shape());
        }
    }

    // Plan 3.2's third level (occt-issues.md, the offset curves' CalculateD3; Kerf decision 1487): the sweep's
    // approximation uses the trihedron's derivatives, which OCCT computed from the offset spine's wrong D2 and D3. This
    // offset of a twisted B-spline has cusps where the tube isn't defined (points of both shapes are up to 0.7 from
    // the radius 0.8), so the two approximations differ by up to 0.09 and in area (native 210.5, the port 204.8, 2πrL
    // 198.7): both are valid, with the same topology.
    private void SameTopology(N.TopoDS_Shape theN, in Shape theP)
    {
        Assert.Equal(theN.IsNull(), theP.IsNull);
        string aFile = NewFile();
        File.WriteAllText(aFile, NativeText(theN));
        Assert.True(BrepTools.Read(out Shape aNP, aFile));
        File.Delete(aFile);
        foreach (ShapeType aType in new[] { ShapeType.Face, ShapeType.Edge, ShapeType.Vertex })
        {
            var aNMap = new IndexedSet<Shape>();
            var aPMap = new IndexedSet<Shape>();
            global::Kerf.Engine.Topology.Exploration.MapShapes(aNP, aType, aNMap);
            global::Kerf.Engine.Topology.Exploration.MapShapes(theP, aType, aPMap);
            Assert.Equal(aNMap.Count, aPMap.Count);
        }
        Assert.True(new global::Kerf.Engine.Healing.Checks.Analyzer(aNP).IsValid());
        Assert.True(new global::Kerf.Engine.Healing.Checks.Analyzer(theP).IsValid());
    }
}
