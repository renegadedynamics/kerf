// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using OcctNet.TestHarness;
using Xunit.Abstractions;
using static Kerf.Geometry.JoinKind;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// BRepOffsetAPI_MakeOffset (and through it BRepFill_OffsetWire, TrimEdgeTool, OffsetAncestors, the medial axis and
/// BRepAlgo_FaceRestrictor) against native OCCT, unit 5O2.
/// </summary>
/// <remarks>
/// <para>Inputs: planar faces and wires built by the port and written to a BRep file that both sides read (the port's
/// copy frozen, as a caller's): random polygons and profiles of segments and arcs of circles in random planes (as a face,
/// sometimes with a hole, or as a wire), random open polylines and arc chains (closed around, or open with
/// IsOpenResult), and build123d.NET's own uses: <c>Offset2D</c> (an empty maker, <c>Init(join)</c>, <c>AddWire</c>,
/// the face's outer wire outwards and its holes inwards; a straight line through <c>Init(plane face, join)</c>), and the
/// slot objects (two collinear segments, and an arc split in two halves, offset by half the height). Curves that aren't
/// lines or circles run through <c>SetApprox</c>, and <c>ConvertFace</c> is compared on its own.</para>
/// <para>Each case performs one or two offsets (the second with another distance or sign on the same maker) with
/// both join types. Compared: IsDone; the result as BRep text (the Free column masked, then canonically); for every
/// edge and vertex of the input, Generated as indices in the result (in order, else as a set: native lists follow
/// BRepFill_OffsetWire's map iterated by address, occt-issues.md); and the input left unchanged. A port's exception
/// where native succeeds or fails is a difference.</para>
/// </remarks>
public sealed class BRepOffsetAPIMakeOffsetOracleTests(ITestOutputHelper output) : IDisposable
{
    private readonly string myDirectory = Directory.CreateTempSubdirectory("occtnet-mkoffset-").FullName;
    private int myFileNumber;
    private readonly List<string> myDifferences = [];
    private int myCases, myPerforms, myIdentical, myFreeMasked, myCanonical, myNotDone, myEmpty, myHistory, myHistoryOrder;

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

    // Writes the port's shape and reads it on both sides (the port's frozen)
    private (N.TopoDS_Shape N, TopoDS_Shape P) Both(in TopoDS_Shape theShape)
    {
        string aFile = NewFile();
        Assert.True(BRepTools.Write(theShape, aFile, true, false, TopTools_FormatVersion.TopTools_FormatVersion_VERSION_3));
        var aNShape = new N.TopoDS_Shape();
        using (var aBuilder = new N.BRep_Builder())
        {
            Assert.True(N.BRepTools.Read(aNShape, aFile, aBuilder));
        }
        Assert.True(BRepTools.Read(out TopoDS_Shape aPShape, aFile));
        File.Delete(aFile);
        aPShape.TShape()!.FreezeGraph();
        return (aNShape, aPShape);
    }

    // ---- inputs ----

    private static Point3D OnPlane(in Frame3D theAx, double theU, double theV)
        => theAx.Location().Translated(new Vector3D(theAx.XDirection()) * theU + new Vector3D(theAx.YDirection()) * theV);

    private static TopoDS_Edge Segment(in Point3D theA, in Point3D theB) => new BRepBuilderAPI_MakeEdge(theA, theB).Edge();

    private static TopoDS_Edge Arc(in Point3D theA, in Point3D theM, in Point3D theB)
        => new BRepBuilderAPI_MakeEdge(new GC_MakeArcOfCircle(theA, theM, theB).Value()).Edge();

    // An arc through a point beside the middle of the chord (theBulge: the offset, relative to the chord's length)
    private static TopoDS_Edge Bulge(in Frame3D theAx, in Point3D theA, in Point3D theB, double theBulge)
    {
        var aSide = new Vector3D(theA, theB);
        Vector3D aNormal = new Vector3D(theAx.Direction()).Crossed(aSide);
        return Arc(theA, theA.Translated(aSide * 0.5).Translated(aNormal * theBulge), theB);
    }

    // A closed polygon around (theU, theV), some sides arcs
    private static TopoDS_Wire ClosedProfile(Oracle theOracle, in Frame3D theAx, double theRadius, double theU, double theV, bool theArcs)
    {
        int aNb = theOracle.Int(3, 8);
        var aPoints = new Point3D[aNb];
        for (int i = 0; i < aNb; i++)
        {
            double anAngle = 2.0 * Math.PI * (i + 0.3 * theOracle.Real(-1.0, 1.0)) / aNb;
            double r = theRadius * theOracle.Real(0.6, 1.4);
            aPoints[i] = OnPlane(theAx, theU + r * Math.Cos(anAngle), theV + r * Math.Sin(anAngle));
        }
        var aWire = new BRepBuilderAPI_MakeWire();
        for (int i = 0; i < aNb; i++)
        {
            Point3D a = aPoints[i], b = aPoints[(i + 1) % aNb];
            aWire.Add(theArcs && theOracle.Int(0, 9) < 4 ? Bulge(theAx, a, b, theOracle.Real(-0.3, 0.3)) : Segment(a, b));
        }
        return aWire.Wire();
    }

    // An open chain of segments and arcs
    private static TopoDS_Wire OpenProfile(Oracle theOracle, in Frame3D theAx, double theSize, bool theArcs)
    {
        int aNb = theOracle.Int(1, 5);
        var aWire = new BRepBuilderAPI_MakeWire();
        double u = 0, v = 0, anAngle = theOracle.Angle();
        Point3D a = OnPlane(theAx, u, v);
        for (int i = 0; i < aNb; i++)
        {
            anAngle += theOracle.Real(-1.5, 1.5);
            double aLength = theSize * theOracle.Real(0.3, 1.0);
            u += aLength * Math.Cos(anAngle);
            v += aLength * Math.Sin(anAngle);
            Point3D b = OnPlane(theAx, u, v);
            aWire.Add(theArcs && theOracle.Bool() ? Bulge(theAx, a, b, theOracle.Real(-0.4, 0.4)) : Segment(a, b));
            a = b;
        }
        return aWire.Wire();
    }

    // ---- the comparison ----

    private sealed record Spine(TopoDS_Shape Shape, bool IsFace, JoinKind Join, bool IsOpenResult, bool IsApprox,
                                bool IsBuild123d, TopoDS_Shape PlaneFace);

    private static N.GeomAbs_JoinType ToNative(JoinKind theJoin) => (N.GeomAbs_JoinType)(int)theJoin;

    private void Compare(Spine theSpine, double[] theOffsets, double theAlt, string theWhat)
    {
        (N.TopoDS_Shape aNIn, TopoDS_Shape aPIn) = Both(theSpine.Shape);
        string anInputText = PortText(aPIn);
        N.TopoDS_Shape? aNPlane = null;
        TopoDS_Shape aPPlane = new();
        if (!theSpine.PlaneFace.IsNull())
        {
            (aNPlane, aPPlane) = Both(theSpine.PlaneFace);
        }
        myCases++;

        // the makers, as the case builds them
        using var aN = new N.BRepOffsetAPI_MakeOffset();
        var aP = new BRepOffsetAPI_MakeOffset();
        if (theSpine.IsFace)
        {
            aN.Init(N.TopoDS.Face(aNIn), ToNative(theSpine.Join), theSpine.IsOpenResult);
            aP.Init(TopoDS.Face(aPIn), theSpine.Join, theSpine.IsOpenResult);
        }
        else if (aNPlane is not null)
        {
            // build123d.NET's straight line: the plane's face, then the wire
            aN.Init(N.TopoDS.Face(aNPlane), ToNative(theSpine.Join), theSpine.IsOpenResult);
            aP.Init(TopoDS.Face(aPPlane), theSpine.Join, theSpine.IsOpenResult);
            aN.AddWire(N.TopoDS.Wire(aNIn));
            aP.AddWire(TopoDS.Wire(aPIn));
        }
        else
        {
            aN.Init(ToNative(theSpine.Join), theSpine.IsOpenResult);
            aP.Init(theSpine.Join, theSpine.IsOpenResult);
            aN.AddWire(N.TopoDS.Wire(aNIn));
            aP.AddWire(TopoDS.Wire(aPIn));
        }
        aN.SetApprox(theSpine.IsApprox);
        aP.SetApprox(theSpine.IsApprox);

        foreach (double anOffset in theOffsets)
        {
            string aWhat = $"{theWhat} offset {anOffset}";
            myPerforms++;
            aN.Perform(anOffset, theAlt);
            try
            {
                aP.Perform(anOffset, theAlt);
            }
            catch (Exception e)
            {
                myDifferences.Add($"{aWhat}: native IsDone {aN.IsDone()}, the port throws {Kerf.Engine.Foundation.OcctTypeNames.Of(e.GetType())} {e.StackTrace?.Split('\n')[0]}");
                return;
            }
            bool isDone = aN.IsDone();
            if (isDone != aP.IsDone())
            {
                myDifferences.Add($"{aWhat}: IsDone native {isDone}, port {aP.IsDone()}");
                return;
            }
            if (!isDone)
            {
                myNotDone++;
                continue;
            }

            N.TopoDS_Shape aNRes = aN.Shape();
            TopoDS_Shape aPRes = aP.Shape();
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
                    myDifferences.Add($"{aWhat}: {aDiff}; canonically {aCanon}");
                    continue;
                }
            }

            // Generated, as indices of the result's edges and vertices
            var aNResMap = new N.TopTools_IndexedMapOfShape();
            N.TopExp.MapShapes(aNRes, aNResMap);
            NCollection_IndexedMap<TopoDS_Shape> aPResMap = new(TopTools_ShapeMapHasher.Instance);
            TopExp.MapShapes(aPRes, aPResMap);
            if (aNResMap.Extent() <= 1)
            {
                myEmpty++;
            }
            var aNInMap = new N.TopTools_IndexedMapOfShape();
            N.TopExp.MapShapes(aNIn, aNInMap);
            NCollection_IndexedMap<TopoDS_Shape> aPInMap = new(TopTools_ShapeMapHasher.Instance);
            TopExp.MapShapes(aPIn, aPInMap);
            var aNH = new List<string>();
            var aPH = new List<string>();
            for (int i = 1; i <= aNInMap.Extent(); i++)
            {
                N.TopoDS_Shape aNS = aNInMap.FindKey(i);
                if (aNS.ShapeType() is not (N.TopAbs_ShapeEnum.TopAbs_EDGE or N.TopAbs_ShapeEnum.TopAbs_VERTEX))
                {
                    continue;
                }
                aNH.Add($"{i}:" + string.Join(",", aN.Generated(aNS).Select(s => aNResMap.FindIndex(s))));
                aPH.Add($"{i}:" + string.Join(",", aP.Generated(aPInMap.FindKey(i)).Select(s => aPResMap.FindIndex(s))));
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
                // the same shapes in another order: native's lists follow addresses (occt-issues.md, 5O2)
                myHistory++;
                myHistoryOrder++;
            }
            else
            {
                myDifferences.Add($"{aWhat}: Generated native [{string.Join(" ", aNH)}], port [{string.Join(" ", aPH)}]");
            }
        }

        if (PortText(aPIn) != anInputText)
        {
            myDifferences.Add($"{theWhat}: the input changed");
        }
    }

    private void Report()
    {
        output.WriteLine($"{myCases} cases, {myPerforms} offsets; {myNotDone} not done on both sides; "
                         + $"results: {myIdentical} identical BRep text ({myFreeMasked} with the Free column masked), "
                         + $"{myCanonical} canonically identical, {myEmpty} empty; Generated identical: {myHistory} "
                         + $"({myHistoryOrder} in another order); differences: {myDifferences.Count}");
        foreach (string aDiff in myDifferences.Take(30))
        {
            output.WriteLine("  " + aDiff);
        }
    }

    private static JoinKind Join(Oracle theOracle) => theOracle.Bool() ? global::Kerf.Geometry.JoinKind.Arc : Intersection;

    [Fact]
    public void Random_closed_profiles_both_joins_and_signs()
    {
        var anOracle = new Oracle(output, 5995);
        foreach (int i in Tape.Budget(300))
        {
            Frame3D anAx = anOracle.Bool() ? new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)) : anOracle.Ax2();
            double aSize = anOracle.Real(1.0, 50.0);
            bool isArcs = i % 2 == 1;
            bool isFace = anOracle.Int(0, 2) > 0;
            bool isHole = isFace && anOracle.Int(0, 3) == 0;
            TopoDS_Shape aShape;
            try
            {
                TopoDS_Wire anOuter = ClosedProfile(anOracle, anAx, aSize, 0, 0, isArcs);
                if (isFace)
                {
                    var aMaker = new BRepBuilderAPI_MakeFace(anOuter, true);
                    if (isHole)
                    {
                        TopoDS_Wire aHole = ClosedProfile(anOracle, anAx, aSize * 0.2, 0, 0, isArcs);
                        // a hole runs the other way to the outer wire
                        aMaker.Add(TopoDS.Wire(aHole.Reversed()));
                    }
                    aShape = aMaker.Face();
                }
                else
                {
                    aShape = anOuter;
                }
            }
            catch (FailureException)
            {
                continue; // a degenerate arc
            }
            JoinKind aJoin = Join(anOracle);
            double d1 = aSize * anOracle.Real(0.02, 0.6) * (anOracle.Bool() ? 1 : -1);
            double[] anOffsets = anOracle.Int(0, 2) == 0 ? [d1, -d1 * anOracle.Real(0.3, 1.2)] : [d1];
            double anAlt = anOracle.Int(0, 4) == 0 ? anOracle.Real(-5, 5) : 0.0;
            Compare(new Spine(aShape, isFace, aJoin, false, false, false, new TopoDS_Shape()), anOffsets, anAlt,
                    $"{(isArcs ? "arcs" : "polygon")} #{i} {(isFace ? isHole ? "face with a hole" : "face" : "wire")} {aJoin} alt {anAlt}");
        }
        Report();
        Assert.Empty(myDifferences);
    }

    [Fact]
    public void Random_open_wires_closed_around_or_open()
    {
        var anOracle = new Oracle(output, 5996);
        foreach (int i in Tape.Budget(300))
        {
            Frame3D anAx = anOracle.Bool() ? new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)) : anOracle.Ax2();
            double aSize = anOracle.Real(1.0, 50.0);
            bool isArcs = i % 2 == 1;
            TopoDS_Wire aWire;
            try
            {
                aWire = OpenProfile(anOracle, anAx, aSize, isArcs);
            }
            catch (FailureException)
            {
                continue;
            }
            // a single segment has no plane: give it one, as build123d.NET does
            TopoDS_Shape aPlane = new();
            if (!isArcs && new BRepBuilderAPI_MakeFace(aWire, true).Error() != BRepBuilderAPI_FaceError.BRepBuilderAPI_FaceDone)
            {
                aPlane = new BRepBuilderAPI_MakeFace(new Plane3D(anAx.Location(), anAx.Direction())).Face();
            }
            JoinKind aJoin = Join(anOracle);
            bool isOpen = anOracle.Bool();
            double d1 = aSize * anOracle.Real(0.02, 0.5) * (anOracle.Bool() ? 1 : -1);
            double[] anOffsets = anOracle.Int(0, 2) == 0 ? [d1, -d1 * anOracle.Real(0.3, 1.2)] : [d1];
            Compare(new Spine(aWire, false, aJoin, isOpen, false, false, aPlane), anOffsets, 0.0,
                    $"open {(isArcs ? "arcs" : "polyline")} #{i} {aJoin} {(isOpen ? "open result" : "closed result")}");
        }
        Report();
        Assert.Empty(myDifferences);
    }

    // TrimEdgeTool's intersections of a bisector line with a parallel circle (Geom2dInt_GInter, IntCurve_IntConicConic):
    // bit-identical since decision 916 fixed the port's angle in LineCircleGeometricIntersection
    [Fact]
    public void Line_circle_and_circle_circle_intersections()
    {
        var anOracle = new Oracle(output, 2);
        int aSame = 0, aDiff = 0;
        foreach (int i in Tape.Budget(2000))
        {
            double cx = anOracle.Real(-10, 10), cy = anOracle.Real(-10, 10), r = anOracle.Real(1, 10);
            double dx = anOracle.Real(-1, 1), dy = anOracle.Real(-1, 1);
            var aC = new Geom2d_Circle(new Axis2D(new Point2D(cx, cy), new Direction2D(dx, dy)), r);
            var aNC = new N.Geom2d_Circle(new N.gp_Ax2d(new N.gp_Pnt2d(cx, cy), new N.gp_Dir2d(dx, dy)), r);
            double px = cx + anOracle.Real(-r, r), py = cy + anOracle.Real(-r, r), ex = anOracle.Real(-1, 1), ey = anOracle.Real(-1, 1);
            Geom2d_Curve aL;
            N.Geom2d_Curve aNL;
            if (i % 2 == 0)
            {
                aL = new Geom2d_Line(new Point2D(px, py), new Direction2D(ex, ey));
                aNL = new N.Geom2d_Line(new N.gp_Pnt2d(px, py), new N.gp_Dir2d(ex, ey));
            }
            else
            {
                double r2 = anOracle.Real(1, 10);
                aL = new Geom2d_Circle(new Axis2D(new Point2D(px, py), new Direction2D(ex, ey)), r2);
                aNL = new N.Geom2d_Circle(new N.gp_Ax2d(new N.gp_Pnt2d(px, py), new N.gp_Dir2d(ex, ey)), r2);
            }
            var aP = new Geom2dAPI_InterCurveCurve(aC, aL, 1e-7);
            var aN = new N.Geom2dAPI_InterCurveCurve(aNC, aNL, 1e-7);
            string sp = "", sn = "";
            for (int k = 1; k <= aP.NbPoints(); k++)
            {
                sp += $"{aP.Point(k).X():R},{aP.Point(k).Y():R};";
            }
            for (int k = 1; k <= aN.NbPoints(); k++)
            {
                sn += $"{aN.Point(k).X():R},{aN.Point(k).Y():R};";
            }
            if (sp == sn)
            {
                aSame++;
            }
            else
            {
                aDiff++;
                output.WriteLine($"{(i % 2 == 0 ? "line" : "circle")} #{i}: native {sn} port {sp}");
            }
        }
        output.WriteLine($"{aSame} identical, {aDiff} different");
        Assert.Equal(0, aDiff);
    }

    [Fact]
    public void Single_arcs_closed_around_and_open()
    {
        var anOracle = new Oracle(output, 1);
        foreach (int i in Tape.Budget(200))
        {
            Frame3D anAx = new(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
            double r = anOracle.Real(5, 20), a0 = anOracle.Angle(), a1 = a0 + anOracle.Real(0.3, 3.0);
            Point3D P(double t) => new(r * Math.Cos(t), r * Math.Sin(t), 0);
            TopoDS_Wire aW = new BRepBuilderAPI_MakeWire(Arc(P(a0), P((a0 + a1) / 2), P(a1))).Wire();
            Compare(new Spine(aW, false, global::Kerf.Geometry.JoinKind.Arc, i % 2 == 0, false, false, new TopoDS_Shape()), [anOracle.Real(0.5, 3)], 0.0, $"single arc #{i} {(i % 2 == 0 ? "open" : "closed")}");
        }
        Report();
        Assert.Empty(myDifferences);
    }

    [Fact]
    public void Build123d_Offset2D_and_slots()
    {
        var anOracle = new Oracle(output, 5997);
        Frame3D anXY = new(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        foreach (int i in Tape.Budget(120))
        {
            double h = anOracle.Real(0.5, 20.0);
            double c = anOracle.Real(1.0, 60.0);
            JoinKind aJoin = i % 4 == 3 ? Intersection : global::Kerf.Geometry.JoinKind.Arc;
            switch (i % 6)
            {
                case 0:
                {
                    // SlotCenterToCenter: two collinear segments (the centre one) offset by h / 2
                    var aW = new BRepBuilderAPI_MakeWire(Segment(new Point3D(-c / 2, 0, 0), new Point3D(0, 0, 0)),
                                                         Segment(new Point3D(0, 0, 0), new Point3D(c / 2, 0, 0))).Wire();
                    Compare(new Spine(aW, false, aJoin, false, false, true, new TopoDS_Shape()), [h / 2], 0.0, $"SlotCenterToCenter #{i} {c} {h}");
                    break;
                }
                case 1:
                {
                    // SlotArc: an arc split in two halves (build123d.NET's workaround for one edge)
                    double r = anOracle.Real(5.0, 40.0), a0 = anOracle.Angle(), a1 = a0 + anOracle.Real(0.3, 4.0);
                    Point3D P(double t) => new(r * Math.Cos(t), r * Math.Sin(t), 0);
                    double am = (a0 + a1) / 2;
                    TopoDS_Wire aW;
                    try
                    {
                        aW = new BRepBuilderAPI_MakeWire(Arc(P(a0), P((a0 + am) / 2), P(am)), Arc(P(am), P((am + a1) / 2), P(a1))).Wire();
                    }
                    catch (FailureException)
                    {
                        continue;
                    }
                    Compare(new Spine(aW, false, aJoin, false, false, true, new TopoDS_Shape()), [Math.Min(h, r * 0.9) / 2], 0.0, $"SlotArc #{i} r {r} h {h}");
                    break;
                }
                case 2:
                {
                    // Offset2D of a face: the outer wire outwards, a hole inwards (each its own maker)
                    TopoDS_Wire anOuter = ClosedProfile(anOracle, anXY, c, 0, 0, true);
                    double d = c * anOracle.Real(0.02, 0.3);
                    Compare(new Spine(anOuter, false, aJoin, false, false, true, new TopoDS_Shape()), [d], 0.0, $"Offset2D outer #{i}");
                    Compare(new Spine(anOuter.Reversed(), false, aJoin, false, false, true, new TopoDS_Shape()), [-d], 0.0, $"Offset2D hole #{i}");
                    break;
                }
                case 3:
                {
                    // a straight line: a plane through it
                    TopoDS_Wire aW = new BRepBuilderAPI_MakeWire(Segment(new Point3D(0, 0, 0), new Point3D(c, anOracle.Real(-5, 5), 0))).Wire();
                    TopoDS_Shape aPlane = new BRepBuilderAPI_MakeFace(new Plane3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1))).Face();
                    Compare(new Spine(aW, false, aJoin, false, false, true, aPlane), [h / 2, -h / 3], 0.0, $"line #{i}");
                    break;
                }
                case 4:
                {
                    // a rectangle, as Offset2D's (Kind.Intersection keeps the corners)
                    var aW = new BRepBuilderAPI_MakePolygon(new Point3D(-c / 2, -h / 2, 0), new Point3D(c / 2, -h / 2, 0), new Point3D(c / 2, h / 2, 0),
                                                            new Point3D(-c / 2, h / 2, 0), true).Wire();
                    double d = Math.Min(c, h) * anOracle.Real(0.05, 0.45) * (anOracle.Bool() ? 1 : -1);
                    Compare(new Spine(aW, false, aJoin, false, false, true, new TopoDS_Shape()), [d], 0.0, $"rectangle #{i} {c}x{h}");
                    break;
                }
                default:
                {
                    // SlotOverall: as the centre-to-centre slot, from the overall width
                    double w = c + h;
                    var aW = new BRepBuilderAPI_MakeWire(Segment(new Point3D(-(w / 2 - h / 2), 0, 0), new Point3D(0, 0, 0)),
                                                         Segment(new Point3D(0, 0, 0), new Point3D(w / 2 - h / 2, 0, 0))).Wire();
                    Compare(new Spine(aW, false, aJoin, false, false, true, new TopoDS_Shape()), [h / 2], 0.0, $"SlotOverall #{i} {w} {h}");
                    break;
                }
            }
        }
        Report();
        Assert.Empty(myDifferences);
    }

    [Fact]
    public void Approximated_curves_and_ConvertFace()
    {
        var anOracle = new Oracle(output, 5998);
        int aConverted = 0;
        foreach (int i in Tape.Budget(60))
        {
            // an ellipse, or a closed B-spline through points around the origin
            TopoDS_Wire aW;
            if (i % 2 == 0)
            {
                double a = anOracle.Real(5, 30), b = a * anOracle.Real(0.3, 0.95);
                var anEllipse = new Geom_Ellipse(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), a, b);
                aW = new BRepBuilderAPI_MakeWire(new BRepBuilderAPI_MakeEdge(anEllipse).Edge()).Wire();
            }
            else
            {
                int aNb = anOracle.Int(4, 8);
                var aPoints = new NCollection_HArray1<Point3D>(1, aNb);
                for (int k = 1; k <= aNb; k++)
                {
                    double t = 2 * Math.PI * (k - 1) / aNb, r = anOracle.Real(10, 20);
                    aPoints[k] = new Point3D(r * Math.Cos(t), r * Math.Sin(t), 0);
                }
                var anInterp = new GeomAPI_Interpolate(aPoints, true, 1.0e-7);
                anInterp.Perform();
                aW = new BRepBuilderAPI_MakeWire(new BRepBuilderAPI_MakeEdge(anInterp.Curve()).Edge()).Wire();
            }
            bool isFace = i % 4 < 2;
            TopoDS_Shape aShape = isFace ? new BRepBuilderAPI_MakeFace(aW, true).Face() : aW;
            Compare(new Spine(aShape, isFace, i % 3 == 0 ? Intersection : global::Kerf.Geometry.JoinKind.Arc, false, true, false, new TopoDS_Shape()),
                    [anOracle.Real(-3, 3)], 0.0, $"approx #{i} {(i % 2 == 0 ? "ellipse" : "B-spline")} {(isFace ? "face" : "wire")}");

            if (isFace)
            {
                (N.TopoDS_Shape aNF, TopoDS_Shape aPF) = Both(aShape);
                string aNT = NativeText(N.BRepOffsetAPI_MakeOffset.ConvertFace(N.TopoDS.Face(aNF), 0.1));
                string aPT = PortText(BRepOffsetAPI_MakeOffset.ConvertFace(TopoDS.Face(aPF), 0.1));
                BRepTextDiff aDiff = BRepTextComparison.Compare(aNT, aPT);
                if (aDiff.IsIdentical || aDiff.IsIdenticalMaskingFree)
                {
                    aConverted++;
                }
                else
                {
                    myDifferences.Add($"ConvertFace #{i}: {aDiff}");
                }
            }
        }
        output.WriteLine($"ConvertFace: {aConverted} identical");
        Report();
        Assert.Empty(myDifferences);
    }
}
