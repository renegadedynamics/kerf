#if HAS_ORACLE
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Xunit.Abstractions;
using N = Build123d.Occt;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// The topology core against native OCCT: random shapes built with BRep_Builder identically on both sides
/// (<see cref="TwinShapes"/>: boxes, cylinder faces with seams, faces with holes, compounds sharing sub-shapes at
/// several locations and orientations), then the port's TopoDS, TopExp, BRep_Tool, BRepTools and BRepAdaptor_Surface
/// answers compared with native OCCT's. The port's shapes are frozen first, as every shape a caller holds is.
/// </summary>
public class TopologyOracleTests(ITestOutputHelper output)
{
    private const int Scenes = 60;

    private static readonly TopAbs_ShapeEnum[] theTypes =
        [TopAbs_COMPOUND, TopAbs_COMPSOLID, TopAbs_SOLID, TopAbs_SHELL, TopAbs_FACE, TopAbs_WIRE, TopAbs_EDGE, TopAbs_VERTEX];

    private sealed class Counts
    {
        public int Shapes;
        public int Pairs;
        public int Edges;
        public int EdgeFaces;
        public int Faces;
    }

    [Fact]
    public void Random_shapes_answer_as_native_OCCT()
    {
        var anOracle = new Oracle(output);
        var aCounts = new Counts();
        for (int aScene = 0; aScene < Scenes; aScene++)
        {
            Twin<N.TopoDS_Shape, TopoDS_Shape> aShape = RandomScene(anOracle, aScene);
            CompareFlags(anOracle, aShape);
            aShape.P.TShape()!.FreezeGraph();
            CompareTopology(anOracle, aShape, aCounts);
            CompareGeometry(anOracle, aShape, aCounts);
        }

        output.WriteLine($"{Scenes} scenes: {aCounts.Shapes} explored sub-shapes, {aCounts.Pairs} identity pairs, {aCounts.Edges} edges, "
                         + $"{aCounts.EdgeFaces} edge-face pairs, {aCounts.Faces} faces");
        anOracle.Report("topology (TopoDS, TopExp, BRep_Tool, BRepTools, BRepAdaptor_Surface)");
    }

    // Scenes

    internal static Twin<N.TopoDS_Shape, TopoDS_Shape> RandomScene(Oracle theOracle, int theScene)
    {
        var aTwins = new TwinShapes();
        switch (theScene % 4)
        {
            case 0:
                return Placed(theOracle, TwinShapes.Up(aTwins.Box(theOracle.Real(0.5, 20), theOracle.Real(0.5, 20), theOracle.Real(0.5, 20))));
            case 1:
            {
                (N.TopLoc_Location aNL, TopLoc_Location aL) = RandomLocation(theOracle);
                return Placed(theOracle, TwinShapes.Up(aTwins.CylinderFace(theOracle.Real(0.5, 5), theOracle.Real(0.5, 5), aL, aNL)));
            }
            case 2:
                return Placed(theOracle, TwinShapes.Up(FaceWithHole(theOracle, aTwins)));
            default:
            {
                // a compound sharing a box and a face at several locations and orientations
                Twin<N.TopoDS_Shape, TopoDS_Shape> aBox = TwinShapes.Up(aTwins.Box(theOracle.Real(1, 5), theOracle.Real(1, 5), theOracle.Real(1, 5)));
                (N.TopLoc_Location aNL, TopLoc_Location aL) = RandomLocation(theOracle);
                Twin<N.TopoDS_Shape, TopoDS_Shape> aFace = TwinShapes.Up(aTwins.CylinderFace(1.0, 2.0, aL, aNL));
                (N.TopLoc_Location aNL1, TopLoc_Location aL1) = RandomLocation(theOracle);
                (N.TopLoc_Location aNL2, TopLoc_Location aL2) = RandomLocation(theOracle);
                Twin<N.TopoDS_Compound, TopoDS_Compound> aCompound = aTwins.Compound(
                    aBox,
                    new(aBox.N.Moved(aNL1), aBox.P.Moved(aL1)),
                    new(aBox.N.Moved(aNL2).Reversed(), aBox.P.Moved(aL2).Reversed()),
                    aFace,
                    new(aFace.N.Moved(aNL1).Oriented(N.TopAbs_Orientation.TopAbs_INTERNAL), aFace.P.Moved(aL1).Oriented(TopAbs_INTERNAL)));
                return Placed(theOracle, TwinShapes.Up(aCompound));
            }
        }
    }

    /// <summary>The shape at a random location (or none), and reversed half the time.</summary>
    private static Twin<N.TopoDS_Shape, TopoDS_Shape> Placed(Oracle theOracle, Twin<N.TopoDS_Shape, TopoDS_Shape> theShape)
    {
        Twin<N.TopoDS_Shape, TopoDS_Shape> aShape = theShape;
        if (theOracle.Bool())
        {
            (N.TopLoc_Location aNL, TopLoc_Location aL) = RandomLocation(theOracle);
            aShape = new(aShape.N.Moved(aNL), aShape.P.Moved(aL));
        }

        return theOracle.Bool() ? new(aShape.N.Reversed(), aShape.P.Reversed()) : aShape;
    }

    /// <summary>A rigid location (a rotation, a translation or both) built from the same raw numbers on both sides.</summary>
    internal static (N.TopLoc_Location, TopLoc_Location) RandomLocation(Oracle theOracle)
    {
        var aNT = new N.gp_Trsf();
        var aT = new gp_Trsf();
        if (theOracle.Bool())
        {
            (N.gp_Ax1 aNAx, gp_Ax1 anAx) = GeomOracleInputs.Ax1s(theOracle);
            double anAngle = theOracle.Angle();
            using (aNAx)
            {
                aNT.SetRotation(aNAx, anAngle);
            }

            aT.SetRotation(anAx, anAngle);
        }

        if (theOracle.Bool())
        {
            gp_Vec aV = theOracle.Vec();
            using var aNV = Oracle.ToNative(aV);
            var aNT2 = new N.gp_Trsf();
            aNT2.SetTranslation(aNV);
            aNT = aNT2.Multiplied(aNT);
            aT = gp_Trsf.Translation(aV).Multiplied(aT);
        }

        return (new N.TopLoc_Location(aNT), new TopLoc_Location(aT));
    }

    /// <summary>A square face with a rectangular hole, on a random plane (the hole's wire first).</summary>
    private static Twin<N.TopoDS_Face, TopoDS_Face> FaceWithHole(Oracle theOracle, TwinShapes theTwins)
    {
        gp_Pnt anOrigin = theOracle.Pnt();
        gp_XYZ aNormal = GeomOracleInputs.RawDir(theOracle);
        gp_XYZ aRef = GeomOracleInputs.RawDir(theOracle);
        gp_XYZ anX = aNormal.Crossed(aRef);
        while (anX.Modulus() < 0.1)
        {
            aRef = GeomOracleInputs.RawDir(theOracle);
            anX = aNormal.Crossed(aRef);
        }

        // the corners in the plane's own coordinates, placed by the port's plane (both sides get these points)
        var aPln = new gp_Pln(new gp_Ax3(anOrigin, new gp_Dir(aNormal), new gp_Dir(anX)));
        double aSize = theOracle.Real(4, 10);
        double hu1 = theOracle.Real(0.5, 1.5), hu2 = theOracle.Real(2, 3.5), hv1 = theOracle.Real(0.5, 1.5), hv2 = theOracle.Real(2, 3.5);
        (double U, double V)[] aUV = [(0, 0), (aSize, 0), (aSize, aSize), (0, aSize), (hu1, hv1), (hu1, hv2), (hu2, hv2), (hu2, hv1)];
        var aPoints = new gp_Pnt[8];
        var aV = new Twin<N.TopoDS_Vertex, TopoDS_Vertex>[8];
        for (int i = 0; i < 8; i++)
        {
            aPoints[i] = ElSLib.Value(aUV[i].U, aUV[i].V, aPln);
            aV[i] = theTwins.Vertex(aPoints[i], 1e-6);
        }

        return theTwins.PolygonFace(aPoints, aV, new Dictionary<(int, int), Twin<N.TopoDS_Edge, TopoDS_Edge>>(),
                                    anOrigin, aNormal, anX, [4, 5, 6, 7], [0, 1, 2, 3]);
    }

    /// <summary>
    /// BRepTools' in-place statics (internal in the port) on unfrozen shapes built identically: Update, Triangulation,
    /// Clean, CleanGeometry and RemoveInternals change the two sides alike.
    /// </summary>
    [Fact]
    public void In_place_statics_change_shapes_as_native_OCCT()
    {
        var anOracle = new Oracle(output);
        for (int aRound = 0; aRound < 20; aRound++)
        {
            var aTwins = new TwinShapes();
            Twin<N.TopoDS_Shape, TopoDS_Shape> aBox =
                TwinShapes.Up(aTwins.Box(anOracle.Real(1, 5), anOracle.Real(1, 5), anOracle.Real(1, 5)));
            List<Twin<N.TopoDS_Shape, TopoDS_Shape>> aFaces = Explore(aBox, TopAbs_FACE);
            foreach (Twin<N.TopoDS_Shape, TopoDS_Shape> aF in aFaces)
            {
                if (anOracle.Bool())
                {
                    aTwins.AttachTriangulation(new(N.TopoDS.Face(aF.N), TopoDS.Face(aF.P)), anOracle.Real(0.01, 1));
                }
            }

            double aDefl = anOracle.Real(0.01, 1);
            anOracle.Same(N.BRepTools.Triangulation(aBox.N, aDefl), BRepTools.Triangulation(aBox.P, aDefl), "Triangulation");

            N.BRepTools.Update(aBox.N);
            BRepTools.Update(aBox.P);
            CompareFlags(anOracle, aBox);

            N.BRepTools.Clean(aBox.N);
            BRepTools.Clean(aBox.P);
            foreach (Twin<N.TopoDS_Shape, TopoDS_Shape> aF in aFaces)
            {
                using var aNL = new N.TopLoc_Location();
                anOracle.Same(N.BRep_Tool.Triangulation(N.TopoDS.Face(aF.N), aNL) is null,
                              BRep_Tool.Triangulation(TopoDS.Face(aF.P), out _) is null, "Triangulation after Clean");
            }

            CompareFlags(anOracle, aBox);

            if (aRound % 2 == 0)
            {
                N.BRepTools.CleanGeometry(aBox.N);
                BRepTools.CleanGeometry(aBox.P);
                foreach (Twin<N.TopoDS_Shape, TopoDS_Shape> aF in aFaces)
                {
                    anOracle.Same(N.BRep_Tool.IsGeometric(N.TopoDS.Face(aF.N)), BRep_Tool.IsGeometric(TopoDS.Face(aF.P)), "IsGeometric(face)");
                }

                foreach (Twin<N.TopoDS_Shape, TopoDS_Shape> anE in Explore(aBox, TopAbs_EDGE))
                {
                    anOracle.Same(N.BRep_Tool.IsGeometric(N.TopoDS.Edge(anE.N)), BRep_Tool.IsGeometric(TopoDS.Edge(anE.P)), "IsGeometric(edge)");
                }

                CompareFlags(anOracle, aBox);
            }

            // internal sub-shapes: a compound of the box, an internal face of it and an internal edge sharing its vertices
            Twin<N.TopoDS_Compound, TopoDS_Compound> aCompound = aTwins.Compound(aBox);
            Twin<N.TopoDS_Shape, TopoDS_Shape> aFace = aFaces[anOracle.Int(0, 5)];
            aTwins.Add(TwinShapes.Up(aCompound), new(aFace.N.Oriented(N.TopAbs_Orientation.TopAbs_INTERNAL), aFace.P.Oriented(TopAbs_INTERNAL)));
            Twin<N.TopoDS_Shape, TopoDS_Shape> anEdge = Explore(aFace, TopAbs_EDGE)[anOracle.Int(0, 3)];
            aTwins.Add(TwinShapes.Up(aCompound), new(anEdge.N.Oriented(N.TopAbs_Orientation.TopAbs_INTERNAL), anEdge.P.Oriented(TopAbs_INTERNAL)));
            bool isForce = anOracle.Bool();
            N.BRepTools.RemoveInternals(aCompound.N, isForce);
            BRepTools.RemoveInternals(aCompound.P, isForce);
            foreach (TopAbs_ShapeEnum aType in theTypes)
            {
                Explore(TwinShapes.Up(aCompound), aType);
            }

            anOracle.Same(aCompound.N.NbChildren(), aCompound.P.NbChildren(), "RemoveInternals");
            CompareFlags(anOracle, TwinShapes.Up(aCompound));
        }

        anOracle.Report("BRepTools in-place statics");
    }

    // Comparisons

    internal static List<Twin<N.TopoDS_Shape, TopoDS_Shape>> Explore(Twin<N.TopoDS_Shape, TopoDS_Shape> theShape,
                                                                    TopAbs_ShapeEnum theType,
                                                                    TopAbs_ShapeEnum theAvoid = TopAbs_SHAPE)
    {
        var aList = new List<Twin<N.TopoDS_Shape, TopoDS_Shape>>();
        var aNExp = new N.TopExp_Explorer(theShape.N, TwinShapes.ToNative(theType), TwinShapes.ToNative(theAvoid));
        var anExp = new TopExp_Explorer(theShape.P, theType, theAvoid);
        for (; anExp.More(); anExp.Next(), aNExp.Next())
        {
            Assert.True(aNExp.More(), $"native exploration of {theType} ended first");
            Assert.Equal(aNExp.Depth(), anExp.Depth());
            aList.Add(new(aNExp.Current(), anExp.Current()));
        }

        Assert.False(aNExp.More(), $"port exploration of {theType} ended first");
        return aList;
    }

    private static void CompareFlags(Oracle theOracle, Twin<N.TopoDS_Shape, TopoDS_Shape> theShape)
    {
        foreach (TopAbs_ShapeEnum aType in theTypes)
        {
            foreach (Twin<N.TopoDS_Shape, TopoDS_Shape> aS in Explore(theShape, aType))
            {
                theOracle.Same(aS.N.Free(), aS.P.Free(), "Free");
                theOracle.Same(aS.N.Modified(), aS.P.Modified(), "Modified");
                theOracle.Same(aS.N.Checked(), aS.P.Checked(), "Checked");
                theOracle.Same(aS.N.Orientable(), aS.P.Orientable(), "Orientable");
                theOracle.Same(aS.N.Closed(), aS.P.Closed(), "Closed");
                theOracle.Same(aS.N.Infinite(), aS.P.Infinite(), "Infinite");
                theOracle.Same(aS.N.Convex(), aS.P.Convex(), "Convex");
                theOracle.Same(aS.N.Locked(), aS.P.Locked(), "Locked (unfrozen)");
            }
        }
    }

    private static void CompareTopology(Oracle theOracle, Twin<N.TopoDS_Shape, TopoDS_Shape> theShape, Counts theCounts)
    {
        foreach (TopAbs_ShapeEnum aType in theTypes)
        {
            foreach (TopAbs_ShapeEnum anAvoid in new[] { TopAbs_SHAPE, TopAbs_FACE, TopAbs_EDGE })
            {
                List<Twin<N.TopoDS_Shape, TopoDS_Shape>> aShapes = Explore(theShape, aType, anAvoid);
                if (anAvoid != TopAbs_SHAPE)
                {
                    continue;
                }

                theCounts.Shapes += aShapes.Count;
                var aNMap = new N.TopTools_IndexedMapOfShape();
                var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
                foreach (Twin<N.TopoDS_Shape, TopoDS_Shape> aS in aShapes)
                {
                    theOracle.Same((int)aS.N.ShapeType(), (int)aS.P.ShapeType(), "ShapeType");
                    theOracle.Same((int)aS.N.Orientation(), (int)aS.P.Orientation(), "Orientation");
                    theOracle.Same(aS.N.NbChildren(), aS.P.NbChildren(), "NbChildren");
                    theOracle.Same(aS.N.Location().IsIdentity(), aS.P.Location().IsIdentity(), "Location.IsIdentity");
                    using (N.gp_Trsf aNT = aS.N.Location().Transformation())
                    {
                        theOracle.Same(aNT, aS.P.Location().Transformation(), "Location");
                    }

                    theOracle.Same(aNMap.Add(aS.N), aMap.Add(aS.P), "IndexedMapOfShape.Add");
                    theOracle.Same(aS.N.Locked(), aS.P.TShape()!.LockedFlag(), "Locked (stored bit)");
                    Assert.True(aS.P.Locked());
                }

                // identity: IsPartner, IsSame and IsEqual between every two, and the hash agrees with IsSame
                for (int i = 0; i < aShapes.Count && i < 40; i++)
                {
                    for (int j = i; j < aShapes.Count && j < 40; j++)
                    {
                        Twin<N.TopoDS_Shape, TopoDS_Shape> a = aShapes[i], b = aShapes[j];
                        theOracle.Same(a.N.IsPartner(b.N), a.P.IsPartner(b.P), "IsPartner");
                        theOracle.Same(a.N.IsSame(b.N), a.P.IsSame(b.P), "IsSame");
                        theOracle.Same(a.N.IsEqual(b.N), a.P.IsEqual(b.P), "IsEqual");
                        theOracle.Same(a.N.IsEqual(b.N), a.P == b.P, "operator ==");
                        if (a.P.IsSame(b.P))
                        {
                            Assert.Equal(a.P.GetHashCode(), b.P.GetHashCode());
                        }

                        theCounts.Pairs++;
                    }
                }

                // the index maps of TopExp agree, with and without the cumulative orientation and location
                var aNMap2 = new N.TopTools_IndexedMapOfShape();
                N.TopExp.MapShapes(theShape.N, TwinShapes.ToNative(aType), aNMap2);
                var aMap2 = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
                TopExp.MapShapes(theShape.P, aType, aMap2);
                theOracle.Same(aNMap2.Extent(), aMap2.Extent(), "MapShapes.Extent");
                for (int i = 1; i <= aMap2.Extent(); i++)
                {
                    N.TopoDS_Shape aNKey = aNMap2.FindKey(i);
                    TopoDS_Shape aKey = aMap2.FindKey(i);
                    theOracle.Same(aShapes.FindIndex(s => s.N.IsEqual(aNKey)), aShapes.FindIndex(s => s.P.IsEqual(aKey)), "MapShapes order");
                }
            }
        }

        foreach ((bool aCumOri, bool aCumLoc) in new[] { (true, true), (false, true), (true, false), (false, false) })
        {
            var aNAll = new N.TopTools_IndexedMapOfShape();
            N.TopExp.MapShapes(theShape.N, aNAll, aCumOri, aCumLoc);
            var anAll = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            TopExp.MapShapes(theShape.P, anAll, aCumOri, aCumLoc);
            theOracle.Same(aNAll.Extent(), anAll.Extent(), "MapShapes (all).Extent");
            for (int i = 1; i <= anAll.Extent(); i++)
            {
                N.TopoDS_Shape aN = aNAll.FindKey(i);
                TopoDS_Shape aP = anAll.FindKey(i);
                theOracle.Same((int)aN.ShapeType(), (int)aP.ShapeType(), "MapShapes (all) type");
                theOracle.Same((int)aN.Orientation(), (int)aP.Orientation(), "MapShapes (all) orientation");
                using N.gp_Trsf aNT = aN.Location().Transformation();
                theOracle.Same(aNT, aP.Location().Transformation(), "MapShapes (all) location");
            }
        }

        // ancestors
        foreach ((TopAbs_ShapeEnum aTS, TopAbs_ShapeEnum aTA) in new[] { (TopAbs_EDGE, TopAbs_FACE), (TopAbs_VERTEX, TopAbs_EDGE), (TopAbs_FACE, TopAbs_SOLID) })
        {
            var aNAnc = new N.TopTools_IndexedDataMapOfShapeListOfShape();
            N.TopExp.MapShapesAndAncestors(theShape.N, TwinShapes.ToNative(aTS), TwinShapes.ToNative(aTA), aNAnc);
            var anAnc = new NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
            TopExp.MapShapesAndAncestors(theShape.P, aTS, aTA, anAnc);
            var aNUnique = new N.TopTools_IndexedDataMapOfShapeListOfShape();
            N.TopExp.MapShapesAndUniqueAncestors(theShape.N, TwinShapes.ToNative(aTS), TwinShapes.ToNative(aTA), aNUnique, true);
            var anUnique = new NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
            TopExp.MapShapesAndUniqueAncestors(theShape.P, aTS, aTA, anUnique, true);
            theOracle.Same(aNAnc.Extent(), anAnc.Extent(), "MapShapesAndAncestors.Extent");
            theOracle.Same(aNUnique.Extent(), anUnique.Extent(), "MapShapesAndUniqueAncestors.Extent");
            for (int i = 1; i <= anAnc.Extent(); i++)
            {
                theOracle.Same(aNAnc.FindFromIndex(i).Extent(), anAnc.FindFromIndex(i).Extent(), "ancestors");
                theOracle.Same(aNUnique.FindFromIndex(i).Extent(), anUnique.FindFromIndex(i).Extent(), "unique ancestors");
            }
        }

        foreach (Twin<N.TopoDS_Shape, TopoDS_Shape> aW in Explore(theShape, TopAbs_WIRE))
        {
            var aNV1 = new N.TopoDS_Vertex();
            var aNV2 = new N.TopoDS_Vertex();
            N.TopExp.Vertices(N.TopoDS.Wire(aW.N), aNV1, aNV2);
            TopExp.Vertices(TopoDS.Wire(aW.P), out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);
            CompareVertex(theOracle, aNV1, aV1, "wire's first vertex");
            CompareVertex(theOracle, aNV2, aV2, "wire's last vertex");
            theOracle.Same(N.BRep_Tool.IsClosed(aW.N), BRep_Tool.IsClosed(aW.P), "IsClosed(wire)");
        }

        foreach (TopAbs_ShapeEnum aType in new[] { TopAbs_SHELL, TopAbs_SOLID, TopAbs_COMPOUND, TopAbs_FACE })
        {
            foreach (Twin<N.TopoDS_Shape, TopoDS_Shape> aS in Explore(theShape, aType))
            {
                theOracle.Same(N.BRep_Tool.IsClosed(aS.N), BRep_Tool.IsClosed(aS.P), $"IsClosed({aType})");
            }
        }

        var aN3D = new N.TopTools_IndexedMapOfShape();
        N.BRepTools.Map3DEdges(theShape.N, aN3D);
        var a3D = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        BRepTools.Map3DEdges(theShape.P, a3D);
        theOracle.Same(aN3D.Extent(), a3D.Extent(), "Map3DEdges");
    }

    private static void CompareVertex(Oracle theOracle, N.TopoDS_Vertex theN, in TopoDS_Vertex theP, string theWhat)
    {
        theOracle.Same(theN.IsNull(), theP.IsNull(), theWhat + " IsNull");
        if (theP.IsNull())
        {
            return;
        }

        theOracle.Same((int)theN.Orientation(), (int)theP.Orientation(), theWhat + " orientation");
        using N.gp_Pnt aNP = N.BRep_Tool.Pnt(theN);
        theOracle.Same(aNP, BRep_Tool.Pnt(theP), theWhat);
        theOracle.Same(N.BRep_Tool.Tolerance(theN), BRep_Tool.Tolerance(theP), theWhat + " tolerance");
    }

    private static void CompareGeometry(Oracle theOracle, Twin<N.TopoDS_Shape, TopoDS_Shape> theShape, Counts theCounts)
    {
        foreach (TopAbs_ShapeEnum aType in new[] { TopAbs_VERTEX, TopAbs_EDGE, TopAbs_FACE })
        {
            theOracle.Same(N.BRep_Tool.MaxTolerance(theShape.N, TwinShapes.ToNative(aType)), BRep_Tool.MaxTolerance(theShape.P, aType),
                           $"MaxTolerance({aType})");
        }

        List<Twin<N.TopoDS_Shape, TopoDS_Shape>> aVertices = Explore(theShape, TopAbs_VERTEX);
        foreach (Twin<N.TopoDS_Shape, TopoDS_Shape> aV in aVertices)
        {
            CompareVertex(theOracle, N.TopoDS.Vertex(aV.N), TopoDS.Vertex(aV.P), "vertex");
        }

        for (int i = 0; i + 1 < aVertices.Count && i < 20; i++)
        {
            theOracle.Same(N.BRepTools.Compare(N.TopoDS.Vertex(aVertices[i].N), N.TopoDS.Vertex(aVertices[i + 1].N)),
                           BRepTools.Compare(TopoDS.Vertex(aVertices[i].P), TopoDS.Vertex(aVertices[i + 1].P)), "Compare(vertices)");
        }

        List<Twin<N.TopoDS_Shape, TopoDS_Shape>> anEdges = Explore(theShape, TopAbs_EDGE);
        for (int k = 0; k < anEdges.Count; k++)
        {
            N.TopoDS_Edge aNE = N.TopoDS.Edge(anEdges[k].N);
            TopoDS_Edge anE = TopoDS.Edge(anEdges[k].P);
            CompareEdge(theOracle, aNE, anE);
            theCounts.Edges++;
            if (k + 1 < anEdges.Count)
            {
                N.TopoDS_Edge aNE2 = N.TopoDS.Edge(anEdges[k + 1].N);
                TopoDS_Edge anE2 = TopoDS.Edge(anEdges[k + 1].P);
                var aNCommon = new N.TopoDS_Vertex();
                TopoDS_Vertex aCommon = new();
                theOracle.Same(N.TopExp.CommonVertex(aNE, aNE2, aNCommon), TopExp.CommonVertex(anE, anE2, ref aCommon), "CommonVertex");
                CompareVertex(theOracle, aNCommon, aCommon, "common vertex");
            }
        }

        foreach (Twin<N.TopoDS_Shape, TopoDS_Shape> aF in Explore(theShape, TopAbs_FACE))
        {
            N.TopoDS_Face aNF = N.TopoDS.Face(aF.N);
            TopoDS_Face aPF = TopoDS.Face(aF.P);
            CompareFace(theOracle, aNF, aPF, theCounts);
        }
    }

    private static void CompareEdge(Oracle theOracle, N.TopoDS_Edge theNE, in TopoDS_Edge theE)
    {
        theOracle.Same(N.BRep_Tool.Tolerance(theNE), BRep_Tool.Tolerance(theE), "edge tolerance");
        theOracle.Same(N.BRep_Tool.SameParameter(theNE), BRep_Tool.SameParameter(theE), "SameParameter");
        theOracle.Same(N.BRep_Tool.SameRange(theNE), BRep_Tool.SameRange(theE), "SameRange");
        theOracle.Same(N.BRep_Tool.Degenerated(theNE), BRep_Tool.Degenerated(theE), "Degenerated");
        theOracle.Same(N.BRep_Tool.IsGeometric(theNE), BRep_Tool.IsGeometric(theE), "IsGeometric(edge)");
        theOracle.Same(N.BRep_Tool.HasContinuity(theNE), BRep_Tool.HasContinuity(theE), "HasContinuity");
        theOracle.Same(N.BRep_Tool.IsClosed(theNE), BRep_Tool.IsClosed(theE), "IsClosed(edge)");
        double aNF = 0, aNL = 0;
        N.BRep_Tool.Range(theNE, ref aNF, ref aNL);
        BRep_Tool.Range(theE, out double f, out double l);
        theOracle.Same(aNF, f, "Range.First");
        theOracle.Same(aNL, l, "Range.Last");

        // the curve and its location, and the placed curve
        using var aNLoc = new N.TopLoc_Location();
        double aNF2 = 0, aNL2 = 0;
        N.Geom_Curve? aNC = N.BRep_Tool.Curve(theNE, aNLoc, ref aNF2, ref aNL2);
        Geom_Curve? aC = BRep_Tool.Curve(theE, out TopLoc_Location aLoc, out double f2, out double l2);
        theOracle.Same(aNC is null, aC is null, "Curve is null");
        using (N.gp_Trsf aNT = aNLoc.Transformation())
        {
            theOracle.Same(aNT, aLoc.Transformation(), "Curve location");
        }

        theOracle.Same(aNF2, f2, "Curve.First");
        theOracle.Same(aNL2, l2, "Curve.Last");
        N.Geom_Curve? aNPlaced = N.BRep_Tool.Curve(theNE, ref aNF2, ref aNL2);
        Geom_Curve? aPlaced = BRep_Tool.Curve(theE, out f2, out l2);
        if (aPlaced is not null)
        {
            foreach (double t in new[] { f2, 0.3 * f2 + 0.7 * l2, l2 })
            {
                using N.gp_Pnt aNP = aNPlaced!.Value(t);
                theOracle.Same(aNP, aPlaced.Value(t), "placed curve value");
            }
        }

        // the vertices and their parameters
        var aNV1 = new N.TopoDS_Vertex();
        var aNV2 = new N.TopoDS_Vertex();
        N.TopExp.Vertices(theNE, aNV1, aNV2);
        TopExp.Vertices(theE, out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);
        CompareVertex(theOracle, aNV1, aV1, "edge's first vertex");
        CompareVertex(theOracle, aNV2, aV2, "edge's last vertex");
        var aNV1c = new N.TopoDS_Vertex();
        var aNV2c = new N.TopoDS_Vertex();
        N.TopExp.Vertices(theNE, aNV1c, aNV2c, true);
        TopExp.Vertices(theE, out TopoDS_Vertex aV1c, out TopoDS_Vertex aV2c, true);
        CompareVertex(theOracle, aNV1c, aV1c, "edge's first vertex (cumulative)");
        CompareVertex(theOracle, aNV2c, aV2c, "edge's last vertex (cumulative)");
        CompareVertex(theOracle, N.TopExp.FirstVertex(theNE, true), TopExp.FirstVertex(theE, true), "FirstVertex");
        CompareVertex(theOracle, N.TopExp.LastVertex(theNE), TopExp.LastVertex(theE), "LastVertex");
        if (!aV1.IsNull())
        {
            theOracle.Same(N.BRep_Tool.Parameter(aNV1, theNE), BRep_Tool.Parameter(aV1, theE), "Parameter(V1, E)");
            theOracle.Same(N.BRep_Tool.Parameter(aNV2, theNE), BRep_Tool.Parameter(aV2, theE), "Parameter(V2, E)");
        }
    }

    private static void CompareFace(Oracle theOracle, N.TopoDS_Face theNF, in TopoDS_Face theF, Counts theCounts)
    {
        theCounts.Faces++;
        theOracle.Same(N.BRep_Tool.Tolerance(theNF), BRep_Tool.Tolerance(theF), "face tolerance");
        theOracle.Same(N.BRep_Tool.NaturalRestriction(theNF), BRep_Tool.NaturalRestriction(theF), "NaturalRestriction");
        theOracle.Same(N.BRep_Tool.IsGeometric(theNF), BRep_Tool.IsGeometric(theF), "IsGeometric(face)");

        double aNU1 = 0, aNU2 = 0, aNV1 = 0, aNV2 = 0;
        N.BRepTools.UVBounds(theNF, ref aNU1, ref aNU2, ref aNV1, ref aNV2);
        BRepTools.UVBounds(theF, out double u1, out double u2, out double v1, out double v2);
        theOracle.Same(aNU1, u1, "UVBounds UMin");
        theOracle.Same(aNU2, u2, "UVBounds UMax");
        theOracle.Same(aNV1, v1, "UVBounds VMin");
        theOracle.Same(aNV2, v2, "UVBounds VMax");

        bool aNUC = false, aNVC = false;
        N.BRepTools.DetectClosedness(theNF, ref aNUC, ref aNVC);
        BRepTools.DetectClosedness(theF, out bool aUC, out bool aVC);
        theOracle.Same(aNUC, aUC, "DetectClosedness U");
        theOracle.Same(aNVC, aVC, "DetectClosedness V");

        // the outer wire, by its position among the face's wires
        N.TopoDS_Wire aNOuter = N.BRepTools.OuterWire(theNF);
        TopoDS_Wire anOuter = BRepTools.OuterWire(theF);
        var aNWires = new N.TopExp_Explorer(theNF, N.TopAbs_ShapeEnum.TopAbs_WIRE);
        var aWires = new TopExp_Explorer(theF, TopAbs_WIRE);
        for (; aWires.More(); aWires.Next(), aNWires.Next())
        {
            theOracle.Same(aNWires.Current().IsSame(aNOuter), aWires.Current().IsSame(anOuter), "OuterWire");
            aNU1 = aNU2 = aNV1 = aNV2 = 0;
            N.BRepTools.UVBounds(theNF, N.TopoDS.Wire(aNWires.Current()), ref aNU1, ref aNU2, ref aNV1, ref aNV2);
            BRepTools.UVBounds(theF, TopoDS.Wire(aWires.Current()), out u1, out u2, out v1, out v2);
            theOracle.Same(aNU1, u1, "UVBounds(wire) UMin");
            theOracle.Same(aNU2, u2, "UVBounds(wire) UMax");
            theOracle.Same(aNV1, v1, "UVBounds(wire) VMin");
            theOracle.Same(aNV2, v2, "UVBounds(wire) VMax");
        }

        // the surface, placed, and the adaptor
        N.Geom_Surface? aNS = N.BRep_Tool.Surface(theNF);
        Geom_Surface? aS = BRep_Tool.Surface(theF);
        using var aNA = new N.BRepAdaptor_Surface(theNF);
        var anA = new BRepAdaptor_Surface(theF);
        theOracle.Same(aNA.FirstUParameter(), anA.FirstUParameter(), "adaptor FirstU");
        theOracle.Same(aNA.LastUParameter(), anA.LastUParameter(), "adaptor LastU");
        theOracle.Same(aNA.FirstVParameter(), anA.FirstVParameter(), "adaptor FirstV");
        theOracle.Same(aNA.LastVParameter(), anA.LastVParameter(), "adaptor LastV");
        theOracle.Same((int)aNA.GetType_(), (int)anA.GetType(), "adaptor type");
        theOracle.Same(aNA.Tolerance(), anA.Tolerance(), "adaptor tolerance");
        foreach ((double a, double b) in new[] { (0.0, 0.0), (0.25, 0.75), (0.9, 0.1) })
        {
            double u = u1 + a * (u2 - u1), v = v1 + b * (v2 - v1);
            using N.gp_Pnt aNP = aNA.Value(u, v);
            theOracle.Same(aNP, anA.Value(u, v), "adaptor value");
            using N.gp_Pnt aNP2 = aNS!.Value(u, v);
            theOracle.Same(aNP2, aS!.Value(u, v), "placed surface value");
        }

        // the vertices' parameters on the face
        var aNVertices = new N.TopExp_Explorer(theNF, N.TopAbs_ShapeEnum.TopAbs_VERTEX);
        for (var aVertices = new TopExp_Explorer(theF, TopAbs_VERTEX); aVertices.More(); aVertices.Next(), aNVertices.Next())
        {
            using N.gp_Pnt2d aNUV = N.BRep_Tool.Parameters(N.TopoDS.Vertex(aNVertices.Current()), theNF);
            theOracle.Same(aNUV, BRep_Tool.Parameters(TopoDS.Vertex(aVertices.Current()), theF), "Parameters(V, F)");
        }

        // the edges on the face
        var aNEdges = new N.TopExp_Explorer(theNF, N.TopAbs_ShapeEnum.TopAbs_EDGE);
        for (var anEdges = new TopExp_Explorer(theF, TopAbs_EDGE); anEdges.More(); anEdges.Next(), aNEdges.Next())
        {
            theCounts.EdgeFaces++;
            N.TopoDS_Edge aNE = N.TopoDS.Edge(aNEdges.Current());
            TopoDS_Edge anE = TopoDS.Edge(anEdges.Current());
            double aNf = 0, aNl = 0;
            N.Geom2d_Curve? aNPC = N.BRep_Tool.CurveOnSurface(aNE, theNF, ref aNf, ref aNl);
            Geom2d_Curve? aPC = BRep_Tool.CurveOnSurface(anE, theF, out double f, out double l);
            theOracle.Same(aNPC is null, aPC is null, "CurveOnSurface is null");
            theOracle.Same(aNf, f, "CurveOnSurface.First");
            theOracle.Same(aNl, l, "CurveOnSurface.Last");
            foreach (double t in new[] { f, 0.6 * f + 0.4 * l, l })
            {
                using N.gp_Pnt2d aNP = aNPC!.Value(t);
                theOracle.Same(aNP, aPC!.Value(t), "pcurve value");
            }

            using var aNP1 = new N.gp_Pnt2d();
            using var aNP2 = new N.gp_Pnt2d();
            N.BRep_Tool.UVPoints(aNE, theNF, aNP1, aNP2);
            BRep_Tool.UVPoints(anE, theF, out gp_Pnt2d aP1, out gp_Pnt2d aP2);
            theOracle.Same(aNP1, aP1, "UVPoints first");
            theOracle.Same(aNP2, aP2, "UVPoints last");
            theOracle.Same(N.BRep_Tool.IsClosed(aNE, theNF), BRep_Tool.IsClosed(anE, theF), "IsClosed(E, F)");
            theOracle.Same(N.BRepTools.IsReallyClosed(aNE, theNF), BRepTools.IsReallyClosed(anE, theF), "IsReallyClosed");
            theOracle.Same((int)N.BRepTools.OriEdgeInFace(aNE, theNF), (int)BRepTools.OriEdgeInFace(anE, theF), "OriEdgeInFace");
            theOracle.Same(N.BRep_Tool.HasContinuity(aNE, theNF, theNF), BRep_Tool.HasContinuity(anE, theF, theF), "HasContinuity(E, F, F)");
            theOracle.Same((int)N.BRep_Tool.Continuity(aNE, theNF, theNF), (int)BRep_Tool.Continuity(anE, theF, theF), "Continuity(E, F, F)");

            double aNr1 = 0, aNr2 = 0;
            N.BRep_Tool.Range(aNE, theNF, ref aNr1, ref aNr2);
            BRep_Tool.Range(anE, theF, out double r1, out double r2);
            theOracle.Same(aNr1, r1, "Range(E, F).First");
            theOracle.Same(aNr2, r2, "Range(E, F).Last");

            var aNEV1 = new N.TopoDS_Vertex();
            var aNEV2 = new N.TopoDS_Vertex();
            N.TopExp.Vertices(aNE, aNEV1, aNEV2);
            TopExp.Vertices(anE, out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);
            theOracle.Same(N.BRep_Tool.Parameter(aNEV1, aNE, theNF), BRep_Tool.Parameter(aV1, anE, theF), "Parameter(V1, E, F)");
            theOracle.Same(N.BRep_Tool.Parameter(aNEV2, aNE, theNF), BRep_Tool.Parameter(aV2, anE, theF), "Parameter(V2, E, F)");

            aNf = aNl = 0;
            double aU1 = 0, aU2 = 0;
            N.BRepTools.UVBounds(theNF, aNE, ref aNf, ref aNl, ref aU1, ref aU2);
            BRepTools.UVBounds(theF, anE, out double eu1, out double eu2, out double ev1, out double ev2);
            theOracle.Same(aNf, eu1, "UVBounds(edge) UMin");
            theOracle.Same(aNl, eu2, "UVBounds(edge) UMax");
            theOracle.Same(aU1, ev1, "UVBounds(edge) VMin");
            theOracle.Same(aU2, ev2, "UVBounds(edge) VMax");
        }
    }
}
#endif
