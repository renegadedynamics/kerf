// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Xunit.Abstractions;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.Oracle.Tests;

/// <summary>
/// The topology core against native OCCT: random shapes built with BRep_Builder identically on both sides
/// (<see cref="TwinShapes"/>: boxes, cylinder faces with seams, faces with holes, compounds sharing sub-shapes at
/// several locations and orientations), then the port's TopoDS, TopExp, BRep_Tool, BRepTools and BRepAdaptor_Surface
/// answers compared with native OCCT's. The port's shapes are frozen first, as every shape a caller holds is.
/// </summary>
public class TopologyOracleTests(ITestOutputHelper output)
{
    private const int Scenes = 60;

    private static readonly ShapeType[] theTypes =
        [ShapeType.Compound, ShapeType.CompSolid, ShapeType.Solid, ShapeType.Shell, ShapeType.Face, ShapeType.Wire, ShapeType.Edge, ShapeType.Vertex];

    internal sealed class Counts
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
        foreach (int aScene in Tape.Budget(Scenes))
        {
            Twin<N.TopoDS_Shape, Shape> aShape = RandomScene(anOracle, aScene);
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

    internal static Twin<N.TopoDS_Shape, Shape> RandomScene(Oracle theOracle, int theScene)
    {
        var aTwins = new TwinShapes();
        switch (theScene % 4)
        {
            case 0:
                return Placed(theOracle, TwinShapes.Up(aTwins.Box(theOracle.Real(0.5, 20), theOracle.Real(0.5, 20), theOracle.Real(0.5, 20))));
            case 1:
            {
                (N.TopLoc_Location aNL, Placement aL) = RandomLocation(theOracle);
                return Placed(theOracle, TwinShapes.Up(aTwins.CylinderFace(theOracle.Real(0.5, 5), theOracle.Real(0.5, 5), aL, aNL)));
            }
            case 2:
                return Placed(theOracle, TwinShapes.Up(FaceWithHole(theOracle, aTwins)));
            default:
            {
                // a compound sharing a box and a face at several locations and orientations
                Twin<N.TopoDS_Shape, Shape> aBox = TwinShapes.Up(aTwins.Box(theOracle.Real(1, 5), theOracle.Real(1, 5), theOracle.Real(1, 5)));
                (N.TopLoc_Location aNL, Placement aL) = RandomLocation(theOracle);
                Twin<N.TopoDS_Shape, Shape> aFace = TwinShapes.Up(aTwins.CylinderFace(1.0, 2.0, aL, aNL));
                (N.TopLoc_Location aNL1, Placement aL1) = RandomLocation(theOracle);
                (N.TopLoc_Location aNL2, Placement aL2) = RandomLocation(theOracle);
                Twin<N.TopoDS_Compound, Compound> aCompound = aTwins.Compound(
                    aBox,
                    new(aBox.N.Moved(aNL1), aBox.P.Moved(aL1)),
                    new(aBox.N.Moved(aNL2).Reversed(), aBox.P.Moved(aL2).Reversed()),
                    aFace,
                    new(aFace.N.Moved(aNL1).Oriented(N.TopAbs_Orientation.TopAbs_INTERNAL), aFace.P.Moved(aL1).Oriented(Internal)));
                return Placed(theOracle, TwinShapes.Up(aCompound));
            }
        }
    }

    /// <summary>The shape at a random location (or none), and reversed half the time.</summary>
    private static Twin<N.TopoDS_Shape, Shape> Placed(Oracle theOracle, Twin<N.TopoDS_Shape, Shape> theShape)
    {
        Twin<N.TopoDS_Shape, Shape> aShape = theShape;
        if (theOracle.Bool())
        {
            (N.TopLoc_Location aNL, Placement aL) = RandomLocation(theOracle);
            aShape = new(aShape.N.Moved(aNL), aShape.P.Moved(aL));
        }

        return theOracle.Bool() ? new(aShape.N.Reversed(), aShape.P.Reversed()) : aShape;
    }

    /// <summary>A rigid location (a rotation, a translation or both) built from the same raw numbers on both sides.</summary>
    internal static (N.TopLoc_Location, Placement) RandomLocation(Oracle theOracle)
    {
        var aNT = new N.gp_Trsf();
        var aT = new Transform3D();
        if (theOracle.Bool())
        {
            (N.gp_Ax1 aNAx, Axis3D anAx) = GeomOracleInputs.Ax1s(theOracle);
            double anAngle = theOracle.Angle();
            using (aNAx)
            {
                aNT.SetRotation(aNAx, anAngle);
            }

            aT = aT.WithRotation(anAx, anAngle);
        }

        if (theOracle.Bool())
        {
            Vector3D aV = theOracle.Vec();
            using var aNV = Oracle.ToNative(aV);
            var aNT2 = new N.gp_Trsf();
            aNT2.SetTranslation(aNV);
            aNT = aNT2.Multiplied(aNT);
            aT = Transform3D.Translation(aV).Multiplied(aT);
        }

        return (new N.TopLoc_Location(aNT), new Placement(aT));
    }

    /// <summary>A square face with a rectangular hole, on a random plane (the hole's wire first).</summary>
    private static Twin<N.TopoDS_Face, Face> FaceWithHole(Oracle theOracle, TwinShapes theTwins)
    {
        Point3D anOrigin = theOracle.Pnt();
        global::Kerf.Engine.Geometry.Xyz aNormal = GeomOracleInputs.RawDir(theOracle);
        global::Kerf.Engine.Geometry.Xyz aRef = GeomOracleInputs.RawDir(theOracle);
        global::Kerf.Engine.Geometry.Xyz anX = aNormal.Crossed(aRef);
        while (anX.Modulus() < 0.1)
        {
            aRef = GeomOracleInputs.RawDir(theOracle);
            anX = aNormal.Crossed(aRef);
        }

        // the corners in the plane's own coordinates, placed by the port's plane (both sides get these points)
        var aPln = new Plane3D(new CoordinateSystem(anOrigin, new Direction3D(aNormal), new Direction3D(anX)));
        double aSize = theOracle.Real(4, 10);
        double hu1 = theOracle.Real(0.5, 1.5), hu2 = theOracle.Real(2, 3.5), hv1 = theOracle.Real(0.5, 1.5), hv2 = theOracle.Real(2, 3.5);
        (double U, double V)[] aUV = [(0, 0), (aSize, 0), (aSize, aSize), (0, aSize), (hu1, hv1), (hu1, hv2), (hu2, hv2), (hu2, hv1)];
        var aPoints = new Point3D[8];
        var aV = new Twin<N.TopoDS_Vertex, Vertex>[8];
        for (int i = 0; i < 8; i++)
        {
            aPoints[i] = global::Kerf.Engine.Geometry.ElSLib.Value(aUV[i].U, aUV[i].V, aPln);
            aV[i] = theTwins.Vertex(aPoints[i], 1e-6);
        }

        return theTwins.PolygonFace(aPoints, aV, new Dictionary<(int, int), Twin<N.TopoDS_Edge, Edge>>(),
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
        foreach (int aRound in Tape.Budget(20))
        {
            var aTwins = new TwinShapes();
            Twin<N.TopoDS_Shape, Shape> aBox =
                TwinShapes.Up(aTwins.Box(anOracle.Real(1, 5), anOracle.Real(1, 5), anOracle.Real(1, 5)));
            List<Twin<N.TopoDS_Shape, Shape>> aFaces = Explore(aBox, ShapeType.Face);
            foreach (Twin<N.TopoDS_Shape, Shape> aF in aFaces)
            {
                if (anOracle.Bool())
                {
                    aTwins.AttachTriangulation(new(N.TopoDS.Face(aF.N), ShapeCasts.Face(aF.P)), anOracle.Real(0.01, 1));
                }
            }

            double aDefl = anOracle.Real(0.01, 1);
            anOracle.Same(N.BRepTools.Triangulation(aBox.N, aDefl), BrepTools.Triangulation(aBox.P, aDefl), "Triangulation");

            N.BRepTools.Update(aBox.N);
            BrepTools.Update(aBox.P);
            CompareFlags(anOracle, aBox);

            N.BRepTools.Clean(aBox.N);
            BrepTools.Clean(aBox.P);
            foreach (Twin<N.TopoDS_Shape, Shape> aF in aFaces)
            {
                using var aNL = new N.TopLoc_Location();
                anOracle.Same(N.BRep_Tool.Triangulation(N.TopoDS.Face(aF.N), aNL) is null,
                              Tool.Triangulation(ShapeCasts.Face(aF.P), out _) is null, "Triangulation after Clean");
            }

            CompareFlags(anOracle, aBox);

            if (aRound % 2 == 0)
            {
                N.BRepTools.CleanGeometry(aBox.N);
                BrepTools.CleanGeometry(aBox.P);
                foreach (Twin<N.TopoDS_Shape, Shape> aF in aFaces)
                {
                    anOracle.Same(N.BRep_Tool.IsGeometric(N.TopoDS.Face(aF.N)), Tool.IsGeometric(ShapeCasts.Face(aF.P)), "IsGeometric(face)");
                }

                foreach (Twin<N.TopoDS_Shape, Shape> anE in Explore(aBox, ShapeType.Edge))
                {
                    anOracle.Same(N.BRep_Tool.IsGeometric(N.TopoDS.Edge(anE.N)), Tool.IsGeometric(ShapeCasts.Edge(anE.P)), "IsGeometric(edge)");
                }

                CompareFlags(anOracle, aBox);
            }

            // internal sub-shapes: a compound of the box, an internal face of it and an internal edge sharing its vertices
            Twin<N.TopoDS_Compound, Compound> aCompound = aTwins.Compound(aBox);
            Twin<N.TopoDS_Shape, Shape> aFace = aFaces[anOracle.Int(0, 5)];
            aTwins.Add(TwinShapes.Up(aCompound), new(aFace.N.Oriented(N.TopAbs_Orientation.TopAbs_INTERNAL), aFace.P.Oriented(Internal)));
            Twin<N.TopoDS_Shape, Shape> anEdge = Explore(aFace, ShapeType.Edge)[anOracle.Int(0, 3)];
            aTwins.Add(TwinShapes.Up(aCompound), new(anEdge.N.Oriented(N.TopAbs_Orientation.TopAbs_INTERNAL), anEdge.P.Oriented(Internal)));
            bool isForce = anOracle.Bool();
            N.BRepTools.RemoveInternals(aCompound.N, isForce);
            BrepTools.RemoveInternals(aCompound.P, isForce);
            foreach (ShapeType aType in theTypes)
            {
                Explore(TwinShapes.Up(aCompound), aType);
            }

            anOracle.Same(aCompound.N.NbChildren(), aCompound.P.ChildCount, "RemoveInternals");
            CompareFlags(anOracle, TwinShapes.Up(aCompound));
        }

        anOracle.Report("BRepTools in-place statics");
    }

    // Comparisons

    internal static List<Twin<N.TopoDS_Shape, Shape>> Explore(Twin<N.TopoDS_Shape, Shape> theShape,
                                                                    ShapeType theType,
                                                                    ShapeType theAvoid = ShapeType.Shape)
    {
        var aList = new List<Twin<N.TopoDS_Shape, Shape>>();
        var aNExp = new N.TopExp_Explorer(theShape.N, TwinShapes.ToNative(theType), TwinShapes.ToNative(theAvoid));
        var anExp = new Explorer(theShape.P, theType, theAvoid);
        for (; anExp.More(); anExp.Next(), aNExp.Next())
        {
            Assert.True(aNExp.More(), $"native exploration of {theType} ended first");
            Assert.Equal(aNExp.Depth(), anExp.Depth());
            aList.Add(new(aNExp.Current(), anExp.Current()));
        }

        Assert.False(aNExp.More(), $"port exploration of {theType} ended first");
        return aList;
    }

    // A frozen shape is Locked; with theStoredLocked the stored bit is compared (a result that shares frozen TShapes).
    internal static void CompareFlags(Oracle theOracle, Twin<N.TopoDS_Shape, Shape> theShape, bool theStoredLocked = false)
    {
        foreach (ShapeType aType in theTypes)
        {
            foreach (Twin<N.TopoDS_Shape, Shape> aS in Explore(theShape, aType))
            {
                theOracle.Same(aS.N.Free(), aS.P.Free(), "Free");
                theOracle.Same(aS.N.Modified(), aS.P.Modified(), "Modified");
                theOracle.Same(aS.N.Checked(), aS.P.Checked(), "Checked");
                theOracle.Same(aS.N.Orientable(), aS.P.Orientable(), "Orientable");
                theOracle.Same(aS.N.Closed(), aS.P.Closed(), "Closed");
                theOracle.Same(aS.N.Infinite(), aS.P.Infinite(), "Infinite");
                theOracle.Same(aS.N.Convex(), aS.P.Convex(), "Convex");
                theOracle.Same(aS.N.Locked(), theStoredLocked ? aS.P.TShape()!.LockedFlag() : aS.P.Locked(), "Locked (unfrozen)");
            }
        }
    }

    internal static void CompareTopology(Oracle theOracle, Twin<N.TopoDS_Shape, Shape> theShape, Counts theCounts)
    {
        foreach (ShapeType aType in theTypes)
        {
            foreach (ShapeType anAvoid in new[] { ShapeType.Shape, ShapeType.Face, ShapeType.Edge })
            {
                List<Twin<N.TopoDS_Shape, Shape>> aShapes = Explore(theShape, aType, anAvoid);
                if (anAvoid != ShapeType.Shape)
                {
                    continue;
                }

                theCounts.Shapes += aShapes.Count;
                var aNMap = new N.TopTools_IndexedMapOfShape();
                var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
                foreach (Twin<N.TopoDS_Shape, Shape> aS in aShapes)
                {
                    theOracle.Same((int)aS.N.ShapeType(), (int)aS.P.ShapeType(), "ShapeType");
                    theOracle.Same((int)aS.N.Orientation(), (int)aS.P.Orientation(), "Orientation");
                    theOracle.Same(aS.N.NbChildren(), aS.P.ChildCount, "NbChildren");
                    theOracle.Same(aS.N.Location().IsIdentity(), aS.P.Location().IsIdentity, "Location.IsIdentity");
                    using (N.gp_Trsf aNT = aS.N.Location().Transformation())
                    {
                        theOracle.Same(aNT, aS.P.Location().Transformation(), "Location");
                    }

                    theOracle.Same(aNMap.Add(aS.N), aMap.Add(aS.P) + 1, "IndexedMapOfShape.Add");
                    theOracle.Same(aS.N.Locked(), aS.P.TShape()!.LockedFlag(), "Locked (stored bit)");
                    Assert.True(aS.P.Locked());
                }

                // identity: IsPartner, IsSame and IsEqual between every two, and the hash agrees with IsSame
                for (int i = 0; i < aShapes.Count && i < 40; i++)
                {
                    for (int j = i; j < aShapes.Count && j < 40; j++)
                    {
                        Twin<N.TopoDS_Shape, Shape> a = aShapes[i], b = aShapes[j];
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
                var aMap2 = new IndexedSet<Shape>(ShapeHasher.Instance);
                Exploration.MapShapes(theShape.P, aType, aMap2);
                theOracle.Same(aNMap2.Extent(), aMap2.Count, "MapShapes.Extent");
                for (int i = 1; i <= aMap2.Count; i++)
                {
                    N.TopoDS_Shape aNKey = aNMap2.FindKey(i);
                    Shape aKey = aMap2[i - 1];
                    theOracle.Same(aShapes.FindIndex(s => s.N.IsEqual(aNKey)), aShapes.FindIndex(s => s.P.IsEqual(aKey)), "MapShapes order");
                }
            }
        }

        foreach ((bool aCumOri, bool aCumLoc) in new[] { (true, true), (false, true), (true, false), (false, false) })
        {
            var aNAll = new N.TopTools_IndexedMapOfShape();
            N.TopExp.MapShapes(theShape.N, aNAll, aCumOri, aCumLoc);
            var anAll = new IndexedSet<Shape>(ShapeHasher.Instance);
            Exploration.MapShapes(theShape.P, anAll, aCumOri, aCumLoc);
            theOracle.Same(aNAll.Extent(), anAll.Count, "MapShapes (all).Extent");
            for (int i = 1; i <= anAll.Count; i++)
            {
                N.TopoDS_Shape aN = aNAll.FindKey(i);
                Shape aP = anAll[i - 1];
                theOracle.Same((int)aN.ShapeType(), (int)aP.ShapeType(), "MapShapes (all) type");
                theOracle.Same((int)aN.Orientation(), (int)aP.Orientation(), "MapShapes (all) orientation");
                using N.gp_Trsf aNT = aN.Location().Transformation();
                theOracle.Same(aNT, aP.Location().Transformation(), "MapShapes (all) location");
            }
        }

        // ancestors
        foreach ((ShapeType aTS, ShapeType aTA) in new[] { (ShapeType.Edge, ShapeType.Face), (ShapeType.Vertex, ShapeType.Edge), (ShapeType.Face, ShapeType.Solid) })
        {
            var aNAnc = new N.TopTools_IndexedDataMapOfShapeListOfShape();
            N.TopExp.MapShapesAndAncestors(theShape.N, TwinShapes.ToNative(aTS), TwinShapes.ToNative(aTA), aNAnc);
            var anAnc = new IndexedMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
            Exploration.MapShapesAndAncestors(theShape.P, aTS, aTA, anAnc);
            var aNUnique = new N.TopTools_IndexedDataMapOfShapeListOfShape();
            N.TopExp.MapShapesAndUniqueAncestors(theShape.N, TwinShapes.ToNative(aTS), TwinShapes.ToNative(aTA), aNUnique, true);
            var anUnique = new IndexedMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
            Exploration.MapShapesAndUniqueAncestors(theShape.P, aTS, aTA, anUnique, true);
            theOracle.Same(aNAnc.Extent(), anAnc.Count, "MapShapesAndAncestors.Extent");
            theOracle.Same(aNUnique.Extent(), anUnique.Count, "MapShapesAndUniqueAncestors.Extent");
            for (int i = 1; i <= anAnc.Count; i++)
            {
                theOracle.Same(aNAnc.FindFromIndex(i).Extent(), anAnc.ValueAt(i - 1).Extent(), "ancestors");
                theOracle.Same(aNUnique.FindFromIndex(i).Extent(), anUnique.ValueAt(i - 1).Extent(), "unique ancestors");
            }
        }

        foreach (Twin<N.TopoDS_Shape, Shape> aW in Explore(theShape, ShapeType.Wire))
        {
            var aNV1 = new N.TopoDS_Vertex();
            var aNV2 = new N.TopoDS_Vertex();
            N.TopExp.Vertices(N.TopoDS.Wire(aW.N), aNV1, aNV2);
            Exploration.Vertices(ShapeCasts.Wire(aW.P), out Vertex aV1, out Vertex aV2);
            CompareVertex(theOracle, aNV1, aV1, "wire's first vertex");
            CompareVertex(theOracle, aNV2, aV2, "wire's last vertex");
            theOracle.Same(N.BRep_Tool.IsClosed(aW.N), Tool.IsClosed(aW.P), "IsClosed(wire)");
        }

        foreach (ShapeType aType in new[] { ShapeType.Shell, ShapeType.Solid, ShapeType.Compound, ShapeType.Face })
        {
            foreach (Twin<N.TopoDS_Shape, Shape> aS in Explore(theShape, aType))
            {
                theOracle.Same(N.BRep_Tool.IsClosed(aS.N), Tool.IsClosed(aS.P), $"IsClosed({aType})");
            }
        }

        var aN3D = new N.TopTools_IndexedMapOfShape();
        N.BRepTools.Map3DEdges(theShape.N, aN3D);
        var a3D = new IndexedSet<Shape>(ShapeHasher.Instance);
        BrepTools.Map3DEdges(theShape.P, a3D);
        theOracle.Same(aN3D.Extent(), a3D.Count, "Map3DEdges");
    }

    private static void CompareVertex(Oracle theOracle, N.TopoDS_Vertex theN, in Vertex theP, string theWhat)
    {
        theOracle.Same(theN.IsNull(), theP.IsNull, theWhat + " IsNull");
        if (theP.IsNull)
        {
            return;
        }

        theOracle.Same((int)theN.Orientation(), (int)theP.Orientation(), theWhat + " orientation");
        using N.gp_Pnt aNP = N.BRep_Tool.Pnt(theN);
        theOracle.Same(aNP, Tool.Pnt(theP), theWhat);
        theOracle.Same(N.BRep_Tool.Tolerance(theN), Tool.Tolerance(theP), theWhat + " tolerance");
    }

    internal static void CompareGeometry(Oracle theOracle, Twin<N.TopoDS_Shape, Shape> theShape, Counts theCounts)
    {
        foreach (ShapeType aType in new[] { ShapeType.Vertex, ShapeType.Edge, ShapeType.Face })
        {
            theOracle.Same(N.BRep_Tool.MaxTolerance(theShape.N, TwinShapes.ToNative(aType)), Tool.MaxTolerance(theShape.P, aType),
                           $"MaxTolerance({aType})");
        }

        List<Twin<N.TopoDS_Shape, Shape>> aVertices = Explore(theShape, ShapeType.Vertex);
        foreach (Twin<N.TopoDS_Shape, Shape> aV in aVertices)
        {
            CompareVertex(theOracle, N.TopoDS.Vertex(aV.N), ShapeCasts.Vertex(aV.P), "vertex");
        }

        for (int i = 0; i + 1 < aVertices.Count && i < 20; i++)
        {
            theOracle.Same(N.BRepTools.Compare(N.TopoDS.Vertex(aVertices[i].N), N.TopoDS.Vertex(aVertices[i + 1].N)),
                           BrepTools.Compare(ShapeCasts.Vertex(aVertices[i].P), ShapeCasts.Vertex(aVertices[i + 1].P)), "Compare(vertices)");
        }

        List<Twin<N.TopoDS_Shape, Shape>> anEdges = Explore(theShape, ShapeType.Edge);
        for (int k = 0; k < anEdges.Count; k++)
        {
            N.TopoDS_Edge aNE = N.TopoDS.Edge(anEdges[k].N);
            Edge anE = ShapeCasts.Edge(anEdges[k].P);
            CompareEdge(theOracle, aNE, anE);
            theCounts.Edges++;
            if (k + 1 < anEdges.Count)
            {
                N.TopoDS_Edge aNE2 = N.TopoDS.Edge(anEdges[k + 1].N);
                Edge anE2 = ShapeCasts.Edge(anEdges[k + 1].P);
                var aNCommon = new N.TopoDS_Vertex();
                Vertex aCommon = new();
                theOracle.Same(N.TopExp.CommonVertex(aNE, aNE2, aNCommon), Exploration.CommonVertex(anE, anE2, ref aCommon), "CommonVertex");
                CompareVertex(theOracle, aNCommon, aCommon, "common vertex");
            }
        }

        foreach (Twin<N.TopoDS_Shape, Shape> aF in Explore(theShape, ShapeType.Face))
        {
            CompareFace(theOracle, N.TopoDS.Face(aF.N), ShapeCasts.Face(aF.P), theCounts);
        }
    }

    private static void CompareEdge(Oracle theOracle, N.TopoDS_Edge theNE, in Edge theE)
    {
        theOracle.Same(N.BRep_Tool.Tolerance(theNE), Tool.Tolerance(theE), "edge tolerance");
        theOracle.Same(N.BRep_Tool.SameParameter(theNE), Tool.SameParameter(theE), "SameParameter");
        theOracle.Same(N.BRep_Tool.SameRange(theNE), Tool.SameRange(theE), "SameRange");
        theOracle.Same(N.BRep_Tool.Degenerated(theNE), Tool.Degenerated(theE), "Degenerated");
        theOracle.Same(N.BRep_Tool.IsGeometric(theNE), Tool.IsGeometric(theE), "IsGeometric(edge)");
        theOracle.Same(N.BRep_Tool.HasContinuity(theNE), Tool.HasContinuity(theE), "HasContinuity");
        theOracle.Same(N.BRep_Tool.IsClosed(theNE), Tool.IsClosed(theE), "IsClosed(edge)");
        double aNF = 0, aNL = 0;
        N.BRep_Tool.Range(theNE, ref aNF, ref aNL);
        Tool.Range(theE, out double f, out double l);
        theOracle.Same(aNF, f, "Range.First");
        theOracle.Same(aNL, l, "Range.Last");

        // the curve and its location, and the placed curve
        using var aNLoc = new N.TopLoc_Location();
        double aNF2 = 0, aNL2 = 0;
        N.Geom_Curve? aNC = N.BRep_Tool.Curve(theNE, aNLoc, ref aNF2, ref aNL2);
        Curve3D? aC = Tool.Curve(theE, out Placement aLoc, out double f2, out double l2);
        theOracle.Same(aNC is null, aC is null, "Curve is null");
        using (N.gp_Trsf aNT = aNLoc.Transformation())
        {
            theOracle.Same(aNT, aLoc.Transformation(), "Curve location");
        }

        theOracle.Same(aNF2, f2, "Curve.First");
        theOracle.Same(aNL2, l2, "Curve.Last");
        N.Geom_Curve? aNPlaced = N.BRep_Tool.Curve(theNE, ref aNF2, ref aNL2);
        Curve3D? aPlaced = Tool.Curve(theE, out f2, out l2);
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
        Exploration.Vertices(theE, out Vertex aV1, out Vertex aV2);
        CompareVertex(theOracle, aNV1, aV1, "edge's first vertex");
        CompareVertex(theOracle, aNV2, aV2, "edge's last vertex");
        var aNV1c = new N.TopoDS_Vertex();
        var aNV2c = new N.TopoDS_Vertex();
        N.TopExp.Vertices(theNE, aNV1c, aNV2c, true);
        Exploration.Vertices(theE, out Vertex aV1c, out Vertex aV2c, true);
        CompareVertex(theOracle, aNV1c, aV1c, "edge's first vertex (cumulative)");
        CompareVertex(theOracle, aNV2c, aV2c, "edge's last vertex (cumulative)");
        CompareVertex(theOracle, N.TopExp.FirstVertex(theNE, true), Exploration.FirstVertex(theE, true), "FirstVertex");
        CompareVertex(theOracle, N.TopExp.LastVertex(theNE), Exploration.LastVertex(theE), "LastVertex");
        if (!aV1.IsNull)
        {
            theOracle.Same(N.BRep_Tool.Parameter(aNV1, theNE), Tool.Parameter(aV1, theE), "Parameter(V1, E)");
            theOracle.Same(N.BRep_Tool.Parameter(aNV2, theNE), Tool.Parameter(aV2, theE), "Parameter(V2, E)");
        }
    }

    private static void CompareFace(Oracle theOracle, N.TopoDS_Face theNF, in Face theF, Counts theCounts)
    {
        theCounts.Faces++;
        theOracle.Same(N.BRep_Tool.Tolerance(theNF), Tool.Tolerance(theF), "face tolerance");
        theOracle.Same(N.BRep_Tool.NaturalRestriction(theNF), Tool.NaturalRestriction(theF), "NaturalRestriction");
        theOracle.Same(N.BRep_Tool.IsGeometric(theNF), Tool.IsGeometric(theF), "IsGeometric(face)");

        double aNU1 = 0, aNU2 = 0, aNV1 = 0, aNV2 = 0;
        N.BRepTools.UVBounds(theNF, ref aNU1, ref aNU2, ref aNV1, ref aNV2);
        BrepTools.UVBounds(theF, out double u1, out double u2, out double v1, out double v2);
        theOracle.Same(aNU1, u1, "UVBounds UMin");
        theOracle.Same(aNU2, u2, "UVBounds UMax");
        theOracle.Same(aNV1, v1, "UVBounds VMin");
        theOracle.Same(aNV2, v2, "UVBounds VMax");

        bool aNUC = false, aNVC = false;
        N.BRepTools.DetectClosedness(theNF, ref aNUC, ref aNVC);
        BrepTools.DetectClosedness(theF, out bool aUC, out bool aVC);
        theOracle.Same(aNUC, aUC, "DetectClosedness U");
        theOracle.Same(aNVC, aVC, "DetectClosedness V");

        // the outer wire, by its position among the face's wires
        N.TopoDS_Wire aNOuter = N.BRepTools.OuterWire(theNF);
        Wire anOuter = BrepTools.OuterWire(theF);
        var aNWires = new N.TopExp_Explorer(theNF, N.TopAbs_ShapeEnum.TopAbs_WIRE);
        var aWires = new Explorer(theF, ShapeType.Wire);
        for (; aWires.More(); aWires.Next(), aNWires.Next())
        {
            theOracle.Same(aNWires.Current().IsSame(aNOuter), aWires.Current().IsSame(anOuter), "OuterWire");
            aNU1 = aNU2 = aNV1 = aNV2 = 0;
            N.BRepTools.UVBounds(theNF, N.TopoDS.Wire(aNWires.Current()), ref aNU1, ref aNU2, ref aNV1, ref aNV2);
            BrepTools.UVBounds(theF, ShapeCasts.Wire(aWires.Current()), out u1, out u2, out v1, out v2);
            theOracle.Same(aNU1, u1, "UVBounds(wire) UMin");
            theOracle.Same(aNU2, u2, "UVBounds(wire) UMax");
            theOracle.Same(aNV1, v1, "UVBounds(wire) VMin");
            theOracle.Same(aNV2, v2, "UVBounds(wire) VMax");
        }

        // the surface, placed, and the adaptor
        N.Geom_Surface? aNS = N.BRep_Tool.Surface(theNF);
        Surface? aS = Tool.Surface(theF);
        using var aNA = new N.BRepAdaptor_Surface(theNF);
        var anA = new FaceSurfaceAdaptor(theF);
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
        for (var aVertices = new Explorer(theF, ShapeType.Vertex); aVertices.More(); aVertices.Next(), aNVertices.Next())
        {
            using N.gp_Pnt2d aNUV = N.BRep_Tool.Parameters(N.TopoDS.Vertex(aNVertices.Current()), theNF);
            theOracle.Same(aNUV, Tool.Parameters(ShapeCasts.Vertex(aVertices.Current()), theF), "Parameters(V, F)");
        }

        // the edges on the face
        var aNEdges = new N.TopExp_Explorer(theNF, N.TopAbs_ShapeEnum.TopAbs_EDGE);
        for (var anEdges = new Explorer(theF, ShapeType.Edge); anEdges.More(); anEdges.Next(), aNEdges.Next())
        {
            theCounts.EdgeFaces++;
            N.TopoDS_Edge aNE = N.TopoDS.Edge(aNEdges.Current());
            Edge anE = ShapeCasts.Edge(anEdges.Current());
            double aNf = 0, aNl = 0;
            N.Geom2d_Curve? aNPC = N.BRep_Tool.CurveOnSurface(aNE, theNF, ref aNf, ref aNl);
            Curve2D? aPC = Tool.CurveOnSurface(anE, theF, out double f, out double l);
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
            Tool.UVPoints(anE, theF, out Point2D aP1, out Point2D aP2);
            theOracle.Same(aNP1, aP1, "UVPoints first");
            theOracle.Same(aNP2, aP2, "UVPoints last");
            theOracle.Same(N.BRep_Tool.IsClosed(aNE, theNF), Tool.IsClosed(anE, theF), "IsClosed(E, F)");
            theOracle.Same(N.BRepTools.IsReallyClosed(aNE, theNF), BrepTools.IsReallyClosed(anE, theF), "IsReallyClosed");
            theOracle.Same((int)N.BRepTools.OriEdgeInFace(aNE, theNF), (int)BrepTools.OriEdgeInFace(anE, theF), "OriEdgeInFace");
            theOracle.Same(N.BRep_Tool.HasContinuity(aNE, theNF, theNF), Tool.HasContinuity(anE, theF, theF), "HasContinuity(E, F, F)");
            theOracle.Same((int)N.BRep_Tool.Continuity(aNE, theNF, theNF), (int)Tool.Continuity(anE, theF, theF), "Continuity(E, F, F)");

            double aNr1 = 0, aNr2 = 0;
            N.BRep_Tool.Range(aNE, theNF, ref aNr1, ref aNr2);
            Tool.Range(anE, theF, out double r1, out double r2);
            theOracle.Same(aNr1, r1, "Range(E, F).First");
            theOracle.Same(aNr2, r2, "Range(E, F).Last");

            var aNEV1 = new N.TopoDS_Vertex();
            var aNEV2 = new N.TopoDS_Vertex();
            N.TopExp.Vertices(aNE, aNEV1, aNEV2);
            Exploration.Vertices(anE, out Vertex aV1, out Vertex aV2);
            theOracle.Same(N.BRep_Tool.Parameter(aNEV1, aNE, theNF), Tool.Parameter(aV1, anE, theF), "Parameter(V1, E, F)");
            theOracle.Same(N.BRep_Tool.Parameter(aNEV2, aNE, theNF), Tool.Parameter(aV2, anE, theF), "Parameter(V2, E, F)");

            aNf = aNl = 0;
            double aU1 = 0, aU2 = 0;
            N.BRepTools.UVBounds(theNF, aNE, ref aNf, ref aNl, ref aU1, ref aU2);
            BrepTools.UVBounds(theF, anE, out double eu1, out double eu2, out double ev1, out double ev2);
            theOracle.Same(aNf, eu1, "UVBounds(edge) UMin");
            theOracle.Same(aNl, eu2, "UVBounds(edge) UMax");
            theOracle.Same(aU1, ev1, "UVBounds(edge) VMin");
            theOracle.Same(aU2, ev2, "UVBounds(edge) VMax");
        }
    }
}
