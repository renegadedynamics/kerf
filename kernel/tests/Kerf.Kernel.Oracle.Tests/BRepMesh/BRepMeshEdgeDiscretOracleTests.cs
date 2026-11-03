// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Measurement;
using Kerf.Engine.Meshing;
using Kerf.Engine.Meshing.Model;
using Kerf.Engine.Meshing.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Reflection;
using Xunit.Abstractions;
using static Kerf.Engine.Meshing.Model.StatusModel;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Engine.Exchange.Brep.FormatVersion;

namespace Kerf.Kernel.Oracle.Tests;

/// <summary>
/// Unit 4C's discrete model against native OCCT, through the edge polygons of native BRepMesh_IncrementalMesh: native
/// OCCT's face mesher only numbers the boundary points, so each polygon on triangulation it stores is the edge's
/// discrete pcurve on that face after EdgeDiscret, ModelHealer and ModelPreProcessor (its parameters, the edge's
/// deflection, and nodes that are the edge's 3D points), and each free edge's 3D polygon is its discrete curve.
/// </summary>
/// <remarks>
/// Decision 494's scheme: native writes the shape without meshes (F1), reads F1, meshes it with the five-argument
/// constructor and writes F2; the port reads F1, builds its model with the same parameters (as initParameters completes
/// them) and compares it with F2's polygons. Faces native couldn't mesh have no polygons and are counted, not compared.
/// The re-meshing test meshes F2 again natively (F3) and the port pre-processes its read of F2: the faces it reuses keep
/// F2's polygons in F3, the others get its pcurves. BRep text writes 15 significant digits, so values agree to about
/// 5e-15; the share that survives the text bit for bit is reported.
/// </remarks>
public sealed class BRepMeshEdgeDiscretOracleTests(ITestOutputHelper output) : IDisposable
{
    private readonly string myDirectory = Directory.CreateTempSubdirectory("occtnet-brepmesh4c-").FullName;
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

    private string NewFile() => Path.Combine(myDirectory, $"m{myFileNumber++}.brep");

    private static string DataOcc
        => Path.Combine(typeof(BRepMeshEdgeDiscretOracleTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>()
                                                                       .Single(a => a.Key == "OcctSourceDir").Value!,
                        "data", "occ");

    private static Shape PortRead(string theFile)
    {
        Assert.True(BrepTools.Read(out Shape aShape, theFile), theFile);
        return aShape;
    }

    private static N.TopoDS_Shape NativeRead(string theFile)
    {
        var aShape = new N.TopoDS_Shape();
        using var aBuilder = new N.BRep_Builder();
        Assert.True(N.BRepTools.Read(aShape, theFile, aBuilder), theFile);
        return aShape;
    }

    private string NativeWrite(N.TopoDS_Shape theShape)
    {
        string aFile = NewFile();
        Assert.True(N.BRepTools.Write(theShape, aFile, true, false, (N.TopTools_FormatVersion)(int)FormatVersion_CURRENT));
        return aFile;
    }

    /// <summary>
    /// Native meshes its read of the file and writes it: the file with the native meshes, and the native faces'
    /// triangulations (in TopExp::MapShapes order), whose nodes the binding gives bit for bit.
    /// </summary>
    private string NativeMesh(string theFile, double theDeflection, bool isRelative, double theAngle,
                              out N.Poly_Triangulation?[] theTriangulations)
    {
        N.TopoDS_Shape aShape = NativeRead(theFile);
        using (new N.BRepMesh_IncrementalMesh(aShape, theDeflection, isRelative, theAngle, false))
        {
        }
        string aFile = NativeWrite(aShape);
        using (var aMap = new N.TopTools_IndexedMapOfShape())
        {
            N.TopExp.MapShapes(aShape, N.TopAbs_ShapeEnum.TopAbs_FACE, aMap);
            N.TopoDS_Shape[] aFaces = aMap.ToArray();
            theTriangulations = new N.Poly_Triangulation?[aFaces.Length];
            for (int i = 0; i < aFaces.Length; i++)
            {
                using N.TopoDS_Face aFace = N.TopoDS.Face(aFaces[i]);
                using var aLoc = new N.TopLoc_Location();
                theTriangulations[i] = N.BRep_Tool.Triangulation(aFace, aLoc);
                aFaces[i].Dispose();
            }
        }
        aShape.Dispose();
        return aFile;
    }

    private static void Dispose(N.Poly_Triangulation?[] theTriangulations)
    {
        foreach (N.Poly_Triangulation? aT in theTriangulations)
        {
            aT?.Dispose();
        }
    }

    /// <summary>The parameters as BRepMesh_IncrementalMesh::initParameters completes the five arguments.</summary>
    private static ToolParameters Params(double theDeflection, bool isRelative, double theAngle)
    {
        var aParams = new ToolParameters { Deflection = theDeflection, Angle = theAngle, Relative = isRelative };
        aParams.DeflectionInterior = theDeflection;
        aParams.MinSize = Math.Max(ToolParameters.RelMinSize() * theDeflection, global::Kerf.Engine.Foundation.Precision.Confusion());
        aParams.AngleInterior = 2.0 * theAngle;
        return aParams;
    }

    /// <summary>The port's model of the shape, as BRepMesh_Context's tools leave it before the faces are meshed.</summary>
    private static ModelModel PortModel(in Shape theShape, in ToolParameters theParameters)
    {
        var aContext = new ToolContext();
        aContext.SetModelBuilder(new MeshModelBuilder());
        aContext.SetEdgeDiscret(new EdgeDiscret());
        aContext.SetModelHealer(new ModelHealer());
        aContext.SetPreProcessor(new ModelPreProcessor());
        aContext.SetShape(theShape);
        aContext.ChangeParameters() = theParameters;
        Assert.True(aContext.BuildModel() && aContext.DiscretizeEdges() && aContext.HealModel() && aContext.PreProcessModel());
        return aContext.GetModel()!;
    }

    private sealed class Tally
    {
        public int Shapes, Edges, Polygons, Polygons3D, Values, ExactNodes, NotMeshed, Reused, Disagreements;
    }

    private static IndexedSet<Shape> Map(in Shape theShape, ShapeType theType)
    {
        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, theType, aMap);
        return aMap;
    }

    /// <summary>
    /// The index of the model's sub-shape in the map: the same TShape at the same location (the model keeps one edge or
    /// face per location); a free edge, which the model takes without its location, by its TShape.
    /// </summary>
    private static int IndexOf(IndexedSet<Shape> theMap, in Shape theShape)
    {
        int anIndex = theMap.IndexOf(theShape) + 1;
        if (anIndex == 0)
        {
            for (int i = 1; i <= theMap.Count && anIndex == 0; i++)
            {
                if (ReferenceEquals(theMap[i - 1].TShape(), theShape.TShape()))
                {
                    anIndex = i;
                }
            }
        }
        Assert.NotEqual(0, anIndex);
        return anIndex;
    }

    private void Disagree(Tally theTally, string theWhat)
    {
        if (theTally.Disagreements++ < 30)
        {
            output.WriteLine("DISAGREE " + theWhat);
        }
    }

    /// <summary>
    /// Compares the model of the port's read of F1 with the polygons of the meshed file: for each pcurve on a face the
    /// meshed file meshed anew, the polygon; for each free edge, the 3D polygon. With <paramref name="theOld"/> (the
    /// file meshed before), a face the port reuses must keep its polygons.
    /// </summary>
    private void Compare(Oracle theO, Tally theTally, string theName, ModelModel theModel, in Shape theF1,
                         in Shape theMeshed, N.Poly_Triangulation?[] theNativeTriangulations, in Shape? theOld)
    {
        IndexedSet<Shape> anEdges1 = Map(theF1, ShapeType.Edge), aFaces1 = Map(theF1, ShapeType.Face);
        IndexedSet<Shape> anEdges2 = Map(theMeshed, ShapeType.Edge), aFaces2 = Map(theMeshed, ShapeType.Face);
        IndexedSet<Shape>? anOldEdges = null, anOldFaces = null;
        if (theOld is Shape anOld)
        {
            anOldEdges = Map(anOld, ShapeType.Edge);
            anOldFaces = Map(anOld, ShapeType.Face);
        }

        theTally.Shapes++;
        for (int i = 0; i < theModel.EdgesNb(); i++)
        {
            EdgeModel aDEdge = theModel.GetEdge(i);
            theTally.Edges++;
            int anEdgeIndex = IndexOf(anEdges1, aDEdge.GetEdge());
            Edge anEdge2 = ShapeCasts.Edge(anEdges2[anEdgeIndex - 1]);
            CurveModel aCurve = aDEdge.GetCurve();
            string aWhat = $"{theName} edge {anEdgeIndex}";

            if (aDEdge.IsFree)
            {
                // The model takes a free edge without its location: its points, and the polygon, are in the edge's
                // own frame.
                Polygon3D? aPoly3D = Tool.Polygon3D(ShapeCasts.Edge(anEdge2.Located(new Placement())), out _);
                if (aDEdge.IsSet(StatusModel.Reused) || aPoly3D is null || aCurve.ParametersNb() == 0)
                {
                    continue;
                }
                theTally.Polygons3D++;
                if (aPoly3D.NodeCount != aCurve.ParametersNb())
                {
                    Disagree(theTally, $"{aWhat} 3D polygon: {aPoly3D.NodeCount} nodes, port {aCurve.ParametersNb()}");
                    continue;
                }
                theO.Same(aPoly3D.Deflection(), aDEdge.GetDeflection(), $"{aWhat} 3D deflection");
                for (int k = 1; k <= aPoly3D.NodeCount; k++)
                {
                    theO.Same(aPoly3D.Parameters()![k], aCurve.GetParameter(k - 1), $"{aWhat} 3D parameter {k}");
                    theO.Same(aPoly3D.Nodes()[k].XYZ(), aCurve.GetPoint(k - 1).XYZ(),
                              $"{aWhat} 3D node {k}", 0.0);
                    theTally.Values += 4;
                }
                continue;
            }

            // The pcurves on each face, in the model's order: OCCT stores the first as the polygon of the forward
            // edge, the last as the reversed edge's (ModelPostProcessor's PolygonCommitter).
            var aByFace = new Dictionary<FaceModel, List<PCurveModel>>(ReferenceEqualityComparer.Instance);
            var aFaceOrder = new List<FaceModel>();
            for (int p = 0; p < aDEdge.PCurvesNb(); p++)
            {
                PCurveModel aPCurve = aDEdge.GetPCurve(p);
                if (!aByFace.TryGetValue(aPCurve.GetFace(), out List<PCurveModel>? aList))
                {
                    aByFace.Add(aPCurve.GetFace(), aList = []);
                    aFaceOrder.Add(aPCurve.GetFace());
                }
                aList.Add(aPCurve);
            }

            foreach (FaceModel aDFace in aFaceOrder)
            {
                int aFaceIndex = IndexOf(aFaces1, aDFace.GetFace());
                Face aFace2 = ShapeCasts.Face(aFaces2[aFaceIndex - 1]);
                Triangulation? aTri = Tool.Triangulation(aFace2, out Placement aLoc);
                if (aTri is null)
                {
                    theTally.NotMeshed++;
                    continue;
                }

                List<PCurveModel> aPCurves = aByFace[aDFace];
                if (aDFace.IsSet(StatusModel.Reused))
                {
                    // Reused by the port: native kept the face's mesh and the edge's polygon on it.
                    theTally.Reused++;
                    Face anOldFace = ShapeCasts.Face(anOldFaces![aFaceIndex - 1]);
                    Edge anOldEdge = ShapeCasts.Edge(anOldEdges![anEdgeIndex - 1]);
                    Triangulation? anOldTri = Tool.Triangulation(anOldFace, out Placement anOldLoc);
                    PolygonOnTriangulation? anOldPoly = Tool.PolygonOnTriangulation(anOldEdge, anOldTri, anOldLoc);
                    PolygonOnTriangulation? aNewPoly = Tool.PolygonOnTriangulation(anEdge2, aTri, aLoc);
                    if (anOldTri is null || aTri.NodeCount != anOldTri.NodeCount || aTri.TriangleCount != anOldTri.TriangleCount
                        || (anOldPoly is null) != (aNewPoly is null)
                        || (aNewPoly is not null && (aNewPoly.NodeCount != anOldPoly!.NodeCount || aNewPoly.Deflection() != anOldPoly.Deflection())))
                    {
                        Disagree(theTally, $"{aWhat} face {aFaceIndex}: reused by the port, re-meshed natively");
                    }
                    continue;
                }

                for (int aSide = 0; aSide < aPCurves.Count; aSide++)
                {
                    PCurveModel aPCurve = aPCurves[aSide == 0 ? 0 : aPCurves.Count - 1];
                    Edge anOriented = ShapeCasts.Edge(anEdge2.Oriented(aSide == 0 ? Forward : Reversed));
                    PolygonOnTriangulation? aPoly = Tool.PolygonOnTriangulation(anOriented, aTri, aLoc);
                    if (aPoly is null)
                    {
                        Disagree(theTally, $"{aWhat} face {aFaceIndex}: no native polygon");
                        continue;
                    }
                    theTally.Polygons++;
                    if (aPoly.NodeCount != aPCurve.ParametersNb())
                    {
                        Disagree(theTally, $"{aWhat} face {aFaceIndex} side {aSide}: {aPoly.NodeCount} nodes, port {aPCurve.ParametersNb()}");
                        continue;
                    }

                    theO.Same(aPoly.Deflection(), aDEdge.GetDeflection(), $"{aWhat} deflection");
                    theTally.Values++;
                    for (int k = 1; k <= aPoly.NodeCount; k++)
                    {
                        theO.Same(aPoly.Parameters()![k], aPCurve.GetParameter(k - 1), $"{aWhat} face {aFaceIndex} parameter {k}");
                        theTally.Values++;
                        // The ends are the vertices' points, which unit 4D's face mesher merges in UV with the
                        // first node registered there: only the inner nodes are the edge's own.
                        if (!aDEdge.GetDegenerated() && k > 1 && k < aPoly.NodeCount)
                        {
                            // Bit for bit: the native node through the binding, the port's point moved as
                            // BRepMesh_ShapeTool::AddInFace moves it into the face's frame.
                            Point3D aPoint = aCurve.GetPoint(k - 1);
                            if (!aLoc.IsIdentity)
                            {
                                Transform3D aTrsf = aLoc.Transformation();
                                aTrsf = aTrsf.Inverted();
                                aPoint = aPoint.Transformed(aTrsf);
                            }
                            using N.gp_Pnt aNative = theNativeTriangulations[aFaceIndex - 1]!.Node(aPoly.Node(k));
                            theO.Same(aNative, aPoint, $"{aWhat} face {aFaceIndex} node {k}", 0.0);
                            theTally.Values += 3;
                            theTally.ExactNodes++;
                        }
                    }
                }
            }
        }
    }

    private void Run(Oracle theO, Tally theTally, string theName, string theF1)
    {
        Shape aPortF1 = PortRead(theF1);
        var aBox = new Box();
        Bounds.Add(aPortF1, ref aBox, false);
        if (aBox.IsVoid)
        {
            return;
        }
        double aSize = 0.0;
        ShapeTool.BoxMaxDimension(aBox, ref aSize);

        foreach ((double aDeflection, bool isRelative, double anAngle) in new[] { (0.05, true, 0.5), (0.004 * aSize, false, 0.3) })
        {
            string aF2 = NativeMesh(theF1, aDeflection, isRelative, anAngle, out N.Poly_Triangulation?[] aNativeTris);
            ModelModel aModel = PortModel(aPortF1, Params(aDeflection, isRelative, anAngle));
            Compare(theO, theTally, $"{theName} ({aDeflection:G3}{(isRelative ? " rel" : "")})", aModel, aPortF1, PortRead(aF2),
                    aNativeTris, null);
            Dispose(aNativeTris);
        }
    }

    private void Report(Oracle theO, Tally theTally, string theWhat)
    {
        output.WriteLine($"{theTally.Shapes} meshings, {theTally.Edges} edges, {theTally.Polygons} polygons on triangulation, "
                         + $"{theTally.Polygons3D} 3D polygons, {theTally.Values} values ({theTally.ExactNodes} inner nodes bit for bit), "
                         + $"{theTally.Reused} reused pcurves, "
                         + $"{theTally.NotMeshed} pcurves on faces native didn't mesh, {theTally.Disagreements} disagreements");
        theO.Report(theWhat);
        Assert.Equal(0, theTally.Disagreements);
    }

    [Fact]
    public void Data_occ_edge_polygons_match_native()
    {
        var anOracle = new Oracle(output, 40301);
        var aTally = new Tally();
        foreach (string aFile in Tape.Budget(Directory.GetFiles(DataOcc, "*.brep").Order(StringComparer.Ordinal), isSpread: true))
        {
            if (new FileInfo(aFile).Length > 1_000_000)
            {
                continue;
            }

            // F1: the shape without its meshes.
            N.TopoDS_Shape aNative = NativeRead(aFile);
            N.BRepTools.Clean(aNative);
            string aF1 = NativeWrite(aNative);
            aNative.Dispose();
            Run(anOracle, aTally, Path.GetFileName(aFile), aF1);
        }
        Report(anOracle, aTally, "BRepMesh model edges on data/occ against native polygons");
    }

    [Fact]
    public void Random_primitive_edge_polygons_match_native()
    {
        var anOracle = new Oracle(output, 40302);
        var aTally = new Tally();
        foreach (int i in Tape.Budget(60))
        {
            Direction3D aZ = anOracle.Dir();
            Direction3D aRef = anOracle.Dir();
            Direction3D aX = aZ.Crossed(aRef.IsParallel(aZ, 0.1) ? (aZ.IsParallel(new Direction3D(1, 0, 0), 0.1) ? new Direction3D(0, 1, 0) : new Direction3D(1, 0, 0)) : aRef);
            using var aNAx2 = new N.gp_Ax2(Oracle.ToNative(anOracle.Pnt()), Oracle.ToNative(aZ), Oracle.ToNative(aX));
            double r = anOracle.Real(0.5, 20), h = anOracle.Real(0.5, 30);
            N.TopoDS_Shape aShape = (i % 5) switch
            {
                0 => new N.BRepPrimAPI_MakeCylinder(aNAx2, r, h).Shape(),
                1 => new N.BRepPrimAPI_MakeCone(aNAx2, r, anOracle.Int(0, 1) == 0 ? 0.0 : r * anOracle.Real(0.1, 0.9), h).Shape(),
                2 => new N.BRepPrimAPI_MakeSphere(aNAx2, r).Shape(),
                3 => new N.BRepPrimAPI_MakeTorus(aNAx2, r + h, r * anOracle.Real(0.1, 0.9)).Shape(),
                _ => new N.BRepPrimAPI_MakeCylinder(aNAx2, r, h, anOracle.Real(0.5, 6.0)).Shape(),
            };
            string aF1 = NativeWrite(aShape);
            aShape.Dispose();
            Run(anOracle, aTally, $"primitive {i}", aF1);
        }
        Report(anOracle, aTally, "BRepMesh model edges on random primitives against native polygons");
    }

    [Fact]
    public void Remeshed_data_occ_reuses_and_replaces_as_native()
    {
        var anOracle = new Oracle(output, 40303);
        var aTally = new Tally();
        foreach (string aName in new[] { "solid.brep", "Room.brep", "mal_tige.brep", "fuse.brep", "face.brep", "bottle.brep", "Pump_Nut.brep", "MODERN_Table_1.brep" })
        {
            N.TopoDS_Shape aNative = NativeRead(Path.Combine(DataOcc, aName));
            N.BRepTools.Clean(aNative);
            string aF1 = NativeWrite(aNative);
            aNative.Dispose();

            // F2: meshed natively; then meshed again, coarser (reused) and finer (replaced).
            string aF2 = NativeMesh(aF1, 0.05, true, 0.5, out N.Poly_Triangulation?[] aF2Tris);
            Dispose(aF2Tris);
            foreach ((double aDeflection, double anAngle) in new[] { (0.1, 0.5), (0.05, 0.5), (0.02, 0.5), (0.05, 0.2) })
            {
                string aF3 = NativeMesh(aF2, aDeflection, true, anAngle, out N.Poly_Triangulation?[] aNativeTris);
                Shape aPortF2 = PortRead(aF2);
                Shape aCopy = BrepTools.ExactCopyForWrite(aPortF2, out _);
                ModelModel aModel = PortModel(aCopy, Params(aDeflection, true, anAngle));
                Compare(anOracle, aTally, $"{aName} again at {aDeflection} {anAngle}", aModel, aCopy, PortRead(aF3), aNativeTris,
                        aPortF2);
                Dispose(aNativeTris);
            }
        }
        Report(anOracle, aTally, "BRepMesh model edges on re-meshed data/occ against native polygons");
        Assert.True(aTally.Reused > 0);
    }
}
