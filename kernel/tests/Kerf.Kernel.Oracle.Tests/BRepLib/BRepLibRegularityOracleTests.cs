// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Text;
using Schematiq.Collections;
using Xunit.Abstractions;
using static Kerf.Topology.ShapeType;
using static Kerf.Engine.Exchange.Brep.FormatVersion;

namespace Kerf.Kernel.Oracle.Tests;

/// <summary>
/// BRepLib's regularity, sorting, range, planar pcurve, face extension and mesh statics against native OCCT.
/// </summary>
/// <remarks>
/// Each scene is a random native shape (BRepToolsIOOracleTests' primitives, sweeps, Booleans, fillets and lofts,
/// placed at random; meshed by BRepMesh where the statics need meshes) written by native OCCT to a file. Both sides read
/// that file, so they start from the same shape. The in-place statics run natively on <c>BRepBuilderAPI_Copy(S, false,
/// true)</c> of the shape read, and in the port on <c>BRepTools.CopyForWrite</c> of its frozen shape read (decision
/// 349: the same copy); the results are written by both sides (version 3 with triangles and normals) and compared byte
/// for byte. The queries are compared value by value, the shapes indexed by <c>TopExp::MapShapes</c>.
/// </remarks>
public class BRepLibRegularityOracleTests(ITestOutputHelper output) : IDisposable
{
    private const int Scenes = 96;

    private readonly string myDirectory = Directory.CreateTempSubdirectory("occtnet-breplib-").FullName;
    private int myFileNumber;

    public void Dispose()
    {
        Directory.Delete(myDirectory, true);
        GC.SuppressFinalize(this);
    }

    private string NewFile() => Path.Combine(myDirectory, $"s{myFileNumber++}.brep");

    private static string Text(string theFile) => File.ReadAllText(theFile, Encoding.Latin1);

    private string NativeText(N.TopoDS_Shape theShape)
    {
        string aFile = NewFile();
        Assert.True(N.BRepTools.Write(theShape, aFile, true, true, N.TopTools_FormatVersion.TopTools_FormatVersion_VERSION_3));
        return Text(aFile);
    }

    private string PortText(in Shape theShape)
    {
        string aFile = NewFile();
        Assert.True(BrepTools.Write(theShape, aFile, true, true, FormatVersion_VERSION_3));
        return Text(aFile);
    }

    /// <summary>A scene: the native shape as read back from its file, and the port's shape read from the same file.</summary>
    private (N.TopoDS_Shape N, Shape P, string What) Scene(Oracle theOracle, int theIndex, bool theMeshed)
    {
        N.TopoDS_Shape aShape = BRepToolsIOOracleTests.NativeShape(theOracle, theIndex, out string aWhat);
        if (theMeshed && !aWhat.EndsWith("meshed", StringComparison.Ordinal))
        {
            using var aMesh = new N.BRepMesh_IncrementalMesh(aShape, theOracle.Real(0.05, 0.5));
            aWhat += ", meshed";
        }

        return SceneOf(aShape, $"scene {theIndex} ({aWhat})");
    }

    /// <summary>The native shape written to a file and read back by both sides.</summary>
    private (N.TopoDS_Shape N, Shape P, string What) SceneOf(N.TopoDS_Shape aShape, string theWhat)
    {
        string aFile = NewFile();
        Assert.True(N.BRepTools.Write(aShape, aFile, true, true, N.TopTools_FormatVersion.TopTools_FormatVersion_VERSION_3));
        var aNShape = new N.TopoDS_Shape();
        using (var aBuilder = new N.BRep_Builder())
        {
            Assert.True(N.BRepTools.Read(aNShape, aFile, aBuilder));
        }

        Assert.True(BrepTools.Read(out Shape aPShape, aFile));
        return (aNShape, aPShape, theWhat);
    }

    private static N.TopoDS_Shape NativeCopy(N.TopoDS_Shape theShape) => new N.BRepBuilderAPI_Copy(theShape, false, true).Shape();

    private static Shape PortCopy(in Shape theShape) => BrepTools.CopyForWrite(theShape, out _);

    private static (N.TopTools_IndexedMapOfShape N, IndexedSet<Shape> P) Map(N.TopoDS_Shape theN, in Shape theP,
                                                                                             ShapeType theType)
    {
        var aNMap = new N.TopTools_IndexedMapOfShape();
        N.TopExp.MapShapes(theN, (N.TopAbs_ShapeEnum)(int)theType, aNMap);
        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theP, theType, aMap);
        Assert.Equal(aNMap.Extent(), aMap.Count);
        return (aNMap, aMap);
    }

    private sealed class Tally
    {
        public int Files;
        public long Bytes;
    }

    private void CompareTexts(N.TopoDS_Shape theN, in Shape theP, string theWhat, Tally theTally)
    {
        string aNText = NativeText(theN);
        string aPText = PortText(theP);
        Assert.True(aNText == aPText, $"{theWhat}: the port's result differs from native OCCT's");
        theTally.Files++;
        theTally.Bytes += aNText.Length;
    }

    [Fact]
    public void EncodeRegularity_codes_the_edges_as_native_OCCT()
    {
        var anOracle = new Oracle(output, 34101);
        var aTally = new Tally();
        foreach (int i in Tape.Budget(Scenes))
        {
            (N.TopoDS_Shape aN, Shape aP, string aWhat) = Scene(anOracle, i, false);
            double aTolAng = anOracle.Bool() ? 1.0e-10 : anOracle.Real(1e-6, 0.1);

            // the whole shape
            N.TopoDS_Shape aNC = NativeCopy(aN);
            Shape aPC = PortCopy(aP);
            N.BRepLib.EncodeRegularity(aNC, aTolAng);
            BrepLibrary.EncodeRegularity(aPC, aTolAng);
            CompareTexts(aNC, aPC, aWhat + ", whole", aTally);

            // every other edge, in a list
            aNC = NativeCopy(aN);
            aPC = PortCopy(aP);
            (N.TopTools_IndexedMapOfShape aNE, IndexedSet<Shape> aPE) = Map(aNC, aPC, ShapeType.Edge);
            var aNList = new N.TopTools_ListOfShape();
            var aPList = new NCollection_List<Shape>();
            for (int e = 1 + (i % 2); e <= aNE.Extent(); e += 2)
            {
                aNList.Append(aNE.FindKey(e));
                aPList.Append(aPE[e - 1]);
            }
            N.BRepLib.EncodeRegularity(aNC, aNList, aTolAng);
            BrepLibrary.EncodeRegularity(aPC, aPList, aTolAng);
            CompareTexts(aNC, aPC, aWhat + ", listed edges", aTally);

            // edge by edge, with the edge's first two faces (or its face twice for a seam)
            aNC = NativeCopy(aN);
            aPC = PortCopy(aP);
            var aNEF = new N.TopTools_IndexedDataMapOfShapeListOfShape();
            N.TopExp.MapShapesAndAncestors(aNC, N.TopAbs_ShapeEnum.TopAbs_EDGE, N.TopAbs_ShapeEnum.TopAbs_FACE, aNEF);
            var aPEF = new IndexedMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
            Exploration.MapShapesAndAncestors(aPC, ShapeType.Edge, ShapeType.Face, aPEF);
            Assert.Equal(aNEF.Extent(), aPEF.Count);
            for (int e = 1; e <= aNEF.Extent(); e++)
            {
                N.TopoDS_Shape[] aNF = aNEF.FindFromIndex(e).ToArray();
                NCollection_List<Shape> aPF = aPEF.ValueAt(e - 1);
                Assert.Equal(aNF.Length, aPF.Extent());
                if (aNF.Length == 0)
                {
                    continue;
                }
                N.BRepLib.EncodeRegularity(N.TopoDS.Edge(aNEF.FindKey(e)), N.TopoDS.Face(aNF[0]), N.TopoDS.Face(aNF[^1]), aTolAng);
                BrepLibrary.EncodeRegularity(ShapeCasts.Edge(aPEF.KeyAt(e - 1)), ShapeCasts.Face(aPF.First()), ShapeCasts.Face(aPF.Last()), aTolAng);
            }
            CompareTexts(aNC, aPC, aWhat + ", edge by edge", aTally);
        }
        output.WriteLine($"EncodeRegularity: {aTally.Files} results, {aTally.Bytes} bytes, all byte-identical");
    }

    [Fact]
    public void ContinuityOfFaces_answers_as_native_OCCT()
    {
        var anOracle = new Oracle(output, 34102);
        var aCounts = new int[8];
        int aValues = 0;
        foreach (int i in Tape.Budget(Scenes))
        {
            (N.TopoDS_Shape aN, Shape aP, string aWhat) = Scene(anOracle, i, false);
            var aNEF = new N.TopTools_IndexedDataMapOfShapeListOfShape();
            N.TopExp.MapShapesAndAncestors(aN, N.TopAbs_ShapeEnum.TopAbs_EDGE, N.TopAbs_ShapeEnum.TopAbs_FACE, aNEF);
            var aPEF = new IndexedMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
            Exploration.MapShapesAndAncestors(aP, ShapeType.Edge, ShapeType.Face, aPEF);
            for (int e = 1; e <= aNEF.Extent(); e++)
            {
                N.TopoDS_Shape[] aNF = aNEF.FindFromIndex(e).ToArray();
                Shape[] aPF = [.. aPEF.ValueAt(e - 1)];
                for (int f1 = 0; f1 < aNF.Length; f1++)
                {
                    for (int f2 = f1; f2 < aNF.Length; f2++)
                    {
                        foreach (double aTolAng in new[] { 1e-10, 1e-3 })
                        {
                            int aNC = (int)N.BRepLib.ContinuityOfFaces(N.TopoDS.Edge(aNEF.FindKey(e)), N.TopoDS.Face(aNF[f1]),
                                                                      N.TopoDS.Face(aNF[f2]), aTolAng);
                            int aPC = (int)BrepLibrary.ContinuityOfFaces(ShapeCasts.Edge(aPEF.KeyAt(e - 1)), ShapeCasts.Face(aPF[f1]),
                                                                     ShapeCasts.Face(aPF[f2]), aTolAng);
                            anOracle.Same(aNC, aPC, $"{aWhat}: ContinuityOfFaces of edge {e}, faces {f1}, {f2}, {aTolAng}");
                            aCounts[aPC]++;
                            aValues++;
                        }
                    }
                }
            }
        }
        output.WriteLine($"ContinuityOfFaces: {aValues} values, all equal; C0 {aCounts[0]}, G1 {aCounts[1]}, C1 {aCounts[2]}, " +
                         $"G2 {aCounts[3]}, C2 {aCounts[4]}, C3 {aCounts[5]}, CN {aCounts[6]}");
    }

    [Fact]
    public void Sorting_ranges_and_bounding_vertices_answer_as_native_OCCT()
    {
        var anOracle = new Oracle(output, 34103);
        int aValues = 0, aValidRanges = 0;
        foreach (int i in Tape.Budget(Scenes))
        {
            (N.TopoDS_Shape aN, Shape aP, string aWhat) = Scene(anOracle, i, anOracle.Bool());
            (N.TopTools_IndexedMapOfShape aNFaces, IndexedSet<Shape> aPFaces) = Map(aN, aP, ShapeType.Face);

            // SortFaces and ReverseSortFaces: the faces' indices
            for (int aReverse = 0; aReverse < 2; aReverse++)
            {
                var aNList = new N.TopTools_ListOfShape();
                if (aReverse == 0)
                {
                    N.BRepLib.SortFaces(aN, aNList);
                }
                else
                {
                    N.BRepLib.ReverseSortFaces(aN, aNList);
                }
                ISchematiqList<Shape> aPList = aReverse == 0 ? BrepLibrary.SortFaces(aP) : BrepLibrary.ReverseSortFaces(aP);
                N.TopoDS_Shape[] aNArray = aNList.ToArray();
                Assert.Equal(aNArray.Length, aPList.Count);
                for (int k = 0; k < aNArray.Length; k++)
                {
                    anOracle.Same(aNFaces.FindIndex(aNArray[k]), aPFaces.IndexOf(aPList[k]) + 1, $"{aWhat}: sorted face {k}");
                    anOracle.Same((int)aNArray[k].Orientation(), (int)aPList[k].Orientation, $"{aWhat}: sorted face {k}");
                    aValues += 2;
                }
            }

            // FindValidRange of each edge, through the edge and through its adaptor with random vertex tolerances
            (N.TopTools_IndexedMapOfShape aNEdges, IndexedSet<Shape> aPEdges) = Map(aN, aP, ShapeType.Edge);
            for (int e = 1; e <= aNEdges.Extent(); e++)
            {
                N.TopoDS_Edge aNE = N.TopoDS.Edge(aNEdges.FindKey(e));
                Edge aPE = ShapeCasts.Edge(aPEdges[e - 1]);
                double aNF = -7, aNL = -7, aPF = -7, aPL = -7;
                bool isN = N.BRepLib.FindValidRange(aNE, ref aNF, ref aNL);
                bool isP = BrepLibrary.FindValidRange(aPE, ref aPF, ref aPL);
                anOracle.Same(isN, isP, $"{aWhat}: FindValidRange of edge {e}");
                anOracle.Same(aNF, aPF, $"{aWhat}: FindValidRange first of edge {e}", 0);
                anOracle.Same(aNL, aPL, $"{aWhat}: FindValidRange last of edge {e}", 0);
                aValues += 3;
                aValidRanges += isP ? 1 : 0;

                if (Tool.Curve(aPE, out _, out _) is null)
                {
                    continue;
                }
                using var aNAC = new N.BRepAdaptor_Curve(aNE);
                var aPAC = new EdgeCurveAdaptor(aPE);
                double aF = aPAC.FirstParameter(), aL = aPAC.LastParameter();
                if (global::Kerf.Engine.Foundation.Precision.IsInfinite(aF) || global::Kerf.Engine.Foundation.Precision.IsInfinite(aL))
                {
                    continue;
                }
                double aTolE = Math.Pow(10, anOracle.Real(-7, -2));
                double aTol1 = (aL - aF) * anOracle.Real(0, 0.6), aTol2 = (aL - aF) * anOracle.Real(0, 0.6);
                Point3D aP1 = aPAC.Value(aF), aP2 = aPAC.Value(aL);
                using N.gp_Pnt aNP1 = Oracle.ToNative(aP1), aNP2 = Oracle.ToNative(aP2);
                aNF = aNL = aPF = aPL = -7;
                isN = N.BRepLib.FindValidRange(aNAC, aTolE, aF, aNP1, aTol1, aL, aNP2, aTol2, ref aNF, ref aNL);
                isP = BrepLibrary.FindValidRange(aPAC, aTolE, aF, aP1, aTol1, aL, aP2, aTol2, ref aPF, ref aPL);
                anOracle.Same(isN, isP, $"{aWhat}: FindValidRange of the adaptor of edge {e}");
                anOracle.Same(aNF, aPF, $"{aWhat}: FindValidRange first of the adaptor of edge {e}", 0);
                anOracle.Same(aNL, aPL, $"{aWhat}: FindValidRange last of the adaptor of edge {e}", 0);
                aValues += 3;
                aValidRanges += isP ? 1 : 0;
            }

            // BoundingVertex of random groups of vertices
            (N.TopTools_IndexedMapOfShape aNV, IndexedSet<Shape> aPV) = Map(aN, aP, ShapeType.Vertex);
            for (int g = 0; g < 5 && aNV.Extent() > 0; g++)
            {
                var aNList = new N.TopTools_ListOfShape();
                var aPList = new NCollection_List<Shape>();
                int aNb = anOracle.Int(1, Math.Min(6, aNV.Extent()));
                for (int k = 0; k < aNb; k++)
                {
                    int v = anOracle.Int(1, aNV.Extent());
                    aNList.Append(aNV.FindKey(v));
                    aPList.Append(aPV[v - 1]);
                }
                using var aNC = new N.gp_Pnt(1, 2, 3);
                var aPC = new Point3D(1, 2, 3);
                double aNTol = -1, aPTol = -1;
                N.BRepLib.BoundingVertex(aNList, aNC, ref aNTol);
                BrepLibrary.BoundingVertex(aPList, ref aPC, ref aPTol);
                anOracle.Same(aNC, aPC, $"{aWhat}: BoundingVertex centre", 0);
                anOracle.Same(aNTol, aPTol, $"{aWhat}: BoundingVertex radius", 0);
                aValues += 4;
            }
        }
        output.WriteLine($"SortFaces, ReverseSortFaces, FindValidRange ({aValidRanges} valid ranges), BoundingVertex: {aValues} values, all bit-identical");
    }

    [Fact]
    public void ExtendFace_and_planar_pcurves_build_what_native_OCCT_builds()
    {
        var anOracle = new Oracle(output, 34104);
        var aTally = new Tally();
        int aPCurves = 0, aNativeInputsChanged = 0;
        foreach (int i in Tape.Budget(Scenes))
        {
            bool isPolygon = i % 3 == 2;
            (N.TopoDS_Shape aN, Shape aP, string aWhat) = isPolygon ? PlanarPolygonFace(anOracle, i) : Scene(anOracle, i, false);
            // the planar pcurves of every edge of every face: on copies (which already carry them: the copy computes
            // them), or on the unfrozen polygon faces (which don't), compared before and after
            N.TopoDS_Shape aNC = isPolygon ? aN : NativeCopy(aN);
            Shape aPC = isPolygon ? aP : PortCopy(aP);
            CompareTexts(aNC, aPC, $"{aWhat}: before the planar pcurves", aTally);
            (N.TopTools_IndexedMapOfShape aNFaces, IndexedSet<Shape> aPFaces) = Map(aNC, aPC, ShapeType.Face);
            for (int f = 1; f <= aNFaces.Extent(); f++)
            {
                N.TopoDS_Face aNF = N.TopoDS.Face(aNFaces.FindKey(f));
                Face aPF = ShapeCasts.Face(aPFaces[f - 1]);
                (N.TopTools_IndexedMapOfShape aNE, IndexedSet<Shape> aPE) = Map(aNF, aPF, ShapeType.Edge);
                for (int e = 1; e <= aNE.Extent(); e++)
                {
                    BrepLibrary.BuildPCurveForEdgeOnPlane(ShapeCasts.Edge(aPE[e - 1]), aPF, out _, out bool isToUpdate);
                    aPCurves += isToUpdate ? 1 : 0;
                    N.BRepLib.BuildPCurveForEdgeOnPlane(N.TopoDS.Edge(aNE.FindKey(e)), aNF);
                    BrepLibrary.BuildPCurveForEdgeOnPlane(ShapeCasts.Edge(aPE[e - 1]), aPF);
                }
            }
            CompareTexts(aNC, aPC, $"{aWhat}: planar pcurves", aTally);
            // ExtendFace, after the copies: OCCT extends a B-spline face's own surface in place (occt-issues.md)
            string aNBefore = NativeText(aN);
            (aNFaces, aPFaces) = Map(aN, aP, ShapeType.Face);
            for (int f = 1; f <= aNFaces.Extent(); f++)
            {
                double anExt = anOracle.Real(0.01, 2);
                bool u1 = anOracle.Bool(), u2 = anOracle.Bool(), v1 = anOracle.Bool(), v2 = anOracle.Bool();
                var aNExt = new N.TopoDS_Face();
                N.BRepLib.ExtendFace(N.TopoDS.Face(aNFaces.FindKey(f)), anExt, u1, u2, v1, v2, aNExt);
                BrepLibrary.ExtendFace(ShapeCasts.Face(aPFaces[f - 1]), anExt, u1, u2, v1, v2, out Face aPExt);
                CompareTexts(aNExt, aPExt, $"{aWhat}: ExtendFace of face {f}", aTally);
            }

            if (NativeText(aN) != aNBefore)
            {
                aNativeInputsChanged++;
            }
        }
        output.WriteLine($"ExtendFace and BuildPCurveForEdgeOnPlane ({aPCurves} pcurves stored): {aTally.Files} results, " +
                         $"{aTally.Bytes} bytes, all byte-identical; ExtendFace changed the native input in {aNativeInputsChanged} scenes");
    }

    [Fact]
    public void ExtendFace_of_a_BSpline_face_changes_the_native_input_and_not_the_port_s()
    {
        // OCCT's ExtendFace extends the face's own B-spline surface in place (GeomLib::ExtendSurfByLength raises its
        // degree and inserts knots), so the face given changes (occt-issues.md); the port extends a copy.
        var anOracle = new Oracle(output, 34106);
        int aChanged = 0, aFaces = 0;
        for (int i = 0; i < 20; i++)
        {
            (N.TopoDS_Shape aN, Shape aP, string aWhat) = Scene(anOracle, 11 + 12 * i, false); // lofts
            string aNBefore = NativeText(aN), aPBefore = PortText(aP);
            (N.TopTools_IndexedMapOfShape aNFaces, IndexedSet<Shape> aPFaces) = Map(aN, aP, ShapeType.Face);
            var aTally = new Tally();
            for (int f = 1; f <= aNFaces.Extent(); f++)
            {
                var aNExt = new N.TopoDS_Face();
                N.BRepLib.ExtendFace(N.TopoDS.Face(aNFaces.FindKey(f)), 0.5, true, true, true, true, aNExt);
                BrepLibrary.ExtendFace(ShapeCasts.Face(aPFaces[f - 1]), 0.5, true, true, true, true, out Face aPExt);
                CompareTexts(aNExt, aPExt, $"{aWhat}: ExtendFace of face {f}", aTally);
                aFaces++;
            }
            aChanged += NativeText(aN) != aNBefore ? 1 : 0;
            Assert.Equal(aPBefore, PortText(aP));
        }
        output.WriteLine($"ExtendFace on {aFaces} loft faces: all extended faces byte-identical; the native input changed in {aChanged} of 20 lofts, the port's in none");
        Assert.True(aChanged > 0);
    }

    /// <summary>
    /// A face on a random plane bounded by a polygon, with a hole half the time, made by BRepBuilderAPI_MakeFace of the
    /// plane and the wires natively and by BRepLib_MakeFace in the port, which store no pcurves. The port's face is read
    /// through the makers' internal accessors: unfrozen, so the pcurves can be stored on it.
    /// </summary>
    private static (N.TopoDS_Shape N, Shape P, string What) PlanarPolygonFace(Oracle theOracle, int theIndex)
    {
        // a placement that native OCCT builds with the same directions (gp_Ax3 recomputes the X direction)
        CoordinateSystem anAx3;
        while (true)
        {
            anAx3 = new CoordinateSystem(theOracle.Pnt(), Oracle.Settled(theOracle.Dir()));
            using N.gp_Ax3 aNCheck = Oracle.ToNative(anAx3);
            using N.gp_Dir aNX = aNCheck.XDirection(), aNY = aNCheck.YDirection(), aNZ = aNCheck.Direction();
            if (Oracle.FromNative(aNX.XYZ()) == anAx3.XDirection.XYZ() && Oracle.FromNative(aNY.XYZ()) == anAx3.YDirection.XYZ()
                && Oracle.FromNative(aNZ.XYZ()) == anAx3.Direction.XYZ())
            {
                break;
            }
        }
        var aPln = new Plane3D(anAx3);
        (N.TopoDS_Wire, Wire) Polygon(double theR, bool theReversed)
        {
            using var aNPolygon = new N.BRepBuilderAPI_MakePolygon();
            var aPolygon = new LibraryMakePolygon();
            int aNb = theOracle.Int(3, 7);
            for (int k = 0; k < aNb; k++)
            {
                double anAngle = (theReversed ? -1 : 1) * 2 * Math.PI * k / aNb;
                double aR = theR * theOracle.Real(0.8, 1.2);
                Point3D aP = global::Kerf.Engine.Geometry.ElSLib.Value(aR * Math.Cos(anAngle), aR * Math.Sin(anAngle), aPln);
                using N.gp_Pnt aNP = Oracle.ToNative(aP);
                aNPolygon.Add(aNP);
                aPolygon.Add(aP);
            }
            aNPolygon.Close();
            aPolygon.Close();
            return (aNPolygon.Wire(), aPolygon.InternalWire());
        }

        using N.gp_Ax3 aNAx3 = Oracle.ToNative(anAx3);
        using var aNPln = new N.gp_Pln(aNAx3);
        (N.TopoDS_Wire aNOuter, Wire anOuter) = Polygon(theOracle.Real(2, 5), false);
        using var aNMaker = new N.BRepBuilderAPI_MakeFace(aNPln, aNOuter, true);
        var aMaker = new LibraryMakeFace(aPln, anOuter, true);
        bool isHoled = theOracle.Bool();
        if (isHoled)
        {
            (N.TopoDS_Wire aNHole, Wire aHole) = Polygon(theOracle.Real(0.3, 1), true);
            aNMaker.Add(aNHole);
            aMaker.Add(aHole);
        }
        return (aNMaker.Face(), aMaker.InternalFace(), $"scene {theIndex} (polygon face on a plane{(isHoled ? ", holed" : "")})");
    }

    [Fact]
    public void Mesh_statics_change_the_meshes_as_native_OCCT()
    {
        var anOracle = new Oracle(output, 34105);
        var aTally = new Tally();
        int aCorrected = 0;
        foreach (int i in Tape.Budget(Scenes))
        {
            (N.TopoDS_Shape aN, Shape aP, string aWhat) = Scene(anOracle, i, true);

            N.TopoDS_Shape aNC = NativeCopy(aN);
            Shape aPC = PortCopy(aP);
            N.BRepLib.UpdateDeflection(aNC);
            BrepLibrary.UpdateDeflection(aPC);
            CompareTexts(aNC, aPC, aWhat + ", UpdateDeflection", aTally);

            // twice: the second call finds the normals of the first
            for (int k = 0; k < 2; k++)
            {
                double anAngTol = anOracle.Real(0.001, 0.5);
                bool isForce = anOracle.Bool();
                bool isN = N.BRepLib.EnsureNormalConsistency(aNC, anAngTol, isForce);
                bool isP = BrepLibrary.EnsureNormalConsistency(aPC, anAngTol, isForce);
                anOracle.Same(isN, isP, $"{aWhat}: EnsureNormalConsistency");
                aCorrected += isP ? 1 : 0;
                CompareTexts(aNC, aPC, aWhat + ", EnsureNormalConsistency", aTally);
            }
        }
        output.WriteLine($"UpdateDeflection and EnsureNormalConsistency ({aCorrected} corrections): {aTally.Files} results, " +
                         $"{aTally.Bytes} bytes, all byte-identical");
    }
}
