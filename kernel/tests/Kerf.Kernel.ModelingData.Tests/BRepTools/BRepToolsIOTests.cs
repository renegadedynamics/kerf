// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Exchange.Brep;
using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;
using static Kerf.Engine.Exchange.Brep.FormatVersion;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// The text format of shapes: TopTools_ShapeSet, TopTools_LocationSet, BRepTools_ShapeSet and BRepTools' Read, Write
/// and Dump. Byte-for-byte agreement with native OCCT is the oracle tests' business (Kerf.Kernel.Oracle.Tests); these
/// check every path, the exact text of small cases (from OCCT's source), round trips and freezing.
/// </summary>
public class BRepToolsIOTests
{
    private static readonly BrepBuilder theBuilder = new();

    private static readonly FormatVersion[] theVersions =
        [FormatVersion_VERSION_1, FormatVersion_VERSION_2, FormatVersion_VERSION_3];

    internal static string Write(in Shape theShape, FormatVersion theVersion = FormatVersion_CURRENT,
                                 bool theWithTriangles = true, bool theWithNormals = false)
    {
        var aWriter = new StringWriter();
        BrepTools.Write(theShape, aWriter, theWithTriangles, theWithNormals, theVersion);
        return aWriter.ToString();
    }

    internal static Shape Read(string theText)
    {
        var aReader = new StringReader(theText);
        StreamUtilities.SetInputLength(aReader, theText.Length);
        BrepTools.Read(out Shape aShape, aReader);
        return aShape;
    }

    private static List<Shape> SubShapes(in Shape theShape)
    {
        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, aMap);
        return [.. aMap];
    }

    [Fact]
    public void A_vertex_is_written_as_OCCT_writes_it()
    {
        Vertex aV = new();
        theBuilder.MakeVertex(ref aV, new Point3D(1, 2.5, -3), 1e-7);
        Assert.Equal("\nCASCADE Topology V3, (c) Open Cascade\n"
                     + "Locations 0\n"
                     + "Curve2ds 0\n"
                     + "Curves 0\n"
                     + "Polygon3D 0\n"
                     + "PolygonOnTriangulations 0\n"
                     + "Surfaces 0\n"
                     + "Triangulations 0\n"
                     + "\nTShapes 1\n"
                     + "Ve\n"
                     + "1e-07\n"
                     + "1 2.5 -3\n"
                     + "0 0\n"
                     + "\n1101101\n"
                     + "*\n"
                     + "\n"
                     + "+1 0 ",
                     Write(aV));

        // version 1 and 2 headers; the file form adds Draw's line
        Assert.StartsWith("\nCASCADE Topology V1, (c) Matra-Datavision\nLocations 0\n", Write(aV, FormatVersion_VERSION_1));
        Assert.StartsWith("\nCASCADE Topology V2, (c) Matra-Datavision\nLocations 0\n", Write(aV, FormatVersion_VERSION_2));
    }

    [Fact]
    public void An_edge_records_its_flags_curve_and_range()
    {
        Edge anEdge = TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(3, 4, 0));
        string aText = Write(anEdge);
        // the edge: tolerance, SameParameter, SameRange, Degenerated; its 3D curve: index, location, range
        Assert.Contains("\nEd\n 1e-07 1 1 0\n1  1 0 0 5\n0\n", aText, StringComparison.Ordinal);
        // its vertices, FORWARD and REVERSED, then the end of the list
        Assert.Contains("\n+3 0 -2 0 *\n", aText, StringComparison.Ordinal);
        Assert.Contains("Curves 1\n1 0 0 0 0.59999999999999998 0.80000000000000004 0 \n", aText, StringComparison.Ordinal);
    }

    [Fact]
    public void Locations_are_written_elementary_then_complex()
    {
        var aT1 = new Transform3D();
        aT1 = aT1.WithTranslation(new Vector3D(1, 2, 3));
        var aT2 = new Transform3D();
        aT2 = aT2.WithRotation(new Axis3D(new Point3D(), new Direction3D(0, 0, 1)), Math.PI / 2);
        var aL1 = new Placement(aT1);
        var aL2 = new Placement(aT2);
        Placement aComplex = aL1 * aL2.Powered(2);

        var aSet = new LocationSet();
        Assert.Equal(0, aSet.Add(new Placement()));
        int anIndex = aSet.Add(aComplex);
        Assert.Equal(3, anIndex);
        Assert.Equal(anIndex, aSet.Add(aComplex));
        // a product's right-hand factor is its first item (TopLoc_Location::Multiplied prepends), added first
        Assert.Equal(1, aSet.Index(aL2));
        Assert.Equal(2, aSet.Index(aL1));
        Assert.True(aSet.Location(4).IsIdentity);
        Assert.True(aSet.Location(0).IsIdentity);

        var aWriter = new StringWriter();
        aSet.Write(aWriter);
        string aText = aWriter.ToString();
        Assert.Equal("Locations 3\n"
                     + "1\n"
                     + "1.11022302462516e-16              -1               0               0 \n"
                     + "              1 1.11022302462516e-16               0               0 \n"
                     + "              0               0               1               0 \n"
                     + "1\n"
                     + "              1               0               0               1 \n"
                     + "              0               1               0               2 \n"
                     + "              0               0               1               3 \n"
                     + "2  1 2 2 1 0\n",
                     aText);
        Assert.Equal(6, StreamUtilities.Precision(aWriter));

        var aRead = new LocationSet();
        aRead.Read(new StringReader(aText));
        Assert.Equal(3, aRead.LocationCount);
        Assert.True(aRead.Location(3).Transformation.TranslationXyz().IsEqual(aComplex.Transformation.TranslationXyz(), 1e-14));
        Assert.Equal(2, aRead.Location(3).FirstPower());
        Assert.Equal(1, aRead.Location(3).NextLocation().FirstPower());
        Assert.True(aRead.Location(3).NextLocation().NextLocation().IsIdentity);

        var aDump = new StringWriter();
        aRead.Dump(aDump);
        Assert.Contains("Complex : L1^2 * L2", aDump.ToString(), StringComparison.Ordinal);
        Assert.Contains("Elementary location", aDump.ToString(), StringComparison.Ordinal);
    }

    [Fact]
    public void Shapes_round_trip_byte_for_byte()
    {
        int aNbTexts = 0;
        foreach ((string aName, Shape aShape) in BinTestShapes.All())
        {
            foreach (FormatVersion aVersion in theVersions)
            {
                foreach ((bool aTri, bool aNorm) in new[] { (true, false), (true, true), (false, false) })
                {
                    string aT1 = Write(aShape, aVersion, aTri, aNorm);
                    Shape aS1 = Read(aT1);
                    Assert.False(aS1.IsNull, aName);
                    string aT2 = Write(aS1, aVersion, aTri, aNorm);
                    if (aVersion != FormatVersion_VERSION_1)
                    {
                        // version 1 reading computes the UV points and sets Checked (BRepTools::Update)
                        Assert.True(aT1 == aT2, $"{aName}, version {(int)aVersion}, {aTri} {aNorm}: first round trip");
                    }

                    string aT3 = Write(Read(aT2), aVersion, aTri, aNorm);
                    Assert.True(aT2 == aT3, $"{aName}, version {(int)aVersion}, {aTri} {aNorm}: second round trip");
                    aNbTexts++;
                }
            }
        }

        Assert.Equal(11 * 9, aNbTexts);
    }

    [Fact]
    public void Read_shapes_have_the_same_structure()
    {
        foreach ((string aName, Shape aShape) in BinTestShapes.All())
        {
            Shape aRead = Read(Write(aShape));
            List<Shape> anOriginal = SubShapes(aShape), aCopy = SubShapes(aRead);
            Assert.True(anOriginal.Count == aCopy.Count, aName);
            for (int i = 0; i < anOriginal.Count; i++)
            {
                Assert.Equal(anOriginal[i].Type, aCopy[i].Type);
                Assert.Equal(anOriginal[i].Orientation, aCopy[i].Orientation);
                Assert.True(anOriginal[i].Placement.Transformation.TranslationXyz()
                                .IsEqual(aCopy[i].Placement.Transformation.TranslationXyz(), 1e-12));
                Assert.Equal(anOriginal[i].ClosedFlag, aCopy[i].ClosedFlag);
                if (anOriginal[i].Type == ShapeType.Vertex)
                {
                    Assert.True(Tool.Pnt(ShapeCasts.Vertex(anOriginal[i])).IsEqual(Tool.Pnt(ShapeCasts.Vertex(aCopy[i])), 1e-12));
                }
            }
        }
    }

    [Fact]
    public void Reading_freezes_shapes_geometry_and_meshes()
    {
        foreach ((string aName, Shape aShape) in BinTestShapes.All())
        {
            Shape aRead = Read(Write(aShape, FormatVersion_VERSION_3, true, true));
            foreach (Shape aSub in SubShapes(aRead))
            {
                Assert.True(aSub.TShape()!.IsFrozen, aName);
                Assert.True(aSub.Locked(), aName);
                Assert.False(aSub.TShape()!.LockedFlag(), aName);
                if (aSub.TShape() is BrepTEdge aTE)
                {
                    foreach (CurveRepresentation aCR in aTE.CurveRepresentations())
                    {
                        Assert.True(aCR.IsFrozen);
                        if (aCR.IsCurve3D && aCR.Curve3D() is { } aC)
                        {
                            Assert.True(aC.IsFrozen);
                        }

                        if (aCR.IsCurveOnSurface())
                        {
                            Assert.True(aCR.PCurve()!.IsFrozen);
                            Assert.True(aCR.Surface()!.IsFrozen);
                        }

                        if (aCR.IsPolygonOnTriangulation())
                        {
                            Assert.True(aCR.PolygonOnTriangulation()!.IsFrozen);
                            Assert.True(aCR.Triangulation()!.IsFrozen);
                        }
                    }
                }
                else if (aSub.TShape() is BrepTFace aTF)
                {
                    Assert.True(aTF.Surface() is null || aTF.Surface()!.IsFrozen);
                    Assert.True(aTF.Triangulation() is null || aTF.Triangulation()!.IsFrozen);
                }
            }
        }
    }

    [Fact]
    public void Reading_clears_the_Free_flag_of_sub_shapes_as_OCCT_does()
    {
        Solid aBox = TestShapes.Box(1, 1, 1);
        // the file says every TShape is free; adding each to its parent clears the flag, as in OCCT
        string aText = Write(aBox).Replace("\n0101000\n", "\n1101000\n", StringComparison.Ordinal)
                                  .Replace("\n0101101\n", "\n1101101\n", StringComparison.Ordinal);
        Shape aRead = Read(aText);
        Assert.True(aRead.Free());
        foreach (Shape aSub in SubShapes(aRead).Where(s => s.Type != ShapeType.Solid))
        {
            Assert.False(aSub.Free());
        }
    }

    [Fact]
    public void Version_2_stores_UV_points_and_version_1_computes_them()
    {
        Face aFace = TestShapes.CylinderFace(2, 5);
        string aV2 = Write(aFace, FormatVersion_VERSION_2);
        string aV1 = Write(aFace, FormatVersion_VERSION_1);
        Assert.True(aV2.Length > aV1.Length);

        Face aFromV2 = ShapeCasts.Face(Read(aV2));
        Face aFromV1 = ShapeCasts.Face(Read(aV1));
        Assert.False(aFromV2.Checked());
        // BRepTools::Update(Face) on reading version 1
        Assert.True(aFromV1.Checked());
        foreach ((Face aF, string aWhich) in new[] { (aFromV2, "2"), (aFromV1, "1") })
        {
            for (var anExp = new Explorer(aF, ShapeType.Edge); anExp.More(); anExp.Next())
            {
                Edge anEdge = ShapeCasts.Edge(anExp.Current());
                Tool.UVPoints(anEdge, aF, out Point2D aP1, out Point2D aP2);
                Curve2D aPC = Tool.CurveOnSurface(anEdge, aF, out double aFirst, out double aLast)!;
                Assert.True(aP1.DistanceTo(aPC.Value(aFirst)) < 1e-12, aWhich);
                Assert.True(aP2.DistanceTo(aPC.Value(aLast)) < 1e-12, aWhich);
            }
        }
    }

    [Fact]
    public void Triangulations_and_normals_follow_the_flags_and_the_version()
    {
        Shape aMeshed = BinTestShapes.MeshedBox(true, true);
        Assert.Contains("Triangulations 0\n", Write(aMeshed, FormatVersion_VERSION_3, false, false), StringComparison.Ordinal);
        string aWithout = Write(aMeshed, FormatVersion_VERSION_3, true, false);
        string aWith = Write(aMeshed, FormatVersion_VERSION_3, true, true);
        string aV2 = Write(aMeshed, FormatVersion_VERSION_2, true, true);
        Assert.DoesNotContain("Triangulations 0\n", aWithout, StringComparison.Ordinal);
        Assert.True(aWith.Length > aWithout.Length);
        // version 2 has no normals
        Assert.False(Tool.Triangulation(ShapeCasts.Face(SubShapes(Read(aV2)).First(s => s.Type == ShapeType.Face)), out _)!.HasNormals);

        Triangulation aTri = Tool.Triangulation(ShapeCasts.Face(SubShapes(Read(aWith)).First(s => s.Type == ShapeType.Face)), out _)!;
        Assert.True(aTri.HasNormals);
        Assert.True(aTri.HasUVNodes);
        aTri = Tool.Triangulation(ShapeCasts.Face(SubShapes(Read(aWithout)).First(s => s.Type == ShapeType.Face)), out _)!;
        Assert.False(aTri.HasNormals);

        // a face with only a triangulation is always written with its triangulation and normals
        Shape anOnly = BinTestShapes.TriangulationOnlyFace();
        Face aFace = ShapeCasts.Face(Read(Write(anOnly, FormatVersion_VERSION_3, false, false)));
        Assert.NotNull(Tool.Triangulation(aFace, out _));
        Assert.Null(Tool.Surface(aFace));
    }

    [Fact]
    public void Files_carry_Draws_header_and_read_back()
    {
        string aDir = Directory.CreateTempSubdirectory("occtnet-brep-").FullName;
        try
        {
            Shape aShape = BinTestShapes.CompoundWithLocations();
            string aFile = Path.Combine(aDir, "a.brep");
            Assert.True(BrepTools.Write(aShape, aFile));
            byte[] aBytes = File.ReadAllBytes(aFile);
            Assert.Equal("DBRep_DrawableShape\n" + Write(aShape), System.Text.Encoding.ASCII.GetString(aBytes));
            Assert.True(BrepTools.Read(out Shape aRead, aFile));
            Assert.Equal(Write(aShape), Write(aRead));

            // CRLF reads the same
            string aCrlf = Path.Combine(aDir, "crlf.brep");
            File.WriteAllText(aCrlf, File.ReadAllText(aFile).Replace("\n", "\r\n", StringComparison.Ordinal));
            Assert.True(BrepTools.Read(out Shape aFromCrlf, aCrlf));
            Assert.Equal(Write(aShape), Write(aFromCrlf));

            Assert.True(BrepTools.Write(aShape, Path.Combine(aDir, "v1.brep"), false, false, FormatVersion_VERSION_1));
            Assert.StartsWith("DBRep_DrawableShape\n\nCASCADE Topology V1", File.ReadAllText(Path.Combine(aDir, "v1.brep")), StringComparison.Ordinal);

            Assert.False(BrepTools.Read(out Shape aNone, Path.Combine(aDir, "missing.brep")));
            Assert.True(aNone.IsNull);
            Assert.False(BrepTools.Write(aShape, Path.Combine(aDir, "no", "such", "dir", "a.brep")));
            File.WriteAllText(Path.Combine(aDir, "empty.brep"), "not a shape\n");
            Assert.False(BrepTools.Read(out _, Path.Combine(aDir, "empty.brep")));
        }
        finally
        {
            Directory.Delete(aDir, true);
        }
    }

    [Fact]
    public void Text_that_isnt_a_shape_set_reads_as_a_null_shape()
    {
        Assert.True(Read("hello\nworld\n").IsNull);
        Assert.True(Read("").IsNull);
        // a version line longer than getline's 99 characters ends the search, as in C++
        Assert.True(Read(new string('x', 150) + "\n" + Write(TestShapes.Vertex(new Point3D()))).IsNull);
        // lines before the version line are skipped
        Assert.False(Read("anything\n" + Write(TestShapes.Vertex(new Point3D()))).IsNull);
    }

    [Fact]
    public void The_shape_set_API()
    {
        Solid aBox = TestShapes.Box(1, 2, 3);
        var aSet = new BrepShapeSet();
        Assert.True(aSet.IsWithTriangles);
        Assert.False(aSet.IsWithNormals);
        aSet.SetWithNormals(true);
        aSet.SetWithTriangles(false);
        Assert.True(aSet.IsWithNormals);
        Assert.False(aSet.IsWithTriangles);
        Assert.Equal(3, aSet.FormatNb());
        aSet.SetFormatNb(7);
        Assert.Equal(3, aSet.FormatNb());
        aSet.SetFormatNb(2);
        Assert.Equal(2, aSet.FormatNb());

        int anIndex = aSet.Add(aBox);
        Assert.Equal(8 + 12 + 6 * 2 + 1 + 1, aSet.ShapeCount);
        Assert.Equal(anIndex, aSet.ShapeCount);
        Assert.Equal(anIndex, aSet.Index(aBox));
        Assert.True(aSet.Shape(anIndex).IsSame(aBox));
        Assert.Equal(0, aSet.Add(new Shape()));
        Assert.Equal(anIndex, aSet.Add(aBox.Moved(new Placement(new Transform3D()))));

        var anExtent = new StringWriter();
        aSet.DumpExtent(anExtent);
        Assert.Equal(" VERTEX    : 8\n EDGE      : 12\n WIRE      : 6\n FACE      : 6\n SHELL     : 1\n SOLID     : 1\n"
                     + " COMPSOLID : 0\n COMPOUND  : 0\n SHAPE     : 34\n", anExtent.ToString());
        string anExtentString = "x";
        aSet.DumpExtent(ref anExtentString);
        Assert.Equal("x" + anExtent, anExtentString);

        var aShapeRef = new StringWriter();
        aSet.Write(aBox.Reversed(), aShapeRef);
        Assert.Equal("-1 0 ", aShapeRef.ToString());
        aSet.Read(out Shape aBack, new StringReader("-1 0"));
        Assert.True(aBack.IsEqual(aBox.Reversed()));

        aSet.Clear();
        Assert.Equal(0, aSet.ShapeCount);
        Assert.Equal(0, aSet.Locations().LocationCount);
    }

    [Fact]
    public void Dumps_describe_the_shapes()
    {
        Shape aShape = BinTestShapes.MeshedCylinder();
        var aDump = new StringWriter();
        BrepTools.Dump(aShape, aDump);
        string aText = aDump.ToString();
        Assert.Contains("Flags : Free, Modified, Checked, Orientable, Closed, Infinite, Convex, Locked", aText, StringComparison.Ordinal);
        Assert.Contains("TShape # 1 : FACE      ", aText, StringComparison.Ordinal);
        Assert.Contains("    - PCurve : ", aText, StringComparison.Ordinal);
        Assert.Contains("Dump of 1 Triangulations", aText, StringComparison.Ordinal);
        Assert.Contains("Dump of ", aText, StringComparison.Ordinal);

        // the Locked column is the stored bit: 0 for a frozen shape that wasn't locked
        Shape aRead = Read(Write(aShape));
        aDump = new StringWriter();
        ShapeTools.Dump(aRead, aDump);
        aText = aDump.ToString();
        Assert.StartsWith("Shape : 7, FORWARD\n", aText, StringComparison.Ordinal);
        Assert.Contains("TShape # 1 : FACE      11010000 ", aText, StringComparison.Ordinal);
        Assert.Contains("TShape # 2 : WIRE      01011000 ", aText, StringComparison.Ordinal);
        Assert.DoesNotContain("PCurve", aText, StringComparison.Ordinal);

        aDump = new StringWriter();
        BrepTools.Dump(new Shape(), aDump);
        Assert.StartsWith("Null shape\nShape : 0, EXTERNAL\n", aDump.ToString(), StringComparison.Ordinal);
    }

    [Fact]
    public void Writing_restores_the_precision_and_leaves_the_shape_unchanged()
    {
        Shape aShape = TestShapes.Frozen(BinTestShapes.MeshedBox(true, false));
        var aWriter = new StringWriter();
        StreamUtilities.Precision(aWriter, 9);
        BrepTools.Write(aShape, aWriter);
        Assert.Equal(9, StreamUtilities.Precision(aWriter));
        Assert.Equal(aWriter.ToString(), Write(aShape));
    }
}
