// Port of: src/DataExchange/TKDESTEP/GTests/StepToTopoDS_TranslateFace_Test.cxx
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Import;
using Kerf.Engine.Exchange.Step.Shapes;
using Kerf.Engine.Exchange.Step.Visual;
using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Meshing;

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Kernel.DataExchange.Tests.TopoStep;

/// <summary>OCCT's GTests of StepToTopoDS_TranslateFace on triangulated faces and surface sets.</summary>
public sealed class StepToTopoDS_TranslateFaceTests
{
    private readonly ImportTool myTool = new();
    private readonly NMTool myNMTool = new();

    public StepToTopoDS_TranslateFaceTests()
    {
        var aTP = new TransientProcess();
        myTool.Init(new Dictionary<StepTopologicalRepresentationItem, Shape>(), aTP);
        myNMTool.SetActive(false);
    }

    private static CoordinatesList createCoords(params Xyz[] thePoints)
    {
        var aPoints = new NCollection_HArray1<Xyz>(1, thePoints.Length);
        for (int i = 0; i < thePoints.Length; ++i)
        {
            aPoints.SetValue(i + 1, thePoints[i]);
        }

        var aCoords = new CoordinatesList();
        aCoords.Init(new TCollection_HAsciiString("coords"), aPoints);
        return aCoords;
    }

    private static NCollection_HArray2<int> createTriangles(params int[][] theTriData)
    {
        var aTriangles = new NCollection_HArray2<int>(1, theTriData.Length, 1, 3);
        for (int i = 1; i <= theTriData.Length; ++i)
        {
            aTriangles.SetValue(i, 1, theTriData[i - 1][0]);
            aTriangles.SetValue(i, 2, theTriData[i - 1][1]);
            aTriangles.SetValue(i, 3, theTriData[i - 1][2]);
        }

        return aTriangles;
    }

    private static NCollection_HArray2<double> createNormals(params Xyz[] theNorms)
    {
        var aNormals = new NCollection_HArray2<double>(1, theNorms.Length, 1, 3);
        for (int i = 1; i <= theNorms.Length; ++i)
        {
            aNormals.SetValue(i, 1, theNorms[i - 1].X());
            aNormals.SetValue(i, 2, theNorms[i - 1].Y());
            aNormals.SetValue(i, 3, theNorms[i - 1].Z());
        }

        return aNormals;
    }

    private static NCollection_HArray1<int> intArray(params int[] theValues)
    {
        var anArray = new NCollection_HArray1<int>(1, theValues.Length);
        for (int i = 0; i < theValues.Length; ++i)
        {
            anArray.SetValue(i + 1, theValues[i]);
        }

        return anArray;
    }

    private static Triangulation getMesh(TranslateFace theTranslator)
    {
        Assert.True(theTranslator.IsDone);
        Face aFace = ShapeCasts.Face(theTranslator.Value());
        Triangulation? aMesh = Tool.Triangulation(aFace, out _);
        Assert.NotNull(aMesh);
        return aMesh;
    }

    [Fact]
    public void TriangulatedFace_DirectNodes()
    {
        var aTF = new TriangulatedFace();
        aTF.Init(new TCollection_HAsciiString("face"),
                 createCoords(new Xyz(0, 0, 0), new Xyz(1, 0, 0), new Xyz(1, 1, 0), new Xyz(0, 1, 0)), 4, null, false,
                 new FaceOrSurface(), null, createTriangles([1, 2, 3], [1, 3, 4]));

        bool aHasGeom = false;
        var aTranslator = new TranslateFace(aTF, myTool, myNMTool, false, ref aHasGeom);

        Triangulation aMesh = getMesh(aTranslator);
        Assert.Equal(4, aMesh.NodeCount);
        Assert.Equal(2, aMesh.TriangleCount);
        Assert.False(aMesh.HasNormals);
        Assert.Equal(0.0, aMesh.Node(1).X, 1e-12);
        Assert.Equal(1.0, aMesh.Node(2).X, 1e-12);
        Assert.Equal(1.0, aMesh.Node(3).Y, 1e-12);

        aMesh.Triangle(1).Get(out int aN1, out int aN2, out int aN3);
        Assert.Equal(1, aN1);
        Assert.Equal(2, aN2);
        Assert.Equal(3, aN3);
    }

    [Fact]
    public void TriangulatedFace_WithPnindex()
    {
        var aTF = new TriangulatedFace();
        aTF.Init(new TCollection_HAsciiString("face"),
                 createCoords(new Xyz(0, 0, 0), new Xyz(10, 0, 0), new Xyz(10, 10, 0), new Xyz(0, 10, 0)), 4, null, false,
                 new FaceOrSurface(), intArray(2, 3, 4), createTriangles([1, 2, 3]));

        bool aHasGeom = false;
        var aTranslator = new TranslateFace(aTF, myTool, myNMTool, false, ref aHasGeom);

        Triangulation aMesh = getMesh(aTranslator);
        Assert.Equal(3, aMesh.NodeCount);
        Assert.Equal(1, aMesh.TriangleCount);
        Assert.Equal(10.0, aMesh.Node(1).X, 1e-12);
        Assert.Equal(0.0, aMesh.Node(1).Y, 1e-12);
        Assert.Equal(10.0, aMesh.Node(2).X, 1e-12);
        Assert.Equal(10.0, aMesh.Node(2).Y, 1e-12);
        Assert.Equal(0.0, aMesh.Node(3).X, 1e-12);
        Assert.Equal(10.0, aMesh.Node(3).Y, 1e-12);
    }

    [Fact]
    public void TriangulatedSurfaceSet_DirectNodes()
    {
        var aTSS = new TriangulatedSurfaceSet();
        aTSS.Init(new TCollection_HAsciiString("surfset"), createCoords(new Xyz(0, 0, 0), new Xyz(2, 0, 0), new Xyz(1, 2, 0)), 3,
                  null, null, createTriangles([1, 2, 3]));

        var aTranslator = new TranslateFace(aTSS, myTool, myNMTool);

        Triangulation aMesh = getMesh(aTranslator);
        Assert.Equal(3, aMesh.NodeCount);
        Assert.Equal(1, aMesh.TriangleCount);
        Assert.Equal(0.0, aMesh.Node(1).X, 1e-12);
        Assert.Equal(2.0, aMesh.Node(2).X, 1e-12);
        Assert.Equal(2.0, aMesh.Node(3).Y, 1e-12);
    }

    [Fact]
    public void TriangulatedSurfaceSet_WithPnindex()
    {
        var aTSS = new TriangulatedSurfaceSet();
        aTSS.Init(new TCollection_HAsciiString("surfset"),
                  createCoords(new Xyz(0, 0, 0), new Xyz(5, 0, 0), new Xyz(5, 5, 0), new Xyz(0, 5, 0)), 4, null,
                  intArray(1, 3, 4), createTriangles([1, 2, 3]));

        var aTranslator = new TranslateFace(aTSS, myTool, myNMTool);

        Triangulation aMesh = getMesh(aTranslator);
        Assert.Equal(3, aMesh.NodeCount);
        Assert.Equal(0.0, aMesh.Node(1).X, 1e-12);
        Assert.Equal(5.0, aMesh.Node(2).X, 1e-12);
        Assert.Equal(5.0, aMesh.Node(2).Y, 1e-12);
        Assert.Equal(0.0, aMesh.Node(3).X, 1e-12);
        Assert.Equal(5.0, aMesh.Node(3).Y, 1e-12);
    }

    [Fact]
    public void TriangulatedFace_WithNormals()
    {
        var aTF = new TriangulatedFace();
        aTF.Init(new TCollection_HAsciiString("face"), createCoords(new Xyz(0, 0, 0), new Xyz(1, 0, 0), new Xyz(0, 1, 0)), 3,
                 createNormals(new Xyz(0, 0, 1), new Xyz(0, 0, 1), new Xyz(0, 0, 1)), false, new FaceOrSurface(), null,
                 createTriangles([1, 2, 3]));

        bool aHasGeom = false;
        var aTranslator = new TranslateFace(aTF, myTool, myNMTool, false, ref aHasGeom);

        Triangulation aMesh = getMesh(aTranslator);
        Assert.True(aMesh.HasNormals);
        Direction3D aNormal = aMesh.Normal(1);
        Assert.Equal(1.0, aNormal.Z, 1e-12);
    }

    [Fact]
    public void TriangulatedFace_SingleNormal()
    {
        var aTF = new TriangulatedFace();
        aTF.Init(new TCollection_HAsciiString("face"), createCoords(new Xyz(0, 0, 0), new Xyz(1, 0, 0), new Xyz(0, 1, 0)), 3,
                 createNormals(new Xyz(0, 1, 0)), false, new FaceOrSurface(), null, createTriangles([1, 2, 3]));

        bool aHasGeom = false;
        var aTranslator = new TranslateFace(aTF, myTool, myNMTool, false, ref aHasGeom);

        Triangulation aMesh = getMesh(aTranslator);
        Assert.True(aMesh.HasNormals);
        // All 3 nodes should share the same normal (0,1,0).
        for (int i = 1; i <= 3; ++i)
        {
            Direction3D aNormal = aMesh.Normal(i);
            Assert.Equal(0.0, aNormal.X, 1e-12);
            Assert.Equal(1.0, aNormal.Y, 1e-12);
            Assert.Equal(0.0, aNormal.Z, 1e-12);
        }
    }

    [Fact]
    public void TriangulatedSurfaceSet_LengthFactor()
    {
        var aTSS = new TriangulatedSurfaceSet();
        aTSS.Init(new TCollection_HAsciiString("surfset"), createCoords(new Xyz(0, 0, 0), new Xyz(1, 0, 0), new Xyz(0, 1, 0)), 3,
                  null, null, createTriangles([1, 2, 3]));

        Factors aFactors = new Factors().InitializedFactors(25.4, 1.0, 1.0);
        var aTranslator = new TranslateFace(aTSS, myTool, myNMTool, aFactors);

        Triangulation aMesh = getMesh(aTranslator);
        // Coordinates should be scaled by length factor 25.4.
        Assert.Equal(25.4, aMesh.Node(2).X, 1e-10);
        Assert.Equal(25.4, aMesh.Node(3).Y, 1e-10);
    }

    private static ComplexTriangulatedFace complexFace(CoordinatesList theCoords, int thePnmax,
                                                                   NCollection_HArray1<Transient>? theStrips,
                                                                   NCollection_HArray1<Transient>? theFans)
    {
        var aCTF = new ComplexTriangulatedFace();
        aCTF.Init(new TCollection_HAsciiString("complex_face"), theCoords, thePnmax, null, false, new FaceOrSurface(), null,
                  theStrips, theFans);
        return aCTF;
    }

    private static NCollection_HArray1<Transient> oneList(NCollection_HArray1<int> theList)
    {
        var aLists = new NCollection_HArray1<Transient>(1, 1);
        aLists.SetValue(1, theList);
        return aLists;
    }

    [Fact]
    public void ComplexTriangulatedFace_TriangleStrips()
    {
        // Triangle strip: [1, 2, 3, 4] produces 2 triangles.
        ComplexTriangulatedFace aCTF =
            complexFace(createCoords(new Xyz(0, 0, 0), new Xyz(1, 0, 0), new Xyz(0, 1, 0), new Xyz(1, 1, 0)), 4,
                        oneList(intArray(1, 2, 3, 4)), null);

        bool aHasGeom = false;
        var aTranslator = new TranslateFace(aCTF, myTool, myNMTool, false, ref aHasGeom);

        Triangulation aMesh = getMesh(aTranslator);
        Assert.Equal(4, aMesh.NodeCount);
        Assert.Equal(2, aMesh.TriangleCount);
    }

    [Fact]
    public void ComplexTriangulatedFace_TriangleFans()
    {
        // Triangle fan: [1, 2, 3, 4, 5] with center at vertex 1 produces 3 triangles.
        ComplexTriangulatedFace aCTF =
            complexFace(createCoords(new Xyz(0, 0, 0), new Xyz(1, 0, 0), new Xyz(1, 1, 0), new Xyz(0, 1, 0), new Xyz(-1, 0, 0)),
                        5, null, oneList(intArray(1, 2, 3, 4, 5)));

        bool aHasGeom = false;
        var aTranslator = new TranslateFace(aCTF, myTool, myNMTool, false, ref aHasGeom);

        Triangulation aMesh = getMesh(aTranslator);
        Assert.Equal(5, aMesh.NodeCount);
        Assert.Equal(3, aMesh.TriangleCount);
    }

    [Fact]
    public void NullInput_TessellatedFace()
    {
        TessellatedFace? aNullTF = null;
        bool aHasGeom = false;
        var aTranslator = new TranslateFace();
        aTranslator.Init(aNullTF!, myTool, myNMTool, false, ref aHasGeom);
        Assert.False(aTranslator.IsDone);
    }

    [Fact]
    public void NullInput_TessellatedSurfaceSet()
    {
        TessellatedSurfaceSet? aNullTSS = null;
        var aTranslator = new TranslateFace();
        aTranslator.Init(aNullTSS!, myTool, myNMTool);
        Assert.False(aTranslator.IsDone);
    }

    [Fact]
    public void ComplexTriangulatedFace_DegenerateStrip()
    {
        // Strip with degenerate: [1, 2, 3, 3] - the second triangle (2,3,3) is degenerate.
        ComplexTriangulatedFace aCTF =
            complexFace(createCoords(new Xyz(0, 0, 0), new Xyz(1, 0, 0), new Xyz(0, 1, 0), new Xyz(1, 1, 0)), 4,
                        oneList(intArray(1, 2, 3, 3)), null);

        bool aHasGeom = false;
        var aTranslator = new TranslateFace(aCTF, myTool, myNMTool, false, ref aHasGeom);

        Triangulation aMesh = getMesh(aTranslator);
        // Only 1 non-degenerate triangle from strip [1,2,3,3].
        Assert.Equal(1, aMesh.TriangleCount);
    }
}
