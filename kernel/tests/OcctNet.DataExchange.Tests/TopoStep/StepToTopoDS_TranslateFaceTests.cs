// Port of: src/DataExchange/TKDESTEP/GTests/StepToTopoDS_TranslateFace_Test.cxx
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace OcctNet.DataExchange.Tests.TopoStep;

/// <summary>OCCT's GTests of StepToTopoDS_TranslateFace on triangulated faces and surface sets.</summary>
public sealed class StepToTopoDS_TranslateFaceTests
{
    private readonly StepToTopoDS_Tool myTool = new();
    private readonly StepToTopoDS_NMTool myNMTool = new();

    public StepToTopoDS_TranslateFaceTests()
    {
        var aTP = new Transfer_TransientProcess();
        myTool.Init(new NCollection_DataMap<StepShape_TopologicalRepresentationItem, TopoDS_Shape>(), aTP);
        myNMTool.SetActive(false);
    }

    private static StepVisual_CoordinatesList createCoords(params gp_XYZ[] thePoints)
    {
        var aPoints = new NCollection_HArray1<gp_XYZ>(1, thePoints.Length);
        for (int i = 0; i < thePoints.Length; ++i)
        {
            aPoints.SetValue(i + 1, thePoints[i]);
        }

        var aCoords = new StepVisual_CoordinatesList();
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

    private static NCollection_HArray2<double> createNormals(params gp_XYZ[] theNorms)
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

    private static Poly_Triangulation getMesh(StepToTopoDS_TranslateFace theTranslator)
    {
        Assert.True(theTranslator.IsDone());
        TopoDS_Face aFace = TopoDS.Face(theTranslator.Value());
        Poly_Triangulation? aMesh = BRep_Tool.Triangulation(aFace, out _);
        Assert.NotNull(aMesh);
        return aMesh;
    }

    [Fact]
    public void TriangulatedFace_DirectNodes()
    {
        var aTF = new StepVisual_TriangulatedFace();
        aTF.Init(new TCollection_HAsciiString("face"),
                 createCoords(new gp_XYZ(0, 0, 0), new gp_XYZ(1, 0, 0), new gp_XYZ(1, 1, 0), new gp_XYZ(0, 1, 0)), 4, null, false,
                 new StepVisual_FaceOrSurface(), null, createTriangles([1, 2, 3], [1, 3, 4]));

        bool aHasGeom = false;
        var aTranslator = new StepToTopoDS_TranslateFace(aTF, myTool, myNMTool, false, ref aHasGeom);

        Poly_Triangulation aMesh = getMesh(aTranslator);
        Assert.Equal(4, aMesh.NbNodes());
        Assert.Equal(2, aMesh.NbTriangles());
        Assert.False(aMesh.HasNormals());
        Assert.Equal(0.0, aMesh.Node(1).X(), 1e-12);
        Assert.Equal(1.0, aMesh.Node(2).X(), 1e-12);
        Assert.Equal(1.0, aMesh.Node(3).Y(), 1e-12);

        aMesh.Triangle(1).Get(out int aN1, out int aN2, out int aN3);
        Assert.Equal(1, aN1);
        Assert.Equal(2, aN2);
        Assert.Equal(3, aN3);
    }

    [Fact]
    public void TriangulatedFace_WithPnindex()
    {
        var aTF = new StepVisual_TriangulatedFace();
        aTF.Init(new TCollection_HAsciiString("face"),
                 createCoords(new gp_XYZ(0, 0, 0), new gp_XYZ(10, 0, 0), new gp_XYZ(10, 10, 0), new gp_XYZ(0, 10, 0)), 4, null, false,
                 new StepVisual_FaceOrSurface(), intArray(2, 3, 4), createTriangles([1, 2, 3]));

        bool aHasGeom = false;
        var aTranslator = new StepToTopoDS_TranslateFace(aTF, myTool, myNMTool, false, ref aHasGeom);

        Poly_Triangulation aMesh = getMesh(aTranslator);
        Assert.Equal(3, aMesh.NbNodes());
        Assert.Equal(1, aMesh.NbTriangles());
        Assert.Equal(10.0, aMesh.Node(1).X(), 1e-12);
        Assert.Equal(0.0, aMesh.Node(1).Y(), 1e-12);
        Assert.Equal(10.0, aMesh.Node(2).X(), 1e-12);
        Assert.Equal(10.0, aMesh.Node(2).Y(), 1e-12);
        Assert.Equal(0.0, aMesh.Node(3).X(), 1e-12);
        Assert.Equal(10.0, aMesh.Node(3).Y(), 1e-12);
    }

    [Fact]
    public void TriangulatedSurfaceSet_DirectNodes()
    {
        var aTSS = new StepVisual_TriangulatedSurfaceSet();
        aTSS.Init(new TCollection_HAsciiString("surfset"), createCoords(new gp_XYZ(0, 0, 0), new gp_XYZ(2, 0, 0), new gp_XYZ(1, 2, 0)), 3,
                  null, null, createTriangles([1, 2, 3]));

        var aTranslator = new StepToTopoDS_TranslateFace(aTSS, myTool, myNMTool);

        Poly_Triangulation aMesh = getMesh(aTranslator);
        Assert.Equal(3, aMesh.NbNodes());
        Assert.Equal(1, aMesh.NbTriangles());
        Assert.Equal(0.0, aMesh.Node(1).X(), 1e-12);
        Assert.Equal(2.0, aMesh.Node(2).X(), 1e-12);
        Assert.Equal(2.0, aMesh.Node(3).Y(), 1e-12);
    }

    [Fact]
    public void TriangulatedSurfaceSet_WithPnindex()
    {
        var aTSS = new StepVisual_TriangulatedSurfaceSet();
        aTSS.Init(new TCollection_HAsciiString("surfset"),
                  createCoords(new gp_XYZ(0, 0, 0), new gp_XYZ(5, 0, 0), new gp_XYZ(5, 5, 0), new gp_XYZ(0, 5, 0)), 4, null,
                  intArray(1, 3, 4), createTriangles([1, 2, 3]));

        var aTranslator = new StepToTopoDS_TranslateFace(aTSS, myTool, myNMTool);

        Poly_Triangulation aMesh = getMesh(aTranslator);
        Assert.Equal(3, aMesh.NbNodes());
        Assert.Equal(0.0, aMesh.Node(1).X(), 1e-12);
        Assert.Equal(5.0, aMesh.Node(2).X(), 1e-12);
        Assert.Equal(5.0, aMesh.Node(2).Y(), 1e-12);
        Assert.Equal(0.0, aMesh.Node(3).X(), 1e-12);
        Assert.Equal(5.0, aMesh.Node(3).Y(), 1e-12);
    }

    [Fact]
    public void TriangulatedFace_WithNormals()
    {
        var aTF = new StepVisual_TriangulatedFace();
        aTF.Init(new TCollection_HAsciiString("face"), createCoords(new gp_XYZ(0, 0, 0), new gp_XYZ(1, 0, 0), new gp_XYZ(0, 1, 0)), 3,
                 createNormals(new gp_XYZ(0, 0, 1), new gp_XYZ(0, 0, 1), new gp_XYZ(0, 0, 1)), false, new StepVisual_FaceOrSurface(), null,
                 createTriangles([1, 2, 3]));

        bool aHasGeom = false;
        var aTranslator = new StepToTopoDS_TranslateFace(aTF, myTool, myNMTool, false, ref aHasGeom);

        Poly_Triangulation aMesh = getMesh(aTranslator);
        Assert.True(aMesh.HasNormals());
        gp_Dir aNormal = aMesh.Normal(1);
        Assert.Equal(1.0, aNormal.Z(), 1e-12);
    }

    [Fact]
    public void TriangulatedFace_SingleNormal()
    {
        var aTF = new StepVisual_TriangulatedFace();
        aTF.Init(new TCollection_HAsciiString("face"), createCoords(new gp_XYZ(0, 0, 0), new gp_XYZ(1, 0, 0), new gp_XYZ(0, 1, 0)), 3,
                 createNormals(new gp_XYZ(0, 1, 0)), false, new StepVisual_FaceOrSurface(), null, createTriangles([1, 2, 3]));

        bool aHasGeom = false;
        var aTranslator = new StepToTopoDS_TranslateFace(aTF, myTool, myNMTool, false, ref aHasGeom);

        Poly_Triangulation aMesh = getMesh(aTranslator);
        Assert.True(aMesh.HasNormals());
        // All 3 nodes should share the same normal (0,1,0).
        for (int i = 1; i <= 3; ++i)
        {
            gp_Dir aNormal = aMesh.Normal(i);
            Assert.Equal(0.0, aNormal.X(), 1e-12);
            Assert.Equal(1.0, aNormal.Y(), 1e-12);
            Assert.Equal(0.0, aNormal.Z(), 1e-12);
        }
    }

    [Fact]
    public void TriangulatedSurfaceSet_LengthFactor()
    {
        var aTSS = new StepVisual_TriangulatedSurfaceSet();
        aTSS.Init(new TCollection_HAsciiString("surfset"), createCoords(new gp_XYZ(0, 0, 0), new gp_XYZ(1, 0, 0), new gp_XYZ(0, 1, 0)), 3,
                  null, null, createTriangles([1, 2, 3]));

        StepData_Factors aFactors = new StepData_Factors().InitializedFactors(25.4, 1.0, 1.0);
        var aTranslator = new StepToTopoDS_TranslateFace(aTSS, myTool, myNMTool, aFactors);

        Poly_Triangulation aMesh = getMesh(aTranslator);
        // Coordinates should be scaled by length factor 25.4.
        Assert.Equal(25.4, aMesh.Node(2).X(), 1e-10);
        Assert.Equal(25.4, aMesh.Node(3).Y(), 1e-10);
    }

    private static StepVisual_ComplexTriangulatedFace complexFace(StepVisual_CoordinatesList theCoords, int thePnmax,
                                                                   NCollection_HArray1<Standard_Transient>? theStrips,
                                                                   NCollection_HArray1<Standard_Transient>? theFans)
    {
        var aCTF = new StepVisual_ComplexTriangulatedFace();
        aCTF.Init(new TCollection_HAsciiString("complex_face"), theCoords, thePnmax, null, false, new StepVisual_FaceOrSurface(), null,
                  theStrips, theFans);
        return aCTF;
    }

    private static NCollection_HArray1<Standard_Transient> oneList(NCollection_HArray1<int> theList)
    {
        var aLists = new NCollection_HArray1<Standard_Transient>(1, 1);
        aLists.SetValue(1, theList);
        return aLists;
    }

    [Fact]
    public void ComplexTriangulatedFace_TriangleStrips()
    {
        // Triangle strip: [1, 2, 3, 4] produces 2 triangles.
        StepVisual_ComplexTriangulatedFace aCTF =
            complexFace(createCoords(new gp_XYZ(0, 0, 0), new gp_XYZ(1, 0, 0), new gp_XYZ(0, 1, 0), new gp_XYZ(1, 1, 0)), 4,
                        oneList(intArray(1, 2, 3, 4)), null);

        bool aHasGeom = false;
        var aTranslator = new StepToTopoDS_TranslateFace(aCTF, myTool, myNMTool, false, ref aHasGeom);

        Poly_Triangulation aMesh = getMesh(aTranslator);
        Assert.Equal(4, aMesh.NbNodes());
        Assert.Equal(2, aMesh.NbTriangles());
    }

    [Fact]
    public void ComplexTriangulatedFace_TriangleFans()
    {
        // Triangle fan: [1, 2, 3, 4, 5] with center at vertex 1 produces 3 triangles.
        StepVisual_ComplexTriangulatedFace aCTF =
            complexFace(createCoords(new gp_XYZ(0, 0, 0), new gp_XYZ(1, 0, 0), new gp_XYZ(1, 1, 0), new gp_XYZ(0, 1, 0), new gp_XYZ(-1, 0, 0)),
                        5, null, oneList(intArray(1, 2, 3, 4, 5)));

        bool aHasGeom = false;
        var aTranslator = new StepToTopoDS_TranslateFace(aCTF, myTool, myNMTool, false, ref aHasGeom);

        Poly_Triangulation aMesh = getMesh(aTranslator);
        Assert.Equal(5, aMesh.NbNodes());
        Assert.Equal(3, aMesh.NbTriangles());
    }

    [Fact]
    public void NullInput_TessellatedFace()
    {
        StepVisual_TessellatedFace? aNullTF = null;
        bool aHasGeom = false;
        var aTranslator = new StepToTopoDS_TranslateFace();
        aTranslator.Init(aNullTF!, myTool, myNMTool, false, ref aHasGeom);
        Assert.False(aTranslator.IsDone());
    }

    [Fact]
    public void NullInput_TessellatedSurfaceSet()
    {
        StepVisual_TessellatedSurfaceSet? aNullTSS = null;
        var aTranslator = new StepToTopoDS_TranslateFace();
        aTranslator.Init(aNullTSS!, myTool, myNMTool);
        Assert.False(aTranslator.IsDone());
    }

    [Fact]
    public void ComplexTriangulatedFace_DegenerateStrip()
    {
        // Strip with degenerate: [1, 2, 3, 3] - the second triangle (2,3,3) is degenerate.
        StepVisual_ComplexTriangulatedFace aCTF =
            complexFace(createCoords(new gp_XYZ(0, 0, 0), new gp_XYZ(1, 0, 0), new gp_XYZ(0, 1, 0), new gp_XYZ(1, 1, 0)), 4,
                        oneList(intArray(1, 2, 3, 3)), null);

        bool aHasGeom = false;
        var aTranslator = new StepToTopoDS_TranslateFace(aCTF, myTool, myNMTool, false, ref aHasGeom);

        Poly_Triangulation aMesh = getMesh(aTranslator);
        // Only 1 non-degenerate triangle from strip [1,2,3,3].
        Assert.Equal(1, aMesh.NbTriangles());
    }
}
