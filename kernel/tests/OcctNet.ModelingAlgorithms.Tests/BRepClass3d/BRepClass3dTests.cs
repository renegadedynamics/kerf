// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.ModelingData.Tests;
using static Occt.TopAbs_State;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Known answers of BRepClass3d: points in, out and on a box (faces, edges, vertices, within the tolerance), a
/// cylinder made of faces, a sphere, a solid with a void, reversed, located and compound shapes, the infinite point,
/// the outer shell, a point in a face; the classifiers only read their frozen inputs.
/// </summary>
public class BRepClass3dTests
{
    private static TopoDS_Shape Box10() => TestShapes.Frozen(TestShapes.Box(10, 10, 10));

    private static TopAbs_State Classify(in TopoDS_Shape theShape, double x, double y, double z, double theTol = 1e-7)
        => new BRepClass3d_SolidClassifier(theShape, new gp_Pnt(x, y, z), theTol).State();

    [Theory]
    [InlineData(5, 5, 5, TopAbs_IN)]
    [InlineData(0.001, 9.999, 5, TopAbs_IN)]
    [InlineData(15, 5, 5, TopAbs_OUT)]
    [InlineData(-0.001, 5, 5, TopAbs_OUT)]
    [InlineData(5, 5, 0, TopAbs_ON)] // on a face
    [InlineData(10, 5, 5, TopAbs_ON)]
    [InlineData(10, 10, 5, TopAbs_ON)] // on an edge
    [InlineData(0, 0, 0, TopAbs_ON)] // at a vertex
    [InlineData(10, 10, 10, TopAbs_ON)]
    public void Box_states(double x, double y, double z, TopAbs_State theExpected)
    {
        Assert.Equal(theExpected, Classify(Box10(), x, y, z));
    }

    [Fact]
    public void Box_tolerance_widens_the_boundary()
    {
        TopoDS_Shape aBox = Box10();
        Assert.Equal(TopAbs_IN, Classify(aBox, 5, 5, 1e-4));
        Assert.Equal(TopAbs_ON, Classify(aBox, 5, 5, 1e-4, 1e-3));
        Assert.Equal(TopAbs_OUT, Classify(aBox, 5, 5, -1e-4));
        Assert.Equal(TopAbs_ON, Classify(aBox, 5, 5, -1e-4, 1e-3));
    }

    [Fact]
    public void Box_on_a_face_gives_the_face()
    {
        TopoDS_Shape aBox = Box10();
        var aClassifier = new BRepClass3d_SolidClassifier(aBox);
        aClassifier.Perform(new gp_Pnt(5, 5, 10), 1e-7);
        Assert.Equal(TopAbs_ON, aClassifier.State());
        Assert.True(aClassifier.IsOnAFace());
        Assert.False(aClassifier.Rejected());
        Assert.True(aClassifier.Face().IsSame(BRepGPropTestShapes.Faces(aBox)[1])); // the top face

        // one classifier, many points
        aClassifier.Perform(new gp_Pnt(5, 5, 5), 1e-7);
        Assert.Equal(TopAbs_IN, aClassifier.State());
        Assert.False(aClassifier.IsOnAFace());
        aClassifier.Perform(new gp_Pnt(50, 5, 5), 1e-7);
        Assert.Equal(TopAbs_OUT, aClassifier.State());
    }

    [Fact]
    public void Reversed_and_located_boxes()
    {
        TopoDS_Shape aBox = Box10();
        // a reversed solid is the space outside the box
        TopoDS_Shape aReversed = aBox.Reversed();
        Assert.Equal(TopAbs_OUT, Classify(aReversed, 5, 5, 5));
        Assert.Equal(TopAbs_IN, Classify(aReversed, 15, 5, 5));
        Assert.Equal(TopAbs_ON, Classify(aReversed, 5, 5, 0));

        TopoDS_Shape aMoved = ClassificationTestShapes.Translated(aBox, new gp_Vec(100, 0, 0));
        Assert.Equal(TopAbs_IN, Classify(aMoved, 105, 5, 5));
        Assert.Equal(TopAbs_OUT, Classify(aMoved, 5, 5, 5));
        Assert.Equal(TopAbs_ON, Classify(aMoved, 100, 5, 5));

        TopoDS_Shape aRotated = ClassificationTestShapes.RotatedZ(aBox, Math.PI / 4);
        Assert.Equal(TopAbs_IN, Classify(aRotated, 0, 7, 5));
        Assert.Equal(TopAbs_OUT, Classify(aRotated, 7, 0, 5));
    }

    [Fact]
    public void Infinite_point()
    {
        var aClassifier = new BRepClass3d_SolidClassifier(Box10());
        aClassifier.PerformInfinitePoint(1e-7);
        Assert.Equal(TopAbs_OUT, aClassifier.State());

        var aHole = new BRepClass3d_SolidClassifier(Box10().Reversed());
        aHole.PerformInfinitePoint(1e-7);
        Assert.Equal(TopAbs_IN, aHole.State());
    }

    [Theory]
    [InlineData(0, 0, 1, TopAbs_IN)]
    [InlineData(1.9, 0.3, 2.9, TopAbs_IN)]
    [InlineData(3, 0, 1, TopAbs_OUT)]
    [InlineData(1.5, 1.5, 1, TopAbs_OUT)] // in the cylinder's box, outside the cylinder
    [InlineData(0, 0, 3.5, TopAbs_OUT)]
    [InlineData(2, 0, 1, TopAbs_ON)] // on the seam
    [InlineData(0, -2, 1, TopAbs_ON)] // on the lateral face
    [InlineData(0.5, 0.5, 3, TopAbs_ON)] // on the top disc
    [InlineData(0, 2, 0, TopAbs_ON)] // on the bottom circle
    public void Cylinder_states(double x, double y, double z, TopAbs_State theExpected)
    {
        TopoDS_Shape aCylinder = TestShapes.Frozen(BRepGPropTestShapes.CylinderSolid(2.0, 3.0));
        Assert.Equal(theExpected, Classify(aCylinder, x, y, z));
    }

    [Fact]
    public void Sphere_states()
    {
        TopoDS_Shape aSphere = TestShapes.Frozen(BRepGPropTestShapes.SphereSolid(10.0));
        Assert.Equal(TopAbs_IN, Classify(aSphere, 1, 2, 3));
        Assert.Equal(TopAbs_OUT, Classify(aSphere, 8, 8, 8));
        Assert.Equal(TopAbs_ON, Classify(aSphere, 0, 10, 0));
    }

    [Fact]
    public void Solid_with_a_void()
    {
        TopoDS_Shape aSolid = TestShapes.Frozen(ClassificationTestShapes.BoxWithVoid(out TopoDS_Shell anOuter, out _));
        Assert.Equal(TopAbs_IN, Classify(aSolid, 1, 1, 1));
        Assert.Equal(TopAbs_OUT, Classify(aSolid, 4, 4, 4)); // in the void
        Assert.Equal(TopAbs_ON, Classify(aSolid, 3, 4, 4)); // on the void's boundary
        Assert.Equal(TopAbs_OUT, Classify(aSolid, 11, 4, 4));

        // the outer shell is the one a point at infinity is outside of
        TopoDS_Shell aFound = BRepClass3d.OuterShell(TopoDS.Solid(aSolid));
        Assert.True(aFound.IsSame(anOuter));
    }

    [Fact]
    public void OuterShell_of_simple_and_null_solids()
    {
        TopoDS_Solid aBox = TopoDS.Solid(Box10());
        TopoDS_Shell aShell = BRepClass3d.OuterShell(aBox);
        Assert.False(aShell.IsNull());
        Assert.True(aShell.IsSame(new TopoDS_Iterator(aBox).Value()));
        Assert.True(BRepClass3d.OuterShell(new TopoDS_Solid()).IsNull());
    }

    [Fact]
    public void Compound_of_boxes()
    {
        TopoDS_Shape aBox = Box10();
        TopoDS_Shape aCompound = TestShapes.Frozen(BRepGPropTestShapes.CompoundOf(aBox, ClassificationTestShapes.Translated(aBox, new gp_Vec(20, 0, 0))));
        Assert.Equal(TopAbs_IN, Classify(aCompound, 5, 5, 5));
        Assert.Equal(TopAbs_IN, Classify(aCompound, 25, 5, 5));
        Assert.Equal(TopAbs_OUT, Classify(aCompound, 15, 5, 5));
    }

    [Fact]
    public void Solid_without_faces_is_the_whole_space()
    {
        var aB = new BRep_Builder();
        TopoDS_Solid anEmpty = new();
        aB.MakeSolid(ref anEmpty);
        TopoDS_Shape aSolid = TestShapes.Frozen(anEmpty);
        var aClassifier = new BRepClass3d_SolidClassifier(aSolid, new gp_Pnt(1, 2, 3), 1e-7);
        Assert.Equal(TopAbs_IN, aClassifier.State());
        aClassifier.PerformInfinitePoint(1e-7);
        Assert.Equal(TopAbs_IN, aClassifier.State());
    }

    [Fact]
    public void Explorer_finds_a_point_in_a_face()
    {
        TopoDS_Face aFace = TestShapes.Frozen(ClassificationTestShapes.HoledSquare());
        gp_Pnt aP = new();
        double u = 0, v = 0;
        Assert.True(BRepClass3d_SolidExplorer.FindAPointInTheFace(aFace, ref aP, ref u, ref v));
        Assert.Equal(0.0, aP.Z(), 1e-12);
        Assert.Equal(u, aP.X(), 1e-12);
        Assert.Equal(v, aP.Y(), 1e-12);
        Assert.Equal(TopAbs_IN, new BRepClass_FaceClassifier(aFace, new gp_Pnt2d(u, v), 1e-7).State());

        var anExplorer = new BRepClass3d_SolidExplorer(Box10());
        Assert.False(anExplorer.Reject(new gp_Pnt()));
        Assert.Equal(20, anExplorer.GetMapEV().Extent()); // 12 edges and 8 vertices
        int aShells = 0, aFaces = 0;
        for (anExplorer.InitShell(); anExplorer.MoreShell(); anExplorer.NextShell())
        {
            aShells++;
            for (anExplorer.InitFace(); anExplorer.MoreFace(); anExplorer.NextFace())
            {
                aFaces++;
                Assert.True(anExplorer.Intersector(anExplorer.CurrentFace()).Face().IsSame(anExplorer.CurrentFace()));
            }
        }

        Assert.Equal(1, aShells);
        Assert.Equal(6, aFaces);

        // a line from inside towards a face
        gp_Lin aL = new();
        double aPar = 0;
        Assert.Equal(0, anExplorer.Segment(new gp_Pnt(5, 5, 5), ref aL, ref aPar));
        Assert.True(aPar > 0);
        Assert.Equal(1, anExplorer.Segment(new gp_Pnt(5, 5, 0), ref aL, ref aPar)); // on the first face
    }

    [Fact]
    public void Passive_classifier_compares_faces_given_one_by_one()
    {
        TopoDS_Shape aBox = Box10();
        var aClassifier = new BRepClass3d_SolidPassiveClassifier();
        aClassifier.Reset(new gp_Lin(new gp_Pnt(5, 5, 5), new gp_Dir(0, 0, 1)), RealLast(), 1e-7);
        foreach (TopoDS_Face aF in BRepGPropTestShapes.Faces(aBox))
        {
            aClassifier.Compare(aF, aF.Orientation());
        }

        Assert.Equal(TopAbs_IN, aClassifier.State());
        Assert.Equal(5.0, aClassifier.Parameter(), 1e-9);

        var anInter = new BRepClass3d_Intersector3d();
        anInter.Perform(new gp_Lin(new gp_Pnt(5, 5, -5), new gp_Dir(0, 0, 1)), RealLast(), 1e-7, BRepGPropTestShapes.Faces(aBox)[0]);
        Assert.True(anInter.IsDone());
        Assert.True(anInter.HasAPoint());
        Assert.Equal(5.0, anInter.WParameter(), 1e-9);
        Assert.Equal(TopAbs_IN, anInter.State());
        Assert.Equal(IntCurveSurface_TransitionOnCurve.IntCurveSurface_In, anInter.Transition());
    }
}
