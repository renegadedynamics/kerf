// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.ModelingData.Tests;
using static Kerf.Topology.Containment;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Known answers of BRepClass3d: points in, out and on a box (faces, edges, vertices, within the tolerance), a
/// cylinder made of faces, a sphere, a solid with a void, reversed, located and compound shapes, the infinite point,
/// the outer shell, a point in a face; the classifiers only read their frozen inputs.
/// </summary>
public class BRepClass3dTests
{
    private static Shape Box10() => TestShapes.Frozen(TestShapes.Box(10, 10, 10));

    private static Containment Classify(in Shape theShape, double x, double y, double z, double theTol = 1e-7)
        => new global::Kerf.Engine.Classification.SolidClassifier(theShape, new Point3D(x, y, z), theTol).State();

    [Theory]
    [InlineData(5, 5, 5, In)]
    [InlineData(0.001, 9.999, 5, In)]
    [InlineData(15, 5, 5, Out)]
    [InlineData(-0.001, 5, 5, Out)]
    [InlineData(5, 5, 0, On)] // on a face
    [InlineData(10, 5, 5, On)]
    [InlineData(10, 10, 5, On)] // on an edge
    [InlineData(0, 0, 0, On)] // at a vertex
    [InlineData(10, 10, 10, On)]
    public void Box_states(double x, double y, double z, Containment theExpected)
    {
        Assert.Equal(theExpected, Classify(Box10(), x, y, z));
    }

    [Fact]
    public void Box_tolerance_widens_the_boundary()
    {
        Shape aBox = Box10();
        Assert.Equal(In, Classify(aBox, 5, 5, 1e-4));
        Assert.Equal(On, Classify(aBox, 5, 5, 1e-4, 1e-3));
        Assert.Equal(Out, Classify(aBox, 5, 5, -1e-4));
        Assert.Equal(On, Classify(aBox, 5, 5, -1e-4, 1e-3));
    }

    [Fact]
    public void Box_on_a_face_gives_the_face()
    {
        Shape aBox = Box10();
        var aClassifier = new global::Kerf.Engine.Classification.SolidClassifier(aBox);
        aClassifier.Perform(new Point3D(5, 5, 10), 1e-7);
        Assert.Equal(On, aClassifier.State());
        Assert.True(aClassifier.IsOnAFace);
        Assert.False(aClassifier.Rejected());
        Assert.True(aClassifier.Face().IsSame(BRepGPropTestShapes.Faces(aBox)[1])); // the top face

        // one classifier, many points
        aClassifier.Perform(new Point3D(5, 5, 5), 1e-7);
        Assert.Equal(In, aClassifier.State());
        Assert.False(aClassifier.IsOnAFace);
        aClassifier.Perform(new Point3D(50, 5, 5), 1e-7);
        Assert.Equal(Out, aClassifier.State());
    }

    [Fact]
    public void Reversed_and_located_boxes()
    {
        Shape aBox = Box10();
        // a reversed solid is the space outside the box
        Shape aReversed = aBox.Reversed();
        Assert.Equal(Out, Classify(aReversed, 5, 5, 5));
        Assert.Equal(In, Classify(aReversed, 15, 5, 5));
        Assert.Equal(On, Classify(aReversed, 5, 5, 0));

        Shape aMoved = ClassificationTestShapes.Translated(aBox, new Vector3D(100, 0, 0));
        Assert.Equal(In, Classify(aMoved, 105, 5, 5));
        Assert.Equal(Out, Classify(aMoved, 5, 5, 5));
        Assert.Equal(On, Classify(aMoved, 100, 5, 5));

        Shape aRotated = ClassificationTestShapes.RotatedZ(aBox, Math.PI / 4);
        Assert.Equal(In, Classify(aRotated, 0, 7, 5));
        Assert.Equal(Out, Classify(aRotated, 7, 0, 5));
    }

    [Fact]
    public void Infinite_point()
    {
        var aClassifier = new global::Kerf.Engine.Classification.SolidClassifier(Box10());
        aClassifier.PerformInfinitePoint(1e-7);
        Assert.Equal(Out, aClassifier.State());

        var aHole = new global::Kerf.Engine.Classification.SolidClassifier(Box10().Reversed());
        aHole.PerformInfinitePoint(1e-7);
        Assert.Equal(In, aHole.State());
    }

    [Theory]
    [InlineData(0, 0, 1, In)]
    [InlineData(1.9, 0.3, 2.9, In)]
    [InlineData(3, 0, 1, Out)]
    [InlineData(1.5, 1.5, 1, Out)] // in the cylinder's box, outside the cylinder
    [InlineData(0, 0, 3.5, Out)]
    [InlineData(2, 0, 1, On)] // on the seam
    [InlineData(0, -2, 1, On)] // on the lateral face
    [InlineData(0.5, 0.5, 3, On)] // on the top disc
    [InlineData(0, 2, 0, On)] // on the bottom circle
    public void Cylinder_states(double x, double y, double z, Containment theExpected)
    {
        Shape aCylinder = TestShapes.Frozen(BRepGPropTestShapes.CylinderSolid(2.0, 3.0));
        Assert.Equal(theExpected, Classify(aCylinder, x, y, z));
    }

    [Fact]
    public void Sphere_states()
    {
        Shape aSphere = TestShapes.Frozen(BRepGPropTestShapes.SphereSolid(10.0));
        Assert.Equal(In, Classify(aSphere, 1, 2, 3));
        Assert.Equal(Out, Classify(aSphere, 8, 8, 8));
        Assert.Equal(On, Classify(aSphere, 0, 10, 0));
    }

    [Fact]
    public void Solid_with_a_void()
    {
        Shape aSolid = TestShapes.Frozen(ClassificationTestShapes.BoxWithVoid(out Shell anOuter, out _));
        Assert.Equal(In, Classify(aSolid, 1, 1, 1));
        Assert.Equal(Out, Classify(aSolid, 4, 4, 4)); // in the void
        Assert.Equal(On, Classify(aSolid, 3, 4, 4)); // on the void's boundary
        Assert.Equal(Out, Classify(aSolid, 11, 4, 4));

        // the outer shell is the one a point at infinity is outside of
        Shell aFound = global::Kerf.Engine.Classification.BRepClass3d.OuterShell(ShapeCasts.Solid(aSolid));
        Assert.True(aFound.IsSame(anOuter));
    }

    [Fact]
    public void OuterShell_of_simple_and_null_solids()
    {
        Solid aBox = ShapeCasts.Solid(Box10());
        Shell aShell = global::Kerf.Engine.Classification.BRepClass3d.OuterShell(aBox);
        Assert.False(aShell.IsNull);
        Assert.True(aShell.IsSame(new Iterator(aBox).Value()));
        Assert.True(global::Kerf.Engine.Classification.BRepClass3d.OuterShell(new Solid()).IsNull);
    }

    [Fact]
    public void Compound_of_boxes()
    {
        Shape aBox = Box10();
        Shape aCompound = TestShapes.Frozen(BRepGPropTestShapes.CompoundOf(aBox, ClassificationTestShapes.Translated(aBox, new Vector3D(20, 0, 0))));
        Assert.Equal(In, Classify(aCompound, 5, 5, 5));
        Assert.Equal(In, Classify(aCompound, 25, 5, 5));
        Assert.Equal(Out, Classify(aCompound, 15, 5, 5));
    }

    [Fact]
    public void Solid_without_faces_is_the_whole_space()
    {
        var aB = new BrepBuilder();
        Solid anEmpty = new();
        aB.MakeSolid(ref anEmpty);
        Shape aSolid = TestShapes.Frozen(anEmpty);
        var aClassifier = new global::Kerf.Engine.Classification.SolidClassifier(aSolid, new Point3D(1, 2, 3), 1e-7);
        Assert.Equal(In, aClassifier.State());
        aClassifier.PerformInfinitePoint(1e-7);
        Assert.Equal(In, aClassifier.State());
    }

    [Fact]
    public void Explorer_finds_a_point_in_a_face()
    {
        Face aFace = TestShapes.Frozen(ClassificationTestShapes.HoledSquare());
        Point3D aP = new();
        double u = 0, v = 0;
        Assert.True(global::Kerf.Engine.Classification.SolidExplorer.FindAPointInTheFace(aFace, ref aP, ref u, ref v));
        Assert.Equal(0.0, aP.Z(), 1e-12);
        Assert.Equal(u, aP.X(), 1e-12);
        Assert.Equal(v, aP.Y(), 1e-12);
        Assert.Equal(In, new global::Kerf.Engine.Classification.FaceClassifier(aFace, new Point2D(u, v), 1e-7).State());

        var anExplorer = new global::Kerf.Engine.Classification.SolidExplorer(Box10());
        Assert.False(anExplorer.Reject(new Point3D()));
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
        Line3D aL = new();
        double aPar = 0;
        Assert.Equal(0, anExplorer.Segment(new Point3D(5, 5, 5), ref aL, ref aPar));
        Assert.True(aPar > 0);
        Assert.Equal(1, anExplorer.Segment(new Point3D(5, 5, 0), ref aL, ref aPar)); // on the first face
    }

    [Fact]
    public void Passive_classifier_compares_faces_given_one_by_one()
    {
        Shape aBox = Box10();
        var aClassifier = new global::Kerf.Engine.Classification.SolidPassiveClassifier();
        aClassifier.Reset(new Line3D(new Point3D(5, 5, 5), new Direction3D(0, 0, 1)), RealLast(), 1e-7);
        foreach (Face aF in BRepGPropTestShapes.Faces(aBox))
        {
            aClassifier.Compare(aF, aF.Orientation());
        }

        Assert.Equal(In, aClassifier.State());
        Assert.Equal(5.0, aClassifier.Parameter(), 1e-9);

        var anInter = new global::Kerf.Engine.Classification.Intersector3d();
        anInter.Perform(new Line3D(new Point3D(5, 5, -5), new Direction3D(0, 0, 1)), RealLast(), 1e-7, BRepGPropTestShapes.Faces(aBox)[0]);
        Assert.True(anInter.IsDone);
        Assert.True(anInter.HasAPoint);
        Assert.Equal(5.0, anInter.WParameter(), 1e-9);
        Assert.Equal(In, anInter.State());
        Assert.Equal(TransitionOnCurve.In, anInter.Transition());
    }
}
