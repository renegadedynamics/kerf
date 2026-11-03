// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Preview;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.ModelingData.Tests;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Known answers of the primitives (BRepPrim through the BRepPrimAPI makers): topology counts, volumes and areas of
/// boxes, wedges, cylinders, cones, spheres, tori (whole and partial), revolutions and half-spaces; the preview box;
/// errors; and publication (every result frozen, a face handed out complete, the caller's face shared by a half-space).
/// </summary>
public class BRepPrimAPIPrimitivesTests
{
    private static int Count(in Shape theS, ShapeType theType)
    {
        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theS, theType, aMap);
        return aMap.Count;
    }

    private static (int, int, int, int) Counts(in Shape theS) =>
        (Count(theS, ShapeType.Shell), Count(theS, ShapeType.Face), Count(theS, ShapeType.Edge), Count(theS, ShapeType.Vertex));

    private static double Volume(in Shape theS)
    {
        Properties aProps = new();
        MassPropertiesBuilder.VolumeProperties(theS, aProps);
        return aProps.Mass();
    }

    private static double Area(in Shape theS)
    {
        Properties aProps = new();
        MassPropertiesBuilder.SurfaceProperties(theS, aProps);
        return aProps.Mass();
    }

    private static bool AllFrozen(in Shape theS)
    {
        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theS, aMap);
        for (int i = 0; i < aMap.Count; i++)
        {
            if (!aMap[i].TShape()!.IsFrozen)
            {
                return false;
            }
        }
        return theS.TShape()!.IsFrozen;
    }

    private static bool IsValid(in Shape theS) => new Analyzer(theS).IsValid();

    // ------------------------------------------------------------ boxes

    [Fact]
    public void Box_forms_give_the_same_box_with_negative_sizes_measured_from_the_corner()
    {
        var aP = new Point3D(1, 2, 3);
        Shape[] aBoxes =
        [
            new MakeBox(new Point3D(-1, -1, 0), 2, 3, 4).Shape(),
            new MakeBox(new Point3D(1, 2, 4), -2, 3 - 6, -4).Shape(),
            new MakeBox(new Point3D(1, -1, 4), new Point3D(-1, 2, 0)).Shape(),
            new MakeBox(new Frame3D(new Point3D(-1, -1, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 2, 3, 4).Shape(),
        ];
        foreach (Shape aBox in aBoxes)
        {
            Assert.Equal(ShapeType.Solid, aBox.ShapeType());
            Assert.Equal((1, 6, 12, 8), Counts(aBox));
            Assert.Equal(24.0, Volume(aBox), 1e-9);
            Assert.Equal(2 * (6 + 8 + 12), Area(aBox), 1e-9);
            Assert.True(IsValid(aBox));
            Assert.True(AllFrozen(aBox));
            var aBnd = new Box();
            Bounds.Add(aBox, ref aBnd);
            aBnd.Get(out double x0, out double y0, out double z0, out double x1, out double y1, out double z1);
            Assert.Equal(-1.0, x0, 1e-6);
            Assert.Equal(-1.0, y0, 1e-6);
            Assert.Equal(0.0, z0, 1e-6);
            Assert.Equal(1.0, x1, 1e-6);
            Assert.Equal(2.0, y1, 1e-6);
            Assert.Equal(4.0, z1, 1e-6);
        }

        // Init replaces the box
        var aMaker = new MakeBox(1, 1, 1);
        aMaker.Init(aP, 2, 2, 2);
        Assert.Equal(8.0, Volume(aMaker.Shape()), 1e-9);
    }

    [Fact]
    public void A_box_face_is_handed_out_complete_frozen_and_shared_with_the_solid()
    {
        var aMaker = new MakeBox(new Frame3D(new Point3D(1, 2, 3), new Direction3D(1, 1, 1)), 2, 3, 4);
        Face aBottom = aMaker.BottomFace();
        Assert.True(AllFrozen(aBottom));
        // each edge of the face already has its pcurves on both faces it bounds
        for (Explorer anExp = new(aBottom, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            int aNbPCurves = 0;
            foreach (CurveRepresentation aRep in ((BrepTEdge)anExp.Current().TShape()!).Curves())
            {
                aNbPCurves += aRep.IsCurveOnSurface() ? 1 : 0;
            }
            Assert.Equal(2, aNbPCurves);
        }

        Shape aSolid = aMaker.Shape();
        Face[] aFaces = [aBottom, aMaker.TopFace(), aMaker.BackFace(), aMaker.FrontFace(), aMaker.LeftFace(), aMaker.RightFace()];
        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(aSolid, ShapeType.Face, aMap);
        foreach (Face aF in aFaces)
        {
            Assert.True(aMap.Contains(aF));
        }

        // Solid() makes a new solid each time around the same shell, as OCCT
        Solid aSolid2 = aMaker.Solid();
        Assert.False(aSolid2.IsSame(aSolid));
        Assert.True(ShapeCasts.Shell(new Iterator(aSolid2).Value()).IsSame(new Iterator(aSolid).Value()));

        // Shell() stores the shell as the result
        Shell aShell = aMaker.Shell();
        Assert.True(aMaker.Shape().IsSame(aShell));
        Assert.Equal(ShapeType.Shell, aMaker.Shape().ShapeType());
    }

    [Fact]
    public void A_flat_box_has_no_shell_but_its_faces()
    {
        var aMaker = new MakeBox(2, 3, 0);
        Assert.ThrowsAny<DomainErrorException>(() => aMaker.Shape());
        Face aBottom = aMaker.BottomFace();
        Assert.True(AllFrozen(aBottom));
        Assert.Equal(6.0, Area(aBottom), 1e-9);
        Face aTop = aMaker.TopFace();
        Assert.Equal(6.0, Area(aTop), 1e-9);
    }

    // ------------------------------------------------------------ wedges

    // A wedge is a prismatoid between the rectangles [0, dx] x [0, dz] at y = 0 and [xmin, xmax] x [zmin, zmax] at y = dy.
    private static double WedgeVolume(double dx, double dy, double dz, double xmin, double zmin, double xmax, double zmax)
    {
        double A1 = dx * dz, A2 = (xmax - xmin) * (zmax - zmin);
        double Am = 0.5 * (dx + xmax - xmin) * 0.5 * (dz + zmax - zmin);
        return dy / 6 * (A1 + A2 + 4 * Am);
    }

    [Theory]
    [InlineData(10, 10, 10, 2, 3, 8, 7, 6, 12, 8)]  // a frustum-like wedge
    [InlineData(10, 10, 10, 0, 0, 10, 10, 6, 12, 8)] // a box
    [InlineData(10, 5, 8, 3, 2, 3, 6, 5, 9, 6)]      // a top segment along z: a 5-face wedge
    [InlineData(10, 5, 8, 2, 4, 9, 4, 5, 9, 6)]      // a top segment along x
    [InlineData(10, 5, 8, 4, 3, 4, 3, 5, 8, 5)]      // a top point: a pyramid
    public void Wedges_are_prismatoids(double dx, double dy, double dz, double xmin, double zmin, double xmax, double zmax,
                                       int theNbFaces, int theNbEdges, int theNbVertices)
    {
        Shape aWedge = new MakeWedge(dx, dy, dz, xmin, zmin, xmax, zmax).Shape();
        Assert.Equal((1, theNbFaces, theNbEdges, theNbVertices), Counts(aWedge));
        Assert.Equal(WedgeVolume(dx, dy, dz, xmin, zmin, xmax, zmax), Volume(aWedge), 1e-9);
        Assert.True(IsValid(aWedge));
        Assert.True(AllFrozen(aWedge));
    }

    [Fact]
    public void A_right_angular_wedge_has_ltx_at_the_top()
    {
        var anAx = new Frame3D(new Point3D(5, 0, 0), new Direction3D(0, 1, 0), new Direction3D(0, 0, 1));
        var aMaker = new MakeWedge(anAx, 4, 3, 2, 1);
        Assert.Equal(WedgeVolume(4, 3, 2, 0, 0, 1, 2), Volume(aMaker.Solid()), 1e-9);
        Assert.Equal((1, 6, 12, 8), Counts(aMaker.Shell()));
        Shape aPrism = new MakeWedge(4, 3, 2, 0).Shape();
        Assert.Equal((1, 5, 9, 6), Counts(aPrism));
        Assert.Equal(4 * 3 / 2.0 * 2, Volume(aPrism), 1e-9);
    }

    // ------------------------------------------------------------ one-axis primitives

    [Theory]
    [InlineData(2.0 * Math.PI, 3, 3, 2)]
    [InlineData(Math.PI, 5, 9, 6)]
    [InlineData(0.5, 5, 9, 6)]
    public void Cylinders_whole_and_partial(double theAngle, int theNbFaces, int theNbEdges, int theNbVertices)
    {
        const double R = 2, H = 5;
        var anAx = new Frame3D(new Point3D(1, 2, 3), new Direction3D(1, 2, 3));
        var aMaker = new ApiMakeCylinder(anAx, R, H, theAngle);
        Shape aCyl = aMaker.Shape();
        Assert.Equal((1, theNbFaces, theNbEdges, theNbVertices), Counts(aCyl));
        Assert.Equal(R * R * H * theAngle / 2, Volume(aCyl), 1e-6);
        Assert.Equal(R * theAngle * H, Area(aMaker.Face()), 1e-6);
        Assert.True(IsValid(aCyl));
        Assert.True(AllFrozen(aCyl));
        Assert.IsType<CylindricalSurface>(Tool.Surface(aMaker.Face()));
    }

    [Theory]
    [InlineData(3.0, 1.0, 2.0 * Math.PI, 3)]
    [InlineData(3.0, 0.0, 2.0 * Math.PI, 2)]
    [InlineData(0.0, 2.0, 2.0 * Math.PI, 2)]
    [InlineData(3.0, 1.0, 1.0, 5)]
    [InlineData(3.0, 0.0, 1.0, 4)]
    public void Cones_whole_partial_and_with_an_apex(double R1, double R2, double theAngle, int theNbFaces)
    {
        const double H = 4;
        Shape aCone = new ApiMakeCone(R1, R2, H, theAngle).Shape();
        Assert.Equal(theNbFaces, Count(aCone, ShapeType.Face));
        Assert.Equal(H / 3 * (R1 * R1 + R1 * R2 + R2 * R2) * theAngle / 2, Volume(aCone), 1e-6);
        Assert.True(IsValid(aCone));
    }

    [Fact]
    public void Cones_with_equal_radii_or_no_height_are_errors()
    {
        Assert.ThrowsAny<DomainErrorException>(() => new ApiMakeCone(2, 2, 3));
        Assert.ThrowsAny<DomainErrorException>(() => new ApiMakeCone(2, 1, 0));
        Assert.ThrowsAny<DomainErrorException>(() => new ApiMakeCone(-1, 1, 3));
    }

    // The volume of a sphere's zone between the latitudes a1 < a2, over an angle around the axis.
    private static double ZoneVolume(double R, double a1, double a2, double theAngle) =>
        Math.PI * R * R * R * ((Math.Sin(a2) - Math.Sin(a1)) - (Math.Pow(Math.Sin(a2), 3) - Math.Pow(Math.Sin(a1), 3)) / 3)
        * theAngle / (2 * Math.PI);

    [Theory]
    [InlineData(-Math.PI / 2, Math.PI / 2, 2.0 * Math.PI, 1)]
    [InlineData(-Math.PI / 2, Math.PI / 2, 1.5, 3)]
    [InlineData(-0.5, 0.7, 2.0 * Math.PI, 3)]
    [InlineData(0.0, Math.PI / 2, 2.0 * Math.PI, 2)]
    [InlineData(-0.5, 0.7, 2.5, 5)]
    public void Spheres_whole_zones_and_wedges(double a1, double a2, double theAngle, int theNbFaces)
    {
        const double R = 3;
        Shape aSphere = new MakeSphere(new Point3D(1, 1, 1), R, a1, a2, theAngle).Shape();
        Assert.Equal(theNbFaces, Count(aSphere, ShapeType.Face));
        Assert.Equal(ZoneVolume(R, a1, a2, theAngle), Volume(aSphere), 1e-6);
        Assert.True(IsValid(aSphere));
    }

    [Fact]
    public void A_negative_sphere_wedge_angle_turns_the_axis_over()
    {
        var aMaker = new MakeSphere(2, -Math.PI / 2);
        Face aFace = aMaker.Face();
        var aSurf = (SphericalSurface)Tool.Surface(aFace)!;
        Assert.Equal(-1.0, aSurf.Position().Direction().Z());
        Assert.Equal(ZoneVolume(2, -Math.PI / 2, Math.PI / 2, Math.PI / 2), Volume(aMaker.Shape()), 1e-6);
    }

    [Theory]
    [InlineData(2.0 * Math.PI, 1)]
    [InlineData(Math.PI, 3)]
    [InlineData(1.0, 3)]
    public void Tori_whole_and_partial(double theAngle, int theNbFaces)
    {
        const double R1 = 5, R2 = 1.5;
        var aMaker = new MakeTorus(new Frame3D(new Point3D(0, 0, 1), new Direction3D(0, 1, 0)), R1, R2, theAngle);
        Shape aTorus = aMaker.Shape();
        Assert.Equal(theNbFaces, Count(aTorus, ShapeType.Face));
        // Pappus: the disk's area times the path of its centre
        Assert.Equal(Math.PI * R2 * R2 * R1 * theAngle, Volume(aTorus), 1e-6);
        Assert.Equal(2 * Math.PI * R2 * R1 * theAngle, Area(aMaker.Face()), 1e-6);
        Assert.True(IsValid(aTorus));
    }

    [Fact]
    public void A_torus_ring_segment_is_a_revolved_circular_segment()
    {
        // V in [-pi/2, pi/2]: the outer half pipe, closed by the disks of radius R1 at z = -R2 and z = R2
        const double R1 = 5, R2 = 1.5;
        Shape aTorus = new MakeTorus(R1, R2, -Math.PI / 2, Math.PI / 2).Shape();
        Assert.Equal(3, Count(aTorus, ShapeType.Face));
        // the cylinder inside, and a half disk of area pi r^2 / 2 with its centroid 4r/(3pi) outside R1 revolved
        double aCentroid = R1 + 4 * R2 / (3 * Math.PI);
        Assert.Equal(Math.PI * R1 * R1 * 2 * R2 + Math.PI * R2 * R2 / 2 * 2 * Math.PI * aCentroid, Volume(aTorus), 1e-6);
        Assert.True(IsValid(aTorus));
    }

    [Fact]
    public void A_revolution_of_a_line_is_a_cylinder_and_of_an_arc_a_sphere()
    {
        // a line parallel to Z at x = 2 between z = 0 and z = 3
        var aLine = new TrimmedCurve3D(new LineCurve3D(new Point3D(2, 0, 0), new Direction3D(0, 0, 1)), 0, 3);
        var aCyl = new MakeRevolution(aLine);
        Assert.Equal(Math.PI * 4 * 3, Volume(aCyl.Shape()), 1e-6);
        Assert.Equal(3, Count(aCyl.Shape(), ShapeType.Face));
        Assert.IsType<RevolutionSurface>(Tool.Surface(aCyl.Face()));

        // a half circle in the XZ plane from the south pole to the north pole, over a quarter turn
        var anArc = new TrimmedCurve3D(new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, -1, 0), new Direction3D(1, 0, 0)), 2),
                                          -Math.PI / 2, Math.PI / 2);
        var aSph = new MakeRevolution(anArc, Math.PI / 2);
        Assert.Equal(4.0 / 3 * Math.PI * 8 / 4, Volume(aSph.Shape()), 1e-6);
        Assert.True(IsValid(aSph.Shape()));
    }

    [Fact]
    public void A_one_axis_face_is_complete_frozen_and_in_the_solid()
    {
        var aMaker = new ApiMakeCone(new Frame3D(), 2, 1, 3, 2.0);
        Face aFace = aMaker.Face();
        Assert.True(AllFrozen(aFace));
        Shell aShell = aMaker.Shell();
        Solid aSolid = aMaker.Solid();
        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(aSolid, aMap);
        Assert.True(aMap.Contains(aFace));
        Assert.True(aMap.Contains(aShell));
        Assert.True(AllFrozen(aSolid));
        Assert.True(((Shape)aShell).Closed());
    }

    // ------------------------------------------------------------ half-spaces

    [Fact]
    public void A_half_space_holds_the_callers_face_on_the_side_of_the_point()
    {
        Face aFace = new MakeFace(new Plane3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), -1, 1, -1, 1).Face();
        var aSnapshot = ShapeSnapshot.Of(aFace);

        var anAbove = new MakeHalfSpace(aFace, new Point3D(0.2, 0.3, 5));
        var aBelow = new MakeHalfSpace(aFace, new Point3D(0.2, 0.3, -5));
        Assert.True(anAbove.IsDone);
        Assert.True(aBelow.IsDone);
        Solid aSolidAbove = anAbove.Solid(), aSolidBelow = aBelow.Solid();
        Assert.True(AllFrozen(aSolidAbove));
        Assert.Equal((1, 1, 4, 4), Counts(aSolidAbove));

        // the face itself, its shell turned toward the matter
        Shape aShellAbove = new Iterator(aSolidAbove).Value(), aShellBelow = new Iterator(aSolidBelow).Value();
        Assert.True(new Iterator(aShellAbove).Value().IsSame(aFace));
        Assert.NotEqual(aShellAbove.Orientation(), aShellBelow.Orientation());
        aSnapshot.AssertUnchanged(aFace);
    }

    [Fact]
    public void A_half_space_on_a_box_shell_keeps_the_shell()
    {
        Shell aShell = new MakeBox(2, 2, 2).Shell();
        var anInside = new MakeHalfSpace(aShell, new Point3D(1, 1, 1));
        var anOutside = new MakeHalfSpace(aShell, new Point3D(5, 1, 1));
        Shape aShellIn = new Iterator(anInside.Solid()).Value();
        Shape aShellOut = new Iterator(anOutside.Shape()).Value();
        Assert.True(aShellIn.IsSame(aShell));
        Assert.True(aShellOut.IsSame(aShell));
        Assert.Equal(aShell.Orientation(), aShellIn.Orientation());
        Assert.NotEqual(aShell.Orientation(), aShellOut.Orientation());
    }

    // ------------------------------------------------------------ preview

    [Fact]
    public void The_preview_box_is_a_vertex_an_edge_a_face_or_a_solid()
    {
        Shape Preview(Point3D theP1, Point3D theP2)
        {
            var aMaker = new MakePreviewBox();
            aMaker.Init(theP1, theP2);
            Shape aShape = aMaker.Shape();
            Assert.True(AllFrozen(aShape));
            return aShape;
        }

        Assert.Equal(ShapeType.Vertex, Preview(new Point3D(1, 2, 3), new Point3D(1, 2, 3)).ShapeType());
        Shape anEdge = Preview(new Point3D(1, 2, 3), new Point3D(1, 2, 7));
        Assert.Equal(ShapeType.Edge, anEdge.ShapeType());
        Properties aProps = new();
        MassPropertiesBuilder.LinearProperties(anEdge, aProps);
        Assert.Equal(4.0, aProps.Mass(), 1e-9);
        Shape aFace = Preview(new Point3D(1, 2, 3), new Point3D(4, 2, 7));
        Assert.Equal(ShapeType.Face, aFace.ShapeType());
        Assert.Equal(12.0, Area(aFace), 1e-9);
        Shape aSolid = Preview(new Point3D(1, 2, 3), new Point3D(4, 3, 7));
        Assert.Equal(ShapeType.Solid, aSolid.ShapeType());
        Assert.Equal(12.0, Volume(aSolid), 1e-9);
    }
}
