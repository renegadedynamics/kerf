// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Offsets.Blend;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// The rolling ball (BiTgte_Blend) by what it must make (unit 12Q, decision 1753): a ball of the radius rolled inside a
/// solid rounds its edges with tubes and its corners with spheres, whose volumes are known; each tube touches its two
/// faces a radius from the edge, and a ball outside a convex edge touches nothing.
/// </summary>
public class BiTgte_LawsTests
{
    private static List<Face> Faces(in Shape theShape) =>
        BRepOffset_MakeOffsetTests.SubShapes(theShape, ShapeType.Face).Select(f => ShapeCasts.Face(f)).ToList();

    // A negative radius rolls the ball inside the solid, as OCCT's rollingball tests do
    private static OffsetBlend Rolled(in Shape theShape, int theFace1, int theFace2, double theRadius, bool theBuild = true)
    {
        List<Face> aFaces = Faces(theShape);
        var aRoll = new OffsetBlend(theShape, theRadius, 1.0e-4, false);
        aRoll.SetFaces(aFaces[theFace1], aFaces[theFace2]);
        aRoll.Perform(theBuild);
        Assert.True(aRoll.IsDone);
        return aRoll;
    }

    private static Point3D Mid(Curve3D theCurve) => theCurve.Value(0.5 * (theCurve.FirstParameter + theCurve.LastParameter));

    [Theory]
    [InlineData(2.0)]
    [InlineData(1.0)]
    public void A_ball_inside_a_box_rounds_its_edges_and_corners(double theRadius)
    {
        Shape aBox = new MakeBox(10, 10, 10).Shape();
        OffsetBlend aRoll = Rolled(aBox, 0, 2, -theRadius);
        // twelve tubes and eight corner spheres, all one branch
        Assert.Equal(20, aRoll.SurfaceCount);
        Assert.Equal(1, aRoll.NbBranches());
        int aSpheres = 0;
        for (int i = 1; i <= aRoll.SurfaceCount; i++)
        {
            if (aRoll.Surface(i) is SphericalSurface aSphere)
            {
                aSpheres++;
                Assert.Equal(theRadius, aSphere.Radius, 1e-9);
                // centred a radius in from a corner along each axis
                Point3D aC = aSphere.Position.Location;
                foreach (double aCoord in new[] { aC.X, aC.Y, aC.Z })
                {
                    Assert.True(Math.Abs(aCoord - theRadius) < 1e-9 || Math.Abs(aCoord - (10 - theRadius)) < 1e-9, aC.ToString());
                }
            }
        }
        Assert.Equal(8, aSpheres);
        Shape aResult = aRoll.Shape();
        Assert.True(new Analyzer(aResult).IsValid());
        Assert.Equal(26, Faces(aResult).Count);
        // the rounded box: its core, six slabs, the quarter cylinders along the edges and the eighth spheres at the corners
        double r = theRadius, a = 10 - (2 * r);
        double aVolume = (a * a * a) + (6 * r * a * a) + (Math.PI * r * r * 3 * a) + (4.0 / 3 * Math.PI * r * r * r);
        Assert.Equal(aVolume, BRepOffset_MakeOffsetTests.Volume(aResult), 5); // the volume integrates the tubes to about 1e-9 of it
    }

    [Fact]
    public void Each_tube_of_a_box_touches_its_two_faces_a_radius_from_their_edge()
    {
        Shape aBox = new MakeBox(10, 10, 10).Shape();
        List<Face> aFaces = Faces(aBox);
        OffsetBlend aRoll = Rolled(aBox, 0, 2, -2);
        int aTubes = 0;
        for (int i = 1; i <= aRoll.SurfaceCount; i++)
        {
            if (aRoll.Surface(i) is SphericalSurface)
            {
                // a corner's centre is a vertex, not a line, and has no faces to name (OCCT's TopoDS::Edge)
                EngineAssert.Throws<TypeMismatchException>(() => aRoll.SupportShape1(i));
                continue;
            }
            aTubes++;
            // the supports are the box's own faces, and the contacts are on them
            Assert.Equal(ContactType.FaceEdge, aRoll.ContactType(i)); // OCCT never says FaceFace
            Face aS1 = ShapeCasts.Face(aRoll.SupportShape1(i)), aS2 = ShapeCasts.Face(aRoll.SupportShape2(i));
            Assert.Contains(aFaces, f => f.IsSame(aS1));
            Assert.Contains(aFaces, f => f.IsSame(aS2));
            Assert.False(aS1.IsSame(aS2));
            foreach ((Face aSupport, Curve3D? aContact) in new[] { (aS1, aRoll.CurveOnShape1(i)), (aS2, aRoll.CurveOnShape2(i)) })
            {
                Plane3D aPlane = ((PlaneSurface)Tool.Surface(aSupport)!).Plane;
                Point3D aP = Mid(aContact!);
                Assert.True(aPlane.DistanceTo(aP) < 1e-7, $"{aP} off its face");
                // two coordinates a radius in from the box's faces, as the edge's are on them
                int anIn = new[] { aP.X, aP.Y, aP.Z }.Count(c => Math.Abs(c - 2) < 1e-7 || Math.Abs(c - 8) < 1e-7);
                Assert.True(anIn >= 1, aP.ToString());
            }
            Assert.Null(aRoll.PCurveOnFace1(i));
        }
        Assert.Equal(12, aTubes);
    }

    [Fact]
    public void A_ball_outside_a_convex_box_touches_nothing()
    {
        Shape aBox = new MakeBox(10, 10, 10).Shape();
        OffsetBlend aRoll = Rolled(aBox, 0, 2, 2);
        Assert.Equal(0, aRoll.SurfaceCount);
        Assert.Equal(0, aRoll.NbBranches());
        Assert.Equal(1000, BRepOffset_MakeOffsetTests.Volume(aRoll.Shape()), 6);
    }

    [Fact]
    public void Without_the_shape_the_same_surfaces_are_made()
    {
        Shape aBox = new MakeBox(10, 10, 10).Shape();
        OffsetBlend aFull = Rolled(aBox, 0, 2, -2), aSurfaces = Rolled(aBox, 0, 2, -2, false);
        Assert.Equal(aFull.SurfaceCount, aSurfaces.SurfaceCount);
        for (int i = 1; i <= aSurfaces.SurfaceCount; i++)
        {
            Assert.Equal(aFull.Surface(i)!.GetType(), aSurfaces.Surface(i)!.GetType());
        }
    }

    [Fact]
    public void A_ball_round_the_foot_of_a_cylinder_takes_a_ring_off_it()
    {
        Shape aCylinder = new ApiMakeCylinder(5, 10).Shape();
        OffsetBlend aRoll = Rolled(aCylinder, 0, 2, -1);
        Assert.Equal(1, aRoll.SurfaceCount);
        // the contacts: on the base a radius in, and on the side a radius up
        Point3D aOn1 = Mid(aRoll.CurveOnShape1(1)!), aOn2 = Mid(aRoll.CurveOnShape2(1)!);
        Assert.Equal(0, aOn1.Z, 1e-7);
        Assert.Equal(4, Math.Sqrt((aOn1.X * aOn1.X) + (aOn1.Y * aOn1.Y)), 1e-7);
        Assert.Equal(1, aOn2.Z, 1e-7);
        Assert.Equal(5, Math.Sqrt((aOn2.X * aOn2.X) + (aOn2.Y * aOn2.Y)), 1e-7);
        Shape aResult = aRoll.Shape();
        Assert.True(new Analyzer(aResult).IsValid());
        // Pappus: the corner between a square of the radius and its quarter circle, turned round the axis at its centroid
        double r = 1, anArea = r * r * (1 - (Math.PI / 4)), aCentroid = 5 - (r * (10 - (3 * Math.PI)) / (3 * (4 - Math.PI)));
        Assert.Equal((Math.PI * 25 * 10) - (2 * Math.PI * aCentroid * anArea), BRepOffset_MakeOffsetTests.Volume(aResult), 6);
    }

    [Fact]
    public void A_ball_round_the_top_of_a_cone_touches_the_top_and_the_cone()
    {
        // radius 5 at the foot and 2 at the top, 10 high: its radius at z is 5 - 0.3 z
        Shape aCone = new ApiMakeCone(5, 2, 10).Shape();
        OffsetBlend aRoll = Rolled(aCone, 0, 1, -1);
        Assert.Equal(1, aRoll.SurfaceCount);
        Point3D aOnTop = Mid(aRoll.CurveOnShape1(1)!), aOnSide = Mid(aRoll.CurveOnShape2(1)!);
        Assert.Equal(10, aOnTop.Z, 1e-7);
        Assert.True(Math.Sqrt((aOnTop.X * aOnTop.X) + (aOnTop.Y * aOnTop.Y)) < 2);
        Assert.Equal(5 - (0.3 * aOnSide.Z), Math.Sqrt((aOnSide.X * aOnSide.X) + (aOnSide.Y * aOnSide.Y)), 1e-6);
        Assert.True(aOnSide.Z < 10);
        // the ball's centre is a radius under the top contact, and the side contact is a radius from it
        Point3D aCentre = new(aOnTop.X, aOnTop.Y, 9);
        Assert.Equal(1, aCentre.DistanceTo(aOnSide), 1e-6);
        Shape aResult = aRoll.Shape();
        Assert.True(new Analyzer(aResult).IsValid());
        Assert.True(BRepOffset_MakeOffsetTests.Volume(aResult) < Math.PI * 10 / 3 * (25 + 10 + 4));
    }

    // The corner between two lines meeting at the interior angle theTheta at (theRho, theZ) in a half-plane, less the
    // disc of radius r touching both: its area and the rho of its centroid, for Pappus. The first line leaves the vertex
    // at the angle theAngle; the second is turned theTheta from it, clockwise for theTurn -1.
    private static (double Area, double Rho) Corner(double theRho, double theZ, double theAngle, double theTheta, int theTurn, double r)
    {
        double aSecond = theAngle + (theTurn * theTheta), aHalf = theAngle + (theTurn * theTheta / 2);
        double ux = Math.Cos(theAngle), uy = Math.Sin(theAngle), wx = Math.Cos(aSecond), wy = Math.Sin(aSecond);
        double bx = Math.Cos(aHalf), by = Math.Sin(aHalf);
        double aTangent = r / Math.Tan(theTheta / 2), aToCentre = r / Math.Sin(theTheta / 2);
        (double X, double Y) aT1 = (theRho + (ux * aTangent), theZ + (uy * aTangent));
        (double X, double Y) aT2 = (theRho + (wx * aTangent), theZ + (wy * aTangent));
        (double X, double Y) aC = (theRho + (bx * aToCentre), theZ + (by * aToCentre));
        // the kite of the vertex, the two tangent points and the centre: two equal triangles
        double aKite = r * aTangent, aKiteRho = ((2 * theRho) + aT1.X + aT2.X + (2 * aC.X)) / 6;
        // the sector of the disc inside the kite, of angle pi - theta, whose centroid is towards the vertex
        double aPhi = Math.PI - theTheta, aSector = 0.5 * r * r * aPhi;
        double aSectorRho = aC.X - (bx * 4 * r * Math.Sin(aPhi / 2) / (3 * aPhi));
        double anArea = aKite - aSector;
        return (anArea, ((aKiteRho * aKite) - (aSectorRho * aSector)) / anArea);
    }

    [Fact]
    public void The_corner_law_gives_the_cylinder_ring_as_its_quarter_circle_does()
    {
        // the cylinder's foot: the vertex at rho 5, the base running in (angle pi), the side turned a right angle up
        (double anArea, double aRho) = Corner(5, 0, Math.PI, Math.PI / 2, -1, 1);
        Assert.Equal(1 - (Math.PI / 4), anArea, 12);
        Assert.Equal(5 - ((10 - (3 * Math.PI)) / (3 * (4 - Math.PI))), aRho, 12);
    }

    [Fact]
    public void A_ball_round_the_foot_of_a_cone_takes_a_ring_off_it()
    {
        // radius 5 at the foot, 2 at the top, 10 high: the side leaves the foot at (5, 0) inwards and up, at atan(10/3)
        Shape aCone = new ApiMakeCone(5, 2, 10).Shape();
        OffsetBlend aRoll = Rolled(aCone, 0, 2, -1);
        Assert.Equal(1, aRoll.SurfaceCount);
        Assert.IsType<ToroidalSurface>(((TrimmedSurface)aRoll.Surface(1)!).BasisSurface);
        Shape aResult = aRoll.Shape();
        Assert.True(new Analyzer(aResult).IsValid());
        double aTheta = Math.Atan2(10, 3);
        (double anArea, double aRho) = Corner(5, 0, Math.PI, aTheta, -1, 1);
        double aFull = Math.PI * 10 / 3 * (25 + 10 + 4);
        Assert.Equal(aFull - (2 * Math.PI * aRho * anArea), BRepOffset_MakeOffsetTests.Volume(aResult), 6);
    }

    [Fact]
    public void A_ball_round_an_elliptic_prism_takes_its_corner_along_the_centroid_path()
    {
        // an ellipse of 5 by 3, 10 high: its tube is approximated, as its centre line is no circle
        Edge anEllipse = new MakeEdge(new Ellipse3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 5, 3)).Edge();
        Face aBase = new MakeFace(new MakeWire(anEllipse).Wire(), true).Face();
        Shape aPrism = new ApiMakePrism(aBase, new Vector3D(0, 0, 10)).Shape();
        OffsetBlend aRoll = Rolled(aPrism, 0, 1, -1);
        Assert.Equal(1, aRoll.SurfaceCount);
        Assert.IsType<BSplineSurface>(aRoll.Surface(1));
        Shape aResult = aRoll.Shape();
        Assert.True(new Analyzer(aResult).IsValid());
        // Pappus along a plane curve: the corner's area times the length of its centroid's path, the ellipse brought in by
        // the centroid's distance d, which is L - 2 pi d for a convex curve curving less than 1 / d
        (double anArea, double aRho) = Corner(5, 0, Math.PI, Math.PI / 2, -1, 1);
        double d = 5 - aRho, aLength = 0;
        const int n = 200000;
        for (int i = 0; i < n; i++)
        {
            double t = (i + 0.5) * 2 * Math.PI / n;
            aLength += Math.Sqrt((25 * Math.Sin(t) * Math.Sin(t)) + (9 * Math.Cos(t) * Math.Cos(t))) * 2 * Math.PI / n;
        }
        double aRemoved = anArea * (aLength - (2 * Math.PI * d));
        // the tube is approximated to 1e-4 over about 38 of area: its volume is good to about 4e-3
        Assert.Equal((Math.PI * 15 * 10) - aRemoved, BRepOffset_MakeOffsetTests.Volume(aResult), 2);
        Assert.True(Math.Abs((Math.PI * 15 * 10) - aRemoved - BRepOffset_MakeOffsetTests.Volume(aResult)) < 4e-3);
    }

    // A box of 10 with its four upright edges rounded at 2, and the indices of its top and of its side at y = 0
    private static (Shape Shape, int Top, int Side) RoundedBox()
    {
        Shape aBox = new MakeBox(10, 10, 10).Shape();
        var aFillet = new global::Kerf.Engine.Fillets.Api.MakeFillet(aBox);
        foreach (Shape anE in BRepOffset_MakeOffsetTests.SubShapes(aBox, ShapeType.Edge))
        {
            Edge anEdge = ShapeCasts.Edge(anE);
            Exploration.Vertices(anEdge, out Vertex aV1, out Vertex aV2);
            if (Tool.Pnt(aV1).Z != Tool.Pnt(aV2).Z)
            {
                aFillet.Add(2, anEdge);
            }
        }
        Shape aRounded = aFillet.Shape();
        List<Face> aFaces = Faces(aRounded);
        int Plane(Func<Point3D, bool> theOn) => aFaces.FindIndex(f =>
            Tool.Surface(f) is PlaneSurface && BRepOffset_MakeOffsetTests.SubShapes(f, ShapeType.Vertex).All(v => theOn(Tool.Pnt(ShapeCasts.Vertex(v)))));
        return (aRounded, Plane(p => Math.Abs(p.Z - 10) < 1e-9), Plane(p => Math.Abs(p.Y) < 1e-9));
    }

    [Fact]
    public void A_ball_round_the_top_of_a_rounded_box_follows_its_lines_and_arcs()
    {
        (Shape aRounded, int aTop, int aSide) = RoundedBox();
        // the rounded box: 1000 less the four corners outside the upright fillets
        Assert.Equal(1000 - (4 * (4 - Math.PI) * 10), BRepOffset_MakeOffsetTests.Volume(aRounded), 9);
        OffsetBlend aRoll = Rolled(aRounded, aTop, aSide, -1);
        // the top's outline: four lines with a cylinder each and four arcs with a torus each
        Assert.Equal(8, aRoll.SurfaceCount);
        Assert.Equal(4, Enumerable.Range(1, 8).Count(i => ((TrimmedSurface)aRoll.Surface(i)!).BasisSurface is ToroidalSurface));
        Assert.Equal(4, Enumerable.Range(1, 8).Count(i => ((TrimmedSurface)aRoll.Surface(i)!).BasisSurface is CylindricalSurface));
        Assert.Equal(1, aRoll.NbBranches());
        Shape aResult = aRoll.Shape();
        Assert.True(new Analyzer(aResult).IsValid());
        // Pappus along the outline: the corner times the outline brought in by its centroid's distance
        (double anArea, double aRho) = Corner(5, 0, Math.PI, Math.PI / 2, -1, 1);
        double aLength = (4 * 6) + (2 * Math.PI * 2) - (2 * Math.PI * (5 - aRho));
        Assert.Equal(1000 - (4 * (4 - Math.PI) * 10) - (anArea * aLength), BRepOffset_MakeOffsetTests.Volume(aResult), 6);
    }

    [Fact]
    public void A_ball_inside_a_box_with_a_wider_fillet_rounds_the_rest_and_turns_round_the_fillet_ends()
    {
        // one edge of the box rounded at 3: the ball of 1 rolls along the other eleven, rounds the six corners left, and
        // turns round the fillet's two end arcs with tori whose tube is the ball and whose ring is 3 less 1
        Shape aBox = new MakeBox(10, 10, 10).Shape();
        var aFillet = new global::Kerf.Engine.Fillets.Api.MakeFillet(aBox);
        aFillet.Add(3, ShapeCasts.Edge(BRepOffset_MakeOffsetTests.SubShapes(aBox, ShapeType.Edge)[0]));
        Shape aOne = aFillet.Shape();
        Assert.Equal(1000 - ((9 - (9 * Math.PI / 4)) * 10), BRepOffset_MakeOffsetTests.Volume(aOne), 9);
        List<Face> aFaces = Faces(aOne);
        // two sides that meet at a sharp edge (the ball's start)
        Assert.IsType<PlaneSurface>(Tool.Surface(aFaces[1]));
        Assert.IsType<PlaneSurface>(Tool.Surface(aFaces[2]));
        OffsetBlend aRoll = Rolled(aOne, 1, 2, -1);
        Assert.Equal(19, aRoll.SurfaceCount);
        int aSpheres = 0, aTori = 0, aTubes = 0;
        for (int i = 1; i <= aRoll.SurfaceCount; i++)
        {
            switch (aRoll.Surface(i) is TrimmedSurface aT ? aT.BasisSurface : aRoll.Surface(i))
            {
                case SphericalSurface aS:
                    aSpheres++;
                    Assert.Equal(1, aS.Radius, 1e-9);
                    break;
                case ToroidalSurface aTorus:
                    aTori++;
                    Assert.Equal(1, aTorus.MinorRadius, 1e-9);
                    Assert.Equal(2, aTorus.MajorRadius, 1e-9);
                    break;
                case CylindricalSurface aC:
                    aTubes++;
                    Assert.Equal(1, aC.Radius, 1e-9);
                    break;
                default:
                    Assert.Fail(aRoll.Surface(i)!.GetType().Name);
                    break;
            }
        }
        Assert.Equal((6, 2, 11), (aSpheres, aTori, aTubes));
        Shape aResult = aRoll.Shape();
        Assert.True(new Analyzer(aResult).IsValid());
        Assert.True(BRepOffset_MakeOffsetTests.Volume(aResult) < BRepOffset_MakeOffsetTests.Volume(aOne));
    }

    [Fact]
    public void A_ball_inside_an_open_box_rounds_its_inner_edges_and_leaves_the_rim()
    {
        // a box without its top: the eight edges between its faces get tubes and its four lower corners spheres; the rim
        // is a free border, between a face and nothing, which the ball has no second face to roll on
        Shape aBox = new MakeBox(10, 10, 10).Shape();
        var aSew = new Sewing(1.0e-6);
        foreach (Shape aFace in BRepOffset_MakeOffsetTests.SubShapes(aBox, ShapeType.Face))
        {
            if (!BRepOffset_MakeOffsetTests.SubShapes(aFace, ShapeType.Vertex).All(v => Math.Abs(Tool.Pnt(ShapeCasts.Vertex(v)).Z - 10) < 1e-9))
            {
                aSew.Add(aFace);
            }
        }
        aSew.Perform();
        Shape anOpen = aSew.SewedShape();
        Assert.Equal(5, Faces(anOpen).Count);
        OffsetBlend aRoll = Rolled(anOpen, 0, 1, -1);
        Assert.Equal(12, aRoll.SurfaceCount);
        int aTubes = 0;
        for (int i = 1; i <= aRoll.SurfaceCount; i++)
        {
            if (aRoll.Surface(i) is SphericalSurface aSphere)
            {
                Assert.Equal(1, aSphere.Position.Location.Z, 1e-9);
                continue;
            }
            aTubes++;
            // no tube reaches above the rim
            Assert.True(Mid(aRoll.CurveOnShape1(i)!).Z <= 10 + 1e-9);
        }
        Assert.Equal(8, aTubes);
        Shape aResult = aRoll.Shape();
        Assert.True(new Analyzer(aResult).IsValid());
        Assert.Equal(17, Faces(aResult).Count);
    }
}
