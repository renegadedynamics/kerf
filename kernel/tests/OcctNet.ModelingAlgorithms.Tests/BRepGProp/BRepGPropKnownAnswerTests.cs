// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.ModelingData.Tests;
using static Occt.GeomAbs_IsoType;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// BRepGProp against closed-form answers: lengths, areas, volumes, centres of mass and inertia of boxes, cylinders,
/// spheres and sphere patches, holed faces, located, reversed and shared shapes, and the mesh-based forms. The shapes
/// are frozen, as a caller's are.
/// </summary>
public class BRepGPropKnownAnswerTests
{
    private const double Tol = 1.0e-10;

    private static void AssertPnt(in gp_Pnt theExpected, in gp_Pnt theActual, double theTol = Tol)
    {
        Assert.Equal(theExpected.X(), theActual.X(), theTol);
        Assert.Equal(theExpected.Y(), theActual.Y(), theTol);
        Assert.Equal(theExpected.Z(), theActual.Z(), theTol);
    }

    private static void AssertMat(in gp_Mat theExpected, in gp_Mat theActual, double theTol)
    {
        for (int r = 1; r <= 3; r++)
        {
            for (int c = 1; c <= 3; c++)
            {
                Assert.Equal(theExpected.Value(r, c), theActual.Value(r, c), theTol);
            }
        }
    }

    private static gp_Mat Diag(double a, double b, double c) => new(a, 0, 0, 0, b, 0, 0, 0, c);

    private static TopLoc_Location RigidLocation()
    {
        gp_Trsf aR = gp_Trsf.Rotation(new gp_Ax1(new gp_Pnt(1, -2, 0.5), new gp_Dir(1, 2, 3)), 0.7);
        return new TopLoc_Location(gp_Trsf.Translation(new gp_Vec(4, -1, 2)).Multiplied(aR));
    }

    // Boxes

    [Fact]
    public void UnitCube_volume_area_length_centre_and_inertia()
    {
        TopoDS_Shape aCube = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        var aCentre = new gp_Pnt(0.5, 0.5, 0.5);

        GProp_GProps aV = BRepGProp.VolumeProperties(aCube);
        Assert.Equal(1.0, aV.Mass(), Tol);
        AssertPnt(aCentre, aV.CentreOfMass());
        AssertMat(Diag(1.0 / 6.0, 1.0 / 6.0, 1.0 / 6.0), aV.MatrixOfInertia(), Tol);
        Assert.Equal(1.0 / 6.0, aV.MomentOfInertia(new gp_Ax1(aCentre, new gp_Dir(1, 1, 1))), Tol);

        GProp_GProps aS = BRepGProp.SurfaceProperties(aCube);
        Assert.Equal(6.0, aS.Mass(), Tol);
        AssertPnt(aCentre, aS.CentreOfMass());
        // about X: the two faces across X give 1/12 + 1/12 each, the four others 1/4 + 1/12 each: 5/3
        AssertMat(Diag(5.0 / 3.0, 5.0 / 3.0, 5.0 / 3.0), aS.MatrixOfInertia(), Tol);

        GProp_GProps aL = BRepGProp.LinearProperties(aCube, true);
        Assert.Equal(12.0, aL.Mass(), Tol);
        AssertPnt(aCentre, aL.CentreOfMass());
    }

    [Fact]
    public void Box_inertia_is_the_closed_form()
    {
        double a = 2, b = 3, c = 4, m = a * b * c;
        GProp_GProps aV = BRepGProp.VolumeProperties(TestShapes.Frozen(TestShapes.Box(a, b, c)));
        Assert.Equal(m, aV.Mass(), 1e-12);
        AssertPnt(new gp_Pnt(1, 1.5, 2), aV.CentreOfMass(), 1e-12);
        AssertMat(Diag(m * (b * b + c * c) / 12, m * (a * a + c * c) / 12, m * (a * a + b * b) / 12), aV.MatrixOfInertia(), 1e-10);

        GProp_PrincipalProps aP = aV.PrincipalProperties();
        Assert.False(aP.HasSymmetryAxis());
        Assert.True(aP.HasSymmetryPoint() == false);
    }

    [Fact]
    public void Located_box_moves_its_centre_and_turns_its_inertia()
    {
        TopLoc_Location aLoc = RigidLocation();
        gp_Trsf aT = aLoc.Transformation();
        TopoDS_Shape aBox = TestShapes.Frozen(TestShapes.Box(2, 3, 4));
        TopoDS_Shape aMoved = aBox.Moved(aLoc);

        GProp_GProps aRef = BRepGProp.VolumeProperties(aBox);
        GProp_GProps aV = BRepGProp.VolumeProperties(aMoved);
        Assert.Equal(24.0, aV.Mass(), 1e-11);
        AssertPnt(aRef.CentreOfMass().Transformed(aT), aV.CentreOfMass(), 1e-11);
        gp_Mat aR = aT.VectorialPart();
        AssertMat(aR.Multiplied(aRef.MatrixOfInertia()).Multiplied(aR.Transposed()), aV.MatrixOfInertia(), 1e-9);

        Assert.Equal(52.0, BRepGProp.SurfaceProperties(aMoved).Mass(), 1e-11);
        Assert.Equal(36.0, BRepGProp.LinearProperties(aMoved, true).Mass(), 1e-11);
        AssertPnt(BRepGProp.LinearProperties(aBox, true).CentreOfMass().Transformed(aT),
                  BRepGProp.LinearProperties(aMoved, true).CentreOfMass(), 1e-11);
    }

    [Fact]
    public void Reversed_solid_has_a_negative_volume()
    {
        TopoDS_Shape aCube = TestShapes.Frozen(TestShapes.Box(1, 2, 3));
        GProp_GProps aV = BRepGProp.VolumeProperties(aCube.Reversed());
        Assert.Equal(-6.0, aV.Mass(), 1e-12);
        AssertPnt(new gp_Pnt(0.5, 1, 1.5), aV.CentreOfMass(), 1e-12);
        // the area doesn't depend on the orientation
        Assert.Equal(22.0, BRepGProp.SurfaceProperties(aCube.Reversed()).Mass(), 1e-12);
    }

    [Fact]
    public void Caller_system_is_reset_with_its_reference_point_at_the_shape_location()
    {
        TopoDS_Shape aCube = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        var aProps = new GProp_GProps(new gp_Pnt(100, 100, 100));
        aProps.Add(BRepGProp.VolumeProperties(aCube), 3.0);
        BRepGProp.VolumeProperties(aCube, aProps);
        Assert.Equal(1.0, aProps.Mass(), Tol);
        AssertPnt(new gp_Pnt(0.5, 0.5, 0.5), aProps.CentreOfMass());
    }

    // Shared solids (the solid-level cache)

    [Fact]
    public void Compound_sharing_a_solid_at_rigid_locations_adds_each_instance()
    {
        TopoDS_Shape aBox = TestShapes.Frozen(TestShapes.Box(1, 2, 3));
        TopLoc_Location aL1 = RigidLocation();
        var aL2 = new TopLoc_Location(gp_Trsf.Translation(new gp_Vec(10, 0, 0)));
        TopoDS_Shape aC = TestShapes.Frozen(BRepGPropTestShapes.CompoundOf(aBox, aBox.Moved(aL1), aBox.Moved(aL2)));

        GProp_GProps aV = BRepGProp.VolumeProperties(aC);
        Assert.Equal(18.0, aV.Mass(), 1e-11);
        var aC0 = new gp_Pnt(0.5, 1, 1.5);
        gp_XYZ anExpected = (aC0.XYZ() + aC0.Transformed(aL1.Transformation()).XYZ() + aC0.Transformed(aL2.Transformation()).XYZ()) / 3.0;
        AssertPnt(new gp_Pnt(anExpected), aV.CentreOfMass(), 1e-11);

        // the same as adding the instances one by one
        var aSum = new GProp_GProps();
        aSum.Add(BRepGProp.VolumeProperties(aBox));
        aSum.Add(BRepGProp.VolumeProperties(aBox.Moved(aL1)));
        aSum.Add(BRepGProp.VolumeProperties(aBox.Moved(aL2)));
        AssertMat(aSum.MatrixOfInertia(), aV.MatrixOfInertia(), 1e-9);
    }

    [Fact]
    public void Compound_sharing_a_solid_reversed_or_scaled_or_twice_in_place()
    {
        TopoDS_Shape aBox = TestShapes.Frozen(TestShapes.Box(1, 2, 3));

        // a reversed instance cancels the first one
        TopoDS_Shape aC1 = TestShapes.Frozen(BRepGPropTestShapes.CompoundOf(aBox, aBox.Reversed()));
        Assert.Equal(0.0, BRepGProp.VolumeProperties(aC1).Mass(), 1e-12);

        // a scaled instance is integrated face by face
        var aScale = new TopLoc_Location(gp_Trsf.Scale(new gp_Pnt(0, 0, 0), 2.0));
        TopoDS_Shape aC2 = TestShapes.Frozen(BRepGPropTestShapes.CompoundOf(aBox, aBox.Moved(aScale)));
        Assert.Equal(6.0 + 48.0, BRepGProp.VolumeProperties(aC2).Mass(), 1e-10);

        // twice in place: SkipShared counts it once
        TopoDS_Shape aC3 = TestShapes.Frozen(BRepGPropTestShapes.CompoundOf(aBox, aBox));
        Assert.Equal(12.0, BRepGProp.VolumeProperties(aC3).Mass(), 1e-11);
        Assert.Equal(6.0, BRepGProp.VolumeProperties(aC3, false, true).Mass(), 1e-11);
        Assert.Equal(44.0, BRepGProp.SurfaceProperties(aC3).Mass(), 1e-11);
        Assert.Equal(22.0, BRepGProp.SurfaceProperties(aC3, true).Mass(), 1e-11);
    }

    [Fact]
    public void Compound_with_a_shared_solid_and_a_free_face()
    {
        TopoDS_Shape aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        TopoDS_Shape aTwin = TestShapes.Frozen(TestShapes.Box(1, 1, 1)); // the same box, another TShape
        TopoDS_Face aFree = TestShapes.Frozen(BRepGPropTestShapes.Faces(TestShapes.Box(2, 2, 2))[1]); // z = 2, normal +Z
        var aL = new TopLoc_Location(gp_Trsf.Translation(new gp_Vec(5, 0, 0)));
        TopoDS_Shape aShared = TestShapes.Frozen(BRepGPropTestShapes.CompoundOf(aBox, aBox.Moved(aL), aFree));
        TopoDS_Shape aDistinct = TestShapes.Frozen(BRepGPropTestShapes.CompoundOf(aBox, aTwin.Moved(aL), aFree));

        // the shared-solid path (cache and free faces) agrees with the face-by-face path; the free face's
        // contribution depends on the reference point, which is the same for both
        GProp_GProps aV1 = BRepGProp.VolumeProperties(aShared);
        GProp_GProps aV2 = BRepGProp.VolumeProperties(aDistinct);
        Assert.Equal(aV2.Mass(), aV1.Mass(), 1e-12);
        AssertPnt(aV2.CentreOfMass(), aV1.CentreOfMass(), 1e-11);
        AssertMat(aV2.MatrixOfInertia(), aV1.MatrixOfInertia(), 1e-10);
    }

    // Cylinders and spheres

    [Fact]
    public void Cylinder_closed_form()
    {
        double r = 2, h = 3, m = Math.PI * r * r * h;
        TopoDS_Shape aCyl = TestShapes.Frozen(BRepGPropTestShapes.CylinderSolid(r, h));

        GProp_GProps aV = BRepGProp.VolumeProperties(aCyl);
        Assert.Equal(m, aV.Mass(), 1e-9);
        AssertPnt(new gp_Pnt(0, 0, h / 2), aV.CentreOfMass(), 1e-9);
        AssertMat(Diag(m * (3 * r * r + h * h) / 12, m * (3 * r * r + h * h) / 12, m * r * r / 2), aV.MatrixOfInertia(), 1e-8);
        Assert.True(aV.PrincipalProperties().HasSymmetryAxis());

        GProp_GProps aS = BRepGProp.SurfaceProperties(aCyl);
        Assert.Equal(2 * Math.PI * r * r + 2 * Math.PI * r * h, aS.Mass(), 1e-9);
        AssertPnt(new gp_Pnt(0, 0, h / 2), aS.CentreOfMass(), 1e-9);

        GProp_GProps aL = BRepGProp.LinearProperties(aCyl, true);
        Assert.Equal(2 * 2 * Math.PI * r + h, aL.Mass(), 1e-9);
    }

    [Fact]
    public void Sphere_closed_form()
    {
        double r = 2, m = 4.0 / 3.0 * Math.PI * r * r * r, a = 4 * Math.PI * r * r;
        TopoDS_Shape aSphere = TestShapes.Frozen(BRepGPropTestShapes.SphereSolid(r));

        GProp_GProps aV = BRepGProp.VolumeProperties(aSphere);
        Assert.Equal(m, aV.Mass(), 1e-9);
        AssertPnt(new gp_Pnt(0, 0, 0), aV.CentreOfMass(), 1e-9);
        AssertMat(Diag(0.4 * m * r * r, 0.4 * m * r * r, 0.4 * m * r * r), aV.MatrixOfInertia(), 1e-8);
        Assert.True(aV.PrincipalProperties().HasSymmetryPoint());

        GProp_GProps aS = BRepGProp.SurfaceProperties(aSphere);
        Assert.Equal(a, aS.Mass(), 1e-9);
        AssertMat(Diag(2.0 / 3.0 * a * r * r, 2.0 / 3.0 * a * r * r, 2.0 / 3.0 * a * r * r), aS.MatrixOfInertia(), 1e-8);
    }

    [Fact]
    public void Sphere_patches_have_their_zone_areas()
    {
        double r = 3;
        var aSphere = new Geom_SphericalSurface(new gp_Ax3(new gp_Pnt(1, 2, 3), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), r);

        // upper hemisphere: 2 Pi r^2, centroid r/2 above the centre
        TopoDS_Face aHemi = TestShapes.Frozen(BRepGPropTestShapes.NaturalFace(
            new Geom_RectangularTrimmedSurface(aSphere, 0, 2 * Math.PI, 0, Math.PI / 2)));
        GProp_GProps aS = BRepGProp.SurfaceProperties(aHemi);
        Assert.Equal(2 * Math.PI * r * r, aS.Mass(), 1e-9);
        AssertPnt(new gp_Pnt(1, 2, 3 + r / 2), aS.CentreOfMass(), 1e-9);

        // a quarter of the upper hemisphere: r^2 (Pi / 2) (sin(Pi/2) - sin(0))
        TopoDS_Face aPatch = TestShapes.Frozen(BRepGPropTestShapes.NaturalFace(
            new Geom_RectangularTrimmedSurface(aSphere, 0, Math.PI / 2, 0, Math.PI / 2)));
        Assert.Equal(r * r * Math.PI / 2, BRepGProp.SurfaceProperties(aPatch).Mass(), 1e-9);

        // adaptive integration gives the same area within its error
        var aProps = new GProp_GProps();
        double anError = BRepGProp.SurfaceProperties(aPatch, aProps, 1.0e-8);
        Assert.Equal(r * r * Math.PI / 2, aProps.Mass(), 1e-7);
        Assert.True(anError >= 0.0 && anError < 1e-6);
    }

    // Holed faces

    private static TopoDS_Face HoledSquare()
    {
        // a 10 x 10 square on z = 0 with a hole [2, 4] x [3, 7]
        var aB = new BRep_Builder();
        var aPlane = new Geom_Plane(new gp_Ax3(new gp_Pnt(), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)));
        TopoDS_Face aF = new();
        aB.MakeFace(ref aF, aPlane, 1e-7);
        (double, double)[][] aLoops =
        [
            [(0, 0), (10, 0), (10, 10), (0, 10)], // counter-clockwise
            [(2, 3), (2, 7), (4, 7), (4, 3)],     // clockwise
        ];
        foreach ((double, double)[] aLoop in aLoops)
        {
            var aV = aLoop.Select(p => TestShapes.Vertex(new gp_Pnt(p.Item1, p.Item2, 0))).ToArray();
            TopoDS_Wire aW = new();
            aB.MakeWire(ref aW);
            for (int i = 0; i < aV.Length; i++)
            {
                TopoDS_Edge anE = TestShapes.Edge(aV[i], aV[(i + 1) % aV.Length]);
                (double x1, double y1) = aLoop[i];
                (double x2, double y2) = aLoop[(i + 1) % aLoop.Length];
                aB.UpdateEdge(anE, new Geom2d_Line(new gp_Pnt2d(x1, y1), new gp_Dir2d(x2 - x1, y2 - y1)), aF, 1e-7);
                aB.Add(aW, anE);
            }

            aW.Closed(true);
            aB.Add(aF, aW);
        }

        return TestShapes.Frozen(aF);
    }

    [Fact]
    public void Holed_face_area_centre_and_boundary_length()
    {
        TopoDS_Face aF = HoledSquare();
        GProp_GProps aS = BRepGProp.SurfaceProperties(aF);
        Assert.Equal(92.0, aS.Mass(), 1e-11);
        AssertPnt(new gp_Pnt((100 * 5 - 8 * 3) / 92.0, (100 * 5 - 8 * 5) / 92.0, 0), aS.CentreOfMass(), 1e-11);

        GProp_GProps aL = BRepGProp.LinearProperties(aF);
        Assert.Equal(40.0 + 12.0, aL.Mass(), 1e-11);

        // the same through the adaptive form
        var aProps = new GProp_GProps();
        BRepGProp.SurfaceProperties(aF, aProps, 1e-9);
        Assert.Equal(92.0, aProps.Mass(), 1e-9);

        // reversing the face doesn't change its area
        Assert.Equal(92.0, BRepGProp.SurfaceProperties(aF.Reversed()).Mass(), 1e-11);
    }

    // The adaptive and Gauss-Kronrod forms

    [Fact]
    public void Adaptive_volume_matches_the_closed_form()
    {
        double r = 2, h = 3;
        TopoDS_Shape aCyl = TestShapes.Frozen(BRepGPropTestShapes.CylinderSolid(r, h));
        var aProps = new GProp_GProps();
        double anError = BRepGProp.VolumeProperties(aCyl, aProps, 1e-8);
        Assert.Equal(Math.PI * r * r * h, aProps.Mass(), 1e-7);
        Assert.True(anError >= 0.0 && anError < 1e-6, $"error {anError}");

        TopoDS_Shape aCube = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        BRepGProp.VolumeProperties(aCube, aProps, 1e-6, true);
        Assert.Equal(1.0, aProps.Mass(), 1e-9);
    }

    [Fact]
    public void Gauss_Kronrod_volume_centre_and_inertia()
    {
        TopoDS_Shape aCube = TestShapes.Frozen(TestShapes.Box(1, 2, 3));
        var aProps = new GProp_GProps();
        double anError = BRepGProp.VolumePropertiesGK(aCube, aProps, 1e-6, false, false, true, true);
        Assert.Equal(6.0, aProps.Mass(), 1e-9);
        AssertPnt(new gp_Pnt(0.5, 1, 1.5), aProps.CentreOfMass(), 1e-9);
        AssertMat(Diag(6 * 13 / 12.0, 6 * 10 / 12.0, 6 * 5 / 12.0), aProps.MatrixOfInertia(), 1e-8);
        Assert.True(anError >= 0.0 && anError < 1e-5, $"error {anError}");

        double r = 2, h = 3;
        TopoDS_Shape aCyl = TestShapes.Frozen(BRepGPropTestShapes.CylinderSolid(r, h));
        BRepGProp.VolumePropertiesGK(aCyl, aProps, 1e-8, true, true, true);
        Assert.Equal(Math.PI * r * r * h, aProps.Mass(), 1e-6);
        AssertPnt(new gp_Pnt(0, 0, h / 2), aProps.CentreOfMass(), 1e-6);

        // the region between the faces of a closed shell and any plane is the enclosed volume
        var aPln = new gp_Pln(new gp_Pnt(0, 0, -5), new gp_Dir(1, 2, 2));
        BRepGProp.VolumePropertiesGK(aCube, aProps, aPln, 1e-6, false, false, true);
        Assert.Equal(6.0, aProps.Mass(), 1e-8);
        AssertPnt(new gp_Pnt(0.5, 1, 1.5), aProps.CentreOfMass(), 1e-8);
    }

    [Fact]
    public void Only_closed_shells_count()
    {
        TopoDS_Solid aBox = TestShapes.Box(1, 1, 1);
        // an open shell: a box without its top face
        var aB = new BRep_Builder();
        TopoDS_Shell anOpen = new();
        aB.MakeShell(ref anOpen);
        List<TopoDS_Face> aFaces = BRepGPropTestShapes.Faces(TestShapes.Box(2, 2, 2));
        for (int i = 0; i < aFaces.Count; i++)
        {
            if (i != 1)
            {
                aB.Add(anOpen, aFaces[i]);
            }
        }

        TopoDS_Shape aC = TestShapes.Frozen(BRepGPropTestShapes.CompoundOf(aBox, BRepGPropTestShapes.SolidOf(anOpen)));
        Assert.Equal(1.0, BRepGProp.VolumeProperties(aC, true).Mass(), 1e-12);

        var aProps = new GProp_GProps();
        Assert.True(BRepGProp.VolumePropertiesGK(aC, aProps, 1e-6, true) >= 0.0);
        Assert.Equal(1.0, aProps.Mass(), 1e-9);

        // no closed shell: GK returns -1
        Assert.Equal(-1.0, BRepGProp.VolumePropertiesGK(TestShapes.Frozen(anOpen), aProps, 1e-6, true));
    }

    // Meshes

    [Fact]
    public void Mesh_forms_measure_the_attached_triangulations()
    {
        TopoDS_Solid aBox = TestShapes.Box(1, 2, 3);
        BRepGPropTestShapes.AttachQuadMeshes(aBox);
        TopoDS_Shape aMeshed = TestShapes.Frozen(aBox);

        GProp_GProps aV = BRepGProp.VolumeProperties(aMeshed, false, false, true);
        Assert.Equal(6.0, aV.Mass(), 1e-12);
        AssertPnt(new gp_Pnt(0.5, 1, 1.5), aV.CentreOfMass(), 1e-12);
        AssertMat(Diag(6 * 13 / 12.0, 6 * 10 / 12.0, 6 * 5 / 12.0), aV.MatrixOfInertia(), 1e-11);

        GProp_GProps aS = BRepGProp.SurfaceProperties(aMeshed, false, true);
        Assert.Equal(22.0, aS.Mass(), 1e-12);
        AssertPnt(new gp_Pnt(0.5, 1, 1.5), aS.CentreOfMass(), 1e-12);

        // located: the rotation path, and a scale (the copy path)
        TopLoc_Location aLoc = RigidLocation();
        GProp_GProps aVL = BRepGProp.VolumeProperties(aMeshed.Moved(aLoc), false, false, true);
        Assert.Equal(6.0, aVL.Mass(), 1e-11);
        AssertPnt(new gp_Pnt(0.5, 1, 1.5).Transformed(aLoc.Transformation()), aVL.CentreOfMass(), 1e-11);
        gp_Mat aR = aLoc.Transformation().VectorialPart();
        AssertMat(aR.Multiplied(aV.MatrixOfInertia()).Multiplied(aR.Transposed()), aVL.MatrixOfInertia(), 1e-9);

        var aScale = new TopLoc_Location(gp_Trsf.Scale(new gp_Pnt(1, 1, 1), 2.0));
        GProp_GProps aVS = BRepGProp.VolumeProperties(aMeshed.Moved(aScale), false, false, true);
        Assert.Equal(48.0, aVS.Mass(), 1e-11);
        AssertPnt(new gp_Pnt(0, 1, 2), aVS.CentreOfMass(), 1e-11);
        Assert.Equal(88.0, BRepGProp.SurfaceProperties(aMeshed.Moved(aScale), false, true).Mass(), 1e-11);
    }

    [Fact]
    public void Faces_without_surface_and_edges_without_curve_use_their_meshes()
    {
        gp_Pnt[] aNodes = [new(0, 0, 0), new(1, 0, 0), new(0, 1, 0), new(0, 0, 1)];
        var aB = new BRep_Builder();
        TopoDS_Shell aShell = new();
        aB.MakeShell(ref aShell);
        foreach ((int, int, int) aTri in new[] { (1, 3, 2), (1, 2, 4), (1, 4, 3), (2, 3, 4) })
        {
            aB.Add(aShell, BRepGPropTestShapes.MeshFace(aNodes, aTri));
        }

        TopoDS_Shape aTetra = TestShapes.Frozen(BRepGPropTestShapes.SolidOf(aShell));
        GProp_GProps aV = BRepGProp.VolumeProperties(aTetra);
        Assert.Equal(1.0 / 6.0, aV.Mass(), 1e-14);
        AssertPnt(new gp_Pnt(0.25, 0.25, 0.25), aV.CentreOfMass(), 1e-14);
        Assert.Equal(1.5 + Math.Sqrt(3) / 2, BRepGProp.SurfaceProperties(aTetra).Mass(), 1e-14);

        TopoDS_Edge aPolyline = TestShapes.Frozen(BRepGPropTestShapes.PolygonEdge(new(0, 0, 0), new(3, 0, 0), new(3, 4, 0)));
        GProp_GProps aL = BRepGProp.LinearProperties(aPolyline);
        Assert.Equal(7.0, aL.Mass(), 1e-14);
        AssertPnt(new gp_Pnt((3 * 1.5 + 4 * 3) / 7.0, 4 * 2 / 7.0, 0), aL.CentreOfMass(), 1e-14);
    }

    [Fact]
    public void MeshCinert_and_MeshProps_on_raw_data()
    {
        var aNodes = new NCollection_Array1<gp_Pnt>(1, 3);
        aNodes[1] = new gp_Pnt(0, 0, 0);
        aNodes[2] = new gp_Pnt(0, 0, 0); // a null segment is skipped
        aNodes[3] = new gp_Pnt(0, 0, 2);
        var aMC = new BRepGProp_MeshCinert();
        aMC.SetLocation(new gp_Pnt(1, 1, 1));
        aMC.Perform(aNodes);
        Assert.Equal(2.0, aMC.Mass(), 1e-15);
        AssertPnt(new gp_Pnt(0, 0, 1), aMC.CentreOfMass(), 1e-15);
        // a rod of length 2 about its centre: m L^2 / 12 across it
        AssertMat(Diag(2 * 4 / 12.0, 2 * 4 / 12.0, 0), aMC.MatrixOfInertia(), 1e-14);

        // one triangle: area and centroid
        var aTri = new Poly_Triangulation(3, 1, false);
        aTri.SetNode(1, new gp_Pnt(0, 0, 0));
        aTri.SetNode(2, new gp_Pnt(3, 0, 0));
        aTri.SetNode(3, new gp_Pnt(0, 3, 0));
        aTri.SetTriangle(1, new Poly_Triangle(1, 2, 3));
        var aMP = new BRepGProp_MeshProps(BRepGProp_MeshProps.BRepGProp_MeshObjType.Sinert);
        aMP.SetLocation(new gp_Pnt(0, 0, 0));
        aMP.Perform(aTri, TopAbs_FORWARD);
        Assert.Equal(4.5, aMP.Mass(), 1e-14);
        AssertPnt(new gp_Pnt(1, 1, 0), aMP.CentreOfMass(), 1e-14);
        Assert.Equal(BRepGProp_MeshProps.BRepGProp_MeshObjType.Sinert, aMP.GetMeshObjType());

        // a pyramid from (0, 0, 1) on it: volume base * height / 3, negative seen from above when reversed
        var aMV = new BRepGProp_MeshProps(BRepGProp_MeshProps.BRepGProp_MeshObjType.Vinert);
        aMV.SetLocation(new gp_Pnt(0, 0, 1));
        aMV.Perform(aTri, TopAbs_REVERSED);
        Assert.Equal(1.5, aMV.Mass(), 1e-14);
        aMV.Perform(aTri, TopAbs_FORWARD);
        Assert.Equal(-1.5, aMV.Mass(), 1e-14);

        // an empty or null mesh leaves the system as it is
        aMV.Perform(null, TopAbs_FORWARD);
        Assert.Equal(-1.5, aMV.Mass(), 1e-14);

        // a degenerate triangle adds nothing
        Span<double> aGProps = stackalloc double[10];
        BRepGProp_MeshProps.CalculateProps(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0), new gp_Pnt(2, 0, 0), new gp_Pnt(0, 0, 0),
                                           false, aGProps, 1, [1.0 / 3.0, 1.0 / 3.0, 0.5]);
        Assert.Equal(0.0, aGProps[0]);
    }

    // The tools

    [Fact]
    public void Face_tool_bounds_normals_orders_and_knots()
    {
        TopoDS_Shape aCyl = TestShapes.Frozen(BRepGPropTestShapes.CylinderSolid(2, 3));
        TopoDS_Face aLateral = BRepGPropTestShapes.Faces(aCyl)[0];
        var aBF = new BRepGProp_Face(aLateral);
        aBF.Bounds(out double u1, out double u2, out double v1, out double v2);
        Assert.Equal(0.0, u1, 1e-12);
        Assert.Equal(2 * Math.PI, u2, 1e-12);
        Assert.Equal(0.0, v1, 1e-12);
        Assert.Equal(3.0, v2, 1e-12);
        aBF.Normal(0.0, 1.0, out gp_Pnt aP, out gp_Vec aN);
        AssertPnt(new gp_Pnt(2, 0, 1), aP, 1e-12);
        Assert.True(aN.X() > 0); // outward, length r
        Assert.Equal(2.0, aN.Magnitude(), 1e-12);
        Assert.Equal(18, aBF.UIntegrationOrder());
        Assert.Equal(18, aBF.VIntegrationOrder());
        Assert.Equal(3, aBF.SUIntSubs());
        Assert.Equal(1, aBF.SVIntSubs());
        var aKnots = new NCollection_Array1<double>(1, 4);
        aBF.UKnots(aKnots);
        Assert.Equal(2 * Math.PI / 3, aKnots[2], 1e-15);
        Assert.False(aBF.NaturalRestriction());
        Assert.True(aBF.GetFace().IsSame(aLateral));

        // reversed: the normal turns
        var aBR = new BRepGProp_Face(TopoDS.Face(aLateral.Reversed()));
        aBR.Normal(0.0, 1.0, out _, out gp_Vec aNR);
        Assert.True(aNR.X() < 0);

        // the boundary arcs: the edges' pcurves, and the sides of the UV rectangle
        var aBD = new BRepGProp_Domain(aLateral);
        int aNb = 0;
        for (; aBD.More(); aBD.Next())
        {
            Assert.True(aBF.Load(aBD.Value()));
            Assert.True(aBF.IntegrationOrder() >= 4);
            aNb++;
        }

        Assert.Equal(4, aNb);
        aBF.Load(true, GeomAbs_IsoU);
        Assert.Equal(0.0, aBF.FirstParameter());
        Assert.Equal(3.0, aBF.LastParameter(), 1e-15);
        AssertPnt(new gp_Pnt(0, 3, 0), new gp_Pnt(aBF.Value2d(0).X(), aBF.Value2d(0).Y(), 0), 1e-15);
        NCollection_HArray1<double> aU = aBF.GetUKnots(0.5, 1.5);
        Assert.Equal(2, aU.Length());
        Assert.Equal(1.5, aU[2]);

        // an edge without a pcurve on the face isn't loaded
        TopoDS_Edge aStray = TestShapes.Frozen(TestShapes.Edge(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0)));
        Assert.False(aBF.Load(aStray));
    }

    [Fact]
    public void Domain_skips_internal_and_external_edges_after_the_first()
    {
        // a square face whose wire also holds an INTERNAL edge
        TopoDS_Face aSquare = BRepGPropTestShapes.Faces(TestShapes.Box(1, 1, 1))[0];
        var aB = new BRep_Builder();
        var aPlane = (Geom_Plane)BRep_Tool.Surface(aSquare)!;
        TopoDS_Face aF = new();
        aB.MakeFace(ref aF, aPlane, 1e-7);
        TopoDS_Wire aW = new();
        aB.MakeWire(ref aW);
        TopoDS_Edge anInternal = TestShapes.Edge(new gp_Pnt(0.2, 0.2, 0), new gp_Pnt(0.8, 0.8, 0));
        aB.Add(aW, anInternal.Oriented(TopAbs_INTERNAL));
        for (var anExp = new TopExp_Explorer(aSquare, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            aB.Add(aW, anExp.Current());
        }

        aB.Add(aW, anInternal.Oriented(TopAbs_EXTERNAL));
        aB.Add(aF, aW);
        TopoDS_Face aFace = TestShapes.Frozen(aF);

        var aD = new BRepGProp_Domain(aFace);
        var anOrientations = new List<TopAbs_Orientation>();
        for (; aD.More(); aD.Next())
        {
            anOrientations.Add(aD.Value().Orientation());
        }

        // OCCT's Domain checks only from Next: the first edge comes even if INTERNAL
        Assert.Equal(5, anOrientations.Count);
        Assert.Equal(TopAbs_INTERNAL, anOrientations[0]);
        Assert.DoesNotContain(TopAbs_EXTERNAL, anOrientations);

        aD.Init();
        Assert.True(aD.More());
    }

    [Fact]
    public void Vinert_by_plane_returns_a_null_mass_as_OCCT_does()
    {
        // OCCT: BRepGProp_Gauss::convert clears the mass and centre of a region bounded by a plane (occt-issues.md)
        TopoDS_Face aTop = BRepGPropTestShapes.Faces(TestShapes.Frozen(TestShapes.Box(1, 1, 1)))[1];
        var aBF = new BRepGProp_Face(aTop);
        var aBD = new BRepGProp_Domain(aTop);
        var aPln = new gp_Pln(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1));
        var aV = new BRepGProp_Vinert(aBF, aBD, aPln, new gp_Pnt(0, 0, 0));
        Assert.Equal(0.0, aV.Mass());

        // by point it is the pyramid on the face: base 1, height 1
        aBD.Init();
        var aVP = new BRepGProp_Vinert(aBF, aBD, new gp_Pnt(0, 0, 0));
        Assert.Equal(1.0 / 3.0, aVP.Mass(), 1e-14);

        // the Gauss-Kronrod form by plane is right: the prism down to z = 0
        aBD.Init();
        var aGK = new BRepGProp_VinertGK(aBF, aBD, aPln, new gp_Pnt(0, 0, 0), 1e-8, true);
        Assert.Equal(1.0, aGK.Mass(), 1e-9);
        AssertPnt(new gp_Pnt(0.5, 0.5, 0.5), aGK.CentreOfMass(), 1e-9);
        Assert.Equal(aGK.GetErrorReached() * aGK.Mass(), aGK.GetAbsolutError(), 1e-20);
    }

    [Fact]
    public void Gauss_asserts_the_type()
    {
        TopoDS_Face aF = BRepGPropTestShapes.Faces(TestShapes.Frozen(TestShapes.Box(1, 1, 1)))[0];
        var aBF = new BRepGProp_Face(aF);
        var aGauss = new BRepGProp_Gauss(BRepGProp_Gauss.BRepGProp_GaussType.Vinert);
        double aMass = 0;
        gp_Pnt aG = new();
        gp_Mat anI = new();
        Assert.Throws<Standard_ProgramError>(() => aGauss.Compute(aBF, new gp_Pnt(), ref aMass, ref aG, ref anI));
    }
}
