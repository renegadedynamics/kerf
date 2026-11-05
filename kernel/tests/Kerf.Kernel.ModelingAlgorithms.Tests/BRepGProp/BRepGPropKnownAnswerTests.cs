// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Measurement;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.ModelingData.Tests;
using static Kerf.Engine.Geometry.IsoType;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// BRepGProp against closed-form answers: lengths, areas, volumes, centres of mass and inertia of boxes, cylinders,
/// spheres and sphere patches, holed faces, located, reversed and shared shapes, and the mesh-based forms. The shapes
/// are frozen, as a caller's are.
/// </summary>
public class BRepGPropKnownAnswerTests
{
    private const double Tol = 1.0e-10;

    private static void AssertPnt(in Point3D theExpected, in Point3D theActual, double theTol = Tol)
    {
        Assert.Equal(theExpected.X, theActual.X, theTol);
        Assert.Equal(theExpected.Y, theActual.Y, theTol);
        Assert.Equal(theExpected.Z, theActual.Z, theTol);
    }

    private static void AssertMat(in Matrix3x3 theExpected, in Matrix3x3 theActual, double theTol)
    {
        for (int r = 1; r <= 3; r++)
        {
            for (int c = 1; c <= 3; c++)
            {
                Assert.Equal(theExpected.Value(r, c), theActual.Value(r, c), theTol);
            }
        }
    }

    private static Matrix3x3 Diag(double a, double b, double c) => new(a, 0, 0, 0, b, 0, 0, 0, c);

    private static Placement RigidLocation()
    {
        Transform3D aR = Transform3D.Rotation(new Axis3D(new Point3D(1, -2, 0.5), new Direction3D(1, 2, 3)), 0.7);
        return new Placement(Transform3D.Translation(new Vector3D(4, -1, 2)).Multiplied(aR));
    }

    // Boxes

    [Fact]
    public void UnitCube_volume_area_length_centre_and_inertia()
    {
        Shape aCube = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        var aCentre = new Point3D(0.5, 0.5, 0.5);

        Properties aV = MassPropertiesBuilder.VolumeProperties(aCube);
        Assert.Equal(1.0, aV.Mass(), Tol);
        AssertPnt(aCentre, aV.CentreOfMass());
        AssertMat(Diag(1.0 / 6.0, 1.0 / 6.0, 1.0 / 6.0), aV.MatrixOfInertia(), Tol);
        Assert.Equal(1.0 / 6.0, aV.MomentOfInertia(new Axis3D(aCentre, new Direction3D(1, 1, 1))), Tol);

        Properties aS = MassPropertiesBuilder.SurfaceProperties(aCube);
        Assert.Equal(6.0, aS.Mass(), Tol);
        AssertPnt(aCentre, aS.CentreOfMass());
        // about X: the two faces across X give 1/12 + 1/12 each, the four others 1/4 + 1/12 each: 5/3
        AssertMat(Diag(5.0 / 3.0, 5.0 / 3.0, 5.0 / 3.0), aS.MatrixOfInertia(), Tol);

        Properties aL = MassPropertiesBuilder.LinearProperties(aCube, true);
        Assert.Equal(12.0, aL.Mass(), Tol);
        AssertPnt(aCentre, aL.CentreOfMass());
    }

    [Fact]
    public void Box_inertia_is_the_closed_form()
    {
        double a = 2, b = 3, c = 4, m = a * b * c;
        Properties aV = MassPropertiesBuilder.VolumeProperties(TestShapes.Frozen(TestShapes.Box(a, b, c)));
        Assert.Equal(m, aV.Mass(), 1e-12);
        AssertPnt(new Point3D(1, 1.5, 2), aV.CentreOfMass(), 1e-12);
        AssertMat(Diag(m * (b * b + c * c) / 12, m * (a * a + c * c) / 12, m * (a * a + b * b) / 12), aV.MatrixOfInertia(), 1e-10);

        PrincipalProperties aP = aV.PrincipalProperties();
        Assert.False(aP.HasSymmetryAxis());
        Assert.True(aP.HasSymmetryPoint() == false);
    }

    [Fact]
    public void Located_box_moves_its_centre_and_turns_its_inertia()
    {
        Placement aLoc = RigidLocation();
        Transform3D aT = aLoc.Transformation;
        Shape aBox = TestShapes.Frozen(TestShapes.Box(2, 3, 4));
        Shape aMoved = aBox.Moved(aLoc);

        Properties aRef = MassPropertiesBuilder.VolumeProperties(aBox);
        Properties aV = MassPropertiesBuilder.VolumeProperties(aMoved);
        Assert.Equal(24.0, aV.Mass(), 1e-11);
        AssertPnt(aRef.CentreOfMass().Transformed(aT), aV.CentreOfMass(), 1e-11);
        Matrix3x3 aR = aT.VectorialPart;
        AssertMat(aR.Multiplied(aRef.MatrixOfInertia()).Multiplied(aR.Transposed()), aV.MatrixOfInertia(), 1e-9);

        Assert.Equal(52.0, MassPropertiesBuilder.SurfaceProperties(aMoved).Mass(), 1e-11);
        Assert.Equal(36.0, MassPropertiesBuilder.LinearProperties(aMoved, true).Mass(), 1e-11);
        AssertPnt(MassPropertiesBuilder.LinearProperties(aBox, true).CentreOfMass().Transformed(aT),
                  MassPropertiesBuilder.LinearProperties(aMoved, true).CentreOfMass(), 1e-11);
    }

    [Fact]
    public void Reversed_solid_has_a_negative_volume()
    {
        Shape aCube = TestShapes.Frozen(TestShapes.Box(1, 2, 3));
        Properties aV = MassPropertiesBuilder.VolumeProperties(aCube.Reversed());
        Assert.Equal(-6.0, aV.Mass(), 1e-12);
        AssertPnt(new Point3D(0.5, 1, 1.5), aV.CentreOfMass(), 1e-12);
        // the area doesn't depend on the orientation
        Assert.Equal(22.0, MassPropertiesBuilder.SurfaceProperties(aCube.Reversed()).Mass(), 1e-12);
    }

    [Fact]
    public void Caller_system_is_reset_with_its_reference_point_at_the_shape_location()
    {
        Shape aCube = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        var aProps = new Properties(new Point3D(100, 100, 100));
        aProps.Add(MassPropertiesBuilder.VolumeProperties(aCube), 3.0);
        MassPropertiesBuilder.VolumeProperties(aCube, aProps);
        Assert.Equal(1.0, aProps.Mass(), Tol);
        AssertPnt(new Point3D(0.5, 0.5, 0.5), aProps.CentreOfMass());
    }

    // Shared solids (the solid-level cache)

    [Fact]
    public void Compound_sharing_a_solid_at_rigid_locations_adds_each_instance()
    {
        Shape aBox = TestShapes.Frozen(TestShapes.Box(1, 2, 3));
        Placement aL1 = RigidLocation();
        var aL2 = new Placement(Transform3D.Translation(new Vector3D(10, 0, 0)));
        Shape aC = TestShapes.Frozen(BRepGPropTestShapes.CompoundOf(aBox, aBox.Moved(aL1), aBox.Moved(aL2)));

        Properties aV = MassPropertiesBuilder.VolumeProperties(aC);
        Assert.Equal(18.0, aV.Mass(), 1e-11);
        var aC0 = new Point3D(0.5, 1, 1.5);
        Xyz anExpected = (aC0.XYZ() + aC0.Transformed(aL1.Transformation).XYZ() + aC0.Transformed(aL2.Transformation).XYZ()) / 3.0;
        AssertPnt(new Point3D(anExpected), aV.CentreOfMass(), 1e-11);

        // the same as adding the instances one by one
        var aSum = new Properties();
        aSum.Add(MassPropertiesBuilder.VolumeProperties(aBox));
        aSum.Add(MassPropertiesBuilder.VolumeProperties(aBox.Moved(aL1)));
        aSum.Add(MassPropertiesBuilder.VolumeProperties(aBox.Moved(aL2)));
        AssertMat(aSum.MatrixOfInertia(), aV.MatrixOfInertia(), 1e-9);
    }

    [Fact]
    public void Compound_sharing_a_solid_reversed_or_scaled_or_twice_in_place()
    {
        Shape aBox = TestShapes.Frozen(TestShapes.Box(1, 2, 3));

        // a reversed instance cancels the first one
        Shape aC1 = TestShapes.Frozen(BRepGPropTestShapes.CompoundOf(aBox, aBox.Reversed()));
        Assert.Equal(0.0, MassPropertiesBuilder.VolumeProperties(aC1).Mass(), 1e-12);

        // a scaled instance is integrated face by face
        var aScale = new Placement(Transform3D.Scale(new Point3D(0, 0, 0), 2.0));
        Shape aC2 = TestShapes.Frozen(BRepGPropTestShapes.CompoundOf(aBox, aBox.Moved(aScale)));
        Assert.Equal(6.0 + 48.0, MassPropertiesBuilder.VolumeProperties(aC2).Mass(), 1e-10);

        // twice in place: SkipShared counts it once
        Shape aC3 = TestShapes.Frozen(BRepGPropTestShapes.CompoundOf(aBox, aBox));
        Assert.Equal(12.0, MassPropertiesBuilder.VolumeProperties(aC3).Mass(), 1e-11);
        Assert.Equal(6.0, MassPropertiesBuilder.VolumeProperties(aC3, false, true).Mass(), 1e-11);
        Assert.Equal(44.0, MassPropertiesBuilder.SurfaceProperties(aC3).Mass(), 1e-11);
        Assert.Equal(22.0, MassPropertiesBuilder.SurfaceProperties(aC3, true).Mass(), 1e-11);
    }

    [Fact]
    public void Compound_with_a_shared_solid_and_a_free_face()
    {
        Shape aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        Shape aTwin = TestShapes.Frozen(TestShapes.Box(1, 1, 1)); // the same box, another TShape
        Face aFree = TestShapes.Frozen(BRepGPropTestShapes.Faces(TestShapes.Box(2, 2, 2))[1]); // z = 2, normal +Z
        var aL = new Placement(Transform3D.Translation(new Vector3D(5, 0, 0)));
        Shape aShared = TestShapes.Frozen(BRepGPropTestShapes.CompoundOf(aBox, aBox.Moved(aL), aFree));
        Shape aDistinct = TestShapes.Frozen(BRepGPropTestShapes.CompoundOf(aBox, aTwin.Moved(aL), aFree));

        // the shared-solid path (cache and free faces) agrees with the face-by-face path; the free face's
        // contribution depends on the reference point, which is the same for both
        Properties aV1 = MassPropertiesBuilder.VolumeProperties(aShared);
        Properties aV2 = MassPropertiesBuilder.VolumeProperties(aDistinct);
        Assert.Equal(aV2.Mass(), aV1.Mass(), 1e-12);
        AssertPnt(aV2.CentreOfMass(), aV1.CentreOfMass(), 1e-11);
        AssertMat(aV2.MatrixOfInertia(), aV1.MatrixOfInertia(), 1e-10);
    }

    // Cylinders and spheres

    [Fact]
    public void Cylinder_closed_form()
    {
        double r = 2, h = 3, m = Math.PI * r * r * h;
        Shape aCyl = TestShapes.Frozen(BRepGPropTestShapes.CylinderSolid(r, h));

        Properties aV = MassPropertiesBuilder.VolumeProperties(aCyl);
        Assert.Equal(m, aV.Mass(), 1e-9);
        AssertPnt(new Point3D(0, 0, h / 2), aV.CentreOfMass(), 1e-9);
        AssertMat(Diag(m * (3 * r * r + h * h) / 12, m * (3 * r * r + h * h) / 12, m * r * r / 2), aV.MatrixOfInertia(), 1e-8);
        Assert.True(aV.PrincipalProperties().HasSymmetryAxis());

        Properties aS = MassPropertiesBuilder.SurfaceProperties(aCyl);
        Assert.Equal(2 * Math.PI * r * r + 2 * Math.PI * r * h, aS.Mass(), 1e-9);
        AssertPnt(new Point3D(0, 0, h / 2), aS.CentreOfMass(), 1e-9);

        Properties aL = MassPropertiesBuilder.LinearProperties(aCyl, true);
        Assert.Equal(2 * 2 * Math.PI * r + h, aL.Mass(), 1e-9);
    }

    [Fact]
    public void Sphere_closed_form()
    {
        double r = 2, m = 4.0 / 3.0 * Math.PI * r * r * r, a = 4 * Math.PI * r * r;
        Shape aSphere = TestShapes.Frozen(BRepGPropTestShapes.SphereSolid(r));

        Properties aV = MassPropertiesBuilder.VolumeProperties(aSphere);
        Assert.Equal(m, aV.Mass(), 1e-9);
        AssertPnt(new Point3D(0, 0, 0), aV.CentreOfMass(), 1e-9);
        AssertMat(Diag(0.4 * m * r * r, 0.4 * m * r * r, 0.4 * m * r * r), aV.MatrixOfInertia(), 1e-8);
        Assert.True(aV.PrincipalProperties().HasSymmetryPoint());

        Properties aS = MassPropertiesBuilder.SurfaceProperties(aSphere);
        Assert.Equal(a, aS.Mass(), 1e-9);
        AssertMat(Diag(2.0 / 3.0 * a * r * r, 2.0 / 3.0 * a * r * r, 2.0 / 3.0 * a * r * r), aS.MatrixOfInertia(), 1e-8);
    }

    [Fact]
    public void Sphere_patches_have_their_zone_areas()
    {
        double r = 3;
        var aSphere = new SphericalSurface(new CoordinateSystem(new Point3D(1, 2, 3), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), r);

        // upper hemisphere: 2 Pi r^2, centroid r/2 above the centre
        Face aHemi = TestShapes.Frozen(BRepGPropTestShapes.NaturalFace(
            new TrimmedSurface(aSphere, 0, 2 * Math.PI, 0, Math.PI / 2)));
        Properties aS = MassPropertiesBuilder.SurfaceProperties(aHemi);
        Assert.Equal(2 * Math.PI * r * r, aS.Mass(), 1e-9);
        AssertPnt(new Point3D(1, 2, 3 + r / 2), aS.CentreOfMass(), 1e-9);

        // a quarter of the upper hemisphere: r^2 (Pi / 2) (sin(Pi/2) - sin(0))
        Face aPatch = TestShapes.Frozen(BRepGPropTestShapes.NaturalFace(
            new TrimmedSurface(aSphere, 0, Math.PI / 2, 0, Math.PI / 2)));
        Assert.Equal(r * r * Math.PI / 2, MassPropertiesBuilder.SurfaceProperties(aPatch).Mass(), 1e-9);

        // adaptive integration gives the same area within its error
        var aProps = new Properties();
        double anError = MassPropertiesBuilder.SurfaceProperties(aPatch, aProps, 1.0e-8);
        Assert.Equal(r * r * Math.PI / 2, aProps.Mass(), 1e-7);
        Assert.True(anError >= 0.0 && anError < 1e-6);
    }

    // Holed faces

    private static Face HoledSquare()
    {
        // a 10 x 10 square on z = 0 with a hole [2, 4] x [3, 7]
        var aB = new BrepBuilder();
        var aPlane = new PlaneSurface(new CoordinateSystem(new Point3D(), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)));
        Face aF = new();
        aB.MakeFace(ref aF, aPlane, 1e-7);
        (double, double)[][] aLoops =
        [
            [(0, 0), (10, 0), (10, 10), (0, 10)], // counter-clockwise
            [(2, 3), (2, 7), (4, 7), (4, 3)],     // clockwise
        ];
        foreach ((double, double)[] aLoop in aLoops)
        {
            var aV = aLoop.Select(p => TestShapes.Vertex(new Point3D(p.Item1, p.Item2, 0))).ToArray();
            Wire aW = new();
            aB.MakeWire(ref aW);
            for (int i = 0; i < aV.Length; i++)
            {
                Edge anE = TestShapes.Edge(aV[i], aV[(i + 1) % aV.Length]);
                (double x1, double y1) = aLoop[i];
                (double x2, double y2) = aLoop[(i + 1) % aLoop.Length];
                aB.UpdateEdge(anE, new LineCurve2D(new Point2D(x1, y1), new Direction2D(x2 - x1, y2 - y1)), aF, 1e-7);
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
        Face aF = HoledSquare();
        Properties aS = MassPropertiesBuilder.SurfaceProperties(aF);
        Assert.Equal(92.0, aS.Mass(), 1e-11);
        AssertPnt(new Point3D((100 * 5 - 8 * 3) / 92.0, (100 * 5 - 8 * 5) / 92.0, 0), aS.CentreOfMass(), 1e-11);

        Properties aL = MassPropertiesBuilder.LinearProperties(aF);
        Assert.Equal(40.0 + 12.0, aL.Mass(), 1e-11);

        // the same through the adaptive form
        var aProps = new Properties();
        MassPropertiesBuilder.SurfaceProperties(aF, aProps, 1e-9);
        Assert.Equal(92.0, aProps.Mass(), 1e-9);

        // reversing the face doesn't change its area
        Assert.Equal(92.0, MassPropertiesBuilder.SurfaceProperties(aF.Reversed()).Mass(), 1e-11);
    }

    // The adaptive and Gauss-Kronrod forms

    [Fact]
    public void Adaptive_volume_matches_the_closed_form()
    {
        double r = 2, h = 3;
        Shape aCyl = TestShapes.Frozen(BRepGPropTestShapes.CylinderSolid(r, h));
        var aProps = new Properties();
        double anError = MassPropertiesBuilder.VolumeProperties(aCyl, aProps, 1e-8);
        Assert.Equal(Math.PI * r * r * h, aProps.Mass(), 1e-7);
        Assert.True(anError >= 0.0 && anError < 1e-6, $"error {anError}");

        Shape aCube = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        MassPropertiesBuilder.VolumeProperties(aCube, aProps, 1e-6, true);
        Assert.Equal(1.0, aProps.Mass(), 1e-9);
    }

    [Fact]
    public void Gauss_Kronrod_volume_centre_and_inertia()
    {
        Shape aCube = TestShapes.Frozen(TestShapes.Box(1, 2, 3));
        var aProps = new Properties();
        double anError = MassPropertiesBuilder.VolumePropertiesGK(aCube, aProps, 1e-6, false, false, true, true);
        Assert.Equal(6.0, aProps.Mass(), 1e-9);
        AssertPnt(new Point3D(0.5, 1, 1.5), aProps.CentreOfMass(), 1e-9);
        AssertMat(Diag(6 * 13 / 12.0, 6 * 10 / 12.0, 6 * 5 / 12.0), aProps.MatrixOfInertia(), 1e-8);
        Assert.True(anError >= 0.0 && anError < 1e-5, $"error {anError}");

        double r = 2, h = 3;
        Shape aCyl = TestShapes.Frozen(BRepGPropTestShapes.CylinderSolid(r, h));
        MassPropertiesBuilder.VolumePropertiesGK(aCyl, aProps, 1e-8, true, true, true);
        Assert.Equal(Math.PI * r * r * h, aProps.Mass(), 1e-6);
        AssertPnt(new Point3D(0, 0, h / 2), aProps.CentreOfMass(), 1e-6);

        // the region between the faces of a closed shell and any plane is the enclosed volume
        var aPln = new Plane3D(new Point3D(0, 0, -5), new Direction3D(1, 2, 2));
        MassPropertiesBuilder.VolumePropertiesGK(aCube, aProps, aPln, 1e-6, false, false, true);
        Assert.Equal(6.0, aProps.Mass(), 1e-8);
        AssertPnt(new Point3D(0.5, 1, 1.5), aProps.CentreOfMass(), 1e-8);
    }

    [Fact]
    public void Only_closed_shells_count()
    {
        Solid aBox = TestShapes.Box(1, 1, 1);
        // an open shell: a box without its top face
        var aB = new BrepBuilder();
        Shell anOpen = new();
        aB.MakeShell(ref anOpen);
        List<Face> aFaces = BRepGPropTestShapes.Faces(TestShapes.Box(2, 2, 2));
        for (int i = 0; i < aFaces.Count; i++)
        {
            if (i != 1)
            {
                aB.Add(anOpen, aFaces[i]);
            }
        }

        Shape aC = TestShapes.Frozen(BRepGPropTestShapes.CompoundOf(aBox, BRepGPropTestShapes.SolidOf(anOpen)));
        Assert.Equal(1.0, MassPropertiesBuilder.VolumeProperties(aC, true).Mass(), 1e-12);

        var aProps = new Properties();
        Assert.True(MassPropertiesBuilder.VolumePropertiesGK(aC, aProps, 1e-6, true) >= 0.0);
        Assert.Equal(1.0, aProps.Mass(), 1e-9);

        // no closed shell: GK returns -1
        Assert.Equal(-1.0, MassPropertiesBuilder.VolumePropertiesGK(TestShapes.Frozen(anOpen), aProps, 1e-6, true));
    }

    // Meshes

    [Fact]
    public void Mesh_forms_measure_the_attached_triangulations()
    {
        Solid aBox = TestShapes.Box(1, 2, 3);
        BRepGPropTestShapes.AttachQuadMeshes(aBox);
        Shape aMeshed = TestShapes.Frozen(aBox);

        Properties aV = MassPropertiesBuilder.VolumeProperties(aMeshed, false, false, true);
        Assert.Equal(6.0, aV.Mass(), 1e-12);
        AssertPnt(new Point3D(0.5, 1, 1.5), aV.CentreOfMass(), 1e-12);
        AssertMat(Diag(6 * 13 / 12.0, 6 * 10 / 12.0, 6 * 5 / 12.0), aV.MatrixOfInertia(), 1e-11);

        Properties aS = MassPropertiesBuilder.SurfaceProperties(aMeshed, false, true);
        Assert.Equal(22.0, aS.Mass(), 1e-12);
        AssertPnt(new Point3D(0.5, 1, 1.5), aS.CentreOfMass(), 1e-12);

        // located: the rotation path, and a scale (the copy path)
        Placement aLoc = RigidLocation();
        Properties aVL = MassPropertiesBuilder.VolumeProperties(aMeshed.Moved(aLoc), false, false, true);
        Assert.Equal(6.0, aVL.Mass(), 1e-11);
        AssertPnt(new Point3D(0.5, 1, 1.5).Transformed(aLoc.Transformation), aVL.CentreOfMass(), 1e-11);
        Matrix3x3 aR = aLoc.Transformation.VectorialPart;
        AssertMat(aR.Multiplied(aV.MatrixOfInertia()).Multiplied(aR.Transposed()), aVL.MatrixOfInertia(), 1e-9);

        var aScale = new Placement(Transform3D.Scale(new Point3D(1, 1, 1), 2.0));
        Properties aVS = MassPropertiesBuilder.VolumeProperties(aMeshed.Moved(aScale), false, false, true);
        Assert.Equal(48.0, aVS.Mass(), 1e-11);
        AssertPnt(new Point3D(0, 1, 2), aVS.CentreOfMass(), 1e-11);
        Assert.Equal(88.0, MassPropertiesBuilder.SurfaceProperties(aMeshed.Moved(aScale), false, true).Mass(), 1e-11);
    }

    [Fact]
    public void Faces_without_surface_and_edges_without_curve_use_their_meshes()
    {
        Point3D[] aNodes = [new(0, 0, 0), new(1, 0, 0), new(0, 1, 0), new(0, 0, 1)];
        var aB = new BrepBuilder();
        Shell aShell = new();
        aB.MakeShell(ref aShell);
        foreach ((int, int, int) aTri in new[] { (1, 3, 2), (1, 2, 4), (1, 4, 3), (2, 3, 4) })
        {
            aB.Add(aShell, BRepGPropTestShapes.MeshFace(aNodes, aTri));
        }

        Shape aTetra = TestShapes.Frozen(BRepGPropTestShapes.SolidOf(aShell));
        Properties aV = MassPropertiesBuilder.VolumeProperties(aTetra);
        Assert.Equal(1.0 / 6.0, aV.Mass(), 1e-14);
        AssertPnt(new Point3D(0.25, 0.25, 0.25), aV.CentreOfMass(), 1e-14);
        Assert.Equal(1.5 + Math.Sqrt(3) / 2, MassPropertiesBuilder.SurfaceProperties(aTetra).Mass(), 1e-14);

        Edge aPolyline = TestShapes.Frozen(BRepGPropTestShapes.PolygonEdge(new(0, 0, 0), new(3, 0, 0), new(3, 4, 0)));
        Properties aL = MassPropertiesBuilder.LinearProperties(aPolyline);
        Assert.Equal(7.0, aL.Mass(), 1e-14);
        AssertPnt(new Point3D((3 * 1.5 + 4 * 3) / 7.0, 4 * 2 / 7.0, 0), aL.CentreOfMass(), 1e-14);
    }

    [Fact]
    public void MeshCinert_and_MeshProps_on_raw_data()
    {
        var aNodes = new NCollection_Array1<Point3D>(1, 3);
        aNodes[1] = new Point3D(0, 0, 0);
        aNodes[2] = new Point3D(0, 0, 0); // a null segment is skipped
        aNodes[3] = new Point3D(0, 0, 2);
        var aMC = new MeshCinert();
        aMC.SetLocation(new Point3D(1, 1, 1));
        aMC.Perform(aNodes);
        Assert.Equal(2.0, aMC.Mass(), 1e-15);
        AssertPnt(new Point3D(0, 0, 1), aMC.CentreOfMass(), 1e-15);
        // a rod of length 2 about its centre: m L^2 / 12 across it
        AssertMat(Diag(2 * 4 / 12.0, 2 * 4 / 12.0, 0), aMC.MatrixOfInertia(), 1e-14);

        // one triangle: area and centroid
        var aTri = new Triangulation(3, 1, false);
        aTri.SetNode(1, new Point3D(0, 0, 0));
        aTri.SetNode(2, new Point3D(3, 0, 0));
        aTri.SetNode(3, new Point3D(0, 3, 0));
        aTri.SetTriangle(1, new Triangle1(1, 2, 3));
        var aMP = new MeshProps(MeshProps.BRepGProp_MeshObjType.Sinert);
        aMP.SetLocation(new Point3D(0, 0, 0));
        aMP.Perform(aTri, Forward);
        Assert.Equal(4.5, aMP.Mass(), 1e-14);
        AssertPnt(new Point3D(1, 1, 0), aMP.CentreOfMass(), 1e-14);
        Assert.Equal(MeshProps.BRepGProp_MeshObjType.Sinert, aMP.GetMeshObjType());

        // a pyramid from (0, 0, 1) on it: volume base * height / 3, negative seen from above when reversed
        var aMV = new MeshProps(MeshProps.BRepGProp_MeshObjType.Vinert);
        aMV.SetLocation(new Point3D(0, 0, 1));
        aMV.Perform(aTri, Reversed);
        Assert.Equal(1.5, aMV.Mass(), 1e-14);
        aMV.Perform(aTri, Forward);
        Assert.Equal(-1.5, aMV.Mass(), 1e-14);

        // an empty or null mesh leaves the system as it is
        aMV.Perform(null, Forward);
        Assert.Equal(-1.5, aMV.Mass(), 1e-14);

        // a degenerate triangle adds nothing
        Span<double> aGProps = stackalloc double[10];
        MeshProps.CalculateProps(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(2, 0, 0), new Point3D(0, 0, 0),
                                           false, aGProps, 1, [1.0 / 3.0, 1.0 / 3.0, 0.5]);
        Assert.Equal(0.0, aGProps[0]);
    }

    // The tools

    [Fact]
    public void Face_tool_bounds_normals_orders_and_knots()
    {
        Shape aCyl = TestShapes.Frozen(BRepGPropTestShapes.CylinderSolid(2, 3));
        Face aLateral = BRepGPropTestShapes.Faces(aCyl)[0];
        var aBF = new FaceIntegrand(aLateral);
        aBF.Bounds(out double u1, out double u2, out double v1, out double v2);
        Assert.Equal(0.0, u1, 1e-12);
        Assert.Equal(2 * Math.PI, u2, 1e-12);
        Assert.Equal(0.0, v1, 1e-12);
        Assert.Equal(3.0, v2, 1e-12);
        aBF.Normal(0.0, 1.0, out Point3D aP, out Vector3D aN);
        AssertPnt(new Point3D(2, 0, 1), aP, 1e-12);
        Assert.True(aN.X > 0); // outward, length r
        Assert.Equal(2.0, aN.Magnitude, 1e-12);
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
        var aBR = new FaceIntegrand(ShapeCasts.Face(aLateral.Reversed()));
        aBR.Normal(0.0, 1.0, out _, out Vector3D aNR);
        Assert.True(aNR.X < 0);

        // the boundary arcs: the edges' pcurves, and the sides of the UV rectangle
        var aBD = new Domain(aLateral);
        int aNb = 0;
        for (; aBD.More(); aBD.Next())
        {
            Assert.True(aBF.Load(aBD.Value()));
            Assert.True(aBF.IntegrationOrder() >= 4);
            aNb++;
        }

        Assert.Equal(4, aNb);
        aBF.Load(true, IsoU);
        Assert.Equal(0.0, aBF.FirstParameter());
        Assert.Equal(3.0, aBF.LastParameter(), 1e-15);
        AssertPnt(new Point3D(0, 3, 0), new Point3D(aBF.Value2d(0).X, aBF.Value2d(0).Y, 0), 1e-15);
        NCollection_HArray1<double> aU = aBF.GetUKnots(0.5, 1.5);
        Assert.Equal(2, aU.Length());
        Assert.Equal(1.5, aU[2]);

        // an edge without a pcurve on the face isn't loaded
        Edge aStray = TestShapes.Frozen(TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(1, 0, 0)));
        Assert.False(aBF.Load(aStray));
    }

    [Fact]
    public void Domain_skips_internal_and_external_edges_after_the_first()
    {
        // a square face whose wire also holds an INTERNAL edge
        Face aSquare = BRepGPropTestShapes.Faces(TestShapes.Box(1, 1, 1))[0];
        var aB = new BrepBuilder();
        var aPlane = (PlaneSurface)Tool.Surface(aSquare)!;
        Face aF = new();
        aB.MakeFace(ref aF, aPlane, 1e-7);
        Wire aW = new();
        aB.MakeWire(ref aW);
        Edge anInternal = TestShapes.Edge(new Point3D(0.2, 0.2, 0), new Point3D(0.8, 0.8, 0));
        aB.Add(aW, anInternal.Oriented(Internal));
        for (var anExp = new Explorer(aSquare, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            aB.Add(aW, anExp.Current());
        }

        aB.Add(aW, anInternal.Oriented(External));
        aB.Add(aF, aW);
        Face aFace = TestShapes.Frozen(aF);

        var aD = new Domain(aFace);
        var anOrientations = new List<Orientation>();
        for (; aD.More(); aD.Next())
        {
            anOrientations.Add(aD.Value().Orientation);
        }

        // OCCT's Domain checks only from Next: the first edge comes even if INTERNAL
        Assert.Equal(5, anOrientations.Count);
        Assert.Equal(Internal, anOrientations[0]);
        Assert.DoesNotContain(External, anOrientations);

        aD.Init();
        Assert.True(aD.More());
    }

    [Fact]
    public void Vinert_by_plane_is_the_prism_down_to_the_plane()
    {
        // The region between the box's top face and the plane z = 0 is the unit cube, whatever the location point.
        // OCCT's BRepGProp_Gauss::convert cleared the mass and centre of a region bounded by a plane (occt-issues.md,
        // decision 1468). The central inertia is a cube's, m (b^2 + c^2) / 12.
        Face aTop = BRepGPropTestShapes.Faces(TestShapes.Frozen(TestShapes.Box(1, 1, 1)))[1];
        var aBF = new FaceIntegrand(aTop);
        var aBD = new Domain(aTop);
        var aPln = new Plane3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        var aV = new Vinert(aBF, aBD, aPln, new Point3D(0, 0, 0));
        Assert.Equal(1.0, aV.Mass(), 1e-12);
        AssertPnt(new Point3D(0.5, 0.5, 0.5), aV.CentreOfMass(), 1e-12);
        AssertMat(Diag(1.0 / 6.0, 1.0 / 6.0, 1.0 / 6.0), aV.MatrixOfInertia(), 1e-12);

        aBD.Init();
        var aV2 = new Vinert(aBF, aBD, aPln, new Point3D(0.2, 0.1, -0.5));
        Assert.Equal(1.0, aV2.Mass(), 1e-12);
        AssertPnt(new Point3D(0.5, 0.5, 0.5), aV2.CentreOfMass(), 1e-12);
        AssertMat(Diag(1.0 / 6.0, 1.0 / 6.0, 1.0 / 6.0), aV2.MatrixOfInertia(), 1e-12);

        // by point it is the pyramid on the face: base 1, height 1
        aBD.Init();
        var aVP = new Vinert(aBF, aBD, new Point3D(0, 0, 0));
        Assert.Equal(1.0 / 3.0, aVP.Mass(), 1e-14);

        // the Gauss-Kronrod form by plane is right: the prism down to z = 0
        aBD.Init();
        var aGK = new VinertGK(aBF, aBD, aPln, new Point3D(0, 0, 0), 1e-8, true);
        Assert.Equal(1.0, aGK.Mass(), 1e-9);
        AssertPnt(new Point3D(0.5, 0.5, 0.5), aGK.CentreOfMass(), 1e-9);
        Assert.Equal(aGK.GetErrorReached() * aGK.Mass(), aGK.GetAbsolutError(), 1e-20);
    }

    [Fact]
    public void Natural_faces_split_V_at_their_V_bounds()
    {
        // The V knots of a natural plane, cylinder or cone face are its V bounds (OCCT gave the U bounds:
        // occt-issues.md, decision 1469), and the adaptive area is the closed form's
        var aCyl = new CylindricalSurface(new CoordinateSystem(new Point3D(1, 2, 3), new Direction3D(0, 0, 1)), 2.0);
        Face aF = BRepGPropTestShapes.NaturalFace(new TrimmedSurface(aCyl, 0.0, 2 * Math.PI, -3.0, 9.0));
        var aBF = new FaceIntegrand(aF);
        Assert.True(aBF.NaturalRestriction());
        var aKnots = new NCollection_Array1<double>(1, aBF.SVIntSubs() + 1);
        aBF.VKnots(aKnots);
        Assert.Equal(-3.0, aKnots[1]);
        Assert.Equal(9.0, aKnots[aKnots.Upper()]);

        var aProps = new Properties(new Point3D(0, 0, 0));
        MassPropertiesBuilder.SurfaceProperties(aF, aProps, 1e-9);
        Assert.Equal(2 * Math.PI * 2.0 * 12.0, aProps.Mass(), 1e-8);
        AssertPnt(new Point3D(1, 2, 6), aProps.CentreOfMass(), 1e-8);

        var aCone = new ConicalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), Math.PI / 6, 1.0);
        Face aCF = BRepGPropTestShapes.NaturalFace(new TrimmedSurface(aCone, 0.0, 2 * Math.PI, 0.5, 4.0));
        var aCKnots = new NCollection_Array1<double>(1, new FaceIntegrand(aCF).SVIntSubs() + 1);
        new FaceIntegrand(aCF).VKnots(aCKnots);
        Assert.Equal(0.5, aCKnots[1]);
        Assert.Equal(4.0, aCKnots[aCKnots.Upper()]);
        // a frustum's lateral area: pi (r1 + r2) s, with r = 1 + v sin(pi/6) and slant s = 3.5
        var aCProps = new Properties(new Point3D(0, 0, 0));
        MassPropertiesBuilder.SurfaceProperties(aCF, aCProps, 1e-9);
        Assert.Equal(Math.PI * (1.25 + 3.0) * 3.5, aCProps.Mass(), 1e-8);
    }

    [Fact]
    public void Gauss_asserts_the_type()
    {
        Face aF = BRepGPropTestShapes.Faces(TestShapes.Frozen(TestShapes.Box(1, 1, 1)))[0];
        var aBF = new FaceIntegrand(aF);
        var aGauss = new MeasurementGauss(MeasurementGauss.BRepGProp_GaussType.Vinert);
        double aMass = 0;
        Point3D aG = new();
        Matrix3x3 anI = new();
        EngineAssert.Throws<ProgramErrorException>(() => aGauss.Compute(aBF, new Point3D(), ref aMass, ref aG, ref anI));
    }
}
