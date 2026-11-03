// Port of: src/ModelingData/TKGeomBase/GTests/GProp_PGProps_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Measurement;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

public class GProp_PGPropsTest
{
    private static readonly double Conf = global::Kerf.Engine.Foundation.Precision.Confusion();

    [Fact]
    public void EmptySet()
    {
        var aProps = new PGProps();
        Assert.Equal(0.0, aProps.Mass(), Conf);
    }

    [Fact]
    public void SinglePoint()
    {
        var aProps = new PGProps();
        aProps.AddPoint(new Point3D(1.0, 2.0, 3.0));

        Assert.Equal(1.0, aProps.Mass(), Conf);
        Assert.Equal(1.0, aProps.CentreOfMass().X(), Conf);
        Assert.Equal(2.0, aProps.CentreOfMass().Y(), Conf);
        Assert.Equal(3.0, aProps.CentreOfMass().Z(), Conf);
    }

    [Fact]
    public void TwoPoints_Barycentre()
    {
        var aProps = new PGProps();
        aProps.AddPoint(new Point3D(0.0, 0.0, 0.0));
        aProps.AddPoint(new Point3D(2.0, 4.0, 6.0));

        Assert.Equal(2.0, aProps.Mass(), Conf);
        Assert.Equal(1.0, aProps.CentreOfMass().X(), Conf);
        Assert.Equal(2.0, aProps.CentreOfMass().Y(), Conf);
        Assert.Equal(3.0, aProps.CentreOfMass().Z(), Conf);
    }

    [Fact]
    public void WeightedPoints()
    {
        var aProps = new PGProps();
        aProps.AddPoint(new Point3D(0.0, 0.0, 0.0), 1.0);
        aProps.AddPoint(new Point3D(4.0, 0.0, 0.0), 3.0);

        Assert.Equal(4.0, aProps.Mass(), Conf);
        // Weighted centroid: (0*1 + 4*3) / 4 = 3
        Assert.Equal(3.0, aProps.CentreOfMass().X(), Conf);
        Assert.Equal(0.0, aProps.CentreOfMass().Y(), Conf);
        Assert.Equal(0.0, aProps.CentreOfMass().Z(), Conf);
    }

    [Fact]
    public void ArrayConstructor()
    {
        var aPnts = new NCollection_Array1<Point3D>(1, 4);
        aPnts[1] = new Point3D(1.0, 0.0, 0.0);
        aPnts[2] = new Point3D(-1.0, 0.0, 0.0);
        aPnts[3] = new Point3D(0.0, 1.0, 0.0);
        aPnts[4] = new Point3D(0.0, -1.0, 0.0);

        var aProps = new PGProps(aPnts);
        Assert.Equal(4.0, aProps.Mass(), Conf);
        Assert.Equal(0.0, aProps.CentreOfMass().X(), Conf);
        Assert.Equal(0.0, aProps.CentreOfMass().Y(), Conf);
        Assert.Equal(0.0, aProps.CentreOfMass().Z(), Conf);
    }

    [Fact]
    public void StaticBarycentre()
    {
        var aPnts = new NCollection_Array1<Point3D>(1, 3);
        aPnts[1] = new Point3D(0.0, 0.0, 0.0);
        aPnts[2] = new Point3D(3.0, 0.0, 0.0);
        aPnts[3] = new Point3D(0.0, 6.0, 0.0);

        Point3D aG = PGProps.Barycentre(aPnts);
        Assert.Equal(1.0, aG.X(), Conf);
        Assert.Equal(2.0, aG.Y(), Conf);
        Assert.Equal(0.0, aG.Z(), Conf);
    }

    [Fact]
    public void MatrixOfInertia_SymmetricPoints()
    {
        // Centroid is at the origin, so origin-frame and central-frame inertia
        // matrices coincide for these symmetric points.
        var aPnts = new NCollection_Array1<Point3D>(1, 4);
        aPnts[1] = new Point3D(1.0, 0.0, 0.0);
        aPnts[2] = new Point3D(-1.0, 0.0, 0.0);
        aPnts[3] = new Point3D(0.0, 1.0, 0.0);
        aPnts[4] = new Point3D(0.0, -1.0, 0.0);

        var aProps = new PGProps(aPnts);
        Matrix3x3 anInertia = aProps.MatrixOfInertia();

        Assert.Equal(2.0, anInertia.Value(1, 1), Conf); // Ixx
        Assert.Equal(2.0, anInertia.Value(2, 2), Conf); // Iyy
        Assert.Equal(4.0, anInertia.Value(3, 3), Conf); // Izz
        Assert.Equal(0.0, anInertia.Value(1, 2), Conf); // Ixy
        Assert.Equal(0.0, anInertia.Value(1, 3), Conf); // Ixz
        Assert.Equal(0.0, anInertia.Value(2, 3), Conf); // Iyz
    }
}
