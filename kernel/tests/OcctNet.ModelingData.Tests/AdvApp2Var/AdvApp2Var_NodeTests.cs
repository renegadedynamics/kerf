// Port of: src/ModelingData/TKGeomBase/GTests/AdvApp2Var_Node_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation.Advanced;
using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace OcctNet.ModelingData.Tests;

public class AdvApp2Var_NodeTest
{
    [Fact]
    public void Constructor_InitializesValuesToZero()
    {
        Node aNode = new(1, 2);
        for (int aUOrder = 0; aUOrder <= 1; ++aUOrder)
        {
            for (int aVOrder = 0; aVOrder <= 2; ++aVOrder)
            {
                Point3D aPoint = aNode.Point(aUOrder, aVOrder);
                Assert.Equal(0.0, aPoint.X());
                Assert.Equal(0.0, aPoint.Y());
                Assert.Equal(0.0, aPoint.Z());
                Assert.Equal(0.0, aNode.Error(aUOrder, aVOrder));
            }
        }
    }

    // The port: C++'s operator= is Assign, which copies the values (and resizes the arrays as OCCT's do).
    [Fact]
    public void Assign_CopiesValuesNotArrays()
    {
        Node aSource = new(new Xy(1.5, -2.0), 1, 1);
        aSource.SetPoint(1, 0, new Point3D(1, 2, 3));
        aSource.SetError(0, 1, 0.25);
        Node aTarget = new(0, 0);
        aTarget.Assign(aSource);
        Assert.Equal(1.5, aTarget.Coord().X());
        Assert.Equal(1, aTarget.UOrder());
        Assert.Equal(2.0, aTarget.Point(1, 0).Y());
        Assert.Equal(0.25, aTarget.Error(0, 1));
        aSource.SetPoint(1, 0, new Point3D(7, 7, 7));
        Assert.Equal(2.0, aTarget.Point(1, 0).Y());
    }
}
