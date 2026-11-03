// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Intersection.Implicit;
using Kerf.Geometry;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>Known answers of IntImp_ComputeTangence.</summary>
public class IntImp_ComputeTangenceTests
{
    [Fact]
    public void ChoixRefIsTheDefaultOrder()
    {
        Assert.Equal(ConstIsoparametric.UIsoparametricOnCaro1, Tangence.ChoixRef(0));
        Assert.Equal(ConstIsoparametric.VIsoparametricOnCaro2, Tangence.ChoixRef(3));
        Assert.ThrowsAny<OutOfRangeException>(() => Tangence.ChoixRef(4));
    }

    [Fact]
    public void TangentPlanesAreTangent()
    {
        Vector3D[] aDPuv = [new(1, 0, 0), new(0, 1, 0), new(0, 2, 0), new(3, 0, 0)];
        double[] anEps = [1e-9, 1e-9, 1e-9, 1e-9];
        double[] aTg = new double[4];
        ConstIsoparametric[] aIso = new ConstIsoparametric[4];
        Assert.True(Tangence.ComputeTangence(aDPuv, anEps, aTg, aIso));

        // A vanishing derivative counts as tangent
        Vector3D[] aDegenerate = [new(0, 0, 0), new(0, 1, 0), new(0, 2, 0), new(3, 0, 0)];
        Assert.True(Tangence.ComputeTangence(aDegenerate, anEps, aTg, aIso));
    }

    [Fact]
    public void CrossingPlanesSortTheIsoparametrics()
    {
        // Surface 1: the plane z = 0 (u along X, v along Y); surface 2: the plane x = 0 (u along Y, v along Z).
        // The intersection is the Y axis: along it u of surface 1 is constant (an iso u), v of surface 2 is constant.
        Vector3D[] aDPuv = [new(1, 0, 0), new(0, 1, 0), new(0, 1, 0), new(0, 0, 1)];
        double[] anEps = [1e-9, 1e-9, 1e-9, 1e-9];
        double[] aTg = new double[4];
        ConstIsoparametric[] aIso = new ConstIsoparametric[4];
        Assert.False(Tangence.ComputeTangence(aDPuv, anEps, aTg, aIso));

        // The tangent to the intersection along the Y axis
        Assert.Equal(0.0, aTg[0], 1e-15);
        Assert.Equal(1.0, Math.Abs(aTg[1]), 1e-15);
        Assert.Equal(1.0, Math.Abs(aTg[2]), 1e-15);
        Assert.Equal(0.0, aTg[3], 1e-15);

        // Sorted by increasing |component| / |derivative| (1, 0, 0, 1 for U1, V1, U2, V2), stably by the bubble sort
        Assert.Equal(ConstIsoparametric.VIsoparametricOnCaro1, aIso[0]);
        Assert.Equal(ConstIsoparametric.UIsoparametricOnCaro2, aIso[1]);
        Assert.Equal(ConstIsoparametric.UIsoparametricOnCaro1, aIso[2]);
        Assert.Equal(ConstIsoparametric.VIsoparametricOnCaro2, aIso[3]);
    }
}
