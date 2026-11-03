// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>NLPlate_NLPlate and its constraints.</summary>
public class NLPlate_NLPlateTests
{
    private static readonly Xyz s_zero = new(0.0, 0.0, 0.0);

    private static Geom_Plane XYPlane() => new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));

    private static void AssertNear(in Xyz theExpected, in Xyz theActual, double theTol)
    {
        Assert.True(theExpected.IsEqual(theActual, theTol), $"expected {theExpected}, got {theActual}");
    }

    [Fact]
    public void Constraints_ClampTheirActiveOrder()
    {
        var p = new Xy(0.1, 0.2);
        var aD1 = new Plate_D1(new Xyz(1.0, 0.0, 0.0), new Xyz(0.0, 1.0, 0.0));
        var aD2 = new Plate_D2(s_zero, s_zero, s_zero);
        var aD3 = new Plate_D3(s_zero, s_zero, s_zero, new Xyz(0.0, 0.0, 1.0));
        NLPlate_HGPPConstraint[] aAll =
        [
            new NLPlate_HPG0Constraint(p, new Xyz(1.0, 2.0, 3.0)),
            new NLPlate_HPG0G1Constraint(p, s_zero, aD1),
            new NLPlate_HPG0G2Constraint(p, s_zero, aD1, aD2),
            new NLPlate_HPG0G3Constraint(p, s_zero, aD1, aD2, aD3),
            new NLPlate_HPG1Constraint(p, aD1),
            new NLPlate_HPG2Constraint(p, aD1, aD2),
            new NLPlate_HPG3Constraint(p, aD1, aD2, aD3),
        ];
        int[] aOrders = [0, 1, 2, 3, 1, 2, 3];
        for (int i = 0; i < aAll.Length; i++)
        {
            Assert.Equal(aOrders[i], aAll[i].ActiveOrder());
            Assert.Equal(p, aAll[i].UV());
            Assert.Equal(i < 4, aAll[i].IsG0());
            aAll[i].SetActiveOrder(-1);
            Assert.Equal(-1, aAll[i].ActiveOrder());
            aAll[i].SetActiveOrder(7);
            Assert.Equal(aOrders[i], aAll[i].ActiveOrder());
        }
        Assert.Equal(new Xyz(1.0, 2.0, 3.0), aAll[0].G0Target());
        Assert.Equal(aD1, aAll[1].G1Target());
        Assert.Equal(aD2, aAll[5].G2Target());
        Assert.Equal(aD3, aAll[6].G3Target());
        Assert.Equal(aD3, aAll[3].G3Target());

        // the base's defaults
        Assert.Equal(s_zero, aAll[4].G0Target());
        Assert.Equal(new Plate_D1(s_zero, s_zero), aAll[0].G1Target());
        Assert.Equal(0.0, aAll[0].G0Criterion());
        Assert.Equal(0, aAll[0].Orientation());
        aAll[0].SetOrientation(1);
        Assert.Equal(0, aAll[0].Orientation());
        aAll[1].SetOrientation(-1);
        Assert.Equal(-1, aAll[1].Orientation());
        aAll[4].SetOrientation(1);
        Assert.Equal(1, aAll[4].Orientation());

        Assert.False(aAll[0].UVFreeSliding());
        aAll[0].SetUVFreeSliding(true);
        Assert.True(aAll[0].UVFreeSliding());
        aAll[4].SetUVFreeSliding(true);
        Assert.False(aAll[4].UVFreeSliding()); // G1 constraints don't slide
        aAll[4].SetIncrementalLoadAllowed(true);
        Assert.True(aAll[4].IncrementalLoadAllowed());
    }

    [Fact]
    public void Solve_DeformsThePlaneThroughTheTargets()
    {
        var aPlate = new NLPlate_NLPlate(XYPlane());
        var aTargets = new (Xy UV, Xyz P)[]
        {
            (new Xy(0.0, 0.0), new Xyz(0.0, 0.0, 0.0)),
            (new Xy(1.0, 0.0), new Xyz(1.0, 0.0, 0.2)),
            (new Xy(0.0, 1.0), new Xyz(0.0, 1.0, -0.1)),
            (new Xy(1.0, 1.0), new Xyz(1.0, 1.0, 0.0)),
            (new Xy(0.5, 0.5), new Xyz(0.5, 0.5, 0.4)),
        };
        foreach (var (uv, pnt) in aTargets)
        {
            aPlate.Load(new NLPlate_HPG0Constraint(uv, pnt));
        }
        aPlate.Load(null);
        Assert.Equal(0, aPlate.MaxActiveConstraintOrder());
        aPlate.Solve(2, 0);
        Assert.True(aPlate.IsDone());
        foreach (var (uv, pnt) in aTargets)
        {
            AssertNear(pnt, aPlate.Evaluate(uv), 1.0e-9);
        }
        Assert.Equal(1, aPlate.Continuity()); // 2 * 2 - 3 - 0: the plane is C-infinite (OCCT stops counting at 10)

        aPlate.Init();
        Assert.Equal(-1, aPlate.MaxActiveConstraintOrder());
        Assert.Equal(new Xyz(0.5, 0.5, 0.0), aPlate.Evaluate(new Xy(0.5, 0.5)));
    }

    [Fact]
    public void Solve_WithG1Targets_MakesTheSurfaceTangent()
    {
        var aPlate = new NLPlate_NLPlate(XYPlane());
        aPlate.Load(new NLPlate_HPG0Constraint(new Xy(0.0, 0.0), new Xyz(0.0, 0.0, 0.0)));
        aPlate.Load(new NLPlate_HPG0Constraint(new Xy(1.0, 0.0), new Xyz(1.0, 0.0, 0.0)));
        aPlate.Load(new NLPlate_HPG0Constraint(new Xy(0.0, 1.0), new Xyz(0.0, 1.0, 0.0)));
        var aD1 = new Plate_D1(new Xyz(1.0, 0.0, 0.3), new Xyz(0.0, 1.0, 0.1));
        var p = new Xy(0.6, 0.6);
        aPlate.Load(new NLPlate_HPG0G1Constraint(p, new Xyz(0.6, 0.6, 0.2), aD1));
        Assert.Equal(1, aPlate.MaxActiveConstraintOrder());
        aPlate.Solve(3, 0);
        Assert.True(aPlate.IsDone());
        AssertNear(new Xyz(0.6, 0.6, 0.2), aPlate.Evaluate(p), 1.0e-6);
        Xyz aN = (aD1.DU() ^ aD1.DV()).Normalized();
        Assert.Equal(0.0, aPlate.EvaluateDerivative(p, 1, 0) * aN, 1.0e-6);
        Assert.Equal(0.0, aPlate.EvaluateDerivative(p, 0, 1) * aN, 1.0e-6);
    }

    [Fact]
    public void IncrementalSolve_ReachesTheTargets()
    {
        var aPlate = new NLPlate_NLPlate(XYPlane());
        foreach (Xy uv in new Xy[] { new(0.0, 0.0), new(1.0, 0.0), new(0.0, 1.0), new(1.0, 1.0) })
        {
            var c = new NLPlate_HPG0Constraint(uv, new Xyz(uv.X(), uv.Y(), 0.0));
            c.SetIncrementalLoadAllowed(true);
            aPlate.Load(c);
        }
        var aD1 = new Plate_D1(new Xyz(1.0, 0.0, 0.5), new Xyz(0.0, 1.0, 0.0));
        var aG1 = new NLPlate_HPG0G1Constraint(new Xy(0.5, 0.5), new Xyz(0.5, 0.5, 0.3), aD1);
        aG1.SetIncrementalLoadAllowed(true);
        aPlate.Load(aG1);
        aPlate.IncrementalSolve(3, 1, 4);
        Assert.True(aPlate.IsDone());
        AssertNear(new Xyz(0.5, 0.5, 0.3), aPlate.Evaluate(new Xy(0.5, 0.5)), 1.0e-6);
        Xyz aN = (aD1.DU() ^ aD1.DV()).Normalized();
        Assert.Equal(0.0, aPlate.EvaluateDerivative(new Xy(0.5, 0.5), 1, 0) * aN, 1.0e-6);
    }

    [Fact]
    public void ConstraintsSliding_MovesTheUVOfASlidingConstraint()
    {
        var aPlate = new NLPlate_NLPlate(XYPlane());
        // a target on the plane, set at the wrong UV: sliding finds the UV of its foot
        var aSliding = new NLPlate_HPG0Constraint(new Xy(0.2, 0.3), new Xyz(0.7, 0.4, 0.0));
        aSliding.SetUVFreeSliding(true);
        aPlate.Load(aSliding);
        var aFixed = new NLPlate_HPG0Constraint(new Xy(0.9, 0.9), new Xyz(0.9, 0.9, 0.0));
        aPlate.Load(aFixed);
        aPlate.ConstraintsSliding();
        Assert.True(new Xy(0.7, 0.4).IsEqual(aSliding.UV(), 1.0e-12));
        Assert.Equal(new Xy(0.9, 0.9), aFixed.UV());
    }

    [Fact]
    public void Solve2_SolvesThePolynomialPartFirst()
    {
        var aPlate = new NLPlate_NLPlate(XYPlane());
        foreach (Xy uv in new Xy[] { new(0.0, 0.0), new(1.0, 0.0), new(0.0, 1.0), new(1.0, 1.0), new(0.4, 0.7) })
        {
            aPlate.Load(new NLPlate_HPG0Constraint(uv, new Xyz(uv.X(), uv.Y(), 0.3 * uv.X() * uv.Y())));
        }
        aPlate.Solve2(2, 0);
        Assert.True(aPlate.IsDone());
        AssertNear(new Xyz(0.4, 0.7, 0.3 * 0.4 * 0.7), aPlate.Evaluate(new Xy(0.4, 0.7)), 1.0e-9);
    }

    [Fact]
    public void Solve_ReportsDoneEvenWhenAnIterationFails()
    {
        // OCCT's Solve sets OK after its loop, whatever the loop found (occt-issues.md). A single constraint gives a
        // flat UV box, which Plate_Plate widens; two constraints at one UV make the matrix singular.
        var aPlate = new NLPlate_NLPlate(XYPlane());
        aPlate.Load(new NLPlate_HPG0Constraint(new Xy(0.5, 0.5), new Xyz(0.0, 0.0, 1.0)));
        aPlate.Load(new NLPlate_HPG0Constraint(new Xy(0.5, 0.5), new Xyz(0.0, 0.0, 2.0)));
        aPlate.Load(new NLPlate_HPG0G1Constraint(new Xy(0.5, 0.5), new Xyz(0.0, 0.0, 3.0),
                                                 new Plate_D1(new Xyz(1.0, 0.0, 0.0), new Xyz(0.0, 1.0, 0.0))));
        aPlate.Solve(2, 0);
        Assert.True(aPlate.IsDone());
        // no iteration succeeded: the surface is the initial plane
        Assert.Equal(new Xyz(0.5, 0.5, 0.0), aPlate.Evaluate(new Xy(0.5, 0.5)));
    }
}
