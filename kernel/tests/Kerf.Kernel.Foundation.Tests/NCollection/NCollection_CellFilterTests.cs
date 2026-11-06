// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;

namespace Kerf.Kernel.Foundation.Tests;

// OCCT has no GTest for NCollection_CellFilter; these check the port's behaviour on 2D points.
public class NCollection_CellFilterTest
{
    private sealed class PointInspector : ICellFilterInspector<int, double[]>
    {
        public static double Coord(int i, double[] thePnt) => thePnt[i];

        public static bool IsEqual(int theT1, int theT2) => theT1 == theT2;

        public List<int> Found { get; } = [];

        public bool PurgeAll { get; set; }

        public CellFilterAction Inspect(int theObject)
        {
            Found.Add(theObject);
            return PurgeAll ? CellFilterAction.CellFilter_Purge : CellFilterAction.CellFilter_Keep;
        }
    }

    [Fact]
    public void AddAndInspect_FindsTargetsInCoveredCells()
    {
        CellFilter<PointInspector, int, double[]> aFilter = new(2, 1.0);
        aFilter.Add(1, [0.5, 0.5]);
        aFilter.Add(2, [0.7, 0.2]);
        aFilter.Add(3, [5.5, 5.5]);

        PointInspector anInspector = new();
        aFilter.Inspect([0.1, 0.1], ref anInspector);
        // Newest first within a cell, as OCCT's list
        Assert.Equal([2, 1], anInspector.Found);

        anInspector = new PointInspector();
        aFilter.Inspect([0.1, 0.1], [6.0, 6.0], ref anInspector);
        Assert.Equal(3, anInspector.Found.Count);
    }

    [Fact]
    public void A_window_of_cells_changes_nothing_a_caller_can_see()
    {
        // The same random adds, removes, purges and inspections, with a window and without; some fall outside the window,
        // and negative coordinates share a cell with small positive ones, as the indexes truncate towards zero
        Random aRandom = new(20261106);
        CellFilter<PointInspector, int, double[]> aPlain = new(2, 0.5);
        CellFilter<PointInspector, int, double[]> aWindowed = new(2, 0.5);
        aWindowed.UseWindow([-3.0, -2.0], [4.0, 3.0]);
        double Coord() => aRandom.NextDouble() * 10.0 - 5.0;
        for (int i = 0; i < 20000; i++)
        {
            double[] aMin = [Coord(), Coord()];
            double[] aMax = [aMin[0] + aRandom.NextDouble() * 2.0, aMin[1] + aRandom.NextDouble() * 2.0];
            switch (aRandom.Next(5))
            {
                case 0:
                case 1:
                    aPlain.Add(i, aMin, aMax);
                    aWindowed.Add(i, aMin, aMax);
                    break;
                case 2:
                    int aTarget = aRandom.Next(Math.Max(1, i));
                    aPlain.Remove(aTarget, aMin, aMax);
                    aWindowed.Remove(aTarget, aMin, aMax);
                    break;
                default:
                    bool isPurge = aRandom.Next(8) == 0;
                    PointInspector aOne = new() { PurgeAll = isPurge }, aTwo = new() { PurgeAll = isPurge };
                    aPlain.Inspect(aMin, aMax, ref aOne);
                    aWindowed.Inspect(aMin, aMax, ref aTwo);
                    Assert.Equal(aOne.Found, aTwo.Found);
                    break;
            }
        }
    }

    [Fact]
    public void A_reset_drops_the_window_and_its_cells()
    {
        CellFilter<PointInspector, int, double[]> aFilter = new(2, 1.0);
        aFilter.UseWindow([0.0, 0.0], [9.0, 9.0]);
        aFilter.Add(1, [2.5, 2.5]);
        aFilter.Reset(1.0);
        PointInspector anInspector = new();
        aFilter.Inspect([2.5, 2.5], ref anInspector);
        Assert.Empty(anInspector.Found);
        aFilter.Add(2, [2.5, 2.5]);
        aFilter.Inspect([2.5, 2.5], ref anInspector);
        Assert.Equal([2], anInspector.Found);
    }

    [Fact]
    public void AddRange_PutsTargetInEveryCell()
    {
        CellFilter<PointInspector, int, double[]> aFilter = new(2, 1.0);
        aFilter.Add(7, [0.5, 0.5], [2.5, 1.5]);

        PointInspector anInspector = new();
        aFilter.Inspect([2.2, 1.2], ref anInspector);
        Assert.Equal([7], anInspector.Found);

        anInspector = new PointInspector();
        aFilter.Inspect([3.2, 1.2], ref anInspector);
        Assert.Empty(anInspector.Found);
    }

    [Fact]
    public void RemoveAndPurge_RemoveTargets()
    {
        CellFilter<PointInspector, int, double[]> aFilter = new(2, 1.0);
        aFilter.Add(1, [0.5, 0.5]);
        aFilter.Add(2, [0.6, 0.6]);
        aFilter.Remove(1, [0.5, 0.5]);

        PointInspector anInspector = new() { PurgeAll = true };
        aFilter.Inspect([0.5, 0.5], ref anInspector);
        Assert.Equal([2], anInspector.Found);

        anInspector = new PointInspector();
        aFilter.Inspect([0.5, 0.5], ref anInspector);
        Assert.Empty(anInspector.Found);
    }

    [Fact]
    public void Reset_ClearsAndChangesCellSize()
    {
        CellFilter<PointInspector, int, double[]> aFilter = new(2, 1.0);
        aFilter.Add(1, [0.5, 0.5]);

        NCollection_Array1<double> aSizes = new(0, 1);
        aSizes[0] = 10.0;
        aSizes[1] = 10.0;
        aFilter.Reset(aSizes);
        aFilter.Add(2, [5.0, 5.0]);

        PointInspector anInspector = new();
        aFilter.Inspect([0.5, 0.5], ref anInspector);
        Assert.Equal([2], anInspector.Found);
    }
}
