// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

// OCCT has no GTest for NCollection_CellFilter; these check the port's behaviour on 2D points.
public class NCollection_CellFilterTest
{
    private sealed class PointInspector : INCollection_CellFilter_Inspector<int, double[]>
    {
        public static double Coord(int i, double[] thePnt) => thePnt[i];

        public static bool IsEqual(int theT1, int theT2) => theT1 == theT2;

        public List<int> Found { get; } = [];

        public bool PurgeAll { get; set; }

        public NCollection_CellFilter_Action Inspect(int theObject)
        {
            Found.Add(theObject);
            return PurgeAll ? NCollection_CellFilter_Action.CellFilter_Purge : NCollection_CellFilter_Action.CellFilter_Keep;
        }
    }

    [Fact]
    public void AddAndInspect_FindsTargetsInCoveredCells()
    {
        NCollection_CellFilter<PointInspector, int, double[]> aFilter = new(2, 1.0);
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
    public void AddRange_PutsTargetInEveryCell()
    {
        NCollection_CellFilter<PointInspector, int, double[]> aFilter = new(2, 1.0);
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
        NCollection_CellFilter<PointInspector, int, double[]> aFilter = new(2, 1.0);
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
        NCollection_CellFilter<PointInspector, int, double[]> aFilter = new(2, 1.0);
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
