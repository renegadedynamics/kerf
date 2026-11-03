// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Kerf.Kernel.Foundation.Tests;

public class NCollectionTests
{
    [Fact]
    public void Array1_default_constructs_its_elements_as_CPlusPlus_does()
    {
        var aDirs = new NCollection_Array1<Direction3D>(1, 3);
        var aTrsfs = new NCollection_Array2<Transform3D>(1, 2, 1, 2);

        Assert.Equal(new Direction3D(), aDirs[2]);
        Assert.Equal(1.0, aDirs[2].X());
        Assert.Equal(new Transform3D(), aTrsfs[2, 2]);
        Assert.Equal(0.0, new NCollection_Array1<double>(1, 3)[3]);
    }

    [Fact]
    public void Array1_bounds_and_ref_access()
    {
        var anArray = new NCollection_Array1<double>(5, 7);
        anArray[5] = 1;
        anArray.ChangeValue(6) += 2;
        anArray.SetValue(7, 3);

        Assert.Equal(3, anArray.Length());
        Assert.Equal((5, 7), (anArray.Lower(), anArray.Upper()));
        Assert.Equal([1.0, 2.0, 3.0], anArray.AsSpan().ToArray());
        Assert.Throws<OutOfRangeException>(() => anArray.Value(8));
    }

    [Fact]
    public void A_buffer_view_shares_storage_and_copies_do_not()
    {
        var anArray = NCollection_Array1<double>.From([1, 2, 3, 4, 5]);
        var aView = new NCollection_Array1<double>(anArray, 2, 1, 3);
        var aCopy = new NCollection_Array1<double>(anArray);

        aView[1] = 20;
        Assert.Equal(20, anArray[2]);
        Assert.Equal(2, aCopy[2]);
        Assert.False(aView.IsDeletable());

        var aGrid = new NCollection_Array2<double>(anArray, 1, 1, 2, 1, 2);
        Assert.Equal(20, aGrid[1, 2]);
        aGrid[2, 1] = 30;
        Assert.Equal(30, anArray[3]);
    }

    [Fact]
    public void Array1_Assign_copies_values_and_Resize_keeps_them()
    {
        var aSource = NCollection_Array1<int>.From([1, 2, 3]);
        var aTarget = new NCollection_Array1<int>(1, 3).Assign(aSource);
        aSource[1] = 9;
        aTarget.Resize(1, 5, true);

        Assert.Equal([1, 2, 3, 0, 0], aTarget.AsSpan().ToArray());
    }

    [Fact]
    public void Array2_is_row_major_with_bounds()
    {
        var anArray = new NCollection_Array2<int>(0, 1, 10, 12);
        anArray[1, 11] = 5;

        Assert.Equal((2, 3), (anArray.NbRows(), anArray.NbColumns()));
        Assert.Equal(5, anArray.AsSpan()[4]);
        anArray.Resize(0, 2, 10, 11, true);
        Assert.Equal(5, anArray[1, 11]);
    }

    [Fact]
    public void Sequence_is_one_based()
    {
        var aSeq = new NCollection_Sequence<string>();
        aSeq.Append("b");
        aSeq.Prepend("a");
        aSeq.InsertAfter(2, "c");
        aSeq.InsertBefore(1, "z");
        aSeq.Remove(1);

        Assert.Equal(["a", "b", "c"], aSeq);
        Assert.Equal("c", aSeq.Last());
        aSeq.Exchange(1, 3);
        Assert.Equal("c", aSeq.First());
    }

    [Fact]
    public void List_iterator_moves_to_the_next_item_after_removal()
    {
        var aList = new NCollection_List<int>([1, 2, 3, 4]);
        for (var anIter = new NCollection_List<int>.Iterator(aList); anIter.More();)
        {
            if (anIter.Value() % 2 == 0)
            {
                aList.Remove(anIter);
            }
            else
            {
                anIter.Next();
            }
        }

        Assert.Equal([1, 3], aList);
    }

    [Fact]
    public void IndexedMap_moves_the_last_key_into_a_removed_index()
    {
        var aMap = new NCollection_IndexedMap<string>();
        aMap.Add("a");
        aMap.Add("b");
        aMap.Add("c");
        Assert.Equal(2, aMap.Add("b"));

        aMap.RemoveFromIndex(1);
        Assert.Equal("c", aMap.FindKey(1));
        Assert.Equal(1, aMap.FindIndex("c"));
        Assert.Equal(0, aMap.FindIndex("a"));
    }

    [Fact]
    public void Progress_reports_and_cancels()
    {
        using var aSource = new CancellationTokenSource();
        var aReports = new List<double>();
        var anIndicator = new ProgressIndicator(aSource.Token, new SynchronousProgress(aReports.Add));

        using (var aScope = new ProgressScope(anIndicator.Start(), "outer", 4))
        {
            for (int i = 0; i < 2 && aScope.More(); i++)
            {
                ProgressRange aStep = aScope.Next();
                aStep.Close();
            }

            aSource.Cancel();
            Assert.False(aScope.More());
        }

        Assert.Equal([0.25, 0.5, 1.0], aReports);
    }

    private sealed class SynchronousProgress(Action<double> theReport) : IProgress<double>
    {
        public void Report(double value) => theReport(value);
    }
}
