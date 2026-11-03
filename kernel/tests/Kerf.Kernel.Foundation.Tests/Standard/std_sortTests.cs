using Kerf.Engine.Foundation.Std;

namespace Kerf.Kernel.Foundation.Tests.Standard;

public class std_sortTests
{
    private readonly struct ByKey : Heap.ILess<(int Key, int Id)>
    {
        public bool Less((int Key, int Id) theLeft, (int Key, int Id) theRight) => theLeft.Key < theRight.Key;
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(31)]
    [InlineData(32)]
    [InlineData(33)]
    [InlineData(41)]
    [InlineData(100)]
    [InlineData(1000)]
    [InlineData(10000)]
    public void sort_SortsAnyRange(int theSize)
    {
        var aRandom = new Random(theSize);
        foreach (int aKeys in new[] { 3, 1000000 })
        {
            var anItems = new (int Key, int Id)[theSize];
            for (int i = 0; i < theSize; ++i)
            {
                anItems[i] = (aRandom.Next(aKeys), i);
            }
            var aSorted = ((int Key, int Id)[])anItems.Clone();
            Sorting.Sort<(int Key, int Id), ByKey>(aSorted, default);
            for (int i = 1; i < theSize; ++i)
            {
                Assert.True(aSorted[i - 1].Key <= aSorted[i].Key);
            }
            Assert.Equal(anItems.OrderBy(x => x.Id), aSorted.OrderBy(x => x.Id));
        }
    }

    [Fact]
    public void sort_SmallRangesKeepTiesInOrder()
    {
        // MSVC sorts up to 32 elements by insertion, which keeps tied elements in their order
        var aRandom = new Random(7);
        var anItems = new (int Key, int Id)[32];
        for (int i = 0; i < anItems.Length; ++i)
        {
            anItems[i] = (aRandom.Next(4), i);
        }
        Sorting.Sort<(int Key, int Id), ByKey>(anItems, default);
        Assert.Equal(anItems.OrderBy(x => x.Key).ThenBy(x => x.Id), anItems);
    }

    [Fact]
    public void sort_AllEqualAndPresorted()
    {
        foreach (Func<int, int> aKey in new Func<int, int>[] { _ => 5, i => i, i => -i, i => i % 7 })
        {
            var anItems = Enumerable.Range(0, 5000).Select(i => (aKey(i), i)).ToArray();
            Sorting.Sort<(int Key, int Id), ByKey>(anItems, default);
            for (int i = 1; i < anItems.Length; ++i)
            {
                Assert.True(anItems[i - 1].Item1 <= anItems[i].Item1);
            }
        }
    }

    [Fact]
    public void stable_sort_KeepsTiesInOrder()
    {
        var aRandom = new Random(11);
        var anItems = new (int Key, int Id)[3000];
        for (int i = 0; i < anItems.Length; ++i)
        {
            anItems[i] = (aRandom.Next(10), i);
        }
        Sorting.stable_sort<(int Key, int Id), ByKey>(anItems, default);
        Assert.Equal(anItems.OrderBy(x => x.Key).ThenBy(x => x.Id), anItems);
    }
}
