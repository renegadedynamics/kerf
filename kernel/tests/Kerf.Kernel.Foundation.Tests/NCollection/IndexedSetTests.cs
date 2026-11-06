// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;

namespace Kerf.Kernel.Foundation.Tests;

/// <summary>
/// A set compares its first few items in turn and hashes the rest (the check on small shapes), which mustn't show: the same
/// operations give the same indices either way, including across the point where it changes.
/// </summary>
public class IndexedSetTests
{
    // What the set promised before: a list, and a dictionary of the indices, with the comparer's own hash
    private sealed class Model(IEqualityComparer<int> comparer)
    {
        public readonly List<int> Items = [];
        public readonly Dictionary<int, int> Indices = new(comparer);

        public int Add(int item)
        {
            if (Indices.TryGetValue(item, out int index))
            {
                return index;
            }

            Indices[item] = Items.Count;
            Items.Add(item);
            return Items.Count - 1;
        }

        public void RemoveAtSwap(int index)
        {
            int last = Items.Count - 1;
            if (index != last)
            {
                (Items[index], Items[last]) = (Items[last], Items[index]);
                Indices[Items[index]] = index;
                Indices[Items[last]] = last;
            }

            Indices.Remove(Items[last]);
            Items.RemoveAt(last);
        }
    }

    // Equal by the value's remainder: so an item can stand for another, and which is kept shows
    private sealed class ByRemainder(int modulus) : IEqualityComparer<int>
    {
        public bool Equals(int x, int y) => x % modulus == y % modulus;

        public int GetHashCode(int obj) => (obj % modulus).GetHashCode();
    }

    [Theory]
    [InlineData(1, 6)]
    [InlineData(2, 12)]
    [InlineData(3, 40)]
    [InlineData(4, 400)]
    public void The_same_operations_give_the_same_indices_as_a_dictionary(int seed, int range)
    {
        var comparer = new ByRemainder(range);
        var random = new Random(seed);
        var set = new IndexedSet<int>(comparer);
        var model = new Model(comparer);
        for (int step = 0; step < 4000; step++)
        {
            int item = random.Next(range * 3);
            switch (random.Next(8))
            {
                case 0 when model.Items.Count > 0:
                {
                    int index = random.Next(model.Items.Count);
                    set.RemoveAtSwap(index);
                    model.RemoveAtSwap(index);
                    break;
                }
                case 1:
                {
                    bool had = model.Indices.TryGetValue(item, out int index);
                    Assert.Equal(had, set.Remove(item));
                    if (had)
                    {
                        model.RemoveAtSwap(index);
                    }

                    break;
                }
                case 2 when model.Items.Count > 1:
                {
                    int a = random.Next(model.Items.Count), b = random.Next(model.Items.Count);
                    set.Swap(a, b);
                    (model.Items[a], model.Items[b]) = (model.Items[b], model.Items[a]);
                    model.Indices[model.Items[a]] = a;
                    model.Indices[model.Items[b]] = b;
                    break;
                }
                case 3 when model.Items.Count > 0 && step % 50 == 3:
                    set.Clear();
                    model.Items.Clear();
                    model.Indices.Clear();
                    break;
                case 4 when model.Items.Count > 0:
                {
                    int index = random.Next(model.Items.Count);
                    bool taken = model.Indices.TryGetValue(item, out int existing) && existing != index;
                    if (taken)
                    {
                        Assert.Throws<DomainErrorException>(() => set.Substitute(index, item));
                    }
                    else
                    {
                        set.Substitute(index, item);
                        model.Indices.Remove(model.Items[index]);
                        model.Items[index] = item;
                        model.Indices[item] = index;
                    }

                    break;
                }
                default:
                {
                    bool isNew = !model.Indices.ContainsKey(item);
                    if (step % 2 == 0)
                    {
                        Assert.Equal(model.Add(item), set.Add(item));
                    }
                    else
                    {
                        model.Add(item);
                        Assert.Equal(isNew, set.TryAdd(item));
                    }

                    break;
                }
            }

            Assert.Equal(model.Items, set);
            Assert.Equal(model.Items.Count, set.Count);
            int probe = random.Next(range * 3);
            int expected = model.Indices.TryGetValue(probe, out int at) ? at : -1;
            Assert.Equal(expected, set.IndexOf(probe));
            Assert.Equal(expected >= 0, set.Contains(probe));
        }
    }

    [Theory]
    [InlineData(5)]
    [InlineData(30)]
    public void A_copy_keeps_the_comparer_and_goes_its_own_way(int count)
    {
        var set = new IndexedSet<int>(new ByRemainder(1000));
        for (int i = 0; i < count; i++)
        {
            set.Add(i);
        }

        var copy = new IndexedSet<int>(set);
        Assert.Same(set.Comparer, copy.Comparer);
        Assert.Equal(count - 1, copy.IndexOf(count - 1 + 1000));
        copy.Add(999);
        Assert.Equal(count, set.Count);
        Assert.Equal(count + 1, copy.Count);
        Assert.False(set.Contains(999));
    }
}
