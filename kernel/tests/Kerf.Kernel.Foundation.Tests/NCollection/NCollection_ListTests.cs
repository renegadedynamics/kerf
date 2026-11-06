// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Kerf.Kernel.Foundation.Tests;

// The list is an array and a count (unit 11Q): each operation against a .NET list doing the same
public class NCollection_ListTests
{
    [Fact]
    public void Every_operation_matches_a_list_doing_the_same()
    {
        var random = new Random(11);
        var list = new NCollection_List<string>();
        var model = new List<string>();
        for (int step = 0; step < 20_000; step++)
        {
            string item = random.Next(40).ToString(System.Globalization.CultureInfo.InvariantCulture);
            var iter = new NCollection_List<string>.Iterator(list);
            int at = model.Count == 0 ? 0 : random.Next(model.Count);
            for (int i = 0; i < at; i++)
                iter.Next();
            switch (random.Next(12))
            {
                case 0 or 1 or 2:
                    list.Append(item);
                    model.Add(item);
                    break;
                case 3:
                    list.Prepend(item);
                    model.Insert(0, item);
                    break;
                case 4:
                    list.RemoveFirst();
                    if (model.Count > 0)
                        model.RemoveAt(0);
                    break;
                case 5:
                    Assert.Equal(model.Remove(item), list.Remove(item));
                    break;
                case 6 when model.Count > 0:
                    list.Remove(iter);
                    model.RemoveAt(at);
                    break;
                case 7 when model.Count > 0:
                    list.InsertBefore(item, iter);
                    model.Insert(at, item);
                    Assert.Equal(at + 1, iter.Index);
                    break;
                case 8 when model.Count > 0:
                    list.InsertAfter(item, iter);
                    model.Insert(at + 1, item);
                    Assert.Equal(at, iter.Index);
                    break;
                case 9:
                    list.Reverse();
                    model.Reverse();
                    break;
                case 10:
                    var other = new NCollection_List<string>([item, item + "x"]);
                    if (random.Next(2) == 0)
                    {
                        list.Append(other);
                        model.AddRange([item, item + "x"]);
                    }
                    else
                    {
                        list.Prepend(other);
                        model.InsertRange(0, [item, item + "x"]);
                    }
                    Assert.True(other.IsEmpty());
                    break;
                default:
                    list.TrimExcess();
                    break;
            }

            if (step % 500 == 499)
            {
                list.Clear();
                model.Clear();
            }

            Assert.Equal(model.Count, list.Extent());
            Assert.Equal(model.Contains(item), list.Contains(item));
            if (step % 64 == 0)
                Assert.Equal(model, list);
        }
    }

    [Fact]
    public void Copies_exchange_and_assign_hold_their_own_items()
    {
        var a = new NCollection_List<int>([1, 2, 3]);
        var copy = new NCollection_List<int>(a);
        var assigned = new NCollection_List<int>().Assign(a);
        a.ChangeFirst() = 9;
        Assert.Equal([9, 2, 3], a);
        Assert.Equal([1, 2, 3], copy);
        Assert.Equal([1, 2, 3], assigned);

        var b = new NCollection_List<int>([7]);
        a.Exchange(b);
        Assert.Equal([7], a);
        Assert.Equal([9, 2, 3], b);
        Assert.Same(a, a.Assign(a));
    }

    [Fact]
    public void Changing_a_list_while_enumerating_it_throws_as_a_lists_does()
    {
        var list = new NCollection_List<int>([1, 2]);
        Assert.Throws<InvalidOperationException>(() =>
        {
            foreach (int item in list)
                list.Append(item);
        });
    }

    [Fact]
    public void An_empty_list_has_no_first_or_last_and_a_trimmed_one_keeps_its_items()
    {
        var list = new NCollection_List<int>();
        Assert.Throws<NoSuchObjectException>(() => list.First());
        Assert.Throws<NoSuchObjectException>(() => list.Last());
        Assert.Throws<NoSuchObjectException>(() => list.ChangeFirst());
        list.TrimExcess();
        list.Append(1);
        list.Append(2);
        list.Append(3);
        list.TrimExcess();
        Assert.Equal([1, 2, 3], list);
        list.Append(4);
        Assert.Equal(4, list.Last());
        Assert.Empty(new NCollection_List<int>(new NCollection_List<int>()));
    }
}
