// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Offsets.MedialAxis;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>MAT_TList (MAT_ListOfEdge, MAT_ListOfBisector): OCCT's doubly linked list with a current position, as
/// MAT2d_Mat2d uses it.</summary>
public class MAT_TListTests
{
    private static (MedialListOfEdge, MedialEdge[]) List(int theNb)
    {
        MedialListOfEdge aL = new();
        MedialEdge[] anEdges = new MedialEdge[theNb];
        for (int i = 0; i < theNb; i++)
        {
            anEdges[i] = new MedialEdge();
            anEdges[i].EdgeNumber(i + 1);
            aL.BackAdd(anEdges[i]);
        }
        return (aL, anEdges);
    }

    private static int[] Numbers(MedialListOfEdge theL)
    {
        List<int> aRes = [];
        theL.First();
        for (int i = 0; i < theL.Number(); i++)
        {
            aRes.Add(theL.Current().EdgeNumber());
            theL.Next();
        }
        return [.. aRes];
    }

    [Fact]
    public void Empty()
    {
        MedialListOfEdge aL = new();
        Assert.True(aL.IsEmpty);
        Assert.Equal(0, aL.Number());
        Assert.Equal(0, aL.Index());
        aL.First();
        Assert.False(aL.More());
    }

    [Fact]
    public void BackAdd_FrontAdd_Iterate()
    {
        (MedialListOfEdge aL, MedialEdge[] anEdges) = List(3);
        MedialEdge aFront = new();
        aFront.EdgeNumber(0);
        aL.FrontAdd(aFront);
        Assert.Equal([0, 1, 2, 3], Numbers(aL));
        Assert.Same(aFront, aL.FirstItem());
        Assert.Same(anEdges[2], aL.LastItem());

        // Not looped: Next past the last item leaves no current item.
        aL.Last();
        Assert.Equal(4, aL.Index());
        aL.Next();
        Assert.False(aL.More());
        Assert.Equal(1, aL.Index());
    }

    [Fact]
    public void Loop_Brackets_PreviousNext()
    {
        (MedialListOfEdge aL, MedialEdge[] anEdges) = List(5);
        aL.Loop();
        aL.Last();
        aL.Next();
        Assert.True(aL.More());
        Assert.Same(anEdges[0], aL.Current());
        Assert.Equal(1, aL.Index());
        aL.Previous();
        Assert.Same(anEdges[4], aL.Current());
        Assert.Equal(5, aL.Index());

        Assert.Same(anEdges[2], aL.Brackets(3));
        Assert.Equal(3, aL.Index());
        Assert.Same(anEdges[1], aL.PreviousItem());
        Assert.Same(anEdges[3], aL.NextItem());
        Assert.Same(anEdges[0], aL[1]);
        Assert.Equal(1, aL.Index());
    }

    [Fact]
    public void Init_FindsAnItemByReference()
    {
        (MedialListOfEdge aL, MedialEdge[] anEdges) = List(4);
        aL.Init(anEdges[2]);
        Assert.Equal(3, aL.Index());
        Assert.Same(anEdges[2], aL.Current());
        aL.Init(new MedialEdge());
        Assert.False(aL.More());
    }

    [Fact]
    public void Unlink_LinkBefore_LinkAfter()
    {
        (MedialListOfEdge aL, MedialEdge[] anEdges) = List(4);
        aL.Loop();
        aL.First();
        aL.Brackets(2);
        aL.Unlink();
        Assert.Equal(3, aL.Number());
        Assert.Equal(1, aL.Index());
        // The current node is the unlinked one; Next goes on to its old successor.
        aL.Next();
        Assert.Same(anEdges[2], aL.Current());
        Assert.Equal(2, aL.Index());
        Assert.Equal([1, 3, 4], Numbers(aL));

        MedialEdge aNew = new();
        aNew.EdgeNumber(9);
        aL.Brackets(2);
        aL.LinkAfter(aNew);
        Assert.Equal([1, 3, 9, 4], Numbers(aL));

        MedialEdge aNew2 = new();
        aNew2.EdgeNumber(8);
        aL.Brackets(1);
        aL.LinkBefore(aNew2);
        Assert.Equal(2, aL.Index());
        Assert.Same(aNew2, aL.FirstItem());
    }

    [Fact]
    public void Unlink_FirstAndLast()
    {
        (MedialListOfEdge aL, MedialEdge[] anEdges) = List(3);
        aL.First();
        aL.Unlink();
        Assert.Same(anEdges[1], aL.FirstItem());
        aL.Last();
        aL.Unlink();
        Assert.Same(anEdges[1], aL.LastItem());
        Assert.Equal(1, aL.Number());
    }

    [Fact]
    public void Permute_ExchangesWithTheNext()
    {
        (MedialListOfEdge aL, _) = List(4);
        aL.First();
        aL.Permute();
        Assert.Equal(2, aL.Index());
        Assert.Equal([2, 1, 3, 4], Numbers(aL));
        aL.First();
        aL.Brackets(3);
        aL.Permute();
        Assert.Equal([2, 1, 4, 3], Numbers(aL));
        Assert.Equal(3, aL.LastItem().EdgeNumber());
    }

    [Fact]
    public void Bisector_KeepsItsChildren()
    {
        MedialBisector aB = new();
        Assert.Equal(-1, aB.BisectorNumber());
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Infinite(), aB.FirstParameter());
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Infinite(), aB.SecondParameter());
        Assert.True(aB.List().IsEmpty);
        MedialBisector aC1 = new(), aC2 = new();
        aB.AddBisector(aC1);
        aB.AddBisector(aC2);
        Assert.Same(aC1, aB.FirstBisector());
        Assert.Same(aC2, aB.LastBisector());
        Assert.Equal(2, aB.List().Number());
    }

    [Fact]
    public void Arc_Neighbours()
    {
        MedialBasicElt aE1 = new(1), aE2 = new(2);
        MedialArc anArc = new(1, 1, aE1, aE2);
        MedialArc anOther = new(2, 2, aE2, aE1);
        MedialNode aN1 = new(1, anArc, 0.0), aN2 = new(2, anArc, 3.0);
        anArc.SetFirstNode(aN1);
        anArc.SetSecondNode(aN2);
        Assert.Same(aN2, anArc.TheOtherNode(aN1));
        Assert.Throws<global::Kerf.Engine.Foundation.DomainErrorException>(() => anArc.TheOtherNode(new MedialNode(3, anArc, 1.0)));
        Assert.False(anArc.HasNeighbour(aN2, MedialSide.Left));
        anArc.SetNeighbour(MedialSide.Left, aN2, anOther);
        Assert.True(anArc.HasNeighbour(aN2, MedialSide.Left));
        Assert.Same(anOther, anArc.Neighbour(aN2, MedialSide.Left));
        Assert.Null(anArc.Neighbour(aN2, MedialSide.Right));
        Assert.Throws<global::Kerf.Engine.Foundation.DomainErrorException>(() => anArc.Neighbour(new MedialNode(3, anArc, 1.0), MedialSide.Left));
        Assert.True(aN1.OnBasicElt());
        Assert.True(aN1.PendingNode());
        Assert.False(new MedialNode(0, anArc, global::Kerf.Engine.Foundation.Precision.Infinite()).OnBasicElt());
        Assert.True(new MedialNode(0, anArc, global::Kerf.Engine.Foundation.Precision.Infinite()).Infinite());
    }

    [Fact]
    public void BiInt_IsAValue()
    {
        BiInt a = new(1, 2), b = new(1, 2), c = new(2, 1);
        Assert.Equal(a, b);
        Assert.True(a == b);
        Assert.True(a != c);
        Assert.True(a.IsEqual(b));
        Assert.Equal(a.GetHashCode(), b.GetHashCode());
        Assert.Equal(c, a.WithFirstIndex(2).WithSecondIndex(1));
        Assert.Equal("MAT2d_BiInt(1, 2)", a.ToString());
    }
}
