// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT/MAT_TList.gxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT/MAT_TList.lxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT/MAT_ListOfBisector.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT/MAT_ListOfBisector_0.cxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT/MAT_ListOfEdge.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT/MAT_ListOfEdge_0.cxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT/MAT_TListNodeOfListOfBisector.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT/MAT_TListNodeOfListOfBisector_0.cxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT/MAT_TListNodeOfListOfEdge.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT/MAT_TListNodeOfListOfEdge_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>A node of a MAT_TList: an item and its neighbours.</summary>
/// <remarks>OCCT instantiates the node as MAT_TListNodeOfListOfBisector and MAT_TListNodeOfListOfEdge; only the
/// lists build and read nodes, so the port has the generic alone (decision 607).</remarks>
public sealed class MAT_TListNode<TItem> : Transient
    where TItem : class
{
    private MAT_TListNode<TItem>? thenext;
    private MAT_TListNode<TItem>? theprevious;
    private TItem? theitem;

    public MAT_TListNode()
    {
    }

    public MAT_TListNode(TItem? anitem)
    {
        theitem = anitem;
    }

    public TItem GetItem() => theitem!;

    public MAT_TListNode<TItem>? Next() => thenext;

    public MAT_TListNode<TItem>? Previous() => theprevious;

    internal void SetItem(TItem? anitem) => theitem = anitem;

    internal void Next(MAT_TListNode<TItem>? atlistnode) => thenext = atlistnode;

    internal void Previous(MAT_TListNode<TItem>? atlistnode) => theprevious = atlistnode;
}

/// <summary>A doubly linked list with a current position, indexed from 1, that can be made circular (Loop).</summary>
/// <remarks>
/// MAT_TList.gxx ported once as a generic (decision 607); OCCT's instantiations are MAT_ListOfBisector and
/// MAT_ListOfEdge. Items are compared by reference (OCCT compares handles). OCCT's destructor unlinks the nodes to
/// break the reference cycles; the collector needs no help. Dump (each item's Dump to std::cout) is left out. The
/// mutators are internal: the lists are working objects of MAT2d_Mat2d.
/// </remarks>
public abstract class MAT_TList<TItem> : Transient
    where TItem : class
{
    private MAT_TListNode<TItem>? thefirstnode;
    private MAT_TListNode<TItem>? thelastnode;
    private MAT_TListNode<TItem>? thecurrentnode;
    private int thecurrentindex;
    private int thenumberofitems;

    protected MAT_TList()
    {
        thecurrentindex = 0;
        thenumberofitems = 0;
    }

    /// <summary>Moves the current position to the first item.</summary>
    public void First()
    {
        thecurrentnode = thefirstnode;
        thecurrentindex = 1;
    }

    /// <summary>Moves the current position to the last item.</summary>
    public void Last()
    {
        thecurrentnode = thelastnode;
        thecurrentindex = thenumberofitems;
    }

    /// <summary>Moves the current position to anitem (by reference), or past the end if it isn't in the list.</summary>
    public void Init(TItem anitem)
    {
        First();
        while (More())
        {
            if (anitem == thecurrentnode!.GetItem())
            {
                break;
            }
            Next();
        }
    }

    public void Next()
    {
        if (!IsEmpty())
        {
            thecurrentnode = thecurrentnode!.Next();
            thecurrentindex = (thecurrentindex % thenumberofitems) + 1;
        }
    }

    public void Previous()
    {
        if (!IsEmpty())
        {
            thecurrentnode = thecurrentnode!.Previous();
            thecurrentindex = ((thecurrentindex + thenumberofitems - 2) % thenumberofitems) + 1;
        }
    }

    public bool More() => thecurrentnode is not null;

    public TItem Current() => thecurrentnode!.GetItem();

    /// <summary>Replaces the current item.</summary>
    internal void Current(TItem anitem) => thecurrentnode!.SetItem(anitem);

    public TItem FirstItem() => thefirstnode!.GetItem();

    public TItem LastItem() => thelastnode!.GetItem();

    public TItem PreviousItem() => thecurrentnode!.Previous()!.GetItem();

    public TItem NextItem() => thecurrentnode!.Next()!.GetItem();

    public int Number() => thenumberofitems;

    public int Index() => thecurrentindex;

    /// <summary>Moves the current position to anindex and returns its item (OCCT's <c>operator()</c> too).</summary>
    public TItem Brackets(int anindex)
    {
        if (thecurrentindex > anindex)
        {
            while (thecurrentindex != anindex)
            {
                thecurrentindex--;
                thecurrentnode = thecurrentnode!.Previous();
            }
        }
        else if (thecurrentindex < anindex)
        {
            while (thecurrentindex != anindex)
            {
                thecurrentindex++;
                thecurrentnode = thecurrentnode!.Next();
            }
        }
        return thecurrentnode!.GetItem();
    }

    /// <summary>OCCT's <c>operator()(anindex)</c>: Brackets(anindex).</summary>
    public TItem this[int anindex] => Brackets(anindex);

    /// <summary>Unlinks the current item; the current position stays on it (its neighbours no longer see it).</summary>
    internal void Unlink()
    {
        bool previousisnull = thecurrentnode!.Previous() is null;
        bool nextisnull = thecurrentnode.Next() is null;

        if (thecurrentindex != 0)
        {
            if (!nextisnull)
            {
                thecurrentnode.Next()!.Previous(thecurrentnode.Previous());
            }
            if (!previousisnull)
            {
                thecurrentnode.Previous()!.Next(thecurrentnode.Next());
            }

            if (thecurrentindex == 1)
            {
                thefirstnode = thecurrentnode.Next();
            }
            else if (thecurrentindex == thenumberofitems)
            {
                thelastnode = thecurrentnode.Previous();
            }
        }
        thenumberofitems--;
        thecurrentindex--;
    }

    internal void LinkBefore(TItem anitem)
    {
        thenumberofitems++;
        if (thecurrentindex != 0)
        {
            thecurrentindex++;
        }

        MAT_TListNode<TItem>? previous;

        MAT_TListNode<TItem> node = new(anitem);

        if (thecurrentnode!.Previous() is not null)
        {
            previous = thecurrentnode.Previous()!;
            previous.Next(node);
            node.Previous(previous);
        }

        if (thecurrentindex == 2)
        {
            thefirstnode = node;
        }

        thecurrentnode.Previous(node);
        node.Next(thecurrentnode);
    }

    internal void LinkAfter(TItem anitem)
    {
        thenumberofitems++;
        MAT_TListNode<TItem>? next;

        MAT_TListNode<TItem> node = new(anitem);

        if (thecurrentnode!.Next() is not null)
        {
            next = thecurrentnode.Next()!;
            next.Previous(node);
            node.Next(next);
        }

        if (thecurrentindex + 1 == thenumberofitems)
        {
            thelastnode = node;
        }

        thecurrentnode.Next(node);
        node.Previous(thecurrentnode);
    }

    internal void FrontAdd(TItem anitem)
    {
        thenumberofitems++;
        if (thecurrentindex != 0)
        {
            thecurrentindex++;
        }

        MAT_TListNode<TItem> node = new(anitem);

        if (thefirstnode is not null)
        {
            thefirstnode.Previous(node);
            node.Next(thefirstnode);
        }
        else
        {
            thelastnode = node;
        }

        thefirstnode = node;
    }

    internal void BackAdd(TItem anitem)
    {
        thenumberofitems++;
        MAT_TListNode<TItem> node = new(anitem);

        if (thelastnode is not null)
        {
            thelastnode.Next(node);
            node.Previous(thelastnode);
        }
        else
        {
            thefirstnode = node;
        }

        thelastnode = node;
    }

    /// <summary>Exchanges the current item and the next one; the current position follows the item.</summary>
    internal void Permute()
    {
        MAT_TListNode<TItem>? previous = thecurrentnode!.Previous();
        MAT_TListNode<TItem> current = thecurrentnode;
        MAT_TListNode<TItem> next = thecurrentnode.Next()!;
        MAT_TListNode<TItem>? nextnext = next.Next();

        if (previous is not null)
        {
            previous.Next(next);
            next.Previous(previous);
        }
        else
        {
            next.Previous(null);
        }
        next.Next(current);
        current.Previous(next);
        if (nextnext is not null)
        {
            current.Next(nextnext);
            nextnext.Previous(current);
        }
        else
        {
            current.Next(null);
        }
        if (thefirstnode == current)
        {
            thefirstnode = next;
        }
        if (thelastnode == next)
        {
            thelastnode = current;
        }
        thecurrentindex++;
    }

    /// <summary>Makes the list circular: the last node's next is the first.</summary>
    internal void Loop()
    {
        thelastnode!.Next(thefirstnode);
        thefirstnode!.Previous(thelastnode);
    }

    public bool IsEmpty() => thenumberofitems == 0;
}

/// <summary>A MAT_TList of MAT_Bisector.</summary>
public sealed class MAT_ListOfBisector : MAT_TList<MAT_Bisector>
{
    public MAT_ListOfBisector()
    {
    }
}

/// <summary>A MAT_TList of MAT_Edge.</summary>
public sealed class MAT_ListOfEdge : MAT_TList<MAT_Edge>
{
    public MAT_ListOfEdge()
    {
    }
}
