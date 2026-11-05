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
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Offsets.MedialAxis;

/// <summary>A node of a MAT_TList: an item and its neighbours.</summary>
/// <remarks>OCCT instantiates the node as MAT_TListNodeOfListOfBisector and MAT_TListNodeOfListOfEdge; only the
/// lists build and read nodes, so the port has the generic alone (decision 607).</remarks>
internal sealed class MedialTListNode<TItem> : Transient
    where TItem : class
{
    private MedialTListNode<TItem>? _thenext;
    private MedialTListNode<TItem>? _theprevious;
    private TItem? _theitem;

    public MedialTListNode()
    {
    }

    public MedialTListNode(TItem? anitem)
    {
        _theitem = anitem;
    }

    public TItem GetItem() => _theitem!;

    public MedialTListNode<TItem>? Next() => _thenext;

    public MedialTListNode<TItem>? Previous() => _theprevious;

    internal void SetItem(TItem? anitem) => _theitem = anitem;

    internal void Next(MedialTListNode<TItem>? atlistnode) => _thenext = atlistnode;

    internal void Previous(MedialTListNode<TItem>? atlistnode) => _theprevious = atlistnode;
}

/// <summary>A doubly linked list with a current position, indexed from 1, that can be made circular (Loop).</summary>
/// <remarks>
/// MAT_TList.gxx ported once as a generic (decision 607); OCCT's instantiations are MAT_ListOfBisector and
/// MAT_ListOfEdge. Items are compared by reference (OCCT compares handles). OCCT's destructor unlinks the nodes to
/// break the reference cycles; the collector needs no help. Dump (each item's Dump to std::cout) is left out. The
/// mutators are internal: the lists are working objects of MAT2d_Mat2d.
/// </remarks>
internal abstract class MedialTList<TItem> : Transient
    where TItem : class
{
    private MedialTListNode<TItem>? _thefirstnode;
    private MedialTListNode<TItem>? _thelastnode;
    private MedialTListNode<TItem>? _thecurrentnode;
    private int _thecurrentindex;
    private int _thenumberofitems;

    protected MedialTList()
    {
        _thecurrentindex = 0;
        _thenumberofitems = 0;
    }

    /// <summary>Moves the current position to the first item.</summary>
    public void First()
    {
        _thecurrentnode = _thefirstnode;
        _thecurrentindex = 1;
    }

    /// <summary>Moves the current position to the last item.</summary>
    public void Last()
    {
        _thecurrentnode = _thelastnode;
        _thecurrentindex = _thenumberofitems;
    }

    /// <summary>Moves the current position to anitem (by reference), or past the end if it isn't in the list.</summary>
    public void Init(TItem anitem)
    {
        First();
        while (More())
        {
            if (anitem == _thecurrentnode!.GetItem())
            {
                break;
            }
            Next();
        }
    }

    public void Next()
    {
        if (!IsEmpty)
        {
            _thecurrentnode = _thecurrentnode!.Next();
            _thecurrentindex = (_thecurrentindex % _thenumberofitems) + 1;
        }
    }

    public void Previous()
    {
        if (!IsEmpty)
        {
            _thecurrentnode = _thecurrentnode!.Previous();
            _thecurrentindex = ((_thecurrentindex + _thenumberofitems - 2) % _thenumberofitems) + 1;
        }
    }

    public bool More() => _thecurrentnode is not null;

    public TItem Current() => _thecurrentnode!.GetItem();

    /// <summary>Replaces the current item.</summary>
    internal void Current(TItem anitem) => _thecurrentnode!.SetItem(anitem);

    public TItem FirstItem() => _thefirstnode!.GetItem();

    public TItem LastItem() => _thelastnode!.GetItem();

    public TItem PreviousItem() => _thecurrentnode!.Previous()!.GetItem();

    public TItem NextItem() => _thecurrentnode!.Next()!.GetItem();

    public int Number() => _thenumberofitems;

    public int Index() => _thecurrentindex;

    /// <summary>Moves the current position to anindex and returns its item (OCCT's <c>operator()</c> too).</summary>
    public TItem Brackets(int anindex)
    {
        if (_thecurrentindex > anindex)
        {
            while (_thecurrentindex != anindex)
            {
                _thecurrentindex--;
                _thecurrentnode = _thecurrentnode!.Previous();
            }
        }
        else if (_thecurrentindex < anindex)
        {
            while (_thecurrentindex != anindex)
            {
                _thecurrentindex++;
                _thecurrentnode = _thecurrentnode!.Next();
            }
        }
        return _thecurrentnode!.GetItem();
    }

    /// <summary>OCCT's <c>operator()(anindex)</c>: Brackets(anindex).</summary>
    public TItem this[int anindex] => Brackets(anindex);

    /// <summary>Unlinks the current item; the current position stays on it (its neighbours no longer see it).</summary>
    internal void Unlink()
    {
        bool previousisnull = _thecurrentnode!.Previous() is null;
        bool nextisnull = _thecurrentnode.Next() is null;

        if (_thecurrentindex != 0)
        {
            if (!nextisnull)
            {
                _thecurrentnode.Next()!.Previous(_thecurrentnode.Previous());
            }
            if (!previousisnull)
            {
                _thecurrentnode.Previous()!.Next(_thecurrentnode.Next());
            }

            if (_thecurrentindex == 1)
            {
                _thefirstnode = _thecurrentnode.Next();
            }
            else if (_thecurrentindex == _thenumberofitems)
            {
                _thelastnode = _thecurrentnode.Previous();
            }
        }
        _thenumberofitems--;
        _thecurrentindex--;
    }

    internal void LinkBefore(TItem anitem)
    {
        _thenumberofitems++;
        if (_thecurrentindex != 0)
        {
            _thecurrentindex++;
        }

        MedialTListNode<TItem>? previous;

        MedialTListNode<TItem> node = new(anitem);

        if (_thecurrentnode!.Previous() is not null)
        {
            previous = _thecurrentnode.Previous()!;
            previous.Next(node);
            node.Previous(previous);
        }

        if (_thecurrentindex == 2)
        {
            _thefirstnode = node;
        }

        _thecurrentnode.Previous(node);
        node.Next(_thecurrentnode);
    }

    internal void LinkAfter(TItem anitem)
    {
        _thenumberofitems++;
        MedialTListNode<TItem>? next;

        MedialTListNode<TItem> node = new(anitem);

        if (_thecurrentnode!.Next() is not null)
        {
            next = _thecurrentnode.Next()!;
            next.Previous(node);
            node.Next(next);
        }

        if (_thecurrentindex + 1 == _thenumberofitems)
        {
            _thelastnode = node;
        }

        _thecurrentnode.Next(node);
        node.Previous(_thecurrentnode);
    }

    internal void FrontAdd(TItem anitem)
    {
        _thenumberofitems++;
        if (_thecurrentindex != 0)
        {
            _thecurrentindex++;
        }

        MedialTListNode<TItem> node = new(anitem);

        if (_thefirstnode is not null)
        {
            _thefirstnode.Previous(node);
            node.Next(_thefirstnode);
        }
        else
        {
            _thelastnode = node;
        }

        _thefirstnode = node;
    }

    internal void BackAdd(TItem anitem)
    {
        _thenumberofitems++;
        MedialTListNode<TItem> node = new(anitem);

        if (_thelastnode is not null)
        {
            _thelastnode.Next(node);
            node.Previous(_thelastnode);
        }
        else
        {
            _thefirstnode = node;
        }

        _thelastnode = node;
    }

    /// <summary>Exchanges the current item and the next one; the current position follows the item.</summary>
    internal void Permute()
    {
        MedialTListNode<TItem>? previous = _thecurrentnode!.Previous();
        MedialTListNode<TItem> current = _thecurrentnode;
        MedialTListNode<TItem> next = _thecurrentnode.Next()!;
        MedialTListNode<TItem>? nextnext = next.Next();

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
        if (_thefirstnode == current)
        {
            _thefirstnode = next;
        }
        if (_thelastnode == next)
        {
            _thelastnode = current;
        }
        _thecurrentindex++;
    }

    /// <summary>Makes the list circular: the last node's next is the first.</summary>
    internal void Loop()
    {
        _thelastnode!.Next(_thefirstnode);
        _thefirstnode!.Previous(_thelastnode);
    }

    public bool IsEmpty => _thenumberofitems == 0;
}

/// <summary>A MAT_TList of MAT_Bisector.</summary>
internal sealed class MedialListOfBisector : MedialTList<MedialBisector>
{
    public MedialListOfBisector()
    {
    }
}

/// <summary>A MAT_TList of MAT_Edge.</summary>
internal sealed class MedialListOfEdge : MedialTList<MedialEdge>
{
    public MedialListOfEdge()
    {
    }
}
