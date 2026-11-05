// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_PointExplorer.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_PointExplorer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Booleans.Legacy.Data;

/// <summary>Explores the points of a data structure in index order, only the kept ones by default.</summary>
/// <remarks><see cref="Point()"/> returns a copy: to change the current point, use <c>ref DS.ChangePoint(explorer.Index())</c>.</remarks>
internal sealed class DsPointExplorer
{
    private int _index;
    private int _max;
    private DsDataStructure? _ds;
    private bool _found;
    private readonly DsPoint _empty = new();
    private bool _findKeep;

    /// <summary>An explorer of nothing.</summary>
    public DsPointExplorer()
    {
        _index = 1;
        _max = 0;
        _ds = null;
        _found = false;
        _findKeep = false;
    }

    /// <summary>An explorer of the points of <paramref name="ds"/> (only the kept ones if <paramref name="findKeep"/>).</summary>
    public DsPointExplorer(DsDataStructure ds, bool findKeep = true)
    {
        Init(ds, findKeep);
    }

    /// <summary>Restarts on the points of <paramref name="ds"/>.</summary>
    public void Init(DsDataStructure ds, bool findKeep = true)
    {
        _index = 1;
        _max = ds.NbPoints();
        _ds = ds;
        _findKeep = findKeep;
        Find();
    }

    private DsDataStructure MYDS => _ds!;

    private void Find()
    {
        _found = false;
        while (_index <= _max)
        {
            if (_findKeep)
            {
                _found = IsPointKeep(_index);
            }
            else
            {
                _found = IsPoint(_index);
            }
            if (_found)
            {
                break;
            }
            else
            {
                _index++;
            }
        }
    }

    /// <summary>Whether there is a current point.</summary>
    public bool More() => _found;

    /// <summary>Moves to the next point.</summary>
    public void Next()
    {
        _index++;
        Find();
    }

    /// <summary>The current point (an empty one if there is none).</summary>
    public DsPoint Point()
    {
        if (_found)
        {
            return MYDS.Point(_index);
        }
        else
        {
            return _empty;
        }
    }

    /// <summary>Whether the data structure has a point <paramref name="i"/>.</summary>
    public bool IsPoint(int i)
    {
        bool b = MYDS._points.ContainsKey(i);
        return b;
    }

    /// <summary>Whether the data structure has a kept point <paramref name="i"/>.</summary>
    public bool IsPointKeep(int i)
    {
        bool b = MYDS._points.ContainsKey(i);
        if (b)
        {
            b = MYDS.Point(i).Keep();
        }
        return b;
    }

    /// <summary>The point <paramref name="i"/> (an empty one if there is none).</summary>
    public DsPoint Point(int i)
    {
        if (IsPoint(i))
        {
            return MYDS.Point(i);
        }
        else
        {
            return _empty;
        }
    }

    /// <summary>The number of points the explorer finds; it restarts it and leaves it at the end, as OCCT does.</summary>
    public int NbPoint()
    {
        _index = 1;
        _max = MYDS.NbPoints();
        Find();
        int n = 0;
        for (; More(); Next())
        {
            n++;
        }
        return n;
    }

    /// <summary>The index of the current point.</summary>
    public int Index() => _index;
}
