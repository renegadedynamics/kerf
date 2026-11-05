// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_SurfaceExplorer.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_SurfaceExplorer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Booleans.Legacy.Data;

/// <summary>Explores the surfaces of a data structure in index order, only the kept ones by default.</summary>
/// <remarks>
/// A new surface is not kept (<see cref="DsSurface"/>), so with the default <c>FindOnlyKeep</c> the explorer
/// skips the surfaces until something keeps them, as in OCCT.
/// </remarks>
internal sealed class DsSurfaceExplorer
{
    private int _index;
    private int _max;
    private DsDataStructure? _ds;
    private bool _found;
    private readonly DsSurface _empty = new();
    private bool _findKeep;

    /// <summary>An explorer of nothing.</summary>
    /// <remarks>OCCT leaves FindKeep unset here; the port's is false.</remarks>
    public DsSurfaceExplorer()
    {
        _index = 1;
        _max = 0;
        _ds = null;
        _found = false;
    }

    /// <summary>An explorer of the surfaces of <paramref name="ds"/> (only the kept ones if <paramref name="findKeep"/>).</summary>
    public DsSurfaceExplorer(DsDataStructure ds, bool findKeep = true)
    {
        Init(ds, findKeep);
    }

    /// <summary>Restarts on the surfaces of <paramref name="ds"/>.</summary>
    public void Init(DsDataStructure ds, bool findKeep = true)
    {
        _index = 1;
        _max = ds.NbSurfaces();
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
                _found = IsSurfaceKeep(_index);
            }
            else
            {
                _found = IsSurface(_index);
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

    /// <summary>Whether there is a current surface.</summary>
    public bool More() => _found;

    /// <summary>Moves to the next surface.</summary>
    public void Next()
    {
        _index++;
        Find();
    }

    /// <summary>The current surface (an empty one if there is none).</summary>
    public DsSurface Surface()
    {
        if (_found)
        {
            return MYDS.Surface(_index);
        }
        else
        {
            return _empty;
        }
    }

    /// <summary>Whether the data structure has a surface <paramref name="i"/>.</summary>
    public bool IsSurface(int i)
    {
        bool b = MYDS._surfaces.ContainsKey(i);
        return b;
    }

    /// <summary>Whether the data structure has a kept surface <paramref name="i"/>.</summary>
    public bool IsSurfaceKeep(int i)
    {
        bool b = MYDS._surfaces.ContainsKey(i);
        if (b)
        {
            b = MYDS.Surface(i).Keep();
        }
        return b;
    }

    /// <summary>The surface <paramref name="i"/> (an empty one if there is none).</summary>
    public DsSurface Surface(int i)
    {
        if (IsSurface(i))
        {
            return MYDS.Surface(i);
        }
        else
        {
            return _empty;
        }
    }

    /// <summary>The number of surfaces the explorer finds; it restarts it and leaves it at the end, as OCCT does.</summary>
    public int NbSurface()
    {
        _index = 1;
        _max = MYDS.NbSurfaces();
        Find();
        int n = 0;
        for (; More(); Next())
        {
            n++;
        }
        return n;
    }

    /// <summary>The index of the current surface.</summary>
    public int Index() => _index;
}
