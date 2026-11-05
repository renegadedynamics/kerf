// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_CurveExplorer.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_CurveExplorer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.Booleans.Legacy.Data;

/// <summary>Explores the curves of a data structure in index order, only the kept ones by default.</summary>
/// <remarks>
/// <see cref="Curve()"/> returns a copy: to change the current curve, use
/// <c>ref DS.ChangeCurve(explorer.Index())</c> (OCCT's callers cast the constness away). OCCT's <c>thread_local</c>
/// empty curve, returned when there is none, is a new empty curve here (decision 1063).
/// </remarks>
internal sealed class DsCurveExplorer
{
    private int _index;
    private int _max;
    private DsDataStructure? _ds;
    private bool _found;
    private bool _findKeep;

    /// <summary>An explorer of nothing.</summary>
    public DsCurveExplorer()
    {
        _index = 1;
        _max = 0;
        _ds = null;
        _found = false;
        _findKeep = false;
    }

    /// <summary>An explorer of the curves of <paramref name="ds"/> (only the kept ones if <paramref name="findKeep"/>).</summary>
    public DsCurveExplorer(DsDataStructure ds, bool findKeep = true)
    {
        Init(ds, findKeep);
    }

    /// <summary>Restarts on the curves of <paramref name="ds"/>.</summary>
    public void Init(DsDataStructure ds, bool findKeep = true)
    {
        _ds = ds;
        _index = 1;
        _max = ds.NbCurves();
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
                _found = IsCurveKeep(_index);
            }
            else
            {
                _found = IsCurve(_index);
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

    /// <summary>Whether there is a current curve.</summary>
    public bool More() => _found;

    /// <summary>Moves to the next curve.</summary>
    public void Next()
    {
        _index++;
        Find();
    }

    /// <summary>The current curve (an empty one if there is none).</summary>
    public DsCurve Curve()
    {
        if (_found)
        {
            return MYDS.Curve(_index);
        }
        else
        {
            return new DsCurve();
        }
    }

    /// <summary>Whether the data structure has a curve <paramref name="i"/> (removed curves included).</summary>
    public bool IsCurve(int i)
    {
        bool b = MYDS._curves.ContainsKey(i);
        return b;
    }

    /// <summary>Whether the data structure has a kept curve <paramref name="i"/>.</summary>
    public bool IsCurveKeep(int i)
    {
        bool b = MYDS._curves.ContainsKey(i);
        if (b)
        {
            b = MYDS.Curve(i).Keep();
        }
        return b;
    }

    /// <summary>The curve <paramref name="i"/> (an empty one if there is none).</summary>
    public DsCurve Curve(int i)
    {
        if (IsCurve(i))
        {
            DsCurve c = MYDS.Curve(i);
            return c;
        }
        return new DsCurve();
    }

    /// <summary>The number of curves the explorer finds; it restarts it and leaves it at the end, as OCCT does.</summary>
    public int NbCurve()
    {
        _index = 1;
        _max = MYDS.NbCurves();
        Find();
        int n = 0;
        for (; More(); Next())
        {
            n++;
        }
        return n;
    }

    /// <summary>The index of the current curve.</summary>
    public int Index() => _index;
}
