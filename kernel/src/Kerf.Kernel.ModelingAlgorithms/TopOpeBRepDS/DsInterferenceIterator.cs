// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_InterferenceIterator.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_InterferenceIterator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.Booleans.Legacy.Data;

/// <summary>
/// Iterates over a list of interferences, stopping only on those that match the conditions set (geometry kind and
/// index, support kind and index; none by default).
/// </summary>
/// <remarks>A working object, as OCCT's iterators. Setting a condition does not move the iterator: call <see cref="Match"/>.</remarks>
internal class DsInterferenceIterator
{
    private readonly NCollection_List<DsInterference>.Iterator _iterator = new();
    private bool _gkDef;
    private DsKind _gk;
    private bool _gDef;
    private int _g;
    private bool _skDef;
    private DsKind _sk;
    private bool _sDef;
    private int _s;

    /// <summary>An iterator over nothing.</summary>
    public DsInterferenceIterator()
    {
        _gkDef = false;
        _gDef = false;
        _skDef = false;
        _sDef = false;
    }

    /// <summary>An iterator over the interferences of <paramref name="l"/>.</summary>
    public DsInterferenceIterator(NCollection_List<DsInterference> l)
    {
        _gkDef = false;
        _gDef = false;
        _skDef = false;
        _sDef = false;
        Init(l);
    }

    /// <summary>Restarts on the list <paramref name="l"/>; the conditions are kept.</summary>
    public void Init(NCollection_List<DsInterference> l)
    {
        _iterator.Initialize(l);
        Match();
    }

    /// <summary>The interferences must have the geometry kind <paramref name="gk"/>.</summary>
    public void GeometryKind(DsKind gk)
    {
        _gkDef = true;
        _gk = gk;
    }

    /// <summary>The interferences must have the geometry index <paramref name="g"/>.</summary>
    public void Geometry(int g)
    {
        _gDef = true;
        _g = g;
    }

    /// <summary>The interferences must have the support kind <paramref name="st"/>.</summary>
    public void SupportKind(DsKind st)
    {
        _skDef = true;
        _sk = st;
    }

    /// <summary>The interferences must have the support index <paramref name="s"/>.</summary>
    public void Support(int s)
    {
        _sDef = true;
        _s = s;
    }

    /// <summary>Moves to the first interference, from the current one, that matches the conditions.</summary>
    public void Match()
    {
        while (_iterator.More())
        {
            DsInterference i = _iterator.Value();
            bool b = MatchInterference(i);
            if (!b)
            {
                _iterator.Next();
            }
            else
            {
                break;
            }
        }
    }

    /// <summary>Whether <paramref name="i"/> matches the conditions (true if none is set).</summary>
    public virtual bool MatchInterference(DsInterference i)
    {
        bool gk = _gkDef ? (i.GeometryType() == _gk) : true;
        bool sk = _skDef ? (i.SupportType() == _sk) : true;
        bool g = _gDef ? (i.Geometry() == _g) : true;
        bool s = _sDef ? (i.Support() == _s) : true;
        return (gk && sk && g && s);
    }

    /// <summary>Whether there is a current interference.</summary>
    public bool More() => _iterator.More();

    /// <summary>Moves to the next matching interference.</summary>
    public void Next()
    {
        if (_iterator.More())
        {
            _iterator.Next();
            Match();
        }
    }

    /// <summary>The current interference.</summary>
    public DsInterference Value() => _iterator.Value();

    /// <summary>The underlying list iterator.</summary>
    internal NCollection_List<DsInterference>.Iterator ChangeIterator() => _iterator;
}
