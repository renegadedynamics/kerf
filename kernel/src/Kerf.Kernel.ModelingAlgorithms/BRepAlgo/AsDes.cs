// Port of: src/ModelingAlgorithms/TKBool/BRepAlgo/BRepAlgo_AsDes.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepAlgo/BRepAlgo_AsDes.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Booleans.Algo;

/// <summary>Stores the future sub-shapes (descendants) of shapes and the shapes containing them (ascendants).</summary>
/// <remarks>
/// A working object of the offset algorithms. For a shape without ascendants or descendants OCCT returns a shared
/// static empty list; the port's is per thread (<c>[ThreadStatic]</c>), so <see cref="ChangeDescendant"/> on such a
/// shape still hands out one list that every later call sees, as in OCCT (marked <c>// OCCT</c>).
/// </remarks>
public sealed class AsDes : Transient
{
    private readonly NCollection_DataMap<Shape, NCollection_List<Shape>> _up = new(ShapeHasher.Instance);
    private readonly NCollection_DataMap<Shape, NCollection_List<Shape>> _down = new(ShapeHasher.Instance);

    [ThreadStatic]
    private static NCollection_List<Shape>? s_empty;

    // OCCT: the function-level static empty list, shared and writable through ChangeDescendant
    private static NCollection_List<Shape> Empty() => s_empty ??= new NCollection_List<Shape>();

    /// <summary>An empty store.</summary>
    public AsDes()
    {
    }

    /// <summary>Removes everything.</summary>
    public void Clear()
    {
        _up.Clear();
        _down.Clear();
    }

    /// <summary>Stores <paramref name="ss"/> as a future sub-shape of <paramref name="s"/>.</summary>
    public void Add(in Shape s, in Shape ss)
    {
        if (!_down.IsBound(s))
        {
            NCollection_List<Shape> l = new();
            _down.Bind(s, l);
        }
        _down[s].Append(ss);

        if (!_up.IsBound(ss))
        {
            NCollection_List<Shape> l = new();
            _up.Bind(ss, l);
        }
        _up[ss].Append(s);
    }

    /// <summary>Stores the shapes of <paramref name="ss"/> as future sub-shapes of <paramref name="s"/>.</summary>
    public void Add(in Shape s, NCollection_List<Shape> ss)
    {
        NCollection_List<Shape>.Iterator it = new(ss);
        for (; it.More(); it.Next())
        {
            Add(s, it.Value());
        }
    }

    /// <summary>Whether <paramref name="s"/> has ascendants.</summary>
    public bool HasAscendant(in Shape s) => _up.IsBound(s);

    /// <summary>Whether <paramref name="s"/> has descendants.</summary>
    public bool HasDescendant(in Shape s) => _down.IsBound(s);

    /// <summary>The shapes containing <paramref name="s"/>.</summary>
    public NCollection_List<Shape> Ascendant(in Shape s)
    {
        if (_up.IsBound(s))
        {
            return _up[s];
        }
        return Empty();
    }

    /// <summary>The future sub-shapes of <paramref name="s"/>.</summary>
    public NCollection_List<Shape> Descendant(in Shape s)
    {
        if (_down.IsBound(s))
        {
            return _down[s];
        }
        return Empty();
    }

    /// <summary>The future sub-shapes of <paramref name="s"/>, to change.</summary>
    public NCollection_List<Shape> ChangeDescendant(in Shape s)
    {
        if (_down.IsBound(s))
        {
            return _down.ChangeFind(s);
        }
        return Empty(); // OCCT: the shared empty list, which a caller can fill
    }

    // Replaces OldS by NewS in L (with OldS's orientation), once per distinct shape (IsEqual)
    private static void ReplaceInList(in Shape oldS, in Shape newS, NCollection_List<Shape> l)
    {
        NCollection_Map<Shape> ms = new(EqualityComparer<Shape>.Default);
        NCollection_List<Shape>.Iterator it = new(l);
        for (; it.More(); it.Next())
        {
            ms.Add(it.Value());
        }
        it.Initialize(l);
        while (it.More())
        {
            if (it.Value().IsSame(oldS))
            {
                Orientation o = it.Value().Orientation();
                if (ms.Add(newS.Oriented(o)))
                {
                    l.InsertBefore(newS.Oriented(o), it);
                }
                l.Remove(it);
            }
            else
            {
                it.Next();
            }
        }
    }

    private static void RemoveInList(in Shape s, NCollection_List<Shape> l)
    {
        NCollection_List<Shape>.Iterator it = new(l);
        while (it.More())
        {
            if (it.Value().IsSame(s))
            {
                l.Remove(it);
                break;
            }
            it.Next();
        }
    }

    /// <summary>
    /// Whether <paramref name="s1"/> and <paramref name="s2"/> have common descendants, stored in <paramref name="lc"/>.
    /// </summary>
    public bool HasCommonDescendant(in Shape s1, in Shape s2, NCollection_List<Shape> lc)
    {
        lc.Clear();
        if (HasDescendant(s1) && HasDescendant(s2))
        {
            NCollection_List<Shape>.Iterator it1 = new(Descendant(s1));
            for (; it1.More(); it1.Next())
            {
                Shape ds1 = it1.Value();
                NCollection_List<Shape>.Iterator it2 = new(Ascendant(ds1));
                for (; it2.More(); it2.Next())
                {
                    Shape ads1 = it2.Value();
                    if (ads1.IsSame(s2))
                    {
                        lc.Append(ds1);
                    }
                }
            }
        }
        return !lc.IsEmpty();
    }

    private void BackReplace(in Shape oldS, in Shape newS, NCollection_List<Shape> l, bool inUp)
    {
        NCollection_List<Shape>.Iterator it = new(l);
        for (; it.More(); it.Next())
        {
            Shape s = it.Value();
            if (inUp)
            {
                if (_up.IsBound(s))
                {
                    ReplaceInList(oldS, newS, _up.ChangeFind(s));
                }
            }
            else
            {
                if (_down.IsBound(s))
                {
                    ReplaceInList(oldS, newS, _down.ChangeFind(s));
                }
            }
        }
    }

    /// <summary>Replaces <paramref name="oldS"/> by <paramref name="newS"/>; the old shape disappears.</summary>
    public void Replace(in Shape oldS, in Shape newS)
    {
        for (int i = 0; i < 2; ++i)
        {
            NCollection_DataMap<Shape, NCollection_List<Shape>> map = i == 0 ? _up : _down;
            if (!map.Find(oldS, out NCollection_List<Shape> pLSOld))
            {
                continue;
            }
            //
            bool inUp = i != 0;
            BackReplace(oldS, newS, pLSOld, inUp);
            //
            if (!map.Find(newS, out NCollection_List<Shape> pLSNew))
            {
                // filter the list
                NCollection_Map<Shape> ms = new(EqualityComparer<Shape>.Default);
                NCollection_List<Shape>.Iterator it = new(pLSOld);
                for (; it.More();)
                {
                    if (!ms.Add(it.Value()))
                    {
                        pLSOld.Remove(it);
                    }
                    else
                    {
                        it.Next();
                    }
                }
                map.Bind(newS, new NCollection_List<Shape>(pLSOld));
            }
            else
            {
                // avoid duplicates
                NCollection_Map<Shape> ms = new(EqualityComparer<Shape>.Default);
                NCollection_List<Shape>.Iterator it = new(pLSNew);
                for (; it.More(); it.Next())
                {
                    ms.Add(it.Value());
                }
                //
                it.Initialize(pLSOld);
                for (; it.More(); it.Next())
                {
                    Shape s = it.Value();
                    if (ms.Add(s))
                    {
                        pLSNew.Append(s);
                    }
                }
            }
            //
            map.UnBind(oldS);
        }
    }

    /// <summary>Removes <paramref name="ss"/>, which must have ascendants and no descendants.</summary>
    /// <exception cref="ConstructionErrorException">It has descendants, or no ascendants.</exception>
    public void Remove(in Shape ss)
    {
        if (_down.IsBound(ss))
        {
            throw new ConstructionErrorException(" BRepAlgo_AsDes::Remove");
        }
        if (!_up.IsBound(ss))
        {
            throw new ConstructionErrorException(" BRepAlgo_AsDes::Remove");
        }
        NCollection_List<Shape>.Iterator it = new(_up[ss]);
        for (; it.More(); it.Next())
        {
            RemoveInList(ss, _down.ChangeFind(it.Value()));
        }
        _up.UnBind(ss);
    }
}
