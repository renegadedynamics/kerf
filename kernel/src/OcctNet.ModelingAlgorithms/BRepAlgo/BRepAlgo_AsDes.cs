// Port of: src/ModelingAlgorithms/TKBool/BRepAlgo/BRepAlgo_AsDes.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepAlgo/BRepAlgo_AsDes.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Topology;

namespace Occt;

/// <summary>Stores the future sub-shapes (descendants) of shapes and the shapes containing them (ascendants).</summary>
/// <remarks>
/// A working object of the offset algorithms. For a shape without ascendants or descendants OCCT returns a shared
/// static empty list; the port's is per thread (<c>[ThreadStatic]</c>), so <see cref="ChangeDescendant"/> on such a
/// shape still hands out one list that every later call sees, as in OCCT (marked <c>// OCCT</c>).
/// </remarks>
public sealed class BRepAlgo_AsDes : Transient
{
    private readonly NCollection_DataMap<Shape, NCollection_List<Shape>> up = new(ShapeHasher.Instance);
    private readonly NCollection_DataMap<Shape, NCollection_List<Shape>> down = new(ShapeHasher.Instance);

    [ThreadStatic]
    private static NCollection_List<Shape>? theEmpty;

    // OCCT: the function-level static empty list, shared and writable through ChangeDescendant
    private static NCollection_List<Shape> empty() => theEmpty ??= new NCollection_List<Shape>();

    /// <summary>An empty store.</summary>
    public BRepAlgo_AsDes()
    {
    }

    /// <summary>Removes everything.</summary>
    public void Clear()
    {
        up.Clear();
        down.Clear();
    }

    /// <summary>Stores <paramref name="SS"/> as a future sub-shape of <paramref name="S"/>.</summary>
    public void Add(in Shape S, in Shape SS)
    {
        if (!down.IsBound(S))
        {
            NCollection_List<Shape> L = new();
            down.Bind(S, L);
        }
        down[S].Append(SS);

        if (!up.IsBound(SS))
        {
            NCollection_List<Shape> L = new();
            up.Bind(SS, L);
        }
        up[SS].Append(S);
    }

    /// <summary>Stores the shapes of <paramref name="SS"/> as future sub-shapes of <paramref name="S"/>.</summary>
    public void Add(in Shape S, NCollection_List<Shape> SS)
    {
        NCollection_List<Shape>.Iterator it = new(SS);
        for (; it.More(); it.Next())
        {
            Add(S, it.Value());
        }
    }

    /// <summary>Whether <paramref name="S"/> has ascendants.</summary>
    public bool HasAscendant(in Shape S) => up.IsBound(S);

    /// <summary>Whether <paramref name="S"/> has descendants.</summary>
    public bool HasDescendant(in Shape S) => down.IsBound(S);

    /// <summary>The shapes containing <paramref name="S"/>.</summary>
    public NCollection_List<Shape> Ascendant(in Shape S)
    {
        if (up.IsBound(S))
        {
            return up[S];
        }
        return empty();
    }

    /// <summary>The future sub-shapes of <paramref name="S"/>.</summary>
    public NCollection_List<Shape> Descendant(in Shape S)
    {
        if (down.IsBound(S))
        {
            return down[S];
        }
        return empty();
    }

    /// <summary>The future sub-shapes of <paramref name="S"/>, to change.</summary>
    public NCollection_List<Shape> ChangeDescendant(in Shape S)
    {
        if (down.IsBound(S))
        {
            return down.ChangeFind(S);
        }
        return empty(); // OCCT: the shared empty list, which a caller can fill
    }

    // Replaces OldS by NewS in L (with OldS's orientation), once per distinct shape (IsEqual)
    private static void ReplaceInList(in Shape OldS, in Shape NewS, NCollection_List<Shape> L)
    {
        NCollection_Map<Shape> aMS = new(EqualityComparer<Shape>.Default);
        NCollection_List<Shape>.Iterator it = new(L);
        for (; it.More(); it.Next())
        {
            aMS.Add(it.Value());
        }
        it.Initialize(L);
        while (it.More())
        {
            if (it.Value().IsSame(OldS))
            {
                Orientation O = it.Value().Orientation();
                if (aMS.Add(NewS.Oriented(O)))
                {
                    L.InsertBefore(NewS.Oriented(O), it);
                }
                L.Remove(it);
            }
            else
            {
                it.Next();
            }
        }
    }

    private static void RemoveInList(in Shape S, NCollection_List<Shape> L)
    {
        NCollection_List<Shape>.Iterator it = new(L);
        while (it.More())
        {
            if (it.Value().IsSame(S))
            {
                L.Remove(it);
                break;
            }
            it.Next();
        }
    }

    /// <summary>
    /// Whether <paramref name="S1"/> and <paramref name="S2"/> have common descendants, stored in <paramref name="LC"/>.
    /// </summary>
    public bool HasCommonDescendant(in Shape S1, in Shape S2, NCollection_List<Shape> LC)
    {
        LC.Clear();
        if (HasDescendant(S1) && HasDescendant(S2))
        {
            NCollection_List<Shape>.Iterator it1 = new(Descendant(S1));
            for (; it1.More(); it1.Next())
            {
                Shape DS1 = it1.Value();
                NCollection_List<Shape>.Iterator it2 = new(Ascendant(DS1));
                for (; it2.More(); it2.Next())
                {
                    Shape ADS1 = it2.Value();
                    if (ADS1.IsSame(S2))
                    {
                        LC.Append(DS1);
                    }
                }
            }
        }
        return !LC.IsEmpty();
    }

    private void BackReplace(in Shape OldS, in Shape NewS, NCollection_List<Shape> L, bool InUp)
    {
        NCollection_List<Shape>.Iterator it = new(L);
        for (; it.More(); it.Next())
        {
            Shape S = it.Value();
            if (InUp)
            {
                if (up.IsBound(S))
                {
                    ReplaceInList(OldS, NewS, up.ChangeFind(S));
                }
            }
            else
            {
                if (down.IsBound(S))
                {
                    ReplaceInList(OldS, NewS, down.ChangeFind(S));
                }
            }
        }
    }

    /// <summary>Replaces <paramref name="theOldS"/> by <paramref name="theNewS"/>; the old shape disappears.</summary>
    public void Replace(in Shape theOldS, in Shape theNewS)
    {
        for (int i = 0; i < 2; ++i)
        {
            NCollection_DataMap<Shape, NCollection_List<Shape>> aMap = i == 0 ? up : down;
            if (!aMap.Find(theOldS, out NCollection_List<Shape> pLSOld))
            {
                continue;
            }
            //
            bool InUp = i != 0;
            BackReplace(theOldS, theNewS, pLSOld, InUp);
            //
            if (!aMap.Find(theNewS, out NCollection_List<Shape> pLSNew))
            {
                // filter the list
                NCollection_Map<Shape> aMS = new(EqualityComparer<Shape>.Default);
                NCollection_List<Shape>.Iterator aIt = new(pLSOld);
                for (; aIt.More();)
                {
                    if (!aMS.Add(aIt.Value()))
                    {
                        pLSOld.Remove(aIt);
                    }
                    else
                    {
                        aIt.Next();
                    }
                }
                aMap.Bind(theNewS, new NCollection_List<Shape>(pLSOld));
            }
            else
            {
                // avoid duplicates
                NCollection_Map<Shape> aMS = new(EqualityComparer<Shape>.Default);
                NCollection_List<Shape>.Iterator aIt = new(pLSNew);
                for (; aIt.More(); aIt.Next())
                {
                    aMS.Add(aIt.Value());
                }
                //
                aIt.Initialize(pLSOld);
                for (; aIt.More(); aIt.Next())
                {
                    Shape aS = aIt.Value();
                    if (aMS.Add(aS))
                    {
                        pLSNew.Append(aS);
                    }
                }
            }
            //
            aMap.UnBind(theOldS);
        }
    }

    /// <summary>Removes <paramref name="SS"/>, which must have ascendants and no descendants.</summary>
    /// <exception cref="ConstructionErrorException">It has descendants, or no ascendants.</exception>
    public void Remove(in Shape SS)
    {
        if (down.IsBound(SS))
        {
            throw new ConstructionErrorException(" BRepAlgo_AsDes::Remove");
        }
        if (!up.IsBound(SS))
        {
            throw new ConstructionErrorException(" BRepAlgo_AsDes::Remove");
        }
        NCollection_List<Shape>.Iterator it = new(up[SS]);
        for (; it.More(); it.Next())
        {
            RemoveInList(SS, down.ChangeFind(it.Value()));
        }
        up.UnBind(SS);
    }
}
