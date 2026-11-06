// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_Collect.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_Collect.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Topology;
using Kerf.Topology;

namespace Kerf.Engine.Construction.Api;

/// <summary>
/// Collects the modifications and generations of a chain of algorithms on an initial shape, so that each image is
/// recorded against the initial sub-shape it came from.
/// </summary>
/// <remarks>
/// A working object (decision 21). It reads the algorithms' lists through their internal forms
/// (<c>InternalModified</c>, <c>InternalGenerated</c>), so the shapes it records aren't frozen by it.
/// <see cref="Modification"/> and <see cref="Generated"/> return its own maps, as OCCT's references.
/// </remarks>
internal sealed class Collect
{
    private readonly Dictionary<Shape, NCollection_List<Shape>> _mod =
        new(ShapeHasher.Instance);
    private readonly Dictionary<Shape, NCollection_List<Shape>> _gen =
        new(ShapeHasher.Instance);

    // OCCT's never-used myInitialShape and myDeleted are left out.

    public Collect()
    {
    }

    private static void BuildBack(Dictionary<Shape, NCollection_List<Shape>> m1,
                                  Dictionary<Shape, Shape> bm1)
    {
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> it in m1)
        {
            Shape ks = it.Key;
            var itl = new NCollection_List<Shape>.Iterator(it.Value);
            for (; itl.More(); itl.Next())
            {
                Shape vs = itl.Value();
                bm1[vs] = ks;
            }
        }
    }

    private static void Replace(NCollection_List<Shape> l, in Shape old,
                                NCollection_List<Shape> New)
    {
        // Suppression de Old dans la liste.
        var it = new NCollection_List<Shape>.Iterator(l);
        while (it.More())
        {
            if (it.Value().IsSame(old))
            {
                l.Remove(it);
                break;
            }
            if (it.More())
            {
                it.Next();
            }
        }

        // Ajout de New a L.
        var copNew = new NCollection_List<Shape>();
        copNew.Assign(New);
        l.Append(copNew);
    }

    private static void StoreImage(Dictionary<Shape, NCollection_List<Shape>> mg,
                                   in Shape s,
                                   Dictionary<Shape, Shape> mgBack,
                                   NCollection_List<Shape> li)
    {
        if (!li.IsEmpty())
        {
            if (mgBack.ContainsKey(s))
            {
                Replace(mg.GetValue(mgBack.GetValue(s)), s, li);
            }
            else
            {
                if (!mg.ContainsKey(s))
                {
                    var empty = new NCollection_List<Shape>();
                    mg[s] = empty;
                }
                // Dans tous les cas on copie la liste pour eviter les pb de
                // const& dans BRepBuilderAPI.
                var it = new NCollection_List<Shape>.Iterator(li);
                for (; it.More(); it.Next())
                {
                    Shape ss = it.Value();
                    mg.GetValue(s).Append(ss);
                }
            }
        }
    }

    private static void Update(Dictionary<Shape, NCollection_List<Shape>> mod,
                               Dictionary<Shape, NCollection_List<Shape>> gen,
                               Dictionary<Shape, Shape> modBack,
                               Dictionary<Shape, Shape> genBack,
                               in Shape si,
                               MakeShape mks,
                               ShapeType shapeType)
    {
        var dejaVu = new HashSet<Shape>(ShapeHasher.Instance);
        var exp = new Explorer();
        for (exp.Init(si, shapeType); exp.More(); exp.Next())
        {
            Shape s = exp.Current();
            if (!dejaVu.Add(s))
            {
                continue;
            }

            // Recuperation de l image de S par MKS.
            NCollection_List<Shape> lim = mks.InternalModified(s);
            if (!lim.IsEmpty())
            {
                if (genBack.ContainsKey(s))
                {
                    // Modif de generation => generation du shape initial
                    StoreImage(gen, s, genBack, lim);
                }
                else
                {
                    StoreImage(mod, s, modBack, lim);
                }
            }
            NCollection_List<Shape> lig = mks.InternalGenerated(s);
            if (!lig.IsEmpty())
            {
                if (modBack.ContainsKey(s))
                {
                    // Generation de modif  => generation du shape initial
                    Shape IS = modBack.GetValue(s);
                    StoreImage(gen, IS, genBack, lig);
                }
                else
                {
                    StoreImage(gen, s, genBack, lig);
                }
            }
        }
    }

    /// <summary>Records the modifications and generations made by <paramref name="mks"/> on <paramref name="si"/>.</summary>
    public void Add(in Shape si, MakeShape mks)
    {
        var genBack = new Dictionary<Shape, Shape>(ShapeHasher.Instance);
        var modBack = new Dictionary<Shape, Shape>(ShapeHasher.Instance);
        BuildBack(_gen, genBack); // Vraiment pas optimum a Revoir
        BuildBack(_mod, modBack);

        Update(_mod, _gen, modBack, genBack, si, mks, ShapeType.Compound);
        Update(_mod, _gen, modBack, genBack, si, mks, ShapeType.CompSolid);
        Update(_mod, _gen, modBack, genBack, si, mks, ShapeType.Solid);
        Update(_mod, _gen, modBack, genBack, si, mks, ShapeType.Shell);
        Update(_mod, _gen, modBack, genBack, si, mks, ShapeType.Face);
        Update(_mod, _gen, modBack, genBack, si, mks, ShapeType.Wire);
        Update(_mod, _gen, modBack, genBack, si, mks, ShapeType.Edge);
        Update(_mod, _gen, modBack, genBack, si, mks, ShapeType.Vertex);
    }

    /// <summary>Records <paramref name="gen"/> as generated from <paramref name="s"/>.</summary>
    public void AddGenerated(in Shape s, in Shape gen)
    {
        var genBack = new Dictionary<Shape, Shape>(ShapeHasher.Instance);
        var modBack = new Dictionary<Shape, Shape>(ShapeHasher.Instance);
        BuildBack(_gen, genBack);
        BuildBack(_mod, modBack);

        var lig = new NCollection_List<Shape>();
        lig.Append(gen);
        if (modBack.ContainsKey(s))
        {
            // Generation de modif  => generation du shape initial
            Shape IS = modBack.GetValue(s);
            StoreImage(_gen, IS, genBack, lig);
        }
        else
        {
            StoreImage(_gen, s, genBack, lig);
        }
    }

    /// <summary>Records <paramref name="mod"/> as modified from <paramref name="s"/>.</summary>
    public void AddModif(in Shape s, in Shape mod)
    {
        var genBack = new Dictionary<Shape, Shape>(ShapeHasher.Instance);
        var modBack = new Dictionary<Shape, Shape>(ShapeHasher.Instance);
        BuildBack(_gen, genBack);
        BuildBack(_mod, modBack);

        var lig = new NCollection_List<Shape>();
        lig.Append(mod);
        if (genBack.ContainsKey(s))
        {
            // Modif de generation => generation du shape initial
            StoreImage(_gen, s, genBack, lig);
        }
        else
        {
            StoreImage(_mod, s, modBack, lig);
        }
    }

    private static void FilterByShape(Dictionary<Shape, NCollection_List<Shape>> mg,
                                      in Shape sf)
    {
        var msf = new HashSet<Shape>(ShapeHasher.Instance);
        var exp = new Explorer();
        bool yaEdge = false;
        bool yaVertex = false;
        for (exp.Init(sf, ShapeType.Face); exp.More(); exp.Next())
        {
            msf.Add(exp.Current());
        }

        // Suppression de toutes les images qui ne sont pas dans MSF.
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> it in mg)
        {
            NCollection_List<Shape> lns = it.Value; // MG.ChangeFind(it.Key())
            var itl = new NCollection_List<Shape>.Iterator(lns);
            while (itl.More())
            {
                Shape ns = itl.Value();
                // Images contiennet des edges => ajout des edges resultat dans MSF.
                if (!yaEdge && ns.Type == ShapeType.Edge)
                {
                    for (exp.Init(sf, ShapeType.Edge); exp.More(); exp.Next())
                    {
                        msf.Add(exp.Current());
                    }
                    yaEdge = true;
                }
                // Images contiennet des vertex => ajout des vertex resultat dans MSF.
                if (!yaVertex && ns.Type == ShapeType.Vertex)
                {
                    for (exp.Init(sf, ShapeType.Vertex); exp.More(); exp.Next())
                    {
                        msf.Add(exp.Current());
                    }
                    yaVertex = true;
                }
                // Si pas dans MSF suprresion de l image.
                if (!msf.Contains(ns))
                {
                    lns.Remove(itl);
                }
                else if (itl.More())
                {
                    itl.Next();
                }
            }
        }
    }

    /// <summary>The modifications: each initial sub-shape and its images (the collector's own map).</summary>
    public Dictionary<Shape, NCollection_List<Shape>> Modification()
    {
        return _mod;
    }

    /// <summary>The generations: each initial sub-shape and what it generated (the collector's own map).</summary>
    public Dictionary<Shape, NCollection_List<Shape>> Generated()
    {
        return _gen;
    }

    /// <summary>Keeps only the images that are sub-shapes of <paramref name="sf"/>.</summary>
    public void Filter(in Shape sf)
    {
        FilterByShape(_gen, sf);
        FilterByShape(_mod, sf);
    }
}
