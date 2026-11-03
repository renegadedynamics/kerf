// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_Collect.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_Collect.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// Collects the modifications and generations of a chain of algorithms on an initial shape, so that each image is
/// recorded against the initial sub-shape it came from.
/// </summary>
/// <remarks>
/// A working object (decision 21). It reads the algorithms' lists through their internal forms
/// (<c>InternalModified</c>, <c>InternalGenerated</c>), so the shapes it records aren't frozen by it.
/// <see cref="Modification"/> and <see cref="Generated"/> return its own maps, as OCCT's references.
/// </remarks>
public sealed class BRepBuilderAPI_Collect
{
    private readonly NCollection_DataMap<Shape, NCollection_List<Shape>> myMod =
        new(ShapeHasher.Instance);
    private readonly NCollection_DataMap<Shape, NCollection_List<Shape>> myGen =
        new(ShapeHasher.Instance);

    // OCCT's never-used myInitialShape and myDeleted are left out.

    public BRepBuilderAPI_Collect()
    {
    }

    private static void BuildBack(NCollection_DataMap<Shape, NCollection_List<Shape>> M1,
                                  NCollection_DataMap<Shape, Shape> BM1)
    {
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> it in M1)
        {
            Shape KS = it.Key;
            var itl = new NCollection_List<Shape>.Iterator(it.Value);
            for (; itl.More(); itl.Next())
            {
                Shape VS = itl.Value();
                BM1.Bind(VS, KS);
            }
        }
    }

    private static void Replace(NCollection_List<Shape> L, in Shape Old,
                                NCollection_List<Shape> New)
    {
        // Suppression de Old dans la liste.
        var it = new NCollection_List<Shape>.Iterator(L);
        while (it.More())
        {
            if (it.Value().IsSame(Old))
            {
                L.Remove(it);
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
        L.Append(copNew);
    }

    private static void StoreImage(NCollection_DataMap<Shape, NCollection_List<Shape>> MG,
                                   in Shape S,
                                   NCollection_DataMap<Shape, Shape> MGBack,
                                   NCollection_List<Shape> LI)
    {
        if (!LI.IsEmpty())
        {
            if (MGBack.IsBound(S))
            {
                Replace(MG.ChangeFind(MGBack[S]), S, LI);
            }
            else
            {
                if (!MG.IsBound(S))
                {
                    var empty = new NCollection_List<Shape>();
                    MG.Bind(S, empty);
                }
                // Dans tous les cas on copie la liste pour eviter les pb de
                // const& dans BRepBuilderAPI.
                var it = new NCollection_List<Shape>.Iterator(LI);
                for (; it.More(); it.Next())
                {
                    Shape SS = it.Value();
                    MG[S].Append(SS);
                }
            }
        }
    }

    private static void Update(NCollection_DataMap<Shape, NCollection_List<Shape>> Mod,
                               NCollection_DataMap<Shape, NCollection_List<Shape>> Gen,
                               NCollection_DataMap<Shape, Shape> ModBack,
                               NCollection_DataMap<Shape, Shape> GenBack,
                               in Shape SI,
                               BRepBuilderAPI_MakeShape MKS,
                               ShapeType ShapeType)
    {
        var DejaVu = new NCollection_Map<Shape>(ShapeHasher.Instance);
        var exp = new Explorer();
        for (exp.Init(SI, ShapeType); exp.More(); exp.Next())
        {
            Shape S = exp.Current();
            if (!DejaVu.Add(S))
            {
                continue;
            }

            // Recuperation de l image de S par MKS.
            NCollection_List<Shape> LIM = MKS.InternalModified(S);
            if (!LIM.IsEmpty())
            {
                if (GenBack.IsBound(S))
                {
                    // Modif de generation => generation du shape initial
                    StoreImage(Gen, S, GenBack, LIM);
                }
                else
                {
                    StoreImage(Mod, S, ModBack, LIM);
                }
            }
            NCollection_List<Shape> LIG = MKS.InternalGenerated(S);
            if (!LIG.IsEmpty())
            {
                if (ModBack.IsBound(S))
                {
                    // Generation de modif  => generation du shape initial
                    Shape IS = ModBack[S];
                    StoreImage(Gen, IS, GenBack, LIG);
                }
                else
                {
                    StoreImage(Gen, S, GenBack, LIG);
                }
            }
        }
    }

    /// <summary>Records the modifications and generations made by <paramref name="MKS"/> on <paramref name="SI"/>.</summary>
    public void Add(in Shape SI, BRepBuilderAPI_MakeShape MKS)
    {
        var GenBack = new NCollection_DataMap<Shape, Shape>(ShapeHasher.Instance);
        var ModBack = new NCollection_DataMap<Shape, Shape>(ShapeHasher.Instance);
        BuildBack(myGen, GenBack); // Vraiment pas optimum a Revoir
        BuildBack(myMod, ModBack);

        Update(myMod, myGen, ModBack, GenBack, SI, MKS, ShapeType.Compound);
        Update(myMod, myGen, ModBack, GenBack, SI, MKS, ShapeType.CompSolid);
        Update(myMod, myGen, ModBack, GenBack, SI, MKS, ShapeType.Solid);
        Update(myMod, myGen, ModBack, GenBack, SI, MKS, ShapeType.Shell);
        Update(myMod, myGen, ModBack, GenBack, SI, MKS, ShapeType.Face);
        Update(myMod, myGen, ModBack, GenBack, SI, MKS, ShapeType.Wire);
        Update(myMod, myGen, ModBack, GenBack, SI, MKS, ShapeType.Edge);
        Update(myMod, myGen, ModBack, GenBack, SI, MKS, ShapeType.Vertex);
    }

    /// <summary>Records <paramref name="Gen"/> as generated from <paramref name="S"/>.</summary>
    public void AddGenerated(in Shape S, in Shape Gen)
    {
        var GenBack = new NCollection_DataMap<Shape, Shape>(ShapeHasher.Instance);
        var ModBack = new NCollection_DataMap<Shape, Shape>(ShapeHasher.Instance);
        BuildBack(myGen, GenBack);
        BuildBack(myMod, ModBack);

        var LIG = new NCollection_List<Shape>();
        LIG.Append(Gen);
        if (ModBack.IsBound(S))
        {
            // Generation de modif  => generation du shape initial
            Shape IS = ModBack[S];
            StoreImage(myGen, IS, GenBack, LIG);
        }
        else
        {
            StoreImage(myGen, S, GenBack, LIG);
        }
    }

    /// <summary>Records <paramref name="Mod"/> as modified from <paramref name="S"/>.</summary>
    public void AddModif(in Shape S, in Shape Mod)
    {
        var GenBack = new NCollection_DataMap<Shape, Shape>(ShapeHasher.Instance);
        var ModBack = new NCollection_DataMap<Shape, Shape>(ShapeHasher.Instance);
        BuildBack(myGen, GenBack);
        BuildBack(myMod, ModBack);

        var LIG = new NCollection_List<Shape>();
        LIG.Append(Mod);
        if (GenBack.IsBound(S))
        {
            // Modif de generation => generation du shape initial
            StoreImage(myGen, S, GenBack, LIG);
        }
        else
        {
            StoreImage(myMod, S, ModBack, LIG);
        }
    }

    private static void FilterByShape(NCollection_DataMap<Shape, NCollection_List<Shape>> MG,
                                      in Shape SF)
    {
        var MSF = new NCollection_Map<Shape>(ShapeHasher.Instance);
        var exp = new Explorer();
        bool YaEdge = false;
        bool YaVertex = false;
        for (exp.Init(SF, ShapeType.Face); exp.More(); exp.Next())
        {
            MSF.Add(exp.Current());
        }

        // Suppression de toutes les images qui ne sont pas dans MSF.
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> it in MG)
        {
            NCollection_List<Shape> LNS = it.Value; // MG.ChangeFind(it.Key())
            var itl = new NCollection_List<Shape>.Iterator(LNS);
            while (itl.More())
            {
                Shape NS = itl.Value();
                // Images contiennet des edges => ajout des edges resultat dans MSF.
                if (!YaEdge && NS.ShapeType() == ShapeType.Edge)
                {
                    for (exp.Init(SF, ShapeType.Edge); exp.More(); exp.Next())
                    {
                        MSF.Add(exp.Current());
                    }
                    YaEdge = true;
                }
                // Images contiennet des vertex => ajout des vertex resultat dans MSF.
                if (!YaVertex && NS.ShapeType() == ShapeType.Vertex)
                {
                    for (exp.Init(SF, ShapeType.Vertex); exp.More(); exp.Next())
                    {
                        MSF.Add(exp.Current());
                    }
                    YaVertex = true;
                }
                // Si pas dans MSF suprresion de l image.
                if (!MSF.Contains(NS))
                {
                    LNS.Remove(itl);
                }
                else if (itl.More())
                {
                    itl.Next();
                }
            }
        }
    }

    /// <summary>The modifications: each initial sub-shape and its images (the collector's own map).</summary>
    public NCollection_DataMap<Shape, NCollection_List<Shape>> Modification()
    {
        return myMod;
    }

    /// <summary>The generations: each initial sub-shape and what it generated (the collector's own map).</summary>
    public NCollection_DataMap<Shape, NCollection_List<Shape>> Generated()
    {
        return myGen;
    }

    /// <summary>Keeps only the images that are sub-shapes of <paramref name="SF"/>.</summary>
    public void Filter(in Shape SF)
    {
        FilterByShape(myGen, SF);
        FilterByShape(myMod, SF);
    }
}
