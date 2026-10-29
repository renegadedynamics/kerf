// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_Collect.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_Collect.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_ShapeEnum;

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
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myMod =
        new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myGen =
        new(TopTools_ShapeMapHasher.Instance);

    // OCCT's never-used myInitialShape and myDeleted are left out.

    public BRepBuilderAPI_Collect()
    {
    }

    private static void BuildBack(NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> M1,
                                  NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> BM1)
    {
        foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> it in M1)
        {
            TopoDS_Shape KS = it.Key;
            var itl = new NCollection_List<TopoDS_Shape>.Iterator(it.Value);
            for (; itl.More(); itl.Next())
            {
                TopoDS_Shape VS = itl.Value();
                BM1.Bind(VS, KS);
            }
        }
    }

    private static void Replace(NCollection_List<TopoDS_Shape> L, in TopoDS_Shape Old,
                                NCollection_List<TopoDS_Shape> New)
    {
        // Suppression de Old dans la liste.
        var it = new NCollection_List<TopoDS_Shape>.Iterator(L);
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
        var copNew = new NCollection_List<TopoDS_Shape>();
        copNew.Assign(New);
        L.Append(copNew);
    }

    private static void StoreImage(NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> MG,
                                   in TopoDS_Shape S,
                                   NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> MGBack,
                                   NCollection_List<TopoDS_Shape> LI)
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
                    var empty = new NCollection_List<TopoDS_Shape>();
                    MG.Bind(S, empty);
                }
                // Dans tous les cas on copie la liste pour eviter les pb de
                // const& dans BRepBuilderAPI.
                var it = new NCollection_List<TopoDS_Shape>.Iterator(LI);
                for (; it.More(); it.Next())
                {
                    TopoDS_Shape SS = it.Value();
                    MG[S].Append(SS);
                }
            }
        }
    }

    private static void Update(NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> Mod,
                               NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> Gen,
                               NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> ModBack,
                               NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> GenBack,
                               in TopoDS_Shape SI,
                               BRepBuilderAPI_MakeShape MKS,
                               TopAbs_ShapeEnum ShapeType)
    {
        var DejaVu = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        var exp = new TopExp_Explorer();
        for (exp.Init(SI, ShapeType); exp.More(); exp.Next())
        {
            TopoDS_Shape S = exp.Current();
            if (!DejaVu.Add(S))
            {
                continue;
            }

            // Recuperation de l image de S par MKS.
            NCollection_List<TopoDS_Shape> LIM = MKS.InternalModified(S);
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
            NCollection_List<TopoDS_Shape> LIG = MKS.InternalGenerated(S);
            if (!LIG.IsEmpty())
            {
                if (ModBack.IsBound(S))
                {
                    // Generation de modif  => generation du shape initial
                    TopoDS_Shape IS = ModBack[S];
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
    public void Add(in TopoDS_Shape SI, BRepBuilderAPI_MakeShape MKS)
    {
        var GenBack = new NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        var ModBack = new NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        BuildBack(myGen, GenBack); // Vraiment pas optimum a Revoir
        BuildBack(myMod, ModBack);

        Update(myMod, myGen, ModBack, GenBack, SI, MKS, TopAbs_COMPOUND);
        Update(myMod, myGen, ModBack, GenBack, SI, MKS, TopAbs_COMPSOLID);
        Update(myMod, myGen, ModBack, GenBack, SI, MKS, TopAbs_SOLID);
        Update(myMod, myGen, ModBack, GenBack, SI, MKS, TopAbs_SHELL);
        Update(myMod, myGen, ModBack, GenBack, SI, MKS, TopAbs_FACE);
        Update(myMod, myGen, ModBack, GenBack, SI, MKS, TopAbs_WIRE);
        Update(myMod, myGen, ModBack, GenBack, SI, MKS, TopAbs_EDGE);
        Update(myMod, myGen, ModBack, GenBack, SI, MKS, TopAbs_VERTEX);
    }

    /// <summary>Records <paramref name="Gen"/> as generated from <paramref name="S"/>.</summary>
    public void AddGenerated(in TopoDS_Shape S, in TopoDS_Shape Gen)
    {
        var GenBack = new NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        var ModBack = new NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        BuildBack(myGen, GenBack);
        BuildBack(myMod, ModBack);

        var LIG = new NCollection_List<TopoDS_Shape>();
        LIG.Append(Gen);
        if (ModBack.IsBound(S))
        {
            // Generation de modif  => generation du shape initial
            TopoDS_Shape IS = ModBack[S];
            StoreImage(myGen, IS, GenBack, LIG);
        }
        else
        {
            StoreImage(myGen, S, GenBack, LIG);
        }
    }

    /// <summary>Records <paramref name="Mod"/> as modified from <paramref name="S"/>.</summary>
    public void AddModif(in TopoDS_Shape S, in TopoDS_Shape Mod)
    {
        var GenBack = new NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        var ModBack = new NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        BuildBack(myGen, GenBack);
        BuildBack(myMod, ModBack);

        var LIG = new NCollection_List<TopoDS_Shape>();
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

    private static void FilterByShape(NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> MG,
                                      in TopoDS_Shape SF)
    {
        var MSF = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        var exp = new TopExp_Explorer();
        bool YaEdge = false;
        bool YaVertex = false;
        for (exp.Init(SF, TopAbs_FACE); exp.More(); exp.Next())
        {
            MSF.Add(exp.Current());
        }

        // Suppression de toutes les images qui ne sont pas dans MSF.
        foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> it in MG)
        {
            NCollection_List<TopoDS_Shape> LNS = it.Value; // MG.ChangeFind(it.Key())
            var itl = new NCollection_List<TopoDS_Shape>.Iterator(LNS);
            while (itl.More())
            {
                TopoDS_Shape NS = itl.Value();
                // Images contiennet des edges => ajout des edges resultat dans MSF.
                if (!YaEdge && NS.ShapeType() == TopAbs_EDGE)
                {
                    for (exp.Init(SF, TopAbs_EDGE); exp.More(); exp.Next())
                    {
                        MSF.Add(exp.Current());
                    }
                    YaEdge = true;
                }
                // Images contiennet des vertex => ajout des vertex resultat dans MSF.
                if (!YaVertex && NS.ShapeType() == TopAbs_VERTEX)
                {
                    for (exp.Init(SF, TopAbs_VERTEX); exp.More(); exp.Next())
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
    public NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> Modification()
    {
        return myMod;
    }

    /// <summary>The generations: each initial sub-shape and what it generated (the collector's own map).</summary>
    public NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> Generated()
    {
        return myGen;
    }

    /// <summary>Keeps only the images that are sub-shapes of <paramref name="SF"/>.</summary>
    public void Filter(in TopoDS_Shape SF)
    {
        FilterByShape(myGen, SF);
        FilterByShape(myMod, SF);
    }
}
