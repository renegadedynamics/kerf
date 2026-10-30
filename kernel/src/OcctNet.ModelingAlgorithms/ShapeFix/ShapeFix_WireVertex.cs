// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_WireVertex.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_WireVertex.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// szv#4 S4163

using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>
/// Fixes disconnected edges in a wire: fixes the vertices of the wire on the basis of the analysis made by
/// <see cref="ShapeAnalysis_WireVertex"/>.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21). <see cref="Fix"/> and <see cref="FixSame"/> write into the edges of the wire data
/// in place (vertices removed and added, tolerances and parameters raised), as OCCT's. When the loaded edges hold
/// frozen shapes (any wire a caller holds), the first fix copies them (<see cref="BRepTools_CopyAtEntry"/>, decision
/// 583) and works on the copies, so the caller's wire never changes; <see cref="Wire"/> gives back the caller's own
/// edges and vertices wherever the fixes left them unchanged, and freezes. A fix after <see cref="Wire"/> copies
/// again.
/// </para>
/// <para>
/// OCCT copies the analyser given to <see cref="Init(ShapeAnalysis_WireVertex)"/> (a value whose arrays and wire data
/// are handles, so the copy shares them); here the analyser is shared.
/// </para>
/// </remarks>
public sealed class ShapeFix_WireVertex
{
    private ShapeAnalysis_WireVertex myAnalyzer;

    // OCCT.NET: the copy of the loaded edges, from the first fix to the public result (null when they hold no frozen
    // shape), and whether the loaded edges must be checked before the next fix.
    private BRepTools_CopyAtEntry? myEntry;
    private bool                   myToEnter;

    // ied_modif_for_compil_Nov-19-1998

    /// <summary>Empty constructor.</summary>
    public ShapeFix_WireVertex()
    {
        myAnalyzer = new ShapeAnalysis_WireVertex();
    }

    /// <summary>
    /// Loads the wire, initializes the internal analyzer (ShapeAnalysis_WireVertex) with the given precision and
    /// performs the analysis.
    /// </summary>
    public void Init(in TopoDS_Wire wire, double preci)
    {
        var sbwd = new ShapeExtend_WireData(wire);
        Init(sbwd, preci);
    }

    /// <summary>
    /// Loads the wire data, initializes the internal analyzer (ShapeAnalysis_WireVertex) with the given precision and
    /// performs the analysis.
    /// </summary>
    public void Init(ShapeExtend_WireData sbwd, double preci)
    {
        myAnalyzer.Load(sbwd);
        myAnalyzer.SetPrecision(preci);
        myAnalyzer.Analyze();
        myToEnter = true;
        myEntry   = null;
    }

    /// <summary>Loads all the data on a wire, already analysed by ShapeAnalysis_WireVertex.</summary>
    public void Init(ShapeAnalysis_WireVertex sawv)
    {
        myAnalyzer = sawv;
        myToEnter  = true;
        myEntry    = null;
    }

    /// <summary>Returns the internal analyzer.</summary>
    public ShapeAnalysis_WireVertex Analyzer() => myAnalyzer;

    /// <summary>
    /// Returns the data on the wire (fixed): the working edges (between the first fix and <see cref="Wire"/>, copies of a
    /// frozen wire's edges).
    /// </summary>
    public ShapeExtend_WireData? WireData() => myAnalyzer.WireData();

    /// <summary>
    /// Returns the resulting wire (fixed), frozen: where the loaded edges were frozen, those the fixes left unchanged
    /// are the caller's own.
    /// </summary>
    public TopoDS_Wire Wire()
    {
        endWork();
        return myAnalyzer.WireData()!.Wire();
    }

    /// <summary>The resulting wire, unfrozen, for ported code fixing a wire it owns: its edges are the working ones.</summary>
    internal TopoDS_Wire InternalWire() => myAnalyzer.WireData()!.InternalWire();

    /// <summary>
    /// Fixes the "Same" or "Close" statuses (the same vertex may be set, without changing parameters). Returns the
    /// count of fixed vertices, 0 if none.
    /// </summary>
    public int FixSame()
    {
        //  FixSame : prend les status "SameCoord" et "Close" et les force a "Same"
        //  reprendre l edge et forcer le vertex. Evt changer sa tolerance. Et voila
        if (!myAnalyzer.IsDone())
        {
            return 0;
        }
        enterWork(); // OCCT.NET: frozen

        int nbfix = 0;
        var B     = new BRep_Builder();

        ShapeExtend_WireData sbwd = myAnalyzer.WireData()!;
        int                  i, nb = sbwd.NbEdges();

        for (i = 1; i <= nb; i++)
        {
            int j    = (i == nb ? 1 : i + 1);
            int stat = myAnalyzer.Status(i);
            if (stat != 1 && stat != 2)
            {
                continue;
            }
            // Ici on prend un vertex et on le generalise aux deux edges
            TopoDS_Edge E1 = sbwd.Edge(i);
            TopoDS_Edge E2 = sbwd.Edge(j);

            var           sae = new ShapeAnalysis_Edge();
            TopoDS_Vertex V1  = sae.LastVertex(E1);
            TopoDS_Vertex V2  = sae.FirstVertex(E2);
            if (V1 == V2) // OCCT: == compares orientations; a shared vertex is REVERSED in E1, FORWARD in E2
            {
                myAnalyzer.SetSameVertex(i);
                continue;
            } // deja fait ...
            if (stat == 2)
            {
                // OK mais en reprenant les tolerances
                sae.Curve3d(sbwd.Edge(i), out _, out _, out double cl);
                B.UpdateVertex(V1, cl, E1, myAnalyzer.Precision());
                sae.Curve3d(sbwd.Edge(j), out _, out double cf, out _);
                B.UpdateVertex(V1, cf, E2, myAnalyzer.Precision());
            }
            // Et remettre ce vtx en commun
            // OCCT: unlike Fix, the edges' old vertices aren't removed (the edges end up with both) and the edges aren't
            // made Free first, so on the edges of a wire (not free) B.Add throws TopoDS_FrozenShape
            V1.Orientation(E2.Orientation());
            B.Add(E2, V1);
            V1.Orientation(E1.Orientation());
            V1.Reverse();
            B.Add(E1, V1);
            myAnalyzer.SetSameVertex(i); // conclusion
            nbfix++;
        }
        return nbfix;
    }

    /// <summary>
    /// Fixes all the statuses except "Disjoined", i.e. the cases in which a common value has been set, with or without
    /// changing parameters. Returns the count of fixed vertices, 0 if none.
    /// </summary>
    public int Fix()
    {
        //  Ici le grand jeu : on repasse partout
        //  stat = 0 (OK) ou <0 (KO) : on passe
        //  stat = 1 ou 2 : on reprend le vtx d origine
        //  sinon on en refait un ...

        //  MAIS ATTENTION : on fait du neuf ... forcement. Donc nouvelles edges
        //   auxquelles on remet les Vertex (assez facile)
        //   Donc deux passes : 1 refaire les VTX  et 2 les remettre dans les edges
        if (!myAnalyzer.IsDone())
        {
            return 0;
        }

        ShapeExtend_WireData sbwd = myAnalyzer.WireData()!;

        int i, nb = sbwd.NbEdges();
        int nbfix = 0;
        for (i = 1; i <= nb; i++)
        {
            //    On note les valeurs
            // szv#4:S4163:12Mar99 optimized
            if (myAnalyzer.Status(i) > 0)
            {
                nbfix++;
            }
        }
        if (nbfix == 0)
        {
            return 0;
        }
        enterWork(); // OCCT.NET: frozen

        var B = new BRep_Builder();

        var VI = new NCollection_HArray1<TopoDS_Shape>(1, nb);
        var VJ = new NCollection_HArray1<TopoDS_Shape>(1, nb);
        var EF = new NCollection_HArray1<TopoDS_Shape>(1, nb);
        var UI = new NCollection_HArray1<double>(1, nb);
        var UJ = new NCollection_HArray1<double>(1, nb);

        for (i = 1; i <= nb; i++)
        {
            //    On note les valeurs
            int j    = (i == nb ? 1 : i + 1);
            int stat = myAnalyzer.Status(i);

            var           sae = new ShapeAnalysis_Edge();
            TopoDS_Vertex V1  = sae.LastVertex(sbwd.Edge(i));
            TopoDS_Vertex V2  = sae.FirstVertex(sbwd.Edge(j));
            VI.SetValue(i, V1);
            VJ.SetValue(j, V2);

            TopoDS_Edge E = sbwd.Edge(i);
            //    E.EmptyCopy();   trop d ennuis
            EF.SetValue(i, E);

            double upre = myAnalyzer.UPrevious(i);
            double ufol = myAnalyzer.UFollowing(j);

            // szv#4:S4163:12Mar99 optimized
            if (stat < 4)
            {
                sae.Curve3d(sbwd.Edge(i), out _, out _, out double cl);
                upre = cl;
            }
            if (stat < 3 || stat == 4)
            {
                sae.Curve3d(sbwd.Edge(j), out _, out double cf, out _);
                ufol = cf;
            }

            UI.SetValue(i, upre);
            UJ.SetValue(j, ufol);
        }

        if (nbfix == 0)
        {
            return nbfix;
        }

        // EmptyCopy pas bon : KK sur les Range (dommage, le reste est bon)
        // Donc on garde l original mais on change les vertex
        // En effet, avant de "ajouter" des vertex, il faut enlever ceux d avant
        // Sinon on garde en double !

        for (i = 1; i <= nb; i++)
        {
            TopoDS_Edge E1 = TopoDS.Edge(EF.Value(i));
            E1.Orientation(TopAbs_FORWARD);
            TopExp.Vertices(E1, out TopoDS_Vertex VA, out TopoDS_Vertex VB);
            E1.Free(true);
            B.Remove(E1, VA);
            B.Remove(E1, VB);
        }

        double Prec = myAnalyzer.Precision();
        for (i = 1; i <= nb; i++)
        {
            //    On y va pour de bon
            //    Changer les coords ?
            int j    = (i == nb ? 1 : i + 1);
            int stat = myAnalyzer.Status(i);
            //    if (stat <= 0) continue;

            TopoDS_Vertex V1   = TopoDS.Vertex(VI.Value(i));
            TopoDS_Edge   E1   = TopoDS.Edge(EF.Value(i));
            TopoDS_Edge   E2   = TopoDS.Edge(EF.Value(j));
            double        upre = UI.Value(i);
            double        ufol = UJ.Value(j);

            if (stat > 2)
            {
                B.UpdateVertex(V1, new gp_Pnt(myAnalyzer.Position(i)), Prec);
            }

            //    ce qui suit : seulement si vertex a reprendre
            if (stat > 0)
            {
                B.UpdateVertex(V1, upre, E1, Prec);
                B.UpdateVertex(V1, ufol, E2, Prec);
                V1.Orientation(TopAbs_FORWARD);
                //    V1.Orientation (E2.Orientation());
            }

            //    Comme on a deshabille les edges, il faut tout remettre
            E2.Free(true); // sur place
            B.Add(E2, V1);
            V1.Orientation(TopAbs_REVERSED);
            //    V1.Orientation (E1.Orientation());    V1.Reverse();
            E1.Free(true); // sur place
            B.Add(E1, V1);

            myAnalyzer.SetSameVertex(i); // conclusion
        }

        //  pour finir, MAJ du STW
        for (i = 1; i <= nb; i++)
        {
            sbwd.Set(TopoDS.Edge(EF.Value(i)), i);
        }

        return nbfix;
    }

    // OCCT.NET: before a fix writes, copies the loaded edges if they hold frozen shapes (after Init or a public result).
    private void enterWork()
    {
        if (!myToEnter)
        {
            return;
        }
        myToEnter = false;
        myEntry   = null;
        ShapeExtend_WireData? sbwd = myAnalyzer.WireData();
        if (sbwd is null)
        {
            return;
        }
        TopoDS_Compound anEdges = edgesCompound(sbwd);
        if (!BRepTools.HoldsFrozen(anEdges))
        {
            return;
        }
        myEntry = new BRepTools_CopyAtEntry(anEdges);
        setEdges(sbwd, myEntry.Shape());

        // The copy clears the Free flag of the edges it puts in the compound; give each copy its original's (a cache
        // bit, which KeepUnchanged ignores), so FixSame's B.Add accepts or refuses an edge as OCCT does on the original.
        var anOriginal = new TopoDS_Iterator(anEdges, false, false);
        for (var aCopy = new TopoDS_Iterator(myEntry.Shape(), false, false); aCopy.More(); aCopy.Next(), anOriginal.Next())
        {
            aCopy.Value().TShape()!.Free(anOriginal.Value().TShape()!.Free());
        }
    }

    // OCCT.NET: before the public result, gives back the caller's edges and vertices the fixes left unchanged.
    private void endWork()
    {
        myToEnter = true;
        BRepTools_CopyAtEntry? anEntry = myEntry;
        myEntry = null;
        ShapeExtend_WireData? sbwd = myAnalyzer.WireData();
        if (anEntry is null || sbwd is null)
        {
            return;
        }
        TopoDS_Shape aFinal = anEntry.Finish(edgesCompound(sbwd));
        setEdges(sbwd, aFinal);
    }

    // The edges, then the non-manifold edges, of the wire data, appended to a new compound as they are.
    private static TopoDS_Compound edgesCompound(ShapeExtend_WireData sbwd)
    {
        var aCompound = new TopoDS_Compound();
        new BRep_Builder().MakeCompound(ref aCompound);
        TopoDS_TShape aTShape = aCompound.TShape()!;
        for (int i = 1; i <= sbwd.NbEdges(); i++)
        {
            aTShape.myShapes.Append(sbwd.Edge(i));
        }
        NCollection_HSequence<TopoDS_Shape> aNM = sbwd.InternalNonmanifoldEdges();
        for (int i = 1; i <= aNM.Length(); i++)
        {
            aTShape.myShapes.Append(aNM[i]);
        }
        return aCompound;
    }

    // Puts back into the wire data, in order, the children of a compound made by edgesCompound.
    private static void setEdges(ShapeExtend_WireData sbwd, in TopoDS_Shape theCompound)
    {
        int aNbEdges = sbwd.NbEdges();
        NCollection_HSequence<TopoDS_Shape> aNM = sbwd.InternalNonmanifoldEdges();
        int i = 1;
        for (var anIt = new TopoDS_Iterator(theCompound, false, false); anIt.More(); anIt.Next(), i++)
        {
            TopoDS_Shape anEdge = anIt.Value();
            if (i <= aNbEdges)
            {
                sbwd.InternalSetEdge(TopoDS.Edge(anEdge), i);
            }
            else
            {
                aNM.SetValue(i - aNbEdges, anEdge);
            }
        }
    }
}
