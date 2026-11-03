// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_WireVertex.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_WireVertex.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// szv#4 S4163

using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Healing.Fixes;

/// <summary>
/// Fixes disconnected edges in a wire: fixes the vertices of the wire on the basis of the analysis made by
/// <see cref="WireVertexAnalysis"/>.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21). <see cref="Fix"/> and <see cref="FixSame"/> write into the edges of the wire data
/// in place (vertices removed and added, tolerances and parameters raised), as OCCT's. When the loaded edges hold
/// frozen shapes (any wire a caller holds), the first fix copies them (<see cref="CopyAtEntry"/>, decision
/// 583) and works on the copies, so the caller's wire never changes; <see cref="Wire"/> gives back the caller's own
/// edges and vertices wherever the fixes left them unchanged, and freezes. A fix after <see cref="Wire"/> copies
/// again.
/// </para>
/// <para>
/// OCCT copies the analyser given to <see cref="Init(WireVertexAnalysis)"/> (a value whose arrays and wire data
/// are handles, so the copy shares them); here the analyser is shared.
/// </para>
/// </remarks>
public sealed class WireVertexFixer
{
    private WireVertexAnalysis _analyzer;

    // OCCT.NET: the copy of the loaded edges, from the first fix to the public result (null when they hold no frozen
    // shape), and whether the loaded edges must be checked before the next fix.
    private CopyAtEntry? _entry;
    private bool                   _toEnter;

    // ied_modif_for_compil_Nov-19-1998

    /// <summary>Empty constructor.</summary>
    public WireVertexFixer()
    {
        _analyzer = new WireVertexAnalysis();
    }

    /// <summary>
    /// Loads the wire, initializes the internal analyzer (ShapeAnalysis_WireVertex) with the given precision and
    /// performs the analysis.
    /// </summary>
    public void Init(in Wire wire, double preci)
    {
        var sbwd = new ExtendWireData(wire);
        Init(sbwd, preci);
    }

    /// <summary>
    /// Loads the wire data, initializes the internal analyzer (ShapeAnalysis_WireVertex) with the given precision and
    /// performs the analysis.
    /// </summary>
    public void Init(ExtendWireData sbwd, double preci)
    {
        _analyzer.Load(sbwd);
        _analyzer.SetPrecision(preci);
        _analyzer.Analyze();
        _toEnter = true;
        _entry   = null;
    }

    /// <summary>Loads all the data on a wire, already analysed by ShapeAnalysis_WireVertex.</summary>
    public void Init(WireVertexAnalysis sawv)
    {
        _analyzer = sawv;
        _toEnter  = true;
        _entry    = null;
    }

    /// <summary>Returns the internal analyzer.</summary>
    public WireVertexAnalysis Analyzer() => _analyzer;

    /// <summary>
    /// Returns the data on the wire (fixed): the working edges (between the first fix and <see cref="Wire"/>, copies of a
    /// frozen wire's edges).
    /// </summary>
    public ExtendWireData? WireData() => _analyzer.WireData();

    /// <summary>
    /// Returns the resulting wire (fixed), frozen: where the loaded edges were frozen, those the fixes left unchanged
    /// are the caller's own.
    /// </summary>
    public Wire Wire()
    {
        EndWork();
        return _analyzer.WireData()!.Wire();
    }

    /// <summary>The resulting wire, unfrozen, for ported code fixing a wire it owns: its edges are the working ones.</summary>
    internal Wire InternalWire() => _analyzer.WireData()!.InternalWire();

    /// <summary>
    /// Fixes the "Same" or "Close" statuses (the same vertex may be set, without changing parameters). Returns the
    /// count of fixed vertices, 0 if none.
    /// </summary>
    public int FixSame()
    {
        //  FixSame : prend les status "SameCoord" et "Close" et les force a "Same"
        //  reprendre l edge et forcer le vertex. Evt changer sa tolerance. Et voila
        if (!_analyzer.IsDone)
        {
            return 0;
        }
        EnterWork(); // OCCT.NET: frozen

        int nbfix = 0;
        var b     = new BrepBuilder();

        ExtendWireData sbwd = _analyzer.WireData()!;
        int                  i, nb = sbwd.EdgeCount;

        for (i = 1; i <= nb; i++)
        {
            int j    = (i == nb ? 1 : i + 1);
            int stat = _analyzer.Status(i);
            if (stat != 1 && stat != 2)
            {
                continue;
            }
            // Ici on prend un vertex et on le generalise aux deux edges
            Edge e1 = sbwd.Edge(i);
            Edge e2 = sbwd.Edge(j);

            var           sae = new EdgeAnalysis();
            Vertex v1  = sae.LastVertex(e1);
            Vertex v2  = sae.FirstVertex(e2);
            if (v1 == v2) // OCCT: == compares orientations; a shared vertex is REVERSED in E1, FORWARD in E2
            {
                _analyzer.SetSameVertex(i);
                continue;
            } // deja fait ...
            if (stat == 2)
            {
                // OK mais en reprenant les tolerances
                sae.Curve3d(sbwd.Edge(i), out _, out _, out double cl);
                b.UpdateVertex(v1, cl, e1, _analyzer.Precision());
                sae.Curve3d(sbwd.Edge(j), out _, out double cf, out _);
                b.UpdateVertex(v1, cf, e2, _analyzer.Precision());
            }
            // Et remettre ce vtx en commun
            // OCCT: unlike Fix, the edges' old vertices aren't removed (the edges end up with both) and the edges aren't
            // made Free first, so on the edges of a wire (not free) B.Add throws TopoDS_FrozenShape
            v1.Orientation(e2.Orientation());
            b.Add(e2, v1);
            v1.Orientation(e1.Orientation());
            v1.Reverse();
            b.Add(e1, v1);
            _analyzer.SetSameVertex(i); // conclusion
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
        if (!_analyzer.IsDone)
        {
            return 0;
        }

        ExtendWireData sbwd = _analyzer.WireData()!;

        int i, nb = sbwd.EdgeCount;
        int nbfix = 0;
        for (i = 1; i <= nb; i++)
        {
            //    On note les valeurs
            // szv#4:S4163:12Mar99 optimized
            if (_analyzer.Status(i) > 0)
            {
                nbfix++;
            }
        }
        if (nbfix == 0)
        {
            return 0;
        }
        EnterWork(); // OCCT.NET: frozen

        var b = new BrepBuilder();

        var vi = new NCollection_HArray1<Shape>(1, nb);
        var vj = new NCollection_HArray1<Shape>(1, nb);
        var ef = new NCollection_HArray1<Shape>(1, nb);
        var ui = new NCollection_HArray1<double>(1, nb);
        var uj = new NCollection_HArray1<double>(1, nb);

        for (i = 1; i <= nb; i++)
        {
            //    On note les valeurs
            int j    = (i == nb ? 1 : i + 1);
            int stat = _analyzer.Status(i);

            var           sae = new EdgeAnalysis();
            Vertex v1  = sae.LastVertex(sbwd.Edge(i));
            Vertex v2  = sae.FirstVertex(sbwd.Edge(j));
            vi.SetValue(i, v1);
            vj.SetValue(j, v2);

            Edge e = sbwd.Edge(i);
            //    E.EmptyCopy();   trop d ennuis
            ef.SetValue(i, e);

            double upre = _analyzer.UPrevious(i);
            double ufol = _analyzer.UFollowing(j);

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

            ui.SetValue(i, upre);
            uj.SetValue(j, ufol);
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
            Edge e1 = ShapeCasts.Edge(ef.Value(i));
            e1.Orientation(Forward);
            Exploration.Vertices(e1, out Vertex va, out Vertex vb);
            e1.Free(true);
            b.Remove(e1, va);
            b.Remove(e1, vb);
        }

        double prec = _analyzer.Precision();
        for (i = 1; i <= nb; i++)
        {
            //    On y va pour de bon
            //    Changer les coords ?
            int j    = (i == nb ? 1 : i + 1);
            int stat = _analyzer.Status(i);
            //    if (stat <= 0) continue;

            Vertex v1   = ShapeCasts.Vertex(vi.Value(i));
            Edge   e1   = ShapeCasts.Edge(ef.Value(i));
            Edge   e2   = ShapeCasts.Edge(ef.Value(j));
            double        upre = ui.Value(i);
            double        ufol = uj.Value(j);

            if (stat > 2)
            {
                b.UpdateVertex(v1, new Point3D(_analyzer.Position(i)), prec);
            }

            //    ce qui suit : seulement si vertex a reprendre
            if (stat > 0)
            {
                b.UpdateVertex(v1, upre, e1, prec);
                b.UpdateVertex(v1, ufol, e2, prec);
                v1.Orientation(Forward);
                //    V1.Orientation (E2.Orientation());
            }

            //    Comme on a deshabille les edges, il faut tout remettre
            e2.Free(true); // sur place
            b.Add(e2, v1);
            v1.Orientation(Reversed);
            //    V1.Orientation (E1.Orientation());    V1.Reverse();
            e1.Free(true); // sur place
            b.Add(e1, v1);

            _analyzer.SetSameVertex(i); // conclusion
        }

        //  pour finir, MAJ du STW
        for (i = 1; i <= nb; i++)
        {
            sbwd.Set(ShapeCasts.Edge(ef.Value(i)), i);
        }

        return nbfix;
    }

    // OCCT.NET: before a fix writes, copies the loaded edges if they hold frozen shapes (after Init or a public result).
    private void EnterWork()
    {
        if (!_toEnter)
        {
            return;
        }
        _toEnter = false;
        _entry   = null;
        ExtendWireData? sbwd = _analyzer.WireData();
        if (sbwd is null)
        {
            return;
        }
        Compound edges = EdgesCompound(sbwd);
        if (!BrepTools.HoldsFrozen(edges))
        {
            return;
        }
        _entry = new CopyAtEntry(edges);
        SetEdges(sbwd, _entry.Shape());

        // The copy clears the Free flag of the edges it puts in the compound; give each copy its original's (a cache
        // bit, which KeepUnchanged ignores), so FixSame's B.Add accepts or refuses an edge as OCCT does on the original.
        var original = new Iterator(edges, false, false);
        for (var copy = new Iterator(_entry.Shape(), false, false); copy.More(); copy.Next(), original.Next())
        {
            copy.Value().TShape()!.Free(original.Value().TShape()!.Free());
        }
    }

    // OCCT.NET: before the public result, gives back the caller's edges and vertices the fixes left unchanged.
    private void EndWork()
    {
        _toEnter = true;
        CopyAtEntry? entry = _entry;
        _entry = null;
        ExtendWireData? sbwd = _analyzer.WireData();
        if (entry is null || sbwd is null)
        {
            return;
        }
        Shape final = entry.Finish(EdgesCompound(sbwd));
        SetEdges(sbwd, final);
    }

    // The edges, then the non-manifold edges, of the wire data, appended to a new compound as they are.
    private static Compound EdgesCompound(ExtendWireData sbwd)
    {
        var compound = new Compound();
        new BrepBuilder().MakeCompound(ref compound);
        TShape tShape = compound.TShape()!;
        for (int i = 1; i <= sbwd.EdgeCount; i++)
        {
            tShape._shapes.Append(sbwd.Edge(i));
        }
        NCollection_HSequence<Shape> nm = sbwd.InternalNonmanifoldEdges();
        for (int i = 1; i <= nm.Length(); i++)
        {
            tShape._shapes.Append(nm[i]);
        }
        return compound;
    }

    // Puts back into the wire data, in order, the children of a compound made by edgesCompound.
    private static void SetEdges(ExtendWireData sbwd, in Shape compound)
    {
        int nbEdges = sbwd.EdgeCount;
        NCollection_HSequence<Shape> nm = sbwd.InternalNonmanifoldEdges();
        int i = 1;
        for (var it = new Iterator(compound, false, false); it.More(); it.Next(), i++)
        {
            Shape edge = it.Value();
            if (i <= nbEdges)
            {
                sbwd.InternalSetEdge(ShapeCasts.Edge(edge), i);
            }
            else
            {
                nm.SetValue(i - nbEdges, edge);
            }
        }
    }
}
