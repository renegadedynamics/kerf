// Port of: src/ModelingAlgorithms/TKShHealing/ShapeExtend/ShapeExtend_Explorer.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeExtend/ShapeExtend_Explorer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Healing.Extend;

/// <summary>
/// Explores shapes and converts between the representations of a complex shape (list, sequence, compound): the type
/// of a shape in the context of a compound, the exploration of compounds, and the conversions.
/// </summary>
/// <remarks>
/// A stateless working object (OCCT uses it by value). The compounds, wires and shells it builds are new shapes
/// holding the caller's: <see cref="CompoundFromSeq"/> and <see cref="SortedCompound"/> freeze what they return
/// (docs/phase3-design.md, section 4.2); ported code that goes on building on them uses
/// <see cref="InternalCompoundFromSeq"/> and <see cref="InternalSortedCompound"/>.
/// </remarks>
internal sealed class ExtendExplorer
{
    /// <summary>Creates an explorer.</summary>
    public ExtendExplorer()
    {
    }

    /// <summary>A compound of the shapes of the sequence. Frozen.</summary>
    public Shape CompoundFromSeq(NCollection_HSequence<Shape> seqval) =>
        Published(InternalCompoundFromSeq(seqval));

    /// <summary>A compound of the shapes of the sequence, as <see cref="CompoundFromSeq"/>, unfrozen (ported code).</summary>
    internal Shape InternalCompoundFromSeq(NCollection_HSequence<Shape> seqval)
    {
        BrepBuilder b = new();
        Compound c = new();
        b.MakeCompound(ref c);
        int i, n = seqval.Length();
        for (i = 1; i <= n; i++)
        {
            b.Add(c, seqval.Value(i));
        }
        return c;
    }

    private static void FillList(NCollection_HSequence<Shape> list, in Shape comp, bool expcomp)
    {
        for (var it = new Iterator(comp); it.More(); it.Next())
        {
            Shape sub = it.Value();
            if (sub.Type != global::Kerf.Topology.ShapeType.Compound)
            {
                list.Append(sub);
            }
            else if (!expcomp)
            {
                list.Append(sub);
            }
            else
            {
                FillList(list, sub, expcomp);
            }
        }
    }

    /// <summary>
    /// The sub-shapes of a compound as a new sequence: <paramref name="comp"/> alone if it isn't a compound, nothing
    /// if it is null; with <paramref name="expcomp"/>, sub-compounds are replaced by their sub-shapes, recursively.
    /// </summary>
    public NCollection_HSequence<Shape> SeqFromCompound(in Shape comp, bool expcomp)
    {
        var list = new NCollection_HSequence<Shape>();
        if (comp.IsNull)
        {
            return list;
        }
        if (comp.Type != global::Kerf.Topology.ShapeType.Compound)
        {
            list.Append(comp);
            return list;
        }
        FillList(list, comp, expcomp);
        return list;
    }

    /// <summary>
    /// Appends the shapes of the sequence to the list, after clearing it if <paramref name="clear"/>.
    /// </summary>
    public void ListFromSeq(NCollection_HSequence<Shape>? seqval, NCollection_List<Shape> lisval, bool clear = true)
    {
        if (clear)
        {
            lisval.Clear();
        }
        if (seqval is null)
        {
            return;
        }
        int i, nb = seqval.Length();
        for (i = 1; i <= nb; i++)
        {
            lisval.Append(seqval.Value(i));
        }
    }

    /// <summary>The shapes of the list as a new sequence.</summary>
    public NCollection_HSequence<Shape> SeqFromList(NCollection_List<Shape> lisval)
    {
        var seqval = new NCollection_HSequence<Shape>();
        for (var it = new NCollection_List<Shape>.Iterator(lisval); it.More(); it.Next())
        {
            seqval.Append(it.Value());
        }
        return seqval;
    }

    /// <summary>
    /// The type of a shape: its true type, unless <paramref name="compound"/> is set and it is a compound; then the
    /// type of its items if they all have the same (an edge and a wire count as wires, a face and a shell as
    /// shells), COMPOUND if they differ, SHAPE if it is empty. SHAPE for a null shape.
    /// </summary>
    public ShapeType ShapeType(in Shape shape, bool compound)
    {
        if (shape.IsNull)
        {
            return global::Kerf.Topology.ShapeType.Shape;
        }
        ShapeType res = shape.Type;
        if (!compound || res != global::Kerf.Topology.ShapeType.Compound)
        {
            return res;
        }
        res = global::Kerf.Topology.ShapeType.Shape;
        for (var iter = new Iterator(shape); iter.More(); iter.Next())
        {
            Shape sh = iter.Value();
            if (sh.IsNull)
            {
                continue;
            }
            ShapeType typ = sh.Type;
            if (typ == global::Kerf.Topology.ShapeType.Compound)
            {
                typ = ShapeType(sh, compound);
            }
            if (res == global::Kerf.Topology.ShapeType.Shape)
            {
                res = typ;
                //   Egalite : OK;  Pseudo-Egalite : EDGE/WIRE ou FACE/SHELL
            }
            else if (res == global::Kerf.Topology.ShapeType.Edge && typ == global::Kerf.Topology.ShapeType.Wire)
            {
                res = typ;
            }
            else if (res == global::Kerf.Topology.ShapeType.Wire && typ == global::Kerf.Topology.ShapeType.Edge)
            {
                continue;
            }
            else if (res == global::Kerf.Topology.ShapeType.Face && typ == global::Kerf.Topology.ShapeType.Shell)
            {
                res = typ;
            }
            else if (res == global::Kerf.Topology.ShapeType.Shell && typ == global::Kerf.Topology.ShapeType.Face)
            {
                continue;
            }
            else if (res != typ)
            {
                return global::Kerf.Topology.ShapeType.Compound;
            }
        }
        return res;
    }

    /// <summary>
    /// A compound of the parts of <paramref name="shape"/> of the given type, explored level by level (only compounds
    /// unless <paramref name="explore"/>): free edges become wires when <paramref name="type"/> is WIRE, free faces
    /// shells when it is SHELL. With <paramref name="compound"/>, the items are gathered in compounds following the
    /// starting compounds, solids and shells. A single item is returned itself, nothing as a null shape. Frozen.
    /// </summary>
    public Shape SortedCompound(in Shape shape, ShapeType type, bool explore, bool compound) =>
        Published(InternalSortedCompound(shape, type, explore, compound));

    /// <summary>The sorted compound, as <see cref="SortedCompound"/>, unfrozen (ported code).</summary>
    internal Shape InternalSortedCompound(in Shape shape, ShapeType type, bool explore, bool compound)
    {
        if (shape.IsNull)
        {
            return shape;
        }
        ShapeType typ = shape.Type;
        Shape sh = new(), sh0;
        int nb = 0;

        //  Compound : on le prend, soit tel quel, soit son contenu
        if (typ == global::Kerf.Topology.ShapeType.Compound || typ == global::Kerf.Topology.ShapeType.CompSolid)
        {
            Compound c = new();
            BrepBuilder b = new();
            b.MakeCompound(ref c);
            for (var it = new Iterator(shape); it.More(); it.Next())
            {
                sh0 = InternalSortedCompound(it.Value(), type, explore, compound);
                if (sh0.IsNull)
                {
                    continue;
                }
                sh = sh0;
                typ = sh.Type;
                if (typ == global::Kerf.Topology.ShapeType.Compound && !compound)
                {
                    for (var it2 = new Iterator(sh); it2.More(); it2.Next())
                    {
                        nb++;
                        sh = it2.Value();
                        b.Add(c, sh);
                    }
                }
                else
                {
                    nb++;
                    b.Add(c, sh);
                }
            }
            if (nb == 0)
            {
                c.Nullify();
            }
            else if (nb == 1)
            {
                return sh;
            }
            return c;
        }

        //   Egalite : OK;  Pseudo-Egalite : EDGE/WIRE ou FACE/SHELL
        if (typ == type)
        {
            return shape;
        }
        if (typ == global::Kerf.Topology.ShapeType.Edge && type == global::Kerf.Topology.ShapeType.Wire)
        {
            BrepBuilder b = new();
            Wire w = new();
            b.MakeWire(ref w);
            b.Add(w, shape);
            return w;
        }
        if (typ == global::Kerf.Topology.ShapeType.Face && type == global::Kerf.Topology.ShapeType.Shell)
        {
            BrepBuilder b = new();
            Shell s = new();
            b.MakeShell(ref s);
            b.Add(s, shape);
            return s;
        }

        //   Le reste : selon exploration
        if (!explore)
        {
            Shape nulsh = new();
            return nulsh;
        }

        //  Ici, on doit explorer
        //  SOLID + mode COMPOUND : reconduire les SHELLs
        if (typ == global::Kerf.Topology.ShapeType.Solid && compound)
        {
            Compound c = new();
            BrepBuilder b = new();
            b.MakeCompound(ref c);
            for (var it = new Iterator(shape); it.More(); it.Next())
            {
                sh0 = InternalSortedCompound(it.Value(), type, explore, compound);
                if (sh0.IsNull)
                {
                    continue;
                }
                sh = sh0;
                nb++;
                b.Add(c, sh);
            }
            if (nb == 0)
            {
                c.Nullify();
            }
            else if (nb == 1)
            {
                return sh;
            }
            return c;
        }

        //  Exploration classique
        Compound cc = new();
        BrepBuilder bb = new();
        bb.MakeCompound(ref cc);
        for (var exp = new Explorer(shape, type); exp.More(); exp.Next())
        {
            nb++;
            sh = exp.Current();
            bb.Add(cc, sh);
        }
        if (nb == 0)
        {
            cc.Nullify();
        }
        else if (nb == 1)
        {
            return sh;
        }
        return cc;
    }

    /// <summary>
    /// Appends the shapes of <paramref name="list"/> to the sequence of their type, creating each sequence that is
    /// null.
    /// </summary>
    public void DispatchList(NCollection_HSequence<Shape>? list,
                             ref NCollection_HSequence<Shape>? vertices,
                             ref NCollection_HSequence<Shape>? edges,
                             ref NCollection_HSequence<Shape>? wires,
                             ref NCollection_HSequence<Shape>? faces,
                             ref NCollection_HSequence<Shape>? shells,
                             ref NCollection_HSequence<Shape>? solids,
                             ref NCollection_HSequence<Shape>? compsols,
                             ref NCollection_HSequence<Shape>? compounds)
    {
        if (list is null)
        {
            return;
        }
        vertices ??= new NCollection_HSequence<Shape>();
        edges ??= new NCollection_HSequence<Shape>();
        wires ??= new NCollection_HSequence<Shape>();
        faces ??= new NCollection_HSequence<Shape>();
        shells ??= new NCollection_HSequence<Shape>();
        solids ??= new NCollection_HSequence<Shape>();
        compsols ??= new NCollection_HSequence<Shape>();
        compounds ??= new NCollection_HSequence<Shape>();

        int i, nb = list.Length();
        for (i = 1; i <= nb; i++)
        {
            Shape sh = list.Value(i);
            if (sh.IsNull)
            {
                continue;
            }
            switch (sh.Type)
            {
                case global::Kerf.Topology.ShapeType.Vertex:
                    vertices.Append(sh);
                    break;
                case global::Kerf.Topology.ShapeType.Edge:
                    edges.Append(sh);
                    break;
                case global::Kerf.Topology.ShapeType.Wire:
                    wires.Append(sh);
                    break;
                case global::Kerf.Topology.ShapeType.Face:
                    faces.Append(sh);
                    break;
                case global::Kerf.Topology.ShapeType.Shell:
                    shells.Append(sh);
                    break;
                case global::Kerf.Topology.ShapeType.Solid:
                    solids.Append(sh);
                    break;
                case global::Kerf.Topology.ShapeType.CompSolid:
                    compsols.Append(sh);
                    break;
                case global::Kerf.Topology.ShapeType.Compound:
                    compounds.Append(sh);
                    break;
                default:
                    break;
            }
        }
    }

    // Freezes the graph of a shape handed to a caller (docs/phase3-design.md, section 4.2).
    private static Shape Published(in Shape shape)
    {
        shape.TShape()?.FreezeGraph();
        return shape;
    }
}
