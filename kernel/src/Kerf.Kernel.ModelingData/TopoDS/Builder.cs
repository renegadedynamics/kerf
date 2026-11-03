// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_Builder.hxx
// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_Builder.lxx
// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_Builder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1990-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Topology;

/// <summary>
/// Creates topological data structures: the Make methods create shapes, Add puts a shape into another and Remove takes
/// it out. The root of the builders; BRep_Builder adds the shapes with geometry.
/// </summary>
/// <remarks>
/// <para>
/// A builder changes shapes in place. Callers build compounds with <see cref="MakeCompound(Shape[])"/>, which
/// returns a frozen compound, and other shapes through the public algorithms. The Make methods, Add and Remove are public
/// for the modelling layer (Kerf unit 10B; they become engine, internal, in unit 10L): a shape they build stays open until
/// the caller freezes it with <see cref="TShape.FreezeGraph"/>, before anything else reads it.
/// </para>
/// <para>
/// Add checks OCCT's rules: any shape goes into a COMPOUND; only a SOLID into a COMPSOLID; only a SHELL, an EDGE or a
/// VERTEX into a SOLID; only a FACE into a SHELL; only a WIRE or a VERTEX into a FACE; only an EDGE into a WIRE; only a
/// VERTEX into an EDGE; nothing into a VERTEX.
/// </para>
/// <para>
/// The Make methods take the shape by <c>ref</c> (C++ <c>T&amp;</c>; some of BRep_Builder's read it before replacing
/// it), so ported code writes <c>TopoDS_Wire W = new(); B.MakeWire(ref W);</c>.
/// </para>
/// </remarks>
public class Builder
{
    public Builder()
    {
    }

    /// <summary>
    /// A frozen compound of the given shapes (OCCT.NET addition, the public way to group shapes). The shapes' Free
    /// flags don't change (docs/phase3-design.md, section 4.4).
    /// </summary>
    /// <exception cref="NullObjectException">A shape is null.</exception>
    public static Compound MakeCompound(params Shape[] shapes)
        => MakeCompound((IEnumerable<Shape>)shapes);

    /// <summary>
    /// A frozen compound of the given shapes (OCCT.NET addition, the public way to group shapes). The shapes' Free
    /// flags don't change (docs/phase3-design.md, section 4.4).
    /// </summary>
    /// <exception cref="NullObjectException">A shape is null.</exception>
    public static Compound MakeCompound(IEnumerable<Shape> shapes)
    {
        var builder = new Builder();
        Compound compound = new Compound();
        builder.MakeCompound(ref compound);
        foreach (Shape shape in shapes)
        {
            Standard_NullObject_Raise_if(shape.IsNull, "TopoDS_Builder::MakeCompound: a shape is null");
            builder.Add(compound, shape);
        }

        compound.TShape()!.FreezeGraph();
        return compound;
    }

    /// <summary>
    /// A frozen wire of the given edges, as OCCT's <c>MakeWire</c> and <c>Add</c> make it (OCCT.NET addition, decision
    /// 10404): no flag is set and no geometry is checked.
    /// </summary>
    /// <exception cref="NullObjectException">A shape is null.</exception>
    /// <exception cref="IncompatibleShapesException">A shape isn't an edge.</exception>
    public static Wire MakeWire(IEnumerable<Shape> edges)
    {
        Wire wire = new Wire();
        new Builder().MakeWire(ref wire);
        Fill(wire, edges, "MakeWire");
        return wire;
    }

    /// <summary>
    /// A frozen shell of the given faces, as OCCT's <c>MakeShell</c> and <c>Add</c> make it (OCCT.NET addition,
    /// decision 1104): no flag is set (it isn't marked closed) and no geometry is checked.
    /// </summary>
    /// <exception cref="NullObjectException">A shape is null.</exception>
    /// <exception cref="IncompatibleShapesException">A shape isn't a face.</exception>
    public static Shell MakeShell(IEnumerable<Shape> faces)
    {
        Shell shell = new Shell();
        new Builder().MakeShell(ref shell);
        Fill(shell, faces, "MakeShell");
        return shell;
    }

    /// <summary>
    /// A frozen solid of the given shells (or edges and vertices), as OCCT's <c>MakeSolid</c> and <c>Add</c> make it
    /// (OCCT.NET addition, decision 1104): the shells are taken as they are oriented, nothing is checked.
    /// </summary>
    /// <exception cref="NullObjectException">A shape is null.</exception>
    /// <exception cref="IncompatibleShapesException">A shape isn't a shell, an edge or a vertex.</exception>
    public static Solid MakeSolid(IEnumerable<Shape> shells)
    {
        Solid solid = new Solid();
        new Builder().MakeSolid(ref solid);
        Fill(solid, shells, "MakeSolid");
        return solid;
    }

    /// <summary>
    /// A frozen composite solid of the given solids, as OCCT's <c>MakeCompSolid</c> and <c>Add</c> make it (OCCT.NET
    /// addition, decision 1104).
    /// </summary>
    /// <exception cref="NullObjectException">A shape is null.</exception>
    /// <exception cref="IncompatibleShapesException">A shape isn't a solid.</exception>
    public static CompSolid MakeCompSolid(IEnumerable<Shape> solids)
    {
        CompSolid compSolid = new CompSolid();
        new Builder().MakeCompSolid(ref compSolid);
        Fill(compSolid, solids, "MakeCompSolid");
        return compSolid;
    }

    // Adds the shapes to the new shape and freezes it.
    private static void Fill(in Shape shape, IEnumerable<Shape> shapes, string what)
    {
        var builder = new Builder();
        foreach (Shape component in shapes)
        {
            Standard_NullObject_Raise_if(component.IsNull, $"TopoDS_Builder::{what}: a shape is null");
            builder.Add(shape, component);
        }

        shape.TShape()!.FreezeGraph();
    }

    /// <summary>Makes an empty wire.</summary>
    public void MakeWire(ref Wire w)
    {
        TWire tw = new TWire();
        MakeShape(ref ShapeCasts.AsShapeRef(ref w), tw);
    }

    /// <summary>Makes an empty shell.</summary>
    public void MakeShell(ref Shell s)
    {
        TShell ts = new TShell();
        MakeShape(ref ShapeCasts.AsShapeRef(ref s), ts);
    }

    /// <summary>Makes a solid covering the whole 3D space.</summary>
    public void MakeSolid(ref Solid s)
    {
        TSolid ts = new TSolid();
        MakeShape(ref ShapeCasts.AsShapeRef(ref s), ts);
    }

    /// <summary>Makes an empty composite solid.</summary>
    public void MakeCompSolid(ref CompSolid c)
    {
        TCompSolid tc = new TCompSolid();
        MakeShape(ref ShapeCasts.AsShapeRef(ref c), tc);
    }

    /// <summary>Makes an empty compound.</summary>
    public void MakeCompound(ref Compound c)
    {
        TCompound tc = new TCompound();
        MakeShape(ref ShapeCasts.AsShapeRef(ref c), tc);
    }

    /// <summary>
    /// Adds the shape <paramref name="aComponent"/> to <paramref name="aShape"/>. Only the TShape of aShape changes,
    /// so it is taken by <c>in</c>.
    /// </summary>
    /// <exception cref="FrozenShapeException">aShape isn't free.</exception>
    /// <exception cref="IncompatibleShapesException">aShape can't contain aComponent.</exception>
    /// <exception cref="ImmutableObjectException">aShape is frozen.</exception>
    public void Add(in Shape aShape, in Shape aComponent)
    {
        // From now the Component cannot be edited
        // OCCT.NET: frozen: a frozen component keeps the Free flag it was frozen with, so its BRep text never
        // changes (docs/phase3-design.md, section 4.4; OCCT clears it).
        TShape componentTShape = aComponent.TShape()
                                         ?? throw new NullObjectException("TopoDS_Builder::Add: the component is null");
        if (!componentTShape.IsFrozen)
        {
            componentTShape.Free(false);
        }

        // Note that freezing aComponent before testing if aShape is free
        // prevents from self-insertion
        // but aShape will be frozen when the Exception is raised
        if (aShape.Free())
        {
            uint iC = (uint)aComponent.ShapeType();
            uint iS = (uint)aShape.ShapeType();
            if ((ATb[iC] & (1u << (int)iS)) != 0)
            {
                Shape child = aComponent;
                // compute the relative Orientation
                if (aShape.Orientation() == Reversed)
                {
                    child.Reverse();
                }

                // and the Relative Location
                Placement loc = aShape.Location();
                if (!loc.IsIdentity)
                {
                    child.Move(loc.Inverted(), false);
                }

                // Add to the subshapes list
                // Kerf: a frozen shape takes the child in the working overlay's copy of it, if one is open
                TShape tShape = WorkingOverlay.ForWrite(aShape.TShape()!);
                tShape.CheckMutable(); // OCCT.NET: frozen
                tShape._shapes.Append(child);
                tShape.Modified(true);
            }
            else
            {
                throw new IncompatibleShapesException("TopoDS_Builder::Add");
            }
        }
        else
        {
            throw new FrozenShapeException("TopoDS_Builder::Add");
        }
    }

    // The types each shape type can be added to, as bit masks of TopAbs_ShapeEnum
    private static readonly uint[] ATb =
    [
        // COMPOUND to:
        1u << (int)ShapeType.Compound,
        // COMPSOLID to:
        1u << (int)ShapeType.Compound,
        // SOLID to:
        (1u << (int)ShapeType.Compound) | (1u << (int)ShapeType.CompSolid),
        // SHELL to:
        (1u << (int)ShapeType.Compound) | (1u << (int)ShapeType.Solid),
        // FACE to:
        (1u << (int)ShapeType.Compound) | (1u << (int)ShapeType.Shell),
        // WIRE to:
        (1u << (int)ShapeType.Compound) | (1u << (int)ShapeType.Face),
        // EDGE to:
        (1u << (int)ShapeType.Compound) | (1u << (int)ShapeType.Solid) | (1u << (int)ShapeType.Wire),
        // VERTEX to:
        (1u << (int)ShapeType.Compound) | (1u << (int)ShapeType.Solid) | (1u << (int)ShapeType.Face) | (1u << (int)ShapeType.Edge),
        // SHAPE to:
        0,
    ];

    /// <summary>
    /// Removes the shape <paramref name="aComponent"/> from <paramref name="aShape"/>. Only the TShape of aShape
    /// changes, so it is taken by <c>in</c>.
    /// </summary>
    /// <exception cref="FrozenShapeException">aShape isn't free.</exception>
    /// <exception cref="ImmutableObjectException">aShape is frozen.</exception>
    public void Remove(in Shape aShape, in Shape aComponent)
    {
        // check if aShape  is not Frozen
        Raise.TopoDS_FrozenShape_Raise_if(!aShape.Free(), "TopoDS_Builder::Remove");

        // compute the relative Orientation and Location of aComponent
        Shape s = aComponent;
        if (aShape.Orientation() == Reversed)
        {
            s.Reverse();
        }

        s.Location(s.Location().Predivided(aShape.Location()), false);

        // Kerf: a frozen shape loses the child in the working overlay's copy of it, if one is open
        TShape tShape = aShape.TShape()!;
        if (tShape.IsFrozen && WorkingOverlay.TryWritable(tShape, out TShape? working))
        {
            tShape = working;
        }
        NCollection_List<Shape> list = tShape._shapes;
        var iter = new NCollection_List<Shape>.Iterator(list);
        while (iter.More())
        {
            if (iter.Value() == s)
            {
                tShape.CheckMutable(); // OCCT.NET: frozen
                list.Remove(iter);
                tShape.Modified(true);
                return;
            }

            iter.Next();
        }
    }

    /// <summary>The basic method to make a shape, used by all the Make methods.</summary>
    private protected static void MakeShape(ref Shape s, TShape t)
    {
        s.TShape(t);
        s.Location(new Placement());
        s.Orientation(Forward);
    }
}
