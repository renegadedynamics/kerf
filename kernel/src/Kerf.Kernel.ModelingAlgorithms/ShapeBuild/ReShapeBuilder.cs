// Port of: src/ModelingAlgorithms/TKShHealing/ShapeBuild/ShapeBuild_ReShape.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeBuild/ShapeBuild_ReShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Healing.Extend.ExtendStatus;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Healing.Build;

/// <summary>
/// Shape Healing's reshaper: records requests to replace or remove shapes (each shape's last request is kept) and
/// applies them to any shape holding some of them, rebuilding their parents. The healing algorithms (ShapeFix,
/// ShapeUpgrade, sewing) share one as their context, and callers read the replacements back from it.
/// </summary>
/// <remarks>
/// <para>
/// A working object, as <see cref="ReShape"/> (decision 358): the public <see cref="Apply(in Shape,
/// ShapeType, int)"/> and the inherited <c>Apply</c>, <c>Value</c>, <c>Status</c> and <c>History</c> freeze the
/// shapes they return; ported code uses the internal forms (<see cref="InternalApply(in Shape, ShapeType,
/// int)"/> and the inherited <c>InternalApply</c>, <c>InternalValue</c>, ...).
/// </para>
/// <para>
/// Apply only writes into the shapes it builds (EmptyCopied parents, ranges copied onto a rebuilt edge, the Closed
/// flag of rebuilt wires and shells): it works on frozen shapes. A healing algorithm that ran on a copy of a frozen
/// shape (decision 583) records its replacements of the copy's sub-shapes here; <see cref="CopyAtEntry"/>
/// then re-keys them to the caller's sub-shapes, so <c>Value</c>, <c>Apply</c> and <c>History</c> answer for the
/// shapes the caller holds.
/// </para>
/// </remarks>
public class ReShapeBuilder : ReShape
{
    /// <summary>An empty reshaper.</summary>
    public ReShapeBuilder()
    {
    }

    /// <summary>
    /// Applies the requests to a shape, down to the type <paramref name="until"/> (not implemented below faces).
    /// <paramref name="buildmode"/> tells what a solid or shell becomes when one of its sub-shapes changed: 0, a
    /// compound if anything was replaced or removed; 1, a compound if anything was removed (replacements ignored); 2,
    /// both ignored: the result has the type of the starting shape. Frozen.
    /// </summary>
    public Shape Apply(in Shape shape, ShapeType until, int buildmode) =>
        Published(InternalApply(shape, until, buildmode));

    /// <summary>The application with a build mode, as <see cref="Apply(in Shape, ShapeType, int)"/>, unfrozen (ported code).</summary>
    internal virtual Shape InternalApply(in Shape shape, ShapeType until, int buildmode)
    {
        if (shape.IsNull)
        {
            return shape;
        }
        Shape newsh;
        if (InternalStatus(shape, out newsh, false) != 0)
        {
            return newsh;
        }

        ShapeType st = shape.ShapeType();
        if (st == until)
        {
            return newsh; // critere d arret
        }

        int modif = 0;
        if (st == ShapeType.Compound || st == ShapeType.CompSolid)
        {
            BrepBuilder b = new();
            Compound c = new();
            b.MakeCompound(ref c);
            for (var it = new Iterator(shape); it.More(); it.Next())
            {
                Shape sh = it.Value();
                int stat = InternalStatus(sh, out newsh, false);
                if (stat != 0)
                {
                    modif = 1;
                }
                if (stat >= 0)
                {
                    b.Add(c, newsh);
                }
            }
            if (modif == 0)
            {
                return shape;
            }
            return c;
        }

        if (st == ShapeType.Solid)
        {
            BrepBuilder b = new();
            Compound c = new();
            b.MakeCompound(ref c);
            Solid s = new();
            b.MakeSolid(ref s);
            for (var it = new Iterator(shape); it.More(); it.Next())
            {
                Shape sh = it.Value();
                newsh = InternalApply(sh, until, buildmode);
                if (newsh.IsNull)
                {
                    modif = -1;
                }
                else if (newsh.ShapeType() != ShapeType.Shell)
                {
                    int nbsub = 0;
                    for (var exh = new Explorer(newsh, ShapeType.Shell); exh.More(); exh.Next())
                    {
                        Shape onesh = exh.Current();
                        b.Add(s, onesh);
                        nbsub++;
                    }
                    if (nbsub == 0)
                    {
                        modif = -1;
                    }
                    b.Add(c, newsh); // c est tout
                }
                else
                {
                    if (modif == 0 && !sh.IsEqual(newsh))
                    {
                        modif = 1;
                    }
                    b.Add(c, newsh);
                    b.Add(s, newsh);
                }
            }

            if ((modif < 0 && buildmode < 2) || (modif == 0 && buildmode < 1))
            {
                return c;
            }
            else
            {
                return s;
            }
        }

        if (st == ShapeType.Shell)
        {
            BrepBuilder b = new();
            Compound c = new();
            b.MakeCompound(ref c);
            Shell s = new();
            b.MakeShell(ref s);
            for (var it = new Iterator(shape); it.More(); it.Next())
            {
                Shape sh = it.Value();
                newsh = InternalApply(sh, until, buildmode);
                if (newsh.IsNull)
                {
                    modif = -1;
                }
                else if (newsh.ShapeType() != ShapeType.Face)
                {
                    int nbsub = 0;
                    for (var exf = new Explorer(newsh, ShapeType.Face); exf.More(); exf.Next())
                    {
                        Shape onesh = exf.Current();
                        b.Add(s, onesh);
                        nbsub++;
                    }
                    if (nbsub == 0)
                    {
                        modif = -1;
                    }
                    b.Add(c, newsh); // c est tout
                }
                else
                {
                    if (modif == 0 && !sh.IsEqual(newsh))
                    {
                        modif = 1;
                    }
                    b.Add(c, newsh);
                    b.Add(s, newsh);
                }
            }
            if ((modif < 0 && buildmode < 2) || (modif == 0 && buildmode < 1))
            {
                return c;
            }
            else
            {
                s.Closed(Tool.IsClosed(s));
                return s;
            }
        }
        // OCCT prints "BRepTools_ReShape::Apply NOT YET IMPLEMENTED" to the console here; the port doesn't (decision 493).
        return shape;
    }

    /// <summary>
    /// Applies the requests, as <see cref="ReShape.InternalApply"/>: a sub-shape can be replaced by a shape
    /// of its type or by a container of shapes of its type; other types are ignored (status FAIL1). Unlike
    /// BRepTools_ReShape's, sub-shapes of the type <paramref name="until"/> and below aren't explored.
    /// </summary>
    internal override Shape InternalApply(in Shape shape, ShapeType until = ShapeType.Shape)
    {
        var inFlight = new HashSet<TShape>(ReferenceEqualityComparer.Instance);
        return ApplyImplCore(shape, until, inFlight);
    }

    // Recursive worker for Apply with a DFS in-flight set keyed by TShape (by reference). Prevents unbounded descent
    // when a replacement is a compound that transitively contains the original shape as a sub-shape (cyclic
    // containment, distinct from cycles in the replacement map itself).
    private Shape ApplyImplCore(in Shape shape, ShapeType until, HashSet<TShape> inFlight)
    {
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (shape.IsNull)
        {
            return shape;
        }

        // Apply direct replacement
        Shape newShape = InternalValue(shape);

        // If shape removed, return NULL
        if (newShape.IsNull)
        {
            _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
            return newShape;
        }

        // DFS cycle guard: if theShape is already being processed further up the call
        // stack, its replacement must be a compound that transitively contains it.
        // Return the direct replacement without descending to break the cycle.
        if (inFlight.Contains(shape.TShape()!))
        {
            return newShape;
        }

        // If shape was replaced, apply modifications to the result recursively.
        bool consLoc = ModeConsiderLocation();
        if ((consLoc && !newShape.IsPartner(shape)) || (!consLoc && !newShape.IsSame(shape)))
        {
            inFlight.Add(shape.TShape()!);
            Shape res = ApplyImplCore(newShape, until, inFlight);
            inFlight.Remove(shape.TShape()!);
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
            return res;
        }

        ShapeType st = shape.ShapeType();
        if (st >= until)
        {
            return newShape; // stop criterion
        }
        if (st == ShapeType.Vertex || st == ShapeType.Shape)
        {
            return shape;
        }

        BrepBuilder builder = new();

        Shape result = shape.EmptyCopied();
        Orientation orient = shape.Orientation();
        result.Orientation(Forward); // protect against INTERNAL or EXTERNAL shapes
        bool modif = false;
        int locStatus = _status;

        // Apply recorded modifications to subshapes
        inFlight.Add(shape.TShape()!);
        for (var it = new Iterator(shape, false); it.More(); it.Next())
        {
            Shape sh = it.Value();
            newShape = ApplyImplCore(sh, until, inFlight);
            if (newShape != sh)
            {
                if (global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_status, ExtendStatus.DONE4))
                {
                    locStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE4);
                }
                modif = true;
            }
            if (newShape.IsNull)
            {
                locStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE4);
                continue;
            }
            locStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE3);
            if (st == ShapeType.Compound || newShape.ShapeType() == sh.ShapeType())
            {
                // Fix for SAMTECH bug OCC322 about absence internal vertices after sewing
                builder.Add(result, newShape);
                continue;
            }
            int nbItems = 0;
            for (var subIt = new Iterator(newShape); subIt.More(); subIt.Next(), nbItems++)
            {
                Shape subSh = subIt.Value();
                if (subSh.ShapeType() == sh.ShapeType())
                {
                    builder.Add(result, subSh);
                }
                else
                {
                    locStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
                }
            }
            if (nbItems == 0)
            {
                locStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
            }
        }
        inFlight.Remove(shape.TShape()!);
        if (!modif)
        {
            return shape;
        }

        // Restore range on edge broken by EmptyCopied()
        if (st == ShapeType.Edge)
        {
            BuildEdgeBuilder sbe = new();
            sbe.CopyRanges(ShapeCasts.Edge(result), ShapeCasts.Edge(shape));
        }
        else if (st == ShapeType.Wire || st == ShapeType.Shell)
        {
            result.Closed(Tool.IsClosed(result));
        }
        result.Orientation(orient);
        _status = locStatus;

        ReplaceCore(shape, result, result.IsNull ? TReplacementKind.TReplacementKind_Remove : TReplacementKind.TReplacementKind_Modify);

        return result;
    }

    /// <summary>
    /// The status of the last Apply: OK (nothing replaced or removed), DONE1 (the starting shape replaced), DONE2 (it
    /// was removed), DONE3 (sub-shapes replaced), DONE4 (sub-shapes removed), FAIL1 (replacements of a bad type not
    /// done).
    /// </summary>
    public virtual bool Status(ExtendStatus status) => global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_status, status);

    // Freezes the graph of a shape handed to a caller (docs/phase3-design.md, section 4.2).
    private static Shape Published(in Shape shape)
    {
        shape.TShape()?.FreezeGraph();
        return shape;
    }
}
