// Port of: src/ModelingAlgorithms/TKShHealing/ShapeBuild/ShapeBuild_ReShape.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeBuild/ShapeBuild_ReShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using static Occt.ShapeExtend_Status;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

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
public class ShapeBuild_ReShape : ReShape
{
    /// <summary>An empty reshaper.</summary>
    public ShapeBuild_ReShape()
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
            BrepBuilder B = new();
            Compound C = new();
            B.MakeCompound(ref C);
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
                    B.Add(C, newsh);
                }
            }
            if (modif == 0)
            {
                return shape;
            }
            return C;
        }

        if (st == ShapeType.Solid)
        {
            BrepBuilder B = new();
            Compound C = new();
            B.MakeCompound(ref C);
            Solid S = new();
            B.MakeSolid(ref S);
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
                        B.Add(S, onesh);
                        nbsub++;
                    }
                    if (nbsub == 0)
                    {
                        modif = -1;
                    }
                    B.Add(C, newsh); // c est tout
                }
                else
                {
                    if (modif == 0 && !sh.IsEqual(newsh))
                    {
                        modif = 1;
                    }
                    B.Add(C, newsh);
                    B.Add(S, newsh);
                }
            }

            if ((modif < 0 && buildmode < 2) || (modif == 0 && buildmode < 1))
            {
                return C;
            }
            else
            {
                return S;
            }
        }

        if (st == ShapeType.Shell)
        {
            BrepBuilder B = new();
            Compound C = new();
            B.MakeCompound(ref C);
            Shell S = new();
            B.MakeShell(ref S);
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
                        B.Add(S, onesh);
                        nbsub++;
                    }
                    if (nbsub == 0)
                    {
                        modif = -1;
                    }
                    B.Add(C, newsh); // c est tout
                }
                else
                {
                    if (modif == 0 && !sh.IsEqual(newsh))
                    {
                        modif = 1;
                    }
                    B.Add(C, newsh);
                    B.Add(S, newsh);
                }
            }
            if ((modif < 0 && buildmode < 2) || (modif == 0 && buildmode < 1))
            {
                return C;
            }
            else
            {
                S.Closed(Tool.IsClosed(S));
                return S;
            }
        }
        // OCCT prints "BRepTools_ReShape::Apply NOT YET IMPLEMENTED" to the console here; the port doesn't (decision 493).
        return shape;
    }

    /// <summary>
    /// Applies the requests, as <see cref="ReShape.InternalApply"/>: a sub-shape can be replaced by a shape
    /// of its type or by a container of shapes of its type; other types are ignored (status FAIL1). Unlike
    /// BRepTools_ReShape's, sub-shapes of the type <paramref name="theUntil"/> and below aren't explored.
    /// </summary>
    internal override Shape InternalApply(in Shape theShape, ShapeType theUntil = ShapeType.Shape)
    {
        var anInFlight = new NCollection_Map<TShape>(ReferenceEqualityComparer.Instance);
        return applyImpl(theShape, theUntil, anInFlight);
    }

    // Recursive worker for Apply with a DFS in-flight set keyed by TShape (by reference). Prevents unbounded descent
    // when a replacement is a compound that transitively contains the original shape as a sub-shape (cyclic
    // containment, distinct from cycles in the replacement map itself).
    private Shape applyImpl(in Shape theShape, ShapeType theUntil, NCollection_Map<TShape> theInFlight)
    {
        _status = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (theShape.IsNull)
        {
            return theShape;
        }

        // Apply direct replacement
        Shape aNewShape = InternalValue(theShape);

        // If shape removed, return NULL
        if (aNewShape.IsNull)
        {
            _status = ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
            return aNewShape;
        }

        // DFS cycle guard: if theShape is already being processed further up the call
        // stack, its replacement must be a compound that transitively contains it.
        // Return the direct replacement without descending to break the cycle.
        if (theInFlight.Contains(theShape.TShape()!))
        {
            return aNewShape;
        }

        // If shape was replaced, apply modifications to the result recursively.
        bool aConsLoc = ModeConsiderLocation();
        if ((aConsLoc && !aNewShape.IsPartner(theShape)) || (!aConsLoc && !aNewShape.IsSame(theShape)))
        {
            theInFlight.Add(theShape.TShape()!);
            Shape aRes = applyImpl(aNewShape, theUntil, theInFlight);
            theInFlight.Remove(theShape.TShape()!);
            _status |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
            return aRes;
        }

        ShapeType aST = theShape.ShapeType();
        if (aST >= theUntil)
        {
            return aNewShape; // stop criterion
        }
        if (aST == ShapeType.Vertex || aST == ShapeType.Shape)
        {
            return theShape;
        }

        BrepBuilder aBuilder = new();

        Shape aResult = theShape.EmptyCopied();
        Orientation anOrient = theShape.Orientation();
        aResult.Orientation(Forward); // protect against INTERNAL or EXTERNAL shapes
        bool aModif = false;
        int aLocStatus = _status;

        // Apply recorded modifications to subshapes
        theInFlight.Add(theShape.TShape()!);
        for (var anIt = new Iterator(theShape, false); anIt.More(); anIt.Next())
        {
            Shape aSh = anIt.Value();
            aNewShape = applyImpl(aSh, theUntil, theInFlight);
            if (aNewShape != aSh)
            {
                if (ShapeExtend.DecodeStatus(_status, ShapeExtend_DONE4))
                {
                    aLocStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE4);
                }
                aModif = true;
            }
            if (aNewShape.IsNull)
            {
                aLocStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE4);
                continue;
            }
            aLocStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE3);
            if (aST == ShapeType.Compound || aNewShape.ShapeType() == aSh.ShapeType())
            {
                // Fix for SAMTECH bug OCC322 about absence internal vertices after sewing
                aBuilder.Add(aResult, aNewShape);
                continue;
            }
            int aNbItems = 0;
            for (var aSubIt = new Iterator(aNewShape); aSubIt.More(); aSubIt.Next(), aNbItems++)
            {
                Shape aSubSh = aSubIt.Value();
                if (aSubSh.ShapeType() == aSh.ShapeType())
                {
                    aBuilder.Add(aResult, aSubSh);
                }
                else
                {
                    aLocStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
                }
            }
            if (aNbItems == 0)
            {
                aLocStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
            }
        }
        theInFlight.Remove(theShape.TShape()!);
        if (!aModif)
        {
            return theShape;
        }

        // Restore range on edge broken by EmptyCopied()
        if (aST == ShapeType.Edge)
        {
            ShapeBuild_Edge anSBE = new();
            anSBE.CopyRanges(ShapeCasts.Edge(aResult), ShapeCasts.Edge(theShape));
        }
        else if (aST == ShapeType.Wire || aST == ShapeType.Shell)
        {
            aResult.Closed(Tool.IsClosed(aResult));
        }
        aResult.Orientation(anOrient);
        _status = aLocStatus;

        ReplaceCore(theShape, aResult, aResult.IsNull ? TReplacementKind.TReplacementKind_Remove : TReplacementKind.TReplacementKind_Modify);

        return aResult;
    }

    /// <summary>
    /// The status of the last Apply: OK (nothing replaced or removed), DONE1 (the starting shape replaced), DONE2 (it
    /// was removed), DONE3 (sub-shapes replaced), DONE4 (sub-shapes removed), FAIL1 (replacements of a bad type not
    /// done).
    /// </summary>
    public virtual bool Status(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(_status, status);

    // Freezes the graph of a shape handed to a caller (docs/phase3-design.md, section 4.2).
    private static Shape Published(in Shape theShape)
    {
        theShape.TShape()?.FreezeGraph();
        return theShape;
    }
}
