// Port of: src/ModelingAlgorithms/TKShHealing/ShapeBuild/ShapeBuild_ReShape.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeBuild/ShapeBuild_ReShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.ShapeExtend_Status;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Shape Healing's reshaper: records requests to replace or remove shapes (each shape's last request is kept) and
/// applies them to any shape holding some of them, rebuilding their parents. The healing algorithms (ShapeFix,
/// ShapeUpgrade, sewing) share one as their context, and callers read the replacements back from it.
/// </summary>
/// <remarks>
/// <para>
/// A working object, as <see cref="BRepTools_ReShape"/> (decision 358): the public <see cref="Apply(in TopoDS_Shape,
/// TopAbs_ShapeEnum, int)"/> and the inherited <c>Apply</c>, <c>Value</c>, <c>Status</c> and <c>History</c> freeze the
/// shapes they return; ported code uses the internal forms (<see cref="InternalApply(in TopoDS_Shape, TopAbs_ShapeEnum,
/// int)"/> and the inherited <c>InternalApply</c>, <c>InternalValue</c>, ...).
/// </para>
/// <para>
/// Apply only writes into the shapes it builds (EmptyCopied parents, ranges copied onto a rebuilt edge, the Closed
/// flag of rebuilt wires and shells): it works on frozen shapes. A healing algorithm that ran on a copy of a frozen
/// shape (decision 5008) records its replacements of the copy's sub-shapes here; <see cref="BRepTools_CopyAtEntry"/>
/// then re-keys them to the caller's sub-shapes, so <c>Value</c>, <c>Apply</c> and <c>History</c> answer for the
/// shapes the caller holds.
/// </para>
/// </remarks>
public class ShapeBuild_ReShape : BRepTools_ReShape
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
    public TopoDS_Shape Apply(in TopoDS_Shape shape, TopAbs_ShapeEnum until, int buildmode) =>
        Published(InternalApply(shape, until, buildmode));

    /// <summary>The application with a build mode, as <see cref="Apply(in TopoDS_Shape, TopAbs_ShapeEnum, int)"/>, unfrozen (ported code).</summary>
    internal virtual TopoDS_Shape InternalApply(in TopoDS_Shape shape, TopAbs_ShapeEnum until, int buildmode)
    {
        if (shape.IsNull())
        {
            return shape;
        }
        TopoDS_Shape newsh;
        if (InternalStatus(shape, out newsh, false) != 0)
        {
            return newsh;
        }

        TopAbs_ShapeEnum st = shape.ShapeType();
        if (st == until)
        {
            return newsh; // critere d arret
        }

        int modif = 0;
        if (st == TopAbs_COMPOUND || st == TopAbs_COMPSOLID)
        {
            BRep_Builder B = new();
            TopoDS_Compound C = new();
            B.MakeCompound(ref C);
            for (var it = new TopoDS_Iterator(shape); it.More(); it.Next())
            {
                TopoDS_Shape sh = it.Value();
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

        if (st == TopAbs_SOLID)
        {
            BRep_Builder B = new();
            TopoDS_Compound C = new();
            B.MakeCompound(ref C);
            TopoDS_Solid S = new();
            B.MakeSolid(ref S);
            for (var it = new TopoDS_Iterator(shape); it.More(); it.Next())
            {
                TopoDS_Shape sh = it.Value();
                newsh = InternalApply(sh, until, buildmode);
                if (newsh.IsNull())
                {
                    modif = -1;
                }
                else if (newsh.ShapeType() != TopAbs_SHELL)
                {
                    int nbsub = 0;
                    for (var exh = new TopExp_Explorer(newsh, TopAbs_SHELL); exh.More(); exh.Next())
                    {
                        TopoDS_Shape onesh = exh.Current();
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

        if (st == TopAbs_SHELL)
        {
            BRep_Builder B = new();
            TopoDS_Compound C = new();
            B.MakeCompound(ref C);
            TopoDS_Shell S = new();
            B.MakeShell(ref S);
            for (var it = new TopoDS_Iterator(shape); it.More(); it.Next())
            {
                TopoDS_Shape sh = it.Value();
                newsh = InternalApply(sh, until, buildmode);
                if (newsh.IsNull())
                {
                    modif = -1;
                }
                else if (newsh.ShapeType() != TopAbs_FACE)
                {
                    int nbsub = 0;
                    for (var exf = new TopExp_Explorer(newsh, TopAbs_FACE); exf.More(); exf.Next())
                    {
                        TopoDS_Shape onesh = exf.Current();
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
                S.Closed(BRep_Tool.IsClosed(S));
                return S;
            }
        }
        // OCCT prints "BRepTools_ReShape::Apply NOT YET IMPLEMENTED" to the console here; the port doesn't (decision 493).
        return shape;
    }

    /// <summary>
    /// Applies the requests, as <see cref="BRepTools_ReShape.InternalApply"/>: a sub-shape can be replaced by a shape
    /// of its type or by a container of shapes of its type; other types are ignored (status FAIL1). Unlike
    /// BRepTools_ReShape's, sub-shapes of the type <paramref name="theUntil"/> and below aren't explored.
    /// </summary>
    internal override TopoDS_Shape InternalApply(in TopoDS_Shape theShape, TopAbs_ShapeEnum theUntil = TopAbs_SHAPE)
    {
        var anInFlight = new NCollection_Map<TopoDS_TShape>(ReferenceEqualityComparer.Instance);
        return applyImpl(theShape, theUntil, anInFlight);
    }

    // Recursive worker for Apply with a DFS in-flight set keyed by TShape (by reference). Prevents unbounded descent
    // when a replacement is a compound that transitively contains the original shape as a sub-shape (cyclic
    // containment, distinct from cycles in the replacement map itself).
    private new TopoDS_Shape applyImpl(in TopoDS_Shape theShape, TopAbs_ShapeEnum theUntil, NCollection_Map<TopoDS_TShape> theInFlight)
    {
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (theShape.IsNull())
        {
            return theShape;
        }

        // Apply direct replacement
        TopoDS_Shape aNewShape = InternalValue(theShape);

        // If shape removed, return NULL
        if (aNewShape.IsNull())
        {
            myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
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
            TopoDS_Shape aRes = applyImpl(aNewShape, theUntil, theInFlight);
            theInFlight.Remove(theShape.TShape()!);
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
            return aRes;
        }

        TopAbs_ShapeEnum aST = theShape.ShapeType();
        if (aST >= theUntil)
        {
            return aNewShape; // stop criterion
        }
        if (aST == TopAbs_VERTEX || aST == TopAbs_SHAPE)
        {
            return theShape;
        }

        BRep_Builder aBuilder = new();

        TopoDS_Shape aResult = theShape.EmptyCopied();
        TopAbs_Orientation anOrient = theShape.Orientation();
        aResult.Orientation(TopAbs_FORWARD); // protect against INTERNAL or EXTERNAL shapes
        bool aModif = false;
        int aLocStatus = myStatus;

        // Apply recorded modifications to subshapes
        theInFlight.Add(theShape.TShape()!);
        for (var anIt = new TopoDS_Iterator(theShape, false); anIt.More(); anIt.Next())
        {
            TopoDS_Shape aSh = anIt.Value();
            aNewShape = applyImpl(aSh, theUntil, theInFlight);
            if (aNewShape != aSh)
            {
                if (ShapeExtend.DecodeStatus(myStatus, ShapeExtend_DONE4))
                {
                    aLocStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE4);
                }
                aModif = true;
            }
            if (aNewShape.IsNull())
            {
                aLocStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE4);
                continue;
            }
            aLocStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE3);
            if (aST == TopAbs_COMPOUND || aNewShape.ShapeType() == aSh.ShapeType())
            {
                // Fix for SAMTECH bug OCC322 about absence internal vertices after sewing
                aBuilder.Add(aResult, aNewShape);
                continue;
            }
            int aNbItems = 0;
            for (var aSubIt = new TopoDS_Iterator(aNewShape); aSubIt.More(); aSubIt.Next(), aNbItems++)
            {
                TopoDS_Shape aSubSh = aSubIt.Value();
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
        if (aST == TopAbs_EDGE)
        {
            ShapeBuild_Edge anSBE = new();
            anSBE.CopyRanges(TopoDS.Edge(aResult), TopoDS.Edge(theShape));
        }
        else if (aST == TopAbs_WIRE || aST == TopAbs_SHELL)
        {
            aResult.Closed(BRep_Tool.IsClosed(aResult));
        }
        aResult.Orientation(anOrient);
        myStatus = aLocStatus;

        replace(theShape, aResult, aResult.IsNull() ? TReplacementKind.TReplacementKind_Remove : TReplacementKind.TReplacementKind_Modify);

        return aResult;
    }

    /// <summary>
    /// The status of the last Apply: OK (nothing replaced or removed), DONE1 (the starting shape replaced), DONE2 (it
    /// was removed), DONE3 (sub-shapes replaced), DONE4 (sub-shapes removed), FAIL1 (replacements of a bad type not
    /// done).
    /// </summary>
    public virtual bool Status(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatus, status);

    // Freezes the graph of a shape handed to a caller (docs/phase3-design.md, section 4.2).
    private static TopoDS_Shape Published(in TopoDS_Shape theShape)
    {
        theShape.TShape()?.FreezeGraph();
        return theShape;
    }
}
