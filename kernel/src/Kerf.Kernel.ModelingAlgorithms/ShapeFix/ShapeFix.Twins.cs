// Kerf: the functional twins of ShapeFix's in-place statics (docs/phase5-6-plan.md, section 2.7; decision 583),
// on the working overlay since Kerf unit 10H4. Not a port of an OCCT file.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using Kerf.Engine;

namespace Kerf.Engine.Healing.Fixes;

internal static partial class ShapeFixFixer
{
    /// <summary>
    /// The functional twin of <see cref="SameParameter"/>: the shape with its edges made SameParameter by
    /// ShapeFix_Edge and the tolerances of its edges on planes updated, frozen. The fix runs on the caller's shape in a
    /// working overlay: the sub-shapes left unchanged are the caller's own, and a shape needing nothing is returned
    /// itself.
    /// <paramref name="isDone"/> is OCCT's result (false if an edge couldn't be made SameParameter, or on a user
    /// break).
    /// </summary>
    public static Shape SameParametered(in Shape shape, bool enforce, out bool isDone, double preci = 0.0,
                                               ProgressRange progress = default,
                                               BasicMsgRegistrator? msgReg = null)
    {
        Shape work = WorkingOverlay.WithUnfrozenCopied(shape, out Dictionary<TShape, TShape>? copies);
        using var overlay = WorkingOverlay.Begin(work);
        isDone = SameParameter(work, enforce, preci, progress, msgReg);
        return Frozen(WorkingOverlay.UncopiedWhereUnchanged(overlay.Finish(work), copies));
    }

    /// <summary>
    /// The functional twin of <see cref="EncodeRegularity"/>: the shape with the continuity of its edges between
    /// their faces encoded, frozen; see <see cref="SameParametered"/>.
    /// </summary>
    public static Shape RegularityEncoded(in Shape shape, double tolang = 1.0e-10)
    {
        Shape work = WorkingOverlay.WithUnfrozenCopied(shape, out Dictionary<TShape, TShape>? copies);
        using var overlay = WorkingOverlay.Begin(work);
        EncodeRegularity(work, tolang);
        return Frozen(WorkingOverlay.UncopiedWhereUnchanged(overlay.Finish(work), copies));
    }

    /// <summary>
    /// The functional twin of <see cref="FixVertexPosition"/>: the shape with its vertices more tolerant than
    /// <paramref name="tolerance"/> moved to the curves' ends (the replacements recorded in
    /// <paramref name="thecontext"/>, which answers for the caller's sub-shapes), frozen; the shape itself when
    /// nothing was done (<paramref name="isDone"/> false).
    /// </summary>
    public static Shape VertexPositionFixed(in Shape theshape, double tolerance, ReShapeBuilder thecontext,
                                                   out bool isDone)
    {
        Shape work = WorkingOverlay.WithUnfrozenCopied(theshape, out Dictionary<TShape, TShape>? copies);
        using var overlay = WorkingOverlay.Begin(work, thecontext);
        isDone = FixVertexPosition(ref work, tolerance, thecontext);
        return Frozen(WorkingOverlay.UncopiedWhereUnchanged(overlay.Finish(work), copies));
    }

    private static Shape Frozen(in Shape shape)
    {
        shape.TShape()?.FreezeGraph();
        return shape;
    }
}
