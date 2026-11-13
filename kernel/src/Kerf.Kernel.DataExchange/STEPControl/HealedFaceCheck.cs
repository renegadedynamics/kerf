// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Healing.Process;
using Kerf.Engine.Topology;
using Kerf.Topology;
using HealingMessages = System.Collections.Generic.Dictionary<Kerf.Topology.Shape, Kerf.Engine.NCollection_List<Kerf.Engine.Foundation.Msg>>;

namespace Kerf.Engine.Exchange.Step;

/// <summary>
/// Kerf (decisions 2307, 2313, 2314): the STEP reader's check of the faces it may have read wrongly. Checking every face
/// costs a fifth of a read; these are the faces where the reader or the healing already knows something happened: (c) the
/// faces the healing reported on (a message on the face or anything in it: a seam added, a wire reversed, an edge fixed)
/// and (d) the faces with a vertex the reader found off its edge's curve by more than the file's uncertainty
/// (<see cref="TransientProcess.OffCurveVertices"/>), and (e) the faces bounded by two edges or fewer, slivers (decision
/// 34706). Each such face of the healed shape is checked as the public
/// <c>Validate</c> checks it, and those that fail go to <see cref="TransientProcess.InvalidFaces"/>, which
/// <c>Step.Read</c> turns into its warning (<c>exchange.invalid-face</c>).
/// </summary>
internal static class HealedFaceCheck
{
    /// <summary>
    /// Checks the faces of <paramref name="healed"/> that came from faces of <paramref name="before"/> the healing
    /// (<paramref name="context"/>, null for none) reported on or that hold a vertex off its curve, and records those
    /// that are invalid in <paramref name="tp"/>.
    /// </summary>
    public static void Check(in Shape before, in Shape healed, ShapeContext? context, TransientProcess tp)
    {
        if (before.IsNull || healed.IsNull)
        {
            return;
        }

        HealingMessages? said = context?.Messages()?.MapShape();
        if (said is { Count: 0 })
        {
            said = null;
        }
        HashSet<Shape> off = tp.OffCurveVertices;

        Dictionary<Shape, Shape>? map = context?.InternalMap();
        var seen = new HashSet<Shape>(ShapeHasher.Instance);
        var checkedFaces = new HashSet<Shape>(ShapeHasher.Instance);
        for (var faces = new Explorer(before, ShapeType.Face); faces.More(); faces.Next())
        {
            Shape face = faces.Current();
            if (!seen.Add(face) || !IsSuspect(face, said, off))
            {
                continue;
            }

            Shape result = face;
            if (map is not null && (map.TryGetValue(face, out Shape mapped) || map.TryGetValue(face.Located(new Placement()), out mapped)))
            {
                result = mapped;
            }
            if (result.IsNull)
            {
                continue;
            }
            for (var pieces = new Explorer(result, ShapeType.Face); pieces.More(); pieces.Next())
            {
                Shape piece = pieces.Current();
                if (checkedFaces.Add(piece) && !new Analyzer(piece, true).IsValid())
                {
                    tp.AddInvalidFace(piece, EntityOf(face, tp));
                }
            }
        }
    }

    /// <summary>
    /// The file's entity <paramref name="face"/> was read from: the one bound to it in <paramref name="tp"/>, looked for
    /// before the healing's results replace the bindings (a face the healing split is bound to the pieces). Only for a
    /// face found invalid, so the walk over the bindings is rare. Null if none is.
    /// </summary>
    private static Kerf.Engine.Foundation.Transient? EntityOf(in Shape face, TransientProcess tp)
    {
        for (int item = 1; item <= tp.MappedCount; item++)
        {
            if (tp.MapItem(item) is ShapeBinder { HasResult: true } binder && binder.Result() is { IsNull: false } bound
                && bound.TShape() == face.TShape())
            {
                return tp.Mapped(item);
            }
        }
        return null;
    }

    /// <summary>
    /// Whether the healing reported on <paramref name="face"/> or anything in it, it has a vertex off its curve, or it is
    /// bounded by two edges or fewer (decision 2427): a sliver between two coincident curves, of no area, whose validity
    /// its pcurves' last bits decide, as <c>bug30790.step</c>'s 150 have it.
    /// </summary>
    private static bool IsSuspect(in Shape face, HealingMessages? said,
                                  HashSet<Shape> off)
    {
        if (HasTwoEdgesOrFewer(face))
        {
            return true;
        }
        if (said is not null && said.ContainsKey(face))
        {
            return true;
        }
        if (said is not null)
        {
            for (var parts = new Explorer(face, ShapeType.Wire); parts.More(); parts.Next())
            {
                if (said.ContainsKey(parts.Current()))
                {
                    return true;
                }
            }
            for (var parts = new Explorer(face, ShapeType.Edge); parts.More(); parts.Next())
            {
                if (said.ContainsKey(parts.Current()))
                {
                    return true;
                }
            }
        }
        for (var parts = new Explorer(face, ShapeType.Vertex); parts.More(); parts.Next())
        {
            if ((said is not null && said.ContainsKey(parts.Current())) || off.Contains(parts.Current()))
            {
                return true;
            }
        }
        return false;
    }

    private static bool HasTwoEdgesOrFewer(in Shape face)
    {
        int count = 0;
        for (var edges = new Explorer(face, ShapeType.Edge); edges.More(); edges.Next())
        {
            if (++count > 2)
            {
                return false;
            }
        }
        return true;
    }
}
