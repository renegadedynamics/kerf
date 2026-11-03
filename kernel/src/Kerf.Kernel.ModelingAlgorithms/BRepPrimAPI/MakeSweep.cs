// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakeSweep.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Construction.Api;

/// <summary>The root of the sweep algorithms (prisms and revolutions): the shapes at the bottom and the top of the sweep.</summary>
/// <remarks>
/// <para><see cref="FirstShape"/> and <see cref="LastShape"/> are publication points: they freeze what they return
/// (design 4.2). OCCT's virtuals are the internal <see cref="InternalFirstShape"/> and <see cref="InternalLastShape"/>.</para>
/// <para>OCCT.NET: a sweep without copy writes into its generating shape (pcurves of the profile's edges on the new faces,
/// vertex parameters, 3D curves, tolerances) and shares its sub-shapes in the result. When the generating shape holds
/// frozen TShapes, the derived algorithms sweep an exact copy of it and give back, in the result, the generating
/// shape's own sub-shapes that the sweep left unchanged (design 4.6, decision 32xx-S2); the history answers for the
/// caller's sub-shapes. These helpers are not OCCT's.</para>
/// </remarks>
public abstract class MakeSweep : MakeShape
{
    // OCCT.NET: the copy for write of a generating shape holding frozen TShapes
    private Shape _genOriginal = new();
    private Shape _genCopy = new();
    private Dictionary<TShape, TShape>? _genCopies; // original's TShape -> copy's
    private Dictionary<TShape, TShape>? _genFinal; // copy's TShape -> the one standing for it now

    private protected MakeSweep()
    {
    }

    /// <summary>The shape at the bottom of the sweep (frozen).</summary>
    public Shape FirstShape() => Published(InternalFirstShape());

    /// <summary>The shape at the top of the sweep (frozen).</summary>
    public Shape LastShape() => Published(InternalLastShape());

    /// <summary><see cref="FirstShape()"/> without freezing (OCCT's virtual).</summary>
    internal abstract Shape InternalFirstShape();

    /// <summary><see cref="LastShape()"/> without freezing (OCCT's virtual).</summary>
    internal abstract Shape InternalLastShape();

    private protected static Shape Published(Shape shape)
    {
        shape.TShape()?.FreezeGraph();
        return shape;
    }

    // ------------------------------------------------------------ OCCT.NET: copy for write of the generating shape

    /// <summary>
    /// The shape to sweep in place of <paramref name="s"/>: an exact copy of it when it holds frozen TShapes,
    /// otherwise <paramref name="s"/> itself (OCCT's in-place sweep).
    /// </summary>
    private protected Shape GeneratingShapeForWrite(in Shape s)
    {
        _genOriginal = s;
        if (s.IsNull || !HasFrozen(s.TShape()!, new HashSet<TShape>(ReferenceEqualityComparer.Instance)))
        {
            _genCopy = s;
            _genCopies = null;
            return s;
        }
        _genCopy = BrepTools.ExactCopyForWrite(s, out _, out _genCopies);
        return _genCopy;
    }

    private static bool HasFrozen(TShape tShape, HashSet<TShape> visited)
    {
        if (tShape.IsFrozen)
        {
            return true;
        }
        if (!visited.Add(tShape))
        {
            return false;
        }
        for (var it = new Iterator(new Shape(tShape, new Placement(), Orientation.Forward));
             it.More(); it.Next())
        {
            if (HasFrozen(it.Value().TShape()!, visited))
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>
    /// After the sweep of a copy: gives back the generating shape's sub-shapes the sweep left unchanged, in the
    /// result <paramref name="result"/> too (BRepTools.KeepUnchangedIn).
    /// </summary>
    private protected void KeepUnchangedGenerating(ref Shape result)
    {
        if (_genCopies is null || _genFinal is not null)
        {
            return; // nothing copied, or already done (a second Build gets the same graph back)
        }
        // the copy's graph is left as the sweep made it: the sweep reads it again for the history and the shapes it
        // builds on demand (its seam test walks the generating shape's faces)
        _genFinal = BrepTools.KeepUnchangedIn(_genOriginal, _genCopy, _genCopies, ref result, out _, true);
    }

    /// <summary>The swept shape's sub-shape standing for the caller's sub-shape <paramref name="s"/> (itself without a copy).</summary>
    private protected Shape ToSwept(in Shape s)
    {
        if (_genCopies is null || s.IsNull || !_genCopies.TryGetValue(s.TShape()!, out TShape? copy))
        {
            return s;
        }
        Shape aS = s;
        aS.TShape(copy);
        return aS;
    }

    /// <summary>A shape the sweep returns, with a copy of the generating shape's sub-shape replaced by the one standing for it now.</summary>
    private protected Shape FromSwept(in Shape s)
    {
        if (_genFinal is null || s.IsNull || !_genFinal.TryGetValue(s.TShape()!, out TShape? now))
        {
            return s;
        }
        Shape aS = s;
        aS.TShape(now);
        return aS;
    }
}
