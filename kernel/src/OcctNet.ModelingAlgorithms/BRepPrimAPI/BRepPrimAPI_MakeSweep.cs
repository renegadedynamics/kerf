// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakeSweep.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;

namespace Occt;

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
public abstract class BRepPrimAPI_MakeSweep : BRepBuilderAPI_MakeShape
{
    // OCCT.NET: the copy for write of a generating shape holding frozen TShapes
    private Shape myGenOriginal = new();
    private Shape myGenCopy = new();
    private Dictionary<TShape, TShape>? myGenCopies; // original's TShape -> copy's
    private Dictionary<TShape, TShape>? myGenFinal; // copy's TShape -> the one standing for it now

    private protected BRepPrimAPI_MakeSweep()
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

    private protected static Shape Published(Shape theShape)
    {
        theShape.TShape()?.FreezeGraph();
        return theShape;
    }

    // ------------------------------------------------------------ OCCT.NET: copy for write of the generating shape

    /// <summary>
    /// The shape to sweep in place of <paramref name="theS"/>: an exact copy of it when it holds frozen TShapes,
    /// otherwise <paramref name="theS"/> itself (OCCT's in-place sweep).
    /// </summary>
    private protected Shape GeneratingShapeForWrite(in Shape theS)
    {
        myGenOriginal = theS;
        if (theS.IsNull || !HasFrozen(theS.TShape()!, new HashSet<TShape>(ReferenceEqualityComparer.Instance)))
        {
            myGenCopy = theS;
            myGenCopies = null;
            return theS;
        }
        myGenCopy = BrepTools.ExactCopyForWrite(theS, out _, out myGenCopies);
        return myGenCopy;
    }

    private static bool HasFrozen(TShape theTShape, HashSet<TShape> theVisited)
    {
        if (theTShape.IsFrozen)
        {
            return true;
        }
        if (!theVisited.Add(theTShape))
        {
            return false;
        }
        for (var anIt = new Iterator(new Shape(theTShape, new Placement(), Orientation.Forward));
             anIt.More(); anIt.Next())
        {
            if (HasFrozen(anIt.Value().TShape()!, theVisited))
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>
    /// After the sweep of a copy: gives back the generating shape's sub-shapes the sweep left unchanged, in the
    /// result <paramref name="theResult"/> too (BRepTools.KeepUnchangedIn).
    /// </summary>
    private protected void KeepUnchangedGenerating(ref Shape theResult)
    {
        if (myGenCopies is null || myGenFinal is not null)
        {
            return; // nothing copied, or already done (a second Build gets the same graph back)
        }
        // the copy's graph is left as the sweep made it: the sweep reads it again for the history and the shapes it
        // builds on demand (its seam test walks the generating shape's faces)
        myGenFinal = BrepTools.KeepUnchangedIn(myGenOriginal, myGenCopy, myGenCopies, ref theResult, out _, true);
    }

    /// <summary>The swept shape's sub-shape standing for the caller's sub-shape <paramref name="theS"/> (itself without a copy).</summary>
    private protected Shape ToSwept(in Shape theS)
    {
        if (myGenCopies is null || theS.IsNull || !myGenCopies.TryGetValue(theS.TShape()!, out TShape? aCopy))
        {
            return theS;
        }
        Shape aS = theS;
        aS.TShape(aCopy);
        return aS;
    }

    /// <summary>A shape the sweep returns, with a copy of the generating shape's sub-shape replaced by the one standing for it now.</summary>
    private protected Shape FromSwept(in Shape theS)
    {
        if (myGenFinal is null || theS.IsNull || !myGenFinal.TryGetValue(theS.TShape()!, out TShape? aNow))
        {
            return theS;
        }
        Shape aS = theS;
        aS.TShape(aNow);
        return aS;
    }
}
