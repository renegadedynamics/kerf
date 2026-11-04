// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakeSweep.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology.Tools;
using Kerf.Topology;

namespace Kerf.Engine.Construction.Api;

/// <summary>The root of the sweep algorithms (prisms and revolutions): the shapes at the bottom and the top of the sweep.</summary>
/// <remarks>
/// <para><see cref="FirstShape"/> and <see cref="LastShape"/> are publication points: they freeze what they return
/// (design 4.2). OCCT's virtuals are the internal <see cref="InternalFirstShape()"/> and <see cref="InternalLastShape()"/>.</para>
/// <para>Kerf: a sweep without copy writes into its generating shape (pcurves of the profile's edges on the new faces,
/// vertex parameters, 3D curves, tolerances) and shares its sub-shapes in the result. The derived algorithms sweep the
/// caller's shape itself in a working overlay (unit 10H3): the writes go to working copies, and the result holds the
/// generating shape's own sub-shapes wherever the sweep left them unchanged. The sweep's tables and histories are
/// keyed by the caller's sub-shapes; what they hand out is mapped to what stands for it in the result
/// (<see cref="FromSwept"/>). These helpers are not OCCT's.</para>
/// </remarks>
internal abstract class MakeSweep : MakeShape
{
    // Kerf: the working overlay the generating shape is swept in (unit 10H3)
    private WorkingOverlay? _overlay;

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

    /// <summary>The shape at the bottom of the sweep generated with <paramref name="shape"/>, a sub-shape of the
    /// generating shape, unfrozen; null where the sweep has none (OCCT's <c>FirstShape(S)</c> of the prism and the
    /// revolution).</summary>
    internal virtual Shape InternalFirstShape(in Shape shape) => new();

    /// <summary>The shape at the top of the sweep generated with <paramref name="shape"/>, unfrozen; null where the sweep
    /// has none.</summary>
    internal virtual Shape InternalLastShape(in Shape shape) => new();

    /// <summary>
    /// Kerf: the bottom and top shapes generated with <paramref name="shape"/> (the caps of a face's sweep, the bottom
    /// and top edges of an edge's), asked as a history query, which may build on demand (unit 10L5, for the public
    /// sweeps' histories).
    /// </summary>
    internal (Shape First, Shape Last) CapsOf(in Shape shape)
    {
        using WorkingOverlay query = BeginQuery();
        return (InternalFirstShape(shape), InternalLastShape(shape));
    }

    private protected static Shape Published(Shape shape)
    {
        shape.TShape()?.FreezeGraph();
        return shape;
    }

    // ------------------------------------------------------------ Kerf: the generating shape in a working overlay

    /// <summary>
    /// Opens the working overlay the sweep of <paramref name="s"/> runs in, current until disposed: the constructor
    /// disposes it once <see cref="FinishGenerating"/> has assembled the result. Null when the sweep runs inside an
    /// operation's overlay (an evolved shape's): its writes are that operation's, as OCCT's are in place.
    /// </summary>
    private protected WorkingOverlay? BeginGenerating(in Shape s)
    {
        _overlay = WorkingOverlay.BeginUnlessWriting(s);
        return _overlay;
    }

    /// <summary>
    /// Assembles the result of the sweep: the generating shape's sub-shapes it changed are replaced by their new
    /// TShapes, in <paramref name="result"/> and in the sweep's own graph. Once only (a second Build gets the same graph).
    /// </summary>
    private protected void FinishGenerating(ref Shape result)
    {
        if (_overlay is null || _overlay.IsFinished)
        {
            return;
        }
        result = _overlay.Finish(result);
    }

    /// <summary>
    /// Opens, for a history query that may build on demand (OCCT's sweep builds the edge a vertex generates when no
    /// face built it, and sets the vertex's parameter on it), an overlay that is dropped unfinished: the vertex is the
    /// generating shape's, which the result shares, and the write would leave it as it is (a FORWARD or REVERSED
    /// vertex gets only the new edge's ranges, and a tolerance of at least <c>Precision::PConfusion()</c>).
    /// </summary>
    private protected static WorkingOverlay BeginQuery() => WorkingOverlay.Begin(new Shape());

    /// <summary>A shape the sweep returns, with a sub-shape of the generating shape mapped to what stands for it now.</summary>
    private protected Shape FromSwept(in Shape s)
    {
        if (_overlay is null || !_overlay.IsFinished || s.IsNull)
        {
            return s;
        }
        return _overlay.Final(s);
    }
}
