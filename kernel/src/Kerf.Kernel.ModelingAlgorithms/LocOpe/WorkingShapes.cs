// OCCT.NET: the working overlay of LocOpe's local operations and BRepFeat's features (units 7H, 10H5), which write into
// the shapes they are given (point representations added to vertices, pcurves to edges, tolerances raised,
// continuities set). Not a port of an OCCT file.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology.Tools;

using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Features.Local;

/// <summary>
/// The working overlay of one local operation, shared by every argument and by the operations it runs, and the way to
/// the result's shapes at its end.
/// </summary>
/// <remarks>
/// <para>
/// A local operation (LocOpe_SplitShape, LocOpe_WiresOnShape with LocOpe_Spliter, LocOpe_Generator, LocOpe_Gluer, the
/// sweeps, a BRepFeat feature) takes a shape and other shapes that refer to it or lie on it (splitting edges and wires,
/// vertices, a glued shape) and, as OCCT does, writes into all of them, often in a later public call than the one that
/// gave them. A shape a caller holds is frozen, so the operation works on the caller's shapes in a detached working
/// overlay (<see cref="WorkingOverlay.Detached"/>, unit 10H5) that each of its public calls that may write enters
/// (<see cref="Enter"/>), so that nothing stays open between the calls. An operation given another's working shapes
/// (the gluer's wires on the shape and generator) works in that one's overlay; one run inside an operation's overlay
/// that takes writes works in that one (<see cref="WorkingOverlay.ForTool"/>), and leaves it to finish.
/// </para>
/// <para>
/// <see cref="Finish"/> ends the operation: the shapes it built or worked with get new TShapes for the caller's
/// sub-shapes it changed, the caller's own for the rest; <see cref="Final(in Shape)"/> then maps a shape the operation
/// worked with to what stands for it. The operation's public accessors freeze what they hand out.
/// </para>
/// </remarks>
internal sealed class WorkingShapes
{
    private WorkingOverlay? _overlay; // made at the first Enter
    private bool _owns;               // false: borrowed from the operation the tool runs in, which finishes it
    private bool _isFinished;

    /// <summary>
    /// True when the operation wrote into the caller's frozen TShapes in an overlay of its own, which
    /// <see cref="Finish"/> then assembles.
    /// </summary>
    public bool IsWritten => _owns && _overlay is { WorkingCount: > 0 };

    /// <summary>True after <see cref="Finish"/>.</summary>
    public bool IsFinished => _isFinished;

    /// <summary>
    /// Makes the operation's working overlay current here until the scope is disposed: around each public call that
    /// may write into the shapes it was given. Nothing once finished.
    /// </summary>
    public WorkingOverlay.Scope Enter()
    {
        if (_isFinished)
        {
            return default;
        }
        _overlay ??= WorkingOverlay.ForTool(new Shape(), null, out _owns);
        return _overlay.Enter();
    }

    /// <summary>
    /// Ends the operation: the shapes <paramref name="shapes"/> it built or worked with are assembled (the operation's
    /// own TShapes get their children replaced in place; a changed caller's shape is replaced, see
    /// <see cref="Final(in Shape)"/>). Once only; nothing is done when nothing was written, or when the overlay belongs
    /// to the operation this one runs in.
    /// </summary>
    public void Finish(IEnumerable<Shape> shapes)
    {
        if (_isFinished)
        {
            return;
        }
        _isFinished = true;
        if (!_owns || _overlay is null)
        {
            return;
        }
        _overlay.Finish(new Shape(), shapes);
    }

    /// <summary>
    /// After <see cref="Finish"/>, what stands for a shape the operation worked with: for a caller's shape it changed
    /// (or one above it), the new TShape; else the shape itself. Before, the shape itself.
    /// </summary>
    public Shape Final(in Shape shape) => _owns && _overlay is { IsFinished: true } ? _overlay.Final(shape) : shape;

    /// <summary>A new list of the <see cref="Final(in Shape)"/> shapes of <paramref name="list"/>.</summary>
    public NCollection_List<Shape> Final(NCollection_List<Shape> list)
    {
        var aList = new NCollection_List<Shape>();
        for (NCollection_List<Shape>.Iterator it = new(list); it.More(); it.Next())
        {
            aList.Append(Final(it.Value()));
        }
        return aList;
    }

    /// <summary>Freezes the shapes of a list a public accessor hands out, and returns it.</summary>
    public static NCollection_List<Shape> Published(NCollection_List<Shape> list)
    {
        for (NCollection_List<Shape>.Iterator it = new(list); it.More(); it.Next())
        {
            it.Value().TShape()?.FreezeGraph();
        }
        return list;
    }

    /// <summary>Freezes a shape a public accessor hands out, and returns it.</summary>
    public static Shape Published(Shape shape)
    {
        shape.TShape()?.FreezeGraph();
        return shape;
    }
}
