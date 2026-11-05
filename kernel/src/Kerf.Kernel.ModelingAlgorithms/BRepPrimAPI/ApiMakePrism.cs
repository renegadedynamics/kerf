// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakePrism.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakePrism.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Library;
using Kerf.Engine.Construction.Sweeping;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology.Tools;

using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;

namespace Kerf.Engine.Construction.Api;

/// <summary>
/// Builds the linear swept topology of a shape (a prism): vertex to edge, edge to face, wire to shell, face to solid,
/// shell to compsolid, compound to compound; along a vector, or semi-infinite or infinite along a direction.
/// </summary>
/// <remarks>
/// <para>Without copy, OCCT shares the base's sub-shapes in the prism and writes pcurves and parameters into them. Kerf
/// sweeps the caller's base itself in a working overlay (<see cref="MakeSweep"/>): the result holds the base's own
/// sub-shapes wherever the sweep left them unchanged, the history answers for the caller's sub-shapes directly, and
/// the base is never changed.</para>
/// <para>The result, <see cref="MakeSweep.FirstShape()"/>, <see cref="LastShape(in Shape)"/> and the
/// history are frozen when returned (design 4.2).</para>
/// </remarks>
internal class ApiMakePrism : MakeSweep
{
    private readonly Prism _prism;

    // perform checks on the argument
    private static Shape CheckCore(in Shape s)
    {
        BrepLibrary.BuildCurves3d(s);
        return s;
    }

    /// <summary>
    /// The prism of base <paramref name="s"/> and vector <paramref name="v"/>. With <paramref name="copy"/>, S's
    /// sub-shapes are copied; with <paramref name="canonize"/>, the extrusions of lines and circles are planes and
    /// cylinders.
    /// </summary>
    public ApiMakePrism(in Shape s, in Vector3D v, bool copy = false, bool canonize = true)
    {
        using WorkingOverlay? overlay = BeginGenerating(s);
        _prism = new Prism(CheckCore(s), v, copy, canonize);
        Build();
    }

    /// <summary>
    /// A semi-infinite (<paramref name="inf"/> false) or infinite prism of base <paramref name="s"/> along
    /// <paramref name="d"/>.
    /// </summary>
    public ApiMakePrism(in Shape s, in Direction3D d, bool inf = true, bool copy = false, bool canonize = true)
    {
        using WorkingOverlay? overlay = BeginGenerating(s);
        _prism = new Prism(CheckCore(s), d, inf, copy, canonize);
        Build();
    }

    /// <summary>The internal sweeping algorithm (keyed by the base's own sub-shapes).</summary>
    internal Prism Prism() => _prism;

    /// <summary>Builds the resulting shape (redefined from MakeShape).</summary>
    public override void Build(ProgressRange range = default)
    {
        _shape = _prism.Shape();
        FinishGenerating(ref _shape);
        Done();
    }

    internal override Shape InternalFirstShape() => FromSwept(_prism.FirstShape());

    internal override Shape InternalLastShape() => FromSwept(_prism.LastShape());

    /// <summary>The list of shapes generated from the shape <paramref name="s"/>.</summary>
    internal override NCollection_List<Shape> InternalGenerated(in Shape s)
    {
        _generated.Clear();
        using WorkingOverlay query = BeginQuery(); // Kerf: may build on demand
        if (_prism.IsUsed(s) && _prism.GenIsUsed(s))
        {
            _generated.Append(FromSwept(_prism.Shape(s)));
        }
        return _generated;
    }

    /// <summary>The shape at the bottom of the prism generated with <paramref name="shape"/>, a sub-shape of the base (frozen).</summary>
    public Shape FirstShape(in Shape shape) => Published(InternalFirstShape(shape));

    /// <summary><see cref="FirstShape(in Shape)"/> without freezing.</summary>
    internal override Shape InternalFirstShape(in Shape shape) => FromSwept(_prism.FirstShape(shape));

    /// <summary>The shape at the top of the prism generated with <paramref name="shape"/>, a sub-shape of the base (frozen).</summary>
    public Shape LastShape(in Shape shape) => Published(InternalLastShape(shape));

    /// <summary><see cref="LastShape(in Shape)"/> without freezing.</summary>
    internal override Shape InternalLastShape(in Shape shape) => FromSwept(_prism.LastShape(shape));

    /// <summary>True if the shape <paramref name="s"/> has been deleted.</summary>
    public override bool IsDeleted(in Shape s)
    {
        return !_prism.IsUsed(s);
    }
}
