// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_BuilderArea.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_BuilderArea.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Topology;
using Kerf.Topology;
using Kerf.Engine;

namespace Kerf.Engine.Booleans;

/// <summary>
/// The root of the algorithms that build areas from a set of shapes: faces from edges (BOPAlgo_BuilderFace) and solids
/// from faces (BOPAlgo_BuilderSolid).
/// </summary>
internal abstract class BuilderArea : BooleanAlgorithm
{
    protected IntersectionContext? _context;
    protected NCollection_List<Shape> _shapes = new();
    protected NCollection_List<Shape> _loops = new();
    protected NCollection_List<Shape> _loopsInternal = new();
    protected NCollection_List<Shape> _areas = new();
    protected IndexedSet<Shape> _shapesToAvoid = new(ShapeHasher.Instance);
    protected bool _avoidInternalShapes;

    protected BuilderArea()
    {
        _avoidInternalShapes = false;
    }

    /// <summary>Sets the context for the algorithms.</summary>
    public void SetContext(IntersectionContext? context) => _context = context;

    /// <summary>The input shapes.</summary>
    public NCollection_List<Shape> Shapes() => _shapes;

    /// <summary>Sets the shapes for building areas (copies the list).</summary>
    public void SetShapes(NCollection_List<Shape> ls) => _shapes = new NCollection_List<Shape>(ls);

    /// <summary>The found loops.</summary>
    public NCollection_List<Shape> Loops() => _loops;

    /// <summary>The found areas.</summary>
    public NCollection_List<Shape> Areas() => _areas;

    /// <summary>Defines the preventing of addition of internal parts into the result.</summary>
    public void SetAvoidInternalShapes(bool avoidInternal) => _avoidInternalShapes = avoidInternal;

    /// <summary>True if the internal parts are not added into the result.</summary>
    public bool IsAvoidInternalShapes => _avoidInternalShapes;

    protected abstract void PerformShapesToAvoid(ProgressRange range);

    protected abstract void PerformLoops(ProgressRange range);

    protected abstract void PerformAreas(ProgressRange range);

    protected abstract void PerformInternalShapes(ProgressRange range);
}
