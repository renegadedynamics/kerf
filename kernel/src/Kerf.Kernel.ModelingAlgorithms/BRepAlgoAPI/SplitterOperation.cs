// Port of: src/ModelingAlgorithms/TKBO/BRepAlgoAPI/BRepAlgoAPI_Splitter.hxx
// Port of: src/ModelingAlgorithms/TKBO/BRepAlgoAPI/BRepAlgoAPI_Splitter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2017-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Booleans.Api;

/// <summary>
/// Splits the objects by the tools: the result holds the splits of the objects only (the splits of the tools are
/// left out). Error: <see cref="AlertTooFewArguments"/> without objects or with fewer than two shapes.
/// </summary>
internal class SplitterOperation : BuilderAlgo
{
    protected NCollection_List<Shape> _tools = new(); //!< Tool arguments of the operation

    /// <summary>An empty splitter.</summary>
    public SplitterOperation()
    {
    }

    /// <summary>A splitter using the intersections already computed by <paramref name="pf"/>.</summary>
    public SplitterOperation(PaveFiller pf)
        : base(pf)
    {
    }

    /// <summary>Sets the tools (copies the list).</summary>
    public void SetTools(NCollection_List<Shape> ls) => _tools = new NCollection_List<Shape>(ls);

    /// <summary>The tools.</summary>
    public NCollection_List<Shape> Tools() => _tools;

    /// <summary>Performs the split.</summary>
    public override void Build(ProgressRange range = default)
    {
        // Set Not Done status by default
        NotDone();
        // Clear the contents
        Clear();
        // Check for availability of arguments and tools
        if (_arguments.IsEmpty() || (_arguments.Extent() + _tools.Extent()) < 2)
        {
            AddError(new AlertTooFewArguments());
            return;
        }

        // If necessary perform intersection of the argument shapes
        using var ps = new ProgressScope(range, "Performing Split operation", _isIntersectionNeeded ? 100 : 30);
        if (_isIntersectionNeeded)
        {
            // Combine Objects and Tools into a single list for intersection
            NCollection_List<Shape> lArgs = new(_arguments);
            for (NCollection_List<Shape>.Iterator it = new(_tools); it.More(); it.Next())
            {
                lArgs.Append(it.Value());
            }

            // Perform intersection
            IntersectShapes(lArgs, ps.Next(70));
            if (HasErrors)
            {
                return;
            }
        }

        // Initialization of the building tool
        Splitter splitter = new();
        _builder = splitter;
        _builder.SetArguments(_arguments);
        splitter.SetTools(_tools);

        // Build result shape basing on the intersection results
        BuildResult(ps.Next(30));
    }
}
