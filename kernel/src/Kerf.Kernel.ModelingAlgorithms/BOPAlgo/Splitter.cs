// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Splitter.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Splitter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2017-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Booleans;

/// <summary>Splits the objects by the tools; the result holds the split objects only.</summary>
/// <remarks>
/// The objects and tools are intersected together (General Fuse), and the result is built from the splits of the
/// objects. With one object split into one shape (or not split), the result is that shape, not a compound.
/// </remarks>
internal class Splitter : ToolsProvider
{
    public Splitter()
    {
    }

    /// <summary>Intersects the objects and tools with a new pave filler and builds the result.</summary>
    public override void Perform(ProgressRange range = default)
    {
        GetReport().Clear();
        //
        if (_entryPoint == 1)
        {
            _paveFiller = null;
        }
        //
        // prepare shapes for intersection
        NCollection_List<Shape> ls = new();
        //
        NCollection_List<Shape>.Iterator itLS = new(_arguments);
        for (; itLS.More(); itLS.Next())
        {
            ls.Append(itLS.Value());
        }
        //
        itLS.Initialize(_tools);
        for (; itLS.More(); itLS.Next())
        {
            ls.Append(itLS.Value());
        }
        //
        PaveFiller pPF = new();
        pPF.SetArguments(ls);
        pPF.SetRunParallel(_runParallel);

        pPF.SetFuzzyValue(_fuzzyValue);
        pPF.SetNonDestructive(_nonDestructive);
        pPF.SetGlue(_glue);
        pPF.SetUseOBB(_useOBB);
        //
        using var ps = new ProgressScope(range, "Performing Split operation", 10);
        pPF.Perform(ps.Next(9));
        //
        _entryPoint = 1;
        PerformInternal(pPF, ps.Next(1));
    }

    protected override void CheckData()
    {
        if (_arguments.IsEmpty() || (_arguments.Extent() + _tools.Extent()) < 2)
        {
            // too few arguments to process
            AddError(new AlertTooFewArguments());
            return;
        }
        //
        CheckFiller();
    }

    protected override void BuildResult(ShapeType type)
    {
        base.BuildResult(type);

        if (type == ShapeType.Compound)
        {
            // The method is called for the last time for this operation.
            // If there is only one argument shape and it has been modified into
            // a single shape, or has not been modified at all, the result shape
            // has to be overwritten to avoid the unnecessary enclosure into compound.
            if (_arguments.Extent() == 1)
            {
                Iterator it = new(_shape);
                if (it.More())
                {
                    Shape sFirst = it.Value();
                    it.Next();
                    if (!it.More())
                    {
                        _shape = sFirst;
                    }
                }
            }
        }
    }
}
