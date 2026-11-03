// Port of: src/ModelingAlgorithms/TKMesh/IMeshTools/IMeshTools_ModelBuilder.hxx
// Port of: src/ModelingAlgorithms/TKMesh/IMeshTools/IMeshTools_ModelBuilder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing.Model;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Foundation.Status;

namespace Kerf.Engine.Meshing.Tools;

/// <summary>A tool building the discrete model of a shape.</summary>
/// <remarks>
/// Statuses by default: <see cref="Done1"/>, the model is built; <see cref="Fail1"/>, an empty shape;
/// <see cref="Fail2"/>, the model wasn't built for an unexpected reason.
/// </remarks>
public abstract class ModelBuilder : Algorithm
{
    /// <summary>A builder.</summary>
    protected ModelBuilder()
    {
    }

    /// <summary>The discrete model of the shape, protected against exceptions; null on failure.</summary>
    public ModelModel? Perform(in Shape shape, in ToolParameters parameters)
    {
        ClearStatus();

        try
        {
            return PerformInternal(shape, parameters);
        }
        catch (Exception exception) when (ModelAlgo.IsCaughtFailure(exception))
        {
            SetStatus(Fail2);
            return null;
        }
    }

    /// <summary>The discrete model of the shape; null on failure.</summary>
    protected abstract ModelModel? PerformInternal(in Shape shape, in ToolParameters parameters);
}
