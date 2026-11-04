// Port of: src/ModelingAlgorithms/TKMesh/IMeshTools/IMeshTools_ModelAlgo.hxx
// Port of: src/ModelingAlgorithms/TKMesh/IMeshTools/IMeshTools_ModelAlgo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing.Model;
using Kerf.Engine.Topology;
using Occt;

namespace Kerf.Engine.Meshing.Tools;

/// <summary>An algorithm that updates or modifies a discrete model.</summary>
public abstract class ModelAlgo : Transient
{
    /// <summary>An algorithm.</summary>
    protected ModelAlgo()
    {
    }

    /// <summary>Processes the model; false if processing failed with an exception.</summary>
    public bool Perform(ModelModel? model, in ToolParameters parameters, ProgressRange range)
    {
        try
        {
            return PerformInternal(model, parameters, range);
        }
        catch (Exception exception) when (IsCaughtFailure(exception))
        {
            return false;
        }
    }

    /// <summary>Processes the model.</summary>
    protected abstract bool PerformInternal(ModelModel? model, in ToolParameters parameters,
                                            ProgressRange range);

    /// <summary>
    /// The exceptions OCCT's <c>catch (Standard_Failure const&amp;)</c> handles under <c>OCC_CATCH_SIGNALS</c> in the
    /// mesher (as BRepCheck_Analyzer's): Standard_Failure, and the .NET forms of the access violations and integer
    /// divisions by zero that OCC_CATCH_SIGNALS turns into failures. A write into a frozen object
    /// (<see cref="ImmutableObjectException"/>, <see cref="LockedShapeException"/>) is let through, so a write into a
    /// shared input can't pass silently (decision 423's rule).
    /// </summary>
    internal static bool IsCaughtFailure(Exception exception)
        => exception is not (ImmutableObjectException or LockedShapeException)
           && (Refusal.IsFailure(exception) || exception is NullReferenceException
               or InvalidCastException
               or IndexOutOfRangeException
               or ArgumentOutOfRangeException
               or DivideByZeroException);
}
