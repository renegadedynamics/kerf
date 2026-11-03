// Port of: src/ModelingAlgorithms/TKMesh/IMeshTools/IMeshTools_ModelAlgo.hxx
// Port of: src/ModelingAlgorithms/TKMesh/IMeshTools/IMeshTools_ModelAlgo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>An algorithm that updates or modifies a discrete model.</summary>
public abstract class IMeshTools_ModelAlgo : Transient
{
    /// <summary>An algorithm.</summary>
    protected IMeshTools_ModelAlgo()
    {
    }

    /// <summary>Processes the model; false if processing failed with an exception.</summary>
    public bool Perform(IMeshData_Model? theModel, in IMeshTools_Parameters theParameters, ProgressRange theRange)
    {
        try
        {
            return performInternal(theModel, theParameters, theRange);
        }
        catch (Exception anException) when (IsCaughtFailure(anException))
        {
            return false;
        }
    }

    /// <summary>Processes the model.</summary>
    protected abstract bool performInternal(IMeshData_Model? theModel, in IMeshTools_Parameters theParameters,
                                            ProgressRange theRange);

    /// <summary>
    /// The exceptions OCCT's <c>catch (Standard_Failure const&amp;)</c> handles under <c>OCC_CATCH_SIGNALS</c> in the
    /// mesher (as BRepCheck_Analyzer's): Standard_Failure, and the .NET forms of the access violations and integer
    /// divisions by zero that OCC_CATCH_SIGNALS turns into failures. A write into a frozen object
    /// (<see cref="ImmutableObjectException"/>, <see cref="TopoDS_LockedShape"/>) is let through, so a write into a
    /// shared input can't pass silently (decision 423's rule).
    /// </summary>
    internal static bool IsCaughtFailure(Exception theException)
        => theException is not (ImmutableObjectException or TopoDS_LockedShape)
           && theException is FailureException
               or NullReferenceException
               or InvalidCastException
               or IndexOutOfRangeException
               or ArgumentOutOfRangeException
               or DivideByZeroException;
}
