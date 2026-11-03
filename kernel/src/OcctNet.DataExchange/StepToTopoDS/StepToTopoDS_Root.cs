// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_Root.hxx
// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_Root.lxx
// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_Root.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;

namespace Occt;

/// <summary>The root of the STEP to topology translators: their state (done), precision and maximum tolerance.</summary>
public class StepToTopoDS_Root
{
    protected bool done;
    private double myPrecision;
    private double myMaxTol;

    protected StepToTopoDS_Root()
    {
        done = false;
        myPrecision = myMaxTol = Kerf.Engine.Foundation.Precision.Confusion();
    }

    public bool IsDone() => done;

    /// <summary>The precision of the translation.</summary>
    public double Precision() => myPrecision;

    public void SetPrecision(double preci) => myPrecision = preci;

    /// <summary>The maximum tolerance of the translation.</summary>
    public double MaxTol() => myMaxTol;

    public void SetMaxTol(double maxpreci) => myMaxTol = maxpreci;

    /// <summary>
    /// What OCCT's <c>catch (Standard_Failure)</c> under <c>OCC_CATCH_SIGNALS</c> catches: Standard_Failure and the .NET
    /// counterparts of system signals, never a write into a frozen object, a cancellation or a lack of memory
    /// (decisions 1183, 1201).
    /// </summary>
    internal static bool CaughtBySignals(Exception theException)
        => theException is not OutOfMemoryException and not OperationCanceledException and not ImmutableObjectException
                           and not LockedShapeException;
}
