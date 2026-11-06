// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_Root.hxx
// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_Root.lxx
// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_Root.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Kerf.Engine.Exchange.Step.Import;

/// <summary>The root of the STEP to topology translators: their state (done), precision and maximum tolerance.</summary>
internal class ImportRoot
{
    protected bool _done;
    private double _precision;
    private double _maxTol;

    protected ImportRoot()
    {
        _done = false;
        _precision = _maxTol = Kerf.Engine.Foundation.Precision.Confusion();
    }

    public bool IsDone => _done;

    /// <summary>The precision of the translation.</summary>
    public double Precision() => _precision;

    public void SetPrecision(double preci) => _precision = preci;

    /// <summary>The maximum tolerance of the translation.</summary>
    public double MaxTol() => _maxTol;

    public void SetMaxTol(double maxpreci) => _maxTol = maxpreci;

    /// <summary>
    /// What OCCT's <c>catch (Standard_Failure)</c> under <c>OCC_CATCH_SIGNALS</c> catches: Standard_Failure and the .NET
    /// counterparts of system signals, never a write into a frozen object, a cancellation or a lack of memory
    /// (decisions 1183, 1201).
    /// </summary>
    internal static bool CaughtBySignals(Exception exception)
        => exception is not OutOfMemoryException and not OperationCanceledException and not ImmutableObjectException;
}
