// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Options.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Options.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2017-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;
using static Kerf.Engine.Foundation.Gravity;

namespace Kerf.Engine.Booleans;

/// <summary>
/// The options of the Boolean component's algorithms: error and warning reporting (alerts in a
/// <see cref="Report"/>), the parallel mode, the fuzzy tolerance and the use of oriented bounding boxes.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21). The allocator goes (translation rules, section 9), with OCCT's constructor taking
/// one. The class isn't abstract and has a public constructor, so that BRepAlgoAPI_Algo, which can't derive from both
/// BRepBuilderAPI_MakeShape and this class as OCCT does, holds one and forwards to it (decision 627). Its state is
/// <c>protected internal</c> for that reason.
/// </para>
/// <para>
/// The global parallel mode (<see cref="GetParallelMode"/>) is the current flow's (process-wide in OCCT), with an internal setter
/// (decisions 415 and 582): callers set the per-algorithm flag (<see cref="SetRunParallel"/>), and Draw-lite's
/// <c>brunparallel</c> the global one. <c>DumpErrors</c> and <c>DumpWarnings</c> write to a <see cref="TextWriter"/>.
/// The first construction loads BOPAlgo's message texts into <see cref="MsgFile"/>'s defaults, as OCCT does.
/// </para>
/// </remarks>
public class Options
{
    // OCCT's process-wide parallel mode: a setting of the operation context (unit 10I)
    internal static readonly ContextKey<bool> ParallelModeSetting = new("BOPAlgo_Options.ParallelMode", false);

    protected internal Report _report;
    protected internal bool _runParallel;
    protected internal double _fuzzyValue;
    protected internal bool _useOBB;

    /// <summary>Options with OCCT's defaults: the global parallel mode, the fuzzy value Precision::Confusion(), no OBB.</summary>
    public Options()
    {
        _report = new Report();
        _runParallel = ParallelModeSetting.Value;
        _fuzzyValue = global::Kerf.Engine.Foundation.Precision.Confusion();
        _useOBB = false;
        BOPAlgo_msg.BOPAlgo_LoadMessages();
    }

    /// <summary>Clears the warnings and errors and any data cached by the algorithm; the options stay.</summary>
    public virtual void Clear() => _report.Clear();

    //!@name Error reporting mechanism

    /// <summary>Adds the alert as an error (fail).</summary>
    public void AddError(Alert alert) => _report.AddAlert(Fail, alert);

    /// <summary>Adds the alert as a warning.</summary>
    public void AddWarning(Alert alert) => _report.AddAlert(Warning, alert);

    /// <summary>True if the algorithm has failed.</summary>
    public bool HasErrors => !_report.GetAlerts(Fail).IsEmpty();

    /// <summary>True if the algorithm has an error of exactly the type.</summary>
    public bool HasError(Type type) => _report.HasAlert(type, Fail);

    /// <summary>True if the algorithm has warnings.</summary>
    public bool HasWarnings => !_report.GetAlerts(Warning).IsEmpty();

    /// <summary>True if the algorithm has a warning of exactly the type.</summary>
    public bool HasWarning(Type type) => _report.HasAlert(type, Warning);

    /// <summary>The report collecting the errors and warnings.</summary>
    public Report GetReport() => _report;

    /// <summary>Writes the errors' message keys, one per line.</summary>
    public void DumpErrors(TextWriter os) => _report.Dump(os, Fail);

    /// <summary>Writes the warnings' message keys, one per line.</summary>
    public void DumpWarnings(TextWriter os) => _report.Dump(os, Warning);

    /// <summary>Clears the warnings.</summary>
    public void ClearWarnings() => _report.Clear(Warning);

    //!@name Parallel processing mode

    /// <summary>
    /// The parallel mode of the current flow of execution (false by default), the default of each new algorithm's flag
    /// (OCCT's global parallel mode).
    /// </summary>
    public static bool GetParallelMode() => ParallelModeSetting.Value;

    /// <summary>
    /// Sets the parallel mode for the rest of the current flow of execution (OCCT: for the process; Draw-lite and tests
    /// only).
    /// </summary>
    internal static void SetParallelMode(bool newMode) => OperationContext.Set(ParallelModeSetting, newMode);

    /// <summary>Switches the parallel processing of this algorithm on or off.</summary>
    public void SetRunParallel(bool flag) => _runParallel = flag;

    /// <summary>True if this algorithm runs in parallel.</summary>
    public bool RunParallel() => _runParallel;

    //!@name Fuzzy tolerance

    /// <summary>Sets the additional tolerance; at least Precision::Confusion().</summary>
    public void SetFuzzyValue(double fuzz) => _fuzzyValue = Max(fuzz, global::Kerf.Engine.Foundation.Precision.Confusion());

    /// <summary>The additional tolerance.</summary>
    public double FuzzyValue() => _fuzzyValue;

    //!@name Usage of Oriented Bounding boxes

    /// <summary>Enables or disables the oriented bounding boxes.</summary>
    public void SetUseOBB(bool useOBB) => _useOBB = useOBB;

    /// <summary>True if the oriented bounding boxes are used.</summary>
    public bool UseOBB() => _useOBB;

    /// <summary>Adds <see cref="AlertUserBreak"/> as an error if the user broke the operation; true then.</summary>
    protected internal bool UserBreak(ProgressScope ps)
    {
        if (ps.UserBreak())
        {
            AddError(new AlertUserBreak());
            return true;
        }
        return false;
    }
}
