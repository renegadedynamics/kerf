// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Options.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Options.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2017-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Message_Gravity;

namespace Occt;

/// <summary>
/// The options of the Boolean component's algorithms: error and warning reporting (alerts in a
/// <see cref="Message_Report"/>), the parallel mode, the fuzzy tolerance and the use of oriented bounding boxes.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21). The allocator goes (translation rules, section 9), with OCCT's constructor taking
/// one. The class isn't abstract and has a public constructor, so that BRepAlgoAPI_Algo, which can't derive from both
/// BRepBuilderAPI_MakeShape and this class as OCCT does, holds one and forwards to it (decision 6116). Its state is
/// <c>protected internal</c> for that reason.
/// </para>
/// <para>
/// The global parallel mode (<see cref="GetParallelMode"/>) is process-wide, as in OCCT, with an internal setter
/// (decisions 415 and 5007): callers set the per-algorithm flag (<see cref="SetRunParallel"/>), and Draw-lite's
/// <c>brunparallel</c> the global one. <c>DumpErrors</c> and <c>DumpWarnings</c> write to a <see cref="TextWriter"/>.
/// The first construction loads BOPAlgo's message texts into <see cref="Message_MsgFile"/>, as OCCT does.
/// </para>
/// </remarks>
public class BOPAlgo_Options
{
    private static volatile bool myGlobalRunParallel = false;

    protected internal Message_Report myReport;
    protected internal bool myRunParallel;
    protected internal double myFuzzyValue;
    protected internal bool myUseOBB;

    /// <summary>Options with OCCT's defaults: the global parallel mode, the fuzzy value Precision::Confusion(), no OBB.</summary>
    public BOPAlgo_Options()
    {
        myReport = new Message_Report();
        myRunParallel = myGlobalRunParallel;
        myFuzzyValue = Precision.Confusion();
        myUseOBB = false;
        BOPAlgo_BOPAlgo_msg.BOPAlgo_LoadMessages();
    }

    /// <summary>Clears the warnings and errors and any data cached by the algorithm; the options stay.</summary>
    public virtual void Clear() => myReport.Clear();

    //!@name Error reporting mechanism

    /// <summary>Adds the alert as an error (fail).</summary>
    public void AddError(Message_Alert theAlert) => myReport.AddAlert(Message_Fail, theAlert);

    /// <summary>Adds the alert as a warning.</summary>
    public void AddWarning(Message_Alert theAlert) => myReport.AddAlert(Message_Warning, theAlert);

    /// <summary>True if the algorithm has failed.</summary>
    public bool HasErrors() => !myReport.GetAlerts(Message_Fail).IsEmpty();

    /// <summary>True if the algorithm has an error of exactly the type.</summary>
    public bool HasError(Type theType) => myReport.HasAlert(theType, Message_Fail);

    /// <summary>True if the algorithm has warnings.</summary>
    public bool HasWarnings() => !myReport.GetAlerts(Message_Warning).IsEmpty();

    /// <summary>True if the algorithm has a warning of exactly the type.</summary>
    public bool HasWarning(Type theType) => myReport.HasAlert(theType, Message_Warning);

    /// <summary>The report collecting the errors and warnings.</summary>
    public Message_Report GetReport() => myReport;

    /// <summary>Writes the errors' message keys, one per line.</summary>
    public void DumpErrors(TextWriter theOS) => myReport.Dump(theOS, Message_Fail);

    /// <summary>Writes the warnings' message keys, one per line.</summary>
    public void DumpWarnings(TextWriter theOS) => myReport.Dump(theOS, Message_Warning);

    /// <summary>Clears the warnings.</summary>
    public void ClearWarnings() => myReport.Clear(Message_Warning);

    //!@name Parallel processing mode

    /// <summary>The global parallel mode (false by default), the default of each new algorithm's flag.</summary>
    public static bool GetParallelMode() => myGlobalRunParallel;

    /// <summary>Sets the global parallel mode (process-wide: Draw-lite and tests only).</summary>
    internal static void SetParallelMode(bool theNewMode) => myGlobalRunParallel = theNewMode;

    /// <summary>Switches the parallel processing of this algorithm on or off.</summary>
    public void SetRunParallel(bool theFlag) => myRunParallel = theFlag;

    /// <summary>True if this algorithm runs in parallel.</summary>
    public bool RunParallel() => myRunParallel;

    //!@name Fuzzy tolerance

    /// <summary>Sets the additional tolerance; at least Precision::Confusion().</summary>
    public void SetFuzzyValue(double theFuzz) => myFuzzyValue = Max(theFuzz, Precision.Confusion());

    /// <summary>The additional tolerance.</summary>
    public double FuzzyValue() => myFuzzyValue;

    //!@name Usage of Oriented Bounding boxes

    /// <summary>Enables or disables the oriented bounding boxes.</summary>
    public void SetUseOBB(bool theUseOBB) => myUseOBB = theUseOBB;

    /// <summary>True if the oriented bounding boxes are used.</summary>
    public bool UseOBB() => myUseOBB;

    /// <summary>Adds <see cref="BOPAlgo_AlertUserBreak"/> as an error if the user broke the operation; true then.</summary>
    protected internal bool UserBreak(Message_ProgressScope thePS)
    {
        if (thePS.UserBreak())
        {
            AddError(new BOPAlgo_AlertUserBreak());
            return true;
        }
        return false;
    }
}
