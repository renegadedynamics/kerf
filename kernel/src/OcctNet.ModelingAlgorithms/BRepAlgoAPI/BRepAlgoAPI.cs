// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).


namespace Occt;

/// <summary>
/// The functional twins of the Boolean operations (decision 579): one call from arguments, tools and options to a
/// frozen result and its history, for callers that hold values (Bento's cells).
/// </summary>
/// <remarks>
/// <para>
/// Each twin freezes its arguments and tools first: they are shared with the result (decision 577), and a frozen
/// shape is locked, so OCCT's own check makes the operation non-destructive (BOPAlgo_PaveFiller_10). It then runs
/// OCCT's algorithm (<see cref="BRepAlgoAPI_Fuse"/> and the others) with the options, and returns the frozen result.
/// The history maps the caller's sub-shapes to the result's; nothing else holds it.
/// </para>
/// <para>
/// An operation that fails throws <see cref="BRepAlgoAPI_Failure"/>, which carries the algorithm's report.
/// <see cref="BRepAlgoAPI_Options.Simplify"/> throws <see cref="Standard_NotImplemented"/> until units 6U and 6P.
/// </para>
/// </remarks>
public static class BRepAlgoAPI
{
    /// <summary>The union of the arguments and the tools.</summary>
    public static TopoDS_Shape Fuse(IReadOnlyList<TopoDS_Shape> theArguments, IReadOnlyList<TopoDS_Shape> theTools,
                                    in BRepAlgoAPI_Options theOptions, out BRepTools_History theHistory) =>
        Run(new BRepAlgoAPI_Fuse(), theArguments, theTools, theOptions, out theHistory);

    /// <inheritdoc cref="Fuse(IReadOnlyList{TopoDS_Shape}, IReadOnlyList{TopoDS_Shape}, in BRepAlgoAPI_Options, out BRepTools_History)"/>
    public static TopoDS_Shape Fuse(IReadOnlyList<TopoDS_Shape> theArguments, IReadOnlyList<TopoDS_Shape> theTools,
                                    in BRepAlgoAPI_Options theOptions) =>
        Fuse(theArguments, theTools, theOptions, out _);

    /// <summary>The common part of the arguments and the tools.</summary>
    public static TopoDS_Shape Common(IReadOnlyList<TopoDS_Shape> theArguments, IReadOnlyList<TopoDS_Shape> theTools,
                                      in BRepAlgoAPI_Options theOptions, out BRepTools_History theHistory) =>
        Run(new BRepAlgoAPI_Common(), theArguments, theTools, theOptions, out theHistory);

    /// <inheritdoc cref="Common(IReadOnlyList{TopoDS_Shape}, IReadOnlyList{TopoDS_Shape}, in BRepAlgoAPI_Options, out BRepTools_History)"/>
    public static TopoDS_Shape Common(IReadOnlyList<TopoDS_Shape> theArguments, IReadOnlyList<TopoDS_Shape> theTools,
                                      in BRepAlgoAPI_Options theOptions) =>
        Common(theArguments, theTools, theOptions, out _);

    /// <summary>The arguments minus the tools.</summary>
    public static TopoDS_Shape Cut(IReadOnlyList<TopoDS_Shape> theArguments, IReadOnlyList<TopoDS_Shape> theTools,
                                   in BRepAlgoAPI_Options theOptions, out BRepTools_History theHistory) =>
        Run(new BRepAlgoAPI_Cut(), theArguments, theTools, theOptions, out theHistory);

    /// <inheritdoc cref="Cut(IReadOnlyList{TopoDS_Shape}, IReadOnlyList{TopoDS_Shape}, in BRepAlgoAPI_Options, out BRepTools_History)"/>
    public static TopoDS_Shape Cut(IReadOnlyList<TopoDS_Shape> theArguments, IReadOnlyList<TopoDS_Shape> theTools,
                                   in BRepAlgoAPI_Options theOptions) =>
        Cut(theArguments, theTools, theOptions, out _);

    /// <summary>
    /// The section of the arguments and the tools: the edges and vertices where they meet, with the section options
    /// (approximation, p-curves on the arguments' or the tools' faces).
    /// </summary>
    public static TopoDS_Shape Section(IReadOnlyList<TopoDS_Shape> theArguments, IReadOnlyList<TopoDS_Shape> theTools,
                                       in BRepAlgoAPI_Options theOptions, in BOPAlgo_SectionAttribute theSectionOptions,
                                       out BRepTools_History theHistory)
    {
        BRepAlgoAPI_Section anAlgo = new();
        anAlgo.Approximation(theSectionOptions.Approximation());
        anAlgo.ComputePCurveOn1(theSectionOptions.PCurveOnS1());
        anAlgo.ComputePCurveOn2(theSectionOptions.PCurveOnS2());
        return Run(anAlgo, theArguments, theTools, theOptions, out theHistory);
    }

    /// <inheritdoc cref="Section(IReadOnlyList{TopoDS_Shape}, IReadOnlyList{TopoDS_Shape}, in BRepAlgoAPI_Options, in BOPAlgo_SectionAttribute, out BRepTools_History)"/>
    public static TopoDS_Shape Section(IReadOnlyList<TopoDS_Shape> theArguments, IReadOnlyList<TopoDS_Shape> theTools,
                                       in BRepAlgoAPI_Options theOptions, in BOPAlgo_SectionAttribute theSectionOptions) =>
        Section(theArguments, theTools, theOptions, theSectionOptions, out _);

    /// <summary>The arguments split by the tools (the tools' splits left out).</summary>
    public static TopoDS_Shape Split(IReadOnlyList<TopoDS_Shape> theArguments, IReadOnlyList<TopoDS_Shape> theTools,
                                     in BRepAlgoAPI_Options theOptions, out BRepTools_History theHistory)
    {
        BRepAlgoAPI_Splitter anAlgo = new();
        anAlgo.SetTools(Published(theTools));
        return Run(anAlgo, theArguments, theOptions, out theHistory);
    }

    /// <inheritdoc cref="Split(IReadOnlyList{TopoDS_Shape}, IReadOnlyList{TopoDS_Shape}, in BRepAlgoAPI_Options, out BRepTools_History)"/>
    public static TopoDS_Shape Split(IReadOnlyList<TopoDS_Shape> theArguments, IReadOnlyList<TopoDS_Shape> theTools,
                                     in BRepAlgoAPI_Options theOptions) =>
        Split(theArguments, theTools, theOptions, out _);

    // A Boolean operation: the arguments and tools set, then Run.
    private static TopoDS_Shape Run(BRepAlgoAPI_BooleanOperation theAlgo, IReadOnlyList<TopoDS_Shape> theArguments,
                                    IReadOnlyList<TopoDS_Shape> theTools, in BRepAlgoAPI_Options theOptions,
                                    out BRepTools_History theHistory)
    {
        theAlgo.SetTools(Published(theTools));
        return Run(theAlgo, theArguments, theOptions, out theHistory);
    }

    private static TopoDS_Shape Run(BRepAlgoAPI_BuilderAlgo theAlgo, IReadOnlyList<TopoDS_Shape> theArguments,
                                    in BRepAlgoAPI_Options theOptions, out BRepTools_History theHistory)
    {
        if (theOptions.Simplify)
        {
            throw new Standard_NotImplemented(
                "BRepAlgoAPI_Options.Simplify: ShapeUpgrade_UnifySameDomain isn't ported yet (units 6U and 6P)");
        }
        theAlgo.SetArguments(Published(theArguments));
        theAlgo.SetFuzzyValue(theOptions.FuzzyValue);
        theAlgo.SetGlue(theOptions.Glue);
        theAlgo.SetUseOBB(theOptions.UseOBB);
        theAlgo.SetCheckInverted(theOptions.CheckInverted);
        theAlgo.SetRunParallel(theOptions.RunParallel);
        theAlgo.SetNonDestructive(theOptions.NonDestructive);
        theAlgo.SetToFillHistory(true);
        theAlgo.Build();
        if (theAlgo.HasErrors() || !theAlgo.IsDone())
        {
            throw new BRepAlgoAPI_Failure(theAlgo.GetReport());
        }
        TopoDS_Shape aResult = theAlgo.Shape();
        theHistory = theAlgo.History() ?? new BRepTools_History();
        return aResult;
    }

    // The shapes as a list, each graph frozen (published: the result shares them).
    private static NCollection_List<TopoDS_Shape> Published(IReadOnlyList<TopoDS_Shape> theShapes)
    {
        NCollection_List<TopoDS_Shape> aList = new();
        for (int i = 0; i < theShapes.Count; ++i)
        {
            TopoDS_Shape aS = theShapes[i];
            aS.TShape()?.FreezeGraph();
            aList.Append(aS);
        }
        return aList;
    }
}

/// <summary>A Boolean operation of <see cref="BRepAlgoAPI"/> failed; <see cref="Report"/> holds its errors and warnings.</summary>
public sealed class BRepAlgoAPI_Failure : StdFail_NotDone
{
    private readonly Message_Report myReport;

    /// <summary>The failure of an operation with this report; the message lists the errors' keys.</summary>
    public BRepAlgoAPI_Failure(Message_Report theReport)
        : base(ErrorText(theReport))
    {
        myReport = theReport;
    }

    /// <summary>The report of the failed operation (its own: nothing else holds it).</summary>
    public Message_Report Report() => myReport;

    private static string ErrorText(Message_Report theReport)
    {
        using StringWriter aText = new();
        theReport.Dump(aText, Message_Gravity.Message_Fail);
        string anErrors = aText.ToString().Trim().Replace('\n', ' ');
        return "The Boolean operation failed: " + (anErrors.Length == 0 ? "not done" : anErrors);
    }
}
