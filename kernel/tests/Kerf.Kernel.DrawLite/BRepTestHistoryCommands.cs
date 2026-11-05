// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using System.Runtime.CompilerServices;
using Occt;

namespace Kerf.Kernel.DrawLite;

/// <summary>
/// BRepTest_HistoryCommands.cxx (<c>setfillhistory</c>, <c>savehistory</c>, <c>modified</c>, <c>generated</c>,
/// <c>isdeleted</c>), with BRepTest_Objects' session (the last operation's history) and BRepTest_DrawableHistory.
/// </summary>
/// <remarks>
/// The session is the interpreter's, as <see cref="BOPTestObjects"/>. The commands that record a history
/// (<see cref="SetHistory"/>, <see cref="AddHistory"/>) are those of the ported algorithms: unit 6G's builders so far;
/// unit 6L adds the Boolean and BRepTest ones.
/// </remarks>
internal static class BRepTestHistoryCommands
{
    private sealed class Session
    {
        public History? History;
        public bool FillHistory = true;
    }

    private static readonly ConditionalWeakTable<DrawInterpreter, Session> theSessions = new();

    private static Session Of(DrawInterpreter theDI) => theSessions.GetValue(theDI, static _ => new Session());

    /// <summary>BRepTest_Objects::SetHistory: the session's history becomes the given one.</summary>
    public static void SetHistory(DrawInterpreter theDI, History? theHistory) => Of(theDI).History = theHistory;

    /// <summary>BRepTest_Objects::SetHistory(arguments, algorithm).</summary>
    public static void SetHistory(DrawInterpreter theDI, NCollection_List<Shape> theArguments, global::Kerf.Engine.Topology.History.IAlgorithm theAlgo)
        => SetHistory(theDI, new History(theArguments, theAlgo));

    /// <summary>BRepTest_Objects::AddHistory: merges the given history into the session's.</summary>
    public static void AddHistory(DrawInterpreter theDI, History? theHistory)
    {
        Session aSession = Of(theDI);
        aSession.History ??= new History();
        aSession.History.Merge(theHistory);
    }

    /// <summary>BRepTest_Objects::History.</summary>
    public static History? History(DrawInterpreter theDI) => Of(theDI).History;

    /// <summary>BRepTest_Objects::IsHistoryNeeded (<c>setfillhistory</c>).</summary>
    public static bool IsHistoryNeeded(DrawInterpreter theDI) => Of(theDI).FillHistory;

    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "setfillhistory", SetFillHistory);
        DrawCxx.Add(theInterpreter, "savehistory", SaveHistory);
        DrawCxx.Add(theInterpreter, "modified", Modified);
        DrawCxx.Add(theInterpreter, "generated", Generated);
        DrawCxx.Add(theInterpreter, "isdeleted", IsDeleted);
    }

    private static int SetFillHistory(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count > 2)
        {
            return 1;
        }
        if (a.Count == 1)
        {
            di.Append("Filling of the history is ").Append(IsHistoryNeeded(theDI) ? "enabled." : "disabled.");
        }
        else
        {
            Of(theDI).FillHistory = theDI.Atoi(a[1]) != 0;
        }
        return 0;
    }

    private static int SaveHistory(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 2)
        {
            return 1;
        }
        // Get the history from the session
        History? aHistory = History(theDI);
        if (aHistory is null)
        {
            di.Append("No history has been prepared yet.");
            return 1;
        }
        theDI.SetDrawable(a[1], new DrawHistory(aHistory));
        return 0;
    }

    private static History? GetHistory(DrawInterpreter theDI, DrawResult di, string theName)
    {
        if (theDI.GetDrawable(theName) is not DrawHistory aHistory)
        {
            di.Append("History with the name ").Append(theName).Append(" does not exist.");
            return null;
        }
        return aHistory.History;
    }

    private static Shape GetShape(DrawInterpreter theDI, DrawResult di, string theName)
    {
        Shape aS = theDI.GetShape(theName);
        if (aS.IsNull)
        {
            di.Append(theName).Append(" is a null shape.");
            return new Shape();
        }
        if (!global::Kerf.Engine.Topology.History.IsSupportedType(aS))
        {
            di.Append("History is not supported for this kind of shape.");
            return new Shape();
        }
        return aS;
    }

    private static Shape MakeCompound(IReadOnlyList<Shape> theLS)
    {
        if (theLS.Count == 1)
        {
            return theLS[0];
        }
        BrepBuilder aBB = new();
        Compound aC = new();
        aBB.MakeCompound(ref aC);
        foreach (Shape aS in theLS)
        {
            aBB.Add(aC, aS);
        }
        return aC;
    }

    private static int Modified(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 4)
        {
            return 1;
        }
        History? aHistory = GetHistory(theDI, di, a[2]);
        if (aHistory is null)
        {
            return 1;
        }
        Shape aS = GetShape(theDI, di, a[3]);
        if (aS.IsNull)
        {
            return 1;
        }
        List<Shape> aModified = [.. aHistory.Modified(aS)];
        if (aModified.Count == 0)
        {
            di.Append("The shape has not been modified.");
            return 0;
        }
        theDI.SetShape(a[1], MakeCompound(aModified));
        return 0;
    }

    private static int Generated(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 4)
        {
            return 1;
        }
        History? aHistory = GetHistory(theDI, di, a[2]);
        if (aHistory is null)
        {
            return 1;
        }
        Shape aS = GetShape(theDI, di, a[3]);
        if (aS.IsNull)
        {
            return 1;
        }
        List<Shape> aGenerated = [.. aHistory.Generated(aS)];
        if (aGenerated.Count == 0)
        {
            di.Append("No shapes were generated from the shape.");
            return 0;
        }
        theDI.SetShape(a[1], MakeCompound(aGenerated));
        return 0;
    }

    private static int IsDeleted(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 3)
        {
            return 1;
        }
        History? aHistory = GetHistory(theDI, di, a[1]);
        if (aHistory is null)
        {
            return 1;
        }
        Shape aS = GetShape(theDI, di, a[2]);
        if (aS.IsNull)
        {
            return 1;
        }
        di.Append(aHistory.IsRemoved(aS) ? "Deleted." : "Not deleted.");
        return 0;
    }
}

/// <summary>A history (BRepTest_DrawableHistory), as <c>savehistory</c> stores it.</summary>
internal sealed class DrawHistory(History theHistory) : DrawDrawable
{
    /// <summary>The history.</summary>
    public History History { get; } = theHistory;

    public override string TypeName => "BRepTest_DrawableHistory";

    public override void Whatis(DrawResult theResult) => theResult.Append("history");

    public override void Dump(TextWriter theStream) => History.Dump(theStream);

    /// <summary>Draw can't save a history (BRepTest_DrawableHistory has no Save); nothing is written.</summary>
    public override void Save(TextWriter theStream)
    {
    }

    public override DrawDrawable Copy() => new DrawHistory(History);
}
