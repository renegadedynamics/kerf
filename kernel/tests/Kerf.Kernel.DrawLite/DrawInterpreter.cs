// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Collections.Concurrent;
using System.Runtime.CompilerServices;
using System.Runtime.ExceptionServices;
using System.Text;

namespace Kerf.Kernel.DrawLite;

/// <summary>An error in a script, or from a command (Tcl's TCL_ERROR), with the line it happened on.</summary>
public class DrawException(string message, int line = 0) : Exception(message)
{
    /// <summary>The error message alone, which is what <c>catch</c> gives a script.</summary>
    public string Reason { get; } = message;

    /// <summary>The line of the innermost command that failed, when known.</summary>
    public int Line { get; internal set; } = line;

    /// <summary>The sourced file the error happened in (with <see cref="Line"/>), when known.</summary>
    public string? File { get; internal set; }

    /// <summary>Tcl's errorCode.</summary>
    public string ErrorCode { get; init; } = "NONE";

    public override string Message => Line > 0 ? $"line {Line}: {Reason}" : Reason;
}

/// <summary>A command the interpreter doesn't know. A corpus run counts these to show what to implement next.</summary>
public sealed class DrawCommandNotFoundException(string command, int line) : DrawException($"invalid command name \"{command}\"", line)
{
    public string Command { get; } = command;
}

/// <summary>The <c>exit</c> command: the script stops.</summary>
public sealed class DrawExitException(int code) : Exception($"exit {code}")
{
    public int Code { get; } = code;
}

/// <summary>A script ran past <see cref="DrawInterpreter.Deadline"/>.</summary>
public sealed class DrawTimeoutException() : Exception("time limit exceeded");

/// <summary>Tcl's return codes other than ok and error: break, continue and return unwind as these.</summary>
internal abstract class TclFlowException(string message) : Exception(message);

internal sealed class TclBreakException() : TclFlowException("invoked \"break\" outside of a loop")
{
    public static readonly TclBreakException Instance = new();
}

internal sealed class TclContinueException() : TclFlowException("invoked \"continue\" outside of a loop")
{
    public static readonly TclContinueException Instance = new();
}

/// <summary><c>return</c>: the value, and the code the procedure returns with (0 for a plain return).</summary>
internal sealed class TclReturnException(int code, string value) : TclFlowException("return")
{
    public int Code { get; } = code;

    public string Value { get; } = value;

    public string ErrorCode { get; init; } = "NONE";
}

/// <summary>A Draw command: the interpreter and the command's arguments (after substitution) in, its result out.</summary>
public delegate string DrawCommand(DrawInterpreter theInterpreter, IReadOnlyList<string> theArgs);

/// <summary>A variable: a scalar value, an array, or neither (declared by global or upvar but not yet set).</summary>
internal sealed class TclVar
{
    public string? Value;
    public Dictionary<string, string>? Array;

    /// <summary>The Draw variable held by this Tcl variable (Draw::Set), cleared when the variable is written or unset.</summary>
    public DrawDrawable? Drawable;

    /// <summary>The Draw variables held by elements of this array.</summary>
    public Dictionary<string, DrawDrawable>? ElementDrawables;

    public bool Exists => Value is not null || Array is not null;
}

/// <summary>A procedure call's variables. Level 0 is the global frame.</summary>
internal sealed class TclFrame(int theLevel, TclFrame? theCaller, IReadOnlyList<string>? theCommand)
{
    public int Level { get; } = theLevel;

    public TclFrame? Caller { get; } = theCaller;

    /// <summary>The words of the procedure call that made this frame (info level).</summary>
    public IReadOnlyList<string>? Command { get; } = theCommand;

    public Dictionary<string, TclVar> Vars { get; } = new(StringComparer.Ordinal);
}

/// <summary>A procedure: its parameters (with defaults) and body.</summary>
internal sealed record TclProc(string Name, IReadOnlyList<(string Name, string? Default)> Parameters, string Body);

/// <summary>
/// A small Tcl interpreter for OCCT's Draw test scripts, so the port can run OCCT's own test corpus (tests/ in
/// the OCCT source) and the Tcl procedures OCCT's harness defines (resources/DrawResources). It follows Tcl 8.6:
/// substitution, procedures with their own variables, upvar/uplevel, the control commands, expr, lists,
/// strings, arrays and dicts. Modelling commands (box, bcut, checkshape, ...) are registered by name as the
/// packages they need are ported.
/// </summary>
public sealed partial class DrawInterpreter
{
    // Tcl's limit of nested procedure calls (interp recursionlimit), and a limit of nested scripts for runaway
    // recursion without procedures. Each level costs several .NET frames, so scripts run on a thread with a
    // large stack (see Eval).
    private const int MaxProcDepth = 1000;
    private const int MaxNesting = 10000;
    private const int StackSize = 256 * 1024 * 1024;

    [ThreadStatic]
    private static bool t_hasLargeStack;
    private static readonly ConcurrentDictionary<string, IReadOnlyList<Command>> s_scripts = new(StringComparer.Ordinal);

    private readonly Dictionary<string, DrawCommand> myCommands = new(StringComparer.Ordinal);
    private readonly Dictionary<string, TclProc> myProcs = new(StringComparer.Ordinal);
    private readonly TclFrame myGlobal = new(0, null, null);
    private TclFrame myFrame;
    private int myNesting;
    private int myProcDepth;

    /// <summary>Where <c>puts</c> writes (stdout and stderr both).</summary>
    public TextWriter Output { get; }

    /// <summary>The file being sourced (info script).</summary>
    public string ScriptFile { get; internal set; } = "";

    /// <summary>True once the script called <c>exit</c>.</summary>
    public bool HasExited { get; private set; }

    /// <summary>
    /// The interpreter's operation context: the settings its commands made (<c>mpparallel</c>, <c>brunparallel</c>,
    /// <c>mpsetdefaultname</c>…, OCCT's process-wide statics), carried from one <see cref="Eval"/> to the next as a
    /// DRAWEXE session keeps them, and seen by no other interpreter (unit 10I). It starts as the context of the flow
    /// that made the interpreter.
    /// </summary>
    internal Kerf.Engine.Foundation.OperationContext Context { get; private set; } = Kerf.Engine.Foundation.OperationContext.Current;

    public int ExitCode { get; private set; }

    /// <summary>
    /// When set, a script still running at this time (UTC) stops with <see cref="DrawTimeoutException"/>, which
    /// scripts can't catch. Draw's cpulimit does the same for OCCT's harness.
    /// </summary>
    public DateTime? Deadline { get; set; }

    public DrawInterpreter(TextWriter? theOutput = null)
    {
        Output = theOutput ?? TextWriter.Null;
        myFrame = myGlobal;
        DrawBuiltins.Register(this);
    }

    /// <summary>Files: read from disk, written to an in-memory overlay.</summary>
    internal TclFileSystem Files { get; } = new();

    /// <summary>
    /// The scripts OCCT's DRAWEXE loads at start (DrawDefault): StandardCommands.tcl, Geometry.tcl,
    /// TestCommands.tcl, CheckCommands.tcl and Vector.tcl from resources/DrawResources, which define the test
    /// harness procedures (checkprops, checknbshapes, locate_data_file, ...). The viewer commands stay no-ops.
    /// </summary>
    public void LoadDrawResources(string theResourcesDirectory)
    {
        foreach (string aScript in DrawResourceScripts)
        {
            string aPath = Path.Combine(theResourcesDirectory, aScript);
            if (File.Exists(aPath))
            {
                Source(TclFileSystem.Slashes(aPath));
            }
        }

        DrawBuiltins.RegisterIgnored(this);
    }

    /// <summary>The scripts of <see cref="LoadDrawResources"/>, in DrawDefault's order (StandardViews.tcl needs Tk).</summary>
    public static IReadOnlyList<string> DrawResourceScripts { get; } =
        ["StandardCommands.tcl", "Geometry.tcl", "TestCommands.tcl", "CheckCommands.tcl", "Vector.tcl"];

    public IReadOnlyCollection<string> CommandNames => myCommands.Keys;

    /// <summary>The procedures defined so far (by <c>proc</c>), by name.</summary>
    public IReadOnlyCollection<string> ProcNames => myProcs.Keys;

    public void Register(string theName, DrawCommand theCommand)
    {
        string aName = CommandKey(theName);
        myCommands[aName] = theCommand;
        myProcs.Remove(aName);
    }

    public bool HasCommand(string theName) => myCommands.ContainsKey(CommandKey(theName));

    /// <summary>A procedure's body, or null when the name isn't a procedure.</summary>
    public string? ProcBody(string theName) => myProcs.TryGetValue(CommandKey(theName), out TclProc? aProc) ? aProc.Body : null;

    internal bool TryGetProc(string theName, out TclProc theProc) => myProcs.TryGetValue(CommandKey(theName), out theProc!);

    internal void DefineProc(TclProc theProc)
    {
        string aName = CommandKey(theProc.Name);
        myCommands[aName] = (theInterpreter, theArgs) => theInterpreter.CallProc(theProc, theArgs);
        myProcs[aName] = theProc;
    }

    internal void RenameCommand(string theOld, string theNew)
    {
        string anOld = CommandKey(theOld);
        if (!myCommands.Remove(anOld, out DrawCommand? aCommand))
        {
            throw new DrawException($"can't {(theNew.Length == 0 ? "delete" : "rename")} \"{theOld}\": command doesn't exist");
        }

        myProcs.Remove(anOld, out TclProc? aProc);
        if (theNew.Length == 0)
        {
            return;
        }

        string aNew = CommandKey(theNew);
        if (myCommands.ContainsKey(aNew))
        {
            myCommands[anOld] = aCommand;
            if (aProc is not null)
            {
                myProcs[anOld] = aProc;
            }

            throw new DrawException($"can't rename to \"{theNew}\": command already exists");
        }

        myCommands[aNew] = aCommand;
        if (aProc is not null)
        {
            myProcs[aNew] = aProc;
        }
    }

    private static string CommandKey(string theName) => theName.StartsWith("::", StringComparison.Ordinal) ? theName.TrimStart(':') : theName;

    // -------------------------------------------------------------------------------------------------------------
    // Evaluation

    /// <summary>
    /// Runs a script at the top level and returns the result of its last command. A <c>return</c> ends the
    /// script with its value; <c>exit</c> ends it and sets <see cref="HasExited"/>.
    /// </summary>
    public string Eval(string theScript)
    {
        if (t_hasLargeStack)
        {
            return EvalTopLevel(theScript);
        }

        // A thread with room for Tcl's 1000 levels of nesting (and for unwinding an error through them)
        string aResult = "";
        ExceptionDispatchInfo? anError = null;
        var aThread = new Thread(() =>
        {
            t_hasLargeStack = true;
            using Kerf.Engine.Foundation.OperationContext.Scope aScope = Kerf.Engine.Foundation.OperationContext.Begin(Context);
            try
            {
                aResult = EvalTopLevel(theScript);
            }
            catch (Exception anException)
            {
                anError = ExceptionDispatchInfo.Capture(anException);
            }
            finally
            {
                Context = Kerf.Engine.Foundation.OperationContext.Current;
            }
        }, StackSize);
        aThread.Start();
        aThread.Join();
        anError?.Throw();
        return aResult;
    }

    private string EvalTopLevel(string theScript)
    {
        try
        {
            return EvalScript(theScript);
        }
        catch (TclReturnException aReturn)
        {
            return aReturn.Code == 1 ? throw new DrawException(aReturn.Value) { ErrorCode = aReturn.ErrorCode } : aReturn.Value;
        }
        catch (TclFlowException aFlow)
        {
            throw new DrawException(aFlow.Message);
        }
        catch (DrawExitException anExit)
        {
            HasExited = true;
            ExitCode = anExit.Code;
            return "";
        }
    }

    /// <summary>Reads a file and runs it at the top level, as <c>source</c> does.</summary>
    public string Source(string thePath) => Eval($"source {TclList.Quote(thePath)}");

    /// <summary>Runs a script in the current frame; break, continue and return propagate as exceptions.</summary>
    internal string EvalScript(string theScript)
    {
        if (!s_scripts.TryGetValue(theScript, out IReadOnlyList<Command>? aCommands))
        {
            aCommands = TclParser.Parse(theScript);
            if (s_scripts.Count > 20000)
            {
                s_scripts.Clear();
            }

            s_scripts[theScript] = aCommands;
        }

        if (Deadline is { } aDeadline && DateTime.UtcNow > aDeadline)
        {
            throw new DrawTimeoutException();
        }

        if (myNesting >= MaxNesting || !RuntimeHelpers.TryEnsureSufficientExecutionStack())
        {
            throw new DrawException("too many nested evaluations (infinite loop?)");
        }

        myNesting++;
        try
        {
            string aResult = "";
            foreach (Command aCommand in aCommands)
            {
                aResult = EvalCommand(aCommand);
            }

            return aResult;
        }
        finally
        {
            myNesting--;
        }
    }

    private string EvalCommand(Command theCommand)
    {
        var anArgs = new List<string>(theCommand.Words.Count);
        try
        {
            foreach (Word aWord in theCommand.Words)
            {
                if (aWord.IsExpanded)
                {
                    anArgs.AddRange(TclList.Parse(Substitute(aWord)));
                }
                else
                {
                    anArgs.Add(Substitute(aWord));
                }
            }
        }
        catch (DrawException anError) when (TagLine(anError, theCommand.Line))
        {
            throw;
        }

        return anArgs.Count == 0 ? "" : Invoke(anArgs, theCommand.Line);
    }

    /// <summary>Calls a command with its words already substituted.</summary>
    public string Invoke(IReadOnlyList<string> theArgs, int theLine = 0)
    {
        if (!myCommands.TryGetValue(theArgs[0], out DrawCommand? aHandler) && !myCommands.TryGetValue(CommandKey(theArgs[0]), out aHandler))
        {
            // As in Tcl, a procedure named unknown gets the calls of commands that don't exist
            if (theArgs[0] != "unknown" && myCommands.ContainsKey("unknown"))
            {
                return Invoke(["unknown", .. theArgs], theLine);
            }

            throw new DrawCommandNotFoundException(theArgs[0], theLine);
        }

        try
        {
            return aHandler(this, theArgs);
        }
        catch (DrawException anError) when (TagLine(anError, theLine))
        {
            throw;
        }
        catch (Exception anError) when (anError is not (DrawException or TclFlowException or DrawExitException or DrawTimeoutException))
        {
            throw new DrawException($"{theArgs[0]}: {Refusal.Unwrapped(anError).Message}", theLine);
        }
    }

    // Errors are tagged in exception filters, which run without unwinding: rethrowing at each of a thousand
    // nested levels would make an error's cost quadratic in the depth
    private static bool TagLine(DrawException theError, int theLine)
    {
        if (theError.Line == 0)
        {
            theError.Line = theLine;
        }

        return false;
    }

    internal static bool TagFile(DrawException theError, string thePath)
    {
        theError.File ??= thePath;
        return false;
    }

    internal string Substitute(Word theWord)
    {
        if (theWord.Literal is { } aLiteral)
        {
            return aLiteral;
        }

        return SubstituteParts(theWord.Parts);
    }

    internal string SubstituteParts(IReadOnlyList<WordPart> theParts)
    {
        if (theParts.Count == 1)
        {
            return SubstitutePart(theParts[0]);
        }

        var aBuilder = new StringBuilder();
        foreach (WordPart aPart in theParts)
        {
            aBuilder.Append(SubstitutePart(aPart));
        }

        return aBuilder.ToString();
    }

    private string SubstitutePart(WordPart thePart) => thePart switch
    {
        WordPart.Text aText => aText.Value,
        WordPart.Variable aVariable => GetVariable(aVariable.Name, aVariable.Index is null ? null : SubstituteParts(aVariable.Index)),
        WordPart.Command aCommand => EvalScript(aCommand.Script),
        _ => "",
    };

    // -------------------------------------------------------------------------------------------------------------
    // Procedures and frames

    internal int Level => myFrame.Level;

    internal TclFrame CurrentFrame => myFrame;

    internal TclFrame GlobalFrame => myGlobal;

    /// <summary>The frames of the procedure calls in progress, outermost first (the global frame isn't among them).</summary>
    private readonly List<TclFrame> myActiveFrames = [];

    /// <summary>The global frame and the frames of the procedure calls in progress (uplevel or not).</summary>
    internal IEnumerable<TclFrame> LiveFrames() => myActiveFrames.Prepend(myGlobal);

    private string CallProc(TclProc theProc, IReadOnlyList<string> theArgs)
    {
        var aFrame = new TclFrame(myFrame.Level + 1, myFrame, theArgs);
        IReadOnlyList<(string Name, string? Default)> aParams = theProc.Parameters;
        int aGiven = theArgs.Count - 1;
        for (int i = 0; i < aParams.Count; i++)
        {
            (string aName, string? aDefault) = aParams[i];
            if (i == aParams.Count - 1 && aName == "args")
            {
                aFrame.Vars[aName] = new TclVar { Value = TclList.Make(theArgs.Skip(i + 1).ToArray()) };
                aGiven = i;
                break;
            }

            string? aValue = i < aGiven ? theArgs[i + 1] : aDefault;
            if (aValue is null)
            {
                throw WrongProcArgs(theProc, theArgs[0]);
            }

            aFrame.Vars[aName] = new TclVar { Value = aValue };
        }

        if (aGiven > aParams.Count)
        {
            throw WrongProcArgs(theProc, theArgs[0]);
        }

        if (myProcDepth >= MaxProcDepth)
        {
            throw new DrawException("too many nested evaluations (infinite loop?)");
        }

        TclFrame aSaved = myFrame;
        myFrame = aFrame;
        myProcDepth++;
        myActiveFrames.Add(aFrame);
        try
        {
            return EvalScript(theProc.Body);
        }
        catch (TclReturnException aReturn)
        {
            return aReturn.Code switch
            {
                0 => aReturn.Value,
                1 => throw new DrawException(aReturn.Value) { ErrorCode = aReturn.ErrorCode },
                3 => throw TclBreakException.Instance,
                4 => throw TclContinueException.Instance,
                _ => throw new TclReturnException(0, aReturn.Value),
            };
        }
        catch (TclBreakException)
        {
            throw new DrawException("invoked \"break\" outside of a loop");
        }
        catch (TclContinueException)
        {
            throw new DrawException("invoked \"continue\" outside of a loop");
        }
        finally
        {
            myFrame = aSaved;
            myProcDepth--;
            myActiveFrames.RemoveAt(myActiveFrames.Count - 1);
        }
    }

    private static DrawException WrongProcArgs(TclProc theProc, string theName)
    {
        var aUsage = new StringBuilder(theName);
        for (int i = 0; i < theProc.Parameters.Count; i++)
        {
            (string aName, string? aDefault) = theProc.Parameters[i];
            aUsage.Append(' ').Append(aName == "args" && i == theProc.Parameters.Count - 1 ? "?arg ...?"
                                     : aDefault is null ? aName : $"?{aName}?");
        }

        return new DrawException($"wrong # args: should be \"{aUsage}\"");
    }

    /// <summary>The frame a level argument (<c>1</c>, <c>#0</c>) names, relative to the current frame.</summary>
    internal TclFrame FrameAt(string theLevel)
    {
        int aTarget;
        if (theLevel.StartsWith('#') && int.TryParse(theLevel.AsSpan(1), out int anAbsolute))
        {
            aTarget = anAbsolute;
        }
        else if (int.TryParse(theLevel, out int aRelative) && aRelative >= 0)
        {
            aTarget = myFrame.Level - aRelative;
        }
        else
        {
            throw new DrawException($"bad level \"{theLevel}\"");
        }

        TclFrame? aFrame = myFrame;
        while (aFrame is not null && aFrame.Level > aTarget)
        {
            aFrame = aFrame.Caller;
        }

        return aFrame is not null && aFrame.Level == aTarget ? aFrame : throw new DrawException($"bad level \"{theLevel}\"");
    }

    /// <summary>True when a word reads as a level (for uplevel and upvar, whose level is optional).</summary>
    internal static bool IsLevel(string theWord)
        => theWord.Length > 0 && (theWord[0] == '#' ? theWord.Length > 1 && theWord.Skip(1).All(char.IsAsciiDigit) : theWord.All(char.IsAsciiDigit));

    /// <summary>Runs a script in another frame (uplevel).</summary>
    internal string EvalInFrame(TclFrame theFrame, string theScript)
    {
        TclFrame aSaved = myFrame;
        myFrame = theFrame;
        try
        {
            return EvalScript(theScript);
        }
        finally
        {
            myFrame = aSaved;
        }
    }

    // -------------------------------------------------------------------------------------------------------------
    // Variables

    /// <summary>Splits <c>name(index)</c> into the array name and index; a scalar name has no index.</summary>
    internal static (string Name, string? Index) SplitName(string theName)
    {
        if (theName.Length > 1 && theName[^1] == ')')
        {
            int anOpen = theName.IndexOf('(');
            if (anOpen >= 0)
            {
                return (theName[..anOpen], theName[(anOpen + 1)..^1]);
            }
        }

        return (theName, null);
    }

    /// <summary>The variable a name refers to in the current frame (or the global one, for ::name).</summary>
    internal TclVar? FindVar(string theName, bool theCreate)
    {
        TclFrame aFrame = myFrame;
        string aName = theName;
        if (aName.StartsWith("::", StringComparison.Ordinal))
        {
            aFrame = myGlobal;
            aName = aName.TrimStart(':');
        }

        if (!aFrame.Vars.TryGetValue(aName, out TclVar? aVar) && theCreate)
        {
            aVar = new TclVar();
            aFrame.Vars[aName] = aVar;
        }

        return aVar;
    }

    internal static string DisplayName(string theName, string? theIndex) => theIndex is null ? theName : $"{theName}({theIndex})";

    public bool TryGetVariable(string theName, out string theValue)
    {
        (string aName, string? anIndex) = SplitName(theName);
        return TryGetVariable(aName, anIndex, out theValue);
    }

    internal bool TryGetVariable(string theName, string? theIndex, out string theValue)
    {
        theValue = "";
        TclVar? aVar = FindVar(theName, false);
        if (aVar is null)
        {
            return false;
        }

        if (theIndex is null)
        {
            if (aVar.Value is null)
            {
                return false;
            }

            theValue = aVar.Value;
            return true;
        }

        if (aVar.Array is not null && aVar.Array.TryGetValue(theIndex, out string? anElement))
        {
            theValue = anElement;
            return true;
        }

        return false;
    }

    internal string GetVariable(string theName, string? theIndex = null)
    {
        if (theIndex is null)
        {
            (theName, theIndex) = SplitName(theName);
        }

        if (TryGetVariable(theName, theIndex, out string aValue))
        {
            return aValue;
        }

        TclVar? aVar = FindVar(theName, false);
        string aDisplay = DisplayName(theName, theIndex);
        string aWhy = theIndex is null
                          ? aVar?.Array is not null ? "variable is array" : "no such variable"
                          : aVar?.Array is not null ? "no such element in array"
                          : aVar?.Value is not null ? "variable isn't array" : "no such variable";
        throw new DrawException($"can't read \"{aDisplay}\": {aWhy}");
    }

    public void SetVariable(string theName, string theValue)
    {
        (string aName, string? anIndex) = SplitName(theName);
        SetVariable(aName, anIndex, theValue);
    }

    internal string SetVariable(string theName, string? theIndex, string theValue)
    {
        TclVar aVar = FindVar(theName, true)!;
        if (theIndex is null)
        {
            if (aVar.Array is not null)
            {
                throw new DrawException($"can't set \"{theName}\": variable is array");
            }

            WriteTrace(aVar, null, theName);
            aVar.Value = theValue;
        }
        else
        {
            if (aVar.Value is not null)
            {
                throw new DrawException($"can't set \"{DisplayName(theName, theIndex)}\": variable isn't array");
            }

            WriteTrace(aVar, theIndex, DisplayName(theName, theIndex));
            aVar.Array ??= new Dictionary<string, string>(StringComparer.Ordinal);
            aVar.Array[theIndex] = theValue;
        }

        return theValue;
    }

    public bool UnsetVariable(string theName)
    {
        (string aName, string? anIndex) = SplitName(theName);
        TclVar? aVar = FindVar(aName, false);
        if (aVar is null || !aVar.Exists)
        {
            return false;
        }

        if (anIndex is null)
        {
            if (aVar.Drawable is { Protected: true })
            {
                // Draw's trace sets a protected variable again
                return true;
            }

            aVar.Value = null;
            aVar.Array = null;
            aVar.Drawable = null;
            aVar.ElementDrawables = null;
            return true;
        }

        aVar.ElementDrawables?.Remove(anIndex);
        return aVar.Array is not null && aVar.Array.Remove(anIndex);
    }

    /// <summary>Makes a local name refer to a variable of another frame (upvar, global).</summary>
    internal void LinkVariable(TclFrame theFrame, string theOther, string theLocal)
    {
        if (SplitName(theLocal).Index is not null)
        {
            throw new DrawException($"bad variable name \"{theLocal}\": can't create a scalar variable that looks like an array element");
        }

        (string anOther, string? anIndex) = SplitName(theOther);
        if (anIndex is not null)
        {
            throw new DrawException($"upvar to an array element (\"{theOther}\") is not supported by Draw-lite");
        }

        TclFrame aFrame = theFrame;
        if (anOther.StartsWith("::", StringComparison.Ordinal))
        {
            aFrame = myGlobal;
            anOther = anOther.TrimStart(':');
        }

        if (!aFrame.Vars.TryGetValue(anOther, out TclVar? aTarget))
        {
            aTarget = new TclVar();
            aFrame.Vars[anOther] = aTarget;
        }

        string aLocal = theLocal.StartsWith("::", StringComparison.Ordinal) ? theLocal.TrimStart(':') : theLocal;
        if (ReferenceEquals(aFrame, myFrame) && aLocal == anOther)
        {
            return;
        }

        myFrame.Vars[aLocal] = aTarget;
    }
}
