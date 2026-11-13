// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Concurrent;
using System.Text;
using System.Text.RegularExpressions;

namespace Kerf.Engine.Foundation;

/// <summary>
/// The bridge from the engine's exceptions to diagnostics (phase10-plan.md 2.5): an exception that escapes an engine
/// algorithm becomes an error with a stable code, for the operations that report failures as results.
/// </summary>
/// <remarks>
/// <para>
/// An OCCT exception (a <see cref="FailureException"/>) has the code <c>engine.</c> followed by its OCCT class name,
/// without the package, in lower case with hyphens between words: <c>StdFail_NotDone</c> is <c>engine.not-done</c>,
/// <c>Standard_ConstructionError</c> <c>engine.construction-error</c>. Any other exception from inside the engine
/// (an index out of range, a null reference) is a fault in the port: <c>engine.fault</c>. The Booleans' own failure
/// (<c>BRepAlgoAPI_Failure</c>) is <c>boolean.failed</c>. docs/api/diagnostics.md
/// lists the codes.
/// </para>
/// <para>
/// The message says in plain words what kind of failure it was, then OCCT's own message if it has one, in Kerf's terms
/// (<see cref="PublicText(string)"/>).
/// </para>
/// </remarks>
internal static partial class FailureDiagnostics
{
    /// <summary>The code of a fault: an exception from the engine that isn't one of OCCT's.</summary>
    public static DiagnosticCode Fault { get; } = new("engine.fault");

    private static readonly ConcurrentDictionary<Type, DiagnosticCode> Codes = new();

    // The exceptions whose code the rule would make misleading: the Booleans' failure isn't Standard_Failure
    private static readonly Dictionary<string, string> Overrides = new(StringComparer.Ordinal)
    {
        ["BRepAlgoAPI_Failure"] = "boolean.failed",
    };

    /// <summary>The code for an exception of type <paramref name="exceptionType"/>.</summary>
    public static DiagnosticCode CodeOf(Type exceptionType)
    {
        KernelArgumentNullException.ThrowIfNull(exceptionType);
        if (!typeof(FailureException).IsAssignableFrom(exceptionType))
            return Fault;
        return Codes.GetOrAdd(exceptionType, static type =>
            Overrides.TryGetValue(OcctTypeNames.Of(type), out var code) ? new(code) : new("engine." + Kebab(OcctName(type))));
    }

    /// <summary>The error for an exception that escaped the engine.</summary>
    public static Diagnostic ToDiagnostic(Exception exception)
    {
        KernelArgumentNullException.ThrowIfNull(exception);
        if (exception is UnconfirmedMeasureException unconfirmed)
        {
            // A measure not known to its precision says so with its own code (decision 34806)
            return unconfirmed.Diagnostic;
        }

        exception = Refusal.Unwrapped(exception);
        var own = PublicText(exception);
        var kind = Describe(exception);
        return Diagnostic.Error(CodeOf(exception.GetType()), own.Length == 0 ? kind : $"{kind}: {own}");
    }

    /// <summary>
    /// What an exception says for a public message: the failure's own message in Kerf's terms (<see cref="PublicText(string)"/>),
    /// empty if it was raised without one, or any other exception's message as it is, as that can hold a path.
    /// </summary>
    public static string PublicText(Exception exception)
    {
        if (Refusal.Of(exception) is not { } failure)
            return exception.Message;
        // A failure raised without a message has .NET's default one, where OCCT's would be empty
        string own = failure.GetMessageString();
        return own.StartsWith("Exception of type '", StringComparison.Ordinal) ? "" : PublicText(own);
    }

    /// <summary>
    /// An engine message for callers who have never heard of OCCT: without the member it starts with
    /// (<c>gp_Dir() - input vector has zero norm</c>), and with any type it names later on by its last part
    /// (<c>Make Geom_Curve (3D) failed</c> says <c>Curve</c>). The engine's messages stay as OCCT words them, as the way
    /// back to its source, and only this leaves.
    /// </summary>
    public static string PublicText(string message)
    {
        string text = LeadingMember().Replace(message.Trim(), "", 1);
        text = NamedMember().Replace(text, "$1");
        return NamedType().Replace(text, "$1").Trim();
    }

    // The type part of an OCCT name: TopoDS_Face, gp_Dir, BRepAlgoAPI_Failure, but not STEP's own CARTESIAN_POINT
    private const string OcctType = @"(?=[A-Za-z0-9]*[a-z])[A-Za-z][A-Za-z0-9]*_[A-Za-z0-9_]+";

    // At the start: an OCCT type, or a member (BRepTools::Read), with the separator after it
    [GeneratedRegex(@"^(?:" + OcctType + @"(?:::~?\w+)?|[A-Za-z][A-Za-z0-9]*::~?\w+)(?:\(\))?(?:\s*[-:,]\s*|\s+|$)")]
    private static partial Regex LeadingMember();

    [GeneratedRegex(@"\b(?:" + OcctType + @"|[A-Za-z][A-Za-z0-9]*)::(~?\w+)(?:\(\))?")]
    private static partial Regex NamedMember();

    [GeneratedRegex(@"\b(?=[A-Za-z0-9]*[a-z])[A-Za-z][A-Za-z0-9]*_([A-Z][A-Za-z0-9]*(?:_[A-Za-z0-9]+)*)\b")]
    private static partial Regex NamedType();

    /// <summary>The plain description of a kind of failure, by the nearest OCCT family it belongs to.</summary>
    internal static string Describe(Exception exception) => exception switch
    {
        NotDoneException => "the algorithm couldn't complete",
        ConstructionErrorException => "the geometry can't be constructed",
        NullVectorException => "a vector or direction has zero length",
        TypeMismatchException => "a shape or geometry is of the wrong type",
        NullObjectException => "something needed is missing",
        NoSuchObjectException or NoMoreObjectException => "something looked for isn't there",
        RangeErrorException => "a value is out of range",
        DomainErrorException => "a value is outside what the operation accepts",
        NumericErrorException => "a numeric error",
        InfiniteSolutionsException => "there are infinitely many solutions",
        MethodNotImplementedException => "not implemented",
        StreamFormatErrorException => "the data is malformed",
        FailureException => "the operation failed",
        _ => "an internal fault",
    };

    // The OCCT class name without its package: StdFail_NotDone → NotDone
    private static string OcctName(Type type)
    {
        var name = OcctTypeNames.Of(type);
        var underscore = name.IndexOf('_');
        return underscore >= 0 ? name[(underscore + 1)..] : name;
    }

    // NotDone → not-done, BRepAlgoAPI → brepalgoapi; runs of capitals stay one word
    internal static string Kebab(string name)
    {
        var text = new StringBuilder(name.Length + 8);
        for (int i = 0; i < name.Length; i++)
        {
            char c = name[i];
            if (!char.IsLetterOrDigit(c))
            {
                if (text.Length > 0 && text[^1] != '-')
                    text.Append('-');
                continue;
            }
            bool wordStart = char.IsUpper(c) && i > 0 && text.Length > 0 && text[^1] != '-'
                && (char.IsLower(name[i - 1]) || (i + 1 < name.Length && char.IsLower(name[i + 1])));
            if (wordStart)
                text.Append('-');
            text.Append(char.ToLowerInvariant(c));
        }
        return text.ToString().Trim('-');
    }
}
