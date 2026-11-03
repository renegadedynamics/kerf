// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using Kerf.Engine.Foundation;

namespace Kerf.Engine.Topology;

/// <summary>
/// The bridge from a <see cref="Report"/>'s alerts to diagnostics (phase10-plan.md 2.5): what the Booleans and the
/// other algorithms with reports recorded becomes warnings and errors with stable codes, naming the sub-shapes an alert
/// holds.
/// </summary>
/// <remarks>
/// <para>
/// An alert's code is its area, a dot, and its OCCT class name without the package and the word <c>Alert</c>, in lower
/// case with hyphens between words: <c>BOPAlgo_AlertSelfInterferingShape</c> is
/// <c>boolean.self-interfering-shape</c>. The areas are by package (BOPAlgo's alerts are <c>boolean</c>); an alert of
/// another package is <c>engine</c>. docs/api/diagnostics.md lists the codes.
/// </para>
/// <para>
/// A <see cref="Gravity.Fail"/> alert is an error; <see cref="Gravity.Alarm"/> and <see cref="Gravity.Warning"/> are
/// warnings; information and traces are left out. The message is the alert's text from the message table
/// (<see cref="MsgFile"/>) where it has one, which BOPAlgo's do.
/// </para>
/// </remarks>
public static class AlertDiagnostics
{
    private static readonly Dictionary<string, string> Areas = new(StringComparer.Ordinal)
    {
        ["BOPAlgo"] = "boolean",
    };

    /// <summary>The code for an alert.</summary>
    public static DiagnosticCode CodeOf(Alert alert)
    {
        ArgumentNullException.ThrowIfNull(alert);
        var name = alert.DynamicTypeName();
        var underscore = name.IndexOf('_');
        var package = underscore >= 0 ? name[..underscore] : "";
        var own = underscore >= 0 ? name[(underscore + 1)..] : name;
        if (own.StartsWith("Alert", StringComparison.Ordinal) && own.Length > "Alert".Length)
            own = own["Alert".Length..];
        var area = Areas.GetValueOrDefault(package, "engine");
        return new(area + "." + FailureDiagnostics.Kebab(own));
    }

    /// <summary>The diagnostic for an alert of <paramref name="gravity"/>, or null for information and traces.</summary>
    public static Diagnostic? ToDiagnostic(Alert alert, Gravity gravity)
    {
        ArgumentNullException.ThrowIfNull(alert);
        Severity severity;
        switch (gravity)
        {
            case Gravity.Fail: severity = Severity.Error; break;
            case Gravity.Alarm or Gravity.Warning: severity = Severity.Warning; break;
            default: return null;
        }
        var key = alert.GetMessageKey();
        var message = MsgFile.HasMsg(key) ? MsgFile.Msg(key) : Words(alert);
        var diagnostic = new Diagnostic { Severity = severity, Code = CodeOf(alert), Message = message };
        return alert is AlertWithShape withShape && !withShape.InternalGetShape().IsNull
            ? diagnostic.WithShapes(withShape.GetShape())
            : diagnostic;
    }

    /// <summary>
    /// The report's warnings and errors, in OCCT's order of gravity (warnings, alarms, then failures) and each
    /// gravity's order of recording.
    /// </summary>
    public static ImmutableArray<Diagnostic> ToDiagnostics(Report report)
    {
        ArgumentNullException.ThrowIfNull(report);
        var diagnostics = ImmutableArray.CreateBuilder<Diagnostic>();
        foreach (var gravity in (ReadOnlySpan<Gravity>)[Gravity.Warning, Gravity.Alarm, Gravity.Fail])
            foreach (var alert in report.GetAlerts(gravity))
                if (ToDiagnostic(alert, gravity) is { } diagnostic)
                    diagnostics.Add(diagnostic);
        return diagnostics.ToImmutable();
    }

    // An alert without a message text: its code's words ("self interfering shape")
    private static string Words(Alert alert)
    {
        var code = CodeOf(alert).Value;
        var words = code[(code.IndexOf('.') + 1)..].Replace('-', ' ');
        return char.ToUpperInvariant(words[0]) + words[1..];
    }
}
