// Port of: src/FoundationClasses/TKernel/Message/Message_Alert.hxx
// Port of: src/FoundationClasses/TKernel/Message/Message_Alert.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2017-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Foundation;

/// <summary>
/// The base of the alerts: situations met by an algorithm (errors, warnings) that it records in a
/// <see cref="Report"/>. An alert's type identifies it; <see cref="GetMessageKey"/> (by default the class name)
/// keys its message text. An alert may merge with another of the same type already in a report
/// (<see cref="SupportsMerge"/>, <see cref="Merge"/>), so that the report doesn't repeat it.
/// </summary>
/// <remarks>
/// OCCT's <c>DEFINE_SIMPLE_ALERT(Alert)</c> macro, an alert without data, is a plain subclass in the port:
/// <c>public sealed class Alert : Message_Alert { }</c>. A working object: alerts belong to the report of the
/// algorithm that made them. <c>DumpJson</c> is left out (translation rules, section 6).
/// </remarks>
internal class Alert : Transient
{
    /// <summary>
    /// The key of the alert's message text (<see cref="MsgFile"/>): by default the name of the alert's class.
    /// </summary>
    public virtual string GetMessageKey() => DynamicTypeName();

    /// <summary>True if alerts of this type can be merged with others of the same type (true by default).</summary>
    public virtual bool SupportsMerge()
    {
        // by default, support merge
        return true;
    }

    /// <summary>Merges this alert's data into <paramref name="target"/> if possible; true if merged (always, by default).</summary>
    public virtual bool Merge(Alert target)
    {
        // by default, merge trivially
        return true;
    }
}
