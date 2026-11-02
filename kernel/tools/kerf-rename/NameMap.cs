// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Text;

namespace Kerf.Tools.Rename;

/// <summary>
/// The reviewed name map: one entry per line, <c>kind id [new-name] [# note]</c>, sorted, so that a review is a diff.
/// </summary>
/// <remarks>
/// Ids are Roslyn's documentation comment ids of the original definitions (<c>T:Occt.gp_Pnt</c>,
/// <c>M:Occt.math_FunctionRoot.IsDone</c>, <c>F:Occt.gp_TrsfForm.gp_Identity</c>). A parameter is its method's id,
/// <c>#</c> and its name as declared. Kinds:
/// <list type="bullet">
/// <item><c>rule name on|off</c>: a systematic rule (<see cref="Rules"/>): parameters, fields, statics, locals, methods.</item>
/// <item><c>package prefix namespace</c>: where the drafter puts a package's types (<c>package gp Kerf.Geometry</c>).</item>
/// <item><c>type id Full.New.Name</c>: a type's new name and namespace.</item>
/// <item><c>member id NewName</c>: a member's new name: method, property, field, event, enum member.</item>
/// <item><c>property id NewName</c>: a parameterless method that becomes a get-only property.</item>
/// <item><c>param id#name newName</c>: a parameter's name, overriding the rule.</item>
/// <item><c>alias name newName</c>: a using alias (<c>math_Vector</c>), in code and in MSBuild's Using items.</item>
/// <item><c>keep id</c>: no rule applies to this symbol (a reported collision, or a reviewer's choice).</item>
/// </list>
/// </remarks>
internal sealed class NameMap
{
    public static class Rules
    {
        /// <summary><c>theX</c> to <c>x</c>.</summary>
        public const string Parameters = "parameters";

        /// <summary>Non-public instance fields: <c>myX</c> to <c>_x</c>, <c>coord</c> to <c>_coord</c>.</summary>
        public const string Fields = "fields";

        /// <summary>Non-public static fields: mutable <c>s_x</c>, constant and read-only <c>X</c>.</summary>
        public const string Statics = "statics";

        /// <summary><c>aX</c>/<c>anX</c> to <c>x</c>.</summary>
        public const string Locals = "locals";

        /// <summary>camelCase methods (OCCT's file-static helpers) to PascalCase.</summary>
        public const string Methods = "methods";

        public static readonly string[] All = [Parameters, Fields, Statics, Locals, Methods];
    }

    public Dictionary<string, bool> RuleSwitches { get; } = new(StringComparer.Ordinal);
    public SortedDictionary<string, string> Packages { get; } = new(StringComparer.Ordinal);
    public SortedDictionary<string, string> Types { get; } = new(StringComparer.Ordinal);
    public SortedDictionary<string, string> Members { get; } = new(StringComparer.Ordinal);
    public SortedDictionary<string, string> Properties { get; } = new(StringComparer.Ordinal);
    public SortedDictionary<string, string> Params { get; } = new(StringComparer.Ordinal);
    public SortedDictionary<string, string> Aliases { get; } = new(StringComparer.Ordinal);
    public SortedSet<string> Keep { get; } = new(StringComparer.Ordinal);

    /// <summary>Notes kept with entries, by "kind id", written back after them.</summary>
    public Dictionary<string, string> Notes { get; } = new(StringComparer.Ordinal);

    public bool Rule(string name) => RuleSwitches.TryGetValue(name, out bool on) && on;

    public static NameMap Load(string path)
    {
        var map = new NameMap();
        int lineNumber = 0;
        foreach (string raw in File.ReadLines(path))
        {
            lineNumber++;
            string line = raw;
            string? note = null;
            int hash = line.IndexOf(" #", StringComparison.Ordinal);
            if (line.StartsWith('#'))
            {
                continue;
            }

            if (hash >= 0)
            {
                note = line[(hash + 2)..].Trim();
                line = line[..hash];
            }

            string[] parts = line.Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries);
            if (parts.Length == 0)
            {
                continue;
            }

            string Need(int i) => i < parts.Length
                ? parts[i]
                : throw new FormatException($"{path}({lineNumber}): '{parts[0]}' needs {i} arguments");

            switch (parts[0])
            {
                case "rule":
                    map.RuleSwitches[Need(1)] = Need(2) == "on";
                    break;
                case "package":
                    map.Packages[Need(1)] = Need(2);
                    break;
                case "type":
                    map.Types[Need(1)] = Need(2);
                    break;
                case "member":
                    map.Members[Need(1)] = Need(2);
                    break;
                case "property":
                    map.Properties[Need(1)] = Need(2);
                    break;
                case "param":
                    map.Params[Need(1)] = Need(2);
                    break;
                case "alias":
                    map.Aliases[Need(1)] = Need(2);
                    break;
                case "keep":
                    map.Keep.Add(Need(1));
                    break;
                default:
                    throw new FormatException($"{path}({lineNumber}): unknown entry '{parts[0]}'");
            }

            if (note is not null)
            {
                map.Notes[parts[0] + " " + parts[1]] = note;
            }
        }

        return map;
    }

    public void Save(string path, IEnumerable<string> header)
    {
        var text = new StringBuilder();
        foreach (string line in header)
        {
            text.Append("# ").AppendLine(line);
        }

        text.AppendLine();
        foreach (string rule in Rules.All.Concat(RuleSwitches.Keys.Except(Rules.All)))
        {
            if (RuleSwitches.TryGetValue(rule, out bool on))
            {
                text.AppendLine($"rule {rule} {(on ? "on" : "off")}");
            }
        }

        Section(text, "package", Packages);
        Section(text, "alias", Aliases);
        Section(text, "type", Types);
        Section(text, "member", Members);
        Section(text, "property", Properties);
        Section(text, "param", Params);
        if (Keep.Count > 0)
        {
            text.AppendLine();
            foreach (string id in Keep)
            {
                text.AppendLine(WithNote("keep " + id, "keep " + id));
            }
        }

        File.WriteAllText(path, text.ToString().ReplaceLineEndings("\n"));
    }

    private void Section(StringBuilder text, string kind, SortedDictionary<string, string> entries)
    {
        if (entries.Count == 0)
        {
            return;
        }

        text.AppendLine();
        foreach ((string id, string name) in entries)
        {
            text.AppendLine(WithNote($"{kind} {id} {name}", kind + " " + id));
        }
    }

    private string WithNote(string line, string key) => Notes.TryGetValue(key, out string? note) ? line + "  # " + note : line;
}
