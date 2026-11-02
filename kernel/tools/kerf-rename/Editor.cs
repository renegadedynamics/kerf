// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Text;
using System.Text.RegularExpressions;
using Microsoft.CodeAnalysis.Text;

namespace Kerf.Tools.Rename;

/// <summary>Turns a document's occurrences into text changes, and MSBuild's Using items into their new names.</summary>
internal sealed class Editor(Plan plan)
{
    private readonly Dictionary<string, HashSet<string>> _newNamesByNamespace = plan.Namespaces
        .GroupBy(p => p.Value, p => plan.NameOf(p.Key)!)
        .ToDictionary(g => g.Key, g => g.ToHashSet(StringComparer.Ordinal), StringComparer.Ordinal);

    /// <summary>The document's new text, or null when nothing changes; the number of changes made.</summary>
    public (string? Text, int Changes) Edit(DocumentScan scan)
    {
        string newline = scan.Text.ToString().Contains("\r\n", StringComparison.Ordinal) ? "\r\n" : "\n";
        var changes = new List<TextChange>();
        var bodies = new List<TextSpan>();
        var usings = new SortedSet<string>(StringComparer.Ordinal);

        // Namespace declarations whose types all move
        var moved = new Dictionary<int, string>();
        foreach ((int start, TextSpan nameSpan, string name, List<string> types) in scan.NamespaceDeclarations)
        {
            if (types.Count > 0 && types.All(plan.Namespaces.ContainsKey) && types.Select(t => plan.Namespaces[t]).Distinct().Count() == 1)
            {
                string target = plan.Namespaces[types[0]];
                moved[start] = target;
                changes.Add(new TextChange(nameSpan, target));
                usings.Add(name);
            }
        }

        // Where a type named on its own needs a using directive, and where adding one would make something ambiguous
        var qualifyIn = new HashSet<string>(StringComparer.Ordinal);
        foreach (Occurrence occurrence in scan.Occurrences)
        {
            if (occurrence.Kind == OccurrenceKind.TypeUnqualified && plan.NameOf(occurrence.Key) is not null
                && plan.Namespaces.TryGetValue(occurrence.Key, out string? target) && !occurrence.Ambiguous
                && !Visible(target, moved.GetValueOrDefault(occurrence.NamespaceDeclaration, occurrence.Namespace)))
            {
                if (_newNamesByNamespace[target].Overlaps(scan.Identifiers))
                {
                    qualifyIn.Add(target);
                }
                else
                {
                    usings.Add(target);
                }
            }
        }

        foreach (string target in qualifyIn)
        {
            usings.Remove(target);
        }

        foreach (Occurrence occurrence in scan.Occurrences)
        {
            if (plan.NameOf(occurrence.Key) is not { } newName)
            {
                continue;
            }

            bool converts = plan.Converts(occurrence.Key);
            switch (occurrence.Kind)
            {
                case OccurrenceKind.Name:
                case OccurrenceKind.MethodGroup:
                    changes.Add(new TextChange(occurrence.Span, occurrence.Qualified ?? newName));
                    break;
                case OccurrenceKind.Invoked:
                case OccurrenceKind.Cref:
                case OccurrenceKind.DeclaredExpression:
                    changes.Add(new TextChange(occurrence.Span, newName));
                    if (converts && occurrence.Extra.Length > 0)
                    {
                        changes.Add(new TextChange(occurrence.Extra, ""));
                    }

                    break;
                case OccurrenceKind.DeclaredAbstract:
                    changes.Add(new TextChange(occurrence.Span, newName));
                    if (converts)
                    {
                        changes.Add(new TextChange(occurrence.Extra, " { get; }"));
                    }

                    break;
                case OccurrenceKind.DeclaredBlock:
                    changes.Add(new TextChange(occurrence.Span, newName));
                    if (converts)
                    {
                        changes.Add(new TextChange(occurrence.Extra, ""));
                        bodies.Add(occurrence.Body);
                    }

                    break;
                case OccurrenceKind.TypeQualified:
                    changes.Add(new TextChange(occurrence.Span, newName));
                    string qualifier = plan.Namespaces[occurrence.Key];
                    changes.Add(new TextChange(occurrence.Extra, occurrence.GlobalQualifier ? "global::" + qualifier : qualifier));
                    break;
                case OccurrenceKind.TypeUnqualified:
                    string? @namespace = plan.Namespaces.GetValueOrDefault(occurrence.Key);
                    // Qualified where the new name would mean something else, even in its own namespace
                    bool qualify = @namespace is not null && (occurrence.Ambiguous || (qualifyIn.Contains(@namespace)
                                   && !Visible(@namespace, moved.GetValueOrDefault(occurrence.NamespaceDeclaration, occurrence.Namespace))));
                    changes.Add(new TextChange(occurrence.Span, qualify ? $"global::{@namespace}.{newName}" : newName));
                    break;
                case OccurrenceKind.AccessorName:
                    changes.Add(new TextChange(occurrence.Span, "\"" + newName + "\""));
                    break;
            }
        }

        foreach (string @namespace in usings)
        {
            if (scan.Usings.Any(u => u.Name == @namespace))
            {
                continue;
            }

            (TextSpan Span, string? Name) next = scan.Usings.FirstOrDefault(u => u.Name is not null && string.CompareOrdinal(u.Name, @namespace) > 0);
            if (next.Name is not null)
            {
                changes.Add(new TextChange(new TextSpan(next.Span.Start, 0), $"using {@namespace};{newline}"));
            }
            else if (scan.Usings.Count > 0)
            {
                // After the last plain using, or before the first static or alias one
                int last = scan.Usings.FindLastIndex(u => u.Name is not null);
                changes.Add(last >= 0
                    ? new TextChange(new TextSpan(scan.Usings[last].Span.End, 0), $"{newline}using {@namespace};")
                    : new TextChange(new TextSpan(scan.Usings[0].Span.Start, 0), $"using {@namespace};{newline}"));
            }
            else
            {
                changes.Add(new TextChange(new TextSpan(scan.FirstTokenStart, 0), $"using {@namespace};{newline}{newline}"));
            }
        }

        if (changes.Count == 0)
        {
            return (null, 0);
        }

        // A method body that becomes a getter: its own changes first, then it is wrapped and re-indented
        foreach (TextSpan body in bodies)
        {
            var inner = changes.Where(c => body.Contains(c.Span)).ToList();
            changes.RemoveAll(c => body.Contains(c.Span));
            string text = SourceText.From(scan.Text.ToString(body))
                .WithChanges(inner.Select(c => new TextChange(new TextSpan(c.Span.Start - body.Start, c.Span.Length), c.NewText!))).ToString();
            changes.Add(new TextChange(body, Getter(text, Indentation(scan.Text, body.Start), newline)));
        }

        var ordered = changes.Distinct().OrderBy(c => c.Span.Start).ThenBy(c => c.Span.Length).ToList();
        for (int i = 1; i < ordered.Count; i++)
        {
            if (ordered[i].Span.Start < ordered[i - 1].Span.End)
            {
                throw new InvalidOperationException($"{scan.Path}: overlapping changes at {ordered[i - 1].Span} and {ordered[i].Span}");
            }
        }

        return (scan.Text.WithChanges(ordered).ToString(), ordered.Count);
    }

    private static bool Visible(string @namespace, string from)
        => from == @namespace || from.StartsWith(@namespace + ".", StringComparison.Ordinal);

    private static string Indentation(SourceText text, int position)
    {
        TextLine line = text.Lines.GetLineFromPosition(position);
        string content = text.ToString(line.Span);
        return content[..(content.Length - content.TrimStart().Length)];
    }

    /// <summary><c>{ body }</c> to <c>{ get { body } }</c>, the body indented one level further.</summary>
    private static string Getter(string block, string indentation, string newline)
    {
        string[] lines = block.Split(newline);
        var text = new StringBuilder();
        text.Append('{').Append(newline).Append(indentation).Append("    get").Append(newline).Append(indentation).Append("    ").Append(lines[0]);
        for (int i = 1; i < lines.Length; i++)
        {
            text.Append(newline);
            if (lines[i].Length > 0)
            {
                text.Append("    ").Append(lines[i]);
            }
        }

        return text.Append(newline).Append(indentation).Append('}').ToString();
    }

    /// <summary>MSBuild's Using items (global usings and their aliases) naming renamed types: the one place outside C#.</summary>
    public int EditMsBuild(string path, IReadOnlyDictionary<string, string> fullNames, IReadOnlyDictionary<string, string> aliases, bool dryRun)
    {
        string text = File.ReadAllText(path);
        int count = 0;
        string edited = Regex.Replace(text, @"<Using\s+Include=""(?<include>[^""]*)""(?<rest>[^>]*)>", match =>
        {
            string include = match.Groups["include"].Value;
            string rest = match.Groups["rest"].Value;
            string head = Regex.Match(include, @"^[\w.]+").Value;
            if (fullNames.TryGetValue(head, out string? newHead))
            {
                include = newHead + include[head.Length..];
            }

            rest = Regex.Replace(rest, @"Alias=""(?<alias>[^""]*)""",
                a => aliases.TryGetValue(a.Groups["alias"].Value, out string? newAlias) ? $"Alias=\"{newAlias}\"" : a.Value);
            string result = $"<Using Include=\"{include}\"{rest}>";
            if (result != match.Value)
            {
                count++;
            }

            return result;
        });

        if (count > 0 && !dryRun)
        {
            File.WriteAllText(path, edited, Program.EncodingOf(path));
        }

        return count;
    }
}
