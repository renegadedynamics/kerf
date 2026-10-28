namespace OcctGen;

/// <summary>
/// The binding whitelist. One OCCT class per line (its header is &lt;Class&gt;.hxx); '#' starts a
/// comment; "-Class::Method" excludes every overload of a member that would otherwise be bound;
/// "@kind Alias = Template&lt;Args&gt;" binds a collection instantiation (see CollectionInfo.Kinds).
/// </summary>
sealed class Config
{
    public List<string> Classes { get; } = [];
    public HashSet<string> Excluded { get; } = [];
    public List<CollectionInfo> Collections { get; } = [];

    public static Config Load(string path)
    {
        var config = new Config();
        foreach (var raw in File.ReadLines(path))
        {
            var line = raw.Split('#')[0].Trim();
            if (line.Length == 0)
                continue;
            if (line.StartsWith('@'))
            {
                var m = System.Text.RegularExpressions.Regex.Match(line, @"^@(\w+)\s+(\w+)\s*=\s*(.+)$");
                if (!m.Success || !CollectionInfo.Kinds.Contains(m.Groups[1].Value))
                    throw new FormatException($"bad collection line: {line}");
                config.Collections.Add(new CollectionInfo(m.Groups[1].Value, m.Groups[2].Value, m.Groups[3].Value));
            }
            else if (line.StartsWith('-'))
                config.Excluded.Add(line[1..]);
            else if (!config.Classes.Contains(line))
                config.Classes.Add(line);
        }
        return config;
    }

    public bool IsExcluded(string cls, string member) => Excluded.Contains($"{cls}::{member}");
}
