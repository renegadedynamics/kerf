namespace OcctGen;

/// <summary>
/// The binding whitelist. One OCCT class per line (its header is &lt;Class&gt;.hxx); '#' starts a
/// comment; "-Class::Method" excludes every overload of a member that would otherwise be bound.
/// </summary>
sealed class Config
{
    public List<string> Classes { get; } = [];
    public HashSet<string> Excluded { get; } = [];

    public static Config Load(string path)
    {
        var config = new Config();
        foreach (var raw in File.ReadLines(path))
        {
            var line = raw.Split('#')[0].Trim();
            if (line.Length == 0)
                continue;
            if (line.StartsWith('-'))
                config.Excluded.Add(line[1..]);
            else if (!config.Classes.Contains(line))
                config.Classes.Add(line);
        }
        return config;
    }

    public bool IsExcluded(string cls, string member) => Excluded.Contains($"{cls}::{member}");
}
