namespace OcctGen;

static class Writer
{
    /// <summary>Writes the file only if its content changed, so unchanged sources don't rebuild.</summary>
    public static void WriteIfChanged(string path, string content)
    {
        content = content.ReplaceLineEndings("\n");
        if (File.Exists(path) && File.ReadAllText(path) == content)
            return;
        Directory.CreateDirectory(Path.GetDirectoryName(path)!);
        File.WriteAllText(path, content);
    }

    /// <summary>Deletes generated files in dir that this run didn't produce (e.g. a class removed from the whitelist).</summary>
    public static void Prune(string dir, string pattern, IEnumerable<string> keep)
    {
        var keepSet = keep.Select(Path.GetFullPath).ToHashSet(StringComparer.OrdinalIgnoreCase);
        if (!Directory.Exists(dir))
            return;
        foreach (var f in Directory.GetFiles(dir, pattern))
            if (!keepSet.Contains(Path.GetFullPath(f)))
                File.Delete(f);
    }
}
