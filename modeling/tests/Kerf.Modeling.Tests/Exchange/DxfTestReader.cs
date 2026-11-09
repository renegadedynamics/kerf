using System.Globalization;

namespace Kerf.Modeling.Tests.Exchange;

/// <summary>
/// A DXF reader for the tests (not shipped): it reads what <see cref="ExportDxf"/> writes back into group codes, checks
/// the file's structure, and gives the header, the tables and the entities. It throws on anything malformed: an odd
/// number of lines, a code that isn't a number, an unclosed section or table, a missing EOF, a handle used twice or a
/// pointer to one that isn't there, an entity on a layer or a layer with a line type the tables don't have, a
/// <c>$HANDSEED</c> not above every handle, or a table's count that doesn't match its records.
/// </summary>
internal sealed class DxfTestReader
{
    internal sealed record Group(int Code, string Value)
    {
        public double Number => double.Parse(Value, NumberStyles.Float, CultureInfo.InvariantCulture);
    }

    /// <summary>A record: its type and its groups after the type.</summary>
    internal sealed record Item(string Type, IReadOnlyList<Group> Groups)
    {
        public string? Text(int code) => Groups.FirstOrDefault(g => g.Code == code)?.Value;

        public double Number(int code) => Groups.First(g => g.Code == code).Number;

        public IReadOnlyList<double> Numbers(int code) => [.. Groups.Where(g => g.Code == code).Select(g => g.Number)];

        public Point2D Point(int code) => new(Number(code), Number(code + 10));

        public IReadOnlyList<Point2D> Points(int code)
        {
            var xs = Numbers(code);
            var ys = Numbers(code + 10);
            Assert.Equal(xs.Count, ys.Count);
            return [.. xs.Zip(ys, (x, y) => new Point2D(x, y))];
        }

        public string Layer => Text(8) ?? throw new InvalidDataException($"{Type} has no layer");
    }

    public Dictionary<string, Item> Header { get; } = new(StringComparer.Ordinal);

    public Dictionary<string, List<Item>> Tables { get; } = new(StringComparer.Ordinal);

    public List<Item> Entities { get; } = [];

    public List<Item> Objects { get; } = [];

    public List<Item> Blocks { get; } = [];

    public bool IsR12 { get; private set; }

    public IReadOnlyList<Item> TableRecords(string table) => Tables.TryGetValue(table, out var records) ? records : [];

    public Item? LayerNamed(string name)
        => TableRecords("LAYER").FirstOrDefault(l => string.Equals(l.Text(2), name, StringComparison.OrdinalIgnoreCase));

    public static DxfTestReader Read(string text)
    {
        var reader = new DxfTestReader();
        reader.Parse(Pairs(text));
        reader.Check();
        return reader;
    }

    private static List<Group> Pairs(string text)
    {
        Assert.DoesNotContain("\n", text.Replace("\r\n", ""));   // \r\n line ends only
        string[] lines = text.Split("\r\n");
        if (lines[^1].Length == 0)
            lines = lines[..^1];
        if (lines.Length % 2 != 0)
            throw new InvalidDataException("An odd number of lines");
        var pairs = new List<Group>();
        for (int i = 0; i < lines.Length; i += 2)
        {
            if (!int.TryParse(lines[i].Trim(), NumberStyles.Integer, CultureInfo.InvariantCulture, out int code))
                throw new InvalidDataException($"Line {i + 1} isn't a group code: {lines[i]}");
            if (lines[i].Length != 3 && code < 1000)
                throw new InvalidDataException($"Line {i + 1}'s code isn't right-aligned in three characters: '{lines[i]}'");
            pairs.Add(new Group(code, lines[i + 1]));
        }
        return pairs;
    }

    // The records from a position up to the next 0 group
    private static Item ReadItem(List<Group> pairs, ref int i)
    {
        string type = pairs[i].Value;
        int start = ++i;
        while (i < pairs.Count && pairs[i].Code != 0)
            i++;
        return new Item(type, pairs.GetRange(start, i - start));
    }

    private void Parse(List<Group> pairs)
    {
        int i = 0;
        while (true)
        {
            if (i >= pairs.Count)
                throw new InvalidDataException("No EOF");
            var item = ReadItem(pairs, ref i);
            if (item.Type == "EOF")
            {
                if (i != pairs.Count)
                    throw new InvalidDataException("Something after EOF");
                return;
            }
            if (item.Type != "SECTION")
                throw new InvalidDataException($"{item.Type} outside a section");
            string section = item.Text(2) ?? throw new InvalidDataException("A section without a name");
            if (section == "HEADER")
            {
                // $NAME then its values, all in one item since none has code 0
                Item header = item;
                string? name = null;
                var values = new List<Group>();
                foreach (var g in header.Groups.Skip(1))
                {
                    if (g.Code == 9)
                    {
                        if (name is not null)
                            Header.Add(name, new Item(name, values));
                        name = g.Value;
                        values = [];
                    }
                    else
                        values.Add(g);
                }
                if (name is not null)
                    Header.Add(name, new Item(name, values));
            }
            while (true)
            {
                if (i >= pairs.Count)
                    throw new InvalidDataException($"Section {section} isn't closed");
                var record = ReadItem(pairs, ref i);
                if (record.Type == "ENDSEC")
                    break;
                switch (section)
                {
                    case "TABLES":
                        if (record.Type != "TABLE")
                            throw new InvalidDataException($"{record.Type} outside a table");
                        string table = record.Text(2)!;
                        var records = new List<Item>();
                        while (true)
                        {
                            if (i >= pairs.Count)
                                throw new InvalidDataException($"Table {table} isn't closed");
                            var r = ReadItem(pairs, ref i);
                            if (r.Type == "ENDTAB")
                                break;
                            if (r.Type != table)
                                throw new InvalidDataException($"{r.Type} in table {table}");
                            records.Add(r);
                        }
                        if ((int)record.Number(70) != records.Count)
                            throw new InvalidDataException($"Table {table} says {record.Number(70)} records and has {records.Count}");
                        Tables.Add(table, records);
                        TableHeads.Add(record);
                        break;
                    case "ENTITIES":
                        Entities.Add(record);
                        break;
                    case "OBJECTS":
                        Objects.Add(record);
                        break;
                    case "BLOCKS":
                        Blocks.Add(record);
                        break;
                    case "CLASSES":
                        if (record.Type != "CLASS")
                            throw new InvalidDataException($"{record.Type} in CLASSES");
                        break;
                    case "HEADER":
                        throw new InvalidDataException($"{record.Type} in the header");
                }
            }
        }
    }

    private List<Item> TableHeads { get; } = [];

    private void Check()
    {
        IsR12 = Header["$ACADVER"].Text(1) == "AC1009";
        // Polylines' vertices aren't entities of their own
        var entities = Entities.Where(e => e.Type is not ("VERTEX" or "SEQEND")).ToList();
        foreach (var entity in Entities)
        {
            if (LayerNamed(entity.Layer) is null)
                throw new InvalidDataException($"{entity.Type} is on layer {entity.Layer}, which isn't in the table");
        }
        foreach (var layer in TableRecords("LAYER"))
        {
            string type = layer.Text(6)!;
            if (!TableRecords("LTYPE").Any(t => string.Equals(t.Text(2), type, StringComparison.OrdinalIgnoreCase)))
                throw new InvalidDataException($"Layer {layer.Text(2)} has line type {type}, which isn't in the table");
        }
        if (IsR12)
        {
            if (Entities.Concat(Tables.Values.SelectMany(t => t)).Any(e => e.Text(5) is not null))
                throw new InvalidDataException("An R12 file with handles");
            return;
        }

        // Every handle once; every pointer to one that's there; the seed above them all
        var all = TableHeads.Concat(Tables.Values.SelectMany(t => t)).Concat(Blocks).Concat(Entities).Concat(Objects).ToList();
        var handles = new HashSet<long>();
        foreach (var item in all)
        {
            string? handle = item.Type == "DIMSTYLE" ? item.Text(105) : item.Text(5);
            if (handle is null)
                throw new InvalidDataException($"{item.Type} has no handle");
            if (!handles.Add(Convert.ToInt64(handle, 16)))
                throw new InvalidDataException($"Handle {handle} is used twice");
        }
        foreach (var item in all)
        {
            foreach (var g in item.Groups.Where(g => g.Code is 330 or 340 or 350 or 390))
            {
                long target = Convert.ToInt64(g.Value, 16);
                if (target != 0 && !handles.Contains(target))
                    throw new InvalidDataException($"{item.Type} points ({g.Code}) to {g.Value}, which isn't there");
            }
        }
        long seed = Convert.ToInt64(Header["$HANDSEED"].Text(5)!, 16);
        if (handles.Any(h => h >= seed))
            throw new InvalidDataException("$HANDSEED isn't above every handle");
        // The entities are in model space
        string model = TableRecords("BLOCK_RECORD").Single(r => r.Text(2) == "*Model_Space").Text(5)!;
        foreach (var entity in entities)
        {
            Assert.Equal(model, entity.Text(330));
        }
    }
}
