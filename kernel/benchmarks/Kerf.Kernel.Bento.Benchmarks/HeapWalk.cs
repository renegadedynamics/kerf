// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Reflection;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;

// What a kept object holds (unit 11Q, sixth slice): walks the instance fields from some roots by reflection, and adds up each object's
// exact size from its method table, by type and by the field that first reached it. Statics aren't followed, so a total that matches
// the collector's count says nothing is held anywhere else. A walker that's given the objects already seen counts only what a new root adds.
internal sealed class HeapWalk
{
    private static readonly Dictionary<Type, FieldInfo[]> Fields = [];
    private static readonly Dictionary<Type, bool> HasReferences = [];

    private readonly HashSet<object> _seen = new(ReferenceEqualityComparer.Instance);
    private readonly Func<FieldInfo, bool>? _skip;

    public HeapWalk(Func<FieldInfo, bool>? skip = null) => _skip = skip;

    public long Bytes { get; private set; }

    public long Objects => _seen.Count;

    public Dictionary<string, (long Count, long Bytes)> ByType { get; } = [];

    public Dictionary<string, (long Count, long Bytes)> ByHolder { get; } = [];

    public IEnumerable<object> Seen => _seen;

    /// <summary>Adds what's reachable from the root and not seen yet, and returns its bytes.</summary>
    public long Add(object? root, string holder = "root")
    {
        long before = Bytes;
        var stack = new Stack<(object Item, string Holder)>();
        if (root is not null)
            stack.Push((root, holder));
        while (stack.TryPop(out var top))
        {
            object item = top.Item;
            Type type = item.GetType();
            if (IsOutside(type) || !_seen.Add(item))
                continue;
            long size = SizeOf(item);
            Bytes += size;
            Bump(ByType, Name(type), size);
            Bump(ByHolder, top.Holder, size);
            if (item is Array array)
            {
                Type element = type.GetElementType()!;
                if (!ContainsReferences(element))
                    continue;
                if (!element.IsValueType)
                {
                    foreach (object? child in array)
                    {
                        if (child is not null)
                            stack.Push((child, top.Holder));
                    }
                    continue;
                }
                foreach (object? boxed in array)
                    PushFields(boxed!, top.Holder, stack);
                continue;
            }
            PushFields(item, null, stack);
        }
        return Bytes - before;
    }

    private void PushFields(object item, string? holder, Stack<(object, string)> stack)
    {
        foreach (FieldInfo field in FieldsOf(item.GetType()))
        {
            if (_skip is not null && _skip(field))
                continue;
            object? value = field.GetValue(item);
            if (value is null)
                continue;
            string through = holder ?? $"{Name(field.DeclaringType!)}.{field.Name}";
            if (field.FieldType.IsValueType)
                PushFields(value, through, stack);
            else
                stack.Push((value, through));
        }
    }

    private static void Bump(Dictionary<string, (long Count, long Bytes)> map, string key, long size)
    {
        ref var slot = ref CollectionsMarshal.GetValueRefOrAddDefault(map, key, out _);
        slot = (slot.Count + 1, slot.Bytes + size);
    }

    private static bool IsOutside(Type type) =>
        typeof(MemberInfo).IsAssignableFrom(type) || typeof(Assembly).IsAssignableFrom(type) || typeof(Module).IsAssignableFrom(type)
        || typeof(Thread).IsAssignableFrom(type) || type.IsPointer;

    private static FieldInfo[] FieldsOf(Type type)
    {
        if (Fields.TryGetValue(type, out var known))
            return known;
        var list = new List<FieldInfo>();
        for (Type? t = type; t is not null; t = t.BaseType)
        {
            foreach (FieldInfo field in t.GetFields(BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.DeclaredOnly))
            {
                if (ContainsReferences(field.FieldType))
                    list.Add(field);
            }
        }
        return Fields[type] = [.. list];
    }

    private static bool ContainsReferences(Type type)
    {
        if (type.IsPointer || type.IsFunctionPointer || type == typeof(IntPtr) || type == typeof(UIntPtr))
            return false;
        if (!type.IsValueType)
            return true;
        if (type.IsPrimitive || type.IsEnum)
            return false;
        if (HasReferences.TryGetValue(type, out bool known))
            return known;
        HasReferences[type] = false;
        bool any = type.GetFields(BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic).Any(f => ContainsReferences(f.FieldType));
        return HasReferences[type] = any;
    }

    /// <summary>An object's exact size on the heap: its method table's base size, and its components if it has any.</summary>
    public static long SizeOf(object item)
    {
        IntPtr table = item.GetType().TypeHandle.Value;
        long size = Marshal.ReadInt32(table, 4);
        if (item is Array array)
            size += (ushort)Marshal.ReadInt16(table) * array.LongLength;
        else if (item is string text)
            size += 2L * text.Length;
        return (size + 7) & ~7L;
    }

    public static string Name(Type type)
    {
        if (type.IsArray)
            return Name(type.GetElementType()!) + "[]";
        if (!type.IsGenericType)
            return type.DeclaringType is { } outer ? Name(outer) + "+" + type.Name : type.Name;
        string name = type.Name;
        int tick = name.IndexOf('`');
        if (tick >= 0)
            name = name[..tick];
        var sb = new StringBuilder();
        if (type.DeclaringType is { IsGenericType: false } plain)
            sb.Append(Name(plain)).Append('+');
        return sb.Append(name).Append('<').Append(string.Join(",", type.GetGenericArguments().Select(Name))).Append('>').ToString();
    }
}

// Identical data stored more than once: gives each object an id from its type, its values and its children's ids, so objects with the same
// id hold the same data. What has an identity of its own (a TShape, by what an edge or face shares) is never the same as anything else.
internal sealed class SameData
{
    private readonly Dictionary<string, int> _ids = [];
    private readonly Dictionary<object, int> _idOf = new(ReferenceEqualityComparer.Instance);
    private readonly Func<Type, bool> _hasIdentity;
    private int _unique = int.MinValue;

    public SameData(Func<Type, bool> hasIdentity) => _hasIdentity = hasIdentity;

    /// <summary>For each type, the objects, the distinct ones among them, and the bytes that would go if each were held once.</summary>
    public List<(string Type, long Count, long Distinct, long Wasted)> Over(IEnumerable<object> objects)
    {
        var groups = new Dictionary<string, (long Count, HashSet<int> Ids, long Wasted)>();
        foreach (object item in objects)
        {
            string name = HeapWalk.Name(item.GetType());
            if (!groups.TryGetValue(name, out var group))
                group = (0, [], 0);
            int id = IdOf(item, 0);
            bool again = !group.Ids.Add(id);
            groups[name] = (group.Count + 1, group.Ids, group.Wasted + (again ? HeapWalk.SizeOf(item) : 0));
        }
        return [.. groups.Select(g => (g.Key, g.Value.Count, (long)g.Value.Ids.Count, g.Value.Wasted)).OrderByDescending(g => g.Item4)];
    }

    private int IdOf(object? item, int depth)
    {
        if (item is null)
            return 0;
        if (_idOf.TryGetValue(item, out int id))
            return id;
        Type type = item.GetType();
        if (depth > 200 || _hasIdentity(type) || typeof(MemberInfo).IsAssignableFrom(type) || item is Delegate)
            return _idOf[item] = _unique++;
        _idOf[item] = _unique++; // a cycle is unique
        var key = new StringBuilder(type.FullName);
        Append(key, item, type, depth);
        string text = key.ToString();
        if (!_ids.TryGetValue(text, out id))
            _ids[text] = id = _ids.Count + 1;
        return _idOf[item] = id;
    }

    private void Append(StringBuilder key, object item, Type type, int depth)
    {
        if (item is string text)
        {
            key.Append('|').Append(text);
            return;
        }
        if (item is Array array)
        {
            Type element = type.GetElementType()!;
            key.Append('#').Append(array.Length);
            if (element.IsPrimitive)
            {
                var bytes = new byte[Buffer.ByteLength(array)];
                Buffer.BlockCopy(array, 0, bytes, 0, bytes.Length);
                key.Append(Convert.ToHexString(System.Security.Cryptography.SHA1.HashData(bytes)));
                return;
            }
            foreach (object? child in array)
            {
                if (element.IsValueType)
                    Append(key, child!, element, depth);
                else
                    key.Append(',').Append(IdOf(child, depth + 1));
            }
            return;
        }
        for (Type? t = type; t is not null && t != typeof(object) && t != typeof(ValueType); t = t.BaseType)
        {
            foreach (FieldInfo field in t.GetFields(BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.DeclaredOnly))
            {
                if (field.FieldType.IsPointer || field.FieldType.IsFunctionPointer)
                    continue;
                object? value;
                try
                {
                    value = field.GetValue(item);
                }
                catch (FieldAccessException)
                {
                    continue;
                }
                if (field.FieldType.IsPrimitive || field.FieldType.IsEnum)
                    key.Append(';').Append(value);
                else if (field.FieldType.IsValueType)
                    Append(key.Append('{'), value!, field.FieldType, depth);
                else
                    key.Append(',').Append(IdOf(value, depth + 1));
            }
        }
    }
}

// Holds the largest arrays a run allocates: the runtime's allocation ticks, read in process. Anything over 100 kB always gets a tick.
internal sealed class LargeAllocations : System.Diagnostics.Tracing.EventListener
{
    private readonly object _gate = new();
    private readonly List<(string Type, long Size)> _ticks = [];

    protected override void OnEventSourceCreated(System.Diagnostics.Tracing.EventSource source)
    {
        if (source.Name == "Microsoft-Windows-DotNETRuntime")
            EnableEvents(source, System.Diagnostics.Tracing.EventLevel.Verbose, (System.Diagnostics.Tracing.EventKeywords)1);
    }

    protected override void OnEventWritten(System.Diagnostics.Tracing.EventWrittenEventArgs data)
    {
        if (data.EventName is null || !data.EventName.StartsWith("GCAllocationTick", StringComparison.Ordinal) || data.Payload is null || data.PayloadNames is null)
            return;
        int size = data.PayloadNames.IndexOf("ObjectSize"), type = data.PayloadNames.IndexOf("TypeName");
        if (size < 0 || type < 0)
            return;
        long bytes = Convert.ToInt64(data.Payload[size]);
        if (bytes >= 85000)
        {
            lock (_gate)
                _ticks.Add(((string?)data.Payload[type] ?? "?", bytes));
        }
    }

    public List<(string Type, long Count, long Largest, long Sum)> Report()
    {
        lock (_gate)
            return [.. _ticks.GroupBy(t => t.Type).Select(g => (g.Key, (long)g.Count(), g.Max(t => t.Size), g.Sum(t => t.Size))).OrderByDescending(g => g.Item4)];
    }
}
