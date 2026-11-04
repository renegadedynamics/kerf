using Kerf.Topology;
using System.Diagnostics;
using System.Diagnostics.CodeAnalysis;
using System.Reflection;
using System.Text;

namespace Kerf.Modeling;

/// <summary>
/// The differential log (OCCT.NET's unit 9D2): with the environment variable <c>BUILD123D_SHAPE_LOG</c> naming a folder,
/// every new kernel shape a <see cref="Shape"/> wraps is written there in OCCT's BRep text, so that a run of the test
/// suite can be compared op by op with native OCCT's log, recorded (OCCT.NET's unit 10A). Off (one static field read) unless the variable is set.
/// </summary>
/// <remarks>
/// <para>
/// A shape is new when its TShape isn't one already logged, or a sub-shape of one, in the same test: extracting faces,
/// relocating and re-wrapping aren't logged, the results of operations are. Empty compounds aren't logged.
/// </para>
/// <para>
/// The running test is found on the stack: the outermost frame in an assembly whose name ends in "Tests", with lambdas
/// and async state machines named after the method that holds them. Each test gets a file of its own,
/// <c>&lt;Class.Method&gt;.brlog</c>, of entries <c>@@ seq class type bytes api inner points</c>, the BRep text of
/// that many bytes, then a newline: <c>api</c> is the outermost build123d call under the test, <c>inner</c> the
/// innermost one that isn't a wrapper's plumbing, <c>points</c> a digest of the exact bits of its vertices' points. Shapes made outside a test go to <c>(none).brlog</c>.
/// </para>
/// <para>
/// The registry of a test is dropped when another test starts, so the suite must run its tests one at a time
/// (<c>dotnet test -- xUnit.ParallelizeTestCollections=false</c>). Logging changes nothing in the shapes: writing BRep
/// text reads them only, and the registry keeps references, not marks.
/// </para>
/// </remarks>
internal static class ShapeLog
{
    /// <summary>The log folder, or null when logging is off.</summary>
    internal static readonly string? Folder = Environment.GetEnvironmentVariable("BUILD123D_SHAPE_LOG") is { Length: > 0 } f ? f : null;

    private static readonly Lock s_lock = new();
    private static string? s_test;
    private static int s_seq;
    // The TShapes seen in the current test, by the hash of the shape without its location; IsPartner settles collisions
    private static readonly Dictionary<ulong, List<global::Kerf.Topology.Shape>> Seen = [];

    /// <summary>Logs <paramref name="wrapped"/> if it is new in the running test.</summary>
    internal static void Record(Shape shape, global::Kerf.Topology.Shape wrapped)
    {
        try
        {
            if (wrapped.IsNull || TopoUtil.IsEmptyCompound(wrapped))
                return;
            var (test, api, inner) = Callers();
            lock (s_lock)
            {
                if (test != s_test)
                {
                    s_test = test;
                    s_seq = 0;
                    Seen.Clear();
                }
                if (!Register(wrapped))
                    return;
                var text = Extras.WriteBRep(wrapped);
                var header = $"@@ {++s_seq} {shape.GetType().Name} {"TopAbs_" + wrapped.Type.ToString().ToUpperInvariant()} {text.Length} {api} {inner} {ExactPoints(wrapped)}\n";
                Directory.CreateDirectory(Folder!);
                using var file = new FileStream(Path.Combine(Folder!, FileName(test) + ".brlog"), FileMode.Append, FileAccess.Write);
                file.Write(Encoding.ASCII.GetBytes(header));
                file.Write(text);
                file.WriteByte((byte)'\n');
            }
        }
        catch (Exception e) when (e is not OutOfMemoryException)
        {
            // Logging must never change what the library does
        }
    }

    /// <summary>
    /// A digest of the exact bits of the shape's vertex points, in exploration order: BRep text has 15 digits, which
    /// can hide a difference of an ulp that a later operation shows.
    /// </summary>
    private static string ExactPoints(global::Kerf.Topology.Shape shape)
    {
        var hash = 14695981039346656037UL;
        foreach (var vertex in shape.Explore<global::Kerf.Topology.Vertex>())
        {
            var point = vertex.Point;
            foreach (var c in (ReadOnlySpan<double>)[point.X, point.Y, point.Z])
                hash = (hash ^ (ulong)BitConverter.DoubleToInt64Bits(c)) * 1099511628211UL;
        }
        return hash.ToString("x16", System.Globalization.CultureInfo.InvariantCulture);
    }

    /// <summary>Registers the TShape and its sub-shapes; false if the TShape was registered already.</summary>
    private static bool Register(global::Kerf.Topology.Shape shape)
    {
        if (!Add(shape))
            return false;
        var stack = new Stack<global::Kerf.Topology.Shape>([shape]);
        while (stack.Count > 0)
        {
            var current = stack.Pop();
            foreach (var child in current.Children)
            {
                if (Add(child))
                    stack.Push(child);
            }
            GC.KeepAlive(current);
        }
        return true;
    }

    private static bool Add(global::Kerf.Topology.Shape shape)
    {
        var identity = new Placement();
        var bare = shape.Located(identity);
        var key = bare.StdHash();
        if (!Seen.TryGetValue(key, out var bucket))
            Seen[key] = bucket = [];
        foreach (var seen in bucket)
            if (seen.IsPartner(bare))
                return false;
        bucket.Add(bare);
        return true;
    }

    private static readonly Assembly Library = typeof(ShapeLog).Assembly;

    // Plumbing that wraps a kernel shape rather than making one
    private static readonly HashSet<string> s_plumbing =
        [".ctor", "Cast", "CastLike", "Rewrapped", "MadeBy", "Result", "Placed", "Clone", "MakeZero", "Record"];

    /// <summary>The running test, and the outermost and innermost build123d calls under it.</summary>
    [UnconditionalSuppressMessage("Trimming", "IL2026", Justification = "Diagnostics only, off unless BUILD123D_SHAPE_LOG is set")]
    private static (string Test, string Api, string Inner) Callers()
    {
        var frames = new StackTrace(2, false).GetFrames();
        string? test = null, api = null, inner = null;
        foreach (var frame in frames)
        {
            var method = frame.GetMethod();
            var type = method?.DeclaringType;
            if (method is null || type is null)
                continue;
            var assembly = type.Assembly;
            if (assembly == Library)
            {
                var name = Name(method);
                if (inner is null && !s_plumbing.Contains(name.Method))
                    inner = name.Full;
                api = name.Full;
            }
            else if (assembly.GetName().Name?.EndsWith("Tests", StringComparison.Ordinal) == true)
                test = Name(method).Full;
        }
        return (test ?? "(none)", api ?? "-", inner ?? api ?? "-");
    }

    /// <summary>A method's name, with lambdas, local functions and async state machines named after their holder.</summary>
    private static (string Full, string Method) Name(MethodBase method)
    {
        var type = method.DeclaringType!;
        var name = method.Name;
        // <Holder>b__0_1, <Holder>g__Local|0_0 (lambdas, local functions); MoveNext of <Holder>d__3 (async, iterators)
        if (name.StartsWith('<'))
            name = name[1..name.IndexOf('>')];
        while (type.Name.StartsWith('<'))
        {
            // a closure class (<>c__DisplayClass) leaves the method's own name; a state machine names its method
            if (method.Name == "MoveNext" && !type.Name.StartsWith("<>", StringComparison.Ordinal))
                name = type.Name[1..type.Name.IndexOf('>')];
            type = type.DeclaringType!;
        }
        return ($"{type.Name}.{name}", name);
    }

    private static string FileName(string test)
    {
        var sb = new StringBuilder(test.Length);
        foreach (var c in test)
            sb.Append(char.IsLetterOrDigit(c) || c is '.' or '_' or '(' or ')' or '-' ? c : '_');
        return sb.ToString();
    }
}
