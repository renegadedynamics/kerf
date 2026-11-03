// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Concurrent;
using System.IO.Compression;
using System.Reflection;
using System.Text;
using Xunit.Sdk;

namespace OcctNative;

/// <summary>
/// Native OCCT, recorded (unit 10A, decision 1336). The oracle tests call native OCCT through <c>N.</c>, which names
/// this namespace: a facade with the members of build123d.NET's native wrappers the tests used (<c>Native.g.cs</c>,
/// generated in unit 10A and frozen since native OCCT went in unit 10B). Each call is a step on the running test's tape:
/// its results (values, the types of the objects it returns, its exception, the files it wrote) are read from the test's
/// recording, embedded in the assembly (<c>Recordings/*.tape.gz</c>), after checking that it is the call recorded, with
/// the same arguments. No native code runs, and the recordings can't be made again.
/// </summary>
/// <remarks>
/// The port's side always runs. A test therefore compares the port with native's results as recorded, exactly as it
/// compared them live; a port whose results change makes the test ask native different questions, which fails as a
/// call not recorded.
/// </remarks>
public static partial class Tape
{
    // the running test's session, in a holder: a theory's row (Row, called in the test) replaces it for the After hook,
    // which an AsyncLocal changed inside the test method wouldn't see
    private sealed class Holder
    {
        public Session? Value;
    }

    private static readonly AsyncLocal<Holder?> s_holder = new();

    private static Holder s_session => s_holder.Value ??= new Holder();
    private static readonly ConcurrentDictionary<string, Dictionary<string, byte[]>> s_files = new(StringComparer.Ordinal);

    /// <summary>The running test's session.</summary>
    internal static Session Current => s_holder.Value?.Value
                                      ?? throw new InvalidOperationException("A native call outside a test: native OCCT is recorded per test "
                                                                             + "(a static field or a constructor can't call it)");

    /// <summary>Starts a test's tape: its recording file is the test class's, its key the method's.</summary>
    internal static void Begin(MethodInfo theMethod)
    {
        Type aClass = theMethod.ReflectedType ?? theMethod.DeclaringType!;
        s_holder.Value = new Holder { Value = new Session(aClass.Name, theMethod.Name) };
    }

    /// <summary>
    /// Names the row of a theory (or another subdivision of a test) whose calls follow: each row has a tape of its own,
    /// so rows can run alone. Ends the previous row's tape.
    /// </summary>
    public static void Row(object theRow)
    {
        Session aSession = Current;
        aSession.Finish();
        s_session.Value = new Session(aSession.File, $"{aSession.BaseKey}[{theRow}]", aSession.BaseKey);
    }

    /// <summary>Ends the test's tape, checked used up.</summary>
    internal static void End(bool theCheck)
    {
        Session? aSession = s_session.Value;
        s_session.Value = null;
        if (aSession is not null)
        {
            aSession.Finish(theCheck);
        }
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Calls

    /// <summary>
    /// Replays one call: <paramref name="theSelf"/> is the object called (null for a static member or a constructor),
    /// <paramref name="theSignature"/> the member, <paramref name="theArgs"/> its arguments (null for an out parameter).
    /// Returns the results: the return value (if not void), then the out and ref parameters', each as the facade's own
    /// value (a facade object, an enum as a long).
    /// </summary>
    public static object?[] Call(NativeHandle? theSelf, string theSignature, params object?[] theArgs)
        => Current.Call(theSelf, theSignature, theArgs);

    /// <summary>A constructor's call: the object's number on the tape.</summary>
    public static Made New(string theSignature, params object?[] theArgs)
    {
        Session aSession = Current;
        aSession.Call(null, theSignature, theArgs);
        return new Made(aSession.NextId());
    }

    /// <summary>
    /// A value the test computed from native objects outside the facade (build123d.NET's scenarios, run on native OCCT
    /// when recorded), read back by <paramref name="theRead"/>; a facade object inside it comes from
    /// <see cref="TapeReader.Object{T}"/>.
    /// </summary>
    public static T Value<T>(string theWhat, Func<TapeReader, T> theRead) => Current.Value(theWhat, theRead);

    /// <summary>A recorded value in one of the tape's own types (string, numbers, arrays of them).</summary>
    public static T Value<T>(string theWhat) => Value(theWhat, r => (T)r.Value()!);

    /// <summary>
    /// A number of random cases: <paramref name="theLive"/> when native OCCT ran live (before unit 10A),
    /// <paramref name="theRecorded"/> as recorded, which is what runs (decision 1337).
    /// </summary>
    public static int Cases(int theLive, int theRecorded) => theRecorded;

    /// <summary>
    /// A loop's items, as recorded (decision 1337): when recording, the items were taken in turn while the loop's
    /// recording stayed under a budget of bytes (<paramref name="theBudget"/>); the replay takes the items recorded. In
    /// order and lazily (for items from a random generator, or that make native calls themselves), the loop stops at the
    /// first item not taken; with <paramref name="isSpread"/> (a corpus), the items are taken in a spread order (every
    /// seventh first, then the next...) so the sample reaches across the corpus, and those not taken are skipped.
    /// <paramref name="theWeight"/> and <paramref name="theMaxWeight"/> leave out, as recorded, the items too large to keep
    /// (a corpus file's size, say).
    /// </summary>
    public static IEnumerable<T> Budget<T>(IEnumerable<T> theItems, long theBudget = DefaultBudget, bool isSpread = false,
                                           Func<T, long>? theWeight = null, long theMaxWeight = long.MaxValue)
    {
        _ = theBudget;
        if (theWeight is not null)
        {
            theItems = theItems.Where(t => theWeight(t) <= theMaxWeight);
        }
        Session aSession = Current;
        IEnumerable<T> anOrder = isSpread
            ? theItems.Select((t, i) => (t, i)).ToList().OrderBy(p => p.i % 7).ThenBy(p => p.i).Select(p => p.t)
            : theItems;
        foreach (T anItem in anOrder)
        {
            if (aSession.Take())
            {
                yield return anItem;
            }
            else if (!isSpread)
            {
                yield break;
            }
        }
    }

    /// <summary>The recorded bytes a loop under <see cref="Budget{T}"/> took by default (uncompressed).</summary>
    public const long DefaultBudget = 750_000;

    /// <summary>A file's size, as a weight for <see cref="Budget{T}"/>.</summary>
    public static long FileSize(string thePath) => new FileInfo(thePath).Length;

    /// <summary>A loop of <paramref name="theCount"/> cases (0 to count - 1), in order, as recorded: see <see cref="Budget{T}"/>.</summary>
    public static IEnumerable<int> Budget(int theCount, long theBudget = DefaultBudget) => Budget(Enumerable.Range(0, theCount), theBudget);

    /// <summary>The results' converted forms: an array of facade objects, typed.</summary>
    public static T[] Array<T>(object? theValue) => theValue switch
    {
        null => null!,
        T[] aTyped => aTyped,
        object?[] anArray => [.. anArray.Select(o => (T)o!)],
        _ => throw new InvalidCastException($"Not an array of {typeof(T).Name}: {theValue.GetType().Name}"),
    };

    /// <summary>A recorded enum value as its type.</summary>
    public static T Enum<T>(object? theValue) where T : struct, System.Enum => (T)System.Enum.ToObject(typeof(T), Convert.ToInt64(theValue));

    /// <summary>A constructor's result: the object's number on the tape.</summary>
    public readonly record struct Made(int Id);

    // ---------------------------------------------------------------------------------------------------------------
    // Files

    /// <summary>The recordings of a test class (test key to tape), from the embedded resource.</summary>
    private static Dictionary<string, byte[]> Load(string theFile)
        => s_files.GetOrAdd(theFile, f =>
        {
            Stream? aStream = typeof(Tape).Assembly.GetManifestResourceStream($"Recordings.{f}.tape.gz");
            var aTapes = new Dictionary<string, byte[]>(StringComparer.Ordinal);
            if (aStream is null)
            {
                return aTapes;
            }
            using (aStream)
            {
                using var aGz = new GZipStream(aStream, CompressionMode.Decompress);
                using var aReader = new BinaryReader(aGz, Encoding.UTF8);
                if (aReader.ReadString() != Magic)
                {
                    throw new InvalidDataException($"{f}.tape.gz: not a tape file");
                }
                int aCount = aReader.ReadInt32();
                for (int i = 0; i < aCount; i++)
                {
                    string aKey = aReader.ReadString();
                    aTapes[aKey] = aReader.ReadBytes(aReader.ReadInt32());
                }
            }
            return aTapes;
        });

    private const string Magic = "occtnet-native-tape-1";

    // ---------------------------------------------------------------------------------------------------------------
    // A test's tape

    internal sealed class Session(string theFile, string theKey, string? theBaseKey = null)
    {
        public string File { get; } = theFile;
        public string Key { get; } = theKey;
        public string BaseKey { get; } = theBaseKey ?? theKey;
        private TapeReader? myReader;
        private readonly Lock myLock = new();
        private int myCalls;
        private int myIds;
        private uint myRunning = 2166136261;

        /// <summary>A budgeted loop's decision for an item, read back.</summary>
        public bool Take()
        {
            lock (myLock)
            {
                string? aStep = Reader.String();
                if (aStep != "take")
                {
                    throw new InvalidOperationException($"{File}.{Key}: a budgeted loop's item, where the recording has {aStep}");
                }
                return Reader.Byte() != 0;
            }
        }

        /// <summary>The next object's number.</summary>
        public int NextId() => Interlocked.Increment(ref myIds);

        private TapeReader Reader
        {
            get
            {
                if (myReader is null)
                {
                    if (!Load(File).TryGetValue(Key, out byte[]? aBytes))
                    {
                        throw new InvalidOperationException($"No recording of native OCCT for {File}.{Key}: the recordings can't be "
                                                            + "made again since native OCCT went (unit 10B; kernel/docs/oracle-testing.md)");
                    }
                    myReader = new TapeReader(aBytes);
                }
                return myReader;
            }
        }

        public void Finish(bool theCheck = true)
        {
            if (theCheck && myReader is not null && !myReader.AtEnd)
            {
                throw new InvalidOperationException($"{File}.{Key}: the test made {myCalls} native calls, fewer than recorded");
            }
        }

        public object?[] Call(NativeHandle? theSelf, string theSignature, object?[] theArgs)
        {
            lock (myLock)
            {
                myCalls++;
                // a running hash of every call's arguments, of which each step keeps a byte: a difference in arguments
                // shows at once, or a few calls later
                myRunning = (myRunning ^ ArgHash(theSelf, theArgs)) * 16777619;
                byte aCheck = (byte)(myRunning ^ (myRunning >> 8) ^ (myRunning >> 16) ^ (myRunning >> 24));
                return Replay(theSignature, theArgs, aCheck);
            }
        }

        // A step: the member, a header byte (bit 0: it threw; bit 1: it wrote files; the rest: how many results), the
        // arguments' check byte, then the results (tagged values) or the exception, then the files written.

        private object?[] Replay(string theSignature, object?[] theArgs, byte theCheck)
        {
            TapeReader aReader = Reader;
            if (aReader.AtEnd)
            {
                throw new InvalidOperationException($"{File}.{Key}: native call {myCalls} ({theSignature}) wasn't recorded: the test makes more calls than it did");
            }
            string aRecorded = aReader.String()!;
            byte aHeader = aReader.Byte();
            byte aCheck = aReader.Byte();
            if (aRecorded != theSignature || aCheck != theCheck)
            {
                throw new InvalidOperationException(
                    $"{File}.{Key}: native call {myCalls} is {theSignature}{(aRecorded == theSignature ? " with other arguments (here or a few calls before)" : "")}; "
                    + $"the recording has {aRecorded} here. The port's results feed native's arguments, or the test changed: compare "
                    + "differently (phase10-plan.md 3.2), as native OCCT can't be recorded again");
            }
            if ((aHeader & 1) != 0)
            {
                throw aReader.Exception();
            }
            var aResults = new object?[aHeader >> 2];
            for (int i = 0; i < aResults.Length; i++)
            {
                aResults[i] = aReader.Value();
            }
            if ((aHeader & 2) != 0)
            {
                int aFiles = aReader.Int();
                for (int i = 0; i < aFiles; i++)
                {
                    int anArg = aReader.Int();
                    byte[] aBytes = (byte[])aReader.Value()!;
                    System.IO.File.WriteAllBytes((string)theArgs[anArg]!, aBytes);
                }
            }
            return aResults;
        }

        /// <summary>A recorded value.</summary>
        public T Value<T>(string theWhat, Func<TapeReader, T> theRead)
        {
            lock (myLock)
            {
                myCalls++;
                string aRecorded = Reader.String()!;
                if (aRecorded != "value " + theWhat)
                {
                    throw new InvalidOperationException($"{File}.{Key}: step {myCalls} is the value {theWhat}; the recording has {aRecorded} here");
                }
                string? anEnd = Reader.String();
                if (anEnd != "value end " + theWhat)
                {
                    throw new InvalidOperationException($"{File}.{Key}: the value {theWhat} isn't where the recording has it ({anEnd})");
                }
                return theRead(Reader);
            }
        }

        /// <summary>
        /// The arguments' hash: numbers to the bit, strings by text, rooted paths by their file's content (temporary names
        /// change from run to run), facade objects by their number on the tape.
        /// </summary>
        private static uint ArgHash(NativeHandle? theSelf, object?[] theArgs)
        {
            var aHash = new Fnv();
            aHash.Add(theSelf?.Id ?? 0);
            foreach (object? anArg in theArgs)
            {
                Add(ref aHash, anArg);
            }
            return aHash.Value;
        }

        private static void Add(ref Fnv theHash, object? theArg)
        {
            switch (theArg)
            {
                case null:
                    theHash.Add(-1L);
                    break;
                case NativeHandle aHandle:
                    theHash.Add(aHandle.Id);
                    break;
                case string aText when IsPath(aText):
                    theHash.Add(System.IO.File.Exists(aText) ? "file " + Convert.ToHexString(System.Security.Cryptography.SHA256.HashData(FileContent(aText)))
                                : Directory.Exists(aText) ? "directory" : "no file");
                    break;
                case string aText:
                    theHash.Add(aText);
                    break;
                case bool b:
                    theHash.Add(b ? 1L : 0L);
                    break;
                case double d:
                    theHash.Add(BitConverter.DoubleToInt64Bits(d));
                    break;
                case float f:
                    theHash.Add(BitConverter.SingleToInt32Bits(f));
                    break;
                case System.Enum e:
                    theHash.Add(Convert.ToInt64(e));
                    break;
                case int or long or ulong or byte or short:
                    theHash.Add(Convert.ToInt64(theArg));
                    break;
                case System.Collections.IEnumerable aSequence:
                    long aCount = 0;
                    foreach (object? anItem in aSequence)
                    {
                        Add(ref theHash, anItem);
                        aCount++;
                    }
                    theHash.Add(aCount);
                    break;
                default:
                    theHash.Add(theArg.ToString() ?? "");
                    break;
            }
        }
    }

    /// <summary>A file's content as an argument: a STEP file's FILE_NAME (its time stamp) left out.</summary>
    private static byte[] FileContent(string thePath)
    {
        byte[] aBytes = System.IO.File.ReadAllBytes(thePath);
        if (aBytes.Length > 12 && Encoding.ASCII.GetString(aBytes, 0, 12) == "ISO-10303-21")
        {
            string aText = Encoding.Latin1.GetString(aBytes);
            return Encoding.Latin1.GetBytes(System.Text.RegularExpressions.Regex.Replace(aText, @"FILE_NAME\((?s:.*?)\);", "FILE_NAME();"));
        }
        return aBytes;
    }

    /// <summary>Whether a string argument names a file: rooted, with a separator.</summary>
    private static bool IsPath(string theText)
        => theText.Length > 3 && theText.Length < 400 && theText.IndexOfAny(['\n', '\r']) < 0 && Path.IsPathFullyQualified(theText);

    private struct Fnv()
    {
        public uint Value = 2166136261;

        public void Add(long theValue)
        {
            for (int i = 0; i < 8; i++)
            {
                Value = (Value ^ (byte)(theValue >> (8 * i))) * 16777619;
            }
        }

        public void Add(string theText)
        {
            foreach (char c in theText)
            {
                Value = (Value ^ c) * 16777619;
            }
            Add(theText.Length);
        }
    }
}

/// <summary>The base of the facade's objects: its number on the tape (its identity in arguments).</summary>
public abstract class NativeHandle : IDisposable
{
    protected NativeHandle(Tape.Made theMade) => Id = theMade.Id;

    /// <summary>The object's number on its test's tape.</summary>
    public int Id { get; }

    /// <summary>Nothing to release: the tests dispose the facade's objects as they disposed native ones.</summary>
    public void Dispose() => GC.SuppressFinalize(this);
}

/// <summary>An exception native OCCT threw (Build123d.Occt's OcctException), replayed.</summary>
public sealed class OcctException(string theOcctType, string theOcctMessage, string? theOperation, string theMessage) : Exception(theMessage)
{
    /// <summary>As the test raises one itself (Build123d.Occt's constructor).</summary>
    public OcctException(string theOcctType, string theMessage)
        : this(theOcctType, theMessage, null, theMessage.Length > 0 ? $"{theOcctType}: {theMessage}" : theOcctType)
    {
    }

    public string OcctType { get; } = theOcctType;

    public string OcctMessage { get; } = theOcctMessage;

    public string? Operation { get; } = theOperation;

    public bool IsFault => OcctType.StartsWith("OSD_Exception", StringComparison.Ordinal) || OcctType == "unknown";
}

/// <summary>Reads a tape.</summary>
public sealed class TapeReader(byte[] theBytes)
{
    private int myPos;
    private readonly List<string> myStrings = [];

    internal bool AtEnd => myPos >= theBytes.Length;

    internal byte Byte() => theBytes[myPos++];

    internal uint UInt32()
    {
        uint aValue = BitConverter.ToUInt32(theBytes, myPos);
        myPos += 4;
        return aValue;
    }

    public long Long()
    {
        ulong aValue = 0;
        for (int aShift = 0; ; aShift += 7)
        {
            byte b = theBytes[myPos++];
            aValue |= (ulong)(b & 0x7F) << aShift;
            if (b < 0x80)
            {
                break;
            }
        }
        return (long)(aValue >> 1) ^ -(long)(aValue & 1);
    }

    public int Int() => (int)Long();

    public string? String()
    {
        int aTag = Int();
        if (aTag == 0)
        {
            return null;
        }
        if (aTag >= 2)
        {
            return myStrings[aTag - 2];
        }
        int aLength = Int();
        string aText = Encoding.UTF8.GetString(theBytes, myPos, aLength);
        myPos += aLength;
        myStrings.Add(aText);
        return aText;
    }

    private T[] Raw<T>() where T : unmanaged
    {
        int aCount = Int();
        var aValues = new T[aCount];
        int aBytes = aCount * System.Runtime.CompilerServices.Unsafe.SizeOf<T>();
        theBytes.AsSpan(myPos, aBytes).CopyTo(System.Runtime.InteropServices.MemoryMarshal.AsBytes(aValues.AsSpan()));
        myPos += aBytes;
        return aValues;
    }

    public object? Value()
    {
        byte aTag = Byte();
        return aTag switch
        {
            0 => null,
            1 => Byte() != 0,
            2 => Int(),
            3 => Long(),
            4 => Raw<ulong>()[0],
            5 => Raw<double>()[0],
            6 => Raw<float>()[0],
            7 => String(),
            8 => Facade.Create(String()!, Tape.Current.NextId()),
            9 => Raw<byte>(),
            10 => Raw<int>(),
            11 => Raw<long>(),
            12 => Raw<double>(),
            13 => Raw<float>(),
            14 => Enumerable.Range(0, Int()).Select(_ => Value()).ToArray(),
            15 => Byte(),
            _ => throw new InvalidDataException($"A bad tag {aTag} on the tape"),
        };
    }

    /// <summary>A facade object (or null), as recorded.</summary>
    public T? Object<T>() where T : class => (T?)Value();

    internal Exception Exception()
    {
        string aType = String()!;
        string aMessage = String()!;
        string anOcctType = String()!;
        string anOcctMessage = String()!;
        string? anOperation = String();
        return Make(aType, aMessage, anOcctType, anOcctMessage, anOperation);
    }

    internal static Exception Make(string theType, string theMessage, string theOcctType, string theOcctMessage, string? theOperation)
    {
        if (theType == "Build123d.Occt.OcctException")
        {
            return new OcctException(theOcctType, theOcctMessage, theOperation, theMessage);
        }
        Type? aType = typeof(object).Assembly.GetType(theType);
        if (aType is not null && typeof(Exception).IsAssignableFrom(aType)
            && aType.GetConstructor([typeof(string)]) is { } aCtor)
        {
            return (Exception)aCtor.Invoke([theMessage]);
        }
        return new InvalidOperationException($"{theType}: {theMessage}");
    }
}

/// <summary>Starts and ends each test's tape (<see cref="Tape"/>).</summary>
[AttributeUsage(AttributeTargets.Assembly | AttributeTargets.Class | AttributeTargets.Method)]
public sealed class NativeTapeAttribute : BeforeAfterTestAttribute
{
    public override void Before(MethodInfo methodUnderTest) => Tape.Begin(methodUnderTest);

    public override void After(MethodInfo methodUnderTest) => Tape.End(true);
}
