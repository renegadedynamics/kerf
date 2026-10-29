using System.Runtime.InteropServices;

namespace Build123d.Occt;

/// <summary>
/// An exception thrown by OCCT, such as Standard_ConstructionError or StdFail_NotDone. Its message starts with what
/// was being done when OCCT failed (see <see cref="OcctContext.Doing"/>), e.g. "fuse failed: StdFail_NotDone: …".
/// </summary>
public sealed partial class OcctException : Exception
{
    public OcctException(string occtType, string message)
        : this(occtType, message, OcctContext.Current)
    {
    }

    private OcctException(string occtType, string message, string? operation)
        : base(Compose(occtType, message, operation))
    {
        OcctType = occtType;
        OcctMessage = message;
        Operation = operation;
    }

    /// <summary>The OCCT exception class name.</summary>
    public string OcctType { get; }

    /// <summary>OCCT's own message, which may be empty.</summary>
    public string OcctMessage { get; }

    /// <summary>What was being done, outermost first ("extrude / fuse"), or null if nothing said.</summary>
    public string? Operation { get; }

    /// <summary>Whether OCCT reported a fault (an access violation, say) rather than a failure of its own.</summary>
    public bool IsFault => OcctType.StartsWith("OSD_Exception", StringComparison.Ordinal) || OcctType == "unknown";

    private static string Compose(string occtType, string message, string? operation)
    {
        var what = message.Length > 0 ? $"{occtType}: {message}" : occtType;
        return operation is null ? what : $"{operation} failed: {what}";
    }

    /// <summary>Throws the error a shim function reported, if any, and frees it.</summary>
    internal static void ThrowIfError(IntPtr err)
    {
        if (err == IntPtr.Zero)
            return;
        var type = Marshal.PtrToStringUTF8(N.b3d_error_type(err)) ?? "";
        var message = Marshal.PtrToStringUTF8(N.b3d_error_message(err)) ?? "";
        N.b3d_error_free(err);
        throw new OcctException(type, message);
    }

    private static partial class N
    {
        [LibraryImport("b3docct")]
        internal static partial IntPtr b3d_error_type(IntPtr e);
        [LibraryImport("b3docct")]
        internal static partial IntPtr b3d_error_message(IntPtr e);
        [LibraryImport("b3docct")]
        internal static partial void b3d_error_free(IntPtr e);
    }
}
