using System.Runtime.InteropServices;

namespace Build123d.Occt;

/// <summary>An exception thrown by OCCT, such as Standard_ConstructionError or StdFail_NotDone.</summary>
public sealed partial class OcctException : Exception
{
    public OcctException(string occtType, string message)
        : base(message.Length > 0 ? $"{occtType}: {message}" : occtType)
    {
        OcctType = occtType;
    }

    /// <summary>The OCCT exception class name.</summary>
    public string OcctType { get; }

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
