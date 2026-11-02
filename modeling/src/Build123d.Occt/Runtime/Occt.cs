using System.Runtime.InteropServices;

namespace Build123d.Occt;

/// <summary>Information about the native OCCT build.</summary>
public static partial class Occt
{
    /// <summary>The OCCT version the shim was built against, e.g. "8.0.1".</summary>
    public static string Version => Marshal.PtrToStringUTF8(N.b3d_occt_version()) ?? "";

    /// <summary>
    /// Makes the shim read <paramref name="address"/>: a deliberate fault, for testing that a fault in native code
    /// comes back as an <see cref="OcctException"/> rather than ending the process.
    /// </summary>
    public static int FaultForTesting(IntPtr address)
    {
        var r = N.b3d_selftest_read(address, out var err);
        OcctException.ThrowIfError(err);
        return r;
    }

    private static partial class N
    {
        [LibraryImport("b3docct")]
        internal static partial IntPtr b3d_occt_version();
        [LibraryImport("b3docct")]
        internal static partial int b3d_selftest_read(IntPtr address, out IntPtr err);
    }
}
