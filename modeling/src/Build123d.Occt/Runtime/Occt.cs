using System.Runtime.InteropServices;

namespace Build123d.Occt;

/// <summary>Information about the native OCCT build.</summary>
public static partial class Occt
{
    /// <summary>The OCCT version the shim was built against, e.g. "8.0.1".</summary>
    public static string Version => Marshal.PtrToStringUTF8(N.b3d_occt_version()) ?? "";

    private static partial class N
    {
        [LibraryImport("b3docct")]
        internal static partial IntPtr b3d_occt_version();
    }
}
