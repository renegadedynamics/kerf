// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#if HAS_ORACLE || HAS_NATIVE
namespace OcctNet.Oracle.Tests;

/// <summary>Native objects made outside the facade (by build123d.NET on native OCCT, when recording), as the facade's.</summary>
internal static class RecordedNative
{
    public static N.TopoDS_Shape Adopt(Build123d.Occt.TopoDS_Shape theShape)
#if ORACLE_DIRECT || HAS_NATIVE
        => theShape;
#else
        => (N.TopoDS_Shape)OcctNative.Tape.Adopt(theShape)!;
#endif

    /// <summary>The native object a facade object stands for, when recording.</summary>
    public static T Real<T>(object theObject) where T : class
#if ORACLE_DIRECT || HAS_NATIVE
        => (T)theObject;
#else
        => (T)((OcctNative.NativeHandle)theObject).Real!;
#endif
}
#endif
