// Kerf: the duck-typed array interface of C++ templates over "an array" (Lower, Upper, Value), for generics.
// Not a port of an OCCT file.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// What a C++ template over "an array of values" calls: <c>Lower()</c>, <c>Upper()</c> and <c>Value(i)</c>. C# generics
/// need it as a constraint (<c>BRepMesh_EdgeParameterProvider&lt;ParametersCollection&gt;</c> is instantiated over
/// <see cref="NCollection_HArray1{T}"/> and over <c>IMeshData_ParametersListArrayAdaptor</c>).
/// </summary>
internal interface INCollection_ArrayRead<out T>
{
    /// <summary>The lower bound.</summary>
    int Lower();

    /// <summary>The upper bound.</summary>
    int Upper();

    /// <summary>The value at the index.</summary>
    T Value(int index);
}
