// Port of: src/ModelingAlgorithms/TKMesh/IMeshData/IMeshData_ParametersList.hxx
// Port of: src/ModelingAlgorithms/TKMesh/IMeshData/IMeshData_ParametersList.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Kerf.Engine.Meshing.Model;

/// <summary>A list of parameters on a curve, indexed from zero.</summary>
internal abstract class ParametersListModel : Transient
{
    /// <summary>An empty list.</summary>
    protected ParametersListModel()
    {
    }

    /// <summary>The parameter with the given index, for reading and writing.</summary>
    /// <remarks>
    /// OCCT returns a reference into a <c>std::deque</c>; the port's is into a list, which an insertion or a removal
    /// moves, as a deque's insertion in the middle does: don't keep it across those.
    /// </remarks>
    public abstract ref double GetParameter(int index);

    /// <summary>The number of parameters.</summary>
    public abstract int ParametersNb();

    /// <summary>Clears the list, keeping the end points if <paramref name="isKeepEndPoints"/>.</summary>
    public abstract void Clear(bool isKeepEndPoints);

    /// <summary>Removes the parameter with the given index.</summary>
    protected abstract void RemoveParameter(int index);
}
