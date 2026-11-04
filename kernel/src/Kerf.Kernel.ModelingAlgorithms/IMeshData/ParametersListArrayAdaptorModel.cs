// Port of: src/ModelingAlgorithms/TKMesh/IMeshData/IMeshData_ParametersListArrayAdaptor.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Meshing.Model;

/// <summary>
/// Shows a parameters list (<see cref="CurveModel"/>, <see cref="PCurveModel"/>) as an array from 0 to
/// <c>ParametersNb() - 1</c>, for tools written over arrays (<see cref="EdgeParameterProvider{ParametersCollection}"/>).
/// </summary>
/// <typeparam name="TParametersListPtrType">The list type: OCCT's template parameter is a handle to it.</typeparam>
internal sealed class ParametersListArrayAdaptorModel<TParametersListPtrType> : Transient, INCollection_ArrayRead<double>
    where TParametersListPtrType : ParametersListModel
{
    private readonly TParametersListPtrType _parameters;

    /// <summary>An adaptor of the given list.</summary>
    public ParametersListArrayAdaptorModel(TParametersListPtrType parameters)
    {
        _parameters = parameters;
    }

    /// <summary>The lower index: 0.</summary>
    public int Lower() => 0;

    /// <summary>The upper index.</summary>
    public int Upper() => _parameters.ParametersNb() - 1;

    /// <summary>The parameter with the given index.</summary>
    public double Value(int index) => _parameters.GetParameter(index);
}
