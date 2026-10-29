// Port of: src/ModelingAlgorithms/TKMesh/IMeshData/IMeshData_ParametersListArrayAdaptor.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Shows a parameters list (<see cref="IMeshData_Curve"/>, <see cref="IMeshData_PCurve"/>) as an array from 0 to
/// <c>ParametersNb() - 1</c>, for tools written over arrays (<see cref="BRepMesh_EdgeParameterProvider{ParametersCollection}"/>).
/// </summary>
/// <typeparam name="ParametersListPtrType">The list type: OCCT's template parameter is a handle to it.</typeparam>
public sealed class IMeshData_ParametersListArrayAdaptor<ParametersListPtrType> : Standard_Transient, INCollection_ArrayRead<double>
    where ParametersListPtrType : IMeshData_ParametersList
{
    private readonly ParametersListPtrType myParameters;

    /// <summary>An adaptor of the given list.</summary>
    public IMeshData_ParametersListArrayAdaptor(ParametersListPtrType theParameters)
    {
        myParameters = theParameters;
    }

    /// <summary>The lower index: 0.</summary>
    public int Lower() => 0;

    /// <summary>The upper index.</summary>
    public int Upper() => myParameters.ParametersNb() - 1;

    /// <summary>The parameter with the given index.</summary>
    public double Value(int theIndex) => myParameters.GetParameter(theIndex);
}
