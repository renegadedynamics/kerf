// Port of: src/ModelingAlgorithms/TKMesh/IMeshData/IMeshData_TessellatedShape.hxx
// Port of: src/ModelingAlgorithms/TKMesh/IMeshData/IMeshData_TessellatedShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Meshing.Model;

/// <summary>A discrete shape with a deflection (<see cref="RealLast"/> until set).</summary>
internal abstract class TessellatedShapeModel : ShapeModel
{
    private double _deflection;

    /// <summary>An entity with a null shape.</summary>
    protected TessellatedShapeModel()
    {
        _deflection = RealLast();
    }

    /// <summary>An entity of the given shape.</summary>
    protected TessellatedShapeModel(in Shape shape) : base(shape)
    {
        _deflection = RealLast();
    }

    /// <summary>The deflection of the discrete model.</summary>
    public double GetDeflection() => _deflection;

    /// <summary>Sets the deflection of the discrete model.</summary>
    public void SetDeflection(double value) => _deflection = value;
}
