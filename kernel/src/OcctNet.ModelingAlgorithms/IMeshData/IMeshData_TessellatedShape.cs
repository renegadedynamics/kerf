// Port of: src/ModelingAlgorithms/TKMesh/IMeshData/IMeshData_TessellatedShape.hxx
// Port of: src/ModelingAlgorithms/TKMesh/IMeshData/IMeshData_TessellatedShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;

namespace Occt;

/// <summary>A discrete shape with a deflection (<see cref="RealLast"/> until set).</summary>
public abstract class IMeshData_TessellatedShape : IMeshData_Shape
{
    private double myDeflection;

    /// <summary>An entity with a null shape.</summary>
    protected IMeshData_TessellatedShape()
    {
        myDeflection = RealLast();
    }

    /// <summary>An entity of the given shape.</summary>
    protected IMeshData_TessellatedShape(in Shape theShape) : base(theShape)
    {
        myDeflection = RealLast();
    }

    /// <summary>The deflection of the discrete model.</summary>
    public double GetDeflection() => myDeflection;

    /// <summary>Sets the deflection of the discrete model.</summary>
    public void SetDeflection(double theValue) => myDeflection = theValue;
}
