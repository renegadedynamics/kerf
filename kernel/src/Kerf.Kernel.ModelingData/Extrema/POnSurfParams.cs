// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_POnSurfParams.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;
using System.Globalization;
using static Kerf.Engine.Extrema.ElementType;

namespace Kerf.Engine.Extrema;

/// <summary>
/// A point on a surface with the data Extrema_GenExtPS needs to compute an initial approximation: the square distance
/// to the projected point, the kind of grid element it lies on and that element's indices.
/// </summary>
/// <remarks>
/// OCCT derives this class from Extrema_POnSurf; C# structs can't derive, so it embeds one (<see cref="POnSurf"/>,
/// C++'s slicing conversion) and forwards <see cref="Value"/>, <see cref="Parameter"/> and <c>SetParameters</c>.
/// <c>new Extrema_POnSurfParams()</c> has OCCT's defaults (an indefinite point, distance 0, a node at (0, 0));
/// <c>default</c> is all zeros. The setters are internal, with the twins <see cref="WithSqrDistance"/>,
/// <see cref="WithElementType"/>, <see cref="WithIndices"/> and the constructor.
/// </remarks>
internal partial struct POnSurfParams : IEquatable<POnSurfParams>
{
    private POnSurf _pOnSurf;
    private double _sqrDistance;
    private ElementType _elementType;
    private int _indexU;
    private int _indexV;

    /// <summary>An indefinite point, on a node (0, 0), at distance 0.</summary>
    public POnSurfParams()
    {
        _pOnSurf = new POnSurf();
        _sqrDistance = 0.0;
        _elementType = Node;
        _indexU = 0;
        _indexV = 0;
    }

    /// <summary>A point on surface with its parameter values on the surface.</summary>
    public POnSurfParams(double u, double v, in Point3D pnt)
    {
        _pOnSurf = new POnSurf(u, v, pnt);
        _sqrDistance = 0.0;
        _elementType = Node;
        _indexU = 0;
        _indexV = 0;
    }

    // Mutators (internal: see docs/translation-rules.md)

    /// <summary>Sets the parameters and the point (Extrema_POnSurf::SetParameters).</summary>
    internal void SetParameters(double u, double v, in Point3D pnt) => _pOnSurf.SetParameters(u, v, pnt);

    /// <summary>Sets the square distance from this point to another one (the point to be projected).</summary>
    internal void SetSqrDistance(double sqrDistance) => _sqrDistance = sqrDistance;

    /// <summary>Sets the element type on which this point is situated.</summary>
    internal void SetElementType(ElementType elementType) => _elementType = elementType;

    /// <summary>Sets the U and V indices of an element that contains this point.</summary>
    internal void SetIndices(int indexU, int indexV)
    {
        _indexU = indexU;
        _indexV = indexV;
    }

    // Twins

    public readonly POnSurfParams WithSqrDistance(double sqrDistance)
    {
        POnSurfParams copy = this;
        copy.SetSqrDistance(sqrDistance);
        return copy;
    }

    public readonly POnSurfParams WithElementType(ElementType elementType)
    {
        POnSurfParams copy = this;
        copy.SetElementType(elementType);
        return copy;
    }

    public readonly POnSurfParams WithIndices(int indexU, int indexV)
    {
        POnSurfParams copy = this;
        copy.SetIndices(indexU, indexV);
        return copy;
    }

    // Queries

    /// <summary>The point on surface alone (C++ converts by slicing).</summary>
    public readonly POnSurf POnSurf() => _pOnSurf;

    /// <summary>The 3D point.</summary>
    public readonly Point3D Value() => _pOnSurf.Value();

    /// <summary>The parameter values on the surface.</summary>
    public readonly void Parameter(out double u, out double v) => _pOnSurf.Parameter(out u, out v);

    /// <summary>The square distance from this point to another one.</summary>
    public readonly double GetSqrDistance() => _sqrDistance;

    /// <summary>The element type on which this point is situated.</summary>
    public readonly ElementType GetElementType() => _elementType;

    /// <summary>The U and V indices of an element that contains this point.</summary>
    public readonly void GetIndices(out int indexU, out int indexV)
    {
        indexU = _indexU;
        indexV = _indexV;
    }

    public readonly bool Equals(POnSurfParams other)
        => _pOnSurf.Equals(other._pOnSurf) && _sqrDistance.Equals(other._sqrDistance)
           && _elementType == other._elementType && _indexU == other._indexU && _indexV == other._indexV;

    public override readonly bool Equals(object? obj) => obj is POnSurfParams other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_pOnSurf, _sqrDistance, _elementType, _indexU, _indexV);

    public static bool operator ==(in POnSurfParams left, in POnSurfParams right) => left.Equals(right);

    public static bool operator !=(in POnSurfParams left, in POnSurfParams right) => !left.Equals(right);

    public override readonly string ToString()
        => string.Create(CultureInfo.InvariantCulture,
            $"Extrema_POnSurfParams({_pOnSurf}, {_sqrDistance:R}, {_elementType}, {_indexU}, {_indexV})");
}
