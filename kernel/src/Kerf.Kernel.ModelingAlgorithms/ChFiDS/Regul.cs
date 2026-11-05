// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_Regul.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_Regul.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Fillets.Data;

/// <summary>
/// A curve of the data structure and its two supports, faces or surfaces (by index): the edges whose continuity
/// the fillet builders set.
/// </summary>
/// <remarks>A C++ value class, so a struct; <c>new()</c> and <c>default</c> are the same (all zero).</remarks>
internal partial struct Regul : IEquatable<Regul>
{
    private int _icurv;
    private int _is1;
    private int _is2;

    public Regul()
    {
        _icurv = 0;
        _is1 = 0;
        _is2 = 0;
    }

    internal void SetCurve(int ic)
    {
        _icurv = Math.Abs(ic);
    }

    internal void SetS1(int is1, bool isFace = true)
    {
        if (isFace)
        {
            _is1 = Math.Abs(is1);
        }
        else
        {
            _is1 = -Math.Abs(is1);
        }
    }

    internal void SetS2(int is2, bool isFace = true)
    {
        if (isFace)
        {
            _is2 = Math.Abs(is2);
        }
        else
        {
            _is2 = -Math.Abs(is2);
        }
    }

    public readonly Regul WithCurve(int ic)
    {
        Regul copy = this;
        copy.SetCurve(ic);
        return copy;
    }

    public readonly Regul WithS1(int is1, bool isFace = true)
    {
        Regul copy = this;
        copy.SetS1(is1, isFace);
        return copy;
    }

    public readonly Regul WithS2(int is2, bool isFace = true)
    {
        Regul copy = this;
        copy.SetS2(is2, isFace);
        return copy;
    }

    /// <summary>True if the first support is a surface, not a face.</summary>
    public readonly bool IsSurface1
    {
        get
        {
            return (_is1 < 0);
        }
    }

    /// <summary>True if the second support is a surface, not a face.</summary>
    public readonly bool IsSurface2
    {
        get
        {
            return (_is2 < 0);
        }
    }

    public readonly int Curve()
    {
        return _icurv;
    }

    public readonly int S1()
    {
        return Math.Abs(_is1);
    }

    public readonly int S2()
    {
        return Math.Abs(_is2);
    }

    public readonly bool Equals(Regul other) => _icurv == other._icurv && _is1 == other._is1 && _is2 == other._is2;

    public override readonly bool Equals(object? obj) => obj is Regul other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_icurv, _is1, _is2);

    public static bool operator ==(in Regul left, in Regul right) => left.Equals(right);

    public static bool operator !=(in Regul left, in Regul right) => !left.Equals(right);

    public override readonly string ToString() => Constants.Format("ChFiDS_Regul", _icurv, _is1, _is2);
}
