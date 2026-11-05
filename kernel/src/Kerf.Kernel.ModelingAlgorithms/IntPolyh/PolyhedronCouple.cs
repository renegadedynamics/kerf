// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_Couple.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_Couple.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Intersection.Polyhedra;

/// <summary>
/// A couple of indices with an analyzed flag and an angle. In IntPolyh_MaillageAffinage, a couple of interfering
/// triangles with the cosine of the angle between them.
/// </summary>
/// <remarks>
/// OCCT's default couple is (-1, -1), not analyzed, angle -2; <c>default(IntPolyh_Couple)</c> is all zeros, so ported
/// code writes <c>new IntPolyh_Couple()</c>. <see cref="Equals(PolyhedronCouple)"/> and <c>==</c> compare the fields
/// exactly; OCCT's <c>operator==</c> (the same two indices in either order) is <see cref="IsEqual"/>, and maps in ported
/// code take <see cref="Hasher"/>.
/// </remarks>
internal partial struct PolyhedronCouple : IEquatable<PolyhedronCouple>
{
    private int _index1;
    private int _index2;
    private bool _analyzed;
    private double _angle;

    public PolyhedronCouple()
    {
        _index1 = -1;
        _index2 = -1;
        _analyzed = false;
        _angle = -2.0;
    }

    public PolyhedronCouple(int triangle1, int triangle2, double angle = -2.0)
    {
        _index1 = triangle1;
        _index2 = triangle2;
        _analyzed = false;
        _angle = angle;
    }

    public readonly int FirstValue() => _index1;

    public readonly int SecondValue() => _index2;

    /// <summary>True if the couple has been analyzed.</summary>
    public readonly bool IsAnalyzed => _analyzed;

    public readonly double Angle() => _angle;

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetCoupleValue(int ind1, int ind2)
    {
        _index1 = ind1;
        _index2 = ind2;
    }

    internal void SetAnalyzed(bool analyzed) => _analyzed = analyzed;

    internal void SetAngle(double angle) => _angle = angle;

    public readonly PolyhedronCouple WithCoupleValue(int ind1, int ind2)
    {
        PolyhedronCouple copy = this;
        copy.SetCoupleValue(ind1, ind2);
        return copy;
    }

    public readonly PolyhedronCouple WithAnalyzed(bool analyzed)
    {
        PolyhedronCouple copy = this;
        copy.SetAnalyzed(analyzed);
        return copy;
    }

    public readonly PolyhedronCouple WithAngle(double angle)
    {
        PolyhedronCouple copy = this;
        copy.SetAngle(angle);
        return copy;
    }

    /// <summary>OCCT's equality: the same two indices, in either order.</summary>
    public readonly bool IsEqual(in PolyhedronCouple other)
        => (_index1 == other._index1 && _index2 == other._index2)
           || (_index1 == other._index2 && _index2 == other._index1);

    /// <summary>
    /// OCCT's <c>std::hash</c> and <c>operator==</c>: <see cref="IsEqual"/>, and a hash of the ordered pair (OCCT hashes
    /// its bytes; the value differs, which IntPolyh never iterates on).
    /// </summary>
    public sealed class Hasher : IEqualityComparer<PolyhedronCouple>
    {
        public static readonly Hasher Instance = new();

        public bool Equals(PolyhedronCouple c1, PolyhedronCouple c2) => c1.IsEqual(c2);

        public int GetHashCode(PolyhedronCouple couple)
        {
            int a0 = couple.FirstValue(), a1 = couple.SecondValue();
            return a0 > a1 ? HashCode.Combine(a1, a0) : HashCode.Combine(a0, a1);
        }
    }

    /// <summary>Writes the couple with its index, as OCCT's <c>Dump(i)</c> prints it.</summary>
    public readonly void Dump(int i, TextWriter stream)
    {
        stream.Write($"\nCouple({Format.D(i, 3)}) : {Format.D(_index1, 5)} "
                        + $"{Format.D(_index2, 5)} {Format.D(_analyzed ? 1 : 0, 3)} "
                        + $"{Format.F(_angle, 5)}\n");
    }

    public readonly bool Equals(PolyhedronCouple other)
        => _index1 == other._index1 && _index2 == other._index2 && _analyzed == other._analyzed
           && _angle.Equals(other._angle);

    public override readonly bool Equals(object? obj) => obj is PolyhedronCouple other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_index1, _index2, _analyzed, _angle);

    public static bool operator ==(in PolyhedronCouple left, in PolyhedronCouple right) => left.Equals(right);

    public static bool operator !=(in PolyhedronCouple left, in PolyhedronCouple right) => !left.Equals(right);

    public override readonly string ToString()
        => $"IntPolyh_Couple({_index1}, {_index2}, {(_analyzed ? "analyzed" : "not analyzed")}, {Constants.Format("angle", _angle)})";
}
