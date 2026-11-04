// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_SurfaceRangeSample.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_SurfaceRangeSample.lxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_SurfaceRangeSample.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2005-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Intersection.Tools;

/// <summary>A sample range of a surface: a curve range sample in U and one in V.</summary>
/// <remarks>
/// A struct, as OCCT's by-value class (its copy constructor and <c>Assign</c> are a plain copy). Equality is OCCT's
/// <c>IsEqual</c>; the hash replaces OCCT's <c>std::hash</c> specialisation.
/// </remarks>
internal partial struct SurfaceRangeSample : IEquatable<SurfaceRangeSample>
{
    private CurveRangeSample _rangeU;
    private CurveRangeSample _rangeV;

    public SurfaceRangeSample()
    {
        _rangeU = new CurveRangeSample();
        _rangeV = new CurveRangeSample();
    }

    public SurfaceRangeSample(int indexU, int depthU, int indexV, int depthV)
    {
        _rangeU = new CurveRangeSample();
        _rangeV = new CurveRangeSample();
        _rangeU.SetRangeIndex(indexU);
        _rangeU.SetDepth(depthU);
        _rangeV.SetRangeIndex(indexV);
        _rangeV.SetDepth(depthV);
    }

    public SurfaceRangeSample(in CurveRangeSample rangeU, in CurveRangeSample rangeV)
    {
        _rangeU = rangeU;
        _rangeV = rangeV;
    }

    internal void SetRanges(in CurveRangeSample rangeU, in CurveRangeSample rangeV)
    {
        _rangeU = rangeU;
        _rangeV = rangeV;
    }

    public readonly void GetRanges(out CurveRangeSample rangeU, out CurveRangeSample rangeV)
    {
        rangeU = _rangeU;
        rangeV = _rangeV;
    }

    internal void SetIndexes(int indexU, int indexV)
    {
        _rangeU.SetRangeIndex(indexU);
        _rangeV.SetRangeIndex(indexV);
    }

    public readonly void GetIndexes(out int indexU, out int indexV)
    {
        indexU = _rangeU.GetRangeIndex();
        indexV = _rangeV.GetRangeIndex();
    }

    public readonly void GetDepths(out int depthU, out int depthV)
    {
        depthU = _rangeU.GetDepth();
        depthV = _rangeV.GetDepth();
    }

    internal void SetSampleRangeU(in CurveRangeSample rangeSampleU) => _rangeU = rangeSampleU;

    public readonly CurveRangeSample GetSampleRangeU() => _rangeU;

    internal void SetSampleRangeV(in CurveRangeSample rangeSampleV) => _rangeV = rangeSampleV;

    public readonly CurveRangeSample GetSampleRangeV() => _rangeV;

    internal void SetIndexU(int indexU) => _rangeU.SetRangeIndex(indexU);

    public readonly int GetIndexU() => _rangeU.GetRangeIndex();

    internal void SetIndexV(int indexV) => _rangeV.SetRangeIndex(indexV);

    public readonly int GetIndexV() => _rangeV.GetRangeIndex();

    internal void SetDepthU(int depthU) => _rangeU.SetDepth(depthU);

    public readonly int GetDepthU() => _rangeU.GetDepth();

    internal void SetDepthV(int depthV) => _rangeV.SetDepth(depthV);

    public readonly int GetDepthV() => _rangeV.GetDepth();

    public readonly ParameterRange GetRangeU(double firstU, double lastU, int nbSampleU) =>
        _rangeU.GetRange(firstU, lastU, nbSampleU);

    public readonly ParameterRange GetRangeV(double firstV, double lastV, int nbSampleV) =>
        _rangeV.GetRange(firstV, lastV, nbSampleV);

    public readonly bool IsEqual(in SurfaceRangeSample other) =>
        _rangeU.IsEqual(other._rangeU) && _rangeV.IsEqual(other._rangeV);

    public readonly int GetRangeIndexUDeeper(int nbSampleU) => _rangeU.GetRangeIndexDeeper(nbSampleU);

    public readonly int GetRangeIndexVDeeper(int nbSampleV) => _rangeV.GetRangeIndexDeeper(nbSampleV);

    // ---- OCCT.NET additions

    public readonly SurfaceRangeSample WithRanges(in CurveRangeSample rangeU, in CurveRangeSample rangeV)
    {
        SurfaceRangeSample copy = this;
        copy.SetRanges(rangeU, rangeV);
        return copy;
    }

    public readonly SurfaceRangeSample WithIndexes(int indexU, int indexV)
    {
        SurfaceRangeSample copy = this;
        copy.SetIndexes(indexU, indexV);
        return copy;
    }

    public readonly SurfaceRangeSample WithIndexU(int indexU)
    {
        SurfaceRangeSample copy = this;
        copy.SetIndexU(indexU);
        return copy;
    }

    public readonly SurfaceRangeSample WithIndexV(int indexV)
    {
        SurfaceRangeSample copy = this;
        copy.SetIndexV(indexV);
        return copy;
    }

    public readonly SurfaceRangeSample WithDepthU(int depthU)
    {
        SurfaceRangeSample copy = this;
        copy.SetDepthU(depthU);
        return copy;
    }

    public readonly SurfaceRangeSample WithDepthV(int depthV)
    {
        SurfaceRangeSample copy = this;
        copy.SetDepthV(depthV);
        return copy;
    }

    public readonly SurfaceRangeSample WithSampleRangeU(in CurveRangeSample rangeSampleU)
    {
        SurfaceRangeSample copy = this;
        copy.SetSampleRangeU(rangeSampleU);
        return copy;
    }

    public readonly SurfaceRangeSample WithSampleRangeV(in CurveRangeSample rangeSampleV)
    {
        SurfaceRangeSample copy = this;
        copy.SetSampleRangeV(rangeSampleV);
        return copy;
    }

    public readonly bool Equals(SurfaceRangeSample other) => IsEqual(other);

    public override readonly bool Equals(object? obj) => obj is SurfaceRangeSample other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_rangeU, _rangeV);

    public static bool operator ==(SurfaceRangeSample left, SurfaceRangeSample right) => left.IsEqual(right);

    public static bool operator !=(SurfaceRangeSample left, SurfaceRangeSample right) => !left.IsEqual(right);

    public override readonly string ToString() => $"IntTools_SurfaceRangeSample(U {_rangeU}, V {_rangeV})";
}
