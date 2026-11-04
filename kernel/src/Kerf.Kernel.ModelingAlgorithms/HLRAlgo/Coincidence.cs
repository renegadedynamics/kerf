// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_Coincidence.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.HiddenLines.Algorithm;

/// <summary>
/// The coincidence of an edge with the boundary of a face, for the hidden-line removal: in 2D, the face's edge and
/// the parameter on it; in 3D, the states before and after.
/// </summary>
/// <remarks>A C++ value class, so a struct; <c>new()</c> and <c>default</c> are the same. The setters are internal.</remarks>
internal partial struct Coincidence : IEquatable<Coincidence>
{
    private int _fe;
    private double _param;
    private Containment _stBef;
    private Containment _stAft;

    public Coincidence()
    {
        _fe = 0;
        _param = 0.0;
        _stBef = In;
        _stAft = In;
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void Set2D(int fe, double param)
    {
        _fe = fe;
        _param = param;
    }

    internal void SetState3D(Containment stbef, Containment staft)
    {
        _stBef = stbef;
        _stAft = staft;
    }

    // Queries

    public readonly void Value2D(out int fe, out double param)
    {
        fe = _fe;
        param = _param;
    }

    public readonly void State3D(out Containment stbef, out Containment staft)
    {
        stbef = _stBef;
        staft = _stAft;
    }

    public readonly Coincidence With2D(int fe, double param)
    {
        Coincidence copy = this;
        copy.Set2D(fe, param);
        return copy;
    }

    public readonly Coincidence WithState3D(Containment stbef, Containment staft)
    {
        Coincidence copy = this;
        copy.SetState3D(stbef, staft);
        return copy;
    }

    public readonly bool Equals(Coincidence other)
        => _fe == other._fe && _param.Equals(other._param) && _stBef == other._stBef
           && _stAft == other._stAft;

    public override readonly bool Equals(object? obj) => obj is Coincidence other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_fe, _param, _stBef, _stAft);

    public static bool operator ==(in Coincidence left, in Coincidence right)
        => left.Equals(right);

    public static bool operator !=(in Coincidence left, in Coincidence right)
        => !left.Equals(right);

    public override readonly string ToString()
        => $"{Constants.Format("HLRAlgo_Coincidence", _fe, _param)} {_stBef} {_stAft}";
}
