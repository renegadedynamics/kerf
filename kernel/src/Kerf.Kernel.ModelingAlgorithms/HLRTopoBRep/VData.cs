// Port of: src/ModelingAlgorithms/TKHLR/HLRTopoBRep/HLRTopoBRep_VData.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRTopoBRep/HLRTopoBRep_VData.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRTopoBRep/HLRTopoBRep_VData.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;
using Occt;

namespace Kerf.Engine.HiddenLines.Topology;

/// <summary>A vertex inserted on an edge, with its parameter on the edge.</summary>
/// <remarks>A C++ value class without mutators, so a readonly struct.</remarks>
internal readonly struct VData : IEquatable<VData>
{
    private readonly double _parameter;
    private readonly Shape _vertex;

    public VData()
    {
        _parameter = 0.0;
        _vertex = new Shape();
    }

    public VData(double p, in Shape v)
    {
        _parameter = p;
        _vertex = v;
    }

    public double Parameter() => _parameter;

    public Shape Vertex() => _vertex;

    public bool Equals(VData other) => _parameter.Equals(other._parameter) && _vertex.Equals(other._vertex);

    public override bool Equals(object? obj) => obj is VData other && Equals(other);

    public override int GetHashCode() => HashCode.Combine(_parameter, _vertex);

    public static bool operator ==(in VData left, in VData right) => left.Equals(right);

    public static bool operator !=(in VData left, in VData right) => !left.Equals(right);

    public override string ToString() => $"HLRTopoBRep_VData({_parameter})";
}
