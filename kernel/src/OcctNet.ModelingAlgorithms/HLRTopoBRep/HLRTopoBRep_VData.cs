// Port of: src/ModelingAlgorithms/TKHLR/HLRTopoBRep/HLRTopoBRep_VData.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRTopoBRep/HLRTopoBRep_VData.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRTopoBRep/HLRTopoBRep_VData.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A vertex inserted on an edge, with its parameter on the edge.</summary>
/// <remarks>A C++ value class without mutators, so a readonly struct.</remarks>
public readonly struct HLRTopoBRep_VData : IEquatable<HLRTopoBRep_VData>
{
    private readonly double myParameter;
    private readonly TopoDS_Shape myVertex;

    public HLRTopoBRep_VData()
    {
        myParameter = 0.0;
        myVertex = new TopoDS_Shape();
    }

    public HLRTopoBRep_VData(double P, in TopoDS_Shape V)
    {
        myParameter = P;
        myVertex = V;
    }

    public double Parameter() => myParameter;

    public TopoDS_Shape Vertex() => myVertex;

    public bool Equals(HLRTopoBRep_VData other) => myParameter.Equals(other.myParameter) && myVertex.Equals(other.myVertex);

    public override bool Equals(object? obj) => obj is HLRTopoBRep_VData other && Equals(other);

    public override int GetHashCode() => HashCode.Combine(myParameter, myVertex);

    public static bool operator ==(in HLRTopoBRep_VData theLeft, in HLRTopoBRep_VData theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in HLRTopoBRep_VData theLeft, in HLRTopoBRep_VData theRight) => !theLeft.Equals(theRight);

    public override string ToString() => $"HLRTopoBRep_VData({myParameter})";
}
