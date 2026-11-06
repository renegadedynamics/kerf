// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Fixed.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Fixed.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>A constant trihedron law: the same tangent, normal and binormal at every parameter.</summary>
internal class Fixed : TrihedronLaw
{
    private Vector3D _t;
    private Vector3D _n;
    private Vector3D _b;

    /// <summary>The law of the trihedron (Tangent, Normal, Tangent ^ Normal), normalized.</summary>
    /// <exception cref="ConstructionErrorException">The vectors are parallel (within 0.01 rad).</exception>
    public Fixed(in Vector3D tangent, in Vector3D normal)
    {
        if (tangent.IsParallel(normal, 0.01))
        {
            throw new ConstructionErrorException("GeomFill_Fixed : Two parallel vectors !");
        }
        _t = tangent;
        _t = _t.Normalized();
        _n = normal;
        _n = _n.Normalized();
        _b = _t ^ _n;
        _b = _b.Normalized();
    }

    public override TrihedronLaw Copy()
    {
        Fixed copy = new Fixed(_t, _n);
        // OCCT sets the curve even when it is null
        copy.SetCurve(_curve!);
        return copy;
    }

    public override bool D0(double param, ref Vector3D tangent, ref Vector3D normal, ref Vector3D biNormal)
    {
        tangent = _t;
        normal = _n;
        biNormal = _b;

        return true;
    }

    public override bool D1(double param, ref Vector3D tangent, ref Vector3D dTangent, ref Vector3D normal, ref Vector3D dNormal,
                            ref Vector3D biNormal, ref Vector3D dBiNormal)
    {
        tangent = _t;
        normal = _n;
        biNormal = _b;

        Vector3D v0 = new Vector3D(0, 0, 0);
        dTangent = dNormal = dBiNormal = v0;

        return true;
    }

    public override bool D2(double param, ref Vector3D tangent, ref Vector3D dTangent, ref Vector3D d2Tangent, ref Vector3D normal,
                            ref Vector3D dNormal, ref Vector3D d2Normal, ref Vector3D biNormal, ref Vector3D dBiNormal,
                            ref Vector3D d2BiNormal)
    {
        tangent = _t;
        normal = _n;
        biNormal = _b;

        Vector3D v0 = new Vector3D(0, 0, 0);
        dTangent = d2Tangent = v0;
        dNormal = d2Normal = v0;
        dBiNormal = d2BiNormal = v0;

        return true;
    }

    public override int NbIntervals(Continuity s) => 1;

    public override void Intervals(NCollection_Array1<double> t, Continuity s)
    {
        t[t.Lower()] = -global::Kerf.Engine.Foundation.Precision.Infinite();
        t[t.Upper()] = global::Kerf.Engine.Foundation.Precision.Infinite();
    }

    public override void GetAverageLaw(ref Vector3D aTangent, ref Vector3D aNormal, ref Vector3D aBiNormal)
    {
        aTangent = _t;
        aNormal = _n;
        aBiNormal = _b;
    }

    /// <summary>Returns true.</summary>
    public override bool IsConstant => true;
}
