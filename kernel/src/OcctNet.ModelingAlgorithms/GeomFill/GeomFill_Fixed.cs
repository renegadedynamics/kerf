// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Fixed.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Fixed.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Occt;

/// <summary>A constant trihedron law: the same tangent, normal and binormal at every parameter.</summary>
public class GeomFill_Fixed : GeomFill_TrihedronLaw
{
    private Vector3D T;
    private Vector3D N;
    private Vector3D B;

    /// <summary>The law of the trihedron (Tangent, Normal, Tangent ^ Normal), normalized.</summary>
    /// <exception cref="ConstructionErrorException">The vectors are parallel (within 0.01 rad).</exception>
    public GeomFill_Fixed(in Vector3D Tangent, in Vector3D Normal)
    {
        if (Tangent.IsParallel(Normal, 0.01))
        {
            throw new ConstructionErrorException("GeomFill_Fixed : Two parallel vectors !");
        }
        T = Tangent;
        T.Normalize();
        N = Normal;
        N.Normalize();
        B = T ^ N;
        B.Normalize();
    }

    public override GeomFill_TrihedronLaw Copy()
    {
        GeomFill_Fixed copy = new GeomFill_Fixed(T, N);
        // OCCT sets the curve even when it is null
        copy.SetCurve(myCurve!);
        return copy;
    }

    public override bool D0(double Param, ref Vector3D Tangent, ref Vector3D Normal, ref Vector3D BiNormal)
    {
        Tangent = T;
        Normal = N;
        BiNormal = B;

        return true;
    }

    public override bool D1(double Param, ref Vector3D Tangent, ref Vector3D DTangent, ref Vector3D Normal, ref Vector3D DNormal,
                            ref Vector3D BiNormal, ref Vector3D DBiNormal)
    {
        Tangent = T;
        Normal = N;
        BiNormal = B;

        Vector3D V0 = new Vector3D(0, 0, 0);
        DTangent = DNormal = DBiNormal = V0;

        return true;
    }

    public override bool D2(double Param, ref Vector3D Tangent, ref Vector3D DTangent, ref Vector3D D2Tangent, ref Vector3D Normal,
                            ref Vector3D DNormal, ref Vector3D D2Normal, ref Vector3D BiNormal, ref Vector3D DBiNormal,
                            ref Vector3D D2BiNormal)
    {
        Tangent = T;
        Normal = N;
        BiNormal = B;

        Vector3D V0 = new Vector3D(0, 0, 0);
        DTangent = D2Tangent = V0;
        DNormal = D2Normal = V0;
        DBiNormal = D2BiNormal = V0;

        return true;
    }

    public override int NbIntervals(Continuity S) => 1;

    public override void Intervals(NCollection_Array1<double> theT, Continuity S)
    {
        theT[theT.Lower()] = -global::Kerf.Engine.Foundation.Precision.Infinite();
        theT[theT.Upper()] = global::Kerf.Engine.Foundation.Precision.Infinite();
    }

    public override void GetAverageLaw(ref Vector3D ATangent, ref Vector3D ANormal, ref Vector3D ABiNormal)
    {
        ATangent = T;
        ANormal = N;
        ABiNormal = B;
    }

    /// <summary>Returns true.</summary>
    public override bool IsConstant() => true;
}
