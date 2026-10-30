// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Fixed.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Fixed.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A constant trihedron law: the same tangent, normal and binormal at every parameter.</summary>
public class GeomFill_Fixed : GeomFill_TrihedronLaw
{
    private gp_Vec T;
    private gp_Vec N;
    private gp_Vec B;

    /// <summary>The law of the trihedron (Tangent, Normal, Tangent ^ Normal), normalized.</summary>
    /// <exception cref="Standard_ConstructionError">The vectors are parallel (within 0.01 rad).</exception>
    public GeomFill_Fixed(in gp_Vec Tangent, in gp_Vec Normal)
    {
        if (Tangent.IsParallel(Normal, 0.01))
        {
            throw new Standard_ConstructionError("GeomFill_Fixed : Two parallel vectors !");
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

    public override bool D0(double Param, ref gp_Vec Tangent, ref gp_Vec Normal, ref gp_Vec BiNormal)
    {
        Tangent = T;
        Normal = N;
        BiNormal = B;

        return true;
    }

    public override bool D1(double Param, ref gp_Vec Tangent, ref gp_Vec DTangent, ref gp_Vec Normal, ref gp_Vec DNormal,
                            ref gp_Vec BiNormal, ref gp_Vec DBiNormal)
    {
        Tangent = T;
        Normal = N;
        BiNormal = B;

        gp_Vec V0 = new gp_Vec(0, 0, 0);
        DTangent = DNormal = DBiNormal = V0;

        return true;
    }

    public override bool D2(double Param, ref gp_Vec Tangent, ref gp_Vec DTangent, ref gp_Vec D2Tangent, ref gp_Vec Normal,
                            ref gp_Vec DNormal, ref gp_Vec D2Normal, ref gp_Vec BiNormal, ref gp_Vec DBiNormal,
                            ref gp_Vec D2BiNormal)
    {
        Tangent = T;
        Normal = N;
        BiNormal = B;

        gp_Vec V0 = new gp_Vec(0, 0, 0);
        DTangent = D2Tangent = V0;
        DNormal = D2Normal = V0;
        DBiNormal = D2BiNormal = V0;

        return true;
    }

    public override int NbIntervals(GeomAbs_Shape S) => 1;

    public override void Intervals(NCollection_Array1<double> theT, GeomAbs_Shape S)
    {
        theT[theT.Lower()] = -Precision.Infinite();
        theT[theT.Upper()] = Precision.Infinite();
    }

    public override void GetAverageLaw(ref gp_Vec ATangent, ref gp_Vec ANormal, ref gp_Vec ABiNormal)
    {
        ATangent = T;
        ANormal = N;
        ABiNormal = B;
    }

    /// <summary>Returns true.</summary>
    public override bool IsConstant() => true;
}
