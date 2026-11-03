// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_DraftLaw.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_DraftLaw.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>The location law of a draft along a wire (a <see cref="GeomFill_LocationDraft"/> per edge).</summary>
public sealed class BRepFill_DraftLaw : BRepFill_Edge3DLaw
{
    //=======================================================================
    // function : ToG0
    // purpose  : Calculate a transformation T tq T.M2 = M1
    //=======================================================================
    private static void ToG0(in Matrix3x3 M1, in Matrix3x3 M2, out Matrix3x3 T)
    {
        T = M2.Inverted();
        T *= M1;
    }

    /// <summary>The law along <paramref name="Path"/>, with copies of <paramref name="Law"/>.</summary>
    public BRepFill_DraftLaw(in TopoDS_Wire Path, GeomFill_LocationDraft Law) : base(Path, Law)
    {
    }

    /// <summary>Cleans the small discontinuities (normals parallel within <paramref name="TolAngular"/>).</summary>
    public void CleanLaw(double TolAngular)
    {
        double First, Last; //, Angle;
        int ipath;
        Matrix3x3 Trsf, M1 = new(), M2 = new();
        Vector3D V = new(), T1 = new(), T2 = new(), N1 = new(), N2 = new();
        //  gp_Dir D;

        myLaws!.Value(1).GetDomain(out First, out Last);
        // D = occ::down_cast<GeomFill_LocationDraft>(myLaws->Value(1))->Direction();
        //  gp_Vec Vd(D);

        for (ipath = 2; ipath <= myLaws.Length(); ipath++)
        {
            myLaws.Value(ipath - 1).D0(Last, ref M1, ref V);
            myLaws.Value(ipath).GetDomain(out First, out Last);
            myLaws.Value(ipath).D0(First, ref M2, ref V);
            T1.SetXYZ(M1.Column(3));
            T2.SetXYZ(M2.Column(3));
            N1.SetXYZ(M1.Column(1));
            N2.SetXYZ(M2.Column(1));
            if (N1.IsParallel(N2, TolAngular))
            { // Correction G0 des normales...
                ToG0(M1, M2, out Trsf);
                myLaws.Value(ipath).SetTrsf(Trsf);
            }
        }
    }
}
