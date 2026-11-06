// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_DraftLaw.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_DraftLaw.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Surfaces;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Engine.Construction.Filling;

/// <summary>The location law of a draft along a wire (a <see cref="LocationDraft"/> per edge).</summary>
internal sealed class DraftLaw : Edge3DLaw
{
    //=======================================================================
    // function : ToG0
    // purpose  : Calculate a transformation T tq T.M2 = M1
    //=======================================================================
    private static void ToG0(in Matrix3x3 m1, in Matrix3x3 m2, out Matrix3x3 t)
    {
        t = m2.Inverted();
        t *= m1;
    }

    /// <summary>The law along <paramref name="path"/>, with copies of <paramref name="law"/>.</summary>
    public DraftLaw(in Wire path, LocationDraft law) : base(path, law)
    {
    }

    /// <summary>Cleans the small discontinuities (normals parallel within <paramref name="tolAngular"/>).</summary>
    public void CleanLaw(double tolAngular)
    {
        double first, last; //, Angle;
        int ipath;
        Matrix3x3 trsf, m1 = new(), m2 = new();
        Vector3D v = new(), t1 = new(), t2 = new(), n1 = new(), n2 = new();
        //  gp_Dir D;

        _laws!.Value(1).GetDomain(out first, out last);
        // D = occ::down_cast<GeomFill_LocationDraft>(myLaws->Value(1))->Direction();
        //  gp_Vec Vd(D);

        for (ipath = 2; ipath <= _laws.Length(); ipath++)
        {
            _laws.Value(ipath - 1).D0(last, ref m1, ref v);
            _laws.Value(ipath).GetDomain(out first, out last);
            _laws.Value(ipath).D0(first, ref m2, ref v);
            t1 = t1.WithXYZ(m1.Column(3));
            t2 = t2.WithXYZ(m2.Column(3));
            n1 = n1.WithXYZ(m1.Column(1));
            n2 = n2.WithXYZ(m2.Column(1));
            if (n1.IsParallel(n2, tolAngular))
            { // Correction G0 des normales...
                ToG0(m1, m2, out trsf);
                _laws.Value(ipath).SetTrsf(trsf);
            }
        }
    }
}
