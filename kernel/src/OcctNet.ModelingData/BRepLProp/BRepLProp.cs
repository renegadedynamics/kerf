// Port of: src/ModelingData/TKBRep/BRepLProp/BRepLProp.hxx
// Port of: src/ModelingData/TKBRep/BRepLProp/BRepLProp.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Geometry.Continuity;
using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>The continuity of a curve built by concatenating two edges, at their junction.</summary>
public static class BRepLProp
{
    /// <summary>
    /// The regularity at the junction of C1 and C2, whose points u1 on C1 and u2 on C2 must coincide; tl and ta are the
    /// linear and angular tolerances used to compare the derivatives.
    /// </summary>
    /// <exception cref="FailureException">The points don't coincide.</exception>
    public static Continuity Continuity(BRepAdaptor_Curve C1, BRepAdaptor_Curve C2, double u1, double u2, double tl, double ta)
    {
        Continuity cont = C0;
        bool fini = false;
        Vector3D d1, d2;
        Continuity cont1 = C1.Continuity(), cont2 = C2.Continuity();
        int n1 = 0, n2 = 0;
        if ((int)cont1 >= 5)
        {
            n1 = 3;
        }
        else if ((int)cont1 == 4)
        {
            n1 = 2;
        }
        else if ((int)cont1 == 2)
        {
            n1 = 1;
        }
        if ((int)cont2 >= 5)
        {
            n2 = 3;
        }
        else if ((int)cont2 == 4)
        {
            n2 = 2;
        }
        else if ((int)cont2 == 2)
        {
            n2 = 1;
        }
        BRepLProp_CLProps clp1 = new BRepLProp_CLProps(C1, u1, n1, tl);
        BRepLProp_CLProps clp2 = new BRepLProp_CLProps(C2, u2, n2, tl);
        if (!(clp1.Value().IsEqual(clp2.Value(), tl)))
        {
            throw new FailureException("Courbes non jointives");
        }
        int min = Min(n1, n2);
        if (min >= 1)
        {
            d1 = clp1.D1();
            d2 = clp2.D1();
            if (C1.Edge().Orientation() == TopAbs_REVERSED)
            {
                d1.Reverse();
            }
            if (C2.Edge().Orientation() == TopAbs_REVERSED)
            {
                d2.Reverse();
            }
            if (d1.IsEqual(d2, tl, ta))
            {
                cont = global::Kerf.Geometry.Continuity.C1;
            }
            else if (clp1.IsTangentDefined() && clp2.IsTangentDefined())
            {
                clp1.Tangent(out Direction3D dir1);
                clp2.Tangent(out Direction3D dir2);
                if (C1.Edge().Orientation() == TopAbs_REVERSED)
                {
                    dir1.Reverse();
                }
                if (C2.Edge().Orientation() == TopAbs_REVERSED)
                {
                    dir2.Reverse();
                }
                if (dir1.IsEqual(dir2, ta))
                {
                    cont = G1;
                }
                fini = true;
            }
            else
            {
                fini = true;
            }
        }
        if (min >= 2 && !fini)
        {
            d1 = clp1.D2();
            d2 = clp2.D2();
            if (d1.IsEqual(d2, tl, ta))
            {
                cont = global::Kerf.Geometry.Continuity.C2;
            }
        }
        TopoDS_Edge E1 = C1.Edge();
        TopoDS_Edge E2 = C2.Edge();
        if (E1.IsSame(E2) && C1.IsPeriodic() && cont >= G1)
        {
            cont = CN;
        }
        return cont;
    }

    /// <summary>As the previous one, with Precision's confusion and angular tolerances.</summary>
    public static Continuity Continuity(BRepAdaptor_Curve C1, BRepAdaptor_Curve C2, double u1, double u2)
    {
        return Continuity(C1, C2, u1, u2, global::Kerf.Engine.Foundation.Precision.Confusion(), global::Kerf.Engine.Foundation.Precision.Angular());
    }
}
