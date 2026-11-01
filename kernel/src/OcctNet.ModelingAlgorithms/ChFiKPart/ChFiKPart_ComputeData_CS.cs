// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_CS.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_CS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The guide circle and cylinder of a toric corner fillet.</summary>
internal static class ChFiKPart_ComputeData_CS
{
    /// <summary>The circle of radius <paramref name="R"/> from the point <paramref name="P1S2"/> to
    /// <paramref name="P2S2"/> of <paramref name="S2"/>, in the plane normal to the plane <paramref name="S1"/>, and
    /// the cylinder around it; [<paramref name="First"/>, <paramref name="Last"/>] its parameters.</summary>
    internal static void ChFiKPart_CornerSpine(Adaptor3d_Surface S1, Adaptor3d_Surface S2, in gp_Pnt2d P1S1,
                                               in gp_Pnt2d P2S1, in gp_Pnt2d P1S2, in gp_Pnt2d P2S2, double R,
                                               ref gp_Cylinder cyl, ref gp_Circ circ, out double First,
                                               out double Last)
    {
        gp_Ax3 ax = S1.Plane().Position();
        gp_Vec V1 = new(ax.XDirection());
        gp_Vec V2 = new(ax.YDirection());
        S2.D1(P1S2.X(), P1S2.Y(), out gp_Pnt P, out gp_Vec du, out gp_Vec dv);
        gp_Vec V = new(P, S1.Value(P1S1.X(), P1S1.Y()));
        V = V.Dot(V1) * V1 + V.Dot(V2) * V2;
        V.Normalize();
        gp_Pnt P2 = S2.Value(P2S2.X(), P2S2.Y());
        gp_Vec Vorien = new(P, P2);
        gp_Pnt cent = new();
        gp_Dir dx = new(V);
        if (V.Dot(Vorien) >= 0.0)
        {
            cent.SetCoord(P.X() + R * V.X(), P.Y() + R * V.Y(), P.Z() + R * V.Z());
            dx.Reverse();
        }
        else
        {
            cent.SetCoord(P.X() - R * V.X(), P.Y() - R * V.Y(), P.Z() - R * V.Z());
        }
        gp_Dir dy = new(new gp_Vec(cent, P2));
        dy = (dx ^ dy) ^ dx;
        gp_Ax2 circax2 = new(cent, dx ^ dy, dx);
        gp_Ax3 cylax3 = new(circax2);
        if ((du ^ dv).Dot(new gp_Vec(dx)) < 0.0)
        {
            cylax3.ZReverse();
        }
        First = 0.0;
        Last = ElCLib.CircleParameter(circax2, P2);
        circ.SetPosition(circax2);
        circ.SetRadius(R);
        cyl.SetPosition(cylax3);
        cyl.SetRadius(R);
    }
}
