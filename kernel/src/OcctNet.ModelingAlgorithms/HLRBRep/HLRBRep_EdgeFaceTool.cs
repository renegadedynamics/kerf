// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_EdgeFaceTool.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_EdgeFaceTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;

namespace Occt;

/// <summary>Geometric tools between an edge's curve and a face's surface for the hidden-line algorithms.</summary>
public static class HLRBRep_EdgeFaceTool
{
    /// <summary>The normal curvature of the surface at (U, V) in the direction Tg (0 where the metric is degenerate).</summary>
    public static double CurvatureValue(HLRBRep_Surface F, double U, double V, in Direction3D Tg)
    {
        F.D2(U, V, out Point3D P, out Vector3D D1U, out Vector3D D1V, out Vector3D D2U, out Vector3D D2V, out Vector3D D2UV);
        Vector3D aTg = new(Tg);
        double d1ut = D1U * aTg;
        double d1vt = D1V * aTg;
        double d1ud1v = D1U * D1V;
        double nmu2 = D1U * D1U;
        double nmv2 = D1V * D1V;
        double det = nmu2 * nmv2 - d1ud1v * d1ud1v;
        if (det > Constants.Resolution())
        {
            double alfa = (d1ut * nmv2 - d1vt * d1ud1v) / det;
            double beta = (d1vt * nmu2 - d1ut * d1ud1v) / det;
            double alfa2 = alfa * alfa;
            double beta2 = beta * beta;
            double alfabeta = alfa * beta;
            Vector3D Nm = D1U ^ D1V;
            Nm.Normalize();
            double N = (Nm * D2U) * alfa2 + 2 * (Nm * D2UV) * alfabeta + (Nm * D2V) * beta2;
            double D = nmu2 * alfa2 + 2 * d1ud1v * alfabeta + nmv2 * beta2;
            return N / D;
        }
        return 0.0;
    }

    /// <summary>
    /// The parameters on the face F of the point of parameter Par on the edge E: from E's pcurve on F if it has one,
    /// otherwise by projecting the 3D point on the face (false if the projection finds nothing).
    /// </summary>
    public static bool UVPoint(double Par, HLRBRep_Curve E, HLRBRep_Surface F, out double U, out double V)
    {
        U = 0.0;
        V = 0.0;
        double pfbid, plbid;
        if (Tool.CurveOnSurface(E.Curve().Edge(), F.Surface().Face(), out pfbid, out plbid) is null)
        {
            BRepExtrema_ExtPF proj = new(new BRepLib_MakeVertex(E.Value3D(Par)).InternalVertex(), F.Surface().Face());
            int i, index = 0;
            double dist2 = RealLast();
            int n = proj.NbExt();
            for (i = 1; i <= n; i++)
            {
                double newdist2 = proj.SquareDistance(i);
                if (newdist2 < dist2)
                {
                    dist2 = newdist2;
                    index = i;
                }
            }
            if (index == 0)
            {
                return false;
            }

            proj.Parameter(index, out U, out V);
        }
        else
        {
            EdgeCurve2DAdaptor PC = new(E.Curve().Edge(), F.Surface().Face());
            PC.D0(Par, out Point2D P2d);
            U = P2d.X();
            V = P2d.Y();
        }
        return true;
    }
}
