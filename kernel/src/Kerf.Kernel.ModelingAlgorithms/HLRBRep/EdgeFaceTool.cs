// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_EdgeFaceTool.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_EdgeFaceTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Library;
using Kerf.Engine.Geometry;

using Kerf.Engine.Measurement.Distance;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.HiddenLines;

/// <summary>Geometric tools between an edge's curve and a face's surface for the hidden-line algorithms.</summary>
public static class EdgeFaceTool
{
    /// <summary>The normal curvature of the surface at (U, V) in the direction Tg (0 where the metric is degenerate).</summary>
    public static double CurvatureValue(HlrSurface f, double u, double v, in Direction3D tg)
    {
        f.D2(u, v, out Point3D p, out Vector3D d1U, out Vector3D d1V, out Vector3D d2U, out Vector3D d2V, out Vector3D d2UV);
        Vector3D aTg = new(tg);
        double d1ut = d1U * aTg;
        double d1vt = d1V * aTg;
        double d1ud1v = d1U * d1V;
        double nmu2 = d1U * d1U;
        double nmv2 = d1V * d1V;
        double det = nmu2 * nmv2 - d1ud1v * d1ud1v;
        if (det > Constants.Resolution())
        {
            double alfa = (d1ut * nmv2 - d1vt * d1ud1v) / det;
            double beta = (d1vt * nmu2 - d1ut * d1ud1v) / det;
            double alfa2 = alfa * alfa;
            double beta2 = beta * beta;
            double alfabeta = alfa * beta;
            Vector3D nm = d1U ^ d1V;
            nm = nm.Normalized();
            double n = (nm * d2U) * alfa2 + 2 * (nm * d2UV) * alfabeta + (nm * d2V) * beta2;
            double d = nmu2 * alfa2 + 2 * d1ud1v * alfabeta + nmv2 * beta2;
            return n / d;
        }
        return 0.0;
    }

    /// <summary>
    /// The parameters on the face F of the point of parameter Par on the edge E: from E's pcurve on F if it has one,
    /// otherwise by projecting the 3D point on the face (false if the projection finds nothing).
    /// </summary>
    public static bool UVPoint(double par, HiddenLineCurve e, HlrSurface f, out double u, out double v)
    {
        u = 0.0;
        v = 0.0;
        double pfbid, plbid;
        if (Tool.CurveOnSurface(e.Curve().Edge(), f.Surface().Face(), out pfbid, out plbid) is null)
        {
            ExtPF proj = new(new LibraryMakeVertex(e.Value3D(par)).InternalVertex(), f.Surface().Face());
            int i, index = 0;
            double dist2 = RealLast();
            int n = proj.ExtCount;
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

            proj.Parameter(index, out u, out v);
        }
        else
        {
            EdgeCurve2DAdaptor pc = new(e.Curve().Edge(), f.Surface().Face());
            pc.D0(par, out Point2D p2d);
            u = p2d.X;
            v = p2d.Y;
        }
        return true;
    }
}
