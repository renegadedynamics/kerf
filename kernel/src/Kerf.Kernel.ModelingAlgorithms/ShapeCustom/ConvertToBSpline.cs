// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_ConvertToBSpline.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_ConvertToBSpline.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Healing.Custom;

/// <summary>
/// The modification that converts surfaces of linear extrusion, of revolution, offset surfaces and planes (each as
/// its mode says) to B-spline surfaces with the same parametrisation.
/// </summary>
/// <remarks>
/// The modes are extrusion, revolution and offset on, plane off by default. With the offset mode off, an offset
/// surface whose basis is to convert keeps its offset over a B-spline basis.
/// </remarks>
public class ConvertToBSpline : CustomModification
{
    private bool _extrMode;
    private bool _revolMode;
    private bool _offsetMode;
    private bool _planeMode;

    /// <summary>A modification converting extrusions, revolutions and offset surfaces, not planes.</summary>
    public ConvertToBSpline()
    {
        _extrMode = true;
        _revolMode = true;
        _offsetMode = true;
        _planeMode = false;
    }

    private bool IsToConvert(Surface s, out Surface ss)
    {
        ss = s;
        if (s is TrimmedSurface rts)
        {
            ss = rts.BasisSurface();
        }
        if (ss is OffsetSurface os)
        {
            if (_offsetMode)
            {
                return true;
            }
            else
            {
                Surface basis = os.BasisSurface();
                return IsToConvert(basis, out _);
            }
        }
        if (ss is ExtrusionSurface)
        {
            return _extrMode;
        }
        if (ss is RevolutionSurface)
        {
            return _revolMode;
        }
        if (ss is PlaneSurface)
        {
            return _planeMode;
        }
        return false;
    }

    /// <summary>
    /// True if the face's surface is to convert: then S is the B-spline (Precision::Approximation(), up to 10000 spans
    /// of degree 15), over the face's UV bounds where the surface is infinite, trimmed as the original.
    /// </summary>
    public override bool NewSurface(in Face f,
                                    out Surface? s,
                                    out Placement l,
                                    out double tol,
                                    out bool revWires,
                                    out bool revFace)
    {
        tol = 0.0;
        revWires = revFace = false;
        s = Tool.Surface(f, out l);
        s!.Bounds(out double u1, out double u2, out double v1, out double v2);
        BrepTools.UVBounds(f, out double umin, out double umax, out double vmin, out double vmax);
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(u1) || global::Kerf.Engine.Foundation.Precision.IsInfinite(u2))
        {
            u1 = umin;
            u2 = umax;
        }
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(v1) || global::Kerf.Engine.Foundation.Precision.IsInfinite(v2))
        {
            v1 = vmin;
            v2 = vmax;
        }

        if (!IsToConvert(s, out Surface surf))
        {
            return false;
        }

        Surface? res;
        if (surf is OffsetSurface os && !_offsetMode)
        {
            Surface basis = os.BasisSurface();
            double offset = os.Offset();
            BSplineSurface? bspl =
                global::Kerf.Engine.Healing.Construct.ShapeConstruct.InternalConvertSurfaceToBSpline(basis,
                                                               u1,
                                                               u2,
                                                               v1,
                                                               v2,
                                                               global::Kerf.Engine.Foundation.Precision.Approximation(),
                                                               surf.Continuity(),
                                                               10000,
                                                               15);
            OffsetSurface nOff = new OffsetSurface(bspl!, offset);
            res = nOff;
        }
        else
        {
            Continuity cnt = surf.Continuity();
            if (surf is OffsetSurface)
            {
                cnt = C0; // pdn 30.06.99 because of hang-up in GeomConvert_ApproxSurface
            }
            res = global::Kerf.Engine.Healing.Construct.ShapeConstruct.InternalConvertSurfaceToBSpline(surf,
                                                                 u1,
                                                                 u2,
                                                                 v1,
                                                                 v2,
                                                                 global::Kerf.Engine.Foundation.Precision.Approximation(),
                                                                 cnt,
                                                                 10000,
                                                                 15);
        }
        if (s is TrimmedSurface rts)
        {
            rts.Bounds(out double uf, out double ul, out double vf, out double vl);
            s = new TrimmedSurface(res!, uf, ul, vf, vl);
        }
        else
        {
            s = res;
        }

        SendMsg(f, new Msg("ConvertToBSpline.NewSurface.MSG0"));

        tol = Tool.Tolerance(f);
        revWires = false;
        revFace = false;
        return true;
    }

    /// <summary>
    /// True if the edge has a pcurve on a surface to convert (so that the edge is copied): then C is a copy of its 3D
    /// curve.
    /// </summary>
    public override bool NewCurve(in Edge e, out Curve3D? c, out Placement L, out double tol)
    {
        c = null;
        L = new Placement();
        tol = 0.0;
        //: p5 abv 26 Feb 99: force copying of edge if any its pcurve will be replaced
        BrepTEdge te = (BrepTEdge)e.TShape()!;

        // iterate on pcurves
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(te.Curves());
        for (; itcr.More(); itcr.Next())
        {
            if (itcr.Value() is not GCurve gc || !gc.IsCurveOnSurface())
            {
                continue;
            }
            Surface s = gc.Surface()!;
            if (!IsToConvert(s, out _))
            {
                continue;
            }
            c = Tool.Curve(e, out L, out double f, out double l);
            if (c is not null)
            {
                c = (Curve3D)c.Copy();
            }
            tol = Tool.Tolerance(e);
            SendMsg(e, new Msg("ConvertToBSpline.NewCurve.MSG0"));
            return true;
        }
        return false;
    }

    /// <summary>False: 3D points are never modified.</summary>
    public override bool NewPoint(in Vertex v, out Point3D p, out double tol)
    {
        p = new Point3D();
        tol = 0.0;
        return false;
    }

    /// <summary>True if the face's surface is converted or the edge was copied: C is a copy of the pcurve.</summary>
    public override bool NewCurve2d(in Edge e,
                                    in Face F,
                                    in Edge newE,
                                    in Face newF,
                                    out Curve2D? c,
                                    out double tol)
    {
        c = null;
        tol = 0.0;
        Surface s = Tool.Surface(F, out Placement L)!;

        // just copy pcurve if either its surface is changing or edge was copied
        if (!IsToConvert(s, out _) && e.IsSame(newE))
        {
            return false;
        }

        c = Tool.CurveOnSurface(e, F, out double f, out double l);
        if (c is not null)
        {
            c = (Curve2D)c.Copy();
        }

        tol = Tool.Tolerance(e);
        return true;
    }

    /// <summary>False: parameters are never modified.</summary>
    public override bool NewParameter(in Vertex v, in Edge e, ref double p, ref double tol)
    {
        return false;
    }

    /// <summary>The continuity of the edge between the faces, unchanged.</summary>
    public override Continuity Continuity(in Edge e,
                                             in Face f1,
                                             in Face f2,
                                             in Edge newE,
                                             in Face newF1,
                                             in Face newF2)
    {
        return Tool.Continuity(e, f1, f2);
    }

    /// <summary>Sets the mode for surfaces of linear extrusion (true by default).</summary>
    public void SetExtrusionMode(bool extrMode)
    {
        _extrMode = extrMode;
    }

    /// <summary>Sets the mode for surfaces of revolution (true by default).</summary>
    public void SetRevolutionMode(bool revolMode)
    {
        _revolMode = revolMode;
    }

    /// <summary>Sets the mode for offset surfaces (true by default).</summary>
    public void SetOffsetMode(bool offsetMode)
    {
        _offsetMode = offsetMode;
    }

    /// <summary>Sets the mode for planes (false by default).</summary>
    public void SetPlaneMode(bool planeMode)
    {
        _planeMode = planeMode;
    }
}
