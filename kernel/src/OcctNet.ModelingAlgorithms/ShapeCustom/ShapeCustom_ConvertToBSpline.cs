// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_ConvertToBSpline.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_ConvertToBSpline.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>
/// The modification that converts surfaces of linear extrusion, of revolution, offset surfaces and planes (each as
/// its mode says) to B-spline surfaces with the same parametrisation.
/// </summary>
/// <remarks>
/// The modes are extrusion, revolution and offset on, plane off by default. With the offset mode off, an offset
/// surface whose basis is to convert keeps its offset over a B-spline basis.
/// </remarks>
public class ShapeCustom_ConvertToBSpline : ShapeCustom_Modification
{
    private bool myExtrMode;
    private bool myRevolMode;
    private bool myOffsetMode;
    private bool myPlaneMode;

    /// <summary>A modification converting extrusions, revolutions and offset surfaces, not planes.</summary>
    public ShapeCustom_ConvertToBSpline()
    {
        myExtrMode = true;
        myRevolMode = true;
        myOffsetMode = true;
        myPlaneMode = false;
    }

    private bool IsToConvert(Geom_Surface S, out Geom_Surface SS)
    {
        SS = S;
        if (S is Geom_RectangularTrimmedSurface RTS)
        {
            SS = RTS.BasisSurface();
        }
        if (SS is Geom_OffsetSurface OS)
        {
            if (myOffsetMode)
            {
                return true;
            }
            else
            {
                Geom_Surface basis = OS.BasisSurface();
                return IsToConvert(basis, out _);
            }
        }
        if (SS is Geom_SurfaceOfLinearExtrusion)
        {
            return myExtrMode;
        }
        if (SS is Geom_SurfaceOfRevolution)
        {
            return myRevolMode;
        }
        if (SS is Geom_Plane)
        {
            return myPlaneMode;
        }
        return false;
    }

    /// <summary>
    /// True if the face's surface is to convert: then S is the B-spline (Precision::Approximation(), up to 10000 spans
    /// of degree 15), over the face's UV bounds where the surface is infinite, trimmed as the original.
    /// </summary>
    public override bool NewSurface(in TopoDS_Face F,
                                    out Geom_Surface? S,
                                    out Placement L,
                                    out double Tol,
                                    out bool RevWires,
                                    out bool RevFace)
    {
        Tol = 0.0;
        RevWires = RevFace = false;
        S = BRep_Tool.Surface(F, out L);
        S!.Bounds(out double U1, out double U2, out double V1, out double V2);
        BRepTools.UVBounds(F, out double Umin, out double Umax, out double Vmin, out double Vmax);
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(U1) || global::Kerf.Engine.Foundation.Precision.IsInfinite(U2))
        {
            U1 = Umin;
            U2 = Umax;
        }
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(V1) || global::Kerf.Engine.Foundation.Precision.IsInfinite(V2))
        {
            V1 = Vmin;
            V2 = Vmax;
        }

        if (!IsToConvert(S, out Geom_Surface surf))
        {
            return false;
        }

        Geom_Surface? res;
        if (surf is Geom_OffsetSurface OS && !myOffsetMode)
        {
            Geom_Surface basis = OS.BasisSurface();
            double offset = OS.Offset();
            Geom_BSplineSurface? bspl =
                ShapeConstruct.InternalConvertSurfaceToBSpline(basis,
                                                               U1,
                                                               U2,
                                                               V1,
                                                               V2,
                                                               global::Kerf.Engine.Foundation.Precision.Approximation(),
                                                               surf.Continuity(),
                                                               10000,
                                                               15);
            Geom_OffsetSurface nOff = new Geom_OffsetSurface(bspl!, offset);
            res = nOff;
        }
        else
        {
            Continuity cnt = surf.Continuity();
            if (surf is Geom_OffsetSurface)
            {
                cnt = C0; // pdn 30.06.99 because of hang-up in GeomConvert_ApproxSurface
            }
            res = ShapeConstruct.InternalConvertSurfaceToBSpline(surf,
                                                                 U1,
                                                                 U2,
                                                                 V1,
                                                                 V2,
                                                                 global::Kerf.Engine.Foundation.Precision.Approximation(),
                                                                 cnt,
                                                                 10000,
                                                                 15);
        }
        if (S is Geom_RectangularTrimmedSurface RTS)
        {
            RTS.Bounds(out double UF, out double UL, out double VF, out double VL);
            S = new Geom_RectangularTrimmedSurface(res!, UF, UL, VF, VL);
        }
        else
        {
            S = res;
        }

        SendMsg(F, new Msg("ConvertToBSpline.NewSurface.MSG0"));

        Tol = BRep_Tool.Tolerance(F);
        RevWires = false;
        RevFace = false;
        return true;
    }

    /// <summary>
    /// True if the edge has a pcurve on a surface to convert (so that the edge is copied): then C is a copy of its 3D
    /// curve.
    /// </summary>
    public override bool NewCurve(in TopoDS_Edge E, out Geom_Curve? C, out Placement L, out double Tol)
    {
        C = null;
        L = new Placement();
        Tol = 0.0;
        //: p5 abv 26 Feb 99: force copying of edge if any its pcurve will be replaced
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;

        // iterate on pcurves
        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(TE.Curves());
        for (; itcr.More(); itcr.Next())
        {
            if (itcr.Value() is not BRep_GCurve GC || !GC.IsCurveOnSurface())
            {
                continue;
            }
            Geom_Surface S = GC.Surface()!;
            if (!IsToConvert(S, out _))
            {
                continue;
            }
            C = BRep_Tool.Curve(E, out L, out double f, out double l);
            if (C is not null)
            {
                C = (Geom_Curve)C.Copy();
            }
            Tol = BRep_Tool.Tolerance(E);
            SendMsg(E, new Msg("ConvertToBSpline.NewCurve.MSG0"));
            return true;
        }
        return false;
    }

    /// <summary>False: 3D points are never modified.</summary>
    public override bool NewPoint(in TopoDS_Vertex V, out Point3D P, out double Tol)
    {
        P = new Point3D();
        Tol = 0.0;
        return false;
    }

    /// <summary>True if the face's surface is converted or the edge was copied: C is a copy of the pcurve.</summary>
    public override bool NewCurve2d(in TopoDS_Edge E,
                                    in TopoDS_Face F,
                                    in TopoDS_Edge NewE,
                                    in TopoDS_Face NewF,
                                    out Geom2d_Curve? C,
                                    out double Tol)
    {
        C = null;
        Tol = 0.0;
        Geom_Surface S = BRep_Tool.Surface(F, out Placement L)!;

        // just copy pcurve if either its surface is changing or edge was copied
        if (!IsToConvert(S, out _) && E.IsSame(NewE))
        {
            return false;
        }

        C = BRep_Tool.CurveOnSurface(E, F, out double f, out double l);
        if (C is not null)
        {
            C = (Geom2d_Curve)C.Copy();
        }

        Tol = BRep_Tool.Tolerance(E);
        return true;
    }

    /// <summary>False: parameters are never modified.</summary>
    public override bool NewParameter(in TopoDS_Vertex V, in TopoDS_Edge E, ref double P, ref double Tol)
    {
        return false;
    }

    /// <summary>The continuity of the edge between the faces, unchanged.</summary>
    public override Continuity Continuity(in TopoDS_Edge E,
                                             in TopoDS_Face F1,
                                             in TopoDS_Face F2,
                                             in TopoDS_Edge NewE,
                                             in TopoDS_Face NewF1,
                                             in TopoDS_Face NewF2)
    {
        return BRep_Tool.Continuity(E, F1, F2);
    }

    /// <summary>Sets the mode for surfaces of linear extrusion (true by default).</summary>
    public void SetExtrusionMode(bool extrMode)
    {
        myExtrMode = extrMode;
    }

    /// <summary>Sets the mode for surfaces of revolution (true by default).</summary>
    public void SetRevolutionMode(bool revolMode)
    {
        myRevolMode = revolMode;
    }

    /// <summary>Sets the mode for offset surfaces (true by default).</summary>
    public void SetOffsetMode(bool offsetMode)
    {
        myOffsetMode = offsetMode;
    }

    /// <summary>Sets the mode for planes (false by default).</summary>
    public void SetPlaneMode(bool planeMode)
    {
        myPlaneMode = planeMode;
    }
}
