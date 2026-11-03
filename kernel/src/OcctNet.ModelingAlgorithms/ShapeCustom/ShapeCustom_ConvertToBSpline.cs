// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_ConvertToBSpline.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_ConvertToBSpline.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
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

    private bool IsToConvert(Surface S, out Surface SS)
    {
        SS = S;
        if (S is TrimmedSurface RTS)
        {
            SS = RTS.BasisSurface();
        }
        if (SS is OffsetSurface OS)
        {
            if (myOffsetMode)
            {
                return true;
            }
            else
            {
                Surface basis = OS.BasisSurface();
                return IsToConvert(basis, out _);
            }
        }
        if (SS is ExtrusionSurface)
        {
            return myExtrMode;
        }
        if (SS is RevolutionSurface)
        {
            return myRevolMode;
        }
        if (SS is PlaneSurface)
        {
            return myPlaneMode;
        }
        return false;
    }

    /// <summary>
    /// True if the face's surface is to convert: then S is the B-spline (Precision::Approximation(), up to 10000 spans
    /// of degree 15), over the face's UV bounds where the surface is infinite, trimmed as the original.
    /// </summary>
    public override bool NewSurface(in Face F,
                                    out Surface? S,
                                    out Placement L,
                                    out double Tol,
                                    out bool RevWires,
                                    out bool RevFace)
    {
        Tol = 0.0;
        RevWires = RevFace = false;
        S = Tool.Surface(F, out L);
        S!.Bounds(out double U1, out double U2, out double V1, out double V2);
        BrepTools.UVBounds(F, out double Umin, out double Umax, out double Vmin, out double Vmax);
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

        if (!IsToConvert(S, out Surface surf))
        {
            return false;
        }

        Surface? res;
        if (surf is OffsetSurface OS && !myOffsetMode)
        {
            Surface basis = OS.BasisSurface();
            double offset = OS.Offset();
            BSplineSurface? bspl =
                ShapeConstruct.InternalConvertSurfaceToBSpline(basis,
                                                               U1,
                                                               U2,
                                                               V1,
                                                               V2,
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
        if (S is TrimmedSurface RTS)
        {
            RTS.Bounds(out double UF, out double UL, out double VF, out double VL);
            S = new TrimmedSurface(res!, UF, UL, VF, VL);
        }
        else
        {
            S = res;
        }

        SendMsg(F, new Msg("ConvertToBSpline.NewSurface.MSG0"));

        Tol = Tool.Tolerance(F);
        RevWires = false;
        RevFace = false;
        return true;
    }

    /// <summary>
    /// True if the edge has a pcurve on a surface to convert (so that the edge is copied): then C is a copy of its 3D
    /// curve.
    /// </summary>
    public override bool NewCurve(in Edge E, out Curve3D? C, out Placement L, out double Tol)
    {
        C = null;
        L = new Placement();
        Tol = 0.0;
        //: p5 abv 26 Feb 99: force copying of edge if any its pcurve will be replaced
        BrepTEdge TE = (BrepTEdge)E.TShape()!;

        // iterate on pcurves
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(TE.Curves());
        for (; itcr.More(); itcr.Next())
        {
            if (itcr.Value() is not GCurve GC || !GC.IsCurveOnSurface())
            {
                continue;
            }
            Surface S = GC.Surface()!;
            if (!IsToConvert(S, out _))
            {
                continue;
            }
            C = Tool.Curve(E, out L, out double f, out double l);
            if (C is not null)
            {
                C = (Curve3D)C.Copy();
            }
            Tol = Tool.Tolerance(E);
            SendMsg(E, new Msg("ConvertToBSpline.NewCurve.MSG0"));
            return true;
        }
        return false;
    }

    /// <summary>False: 3D points are never modified.</summary>
    public override bool NewPoint(in Vertex V, out Point3D P, out double Tol)
    {
        P = new Point3D();
        Tol = 0.0;
        return false;
    }

    /// <summary>True if the face's surface is converted or the edge was copied: C is a copy of the pcurve.</summary>
    public override bool NewCurve2d(in Edge E,
                                    in Face F,
                                    in Edge NewE,
                                    in Face NewF,
                                    out Curve2D? C,
                                    out double Tol)
    {
        C = null;
        Tol = 0.0;
        Surface S = Tool.Surface(F, out Placement L)!;

        // just copy pcurve if either its surface is changing or edge was copied
        if (!IsToConvert(S, out _) && E.IsSame(NewE))
        {
            return false;
        }

        C = Tool.CurveOnSurface(E, F, out double f, out double l);
        if (C is not null)
        {
            C = (Curve2D)C.Copy();
        }

        Tol = Tool.Tolerance(E);
        return true;
    }

    /// <summary>False: parameters are never modified.</summary>
    public override bool NewParameter(in Vertex V, in Edge E, ref double P, ref double Tol)
    {
        return false;
    }

    /// <summary>The continuity of the edge between the faces, unchanged.</summary>
    public override Continuity Continuity(in Edge E,
                                             in Face F1,
                                             in Face F2,
                                             in Edge NewE,
                                             in Face NewF1,
                                             in Face NewF2)
    {
        return Tool.Continuity(E, F1, F2);
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
