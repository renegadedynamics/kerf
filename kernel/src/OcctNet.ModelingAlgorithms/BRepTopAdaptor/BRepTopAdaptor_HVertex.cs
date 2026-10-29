// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepTopAdaptor/BRepTopAdaptor_HVertex.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepTopAdaptor/BRepTopAdaptor_HVertex.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>A vertex of a face's edge, seen by the intersection algorithms through BRepTopAdaptor_TopolTool.</summary>
/// <remarks>A working object (decision 101), keeping the vertex and the edge's 2D adaptor.</remarks>
public class BRepTopAdaptor_HVertex : Adaptor3d_HVertex
{
    private TopoDS_Vertex myVtx;
    private readonly BRepAdaptor_Curve2d myCurve;

    /// <summary>The vertex <paramref name="Vtx"/> of the edge of <paramref name="Curve"/>.</summary>
    public BRepTopAdaptor_HVertex(in TopoDS_Vertex Vtx, BRepAdaptor_Curve2d Curve)
    {
        myVtx = Vtx;
        myCurve = Curve;
    }

    /// <summary>The vertex.</summary>
    public TopoDS_Vertex Vertex() => myVtx;

    /// <summary>OCCT's non-const access to the vertex.</summary>
    internal ref TopoDS_Vertex ChangeVertex() => ref myVtx;

    /// <summary>The 2D curve the vertex was found on.</summary>
    internal BRepAdaptor_Curve2d Curve() => myCurve;

    /// <summary>Nothing: (RealFirst, RealFirst), as in OCCT.</summary>
    public override gp_Pnt2d Value()
    {
        //  return myCurve->Value(Parameter(myCurve));
        return new gp_Pnt2d(RealFirst(), RealFirst()); // do nothing
    }

    /// <summary>The parameter of the vertex on the edge of <paramref name="C"/> (a BRepAdaptor_Curve2d).</summary>
    public override double Parameter(Adaptor2d_Curve2d C)
    {
        var brhc = (BRepAdaptor_Curve2d)C;
        return BRep_Tool.Parameter(myVtx, brhc.Edge(), brhc.Face());
    }

    /// <summary>The parametric resolution (2D) of the vertex's tolerance on the edge of <paramref name="C"/>.</summary>
    public override double Resolution(Adaptor2d_Curve2d C)
    {
        var brhc = (BRepAdaptor_Curve2d)C;
        TopoDS_Face F = brhc.Face();
        var S = new BRepAdaptor_Surface(F, false);
        double tv = BRep_Tool.Tolerance(myVtx);
        double pp, p = BRep_Tool.Parameter(myVtx, brhc.Edge(), brhc.Face());
        TopAbs_Orientation Or = Orientation();
        C.D1(p, out gp_Pnt2d p2d, out gp_Vec2d v2d);
        gp_Vec DC = new();
        S.D1(p2d.X(), p2d.Y(), out gp_Pnt P, out gp_Vec DU, out gp_Vec DV);
        DC.SetLinearForm(v2d.X(), DU, v2d.Y(), DV);
        double ResUV, mag = DC.Magnitude();

        double URes = S.UResolution(tv);
        double VRes = S.VResolution(tv);
        double tURes = C.Resolution(URes);
        double tVRes = C.Resolution(VRes);
        double ResUV1 = Max(tURes, tVRes);

        if (mag < 1e-12)
        {
            return (ResUV1);
        }

        // for lack of better options limit the parametric solution to
        // 10 million*tolerance of the point
        if (tv > 1.0e7 * mag)
        {
            ResUV = 1.0e7;
        }
        else
        {
            ResUV = tv / mag;
        }

        // Control
        if (Or == TopAbs_REVERSED)
        {
            pp = p + ResUV;
        }
        else
        {
            pp = p - ResUV;
        }

        double UMin = C.FirstParameter();
        double UMax = C.LastParameter();
        if (pp > UMax)
        {
            pp = UMax;
        }

        if (pp < UMin)
        {
            pp = UMin;
        }

        C.D0(pp, out p2d);
        S.D0(p2d.X(), p2d.Y(), out gp_Pnt P1);

        double Dist = P.Distance(P1);
        if ((Dist > 1e-12) && ((Dist > 1.1 * tv) || (Dist < 0.8 * tv)))
        {
            // Refine if possible
            double Dist1;
            if (Or == TopAbs_REVERSED)
            {
                pp = p + tv / Dist;
            }
            else
            {
                pp = p - tv / Dist;
            }

            if (pp > UMax)
            {
                pp = UMax;
            }

            if (pp < UMin)
            {
                pp = UMin;
            }

            C.D1(pp, out p2d, out v2d);
            S.D1(p2d.X(), p2d.Y(), out P1, out DU, out DV);
            DC.SetLinearForm(v2d.X(), DU, v2d.Y(), DV);
            Dist1 = P.Distance(P1);
            if (Math.Abs(Dist1 - tv) < Math.Abs(Dist - tv))
            {
                // Take the result of interpolation
                ResUV = tv / Dist;
                Dist = Dist1;
            }

            mag = DC.Magnitude();
            if (tv > 1.0e7 * mag)
            {
                mag = tv * 1.0e-7;
            }

            if (Or == TopAbs_REVERSED)
            {
                pp = p + tv / mag;
            }
            else
            {
                pp = p - tv / mag;
            }

            if (pp > UMax)
            {
                pp = UMax;
            }

            if (pp < UMin)
            {
                pp = UMin;
            }

            C.D0(pp, out p2d);
            S.D0(p2d.X(), p2d.Y(), out P1);
            Dist1 = P.Distance(P1);
            if (Math.Abs(Dist1 - tv) < Math.Abs(Dist - tv))
            {
                // Take the new estimation
                ResUV = tv / mag;
            }
        }

        return Min(ResUV, ResUV1);
    }

    /// <summary>The orientation of the vertex.</summary>
    public override TopAbs_Orientation Orientation() => myVtx.Orientation();

    /// <summary>True if <paramref name="Other"/> (a BRepTopAdaptor_HVertex) holds the same vertex.</summary>
    public override bool IsSame(Adaptor3d_HVertex Other)
    {
        var brhv = (BRepTopAdaptor_HVertex)Other;
        return myVtx.IsSame(brhv.Vertex());
    }
}
