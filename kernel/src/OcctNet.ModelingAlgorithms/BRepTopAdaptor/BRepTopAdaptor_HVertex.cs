// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepTopAdaptor/BRepTopAdaptor_HVertex.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepTopAdaptor/BRepTopAdaptor_HVertex.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;

namespace Occt;

/// <summary>A vertex of a face's edge, seen by the intersection algorithms through BRepTopAdaptor_TopolTool.</summary>
/// <remarks>A working object (decision 101), keeping the vertex and the edge's 2D adaptor.</remarks>
public class BRepTopAdaptor_HVertex : HVertex
{
    private Vertex myVtx;
    private readonly EdgeCurve2DAdaptor myCurve;

    /// <summary>The vertex <paramref name="Vtx"/> of the edge of <paramref name="Curve"/>.</summary>
    public BRepTopAdaptor_HVertex(in Vertex Vtx, EdgeCurve2DAdaptor Curve)
    {
        myVtx = Vtx;
        myCurve = Curve;
    }

    /// <summary>The vertex.</summary>
    public Vertex Vertex() => myVtx;

    /// <summary>OCCT's non-const access to the vertex.</summary>
    internal ref Vertex ChangeVertex() => ref myVtx;

    /// <summary>The 2D curve the vertex was found on.</summary>
    internal EdgeCurve2DAdaptor Curve() => myCurve;

    /// <summary>Nothing: (RealFirst, RealFirst), as in OCCT.</summary>
    public override Point2D Value()
    {
        //  return myCurve->Value(Parameter(myCurve));
        return new Point2D(RealFirst(), RealFirst()); // do nothing
    }

    /// <summary>The parameter of the vertex on the edge of <paramref name="C"/> (a BRepAdaptor_Curve2d).</summary>
    public override double Parameter(Curve2DAdaptorBase C)
    {
        var brhc = (EdgeCurve2DAdaptor)C;
        return Tool.Parameter(myVtx, brhc.Edge(), brhc.Face());
    }

    /// <summary>The parametric resolution (2D) of the vertex's tolerance on the edge of <paramref name="C"/>.</summary>
    public override double Resolution(Curve2DAdaptorBase C)
    {
        var brhc = (EdgeCurve2DAdaptor)C;
        Face F = brhc.Face();
        var S = new FaceSurfaceAdaptor(F, false);
        double tv = Tool.Tolerance(myVtx);
        double pp, p = Tool.Parameter(myVtx, brhc.Edge(), brhc.Face());
        Orientation Or = Orientation();
        C.D1(p, out Point2D p2d, out Vector2D v2d);
        Vector3D DC = new();
        S.D1(p2d.X(), p2d.Y(), out Point3D P, out Vector3D DU, out Vector3D DV);
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
        if (Or == Reversed)
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
        S.D0(p2d.X(), p2d.Y(), out Point3D P1);

        double Dist = P.Distance(P1);
        if ((Dist > 1e-12) && ((Dist > 1.1 * tv) || (Dist < 0.8 * tv)))
        {
            // Refine if possible
            double Dist1;
            if (Or == Reversed)
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

            if (Or == Reversed)
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
    public override Orientation Orientation() => myVtx.Orientation();

    /// <summary>True if <paramref name="Other"/> (a BRepTopAdaptor_HVertex) holds the same vertex.</summary>
    public override bool IsSame(HVertex Other)
    {
        var brhv = (BRepTopAdaptor_HVertex)Other;
        return myVtx.IsSame(brhv.Vertex());
    }
}
