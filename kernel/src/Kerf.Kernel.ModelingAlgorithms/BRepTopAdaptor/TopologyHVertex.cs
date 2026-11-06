// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepTopAdaptor/BRepTopAdaptor_HVertex.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepTopAdaptor/BRepTopAdaptor_HVertex.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Topology.Adaptors;

/// <summary>A vertex of a face's edge, seen by the intersection algorithms through BRepTopAdaptor_TopolTool.</summary>
/// <remarks>A working object (decision 101), keeping the vertex and the edge's 2D adaptor.</remarks>
internal class TopologyHVertex : HVertex
{
    private Vertex _vtx;
    private readonly EdgeCurve2DAdaptor _curve;

    /// <summary>The vertex <paramref name="vtx"/> of the edge of <paramref name="curve"/>.</summary>
    public TopologyHVertex(in Vertex vtx, EdgeCurve2DAdaptor curve)
    {
        _vtx = vtx;
        _curve = curve;
    }

    /// <summary>The vertex.</summary>
    public Vertex Vertex() => _vtx;

    /// <summary>OCCT's non-const access to the vertex.</summary>
    internal ref Vertex ChangeVertex() => ref _vtx;

    /// <summary>The 2D curve the vertex was found on.</summary>
    internal EdgeCurve2DAdaptor Curve() => _curve;

    /// <summary>Nothing: (RealFirst, RealFirst), as in OCCT.</summary>
    public override Point2D Value()
    {
        //  return myCurve->Value(Parameter(myCurve));
        return new Point2D(RealFirst(), RealFirst()); // do nothing
    }

    /// <summary>The parameter of the vertex on the edge of <paramref name="c"/> (a BRepAdaptor_Curve2d).</summary>
    public override double Parameter(Curve2DAdaptorBase c)
    {
        var brhc = (EdgeCurve2DAdaptor)c;
        return Tool.Parameter(_vtx, brhc.Edge(), brhc.Face());
    }

    /// <summary>The parametric resolution (2D) of the vertex's tolerance on the edge of <paramref name="c"/>.</summary>
    public override double Resolution(Curve2DAdaptorBase c)
    {
        var brhc = (EdgeCurve2DAdaptor)c;
        Face f = brhc.Face();
        var s = new FaceSurfaceAdaptor(f, false);
        double tv = Tool.Tolerance(_vtx);
        double pp, p = Tool.Parameter(_vtx, brhc.Edge(), brhc.Face());
        Orientation or = Orientation();
        c.D1(p, out Point2D p2d, out Vector2D v2d);
        Vector3D dc = new();
        s.D1(p2d.X, p2d.Y, out Point3D P, out Vector3D du, out Vector3D dv);
        dc = Vector3D.LinearForm(v2d.X, du, v2d.Y, dv);
        double resUV, mag = dc.Magnitude;

        double uRes = s.UResolution(tv);
        double vRes = s.VResolution(tv);
        double tURes = c.Resolution(uRes);
        double tVRes = c.Resolution(vRes);
        double resUV1 = Max(tURes, tVRes);

        if (mag < 1e-12)
        {
            return (resUV1);
        }

        // for lack of better options limit the parametric solution to
        // 10 million*tolerance of the point
        if (tv > 1.0e7 * mag)
        {
            resUV = 1.0e7;
        }
        else
        {
            resUV = tv / mag;
        }

        // Control
        if (or == Reversed)
        {
            pp = p + resUV;
        }
        else
        {
            pp = p - resUV;
        }

        double uMin = c.FirstParameter();
        double uMax = c.LastParameter();
        if (pp > uMax)
        {
            pp = uMax;
        }

        if (pp < uMin)
        {
            pp = uMin;
        }

        c.D0(pp, out p2d);
        s.D0(p2d.X, p2d.Y, out Point3D p1);

        double dist = P.DistanceTo(p1);
        if ((dist > 1e-12) && ((dist > 1.1 * tv) || (dist < 0.8 * tv)))
        {
            // Refine if possible
            double dist1;
            if (or == Reversed)
            {
                pp = p + tv / dist;
            }
            else
            {
                pp = p - tv / dist;
            }

            if (pp > uMax)
            {
                pp = uMax;
            }

            if (pp < uMin)
            {
                pp = uMin;
            }

            c.D1(pp, out p2d, out v2d);
            s.D1(p2d.X, p2d.Y, out p1, out du, out dv);
            dc = Vector3D.LinearForm(v2d.X, du, v2d.Y, dv);
            dist1 = P.DistanceTo(p1);
            if (Math.Abs(dist1 - tv) < Math.Abs(dist - tv))
            {
                // Take the result of interpolation
                resUV = tv / dist;
                dist = dist1;
            }

            mag = dc.Magnitude;
            if (tv > 1.0e7 * mag)
            {
                mag = tv * 1.0e-7;
            }

            if (or == Reversed)
            {
                pp = p + tv / mag;
            }
            else
            {
                pp = p - tv / mag;
            }

            if (pp > uMax)
            {
                pp = uMax;
            }

            if (pp < uMin)
            {
                pp = uMin;
            }

            c.D0(pp, out p2d);
            s.D0(p2d.X, p2d.Y, out p1);
            dist1 = P.DistanceTo(p1);
            if (Math.Abs(dist1 - tv) < Math.Abs(dist - tv))
            {
                // Take the new estimation
                resUV = tv / mag;
            }
        }

        return Min(resUV, resUV1);
    }

    /// <summary>The orientation of the vertex.</summary>
    public override Orientation Orientation() => _vtx.Orientation;

    /// <summary>True if <paramref name="other"/> (a BRepTopAdaptor_HVertex) holds the same vertex.</summary>
    public override bool IsSame(HVertex other)
    {
        var brhv = (TopologyHVertex)other;
        return _vtx.IsSame(brhv.Vertex());
    }
}
