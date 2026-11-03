// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Legacy.Tools;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Measurement;
using Kerf.Engine.Offsets;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Fillets.Data.TypeOfConcavity;
using static Kerf.Engine.Fillets.Builder_0;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Engine.Measurement.StatusErrorType;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Fillets;

/// <summary>The fillet package's functions: the concavity of the connection of two faces along an edge.</summary>
/// <remarks>
/// The whole of <c>ChFi3d.cxx</c> is ported (unit 7E, for <see cref="Analyse"/>): DefineConnectType,
/// IsTangentFaces, ConcaveSide, both NextSide, SameSide and the file's Correct2dPoint. They reach only
/// <c>ChFi3d_Coefficient</c> of the builders' free functions and <see cref="LegacyTool.Nt"/>, not the fillet
/// builder (plan 1.3).
/// </remarks>
public static class ChFi3d
{
    //=================================================================================================

    /// <summary>Defines the type of concavity in the edge of connection of two faces.</summary>
    public static TypeOfConcavity DefineConnectType(in Edge e, in Face f1, in Face f2,
                                                           double sinTol, bool correctPoint)
    {
        Surface s1 = Tool.Surface(f1)!;
        Surface s2 = Tool.Surface(f2)!;
        //
        double f, l;
        Curve2D? c1 = Tool.CurveOnSurface(e, f1, out f, out l);
        // For the case of seam edge
        Edge ee = e;
        if (f1.IsSame(f2))
        {
            ee.Reverse();
        }
        Curve2D? c2 = Tool.CurveOnSurface(ee, f2, out f, out l);
        if (c1 is null || c2 is null)
        {
            return TypeOfConcavity.Other;
        }

        EdgeCurveAdaptor c = new(e);
        f = c.FirstParameter();
        l = c.LastParameter();
        //
        double parOnC = 0.5 * (f + l);
        Vector3D t1 = c.DN(parOnC, 1);
        if (t1.SquareMagnitude() <= Constants.Resolution())
        {
            parOnC = IntersectionTools.IntermediatePoint(f, l);
            t1 = c.DN(parOnC, 1);
        }
        if (t1.SquareMagnitude() > Constants.Resolution())
        {
            t1.Normalize();
        }

        if (BrepTools.OriEdgeInFace(e, f1) == Reversed)
        {
            t1.Reverse();
        }
        if (f1.Orientation() == Reversed)
        {
            t1.Reverse();
        }

        Point2D p = c1.Value(parOnC);
        Point3D p3;
        Vector3D d1U, d1V;

        if (correctPoint)
        {
            Correct2dPoint(f1, ref p);
        }
        //
        s1.D1(p.X(), p.Y(), out p3, out d1U, out d1V);
        Vector3D dn1 = d1U ^ d1V;
        if (f1.Orientation() == Reversed)
        {
            dn1.Reverse();
        }

        p = c2.Value(parOnC);
        if (correctPoint)
        {
            Correct2dPoint(f2, ref p);
        }
        s2.D1(p.X(), p.Y(), out p3, out d1U, out d1V);
        Vector3D dn2 = d1U ^ d1V;
        if (f2.Orientation() == Reversed)
        {
            dn2.Reverse();
        }

        dn1.Normalize();
        dn2.Normalize();

        Vector3D proVec = dn1 ^ dn2;
        double normProVec = proVec.Magnitude();
        if (normProVec < sinTol)
        {
            // plane
            if (dn1.Dot(dn2) > 0)
            {
                // Tangent
                return TypeOfConcavity.Tangential;
            }
            else
            {
                // Mixed not finished! (OCCT_DEBUG trace left out)
                return TypeOfConcavity.Convex;
            }
        }
        else
        {
            if (normProVec > Constants.Resolution())
            {
                proVec /= normProVec;
            }
            double prod = t1.Dot(proVec);
            if (prod > 0.0)
            {
                //
                return TypeOfConcavity.Convex;
            }
            else
            {
                // reenters
                return TypeOfConcavity.Concave;
            }
        }
    }

    //=================================================================================================

    /// <summary>True if <paramref name="edge"/> between <paramref name="face1"/> and <paramref name="face2"/> is tangent.</summary>
    public static bool IsTangentFaces(in Edge edge, in Face face1, in Face face2,
                                      Continuity order = G1)
    {
        if (order == G1 && Tool.Continuity(edge, face1, face2) != C0)
        {
            return true;
        }

        double tolC0 = Max(0.001, 1.5 * Tool.Tolerance(edge));

        double first;
        double last;

        Curve2D? c2d1, c2d2;

        if (!face1.IsSame(face2) && Tool.IsClosed(edge, face1)
            && Tool.IsClosed(edge, face2))
        {
            // Find the edge in the face 1: this edge will have correct orientation
            Edge edgeInFace1 = new();
            Face aFace1 = face1;
            aFace1.Orientation(Forward);
            Explorer explo = new(aFace1, ShapeType.Edge);
            for (; explo.More(); explo.Next())
            {
                Edge anEdge = ShapeCasts.Edge(explo.Current());
                if (anEdge.IsSame(edge))
                {
                    edgeInFace1 = anEdge;
                    break;
                }
            }
            if (edgeInFace1.IsNull)
            {
                return false;
            }

            c2d1 = Tool.CurveOnSurface(edgeInFace1, aFace1, out first, out last);
            Face aFace2 = face2;
            aFace2.Orientation(Forward);
            edgeInFace1.Reverse();
            c2d2 = Tool.CurveOnSurface(edgeInFace1, aFace2, out first, out last);
        }
        else
        {
            // Obtaining of pcurves of edge on two faces.
            c2d1 = Tool.CurveOnSurface(edge, face1, out first, out last);
            // For the case of seam edge
            Edge ee = edge;
            if (face1.IsSame(face2))
            {
                ee.Reverse();
            }
            c2d2 = Tool.CurveOnSurface(ee, face2, out first, out last);
        }

        if (c2d1 is null || c2d2 is null)
        {
            return false;
        }

        // Obtaining of two surfaces from adjacent faces.
        Surface? surf1 = Tool.Surface(face1);
        Surface? surf2 = Tool.Surface(face2);

        if (surf1 is null || surf2 is null)
        {
            return false;
        }

        // Computation of the number of samples on the edge.
        // (OCCT copies the adaptors into handles; the port makes them on the faces directly.)
        FaceSurfaceAdaptor bahs1 = new(face1);
        FaceSurfaceAdaptor bahs2 = new(face2);
        BrepTopolTool tool1 = new(bahs1);
        BrepTopolTool tool2 = new(bahs2);
        int nbSamples1 = tool1.NbSamples();
        int nbSamples2 = tool2.NbSamples();
        int nbSamples = Max(nbSamples1, nbSamples2);

        // Computation of the continuity.
        double par;
        double delta = (last - first) / (nbSamples - 1);
        int i, nbNotDone = 0;

        for (i = 1, par = first; i <= nbSamples; i++, par += delta)
        {
            if (i == nbSamples)
            {
                par = last;
            }

            SurfaceContinuity cont =
                new(c2d1, c2d2, par, surf1, surf2, order, 0.001, tolC0, 0.1, 0.1, 0.1);
            if (!cont.IsDone)
            {
                if (order == C2 && cont.StatusError() == StatusErrorType.NullSecondDerivative)
                {
                    continue;
                }

                nbNotDone++;
                continue;
            }

            if (order == G1)
            {
                if (!cont.IsG1())
                {
                    return false;
                }
            }
            else if (!cont.IsG2())
            {
                return false;
            }
        }

        if (nbNotDone == nbSamples)
        {
            return false;
        }

        // Compare normals of tangent faces in the middle point
        double midPar = (first + last) / 2.0;
        Point2D uv1 = c2d1.Value(midPar);
        Point2D uv2 = c2d2.Value(midPar);
        Direction3D normal1 = new(), normal2 = new();
        LegacyTool.Nt(uv1, face1, ref normal1);
        LegacyTool.Nt(uv2, face2, ref normal2);
        double dot = normal1.Dot(normal2);
        return dot >= 0.0;
    }

    //=======================================================================
    // function : ConcaveSide
    // purpose  : calculate the concave face at the neighborhood of the border of
    //           2 faces.
    //=======================================================================

    /// <summary>
    /// Returns Reversed in <paramref name="or1"/> and (or) <paramref name="or2"/> if the concave edge defined by the
    /// interior of the faces of <paramref name="s1"/> and <paramref name="s2"/>, in the neighbourhood of their boundary
    /// <paramref name="e"/>, is opposite to the normal of their surface. The faces' orientations are not taken into
    /// account. Returns 0 if the calculation fails (tangency), otherwise the number of the choice of fillet or chamfer
    /// matching the orientations computed and the tangent to the guide line read on <paramref name="e"/>.
    /// </summary>
    public static int ConcaveSide(FaceSurfaceAdaptor s1, FaceSurfaceAdaptor s2, in Edge e,
                                  out Orientation or1, out Orientation or2)
    {
        int choixConge;
        or1 = or2 = Forward;
        EdgeCurveAdaptor ce = new(e);
        double first = ce.FirstParameter();
        double last = ce.LastParameter();
        double par = 0.691254 * first + 0.308746 * last;

        Point3D pt, pt1, pt2;
        Vector3D tgE, tgE1, tgE2, ns1, ns2, dint1, dint2;
        Face f1 = s1.Face();
        Face f2 = s2.Face();
        // F1.Orientation(TopAbs_FORWARD);
        // F2.Orientation(TopAbs_FORWARD);

        ce.D1(par, out pt, out tgE);
        tgE.Normalize();
        tgE2 = tgE1 = tgE;
        if (e.Orientation() == Reversed)
        {
            tgE.Reverse();
        }

        Edge e1 = e, e2 = e;
        e1.Orientation(Forward);
        e2.Orientation(Forward);

        if (f1.IsSame(f2) && Tool.IsClosed(e, f1))
        {
            e2.Orientation(Reversed);
            tgE2.Reverse();
        }
        else
        {
            Explorer exp = new();
            bool found = false;
            for (exp.Init(f1, ShapeType.Edge); exp.More() && !found; exp.Next())
            {
                if (e.IsSame(ShapeCasts.Edge(exp.Current())))
                {
                    if (exp.Current().Orientation() == Reversed)
                    {
                        tgE1.Reverse();
                    }
                    found = true;
                }
            }
            if (!found)
            {
                return 0;
            }
            found = false;
            for (exp.Init(f2, ShapeType.Edge); exp.More() && !found; exp.Next())
            {
                if (e.IsSame(ShapeCasts.Edge(exp.Current())))
                {
                    if (exp.Current().Orientation() == Reversed)
                    {
                        tgE2.Reverse();
                    }
                    found = true;
                }
            }
            if (!found)
            {
                return 0;
            }
        }
        EdgeCurve2DAdaptor pc1 = new(e1, f1);
        EdgeCurve2DAdaptor pc2 = new(e2, f2);
        Point2D p2d1, p2d2;
        Vector3D du1, dv1, du2, dv2;
        p2d1 = pc1.Value(par);
        p2d2 = pc2.Value(par);
        s1.D1(p2d1.X(), p2d1.Y(), out pt1, out du1, out dv1);
        ns1 = du1.Crossed(dv1);
        ns1.Normalize();
        if (f1.Orientation() == Reversed)
        {
            ns1.Reverse();
        }
        s2.D1(p2d2.X(), p2d2.Y(), out pt2, out du2, out dv2);
        ns2 = du2.Crossed(dv2);
        ns2.Normalize();
        if (f2.Orientation() == Reversed)
        {
            ns2.Reverse();
        }

        dint1 = ns1.Crossed(tgE1);
        dint2 = ns2.Crossed(tgE2);
        double ang = ns1.CrossMagnitude(ns2);
        if (ang > 0.0001 * M_PI)
        {
            double scal = ns2.Dot(dint1);
            if (scal <= 0.0)
            {
                ns2.Reverse();
                or2 = Reversed;
            }
            scal = ns1.Dot(dint2);
            if (scal <= 0.0)
            {
                ns1.Reverse();
                or1 = Reversed;
            }
        }
        else
        {
            // the faces are locally tangent - this is fake!
            if (dint1.Dot(dint2) < 0.0)
            {
                // This is a forgotten regularity
                Vector3D ddu, ddv, dduv;
                s1.D2(p2d1.X(), p2d1.Y(), out pt1, out du1, out dv1, out ddu, out ddv, out dduv);
                du1 += (du1 * dint1 < 0) ? -ddu : ddu;
                dv1 += (dv1 * dint1 < 0) ? -ddv : ddv;
                ns1 = du1.Crossed(dv1);
                ns1.Normalize();
                if (f1.Orientation() == Reversed)
                {
                    ns1.Reverse();
                }
                s2.D2(p2d2.X(), p2d2.Y(), out pt2, out du2, out dv2, out ddu, out ddv, out dduv);
                du2 += (du2 * dint2 < 0) ? -ddu : ddu;
                dv2 += (dv2 * dint2 < 0) ? -ddv : ddv;
                ns2 = du2.Crossed(dv2);
                ns2.Normalize();
                if (f2.Orientation() == Reversed)
                {
                    ns2.Reverse();
                }

                dint1 = ns1.Crossed(tgE1);
                dint2 = ns2.Crossed(tgE2);
                ang = ns1.CrossMagnitude(ns2);
                if (ang > 0.0001 * M_PI)
                {
                    double scal = ns2.Dot(dint1);
                    if (scal <= 0.0)
                    {
                        ns2.Reverse();
                        or2 = Reversed;
                    }
                    scal = ns1.Dot(dint2);
                    if (scal <= 0.0)
                    {
                        ns1.Reverse();
                        or1 = Reversed;
                    }
                }
                else
                {
                    // (OCCT_DEBUG trace "ConcaveSide : no concave face" left out)
                    // This 10 shows that the face at end is in the extension of one of two base faces
                    return 10;
                }
            }
            else
            {
                // here it turns back, the points are taken in faces
                // neither too close nor too far as much as possible.
                double u, v;
                ChFi3d_Coefficient(dint1, du1, dv1, out u, out v);
                p2d1.SetX(p2d1.X() + u);
                p2d1.SetY(p2d1.Y() + v);
                ChFi3d_Coefficient(dint1, du2, dv2, out u, out v);
                p2d2.SetX(p2d2.X() + u);
                p2d2.SetY(p2d2.Y() + v);
                s1.D1(p2d1.X(), p2d1.Y(), out pt1, out du1, out dv1);
                ns1 = du1.Crossed(dv1);
                if (f1.Orientation() == Reversed)
                {
                    ns1.Reverse();
                }
                s2.D1(p2d2.X(), p2d2.Y(), out pt2, out du2, out dv2);
                ns2 = du2.Crossed(dv2);
                if (f2.Orientation() == Reversed)
                {
                    ns2.Reverse();
                }
                Vector3D vref = new(pt1, pt2);
                if (ns1.Dot(vref) < 0.0)
                {
                    or1 = Reversed;
                }
                if (ns2.Dot(vref) > 0.0)
                {
                    or2 = Reversed;
                }
            }
        }

        if (or1 == Forward)
        {
            if (or2 == Forward)
            {
                choixConge = 1;
            }
            else
            {
                choixConge = 7;
            }
        }
        else
        {
            if (or2 == Forward)
            {
                choixConge = 3;
            }
            else
            {
                choixConge = 5;
            }
        }
        if ((ns1.Crossed(ns2)).Dot(tgE) >= 0.0)
        {
            choixConge++;
        }
        return choixConge;
    }

    //=================================================================================================

    /// <summary>
    /// As <see cref="ConcaveSide"/>, with the orientations deduced from the result of <see cref="ConcaveSide"/> on the
    /// first pair of faces of the fillet or chamfer.
    /// </summary>
    public static int NextSide(ref Orientation or1, ref Orientation or2, Orientation orSave1,
                               Orientation orSave2, int choixSave)
    {
        if (or1 == Forward)
        {
            or1 = orSave1;
        }
        else
        {
            or1 = global::Kerf.Engine.Topology.TopAbs.Reverse(orSave1);
        }
        if (or2 == Forward)
        {
            or2 = orSave2;
        }
        else
        {
            or2 = global::Kerf.Engine.Topology.TopAbs.Reverse(orSave2);
        }

        int choixConge;
        if (or1 == Forward)
        {
            if (or2 == Forward)
            {
                choixConge = 1;
            }
            else
            {
                if (choixSave < 0)
                {
                    choixConge = 3;
                }
                else
                {
                    choixConge = 7;
                }
            }
        }
        else
        {
            if (or2 == Forward)
            {
                if (choixSave < 0)
                {
                    choixConge = 7;
                }
                else
                {
                    choixConge = 3;
                }
            }
            else
            {
                choixConge = 5;
            }
        }
        if (Math.Abs(choixSave) % 2 == 0)
        {
            choixConge++;
        }
        return choixConge;
    }

    //=================================================================================================

    /// <summary>As the other NextSide, on an edge only.</summary>
    public static void NextSide(ref Orientation or, Orientation orSave, Orientation orFace)
    {
        if (or == orFace)
        {
            or = orSave;
        }
        else
        {
            or = global::Kerf.Engine.Topology.TopAbs.Reverse(orSave);
        }
    }

    //=================================================================================================

    /// <summary>
    /// While processing a corner, true if two fillets or chamfers making a face have the same concave sides, false if
    /// opposite ones.
    /// </summary>
    public static bool SameSide(Orientation or, Orientation orSave1, Orientation orSave2,
                                Orientation orFace1, Orientation orFace2)
    {
        Orientation o1, o2;
        if (or == orFace1)
        {
            o1 = orSave1;
        }
        else
        {
            o1 = global::Kerf.Engine.Topology.TopAbs.Reverse(orSave1);
        }
        if (or == orFace2)
        {
            o2 = orSave2;
        }
        else
        {
            o2 = global::Kerf.Engine.Topology.TopAbs.Reverse(orSave2);
        }
        return (o1 == o2);
    }

    //=================================================================================================

    private static void Correct2dPoint(in Face f, ref Point2D p2d)
    {
        FaceSurfaceAdaptor bas = new(f, false);
        if (bas.GetType() < SurfaceKind.BezierSurface)
        {
            return;
        }
        //
        const double coeff = 0.01;
        double eps;
        double u1, u2, v1, v2;
        //
        bas.Initialize(f, true);
        u1 = bas.FirstUParameter();
        u2 = bas.LastUParameter();
        v1 = bas.FirstVParameter();
        v2 = bas.LastVParameter();
        if (!(global::Kerf.Engine.Foundation.Precision.IsInfinite(u1) || global::Kerf.Engine.Foundation.Precision.IsInfinite(u2)))
        {
            eps = Max(coeff * (u2 - u1), global::Kerf.Engine.Foundation.Precision.PConfusion());
            if (Math.Abs(p2d.X() - u1) < eps)
            {
                p2d.SetX(u1 + eps);
            }
            if (Math.Abs(p2d.X() - u2) < eps)
            {
                p2d.SetX(u2 - eps);
            }
        }
        if (!(global::Kerf.Engine.Foundation.Precision.IsInfinite(v1) || global::Kerf.Engine.Foundation.Precision.IsInfinite(v2)))
        {
            eps = Max(coeff * (v2 - v1), global::Kerf.Engine.Foundation.Precision.PConfusion());
            if (Math.Abs(p2d.Y() - v1) < eps)
            {
                p2d.SetY(v1 + eps);
            }
            if (Math.Abs(p2d.Y() - v2) < eps)
            {
                p2d.SetY(v2 - eps);
            }
        }
    }
}
