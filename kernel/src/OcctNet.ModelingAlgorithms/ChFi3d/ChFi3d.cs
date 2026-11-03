// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Occt.ChFiDS_TypeOfConcavity;
using static Occt.ChFi3d_Builder_0;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Occt.LocalAnalysis_StatusErrorType;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>The fillet package's functions: the concavity of the connection of two faces along an edge.</summary>
/// <remarks>
/// The whole of <c>ChFi3d.cxx</c> is ported (unit 7E, for <see cref="BRepOffset_Analyse"/>): DefineConnectType,
/// IsTangentFaces, ConcaveSide, both NextSide, SameSide and the file's Correct2dPoint. They reach only
/// <c>ChFi3d_Coefficient</c> of the builders' free functions and <see cref="TopOpeBRepTool_TOOL.Nt"/>, not the fillet
/// builder (plan 1.3).
/// </remarks>
public static class ChFi3d
{
    //=================================================================================================

    /// <summary>Defines the type of concavity in the edge of connection of two faces.</summary>
    public static ChFiDS_TypeOfConcavity DefineConnectType(in TopoDS_Edge E, in TopoDS_Face F1, in TopoDS_Face F2,
                                                           double SinTol, bool CorrectPoint)
    {
        Geom_Surface S1 = BRep_Tool.Surface(F1)!;
        Geom_Surface S2 = BRep_Tool.Surface(F2)!;
        //
        double f, l;
        Geom2d_Curve? C1 = BRep_Tool.CurveOnSurface(E, F1, out f, out l);
        // For the case of seam edge
        TopoDS_Edge EE = E;
        if (F1.IsSame(F2))
        {
            EE.Reverse();
        }
        Geom2d_Curve? C2 = BRep_Tool.CurveOnSurface(EE, F2, out f, out l);
        if (C1 is null || C2 is null)
        {
            return ChFiDS_Other;
        }

        BRepAdaptor_Curve C = new(E);
        f = C.FirstParameter();
        l = C.LastParameter();
        //
        double ParOnC = 0.5 * (f + l);
        Vector3D T1 = C.DN(ParOnC, 1);
        if (T1.SquareMagnitude() <= Constants.Resolution())
        {
            ParOnC = IntTools_Tools.IntermediatePoint(f, l);
            T1 = C.DN(ParOnC, 1);
        }
        if (T1.SquareMagnitude() > Constants.Resolution())
        {
            T1.Normalize();
        }

        if (BRepTools.OriEdgeInFace(E, F1) == TopAbs_REVERSED)
        {
            T1.Reverse();
        }
        if (F1.Orientation() == TopAbs_REVERSED)
        {
            T1.Reverse();
        }

        Point2D P = C1.Value(ParOnC);
        Point3D P3;
        Vector3D D1U, D1V;

        if (CorrectPoint)
        {
            Correct2dPoint(F1, ref P);
        }
        //
        S1.D1(P.X(), P.Y(), out P3, out D1U, out D1V);
        Vector3D DN1 = D1U ^ D1V;
        if (F1.Orientation() == TopAbs_REVERSED)
        {
            DN1.Reverse();
        }

        P = C2.Value(ParOnC);
        if (CorrectPoint)
        {
            Correct2dPoint(F2, ref P);
        }
        S2.D1(P.X(), P.Y(), out P3, out D1U, out D1V);
        Vector3D DN2 = D1U ^ D1V;
        if (F2.Orientation() == TopAbs_REVERSED)
        {
            DN2.Reverse();
        }

        DN1.Normalize();
        DN2.Normalize();

        Vector3D ProVec = DN1 ^ DN2;
        double NormProVec = ProVec.Magnitude();
        if (NormProVec < SinTol)
        {
            // plane
            if (DN1.Dot(DN2) > 0)
            {
                // Tangent
                return ChFiDS_Tangential;
            }
            else
            {
                // Mixed not finished! (OCCT_DEBUG trace left out)
                return ChFiDS_Convex;
            }
        }
        else
        {
            if (NormProVec > Constants.Resolution())
            {
                ProVec /= NormProVec;
            }
            double Prod = T1.Dot(ProVec);
            if (Prod > 0.0)
            {
                //
                return ChFiDS_Convex;
            }
            else
            {
                // reenters
                return ChFiDS_Concave;
            }
        }
    }

    //=================================================================================================

    /// <summary>True if <paramref name="theEdge"/> between <paramref name="theFace1"/> and <paramref name="theFace2"/> is tangent.</summary>
    public static bool IsTangentFaces(in TopoDS_Edge theEdge, in TopoDS_Face theFace1, in TopoDS_Face theFace2,
                                      Continuity theOrder = G1)
    {
        if (theOrder == G1 && BRep_Tool.Continuity(theEdge, theFace1, theFace2) != C0)
        {
            return true;
        }

        double TolC0 = Max(0.001, 1.5 * BRep_Tool.Tolerance(theEdge));

        double aFirst;
        double aLast;

        Geom2d_Curve? aC2d1, aC2d2;

        if (!theFace1.IsSame(theFace2) && BRep_Tool.IsClosed(theEdge, theFace1)
            && BRep_Tool.IsClosed(theEdge, theFace2))
        {
            // Find the edge in the face 1: this edge will have correct orientation
            TopoDS_Edge anEdgeInFace1 = new();
            TopoDS_Face aFace1 = theFace1;
            aFace1.Orientation(TopAbs_FORWARD);
            TopExp_Explorer anExplo = new(aFace1, TopAbs_EDGE);
            for (; anExplo.More(); anExplo.Next())
            {
                TopoDS_Edge anEdge = TopoDS.Edge(anExplo.Current());
                if (anEdge.IsSame(theEdge))
                {
                    anEdgeInFace1 = anEdge;
                    break;
                }
            }
            if (anEdgeInFace1.IsNull())
            {
                return false;
            }

            aC2d1 = BRep_Tool.CurveOnSurface(anEdgeInFace1, aFace1, out aFirst, out aLast);
            TopoDS_Face aFace2 = theFace2;
            aFace2.Orientation(TopAbs_FORWARD);
            anEdgeInFace1.Reverse();
            aC2d2 = BRep_Tool.CurveOnSurface(anEdgeInFace1, aFace2, out aFirst, out aLast);
        }
        else
        {
            // Obtaining of pcurves of edge on two faces.
            aC2d1 = BRep_Tool.CurveOnSurface(theEdge, theFace1, out aFirst, out aLast);
            // For the case of seam edge
            TopoDS_Edge EE = theEdge;
            if (theFace1.IsSame(theFace2))
            {
                EE.Reverse();
            }
            aC2d2 = BRep_Tool.CurveOnSurface(EE, theFace2, out aFirst, out aLast);
        }

        if (aC2d1 is null || aC2d2 is null)
        {
            return false;
        }

        // Obtaining of two surfaces from adjacent faces.
        Geom_Surface? aSurf1 = BRep_Tool.Surface(theFace1);
        Geom_Surface? aSurf2 = BRep_Tool.Surface(theFace2);

        if (aSurf1 is null || aSurf2 is null)
        {
            return false;
        }

        // Computation of the number of samples on the edge.
        // (OCCT copies the adaptors into handles; the port makes them on the faces directly.)
        BRepAdaptor_Surface aBAHS1 = new(theFace1);
        BRepAdaptor_Surface aBAHS2 = new(theFace2);
        BRepTopAdaptor_TopolTool aTool1 = new(aBAHS1);
        BRepTopAdaptor_TopolTool aTool2 = new(aBAHS2);
        int aNbSamples1 = aTool1.NbSamples();
        int aNbSamples2 = aTool2.NbSamples();
        int aNbSamples = Max(aNbSamples1, aNbSamples2);

        // Computation of the continuity.
        double aPar;
        double aDelta = (aLast - aFirst) / (aNbSamples - 1);
        int i, nbNotDone = 0;

        for (i = 1, aPar = aFirst; i <= aNbSamples; i++, aPar += aDelta)
        {
            if (i == aNbSamples)
            {
                aPar = aLast;
            }

            LocalAnalysis_SurfaceContinuity aCont =
                new(aC2d1, aC2d2, aPar, aSurf1, aSurf2, theOrder, 0.001, TolC0, 0.1, 0.1, 0.1);
            if (!aCont.IsDone())
            {
                if (theOrder == C2 && aCont.StatusError() == LocalAnalysis_NullSecondDerivative)
                {
                    continue;
                }

                nbNotDone++;
                continue;
            }

            if (theOrder == G1)
            {
                if (!aCont.IsG1())
                {
                    return false;
                }
            }
            else if (!aCont.IsG2())
            {
                return false;
            }
        }

        if (nbNotDone == aNbSamples)
        {
            return false;
        }

        // Compare normals of tangent faces in the middle point
        double MidPar = (aFirst + aLast) / 2.0;
        Point2D uv1 = aC2d1.Value(MidPar);
        Point2D uv2 = aC2d2.Value(MidPar);
        Direction3D normal1 = new(), normal2 = new();
        TopOpeBRepTool_TOOL.Nt(uv1, theFace1, ref normal1);
        TopOpeBRepTool_TOOL.Nt(uv2, theFace2, ref normal2);
        double dot = normal1.Dot(normal2);
        return dot >= 0.0;
    }

    //=======================================================================
    // function : ConcaveSide
    // purpose  : calculate the concave face at the neighborhood of the border of
    //           2 faces.
    //=======================================================================

    /// <summary>
    /// Returns Reversed in <paramref name="Or1"/> and (or) <paramref name="Or2"/> if the concave edge defined by the
    /// interior of the faces of <paramref name="S1"/> and <paramref name="S2"/>, in the neighbourhood of their boundary
    /// <paramref name="E"/>, is opposite to the normal of their surface. The faces' orientations are not taken into
    /// account. Returns 0 if the calculation fails (tangency), otherwise the number of the choice of fillet or chamfer
    /// matching the orientations computed and the tangent to the guide line read on <paramref name="E"/>.
    /// </summary>
    public static int ConcaveSide(BRepAdaptor_Surface S1, BRepAdaptor_Surface S2, in TopoDS_Edge E,
                                  out TopAbs_Orientation Or1, out TopAbs_Orientation Or2)
    {
        int ChoixConge;
        Or1 = Or2 = TopAbs_FORWARD;
        BRepAdaptor_Curve CE = new(E);
        double first = CE.FirstParameter();
        double last = CE.LastParameter();
        double par = 0.691254 * first + 0.308746 * last;

        Point3D pt, pt1, pt2;
        Vector3D tgE, tgE1, tgE2, ns1, ns2, dint1, dint2;
        TopoDS_Face F1 = S1.Face();
        TopoDS_Face F2 = S2.Face();
        // F1.Orientation(TopAbs_FORWARD);
        // F2.Orientation(TopAbs_FORWARD);

        CE.D1(par, out pt, out tgE);
        tgE.Normalize();
        tgE2 = tgE1 = tgE;
        if (E.Orientation() == TopAbs_REVERSED)
        {
            tgE.Reverse();
        }

        TopoDS_Edge E1 = E, E2 = E;
        E1.Orientation(TopAbs_FORWARD);
        E2.Orientation(TopAbs_FORWARD);

        if (F1.IsSame(F2) && BRep_Tool.IsClosed(E, F1))
        {
            E2.Orientation(TopAbs_REVERSED);
            tgE2.Reverse();
        }
        else
        {
            TopExp_Explorer Exp = new();
            bool found = false;
            for (Exp.Init(F1, TopAbs_EDGE); Exp.More() && !found; Exp.Next())
            {
                if (E.IsSame(TopoDS.Edge(Exp.Current())))
                {
                    if (Exp.Current().Orientation() == TopAbs_REVERSED)
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
            for (Exp.Init(F2, TopAbs_EDGE); Exp.More() && !found; Exp.Next())
            {
                if (E.IsSame(TopoDS.Edge(Exp.Current())))
                {
                    if (Exp.Current().Orientation() == TopAbs_REVERSED)
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
        BRepAdaptor_Curve2d pc1 = new(E1, F1);
        BRepAdaptor_Curve2d pc2 = new(E2, F2);
        Point2D p2d1, p2d2;
        Vector3D DU1, DV1, DU2, DV2;
        p2d1 = pc1.Value(par);
        p2d2 = pc2.Value(par);
        S1.D1(p2d1.X(), p2d1.Y(), out pt1, out DU1, out DV1);
        ns1 = DU1.Crossed(DV1);
        ns1.Normalize();
        if (F1.Orientation() == TopAbs_REVERSED)
        {
            ns1.Reverse();
        }
        S2.D1(p2d2.X(), p2d2.Y(), out pt2, out DU2, out DV2);
        ns2 = DU2.Crossed(DV2);
        ns2.Normalize();
        if (F2.Orientation() == TopAbs_REVERSED)
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
                Or2 = TopAbs_REVERSED;
            }
            scal = ns1.Dot(dint2);
            if (scal <= 0.0)
            {
                ns1.Reverse();
                Or1 = TopAbs_REVERSED;
            }
        }
        else
        {
            // the faces are locally tangent - this is fake!
            if (dint1.Dot(dint2) < 0.0)
            {
                // This is a forgotten regularity
                Vector3D DDU, DDV, DDUV;
                S1.D2(p2d1.X(), p2d1.Y(), out pt1, out DU1, out DV1, out DDU, out DDV, out DDUV);
                DU1 += (DU1 * dint1 < 0) ? -DDU : DDU;
                DV1 += (DV1 * dint1 < 0) ? -DDV : DDV;
                ns1 = DU1.Crossed(DV1);
                ns1.Normalize();
                if (F1.Orientation() == TopAbs_REVERSED)
                {
                    ns1.Reverse();
                }
                S2.D2(p2d2.X(), p2d2.Y(), out pt2, out DU2, out DV2, out DDU, out DDV, out DDUV);
                DU2 += (DU2 * dint2 < 0) ? -DDU : DDU;
                DV2 += (DV2 * dint2 < 0) ? -DDV : DDV;
                ns2 = DU2.Crossed(DV2);
                ns2.Normalize();
                if (F2.Orientation() == TopAbs_REVERSED)
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
                        Or2 = TopAbs_REVERSED;
                    }
                    scal = ns1.Dot(dint2);
                    if (scal <= 0.0)
                    {
                        ns1.Reverse();
                        Or1 = TopAbs_REVERSED;
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
                ChFi3d_Coefficient(dint1, DU1, DV1, out u, out v);
                p2d1.SetX(p2d1.X() + u);
                p2d1.SetY(p2d1.Y() + v);
                ChFi3d_Coefficient(dint1, DU2, DV2, out u, out v);
                p2d2.SetX(p2d2.X() + u);
                p2d2.SetY(p2d2.Y() + v);
                S1.D1(p2d1.X(), p2d1.Y(), out pt1, out DU1, out DV1);
                ns1 = DU1.Crossed(DV1);
                if (F1.Orientation() == TopAbs_REVERSED)
                {
                    ns1.Reverse();
                }
                S2.D1(p2d2.X(), p2d2.Y(), out pt2, out DU2, out DV2);
                ns2 = DU2.Crossed(DV2);
                if (F2.Orientation() == TopAbs_REVERSED)
                {
                    ns2.Reverse();
                }
                Vector3D vref = new(pt1, pt2);
                if (ns1.Dot(vref) < 0.0)
                {
                    Or1 = TopAbs_REVERSED;
                }
                if (ns2.Dot(vref) > 0.0)
                {
                    Or2 = TopAbs_REVERSED;
                }
            }
        }

        if (Or1 == TopAbs_FORWARD)
        {
            if (Or2 == TopAbs_FORWARD)
            {
                ChoixConge = 1;
            }
            else
            {
                ChoixConge = 7;
            }
        }
        else
        {
            if (Or2 == TopAbs_FORWARD)
            {
                ChoixConge = 3;
            }
            else
            {
                ChoixConge = 5;
            }
        }
        if ((ns1.Crossed(ns2)).Dot(tgE) >= 0.0)
        {
            ChoixConge++;
        }
        return ChoixConge;
    }

    //=================================================================================================

    /// <summary>
    /// As <see cref="ConcaveSide"/>, with the orientations deduced from the result of <see cref="ConcaveSide"/> on the
    /// first pair of faces of the fillet or chamfer.
    /// </summary>
    public static int NextSide(ref TopAbs_Orientation Or1, ref TopAbs_Orientation Or2, TopAbs_Orientation OrSave1,
                               TopAbs_Orientation OrSave2, int ChoixSave)
    {
        if (Or1 == TopAbs_FORWARD)
        {
            Or1 = OrSave1;
        }
        else
        {
            Or1 = TopAbs.Reverse(OrSave1);
        }
        if (Or2 == TopAbs_FORWARD)
        {
            Or2 = OrSave2;
        }
        else
        {
            Or2 = TopAbs.Reverse(OrSave2);
        }

        int ChoixConge;
        if (Or1 == TopAbs_FORWARD)
        {
            if (Or2 == TopAbs_FORWARD)
            {
                ChoixConge = 1;
            }
            else
            {
                if (ChoixSave < 0)
                {
                    ChoixConge = 3;
                }
                else
                {
                    ChoixConge = 7;
                }
            }
        }
        else
        {
            if (Or2 == TopAbs_FORWARD)
            {
                if (ChoixSave < 0)
                {
                    ChoixConge = 7;
                }
                else
                {
                    ChoixConge = 3;
                }
            }
            else
            {
                ChoixConge = 5;
            }
        }
        if (Math.Abs(ChoixSave) % 2 == 0)
        {
            ChoixConge++;
        }
        return ChoixConge;
    }

    //=================================================================================================

    /// <summary>As the other NextSide, on an edge only.</summary>
    public static void NextSide(ref TopAbs_Orientation Or, TopAbs_Orientation OrSave, TopAbs_Orientation OrFace)
    {
        if (Or == OrFace)
        {
            Or = OrSave;
        }
        else
        {
            Or = TopAbs.Reverse(OrSave);
        }
    }

    //=================================================================================================

    /// <summary>
    /// While processing a corner, true if two fillets or chamfers making a face have the same concave sides, false if
    /// opposite ones.
    /// </summary>
    public static bool SameSide(TopAbs_Orientation Or, TopAbs_Orientation OrSave1, TopAbs_Orientation OrSave2,
                                TopAbs_Orientation OrFace1, TopAbs_Orientation OrFace2)
    {
        TopAbs_Orientation o1, o2;
        if (Or == OrFace1)
        {
            o1 = OrSave1;
        }
        else
        {
            o1 = TopAbs.Reverse(OrSave1);
        }
        if (Or == OrFace2)
        {
            o2 = OrSave2;
        }
        else
        {
            o2 = TopAbs.Reverse(OrSave2);
        }
        return (o1 == o2);
    }

    //=================================================================================================

    private static void Correct2dPoint(in TopoDS_Face theF, ref Point2D theP2d)
    {
        BRepAdaptor_Surface aBAS = new(theF, false);
        if (aBAS.GetType() < BezierSurface)
        {
            return;
        }
        //
        const double coeff = 0.01;
        double eps;
        double u1, u2, v1, v2;
        //
        aBAS.Initialize(theF, true);
        u1 = aBAS.FirstUParameter();
        u2 = aBAS.LastUParameter();
        v1 = aBAS.FirstVParameter();
        v2 = aBAS.LastVParameter();
        if (!(global::Kerf.Engine.Foundation.Precision.IsInfinite(u1) || global::Kerf.Engine.Foundation.Precision.IsInfinite(u2)))
        {
            eps = Max(coeff * (u2 - u1), global::Kerf.Engine.Foundation.Precision.PConfusion());
            if (Math.Abs(theP2d.X() - u1) < eps)
            {
                theP2d.SetX(u1 + eps);
            }
            if (Math.Abs(theP2d.X() - u2) < eps)
            {
                theP2d.SetX(u2 - eps);
            }
        }
        if (!(global::Kerf.Engine.Foundation.Precision.IsInfinite(v1) || global::Kerf.Engine.Foundation.Precision.IsInfinite(v2)))
        {
            eps = Max(coeff * (v2 - v1), global::Kerf.Engine.Foundation.Precision.PConfusion());
            if (Math.Abs(theP2d.Y() - v1) < eps)
            {
                theP2d.SetY(v1 + eps);
            }
            if (Math.Abs(theP2d.Y() - v2) < eps)
            {
                theP2d.SetY(v2 - eps);
            }
        }
    }
}
