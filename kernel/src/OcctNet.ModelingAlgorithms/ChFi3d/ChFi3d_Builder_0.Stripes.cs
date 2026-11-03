// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder_0.cxx (ChFi3d_StripeEdgeInter to CurveCleaner)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;

namespace Occt;

internal static partial class ChFi3d_Builder_0
{
    //=======================================================================
    // function : StripeEdgeInter
    // purpose  : This function examines two stripes for an intersection
    //           between curves of interference with faces. If the intersection
    //           exists, it will cause bad result, so it's better to quit.
    // remark   : If someone somewhen computes the interference between stripes,
    //           this function will become useless.
    // author   : akm, 06/02/02. Against bug OCC119.
    //=======================================================================

    /// <summary>Throws when the traces on a common face of two stripes without a common corner cross.</summary>
    /// <exception cref="NotDoneException">The traces cross: the fillets' radii are too big.</exception>
    internal static void ChFi3d_StripeEdgeInter(ChFiDS_Stripe theStripe1, ChFiDS_Stripe theStripe2,
                                                TopOpeBRepDS_DataStructure DStr, double tol2d)
    {
        // Do not check the stripeshaving common corner points
        for (int iSur1 = 1; iSur1 <= 2; iSur1++)
        {
            for (int iSur2 = 1; iSur2 <= 2; iSur2++)
            {
                if (theStripe1.IndexPoint(false, iSur1) == theStripe2.IndexPoint(false, iSur2)
                    || theStripe1.IndexPoint(false, iSur1) == theStripe2.IndexPoint(true, iSur2)
                    || theStripe1.IndexPoint(true, iSur1) == theStripe2.IndexPoint(false, iSur2)
                    || theStripe1.IndexPoint(true, iSur1) == theStripe2.IndexPoint(true, iSur2))
                {
                    return;
                }
            }
        }

        NCollection_HSequence<ChFiDS_SurfData> aSurDat1 = theStripe1.SetOfSurfData()!;
        NCollection_HSequence<ChFiDS_SurfData> aSurDat2 = theStripe2.SetOfSurfData()!;

        Geom2dInt_GInter anIntersector = new();
        int iPart1, iPart2;
        int Ishape11, Ishape12, Ishape21, Ishape22;
        // Loop on parts of the first stripe
        for (iPart1 = 1; iPart1 <= aSurDat1.Length(); iPart1++)
        {
            ChFiDS_SurfData aDat1 = aSurDat1.Value(iPart1);
            Ishape11 = aDat1.IndexOfS1();
            Ishape12 = aDat1.IndexOfS2();
            // Loop on parts of the second stripe
            for (iPart2 = 1; iPart2 <= aSurDat2.Length(); iPart2++)
            {
                ChFiDS_SurfData aDat2 = aSurDat2.Value(iPart2);
                Ishape21 = aDat2.IndexOfS1();
                Ishape22 = aDat2.IndexOfS2();

                // Find those FaceInterferences able to intersect
                ChFiDS_FaceInterference aFI1, aFI2;
                if (Ishape11 == Ishape21)
                {
                    aFI1 = aDat1.InterferenceOnS1();
                    aFI2 = aDat2.InterferenceOnS1();
                }
                else if (Ishape11 == Ishape22)
                {
                    aFI1 = aDat1.InterferenceOnS1();
                    aFI2 = aDat2.InterferenceOnS2();
                }
                else if (Ishape12 == Ishape21)
                {
                    aFI1 = aDat1.InterferenceOnS2();
                    aFI2 = aDat2.InterferenceOnS1();
                }
                else if (Ishape12 == Ishape22)
                {
                    aFI1 = aDat1.InterferenceOnS2();
                    aFI2 = aDat2.InterferenceOnS2();
                }
                else
                {
                    // No common faces
                    continue;
                }

                if (aFI1.FirstParameter() == aFI1.LastParameter() || aFI2.FirstParameter() == aFI2.LastParameter()
                    || aFI1.PCurveOnFace() is null || aFI2.PCurveOnFace() is null)
                {
                    // Do not waste time on degenerates
                    continue;
                }
                // Examine for intersections
                CurveAdaptor2D aPCurve1 = new(aFI1.PCurveOnFace()!, aFI1.FirstParameter(),
                                                   aFI1.LastParameter());
                CurveAdaptor2D aPCurve2 = new(aFI2.PCurveOnFace()!, aFI2.FirstParameter(),
                                                   aFI2.LastParameter());
                anIntersector.Perform(aPCurve1, aPCurve2, tol2d, global::Kerf.Engine.Foundation.Precision.PConfusion());
                if (anIntersector.NbSegments() > 0 || anIntersector.NbPoints() > 0)
                {
                    throw new NotDoneException("StripeEdgeInter : fillets have too big radiuses");
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>The index of the stripe's surface data at its end on <paramref name="V1"/> (1 or the last), and the
    /// sense (1 at the start, -1 at the end); 0 when the stripe has no surface data.</summary>
    /// <exception cref="ConstructionErrorException"><paramref name="V1"/> is not an end of the spine.</exception>
    internal static int ChFi3d_IndexOfSurfData(in Vertex V1, ChFiDS_Stripe CD, out int sens)
    {
        ChFiDS_Spine spine = CD.Spine()!;
        int Index = 0;
        sens = 1;
        Vertex Vref;
        Edge E = spine.Edges(1);
        if (E.Orientation() == Reversed)
        {
            Vref = Exploration.LastVertex(E);
        }
        else
        {
            Vref = Exploration.FirstVertex(E);
        }
        if (Vref.IsSame(V1))
        {
            Index = 1;
        }
        else
        {
            Edge E1 = spine.Edges(spine.NbEdges());
            if (E1.Orientation() == Reversed)
            {
                Vref = Exploration.FirstVertex(E1);
            }
            else
            {
                Vref = Exploration.LastVertex(E1);
            }
            sens = -1;
            if (CD.SetOfSurfData() is null)
            {
                return 0;
            }
            else if (Vref.IsSame(V1))
            {
                Index = CD.SetOfSurfData()!.Length();
            }
            else
            {
                throw new ConstructionErrorException("ChFi3d_IndexOfSurfData() - wrong construction parameters");
            }
        }
        return Index;
    }

    //=================================================================================================

    /// <summary>The spine's edge at its end on <paramref name="V1"/>, and the sense (1 at the start, -1 at the
    /// end).</summary>
    /// <exception cref="ConstructionErrorException"><paramref name="V1"/> is not an end of the spine.</exception>
    internal static Edge ChFi3d_EdgeFromV1(in Vertex V1, ChFiDS_Stripe CD, out int sens)
    {
        ChFiDS_Spine spine = CD.Spine()!;
        sens = 1;
        Vertex Vref;
        Edge E = spine.Edges(1);
        if (E.Orientation() == Reversed)
        {
            Vref = Exploration.LastVertex(E);
        }
        else
        {
            Vref = Exploration.FirstVertex(E);
        }
        if (Vref.IsSame(V1))
        {
            return E;
        }
        else
        {
            Edge E1 = spine.Edges(spine.NbEdges());
            if (E1.Orientation() == Reversed)
            {
                Vref = Exploration.FirstVertex(E1);
            }
            else
            {
                Vref = Exploration.LastVertex(E1);
            }
            sens = -1;
            if (Vref.IsSame(V1))
            {
                return E1;
            }
            else
            {
                throw new ConstructionErrorException("ChFi3d_IndexOfSurfData() - wrong construction parameters");
            }
        }
    }

    //=================================================================================================

    /// <summary>A 3D tolerance for the 2D tolerance <paramref name="tol2d"/> on <paramref name="S"/>, by its
    /// resolutions.</summary>
    internal static double ChFi3d_ConvTol2dToTol3d(SurfaceAdaptorBase S, double tol2d)
    {
        double ures = S.UResolution(1.0e-7);
        double vres = S.VResolution(1.0e-7);
        double uresto3d = 1.0e-7 * tol2d / ures;
        double vresto3d = 1.0e-7 * tol2d / vres;
        return Math.Max(uresto3d, vresto3d);
    }

    //=======================================================================
    // function : EvalTolReached
    // purpose  : The function above is too hard because
    //           parametrization of surfaces is not homogeneous.
    //=======================================================================

    /// <summary>1.5 times the largest distance between <paramref name="C"/> and its pcurves on the surfaces, at 45
    /// points (at least the confusion).</summary>
    internal static double ChFi3d_EvalTolReached(SurfaceAdaptorBase S1, Curve2D pc1, SurfaceAdaptorBase S2,
                                                 Curve2D pc2, Curve3D C)
    {
        double distmax = 0.0;

        double f = C.FirstParameter();
        double l = C.LastParameter();
        int nbp = 45;
        double step = 1.0 / (nbp - 1);
        for (int i = 0; i < nbp; i++)
        {
            double t, u, v;
            t = step * i;
            t = (1 - t) * f + t * l;
            pc1.Value(t).Coord(out u, out v);
            Point3D pS1 = S1.Value(u, v);
            pc2.Value(t).Coord(out u, out v);
            Point3D pS2 = S2.Value(u, v);
            Point3D pC = C.Value(t);
            double d = pS1.SquareDistance(pC);
            if (d > distmax)
            {
                distmax = d;
            }
            d = pS2.SquareDistance(pC);
            if (d > distmax)
            {
                distmax = d;
            }
            d = pS1.SquareDistance(pS2);
            if (d > distmax)
            {
                distmax = d;
            }
        }
        distmax = 1.5 * Math.Sqrt(distmax);
        distmax = Math.Max(distmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        return distmax;
    }

    //=================================================================================================

    /// <summary>The surface under the adaptor, trimmed to the adaptor's bounds (within the surface's own).</summary>
    internal static Surface? trsfsurf(SurfaceAdaptorBase HS, ref TopolTool? dom)
    {
        // Pour l utilisation des domaines voir avec BUBUCH!!
        Surface? res = null;
        if (HS is FaceSurfaceAdaptor hbs)
        {
            res = hbs.GeomSurfaceTransformed();
        }
        else if (HS is SurfaceAdaptor hgs)
        {
            res = hgs.Surface();
        }
        if (res is TrimmedSurface tr)
        {
            res = tr.BasisSurface();
        }

        double U1 = HS.FirstUParameter(), U2 = HS.LastUParameter();
        double V1 = HS.FirstVParameter(), V2 = HS.LastVParameter();
        if (res is not null)
        {
            // Protection against Construction Errors
            double u1, u2, v1, v2;
            res.Bounds(out u1, out u2, out v1, out v2);
            if (!res.IsUPeriodic)
            {
                if (U1 < u1)
                {
                    U1 = u1;
                }
                if (U2 > u2)
                {
                    U2 = u2;
                }
            }
            if (!res.IsVPeriodic)
            {
                if (V1 < v1)
                {
                    V1 = v1;
                }
                if (V2 > v2)
                {
                    V2 = v2;
                }
            }
            res = new TrimmedSurface(res, U1, U2, V1, V2);
        }
        //  occ::handle<GeomAdaptor_Surface> temp = new GeomAdaptor_Surface(res,U1,U2,V1,V2);
        //  dom = new Adaptor3d_TopolTool(temp);
        return res;
    }

    //=======================================================================
    // function : CurveCleaner
    // purpose  : Makes a BSpline as much continued as possible
    //           at a given tolerance
    //=======================================================================

    private static void CurveCleaner(BSplineCurve3D BS, double Tol, int MultMin)
    {
        double tol = Tol;
        int Mult, ii;
        int NbK = BS.KnotCount;

        for (Mult = BS.Degree(); Mult > MultMin; Mult--)
        {
            tol *= 0.5; // Progressive reduction
            for (ii = NbK; ii > 1; ii--)
            {
                if (BS.Multiplicity(ii) == Mult)
                {
                    BS.RemoveKnot(ii, Mult - 1, tol);
                }
            }
        }
    }
}
