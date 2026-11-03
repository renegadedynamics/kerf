// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder_0.cxx (ChFi3d_StripeEdgeInter to CurveCleaner)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Legacy.Data;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Fillets;

internal static partial class Builder_0
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
    internal static void ChFi3d_StripeEdgeInter(Stripe stripe1, Stripe stripe2,
                                                DsDataStructure dStr, double tol2d)
    {
        // Do not check the stripeshaving common corner points
        for (int iSur1 = 1; iSur1 <= 2; iSur1++)
        {
            for (int iSur2 = 1; iSur2 <= 2; iSur2++)
            {
                if (stripe1.IndexPoint(false, iSur1) == stripe2.IndexPoint(false, iSur2)
                    || stripe1.IndexPoint(false, iSur1) == stripe2.IndexPoint(true, iSur2)
                    || stripe1.IndexPoint(true, iSur1) == stripe2.IndexPoint(false, iSur2)
                    || stripe1.IndexPoint(true, iSur1) == stripe2.IndexPoint(true, iSur2))
                {
                    return;
                }
            }
        }

        NCollection_HSequence<SurfData> surDat1 = stripe1.SetOfSurfData()!;
        NCollection_HSequence<SurfData> surDat2 = stripe2.SetOfSurfData()!;

        GInter intersector = new();
        int iPart1, iPart2;
        int ishape11, ishape12, ishape21, ishape22;
        // Loop on parts of the first stripe
        for (iPart1 = 1; iPart1 <= surDat1.Length(); iPart1++)
        {
            SurfData dat1 = surDat1.Value(iPart1);
            ishape11 = dat1.IndexOfS1();
            ishape12 = dat1.IndexOfS2();
            // Loop on parts of the second stripe
            for (iPart2 = 1; iPart2 <= surDat2.Length(); iPart2++)
            {
                SurfData dat2 = surDat2.Value(iPart2);
                ishape21 = dat2.IndexOfS1();
                ishape22 = dat2.IndexOfS2();

                // Find those FaceInterferences able to intersect
                FaceInterference fi1, fi2;
                if (ishape11 == ishape21)
                {
                    fi1 = dat1.InterferenceOnS1();
                    fi2 = dat2.InterferenceOnS1();
                }
                else if (ishape11 == ishape22)
                {
                    fi1 = dat1.InterferenceOnS1();
                    fi2 = dat2.InterferenceOnS2();
                }
                else if (ishape12 == ishape21)
                {
                    fi1 = dat1.InterferenceOnS2();
                    fi2 = dat2.InterferenceOnS1();
                }
                else if (ishape12 == ishape22)
                {
                    fi1 = dat1.InterferenceOnS2();
                    fi2 = dat2.InterferenceOnS2();
                }
                else
                {
                    // No common faces
                    continue;
                }

                if (fi1.FirstParameter() == fi1.LastParameter() || fi2.FirstParameter() == fi2.LastParameter()
                    || fi1.PCurveOnFace() is null || fi2.PCurveOnFace() is null)
                {
                    // Do not waste time on degenerates
                    continue;
                }
                // Examine for intersections
                CurveAdaptor2D pCurve1 = new(fi1.PCurveOnFace()!, fi1.FirstParameter(),
                                                   fi1.LastParameter());
                CurveAdaptor2D pCurve2 = new(fi2.PCurveOnFace()!, fi2.FirstParameter(),
                                                   fi2.LastParameter());
                intersector.Perform(pCurve1, pCurve2, tol2d, global::Kerf.Engine.Foundation.Precision.PConfusion());
                if (intersector.SegmentCount > 0 || intersector.PointCount > 0)
                {
                    throw new NotDoneException("StripeEdgeInter : fillets have too big radiuses");
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>The index of the stripe's surface data at its end on <paramref name="v1"/> (1 or the last), and the
    /// sense (1 at the start, -1 at the end); 0 when the stripe has no surface data.</summary>
    /// <exception cref="ConstructionErrorException"><paramref name="v1"/> is not an end of the spine.</exception>
    internal static int ChFi3d_IndexOfSurfData(in Vertex v1, Stripe cd, out int sens)
    {
        Spine spine = cd.Spine()!;
        int index = 0;
        sens = 1;
        Vertex vref;
        Edge e = spine.Edges(1);
        if (e.Orientation == Reversed)
        {
            vref = Exploration.LastVertex(e);
        }
        else
        {
            vref = Exploration.FirstVertex(e);
        }
        if (vref.IsSame(v1))
        {
            index = 1;
        }
        else
        {
            Edge e1 = spine.Edges(spine.EdgeCount);
            if (e1.Orientation == Reversed)
            {
                vref = Exploration.FirstVertex(e1);
            }
            else
            {
                vref = Exploration.LastVertex(e1);
            }
            sens = -1;
            if (cd.SetOfSurfData() is null)
            {
                return 0;
            }
            else if (vref.IsSame(v1))
            {
                index = cd.SetOfSurfData()!.Length();
            }
            else
            {
                throw new ConstructionErrorException("ChFi3d_IndexOfSurfData() - wrong construction parameters");
            }
        }
        return index;
    }

    //=================================================================================================

    /// <summary>The spine's edge at its end on <paramref name="v1"/>, and the sense (1 at the start, -1 at the
    /// end).</summary>
    /// <exception cref="ConstructionErrorException"><paramref name="v1"/> is not an end of the spine.</exception>
    internal static Edge ChFi3d_EdgeFromV1(in Vertex v1, Stripe cd, out int sens)
    {
        Spine spine = cd.Spine()!;
        sens = 1;
        Vertex vref;
        Edge e = spine.Edges(1);
        if (e.Orientation == Reversed)
        {
            vref = Exploration.LastVertex(e);
        }
        else
        {
            vref = Exploration.FirstVertex(e);
        }
        if (vref.IsSame(v1))
        {
            return e;
        }
        else
        {
            Edge e1 = spine.Edges(spine.EdgeCount);
            if (e1.Orientation == Reversed)
            {
                vref = Exploration.FirstVertex(e1);
            }
            else
            {
                vref = Exploration.LastVertex(e1);
            }
            sens = -1;
            if (vref.IsSame(v1))
            {
                return e1;
            }
            else
            {
                throw new ConstructionErrorException("ChFi3d_IndexOfSurfData() - wrong construction parameters");
            }
        }
    }

    //=================================================================================================

    /// <summary>A 3D tolerance for the 2D tolerance <paramref name="tol2d"/> on <paramref name="s"/>, by its
    /// resolutions.</summary>
    internal static double ChFi3d_ConvTol2dToTol3d(SurfaceAdaptorBase s, double tol2d)
    {
        double ures = s.UResolution(1.0e-7);
        double vres = s.VResolution(1.0e-7);
        double uresto3d = 1.0e-7 * tol2d / ures;
        double vresto3d = 1.0e-7 * tol2d / vres;
        return Math.Max(uresto3d, vresto3d);
    }

    //=======================================================================
    // function : EvalTolReached
    // purpose  : The function above is too hard because
    //           parametrization of surfaces is not homogeneous.
    //=======================================================================

    /// <summary>1.5 times the largest distance between <paramref name="c"/> and its pcurves on the surfaces, at 45
    /// points (at least the confusion).</summary>
    internal static double ChFi3d_EvalTolReached(SurfaceAdaptorBase s1, Curve2D pc1, SurfaceAdaptorBase s2,
                                                 Curve2D pc2, Curve3D c)
    {
        double distmax = 0.0;

        double f = c.FirstParameter;
        double l = c.LastParameter;
        int nbp = 45;
        double step = 1.0 / (nbp - 1);
        for (int i = 0; i < nbp; i++)
        {
            double t, u, v;
            t = step * i;
            t = (1 - t) * f + t * l;
            pc1.Value(t).Coord(out u, out v);
            Point3D pS1 = s1.Value(u, v);
            pc2.Value(t).Coord(out u, out v);
            Point3D pS2 = s2.Value(u, v);
            Point3D pC = c.Value(t);
            double d = pS1.SquareDistanceTo(pC);
            if (d > distmax)
            {
                distmax = d;
            }
            d = pS2.SquareDistanceTo(pC);
            if (d > distmax)
            {
                distmax = d;
            }
            d = pS1.SquareDistanceTo(pS2);
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
    internal static Surface? Trsfsurf(SurfaceAdaptorBase hs, ref TopolTool? dom)
    {
        // Pour l utilisation des domaines voir avec BUBUCH!!
        Surface? res = null;
        if (hs is FaceSurfaceAdaptor hbs)
        {
            res = hbs.GeomSurfaceTransformed();
        }
        else if (hs is SurfaceAdaptor hgs)
        {
            res = hgs.Surface();
        }
        if (res is TrimmedSurface tr)
        {
            res = tr.BasisSurface;
        }

        double U1 = hs.FirstUParameter(), U2 = hs.LastUParameter();
        double V1 = hs.FirstVParameter(), V2 = hs.LastVParameter();
        if (res is not null)
        {
            // Protection against Construction Errors
            double u1, u2, v1, v2;
            res.ParameterBounds(out u1, out u2, out v1, out v2);
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

    private static void CurveCleaner(BSplineCurve3D bs, double Tol, int multMin)
    {
        double tol = Tol;
        int mult, ii;
        int nbK = bs.KnotCount;

        for (mult = bs.Degree; mult > multMin; mult--)
        {
            tol *= 0.5; // Progressive reduction
            for (ii = nbK; ii > 1; ii--)
            {
                if (bs.Multiplicity(ii) == mult)
                {
                    bs.RemoveKnot(ii, mult - 1, tol);
                }
            }
        }
    }
}
