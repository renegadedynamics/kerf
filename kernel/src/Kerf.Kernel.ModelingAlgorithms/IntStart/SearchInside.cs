// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntStart/IntStart_SearchInside.gxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntStart/IntStart_SearchInside.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Implicit;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Intersection.Walking;

/// <summary>
/// What the interior search reads from its function (OCCT's TheFunction macro: IntPatch_TheSurfFunction,
/// Contap_SurfFunction), besides its values: the surface it is loaded on, its tolerance, and at a root the point and
/// the tangents of the intersection line.
/// </summary>
internal interface ISurfFunction<ThePSurface>
{
    /// <summary>Loads the parametric surface.</summary>
    void Set(ThePSurface ps);

    /// <summary>The value Tol such that the function is null where |Root()| &lt;= Tol.</summary>
    double Tolerance();

    /// <summary>The value of the last evaluation.</summary>
    double Root();

    /// <summary>True if the surfaces are tangent at the last evaluated point.</summary>
    bool IsTangent();

    /// <summary>The point of the last evaluation.</summary>
    Point3D Point();

    Vector3D Direction3d();

    Direction2D Direction2d();
}

/// <summary>
/// The points of a parametric surface, inside its domain, where a function F(u, v) vanishes and the intersection is
/// not tangent: the passing points and the starting points of closed walking lines of the implicit/parametric
/// intersection. Newton from the domain's sample points (and twelve points near its corners), keeping the new
/// solutions classified inside the domain.
/// </summary>
/// <remarks>
/// OCCT's generic file IntStart_SearchInside.gxx, ported once as a generic class over the surface, its tool and the
/// function (decisions 337 and 607). TheTopolTool is Adaptor3d_TopolTool in every instantiation
/// (IntPatch_TheSearchInside, Contap_TheSearchInside) and is named directly; TheSITool is not used by the body and is
/// not a parameter. OCCT's instantiations are sealed subclasses.
/// </remarks>
internal class SearchInside<ThePSurface, ThePSurfaceTool, TheFunction>
    where ThePSurfaceTool : IZerCSParPSurfaceTool<ThePSurface>
    where TheFunction : FunctionSetWithDerivatives, ISurfFunction<ThePSurface>
{
    private bool _done;
    private readonly NCollection_Sequence<InteriorPoint> _list = new();

    public SearchInside()
    {
        _done = false;
    }

    public SearchInside(TheFunction func, ThePSurface ps, TopolTool t, double epsilon)
    {
        Perform(func, ps, t, epsilon);
    }

    /// <summary>
    /// Searches the interior points from the sample points of the domain <paramref name="t"/>; two solutions within
    /// <paramref name="epsilon"/> in 3D (and the parametric resolution) are one.
    /// </summary>
    public void Perform(TheFunction func, ThePSurface ps, TopolTool t, double epsilon)
    {
        _done = false;
        _list.Clear();
        math_Vector binf = new(1, 2), bsup = new(1, 2), uVap = new(1, 2), toler = new(1, 2);
        Point3D psol;
        bool testpnt;
        int i, j, nbpt;
        Containment situ;
        double umin, umax, vmin, vmax;
        binf[1] = umin = ThePSurfaceTool.FirstUParameter(ps);
        binf[2] = vmin = ThePSurfaceTool.FirstVParameter(ps);
        bsup[1] = umax = ThePSurfaceTool.LastUParameter(ps);
        bsup[2] = vmax = ThePSurfaceTool.LastVParameter(ps);

        int nbsampleU = t.NbSamplesU();
        int nbsampleV = t.NbSamplesV();
        int nbsample = t.NbSamples();

        double du = bsup[1] - binf[1];
        double dv = bsup[2] - binf[2];
        du /= (double)nbsampleU * 0.5;
        dv /= (double)nbsampleV * 0.5;

        double toler1 = toler[1] = ThePSurfaceTool.UResolution(ps, global::Kerf.Engine.Foundation.Precision.Confusion());
        double toler2 = toler[2] = ThePSurfaceTool.VResolution(ps, global::Kerf.Engine.Foundation.Precision.Confusion());
        double maxtoler1toler2 = toler1;
        if (toler2 > maxtoler1toler2)
        {
            maxtoler1toler2 = toler2;
        }

        //-- lbr le 15 mai 97
        //-- on interdit aux points d'etre trop prets des restrictions
        maxtoler1toler2 *= 1000;
        if (maxtoler1toler2 > du * 0.001)
        {
            maxtoler1toler2 = du * 0.001;
        }
        if (maxtoler1toler2 > dv * 0.001)
        {
            maxtoler1toler2 = dv * 0.001;
        }

        func.Set(ps);
        double tol = func.Tolerance();

        FunctionSetRoot rsnld = new(func, toler);

        //-- lbr le 15 mai 97
        umin += du * 0.01;
        vmin += dv * 0.01;
        umax -= du * 0.01;
        vmax -= dv * 0.01;

        //-- lbr le 30 octobre 97 :
        //-- Si une surface vient tangenter 2 edges proche d un coin
        //-- il faut faire attention qu un point de depart soit trouve au
        //-- voisinage du coin. Car ds le cas contraire, le cheminement ne
        //-- pourra pas passer au travers des frontieres :
        //--
        //-- typiquement I est un cylindre (conge)
        //--
        //--                 PPPPPPPPPPPPPPPPPPPP*PPPPPPPPPPPPPPPP
        //--                 P               I     I
        //--                 P           I           I
        //--                 P        I
        //--                 P      #  il faut trouver un point ici
        //--                 P    I
        //--                 P  I
        //--                 PI
        //--                 *                    I
        //--                 PI                 I
        //--                 P  I I I I I I I I
        //--

        math_Vector valf = new(1, 1);
        for (i = 1; i <= nbsample + 12; i++)
        {
            Point2D s2d = new();
            bool nepastester = false;
            if (i <= nbsample)
            {
                t.SamplePoint(i, out s2d, out _);
                uVap[1] = s2d.X;
                uVap[2] = s2d.Y;

                double u1, v1, u2, v2;
                u1 = binf[1] = Max(umin, uVap[1] - du);
                v1 = binf[2] = Max(vmin, uVap[2] - dv);
                u2 = bsup[1] = Min(umax, uVap[1] + du);
                v2 = bsup[2] = Min(vmax, uVap[2] + dv);

                //-- gp_Pnt Pmilieu = ThePSurfaceTool::Value(PS,0.5*(u1+u2),0.5*(v1+v2));
                Point3D pextrm1 = ThePSurfaceTool.Value(ps, u1, v1);
                Point3D pextrm2 = ThePSurfaceTool.Value(ps, u2, v2);
                func.Value(uVap, valf);
                double rvalf = valf[1];
                double distPP = pextrm1.SquareDistanceTo(pextrm2);
                if (rvalf * rvalf > 3.0 * distPP)
                {
                    nepastester = true;
                }
            }
            else
            {
                if (i == nbsample + 1)
                {
                    s2d = s2d.WithCoord(umin + du * 0.02, vmin + dv * 0.02);
                }
                else if (i == nbsample + 2)
                {
                    s2d = s2d.WithCoord(umax - du * 0.02, vmin + dv * 0.02);
                }
                else if (i == nbsample + 3)
                {
                    s2d = s2d.WithCoord(umin + du * 0.02, vmax - dv * 0.02);
                }
                else if (i == nbsample + 4)
                {
                    s2d = s2d.WithCoord(umax - du * 0.02, vmax - dv * 0.02);
                }

                else if (i == nbsample + 5)
                {
                    s2d = s2d.WithCoord(umin + du * 0.02, vmin + dv * 0.02);
                }
                else if (i == nbsample + 6)
                {
                    s2d = s2d.WithCoord(umax - du * 0.02, vmin + dv * 0.02);
                }
                else if (i == nbsample + 7)
                {
                    s2d = s2d.WithCoord(umin + du * 0.02, vmax - dv * 0.02);
                }
                else if (i == nbsample + 8)
                {
                    s2d = s2d.WithCoord(umax - du * 0.02, vmax - dv * 0.02);
                }

                else if (i == nbsample + 9)
                {
                    s2d = s2d.WithCoord(umin + du * 0.005, vmin + dv * 0.005);
                }
                else if (i == nbsample + 10)
                {
                    s2d = s2d.WithCoord(umax - du * 0.005, vmin + dv * 0.005);
                }
                else if (i == nbsample + 11)
                {
                    s2d = s2d.WithCoord(umin + du * 0.005, vmax - dv * 0.005);
                }
                else
                {
                    s2d = s2d.WithCoord(umax - du * 0.005, vmax - dv * 0.005);
                }

                uVap[1] = s2d.X;
                uVap[2] = s2d.Y;

                binf[1] = Max(umin, uVap[1] - du);
                binf[2] = Max(vmin, uVap[2] - dv);
                bsup[1] = Min(umax, uVap[1] + du);
                bsup[2] = Min(vmax, uVap[2] + dv);
            }

            if (nepastester == false)
            {
                rsnld.Perform(func, uVap, binf, bsup);
                if (rsnld.IsDone)
                {
                    if (Math.Abs(func.Root()) <= tol)
                    {
                        if (!func.IsTangent())
                        {
                            psol = func.Point();
                            rsnld.Root(uVap);
                            // On regarde si le point trouve est bien un nouveau point.
                            j = 1;
                            nbpt = _list.Length();
                            testpnt = (j <= nbpt);

                            while (testpnt)
                            {
                                ref InteriorPoint iPj = ref _list[j];
                                Point3D pj = iPj.Value();
                                if ((Math.Abs(pj.X - psol.X) <= epsilon) && (Math.Abs(pj.Y - psol.Y) <= epsilon)
                                    && (Math.Abs(pj.Z - psol.Z) <= epsilon)
                                    && (Math.Abs(uVap[1] - iPj.UParameter()) <= toler1)
                                    && (Math.Abs(uVap[2] - iPj.VParameter()) <= toler2))
                                {
                                    testpnt = false;
                                }
                                else
                                {
                                    j = j + 1;
                                    testpnt = (j <= nbpt);
                                }
                            }
                            if (j > nbpt)
                            {
                                //	    situ = TheSITool::Classify(PS,UVap(1),UVap(2));
                                situ = t.Classify(new Point2D(uVap[1], uVap[2]), maxtoler1toler2,
                                                  false); //-- ,false pour ne pas recadrer on Periodic
                                if (situ == In)
                                {
                                    _list.Append(new InteriorPoint(psol, uVap[1], uVap[2], func.Direction3d(),
                                                                          new Vector2D(func.Direction2d())));
                                }
                            }
                        }
                    }
                }
            }
        }
        _done = true;
    }

    /// <summary>
    /// Searches one interior point from (<paramref name="uStart"/>, <paramref name="vStart"/>), if that point is in
    /// the surface's bounds.
    /// </summary>
    public void Perform(TheFunction func, ThePSurface ps, double uStart, double vStart)
    {
        _done = false;
        _list.Clear();
        math_Vector binf = new(1, 2), bsup = new(1, 2), toler = new(1, 2);

        binf[1] = ThePSurfaceTool.FirstUParameter(ps);
        binf[2] = ThePSurfaceTool.FirstVParameter(ps);
        bsup[1] = ThePSurfaceTool.LastUParameter(ps);
        bsup[2] = ThePSurfaceTool.LastVParameter(ps);

        toler[1] = ThePSurfaceTool.UResolution(ps, global::Kerf.Engine.Foundation.Precision.Confusion());
        toler[2] = ThePSurfaceTool.VResolution(ps, global::Kerf.Engine.Foundation.Precision.Confusion());

        if (uStart - binf[1] > -toler[1] && uStart - bsup[1] < toler[1] && vStart - binf[2] > -toler[2]
            && vStart - bsup[2] < toler[2])
        {
            func.Set(ps);
            math_Vector uVap = new(1, 2);
            uVap[1] = uStart;
            uVap[2] = vStart;

            FunctionSetRoot rsnld = new(func, toler);
            rsnld.Perform(func, uVap, binf, bsup);
            if (rsnld.IsDone)
            {
                double tol = func.Tolerance();
                double valf = func.Root();
                if (Math.Abs(valf) <= tol && !func.IsTangent())
                {
                    Point3D psol = func.Point();
                    rsnld.Root(uVap);
                    InteriorPoint intp = new(psol, uVap[1], uVap[2], func.Direction3d(),
                                                     new Vector2D(func.Direction2d()));
                    _list.Append(intp);
                }
            }
        }

        _done = true;
    }

    public bool IsDone => _done;

    /// <summary>The number of points; throws StdFail_NotDone if not done.</summary>
    public int PointCount
    {
        get
        {
            if (!_done)
            {
                throw new NotDoneException();
            }
            return _list.Length();
        }
    }

    /// <summary>The point of range <paramref name="index"/>; throws StdFail_NotDone if not done.</summary>
    public InteriorPoint Value(int index)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        return _list.Value(index);
    }
}
