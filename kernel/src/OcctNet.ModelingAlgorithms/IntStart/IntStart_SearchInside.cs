// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntStart/IntStart_SearchInside.gxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntStart/IntStart_SearchInside.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// What the interior search reads from its function (OCCT's TheFunction macro: IntPatch_TheSurfFunction,
/// Contap_SurfFunction), besides its values: the surface it is loaded on, its tolerance, and at a root the point and
/// the tangents of the intersection line.
/// </summary>
public interface IIntStart_SurfFunction<ThePSurface>
{
    /// <summary>Loads the parametric surface.</summary>
    void Set(ThePSurface PS);

    /// <summary>The value Tol such that the function is null where |Root()| &lt;= Tol.</summary>
    double Tolerance();

    /// <summary>The value of the last evaluation.</summary>
    double Root();

    /// <summary>True if the surfaces are tangent at the last evaluated point.</summary>
    bool IsTangent();

    /// <summary>The point of the last evaluation.</summary>
    gp_Pnt Point();

    gp_Vec Direction3d();

    gp_Dir2d Direction2d();
}

/// <summary>
/// The points of a parametric surface, inside its domain, where a function F(u, v) vanishes and the intersection is
/// not tangent: the passing points and the starting points of closed walking lines of the implicit/parametric
/// intersection. Newton from the domain's sample points (and twelve points near its corners), keeping the new
/// solutions classified inside the domain.
/// </summary>
/// <remarks>
/// OCCT's generic file IntStart_SearchInside.gxx, ported once as a generic class over the surface, its tool and the
/// function (decisions 337 and 5101). TheTopolTool is Adaptor3d_TopolTool in every instantiation
/// (IntPatch_TheSearchInside, Contap_TheSearchInside) and is named directly; TheSITool is not used by the body and is
/// not a parameter. OCCT's instantiations are sealed subclasses.
/// </remarks>
public class IntStart_SearchInside<ThePSurface, ThePSurfaceTool, TheFunction>
    where ThePSurfaceTool : IIntImp_ZerCSParPSurfaceTool<ThePSurface>
    where TheFunction : math_FunctionSetWithDerivatives, IIntStart_SurfFunction<ThePSurface>
{
    private bool done;
    private readonly NCollection_Sequence<IntSurf_InteriorPoint> list = new();

    public IntStart_SearchInside()
    {
        done = false;
    }

    public IntStart_SearchInside(TheFunction Func, ThePSurface PS, Adaptor3d_TopolTool T, double Epsilon)
    {
        Perform(Func, PS, T, Epsilon);
    }

    /// <summary>
    /// Searches the interior points from the sample points of the domain <paramref name="T"/>; two solutions within
    /// <paramref name="Epsilon"/> in 3D (and the parametric resolution) are one.
    /// </summary>
    public void Perform(TheFunction Func, ThePSurface PS, Adaptor3d_TopolTool T, double Epsilon)
    {
        done = false;
        list.Clear();
        math_Vector Binf = new(1, 2), Bsup = new(1, 2), UVap = new(1, 2), toler = new(1, 2);
        gp_Pnt psol;
        bool testpnt;
        int i, j, nbpt;
        TopAbs_State situ;
        double umin, umax, vmin, vmax;
        Binf[1] = umin = ThePSurfaceTool.FirstUParameter(PS);
        Binf[2] = vmin = ThePSurfaceTool.FirstVParameter(PS);
        Bsup[1] = umax = ThePSurfaceTool.LastUParameter(PS);
        Bsup[2] = vmax = ThePSurfaceTool.LastVParameter(PS);

        int NbsampleU = T.NbSamplesU();
        int NbsampleV = T.NbSamplesV();
        int Nbsample = T.NbSamples();

        double du = Bsup[1] - Binf[1];
        double dv = Bsup[2] - Binf[2];
        du /= (double)NbsampleU * 0.5;
        dv /= (double)NbsampleV * 0.5;

        double toler1 = toler[1] = ThePSurfaceTool.UResolution(PS, Precision.Confusion());
        double toler2 = toler[2] = ThePSurfaceTool.VResolution(PS, Precision.Confusion());
        double Maxtoler1toler2 = toler1;
        if (toler2 > Maxtoler1toler2)
        {
            Maxtoler1toler2 = toler2;
        }

        //-- lbr le 15 mai 97
        //-- on interdit aux points d'etre trop prets des restrictions
        Maxtoler1toler2 *= 1000;
        if (Maxtoler1toler2 > du * 0.001)
        {
            Maxtoler1toler2 = du * 0.001;
        }
        if (Maxtoler1toler2 > dv * 0.001)
        {
            Maxtoler1toler2 = dv * 0.001;
        }

        Func.Set(PS);
        double Tol = Func.Tolerance();

        math_FunctionSetRoot Rsnld = new(Func, toler);

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

        math_Vector Valf = new(1, 1);
        for (i = 1; i <= Nbsample + 12; i++)
        {
            gp_Pnt2d s2d = new();
            bool nepastester = false;
            if (i <= Nbsample)
            {
                T.SamplePoint(i, out s2d, out _);
                UVap[1] = s2d.X();
                UVap[2] = s2d.Y();

                double u1, v1, u2, v2;
                u1 = Binf[1] = Max(umin, UVap[1] - du);
                v1 = Binf[2] = Max(vmin, UVap[2] - dv);
                u2 = Bsup[1] = Min(umax, UVap[1] + du);
                v2 = Bsup[2] = Min(vmax, UVap[2] + dv);

                //-- gp_Pnt Pmilieu = ThePSurfaceTool::Value(PS,0.5*(u1+u2),0.5*(v1+v2));
                gp_Pnt Pextrm1 = ThePSurfaceTool.Value(PS, u1, v1);
                gp_Pnt Pextrm2 = ThePSurfaceTool.Value(PS, u2, v2);
                Func.Value(UVap, Valf);
                double rvalf = Valf[1];
                double DistPP = Pextrm1.SquareDistance(Pextrm2);
                if (rvalf * rvalf > 3.0 * DistPP)
                {
                    nepastester = true;
                }
            }
            else
            {
                if (i == Nbsample + 1)
                {
                    s2d.SetCoord(umin + du * 0.02, vmin + dv * 0.02);
                }
                else if (i == Nbsample + 2)
                {
                    s2d.SetCoord(umax - du * 0.02, vmin + dv * 0.02);
                }
                else if (i == Nbsample + 3)
                {
                    s2d.SetCoord(umin + du * 0.02, vmax - dv * 0.02);
                }
                else if (i == Nbsample + 4)
                {
                    s2d.SetCoord(umax - du * 0.02, vmax - dv * 0.02);
                }

                else if (i == Nbsample + 5)
                {
                    s2d.SetCoord(umin + du * 0.02, vmin + dv * 0.02);
                }
                else if (i == Nbsample + 6)
                {
                    s2d.SetCoord(umax - du * 0.02, vmin + dv * 0.02);
                }
                else if (i == Nbsample + 7)
                {
                    s2d.SetCoord(umin + du * 0.02, vmax - dv * 0.02);
                }
                else if (i == Nbsample + 8)
                {
                    s2d.SetCoord(umax - du * 0.02, vmax - dv * 0.02);
                }

                else if (i == Nbsample + 9)
                {
                    s2d.SetCoord(umin + du * 0.005, vmin + dv * 0.005);
                }
                else if (i == Nbsample + 10)
                {
                    s2d.SetCoord(umax - du * 0.005, vmin + dv * 0.005);
                }
                else if (i == Nbsample + 11)
                {
                    s2d.SetCoord(umin + du * 0.005, vmax - dv * 0.005);
                }
                else
                {
                    s2d.SetCoord(umax - du * 0.005, vmax - dv * 0.005);
                }

                UVap[1] = s2d.X();
                UVap[2] = s2d.Y();

                Binf[1] = Max(umin, UVap[1] - du);
                Binf[2] = Max(vmin, UVap[2] - dv);
                Bsup[1] = Min(umax, UVap[1] + du);
                Bsup[2] = Min(vmax, UVap[2] + dv);
            }

            if (nepastester == false)
            {
                Rsnld.Perform(Func, UVap, Binf, Bsup);
                if (Rsnld.IsDone())
                {
                    if (Math.Abs(Func.Root()) <= Tol)
                    {
                        if (!Func.IsTangent())
                        {
                            psol = Func.Point();
                            Rsnld.Root(UVap);
                            // On regarde si le point trouve est bien un nouveau point.
                            j = 1;
                            nbpt = list.Length();
                            testpnt = (j <= nbpt);

                            while (testpnt)
                            {
                                ref IntSurf_InteriorPoint IPj = ref list[j];
                                gp_Pnt Pj = IPj.Value();
                                if ((Math.Abs(Pj.X() - psol.X()) <= Epsilon) && (Math.Abs(Pj.Y() - psol.Y()) <= Epsilon)
                                    && (Math.Abs(Pj.Z() - psol.Z()) <= Epsilon)
                                    && (Math.Abs(UVap[1] - IPj.UParameter()) <= toler1)
                                    && (Math.Abs(UVap[2] - IPj.VParameter()) <= toler2))
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
                                situ = T.Classify(new gp_Pnt2d(UVap[1], UVap[2]), Maxtoler1toler2,
                                                  false); //-- ,false pour ne pas recadrer on Periodic
                                if (situ == TopAbs_IN)
                                {
                                    list.Append(new IntSurf_InteriorPoint(psol, UVap[1], UVap[2], Func.Direction3d(),
                                                                          new gp_Vec2d(Func.Direction2d())));
                                }
                            }
                        }
                    }
                }
            }
        }
        done = true;
    }

    /// <summary>
    /// Searches one interior point from (<paramref name="UStart"/>, <paramref name="VStart"/>), if that point is in
    /// the surface's bounds.
    /// </summary>
    public void Perform(TheFunction Func, ThePSurface PS, double UStart, double VStart)
    {
        done = false;
        list.Clear();
        math_Vector Binf = new(1, 2), Bsup = new(1, 2), toler = new(1, 2);

        Binf[1] = ThePSurfaceTool.FirstUParameter(PS);
        Binf[2] = ThePSurfaceTool.FirstVParameter(PS);
        Bsup[1] = ThePSurfaceTool.LastUParameter(PS);
        Bsup[2] = ThePSurfaceTool.LastVParameter(PS);

        toler[1] = ThePSurfaceTool.UResolution(PS, Precision.Confusion());
        toler[2] = ThePSurfaceTool.VResolution(PS, Precision.Confusion());

        if (UStart - Binf[1] > -toler[1] && UStart - Bsup[1] < toler[1] && VStart - Binf[2] > -toler[2]
            && VStart - Bsup[2] < toler[2])
        {
            Func.Set(PS);
            math_Vector UVap = new(1, 2);
            UVap[1] = UStart;
            UVap[2] = VStart;

            math_FunctionSetRoot Rsnld = new(Func, toler);
            Rsnld.Perform(Func, UVap, Binf, Bsup);
            if (Rsnld.IsDone())
            {
                double tol = Func.Tolerance();
                double valf = Func.Root();
                if (Math.Abs(valf) <= tol && !Func.IsTangent())
                {
                    gp_Pnt psol = Func.Point();
                    Rsnld.Root(UVap);
                    IntSurf_InteriorPoint intp = new(psol, UVap[1], UVap[2], Func.Direction3d(),
                                                     new gp_Vec2d(Func.Direction2d()));
                    list.Append(intp);
                }
            }
        }

        done = true;
    }

    public bool IsDone() => done;

    /// <summary>The number of points; throws StdFail_NotDone if not done.</summary>
    public int NbPoints()
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        return list.Length();
    }

    /// <summary>The point of range <paramref name="Index"/>; throws StdFail_NotDone if not done.</summary>
    public IntSurf_InteriorPoint Value(int Index)
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        return list.Value(Index);
    }
}
