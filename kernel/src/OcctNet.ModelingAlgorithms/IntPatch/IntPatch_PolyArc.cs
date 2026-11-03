// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_PolyArc.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_PolyArc.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The polygon of an arc of restriction (a 2D curve) between two parameters, resampled on the part that lies near the
/// other polygon's box, with its deflection; it can be shifted by an offset (a period).
/// </summary>
public class IntPatch_PolyArc : IntPatch_Polygo
{
    private readonly NCollection_Array1<Point2D> brise;
    private readonly NCollection_Array1<double> param;
    private double offsetx;
    private double offsety;
    private readonly bool ferme;

    private static void MinMax(double a1, double a2, out double amin, out double amax)
    {
        if (a1 < a2)
        {
            amin = a1;
            amax = a2;
        }
        else
        {
            amin = a2;
            amax = a1;
        }
    }

    /// <summary>
    /// The polygon of <paramref name="Line"/> between <paramref name="aPdeb"/> and <paramref name="aPfin"/> (finite)
    /// with <paramref name="NbSample"/> points. Throws Standard_ConstructionError on infinite bounds or fewer than two
    /// samples.
    /// </summary>
    public IntPatch_PolyArc(Adaptor2d_Curve2d Line, int NbSample, double aPdeb, double aPfin, in Box2d BoxOtherPolygon)
    {
        brise = new NCollection_Array1<Point2D>(1, Max(1, NbSample));
        param = new NCollection_Array1<double>(1, Max(1, NbSample));
        offsetx = 0.0;
        offsety = 0.0;

        double Pdeb = aPdeb;
        double Pfin = aPfin;
        Point2D p2d;

        if (Pdeb == RealFirst() || Pfin == RealLast() || NbSample <= 1)
        {
            throw new ConstructionErrorException();
        }
        //----------------------------------------------------------------------
        //-- On veut eviter les cas ou  le present polygone est beaucoup plus
        //-- grand que l objet en second.
        //--
        //-- Par exemple lorsque l objet en second est une ligne de cheminement
        //-- qui contient de nombreux segments (>>100), une fleche nulle
        //-- et ce polygone quelques segments et une fleche qui contient
        //-- toute la ligne de cheminement.
        //--
        //-- Dans ce cas (tout un polygone compris dans la zone d influence)
        //-- les calculs deviennent tres longs (N2)
        //----------------------------------------------------------------------
        int IndexInf = NbSample + 1;
        int IndexSup = 0;

        double bx0, by0, bxmin, bxmax, bymin, bymax, r, r2;

        BoxOtherPolygon.Get(out bxmin, out bymin, out bxmax, out bymax);
        r = (bxmax - bxmin) + (bymax - bymin);
        bx0 = (bxmax + bxmin) * 0.5;
        by0 = (bymax + bymin) * 0.5;

        double Pas;
        double X, Y, Xs, Ys, Xm, Ym, XXs, YYs;

        r *= 0.8;
        r2 = r * r * 49.0;
        int nbloop = 0;

        do
        {
            nbloop++;
            Pas = (Pfin - Pdeb) / (NbSample - 1);
            param[1] = Pdeb;
            Line.D0(Pdeb, out p2d);
            Xs = p2d.X();
            Ys = p2d.Y();
            brise[1].SetCoord(Xs, Ys);

            myBox.SetVoid();

            myBox.Add(brise[1]);
            myError = 0.0;

            for (int i = 2; i <= NbSample; i++)
            {
                param[i] = Pdeb + (i - 1) * Pas;
                Line.D0(param[i], out p2d);
                X = p2d.X();
                Y = p2d.Y();
                brise[i].SetCoord(X, Y);
                XXs = 0.5 * (Xs + X);
                YYs = 0.5 * (Ys + Y);
                //------------------------------------------------------------
                //-- On recherche le debut et la fin de la zone significative
                //------------------------------------------------------------
                // MSV: (see cda 002 H2) if segment is too large (>>r) we have
                //      a risk to jump through BoxOtherPolygon, therefore we should
                //      check this condition if the first one is failure.
                bool isMidPtInBox = (Math.Abs(bx0 - XXs) + Math.Abs(by0 - YYs)) < r;
                bool isSegOut = true;
                if (!isMidPtInBox)
                {
                    double d = (X - Xs) * (X - Xs) + (Y - Ys) * (Y - Ys);
                    if (d > r2)
                    {
                        MinMax(Xs, X, out double xmin, out double xmax);
                        MinMax(Ys, Y, out double ymin, out double ymax);
                        isSegOut = (xmax < bxmin || xmin > bxmax || ymax < bymin || ymin > bymax);
                    }
                }
                if (isMidPtInBox || !isSegOut)
                {
                    // MSV: take the previous and the next segments too, because of
                    //      we check only the middle point (see BUC60946)
                    // if(IndexInf>i) IndexInf=i-1;
                    // if(IndexSup<i) IndexSup=i;
                    if (IndexInf > i)
                    {
                        IndexInf = Max(i - 2, 1);
                    }
                    if (IndexSup < i)
                    {
                        IndexSup = Min(i + 1, NbSample);
                    }
                }

                myBox.Add(brise[i]);
                Line.D0(param[i] - Pas * 0.5, out p2d);
                Xm = p2d.X() - XXs;
                Ym = p2d.Y() - YYs;
                Xm = Math.Sqrt(Xm * Xm + Ym * Ym);
                myError = Max(myError, Xm);
                Xs = X;
                Ys = Y;
            }
            if (IndexInf > IndexSup)
            {
                r += r;
                r2 = r * r * 49.0;
                //-- std::cout<<" Le rayon : "<<r<<" est insuffisant "<<std::endl;
            }
            else
            {
                //----------------------------------------------
                //-- Si le nombre de points significatifs est
                //-- insuffisant, on reechantillonne une fois
                //-- encore
                //----------------------------------------------
                if ((IndexSup - IndexInf) < (NbSample / 2))
                {
                    //-- std::cout<<" --- On remet ca entre les index "<<IndexInf<<" et "<<IndexSup<<std::endl;
                    nbloop = 10;
                    // if(IndexInf>1) IndexInf--;
                    // if(IndexSup<NbSample) IndexSup++;
                    Pdeb = param[IndexInf];
                    Pfin = param[IndexSup];
                    // IndexInf = IndexSup+1;
                    IndexInf = NbSample + 1;
                    IndexSup = 0;
                }
            }
        } while ((IndexInf > IndexSup) && nbloop <= 10);
        myError *= 1.2;
        if (myError < 0.00000001)
        {
            myError = 0.00000001;
        }
        myBox.Enlarge(myError);

        ferme = (Line.Value(aPdeb).Distance(Line.Value(aPfin)) <= 1e-7);
    }

    public override bool Closed() => ferme;

    public override int NbPoints() => brise.Length();

    /// <summary>The point of range <paramref name="Index"/>, shifted by the offset.</summary>
    public override Point2D Point(int Index)
    {
        if (offsetx == 0.0 && offsety == 0.0)
        {
            return (brise[Index]);
        }
        Point2D P = brise[Index];
        return (new Point2D(P.X() + offsetx, P.Y() + offsety));
    }

    /// <summary>The parameter on the arc of the point of range <paramref name="Index"/>.</summary>
    public double Parameter(int Index) => param[Index];

    /// <summary>Shifts the polygon (and its box) by (<paramref name="ox"/>, <paramref name="oy"/>).</summary>
    public void SetOffset(double ox, double oy)
    {
        double xmin, ymin, xmax, ymax, g;
        myBox.Get(out xmin, out ymin, out xmax, out ymax);
        g = myBox.GetGap();

        myBox.SetVoid();

        // OCCT: Get returns the bounds with the gap, and the box is reset to the unshifted bounds before the shifted
        // ones are added: it covers both, and grows by the gap at each call (reproduced).
        myBox.Update(xmin - offsetx, ymin - offsety, xmax - offsetx, ymax - offsety);
        offsetx = ox;
        offsety = oy;
        myBox.Update(xmin + offsetx, ymin + offsety, xmax + offsetx, ymax + offsety);
        myBox.SetGap(g);
    }
}
