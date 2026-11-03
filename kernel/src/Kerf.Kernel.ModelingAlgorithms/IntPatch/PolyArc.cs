// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_PolyArc.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_PolyArc.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Intersection.Patches;

/// <summary>
/// The polygon of an arc of restriction (a 2D curve) between two parameters, resampled on the part that lies near the
/// other polygon's box, with its deflection; it can be shifted by an offset (a period).
/// </summary>
public class PolyArc : Polygo
{
    private readonly NCollection_Array1<Point2D> _brise;
    private readonly NCollection_Array1<double> _param;
    private double _offsetx;
    private double _offsety;
    private readonly bool _ferme;

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
    /// The polygon of <paramref name="line"/> between <paramref name="aPdeb"/> and <paramref name="aPfin"/> (finite)
    /// with <paramref name="nbSample"/> points. Throws Standard_ConstructionError on infinite bounds or fewer than two
    /// samples.
    /// </summary>
    public PolyArc(Curve2DAdaptorBase line, int nbSample, double aPdeb, double aPfin, in Box2d boxOtherPolygon)
    {
        _brise = new NCollection_Array1<Point2D>(1, Max(1, nbSample));
        _param = new NCollection_Array1<double>(1, Max(1, nbSample));
        _offsetx = 0.0;
        _offsety = 0.0;

        double pdeb = aPdeb;
        double pfin = aPfin;
        Point2D p2d;

        if (pdeb == RealFirst() || pfin == RealLast() || nbSample <= 1)
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
        int indexInf = nbSample + 1;
        int indexSup = 0;

        double bx0, by0, bxmin, bxmax, bymin, bymax, r, r2;

        boxOtherPolygon.Get(out bxmin, out bymin, out bxmax, out bymax);
        r = (bxmax - bxmin) + (bymax - bymin);
        bx0 = (bxmax + bxmin) * 0.5;
        by0 = (bymax + bymin) * 0.5;

        double pas;
        double x, y, xs, ys, xm, ym, xXs, yYs;

        r *= 0.8;
        r2 = r * r * 49.0;
        int nbloop = 0;

        do
        {
            nbloop++;
            pas = (pfin - pdeb) / (nbSample - 1);
            _param[1] = pdeb;
            line.D0(pdeb, out p2d);
            xs = p2d.X();
            ys = p2d.Y();
            _brise[1] = _brise[1].WithCoord(xs, ys);

            _box.SetVoid();

            _box.Add(_brise[1]);
            _error = 0.0;

            for (int i = 2; i <= nbSample; i++)
            {
                _param[i] = pdeb + (i - 1) * pas;
                line.D0(_param[i], out p2d);
                x = p2d.X();
                y = p2d.Y();
                _brise[i] = _brise[i].WithCoord(x, y);
                xXs = 0.5 * (xs + x);
                yYs = 0.5 * (ys + y);
                //------------------------------------------------------------
                //-- On recherche le debut et la fin de la zone significative
                //------------------------------------------------------------
                // MSV: (see cda 002 H2) if segment is too large (>>r) we have
                //      a risk to jump through BoxOtherPolygon, therefore we should
                //      check this condition if the first one is failure.
                bool isMidPtInBox = (Math.Abs(bx0 - xXs) + Math.Abs(by0 - yYs)) < r;
                bool isSegOut = true;
                if (!isMidPtInBox)
                {
                    double d = (x - xs) * (x - xs) + (y - ys) * (y - ys);
                    if (d > r2)
                    {
                        MinMax(xs, x, out double xmin, out double xmax);
                        MinMax(ys, y, out double ymin, out double ymax);
                        isSegOut = (xmax < bxmin || xmin > bxmax || ymax < bymin || ymin > bymax);
                    }
                }
                if (isMidPtInBox || !isSegOut)
                {
                    // MSV: take the previous and the next segments too, because of
                    //      we check only the middle point (see BUC60946)
                    // if(IndexInf>i) IndexInf=i-1;
                    // if(IndexSup<i) IndexSup=i;
                    if (indexInf > i)
                    {
                        indexInf = Max(i - 2, 1);
                    }
                    if (indexSup < i)
                    {
                        indexSup = Min(i + 1, nbSample);
                    }
                }

                _box.Add(_brise[i]);
                line.D0(_param[i] - pas * 0.5, out p2d);
                xm = p2d.X() - xXs;
                ym = p2d.Y() - yYs;
                xm = Math.Sqrt(xm * xm + ym * ym);
                _error = Max(_error, xm);
                xs = x;
                ys = y;
            }
            if (indexInf > indexSup)
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
                if ((indexSup - indexInf) < (nbSample / 2))
                {
                    //-- std::cout<<" --- On remet ca entre les index "<<IndexInf<<" et "<<IndexSup<<std::endl;
                    nbloop = 10;
                    // if(IndexInf>1) IndexInf--;
                    // if(IndexSup<NbSample) IndexSup++;
                    pdeb = _param[indexInf];
                    pfin = _param[indexSup];
                    // IndexInf = IndexSup+1;
                    indexInf = nbSample + 1;
                    indexSup = 0;
                }
            }
        } while ((indexInf > indexSup) && nbloop <= 10);
        _error *= 1.2;
        if (_error < 0.00000001)
        {
            _error = 0.00000001;
        }
        _box.Enlarge(_error);

        _ferme = (line.Value(aPdeb).Distance(line.Value(aPfin)) <= 1e-7);
    }

    public override bool Closed() => _ferme;

    public override int PointCount => _brise.Length();

    /// <summary>The point of range <paramref name="index"/>, shifted by the offset.</summary>
    public override Point2D Point(int index)
    {
        if (_offsetx == 0.0 && _offsety == 0.0)
        {
            return (_brise[index]);
        }
        Point2D p = _brise[index];
        return (new Point2D(p.X() + _offsetx, p.Y() + _offsety));
    }

    /// <summary>The parameter on the arc of the point of range <paramref name="index"/>.</summary>
    public double Parameter(int index) => _param[index];

    /// <summary>Shifts the polygon (and its box) by (<paramref name="ox"/>, <paramref name="oy"/>).</summary>
    public void SetOffset(double ox, double oy)
    {
        double xmin, ymin, xmax, ymax, g;
        _box.Get(out xmin, out ymin, out xmax, out ymax);
        g = _box.GetGap();

        _box.SetVoid();

        // OCCT: Get returns the bounds with the gap, and the box is reset to the unshifted bounds before the shifted
        // ones are added: it covers both, and grows by the gap at each call (reproduced).
        _box.Update(xmin - _offsetx, ymin - _offsety, xmax - _offsetx, ymax - _offsety);
        _offsetx = ox;
        _offsety = oy;
        _box.Update(xmin + _offsetx, ymin + _offsety, xmax + _offsetx, ymax + _offsety);
        _box.SetGap(g);
    }
}
