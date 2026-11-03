// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_Polygon2dGen.gxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_Polygon2dGen.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Bounds;

using Kerf.Engine.Intersection.Polygons;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Intersection.Curves;

/// <summary>
/// A polygon sampling a 2D curve on a domain, with the over-estimated deflection and the box the 2D
/// interferences need; it can be refined where it meets the box of another polygon.
/// </summary>
/// <remarks>
/// OCCT's generic file IntCurve_Polygon2dGen.gxx, ported once as a generic class (decision 324); its
/// instantiation Geom2dInt_ThePolygon2dOfTheIntPCurvePCurveOfGInter is a sealed subclass. OCCT's <c>Dump</c>, which
/// prints only when a static debug flag is set, is left out.
/// </remarks>
public class Polygon2dGen<TheCurve, TheCurveTool> : Polygon2d
    where TheCurveTool : ICurveCurveTool<TheCurve>
{
    private const double MAJORATION_DEFLECTION = 1.5;

    private double _theDeflection;
    private int _nbPntIn;
    private int _theMaxNbPoints;
    private readonly NCollection_Array1<Point2D> _thePnts;
    private readonly NCollection_Array1<double> _theParams;
    private readonly NCollection_Array1<int> _theIndex;
    private bool _closedPolygon;
    private double _binf;
    private double _bsup;

    //======================================================================
    //== We take samples on the Domain of the Curve  NbPts Points
    //== with constant parameters.
    //==
    //== We estimate the maximum deflection taking the max distance between the
    //== right Curve.Value(X(i))-->Curve.Value(X(i+1))
    //== and the point Curve.Value(X(i+1/2))
    //======================================================================
    /// <summary>
    /// Samples <paramref name="tNbPts"/> points (at least 3) of the curve at constant parameter steps on the
    /// domain <paramref name="d"/>, which must be bounded.
    /// </summary>
    public Polygon2dGen(in TheCurve c, int tNbPts, in TwoDDomain d, double tol)
    {
        _thePnts = new NCollection_Array1<Point2D>(1, (tNbPts < 3) ? 6 : (tNbPts + tNbPts));
        _theParams = new NCollection_Array1<double>(1, (tNbPts < 3) ? 6 : (tNbPts + tNbPts));
        _theIndex = new NCollection_Array1<int>(1, (tNbPts < 3) ? 6 : (tNbPts + tNbPts));

        int nbPts = (tNbPts < 3) ? 3 : tNbPts;
        _theMaxNbPoints = nbPts + nbPts;
        _nbPntIn = nbPts;
        //-----------------------------------------------------
        //--- Initialization of the breaking with d_Parametre constant
        //---
        _binf = d.FirstParameter();
        _bsup = d.LastParameter();
        //-----------------------------------------------------
        //-- IntRes2d Raise if HasFirst returns False
        //-- and accesses First Parameter
        //--
        double u = _binf;
        double u1 = _bsup;
        double du = (u1 - u) / (double)(nbPts - 1);
        int i = 1;

        do
        {
            Point2D p = TheCurveTool.Value(c, u);
            _box.Add(p);
            _theIndex.SetValue(i, i);
            _thePnts.SetValue(i, p);
            _theParams.SetValue(i, u);
            u += du;
            i++;
        } while (i <= nbPts);

        //-----------------------------------------------------
        //--- Calculate a maximal deflection
        //---
        _theDeflection = Min(0.000000001, tol / 100.0);
        i = 1;
        u = d.FirstParameter();
        u += du * 0.5;

        do
        {
            Point2D pm = TheCurveTool.Value(c, u);
            Point2D p1 = _thePnts.Value(i);
            Point2D p2 = _thePnts.Value(i + 1);

            u += du;
            i++;

            double dx, dy, t = 0;
            dx = p1.X - p2.X;
            if (dx < 0)
            {
                dx = -dx;
            }
            dy = p1.Y - p2.Y;
            if (dy < 0)
            {
                dy = -dy;
            }
            if (dx + dy > 1e-12)
            {
                Line2D l = new(p1, new Direction2D(new Vector2D(p1, p2)));
                t = l.DistanceTo(pm);
                if (t > _theDeflection)
                {
                    _theDeflection = t;
                }
            }
        } while (i < nbPts);

        _box.Enlarge(_theDeflection * MAJORATION_DEFLECTION);
        _closedPolygon = false;
    }

    /// <summary>
    /// Keeps only the parts of the polygon near the box of the other polygon, refined where the deflection
    /// requires it.
    /// </summary>
    public void ComputeWithBox(in TheCurve c, in Box2d boxOtherPolygon)
    {
        if (_box.IsOut(boxOtherPolygon))
        {
            _nbPntIn = 2;
            _box.SetVoid();
        }
        else
        {
            boxOtherPolygon.Get(out double bx0, out double by0, out double bx1, out double by1);

            bx0 -= _theDeflection;
            by0 -= _theDeflection;
            bx1 += _theDeflection;
            by1 += _theDeflection;
            int maxIndexUsed = 1;
            int i, nbp;
            int rprec, ri;
            double x, y;

            nbp = 0;
            x = _thePnts.Value(_theIndex.Value(1)).X;
            y = _thePnts.Value(_theIndex.Value(1)).Y;

            rprec = CalculRegion(x, y, bx0, bx1, by0, by1);
            for (i = 2; i <= _nbPntIn; i++)
            {
                Point2D p2d = _thePnts.Value(_theIndex.Value(i));
                ri = CalculRegion(p2d.X, p2d.Y, bx0, bx1, by0, by1);
                if ((ri & rprec) == 0)
                {
                    if (nbp != 0)
                    {
                        if (_theIndex.Value(nbp) != _theIndex.Value(i - 1))
                        {
                            nbp++;
                            _theIndex.SetValue(nbp, _theIndex.Value(i - 1));
                        }
                    }
                    else
                    {
                        nbp++;
                        _theIndex.SetValue(nbp, _theIndex.Value(i - 1));
                    }
                    nbp++;
                    _theIndex.SetValue(nbp, _theIndex.Value(i));
                    if (_theIndex.Value(i) > maxIndexUsed)
                    {
                        maxIndexUsed = _theIndex.Value(i);
                    }

                    rprec = ri;
                } // if((Ri & Rprec)==0) condition

                rprec = ri;
            }
            if (nbp == 1)
            {
                _nbPntIn = 2;
                _box.SetVoid();
            }
            else
            {
                _box.SetVoid();
                if (nbp != 0)
                {
                    _box.Add(_thePnts.Value(_theIndex.Value(1)));
                }
                double ratioDeflection;
                int nbpassagedeflection = 0;
                do
                {
                    nbpassagedeflection++;
                    double newDeflection = _theDeflection;
                    for (i = 2; i <= nbp; i++)
                    {
                        int ii = _theIndex.Value(i);
                        int iim1 = _theIndex.Value(i - 1);
                        Point2D pi = _thePnts.Value(ii);
                        Point2D pim1 = _thePnts.Value(iim1);
                        _box.Add(pi);
                        int regi = CalculRegion(pi.X, pi.Y, bx0, bx1, by0, by1);
                        int regim1 = CalculRegion(pim1.X, pim1.Y, bx0, bx1, by0, by1);
                        if ((regi & regim1) == 0)
                        {
                            double u = 0.5 * (_theParams.Value(ii) + _theParams.Value(iim1));
                            Point2D pm = TheCurveTool.Value(c, u);
                            double dx, dy, t = 0;
                            dx = pim1.X - pi.X;
                            if (dx < 0)
                            {
                                dx = -dx;
                            }
                            dy = pim1.Y - pi.Y;
                            if (dy < 0)
                            {
                                dy = -dy;
                            }
                            if (dx + dy > 1e-12)
                            {
                                Line2D l = new(pim1, new Direction2D(new Vector2D(pim1, pi)));
                                t = l.DistanceTo(pm);
                                if ((maxIndexUsed < (_theMaxNbPoints - 1)) && (t > (_theDeflection * 0.5)))
                                {
                                    Point2D p1 = pim1;
                                    nbp++;
                                    for (int j = nbp; j >= i + 1; j--)
                                    {
                                        _theIndex.SetValue(j, _theIndex.Value(j - 1));
                                    }
                                    maxIndexUsed++;
                                    _theIndex.SetValue(i, maxIndexUsed);
                                    _thePnts.SetValue(maxIndexUsed, pm);
                                    _theParams.SetValue(maxIndexUsed, u);

                                    // OCCT computes a deflection at the new quarter point and never uses it.
                                    double u1m = 0.5 * (u + _theParams.Value(_theIndex.Value(i - 1)));
                                    Point2D p1m = TheCurveTool.Value(c, u1m);
                                    Line2D l1m = new(p1, new Direction2D(new Vector2D(p1, pm)));
                                    t = l1m.DistanceTo(p1m);
                                    i--;
                                }
                            }
                            else
                            {
                                if (t > newDeflection)
                                {
                                    newDeflection = t;
                                }
                            }
                        }
                    }
                    if (newDeflection != 0)
                    {
                        ratioDeflection = _theDeflection / newDeflection;
                    }
                    else
                    {
                        ratioDeflection = 10.0;
                    }
                    _theDeflection = newDeflection;
                    _nbPntIn = nbp;
                } while ((ratioDeflection < 3.0) && (nbpassagedeflection < 3)
                         && (maxIndexUsed < (_theMaxNbPoints - 2)));
            }

            _theDeflection *= MAJORATION_DEFLECTION;
            _box.Enlarge(_theDeflection);
        }
        _closedPolygon = false;
    }

    /// <summary>The over-estimated deflection of the polygon.</summary>
    public override double DeflectionOverEstimation() => (_theDeflection);

    /// <summary>Sets the deflection and enlarges the box by it.</summary>
    public void SetDeflectionOverEstimation(double x)
    {
        _theDeflection = x;
        _box.Enlarge(_theDeflection);
    }

    /// <summary>Sets whether the polygon is closed.</summary>
    public void Closed(bool flag)
    {
        _closedPolygon = flag;
    }

    public override bool Closed() => _closedPolygon;

    public override int SegmentCount => ((_closedPolygon) ? _nbPntIn : _nbPntIn - 1);

    /// <summary>The points of the segment <paramref name="index"/>.</summary>
    /// <exception cref="OutOfRangeException">The index is the last point's of an open polygon.</exception>
    public override void Segment(int index, out Point2D begin, out Point2D end)
    {
        int ind = index;
        begin = _thePnts[_theIndex[index]];
        if (index >= _nbPntIn)
        {
            if (!_closedPolygon)
            {
                throw new OutOfRangeException("IntCurve_Polygon2dGen::Segment!");
            }
            ind = 0;
        }
        end = _thePnts[_theIndex[ind + 1]];
    }

    /// <summary>The parameter of the first point.</summary>
    public double InfParameter() => (_theParams.Value(_theIndex[1]));

    /// <summary>The parameter of the last point.</summary>
    public double SupParameter() => (_theParams.Value(_theIndex[_nbPntIn]));

    /// <summary>True if two segments of the polygon go in opposite directions (a self-intersection is possible).</summary>
    public bool AutoIntersectionIsPossible()
    {
        Vector2D vRef = new(_thePnts.Value(_theIndex.Value(1)), _thePnts.Value(_theIndex.Value(2)));
        for (int i = 3; i <= _nbPntIn; i++)
        {
            Vector2D v = new(_thePnts.Value(_theIndex.Value(i - 1)), _thePnts.Value(_theIndex.Value(i)));
            if (v.Dot(vRef) < 0.0)
            {
                return (true);
            }
        }
        return (false);
    }

    /// <summary>
    /// The parameter on the curve of the point at <paramref name="theParamOnLine"/> (0 to 1) on the segment
    /// <paramref name="aindex"/>.
    /// </summary>
    public double ApproxParamOnCurve(int aindex, double theParamOnLine)
    {
        int indexp1, index = aindex;
        double paramOnLine = theParamOnLine;
        // OCCT prints "OutOfRange Polygon2d::ApproxParamOnCurve" when Index > NbPntIn, and goes on.
        if ((index == _nbPntIn) && (paramOnLine == 0.0))
        {
            index--;
            paramOnLine = 1.0;
        }
        if (index == 0)
        {
            index = 1;
            paramOnLine = 0.0;
        }
        indexp1 = _theIndex.Value(index + 1);
        index = _theIndex.Value(index);

        double du = _theParams.Value(indexp1) - _theParams.Value(index);
        double u = _theParams.Value(index) + paramOnLine * du;
        return (u);
    }

    /// <summary>The region code of the point (x, y) against the box [x1, x2] × [y1, y2] (0 inside).</summary>
    public int CalculRegion(double x, double y, double x1, double x2, double y1, double y2)
    {
        int r;
        if (x < x1)
        {
            r = 1;
        }
        else
        {
            if (x > x2)
            {
                r = 2;
            }
            else
            {
                r = 0;
            }
        }
        if (y < y1)
        {
            r |= 4;
        }
        else
        {
            if (y > y2)
            {
                r |= 8;
            }
        }
        return (r);
    }
}
