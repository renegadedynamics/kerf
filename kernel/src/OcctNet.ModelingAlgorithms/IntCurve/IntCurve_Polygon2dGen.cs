// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_Polygon2dGen.gxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_Polygon2dGen.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// A polygon sampling a 2D curve on a domain, with the over-estimated deflection and the box the 2D
/// interferences need; it can be refined where it meets the box of another polygon.
/// </summary>
/// <remarks>
/// OCCT's generic file IntCurve_Polygon2dGen.gxx, ported once as a generic class (decision 324); its
/// instantiation Geom2dInt_ThePolygon2dOfTheIntPCurvePCurveOfGInter is a sealed subclass. OCCT's <c>Dump</c>, which
/// prints only when a static debug flag is set, is left out.
/// </remarks>
public class IntCurve_Polygon2dGen<TheCurve, TheCurveTool> : Intf_Polygon2d
    where TheCurveTool : IIntCurve_CurveTool<TheCurve>
{
    private const double MAJORATION_DEFLECTION = 1.5;

    private double TheDeflection;
    private int NbPntIn;
    private int TheMaxNbPoints;
    private readonly NCollection_Array1<Point2D> ThePnts;
    private readonly NCollection_Array1<double> TheParams;
    private readonly NCollection_Array1<int> TheIndex;
    private bool ClosedPolygon;
    private double Binf;
    private double Bsup;

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
    /// domain <paramref name="D"/>, which must be bounded.
    /// </summary>
    public IntCurve_Polygon2dGen(in TheCurve C, int tNbPts, in IntRes2d_Domain D, double Tol)
    {
        ThePnts = new NCollection_Array1<Point2D>(1, (tNbPts < 3) ? 6 : (tNbPts + tNbPts));
        TheParams = new NCollection_Array1<double>(1, (tNbPts < 3) ? 6 : (tNbPts + tNbPts));
        TheIndex = new NCollection_Array1<int>(1, (tNbPts < 3) ? 6 : (tNbPts + tNbPts));

        int NbPts = (tNbPts < 3) ? 3 : tNbPts;
        TheMaxNbPoints = NbPts + NbPts;
        NbPntIn = NbPts;
        //-----------------------------------------------------
        //--- Initialization of the breaking with d_Parametre constant
        //---
        Binf = D.FirstParameter();
        Bsup = D.LastParameter();
        //-----------------------------------------------------
        //-- IntRes2d Raise if HasFirst returns False
        //-- and accesses First Parameter
        //--
        double u = Binf;
        double u1 = Bsup;
        double du = (u1 - u) / (double)(NbPts - 1);
        int i = 1;

        do
        {
            Point2D P = TheCurveTool.Value(C, u);
            myBox.Add(P);
            TheIndex.SetValue(i, i);
            ThePnts.SetValue(i, P);
            TheParams.SetValue(i, u);
            u += du;
            i++;
        } while (i <= NbPts);

        //-----------------------------------------------------
        //--- Calculate a maximal deflection
        //---
        TheDeflection = Min(0.000000001, Tol / 100.0);
        i = 1;
        u = D.FirstParameter();
        u += du * 0.5;

        do
        {
            Point2D Pm = TheCurveTool.Value(C, u);
            Point2D P1 = ThePnts.Value(i);
            Point2D P2 = ThePnts.Value(i + 1);

            u += du;
            i++;

            double dx, dy, t = 0;
            dx = P1.X() - P2.X();
            if (dx < 0)
            {
                dx = -dx;
            }
            dy = P1.Y() - P2.Y();
            if (dy < 0)
            {
                dy = -dy;
            }
            if (dx + dy > 1e-12)
            {
                Line2D L = new(P1, new Direction2D(new Vector2D(P1, P2)));
                t = L.Distance(Pm);
                if (t > TheDeflection)
                {
                    TheDeflection = t;
                }
            }
        } while (i < NbPts);

        myBox.Enlarge(TheDeflection * MAJORATION_DEFLECTION);
        ClosedPolygon = false;
    }

    /// <summary>
    /// Keeps only the parts of the polygon near the box of the other polygon, refined where the deflection
    /// requires it.
    /// </summary>
    public void ComputeWithBox(in TheCurve C, in Box2d BoxOtherPolygon)
    {
        if (myBox.IsOut(BoxOtherPolygon))
        {
            NbPntIn = 2;
            myBox.SetVoid();
        }
        else
        {
            BoxOtherPolygon.Get(out double bx0, out double by0, out double bx1, out double by1);

            bx0 -= TheDeflection;
            by0 -= TheDeflection;
            bx1 += TheDeflection;
            by1 += TheDeflection;
            int MaxIndexUsed = 1;
            int i, nbp;
            int Rprec, Ri;
            double x, y;

            nbp = 0;
            x = ThePnts.Value(TheIndex.Value(1)).X();
            y = ThePnts.Value(TheIndex.Value(1)).Y();

            Rprec = CalculRegion(x, y, bx0, bx1, by0, by1);
            for (i = 2; i <= NbPntIn; i++)
            {
                Point2D P2d = ThePnts.Value(TheIndex.Value(i));
                Ri = CalculRegion(P2d.X(), P2d.Y(), bx0, bx1, by0, by1);
                if ((Ri & Rprec) == 0)
                {
                    if (nbp != 0)
                    {
                        if (TheIndex.Value(nbp) != TheIndex.Value(i - 1))
                        {
                            nbp++;
                            TheIndex.SetValue(nbp, TheIndex.Value(i - 1));
                        }
                    }
                    else
                    {
                        nbp++;
                        TheIndex.SetValue(nbp, TheIndex.Value(i - 1));
                    }
                    nbp++;
                    TheIndex.SetValue(nbp, TheIndex.Value(i));
                    if (TheIndex.Value(i) > MaxIndexUsed)
                    {
                        MaxIndexUsed = TheIndex.Value(i);
                    }

                    Rprec = Ri;
                } // if((Ri & Rprec)==0) condition

                Rprec = Ri;
            }
            if (nbp == 1)
            {
                NbPntIn = 2;
                myBox.SetVoid();
            }
            else
            {
                myBox.SetVoid();
                if (nbp != 0)
                {
                    myBox.Add(ThePnts.Value(TheIndex.Value(1)));
                }
                double RatioDeflection;
                int nbpassagedeflection = 0;
                do
                {
                    nbpassagedeflection++;
                    double NewDeflection = TheDeflection;
                    for (i = 2; i <= nbp; i++)
                    {
                        int Ii = TheIndex.Value(i);
                        int Iim1 = TheIndex.Value(i - 1);
                        Point2D Pi = ThePnts.Value(Ii);
                        Point2D Pim1 = ThePnts.Value(Iim1);
                        myBox.Add(Pi);
                        int Regi = CalculRegion(Pi.X(), Pi.Y(), bx0, bx1, by0, by1);
                        int Regim1 = CalculRegion(Pim1.X(), Pim1.Y(), bx0, bx1, by0, by1);
                        if ((Regi & Regim1) == 0)
                        {
                            double u = 0.5 * (TheParams.Value(Ii) + TheParams.Value(Iim1));
                            Point2D Pm = TheCurveTool.Value(C, u);
                            double dx, dy, t = 0;
                            dx = Pim1.X() - Pi.X();
                            if (dx < 0)
                            {
                                dx = -dx;
                            }
                            dy = Pim1.Y() - Pi.Y();
                            if (dy < 0)
                            {
                                dy = -dy;
                            }
                            if (dx + dy > 1e-12)
                            {
                                Line2D L = new(Pim1, new Direction2D(new Vector2D(Pim1, Pi)));
                                t = L.Distance(Pm);
                                if ((MaxIndexUsed < (TheMaxNbPoints - 1)) && (t > (TheDeflection * 0.5)))
                                {
                                    Point2D P1 = Pim1;
                                    nbp++;
                                    for (int j = nbp; j >= i + 1; j--)
                                    {
                                        TheIndex.SetValue(j, TheIndex.Value(j - 1));
                                    }
                                    MaxIndexUsed++;
                                    TheIndex.SetValue(i, MaxIndexUsed);
                                    ThePnts.SetValue(MaxIndexUsed, Pm);
                                    TheParams.SetValue(MaxIndexUsed, u);

                                    // OCCT computes a deflection at the new quarter point and never uses it.
                                    double u1m = 0.5 * (u + TheParams.Value(TheIndex.Value(i - 1)));
                                    Point2D P1m = TheCurveTool.Value(C, u1m);
                                    Line2D L1m = new(P1, new Direction2D(new Vector2D(P1, Pm)));
                                    t = L1m.Distance(P1m);
                                    i--;
                                }
                            }
                            else
                            {
                                if (t > NewDeflection)
                                {
                                    NewDeflection = t;
                                }
                            }
                        }
                    }
                    if (NewDeflection != 0)
                    {
                        RatioDeflection = TheDeflection / NewDeflection;
                    }
                    else
                    {
                        RatioDeflection = 10.0;
                    }
                    TheDeflection = NewDeflection;
                    NbPntIn = nbp;
                } while ((RatioDeflection < 3.0) && (nbpassagedeflection < 3)
                         && (MaxIndexUsed < (TheMaxNbPoints - 2)));
            }

            TheDeflection *= MAJORATION_DEFLECTION;
            myBox.Enlarge(TheDeflection);
        }
        ClosedPolygon = false;
    }

    /// <summary>The over-estimated deflection of the polygon.</summary>
    public override double DeflectionOverEstimation() => (TheDeflection);

    /// <summary>Sets the deflection and enlarges the box by it.</summary>
    public void SetDeflectionOverEstimation(double x)
    {
        TheDeflection = x;
        myBox.Enlarge(TheDeflection);
    }

    /// <summary>Sets whether the polygon is closed.</summary>
    public void Closed(bool flag)
    {
        ClosedPolygon = flag;
    }

    public override bool Closed() => ClosedPolygon;

    public override int NbSegments() => ((ClosedPolygon) ? NbPntIn : NbPntIn - 1);

    /// <summary>The points of the segment <paramref name="theIndex"/>.</summary>
    /// <exception cref="OutOfRangeException">The index is the last point's of an open polygon.</exception>
    public override void Segment(int theIndex, out Point2D theBegin, out Point2D theEnd)
    {
        int ind = theIndex;
        theBegin = ThePnts[TheIndex[theIndex]];
        if (theIndex >= NbPntIn)
        {
            if (!ClosedPolygon)
            {
                throw new OutOfRangeException("IntCurve_Polygon2dGen::Segment!");
            }
            ind = 0;
        }
        theEnd = ThePnts[TheIndex[ind + 1]];
    }

    /// <summary>The parameter of the first point.</summary>
    public double InfParameter() => (TheParams.Value(TheIndex[1]));

    /// <summary>The parameter of the last point.</summary>
    public double SupParameter() => (TheParams.Value(TheIndex[NbPntIn]));

    /// <summary>True if two segments of the polygon go in opposite directions (a self-intersection is possible).</summary>
    public bool AutoIntersectionIsPossible()
    {
        Vector2D VRef = new(ThePnts.Value(TheIndex.Value(1)), ThePnts.Value(TheIndex.Value(2)));
        for (int i = 3; i <= NbPntIn; i++)
        {
            Vector2D V = new(ThePnts.Value(TheIndex.Value(i - 1)), ThePnts.Value(TheIndex.Value(i)));
            if (V.Dot(VRef) < 0.0)
            {
                return (true);
            }
        }
        return (false);
    }

    /// <summary>
    /// The parameter on the curve of the point at <paramref name="TheParamOnLine"/> (0 to 1) on the segment
    /// <paramref name="Aindex"/>.
    /// </summary>
    public double ApproxParamOnCurve(int Aindex, double TheParamOnLine)
    {
        int Indexp1, Index = Aindex;
        double ParamOnLine = TheParamOnLine;
        // OCCT prints "OutOfRange Polygon2d::ApproxParamOnCurve" when Index > NbPntIn, and goes on.
        if ((Index == NbPntIn) && (ParamOnLine == 0.0))
        {
            Index--;
            ParamOnLine = 1.0;
        }
        if (Index == 0)
        {
            Index = 1;
            ParamOnLine = 0.0;
        }
        Indexp1 = TheIndex.Value(Index + 1);
        Index = TheIndex.Value(Index);

        double du = TheParams.Value(Indexp1) - TheParams.Value(Index);
        double u = TheParams.Value(Index) + ParamOnLine * du;
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
