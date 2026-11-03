// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GLocateExtPC.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_LocateExtPC2d.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>
/// The extremum distance between a point and a 2D curve nearest a starting parameter U0. On elementary curves it is
/// the extremum of <see cref="Extrema_ExtPC2d"/> whose parameter is nearest U0; on B-spline, offset and other curves
/// the local search (<see cref="Extrema_LocEPCOfLocateExtPC2d"/>) starts on the C2 interval holding U0 and moves out to
/// the neighbouring intervals until it succeeds; on Bezier curves it is the local search alone.
/// </summary>
/// <remarks>
/// The 2D instantiation of OCCT's template Extrema_GLocateExtPC (Extrema_LocateExtPC2d). An algorithm object (decision
/// 117). Keeps a reference to the curve, as OCCT does.
/// </remarks>
public class Extrema_LocateExtPC2d
{
    private Extrema_POnCurv2d mypp = new();
    private Adaptor2d_Curve2d? myC;
    private double mydist2;
    private bool myismin;
    private bool myDone;
    private double myumin;
    private double myusup;
    private double mytol;
    private readonly Extrema_LocEPCOfLocateExtPC2d myLocExtPC = new();
    private readonly Extrema_ExtPC2d myExtremPC = new();
    private CurveKind type;
    private int numberext;

    public Extrema_LocateExtPC2d()
    {
        myC = null;
        mydist2 = 0.0;
        myismin = false;
        myDone = false;
        myumin = 0.0;
        myusup = 0.0;
        mytol = 0.0;
        type = OtherCurve;
        numberext = 0;
    }

    /// <summary>The extremum between theP and theC nearest theU0, on the curve's whole range.</summary>
    public Extrema_LocateExtPC2d(in Point2D theP, Adaptor2d_Curve2d theC, double theU0, double theTolF)
    {
        Initialize(theC, Extrema_Curve2dTool.FirstParameter(theC), Extrema_Curve2dTool.LastParameter(theC), theTolF);
        Perform(theP, theU0);
    }

    /// <summary>The extremum between theP and theC nearest theU0, on [theUmin, theUsup].</summary>
    public Extrema_LocateExtPC2d(in Point2D theP, Adaptor2d_Curve2d theC, double theU0, double theUmin, double theUsup,
        double theTolF)
    {
        Initialize(theC, theUmin, theUsup, theTolF);
        Perform(theP, theU0);
    }

    /// <summary>Sets the curve, the range and the tolerance.</summary>
    public void Initialize(Adaptor2d_Curve2d theC, double theUmin, double theUsup, double theTolF)
    {
        myC = theC;
        mytol = theTolF;
        myumin = theUmin;
        myusup = theUsup;
        type = Extrema_Curve2dTool.GetType(theC);
        double tolu = Extrema_Curve2dTool.Resolution(theC, global::Kerf.Engine.Foundation.Precision.Confusion());
        if ((type == BSplineCurve) || (type == BezierCurve)
            || (type == OffsetCurve) || (type == OtherCurve))
        {
            myLocExtPC.Initialize(theC, theUmin, theUsup, tolu);
        }
        else
        {
            myExtremPC.Initialize(theC, theUmin, theUsup, tolu);
        }
    }

    /// <summary>Searches the extremum for theP nearest the parameter theU0.</summary>
    public void Perform(in Point2D theP, double theU0)
    {
        int i, i1, i2, inter;
        double Par, valU, valU2 = RealLast(), local_u0;
        double myintuinf = 0, myintusup = 0;
        local_u0 = theU0;
        switch (type)
        {
            case OtherCurve:
            case OffsetCurve:
            case BSplineCurve:
            {
                // Search for extremum is done interval by continuous C2 interval
                int n = Extrema_Curve2dTool.NbIntervals(myC!, C2);
                NCollection_Array1<double> theInter = new(1, n + 1);
                Extrema_Curve2dTool.Intervals(myC!, theInter, C2);
                //
                // be gentle with the caller
                //
                if (local_u0 < myumin)
                {
                    local_u0 = myumin;
                }
                else if (local_u0 > myusup)
                {
                    local_u0 = myusup;
                }
                // Search for interval containing U0
                bool found = false;
                inter = 1;
                while (!found && inter <= n)
                {
                    myintuinf = Max(theInter[inter], myumin);
                    myintusup = Min(theInter[inter + 1], myusup);
                    if ((local_u0 >= myintuinf) && (local_u0 < myintusup))
                    {
                        found = true;
                    }
                    inter++;
                }

                if (found)
                {
                    inter--; // IFV 16.06.00 - inter is increased after found!
                }

                // Try on found interval
                myLocExtPC.Initialize(myC!, myintuinf, myintusup, mytol);
                myLocExtPC.Perform(theP, local_u0);
                myDone = myLocExtPC.IsDone();
                if (myDone)
                {
                    mypp = myLocExtPC.Point();
                    myismin = myLocExtPC.IsMin();
                    mydist2 = myLocExtPC.SquareDistance();
                }
                else
                {
                    int k = 1;
                    // Try on neighboring intervals:
                    i1 = inter;
                    i2 = inter;
                    double s1inf, s2inf, s1sup, s2sup;
                    Point2D P1;
                    Vector2D V1;
                    Extrema_Curve2dTool.D1(myC!, myintuinf, out P1, out V1);
                    s2inf = (new Vector2D(theP, P1) * V1);
                    Extrema_Curve2dTool.D1(myC!, myintusup, out P1, out V1);
                    s1sup = (new Vector2D(theP, P1) * V1);

                    while (!myDone && (i2 > 0) && (i1 <= n))
                    {
                        i1 = inter + k;
                        i2 = inter - k;
                        if (i1 <= n)
                        {
                            myintuinf = Max(theInter[i1], myumin);
                            myintusup = Min(theInter[i1 + 1], myusup);
                            if (myintuinf < myintusup)
                            {
                                Extrema_Curve2dTool.D1(myC!, myintuinf, out P1, out V1);
                                s2sup = (new Vector2D(theP, P1) * V1);
                                if (global::Kerf.Engine.Foundation.Precision.IsInfinite(s2sup) || global::Kerf.Engine.Foundation.Precision.IsInfinite(s1sup))
                                {
                                    break;
                                }
                                if (s1sup * s2sup <= RealEpsilon())
                                {
                                    // extremum:
                                    myDone = true;
                                    mypp.SetValues(myintuinf, P1);
                                    myismin = (s1sup <= 0.0);
                                    mydist2 = theP.SquareDistance(P1);
                                    break;
                                }

                                Extrema_Curve2dTool.D1(myC!, myintusup, out P1, out V1);
                                s1sup = (new Vector2D(theP, P1) * V1);
                                myLocExtPC.Initialize(myC!, myintuinf, myintusup, mytol);
                                myLocExtPC.Perform(theP, (myintuinf + myintusup) * 0.5);
                                myDone = myLocExtPC.IsDone();
                                if (myDone)
                                {
                                    mypp = myLocExtPC.Point();
                                    myismin = myLocExtPC.IsMin();
                                    mydist2 = myLocExtPC.SquareDistance();
                                    break;
                                }
                            }
                        }

                        if (i2 > 0)
                        {
                            myintuinf = Max(theInter[i2], myumin);
                            myintusup = Min(theInter[i2 + 1], myusup);
                            if (myintuinf < myintusup)
                            {
                                Extrema_Curve2dTool.D1(myC!, myintusup, out P1, out V1);
                                s1inf = (new Vector2D(theP, P1) * V1);
                                if (global::Kerf.Engine.Foundation.Precision.IsInfinite(s2inf) || global::Kerf.Engine.Foundation.Precision.IsInfinite(s1inf))
                                {
                                    break;
                                }
                                if (s1inf * s2inf <= RealEpsilon())
                                {
                                    // extremum:
                                    myDone = true;
                                    mypp.SetValues(myintusup, P1);
                                    myismin = (s1inf <= 0.0);
                                    mydist2 = theP.SquareDistance(P1);
                                    break;
                                }

                                Extrema_Curve2dTool.D1(myC!, myintuinf, out P1, out V1);
                                s2inf = (new Vector2D(theP, P1) * V1);
                                myLocExtPC.Initialize(myC!, myintuinf, myintusup, mytol);
                                myLocExtPC.Perform(theP, (myintuinf + myintusup) * 0.5);
                                myDone = myLocExtPC.IsDone();

                                if (myDone)
                                {
                                    mypp = myLocExtPC.Point();
                                    myismin = myLocExtPC.IsMin();
                                    mydist2 = myLocExtPC.SquareDistance();
                                    break;
                                }
                            }
                        }

                        k++;
                    }
                }
            }

            break;

            case BezierCurve:
            {
                myLocExtPC.Perform(theP, theU0);
                myDone = myLocExtPC.IsDone();
            }

            break;
            default:
            {
                myExtremPC.Perform(theP);
                numberext = 0;
                if (myExtremPC.IsDone())
                {
                    for (i = 1; i <= myExtremPC.NbExt(); i++)
                    {
                        Par = myExtremPC.Point(i).Parameter();
                        valU = Math.Abs(Par - theU0);
                        if (valU <= valU2)
                        {
                            valU2 = valU;
                            numberext = i;
                            myDone = true;
                        }
                    }
                }

                if (numberext == 0)
                {
                    myDone = false;
                }

                break;
            }
        }
    }

    /// <summary>True if the distance is found.</summary>
    public bool IsDone() => myDone;

    /// <summary>The square distance of the extremum.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public double SquareDistance()
    {
        if (!IsDone())
        {
            throw new NotDoneException();
        }
        double d = 0;
        if ((type == BezierCurve))
        {
            d = myLocExtPC.SquareDistance();
        }
        else if (type == BSplineCurve || type == OffsetCurve || type == OtherCurve)
        {
            d = mydist2;
        }
        else
        {
            if (numberext != 0)
            {
                d = myExtremPC.SquareDistance(numberext);
            }
        }
        return d;
    }

    /// <summary>True if the extremum is a minimum.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public bool IsMin()
    {
        if (!IsDone())
        {
            throw new NotDoneException();
        }
        bool b = false;
        if ((type == BezierCurve))
        {
            b = myLocExtPC.IsMin();
        }
        else if (type == BSplineCurve || type == OffsetCurve || type == OtherCurve)
        {
            b = myismin;
        }
        else
        {
            if (numberext != 0)
            {
                b = myExtremPC.IsMin(numberext);
            }
        }
        return b;
    }

    /// <summary>The point of the extremum.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public Extrema_POnCurv2d Point()
    {
        if (!IsDone())
        {
            throw new NotDoneException();
        }
        if (type == BezierCurve)
        {
            return myLocExtPC.Point();
        }
        else if (type == BSplineCurve || type == OffsetCurve || type == OtherCurve)
        {
            return mypp;
        }
        return myExtremPC.Point(numberext);
    }
}
