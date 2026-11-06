// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntRes2d/IntRes2d_Intersection.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntRes2d/IntRes2d_Intersection.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntRes2d/IntRes2d_Intersection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;

namespace Kerf.Engine.Intersection.TwoD;

/// <summary>
/// The root class of all the intersections between two 2D curves: the results (points and segments) of the
/// intersection algorithms.
/// </summary>
/// <remarks>
/// An abstract class: OCCT's constructors are protected, and only the algorithms derive from it. OCCT's protected
/// copy constructor is kept (it copies the result sequences); its default assignment operator is
/// <see cref="SetValues"/> or <see cref="Assign"/> in ported code.
/// </remarks>
internal abstract class Intersection2D
{
    protected NCollection_Sequence<IntersectionPoint2D> _lpnt = new();
    protected NCollection_Sequence<IntersectionSegment2D> _lseg = new();
    protected bool _done;
    protected bool _reverse;

    /// <summary>Empty constructor.</summary>
    protected Intersection2D()
    {
        _done = _reverse = false;
    }

    /// <summary>Copies the points and segments of <paramref name="other"/>; the new object is not done.</summary>
    protected Intersection2D(Intersection2D other)
    {
        _done = _reverse = false;
        _lpnt = new NCollection_Sequence<IntersectionPoint2D>(other._lpnt);
        _lseg = new NCollection_Sequence<IntersectionSegment2D>(other._lseg);
    }

    /// <summary>C++'s default assignment operator: copies the results and both flags.</summary>
    protected void Assign(Intersection2D other)
    {
        _lpnt.Assign(other._lpnt);
        _lseg.Assign(other._lseg);
        _done = other._done;
        _reverse = other._reverse;
    }

    /// <summary>True when the computation was successful.</summary>
    public bool IsDone => _done;

    /// <summary>True if there is no intersection between the given arguments.</summary>
    /// <exception cref="NotDoneException"><see cref="IsDone"/> is false.</exception>
    public bool IsEmpty
    {
        get
        {
            if (!_done)
            {
                throw new NotDoneException();
            }

            return ((_lpnt.Length() == 0) && (_lseg.Length() == 0));
        }
    }

    /// <summary>The number of intersection points between the two curves.</summary>
    /// <exception cref="NotDoneException"><see cref="IsDone"/> is false.</exception>
    public int PointCount
    {
        get
        {
            if (!_done)
            {
                throw new NotDoneException();
            }

            return _lpnt.Length();
        }
    }

    /// <summary>The intersection point of range <paramref name="n"/> (1 to <see cref="PointCount"/>).</summary>
    /// <exception cref="NotDoneException"><see cref="IsDone"/> is false.</exception>
    /// <exception cref="OutOfRangeException"><paramref name="n"/> is out of range.</exception>
    public IntersectionPoint2D Point(int n)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }

        return _lpnt[n];
    }

    /// <summary>The number of intersection segments between the two curves.</summary>
    /// <exception cref="NotDoneException"><see cref="IsDone"/> is false.</exception>
    public int SegmentCount
    {
        get
        {
            if (!_done)
            {
                throw new NotDoneException();
            }

            return _lseg.Length();
        }
    }

    /// <summary>The intersection segment of range <paramref name="n"/> (1 to <see cref="SegmentCount"/>).</summary>
    /// <exception cref="NotDoneException"><see cref="IsDone"/> is false.</exception>
    /// <exception cref="OutOfRangeException"><paramref name="n"/> is out of range.</exception>
    public IntersectionSegment2D Segment(int n)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }

        return _lseg[n];
    }

    /// <summary>Whether the next computation reports its parameters and transitions with the curves swapped.</summary>
    public void SetReversedParameters(bool flag)
    {
        _reverse = flag;
    }

    /// <summary>Copies the results of <paramref name="other"/> if it is done; otherwise this becomes not done.</summary>
    protected void SetValues(Intersection2D other)
    {
        int i;
        if (other._done)
        {
            _lseg.Clear();
            _lpnt.Clear();
            int n = other._lpnt.Length();
            for (i = 1; i <= n; i++)
            {
                _lpnt.Append(other._lpnt[i]);
            }

            n = other._lseg.Length();
            for (i = 1; i <= n; i++)
            {
                _lseg.Append(other._lseg[i]);
            }

            //-----------------------
            // lpnt=Other.lpnt;  Pose des problemes
            // lseg=Other.lseg;  pour des objets composites
            //-----------------------
            _done = true;
        }
        else
        {
            _done = false;
        }
    }

    /// <summary>
    /// Merges the results of another intersection, for composite curves only. The parameters are the bounds of the
    /// composite curves; points strictly inside them get the position Middle, and segments that meet end to end are
    /// joined.
    /// </summary>
    protected void Append(Intersection2D other, double firstParam1, double lastParam1, double firstParam2,
                          double lastParam2)
    {
        if (other._done)
        {
            //-- Verification of the Position of the IntersectionPoints
            int n = other._lpnt.Length();
            int i;
            for (i = 1; i <= n; i++)
            {
                IntersectionPoint2D p = other._lpnt[i];
                double pParamOnFirst = p.ParamOnFirst();
                double pParamOnSecond = p.ParamOnSecond();
                Transition t1 = p.TransitionOfFirst();
                Transition t2 = p.TransitionOfSecond();
                Point2D pt = p.Value();

                InternalVerifyPosition(ref t1,
                                       ref t2,
                                       pParamOnFirst,
                                       pParamOnSecond,
                                       firstParam1,
                                       lastParam1,
                                       firstParam2,
                                       lastParam2);

                this.Insert(new IntersectionPoint2D(pt, pParamOnFirst, pParamOnSecond, t1, t2, false));
            }

            //--------------------------------------------------
            //-- IntersectionSegment
            //-- (we assume that a composite curve is always bounded)
            //-- (a segment has always a FirstPoint and a LastPoint)
            //--------------------------------------------------
            n = other._lseg.Length();
            double segModifP1First = 0, segModifP1Second = 0;
            double segModifP2First = 0, segModifP2Second = 0;

            for (i = 1; i <= n; i++)
            {
                IntersectionPoint2D p1 = other._lseg[i].FirstPoint();

                double p1PParamOnFirst = p1.ParamOnFirst();
                double p1PParamOnSecond = p1.ParamOnSecond();
                Transition p1T1 = p1.TransitionOfFirst();
                Transition p1T2 = p1.TransitionOfSecond();
                Point2D p1Pt = p1.Value();

                InternalVerifyPosition(ref p1T1,
                                       ref p1T2,
                                       p1PParamOnFirst,
                                       p1PParamOnSecond,
                                       firstParam1,
                                       lastParam1,
                                       firstParam2,
                                       lastParam2);

                IntersectionPoint2D p2 = other._lseg[i].LastPoint();

                double p2PParamOnFirst = p2.ParamOnFirst();
                double p2PParamOnSecond = p2.ParamOnSecond();
                Transition p2T1 = p2.TransitionOfFirst();
                Transition p2T2 = p2.TransitionOfSecond();
                Point2D p2Pt = p2.Value();

                bool opposite = other._lseg[i].IsOpposite;

                InternalVerifyPosition(ref p2T1,
                                       ref p2T2,
                                       p2PParamOnFirst,
                                       p2PParamOnSecond,
                                       firstParam1,
                                       lastParam1,
                                       firstParam2,
                                       lastParam2);

                //-- Loop on the previous segments
                //--
                int an = _lseg.Length();
                bool notYetModified = true;

                for (int j = 1; (j <= an) && (notYetModified); j++)
                {
                    IntersectionPoint2D anP1 = _lseg[j].FirstPoint();
                    double anP1PParamOnFirst = anP1.ParamOnFirst();
                    double anP1PParamOnSecond = anP1.ParamOnSecond();

                    IntersectionPoint2D anP2 = _lseg[j].LastPoint();
                    double anP2PParamOnFirst = anP2.ParamOnFirst();
                    double anP2PParamOnSecond = anP2.ParamOnSecond();

                    if (opposite == _lseg[j].IsOpposite)
                    {
                        //---------------------------------------------------------------
                        //--    AnP1---------AnP2
                        //--                  P1-------------P2
                        //--
                        if (PARAMEQUAL(p1PParamOnFirst, anP2PParamOnFirst)
                            && PARAMEQUAL(p1PParamOnSecond, anP2PParamOnSecond))
                        {
                            notYetModified = false;
                            _lseg[j] = new IntersectionSegment2D(anP1, p2, opposite, false);
                            segModifP1First = anP1PParamOnFirst;
                            segModifP1Second = anP1PParamOnSecond;
                            segModifP2First = p2PParamOnFirst;
                            segModifP2Second = p2PParamOnSecond;
                        }
                        //---------------------------------------------------------------
                        //--                                AnP1---------AnP2
                        //--                  P1-------------P2
                        //--
                        else if (PARAMEQUAL(p2PParamOnFirst, anP1PParamOnFirst)
                                 && PARAMEQUAL(p2PParamOnSecond, anP1PParamOnSecond))
                        {
                            notYetModified = false;
                            _lseg[j] = new IntersectionSegment2D(p1, anP2, opposite, false);
                            segModifP1First = p1PParamOnFirst;
                            segModifP1Second = p1PParamOnSecond;
                            segModifP2First = anP2PParamOnFirst;
                            segModifP2Second = anP2PParamOnSecond;
                        }

                        //---------------------------------------------------------------
                        //--    AnP2---------AnP1
                        //--                  P1-------------P2
                        //--
                        if (PARAMEQUAL(p1PParamOnFirst, anP1PParamOnFirst)
                            && PARAMEQUAL(p1PParamOnSecond, anP1PParamOnSecond))
                        {
                            notYetModified = false;
                            _lseg[j] = new IntersectionSegment2D(anP2, p2, opposite, false);
                            segModifP1First = p2PParamOnFirst;
                            segModifP1Second = p2PParamOnSecond;
                            segModifP2First = anP2PParamOnFirst;
                            segModifP2Second = anP2PParamOnSecond;
                        }
                        //---------------------------------------------------------------
                        //--                                AnP2---------AnP1
                        //--                  P1-------------P2
                        //--
                        else if (PARAMEQUAL(p2PParamOnFirst, anP2PParamOnFirst)
                                 && PARAMEQUAL(p2PParamOnSecond, anP2PParamOnSecond))
                        {
                            notYetModified = false;
                            _lseg[j] = new IntersectionSegment2D(p1, anP1, opposite, false);
                            segModifP1First = p1PParamOnFirst;
                            segModifP1Second = p1PParamOnSecond;
                            segModifP2First = anP1PParamOnFirst;
                            segModifP2Second = anP1PParamOnSecond;
                        }
                    }
                }

                if (notYetModified)
                {
                    this.Append(new IntersectionSegment2D(
                        new IntersectionPoint2D(p1Pt, p1PParamOnFirst, p1PParamOnSecond, p1T1, p1T2, false),
                        new IntersectionPoint2D(p2Pt, p2PParamOnFirst, p2PParamOnSecond, p2T1, p2T2, false),
                        opposite,
                        false));
                } //-- if(NotYetModified)
                else
                {
                    //--------------------------------------------------------------
                    //-- Are some Existing Points in this segment ?
                    //--------------------------------------------------------------
                    int rnbpts = _lpnt.Length();
                    for (int rp = 1; (rp <= rnbpts) && (rp >= 1); rp++)
                    {
                        double ponFirst = _lpnt[rp].ParamOnFirst();
                        double ponSecond = _lpnt[rp].ParamOnSecond();

                        if (((ponFirst >= segModifP1First && ponFirst <= segModifP2First)
                             || (ponFirst <= segModifP1First && ponFirst >= segModifP2First))
                            && ((ponSecond >= segModifP1Second && ponSecond <= segModifP2Second)
                                || (ponSecond <= segModifP1Second && ponSecond >= segModifP2Second)))
                        {
                            _lpnt.Remove(rp);
                            rp--;
                            rnbpts--;
                        }
                    } //-- for(int rp=1; (rp<=rnbpts)&&(rp>=1); rp++)
                }
            }

            //--------------------------------------------------
            //-- Remove some Points ?
            //-- Example : Points which lie in a segment.
            //--------------------------------------------------

            _done = true;
        }
        else
        {
            _done = false;
        }
    }

    protected void Append(in IntersectionSegment2D seg)
    {
        _lseg.Append(seg);
    }

    protected void Append(in IntersectionPoint2D pnt)
    {
        _lpnt.Append(pnt);
    }

    /// <summary>
    /// Inserts a point, keeping the points sorted by their parameter on the first curve, unless an equal point
    /// (parameters within 1e-8, same transitions) is already there.
    /// </summary>
    protected void Insert(in IntersectionPoint2D pnt)
    {
        int n = _lpnt.Length();
        if (n == 0)
        {
            _lpnt.Append(pnt);
        }
        else
        {
            double u = pnt.ParamOnFirst();
            int i = 1;
            int b = n + 1;
            while (i <= n)
            {
                IntersectionPoint2D pnti = _lpnt[i];
                double ui = pnti.ParamOnFirst();
                if (ui >= u)
                {
                    b = i;
                    i = n;
                }

                if (PARAMEQUAL(ui, u))
                {
                    if (PARAMEQUAL((pnt.ParamOnSecond()), (pnti.ParamOnSecond())))
                    {
                        if ((TransitionEqual(pnt.TransitionOfFirst(), pnti.TransitionOfFirst()))
                            && (TransitionEqual(pnt.TransitionOfSecond(), pnti.TransitionOfSecond())))
                        {
                            b = 0;
                            i = n;
                        }
                    }
                }

                //----------------------------------------------------------------
                i++;
            }

            if (b > n)
            {
                _lpnt.Append(pnt);
            }
            else if (b > 0)
            {
                _lpnt.InsertBefore(b, pnt);
            }
        }
    }

    /// <summary>Clears the results if they were computed, and marks the object not done.</summary>
    protected void ResetFields()
    {
        if (_done)
        {
            _lseg.Clear();
            _lpnt.Clear();
            _done = false;
        }
    }

    protected bool ReversedParameters() => (_reverse);

    // C++ macro PARAMEQUAL
    private static bool PARAMEQUAL(double a, double b) => Math.Abs((a) - (b)) < (1e-8);

    private static bool TransitionEqual(in Transition t1, in Transition t2)
    {
        if (t1.PositionOnCurve() == t2.PositionOnCurve())
        {
            if (t1.TransitionType() == t2.TransitionType())
            {
                if (t1.TransitionType() == TypeTrans.Touch)
                {
                    if (t1.IsTangent == t2.IsTangent)
                    {
                        if (t1.Situation() == t2.Situation())
                        {
                            if (t1.IsOpposite == t2.IsOpposite)
                            {
                                return (true);
                            }
                        }
                    }
                }
                else
                {
                    return (true);
                }
            }
        }

        return (false);
    }

    private static void InternalVerifyPosition(ref Transition t1,
                                               ref Transition t2,
                                               double pParamOnFirst,
                                               double pParamOnSecond,
                                               double firstParam1,
                                               double lastParam1,
                                               double firstParam2,
                                               double lastParam2)
    {
        if (t1.PositionOnCurve() != Position2D.Middle)
        {
            if (!(PARAMEQUAL(pParamOnFirst, firstParam1) || PARAMEQUAL(pParamOnFirst, lastParam1)))
            {
                if ((pParamOnFirst > firstParam1) && (pParamOnFirst < lastParam1))
                {
                    t1.SetPosition(Position2D.Middle);
                }
            }
        }

        if (t2.PositionOnCurve() != Position2D.Middle)
        {
            if (!(PARAMEQUAL(pParamOnSecond, firstParam2) || PARAMEQUAL(pParamOnSecond, lastParam2)))
            {
                if ((pParamOnSecond > firstParam2) && (pParamOnSecond < lastParam2))
                {
                    t2.SetPosition(Position2D.Middle);
                }
            }
        }
    }
}
