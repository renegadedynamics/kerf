// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntRes2d/IntRes2d_Intersection.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntRes2d/IntRes2d_Intersection.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntRes2d/IntRes2d_Intersection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.IntRes2d_Position;
using static Occt.IntRes2d_TypeTrans;

namespace Occt;

/// <summary>
/// The root class of all the intersections between two 2D curves: the results (points and segments) of the
/// intersection algorithms.
/// </summary>
/// <remarks>
/// An abstract class: OCCT's constructors are protected, and only the algorithms derive from it. OCCT's protected
/// copy constructor is kept (it copies the result sequences); its default assignment operator is
/// <see cref="SetValues"/> or <see cref="Assign"/> in ported code.
/// </remarks>
public abstract class IntRes2d_Intersection
{
    protected NCollection_Sequence<IntRes2d_IntersectionPoint> lpnt = new();
    protected NCollection_Sequence<IntRes2d_IntersectionSegment> lseg = new();
    protected bool done;
    protected bool reverse;

    /// <summary>Empty constructor.</summary>
    protected IntRes2d_Intersection()
    {
        done = reverse = false;
    }

    /// <summary>Copies the points and segments of <paramref name="Other"/>; the new object is not done.</summary>
    protected IntRes2d_Intersection(IntRes2d_Intersection Other)
    {
        done = reverse = false;
        lpnt = new NCollection_Sequence<IntRes2d_IntersectionPoint>(Other.lpnt);
        lseg = new NCollection_Sequence<IntRes2d_IntersectionSegment>(Other.lseg);
    }

    /// <summary>C++'s default assignment operator: copies the results and both flags.</summary>
    protected void Assign(IntRes2d_Intersection theOther)
    {
        lpnt.Assign(theOther.lpnt);
        lseg.Assign(theOther.lseg);
        done = theOther.done;
        reverse = theOther.reverse;
    }

    /// <summary>True when the computation was successful.</summary>
    public bool IsDone() => done;

    /// <summary>True if there is no intersection between the given arguments.</summary>
    /// <exception cref="StdFail_NotDone"><see cref="IsDone"/> is false.</exception>
    public bool IsEmpty()
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }

        return ((lpnt.Length() == 0) && (lseg.Length() == 0));
    }

    /// <summary>The number of intersection points between the two curves.</summary>
    /// <exception cref="StdFail_NotDone"><see cref="IsDone"/> is false.</exception>
    public int NbPoints()
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }

        return lpnt.Length();
    }

    /// <summary>The intersection point of range <paramref name="N"/> (1 to <see cref="NbPoints"/>).</summary>
    /// <exception cref="StdFail_NotDone"><see cref="IsDone"/> is false.</exception>
    /// <exception cref="Standard_OutOfRange"><paramref name="N"/> is out of range.</exception>
    public IntRes2d_IntersectionPoint Point(int N)
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }

        return lpnt[N];
    }

    /// <summary>The number of intersection segments between the two curves.</summary>
    /// <exception cref="StdFail_NotDone"><see cref="IsDone"/> is false.</exception>
    public int NbSegments()
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }

        return lseg.Length();
    }

    /// <summary>The intersection segment of range <paramref name="N"/> (1 to <see cref="NbSegments"/>).</summary>
    /// <exception cref="StdFail_NotDone"><see cref="IsDone"/> is false.</exception>
    /// <exception cref="Standard_OutOfRange"><paramref name="N"/> is out of range.</exception>
    public IntRes2d_IntersectionSegment Segment(int N)
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }

        return lseg[N];
    }

    /// <summary>Whether the next computation reports its parameters and transitions with the curves swapped.</summary>
    public void SetReversedParameters(bool flag)
    {
        reverse = flag;
    }

    /// <summary>Copies the results of <paramref name="Other"/> if it is done; otherwise this becomes not done.</summary>
    protected void SetValues(IntRes2d_Intersection Other)
    {
        int i;
        if (Other.done)
        {
            lseg.Clear();
            lpnt.Clear();
            int N = Other.lpnt.Length();
            for (i = 1; i <= N; i++)
            {
                lpnt.Append(Other.lpnt[i]);
            }

            N = Other.lseg.Length();
            for (i = 1; i <= N; i++)
            {
                lseg.Append(Other.lseg[i]);
            }

            //-----------------------
            // lpnt=Other.lpnt;  Pose des problemes
            // lseg=Other.lseg;  pour des objets composites
            //-----------------------
            done = true;
        }
        else
        {
            done = false;
        }
    }

    /// <summary>
    /// Merges the results of another intersection, for composite curves only. The parameters are the bounds of the
    /// composite curves; points strictly inside them get the position Middle, and segments that meet end to end are
    /// joined.
    /// </summary>
    protected void Append(IntRes2d_Intersection Other, double FirstParam1, double LastParam1, double FirstParam2,
                          double LastParam2)
    {
        if (Other.done)
        {
            //-- Verification of the Position of the IntersectionPoints
            int n = Other.lpnt.Length();
            int i;
            for (i = 1; i <= n; i++)
            {
                IntRes2d_IntersectionPoint P = Other.lpnt[i];
                double PParamOnFirst = P.ParamOnFirst();
                double PParamOnSecond = P.ParamOnSecond();
                IntRes2d_Transition T1 = P.TransitionOfFirst();
                IntRes2d_Transition T2 = P.TransitionOfSecond();
                gp_Pnt2d Pt = P.Value();

                InternalVerifyPosition(ref T1,
                                       ref T2,
                                       PParamOnFirst,
                                       PParamOnSecond,
                                       FirstParam1,
                                       LastParam1,
                                       FirstParam2,
                                       LastParam2);

                this.Insert(new IntRes2d_IntersectionPoint(Pt, PParamOnFirst, PParamOnSecond, T1, T2, false));
            }

            //--------------------------------------------------
            //-- IntersectionSegment
            //-- (we assume that a composite curve is always bounded)
            //-- (a segment has always a FirstPoint and a LastPoint)
            //--------------------------------------------------
            n = Other.lseg.Length();
            double SegModif_P1First = 0, SegModif_P1Second = 0;
            double SegModif_P2First = 0, SegModif_P2Second = 0;

            for (i = 1; i <= n; i++)
            {
                IntRes2d_IntersectionPoint P1 = Other.lseg[i].FirstPoint();

                double P1PParamOnFirst = P1.ParamOnFirst();
                double P1PParamOnSecond = P1.ParamOnSecond();
                IntRes2d_Transition P1T1 = P1.TransitionOfFirst();
                IntRes2d_Transition P1T2 = P1.TransitionOfSecond();
                gp_Pnt2d P1Pt = P1.Value();

                InternalVerifyPosition(ref P1T1,
                                       ref P1T2,
                                       P1PParamOnFirst,
                                       P1PParamOnSecond,
                                       FirstParam1,
                                       LastParam1,
                                       FirstParam2,
                                       LastParam2);

                IntRes2d_IntersectionPoint P2 = Other.lseg[i].LastPoint();

                double P2PParamOnFirst = P2.ParamOnFirst();
                double P2PParamOnSecond = P2.ParamOnSecond();
                IntRes2d_Transition P2T1 = P2.TransitionOfFirst();
                IntRes2d_Transition P2T2 = P2.TransitionOfSecond();
                gp_Pnt2d P2Pt = P2.Value();

                bool Opposite = Other.lseg[i].IsOpposite();

                InternalVerifyPosition(ref P2T1,
                                       ref P2T2,
                                       P2PParamOnFirst,
                                       P2PParamOnSecond,
                                       FirstParam1,
                                       LastParam1,
                                       FirstParam2,
                                       LastParam2);

                //-- Loop on the previous segments
                //--
                int an = lseg.Length();
                bool NotYetModified = true;

                for (int j = 1; (j <= an) && (NotYetModified); j++)
                {
                    IntRes2d_IntersectionPoint AnP1 = lseg[j].FirstPoint();
                    double AnP1PParamOnFirst = AnP1.ParamOnFirst();
                    double AnP1PParamOnSecond = AnP1.ParamOnSecond();

                    IntRes2d_IntersectionPoint AnP2 = lseg[j].LastPoint();
                    double AnP2PParamOnFirst = AnP2.ParamOnFirst();
                    double AnP2PParamOnSecond = AnP2.ParamOnSecond();

                    if (Opposite == lseg[j].IsOpposite())
                    {
                        //---------------------------------------------------------------
                        //--    AnP1---------AnP2
                        //--                  P1-------------P2
                        //--
                        if (PARAMEQUAL(P1PParamOnFirst, AnP2PParamOnFirst)
                            && PARAMEQUAL(P1PParamOnSecond, AnP2PParamOnSecond))
                        {
                            NotYetModified = false;
                            lseg[j] = new IntRes2d_IntersectionSegment(AnP1, P2, Opposite, false);
                            SegModif_P1First = AnP1PParamOnFirst;
                            SegModif_P1Second = AnP1PParamOnSecond;
                            SegModif_P2First = P2PParamOnFirst;
                            SegModif_P2Second = P2PParamOnSecond;
                        }
                        //---------------------------------------------------------------
                        //--                                AnP1---------AnP2
                        //--                  P1-------------P2
                        //--
                        else if (PARAMEQUAL(P2PParamOnFirst, AnP1PParamOnFirst)
                                 && PARAMEQUAL(P2PParamOnSecond, AnP1PParamOnSecond))
                        {
                            NotYetModified = false;
                            lseg[j] = new IntRes2d_IntersectionSegment(P1, AnP2, Opposite, false);
                            SegModif_P1First = P1PParamOnFirst;
                            SegModif_P1Second = P1PParamOnSecond;
                            SegModif_P2First = AnP2PParamOnFirst;
                            SegModif_P2Second = AnP2PParamOnSecond;
                        }

                        //---------------------------------------------------------------
                        //--    AnP2---------AnP1
                        //--                  P1-------------P2
                        //--
                        if (PARAMEQUAL(P1PParamOnFirst, AnP1PParamOnFirst)
                            && PARAMEQUAL(P1PParamOnSecond, AnP1PParamOnSecond))
                        {
                            NotYetModified = false;
                            lseg[j] = new IntRes2d_IntersectionSegment(AnP2, P2, Opposite, false);
                            SegModif_P1First = P2PParamOnFirst;
                            SegModif_P1Second = P2PParamOnSecond;
                            SegModif_P2First = AnP2PParamOnFirst;
                            SegModif_P2Second = AnP2PParamOnSecond;
                        }
                        //---------------------------------------------------------------
                        //--                                AnP2---------AnP1
                        //--                  P1-------------P2
                        //--
                        else if (PARAMEQUAL(P2PParamOnFirst, AnP2PParamOnFirst)
                                 && PARAMEQUAL(P2PParamOnSecond, AnP2PParamOnSecond))
                        {
                            NotYetModified = false;
                            lseg[j] = new IntRes2d_IntersectionSegment(P1, AnP1, Opposite, false);
                            SegModif_P1First = P1PParamOnFirst;
                            SegModif_P1Second = P1PParamOnSecond;
                            SegModif_P2First = AnP1PParamOnFirst;
                            SegModif_P2Second = AnP1PParamOnSecond;
                        }
                    }
                }

                if (NotYetModified)
                {
                    this.Append(new IntRes2d_IntersectionSegment(
                        new IntRes2d_IntersectionPoint(P1Pt, P1PParamOnFirst, P1PParamOnSecond, P1T1, P1T2, false),
                        new IntRes2d_IntersectionPoint(P2Pt, P2PParamOnFirst, P2PParamOnSecond, P2T1, P2T2, false),
                        Opposite,
                        false));
                } //-- if(NotYetModified)
                else
                {
                    //--------------------------------------------------------------
                    //-- Are some Existing Points in this segment ?
                    //--------------------------------------------------------------
                    int rnbpts = lpnt.Length();
                    for (int rp = 1; (rp <= rnbpts) && (rp >= 1); rp++)
                    {
                        double PonFirst = lpnt[rp].ParamOnFirst();
                        double PonSecond = lpnt[rp].ParamOnSecond();

                        if (((PonFirst >= SegModif_P1First && PonFirst <= SegModif_P2First)
                             || (PonFirst <= SegModif_P1First && PonFirst >= SegModif_P2First))
                            && ((PonSecond >= SegModif_P1Second && PonSecond <= SegModif_P2Second)
                                || (PonSecond <= SegModif_P1Second && PonSecond >= SegModif_P2Second)))
                        {
                            lpnt.Remove(rp);
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

            done = true;
        }
        else
        {
            done = false;
        }
    }

    protected void Append(in IntRes2d_IntersectionSegment Seg)
    {
        lseg.Append(Seg);
    }

    protected void Append(in IntRes2d_IntersectionPoint Pnt)
    {
        lpnt.Append(Pnt);
    }

    /// <summary>
    /// Inserts a point, keeping the points sorted by their parameter on the first curve, unless an equal point
    /// (parameters within 1e-8, same transitions) is already there.
    /// </summary>
    protected void Insert(in IntRes2d_IntersectionPoint Pnt)
    {
        int n = lpnt.Length();
        if (n == 0)
        {
            lpnt.Append(Pnt);
        }
        else
        {
            double u = Pnt.ParamOnFirst();
            int i = 1;
            int b = n + 1;
            while (i <= n)
            {
                IntRes2d_IntersectionPoint Pnti = lpnt[i];
                double ui = Pnti.ParamOnFirst();
                if (ui >= u)
                {
                    b = i;
                    i = n;
                }

                if (PARAMEQUAL(ui, u))
                {
                    if (PARAMEQUAL((Pnt.ParamOnSecond()), (Pnti.ParamOnSecond())))
                    {
                        if ((TransitionEqual(Pnt.TransitionOfFirst(), Pnti.TransitionOfFirst()))
                            && (TransitionEqual(Pnt.TransitionOfSecond(), Pnti.TransitionOfSecond())))
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
                lpnt.Append(Pnt);
            }
            else if (b > 0)
            {
                lpnt.InsertBefore(b, Pnt);
            }
        }
    }

    /// <summary>Clears the results if they were computed, and marks the object not done.</summary>
    protected void ResetFields()
    {
        if (done)
        {
            lseg.Clear();
            lpnt.Clear();
            done = false;
        }
    }

    protected bool ReversedParameters() => (reverse);

    // C++ macro PARAMEQUAL
    private static bool PARAMEQUAL(double a, double b) => Math.Abs((a) - (b)) < (1e-8);

    private static bool TransitionEqual(in IntRes2d_Transition T1, in IntRes2d_Transition T2)
    {
        if (T1.PositionOnCurve() == T2.PositionOnCurve())
        {
            if (T1.TransitionType() == T2.TransitionType())
            {
                if (T1.TransitionType() == IntRes2d_Touch)
                {
                    if (T1.IsTangent() == T2.IsTangent())
                    {
                        if (T1.Situation() == T2.Situation())
                        {
                            if (T1.IsOpposite() == T2.IsOpposite())
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

    private static void InternalVerifyPosition(ref IntRes2d_Transition T1,
                                               ref IntRes2d_Transition T2,
                                               double PParamOnFirst,
                                               double PParamOnSecond,
                                               double FirstParam1,
                                               double LastParam1,
                                               double FirstParam2,
                                               double LastParam2)
    {
        if (T1.PositionOnCurve() != IntRes2d_Middle)
        {
            if (!(PARAMEQUAL(PParamOnFirst, FirstParam1) || PARAMEQUAL(PParamOnFirst, LastParam1)))
            {
                if ((PParamOnFirst > FirstParam1) && (PParamOnFirst < LastParam1))
                {
                    T1.SetPosition(IntRes2d_Middle);
                }
            }
        }

        if (T2.PositionOnCurve() != IntRes2d_Middle)
        {
            if (!(PARAMEQUAL(PParamOnSecond, FirstParam2) || PARAMEQUAL(PParamOnSecond, LastParam2)))
            {
                if ((PParamOnSecond > FirstParam2) && (PParamOnSecond < LastParam2))
                {
                    T2.SetPosition(IntRes2d_Middle);
                }
            }
        }
    }
}
