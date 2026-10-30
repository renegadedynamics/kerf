// Port of: src/ModelingAlgorithms/TKGeomAlgo/Hatch/Hatch_Hatcher.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Hatch/Hatch_Hatcher.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Hatch/Hatch_Hatcher.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Hatch_LineForm;

namespace Occt;

/// <summary>
/// Computes cross hatchings in a 2D plane, mainly for display: lines are stored (<c>AddLine</c>, <c>AddXLine</c>,
/// <c>AddYLine</c>), trimmed by a boundary whose inside is on its left (<c>Trim</c>), and read back as parameter
/// intervals on each line (<c>NbIntervals</c>, <c>Start</c>, <c>End</c>). An interval may start at RealFirst() and
/// end at RealLast().
/// </summary>
/// <remarks>
/// In the oriented mode the orientation of the trimming curves is considered (hatchings are kept on their left, and
/// may be infinite); in the unoriented mode the hatchings are always finite. A working object (decision 21).
/// </remarks>
public sealed class Hatch_Hatcher
{
    private double myToler;
    private readonly NCollection_Sequence<Hatch_Line> myLines = new();
    private readonly bool myOrient;

    /// <summary>An empty hatcher; <paramref name="Tol"/> is the tolerance for intersections.</summary>
    public Hatch_Hatcher(double Tol, bool Oriented = true)
    {
        myToler = Tol;
        myOrient = Oriented;
    }

    /// <summary>Sets the tolerance for intersections.</summary>
    public void Tolerance(double Tol)
    {
        myToler = Tol;
    }

    /// <summary>The tolerance for intersections.</summary>
    public double Tolerance() => myToler;

    /// <summary>Adds a line <paramref name="L"/> to be trimmed; its form <paramref name="T"/> is only kept for information.</summary>
    public void AddLine(in gp_Lin2d L, Hatch_LineForm T = Hatch_ANYLINE)
    {
        Hatch_Line HL = new(L, T);
        myLines.Append(HL);
    }

    /// <summary>
    /// Adds an infinite line of direction <paramref name="D"/> at the (signed) distance <paramref name="Dist"/> from
    /// the origin: the points P with OP · V = Dist, V perpendicular to D in the direct sense; P's parameter is OP · D.
    /// </summary>
    public void AddLine(in gp_Dir2d D, double Dist)
    {
        double X = D.X();
        double Y = D.Y();
        gp_Pnt2d O = new(-Y * Dist, X * Dist);
        gp_Lin2d L = new(O, D);
        AddLine(L, Hatch_ANYLINE);
    }

    /// <summary>Adds an infinite line parallel to the Y axis at the abscissa <paramref name="X"/>.</summary>
    public void AddXLine(double X)
    {
        gp_Pnt2d O = new(X, 0);
        gp_Dir2d D = new(gp_Dir2d.D.Y);
        gp_Lin2d L = new(O, D);
        AddLine(L, Hatch_XLINE);
    }

    /// <summary>Adds an infinite line parallel to the X axis at the ordinate <paramref name="Y"/>.</summary>
    public void AddYLine(double Y)
    {
        gp_Pnt2d O = new(0, Y);
        gp_Dir2d D = new(gp_Dir2d.D.X);
        gp_Lin2d L = new(O, D);
        AddLine(L, Hatch_YLINE);
    }

    /// <summary>Trims the lines at their intersections with <paramref name="L"/>.</summary>
    public void Trim(in gp_Lin2d L, int Index = 0)
    {
        Trim(L, RealFirst(), RealLast(), Index);
    }

    /// <summary>Trims the lines at their intersections with <paramref name="L"/> in its parameter range [Start, End].</summary>
    public void Trim(in gp_Lin2d L, double Start, double End, int Index = 0)
    {
        IntAna2d_IntPoint Pinter;
        IntAna2d_AnaIntersection Inters = new();
        int iLine;
        for (iLine = 1; iLine <= myLines.Length(); iLine++)
        {
            Inters.Perform(myLines[iLine].myLin, L);
            if (Inters.IsDone())
            {
                if (!Inters.IdenticalElements() && !Inters.ParallelElements())
                {
                    // we have got something
                    Pinter = Inters.Point(1);
                    double linePar = Pinter.ParamOnSecond();
                    if (linePar - Start < -myToler)
                    {
                        continue;
                    }
                    if (linePar - End > myToler)
                    {
                        continue;
                    }
                    double norm = L.Direction() ^ myLines[iLine].myLin.Direction();
                    if (linePar - Start < myToler)
                    {
                        // on the limit of the trimming segment
                        // accept if the other extremity is on the left
                        if (norm < 0)
                        {
                            continue;
                        }
                    }
                    if (linePar - End > -myToler)
                    {
                        // on the limit of the trimming segment
                        // accept if the other extremity is on the left
                        if (norm > 0)
                        {
                            continue;
                        }
                    }
                    // insert the parameter
                    myLines[iLine].AddIntersection(Pinter.ParamOnFirst(),
                                                   norm > 0,
                                                   Index,
                                                   Pinter.ParamOnSecond(),
                                                   myToler);
                }
            }
        }
    }

    /// <summary>Trims the lines at their intersections with the oriented segment from <paramref name="P1"/> to <paramref name="P2"/>.</summary>
    public void Trim(in gp_Pnt2d P1, in gp_Pnt2d P2, int Index = 0)
    {
        gp_Vec2d V = new(P1, P2);
        if (Math.Abs(V.X()) > .9 * RealLast())
        {
            V.Multiply(1 / V.X());
        }
        else if (Math.Abs(V.Y()) > .9 * RealLast())
        {
            V.Multiply(1 / V.Y());
        }
        if (V.Magnitude() > myToler)
        {
            gp_Dir2d D = new(V);
            gp_Lin2d L = new(P1, D);
            Trim(L, 0, P1.Distance(P2), Index);
        }
    }

    /// <summary>The total number of intervals on all the lines.</summary>
    public int NbIntervals()
    {
        int i, nb = 0;
        for (i = 1; i <= myLines.Length(); i++)
        {
            nb += NbIntervals(i);
        }
        return nb;
    }

    /// <summary>The number of lines.</summary>
    public int NbLines() => myLines.Length();

    /// <summary>The line of index <paramref name="I"/>.</summary>
    public gp_Lin2d Line(int I) => myLines[I].myLin;

    /// <summary>The form of the line of index <paramref name="I"/>.</summary>
    public Hatch_LineForm LineForm(int I) => myLines[I].myForm;

    /// <summary>True if the line of index <paramref name="I"/> has a constant X.</summary>
    public bool IsXLine(int I) => LineForm(I) == Hatch_XLINE;

    /// <summary>True if the line of index <paramref name="I"/> has a constant Y.</summary>
    public bool IsYLine(int I) => LineForm(I) == Hatch_YLINE;

    /// <summary>The X or Y coordinate of the line of index <paramref name="I"/>, an X or a Y line.</summary>
    /// <exception cref="Standard_OutOfRange">The line is neither an X nor a Y line.</exception>
    public double Coordinate(int I)
    {
        switch (myLines[I].myForm)
        {
            case Hatch_XLINE:
                return myLines[I].myLin.Location().X();

            case Hatch_YLINE:
                return myLines[I].myLin.Location().Y();

            case Hatch_ANYLINE:
                throw new Standard_OutOfRange("Hatcher : not an X or Y line");
        }

        return 0.0;
    }

    /// <summary>The number of intervals on the line of index <paramref name="I"/>.</summary>
    public int NbIntervals(int I)
    {
        int l = myLines[I].myInters.Length();
        if (l == 0)
        {
            l = myOrient ? 1 : 0;
        }
        else
        {
            l = l / 2;
            if (myOrient)
            {
                if (!myLines[I].myInters[1].myStart)
                {
                    l++;
                }
            }
        }
        return l;
    }

    /// <summary>The first parameter of the interval <paramref name="J"/> on the line <paramref name="I"/>.</summary>
    public double Start(int I, int J)
    {
        if (myLines[I].myInters.IsEmpty())
        {
            if (J != 1 || !myOrient)
            {
                throw new Standard_OutOfRange();
            }
            return RealFirst();
        }
        else
        {
            int jj = 2 * J - 1;
            if (!myLines[I].myInters[1].myStart && myOrient)
            {
                jj--;
            }
            if (jj == 0)
            {
                return RealFirst();
            }
            return myLines[I].myInters[jj].myPar1;
        }
    }

    /// <summary>
    /// The index of the trimming element and the parameter on it at the start of the interval <paramref name="J"/> on
    /// the line <paramref name="I"/> (0 and 0 at an infinite start).
    /// </summary>
    public void StartIndex(int I, int J, out int Index, out double Par2)
    {
        if (myLines[I].myInters.IsEmpty())
        {
            if (J != 1) // OCCT: doesn't check myOrient, as Start and End do (occt-issues.md)
            {
                throw new Standard_OutOfRange();
            }
            Index = 0;
            Par2 = 0;
        }
        else
        {
            int jj = 2 * J - 1;
            if (!myLines[I].myInters[1].myStart && myOrient)
            {
                jj--;
            }
            if (jj == 0)
            {
                Index = 0;
                Par2 = 0;
            }
            else
            {
                Index = myLines[I].myInters[jj].myIndex;
                Par2 = myLines[I].myInters[jj].myPar2;
            }
        }
    }

    /// <summary>The last parameter of the interval <paramref name="J"/> on the line <paramref name="I"/>.</summary>
    public double End(int I, int J)
    {
        if (myLines[I].myInters.IsEmpty())
        {
            if (J != 1 || !myOrient)
            {
                throw new Standard_OutOfRange();
            }
            return RealLast();
        }
        else
        {
            int jj = 2 * J;
            if (!myLines[I].myInters[1].myStart && myOrient)
            {
                jj--;
            }
            if (jj > myLines[I].myInters.Length())
            {
                return RealLast();
            }
            return myLines[I].myInters[jj].myPar1;
        }
    }

    /// <summary>
    /// The index of the trimming element and the parameter on it at the end of the interval <paramref name="J"/> on
    /// the line <paramref name="I"/> (0 and 0 at an infinite end).
    /// </summary>
    public void EndIndex(int I, int J, out int Index, out double Par2)
    {
        if (myLines[I].myInters.IsEmpty())
        {
            if (J != 1) // OCCT: doesn't check myOrient, as Start and End do (occt-issues.md)
            {
                throw new Standard_OutOfRange();
            }
            Index = 0;
            Par2 = 0;
        }
        else
        {
            int jj = 2 * J;
            if (!myLines[I].myInters[1].myStart && myOrient)
            {
                jj--;
            }
            if (jj > myLines[I].myInters.Length())
            {
                Index = 0;
                Par2 = 0;
            }
            else
            {
                Index = myLines[I].myInters[jj].myIndex;
                Par2 = myLines[I].myInters[jj].myPar2;
            }
        }
    }
}
