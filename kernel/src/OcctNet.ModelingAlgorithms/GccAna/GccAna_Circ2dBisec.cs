// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2dBisec.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2dBisec.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The bisecting curves of two 2D circles: the loci of the points equidistant from both. Depending on their relative
/// position, ellipses, hyperbolas, circles or lines.
/// </summary>
/// <remarks>
/// An algorithm object computed by its constructor (decision 117); as in OCCT, each solution is built when it is
/// asked for.
/// </remarks>
public sealed class GccAna_Circ2dBisec
{
    private bool WellDone;
    private int NbrSol;
    private int intersection;
    private bool sameradius;
    private Circle2D circle1;
    private Circle2D circle2;

    /// <summary>Constructs the bisecting curves of Circ1 and Circ2.</summary>
    public GccAna_Circ2dBisec(in Circle2D Circ1, in Circle2D Circ2)
    {
        //=========================================================================
        //  Initialization of fields :                                            +
        //            - circle1  (Circle : first argument)                        +
        //            - circle2  (Line   : second argument)                       +
        //            - intersection (Integer showing the smallest position of    +
        //                            two circles correspondingly to each other)  +
        //            - sameradius   (Boolean showing if the two circles have     +
        //                            the same radius or not)                     +
        //            - NbrSol   (Integer showing the number of solutions)        +
        //            - WellDone (Boolean showing success or failure of the algo) +
        //=========================================================================

        WellDone = false;
        double Tol = global::Kerf.Engine.Foundation.Precision.Confusion();

        double R1 = Circ1.Radius();
        double R2 = Circ2.Radius();
        sameradius = Math.Abs(R1 - R2) <= Tol;
        if (R1 < R2)
        {
            circle1 = Circ2;
            circle2 = Circ1;
            R1 = circle1.Radius();
            R2 = circle2.Radius();
        }
        else
        {
            circle1 = Circ1;
            circle2 = Circ2;
        }
        double dist = circle2.Location().Distance(circle1.Location());
        if (R1 - dist - R2 > Tol)
        {
            intersection = 0;
            NbrSol = 2;
            WellDone = true;
        }
        else if (Math.Abs(R1 - dist - R2) <= Tol)
        {
            intersection = 1;
            if (sameradius)
            {
                NbrSol = 0;
                WellDone = true;
            }
            else
            {
                NbrSol = 2;
                WellDone = true;
            }
        }
        else if ((dist + R2 - R1 > Tol) && (R1 - dist + R2 > Tol))
        {
            intersection = 2;
            if (sameradius)
            {
                NbrSol = 2;
                WellDone = true;
            }
            else
            {
                NbrSol = 3;
                WellDone = true;
            }
        }
        else if (Math.Abs(R1 - dist + R2) <= Tol)
        {
            intersection = 3;
            if (sameradius)
            {
                NbrSol = 2;
                WellDone = true;
            }
            else
            {
                NbrSol = 3;
                WellDone = true;
            }
        }
        else
        {
            intersection = 4;
            if (sameradius)
            {
                NbrSol = 3;
                WellDone = true;
            }
            else
            {
                NbrSol = 4;
                WellDone = true;
            }
        }
    }

    //=========================================================================
    //  Processing.                                                           +
    //  Return the coordinates of centers of circles circle1 and circle2      +
    //  (xcencir1, ycencir1, xcencir2, ycencir2).                             +
    //  Also return the radiuses of two circles R1 and R2.                    +
    //=========================================================================

    /// <summary>
    /// The solution number Index. As in OCCT, it is null for the index combinations OCCT's cases don't cover.
    /// </summary>
    public GccInt_Bisec ThisSolution(int Index)
    {
        double Tol = 1.0e-14;
        GccInt_Bisec? bissol = null;

        if (!WellDone)
        {
            throw new NotDoneException();
        }
        else if (Index <= 0 || Index > NbrSol)
        {
            throw new OutOfRangeException();
        }
        else
        {
            double xcencir1 = circle1.Location().X();
            double ycencir1 = circle1.Location().Y();
            double xcencir2 = circle2.Location().X();
            double ycencir2 = circle2.Location().Y();
            double dist = circle1.Location().Distance(circle2.Location());

            Point2D pcen = new((xcencir1 + xcencir2) / 2.0, (ycencir1 + ycencir2) / 2.0);
            Direction2D dircen = new(), medcen = new();
            if (dist > Tol)
            {
                dircen.SetCoord(xcencir2 - xcencir1, ycencir2 - ycencir1);
                medcen.SetCoord(ycencir2 - ycencir1, xcencir1 - xcencir2);
            }
            Direction2D dirx = new(Direction2D.D.X);
            Axis2D acenx = new(pcen, dirx);
            Axis2D acencen = new(pcen, dircen);

            double R1 = circle1.Radius();
            double R2 = circle2.Radius();

            if ((NbrSol == 1) && (intersection == 0))
            {
                double R;
                if (Index == 1)
                {
                    R = (R1 + R2) / 2.0;
                }
                else
                {
                    R = (R1 - R2) / 2.0;
                }
                Circle2D C = new(acenx, R);
                bissol = new GccInt_BCirc(C);
            }
            else if ((NbrSol == 2) && (intersection == 1))
            {
                if (Index == 1)
                {
                    Ellipse2D E = new(acencen,
                                       (R1 + R2) / 2.0,
                                       Math.Sqrt((R1 * R1 + R2 * R2 - dist * dist) / 4.0 + R1 * R2 / 2.0));
                    bissol = new GccInt_BElips(E);
                }
                else if (Index == 2)
                {
                    Line2D L = new(circle1.Location(), dircen);
                    bissol = new GccInt_BLine(L);
                }
            }
            else if ((NbrSol == 2) && (intersection == 0))
            {
                if (Index == 1)
                {
                    if (Math.Abs(xcencir2 - xcencir1) < Tol && Math.Abs(ycencir2 - ycencir1) < Tol)
                    {
                        Circle2D C = new(acenx, (R1 + R2) / 2.0);
                        bissol = new GccInt_BCirc(C);
                    }
                    else
                    {
                        Ellipse2D E = new(acencen,
                                           (R1 + R2) / 2.0,
                                           Math.Sqrt((R1 * R1 + R2 * R2 - dist * dist) / 4.0 + R1 * R2 / 2.0));
                        bissol = new GccInt_BElips(E);
                    }
                }
                else if (Index == 2)
                {
                    if (Math.Abs(xcencir2 - xcencir1) < Tol && Math.Abs(ycencir2 - ycencir1) < Tol)
                    {
                        Circle2D C = new(acencen, (R1 - R2) / 2.0);
                        bissol = new GccInt_BCirc(C);
                    }
                    else
                    {
                        Ellipse2D E = new(acencen,
                                           (R1 - R2) / 2.0,
                                           Math.Sqrt((R1 * R1 + R2 * R2 - dist * dist) / 4.0 - R1 * R2 / 2.0));
                        bissol = new GccInt_BElips(E);
                    }
                }
            }
            else if (intersection == 2)
            {
                if (sameradius)
                {
                    if (Index == 1)
                    {
                        Line2D L = new(pcen, medcen);
                        bissol = new GccInt_BLine(L);
                    }
                    else if (Index == 2)
                    {
                        Ellipse2D E = new(acencen, R1, Math.Sqrt(R1 * R1 - dist * dist / 4.0));
                        bissol = new GccInt_BElips(E);
                    }
                }
                else
                {
                    if (Index == 1)
                    {
                        Hyperbola2D H1;
                        H1 = new Hyperbola2D(acencen,
                                           (R1 - R2) / 2.0,
                                           Math.Sqrt(dist * dist - (R1 - R2) * (R1 - R2)) / 2.0);
                        bissol = new GccInt_BHyper(H1);
                    }
                    else if (Index == 2)
                    {
                        Hyperbola2D H1 = new(acencen,
                                           (R1 - R2) / 2.0,
                                           Math.Sqrt(dist * dist - (R1 - R2) * (R1 - R2)) / 2.0);
                        bissol = new GccInt_BHyper(H1.OtherBranch());
                    }
                    else if (Index == 3)
                    {
                        Ellipse2D E = new(acencen,
                                           (R1 + R2) / 2.0,
                                           Math.Sqrt((R1 * R1 + R2 * R2 - dist * dist) / 4.0 + R1 * R2 / 2.0));
                        bissol = new GccInt_BElips(E);
                    }
                }
            }
            else if (intersection == 3)
            {
                if (sameradius)
                {
                    if (Index == 1)
                    {
                        Line2D L = new(pcen, dircen);
                        bissol = new GccInt_BLine(L);
                    }
                    else if (Index == 2)
                    {
                        Line2D L = new(pcen, medcen);
                        bissol = new GccInt_BLine(L);
                    }
                }
                else
                {
                    if (Index == 1)
                    {
                        Line2D L = new(pcen, dircen);
                        bissol = new GccInt_BLine(L);
                    }
                    else if (Index == 2)
                    {
                        Hyperbola2D H1 = new(acencen,
                                           (R1 - R2) / 2.0,
                                           Math.Sqrt(dist * dist - (R1 - R2) * (R1 - R2)) / 2.0);
                        bissol = new GccInt_BHyper(H1);
                    }
                    else if (Index == 3)
                    {
                        Hyperbola2D H1 = new(acencen,
                                           (R1 - R2) / 2.0,
                                           Math.Sqrt(dist * dist - (R1 - R2) * (R1 - R2)) / 2.0);
                        bissol = new GccInt_BHyper(H1.OtherBranch());
                    }
                }
            }
            else if (intersection == 4)
            {
                if (sameradius)
                {
                    if (Index == 1)
                    {
                        Line2D L = new(pcen, medcen);
                        bissol = new GccInt_BLine(L);
                    }
                    else if (Index == 2)
                    {
                        Hyperbola2D H1 = new(acencen, R1, Math.Sqrt(dist * dist - 4 * R1 * R1) / 2.0);
                        bissol = new GccInt_BHyper(H1);
                    }
                    else if (Index == 3)
                    {
                        Hyperbola2D H1 = new(acencen, R1, Math.Sqrt(dist * dist - 4 * R1 * R1) / 2.0);
                        bissol = new GccInt_BHyper(H1.OtherBranch());
                    }
                }
                else
                {
                    if (Index == 1)
                    {
                        Hyperbola2D H1 = new(acencen,
                                           (R1 - R2) / 2.0,
                                           Math.Sqrt(dist * dist - (R1 - R2) * (R1 - R2)) / 2.0);
                        bissol = new GccInt_BHyper(H1);
                    }
                    else if (Index == 2)
                    {
                        Hyperbola2D H1 = new(acencen,
                                           (R1 - R2) / 2.0,
                                           Math.Sqrt(dist * dist - (R1 - R2) * (R1 - R2)) / 2.0);
                        bissol = new GccInt_BHyper(H1.OtherBranch());
                    }
                    else if (Index == 3)
                    {
                        Hyperbola2D H1 = new(acencen,
                                           (R1 + R2) / 2.0,
                                           Math.Sqrt(dist * dist - (R1 + R2) * (R1 + R2)) / 2.0);
                        bissol = new GccInt_BHyper(H1);
                    }
                    else if (Index == 4)
                    {
                        Hyperbola2D H1 = new(acencen,
                                           (R1 + R2) / 2.0,
                                           Math.Sqrt(dist * dist - (R1 + R2) * (R1 + R2)) / 2.0);
                        bissol = new GccInt_BHyper(H1.OtherBranch());
                    }
                }
            }
        }
        return bissol!;
    }

    /// <summary>True if the construction succeeded.</summary>
    public bool IsDone()
    {
        return WellDone;
    }

    /// <summary>The number of bisecting curves.</summary>
    public int NbSolutions()
    {
        if (!WellDone)
        {
            throw new NotDoneException();
        }
        return NbrSol;
    }
}
