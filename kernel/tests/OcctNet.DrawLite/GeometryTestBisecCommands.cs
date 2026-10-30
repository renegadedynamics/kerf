// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using static Occt.GccInt_IType;

namespace OcctNet.DrawLite;

/// <summary>
/// GeometryTest_CurveCommands.cxx's <c>bisec</c>: the bisectors of two 2D lines, circles or points with GccAna (unit
/// 5K1's solvers; Draw's command doesn't use the Bisector package). One solution is bound to the result name, several
/// to <c>name_1</c>, <c>name_2</c>, ...
/// </summary>
internal static class GeometryTestBisecCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "bisec", Bisec);
    }

    private static void Solution(DrawInterpreter theDI, GccInt_Bisec Bis, string name, int i)
    {
        string solname = i == 0 ? name : $"{name}_{i}";

        switch (Bis.ArcType())
        {
            case GccInt_Lin:
                DrawTrSurf.Set(theDI, solname, new Geom2d_Line(Bis.Line()));
                break;
            case GccInt_Cir:
                DrawTrSurf.Set(theDI, solname, new Geom2d_Circle(Bis.Circle()));
                break;
            case GccInt_Ell:
                DrawTrSurf.Set(theDI, solname, new Geom2d_Ellipse(Bis.Ellipse()));
                break;
            case GccInt_Par:
                DrawTrSurf.Set(theDI, solname, new Geom2d_Parabola(Bis.Parabola()));
                break;
            case GccInt_Hpr:
                DrawTrSurf.Set(theDI, solname, new Geom2d_Hyperbola(Bis.Hyperbola()));
                break;
            case GccInt_Pnt:
                DrawTrSurf.Set(theDI, solname, Bis.Point());
                break;
        }
    }

    /// <summary>
    /// bisec result line/circle/point line/circle/point. As in OCCT, a curve that is neither a line nor a circle with
    /// a point, or a first curve with a second argument that is neither, reports no solution (the latter after its
    /// message) instead of failing.
    /// </summary>
    private static int Bisec(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 4)
        {
            return 1;
        }

        Geom2d_Curve? C1 = DrawTrSurf.GetCurve2d(theDI, a[2]);
        Geom2d_Curve? C2 = DrawTrSurf.GetCurve2d(theDI, a[3]);
        gp_Pnt2d P1 = new(), P2 = new();
        bool ip1 = DrawTrSurf.GetPoint2d(theDI, a[2], ref P1);
        bool ip2 = DrawTrSurf.GetPoint2d(theDI, a[3], ref P2);
        int i, Compt = 0;
        int NbSol = 0;

        if (C1 is not null)
        {
            Type Type1 = C1.GetType();
            if (C2 is not null)
            {
                Type Type2 = C2.GetType();
                if (Type1 == typeof(Geom2d_Line) && Type2 == typeof(Geom2d_Line))
                {
                    GccAna_Lin2dBisec Bis = new(((Geom2d_Line)C1).Lin2d(), ((Geom2d_Line)C2).Lin2d());
                    if (Bis.IsDone())
                    {
                        NbSol = Bis.NbSolutions();
                        for (i = 1; i <= NbSol; i++)
                        {
                            DrawTrSurf.Set(theDI, $"{a[1]}_{i}", new Geom2d_Line(Bis.ThisSolution(i)));
                        }
                    }
                    else
                    {
                        di.Append(" Bisec has failed !!\n");
                        return 1;
                    }
                }
                else if (Type1 == typeof(Geom2d_Line) && Type2 == typeof(Geom2d_Circle))
                {
                    GccAna_CircLin2dBisec Bis = new(((Geom2d_Circle)C2).Circ2d(), ((Geom2d_Line)C1).Lin2d());
                    if (Bis.IsDone())
                    {
                        NbSol = Bis.NbSolutions();
                        if (NbSol >= 2)
                        {
                            Compt = 1;
                        }
                        for (i = 1; i <= NbSol; i++)
                        {
                            Solution(theDI, Bis.ThisSolution(i), a[1], Compt);
                            Compt++;
                        }
                    }
                    else
                    {
                        di.Append(" Bisec has failed !!\n");
                        return 1;
                    }
                }
                else if (Type2 == typeof(Geom2d_Line) && Type1 == typeof(Geom2d_Circle))
                {
                    GccAna_CircLin2dBisec Bis = new(((Geom2d_Circle)C1).Circ2d(), ((Geom2d_Line)C2).Lin2d());
                    if (Bis.IsDone())
                    {
                        NbSol = Bis.NbSolutions();
                        if (NbSol >= 2)
                        {
                            Compt = 1;
                        }
                        for (i = 1; i <= NbSol; i++)
                        {
                            Solution(theDI, Bis.ThisSolution(i), a[1], Compt);
                            Compt++;
                        }
                    }
                    else
                    {
                        di.Append(" Bisec has failed !!\n");
                        return 1;
                    }
                }
                else if (Type2 == typeof(Geom2d_Circle) && Type1 == typeof(Geom2d_Circle))
                {
                    GccAna_Circ2dBisec Bis = new(((Geom2d_Circle)C1).Circ2d(), ((Geom2d_Circle)C2).Circ2d());
                    if (Bis.IsDone())
                    {
                        NbSol = Bis.NbSolutions();
                        if (NbSol >= 2)
                        {
                            Compt = 1;
                        }
                        for (i = 1; i <= NbSol; i++)
                        {
                            Solution(theDI, Bis.ThisSolution(i), a[1], Compt);
                            Compt++;
                        }
                    }
                    else
                    {
                        di.Append(" Bisec has failed !!\n");
                        return 1;
                    }
                }
                else
                {
                    di.Append(" args must be line/circle/point line/circle/point\n");
                    return 1;
                }
            }
            else if (ip2)
            {
                if (Type1 == typeof(Geom2d_Circle))
                {
                    GccAna_CircPnt2dBisec Bis = new(((Geom2d_Circle)C1).Circ2d(), P2);
                    if (Bis.IsDone())
                    {
                        NbSol = Bis.NbSolutions();
                        if (NbSol >= 2)
                        {
                            Compt = 1;
                        }
                        for (i = 1; i <= NbSol; i++)
                        {
                            Solution(theDI, Bis.ThisSolution(i), a[1], Compt);
                            Compt++;
                        }
                    }
                    else
                    {
                        di.Append(" Bisec has failed !!\n");
                        return 1;
                    }
                }
                else if (Type1 == typeof(Geom2d_Line))
                {
                    GccAna_LinPnt2dBisec Bis = new(((Geom2d_Line)C1).Lin2d(), P2);
                    if (Bis.IsDone())
                    {
                        NbSol = 1;
                        Solution(theDI, Bis.ThisSolution(), a[1], 0);
                    }
                    else
                    {
                        di.Append(" Bisec has failed !!\n");
                        return 1;
                    }
                }
            }
            else
            {
                // OCCT: no return here, so "There is 0 Solution." follows.
                di.Append(" the second arg must be line/circle/point \n");
            }
        }
        else if (ip1)
        {
            if (C2 is not null)
            {
                Type Type2 = C2.GetType();
                if (Type2 == typeof(Geom2d_Circle))
                {
                    GccAna_CircPnt2dBisec Bis = new(((Geom2d_Circle)C2).Circ2d(), P1);
                    if (Bis.IsDone())
                    {
                        NbSol = Bis.NbSolutions();
                        if (NbSol >= 2)
                        {
                            Compt = 1;
                        }
                        for (i = 1; i <= Bis.NbSolutions(); i++)
                        {
                            Solution(theDI, Bis.ThisSolution(i), a[1], Compt);
                            Compt++;
                        }
                    }
                    else
                    {
                        di.Append(" Bisec has failed !!\n");
                        return 1;
                    }
                }
                else if (Type2 == typeof(Geom2d_Line))
                {
                    GccAna_LinPnt2dBisec Bis = new(((Geom2d_Line)C2).Lin2d(), P1);
                    if (Bis.IsDone())
                    {
                        NbSol = 1;
                        Solution(theDI, Bis.ThisSolution(), a[1], 0);
                    }
                    else
                    {
                        di.Append(" Bisec has failed !!\n");
                        return 1;
                    }
                }
            }
            else if (ip2)
            {
                GccAna_Pnt2dBisec Bis = new(P1, P2);
                if (Bis.HasSolution())
                {
                    NbSol = 1;
                    DrawTrSurf.Set(theDI, a[1], new Geom2d_Line(Bis.ThisSolution()));
                }
                else
                {
                    di.Append(" Bisec has failed !!\n");
                    return 1;
                }
            }
            else
            {
                di.Append(" the second arg must be line/circle/point \n");
                return 1;
            }
        }
        else
        {
            di.Append(" args must be line/circle/point line/circle/point\n");
            return 1;
        }

        if (NbSol >= 2)
        {
            di.Append("There are ").Append(NbSol).Append(" Solutions.\n");
        }
        else
        {
            di.Append("There is ").Append(NbSol).Append(" Solution.\n");
        }

        return 0;
    }
}
