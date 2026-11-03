// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Constraints;
using Kerf.Engine.Construction.Constraints.Analytic;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Constraints.ConstraintCurveType;

namespace Kerf.Kernel.DrawLite;

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

    private static void Solution(DrawInterpreter theDI, ConstraintBisec Bis, string name, int i)
    {
        string solname = i == 0 ? name : $"{name}_{i}";

        switch (Bis.ArcType())
        {
            case ConstraintCurveType.Lin:
                DrawTrSurf.Set(theDI, solname, new LineCurve2D(Bis.Line()));
                break;
            case ConstraintCurveType.Cir:
                DrawTrSurf.Set(theDI, solname, new CircleCurve2D(Bis.Circle()));
                break;
            case ConstraintCurveType.Ell:
                DrawTrSurf.Set(theDI, solname, new EllipseCurve2D(Bis.Ellipse()));
                break;
            case ConstraintCurveType.Par:
                DrawTrSurf.Set(theDI, solname, new ParabolaCurve2D(Bis.Parabola()));
                break;
            case ConstraintCurveType.Hpr:
                DrawTrSurf.Set(theDI, solname, new HyperbolaCurve2D(Bis.Hyperbola()));
                break;
            case ConstraintCurveType.Pnt:
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

        Curve2D? C1 = DrawTrSurf.GetCurve2d(theDI, a[2]);
        Curve2D? C2 = DrawTrSurf.GetCurve2d(theDI, a[3]);
        Point2D P1 = new(), P2 = new();
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
                if (Type1 == typeof(LineCurve2D) && Type2 == typeof(LineCurve2D))
                {
                    Lin2dBisec Bis = new(((LineCurve2D)C1).Line, ((LineCurve2D)C2).Line);
                    if (Bis.IsDone)
                    {
                        NbSol = Bis.SolutionCount;
                        for (i = 1; i <= NbSol; i++)
                        {
                            DrawTrSurf.Set(theDI, $"{a[1]}_{i}", new LineCurve2D(Bis.ThisSolution(i)));
                        }
                    }
                    else
                    {
                        di.Append(" Bisec has failed !!\n");
                        return 1;
                    }
                }
                else if (Type1 == typeof(LineCurve2D) && Type2 == typeof(CircleCurve2D))
                {
                    CircLin2dBisec Bis = new(((CircleCurve2D)C2).Circle, ((LineCurve2D)C1).Line);
                    if (Bis.IsDone)
                    {
                        NbSol = Bis.SolutionCount;
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
                else if (Type2 == typeof(LineCurve2D) && Type1 == typeof(CircleCurve2D))
                {
                    CircLin2dBisec Bis = new(((CircleCurve2D)C1).Circle, ((LineCurve2D)C2).Line);
                    if (Bis.IsDone)
                    {
                        NbSol = Bis.SolutionCount;
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
                else if (Type2 == typeof(CircleCurve2D) && Type1 == typeof(CircleCurve2D))
                {
                    Circ2dBisec Bis = new(((CircleCurve2D)C1).Circle, ((CircleCurve2D)C2).Circle);
                    if (Bis.IsDone)
                    {
                        NbSol = Bis.SolutionCount;
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
                if (Type1 == typeof(CircleCurve2D))
                {
                    CircPnt2dBisec Bis = new(((CircleCurve2D)C1).Circle, P2);
                    if (Bis.IsDone)
                    {
                        NbSol = Bis.SolutionCount;
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
                else if (Type1 == typeof(LineCurve2D))
                {
                    LinPnt2dBisec Bis = new(((LineCurve2D)C1).Line, P2);
                    if (Bis.IsDone)
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
                if (Type2 == typeof(CircleCurve2D))
                {
                    CircPnt2dBisec Bis = new(((CircleCurve2D)C2).Circle, P1);
                    if (Bis.IsDone)
                    {
                        NbSol = Bis.SolutionCount;
                        if (NbSol >= 2)
                        {
                            Compt = 1;
                        }
                        for (i = 1; i <= Bis.SolutionCount; i++)
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
                else if (Type2 == typeof(LineCurve2D))
                {
                    LinPnt2dBisec Bis = new(((LineCurve2D)C2).Line, P1);
                    if (Bis.IsDone)
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
                Pnt2dBisec Bis = new(P1, P2);
                if (Bis.HasSolution)
                {
                    NbSol = 1;
                    DrawTrSurf.Set(theDI, a[1], new LineCurve2D(Bis.ThisSolution()));
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
