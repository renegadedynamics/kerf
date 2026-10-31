// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT2d/MAT2d_MiniPath.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT2d/MAT2d_MiniPath.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Computes a path linking all the lines of a set of lines, as a sequence of connexions. The connexions form a
/// tree whose nodes are the lines: the ancestor of a line is the connexion ending on it, its children the
/// connexions starting on it, ordered by MAT2d_Connexion.IsAfter.
/// </summary>
/// <remarks>A working object of MAT2d_Circuit. The maps are keyed by line index and only looked up.</remarks>
public sealed class MAT2d_MiniPath
{
    private readonly NCollection_DataMap<int, NCollection_Sequence<MAT2d_Connexion>> theConnexions = new();
    private readonly NCollection_DataMap<int, MAT2d_Connexion> theFather = new();
    private readonly NCollection_Sequence<MAT2d_Connexion> thePath = new();
    private double theDirection;
    private int indStart;

    public MAT2d_MiniPath()
    {
        theDirection = 1.0;
        indStart = 0;
    }

    /// <summary>Computes the path linking the lines of Figure, starting on the line IndStart. Sense is true if the
    /// circuit turns in the trigonometric sense.</summary>
    public void Perform(NCollection_Sequence<NCollection_Sequence<Geom2d_Geometry>> Figure, int IndStart, bool Sense)
    {
        int i, j;
        int NbLines = Figure.Length();
        NCollection_Array2<MAT2d_Connexion> Connexion = new(1, NbLines, 1, NbLines);

        indStart = IndStart;
        theDirection = 1.0;
        if (Sense)
        {
            theDirection = -1.0;
        }

        // The connexions realising the minimum of distance between the elements of the figure.
        for (i = 1; i < NbLines; i++)
        {
            for (j = i + 1; j <= NbLines; j++)
            {
                Connexion[i, j] = MinimumL1L2(Figure, i, j);
                Connexion[j, i] = Connexion[i, j].Reverse();
            }
        }

        NCollection_Sequence<int> Set1 = new();
        NCollection_Sequence<int> Set2 = new();
        double DistS1S2;
        int IndiceLine1, IndiceLine2;
        int ISuiv = 0, MinOnSet1 = 0, MinOnSet2 = 0;

        // - 0 Set1 starts with the start line, Set2 holds the others.
        Set1.Append(IndStart);
        for (i = 1; i <= NbLines; i++)
        {
            if (i != IndStart)
            {
                Set2.Append(i);
            }
        }

        // - 1 The shortest connexion C between Set1 and Set2.
        // - 2 The line of Set2 realising it moves to Set1.
        // - 3 C is inserted in the connexions.
        // - 4 Back to 1 while Set2 isn't empty.
        while (!Set2.IsEmpty())
        {
            DistS1S2 = RealLast();
            for (i = 1; i <= Set1.Length(); i++)
            {
                IndiceLine1 = Set1.Value(i);
                for (j = 1; j <= Set2.Length(); j++)
                {
                    IndiceLine2 = Set2.Value(j);
                    if (Connexion[IndiceLine1, IndiceLine2].Distance() < DistS1S2)
                    {
                        ISuiv = j;
                        DistS1S2 = Connexion[IndiceLine1, IndiceLine2].Distance();
                        MinOnSet1 = IndiceLine1;
                        MinOnSet2 = IndiceLine2;
                    }
                }
            }
            Set1.Append(Set2.Value(ISuiv));
            Set2.Remove(ISuiv);
            Append(Connexion[MinOnSet1, MinOnSet2]);
        }

        // The path, running over the connexions.
        RunOnConnexions();
    }

    // Inserts a connexion in the tree of connexions (the lines are its nodes, the connexions its branches).
    private void Append(MAT2d_Connexion C)
    {
        MAT2d_Connexion CC;

        if (!theConnexions.IsBound(C.IndexFirstLine()))
        {
            NCollection_Sequence<MAT2d_Connexion> Seq0 = new();
            theConnexions.Bind(C.IndexFirstLine(), Seq0);
            theConnexions[C.IndexFirstLine()].Append(C);
            theFather.Bind(C.IndexSecondLine(), C);
            return;
        }

        NCollection_Sequence<MAT2d_Connexion> Seq = theConnexions[C.IndexFirstLine()];
        int IndexAfter = 0;
        int NbConnexions = Seq.Length();

        for (int i = 1; i <= NbConnexions; i++)
        {
            CC = Seq.Value(i);
            if (CC.IsAfter(C, theDirection))
            {
                IndexAfter = i;
                break;
            }
        }
        // C is inserted before IndexAfter; if 0, no connexion is after C, which is the last.
        if (IndexAfter == 0)
        {
            Seq.Append(C);
        }
        else
        {
            Seq.InsertBefore(IndexAfter, C);
        }
        theFather.Bind(C.IndexSecondLine(), C);
    }

    /// <summary>The sequence of connexions of the path.</summary>
    public NCollection_Sequence<MAT2d_Connexion> Path() => thePath;

    /// <summary>True if a connexion starts on the line Index.</summary>
    public bool IsConnexionsFrom(int i) => theConnexions.IsBound(i);

    /// <summary>The connexions starting on the line Index.</summary>
    public NCollection_Sequence<MAT2d_Connexion> ConnexionsFrom(int i) => theConnexions.ChangeFind(i);

    /// <summary>True if the line Index is the root.</summary>
    public bool IsRoot(int ILine) => ILine == indStart;

    /// <summary>The connexion ending on the line ILine.</summary>
    public MAT2d_Connexion Father(int ILine) => theFather.ChangeFind(ILine);

    /// <summary>Builds the path: an exploration of the tree of connexions and their reverses.</summary>
    public void RunOnConnexions()
    {
        int i;
        MAT2d_Connexion C;
        NCollection_Sequence<MAT2d_Connexion> SC = theConnexions.Find(indStart);

        thePath.Clear();

        for (i = 1; i <= SC.Length(); i++)
        {
            C = SC.Value(i);
            thePath.Append(C);
            ExploSons(thePath, C);
            thePath.Append(C.Reverse());
        }
    }

    private void ExploSons(NCollection_Sequence<MAT2d_Connexion> CResult, MAT2d_Connexion CRef)
    {
        int i;
        int Index = CRef.IndexSecondLine();

        if (!theConnexions.IsBound(Index))
        {
            return;
        }

        NCollection_Sequence<MAT2d_Connexion> SC = theConnexions.Find(Index);
        MAT2d_Connexion CRR = CRef.Reverse();
        MAT2d_Connexion C;

        for (i = 1; i <= SC.Length(); i++)
        {
            C = SC.Value(i);
            if (C.IsAfter(CRR, theDirection))
            {
                CResult.Append(C);
                ExploSons(CResult, C);
                CResult.Append(C.Reverse());
            }
        }

        for (i = 1; i <= SC.Length(); i++)
        {
            C = SC.Value(i);
            if (!C.IsAfter(CRR, theDirection))
            {
                CResult.Append(C);
                ExploSons(CResult, C);
                CResult.Append(C.Reverse());
            }
            else
            {
                break;
            }
        }
    }

    // The connexion realising the minimum of distance between the lines IL1 and IL2 of Figure, oriented from IL1
    // to IL2.
    private static MAT2d_Connexion MinimumL1L2(NCollection_Sequence<NCollection_Sequence<Geom2d_Geometry>> Figure,
        int IL1, int IL2)
    {
        Extrema_POnCurv2d PointOnCurv1, PointOnCurv2;
        int IC1, IC2, IMinC1 = 0, IMinC2 = 0, i;
        double DistL1L2_2, DistP1P2_2;
        double ParameterOnC1 = 0.0, ParameterOnC2 = 0.0;
        NCollection_Sequence<Geom2d_Geometry> L1, L2;
        gp_Pnt2d Point1 = new(), Point2 = new(), P1 = new(), P2 = new();
        Geom2d_Curve? Item1 = null;
        Geom2d_Curve? Item2 = null;

        L1 = Figure.Value(IL1);
        L2 = Figure.Value(IL2);

        DistL1L2_2 = RealLast();

        // The extrema of distance between the components of L1 and L2.
        for (IC1 = 1; IC1 <= L1.Length(); IC1++)
        {
            Type Type1 = L1.Value(IC1).GetType();
            if (Type1 != typeof(Geom2d_CartesianPoint))
            {
                Item1 = (Geom2d_Curve)L1.Value(IC1);
            }
            else
            {
                P1 = ((Geom2d_Point)L1.Value(IC1)).Pnt2d();
            }

            for (IC2 = 1; IC2 <= L2.Length(); IC2++)
            {
                Type Type2 = L2.Value(IC2).GetType();
                if (Type2 != typeof(Geom2d_CartesianPoint))
                {
                    Item2 = (Geom2d_Curve)L2.Value(IC2);
                }
                else
                {
                    P2 = ((Geom2d_Point)L2.Value(IC2)).Pnt2d();
                }

                if (Type1 == typeof(Geom2d_CartesianPoint) && Type2 == typeof(Geom2d_CartesianPoint))
                {
                    DistP1P2_2 = P1.SquareDistance(P2);
                    if (DistP1P2_2 <= DistL1L2_2)
                    {
                        DistL1L2_2 = DistP1P2_2;
                        IMinC1 = IC1;
                        IMinC2 = IC2;
                        Point1 = P1;
                        Point2 = P2;
                        ParameterOnC1 = 0.0;
                        ParameterOnC2 = 0.0;
                    }
                }
                else if (Type1 == typeof(Geom2d_CartesianPoint))
                {
                    Geom2dAdaptor_Curve C2 = new(Item2!);
                    Extrema_ExtPC2d Extremas = new(P1, C2);
                    if (Extremas.IsDone())
                    {
                        for (i = 1; i <= Extremas.NbExt(); i++)
                        {
                            if (Extremas.SquareDistance(i) < DistL1L2_2)
                            {
                                DistL1L2_2 = Extremas.SquareDistance(i);
                                IMinC1 = IC1;
                                IMinC2 = IC2;
                                PointOnCurv2 = Extremas.Point(i);
                                ParameterOnC1 = 0.0;
                                ParameterOnC2 = PointOnCurv2.Parameter();
                                Point1 = P1;
                                Point2 = PointOnCurv2.Value();
                            }
                        }
                    }
                }
                else if (Type2 == typeof(Geom2d_CartesianPoint))
                {
                    Geom2dAdaptor_Curve C1 = new(Item1!);
                    Extrema_ExtPC2d Extremas = new(P2, C1);
                    if (Extremas.IsDone())
                    {
                        for (i = 1; i <= Extremas.NbExt(); i++)
                        {
                            if (Extremas.SquareDistance(i) < DistL1L2_2)
                            {
                                DistL1L2_2 = Extremas.SquareDistance(i);
                                IMinC1 = IC1;
                                IMinC2 = IC2;
                                PointOnCurv1 = Extremas.Point(i);
                                ParameterOnC2 = 0.0;
                                ParameterOnC1 = PointOnCurv1.Parameter();
                                Point1 = PointOnCurv1.Value();
                                Point2 = P2;
                            }
                        }
                    }
                }
                else
                {
                    Geom2dAdaptor_Curve C1 = new(Item1!);
                    Geom2dAdaptor_Curve C2 = new(Item2!);
                    Extrema_ExtCC2d Extremas = new(C1, C2);
                    if (!Extremas.IsParallel() && Extremas.IsDone())
                    {
                        for (i = 1; i <= Extremas.NbExt(); i++)
                        {
                            if (Extremas.SquareDistance(i) < DistL1L2_2)
                            {
                                DistL1L2_2 = Extremas.SquareDistance(i);
                                IMinC1 = IC1;
                                IMinC2 = IC2;
                                Extremas.Points(i, out PointOnCurv1, out PointOnCurv2);
                                ParameterOnC1 = PointOnCurv1.Parameter();
                                ParameterOnC2 = PointOnCurv2.Parameter();
                                Point1 = PointOnCurv1.Value();
                                Point2 = PointOnCurv2.Value();
                            }
                        }
                    }
                }
            }
        }

        MAT2d_Connexion ConnexionL1L2 = new(IL1, IL2, IMinC1, IMinC2, Math.Sqrt(DistL1L2_2), ParameterOnC1,
            ParameterOnC2, Point1, Point2);

        return ConnexionL1L2;
    }
}
