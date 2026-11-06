// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT2d/MAT2d_MiniPath.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT2d/MAT2d_MiniPath.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;

namespace Kerf.Engine.Offsets.MedialAxis;

/// <summary>
/// Computes a path linking all the lines of a set of lines, as a sequence of connexions. The connexions form a
/// tree whose nodes are the lines: the ancestor of a line is the connexion ending on it, its children the
/// connexions starting on it, ordered by MAT2d_Connexion.IsAfter.
/// </summary>
/// <remarks>A working object of MAT2d_Circuit. The maps are keyed by line index and only looked up.</remarks>
internal sealed class MiniPath
{
    private readonly Dictionary<int, NCollection_Sequence<Connexion>> _connexions = new();
    private readonly Dictionary<int, Connexion> _father = new();
    private readonly NCollection_Sequence<Connexion> _path = new();
    private double _direction;
    private int _indStart;

    public MiniPath()
    {
        _direction = 1.0;
        _indStart = 0;
    }

    /// <summary>Computes the path linking the lines of Figure, starting on the line IndStart. Sense is true if the
    /// circuit turns in the trigonometric sense.</summary>
    public void Perform(NCollection_Sequence<NCollection_Sequence<Geometry2D>> figure, int indStart, bool sense)
    {
        int i, j;
        int nbLines = figure.Length();
        NCollection_Array2<Connexion> connexion = new(1, nbLines, 1, nbLines);

        _indStart = indStart;
        _direction = 1.0;
        if (sense)
        {
            _direction = -1.0;
        }

        // The connexions realising the minimum of distance between the elements of the figure.
        for (i = 1; i < nbLines; i++)
        {
            for (j = i + 1; j <= nbLines; j++)
            {
                connexion[i, j] = MinimumL1L2(figure, i, j);
                connexion[j, i] = connexion[i, j].Reverse();
            }
        }

        NCollection_Sequence<int> set1 = new();
        NCollection_Sequence<int> set2 = new();
        double distS1S2;
        int indiceLine1, indiceLine2;
        int iSuiv = 0, minOnSet1 = 0, minOnSet2 = 0;

        // - 0 Set1 starts with the start line, Set2 holds the others.
        set1.Append(indStart);
        for (i = 1; i <= nbLines; i++)
        {
            if (i != indStart)
            {
                set2.Append(i);
            }
        }

        // - 1 The shortest connexion C between Set1 and Set2.
        // - 2 The line of Set2 realising it moves to Set1.
        // - 3 C is inserted in the connexions.
        // - 4 Back to 1 while Set2 isn't empty.
        while (!set2.IsEmpty())
        {
            distS1S2 = RealLast();
            for (i = 1; i <= set1.Length(); i++)
            {
                indiceLine1 = set1.Value(i);
                for (j = 1; j <= set2.Length(); j++)
                {
                    indiceLine2 = set2.Value(j);
                    if (connexion[indiceLine1, indiceLine2].Distance() < distS1S2)
                    {
                        iSuiv = j;
                        distS1S2 = connexion[indiceLine1, indiceLine2].Distance();
                        minOnSet1 = indiceLine1;
                        minOnSet2 = indiceLine2;
                    }
                }
            }
            set1.Append(set2.Value(iSuiv));
            set2.Remove(iSuiv);
            Append(connexion[minOnSet1, minOnSet2]);
        }

        // The path, running over the connexions.
        RunOnConnexions();
    }

    // Inserts a connexion in the tree of connexions (the lines are its nodes, the connexions its branches).
    private void Append(Connexion c)
    {
        Connexion cc;

        if (!_connexions.ContainsKey(c.IndexFirstLine()))
        {
            NCollection_Sequence<Connexion> seq0 = new();
            _connexions[c.IndexFirstLine()] = seq0;
            _connexions.GetValue(c.IndexFirstLine()).Append(c);
            _father[c.IndexSecondLine()] = c;
            return;
        }

        NCollection_Sequence<Connexion> seq = _connexions.GetValue(c.IndexFirstLine());
        int indexAfter = 0;
        int nbConnexions = seq.Length();

        for (int i = 1; i <= nbConnexions; i++)
        {
            cc = seq.Value(i);
            if (cc.IsAfter(c, _direction))
            {
                indexAfter = i;
                break;
            }
        }
        // C is inserted before IndexAfter; if 0, no connexion is after C, which is the last.
        if (indexAfter == 0)
        {
            seq.Append(c);
        }
        else
        {
            seq.InsertBefore(indexAfter, c);
        }
        _father[c.IndexSecondLine()] = c;
    }

    /// <summary>The sequence of connexions of the path.</summary>
    public NCollection_Sequence<Connexion> Path() => _path;

    /// <summary>True if a connexion starts on the line Index.</summary>
    public bool IsConnexionsFrom(int i) => _connexions.ContainsKey(i);

    /// <summary>The connexions starting on the line Index.</summary>
    public NCollection_Sequence<Connexion> ConnexionsFrom(int i) => _connexions.GetValue(i);

    /// <summary>True if the line Index is the root.</summary>
    public bool IsRoot(int iLine) => iLine == _indStart;

    /// <summary>The connexion ending on the line ILine.</summary>
    public Connexion Father(int iLine) => _father.GetValue(iLine);

    /// <summary>Builds the path: an exploration of the tree of connexions and their reverses.</summary>
    public void RunOnConnexions()
    {
        int i;
        Connexion c;
        NCollection_Sequence<Connexion> sc = _connexions.GetValue(_indStart);

        _path.Clear();

        for (i = 1; i <= sc.Length(); i++)
        {
            c = sc.Value(i);
            _path.Append(c);
            ExploSons(_path, c);
            _path.Append(c.Reverse());
        }
    }

    private void ExploSons(NCollection_Sequence<Connexion> cResult, Connexion cRef)
    {
        int i;
        int index = cRef.IndexSecondLine();

        if (!_connexions.ContainsKey(index))
        {
            return;
        }

        NCollection_Sequence<Connexion> sc = _connexions.GetValue(index);
        Connexion crr = cRef.Reverse();
        Connexion c;

        for (i = 1; i <= sc.Length(); i++)
        {
            c = sc.Value(i);
            if (c.IsAfter(crr, _direction))
            {
                cResult.Append(c);
                ExploSons(cResult, c);
                cResult.Append(c.Reverse());
            }
        }

        for (i = 1; i <= sc.Length(); i++)
        {
            c = sc.Value(i);
            if (!c.IsAfter(crr, _direction))
            {
                cResult.Append(c);
                ExploSons(cResult, c);
                cResult.Append(c.Reverse());
            }
            else
            {
                break;
            }
        }
    }

    // The connexion realising the minimum of distance between the lines IL1 and IL2 of Figure, oriented from IL1
    // to IL2.
    private static Connexion MinimumL1L2(NCollection_Sequence<NCollection_Sequence<Geometry2D>> figure,
        int il1, int il2)
    {
        POnCurv2d pointOnCurv1, pointOnCurv2;
        int ic1, ic2, iMinC1 = 0, iMinC2 = 0, i;
        double DistL1L2_2, DistP1P2_2;
        double parameterOnC1 = 0.0, parameterOnC2 = 0.0;
        NCollection_Sequence<Geometry2D> l1, l2;
        Point2D point1 = new(), point2 = new(), p1 = new(), p2 = new();
        Curve2D? item1 = null;
        Curve2D? item2 = null;

        l1 = figure.Value(il1);
        l2 = figure.Value(il2);

        DistL1L2_2 = RealLast();

        // The extrema of distance between the components of L1 and L2.
        for (ic1 = 1; ic1 <= l1.Length(); ic1++)
        {
            Type type1 = l1.Value(ic1).GetType();
            if (type1 != typeof(CartesianPoint2))
            {
                item1 = (Curve2D)l1.Value(ic1);
            }
            else
            {
                p1 = ((Point2)l1.Value(ic1)).Pnt2d();
            }

            for (ic2 = 1; ic2 <= l2.Length(); ic2++)
            {
                Type type2 = l2.Value(ic2).GetType();
                if (type2 != typeof(CartesianPoint2))
                {
                    item2 = (Curve2D)l2.Value(ic2);
                }
                else
                {
                    p2 = ((Point2)l2.Value(ic2)).Pnt2d();
                }

                if (type1 == typeof(CartesianPoint2) && type2 == typeof(CartesianPoint2))
                {
                    DistP1P2_2 = p1.SquareDistanceTo(p2);
                    if (DistP1P2_2 <= DistL1L2_2)
                    {
                        DistL1L2_2 = DistP1P2_2;
                        iMinC1 = ic1;
                        iMinC2 = ic2;
                        point1 = p1;
                        point2 = p2;
                        parameterOnC1 = 0.0;
                        parameterOnC2 = 0.0;
                    }
                }
                else if (type1 == typeof(CartesianPoint2))
                {
                    CurveAdaptor2D c2 = new(item2!);
                    ExtPC2d extremas = new(p1, c2);
                    if (extremas.IsDone)
                    {
                        for (i = 1; i <= extremas.ExtCount; i++)
                        {
                            if (extremas.SquareDistance(i) < DistL1L2_2)
                            {
                                DistL1L2_2 = extremas.SquareDistance(i);
                                iMinC1 = ic1;
                                iMinC2 = ic2;
                                pointOnCurv2 = extremas.Point(i);
                                parameterOnC1 = 0.0;
                                parameterOnC2 = pointOnCurv2.Parameter();
                                point1 = p1;
                                point2 = pointOnCurv2.Value();
                            }
                        }
                    }
                }
                else if (type2 == typeof(CartesianPoint2))
                {
                    CurveAdaptor2D c1 = new(item1!);
                    ExtPC2d extremas = new(p2, c1);
                    if (extremas.IsDone)
                    {
                        for (i = 1; i <= extremas.ExtCount; i++)
                        {
                            if (extremas.SquareDistance(i) < DistL1L2_2)
                            {
                                DistL1L2_2 = extremas.SquareDistance(i);
                                iMinC1 = ic1;
                                iMinC2 = ic2;
                                pointOnCurv1 = extremas.Point(i);
                                parameterOnC2 = 0.0;
                                parameterOnC1 = pointOnCurv1.Parameter();
                                point1 = pointOnCurv1.Value();
                                point2 = p2;
                            }
                        }
                    }
                }
                else
                {
                    CurveAdaptor2D c1 = new(item1!);
                    CurveAdaptor2D c2 = new(item2!);
                    ExtCC2d extremas = new(c1, c2);
                    if (!extremas.IsParallel && extremas.IsDone)
                    {
                        for (i = 1; i <= extremas.ExtCount; i++)
                        {
                            if (extremas.SquareDistance(i) < DistL1L2_2)
                            {
                                DistL1L2_2 = extremas.SquareDistance(i);
                                iMinC1 = ic1;
                                iMinC2 = ic2;
                                extremas.Points(i, out pointOnCurv1, out pointOnCurv2);
                                parameterOnC1 = pointOnCurv1.Parameter();
                                parameterOnC2 = pointOnCurv2.Parameter();
                                point1 = pointOnCurv1.Value();
                                point2 = pointOnCurv2.Value();
                            }
                        }
                    }
                }
            }
        }

        Connexion connexionL1L2 = new(il1, il2, iMinC1, iMinC2, Math.Sqrt(DistL1L2_2), parameterOnC1,
            parameterOnC2, point1, point2);

        return connexionL1L2;
    }
}
