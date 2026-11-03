// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT2d/MAT2d_Circuit.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT2d/MAT2d_Circuit.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Objects;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.JoinKind;

namespace Kerf.Engine.Offsets.MedialAxis;

/// <summary>
/// A circuit on a set of lines: one closed sequence of items (curves and the points of their sharp corners)
/// passing by all the lines and the connexions of their MAT2d_MiniPath.
/// </summary>
/// <remarks>
/// A working object of MAT2d_Tool2d. Perform inserts points into the lines of the figure it is given, as OCCT does
/// (the figure belongs to MAT2d_Mat2d's caller, BRepMAT2d_BisectingLocus, which built it); the curves it reverses
/// are copies. The map of references (keyed by MAT2d_BiInt) is iterated once, to sort each entry independently: its
/// order doesn't matter. The OCCT_DEBUG drawing is left out.
/// </remarks>
public sealed class Circuit : Transient
{
    private double _direction;
    private NCollection_Sequence<Geometry2D> _geomElements = new();
    private readonly Dictionary<int, Connexion> _connexionMap = new();
    private readonly Dictionary<BiInt, NCollection_Sequence<int>> _linkRefEqui = new();
    private readonly NCollection_Sequence<int> _linesLength = new();
    private readonly JoinKind _joinType;
    private readonly bool _isOpenResult;

    public Circuit(JoinKind aJoinType = Arc, bool isOpenResult = false)
    {
        _direction = 0.0;
        _joinType = aJoinType;
        _isOpenResult = isOpenResult;
    }

    /// <summary>Builds the circuit on the lines of FigItem (changed in place: corners and end points inserted, open
    /// lines doubled), IsClosed telling which are closed, from the line IndRefLine; Trigo if it turns in the
    /// trigonometric sense.</summary>
    public void Perform(NCollection_Sequence<NCollection_Sequence<Geometry2D>> figItem,
        NCollection_Sequence<bool> isClosed, int indRefLine, bool trigo)
    {
        int nbLines = figItem.Length();
        int i;
        NCollection_Array1<bool> open = new(1, nbLines);
        NCollection_Sequence<Connexion> sVide = new();
        Connexion? connexionNul = null;

        if (trigo)
        {
            _direction = 1.0;
        }
        else
        {
            _direction = -1.0;
        }

        //---------------------
        // Reinitialisation.
        //---------------------
        _geomElements.Clear();
        _connexionMap.Clear();
        _linkRefEqui.Clear();
        _linesLength.Clear();

        //----------------------------
        // Detection of the open lines.
        //----------------------------
        for (i = 1; i <= nbLines; i++)
        {
            TrimmedCurve2D curve;
            curve = (TrimmedCurve2D)figItem.Value(i).First();
            Point2D p1 = curve.StartPoint;
            curve = (TrimmedCurve2D)figItem.Value(i).Last();
            Point2D p2 = curve.EndPoint;
            if (isClosed.Value(i))
            {
                open[i] = false;
            }
            else if (p1.IsEqual(p2, global::Kerf.Engine.Foundation.Precision.Confusion()))
            {
                open[i] = false;
            }
            else
            {
                open[i] = true;
            }
        }

        //---------------------------------------------------------------
        // Insertion of the sharp corners, or of the ends of each curve if the line is open.
        //---------------------------------------------------------------
        for (i = 1; i <= nbLines; i++)
        {
            if (open[i])
            {
                InitOpen(figItem.ChangeValue(i));
                _linesLength.Append(figItem.Value(i).Length());
            }
            else
            {
                InsertCorner(figItem.ChangeValue(i));
                _linesLength.Append(figItem.Value(i).Length());
            }
        }

        //---------------------------------
        // A single line: nothing to do.
        //---------------------------------
        if (nbLines == 1)
        {
            if (open[1])
            {
                DoubleLine(figItem.ChangeValue(1), sVide, connexionNul, _direction);
                _linesLength.SetValue(1, figItem.Value(1).Length());
            }
            _geomElements = new NCollection_Sequence<Geometry2D>(figItem.Value(1));
            UpDateLink(1, 1, 1, _geomElements.Length());
            _linesLength.Append(figItem.Value(1).Length());
            return;
        }

        //------------------
        // Several lines.
        //------------------

        // The set of connexions making the path.
        MiniPath road = new();
        road.Perform(figItem, indRefLine, trigo);

        // Closing the open lines.
        for (i = 1; i <= nbLines; i++)
        {
            if (open[i])
            {
                Connexion? cf;
                if (road.IsRoot(i))
                {
                    cf = connexionNul;
                }
                else
                {
                    cf = road.Father(i);
                }
                if (road.IsConnexionsFrom(i))
                {
                    DoubleLine(figItem.ChangeValue(i), road.ConnexionsFrom(i), cf, _direction);
                }
                else
                {
                    DoubleLine(figItem.ChangeValue(i), sVide, cf, _direction);
                }
                _linesLength.SetValue(i, figItem.Value(i).Length());
            }
        }

        // The path.
        road.RunOnConnexions();

        // The circuit.
        ConstructCircuit(figItem, indRefLine, road);
    }

    // True if the common point of Geom1 and Geom2 is a sharp (salient) corner with respect to Direction.
    private bool IsSharpCorner(Geometry2D geom1, Geometry2D geom2, double direction)
    {
        double proVec = CrossProd(geom1, geom2, out double dotProd);
        int nbTest = 1;
        double du = global::Kerf.Engine.Foundation.Precision.Confusion();
        TrimmedCurve2D c1, c2;

        c1 = (TrimmedCurve2D)geom1;
        c2 = (TrimmedCurve2D)geom2;
        // The same criterion as in InitOpen.
        double tolAng = 1.0E-8;

        if (_joinType == Arc)
        {
            while (nbTest <= 10)
            {
                if ((proVec) * direction < -tolAng)
                {
                    return true; // Salient.
                }
                if ((proVec) * direction > tolAng)
                {
                    return false; // Re-entrant.
                }
                else
                {
                    if (dotProd > 0)
                    {
                        return false; // Flat.
                    }
                    tolAng = 1.0E-8;
                    double u1 = c1.LastParameter - nbTest * du;
                    double u2 = c2.FirstParameter + nbTest * du;
                    Direction2D dir1 = new(c1.DN(u1, 1));
                    Direction2D dir2 = new(c2.DN(u2, 1));
                    dotProd = dir1.Dot(dir2);
                    proVec = dir1 ^ dir2;
                    nbTest++;
                }
            }

            // Cusp: parallels to the two curves on the side of the domain; no intersection: salient, else
            // re-entrant.
            double d;
            double tol = global::Kerf.Engine.Foundation.Precision.Confusion();
            double milC1 = (c1.LastParameter + c1.FirstParameter) * 0.5;
            double milC2 = (c2.LastParameter + c2.FirstParameter) * 0.5;
            Point2D p = c1.Value(c1.LastParameter);
            Point2D p1 = c1.Value(milC1);
            Point2D p2 = c2.Value(milC2);

            d = Min(p1.DistanceTo(p), p2.DistanceTo(p));
            d /= 10;

            if (direction < 0.0)
            {
                d = -d;
            }

            CurveAdaptor2D hc1 = new(c1);
            CurveAdaptor2D hc2 = new(c2);
            OffsetCurveAdaptor2D oc1 = new(hc1, d, milC1, c1.LastParameter);
            OffsetCurveAdaptor2D oc2 = new(hc2, d, c2.FirstParameter, milC2);
            GInter intersect = new();
            intersect.Perform(oc1, oc2, tol, tol);

            return !intersect.IsDone || intersect.IsEmpty;
        }
        else if (_joinType == global::Kerf.Geometry.JoinKind.Intersection)
        {
            if (Math.Abs(proVec) <= tolAng && dotProd < 0)
            {
                while (nbTest <= 10)
                {
                    double u1 = c1.LastParameter - nbTest * du;
                    double u2 = c2.FirstParameter + nbTest * du;
                    Direction2D dir1 = new(c1.DN(u1, 1));
                    Direction2D dir2 = new(c2.DN(u2, 1));
                    dotProd = dir1.Dot(dir2);
                    proVec = dir1 ^ dir2;
                    if ((proVec) * direction < -tolAng)
                    {
                        return true; // Salient.
                    }
                    if ((proVec) * direction > tolAng)
                    {
                        return false; // Re-entrant.
                    }

                    nbTest++;
                }
                return false;
            }
            else
            {
                return false;
            }
        }
        return false;
    }

    private static void SubSequence(NCollection_Sequence<Geometry2D> s1, int IF, int il,
        NCollection_Sequence<Geometry2D> s2)
    {
        s2.Clear();
        for (int i = IF; i <= il; i++)
        {
            s2.Append(s1.Value(i));
        }
    }

    private void ConstructCircuit(NCollection_Sequence<NCollection_Sequence<Geometry2D>> figItem,
        int indRefLine, MiniPath road)
    {
        Connexion prevC;
        Connexion? curC = null;
        NCollection_Sequence<Geometry2D> setOfItem = new();
        int nbConnexions;
        int iLastItem;
        int indLast;
        int i;

        nbConnexions = road.Path().Length();

        // From the first element of the reference line.
        prevC = road.Path().Value(1);
        SubSequence(figItem.Value(indRefLine), 1, prevC.IndexItemOnFirst(), _geomElements);
        UpDateLink(1, indRefLine, 1, prevC.IndexItemOnFirst());
        _connexionMap[_geomElements.Length() + 1] = prevC;
        iLastItem = _geomElements.Length();

        // The portions of lines between two successive connexions.
        for (i = 2; i <= nbConnexions; i++)
        {
            curC = road.Path().Value(i);
            if (PassByLast(prevC, curC))
            {
                // The portion passes by the last element of the line: from PrevC to the last element, then, if
                // the line has more than one element, from its first element to CurC.
                indLast = figItem.Value(curC.IndexFirstLine()).Length();
                SubSequence(figItem.Value(curC.IndexFirstLine()), prevC.IndexItemOnSecond(), indLast, setOfItem);
                UpDateLink(iLastItem + 1, curC.IndexFirstLine(), prevC.IndexItemOnSecond(), indLast);
                _geomElements.Append(setOfItem);
                iLastItem = _geomElements.Length();

                if (figItem.Value(curC.IndexFirstLine()).Length() > 1)
                {
                    SubSequence(figItem.Value(curC.IndexFirstLine()), 1, curC.IndexItemOnFirst(), setOfItem);
                    UpDateLink(iLastItem + 1, curC.IndexFirstLine(), 1, curC.IndexItemOnFirst());
                    _geomElements.Append(setOfItem);
                    iLastItem = _geomElements.Length();
                }
                _connexionMap[iLastItem + 1] = curC;
            }
            else
            {
                // The portion doesn't pass by the last element of the line.
                SubSequence(figItem.Value(curC.IndexFirstLine()), prevC.IndexItemOnSecond(), curC.IndexItemOnFirst(),
                    setOfItem);
                UpDateLink(iLastItem + 1, curC.IndexFirstLine(), prevC.IndexItemOnSecond(), curC.IndexItemOnFirst());
                _geomElements.Append(setOfItem);
                iLastItem = _geomElements.Length();
                _connexionMap[iLastItem + 1] = curC;
            }
            prevC = curC;
        }

        // Closing: from the last connexion to the last element of the reference line.
        indLast = figItem.Value(indRefLine).Length();
        if (indLast == 1)
        {
            _connexionMap[1] = curC!;
            _connexionMap.Remove(iLastItem + 1);
        }
        else
        {
            SubSequence(figItem.Value(indRefLine), prevC.IndexItemOnSecond(), indLast, setOfItem);
            UpDateLink(iLastItem + 1, indRefLine, prevC.IndexItemOnSecond(), indLast);
            _geomElements.Append(setOfItem);
        }

        // Sorting of the RefToEqui of each element (each entry independently of the others).
        foreach (KeyValuePair<BiInt, NCollection_Sequence<int>> ite in _linkRefEqui)
        {
            if (ite.Value.Length() > 1)
            {
                SortRefToEqui(ite.Key);
            }
        }
    }

    private static void InitOpen(NCollection_Sequence<Geometry2D> line)
    {
        TrimmedCurve2D curve;

        curve = (TrimmedCurve2D)line.First();
        line.InsertBefore(1, new CartesianPoint2(curve.StartPoint));
        curve = (TrimmedCurve2D)line.Last();
        line.Append(new CartesianPoint2(curve.EndPoint));

        for (int i = 2; i <= line.Length() - 2; i++)
        {
            if (Math.Abs(CrossProd(line.Value(i), line.Value(i + 1), out double dotProd)) > 1.0E-8 || dotProd < 0.0)
            {
                curve = (TrimmedCurve2D)line.Value(i);
                line.InsertAfter(i, new CartesianPoint2(curve.EndPoint));
                i++;
            }
        }
    }

    private void DoubleLine(NCollection_Sequence<Geometry2D> line,
        NCollection_Sequence<Connexion> connexionFrom, Connexion? connexionFather, double sideRef)
    {
        Type type;
        TrimmedCurve2D curve;
        int nbItems = line.Length();
        int i;
        double proVec;
        Connexion cc;

        //--------------------------
        // Completion of the line.
        //--------------------------
        if (!_isOpenResult)
        {
            for (i = nbItems - 1; i > 1; i--)
            {
                type = line.Value(i).GetType();
                if (type == typeof(CartesianPoint2))
                {
                    line.Append(line.Value(i));
                }
                else
                {
                    curve = (TrimmedCurve2D)line.Value(i).Copy();
                    curve.Reverse();
                    line.Append(curve);
                }
            }
        }

        //------------------------------------------
        // The connexions along the line.
        //------------------------------------------
        int iAfter = connexionFrom.Length();
        int nbConnexions = iAfter;
        int indCOF;

        for (i = 1; i <= iAfter; i++)
        {
            cc = connexionFrom.Value(i);
            indCOF = cc.IndexItemOnFirst();
            type = line.Value(indCOF).GetType();

            if (type == typeof(CartesianPoint2))
            {
                if (indCOF != nbItems && indCOF != 1)
                {
                    proVec = CrossProd(line.Value(indCOF - 1), line.Value(indCOF + 1), out _);
                    if ((proVec) * sideRef > 0)
                    {
                        cc.IndexItemOnFirst(2 * nbItems - indCOF);
                        connexionFrom.InsertAfter(iAfter, cc);
                        connexionFrom.Remove(i);
                        iAfter--;
                        i--;
                    }
                }
            }
            else if (Side(cc, line) != sideRef)
            {
                curve = (TrimmedCurve2D)line.Value(indCOF);
                cc.IndexItemOnFirst(2 * nbItems - indCOF);
                cc.ParameterOnFirst(curve.ReversedParameter(cc.ParameterOnFirst()));
                connexionFrom.InsertAfter(iAfter, cc);
                connexionFrom.Remove(i);
                iAfter--;
                i--;
            }
        }

        //---------------------------
        // The father connexion.
        //---------------------------
        if (connexionFather is not null)
        {
            cc = connexionFather.Reverse();
            indCOF = cc.IndexItemOnFirst();
            type = line.Value(indCOF).GetType();

            if (type == typeof(CartesianPoint2))
            {
                if (indCOF != nbItems && indCOF != 1)
                {
                    proVec = CrossProd(line.Value(indCOF - 1), line.Value(indCOF + 1), out _);
                    if ((proVec) * sideRef > 0)
                    {
                        connexionFather.IndexItemOnSecond(2 * nbItems - indCOF);
                    }
                }
            }
            else if (Side(cc, line) != sideRef)
            {
                curve = (TrimmedCurve2D)line.Value(indCOF);
                connexionFather.IndexItemOnSecond(2 * nbItems - indCOF);
                connexionFather.ParameterOnSecond(curve.ReversedParameter(connexionFather.ParameterOnSecond()));
            }
        }

        //-------------------------------------
        // Removal of the re-entrant corners.
        //-------------------------------------
        int indLine = 1;
        int iCorres = 1;
        NCollection_Array1<int> corres = new(1, line.Length());

        while (line.Value(indLine) != line.Last())
        {
            corres[iCorres] = indLine;
            type = line.Value(indLine).GetType();
            if (type == typeof(CartesianPoint2) && iCorres != 1 && iCorres != nbItems)
            {
                if (!IsSharpCorner(line.Value(indLine - 1), line.Value(indLine + 1), sideRef))
                {
                    line.Remove(indLine);
                    indLine--;
                    corres[iCorres] = 0;
                }
            }
            indLine++;
            iCorres++;
        }
        corres[iCorres] = indLine;

        if (!_isOpenResult)
        {
            for (i = 1; i < 2 * nbItems - 2; i++)
            {
                if (corres[i] == 0)
                {
                    corres[i] = corres[2 * nbItems - i];
                }
            }

            //----------------------------
            // The connexions.
            //----------------------------
            for (i = 1; i <= nbConnexions; i++)
            {
                cc = connexionFrom.ChangeValue(i);
                cc.IndexItemOnFirst(corres[cc.IndexItemOnFirst()]);
            }

            if (connexionFather is not null)
            {
                connexionFather.IndexItemOnSecond(corres[connexionFather.IndexItemOnSecond()]);
            }
        }
    }

    private void InsertCorner(NCollection_Sequence<Geometry2D> line)
    {
        int i, isuiv;
        TrimmedCurve2D curve;
        bool insert;

        for (i = 1; i <= line.Length(); i++)
        {
            isuiv = (i == line.Length()) ? 1 : i + 1;
            insert = IsSharpCorner(line.Value(i), line.Value(isuiv), _direction);

            if (insert)
            {
                curve = (TrimmedCurve2D)line.Value(isuiv);
                line.InsertAfter(i, new CartesianPoint2(curve.StartPoint));
                i++;
            }
        }
    }

    /// <summary>The number of items.</summary>
    public int NumberOfItems() => _geomElements.Length();

    /// <summary>The number of items on the line I.</summary>
    public int LineLength(int i) => _linesLength.Value(i);

    /// <summary>The item of index Index.</summary>
    public Geometry2D Value(int index) => _geomElements.Value(index);

    /// <summary>The indices of the items of the circuit made from the curve IndCurve of the line IndLine of the
    /// initial figure.</summary>
    public NCollection_Sequence<int> RefToEqui(int indLine, int indCurve)
    {
        BiInt key = new(indLine, indCurve);
        return _linkRefEqui.GetValue(key);
    }

    private void SortRefToEqui(in BiInt biRef)
    {
        int i;
        NCollection_Sequence<int> s = _linkRefEqui.GetValue(biRef);
        NCollection_Sequence<int> sFin;

        for (i = 1; i <= s.Length(); i++)
        {
            if (!ConnexionOn(s.Value(i)))
            {
                break;
            }
        }
        if (i > 1 && i <= s.Length())
        {
            sFin = new NCollection_Sequence<int>(s);
            sFin.Split(i, s);
            s.Append(sFin);
        }
    }

    /// <summary>The connexion on the item I.</summary>
    public Connexion Connexion(int i) => _connexionMap.GetValue(i);

    /// <summary>True if there is a connexion on the item I.</summary>
    public bool ConnexionOn(int i) => _connexionMap.ContainsKey(i);

    private static double Side(Connexion c1, NCollection_Sequence<Geometry2D> line)
    {
        TrimmedCurve2D curve;

        Vector2D vect1 = new(c1.PointOnSecond().X - c1.PointOnFirst().X,
            c1.PointOnSecond().Y - c1.PointOnFirst().Y);
        curve = (TrimmedCurve2D)line.Value(c1.IndexItemOnFirst());
        Vector2D vect2 = curve.DN(c1.ParameterOnFirst(), 1);
        if ((vect1 ^ vect2) > 0.0)
        {
            return -1.0;
        }
        else
        {
            return 1.0;
        }
    }

    private bool PassByLast(Connexion c1, Connexion c2)
    {
        if (c2.IndexFirstLine() == c1.IndexSecondLine())
        {
            if (c2.IndexItemOnFirst() < c1.IndexItemOnSecond())
            {
                return true;
            }
            else if (c2.IndexItemOnFirst() == c1.IndexItemOnSecond())
            {
                if (c1.IndexFirstLine() == c2.IndexSecondLine())
                {
                    return true;
                }
                if (c2.ParameterOnFirst() == c1.ParameterOnSecond())
                {
                    Vector2D vect1 = new(c1.PointOnSecond(), c1.PointOnFirst());
                    Vector2D vect2 = new(c2.PointOnFirst(), c2.PointOnSecond());
                    if ((vect1 ^ vect2) * _direction > 0)
                    {
                        return true;
                    }
                }
                else if (c2.ParameterOnFirst() < c1.ParameterOnSecond())
                {
                    return true;
                }
            }
        }
        return false;
    }

    private void UpDateLink(int iFirst, int iLine, int iCurveFirst, int iCurveLast)
    {
        int iEqui = iFirst;
        int i;

        for (i = iCurveFirst; i <= iCurveLast; i++)
        {
            BiInt key = new(iLine, i);
            if (_linkRefEqui.ContainsKey(key))
            {
                _linkRefEqui.GetValue(key).Append(iEqui);
            }
            else
            {
                NCollection_Sequence<int> l = new();
                _linkRefEqui[key] = l;
                _linkRefEqui.GetValue(key).Append(iEqui);
            }
            iEqui++;
        }
    }

    // The cross and dot products of the tangent directions at the end of Geom1 and at the start of Geom2 (both
    // curves).
    private static double CrossProd(Geometry2D geom1, Geometry2D geom2, out double dotProd)
    {
        TrimmedCurve2D curve;

        curve = (TrimmedCurve2D)geom1;
        Direction2D dir1 = new(curve.DN(curve.LastParameter, 1));
        curve = (TrimmedCurve2D)geom2;
        Direction2D dir2 = new(curve.DN(curve.FirstParameter, 1));
        dotProd = dir1.Dot(dir2);
        return dir1 ^ dir2;
    }
}
