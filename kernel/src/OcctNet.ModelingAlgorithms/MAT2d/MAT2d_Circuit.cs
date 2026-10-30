// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT2d/MAT2d_Circuit.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT2d/MAT2d_Circuit.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_JoinType;

namespace Occt;

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
public sealed class MAT2d_Circuit : Standard_Transient
{
    private double direction;
    private NCollection_Sequence<Geom2d_Geometry> geomElements = new();
    private readonly NCollection_DataMap<int, MAT2d_Connexion> connexionMap = new();
    private readonly NCollection_DataMap<MAT2d_BiInt, NCollection_Sequence<int>> linkRefEqui = new();
    private readonly NCollection_Sequence<int> linesLength = new();
    private readonly GeomAbs_JoinType myJoinType;
    private readonly bool myIsOpenResult;

    public MAT2d_Circuit(GeomAbs_JoinType aJoinType = GeomAbs_Arc, bool IsOpenResult = false)
    {
        direction = 0.0;
        myJoinType = aJoinType;
        myIsOpenResult = IsOpenResult;
    }

    /// <summary>Builds the circuit on the lines of FigItem (changed in place: corners and end points inserted, open
    /// lines doubled), IsClosed telling which are closed, from the line IndRefLine; Trigo if it turns in the
    /// trigonometric sense.</summary>
    public void Perform(NCollection_Sequence<NCollection_Sequence<Geom2d_Geometry>> FigItem,
        NCollection_Sequence<bool> IsClosed, int IndRefLine, bool Trigo)
    {
        int NbLines = FigItem.Length();
        int i;
        NCollection_Array1<bool> Open = new(1, NbLines);
        NCollection_Sequence<MAT2d_Connexion> SVide = new();
        MAT2d_Connexion? ConnexionNul = null;

        if (Trigo)
        {
            direction = 1.0;
        }
        else
        {
            direction = -1.0;
        }

        //---------------------
        // Reinitialisation.
        //---------------------
        geomElements.Clear();
        connexionMap.Clear();
        linkRefEqui.Clear();
        linesLength.Clear();

        //----------------------------
        // Detection of the open lines.
        //----------------------------
        for (i = 1; i <= NbLines; i++)
        {
            Geom2d_TrimmedCurve Curve;
            Curve = (Geom2d_TrimmedCurve)FigItem.Value(i).First();
            gp_Pnt2d P1 = Curve.StartPoint();
            Curve = (Geom2d_TrimmedCurve)FigItem.Value(i).Last();
            gp_Pnt2d P2 = Curve.EndPoint();
            if (IsClosed.Value(i))
            {
                Open[i] = false;
            }
            else if (P1.IsEqual(P2, Precision.Confusion()))
            {
                Open[i] = false;
            }
            else
            {
                Open[i] = true;
            }
        }

        //---------------------------------------------------------------
        // Insertion of the sharp corners, or of the ends of each curve if the line is open.
        //---------------------------------------------------------------
        for (i = 1; i <= NbLines; i++)
        {
            if (Open[i])
            {
                InitOpen(FigItem.ChangeValue(i));
                linesLength.Append(FigItem.Value(i).Length());
            }
            else
            {
                InsertCorner(FigItem.ChangeValue(i));
                linesLength.Append(FigItem.Value(i).Length());
            }
        }

        //---------------------------------
        // A single line: nothing to do.
        //---------------------------------
        if (NbLines == 1)
        {
            if (Open[1])
            {
                DoubleLine(FigItem.ChangeValue(1), SVide, ConnexionNul, direction);
                linesLength.SetValue(1, FigItem.Value(1).Length());
            }
            geomElements = new NCollection_Sequence<Geom2d_Geometry>(FigItem.Value(1));
            UpDateLink(1, 1, 1, geomElements.Length());
            linesLength.Append(FigItem.Value(1).Length());
            return;
        }

        //------------------
        // Several lines.
        //------------------

        // The set of connexions making the path.
        MAT2d_MiniPath Road = new();
        Road.Perform(FigItem, IndRefLine, Trigo);

        // Closing the open lines.
        for (i = 1; i <= NbLines; i++)
        {
            if (Open[i])
            {
                MAT2d_Connexion? CF;
                if (Road.IsRoot(i))
                {
                    CF = ConnexionNul;
                }
                else
                {
                    CF = Road.Father(i);
                }
                if (Road.IsConnexionsFrom(i))
                {
                    DoubleLine(FigItem.ChangeValue(i), Road.ConnexionsFrom(i), CF, direction);
                }
                else
                {
                    DoubleLine(FigItem.ChangeValue(i), SVide, CF, direction);
                }
                linesLength.SetValue(i, FigItem.Value(i).Length());
            }
        }

        // The path.
        Road.RunOnConnexions();

        // The circuit.
        ConstructCircuit(FigItem, IndRefLine, Road);
    }

    // True if the common point of Geom1 and Geom2 is a sharp (salient) corner with respect to Direction.
    private bool IsSharpCorner(Geom2d_Geometry Geom1, Geom2d_Geometry Geom2, double Direction)
    {
        double ProVec = CrossProd(Geom1, Geom2, out double DotProd);
        int NbTest = 1;
        double DU = Precision.Confusion();
        Geom2d_TrimmedCurve C1, C2;

        C1 = (Geom2d_TrimmedCurve)Geom1;
        C2 = (Geom2d_TrimmedCurve)Geom2;
        // The same criterion as in InitOpen.
        double TolAng = 1.0E-8;

        if (myJoinType == GeomAbs_Arc)
        {
            while (NbTest <= 10)
            {
                if ((ProVec) * Direction < -TolAng)
                {
                    return true; // Salient.
                }
                if ((ProVec) * Direction > TolAng)
                {
                    return false; // Re-entrant.
                }
                else
                {
                    if (DotProd > 0)
                    {
                        return false; // Flat.
                    }
                    TolAng = 1.0E-8;
                    double U1 = C1.LastParameter() - NbTest * DU;
                    double U2 = C2.FirstParameter() + NbTest * DU;
                    gp_Dir2d Dir1 = new(C1.DN(U1, 1));
                    gp_Dir2d Dir2 = new(C2.DN(U2, 1));
                    DotProd = Dir1.Dot(Dir2);
                    ProVec = Dir1 ^ Dir2;
                    NbTest++;
                }
            }

            // Cusp: parallels to the two curves on the side of the domain; no intersection: salient, else
            // re-entrant.
            double D;
            double Tol = Precision.Confusion();
            double MilC1 = (C1.LastParameter() + C1.FirstParameter()) * 0.5;
            double MilC2 = (C2.LastParameter() + C2.FirstParameter()) * 0.5;
            gp_Pnt2d P = C1.Value(C1.LastParameter());
            gp_Pnt2d P1 = C1.Value(MilC1);
            gp_Pnt2d P2 = C2.Value(MilC2);

            D = Min(P1.Distance(P), P2.Distance(P));
            D /= 10;

            if (Direction < 0.0)
            {
                D = -D;
            }

            Geom2dAdaptor_Curve HC1 = new(C1);
            Geom2dAdaptor_Curve HC2 = new(C2);
            Adaptor2d_OffsetCurve OC1 = new(HC1, D, MilC1, C1.LastParameter());
            Adaptor2d_OffsetCurve OC2 = new(HC2, D, C2.FirstParameter(), MilC2);
            Geom2dInt_GInter Intersect = new();
            Intersect.Perform(OC1, OC2, Tol, Tol);

            return !Intersect.IsDone() || Intersect.IsEmpty();
        }
        else if (myJoinType == GeomAbs_Intersection)
        {
            if (Math.Abs(ProVec) <= TolAng && DotProd < 0)
            {
                while (NbTest <= 10)
                {
                    double U1 = C1.LastParameter() - NbTest * DU;
                    double U2 = C2.FirstParameter() + NbTest * DU;
                    gp_Dir2d Dir1 = new(C1.DN(U1, 1));
                    gp_Dir2d Dir2 = new(C2.DN(U2, 1));
                    DotProd = Dir1.Dot(Dir2);
                    ProVec = Dir1 ^ Dir2;
                    if ((ProVec) * Direction < -TolAng)
                    {
                        return true; // Salient.
                    }
                    if ((ProVec) * Direction > TolAng)
                    {
                        return false; // Re-entrant.
                    }

                    NbTest++;
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

    private static void SubSequence(NCollection_Sequence<Geom2d_Geometry> S1, int IF, int IL,
        NCollection_Sequence<Geom2d_Geometry> S2)
    {
        S2.Clear();
        for (int i = IF; i <= IL; i++)
        {
            S2.Append(S1.Value(i));
        }
    }

    private void ConstructCircuit(NCollection_Sequence<NCollection_Sequence<Geom2d_Geometry>> FigItem,
        int IndRefLine, MAT2d_MiniPath Road)
    {
        MAT2d_Connexion PrevC;
        MAT2d_Connexion? CurC = null;
        NCollection_Sequence<Geom2d_Geometry> SetOfItem = new();
        int NbConnexions;
        int ILastItem;
        int IndLast;
        int i;

        NbConnexions = Road.Path().Length();

        // From the first element of the reference line.
        PrevC = Road.Path().Value(1);
        SubSequence(FigItem.Value(IndRefLine), 1, PrevC.IndexItemOnFirst(), geomElements);
        UpDateLink(1, IndRefLine, 1, PrevC.IndexItemOnFirst());
        connexionMap.Bind(geomElements.Length() + 1, PrevC);
        ILastItem = geomElements.Length();

        // The portions of lines between two successive connexions.
        for (i = 2; i <= NbConnexions; i++)
        {
            CurC = Road.Path().Value(i);
            if (PassByLast(PrevC, CurC))
            {
                // The portion passes by the last element of the line: from PrevC to the last element, then, if
                // the line has more than one element, from its first element to CurC.
                IndLast = FigItem.Value(CurC.IndexFirstLine()).Length();
                SubSequence(FigItem.Value(CurC.IndexFirstLine()), PrevC.IndexItemOnSecond(), IndLast, SetOfItem);
                UpDateLink(ILastItem + 1, CurC.IndexFirstLine(), PrevC.IndexItemOnSecond(), IndLast);
                geomElements.Append(SetOfItem);
                ILastItem = geomElements.Length();

                if (FigItem.Value(CurC.IndexFirstLine()).Length() > 1)
                {
                    SubSequence(FigItem.Value(CurC.IndexFirstLine()), 1, CurC.IndexItemOnFirst(), SetOfItem);
                    UpDateLink(ILastItem + 1, CurC.IndexFirstLine(), 1, CurC.IndexItemOnFirst());
                    geomElements.Append(SetOfItem);
                    ILastItem = geomElements.Length();
                }
                connexionMap.Bind(ILastItem + 1, CurC);
            }
            else
            {
                // The portion doesn't pass by the last element of the line.
                SubSequence(FigItem.Value(CurC.IndexFirstLine()), PrevC.IndexItemOnSecond(), CurC.IndexItemOnFirst(),
                    SetOfItem);
                UpDateLink(ILastItem + 1, CurC.IndexFirstLine(), PrevC.IndexItemOnSecond(), CurC.IndexItemOnFirst());
                geomElements.Append(SetOfItem);
                ILastItem = geomElements.Length();
                connexionMap.Bind(ILastItem + 1, CurC);
            }
            PrevC = CurC;
        }

        // Closing: from the last connexion to the last element of the reference line.
        IndLast = FigItem.Value(IndRefLine).Length();
        if (IndLast == 1)
        {
            connexionMap.Bind(1, CurC!);
            connexionMap.UnBind(ILastItem + 1);
        }
        else
        {
            SubSequence(FigItem.Value(IndRefLine), PrevC.IndexItemOnSecond(), IndLast, SetOfItem);
            UpDateLink(ILastItem + 1, IndRefLine, PrevC.IndexItemOnSecond(), IndLast);
            geomElements.Append(SetOfItem);
        }

        // Sorting of the RefToEqui of each element (each entry independently of the others).
        foreach (KeyValuePair<MAT2d_BiInt, NCollection_Sequence<int>> Ite in linkRefEqui)
        {
            if (Ite.Value.Length() > 1)
            {
                SortRefToEqui(Ite.Key);
            }
        }
    }

    private static void InitOpen(NCollection_Sequence<Geom2d_Geometry> Line)
    {
        Geom2d_TrimmedCurve Curve;

        Curve = (Geom2d_TrimmedCurve)Line.First();
        Line.InsertBefore(1, new Geom2d_CartesianPoint(Curve.StartPoint()));
        Curve = (Geom2d_TrimmedCurve)Line.Last();
        Line.Append(new Geom2d_CartesianPoint(Curve.EndPoint()));

        for (int i = 2; i <= Line.Length() - 2; i++)
        {
            if (Math.Abs(CrossProd(Line.Value(i), Line.Value(i + 1), out double DotProd)) > 1.0E-8 || DotProd < 0.0)
            {
                Curve = (Geom2d_TrimmedCurve)Line.Value(i);
                Line.InsertAfter(i, new Geom2d_CartesianPoint(Curve.EndPoint()));
                i++;
            }
        }
    }

    private void DoubleLine(NCollection_Sequence<Geom2d_Geometry> Line,
        NCollection_Sequence<MAT2d_Connexion> ConnexionFrom, MAT2d_Connexion? ConnexionFather, double SideRef)
    {
        Type Type;
        Geom2d_TrimmedCurve Curve;
        int NbItems = Line.Length();
        int i;
        double ProVec;
        MAT2d_Connexion CC;

        //--------------------------
        // Completion of the line.
        //--------------------------
        if (!myIsOpenResult)
        {
            for (i = NbItems - 1; i > 1; i--)
            {
                Type = Line.Value(i).GetType();
                if (Type == typeof(Geom2d_CartesianPoint))
                {
                    Line.Append(Line.Value(i));
                }
                else
                {
                    Curve = (Geom2d_TrimmedCurve)Line.Value(i).Copy();
                    Curve.Reverse();
                    Line.Append(Curve);
                }
            }
        }

        //------------------------------------------
        // The connexions along the line.
        //------------------------------------------
        int IAfter = ConnexionFrom.Length();
        int NbConnexions = IAfter;
        int IndCOF;

        for (i = 1; i <= IAfter; i++)
        {
            CC = ConnexionFrom.Value(i);
            IndCOF = CC.IndexItemOnFirst();
            Type = Line.Value(IndCOF).GetType();

            if (Type == typeof(Geom2d_CartesianPoint))
            {
                if (IndCOF != NbItems && IndCOF != 1)
                {
                    ProVec = CrossProd(Line.Value(IndCOF - 1), Line.Value(IndCOF + 1), out _);
                    if ((ProVec) * SideRef > 0)
                    {
                        CC.IndexItemOnFirst(2 * NbItems - IndCOF);
                        ConnexionFrom.InsertAfter(IAfter, CC);
                        ConnexionFrom.Remove(i);
                        IAfter--;
                        i--;
                    }
                }
            }
            else if (Side(CC, Line) != SideRef)
            {
                Curve = (Geom2d_TrimmedCurve)Line.Value(IndCOF);
                CC.IndexItemOnFirst(2 * NbItems - IndCOF);
                CC.ParameterOnFirst(Curve.ReversedParameter(CC.ParameterOnFirst()));
                ConnexionFrom.InsertAfter(IAfter, CC);
                ConnexionFrom.Remove(i);
                IAfter--;
                i--;
            }
        }

        //---------------------------
        // The father connexion.
        //---------------------------
        if (ConnexionFather is not null)
        {
            CC = ConnexionFather.Reverse();
            IndCOF = CC.IndexItemOnFirst();
            Type = Line.Value(IndCOF).GetType();

            if (Type == typeof(Geom2d_CartesianPoint))
            {
                if (IndCOF != NbItems && IndCOF != 1)
                {
                    ProVec = CrossProd(Line.Value(IndCOF - 1), Line.Value(IndCOF + 1), out _);
                    if ((ProVec) * SideRef > 0)
                    {
                        ConnexionFather.IndexItemOnSecond(2 * NbItems - IndCOF);
                    }
                }
            }
            else if (Side(CC, Line) != SideRef)
            {
                Curve = (Geom2d_TrimmedCurve)Line.Value(IndCOF);
                ConnexionFather.IndexItemOnSecond(2 * NbItems - IndCOF);
                ConnexionFather.ParameterOnSecond(Curve.ReversedParameter(ConnexionFather.ParameterOnSecond()));
            }
        }

        //-------------------------------------
        // Removal of the re-entrant corners.
        //-------------------------------------
        int IndLine = 1;
        int ICorres = 1;
        NCollection_Array1<int> Corres = new(1, Line.Length());

        while (Line.Value(IndLine) != Line.Last())
        {
            Corres[ICorres] = IndLine;
            Type = Line.Value(IndLine).GetType();
            if (Type == typeof(Geom2d_CartesianPoint) && ICorres != 1 && ICorres != NbItems)
            {
                if (!IsSharpCorner(Line.Value(IndLine - 1), Line.Value(IndLine + 1), SideRef))
                {
                    Line.Remove(IndLine);
                    IndLine--;
                    Corres[ICorres] = 0;
                }
            }
            IndLine++;
            ICorres++;
        }
        Corres[ICorres] = IndLine;

        if (!myIsOpenResult)
        {
            for (i = 1; i < 2 * NbItems - 2; i++)
            {
                if (Corres[i] == 0)
                {
                    Corres[i] = Corres[2 * NbItems - i];
                }
            }

            //----------------------------
            // The connexions.
            //----------------------------
            for (i = 1; i <= NbConnexions; i++)
            {
                CC = ConnexionFrom.ChangeValue(i);
                CC.IndexItemOnFirst(Corres[CC.IndexItemOnFirst()]);
            }

            if (ConnexionFather is not null)
            {
                ConnexionFather.IndexItemOnSecond(Corres[ConnexionFather.IndexItemOnSecond()]);
            }
        }
    }

    private void InsertCorner(NCollection_Sequence<Geom2d_Geometry> Line)
    {
        int i, isuiv;
        Geom2d_TrimmedCurve Curve;
        bool Insert;

        for (i = 1; i <= Line.Length(); i++)
        {
            isuiv = (i == Line.Length()) ? 1 : i + 1;
            Insert = IsSharpCorner(Line.Value(i), Line.Value(isuiv), direction);

            if (Insert)
            {
                Curve = (Geom2d_TrimmedCurve)Line.Value(isuiv);
                Line.InsertAfter(i, new Geom2d_CartesianPoint(Curve.StartPoint()));
                i++;
            }
        }
    }

    /// <summary>The number of items.</summary>
    public int NumberOfItems() => geomElements.Length();

    /// <summary>The number of items on the line I.</summary>
    public int LineLength(int I) => linesLength.Value(I);

    /// <summary>The item of index Index.</summary>
    public Geom2d_Geometry Value(int Index) => geomElements.Value(Index);

    /// <summary>The indices of the items of the circuit made from the curve IndCurve of the line IndLine of the
    /// initial figure.</summary>
    public NCollection_Sequence<int> RefToEqui(int IndLine, int IndCurve)
    {
        MAT2d_BiInt Key = new(IndLine, IndCurve);
        return linkRefEqui.Find(Key);
    }

    private void SortRefToEqui(in MAT2d_BiInt BiRef)
    {
        int i;
        NCollection_Sequence<int> S = linkRefEqui.ChangeFind(BiRef);
        NCollection_Sequence<int> SFin;

        for (i = 1; i <= S.Length(); i++)
        {
            if (!ConnexionOn(S.Value(i)))
            {
                break;
            }
        }
        if (i > 1 && i <= S.Length())
        {
            SFin = new NCollection_Sequence<int>(S);
            SFin.Split(i, S);
            S.Append(SFin);
        }
    }

    /// <summary>The connexion on the item I.</summary>
    public MAT2d_Connexion Connexion(int I) => connexionMap.Find(I);

    /// <summary>True if there is a connexion on the item I.</summary>
    public bool ConnexionOn(int I) => connexionMap.IsBound(I);

    private static double Side(MAT2d_Connexion C1, NCollection_Sequence<Geom2d_Geometry> Line)
    {
        Geom2d_TrimmedCurve Curve;

        gp_Vec2d Vect1 = new(C1.PointOnSecond().X() - C1.PointOnFirst().X(),
            C1.PointOnSecond().Y() - C1.PointOnFirst().Y());
        Curve = (Geom2d_TrimmedCurve)Line.Value(C1.IndexItemOnFirst());
        gp_Vec2d Vect2 = Curve.DN(C1.ParameterOnFirst(), 1);
        if ((Vect1 ^ Vect2) > 0.0)
        {
            return -1.0;
        }
        else
        {
            return 1.0;
        }
    }

    private bool PassByLast(MAT2d_Connexion C1, MAT2d_Connexion C2)
    {
        if (C2.IndexFirstLine() == C1.IndexSecondLine())
        {
            if (C2.IndexItemOnFirst() < C1.IndexItemOnSecond())
            {
                return true;
            }
            else if (C2.IndexItemOnFirst() == C1.IndexItemOnSecond())
            {
                if (C1.IndexFirstLine() == C2.IndexSecondLine())
                {
                    return true;
                }
                if (C2.ParameterOnFirst() == C1.ParameterOnSecond())
                {
                    gp_Vec2d Vect1 = new(C1.PointOnSecond(), C1.PointOnFirst());
                    gp_Vec2d Vect2 = new(C2.PointOnFirst(), C2.PointOnSecond());
                    if ((Vect1 ^ Vect2) * direction > 0)
                    {
                        return true;
                    }
                }
                else if (C2.ParameterOnFirst() < C1.ParameterOnSecond())
                {
                    return true;
                }
            }
        }
        return false;
    }

    private void UpDateLink(int IFirst, int ILine, int ICurveFirst, int ICurveLast)
    {
        int IEqui = IFirst;
        int i;

        for (i = ICurveFirst; i <= ICurveLast; i++)
        {
            MAT2d_BiInt Key = new(ILine, i);
            if (linkRefEqui.IsBound(Key))
            {
                linkRefEqui[Key].Append(IEqui);
            }
            else
            {
                NCollection_Sequence<int> L = new();
                linkRefEqui.Bind(Key, L);
                linkRefEqui[Key].Append(IEqui);
            }
            IEqui++;
        }
    }

    // The cross and dot products of the tangent directions at the end of Geom1 and at the start of Geom2 (both
    // curves).
    private static double CrossProd(Geom2d_Geometry Geom1, Geom2d_Geometry Geom2, out double DotProd)
    {
        Geom2d_TrimmedCurve Curve;

        Curve = (Geom2d_TrimmedCurve)Geom1;
        gp_Dir2d Dir1 = new(Curve.DN(Curve.LastParameter(), 1));
        Curve = (Geom2d_TrimmedCurve)Geom2;
        gp_Dir2d Dir2 = new(Curve.DN(Curve.FirstParameter(), 1));
        DotProd = Dir1.Dot(Dir2);
        return Dir1 ^ Dir2;
    }
}
