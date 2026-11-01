// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_CSIntersector.hxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_CSIntersector.lxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_CSIntersector.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.IntCurveSurface_TransitionOnCurve;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// The intersection between a set of axes, circles or curves and the faces of a shape. The intersection points are
/// sorted in increasing parameter along each element.
/// </summary>
/// <remarks>A working object (decision 21): it only reads the shape; the points' faces are the shape's own.</remarks>
public sealed class LocOpe_CSIntersector
{
    private bool myDone;
    private TopoDS_Shape myShape = new();
    private NCollection_Sequence<LocOpe_PntFace>[]? myPoints;
    private int myNbelem;

    /// <summary>Empty constructor.</summary>
    public LocOpe_CSIntersector()
    {
        myDone = false;
        myPoints = null;
        myNbelem = 0;
    }

    /// <summary>Creates the intersection with the shape <paramref name="S"/>.</summary>
    public LocOpe_CSIntersector(in TopoDS_Shape S)
    {
        myDone = false;
        myShape = S;
        myPoints = null;
        myNbelem = 0;
    }

    /// <summary>Initializes the intersection with the shape <paramref name="S"/>.</summary>
    public void Init(in TopoDS_Shape S)
    {
        myDone = false;
        myShape = S;
        myPoints = null;
        myNbelem = 0;
    }

    private void newPoints()
    {
        myPoints = new NCollection_Sequence<LocOpe_PntFace>[myNbelem];
        for (int i = 0; i < myNbelem; i++)
        {
            myPoints[i] = new NCollection_Sequence<LocOpe_PntFace>();
        }
    }

    public void Perform(NCollection_Sequence<gp_Lin> Slin)
    {
        if (myShape.IsNull() || Slin.Length() <= 0)
        {
            throw new Standard_ConstructionError();
        }
        myDone = false;

        myNbelem = Slin.Length();
        newPoints();

        double binf = RealFirst();
        double bsup = RealLast();
        TopExp_Explorer exp = new(myShape, TopAbs_FACE);
        for (; exp.More(); exp.Next())
        {
            TopoDS_Face theface = TopoDS.Face(exp.Current());
            IntCurvesFace_Intersector theInt = new(theface, Precision.PConfusion());
            for (int i = 1; i <= myNbelem; i++)
            {
                theInt.Perform(Slin[i], binf, bsup);
                if (theInt.IsDone())
                {
                    AddPoints(theInt, myPoints![i - 1], theface);
                }
            }
        }
        myDone = true;
    }

    public void Perform(NCollection_Sequence<gp_Circ> Scir)
    {
        if (myShape.IsNull() || Scir.Length() <= 0)
        {
            throw new Standard_ConstructionError();
        }
        myDone = false;

        myNbelem = Scir.Length();
        newPoints();

        TopExp_Explorer exp = new(myShape, TopAbs_FACE);
        GeomAdaptor_Curve HC = new();
        double binf = 0.0;
        double bsup = 2.0 * Math.PI;

        for (; exp.More(); exp.Next())
        {
            TopoDS_Face theface = TopoDS.Face(exp.Current());
            IntCurvesFace_Intersector theInt = new(theface, 0.0);
            for (int i = 1; i <= myNbelem; i++)
            {
                HC.Load(new Geom_Circle(Scir[i]));
                theInt.Perform(HC, binf, bsup);
                if (theInt.IsDone())
                {
                    AddPoints(theInt, myPoints![i - 1], theface);
                }
            }
        }
        myDone = true;
    }

    public void Perform(NCollection_Sequence<Geom_Curve?> Scur)
    {
        if (myShape.IsNull() || Scur.Length() <= 0)
        {
            throw new Standard_ConstructionError();
        }
        myDone = false;

        myNbelem = Scur.Length();
        newPoints();

        TopExp_Explorer exp = new(myShape, TopAbs_FACE);
        GeomAdaptor_Curve HC = new();
        for (; exp.More(); exp.Next())
        {
            TopoDS_Face theface = TopoDS.Face(exp.Current());
            IntCurvesFace_Intersector theInt = new(theface, 0.0);
            for (int i = 1; i <= myNbelem; i++)
            {
                Geom_Curve? aCur = Scur[i];
                if (aCur is null)
                {
                    continue;
                }
                HC.Load(aCur);
                double binf = HC.FirstParameter();
                double bsup = HC.LastParameter();
                theInt.Perform(HC, binf, bsup);
                if (theInt.IsDone())
                {
                    AddPoints(theInt, myPoints![i - 1], theface);
                }
            }
        }
        myDone = true;
    }

    /// <summary>Returns true if the intersection has been done.</summary>
    public bool IsDone() => myDone;

    /// <summary>Returns the number of intersection points on the element of range <paramref name="I"/>.</summary>
    public int NbPoints(int I)
    {
        if (!myDone)
        {
            throw new StdFail_NotDone();
        }
        if (I <= 0 || I > myNbelem)
        {
            throw new Standard_OutOfRange();
        }
        return myPoints![I - 1].Length();
    }

    /// <summary>
    /// Returns the intersection point of range <paramref name="Index"/> on the element of range <paramref name="I"/>,
    /// in increasing order of parameter along the element.
    /// </summary>
    public LocOpe_PntFace Point(int I, int Index)
    {
        if (!myDone)
        {
            throw new StdFail_NotDone();
        }
        if (I <= 0 || I > myNbelem)
        {
            throw new Standard_OutOfRange();
        }
        return myPoints![I - 1].Value(Index);
    }

    public void Destroy()
    {
        myPoints = null;
    }

    /// <summary>
    /// On the element of range <paramref name="I"/>, searches the first intersection point located after the parameter
    /// <paramref name="From"/> whose orientation is not EXTERNAL. If found, returns true; <paramref name="Or"/> is the
    /// orientation of the point, <paramref name="IndFrom"/> and <paramref name="IndTo"/> the interval of indices of the
    /// points it gathers. <paramref name="Tol"/> decides when two parameters are equal. The outputs are left unchanged
    /// where OCCT leaves them.
    /// </summary>
    public bool LocalizeAfter(int I, double From, double Tol, ref TopAbs_Orientation Or, ref int IndFrom, ref int IndTo)
    {
        if (!myDone)
        {
            throw new StdFail_NotDone();
        }
        if (I <= 0 || I > myNbelem)
        {
            throw new Standard_OutOfRange();
        }
        return LocAfter(myPoints![I - 1], From, Tol, ref Or, ref IndFrom, ref IndTo);
    }

    /// <summary>
    /// On the element of range <paramref name="I"/>, searches the first intersection point located before the
    /// parameter <paramref name="From"/> whose orientation is not EXTERNAL (see <see cref="LocalizeAfter(int, double, double, ref TopAbs_Orientation, ref int, ref int)"/>).
    /// </summary>
    public bool LocalizeBefore(int I, double From, double Tol, ref TopAbs_Orientation Or, ref int IndFrom, ref int IndTo)
    {
        if (!myDone)
        {
            throw new StdFail_NotDone();
        }
        if (I <= 0 || I > myNbelem)
        {
            throw new Standard_OutOfRange();
        }
        return LocBefore(myPoints![I - 1], From, Tol, ref Or, ref IndFrom, ref IndTo);
    }

    /// <summary>
    /// On the element of range <paramref name="I"/>, searches the first intersection point after the index
    /// <paramref name="FromInd"/> whose orientation is not EXTERNAL.
    /// </summary>
    public bool LocalizeAfter(int I, int FromInd, double Tol, ref TopAbs_Orientation Or, ref int IndFrom, ref int IndTo)
    {
        if (!myDone)
        {
            throw new StdFail_NotDone();
        }
        if (I <= 0 || I > myNbelem)
        {
            throw new Standard_OutOfRange();
        }
        return LocAfter(myPoints![I - 1], FromInd, Tol, ref Or, ref IndFrom, ref IndTo);
    }

    /// <summary>
    /// On the element of range <paramref name="I"/>, searches the first intersection point before the index
    /// <paramref name="FromInd"/> whose orientation is not EXTERNAL. As in OCCT, the index is taken as a parameter
    /// (occt-issues.md).
    /// </summary>
    public bool LocalizeBefore(int I, int FromInd, double Tol, ref TopAbs_Orientation Or, ref int IndFrom, ref int IndTo)
    {
        if (!myDone)
        {
            throw new StdFail_NotDone();
        }
        if (I <= 0 || I > myNbelem)
        {
            throw new Standard_OutOfRange();
        }
        // OCCT: there is no LocBefore by index; the int converts to the parameter From
        return LocBefore(myPoints![I - 1], (double)FromInd, Tol, ref Or, ref IndFrom, ref IndTo);
    }

    private static bool LocAfter(NCollection_Sequence<LocOpe_PntFace> Spt, double From, double Tol,
                                 ref TopAbs_Orientation Or, ref int IndFrom, ref int IndTo)
    {
        double param, FMEPS = From - Tol;
        int i, ifirst, nbpoints = Spt.Length();
        for (ifirst = 1; ifirst <= nbpoints; ifirst++)
        {
            if (Spt.Value(ifirst).Parameter() >= FMEPS)
            {
                break;
            }
        }
        bool RetVal = false;
        if (ifirst <= nbpoints)
        {
            i = ifirst;
            IndFrom = ifirst;
            bool found = false;
            while (!found)
            {
                Or = Spt.Value(i).Orientation();
                param = Spt.Value(i).Parameter();
                i = i + 1;
                while (i <= nbpoints)
                {
                    if (Spt.Value(i).Parameter() - param <= Tol)
                    {
                        if (Or != TopAbs_EXTERNAL && Or != Spt.Value(i).Orientation())
                        {
                            Or = TopAbs_EXTERNAL;
                        }
                        i++;
                    }
                    else
                    {
                        break;
                    }
                }
                if (Or == TopAbs_EXTERNAL)
                {
                    found = (i > nbpoints);
                    IndFrom = i;
                }
                else
                { // on a une intersection franche
                    IndTo = i - 1;
                    found = true;
                    RetVal = true;
                }
            }
        }

        return RetVal;
    }

    private static bool LocBefore(NCollection_Sequence<LocOpe_PntFace> Spt, double From, double Tol,
                                  ref TopAbs_Orientation Or, ref int IndFrom, ref int IndTo)
    {
        double param, FPEPS = From + Tol;
        int i, ifirst, nbpoints = Spt.Length();
        for (ifirst = nbpoints; ifirst >= 1; ifirst--)
        {
            if (Spt.Value(ifirst).Parameter() <= FPEPS)
            {
                break;
            }
        }
        bool RetVal = false;
        if (ifirst >= 1)
        {
            i = ifirst;
            IndTo = ifirst;
            bool found = false;
            while (!found)
            {
                Or = Spt.Value(i).Orientation();
                param = Spt.Value(i).Parameter();
                i = i - 1;
                while (i >= 1)
                {
                    if (param - Spt.Value(i).Parameter() <= Tol)
                    {
                        if (Or != TopAbs_EXTERNAL && Or != Spt.Value(i).Orientation())
                        {
                            Or = TopAbs_EXTERNAL;
                        }
                        i--;
                    }
                    else
                    {
                        break;
                    }
                }
                if (Or == TopAbs_EXTERNAL)
                {
                    found = (i < 1);
                    IndTo = i;
                }
                else
                { // on a une intersection franche
                    IndFrom = i + 1;
                    found = true;
                    RetVal = true;
                }
            }
        }

        return RetVal;
    }

    private static bool LocAfter(NCollection_Sequence<LocOpe_PntFace> Spt, int FromInd, double Tol,
                                 ref TopAbs_Orientation Or, ref int IndFrom, ref int IndTo)
    {
        int nbpoints = Spt.Length();
        if (FromInd >= nbpoints)
        {
            return false;
        }

        double param, FMEPS;
        int i, ifirst;
        if (FromInd >= 1)
        {
            FMEPS = Spt.Value(FromInd).Parameter() - Tol;
            for (ifirst = FromInd + 1; ifirst <= nbpoints; ifirst++)
            {
                if (Spt.Value(ifirst).Parameter() >= FMEPS)
                {
                    break;
                }
            }
        }
        else
        {
            ifirst = 1;
        }

        bool RetVal = false;
        if (ifirst <= nbpoints)
        {
            i = ifirst;
            IndFrom = ifirst;
            bool found = false;
            while (!found)
            {
                Or = Spt.Value(i).Orientation();
                param = Spt.Value(i).Parameter();
                i = i + 1;
                while (i <= nbpoints)
                {
                    if (Spt.Value(i).Parameter() - param <= Tol)
                    {
                        if (Or != TopAbs_EXTERNAL && Or != Spt.Value(i).Orientation())
                        {
                            Or = TopAbs_EXTERNAL;
                        }
                        i++;
                    }
                    else
                    {
                        break;
                    }
                }
                if (Or == TopAbs_EXTERNAL)
                {
                    found = (i > nbpoints);
                    IndFrom = i;
                }
                else
                { // on a une intersection franche
                    IndTo = i - 1;
                    found = true;
                    RetVal = true;
                }
            }
        }
        return RetVal;
    }

    /// <summary>
    /// Adds the points of <paramref name="theInt"/> on <paramref name="theface"/> to <paramref name="theSeq"/>, sorted
    /// by parameter (shared with LocOpe_CurveShapeIntersector, as in OCCT's two files).
    /// </summary>
    internal static void AddPoints(IntCurvesFace_Intersector theInt, NCollection_Sequence<LocOpe_PntFace> theSeq, in TopoDS_Face theface)
    {
        int nbpoints = theSeq.Length();
        int newpnt = theInt.NbPnt();
        double param, paramu, paramv;
        for (int j = 1; j <= newpnt; j++)
        {
            gp_Pnt thept = theInt.Pnt(j);
            param = theInt.WParameter(j);
            paramu = theInt.UParameter(j);
            paramv = theInt.VParameter(j);

            TopAbs_Orientation theor = TopAbs_FORWARD;

            switch (theInt.Transition(j))
            {
                case IntCurveSurface_In:
                    theor = TopAbs_FORWARD;
                    break;
                case IntCurveSurface_Out:
                    theor = TopAbs_REVERSED;
                    break;
                case IntCurveSurface_Tangent:
                    theor = TopAbs_EXTERNAL;
                    break;
            }
            LocOpe_PntFace newpt = new(thept, theface, theor, param, paramu, paramv);
            int k;
            for (k = 1; k <= nbpoints; k++)
            {
                if (theSeq.Value(k).Parameter() - param > 0.0)
                {
                    break;
                }
            }
            if (k <= nbpoints)
            {
                theSeq.InsertBefore(k, newpt);
            }
            else
            {
                theSeq.Append(newpt);
            }
            nbpoints++;
        }
    }
}
