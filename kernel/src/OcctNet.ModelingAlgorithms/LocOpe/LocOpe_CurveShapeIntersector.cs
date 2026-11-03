// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_CurveShapeIntersector.hxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_CurveShapeIntersector.lxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_CurveShapeIntersector.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Occt.IntCurveSurface_TransitionOnCurve;
using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>
/// The intersection between an axis or a circle and the faces of a shape, the points sorted in increasing parameter
/// along the axis or the circle. The transitions take the faces' orientations into account.
/// </summary>
/// <remarks>A working object (decision 21): it only reads the shape; the points' faces are the shape's own.</remarks>
public sealed class LocOpe_CurveShapeIntersector
{
    private bool myDone;
    private readonly NCollection_Sequence<LocOpe_PntFace> myPoints = new();

    public LocOpe_CurveShapeIntersector()
    {
        myDone = false;
    }

    public LocOpe_CurveShapeIntersector(in Axis3D Axis, in TopoDS_Shape S)
    {
        Init(Axis, S);
    }

    public LocOpe_CurveShapeIntersector(in Circle3D C, in TopoDS_Shape S)
    {
        Init(C, S);
    }

    public void Init(in Axis3D Axis, in TopoDS_Shape S)
    {
        myDone = false;
        myPoints.Clear();
        if (S.IsNull())
        {
            return;
        }
        double Tol = global::Kerf.Engine.Foundation.Precision.Confusion();

        BRepIntCurveSurface_Inter theInt = new();
        theInt.Init(S, new Line3D(Axis), Tol);
        Perform(theInt, myPoints);
        myDone = true;
    }

    public void Init(in Circle3D C, in TopoDS_Shape S)
    {
        myDone = false;
        myPoints.Clear();
        if (S.IsNull())
        {
            return;
        }
        double Tol = global::Kerf.Engine.Foundation.Precision.Confusion();

        Geom_Circle GC = new(C);
        GeomAdaptor_Curve AC = new(GC, 0.0, 2.0 * Math.PI);

        BRepIntCurveSurface_Inter theInt = new();
        theInt.Init(S, AC, Tol);

        Perform(theInt, myPoints);
        myDone = true;
    }

    public bool IsDone() => myDone;

    public int NbPoints()
    {
        if (!myDone)
        {
            throw new NotDoneException();
        }
        return myPoints.Length();
    }

    public LocOpe_PntFace Point(int I)
    {
        if (!myDone)
        {
            throw new NotDoneException();
        }
        return myPoints.Value(I);
    }

    /// <summary>
    /// Searches the first intersection point located after the parameter <paramref name="From"/> whose orientation
    /// is not EXTERNAL; see <see cref="LocOpe_CSIntersector"/>. The outputs are left unchanged where OCCT leaves them.
    /// </summary>
    public bool LocalizeAfter(double From, ref TopAbs_Orientation Or, ref int IndFrom, ref int IndTo)
    {
        if (!myDone)
        {
            throw new NotDoneException();
        }
        double Eps = global::Kerf.Engine.Foundation.Precision.Confusion();
        double param, FMEPS = From - Eps;
        int i, ifirst, nbpoints = myPoints.Length();
        for (ifirst = 1; ifirst <= nbpoints; ifirst++)
        {
            if (myPoints.Value(ifirst).Parameter() >= FMEPS)
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
                Or = myPoints.Value(i).Orientation();
                param = myPoints.Value(i).Parameter();
                i = i + 1;
                while (i <= nbpoints)
                {
                    if (myPoints.Value(i).Parameter() - param <= Eps)
                    {
                        if (Or != TopAbs_EXTERNAL && Or != myPoints.Value(i).Orientation())
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
    /// Searches the first intersection point located before the parameter <paramref name="From"/> whose orientation
    /// is not EXTERNAL.
    /// </summary>
    public bool LocalizeBefore(double From, ref TopAbs_Orientation Or, ref int IndFrom, ref int IndTo)
    {
        if (!myDone)
        {
            throw new NotDoneException();
        }
        double Eps = global::Kerf.Engine.Foundation.Precision.Confusion();
        double param, FPEPS = From + Eps;
        int i, ifirst, nbpoints = myPoints.Length();
        for (ifirst = nbpoints; ifirst >= 1; ifirst--)
        {
            if (myPoints.Value(ifirst).Parameter() <= FPEPS)
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
                Or = myPoints.Value(i).Orientation();
                param = myPoints.Value(i).Parameter();
                i = i - 1;
                while (i >= 1)
                {
                    if (param - myPoints.Value(i).Parameter() <= Eps)
                    {
                        if (Or != TopAbs_EXTERNAL && Or != myPoints.Value(i).Orientation())
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

    /// <summary>Searches the first intersection point after the index <paramref name="FromInd"/> whose orientation is not EXTERNAL.</summary>
    public bool LocalizeAfter(int FromInd, ref TopAbs_Orientation Or, ref int IndFrom, ref int IndTo)
    {
        if (!myDone)
        {
            throw new NotDoneException();
        }
        int nbpoints = myPoints.Length();
        if (FromInd >= nbpoints)
        {
            return false;
        }

        double Eps = global::Kerf.Engine.Foundation.Precision.Confusion();
        double param, FMEPS;
        int i, ifirst;
        if (FromInd >= 1)
        {
            FMEPS = myPoints.Value(FromInd).Parameter() - Eps;
            for (ifirst = FromInd + 1; ifirst <= nbpoints; ifirst++)
            {
                if (myPoints.Value(ifirst).Parameter() >= FMEPS)
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
                Or = myPoints.Value(i).Orientation();
                param = myPoints.Value(i).Parameter();
                i = i + 1;
                while (i <= nbpoints)
                {
                    if (myPoints.Value(i).Parameter() - param <= Eps)
                    {
                        if (Or != TopAbs_EXTERNAL && Or != myPoints.Value(i).Orientation())
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

    /// <summary>Searches the first intersection point before the index <paramref name="FromInd"/> whose orientation is not EXTERNAL.</summary>
    public bool LocalizeBefore(int FromInd, ref TopAbs_Orientation Or, ref int IndFrom, ref int IndTo)
    {
        if (!myDone)
        {
            throw new NotDoneException();
        }
        int nbpoints = myPoints.Length();
        if (FromInd <= 1)
        {
            return false;
        }

        double Eps = global::Kerf.Engine.Foundation.Precision.Confusion();
        double param, FPEPS;
        int i, ifirst;
        if (FromInd <= nbpoints)
        {
            FPEPS = myPoints.Value(FromInd).Parameter() + Eps;
            for (ifirst = FromInd - 1; ifirst >= 1; ifirst--)
            {
                if (myPoints.Value(ifirst).Parameter() <= FPEPS)
                {
                    break;
                }
            }
        }
        else
        {
            ifirst = nbpoints;
        }

        bool RetVal = false;
        if (ifirst >= 1)
        {
            i = ifirst;
            IndTo = ifirst;
            bool found = false;
            while (!found)
            {
                Or = myPoints.Value(i).Orientation();
                param = myPoints.Value(i).Parameter();
                i = i - 1;
                while (i >= 1)
                {
                    if (param - myPoints.Value(i).Parameter() <= Eps)
                    {
                        if (Or != TopAbs_EXTERNAL && Or != myPoints.Value(i).Orientation())
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

    private static void Perform(BRepIntCurveSurface_Inter theInt, NCollection_Sequence<LocOpe_PntFace> thePoints)
    {
        double param, paramu, paramv;
        int i, nbpoints = 0;

        TopAbs_Orientation theor = TopAbs_FORWARD, orface;

        while (theInt.More())
        {
            Point3D thept = theInt.Pnt();
            TopoDS_Face theface = theInt.Face();
            orface = theface.Orientation();
            param = theInt.W();
            paramu = theInt.U();
            paramv = theInt.V();

            switch (theInt.Transition())
            {
                case IntCurveSurface_In:
                    if (orface == TopAbs_FORWARD)
                    {
                        theor = TopAbs_FORWARD;
                    }
                    else if (orface == TopAbs_REVERSED)
                    {
                        theor = TopAbs_REVERSED;
                    }
                    else
                    {
                        theor = TopAbs_EXTERNAL;
                    }
                    break;
                case IntCurveSurface_Out:
                    if (orface == TopAbs_FORWARD)
                    {
                        theor = TopAbs_REVERSED;
                    }
                    else if (orface == TopAbs_REVERSED)
                    {
                        theor = TopAbs_FORWARD;
                    }
                    else
                    {
                        theor = TopAbs_EXTERNAL;
                    }
                    break;
                case IntCurveSurface_Tangent:
                    theor = TopAbs_EXTERNAL;
                    break;
            }

            LocOpe_PntFace newpt = new(thept, theface, theor, param, paramu, paramv);

            for (i = 1; i <= nbpoints; i++)
            {
                if (thePoints.Value(i).Parameter() - param > 0.0)
                {
                    break;
                }
            }
            if (i <= nbpoints)
            {
                thePoints.InsertBefore(i, newpt);
            }
            else
            {
                thePoints.Append(newpt);
            }
            nbpoints++;
            theInt.Next();
        }
    }
}
