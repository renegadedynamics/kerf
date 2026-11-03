// Port of: src/ModelingAlgorithms/TKGeomAlgo/HatchGen/HatchGen_PointOnElement.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/HatchGen/HatchGen_PointOnElement.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/HatchGen/HatchGen_PointOnElement.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.HatchGen_IntersectionType;
using static Occt.IntRes2d_Position;
using static Occt.IntRes2d_Situation;
using static Occt.IntRes2d_TypeTrans;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Occt;

/// <summary>An intersection point on an element of a hatcher, with the type of the intersection there.</summary>
/// <remarks>A value class of OCCT, copied where OCCT copies it (decision 664); its setters are internal.</remarks>
public sealed class HatchGen_PointOnElement : HatchGen_IntersectionPoint
{
    private HatchGen_IntersectionType myType;

    /// <summary>An empty point on element.</summary>
    public HatchGen_PointOnElement()
    {
        myType = HatchGen_UNDETERMINED;
    }

    /// <summary>A copy of <paramref name="Other"/>.</summary>
    public HatchGen_PointOnElement(HatchGen_PointOnElement Other)
        : base(Other)
    {
        myType = Other.myType;
    }

    /// <summary>
    /// The point on the element (the second curve of <paramref name="Point"/>), with the states before and after it
    /// along the hatching (the first curve).
    /// </summary>
    public HatchGen_PointOnElement(in IntRes2d_IntersectionPoint Point)
    {
        myType = HatchGen_UNDETERMINED;
        IntRes2d_Transition TrsH = Point.TransitionOfFirst();
        IntRes2d_Transition TrsE = Point.TransitionOfSecond();

        myIndex = 0;

        myParam = Point.ParamOnSecond();

        switch (TrsE.PositionOnCurve())
        {
            case IntRes2d_Head:
                myPosit = Forward;
                break;
            case IntRes2d_Middle:
                myPosit = Internal;
                break;
            case IntRes2d_End:
                myPosit = Reversed;
                break;
        }

        switch (TrsH.TransitionType())
        {
            case IntRes2d_In:
            {
                myBefore = Out;
                myAfter = In;
                myType = (myPosit == Internal) ? HatchGen_TRUE : HatchGen_TOUCH;
                break;
            }
            case IntRes2d_Out:
            {
                myBefore = In;
                myAfter = Out;
                myType = (myPosit == Internal) ? HatchGen_TRUE : HatchGen_TOUCH;
                break;
            }
            //  Modified by Sergey KHROMOV - Fri Jan  5 12:07:34 2001 Begin
            case IntRes2d_Touch:
            {
                switch (TrsH.Situation())
                {
                    case IntRes2d_Inside:
                    {
                        myType = HatchGen_TANGENT;
                        switch (myPosit)
                        {
                            case Forward:
                            {
                                if (TrsE.IsOpposite())
                                {
                                    myBefore = In;
                                    myAfter = Out;
                                }
                                else
                                {
                                    myBefore = Out;
                                    myAfter = In;
                                }
                                break;
                            }
                            case Internal:
                            {
                                myBefore = In;
                                myAfter = In;
                                break;
                            }
                            case Reversed:
                            {
                                if (TrsE.IsOpposite())
                                {
                                    myBefore = Out;
                                    myAfter = In;
                                }
                                else
                                {
                                    myBefore = In;
                                    myAfter = Out;
                                }
                                break;
                            }
                            case External:
                                break;
                        }
                        break;
                    }
                    case IntRes2d_Outside:
                    {
                        myType = HatchGen_TANGENT;
                        switch (myPosit)
                        {
                            case Forward:
                            {
                                if (TrsE.IsOpposite())
                                {
                                    myBefore = Out;
                                    myAfter = In;
                                }
                                else
                                {
                                    myBefore = In;
                                    myAfter = Out;
                                }
                                break;
                            }
                            case Internal:
                            {
                                myBefore = Out;
                                myAfter = Out;
                                break;
                            }
                            case Reversed:
                            {
                                if (TrsE.IsOpposite())
                                {
                                    myBefore = In;
                                    myAfter = Out;
                                }
                                else
                                {
                                    myBefore = Out;
                                    myAfter = In;
                                }
                                break;
                            }
                            case External:
                                break;
                        }
                        break;
                    }
                    case IntRes2d_Unknown:
                    {
                        myBefore = Unknown;
                        myAfter = Unknown;
                        myType = HatchGen_TANGENT;
                        break;
                    }
                }
                break;
            }
            //  Modified by Sergey KHROMOV - Fri Jan  5 12:07:46 2001 End
            case IntRes2d_Undecided:
            {
                myBefore = Unknown;
                myAfter = Unknown;
                myType = HatchGen_UNDETERMINED;
                break;
            }
        }
        mySegBeg = false;
        mySegEnd = false;
    }

    /// <summary>Sets the intersection type at this point.</summary>
    internal void SetIntersectionType(HatchGen_IntersectionType Type)
    {
        myType = Type;
    }

    /// <summary>The intersection type at this point.</summary>
    public HatchGen_IntersectionType IntersectionType() => myType;

    /// <summary>
    /// True if the point is identical to <paramref name="Point"/>: the same index, position, type, states and segment
    /// flags, and parameters within <paramref name="Confusion"/>.
    /// </summary>
    public bool IsIdentical(HatchGen_PointOnElement Point, double Confusion)
    {
        double Delta = Math.Abs(myParam - Point.myParam);
        return ((Delta <= Confusion) && (myIndex == Point.myIndex) && (myPosit == Point.myPosit)
                && (myType == Point.myType) && (myBefore == Point.myBefore) && (myAfter == Point.myAfter)
                && (mySegBeg == Point.mySegBeg) && (mySegEnd == Point.mySegEnd));
    }

    /// <summary>True if the point is different from <paramref name="Point"/> (not <see cref="IsIdentical"/>).</summary>
    public bool IsDifferent(HatchGen_PointOnElement Point, double Confusion)
    {
        double Delta = Math.Abs(myParam - Point.myParam);
        return ((Delta > Confusion) || (myIndex != Point.myIndex) || (myPosit != Point.myPosit)
                || (myType != Point.myType) || (myBefore != Point.myBefore) || (myAfter != Point.myAfter)
                || (mySegBeg != Point.mySegBeg) || (mySegEnd != Point.mySegEnd));
    }

    /// <summary>Writes the point to <paramref name="theOS"/>, as OCCT writes it to std::cout.</summary>
    public override void Dump(TextWriter theOS, int Index = 0)
    {
        theOS.Write("    --- Point on element ");
        if (Index > 0)
        {
            theOS.Write($"# {Index,3} ");
        }
        else
        {
            theOS.Write("------");
        }
        theOS.Write("---------------\n");
        theOS.Write($"        Index of the element = {myIndex}\n");
        theOS.Write($"        Parameter on element = {Number(myParam)}\n");
        theOS.Write($"        Position  on element = {PositionName(myPosit)}\n");
        theOS.Write("        Intersection Type    = ");
        switch (myType)
        {
            case HatchGen_TRUE:
                theOS.Write("TRUE");
                break;
            case HatchGen_TOUCH:
                theOS.Write("TOUCH");
                break;
            case HatchGen_TANGENT:
                theOS.Write("TANGENT");
                break;
            case HatchGen_UNDETERMINED:
                theOS.Write("UNDETERMINED");
                break;
        }
        theOS.Write('\n');
        theOS.Write($"        State Before         = {StateName(myBefore)}\n");
        theOS.Write($"        State After          = {StateName(myAfter)}\n");
        theOS.Write($"        Beginning of segment = {(mySegBeg ? "TRUE" : "FALSE")}\n");
        theOS.Write($"        End       of segment = {(mySegEnd ? "TRUE" : "FALSE")}\n");
        theOS.Write("    ------------------------------------------\n");
    }
}
