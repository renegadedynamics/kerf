// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_PolyHLRToShape.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_PolyHLRToShape.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_PolyHLRToShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// The results of a polygonal hidden-line removal (HLRBRep_PolyAlgo, after <c>Update</c>) as compounds of 2D line
/// segments: visible or hidden, of one kind (sharp edges, smooth edges, sewn edges, outlines), for all the shapes or
/// the edges and faces of one shape.
/// </summary>
/// <remarks>
/// <see cref="Update"/> hides the algorithm's segments once (in hide mode) and keeps the visible and hidden parts;
/// <see cref="Show"/> switches to the segments without hiding. Each compound is built anew, published frozen, and is
/// null when it would be empty.
/// </remarks>
public class HLRBRep_PolyHLRToShape
{
    private HLRBRep_PolyAlgo? myAlgo;
    private readonly NCollection_List<HLRBRep_BiPnt2D> myBiPntVis = new();
    private readonly NCollection_List<HLRBRep_BiPnt2D> myBiPntHid = new();
    private bool myHideMode;

    public HLRBRep_PolyHLRToShape()
    {
    }

    /// <summary>Hides the segments of A and keeps their visible and hidden parts.</summary>
    public void Update(HLRBRep_PolyAlgo A)
    {
        myAlgo = A;
        myHideMode = true;
        double sta, end;
        float tolsta, tolend;
        HLRAlgo_EdgeIterator It = new();
        myBiPntVis.Clear();
        myBiPntHid.Clear();
        TopoDS_Shape S;
        bool reg1, regn, outl, intl;
        Transform3D T = myAlgo.Projector().Transformation();
        HLRAlgo_EdgeStatus status = new();

        for (myAlgo.InitHide(); myAlgo.MoreHide(); myAlgo.NextHide())
        {
            ref HLRAlgo_BiPoint.PointsT aPoints = ref myAlgo.Hide(status, out S, out reg1, out regn, out outl, out intl);
            Xyz aSta = aPoints.Pnt1;
            Xyz aEnd = aPoints.Pnt2;
            T.Transforms(ref aSta);
            T.Transforms(ref aEnd);
            Xy aSta2D = new(aSta.X(), aSta.Y());
            Xy aEnd2D = new(aEnd.X(), aEnd.Y());
            Xy aD = aEnd2D - aSta2D;
            if (aD.Modulus() > 1.0e-10)
            {
                for (It.InitVisible(status); It.MoreVisible(); It.NextVisible())
                {
                    It.Visible(out sta, out tolsta, out end, out tolend);
                    myBiPntVis.Append(new HLRBRep_BiPnt2D(aSta2D + sta * aD, aSta2D + end * aD, S, reg1, regn, outl, intl));
                }

                for (It.InitHidden(status); It.MoreHidden(); It.NextHidden())
                {
                    It.Hidden(out sta, out tolsta, out end, out tolend);
                    myBiPntHid.Append(new HLRBRep_BiPnt2D(aSta2D + sta * aD, aSta2D + end * aD, S, reg1, regn, outl, intl));
                }
            }
        }
    }

    /// <summary>Switches to the segments without hiding.</summary>
    public void Show()
    {
        myHideMode = false;
    }

    /// <summary>Switches to the hidden-line results (the default).</summary>
    public void Hide()
    {
        myHideMode = true;
    }

    /// <summary>The visible sharp edges.</summary>
    public TopoDS_Shape VCompound() => InternalCompound(4, true, new TopoDS_Shape());

    public TopoDS_Shape VCompound(in TopoDS_Shape S) => InternalCompound(4, true, S);

    /// <summary>The visible smooth edges.</summary>
    public TopoDS_Shape Rg1LineVCompound() => InternalCompound(2, true, new TopoDS_Shape());

    public TopoDS_Shape Rg1LineVCompound(in TopoDS_Shape S) => InternalCompound(2, true, S);

    /// <summary>The visible sewn edges.</summary>
    public TopoDS_Shape RgNLineVCompound() => InternalCompound(3, true, new TopoDS_Shape());

    public TopoDS_Shape RgNLineVCompound(in TopoDS_Shape S) => InternalCompound(3, true, S);

    /// <summary>The visible outlines.</summary>
    public TopoDS_Shape OutLineVCompound() => InternalCompound(1, true, new TopoDS_Shape());

    public TopoDS_Shape OutLineVCompound(in TopoDS_Shape S) => InternalCompound(1, true, S);

    /// <summary>The hidden sharp edges.</summary>
    public TopoDS_Shape HCompound() => InternalCompound(4, false, new TopoDS_Shape());

    public TopoDS_Shape HCompound(in TopoDS_Shape S) => InternalCompound(4, false, S);

    /// <summary>The hidden smooth edges.</summary>
    public TopoDS_Shape Rg1LineHCompound() => InternalCompound(2, false, new TopoDS_Shape());

    public TopoDS_Shape Rg1LineHCompound(in TopoDS_Shape S) => InternalCompound(2, false, S);

    /// <summary>The hidden sewn edges.</summary>
    public TopoDS_Shape RgNLineHCompound() => InternalCompound(3, false, new TopoDS_Shape());

    public TopoDS_Shape RgNLineHCompound(in TopoDS_Shape S) => InternalCompound(3, false, S);

    /// <summary>The hidden outlines.</summary>
    public TopoDS_Shape OutLineHCompound() => InternalCompound(1, false, new TopoDS_Shape());

    public TopoDS_Shape OutLineHCompound(in TopoDS_Shape S) => InternalCompound(1, false, S);

    private TopoDS_Shape InternalCompound(int typ, bool visible, in TopoDS_Shape S)
    {
        NCollection_Map<TopoDS_Shape> Map = new(TopTools_ShapeMapHasher.Instance);
        if (!S.IsNull())
        {
            TopExp_Explorer ex = new();
            for (ex.Init(S, TopAbs_EDGE); ex.More(); ex.Next())
            {
                Map.Add(ex.Current());
            }
            for (ex.Init(S, TopAbs_FACE); ex.More(); ex.Next())
            {
                Map.Add(ex.Current());
            }
        }
        bool todraw, reg1, regn, outl, intl;
        bool added = false;
        TopoDS_Shape Result;
        BRep_Builder B = new();
        TopoDS_Compound aCompound = new();
        B.MakeCompound(ref aCompound);
        Result = aCompound;

        if (myHideMode)
        {
            NCollection_List<HLRBRep_BiPnt2D>.Iterator It = new();
            if (visible)
            {
                It.Initialize(myBiPntVis);
            }
            else
            {
                It.Initialize(myBiPntHid);
            }

            for (; It.More(); It.Next())
            {
                HLRBRep_BiPnt2D BP = It.Value();
                reg1 = BP.Rg1Line();
                regn = BP.RgNLine();
                outl = BP.OutLine();
                intl = BP.IntLine();
                if (typ == 1)
                {
                    todraw = intl;
                }
                else if (typ == 2)
                {
                    todraw = reg1 && !regn && !outl;
                }
                else if (typ == 3)
                {
                    todraw = regn && !outl;
                }
                else
                {
                    todraw = !intl && (!reg1 || outl);
                }
                if (todraw)
                {
                    if (!S.IsNull())
                    {
                        todraw = Map.Contains(BP.Shape());
                    }
                }
                if (todraw)
                {
                    Point2D FirstP2d = BP.P1();
                    Point2D LastP2d = BP.P2();
                    if (FirstP2d.SquareDistance(LastP2d) > 1.0e-20)
                    {
                        B.Add(Result, new BRepLib_MakeEdge2d(BP.P1(), BP.P2()).InternalEdge());
                        added = true;
                    }
                }
            }
        }
        else
        {
            Transform3D T = myAlgo!.Projector().Transformation();
            TopoDS_Shape SBP;

            for (myAlgo.InitShow(); myAlgo.MoreShow(); myAlgo.NextShow())
            {
                ref HLRAlgo_BiPoint.PointsT aPoints = ref myAlgo.Show(out SBP, out reg1, out regn, out outl, out intl);
                if (typ == 1)
                {
                    todraw = intl;
                }
                else if (typ == 2)
                {
                    todraw = reg1 && !regn && !outl;
                }
                else if (typ == 3)
                {
                    todraw = regn && !outl;
                }
                else
                {
                    todraw = !intl && (!reg1 || outl);
                }
                if (todraw)
                {
                    if (!S.IsNull())
                    {
                        todraw = Map.Contains(SBP);
                    }
                }
                if (todraw)
                {
                    Xyz aSta = aPoints.Pnt1, aEnd = aPoints.Pnt2;
                    T.Transforms(ref aSta);
                    T.Transforms(ref aEnd);
                    Xy aSta2D = new(aSta.X(), aSta.Y());
                    Xy aEnd2D = new(aEnd.X(), aEnd.Y());
                    Xy aD = aEnd2D - aSta2D;
                    if (aD.SquareModulus() > 1.0e-20)
                    {
                        B.Add(Result, new BRepLib_MakeEdge2d(new Point2D(aSta2D), new Point2D(aEnd2D)).InternalEdge());
                        added = true;
                    }
                }
            }
        }
        if (!added)
        {
            Result = new TopoDS_Shape();
        }
        Result.TShape()?.FreezeGraph();
        return Result;
    }
}
