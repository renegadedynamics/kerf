// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_PolyHLRToShape.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_PolyHLRToShape.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_PolyHLRToShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Library;
using Kerf.Engine.Geometry;
using Kerf.Engine.HiddenLines.Algorithm;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.HiddenLines;

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
public class PolyHlrToShape
{
    private PolyAlgo? _algo;
    private readonly NCollection_List<BiPnt2D> _biPntVis = new();
    private readonly NCollection_List<BiPnt2D> _biPntHid = new();
    private bool _hideMode;

    public PolyHlrToShape()
    {
    }

    /// <summary>Hides the segments of A and keeps their visible and hidden parts.</summary>
    public void Update(PolyAlgo a)
    {
        _algo = a;
        _hideMode = true;
        double sta, end;
        float tolsta, tolend;
        EdgeIterator it = new();
        _biPntVis.Clear();
        _biPntHid.Clear();
        Shape s;
        bool reg1, regn, outl, intl;
        Transform3D t = _algo.Projector().Transformation();
        EdgeStatus status = new();

        for (_algo.InitHide(); _algo.MoreHide(); _algo.NextHide())
        {
            ref AlgoBiPoint.PointsT points = ref _algo.Hide(status, out s, out reg1, out regn, out outl, out intl);
            Xyz aSta = points.Pnt1;
            Xyz aEnd = points.Pnt2;
            t.Transforms(ref aSta);
            t.Transforms(ref aEnd);
            Xy sta2D = new(aSta.X(), aSta.Y());
            Xy end2D = new(aEnd.X(), aEnd.Y());
            Xy d = end2D - sta2D;
            if (d.Modulus() > 1.0e-10)
            {
                for (it.InitVisible(status); it.MoreVisible(); it.NextVisible())
                {
                    it.Visible(out sta, out tolsta, out end, out tolend);
                    _biPntVis.Append(new BiPnt2D(sta2D + sta * d, sta2D + end * d, s, reg1, regn, outl, intl));
                }

                for (it.InitHidden(status); it.MoreHidden(); it.NextHidden())
                {
                    it.Hidden(out sta, out tolsta, out end, out tolend);
                    _biPntHid.Append(new BiPnt2D(sta2D + sta * d, sta2D + end * d, s, reg1, regn, outl, intl));
                }
            }
        }
    }

    /// <summary>Switches to the segments without hiding.</summary>
    public void Show()
    {
        _hideMode = false;
    }

    /// <summary>Switches to the hidden-line results (the default).</summary>
    public void Hide()
    {
        _hideMode = true;
    }

    /// <summary>The visible sharp edges.</summary>
    public Shape VCompound() => InternalCompound(4, true, new Shape());

    public Shape VCompound(in Shape s) => InternalCompound(4, true, s);

    /// <summary>The visible smooth edges.</summary>
    public Shape Rg1LineVCompound() => InternalCompound(2, true, new Shape());

    public Shape Rg1LineVCompound(in Shape s) => InternalCompound(2, true, s);

    /// <summary>The visible sewn edges.</summary>
    public Shape RgNLineVCompound() => InternalCompound(3, true, new Shape());

    public Shape RgNLineVCompound(in Shape s) => InternalCompound(3, true, s);

    /// <summary>The visible outlines.</summary>
    public Shape OutLineVCompound() => InternalCompound(1, true, new Shape());

    public Shape OutLineVCompound(in Shape s) => InternalCompound(1, true, s);

    /// <summary>The hidden sharp edges.</summary>
    public Shape HCompound() => InternalCompound(4, false, new Shape());

    public Shape HCompound(in Shape s) => InternalCompound(4, false, s);

    /// <summary>The hidden smooth edges.</summary>
    public Shape Rg1LineHCompound() => InternalCompound(2, false, new Shape());

    public Shape Rg1LineHCompound(in Shape s) => InternalCompound(2, false, s);

    /// <summary>The hidden sewn edges.</summary>
    public Shape RgNLineHCompound() => InternalCompound(3, false, new Shape());

    public Shape RgNLineHCompound(in Shape s) => InternalCompound(3, false, s);

    /// <summary>The hidden outlines.</summary>
    public Shape OutLineHCompound() => InternalCompound(1, false, new Shape());

    public Shape OutLineHCompound(in Shape s) => InternalCompound(1, false, s);

    private Shape InternalCompound(int typ, bool visible, in Shape s)
    {
        HashSet<Shape> map = new(ShapeHasher.Instance);
        if (!s.IsNull)
        {
            Explorer ex = new();
            for (ex.Init(s, ShapeType.Edge); ex.More(); ex.Next())
            {
                map.Add(ex.Current());
            }
            for (ex.Init(s, ShapeType.Face); ex.More(); ex.Next())
            {
                map.Add(ex.Current());
            }
        }
        bool todraw, reg1, regn, outl, intl;
        bool added = false;
        Shape result;
        BrepBuilder b = new();
        Compound compound = new();
        b.MakeCompound(ref compound);
        result = compound;

        if (_hideMode)
        {
            NCollection_List<BiPnt2D>.Iterator it = new();
            if (visible)
            {
                it.Initialize(_biPntVis);
            }
            else
            {
                it.Initialize(_biPntHid);
            }

            for (; it.More(); it.Next())
            {
                BiPnt2D bp = it.Value();
                reg1 = bp.Rg1Line();
                regn = bp.RgNLine();
                outl = bp.OutLine();
                intl = bp.IntLine();
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
                    if (!s.IsNull)
                    {
                        todraw = map.Contains(bp.Shape());
                    }
                }
                if (todraw)
                {
                    Point2D firstP2d = bp.P1();
                    Point2D lastP2d = bp.P2();
                    if (firstP2d.SquareDistanceTo(lastP2d) > 1.0e-20)
                    {
                        b.Add(result, new LibraryMakeEdge2d(bp.P1(), bp.P2()).InternalEdge());
                        added = true;
                    }
                }
            }
        }
        else
        {
            Transform3D t = _algo!.Projector().Transformation();
            Shape sbp;

            for (_algo.InitShow(); _algo.MoreShow(); _algo.NextShow())
            {
                ref AlgoBiPoint.PointsT points = ref _algo.Show(out sbp, out reg1, out regn, out outl, out intl);
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
                    if (!s.IsNull)
                    {
                        todraw = map.Contains(sbp);
                    }
                }
                if (todraw)
                {
                    Xyz sta = points.Pnt1, end = points.Pnt2;
                    t.Transforms(ref sta);
                    t.Transforms(ref end);
                    Xy sta2D = new(sta.X(), sta.Y());
                    Xy end2D = new(end.X(), end.Y());
                    Xy d = end2D - sta2D;
                    if (d.SquareModulus() > 1.0e-20)
                    {
                        b.Add(result, new LibraryMakeEdge2d(new Point2D(sta2D), new Point2D(end2D)).InternalEdge());
                        added = true;
                    }
                }
            }
        }
        if (!added)
        {
            result = new Shape();
        }
        result.TShape()?.FreezeGraph();
        return result;
    }
}
