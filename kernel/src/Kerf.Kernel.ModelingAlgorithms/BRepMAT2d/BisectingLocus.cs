// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepMAT2d/BRepMAT2d_BisectingLocus.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepMAT2d/BRepMAT2d_BisectingLocus.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Offsets.Bisectors;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.JoinKind;
using static Kerf.Engine.Offsets.MedialAxis.MedialSide;

namespace Kerf.Engine.Offsets.MedialAxis;

/// <summary>
/// The bisecting locus (medial axis) of the contours of a BRepMAT2d_Explorer, as a MAT_Graph: its basic elements
/// are the elements of the contours (curves, cut at their extrema of curvature and inflections, and the points of
/// sharp corners), its arcs the bisectors, its nodes the ends of the arcs.
/// </summary>
/// <remarks>
/// <para>If the contours are closed they cut the plane into areas, and the locus is computed in the area on the
/// side aSide of the contour LineIndex; with open contours only it covers the plane. The closed contours must be
/// oriented compatibly, lie in the area, and not cross themselves or each other.</para>
/// <para>A working object. GeomBis hands out the tool's Bisector_Bisec (OCCT returns a copy sharing the same curve):
/// its curve is the working curve the algorithm trimmed, unfrozen, with internal mutators only.</para>
/// </remarks>
internal sealed class BisectingLocus
{
    private MedialGraph? _graph;
    private readonly Tool2d _tool = new();
    private bool _isDone;
    private readonly Dictionary<BiInt, int> _nbSect = new();
    private int _nbContours;

    public BisectingLocus()
    {
        _isDone = false;
        _nbContours = 0;
    }

    /// <summary>Computes the bisecting locus of the contours of anExplo, on the side aSide of its contour
    /// IndexLine.</summary>
    public void Compute(MedialAxisExplorer anExplo, int indexLine = 1, MedialSide aSide = MedialSide.Left,
        JoinKind aJoinType = Arc, bool isOpenResult = false)
    {
        Mat2d theMAT = new(isOpenResult);
        MedialListOfBisector theRoots = new();
        NCollection_Sequence<NCollection_Sequence<Geometry2D>> figure = new();
        int i;

        _nbSect.Clear();
        _graph = new MedialGraph();
        _nbContours = anExplo.NumberOfContours();
        if (_nbContours == 0)
        {
            return;
        }

        // The data of anExplo.
        for (i = 1; i <= anExplo.NumberOfContours(); i++)
        {
            NCollection_Sequence<Geometry2D> line = new();
            figure.Append(line);
            for (anExplo.Init(i); anExplo.More(); anExplo.Next())
            {
                figure.ChangeValue(i).Append(anExplo.Value());
            }
        }

        // The curves are cut.
        CutSketch(figure, _nbSect);

        // The circuit on which the map is computed.
        Circuit aCircuit = new(aJoinType, isOpenResult);
        aCircuit.Perform(figure, anExplo.GetIsClosed(), indexLine, (aSide == MedialSide.Left));

        // The tool.
        _tool.Sense(aSide);
        _tool.SetJoinType(aJoinType);
        _tool.InitItems(aCircuit);

        // The algorithm.
        if (isOpenResult)
        {
            theMAT.CreateMatOpen(_tool);
        }
        else
        {
            theMAT.CreateMat(_tool);
        }

        _isDone = theMAT.IsDone;
        if (!_isDone)
        {
            return;
        }

        // The graph, from the result of the algorithm.
        for (theMAT.Init(); theMAT.More(); theMAT.Next())
        {
            theRoots.BackAdd(theMAT.Bisector());
        }

        _graph.Perform(theMAT.SemiInfinite(), theRoots, _tool.NumberOfItems(), theMAT.NumberOfBisectors());

        // Merging of the doubled basic elements if there are several lines.
        if (anExplo.NumberOfContours() > 1)
        {
            Dictionary<int, MedialBasicElt> newMap = new();
            int indexLast = 1;

            // NewMap: the elements ordered along the lines of the contour, without duplicates; the doubled arcs
            // are merged and the nodes updated at the same time.
            for (i = 1; i <= anExplo.NumberOfContours(); i++)
            {
                RenumerationAndFusion(i, _tool.Circuit().LineLength(i), ref indexLast, newMap);
            }

            // The new map in the graph, and the arcs and nodes compacted (the holes left by the merged arcs).
            _graph.ChangeBasicElts(newMap);
            _graph.CompactArcs();
            _graph.CompactNodes();
        }
    }

    // OCCT declares MergeArc1, MergeArc2 and the geometric indices once and reads them after each fusion:
    // FusionOfBasicElts leaves them unchanged when the two elements are one, and MergeArc2 (with its indices) when
    // the element's start and end arcs are one. They keep their previous values here too (false and 0 at first,
    // where OCCT's are uninitialised: occt-issues.md).
    private void RenumerationAndFusion(int iLine, int lengthLine, ref int indexLast,
        Dictionary<int, MedialBasicElt> newMap)
    {
        int indFirst;
        int i, j;
        int geomIndexArc1 = 0, geomIndexArc2 = 0, geomIndexArc3 = 0, geomIndexArc4 = 0;
        bool mergeArc1 = false, mergeArc2 = false; // OCCT: uninitialised

        for (i = 1; i <= lengthLine; i++)
        {
            NCollection_Sequence<int> s = _tool.Circuit().RefToEqui(iLine, i);
            indFirst = s.Value(1);
            newMap[indexLast] = _graph!.ChangeBasicElt(indFirst);
            indexLast++;

            for (j = 2; j <= s.Length(); j++)
            {
                _graph.FusionOfBasicElts(indFirst, s.Value(j), ref mergeArc1, ref geomIndexArc1, ref geomIndexArc2,
                    ref mergeArc2, ref geomIndexArc3, ref geomIndexArc4);
                if (mergeArc1)
                {
                    _tool.BisecFusion(geomIndexArc1, geomIndexArc2);
                }
                if (mergeArc2)
                {
                    _tool.BisecFusion(geomIndexArc3, geomIndexArc4);
                }
            }
        }
    }

    /// <summary>True if Compute succeeded.</summary>
    public bool IsDone => _isDone;

    /// <summary>The graph of the locus.</summary>
    public MedialGraph Graph() => _graph!;

    /// <summary>The number of contours.</summary>
    public int NumberOfContours() => _nbContours;

    /// <summary>The number of basic elements on the line IndLine.</summary>
    public int NumberOfElts(int indLine) => _tool.Circuit().LineLength(indLine);

    /// <summary>The number of sections of the curve Index of the contour IndLine of the explorer.</summary>
    public int NumberOfSections(int indLine, int index)
    {
        BiInt b = new(indLine, index);
        return _nbSect.GetValue(b);
    }

    /// <summary>The basic element at the position Index on the contour IndLine (they are sorted along it).
    /// </summary>
    public MedialBasicElt BasicElt(int indLine, int index)
    {
        int i;
        int ind = index;

        for (i = 1; i < indLine; i++)
        {
            ind = ind + _tool.Circuit().LineLength(i);
        }

        return _graph!.BasicElt(ind);
    }

    /// <summary>The bisector of the arc anArc; Reverse is false when the first node of anArc is the first point of
    /// the curve.</summary>
    public Bisec GeomBis(MedialArc anArc, out bool reverse)
    {
        reverse = false;

        Curve2D bis = _tool.GeomBis(anArc.GeomIndex()).Value();

        if (bis.FirstParameter <= -global::Kerf.Engine.Foundation.Precision.Infinite())
        {
            reverse = true;
        }
        else if (bis.LastParameter < global::Kerf.Engine.Foundation.Precision.Infinite())
        {
            Point2D pf = bis.Value(bis.FirstParameter);
            Point2D pl = bis.Value(bis.LastParameter);
            Point2D pNode = GeomElt(anArc.FirstNode());
            if (pNode.SquareDistanceTo(pf) > pNode.SquareDistanceTo(pl))
            {
                reverse = true;
            }
        }
        return _tool.GeomBis(anArc.GeomIndex());
    }

    /// <summary>The geometry of the basic element aBasicElt.</summary>
    public Geometry2D GeomElt(MedialBasicElt aBasicElt) => _tool.GeomElt(aBasicElt.GeomIndex());

    /// <summary>The point of the node aNode.</summary>
    public Point2D GeomElt(MedialNode aNode) => _tool.GeomPnt(aNode.GeomIndex());

    // Cuts the curves of the figure at their extrema of curvature and inflections; NbSect counts the sections of
    // each initial curve.
    private static void CutSketch(NCollection_Sequence<NCollection_Sequence<Geometry2D>> figure,
        Dictionary<BiInt, int> nbSect)
    {
        CutCurve cuter = new();
        int i, j, k, ico;
        int iCurveInit;
        int nbSection;

        for (i = 1; i <= figure.Length(); i++)
        {
            NCollection_Sequence<Geometry2D> contour = figure.ChangeValue(i);
            iCurveInit = 0;

            for (j = 1; j <= contour.Length(); j++)
            {
                iCurveInit++;
                cuter.Perform((Curve2D)contour.ChangeValue(j));
                nbSection = 1;
                if (!cuter.UnModified())
                {
                    ico = j;
                    nbSection = cuter.CurveCount;
                    for (k = 1; k <= nbSection; k++)
                    {
                        contour.InsertAfter(j, cuter.Value(k));
                        j++;
                    }
                    contour.Remove(ico);
                    j--;
                }
                BiInt b = new(i, iCurveInit);
                nbSect[b] = nbSection;
            }
        }
    }
}
