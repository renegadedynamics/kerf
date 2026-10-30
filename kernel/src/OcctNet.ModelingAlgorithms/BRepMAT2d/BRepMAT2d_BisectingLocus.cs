// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepMAT2d/BRepMAT2d_BisectingLocus.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepMAT2d/BRepMAT2d_BisectingLocus.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_JoinType;
using static Occt.MAT_Side;

namespace Occt;

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
public sealed class BRepMAT2d_BisectingLocus
{
    private MAT_Graph? theGraph;
    private readonly MAT2d_Tool2d theTool = new();
    private bool isDone;
    private readonly NCollection_DataMap<MAT2d_BiInt, int> nbSect = new();
    private int nbContours;

    public BRepMAT2d_BisectingLocus()
    {
        isDone = false;
        nbContours = 0;
    }

    /// <summary>Computes the bisecting locus of the contours of anExplo, on the side aSide of its contour
    /// IndexLine.</summary>
    public void Compute(BRepMAT2d_Explorer anExplo, int IndexLine = 1, MAT_Side aSide = MAT_Left,
        GeomAbs_JoinType aJoinType = GeomAbs_Arc, bool IsOpenResult = false)
    {
        MAT2d_Mat2d TheMAT = new(IsOpenResult);
        MAT_ListOfBisector TheRoots = new();
        NCollection_Sequence<NCollection_Sequence<Geom2d_Geometry>> Figure = new();
        int i;

        nbSect.Clear();
        theGraph = new MAT_Graph();
        nbContours = anExplo.NumberOfContours();
        if (nbContours == 0)
        {
            return;
        }

        // The data of anExplo.
        for (i = 1; i <= anExplo.NumberOfContours(); i++)
        {
            NCollection_Sequence<Geom2d_Geometry> Line = new();
            Figure.Append(Line);
            for (anExplo.Init(i); anExplo.More(); anExplo.Next())
            {
                Figure.ChangeValue(i).Append(anExplo.Value());
            }
        }

        // The curves are cut.
        CutSketch(Figure, nbSect);

        // The circuit on which the map is computed.
        MAT2d_Circuit ACircuit = new(aJoinType, IsOpenResult);
        ACircuit.Perform(Figure, anExplo.GetIsClosed(), IndexLine, (aSide == MAT_Left));

        // The tool.
        theTool.Sense(aSide);
        theTool.SetJoinType(aJoinType);
        theTool.InitItems(ACircuit);

        // The algorithm.
        if (IsOpenResult)
        {
            TheMAT.CreateMatOpen(theTool);
        }
        else
        {
            TheMAT.CreateMat(theTool);
        }

        isDone = TheMAT.IsDone();
        if (!isDone)
        {
            return;
        }

        // The graph, from the result of the algorithm.
        for (TheMAT.Init(); TheMAT.More(); TheMAT.Next())
        {
            TheRoots.BackAdd(TheMAT.Bisector());
        }

        theGraph.Perform(TheMAT.SemiInfinite(), TheRoots, theTool.NumberOfItems(), TheMAT.NumberOfBisectors());

        // Merging of the doubled basic elements if there are several lines.
        if (anExplo.NumberOfContours() > 1)
        {
            NCollection_DataMap<int, MAT_BasicElt> NewMap = new();
            int IndexLast = 1;

            // NewMap: the elements ordered along the lines of the contour, without duplicates; the doubled arcs
            // are merged and the nodes updated at the same time.
            for (i = 1; i <= anExplo.NumberOfContours(); i++)
            {
                RenumerationAndFusion(i, theTool.Circuit().LineLength(i), ref IndexLast, NewMap);
            }

            // The new map in the graph, and the arcs and nodes compacted (the holes left by the merged arcs).
            theGraph.ChangeBasicElts(NewMap);
            theGraph.CompactArcs();
            theGraph.CompactNodes();
        }
    }

    // OCCT declares MergeArc1, MergeArc2 and the geometric indices once and reads them after each fusion:
    // FusionOfBasicElts leaves them unchanged when the two elements are one, and MergeArc2 (with its indices) when
    // the element's start and end arcs are one. They keep their previous values here too (false and 0 at first,
    // where OCCT's are uninitialised: occt-issues.md).
    private void RenumerationAndFusion(int ILine, int LengthLine, ref int IndexLast,
        NCollection_DataMap<int, MAT_BasicElt> NewMap)
    {
        int IndFirst;
        int i, j;
        int GeomIndexArc1 = 0, GeomIndexArc2 = 0, GeomIndexArc3 = 0, GeomIndexArc4 = 0;
        bool MergeArc1 = false, MergeArc2 = false; // OCCT: uninitialised

        for (i = 1; i <= LengthLine; i++)
        {
            NCollection_Sequence<int> S = theTool.Circuit().RefToEqui(ILine, i);
            IndFirst = S.Value(1);
            NewMap.Bind(IndexLast, theGraph!.ChangeBasicElt(IndFirst));
            IndexLast++;

            for (j = 2; j <= S.Length(); j++)
            {
                theGraph.FusionOfBasicElts(IndFirst, S.Value(j), ref MergeArc1, ref GeomIndexArc1, ref GeomIndexArc2,
                    ref MergeArc2, ref GeomIndexArc3, ref GeomIndexArc4);
                if (MergeArc1)
                {
                    theTool.BisecFusion(GeomIndexArc1, GeomIndexArc2);
                }
                if (MergeArc2)
                {
                    theTool.BisecFusion(GeomIndexArc3, GeomIndexArc4);
                }
            }
        }
    }

    /// <summary>True if Compute succeeded.</summary>
    public bool IsDone() => isDone;

    /// <summary>The graph of the locus.</summary>
    public MAT_Graph Graph() => theGraph!;

    /// <summary>The number of contours.</summary>
    public int NumberOfContours() => nbContours;

    /// <summary>The number of basic elements on the line IndLine.</summary>
    public int NumberOfElts(int IndLine) => theTool.Circuit().LineLength(IndLine);

    /// <summary>The number of sections of the curve Index of the contour IndLine of the explorer.</summary>
    public int NumberOfSections(int IndLine, int Index)
    {
        MAT2d_BiInt B = new(IndLine, Index);
        return nbSect.Find(B);
    }

    /// <summary>The basic element at the position Index on the contour IndLine (they are sorted along it).
    /// </summary>
    public MAT_BasicElt BasicElt(int IndLine, int Index)
    {
        int i;
        int Ind = Index;

        for (i = 1; i < IndLine; i++)
        {
            Ind = Ind + theTool.Circuit().LineLength(i);
        }

        return theGraph!.BasicElt(Ind);
    }

    /// <summary>The bisector of the arc anArc; Reverse is false when the first node of anArc is the first point of
    /// the curve.</summary>
    public Bisector_Bisec GeomBis(MAT_Arc anArc, out bool Reverse)
    {
        Reverse = false;

        Geom2d_Curve Bis = theTool.GeomBis(anArc.GeomIndex()).Value();

        if (Bis.FirstParameter() <= -Precision.Infinite())
        {
            Reverse = true;
        }
        else if (Bis.LastParameter() < Precision.Infinite())
        {
            gp_Pnt2d PF = Bis.Value(Bis.FirstParameter());
            gp_Pnt2d PL = Bis.Value(Bis.LastParameter());
            gp_Pnt2d PNode = GeomElt(anArc.FirstNode());
            if (PNode.SquareDistance(PF) > PNode.SquareDistance(PL))
            {
                Reverse = true;
            }
        }
        return theTool.GeomBis(anArc.GeomIndex());
    }

    /// <summary>The geometry of the basic element aBasicElt.</summary>
    public Geom2d_Geometry GeomElt(MAT_BasicElt aBasicElt) => theTool.GeomElt(aBasicElt.GeomIndex());

    /// <summary>The point of the node aNode.</summary>
    public gp_Pnt2d GeomElt(MAT_Node aNode) => theTool.GeomPnt(aNode.GeomIndex());

    // Cuts the curves of the figure at their extrema of curvature and inflections; NbSect counts the sections of
    // each initial curve.
    private static void CutSketch(NCollection_Sequence<NCollection_Sequence<Geom2d_Geometry>> Figure,
        NCollection_DataMap<MAT2d_BiInt, int> NbSect)
    {
        MAT2d_CutCurve Cuter = new();
        int i, j, k, ico;
        int ICurveInit;
        int NbSection;

        for (i = 1; i <= Figure.Length(); i++)
        {
            NCollection_Sequence<Geom2d_Geometry> Contour = Figure.ChangeValue(i);
            ICurveInit = 0;

            for (j = 1; j <= Contour.Length(); j++)
            {
                ICurveInit++;
                Cuter.Perform((Geom2d_Curve)Contour.ChangeValue(j));
                NbSection = 1;
                if (!Cuter.UnModified())
                {
                    ico = j;
                    NbSection = Cuter.NbCurves();
                    for (k = 1; k <= NbSection; k++)
                    {
                        Contour.InsertAfter(j, Cuter.Value(k));
                        j++;
                    }
                    Contour.Remove(ico);
                    j--;
                }
                MAT2d_BiInt B = new(i, ICurveInit);
                NbSect.Bind(B, NbSection);
            }
        }
    }
}
