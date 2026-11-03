// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ClosedEdgeDivide.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ClosedEdgeDivide.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2000-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Computes the splitting value of a closed edge (both vertices the same, not degenerated): the point of its curve
/// farthest from the vertex among 21 samples, if farther than the vertex's tolerance.
/// </summary>
/// <remarks>A working object (decision 21). It only reads the edge.</remarks>
public class ShapeUpgrade_ClosedEdgeDivide : ShapeUpgrade_EdgeDivide
{
    /// <summary>Empty constructor.</summary>
    public ShapeUpgrade_ClosedEdgeDivide()
    {
    }

    /// <summary>Computes the splitting value of the edge; true if it is closed and has to be split.</summary>
    public override bool Compute(in TopoDS_Edge anEdge)
    {
        Clear();
        var sae = new ShapeAnalysis_Edge();
        TopoDS_Vertex V1 = sae.FirstVertex(anEdge);
        TopoDS_Vertex V2 = sae.LastVertex(anEdge);
        if (V1.IsSame(V2) && !BRep_Tool.Degenerated(anEdge))
        {
            const int nbPoints = 23;
            Point3D pntV = BRep_Tool.Pnt(V1);
            double TolV1 = LimitTolerance(BRep_Tool.Tolerance(V1));
            TolV1 = TolV1 * TolV1;
            Geom_Curve? curve3d = BRep_Tool.Curve(anEdge, out double f, out double l);
            myHasCurve3d = curve3d is not null;
            double f2d = 0.0, l2d = 0.0;
            Geom2d_Curve? pcurve1 = null;
            if (!myFace.IsNull())
            { // process free edges
                sae.PCurve(anEdge, myFace, out pcurve1, out f2d, out l2d, false);
            }
            myHasCurve2d = pcurve1 is not null;

            if (myHasCurve3d)
            {
                double maxPar = f, dMax = 0;
                double step = (l - f) / (nbPoints - 1);
                double param = f + step;
                for (int i = 1; i < 23; i++, param += step)
                {
                    Point3D curPnt = curve3d!.Value(param);
                    double dist = pntV.SquareDistance(curPnt);
                    if (dist > dMax)
                    {
                        maxPar = param;
                        dMax = dist;
                    }
                }
                if (dMax > TolV1)
                {
                    ShapeUpgrade_SplitCurve3d theSplit3dTool = GetSplitCurve3dTool()!;
                    theSplit3dTool.Init(curve3d!, f, l);

                    var values = new NCollection_HSequence<double>();
                    values.Append(maxPar);
                    theSplit3dTool.SetSplitValues(values);
                    myKnots3d = theSplit3dTool.SplitValues();

                    if (myHasCurve2d)
                    {
                        ShapeUpgrade_SplitCurve2d theSplit2dTool = GetSplitCurve2dTool()!;
                        theSplit2dTool.Init(pcurve1!, f2d, l2d);
                        myKnots2d = theSplit2dTool.SplitValues();
                    }
                    return true;
                }
                else
                {
                    return false;
                }
            }

            if (myHasCurve2d)
            {
                Geom_Surface surf = BRep_Tool.Surface(myFace)!;

                double maxPar = f2d, dMax = 0;
                double step = (l2d - f2d) / (nbPoints - 1);
                double param = f2d + step;
                for (int i = 1; i < 23; i++, param += step)
                {
                    Point2D p2d = pcurve1!.Value(param);
                    Point3D curPnt = surf.Value(p2d.X(), p2d.Y());
                    double dist = pntV.SquareDistance(curPnt);
                    if (dist > dMax)
                    {
                        maxPar = param;
                        dMax = dist;
                    }
                }
                if (dMax > TolV1)
                {
                    ShapeUpgrade_SplitCurve2d theSplit2dTool = GetSplitCurve2dTool()!;
                    theSplit2dTool.Init(pcurve1!, f2d, l2d);

                    var values = new NCollection_HSequence<double>();
                    values.Append(maxPar);
                    theSplit2dTool.SetSplitValues(values);
                    myKnots2d = theSplit2dTool.SplitValues();
                    return true;
                }
                else
                {
                    return false;
                }
            }

            return false;
        }
        else
        {
            return false;
        }
    }
}
