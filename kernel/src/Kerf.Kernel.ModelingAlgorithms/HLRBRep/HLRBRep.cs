// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Library;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.HiddenLines;

/// <summary>
/// The hidden-line removal's edge builders: a part of an edge's projected curve as a 2D edge, or as a 3D edge on
/// the edge's own curve, and the angle and deflection of the polygonal hidden-line removal.
/// </summary>
/// <remarks>
/// The edges are built unfrozen, for ported code (HLRBRep_HLRToShape freezes the compounds it returns). A 3D edge
/// shares the curve and, at its ends, the vertices of the edge HLRBRep_Curve was loaded with: a caller's edge, or one
/// the outliner split from it.
/// </remarks>
internal static class HlrbRep
{
    /// <summary>The part of the projected curve ec between the 3D parameters U1 and U2, as a 2D edge.</summary>
    public static Edge MakeEdge(HiddenLineCurve ec, double u1, double u2)
    {
        Edge edg = new();
        double sta = ec.Parameter2d(u1);
        double end = ec.Parameter2d(u2);

        switch (ec.GetType())
        {
            case Line:
                edg = new LibraryMakeEdge2d(ec.Line(), sta, end).InternalEdge();
                break;

            case Circle:
                edg = new LibraryMakeEdge2d(ec.Circle(), sta, end).InternalEdge();
                break;

            case Ellipse:
                edg = new LibraryMakeEdge2d(ec.Ellipse(), sta, end).InternalEdge();
                break;

            case Hyperbola:
                edg = new LibraryMakeEdge2d(ec.Hyperbola(), sta, end).InternalEdge();
                break;

            case Parabola:
                edg = new LibraryMakeEdge2d(ec.Parabola(), sta, end).InternalEdge();
                break;

            case BezierCurve:
            {
                NCollection_Array1<Point2D> poles = new(1, ec.PoleCount);
                BezierCurve2D ec2d;
                if (ec.IsRational)
                {
                    NCollection_Array1<double> weights = new(1, ec.PoleCount);
                    ec.PolesAndWeights(poles, weights);
                    ec2d = new BezierCurve2D(poles, weights);
                }
                else
                {
                    ec.Poles(poles);
                    ec2d = new BezierCurve2D(poles);
                }
                LibraryMakeEdge2d mke2d = new(ec2d, sta, end);
                if (mke2d.IsDone)
                {
                    edg = mke2d.InternalEdge();
                }
                break;
            }

            case BSplineCurve:
            {
                BSplineCurve2D ec2d;
                CurveAdaptor gAcurve = ec.GetCurve().Curve();
                Edge edge = ec.GetCurve().Edge();
                double fpar, lpar;
                Curve3D? curve = Tool.Curve(edge, out fpar, out lpar);
                if (curve is TrimmedCurve3D trimmed)
                {
                    curve = trimmed.BasisCurve;
                }
                BSplineCurve3D bSplCurve = (BSplineCurve3D)curve!;
                BSplineCurve3D theCurve = (BSplineCurve3D)bSplCurve.Copy();
                if (theCurve.IsPeriodic && !gAcurve.IsClosed)
                {
                    theCurve.Segment(sta, end);
                    NCollection_Array1<Point2D> poles = new(1, theCurve.PoleCount);
                    NCollection_Array1<double> knots = new(theCurve.InternalKnots());
                    NCollection_Array1<int> mults = new(theCurve.InternalMultiplicities());
                    if (theCurve.IsRational)
                    {
                        NCollection_Array1<double> weights = new(1, theCurve.PoleCount);
                        ec.PolesAndWeights(theCurve, poles, weights);
                        ec2d = new BSplineCurve2D(poles, weights, knots, mults, theCurve.Degree,
                                                       theCurve.IsPeriodic);
                    }
                    else
                    {
                        ec.Poles(theCurve, poles);
                        ec2d = new BSplineCurve2D(poles, knots, mults, theCurve.Degree, theCurve.IsPeriodic);
                    }
                }
                else
                {
                    NCollection_Array1<Point2D> poles = new(1, ec.PoleCount);
                    NCollection_Array1<double> knots = new(1, ec.KnotCount);
                    NCollection_Array1<int> mults = new(1, ec.KnotCount);
                    //-- ec.KnotsAndMultiplicities(knots,mults);
                    ec.Knots(knots);
                    ec.Multiplicities(mults);
                    if (ec.IsRational)
                    {
                        NCollection_Array1<double> weights = new(1, ec.PoleCount);
                        ec.PolesAndWeights(poles, weights);
                        ec2d = new BSplineCurve2D(poles, weights, knots, mults, ec.Degree(), ec.IsPeriodic);
                    }
                    else
                    {
                        ec.Poles(poles);
                        ec2d = new BSplineCurve2D(poles, knots, mults, ec.Degree(), ec.IsPeriodic);
                    }
                }
                LibraryMakeEdge2d mke2d = new(ec2d, sta, end);
                if (mke2d.IsDone)
                {
                    edg = mke2d.InternalEdge();
                }
                break;
            }
            default:
            {
                const int nbPnt = 15;
                NCollection_Array1<Point2D> poles = new(1, nbPnt);
                NCollection_Array1<double> knots = new(1, nbPnt);
                NCollection_Array1<int> mults = new(1, nbPnt);
                mults.Init(1);
                mults[1] = 2;
                mults[nbPnt] = 2;
                double step = (u2 - u1) / (nbPnt - 1);
                double par3d = u1;
                for (int i = 1; i < nbPnt; i++)
                {
                    poles[i] = ec.Value(par3d);
                    knots[i] = par3d;
                    par3d += step;
                }
                poles[nbPnt] = ec.Value(u2);
                knots[nbPnt] = u2;

                BSplineCurve2D ec2d = new(poles, knots, mults, 1);
                LibraryMakeEdge2d mke2d = new(ec2d, sta, end);
                if (mke2d.IsDone)
                {
                    edg = mke2d.InternalEdge();
                }
                break;
            }
        }
        return edg;
    }

    /// <summary>
    /// The part of the edge of ec between the 3D parameters U1 and U2, as a 3D edge on the edge's curve, sharing the
    /// edge's vertices where it ends at them.
    /// </summary>
    public static Edge MakeEdge3d(HiddenLineCurve ec, double u1, double u2)
    {
        Edge edg;
        // double sta = ec.Parameter2d(U1);
        // double end = ec.Parameter2d(U2);

        Edge edge = ec.GetCurve().Edge();
        double fpar, lpar;
        // BRep_Tool::Range(anEdge, fpar, lpar);
        // Geom_Curve aCurve = BRep_Tool::Curve(anEdge, fpar, lpar);
        EdgeCurveAdaptor bAcurve = new(edge);
        fpar = bAcurve.FirstParameter();
        lpar = bAcurve.LastParameter();

        edg = ShapeCasts.Edge(((Shape)edge).EmptyCopied());
        edg.Orient(Forward);
        BrepBuilder bb = new();
        bb.Range(edg, u1, u2);

        // Share vertices if possible
        Vertex v1, v2, v1new, v2new;
        Exploration.Vertices(edge, out v1, out v2);

        double tol = global::Kerf.Engine.Foundation.Precision.PConfusion();
        if (Math.Abs(fpar - u1) <= tol)
        {
            v1new = v1;
        }
        else
        {
            Point3D pnt = bAcurve.Value(u1);
            v1new = new LibraryMakeVertex(pnt).InternalVertex();
        }
        if (Math.Abs(lpar - u2) <= tol)
        {
            v2new = v2;
        }
        else
        {
            Point3D pnt = bAcurve.Value(u2);
            v2new = new LibraryMakeVertex(pnt).InternalVertex();
        }

        v1new.Orient(Forward);
        v2new.Orient(Reversed);
        bb.Add(edg, v1new);
        bb.Add(edg, v2new);
        return edg;
    }

    private static readonly double HAngMin = 1 * M_PI / 180;
    private static readonly double HAngLim = 5 * M_PI / 180;
    private static readonly double HAngMax = 35 * M_PI / 180;

    /// <summary>The angle and the deflection of the polygonal hidden-line removal for the angle InAngl.</summary>
    public static void PolyHLRAngleAndDeflection(double inAngl, out double outAngl, out double outDefl)
    {
        outAngl = inAngl;
        if (outAngl < HAngMin)
        {
            outAngl = HAngMin;
        }
        if (outAngl > HAngMax)
        {
            outAngl = HAngMax;
        }
        outAngl = HAngLim + Math.Sqrt((outAngl - HAngMin) * (HAngMax - HAngLim) * (HAngMax - HAngLim) / (HAngMax - HAngMin));
        outDefl = outAngl * outAngl * 0.5;
    }
}
