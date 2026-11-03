// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.CurveKind;
using static Kerf.Topology.Orientation;

namespace Occt;

/// <summary>
/// The hidden-line removal's edge builders: a part of an edge's projected curve as a 2D edge, or as a 3D edge on
/// the edge's own curve, and the angle and deflection of the polygonal hidden-line removal.
/// </summary>
/// <remarks>
/// The edges are built unfrozen, for ported code (HLRBRep_HLRToShape freezes the compounds it returns). A 3D edge
/// shares the curve and, at its ends, the vertices of the edge HLRBRep_Curve was loaded with: the outliner's working
/// shape, which is the caller's shape or its copy at the entry (decision 1095).
/// </remarks>
public static class HLRBRep
{
    /// <summary>The part of the projected curve ec between the 3D parameters U1 and U2, as a 2D edge.</summary>
    public static Edge MakeEdge(HLRBRep_Curve ec, double U1, double U2)
    {
        Edge Edg = new();
        double sta = ec.Parameter2d(U1);
        double end = ec.Parameter2d(U2);

        switch (ec.GetType())
        {
            case Line:
                Edg = new BRepLib_MakeEdge2d(ec.Line(), sta, end).InternalEdge();
                break;

            case Circle:
                Edg = new BRepLib_MakeEdge2d(ec.Circle(), sta, end).InternalEdge();
                break;

            case Ellipse:
                Edg = new BRepLib_MakeEdge2d(ec.Ellipse(), sta, end).InternalEdge();
                break;

            case Hyperbola:
                Edg = new BRepLib_MakeEdge2d(ec.Hyperbola(), sta, end).InternalEdge();
                break;

            case Parabola:
                Edg = new BRepLib_MakeEdge2d(ec.Parabola(), sta, end).InternalEdge();
                break;

            case BezierCurve:
            {
                NCollection_Array1<Point2D> Poles = new(1, ec.NbPoles());
                BezierCurve2D ec2d;
                if (ec.IsRational())
                {
                    NCollection_Array1<double> Weights = new(1, ec.NbPoles());
                    ec.PolesAndWeights(Poles, Weights);
                    ec2d = new BezierCurve2D(Poles, Weights);
                }
                else
                {
                    ec.Poles(Poles);
                    ec2d = new BezierCurve2D(Poles);
                }
                BRepLib_MakeEdge2d mke2d = new(ec2d, sta, end);
                if (mke2d.IsDone())
                {
                    Edg = mke2d.InternalEdge();
                }
                break;
            }

            case BSplineCurve:
            {
                BSplineCurve2D ec2d;
                CurveAdaptor GAcurve = ec.GetCurve().Curve();
                Edge anEdge = ec.GetCurve().Edge();
                double fpar, lpar;
                Curve3D? aCurve = Tool.Curve(anEdge, out fpar, out lpar);
                if (aCurve is TrimmedCurve3D aTrimmed)
                {
                    aCurve = aTrimmed.BasisCurve();
                }
                BSplineCurve3D BSplCurve = (BSplineCurve3D)aCurve!;
                BSplineCurve3D theCurve = (BSplineCurve3D)BSplCurve.Copy();
                if (theCurve.IsPeriodic && !GAcurve.IsClosed)
                {
                    theCurve.Segment(sta, end);
                    NCollection_Array1<Point2D> Poles = new(1, theCurve.PoleCount);
                    NCollection_Array1<double> knots = new(theCurve.InternalKnots());
                    NCollection_Array1<int> mults = new(theCurve.InternalMultiplicities());
                    if (theCurve.IsRational)
                    {
                        NCollection_Array1<double> Weights = new(1, theCurve.PoleCount);
                        ec.PolesAndWeights(theCurve, Poles, Weights);
                        ec2d = new BSplineCurve2D(Poles, Weights, knots, mults, theCurve.Degree(),
                                                       theCurve.IsPeriodic);
                    }
                    else
                    {
                        ec.Poles(theCurve, Poles);
                        ec2d = new BSplineCurve2D(Poles, knots, mults, theCurve.Degree(), theCurve.IsPeriodic);
                    }
                }
                else
                {
                    NCollection_Array1<Point2D> Poles = new(1, ec.NbPoles());
                    NCollection_Array1<double> knots = new(1, ec.NbKnots());
                    NCollection_Array1<int> mults = new(1, ec.NbKnots());
                    //-- ec.KnotsAndMultiplicities(knots,mults);
                    ec.Knots(knots);
                    ec.Multiplicities(mults);
                    if (ec.IsRational())
                    {
                        NCollection_Array1<double> Weights = new(1, ec.NbPoles());
                        ec.PolesAndWeights(Poles, Weights);
                        ec2d = new BSplineCurve2D(Poles, Weights, knots, mults, ec.Degree(), ec.IsPeriodic());
                    }
                    else
                    {
                        ec.Poles(Poles);
                        ec2d = new BSplineCurve2D(Poles, knots, mults, ec.Degree(), ec.IsPeriodic());
                    }
                }
                BRepLib_MakeEdge2d mke2d = new(ec2d, sta, end);
                if (mke2d.IsDone())
                {
                    Edg = mke2d.InternalEdge();
                }
                break;
            }
            default:
            {
                const int nbPnt = 15;
                NCollection_Array1<Point2D> Poles = new(1, nbPnt);
                NCollection_Array1<double> knots = new(1, nbPnt);
                NCollection_Array1<int> mults = new(1, nbPnt);
                mults.Init(1);
                mults[1] = 2;
                mults[nbPnt] = 2;
                double step = (U2 - U1) / (nbPnt - 1);
                double par3d = U1;
                for (int i = 1; i < nbPnt; i++)
                {
                    Poles[i] = ec.Value(par3d);
                    knots[i] = par3d;
                    par3d += step;
                }
                Poles[nbPnt] = ec.Value(U2);
                knots[nbPnt] = U2;

                BSplineCurve2D ec2d = new(Poles, knots, mults, 1);
                BRepLib_MakeEdge2d mke2d = new(ec2d, sta, end);
                if (mke2d.IsDone())
                {
                    Edg = mke2d.InternalEdge();
                }
                break;
            }
        }
        return Edg;
    }

    /// <summary>
    /// The part of the edge of ec between the 3D parameters U1 and U2, as a 3D edge on the edge's curve, sharing the
    /// edge's vertices where it ends at them.
    /// </summary>
    public static Edge MakeEdge3d(HLRBRep_Curve ec, double U1, double U2)
    {
        Edge Edg;
        // double sta = ec.Parameter2d(U1);
        // double end = ec.Parameter2d(U2);

        Edge anEdge = ec.GetCurve().Edge();
        double fpar, lpar;
        // BRep_Tool::Range(anEdge, fpar, lpar);
        // Geom_Curve aCurve = BRep_Tool::Curve(anEdge, fpar, lpar);
        EdgeCurveAdaptor BAcurve = new(anEdge);
        fpar = BAcurve.FirstParameter();
        lpar = BAcurve.LastParameter();

        Edg = ShapeCasts.Edge(((Shape)anEdge).EmptyCopied());
        Edg.Orientation(Forward);
        BrepBuilder BB = new();
        BB.Range(Edg, U1, U2);

        // Share vertices if possible
        Vertex V1, V2, V1new, V2new;
        Exploration.Vertices(anEdge, out V1, out V2);

        double Tol = global::Kerf.Engine.Foundation.Precision.PConfusion();
        if (Math.Abs(fpar - U1) <= Tol)
        {
            V1new = V1;
        }
        else
        {
            Point3D aPnt = BAcurve.Value(U1);
            V1new = new BRepLib_MakeVertex(aPnt).InternalVertex();
        }
        if (Math.Abs(lpar - U2) <= Tol)
        {
            V2new = V2;
        }
        else
        {
            Point3D aPnt = BAcurve.Value(U2);
            V2new = new BRepLib_MakeVertex(aPnt).InternalVertex();
        }

        V1new.Orientation(Forward);
        V2new.Orientation(Reversed);
        BB.Add(Edg, V1new);
        BB.Add(Edg, V2new);
        return Edg;
    }

    private static readonly double HAngMin = 1 * M_PI / 180;
    private static readonly double HAngLim = 5 * M_PI / 180;
    private static readonly double HAngMax = 35 * M_PI / 180;

    /// <summary>The angle and the deflection of the polygonal hidden-line removal for the angle InAngl.</summary>
    public static void PolyHLRAngleAndDeflection(double InAngl, out double OutAngl, out double OutDefl)
    {
        OutAngl = InAngl;
        if (OutAngl < HAngMin)
        {
            OutAngl = HAngMin;
        }
        if (OutAngl > HAngMax)
        {
            OutAngl = HAngMax;
        }
        OutAngl = HAngLim + Math.Sqrt((OutAngl - HAngMin) * (HAngMax - HAngLim) * (HAngMax - HAngLim) / (HAngMax - HAngMin));
        OutDefl = OutAngl * OutAngl * 0.5;
    }
}
