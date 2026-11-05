// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dHatch/Geom2dHatch_Intersector.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dHatch/Geom2dHatch_Intersector.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dHatch/Geom2dHatch_Intersector.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Classification.Topological;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Properties;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Classification.Hatching;

/// <summary>
/// The intersector of a 2D hatcher: intersects hatchings with elements (<see cref="Intersect"/>), and, for the
/// classifier, a segment of a 2D line with an element (<see cref="Perform"/>).
/// </summary>
/// <remarks>
/// A working object (decision 21) deriving from <see cref="GInter"/>, as in OCCT; it implements TopClass's
/// intersector interface for <see cref="FClass2dOfClassifier2D"/> (decisions 288, 391 and 665).
/// </remarks>
internal sealed class Intersector2D : GInter, IIntersector<CurveAdaptor2D>
{
    private double _confusionTolerance;
    private double _tangencyTolerance;

    /// <summary>An intersector with the confusion and tangency tolerances given.</summary>
    public Intersector2D(double confusion, double tangency)
    {
        _confusionTolerance = confusion;
        _tangencyTolerance = tangency;
    }

    /// <summary>An intersector with zero tolerances.</summary>
    public Intersector2D()
    {
        _confusionTolerance = 0.0;
        _tangencyTolerance = 0.0;
    }

    /// <summary>
    /// A copy of <paramref name="other"/>: its tolerances and results, not done (C++'s copy constructor, through
    /// IntRes2d_Intersection's).
    /// </summary>
    public Intersector2D(Intersector2D other)
    {
        _confusionTolerance = other._confusionTolerance;
        _tangencyTolerance = other._tangencyTolerance;
        _lpnt = new NCollection_Sequence<IntersectionPoint2D>(other._lpnt);
        _lseg = new NCollection_Sequence<IntersectionSegment2D>(other._lseg);
    }

    /// <summary>C++'s assignment: the tolerances, the results and the flags of <paramref name="other"/>.</summary>
    internal void Assign(Intersector2D other)
    {
        base.Assign(other);
        _confusionTolerance = other._confusionTolerance;
        _tangencyTolerance = other._tangencyTolerance;
    }

    /// <summary>The confusion tolerance of the intersector.</summary>
    public double ConfusionTolerance() => _confusionTolerance;

    /// <summary>Sets the confusion tolerance of the intersector.</summary>
    public void SetConfusionTolerance(double confusion)
    {
        _confusionTolerance = confusion;
    }

    /// <summary>The tangency tolerance of the intersector.</summary>
    public double TangencyTolerance() => _tangencyTolerance;

    /// <summary>Sets the tangency tolerance of the intersector.</summary>
    public void SetTangencyTolerance(double tangency)
    {
        _tangencyTolerance = tangency;
    }

    /// <summary>Intersects the curves <paramref name="c1"/> and <paramref name="c2"/>; the results are this object's.</summary>
    public void Intersect(CurveAdaptor2D c1, CurveAdaptor2D c2)
    {
        Curve2DAdaptorBase aC1 = c1, aC2 = c2;
        base.Perform(aC1, aC2, _confusionTolerance, _tangencyTolerance);
    }

    /// <summary>
    /// Intersects the segment [0, <paramref name="p"/>] of the line <paramref name="l"/> (P may be RealLast()), with
    /// the tolerance <paramref name="tol"/> on the segment, and the curve <paramref name="c"/>. The order matters:
    /// the segment is the first curve, the element the second.
    /// </summary>
    public void Perform(in Line2D l, double p, double tol, in CurveAdaptor2D c)
    {
        // double pfbid,plbid;
        TwoDDomain dl = new();
        if (p != RealLast())
        {
            dl.SetValues(l.Location, 0.0, tol, global::Kerf.Engine.Geometry.ElCLib.Value(p, l), p, tol);
        }
        else
        {
            dl.SetValues(l.Location, 0.0, tol, true);
        }

        TwoDDomain de = new(c.Value(c.FirstParameter()),
                                 c.FirstParameter(),
                                 global::Kerf.Engine.Foundation.Precision.PIntersection(),
                                 c.Value(c.LastParameter()),
                                 c.LastParameter(),
                                 global::Kerf.Engine.Foundation.Precision.PIntersection());

        LineCurve2D gl = new(l);
        CurveAdaptor2D cga = new(gl);
        GInter inter = new(cga, dl, c, de, global::Kerf.Engine.Foundation.Precision.PConfusion(), global::Kerf.Engine.Foundation.Precision.PIntersection());
        SetValues(inter);
    }

    /// <summary>
    /// The tangent <paramref name="tang"/>, normal <paramref name="norm"/> and curvature <paramref name="c"/> of the
    /// curve of <paramref name="e"/> at the parameter <paramref name="u"/>. Where the tangent is undefined, it is
    /// (1, 0) and the curvature 0.
    /// </summary>
    public void LocalGeometry(in CurveAdaptor2D e, double u, out Direction2D tang, out Direction2D norm, out double c)
    {
        CLProps2d prop = new(e.Curve()!, u, 2, global::Kerf.Engine.Foundation.Precision.PConfusion());
        c = 0.0;
        if (prop.IsTangentDefined())
        {
            prop.Tangent(out tang);
            c = prop.Curvature();
        }
        else
        {
            tang = new Direction2D(1.0, 0.0);
        }

        if (c > global::Kerf.Engine.Foundation.Precision.PConfusion() && c < RealLast())
        {
            prop.Normal(out norm);
        }
        else
        {
            norm = new Direction2D();
            norm = norm.WithCoord(tang.Y, -tang.X);
        }
    }
}
