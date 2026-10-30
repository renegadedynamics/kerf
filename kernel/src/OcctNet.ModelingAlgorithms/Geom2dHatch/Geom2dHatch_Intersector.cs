// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dHatch/Geom2dHatch_Intersector.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dHatch/Geom2dHatch_Intersector.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dHatch/Geom2dHatch_Intersector.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The intersector of a 2D hatcher: intersects hatchings with elements (<see cref="Intersect"/>), and, for the
/// classifier, a segment of a 2D line with an element (<see cref="Perform"/>).
/// </summary>
/// <remarks>
/// A working object (decision 21) deriving from <see cref="Geom2dInt_GInter"/>, as in OCCT; it implements TopClass's
/// intersector interface for <see cref="Geom2dHatch_FClass2dOfClassifier"/> (decisions 288, 391 and 665).
/// </remarks>
public sealed class Geom2dHatch_Intersector : Geom2dInt_GInter, ITopClass_Intersector<Geom2dAdaptor_Curve>
{
    private double myConfusionTolerance;
    private double myTangencyTolerance;

    /// <summary>An intersector with the confusion and tangency tolerances given.</summary>
    public Geom2dHatch_Intersector(double Confusion, double Tangency)
    {
        myConfusionTolerance = Confusion;
        myTangencyTolerance = Tangency;
    }

    /// <summary>An intersector with zero tolerances.</summary>
    public Geom2dHatch_Intersector()
    {
        myConfusionTolerance = 0.0;
        myTangencyTolerance = 0.0;
    }

    /// <summary>
    /// A copy of <paramref name="Other"/>: its tolerances and results, not done (C++'s copy constructor, through
    /// IntRes2d_Intersection's).
    /// </summary>
    public Geom2dHatch_Intersector(Geom2dHatch_Intersector Other)
    {
        myConfusionTolerance = Other.myConfusionTolerance;
        myTangencyTolerance = Other.myTangencyTolerance;
        lpnt = new NCollection_Sequence<IntRes2d_IntersectionPoint>(Other.lpnt);
        lseg = new NCollection_Sequence<IntRes2d_IntersectionSegment>(Other.lseg);
    }

    /// <summary>C++'s assignment: the tolerances, the results and the flags of <paramref name="Other"/>.</summary>
    internal void Assign(Geom2dHatch_Intersector Other)
    {
        base.Assign(Other);
        myConfusionTolerance = Other.myConfusionTolerance;
        myTangencyTolerance = Other.myTangencyTolerance;
    }

    /// <summary>The confusion tolerance of the intersector.</summary>
    public double ConfusionTolerance() => myConfusionTolerance;

    /// <summary>Sets the confusion tolerance of the intersector.</summary>
    public void SetConfusionTolerance(double Confusion)
    {
        myConfusionTolerance = Confusion;
    }

    /// <summary>The tangency tolerance of the intersector.</summary>
    public double TangencyTolerance() => myTangencyTolerance;

    /// <summary>Sets the tangency tolerance of the intersector.</summary>
    public void SetTangencyTolerance(double Tangency)
    {
        myTangencyTolerance = Tangency;
    }

    /// <summary>Intersects the curves <paramref name="C1"/> and <paramref name="C2"/>; the results are this object's.</summary>
    public void Intersect(Geom2dAdaptor_Curve C1, Geom2dAdaptor_Curve C2)
    {
        Adaptor2d_Curve2d aC1 = C1, aC2 = C2;
        base.Perform(aC1, aC2, myConfusionTolerance, myTangencyTolerance);
    }

    /// <summary>
    /// Intersects the segment [0, <paramref name="P"/>] of the line <paramref name="L"/> (P may be RealLast()), with
    /// the tolerance <paramref name="Tol"/> on the segment, and the curve <paramref name="C"/>. The order matters:
    /// the segment is the first curve, the element the second.
    /// </summary>
    public void Perform(in gp_Lin2d L, double P, double Tol, in Geom2dAdaptor_Curve C)
    {
        // double pfbid,plbid;
        IntRes2d_Domain DL = new();
        if (P != RealLast())
        {
            DL.SetValues(L.Location(), 0.0, Tol, ElCLib.Value(P, L), P, Tol);
        }
        else
        {
            DL.SetValues(L.Location(), 0.0, Tol, true);
        }

        IntRes2d_Domain DE = new(C.Value(C.FirstParameter()),
                                 C.FirstParameter(),
                                 Precision.PIntersection(),
                                 C.Value(C.LastParameter()),
                                 C.LastParameter(),
                                 Precision.PIntersection());

        Geom2d_Line GL = new(L);
        Geom2dAdaptor_Curve CGA = new(GL);
        Geom2dInt_GInter Inter = new(CGA, DL, C, DE, Precision.PConfusion(), Precision.PIntersection());
        SetValues(Inter);
    }

    /// <summary>
    /// The tangent <paramref name="Tang"/>, normal <paramref name="Norm"/> and curvature <paramref name="C"/> of the
    /// curve of <paramref name="E"/> at the parameter <paramref name="U"/>. Where the tangent is undefined, it is
    /// (1, 0) and the curvature 0.
    /// </summary>
    public void LocalGeometry(in Geom2dAdaptor_Curve E, double U, out gp_Dir2d Tang, out gp_Dir2d Norm, out double C)
    {
        GeomLProp_CLProps2d Prop = new(E.Curve()!, U, 2, Precision.PConfusion());
        C = 0.0;
        if (Prop.IsTangentDefined())
        {
            Prop.Tangent(out Tang);
            C = Prop.Curvature();
        }
        else
        {
            Tang = new gp_Dir2d(1.0, 0.0);
        }

        if (C > Precision.PConfusion() && C < RealLast())
        {
            Prop.Normal(out Norm);
        }
        else
        {
            Norm = new gp_Dir2d();
            Norm.SetCoord(Tang.Y(), -Tang.X());
        }
    }
}
