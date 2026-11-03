using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;
using static Occt.IntRes2d_Position;
using static Occt.IntRes2d_TypeTrans;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Known-answer tests of IntCurve_IntConicConic (every pair of conics), IntCurve_IntImpConicParConic and the conic
/// tools.
/// </summary>
public class IntCurve_IntConicConicTests
{
    private const double TolConf = 1.0e-9;
    private const double Tol = 1.0e-9;

    private static Axis2D Ax(double x, double y, double dx, double dy) => new(new Point2D(x, y), new Direction2D(dx, dy));

    private static IntRes2d_Domain Dom(Func<double, Point2D> f, double u0, double u1, double tol = 1.0e-9)
        => new(f(u0), u0, tol, f(u1), u1, tol);

    private static IntRes2d_Domain Closed(Func<double, Point2D> f, double u0 = 0, double u1 = 2 * Math.PI)
        => Dom(f, u0, u1).WithEquivalentParameters(u0, u1);

    /// <summary>Every point and segment end lies on both curves at its parameters.</summary>
    private static void CheckOnCurves(IntRes2d_Intersection I, Func<double, Point2D> f1, Func<double, Point2D> f2,
                                      double tol = 1.0e-6)
    {
        Assert.True(I.IsDone());
        for (int i = 1; i <= I.NbPoints(); i++)
        {
            CheckPoint(I.Point(i), f1, f2, tol);
        }
        for (int i = 1; i <= I.NbSegments(); i++)
        {
            IntRes2d_IntersectionSegment S = I.Segment(i);
            if (S.HasFirstPoint())
            {
                CheckPoint(S.FirstPoint(), f1, f2, tol);
            }
            if (S.HasLastPoint())
            {
                CheckPoint(S.LastPoint(), f1, f2, tol);
            }
        }
    }

    private static void CheckPoint(IntRes2d_IntersectionPoint P, Func<double, Point2D> f1, Func<double, Point2D> f2,
                                   double tol)
    {
        Assert.True(P.Value().Distance(f1(P.ParamOnFirst())) < tol,
                    $"{P.Value()} vs C1({P.ParamOnFirst()}) = {f1(P.ParamOnFirst())}");
        Assert.True(P.Value().Distance(f2(P.ParamOnSecond())) < tol,
                    $"{P.Value()} vs C2({P.ParamOnSecond()}) = {f2(P.ParamOnSecond())}");
    }

    private static bool HasPointNear(IntRes2d_Intersection I, double x, double y, double tol = 1.0e-6)
    {
        for (int i = 1; i <= I.NbPoints(); i++)
        {
            if (I.Point(i).Value().Distance(new Point2D(x, y)) < tol)
            {
                return true;
            }
        }
        return false;
    }

    // ------------------------------------------------------------------ line / line

    [Fact]
    public void LineLine_Crossing()
    {
        Line2D L1 = new(Ax(0, 0, 1, 0));
        Line2D L2 = new(Ax(2, 0, 1, 1));
        IntCurve_IntConicConic I = new(L1, Dom(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, L1), -10, 10), L2,
                                       Dom(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, L2), -10, 10), TolConf, Tol);
        Assert.Equal(1, I.NbPoints());
        Assert.Equal(0, I.NbSegments());
        IntRes2d_IntersectionPoint P = I.Point(1);
        Assert.Equal(2.0, P.Value().X(), 12);
        Assert.Equal(0.0, P.Value().Y(), 12);
        Assert.Equal(2.0, P.ParamOnFirst(), 12);
        Assert.Equal(0.0, P.ParamOnSecond(), 12);
        // L1 x L2 > 0: the first curve goes out, the second in.
        Assert.Equal(IntRes2d_Out, P.TransitionOfFirst().TransitionType());
        Assert.Equal(IntRes2d_In, P.TransitionOfSecond().TransitionType());
        Assert.Equal(IntRes2d_Middle, P.TransitionOfFirst().PositionOnCurve());
    }

    [Fact]
    public void LineLine_CrossingUnbounded()
    {
        Line2D L1 = new(Ax(0, 1, 1, 0));
        Line2D L2 = new(Ax(3, -2, 0, 1));
        IntCurve_IntConicConic I = new(L1, new IntRes2d_Domain(), L2, new IntRes2d_Domain(), TolConf, Tol);
        Assert.Equal(1, I.NbPoints());
        Assert.True(HasPointNear(I, 3, 1));
        CheckOnCurves(I, u => global::Kerf.Engine.Geometry.ElCLib.Value(u, L1), u => global::Kerf.Engine.Geometry.ElCLib.Value(u, L2));
    }

    [Fact]
    public void LineLine_AtDomainEnd()
    {
        Line2D L1 = new(Ax(0, 0, 1, 0));
        Line2D L2 = new(Ax(2, 0, 0, 1));
        IntCurve_IntConicConic I = new(L1, Dom(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, L1), 0, 2), L2,
                                       Dom(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, L2), 0, 5), TolConf, Tol);
        Assert.Equal(1, I.NbPoints());
        IntRes2d_IntersectionPoint P = I.Point(1);
        Assert.Equal(IntRes2d_End, P.TransitionOfFirst().PositionOnCurve());
        Assert.Equal(IntRes2d_Head, P.TransitionOfSecond().PositionOnCurve());
    }

    [Fact]
    public void LineLine_OutsideDomain()
    {
        Line2D L1 = new(Ax(0, 0, 1, 0));
        Line2D L2 = new(Ax(5, 0, 0, 1));
        IntCurve_IntConicConic I = new(L1, Dom(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, L1), 0, 2), L2,
                                       Dom(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, L2), -1, 1), TolConf, Tol);
        Assert.True(I.IsDone());
        Assert.True(I.IsEmpty());
    }

    [Fact]
    public void LineLine_Parallel()
    {
        Line2D L1 = new(Ax(0, 0, 1, 0));
        Line2D L2 = new(Ax(0, 1, 1, 0));
        IntCurve_IntConicConic I = new(L1, Dom(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, L1), -1, 1), L2,
                                       Dom(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, L2), -1, 1), TolConf, Tol);
        Assert.True(I.IsDone());
        Assert.True(I.IsEmpty());
    }

    [Fact]
    public void LineLine_CoincidentBounded_IsASegment()
    {
        Line2D L1 = new(Ax(0, 0, 1, 0));
        Line2D L2 = new(Ax(1, 0, -1, 0));
        IntCurve_IntConicConic I = new(L1, Dom(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, L1), 0, 4), L2,
                                       Dom(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, L2), -5, 0), TolConf, Tol);
        Assert.Equal(0, I.NbPoints());
        Assert.Equal(1, I.NbSegments());
        IntRes2d_IntersectionSegment S = I.Segment(1);
        Assert.True(S.IsOpposite());
        Assert.Equal(1.0, S.FirstPoint().ParamOnFirst(), 12);
        Assert.Equal(4.0, S.LastPoint().ParamOnFirst(), 12);
        CheckOnCurves(I, u => global::Kerf.Engine.Geometry.ElCLib.Value(u, L1), u => global::Kerf.Engine.Geometry.ElCLib.Value(u, L2));
    }

    [Fact]
    public void LineLine_CoincidentUnbounded_IsAnInfiniteSegment()
    {
        Line2D L1 = new(Ax(0, 0, 1, 0));
        Line2D L2 = new(Ax(3, 0, 1, 0));
        IntCurve_IntConicConic I = new(L1, new IntRes2d_Domain(), L2, new IntRes2d_Domain(), TolConf, Tol);
        Assert.Equal(1, I.NbSegments());
        Assert.False(I.Segment(1).HasFirstPoint());
        Assert.False(I.Segment(1).HasLastPoint());
        Assert.False(I.Segment(1).IsOpposite());
    }

    // ------------------------------------------------------------------ line / circle

    private static readonly Circle2D theUnitCircle = new(Ax(0, 0, 1, 0), 1.0);

    [Fact]
    public void LineCircle_Secant()
    {
        Line2D L = new(Ax(-3, 0.5, 1, 0));
        IntCurve_IntConicConic I = new(L, Dom(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, L), 0, 6), theUnitCircle,
                                       Closed(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theUnitCircle)), TolConf, Tol);
        Assert.Equal(2, I.NbPoints());
        Assert.Equal(0, I.NbSegments());
        double x = Math.Sqrt(0.75);
        Assert.True(HasPointNear(I, x, 0.5));
        Assert.True(HasPointNear(I, -x, 0.5));
        CheckOnCurves(I, u => global::Kerf.Engine.Geometry.ElCLib.Value(u, L), u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theUnitCircle));
        for (int i = 1; i <= 2; i++)
        {
            Assert.False(I.Point(i).TransitionOfFirst().IsTangent());
        }
    }

    [Fact]
    public void LineCircle_Tangent()
    {
        Line2D L = new(Ax(-3, 1, 1, 0));
        IntCurve_IntConicConic I = new(L, Dom(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, L), 0, 6), theUnitCircle,
                                       Closed(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theUnitCircle)), TolConf, Tol);
        Assert.Equal(1, I.NbPoints() + I.NbSegments());
        if (I.NbPoints() == 1)
        {
            Assert.Equal(0.0, I.Point(1).Value().X(), 6);
            Assert.Equal(1.0, I.Point(1).Value().Y(), 6);
            Assert.Equal(Math.PI / 2, I.Point(1).ParamOnSecond(), 6);
        }
    }

    [Fact]
    public void LineCircle_Miss()
    {
        Line2D L = new(Ax(-3, 2, 1, 0));
        IntCurve_IntConicConic I = new(L, Dom(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, L), 0, 6), theUnitCircle,
                                       Closed(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theUnitCircle)), TolConf, Tol);
        Assert.True(I.IsDone());
        Assert.True(I.IsEmpty());
    }

    [Fact]
    public void LineCircle_ArcDomain()
    {
        Line2D L = new(Ax(-3, 0.5, 1, 0));
        IntCurve_IntConicConic upper = new(L, Dom(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, L), 0, 6), theUnitCircle,
                                           Dom(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theUnitCircle), 0, Math.PI), TolConf, Tol);
        Assert.Equal(2, upper.NbPoints());
        IntCurve_IntConicConic lower = new(L, Dom(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, L), 0, 6), theUnitCircle,
                                           Dom(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theUnitCircle), Math.PI, 2 * Math.PI), TolConf,
                                           Tol);
        Assert.True(lower.IsEmpty());
        IntCurve_IntConicConic right = new(L, Dom(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, L), 0, 6), theUnitCircle,
                                           Dom(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theUnitCircle), 0, Math.PI / 2), TolConf, Tol);
        Assert.Equal(1, right.NbPoints());
        Assert.Equal(Math.PI / 6, right.Point(1).ParamOnSecond(), 9);
    }

    [Fact]
    public void LineCircle_ReversedParameters()
    {
        Line2D L = new(Ax(-3, 0.5, 1, 0));
        IntCurve_IntConicConic I = new();
        I.SetReversedParameters(true);
        I.Perform(L, Dom(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, L), 0, 6), theUnitCircle, Closed(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theUnitCircle)),
                  TolConf, Tol);
        Assert.Equal(2, I.NbPoints());
        CheckOnCurves(I, u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theUnitCircle), u => global::Kerf.Engine.Geometry.ElCLib.Value(u, L));
    }

    // ------------------------------------------------------------------ circle / circle

    [Fact]
    public void CircleCircle_Secant()
    {
        Circle2D C2 = new(Ax(1, 0, 1, 0), 1.0);
        IntCurve_IntConicConic I = new(theUnitCircle, Closed(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theUnitCircle)), C2,
                                       Closed(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, C2)), TolConf, Tol);
        Assert.Equal(2, I.NbPoints());
        Assert.True(HasPointNear(I, 0.5, Math.Sqrt(0.75)));
        Assert.True(HasPointNear(I, 0.5, -Math.Sqrt(0.75)));
        CheckOnCurves(I, u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theUnitCircle), u => global::Kerf.Engine.Geometry.ElCLib.Value(u, C2));
    }

    [Fact]
    public void CircleCircle_TangentOutside()
    {
        Circle2D C2 = new(Ax(0, 2, 1, 0), 1.0);
        IntCurve_IntConicConic I = new(theUnitCircle, Closed(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theUnitCircle)), C2,
                                       Closed(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, C2)), TolConf, Tol);
        CheckTangentContact(I, 0, 1, u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theUnitCircle), u => global::Kerf.Engine.Geometry.ElCLib.Value(u, C2));
    }

    [Fact]
    public void CircleCircle_TangentAtTheSeam_IsSplitInTwo()
    {
        // The contact arc around parameter 0 is cut by the domain [0, 2 PI] into two pieces.
        Circle2D C2 = new(Ax(2, 0, 1, 0), 1.0);
        IntCurve_IntConicConic I = new(theUnitCircle, Closed(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theUnitCircle)), C2,
                                       Closed(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, C2)), TolConf, Tol);
        Assert.Equal(2, I.NbPoints() + I.NbSegments());
        CheckOnCurves(I, u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theUnitCircle), u => global::Kerf.Engine.Geometry.ElCLib.Value(u, C2));
        for (int i = 1; i <= I.NbSegments(); i++)
        {
            Assert.True(I.Segment(i).FirstPoint().Value().Distance(new Point2D(1, 0)) < 1.0e-3);
            Assert.True(I.Segment(i).LastPoint().Value().Distance(new Point2D(1, 0)) < 1.0e-3);
        }
    }

    /// <summary>
    /// Tangent circles touch inside the tolerance tube over a short arc: OCCT reports one point or one short
    /// segment around the contact.
    /// </summary>
    private static void CheckTangentContact(IntCurve_IntConicConic I, double x, double y, Func<double, Point2D> f1,
                                            Func<double, Point2D> f2)
    {
        Assert.Equal(1, I.NbPoints() + I.NbSegments());
        CheckOnCurves(I, f1, f2);
        if (I.NbPoints() == 1)
        {
            Assert.True(I.Point(1).Value().Distance(new Point2D(x, y)) < 1.0e-3);
        }
        else
        {
            IntRes2d_IntersectionSegment S = I.Segment(1);
            Assert.True(S.FirstPoint().Value().Distance(new Point2D(x, y)) < 1.0e-3);
            Assert.True(S.LastPoint().Value().Distance(new Point2D(x, y)) < 1.0e-3);
            // The ends of the contact arc are crossings of the tolerance tube, not tangencies.
            Assert.True(S.FirstPoint().ParamOnFirst() < S.LastPoint().ParamOnFirst());
        }
    }

    [Fact]
    public void CircleCircle_TangentInside()
    {
        Circle2D C1 = new(Ax(0, 0, 1, 0), 2.0);
        Circle2D C2 = new(Ax(0, 1, 1, 0), 1.0);
        IntCurve_IntConicConic I = new(C1, Closed(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, C1)), C2, Closed(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, C2)),
                                       TolConf, Tol);
        CheckTangentContact(I, 0, 2, u => global::Kerf.Engine.Geometry.ElCLib.Value(u, C1), u => global::Kerf.Engine.Geometry.ElCLib.Value(u, C2));
    }

    [Fact]
    public void CircleCircle_Concentric()
    {
        Circle2D C2 = new(Ax(0, 0, 1, 0), 2.0);
        IntCurve_IntConicConic I = new(theUnitCircle, Closed(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theUnitCircle)), C2,
                                       Closed(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, C2)), TolConf, Tol);
        Assert.True(I.IsDone());
        Assert.True(I.IsEmpty());
    }

    [Fact]
    public void CircleCircle_Identical_IsASegment()
    {
        Circle2D C2 = new(Ax(0, 0, 0, 1), 1.0);
        IntCurve_IntConicConic I = new(theUnitCircle, Closed(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theUnitCircle)), C2,
                                       Closed(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, C2)), TolConf, Tol);
        Assert.True(I.NbSegments() >= 1);
        Assert.Equal(0, I.NbPoints());
        CheckOnCurves(I, u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theUnitCircle), u => global::Kerf.Engine.Geometry.ElCLib.Value(u, C2));
        Assert.False(I.Segment(1).IsOpposite());
    }

    [Fact]
    public void CircleCircle_OppositeSenses()
    {
        Circle2D C2 = new(Ax(1, 0, 1, 0), 1.0, false);
        IntCurve_IntConicConic I = new(theUnitCircle, Closed(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theUnitCircle)), C2,
                                       Closed(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, C2)), TolConf, Tol);
        Assert.Equal(2, I.NbPoints());
        Assert.True(HasPointNear(I, 0.5, Math.Sqrt(0.75)));
        Assert.True(HasPointNear(I, 0.5, -Math.Sqrt(0.75)));
        CheckOnCurves(I, u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theUnitCircle), u => global::Kerf.Engine.Geometry.ElCLib.Value(u, C2));
    }

    [Fact]
    public void CircleCircle_ArcDomains()
    {
        Circle2D C2 = new(Ax(1, 0, 1, 0), 1.0);
        IntCurve_IntConicConic I = new(theUnitCircle, Dom(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theUnitCircle), 0, Math.PI), C2,
                                       Closed(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, C2)), TolConf, Tol);
        Assert.Equal(1, I.NbPoints());
        Assert.True(HasPointNear(I, 0.5, Math.Sqrt(0.75)));
        Assert.Equal(Math.PI / 3, I.Point(1).ParamOnFirst(), 9);
        Assert.Equal(2 * Math.PI / 3, I.Point(1).ParamOnSecond(), 9);
    }

    // ------------------------------------------------------------------ line / ellipse

    private static readonly Ellipse2D theEllipse = new(Ax(0, 0, 1, 0), 2.0, 1.0);

    [Fact]
    public void LineEllipse_Secant()
    {
        Line2D L = new(Ax(1, -3, 0, 1));
        IntCurve_IntConicConic I = new(L, Dom(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, L), 0, 6), theEllipse,
                                       Closed(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theEllipse)), TolConf, Tol);
        Assert.Equal(2, I.NbPoints());
        Assert.True(HasPointNear(I, 1, Math.Sqrt(0.75)));
        Assert.True(HasPointNear(I, 1, -Math.Sqrt(0.75)));
        CheckOnCurves(I, u => global::Kerf.Engine.Geometry.ElCLib.Value(u, L), u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theEllipse));
    }

    [Fact]
    public void LineEllipse_Oblique()
    {
        Line2D L = new(Ax(-3, -1, 1, 0.5));
        IntCurve_IntConicConic I = new(L, Dom(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, L), -10, 10), theEllipse,
                                       Closed(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theEllipse)), TolConf, Tol);
        Assert.Equal(2, I.NbPoints());
        CheckOnCurves(I, u => global::Kerf.Engine.Geometry.ElCLib.Value(u, L), u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theEllipse));
    }

    [Fact]
    public void LineEllipse_Tangent()
    {
        Line2D L = new(Ax(-3, 1, 1, 0));
        IntCurve_IntConicConic I = new(L, Dom(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, L), 0, 6), theEllipse,
                                       Closed(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theEllipse)), TolConf, Tol);
        Assert.Equal(1, I.NbPoints());
        Assert.True(HasPointNear(I, 0, 1, 1.0e-5));
    }

    [Fact]
    public void LineEllipse_Miss()
    {
        Line2D L = new(Ax(-3, 1.5, 1, 0));
        IntCurve_IntConicConic I = new(L, Dom(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, L), 0, 6), theEllipse,
                                       Closed(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theEllipse)), TolConf, Tol);
        Assert.True(I.IsEmpty());
    }

    // ------------------------------------------------------------------ the other pairs

    [Fact]
    public void CircleEllipse_FourPoints()
    {
        Circle2D C = new(Ax(0, 0, 1, 0), 1.5);
        IntCurve_IntConicConic I = new(C, Closed(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, C)), theEllipse,
                                       Closed(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theEllipse)), TolConf, Tol);
        Assert.Equal(4, I.NbPoints());
        double x = Math.Sqrt(5.0 / 3.0), y = Math.Sqrt(2.25 - 5.0 / 3.0);
        Assert.True(HasPointNear(I, x, y));
        Assert.True(HasPointNear(I, -x, y));
        Assert.True(HasPointNear(I, x, -y));
        Assert.True(HasPointNear(I, -x, -y));
        CheckOnCurves(I, u => global::Kerf.Engine.Geometry.ElCLib.Value(u, C), u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theEllipse));
    }

    [Fact]
    public void EllipseEllipse_FourPoints()
    {
        Ellipse2D E2 = new(Ax(0, 0, 0, 1), 2.0, 1.0);
        IntCurve_IntConicConic I = new(theEllipse, Closed(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theEllipse)), E2,
                                       Closed(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, E2)), TolConf, Tol);
        Assert.Equal(4, I.NbPoints());
        double x = Math.Sqrt(0.8);
        Assert.True(HasPointNear(I, x, x));
        Assert.True(HasPointNear(I, -x, -x));
        CheckOnCurves(I, u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theEllipse), u => global::Kerf.Engine.Geometry.ElCLib.Value(u, E2));
    }

    [Fact]
    public void EllipseEllipse_Disjoint()
    {
        Ellipse2D E2 = new(Ax(10, 0, 1, 0), 2.0, 1.0);
        IntCurve_IntConicConic I = new(theEllipse, Closed(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theEllipse)), E2,
                                       Closed(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, E2)), TolConf, Tol);
        Assert.True(I.IsEmpty());
    }

    private static readonly Parabola2D theParabola = new(Ax(0, 0, 1, 0), 1.0); // y^2 = 4x

    [Fact]
    public void LineParabola_Secant()
    {
        Line2D L = new(Ax(1, -5, 0, 1));
        IntCurve_IntConicConic I = new(L, Dom(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, L), 0, 10), theParabola,
                                       Dom(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theParabola), -10, 10), TolConf, Tol);
        Assert.Equal(2, I.NbPoints());
        Assert.True(HasPointNear(I, 1, 2));
        Assert.True(HasPointNear(I, 1, -2));
        CheckOnCurves(I, u => global::Kerf.Engine.Geometry.ElCLib.Value(u, L), u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theParabola));
    }

    [Fact]
    public void LineParabola_UnboundedParabola()
    {
        Line2D L = new(Ax(1, -5, 0, 1));
        IntCurve_IntConicConic I = new(L, Dom(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, L), 0, 10), theParabola, new IntRes2d_Domain(),
                                       TolConf, Tol);
        Assert.Equal(2, I.NbPoints());
        CheckOnCurves(I, u => global::Kerf.Engine.Geometry.ElCLib.Value(u, L), u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theParabola));
    }

    private static readonly Hyperbola2D theHyperbola = new(Ax(0, 0, 1, 0), 1.0, 1.0); // x^2 - y^2 = 1, x > 0

    [Fact]
    public void LineHyperbola_Secant()
    {
        Line2D L = new(Ax(2, -5, 0, 1));
        IntCurve_IntConicConic I = new(L, Dom(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, L), 0, 10), theHyperbola,
                                       Dom(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theHyperbola), -3, 3), TolConf, Tol);
        Assert.Equal(2, I.NbPoints());
        Assert.True(HasPointNear(I, 2, Math.Sqrt(3)));
        Assert.True(HasPointNear(I, 2, -Math.Sqrt(3)));
        CheckOnCurves(I, u => global::Kerf.Engine.Geometry.ElCLib.Value(u, L), u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theHyperbola));
    }

    [Fact]
    public void CircleParabola_TwoPoints()
    {
        Circle2D C = new(Ax(0, 0, 1, 0), Math.Sqrt(5));
        IntCurve_IntConicConic I = new(C, Closed(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, C)), theParabola,
                                       Dom(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theParabola), -10, 10), TolConf, Tol);
        Assert.Equal(2, I.NbPoints());
        Assert.True(HasPointNear(I, 1, 2));
        Assert.True(HasPointNear(I, 1, -2));
        CheckOnCurves(I, u => global::Kerf.Engine.Geometry.ElCLib.Value(u, C), u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theParabola));
    }

    [Fact]
    public void CircleHyperbola_TwoPoints()
    {
        Circle2D C = new(Ax(0, 0, 1, 0), Math.Sqrt(7));
        IntCurve_IntConicConic I = new(C, Closed(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, C)), theHyperbola,
                                       Dom(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theHyperbola), -3, 3), TolConf, Tol);
        Assert.Equal(2, I.NbPoints());
        Assert.True(HasPointNear(I, 2, Math.Sqrt(3)));
        Assert.True(HasPointNear(I, 2, -Math.Sqrt(3)));
        CheckOnCurves(I, u => global::Kerf.Engine.Geometry.ElCLib.Value(u, C), u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theHyperbola));
    }

    [Fact]
    public void EllipseParabola_TwoPoints()
    {
        IntCurve_IntConicConic I = new(theEllipse, Closed(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theEllipse)), theParabola,
                                       Dom(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theParabola), -10, 10), TolConf, Tol);
        Assert.Equal(2, I.NbPoints());
        double x = -8 + Math.Sqrt(68);
        Assert.True(HasPointNear(I, x, Math.Sqrt(4 * x)));
        Assert.True(HasPointNear(I, x, -Math.Sqrt(4 * x)));
        CheckOnCurves(I, u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theEllipse), u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theParabola));
    }

    [Fact]
    public void EllipseHyperbola_TwoPoints()
    {
        // x^2/4 + y^2 = 1 and x^2 - y^2 = 1: x^2 = 8/5, y^2 = 3/5 (right branch only).
        IntCurve_IntConicConic I = new(theEllipse, Closed(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theEllipse)), theHyperbola,
                                       Dom(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theHyperbola), -3, 3), TolConf, Tol);
        Assert.Equal(2, I.NbPoints());
        Assert.True(HasPointNear(I, Math.Sqrt(1.6), Math.Sqrt(0.6)));
        Assert.True(HasPointNear(I, Math.Sqrt(1.6), -Math.Sqrt(0.6)));
        CheckOnCurves(I, u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theEllipse), u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theHyperbola));
    }

    [Fact]
    public void ParabolaParabola_TwoPoints()
    {
        Parabola2D P2 = new(Ax(0, 0, 0, 1), 1.0); // x^2 = 4y
        IntCurve_IntConicConic I = new(theParabola, Dom(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theParabola), -10, 10), P2,
                                       Dom(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, P2), -10, 10), TolConf, Tol);
        Assert.True(HasPointNear(I, 4, 4));
        Assert.True(HasPointNear(I, 0, 0));
        Assert.Equal(2, I.NbPoints());
        CheckOnCurves(I, u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theParabola), u => global::Kerf.Engine.Geometry.ElCLib.Value(u, P2));
    }

    [Fact]
    public void ParabolaHyperbola_TwoPoints()
    {
        // y^2 = 4x and x^2 - y^2 = 1: x^2 - 4x - 1 = 0, x = 2 + sqrt(5).
        IntCurve_IntConicConic I = new(theParabola, Dom(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theParabola), -10, 10), theHyperbola,
                                       Dom(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theHyperbola), -3, 3), TolConf, Tol);
        double x = 2 + Math.Sqrt(5);
        Assert.Equal(2, I.NbPoints());
        Assert.True(HasPointNear(I, x, Math.Sqrt(4 * x)));
        Assert.True(HasPointNear(I, x, -Math.Sqrt(4 * x)));
        CheckOnCurves(I, u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theParabola), u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theHyperbola));
    }

    [Fact]
    public void HyperbolaHyperbola_TwoPoints()
    {
        // x^2 - y^2 = 1 (x > 0) and (3 - x)^2 - y^2 = 1 (x < 3): the branch of a hyperbola centred at (3, 0)
        // that opens towards -x.
        Hyperbola2D H2 = new(Ax(3, 0, -1, 0), 1.0, 1.0);
        IntCurve_IntConicConic I = new(theHyperbola, Dom(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theHyperbola), -3, 3), H2,
                                       Dom(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, H2), -3, 3), TolConf, Tol);
        // x^2 = (3 - x)^2 -> x = 1.5, y^2 = 1.25.
        Assert.Equal(2, I.NbPoints());
        Assert.True(HasPointNear(I, 1.5, Math.Sqrt(1.25)));
        Assert.True(HasPointNear(I, 1.5, -Math.Sqrt(1.25)));
        CheckOnCurves(I, u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theHyperbola), u => global::Kerf.Engine.Geometry.ElCLib.Value(u, H2));
    }

    // ------------------------------------------------------------------ IntImpConicParConic and the tools

    [Fact]
    public void IntImpConicParConic_CircleAndEllipse()
    {
        IntCurve_IConicTool ITool = new(new Circle2D(Ax(0, 0, 1, 0), 1.5));
        IntCurve_PConic PCurve = new(theEllipse);
        IntRes2d_Domain D1 = Closed(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, new Circle2D(Ax(0, 0, 1, 0), 1.5)));
        IntRes2d_Domain D2 = Closed(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theEllipse));
        IntCurve_IntImpConicParConic I = new(ITool, D1, PCurve, D2, TolConf, Tol);
        Assert.Equal(4, I.NbPoints());
        CheckOnCurves(I, u => ITool.Value(u), u => IntCurve_PConicTool.Value(PCurve, u));
    }

    [Fact]
    public void IntImpConicParConic_UnboundedParametricDomainThrows()
    {
        IntCurve_IConicTool ITool = new(theUnitCircle);
        IntCurve_PConic PCurve = new(new Line2D(Ax(0, 0, 1, 0)));
        Assert.ThrowsAny<ConstructionErrorException>(
            () => new IntCurve_IntImpConicParConic(ITool, Closed(u => global::Kerf.Engine.Geometry.ElCLib.Value(u, theUnitCircle)), PCurve,
                                                   new IntRes2d_Domain(), TolConf, Tol));
    }

    [Fact]
    public void IConicTool_DistanceAndParameter()
    {
        IntCurve_IConicTool circle = new(new Circle2D(Ax(1, 1, 1, 0), 2.0));
        Assert.Equal(1.0, circle.Distance(new Point2D(4, 1)), 14);
        Assert.Equal(-1.0, circle.Distance(new Point2D(1, 2)), 14);
        Assert.Equal(Math.PI / 2, circle.FindParameter(new Point2D(1, 5)), 14);
        Assert.Equal(3 * Math.PI / 2, circle.FindParameter(new Point2D(1, -5)), 14);

        IntCurve_IConicTool line = new(new Line2D(Ax(0, 1, 1, 0)));
        Assert.Equal(0.0, line.Distance(new Point2D(7, 1)), 14);
        Assert.Equal(1.0, Math.Abs(line.Distance(new Point2D(7, 2))), 14);
        Assert.Equal(7.0, line.FindParameter(new Point2D(7, 2)), 14);

        IntCurve_IConicTool ellipse = new(theEllipse);
        Assert.Equal(0.0, ellipse.Distance(new Point2D(2, 0)), 14);
        Assert.True(ellipse.Distance(new Point2D(0, 0)) < 0);
        Assert.True(ellipse.Distance(new Point2D(3, 3)) > 0);

        IntCurve_IConicTool parab = new(theParabola);
        Assert.Equal(0.0, parab.Distance(new Point2D(1, 2)), 14);
        IntCurve_IConicTool hypr = new(theHyperbola);
        Assert.Equal(0.0, hypr.Distance(new Point2D(2, Math.Sqrt(3))), 12);
    }

    [Fact]
    public void IConicTool_GradientMatchesFiniteDifferences()
    {
        IntCurve_IConicTool[] tools =
        [
            new(new Line2D(Ax(0, 1, 0.6, 0.8))), new(new Circle2D(Ax(1, 1, 0, 1), 2.0)),
            new(new Ellipse2D(Ax(0.5, -1, 1, 1), 3.0, 1.0)), new(theParabola), new(theHyperbola)
        ];
        Point2D P = new(2.3, 0.7);
        const double h = 1.0e-6;
        foreach (IntCurve_IConicTool T in tools)
        {
            Vector2D G = T.GradDistance(P);
            double gx = (T.Distance(new Point2D(P.X() + h, P.Y())) - T.Distance(new Point2D(P.X() - h, P.Y())))
                        / (2 * h);
            double gy = (T.Distance(new Point2D(P.X(), P.Y() + h)) - T.Distance(new Point2D(P.X(), P.Y() - h)))
                        / (2 * h);
            Assert.Equal(gx, G.X(), 5);
            Assert.Equal(gy, G.Y(), 5);
        }
    }

    [Fact]
    public void PConicTool_MatchesElCLib()
    {
        IntCurve_PConic E = new(theEllipse);
        Assert.Equal(Ellipse, E.TypeCurve());
        Assert.Equal(20, IntCurve_PConicTool.NbSamples(E));
        Assert.Equal(50, IntCurve_PConicTool.NbSamples(new IntCurve_PConic(theHyperbola), 0, 1));
        Assert.Equal(7, IntCurve_PConicTool.NbSamples(E.WithAccuracy(7)));
        Assert.Equal(1.0e-8, IntCurve_PConicTool.EpsX(E));
        for (double u = -2; u <= 2; u += 0.37)
        {
            Assert.Equal(global::Kerf.Engine.Geometry.ElCLib.Value(u, theEllipse), IntCurve_PConicTool.Value(E, u));
            IntCurve_PConicTool.D2(E, u, out Point2D P, out Vector2D T, out Vector2D N);
            global::Kerf.Engine.Geometry.ElCLib.D2(u, theEllipse, out Point2D P0, out Vector2D T0, out Vector2D N0);
            Assert.Equal(P0, P);
            Assert.Equal(T0, T);
            Assert.Equal(N0, N);
            Assert.Equal(global::Kerf.Engine.Geometry.ElCLib.Value(u, theParabola), IntCurve_PConicTool.Value(new IntCurve_PConic(theParabola), u));
            Assert.Equal(global::Kerf.Engine.Geometry.ElCLib.Value(u, theHyperbola), IntCurve_PConicTool.Value(new IntCurve_PConic(theHyperbola), u));
        }
    }

    [Fact]
    public void ProjectOnPConicTool_ClampsToTheInterval()
    {
        IntCurve_PConic C = new(theUnitCircle);
        Point2D P = new(0, -3);
        Assert.Equal(3 * Math.PI / 2, IntCurve_ProjectOnPConicTool.FindParameter(C, P, 0.0), 14);
        Assert.Equal(1.0, IntCurve_ProjectOnPConicTool.FindParameter(C, P, 0.5, 1.0, 0.0), 14);
        Assert.Equal(3 * Math.PI / 2, IntCurve_ProjectOnPConicTool.FindParameter(C, P, 1.0, 1.0, 0.0), 14);
        IntCurve_PConic L = new(new Line2D(Ax(0, 0, 1, 0)));
        Assert.Equal(-2.0, IntCurve_ProjectOnPConicTool.FindParameter(L, new Point2D(5, 1), -2.0, -4.0, 0.0), 14);
    }
}
