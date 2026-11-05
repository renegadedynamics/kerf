// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass3d/BRepClass3d_Intersector3d.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass3d/BRepClass3d_Intersector3d.lxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass3d/BRepClass3d_Intersector3d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Intersection.CurveSurface.TransitionOnCurve;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Classification;

/// <summary>
/// The intersection of a line with a face, keeping only the point nearest to the line's origin (at a parameter above
/// -Tol) that is IN or ON the face.
/// </summary>
/// <remarks>
/// A working object (decision 21). Before the first point is found the transition is OCCT's uninitialised value; the
/// port's is IntCurveSurface_Tangent (the enum's zero).
/// </remarks>
internal class Intersector3d
{
    private Point3D _pnt = new();
    private double _u;
    private double _v;
    private double _w;
    private TransitionOnCurve _transition;
    private bool _done;
    private bool _hasapoint;
    private Containment _state;
    private Face _face = new();

    /// <summary>An empty intersector.</summary>
    public Intersector3d()
    {
        _u = 0.0;
        _v = 0.0;
        _w = 0.0;
        _done = false;
        _hasapoint = false;
        _state = Unknown;
    }

    /// <summary>
    /// Intersects the line <paramref name="l"/> with the face <paramref name="face"/>, keeping the point with the
    /// smallest parameter on the line above -<paramref name="tol"/>. <paramref name="prm"/> is unused (OCCT's).
    /// </summary>
    public void Perform(in Line3D l, double prm, double tol, in Face face)
    {
        var hics = new HInter();

        var surface = new FaceSurfaceAdaptor();
        var classifier2d = new FaceClassifier();

        var geomline = new LineCurve3D(l);
        // C++ builds adaptors and copies them into handles: one adaptor of each here
        var ll = new CurveAdaptor(geomline);

        surface.Initialize(face, true);

        bool isUPer = surface.IsUPeriodic;
        bool isVPer = surface.IsVPeriodic;
        double uperiod = isUPer ? surface.UPeriod() : 0.0;
        double vperiod = isVPer ? surface.VPeriod() : 0.0;

        double u1 = surface.FirstUParameter();
        double u2 = surface.LastUParameter();
        double v1 = surface.FirstVParameter();
        double v2 = surface.LastVParameter();

        //--
        CurveAdaptor hll = ll;
        FaceSurfaceAdaptor hsurface = surface;
        //--
        hics.Perform(hll, hsurface);

        _w = RealLast();
        if (hics.IsDone)
        {
            for (int index = hics.PointCount; index >= 1; index--)
            {
                Point2D puv = new(hics.Point(index).U(), hics.Point(index).V());

                int n1 = 0;
                int n2 = 0;

                double x = puv.X;
                double y = puv.Y;

                if (isUPer)
                {
                    if (x > u2)
                    {
                        n1 = RealToInt((x - u1) / uperiod);
                    }

                    if (x < u1)
                    {
                        n1 = RealToInt((x - u2) / uperiod);
                    }

                    puv = puv.WithX(x - uperiod * n1);
                }

                if (isVPer)
                {
                    if (y > v2)
                    {
                        n2 = RealToInt((y - v1) / vperiod);
                    }

                    if (y < v1)
                    {
                        n2 = RealToInt((y - v2) / vperiod);
                    }

                    puv = puv.WithY(y - vperiod * n2);
                }

                classifier2d.Perform(face, puv, tol);
                Containment currentstate = classifier2d.State();
                if (currentstate == global::Kerf.Topology.Containment.In || currentstate == On)
                {
                    CurveSurfaceIntersectionPoint hicsPoint = hics.Point(index);
                    double hicsw = hicsPoint.W();
                    //  Modified by skv - Fri Mar  4 12:07:34 2005 OCC7966 Begin
                    if ((_w > hicsw) && (hicsw > -tol))
                    {
                        // 	if(W > HICSW) {
                        //  Modified by skv - Fri Mar  4 12:07:34 2005 OCC7966 End
                        _hasapoint = true;
                        _u = hicsPoint.U();
                        _v = hicsPoint.V();
                        _w = hicsw;
                        _transition = hicsPoint.Transition();
                        _pnt = hicsPoint.Pnt();
                        _state = currentstate;
                        _face = face;
                        if (face.Orientation == Reversed)
                        {
                            if (_transition == TransitionOnCurve.In)
                            {
                                _transition = TransitionOnCurve.Out;
                            }
                            else
                            {
                                _transition = TransitionOnCurve.In;
                            }
                        }
                    }
                } //-- classifier state is IN or ON

                _done = true;
            } //-- Loop on Intersection points.
        } //-- HICS.IsDone()
    }

    /// <summary>True when the intersection has been computed (OCCT sets it only when the line meets the surface).</summary>
    public bool IsDone => (_done);

    /// <summary>True if a point has been found.</summary>
    public bool HasAPoint => (_hasapoint);

    /// <summary>The point.</summary>
    public Point3D Pnt() => (_pnt);

    /// <summary>The U parameter of the point on the surface.</summary>
    public double UParameter() => (_u);

    /// <summary>The V parameter of the point on the surface.</summary>
    public double VParameter() => (_v);

    /// <summary>The parameter of the point on the line.</summary>
    public double WParameter() => (_w);

    /// <summary>The transition of the line on the face (taking the face's orientation into account).</summary>
    public TransitionOnCurve Transition() => (_transition);

    /// <summary>The state of the point in the face: IN, or ON its boundary.</summary>
    public Containment State() => (_state);

    /// <summary>The face.</summary>
    public Face Face() => (_face);
}
