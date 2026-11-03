// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass3d/BRepClass3d_Intersector3d.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass3d/BRepClass3d_Intersector3d.lxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass3d/BRepClass3d_Intersector3d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.IntCurveSurface_TransitionOnCurve;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Occt;

/// <summary>
/// The intersection of a line with a face, keeping only the point nearest to the line's origin (at a parameter above
/// -Tol) that is IN or ON the face.
/// </summary>
/// <remarks>
/// A working object (decision 21). Before the first point is found the transition is OCCT's uninitialised value; the
/// port's is IntCurveSurface_Tangent (the enum's zero).
/// </remarks>
public class BRepClass3d_Intersector3d
{
    private Point3D pnt = new();
    private double U;
    private double V;
    private double W;
    private IntCurveSurface_TransitionOnCurve transition;
    private bool done;
    private bool hasapoint;
    private Containment state;
    private Face face = new();

    /// <summary>An empty intersector.</summary>
    public BRepClass3d_Intersector3d()
    {
        U = 0.0;
        V = 0.0;
        W = 0.0;
        done = false;
        hasapoint = false;
        state = Unknown;
    }

    /// <summary>
    /// Intersects the line <paramref name="L"/> with the face <paramref name="Face"/>, keeping the point with the
    /// smallest parameter on the line above -<paramref name="Tol"/>. <paramref name="Prm"/> is unused (OCCT's).
    /// </summary>
    public void Perform(in Line3D L, double Prm, double Tol, in Face Face)
    {
        var HICS = new IntCurveSurface_HInter();

        var surface = new FaceSurfaceAdaptor();
        var classifier2d = new BRepClass_FaceClassifier();

        var geomline = new LineCurve3D(L);
        // C++ builds adaptors and copies them into handles: one adaptor of each here
        var LL = new CurveAdaptor(geomline);

        surface.Initialize(Face, true);

        bool IsUPer = surface.IsUPeriodic;
        bool IsVPer = surface.IsVPeriodic;
        double uperiod = IsUPer ? surface.UPeriod() : 0.0;
        double vperiod = IsVPer ? surface.VPeriod() : 0.0;

        double U1 = surface.FirstUParameter();
        double U2 = surface.LastUParameter();
        double V1 = surface.FirstVParameter();
        double V2 = surface.LastVParameter();

        //--
        CurveAdaptor HLL = LL;
        FaceSurfaceAdaptor Hsurface = surface;
        //--
        HICS.Perform(HLL, Hsurface);

        W = RealLast();
        if (HICS.IsDone())
        {
            for (int index = HICS.NbPoints(); index >= 1; index--)
            {
                Point2D Puv = new(HICS.Point(index).U(), HICS.Point(index).V());

                int N1 = 0;
                int N2 = 0;

                double X = Puv.X();
                double Y = Puv.Y();

                if (IsUPer)
                {
                    if (X > U2)
                    {
                        N1 = RealToInt((X - U1) / uperiod);
                    }

                    if (X < U1)
                    {
                        N1 = RealToInt((X - U2) / uperiod);
                    }

                    Puv.SetX(X - uperiod * N1);
                }

                if (IsVPer)
                {
                    if (Y > V2)
                    {
                        N2 = RealToInt((Y - V1) / vperiod);
                    }

                    if (Y < V1)
                    {
                        N2 = RealToInt((Y - V2) / vperiod);
                    }

                    Puv.SetY(Y - vperiod * N2);
                }

                classifier2d.Perform(Face, Puv, Tol);
                Containment currentstate = classifier2d.State();
                if (currentstate == In || currentstate == On)
                {
                    IntCurveSurface_IntersectionPoint HICSPoint = HICS.Point(index);
                    double HICSW = HICSPoint.W();
                    //  Modified by skv - Fri Mar  4 12:07:34 2005 OCC7966 Begin
                    if ((W > HICSW) && (HICSW > -Tol))
                    {
                        // 	if(W > HICSW) {
                        //  Modified by skv - Fri Mar  4 12:07:34 2005 OCC7966 End
                        hasapoint = true;
                        U = HICSPoint.U();
                        V = HICSPoint.V();
                        W = HICSW;
                        transition = HICSPoint.Transition();
                        pnt = HICSPoint.Pnt();
                        state = currentstate;
                        face = Face;
                        if (Face.Orientation() == Reversed)
                        {
                            if (transition == IntCurveSurface_In)
                            {
                                transition = IntCurveSurface_Out;
                            }
                            else
                            {
                                transition = IntCurveSurface_In;
                            }
                        }
                    }
                } //-- classifier state is IN or ON

                done = true;
            } //-- Loop on Intersection points.
        } //-- HICS.IsDone()
    }

    /// <summary>True when the intersection has been computed (OCCT sets it only when the line meets the surface).</summary>
    public bool IsDone() => (done);

    /// <summary>True if a point has been found.</summary>
    public bool HasAPoint() => (hasapoint);

    /// <summary>The point.</summary>
    public Point3D Pnt() => (pnt);

    /// <summary>The U parameter of the point on the surface.</summary>
    public double UParameter() => (U);

    /// <summary>The V parameter of the point on the surface.</summary>
    public double VParameter() => (V);

    /// <summary>The parameter of the point on the line.</summary>
    public double WParameter() => (W);

    /// <summary>The transition of the line on the face (taking the face's orientation into account).</summary>
    public IntCurveSurface_TransitionOnCurve Transition() => (transition);

    /// <summary>The state of the point in the face: IN, or ON its boundary.</summary>
    public Containment State() => (state);

    /// <summary>The face.</summary>
    public Face Face() => (face);
}
