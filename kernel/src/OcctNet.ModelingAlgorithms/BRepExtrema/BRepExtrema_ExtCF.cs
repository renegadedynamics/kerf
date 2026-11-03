// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_ExtCF.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_ExtCF.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Kerf.Geometry.SurfaceKind;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>The extrema of the distance between an edge and a face (points inside the face or on its boundary).</summary>
/// <remarks>
/// A working object (decision 21). OCCT copies a local surface adaptor into a new handle; the port keeps the adaptor
/// it made (decision 394).
/// </remarks>
public sealed class BRepExtrema_ExtCF
{
    private readonly Extrema_ExtCS myExtCS = new();
    private readonly NCollection_Sequence<double> mySqDist = new();
    private readonly NCollection_Sequence<Extrema_POnSurf> myPointsOnS = new();
    private readonly NCollection_Sequence<Extrema_POnCurv> myPointsOnC = new();
    private BRepAdaptor_Surface? myHS;

    public BRepExtrema_ExtCF()
    {
    }

    /// <summary>Computes all the extrema between the edge and the face.</summary>
    public BRepExtrema_ExtCF(in TopoDS_Edge E, in TopoDS_Face F)
    {
        Initialize(E, F);
        Perform(E, F);
    }

    public void Initialize(in TopoDS_Edge E, in TopoDS_Face F)
    {
        var Surf = new BRepAdaptor_Surface(F);
        if (Surf.GetType() == OtherSurface || !BRep_Tool.IsGeometric(E))
        {
            return; // protect against non-geometric type (e.g. triangulation)
        }
        var aC = new BRepAdaptor_Curve(E);
        myHS = Surf;
        double aTolC, aTolS;
        //
        aTolS = Min(BRep_Tool.Tolerance(F), global::Kerf.Engine.Foundation.Precision.Confusion());
        aTolS = Min(Surf.UResolution(aTolS), Surf.VResolution(aTolS));
        aTolS = Max(aTolS, global::Kerf.Engine.Foundation.Precision.PConfusion());
        //
        aTolC = Min(BRep_Tool.Tolerance(E), global::Kerf.Engine.Foundation.Precision.Confusion());
        aTolC = aC.Resolution(aTolC);
        aTolC = Max(aTolC, global::Kerf.Engine.Foundation.Precision.PConfusion());
        //
        double U1, U2, V1, V2;
        BRepTools.UVBounds(F, out U1, out U2, out V1, out V2);
        myExtCS.Initialize(myHS, U1, U2, V1, V2, aTolC, aTolS);
    }

    /// <summary>
    /// Computes the extrema for the edge. The face is used only to classify the points, not for the surface (which
    /// is the one given to Initialize).
    /// </summary>
    public void Perform(in TopoDS_Edge E, in TopoDS_Face F2)
    {
        mySqDist.Clear();
        myPointsOnS.Clear();
        myPointsOnC.Clear();

        if (myHS is null)
        {
            return; // protect against non-geometric type (e.g. triangulation)
        }

        double U1, U2;
        BRep_Tool.Range(E, out U1, out U2);

        var Curv = new BRepAdaptor_Curve(E);
        BRepAdaptor_Curve HC = Curv;
        myExtCS.Perform(HC, U1, U2);

        if (!myExtCS.IsDone())
        {
            return;
        }

        if (myExtCS.IsParallel())
        {
            mySqDist.Append(myExtCS.SquareDistance(1));
        }
        else
        {
            // Exploration of points and classification
            double Tol = BRep_Tool.Tolerance(F2);
            var classifier = new BRepTopAdaptor_FClass2d(F2, Tol);

            // If the underlying surface of the face is periodic
            // Extrema should return the point within the period,
            // so there is no point to adjust it in classifier.
            bool isAdjustPeriodic = false;

            for (int i = 1; i <= myExtCS.NbExt(); i++)
            {
                myExtCS.Points(i, out Extrema_POnCurv P1, out Extrema_POnSurf P2);
                P2.Parameter(out U1, out U2);
                var Puv = new Point2D(U1, U2);
                TopAbs_State state = classifier.Perform(Puv, isAdjustPeriodic);
                if (state == TopAbs_ON || state == TopAbs_IN)
                {
                    mySqDist.Append(myExtCS.SquareDistance(i));
                    myPointsOnC.Append(P1);
                    myPointsOnS.Append(P2);
                }
            }
        }
    }

    /// <summary>True if the distances are found.</summary>
    public bool IsDone() => myExtCS.IsDone();

    /// <summary>The number of extremum distances.</summary>
    public int NbExt() => mySqDist.Length();

    /// <summary>The <paramref name="N"/>th extremum square distance.</summary>
    public double SquareDistance(int N) => mySqDist.Value(N);

    /// <summary>True if the curve is on a parallel surface.</summary>
    public bool IsParallel() => myExtCS.IsParallel();

    /// <summary>The parameter on the edge of the <paramref name="N"/>th extremum distance.</summary>
    public double ParameterOnEdge(int N) => myPointsOnC.Value(N).Parameter();

    /// <summary>The parameters on the face of the <paramref name="N"/>th extremum distance.</summary>
    public void ParameterOnFace(int N, out double U, out double V) => myPointsOnS.Value(N).Parameter(out U, out V);

    /// <summary>The point on the edge of the <paramref name="N"/>th extremum distance.</summary>
    public Point3D PointOnEdge(int N) => myPointsOnC.Value(N).Value();

    /// <summary>The point on the face of the <paramref name="N"/>th extremum distance.</summary>
    public Point3D PointOnFace(int N) => myPointsOnS.Value(N).Value();
}
