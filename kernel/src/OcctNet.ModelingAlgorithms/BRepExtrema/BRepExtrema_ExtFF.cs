// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_ExtFF.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_ExtFF.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Containment;

namespace Occt;

/// <summary>The extrema of the distance between two faces (points inside the faces or on their boundaries).</summary>
/// <remarks>
/// A working object (decision 21). OCCT copies local surface adaptors into new handles; the port keeps the adaptors
/// it made (decision 394).
/// </remarks>
public sealed class BRepExtrema_ExtFF
{
    private readonly ExtSS myExtSS = new();
    private readonly NCollection_Sequence<double> mySqDist = new();
    private readonly NCollection_Sequence<POnSurf> myPointsOnS1 = new();
    private readonly NCollection_Sequence<POnSurf> myPointsOnS2 = new();
    private FaceSurfaceAdaptor? myHS;

    public BRepExtrema_ExtFF()
    {
    }

    /// <summary>Computes all the extrema between the faces.</summary>
    public BRepExtrema_ExtFF(in Face F1, in Face F2)
    {
        Initialize(F2);
        Perform(F1, F2);
    }

    public void Initialize(in Face F2)
    {
        var Surf = new FaceSurfaceAdaptor(F2);
        if (Surf.GetType() == OtherSurface)
        {
            return; // protect against non-geometric type (e.g. triangulation)
        }

        myHS = Surf;
        double Tol = Min(Tool.Tolerance(F2), global::Kerf.Engine.Foundation.Precision.Confusion());
        Tol = Min(Surf.UResolution(Tol), Surf.VResolution(Tol));
        Tol = Max(Tol, global::Kerf.Engine.Foundation.Precision.PConfusion());
        double U1, U2, V1, V2;
        BrepTools.UVBounds(F2, out U1, out U2, out V1, out V2);
        myExtSS.Initialize(myHS, U1, U2, V1, V2, Tol);
    }

    /// <summary>
    /// Computes the extrema for the face <paramref name="F1"/>. The face <paramref name="F2"/> is used only to classify
    /// the points, not for the surface (which is the one given to Initialize).
    /// </summary>
    public void Perform(in Face F1, in Face F2)
    {
        mySqDist.Clear();
        myPointsOnS1.Clear();
        myPointsOnS2.Clear();

        var Surf1 = new FaceSurfaceAdaptor(F1);
        if (myHS is null || Surf1.GetType() == OtherSurface)
        {
            return; // protect against non-geometric type (e.g. triangulation)
        }

        FaceSurfaceAdaptor HS1 = Surf1;
        double Tol1 = Min(Tool.Tolerance(F1), global::Kerf.Engine.Foundation.Precision.Confusion());
        Tol1 = Min(Surf1.UResolution(Tol1), Surf1.VResolution(Tol1));
        Tol1 = Max(Tol1, global::Kerf.Engine.Foundation.Precision.PConfusion());
        double U1, U2, V1, V2;
        BrepTools.UVBounds(F1, out U1, out U2, out V1, out V2);
        myExtSS.Perform(HS1, U1, U2, V1, V2, Tol1);

        if (!myExtSS.IsDone)
        {
            return;
        }

        if (myExtSS.IsParallel)
        {
            mySqDist.Append(myExtSS.SquareDistance(1));
        }
        else
        {
            // Exploration of points and classification
            var classifier = new BRepClass_FaceClassifier();
            double Tol2 = Tool.Tolerance(F2);

            int i;
            for (i = 1; i <= myExtSS.ExtCount; i++)
            {
                myExtSS.Points(i, out POnSurf P1, out POnSurf P2);
                P1.Parameter(out U1, out U2);
                var Puv1 = new Point2D(U1, U2);
                classifier.Perform(F1, Puv1, Tol1);
                Containment state1 = classifier.State();
                if (state1 == On || state1 == In)
                {
                    P2.Parameter(out U1, out U2);
                    var Puv2 = new Point2D(U1, U2);
                    classifier.Perform(F2, Puv2, Tol2);
                    Containment state2 = classifier.State();
                    if (state2 == On || state2 == In)
                    {
                        mySqDist.Append(myExtSS.SquareDistance(i));
                        myPointsOnS1.Append(P1);
                        myPointsOnS2.Append(P2);
                    }
                }
            }
        }
    }

    /// <summary>True if the distances are found.</summary>
    public bool IsDone() => myExtSS.IsDone;

    /// <summary>True if the surfaces are parallel.</summary>
    public bool IsParallel() => myExtSS.IsParallel;

    /// <summary>The number of extremum distances.</summary>
    public int NbExt() => mySqDist.Length();

    /// <summary>The <paramref name="N"/>th extremum square distance.</summary>
    public double SquareDistance(int N) => mySqDist.Value(N);

    /// <summary>The parameters on the face F1 of the <paramref name="N"/>th extremum distance.</summary>
    public void ParameterOnFace1(int N, out double U, out double V) => myPointsOnS1.Value(N).Parameter(out U, out V);

    /// <summary>The parameters on the face F2 of the <paramref name="N"/>th extremum distance.</summary>
    public void ParameterOnFace2(int N, out double U, out double V) => myPointsOnS2.Value(N).Parameter(out U, out V);

    /// <summary>The point on the face F1 of the <paramref name="N"/>th extremum distance.</summary>
    public Point3D PointOnFace1(int N) => myPointsOnS1.Value(N).Value();

    /// <summary>The point on the face F2 of the <paramref name="N"/>th extremum distance.</summary>
    public Point3D PointOnFace2(int N) => myPointsOnS2.Value(N).Value();
}
