// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_ExtPF.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_ExtPF.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Occt.Extrema_ExtAlgo;
using static Occt.Extrema_ExtFlag;
using static Kerf.Geometry.SurfaceKind;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>The extrema of the distance between a vertex and a face (points inside the face or on its boundary).</summary>
/// <remarks>A working object (decision 21): initialize with a face, then perform with any number of vertices.</remarks>
public sealed class BRepExtrema_ExtPF
{
    private readonly Extrema_ExtPS myExtPS = new();
    private readonly NCollection_Sequence<double> mySqDist = new();
    private readonly NCollection_Sequence<Extrema_POnSurf> myPoints = new();
    private readonly BRepAdaptor_Surface mySurf = new();

    public BRepExtrema_ExtPF()
    {
    }

    /// <summary>Computes all the extrema between the vertex and the face.</summary>
    public BRepExtrema_ExtPF(in TopoDS_Vertex TheVertex,
                             in TopoDS_Face TheFace,
                             Extrema_ExtFlag TheFlag = Extrema_ExtFlag_MINMAX,
                             Extrema_ExtAlgo TheAlgo = Extrema_ExtAlgo_Grad)
    {
        Initialize(TheFace, TheFlag, TheAlgo);
        Perform(TheVertex, TheFace);
    }

    public void Initialize(in TopoDS_Face TheFace,
                           Extrema_ExtFlag TheFlag = Extrema_ExtFlag_MINMAX,
                           Extrema_ExtAlgo TheAlgo = Extrema_ExtAlgo_Grad)
    {
        // cette surface doit etre en champ. Extrema ne fait
        // pas de copie et prend seulement un pointeur dessus.
        mySurf.Initialize(TheFace, false);

        if (mySurf.GetType() == OtherSurface)
        {
            return; // protect against non-geometric type (e.g. triangulation)
        }

        double Tol = Min(BRep_Tool.Tolerance(TheFace), global::Kerf.Engine.Foundation.Precision.Confusion());
        double aTolU, aTolV;
        aTolU = Max(mySurf.UResolution(Tol), global::Kerf.Engine.Foundation.Precision.PConfusion());
        aTolV = Max(mySurf.VResolution(Tol), global::Kerf.Engine.Foundation.Precision.PConfusion());
        double U1, U2, V1, V2;
        BRepTools.UVBounds(TheFace, out U1, out U2, out V1, out V2);
        myExtPS.SetFlag(TheFlag);
        myExtPS.SetAlgo(TheAlgo);
        myExtPS.Initialize(mySurf, U1, U2, V1, V2, aTolU, aTolV);
    }

    /// <summary>
    /// Computes the extrema for the vertex. The face is used only to classify the points, not for the surface (which
    /// is the one given to Initialize).
    /// </summary>
    public void Perform(in TopoDS_Vertex TheVertex, in TopoDS_Face TheFace)
    {
        mySqDist.Clear();
        myPoints.Clear();

        Point3D P = BRep_Tool.Pnt(TheVertex);
        if (mySurf.GetType() == OtherSurface)
        {
            return; // protect against non-geometric type (e.g. triangulation)
        }

        myExtPS.Perform(P);

        // Exploration of points and classification
        if (myExtPS.IsDone())
        {
            var classifier = new BRepClass_FaceClassifier();
            double U1, U2;
            double Tol = BRep_Tool.Tolerance(TheFace);
            for (int i = 1; i <= myExtPS.NbExt(); i++)
            {
                myExtPS.Point(i).Parameter(out U1, out U2);
                var Puv = new Point2D(U1, U2);
                classifier.Perform(TheFace, Puv, Tol);
                TopAbs_State state = classifier.State();
                if (state == TopAbs_ON || state == TopAbs_IN)
                {
                    mySqDist.Append(myExtPS.SquareDistance(i));
                    myPoints.Append(myExtPS.Point(i));
                }
            }
        }
    }

    /// <summary>True if the distances are found.</summary>
    public bool IsDone() => myExtPS.IsDone();

    /// <summary>The number of extremum distances.</summary>
    public int NbExt() => myPoints.Length();

    /// <summary>The <paramref name="N"/>th extremum square distance.</summary>
    public double SquareDistance(int N) => mySqDist.Value(N);

    /// <summary>The parameters on the face of the <paramref name="N"/>th extremum distance.</summary>
    public void Parameter(int N, out double U, out double V) => myPoints.Value(N).Parameter(out U, out V);

    /// <summary>The point of the <paramref name="N"/>th extremum distance.</summary>
    public Point3D Point(int N) => myPoints.Value(N).Value();

    public void SetFlag(Extrema_ExtFlag F) => myExtPS.SetFlag(F);

    public void SetAlgo(Extrema_ExtAlgo A) => myExtPS.SetAlgo(A);
}
