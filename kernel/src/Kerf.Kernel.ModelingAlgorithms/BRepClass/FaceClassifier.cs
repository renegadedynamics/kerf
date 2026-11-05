// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass/BRepClass_FaceClassifier.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass/BRepClass_FaceClassifier.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Classification;

/// <summary>
/// Classifies a point in a face: a 2D point in its parameter space, or a 3D point through its nearest projection on
/// the face's surface.
/// </summary>
/// <remarks>A working object (decision 21). It only reads the face.</remarks>
internal class FaceClassifier : FClassifier
{
    /// <summary>An undefined classifier.</summary>
    public FaceClassifier()
    {
    }

    /// <summary>Classifies <paramref name="p"/> with the tolerance <paramref name="tol"/> in the face explored by <paramref name="f"/>.</summary>
    public FaceClassifier(FaceExplorer f, in Point2D p, double tol)
        : base(f, p, tol)
    {
    }

    /// <summary>
    /// Classifies the 3D point <paramref name="p"/> with the tolerance <paramref name="tol"/> on the face
    /// <paramref name="f"/> (see <see cref="Perform(in Face, in Point3D, double, bool, double)"/>).
    /// </summary>
    public FaceClassifier(in Face f,
                                    in Point3D p,
                                    double tol,
                                    bool useBndBox = false,
                                    double gapCheckTol = 0.1)
    {
        Perform(f, p, tol, useBndBox, gapCheckTol);
    }

    /// <summary>
    /// Classifies the 2D point <paramref name="p"/> with the tolerance <paramref name="tol"/> in the face
    /// <paramref name="f"/>. Bounding boxes are recommended (<paramref name="useBndBox"/>) for faces of more than
    /// 10 edges, mostly B-splines; <paramref name="gapCheckTol"/> is the vertex tolerance from which the gaps of
    /// the wires are checked.
    /// </summary>
    public FaceClassifier(in Face f,
                                    in Point2D p,
                                    double tol,
                                    bool useBndBox = false,
                                    double gapCheckTol = 0.1)
    {
        Perform(f, p, tol, useBndBox, gapCheckTol);
    }

    /// <summary>
    /// Classifies the 2D point <paramref name="p"/> with the tolerance <paramref name="tol"/> in the face
    /// <paramref name="f"/> (see the constructor).
    /// </summary>
    public void Perform(in Face f,
                        in Point2D p,
                        double tol,
                        bool useBndBox = false,
                        double gapCheckTol = 0.1)
    {
        var fex = new FaceExplorer(f);
        fex.SetMaxTolerance(gapCheckTol);
        fex.SetUseBndBox(useBndBox);
        Perform(fex, p, tol);
    }

    /// <summary>
    /// Classifies the 3D point <paramref name="p"/> on the face <paramref name="f"/>: the 2D point of its
    /// nearest projection on the surface (within the face's UV bounds) is classified. The point is rejected (OUT)
    /// when it has no projection.
    /// </summary>
    public void Perform(in Face f,
                        in Point3D p,
                        double tol,
                        bool useBndBox = false,
                        double gapCheckTol = 0.1)
    {
        int nbExt, indice, i;
        double u1, u2, v1, v2, maxDist, d;
        Point2D puv;
        var extrema = new ExtPS();
        //
        maxDist = RealLast();
        indice = 0;
        //
        var surf = new FaceSurfaceAdaptor(f, false);
        BrepTools.UVBounds(f, out u1, out u2, out v1, out v2);
        extrema.Initialize(surf, u1, u2, v1, v2, tol, tol);
        //
        // modified by NIZNHY-PKV Wed Aug 13 11:28:47 2008f
        _rejected = true;
        // modified by NIZNHY-PKV Wed Aug 13 11:28:49 2008t
        extrema.Perform(p);
        if (!extrema.IsDone)
        {
            return;
        }

        //
        nbExt = extrema.ExtCount;
        if (nbExt == 0)
        {
            return;
        }

        //
        for (i = 1; i <= nbExt; ++i)
        {
            d = extrema.SquareDistance(i);
            if (d < maxDist)
            {
                maxDist = d;
                indice = i;
            }
        }

        //
        if (indice != 0)
        {
            extrema.Point(indice).Parameter(out u1, out u2);
            puv = new Point2D(u1, u2);
            Perform(f, puv, tol, useBndBox, gapCheckTol);
        }
    }
}
