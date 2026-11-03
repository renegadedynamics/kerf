// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass/BRepClass_FaceClassifier.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass/BRepClass_FaceClassifier.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Classifies a point in a face: a 2D point in its parameter space, or a 3D point through its nearest projection on
/// the face's surface.
/// </summary>
/// <remarks>A working object (decision 21). It only reads the face.</remarks>
public class BRepClass_FaceClassifier : BRepClass_FClassifier
{
    /// <summary>An undefined classifier.</summary>
    public BRepClass_FaceClassifier()
    {
    }

    /// <summary>Classifies <paramref name="P"/> with the tolerance <paramref name="Tol"/> in the face explored by <paramref name="F"/>.</summary>
    public BRepClass_FaceClassifier(BRepClass_FaceExplorer F, in Point2D P, double Tol)
        : base(F, P, Tol)
    {
    }

    /// <summary>
    /// Classifies the 3D point <paramref name="theP"/> with the tolerance <paramref name="theTol"/> on the face
    /// <paramref name="theF"/> (see <see cref="Perform(in TopoDS_Face, in Point3D, double, bool, double)"/>).
    /// </summary>
    public BRepClass_FaceClassifier(in TopoDS_Face theF,
                                    in Point3D theP,
                                    double theTol,
                                    bool theUseBndBox = false,
                                    double theGapCheckTol = 0.1)
    {
        Perform(theF, theP, theTol, theUseBndBox, theGapCheckTol);
    }

    /// <summary>
    /// Classifies the 2D point <paramref name="theP"/> with the tolerance <paramref name="theTol"/> in the face
    /// <paramref name="theF"/>. Bounding boxes are recommended (<paramref name="theUseBndBox"/>) for faces of more than
    /// 10 edges, mostly B-splines; <paramref name="theGapCheckTol"/> is the vertex tolerance from which the gaps of
    /// the wires are checked.
    /// </summary>
    public BRepClass_FaceClassifier(in TopoDS_Face theF,
                                    in Point2D theP,
                                    double theTol,
                                    bool theUseBndBox = false,
                                    double theGapCheckTol = 0.1)
    {
        Perform(theF, theP, theTol, theUseBndBox, theGapCheckTol);
    }

    /// <summary>
    /// Classifies the 2D point <paramref name="theP"/> with the tolerance <paramref name="theTol"/> in the face
    /// <paramref name="theF"/> (see the constructor).
    /// </summary>
    public void Perform(in TopoDS_Face theF,
                        in Point2D theP,
                        double theTol,
                        bool theUseBndBox = false,
                        double theGapCheckTol = 0.1)
    {
        var aFex = new BRepClass_FaceExplorer(theF);
        aFex.SetMaxTolerance(theGapCheckTol);
        aFex.SetUseBndBox(theUseBndBox);
        Perform(aFex, theP, theTol);
    }

    /// <summary>
    /// Classifies the 3D point <paramref name="theP"/> on the face <paramref name="theF"/>: the 2D point of its
    /// nearest projection on the surface (within the face's UV bounds) is classified. The point is rejected (OUT)
    /// when it has no projection.
    /// </summary>
    public void Perform(in TopoDS_Face theF,
                        in Point3D theP,
                        double theTol,
                        bool theUseBndBox = false,
                        double theGapCheckTol = 0.1)
    {
        int aNbExt, aIndice, i;
        double aU1, aU2, aV1, aV2, aMaxDist, aD;
        Point2D aPuv;
        var aExtrema = new Extrema_ExtPS();
        //
        aMaxDist = RealLast();
        aIndice = 0;
        //
        var aSurf = new BRepAdaptor_Surface(theF, false);
        BRepTools.UVBounds(theF, out aU1, out aU2, out aV1, out aV2);
        aExtrema.Initialize(aSurf, aU1, aU2, aV1, aV2, theTol, theTol);
        //
        // modified by NIZNHY-PKV Wed Aug 13 11:28:47 2008f
        rejected = true;
        // modified by NIZNHY-PKV Wed Aug 13 11:28:49 2008t
        aExtrema.Perform(theP);
        if (!aExtrema.IsDone())
        {
            return;
        }

        //
        aNbExt = aExtrema.NbExt();
        if (aNbExt == 0)
        {
            return;
        }

        //
        for (i = 1; i <= aNbExt; ++i)
        {
            aD = aExtrema.SquareDistance(i);
            if (aD < aMaxDist)
            {
                aMaxDist = aD;
                aIndice = i;
            }
        }

        //
        if (aIndice != 0)
        {
            aExtrema.Point(aIndice).Parameter(out aU1, out aU2);
            aPuv = new Point2D(aU1, aU2);
            Perform(theF, aPuv, theTol, theUseBndBox, theGapCheckTol);
        }
    }
}
