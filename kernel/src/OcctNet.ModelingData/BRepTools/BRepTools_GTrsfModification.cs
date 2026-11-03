// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_GTrsfModification.hxx
// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_GTrsfModification.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Meshing;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.TransformForm;

namespace Occt;

/// <summary>
/// The modification that applies a general transformation (gp_GTrsf, affine) to a shape whose surfaces and curves are
/// all B-splines or Beziers, by transforming their poles. Triangulations and polygons are transformed too.
/// </summary>
/// <remarks>
/// Any other surface or curve throws <see cref="NoSuchObjectException"/>, as in OCCT (BRepBuilderAPI_GTransform
/// converts the shape to NURBS first).
/// </remarks>
public class BRepTools_GTrsfModification : BRepTools_Modification
{
    private GeneralTransform3D myGTrsf;
    private readonly double myGScale;

    /// <summary>The modification applying <paramref name="T"/>; tolerances grow by the sup norm of its matrix.</summary>
    public BRepTools_GTrsfModification(in GeneralTransform3D T)
    {
        myGTrsf = T;
        // use the sup norm as the maximum dilation for the tolerance
        double loc1, loc2, loc3, loc4;

        loc1 = Max(Math.Abs(T.Value(1, 1)), Math.Abs(T.Value(1, 2)));
        loc2 = Max(Math.Abs(T.Value(2, 1)), Math.Abs(T.Value(2, 2)));
        loc3 = Max(Math.Abs(T.Value(3, 1)), Math.Abs(T.Value(3, 2)));
        loc4 = Max(Math.Abs(T.Value(1, 3)), Math.Abs(T.Value(2, 3)));

        loc1 = Max(loc1, loc2);
        loc2 = Max(loc3, loc4);

        loc1 = Max(loc1, loc2);

        myGScale = Max(loc1, Math.Abs(T.Value(3, 3)));
    }

    /// <summary>The transformation, for reading and writing (a working object).</summary>
    public ref GeneralTransform3D GTrsf() => ref myGTrsf;

    /// <summary>
    /// The face's B-spline or Bezier surface placed and transformed, at the identity location; false if the face has
    /// no surface.
    /// </summary>
    /// <exception cref="NoSuchObjectException">The surface is neither a B-spline nor a Bezier surface.</exception>
    public override bool NewSurface(in TopoDS_Face F,
                                    out Geom_Surface? S,
                                    out Placement L,
                                    out double Tol,
                                    out bool RevWires,
                                    out bool RevFace)
    {
        Tol = 0.0;
        RevWires = RevFace = false;
        GeneralTransform3D gtrsf = new GeneralTransform3D();
        gtrsf.SetVectorialPart(myGTrsf.VectorialPart());
        gtrsf.SetTranslationPart(myGTrsf.TranslationPart());
        S = BRep_Tool.Surface(F, out L);
        if (S is null)
        {
            // processing the case when there is no geometry
            return false;
        }
        S = (Geom_Surface)S.Copy();

        Tol = BRep_Tool.Tolerance(F);
        Tol *= myGScale;
        RevWires = false;
        RevFace = myGTrsf.IsNegative;
        S = (Geom_Surface)S.Transformed(L.Transformation());

        if (S.GetType() == typeof(Geom_BSplineSurface))
        {
            Geom_BSplineSurface S2 = (Geom_BSplineSurface)S;
            for (int i = 1; i <= S2.NbUPoles(); i++)
            {
                for (int j = 1; j <= S2.NbVPoles(); j++)
                {
                    Xyz coor = S2.Pole(i, j).Coord();
                    gtrsf.Transforms(ref coor);
                    Point3D P = new Point3D(coor);
                    S2.SetPole(i, j, P);
                }
            }
        }
        else if (S.GetType() == typeof(Geom_BezierSurface))
        {
            Geom_BezierSurface S2 = (Geom_BezierSurface)S;
            for (int i = 1; i <= S2.NbUPoles(); i++)
            {
                for (int j = 1; j <= S2.NbVPoles(); j++)
                {
                    Xyz coor = S2.Pole(i, j).Coord();
                    gtrsf.Transforms(ref coor);
                    Point3D P = new Point3D(coor);
                    S2.SetPole(i, j, P);
                }
            }
        }
        else
        {
            throw new NoSuchObjectException("BRepTools_GTrsfModification : Pb no BSpline/Bezier Type Surface");
        }

        L.Identity();
        return true;
    }

    /// <summary>
    /// The edge's B-spline or Bezier curve placed, transformed and trimmed to the edge's range, at the identity
    /// location; false if the edge has no 3D curve.
    /// </summary>
    /// <exception cref="NoSuchObjectException">The curve is neither a B-spline nor a Bezier curve.</exception>
    public override bool NewCurve(in TopoDS_Edge E, out Geom_Curve? C, out Placement L, out double Tol)
    {
        GeneralTransform3D gtrsf = new GeneralTransform3D();
        gtrsf.SetVectorialPart(myGTrsf.VectorialPart());
        gtrsf.SetTranslationPart(myGTrsf.TranslationPart());
        Tol = BRep_Tool.Tolerance(E) * myGScale;
        C = BRep_Tool.Curve(E, out L, out double f, out double l);

        if (C is not null)
        {
            C = (Geom_Curve)C.Copy().Transformed(L.Transformation());
            if (C.GetType() == typeof(Geom_BSplineCurve))
            {
                Geom_BSplineCurve C2 = (Geom_BSplineCurve)C;
                for (int i = 1; i <= C2.NbPoles(); i++)
                {
                    Xyz coor = C2.Pole(i).Coord();
                    gtrsf.Transforms(ref coor);
                    Point3D P = new Point3D(coor);
                    C2.SetPole(i, P);
                }
            }
            else if (C.GetType() == typeof(Geom_BezierCurve))
            {
                Geom_BezierCurve C2 = (Geom_BezierCurve)C;
                for (int i = 1; i <= C2.NbPoles(); i++)
                {
                    Xyz coor = C2.Pole(i).Coord();
                    gtrsf.Transforms(ref coor);
                    Point3D P = new Point3D(coor);
                    C2.SetPole(i, P);
                }
            }
            else
            {
                throw new NoSuchObjectException("BRepTools_GTrsfModification : Pb no BSpline/Bezier Type Curve");
            }
            C = new Geom_TrimmedCurve(C, f, l);
        }
        L.Identity();
        return C is not null;
    }

    /// <summary>The vertex's point transformed; always true.</summary>
    public override bool NewPoint(in TopoDS_Vertex V, out Point3D P, out double Tol)
    {
        Point3D Pnt = BRep_Tool.Pnt(V);
        Tol = BRep_Tool.Tolerance(V);
        Tol *= myGScale;
        Xyz coor = Pnt.Coord();
        myGTrsf.Transforms(ref coor);
        P = new Point3D();
        P.SetXYZ(coor);

        return true;
    }

    /// <summary>The edge's pcurve on the face, trimmed to its range; false if it has none.</summary>
    public override bool NewCurve2d(in TopoDS_Edge E,
                                    in TopoDS_Face F,
                                    in TopoDS_Edge NewE,
                                    in TopoDS_Face NewF,
                                    out Geom2d_Curve? C,
                                    out double Tol)
    {
        Tol = BRep_Tool.Tolerance(E);
        Tol *= myGScale;
        C = BRep_Tool.CurveOnSurface(E, F, out double f, out double l);
        if (C is null)
        {
            // processing the case when there is no geometry
            return false;
        }
        C = new Geom2d_TrimmedCurve(C, f, l);
        return true;
    }

    /// <summary>The vertex's parameter on the edge, unchanged; always true.</summary>
    public override bool NewParameter(in TopoDS_Vertex V, in TopoDS_Edge E, ref double P, ref double Tol)
    {
        Tol = BRep_Tool.Tolerance(V);
        Tol *= myGScale;
        P = BRep_Tool.Parameter(V, E);
        return true;
    }

    /// <summary>The edge's continuity between the original faces.</summary>
    public override Continuity Continuity(in TopoDS_Edge E,
                                             in TopoDS_Face F1,
                                             in TopoDS_Face F2,
                                             in TopoDS_Edge NewE,
                                             in TopoDS_Face NewF1,
                                             in TopoDS_Face NewF2)
    {
        return BRep_Tool.Continuity(E, F1, F2);
    }

    /// <summary>The face's triangulation transformed; false if it has none.</summary>
    public override bool NewTriangulation(in TopoDS_Face theFace, out Triangulation? theTriangulation)
    {
        theTriangulation = BRep_Tool.Triangulation(theFace, out Placement aLoc);
        if (theTriangulation is null)
        {
            return false;
        }

        GeneralTransform3D aGTrsf = new GeneralTransform3D();
        aGTrsf.SetVectorialPart(myGTrsf.VectorialPart());
        aGTrsf.SetTranslationPart(myGTrsf.TranslationPart());
        aGTrsf.Multiply(new GeneralTransform3D(aLoc.Transformation()));

        theTriangulation = theTriangulation.Copy();
        theTriangulation.SetCachedMinMax(new Box()); // clear bounding box
        theTriangulation.Deflection(theTriangulation.Deflection() * Math.Abs(myGScale));
        // apply transformation to 3D nodes
        for (int anInd = 1; anInd <= theTriangulation.NodeCount; ++anInd)
        {
            Point3D aP = theTriangulation.Node(anInd);
            Xyz aXYZ = aP.XYZ();
            aGTrsf.Transforms(ref aXYZ);
            aP.SetXYZ(aXYZ);
            theTriangulation.SetNode(anInd, aP);
        }
        // modify triangles orientation in case of mirror transformation
        if (myGScale < 0.0)
        {
            for (int anInd = 1; anInd <= theTriangulation.TriangleCount; ++anInd)
            {
                Triangle1 aTria = theTriangulation.Triangle(anInd);
                aTria.Get(out int aN1, out int aN2, out int aN3);
                aTria.Set(aN1, aN3, aN2);
                theTriangulation.SetTriangle(anInd, aTria);
            }
        }
        // modify normals
        if (theTriangulation.HasNormals)
        {
            for (int anInd = 1; anInd <= theTriangulation.NodeCount; ++anInd)
            {
                Direction3D aNormal = theTriangulation.Normal(anInd);
                Matrix3x3 aMat = aGTrsf.VectorialPart();
                aMat.SetDiagonal(1.0, 1.0, 1.0);
                Transform3D aTrsf = new Transform3D();
                aTrsf.SetForm(Rotation);
                aTrsf._matrix = aMat;
                aNormal.Transform(aTrsf);
                theTriangulation.SetNormal(anInd, aNormal);
            }
        }

        return true;
    }

    /// <summary>The edge's 3D polygon transformed; false if it has none.</summary>
    public override bool NewPolygon(in TopoDS_Edge theEdge, out Polygon3D? thePoly)
    {
        thePoly = BRep_Tool.Polygon3D(theEdge, out Placement aLoc);
        if (thePoly is null)
        {
            return false;
        }

        GeneralTransform3D aGTrsf = new GeneralTransform3D();
        aGTrsf.SetVectorialPart(myGTrsf.VectorialPart());
        aGTrsf.SetTranslationPart(myGTrsf.TranslationPart());
        aGTrsf.Multiply(new GeneralTransform3D(aLoc.Transformation()));

        thePoly = thePoly.Copy();
        thePoly.Deflection(thePoly.Deflection() * Math.Abs(myGScale));
        // transform nodes
        NCollection_Array1<Point3D> aNodesArray = thePoly.ChangeNodes();
        for (int anId = aNodesArray.Lower(); anId <= aNodesArray.Upper(); ++anId)
        {
            ref Point3D aP = ref aNodesArray[anId];
            Xyz aXYZ = aP.XYZ();
            aGTrsf.Transforms(ref aXYZ);
            aP.SetXYZ(aXYZ);
        }
        return true;
    }

    /// <summary>
    /// The edge's polygon on the face's triangulation, copied (null if it has none); false if the face has no
    /// triangulation.
    /// </summary>
    public override bool NewPolygonOnTriangulation(in TopoDS_Edge theEdge, in TopoDS_Face theFace, out PolygonOnTriangulation? thePoly)
    {
        thePoly = null;
        Triangulation? aT = BRep_Tool.Triangulation(theFace, out Placement aLoc);
        if (aT is null)
        {
            return false;
        }

        thePoly = BRep_Tool.PolygonOnTriangulation(theEdge, aT, aLoc);
        if (thePoly is not null)
        {
            thePoly = thePoly.Copy();
        }
        return true;
    }
}
