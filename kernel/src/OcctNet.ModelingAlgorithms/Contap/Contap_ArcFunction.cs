// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_ArcFunction.hxx
// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_ArcFunction.lxx
// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_ArcFunction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Occt.Contap_TFunction;

namespace Occt;

/// <summary>
/// The contour function along an arc of restriction of a surface: the normal's component along the view direction
/// (or towards the eye), scaled by the mean normal length, F(t) = N(S(A(t))) . D / mean, with its derivative; the
/// function of the boundary search (<see cref="Contap_TheSearch"/>). Each state saved by <see cref="GetStateNumber"/>
/// keeps the last evaluated point.
/// </summary>
/// <remarks>
/// A working object. It implements <see cref="IIntStart_ArcFunction"/>, what IntStart_SearchOnBoundaries reads; its
/// quadric is OCCT's default one, never set. With <see cref="Contap_DraftPrs"/> OCCT computes nothing and leaves the
/// value and derivative unset; the port returns 0 (decision 1076).
/// </remarks>
public class Contap_ArcFunction : FunctionWithDerivative, IIntStart_ArcFunction
{
    private Adaptor2d_Curve2d? myArc;
    private Adaptor3d_Surface? mySurf;
    private double myMean;
    private Contap_TFunction myType;
    private Direction3D myDir;
    private double myCosAng;
    private Point3D myEye;
    private Point3D solpt;
    private readonly NCollection_Sequence<Point3D> seqpt = new();
    private IntSurf_Quadric myQuad;

    public Contap_ArcFunction()
    {
        myMean = 1.0;
        myType = Contap_ContourStd;
        myDir = new Direction3D(Direction3D.D.Z);
        myCosAng = 0.0;
        myEye = new Point3D();
        solpt = new Point3D();
        myQuad = new IntSurf_Quadric();
    }

    /// <summary>Sets the surface, and the mean length of its normal at its sample points.</summary>
    public void Set(Adaptor3d_Surface S)
    {
        mySurf = S;
        int i;
        int nbs = Contap_HContTool.NbSamplePoints(S);
        double U, V;
        //  gp_Vec d1u,d1v;
        Vector3D norm;
        if (nbs > 0)
        {
            myMean = 0.0;
            for (i = 1; i <= nbs; i++)
            {
                Contap_HContTool.SamplePoint(S, i, out U, out V);
                //      Adaptor3d_HSurfaceTool::D1(S,U,V,solpt,d1u,d1v);
                //      myMean = myMean + d1u.Crossed(d1v).Magnitude();
                Contap_SurfProps.Normale(S, U, V, out solpt, out norm);
                myMean = myMean + norm.Magnitude();
            }
            myMean = myMean / ((double)nbs);
        }
    }

    /// <summary>A silhouette for the view direction <paramref name="Direction"/>.</summary>
    public void Set(in Direction3D Direction)
    {
        myType = Contap_ContourStd;
        myDir = Direction;
    }

    /// <summary>A draft contour of angle <paramref name="Angle"/> for the direction <paramref name="Direction"/>.</summary>
    public void Set(in Direction3D Direction, double Angle)
    {
        myType = Contap_DraftStd;
        myDir = Direction;
        myCosAng = Math.Cos(M_PI / 2.0 + Angle);
    }

    /// <summary>A silhouette for the eye <paramref name="Eye"/>.</summary>
    public void Set(in Point3D Eye)
    {
        myType = Contap_ContourPrs;
        myEye = Eye;
    }

    /// <summary>A draft contour of angle <paramref name="Angle"/> for the eye <paramref name="Eye"/>.</summary>
    public void Set(in Point3D Eye, double Angle)
    {
        myType = Contap_DraftPrs;
        myEye = Eye;
        myCosAng = Math.Cos(M_PI / 2.0 + Angle);
    }

    /// <summary>Sets the arc (and forgets the saved points).</summary>
    public void Set(Adaptor2d_Curve2d A)
    {
        myArc = A;
        seqpt.Clear();
    }

    public override bool Value(double U, out double F)
    {
        F = 0.0; // OCCT leaves F unset for Contap_DraftPrs (decision 1076)
        // gp_Vec d1u,d1v;
        Point2D pt2d = Contap_HCurve2dTool.Value(myArc!, U);
        //  Adaptor3d_HSurfaceTool::D1(mySurf,pt2d.X(),pt2d.Y(),solpt,d1u,d1v);
        //  gp_Vec norm(d1u.Crossed(d1v));
        Contap_SurfProps.Normale(mySurf!, pt2d.X(), pt2d.Y(), out solpt, out Vector3D norm);

        switch (myType)
        {
            case Contap_ContourStd:
                F = (norm.Dot(new Vector3D(myDir))) / myMean;
                break;
            case Contap_ContourPrs:
                F = (norm.Dot(new Vector3D(myEye, solpt))) / myMean;
                break;
            case Contap_DraftStd:
                F = (norm.Dot(new Vector3D(myDir)) - myCosAng * norm.Magnitude()) / myMean;
                break;
            case Contap_DraftPrs:
            default:
                break;
        }
        return true;
    }

    public override bool Derivative(double U, out double D)
    {
        double dfu = 0.0, dfv = 0.0;
        //  gp_Vec d1u,d1v,d2u,d2v,d2uv;
        Contap_HCurve2dTool.D1(myArc!, U, out Point2D pt2d, out Vector2D d2d);
        //  Adaptor3d_HSurfaceTool::D2(mySurf,pt2d.X(),pt2d.Y(),solpt,d1u,d1v,d2u,d2v,d2uv);
        Contap_SurfProps.NormAndDn(mySurf!, pt2d.X(), pt2d.Y(), out solpt, out Vector3D norm, out Vector3D dnu,
                                   out Vector3D dnv);

        switch (myType)
        {
            case Contap_ContourStd:
                //      dfu = ((d2u.Crossed(d1v) + d1u.Crossed(d2uv)).Dot(myDir))/myMean;
                //      dfv = ((d2uv.Crossed(d1v) + d1u.Crossed(d2v)).Dot(myDir))/myMean;
                dfu = (dnu.Dot(new Vector3D(myDir))) / myMean;
                dfv = (dnv.Dot(new Vector3D(myDir))) / myMean;
                break;
            case Contap_ContourPrs:
            {
                Vector3D Ep = new(myEye, solpt);
                dfu = (dnu.Dot(Ep)) / myMean;
                dfv = (dnv.Dot(Ep)) / myMean;
            }
            break;
            case Contap_DraftStd:
                norm.Normalize();
                dfu = (dnu.Dot(new Vector3D(myDir)) - myCosAng * dnu.Dot(norm)) / myMean;
                dfv = (dnv.Dot(new Vector3D(myDir)) - myCosAng * dnv.Dot(norm)) / myMean;
                break;
            case Contap_DraftPrs:
            default:
                break;
        }
        D = d2d.X() * dfu + d2d.Y() * dfv;
        return true;
    }

    public override bool Values(double U, out double F, out double D)
    {
        F = 0.0; // OCCT leaves F unset for Contap_DraftPrs (decision 1076)
        double dfu = 0.0, dfv = 0.0;
        // gp_Vec d1u,d1v,d2u,d2v,d2uv;
        Contap_HCurve2dTool.D1(myArc!, U, out Point2D pt2d, out Vector2D d2d);
        //  Adaptor3d_HSurfaceTool::D2(mySurf,pt2d.X(),pt2d.Y(),solpt,d1u,d1v,d2u,d2v,d2uv);
        //  gp_Vec norm(d1u.Crossed(d1v));
        Contap_SurfProps.NormAndDn(mySurf!, pt2d.X(), pt2d.Y(), out solpt, out Vector3D norm, out Vector3D dnu,
                                   out Vector3D dnv);

        switch (myType)
        {
            case Contap_ContourStd:
                F = (norm.Dot(new Vector3D(myDir))) / myMean;
                dfu = (dnu.Dot(new Vector3D(myDir))) / myMean;
                dfv = (dnv.Dot(new Vector3D(myDir))) / myMean;
                break;
            case Contap_ContourPrs:
            {
                Vector3D Ep = new(myEye, solpt);
                F = (norm.Dot(Ep)) / myMean;
                dfu = (dnu.Dot(Ep)) / myMean;
                dfv = (dnv.Dot(Ep)) / myMean;
            }
            break;
            case Contap_DraftStd:
                F = (norm.Dot(new Vector3D(myDir)) - myCosAng * norm.Magnitude()) / myMean;
                norm.Normalize();
                dfu = (dnu.Dot(new Vector3D(myDir)) - myCosAng * dnu.Dot(norm)) / myMean;
                dfv = (dnv.Dot(new Vector3D(myDir)) - myCosAng * dnv.Dot(norm)) / myMean;
                break;
            case Contap_DraftPrs:
            default:
                break;
        }

        D = d2d.X() * dfu + d2d.Y() * dfv;
        return true;
    }

    /// <summary>Saves the last evaluated point; returns its rank for <see cref="Valpoint"/>.</summary>
    public override int GetStateNumber()
    {
        seqpt.Append(solpt);
        return seqpt.Length();
    }

    /// <summary>The number of samples on the arc: the largest of the surface's and the arc's.</summary>
    public int NbSamples()
    {
        return Max(Max(Contap_HContTool.NbSamplesU(mySurf!, 0.0, 0.0), Contap_HContTool.NbSamplesV(mySurf!, 0.0, 0.0)),
                   Contap_HContTool.NbSamplesOnArc(myArc!));
    }

    /// <summary>The point saved at state <paramref name="Index"/>.</summary>
    public Point3D Valpoint(int Index) => seqpt.Value(Index);

    /// <summary>OCCT's default quadric (the contour function has none).</summary>
    public IntSurf_Quadric Quadric() => (myQuad);

    /// <summary>The surface.</summary>
    public Adaptor3d_Surface Surface() => (mySurf!);

    /// <summary>The point computed by the last call to <see cref="Value"/> (or the derivatives).</summary>
    public Point3D LastComputedPoint() => solpt;
}
