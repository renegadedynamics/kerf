// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_ArcFunction.hxx
// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_ArcFunction.lxx
// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_ArcFunction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

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
/// value and derivative unset; the port returns 0 (decision 10102).
/// </remarks>
public class Contap_ArcFunction : math_FunctionWithDerivative, IIntStart_ArcFunction
{
    private Adaptor2d_Curve2d? myArc;
    private Adaptor3d_Surface? mySurf;
    private double myMean;
    private Contap_TFunction myType;
    private gp_Dir myDir;
    private double myCosAng;
    private gp_Pnt myEye;
    private gp_Pnt solpt;
    private readonly NCollection_Sequence<gp_Pnt> seqpt = new();
    private IntSurf_Quadric myQuad;

    public Contap_ArcFunction()
    {
        myMean = 1.0;
        myType = Contap_ContourStd;
        myDir = new gp_Dir(gp_Dir.D.Z);
        myCosAng = 0.0;
        myEye = new gp_Pnt();
        solpt = new gp_Pnt();
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
        gp_Vec norm;
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
    public void Set(in gp_Dir Direction)
    {
        myType = Contap_ContourStd;
        myDir = Direction;
    }

    /// <summary>A draft contour of angle <paramref name="Angle"/> for the direction <paramref name="Direction"/>.</summary>
    public void Set(in gp_Dir Direction, double Angle)
    {
        myType = Contap_DraftStd;
        myDir = Direction;
        myCosAng = Math.Cos(M_PI / 2.0 + Angle);
    }

    /// <summary>A silhouette for the eye <paramref name="Eye"/>.</summary>
    public void Set(in gp_Pnt Eye)
    {
        myType = Contap_ContourPrs;
        myEye = Eye;
    }

    /// <summary>A draft contour of angle <paramref name="Angle"/> for the eye <paramref name="Eye"/>.</summary>
    public void Set(in gp_Pnt Eye, double Angle)
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
        F = 0.0; // OCCT leaves F unset for Contap_DraftPrs (decision 10102)
        // gp_Vec d1u,d1v;
        gp_Pnt2d pt2d = Contap_HCurve2dTool.Value(myArc!, U);
        //  Adaptor3d_HSurfaceTool::D1(mySurf,pt2d.X(),pt2d.Y(),solpt,d1u,d1v);
        //  gp_Vec norm(d1u.Crossed(d1v));
        Contap_SurfProps.Normale(mySurf!, pt2d.X(), pt2d.Y(), out solpt, out gp_Vec norm);

        switch (myType)
        {
            case Contap_ContourStd:
                F = (norm.Dot(new gp_Vec(myDir))) / myMean;
                break;
            case Contap_ContourPrs:
                F = (norm.Dot(new gp_Vec(myEye, solpt))) / myMean;
                break;
            case Contap_DraftStd:
                F = (norm.Dot(new gp_Vec(myDir)) - myCosAng * norm.Magnitude()) / myMean;
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
        Contap_HCurve2dTool.D1(myArc!, U, out gp_Pnt2d pt2d, out gp_Vec2d d2d);
        //  Adaptor3d_HSurfaceTool::D2(mySurf,pt2d.X(),pt2d.Y(),solpt,d1u,d1v,d2u,d2v,d2uv);
        Contap_SurfProps.NormAndDn(mySurf!, pt2d.X(), pt2d.Y(), out solpt, out gp_Vec norm, out gp_Vec dnu,
                                   out gp_Vec dnv);

        switch (myType)
        {
            case Contap_ContourStd:
                //      dfu = ((d2u.Crossed(d1v) + d1u.Crossed(d2uv)).Dot(myDir))/myMean;
                //      dfv = ((d2uv.Crossed(d1v) + d1u.Crossed(d2v)).Dot(myDir))/myMean;
                dfu = (dnu.Dot(new gp_Vec(myDir))) / myMean;
                dfv = (dnv.Dot(new gp_Vec(myDir))) / myMean;
                break;
            case Contap_ContourPrs:
            {
                gp_Vec Ep = new(myEye, solpt);
                dfu = (dnu.Dot(Ep)) / myMean;
                dfv = (dnv.Dot(Ep)) / myMean;
            }
            break;
            case Contap_DraftStd:
                norm.Normalize();
                dfu = (dnu.Dot(new gp_Vec(myDir)) - myCosAng * dnu.Dot(norm)) / myMean;
                dfv = (dnv.Dot(new gp_Vec(myDir)) - myCosAng * dnv.Dot(norm)) / myMean;
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
        F = 0.0; // OCCT leaves F unset for Contap_DraftPrs (decision 10102)
        double dfu = 0.0, dfv = 0.0;
        // gp_Vec d1u,d1v,d2u,d2v,d2uv;
        Contap_HCurve2dTool.D1(myArc!, U, out gp_Pnt2d pt2d, out gp_Vec2d d2d);
        //  Adaptor3d_HSurfaceTool::D2(mySurf,pt2d.X(),pt2d.Y(),solpt,d1u,d1v,d2u,d2v,d2uv);
        //  gp_Vec norm(d1u.Crossed(d1v));
        Contap_SurfProps.NormAndDn(mySurf!, pt2d.X(), pt2d.Y(), out solpt, out gp_Vec norm, out gp_Vec dnu,
                                   out gp_Vec dnv);

        switch (myType)
        {
            case Contap_ContourStd:
                F = (norm.Dot(new gp_Vec(myDir))) / myMean;
                dfu = (dnu.Dot(new gp_Vec(myDir))) / myMean;
                dfv = (dnv.Dot(new gp_Vec(myDir))) / myMean;
                break;
            case Contap_ContourPrs:
            {
                gp_Vec Ep = new(myEye, solpt);
                F = (norm.Dot(Ep)) / myMean;
                dfu = (dnu.Dot(Ep)) / myMean;
                dfv = (dnv.Dot(Ep)) / myMean;
            }
            break;
            case Contap_DraftStd:
                F = (norm.Dot(new gp_Vec(myDir)) - myCosAng * norm.Magnitude()) / myMean;
                norm.Normalize();
                dfu = (dnu.Dot(new gp_Vec(myDir)) - myCosAng * dnu.Dot(norm)) / myMean;
                dfv = (dnv.Dot(new gp_Vec(myDir)) - myCosAng * dnv.Dot(norm)) / myMean;
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
    public gp_Pnt Valpoint(int Index) => seqpt.Value(Index);

    /// <summary>OCCT's default quadric (the contour function has none).</summary>
    public IntSurf_Quadric Quadric() => (myQuad);

    /// <summary>The surface.</summary>
    public Adaptor3d_Surface Surface() => (mySurf!);

    /// <summary>The point computed by the last call to <see cref="Value"/> (or the derivatives).</summary>
    public gp_Pnt LastComputedPoint() => solpt;
}
