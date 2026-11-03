// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_EnergyOfMVC.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_EnergyOfMVC.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_EnergyOfMVC.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Occt.FairCurve_AnalysisCode;

namespace Occt;

/// <summary>
/// The energy criterion that FairCurve_MinimalVariation minimises: tension, plus jerk and sagging mixed by the
/// physical ratio.
/// </summary>
public class FairCurve_EnergyOfMVC : FairCurve_Energy
{
    private double MyLengthSliding;
    private readonly double OriginalSliding;
    private readonly FairCurve_BattenLaw MyBattenLaw;
    private readonly double MyPhysicalRatio;
    private readonly FairCurve_DistributionOfTension MyTension;
    private readonly FairCurve_DistributionOfSagging MySagging;
    private readonly FairCurve_DistributionOfJerk MyJerk;
    private FairCurve_AnalysisCode MyStatus;

    /// <summary>
    /// The energy of the curve of order <paramref name="BSplOrder"/> over <paramref name="FlatKnots"/> and
    /// <paramref name="Poles"/> (kept, and written by each evaluation). Angles are measured from the Ox axis.
    /// </summary>
    /// <exception cref="DomainErrorException"><paramref name="PhysicalRatio"/> is not in [0, 1].</exception>
    public FairCurve_EnergyOfMVC(int BSplOrder,
                                 NCollection_HArray1<double> FlatKnots,
                                 NCollection_HArray1<Point2D> Poles,
                                 int ContrOrder1,
                                 int ContrOrder2,
                                 FairCurve_BattenLaw Law,
                                 double PhysicalRatio,
                                 double LengthSliding,
                                 bool FreeSliding = true,
                                 double Angle1 = 0,
                                 double Angle2 = 0,
                                 double Curvature1 = 0,
                                 double Curvature2 = 0)
        : base(Poles,
               ContrOrder1,
               ContrOrder2,
               FreeSliding,
               Angle1,
               Angle2,
               BSplOrder - 1,
               Curvature1,
               Curvature2)
    {
        MyLengthSliding = LengthSliding;
        OriginalSliding = LengthSliding;
        // by-value members in OCCT: each keeps its own copy of the law (decision 858)
        MyBattenLaw = new FairCurve_BattenLaw(Law);
        MyPhysicalRatio = PhysicalRatio;
        MyTension = new FairCurve_DistributionOfTension(BSplOrder, FlatKnots, Poles, 1, LengthSliding, Law, FreeSliding ? 1 : 0, true);
        MySagging = new FairCurve_DistributionOfSagging(BSplOrder, FlatKnots, Poles, 1, Law, FreeSliding ? 1 : 0);
        MyJerk = new FairCurve_DistributionOfJerk(BSplOrder, FlatKnots, Poles, 1, Law, FreeSliding ? 1 : 0);
        MyStatus = FairCurve_OK;
        Standard_DomainError_Raise_if(PhysicalRatio < 0 || PhysicalRatio > 1,
                                      "FairCurve_EnergyOfMVC: PhysicalRatio error");
    }

    /// <summary>The length sliding (P1P2 + Sliding).</summary>
    public double LengthSliding()
    {
        return MyLengthSliding;
    }

    /// <summary>The status of the latest evaluation.</summary>
    public FairCurve_AnalysisCode Status()
    {
        return MyStatus;
    }

    protected override void ComputePoles(math_Vector X)
    {
        base.ComputePoles(X);
        if (MyWithAuxValue)
        {
            MyLengthSliding = X[X.Upper()];
        }
    }

    /// <summary>Computes the variables <paramref name="X"/> that correspond to the poles (and the sliding).</summary>
    public override bool Variable(math_Vector X)
    {
        bool Ok;
        Ok = base.Variable(X);
        if (MyWithAuxValue)
        {
            X[X.Upper()] = MyLengthSliding;
        }
        return Ok;
    }

    protected override bool Compute(int DerivativeOrder, math_Vector Result)
    {
        math_Vector Debut = new(1, 1, 0.0), Fin = new(1, 1, 1.0);
        math_IntegerVector MyOrder = new(1, 1, 24);
        bool Ok = false;

        // Blindage contre les longueur de glissement trop exotique
        MyStatus = FairCurve_OK;
        if (MyLengthSliding > 10 * OriginalSliding)
        {
            MyStatus = FairCurve_InfiniteSliding;
            return false;
        }
        if (MyLengthSliding < OriginalSliding / 100)
        {
            MyLengthSliding = OriginalSliding / 100;
        }

        // Mise a jour des objets sous-fonction
        MyTension.SetDerivativeOrder(DerivativeOrder);
        MyTension.SetLengthSliding(MyLengthSliding);
        MySagging.SetDerivativeOrder(DerivativeOrder);
        MyJerk.SetDerivativeOrder(DerivativeOrder);
        // OCCT: updates this copy of the law only, which nothing reads; the distributions keep the initial sliding
        MyBattenLaw.SetSliding(MyLengthSliding);

        //  Integrations

        // on decoupe afin d'avoir au moins 2 points d'integration par poles
        // 24 points de Gauss => 12 poles maximum.

        int NbInterv = (MyPoles.Length() - 1) / 12 + 1, ii;
        double Delta = 1.0 / NbInterv;
        Result.Init(0);

        if (MyPhysicalRatio <= 1.0e-12)
        {

            // Cas purement non physique --------------------------

            for (ii = 1; ii <= NbInterv; ii++)
            {
                Debut[1] = (ii - 1) * Delta;
                Fin[1] = ii * Delta;

                GaussSetIntegration SumTension = new(MyTension, Debut, Fin, MyOrder);
                Ok = SumTension.IsDone;
                if (!Ok)
                {
                    return Ok;
                }

                GaussSetIntegration SumJerk = new(MyJerk, Debut, Fin, MyOrder);
                Ok = SumJerk.IsDone;
                if (!Ok)
                {
                    return Ok;
                }

                Result += SumJerk.Value() + SumTension.Value(); // Cas purement non physique
            }
        }
        else
        {
            // Cas mixte  --------------------------
            for (ii = 1; ii <= NbInterv; ii++)
            {
                Debut[1] = (ii - 1) * Delta;
                Fin[1] = ii * Delta;

                GaussSetIntegration SumTension = new(MyTension, Debut, Fin, MyOrder);
                Ok = SumTension.IsDone;
                if (!Ok)
                {
                    return Ok;
                }

                GaussSetIntegration SumSagging = new(MySagging, Debut, Fin, MyOrder);
                Ok = SumSagging.IsDone;
                if (!Ok)
                {
                    return Ok;
                }

                GaussSetIntegration SumJerk = new(MyJerk, Debut, Fin, MyOrder);
                Ok = SumJerk.IsDone;
                if (!Ok)
                {
                    return Ok;
                }

                Result += SumJerk.Value() * (1 - MyPhysicalRatio) + SumSagging.Value() * MyPhysicalRatio
                          + SumTension.Value();
            }
        }

        return Ok;
    }
}
