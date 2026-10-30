// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_EnergyOfBatten.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_EnergyOfBatten.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_EnergyOfBatten.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.FairCurve_AnalysisCode;

namespace Occt;

/// <summary>The energy criterion that FairCurve_Batten minimises: the batten's tension and sagging.</summary>
public class FairCurve_EnergyOfBatten : FairCurve_Energy
{
    private double MyLengthSliding;
    private readonly double OriginalSliding;
    private readonly FairCurve_BattenLaw MyBattenLaw;
    private readonly FairCurve_DistributionOfTension MyTension;
    private readonly FairCurve_DistributionOfSagging MySagging;
    private FairCurve_AnalysisCode MyStatus;

    /// <summary>
    /// The energy of the curve of order <paramref name="BSplOrder"/> over <paramref name="FlatKnots"/> and
    /// <paramref name="Poles"/> (kept, and written by each evaluation). Angles are measured from the Ox axis.
    /// </summary>
    public FairCurve_EnergyOfBatten(int BSplOrder,
                                    NCollection_HArray1<double> FlatKnots,
                                    NCollection_HArray1<gp_Pnt2d> Poles,
                                    int ContrOrder1,
                                    int ContrOrder2,
                                    FairCurve_BattenLaw Law,
                                    double LengthSliding,
                                    bool FreeSliding = true,
                                    double Angle1 = 0,
                                    double Angle2 = 0)
        : base(Poles, ContrOrder1, ContrOrder2, FreeSliding, Angle1, Angle2)
    {
        MyLengthSliding = LengthSliding;
        OriginalSliding = LengthSliding;
        // by-value members in OCCT: each keeps its own copy of the law (decision 5891)
        MyBattenLaw = new FairCurve_BattenLaw(Law);
        MyTension = new FairCurve_DistributionOfTension(BSplOrder, FlatKnots, Poles, 1, LengthSliding, Law, FreeSliding ? 1 : 0);
        MySagging = new FairCurve_DistributionOfSagging(BSplOrder, FlatKnots, Poles, 1, Law, FreeSliding ? 1 : 0);
        MyStatus = FairCurve_OK;
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

    protected override void ComputePoles(math_Vector X)
    {
        base.ComputePoles(X);
        if (MyWithAuxValue)
        {
            MyLengthSliding = X[X.Upper()];
        }
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
        // OCCT: updates this copy of the law only, which nothing reads; the distributions keep the initial sliding
        MyBattenLaw.SetSliding(MyLengthSliding);

        //  Integrations

        // on decoupe afin d'avoir au moins 2 points d'integration par poles
        // 24 points de Gauss => 12 poles maximum.

        int NbInterv = (MyPoles.Length() - 1) / 12 + 1, ii;
        double Delta = 1.0 / NbInterv;
        Result.Init(0);

        for (ii = 1; ii <= NbInterv; ii++)
        {
            Debut[1] = (ii - 1) * Delta;
            Fin[1] = ii * Delta;

            math_GaussSetIntegration SumTension = new(MyTension, Debut, Fin, MyOrder);
            Ok = SumTension.IsDone();
            if (!Ok)
            {
                return Ok;
            }

            math_GaussSetIntegration SumSagging = new(MySagging, Debut, Fin, MyOrder);
            Ok = SumSagging.IsDone();

            // OCCT: a failed sagging integration goes on to the next interval, whose result is returned
            if (Ok)
            {
                Result += SumSagging.Value() + SumTension.Value();
            }
        }

        return Ok;
    }
}
