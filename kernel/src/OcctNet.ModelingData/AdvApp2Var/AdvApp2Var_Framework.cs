// Port of: src/ModelingData/TKGeomBase/AdvApp2Var/AdvApp2Var_Framework.hxx
// Port of: src/ModelingData/TKGeomBase/AdvApp2Var/AdvApp2Var_Framework.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_IsoType;

namespace Occt;

/// <summary>
/// The constraints of <see cref="AdvApp2Var_ApproxAFunc2Var"/>: the nodes of the grid, and its iso-curves in strips
/// (OCCT's "strips" are sequences of <see cref="AdvApp2Var_Iso"/>; AdvApp2Var has no Strip class).
/// </summary>
/// <remarks>
/// A working object of the approximation, with OCCT's public mutators (decision 1903). The constructor copies the
/// sequences and their strips, as OCCT's does; the nodes and isos are shared.
/// </remarks>
public class AdvApp2Var_Framework
{
    private readonly NCollection_Sequence<AdvApp2Var_Node> myNodeConstraints;
    private readonly NCollection_Sequence<NCollection_Sequence<AdvApp2Var_Iso>> myUConstraints;
    private readonly NCollection_Sequence<NCollection_Sequence<AdvApp2Var_Iso>> myVConstraints;

    private static int findStripByBounds(NCollection_Sequence<NCollection_Sequence<AdvApp2Var_Iso>> theConstraints,
                                         double theFirst,
                                         double theLast)
    {
        int aStripIndex = 1;
        while (aStripIndex < theConstraints.Length()
               && (theConstraints.Value(aStripIndex).First().T0() != theFirst
                   || theConstraints.Value(aStripIndex).First().T1() != theLast))
        {
            ++aStripIndex;
        }
        return aStripIndex;
    }

    private static int findIsoByConstant(NCollection_Sequence<AdvApp2Var_Iso> theStrip, int theNbIso, double theConst)
    {
        int anIsoIndex = 1;
        while (anIsoIndex <= theNbIso && theStrip.Value(anIsoIndex).Constante() != theConst)
        {
            ++anIsoIndex;
        }
        return anIsoIndex;
    }

    private static NCollection_Sequence<NCollection_Sequence<AdvApp2Var_Iso>> copyStrips(
        NCollection_Sequence<NCollection_Sequence<AdvApp2Var_Iso>> theStrips)
    {
        NCollection_Sequence<NCollection_Sequence<AdvApp2Var_Iso>> aCopy = new();
        foreach (NCollection_Sequence<AdvApp2Var_Iso> aStrip in theStrips)
        {
            aCopy.Append(new NCollection_Sequence<AdvApp2Var_Iso>(aStrip));
        }
        return aCopy;
    }

    public AdvApp2Var_Framework()
    {
        myNodeConstraints = new NCollection_Sequence<AdvApp2Var_Node>();
        myUConstraints = new NCollection_Sequence<NCollection_Sequence<AdvApp2Var_Iso>>();
        myVConstraints = new NCollection_Sequence<NCollection_Sequence<AdvApp2Var_Iso>>();
    }

    public AdvApp2Var_Framework(NCollection_Sequence<AdvApp2Var_Node> Frame,
                                NCollection_Sequence<NCollection_Sequence<AdvApp2Var_Iso>> UFrontier,
                                NCollection_Sequence<NCollection_Sequence<AdvApp2Var_Iso>> VFrontier)
    {
        myNodeConstraints = new NCollection_Sequence<AdvApp2Var_Node>(Frame);
        myUConstraints = copyStrips(UFrontier);
        myVConstraints = copyStrips(VFrontier);
    }

    /// <summary>
    /// The first iso not approximated, with its indices in <paramref name="IndexIso"/> and
    /// <paramref name="IndexStrip"/>; null (indices unchanged) if every iso is.
    /// </summary>
    public AdvApp2Var_Iso? FirstNotApprox(ref int IndexIso, ref int IndexStrip)
    {
        for (int anUVIter = 0; anUVIter < 2; ++anUVIter)
        {
            NCollection_Sequence<NCollection_Sequence<AdvApp2Var_Iso>> aSeq =
                anUVIter == 0 ? myUConstraints : myVConstraints;
            int i = 1;
            foreach (NCollection_Sequence<AdvApp2Var_Iso> S in aSeq)
            {
                int j = 1;
                foreach (AdvApp2Var_Iso anIso in S)
                {
                    if (!anIso.IsApproximated())
                    {
                        IndexIso = j;
                        IndexStrip = i;
                        return anIso;
                    }
                    j++;
                }
                i++;
            }
        }
        return null;
    }

    public int FirstNode(GeomAbs_IsoType Type, int IndexIso, int IndexStrip)
    {
        int aNbIsoInV = myUConstraints.Length() + 1;
        if (Type == GeomAbs_IsoU)
        {
            return aNbIsoInV * (IndexStrip - 1) + IndexIso;
        }
        return aNbIsoInV * (IndexIso - 1) + IndexStrip;
    }

    public int LastNode(GeomAbs_IsoType Type, int IndexIso, int IndexStrip)
    {
        int aNbIsoInV = myUConstraints.Length() + 1;
        if (Type == GeomAbs_IsoU)
        {
            return aNbIsoInV * IndexStrip + IndexIso;
        }
        return aNbIsoInV * (IndexIso - 1) + IndexStrip + 1;
    }

    public void ChangeIso(int IndexIso, int IndexStrip, AdvApp2Var_Iso theIso)
    {
        NCollection_Sequence<AdvApp2Var_Iso> aStrip = theIso.Type() == GeomAbs_IsoV
                                                          ? myUConstraints.ChangeValue(IndexStrip)
                                                          : myVConstraints.ChangeValue(IndexStrip);
        aStrip.SetValue(IndexIso, theIso);
    }

    public AdvApp2Var_Node Node(int IndexNode) => myNodeConstraints.Value(IndexNode);

    /// <summary>The node at (<paramref name="U"/>, <paramref name="V"/>), or the last node if there is none.</summary>
    public AdvApp2Var_Node Node(double U, double V)
    {
        foreach (AdvApp2Var_Node aNode in myNodeConstraints)
        {
            if (aNode.Coord().X() == U && aNode.Coord().Y() == V)
            {
                return aNode;
            }
        }
        return myNodeConstraints.Last();
    }

    public AdvApp2Var_Iso IsoU(double U, double V0, double V1)
    {
        int aStripIndex = findStripByBounds(myVConstraints, V0, V1);
        NCollection_Sequence<AdvApp2Var_Iso> aStrip = myVConstraints.Value(aStripIndex);
        int aIsoIndex = findIsoByConstant(aStrip, myUConstraints.Length(), U);
        return aStrip.Value(aIsoIndex);
    }

    public AdvApp2Var_Iso IsoV(double U0, double U1, double V)
    {
        int aStripIndex = findStripByBounds(myUConstraints, U0, U1);
        NCollection_Sequence<AdvApp2Var_Iso> aStrip = myUConstraints.Value(aStripIndex);
        int aIsoIndex = findIsoByConstant(aStrip, myVConstraints.Length(), V);
        return aStrip.Value(aIsoIndex);
    }

    public void UpdateInU(double CuttingValue)
    {
        int aUStripIndex = 1;
        foreach (NCollection_Sequence<AdvApp2Var_Iso> anUConst in myUConstraints)
        {
            if (anUConst.First().U0() <= CuttingValue && anUConst.First().U1() >= CuttingValue)
            {
                break;
            }
            ++aUStripIndex;
        }

        {
            NCollection_Sequence<AdvApp2Var_Iso> aStrip = myUConstraints.Value(aUStripIndex);
            double aUFirst = aStrip.First().U0();
            double aULast = aStrip.First().U1();

            // Modify the V isos of the U strip at aUStripIndex.
            foreach (AdvApp2Var_Iso anIso in aStrip)
            {
                anIso.ChangeDomain(aUFirst, CuttingValue);
                anIso.ResetApprox();
            }

            // Insert a new U strip after aUStripIndex.
            NCollection_Sequence<AdvApp2Var_Iso> aNewStrip = new();
            foreach (AdvApp2Var_Iso anIso in aStrip)
            {
                AdvApp2Var_Iso aNewIso = new(anIso.Type(),
                                             anIso.Constante(),
                                             CuttingValue,
                                             aULast,
                                             anIso.V0(),
                                             anIso.V1(),
                                             0,
                                             anIso.UOrder(),
                                             anIso.VOrder());
                aNewIso.ResetApprox();
                aNewStrip.Append(aNewIso);
            }
            myUConstraints.InsertAfter(aUStripIndex, aNewStrip);
        }

        //  Insert a new Iso U=U* in each V strip after aUStripIndex
        //  and restrict the domains of the adjacent Isos
        for (int aVStripIndex = 1; aVStripIndex <= myVConstraints.Length(); aVStripIndex++)
        {
            NCollection_Sequence<AdvApp2Var_Iso> aStrip = myVConstraints.ChangeValue(aVStripIndex);
            AdvApp2Var_Iso anIso = aStrip.Value(aUStripIndex);
            anIso.ChangeDomain(anIso.U0(), CuttingValue, anIso.V0(), anIso.V1());

            AdvApp2Var_Iso aNewIso = new(anIso.Type(),
                                         CuttingValue,
                                         anIso.U0(),
                                         CuttingValue,
                                         anIso.V0(),
                                         anIso.V1(),
                                         0,
                                         anIso.UOrder(),
                                         anIso.VOrder());
            aNewIso.ResetApprox();
            aStrip.InsertAfter(aUStripIndex, aNewIso);

            anIso = aStrip.Value(aUStripIndex + 2);
            anIso.ChangeDomain(CuttingValue, anIso.U1(), anIso.V0(), anIso.V1());
        }

        //  Insert the new nodes (U*,Vj)
        AdvApp2Var_Node aNext;
        AdvApp2Var_Node aPrev = myNodeConstraints.First();
        for (int aNodeIndex = 1; aNodeIndex < myNodeConstraints.Length(); aNodeIndex++)
        {
            aNext = myNodeConstraints.Value(aNodeIndex + 1);
            if (aPrev.Coord().X() < CuttingValue && aNext.Coord().X() > CuttingValue
                && aPrev.Coord().Y() == aNext.Coord().Y())
            {
                gp_XY aNewUV = new(CuttingValue, aPrev.Coord().Y());
                AdvApp2Var_Node aNewNode = new(aNewUV, aPrev.UOrder(), aPrev.VOrder());
                myNodeConstraints.InsertAfter(aNodeIndex, aNewNode);
            }
            aPrev = aNext;
        }
    }

    public void UpdateInV(double CuttingValue)
    {
        int aVStripIndex = 1;
        while (myVConstraints.Value(aVStripIndex).First().V0() > CuttingValue
               || myVConstraints.Value(aVStripIndex).First().V1() < CuttingValue)
        {
            aVStripIndex++;
        }

        {
            NCollection_Sequence<AdvApp2Var_Iso> aStrip = myVConstraints.ChangeValue(aVStripIndex);
            double aVFirst = aStrip.First().V0();
            double aVLast = aStrip.First().V1();

            // Modify the U isos of the V strip at aVStripIndex.
            foreach (AdvApp2Var_Iso anIso in aStrip)
            {
                anIso.ChangeDomain(aVFirst, CuttingValue);
                anIso.ResetApprox();
            }

            // Insert a new V strip after aVStripIndex.
            NCollection_Sequence<AdvApp2Var_Iso> aNewStrip = new();
            foreach (AdvApp2Var_Iso anIso in aStrip)
            {
                AdvApp2Var_Iso aNewIso = new(anIso.Type(),
                                             anIso.Constante(),
                                             anIso.U0(),
                                             anIso.U1(),
                                             CuttingValue,
                                             aVLast,
                                             0,
                                             anIso.UOrder(),
                                             anIso.VOrder());
                aNewIso.ResetApprox();
                aNewStrip.Append(aNewIso);
            }
            myVConstraints.InsertAfter(aVStripIndex, aNewStrip);
        }

        // Insert a new Iso V=V* in each U strip after aVStripIndex
        // and restrict the domains of the adjacent Isos
        foreach (NCollection_Sequence<AdvApp2Var_Iso> aStrip in myUConstraints)
        {
            AdvApp2Var_Iso anIso = aStrip.Value(aVStripIndex);
            anIso.ChangeDomain(anIso.U0(), anIso.U1(), anIso.V0(), CuttingValue);

            AdvApp2Var_Iso aNewIso = new(anIso.Type(),
                                         CuttingValue,
                                         anIso.U0(),
                                         anIso.U1(),
                                         anIso.V0(),
                                         CuttingValue,
                                         0,
                                         anIso.UOrder(),
                                         anIso.VOrder());
            aNewIso.ResetApprox();
            aStrip.InsertAfter(aVStripIndex, aNewIso);

            anIso = aStrip.Value(aVStripIndex + 2);
            anIso.ChangeDomain(anIso.U0(), anIso.U1(), CuttingValue, anIso.V1());
        }

        //  Insert the new nodes (Ui,V*)
        int aNodeStartIndex = 1;
        while (aNodeStartIndex <= myNodeConstraints.Length()
               && myNodeConstraints.Value(aNodeStartIndex).Coord().Y() < CuttingValue)
        {
            aNodeStartIndex += myUConstraints.Length() + 1;
        }
        for (int anIsoIndex = 1; anIsoIndex <= myUConstraints.Length() + 1; anIsoIndex++)
        {
            AdvApp2Var_Node aJNode = myNodeConstraints.Value(anIsoIndex);
            gp_XY NewUV = new(aJNode.Coord().X(), CuttingValue);
            AdvApp2Var_Node aNewNode = new(NewUV, aJNode.UOrder(), aJNode.VOrder());
            myNodeConstraints.InsertAfter(aNodeStartIndex + anIsoIndex - 2, aNewNode);
        }
    }

    /// <summary>The approximation of the U iso <paramref name="IndexIso"/> of strip <paramref name="IndexStrip"/>.</summary>
    public NCollection_HArray1<double>? UEquation(int IndexIso, int IndexStrip) =>
        myVConstraints.Value(IndexStrip).Value(IndexIso).Polynom();

    /// <summary>The approximation of the V iso <paramref name="IndexIso"/> of strip <paramref name="IndexStrip"/>.</summary>
    public NCollection_HArray1<double>? VEquation(int IndexIso, int IndexStrip) =>
        myUConstraints.Value(IndexStrip).Value(IndexIso).Polynom();
}
