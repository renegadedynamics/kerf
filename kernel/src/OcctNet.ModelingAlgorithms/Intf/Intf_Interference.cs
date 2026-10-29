// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf_Interference.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf_Interference.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf_Interference.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The result of an interference computation between 2D or 3D polygons or polyhedra: sequences of intersection
/// points, polylines of intersection and zones of tangency.
/// </summary>
/// <remarks>
/// An abstract class (OCCT's constructor is protected). <see cref="LineValue"/> and <see cref="ZoneValue"/> return
/// the stored lines and zones, as OCCT's const references; their mutators are internal. OCCT's <c>Dump</c> (to
/// std::cout) is left out.
/// </remarks>
public abstract class Intf_Interference
{
    protected readonly NCollection_Sequence<Intf_SectionPoint> mySPoins = new();
    protected readonly NCollection_Sequence<Intf_SectionLine> mySLines = new();
    protected readonly NCollection_Sequence<Intf_TangentZone> myTZones = new();
    protected bool SelfIntf;
    protected double Tolerance;

    /// <summary>Empty constructor.</summary>
    protected Intf_Interference(bool Self)
    {
        SelfIntf = Self;
        Tolerance = 0.0;
    }

    /// <summary>The number of intersection points in the interference.</summary>
    public int NbSectionPoints() => mySPoins.Length();

    /// <summary>The intersection point of address <paramref name="Index"/> in the interference.</summary>
    public Intf_SectionPoint PntValue(int Index) => mySPoins[Index];

    /// <summary>The number of polylines of intersection in the interference.</summary>
    public int NbSectionLines() => mySLines.Length();

    /// <summary>The polyline of intersection of address <paramref name="Index"/> in the interference (the stored one).</summary>
    public Intf_SectionLine LineValue(int Index) => mySLines[Index];

    /// <summary>The number of zones of tangency in the interference.</summary>
    public int NbTangentZones() => myTZones.Length();

    /// <summary>The zone of tangency of address <paramref name="Index"/> in the interference (the stored one).</summary>
    public Intf_TangentZone ZoneValue(int Index) => myTZones[Index];

    /// <summary>The tolerance used for the calculation.</summary>
    public double GetTolerance() => Tolerance;

    /// <summary>True if the polylines of intersection or the zones of tangency contain the point <paramref name="LePnt"/>.</summary>
    public bool Contains(in Intf_SectionPoint LePnt)
    {
        //-- LePnt.Dump(0);
        for (int l = 1; l <= mySLines.Length(); l++)
        {
            if (mySLines[l].Contains(LePnt))
            {
                return true;
            }
        }

        for (int t = 1; t <= myTZones.Length(); t++)
        {
            //-- myTZones(t).Dump(2);
            if (myTZones[t].Contains(LePnt))
            {
                return true;
            }
        }

        return false;
    }

    /// <summary>
    /// Merges a new zone of tangency into a current zone that shares points with it; returns true when done.
    /// </summary>
    public bool Insert(Intf_TangentZone LaZone)
    {
        if (myTZones.Length() <= 0)
        {
            return false;
        }

        int lzin = 0;      // Index in the list of the zone of interest.
        int lunp = 0;      // Index of the 1st stop point in this zone.
        int lotp = 0;      // Index of the 2nd stop point in this zone.
        int lunl = 0;      // Index of the 1st point of the new zone.
        int lotl = 0;      // Index of the 2nd point of the new zone.
        bool same = false; // Search direction of the stop of the new zone.
        bool Inserted = true; // Has the insertion succeeded ?
        int npcz = -1;     // Number of points in the current zone
        int nplz = LaZone.NumberOfPoints(); // in the new zone

        // Loop on TangentZone :
        for (int Iz = 1; Iz <= myTZones.Length(); Iz++)
        {
            // Loop on edges of the TangentZone :
            npcz = myTZones[Iz].NumberOfPoints();
            int Ipz0, Ipz1, Ipz2;
            for (Ipz1 = 1; Ipz1 <= npcz; Ipz1++)
            {
                Ipz0 = Ipz1 - 1;
                if (Ipz0 <= 0)
                {
                    Ipz0 = npcz;
                }

                Ipz2 = (Ipz1 % npcz) + 1;

                // Loop on edges of the new TangentZone and search of the
                // corresponding point or edge:
                int Ilz1, Ilz2;
                for (Ilz1 = 1; Ilz1 <= nplz; Ilz1++)
                {
                    Ilz2 = (Ilz1 % nplz) + 1;

                    if ((myTZones[Iz].GetPoint(Ipz1)).IsEqual(LaZone.GetPoint(Ilz1)))
                    {
                        if ((myTZones[Iz].GetPoint(Ipz0)).IsEqual(LaZone.GetPoint(Ilz2)))
                        {
                            lzin = Iz;
                            lunp = Ipz0;
                            lotp = Ipz1;
                            lunl = Ilz1;
                            lotl = Ilz2;
                            same = false;
                            break;
                        }
                        else if ((myTZones[Iz].GetPoint(Ipz2)).IsEqual(LaZone.GetPoint(Ilz2)))
                        {
                            lzin = Iz;
                            lunp = Ipz1;
                            lotp = Ipz2;
                            lunl = Ilz1;
                            lotl = Ilz2;
                            same = true;
                            break;
                        }
                        else
                        {
                            lzin = Iz;
                            lunp = Ipz1;
                            lunl = Ilz1;
                        }
                    }
                }

                if (lotp != 0)
                {
                    break;
                }
            }

            if (lotp != 0)
            {
                break;
            }
        }

        int Ilc;
        if (lotp != 0)
        {
            for (Ilc = lotl + 1; (((Ilc - 1) % nplz) + 1) != lunl; Ilc++)
            {
                myTZones[lzin].InsertBefore(lotp, LaZone.GetPoint(((Ilc - 1) % nplz) + 1));
                if (!same)
                {
                    lotp++;
                }
            }
        }
        else if (lunp > 0)
        {
            bool loop = false;
            for (Ilc = lunl; ; Ilc++)
            {
                myTZones[lzin].InsertBefore(lunp, LaZone.GetPoint((((Ilc - 1) % nplz) + 1)));
                lunp++;
                if (loop && (((Ilc - 1) % nplz) + 1) == lunl)
                {
                    break;
                }

                loop = true;
            }
        }
        else
        {
            Inserted = false;
        }

        if (Inserted)
        {
            // C++ copies the zone, then removes it from the sequence: here the removed object itself is reinserted
            Intf_TangentZone theNew = myTZones[lzin];
            myTZones.Remove(lzin);
            if (!Insert(theNew))
            {
                myTZones.Append(theNew);
            }
        }

        return Inserted;
    }

    /// <summary>Inserts a new segment of intersection in the current polylines of intersection of the interference.</summary>
    public void Insert(in Intf_SectionPoint pdeb, in Intf_SectionPoint pfin)
    {
        bool Inserted = false;
        int TheLS = 0;
        bool Begin = false;
        Intf_SectionPoint TheBout = pfin;
        int ils, nd, nf;

        for (ils = 1; ils <= mySLines.Length(); ils++)
        {
            Intf_SectionLine SL = mySLines[ils];
            nd = SL.IsEnd(pdeb);
            nf = SL.IsEnd(pfin);
            if (nd == 1)
            {
                if (nf > 1)
                {
                    SL.Close();
                }

                Inserted = true;
                TheLS = ils;
                Begin = true;
                break;
            }
            else if (nd > 1)
            {
                if (nf == 1)
                {
                    SL.Close();
                }

                Inserted = true;
                TheLS = ils;
                Begin = false;
                break;
            }
            else if (nf == 1)
            {
                Inserted = true;
                TheLS = ils;
                Begin = true;
                TheBout = pdeb;
                break;
            }
            else if (nf > 1)
            {
                Inserted = true;
                TheLS = ils;
                Begin = false;
                TheBout = pdeb;
                break;
            }
        }

        if (!Inserted)
        {
            Intf_SectionLine LaLS = new();
            LaLS.Append(pdeb);
            LaLS.Append(pfin);
            mySLines.Append(LaLS);
        }
        else
        {
            nd = 0;
            for (ils = 1; ils <= mySLines.Length(); ils++)
            {
                if (ils != TheLS)
                {
                    nd = mySLines[ils].IsEnd(TheBout);
                    if (nd == 1)
                    {
                        if (Begin)
                        {
                            mySLines[TheLS].Reverse();
                        }

                        mySLines[ils].Prepend(mySLines[TheLS]);
                        break;
                    }
                    else if (nd > 1)
                    {
                        if (!Begin)
                        {
                            mySLines[TheLS].Reverse();
                        }

                        mySLines[ils].Append(mySLines[TheLS]);
                        break;
                    }
                }
            }

            if (nd > 0)
            {
                mySLines.Remove(TheLS);
            }
            else
            {
                if (Begin)
                {
                    mySLines[TheLS].Prepend(TheBout);
                }
                else
                {
                    mySLines[TheLS].Append(TheBout);
                }
            }
        }
    }

    /// <summary>Only one argument for the intersection: clears the results.</summary>
    protected void SelfInterference(bool Self)
    {
        SelfIntf = Self;
        mySPoins.Clear();
        mySLines.Clear();
        myTZones.Clear();
    }

    public override string ToString()
    {
        System.Text.StringBuilder aBuilder = new(GetType().Name);
        aBuilder.Append("(points: ").Append(mySPoins.Length());
        aBuilder.Append(", lines: ").Append(mySLines.Length());
        aBuilder.Append(", zones: ").Append(myTZones.Length()).Append(')');
        return aBuilder.ToString();
    }
}
