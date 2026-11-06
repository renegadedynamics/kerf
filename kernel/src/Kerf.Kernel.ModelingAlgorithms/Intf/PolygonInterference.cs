// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf_Interference.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf_Interference.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf_Interference.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Intersection.Polygons;

/// <summary>
/// The result of an interference computation between 2D or 3D polygons or polyhedra: sequences of intersection
/// points, polylines of intersection and zones of tangency.
/// </summary>
/// <remarks>
/// An abstract class (OCCT's constructor is protected). <see cref="LineValue"/> and <see cref="ZoneValue"/> return
/// the stored lines and zones, as OCCT's const references; their mutators are internal. OCCT's <c>Dump</c> (to
/// std::cout) is left out.
/// </remarks>
internal abstract class PolygonInterference
{
    protected readonly NCollection_Sequence<SectionPoint> _sPoins = new();
    protected readonly NCollection_Sequence<SectionLine> _sLines = new();
    protected readonly NCollection_Sequence<TangentZone> _tZones = new();
    protected bool _selfIntf;
    protected double _tolerance;

    /// <summary>Empty constructor.</summary>
    protected PolygonInterference(bool self)
    {
        _selfIntf = self;
        _tolerance = 0.0;
    }

    /// <summary>The number of intersection points in the interference.</summary>
    public int SectionPointCount => _sPoins.Length();

    /// <summary>The intersection point of address <paramref name="index"/> in the interference.</summary>
    public SectionPoint PntValue(int index) => _sPoins[index];

    /// <summary>The number of polylines of intersection in the interference.</summary>
    public int SectionLineCount => _sLines.Length();

    /// <summary>The polyline of intersection of address <paramref name="index"/> in the interference (the stored one).</summary>
    public SectionLine LineValue(int index) => _sLines[index];

    /// <summary>The number of zones of tangency in the interference.</summary>
    public int TangentZoneCount => _tZones.Length();

    /// <summary>The zone of tangency of address <paramref name="index"/> in the interference (the stored one).</summary>
    public TangentZone ZoneValue(int index) => _tZones[index];

    /// <summary>The tolerance used for the calculation.</summary>
    public double GetTolerance() => _tolerance;

    /// <summary>True if the polylines of intersection or the zones of tangency contain the point <paramref name="lePnt"/>.</summary>
    public bool Contains(in SectionPoint lePnt)
    {
        //-- LePnt.Dump(0);
        for (int l = 1; l <= _sLines.Length(); l++)
        {
            if (_sLines[l].Contains(lePnt))
            {
                return true;
            }
        }

        for (int t = 1; t <= _tZones.Length(); t++)
        {
            //-- myTZones(t).Dump(2);
            if (_tZones[t].Contains(lePnt))
            {
                return true;
            }
        }

        return false;
    }

    /// <summary>
    /// Merges a new zone of tangency into a current zone that shares points with it; returns true when done.
    /// </summary>
    public bool Insert(TangentZone laZone)
    {
        if (_tZones.Length() <= 0)
        {
            return false;
        }

        int lzin = 0;      // Index in the list of the zone of interest.
        int lunp = 0;      // Index of the 1st stop point in this zone.
        int lotp = 0;      // Index of the 2nd stop point in this zone.
        int lunl = 0;      // Index of the 1st point of the new zone.
        int lotl = 0;      // Index of the 2nd point of the new zone.
        bool same = false; // Search direction of the stop of the new zone.
        bool inserted = true; // Has the insertion succeeded ?
        int npcz = -1;     // Number of points in the current zone
        int nplz = laZone.NumberOfPoints(); // in the new zone

        // Loop on TangentZone :
        for (int iz = 1; iz <= _tZones.Length(); iz++)
        {
            // Loop on edges of the TangentZone :
            npcz = _tZones[iz].NumberOfPoints();
            int ipz0, ipz1, ipz2;
            for (ipz1 = 1; ipz1 <= npcz; ipz1++)
            {
                ipz0 = ipz1 - 1;
                if (ipz0 <= 0)
                {
                    ipz0 = npcz;
                }

                ipz2 = (ipz1 % npcz) + 1;

                // Loop on edges of the new TangentZone and search of the
                // corresponding point or edge:
                int ilz1, ilz2;
                for (ilz1 = 1; ilz1 <= nplz; ilz1++)
                {
                    ilz2 = (ilz1 % nplz) + 1;

                    if ((_tZones[iz].GetPoint(ipz1)).IsEqual(laZone.GetPoint(ilz1)))
                    {
                        if ((_tZones[iz].GetPoint(ipz0)).IsEqual(laZone.GetPoint(ilz2)))
                        {
                            lzin = iz;
                            lunp = ipz0;
                            lotp = ipz1;
                            lunl = ilz1;
                            lotl = ilz2;
                            same = false;
                            break;
                        }
                        else if ((_tZones[iz].GetPoint(ipz2)).IsEqual(laZone.GetPoint(ilz2)))
                        {
                            lzin = iz;
                            lunp = ipz1;
                            lotp = ipz2;
                            lunl = ilz1;
                            lotl = ilz2;
                            same = true;
                            break;
                        }
                        else
                        {
                            lzin = iz;
                            lunp = ipz1;
                            lunl = ilz1;
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

        int ilc;
        if (lotp != 0)
        {
            for (ilc = lotl + 1; (((ilc - 1) % nplz) + 1) != lunl; ilc++)
            {
                _tZones[lzin].InsertBefore(lotp, laZone.GetPoint(((ilc - 1) % nplz) + 1));
                if (!same)
                {
                    lotp++;
                }
            }
        }
        else if (lunp > 0)
        {
            bool loop = false;
            for (ilc = lunl; ; ilc++)
            {
                _tZones[lzin].InsertBefore(lunp, laZone.GetPoint((((ilc - 1) % nplz) + 1)));
                lunp++;
                if (loop && (((ilc - 1) % nplz) + 1) == lunl)
                {
                    break;
                }

                loop = true;
            }
        }
        else
        {
            inserted = false;
        }

        if (inserted)
        {
            // C++ copies the zone, then removes it from the sequence: here the removed object itself is reinserted
            TangentZone theNew = _tZones[lzin];
            _tZones.Remove(lzin);
            if (!Insert(theNew))
            {
                _tZones.Append(theNew);
            }
        }

        return inserted;
    }

    /// <summary>Inserts a new segment of intersection in the current polylines of intersection of the interference.</summary>
    public void Insert(in SectionPoint pdeb, in SectionPoint pfin)
    {
        bool inserted = false;
        int theLS = 0;
        bool begin = false;
        SectionPoint theBout = pfin;
        int ils, nd, nf;

        for (ils = 1; ils <= _sLines.Length(); ils++)
        {
            SectionLine sl = _sLines[ils];
            nd = sl.IsEnd(pdeb);
            nf = sl.IsEnd(pfin);
            if (nd == 1)
            {
                if (nf > 1)
                {
                    sl.Close();
                }

                inserted = true;
                theLS = ils;
                begin = true;
                break;
            }
            else if (nd > 1)
            {
                if (nf == 1)
                {
                    sl.Close();
                }

                inserted = true;
                theLS = ils;
                begin = false;
                break;
            }
            else if (nf == 1)
            {
                inserted = true;
                theLS = ils;
                begin = true;
                theBout = pdeb;
                break;
            }
            else if (nf > 1)
            {
                inserted = true;
                theLS = ils;
                begin = false;
                theBout = pdeb;
                break;
            }
        }

        if (!inserted)
        {
            SectionLine laLS = new();
            laLS.Append(pdeb);
            laLS.Append(pfin);
            _sLines.Append(laLS);
        }
        else
        {
            nd = 0;
            for (ils = 1; ils <= _sLines.Length(); ils++)
            {
                if (ils != theLS)
                {
                    nd = _sLines[ils].IsEnd(theBout);
                    if (nd == 1)
                    {
                        if (begin)
                        {
                            _sLines[theLS].Reverse();
                        }

                        _sLines[ils].Prepend(_sLines[theLS]);
                        break;
                    }
                    else if (nd > 1)
                    {
                        if (!begin)
                        {
                            _sLines[theLS].Reverse();
                        }

                        _sLines[ils].Append(_sLines[theLS]);
                        break;
                    }
                }
            }

            if (nd > 0)
            {
                _sLines.Remove(theLS);
            }
            else
            {
                if (begin)
                {
                    _sLines[theLS].Prepend(theBout);
                }
                else
                {
                    _sLines[theLS].Append(theBout);
                }
            }
        }
    }

    /// <summary>Only one argument for the intersection: clears the results.</summary>
    protected void SelfInterference(bool self)
    {
        _selfIntf = self;
        _sPoins.Clear();
        _sLines.Clear();
        _tZones.Clear();
    }

    public override string ToString()
    {
        System.Text.StringBuilder builder = new(global::Kerf.Engine.Foundation.OcctTypeNames.Of(GetType()));
        builder.Append("(points: ").Append(_sPoins.Length());
        builder.Append(", lines: ").Append(_sLines.Length());
        builder.Append(", zones: ").Append(_tZones.Length()).Append(')');
        return builder.ToString();
    }
}
