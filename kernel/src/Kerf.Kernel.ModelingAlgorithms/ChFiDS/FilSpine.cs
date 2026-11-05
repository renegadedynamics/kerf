// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_FilSpine.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_FilSpine.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Laws;
using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;

namespace Kerf.Engine.Fillets.Data;

/// <summary>
/// The spine of a fillet, with its radius: constant, or given at points of the spine (pairs of abscissa and
/// radius), from which it builds a radius law (C2) for each elementary spine.
/// </summary>
/// <remarks>
/// A working object of the fillet builders (decision 21): the radii are set through the internal mutators; the laws
/// are built when the builders append the elementary spines. The laws are the builder's own.
/// </remarks>
internal sealed class FilSpine : Spine
{
    private readonly NCollection_Sequence<Xy> _parandrad = new();
    private readonly NCollection_List<LawsFunction> _laws = new();

    public FilSpine()
    {
    }

    public FilSpine(double tol)
        : base(tol)
    {
    }

    internal override void Reset(bool allData = false)
    {
        base.Reset(allData);
        _laws.Clear();
        if (allData)
        {
            _parandrad.Clear();
        }
        else if (_parandrad.Length() > 0) // Complete parandrad; a contour without radii has none (decision 18601)
        {
            double spinedeb = FirstParameter();
            double spinefin = LastParameter();

            Xy firstUandR = _parandrad.First();
            Xy lastUandR = _parandrad.Last();
            if (Math.Abs(spinedeb - firstUandR.X()) > Constants.Resolution())
            {
                firstUandR.SetX(spinedeb);
                _parandrad.Prepend(firstUandR);
            }

            if (Math.Abs(spinefin - lastUandR.X()) > Constants.Resolution())
            {
                lastUandR.SetX(spinefin);
                _parandrad.Append(lastUandR);
            }

            if (IsPeriodic)
            {
                _parandrad[_parandrad.Length()].SetY(_parandrad[1].Y());
            }
        }
    }

    /// <summary>Sets the constant radius on the edge <paramref name="e"/>.</summary>
    internal void SetRadius(double radius, in Edge e)
    {
        _splitdone = false;
        int ie = Index(e);
        Xy firstUandR = new Xy(0.0, radius), lastUandR = new Xy(1.0, radius);
        SetRadius(firstUandR, ie);
        SetRadius(lastUandR, ie);
    }

    /// <summary>Removes the constant radius on the edge <paramref name="e"/>.</summary>
    internal void UnSetRadius(in Edge e)
    {
        _splitdone = false;
        int ie = Index(e);

        double uf = FirstParameter(ie);
        double ul = LastParameter(ie);
        int ifirst = 0, ilast = 0;
        for (int i = 1; i <= _parandrad.Length(); i++)
        {
            if (Math.Abs(_parandrad[i].X() - uf) <= Constants.Resolution())
            {
                ifirst = i;
            }

            if (Math.Abs(_parandrad[i].X() - ul) <= Constants.Resolution())
            {
                ilast = i;
            }
        }

        if (ifirst != 0 && ilast != 0)
        {
            _parandrad.Remove(ifirst, ilast);
        }
    }

    /// <summary>Sets the radius at the vertex <paramref name="v"/>.</summary>
    internal void SetRadius(double radius, in Vertex v)
    {
        double npar = Absc(v);
        Xy uandR = new Xy(npar, radius);
        SetRadius(uandR, 0);
    }

    /// <summary>Sets the constant radius on the whole spine.</summary>
    internal void SetRadius(double radius)
    {
        _parandrad.Clear();
        Xy firstUandR = new Xy(FirstParameter(), radius);
        Xy lastUandR = new Xy(LastParameter(), radius);
        SetRadius(firstUandR, 0);
        SetRadius(lastUandR, 0);
    }

    /// <summary>
    /// Sets the radius UandR.Y() at the abscissa UandR.X() (or, when <paramref name="iinC"/> is not 0, at the
    /// relative position UandR.X() on the edge IinC).
    /// </summary>
    internal void SetRadius(in Xy uandR, int iinC)
    {
        double w;
        if (iinC == 0)
        {
            w = uandR.X();
        }
        else
        {
            double Uf = FirstParameter(iinC);
            double Ul = LastParameter(iinC);
            w = Uf + uandR.X() * (Ul - Uf);
        }

        Xy pr = new Xy(w, uandR.Y());
        int i;
        for (i = 1; i <= _parandrad.Length(); i++)
        {
            if (_parandrad.Value(i).X() == w)
            {
                _parandrad.ChangeValue(i).SetY(uandR.Y());
                if (!_splitdone)
                {
                    return;
                }
                else
                {
                    break;
                }
            }
            else if (_parandrad.Value(i).X() > w)
            {
                _parandrad.InsertBefore(i, pr);
                if (!_splitdone)
                {
                    return;
                }
                else
                {
                    break;
                }
            }
        }

        if (i == _parandrad.Length() + 1)
        {
            _parandrad.Append(pr);
        }

        // si le split est done il faut rejouer la law
        // correspondant au parametre W
        if (_splitdone)
        {
            NCollection_List<ElSpine>.Iterator it = new(_elspines);
            NCollection_List<LawsFunction>.Iterator itl = new(_laws);
            ElSpine els = it.Value();
            if (els.IsPeriodic)
            {
                itl.ChangeValue() = ComputeLaw(els);
            }
            else
            {
                for (; it.More(); it.Next(), itl.Next())
                {
                    els = it.Value();
                    double uf = els.FirstParameter();
                    double ul = els.LastParameter();
                    if (uf <= w && w <= ul)
                    {
                        itl.ChangeValue() = ComputeLaw(els);
                    }
                }
            }
        }
    }

    /// <summary>Removes the radius at the vertex <paramref name="v"/>.</summary>
    internal void UnSetRadius(in Vertex v)
    {
        double npar = Absc(v);
        for (int i = 1; i <= _parandrad.Length(); i++)
        {
            if (_parandrad.Value(i).X() == npar)
            {
                _parandrad.Remove(i);
                break;
            }
        }
    }

    /// <summary>
    /// The law of evolution of the radius on the whole spine: the law's range is mapped onto the spine's, from its
    /// first vertex to its last, and the radius interpolates the law's values at <see cref="LawSamples"/> + 1 evenly
    /// spaced points. OCCT built a composite law from <paramref name="c"/>, dropped it and cleared the radii, so a fillet
    /// by a law failed (decision 1433). <paramref name="iinC"/> is unused, as in OCCT.
    /// </summary>
    internal void SetRadius(LawsFunction c, int iinC)
    {
        _ = iinC;
        _splitdone = false;
        _parandrad.Clear();
        c.Bounds(out double lawFirst, out double lawLast);
        double spineFirst = FirstParameter(), spineLast = LastParameter();
        for (int i = 0; i <= LawSamples; i++)
        {
            double t = (double)i / LawSamples;
            SetRadius(new Xy(spineFirst + t * (spineLast - spineFirst), c.Value(lawFirst + t * (lawLast - lawFirst))), 0);
        }
    }

    /// <summary>The intervals in which a radius law is sampled (decision 1433).</summary>
    internal const int LawSamples = 32;

    /// <summary>True if the radius is constant all along the spine.</summary>
    public bool IsConstant()
    {
        if (_parandrad.IsEmpty())
        {
            return false;
        }

        bool isconst = true;
        double radius = _parandrad[1].Y();
        for (int i = 2; i <= _parandrad.Length(); i++)
        {
            if (Math.Abs(radius - _parandrad[i].Y()) > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                isconst = false;
                break;
            }
        }

        return isconst;
    }

    /// <summary>True if the radius is constant all along the edge <paramref name="ie"/>.</summary>
    public bool IsConstant(int ie)
    {
        double uf = FirstParameter(ie);
        double ul = LastParameter(ie);

        double startRad = 0.0, par, rad;
        int i;
        for (i = 1; i < _parandrad.Length(); i++)
        {
            par = _parandrad[i].X();
            rad = _parandrad[i].Y();
            double nextpar = _parandrad[i + 1].X();
            if (Math.Abs(uf - par) <= Constants.Resolution()
                || (par < uf && uf < nextpar && nextpar - uf > Constants.Resolution()))
            {
                startRad = rad;
                break;
            }
        }

        for (i++; i <= _parandrad.Length(); i++)
        {
            par = _parandrad[i].X();
            rad = _parandrad[i].Y();
            if (Math.Abs(rad - startRad) > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                return false;
            }

            if (Math.Abs(ul - par) <= Constants.Resolution())
            {
                return true;
            }

            if (par > ul)
            {
                return true;
            }
        }

        return true;
    }

    /// <summary>The radius on the edge <paramref name="e"/>, if constant there.</summary>
    public double Radius(in Edge e)
    {
        int ie = Index(e);
        return Radius(ie);
    }

    /// <summary>The radius on the edge <paramref name="ie"/>; throws <see cref="DomainErrorException"/> if it is not
    /// constant there.</summary>
    public double Radius(int ie)
    {
        double uf = FirstParameter(ie);
        double ul = LastParameter(ie);

        double startRad = 0.0, par, rad;
        int i;
        for (i = 1; i < _parandrad.Length(); i++)
        {
            par = _parandrad[i].X();
            rad = _parandrad[i].Y();
            double nextpar = _parandrad[i + 1].X();
            if (Math.Abs(uf - par) <= Constants.Resolution()
                || (par < uf && uf < nextpar && nextpar - uf > Constants.Resolution()))
            {
                startRad = rad;
                break;
            }
        }

        for (i++; i <= _parandrad.Length(); i++)
        {
            par = _parandrad[i].X();
            rad = _parandrad[i].Y();
            if (Math.Abs(rad - startRad) > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                throw new DomainErrorException("Edge is not constant");
            }

            if (Math.Abs(ul - par) <= Constants.Resolution())
            {
                return startRad;
            }

            if (par > ul)
            {
                return startRad;
            }
        }

        return startRad;
    }

    /// <summary>The radius of the spine; throws <see cref="DomainErrorException"/> if it is not constant.</summary>
    public double Radius()
    {
        if (!IsConstant())
        {
            throw new DomainErrorException("Spine is not constant");
        }

        return _parandrad[1].Y();
    }

    internal override void AppendElSpine(ElSpine els)
    {
        base.AppendElSpine(els);
        AppendLaw(els);
    }

    private void AppendLaw(ElSpine els)
    {
        Composite l = ComputeLaw(els);
        _laws.Append(l);
    }

    private static void Mklaw(NCollection_List<LawsFunction> res, NCollection_Sequence<Xy> pr, double curdeb,
                              double curfin, double rdeb, double rfin, bool recadre, double deb, double fin,
                              double tol3d)
    {
        NCollection_Sequence<Xy> npr = new();
        double rad = rdeb, raf = rfin;
        bool yaunpointsurledeb = false;
        bool yaunpointsurlefin = false;
        if (!pr.IsEmpty())
        {
            for (int i = 1; i <= pr.Length(); i++)
            {
                Xy cur = pr.Value(i);
                double wcur = cur.X();
                if (recadre)
                {
                    wcur = global::Kerf.Engine.Geometry.ElCLib.InPeriod(wcur, deb, fin);
                }

                if (curdeb - tol3d <= wcur && wcur <= curfin + tol3d)
                {
                    if (wcur - curdeb < tol3d)
                    {
                        yaunpointsurledeb = true;
                        Xy ncur = cur;
                        if (rdeb < 0.0)
                        {
                            rad = cur.Y();
                        }

                        ncur.SetCoord(curdeb, rad);
                        npr.Append(ncur);
                    }
                    else if (curfin - wcur < tol3d)
                    {
                        yaunpointsurlefin = true;
                        Xy ncur = cur;
                        if (rfin < 0.0)
                        {
                            raf = cur.Y();
                        }

                        ncur.SetCoord(curfin, raf);
                        npr.Append(ncur);
                    }
                    else
                    {
                        npr.Append(new Xy(wcur, cur.Y()));
                    }
                }
            }
        }

        if (npr.IsEmpty())
        {
            if (rdeb < 0.0 && rfin < 0.0)
            {
                throw new DomainErrorException("Impossible to create the law");
            }
            else if (rdeb < 0.0 || rfin < 0.0)
            {
                double r = (rfin < 0.0) ? rdeb : rfin;
                Constant loi = new();
                loi.Set(r, curdeb, curfin);
                res.Append(loi);
            }
            else
            {
                S loi = new();
                loi.Set(curdeb, rdeb, curfin, rfin);
                res.Append(loi);
            }
        }
        else
        {
            if (!yaunpointsurledeb && rdeb >= 0.0)
            {
                npr.Append(new Xy(curdeb, rdeb));
            }

            if (!yaunpointsurlefin && rfin >= 0.0)
            {
                npr.Append(new Xy(curfin, rfin));
            }

            int nbp = npr.Length();
            int i;
            for (i = 1; i < nbp; i++)
            {
                for (int j = i + 1; j <= nbp; j++)
                {
                    if (npr.Value(i).X() > npr.Value(j).X())
                    {
                        Xy temp = npr.Value(i);
                        npr.ChangeValue(i) = npr.Value(j);
                        npr.ChangeValue(j) = temp;
                    }
                }
            }

            // Duplicates are removed.
            bool fini = (nbp <= 1);
            i = 1;
            while (!fini)
            {
                if (Math.Abs(npr.Value(i).X() - npr.Value(i + 1).X()) < tol3d)
                {
                    npr.Remove(i);
                    nbp--;
                }
                else
                {
                    i++;
                }

                fini = (i >= nbp);
            }

            if (rad < 0.0)
            {
                Constant loi = new();
                loi.Set(npr.First().Y(), curdeb, npr.First().X());
                res.Append(loi);
            }

            if (nbp > 1)
            {
                NCollection_Array1<Point2D> tpr = new(1, nbp);
                for (int l = 1; l <= nbp; l++)
                {
                    tpr[l] = tpr[l].WithXY(npr.Value(l));
                }

                Interpol curloi = new();
                curloi.Set(tpr, 0.0, 0.0, false);
                res.Append(curloi);
            }

            if (raf < 0.0)
            {
                Constant loi = new();
                loi.Set(npr.Last().Y(), npr.Last().X(), curfin);
                res.Append(loi);
            }
        }
    }

    private Composite ComputeLaw(ElSpine els)
    {
        double tol3d = global::Kerf.Engine.Foundation.Precision.Confusion();
        double deb, fin, curdeb, curfin;
        curdeb = deb = els.FirstParameter();
        curfin = fin = els.LastParameter();
        int ideb = Index(deb, true);
        int ifin = Index(fin, false);
        int len = EdgeCount;
        // if the spine is periodic, attention to the index and parameters
        double spinedeb = FirstParameter();
        double spinefin = LastParameter();

        int nbed = ifin - ideb + 1;
        int bidfin = ifin;

        Composite loi = new();
        NCollection_List<LawsFunction> list = loi.ChangeLaws();
        double rdeb = 0.0, rfin = 0.0, rcur;
        int icur = 1;
        Constant? lastloi = null;
        bool lawencours = false;

        if (IsPeriodic)
        {
            if (deb < 0 && ideb > ifin)
            {
                bidfin += len;
            }
            else if (fin > LastParameter(len) && ideb > ifin)
            {
                bidfin += len;
            }

            nbed = bidfin - ideb + 1;
        }

        NCollection_Array1<int> ind = new(1, nbed);
        int j = 1;
        for (int i = ideb; i <= bidfin; i++)
        {
            ind[j++] = ((i - 1) % len) + 1;
        }

        if (els.IsPeriodic)
        {
            // A pereodic composite is created at range, which is eventually
            // offset relatively to the elspine, to avoid a single point at
            // origin.
            loi.SetPeriodic();
            // Is there a constant edge?
            int k;
            for (k = 1; k <= len; k++)
            {
                if (IsConstant(k))
                { // yes  !
                    spinedeb = deb = curdeb = FirstParameter(k);
                    spinefin = fin = deb + Period();
                    for (int l = 1; l <= len; l++)
                    {
                        ind[l] = ((k + l - 2) % len) + 1;
                    }

                    rdeb = rfin = Radius(k);
                    icur++;
                    if (len == 1)
                    {
                        curfin = LastParameter(k); // because InPeriod will make 0.!!!
                    }
                    else
                    {
                        curfin = global::Kerf.Engine.Geometry.ElCLib.InPeriod(LastParameter(k), spinedeb, spinefin);
                    }

                    Constant curloi = new();
                    curloi.Set(rdeb, curdeb, curfin);
                    list.Append(curloi);
                    curdeb = curfin;
                    break;
                }
            }

            if (k > len)
            { // no !
                if (_parandrad.IsEmpty())
                {
                    throw new DomainErrorException("Radius not defined");
                }

                int nbp = _parandrad.Length();
                if (nbp > 1)
                {
                    deb = _parandrad.First().X();
                    fin = deb + Period();
                    if (_parandrad.Last().X() - fin < -tol3d)
                    {
                        nbp++;
                    }
                }
                else
                {
                    nbp++;
                }

                NCollection_Array1<Point2D> pr = new(1, nbp);
                for (int l = 1; l < nbp; l++)
                {
                    pr[l] = pr[l].WithXY(_parandrad[l]);
                }

                pr[nbp] = pr[nbp].WithCoord(fin, pr[1].Y);
                Interpol curloi = new();
                curloi.Set(pr, true);
                list.Append(curloi);
                return loi;
            }
        }
        else if (IsPeriodic)
        {
            // start radius.
            if (IsConstant(ind[1]))
            {
                rdeb = Radius(ind[1]);
                curfin = LastParameter(ind[1]);
                curfin = global::Kerf.Engine.Geometry.ElCLib.InPeriod(curfin, spinedeb + tol3d, spinefin + tol3d);
                curfin = Min(fin, curfin);
                Constant curloi = new();
                curloi.Set(rdeb, curdeb, curfin);
                list.Append(curloi);
                curdeb = curfin;
                icur++;
            }
            else
            {
                // There is inevitably kpart right before!
                int iprec = (ind[1] - 1);
                if (iprec == 0)
                {
                    iprec = len;
                }

                if (IsConstant(iprec))
                {
                    rdeb = Radius(iprec);
                }
                else
                {
                    throw new DomainErrorException("AppendLaw : previous constant is missing!");
                }

                lawencours = true;
            }

            // the raduis at end.
            if (IsConstant(ind[nbed]))
            {
                rfin = Radius(ind[nbed]);
            }
            else
            {
                // There is inevitably kpart right after!
                int isuiv = (ind[nbed] + 1);
                if (isuiv == len + 1)
                {
                    isuiv = 1;
                }

                if (IsConstant(isuiv))
                {
                    rfin = Radius(isuiv);
                }
                else
                {
                    throw new DomainErrorException("AppendLaw : next constant is missing!");
                }
            }
        }
        else
        {
            // the radius at start.
            if (IsConstant(ind[1]))
            {
                rdeb = Radius(ind[1]);
                curfin = Min(fin, LastParameter(ind[1]));
                Constant curloi = new();
                curloi.Set(rdeb, curdeb, curfin);
                list.Append(curloi);
                curdeb = curfin;
                icur++;
            }
            else
            {
                if (ind[1] > 1)
                {
                    if (IsConstant(ind[1] - 1))
                    {
                        rdeb = Radius(ind[1] - 1);
                    }
                    else
                    {
                        throw new DomainErrorException("AppendLaw : previous constant is missing");
                    }
                }
                else if (_parandrad.IsEmpty())
                {
                    throw new DomainErrorException("AppendLaw : no radius on vertex");
                }
                else
                {
                    rdeb = -1.0;
                }

                lawencours = true;
            }

            // the radius at end.
            if (IsConstant(ind[nbed]))
            {
                rfin = Radius(ind[nbed]);
            }
            else
            {
                if (ind[nbed] < len)
                {
                    if (IsConstant(ind[nbed] + 1))
                    {
                        rfin = Radius(ind[nbed] + 1);
                    }
                    else
                    {
                        throw new DomainErrorException("AppendLaw : next constant is missing");
                    }
                }
                else if (_parandrad.IsEmpty())
                {
                    throw new DomainErrorException("AppendLaw : no radius on vertex");
                }
                else
                {
                    rfin = -1.0;
                }
            }
        }

        // There are infos on the extremities of the elspine,
        // all edges are parsed
        for (; icur <= nbed; icur++)
        {
            if (IsConstant(ind[icur]))
            {
                rcur = Radius(ind[icur]);
                if (lawencours)
                {
                    NCollection_List<LawsFunction> temp = new();
                    Mklaw(temp, _parandrad, curdeb, curfin, rdeb, rcur, IsPeriodic, spinedeb, spinefin, tol3d);
                    list.Append(temp);
                    lawencours = false;
                    curdeb = curfin;
                }

                curfin = LastParameter(ind[icur]);
                if (IsPeriodic)
                {
                    curfin = global::Kerf.Engine.Geometry.ElCLib.InPeriod(curfin, spinedeb + tol3d, spinefin + tol3d);
                    if (ind[icur] == ind[nbed])
                    {
                        // Attention the curfin can be wrong if the last edge passes
                        // above the  origin periodic spline.
                        double biddeb = FirstParameter(ind[icur]);
                        biddeb = global::Kerf.Engine.Geometry.ElCLib.InPeriod(biddeb, spinedeb + tol3d, spinefin + tol3d);
                        if (biddeb >= curfin)
                        {
                            curfin = fin;
                        }
                        else
                        {
                            curfin = Min(fin, curfin);
                        }
                    }
                    else
                    {
                        curfin = Min(fin, curfin);
                    }
                }

                if ((curfin - curdeb) > tol3d)
                {
                    rdeb = rcur;
                    Constant curloi = new();
                    curloi.Set(rdeb, curdeb, curfin);
                    list.Append(curloi);
                    curdeb = curfin;
                }
            }
            else
            {
                curfin = LastParameter(ind[icur]);
                if (IsPeriodic)
                {
                    curfin = global::Kerf.Engine.Geometry.ElCLib.InPeriod(curfin, spinedeb + tol3d, spinefin + tol3d);
                }

                curfin = Min(fin, curfin);
                lawencours = true;
                if (ind[icur] == ind[nbed])
                {
                    // Attention the curfin can be wrong if the last edge passes
                    // above the  origin periodic spline.
                    if (IsPeriodic)
                    {
                        double biddeb = FirstParameter(ind[icur]);
                        curfin = LastParameter(ind[icur]);
                        biddeb = global::Kerf.Engine.Geometry.ElCLib.InPeriod(biddeb, spinedeb + tol3d, spinefin + tol3d);
                        curfin = global::Kerf.Engine.Geometry.ElCLib.InPeriod(curfin, spinedeb + tol3d, spinefin + tol3d);
                        if (biddeb >= curfin)
                        {
                            curfin = fin;
                        }
                        else
                        {
                            curfin = Min(fin, curfin);
                        }
                    }
                    // or if it is the end of spine with extension.
                    else if (ind[icur] == len)
                    {
                        curfin = fin;
                    }

                    NCollection_List<LawsFunction> temp = new();
                    Mklaw(temp, _parandrad, curdeb, curfin, rdeb, rfin, IsPeriodic, spinedeb, spinefin, tol3d);
                    list.Append(temp);
                }
            }
        }

        if (lastloi is not null)
        {
            list.Append(lastloi);
        }

        return loi;
    }

    /// <summary>The radius law of the elementary spine <paramref name="els"/>, or null.</summary>
    public Composite? Law(ElSpine els)
    {
        NCollection_List<ElSpine>.Iterator itsp = new(_elspines);
        NCollection_List<LawsFunction>.Iterator itl = new(_laws);
        for (; itsp.More(); itsp.Next(), itl.Next())
        {
            if (els == itsp.Value())
            {
                return itl.Value() as Composite;
            }
        }

        return null;
    }

    /// <summary>The elementary law on the (non-constant) edge <paramref name="e"/>, for reading and writing.</summary>
    internal ref LawsFunction? ChangeLaw(in Edge e)
    {
        if (!SplitDone())
        {
            throw new DomainErrorException("ChFiDS_FilSpine::ChangeLaw : the limits are not up-to-date");
        }

        int ie = Index(e);
        if (IsConstant(ie))
        {
            throw new DomainErrorException("ChFiDS_FilSpine::ChangeLaw : no law on constant edges");
        }

        ElSpine? hsp = ElSpine(ie);
        double w = 0.5 * (FirstParameter(ie) + LastParameter(ie));
        Composite lc = Law(hsp!)!;
        return ref lc.ChangeElementaryLaw(w);
    }

    /// <summary>The maximum radius, from the radii set and the laws (sampled at six points each).</summary>
    public double MaxRadFromSeqAndLaws()
    {
        double maxRad = 0.0;

        for (int i = 1; i <= _parandrad.Length(); i++)
        {
            if (_parandrad[i].Y() > maxRad)
            {
                maxRad = _parandrad[i].Y();
            }
        }

        NCollection_List<LawsFunction>.Iterator itl = new(_laws);
        for (; itl.More(); itl.Next())
        {
            LawsFunction law = itl.Value();
            double fpar, lpar, par, delta, rad;
            law.Bounds(out fpar, out lpar);
            delta = (lpar - fpar) * 0.2;
            for (int i = 0; i <= 4; i++)
            {
                par = fpar + i * delta;
                rad = law.Value(par);
                if (rad > maxRad)
                {
                    maxRad = rad;
                }
            }

            rad = law.Value(lpar);
            if (rad > maxRad)
            {
                maxRad = rad;
            }
        }

        return maxRad;
    }
}
