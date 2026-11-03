// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_FilSpine.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_FilSpine.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Kerf.Topology;

namespace Occt;

/// <summary>
/// The spine of a fillet, with its radius: constant, or given at points of the spine (pairs of abscissa and
/// radius), from which it builds a radius law (C2) for each elementary spine.
/// </summary>
/// <remarks>
/// A working object of the fillet builders (decision 21): the radii are set through the internal mutators; the laws
/// are built when the builders append the elementary spines. The laws are the builder's own.
/// </remarks>
public sealed class ChFiDS_FilSpine : ChFiDS_Spine
{
    private readonly NCollection_Sequence<Xy> parandrad = new();
    private readonly NCollection_List<Law_Function> laws = new();

    public ChFiDS_FilSpine()
    {
    }

    public ChFiDS_FilSpine(double Tol)
        : base(Tol)
    {
    }

    internal override void Reset(bool AllData = false)
    {
        base.Reset(AllData);
        laws.Clear();
        if (AllData)
        {
            parandrad.Clear();
        }
        else // Complete parandrad
        {
            double spinedeb = FirstParameter();
            double spinefin = LastParameter();

            Xy FirstUandR = parandrad.First();
            Xy LastUandR = parandrad.Last();
            if (Math.Abs(spinedeb - FirstUandR.X()) > Constants.Resolution())
            {
                FirstUandR.SetX(spinedeb);
                parandrad.Prepend(FirstUandR);
            }

            if (Math.Abs(spinefin - LastUandR.X()) > Constants.Resolution())
            {
                LastUandR.SetX(spinefin);
                parandrad.Append(LastUandR);
            }

            if (IsPeriodic())
            {
                parandrad[parandrad.Length()].SetY(parandrad[1].Y());
            }
        }
    }

    /// <summary>Sets the constant radius on the edge <paramref name="E"/>.</summary>
    internal void SetRadius(double Radius, in Edge E)
    {
        splitdone = false;
        int IE = Index(E);
        Xy FirstUandR = new Xy(0.0, Radius), LastUandR = new Xy(1.0, Radius);
        SetRadius(FirstUandR, IE);
        SetRadius(LastUandR, IE);
    }

    /// <summary>Removes the constant radius on the edge <paramref name="E"/>.</summary>
    internal void UnSetRadius(in Edge E)
    {
        splitdone = false;
        int IE = Index(E);

        double Uf = FirstParameter(IE);
        double Ul = LastParameter(IE);
        int ifirst = 0, ilast = 0;
        for (int i = 1; i <= parandrad.Length(); i++)
        {
            if (Math.Abs(parandrad[i].X() - Uf) <= Constants.Resolution())
            {
                ifirst = i;
            }

            if (Math.Abs(parandrad[i].X() - Ul) <= Constants.Resolution())
            {
                ilast = i;
            }
        }

        if (ifirst != 0 && ilast != 0)
        {
            parandrad.Remove(ifirst, ilast);
        }
    }

    /// <summary>Sets the radius at the vertex <paramref name="V"/>.</summary>
    internal void SetRadius(double Radius, in Vertex V)
    {
        double npar = Absc(V);
        Xy UandR = new Xy(npar, Radius);
        SetRadius(UandR, 0);
    }

    /// <summary>Sets the constant radius on the whole spine.</summary>
    internal void SetRadius(double Radius)
    {
        parandrad.Clear();
        Xy FirstUandR = new Xy(FirstParameter(), Radius);
        Xy LastUandR = new Xy(LastParameter(), Radius);
        SetRadius(FirstUandR, 0);
        SetRadius(LastUandR, 0);
    }

    /// <summary>
    /// Sets the radius UandR.Y() at the abscissa UandR.X() (or, when <paramref name="IinC"/> is not 0, at the
    /// relative position UandR.X() on the edge IinC).
    /// </summary>
    internal void SetRadius(in Xy UandR, int IinC)
    {
        double W;
        if (IinC == 0)
        {
            W = UandR.X();
        }
        else
        {
            double Uf = FirstParameter(IinC);
            double Ul = LastParameter(IinC);
            W = Uf + UandR.X() * (Ul - Uf);
        }

        Xy pr = new Xy(W, UandR.Y());
        int i;
        for (i = 1; i <= parandrad.Length(); i++)
        {
            if (parandrad.Value(i).X() == W)
            {
                parandrad.ChangeValue(i).SetY(UandR.Y());
                if (!splitdone)
                {
                    return;
                }
                else
                {
                    break;
                }
            }
            else if (parandrad.Value(i).X() > W)
            {
                parandrad.InsertBefore(i, pr);
                if (!splitdone)
                {
                    return;
                }
                else
                {
                    break;
                }
            }
        }

        if (i == parandrad.Length() + 1)
        {
            parandrad.Append(pr);
        }

        // si le split est done il faut rejouer la law
        // correspondant au parametre W
        if (splitdone)
        {
            NCollection_List<ChFiDS_ElSpine>.Iterator It = new(elspines);
            NCollection_List<Law_Function>.Iterator Itl = new(laws);
            ChFiDS_ElSpine Els = It.Value();
            if (Els.IsPeriodic)
            {
                Itl.ChangeValue() = ComputeLaw(Els);
            }
            else
            {
                for (; It.More(); It.Next(), Itl.Next())
                {
                    Els = It.Value();
                    double uf = Els.FirstParameter();
                    double ul = Els.LastParameter();
                    if (uf <= W && W <= ul)
                    {
                        Itl.ChangeValue() = ComputeLaw(Els);
                    }
                }
            }
        }
    }

    /// <summary>Removes the radius at the vertex <paramref name="V"/>.</summary>
    internal void UnSetRadius(in Vertex V)
    {
        double npar = Absc(V);
        for (int i = 1; i <= parandrad.Length(); i++)
        {
            if (parandrad.Value(i).X() == npar)
            {
                parandrad.Remove(i);
                break;
            }
        }
    }

    /// <summary>
    /// The law of evolution of the radius on the whole spine. OCCT builds a composite law from <paramref name="C"/>,
    /// drops it, and clears the radii.
    /// </summary>
    internal void SetRadius(Law_Function C, int IinC)
    {
        splitdone = false;
        Law_Composite prout = new();
        NCollection_List<Law_Function> lst = prout.ChangeLaws();
        lst.Append(C);
        parandrad.Clear();
    }

    /// <summary>True if the radius is constant all along the spine.</summary>
    public bool IsConstant()
    {
        if (parandrad.IsEmpty())
        {
            return false;
        }

        bool isconst = true;
        double Radius = parandrad[1].Y();
        for (int i = 2; i <= parandrad.Length(); i++)
        {
            if (Math.Abs(Radius - parandrad[i].Y()) > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                isconst = false;
                break;
            }
        }

        return isconst;
    }

    /// <summary>True if the radius is constant all along the edge <paramref name="IE"/>.</summary>
    public bool IsConstant(int IE)
    {
        double Uf = FirstParameter(IE);
        double Ul = LastParameter(IE);

        double StartRad = 0.0, par, rad;
        int i;
        for (i = 1; i < parandrad.Length(); i++)
        {
            par = parandrad[i].X();
            rad = parandrad[i].Y();
            double nextpar = parandrad[i + 1].X();
            if (Math.Abs(Uf - par) <= Constants.Resolution()
                || (par < Uf && Uf < nextpar && nextpar - Uf > Constants.Resolution()))
            {
                StartRad = rad;
                break;
            }
        }

        for (i++; i <= parandrad.Length(); i++)
        {
            par = parandrad[i].X();
            rad = parandrad[i].Y();
            if (Math.Abs(rad - StartRad) > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                return false;
            }

            if (Math.Abs(Ul - par) <= Constants.Resolution())
            {
                return true;
            }

            if (par > Ul)
            {
                return true;
            }
        }

        return true;
    }

    /// <summary>The radius on the edge <paramref name="E"/>, if constant there.</summary>
    public double Radius(in Edge E)
    {
        int IE = Index(E);
        return Radius(IE);
    }

    /// <summary>The radius on the edge <paramref name="IE"/>; throws <see cref="DomainErrorException"/> if it is not
    /// constant there.</summary>
    public double Radius(int IE)
    {
        double Uf = FirstParameter(IE);
        double Ul = LastParameter(IE);

        double StartRad = 0.0, par, rad;
        int i;
        for (i = 1; i < parandrad.Length(); i++)
        {
            par = parandrad[i].X();
            rad = parandrad[i].Y();
            double nextpar = parandrad[i + 1].X();
            if (Math.Abs(Uf - par) <= Constants.Resolution()
                || (par < Uf && Uf < nextpar && nextpar - Uf > Constants.Resolution()))
            {
                StartRad = rad;
                break;
            }
        }

        for (i++; i <= parandrad.Length(); i++)
        {
            par = parandrad[i].X();
            rad = parandrad[i].Y();
            if (Math.Abs(rad - StartRad) > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                throw new DomainErrorException("Edge is not constant");
            }

            if (Math.Abs(Ul - par) <= Constants.Resolution())
            {
                return StartRad;
            }

            if (par > Ul)
            {
                return StartRad;
            }
        }

        return StartRad;
    }

    /// <summary>The radius of the spine; throws <see cref="DomainErrorException"/> if it is not constant.</summary>
    public double Radius()
    {
        if (!IsConstant())
        {
            throw new DomainErrorException("Spine is not constant");
        }

        return parandrad[1].Y();
    }

    internal override void AppendElSpine(ChFiDS_ElSpine Els)
    {
        base.AppendElSpine(Els);
        AppendLaw(Els);
    }

    private void AppendLaw(ChFiDS_ElSpine Els)
    {
        Law_Composite l = ComputeLaw(Els);
        laws.Append(l);
    }

    private static void mklaw(NCollection_List<Law_Function> res, NCollection_Sequence<Xy> pr, double curdeb,
                              double curfin, double Rdeb, double Rfin, bool recadre, double deb, double fin,
                              double tol3d)
    {
        NCollection_Sequence<Xy> npr = new();
        double rad = Rdeb, raf = Rfin;
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
                        if (Rdeb < 0.0)
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
                        if (Rfin < 0.0)
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
            if (Rdeb < 0.0 && Rfin < 0.0)
            {
                throw new DomainErrorException("Impossible to create the law");
            }
            else if (Rdeb < 0.0 || Rfin < 0.0)
            {
                double r = (Rfin < 0.0) ? Rdeb : Rfin;
                Law_Constant loi = new();
                loi.Set(r, curdeb, curfin);
                res.Append(loi);
            }
            else
            {
                Law_S loi = new();
                loi.Set(curdeb, Rdeb, curfin, Rfin);
                res.Append(loi);
            }
        }
        else
        {
            if (!yaunpointsurledeb && Rdeb >= 0.0)
            {
                npr.Append(new Xy(curdeb, Rdeb));
            }

            if (!yaunpointsurlefin && Rfin >= 0.0)
            {
                npr.Append(new Xy(curfin, Rfin));
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
                Law_Constant loi = new();
                loi.Set(npr.First().Y(), curdeb, npr.First().X());
                res.Append(loi);
            }

            if (nbp > 1)
            {
                NCollection_Array1<Point2D> tpr = new(1, nbp);
                for (int l = 1; l <= nbp; l++)
                {
                    tpr[l].SetXY(npr.Value(l));
                }

                Law_Interpol curloi = new();
                curloi.Set(tpr, 0.0, 0.0, false);
                res.Append(curloi);
            }

            if (raf < 0.0)
            {
                Law_Constant loi = new();
                loi.Set(npr.Last().Y(), npr.Last().X(), curfin);
                res.Append(loi);
            }
        }
    }

    private Law_Composite ComputeLaw(ChFiDS_ElSpine Els)
    {
        double tol3d = global::Kerf.Engine.Foundation.Precision.Confusion();
        double deb, fin, curdeb, curfin;
        curdeb = deb = Els.FirstParameter();
        curfin = fin = Els.LastParameter();
        int ideb = Index(deb, true);
        int ifin = Index(fin, false);
        int len = NbEdges();
        // if the spine is periodic, attention to the index and parameters
        double spinedeb = FirstParameter();
        double spinefin = LastParameter();

        int nbed = ifin - ideb + 1;
        int bidfin = ifin;

        Law_Composite loi = new();
        NCollection_List<Law_Function> list = loi.ChangeLaws();
        double Rdeb = 0.0, Rfin = 0.0, Rcur;
        int icur = 1;
        Law_Constant? lastloi = null;
        bool lawencours = false;

        if (IsPeriodic())
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

        if (Els.IsPeriodic)
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

                    Rdeb = Rfin = Radius(k);
                    icur++;
                    if (len == 1)
                    {
                        curfin = LastParameter(k); // because InPeriod will make 0.!!!
                    }
                    else
                    {
                        curfin = global::Kerf.Engine.Geometry.ElCLib.InPeriod(LastParameter(k), spinedeb, spinefin);
                    }

                    Law_Constant curloi = new();
                    curloi.Set(Rdeb, curdeb, curfin);
                    list.Append(curloi);
                    curdeb = curfin;
                    break;
                }
            }

            if (k > len)
            { // no !
                if (parandrad.IsEmpty())
                {
                    throw new DomainErrorException("Radius not defined");
                }

                int nbp = parandrad.Length();
                if (nbp > 1)
                {
                    deb = parandrad.First().X();
                    fin = deb + Period();
                    if (parandrad.Last().X() - fin < -tol3d)
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
                    pr[l].SetXY(parandrad[l]);
                }

                pr[nbp].SetCoord(fin, pr[1].Y());
                Law_Interpol curloi = new();
                curloi.Set(pr, true);
                list.Append(curloi);
                return loi;
            }
        }
        else if (IsPeriodic())
        {
            // start radius.
            if (IsConstant(ind[1]))
            {
                Rdeb = Radius(ind[1]);
                curfin = LastParameter(ind[1]);
                curfin = global::Kerf.Engine.Geometry.ElCLib.InPeriod(curfin, spinedeb + tol3d, spinefin + tol3d);
                curfin = Min(fin, curfin);
                Law_Constant curloi = new();
                curloi.Set(Rdeb, curdeb, curfin);
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
                    Rdeb = Radius(iprec);
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
                Rfin = Radius(ind[nbed]);
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
                    Rfin = Radius(isuiv);
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
                Rdeb = Radius(ind[1]);
                curfin = Min(fin, LastParameter(ind[1]));
                Law_Constant curloi = new();
                curloi.Set(Rdeb, curdeb, curfin);
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
                        Rdeb = Radius(ind[1] - 1);
                    }
                    else
                    {
                        throw new DomainErrorException("AppendLaw : previous constant is missing");
                    }
                }
                else if (parandrad.IsEmpty())
                {
                    throw new DomainErrorException("AppendLaw : no radius on vertex");
                }
                else
                {
                    Rdeb = -1.0;
                }

                lawencours = true;
            }

            // the radius at end.
            if (IsConstant(ind[nbed]))
            {
                Rfin = Radius(ind[nbed]);
            }
            else
            {
                if (ind[nbed] < len)
                {
                    if (IsConstant(ind[nbed] + 1))
                    {
                        Rfin = Radius(ind[nbed] + 1);
                    }
                    else
                    {
                        throw new DomainErrorException("AppendLaw : next constant is missing");
                    }
                }
                else if (parandrad.IsEmpty())
                {
                    throw new DomainErrorException("AppendLaw : no radius on vertex");
                }
                else
                {
                    Rfin = -1.0;
                }
            }
        }

        // There are infos on the extremities of the elspine,
        // all edges are parsed
        for (; icur <= nbed; icur++)
        {
            if (IsConstant(ind[icur]))
            {
                Rcur = Radius(ind[icur]);
                if (lawencours)
                {
                    NCollection_List<Law_Function> temp = new();
                    mklaw(temp, parandrad, curdeb, curfin, Rdeb, Rcur, IsPeriodic(), spinedeb, spinefin, tol3d);
                    list.Append(temp);
                    lawencours = false;
                    curdeb = curfin;
                }

                curfin = LastParameter(ind[icur]);
                if (IsPeriodic())
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
                    Rdeb = Rcur;
                    Law_Constant curloi = new();
                    curloi.Set(Rdeb, curdeb, curfin);
                    list.Append(curloi);
                    curdeb = curfin;
                }
            }
            else
            {
                curfin = LastParameter(ind[icur]);
                if (IsPeriodic())
                {
                    curfin = global::Kerf.Engine.Geometry.ElCLib.InPeriod(curfin, spinedeb + tol3d, spinefin + tol3d);
                }

                curfin = Min(fin, curfin);
                lawencours = true;
                if (ind[icur] == ind[nbed])
                {
                    // Attention the curfin can be wrong if the last edge passes
                    // above the  origin periodic spline.
                    if (IsPeriodic())
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

                    NCollection_List<Law_Function> temp = new();
                    mklaw(temp, parandrad, curdeb, curfin, Rdeb, Rfin, IsPeriodic(), spinedeb, spinefin, tol3d);
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

    /// <summary>The radius law of the elementary spine <paramref name="Els"/>, or null.</summary>
    public Law_Composite? Law(ChFiDS_ElSpine Els)
    {
        NCollection_List<ChFiDS_ElSpine>.Iterator Itsp = new(elspines);
        NCollection_List<Law_Function>.Iterator Itl = new(laws);
        for (; Itsp.More(); Itsp.Next(), Itl.Next())
        {
            if (Els == Itsp.Value())
            {
                return Itl.Value() as Law_Composite;
            }
        }

        return null;
    }

    /// <summary>The elementary law on the (non-constant) edge <paramref name="E"/>, for reading and writing.</summary>
    internal ref Law_Function? ChangeLaw(in Edge E)
    {
        if (!SplitDone())
        {
            throw new DomainErrorException("ChFiDS_FilSpine::ChangeLaw : the limits are not up-to-date");
        }

        int IE = Index(E);
        if (IsConstant(IE))
        {
            throw new DomainErrorException("ChFiDS_FilSpine::ChangeLaw : no law on constant edges");
        }

        ChFiDS_ElSpine? hsp = ElSpine(IE);
        double w = 0.5 * (FirstParameter(IE) + LastParameter(IE));
        Law_Composite lc = Law(hsp!)!;
        return ref lc.ChangeElementaryLaw(w);
    }

    /// <summary>The maximum radius, from the radii set and the laws (sampled at six points each).</summary>
    public double MaxRadFromSeqAndLaws()
    {
        double MaxRad = 0.0;

        for (int i = 1; i <= parandrad.Length(); i++)
        {
            if (parandrad[i].Y() > MaxRad)
            {
                MaxRad = parandrad[i].Y();
            }
        }

        NCollection_List<Law_Function>.Iterator itl = new(laws);
        for (; itl.More(); itl.Next())
        {
            Law_Function law = itl.Value();
            double fpar, lpar, par, delta, rad;
            law.Bounds(out fpar, out lpar);
            delta = (lpar - fpar) * 0.2;
            for (int i = 0; i <= 4; i++)
            {
                par = fpar + i * delta;
                rad = law.Value(par);
                if (rad > MaxRad)
                {
                    MaxRad = rad;
                }
            }

            rad = law.Value(lpar);
            if (rad > MaxRad)
            {
                MaxRad = rad;
            }
        }

        return MaxRad;
    }
}
